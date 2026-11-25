// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Resource provider for the mjdb:// URI scheme.
//
// mjdb is a URI interface and directory-layout convention for managing and referencing versioned
// MuJoCo assets. This provider implements the interface by resolving URIs against a local
// directory on disk.
//
// The URI mjdb://{kind}/{origin}/{uid}?version={version} resolves to the file
// {root}/{kind}/{origin}/{uid}/{version}, where {root} is the value of the MJDB_ROOT_DIR
// environment variable. A URI without a version resolves to {root}/{kind}/{origin}/{uid}/active.
// The files have no extension: each is either an MJZ archive or an MJCF file, told apart by its
// leading bytes.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "user/user_util.h"

namespace {

constexpr std::string_view kScheme          = "mjdb://";
constexpr std::string_view kDefaultVersion  = "active";
constexpr char             kRootDirEnvVar[] = "MJDB_ROOT_DIR";

bool ConsumePrefix(std::string_view& str, std::string_view prefix) {
  if (!str.starts_with(prefix)) { return false; }
  str.remove_prefix(prefix.size());
  return true;
}

void ConsumeSuffix(std::string_view& str, std::string_view suffix) {
  if (str.ends_with(suffix)) { str.remove_suffix(suffix.size()); }
}

// Returns true if name can be one component of a file path below the root directory.
bool IsPathComponent(std::string_view name) {
  return !name.empty() &&
         name != "." &&
         name != ".." &&
         name.find_first_of("/\\") == std::string_view::npos;
}

// Returns the file path that an mjdb:// URI resolves to under root, or nullopt if the URI is
// malformed. A URI whose components could name a file outside of root is malformed.
std::optional<std::string> ResolveUri(std::string_view uri, std::string_view root) {
  if (!ConsumePrefix(uri, kScheme)) { return std::nullopt; }

  // The extension that Open appends to a resource's name is not part of its URI.
  ConsumeSuffix(uri, ".mjz");
  ConsumeSuffix(uri, ".xml");

  std::string_view  query;
  const std::size_t query_pos = uri.find('?');
  if (query_pos != std::string_view::npos) {
    query = uri.substr(query_pos + 1);
    uri   = uri.substr(0, query_pos);
    ConsumeSuffix(uri, ".mjz");
    ConsumeSuffix(uri, ".xml");
  }

  // {kind}/{origin}/{uid}, where the uid can have several components.
  const std::size_t kind_end = uri.find('/');
  if (kind_end == std::string_view::npos) { return std::nullopt; }
  const std::size_t origin_end = uri.find('/', kind_end + 1);
  if (origin_end == std::string_view::npos) { return std::nullopt; }
  const std::string_view kind   = uri.substr(0, kind_end);
  const std::string_view origin = uri.substr(kind_end + 1, origin_end - kind_end - 1);
  const std::string_view uid    = uri.substr(origin_end + 1);

  // The first version parameter of the query, if any.
  std::string_view version = kDefaultVersion;
  while (!query.empty()) {
    std::string_view param = query.substr(0, query.find('&'));
    query.remove_prefix(std::min(param.size() + 1, query.size()));
    if (ConsumePrefix(param, "version=")) {
      version = param;
      break;
    }
  }

  if (!IsPathComponent(kind) || !IsPathComponent(origin) || !IsPathComponent(version)) {
    return std::nullopt;
  }
  for (std::string_view components = uid;;) {
    const std::size_t component_end = components.find('/');
    if (!IsPathComponent(components.substr(0, component_end))) { return std::nullopt; }
    if (component_end == std::string_view::npos) { break; }
    components.remove_prefix(component_end + 1);
  }

  std::string path(root);
  for (std::string_view component : {kind, origin, uid, version}) {
    path += '/';
    path += component;
  }
  return path;
}

// Returns the extension of the file format that content starts with, or an empty string if
// the format is not recognized.
std::string_view ExtensionFromContent(std::string_view content) {
  // Zip local file header, empty archive and spanned archive signatures.
  for (std::string_view signature : {"PK\x03\x04", "PK\x05\x06", "PK\x07\x08"}) {
    if (content.starts_with(signature)) { return ".mjz"; }
  }

  // XML, after an optional UTF-8 byte order mark and whitespace.
  ConsumePrefix(content, "\xEF\xBB\xBF");
  const std::size_t first = content.find_first_not_of(" \t\n\v\f\r");
  if (first != std::string_view::npos && content[first] == '<') { return ".xml"; }

  return "";
}

// Returns the directory that mjdb:// URIs resolve under, or an empty string if there is none.
std::string RootDir() {
  const char* env  = std::getenv(kRootDirEnvVar);
  std::string root = env ? env : "";
  while (root.size() > 1 && (root.back() == '/' || root.back() == '\\')) { root.pop_back(); }
  return root;
}

// Reads the local file at path into contents. Returns false if it is not a readable, non-empty
// file.
bool ReadFile(const std::string& path, std::vector<uint8_t>& contents) {
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec)) { return false; }
  contents = mujoco::user::FileToMemory(path.c_str());
  return !contents.empty();
}

int Open(mjResource* resource) {
  if (!resource || !resource->name) { return 0; }

  const std::string root = RootDir();
  if (root.empty()) {
    mju_warning("mjdb: cannot open '%s' because %s is not set", resource->name, kRootDirEnvVar);
    return 0;
  }

  const std::optional<std::string> path = ResolveUri(resource->name, root);
  if (!path) {
    mju_warning("mjdb: invalid URI '%s'", resource->name);
    return 0;
  }

  auto contents = std::make_unique<std::vector<uint8_t>>();
  if (!ReadFile(*path, *contents)) {
    mju_warning("mjdb: cannot read '%s', which '%s' resolves to", path->c_str(), resource->name);
    return 0;
  }

  const std::string_view extension = ExtensionFromContent(
      std::string_view(reinterpret_cast<const char*>(contents->data()), contents->size()));
  if (extension.empty()) {
    mju_warning("mjdb: '%s' is neither an MJZ archive nor an XML file", path->c_str());
    return 0;
  }

  // Decoders identify a resource's format by the extension of its name, so append it.
  const std::string_view name = resource->name;
  if (!name.ends_with(extension)) {
    char* qualified_name = new char[name.size() + extension.size() + 1];
    std::memcpy(qualified_name, name.data(), name.size());
    std::memcpy(qualified_name + name.size(), extension.data(), extension.size());
    qualified_name[name.size() + extension.size()] = '\0';
    delete[] resource->name;
    resource->name = qualified_name;
  }

  resource->data = contents.release();
  return 1;
}

int Read(mjResource* resource, const void** buffer) {
  if (!resource || !resource->data || !buffer) { return -1; }
  const auto* contents = static_cast<const std::vector<uint8_t>*>(resource->data);
  *buffer              = contents->data();
  return static_cast<int>(contents->size());
}

void Close(mjResource* resource) {
  if (!resource || !resource->data) { return; }
  delete static_cast<std::vector<uint8_t>*>(resource->data);
  resource->data = nullptr;
}

}  // namespace

mjPLUGIN_LIB_INIT(mjdb_provider) {
  mjpResourceProvider provider;
  mjp_defaultProvider(&provider);
  provider.prefix = "mjdb";
  provider.open   = Open;
  provider.read   = Read;
  provider.close  = Close;
  mjp_registerProvider(&provider);
}
