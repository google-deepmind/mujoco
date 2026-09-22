// Copyright 2026 PickNik Inc.
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

// Resource provider for ROS-style package:// URIs.
//
// A resource named package://<package>/<path> is served from the OS filesystem
// by locating <package> the way ROS tools do, without depending on ROS:
//   1. AMENT_PREFIX_PATH (ROS 2 install spaces): <prefix>/share/<package>, when
//      the ament index marker
//      <prefix>/share/ament_index/resource_index/packages/<package> exists
//   2. ROS_PACKAGE_PATH (ROS 1 install spaces and plain source trees):
//      <dir>/<package>, when it contains a package.xml
// The resolved file always lies inside the package directory.

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::package_uri {
namespace {

namespace fs = std::filesystem;

constexpr std::string_view kScheme = "package://";

#ifdef _WIN32
constexpr char kPathListSeparator = ';';
#else
constexpr char kPathListSeparator = ':';
#endif

// split a PATH-style environment variable into its non-empty entries
std::vector<std::string> PathList(const char* name) {
  std::vector<std::string> entries;
  const char* value = std::getenv(name);
  if (!value) {
    return entries;
  }

  std::string_view rest = value;
  while (!rest.empty()) {
    std::size_t pos = rest.find(kPathListSeparator);
    std::string_view entry = rest.substr(0, pos);
    if (!entry.empty()) {
      entries.emplace_back(entry);
    }
    if (pos == std::string_view::npos) {
      break;
    }
    rest.remove_prefix(pos + 1);
  }
  return entries;
}

// directory of a package, or an empty path if it is not found
fs::path FindPackage(const std::string& package) {
  std::error_code ec;

  // ROS 2 install spaces: the ament index marks installed packages
  for (const std::string& prefix : PathList("AMENT_PREFIX_PATH")) {
    fs::path share = fs::path(prefix) / "share";
    if (fs::exists(
            share / "ament_index" / "resource_index" / "packages" / package,
            ec)) {
      return share / package;
    }
  }

  // ROS 1 install spaces and source trees: a package is a directory with a
  // package.xml
  for (const std::string& dir : PathList("ROS_PACKAGE_PATH")) {
    fs::path candidate = fs::path(dir) / package;
    if (fs::exists(candidate / "package.xml", ec)) {
      return candidate;
    }
  }

  return {};
}

// resolve package://<package>/<path> to an OS path, or an empty string if it
// cannot be resolved
std::string Resolve(std::string_view name) {
  // MuJoCo matches schemes case-insensitively, so accept any casing of the
  // scheme
  if (name.size() <= kScheme.size()) {
    return {};
  }
  for (std::size_t i = 0; i < kScheme.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(name[i])) != kScheme[i]) {
      return {};
    }
  }
  name.remove_prefix(kScheme.size());

  // split into package and path, both non-empty
  std::size_t slash = name.find('/');
  if (slash == 0 || slash == std::string_view::npos ||
      slash + 1 == name.size()) {
    return {};
  }
  std::string package(name.substr(0, slash));
  fs::path relative(name.substr(slash + 1));

  // keep the file inside the package: the package must be a plain directory
  // name, and the path must not be rooted or climb out with ".."
  if (package == "." || package == ".." ||
      package.find_first_of(":\\") != std::string::npos) {
    return {};
  }
  if (relative.has_root_path()) {
    return {};
  }
  for (const fs::path& part : relative) {
    if (part == "..") {
      return {};
    }
  }

  fs::path package_dir = FindPackage(package);
  if (package_dir.empty()) {
    return {};
  }
  return (package_dir / relative).string();
}

// read the whole file into resource->data
int Open(mjResource* resource) {
  std::string path = Resolve(resource->name);
  if (path.empty()) {
    return 0;
  }

  std::error_code ec;
  std::uintmax_t size = fs::file_size(path, ec);
  if (ec || !fs::is_regular_file(path, ec) ||
      size > static_cast<std::uintmax_t>(std::numeric_limits<int>::max())) {
    return 0;
  }

  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return 0;
  }

  auto* bytes = new std::string(size, '\0');
  file.read(bytes->data(), bytes->size());
  if (static_cast<std::uintmax_t>(file.gcount()) != size) {
    delete bytes;
    return 0;
  }

  resource->data = bytes;
  return 1;
}

int Read(mjResource* resource, const void** buffer) {
  const auto* bytes = static_cast<const std::string*>(resource->data);
  *buffer = bytes->data();
  return bytes->size();
}

void Close(mjResource* resource) {
  delete static_cast<std::string*>(resource->data);
  resource->data = nullptr;
}

}  // namespace

mjPLUGIN_LIB_INIT(package_uri) {
  mjpResourceProvider provider;
  mjp_defaultResourceProvider(&provider);
  provider.prefix = "package";
  provider.open = Open;
  provider.read = Read;
  provider.close = Close;
  mjp_registerResourceProvider(&provider);
}

}  // namespace mujoco::plugin::package_uri
