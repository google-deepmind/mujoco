// Copyright 2025 DeepMind Technologies Limited
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

#include "mjcf/mjcf_file_format.h"

#include <array>
#include <memory>
#include <string>
#include <unordered_set>

#include <mujoco/mujoco.h>
#include "mjcf/mujoco_to_usd.h"
#include "third_party/tinyxml2/tinyxml2.h"
#include <pxr/base/tf/diagnostic.h>
#include <pxr/base/tf/enum.h>
#include <pxr/base/tf/pathUtils.h>
#include <pxr/base/tf/registryManager.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/stringUtils.h>
#include <pxr/base/tf/type.h>
#include <pxr/base/work/loops.h>
#include <pxr/pxr.h>
#include <pxr/usd/ar/asset.h>
#include <pxr/usd/ar/resolvedPath.h>
#include <pxr/usd/ar/resolver.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/usdaFileFormat.h>

PXR_NAMESPACE_OPEN_SCOPE

TF_DEFINE_PUBLIC_TOKENS(UsdMjcfFileFormatTokens, USD_MJCF_FILE_FORMAT_TOKENS);

TF_REGISTRY_FUNCTION(TfType) {
  SDF_DEFINE_FILE_FORMAT(UsdMjcfFileFormat, SdfFileFormat);
}

enum ErrorCodes { XmlParsingError };
TF_REGISTRY_FUNCTION(TfEnum) {
  TF_ADD_ENUM_NAME(XmlParsingError, "Error when parsing XML.");
};

namespace {

void ResolveMjcfDependencies(const std::string &xml_string,
                             const std::string &resolved_path);

void AccumulateFilesRecursive(std::unordered_set<std::string> &files,
                              tinyxml2::XMLElement *elem,
                              const std::string &resolved_path) {
  // get filename
  const char *file = elem->Attribute("file");

  if (file != nullptr) {
    auto identifier = pxr::ArGetResolver().CreateIdentifier(
        std::string(file), pxr::ArResolvedPath(resolved_path));
    if (!strcasecmp(elem->Value(), "include") ||
        !strcasecmp(elem->Value(), "model")) {
      auto include_resolved_path = pxr::ArGetResolver().Resolve(identifier);
      auto asset = pxr::ArGetResolver().OpenAsset(include_resolved_path);
      ResolveMjcfDependencies(asset->GetBuffer().get(), include_resolved_path);

      // Neither of these elements should have children.
      return;
    }

    files.insert(identifier);
  }

  if (!strcasecmp(elem->Value(), "texture")) {
    static const char *attributes[] = {"fileright", "fileup",    "fileleft",
                                       "filedown",  "filefront", "fileback"};
    for (const auto &attribute : attributes) {
      const char *file = elem->Attribute(attribute);
      if (file != nullptr) {
        auto identifier = pxr::ArGetResolver().CreateIdentifier(
            std::string(file), pxr::ArResolvedPath(resolved_path));
        files.insert(identifier);
      }
    }
  }

  tinyxml2::XMLElement *child = elem->FirstChildElement();
  for (; child; child = child->NextSiblingElement()) {
    AccumulateFilesRecursive(files, child, resolved_path);
  }
}

void ResolveMjcfDependencies(const std::string &xml_string,
                             const std::string &resolved_path) {
  // load XML file or parse string
  tinyxml2::XMLDocument doc;
  doc.Parse(xml_string.c_str());

  // error checking
  if (doc.Error()) {
    TF_ERROR(XmlParsingError, "%d:\n%s\n", doc.ErrorID(), doc.ErrorStr());
    return;
  }

  // get top-level element
  tinyxml2::XMLElement *root = doc.RootElement();
  if (!root) {
    TF_ERROR(XmlParsingError, "XML root element not found");
    return;
  }

  // Accumulate file dependencies.
  std::unordered_set<std::string> files = {};
  AccumulateFilesRecursive(files, root, resolved_path);

  auto open_asset = [resolved_path](const std::string &identifier) {
    pxr::ArGetResolver().OpenAsset(pxr::ArGetResolver().Resolve(identifier));
  };
  // Open all assets in parallel.
  pxr::WorkParallelForEach(files.begin(), files.end(), open_asset);
}
}  // namespace

UsdMjcfFileFormat::UsdMjcfFileFormat()
    : SdfFileFormat(
          UsdMjcfFileFormatTokens->Id, UsdMjcfFileFormatTokens->Version,
          UsdMjcfFileFormatTokens->Target, UsdMjcfFileFormatTokens->Id) {}

UsdMjcfFileFormat::~UsdMjcfFileFormat() {}

bool UsdMjcfFileFormat::CanRead(const std::string &filePath) const {
  auto extension = pxr::TfGetExtension(filePath);
  if (extension.empty()) {
    return false;
  }

  return extension == this->GetFormatId();
}

bool UsdMjcfFileFormat::ReadImpl(pxr::SdfLayer *layer, mjSpec *spec) const {
  auto args = layer->GetFileFormatArguments();

  bool toggleUsdPhysics = false;
  const auto it =
      args.find(UsdMjcfFileFormatTokens->ToggleUsdPhysicsArg.GetString());
  if (it != args.end()) {
    toggleUsdPhysics = pxr::TfUnstringify<bool>(it->second);
  }

  auto data = InitData(args);

  auto success = mujoco::usd::WriteSpecToData(spec, data, toggleUsdPhysics);
  mj_deleteSpec(spec);
  if (!success) {
    return false;
  }

  _SetLayerData(layer, data);

  return true;
}

bool UsdMjcfFileFormat::ReadFromString(pxr::SdfLayer *layer,
                                       const std::string &str) const {
  std::array<char, 1024> error;
  mjSpec *spec =
      mj_parseXMLString(str.c_str(), nullptr, error.data(), error.size());
  if (spec == nullptr) {
    TF_WARN(XmlParsingError, "%s", error.data());
    return false;
  }

  return ReadImpl(layer, spec);
}

bool UsdMjcfFileFormat::Read(pxr::SdfLayer *layer,
                             const std::string &resolved_path,
                             bool metadata_only) const {
  // Resolved all dependencies so that they are accessible when parsing
  // the XML.
  std::shared_ptr<pxr::ArAsset> asset =
      pxr::ArGetResolver().OpenAsset(pxr::ArResolvedPath(resolved_path));
  auto buffer = asset->GetBuffer();
  ResolveMjcfDependencies(buffer.get(), resolved_path);

  // Parse to USD.
  std::array<char, 1024> error;
  mjSpec *spec =
      mj_parseXML(resolved_path.c_str(), nullptr, error.data(), error.size());
  if (spec == nullptr) {
    TF_WARN(XmlParsingError, "%s", error.data());
    return false;
  }
  return ReadImpl(layer, spec);
}

bool UsdMjcfFileFormat::WriteToString(const SdfLayer &layer, std::string *str,
                                      const std::string &comment) const {
  return SdfFileFormat::FindById(pxr::UsdUsdaFileFormatTokens->Id)
      ->WriteToString(layer, str, comment);
}

PXR_NAMESPACE_CLOSE_SCOPE
