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

#include "test/experimental/usd/plugins/mjcf/fixture.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/log/check.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
namespace mujoco {

using pxr::SdfPath;

pxr::SdfLayerRefPtr LoadLayer(
    const std::string& xml,
    const pxr::SdfFileFormat::FileFormatArguments& args) {
  auto layer = pxr::SdfLayer::CreateAnonymous(
      "test_layer", pxr::SdfFileFormat::FindByExtension("xml"), args);
  layer->ImportFromString(xml);
  EXPECT_THAT(layer, testing::NotNull());
  return layer;
}

template <>
void ExpectAttributeEqual<pxr::SdfAssetPath>(pxr::UsdStageRefPtr stage,
                                             const char* path,
                                             const pxr::SdfAssetPath& value) {
  auto attr = stage->GetAttributeAtPath(pxr::SdfPath(path));
  EXPECT_TRUE(attr.IsValid());
  pxr::SdfAssetPath attr_value;
  attr.Get(&attr_value);
  EXPECT_EQ(attr_value.GetAssetPath(), value.GetAssetPath());
}

void ExpectAttributeHasConnection(pxr::UsdStageRefPtr stage, const char* path,
                                  const char* connection_path) {
  auto attr = stage->GetAttributeAtPath(SdfPath(path));
  EXPECT_TRUE(attr.IsValid());
  pxr::SdfPathVector sources;
  attr.GetConnections(&sources);
  EXPECT_EQ(sources.size(), 1);
  EXPECT_EQ(sources[0], SdfPath(connection_path));
}
//
}  // namespace mujoco
