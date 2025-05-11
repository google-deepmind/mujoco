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

#ifndef MUJOCO_TEST_EXPERIMENTAL_USD_PLUGINS_MJCF_FIXTURE_H_
#define MUJOCO_TEST_EXPERIMENTAL_USD_PLUGINS_MJCF_FIXTURE_H_

#include <string>

#include <gtest/gtest.h>
#include "test/fixture.h"
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/stage.h>

#define EXPECT_PRIM_VALID(stage, path) \
  EXPECT_TRUE((stage)->GetPrimAtPath(SdfPath(path)).IsValid());

#define EXPECT_PRIM_KIND(stage, path, kind)                          \
  {                                                                  \
    pxr::TfToken prim_kind;                                          \
    pxr::UsdModelAPI::Get(stage, SdfPath(path)).GetKind(&prim_kind); \
    EXPECT_EQ(kind, prim_kind);                                      \
  }
namespace mujoco {

pxr::SdfLayerRefPtr LoadLayer(const std::string& xml);

template <typename T>
void ExpectAttributeEqual(pxr::UsdStageRefPtr stage, const char* path,
                          const T& value) {
  auto attr = stage->GetAttributeAtPath(pxr::SdfPath(path));
  EXPECT_TRUE(attr.IsValid());
  T attr_value;
  attr.Get(&attr_value);
  EXPECT_EQ(attr_value, value);
}

// Specialization for SdfAssetPath, so that we can compare only the asset path
// and not care about whatever the resolved path is.
// Otherwise the default operator== would fail because it tests for equality of
// the asset path AND the resolved path.
template <>
void ExpectAttributeEqual<pxr::SdfAssetPath>(pxr::UsdStageRefPtr stage,
                                             const char* path,
                                             const pxr::SdfAssetPath& value);

void ExpectAttributeHasConnection(pxr::UsdStageRefPtr stage, const char* path,
                                  const char* connection_path);

using MjcfSdfFileFormatPluginTest = MujocoTest;

}  // namespace mujoco
#endif  // MUJOCO_TEST_EXPERIMENTAL_USD_PLUGINS_MJCF_FIXTURE_H_
