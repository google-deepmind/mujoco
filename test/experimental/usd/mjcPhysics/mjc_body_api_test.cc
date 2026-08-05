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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/experimental/usd/mjcPhysics/bodyAPI.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>

namespace mujoco {
namespace {

using pxr::SdfPath;
using MjcPhysicsBodyTest = MujocoTest;
using testing::NotNull;

TEST_F(MjcPhysicsBodyTest, TestBodyAPIDefaultsAndAttributes) {
  auto stage = pxr::UsdStage::CreateInMemory();
  auto body_xform = pxr::UsdGeomXform::Define(stage, SdfPath("/World/Body"));
  auto body_api = pxr::MjcPhysicsBodyAPI::Apply(body_xform.GetPrim());

  EXPECT_TRUE(body_xform.GetPrim().HasAPI<pxr::MjcPhysicsBodyAPI>());

  // Verify attribute defaults
  pxr::TfToken sleep_val;
  EXPECT_TRUE(body_api.GetSleepAttr().Get(&sleep_val));
  EXPECT_EQ(sleep_val, pxr::MjcPhysicsTokens->auto_);

  double gravcomp_val;
  EXPECT_TRUE(body_api.GetGravCompAttr().Get(&gravcomp_val));
  EXPECT_EQ(gravcomp_val, 0.0);

  // Verify setting custom values
  body_api.CreateSleepAttr().Set(pxr::MjcPhysicsTokens->allowed);
  body_api.CreateGravCompAttr().Set(1.5);

  EXPECT_TRUE(body_api.GetSleepAttr().Get(&sleep_val));
  EXPECT_EQ(sleep_val, pxr::MjcPhysicsTokens->allowed);

  EXPECT_TRUE(body_api.GetGravCompAttr().Get(&gravcomp_val));
  EXPECT_EQ(gravcomp_val, 1.5);
}

}  // namespace
}  // namespace mujoco
