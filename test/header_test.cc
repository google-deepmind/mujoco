// Copyright 2021 DeepMind Technologies Limited
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

// Tests for structures in the public headers.

#include <cstddef>
#include <cstring>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using HeaderTest = MujocoTest;

TEST_F(HeaderTest, IntsHave4Bytes) {
  EXPECT_EQ(4, sizeof(int));
}

TEST_F(HeaderTest, IntsHaveAtLeast31Bits) {
  int shift_left_30 = 1 << 30;
  EXPECT_GT(shift_left_30, 0);
}

TEST_F(HeaderTest, EnumsAreInts) {
  EXPECT_EQ(sizeof(mjtDisableBit),      sizeof(int));
  EXPECT_EQ(sizeof(mjtEnableBit),       sizeof(int));
  EXPECT_EQ(sizeof(mjtJoint),           sizeof(int));
  EXPECT_EQ(sizeof(mjtGeom),            sizeof(int));
  EXPECT_EQ(sizeof(mjtCamLight),        sizeof(int));
  EXPECT_EQ(sizeof(mjtTexture),         sizeof(int));
  EXPECT_EQ(sizeof(mjtIntegrator),      sizeof(int));
  EXPECT_EQ(sizeof(mjtCone),            sizeof(int));
  EXPECT_EQ(sizeof(mjtJacobian),        sizeof(int));
  EXPECT_EQ(sizeof(mjtSolver),          sizeof(int));
  EXPECT_EQ(sizeof(mjtEq),              sizeof(int));
  EXPECT_EQ(sizeof(mjtWrap),            sizeof(int));
  EXPECT_EQ(sizeof(mjtTrn),             sizeof(int));
  EXPECT_EQ(sizeof(mjtDyn),             sizeof(int));
  EXPECT_EQ(sizeof(mjtGain),            sizeof(int));
  EXPECT_EQ(sizeof(mjtBias),            sizeof(int));
  EXPECT_EQ(sizeof(mjtObj),             sizeof(int));
  EXPECT_EQ(sizeof(mjtConstraint),      sizeof(int));
  EXPECT_EQ(sizeof(mjtConstraintState), sizeof(int));
  EXPECT_EQ(sizeof(mjtSensor),          sizeof(int));
  EXPECT_EQ(sizeof(mjtStage),           sizeof(int));
  EXPECT_EQ(sizeof(mjtDataType),        sizeof(int));
  EXPECT_EQ(sizeof(mjtLRMode),          sizeof(int));
  EXPECT_EQ(sizeof(mjtWarning),         sizeof(int));
  EXPECT_EQ(sizeof(mjtTimer),           sizeof(int));
  EXPECT_EQ(sizeof(mjtGridPos),         sizeof(int));
  EXPECT_EQ(sizeof(mjtFramebuffer),     sizeof(int));
  EXPECT_EQ(sizeof(mjtFontScale),       sizeof(int));
  EXPECT_EQ(sizeof(mjtFont),            sizeof(int));
  EXPECT_EQ(sizeof(mjtButton),          sizeof(int));
  EXPECT_EQ(sizeof(mjtEvent),           sizeof(int));
  EXPECT_EQ(sizeof(mjtItem),            sizeof(int));
  EXPECT_EQ(sizeof(mjtCatBit),          sizeof(int));
  EXPECT_EQ(sizeof(mjtMouse),           sizeof(int));
  EXPECT_EQ(sizeof(mjtPertBit),         sizeof(int));
  EXPECT_EQ(sizeof(mjtCamera),          sizeof(int));
  EXPECT_EQ(sizeof(mjtLabel),           sizeof(int));
  EXPECT_EQ(sizeof(mjtFrame),           sizeof(int));
  EXPECT_EQ(sizeof(mjtVisFlag),         sizeof(int));
  EXPECT_EQ(sizeof(mjtRndFlag),         sizeof(int));
  EXPECT_EQ(sizeof(mjtStereo),          sizeof(int));
}

}  // namespace
}  // namespace mujoco
