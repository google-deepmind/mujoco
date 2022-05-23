// Copyright 2022 DeepMind Technologies Limited
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

// Tests for engine/engine_util_spatial.c

#include "src/engine/engine_util_spatial.h"

#include <cmath>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjtnum.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;

using Quat2MatTest = MujocoTest;

std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

TEST_F(Quat2MatTest, NoRotation) {
  mjtNum result[9] = {0};
  mjtNum quat[] = {1, 0, 0, 0};
  mju_quat2Mat(result, quat);
  EXPECT_THAT(
      AsVector(result, 9),
      ElementsAre(1, 0, 0,
                  0, 1, 0,
                  0, 0, 1)
  );
}

TEST_F(Quat2MatTest, TinyRotation) {
  mjtNum result[9] = {0};
  // An angle so small that cos(angle) == 1.0 to double accuracy
  mjtNum angle = 1e-8;
  mjtNum quat[] = {cos(angle/2), sin(angle/2), 0, 0};
  mju_quat2Mat(result, quat);
  EXPECT_THAT(
      AsVector(result, 9),
      ElementsAre(1, 0         ,  0         ,
                  0, cos(angle), -sin(angle),
                  0, sin(angle),  cos(angle))
  );
}

using MulQuatTest = MujocoTest;

TEST_F(MulQuatTest, TinyRotation) {
  mjtNum null_quat[4] = {1, 0, 0, 0};
  mjtNum result[4];
  // An angle so small that cos(angle) == 1.0 to double accuracy
  mjtNum angle = 1e-8;
  mjtNum quat[] = {cos(angle/2), sin(angle/2), 0, 0};
  mju_mulQuat(result, null_quat, quat);
  EXPECT_THAT(
      AsVector(result, 4),
      ElementsAre(cos(angle/2), sin(angle/2), 0, 0)
  );
}

using RotVecQuatTest = MujocoTest;

TEST_F(RotVecQuatTest, NoRotation) {
  mjtNum result[3];
  mjtNum vec[] = {1, 2, 3};
  mjtNum quat[] = {1, 0, 0, 0};
  mju_rotVecQuat(result, vec, quat);
  EXPECT_THAT(
      AsVector(result, 3),
      ElementsAre(1, 2, 3)
  );
}

TEST_F(RotVecQuatTest, TinyRotation) {
  mjtNum result[3];
  mjtNum vec[] = {0, 1, 0};
  // An angle so small that cos(angle) == 1.0 to double accuracy
  mjtNum angle = 1e-8;
  mjtNum quat[] = {cos(angle/2), sin(angle/2), 0, 0};
  mju_rotVecQuat(result, vec, quat);
  EXPECT_THAT(
      AsVector(result, 3),
      ElementsAre(0, cos(angle), sin(angle))
  );
}

}  // namespace
}  // namespace mujoco
