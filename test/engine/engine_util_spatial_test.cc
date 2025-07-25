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

#include <cmath>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_spatial.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::Pointwise;

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

// Rotate a vector by explicitly converting the quaternion to a 3x3 matrix
void RotVecQuatWithMatrix(mjtNum res[3], const mjtNum vec[3],
                          const mjtNum quat[4]) {
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    mju_copy3(res, vec);
  } else {
    mjtNum mat[9];
    mju_quat2Mat(mat, quat);
    mju_rotVecMat(res, vec, mat);
  }
}

TEST_F(RotVecQuatTest, TestEquivalence) {
  mjtNum resultActual[3], resultExpected[3], quat[4];
  // List of rotation axes
  mjtNum vecs[5][3] = {
      {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-0.5, 1, -0.5}, {1.22, -2.33, 3.44}};
  // List of angles to rotate by, in degrees
  mjtNum angles[6] = {0.0, 1e-8, 31, 47, 181, 271};
  static const mjtNum eps = 1e-15;
  for (auto vec : vecs) {
    // Unit-normalize the vector
    mju_normalize3(vec);
    for (auto angleDegree : angles) {
      // Convert the axis-angle to a quaternion
      auto angleRad = angleDegree * mjPI / 180;
      mju_axisAngle2Quat(quat, vec, angleRad);
      // Rotate
      mju_rotVecQuat(resultActual, vec, quat);
      RotVecQuatWithMatrix(resultExpected, vec, quat);
      // Compare
      EXPECT_NEAR(resultExpected[0], resultActual[0], eps);
      EXPECT_NEAR(resultExpected[1], resultActual[1], eps);
      EXPECT_NEAR(resultExpected[2], resultActual[2], eps);
    }
  }
}

using Euler2QuatTest = MujocoTest;

TEST_F(Euler2QuatTest, BadSeq) {
  EXPECT_FATAL_FAILURE({
        mjtNum quat[4];
        mjtNum euler[3] = {0};
        char seq[] = "xiz";
        mju_euler2Quat(quat, euler, seq);
  }, "mju_euler2Quat: seq[1] is 'i', should be one of x, y, z, X, Y, Z");
}

TEST_F(Euler2QuatTest, BadSeqLength) {
  EXPECT_FATAL_FAILURE({
        mjtNum quat[4];
        mjtNum euler[3] = {0};
        char seq[] = "xyzy";
        mju_euler2Quat(quat, euler, seq);
  }, "mju_euler2Quat: seq must contain exactly 3 characters");
}

TEST_F(Euler2QuatTest, Euler2Quat) {
  double quat[4] = {0};
  double tol = 1e-14;

  char seq[] = "xyz";
  double euler[3] = {mjPI, 0, 0};
  double expected[4] = {0, 1, 0, 0};
  mju_euler2Quat(quat, euler, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected));

  euler[1] = mjPI;
  double expected2[4] = {0, 0, 0, 1};
  mju_euler2Quat(quat, euler, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected2));

  char seq2[] = "XYZ";
  double expected3[4] = {0, 0, 0, -1};
  mju_euler2Quat(quat, euler, seq2);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected3));

  double euler2[3] = {2*mjPI, 2*mjPI, 2*mjPI};
  double expected4[4] = {-1, 0, 0, 0};
  mju_euler2Quat(quat, euler2, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected4));
  mju_euler2Quat(quat, euler2, seq2);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected4));

  double euler3[3] = {mjPI/2, mjPI/2, mjPI/2};
  double expected5[4] = {0, mju_sqrt(.5), 0, mju_sqrt(.5)};
  mju_euler2Quat(quat, euler3, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected5));
  mju_euler2Quat(quat, euler3, seq2);
  double expected6[4] = {mju_sqrt(.5), 0, mju_sqrt(.5), 0};
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected6));
}

}  // namespace
}  // namespace mujoco
