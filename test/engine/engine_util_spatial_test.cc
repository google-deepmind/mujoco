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
#include <cstdlib>

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
    mju_mulMatVec3(res, mat, vec);
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
  mjtNum quat[4] = {0};
  mjtNum tol = 1e-14;

  char seq[] = "xyz";
  mjtNum euler[3] = {mjPI, 0, 0};
  mjtNum expected[4] = {0, 1, 0, 0};
  mju_euler2Quat(quat, euler, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected));

  euler[1] = mjPI;
  mjtNum expected2[4] = {0, 0, 0, 1};
  mju_euler2Quat(quat, euler, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected2));

  char seq2[] = "XYZ";
  mjtNum expected3[4] = {0, 0, 0, -1};
  mju_euler2Quat(quat, euler, seq2);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected3));

  mjtNum euler2[3] = {2*mjPI, 2*mjPI, 2*mjPI};
  mjtNum expected4[4] = {-1, 0, 0, 0};
  mju_euler2Quat(quat, euler2, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected4));
  mju_euler2Quat(quat, euler2, seq2);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected4));

  mjtNum euler3[3] = {mjPI/2, mjPI/2, mjPI/2};
  mjtNum expected5[4] = {0, mju_sqrt(.5), 0, mju_sqrt(.5)};
  mju_euler2Quat(quat, euler3, seq);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected5));
  mju_euler2Quat(quat, euler3, seq2);
  mjtNum expected6[4] = {mju_sqrt(.5), 0, mju_sqrt(.5), 0};
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), expected6));
}

using Mat2RotTest = MujocoTest;

TEST_F(Mat2RotTest, RotationFromArbitraryMatrix) {
  // create arbitrary target rotation matrix
  mjtNum target[4], rot[9];
  mjtNum axis[3] = {1, 1, 1};
  mju_axisAngle2Quat(target, axis, mjPI/6);
  mju_normalize4(target);
  mju_quat2Mat(rot, target);

  // combine rotation with arbitrary stretch
  mjtNum mat[9];
  mjtNum deformation_gradient[9] = {0.5, 0.25, 0.125,
                                    0.3, 0.66, 0.999,
                                    0.4, 0.22, 0.111};
  mjtNum stretch[9];
  mju_mulMatTMat3(stretch, deformation_gradient, deformation_gradient);
  mju_mulMatMat3(mat, rot, stretch);

  // calculate rotational part of the matrix
  mjtNum quat[4] = {1, 0, 0, 0};
  int niter = mju_mat2Rot(quat, mat);
  EXPECT_THAT(quat, Pointwise(DoubleNear(1e-8), target));
  EXPECT_LE(niter, 150);
}

TEST_F(Mat2RotTest, IdentityFromRandomRotation) {
  // This test is based on the following paper:
  // Müller, Matthias, Jan Bender, Nuttapong Chentanez, and Miles Macklin. "A
  // robust method to extract the rotational part of deformations." In
  // Proceedings of the 9th International Conference on Motion in Games, pp.
  // 55-60. 2016.
  mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  srand(123);

  for (int i = 0; i < 100; ++i) {
    // random quaternion
    mjtNum quat[4];
    for (int j = 0; j < 4; ++j) {
      quat[j] = rand() / (float)RAND_MAX;  // NOLINT
    }

    // calculate rotational part of the matrix
    mjtNum res[9];
    mju_normalize4(quat);
    EXPECT_LE(mju_mat2Rot(quat, mat), 40);
    mju_quat2Mat(res, quat);
    EXPECT_THAT(res, Pointwise(DoubleNear(1e-6), mat));
  }
}

TEST_F(Mat2RotTest, SpecialCases) {
  mjtNum eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjtNum quat[4] = {1, 0, 0, 0};
  EXPECT_EQ(mju_mat2Rot(quat, eye), 0);
  EXPECT_THAT(quat, Pointwise(DoubleNear(1e-8), {1, 0, 0, 0}));
  mjtNum zero[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(mju_mat2Rot(quat, zero), 0);
  EXPECT_THAT(quat, Pointwise(DoubleNear(1e-8), {1, 0, 0, 0}));
  mjtNum ones[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  EXPECT_EQ(mju_mat2Rot(quat, ones), 0);
  EXPECT_THAT(quat, Pointwise(DoubleNear(1e-8), {1, 0, 0, 0}));
}

}  // namespace
}  // namespace mujoco
