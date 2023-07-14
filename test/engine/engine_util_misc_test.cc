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

// Tests for engine/engine_util_solve.c.

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::HasSubstr;
using ::testing::Ne;
using ::testing::StrEq;

TEST_F(MujocoTest, PrintsMemoryWarning) {
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 10)),
              HasSubstr("1K bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 20)),
              HasSubstr("1M bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 30)),
              HasSubstr("1G bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 40)),
              HasSubstr("1T bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 50)),
              HasSubstr("1P bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 60)),
              HasSubstr("1E bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 30) + 1),
              HasSubstr("1073741825 bytes"));
}

TEST_F(MujocoTest, Sigmoid) {
  // function values
  EXPECT_EQ(mju_sigmoid(-1),  0);
  EXPECT_EQ(mju_sigmoid(0),   0);
  EXPECT_EQ(mju_sigmoid(0.5), 0.5);
  EXPECT_EQ(mju_sigmoid(1),   1);
  EXPECT_EQ(mju_sigmoid(2),   1);

  // epsilon for finite-differencing
  const mjtNum dx = 1e-7;

  // derivative at 0
  mjtNum dy_dx_0 = (mju_sigmoid(0 + dx) - mju_sigmoid(0)) / dx;
  EXPECT_THAT(dy_dx_0, DoubleNear(0, dx));

  // derivative at 1
  mjtNum dy_dx_1 = (mju_sigmoid(1) - mju_sigmoid(1 - dx)) / dx;
  EXPECT_THAT(dy_dx_1, DoubleNear(0, dx));

  // derivative at 0.5
  const mjtNum x = 0.5;
  mjtNum dy_dx_0p5 = (mju_sigmoid(x + dx) - mju_sigmoid(x - dx)) / (2*dx);
  mjtNum expected = 30*x*x*x*x - 60*x*x*x + 30*x*x;
  EXPECT_THAT(dy_dx_0p5, DoubleNear(expected, dx));
}

// compute time constant as in Millard et al. (2013) https://doi.org/10.1115/1.4023390
mjtNum muscleDynamicsMillard(mjtNum ctrl, mjtNum act, const mjtNum prm[2]) {
  // clamp control
  mjtNum ctrlclamp = mju_clip(ctrl, 0, 1);

  // clamp activation
  mjtNum actclamp = mju_clip(act, 0, 1);

  mjtNum tau;
  if (ctrlclamp > act) {
    tau = prm[0] * (0.5 + 1.5*actclamp);
  } else {
    tau = prm[1] / (0.5 + 1.5*actclamp);
  }

  // filter output
  return (ctrlclamp-act) / mjMAX(mjMINVAL, tau);
}

TEST_F(MujocoTest, SmoothMuscleDynamics) {
  mjtNum prm[3] = {0.01, 0.04, 0.0};

  // exact equality if tau_smooth = 0
  for (mjtNum ctrl : {-0.1, 0.0, 0.4, 0.5, 1.0, 1.1}) {
    for (mjtNum act : {-0.1, 0.0, 0.4, 0.5, 1.0, 1.1}) {
      mjtNum actdot_old = muscleDynamicsMillard(ctrl, act, prm);
      mjtNum actdot_new = mju_muscleDynamics(ctrl, act, prm);
      EXPECT_EQ(actdot_new, actdot_old);
    }
  }

  // positive tau_smooth
  mjtNum tau_smooth = 0.2;
  prm[2] = tau_smooth;
  mjtNum act = 0.5;
  mjtNum eps = 1e-6;

  mjtNum ctrl = 0.4 - eps;  // smaller than act by just over 0.5*tau_smooth
  EXPECT_EQ(muscleDynamicsMillard(ctrl, act, prm),
            mju_muscleDynamics(ctrl, act, prm));

  ctrl = 0.6 + eps;         // larger than act by just over 0.5*tau_smooth
  EXPECT_EQ(muscleDynamicsMillard(ctrl, act, prm),
            mju_muscleDynamics(ctrl, act, prm));

  // right in the middle should give average of time constants
  mjtNum tau_act = 0.2;
  mjtNum tau_deact = 0.3;
  for (mjtNum dctrl : {0.0, 0.1, 0.2, 1.0, 1.1}) {
    mjtNum lower = mju_muscleDynamicsTimescale(-dctrl,
                                               tau_act, tau_deact, tau_smooth);
    mjtNum upper = mju_muscleDynamicsTimescale(dctrl,
                                               tau_act, tau_deact, tau_smooth);
    EXPECT_EQ(0.5*(upper + lower), 0.5*(tau_act + tau_deact));
  }
}

TEST_F(MujocoTest, mju_makefullname) {
  char buffer[1000];
  constexpr char path[] = "engine/testdata/";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  ASSERT_THAT(buffer, StrEq("engine/testdata/file"));
  EXPECT_THAT(n, 0);
}

TEST_F(MujocoTest, mju_makefullname2) {
  char buffer[1000];
  constexpr char path[] = "engine\\testdata\\";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  ASSERT_THAT(buffer, StrEq("engine\\testdata\\file"));
  EXPECT_THAT(n, 0);
}


TEST_F(MujocoTest, mju_makefullname_missingSlash) {
  char buffer[1000];
  constexpr char path[] = "engine/testdata";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  ASSERT_THAT(buffer, StrEq("engine/testdata/file"));
  EXPECT_THAT(n, 0);
}

TEST_F(MujocoTest, mju_makefullname_withoutDir) {
  char buffer[1000];
  constexpr char *path = NULL;
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  ASSERT_THAT(buffer, StrEq("file"));
  EXPECT_THAT(n, 0);
}

TEST_F(MujocoTest, mju_makefullname_withoutDir2) {
  char buffer[1000];
  constexpr char path[] = "";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  ASSERT_THAT(buffer, StrEq("file"));
  EXPECT_THAT(n, 0);
}

TEST_F(MujocoTest, mju_makefullname_error) {
  char buffer[1000];
  constexpr char path[] = "engine/testdata";
  constexpr char *file = NULL;
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  EXPECT_THAT(n, Ne(0));
}

TEST_F(MujocoTest, mju_makefullname_error2) {
  char buffer[1000];
  constexpr char path[] = "engine/testdata";
  constexpr char file[] = "";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  EXPECT_THAT(n, Ne(0));
}

TEST_F(MujocoTest, mju_makefullname_error3) {
  char buffer[20];
  constexpr char path[] = "engine/testdata/";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  EXPECT_THAT(n, Ne(0));
}

TEST_F(MujocoTest, mju_makefullname_error4) {
  char buffer[20];
  constexpr char path[] = "engine/testdata";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  EXPECT_THAT(n, Ne(0));
}

TEST_F(MujocoTest, mju_makefullname_error5) {
  char buffer[4];
  constexpr char path[] = "";
  constexpr char file[] = "file";
  int n = mju_makefullname(buffer, sizeof(buffer), path, file);
  EXPECT_THAT(n, Ne(0));
}

}  // namespace
}  // namespace mujoco
