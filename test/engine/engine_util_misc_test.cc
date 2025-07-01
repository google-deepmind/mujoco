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

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

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
using ::testing::Pointwise;
using ::testing::StrEq;
using ::testing::ElementsAreArray;

using UtilMiscTest = MujocoTest;

TEST_F(UtilMiscTest, PrintsMemoryWarning) {
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

TEST_F(UtilMiscTest, Sigmoid) {
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

TEST_F(UtilMiscTest, SphereWrap) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <site size=".015" rgba="1 0 0 1"/>
    </default>

    <worldbody>
      <light pos="0 0 3"/>

      <site name="fixed" pos="0 0 1"/>
      <geom name="sphere" size=".1" pos="0 0 0.5"/>
      <site name="sidesite" pos=".52 0 .5"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom size=".05"/>
        <site name="body" pos="0 0 .05"/>
      </body>
    </worldbody>

    <tendon>
      <spatial name="tendon" range="0 0.8">
        <site site="fixed"/>
        <geom geom="sphere" sidesite="sidesite"/>
        <site site="body"/>
      </spatial>
    </tendon>

    <sensor>
      <tendonpos tendon="tendon"/>
    </sensor>

    <keyframe>
      <key qpos="-0.00653537 -0.068031 0.301253 0.982186 -0.180204 -0.0273515 0.0457068"/>
      <key qpos="-0.00653537 -0.069 0.301253 0.982186 -0.180204 -0.0273515 0.0457068"/>
    </keyframe>
  </mujoco>
  )";

  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // measure tendon length for keyframe 0
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);
  mjtNum ten_length0 = data->sensordata[0];

  // measure tendon length for keyframe 1
  mj_resetDataKeyframe(model, data, 1);
  mj_forward(model, data);
  mjtNum ten_length1 = data->sensordata[0];

  // difference should be small
  mjtNum diff = ten_length1 - ten_length0;
  EXPECT_LT(mju_abs(diff), 1e-3);

  mj_deleteData(data);
  mj_deleteModel(model);
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

TEST_F(UtilMiscTest, SmoothMuscleDynamics) {
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

TEST_F(UtilMiscTest, MuscleGainLength) {
  mjtNum lmin = 0.5;
  mjtNum lmax = 1.5;

  EXPECT_EQ(mju_muscleGainLength(0.0,  lmin, lmax), 0);
  EXPECT_EQ(mju_muscleGainLength(0.5,  lmin, lmax), 0);
  EXPECT_EQ(mju_muscleGainLength(0.75, lmin, lmax), 0.5);
  EXPECT_EQ(mju_muscleGainLength(1.0,  lmin, lmax), 1);
  EXPECT_EQ(mju_muscleGainLength(1.25, lmin, lmax), 0.5);
  EXPECT_EQ(mju_muscleGainLength(1.5,  lmin, lmax), 0);
  EXPECT_EQ(mju_muscleGainLength(2.0,  lmin, lmax), 0);
}

// --------------------------------- Interpolation -----------------------------

using InterpolationTest = MujocoTest;

TEST_F(InterpolationTest, mju_defGradient) {
  int order = 1;
  mjtNum mat[9];
  mjtNum p1[3] = {.5, .5, .5};
  mjtNum p2[3] = {.25, .25, .25};
  mjtNum dof0[24] = {0, 0, 0,  0, 0, 1,  0, 1, 0,  0, 1, 1,
                     1, 0, 0,  1, 0, 1,  1, 1, 0,  1, 1, 1};

  // identity
  mjtNum dof1[24];
  for (int i = 0; i < 24; ++i) dof1[i] = dof0[i];
  mju_defGradient(mat, p1, dof1, order);
  EXPECT_THAT(mat, ElementsAreArray({1, 0, 0, 0, 1, 0, 0, 0, 1}));

  // translation
  mjtNum dof2[24];
  for (int i = 0; i < 24; ++i) dof2[i] = 2 + dof0[i];
  mju_defGradient(mat, p1, dof2, order);
  EXPECT_THAT(mat, ElementsAreArray({1, 0, 0, 0, 1, 0, 0, 0, 1}));
  mju_defGradient(mat, p2, dof2, order);
  EXPECT_THAT(mat, ElementsAreArray({1, 0, 0, 0, 1, 0, 0, 0, 1}));

  // constant stretch
  mjtNum dof3[24];
  for (int i = 0; i < 24; ++i) dof3[i] = 2*dof0[i];
  mju_defGradient(mat, p1, dof3, order);
  EXPECT_THAT(mat, ElementsAreArray({2, 0, 0, 0, 2, 0, 0, 0, 2}));
  mju_defGradient(mat, p2, dof3, order);
  EXPECT_THAT(mat, ElementsAreArray({2, 0, 0, 0, 2, 0, 0, 0, 2}));

  // axial stretch
  mjtNum dof4[24];
  for (int i = 0; i < 24; ++i) dof4[i] = (i%3 == 1 ? 2 : 1)*dof0[i];
  mju_defGradient(mat, p1, dof4, order);
  EXPECT_THAT(mat, ElementsAreArray({1, 0, 0, 0, 2, 0, 0, 0, 1}));
  mju_defGradient(mat, p2, dof4, order);
  EXPECT_THAT(mat, ElementsAreArray({1, 0, 0, 0, 2, 0, 0, 0, 1}));

  // z-axis 90 degree rotation
  mjtNum dof5[24];
  for (int i = 0; i < 8; ++i) {
    mjtNum quat[4] = {0, 0, 0, 1};
    mjtNum axis[3] = {0, 0, 1};
    mju_axisAngle2Quat(quat, axis, mjPI/2);
    mju_rotVecQuat(dof5 + 3*i, dof0 + 3*i, quat);
  }
  mju_defGradient(mat, p1, dof5, order);
  EXPECT_THAT(mat, Pointwise(DoubleNear(1e-8), {0, -1, 0, 1, 0, 0, 0, 0, 1}));
  mju_defGradient(mat, p2, dof5, order);
  EXPECT_THAT(mat, Pointwise(DoubleNear(1e-8), {0, -1, 0, 1, 0, 0, 0, 0, 1}));

  // z-axis 30 degree rotation
  mjtNum dof6[24];
  mjtNum rot6[9];
  for (int i = 0; i < 8; ++i) {
    mjtNum quat[4];
    mjtNum axis[3] = {0, 0, 1};
    mju_axisAngle2Quat(quat, axis, mjPI/6);
    mju_rotVecQuat(dof6 + 3*i, dof0 + 3*i, quat);
    mju_quat2Mat(rot6, quat);
  }
  mju_defGradient(mat, p1, dof6, order);
  EXPECT_THAT(mat, Pointwise(DoubleNear(1e-8), rot6));
  mju_defGradient(mat, p2, dof6, order);
  EXPECT_THAT(mat, Pointwise(DoubleNear(1e-8), rot6));

  // z-axis CoM rotation
  mjtNum dof7[24];
  mjtNum rot7[9];
  for (int i = 0; i < 8; ++i) {
    mjtNum quat[4];
    mjtNum axis[3] = {0, 0, 1};
    mjtNum offset[3] = {-.5, -.5, 0};
    mju_axisAngle2Quat(quat, axis, mjPI/6);
    mju_add3(dof7 + 3*i, dof0 + 3*i, offset);
    mju_rotVecQuat(dof7 + 3*i, dof0 + 3*i, quat);
    mju_quat2Mat(rot7, quat);
  }
  mju_defGradient(mat, p1, dof7, order);
  EXPECT_THAT(mat, Pointwise(DoubleNear(1e-8), rot7));
  mju_defGradient(mat, p2, dof7, order);
  EXPECT_THAT(mat, Pointwise(DoubleNear(1e-8), rot7));
}

// --------------------------------- Base64 ------------------------------------

using Base64Test = MujocoTest;

TEST_F(Base64Test, mju_encodeBase64) {
  std::array<char, 9> buffer;
  std::array<std::uint8_t, 5> arr = {15, 134, 190, 255, 240};

  std::size_t n = mju_encodeBase64(buffer.data(), arr.data(), arr.size());

  EXPECT_THAT(buffer.data(), StrEq("D4a+//A="));
  EXPECT_THAT(n, std::strlen(buffer.data()) + 1);
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_encodeBase64_align0) {
  std::array<char, 5> buffer;
  std::array<std::uint8_t, 3> arr = {'A', 'B', 'C'};

  std::size_t n = mju_encodeBase64(buffer.data(), arr.data(), arr.size());

  EXPECT_THAT(buffer.data(), StrEq("QUJD"));
  EXPECT_THAT(n, std::strlen(buffer.data()) + 1);
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_encodeBase64_align1) {
  std::array<char, 5> buffer;
  std::array<std::uint8_t, 2> arr = {'A', 'B'};

  std::size_t n = mju_encodeBase64(buffer.data(), arr.data(), arr.size());

  EXPECT_THAT(buffer.data(), StrEq("QUI="));
  EXPECT_THAT(n, std::strlen(buffer.data()) + 1);
  EXPECT_THAT(n, buffer.size());

}

TEST_F(Base64Test, mju_encodeBase64_align2) {
  std::array<char, 5> buffer;
  std::array<std::uint8_t, 1> arr = {'A'};

  std::size_t n = mju_encodeBase64(buffer.data(), arr.data(), arr.size());

  EXPECT_THAT(buffer.data(), StrEq("QQ=="));
  EXPECT_THAT(n, std::strlen(buffer.data()) + 1);
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_encodeBase64_null) {
  std::array<char, 1> buffer;

  std::size_t n = mju_encodeBase64(buffer.data(), NULL, 0);

  EXPECT_THAT(n, 1);
  EXPECT_THAT(buffer[0], '\0');
}

TEST_F(Base64Test, mju_encodeBase64_ones) {
  std::array<char, 5> buffer;
  std::array<std::uint8_t, 3> arr = {255, 255, 255};

  std::size_t n = mju_encodeBase64(buffer.data(), arr.data(), arr.size());

  EXPECT_THAT(buffer.data(), StrEq("////"));
  EXPECT_THAT(n, std::strlen(buffer.data()) + 1);
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_isValidBase64_emptyStr) {
  std::size_t n = mju_isValidBase64("");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_invalid1) {
  std::size_t n = mju_isValidBase64("A");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_invalid2) {
  std::size_t n = mju_isValidBase64("AAA");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_invalid3) {
  std::size_t n = mju_isValidBase64("A==A");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_invalid5) {
  std::size_t n = mju_isValidBase64("A===");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_invalid6) {
  std::size_t n = mju_isValidBase64("aaaa====");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_invalid7) {
  std::size_t n = mju_isValidBase64("A#AA");

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_isValidBase64_valid1) {
  std::size_t n = mju_isValidBase64("AB+/");

  EXPECT_THAT(n, 3);
}

TEST_F(Base64Test, mju_isValidBase64_valid2) {
  std::size_t n = mju_isValidBase64("ABC=");

  EXPECT_THAT(n, 2);
}

TEST_F(Base64Test, mju_isValidBase64_valid3) {
  std::size_t n = mju_isValidBase64("AB==");

  EXPECT_THAT(n, 1);
}

TEST_F(Base64Test, mju_isValidBase64_valid4) {
  std::size_t n = mju_isValidBase64("az09AZ+/11==");

  EXPECT_THAT(n, 7);
}

TEST_F(Base64Test, mju_decodeBase64) {
  std::array<std::uint8_t, 5> buffer;
  const char *s = "D4a+//A=";

  std::size_t n = mju_decodeBase64(buffer.data(), s);

  EXPECT_THAT(buffer, ElementsAreArray({15, 134, 190, 255, 240}));
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_decodeBase6_align0) {
  std::array<std::uint8_t, 3> buffer;
  const char *s = "QUJD";

  std::size_t n = mju_decodeBase64(buffer.data(), s);

  EXPECT_THAT(buffer, ElementsAreArray({'A', 'B', 'C'}));
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_decodeBase64_align1) {
  std::array<std::uint8_t, 2> buffer;
  const char *s = "QUI=";

  std::size_t n = mju_decodeBase64(buffer.data(), s);

  EXPECT_THAT(buffer, ElementsAreArray({'A', 'B'}));
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_decodeBase64_align2) {
  std::array<std::uint8_t, 1> buffer;
  const char *s = "QQ==";

  std::size_t n = mju_decodeBase64(buffer.data(), s);

  EXPECT_THAT(buffer, ElementsAreArray({'A'}));
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, mju_decodeBase64_null) {
  const char *s = "";

  std::size_t n = mju_decodeBase64(NULL, s);

  EXPECT_THAT(n, 0);
}

TEST_F(Base64Test, mju_decodeBase64_ones) {
  std::array<std::uint8_t, 3> buffer;
  const char *s = "////";

  std::size_t n = mju_decodeBase64(buffer.data(), s);

  EXPECT_THAT(buffer, ElementsAreArray({255, 255, 255}));
  EXPECT_THAT(n, buffer.size());
}

TEST_F(Base64Test, decodeAndEncode) {
  std::array<std::uint8_t, 5> buffer1;
  std::array<char, 9> buffer2;
  const char *s = "D4a+/vA=";

  mju_decodeBase64(buffer1.data(), s);
  mju_encodeBase64(buffer2.data(), buffer1.data(), buffer1.size());

  EXPECT_THAT(buffer2.data(), StrEq(s));
}

}  // namespace
}  // namespace mujoco
