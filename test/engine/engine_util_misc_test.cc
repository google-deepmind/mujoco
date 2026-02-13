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
#include <vector>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::ElementsAreArray;
using ::testing::HasSubstr;
using ::testing::NotNull;
using ::testing::Pointwise;
using ::testing::StrEq;

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

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
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

TEST_F(UtilMiscTest, MjuSparseMap) {
  // nr = 3
  // src = [[1, 2, 0],
  //        [0, 3, 4],
  //        [5, 0, 6]]
  constexpr int nr = 3;
  const mjtNum mat_src[] = {1, 2, 3, 4, 5, 6};
  const int rownnz_src[] = {2, 2, 2};
  const int rowadr_src[] = {0, 2, 4};
  const int colind_src[] = {0, 1, 1, 2, 0, 2};

  // res = [[1, 0, 0],
  //        [0, 3, 0],
  //        [5, 0, 6]]
  constexpr int nnz_res = 4;
  const int rownnz_res[] = {1, 1, 2};
  const int rowadr_res[] = {0, 1, 2};
  const int colind_res[] = {0, 1, 0, 2};

  int map[nnz_res];
  mju_sparseMap(map, nr, rowadr_res, rownnz_res, colind_res, rowadr_src,
                rownnz_src, colind_src);

  // Expected map:
  // res[0] (1 at 0,0) -> src[0] (1 at 0,0) => map[0] = 0
  // res[1] (3 at 1,1) -> src[2] (3 at 1,1) => map[1] = 2
  // res[2] (5 at 2,0) -> src[4] (5 at 2,0) => map[2] = 4
  // res[3] (6 at 2,2) -> src[5] (6 at 2,2) => map[3] = 5
  EXPECT_THAT(map, ElementsAre(0, 2, 4, 5));

  // Verify the map by checking values
  mjtNum mat_res_gathered[nnz_res];
  mju_gather(mat_res_gathered, mat_src, map, nnz_res);
  EXPECT_THAT(AsVector(mat_res_gathered, nnz_res),
              ElementsAre(1, 3, 5, 6));
}


TEST_F(UtilMiscTest, MjuSparseLower2SymMap) {
  // nr = 3
  // src = [[1, 0, 0],
  //        [2, 3, 0],
  //        [4, 5, 6]]
  constexpr int nr = 3;
  const mjtNum mat_src[] = {1, 2, 3, 4, 5, 6};
  const int rownnz_src[] = {1, 2, 3};
  const int rowadr_src[] = {0, 1, 3};
  const int colind_src[] = {0, 0, 1, 0, 1, 2};

  // res = [[*, *, *],
  //        [*, *, *],
  //        [*, *, *]] (dense symmetric)
  constexpr int res_nnz = 9;
  const int rownnz_res[] = {3, 3, 3};
  const int rowadr_res[] = {0, 3, 6};
  const int colind_res[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};

  int map[res_nnz];
  int cursor[nr];

  mju_lower2SymMap(map, nr, rowadr_res, rownnz_res, colind_res,
                   rowadr_src, rownnz_src, colind_src, cursor);

  // Expected map:
  // res(0,0) -> src(0,0) (k=0) => map[0] = 0
  // res(0,1) -> src(1,0) (k=1) => map[1] = 1
  // res(0,2) -> src(2,0) (k=3) => map[2] = 3
  // res(1,0) -> src(1,0) (k=1) => map[3] = 1
  // res(1,1) -> src(1,1) (k=2) => map[4] = 2
  // res(1,2) -> src(2,1) (k=4) => map[5] = 4
  // res(2,0) -> src(2,0) (k=3) => map[6] = 3
  // res(2,1) -> src(2,1) (k=4) => map[7] = 4
  // res(2,2) -> src(2,2) (k=5) => map[8] = 5
  EXPECT_THAT(map, ElementsAre(0, 1, 3, 1, 2, 4, 3, 4, 5));

  // Verify the map by checking values
  mjtNum mat_res[res_nnz];
  mju_gatherMasked(mat_res, mat_src, map, res_nnz);

  EXPECT_THAT(AsVector(mat_res, res_nnz),
              ElementsAre(1, 2, 4, 2, 3, 5, 4, 5, 6));
}

TEST_F(UtilMiscTest, MjuSparseLower2SymMapPartial) {
  // nr = 3
  // src = [[1, 0, 0],
  //        [2, 3, 0],
  //        [0, 0, 6]]
  constexpr int nr = 3;
  const mjtNum mat_src[] = {1, 2, 3, 6};
  const int rownnz_src[] = {1, 2, 1};
  const int rowadr_src[] = {0, 1, 3};
  const int colind_src[] = {0, 0, 1, 2};

  // res with a sparse symmetric pattern
  // res = [[*, *, *],
  //        [*, *, 0],
  //        [*, 0, *]]
  constexpr int res_nnz = 7;
  const int rownnz_res[] = {3, 2, 2};
  const int rowadr_res[] = {0, 3, 5};
  const int colind_res[] = {0, 1, 2, 0, 1, 0, 2};

  int map[res_nnz];
  int cursor[nr];

  mju_lower2SymMap(map, nr, rowadr_res, rownnz_res, colind_res,
                   rowadr_src, rownnz_src, colind_src, cursor);

  // Expected map for the non-zeros in res:
  // res(0,0) -> src(0,0) (k=0) => map[0] = 0
  // res(0,1) -> src(1,0) (k=1) => map[1] = 1
  // res(0,2) -> Unmapped       => map[2] = -1
  // res(1,0) -> src(1,0) (k=1) => map[3] = 1
  // res(1,1) -> src(1,1) (k=2) => map[4] = 2
  // res(2,0) -> Unmapped       => map[5] = -1
  // res(2,2) -> src(2,2) (k=3) => map[6] = 3
  EXPECT_THAT(map, ElementsAre(0, 1, -1, 1, 2, -1, 3));

  // Verify the map by checking values
  mjtNum mat_res[res_nnz];
  mju_gatherMasked(mat_res, mat_src, map, res_nnz);

  // Expected res values based on map:
  // mat_res[0] = mat_src[0] = 1
  // mat_res[1] = mat_src[1] = 2
  // mat_res[2]              = 0 (unmapped)
  // mat_res[3] = mat_src[1] = 2
  // mat_res[4] = mat_src[2] = 3
  // mat_res[5]              = 0 (unmapped)
  // mat_res[6] = mat_src[3] = 6
  EXPECT_THAT(AsVector(mat_res, res_nnz), ElementsAre(1, 2, 0, 2, 3, 0, 6));
}

TEST_F(UtilMiscTest, MjuIsZero) {
  mjtNum vec[1] = {1};
  EXPECT_EQ(mju_isZero(vec, 1), 0);
  EXPECT_EQ(mju_isZero(vec, 0), 1);
  vec[0] = 0;
  EXPECT_EQ(mju_isZero(vec, 1), 1);
  vec[0] = -0.0;
  EXPECT_EQ(mju_isZero(vec, 1), 1);
  EXPECT_EQ(mju_isZeroByte((const unsigned char*)vec, sizeof(mjtNum)), 0);
}

TEST_F(UtilMiscTest, MjuIsZeroByte) {
  // Zero length array
  EXPECT_TRUE(mju_isZeroByte(nullptr, 0));

  // zero length array with non-null pointer
  unsigned char vec0[1] = {0};
  EXPECT_TRUE(mju_isZeroByte(vec0, sizeof(vec0)));

  // one zero element array
  unsigned char vec1[1] = {0};
  EXPECT_TRUE(mju_isZeroByte(vec1, sizeof(vec1)));

  // one non-zero element array
  unsigned char vec2[2] = {1};
  EXPECT_FALSE(mju_isZeroByte(vec2, sizeof(vec2)));

  // Non-zero at start
  unsigned char vec3[3] = {1, 0, 0};
  EXPECT_FALSE(mju_isZeroByte(vec3, sizeof(vec3)));

  // Non-zero at end
  unsigned char vec4[3] = {0, 0, 1};
  EXPECT_FALSE(mju_isZeroByte(vec4, sizeof(vec4)));

  // Non-zero in middle
  unsigned char vec5[3] = {0, 1, 0};
  EXPECT_FALSE(mju_isZeroByte(vec5, sizeof(vec5)));
}

// --------------------------------- Interpolation -----------------------------

using InterpolationTest = MujocoTest;

TEST_F(InterpolationTest, mju_interpolate3D) {
  // quadratic functions should be interpolated exactly if order = 2
  auto quadratic_function_1 = [](mjtNum x, mjtNum y, mjtNum z) {
    return x*x + y*y + z*z;
  };
  auto quadratic_function_2 = [](mjtNum x, mjtNum y, mjtNum z) {
    return x*y*z + y*z*z + x*z*z;
  };
  auto quadratic_function_3 = [](mjtNum x, mjtNum y, mjtNum z) {
    return x*y*z + y*z*z + x*z*z + y*y*z + x*x*z + x + y + z;
  };
  static constexpr int order = 2;
  mjtNum coeff[3*(order+1)*(order+1)*(order+1)];
  int index = 0;
  for (int i = 0; i <= order; ++i) {
    for (int j = 0; j <= order; ++j) {
      for (int k = 0; k <= order; ++k) {
        coeff[3*index+0] = quadratic_function_1(.5*i, .5*j, .5*k);
        coeff[3*index+1] = quadratic_function_2(.5*i, .5*j, .5*k);
        coeff[3*index+2] = quadratic_function_3(.5*i, .5*j, .5*k);
        index++;
      }
    }
  }
  static constexpr int nsample = 5;
  for (int i = 0; i < nsample; ++i) {
    mjtNum sample[3];
    mjtNum expected[3];
    mjtNum res[3] = {0};
    sample[0] = mju_Halton(i, 2);
    sample[1] = mju_Halton(i, 3);
    sample[2] = mju_Halton(i, 5);
    expected[0] = quadratic_function_1(sample[0], sample[1], sample[2]);
    expected[1] = quadratic_function_2(sample[0], sample[1], sample[2]);
    expected[2] = quadratic_function_3(sample[0], sample[1], sample[2]);
    mju_interpolate3D(res, sample, coeff, order);
    EXPECT_NEAR(res[0], expected[0], 1e-10);
    EXPECT_NEAR(res[1], expected[1], 1e-10);
    EXPECT_NEAR(res[2], expected[2], 1e-10);
  }
}

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

// --------------------------------- History Buffers ---------------------------

using HistoryTest = MujocoTest;

// buffer layout: [user(1), cursor(1), times(n), values(n*dim)]
// cursor points to newest element (logical index n-1)
// after init, cursor=n-1, so physical indices equal logical indices

TEST_F(HistoryTest, Init) {
  constexpr int n = 4;
  constexpr int dim = 1;
  mjtNum buf[2 + n + n*dim];

  std::vector<mjtNum> times = {4, 6, 8, 10};
  std::vector<mjtNum> values = {99, 99, 99, 99};
  mju_historyInit(buf, n, dim, times.data(), values.data(), 0.0);

  // check header
  EXPECT_EQ(buf[0], 0.0);  // user
  EXPECT_EQ(buf[1], static_cast<mjtNum>(n-1));  // cursor = n-1

  // timestamps: [4, 6, 8, 10] (t=10 is newest)
  // values: [99, 99, 99, 99]
  // verify via read function (logical order)
  mjtNum res;
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 4.0, 0), 99.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 99.0);
}

TEST_F(HistoryTest, Init_Vector) {
  constexpr int n = 3;
  constexpr int dim = 2;
  mjtNum buf[2 + n + n*dim];

  std::vector<mjtNum> times = {-2, -1, 0};
  std::vector<mjtNum> values = {1.0, 2.0, 1.0, 2.0, 1.0, 2.0};
  mju_historyInit(buf, n, dim, times.data(), values.data(), 0.0);

  EXPECT_EQ(buf[1], static_cast<mjtNum>(n-1));  // cursor = n-1

  // verify via read function
  mjtNum res[dim];
  const mjtNum* ptr = mju_historyRead(buf, n, dim, res, -2.0, 0);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(ptr[0], 1.0);
  EXPECT_EQ(ptr[1], 2.0);
}

TEST_F(HistoryTest, Append) {
  constexpr int n = 4;
  constexpr int dim = 1;
  // Initialize buffer properly, then insert
  mjtNum buf[2 + 2*n];
  buf[0] = 0.0;
  buf[1] = n - 1;
  // timestamps: [4, 6, 8, 10]
  mjtNum times[] = {4, 6, 8, 10};
  mju_copy(buf + 2, times, n);
  // values: [0, 0, 0, 0]
  mju_zero(buf + 2 + n, n);

  // overwrite with specific values
  *mju_historyInsert(buf, n, dim, 4.0) = 1.0;
  *mju_historyInsert(buf, n, dim, 6.0) = 2.0;
  *mju_historyInsert(buf, n, dim, 8.0) = 3.0;
  *mju_historyInsert(buf, n, dim, 10.0) = 4.0;

  // now append at t=12
  *mju_historyInsert(buf, n, dim, 12.0) = 99.0;

  // verify logical order: [6, 8, 10, 12] -> [2, 3, 4, 99]
  mjtNum res;
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 4.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 12.0, 0), 99.0);

  // oldest should now be t=6
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 4.0, 0), 2.0);
}

TEST_F(HistoryTest, Append_Multiple) {
  constexpr int n = 3;
  constexpr int dim = 1;
  mjtNum buf[2 + 2*n];
  buf[0] = 0.0;
  buf[1] = n - 1;
  mjtNum times[] = {-2, -1, 0};
  mju_copy(buf + 2, times, n);
  mju_zero(buf + 2 + n, n);

  for (int i = 1; i <= 4; i++) {
    mjtNum i_real = static_cast<mjtNum>(i);
    *mju_historyInsert(buf, n, dim, i_real) = i_real;
  }
  // Final: logical timestamps [2, 3, 4], values [2, 3, 4]
  mjtNum res;
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 2.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 3.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 4.0, 0), 4.0);
}

TEST_F(HistoryTest, ReadVector_ExactMatch) {
  constexpr int n = 3;
  constexpr int dim = 2;
  mjtNum buf[2 + n + n*dim];
  buf[0] = 0.0;
  buf[1] = n - 1;
  mjtNum times[] = {0, 1, 2};
  mju_copy(buf + 2, times, n);
  mju_zero(buf + 2 + n, n*dim);

  // set values: t=0->(1,2), t=1->(3,4), t=2->(5,6)
  mjtNum* slot0 = mju_historyInsert(buf, n, dim, 0.0);
  slot0[0] = 1.0; slot0[1] = 2.0;
  mjtNum* slot1 = mju_historyInsert(buf, n, dim, 1.0);
  slot1[0] = 3.0; slot1[1] = 4.0;
  mjtNum* slot2 = mju_historyInsert(buf, n, dim, 2.0);
  slot2[0] = 5.0; slot2[1] = 6.0;

  mjtNum res[dim];
  const mjtNum* ptr = mju_historyRead(buf, n, dim, res, 1.0, 0);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(ptr[0], 3.0);
  EXPECT_EQ(ptr[1], 4.0);
}

TEST_F(HistoryTest, ReadVector_ZOH) {
  constexpr int n = 3;
  constexpr int dim = 2;
  mjtNum buf[2 + n + n*dim];
  buf[0] = 0.0;
  buf[1] = n - 1;
  mjtNum times[] = {0, 1, 2};
  mju_copy(buf + 2, times, n);
  mju_zero(buf + 2 + n, n*dim);

  mjtNum* slot0 = mju_historyInsert(buf, n, dim, 0.0);
  slot0[0] = 1.0; slot0[1] = 2.0;
  mjtNum* slot1 = mju_historyInsert(buf, n, dim, 1.0);
  slot1[0] = 3.0; slot1[1] = 4.0;
  mjtNum* slot2 = mju_historyInsert(buf, n, dim, 2.0);
  slot2[0] = 5.0; slot2[1] = 6.0;

  mjtNum res[dim];
  const mjtNum* ptr = mju_historyRead(buf, n, dim, res, 0.5, 0);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(ptr[0], 1.0);
  EXPECT_EQ(ptr[1], 2.0);
}

TEST_F(HistoryTest, ReadVector_Linear) {
  constexpr int n = 3;
  constexpr int dim = 2;
  mjtNum buf[2 + n + n*dim];
  buf[0] = 0.0;
  buf[1] = n - 1;
  mjtNum times[] = {0, 1, 2};
  mju_copy(buf + 2, times, n);
  mju_zero(buf + 2 + n, n*dim);

  mjtNum* slot0 = mju_historyInsert(buf, n, dim, 0.0);
  slot0[0] = 1.0; slot0[1] = 2.0;
  mjtNum* slot1 = mju_historyInsert(buf, n, dim, 1.0);
  slot1[0] = 3.0; slot1[1] = 4.0;
  mjtNum* slot2 = mju_historyInsert(buf, n, dim, 2.0);
  slot2[0] = 5.0; slot2[1] = 6.0;

  mjtNum res[dim];
  const mjtNum* ptr = mju_historyRead(buf, n, dim, res, 0.5, 1);
  EXPECT_EQ(ptr, nullptr);
  EXPECT_THAT(res[0], DoubleNear(2.0, 1e-10));  // (1+3)/2
  EXPECT_THAT(res[1], DoubleNear(3.0, 1e-10));  // (2+4)/2
}

TEST_F(HistoryTest, InsertOutOfOrder) {
  constexpr int n = 4;
  constexpr int dim = 1;
  mjtNum buf[2 + 2*n];
  mjtNum res;

  auto reset = [&]() {
    buf[0] = 0.0;
    buf[1] = n - 1;
    mjtNum times[] = {4, 6, 8, 10};
    mju_copy(buf + 2, times, n);
    mju_zero(buf + 2 + n, n);

    *mju_historyInsert(buf, n, dim, 4.0) = 1.0;
    *mju_historyInsert(buf, n, dim, 6.0) = 2.0;
    *mju_historyInsert(buf, n, dim, 8.0) = 3.0;
    *mju_historyInsert(buf, n, dim, 10.0) = 4.0;
  };

  // insert in middle (between t=8 and t=10)
  reset();
  *mju_historyInsert(buf, n, dim, 9.0) = 99.0;
  // logical: [6, 8, 9, 10] -> [2, 3, 99, 4]
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 9.0, 0), 99.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 4.0);

  // insert near start (between t=4 and t=6)
  reset();
  *mju_historyInsert(buf, n, dim, 5.0) = 99.0;
  // logical: [5, 6, 8, 10] -> [99, 2, 3, 4]
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 5.0, 0), 99.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 4.0);

  // insert before oldest (t=3 < t=4): replaces oldest
  reset();
  *mju_historyInsert(buf, n, dim, 3.0) = 99.0;
  // logical: [3, 6, 8, 10] -> [99, 2, 3, 4]
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 3.0, 0), 99.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 4.0);
}

TEST_F(HistoryTest, InsertReplaceOnCollision) {
  constexpr int n = 4;
  constexpr int dim = 1;
  mjtNum buf[2 + 2*n];
  mjtNum res;

  auto reset = [&]() {
    // timestamps: [4, 6, 8, 10], values initialized to 0
    buf[0] = 0.0;
    buf[1] = n - 1;
    mjtNum times[] = {4, 6, 8, 10};
    mju_copy(buf + 2, times, n);
    mju_zero(buf + 2 + n, n);

    *mju_historyInsert(buf, n, dim, 4.0) = 1.0;
    *mju_historyInsert(buf, n, dim, 6.0) = 2.0;
    *mju_historyInsert(buf, n, dim, 8.0) = 3.0;
    *mju_historyInsert(buf, n, dim, 10.0) = 4.0;
  };

  // collision in middle (t=8)
  reset();
  *mju_historyInsert(buf, n, dim, 8.0) = 99.0;
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 4.0, 0), 1.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 99.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 4.0);

  // collision at newest (t=10)
  reset();
  *mju_historyInsert(buf, n, dim, 10.0) = 99.0;
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 4.0, 0), 1.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 99.0);

  // collision at oldest (t=4)
  reset();
  *mju_historyInsert(buf, n, dim, 4.0) = 99.0;
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 4.0, 0), 99.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 6.0, 0), 2.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 8.0, 0), 3.0);
  EXPECT_EQ(*mju_historyRead(buf, n, dim, &res, 10.0, 0), 4.0);
}

void TriggerHistoryInitNonMonotonic() {
  mjtNum buf[10];
  mjtNum times[4] = {1, 2, 2, 4};  // not strictly increasing
  mjtNum values[4] = {0};
  mju_historyInit(buf, 4, 1, times, values, 0.0);
}

TEST_F(HistoryTest, Init_NonMonotonic) {
  EXPECT_FATAL_FAILURE(TriggerHistoryInitNonMonotonic(),
                       "mju_historyInit: times must be strictly increasing");
}

TEST_F(HistoryTest, CubicInterpolation) {
  int n = 2;
  int dim = 2;
  mjtNum buf[100];  // 2 + 2 + 2*2 = 8
  buf[0] = 0.0;
  buf[1] = n - 1;
  mjtNum times[] = {-1, 0};
  mju_copy(buf + 2, times, n);
  mju_zero(buf + 2 + n, n*dim);

  // Insert (0, 0, 1) and (1, 1, 0).
  // Dim 0: 0 -> 1. Spline: p(x) = 3x^2 - 2x^3
  // Dim 1: 1 -> 0. Spline: p(x) = 1 - 3x^2 + 2x^3
  mjtNum* slot0 = mju_historyInsert(buf, n, dim, 0.0);
  slot0[0] = 0.0; slot0[1] = 1.0;
  mjtNum* slot1 = mju_historyInsert(buf, n, dim, 1.0);
  slot1[0] = 1.0; slot1[1] = 0.0;

  mjtNum res[2];

  // Test midpoint x=0.5
  // Dim 0: 0.5
  // Dim 1: 1 - 0.5 = 0.5
  mju_historyRead(buf, n, dim, res, 0.5, 2);
  EXPECT_NEAR(res[0], 0.5, 1e-9);
  EXPECT_NEAR(res[1], 0.5, 1e-9);

  // Test x=0.25
  // Dim 0: 3*0.25^2 - 2*0.25^3
  // Dim 1: 1 - (3*0.25^2 - 2*0.25^3)
  mju_historyRead(buf, n, dim, res, 0.25, 2);
  mjtNum expected_0_25 = 3*0.25*0.25 - 2*0.25*0.25*0.25;
  EXPECT_NEAR(res[0], expected_0_25, 1e-9);
  EXPECT_NEAR(res[1], 1.0 - expected_0_25, 1e-9);

  // Test x=0.8
  // Dim 0: 3*0.8^2 - 2*0.8^3
  // Dim 1: 1 - (3*0.8^2 - 2*0.8^3)
  mju_historyRead(buf, n, dim, res, 0.8, 2);
  mjtNum expected_0_8 = 3*0.8*0.8 - 2*0.8*0.8*0.8;
  EXPECT_NEAR(res[0], expected_0_8, 1e-9);
  EXPECT_NEAR(res[1], 1.0 - expected_0_8, 1e-9);
}

}  // namespace
}  // namespace mujoco
