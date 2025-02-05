// Copyright 2024 DeepMind Technologies Limited
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

// Tests for engine/engine_collision_gjk.c.

#include "src/engine/engine_collision_gjk.h"

#include <array>
#include <vector>

#include <ccd/ccd.h>
#include <ccd/vec3.h>

#include "src/engine/engine_collision_convex.h"
#include <mujoco/mujoco.h>
#include <mujoco/mjtnum.h>
#include "test/fixture.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// uncomment to run tests with libccd
// #define TEST_WITH_LIBCCD

namespace mujoco {
namespace {

using ::testing::NotNull;
using ::testing::ElementsAre;
using ::testing::Pointwise;
using ::testing::DoubleNear;

constexpr mjtNum kTolerance = 1e-6;
constexpr int kMaxIterations = 1000;
constexpr char kEllipoid[] = R"(
<mujoco model="Ellipsoid Test">
  <compiler angle="radian"/>
  <size nkey="1"/>
  <worldbody>
    <geom name="geom1" size="0.1 0.1 0.1" pos="0 0 -0.1" type="ellipsoid"/>
    <body pos="0 0 0.1">
      <joint type="free" limited="false" actuatorfrclimited="false"/>
      <geom name="geom2" size="0.01 0.02 0.1" type="ellipsoid"/>
    </body>
  </worldbody>

  <keyframe>
    <key time="1.446"
      qpos="0.000886189 -0.0303047 0.0951303 0.98783 0.155468 0.00454524 1.60038e-06"
      qvel="0.00769267 -0.258656 -0.0775641 2.73712 0.0813998 -0.000166485"/>
  </keyframe>
</mujoco>)";

mjtNum GeomDist(mjModel* m, mjData* d, int g1, int g2, mjtNum x1[3],
                mjtNum x2[3], mjtNum cutoff = mjMAXVAL) {
  mjCCDConfig config;
  mjCCDStatus status;

  // set config
  config.max_iterations = kMaxIterations,
  config.tolerance = kTolerance,
  config.max_contacts = 0;   // no geom contacts needed
  config.dist_cutoff = cutoff;

  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, m, d, g1, 0);
  mjc_initCCDObj(&obj2, m, d, g2, 0);

  mjtNum dist = mjc_ccd(&config, &status, &obj1, &obj2);
  if (status.nx > 0) {
    if (x1 != nullptr) mju_copy3(x1, status.x1);
    if (x2 != nullptr) mju_copy3(x2, status.x2);
  }
  return dist;
}

int Penetration(mjtNum& depth, std::vector<mjtNum>& dir,
                std::vector<mjtNum>& pos, mjModel* model, mjData* data,
                int g1, int g2, mjtNum margin = 0, int max_contacts = 1) {
  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, model, data, g1, margin);
  mjc_initCCDObj(&obj2, model, data, g2, margin);

#if defined(TEST_WITH_LIBCCD)
  if (max_contacts == 1) {
    ccd_t ccd;
    CCD_INIT(&ccd);
    ccd.mpr_tolerance = kTolerance;
    ccd.epa_tolerance = kTolerance;
    ccd.max_iterations = kMaxIterations;
    ccd.center1 = mjccd_center;
    ccd.center2 = mjccd_center;
    ccd.support1 = mjccd_support;
    ccd.support2 = mjccd_support;

    ccd_real_t ccd_depth;
    ccd_vec3_t ccd_dir, ccd_pos;

    int ret = ccdMPRPenetration(&obj1, &obj2, &ccd, &ccd_depth, &ccd_dir,
                                &ccd_pos);
    if (ret) return 0;
    dir.resize(3);
    pos.resize(3);
    depth = -ccd_depth;
    mju_copy3(dir.data(), ccd_dir.v);
    mju_copy3(pos.data(), ccd_pos.v);
    return 1;
  }
#endif

  mjCCDConfig config;
  mjCCDStatus status;

  // set config
  config.max_iterations = kMaxIterations;
  config.tolerance = kTolerance;
  config.max_contacts = max_contacts;
  config.dist_cutoff = 0;  // no geom distances needed
  config.max_contacts = max_contacts;

  mjtNum dist = mjc_ccd(&config, &status, &obj1, &obj2);
  if (dist < 0) {
    dir.resize(3 * status.nx);
    pos.resize(3 * status.nx);
    for (int i = 0; i < status.nx; ++i) {
      // compute direction
      mju_sub3(&dir[3 * i], status.x1 + 3 * i, status.x2 + 3 * i);
      mju_normalize3(&dir[3 * i]);

      // compute position
      pos[3 * i + 0] = 0.5 * (status.x1[0 + 3 * i] + status.x2[0 + 3 * i]);
      pos[3 * i + 1] = 0.5 * (status.x1[1 + 3 * i] + status.x2[1 + 3 * i]);
      pos[3 * i + 2] = 0.5 * (status.x1[2 + 3 * i] + status.x2[2 + 3 * i]);
    }
    depth = dist;
    return status.nx;
  }

  // no contacts
  return 0;
}

using MjGjkTest = MujocoTest;

TEST_F(MjGjkTest, SphereSphereDist) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="geom1" type="sphere" pos="-1.5 0 0" size="1"/>
      <geom name="geom2" type="sphere" pos="1.5 0 0" size="1"/>
    </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  mjtNum x1[3], x2[3];
  mjtNum dist = GeomDist(model, data, geom1, geom2, x1, x2);

  EXPECT_EQ(dist, 1);
  EXPECT_THAT(x1, ElementsAre(-.5, 0, 0));
  EXPECT_THAT(x2, ElementsAre(.5, 0, 0));
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, SphereSphereDistCutoff) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="geom1" type="sphere" pos="-1.5 0 0" size="1"/>
      <geom name="geom2" type="sphere" pos="1.5 0 0" size="1"/>
    </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  mjtNum dist = GeomDist(model, data, geom1, geom2, nullptr, nullptr, .999999);

  EXPECT_EQ(dist, mjMAXVAL);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, SphereSphereNoDist) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="geom1" type="sphere" pos="-1.5 0 0" size="1"/>
      <geom name="geom2" type="sphere" pos="1.5 0 0" size="1"/>
    </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 0);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, SphereSphereIntersect) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="sphere" pos="-1 0 0" size="3"/>
    <geom name="geom2" type="sphere" pos="3 0 0" size="3"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 1);

  // penetration depth
  EXPECT_NEAR(dist, -2, kTolerance);

  // direction
  EXPECT_NEAR(dir[0], 1, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], 0, kTolerance);

  // position
  EXPECT_NEAR(pos[0], 1, kTolerance);
  EXPECT_NEAR(pos[1], 0, kTolerance);
  EXPECT_NEAR(pos[2], 0, kTolerance);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxDepth) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="-1 0 0" size="2.5 2.5 2.5"/>
    <geom name="geom2" type="box" pos="1.5 0 0" size="1 1 1"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");

  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 1);

  EXPECT_NEAR(dist, -1, kTolerance);
  EXPECT_NEAR(dir[0], 1, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], 0, kTolerance);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxDepth2) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="0 0 0" size="5 5 .1"/>
    <geom name="geom2" type="box" pos="0 0 0" size="1 1 1"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat + 9;
  mjtNum* xpos = data->geom_xpos + 3;

  xpos[0] = -0.000171208577507291721461757383;
  xpos[1] = -0.000171208577507290908310128019;
  xpos[2] = 1.067119586248553853025100579544;

  xmat[0] = 0.999999966039443077825410455262;
  xmat[1] = -0.000000033960556969622165789148;
  xmat[2] = -0.000260616790777324182967061850;
  xmat[3] = -0.000000033960556972087627235699;
  xmat[4] = 0.999999966039443077825410455262;
  xmat[5] = -0.000260616790777321797722282382;
  xmat[6] = 0.000260616790777324182967061850;
  xmat[7] = 0.000260616790777321797722282382;
  xmat[8] = 0.999999932078886044628518448008;

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  if (ncons == 1) {
    EXPECT_NEAR(dist, -0.033401579411886845, kTolerance);
    EXPECT_NEAR(dir[0], 0, kTolerance);
    EXPECT_NEAR(dir[1], 0, kTolerance);
    EXPECT_NEAR(dir[2], 1, kTolerance);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxDepth3) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="0 0 0" size="0.25 0.25 0.05"/>
    <geom name="geom2" type="box" pos="0 0 0" size="0.25 0.25 0.05"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat;
  mjtNum* xpos = data->geom_xpos;

  xmat[0] = 0.965925826289068201191412299522;
  xmat[1] = -0.258819045102520739476403832668;
  xmat[2] = 0.000000000000000006339100926609;
  xmat[3] = 0.258819045102520739476403832668;
  xmat[4] = 0.965925826289068201191412299522;
  xmat[5] = -0.000000000000000214827792362716;
  xmat[6] = 0.000000000000000049478422780336;
  xmat[7] = 0.000000000000000209148392896446;
  xmat[8] = 1.000000000000000000000000000000;

  xpos[0] = -0.015346499999999199323474918799;
  xpos[1] = -0.023505500000000002086553152481;
  xpos[2] = -4.562296442400120888294168253196;

  xmat = data->geom_xmat + 9;
  xpos = data->geom_xpos + 3;

  xmat[0] = 0.866025403784438707610604524234;
  xmat[1] = -0.499999999999999944488848768742;
  xmat[2] = 0.000000000000000018716705841316;
  xmat[3] = 0.499999999999999944488848768742;
  xmat[4] = 0.866025403784438707610604524234;
  xmat[5] = -0.000000000000000263161736875730;
  xmat[6] = 0.000000000000000115371725704125;
  xmat[7] = 0.000000000000000237263102359077;
  xmat[8] = 1.000000000000000000000000000000;

  xpos[0] = -0.015346499999999797803074130798;
  xpos[1] = -0.023505499999999998617106200527;
  xpos[2] = -4.659230360891631228525966434972;

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 1);
  EXPECT_NEAR(dist, -0.003066, kTolerance);
  EXPECT_NEAR(dir[0], 0, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], -1, kTolerance);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxTouching) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="0 0 1.859913200000001376466229885409" size="1 1 1"/>
    <geom name="geom2" type="box" pos="0 2 1.859913200000001376466229885409" size="1 1 1"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="0 0 1.9" size="1 1 1"/>
    <geom name="geom2" type="box" pos="0 0 0" size="10 10 1"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 4);
  EXPECT_NEAR(dist, -.1, kTolerance);

  EXPECT_NEAR(dir[0], 0, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], -1, kTolerance);

  EXPECT_THAT(pos, Pointwise(DoubleNear(kTolerance), {-1.0,  1.0, 0.95,
                                                       1.0,  1.0, 0.95,
                                                       1.0, -1.0, 0.95,
                                                      -1.0, -1.0, 0.95}));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD2) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="9.5 9.5 1.9" size="1 1 1"/>
    <geom name="geom2" type="box" pos="0 0 0" size="10 10 1"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 4);
  EXPECT_NEAR(dist, -.1, kTolerance);

  EXPECT_NEAR(dir[0], 0, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], -1, kTolerance);

  EXPECT_THAT(pos, Pointwise(DoubleNear(kTolerance), { 8.5, 10.0, 0.95,
                                                      10.0, 10.0, 0.95,
                                                      10.0,  8.5, 0.95,
                                                       8.5,  8.5, 0.95}));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD3) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom type="box" name="geom1" size="5 5 .1" pos="0 0 0"/>
    <geom type="box" name="geom2" size="1 1 1"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat + 9;
  mjtNum* xpos = data->geom_xpos + 3;

  xmat[0] = 0.999999806540386004805043285160;
  xmat[1] = -0.000014738590672566122784237219;
  xmat[2] = 0.000621853651764864637230267874;
  xmat[3] = -0.000621853434269146370175218586;
  xmat[4] = 0.000014756878555191479777952690;
  xmat[5] = 0.999999806540251667819063641218;
  xmat[6] = -0.000014747764440060310685981504;
  xmat[7] = -0.999999999782504311873765345808;
  xmat[8] = 0.000014747710457105431443303178;

  xpos[0] = -0.941218618591869393696924817050;
  xpos[1] = 2.209729011624415928594089564285;
  xpos[2] = 1.095456702630382306296041861060;


  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 4);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD4) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
   <geom name="geom1" size="0.25 0.25 0.05" type="box"/>
   <geom name="geom2" size="0.25 0.25 0.05" type="box"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat;
  mjtNum* xpos = data->geom_xpos;

  xmat[0] = 0.500063246694118501700643264485;
  xmat[1] = -0.865988885078582182330819705385;
  xmat[2] = -0.000015036290463686326402846169;
  xmat[3] = 0.865988885208801795201338791230;
  xmat[4] = 0.500063246603650646271432833601;
  xmat[5] = 0.000009541064416582982810641038;
  xmat[6] = -0.000000743359510433135621196039;
  xmat[7] = -0.000017792396065397684211655677;
  xmat[8] = 0.999999999841438502734547455475;

  xpos[0] = -0.015346718925143524800414063236;
  xpos[1] = -0.023500448793229846561336771060;
  xpos[2] = -4.859382717259980388746498647379;

  xmat = data->geom_xmat + 9;
  xpos = data->geom_xpos + 3;

  xmat[0] = 0.999999999448633714038692232862;
  xmat[1] = -0.000033207420761195452995305499;
  xmat[2] = -0.000000044925527333868828730462;
  xmat[3] = 0.000033207420790006526530903364;
  xmat[4] = 0.999999999448428988912951353996;
  xmat[5] = 0.000000641458652741046316968134;
  xmat[6] = 0.000000044904226121706672357864;
  xmat[7] = -0.000000641460144248257277838641;
  xmat[8] = 0.999999999999794386695839421009;

  xpos[0] = -0.015347749710384111718197708285;
  xpos[1] = -0.023500601273213628239489025873;
  xpos[2] = -4.958782854594746325460619118530;

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 8);
  EXPECT_NEAR(dist, -0.00060425119242707459, kTolerance);

  EXPECT_NEAR(dir[0], 0, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], -1, kTolerance);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD5) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
   <geom name="geom1" size="0.25 0.25 0.05" type="box"/>
   <geom name="geom2" size="0.25 0.25 0.05" type="box"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat;
  mjtNum* xpos = data->geom_xpos;

  xmat[0] = 0.965955045562010394810670277366;
  xmat[1] = -0.258709898141739669252814337597;
  xmat[2] = -0.000196358811267032467391333017;
  xmat[3] = 0.258709919231419560592399875532;
  xmat[4] = 0.965955055634174608591990818240;
  xmat[5] = 0.000090476785846218643442895324;
  xmat[6] = 0.000166266546411239724218358860;
  xmat[7] = -0.000138196479997660070767120932;
  xmat[8] = 0.999999976628582865068040064216;

  xpos[0] = -0.015381524498156991936914650410;
  xpos[1] = -0.023527931890396581310342938309;
  xpos[2] = -4.559214004409498421921398403356;

  xmat = data->geom_xmat + 9;
  xpos = data->geom_xpos + 3;

  xmat[0] = 0.866076536677693908927722077351;
  xmat[1] = -0.499911388413602053581996642606;
  xmat[2] = -0.000190658753729162216972170540;
  xmat[3] = 0.499911409912061843741071243130;
  xmat[4] = 0.866076540935322825021103199106;
  xmat[5] = 0.000086494189368211055798235654;
  xmat[6] = 0.000121885643632020743577087929;
  xmat[7] = -0.000170223074359586521841353202;
  xmat[8] = 0.999999978083999430111816764111;

  xpos[0] = -0.015358668590921718474784363195;
  xpos[1] = -0.023542070504611382203430380855;
  xpos[2] = -4.659108354876987156956147373421;


  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 8);
  EXPECT_NEAR(dist, -0.0001077858631973211, kTolerance);

  EXPECT_NEAR(dir[0], 0.00019065, kTolerance);
  EXPECT_NEAR(dir[1], -8.6494189274575805e-05, kTolerance);
  EXPECT_NEAR(dir[2], -1, kTolerance);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD6) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" size=".5 .5 .1" pos="0 0 -.1"/>
    <geom name="geom2" type="box" size=".1 .1 .1" pos="0 0 0"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat + 9;
  mjtNum* xpos = data->geom_xpos + 3;

  xmat[0] = -0.412617528992808124677083014831;
  xmat[1] = -0.910903939143411389700588642881;
  xmat[2] = -0.000887930675351447824816819576;
  xmat[3] = 0.910904370383107120368038067681;
  xmat[4] = -0.412617275794986082537718630192;
  xmat[5] = -0.000460143975736545586020798115;
  xmat[6] = 0.000052771423713213129642884969;
  xmat[7] = -0.000998683403024198425301793947;
  xmat[8] = 0.999999499923193035932911243435;

  xpos[0] = 0.413029898172642018217004533653;
  xpos[1] = 0.190777715293135141649827346555;
  xpos[2] = 0.100006658017411736993906856696;

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 5);
  EXPECT_NEAR(dist, -0.00009843, kTolerance);

  EXPECT_NEAR(dir[0], -0.0008879306751646528, kTolerance);
  EXPECT_NEAR(dir[1], -0.00046014397575771832, kTolerance);
  EXPECT_NEAR(dir[2], 1, kTolerance);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD7) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" size=".25 .25 .05" pos="0 0 0"/>
    <geom name="geom2" type="box" size=".25 .25 .05" pos="0 0 0"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat;
  mjtNum* xpos = data->geom_xpos;

  xmat[0] = 0.482851932827058627495375731087;
  xmat[1] = -0.875697459006381406787511423317;
  xmat[2] = 0.002823341095436950488190008812;
  xmat[3] = 0.875701084072774249555948244961;
  xmat[4] = 0.482853601927766051815638093103;
  xmat[5] = -0.000102269990141710693382082198;
  xmat[6] = -0.001273702846902712276094815635;
  xmat[7] = 0.002521784120391480209927292933;
  xmat[8] = 0.999996009134990648803409385437;

  xpos[0] = -0.002020740254618143012105280221;
  xpos[1] = -0.022654384848980465422263463893;
  xpos[2] = -4.858542902144324493463045655517;

  xmat = data->geom_xmat + 9;
  xpos = data->geom_xpos + 3;

  xmat[0] = 0.999985133805306514176436394337;
  xmat[1] = -0.005293845271528460454113496070;
  xmat[2] = 0.001306663930443651821383665990;
  xmat[3] = 0.005293871114312041943616993223;
  xmat[4] = 0.999985987232967277194006783247;
  xmat[5] = -0.000016319793417504115210251922;
  xmat[6] = -0.001306559226005186893221354794;
  xmat[7] = 0.000023236861241766870316309210;
  xmat[8] = 0.999999146181155484924829579541;

  xpos[0] = -0.011066235018223425159988870803;
  xpos[1] = -0.023114696036485724711662115283;
  xpos[2] = -4.958375812037025376355359185254;


  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 8);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD8) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" size=".25 .25 .05" pos="0 0 0"/>
    <geom name="geom2" type="box" size=".25 .25 .05" pos="0 0 0"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat;
  mjtNum* xpos = data->geom_xpos;

  xmat[0] = 1.000000000000000000000000000000;
  xmat[1] = 0.000000000000000000000000000000;
  xmat[2] = 0.000000000000000000000000000000;
  xmat[3] = 0.000000000000000000000000000000;
  xmat[4] = 1.000000000000000000000000000000;
  xmat[5] = 0.000000000000000000000000000000;
  xmat[6] = 0.000000000000000000000000000000;
  xmat[7] = 0.000000000000000000000000000000;
  xmat[8] = 1.000000000000000000000000000000;

  xpos[0] = -0.015346500000000000765720820084;
  xpos[1] = -0.023505499999999998617106200527;
  xpos[2] = -4.859662640000005140450412000064;

  xmat = data->geom_xmat + 9;
  xpos = data->geom_xpos + 3;

  xmat[0] = 1.000000000000000000000000000000;
  xmat[1] = 0.000000000000000000000000000000;
  xmat[2] = 0.000000000000000000000000000000;
  xmat[3] = 0.000000000000000000000000000000;
  xmat[4] = 1.000000000000000000000000000000;
  xmat[5] = -0.000000000000000015361939765351;
  xmat[6] = 0.000000000000000000000000000000;
  xmat[7] = 0.000000000000000015361939765351;
  xmat[8] = 1.000000000000000000000000000000;

  xpos[0] = -0.015346500000000000765720820084;
  xpos[1] = -0.023505499999999998617106200527;
  xpos[2] = -4.958574289672835533338002278470;

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 4);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBoxMultiCCD9) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" size=".025 .025 .025" pos="0 0 0"/>
    <geom name="geom2" type="box" size=".025 .025 .025" pos="0 0 0"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjtNum* xmat = data->geom_xmat;
  mjtNum* xpos = data->geom_xpos;

  xmat[0] = 1.0000000000000000000000000000000000000000;
  xmat[1] = -0.0000000000000000000000000000000000050579;
  xmat[2] = 0.0000000000000000001927715439853908006818;
  xmat[3] = 0.0000000000000000000000000000000000056403;
  xmat[4] = 1.0000000000000000000000000000000000000000;
  xmat[5] = -0.0000000000000000030208514688407265124010;
  xmat[6] = -0.0000000000000000001927715439853908006818;
  xmat[7] = 0.0000000000000000030208514688407265124010;
  xmat[8] = 1.0000000000000000000000000000000000000000;

  xpos[0] = -0.1071400000000000268807198722242901567370;
  xpos[1] = -0.1928599999999999758948376893386011943221;
  xpos[2] = 0.1749951524564917204607183975895168259740;

  xmat = data->geom_xmat + 9;
  xpos = data->geom_xpos + 3;

  xmat[0] = 1.0000000000000000000000000000000000000000;
  xmat[1] = 0.0000000000000000000000000000000037070001;
  xmat[2] = -0.0000000000000000649578747741268744461630;
  xmat[3] = 0.0000000000000000000000000000000064174485;
  xmat[4] = 1.0000000000000000000000000000000000000000;
  xmat[5] = 0.0000000000000001558617582226398399563910;
  xmat[6] = 0.0000000000000000649578747741268744461630;
  xmat[7] = -0.0000000000000001558617582226398399563910;
  xmat[8] = 1.0000000000000000000000000000000000000000;

  xpos[0] = -0.1071400000000000268807198722242901567370;
  xpos[1] = -0.1928599999999999758948376893386011943221;
  xpos[2] = 0.2156259187793853615566774806211469694972;


  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 0, 1000);

  EXPECT_EQ(ncons, 4);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, SmallBoxMesh) {
  static constexpr char xml[] = R"(
  <mujoco>
  <asset>
    <mesh name="box" scale=".5 .5 .1"
      vertex="-1 -1 -1
               1 -1 -1
               1  1 -1
               1  1  1
               1 -1  1
              -1  1 -1
              -1  1  1
              -1 -1  1"/>
    <mesh name="smallbox" scale=".1 .1 .1"
      vertex="-1 -1 -1
               1 -1 -1
               1  1 -1
               1  1  1
               1 -1  1
              -1  1 -1
              -1  1  1
              -1 -1  1"/>
  </asset>

  <worldbody>
    <geom name="geom1" pos="0 0 -.1" mesh="box" type="mesh"/>
    <geom name="geom2" pos="0 0 .1" mesh="smallbox" type="mesh"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 1);
  EXPECT_NEAR(dist, 0, kTolerance);

  // direction
  EXPECT_NEAR(dir[0], 0, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], 1, kTolerance);

  // position
  EXPECT_NEAR(mju_abs(pos[0]), 0.08333333, kTolerance);  // -pos[0] on ARM
  EXPECT_NEAR(pos[1], 0, kTolerance);
  EXPECT_NEAR(pos[2], 0, kTolerance);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, EllipsoidEllipsoidPenetrating) {
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(kEllipoid, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 1);
  EXPECT_NEAR(dist, -0.00022548856248122027, kTolerance);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, EllipsoidEllipsoid) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="ellipsoid" pos="1.5 0 -.5" size=".15 .30 .20"/>
    <geom name="geom2" type="ellipsoid" pos="1.5 .5 .5" size=".10 .10 .15"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  mjtNum dist = GeomDist(model, data, geom1, geom2, nullptr, nullptr);

  EXPECT_NEAR(dist, 0.7542, .0001);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, BoxBox) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="box" pos="-1.5 .5 0" size="1 1 1"/>
    <geom name="geom2" type="box" pos="1.5 0 0" size="1 1 1"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  mjtNum dist = GeomDist(model, data, geom1, geom2, nullptr, nullptr);

  EXPECT_EQ(dist, 1);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, LongBox) {
  static constexpr char xml[] = R"(
<mujoco>
  <asset>
    <mesh name="long_box"
          vertex="-1 -1 -1 1 -1 -1 1 1 -1 1 1 1 1 -1 1 -1 1 -1 -1 1 1 -1 -1 1"
          scale=".6 .03 .03"/>
  </asset>
  <worldbody>
    <geom name="geom1" type="box" size="1 1 .3" pos="0 0 -.3"/>
    <geom name="geom2" type="mesh" mesh="long_box" pos="0 0 .02" euler="0 0 40"/>
  </worldbody>
</mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2);

  EXPECT_EQ(ncons, 1);
  EXPECT_NEAR(dist, -0.01, kTolerance);

  EXPECT_NEAR(dir[0], 0, kTolerance);
  EXPECT_NEAR(dir[1], 0, kTolerance);
  EXPECT_NEAR(dir[2], 1, kTolerance);

  EXPECT_NEAR(pos[0], 0, kTolerance);
  EXPECT_NEAR(pos[1], 0, kTolerance);
  EXPECT_NEAR(pos[2], -0.005, kTolerance);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, EllipsoidEllipsoidIntersect) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="ellipsoid" pos="1.5 0 -.5" size=".15 .30 .20"/>
    <geom name="geom2" type="ellipsoid" pos="1.5 .5 .5" size=".10 .10 .15"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  std::vector<mjtNum> dir, pos;
  mjtNum dist;
  int ncons = Penetration(dist, dir, pos, model, data, geom1, geom2, 15);

  EXPECT_EQ(ncons, 1);
  EXPECT_NEAR(dist, -14.245732934582151, kTolerance);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjGjkTest, CapsuleCapsule) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="capsule" pos="-.3 .2 -.4" size=".15 .30"/>
    <geom name="geom2" type="capsule" pos=".3 .2 .4" size=".10 .10"/>
  </worldbody>
  </mujoco>)";

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int geom1 = mj_name2id(model, mjOBJ_GEOM, "geom1");
  int geom2 = mj_name2id(model, mjOBJ_GEOM, "geom2");
  mjtNum dist = GeomDist(model, data, geom1, geom2, nullptr, nullptr);

  EXPECT_NEAR(dist, 0.4711, .0001);
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
