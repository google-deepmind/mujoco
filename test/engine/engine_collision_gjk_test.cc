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

#include <ccd/ccd.h>
#include <ccd/vec3.h>

#include "src/engine/engine_collision_convex.h"
#include <mujoco/mujoco.h>
#include <mujoco/mjtnum.h>
#include "test/fixture.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace mujoco {
namespace {

using ::testing::NotNull;
using ::testing::ElementsAre;

constexpr mjtNum kTolerance = 1e-6;
constexpr int kMaxIterations = 1000;

// ccd center function
void mjccd_center(const void *obj, ccd_vec3_t *center) {
  mjc_center(center->v, (const mjCCDObj*) obj);
}

// ccd support function
void mjccd_support(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *vec) {
  mjc_support(vec->v, (mjCCDObj*) obj, _dir->v);
}

mjtNum GeomDist(mjModel* m, mjData* d, int g1, int g2, mjtNum x1[3],
                mjtNum x2[3]) {
  mjCCDConfig config;
  mjCCDStatus status;

  // set config
  config.max_iterations = kMaxIterations,
  config.tolerance = kTolerance,
  config.contacts = 0;   // no geom contacts needed
  config.distances = 1;  // no geom distances needed

  mjCCDObj obj1 = {m, d, g1, -1, -1, -1, -1, 0, {1, 0, 0, 0}, mjc_center,
                   mjc_support};
  mjCCDObj obj2 = {m, d, g2, -1, -1, -1, -1, 0, {1, 0, 0, 0}, mjc_center,
                   mjc_support};
  mjtNum dist = mjc_ccd(&config, &status, &obj1, &obj2);
  if (x1 != nullptr) mju_copy3(x1, status.x1);
  if (x2 != nullptr) mju_copy3(x2, status.x2);
  return dist;
}

// drop in replacement for ccdMPRPenetration taken from mjc_penetration
int PenetrationWrapper(mjCCDObj* obj1, mjCCDObj* obj2, const ccd_t* ccd,
                       ccd_real_t* depth, ccd_vec3_t* dir, ccd_vec3_t* pos) {
  mjCCDConfig config;
  mjCCDStatus status;

  // set config
  config.max_iterations = ccd->max_iterations,
  config.tolerance = ccd->mpr_tolerance,
  config.contacts = 1;
  config.distances = 0;  // no geom distances needed

  mjtNum dist = mjc_ccd(&config, &status, obj1, obj2);
  if (dist < 0) {
    if (depth) *depth = -dist;
    if (dir) {
      mju_sub3(dir->v, status.x1, status.x2);
      mju_normalize3(dir->v);
    }
    if (pos) {
      pos->v[0] = 0.5 * (status.x1[0] + status.x2[0]);
      pos->v[1] = 0.5 * (status.x1[1] + status.x2[1]);
      pos->v[2] = 0.5 * (status.x1[2] + status.x2[2]);
    }
    return 0;
  }
  if (depth) *depth = 0;
  if (dir) mju_zero3(dir->v);
  if (pos) mju_zero3(dir->v);
  return 1;
}

mjtNum Penetration(mjModel* m, mjData* d, int g1, int g2,
                   mjtNum dir[3] = nullptr, mjtNum pos[3] = nullptr) {
  mjCCDObj obj1 = {m, d, g1, -1, -1, -1, -1, 0, {1, 0, 0, 0}, mjc_center,
                   mjc_support};
  mjCCDObj obj2 = {m, d, g2, -1, -1, -1, -1, 0, {1, 0, 0, 0}, mjc_center,
                   mjc_support};
  ccd_t ccd;
  // CCD_INIT(&ccd);  // uncomment to run ccdMPRPenetration
  ccd.mpr_tolerance = kTolerance;
  ccd.epa_tolerance = kTolerance;
  ccd.max_iterations = kMaxIterations;
  ccd.center1 = mjccd_center;
  ccd.center2 = mjccd_center;
  ccd.support1 = mjccd_support;
  ccd.support2 = mjccd_support;

  ccd_real_t depth;
  ccd_vec3_t ccd_dir, ccd_pos;

  int ret = PenetrationWrapper(&obj1, &obj2, &ccd, &depth, &ccd_dir, &ccd_pos);
  if (ret) return 0;  // objects not colliding
  if (dir) mju_copy3(dir, ccd_dir.v);
  if (pos) mju_copy3(pos, ccd_pos.v);
  return -depth;
}

using MjGjkTest = MujocoTest;

TEST_F(MjGjkTest, SphereSphere) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body pos="-1.5 0 0">
      <freejoint/>
      <geom name="geom1" type="sphere" size="1"/>
    </body>
    <body pos="1.5 0 0">
      <freejoint/>
      <geom name="geom2" type="sphere" size="1"/>
    </body>
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
    <geom name="geom1" type="box"  pos="-1.5 .5 0" size="1 1 1"/>
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

TEST_F(MjGjkTest, EllipsoidEllipsoidIntersect) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <geom name="geom1" type="ellipsoid" pos="1.5 0 -.5" size="2.25 4.5 3"/>
    <geom name="geom2" type="ellipsoid" pos="1.5 .5 .5" size="1.5 1.5 2.25"/>
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

  EXPECT_NEAR(dist, 0, kTolerance);
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
