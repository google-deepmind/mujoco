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

// Tests for engine/engine_collision_driver.c.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"
#include "src/engine/engine_collision_driver.h"


namespace mujoco {
namespace {

using MjCollisionTest = MujocoTest;
using GeomPair = std::pair<std::string, std::string>;
using ::testing::IsEmpty;
using ::testing::ElementsAre;
using ::testing::NotNull;

// Returns a sorted list of pairs of colliding geom names, where each pair of
// geom names is sorted.
static std::vector<GeomPair> colliding_pairs(
    const mjModel* model, const mjData* data) {
  std::vector<GeomPair> result;
  for (int i = 0; i < data->ncon; i++) {
    std::string geom1 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom[0]);
    std::string geom2 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom[1]);
    result.push_back(GeomPair(std::min(geom1, geom2), std::max(geom1, geom2)));
  }
  std::sort(result.begin(), result.end());
  return result;
}

TEST_F(MjCollisionTest, AllCollisions) {
  static const char* const kModelFilePath =
      "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);

  // mjCOL_ALL is the default
  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data), ElementsAre(
      GeomPair("box", "sphere_collides"),
      GeomPair("box", "sphere_predefined")
  ));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, EmptyModel) {
  char error[1024];
  mjModel* model = LoadModelFromString("<mujoco/>", error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data), IsEmpty());

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, ZeroedHessian) {
  static const char* const kModelFilePath =
      "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);

  mj_fwdPosition(model, data);
  for (int i = 0; i < data->ncon; i++) {
    for (int j = 0; j < 36; j++) {
      EXPECT_FALSE(isnan(data->contact[i].H[j]))
          << "NaN in contact[" << i << "].H[" << j << "]";
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, ContactCount) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="plane" size="5 5 .01"/>
      </body>
      <body pos="0 0 0.9">
        <freejoint/>
        <geom type="sphere" size="1" pos="-1 -1 0"/>
        <geom type="sphere" size="1" pos="-1  1 0"/>
        <geom type="sphere" size="1" pos=" 1 -1 0"/>
        <geom type="sphere" size="1" pos=" 1  1 0"/>
        <geom type="sphere" size="1" pos="-2 -2 0"/>
        <geom type="sphere" size="1" pos="-2  2 0"/>
        <geom type="sphere" size="1" pos=" 2 -2 0"/>
        <geom type="sphere" size="1" pos=" 2  2 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m, d);

  // there are 8 spheres, all touching the floor
  EXPECT_EQ(d->ncon, 8);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(MjCollisionTest, FilterParent) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 0">
        <freejoint/>
        <geom name="colliding1" size="1" pos="0 0 100"/>
        <body>
          <geom size="1"/>
          <body>
            <joint axis="1 0 0"/>
            <geom size="1" pos="0 0 50"/>
            <body>
              <geom name="colliding2" size="1" pos="0 0 99.5"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m, d);

  // there should be zero contacts, because colliding1 and colliding2 are in
  // bodies that have a parent-child relationship, through welds
  EXPECT_EQ(d->ncon, 0);

  // when this filtering is disabled, the geoms should collide
  m->opt.disableflags |= mjDSBL_FILTERPARENT;
  mj_fwdPosition(m, d);

  EXPECT_THAT(colliding_pairs(m, d),
              ElementsAre(GeomPair("colliding1", "colliding2")));

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(MjCollisionTest, FilterParentDoesntAffectWorldBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="colliding1" size="1" pos="0 0 100"/>
      <body pos="0 0 0">
        <joint axis="1 0 0"/>
        <geom name="colliding2" size="1" pos="0 0 99.5"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m, d);

  // even though colliding1 and colliding2 are have a parent-child relationship,
  // they collide because colliding1 is in <worldbody>
  EXPECT_THAT(colliding_pairs(m, d),
              ElementsAre(GeomPair("colliding1", "colliding2")));

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(MjCollisionTest, TestOBB) {
  mjtNum bvh1[6] = {-1, -1, -1, 1, 1, 1};
  mjtNum bvh2[6] = {-1, -1, -1, 1, 1, 1};
  mjtNum pos1[3] = {0, 0, 0};
  mjtNum mat1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjtNum pos2[3] = {1.71, 1.71, 0};  // just a little more than 1+sqrt(2)/2
  mjtNum mat2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  EXPECT_THAT(
    mj_collideOBB(bvh1, bvh2, pos1, mat1, pos2, mat2, 0, NULL, NULL, 0), true);

  // rotate by 45 degrees
  mat2[0] = 1./mju_sqrt(2.); mat2[1] = -1./mju_sqrt(2.);
  mat2[3] = 1./mju_sqrt(2.); mat2[4] =  1./mju_sqrt(2.);

  EXPECT_THAT(
    mj_collideOBB(bvh1, bvh2, pos1, mat1, pos2, mat2, 0, NULL, NULL, 0), false);
}

TEST_F(MjCollisionTest, PlaneInBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom pos="0 0 0" type="plane" size="1 1 .01"/>
      </body>
      <body pos="0 0 .0499">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".05"/>
      </body>
    </worldbody>
    </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, NotNull());
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(MjCollisionTest, PinchingSucceeds) {
  constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    <worldbody>
      <geom name="floor" type="plane" size="0 0 1"/>

      <body name="gripper" pos="0 0 0.5">
        <joint name="lift" type="slide" axis="0 0 1" damping="50"/>
        <geom type="box" size="0.2 0.05 0.02" rgba="0.5 0.5 0.5 1"/> <!-- base -->

        <body name="left_finger" pos="-0.1 0 -0.1">
          <joint name="left_slide" type="slide" axis="1 0 0" damping="10"/>
          <geom type="box" size="0.02 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>

        <body name="right_finger" pos="0.1 0 -0.1">
          <joint name="right_slide" type="slide" axis="-1 0 0" damping="10"/>
          <geom type="box" size="0.02 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>
      </body>

      <flexcomp name="cloth" type="grid" dim="2" count="9 9 1" spacing="0.05 0.05 0.05"
                pos="0 0 0.1" radius="0.01">
        <edge equality="true"/>
      </flexcomp>
    </worldbody>

    <equality>
      <joint joint1="right_slide" joint2="left_slide"/>
    </equality>

    <tendon>
      <fixed name="grasp">
        <joint joint="right_slide" coef="1"/>
        <joint joint="left_slide" coef="1"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="lift" joint="lift" kp="600" dampratio="1" ctrlrange="-1 1"/>
      <position name="grasp" tendon="grasp" kp="200" dampratio="1" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, NotNull());

  int lift_id = mj_name2id(m, mjOBJ_ACTUATOR, "lift");
  int grasp_id = mj_name2id(m, mjOBJ_ACTUATOR, "grasp");

  // Phase 1: Lower gripper.
  // The gripper base starts at z=0.5. The finger has length 0.2 (size 0.1),
  // extending from z=0.4 to z=0.2 relative to base (center at -0.1).
  // The cloth is at z=0.1. We need to lower the gripper so the fingertips
  // reach the cloth. A lift value of -0.35 places the fingertips near z=0.05.

  for (int i = 0; i < 1000; ++i) {
    d->ctrl[lift_id] = -0.35;  // Lower
    d->ctrl[grasp_id] = 0;     // Open
    mj_step(m, d);
  }

  // Phase 2: Pinch
  for (int i = 0; i < 1000; ++i) {
    d->ctrl[lift_id] = -0.35;  // Hold height
    d->ctrl[grasp_id] = 0.8;   // Close (max 1)
    mj_step(m, d);
  }

  // Phase 3: Lift
  for (int i = 0; i < 2000; ++i) {
    d->ctrl[lift_id] = 0.5;   // Lift up
    d->ctrl[grasp_id] = 0.8;  // Keep closed
    mj_step(m, d);
  }

  // Check if cloth is lifted
  // flex verts are in d->flexvert_xpos
  // original z is ~0.1 (falling to floor ~0.0)
  // gripper lifted to > 0.5 probably

  // Find average Z of cloth
  double avg_z = 0;
  int nvert = m->flex_vertnum[0];
  for (int i = 0; i < nvert; ++i) {
    avg_z += d->flexvert_xpos[3 * i + 2];
  }
  avg_z /= nvert;

  // If lifted, avg_z should be significantly > 0.1
  // If failed (slipped), avg_z should be near 0 (floor)

  // Specialized primitives (mjraw_BoxTriangle, mjraw_CapsuleTriangle) should
  // enable stable pinching, so we expect the cloth to be lifted.
  EXPECT_GT(avg_z, 0.2) << "Cloth slipped out of gripper!";

  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
