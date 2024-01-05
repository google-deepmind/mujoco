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

#include <cstddef>
#include <string>
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
  mjModel* model = LoadModelFromString("<mujoco/>");
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
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
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
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
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
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
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
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, NotNull());
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
