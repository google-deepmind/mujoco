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

// Tests for ray casting.

#include <array>
#include <cstddef>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

static constexpr char kRayCastingModel[] = R"(
<mujoco>
  <worldbody>
    <geom name="static_group1" type="sphere" size=".1" pos="1 0 0"
     group="1"/>
    <body pos="0 0 0">
      <body pos="0 0 0">
        <geom name="group0" type="sphere" size=".1" pos="3 0 0"/>
      </body>
      <geom name="group2" type="sphere" size=".1" pos="5 0 0" group="2"/>
    </body>
  </worldbody>
</mujoco>
)";

using ::testing::NotNull;
using RayTest = MujocoTest;

TEST_F(RayTest, NoExclusions) {
  mjModel* model = LoadModelFromString(kRayCastingModel);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  mjtNum pnt[] = {0.0, 0.0, 0.0};
  mjtNum vec[] = {1.0, 0.0, 0.0};
  mjtByte* geomgroup = nullptr;
  mjtByte flg_static = 1;  // Include static geoms
  int bodyexclude = -1;
  int geomid = -1;

  mj_kinematics(model, data);
  mjtNum distance = mj_ray(model, data, pnt, vec, geomgroup, flg_static,
                           bodyexclude, &geomid);
  EXPECT_STREQ(mj_id2name(model, mjOBJ_GEOM, geomid), "static_group1");
  EXPECT_FLOAT_EQ(distance, 0.9);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(RayTest, Exclusions) {
  mjModel* model = LoadModelFromString(kRayCastingModel);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  mjtNum pnt[] = {0.0, 0.0, 0.0};
  mjtNum vec[] = {1.0, 0.0, 0.0};
  mjtByte geomgroup[] = {1, 1, 1};
  mjtByte flg_static = 1;
  int bodyexclude = -1;
  int geomid = -1;

  mj_kinematics(model, data);
  mjtNum distance = mj_ray(model, data, pnt, vec, geomgroup, flg_static,
                           bodyexclude, &geomid);
  EXPECT_STREQ(mj_id2name(model, mjOBJ_GEOM, geomid), "static_group1");
  EXPECT_FLOAT_EQ(distance, 0.9);

  // Exclude nearest geom
  geomgroup[1] = 0;
  distance = mj_ray(model, data, pnt, vec, geomgroup, flg_static, bodyexclude,
                    &geomid);
  EXPECT_STREQ(mj_id2name(model, mjOBJ_GEOM, geomid), "group0");
  EXPECT_FLOAT_EQ(distance, 2.9);

  geomgroup[0] = 0;
  distance = mj_ray(model, data, pnt, vec, geomgroup, flg_static, bodyexclude,
                    &geomid);
  EXPECT_STREQ(mj_id2name(model, mjOBJ_GEOM, geomid), "group2");
  EXPECT_FLOAT_EQ(distance, 4.9);

  geomgroup[2] = 0;
  distance = mj_ray(model, data, pnt, vec, geomgroup, flg_static, bodyexclude,
                    &geomid);
  EXPECT_EQ(geomid, -1);
  EXPECT_FLOAT_EQ(distance, -1);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(RayTest, ExcludeStatic) {
  mjModel* model = LoadModelFromString(kRayCastingModel);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  mjtNum pnt[] = {0.0, 0.0, 0.0};
  mjtNum vec[] = {1.0, 0.0, 0.0};
  mjtByte geomgroup[] = {1, 1, 1};
  mjtByte flg_static = 0;  // Exclude static geoms
  int bodyexclude = -1;
  int geomid = -1;

  mj_kinematics(model, data);
  mjtNum distance = mj_ray(model, data, pnt, vec, geomgroup, flg_static,
                           bodyexclude, &geomid);
  EXPECT_STREQ(mj_id2name(model, mjOBJ_GEOM, geomid), "group0");
  EXPECT_FLOAT_EQ(distance, 2.9);
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
