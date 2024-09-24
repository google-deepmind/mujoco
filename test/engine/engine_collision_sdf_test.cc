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

// Tests for sdf collisions.

#include <cstdio>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_collision_sdf.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using SdfTest = MujocoTest;

static constexpr int kpoints = 6;
static constexpr int kgeoms = 5;
static constexpr char kSdfModel[] = R"(
<mujoco>
  <worldbody>
    <geom type="plane" size="5 5 .1" pos="0 0 -1"/>
    <body pos="-.1 .2 2" euler="0 0 45">
      <geom type="sphere" size="1"/>
      <geom type="capsule" size=".1" fromto="-2 -2 1 2 2 1"/>
      <geom type="cylinder" size="1" fromto="0 0 0 2 0 0"/>
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>
)";

TEST_F(SdfTest, SdfPrimitive) {
  mjModel* model = LoadModelFromString(kSdfModel);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());
  ASSERT_THAT(model->ngeom, kgeoms);

  mjSDF sdf;
  const mjpPlugin* null_plugin = NULL;
  mjtNum gradient[3], dist[kgeoms][kpoints] = {
    {0, 0, 0, 0, 1, 1},  // plane
    {-1, 0, 0, mju_sqrt(2)-1, mju_sqrt(2)-1, mju_sqrt(3)-1},  // sphere
    {-.1, .9, .9, mju_sqrt(2)-.1, .9, mju_sqrt(2)-.1},  // capsule
    {-1, 0, 0, mju_sqrt(2)-1, 0, mju_sqrt(2)-1},  // cylinder
    {-1, 0, 0, 0, 0, 0},  // box
  };
  mjtNum points[kpoints][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                               {1, 1, 0}, {0, 1, 1}, {1, 1, 1}};

  for (int i = 0; i < kgeoms; i++) {
    sdf.plugin = &null_plugin;
    sdf.id = &i;
    sdf.type = mjSDFTYPE_SINGLE;
    sdf.geomtype = (mjtGeom*)(model->geom_type+i);
    for (int j = 0; j < kpoints; j++) {
      ASSERT_THAT(mjc_distance(model, data, &sdf, points[j]), dist[i][j]);
      mjc_gradient(model, data, &sdf, gradient, points[j]);
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
