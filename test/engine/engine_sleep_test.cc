// Copyright 2025 DeepMind Technologies Limited
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

// Tests for engine/engine_sleep.c.

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_sleep.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using ::testing::NotNull;

using SleepTest = MujocoTest;

static constexpr char kSimple[] = R"(
<mujoco>
  <option>
    <flag constraint="disable" contact="disable"/>
  </option>

  <default>
    <geom size="1"/>
  </default>

  <worldbody>
    <body>
      <joint type="ball"/>
      <geom/>
    </body>

    <body>
      <geom/>
    </body>

    <geom/>

    <body>
      <joint/>
      <geom/>
      <geom/>
      <body pos="1 0 0">
        <joint/>
        <geom/>
      </body>
    </body>
  </worldbody>
</mujoco>
)";

static constexpr int kAwake = -(1+mjMINAWAKE);

TEST_F(SleepTest, MjSleepUpdate) {
  char error[1024];
  mjModel* m = LoadModelFromString(kSimple, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  // ntree = 2, nbody = 5, nv = 5, njnt = 3, ngeom = 6
  // body 0: world, 1 geom
  // body 1: 1 ball join, 3 dofs, 1 geom
  // body 2: no joint, 1 geom
  // body 3: hinge joint, 1 dof, 2 geoms
  // body 4: child of body 2, hinge joint, 1 dof, 1 geom

  EXPECT_EQ(m->ntree, 2);
  EXPECT_EQ(m->nbody, 5);
  EXPECT_EQ(m->nv, 5);
  EXPECT_EQ(m->njnt, 3);
  EXPECT_EQ(m->ngeom, 6);

  EXPECT_THAT(AsVector(m->body_treeid, m->nbody),
              ElementsAre(-1, 0, -1, 1, 1));
  EXPECT_THAT(AsVector(m->dof_bodyid, m->nv),
              ElementsAre(1, 1, 1, 3, 4));
  EXPECT_THAT(AsVector(m->geom_bodyid, m->ngeom),
              ElementsAre(0, 1, 2, 3, 3, 4));
  EXPECT_THAT(AsVector(m->jnt_bodyid, m->njnt),
              ElementsAre(1, 3, 4));

  // Test Case 1: Initial state
  EXPECT_THAT(AsVector(d->tree_asleep, m->ntree),
              ElementsAre(kAwake, kAwake));
  EXPECT_EQ(d->ntree_awake, 2);
  EXPECT_EQ(d->nv_awake, 5);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre(0, 1, 2, 3, 4));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree),
              ElementsAre(1, 1));
  EXPECT_THAT(AsVector(d->body_awake, m->nbody),
              ElementsAre(mjS_STATIC,
                          mjS_AWAKE,
                          mjS_STATIC,
                          mjS_AWAKE,
                          mjS_AWAKE));

  // Test Case 2: Call mj_sleepUpdate, expect no changes
  mj_updateSleep(m, d);
  EXPECT_EQ(d->ntree_awake, 2);
  EXPECT_EQ(d->nv_awake, 5);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre(0, 1, 2, 3, 4));
  EXPECT_THAT(AsVector(d->body_awake, m->nbody),
              ElementsAre(mjS_STATIC,
                          mjS_AWAKE,
                          mjS_STATIC,
                          mjS_AWAKE,
                          mjS_AWAKE));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree),
              ElementsAre(1, 1));

  // Test Case 3: Tree 0 asleep
  d->tree_asleep[0] = 0; d->tree_asleep[1] = -1;
  mj_updateSleep(m, d);
  EXPECT_EQ(d->ntree_awake, 1);
  EXPECT_EQ(d->nv_awake, 2);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre(3, 4));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree),
              ElementsAre(0, 1));
  EXPECT_THAT(AsVector(d->body_awake, m->nbody),
              ElementsAre(mjS_STATIC,
                          mjS_ASLEEP,
                          mjS_STATIC,
                          mjS_AWAKE,
                          mjS_AWAKE));

  // Test Case 4: Tree 1 asleep
  d->tree_asleep[0] = -1; d->tree_asleep[1] = 1;
  mj_updateSleep(m, d);
  EXPECT_EQ(d->ntree_awake, 1);
  EXPECT_EQ(d->nv_awake, 3);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre(0, 1, 2));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree),
              ElementsAre(1, 0));
  EXPECT_THAT(AsVector(d->body_awake, m->nbody),
              ElementsAre(mjS_STATIC,
                          mjS_AWAKE,
                          mjS_STATIC,
                          mjS_ASLEEP,
                          mjS_ASLEEP));

  // Test Case 5: All trees asleep
  d->tree_asleep[0] = 0; d->tree_asleep[1] = 1;
  mj_updateSleep(m, d);
  EXPECT_EQ(d->ntree_awake, 0);
  EXPECT_EQ(d->nv_awake, 0);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre());
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree),
              ElementsAre(0, 0));
  EXPECT_THAT(AsVector(d->body_awake, m->nbody),
              ElementsAre(mjS_STATIC,
                          mjS_ASLEEP,
                          mjS_STATIC,
                          mjS_ASLEEP,
                          mjS_ASLEEP));

  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
