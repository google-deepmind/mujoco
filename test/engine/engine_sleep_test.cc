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

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_sleep.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using ::testing::IsNull;
using ::testing::HasSubstr;
using ::testing::NotNull;
using ::std::string;

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

TEST_F(SleepTest, MjWakeTree) {
  // one awake tree and two cycles
  int asleep[] = {kAwake, 2, 1, 3};
  EXPECT_EQ(mj_wakeTree(asleep, 4, 0, kAwake), 0);
  EXPECT_THAT(AsVector(asleep, 4), ElementsAre(kAwake, 2, 1, 3));
  EXPECT_EQ(mj_wakeTree(asleep, 4, 1, kAwake), 2);
  EXPECT_THAT(AsVector(asleep, 4),
              ElementsAre(kAwake, kAwake, kAwake, 3));
  EXPECT_EQ(mj_wakeTree(asleep, 4, 3, kAwake), 1);
  EXPECT_THAT(AsVector(asleep, 4),
              ElementsAre(kAwake, kAwake, kAwake, kAwake));
}

TEST_F(SleepTest, BadWakeTree) {
  EXPECT_FATAL_FAILURE(
      ([] {
        int asleep_bad1[] = {-1, 0};
        mj_wakeTree(asleep_bad1, 2, 1, kAwake);
      }()),
      "invalid sleep state index -1 when waking tree 1");

  EXPECT_FATAL_FAILURE(
      ([] {
        int asleep_bad2[] = {-1, 2};
        mj_wakeTree(asleep_bad2, 2, 1, kAwake);
      }()),
      "invalid sleep state index 2 when waking tree 1");

  EXPECT_FATAL_FAILURE(
      ([] {
        int asleep_bad3[] = {1, 2, 1};
        mj_wakeTree(asleep_bad3, 3, 0, kAwake);
      }()),
      "tree 0 is not in a cycle");
}

static const char* const kStaticModel = "engine/testdata/sleep/static.xml";
static const char* const kSmoothModel = "engine/testdata/sleep/smooth.xml";
static const char* const kInitModel = "engine/testdata/sleep/init.xml";
static const char* const kInitIslandModel =
    "engine/testdata/sleep/init_island.xml";
static const char* const kTendonModel = "engine/testdata/sleep/tendon.xml";
static const char* const kContactModel = "engine/testdata/sleep/contact.xml";
static const char* const kPairModel = "engine/testdata/sleep/contactpair.xml";
static const char* const kSensorModel = "engine/testdata/sleep/sensor.xml";

// roll out some models with sleeping enabled, valuable under ASAN and MSAN
TEST_F(SleepTest, KickTires) {
  for (const char* path :
       {kStaticModel, kInitModel, kInitIslandModel, kSensorModel, kTendonModel,
        kContactModel, kPairModel, kSmoothModel}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char error[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
    ASSERT_THAT(m, NotNull()) << error;

    int duration_id = mj_name2id(m, mjOBJ_NUMERIC, "duration");
    ASSERT_GE(duration_id, 0);
    mjtNum duration = m->numeric_data[m->numeric_adr[duration_id]];

    mjData* d = mj_makeData(m);
    while (d->time < duration) {
      mj_step(m, d);
    }

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

// Test that sleeping does not affect the simulation of awake trees:
// Roll out kSmoothModel, where all trees go to sleep within `duration` seconds
// in two mjData's, one with sleeping enabled and one without; expect the same
// values (for selected arrays) in awake trees in both.
TEST_F(SleepTest, WakingUnaffectedBySleeping) {
  const std::string xml_path = GetTestDataFilePath(kSmoothModel);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;

  int duration_id = mj_name2id(m, mjOBJ_NUMERIC, "duration");
  ASSERT_GE(duration_id, 0);
  mjtNum duration = m->numeric_data[m->numeric_adr[duration_id]];

  for (mjtJacobian jacobian : {mjJAC_DENSE, mjJAC_SPARSE}) {
    m->opt.jacobian = jacobian;
    for (mjtIntegrator integrator :  // TODO: b/457674312 - Add support for RK4.
        {mjINT_EULER, mjINT_IMPLICITFAST, mjINT_IMPLICIT}) {
      m->opt.integrator = integrator;

      // make data with sleeping enabled
      m->opt.enableflags |= mjENBL_SLEEP;
      mjData* d_sleep = mj_makeData(m);

      // make data with sleeping disabled
      m->opt.enableflags &= ~mjENBL_SLEEP;
      mjData* d_nosleep = mj_makeData(m);

      // disable constraints, contacts
      m->opt.disableflags |= mjDSBL_CONSTRAINT | mjDSBL_CONTACT;

      ASSERT_EQ(d_sleep->nbody_awake, m->nbody);
      int nbody_awake = -1;

      while (d_nosleep->time < duration) {
        m->opt.enableflags |= mjENBL_SLEEP;
        mj_step(m, d_sleep);
        m->opt.enableflags &= ~mjENBL_SLEEP;
        mj_step(m, d_nosleep);

        // if nbody_awake is not changed, skip
        if (d_sleep->nbody_awake == nbody_awake) {
          continue;
        }

        // compare xpos
        for (int i = 0; i < m->nbody; i++) {
          if (d_sleep->body_awake[i] == mjS_ASLEEP) continue;
          auto xpos1 = AsVector(d_nosleep->xpos + 3 * i, 3);
          auto xpos2 = AsVector(d_sleep->xpos + 3 * i, 3);
          EXPECT_EQ(xpos1, xpos2)
              << " xpos[" << i << "] at time " << d_nosleep->time;
        }

        // compare M and qLD
        for (int i = 0; i < d_sleep->nv_awake; i++) {
          int j = d_sleep->dof_awake_ind[i];
          auto M1 = AsVector(d_nosleep->M + m->M_rowadr[j], m->M_rownnz[j]);
          auto M2 = AsVector(d_sleep->M + m->M_rowadr[j], m->M_rownnz[j]);
          EXPECT_EQ(M1, M2) << " M[" << j << ",:] at time " << d_nosleep->time;
          auto qLD1 = AsVector(d_nosleep->qLD + m->M_rowadr[j], m->M_rownnz[j]);
          auto qLD2 = AsVector(d_sleep->qLD + m->M_rowadr[j], m->M_rownnz[j]);
          EXPECT_EQ(qLD1, qLD2)
              << " qLD[" << j << ",:] at time " << d_nosleep->time;
        }

        // compare cvel
        for (int i = 0; i < d_sleep->nbody_awake; i++) {
          if (d_sleep->body_awake[i] == mjS_ASLEEP) continue;
          auto cvel1 = AsVector(d_nosleep->cvel + 6 * i, 6);
          auto cvel2 = AsVector(d_sleep->cvel + 6 * i, 6);
          EXPECT_EQ(cvel1, cvel2)
              << " cvel[" << i << "] at time " << d_nosleep->time;
        }

        // compare subtree_angmom, only for dynamic bodies
        for (int i = 0; i < d_sleep->nbody_awake; i++) {
          if (d_sleep->body_awake[i] != mjS_AWAKE) continue;
          auto subtree_angmom1 = AsVector(d_nosleep->subtree_angmom + 3 * i, 3);
          auto subtree_angmom2 = AsVector(d_sleep->subtree_angmom + 3 * i, 3);
          EXPECT_EQ(subtree_angmom1, subtree_angmom2)
              << " subtree_angmom[" << i << "] at time " << d_nosleep->time;
        }

        // compare qfrc/qacc arrays
        for (int i = 0; i < d_sleep->nv_awake; i++) {
          int j = d_sleep->dof_awake_ind[i];
          EXPECT_EQ(d_nosleep->qfrc_smooth[j], d_sleep->qfrc_smooth[j])
              << " qfrc_smooth[" << j << "] at time " << d_nosleep->time;
          EXPECT_EQ(d_nosleep->qacc_smooth[j], d_sleep->qacc_smooth[j])
              << " qacc_smooth[" << j << "] at time " << d_nosleep->time;
          EXPECT_EQ(d_nosleep->qacc[j], d_sleep->qacc[j])
              << " qacc[" << j << "] at time " << d_nosleep->time;
        }

        nbody_awake = d_sleep->nbody_awake;
      }

      mj_deleteData(d_sleep);
      mj_deleteData(d_nosleep);
    }
  }
  mj_deleteModel(m);
}


// Test that waking does not affect sleeping trees for pos/vel-dependent arrays.
// Roll out models where some trees wake and/or sleep. At kCompare intervals,
// copy the state from the mjData with sleeping enabled to another mjData and
// call mj_forward with sleeping disabled. Expect pos/vel-dependent arrays to be
// unchanged for all trees and frc/acc-dependent arrays to be the same for awake
// trees.
TEST_F(SleepTest, SleepingUnaffectedByWaking) {
  for (const char* path : {kInitModel, kInitIslandModel, kTendonModel,
                           kContactModel, kSensorModel, kSmoothModel}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char error[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
    ASSERT_THAT(m, NotNull()) << error;

    const int kCompare = 10;  // number of comparisons per rollout

    // TODO: b/457674312 - Add support for RK4.
    for (mjtIntegrator integrator :
         {mjINT_EULER, mjINT_IMPLICITFAST, mjINT_IMPLICIT}) {
      m->opt.integrator = integrator;

      // make data with sleeping enabled
      m->opt.enableflags |= mjENBL_SLEEP;
      mjData* d_sleep = mj_makeData(m);

      // make data with sleeping disabled
      m->opt.enableflags &= ~mjENBL_SLEEP;
      mjData* d_nosleep = mj_makeData(m);

      int duration_id = mj_name2id(m, mjOBJ_NUMERIC, "duration");
      ASSERT_GE(duration_id, 0);
      mjtNum duration = m->numeric_data[m->numeric_adr[duration_id]];

      int compare_interval = duration / (m->opt.timestep * kCompare);
      int nsteps = 0;
      while (d_sleep->time < duration) {
        // step d_sleep with sleeping enabled
        m->opt.enableflags |= mjENBL_SLEEP;
        mj_step(m, d_sleep);
        nsteps++;

        // every compare_interval steps, compare with d_nosleep
        if (nsteps % compare_interval != 0) {
          continue;
        }

        // call mj_forward to update d_sleep
        mj_forward(m, d_sleep);

        // copy state from d_sleep to d_nosleep
        mj_copyData(d_nosleep, m, d_sleep);

        // forward d_nosleep with sleeping disabled
        m->opt.enableflags &= ~mjENBL_SLEEP;
        mj_forward(m, d_nosleep);

        // ==== compare arrays for all dofs / bodies / sensors ====

        // compare xpos
        for (int i = 0; i < m->nbody; i++) {
          auto xpos1 = AsVector(d_sleep->xpos + 3 * i, 3);
          auto xpos2 = AsVector(d_nosleep->xpos + 3 * i, 3);
          EXPECT_EQ(xpos1, xpos2)
              << " xpos[" << i << "] at time " << d_sleep->time;
        }

        // compare M and qLD
        for (int i = 0; i < m->nv; i++) {
          auto M1 = AsVector(d_sleep->M + m->M_rowadr[i], m->M_rownnz[i]);
          auto M2 = AsVector(d_nosleep->M + m->M_rowadr[i], m->M_rownnz[i]);
          EXPECT_EQ(M1, M2) << " M[" << i << ",:] at time " << d_sleep->time;
          auto qLD1 = AsVector(d_sleep->qLD + m->M_rowadr[i], m->M_rownnz[i]);
          auto qLD2 = AsVector(d_nosleep->qLD + m->M_rowadr[i], m->M_rownnz[i]);
          EXPECT_EQ(qLD1, qLD2)
              << " qLD[" << i << ",:] at time " << d_sleep->time;
        }

        // compare cvel
        for (int i = 0; i < m->nbody; i++) {
          auto cvel1 = AsVector(d_sleep->cvel + 6 * i, 6);
          auto cvel2 = AsVector(d_nosleep->cvel + 6 * i, 6);
          EXPECT_EQ(cvel1, cvel2)
              << " cvel[" << i << "] at time " << d_sleep->time;
        }

        // compare qfrc arrays
        for (int i = 0; i < m->nv; i++) {
          EXPECT_EQ(d_sleep->qfrc_fluid[i], d_nosleep->qfrc_fluid[i])
              << " qfrc_fluid[" << i << "] at time " << d_sleep->time;
          EXPECT_EQ(d_sleep->qfrc_damper[i], d_nosleep->qfrc_damper[i])
              << " qfrc_damper[" << i << "] at time " << d_sleep->time;
          EXPECT_EQ(d_sleep->qfrc_spring[i], d_nosleep->qfrc_spring[i])
              << " qfrc_spring[" << i << "] at time " << d_sleep->time;
          EXPECT_EQ(d_sleep->qfrc_gravcomp[i], d_nosleep->qfrc_gravcomp[i])
              << " qfrc_gravcomp[" << i << "] at time " << d_sleep->time;
          EXPECT_EQ(d_sleep->qfrc_bias[i], d_nosleep->qfrc_bias[i])
              << " qfrc_bias[" << i << "] at time " << d_sleep->time;
        }

        // compare sensordata
        for (int i = 0; i < m->nsensor; i++) {
          int dim = m->sensor_dim[i];
          int adr = m->sensor_adr[i];
          auto data1 = AsVector(d_sleep->sensordata + adr, dim);
          auto data2 = AsVector(d_nosleep->sensordata + adr, dim);
          EXPECT_EQ(data1, data2)
              << " sensor " << i << " at time " << d_sleep->time;
        }

        // ==== compare arrays for awake dofs only ====

        // compare qacc arrays for awake dofs
        for (int j = 0; j < d_sleep->nv_awake; j++) {
          int i = d_sleep->dof_awake_ind[j];
          EXPECT_EQ(d_sleep->qacc_smooth[i], d_nosleep->qacc_smooth[i])
              << " qacc_smooth[" << i << "] at time " << d_sleep->time;
          EXPECT_EQ(d_sleep->qacc[i], d_nosleep->qacc[i])
              << " qacc[" << i << "] at time " << d_sleep->time;
        }
      }

      mj_deleteData(d_nosleep);
      mj_deleteData(d_sleep);
    }
    mj_deleteModel(m);
  }
}

static const char* const kEqualityModel = "engine/testdata/sleep/equality.xml";

// Activate equality between sleeping and awake trees, useful under ASAN/MSAN.
TEST_F(SleepTest, Equality) {
  const std::string xml_path = GetTestDataFilePath(kEqualityModel);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;

  mjData* d = mj_makeData(m);
  while (d->ntree_awake == m->ntree) {
    mj_step(m, d);
  }

  int dd = mj_name2id(m, mjOBJ_EQUALITY, "dyn/dyn");
  ASSERT_GE(dd, 0);

  mj_step(m, d);
  d->eq_active[dd] = 1;
  mj_step(m, d);

  mj_deleteData(d);
  mj_deleteModel(m);
}

static const char* const kInitIslandFailModel =
    "engine/testdata/sleep/init_island_fail.xml";

TEST_F(SleepTest, InitIslandFail) {
  const std::string xml_path = GetTestDataFilePath(kInitIslandFailModel);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error,
              HasSubstr("3 trees were marked as sleep='init' but only 0 could "
                        "be slept.\nBody 'asleep_init0' (id=1) is the root of "
                        "the first tree that could not be slept."));
}

}  // namespace
}  // namespace mujoco
