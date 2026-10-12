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

#include "src/engine/engine_sleep.h"

#include <cmath>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::string;
using ::std::vector;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
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

static constexpr int kAwake = -(1 + mjMINAWAKE);

TEST_F(SleepTest, MjSleepUpdate) {
  char error[1024];
  MjModelPtr m = LoadModelFromString(kSimple, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

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

  EXPECT_THAT(AsVector(m->body_treeid, m->nbody), ElementsAre(-1, 0, -1, 1, 1));
  EXPECT_THAT(AsVector(m->dof_bodyid, m->nv), ElementsAre(1, 1, 1, 3, 4));
  EXPECT_THAT(AsVector(m->geom_bodyid, m->ngeom),
              ElementsAre(0, 1, 2, 3, 3, 4));
  EXPECT_THAT(AsVector(m->jnt_bodyid, m->njnt), ElementsAre(1, 3, 4));

  // Test Case 1: Initial state
  EXPECT_THAT(AsVector(d->tree_asleep, m->ntree), ElementsAre(kAwake, kAwake));
  EXPECT_EQ(d->ntree_awake, 2);
  EXPECT_EQ(d->nv_awake, 5);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre(0, 1, 2, 3, 4));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree), ElementsAre(1, 1));
  EXPECT_THAT(
      AsVector(d->body_awake, m->nbody),
      ElementsAre(mjS_STATIC, mjS_AWAKE, mjS_STATIC, mjS_AWAKE, mjS_AWAKE));

  // Test Case 2: Call mj_sleepUpdate, expect no changes
  mj_updateSleep(m.get(), d.get());
  EXPECT_EQ(d->ntree_awake, 2);
  EXPECT_EQ(d->nv_awake, 5);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake),
              ElementsAre(0, 1, 2, 3, 4));
  EXPECT_THAT(
      AsVector(d->body_awake, m->nbody),
      ElementsAre(mjS_STATIC, mjS_AWAKE, mjS_STATIC, mjS_AWAKE, mjS_AWAKE));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree), ElementsAre(1, 1));

  // Test Case 3: Tree 0 asleep
  d->tree_asleep[0] = 0;
  d->tree_asleep[1] = -1;
  mj_updateSleep(m.get(), d.get());
  EXPECT_EQ(d->ntree_awake, 1);
  EXPECT_EQ(d->nv_awake, 2);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake), ElementsAre(3, 4));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree), ElementsAre(0, 1));
  EXPECT_THAT(
      AsVector(d->body_awake, m->nbody),
      ElementsAre(mjS_STATIC, mjS_ASLEEP, mjS_STATIC, mjS_AWAKE, mjS_AWAKE));

  // Test Case 4: Tree 1 asleep
  d->tree_asleep[0] = -1;
  d->tree_asleep[1] = 1;
  mj_updateSleep(m.get(), d.get());
  EXPECT_EQ(d->ntree_awake, 1);
  EXPECT_EQ(d->nv_awake, 3);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake), ElementsAre(0, 1, 2));
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree), ElementsAre(1, 0));
  EXPECT_THAT(
      AsVector(d->body_awake, m->nbody),
      ElementsAre(mjS_STATIC, mjS_AWAKE, mjS_STATIC, mjS_ASLEEP, mjS_ASLEEP));

  // Test Case 5: All trees asleep
  d->tree_asleep[0] = 0;
  d->tree_asleep[1] = 1;
  mj_updateSleep(m.get(), d.get());
  EXPECT_EQ(d->ntree_awake, 0);
  EXPECT_EQ(d->nv_awake, 0);
  EXPECT_THAT(AsVector(d->dof_awake_ind, d->nv_awake), ElementsAre());
  EXPECT_THAT(AsVector(d->tree_awake, m->ntree), ElementsAre(0, 0));
  EXPECT_THAT(
      AsVector(d->body_awake, m->nbody),
      ElementsAre(mjS_STATIC, mjS_ASLEEP, mjS_STATIC, mjS_ASLEEP, mjS_ASLEEP));
}

TEST_F(SleepTest, MjWakeIsland) {
  // one awake tree and two cycles
  int asleep[] = {kAwake, 2, 1, 3};
  EXPECT_EQ(mj_wakeIsland(asleep, 4, 0, kAwake, nullptr, 0), 0);
  EXPECT_THAT(AsVector(asleep, 4), ElementsAre(kAwake, 2, 1, 3));
  EXPECT_EQ(mj_wakeIsland(asleep, 4, 1, kAwake, nullptr, 0), 2);
  EXPECT_THAT(AsVector(asleep, 4), ElementsAre(kAwake, kAwake, kAwake, 3));
  EXPECT_EQ(mj_wakeIsland(asleep, 4, 3, kAwake, nullptr, 0), 1);
  EXPECT_THAT(AsVector(asleep, 4), ElementsAre(kAwake, kAwake, kAwake, kAwake));
}

TEST_F(SleepTest, BadWakeIsland) {
  EXPECT_FATAL_FAILURE(([] {
                         int asleep_bad1[] = {-1, 0};
                         mj_wakeIsland(asleep_bad1, 2, 1, kAwake, nullptr, 0);
                       }()),
                       "invalid sleep state index -1 when waking tree 1");

  EXPECT_FATAL_FAILURE(([] {
                         int asleep_bad2[] = {-1, 2};
                         mj_wakeIsland(asleep_bad2, 2, 1, kAwake, nullptr, 0);
                       }()),
                       "invalid sleep state index 2 when waking tree 1");

  EXPECT_FATAL_FAILURE(([] {
                         int asleep_bad3[] = {1, 2, 1};
                         mj_wakeIsland(asleep_bad3, 3, 0, kAwake, nullptr, 0);
                       }()),
                       "tree 0 is not in a cycle");
}

static const char* const kStaticModel = "engine/testdata/sleep/static.xml";
static const char* const kMocapcModel = "engine/testdata/sleep/mocap.xml";
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
       {kStaticModel, kMocapcModel, kInitModel, kInitIslandModel, kSensorModel,
        kTendonModel, kContactModel, kPairModel, kSmoothModel}) {
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
         {mjINT_EULER, mjINT_IMPLICITFAST, mjINT_IMPLICIT, mjINT_DISCRETE}) {
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
  for (const char* path :
       {kInitModel, kMocapcModel, kInitIslandModel, kTendonModel, kContactModel,
        kSensorModel, kSmoothModel}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char error[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
    ASSERT_THAT(m, NotNull()) << error;

    const int kCompare = 10;  // number of comparisons per rollout

    // for some sensors, the comparison is expected to fail (at least once)
    vector<bool> sensor_mismatch(m->nsensor, false);

    // TODO: b/457674312 - Add support for RK4.
    for (mjtIntegrator integrator :
         {mjINT_EULER, mjINT_IMPLICITFAST, mjINT_IMPLICIT, mjINT_DISCRETE}) {
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
          if (m->nuser_sensor == 1 && m->sensor_user[i] == 1) {
            // user=1 means sensor value cannot be determined at sleep time
            sensor_mismatch[i] = sensor_mismatch[i] || (data1 != data2);
            EXPECT_EQ(mj_sleepState(m, d_sleep, mjOBJ_SENSOR, i), mjS_AWAKE);
          } else {
            // otherwise expect perfect match
            EXPECT_EQ(data1, data2)
                << " sensor " << i << " at time " << d_sleep->time;
          }
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

      for (int i = 0; i < m->nsensor; i++) {
        if (m->nuser_sensor == 1 && m->sensor_user[i] == 1) {
          EXPECT_TRUE(sensor_mismatch[i])
              << "contact sensor " << i << " comparison was expected to fail";
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

// Test that the free-body implicit (gyroscopic) solve doesn't break the sleep
// qvel=0 invariant. A standalone free body with high viscosity should
// eventually go to sleep, and after sleeping, qvel/qacc must be exactly zero.
TEST_F(SleepTest, FreeBodySleepZeroVelocity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast" viscosity="10"
            sleep_tolerance="0.01">
      <flag sleep="enable" gravity="disable" constraint="disable"
            contact="disable"/>
    </option>
    <worldbody>
      <body>
        <freejoint/>
        <geom type="box" size=".1 .2 .3" mass="1" euler="10 20 30"
              pos=".03 .02 .01"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // give initial velocity (both translational and angular)
  d->qvel[0] = 0.5;
  d->qvel[1] = 0.5;
  d->qvel[2] = 0.5;
  d->qvel[3] = 1.0;
  d->qvel[4] = 2.0;
  d->qvel[5] = 3.0;

  // step until body goes to sleep
  for (int step = 0; step < 1000; step++) {
    mj_step(m.get(), d.get());
    if (d->ntree_awake == 0) break;
  }

  // body should have gone to sleep
  ASSERT_EQ(d->ntree_awake, 0) << "body did not go to sleep";

  // qvel and qacc must be exactly zero for sleeping body
  for (int i = 0; i < 6; i++) {
    EXPECT_EQ(d->qvel[i], 0.0) << "qvel[" << i << "] not zero after sleep";
    EXPECT_EQ(d->qacc[i], 0.0) << "qacc[" << i << "] not zero after sleep";
  }
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

// Test that a constrained flex eventually goes to sleep.
TEST_F(SleepTest, FlexEdgeSleep) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/flex_edge.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_GT(m->flex_edgeequality[0], 0);
  mjData* d = mj_makeData(m);

  // give initial velocity
  d->qvel[0] = 0.1;

  // step until body goes to sleep
  for (int step = 0; step < 2000; step++) {
    mj_step(m, d);
    if (d->ntree_awake == 0) break;
  }

  EXPECT_EQ(d->ntree_awake, 0) << "flex did not go to sleep";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test that a constraint-free flex never sleeps.
TEST_F(SleepTest, FlexNoConstraintNeverSleeps) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/flex_nocnstr.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_EQ(m->flex_edgeequality[0], 0);
  mjData* d = mj_makeData(m);

  // step for a while
  for (int step = 0; step < 500; step++) {
    mj_step(m, d);
  }

  // tree should have AUTO_NEVER policy
  EXPECT_EQ(m->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_GT(d->ntree_awake, 0) << "constraint-free flex should not sleep";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test that mj_sleepState returns correct values for flex objects.
TEST_F(SleepTest, FlexSleepState) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/flex_state.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  // initially awake
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, 0), mjS_AWAKE);

  // step until flex goes to sleep
  for (int step = 0; step < 1000; step++) {
    mj_step(m, d);
    if (d->ntree_awake == 0) break;
  }

  // flex should be asleep
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, 0), mjS_ASLEEP);

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test that a sleeping flex wakes on contact with a falling ball,
// then both go back to sleep after the ball rolls off.
TEST_F(SleepTest, FlexWakeContact) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/flex_contact.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  int flexid = mj_name2id(m, mjOBJ_FLEX, "f1");
  ASSERT_GE(flexid, 0);

  // phase 1: flex settles and goes to sleep
  for (int step = 0; step < 5000; step++) {
    mj_step(m, d);
    if (mj_sleepState(m, d, mjOBJ_FLEX, flexid) == mjS_ASLEEP) break;
  }
  ASSERT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, flexid), mjS_ASLEEP)
      << "flex did not go to sleep";

  // phase 2: ball hits flex, flex wakes up
  for (int step = 0; step < 5000; step++) {
    mj_step(m, d);
    if (mj_sleepState(m, d, mjOBJ_FLEX, flexid) == mjS_AWAKE) break;
  }
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, flexid), mjS_AWAKE)
      << "flex should have been woken by ball contact";

  // phase 3: ball rolls off, everything goes back to sleep
  for (int step = 0; step < 10000; step++) {
    mj_step(m, d);
    if (d->ntree_awake == 0) break;
  }
  EXPECT_EQ(d->ntree_awake, 0) << "all trees should be asleep again";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test full sleep/wake lifecycle with two grippers and two flex objects.
TEST_F(SleepTest, HollowVsSolidSleep) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/hollow_vs_solid.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  // look up flex IDs
  int solid_flex = mj_name2id(m, mjOBJ_FLEX, "soft_mesh");
  int hollow_flex = mj_name2id(m, mjOBJ_FLEX, "soft_mesh_2");
  ASSERT_GE(solid_flex, 0);
  ASSERT_GE(hollow_flex, 0);

  // look up actuator IDs
  int grasp_r = mj_name2id(m, mjOBJ_ACTUATOR, "grasp_r");
  int grasp_s = mj_name2id(m, mjOBJ_ACTUATOR, "grasp_s");
  ASSERT_GE(grasp_r, 0);
  ASSERT_GE(grasp_s, 0);

  // phase 1: everything starts awake
  mj_forward(m, d);
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, solid_flex), mjS_AWAKE);
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, hollow_flex), mjS_AWAKE);

  // phase 2: both flexes go to sleep within 500 steps
  for (int step = 0; step < 500; step++) {
    mj_step(m, d);
  }
  ASSERT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, solid_flex), mjS_ASLEEP)
      << "solid flex did not go to sleep";
  ASSERT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, hollow_flex), mjS_ASLEEP)
      << "hollow flex did not go to sleep";

  // phase 3: close gripper_solid (grasp_r), it wakes soft_mesh_2 (hollow)
  d->ctrl[grasp_r] = 1;
  for (int step = 0; step < 2000; step++) {
    mj_step(m, d);
    if (mj_sleepState(m, d, mjOBJ_FLEX, hollow_flex) == mjS_AWAKE) break;
  }
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, hollow_flex), mjS_AWAKE)
      << "hollow flex should be woken by gripper_solid";
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, solid_flex), mjS_ASLEEP)
      << "solid flex should still be asleep";

  // phase 4: close gripper_hollow (grasp_s), it wakes soft_mesh (solid)
  d->ctrl[grasp_s] = 1;
  for (int step = 0; step < 2000; step++) {
    mj_step(m, d);
    if (mj_sleepState(m, d, mjOBJ_FLEX, solid_flex) == mjS_AWAKE) break;
  }
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, solid_flex), mjS_AWAKE)
      << "solid flex should be woken by gripper_hollow";

  // phase 5: open gripper_solid (grasp_r=0), hollow flex goes back to sleep
  d->ctrl[grasp_r] = 0;
  for (int step = 0; step < 2000; step++) {
    mj_step(m, d);
    if (mj_sleepState(m, d, mjOBJ_FLEX, hollow_flex) == mjS_ASLEEP) break;
  }
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, hollow_flex), mjS_ASLEEP)
      << "hollow flex should go back to sleep after gripper release";

  // phase 6: open gripper_hollow (grasp_s=0), solid flex goes back to sleep
  d->ctrl[grasp_s] = 0;
  for (int step = 0; step < 2000; step++) {
    mj_step(m, d);
    if (mj_sleepState(m, d, mjOBJ_FLEX, solid_flex) == mjS_ASLEEP) break;
  }
  EXPECT_EQ(mj_sleepState(m, d, mjOBJ_FLEX, solid_flex), mjS_ASLEEP)
      << "solid flex should go back to sleep after gripper release";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test that a sleeping flex touching a non-world static body doesn't crash.
TEST_F(SleepTest, FlexStaticContact) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/flex_static.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  // step until flex goes to sleep, should not throw
  for (int step = 0; step < 1000; step++) {
    mj_step(m, d);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test that a sleeping flex near a mocap body doesn't crash.
TEST_F(SleepTest, FlexMocapContact) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/sleep/flex_mocap.xml");
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  for (int step = 0; step < 200; step++) {
    mj_step(m, d);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// mocap bodies are their own weld root: dragging into a sleeping tree wakes it
TEST_F(SleepTest, MocapWakes) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <geom type="plane" size="5 5 .1"/>
      <body name="hand" mocap="true" pos="2 0 .1">
        <geom type="sphere" size=".1"/>
        <body name="finger">
          <geom type="sphere" size=".05" pos=".1 0 0"/>
        </body>
      </body>
      <body name="box" pos="0 0 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  // mocap body is its own weld root, static child inherits it,
  // box is its own (jointed)
  int hand = mj_name2id(m.get(), mjOBJ_BODY, "hand");
  int finger = mj_name2id(m.get(), mjOBJ_BODY, "finger");
  EXPECT_EQ(m->body_weldid[hand], hand);
  EXPECT_EQ(m->body_weldid[finger], hand);

  // let the box fall asleep
  MjDataPtr d = MakeData(m);
  int steps = 0;
  while (d->ntree_awake > 0 && steps++ < 10000) {
    mj_step(m.get(), d.get());
  }
  ASSERT_EQ(d->ntree_awake, 0);

  // drag the mocap hand into the sleeping box: box wakes
  d->mocap_pos[0] = 0;
  mj_step(m.get(), d.get());
  EXPECT_EQ(d->ntree_awake, 1);
}

// mocap bodies do not generate contacts with static bodies or each other
TEST_F(SleepTest, MocapNoStaticContacts) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="5 5 .1"/>
      <body mocap="true" pos="0 0 .05">
        <geom type="sphere" size=".1"/>
      </body>
      <body mocap="true" pos=".05 0 .05">
        <geom type="sphere" size=".1"/>
      </body>
      <geom type="box" size=".1 .1 .1" pos="-.05 0 .05"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // everything interpenetrates, nothing can move: no contacts
  mj_forward(m.get(), d.get());
  EXPECT_EQ(d->ncon, 0);
}

// a plane on a mocap body collides like a plane on a static body
TEST_F(SleepTest, MocapPlane) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body mocap="true">
        <geom type="plane" size="5 5 .1"/>
      </body>
      <body pos="0 0 .5">
        <freejoint/>
        <geom type="box" size=".1 .1 .1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // box falls onto the mocap plane and rests on it
  while (d->time < 2) {
    mj_step(m.get(), d.get());
  }
  EXPECT_GT(d->ncon, 0);
  EXPECT_NEAR(d->qpos[2], 0.1, 1e-3);
}

// a tree welded by equality to a mocap body is woken when asleep
TEST_F(SleepTest, MocapWeldEqualityWakes) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <geom type="plane" size="5 5 .1"/>
      <body name="target" mocap="true" pos="2 0 .5">
        <geom type="sphere" size=".05" contype="0" conaffinity="0"/>
      </body>
      <body name="box" pos="0 0 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1"/>
      </body>
    </worldbody>
    <equality>
      <weld name="grab" body1="target" body2="box" active="false"/>
    </equality>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // equality inactive: box falls asleep on the floor
  int steps = 0;
  while (d->ntree_awake > 0 && steps++ < 10000) {
    mj_step(m.get(), d.get());
  }
  ASSERT_EQ(d->ntree_awake, 0);

  // activate the weld to the mocap body: box wakes
  d->eq_active[mj_name2id(m.get(), mjOBJ_EQUALITY, "grab")] = 1;
  mj_step(m.get(), d.get());
  EXPECT_EQ(d->ntree_awake, 1);
}

// ---------------------------------------------------------------------------
// Flex contacts and the island invariant (sleeping trees never in an island).
//
// A flex element contact's constraint row spans every vertex of the element.
// The wake sweep must therefore treat a flex contact side as all of those
// bodies, and the wake sources (contacts, equalities, recollision) must be
// iterated to a fixed point: a contact wakes one vertex, the flex's edge
// equalities wake the rest, and only then do the flex's other contacts
// (with bodies that were asleep under it, invisible to the broadphase while
// both slept) count and wake those bodies.
// ---------------------------------------------------------------------------

// 3x3 sheet lying on the floor, first row pinned to the world: 18 of its 48
// triangles have a pinned FIRST vertex
static constexpr char kPinnedSheet[] = R"(
<mujoco>
  <option timestep="0.002"><flag sleep="enable"/></option>
  <worldbody>
    <geom type="plane" size="2 2 .1"/>
    <flexcomp name="sheet" type="grid" count="3 3 1" spacing=".1 .1 .1"
              pos="0 0 .006" radius=".005" dim="2" mass="0.2">
      <pin id="0 1 2"/>
      <edge equality="true" damping="20"/>
    </flexcomp>
    <body name="box" pos="1 1 .05">
      <freejoint/>
      <geom type="box" size=".02 .02 .02" mass=".05"/>
    </body>
  </worldbody>
</mujoco>
)";

// the pinned sheet with a small box resting on it, far from the drop point
static constexpr char kPinnedSheetWithRester[] = R"(
<mujoco>
  <option timestep="0.002"><flag sleep="enable"/></option>
  <worldbody>
    <geom type="plane" size="2 2 .1"/>
    <flexcomp name="sheet" type="grid" count="3 3 1" spacing=".1 .1 .1"
              pos="0 0 .006" radius=".005" dim="2" mass="0.2">
      <pin id="0 1 2"/>
      <edge equality="true" damping="20"/>
    </flexcomp>
    <body name="rester" pos="0.08 0.08 .04">
      <freejoint/>
      <geom type="box" size=".015 .015 .015" mass=".02"/>
    </body>
    <body name="box" pos="1 1 .05">
      <freejoint/>
      <geom type="box" size=".02 .02 .02" mass=".05"/>
    </body>
  </worldbody>
</mujoco>
)";

// quadratic interpolated flex: 27 node bodies drive 64 vertices; any contact
// with one of its elements couples all 27 nodes
static constexpr char kInterpolatedCube[] = R"(
<mujoco>
  <option timestep="0.002"><flag sleep="enable"/></option>
  <worldbody>
    <geom type="plane" size="2 2 .1"/>
    <flexcomp name="cube" type="grid" count="4 4 4" spacing=".05 .05 .05"
              pos="0 0 .08" radius=".005" dim="3" mass="0.5" dof="quadratic">
      <pin id="0 1 2 3"/>
      <edge equality="true" damping="20"/>
      <contact selfcollide="none"/>
    </flexcomp>
    <body name="box" pos="1 1 .05">
      <freejoint/>
      <geom type="box" size=".02 .02 .02" mass=".05"/>
    </body>
  </worldbody>
</mujoco>
)";

// 6 s: the flex is asleep long before
static constexpr int kSettleSteps = 3000;

// 1.2 s: covers a 0.4 m fall and the landing
static constexpr int kDropSteps = 600;

// m above the target; the flex re-sleeps during the fall
static constexpr mjtNum kDropHeight = 0.4;

// teleport the free body `body` to `pos` at rest through mj_setState (which
// wakes it)
static void TeleportFreeBody(const mjModel* m, mjData* d, int body,
                             const mjtNum pos[3]) {
  int qpos_adr = m->jnt_qposadr[m->body_jntadr[body]];
  int dof_adr = m->jnt_dofadr[m->body_jntadr[body]];
  int spec = mjSTATE_FULLPHYSICS;
  std::vector<mjtNum> state(mj_stateSize(m, spec));
  mj_getState(m, d, state.data(), spec);
  // FULLPHYSICS = time, qpos, qvel, act
  for (int i = 0; i < 3; i++) state[1 + qpos_adr + i] = pos[i];
  for (int i = 0; i < 6; i++) state[1 + m->nq + dof_adr + i] = 0;
  mj_setState(m, d, state.data(), spec);
}

// is every dynamic tree of flex f asleep
static bool FlexAsleep(const mjModel* m, const mjData* d, int f) {
  int num = m->flex_interp[f] ? m->flex_nodenum[f] : m->flex_vertnum[f];
  const int* bodyid = m->flex_interp[f]
                          ? m->flex_nodebodyid + m->flex_nodeadr[f]
                          : m->flex_vertbodyid + m->flex_vertadr[f];
  for (int j = 0; j < num; j++) {
    int tree = m->body_treeid[bodyid[j]];
    if (tree >= 0 && d->tree_asleep[tree] < 0) return false;
  }
  return true;
}

// is every dynamic tree of flex f awake
static bool FlexAwake(const mjModel* m, const mjData* d, int f) {
  int num = m->flex_interp[f] ? m->flex_nodenum[f] : m->flex_vertnum[f];
  const int* bodyid = m->flex_interp[f]
                          ? m->flex_nodebodyid + m->flex_nodeadr[f]
                          : m->flex_vertbodyid + m->flex_vertadr[f];
  for (int j = 0; j < num; j++) {
    int tree = m->body_treeid[bodyid[j]];
    if (tree >= 0 && d->tree_asleep[tree] >= 0) return false;
  }
  return true;
}

// centroid of element e of (non-interpolated) flex f
static void ElementCentroid(const mjModel* m, const mjData* d, int f, int e,
                            mjtNum out[3]) {
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e * (dim + 1);
  mju_zero3(out);
  for (int k = 0; k <= dim; k++) {
    mju_addTo3(out, d->flexvert_xpos + 3 * (m->flex_vertadr[f] + edata[k]));
  }
  mju_scl3(out, out, 1.0 / (dim + 1));
}

// first element of flex f whose first vertex is pinned (world-attached) and
// that has a dynamic vertex, or -1
static int PinnedFirstVertexElement(const mjModel* m, int f) {
  int dim = m->flex_dim[f];
  for (int e = 0; e < m->flex_elemnum[f]; e++) {
    const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e * (dim + 1);
    if (m->body_treeid[m->flex_vertbodyid[m->flex_vertadr[f] + edata[0]]] >= 0)
      continue;
    for (int k = 1; k <= dim; k++) {
      if (m->body_treeid[m->flex_vertbodyid[m->flex_vertadr[f] + edata[k]]] >=
          0)
        return e;
    }
  }
  return -1;
}

// a box landing on an element whose first vertex is pinned wakes the sheet
TEST_F(SleepTest, FlexPinnedFirstVertexContactWakes) {
  char error[1024];
  mjModel* m =
      LoadModelFromString(kPinnedSheet, error, sizeof(error)).release();
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  int f = mj_name2id(m, mjOBJ_FLEX, "sheet");
  int box = mj_name2id(m, mjOBJ_BODY, "box");
  ASSERT_GE(f, 0);
  ASSERT_GE(box, 0);

  for (int step = 0; step < kSettleSteps; step++) mj_step(m, d);
  ASSERT_TRUE(FlexAsleep(m, d, f)) << "sheet did not fall asleep";

  int e = PinnedFirstVertexElement(m, f);
  ASSERT_GE(e, 0) << "the sheet has no element with a pinned first vertex";
  mjtNum pos[3];
  ElementCentroid(m, d, f, e, pos);
  pos[2] += kDropHeight;
  TeleportFreeBody(m, d, box, pos);

  // the sheet is asleep when the box lands; the landing wakes it
  bool slept_before_landing = false, woke = false;
  for (int step = 0; step < kDropSteps; step++) {
    bool asleep = FlexAsleep(m, d, f);
    mj_step(m, d);
    slept_before_landing |= (asleep && d->ncon == 0);
    woke |= (asleep && d->ncon > 0 && !FlexAsleep(m, d, f));
  }
  EXPECT_TRUE(slept_before_landing) << "sheet was awake when the box landed";
  EXPECT_TRUE(woke) << "landing box did not wake the sheet";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// a body asleep ON a sleeping flex wakes in the same step as the flex when a
// contact elsewhere wakes the flex: contact -> one vertex, edge equalities ->
// the flex, recollision -> the body under it (the wake fixed point)
TEST_F(SleepTest, FlexWakeChainReachesBodyRestingOnIt) {
  char error[1024];
  mjModel* m = LoadModelFromString(kPinnedSheetWithRester, error, sizeof(error))
                   .release();
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  int f = mj_name2id(m, mjOBJ_FLEX, "sheet");
  int box = mj_name2id(m, mjOBJ_BODY, "box");
  int rester = mj_name2id(m, mjOBJ_BODY, "rester");
  ASSERT_GE(f, 0);
  ASSERT_GE(box, 0);
  ASSERT_GE(rester, 0);
  int rester_tree = m->body_treeid[rester];

  for (int step = 0; step < kSettleSteps; step++) mj_step(m, d);
  ASSERT_TRUE(FlexAsleep(m, d, f)) << "sheet did not fall asleep";
  ASSERT_GE(d->tree_asleep[rester_tree], 0) << "rester did not fall asleep";
  EXPECT_GT(d->xpos[3 * rester + 2], 0.02)
      << "rester is not resting on the sheet";

  // drop the box on the element farthest from the rester (element 0, at -x -y)
  mjtNum pos[3];
  ElementCentroid(m, d, f, 0, pos);
  pos[2] += kDropHeight;
  TeleportFreeBody(m, d, box, pos);

  bool chain = false;
  for (int step = 0; step < kDropSteps; step++) {
    bool sheet_asleep = FlexAsleep(m, d, f);
    bool rester_asleep = d->tree_asleep[rester_tree] >= 0;
    mj_step(m, d);
    if (sheet_asleep && rester_asleep && !FlexAsleep(m, d, f)) {
      // the step that woke the sheet must have woken the rester with it
      EXPECT_LT(d->tree_asleep[rester_tree], 0)
          << "rester stayed asleep in the step the sheet woke";
      chain = true;
    }
  }
  EXPECT_TRUE(chain)
      << "the sheet never slept-then-woke with the rester asleep on it";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// a contact with an interpolated flex couples all of its nodes (27 here): all
// of them wake, and the wake sweep must not assume a small fixed number of
// bodies
TEST_F(SleepTest, InterpolatedFlexContactWakesAllNodes) {
  char error[1024];
  mjModel* m =
      LoadModelFromString(kInterpolatedCube, error, sizeof(error)).release();
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  int f = mj_name2id(m, mjOBJ_FLEX, "cube");
  int box = mj_name2id(m, mjOBJ_BODY, "box");
  ASSERT_GE(f, 0);
  ASSERT_GE(box, 0);
  ASSERT_TRUE(m->flex_interp[f]);
  ASSERT_EQ(m->flex_nodenum[f], 27);

  for (int step = 0; step < kSettleSteps; step++) mj_step(m, d);
  ASSERT_TRUE(FlexAsleep(m, d, f)) << "cube did not fall asleep";

  // drop the box onto the top of the cube
  mjtNum top = -1;
  for (int v = 0; v < m->flex_vertnum[f]; v++) {
    top = mju_max(top, d->flexvert_xpos[3 * (m->flex_vertadr[f] + v) + 2]);
  }
  mjtNum pos[3] = {0, 0, top + kDropHeight};
  TeleportFreeBody(m, d, box, pos);

  bool woke_all = false;
  for (int step = 0; step < kDropSteps; step++) {
    bool asleep = FlexAsleep(m, d, f);
    mj_step(m, d);
    woke_all |= (asleep && d->ncon > 0 && FlexAwake(m, d, f));
  }
  EXPECT_TRUE(woke_all)
      << "the landing box did not wake every node of the cube";
  for (int i = 0; i < m->nq; i++) EXPECT_TRUE(std::isfinite(d->qpos[i]));

  mj_deleteData(d);
  mj_deleteModel(m);
}

// ---------------------------------------------------------------------------
// mj_setState under sleep: the rollout pattern (one warm mjData per thread,
// mj_setState(m[r], d, state0[r]) with possibly different models m[r]).
// ---------------------------------------------------------------------------

// two boxes resting on the floor, far apart
static constexpr char kTwoBoxes[] = R"(
<mujoco>
  <option timestep="0.002"><flag sleep="enable"/></option>
  <worldbody>
    <geom type="plane" size="2 2 .1"/>
    <body name="a" pos="0 0 .05">
      <freejoint/>
      <geom type="box" size=".05 .05 .05" mass=".1"/>
    </body>
    <body name="b" pos="1 0 .05">
      <freejoint/>
      <geom type="box" size=".05 .05 .05" mass=".1"/>
    </body>
  </worldbody>
</mujoco>
)";

// kTwoBoxes with the floor raised: same sizes, different static geometry
static constexpr char kTwoBoxesRaisedFloor[] = R"(
<mujoco>
  <option timestep="0.002"><flag sleep="enable"/></option>
  <worldbody>
    <geom type="plane" pos="0 0 .3" size="2 2 .1"/>
    <body name="a" pos="0 0 .35">
      <freejoint/>
      <geom type="box" size=".05 .05 .05" mass=".1"/>
    </body>
    <body name="b" pos="1 0 .35">
      <freejoint/>
      <geom type="box" size=".05 .05 .05" mass=".1"/>
    </body>
  </worldbody>
</mujoco>
)";

static constexpr mjtNum kBoxHalfSize = 0.05;
static constexpr mjtNum kRaisedFloorHeight = 0.3;

// 0.8 s: covers a 0.25 m fall and the landing
static constexpr int kLandSteps = 400;

// teleporting a sleeping body through mj_setState wakes it immediately and
// leaves sleeping trees whose qpos did not change asleep
TEST_F(SleepTest, SetStateTeleportWakesOnlyMovedTree) {
  char error[1024];
  mjModel* m = LoadModelFromString(kTwoBoxes, error, sizeof(error)).release();
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  int a = mj_name2id(m, mjOBJ_BODY, "a");
  int b = mj_name2id(m, mjOBJ_BODY, "b");
  int tree_a = m->body_treeid[a];
  int tree_b = m->body_treeid[b];

  for (int step = 0; step < kSettleSteps; step++) mj_step(m, d);
  ASSERT_GE(d->tree_asleep[tree_a], 0) << "a did not fall asleep";
  ASSERT_GE(d->tree_asleep[tree_b], 0) << "b did not fall asleep";

  // teleport a to just above b, at rest
  static constexpr mjtNum kGap = 0.1;
  mjtNum pos[3] = {d->xpos[3 * b], d->xpos[3 * b + 1],
                   d->xpos[3 * b + 2] + 2 * kBoxHalfSize + kGap};
  TeleportFreeBody(m, d, a, pos);
  EXPECT_LT(d->tree_asleep[tree_a], 0)
      << "teleported a is still asleep after mj_setState";
  EXPECT_GE(d->tree_asleep[tree_b], 0)
      << "unchanged b was woken by mj_setState";
  EXPECT_EQ(d->xpos[3 * a + 2], pos[2])
      << "kinematics of a not refreshed by mj_setState";

  // a lands on b and wakes it
  bool woke_b = false;
  for (int step = 0; step < kLandSteps; step++) {
    mj_step(m, d);
    woke_b |= d->tree_asleep[tree_b] < 0;
  }
  EXPECT_TRUE(woke_b) << "b never woke from a landing on it";
  EXPECT_NEAR(d->xpos[3 * a + 2], 3 * kBoxHalfSize, 0.01)
      << "a did not come to rest on b";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// a warm mjData reused with a model whose static geometry differs (rollout with
// a list of models) sees the new static poses after mj_setState, not the ones
// baked in by mj_resetData
TEST_F(SleepTest, SetStateRefreshesStaticPosesOfDifferingModel) {
  char error[1024];
  mjModel* m = LoadModelFromString(kTwoBoxes, error, sizeof(error)).release();
  ASSERT_THAT(m, NotNull()) << error;
  mjModel* m_raised =
      LoadModelFromString(kTwoBoxesRaisedFloor, error, sizeof(error)).release();
  ASSERT_THAT(m_raised, NotNull()) << error;
  ASSERT_EQ(mj_stateSize(m, mjSTATE_FULLPHYSICS),
            mj_stateSize(m_raised, mjSTATE_FULLPHYSICS));
  int a = mj_name2id(m, mjOBJ_BODY, "a");
  int b = mj_name2id(m, mjOBJ_BODY, "b");
  int floor = 0;  // the plane is the only geom on the world body
  ASSERT_EQ(m->geom_bodyid[floor], 0);
  ASSERT_EQ(m_raised->geom_pos[3 * floor + 2], kRaisedFloorHeight);

  // warm data: made and settled with the low floor, both boxes asleep on it
  mjData* d = mj_makeData(m);
  for (int step = 0; step < kSettleSteps; step++) mj_step(m, d);
  ASSERT_GE(d->tree_asleep[m->body_treeid[a]], 0);
  ASSERT_GE(d->tree_asleep[m->body_treeid[b]], 0);
  ASSERT_EQ(d->geom_xpos[3 * floor + 2], 0);

  // next rollout: the raised-floor model with its initial state, both boxes
  // above its floor
  std::vector<mjtNum> state(mj_stateSize(m_raised, mjSTATE_FULLPHYSICS));
  mjData* d_raised = mj_makeData(m_raised);
  mj_getState(m_raised, d_raised, state.data(), mjSTATE_FULLPHYSICS);
  mj_deleteData(d_raised);
  mj_setState(m_raised, d, state.data(), mjSTATE_FULLPHYSICS);
  EXPECT_EQ(d->geom_xpos[3 * floor + 2], kRaisedFloorHeight)
      << "static floor pose not refreshed";

  // the boxes come to rest on the raised floor, not on the stale low one
  for (int step = 0; step < kLandSteps; step++) mj_step(m_raised, d);
  EXPECT_NEAR(d->xpos[3 * a + 2], kRaisedFloorHeight + kBoxHalfSize, 0.01);
  EXPECT_NEAR(d->xpos[3 * b + 2], kRaisedFloorHeight + kBoxHalfSize, 0.01);

  mj_deleteData(d);
  mj_deleteModel(m_raised);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
