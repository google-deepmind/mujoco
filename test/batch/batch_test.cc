// Copyright 2026 DeepMind Technologies Limited
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

// Tests for experimental/batch.h.

#include <mujoco/experimental/batch.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using BatchTest = MujocoTest;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

// activation dynamics, a mocap weld, a keyframe, and sensors that set mjData's
// lazy-evaluation flags (accelerometer, subtreelinvel)
static constexpr char kXml[] = R"(
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 .1"/>
    <body name="cart" pos="0 0 .1">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="box" size=".1 .1 .05" mass="1"/>
      <body name="pole" pos="0 0 .05">
        <joint name="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 .5" size=".02" mass=".1"/>
        <site name="tip" pos="0 0 .5"/>
      </body>
    </body>
    <body name="ball" pos="0 1 .5">
      <freejoint/>
      <geom type="sphere" size=".05" mass=".2"/>
    </body>
    <body name="mocap" mocap="true" pos="0 1 .5">
      <geom type="sphere" size=".02" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality><weld body1="mocap" body2="ball"/></equality>
  <actuator>
    <motor joint="slide" gear="5"/>
    <general joint="hinge" dyntype="filter" dynprm="0.02" gainprm="5" biastype="affine" biasprm="0 -5 0"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge"/>
    <accelerometer site="tip"/>
    <subtreelinvel body="cart"/>
  </sensor>
  <keyframe>
    <key qpos="0.3 0.5 0 1 .5 1 0 0 0" ctrl="0.1 0.2" act="0.2" mpos="0 1 .5"/>
  </keyframe>
</mujoco>
)";

static constexpr int kNsim = 8;

// offset of a state component in an mjSTATE_INTEGRATION row
int Offset(const mjModel* m, mjtState component) {
  return mj_stateSize(m,
                      mjSTATE_INTEGRATION & (static_cast<int>(component) - 1));
}

std::vector<mjtNum> GetState(const mjModel* m, const mjData* d) {
  std::vector<mjtNum> row(mj_stateSize(m, mjSTATE_INTEGRATION));
  mj_getState(m, d, row.data(), mjSTATE_INTEGRATION);
  return row;
}

struct BatchPtr {
  mjBatch* b;
  explicit BatchPtr(mjBatch* batch) : b(batch) {}
  ~BatchPtr() { mjb_deleteBatch(b); }
  mjtNum* Row(int i) {
    return mjb_state(b) + static_cast<size_t>(i) * mjb_nstate(b);
  }
};

// 50 calls on a batch and on one mjData per simulation, with writes to every
// kind of input, a keyframe reset and subsets; the states and outputs agree
// after each
void Lockstep(int nthread, int nstep, int persistent = 0) {
  MjModelPtr model = LoadModelFromString(kXml);
  ASSERT_THAT(model, NotNull());
  const mjModel* m = model.get();
  BatchPtr batch(mjb_makeBatch(m, kNsim, nthread, persistent, nullptr, 0));
  ASSERT_THAT(batch.b, NotNull());
  mjBatch* b = batch.b;
  EXPECT_EQ(mjb_nthread(b), nthread);
  EXPECT_EQ(mjb_persistent(b), persistent);

  std::vector<MjDataPtr> ref;
  for (int i = 0; i < kNsim; ++i) ref.push_back(MakeData(model));

  int size, elemsize;
  auto* sensordata =
      static_cast<mjtNum*>(mjb_output(b, "sensordata", &size, &elemsize));
  ASSERT_THAT(sensordata, NotNull());
  EXPECT_EQ(size, m->nsensordata);
  EXPECT_EQ(elemsize, sizeof(mjtNum));
  auto* site_xpos =
      static_cast<mjtNum*>(mjb_output(b, "site_xpos", &size, nullptr));
  ASSERT_THAT(site_xpos, NotNull());

  int ctrl = Offset(m, mjSTATE_CTRL), qpos = Offset(m, mjSTATE_QPOS);
  int mocap = Offset(m, mjSTATE_MOCAP_POS),
      xfrc = Offset(m, mjSTATE_XFRC_APPLIED);
  int eq = Offset(m, mjSTATE_EQ_ACTIVE);
  int ball = mj_name2id(m, mjOBJ_BODY, "ball");

  // a write lands in the batch's row and in the reference's data
  auto write = [&](int i, int offset, mjtNum value, mjtNum* field) {
    batch.Row(i)[offset] = value;
    *field = value;
  };

  for (int call = 0; call < 50; ++call) {
    for (int i = 0; i < kNsim; ++i) {
      for (int k = 0; k < m->nu; ++k) {
        write(i, ctrl + k, 0.5 * mju_sin(0.3 * call + i + k), ref[i]->ctrl + k);
      }
      if (call == 10 && i % 3 == 1)
        write(i, qpos + 1, 0.1 * i, ref[i]->qpos + 1);
      if (call == 15) write(i, mocap + 0, 0.02 * i, ref[i]->mocap_pos + 0);
      if (call == 20 && (i == 2 || i == 3)) {
        write(i, xfrc + 6 * ball + 3, 0.3, ref[i]->xfrc_applied + 6 * ball + 3);
      }
      if (call == 30 && i % 2 == 0) {
        batch.Row(i)[eq] = 0;
        ref[i]->eq_active[0] = 0;
      }
    }

    std::vector<int> ids;
    if (call == 25) {
      ids = {1, 5};
      ASSERT_EQ(mjb_reset(b, ids.data(), ids.size(), 0), 0);
      for (int i : ids) {
        mj_resetDataKeyframe(m, ref[i].get(), 0);
        mj_forward(m, ref[i].get());
      }
    } else {
      if (call % 4 == 3) ids = {0, 2, 3, 7};
      const int* p = ids.empty() ? nullptr : ids.data();
      ASSERT_EQ(mjb_step(b, p, ids.size(), nstep), 0) << mjb_error(b, 0);
      for (int i = 0; i < kNsim; ++i) {
        if (!ids.empty() && std::find(ids.begin(), ids.end(), i) == ids.end())
          continue;
        for (int k = 0; k < nstep; ++k) mj_step(m, ref[i].get());
      }
    }

    // rows of simulations that did not run hold the writes not yet applied
    for (int i = 0; i < kNsim; ++i) {
      bool ran =
          ids.empty() || std::find(ids.begin(), ids.end(), i) != ids.end();
      if (!ran) continue;
      std::vector<mjtNum> expected = GetState(m, ref[i].get());
      std::vector<mjtNum> actual(batch.Row(i), batch.Row(i) + mjb_nstate(b));
      ASSERT_EQ(actual, expected) << "call " << call << " sim " << i;
      for (int k = 0; k < m->nsensordata; ++k) {
        ASSERT_EQ(sensordata[i * m->nsensordata + k], ref[i]->sensordata[k])
            << "call " << call << " sim " << i;
      }
      for (int k = 0; k < 3 * m->nsite; ++k) {
        ASSERT_EQ(site_xpos[i * 3 * m->nsite + k], ref[i]->site_xpos[k]);
      }
    }
  }
}

TEST_F(BatchTest, LockstepOneThread) { Lockstep(1, 1); }
TEST_F(BatchTest, LockstepThreeThreads) { Lockstep(3, 1); }
TEST_F(BatchTest, LockstepSevenThreadsMultiStep) { Lockstep(7, 3); }
TEST_F(BatchTest, LockstepPersistent) { Lockstep(3, 2, 1); }

// per-simulation masses with derived constants, against one model per
// simulation
TEST_F(BatchTest, ExpandAndSetConst) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  int pole = mj_name2id(m, mjOBJ_BODY, "pole");
  std::vector<std::vector<mjtNum>> finals;
  for (int nthread : {1, 4}) {
    BatchPtr batch(mjb_makeBatch(m, kNsim, nthread, 0, nullptr, 0));
    mjBatch* b = batch.b;
    auto* mass =
        static_cast<mjtNum*>(mjb_expand(b, "body_mass", nullptr, nullptr));
    ASSERT_THAT(mass, NotNull());
    for (int i = 0; i < kNsim; ++i) mass[i * m->nbody + pole] = 0.1 + 0.2 * i;
    ASSERT_EQ(mjb_setConst(b, nullptr, 0), 0);

    // mj_setConst's outputs now have per-simulation storage
    auto* subtree = static_cast<mjtNum*>(
        mjb_expand(b, "body_subtreemass", nullptr, nullptr));
    for (int i = 0; i < kNsim; ++i) {
      EXPECT_EQ(subtree[i * m->nbody + pole], mass[i * m->nbody + pole]);
    }

    int ctrl = Offset(m, mjSTATE_CTRL);
    for (int i = 0; i < kNsim; ++i) batch.Row(i)[ctrl] = 1.0;
    ASSERT_EQ(mjb_step(b, nullptr, 0, 20), 0);
    std::vector<mjtNum> all;
    for (int i = 0; i < kNsim; ++i) {
      all.insert(all.end(), batch.Row(i), batch.Row(i) + mjb_nstate(b));
    }
    finals.push_back(all);

    for (int i = 0; i < kNsim; ++i) {
      mjModel* mi = mj_copyModel(nullptr, m);
      mi->body_mass[pole] = mass[i * m->nbody + pole];  // as stored
      mjData* d = mj_makeData(mi);
      mj_setConst(mi, d);
      mj_resetData(mi, d);
      d->ctrl[0] = 1.0;
      for (int k = 0; k < 20; ++k) mj_step(mi, d);
      EXPECT_EQ(std::vector<mjtNum>(batch.Row(i), batch.Row(i) + mjb_nstate(b)),
                GetState(mi, d))
          << "sim " << i;
      mj_deleteData(d);
      mj_deleteModel(mi);
    }
  }
  EXPECT_EQ(finals[0], finals[1]);
}

// the scalars mj_setConst writes (gravity compensation flags) are per
// simulation
TEST_F(BatchTest, GravcompIsPerSimulation) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  int pole = mj_name2id(m, mjOBJ_BODY, "pole");
  BatchPtr batch(mjb_makeBatch(m, kNsim, 3, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* gravcomp =
      static_cast<mjtNum*>(mjb_expand(b, "body_gravcomp", nullptr, nullptr));
  for (int i = 0; i < kNsim; i += 2) gravcomp[i * m->nbody + pole] = 1;
  ASSERT_EQ(mjb_setConst(b, nullptr, 0), 0);
  int qpos = Offset(m, mjSTATE_QPOS), qvel = Offset(m, mjSTATE_QVEL);
  for (int i = 0; i < kNsim; ++i) batch.Row(i)[qpos + 1] = 0.3;
  ASSERT_EQ(mjb_step(b, nullptr, 0, 5), 0);
  for (int i = 0; i < kNsim; i += 2) {
    EXPECT_NE(batch.Row(i)[qvel + 1], batch.Row(i + 1)[qvel + 1]);
    EXPECT_EQ(batch.Row(i)[qvel + 1], batch.Row(0)[qvel + 1]);
  }
}

// reset uses each simulation's expanded qpos0 and discards writes made before
// it
TEST_F(BatchTest, ResetExpandedQpos0) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  BatchPtr batch(mjb_makeBatch(m, kNsim, 2, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* qpos0 = static_cast<mjtNum*>(mjb_expand(b, "qpos0", nullptr, nullptr));
  for (int i = 0; i < kNsim; ++i) qpos0[i * m->nq + 1] = 0.1 * i;
  auto* xpos = static_cast<mjtNum*>(mjb_output(b, "xpos", nullptr, nullptr));
  int qpos = Offset(m, mjSTATE_QPOS);
  batch.Row(5)[qpos] = 0.25;  // discarded by the reset
  int ids[] = {2, 5};
  ASSERT_EQ(mjb_reset(b, ids, 2, -1), 0);
  for (int i : ids) {
    EXPECT_EQ(batch.Row(i)[qpos], 0);
    EXPECT_EQ(batch.Row(i)[qpos + 1], qpos0[i * m->nq + 1]);
    EXPECT_NE(xpos[i * 3 * m->nbody + 3 * 1 + 2], 0);  // reset runs mj_forward
  }
  EXPECT_EQ(xpos[0 * 3 * m->nbody + 3 * 1 + 2],
            0);  // not run: output not filled yet
}

// a MuJoCo error in one simulation is reported; the others run, and it keeps
// its state
TEST_F(BatchTest, ErrorKeepsFailingSimulation) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  for (int nthread : {1, 3}) {
    BatchPtr batch(mjb_makeBatch(m, kNsim, nthread, 0, nullptr, 0));
    mjBatch* b = batch.b;
    auto* eq_type =
        static_cast<int*>(mjb_expand(b, "eq_type", nullptr, nullptr));
    eq_type[2] = 99;
    std::vector<mjtNum> before(batch.Row(2), batch.Row(2) + mjb_nstate(b));
    EXPECT_EQ(mjb_step(b, nullptr, 0, 3), 1);
    EXPECT_NE(mjb_status(b)[2], 0);
    EXPECT_EQ(mjb_status(b)[0], 0);
    EXPECT_THAT(std::string(mjb_error(b, 2)), Not(testing::IsEmpty()));
    EXPECT_STREQ(mjb_error(b, 0), "");
    EXPECT_EQ(std::vector<mjtNum>(batch.Row(2), batch.Row(2) + mjb_nstate(b)),
              before);
    EXPECT_GT(batch.Row(0)[0], 0);  // time advanced in the others
    eq_type[2] = m->eq_type[0];
    EXPECT_EQ(mjb_step(b, nullptr, 0, 1), 0);
    EXPECT_STREQ(mjb_error(b, 2), "");
    EXPECT_EQ(mjb_status(b)[2], 0);
    EXPECT_EQ(batch.Row(2)[0], m->opt.timestep);
    EXPECT_EQ(batch.Row(0)[0], 4 * m->opt.timestep);
  }
}

TEST_F(BatchTest, SleepIsRejected) {
  std::string xml = kXml;
  xml.replace(xml.find("<option"), 7,
              "<option><flag sleep=\"enable\"/></option><option");
  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  char error[256];
  EXPECT_THAT(mjb_makeBatch(model.get(), kNsim, 1, 0, error, sizeof(error)),
              IsNull());
  EXPECT_THAT(std::string(error), HasSubstr("persistent"));
  BatchPtr batch(mjb_makeBatch(model.get(), kNsim, 1, 1, error, sizeof(error)));
  EXPECT_THAT(batch.b, NotNull());
}

TEST_F(BatchTest, OutputsAndExpandRefuse) {
  MjModelPtr model = LoadModelFromString(kXml);
  BatchPtr batch(mjb_makeBatch(model.get(), kNsim, 1, 0, nullptr, 0));
  EXPECT_THAT(mjb_output(batch.b, "qpos", nullptr, nullptr),
              IsNull());  // in the state
  EXPECT_THAT(mjb_output(batch.b, "nope", nullptr, nullptr), IsNull());
  EXPECT_THAT(mjb_expand(batch.b, "mesh_vert", nullptr, nullptr),
              IsNull());  // asset
  EXPECT_THAT(mjb_expand(batch.b, "nope", nullptr, nullptr), IsNull());
}

TEST_F(BatchTest, BadIds) {
  MjModelPtr model = LoadModelFromString(kXml);
  BatchPtr batch(mjb_makeBatch(model.get(), kNsim, 2, 0, nullptr, 0));
  int unsorted[] = {3, 1};
  EXPECT_THAT(MjuErrorMessageFrom(mjb_step)(batch.b, unsorted, 2, 1),
              HasSubstr("sorted"));
  int out_of_range[] = {kNsim};
  EXPECT_THAT(MjuErrorMessageFrom(mjb_forward)(batch.b, out_of_range, 1),
              HasSubstr("sorted"));
  // the batch is still usable: no lock was left held
  EXPECT_EQ(mjb_step(batch.b, nullptr, 0, 1), 0);
}

// apply runs any function on each simulation; with save=0 the state is
// untouched
TEST_F(BatchTest, Apply) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  BatchPtr batch(mjb_makeBatch(m, kNsim, 3, 0, nullptr, 0));
  mjBatch* b = batch.b;
  int qpos = Offset(m, mjSTATE_QPOS);
  for (int i = 0; i < kNsim; ++i) batch.Row(i)[qpos + 1] = 0.1 * i;
  std::vector<mjtNum> before(mjb_state(b),
                             mjb_state(b) + kNsim * mjb_nstate(b));

  std::vector<mjtNum> jacp(kNsim * 3 * m->nv);
  auto jac_tip = +[](const mjModel* m, mjData* d, int sim, void* arg) {
    mj_kinematics(m, d);
    mj_comPos(m, d);
    mjtNum* out = static_cast<mjtNum*>(arg) + sim * 3 * m->nv;
    mj_jacSite(m, d, out, nullptr, mj_name2id(m, mjOBJ_SITE, "tip"));
  };
  ASSERT_EQ(mjb_apply(b, nullptr, 0, jac_tip, jacp.data(), 0), 0);
  EXPECT_EQ(
      std::vector<mjtNum>(mjb_state(b), mjb_state(b) + kNsim * mjb_nstate(b)),
      before);

  MjDataPtr d = MakeData(model);
  std::vector<mjtNum> expected(3 * m->nv);
  for (int i = 0; i < kNsim; ++i) {
    mj_setState(m, d.get(), batch.Row(i), mjSTATE_INTEGRATION);
    mj_kinematics(m, d.get());
    mj_comPos(m, d.get());
    mj_jacSite(m, d.get(), expected.data(), nullptr,
               mj_name2id(m, mjOBJ_SITE, "tip"));
    EXPECT_EQ(std::vector<mjtNum>(jacp.begin() + i * 3 * m->nv,
                                  jacp.begin() + (i + 1) * 3 * m->nv),
              expected)
        << "sim " << i;
  }
}

// apply publishes no outputs: a callback that computes only some fields must
// not overwrite rows with what the worker's data holds from another simulation
TEST_F(BatchTest, ApplyLeavesOutputsAlone) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body><joint name="j" type="slide" axis="0 0 1"/><geom size=".1"/></body>
    </worldbody>
    <sensor><jointvel joint="j"/></sensor>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml);
  const mjModel* m = model.get();
  BatchPtr batch(mjb_makeBatch(m, 4, 1, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* sensordata =
      static_cast<mjtNum*>(mjb_output(b, "sensordata", nullptr, nullptr));
  int qvel = Offset(m, mjSTATE_QVEL);
  for (int i = 0; i < 4; ++i) batch.Row(i)[qvel] = i + 1;
  ASSERT_EQ(mjb_forward(b, nullptr, 0), 0);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(sensordata[i], i + 1);
  auto kinematics =
      +[](const mjModel* m, mjData* d, int, void*) { mj_kinematics(m, d); };
  ASSERT_EQ(mjb_apply(b, nullptr, 0, kinematics, nullptr, 0), 0);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(sensordata[i], i + 1);
}

// a simulation that fails set_const does not cost the others their derived
// constants
TEST_F(BatchTest, SetConstErrorKeepsOthersComplete) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.002"/>
    <worldbody>
      <body pos="0 0 1">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".1" mass="1" contype="0" conaffinity="0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml);
  const mjModel* m = model.get();
  BatchPtr batch(mjb_makeBatch(m, 2, 1, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* mass =
      static_cast<mjtNum*>(mjb_expand(b, "body_mass", nullptr, nullptr));
  mass[0 * m->nbody + 1] = 2;
  mass[1 * m->nbody + 1] = 0;  // singular: mj_setConst raises in simulation 1

  // the singular simulation also warns, on a worker thread, where the fixture's
  // thread-local mock does not reach; accept that warning here
  static mjfLogHandler fixture_handler = nullptr;
  fixture_handler = mju_setLogHandler(+[](const mjLogMessage* msg) {
    if (msg->level == mjLOG_WARNING && std::strstr(msg->subject, "singular"))
      return;
    fixture_handler(msg);
  });
  EXPECT_EQ(mjb_setConst(b, nullptr, 0), 1);
  EXPECT_EQ(mjb_status(b)[0], 0);
  EXPECT_NE(mjb_status(b)[1], 0);
  mju_setLogHandler(fixture_handler);
  int ids[] = {0};
  ASSERT_EQ(mjb_step(b, ids, 1, 1), 0);

  mjModel* ref = mj_copyModel(nullptr, m);
  ref->body_mass[1] = 2;
  mjData* d = mj_makeData(ref);
  mj_setConst(ref, d);
  mj_resetData(ref, d);
  mj_step(ref, d);
  EXPECT_EQ(std::vector<mjtNum>(batch.Row(0), batch.Row(0) + mjb_nstate(b)),
            GetState(ref, d));
  mj_deleteData(d);
  mj_deleteModel(ref);
}

// a persistent batch keeps one mjData per simulation, so sleeping works: trees
// fall asleep and wake on writes exactly as in a loop over one mjData per
// simulation
TEST_F(BatchTest, PersistentSleepMatchesLoop) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast" viscosity="10" sleep_tolerance="0.01">
      <flag sleep="enable" gravity="disable" constraint="disable" contact="disable"/>
    </option>
    <worldbody>
      <body><freejoint/><geom type="box" size=".1 .2 .3" mass="1" euler="10 20 30"/></body>
      <body pos="1 0 0"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  const mjModel* m = model.get();
  constexpr int nsim = 4;
  BatchPtr batch(mjb_makeBatch(m, nsim, 2, 1, nullptr, 0));
  ASSERT_THAT(batch.b, NotNull());
  mjBatch* b = batch.b;
  std::vector<MjDataPtr> ref;
  int qvel = Offset(m, mjSTATE_QVEL);
  for (int i = 0; i < nsim; ++i) {
    ref.push_back(MakeData(model));
    for (int k = 0; k < m->nv; ++k) {
      mjtNum v = 0.1 * (i + 1) * (k % 3 + 1);
      batch.Row(i)[qvel + k] = v;
      ref[i]->qvel[k] = v;
    }
  }
  bool slept = false;
  for (int call = 0; call < 100; ++call) {
    if (call == 60) {  // a write to a sleeping simulation wakes it
      batch.Row(1)[qvel] = 0.7;
      ref[1]->qvel[0] = 0.7;
    }
    ASSERT_EQ(mjb_step(b, nullptr, 0, 10), 0) << mjb_error(b, 0);
    for (int i = 0; i < nsim; ++i) {
      for (int k = 0; k < 10; ++k) mj_step(m, ref[i].get());
      slept = slept || ref[i]->ntree_awake < m->ntree;
      ASSERT_EQ(std::vector<mjtNum>(batch.Row(i), batch.Row(i) + mjb_nstate(b)),
                GetState(m, ref[i].get()))
          << "call " << call << " sim " << i;
    }
  }
  EXPECT_TRUE(slept) << "no tree fell asleep: the test does not exercise sleep";
}

// sleep enabled per simulation through mjOption needs a persistent batch too
TEST_F(BatchTest, ExpandedSleepFlagNeedsPersistent) {
  MjModelPtr model = LoadModelFromString(kXml);
  BatchPtr batch(mjb_makeBatch(model.get(), kNsim, 1, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* enableflags =
      static_cast<int*>(mjb_expand(b, "opt.enableflags", nullptr, nullptr));
  ASSERT_THAT(enableflags, NotNull());
  enableflags[3] |= mjENBL_SLEEP;
  EXPECT_EQ(mjb_step(b, nullptr, 0, 1), 1);
  EXPECT_NE(mjb_status(b)[3], 0);
  EXPECT_THAT(std::string(mjb_error(b, 3)), HasSubstr("persistent"));
}

// mjOption fields are expandable: per-simulation gravity, against one model
// each
TEST_F(BatchTest, ExpandOption) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  BatchPtr batch(mjb_makeBatch(m, kNsim, 3, 0, nullptr, 0));
  mjBatch* b = batch.b;
  int size, elemsize;
  auto* gravity =
      static_cast<mjtNum*>(mjb_expand(b, "opt.gravity", &size, &elemsize));
  ASSERT_THAT(gravity, NotNull());
  EXPECT_EQ(size, 3);
  EXPECT_EQ(elemsize, sizeof(mjtNum));
  EXPECT_THAT(mjb_expand(b, "opt.timestep", &size, nullptr), NotNull());
  EXPECT_EQ(size, 1);
  for (int i = 0; i < kNsim; ++i) gravity[3 * i + 2] = -1.0 * i;
  ASSERT_EQ(mjb_step(b, nullptr, 0, 10), 0);
  for (int i = 0; i < kNsim; ++i) {
    mjModel* mi = mj_copyModel(nullptr, m);
    mi->opt.gravity[2] = gravity[3 * i + 2];
    mjData* d = mj_makeData(mi);
    for (int k = 0; k < 10; ++k) mj_step(mi, d);
    EXPECT_EQ(std::vector<mjtNum>(batch.Row(i), batch.Row(i) + mjb_nstate(b)),
              GetState(mi, d))
        << "sim " << i;
    mj_deleteData(d);
    mj_deleteModel(mi);
  }
}

// every simulation that raises is reported, with its own message
TEST_F(BatchTest, AllFailuresReported) {
  MjModelPtr model = LoadModelFromString(kXml);
  BatchPtr batch(mjb_makeBatch(model.get(), kNsim, 3, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* eq_type = static_cast<int*>(mjb_expand(b, "eq_type", nullptr, nullptr));
  eq_type[1] = eq_type[5] = 99;
  EXPECT_EQ(mjb_step(b, nullptr, 0, 1), 2);
  for (int i = 0; i < kNsim; ++i) {
    bool failed = i == 1 || i == 5;
    EXPECT_EQ(mjb_status(b)[i] != 0, failed) << "sim " << i;
    EXPECT_EQ(std::string(mjb_error(b, i)).empty(), !failed) << "sim " << i;
  }
}

// mjb_rollout is a loop of mj_setState(control) and mj_step with records after
// each substep, on the selected simulations, in call order
TEST_F(BatchTest, RolloutMatchesLoop) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  constexpr int nstep = 6;
  for (int nthread : {1, 3}) {
    BatchPtr batch(mjb_makeBatch(m, kNsim, nthread, 0, nullptr, 0));
    mjBatch* b = batch.b;
    int ids[] = {1, 4, 6};
    int n = 3;
    int spec = mjSTATE_CTRL | mjSTATE_QFRC_APPLIED;
    int ncontrol = mj_stateSize(m, spec);
    std::vector<mjtNum> control(n * nstep * ncontrol);
    for (int k = 0; k < control.size(); ++k)
      control[k] = 0.3 * mju_sin(0.7 * k);
    int nstate = mj_stateSize(m, mjSTATE_FULLPHYSICS);
    std::vector<mjtNum> states(n * nstep * nstate),
        sensors(n * nstep * m->nsensordata);
    mjBatchRecord record[] = {{nullptr, mjSTATE_FULLPHYSICS, states.data()},
                              {"sensordata", 0, sensors.data()}};
    ASSERT_EQ(mjb_rollout(b, ids, n, nstep, control.data(), spec, record, 2),
              0);

    for (int j = 0; j < n; ++j) {
      MjDataPtr d = MakeData(model);
      std::vector<mjtNum> row(nstate);
      for (int k = 0; k < nstep; ++k) {
        mj_setState(m, d.get(), control.data() + (j * nstep + k) * ncontrol,
                    spec);
        mj_step(m, d.get());
        mj_getState(m, d.get(), row.data(), mjSTATE_FULLPHYSICS);
        size_t at = j * nstep + k;
        EXPECT_EQ(std::vector<mjtNum>(states.begin() + at * nstate,
                                      states.begin() + (at + 1) * nstate),
                  row)
            << "j " << j << " k " << k;
        EXPECT_EQ(
            std::vector<mjtNum>(sensors.begin() + at * m->nsensordata,
                                sensors.begin() + (at + 1) * m->nsensordata),
            std::vector<mjtNum>(d->sensordata, d->sensordata + m->nsensordata));
      }
      // the row continues from the rollout's last state
      EXPECT_EQ(std::vector<mjtNum>(batch.Row(ids[j]),
                                    batch.Row(ids[j]) + mjb_nstate(b)),
                GetState(m, d.get()));
    }
    EXPECT_EQ(batch.Row(0)[0], 0);  // not selected: did not run
  }
}

// a simulation whose warning counters rise stops, and its remaining records
// repeat
TEST_F(BatchTest, RolloutStopsAtWarning) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  BatchPtr batch(
      mjb_makeBatch(m, 2, 1, 0, nullptr, 0));  // on the caller's thread
  mjBatch* b = batch.b;
  batch.Row(0)[Offset(m, mjSTATE_QPOS)] =
      std::numeric_limits<mjtNum>::quiet_NaN();
  mock_warning_handler.ExpectWarnings("QPOS");
  constexpr int nstep = 4;
  std::vector<mjtNum> qpos(2 * nstep * m->nq);
  mjBatchRecord record[] = {{"qpos", 0, qpos.data()}};
  ASSERT_EQ(mjb_rollout(b, nullptr, 0, nstep, nullptr, 0, record, 1), 0);
  for (int k = 1; k < nstep; ++k) {
    EXPECT_EQ(std::vector<mjtNum>(qpos.begin() + k * m->nq,
                                  qpos.begin() + (k + 1) * m->nq),
              std::vector<mjtNum>(qpos.begin(), qpos.begin() + m->nq))
        << "k " << k;
  }
  // the other simulation ran all its steps
  EXPECT_NE(std::vector<mjtNum>(qpos.begin() + (nstep + 1) * m->nq,
                                qpos.begin() + (nstep + 2) * m->nq),
            std::vector<mjtNum>(qpos.begin() + nstep * m->nq,
                                qpos.begin() + (nstep + 1) * m->nq));
}

// thread models share the asset arrays with the batch's model: a mesh and a
// heightfield in contact, with per-simulation masses and constants, against one
// model each
TEST_F(BatchTest, AssetsAreShared) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet" vertex="0 0 0  .2 0 0  0 .2 0  0 0 .2"/>
      <hfield name="bumps" nrow="3" ncol="3" size="1 1 .1 .1" elevation="0 1 0 1 0 1 0 1 0"/>
    </asset>
    <worldbody>
      <geom type="hfield" hfield="bumps"/>
      <body pos="0 0 .5"><freejoint/><geom type="mesh" mesh="tet"/></body>
      <body pos=".3 .3 .6"><freejoint/><geom type="box" size=".05 .05 .05"/></body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  const mjModel* m = model.get();
  constexpr int nsim = 6;
  BatchPtr batch(mjb_makeBatch(m, nsim, 3, 0, nullptr, 0));
  mjBatch* b = batch.b;
  auto* mass =
      static_cast<mjtNum*>(mjb_expand(b, "body_mass", nullptr, nullptr));
  for (int i = 0; i < nsim; ++i) mass[i * m->nbody + 1] = 0.5 + 0.25 * i;
  ASSERT_EQ(mjb_setConst(b, nullptr, 0), 0);
  ASSERT_EQ(mjb_step(b, nullptr, 0, 200), 0);
  for (int i = 0; i < nsim; ++i) {
    mjModel* mi = mj_copyModel(nullptr, m);
    mi->body_mass[1] = mass[i * m->nbody + 1];
    mjData* d = mj_makeData(mi);
    mj_setConst(mi, d);
    mj_resetData(mi, d);
    for (int k = 0; k < 200; ++k) mj_step(mi, d);
    EXPECT_GT(d->ncon, 0)
        << "no contact: the test does not exercise the assets";
    EXPECT_EQ(std::vector<mjtNum>(batch.Row(i), batch.Row(i) + mjb_nstate(b)),
              GetState(mi, d))
        << "sim " << i;
    mj_deleteData(d);
    mj_deleteModel(mi);
  }
}

// a function run by mjb_apply may call another batch: the enclosing
// simulation's error trap and the caller's log handler survive the nested call
TEST_F(BatchTest, ApplyCallsAnotherBatch) {
  MjModelPtr model = LoadModelFromString(kXml);
  const mjModel* m = model.get();
  BatchPtr outer(mjb_makeBatch(m, kNsim, 2, 0, nullptr, 0));
  BatchPtr inner(mjb_makeBatch(m, kNsim, 2, 0, nullptr, 0));
  auto step_inner = +[](const mjModel* m, mjData* d, int sim, void* arg) {
    mjb_step(static_cast<mjBatch*>(arg), &sim, 1, 1);
    if (sim == 2) mju_error("raised after the nested call");
  };
  mjfLogHandler handler = _mjPRIVATE_setTlsLogHandler(nullptr);
  _mjPRIVATE_setTlsLogHandler(handler);

  EXPECT_EQ(mjb_apply(outer.b, nullptr, 0, step_inner, inner.b, 0), 1);
  EXPECT_EQ(_mjPRIVATE_setTlsLogHandler(handler), handler);
  for (int i = 0; i < kNsim; ++i) {
    EXPECT_EQ(mjb_status(outer.b)[i], i == 2) << "sim " << i;
    EXPECT_EQ(mjb_status(inner.b)[i], 0) << "sim " << i;
    EXPECT_EQ(inner.Row(i)[0], m->opt.timestep) << "sim " << i;
  }
  EXPECT_THAT(mjb_error(outer.b, 2), HasSubstr("raised after the nested call"));
}

// a function run by mjb_apply that calls its own batch fails instead of
// deadlocking
TEST_F(BatchTest, ApplyCallingItsOwnBatchFails) {
  MjModelPtr model = LoadModelFromString(kXml);
  BatchPtr batch(mjb_makeBatch(model.get(), kNsim, 2, 0, nullptr, 0));
  auto step_self = +[](const mjModel* m, mjData* d, int sim, void* arg) {
    mjb_step(static_cast<mjBatch*>(arg), &sim, 1, 1);
  };
  EXPECT_EQ(mjb_apply(batch.b, nullptr, 0, step_self, batch.b, 0), kNsim);
  EXPECT_THAT(mjb_error(batch.b, 0), HasSubstr("its own batch"));
  EXPECT_EQ(batch.Row(0)[0], 0);
}

// set_const in a persistent batch computes on scratch data: a sleeping
// simulation's mjData would skip its trees, and must keep its sleep state
TEST_F(BatchTest, PersistentSetConstIgnoresSleep) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast" viscosity="10" sleep_tolerance="0.01">
      <flag sleep="enable" gravity="disable" constraint="disable" contact="disable"/>
    </option>
    <worldbody>
      <body><freejoint/><geom type="box" size=".1 .2 .3" mass="1"/></body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  const mjModel* m = model.get();
  constexpr int nsim = 2;
  BatchPtr batch(mjb_makeBatch(m, nsim, 1, 1, nullptr, 0));
  ASSERT_THAT(batch.b, NotNull());
  mjBatch* b = batch.b;
  int qvel = Offset(m, mjSTATE_QVEL);
  for (int i = 0; i < nsim; ++i) batch.Row(i)[qvel] = 0.1;
  ASSERT_EQ(mjb_step(b, nullptr, 0, 1000), 0);

  auto awake = +[](const mjModel* m, mjData* d, int sim, void* arg) {
    static_cast<int*>(arg)[sim] = d->ntree_awake;
  };
  int ntree_awake[nsim];
  ASSERT_EQ(mjb_apply(b, nullptr, 0, awake, ntree_awake, 0), 0);
  ASSERT_EQ(ntree_awake[0], 0) << "the test needs a sleeping simulation";

  auto* mass =
      static_cast<mjtNum*>(mjb_expand(b, "body_mass", nullptr, nullptr));
  mass[1 * m->nbody + 1] = 2;
  ASSERT_EQ(mjb_setConst(b, nullptr, 0), 0);

  mjModel* m2 = mj_copyModel(nullptr, m);
  m2->body_mass[1] = 2;
  mjData* d2 = mj_makeData(m2);
  mj_setConst(m2, d2);
  auto* invweight =
      static_cast<mjtNum*>(mjb_expand(b, "body_invweight0", nullptr, nullptr));
  ASSERT_THAT(invweight, NotNull());
  EXPECT_EQ(invweight[0 * 2 * m->nbody + 2], m->body_invweight0[2]);
  EXPECT_EQ(invweight[1 * 2 * m->nbody + 2], m2->body_invweight0[2]);
  EXPECT_NE(m2->body_invweight0[2], m->body_invweight0[2]);
  mj_deleteData(d2);
  mj_deleteModel(m2);

  ASSERT_EQ(mjb_apply(b, nullptr, 0, awake, ntree_awake, 0), 0);
  EXPECT_EQ(ntree_awake[0], 0) << "set_const woke a simulation";
}

}  // namespace
}  // namespace mujoco
