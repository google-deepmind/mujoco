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

// Tests for engine/engine_support.c.

#include "src/engine/engine_support.h"

#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

using ::testing::DoubleNear;
using ::testing::ContainsRegex;
using ::testing::MatchesRegex;
using ::testing::Pointwise;
using ::testing::ElementsAreArray;
using JacobianTest = MujocoTest;
static const mjtNum max_abs_err = std::numeric_limits<float>::epsilon();

static constexpr char kJacobianTestingModel[] = R"(
<mujoco>
  <worldbody>
    <body name="distractor1" pos="0 0 .3">
      <freejoint/>
      <geom size=".1"/>
    </body>
    <body name="main">
      <freejoint/>
      <geom size=".1"/>
      <body pos=".1 0 0">
        <joint axis="0 1 0"/>
        <geom type="capsule" size=".03" fromto="0 0 0 .2 0 0"/>
      </body>
      <body pos="0 .1 0">
        <joint type="ball"/>
        <geom type="capsule" size=".03" fromto="0 0 0 0 .2 0"/>
        <body pos="0 .2 0">
          <joint type="slide" axis="1 1 1"/>
          <geom size=".05"/>
        </body>
      </body>
    </body>
    <body name="distractor2" pos="0 0 -.3">
      <freejoint/>
      <geom size=".1"/>
    </body>
  </worldbody>
</mujoco>
)";

// compare analytic and finite-differenced subtree-com Jacobian
TEST_F(JacobianTest, SubtreeJac) {
  mjModel* model = LoadModelFromString(kJacobianTestingModel);
  int nv = model->nv;
  int bodyid = mj_name2id(model, mjOBJ_BODY, "main");
  mjData* data = mj_makeData(model);
  mjtNum* jac_subtree = (mjtNum*) mju_malloc(sizeof(mjtNum)*3*nv);
  mjtNum* qpos = (mjtNum*) mju_malloc(sizeof(mjtNum)*model->nq);
  mjtNum* nudge = (mjtNum*) mju_malloc(sizeof(mjtNum)*nv);

  // all we need for Jacobians are kinematics and CoM-related quantities
  mj_kinematics(model, data);
  mj_comPos(model, data);

  // get subtree CoM Jacobian of free body
  mj_jacSubtreeCom(model, data, jac_subtree, bodyid);

  // save current subtree-com and qpos, clear nudge
  mjtNum subtree_com[3];
  mju_copy3(subtree_com, data->subtree_com+3*bodyid);
  mju_copy(qpos, data->qpos, model->nq);
  mju_zero(nudge, nv);

  // compare analytic Jacobian to finite-difference approximation
  static const mjtNum eps = 1e-6;
  for (int i=0; i < nv; i++) {
    // reset qpos, nudge i-th dof, update data->qpos, reset nudge
    mju_copy(data->qpos, qpos, model->nq);
    nudge[i] = 1;
    mj_integratePos(model, data->qpos, nudge, eps);
    nudge[i] = 0;

    // kinematics and comPos to get nudged com
    mj_kinematics(model, data);
    mj_comPos(model, data);

    // compare finite-differenced and analytic Jacobian
    for (int j=0; j < 3; j++) {
      mjtNum findiff = (data->subtree_com[3*bodyid+j] - subtree_com[j]) / eps;
      EXPECT_THAT(jac_subtree[nv*j+i], DoubleNear(findiff, eps));
    }
  }

  mju_free(nudge);
  mju_free(qpos);
  mju_free(jac_subtree);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// confirm that applying linear forces via the subtree-com Jacobian only creates
// the expected linear accelerations (no accelerations of internal joints)
TEST_F(JacobianTest, SubtreeJacNoInternalAcc) {
  mjModel* model = LoadModelFromString(kJacobianTestingModel);
  int nv = model->nv;
  int bodyid = mj_name2id(model, mjOBJ_BODY, "main");
  mjData* data = mj_makeData(model);
  mjtNum* jac_subtree = (mjtNum*) mju_malloc(sizeof(mjtNum)*3*nv);

  // all we need for Jacobians are kinematics and CoM-related quantities
  mj_kinematics(model, data);
  mj_comPos(model, data);

  // get subtree CoM Jacobian of free body
  mj_jacSubtreeCom(model, data, jac_subtree, bodyid);

  // uncomment for debugging
  // mju_printMat(jac_subtree, 3, nv);

  // call fwdPosition since we'll need the factorised mass matrix in the test
  mj_fwdPosition(model, data);

  // treating the subtree Jacobian as the projection of 3 axis-aligned unit
  // forces into joint space, solve for the resulting accelerations in-place
  mj_solveM(model, data, jac_subtree, jac_subtree, 3);

  // expect to find accelerations of magnitude 1/subtreemass in the first 3
  // coordinates of the free joint and 0s elsewhere, since applying forces to
  // the CoM should accelerate the whole mechanism without any internal motion
  int body_dofadr = model->body_dofadr[bodyid];
  mjtNum invtreemass = 1.0/model->body_subtreemass[bodyid];
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < nv; c++) {
      mjtNum expected = c - body_dofadr == r ? invtreemass : 0.0;
      EXPECT_THAT(jac_subtree[nv*r+c], DoubleNear(expected, max_abs_err));
    }
  }

  mju_free(jac_subtree);
  mj_deleteData(data);
  mj_deleteModel(model);
}

using Name2idTest = MujocoTest;

static constexpr char name2idTestingModel[] = R"(
<mujoco>
    <asset>
      <texture name="texture1" type="2d" builtin="checker" rgb1="1 1 1"
       rgb2="1 1 1" width="300" height="300" mark="none"/>
      <material name="material1" texture="texture1" texrepeat="1 1"
       texuniform="true" reflectance=".2"/>
    </asset>

    <asset>
      <mesh name="mesh1" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>

  <worldbody>
    <light name="light1" pos="0 0 1"/>
    <site name="site1" pos="0 0 .3" size=".01"/>
    <site name="site2" pos="-.1 -.1 -.1" size=".01"/>
    <camera name="camera1" pos="0 -1.3 .5" xyaxes="1 0 0 0 1 2"/>
    <body name="body1">
      <joint axis="0 1 0" name="joint1"/>
      <geom size="1" name="body1_geom1"/>
      <geom size="1" name="body1_geom2"/>
    </body>
    <body name="body2">
      <joint axis="0 1 0" name="joint2"/>
      <geom size="1" name="body2_geom1"/>
      <geom size="1" name="camera1"/>
    </body>
    <body name="">
    </body>
  </worldbody>

  <tendon>
    <spatial name="tendon1" limited="true" range="0 0.35" width="0.003">
      <site site="site1"/>
      <site site="site2"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="actuator1" joint="joint1" gear="1"/>
  </actuator>

  <sensor>
    <accelerometer name="sensor1" site="site1"/>
  </sensor>
</mujoco>
)";

TEST_F(Name2idTest, FindIds) {
    mjModel* model = LoadModelFromString(name2idTestingModel);

    EXPECT_THAT(mj_name2id(model, mjOBJ_BODY, "world"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_BODY, "body1"), 1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_BODY, "body2"), 2);
    EXPECT_THAT(mj_name2id(model, mjOBJ_GEOM, "body1_geom1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_GEOM, "body1_geom2"), 1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_JOINT, "joint2"), 1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_MESH, "mesh1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_LIGHT, "light1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_CAMERA, "camera1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_SITE, "site2"), 1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_MATERIAL, "material1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_TEXTURE, "texture1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_TENDON, "tendon1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_ACTUATOR, "actuator1"), 0);
    EXPECT_THAT(mj_name2id(model, mjOBJ_SENSOR, "sensor1"), 0);

    mj_deleteModel(model);
}

TEST_F(Name2idTest,  MissingIds) {
    mjModel* model = LoadModelFromString(name2idTestingModel);

    EXPECT_THAT(mj_name2id(model, mjOBJ_BODY, "abody3"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_GEOM, "abody2_geom2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_JOINT, "joint3"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_MESH, "amesh2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_LIGHT, "alight2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_CAMERA, "acamera2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_SITE, "asite3"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_MATERIAL, "amaterial2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_TEXTURE, "atexture2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_TENDON, "atendon2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_ACTUATOR, "aactuator2"), -1);
    EXPECT_THAT(mj_name2id(model, mjOBJ_SENSOR, "asensor2"), -1);

    mj_deleteModel(model);
}

TEST_F(Name2idTest, EmptyIds) {
    mjModel* model = LoadModelFromString(name2idTestingModel);

    EXPECT_THAT(mj_name2id(model, mjOBJ_BODY, ""), -1);

    mj_deleteModel(model);
}

TEST_F(Name2idTest, Namespaces) {
    mjModel* model = LoadModelFromString(name2idTestingModel);

    EXPECT_THAT(mj_name2id(model, mjOBJ_GEOM, "camera1"), 3);

    mj_deleteModel(model);
}

using VersionTest = MujocoTest;

TEST_F(VersionTest, MjVersion) {
  EXPECT_EQ(mj_version(), mjVERSION_HEADER);
}

TEST_F(VersionTest, MjVersionString) {
#if GTEST_USES_SIMPLE_RE == 1
  auto regex_matcher = ContainsRegex("^\\d+\\.\\d+\\.\\d+");
#else
  auto regex_matcher = MatchesRegex("^[0-9]+\\.[0-9]+\\.[0-9]+(-[0-9a-z]+)?$");
#endif
  EXPECT_THAT(std::string(mj_versionString()), regex_matcher);
}


using SupportTest = MujocoTest;

// utility: generate two random quaternions with a given angle difference
void randomQuatPair(mjtNum qa[4], mjtNum qb[4], mjtNum angle, int seed) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);

  // sample qa = qb
  for (int i=0; i < 4; i++) {
    qa[i] = qb[i] = dist(rng);
  }
  mju_normalize4(qa);
  mju_normalize4(qb);

  // integrate qb in random direction by angle
  mjtNum dir[3];
  for (int i=0; i < 3; i++) {
    dir[i] = dist(rng);
  }
  mju_normalize3(dir);
  mju_quatIntegrate(qb, dir, angle);
}

static constexpr char ballJointModel[] = R"(
<mujoco>
  <worldbody>
    <body>
      <joint type="ball"/>
      <geom size="1"/>
    </body>
  </worldbody>
</mujoco>
)";

TEST_F(SupportTest, DifferentiatePosSubQuat) {
  const mjtNum eps = 1e-12;  // epsilon for float comparison

  mjModel* model = LoadModelFromString(ballJointModel);

  int seed = 1;
  for (mjtNum angle : {0.0, 1e-5, 1e-2}) {
    for (mjtNum dt : {1e-6, 1e-3, 1e-1}) {
      // random quaternion pair with given angle difference
      mjtNum qpos1[4], qpos2[4];
      randomQuatPair(qpos1, qpos2, angle, seed++);

      // get velocity given timestep
      mjtNum qvel[3];
      mj_differentiatePos(model, qvel, dt, qpos1, qpos2);

      // equivalent computation
      mjtNum qneg[4], qdif[4], qvel_expect[3];
      mju_negQuat(qneg, qpos1);
      mju_mulQuat(qdif, qneg, qpos2);
      mju_quat2Vel(qvel_expect, qdif, dt);

      // expect numerical equality
      EXPECT_THAT(AsVector(qvel, 3), Pointwise(DoubleNear(eps), qvel_expect));
    }
  }

  mj_deleteModel(model);
}

static const char* const kDefaultModel = "testdata/model.xml";

TEST_F(SupportTest, GetSetStateStepEqual) {
  const std::string xml_path = GetTestDataFilePath(kDefaultModel);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(3);
  std::normal_distribution<double> dist(0, .01);

  // set controls and applied joint forces to random values
  for (int i=0; i < model->nu; i++) data->ctrl[i] = dist(rng);
  for (int i=0; i < model->nv; i++) data->qfrc_applied[i] = dist(rng);
  for (int i=0; i < model->neq; i++) data->eq_active[i] = dist(rng) > 0;

  // take one step
  mj_step(model, data);

  int spec = mjSTATE_INTEGRATION;
  int size = mj_stateSize(model, spec);

  // save the initial state and step
  std::vector<mjtNum> state0a(size);
  mj_getState(model, data, state0a.data(), spec);

  // get the initial state, expect equality
  std::vector<mjtNum> state0b(size);
  mj_getState(model, data, state0b.data(), spec);
  EXPECT_EQ(state0a, state0b);

  // take one step
  mj_step(model, data);

  // save the resulting state
  std::vector<mjtNum> state1a(size);
  mj_getState(model, data, state1a.data(), spec);

  // expect the state to be different after stepping
  EXPECT_THAT(state0a, testing::Ne(state1a));

  // reset to the saved state, step again, get the resulting state
  mj_setState(model, data, state0a.data(), spec);
  mj_step(model, data);
  std::vector<mjtNum> state1b(size);
  mj_getState(model, data, state1b.data(), spec);

  // expect the state to be the same after re-stepping
  EXPECT_EQ(state1a, state1b);

  mj_deleteData(data);
  mj_deleteModel(model);
}

using AddMTest = MujocoTest;

TEST_F(AddMTest, DenseSameAsSparse) {
  mjModel* m = LoadModelFromPath("humanoid100/humanoid100.xml");
  mjData* d = mj_makeData(m);
  int nv = m->nv;

  // force use of sparse matrices
  m->opt.jacobian = mjJAC_SPARSE;

  // warm-up rollout to get a typical state
  while (d->time < 2) {
    mj_step(m, d);
  }

  // dense zero matrix
  std::vector<mjtNum> dst_sparse = std::vector(nv * nv, 0.0);

  // sparse zero matrix
  std::vector<mjtNum> dst_dense = std::vector(nv * nv, 0.0);
  std::vector<int> rownnz = std::vector(nv, nv);
  std::vector<int> rowadr = std::vector(nv, 0);
  std::vector<int> colind = std::vector(nv * nv, 0);

  // set sparse structure
  for (int i = 0; i < nv; i++) {
    rowadr[i] = i * nv;
    for (int j = 0; j < nv; j++) {
      colind[rowadr[i] + j] = j;
    }
  }

  // sparse addM
  mj_addM(m, d, dst_sparse.data(), rownnz.data(),
          rowadr.data(), colind.data());

  // dense addM
  mj_addM(m, d, dst_dense.data(), NULL, NULL, NULL);

  // dense comparison, should be same matrix
  EXPECT_THAT(dst_dense, ElementsAreArray(dst_sparse));

  // clean up
  mj_deleteData(d);
  mj_deleteModel(m);
}

static const char* const kIlslandEfcPath =
    "engine/testdata/island/island_efc.xml";

TEST_F(SupportTest, MulMIsland) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // allocate vec, fill with arbitrary values
  mjtNum* vec = (mjtNum*) mju_malloc(sizeof(mjtNum)*model->nv);
  for (int i=0; i < model->nv; i++) {
    vec[i] = 0.2 + 0.3*i;
  }

  // simulate for 0.2 seconds
  mj_resetData(model, data);
  while (data->time < 0.2) {
    mj_step(model, data);
  }
  mj_forward(model, data);

  // multiply by Mass matrix: Mvec = M * vec
  mjtNum* Mvec = (mjtNum*) mju_malloc(sizeof(mjtNum)*data->nefc);
  mj_mulM(model, data, Mvec, vec);

  // iterate over islands
  for (int i=0; i < data->nisland; i++) {
    // allocate dof vectors for island
    int dofnum = data->island_dofnum[i];
    mjtNum* vec_i = (mjtNum*)mju_malloc(sizeof(mjtNum) * dofnum);
    mjtNum* Mvec_i = (mjtNum*)mju_malloc(sizeof(mjtNum) * dofnum);

    // copy values into vec_i
    int* dofind = data->island_dofind + data->island_dofadr[i];
    for (int j=0; j < dofnum; j++) {
      vec_i[j] = vec[dofind[j]];
    }

    // === compressed: use vec_i

    // multiply by Jacobian, for this island
    int flg_vecunc = 0;
    mj_mulM_island(model, data, Mvec_i, vec_i, i, flg_vecunc);

    // expect corresponding values to match
    for (int j=0; j < dofnum; j++) {
      EXPECT_THAT(Mvec_i[j], DoubleNear(Mvec[dofind[j]], 1e-12));
    }

    // === uncompressed: use vec
    mju_zero(Mvec_i, dofnum);  // clear output

    // multiply by Jacobian, for this island
    flg_vecunc = 1;
    mj_mulM_island(model, data, Mvec_i, vec, i, flg_vecunc);

    // expect corresponding values to match
    for (int j=0; j < dofnum; j++) {
      EXPECT_THAT(Mvec_i[j], DoubleNear(Mvec[dofind[j]], 1e-12));
    }

    mju_free(vec_i);
    mju_free(Mvec_i);
  }

  mju_free(Mvec);
  mju_free(vec);
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
