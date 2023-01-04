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

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::ContainsRegex;
using ::testing::MatchesRegex;
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

  // all we need for Jacobians are kinematics and CoM-related quantitites
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
  for (int i=0; i<nv; i++) {
    // reset qpos, nudge i-th dof, update data->qpos, reset nudge
    mju_copy(data->qpos, qpos, model->nq);
    nudge[i] = 1;
    mj_integratePos(model, data->qpos, nudge, eps);
    nudge[i] = 0;

    // kinematics and comPos to get nudged com
    mj_kinematics(model, data);
    mj_comPos(model, data);

    // compare finite-differenced and analytic Jacobian
    for (int j=0; j<3; j++) {
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

  // all we need for Jacobians are kinematics and CoM-related quantitites
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

}  // namespace
}  // namespace mujoco
