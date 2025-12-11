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

// Tests for engine/{engine_support.c and engine_core_util.c}

#include "src/engine/engine_core_util.h"
#include "src/engine/engine_support.h"

#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::vector;
using ::testing::ContainsRegex;  // NOLINT
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::MatchesRegex;
using ::testing::Ne;
using ::testing::NotNull;
using ::testing::Pointwise;

using AngMomMatTest = MujocoTest;

static constexpr char AngMomTestingModel[] = R"(
<mujoco>
  <option>
    <flag gravity="disable"/>
  </option>
  <worldbody>
    <body name="link1" pos="0 0 0.5">
      <freejoint/>
      <geom type="ellipsoid" size="0.15 0.17 0.19" quat="1 .2 .3 .4"/>
      <body name="link2" >
        <joint type="hinge" axis="1 0 0" />
        <geom type="capsule" size="0.05" fromto="0 0 0  0 0.5 0"/>
        <body pos="0 0.6 0">
          <joint type="slide" axis="1 0 0"/>
          <geom type="capsule" size="0.05 0.2" quat="0.707 0 0.707 0"/>
          <body name="link3">
            <joint type="ball" pos="0.2 0 0"/>
            <geom type="capsule" pos="0.2 0 0" size="0.03 0.4"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <keyframe>
    <key qvel="0 0 0 .1 .2 .3 .4 .5 .4 .3 .2"/>
  </keyframe>
</mujoco>
)";

// compare subtree angular momentum computed in two ways
TEST_F(AngMomMatTest, CompareAngMom) {
  char error[1024];
  mjModel* model =
      LoadModelFromString(AngMomTestingModel, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  int nv = model->nv;
  int bodyid = mj_name2id(model, mjOBJ_BODY, "link1");

  mjData* data = mj_makeData(model);

  // reset to the keyframe with some angular velocities
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);

  // get the reference value of angular momentum
  mj_subtreeVel(model, data);
  mjtNum angmom_ref[3];
  mju_copy3(angmom_ref, data->subtree_angmom+3*bodyid);

  // compute angular momentum using the angular momentum matrix
  mjtNum* angmom_mat = (mjtNum*) mju_malloc(sizeof(mjtNum)*3*nv);
  mj_angmomMat(model, data, angmom_mat, bodyid);
  mjtNum angmom_test[3];
  mju_mulMatVec(angmom_test, angmom_mat, data->qvel, 3, nv);

  // compare the two angular momentum values
  static const mjtNum tol = 1e-8;
  for (int i = 0; i < 3; i++) {
    EXPECT_THAT(angmom_ref[i], DoubleNear(angmom_test[i], tol));
  }

  mju_free(angmom_mat);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// compare subtree angular momentum matrix: analytical and findiff
TEST_F(AngMomMatTest, CompareAngMomMats) {
  char error[1024];
  mjModel* model =
      LoadModelFromString(AngMomTestingModel, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  int nv = model->nv;
  int bodyid = mj_name2id(model, mjOBJ_BODY, "link1");
  mjData* data = mj_makeData(model);
  mjtNum* angmom_mat = (mjtNum*) mju_malloc(sizeof(mjtNum)*3*nv);
  mjtNum* angmom_mat_fd = (mjtNum*) mju_malloc(sizeof(mjtNum)*3*nv);

  // reset to the keyframe with some angular velocities
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);

  // compute the angular momentum matrix using the analytical method
  mj_angmomMat(model, data, angmom_mat, bodyid);

  // compute the angular momentum matrix using finite differences
  static const mjtNum eps = 1e-6;
  for (int i = 0; i < nv; i++) {
    // reset vel, forward nudge i-th dof, get angmom
    mju_copy(data->qvel, model->key_qvel, model->nv);
    data->qvel[i] += eps;
    mj_forward(model, data);
    mj_subtreeVel(model, data);
    mjtNum agmf[3];
    mju_copy3(agmf, data->subtree_angmom+3*bodyid);

    // reset vel, backward nudge i-th dof, get angmom
    mju_copy(data->qvel, model->key_qvel, model->nv);
    data->qvel[i] -= eps;
    mj_forward(model, data);
    mj_subtreeVel(model, data);
    mjtNum agmb[3];
    mju_copy3(agmb, data->subtree_angmom+3*bodyid);

    // finite-difference the angmom matrix
    for (int j = 0; j < 3; j++) {
      angmom_mat_fd[nv*j+i] = (agmf[j] - agmb[j]) / (2 * eps);
    }
  }

  // compare the two matrices
  static const mjtNum tol = 1e-8;
  for (int i = 0; i < 3*nv; i++) {
    EXPECT_THAT(angmom_mat_fd[i], DoubleNear(angmom_mat[i], tol));
  }

  mju_free(angmom_mat_fd);
  mju_free(angmom_mat);
  mj_deleteData(data);
  mj_deleteModel(model);
}

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
  char error[1024];
  mjModel* model =
      LoadModelFromString(kJacobianTestingModel, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
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
  char error[1024];
  mjModel* model =
      LoadModelFromString(kJacobianTestingModel, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
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

static constexpr char kQuat[] = R"(
<mujoco>
  <worldbody>
    <body name="query">
      <joint type="ball"/>
      <geom size="1"/>
      <site name="query" pos=".1 .2 .3"/>
    </body>
  </worldbody>
  <keyframe>
    <key qvel="2 3 5"/>
  </keyframe>
</mujoco>
)";

static constexpr char kFreeBall[] = R"(
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
        <body pos=".2 0 0">
          <joint type="ball" stiffness="20"/>
          <geom type="capsule" size=".03" fromto="0 0 0 0 .2 0"/>
          <body name="query" pos="0 .2 0">
            <joint type="slide" axis="1 1 1"/>
            <geom size=".05"/>
            <site name="query" pos=".1 .2 .3"/>
          </body>
        </body>
      </body>
    </body>
    <body name="distractor2" pos="0 0 -.3">
      <freejoint/>
      <geom size=".1"/>
    </body>
  </worldbody>
  <keyframe>
    <key qvel="1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"/>
  </keyframe>
</mujoco>
)";

static constexpr char kQuatlessPendulum[] = R"(
<mujoco>
  <option integrator="implicit">
    <flag constraint="disable"/>
  </option>
  <worldbody>
    <body pos="0.15 0 0">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 .1 0 0"/>
      <body pos="0.1 0 0">
        <joint type="slide" axis="1 0 0" stiffness="200"/>
        <geom type="capsule" size="0.015" fromto="-.1 0 0 .1 0 0"/>
        <body pos=".1 0 0">
          <joint axis="1 0 0"/>
          <joint axis="0 1 0"/>
          <joint axis="0 0 1"/>
          <geom type="box" size=".02" fromto="0 0 0 0 .1 0"/>
          <body name="query" pos="0 .1 0">
            <joint axis="1 0 0"/>
            <geom type="capsule" size="0.02" fromto="0 0 0 0 .1 0"/>
            <site name="query" pos=".1 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
)";

static constexpr char kTelescope[] = R"(
<mujoco>
  <worldbody>
    <body>
      <joint type="ball"/>
      <geom type="capsule" size="0.02" fromto="0 .02 0 .1 .02 0"/>
      <body pos=".1 .02 0">
        <joint type="slide" axis="1 0 0"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 .1 0 0"/>
        <body pos=".1 .02 0">
          <joint type="slide" axis="1 0 0"/>
          <geom type="capsule" size="0.02" fromto="0 0 0 .1 0 0"/>
          <body pos=".1 .02 0" name="query">
            <joint type="slide" axis="1 0 0"/>
            <geom type="capsule" size="0.02" fromto="0 0 0 .1 0 0"/>
            <site name="query" pos=".1 0 0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <keyframe>
    <key qvel="1 1 1 1 1 1"/>
  </keyframe>
</mujoco>
)";

static constexpr char kHinge[] = R"(
<mujoco>
  <worldbody>
    <body name="query">
      <joint name="link1" axis="0 1 0"/>
      <geom type="capsule" size=".02" fromto="0 0 0 0 0 -1"/>
      <site name="query" pos="0 0 -1"/>
    </body>
  </worldbody>

  <keyframe>
    <key qpos="1" qvel="1"/>
  </keyframe>
</mujoco>
)";

// compare mj_jacDot with finite-differenced mj_jac
TEST_F(JacobianTest, JacDot) {
  for (auto xml : {kHinge, kQuat, kTelescope, kFreeBall, kQuatlessPendulum}) {
    char error[1024];
    mjModel* model = LoadModelFromString(xml, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;
    int nv = model->nv;
    mjData* data = mj_makeData(model);

    // load keyframe if present, step for a bit
    if (model->nkey) mj_resetDataKeyframe(model, data, 0);
    while (data->time < 0.1) {
      mj_step(model, data);
    }

    // minimal call required for mj_jacDot outputs to be valid
    mj_kinematics(model, data);
    mj_comPos(model, data);
    mj_comVel(model, data);

    // get bodyid
    int bodyid = mj_name2id(model, mjOBJ_BODY, "query");
    EXPECT_GT(bodyid, 0);

    // get site position
    int siteid = mj_name2id(model, mjOBJ_SITE, "query");
    EXPECT_GT(siteid, -1);
    mjtNum point[3];
    mju_copy3(point, data->site_xpos+3*siteid);

    // jac, jac_dot
    vector<mjtNum> jacp(3*nv);
    vector<mjtNum> jacr(3*nv);
    mj_jac(model, data, jacp.data(), jacr.data(), point, bodyid);
    vector<mjtNum> jacp_dot(3*nv);
    vector<mjtNum> jacr_dot(3*nv);
    mj_jacDot(model, data, jacp_dot.data(), jacr_dot.data(), point, bodyid);

    // jac_h: jacobian after integrating qpos with a timestep of h
    mjtNum h = 1e-7;
    mj_integratePos(model, data->qpos, data->qvel, h);
    mj_kinematics(model, data);
    mj_comPos(model, data);
    vector<mjtNum> jacp_h(3*nv);
    vector<mjtNum> jacr_h(3*nv);
    mju_copy3(point, data->site_xpos+3*siteid);  // get updated site position
    mj_jac(model, data, jacp_h.data(), jacr_h.data(), point, bodyid);

    // jac_dot_h finite-difference approximation
    vector<mjtNum> jacp_dot_h(3*nv);
    mju_sub(jacp_dot_h.data(), jacp_h.data(), jacp.data(), 3*nv);
    mju_scl(jacp_dot_h.data(), jacp_dot_h.data(), 1/h, 3*nv);
    vector<mjtNum> jacr_dot_h(3*nv);
    mju_sub(jacr_dot_h.data(), jacr_h.data(), jacr.data(), 3*nv);
    mju_scl(jacr_dot_h.data(), jacr_dot_h.data(), 1/h, 3*nv);

    // compare finite-differenced and analytic
    mjtNum tol = 1e-5;
    EXPECT_THAT(jacp_dot, Pointwise(DoubleNear(tol), jacp_dot_h));
    EXPECT_THAT(jacr_dot, Pointwise(DoubleNear(tol), jacr_dot_h));

    mj_deleteData(data);
    mj_deleteModel(model);
  }
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
    char error[1024];
    mjModel* model =
        LoadModelFromString(name2idTestingModel, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;

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
    char error[1024];
    mjModel* model =
        LoadModelFromString(name2idTestingModel, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;

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
    char error[1024];
    mjModel* model =
        LoadModelFromString(name2idTestingModel, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;

    EXPECT_THAT(mj_name2id(model, mjOBJ_BODY, ""), -1);

    mj_deleteModel(model);
}

TEST_F(Name2idTest, Namespaces) {
    char error[1024];
    mjModel* model =
        LoadModelFromString(name2idTestingModel, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;

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

  char error[1024];
  mjModel* model = LoadModelFromString(ballJointModel, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

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

  int signature = mjSTATE_INTEGRATION;
  int size = mj_stateSize(model, signature);

  // save the initial state and step
  vector<mjtNum> state0a(size);
  mj_getState(model, data, state0a.data(), signature);

  // get the initial state, expect equality
  vector<mjtNum> state0b(size);
  mj_getState(model, data, state0b.data(), signature);
  EXPECT_EQ(state0a, state0b);

  // take one step
  mj_step(model, data);

  // save the resulting state
  vector<mjtNum> state1a(size);
  mj_getState(model, data, state1a.data(), signature);

  // expect the state to be different after stepping
  EXPECT_THAT(state0a, testing::Ne(state1a));

  // reset to the saved state, step again, get the resulting state
  mj_setState(model, data, state0a.data(), signature);
  mj_step(model, data);
  vector<mjtNum> state1b(size);
  mj_getState(model, data, state1b.data(), signature);

  // expect the state to be the same after re-stepping
  EXPECT_EQ(state1a, state1b);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(SupportTest, CopyState) {
  const std::string xml_path = GetTestDataFilePath(kDefaultModel);
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);

  mjData* src = mj_makeData(m);
  mjData* dst = mj_makeData(m);

  // init both datas to default
  mj_resetData(m, src);
  mj_resetData(m, dst);

  // modify d_src
  src->time = 1.23;
  for (int i=0; i < m->nq; ++i) src->qpos[i] = i*0.1;
  for (int i=0; i < m->nv; ++i) src->qvel[i] = i*0.2;
  for (int i=0; i < m->na; ++i) src->act[i] = i*0.3;
  for (int i=0; i < m->nu; ++i) src->ctrl[i] = i*0.4;

  for (int i=0; i < m->neq; ++i) src->eq_active[i] = 1 - m->eq_active0[i];

  // check that states differ
  EXPECT_NE(src->time, dst->time);
  EXPECT_THAT(AsVector(src->qpos, m->nq), Ne(AsVector(dst->qpos, m->nq)));
  EXPECT_THAT(AsVector(src->ctrl, m->nu), Ne(AsVector(dst->ctrl, m->nu)));

  // copy state with signature
  int signature = mjSTATE_FULLPHYSICS | mjSTATE_EQ_ACTIVE;
  mj_copyState(m, src, dst, signature);

  // check copied components
  EXPECT_EQ(dst->time, src->time);
  EXPECT_EQ(AsVector(dst->qpos, m->nq), AsVector(src->qpos, m->nq));
  EXPECT_EQ(AsVector(dst->qvel, m->nv), AsVector(src->qvel, m->nv));
  EXPECT_EQ(AsVector(dst->act, m->na), AsVector(src->act, m->na));
  EXPECT_EQ(AsVector(dst->eq_active, m->neq), AsVector(src->eq_active, m->neq));

  // check non-copied components (CTRL not in signature)
  EXPECT_THAT(AsVector(dst->ctrl, m->nu), Ne(AsVector(src->ctrl, m->nu)));
  EXPECT_EQ(AsVector(dst->ctrl, m->nu), vector<mjtNum>(m->nu, 0.0));

  mj_deleteData(src);
  mj_deleteData(dst);
  mj_deleteModel(m);
}

TEST_F(SupportTest, ExtractState) {
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

  // take a state that will be used as src
  int srcsig = mjSTATE_TIME | mjSTATE_QPOS | mjSTATE_QVEL | mjSTATE_CTRL;
  int srcsize = mj_stateSize(model, srcsig);
  vector<mjtNum> srcstate(srcsize);
  mj_getState(model, data, srcstate.data(), srcsig);

  // extract a subset consisting of only a single bit in srcsig
  int dstsig1 = mjSTATE_CTRL;
  int dstsize1 = mj_stateSize(model, dstsig1);
  EXPECT_LT(dstsize1, srcsize);
  EXPECT_EQ(dstsize1, model->nu);
  vector<mjtNum> dststate1(dstsize1);
  mj_extractState(model, srcstate.data(), srcsig, dststate1.data(), dstsig1);
  EXPECT_EQ(dststate1, AsVector(data->ctrl, model->nu));

  // extract a subset consisting of multiple non-consecutive bits in srcsig
  int dstsig2 = mjSTATE_QPOS | mjSTATE_CTRL;
  int dstsize2 = mj_stateSize(model, dstsig2);
  EXPECT_LT(dstsize2, srcsize);
  EXPECT_EQ(dstsize2, model->nq + model->nu);
  vector<mjtNum> dststate2(dstsize2);
  mj_extractState(model, srcstate.data(), srcsig, dststate2.data(), dstsig2);
  EXPECT_EQ(AsVector(dststate2.data(), model->nq),
            AsVector(data->qpos, model->nq));
  EXPECT_EQ(AsVector(dststate2.data() + model->nq, model->nu),
            AsVector(data->ctrl, model->nu));

  // test that an error is correctly raised if dstsig is not a subset of srcsig
  static int error_count;
  static char last_error_msg[128];
  error_count = 0;
  last_error_msg[0] = '\0';
  auto* error_handler = +[](const char* msg) {
    std::strncpy(last_error_msg, msg, sizeof(last_error_msg));
    ++error_count;
  };

  auto* old_mju_user_error = mju_user_error;
  mju_user_error = error_handler;

  mj_extractState(model, nullptr, srcsig, nullptr, mjSTATE_QFRC_APPLIED);
  EXPECT_EQ(error_count, 1);
  EXPECT_EQ(std::string_view(last_error_msg),
            "mj_extractState: dstsig is not a subset of srcsig");

  mj_extractState(model, nullptr, -1, nullptr, mjSTATE_QFRC_APPLIED);
  EXPECT_EQ(error_count, 2);
  EXPECT_EQ(std::string_view(last_error_msg),
            "mj_extractState: invalid srcsig -1 < 0");

  mju_user_error = old_mju_user_error;

  mj_deleteData(data);
  mj_deleteModel(model);
}

using InertiaTest = MujocoTest;

static const char* const kInertiaPath = "engine/testdata/inertia.xml";

TEST_F(InertiaTest, AddMdenseSameAsSparse) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
  int nv = m->nv;

  mjData* d = mj_makeData(m);

  mj_step(m, d);

  // dense matrix, all values are 3.0
  vector<mjtNum> dst_dense(nv * nv, 3.0);

  // sparse matrix, all values are 3.0
  vector<mjtNum> dst_sparse(nv * nv, 3.0);
  vector<int> rownnz(nv, nv);
  vector<int> rowadr(nv, 0);
  vector<int> colind(nv * nv, 0);

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
  mj_addM(m, d, dst_dense.data(), nullptr, nullptr, nullptr);

  // dense comparison (lower triangle)
  for (int i=0; i < nv; i++) {
    for (int j=0; j < nv; j++) {
      EXPECT_EQ(dst_dense[i*nv+j], dst_sparse[i*nv+j]);
    }
  }

  // clean up
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(InertiaTest, mulM) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error;
  int nv = model->nv;

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // dense M matrix
  vector<mjtNum> Mdense(nv*nv);
  mj_fullM(model, Mdense.data(), data->qM);

  // arbitrary RHS vector
  vector<mjtNum> vec(nv);
  for (int i=0; i < nv; i++) vec[i] = vec[i] = 20 + 30*i;

  // multiply directly
  vector<mjtNum> res1(nv, 0);
  mju_mulMatVec(res1.data(), Mdense.data(), vec.data(), nv, nv);

  // multiply with mj_mulM
  vector<mjtNum> res2(nv, 0);
  mj_mulM(model, data, res2.data(), vec.data());

  // expect vectors to match to floating point precision
  EXPECT_THAT(res1, Pointwise(DoubleNear(1e-10), res2));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(InertiaTest, mulM2) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error;
  int nv = model->nv;

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // arbitrary RHS vector
  vector<mjtNum> vec(nv);
  for (int i=0; i < nv; i++) vec[i] = .2 + .3*i;

  // multiply sqrtMvec = M^1/2 * vec
  vector<mjtNum> sqrtMvec(nv);
  mj_mulM2(model, data, sqrtMvec.data(), vec.data());

  // multiply Mvec = M * vec
  vector<mjtNum> Mvec(nv);
  mj_mulM(model, data, Mvec.data(), vec.data());

  // compute vec' * M * vec in two different ways, expect them to match
  mjtNum sqrtMvec2 = mju_dot(sqrtMvec.data(), sqrtMvec.data(), nv);
  mjtNum vecMvec = mju_dot(vec.data(), Mvec.data(), nv);
  EXPECT_FLOAT_EQ(sqrtMvec2, vecMvec);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(InertiaTest, FullM) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
  int nv = m->nv;

  // forward dynamics, populate qM and qLD
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // get dense mass matrix from qM using mj_fullM
  vector<mjtNum> M(nv * nv);
  mj_fullM(m, M.data(), d->qM);

  // get dense mass matrix from M using mju_sparse2dense
  vector<mjtNum> M_CSR(nv * nv);
  mju_sparse2dense(M_CSR.data(), d->M, nv, nv,
                   m->M_rownnz, m->M_rowadr, m->M_colind);

  // expect lower triangles to match exactly
  for (int i = 0; i < nv; ++i) {
    for (int j = 0; j <= i; ++j) {
      EXPECT_EQ(M[i * nv + j], M_CSR[i * nv + j]);
    }
  }

  // get dense LTDL factor (D on the diagonal)
  vector<mjtNum> LD(nv * nv);
  mju_sparse2dense(LD.data(), d->qLD, nv, nv,
                   m->M_rownnz, m->M_rowadr, m->M_colind);

  // extract L and D from LD
  vector<mjtNum> L = LD;
  vector<mjtNum> D(nv * nv, 0.0);
  for (int i = 0; i < nv; i++) {
    D[i * nv + i] = LD[i * nv + i];
    L[i * nv + i] = 1.0;
  }

  // compute DL = D * L
  vector<mjtNum> DL(nv * nv, 0.0);
  mju_mulMatMat(DL.data(), D.data(), L.data(), nv, nv, nv);

  // compute the triple product P = L^T * D * L
  vector<mjtNum> P(nv * nv, 0.0);
  mju_mulMatTMat(P.data(), L.data(), DL.data(), nv, nv, nv);

  // expect M and P to match to high precision
  EXPECT_THAT(M, Pointwise(DoubleNear(1e-10), P));

  mj_deleteData(d);
  mj_deleteModel(m);
}

static constexpr char GeomDistanceTestingModel1[] = R"(
<mujoco>
  <option>
    <flag nativeccd="enable"/>
  </option>
  <asset>
    <mesh name="box" scale=".1 .1 .1" vertex="0 0 0  1 0 0  0 1 0  1 1 0
                                              0 0 1  1 0 1  0 1 1  1 1 1"/>
  </asset>

  <worldbody>
    <geom type="plane" size="1 1 1"/>
    <geom pos="0 0 1" size="0.2"/>
    <geom pos="1 0 1" size="0.3"/>
    <geom type="mesh" mesh="box"/>
  </worldbody>
</mujoco>
)";

static constexpr char GeomDistanceTestingModel2[] = R"(
<mujoco>
  <worldbody>
    <geom type="sphere" size=".1"/>
    <geom type="ellipsoid" size=".1 .1 .1" pos="0 0 1"/>
  </worldbody>
</mujoco>
)";

TEST_F(SupportTest, GeomDistance) {
  char error[1024];
  mjModel* model =
      LoadModelFromString(GeomDistanceTestingModel1, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);
  mj_kinematics(model, data);

  // plane-sphere, distmax too small
  mjtNum distmax = 0.5;
  EXPECT_EQ(mj_geomDistance(model, data, 0, 1, distmax, nullptr), 0.5);
  mjtNum fromto[6];
  EXPECT_EQ(mj_geomDistance(model, data, 0, 1, distmax, fromto), 0.5);
  EXPECT_THAT(fromto, Pointwise(Eq(), vector<mjtNum>{0, 0, 0, 0, 0, 0}));

  // plane-sphere
  distmax = 1.0;
  EXPECT_DOUBLE_EQ(mj_geomDistance(model, data, 0, 1, 1.0, fromto), 0.8);
  mjtNum eps = 1e-12;
  EXPECT_THAT(fromto, Pointwise(DoubleNear(eps),
                                vector<mjtNum>{0, 0, 0, 0, 0, 0.8}));

  // sphere-plane
  EXPECT_DOUBLE_EQ(mj_geomDistance(model, data, 1, 0, 1.0, fromto), 0.8);
  EXPECT_THAT(fromto, Pointwise(DoubleNear(eps),
                                vector<mjtNum>{0, 0, 0.8, 0, 0, 0}));

  // sphere-sphere
  EXPECT_DOUBLE_EQ(mj_geomDistance(model, data, 1, 2, 1.0, fromto), 0.5);
  EXPECT_THAT(fromto, Pointwise(DoubleNear(eps),
                                vector<mjtNum>{.2, 0, 1, .7, 0, 1}));

  // sphere-sphere, flipped order
  EXPECT_DOUBLE_EQ(mj_geomDistance(model, data, 2, 1, 1.0, fromto), 0.5);
  EXPECT_THAT(fromto, Pointwise(DoubleNear(eps),
                                vector<mjtNum>{.7, 0, 1, .2, 0, 1}));

  // mesh-sphere (close distmax)
  distmax = 0.701;
  eps = model->opt.ccd_tolerance;
  EXPECT_THAT(mj_geomDistance(model, data, 3, 1, distmax, fromto),
              DoubleNear(0.7, eps));
  EXPECT_THAT(fromto, Pointwise(DoubleNear(eps),
                                vector<mjtNum>{0, 0, .1, 0, 0, .8}));

  // mesh-sphere (far distmax)
  distmax = 1.0;
  EXPECT_THAT(mj_geomDistance(model, data, 3, 1, distmax, fromto),
              DoubleNear(0.7, eps));
  EXPECT_THAT(fromto, Pointwise(DoubleNear(eps),
                                vector<mjtNum>{0, 0, .1, 0, 0, .8}));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(SupportTest, GeomDistanceFromToFlipped) {
  mjtNum distmax = 10.0;
  char error[1024];
  mjModel* model =
      LoadModelFromString(GeomDistanceTestingModel2, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);
  mj_kinematics(model, data);

  mjtNum fromto01[6];
  mjtNum fromto10[6];

  for (int flag : {0, (int)mjDSBL_NATIVECCD}) {
    model->opt.disableflags = flag;
    mj_geomDistance(model, data, 0, 1, distmax, fromto01);
    mj_geomDistance(model, data, 1, 0, distmax, fromto10);
    mjtNum fromto10flipped[6] = {fromto10[3], fromto10[4], fromto10[5],
                                 fromto10[0], fromto10[1], fromto10[2]};

    EXPECT_THAT(AsVector(fromto10flipped, 6),
                Pointwise(DoubleNear(1.0e-12), fromto01));
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}

static constexpr char kSetKeyframeTestingModel[] = R"(
<mujoco>
  <size nkey="2"/>

  <worldbody>
    <body>
      <joint name="joint" axis="0 1 0"/>
      <geom size=".1" pos="1 0 0"/>
    </body>
  </worldbody>

  <actuator>
    <intvelocity joint="joint" actrange="-1 1" kp="100" dampratio="1"/>
  </actuator>
</mujoco>
)";

TEST_F(SupportTest, SetKeyframe) {
  char error[1024];
  mjModel* model =
      LoadModelFromString(kSetKeyframeTestingModel, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  data->ctrl[0] = 1;
  while (data->time < 1) {
    mj_step(model, data);
  }

  mj_setKeyframe(model, data, 1);
  EXPECT_EQ(data->time, model->key_time[1]);
  EXPECT_EQ(data->ctrl[0], model->key_ctrl[model->nu * 1]);
  EXPECT_EQ(data->qpos[0], model->key_qpos[model->nq * 1]);
  EXPECT_EQ(data->qvel[0], model->key_qvel[model->nv * 1]);
  EXPECT_EQ(data->act[0], model->key_act[model->na * 1]);

  mj_step(model, data);
  mj_setKeyframe(model, data, 0);
  EXPECT_EQ(data->time, model->key_time[0]);
  EXPECT_EQ(data->ctrl[0], model->key_ctrl[model->nu * 0]);
  EXPECT_EQ(data->qpos[0], model->key_qpos[model->nq * 0]);
  EXPECT_EQ(data->qvel[0], model->key_qvel[model->nv * 0]);
  EXPECT_EQ(data->act[0], model->key_act[model->na * 0]);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(SupportTest, ContactSensorDim) {
  int dataSpec = 1 << mjCONDATA_FOUND   |
                 1 << mjCONDATA_FORCE   |
                 1 << mjCONDATA_DIST    |
                 1 << mjCONDATA_POS     |
                 1 << mjCONDATA_TANGENT;

  EXPECT_EQ(mju_condataSize(dataSpec), 1+3+1+3+3);
}

}  // namespace
}  // namespace mujoco
