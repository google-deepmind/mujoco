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

// Tests for engine/engine_core_util.c.

#include "src/engine/engine_core_util.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using ::testing::Pointwise;

using FlexGatherStateTest = MujocoTest;

TEST_F(FlexGatherStateTest, mju_flexGatherState_Grid) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="flex0" type="grid" count="3 3 3" spacing=".1 .1 .1"
                dim="3" mass="1" radius="0.01" dof="trilinear">
        <elasticity young="5e4" poisson="0.2"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  ASSERT_EQ(model->nflex, 1);
  int f = 0;
  int nodenum = model->flex_nodenum[f];
  int nstart = model->flex_nodeadr[f];

  // Simulate a rotated state (90 degrees around Z axis)
  ASSERT_TRUE(model->flex_centered[f]);
  for (int i = 0; i < nodenum; i++) {
    int b = model->flex_nodebodyid[nstart + i];
    mjtNum x = data->xpos[3*b + 0];
    mjtNum y = data->xpos[3*b + 1];
    mjtNum z = data->xpos[3*b + 2];

    // Rotate 90 degrees around Z: (x, y, z) -> (-y, x, z)
    data->xpos[3*b + 0] = -y;
    data->xpos[3*b + 1] = x;
    data->xpos[3*b + 2] = z;
  }

  std::vector<mjtNum> xpos(3 * nodenum);
  mju_flexGatherState(model, data, f, xpos.data(), NULL);

  // Verify that gathered xpos matches the rotated data->xpos
  for (int i = 0; i < nodenum; i++) {
    int b = model->flex_nodebodyid[nstart + i];
    EXPECT_NEAR(xpos[3*i + 0], data->xpos[3*b + 0], 1e-5);
    EXPECT_NEAR(xpos[3*i + 1], data->xpos[3*b + 1], 1e-5);
    EXPECT_NEAR(xpos[3*i + 2], data->xpos[3*b + 2], 1e-5);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}


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
  for (int i = 0; i < 3; i++) {
    EXPECT_THAT(angmom_ref[i], MjNear(angmom_test[i], 1e-8, 1e-4));
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
  static constexpr mjtNum eps = MjTol(1e-6, 1e-3);
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
  for (int i = 0; i < 3*nv; i++) {
    EXPECT_THAT(angmom_mat_fd[i], MjNear(angmom_mat[i], 1e-8, 2e-4));
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
      EXPECT_THAT(jac_subtree[nv*j+i], MjNear(findiff, eps, 1e-2));
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
      EXPECT_THAT(jac_subtree[nv*r+c], MjNear(expected, max_abs_err, 1e-4));
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
    std::vector<mjtNum> jacp(3*nv);
    std::vector<mjtNum> jacr(3*nv);
    mj_jac(model, data, jacp.data(), jacr.data(), point, bodyid);
    std::vector<mjtNum> jacp_dot(3*nv);
    std::vector<mjtNum> jacr_dot(3*nv);
    mj_jacDot(model, data, jacp_dot.data(), jacr_dot.data(), point, bodyid);

    // jac_h: jacobian after integrating qpos with a timestep of h
    constexpr mjtNum h = MjTol(1e-7, 5e-4);
    mj_integratePos(model, data->qpos, data->qvel, h);
    mj_kinematics(model, data);
    mj_comPos(model, data);
    std::vector<mjtNum> jacp_h(3*nv);
    std::vector<mjtNum> jacr_h(3*nv);
    mju_copy3(point, data->site_xpos+3*siteid);  // get updated site position
    mj_jac(model, data, jacp_h.data(), jacr_h.data(), point, bodyid);

    // jac_dot_h finite-difference approximation
    std::vector<mjtNum> jacp_dot_h(3*nv);
    mju_sub(jacp_dot_h.data(), jacp_h.data(), jacp.data(), 3*nv);
    mju_scl(jacp_dot_h.data(), jacp_dot_h.data(), 1/h, 3*nv);
    std::vector<mjtNum> jacr_dot_h(3*nv);
    mju_sub(jacr_dot_h.data(), jacr_h.data(), jacr.data(), 3*nv);
    mju_scl(jacr_dot_h.data(), jacr_dot_h.data(), 1/h, 3*nv);

    // compare finite-differenced and analytic
    mjtNum tol = 1e-5;
    EXPECT_THAT(jacp_dot, Pointwise(MjNear(tol, 5e-2), jacp_dot_h));
    EXPECT_THAT(jacr_dot, Pointwise(MjNear(tol, 5e-2), jacr_dot_h));

    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

// compare mj_jacDotSparse with dense mj_jacDot
TEST_F(JacobianTest, JacDotSparse) {
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

    // get bodyid and site position
    int bodyid = mj_name2id(model, mjOBJ_BODY, "query");
    EXPECT_GT(bodyid, 0);
    int siteid = mj_name2id(model, mjOBJ_SITE, "query");
    EXPECT_GT(siteid, -1);
    mjtNum point[3];
    mju_copy3(point, data->site_xpos+3*siteid);

    // dense jacDot
    std::vector<mjtNum> jacp_dense(3*nv);
    std::vector<mjtNum> jacr_dense(3*nv);
    mj_jacDot(model, data, jacp_dense.data(), jacr_dense.data(), point, bodyid);

    // compute body chain using public mjModel fields
    std::vector<int> chain(nv);
    int NV = 0;
    int weldbody = model->body_weldid[bodyid];
    if (weldbody) {
      int da = model->body_dofadr[weldbody] + model->body_dofnum[weldbody] - 1;
      while (da >= 0) {
        chain[NV++] = da;
        da = model->dof_parentid[da];
      }
      std::reverse(chain.begin(), chain.begin() + NV);
    }
    EXPECT_GT(NV, 0);

    // sparse jacDot
    std::vector<mjtNum> jacp_sparse(3*NV);
    std::vector<mjtNum> jacr_sparse(3*NV);
    mj_jacDotSparse(model, data, jacp_sparse.data(), jacr_sparse.data(),
                    point, bodyid, NV, chain.data());

    // expand sparse to dense and compare
    std::vector<mjtNum> jacp_expanded(3*nv, 0);
    std::vector<mjtNum> jacr_expanded(3*nv, 0);
    for (int ci = 0; ci < NV; ci++) {
      int di = chain[ci];
      for (int r = 0; r < 3; r++) {
        jacp_expanded[di+r*nv] = jacp_sparse[ci+r*NV];
        jacr_expanded[di+r*nv] = jacr_sparse[ci+r*NV];
      }
    }

    // expect bitwise equality
    EXPECT_EQ(jacp_expanded, jacp_dense);
    EXPECT_EQ(jacr_expanded, jacr_dense);

    mj_deleteData(data);
    mj_deleteModel(model);
  }
}


// validate rotational Jacobian used in welds
TEST_F(JacobianTest, WeldRotJacobian) {
#ifdef mjUSESINGLE
  GTEST_SKIP() << "FD Jacobian with eps=1e-6 below float32 precision";
#endif
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="dense"/>
    <worldbody>
      <body>
        <joint type="ball"/>
        <geom size=".1"/>
      </body>
      <body pos=".5 0 0">
        <joint axis="1 0 0" pos="0 0 .01"/>
        <joint axis="0 1 0" pos=".02 0 0"/>
        <joint axis="0 0 1" pos="0 .03 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, testing::NotNull()) << error;
  ASSERT_EQ(model->nq, 7);
  ASSERT_EQ(model->nv, 6);
  static const int nv = 6;  // for increased readability
  mjData* data = mj_makeData(model);

  // arbitrary initial values for the ball and hinge joints
  mjtNum qpos0[7] = {.5, .5, .5, .5, .7, .8, .9};

  // compute required quantities using mj_step1
  mj_step1(model, data);

  // get orientation error
  mjtNum res[3];
  // compute rotation residual following formula in mj_instantiateEquality
  auto RotationResidual = [](const mjModel *model, mjData *data,
                             const mjtNum qpos[7], const mjtNum dqpos[6],
                             mjtNum res[3]) {
    // copy configuration, compute required quantities with mj_step1
    mju_copy(data->qpos, qpos, 7);

    // perturb configuration if given
    if (dqpos) {
      mj_integratePos(model, data->qpos, dqpos, 1);
    }

    // update relevant quantities
    mj_step1(model, data);

    // compute orientation residual
    mjtNum quat1[4], quat2[4], quat3[4];
    mju_copy4(quat1, data->xquat+4*1);
    mju_negQuat(quat2, data->xquat+4*2);
    mju_mulQuat(quat3, quat2, quat1);
    mju_copy3(res, quat3+1);
  };

  RotationResidual(model, data, qpos0, NULL, res);

  // compute Jacobian with finite-differencing
  mjtNum jacFD[3*nv];
  mjtNum dqpos[nv] = {0};
  mjtNum dres[3];
  const mjtNum eps = 1e-6;
  for (int i=0; i < nv; i++) {
    // nudge i-th dof
    dqpos[i] = eps;

    // get nudged residual
    RotationResidual(model, data, qpos0, dqpos, dres);

    // remove nudge
    dqpos[i] = 0.0;

    // compute Jacobian column
    for (int j=0; j < 3; j++) {
      jacFD[nv*j + i] = (dres[j] - res[j]) / eps;
    }
  }

  // reset mjData to qpos0
  mju_copy(data->qpos, qpos0, 7);
  mj_step1(model, data);

  // intermediate quaternions quat1 and quat2
  mjtNum quat1[4], negQuat2[4];
  mju_copy4(quat1, data->xquat+4*1);
  mju_negQuat(negQuat2, data->xquat+4*2);

  // get analytical Jacobian following formula in mj_instantiateEquality
  mjtNum jacdif[3*nv], jac0[3*nv], jac1[3*nv];
  mjtNum point[3] = {0};

  // rotational Jacobian difference
  mj_jacDifPair(model, data, NULL, 2, 1, point, point,
                NULL, NULL, NULL, jac0, jac1, jacdif, mj_isSparse(model),
                /*flg_skipcommon=*/0);

  // formula: 0.5 * neg(quat2) * (jac1-jac2) * quat1
  mjtNum axis[3], quat3[4], quat4[4];
  for (int j=0; j < nv; j++) {
    // axis = [jac1-jac2]_col(j)
    axis[0] = jacdif[0*nv+j];
    axis[1] = jacdif[1*nv+j];
    axis[2] = jacdif[2*nv+j];

    // apply formula
    mju_mulQuatAxis(quat3, negQuat2, axis);
    mju_mulQuat(quat4, quat3, quat1);

    // correct Jacobian
    jacdif[0*nv+j] = 0.5*quat4[1];
    jacdif[1*nv+j] = 0.5*quat4[2];
    jacdif[2*nv+j] = 0.5*quat4[3];
  }

  // test that analytical and finite-differenced Jacobians match
  EXPECT_THAT(AsVector(jacFD, 3*nv),
              Pointwise(MjNear(eps, 1e-3), AsVector(jacdif, 3*nv)));

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco


