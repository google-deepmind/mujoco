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

// Tests for engine/engine_core_constraint.c.

#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_constraint.h"
#include "src/engine/engine_support.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::NotNull;
using ::testing::Pointwise;
using CoreConstraintTest = MujocoTest;

// compute rotation residual following formula in mj_instantiateEquality
void RotationResidual(const mjModel *model, mjData *data,
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
}

// validate rotational Jacobian used in welds
TEST_F(CoreConstraintTest, WeldRotJacobian) {
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, testing::NotNull());
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
                NULL, NULL, NULL, jac0, jac1, jacdif);

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
              Pointwise(DoubleNear(eps), AsVector(jacdif, 3*nv)));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// test formulas for penetration at rest
TEST_F(CoreConstraintTest, RestPenetration) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="1 1 1"/>
      <body pos="0 0 .2">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, testing::NotNull());
  mjtNum gravity = -model->opt.gravity[2];
  mjtNum damping_ratio = 0.8;
  mjData* data = mj_makeData(model);

  for (const mjtNum reference : {-100.0, -10.0, 0.1, 0.01}) {
    for (const mjtNum impedance : {0.3, 0.9, 0.99}) {
      // set solimp
      for (int i=0; i < model->ngeom; i++) {
        model->geom_solimp[i*mjNIMP + 0] = impedance;
        model->geom_solimp[i*mjNIMP + 1] = impedance;
      }

      // set solref
      for (int i=0; i < model->ngeom; i++) {
        model->geom_solref[i*mjNREF + 0] = reference;
        model->geom_solref[i*mjNREF + 1] = reference < 0 ? -10 : damping_ratio;
      }

      // simulate for 50 seconds
      mj_resetData(model, data);
      while (data->time < 50) {
        mj_step(model, data);
      }

      mjtNum depth = -data->contact[0].dist;
      mjtNum expected_depth;
      if (reference < 0) {
        expected_depth = gravity * (1 - impedance) / -reference;
      } else {
        mjtNum tc_dr = reference * damping_ratio;
        expected_depth = gravity * (1 - impedance) * tc_dr * tc_dr;
      }

      EXPECT_THAT(depth, DoubleNear(expected_depth, 1e-10));
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kDoflessContactPath =
    "engine/testdata/core_constraint/dofless_contact.xml";
static const char* const kDoflessTendonFrictionalPath =
    "engine/testdata/core_constraint/dofless_tendon_frictional.xml";
static const char* const kDoflessTendonLimitedPath =
    "engine/testdata/core_constraint/dofless_tendon_limited.xml";
static const char* const kDoflessTendonLimitedMarginPath =
    "engine/testdata/core_constraint/dofless_tendon_limitedmargin.xml";
static const char* const kDoflessWeldPath =
    "engine/testdata/core_constraint/dofless_weld.xml";
static const char* const kJointLimitedBilateralMarginPath =
    "engine/testdata/core_constraint/joint_limited_bilateral_margin.xml";
static const char* const kTendonLimitedBilateralMarginPath =
    "engine/testdata/core_constraint/tendon_limited_bilateral_margin.xml";

TEST_F(CoreConstraintTest, JacobianPreAllocate) {
  for (const char* local_path :
       {kDoflessContactPath, kDoflessTendonFrictionalPath,
        kDoflessTendonLimitedPath, kDoflessTendonLimitedMarginPath,
        kDoflessWeldPath, kJointLimitedBilateralMarginPath,
        kTendonLimitedBilateralMarginPath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);

    // iterate through dense and sparse
    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
      model->opt.jacobian = sparsity;
      mjData* data = mj_makeData(model);

      mj_step(model, data);

      mj_deleteData(data);
      mj_deleteModel(model);
    }
  }
}

TEST_F(CoreConstraintTest, EqualityBodySite) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/equality_site_body_compare.xml");

  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // simulate, get diag(A)
  while (data->time < 0.1) {
    mj_step(model, data);
  }
  int nefc_site = data->nefc;
  std::vector<mjtNum> dA = AsVector(data->efc_diagApprox, nefc_site);

  // reset
  mj_resetData(model, data);

  // turn site-defined equalities off, equivalent body-defined equalities on
  for (int e=0; e < 4; e++) data->eq_active[e] = 1 - data->eq_active[e];

  // simulate again, get diag(A)
  while (data->time < 0.1) {
    mj_step(model, data);
  }

  // compare
  EXPECT_EQ(nefc_site, data->nefc);
  EXPECT_THAT(AsVector(data->efc_diagApprox, data->nefc),
              Pointwise(DoubleNear(1e-12), dA));

  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kIlslandEfcPath =
    "engine/testdata/island/island_efc.xml";

// validate mj_constraintUpdate_impl
TEST_F(CoreConstraintTest, ConstraintUpdateImpl) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  char err[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, err, 1024);
  ASSERT_THAT(model, NotNull()) << err;
  mjData* d1 = mj_makeData(model);
  mjData* d2 = mj_makeData(model);

  // iterate over sparsity and cone
  for (mjtJacobian sparsity : {mjJAC_SPARSE, mjJAC_DENSE}) {
    for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      model->opt.jacobian = sparsity;
      model->opt.cone = cone;

      // simulate for 0.2 seconds
      mj_resetData(model, d1);
      mj_resetData(model, d2);
      while (d1->time < 0.2) {
        mj_step(model, d1);
        mj_step(model, d2);
      }
      mj_forward(model, d1);
      mj_forward(model, d2);

      // get sizes
      int nefc = d1->nefc;
      int nv = model->nv;
      int nisland = d1->nisland;
      EXPECT_GT(nisland, 0);

      // get jar = J*a - aref
      mjtNum* jar = (mjtNum*)mju_malloc(sizeof(mjtNum) * nefc);
      mj_mulJacVec(model, d1, jar, d1->qacc);
      mju_subFrom(jar, d1->efc_aref, nefc);

      // constraint update for data1 given jar
      mjtNum cost1;
      mj_constraintUpdate(model, d1, jar, &cost1, /*flg_coneHessian=*/1);

      // iterate over islands, check match
      mjtNum cost2 = 0;
      for (int island=0; island < nisland; island++) {
        // clear outputs from data2
        for (int i=0; i < nefc; i++) d2->efc_state[i] = -1;
        mju_zero(d2->efc_force, nefc);
        mju_zero(d2->qfrc_constraint, nv);
        for (int i=0; i < d2->ncon; i++) mju_zero(d2->contact[i].H, 36);

        // sizes and indices, in this island
        int efcnum = d2->island_nefc[island];

        // gather values into jari
        mjtNum* jari = (mjtNum*)mju_malloc(sizeof(mjtNum) * efcnum);
        int* map2efc = d2->map_iefc2efc + d2->island_iefcadr[island];
        mju_gather(jari, jar, map2efc, efcnum);

        // update constraints for this island
        mjtNum cost2i;
        int ne  = d2->island_ne[island];
        int nf  = d2->island_nf[island];
        int adr = d2->island_iefcadr[island];
        int* state = d2->iefc_state + adr;
        mjtNum *force = d2->iefc_force + adr;
        mj_constraintUpdate_impl(ne, nf, efcnum,
                                 d2->iefc_D + adr,
                                 d2->iefc_R + adr,
                                 d2->iefc_frictionloss + adr,
                                 jari,
                                 d2->iefc_type + adr,
                                 d2->iefc_id + adr,
                                 d2->contact,
                                 state,
                                 force,
                                 &cost2i,
                                 /*flg_coneHessian=*/1);

        // compare nefc vectors
        for (int c=0; c < efcnum; c++) {
          int i = map2efc[c];
          EXPECT_EQ(d2->efc_island[i], island);
          EXPECT_EQ(state[c], d1->efc_state[i]);
          EXPECT_THAT(force[c], DoubleNear(d1->efc_force[i], 1e-12));
        }

        // compare cone Hessians
        if (cone == mjCONE_ELLIPTIC) {
          for (int c=0; c < d2->ncon; c++) {
            int efcadr = d2->contact[c].efc_address;
            if (d2->efc_island[efcadr] == island &&
                d2->efc_state[efcadr] == mjCNSTRSTATE_CONE) {
              for (int j=0; j < 36; j++) {
                EXPECT_THAT(d2->contact[c].H[j],
                            DoubleNear(d1->contact[c].H[j], 1e-12));
              }
            }
          }
        }

        // add island cost to total cost
        cost2 += cost2i;

        mju_free(jari);
      }

      // expect monolithic total cost
      EXPECT_THAT(cost1, DoubleNear(cost2, 1e-12));

      mju_free(jar);
    }
  }

  mj_deleteData(d2);
  mj_deleteData(d1);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
