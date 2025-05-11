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
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
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

TEST_F(CoreConstraintTest, MulJacVecIsland) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // allocate vec_nv, fill with arbitrary values
  mjtNum* vec_nv = (mjtNum*) mju_malloc(sizeof(mjtNum)*model->nv);
  for (int i=0; i < model->nv; i++) {
    vec_nv[i] = 0.2 + 0.3*i;
  }

  // iterate through dense and sparse
  for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
    model->opt.jacobian = sparsity;

    // simulate for 0.2 seconds
    mj_resetData(model, data);
    while (data->time < 0.2) {
      mj_step(model, data);
    }
    mj_forward(model, data);

    // multiply by Jacobian: vec_nefc = J * vec_nv
    mjtNum* vec_nefc = (mjtNum*) mju_malloc(sizeof(mjtNum)*data->nefc);
    mj_mulJacVec(model, data, vec_nefc, vec_nv);
    mjtNum* vec_nefc_tmp = (mjtNum*) mju_malloc(sizeof(mjtNum)*data->nefc);

    // iterate over islands
    for (int i=0; i < data->nisland; i++) {
      // allocate dof and efc vectors for island
      int dofnum = data->island_dofnum[i];
      mjtNum* vec_nvi = (mjtNum*)mju_malloc(sizeof(mjtNum) * dofnum);
      int efcnum = data->island_efcnum[i];
      mjtNum* vec_nefci = (mjtNum*)mju_malloc(sizeof(mjtNum) * efcnum);

      // get indices
      int* dofind = data->island_dofind + data->island_dofadr[i];
      int* efcind = data->island_efcind + data->island_efcadr[i];

      // copy values into vec_nvi
      for (int j=0; j < dofnum; j++) {
        vec_nvi[j] = vec_nv[dofind[j]];
      }

      // ===== both compressed
      int flg_resunc = 0;
      int flg_vecunc = 0;
      mju_zero(vec_nefci, efcnum);  // clear output
      mj_mulJacVec_island(model, data, vec_nefci, vec_nvi,
                          i, flg_resunc, flg_vecunc);

      // expect corresponding values to match
      for (int j=0; j < efcnum; j++) {
        EXPECT_THAT(vec_nefci[j], DoubleNear(vec_nefc[efcind[j]], 1e-12));
      }

      // ===== input uncompressed: read from vec_nv
      flg_resunc = 0;
      flg_vecunc = 1;
      mju_zero(vec_nefci, efcnum);  // clear output
      mj_mulJacVec_island(model, data, vec_nefci, vec_nv,
                          i, flg_resunc, flg_vecunc);

      // expect corresponding values to match
      for (int j=0; j < efcnum; j++) {
        EXPECT_THAT(vec_nefci[j], DoubleNear(vec_nefc[efcind[j]], 1e-12));
      }

      // ===== output uncompressed: write to vec_nefc_tmp
      flg_resunc = 1;
      flg_vecunc = 0;
      mju_zero(vec_nefc_tmp, data->nefc);  // clear output
      mj_mulJacVec_island(model, data, vec_nefc_tmp, vec_nvi,
                          i, flg_resunc, flg_vecunc);

      // expect corresponding values to match
      for (int j=0; j < efcnum; j++) {
        EXPECT_THAT(vec_nefc_tmp[efcind[j]],
                    DoubleNear(vec_nefc[efcind[j]], 1e-12));
      }

      mju_free(vec_nvi);
      mju_free(vec_nefci);
    }

    mju_free(vec_nefc_tmp);
    mju_free(vec_nefc);
  }

  mju_free(vec_nv);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(CoreConstraintTest, MulJacTVecIsland) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // allocate vec_nv
  mjtNum* vec_nv = (mjtNum*) mju_malloc(sizeof(mjtNum)*model->nv);
  mjtNum* vec_nv_tmp = (mjtNum*) mju_malloc(sizeof(mjtNum)*model->nv);

  // iterate through dense and sparse
  for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
    model->opt.jacobian = sparsity;

    // simulate for 0.3 seconds
    mj_resetData(model, data);
    while (data->time < 0.3) {
      mj_step(model, data);
    }
    mj_forward(model, data);

    // allocate vec_nefc, fill with arbitrary values
    mjtNum* vec_nefc = (mjtNum*) mju_malloc(sizeof(mjtNum)*data->nefc);
    for (int i=0; i < data->nefc; i++) {
      vec_nefc[i] = 0.2 + 0.3*i;
    }

    // multiply by Jacobian: vec_nv = J^T * vec_nefc
    mj_mulJacTVec(model, data, vec_nv, vec_nefc);

    // iterate over islands
    for (int i=0; i < data->nisland; i++) {
      // allocate dof and efc vectors for island
      int dofnum = data->island_dofnum[i];
      mjtNum* vec_nvi = (mjtNum*)mju_malloc(sizeof(mjtNum) * dofnum);
      int efcnum = data->island_efcnum[i];
      mjtNum* vec_nefci = (mjtNum*)mju_malloc(sizeof(mjtNum) * efcnum);

      // get indices
      int* efcind = data->island_efcind + data->island_efcadr[i];
      int* dofind = data->island_dofind + data->island_dofadr[i];

      // copy values into vec_nefci
      for (int j=0; j < efcnum; j++) {
        vec_nefci[j] = vec_nefc[efcind[j]];
      }

      // ==== both compressed
      int flg_resunc = 0;
      int flg_vecunc = 0;
      mju_zero(vec_nvi, dofnum);  // clear output
      mj_mulJacTVec_island(model, data, vec_nvi, vec_nefci,
                           i, flg_resunc, flg_vecunc);

      // expect corresponding values to match
      for (int j=0; j < dofnum; j++) {
        EXPECT_THAT(vec_nvi[j], DoubleNear(vec_nv[dofind[j]], 1e-12));
      }

      // ===== input uncompressed: read from vec_nefc
      flg_resunc = 0;
      flg_vecunc = 1;
      mju_zero(vec_nvi, dofnum);  // clear output
      mj_mulJacTVec_island(model, data, vec_nvi, vec_nefc,
                           i, flg_resunc, flg_vecunc);

      // expect corresponding values to match
      for (int j=0; j < dofnum; j++) {
        EXPECT_THAT(vec_nvi[j], DoubleNear(vec_nv[dofind[j]], 1e-12));
      }

      // ===== output uncompressed: write to vec_nv_tmp
      flg_resunc = 1;
      flg_vecunc = 0;
      mju_zero(vec_nv_tmp, model->nv);  // clear output
      mj_mulJacTVec_island(model, data, vec_nv_tmp, vec_nefci,
                           i, flg_resunc, flg_vecunc);

      // expect corresponding values to match
      for (int j=0; j < dofnum; j++) {
        EXPECT_THAT(vec_nv_tmp[dofind[j]],
                    DoubleNear(vec_nv[dofind[j]], 1e-12));
      }

      mju_free(vec_nvi);
      mju_free(vec_nefci);
    }
    mju_free(vec_nefc);
  }

  mju_free(vec_nv_tmp);
  mju_free(vec_nv);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// compare mj_constraintUpdate and mj_constraintUpdate_island
TEST_F(CoreConstraintTest, ConstraintUpdateIsland) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data1 = mj_makeData(model);
  mjData* data2 = mj_makeData(model);

  // iterate over sparsity and cone
  for (mjtJacobian sparsity : {mjJAC_SPARSE, mjJAC_DENSE}) {
    for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      model->opt.jacobian = sparsity;
      model->opt.cone = cone;

      // simulate for 0.2 seconds
      mj_resetData(model, data1);
      mj_resetData(model, data2);
      while (data1->time < 0.2) {
        mj_step(model, data1);
        mj_step(model, data2);
      }
      mj_forward(model, data1);
      mj_forward(model, data2);

      // get sizes
      int nefc = data1->nefc;
      int nv = model->nv;
      int nisland = data1->nisland;
      EXPECT_GT(nisland, 0);

      // get jar = J*a - aref
      mjtNum* jar = (mjtNum*)mju_malloc(sizeof(mjtNum) * nefc);
      mj_mulJacVec(model, data1, jar, data1->qacc);
      mju_subFrom(jar, data1->efc_aref, nefc);

      // constraint update for data1 given jar
      mjtNum cost1;
      mj_constraintUpdate(model, data1, jar, &cost1, /*flg_coneHessian=*/1);

      // iterate over islands, check match
      mjtNum cost2 = 0;
      for (int island=0; island < nisland; island++) {
        // clear outputs from data2
        for (int i=0; i < nefc; i++) data2->efc_state[i] = -1;
        mju_zero(data2->efc_force, nefc);
        mju_zero(data2->qfrc_constraint, nv);
        for (int i=0; i < data2->ncon; i++) mju_zero(data2->contact[i].H, 36);

        // sizes and indices, in this island
        int dofnum = data2->island_dofnum[island];
        int efcnum = data2->island_efcnum[island];
        int* dofind = data2->island_dofind + data2->island_dofadr[island];
        int* efcind = data2->island_efcind + data2->island_efcadr[island];

        // get jar restricted to island
        mjtNum* jari = (mjtNum*)mju_malloc(sizeof(mjtNum) * efcnum);
        for (int c=0; c < efcnum; c++) {
          jari[c] = jar[efcind[c]];
        }

        // update constraints for this island
        mjtNum cost2i;
        mj_constraintUpdate_island(model, data2, jari, &cost2i,
                                  /*flg_coneHessian=*/1, island);

        // compare nefc vectors
        for (int c=0; c < efcnum; c++) {
          int i = efcind[c];
          EXPECT_EQ(data2->efc_island[i], island);
          EXPECT_EQ(data2->efc_state[i], data1->efc_state[i]);
          EXPECT_THAT(data2->efc_force[i],
                      DoubleNear(data1->efc_force[i], 1e-12));
        }

        // compare qfrc_constraint
        for (int c=0; c < dofnum; c++) {
          int i = dofind[c];
          EXPECT_THAT(data2->qfrc_constraint[i],
                      DoubleNear(data1->qfrc_constraint[i], 1e-12));
        }

        // compare cone Hessians
        if (cone == mjCONE_ELLIPTIC) {
          for (int c=0; c < data2->ncon; c++) {
            int efcadr = data2->contact[c].efc_address;
            if (data2->efc_island[efcadr] == island &&
                data2->efc_state[efcadr] == mjCNSTRSTATE_CONE) {
              for (int j=0; j < 36; j++) {
                EXPECT_THAT(data2->contact[c].H[j],
                            DoubleNear(data1->contact[c].H[j], 1e-12));
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

  mj_deleteData(data2);
  mj_deleteData(data1);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
