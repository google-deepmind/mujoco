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

#include <array>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_constraint.h"
#include "src/engine/engine_core_util.h"
#include "src/engine/engine_support.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using ::testing::Pointwise;

using CoreConstraintTest = MujocoTest;

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
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, testing::NotNull()) << error;
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
        mjtNum time = data->time;
        mj_step(model, data);
        ASSERT_GT(data->time, time) << "Divergence detected";
      }

      mjtNum depth = -data->contact[0].dist;
      mjtNum expected_depth;
      if (reference < 0) {
        expected_depth = gravity * (1 - impedance) / -reference;
      } else {
        mjtNum tc_dr = reference * damping_ratio;
        expected_depth = gravity * (1 - impedance) * tc_dr * tc_dr;
      }

      EXPECT_THAT(depth, MjNear(expected_depth, 1e-10, 1e-3));
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
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
  }
  int nefc_site = data->nefc;
  std::vector<mjtNum> dA = AsVector(data->efc_diagApprox, nefc_site);

  // reset
  mj_resetData(model, data);

  // turn site-defined equalities off, equivalent body-defined equalities on
  for (int e=0; e < 4; e++) data->eq_active[e] = 1 - data->eq_active[e];

  // simulate again, get diag(A)
  while (data->time < 0.1) {
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
  }

  // compare
  EXPECT_EQ(nefc_site, data->nefc);
  EXPECT_THAT(AsVector(data->efc_diagApprox, data->nefc),
              Pointwise(MjNear(1e-12, 1e-4), dA));

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
        mjtNum time1 = d1->time;
        mjtNum time2 = d2->time;
        mj_step(model, d1);
        ASSERT_GT(d1->time, time1) << "Divergence detected";
        mj_step(model, d2);
        ASSERT_GT(d2->time, time2) << "Divergence detected";
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
          EXPECT_THAT(force[c], MjNear(d1->efc_force[i], 1e-12, 1e-4));
        }

        // compare cone Hessians
        if (cone == mjCONE_ELLIPTIC) {
          for (int c=0; c < d2->ncon; c++) {
            int efcadr = d2->contact[c].efc_address;
            if (d2->efc_island[efcadr] == island &&
                d2->efc_state[efcadr] == mjCNSTRSTATE_CONE) {
              for (int j=0; j < 36; j++) {
                EXPECT_THAT(d2->contact[c].H[j],
                            MjNear(d1->contact[c].H[j], 1e-12, 1e-4));
              }
            }
          }
        }

        // add island cost to total cost
        cost2 += cost2i;

        mju_free(jari);
      }

      // expect monolithic total cost
      EXPECT_THAT(cost1, MjNear(cost2, 1e-12, 1e-4));

      mju_free(jar);
    }
  }

  mj_deleteData(d2);
  mj_deleteData(d1);
  mj_deleteModel(model);
}

// check mjEQ_FLEXVERT
TEST_F(CoreConstraintTest, FlexvertEquality) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="dense"/>
    <worldbody>
      <flexcomp name="flex" type="grid" count="3 3 1" spacing=".05 .15 .25" dim="2">
        <edge equality="vert"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, testing::NotNull()) << error;
  mjData* data = mj_makeData(model);
  ASSERT_EQ(model->neq, 1);
  ASSERT_EQ(model->eq_type[0], mjEQ_FLEXVERT);
  ASSERT_EQ(model->nflex, 1);
  ASSERT_EQ(model->flex_vertnum[0], 9);
  ASSERT_EQ(model->flex_edgenum[0], 16);

  // step1 to populate flexvert_length
  mj_step1(model, data);
  EXPECT_EQ(data->ne, 2*model->flex_vertnum[0]);
  EXPECT_EQ(data->nefc, 18);
  for (int i = 0; i < 18; ++i) {
    EXPECT_EQ(data->efc_type[i], mjCNSTR_EQUALITY);
    EXPECT_NEAR(data->efc_pos[i], 0, MjTol(1e-9, 1e-5));
  }

  // check that efc_J has rigid-body motions in kernel
  std::vector<mjtNum> qvel(model->nv);
  std::vector<mjtNum> Jqvel(data->nefc);

  // pure translations
  for (int i = 0; i < 3; ++i) {
    mju_zero(qvel.data(), model->nv);
    for (int j = 0; j < model->flex_vertnum[0]; ++j) {
      qvel[3*j+i] = 1.0;
    }
    mj_mulJacVec(model, data, Jqvel.data(), qvel.data());
    for (int j = 0; j < data->nefc; ++j) {
      EXPECT_NEAR(Jqvel[j], 0, MjTol(1e-9, 1e-5));
    }
  }

  // pure rotations
  for (int i = 0; i < 3; ++i) {
    mju_zero(qvel.data(), model->nv);
    for (int j = 0; j < model->flex_vertnum[0]; ++j) {
      mjtNum* p = data->flexvert_xpos + 3 * j;
      mjtNum axisvel[3] = {0};
      axisvel[i] = 1.0;
      mjtNum linvel[3];
      mju_cross(linvel, axisvel, p);
      qvel[3 * j + 0] = linvel[0];
      qvel[3 * j + 1] = linvel[1];
      qvel[3 * j + 2] = linvel[2];
    }
    mj_mulJacVec(model, data, Jqvel.data(), qvel.data());
    for (int j = 0; j < data->nefc; ++j) {
      EXPECT_NEAR(Jqvel[j], 0, MjTol(1e-9, 1e-5));
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Test flex strain constraint with pinned nodes attached to freejoint parent
TEST_F(CoreConstraintTest, BoxShellPinnedParentWithFreejoint) {
#ifdef mjUSESINGLE
  GTEST_SKIP() << "FD Jacobian mismatch due to float32 precision";
#endif
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="implicitfast" jacobian="dense" gravity="0 0 0"/>
  <worldbody>
    <geom type="plane" size="10 10 1" pos="0 0 -.1"/>
    <body>
      <joint type="free"/>
      <geom type="box" size="0.13 0.18 0.036"/>
      <body name="parent">
        <flexcomp name="test" type="box"
                  spacing=".1 .02 .1" radius="0.001"
                  pos="0 0 .2" dof="trilinear" xyaxes="0 1 0 0 0 1" mass="1" dim="3">
          <contact selfcollide="none"/>
          <edge equality="strain"/>
          <pin id="0 2 4 6"/>
        </flexcomp>
      </body>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);

  mj_resetData(m, d);
  mj_forward(m, d);

  // Check that we have constraints
  EXPECT_GT(d->nefc, 0) << "No constraints generated";
  EXPECT_GT(d->ne, 0) << "Expected some strain constraints";

  // Check qacc and forces at rest with gravity=0
  EXPECT_NEAR(d->qacc_smooth[6], 0, 1e-6) << "qacc_smooth should be 0 at rest";
  EXPECT_NEAR(d->qacc[6], 0, 1e-6) << "qacc should be 0 at rest";

  // Check initial constraint values (efc_pos)
  bool has_bad_constraint = false;
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_type[i] == mjCNSTR_EQUALITY) {
      if (mju_abs(d->efc_pos[i]) > 1.0) {
        has_bad_constraint = true;
      }
    }
  }
  EXPECT_FALSE(has_bad_constraint)
      << "Some constraint values are too large at rest";

  // Check Jacobian values - look for NaN or huge values
  int nv = m->nv;
  bool has_bad_jacobian = false;
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_type[i] == mjCNSTR_EQUALITY) {
      for (int j = 0; j < nv; j++) {
        mjtNum val = d->efc_J[i*nv + j];
        if (mju_isBad(val) || mju_abs(val) > 1e10) {
          has_bad_jacobian = true;
        }
      }
    }
  }
  EXPECT_FALSE(has_bad_jacobian) << "Jacobian contains NaN or huge values";

  // Verify Jacobian with finite differences for first few constraints
  mjtNum eps = 1e-6;
  std::vector<mjtNum> qpos0(m->nq);
  mju_copy(qpos0.data(), d->qpos, m->nq);

  // Store original constraint values
  std::vector<mjtNum> efc_pos0(d->nefc);
  mju_copy(efc_pos0.data(), d->efc_pos, d->nefc);

  int num_constraints_to_check = mju_min(3, d->ne);
  bool has_jacobian_mismatch = false;
  for (int j = 0; j < nv && j < 6; j++) {
    mju_copy(d->qpos, qpos0.data(), m->nq);
    mjtNum dqpos[100] = {0};
    dqpos[j] = eps;
    mj_integratePos(m, d->qpos, dqpos, 1);
    mj_forward(m, d);

    for (int i = 0; i < num_constraints_to_check; i++) {
      mjtNum fd = (d->efc_pos[i] - efc_pos0[i]) / eps;
      mjtNum analytic = d->efc_J[i*nv + j];
      // Use relative tolerance with absolute floor to handle near-zero values
      mjtNum tol = mju_max(1e-8, 0.1 * (mju_abs(fd) + mju_abs(analytic)));
      if (mju_abs(fd - analytic) > tol) {
        has_jacobian_mismatch = true;
      }
    }
  }
  EXPECT_FALSE(has_jacobian_mismatch)
      << "Jacobian FD mismatch at initial config";

  // Test rotation invariance: rotate via freejoint quaternion
  mju_copy(d->qpos, qpos0.data(), m->nq);
  mjtNum angle = 0.785398;  // 45 degrees
  d->qpos[3] = mju_cos(angle/2);  // w
  d->qpos[4] = 0;
  d->qpos[5] = 0;
  d->qpos[6] = mju_sin(angle/2);  // z
  mj_forward(m, d);

  mjtNum max_strain_rotated = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_strain_rotated) {
      max_strain_rotated = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_strain_rotated, 1e-6)
      << "Strain should remain ~0 after rigid rotation";

  // Check Jacobian in rotated configuration via FD
  std::vector<mjtNum> qpos_rot(m->nq);
  mju_copy(qpos_rot.data(), d->qpos, m->nq);
  std::vector<mjtNum> efc_pos_rot(d->nefc);
  mju_copy(efc_pos_rot.data(), d->efc_pos, d->nefc);

  bool has_rotated_jacobian_mismatch = false;
  for (int j = 0; j < nv; j++) {
    mju_copy(d->qpos, qpos_rot.data(), m->nq);
    mjtNum dqpos[100] = {0};
    dqpos[j] = eps;
    mj_integratePos(m, d->qpos, dqpos, 1);
    mj_forward(m, d);

    mjtNum fd = (d->efc_pos[0] - efc_pos_rot[0]) / eps;
    mjtNum analytic = d->efc_J[0*nv + j];
    mjtNum tol = 0.1 * (mju_abs(fd) + mju_abs(analytic) + 1e-8);
    if ((mju_abs(fd) > 1e-8 || mju_abs(analytic) > 1e-8) &&
        mju_abs(fd - analytic) > tol) {
      has_rotated_jacobian_mismatch = true;
    }
  }
  EXPECT_FALSE(has_rotated_jacobian_mismatch)
      << "Jacobian FD mismatch in rotated config";

  // Reset for simulation
  mju_copy(d->qpos, qpos0.data(), m->nq);
  mj_forward(m, d);

  // Run simulation only if checks pass
  if (!has_bad_constraint && !has_bad_jacobian) {
    for (int i = 0; i < 2000; i++) {
      mj_step(m, d);

      ASSERT_FALSE(mju_isBad(d->qpos[0]))
          << "Simulation became unstable at step " << i;
      ASSERT_FALSE(mju_isBad(d->qvel[0]))
          << "Velocity became unstable at step " << i;

      for (int j = 0; j < m->nv; j++) {
        ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
            << "Velocity exploded at step " << i << ", qvel[" << j
            << "]=" << d->qvel[j];
      }
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test flex strain constraint WITHOUT pinned nodes (simpler case)
TEST_F(CoreConstraintTest, StrainConstraintNoPinning) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="implicitfast" jacobian="dense"/>
  <worldbody>
    <body name="parent">
      <joint type="free"/>
      <geom type="box" size=".01 .01 .01" mass=".1"/>
      <flexcomp name="test" type="box"
                spacing=".1 .1 .1" radius="0.001"
                pos="0 0 .5" dof="trilinear" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);

  mj_resetData(m, d);
  mj_forward(m, d);

  // Check constraints
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // Check no contacts
  EXPECT_EQ(d->ncon, 0);

  // Check that initial strain is ~0
  mjtNum max_pos = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_pos) {
      max_pos = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_pos, 1e-6) << "Initial strain should be ~0";

  // Check Jacobian for NaN
  int nv = m->nv;
  bool has_bad_jacobian = false;
  for (int i = 0; i < d->ne; i++) {
    for (int j = 0; j < nv; j++) {
      if (mju_isBad(d->efc_J[i*nv + j])) {
        has_bad_jacobian = true;
      }
    }
  }
  EXPECT_FALSE(has_bad_jacobian) << "Jacobian has NaN";

  // Test rigid rotation: rotate flex and check strain still ~0
  std::vector<mjtNum> qpos0(m->nq);
  mju_copy(qpos0.data(), d->qpos, m->nq);
  // Rotate by 45 degrees around Z axis via quaternion
  mjtNum angle = 0.785398;  // 45 degrees
  d->qpos[3] = mju_cos(angle/2);  // w
  d->qpos[4] = 0;                 // x
  d->qpos[5] = 0;                 // y
  d->qpos[6] = mju_sin(angle/2);  // z
  mj_forward(m, d);

  mjtNum max_strain_rotated = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_strain_rotated) {
      max_strain_rotated = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_strain_rotated, 1e-6)
      << "Strain should remain ~0 after rigid rotation";

  // Run simulation for a few steps to check stability
  mju_copy(d->qpos, qpos0.data(), m->nq);
  mj_forward(m, d);

  for (int i = 0; i < 100; i++) {
    mj_step(m, d);
    ASSERT_FALSE(mju_isBad(d->qpos[0])) << "Simulation unstable at step " << i;
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test flex strain constraint with quadratic interpolation
TEST_F(CoreConstraintTest, StrainConstraintQuadratic) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="implicitfast" jacobian="dense"/>
  <worldbody>
    <body name="parent">
      <joint type="free"/>
      <geom type="box" size=".01 .01 .01" mass=".1"/>
      <flexcomp name="test" type="box"
                spacing=".1 .1 .1" radius="0.001"
                pos="0 0 .5" dof="quadratic" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);

  mj_resetData(m, d);
  mj_forward(m, d);

  // Check constraints generated
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // Check that initial strain is ~0
  mjtNum max_pos = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_pos) {
      max_pos = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_pos, 1e-6) << "Initial strain should be ~0";

  // Check Jacobian for NaN
  int nv = m->nv;
  bool has_bad_jacobian = false;
  for (int i = 0; i < d->ne; i++) {
    for (int j = 0; j < nv; j++) {
      if (mju_isBad(d->efc_J[i*nv + j])) {
        has_bad_jacobian = true;
      }
    }
  }
  EXPECT_FALSE(has_bad_jacobian) << "Jacobian has NaN";

  // Run simulation for a few steps
  for (int i = 0; i < 100; i++) {
    mj_step(m, d);
    ASSERT_FALSE(mju_isBad(d->qpos[0]))
        << "Simulation unstable at step " << i;
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test quadratic passive forces (no constraints) for stability
TEST_F(CoreConstraintTest, QuadraticPassiveForceStability) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="implicitfast" solver="CG" tolerance="1e-6"/>
  <worldbody>
    <geom type="plane" size="10 10 1"/>
    <flexcomp name="test" type="grid" count="3 3 3"
              spacing=".05 .05 .05" radius="0.001"
              pos="0 0 .3" dof="quadratic" mass="1" dim="3">
      <contact selfcollide="none"/>
      <elasticity young="1e4" damping="0.01"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);

  // Run for 500 steps — should stay stable
  for (int i = 0; i < 500; i++) {
    mj_step(m, d);
    ASSERT_FALSE(mju_isBad(d->qpos[0]))
        << "Passive quadratic unstable at step " << i;
    for (int j = 0; j < m->nv; j++) {
      ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
          << "Velocity exploded at step " << i;
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Test quadratic with anisotropic cells (like what mesh bounding box creates)
TEST_F(CoreConstraintTest, QuadraticAnisotropicStrain) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="implicitfast" solver="CG" tolerance="1e-6"/>
  <size memory="50M"/>
  <worldbody>
    <geom type="plane" size="10 10 1"/>
    <body name="parent">
      <joint type="free"/>
      <geom type="box" size=".01 .01 .01" mass=".1"/>
      <flexcomp name="test" type="grid" count="3 3 3"
                spacing=".1 .05 .08" radius="0.001"
                pos="0 0 .5" dof="quadratic" mass="1" dim="3">
        <contact selfcollide="none" internal="false"/>
        <edge equality="strain" damping="0.01"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);

  mj_forward(m, d);
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // Run for 200 steps with gravity + contact
  for (int i = 0; i < 200; i++) {
    mj_step(m, d);
    ASSERT_FALSE(mju_isBad(d->qpos[0]))
        << "Anisotropic quadratic unstable at step " << i;
    for (int j = 0; j < m->nv; j++) {
      ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
          << "Velocity exploded at step " << i
          << ", qvel[" << j << "]=" << d->qvel[j];
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(CoreConstraintTest, ContactSharedDofJacobian) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="sparse"/>
    <worldbody>
      <body pos="0 0 0.5">
        <joint type="slide" axis="0 0 1"/>
        <geom size="0.01"/>
        <body pos="0 0.04 0">
          <joint type="slide" axis="0 1 0"/>
          <geom type="sphere" size="0.05" condim="1"/>
        </body>
        <body pos="0 -0.04 0">
          <joint type="slide" axis="0 1 0"/>
          <geom type="sphere" size="0.05" condim="1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  ASSERT_EQ(model->nv, 3);
  ASSERT_TRUE(mj_isSparse(model));
  mjData* data = mj_makeData(model);

  mj_forward(model, data);

  ASSERT_EQ(data->ncon, 1);
  ASSERT_GE(data->nefc, 1);

  EXPECT_EQ(data->efc_J_rownnz[0], 2);

  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kJdotvConnect2dPath =
    "engine/testdata/core_constraint/jdotv_connect_2d.xml";
static const char* const kJdotvConnect3dPath =
    "engine/testdata/core_constraint/jdotv_connect_3d.xml";
static const char* const kJdotvWeld3dPath =
    "engine/testdata/core_constraint/jdotv_weld_3d.xml";

// validate mj_Jdotv against finite-differenced constraint Jacobian
TEST_F(CoreConstraintTest, JdotvFiniteDifference) {

  for (const char* path : {kJdotvConnect2dPath,
                           kJdotvConnect3dPath,
                           kJdotvWeld3dPath}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char err[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    ASSERT_THAT(m, NotNull()) << err << " for " << path;
    int nv = m->nv;
    mjData* d = mj_makeData(m);

    // simulate for 1 second to accumulate velocity
    while (d->time < 1.0) {
      mj_step(m, d);
    }

    // forward to populate constraints
    mj_forward(m, d);
    ASSERT_GT(d->ne, 0) << "no equality constraints for " << path;
    int ne = d->ne;

    // get dense J_0 (ne x nv)
    std::vector<mjtNum> J0(ne * nv);
    if (mj_isSparse(m)) {
      mju_sparse2dense(J0.data(), d->efc_J, ne, nv,
                       d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
    } else {
      mju_copy(J0.data(), d->efc_J, ne * nv);
    }

    // compute mj_Jdotv at current state
    std::vector<mjtNum> jdv(ne, 0);
    mj_Jdotv(m, d, jdv.data());

    // save qpos and qvel
    std::vector<mjtNum> qpos0(m->nq), qvel0(nv);
    mju_copy(qpos0.data(), d->qpos, m->nq);
    mju_copy(qvel0.data(), d->qvel, nv);

    // integrate qpos forward by h using qvel
    const mjtNum h = MjTol(1e-7, 5e-4);
    mj_integratePos(m, d->qpos, d->qvel, h);
    mj_forward(m, d);

    // get dense J_h (ne x nv)
    ASSERT_EQ(d->ne, ne) << "constraint count changed after integration";
    std::vector<mjtNum> Jh(ne * nv);
    if (mj_isSparse(m)) {
      mju_sparse2dense(Jh.data(), d->efc_J, ne, nv,
                       d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
    } else {
      mju_copy(Jh.data(), d->efc_J, ne * nv);
    }

    // FD: Jdotv_fd[i] = -sum_j (Jh[i,j] - J0[i,j]) / h * qvel[j]
    // (negated because mj_Jdotv subtracts)
    std::vector<mjtNum> jdv_fd(ne, 0);
    for (int i = 0; i < ne; i++) {
      for (int j = 0; j < nv; j++) {
        jdv_fd[i] -= (Jh[i*nv+j] - J0[i*nv+j]) / h * qvel0[j];
      }
    }

    // compare
    EXPECT_THAT(AsVector(jdv.data(), ne),
                Pointwise(MjNear(1e-4, 1e-2), AsVector(jdv_fd.data(), ne)))
        << "Jdotv FD mismatch for " << path;

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

// Test 2: forward-inverse identity preserved with Jdot*v correction
TEST_F(CoreConstraintTest, JdotvFwdInvIdentity) {
  for (const char* path : {kJdotvConnect2dPath,
                           kJdotvConnect3dPath,
                           kJdotvWeld3dPath}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char err[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    ASSERT_THAT(m, NotNull()) << err;
    mjData* d = mj_makeData(m);

    // give initial velocity
    for (int i = 0; i < m->nv; i++) d->qvel[i] = 0.5 * (i + 1);

    // forward (with correction ON by default)
    mj_forward(m, d);
    mj_compareFwdInv(m, d);
    mjtNum fwdinv = d->solver_fwdinv[0];

    mjtNum epsilon = MjTol(1e-10, 1e-2);
    EXPECT_LT(fwdinv, epsilon)
        << "fwdinv broken for " << path
        << " (fwdinv=" << fwdinv << ")";

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

}  // namespace
}  // namespace mujoco
