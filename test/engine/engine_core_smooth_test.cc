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

// Tests for engine/engine_core_smooth.c.

#include "src/engine/engine_core_smooth.h"

#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/types/span.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_misc.h"
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::string;
using ::std::vector;
using ::testing::Each;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::ElementsAre;
using ::testing::Not;
using ::testing::NotNull;
using ::testing::Pointwise;
using CoreSmoothTest = MujocoTest;

constexpr bool EndsWith(std::string_view str, std::string_view suffix) {
  return str.size() >= suffix.size() &&
         str.substr(str.size() - suffix.size()) == suffix;
}

// mjData values corresponding to the world body should be zero or identity
TEST_F(CoreSmoothTest, MjDataWorldBodyValuesAreInitialized) {
  constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag gravity="disable"/>
    </option>
    <worldbody/>
    <sensor>
      <subtreelinvel body="world"/>
    </sensor>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_resetDataDebug(model.get(), data.get(), 'd');
  mj_forward(model.get(), data.get());
  mj_rnePostConstraint(model.get(), data.get());

  {
#define X(type, name, d0, d1)                                               \
  if constexpr (std::string_view(#d0) == "nbody") {                         \
    absl::Span<type> values(data->name, model->d0 * d1);                    \
    if constexpr (EndsWith(#name, "quat")) {                                \
      EXPECT_THAT(values, ElementsAre(1, 0, 0, 0)) << #name;                \
    } else if constexpr (EndsWith(#name, "mat")) {                          \
      EXPECT_THAT(values, ElementsAre(1, 0, 0, 0, 1, 0, 0, 0, 1)) << #name; \
    } else if constexpr (std::string_view(#type) == "mjtNum") {             \
      EXPECT_THAT(values, Each(MjNear(0.0, 1e-7, 1e-7))) << #name;          \
    }                                                                       \
  }
    MJDATA_POINTERS
#undef X
  }
}

// --------------------------- mj_kinematics -----------------------------------

TEST_F(CoreSmoothTest, MjKinematicsWorldXipos) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  mj_resetDataDebug(model.get(), data.get(), 'd');
  mj_kinematics(model.get(), data.get());
  EXPECT_THAT(AsVector(&data->xipos[0], 3), ElementsAre(0, 0, 0));
}

// ----------------------------- mj_tendon -------------------------------------

TEST_F(CoreSmoothTest, FixedTendonSortedIndices) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="sparse"/>

    <worldbody>
      <body>
        <geom size=".1"/>
        <joint name="0"/>
      </body>
      <body pos="1 0 0">
        <geom size=".1"/>
        <joint name="1"/>
      </body>
      <body pos="2 0 0">
        <geom size=".1"/>
        <joint name="2"/>
      </body>
    </worldbody>

    <tendon>
      <fixed>
        <joint coef="3" joint="2"/>
        <joint coef="2" joint="1"/>
        <joint coef="1" joint="0"/>
      </fixed>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_EQ(model->ntendon, 1);
  ASSERT_EQ(model->nwrap, 3);

  MjDataPtr data = MakeData(model);
  mj_fwdPosition(model.get(), data.get());

  mjtNum* J = data->ten_J;
  EXPECT_THAT(vector<mjtNum>(J, J + 3), ElementsAre(1, 2, 3));
}

static const char* const kTen_J0 = "engine/testdata/core_smooth/ten_J0.xml";
static const char* const kTen_J1 = "engine/testdata/core_smooth/ten_J1.xml";
static const char* const kTen_J2 = "engine/testdata/core_smooth/ten_J2.xml";
static const char* const kTen_J3 = "engine/testdata/core_smooth/ten_J3.xml";

TEST_F(CoreSmoothTest, TendonJdot) {
  for (const char* local_path : {kTen_J0, kTen_J1, kTen_J2, kTen_J3}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    char error[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    int nv = m->nv;
    ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
    EXPECT_EQ(m->ntendon, 1);
    mjData* d = mj_makeData(m);

    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      m->opt.jacobian = sparsity;

      if (m->nkey) {
        mj_resetDataKeyframe(m, d, 0);
      } else {
        mj_resetData(m, d);
        while (d->time < 1) {
          mjtNum time = d->time;
          mj_step(m, d);
          ASSERT_GT(d->time, time) << "Divergence detected";
        }
      }

      mj_forward(m, d);

      // get current J for the tendon
      vector<mjtNum> ten_J(d->ten_J, d->ten_J + nv);

      // compute finite-differenced Jdot
      mjtNum h = MjTol(1e-7, 5e-4);
      mj_integratePos(m, d->qpos, d->qvel, h);
      mj_kinematics(m, d);
      mj_comPos(m, d);
      mj_tendon(m, d);
      vector<mjtNum> ten_Jh(d->ten_J, d->ten_J + nv);
      mju_subFrom(ten_Jh.data(), ten_J.data(), nv);
      mju_scl(ten_Jh.data(), ten_Jh.data(), 1.0 / h, nv);

      // test dot product against finite differences
      mjtNum dot = mj_tendonDot(m, d, 0, d->qvel);
      mjtNum expected_dot = mju_dot(ten_Jh.data(), d->qvel, nv);
      EXPECT_NEAR(dot, expected_dot, MjTol(1e-5, 2e-3));
    }

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

static const char* const kTen_offtree =
    "engine/testdata/core_smooth/ten_armature_offtree.xml";

TEST_F(CoreSmoothTest, TendonArmature) {
  const std::string xml_path = GetTestDataFilePath(kTen_offtree);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
  int nv = m->nv;
  mjData* d = mj_makeData(m);

  for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
    m->opt.jacobian = sparsity;

    mj_forward(m, d);

    // get full M, includes both CRB and tendon inertia
    vector<mjtNum> M(nv * nv);
    mj_fullM(m, d, M.data());

    // put only CRB inertia in M2
    mj_crb(m, d);
    vector<mjtNum> M2(nv * nv);
    mj_fullM(m, d, M2.data());

    vector<mjtNum> ten_J(nv);       // tendon Jacobian
    vector<mjtNum> ten_M(nv * nv);  // tendon inertia

    // add tendon inertias to M2 using outer product
    for (int j = 0; j < m->ntendon; j++) {
      // get tendon Jacobian
      int rowadr = m->ten_J_rowadr[j];
      int* rownnz = m->ten_J_rownnz + j;
      int zero = 0;
      mju_sparse2dense(ten_J.data(), d->ten_J + rowadr, 1, nv, rownnz, &zero,
                       m->ten_J_colind + rowadr);

      // get tendon inertia only, using outer product
      mju_mulMatMat(ten_M.data(), ten_J.data(), ten_J.data(), nv, 1, nv);
      mju_scl(ten_M.data(), ten_M.data(), m->tendon_armature[j], nv * nv);

      // manually add values, at nonzeros only
      for (int i = 0; i < nv * nv; i++) {
        if (M[i]) M2[i] += ten_M[i];
      }
    }

    // expect matrices to match
    EXPECT_THAT(M2, Pointwise(MjNear(1e-9, 1e-5), M));
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

static const char* const kTen_i0 =
    "engine/testdata/core_smooth/ten_armature_0.xml";
static const char* const kTen_i1 =
    "engine/testdata/core_smooth/ten_armature_1.xml";
static const char* const kTen_i2 =
    "engine/testdata/core_smooth/ten_armature_2.xml";
static const char* const kTen_i3 =
    "engine/testdata/core_smooth/ten_armature_3.xml";
static const char* const kTen_i4 =
    "engine/testdata/core_smooth/ten_armature_4.xml";

TEST_F(CoreSmoothTest, TendonArmatureConservesEnergy) {
  for (const char* local_path : {kTen_i0, kTen_i1, kTen_i2, kTen_i3, kTen_i4}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    char error[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
    mjData* d = mj_makeData(m);

    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      m->opt.jacobian = sparsity;

      mj_resetDataKeyframe(m, d, 0);
      mj_forward(m, d);

      double energy_0 = d->energy[0] + d->energy[1];

      double eps = std::max(energy_0, 1.0) * 1e-5;
      while (d->time < 1) {
        mjtNum time = d->time;
        mj_step(m, d);
        ASSERT_GT(d->time, time) << "Divergence detected";
        mjtNum energy_t = d->energy[0] + d->energy[1];
        EXPECT_NEAR(energy_t, energy_0, MjTol(eps, 3 * eps));
      }
    }
    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

TEST_F(CoreSmoothTest, TendonArmatureConservesMomentum) {
  const std::string xml_path = GetTestDataFilePath(kTen_i4);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
  mjData* d = mj_makeData(m);

  for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
    m->opt.jacobian = sparsity;

    mj_resetData(m, d);
    mj_forward(m, d);

    // this model contains subtreelinvel and subtreeangmom sensors
    vector<mjtNum> sdata_0 = AsVector(d->sensordata, m->nsensordata);
    EXPECT_THAT(sdata_0, Each(MjNear(0.0, 1e-15, 1e-7)));
    while (d->time < 1) {
      mjtNum time = d->time;
      mj_step(m, d);
      ASSERT_GT(d->time, time) << "Divergence detected";
      vector<mjtNum> sdata_t = AsVector(d->sensordata, m->nsensordata);
      EXPECT_THAT(sdata_t, Pointwise(MjNear(1e-5, 5e-4), sdata_0));
    }

    // momentum is conserved nontrivially (velocities are non-zero)
    EXPECT_GT(d->energy[1], 0);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

static const char* const kTen_i0_equiv =
    "engine/testdata/core_smooth/ten_armature_0_equiv.xml";
static const char* const kTen_i1_equiv =
    "engine/testdata/core_smooth/ten_armature_1_equiv.xml";

TEST_F(CoreSmoothTest, TendonInertiaEquivalent) {
  for (const char* lpath : {kTen_i0, kTen_i1}) {
    // load tendon model
    const std::string path = GetTestDataFilePath(lpath);
    char error[1024];
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
    int gid = mj_name2id(m, mjOBJ_GEOM, "query");
    mjData* d = mj_makeData(m);

    if (m->nkey) mj_resetDataKeyframe(m, d, 0);

    // load equivalent model
    const char* lpath_e = lpath == kTen_i0 ? kTen_i0_equiv : kTen_i1_equiv;
    const std::string path_e = GetTestDataFilePath(lpath_e);
    mjModel* m_e = mj_loadXML(path_e.c_str(), nullptr, error, sizeof(error));
    ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;
    int gid_e = mj_name2id(m_e, mjOBJ_GEOM, "query");
    mjData* d_e = mj_makeData(m_e);

    if (m_e->nkey) mj_resetDataKeyframe(m_e, d_e, 0);

    // the equality constraint in kTen_i1_equiv reduces precision
    double eps = lpath == kTen_i0 ? 1e-6 : 1e-3;

    while (d->time < 1) {
      mjtNum time = d->time;
      mj_step(m, d);
      ASSERT_GT(d->time, time) << "Divergence detected";
      vector<mjtNum> xpos = AsVector(d->geom_xpos + 3 * gid, 3);

      time = d_e->time;
      mj_step(m_e, d_e);
      ASSERT_GT(d_e->time, time) << "Divergence detected";
      vector<mjtNum> xpos_e = AsVector(d_e->geom_xpos + 3 * gid_e, 3);
      EXPECT_THAT(xpos, Pointwise(MjNear(eps, 10 * eps), xpos_e));
    }
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deleteData(d_e);
    mj_deleteModel(m_e);
  }
}

// --------------------------- connect constraint ------------------------------

// test that bodies hanging on connects lead to expected force sensor readings
void TestConnect(const char* const filepath) {
  const std::string xml_path = GetTestDataFilePath(filepath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i = 0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int i = 0; i < 3; i++) {
    EXPECT_THAT(data->sensordata[i] - model->sensor_user[i],
                MjNear(0, 1e-6, 2e-4));
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(CoreSmoothTest, RnePostConnectForceSlide) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/force_slide.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostConnectForceSlideRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/force_slide_rotated.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostConnectForceFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/force_free.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostConnectTorque) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/torque_free.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostConnectMultipleConstraints) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/multiple_constraints.xml";
  TestConnect(kModelFilePath);
}

// --------------------------- weld constraint ---------------------------------

// test that bodies attached with welds lead to expected force sensor readings
void TestWeld(const char* const filepath) {
  const std::string xml_path = GetTestDataFilePath(filepath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i = 0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int sensor_index = 0; sensor_index < model->nsensor; sensor_index++) {
    for (int i = 0; i < 3; i++) {
      EXPECT_NEAR(data->sensordata[model->sensor_adr[sensor_index] + i],
                  model->sensor_user[model->nuser_sensor * sensor_index + i],
                  MjTol(1e-6, 5e-5));
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(CoreSmoothTest, RnePostWeldForceFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_free.xml";
  TestWeld(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostWeldForceFreeRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_free_rotated.xml";
  TestWeld(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostWeldForceTorqueFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_torque_free.xml";
  TestWeld(kModelFilePath);
}

TEST_F(CoreSmoothTest, RnePostWeldForceTorqueFreeRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_torque_free_rotated.xml";
  TestWeld(kModelFilePath);
}

TEST_F(CoreSmoothTest, WeldRatioForceFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_force_free.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, WeldRatioForceSlide) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_force_slide.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, WeldRatioTorqueFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_torque_free.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, WeldRatioForceSlideRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/"
      "tfratio0_force_slide_rotated.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, WeldRatioMultipleConstraints) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/"
      "tfratio0_multiple_constraints.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, EqualityBodySite) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/equality_site_body_compare.xml");

  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // simulate, get sensordata
  while (data->time < 0.1) {
    mj_step(model, data);
  }
  vector<mjtNum> sdata = AsVector(data->sensordata, model->nsensordata);

  // reset
  mj_resetData(model, data);

  // turn site-defined equalities off, equivalent body-defined equalities on
  for (int e = 0; e < 4; e++) data->eq_active[e] = 1 - data->eq_active[e];

  // simulate again, get sensordata
  while (data->time < 0.1) {
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
  }

  // compare
  EXPECT_THAT(AsVector(data->sensordata, model->nsensordata),
              Pointwise(MjNear(1e-8, 5e-4), sdata));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// --------------------------- site actuators ----------------------------------

// Test Cartesian position control using site transmission with refsite
TEST_F(CoreSmoothTest, RefsiteBringsToPose) {
  constexpr char kRefsitePath[] = "engine/testdata/actuation/refsite.xml";
  const std::string xml_path = GetTestDataFilePath(kRefsitePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  // set pose target in ctrl (3 positions, 3 rotations)
  mjtNum targetpos[] = {.01, .02, .03};
  mjtNum targetrot[] = {.1, .2, .3};
  mju_copy3(data->ctrl, targetpos);
  mju_copy3(data->ctrl + 3, targetrot);

  // step for 5 seconds
  while (data->time < 10) {
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
  }

  // get site IDs
  int refsite_id = mj_name2id(model, mjOBJ_SITE, "reference");
  int site_id = mj_name2id(model, mjOBJ_SITE, "end_effector");

  // check that position matches target to within 1e-3 length units
  mjtNum relpos[3];
  mju_sub3(relpos, data->site_xpos + 3 * site_id,
           data->site_xpos + 3 * refsite_id);
  EXPECT_THAT(relpos, Pointwise(MjNear(1e-3, 5e-3), targetpos));

  // check that orientation matches target to within 0.06 radians
  mjtNum site_xquat[4], refsite_xquat[4], relrot[3];
  mju_mat2Quat(refsite_xquat, data->site_xmat + 9 * refsite_id);
  mju_mat2Quat(site_xquat, data->site_xmat + 9 * site_id);
  mju_subQuat(relrot, site_xquat, refsite_xquat);
  EXPECT_THAT(relrot, Pointwise(MjNear(0.06, 0.06), targetrot));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Test Cartesian position control w.r.t moving refsite
TEST_F(CoreSmoothTest, RefsiteConservesMomentum) {
  constexpr char kRefsitePath[] = "engine/testdata/actuation/refsite_free.xml";
  const std::string xml_path = GetTestDataFilePath(kRefsitePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  // this test asserts tight momentum conservation: solve exactly, no early termination
  model->opt.tolerance = 0;

  data->ctrl[0] = 1;
  data->ctrl[1] = -1;

  // simulate, assert that momentum is conserved
  mjtNum eps = MjTol(1e-9, 2e-6);
  while (data->time < 1) {
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
    for (int i = 0; i < 6; i++) {
      EXPECT_LT(mju_abs(data->sensordata[i]), eps);
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Test smooth tracking of a rotational target ramped through the pi boundary.
// Rotational transmission lengths live in (-pi, pi]; a servo whose error is
// computed in the chart rather than on the circle loses the target once it
// crosses pi and enters a phase-slipping limit cycle.
TEST_F(CoreSmoothTest, RefsiteTracksWindingTarget) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <site name="reference"/>
      <body name="box">
        <freejoint/>
        <geom type="box" size=".05 .07 .03"/>
        <site name="end_effector"/>
      </body>
    </worldbody>
    <actuator>
      <position name="rz" site="end_effector" refsite="reference"
                gear="0 0 0 0 0 1" kp="1" dampratio="1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  int rz = mj_name2id(model.get(), mjOBJ_ACTUATOR, "rz");
  ASSERT_GE(rz, 0);

  // ramp the rz target from 0 to 2*pi, slowly enough to track
  const mjtNum rate = 0.5;  // rad/s
  while (data->time < 2 * mjPI / rate) {
    data->ctrl[rz] = rate * data->time;
    mj_step(model.get(), data);

    // distance between target and actuator length, measured on the circle
    mjtNum error = data->ctrl[rz] - data->actuator_length[rz];
    error -= 2 * mjPI * mju_round(error / (2 * mjPI));
    ASSERT_LT(mju_abs(error), 0.5)
        << "tracking lost at time " << data->time << ", target "
        << data->ctrl[rz] << ", length " << data->actuator_length[rz];
  }

  mj_deleteData(data);
}

// Test single-axis winding on a ball joint with per-axis (wrapped) servos:
// a target ramped through pi is tracked smoothly.
TEST_F(CoreSmoothTest, BallTracksWindingTarget) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <position name="rx" joint="ball" gear="1 0 0" kp="1" dampratio="1"/>
      <position name="ry" joint="ball" gear="0 1 0" kp="1" dampratio="1"/>
      <position name="rz" joint="ball" gear="0 0 1" kp="1" dampratio="1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  int rz = mj_name2id(model.get(), mjOBJ_ACTUATOR, "rz");
  ASSERT_GE(rz, 0);

  // ramp the rz target from 0 to 2*pi, assert tracking on the circle
  const mjtNum rate = 0.5;  // rad/s
  while (data->time < 2 * mjPI / rate) {
    data->ctrl[rz] = rate * data->time;
    mj_step(model.get(), data);
    mjtNum error = data->ctrl[rz] - data->actuator_length[rz];
    error -= 2 * mjPI * mju_round(error / (2 * mjPI));
    ASSERT_LT(mju_abs(error), 0.5) << "tracking lost at time " << data->time;
  }

  mj_deleteData(data);
}

// Wrapped rotational intvelocity: actrange is optional, constant ctrl produces
// steady rotation over many periods, activation stays bounded.
TEST_F(CoreSmoothTest, IntVelocityWindsWithBoundedAct) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity name="rz" joint="ball" gear="0 0 1" kp="1" dampratio="1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  // command constant angular rate for 4 full turns
  const mjtNum rate = 1.0;  // rad/s
  data->ctrl[0] = rate;
  while (data->time < 8 * mjPI / rate) {
    mj_step(model.get(), data);
    ASSERT_LT(mju_abs(data->act[0]), mjPI + 0.1) << "act unbounded";
  }

  // steady rotation at the commanded rate
  EXPECT_NEAR(data->actuator_velocity[0], rate, 0.01);

  mj_deleteData(data);
}

// mj_forward must not mutate state: wrapping of act happens at integration
// time (mj_advance), never in the forward pass.
TEST_F(CoreSmoothTest, ForwardDoesNotMutateAct) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity name="rz" joint="ball" gear="0 0 1" kp="1" dampratio="1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  // forward leaves a far-from-length activation untouched, bit-for-bit
  data->act[0] = 100;
  mj_forward(model.get(), data);
  mj_forward(model.get(), data);
  EXPECT_EQ(data->act[0], 100);

  // stepping wraps it to a bounded representative
  mj_step(model.get(), data);
  EXPECT_LT(mju_abs(data->act[0]), mjPI + 0.1);

  mj_deleteData(data);
}

// expmap (axis-angle) vector to quaternion
static void Expmap2Quat(mjtNum quat[4], const mjtNum v[3]) {
  mjtNum angle = mju_norm3(v);
  if (angle < mjMINVAL) {
    quat[0] = 1;
    quat[1] = quat[2] = quat[3] = 0;
  } else {
    mjtNum axis[3] = {v[0]/angle, v[1]/angle, v[2]/angle};
    mju_axisAngle2Quat(quat, axis, angle);
  }
}

// geodesic distance between the orientations given by expmap vectors u and v
static mjtNum GeodesicError(const mjtNum u[3], const mjtNum v[3]) {
  mjtNum q_tgt[4], q_cur[4], q_err[4], e[3];
  Expmap2Quat(q_tgt, u);
  Expmap2Quat(q_cur, v);
  mju_negQuat(q_cur, q_cur);
  mju_mulQuat(q_err, q_tgt, q_cur);
  mju_quat2Vel(e, q_err, 1);
  return mju_norm3(e);
}

// mixed model: three scalar translation servos and one SO3 orientation servo
static constexpr char kSO3RefsiteXml[] = R"(
<mujoco>
  <option integrator="implicitfast">
    <flag contact="disable" gravity="disable"/>
  </option>
  <worldbody>
    <site name="reference"/>
    <body name="box">
      <freejoint/>
      <geom type="box" size=".05 .07 .03"/>
      <site name="end_effector"/>
    </body>
  </worldbody>
  <actuator>
    <position name="x" site="end_effector" refsite="reference" gear="1 0 0 0 0 0"
              kp="100" dampratio="1"/>
    <orientation name="orient" site="end_effector" refsite="reference" kp="1" dampratio="1"/>
    <position name="y" site="end_effector" refsite="reference" gear="0 1 0 0 0 0"
              kp="100" dampratio="1"/>
    <position name="z" site="end_effector" refsite="reference" gear="0 0 1 0 0 0"
              kp="100" dampratio="1"/>
  </actuator>
  <sensor>
    <actuatorpos actuator="orient"/>
    <actuatorfrc actuator="orient"/>
  </sensor>
</mujoco>
)";

// Layout of a mixed model: the SO3 actuator owns 3-wide control and output
// blocks, misaligning nu/nout/nactuator with the actuator index; sensors on it
// are 3-dimensional; the model round-trips through XML.
TEST_F(CoreSmoothTest, SO3MixedModelLayout) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kSO3RefsiteXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  // counts: 4 actuators, 3+3 controls, 3+3 force outputs
  EXPECT_EQ(model->nactuator, 4);
  EXPECT_EQ(model->nu, 6);
  EXPECT_EQ(model->nout, 6);

  // the orientation actuator is second, exercising address accumulation
  int orient = mj_name2id(model.get(), mjOBJ_ACTUATOR, "orient");
  ASSERT_EQ(orient, 1);
  EXPECT_EQ(model->actuator_ctrladr[orient], 1);
  EXPECT_EQ(model->actuator_ctrlnum[orient], 3);
  EXPECT_EQ(model->actuator_outadr[orient], 1);
  EXPECT_EQ(model->actuator_outnum[orient], 3);
  EXPECT_EQ(model->actuator_trntype[orient], mjTRN_SO3);

  // actuator sensors report one value per force output
  EXPECT_EQ(model->sensor_dim[0], 3);
  EXPECT_EQ(model->sensor_dim[1], 3);
  EXPECT_EQ(model->sensor_adr[1], 3);

  // XML round-trip preserves the layout
  std::string saved = SaveAndReadXml(model.get());
  MjModelPtr model2 = LoadModelFromString(saved.c_str(), error, sizeof(error));
  ASSERT_THAT(model2.get(), NotNull()) << error;
  EXPECT_EQ(model2->nactuator, 4);
  EXPECT_EQ(model2->nu, 6);
  EXPECT_EQ(model2->nout, 6);
  EXPECT_EQ(model2->actuator_trntype[orient], mjTRN_SO3);
}

// A mixed-axis orientation target beyond the pi shell is a true equilibrium:
// zero force when the body is at the commanded orientation, convergence to it
// from the initial state.
TEST_F(CoreSmoothTest, SO3RefsiteMixedAxisEquilibrium) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kSO3RefsiteXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  int orient = mj_name2id(model.get(), mjOBJ_ACTUATOR, "orient");
  int uadr = model->actuator_ctrladr[orient];
  int oadr = model->actuator_outadr[orient];

  // target: 5.66 rad rotation about the mixed axis (1,1,0)/sqrt(2), beyond pi;
  // canonical (shortest) expmap is u*(1 - 2*pi/norm(u)) = (-.4429, -.4429, 0)
  mjtNum target[3] = {4, 4, 0};
  mjtNum shrink = 1 - 2*mjPI/mju_norm3(target);
  mjtNum canonical[3] = {target[0]*shrink, target[1]*shrink, target[2]*shrink};
  mju_copy3(data->ctrl + uadr, target);

  // place the body exactly at the commanded orientation: force must vanish
  Expmap2Quat(data->qpos + 3, target);
  mj_forward(model.get(), data);
  for (int k=0; k < 3; k++) {
    EXPECT_LT(mju_abs(data->actuator_force[oadr + k]), MjTol(1e-10, 1e-6));
    EXPECT_LT(mju_abs(data->actuator_length[oadr + k] - canonical[k]),
              MjTol(1e-10, 1e-6));

    // sensors: actuatorpos = canonical expmap, actuatorfrc = 0
    EXPECT_LT(mju_abs(data->sensordata[k] - canonical[k]), MjTol(1e-10, 1e-6));
    EXPECT_LT(mju_abs(data->sensordata[3 + k]), MjTol(1e-10, 1e-6));
  }

  // from the initial state, converge to the commanded orientation
  mj_resetData(model.get(), data);
  mju_copy3(data->ctrl + uadr, target);
  while (data->time < 10) {
    mj_step(model.get(), data);
  }
  for (int k=0; k < 3; k++) {
    EXPECT_LT(mju_abs(data->actuator_length[oadr + k] - canonical[k]), 1e-3);
    EXPECT_LT(mju_abs(data->actuator_velocity[oadr + k]), 1e-3);
  }

  mj_deleteData(data);
}

// Test smooth tracking while winding one axis with another axis held nonzero:
// the regime where per-axis servo errors cannot work and only the geodesic
// error on SO(3) tracks correctly.
TEST_F(CoreSmoothTest, SO3RefsiteTracksMixedWindingTarget) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kSO3RefsiteXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  int orient = mj_name2id(model.get(), mjOBJ_ACTUATOR, "orient");
  int uadr = model->actuator_ctrladr[orient];
  int oadr = model->actuator_outadr[orient];

  // hold rx at 1 rad, let the servo settle
  data->ctrl[uadr] = 1;
  while (data->time < 2) {
    mj_step(model.get(), data);
  }

  // ramp the rz target from 0 to 2*pi
  const mjtNum rate = 0.5;  // rad/s
  mjtNum start = data->time;
  while (data->time - start < 2*mjPI / rate) {
    data->ctrl[uadr + 2] = rate * (data->time - start);
    mj_step(model.get(), data);

    // geodesic distance between commanded and current orientation
    mjtNum err = GeodesicError(data->ctrl + uadr, data->actuator_length + oadr);
    ASSERT_LT(err, 0.5) << "tracking lost at time " << data->time
                        << ", target rz " << data->ctrl[uadr + 2];
  }

  mj_deleteData(data);
}

// Mixed-axis target beyond the pi shell on ball joints: the SO3 actuator has
// an exact equilibrium at the commanded orientation, per-axis wrapped servos
// do not.
TEST_F(CoreSmoothTest, SO3BallMixedAxisContrast) {
  constexpr char kOrientationPath[] =
      "engine/testdata/actuation/orientation.xml";
  const std::string xml_path = GetTestDataFilePath(kOrientationPath);
  char error[1024];
  MjModelPtr model(mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error)));
  ASSERT_THAT(model.get(), NotNull()) << "Failed to load model: " << error;
  mjData* data = mj_makeData(model.get());

  // place both joints exactly at the target orientation, expmap (4, 4, 0)
  mjtNum target[3] = {4, 4, 0};
  mjtNum quat[4];
  Expmap2Quat(quat, target);
  for (const char* name : {"peraxis", "expmap"}) {
    int jnt = mj_name2id(model.get(), mjOBJ_JOINT, name);
    mju_copy4(data->qpos + model->jnt_qposadr[jnt], quat);
  }

  int rx_peraxis = mj_name2id(model.get(), mjOBJ_ACTUATOR, "rx_peraxis");
  int orient = mj_name2id(model.get(), mjOBJ_ACTUATOR, "expmap");
  int uadr = model->actuator_ctrladr[orient];
  int oadr = model->actuator_outadr[orient];
  data->ctrl[rx_peraxis] = data->ctrl[rx_peraxis + 1] = 4;
  data->ctrl[uadr] = data->ctrl[uadr + 1] = 4;
  mj_forward(model.get(), data);

  // SO3: zero force at the commanded orientation
  for (int k=0; k < 3; k++) {
    EXPECT_LT(mju_abs(data->actuator_force[oadr + k]), MjTol(1e-10, 1e-6));
  }

  // per-axis: residual force kp*(4.443 - 2*pi) = -1.84 on the wound members
  EXPECT_GT(mju_abs(data->actuator_force[rx_peraxis]), 1);
  EXPECT_GT(mju_abs(data->actuator_force[rx_peraxis + 1]), 1);

  mj_deleteData(data);
}

// Geodesic servo converges to large mixed-axis targets from rest: no limit
// cycles. Regression test: a parent-frame error driving child-frame torques is
// non-gradient feedback which pumps energy at large angles, settling into
// steady spinning.
TEST_F(CoreSmoothTest, SO3LargeAngleConvergence) {
  constexpr char kOrientationPath[] =
      "engine/testdata/actuation/orientation.xml";
  const std::string xml_path = GetTestDataFilePath(kOrientationPath);
  char error[1024];
  MjModelPtr model(mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error)));
  ASSERT_THAT(model.get(), NotNull()) << "Failed to load model: " << error;
  mjData* data = mj_makeData(model.get());

  int expmap = mj_name2id(model.get(), mjOBJ_ACTUATOR, "expmap");
  int jnt = mj_name2id(model.get(), mjOBJ_JOINT, "expmap");
  int uadr = model->actuator_ctrladr[expmap];
  int dofadr = model->jnt_dofadr[jnt];

  mjtNum targets[4][3] = {{1, 1, 1}, {-1, -1, 0}, {1, -1, 1}, {2.2, 2.2, 2.2}};
  for (const auto& u : targets) {
    SCOPED_TRACE(testing::Message()
                 << "target (" << u[0] << ", " << u[1] << ", " << u[2] << ")");
    mj_resetData(model.get(), data);
    mju_copy3(data->ctrl + uadr, u);
    for (int i = 0; i < 4000; i++) {
      mj_step(model.get(), data);
    }

    // orientation error and angular velocity vanish
    mjtNum q_tgt[4], q_cur[4], e[3];
    mjtNum axis[3] = {u[0], u[1], u[2]};
    mjtNum angle = mju_normalize3(axis);
    mju_axisAngle2Quat(q_tgt, axis, angle);
    mju_copy4(q_cur, data->qpos + model->jnt_qposadr[jnt]);
    mju_normalize4(q_cur);
    mju_subQuat(e, q_tgt, q_cur);
    EXPECT_LT(mju_norm3(e), MjTol(1e-4, 1e-2));
    EXPECT_LT(mju_norm3(data->qvel + dofadr), MjTol(1e-4, 1e-2));
  }
  mj_deleteData(data);
}

// Neutral ctrl: reset zeroes all controls except quat (to the identity).
TEST_F(CoreSmoothTest, SO3QuatNeutralCtrl) {
  constexpr char kOrientationPath[] =
      "engine/testdata/actuation/orientation.xml";
  const std::string xml_path = GetTestDataFilePath(kOrientationPath);
  char error[1024];
  MjModelPtr model(mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error)));
  ASSERT_THAT(model.get(), NotNull()) << "Failed to load model: " << error;
  mjData* data = mj_makeData(model.get());

  int quat = mj_name2id(model.get(), mjOBJ_ACTUATOR, "quat");
  int uadr = model->actuator_ctrladr[quat];
  for (int trial = 0; trial < 2; trial++) {
    for (int j = 0; j < model->nu; j++) {
      EXPECT_EQ(data->ctrl[j], j == uadr ? 1 : 0)
          << "ctrl " << j << " trial " << trial;
    }
    mju_fill(data->ctrl, 0.5, model->nu);
    mj_resetData(model.get(), data);
  }
  mj_deleteData(data);
}

// Input names: NULL for single-input actuators, chart components for SO3.
TEST_F(CoreSmoothTest, ActuatorInputNames) {
  constexpr char kOrientationPath[] =
      "engine/testdata/actuation/orientation.xml";
  const std::string xml_path = GetTestDataFilePath(kOrientationPath);
  char error[1024];
  MjModelPtr model(mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error)));
  ASSERT_THAT(model.get(), NotNull()) << "Failed to load model: " << error;

  int rx_peraxis = mj_name2id(model.get(), mjOBJ_ACTUATOR, "rx_peraxis");
  int expmap = mj_name2id(model.get(), mjOBJ_ACTUATOR, "expmap");
  int quat = mj_name2id(model.get(), mjOBJ_ACTUATOR, "quat");
  EXPECT_EQ(mj_actuatorInputName(model.get(), rx_peraxis, 0), nullptr);
  EXPECT_STREQ(mj_actuatorInputName(model.get(), expmap, 0), "rx");
  EXPECT_STREQ(mj_actuatorInputName(model.get(), expmap, 2), "rz");
  EXPECT_EQ(mj_actuatorInputName(model.get(), expmap, 3),
            nullptr);  // out of range
  EXPECT_STREQ(mj_actuatorInputName(model.get(), quat, 0), "qw");
  EXPECT_STREQ(mj_actuatorInputName(model.get(), quat, 3), "qz");
}

// SO3 integrator variant: act is the 3D orientation setpoint; constant ctrl
// produces steady rotation over many periods with bounded activation.
TEST_F(CoreSmoothTest, SO3IntVelocityWindsWithBoundedAct) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <general name="rot" joint="ball" dyntype="integrator"
               gaintype="so3" biastype="so3" gainprm="1" biasprm="0 -1 -1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  // one actuator: 3 controls, 3 outputs, 3 activations
  EXPECT_EQ(model->nu, 3);
  EXPECT_EQ(model->nout, 3);
  EXPECT_EQ(model->na, 3);

  mjData* data = mj_makeData(model.get());

  // spin about z for 4 full turns
  const mjtNum rate = 1.0;  // rad/s
  data->ctrl[2] = rate;
  while (data->time < 8*mjPI / rate) {
    mj_step(model.get(), data);
    ASSERT_LT(mju_norm3(data->act), mjPI + 0.1) << "act unbounded";
  }

  // steady rotation at the commanded rate about z
  EXPECT_NEAR(data->actuator_velocity[2], rate, 0.02);

  mj_deleteData(data);
}

// Compile-time validation of the SO3 actuator.
TEST_F(CoreSmoothTest, SO3CompileErrors) {
  char error[1024];

  // hinge joint target: rejected
  static constexpr char hinge_xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size=".05"/>
      </body>
    </worldbody>
    <actuator>
      <orientation joint="hinge" kp="1"/>
    </actuator>
  </mujoco>
  )";
  MjModelPtr model = LoadModelFromString(hinge_xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("ball"));

  // site without refsite: rejected
  static constexpr char nosite_xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <geom size=".05"/>
        <site name="ee"/>
      </body>
    </worldbody>
    <actuator>
      <orientation site="ee" kp="1"/>
    </actuator>
  </mujoco>
  )";
  model = LoadModelFromString(nosite_xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("refsite"));

  // mismatched gaintype/biastype: rejected
  static constexpr char mismatch_xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="ball" gaintype="so3" biastype="affine" gainprm="1"/>
    </actuator>
  </mujoco>
  )";
  model = LoadModelFromString(mismatch_xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("both"));

  // forcerange with nonzero lower bound: rejected (force clamped on the norm)
  static constexpr char forcerange_xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <orientation joint="ball" kp="1" forcerange="-4 4"/>
    </actuator>
  </mujoco>
  )";
  model = LoadModelFromString(forcerange_xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("lower bound must be 0"));
}

// forcerange clamps the norm of the SO3 output torque, preserving direction.
TEST_F(CoreSmoothTest, SO3ForcerangeClampsNorm) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <body>
        <joint name="unclamped" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
      <body pos="0 0 .3">
        <joint name="clamped" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <orientation name="unclamped" joint="unclamped" kp="1"/>
      <orientation name="clamped" joint="clamped" kp="1" forcerange="0 .5"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  mjData* data = mj_makeData(model.get());

  // command the same mixed-axis target, error norm sqrt(5) > 0.5
  int unclamped = mj_name2id(model.get(), mjOBJ_ACTUATOR, "unclamped");
  int clamped = mj_name2id(model.get(), mjOBJ_ACTUATOR, "clamped");
  for (int i : {unclamped, clamped}) {
    int uadr = model->actuator_ctrladr[i];
    data->ctrl[uadr + 0] = 1;
    data->ctrl[uadr + 1] = 2;
    data->ctrl[uadr + 2] = 0;
  }
  mj_forward(model.get(), data);

  // clamped force has norm forcerange[1], parallel to the unclamped force
  const mjtNum* f_unclamped =
      data->actuator_force + model->actuator_outadr[unclamped];
  const mjtNum* f_clamped =
      data->actuator_force + model->actuator_outadr[clamped];
  mjtNum norm_unclamped = mju_norm3(f_unclamped);
  EXPECT_GT(norm_unclamped, 0.5);
  EXPECT_NEAR(mju_norm3(f_clamped), 0.5, MjTol(1e-12, 1e-6));
  mjtNum scale = 0.5 / norm_unclamped;
  for (int k = 0; k < 3; k++) {
    EXPECT_NEAR(f_clamped[k], scale * f_unclamped[k], MjTol(1e-12, 1e-6));
  }

  mj_deleteData(data);
}

// Quat-setpoint variant of the SO3 servo: 4 inputs, 3 outputs -- the first
// actuator with different input and output widths.
TEST_F(CoreSmoothTest, SO3QuatSetpoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <body pos="-.15 0 .2">
        <joint name="ball_scalar" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
      <body pos=".15 0 .2">
        <joint name="ball_quat" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <position name="rz" joint="ball_scalar" gear="0 0 1" kp="1" dampratio="1"/>
      <orientation name="orient" joint="ball_quat" kp="1" dampratio="1" input="quat"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  // rectangular layout: 2 actuators, 1+4 controls, 1+3 force outputs
  EXPECT_EQ(model->nactuator, 2);
  EXPECT_EQ(model->nu, 5);
  EXPECT_EQ(model->nout, 4);
  int orient = mj_name2id(model.get(), mjOBJ_ACTUATOR, "orient");
  int uadr = model->actuator_ctrladr[orient];
  int oadr = model->actuator_outadr[orient];
  EXPECT_EQ(uadr, 1);
  EXPECT_EQ(model->actuator_ctrlnum[orient], 4);
  EXPECT_EQ(oadr, 1);
  EXPECT_EQ(model->actuator_outnum[orient], 3);

  // XML round-trip preserves the input chart
  std::string saved = SaveAndReadXml(model.get());
  MjModelPtr model2 = LoadModelFromString(saved.c_str(), error, sizeof(error));
  ASSERT_THAT(model2.get(), NotNull()) << error;
  EXPECT_EQ(model2->nu, 5);
  EXPECT_EQ(model2->nout, 4);

  mjData* data = mj_makeData(model.get());

  // zero ctrl commands the identity orientation: zero force at qpos0
  mj_forward(model.get(), data);
  for (int k=0; k < 3; k++) {
    EXPECT_LT(mju_abs(data->actuator_force[oadr + k]), MjTol(1e-10, 1e-6));
  }

  // target beyond the pi shell, mixed axis
  mjtNum target[3] = {4, 4, 0};
  mjtNum q_tgt[4];
  Expmap2Quat(q_tgt, target);
  int jnt = mj_name2id(model.get(), mjOBJ_JOINT, "ball_quat");

  // scale and antipodal invariance: q, 2q and -q command the same orientation
  mjtNum ctrl_variants[3][4];
  mju_copy4(ctrl_variants[0], q_tgt);
  for (int k=0; k < 4; k++) {
    ctrl_variants[1][k] = 2*q_tgt[k];
    ctrl_variants[2][k] = -q_tgt[k];
  }
  for (int v=0; v < 3; v++) {
    mj_resetData(model.get(), data);
    mju_copy4(data->qpos + model->jnt_qposadr[jnt], q_tgt);
    mju_copy4(data->ctrl + uadr, ctrl_variants[v]);
    mj_forward(model.get(), data);
    for (int k=0; k < 3; k++) {
      EXPECT_LT(mju_abs(data->actuator_force[oadr + k]), MjTol(1e-10, 1e-6))
          << "variant " << v;
    }
  }

  // from the initial state, converge to the commanded orientation
  mj_resetData(model.get(), data);
  mju_copy4(data->ctrl + uadr, q_tgt);
  while (data->time < 10) {
    mj_step(model.get(), data);
  }
  mjtNum shrink = 1 - 2*mjPI/mju_norm3(target);
  for (int k=0; k < 3; k++) {
    EXPECT_LT(mju_abs(data->actuator_length[oadr + k] - target[k] * shrink),
              1e-3);
    EXPECT_LT(mju_abs(data->actuator_velocity[oadr + k]), 1e-3);
  }

  mj_deleteData(data);
}

// The quat input chart requires stateless dynamics.
TEST_F(CoreSmoothTest, SO3QuatSetpointRequiresStateless) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="ball" type="ball"/>
        <geom type="box" size=".05 .07 .03"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="ball" dyntype="integrator" input="quat"
               gaintype="so3" biastype="so3" gainprm="1" biasprm="0 -1 -1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("dyntype"));
}

static const char* const kInertiaPath = "engine/testdata/inertia.xml";

TEST_F(CoreSmoothTest, FactorI) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error;

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // dense L matrix
  int nv = model->nv;
  vector<mjtNum> Ldense(nv * nv, 0);
  mju_sparse2dense(Ldense.data(), data->qLD, nv, nv, model->M_rownnz,
                   model->M_rowadr, model->M_colind);
  for (int i = 0; i < nv; i++) {
    // set diagonal to 1
    Ldense[i * nv + i] = 1;
  }

  // dense D matrix
  vector<mjtNum> Ddense(nv * nv);
  mju_sparse2dense(Ddense.data(), data->qLD, nv, nv, model->M_rownnz,
                   model->M_rowadr, model->M_colind);
  for (int i = 0; i < nv; i++) {
    for (int j = 0; j < nv; j++) {
      // zero everything except the diagonal
      if (i != j) Ddense[i * nv + j] = 0;
    }
  }

  // perform multiplication: M = L^T * D * L
  vector<mjtNum> tmp(nv * nv);
  vector<mjtNum> M(nv * nv);
  mju_mulMatMat(tmp.data(), Ddense.data(), Ldense.data(), nv, nv, nv);
  mju_mulMatTMat(M.data(), Ldense.data(), tmp.data(), nv, nv, nv);

  // dense M matrix
  vector<mjtNum> Mexpected(nv * nv);
  mj_fullM(model, data, Mexpected.data());

  // expect matrices to match to floating point precision
  EXPECT_THAT(M, Pointwise(MjNear(1e-12, 3e-4), Mexpected));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(CoreSmoothTest, SolveLD) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  int nv = m->nv;

  // arbitrary RHS vector y
  vector<mjtNum> y(nv);
  for (int i = 0; i < nv; i++) y[i] = 20 + 30 * i;
  for (int i = 0; i < nv; i += 2) y[i] = 0;

  // x = inv(M) * y
  vector<mjtNum> x = y;
  mj_solveLD(x.data(), d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr,
             m->M_colind, nullptr);

  // z = M * x
  vector<mjtNum> z(nv);
  mj_mulM(m, d, z.data(), x.data());

  // expect z to match y
  for (int i = 0; i < nv; i++) {
    EXPECT_NEAR(z[i], y[i], MjTol(1e-12, 5e-4));
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(CoreSmoothTest, SolveLDmultipleVectors) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  int nv = m->nv;
  int n = 3;

  // Y: arbitrary RHS vectors (nv x n)
  vector<mjtNum> Y(nv * n);
  for (int i = 0; i < nv * n; i++) Y[i] = 2 + 3 * i;
  for (int i = 0; i < nv * n; i += 3) Y[i] = 0;

  // X = inv(M) * Y
  vector<mjtNum> X = Y;
  mj_solveLD(X.data(), d->qLD, d->qLDiagInv, nv, n, m->M_rownnz, m->M_rowadr,
             m->M_colind, nullptr);

  // verify each vector: Z_i = M * X_i
  for (int i = 0; i < n; i++) {
    vector<mjtNum> z(nv);
    mj_mulM(m, d, z.data(), X.data() + i * nv);
    for (int j = 0; j < nv; j++) {
      EXPECT_NEAR(z[j], Y[i * nv + j], MjTol(1e-12, 5e-4));
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(CoreSmoothTest, SolveM2) {
  const std::string xml_path = GetTestDataFilePath(kInertiaPath);
  char error[1024];
  mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error;

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // inverse square root of D from inertia LDL decomposition
  int nv = m->nv;
  vector<mjtNum> sqrtInvD(nv);
  for (int i = 0; i < nv; i++) {
    int diag = m->M_rowadr[i] + m->M_rownnz[i] - 1;
    sqrtInvD[i] = 1 / mju_sqrt(d->qLD[diag]);
  }

  // compare full solve and half solve
  int n = 3;
  vector<mjtNum> vec(nv * n);
  vector<mjtNum> vec2(nv * n);
  for (int i = 0; i < nv * n; i++) vec[i] = vec2[i] = 2 + 3 * i;
  for (int i = 0; i < nv * n; i += 3) vec[i] = vec2[i] = 0;
  vector<mjtNum> res(nv * n);

  mj_solveM2(m, d, res.data(), vec.data(), sqrtInvD.data(), n);
  mj_solveLD(vec2.data(), d->qLD, d->qLDiagInv, nv, n, m->M_rownnz, m->M_rowadr,
             m->M_colind, nullptr);

  // expect equality of dot(v, M^-1 * v) and dot(M^-1/2 * v, M^-1/2 * v)
  for (int i = 0; i < n; i++) {
    EXPECT_NEAR(mju_dot(vec2.data() + i * nv, vec.data() + i * nv, nv),
                mju_dot(res.data() + i * nv, res.data() + i * nv, nv),
                MjTol(1e-10, 1e-2));
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}



TEST_F(CoreSmoothTest, FlexVertLengthScaling) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="sparse"/>
    <worldbody>
      <flexcomp name="g" type="grid" count="3 3 1" spacing="1 1 1" dim="2" radius=".05">
        <edge equality="vert"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);

  // check that nJfv is correct:
  // corner vertices: 2 * (1+3) * 3 + 2 * (1+2) * 3 = 42
  // edge vertices: 4 * (1+4) * 3 = 60
  // center vertex: 1 * (1+6) * 3 = 21
  // nJfv = 42 + 60 + 21 = 123
  EXPECT_EQ(m->nJfv, 123);

  // Run kinematics to populate xpos/xmat initially
  mj_fwdKinematics(m.get(), d.get());

  // Check invariants for scale=1
  // The constraints should be satisfied
  int nvert = m->flex_vertnum[0];
  ASSERT_EQ(nvert, 9);
  for (int i = 0; i < nvert; i++) {
    EXPECT_NEAR(d->flexvert_length[2 * i + 0], 0.0, MjTol(1e-5, 5e-5));
    EXPECT_NEAR(d->flexvert_length[2 * i + 1], 0.0, MjTol(1e-5, 5e-5));
  }

  // set qvel to rigid rotation
  ASSERT_EQ(m->nv, 3 * nvert);
  mju_zero(d->qvel, m->nv);
  for (int i = 0; i < nvert; i++) {
    const mjtNum* p = d->xpos + 3 * m->flex_vertbodyid[i];
    d->qvel[3 * i + 0] = -p[1];
    d->qvel[3 * i + 1] = p[0];
    d->qvel[3 * i + 2] = 1.0;
  }

  // check that Jacobian times velocity is zero for rigid body motion
  vector<mjtNum> Jv(2 * nvert, 0);
  for (int i = 0; i < 2 * nvert; i++) {
    int row_start = m->flexvert_J_rowadr[i];
    int row_nnz = m->flexvert_J_rownnz[i];
    for (int j = 0; j < row_nnz; j++) {
      Jv[i] += d->flexvert_J[row_start + j] *
               d->qvel[m->flexvert_J_colind[row_start + j]];
    }
  }
  EXPECT_THAT(Jv, Each(MjNear(0.0, 1e-7, 1e-4)));

  // check sparsity pattern
  int corners[] = {0, 2, 6, 8};
  int edges[] = {1, 3, 5, 7};
  int center[] = {4};
  for (int i : corners) {
    EXPECT_EQ(m->flexvert_J_rownnz[2 * i + 0], (i == 0 || i == 8) ? 12 : 9);
    EXPECT_EQ(m->flexvert_J_rownnz[2 * i + 1], (i == 0 || i == 8) ? 12 : 9);
  }
  for (int i : edges) {
    EXPECT_EQ(m->flexvert_J_rownnz[2 * i + 0], 15);
    EXPECT_EQ(m->flexvert_J_rownnz[2 * i + 1], 15);
  }
  for (int i : center) {
    EXPECT_EQ(m->flexvert_J_rownnz[2 * i + 0], 21);
    EXPECT_EQ(m->flexvert_J_rownnz[2 * i + 1], 21);
  }

  // check rowadr
  EXPECT_EQ(m->flexvert_J_rowadr[0], 0);
  for (int i = 1; i < 2 * nvert; i++) {
    EXPECT_EQ(m->flexvert_J_rowadr[i],
              m->flexvert_J_rowadr[i - 1] + m->flexvert_J_rownnz[i - 1]);
  }

  // check that colind are sorted and unique
  int nnzJ = 0;
  for (int i = 0; i < 2 * nvert; i++) {
    nnzJ += m->flexvert_J_rownnz[i];
  }
  EXPECT_EQ(nnzJ, 2 * m->nJfv);
  for (int i = 0; i < 2 * nvert; i++) {
    int row_start = m->flexvert_J_rowadr[i];
    int row_nnz = m->flexvert_J_rownnz[i];
    for (int j = 0; j < row_nnz - 1; j++) {
      EXPECT_LE(m->flexvert_J_colind[row_start + j],
                m->flexvert_J_colind[row_start + j + 1]);
    }
  }

  // Finite-difference check for flexvert_J
  auto fd_check = [&](double tolerance) {
    std::vector<mjtNum> qpos0(m->nq);
    mju_copy(qpos0.data(), d->qpos, m->nq);
    mj_kinematics(m.get(), d.get());
    mj_flex(m.get(), d.get());

    mjtNum eps = MjTol(1e-6, 1e-4);
    int nflexvert = m->flex_vertnum[0];
    std::vector<mjtNum> jac_fd(2 * nflexvert * m->nv);
    std::vector<mjtNum> qpos_backup(m->nq);
    mju_copy(qpos_backup.data(), d->qpos, m->nq);

    for (int i = 0; i < m->nv; ++i) {
      std::vector<mjtNum> qvel(m->nv, 0);
      qvel[i] = 1.0;

      // plus
      mju_copy(d->qpos, qpos_backup.data(), m->nq);
      mj_integratePos(m.get(), d->qpos, qvel.data(), eps);
      mj_kinematics(m.get(), d.get());
      mj_flex(m.get(), d.get());
      std::vector<mjtNum> L_plus(2 * nflexvert);
      for (int e = 0; e < 2 * nflexvert; ++e) {
        L_plus[e] = d->flexvert_length[e];
      }

      // minus
      mju_copy(d->qpos, qpos_backup.data(), m->nq);
      mj_integratePos(m.get(), d->qpos, qvel.data(), -eps);
      mj_kinematics(m.get(), d.get());
      mj_flex(m.get(), d.get());
      std::vector<mjtNum> L_minus(2 * nflexvert);
      for (int e = 0; e < 2 * nflexvert; ++e) {
        L_minus[e] = d->flexvert_length[e];
      }

      for (int e = 0; e < 2 * nflexvert; ++e) {
        jac_fd[e * m->nv + i] = (L_plus[e] - L_minus[e]) / (2 * eps);
      }
    }
    mju_copy(d->qpos, qpos_backup.data(), m->nq);
    mj_kinematics(m.get(), d.get());
    mj_flex(m.get(), d.get());

    // Compare with analytic
    std::vector<mjtNum> jac_analytic(2 * nflexvert * m->nv);
    mju_zero(jac_analytic.data(), 2 * nflexvert * m->nv);
    for (int e = 0; e < 2 * nflexvert; ++e) {
      int row_start = m->flexvert_J_rowadr[e];
      int row_nnz = m->flexvert_J_rownnz[e];
      for (int i = 0; i < row_nnz; ++i) {
        jac_analytic[e * m->nv + m->flexvert_J_colind[row_start + i]] =
            d->flexvert_J[row_start + i];
      }
    }
    EXPECT_THAT(jac_analytic, Not(Each(MjNear(0.0, 1e-7, 1e-4))));
    EXPECT_THAT(jac_analytic, Pointwise(MjNear(tolerance, 1e-1), jac_fd));

    mju_copy(d->qpos, qpos0.data(), m->nq);
    mj_kinematics(m.get(), d.get());
    mj_flex(m.get(), d.get());
  };

  fd_check(MjTol(5e-5, 5e-2));

  // Set qpos to put flex in scale=2 configuration.
  for (int i = 0; i < nvert; i++) {
    d->qpos[3 * i + 0] = d->xpos[3 * (i + 1) + 0];
    d->qpos[3 * i + 1] = d->xpos[3 * (i + 1) + 1];
    d->qpos[3 * i + 2] = d->xpos[3 * (i + 1) + 2];
  }
  mj_fwdKinematics(m.get(), d.get());

  // Get mass scaling factor
  mjtNum scale = 1.0;
  int b = m->flex_vertbodyid[0];
  if (b >= 0 && m->body_mass[b] > mjMINVAL) {
    scale = mju_sqrt(m->body_mass[b]);
  }

  // Check invariants for scale=2
  // F should be [2, 2]. C = F'F = 4I.
  // Strain E = C - I = 3I.
  // Invariant 0: Trace(E) = 3 + 3 = 6
  // Invariant 1: Det(C) - 1 = 4 * 4 - 1 = 15
  // Note: constraints are now scaled by sqrt(mass)
  for (int i = 0; i < nvert; i++) {
    EXPECT_NEAR(d->flexvert_length[2 * i + 0], 6.0 * scale, MjTol(1e-5, 5e-4));
    EXPECT_NEAR(d->flexvert_length[2 * i + 1], 15.0 * scale, MjTol(1e-5, 5e-4));
  }

  // Perturb z-positions so configuration is not flat
  for (int i = 0; i < nvert; i++) {
    d->qpos[3 * i + 2] += 0.01 * (i % 2 ? 1 : -1);
  }
  fd_check(5e-5);
}

// Test failure case for flexvert_J sparsity with skipped flexes
TEST_F(CoreSmoothTest, FlexvertJSparsitySkippedFlex) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="sparse"/>
    <worldbody>
      <body name="body0">
        <joint type="free"/>
        <geom type="sphere" size="0.1"/>
        <flexcomp name="f0" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" radius="0.01" dim="2">
           <edge damping="1"/>
        </flexcomp>
      </body>
      <body name="body1" pos="1 0 0">
        <joint type="free"/>
        <geom type="sphere" size="0.1"/>
        <!-- This flex is rigid, so it will be skipped in flexvert_J computation -->
        <flexcomp name="f1" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" radius="0.01" dim="2" rigid="true"/>
      </body>
      <body name="body2" pos="2 0 0">
        <joint type="free"/>
        <geom type="sphere" size="0.1"/>
        <flexcomp name="f2" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" radius="0.01" dim="2">
           <edge damping="1"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // Forward dynamics to compute Jacobians
  mj_forward(model.get(), data.get());

  // Check sparsity overlap
  // Flex 0 starts at row 0
  // Flex 1 is skipped
  // Flex 2 should start after Flex 0's rows
  // If the bug exists, Flex 2's rows might start at 0, overwriting Flex 0

  int f0_vert_start = model->flex_vertadr[0];
  int f0_vert_num = model->flex_vertnum[0];
  int f2_vert_start = model->flex_vertadr[2];

  // Check last row of flex 0
  int f0_last_row = 2 * (f0_vert_start + f0_vert_num - 1) + 1;
  int f0_end_adr = model->flexvert_J_rowadr[f0_last_row] +
                   model->flexvert_J_rownnz[f0_last_row];

  // Check first row of flex 2
  int f2_first_row = 2 * (f2_vert_start);
  int f2_start_adr = model->flexvert_J_rowadr[f2_first_row];

  // Verify that Flex 2 starts AFTER Flex 0 ends
  EXPECT_GE(f2_start_adr, f0_end_adr)
      << "Flex 2 Jacobian overwrites Flex 0 Jacobian due to skipped Flex 1";
}

// Test stability of flexvert constraint under different integrator/solver
// configurations
TEST_F(CoreSmoothTest, FlexVertStability) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="sparse"/>
    <worldbody>
      <geom name="floor" type="plane" size="0 0 .1"/>
      <flexcomp name="flex" type="grid" count="10 10 1" spacing="0.05 0.05 0.05" radius="0.01" dim="2" mass="1" pos="0 0 1">
           <edge equality="vert" damping="0.1"/>
           <contact solref="0.003"/>
           <elasticity young="3e5" poisson="0" thickness="8e-3" elastic2d="bend"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  struct TestCase {
    mjtIntegrator integrator;
    mjtSolver solver;
    mjtNum tolerance;
    bool expect_stable;
  };

  std::vector<TestCase> cases = {
      // Explicit integration with Newton solver should be stable
      {mjINT_RK4, mjSOL_NEWTON, 1e-6, true},
      // ImplicitFast with CG solver should now be STABLE with mass weighting
      {mjINT_IMPLICITFAST, mjSOL_CG, 1e-6, true},
      // ImplicitFast with Newton solver should be stable
      {mjINT_IMPLICITFAST, mjSOL_NEWTON, 1e-6, true},
  };

  for (const auto& test_case : cases) {
    char error[1024];
    mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
    ASSERT_THAT(spec, NotNull()) << error;

    spec->option.integrator = test_case.integrator;
    spec->option.solver = test_case.solver;
    spec->option.tolerance = test_case.tolerance;

    mjModel* model = mj_compile(spec, nullptr);
    ASSERT_THAT(model, NotNull())
        << error << " (Case: " << test_case.integrator << ", "
        << test_case.solver << ", " << test_case.tolerance << ")";
    mjData* data = mj_makeData(model);

    // Run simulation
    bool exploded = false;
    for (int i = 0; i < 100; ++i) {
      mj_step(model, data);

      // Check for explosion
      for (int j = 0; j < model->nv; ++j) {
        if (mju_abs(data->qvel[j]) > 1000.0) {
          exploded = true;
          break;
        }
      }
      if (exploded) break;
    }

    if (test_case.expect_stable) {
      EXPECT_FALSE(exploded) << "Expected stable simulation for "
                             << test_case.integrator << "/" << test_case.solver;
    } else {
      EXPECT_TRUE(exploded) << "Expected explosion for " << test_case.integrator
                            << "/" << test_case.solver
                            << ". If this passes, the reproduction is no "
                               "longer valid (which is good, but unexpected).";
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(spec);
  }
}

}  // namespace
}  // namespace mujoco
