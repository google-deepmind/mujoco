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

// Tests for engine/engine_metric.c.

#include "src/engine/engine_metric.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_util.h"
#include "src/engine/engine_derivative.h"
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_sparse.h"
#include "test/engine/flex_test_fixture.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::vector;
using ::testing::NotNull;
using ::testing::Pointwise;
using MetricTest = MujocoTest;

TEST_P(FlexFrameTest, ConstantBendingFactor) {
  ASSERT_NO_FATAL_FAILURE(LoadPatch("bend"));
  ExpectBendingFactorInverse();
}

TEST_P(FlexFrameTest, ConstantBendingFactorAfterFrameChange) {
  ASSERT_NO_FATAL_FAILURE(LoadPatch("bend"));
  int body = mj_name2id(model.get(), mjOBJ_BODY, "v1");
  const mjtNum rotation[4] = {0.5, 0.5, 0.5, 0.5};
  mju_copy4(model->body_quat + 4 * body, rotation);
  mj_setConst(model.get(), data.get());
  mj_forward(model.get(), data.get());
  ExpectBendingFactorInverse();
}

INSTANTIATE_TEST_SUITE_P(Frames, FlexFrameTest, FlexFrameCases(),
                         FlexFrameCaseName);

// twist-twist block of the metric (1D DER rods with per-edge twist joints): the
// torsion tridiagonal 2 kt plus the bending stiffness of each twist angle
// (mj_rodChain), against central differences of the spring force in the
// twist angles. Exact at rest (straight, helix, vee and a curved box ribbon,
// whose material curvatures give the twist angles bending stiffness); away from
// rest (a bent box ribbon) the off-diagonals stay exact and the diagonal only
// gains where a hinge side's second derivative is negative and clamped to 0
TEST_F(MetricTest, FlexRodTwistHessian) {
  static const char* const kBoxArc = R"(
  <mujoco>
    <option integrator="discrete" gravity="0 0 0"/>
    <worldbody>
      <flexcomp name="arc" type="direct" dim="1" radius="0.005" mass="0.1"
                element="0 1  1 2  2 3  3 4  4 5"
                point="0.2 0 0  0.190211 0.0618034 0  0.161803 0.117557 0
                       0.117557 0.161803 0  0.0618034 0.190211 0  0 0.2 0">
        <pin id="0"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e7" shear="4e6" section="box"
                    sectionsize="0.012 0.003"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  struct Case {
    const char* xml;
    bool bend;  // deform the vertices away from rest
  };
  for (Case c : {Case{kRodStraightPinned, false}, Case{kRodHelixPinned, false},
                 Case{kRodVeePinned, false}, Case{kBoxArc, false},
                 Case{kBoxArc, true}}) {
    char error[1024];
    MjModelPtr model = LoadModelFromString(c.xml, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    SCOPED_TRACE(std::string(model->names + model->name_flexadr[0]) +
                 (c.bend ? " bent" : " rest"));
    model->flex_damping[0] = 0;  // the stiffness part only: B = h^2 K
    const mjtNum h = model->opt.timestep;
    int nv = model->nv;
    MjDataPtr data = MakeData(model);
    if (c.bend) {
      for (int j = 0; j < model->njnt; j++) {
        if (model->jnt_type[j] == mjJNT_SLIDE) {
          int qadr = model->jnt_qposadr[j];
          data->qpos[qadr] = 0.004 * std::sin(1.3 * qadr + 0.4);
        }
      }
    }
    mj_forward(model.get(), data.get());
    for (int f = 0; f < model->nflex; f++) {
      if (model->flex_plasticadr[f] >= 0) {
        mj_rodChain(model.get(), data.get(), f, nullptr, nullptr, nullptr,
                    nullptr, nullptr, nullptr,
                    data->plastic + model->flex_plasticadr[f]);
      }
    }
    mj_forward(model.get(), data.get());

    int nedge =
        static_cast<int>(model->flex_bending[model->flex_bendingadr[0]]) + 1;
    vector<int> dofs;
    for (int e = 0; e < nedge; e++) {
      int jnt = mj_rodChainTwistJnt(model.get(), 0, e);
      ASSERT_GE(jnt, 0) << "edge " << e;
      dofs.push_back(model->jnt_dofadr[jnt]);
    }
    int n = dofs.size();

    // metric columns and central differences of the spring force
    mjtNum eps = MjEps(1e-6, 1e-3), scale = 0, max_err = 0, kbend = 0;
    vector<mjtNum> vec(nv), res(nv);
    for (int j = 0; j < n; j++) {
      mju_zero(vec.data(), nv);
      mju_zero(res.data(), nv);
      vec[dofs[j]] = 1;
      mj_effMulAdd(model.get(), data.get(), res.data(), vec.data(), 0);
      int qadr = model->jnt_qposadr[model->dof_jntid[dofs[j]]];
      mjData* plus = mj_copyData(NULL, model.get(), data.get());
      mjData* minus = mj_copyData(NULL, model.get(), data.get());
      plus->qpos[qadr] += eps;
      minus->qpos[qadr] -= eps;
      mj_forward(model.get(), plus);
      mj_forward(model.get(), minus);
      for (int i = 0; i < n; i++) {
        mjtNum fd =
            -(plus->qfrc_spring[dofs[i]] - minus->qfrc_spring[dofs[i]]) /
            (2 * eps);
        mjtNum ana = res[dofs[i]] / (h * h);
        mjtNum err = ana - fd;
        if (c.bend && i == j) {
          err = mju_min(0, err);  // PSD clamp: the diagonal may only gain
        }
        max_err = mju_max(max_err, mju_abs(err));
        scale = mju_max(scale, mju_abs(fd));
      }
      kbend = mju_max(kbend, data->efm_ck[dofs[j]] / h);
      mj_deleteData(plus);
      mj_deleteData(minus);
    }
    EXPECT_GT(scale, 0);
    EXPECT_LT(max_err, MjTol(1e-6, 1e-2) * scale);

    // curved and bent rods exercise the bending stiffness of the twist angles
    bool straight = (c.xml == kRodStraightPinned) && !c.bend;
    if (straight) {
      EXPECT_LT(kbend, MjTol(1e-9, 1e-4) * scale);
    } else {
      EXPECT_GT(kbend, 1e-3 * scale);
    }
  }
}

// verify passive contact metric and velocity shift on interpolated flexes
// (trilinear volumetric, trilinear shell elastic2d="bend", quadratic, and with
// pinned cage nodes) against central finite differences of qfrc_spring
TEST_F(MetricTest, FlexInterpPassiveContactDerivatives) {
  struct Case {
    const char* name;
    const char* dof;
    const char* elasticity;
    const char* pin;
  };
  const Case cases[] = {
      {"trilinear_vol", "trilinear",
       R"(<elasticity young="1e3" poisson="0.3" damping="0"/>)", ""},
      {"trilinear_shell_bend", "trilinear",
       R"(<elasticity young="1e3" poisson="0.3" damping="0" thickness="0.01" elastic2d="bend"/>)",
       ""},
      {"quadratic_vol", "quadratic",
       R"(<elasticity young="1e3" poisson="0.3" damping="0"/>)", ""},
      {"trilinear_pinned", "trilinear",
       R"(<elasticity young="1e3" poisson="0.3" damping="0"/>)",
       R"(<pin id="1 3 5 7"/>)"},
  };

  for (const Case& tc : cases) {
    SCOPED_TRACE(tc.name);
    std::string xml = std::string(R"(
    <mujoco>
      <option integrator="discrete" gravity="0 0 0" timestep="0.002"/>
      <worldbody>
        <geom name="floor" type="plane" size="2 2 0.1"/>
        <flexcomp name="box" type="grid" count="3 3 3" cellcount="1 1 1"
                  spacing="0.1 0.1 0.1" radius="0.01" dim="3" mass="1.0"
                  pos="0 0 0.09" dof=")") +
                      tc.dof + R"(">
          <contact selfcollide="none" passive="true"/>
          )" + tc.elasticity +
                      "\n" + tc.pin + R"(
        </flexcomp>
      </worldbody>
    </mujoco>
    )";

    char error[1024] = {0};
    MjModelPtr m = LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << error;
    MjDataPtr d = MakeData(m);
    mj_forward(m.get(), d.get());

    ASSERT_GT(d->nefmcon, 0);
    ASSERT_EQ(d->nefc, 0);

    // At rest qpos = 0, elastic force is zero so qfrc_spring equals the
    // contact force interpolated onto cage nodes via mj_contactBodyWeight.
    std::vector<mjtNum> expected_qfrc(m->nv, 0);
    for (int i = 0; i < d->ncon; i++) {
      const mjContact* con = d->contact + i;
      if (con->exclude != 4 || con->dist >= 0) continue;
      mjtNum k = mjd_flexContactStiffness(m.get(), d.get(), con, nullptr);
      ASSERT_GT(k, 0);
      mjtNum fn = -k * con->dist;
      mjtNum force[3] = {fn * con->frame[0], fn * con->frame[1],
                         fn * con->frame[2]};
      for (int side = 0; side < 2; side++) {
        if (con->flex[side] < 0) continue;
        int body[128];
        mjtNum weight[128];
        int nb =
            mj_contactBodyWeight(m.get(), d.get(), con, side, body, weight);
        mjtNum sign = (side == 0) ? -1.0 : 1.0;
        for (int j = 0; j < nb; j++) {
          int b = m->body_weldid[body[j]];
          if (m->body_dofnum[b] == 3) {
            int da = m->body_dofadr[b];
            expected_qfrc[da + 0] += sign * weight[j] * force[0];
            expected_qfrc[da + 1] += sign * weight[j] * force[1];
            expected_qfrc[da + 2] += sign * weight[j] * force[2];
          }
        }
      }
    }
    for (int i = 0; i < m->nv; i++) {
      EXPECT_NEAR(d->qfrc_spring[i], expected_qfrc[i],
                  MjTol(1e-9, 1e-3) * mju_max(1.0, mju_abs(expected_qfrc[i])))
          << "dof " << i;
    }

    int nv = m->nv;
    mjtNum h = m->opt.timestep;
    std::vector<mjtNum> vec(nv), res_with_con(nv, 0), res_no_con(nv, 0);
    for (int i = 0; i < nv; i++) {
      vec[i] = mju_Halton(i, 3) - 0.5;
    }
    mj_effMulAdd(m.get(), d.get(), res_with_con.data(), vec.data(),
                 /*flg_contact=*/1);
    mj_effMulAdd(m.get(), d.get(), res_no_con.data(), vec.data(),
                 /*flg_contact=*/0);

    // Isolate the contact-only spring force by comparing against a copy with
    // contact disabled, perturbed by +/- eps*vec (small enough that active
    // contacts remain penetrating with constant plane normal).
    mjtNum eps = MjEps(1e-7, 1e-5);
    MjDataPtr d_plus(mj_copyData(nullptr, m.get(), d.get()));
    MjDataPtr d_minus(mj_copyData(nullptr, m.get(), d.get()));
    mju_addToScl(d_plus->qpos, vec.data(), eps, nv);
    mju_addToScl(d_minus->qpos, vec.data(), -eps, nv);
    mj_forward(m.get(), d_plus.get());
    mj_forward(m.get(), d_minus.get());

    m->opt.disableflags |= mjDSBL_CONTACT;
    MjDataPtr d_plus_nocon(mj_copyData(nullptr, m.get(), d_plus.get()));
    MjDataPtr d_minus_nocon(mj_copyData(nullptr, m.get(), d_minus.get()));
    mj_forward(m.get(), d_plus_nocon.get());
    mj_forward(m.get(), d_minus_nocon.get());
    m->opt.disableflags &= ~mjDSBL_CONTACT;

    mjtNum max_err = 0, scale = 0;
    for (int i = 0; i < nv; i++) {
      mjtNum f_plus = d_plus->qfrc_spring[i] - d_plus_nocon->qfrc_spring[i];
      mjtNum f_minus = d_minus->qfrc_spring[i] - d_minus_nocon->qfrc_spring[i];
      mjtNum fd_h2K = -h * h * (f_plus - f_minus) / (2 * eps);
      mjtNum analytic_h2K = res_with_con[i] - res_no_con[i];
      max_err = mju_max(max_err, mju_abs(analytic_h2K - fd_h2K));
      scale = mju_max(scale, mju_abs(fd_h2K));
    }
    EXPECT_GT(scale, 1e-2);
    EXPECT_LT(max_err, MjTol(1e-6, 1e-3) * scale);

    // Also verify velocity shift d->efm_c: setting qvel = vec must shift efm_c
    // by -(1/h) * (h^2 K_total * vec) = -h * K_total * vec.
    mju_copy(d->qvel, vec.data(), nv);
    mj_forward(m.get(), d.get());
    for (int i = 0; i < nv; i++) {
      EXPECT_NEAR(
          d->efm_c[i], -res_with_con[i] / h,
          MjTol(1e-9, 1e-3) * mju_max(1.0, mju_abs(res_with_con[i] / h)))
          << "shift mismatch at DOF " << i;
    }
  }
}

TEST_P(FlexAttachmentTest, MatrixFreeMetric) { ExpectMetric(false); }
TEST_P(FlexAttachmentTest, MixedAssembledAndMatrixFreeMetric) {
  ExpectMetric(true);
}

TEST_P(FlexAttachmentTest, EffectiveSolveResidual) {
  ASSERT_NO_FATAL_FAILURE(Load("both", true));
  Deform();
  auto rhs = Direction(3);
  std::vector<mjtNum> solution(model->nv), product(model->nv);
  mj_effSolve(model.get(), data.get(), solution.data(), rhs.data());
  mj_mulM(model.get(), data.get(), product.data(), solution.data());
  mj_effMulAdd(model.get(), data.get(), product.data(), solution.data(), 1);
  EXPECT_THAT(product, Pointwise(MjNear(2e-8, 2e-5), rhs));
}

TEST_P(FlexAttachmentTest, MetricIsSymmetricAndPositive) {
  Deform();
  auto u = Direction(3), v = Direction(5);
  std::vector<mjtNum> Ku(model->nv, 0), Kv(model->nv, 0);
  mj_effMulAdd(model.get(), data.get(), Ku.data(), u.data(), 1);
  mj_effMulAdd(model.get(), data.get(), Kv.data(), v.data(), 1);
  EXPECT_THAT(mju_dot(u.data(), Kv.data(), model->nv),
              MjNear(mju_dot(v.data(), Ku.data(), model->nv), 1e-12, 1e-7));
  EXPECT_GE(mju_dot(u.data(), Ku.data(), model->nv), -MjTol(1e-12, 1e-7));
}

TEST_P(FlexAttachmentTest, SupportsNewtonForDiscreteIntegrator) {
  for (bool mixed : {false, true}) {
    std::string xml = Xml(false, "both", mixed);
    const std::string solver_str = "solver=\"CG\"";
    xml.replace(xml.find(solver_str), solver_str.size(),
                "solver=\"CG\" tolerance=\"1e-14\"");
    const std::string wb_str = "<worldbody>";
    xml.replace(xml.find(wb_str), wb_str.size(),
                "<worldbody><geom type='plane' pos='0 0 -.05' size='2 2 .1'/>");
    const std::string nocontact = "contype=\"0\" conaffinity=\"0\" ";
    for (size_t pos = 0;
         (pos = xml.find(nocontact, pos)) != std::string::npos;) {
      xml.erase(pos, nocontact.size());
    }

    char error[1024];
    auto m_test = LoadModelFromString(xml, error, sizeof(error));
    ASSERT_THAT(m_test.get(), NotNull()) << error;
    auto d_test = MakeData(m_test);
    int nv = m_test->nv;

    std::vector<mjtNum> dir(nv);
    for (int i = 0; i < nv; i++) dir[i] = mju_Halton(i, 2) - .5;
    mj_integratePos(m_test.get(), d_test->qpos, dir.data(), .05);
    mju_copy(d_test->qvel, dir.data(), nv);

    m_test->opt.solver = mjSOL_CG;
    mj_forward(m_test.get(), d_test.get());
    std::vector<mjtNum> qacc_cg(d_test->qacc, d_test->qacc + nv);

    m_test->opt.solver = mjSOL_NEWTON;
    mj_forward(m_test.get(), d_test.get());
    EXPECT_GT(d_test->nefmK, 0);
    EXPECT_EQ(d_test->efm_active, 2);

    // the carrier's joint dofs have rows: the articulated pull-back is in the
    // CSR
    int carrier = mj_name2id(m_test.get(), mjOBJ_BODY, "carrier");
    EXPECT_GT(d_test->efm_K_rownnz[m_test->body_dofadr[carrier]], 0);

    std::vector<mjtNum> rhs(nv), sol(nv), prod(nv);
    for (int i = 0; i < nv; i++) rhs[i] = mju_Halton(i, 3) - .5;
    mj_effSolve(m_test.get(), d_test.get(), sol.data(), rhs.data());
    mj_mulM(m_test.get(), d_test.get(), prod.data(), sol.data());
    mj_effMulAdd(m_test.get(), d_test.get(), prod.data(), sol.data(), 1);
    EXPECT_THAT(prod, Pointwise(MjNear(2e-8, 2e-5), rhs));

    mjtNum scale = mju_norm(qacc_cg.data(), nv) / mju_sqrt((mjtNum)nv) + 1;
    EXPECT_THAT(AsVector(d_test->qacc, nv),
                Pointwise(MjNear(1e-12 * scale, 1e-4 * scale), qacc_cg));
  }
}

INSTANTIATE_TEST_SUITE_P(Attachments, FlexAttachmentTest, FlexAttachmentCases(),
                         FlexAttachmentCaseName);

// an interpolated node on a jointless child of a moving body moves with that
// body: it is not a fixed node, so the stiffness must reach the body's dofs
// under both solvers
TEST_F(MetricTest, InterpNodeOnWeldedChild) {
  static const char* const kXml = R"(
  <mujoco>
    <option integrator="discrete" solver="CG" tolerance="1e-14" iterations="2000"/>
    <worldbody>
      <body name="arm">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size=".02" pos="-.1 0 0" mass=".3"/>
        <body name="welded"/>
      </body>
      <flexcomp name="pad" type="grid" count="3 3 3" spacing=".05 .05 .05"
                pos=".1 0 0" radius=".005" dim="3" mass=".5" dof="trilinear"
                cellcount="1 1 1">
        <pin id="0"/>
        <contact contype="0" conaffinity="0" selfcollide="none"/>
        <elasticity young="1e4" poisson=".2" damping=".1"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  // move the pinned node from the world to the welded child, whose frame is the
  // world frame at qpos0, so its local coordinates are unchanged
  mjSpec* spec = mj_parseXMLString(kXml, nullptr, nullptr, 0);
  ASSERT_THAT(spec, NotNull());
  mjsFlex* flex = mjs_asFlex(mjs_findElement(spec, mjOBJ_FLEX, "pad"));
  ASSERT_THAT(flex, NotNull());
  ASSERT_EQ((*flex->nodebody)[0], "world");
  (*flex->nodebody)[0] = "welded";
  MjModelPtr model(mj_compile(spec, nullptr));
  mj_deleteSpec(spec);
  ASSERT_THAT(model.get(), NotNull());
  int welded = mj_name2id(model.get(), mjOBJ_BODY, "welded");
  ASSERT_EQ(model->flex_nodebodyid[model->flex_nodeadr[0]], welded);
  EXPECT_EQ(mjd_flexInterpAssemblable(model.get()), 0);

  int nv = model->nv;
  int hinge = model->jnt_dofadr[mj_name2id(model.get(), mjOBJ_JOINT, "hinge")];
  MjDataPtr data = MakeData(model);
  std::vector<mjtNum> dir(nv);
  for (int i = 0; i < nv; i++) dir[i] = mju_Halton(i, 2) - .5;
  mj_integratePos(model.get(), data->qpos, dir.data(), .02);
  mju_copy(data->qvel, dir.data(), nv);

  // reference: the matrix-free interp operator, which follows the node's body
  // chain
  mj_forward(model.get(), data.get());
  std::vector<mjtNum> vec(nv), expected(nv, 0);
  for (int i = 0; i < nv; i++) vec[i] = mju_Halton(i, 3) - .5;
  mjtNum h = model->opt.timestep;
  mjd_flexInterp_mul(model.get(), data.get(), expected.data(), vec.data(),
                     -h * h, -h, data->flexelem_krot);
  EXPECT_NE(expected[hinge], 0);

  // CG applies the operator matrix-free, Newton assembles the general CSR
  for (mjtSolver solver : {mjSOL_CG, mjSOL_NEWTON}) {
    model->opt.solver = solver;
    mj_forward(model.get(), data.get());
    std::vector<mjtNum> actual(nv, 0);
    mj_effMulAdd(model.get(), data.get(), actual.data(), vec.data(), 1);
    mjtNum scale = mju_norm(expected.data(), nv) / mju_sqrt((mjtNum)nv) + 1;
    EXPECT_THAT(actual,
                Pointwise(MjNear(1e-12 * scale, 1e-5 * scale), expected))
        << "solver " << solver;
    if (solver == mjSOL_NEWTON) {
      EXPECT_EQ(data->efm_active, 2);
      EXPECT_GT(data->efm_K_rownnz[hinge], 0);
    }
  }
}

TEST_F(MetricTest, EffSolve) {
  // relative residual of (M+K)x - b after mj_effSolve
  auto solve_residual = [](const mjModel* m, mjData* d) {
    int nv = m->nv;
    std::vector<mjtNum> b(nv), x(nv), r(nv);
    for (int i = 0; i < nv; i++) {
      b[i] = mju_Halton(i, 3) - 0.5;
    }
    mj_effSolve(m, d, x.data(), b.data());
    mju_mulSymVecSparse(r.data(), d->M, x.data(), nv, m->M_rownnz, m->M_rowadr,
                        m->M_colind);
    mj_effMulAdd(m, d, r.data(), x.data(), /*flg_contact=*/1);
    mju_subFrom(r.data(), b.data(), nv);
    return mju_norm(r.data(), nv) / mju_norm(b.data(), nv);
  };

  // stretch + bending cloth on world: per-step factor, exact
  static const char* const kXmlBoth = R"(
  <mujoco>
    <option solver="CG" integrator="discrete"/>
    <worldbody>
      <flexcomp name="cloth" type="grid" count="6 6 1" spacing="0.05 0.05 0.05"
                radius=".005" dim="2" mass="0.5" pos="0 0 1" dof="full">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e3" poisson="0.2" damping="0.1" elastic2d="both" thickness="0.01"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kXmlBoth, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GE(data->efm_active, 1);
  EXPECT_GT(data->nefmK, 0);
  EXPECT_GT(data->nefmdof, 0);
  EXPECT_LT(solve_residual(model.get(), data.get()), MjTol(1e-8, 1e-4));

  // bending-only cloth: no CSR or per-step factor, constant factor covers,
  // exact
  static const char* const kXmlBend = R"(
  <mujoco>
    <option solver="CG" integrator="discrete"/>
    <worldbody>
      <flexcomp name="cloth" type="grid" count="6 6 1" spacing="0.05 0.05 0.05"
                radius=".005" dim="2" mass="0.5" pos="0 0 1" dof="full">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e3" poisson="0.2" damping="0.1" elastic2d="bend" thickness="0.01"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  model = LoadModelFromString(kXmlBend, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GE(data->efm_active, 1);
  EXPECT_EQ(data->nefmK, 0);
  EXPECT_EQ(data->nefmdof, 0);
  EXPECT_GT(model->nefm0dof, 0);
  EXPECT_LT(solve_residual(model.get(), data.get()), MjTol(1e-8, 1e-4));

  // A cloth under a jointed parent uses the matrix-free attachment mapping;
  // the iterative solve must still meet its tolerance.
  static const char* const kXmlMoving = R"(
  <mujoco>
    <option solver="CG" integrator="discrete"/>
    <worldbody>
      <body name="base" pos="0 0 1">
        <joint type="slide" axis="0 0 1"/>
        <geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/>
        <flexcomp name="cloth" type="grid" count="6 6 1" spacing="0.05 0.05 0.05"
                  radius=".005" dim="2" mass="0.5" pos="0 0 0" dof="full">
          <contact selfcollide="none" contype="0" conaffinity="0"/>
          <elasticity young="1e3" poisson="0.2" damping="0.1" elastic2d="stretch" thickness="0.01"/>
          <pin id="0"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";
  model = LoadModelFromString(kXmlMoving, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GE(data->efm_active, 1);
  EXPECT_LT(solve_residual(model.get(), data.get()), 1e-4);
}

// Verify mj_effSolve's Eisenstat SGS path (and articulated fallback) on 3D
// flexes against an exact dense Cholesky solve of (M + B) x = b, covering
// uncovered trees, fluid drag, in-place aliasing, non-CSR rank-1 couplings
// (tendon, actuator, passive flex contact), and sleeping trees.
TEST_F(MetricTest, EffSolve3DFlexExact) {
  auto verify_exact = [](const mjModel* m, mjData* d) {
    int nv = m->nv;
    ASSERT_GT(nv, 0);
    ASSERT_GE(d->efm_active, 1);
    ASSERT_GT(d->nefmK, 0);

    // assemble dense A = M + B via mj_effMulAdd
    vector<mjtNum> A(nv * nv, 0);
    vector<mjtNum> ej(nv, 0), col(nv, 0);
    for (int j = 0; j < nv; j++) {
      ej[j] = 1;
      mju_mulSymVecSparse(col.data(), d->M, ej.data(), nv, m->M_rownnz,
                          m->M_rowadr, m->M_colind);
      mj_effMulAdd(m, d, col.data(), ej.data(), /*flg_contact=*/1);
      for (int i = 0; i < nv; i++) A[i * nv + j] = col[i];
      ej[j] = 0;
    }

    // dense Cholesky reference solve A * x_ref = b
    vector<mjtNum> L = A;
    ASSERT_EQ(mju_cholFactor(L.data(), nv, 0), nv);
    vector<mjtNum> b(nv), x_ref(nv), x(nv), Ax(nv);
    for (int i = 0; i < nv; i++) {
      b[i] = 1.0 + 0.1 * ((i * 7) % 13) - 0.5;
    }
    mju_cholSolve(x_ref.data(), L.data(), b.data(), nv);

    mj_effSolve(m, d, x.data(), b.data());
    EXPECT_EQ(d->warning[mjWARN_INERTIA].number, 0);

    // verify true residual and agreement with dense Cholesky
    mju_mulSymVecSparse(Ax.data(), d->M, x.data(), nv, m->M_rownnz, m->M_rowadr,
                        m->M_colind);
    mj_effMulAdd(m, d, Ax.data(), x.data(), /*flg_contact=*/1);
    mjtNum r2 = 0, b2 = 0, e2 = 0, xr2 = 0;
    for (int i = 0; i < nv; i++) {
      mjtNum ri = b[i] - Ax[i];
      mjtNum ei = x[i] - x_ref[i];
      r2 += ri * ri;
      b2 += b[i] * b[i];
      e2 += ei * ei;
      xr2 += x_ref[i] * x_ref[i];
    }
    EXPECT_LT(mju_sqrt(r2 / b2), MjTol(1e-7, 5e-4));
    EXPECT_LT(mju_sqrt(e2 / xr2), MjTol(1e-6, 1e-3));

    // in-place aliasing x == b must produce the same result
    vector<mjtNum> x_inplace = b;
    mj_effSolve(m, d, x_inplace.data(), x_inplace.data());
    mjtNum d2 = 0;
    for (int i = 0; i < nv; i++) {
      mjtNum di = x_inplace[i] - x[i];
      d2 += di * di;
    }
    EXPECT_LT(mju_sqrt(d2 / xr2), MjTol(1e-12, 1e-6));
  };

  char error[1024];

  // Case 1: simple 3D FEM + uncovered articulated hinge + fluid drag.
  static const char* const kXmlSimple = R"(
  <mujoco>
    <option solver="CG" integrator="discrete" tolerance="1e-10" iterations="200"
            viscosity="0.5" density="1.2"/>
    <worldbody>
      <body name="pend" pos="0.5 0 0.5">
        <joint type="hinge" axis="0 1 0" stiffness="5" damping="0.2"/>
        <geom type="capsule" size="0.02 0.1" mass="0.3" contype="0" conaffinity="0"/>
        <body pos="0 0 -0.2">
          <joint type="hinge" axis="1 0 0" damping="0.1"/>
          <geom type="sphere" size="0.03" mass="0.2" contype="0" conaffinity="0"/>
        </body>
      </body>
      <flexcomp name="block" type="box" count="3 3 2" spacing="0.04 0.04 0.04"
                radius="0.005" dim="3" mass="0.3" pos="0 0 0.5" dof="full">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="5e3" poisson="0.25" damping="0.05"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  {
    auto model = LoadModelFromString(kXmlSimple, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    auto data = MakeData(model);
    data->qvel[0] = 0.7;
    data->qvel[1] = -0.4;
    mj_forward(model.get(), data.get());
    EXPECT_EQ(data->nefmL, 0);
    verify_exact(model.get(), data.get());
  }

  // Case 2: simple 3D flex with passive contact, free body, tendon, actuator.
  static const char* const kXmlCouplings = R"(
  <mujoco>
    <option solver="CG" integrator="discrete" tolerance="1e-10" iterations="200"/>
    <worldbody>
      <geom type="plane" size="1 1 0.1"/>
      <body name="arm" pos="0.4 0 0.3">
        <joint name="h1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.03" mass="0.2" contype="0" conaffinity="0"/>
        <body pos="0.1 0 0">
          <joint name="h2" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.03" mass="0.2" contype="0" conaffinity="0"/>
        </body>
      </body>
      <body name="freeb" pos="-0.4 0 0.3">
        <freejoint/>
        <geom type="box" size="0.04 0.04 0.04" mass="0.4" contype="0" conaffinity="0"/>
      </body>
      <flexcomp name="block" type="box" count="3 3 2" spacing="0.04 0.04 0.04"
                radius="0.015" dim="3" mass="0.3" pos="0 0 0.018" dof="full">
        <contact selfcollide="none" passive="true"/>
        <elasticity young="5e3" poisson="0.25" damping="0.05"/>
      </flexcomp>
    </worldbody>
    <tendon>
      <fixed name="t1" stiffness="20" damping="1">
        <joint joint="h1" coef="1"/>
        <joint joint="h2" coef="-0.5"/>
      </fixed>
    </tendon>
    <actuator>
      <position name="act" tendon="t1" kp="30" kv="2"/>
    </actuator>
  </mujoco>
  )";
  {
    auto model = LoadModelFromString(kXmlCouplings, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    auto data = MakeData(model);
    data->qpos[0] = 0.2;
    data->ctrl[0] = 0.1;
    mj_forward(model.get(), data.get());
    ASSERT_GT(data->nefmT, 0);
    ASSERT_GT(data->nefmA, 0);
    ASSERT_GT(data->nefmcon, 0);
    verify_exact(model.get(), data.get());
  }

  // Case 3: articulated 3D flex under Newton (general CSR fallback).
  static const char* const kXmlArticulated = R"(
  <mujoco>
    <option solver="Newton" integrator="discrete" tolerance="1e-10" iterations="200"/>
    <worldbody>
      <body name="base" pos="0 0 0.3">
        <joint type="slide" axis="0 0 1" stiffness="10" damping="0.5"/>
        <geom type="sphere" size="0.01" mass="0.5" contype="0" conaffinity="0"/>
        <flexcomp name="block" type="box" count="3 3 2" spacing="0.04 0.04 0.04"
                  radius="0.005" dim="3" mass="0.3" pos="0 0 0" dof="full">
          <contact selfcollide="none" contype="0" conaffinity="0"/>
          <elasticity young="5e3" poisson="0.25" damping="0.05"/>
          <pin id="0"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";
  {
    auto model = LoadModelFromString(kXmlArticulated, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    auto data = MakeData(model);
    mj_forward(model.get(), data.get());
    verify_exact(model.get(), data.get());
  }

  // Case 4: awake 3D flex + sleeping rigid body (sleep_filter active).
  static const char* const kXmlSleep = R"(
  <mujoco>
    <option solver="CG" integrator="discrete" tolerance="1e-10" iterations="200">
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <body name="sleeper" pos="1 0 0.5" sleep="init">
        <freejoint/>
        <geom type="sphere" size="0.05" mass="0.5" contype="0" conaffinity="0"/>
      </body>
      <flexcomp name="block" type="box" count="3 3 2" spacing="0.04 0.04 0.04"
                radius="0.005" dim="3" mass="0.3" pos="0 0 0.5" dof="full">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="5e3" poisson="0.25" damping="0.05"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  {
    auto model = LoadModelFromString(kXmlSleep, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    auto data = MakeData(model);
    mj_forward(model.get(), data.get());
    ASSERT_LT(data->nv_awake, model->nv);
    verify_exact(model.get(), data.get());
  }

  // Case 5: tilted passive contact on 3D flex under tight tolerance and budget
  // (folding efm_con diagonals into SGS D without off-diagonal contact terms
  // crushes orthogonal-mode eigenvalues and stalls convergence).
  static const char* const kXmlTiltedPassive = R"(
  <mujoco>
    <option timestep="0.01" solver="Newton" integrator="discrete"
            tolerance="1e-12" iterations="100"/>
    <worldbody>
      <geom type="plane" size="1 1 0.1" euler="25 35 0"/>
      <flexcomp name="block" type="grid" count="3 3 2" spacing="0.03 0.03 0.03"
                radius="0.004" dim="3" mass="0.3" pos="0 0 0.02" dof="full">
        <contact selfcollide="none" passive="true"/>
        <elasticity young="1e3" poisson="0.2" damping="0.01"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  {
    auto model = LoadModelFromString(kXmlTiltedPassive, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    auto data = MakeData(model);
    for (int step = 0; step < 5; step++) {
      mj_step(model.get(), data.get());
    }
    EXPECT_EQ(data->warning[mjWARN_INERTIA].number, 0);
    ASSERT_GT(data->nefmcon, 0);
    EXPECT_EQ(data->nefmL, 0);
    verify_exact(model.get(), data.get());
  }
}

// Verify Eisenstat SGS converges substantially faster than qH diagonal PCG on
// a stiff 3D FEM flex within a tight iteration budget.
TEST_F(MetricTest, EffSolve3DFlexConvergence) {
  static const char* const kXmlStiff3D = R"(
  <mujoco>
    <option timestep="0.003" solver="CG" integrator="discrete"
            tolerance="1e-5" iterations="35"/>
    <worldbody>
      <flexcomp name="clay" type="grid" dim="3" count="8 8 4"
                spacing="0.02 0.02 0.025" radius="0.002" mass="1.0"
                pos="0 0 0.1">
        <pin gridrange="0 0 0 7 7 0"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="2e5" poisson="0.2" damping="0.02"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  auto model = LoadModelFromString(kXmlStiff3D, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  auto data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GT(data->nefmK, 0);
  ASSERT_EQ(data->nefmL, 0);

  int nv = model->nv;
  vector<mjtNum> b(nv), x(nv), Ax(nv);
  for (int i = 0; i < nv; i++) {
    b[i] = 1.0 + 0.3 * ((i * 5) % 11) - 1.5;
  }

  // SGS PCG must converge within 35 iterations without warning.
  mj_effSolve(model.get(), data.get(), x.data(), b.data());
  EXPECT_EQ(data->warning[mjWARN_INERTIA].number, 0);
  mju_mulSymVecSparse(Ax.data(), data->M, x.data(), nv, model->M_rownnz,
                      model->M_rowadr, model->M_colind);
  mj_effMulAdd(model.get(), data.get(), Ax.data(), x.data(), /*flg_contact=*/1);
  mjtNum r2 = 0, b2 = 0;
  for (int i = 0; i < nv; i++) {
    mjtNum ri = b[i] - Ax[i];
    r2 += ri * ri;
    b2 += b[i] * b[i];
  }
  EXPECT_LT(mju_sqrt(r2 / b2), MjTol(1e-5, 2e-5));

  // Reference qH-preconditioned PCG (using mj_effPrec) does not converge in 35
  // iterations on the same problem.
  vector<mjtNum> r = b, z(nv), p(nv), Ap(nv);
  mj_effPrec(model.get(), data.get(), z.data(), r.data());
  p = z;
  mjtNum rz = mju_dot(r.data(), z.data(), nv);
  for (int it = 0; it < model->opt.iterations; it++) {
    mju_mulSymVecSparse(Ap.data(), data->M, p.data(), nv, model->M_rownnz,
                        model->M_rowadr, model->M_colind);
    mj_effMulAdd(model.get(), data.get(), Ap.data(), p.data(),
                 /*flg_contact=*/1);
    mjtNum pAp = mju_dot(p.data(), Ap.data(), nv);
    ASSERT_GT(pAp, 0);
    mjtNum alpha = rz / pAp;
    mju_addToScl(r.data(), Ap.data(), -alpha, nv);
    mj_effPrec(model.get(), data.get(), z.data(), r.data());
    mjtNum rznew = mju_dot(r.data(), z.data(), nv);
    mju_addScl(p.data(), z.data(), p.data(), rznew / rz, nv);
    rz = rznew;
  }
  mjtNum qh_relres = mju_sqrt(mju_dot(r.data(), r.data(), nv) / b2);
  EXPECT_GT(qh_relres, 10 * model->opt.tolerance);
}

// A cloth with per-step stretch stiffness, used by the two tests below.
// Independent flex slides with an articulated descendant: the mass matrix
// couples the covered vertex triple to an uncovered hinge in the same kinematic
// tree.
static const char* const kStretchCloth = R"(
<mujoco>
  <option solver="CG" integrator="discrete"/>
  <default>
    <geom type="sphere" size=".01" mass=".1" contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="v0">
      <joint type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 1 0"/>
      <joint type="slide" axis="0 0 1"/>
      <geom/>
      <body pos=".02 0 0">
        <joint axis="0 1 0"/>
        <geom pos=".1 0 0"/>
      </body>
    </body>
    <body name="v1" pos=".1 0 0">
      <joint type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 1 0"/>
      <joint type="slide" axis="0 0 1"/>
      <geom/>
    </body>
    <body name="v2" pos="0 .1 0">
      <joint type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 1 0"/>
      <joint type="slide" axis="0 0 1"/>
      <geom/>
    </body>
    <body name="v3" pos=".1 .1 0">
      <joint type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 1 0"/>
      <joint type="slide" axis="0 0 1"/>
      <geom/>
    </body>
  </worldbody>
  <deformable>
    <flex dim="2" body="v0 v1 v2 v3" element="0 1 2 1 3 2">
      <contact selfcollide="none" contype="0" conaffinity="0"/>
      <elasticity young="1e3" poisson=".2" damping=".1" elastic2d="stretch" thickness=".01"/>
    </flex>
  </deformable>
</mujoco>
)";

// A metric too ill-conditioned for the 3x3 blocks exhausts mj_effSolve's
// iteration budget; it must report that rather than return an under-converged
// qacc_smooth silently. Forced by conditioning rather than by an unreachable
// opt.tolerance, which cannot be expressed in single precision: there the
// residual reaches exactly zero and CG breaks down on a converged solve.
TEST_F(MetricTest, EffSolveCapWarns) {
  static const char* const kStiffCloth = R"(
  <mujoco>
    <option solver="CG" integrator="discrete"/>
    <worldbody>
      <body name="base" pos="0 0 1">
        <joint type="slide" axis="0 0 1"/>
        <geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/>
        <flexcomp name="cloth" type="grid" count="24 24 1" dim="2"
                  spacing="0.02 0.02 0.02" radius=".002" mass="0.02" dof="full">
          <contact selfcollide="none" contype="0" conaffinity="0"/>
          <elasticity young="1e9" poisson="0.45" damping="0" elastic2d="stretch"
                      thickness="0.02"/>
          <pin id="0"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kStiffCloth, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  // Excite in-plane stretch modes; gravity alone translates this flat cloth.
  for (int i = 0; i < model->nv; i++) {
    data->qfrc_applied[i] = mju_Halton(i, 2) - 0.5;
  }

  MockWarningHandler warning_handler;
  // both: the specific cause, then the mjWARN_INERTIA it is reported through
  warning_handler.ExpectWarnings("Flex stiffness is too ill-conditioned");
  warning_handler.ExpectWarnings("Inertia matrix is too close to singular");
  mj_forward(model.get(), data.get());
  testing::Mock::VerifyAndClearExpectations(&warning_handler);
}

// effCholFill includes efm_diag and efm_fluid on covered triples, and
// mj_effShift invalidates a previously factored effChol so mj_forwardSkip
// re-factors with updated velocity-dependent fluid drag.
TEST_F(MetricTest, EffCholDiagFluidAndSkip) {
  static const char* const kFluidClothXml = R"(
  <mujoco>
    <option timestep="0.01" integrator="discrete" solver="CG"
            iterations="2" tolerance="1e-8" viscosity="1" density="10"/>
    <worldbody>
      <flexcomp name="cloth" type="grid" count="4 4 1" spacing="0.05 0.05 1"
                pos="0 0 0.5" radius="0.005" mass="0.1" dim="2">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e3" poisson="0.2" thickness="2e-3"
                    elastic2d="stretch"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kFluidClothXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  auto check_exact = [&](const mjModel* m, mjData* d) {
    int nv = m->nv;
    std::vector<mjtNum> b(nv), x(nv), r(nv);
    for (int i = 0; i < nv; i++) {
      b[i] = mju_Halton(i, 5) - 0.5;
    }
    mj_effSolve(m, d, x.data(), b.data());
    EXPECT_EQ(d->warning[mjWARN_INERTIA].number, 0);
    mju_mulSymVecSparse(r.data(), d->M, x.data(), nv, m->M_rownnz, m->M_rowadr,
                        m->M_colind);
    mj_effMulAdd(m, d, r.data(), x.data(), /*flg_contact=*/1);
    mju_subFrom(r.data(), b.data(), nv);
    EXPECT_LT(mju_norm(r.data(), nv) / mju_norm(b.data(), nv),
              MjTol(1e-8, 1e-4));
  };

  mj_forward(model.get(), data.get());
  ASSERT_THAT(data->efm_diag, NotNull());
  ASSERT_THAT(data->efm_fluid, NotNull());
  EXPECT_GT(mju_norm(data->efm_fluid, model->nC), 0);
  ASSERT_GT(data->nefmL, 0);
  EXPECT_EQ(data->efm_L[0], 1);
  check_exact(model.get(), data.get());

  // Change qvel and re-run from the velocity stage only: fluid drag changes
  // efm_fluid/efm_diag without rebuilding efm_L, so mj_effShift must
  // invalidate the factor.
  for (int i = 0; i < model->nv; i++) {
    data->qvel[i] = 10.0 * (mju_Halton(i, 7) - 0.5);
  }
  mj_forwardSkip(model.get(), data.get(), mjSTAGE_POS, 0);
  EXPECT_EQ(data->efm_L[0], 1);
  check_exact(model.get(), data.get());
}

// Passive flex contacts between covered 2D cloths (and static geoms) are
// factored into effChol with dynamic minimum-degree reordering, so mj_effSolve
// converges in a few iterations without warning even under tight budgets, and
// tight arena limits fall back cleanly without overflow.
TEST_F(MetricTest, EffSolvePassiveFlexContact) {
  static const char* const kDrapeXml = R"(
  <mujoco>
    <option timestep="0.002" integrator="discrete" solver="CG"
            iterations="5" tolerance="1e-8"/>
    <worldbody>
      <geom type="sphere" pos="0 0 0" size="0.08"/>
      <flexcomp name="c0" type="grid" count="5 5 1" spacing="0.04 0.04 1"
                pos="0 0 0.09" radius="0.005" mass="0.05" dim="2">
        <contact selfcollide="auto" passive="true"/>
        <elasticity young="1e4" poisson="0.2" thickness="1e-3"
                    elastic2d="both" damping="1e-4"/>
      </flexcomp>
      <flexcomp name="c1" type="grid" count="5 5 1" spacing="0.04 0.04 1"
                pos="0.005 0 0.105" radius="0.005" mass="0.05" dim="2">
        <contact selfcollide="auto" passive="true"/>
        <elasticity young="1e4" poisson="0.2" thickness="1e-3"
                    elastic2d="both" damping="1e-4"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kDrapeXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  int saw_con = 0;
  for (int s = 0; s < 50; s++) {
    mj_step(model.get(), data.get());
    ASSERT_EQ(data->warning[mjWARN_INERTIA].number, 0) << "step " << s;
    if (data->nefmcon > 0) {
      saw_con++;
    }
  }
  ASSERT_GT(saw_con, 0);
  ASSERT_GT(data->nefmcon, 0);
  ASSERT_GT(data->nefmL, 0);
  EXPECT_EQ(data->efm_L[0], 1);

  int nv = model->nv;
  std::vector<mjtNum> b(nv), x(nv), r(nv);
  for (int i = 0; i < nv; i++) {
    b[i] = mju_Halton(i, 3) - 0.5;
  }
  mj_effSolve(model.get(), data.get(), x.data(), b.data());
  EXPECT_EQ(data->warning[mjWARN_INERTIA].number, 0);
  mju_mulSymVecSparse(r.data(), data->M, x.data(), nv, model->M_rownnz,
                      model->M_rowadr, model->M_colind);
  mj_effMulAdd(model.get(), data.get(), r.data(), x.data(), /*flg_contact=*/1);
  mju_subFrom(r.data(), b.data(), nv);
  EXPECT_LT(mju_norm(r.data(), nv) / mju_norm(b.data(), nv), MjTol(1e-8, 1e-4));

  // Sweep narena from the non-effChol peak up to the sparse-factor peak across
  // the passive-contact effCholReserve budget: tight arenas may warn and fall
  // back, but must never overflow the arena.
  {
    MockWarningHandler warning_handler;
    warning_handler.ExpectWarnings("Insufficient arena memory");
    warning_handler.ExpectWarnings("Flex stiffness is too ill-conditioned");
    warning_handler.ExpectWarnings("Inertia matrix is too close to singular");
    int saved_perm0 = model->efmC_perm[0];
    model->efmC_perm[0] = -1;
    MjDataPtr db = MakeData(model);
    mju_copy(db->qpos, data->qpos, model->nq);
    mju_copy(db->qvel, data->qvel, model->nv);
    mj_forward(model.get(), db.get());
    size_t lo = db->maxuse_arena + 4096;
    model->efmC_perm[0] = saved_perm0;

    size_t orig = model->narena;
    size_t hi = data->maxuse_arena + 4096;
    for (int k = 0; k <= 16; k++) {
      model->narena = lo + (hi > lo ? ((hi - lo) * k) / 16 : 0);
      MjDataPtr dt = MakeData(model);
      mju_copy(dt->qpos, data->qpos, model->nq);
      mju_copy(dt->qvel, data->qvel, model->nv);
      mj_forward(model.get(), dt.get());
      EXPECT_LE(dt->maxuse_arena, dt->narena);
    }
    model->narena = orig;
  }
}

// Stiff 2D bending-only flex under Newton includes 2-hop bending flap pairs in
// the per-step sparse factor so mj_effSolve converges in one shot.
TEST_F(MetricTest, EffSolveBendFlapsNewton) {
  static const char* const kPaperXml = R"(
  <mujoco>
    <option timestep="0.002" integrator="discrete" solver="Newton"
            iterations="2" tolerance="1e-8"/>
    <worldbody>
      <flexcomp name="paper" type="grid" count="6 6 1" spacing="0.04 0.04 1"
                pos="0 0 0.2" radius="0.002" mass="0.02" dim="2">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e7" poisson="0.2" thickness="2e-3"
                    elastic2d="bend" damping="1e-4"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kPaperXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GT(data->nefmK, 0);
  ASSERT_GT(data->nefmL, 0);
  EXPECT_EQ(data->efm_L[0], 1);
  EXPECT_EQ(data->warning[mjWARN_INERTIA].number, 0);

  int nv = model->nv;
  std::vector<mjtNum> b(nv), x(nv), r(nv);
  for (int i = 0; i < nv; i++) {
    b[i] = mju_Halton(i, 3) - 0.5;
  }
  mj_effSolve(model.get(), data.get(), x.data(), b.data());
  EXPECT_EQ(data->warning[mjWARN_INERTIA].number, 0);
  mju_mulSymVecSparse(r.data(), data->M, x.data(), nv, model->M_rownnz,
                      model->M_rowadr, model->M_colind);
  mj_effMulAdd(model.get(), data.get(), r.data(), x.data(), /*flg_contact=*/1);
  mju_subFrom(r.data(), b.data(), nv);
  EXPECT_LT(mju_norm(r.data(), nv) / mju_norm(b.data(), nv), MjTol(1e-8, 1e-4));
}

// PCG requires a symmetric preconditioner. mj_effPrec must satisfy
// u.P(v) == v.P(u); it did not when the covered and uncovered dofs shared a
// kinematic tree, which is what the articulated vertex body here exercises.
TEST_F(MetricTest, EffPrecIsSymmetric) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kStretchCloth, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GE(data->efm_active, 1);
  ASSERT_EQ(data->nefmdof, 4);
  ASSERT_EQ(model->nv, 13);

  int nv = model->nv;
  std::vector<mjtNum> u(nv), v(nv), Pu(nv), Pv(nv);
  for (int trial = 0; trial < 5; trial++) {
    for (int i = 0; i < nv; i++) {
      u[i] = mju_Halton(i + trial * nv, 2) - 0.5;
      v[i] = mju_Halton(i + trial * nv, 5) - 0.5;
    }
    mj_effPrec(model.get(), data.get(), Pu.data(), u.data());
    mj_effPrec(model.get(), data.get(), Pv.data(), v.data());
    mjtNum a = mju_dot(v.data(), Pu.data(), nv);
    mjtNum b = mju_dot(u.data(), Pv.data(), nv);
    EXPECT_THAT(a, MjNear(b, 1e-10, 1e-4))
        << "preconditioner is not symmetric, trial " << trial;
  }
}

// Per-island and monolithic CG agree for both 3D flexes (effIslandStep) and 2D
// flexes + sleep (effCholReserved).
TEST_F(MetricTest, EffPrecFoldMultiIsland) {
  static constexpr char kXml3D[] = R"(
  <mujoco>
    <option solver="CG" integrator="discrete" tolerance="1e-12" iterations="300"/>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="cube0" type="grid" dim="3" count="2 2 2" spacing=".05 .05 .05"
                radius=".008" mass=".2" pos="-.3 0 .025">
        <contact selfcollide="none"/>
        <elasticity young="2e4" poisson=".3" damping=".01"/>
      </flexcomp>
      <flexcomp name="cube1" type="grid" dim="3" count="2 2 2" spacing=".05 .05 .05"
                radius=".008" mass=".3" pos=".3 0 .025">
        <contact selfcollide="none"/>
        <elasticity young="5e4" poisson=".25" damping=".01"/>
      </flexcomp>
      <body name="box" pos="0 .4 .04">
        <freejoint/>
        <geom type="box" size=".05 .05 .05" mass=".4"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kXml3D, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  for (mjtJacobian jac : {mjJAC_SPARSE, mjJAC_DENSE}) {
    model->opt.jacobian = jac;
    model->opt.disableflags &= ~mjDSBL_ISLAND;
    MjDataPtr data = MakeData(model);
    mjModel* m = model.get();
    mjData* d = data.get();
    mj_forward(m, d);

    ASSERT_EQ(d->nisland, 3);
    ASSERT_EQ(d->nefmdof, 16);

    model->opt.disableflags |= mjDSBL_ISLAND;
    MjDataPtr data_mono = MakeData(model);
    mj_forward(m, data_mono.get());
    for (int i = 0; i < m->nv; i++) {
      EXPECT_THAT(d->qacc[i], MjNear(data_mono->qacc[i], 1e-4, 1e-2))
          << "jac=" << jac << " dof=" << i;
    }
  }

  // 2D flexes with sleep="enable": effCholReserved holds so per-island CG
  // exercises the !has_isl fallback in mj_effPrecIsland.
  static constexpr char kXml2DSleep[] = R"(
  <mujoco>
    <option solver="CG" integrator="discrete" tolerance="1e-12" iterations="300">
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="s0" type="grid" dim="2" count="3 3 1" spacing=".05 .05 1"
                radius=".005" mass=".1" pos="-.3 0 .004">
        <contact selfcollide="none"/>
        <elasticity young="1e4" poisson=".3" thickness="2e-3" elastic2d="stretch"/>
      </flexcomp>
      <flexcomp name="s1" type="grid" dim="2" count="3 3 1" spacing=".05 .05 1"
                radius=".005" mass=".15" pos=".3 0 .004">
        <contact selfcollide="none"/>
        <elasticity young="2e4" poisson=".3" thickness="2e-3" elastic2d="stretch"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr model2d = LoadModelFromString(kXml2DSleep, error, sizeof(error));
  ASSERT_THAT(model2d.get(), NotNull()) << error;
  MjDataPtr data2d_isl = MakeData(model2d);
  mj_forward(model2d.get(), data2d_isl.get());
  ASSERT_EQ(data2d_isl->nisland, 2);

  model2d->opt.enableflags &= ~mjENBL_SLEEP;
  model2d->opt.disableflags |= mjDSBL_ISLAND;
  MjDataPtr data2d_mono = MakeData(model2d);
  mj_forward(model2d.get(), data2d_mono.get());
  for (int i = 0; i < model2d->nv; i++) {
    EXPECT_THAT(data2d_isl->qacc[i], MjNear(data2d_mono->qacc[i], 1e-4, 1e-2))
        << "2d sleep dof=" << i;
  }
}

TEST_F(MetricTest, EffUseQHDiagExactAndGeneralKDiag) {
  // 1. Undamped 3D flex with diagexact="enable": efm_diag is NULL while
  // nefmK > 0, so mj_effUseQH ensures mj_makeYNumeric whitens against qH.
  static constexpr char kXmlDiagExact[] = R"(
  <mujoco>
    <option integrator="discrete" timestep="0.01">
      <flag diagexact="enable"/>
    </option>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="cube" type="grid" dim="3" count="2 2 2"
                spacing=".05 .05 .05" radius=".01" mass=".2" pos="0 0 .025">
        <contact selfcollide="none"/>
        <elasticity young="1e5" poisson=".3" damping="0"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m_stiff = LoadModelFromString(kXmlDiagExact, error, sizeof(error));
  ASSERT_THAT(m_stiff.get(), testing::NotNull()) << error;
  MjDataPtr d_stiff = MakeData(m_stiff);
  mj_forward(m_stiff.get(), d_stiff.get());
  ASSERT_EQ(d_stiff->efm_diag, nullptr);
  ASSERT_GT(d_stiff->nefmK, 0);
  ASSERT_GT(d_stiff->nefc, 0);

  MjModelPtr m_soft = LoadModelFromString(kXmlDiagExact, error, sizeof(error));
  ASSERT_THAT(m_soft.get(), testing::NotNull()) << error;
  m_soft->flex_stiffness[0] = 0;
  for (int i = 0; i < m_soft->nflexstiffness; ++i) {
    m_soft->flex_stiffness[i] = 0;
  }
  MjDataPtr d_soft = MakeData(m_soft);
  mj_forward(m_soft.get(), d_soft.get());
  ASSERT_EQ(d_soft->nefc, d_stiff->nefc);
  EXPECT_LT(d_stiff->efc_diagA[0], d_soft->efc_diagA[0]);

  // 2. Articulated flex under Newton (general CSR, nefmdof == 0, nefmK > 0):
  // articulated trees omit diag(efm_K) from qH so parent-child inertia
  // couplings are not distorted without efm_K's off-diagonals, keeping qH ==
  // qLD when undamped; standalone 3-DOF slider trees (including trilinear
  // nodes) include diag(efm_K).
  static constexpr char kXmlNonSimple[] = R"(
  <mujoco>
    <option integrator="discrete" solver="Newton" timestep="0.01"/>
    <worldbody>
      <body name="base" pos="0 0 1">
        <joint type="slide" axis="0 0 1"/>
        <geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/>
        <flexcomp name="cloth" type="grid" count="3 3 1" spacing=".05 .05 .05"
                  radius=".005" dim="2" mass="0.5" pos="0 0 0" dof="full">
          <pin id="0"/>
          <contact selfcollide="none" contype="0" conaffinity="0"/>
          <elasticity young="1e4" poisson="0.2" damping="0" elastic2d="stretch"
                      thickness="0.01"/>
        </flexcomp>
      </body>
      <flexcomp name="tri" type="grid" count="2 2 2" spacing=".05 .05 .05"
                radius=".005" dim="3" mass="0.5" pos="0.5 0 1" dof="trilinear">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e4" poisson="0.2" damping="0"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m_gen = LoadModelFromString(kXmlNonSimple, error, sizeof(error));
  ASSERT_THAT(m_gen.get(), testing::NotNull()) << error;
  MjDataPtr d_gen = MakeData(m_gen);
  mj_forward(m_gen.get(), d_gen.get());
  ASSERT_GT(d_gen->nefmK, 0);
  int cloth_dof = 1;
  int child_diag = m_gen->M_rowadr[cloth_dof] + m_gen->M_rownnz[cloth_dof] - 1;
  EXPECT_MJTNUM_EQ(d_gen->qH[child_diag], d_gen->qLD[child_diag]);
  int tri_dof = m_gen->nv - 3;
  int tri_diag = m_gen->M_rowadr[tri_dof] + m_gen->M_rownnz[tri_dof] - 1;
  EXPECT_GT(d_gen->qH[tri_diag], d_gen->qLD[tri_diag]);
}

// Newton solver solves (M + S) * qacc_smooth = rhs directly by sparse or dense
// Cholesky whenever 3x3-block effChol does not cover the metric (uncovered 3D
// passive flex contacts, articulated/interpolated flexes, or 3D tetrahedral
// flexes), without hitting mjWARN_INERTIA at iterations=1, and falls back
// safely under tight stack.
TEST_F(MetricTest, EffSolveNewtonDirect) {
  auto check_residual = [](const mjModel* m, mjData* d) {
    std::vector<mjtNum> rhs(m->nv), lhs(m->nv);
    mju_add(rhs.data(), d->qfrc_smooth, d->efm_c, m->nv);
    if (d->efm_ca) {
      mju_addTo(rhs.data(), d->efm_ca, m->nv);
    }
    mj_mulM(m, d, lhs.data(), d->qacc_smooth);
    mj_effMulAdd(m, d, lhs.data(), d->qacc_smooth, /*flg_contact=*/1);
    mjtNum err = 0, norm = mju_norm(rhs.data(), m->nv);
    ASSERT_GT(norm, 0);
    for (int i = 0; i < m->nv; i++) {
      mjtNum diff = lhs[i] - rhs[i];
      err += diff * diff;
    }
    EXPECT_NEAR(mju_sqrt(err) / norm, 0.0, MjTol(1e-12, 5e-5));
  };

  auto rel_err = [](const mjtNum* a, const mjtNum* b, int n) {
    mjtNum num = 0, den = 0;
    for (int i = 0; i < n; i++) {
      mjtNum diff = a[i] - b[i];
      num += diff * diff;
      den += b[i] * b[i];
    }
    return mju_sqrt(num / mju_max(mjMINVAL, den));
  };

  // 1. Uncovered passive flex contacts (3D tet + 2D cloth): sparse, dense,
  // aliasing (x == b), and tight-stack fallback.
  static constexpr char kUncoveredPassiveXml[] = R"(
  <mujoco>
    <option integrator="discrete" solver="Newton" iterations="1"
            tolerance="1e-10"/>
    <worldbody>
      <geom type="plane" size="1 1 0.1" friction="0.6"/>
      <flexcomp name="lower" type="grid" count="3 3 2" spacing="0.04 0.04 0.04"
                pos="0 0 0.024" radius="0.006" mass="0.3" dim="3">
        <contact selfcollide="none" passive="true" condim="3" friction="0.6"/>
        <elasticity young="5e3" poisson="0.3" damping="1e-2"/>
      </flexcomp>
      <flexcomp name="upper" type="grid" count="4 4 1" spacing="0.04 0.04 0.04"
                pos="0.01 0.01 0.052" radius="0.006" mass="0.2" dim="2">
        <contact selfcollide="auto" passive="true" condim="3" friction="0.6"/>
        <elasticity young="5e3" poisson="0.3" thickness="2e-3"
                    elastic2d="both" damping="1e-2"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m =
      LoadModelFromString(kUncoveredPassiveXml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  for (int i = 0; i < m->nv; i++) {
    d->qvel[i] = 0.2 * mju_sin(0.9 * i + 0.3) - 0.1 * (i % 3 == 2);
  }
  mj_forward(m.get(), d.get());
  ASSERT_GT(d->nefmcon, 0);
  EXPECT_EQ(d->warning[mjWARN_INERTIA].number, 0);
  ASSERT_NO_FATAL_FAILURE(check_residual(m.get(), d.get()));

  // in-place aliasing (x == b) matches out-of-place solve
  std::vector<mjtNum> rhs(m->nv);
  mju_add(rhs.data(), d->qfrc_smooth, d->efm_c, m->nv);
  if (d->efm_ca) {
    mju_addTo(rhs.data(), d->efm_ca, m->nv);
  }
  std::vector<mjtNum> alias_buf = rhs;
  mj_effSolve(m.get(), d.get(), alias_buf.data(), alias_buf.data());
  for (int i = 0; i < m->nv; i++) {
    EXPECT_MJTNUM_EQ(alias_buf[i], d->qacc_smooth[i]);
  }

  // dense Jacobian path (!mj_isSparse) matches sparse path
  m->opt.jacobian = mjJAC_DENSE;
  MjDataPtr d_dense = MakeData(m);
  mju_copy(d_dense->qvel, d->qvel, m->nv);
  mj_forward(m.get(), d_dense.get());
  EXPECT_EQ(d_dense->warning[mjWARN_INERTIA].number, 0);
  EXPECT_NEAR(rel_err(d_dense->qacc_smooth, d->qacc_smooth, m->nv), 0.0,
              MjTol(1e-12, 2e-4));
  m->opt.jacobian = mjJAC_AUTO;

  // tight-stack fallback: restrict available stack so effSolveNewton backs off
  // to PCG (which expectedly warns at iterations=1) without crashing
  {
    MockWarningHandler warning_handler;
    warning_handler.ExpectWarnings("Flex stiffness is too ill-conditioned");
    warning_handler.ExpectWarnings("Inertia matrix is too close to singular");
    mj_markStack(d.get());
    size_t keep = 8 * static_cast<size_t>(m->nv) * sizeof(mjtNum) + 4096;
    size_t avail = d->narena - d->pstack - d->parena;
    if (avail > keep) {
      mj_stackAllocByte(d.get(), avail - keep, 8);
    }
    std::vector<mjtNum> fallback(m->nv);
    mj_effSolve(m.get(), d.get(), fallback.data(), rhs.data());
    mj_freeStack(d.get());
    EXPECT_GT(d->warning[mjWARN_INERTIA].number, 0);
  }

  // 2. Stiff interpolated (quadratic) flex under Newton: direct Cholesky solves
  // in one shot at iterations=1 without mjWARN_INERTIA.
  static constexpr char kInterpXml[] = R"(
  <mujoco>
    <option integrator="discrete" solver="Newton" iterations="1"
            tolerance="1e-10" timestep="0.001"/>
    <worldbody>
      <body name="base" pos="0 0 0.5">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="box" size="0.02 0.02 0.02" mass="0.1"
              contype="0" conaffinity="0"/>
        <flexcomp name="beam" type="grid" count="5 2 2"
                  spacing="0.04 0.03 0.03" pos="0.1 0 0" dim="3"
                  dof="quadratic" mass="0.2" radius="0.002">
          <elasticity young="1e7" poisson="0.3" damping="0.002"/>
          <contact selfcollide="none" contype="0" conaffinity="0"/>
          <pin id="0 1 2 3"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m_interp = LoadModelFromString(kInterpXml, error, sizeof(error));
  ASSERT_THAT(m_interp.get(), NotNull()) << error;
  MjDataPtr d_interp = MakeData(m_interp);
  mj_forward(m_interp.get(), d_interp.get());
  EXPECT_EQ(d_interp->warning[mjWARN_INERTIA].number, 0);
  ASSERT_NO_FATAL_FAILURE(check_residual(m_interp.get(), d_interp.get()));

  // 3. Stiff standalone 3D tet with nefmcon=0: solved directly by Cholesky
  // under Newton at iterations=1 without invoking iterative SGS.
  static constexpr char kStiffTetXml[] = R"(
  <mujoco>
    <option integrator="discrete" solver="Newton" iterations="1"
            tolerance="1e-10" timestep="0.002"/>
    <worldbody>
      <flexcomp name="tet" type="grid" count="3 3 3" spacing="0.04 0.04 0.04"
                pos="0 0 0.5" radius="0.002" mass="0.3" dim="3">
        <elasticity young="1e6" poisson="0.3" damping="0.01"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <pin id="0 1 2"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m_tet = LoadModelFromString(kStiffTetXml, error, sizeof(error));
  ASSERT_THAT(m_tet.get(), NotNull()) << error;
  MjDataPtr d_tet = MakeData(m_tet);
  for (int i = 0; i < m_tet->nv; i++) {
    d_tet->qvel[i] = 0.1 * mju_sin(0.7 * i + 0.2);
  }
  mj_forward(m_tet.get(), d_tet.get());
  ASSERT_EQ(d_tet->nefmcon, 0);
  EXPECT_EQ(d_tet->warning[mjWARN_INERTIA].number, 0);
  ASSERT_NO_FATAL_FAILURE(check_residual(m_tet.get(), d_tet.get()));

  // 4. Covered 2D cloth alongside a multi-DOF stiff tendon at iterations=0:
  // effCholCoversMetric routes to effSolveNewton without eagerly filling efm_L.
  static constexpr char kClothTendonXml[] = R"(
  <mujoco>
    <option integrator="discrete" solver="Newton" iterations="0"
            tolerance="1e-10" timestep="0.002"/>
    <worldbody>
      <flexcomp name="cloth" type="grid" count="4 4 1" spacing="0.05 0.05 1"
                pos="0 0 0.5" radius="0.002" mass="0.2" dim="2">
        <elasticity young="1e4" poisson="0.2" thickness="0.002"
                    elastic2d="both" damping="0.01"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <pin id="0 3"/>
      </flexcomp>
      <body pos="1 0 0.5">
        <joint name="j1" type="slide" axis="1 0 0"/>
        <geom type="sphere" size="0.02" mass="0.2" contype="0" conaffinity="0"/>
      </body>
      <body pos="1.2 0 0.5">
        <joint name="j2" type="slide" axis="1 0 0"/>
        <geom type="sphere" size="0.02" mass="0.2" contype="0" conaffinity="0"/>
      </body>
    </worldbody>
    <tendon>
      <fixed name="t" stiffness="1e5" damping="10">
        <joint joint="j1" coef="1"/>
        <joint joint="j2" coef="-1"/>
      </fixed>
    </tendon>
  </mujoco>
  )";
  MjModelPtr m_ten = LoadModelFromString(kClothTendonXml, error, sizeof(error));
  ASSERT_THAT(m_ten.get(), NotNull()) << error;
  MjDataPtr d_ten = MakeData(m_ten);
  d_ten->qpos[m_ten->nq - 2] = 0.05;
  mj_forward(m_ten.get(), d_ten.get());
  ASSERT_GT(d_ten->nefmL, 0);
  EXPECT_EQ(d_ten->efm_L[0], 2);  // EFF_CHOL_PENDING: efm_L not filled
  ASSERT_GT(d_ten->nefmT, 0);
  EXPECT_EQ(d_ten->warning[mjWARN_INERTIA].number, 0);
  ASSERT_NO_FATAL_FAILURE(check_residual(m_ten.get(), d_ten.get()));
}

}  // namespace
}  // namespace mujoco
