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

#ifndef MUJOCO_TEST_ENGINE_FLEX_TEST_FIXTURE_H_
#define MUJOCO_TEST_ENGINE_FLEX_TEST_FIXTURE_H_

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_util.h"
#include "src/engine/engine_derivative.h"
#include "src/engine/engine_metric.h"
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {

// 1D rods pinned at their first vertex (free first edge, per-edge twist joints)
inline constexpr char kRodStraightPinned[] = R"(
<mujoco>
  <option integrator="discrete" gravity="0 0 0"/>
  <worldbody>
    <body name="turned" euler="20 30 40">
      <flexcomp name="rod" type="grid" count="6 1 1" spacing="0.1 0.1 0.1"
                radius="0.01" dim="1" mass="1">
        <pin id="0"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e7"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
)";
inline constexpr char kRodHelixPinned[] = R"(
<mujoco>
  <option integrator="discrete" gravity="0 0 0"/>
  <worldbody>
    <body name="turned" euler="20 30 40">
      <flexcomp name="helix" type="direct" dim="1" radius="0.01" mass="1"
                element="0 1  1 2  2 3  3 4  4 5"
                point="0.15 0.00 0.00   0.106 0.106 0.04   0.00 0.15 0.08
                       -0.106 0.106 0.12   -0.15 0.00 0.16
                       -0.106 -0.106 0.20">
        <pin id="0"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e7"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
)";
inline constexpr char kRodVeePinned[] = R"(
<mujoco>
  <option integrator="discrete" gravity="0 0 0"/>
  <worldbody>
    <body name="turned" euler="20 30 40">
      <flexcomp name="vee" type="direct" dim="1" radius="0.01" mass="1"
                element="0 1  1 2" point="0 0 0  0.1 0 0  0.18 0.06 0.02">
        <pin id="0"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="1e7"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
)";

struct FlexFrameCase {
  const char* name;
  bool welded;
  bool rotated_siblings;
  int nflex;
};

class FlexFrameTest : public MujocoTest,
                      public testing::WithParamInterface<FlexFrameCase> {
 protected:
  void SetUp() override { LoadPatch(); }

  // Two equilateral triangles; only the body frames and attachment topology
  // vary.
  void LoadPatch(const char* elastic2d = "both") {
    std::string xml = R"(
      <mujoco>
        <option timestep="0.002" integrator="discrete" solver="CG"/>
        <worldbody><body name="parent" euler="90 35 20">
    )";
    const char* positions[] = {"0 0 0", "0.1 0 0", "0.05 0.0866025404 0",
                               "0.15 0.0866025404 0"};
    const char* rotations[] = {"0 0 0", "30 -15 20", "-20 40 10", "45 10 -35"};
    for (int v = 0; v < 4; v++) {
      const char* rotation =
          GetParam().rotated_siblings ? rotations[v] : "0 0 0";
      if (GetParam().welded && v == 3) {
        rotation = "25 -15 40";
      }
      xml += "<body name='v" + std::to_string(v) + "' pos='" + positions[v] +
             "' euler='" + rotation + R"('>
          <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
          <joint type="slide" axis="1 0 0" armature="0.01"/>
          <joint type="slide" axis="0 1 0" armature="0.02"/>
          <joint type="slide" axis="0 0 1" armature="0.03"/>
      )";
      if (GetParam().welded && v == 3) {
        xml += R"(<body name="welded" euler="20 10 30">
          <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
        </body>)";
      }
      xml += "</body>";
    }
    xml += "</body></worldbody><deformable>";
    for (int f = 0; f < GetParam().nflex; f++) {
      xml += std::string("<flex dim='2' body='v0 v1 v2 ") +
             (GetParam().welded ? "welded" : "v3") + R"(' element="0 1 2 1 3 2">
          <contact selfcollide="none" contype="0" conaffinity="0"/>
          <elasticity young="1e4" poisson="0.3" thickness="0.01" damping="0.02"
      )";
      xml += std::string("elastic2d='") + elastic2d + "'/></flex>";
    }
    xml += "</deformable></mujoco>";
    char error[1024];
    data.reset();
    model = LoadModelFromString(xml, error, sizeof(error));
    ASSERT_THAT(model.get(), ::testing::NotNull()) << error;
    ASSERT_EQ(model->nv, 12);
    if (!GetParam().welded) {
      for (int v = 0; v < 4; v++) {
        EXPECT_EQ(model->body_simple[model->flex_vertbodyid[v]], 2);
      }
    }
    data = MakeData(model);
    mj_forward(model.get(), data.get());
  }

  std::vector<mjtNum> Direction() const {
    std::vector<mjtNum> vec(model->nv);
    for (int i = 0; i < model->nv; i++) {
      vec[i] = mju_Halton(i, 3) - 0.5;
    }
    return vec;
  }

  // Dilate in world coordinates, then add a small out-of-plane perturbation.
  // All edges stay in tension, where the stretch Hessian is exact.
  void Deform() {
    for (int v = 0; v < 4; v++) {
      int body = model->body_weldid[model->flex_vertbodyid[v]];
      mjtNum delta[3];
      mju_scl3(delta, data->flexvert_xpos + 3 * v, 0.05);
      delta[2] += 1e-4 * (mju_Halton(v, 2) - 0.5);
      mju_mulMatTVec3(data->qpos + model->body_dofadr[body],
                      data->xmat + 9 * body, delta);
    }
    mj_forward(model.get(), data.get());
  }

  std::vector<mjtNum> Apply(const std::vector<mjtNum>& vec, mjtNum s1,
                            mjtNum s2) {
    std::vector<mjtNum> result(model->nv, 0);
    mjd_flexBend_mul(model.get(), data.get(), result.data(), vec.data(), s1,
                     s2);
    mjd_flexStretch_mul(model.get(), data.get(), result.data(), vec.data(), s1,
                        s2);
    return result;
  }

  void ExpectDerivative(bool velocity) {
    // The stretch operator includes geometric stiffness; its velocity part
    // equals the force Jacobian at the unstretched reference, where that extra
    // term vanishes.
    if (!velocity) {
      Deform();
    }
    auto vec = Direction();
    auto result = Apply(vec, !velocity, velocity);
    MjDataPtr perturbed(mj_copyData(nullptr, model.get(), data.get()));
    mjtNum eps = MjEps(1e-7, 1e-4);
    mju_addToScl(velocity ? perturbed->qvel : perturbed->qpos, vec.data(), eps,
                 model->nv);
    mj_forward(model.get(), perturbed.get());
    const mjtNum* force = velocity ? data->qfrc_damper : data->qfrc_spring;
    const mjtNum* pert_force =
        velocity ? perturbed->qfrc_damper : perturbed->qfrc_spring;
    for (int i = 0; i < model->nv; i++) {
      mjtNum fd = -(pert_force[i] - force[i]) / eps;
      EXPECT_NEAR(result[i], fd, MjTol(1e-3, 5e-2) * mju_max(1, mju_abs(fd)))
          << i;
    }
  }

  void ExpectAssembly(mjtNum s1, mjtNum s2) {
    Deform();
    auto vec = Direction();
    auto result = Apply(vec, s1, s2);
    std::vector<int> rownnz(model->nv), rowadr(model->nv);
    int nnz = mjd_flexStiff_assemble(model.get(), data.get(), rownnz.data(),
                                     rowadr.data(), nullptr, nullptr, s1, s2, 1,
                                     1, nullptr);
    std::vector<int> colind(nnz);
    std::vector<mjtNum> val(nnz);
    mjd_flexStiff_assemble(model.get(), data.get(), rownnz.data(),
                           rowadr.data(), colind.data(), val.data(), s1, s2, 1,
                           1, nullptr);
    for (int i = 0; i < model->nv; i++) {
      mjtNum assembled = mju_dotSparse(val.data() + rowadr[i], vec.data(),
                                       rownnz[i], colind.data() + rowadr[i]);
      EXPECT_THAT(assembled, MjNear(result[i], 1e-10, 2e-5)) << i;
    }
  }

  void ExpectBendingFactorInverse() {
    // Bending-only models use the constant factor, without per-step block
    // preconditioning.
    ASSERT_EQ(data->nefmdof, 0);
    ASSERT_EQ(model->nefm0dof, model->nv);
    int nv = model->nv;
    std::vector<mjtNum> rhs(nv), solution(nv), product(nv);
    mjtNum h = model->opt.timestep;
    for (int col = 0; col < nv; col++) {
      mju_zero(rhs.data(), nv);
      rhs[col] = 1;
      mj_effPrec(model.get(), data.get(), solution.data(), rhs.data());
      mj_mulM(model.get(), data.get(), product.data(), solution.data());
      mjd_flexBend_mul(model.get(), data.get(), product.data(), solution.data(),
                       h * h, h);
      for (int row = 0; row < nv; row++) {
        EXPECT_THAT(product[row], MjNear(rhs[row], 1e-12, 2e-6))
            << row << ", " << col;
      }
    }
  }

  MjModelPtr model;
  MjDataPtr data;
};

inline auto FlexFrameCases() {
  return ::testing::Values(FlexFrameCase{"Aligned", false, false, 1},
                           FlexFrameCase{"Welded", true, false, 1},
                           FlexFrameCase{"RotatedSiblings", false, true, 1},
                           FlexFrameCase{"SharedFlexBodies", true, true, 2});
}

inline std::string FlexFrameCaseName(
    const ::testing::TestParamInfo<FlexFrameCase>& info) {
  return info.param.name;
}

struct FlexAttachmentCase {
  const char* name;
  const char* joints;
  bool sliding_child = false;
  bool shared_body = false;
};

class FlexAttachmentTest
    : public MujocoTest,
      public testing::WithParamInterface<FlexAttachmentCase> {
 protected:
  void SetUp() override { Load(); }

  std::string Xml(bool independent, const char* elasticity,
                  bool mixed = false) {
    const char* inertia =
        R"(<inertial pos="0 0 0" mass="0.1" diaginertia=".01 .02 .03"/>)";
    const char* slides = R"(<joint type="slide" axis="1 0 0"/>
      <joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>)";
    const char* positions[] = {"0 0 0", ".1 0 0", "0 .1 0", ".1 .1 0"};
    std::string xml = R"(<mujoco><option integrator="discrete" solver="CG"
      gravity="0 0 0" iterations="500"/><worldbody>)";
    int count = independent ? 4 : (GetParam().shared_body ? 2 : 3);
    for (int v = 0; v < count; v++) {
      xml += "<body name='v" + std::to_string(v) + "' pos='" + positions[v] +
             "'>" + inertia + slides + "</body>";
    }
    if (!independent) {
      xml += std::string("<body name='carrier' pos='.08 .08 -.02'>") + inertia +
             GetParam().joints;
      if (GetParam().shared_body) xml += "<body name='v2' pos='-.08 .02 .02'/>";
      xml += "<body name='v3' pos='.02 .02 .02'>";
      if (GetParam().sliding_child) xml += std::string(inertia) + slides;
      xml += "</body></body>";
    }
    if (mixed) {
      xml += R"(<flexcomp name="ordinary" dim="2" type="grid" count="3 3 1"
        spacing=".1 .1 .1" pos="2 0 0" mass="1">
        <contact contype="0" conaffinity="0" selfcollide="none"/>
        <elasticity young="100" thickness=".02" damping=".02" elastic2d="stretch"/>
      </flexcomp>)";
    }
    xml +=
        R"(</worldbody><deformable><flex name="patch" dim="2" body="v0 v1 v2 v3"
      element="0 1 2 1 3 2"><contact contype="0" conaffinity="0" selfcollide="none"/>
      <elasticity young="100" poisson=".3" thickness=".02" damping=".02" elastic2d=")";
    return xml + elasticity + R"("/></flex></deformable></mujoco>)";
  }

  void Load(const char* elasticity = "both", bool mixed = false) {
    char error[1024];
    data.reset();
    model = LoadModelFromString(Xml(false, elasticity, mixed), error,
                                sizeof(error));
    ASSERT_THAT(model.get(), ::testing::NotNull()) << error;
    data = MakeData(model);
    mj_forward(model.get(), data.get());
    flex = mj_name2id(model.get(), mjOBJ_FLEX, "patch");
    ASSERT_FALSE(mj_flexSimple(model.get(), flex));
  }

  std::vector<mjtNum> Direction(int base = 2) {
    std::vector<mjtNum> result(model->nv);
    for (int i = 0; i < model->nv; i++) result[i] = mju_Halton(i, base) - .5;
    return result;
  }

  void Deform() {
    auto direction = Direction();
    mj_integratePos(model.get(), data->qpos, direction.data(), .05);
    mju_copy(data->qvel, direction.data(), model->nv);
    mj_forward(model.get(), data.get());
  }

  // Independent oracle: public point Jacobians and force application, not the
  // flex helpers.
  std::vector<mjtNum> Gather(const std::vector<mjtNum>& vec) {
    std::vector<mjtNum> result(12), jac(3 * model->nv);
    for (int v = 0; v < 4; v++) {
      int gv = model->flex_vertadr[flex] + v;
      mj_jac(model.get(), data.get(), jac.data(), nullptr,
             data->flexvert_xpos + 3 * gv, model->flex_vertbodyid[gv]);
      mju_mulMatVec(result.data() + 3 * v, jac.data(), vec.data(), 3,
                    model->nv);
    }
    return result;
  }

  std::vector<mjtNum> Scatter(const mjtNum* force) {
    std::vector<mjtNum> result(model->nv, 0);
    for (int v = 0; v < 4; v++) {
      int gv = model->flex_vertadr[flex] + v;
      mj_applyFT(model.get(), data.get(), force + 3 * v, nullptr,
                 data->flexvert_xpos + 3 * gv, model->flex_vertbodyid[gv],
                 result.data());
    }
    return result;
  }

  void Reference(const char* elasticity) {
    char error[1024];
    reference_data.reset();
    reference =
        LoadModelFromString(Xml(true, elasticity), error, sizeof(error));
    ASSERT_THAT(reference.get(), ::testing::NotNull()) << error;
    reference_data = MakeData(reference);
    auto velocity = Gather(AsVector(data->qvel, model->nv));
    mju_copy(reference_data->qvel, velocity.data(), 12);
    for (int v = 0; v < 4; v++) {
      int body = reference->flex_vertbodyid[v];
      mju_sub3(reference_data->qpos + 3 * v,
               data->flexvert_xpos + 3 * (model->flex_vertadr[flex] + v),
               reference->body_pos + 3 * body);
    }
    mj_forward(reference.get(), reference_data.get());
  }

  void ExpectForce(const char* elasticity, bool damping) {
    ASSERT_NO_FATAL_FAILURE(Load(elasticity));
    Deform();
    ASSERT_NO_FATAL_FAILURE(Reference(elasticity));
    auto expected = Scatter(damping ? reference_data->qfrc_damper
                                    : reference_data->qfrc_spring);
    const mjtNum* actual = damping ? data->qfrc_damper : data->qfrc_spring;
    EXPECT_GT(mju_norm(expected.data(), model->nv), MjTol(1e-7, 1e-4));
    EXPECT_THAT(AsVector(actual, model->nv),
                ::testing::Pointwise(MjNear(1e-10, 1e-5), expected));
  }

  void ExpectOperator(bool bend) {
    Deform();
    ASSERT_NO_FATAL_FAILURE(Reference("both"));
    auto vec = Direction(3), world = Gather(vec);
    auto apply = bend ? mjd_flexBend_mul : mjd_flexStretch_mul;
    std::vector<mjtNum> world_result(12, 0), result(model->nv + 3, 0);
    apply(reference.get(), reference_data.get(), world_result.data(),
          world.data(), 1, 0);
    result[model->nv] = result[model->nv + 1] = result[model->nv + 2] = 123;
    apply(model.get(), data.get(), result.data(), vec.data(), 1, 0);
    EXPECT_THAT(AsVector(result.data(), model->nv),
                ::testing::Pointwise(MjNear(1e-10, 2e-5),
                                     Scatter(world_result.data())));
    EXPECT_THAT(AsVector(result.data() + model->nv, 3), ::testing::Each(123));
  }

  void ExpectMetric(bool mixed) {
    ASSERT_NO_FATAL_FAILURE(Load("both", mixed));
    Deform();
    EXPECT_EQ(data->nefmK > 0, mixed);
    auto vec = Direction(3);
    std::vector<mjtNum> expected(model->nv, 0), actual(model->nv, 0);
    mjtNum h = model->opt.timestep;
    mjd_flexBend_mul(model.get(), data.get(), expected.data(), vec.data(),
                     h * h, h);
    mjd_flexStretch_mul(model.get(), data.get(), expected.data(), vec.data(),
                        h * h, h);
    mj_effMulAdd(model.get(), data.get(), actual.data(), vec.data(), 1);
    EXPECT_THAT(actual, ::testing::Pointwise(MjNear(1e-12, 1e-7), expected));
  }

  int flex;
  MjModelPtr model, reference;
  MjDataPtr data, reference_data;
};

inline auto FlexAttachmentCases() {
  return ::testing::Values(
      FlexAttachmentCase{"Hinge", "<joint axis='0 1 0'/>"},
      FlexAttachmentCase{"Ball", "<joint type='ball'/>"},
      FlexAttachmentCase{"Free", "<freejoint/>"},
      FlexAttachmentCase{"ReorderedSlides", R"(
                      <joint type="slide" axis="0 1 0"/>
                      <joint type="slide" axis="1 0 0"/>
                      <joint type="slide" axis="0 0 -1"/>)"},
      FlexAttachmentCase{"MovingAncestor", "<joint type='ball'/>", true},
      FlexAttachmentCase{"SharedBody", "<joint type='ball'/>", false, true});
}

inline std::string FlexAttachmentCaseName(
    const ::testing::TestParamInfo<FlexAttachmentCase>& info) {
  return info.param.name;
}

}  // namespace mujoco

#endif  // MUJOCO_TEST_ENGINE_FLEX_TEST_FIXTURE_H_
