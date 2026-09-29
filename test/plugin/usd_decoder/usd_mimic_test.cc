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

#include <filesystem>
#include <memory>
#include <numbers>
#include <string>
#include <tuple>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <pxr/base/arch/fileSystem.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

class UsdMimicTest : public MujocoTest {
 public:
  static void SetUpTestSuite() {
    mj_loadPluginLibrary(USD_DECODER_PLUGIN_PATH);
  }

 protected:
  void SetUp() override {
    directory_ = pxr::ArchMakeTmpSubdir(
        std::filesystem::temp_directory_path().string(), "usd_mimic");
    ASSERT_FALSE(directory_.empty());
  }

  void TearDown() override {
    if (!directory_.empty()) std::filesystem::remove_all(directory_);
  }

  pxr::UsdStageRefPtr MakeStage(bool angular = true, bool mjc_api = true) {
    auto stage = pxr::UsdStage::CreateInMemory();
    EXPECT_TRUE(stage->GetRootLayer()->ImportFromString(R"(#usda 1.0
(metersPerUnit = 1)
def PhysicsScene "scene" (prepend apiSchemas = ["MjcSceneAPI"]) {
    uniform token mjc:compiler:angle = "degree"
}
def Xform "leader" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
    def Sphere "geom" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
        double radius = 1
    }
    def PhysicsRevoluteJoint "joint" {
        rel physics:body1 = </leader>
    }
}
def Xform "other" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
    def Sphere "geom" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
        double radius = 1
    }
    def PhysicsRevoluteJoint "joint" {
        rel physics:body1 = </other>
    }
}
def Xform "follower" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
    def Sphere "geom" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
        double radius = 1
    }
    def PhysicsRevoluteJoint "joint" {
        rel physics:body1 = </follower>
        rel newton:mimicJoint
        rel mjc:target
        float newton:mimicCoef0
        float newton:mimicCoef1
        double mjc:coef0
        double mjc:coef1
        bool newton:mimicEnabled
        bool physics:jointEnabled
    }
}
)"));
    auto follower = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
    EXPECT_TRUE(follower.ApplyAPI(
        pxr::TfToken(mjc_api ? "MjcEqualityJointAPI" : "NewtonMimicAPI")));
    if (!angular) {
      for (const char* path :
           {"/leader/joint", "/other/joint", "/follower/joint"}) {
        EXPECT_TRUE(stage->GetPrimAtPath(pxr::SdfPath(path))
                        .SetTypeName(pxr::TfToken("PhysicsPrismaticJoint")));
      }
    }
    return stage;
  }

  MjModelPtr Compile(const pxr::UsdStageRefPtr& stage) {
    const std::string path = directory_ + "/mimic.usda";
    EXPECT_TRUE(stage->GetRootLayer()->Export(path));
    char error[1024] = {};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parse(path.c_str(), nullptr, nullptr, error, sizeof(error)),
        mj_deleteSpec);
    if (!spec) {
      ADD_FAILURE() << error;
      return nullptr;
    }
    MjModelPtr model(mj_compile(spec.get(), nullptr));
    EXPECT_NE(model, nullptr) << mjs_getError(spec.get());
    return model;
  }

  std::string directory_;
};

// MJC schema defaults must not override an authored Newton value. Test both
// direct NewtonMimicAPI and inheritance through MjcEqualityJointAPI.
class UsdMimicCoefficientsTest
    : public UsdMimicTest,
      public testing::WithParamInterface<std::tuple<bool, bool, bool, int>> {};

TEST_P(UsdMimicCoefficientsTest, UnitsDefaultsAndAuthoredPrecedence) {
  const auto [angular, degrees, mjc_api, source] = GetParam();
  auto stage = MakeStage(angular, mjc_api);
  auto joint = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
  ASSERT_TRUE(joint.GetRelationship(pxr::TfToken("newton:mimicJoint"))
                  .SetTargets({pxr::SdfPath("/leader/joint")}));
  ASSERT_TRUE(
      stage->GetAttributeAtPath(pxr::SdfPath("/scene.mjc:compiler:angle"))
          .Set(pxr::TfToken(degrees ? "degree" : "radian")));
  if (source == 1 || source == 3) {
    ASSERT_TRUE(
        joint.GetAttribute(pxr::TfToken("newton:mimicCoef0")).Set(18.0f));
    ASSERT_TRUE(
        joint.GetAttribute(pxr::TfToken("newton:mimicCoef1")).Set(2.0f));
  }
  if (source >= 2) {
    // An explicitly authored default still overrides the Newton value.
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("mjc:coef0")).Set(0.0));
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("mjc:coef1")).Set(1.0));
    mock_warning_handler.ExpectWarnings("deprecated mjc:");
  }
  auto model = Compile(stage);
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->neq, 1);
  const double expected =
      source == 1 ? (angular ? std::numbers::pi / 10 : 18) : 0;
  EXPECT_NEAR(model->eq_data[0], expected, MjTol(1e-15, 1e-7));
  EXPECT_EQ(model->eq_data[1], source == 1 ? 2 : 1);
}

INSTANTIATE_TEST_SUITE_P(Mimic, UsdMimicCoefficientsTest,
                         testing::Combine(testing::Bool(), testing::Bool(),
                                          testing::Bool(),
                                          testing::Values(0, 1, 2, 3)));

TEST_F(UsdMimicTest, LegacyPolynomialKeepsNativeUnits) {
  auto stage = MakeStage();
  auto joint = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
  ASSERT_TRUE(joint.GetRelationship(pxr::TfToken("mjc:target"))
                  .SetTargets({pxr::SdfPath("/leader/joint")}));
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("mjc:coef" + std::to_string(i)))
                    .Set(0.25 * (i + 1)));
  }
  mock_warning_handler.ExpectWarnings("deprecated mjc:");
  auto model = Compile(stage);
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->neq, 1);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(model->eq_data[i], 0.25 * (i + 1));
}

// -1 means unauthored, 0 false, 1 true. An explicit mimic flag is independent
// of the joint's enabled state; the latter is only a legacy fallback.
class UsdMimicEnabledTest
    : public UsdMimicTest,
      public testing::WithParamInterface<std::tuple<bool, int, int>> {};

TEST_P(UsdMimicEnabledTest, ExplicitMimicFlagAndLegacyFallback) {
  const auto [mjc_api, mimic_enabled, joint_enabled] = GetParam();
  auto stage = MakeStage(true, mjc_api);
  auto joint = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
  ASSERT_TRUE(joint.GetRelationship(pxr::TfToken("newton:mimicJoint"))
                  .SetTargets({pxr::SdfPath("/leader/joint")}));
  if (mimic_enabled >= 0) {
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("newton:mimicEnabled"))
                    .Set(mimic_enabled != 0));
  }
  if (joint_enabled >= 0) {
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("physics:jointEnabled"))
                    .Set(joint_enabled != 0));
  }
  auto model = Compile(stage);
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->neq, 1);
  const bool expected =
      mimic_enabled >= 0 ? mimic_enabled != 0 : joint_enabled != 0;
  EXPECT_EQ(model->eq_active0[0], expected);
  // Disabling the equality must preserve its follower joint and DOF.
  EXPECT_EQ(model->njnt, 3);
  EXPECT_EQ(model->nv, 3);
  auto data = MakeData(model);
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(data->eq_active[0], expected);
}

INSTANTIATE_TEST_SUITE_P(Mimic, UsdMimicEnabledTest,
                         testing::Combine(testing::Bool(),
                                          testing::Values(-1, 0, 1),
                                          testing::Values(-1, 0, 1)));

TEST_F(UsdMimicTest, CoefficientPrecedenceIsPerAttribute) {
  mock_warning_handler.ExpectWarnings("deprecated mjc:");
  for (int legacy_coef : {0, 1}) {
    SCOPED_TRACE(legacy_coef);
    auto stage = MakeStage();
    auto joint = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
    ASSERT_TRUE(joint.GetRelationship(pxr::TfToken("newton:mimicJoint"))
                    .SetTargets({pxr::SdfPath("/leader/joint")}));
    ASSERT_TRUE(
        joint.GetAttribute(pxr::TfToken("newton:mimicCoef0")).Set(18.0f));
    ASSERT_TRUE(
        joint.GetAttribute(pxr::TfToken("newton:mimicCoef1")).Set(2.0f));
    ASSERT_TRUE(joint
                    .GetAttribute(
                        pxr::TfToken("mjc:coef" + std::to_string(legacy_coef)))
                    .Set(0.5));
    auto model = Compile(stage);
    ASSERT_NE(model, nullptr);
    ASSERT_EQ(model->neq, 1);
    EXPECT_NEAR(model->eq_data[0],
                legacy_coef == 0 ? 0.5 : std::numbers::pi / 10,
                MjTol(1e-15, 1e-7));
    EXPECT_EQ(model->eq_data[1], legacy_coef == 1 ? 0.5 : 2.0);
  }
}

class UsdLoopJointEnabledTest
    : public UsdMimicTest,
      public testing::WithParamInterface<std::tuple<bool, bool>> {};

TEST_P(UsdLoopJointEnabledTest, WeldAndConnectKeepJointEnabledSemantics) {
  const auto [weld, enabled] = GetParam();
  auto stage = MakeStage();
  auto joint = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
  ASSERT_TRUE(joint.RemoveAPI(pxr::TfToken("MjcEqualityJointAPI")));
  ASSERT_TRUE(joint.SetTypeName(
      pxr::TfToken(weld ? "PhysicsFixedJoint" : "PhysicsSphericalJoint")));
  ASSERT_TRUE(joint.ApplyAPI(
      pxr::TfToken(weld ? "MjcEqualityWeldAPI" : "MjcEqualityConnectAPI")));
  ASSERT_TRUE(joint.CreateRelationship(pxr::TfToken("physics:body0"))
                  .SetTargets({pxr::SdfPath("/leader")}));
  ASSERT_TRUE(
      joint
          .CreateAttribute(pxr::TfToken("physics:excludeFromArticulation"),
                           pxr::SdfValueTypeNames->Bool)
          .Set(true));
  ASSERT_TRUE(
      joint.GetAttribute(pxr::TfToken("physics:jointEnabled")).Set(enabled));
  ASSERT_TRUE(
      joint.GetAttribute(pxr::TfToken("newton:mimicEnabled")).Set(!enabled));
  auto model = Compile(stage);
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->neq, 1);
  EXPECT_EQ(model->eq_type[0], weld ? mjEQ_WELD : mjEQ_CONNECT);
  EXPECT_EQ(model->eq_active0[0], enabled);
}

INSTANTIATE_TEST_SUITE_P(Mimic, UsdLoopJointEnabledTest,
                         testing::Combine(testing::Bool(), testing::Bool()));

class UsdMimicTargetTest : public UsdMimicTest,
                           public testing::WithParamInterface<int> {};

TEST_P(UsdMimicTargetTest, AuthoredMJCPrecedesNewtonIncludingEmptyTargets) {
  const int source = GetParam();
  auto stage = MakeStage();
  auto joint = stage->GetPrimAtPath(pxr::SdfPath("/follower/joint"));
  if (source == 1 || source >= 3) {
    ASSERT_TRUE(joint.GetRelationship(pxr::TfToken("newton:mimicJoint"))
                    .SetTargets({pxr::SdfPath("/leader/joint")}));
  }
  if (source >= 2) {
    pxr::SdfPathVector targets;
    if (source != 4) targets.push_back(pxr::SdfPath("/other/joint"));
    ASSERT_TRUE(
        joint.GetRelationship(pxr::TfToken("mjc:target")).SetTargets(targets));
    mock_warning_handler.ExpectWarnings("deprecated mjc:target");
  }
  auto model = Compile(stage);
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->neq, 1);
  int expected = -1;
  if (source == 1)
    expected = mj_name2id(model.get(), mjOBJ_JOINT, "/leader/joint");
  if (source == 2 || source == 3)
    expected = mj_name2id(model.get(), mjOBJ_JOINT, "/other/joint");
  EXPECT_EQ(model->eq_obj2id[0], expected);
}

INSTANTIATE_TEST_SUITE_P(Mimic, UsdMimicTargetTest, testing::Range(0, 5));

}  // namespace
}  // namespace mujoco
