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
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// Joint kind, compiler angle units, and damping source (none/Newton/MJC/both).
class UsdJointDampingTest
    : public MujocoTest,
      public testing::WithParamInterface<std::tuple<bool, bool, int>> {
 public:
  static void SetUpTestSuite() {
    mj_loadPluginLibrary(USD_DECODER_PLUGIN_PATH);
  }

 protected:
  void SetUp() override {
    directory_ = pxr::ArchMakeTmpSubdir(
        std::filesystem::temp_directory_path().string(), "usd_damping");
    ASSERT_FALSE(directory_.empty());
  }

  void TearDown() override {
    if (!directory_.empty()) {
      std::filesystem::remove_all(directory_);
    }
  }

  std::string directory_;
};

TEST_P(UsdJointDampingTest, DampingUnitsAndLegacyPrecedence) {
  const auto [angular, degrees, source] = GetParam();
  auto stage = pxr::UsdStage::CreateInMemory();
  ASSERT_TRUE(stage->GetRootLayer()->ImportFromString(R"(#usda 1.0
(
    metersPerUnit = 1
    upAxis = "Z"
)
def PhysicsScene "scene" (
    prepend apiSchemas = ["MjcSceneAPI"]
) {
    uniform token mjc:compiler:angle = "degree"
}
def Xform "body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsArticulationRootAPI"]
) {
    def Sphere "geom" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    ) {
        double radius = 1
    }
    def PhysicsRevoluteJoint "joint" (
        prepend apiSchemas = ["MjcJointAPI"]
    ) {
        rel physics:body1 = </body>
        float newton:damping
        double mjc:damping
    }
}
)"));
  auto joint = stage->GetPrimAtPath(pxr::SdfPath("/body/joint"));
  if (!angular) {
    ASSERT_TRUE(joint.SetTypeName(pxr::TfToken("PhysicsPrismaticJoint")));
  }
  ASSERT_TRUE(
      stage->GetAttributeAtPath(pxr::SdfPath("/scene.mjc:compiler:angle"))
          .Set(pxr::TfToken(degrees ? "degree" : "radian")));
  if (source == 1 || source == 3) {
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("newton:damping")).Set(0.25f));
  }
  if (source == 2 || source == 3) {
    ASSERT_TRUE(joint.GetAttribute(pxr::TfToken("mjc:damping")).Set(0.7));
    mock_warning_handler.ExpectWarnings("deprecated mjc:damping");
  }
  const std::string path = directory_ + "/joint.usda";
  ASSERT_TRUE(stage->GetRootLayer()->Export(path));
  char error[1024] = {};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parse(path.c_str(), nullptr, nullptr, error, sizeof(error)),
      mj_deleteSpec);
  ASSERT_NE(spec, nullptr) << error;
  EXPECT_EQ(spec->compiler.degree, degrees);
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_compile(spec.get(), nullptr), mj_deleteModel);
  ASSERT_NE(model, nullptr) << mjs_getError(spec.get());
  ASSERT_EQ(model->njnt, 1);
  EXPECT_EQ(model->jnt_type[0], angular ? mjJNT_HINGE : mjJNT_SLIDE);
  double expected = 0;
  if (source == 1) {
    expected = angular ? 0.25 * 180.0 / std::numbers::pi : 0.25;
  } else if (source >= 2) {
    expected = 0.7;
  }
  EXPECT_NEAR(model->dof_damping[0], expected, 1e-6);
}

INSTANTIATE_TEST_SUITE_P(JointDamping, UsdJointDampingTest,
                         testing::Combine(testing::Bool(), testing::Bool(),
                                          testing::Values(0, 1, 2, 3)));

}  // namespace
}  // namespace mujoco
