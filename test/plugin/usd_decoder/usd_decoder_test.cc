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

#include <cmath>
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

class UsdMassModelTest : public MujocoTest {
 public:
  static void SetUpTestSuite() {
    mj_loadPluginLibrary(USD_DECODER_PLUGIN_PATH);
  }

 protected:
  void SetUp() override {
    directory_ = pxr::ArchMakeTmpSubdir(
        std::filesystem::temp_directory_path().string(), "usd_inertia");
    ASSERT_FALSE(directory_.empty());
  }

  void TearDown() override {
    if (!directory_.empty()) {
      std::filesystem::remove_all(directory_);
    }
  }

  // Unit density makes the compiled mass equal volume or surface area.
  pxr::UsdStageRefPtr MakeStage(bool mesh, bool collider, bool mjc_api,
                                bool shell) {
    auto stage = pxr::UsdStage::CreateInMemory();
    EXPECT_TRUE(stage->GetRootLayer()->ImportFromString(R"(#usda 1.0
(metersPerUnit = 1)
def Xform "body" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
    def Mesh "geom" (
        prepend apiSchemas = ["NewtonMassAPI", "PhysicsMassAPI", "PhysicsMeshCollisionAPI"]
    ) {
        point3f[] points = [(0,0,0),(1,0,0),(0,1,0),(0,0,1)]
        int[] faceVertexCounts = [3,3,3,3]
        int[] faceVertexIndices = [0,2,1,0,1,3,0,3,2,1,2,3]
        uniform token physics:approximation = "convexHull"
        uniform token newton:massModel = "solid"
        float physics:density = 1
        double radius = 1
        token mjc:inertia
    }
}
)"));
    auto geom = stage->GetPrimAtPath(pxr::SdfPath("/body/geom"));
    if (!mesh) {
      EXPECT_TRUE(geom.SetTypeName(pxr::TfToken("Sphere")));
    }
    if (collider) {
      EXPECT_TRUE(geom.ApplyAPI(pxr::TfToken("PhysicsCollisionAPI")));
    }
    if (mjc_api) {
      EXPECT_TRUE(geom.ApplyAPI(
          pxr::TfToken(mesh ? "MjcMeshCollisionAPI" : "MjcCollisionAPI")));
    }
    EXPECT_TRUE(geom.GetAttribute(pxr::TfToken("newton:massModel"))
                    .Set(pxr::TfToken(shell ? "shell" : "solid")));
    return stage;
  }

  void CheckStage(const pxr::UsdStageRefPtr& stage, bool mesh, bool shell,
                  mjtMeshInertia expected_inertia) {
    const std::string path = directory_ + "/geom.usda";
    ASSERT_TRUE(stage->GetRootLayer()->Export(path));
    char error[1024] = {};
    std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
        mj_parse(path.c_str(), nullptr, nullptr, error, sizeof(error)),
        mj_deleteSpec);
    ASSERT_NE(spec, nullptr) << error;
    auto* geom = mjs_asGeom(mjs_firstElement(spec.get(), mjOBJ_GEOM));
    ASSERT_NE(geom, nullptr);
    // Visual density parsing is separate from mass-model parsing; make the
    // reference density explicit here for both visual and collision geoms.
    geom->density = 1;
    EXPECT_EQ(geom->typeinertia,
              !mesh && shell ? mjINERTIA_SHELL : mjINERTIA_VOLUME);
    if (mesh) {
      auto* asset = mjs_asMesh(mjs_firstElement(spec.get(), mjOBJ_MESH));
      ASSERT_NE(asset, nullptr);
      EXPECT_EQ(asset->inertia, expected_inertia);
    }
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_compile(spec.get(), nullptr), mj_deleteModel);
    ASSERT_NE(model, nullptr) << mjs_getError(spec.get());
    ASSERT_EQ(model->nbody, 2);
    const double expected_mass =
        mesh ? (shell ? (3 + std::sqrt(3.0)) / 2 : 1.0 / 6)
             : (shell ? 4 * std::numbers::pi : 4 * std::numbers::pi / 3);
    EXPECT_NEAR(model->body_mass[1], expected_mass, 1e-6);
  }

  std::string directory_;
};

// Mesh/primitive, collider/visual, MJC API present/absent, shell/solid.
class UsdNewtonMassModelTest
    : public UsdMassModelTest,
      public testing::WithParamInterface<std::tuple<bool, bool, bool, bool>> {};

TEST_P(UsdNewtonMassModelTest, InertiaLivesOnMeshAssetOrPrimitiveGeom) {
  const auto [mesh, collider, mjc_api, shell] = GetParam();
  auto stage = MakeStage(mesh, collider, mjc_api, shell);
  CheckStage(stage, mesh, shell,
             shell ? mjMESH_INERTIA_SHELL : mjMESH_INERTIA_CONVEX);
}

INSTANTIATE_TEST_SUITE_P(MassModel, UsdNewtonMassModelTest,
                         testing::Combine(testing::Bool(), testing::Bool(),
                                          testing::Bool(), testing::Bool()));

TEST_F(UsdMassModelTest, DeprecatedMeshInertiaOverridesConvexApproximation) {
  for (bool collider : {false, true}) {
    for (const char* inertia : {"exact", "convex", "legacy", "shell"}) {
      SCOPED_TRACE(collider);
      SCOPED_TRACE(inertia);
      auto stage = MakeStage(true, collider, true, false);
      ASSERT_TRUE(
          stage->GetAttributeAtPath(pxr::SdfPath("/body/geom.mjc:inertia"))
              .Set(pxr::TfToken(inertia)));
      mock_warning_handler.ExpectWarnings("deprecated mjc:inertia");
      mjtMeshInertia expected_inertia = mjMESH_INERTIA_LEGACY;
      if (std::string(inertia) == "exact") {
        expected_inertia = mjMESH_INERTIA_EXACT;
      } else if (std::string(inertia) == "convex") {
        expected_inertia = mjMESH_INERTIA_CONVEX;
      } else if (std::string(inertia) == "shell") {
        expected_inertia = mjMESH_INERTIA_SHELL;
      }
      CheckStage(stage, true, std::string(inertia) == "shell",
                 expected_inertia);
    }
  }
}

}  // namespace
}  // namespace mujoco
