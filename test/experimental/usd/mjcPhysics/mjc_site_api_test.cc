// Copyright 2025 DeepMind Technologies Limited
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/experimental/usd.h>
#include <mujoco/mujoco.h>
#include "src/experimental/usd/mjcPhysics/siteAPI.h"
#include "test/fixture.h"
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#define EXPECT_SITE_TYPE(spec, site_path, expected_type)                  \
  {                                                                       \
    mjsElement* site_element =                                            \
        mjs_findElement(spec, mjOBJ_SITE, site_path.GetString().c_str()); \
    EXPECT_THAT(site_element, NotNull());                                 \
    mjsSite* site = mjs_asSite(site_element);                             \
    EXPECT_EQ(site->type, expected_type);                                 \
  }

namespace mujoco {
namespace {

using pxr::MjcPhysicsSiteAPI;
using pxr::SdfPath;
using MjcSiteApiTest = MujocoTest;
using testing::NotNull;

TEST_F(MjcSiteApiTest, TestApply) {
  auto stage = pxr::UsdStage::CreateInMemory();

  auto test_body_path = SdfPath("/World/TestBody");
  auto body = pxr::UsdGeomXform::Define(stage, test_body_path);
  pxr::UsdPhysicsRigidBodyAPI::Apply(body.GetPrim());

  auto test_collider_path =
      test_body_path.AppendChild(pxr::TfToken("Collider"));
  auto collider = pxr::UsdGeomSphere::Define(stage, test_collider_path);
  pxr::UsdPhysicsCollisionAPI::Apply(collider.GetPrim());

  auto test_sphere_site_path =
      test_body_path.AppendChild(pxr::TfToken("SphereSite"));
  auto test_cylinder_site_path =
      test_body_path.AppendChild(pxr::TfToken("CylinderSite"));
  auto test_capsule_site_path =
      test_body_path.AppendChild(pxr::TfToken("CapsuleSite"));
  auto test_box_site_path = test_body_path.AppendChild(pxr::TfToken("BoxSite"));

  auto sphere = pxr::UsdGeomSphere::Define(stage, test_sphere_site_path);
  MjcPhysicsSiteAPI::Apply(sphere.GetPrim());

  auto cylinder = pxr::UsdGeomCylinder::Define(stage, test_cylinder_site_path);
  MjcPhysicsSiteAPI::Apply(cylinder.GetPrim());

  auto capsule = pxr::UsdGeomCapsule::Define(stage, test_capsule_site_path);
  MjcPhysicsSiteAPI::Apply(capsule.GetPrim());

  auto box = pxr::UsdGeomCube::Define(stage, test_box_site_path);
  MjcPhysicsSiteAPI::Apply(box.GetPrim());

  mjSpec* spec = mj_parseUSDStage(stage);
  mjModel* default_model = mj_compile(spec, nullptr);
  EXPECT_THAT(default_model, NotNull()) << mjs_getError(spec);

  EXPECT_SITE_TYPE(spec, test_sphere_site_path, mjGEOM_SPHERE);
  EXPECT_SITE_TYPE(spec, test_cylinder_site_path, mjGEOM_CYLINDER);
  EXPECT_SITE_TYPE(spec, test_capsule_site_path, mjGEOM_CAPSULE);
  EXPECT_SITE_TYPE(spec, test_box_site_path, mjGEOM_BOX);

  mj_deleteModel(default_model);
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
