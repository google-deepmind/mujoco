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

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "src/experimental/usd/mjcPhysics/actuatorAPI.h"
#include "src/experimental/usd/mjcPhysics/collisionAPI.h"
#include "src/experimental/usd/mjcPhysics/meshCollisionAPI.h"
#include "src/experimental/usd/mjcPhysics/sceneAPI.h"
#include "src/experimental/usd/mjcPhysics/siteAPI.h"
#include "src/experimental/usd/mjcPhysics/tokens.h"
#include "test/experimental/usd/test_utils.h"
#include "test/fixture.h"
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/pxr.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>  // IWYU pragma: keep, used for TraverseAll
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdPhysics/tokens.h>

PXR_NAMESPACE_OPEN_SCOPE
// clang-format off
TF_DEFINE_PRIVATE_TOKENS(_tokens,
                         (st)
                         );
// clang-format on
PXR_NAMESPACE_CLOSE_SCOPE

namespace mujoco {
namespace usd {
namespace {

using pxr::MjcPhysicsSiteAPI;
using pxr::MjcPhysicsTokens;
using pxr::SdfPath;
using MjcfSdfFileFormatPluginTest = MujocoTest;

static const char* kMaterialsPath =
    "experimental/usd/plugins/mjcf/testdata/materials.xml";
static const char* kMeshObjPath =
    "experimental/usd/plugins/mjcf/testdata/mesh_obj.xml";

TEST_F(MjcfSdfFileFormatPluginTest, TestClassAuthored) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <default>
        <default class="test">
        </default>
      </default>
      <worldbody>
        <body name="test_body" pos="0 0 0">
          <geom class="test" type="sphere" size="2 2 2"/>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_VALID(stage, "/__class__");
  EXPECT_PRIM_VALID(stage, "/__class__/test");
}

TEST_F(MjcfSdfFileFormatPluginTest, TestBasicMeshSources) {
  static constexpr char kXml[] = R"(
    <mujoco model="mesh test">
      <asset>
        <mesh name="tetrahedron" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      </asset>
      <worldbody>
        <body name="test_body">
          <geom type="mesh" mesh="tetrahedron"/>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_VALID(stage, "/mesh_test");
  EXPECT_PRIM_VALID(stage, "/mesh_test/test_body/tetrahedron");
  EXPECT_PRIM_VALID(stage, "/mesh_test/test_body/tetrahedron/Mesh");
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMaterials) {
  const std::string xml_path = GetTestDataFilePath(kMaterialsPath);

  auto stage = pxr::UsdStage::Open(xml_path);
  EXPECT_THAT(stage, testing::NotNull());

  EXPECT_PRIM_VALID(stage, "/mesh_test");
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials");

  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_red");
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_red/PreviewSurface");
  ExpectAttributeEqual(
      stage,
      "/mesh_test/Materials/material_red/PreviewSurface.inputs:diffuseColor",
      pxr::GfVec3f(0.8, 0, 0));

  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_texture");
  EXPECT_PRIM_VALID(stage,
                    "/mesh_test/Materials/material_texture/PreviewSurface");
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_texture/uvmap");
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_texture/diffuse");
  ExpectAttributeHasConnection(
      stage,
      "/mesh_test/Materials/material_texture/"
      "PreviewSurface.inputs:diffuseColor",
      "/mesh_test/Materials/material_texture/diffuse.outputs:rgb");
  ExpectAttributeEqual(
      stage, "/mesh_test/Materials/material_texture/diffuse.inputs:file",
      pxr::SdfAssetPath("textures/cube.png"));

  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_layered");
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_layered/uvmap");
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_layered/diffuse");

  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_layered/normal");
  ExpectAttributeHasConnection(
    stage,
    "/mesh_test/Materials/material_layered/"
    "PreviewSurface.inputs:normal",
    "/mesh_test/Materials/material_layered/normal.outputs:rgb");
  ExpectAttributeEqual(
    stage, "/mesh_test/Materials/material_layered/normal.inputs:file",
    pxr::SdfAssetPath("textures/normal.png"));

  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_layered/orm_packed");
  ExpectAttributeHasConnection(
    stage,
    "/mesh_test/Materials/material_layered/"
    "PreviewSurface.inputs:occlusion",
    "/mesh_test/Materials/material_layered/orm_packed.outputs:r");
  ExpectAttributeHasConnection(
    stage,
    "/mesh_test/Materials/material_layered/"
    "PreviewSurface.inputs:roughness",
    "/mesh_test/Materials/material_layered/orm_packed.outputs:g");
  ExpectAttributeHasConnection(
    stage,
    "/mesh_test/Materials/material_layered/"
    "PreviewSurface.inputs:metallic",
    "/mesh_test/Materials/material_layered/orm_packed.outputs:b");
  ExpectAttributeEqual(
    stage, "/mesh_test/Materials/material_layered/orm_packed.inputs:file",
    pxr::SdfAssetPath("textures/orm.png"));

  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_metallic");
  EXPECT_PRIM_VALID(stage,
                    "/mesh_test/Materials/material_metallic/PreviewSurface");
  ExpectAttributeEqual(stage,
    "/mesh_test/Materials/material_metallic/PreviewSurface.inputs:metallic",
    0.6f);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestGeomRgba) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <worldbody>
        <geom type="sphere" name="sphere_red" size="1" rgba="1 0 0 1"/>
        <geom type="sphere" name="sphere_default" size="1"/>
        <geom type="sphere" name="sphere_also_default" size="1" rgba="0.5 0.5 0.5 1"/>
        <geom type="sphere" name="sphere_almost_default" size="1" rgba="0.5 0.5 0.5 0.9"/>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);
  auto stage = pxr::UsdStage::Open(layer);

  EXPECT_PRIM_VALID(stage, "/test/sphere_red");
  ExpectAttributeEqual(stage, "/test/sphere_red.primvars:displayColor",
                       pxr::VtArray<pxr::GfVec3f>{{1, 0, 0}});
  EXPECT_ATTRIBUTE_HAS_NO_VALUE(stage,
                                "/test/sphere_red.primvars:displayOpacity");

  // There's no mechanism in Mujoco to specify whether an attribute was set
  // explicitly or not. We do the same as Mujoco does, which is to compare with
  // the default value.
  // Which explains why not setting rgba is the same as setting it to the
  // default value of (0.5, 0.5, 0.5, 1).
  EXPECT_PRIM_VALID(stage, "/test/sphere_default");
  EXPECT_ATTRIBUTE_HAS_NO_VALUE(stage,
                                "/test/sphere_default.primvars:displayColor");
  EXPECT_ATTRIBUTE_HAS_NO_VALUE(stage,
                                "/test/sphere_default.primvars:displayOpacity");

  EXPECT_PRIM_VALID(stage, "/test/sphere_also_default");
  EXPECT_ATTRIBUTE_HAS_NO_VALUE(
      stage, "/test/sphere_also_default.primvars:displayColor");
  EXPECT_ATTRIBUTE_HAS_NO_VALUE(
      stage, "/test/sphere_also_default.primvars:displayOpacity");

  EXPECT_PRIM_VALID(stage, "/test/sphere_almost_default");
  ExpectAttributeEqual(stage,
                       "/test/sphere_almost_default.primvars:displayColor",
                       pxr::VtArray<pxr::GfVec3f>{{0.5, 0.5, 0.5}});
  ExpectAttributeEqual(stage,
                       "/test/sphere_almost_default.primvars:displayOpacity",
                       pxr::VtArray<float>{0.9});
}

TEST_F(MjcfSdfFileFormatPluginTest, TestFaceVaryingMeshSourcesSimpleMjcfMesh) {
  static constexpr char kXml[] = R"(
    <mujoco model="mesh test">
      <asset>
        <mesh
          name="tetrahedron"
          face="0 3 2  0 1 3  0 2 1  1 2 3"
          vertex="0 1 0  0 0 0  1 0 1  1 0 -1"
          normal="1 0 0  0 1 0  0 0 1  -1 0 0"
          texcoord="0.5 0.5  0 0.5  1 1  1 0"/>
      </asset>
      <worldbody>
        <body name="test_body">
          <geom type="mesh" mesh="tetrahedron"/>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);

  auto mesh = pxr::UsdGeomMesh::Get(
      stage, SdfPath("/mesh_test/test_body/tetrahedron/Mesh"));
  ASSERT_TRUE(mesh);
  pxr::VtArray<int> face_vertex_counts;
  mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts);
  EXPECT_EQ(face_vertex_counts.size(), 4);
  EXPECT_EQ(face_vertex_counts, pxr::VtArray<int>({3, 3, 3, 3}));

  pxr::VtArray<int> face_vertex_indices;
  mesh.GetFaceVertexIndicesAttr().Get(&face_vertex_indices);
  EXPECT_EQ(face_vertex_indices.size(), 12);
  EXPECT_EQ(face_vertex_indices,
            pxr::VtArray<int>({0, 3, 2, 0, 1, 3, 0, 2, 1, 1, 2, 3}));

  pxr::VtArray<pxr::GfVec3f> normals;
  mesh.GetNormalsAttr().Get(&normals);
  EXPECT_EQ(normals.size(), face_vertex_indices.size());
  // We can't directly check the normals values because they are altered by
  // Mujoco's compiling step. So we at least check that normals with the same
  // original index are the same.
  for (int i = 0; i < face_vertex_indices.size(); ++i) {
    for (int j = i + 1; j < face_vertex_indices.size(); ++j) {
      if (face_vertex_indices[i] == face_vertex_indices[j]) {
        EXPECT_EQ(normals[i], normals[j]);
      }
    }
  }

  auto primvars_api = pxr::UsdGeomPrimvarsAPI(mesh.GetPrim());

  pxr::VtArray<pxr::GfVec2f> texcoords;
  EXPECT_TRUE(primvars_api.HasPrimvar(pxr::_tokens->st));
  auto primvar_st = primvars_api.GetPrimvar(pxr::_tokens->st);
  primvar_st.Get(&texcoords);
  EXPECT_EQ(texcoords.size(), face_vertex_indices.size());

  // Check the faceVarying texcoords against the manually indexed source
  // texcoords.
  pxr::VtArray<pxr::GfVec2f> source_texcoords{
      {0.5, 0.5}, {0, 0.5}, {1, 0}, {1, 1}};
  for (int i = 0; i < face_vertex_indices.size(); ++i) {
    EXPECT_EQ(texcoords[i], source_texcoords[face_vertex_indices[i]]);
  }
}

TEST_F(MjcfSdfFileFormatPluginTest,
       TestFaceVaryingMeshSourcesObjWithIndexedNormals) {
  const std::string xml_path = GetTestDataFilePath(kMeshObjPath);

  auto stage = pxr::UsdStage::Open(xml_path);
  EXPECT_THAT(stage, testing::NotNull());

  auto mesh =
      pxr::UsdGeomMesh::Get(stage, SdfPath("/mesh_test/test_body/mesh/Mesh"));
  ASSERT_TRUE(mesh);
  pxr::VtArray<int> face_vertex_counts;
  mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts);
  EXPECT_EQ(face_vertex_counts.size(), 4);
  EXPECT_EQ(face_vertex_counts, pxr::VtArray<int>({3, 3, 3, 3}));

  pxr::VtArray<int> face_vertex_indices;
  mesh.GetFaceVertexIndicesAttr().Get(&face_vertex_indices);
  EXPECT_EQ(face_vertex_indices.size(), 12);
  EXPECT_EQ(face_vertex_indices,
            pxr::VtArray<int>({0, 3, 2, 0, 1, 3, 0, 2, 1, 1, 2, 3}));

  pxr::VtArray<pxr::GfVec3f> normals;
  mesh.GetNormalsAttr().Get(&normals);
  EXPECT_EQ(normals.size(), face_vertex_indices.size());
  // We can't directly check the normals values because they are altered by
  // Mujoco's compiling step.
  // We also can't access the normals indexing data, and can't use the vertex
  // indexing data here because they are separate.
  // So we check that the first half of the normals are the same, then the
  // second half, as set in the OBJ file.
  pxr::GfVec3f first_half_normal = normals[0];
  pxr::GfVec3f second_half_normal = normals[face_vertex_indices.size() / 2];
  EXPECT_NE(first_half_normal, second_half_normal);
  int i = 0;
  for (; i < face_vertex_indices.size() / 2; ++i) {
    EXPECT_EQ(normals[i], first_half_normal);
  }
  for (; i < face_vertex_indices.size(); ++i) {
    EXPECT_EQ(normals[i], second_half_normal);
  }

  auto primvars_api = pxr::UsdGeomPrimvarsAPI(mesh.GetPrim());

  pxr::VtArray<pxr::GfVec2f> texcoords;
  EXPECT_TRUE(primvars_api.HasPrimvar(pxr::_tokens->st));
  auto primvar_st = primvars_api.GetPrimvar(pxr::_tokens->st);
  primvar_st.Get(&texcoords);
  EXPECT_EQ(texcoords.size(), face_vertex_indices.size());

  // Check the faceVarying texcoords against the manually indexed source
  // texcoords.
  // NOTE: For OBJ we must use different indices for the texcoords than for the
  // vertices!
  std::vector<int> source_face_texcoord_indices{0, 1, 2, 1, 2, 3,
                                                2, 3, 0, 3, 0, 1};
  pxr::VtArray<pxr::GfVec2f> source_texcoords{
      {0.5, 0.5}, {0, 0.5}, {1, 0}, {1, 1}};
  // NOTE: The v component of the texcoords is flipped when Mujoco loads the
  // OBJ.
  for (auto& uv : source_texcoords) {
    uv[1] = 1 - uv[1];
  }
  for (int i = 0; i < source_face_texcoord_indices.size(); ++i) {
    EXPECT_EQ(texcoords[i], source_texcoords[source_face_texcoord_indices[i]]);
  }
}

TEST_F(MjcfSdfFileFormatPluginTest, TestBody) {
  static constexpr char kXml[] = R"(
    <mujoco model="body test">
      <asset>
        <mesh name="tetrahedron" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      </asset>
      <worldbody>
        <body name="test_body" pos="0 1 0">
          <joint type="free" />
          <frame pos="0 0 1">
            <frame pos="0 0 1">
              <body name="test_body_2" pos="1 0 0">
                <geom type="mesh" mesh="tetrahedron"/>
              </body>
            </frame>
          </frame>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);

  EXPECT_PRIM_VALID(stage, "/body_test");
  EXPECT_PRIM_VALID(stage, "/body_test/test_body");
  EXPECT_PRIM_VALID(stage, "/body_test/test_body/test_body_2");
}

TEST_F(MjcfSdfFileFormatPluginTest, TestBasicParenting) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <worldbody>
        <body name="root" pos="0 1 0">
              <body name="root/body_1" pos="1 0 0" />
              <body name="root/body_2" pos="1 0 0">
                <body name="root/body_3" pos="1 0 0" />
              </body>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_VALID(stage, "/test/root");
  EXPECT_PRIM_VALID(stage, "/test/root/root_body_1");
  EXPECT_PRIM_VALID(stage, "/test/root/root_body_2");
  EXPECT_PRIM_VALID(stage, "/test/root/root_body_2/root_body_3");
}

TEST_F(MjcfSdfFileFormatPluginTest, TestJointsDoNotAffectParenting) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <asset>
        <mesh name="tetrahedron" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      </asset>
      <worldbody>
        <body name="root" pos="0 1 0">
          <joint type="free" />
          <geom type="mesh" mesh="tetrahedron"/>
          <body name="middle">
            <body name="tet">
              <joint type="hinge" />
              <geom type="mesh" mesh="tetrahedron"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_VALID(stage, "/test/root");
  EXPECT_PRIM_VALID(stage, "/test/root/middle");
  EXPECT_PRIM_VALID(stage, "/test/root/middle/tet");
}

TEST_F(MjcfSdfFileFormatPluginTest, TestKindAuthoring) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <asset>
        <mesh name="tetrahedron" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      </asset>
      <worldbody>
        <body name="root" pos="0 1 0">
          <joint type="free" />
          <geom type="mesh" mesh="tetrahedron"/>
          <body name="middle">
            <body name="tet">
              <joint type="hinge" />
              <geom type="mesh" mesh="tetrahedron"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_KIND(stage, "/test", pxr::KindTokens->group);
  EXPECT_PRIM_KIND(stage, "/test/root", pxr::KindTokens->component);
  EXPECT_PRIM_KIND(stage, "/test/root/middle", pxr::KindTokens->subcomponent);
  EXPECT_PRIM_KIND(stage, "/test/root/middle/tet",
                   pxr::KindTokens->subcomponent);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestAttributesMatchSchemaTypes) {
  // TODO(robinalazard): Make the scene much more comprehensive. We ideally want
  // to test all the prims that the plugin can generate.
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <worldbody>
        <geom type="plane" name="plane_geom" size="10 20 0.1"/>
        <geom type="box" name="box_geom" size="10 20 30"/>
        <geom type="sphere" name="sphere_geom" size="10 20 30"/>
        <geom type="capsule" name="capsule_geom" size="10 20 30"/>
        <geom type="cylinder" name="cylinder_geom" size="10 20 30"/>
        <geom type="ellipsoid" name="ellipsoid_geom" size="10 20 30"/>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);
  auto stage = pxr::UsdStage::Open(layer);

  for (const auto& prim : stage->TraverseAll()) {
    ExpectAllAuthoredAttributesMatchSchemaTypes(prim);
  }
}

TEST_F(MjcfSdfFileFormatPluginTest, TestGeomsPrims) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <worldbody>
        <geom type="plane" name="plane_geom" size="10 20 0.1"/>
        <geom type="box" name="box_geom" size="10 20 30"/>
        <geom type="sphere" name="sphere_geom" size="10 20 30"/>
        <geom type="capsule" name="capsule_geom" size="10 20 30"/>
        <geom type="cylinder" name="cylinder_geom" size="10 20 30"/>
        <geom type="ellipsoid" name="ellipsoid_geom" size="10 20 30"/>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfLayerRefPtr layer = LoadLayer(kXml);
  auto stage = pxr::UsdStage::Open(layer);

  // Note that all sizes are multiplied by 2 because Mujoco uses half sizes.

  // Plane
  EXPECT_PRIM_VALID(stage, "/test/plane_geom");
  EXPECT_PRIM_IS_A(stage, "/test/plane_geom", pxr::UsdGeomPlane);
  ExpectAttributeEqual(stage, "/test/plane_geom.width", 2 * 10.0);
  ExpectAttributeEqual(stage, "/test/plane_geom.length", 2 * 20.0);
  // Box
  EXPECT_PRIM_VALID(stage, "/test/box_geom");
  EXPECT_PRIM_IS_A(stage, "/test/box_geom", pxr::UsdGeomCube);
  // Box is a special case, it uses a UsdGeomCube and scales it with
  // xformOp:scale. The radius is always set to 2.
  ExpectAttributeEqual(stage, "/test/box_geom.size", 2.0);
  ExpectAttributeEqual(stage, "/test/box_geom.xformOp:scale",
                       pxr::GfVec3f(10.0, 20.0, 30.0));
  // Sphere
  EXPECT_PRIM_VALID(stage, "/test/sphere_geom");
  EXPECT_PRIM_IS_A(stage, "/test/sphere_geom", pxr::UsdGeomSphere);
  ExpectAttributeEqual(stage, "/test/sphere_geom.radius", 10.0);
  // Capsule
  EXPECT_PRIM_VALID(stage, "/test/capsule_geom");
  EXPECT_PRIM_IS_A(stage, "/test/capsule_geom", pxr::UsdGeomCapsule);
  ExpectAttributeEqual(stage, "/test/capsule_geom.radius", 10.0);
  ExpectAttributeEqual(stage, "/test/capsule_geom.height", 2 * 20.0);
  // Cylinder
  EXPECT_PRIM_VALID(stage, "/test/cylinder_geom");
  EXPECT_PRIM_IS_A(stage, "/test/cylinder_geom", pxr::UsdGeomCylinder);
  ExpectAttributeEqual(stage, "/test/cylinder_geom.radius", 10.0);
  ExpectAttributeEqual(stage, "/test/cylinder_geom.height", 2 * 20.0);
  // Ellipsoid
  EXPECT_PRIM_VALID(stage, "/test/ellipsoid_geom");
  // Ellipsoid is a special case, it uses a UsdGeomSphere and scales it with
  // xformOp:scale. The radius is always set to 1.
  EXPECT_PRIM_IS_A(stage, "/test/ellipsoid_geom", pxr::UsdGeomSphere);
  ExpectAttributeEqual(stage, "/test/ellipsoid_geom.radius", 1.0);
  ExpectAttributeEqual(stage, "/test/ellipsoid_geom.xformOp:scale",
                       pxr::GfVec3f(10.0, 20.0, 30.0));
}

static const pxr::SdfPath kPhysicsScenePrimPath("/test/PhysicsScene");

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimTimestep) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option timestep="0.005"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionTimestep),
      0.005);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimCone) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option cone="elliptic"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionCone),
      MjcPhysicsTokens->elliptic);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimWind) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option wind="1 2 3"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionWind),
      pxr::GfVec3d(1, 2, 3));
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimApirate) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option apirate="1.2"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionApirate),
      1.2);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimImpratio) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option impratio="0.8"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionImpratio),
      0.8);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimMagnetic) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option magnetic="1 2 3"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionMagnetic),
      pxr::GfVec3d(1, 2, 3));
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimDensity) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option density="1.2"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionDensity),
      1.2);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimViscosity) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option viscosity="0.8"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionViscosity),
                       0.8);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimO_margin) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option o_margin="0.001"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionO_margin),
      0.001);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimO_solref) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option o_solref="0.1 0.2"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionO_solref),
      pxr::VtArray<double>({0.1, 0.2}));
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimO_solimp) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option o_solimp="0.1 0.2 0.3 0.4 0.5"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionO_solimp),
      pxr::VtArray<double>({0.1, 0.2, 0.3, 0.4, 0.5}));
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimTolerance) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option tolerance="0.0012"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionTolerance),
                       0.0012);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimLSTolerance) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option ls_tolerance="0.0034"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionLs_tolerance),
                       0.0034);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimNoslipTolerance) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option noslip_tolerance="0.0056"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionNoslip_tolerance),
                       0.0056);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimCCDTolerance) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option ccd_tolerance="0.0078"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionCcd_tolerance),
                       0.0078);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimOFriction) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option o_friction="0.1 0.2 0.3 0.4 0.5"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionO_friction),
                       pxr::VtArray<double>({0.1, 0.2, 0.3, 0.4, 0.5}));
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimIntegrator) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option integrator="RK4"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionIntegrator),
                       MjcPhysicsTokens->rk4);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimJacobian) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option jacobian="sparse"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionJacobian),
      MjcPhysicsTokens->sparse);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimSolver) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option solver="CG"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(
      stage,
      kPhysicsScenePrimPath.AppendProperty(MjcPhysicsTokens->mjcOptionSolver),
      MjcPhysicsTokens->cg);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimIterations) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option iterations="10"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionIterations),
                       10);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimLSIterations) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option ls_iterations="20"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionLs_iterations),
                       20);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimNoslipIterations) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option noslip_iterations="30"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionNoslip_iterations),
                       30);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimCCDIterations) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option ccd_iterations="40"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionCcd_iterations),
                       40);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimSDFInitPoints) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option sdf_initpoints="50"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionSdf_initpoints),
                       50);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimSDFIterations) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option sdf_iterations="60"> </option>
    </mujoco>
  )"));

  ExpectAttributeEqual(stage,
                       kPhysicsScenePrimPath.AppendProperty(
                           MjcPhysicsTokens->mjcOptionSdf_iterations),
                       60);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimDisableFlags) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option>
        <flag
          constraint="disable"
          equality="disable"
          frictionloss="disable"
          limit="disable"
          contact="disable"
          passive="disable"
          gravity="disable"
          clampctrl="disable"
          warmstart="disable"
          filterparent="disable"
          actuation="disable"
          refsafe="disable"
          sensor="disable"
          midphase="disable"
          nativeccd="disable"
          eulerdamp="disable"
          autoreset="disable"
        />
      </option>
    </mujoco>
  )"));

  const std::vector<pxr::TfToken> kFlags = {
      MjcPhysicsTokens->mjcFlagConstraint,
      MjcPhysicsTokens->mjcFlagEquality,
      MjcPhysicsTokens->mjcFlagFrictionloss,
      MjcPhysicsTokens->mjcFlagLimit,
      MjcPhysicsTokens->mjcFlagContact,
      MjcPhysicsTokens->mjcFlagPassive,
      MjcPhysicsTokens->mjcFlagGravity,
      MjcPhysicsTokens->mjcFlagClampctrl,
      MjcPhysicsTokens->mjcFlagWarmstart,
      MjcPhysicsTokens->mjcFlagFilterparent,
      MjcPhysicsTokens->mjcFlagActuation,
      MjcPhysicsTokens->mjcFlagRefsafe,
      MjcPhysicsTokens->mjcFlagSensor,
      MjcPhysicsTokens->mjcFlagMidphase,
      MjcPhysicsTokens->mjcFlagNativeccd,
      MjcPhysicsTokens->mjcFlagEulerdamp,
      MjcPhysicsTokens->mjcFlagAutoreset,
  };
  for (const auto& flag : kFlags) {
    ExpectAttributeEqual(stage, kPhysicsScenePrimPath.AppendProperty(flag),
                         false);
  }
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsScenePrimEnableFlags) {
  auto stage = pxr::UsdStage::Open(LoadLayer(R"(
    <mujoco model="test">
      <option>
        <flag
          override="enable"
          energy="enable"
          fwdinv="enable"
          invdiscrete="enable"
          multiccd="enable"
          island="enable"
        />
      </option>
    </mujoco>
  )"));

  // clang-format off
  const std::vector<pxr::TfToken> kFlags = {
      MjcPhysicsTokens->mjcFlagOverride,
      MjcPhysicsTokens->mjcFlagEnergy,
      MjcPhysicsTokens->mjcFlagFwdinv,
      MjcPhysicsTokens->mjcFlagInvdiscrete,
      MjcPhysicsTokens->mjcFlagMulticcd,
      MjcPhysicsTokens->mjcFlagIsland,
  };
  // clang-format on
  for (const auto& flag : kFlags) {
    ExpectAttributeEqual(stage, kPhysicsScenePrimPath.AppendProperty(flag),
                         true);
  }
}

static constexpr char kSiteXml[] = R"(
    <mujoco model="test">
      <worldbody>
        <site type="box" name="box_site"/>
        <body name="ball">
          <site type="sphere" name="sphere_site"/>
          <site type="capsule" name="capsule_site"/>
          <site type="cylinder" name="cylinder_site"/>
          <site type="ellipsoid" name="ellipsoid_site"/>
          <geom type="sphere" size="1 1 1"/>
        </body>
      </worldbody>
    </mujoco>
  )";

TEST_F(MjcfSdfFileFormatPluginTest, TestSitePrimsAuthored) {
  pxr::SdfLayerRefPtr layer = LoadLayer(kSiteXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_VALID(stage, "/test/box_site");
  EXPECT_PRIM_IS_A(stage, "/test/box_site", pxr::UsdGeomCube);
  EXPECT_PRIM_API_APPLIED(stage, "/test/box_site", pxr::MjcPhysicsSiteAPI);
  EXPECT_PRIM_VALID(stage, "/test/ball/sphere_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/sphere_site", pxr::UsdGeomSphere);
  EXPECT_PRIM_API_APPLIED(stage, "/test/ball/sphere_site",
                          pxr::MjcPhysicsSiteAPI);
  EXPECT_PRIM_VALID(stage, "/test/ball/capsule_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/capsule_site", pxr::UsdGeomCapsule);
  EXPECT_PRIM_API_APPLIED(stage, "/test/ball/capsule_site",
                          pxr::MjcPhysicsSiteAPI);
  EXPECT_PRIM_VALID(stage, "/test/ball/cylinder_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/cylinder_site", pxr::UsdGeomCylinder);
  EXPECT_PRIM_API_APPLIED(stage, "/test/ball/cylinder_site",
                          pxr::MjcPhysicsSiteAPI);
  EXPECT_PRIM_VALID(stage, "/test/ball/ellipsoid_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/ellipsoid_site", pxr::UsdGeomSphere);
  EXPECT_PRIM_API_APPLIED(stage, "/test/ball/ellipsoid_site",
                          pxr::MjcPhysicsSiteAPI);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestSitePrimsPurpose) {
  pxr::SdfLayerRefPtr layer = LoadLayer(kSiteXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_PURPOSE(stage, "/test/box_site", pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/sphere_site",
                      pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/capsule_site",
                      pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/cylinder_site",
                      pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/ellipsoid_site",
                      pxr::UsdGeomTokens->guide);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsToggleSdfFormatArg) {
  std::string xml_path = GetTestDataFilePath(kMeshObjPath);

  // Test that the default is no physics.
  auto stage_no_physics = pxr::UsdStage::Open(xml_path);
  EXPECT_THAT(stage_no_physics, testing::NotNull());
  EXPECT_PRIM_VALID(stage_no_physics, "/mesh_test/test_body");
  EXPECT_PRIM_API_NOT_APPLIED(stage_no_physics, "/mesh_test/test_body",
                              pxr::UsdPhysicsRigidBodyAPI);

  // Then test that the physics flag enables physics.
  std::string xml_path_physics_flag =
      xml_path + ":SDF_FORMAT_ARGS:usdMjcfToggleUsdPhysics=true";
  auto stage_with_physics = pxr::UsdStage::Open(xml_path_physics_flag);
  EXPECT_THAT(stage_with_physics, testing::NotNull());

  EXPECT_PRIM_VALID(stage_with_physics, "/mesh_test/test_body");
  EXPECT_PRIM_API_APPLIED(stage_with_physics, "/mesh_test/test_body",
                          pxr::UsdPhysicsRigidBodyAPI);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsRigidBody) {
  static constexpr char kXml[] = R"(
    <mujoco model="physics_test">
      <worldbody>
        <body name="test_body" pos="0 0 0">
          <geom name="test_geom" type="sphere" size="1"/>
          <body name="test_body_2" pos="2 0 0">
            <geom name="test_geom_2" type="sphere" size="1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfFileFormat::FileFormatArguments args;
  args["usdMjcfToggleUsdPhysics"] = "true";
  pxr::SdfLayerRefPtr layer = LoadLayer(kXml, args);
  auto stage = pxr::UsdStage::Open(layer);

  EXPECT_THAT(stage, testing::NotNull());
  EXPECT_PRIM_VALID(stage, "/physics_test");
  EXPECT_PRIM_VALID(stage, "/physics_test/test_body");
  EXPECT_PRIM_VALID(stage, "/physics_test/test_body/test_body_2");

  EXPECT_PRIM_API_APPLIED(stage, "/physics_test/test_body",
                          pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/physics_test/test_body",
                          pxr::UsdPhysicsArticulationRootAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/physics_test/test_body/test_body_2",
                          pxr::UsdPhysicsRigidBodyAPI);

  // Only the root body should have the articulation API applied.
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/physics_test/test_body/test_body_2",
                              pxr::UsdPhysicsArticulationRootAPI);

  // Geoms should not have RigidBodyAPI applied either.
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/physics_test/test_body/test_geom",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage,
                              "/physics_test/test_body/test_body_2/test_geom_2",
                              pxr::UsdPhysicsRigidBodyAPI);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsColliders) {
  static constexpr char kXml[] = R"(
    <mujoco model="test">
      <asset>
        <mesh name="tetrahedron" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      </asset>
      <worldbody>
        <geom name="ground" type="plane" size="5 5 0.1"
              contype="1" conaffinity="1"/>
        <body name="body_0" pos="-3 0 2">
          <joint type="free"/>
          <geom name="body_0_col" type="sphere" size="1"
                contype="1" conaffinity="1"/>
          <body name="body_0_0" pos="-1 0 2">
            <geom name="body_0_0_col" type="sphere" size="1"
                  contype="1" conaffinity="1"/>
          </body>
        </body>
        <body name="body_1" pos="0 0 3">
          <joint type="free"/>
          <geom name="body_1_col_0" type="sphere" size="1"
                contype="1" conaffinity="1"/>
          <geom name="body_1_col_1" type="sphere" size="1" pos="-1 0 2"
                contype="1" conaffinity="1"/>
        </body>
        <body name="body_2" pos="3 0 3">
          <joint type="free"/>
          <geom name="body_2_nocol" type="sphere" size="1"
                contype="0" conaffinity="0"/>
        </body>
        <body name="body_3" pos="0 3 3">
          <joint type="free"/>
          <geom name="body_3_col" type="mesh" mesh="tetrahedron"
                contype="1" conaffinity="1"/>
        </body>
      </worldbody>
    </mujoco>
  )";

  pxr::SdfFileFormat::FileFormatArguments args;
  args["usdMjcfToggleUsdPhysics"] = "true";
  pxr::SdfLayerRefPtr layer = LoadLayer(kXml, args);
  auto stage = pxr::UsdStage::Open(layer);

  EXPECT_THAT(stage, testing::NotNull());
  EXPECT_PRIM_VALID(stage, "/test");

  // Expected hierarchy under /test:
  //
  // ground [collider]
  //
  // body_0 [rigidbody]
  //   body_0/body_0_col [collider]
  //
  // body_0/body_0_0 [rigidbody]
  //     body_0/body_0_0/body_0_0_col [collider]
  //
  // body_1 [rigidbody]
  //   body_1/body_1_col_0 [collider]
  //   body_1/body_1_col_1 [collider]
  //
  // body_2 [rigidbody]
  //   body_2/body_2_nocol []
  //
  // body_3 [rigidbody]
  //   body_3/body_3_col []  <-- Intermediate prim for mesh instancing
  //     body_3/body_3_col/Mesh [collider, mesh collider]

  // ground [collider] (Static collider)
  EXPECT_PRIM_VALID(stage, "/test/ground");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/ground",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/ground", pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/ground", pxr::MjcPhysicsCollisionAPI);

  // body_0/body_0_0 [rigidbody] (Nested body - reparented)
  EXPECT_PRIM_VALID(stage, "/test/body_0/body_0_0");
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_0/body_0_0",
                          pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_0/body_0_0",
                              pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_0/body_0_0",
                              pxr::MjcPhysicsCollisionAPI);
  //   body_0/body_0_0/body_0_0_col [collider]
  EXPECT_PRIM_VALID(stage, "/test/body_0/body_0_0/body_0_0_col");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_0/body_0_0/body_0_0_col",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_0/body_0_0/body_0_0_col",
                          pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_0/body_0_0/body_0_0_col",
                          pxr::MjcPhysicsCollisionAPI);

  // body_1 [rigidbody]
  EXPECT_PRIM_VALID(stage, "/test/body_1");
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_1", pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_1",
                              pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_1",
                              pxr::MjcPhysicsCollisionAPI);
  //   body_1/body_1_col_0 [collider]
  EXPECT_PRIM_VALID(stage, "/test/body_1/body_1_col_0");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_1/body_1_col_0",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_1/body_1_col_0",
                          pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_1/body_1_col_0",
                          pxr::MjcPhysicsCollisionAPI);
  //   body_1/body_1_col_1 [collider]
  EXPECT_PRIM_VALID(stage, "/test/body_1/body_1_col_1");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_1/body_1_col_1",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_1/body_1_col_1",
                          pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_1/body_1_col_1",
                          pxr::MjcPhysicsCollisionAPI);

  // body_2 [rigidbody]
  EXPECT_PRIM_VALID(stage, "/test/body_2");
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_2", pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_2",
                              pxr::UsdPhysicsCollisionAPI);
  //   body_2/body_2_nocol [] (No physics APIs applied)
  EXPECT_PRIM_VALID(stage, "/test/body_2/body_2_nocol");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_2/body_2_nocol",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_2/body_2_nocol",
                              pxr::UsdPhysicsCollisionAPI);

  // body_3 [rigidbody]
  EXPECT_PRIM_VALID(stage, "/test/body_3");
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_3", pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_3",
                              pxr::UsdPhysicsCollisionAPI);
  //   body_3/body_3_col [] (Intermediate prim for mesh instancing)
  EXPECT_PRIM_VALID(stage, "/test/body_3/body_3_col");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_3/body_3_col",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_3/body_3_col",
                              pxr::UsdPhysicsCollisionAPI);
  //     body_3/body_3_col/Mesh [collider, mesh collider]
  EXPECT_PRIM_VALID(stage, "/test/body_3/body_3_col/Mesh");
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body_3/body_3_col/Mesh",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_3/body_3_col/Mesh",
                          pxr::UsdPhysicsCollisionAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_3/body_3_col/Mesh",
                          pxr::UsdPhysicsMeshCollisionAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body_3/body_3_col/Mesh",
                          pxr::MjcPhysicsMeshCollisionAPI);
  ExpectAttributeEqual(stage,
                       "/test/body_3/body_3_col/Mesh.physics:approximation",
                       pxr::UsdPhysicsTokens->convexHull);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMjcPhysicsCollisionAPI) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <geom name="box" type="box" size=".05 .05 .05" mass="0.1" shellinertia="true"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  ExpectAttributeEqual(stage, "/test/body/box.mjc:shellinertia", true);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMjcPhysicsMeshCollisionAPI) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <asset>
      <mesh name="tet_legacy" inertia="legacy" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="tet_exact" inertia="exact" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="tet_convex" inertia="convex" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="tet_shell" inertia="shell" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>
    <worldbody>
      <body name="body">
        <geom name="tet_legacy" type="mesh" mesh="tet_legacy"/>
        <geom name="tet_exact" type="mesh" mesh="tet_exact"/>
        <geom name="tet_convex" type="mesh" mesh="tet_convex"/>
        <geom name="tet_shell" type="mesh" mesh="tet_shell"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  ExpectAttributeEqual(stage, "/test/body/tet_legacy/Mesh.mjc:inertia",
                       MjcPhysicsTokens->legacy);
  ExpectAttributeEqual(stage, "/test/body/tet_exact/Mesh.mjc:inertia",
                       MjcPhysicsTokens->exact);
  ExpectAttributeEqual(stage, "/test/body/tet_convex/Mesh.mjc:inertia",
                       MjcPhysicsTokens->convex);
  ExpectAttributeEqual(stage, "/test/body/tet_shell/Mesh.mjc:inertia",
                       MjcPhysicsTokens->shell);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMassAPIApplied) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <geom name="box" type="box" size=".05 .05 .05" mass="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  EXPECT_PRIM_VALID(stage, "/test/body");
  EXPECT_PRIM_VALID(stage, "/test/body/box");
  EXPECT_PRIM_API_APPLIED(stage, "/test/body/box", pxr::UsdPhysicsMassAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/test/body", pxr::UsdPhysicsMassAPI);
  ExpectAttributeEqual(stage, "/test/body/box.physics:mass", 0.1f);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMassAPIAppliedToBody) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <inertial pos="1 2 3" mass="3" diaginertia="1 1 1"/>
        <geom name="box" type="box" size=".05 .05 .05" mass="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  EXPECT_PRIM_VALID(stage, "/test/body");
  EXPECT_PRIM_VALID(stage, "/test/body/box");
  EXPECT_PRIM_API_APPLIED(stage, "/test/body/box", pxr::UsdPhysicsMassAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/test/body", pxr::UsdPhysicsMassAPI);
  // Make sure that body gets it's inertial elements from the inertial element
  // and not from the subtree.
  ExpectAttributeEqual(stage, "/test/body.physics:mass", 3.0f);
  ExpectAttributeEqual(stage, "/test/body.physics:centerOfMass",
                       pxr::GfVec3f(1, 2, 3));
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMassAPIDensity) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <geom name="box" type="box" size=".05 .05 .05" density="1234"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  ExpectAttributeEqual(stage, "/test/body/box.physics:density", 1234.0f);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMjcPhysicsActuatorGeneral) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <geom name="box" type="box" size=".05 .05 .05" density="1234"/>
        <site name="site"/>
        <site name="ref"/>
      </body>
    </worldbody>
    <actuator>
      <general
        name="general"
        site="site"
        refsite="ref"
        ctrllimited="true"
        ctrlrange="0 1"
        forcelimited="true"
        forcerange="2 3"
        actlimited="false"
        actrange="4 5"
        lengthrange="6 7"
        actdim="1"
        actearly="true"
        dyntype="filter"
        gaintype="user"
        biastype="user"
        gear="1 2 3 4 5 6"
        dynprm="0 1 2 3 4 5 6 7 8 9"
        gainprm="0 1 2 3 4 5 6 7 8 9"
        biasprm="0 1 2 3 4 5 6 7 8 9"
      />
    </actuator>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  EXPECT_PRIM_API_APPLIED(stage, "/test/body/site", pxr::MjcPhysicsActuatorAPI);
  EXPECT_REL_HAS_TARGET(stage, "/test/body/site.mjc:refSite", "/test/body/ref");
  ExpectAttributeEqual(stage, "/test/body/site.mjc:ctrlLimited",
                       pxr::MjcPhysicsTokens->true_);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:ctrlRange:min", 0.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:ctrlRange:max", 1.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:forceLimited",
                       pxr::MjcPhysicsTokens->true_);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:forceRange:min", 2.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:forceRange:max", 3.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:actLimited",
                       pxr::MjcPhysicsTokens->false_);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:actRange:min", 4.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:actRange:max", 5.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:lengthRange:min", 6.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:lengthRange:max", 7.0);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:actDim", 1);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:dynType",
                       MjcPhysicsTokens->filter);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:gainType",
                       MjcPhysicsTokens->user);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:biasType",
                       MjcPhysicsTokens->user);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:actEarly", true);
  ExpectAttributeEqual(stage, "/test/body/site.mjc:gear",
                       pxr::VtDoubleArray{{1, 2, 3, 4, 5, 6}});
  ExpectAttributeEqual(stage, "/test/body/site.mjc:dynPrm",
                       pxr::VtDoubleArray{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}});
  ExpectAttributeEqual(stage, "/test/body/site.mjc:gainPrm",
                       pxr::VtDoubleArray{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}});
  ExpectAttributeEqual(stage, "/test/body/site.mjc:biasPrm",
                       pxr::VtDoubleArray{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9}});
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMjcPhysicsBodyActuator) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <geom name="box" type="box" size=".05 .05 .05" density="1234"/>
      </body>
    </worldbody>
    <actuator>
      <general
        name="general"
        body="body"
      />
    </actuator>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  EXPECT_PRIM_API_APPLIED(stage, "/test/body", pxr::MjcPhysicsActuatorAPI);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestMjcPhysicsSliderCrankActuator) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <worldbody>
      <body name="body">
        <geom name="box" type="box" size=".05 .05 .05" density="1234"/>
        <site name="crank"/>
        <site name="slider"/>
      </body>
    </worldbody>
    <actuator>
      <general
        name="general"
        cranksite="crank"
        slidersite="slider"
        cranklength="1.23"
      />
    </actuator>
  </mujoco>
  )";
  auto stage = OpenStageWithPhysics(xml);

  EXPECT_PRIM_API_APPLIED(stage, "/test/body/crank",
                          pxr::MjcPhysicsActuatorAPI);
  EXPECT_REL_HAS_TARGET(stage, "/test/body/crank.mjc:sliderSite",
                        "/test/body/slider");
  ExpectAttributeEqual(stage, "/test/body/crank.mjc:crankLength", 1.23);
}

}  // namespace
}  // namespace usd
}  // namespace mujoco
