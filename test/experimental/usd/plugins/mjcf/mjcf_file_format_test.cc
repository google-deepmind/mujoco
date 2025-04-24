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
#include <absl/strings/string_view.h>
#include "test/experimental/usd/plugins/mjcf/fixture.h"
#include "test/fixture.h"
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/pxr.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/prim.h>
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
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

PXR_NAMESPACE_OPEN_SCOPE
// clang-format off
TF_DEFINE_PRIVATE_TOKENS(_tokens,
                         (st)
                         );
// clang-format on
PXR_NAMESPACE_CLOSE_SCOPE

namespace mujoco {
namespace {

using pxr::SdfPath;

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
  EXPECT_PRIM_VALID(stage, "/mesh_test/test_body/test_body/tetrahedron");
  EXPECT_PRIM_VALID(stage, "/mesh_test/test_body/test_body/tetrahedron/Mesh");
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
  EXPECT_PRIM_VALID(stage, "/mesh_test/Materials/material_texture/texture");
  ExpectAttributeHasConnection(
      stage,
      "/mesh_test/Materials/material_texture/"
      "PreviewSurface.inputs:diffuseColor",
      "/mesh_test/Materials/material_texture/texture.outputs:rgb");
  ExpectAttributeEqual(
      stage, "/mesh_test/Materials/material_texture/texture.inputs:file",
      pxr::SdfAssetPath("textures/cube.png"));
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
      stage, SdfPath("/mesh_test/test_body/test_body/tetrahedron/Mesh"));
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

  auto mesh = pxr::UsdGeomMesh::Get(
      stage, SdfPath("/mesh_test/test_body/test_body/mesh/Mesh"));
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
  EXPECT_PRIM_VALID(stage, "/body_test/test_body/test_body");
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
  EXPECT_PRIM_VALID(stage, "/test/root/root");
  EXPECT_PRIM_VALID(stage, "/test/root/root_body_1");
  EXPECT_PRIM_VALID(stage, "/test/root/root_body_2");
  EXPECT_PRIM_VALID(stage, "/test/root/root_body_3");
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
  EXPECT_PRIM_VALID(stage, "/test/root/root");
  EXPECT_PRIM_VALID(stage, "/test/root/middle");
  EXPECT_PRIM_VALID(stage, "/test/root/tet");
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
  EXPECT_PRIM_KIND(stage, "/test/root/root", pxr::KindTokens->subcomponent);
  EXPECT_PRIM_KIND(stage, "/test/root/middle", pxr::KindTokens->subcomponent);
  EXPECT_PRIM_KIND(stage, "/test/root/tet", pxr::KindTokens->subcomponent);
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
  ExpectAttributeEqual(stage, "/test/sphere_geom.radius", 2 * 10.0);
  // Capsule
  EXPECT_PRIM_VALID(stage, "/test/capsule_geom");
  EXPECT_PRIM_IS_A(stage, "/test/capsule_geom", pxr::UsdGeomCapsule);
  ExpectAttributeEqual(stage, "/test/capsule_geom.radius", 2 * 10.0);
  ExpectAttributeEqual(stage, "/test/capsule_geom.height", 2 * 20.0);
  // Cylinder
  EXPECT_PRIM_VALID(stage, "/test/cylinder_geom");
  EXPECT_PRIM_IS_A(stage, "/test/cylinder_geom", pxr::UsdGeomCylinder);
  ExpectAttributeEqual(stage, "/test/cylinder_geom.radius", 2 * 10.0);
  ExpectAttributeEqual(stage, "/test/cylinder_geom.height", 2 * 20.0);
  // Ellipsoid
  EXPECT_PRIM_VALID(stage, "/test/ellipsoid_geom");
  // Ellipsoid is a special case, it uses a UsdGeomSphere and scales it with
  // xformOp:scale. The radius is always set to 1.
  EXPECT_PRIM_IS_A(stage, "/test/ellipsoid_geom", pxr::UsdGeomSphere);
  ExpectAttributeEqual(stage, "/test/ellipsoid_geom.radius", 1.0);
  ExpectAttributeEqual(stage, "/test/ellipsoid_geom.xformOp:scale",
                       pxr::GfVec3f(2.0 * 10.0, 2.0 * 20.0, 2.0 * 30.0));
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
  EXPECT_PRIM_VALID(stage, "/test/ball/ball/sphere_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/ball/sphere_site", pxr::UsdGeomSphere);
  EXPECT_PRIM_VALID(stage, "/test/ball/ball/capsule_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/ball/capsule_site", pxr::UsdGeomCapsule);
  EXPECT_PRIM_VALID(stage, "/test/ball/ball/cylinder_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/ball/cylinder_site",
                   pxr::UsdGeomCylinder);
  EXPECT_PRIM_VALID(stage, "/test/ball/ball/ellipsoid_site");
  EXPECT_PRIM_IS_A(stage, "/test/ball/ball/ellipsoid_site", pxr::UsdGeomSphere);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestSitePrimsPurpose) {
  pxr::SdfLayerRefPtr layer = LoadLayer(kSiteXml);

  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_PRIM_PURPOSE(stage, "/test/box_site", pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/ball/sphere_site",
                      pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/ball/capsule_site",
                      pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/ball/cylinder_site",
                      pxr::UsdGeomTokens->guide);
  EXPECT_PRIM_PURPOSE(stage, "/test/ball/ball/ellipsoid_site",
                      pxr::UsdGeomTokens->guide);
}

TEST_F(MjcfSdfFileFormatPluginTest, TestPhysicsToggleSdfFormatArg) {
  std::string xml_path = GetTestDataFilePath(kMeshObjPath);

  // Test that the default is no physics.
  auto stage_no_physics = pxr::UsdStage::Open(xml_path);
  EXPECT_THAT(stage_no_physics, testing::NotNull());
  EXPECT_PRIM_VALID(stage_no_physics, "/mesh_test/test_body/test_body");
  EXPECT_PRIM_API_NOT_APPLIED(stage_no_physics,
                              "/mesh_test/test_body/test_body",
                              pxr::UsdPhysicsRigidBodyAPI);

  // Then test that the physics flag enables physics.
  std::string xml_path_physics_flag =
      xml_path + ":SDF_FORMAT_ARGS:usdMjcfToggleUsdPhysics=true";
  auto stage_with_physics = pxr::UsdStage::Open(xml_path_physics_flag);
  EXPECT_THAT(stage_with_physics, testing::NotNull());

  EXPECT_PRIM_VALID(stage_with_physics, "/mesh_test/test_body/test_body");
  EXPECT_PRIM_API_APPLIED(stage_with_physics, "/mesh_test/test_body/test_body",
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
  EXPECT_PRIM_VALID(stage, "/physics_test/test_body/test_body");
  // USD does not allow nested rigidbodies so we put them as siblings to the
  // first body in the hierarchy.
  EXPECT_PRIM_VALID(stage, "/physics_test/test_body/test_body_2");

  // The parent containing the body should not have the RigidBodyAPI applied.
  EXPECT_PRIM_API_NOT_APPLIED(stage, "/physics_test/test_body",
                              pxr::UsdPhysicsRigidBodyAPI);

  EXPECT_PRIM_API_APPLIED(stage, "/physics_test/test_body/test_body",
                          pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_APPLIED(stage, "/physics_test/test_body/test_body_2",
                          pxr::UsdPhysicsRigidBodyAPI);

  // Geoms should not have RigidBodyAPI applied either.
  EXPECT_PRIM_API_NOT_APPLIED(stage,
                              "/physics_test/test_body/test_body/test_geom",
                              pxr::UsdPhysicsRigidBodyAPI);
  EXPECT_PRIM_API_NOT_APPLIED(stage,
                              "/physics_test/test_body/test_body_2/test_geom_2",
                              pxr::UsdPhysicsRigidBodyAPI);
}

}  // namespace
}  // namespace mujoco
