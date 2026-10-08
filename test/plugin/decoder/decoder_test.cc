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

// Tests for decoder plugins.

#include <string.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// A simple mjSpec with one body and one geom.
static mjSpec* MakeSimpleSpec() {
  mjSpec* s = mj_makeSpec();
  mjsBody* world = mjs_findBody(s, "world");
  mjsBody* body = mjs_addBody(world, nullptr);
  mjsGeom* geom = mjs_addGeom(body, nullptr);
  geom->size[0] = 1.0;
  geom->size[1] = 1.0;
  geom->size[2] = 1.0;
  return s;
}

// Always returns a simple mjSpec, ignoring the resource.
mjSpec* FakeDecode(mjResource* resource, const mjVFS* vfs, char* error,
                   int error_sz) {
  return MakeSimpleSpec();
}

// Can decode any resource that has a .fakeformat extension.
int FakeCanDecode(const mjResource* resource) {
  const char* ext = strrchr(resource->name, '.');
  if (ext) {
    return strcmp(ext, ".fakeformat") == 0 ||
           strcmp(ext, ".alsoFakeFormat") == 0;
  }
  return 0;
}

mjpDecoder FakeDecoder() {
  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/fakeformat";
  decoder.extension = ".fakeformat|.alsoFakeFormat";
  decoder.can_decode = FakeCanDecode;
  decoder.decode = FakeDecode;
  return decoder;
}

using DecoderPluginTest = MujocoTest;

TEST_F(DecoderPluginTest, CanDecode) {
  mjpDecoder decoder = FakeDecoder();
  mjp_registerDecoder(&decoder);

  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <model name="fakeformat" file="dummy.fakeformat"/>
      <model name="also_fakeformat" file="dummy.alsoFakeFormat"/>
    </asset>
    <worldbody>
      <attach model="fakeformat" prefix="test"/>
    </worldbody>
  </mujoco>
  )";

  // Check referencing a resource via XML invokes the decoder.
  char error[1024];
  mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
  mjModel* model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);

  // Check mj_parse with extension .fakeformat
  spec = mj_parse("dummy.fakeformat", nullptr, nullptr, error, sizeof(error));
  model = mj_compile(spec, nullptr);
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);

  // Check mj_parse with extension .alsoFakeFormat
  spec =
      mj_parse("dummy.alsoFakeFormat", nullptr, nullptr, error, sizeof(error));
  model = mj_compile(spec, nullptr);
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);

  // Check mj_parse with content_type
  spec = mj_parse("dummy.fakeformat", "model/fakeformat", nullptr, error,
                  sizeof(error));
  model = mj_compile(spec, nullptr);
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST_F(DecoderPluginTest, DecodeWithResourceArgs) {
  static auto decode_args_fn = +[](mjResource* resource, const mjVFS* vfs,
                                   char* error, int error_sz) -> mjSpec* {
    mjSpec* s = MakeSimpleSpec();
    if (resource && resource->args) {
      std::string_view args_view(resource->args);
      size_t pos = args_view.find("size=");
      if (pos != std::string_view::npos) {
        mjsElement* elem = mjs_firstElement(s, mjOBJ_GEOM);
        mjsGeom* geom = mjs_asGeom(elem);
        if (geom) {
          geom->size[0] = std::atof(args_view.data() + pos + 5);
        }
      }
    }
    return s;
  };

  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/argsformat";
  decoder.extension = ".argsformat";
  decoder.can_decode = +[](const mjResource* r) -> int {
    return std::string_view(r->name).ends_with(".argsformat");
  };
  decoder.decode = decode_args_fn;
  mjp_registerDecoder(&decoder);

  mjResource resource;
  std::memset(&resource, 0, sizeof(resource));
  resource.name = const_cast<char*>("test.argsformat");
  resource.args = "size=42.0&foo=bar";

  mjSpec* spec =
      mju_decodeResource(&resource, "model/argsformat", nullptr, nullptr, 0);
  ASSERT_THAT(spec, testing::NotNull());
  mjsElement* elem = mjs_firstElement(spec, mjOBJ_GEOM);
  mjsGeom* geom = mjs_asGeom(elem);
  ASSERT_THAT(geom, testing::NotNull());
  EXPECT_DOUBLE_EQ(geom->size[0], 42.0);
  mj_deleteSpec(spec);
}

// Always fails with a descriptive error.
mjSpec* FailDecode(mjResource* resource, const mjVFS* vfs, char* error,
                   int error_sz) {
  if (error && error_sz > 0) {
    std::snprintf(error, error_sz, "failformat: bad thing in '%s'",
                  resource->name);
  }
  return nullptr;
}

mjpDecoder FailDecoder() {
  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/failformat";
  decoder.extension = ".failformat";
  decoder.can_decode = +[](const mjResource* r) -> int {
    return std::string_view(r->name).ends_with(".failformat");
  };
  decoder.decode = FailDecode;
  return decoder;
}

TEST_F(DecoderPluginTest, DecodeErrorReachesCaller) {
  mjpDecoder decoder = FailDecoder();
  mjp_registerDecoder(&decoder);

  // mju_decodeResource forwards the decoder's error
  mjResource resource;
  std::memset(&resource, 0, sizeof(resource));
  resource.name = const_cast<char*>("test.failformat");
  char error[1024] = "";
  mjSpec* spec = mju_decodeResource(&resource, "model/failformat", nullptr,
                                    error, sizeof(error));
  EXPECT_THAT(spec, testing::IsNull());
  EXPECT_STREQ(error, "failformat: bad thing in 'test.failformat'");

  // a null error buffer is allowed
  spec = mju_decodeResource(&resource, "model/failformat", nullptr, nullptr, 0);
  EXPECT_THAT(spec, testing::IsNull());

  // missing decoder is reported through the error buffer
  resource.name = const_cast<char*>("test.nosuchformat");
  spec = mju_decodeResource(&resource, "model/nosuchformat", nullptr, error,
                            sizeof(error));
  EXPECT_THAT(spec, testing::IsNull());
  EXPECT_THAT(error, testing::HasSubstr("could not find decoder"));

  // mj_parse reports the decoder's error instead of a generic message
  spec = mj_parse("test.failformat", "model/failformat", nullptr, error,
                  sizeof(error));
  EXPECT_THAT(spec, testing::IsNull());
  EXPECT_STREQ(error, "failformat: bad thing in 'test.failformat'");
}

TEST_F(DecoderPluginTest, MeshDecodeErrorReachesCompiler) {
  mjpDecoder decoder = FailDecoder();
  mjp_registerDecoder(&decoder);

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  const char contents[] = "garbage";
  mj_addBufferVFS(&vfs, "bad.failformat", contents, sizeof(contents));

  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="bad" file="bad.failformat" content_type="model/failformat"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="bad"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = "";
  mjSpec* spec = mj_parseXMLString(xml, &vfs, error, sizeof(error));
  ASSERT_THAT(spec, testing::NotNull()) << error;
  mjModel* model = mj_compile(spec, &vfs);
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(mjs_getError(spec),
              testing::HasSubstr("failformat: bad thing in 'bad.failformat'"));
  mj_deleteSpec(spec);
  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshDecoder41) {
  static constexpr char msh[] =
      "$MeshFormat\n"
      "4.1 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "1 4 1 4\n"
      "3 1 0 4\n"
      "1\n2\n3\n4\n"
      "0 0 0\n"
      "1 0 0\n"
      "0 1 0\n"
      "0 0 1\n"
      "$EndNodes\n"
      "$Elements\n"
      "1 1 1 1\n"
      "3 1 4 1\n"
      "1 1 2 3 4\n"
      "$EndElements\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.msh", msh, sizeof(msh) - 1);

  mjResource* resource =
      mju_openResource("", "tetrahedron.msh", &vfs, nullptr, 0);
  ASSERT_THAT(resource, testing::NotNull());

  const mjpDecoder* decoder = mjp_findDecoder(resource, "model/vnd.gmsh");
  ASSERT_THAT(decoder, testing::NotNull());
  EXPECT_TRUE(decoder->can_decode(resource));

  mjSpec* spec = decoder->decode(resource, &vfs, nullptr, 0);
  ASSERT_THAT(spec, testing::NotNull());

  // Verify no flex is created in the decoder
  EXPECT_THAT(mjs_firstElement(spec, mjOBJ_FLEX), testing::IsNull());

  // Check mesh specification
  mjsElement* mesh_elem = mjs_firstElement(spec, mjOBJ_MESH);
  ASSERT_THAT(mesh_elem, testing::NotNull());
  mjsMesh* mesh = mjs_asMesh(mesh_elem);
  ASSERT_THAT(mesh, testing::NotNull());
  EXPECT_EQ(mesh->usernode->size(), 12);
  EXPECT_EQ(mesh->uservert->size(), 12);
  EXPECT_EQ(mesh->usertet->size(), 4);
  EXPECT_EQ(mesh->userface->size(), 12);

  mj_deleteSpec(spec);
  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshDecoder22) {
  static constexpr char msh[] =
      "$MeshFormat\n"
      "2.2 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "4\n"
      "1 0 0 0\n"
      "2 1 0 0\n"
      "3 0 1 0\n"
      "4 0 0 1\n"
      "$EndNodes\n"
      "$Elements\n"
      "1\n"
      "1 4 0 1 2 3 4\n"
      "$EndElements\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron22.msh", msh, sizeof(msh) - 1);

  mjResource* resource =
      mju_openResource("", "tetrahedron22.msh", &vfs, nullptr, 0);
  ASSERT_THAT(resource, testing::NotNull());

  const mjpDecoder* decoder = mjp_findDecoder(resource, "");
  ASSERT_THAT(decoder, testing::NotNull());
  EXPECT_TRUE(decoder->can_decode(resource));

  mjSpec* spec = decoder->decode(resource, &vfs, nullptr, 0);
  ASSERT_THAT(spec, testing::NotNull());

  // Verify no flex is created in the decoder
  EXPECT_THAT(mjs_firstElement(spec, mjOBJ_FLEX), testing::IsNull());

  mjsElement* mesh_elem = mjs_firstElement(spec, mjOBJ_MESH);
  ASSERT_THAT(mesh_elem, testing::NotNull());
  mjsMesh* mesh = mjs_asMesh(mesh_elem);
  ASSERT_THAT(mesh, testing::NotNull());
  EXPECT_EQ(mesh->usernode->size(), 12);
  EXPECT_EQ(mesh->uservert->size(), 12);
  EXPECT_EQ(mesh->usertet->size(), 4);
  EXPECT_EQ(mesh->userface->size(), 12);

  mj_deleteSpec(spec);
  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshDecoderInvalid) {
  static constexpr char msh[] =
      "$MeshFormat\n"
      "4.1 0 8\n"
      "$EndMeshFormat\n"
      "$Elements\n"
      "1 1 1 1\n"
      "$EndElements\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "invalid.msh", msh, sizeof(msh) - 1);

  mjResource* resource = mju_openResource("", "invalid.msh", &vfs, nullptr, 0);
  ASSERT_THAT(resource, testing::NotNull());

  const mjpDecoder* decoder = mjp_findDecoder(resource, "model/vnd.gmsh");
  ASSERT_THAT(decoder, testing::NotNull());

  char error[1024] = "";
  mjSpec* spec = decoder->decode(resource, &vfs, error, sizeof(error));
  EXPECT_THAT(spec, testing::IsNull());
  EXPECT_STREQ(error, "GMSH decoder: GMSH file missing $Nodes");

  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshDecoder1DUnsupported) {
  static constexpr char msh[] =
      "$MeshFormat\n"
      "4.1 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "1 2 1 2\n"
      "1 1 0 2\n"
      "1\n2\n"
      "0 0 0\n"
      "1 0 0\n"
      "$EndNodes\n"
      "$Elements\n"
      "1 1 1 1\n"
      "1 1 1 1\n"
      "1 1 2\n"
      "$EndElements\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "line.msh", msh, sizeof(msh) - 1);

  mjResource* resource = mju_openResource("", "line.msh", &vfs, nullptr, 0);
  ASSERT_THAT(resource, testing::NotNull());

  const mjpDecoder* decoder = mjp_findDecoder(resource, "model/vnd.gmsh");
  ASSERT_THAT(decoder, testing::NotNull());

  char error[1024] = "";
  mjSpec* spec = decoder->decode(resource, &vfs, error, sizeof(error));
  EXPECT_THAT(spec, testing::IsNull());
  EXPECT_STREQ(error, "GMSH decoder: 1D meshes are not supported");

  mju_closeResource(resource);
  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshMeshInModel) {
  static constexpr char msh[] =
      "$MeshFormat\n"
      "2.2 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "4\n"
      "1 0 0 0\n"
      "2 1 0 0\n"
      "3 0 1 0\n"
      "4 0 0 1\n"
      "$EndNodes\n"
      "$Elements\n"
      "1\n"
      "1 4 0 1 2 3 4\n"
      "$EndElements\n";

  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tetmesh" content_type="model/vnd.gmsh" file="tetrahedron.msh"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="tetmesh"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.msh", msh, sizeof(msh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nmesh, 1);
  EXPECT_EQ(model->nmeshvert, 4);
  EXPECT_EQ(model->nmeshface, 4);

  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshMeshFromCube41File) {
  const std::string msh_path =
      GetTestDataFilePath("user/testdata/cube_41_ascii_vol_gmshApp.msh");
  std::string xml =
      "<mujoco>\n"
      "  <asset>\n"
      "    <mesh name=\"cube\" content_type=\"model/vnd.gmsh\" file=\"" +
      msh_path +
      "\"/>\n"
      "  </asset>\n"
      "  <worldbody>\n"
      "    <geom type=\"mesh\" mesh=\"cube\"/>\n"
      "  </worldbody>\n"
      "</mujoco>\n";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nmesh, 1);
  EXPECT_GT(model->nmeshvert, 0);
  EXPECT_GT(model->nmeshface, 0);
}

TEST_F(DecoderPluginTest, GmshMeshBoundaryCompactionWithInteriorNode) {
  // 5 nodes: 4 outer tetrahedron vertices + 1 interior node.
  // 4 tetrahedra connecting the interior node to each outer triangular face.
  // The boundary contains only the 4 outer vertices and 4 outer faces;
  // node 5 is interior and pruned during compaction.
  static constexpr char msh[] =
      "$MeshFormat\n"
      "2.2 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "5\n"
      "1 0 0 0\n"
      "2 1 0 0\n"
      "3 0 1 0\n"
      "4 0 0 1\n"
      "5 0.25 0.25 0.25\n"
      "$EndNodes\n"
      "$Elements\n"
      "4\n"
      "1 4 0 1 2 3 5\n"
      "2 4 0 1 2 5 4\n"
      "3 4 0 1 5 3 4\n"
      "4 4 0 5 2 3 4\n"
      "$EndElements\n";

  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="compacted_mesh" content_type="model/vnd.gmsh" file="compact.msh"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="compacted_mesh"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "compact.msh", msh, sizeof(msh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nmesh, 1);
  // 5 total nodes, but only 4 on boundary -> nmeshvert < node count
  EXPECT_EQ(model->nmeshvert, 4);
  EXPECT_EQ(model->nmeshface, 4);

  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshMeshOutOfRangeElementIndex) {
  // Element references node 99 which does not exist (only 4 nodes defined)
  static constexpr char msh[] =
      "$MeshFormat\n"
      "2.2 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "4\n"
      "1 0 0 0\n"
      "2 1 0 0\n"
      "3 0 1 0\n"
      "4 0 0 1\n"
      "$EndNodes\n"
      "$Elements\n"
      "1\n"
      "1 4 0 1 2 3 99\n"
      "$EndElements\n";

  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="bad_mesh" content_type="model/vnd.gmsh" file="bad.msh"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="bad_mesh"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "bad.msh", msh, sizeof(msh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error, testing::HasSubstr("GMSH decoder: Invalid node tag"));

  mj_deleteVFS(&vfs);
}

TEST_F(DecoderPluginTest, GmshFlexcompInModel) {
  static constexpr char msh[] =
      "$MeshFormat\n"
      "2.2 0 8\n"
      "$EndMeshFormat\n"
      "$Nodes\n"
      "4\n"
      "1 0 0 0\n"
      "2 1 0 0\n"
      "3 0 1 0\n"
      "4 0 0 1\n"
      "$EndNodes\n"
      "$Elements\n"
      "1\n"
      "1 4 0 1 2 3 4\n"
      "$EndElements\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.msh", msh, sizeof(msh) - 1);

  // 1. Success with dim="3"
  static constexpr char xml_dim3[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="tet" type="gmsh" dim="3" radius=".001"
                file="tetrahedron.msh"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml_dim3, error, sizeof(error), &vfs);
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 3);
  EXPECT_EQ(model->nflexelem, 1);
  EXPECT_EQ(model->nflexvert, 4);

  // 2. Success with omitted dim (inferred as 3)
  static constexpr char xml_nodim[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="tet" type="gmsh" radius=".001"
                file="tetrahedron.msh"/>
    </worldbody>
  </mujoco>
  )";
  model = LoadModelFromString(xml_nodim, error, sizeof(error), &vfs);
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 3);

  // 3. Error with dim="2" (mismatch with 3D mesh)
  static constexpr char xml_dim2[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="tet" type="gmsh" dim="2" radius=".001"
                file="tetrahedron.msh"/>
    </worldbody>
  </mujoco>
  )";
  model = LoadModelFromString(xml_dim2, error, sizeof(error), &vfs);
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error,
              testing::HasSubstr(
                  "flexcomp dim does not match GMSH mesh dimensionality"));

  mj_deleteVFS(&vfs);
}

}  // namespace
}  // namespace mujoco
