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

// Tests for user/user_objects.cc.

#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <absl/strings/str_replace.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjCMeshTest = MujocoTest;

static const char* const kMeshPath =
    "user/testdata/mesh.xml";
static const char* const kDuplicateVerticesPath =
    "user/testdata/duplicate_vertices.xml";
static const char* const kCubePath =
    "user/testdata/cube.xml";
static const char* const kCubeCompletePath =
    "user/testdata/cube_complete.obj";
static const char* const kTorusPath =
    "user/testdata/torus.xml";
static const char* const kTorusMaxhullVertPath =
    "user/testdata/torus_maxhullvert.xml";
static const char* const kTorusDefaultMaxhullVertPath =
    "user/testdata/torus_maxhullvert_default.xml";
static const char* const kCompareInertiaPath =
    "user/testdata/inertia_compare.xml";
static const char* const kConvexInertiaPath =
    "user/testdata/inertia_convex.xml";
static const char* const kConcaveInertiaPath =
    "user/testdata/inertia_concave.xml";
static const char* const kShellInertiaPath =
    "user/testdata/inertia_shell.xml";
static const char* const kTorusQuadsPath =
    "user/testdata/torus_quads.xml";
static const char* const kTexturedTorusPath =
    "user/testdata/textured_torus.xml";
static const char* const kDuplicateOBJPath =
    "user/testdata/duplicate.xml";
static const char* const kMalformedFaceOBJPath =
    "user/testdata/malformed_face.xml";
static const char* const kCubeSkinPath =
    "user/testdata/cube_skin.xml";

using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

static constexpr mjtNum kMaxAbsErr = std::numeric_limits<float>::epsilon();

// ------------- test invalid filenames ----------------------------------------

TEST_F(MjCMeshTest, UnknownMeshFormat) {
  static constexpr char xml_format[] = R"(
    <mujoco>
      <asset>
        <mesh name="m" file="%s"/>
      </asset>
      <worldbody>
        <geom type="mesh" mesh="m"/>
      </worldbody>
    </mujoco>
  )";

  std::vector<std::string> invalid_names = {
    "noextension",
    "anobj",
    "f",
    "mesh.exe",
    "file%s"
  };
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  for (const auto& name : invalid_names) {
    mj_addBufferVFS(&vfs, name.c_str(), nullptr, 0);
    std::string xml = absl::StrFormat(xml_format, name);
    std::array<char, 1024> error;
    mjModel* model =
        LoadModelFromString(xml.c_str(), error.data(), error.size(), &vfs);
    ASSERT_THAT(model, testing::IsNull())
        << "Should fail to load a mesh named: " << name;
    EXPECT_THAT(error.data(), HasSubstr("unknown or unsupported mesh file: "));
    EXPECT_THAT(error.data(), HasSubstr(name));
  }

  mj_deleteVFS(&vfs);
}

// -------------------- test OS filesystem fallback ----------------------------

TEST_F(MjCMeshTest, LoadMSHWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" file="unknown_file.msh"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should fallback to OS filesystem
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

TEST_F(MjCMeshTest, LoadOBJWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" file="unknown_file.obj"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should fallback to OS filesystem
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

TEST_F(MjCMeshTest, LoadSTLWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" file="unknown_file.stl"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should fallback to OS filesystem
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

// ------------- test content_type attributes ----------------------------------

TEST_F(MjCMeshTest, LoadMSHWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" content_type="model/vnd.mujoco.msh" file="some_file"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try opening the file (not found obviously)
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

TEST_F(MjCMeshTest, LoadOBJWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" content_type="model/obj" file="some_file"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try opening the file (not found obviously)
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

TEST_F(MjCMeshTest, LoadSTLWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" content_type="model/stl" file="some_file"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try opening the file (not found obviously)
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

TEST_F(MjCMeshTest, LoadMSHWithContentTypeError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" content_type="model/unknown" file="some_file"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "some_file", nullptr, 0);

  // should error with unknown file type
  mjModel* model = LoadModelFromString(xml, error, error_sz, &vfs);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("unsupported mesh type: 'model/unknown'"));
  mj_deleteVFS(&vfs);
}

TEST_F(MjCMeshTest, LoadMSHWithInvalidContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" content_type="model" file="some_file"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "some_file", nullptr, 0);

  // should error with unknown file type
  mjModel* model = LoadModelFromString(xml, error, error_sz, &vfs);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("invalid content type: 'model'"));
  mj_deleteVFS(&vfs);
}

TEST_F(MjCMeshTest, LoadMSHWithContentTypeParam) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh1" content_type="model/vnd.mujoco.msh;parameter=value" file="some_file"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="mesh1"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try opening the file (not found obviously)
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
}

// ------------- test vertex deduplication (STL) ------------------------------

TEST_F(MjCMeshTest, DeduplicateSTLVertices) {
  const std::string xml_path = GetTestDataFilePath(kDuplicateVerticesPath);
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error, error_sz);
  ASSERT_THAT(model, NotNull()) << error;
  ASSERT_EQ(model->nmeshvert, 4);
  mj_deleteModel(model);
}

// -------------------- test Mesh loading (MSH) --------------------------------

TEST_F(MjCMeshTest, LoadMSH) {
  const std::string xml_path = GetTestDataFilePath(kMeshPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  ASSERT_EQ(model->nmeshvert, 36);
  mj_deleteModel(model);
}

// ------------- test OBJ loading ----------------------------------------------

using MjCMeshTest = MujocoTest;

TEST_F(MjCMeshTest, LoadCube) {
  const std::string xml_path = GetTestDataFilePath(kCubePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, nullptr, 0);
  ASSERT_GT(model->ngeom, 0);
  ASSERT_EQ(model->nmeshvert, 8);
  ASSERT_EQ(model->nmeshface, 12);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, LoadTorus) {
  const std::string xml_path = GetTestDataFilePath(kTorusPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_GT(model->ngeom, 0);
  ASSERT_GT(model->nmeshvert, 0);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, LoadTorusQuads) {
  const std::string xml_path = GetTestDataFilePath(kTorusQuadsPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_GT(model->ngeom, 0);
  ASSERT_GT(model->nmeshvert, 0);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, LoadTexturedTorus) {
  const std::string xml_path = GetTestDataFilePath(kTexturedTorusPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_GT(model->ngeom, 0);
  ASSERT_GT(model->nmeshvert, 0);
  ASSERT_GT(model->ntex, 0);
  ASSERT_GT(model->ntexdata, 0);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, KeepDuplicateOBJVertices) {
  const std::string xml_path = GetTestDataFilePath(kDuplicateOBJPath);
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error, error_sz);
  ASSERT_EQ(model->nmeshvert, 16);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, SaveMeshOnce) {
  const std::string xml_path = GetTestDataFilePath(kCubePath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // Confirm the mesh file is loaded and stored in paths
  EXPECT_EQ(
      std::string(&model->paths[model->mesh_pathadr[0]]), "cube.obj");
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(testing::HasSubstr("vertex")));
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, TinyMeshLoads) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tiny" vertex="0 0 0  1e-4 0 0  0 1e-4 0  0 0 1e-4"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="tiny"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mj_deleteModel(model);
}

// ------------- test max hull vert -------------------------------------------
TEST_F(MjCMeshTest, MaxHullVert) {
  const std::string xml_path = GetTestDataFilePath(kTorusMaxhullVertPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_GT(model->ngeom, 0);
  ASSERT_EQ(model->mesh_graph[0], 4);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, MaxHullVertDefault) {
  const std::string xml_path =
      GetTestDataFilePath(kTorusDefaultMaxhullVertPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_GT(model->ngeom, 0);
  ASSERT_EQ(model->mesh_graph[0], 64);
  mj_deleteModel(model);
}

// ------------- test inline loading ------------------------------------------
TEST_F(MjCMeshTest, FaceNormalAutogenerated) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        normal="1 0 0  0 1 0  0 0 1  0.707 0 0.707"
        face="0 2 1  0 3 2" />
    </asset>
    <worldbody>
      <geom type="mesh" mesh="example_mesh"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mj_deleteModel(model);
}

// ------------- test inertia -------------------------------------------------

TEST_F(MjCMeshTest, SmallInertiaLoads) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tiny" vertex="0 0 0  1e-4 0 0  0 1e-4 0  0 0 1e-4"/>
    </asset>
    <worldbody>
      <body>
        <freejoint/>
        <geom type="mesh" mesh="tiny"/>
        <geom name="small" size=".001"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, TinyInertiaFails) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tiny" vertex="0 0 0  1e-4 0 0  0 1e-4 0  0 0 1e-4"/>
    </asset>
    <worldbody>
      <body>
        <freejoint/>
        <geom type="mesh" mesh="tiny"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(
      error.data(),
      HasSubstr(
          "mass and inertia of moving bodies must be larger than mjMINVAL"));
}

TEST_F(MjCMeshTest, FlippedFaceAllowedLegacyInertia) {
  const std::string xml_path = GetTestDataFilePath(kMalformedFaceOBJPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_THAT(model->nmeshface, 4);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, MissingFaceAllowedConvexInertia) {
  const std::string xml_path = GetTestDataFilePath(kCompareInertiaPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_THAT(model->nmeshface, 7);
  EXPECT_NEAR(model->body_inertia[3], model->body_inertia[6], kMaxAbsErr);
  EXPECT_NEAR(model->body_inertia[4], model->body_inertia[7], kMaxAbsErr);
  EXPECT_NEAR(model->body_inertia[5], model->body_inertia[8], kMaxAbsErr);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, FlippedFaceFailsExactInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh" inertia="exact"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="2 0 3  0 1 3  1 2 3  0 1 2" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error.data(), HasSubstr(
        "Error: faces of mesh 'example_mesh' have inconsistent orientation. "
        "Please check the faces containing the vertices 1 and 2."));
}

void CheckTetrahedronWasRescaled(mjModel* model) {
  // The rotated and rescaled positions of the tetrahedron
  // with vertices (0, 0, 0), (1, 0, 0), (0, 2, 0), (0, 0, 3)
  // after mesh preprocessing is performed
  std::vector<mjtNum> vert = {
    -0.51610732078552246,  -0.57402724027633667, -0.5283237099647522,
     0.42337465286254883,  -0.90627568960189819, -0.61189728975296021,
     0.065528042614459991,  1.2306677103042603,  -1.1645441055297852,
     0.027204651385545731,  0.24963514506816864,  2.3047652244567871};
  mjtNum tolerance = std::numeric_limits<float>::epsilon();
  for (int i=0; i < 12; ++i) {
    EXPECT_NEAR(model->mesh_vert[i], vert[i], tolerance);
  }
}

TEST_F(MjCMeshTest, FlippedFaceAllowedWorld) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 2 0  0 0 3"
        face="2 0 3  0 1 3  1 2 3  0 1 2" />
    </asset>
    <worldbody>
      <geom type="mesh" mesh="example_mesh"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  CheckTetrahedronWasRescaled(model);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, FlippedFaceAllowedNoMass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 2 0  0 0 3"
        face="2 0 3  0 1 3  1 2 3  0 1 2" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh" mass="0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  CheckTetrahedronWasRescaled(model);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, FlippedFaceAllowedInertial) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 2 0  0 0 3"
        face="2 0 3  0 1 3  1 2 3  0 1 2" />
    </asset>
    <worldbody>
      <body>
        <inertial pos="0 0 0" mass="1"/>
        <geom type="mesh" mesh="example_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  CheckTetrahedronWasRescaled(model);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, FlippedFaceAllowedNegligibleArea) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 2 0  0 0 3  0 0 3"
        face="2 0 3  0 1 3  1 2 3  0 2 1  0 3 4" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  CheckTetrahedronWasRescaled(model);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, AreaTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1e-8 0 0  0 1e-8 0  0 0 1e-8"
        face="2 0 3  0 1 3  1 2 3  0 2 1" inertia="shell"/>
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error.data(), HasSubstr("mesh surface area is too small"));
}

TEST_F(MjCMeshTest, VolumeTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="0 2 1" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error.data(), HasSubstr("mesh volume is too small"));
  mj_deleteModel(model);

}

TEST_F(MjCMeshTest, VisualVolumeTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" mass="0"/>
      </default>
    </default>
    <asset>
      <mesh name="example_mesh"
        vertex="0 -4e-16 0  1 0 4e-16  0 1 0  0 0 1"
        face="0 2 1" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh" class="visual"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error.data(), HasSubstr("mesh volume is too small"));
  mj_deleteModel(model);

}

TEST_F(MjCMeshTest, VisualVolumeSmallAllowedShell) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" mass="0"/>
      </default>
    </default>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  1 1 1e-6"
        face="0 1 2  2 1 3" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh" class="visual"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_LE(mju_abs(model->geom_size[0]), 1);
  EXPECT_LE(mju_abs(model->geom_size[1]), 1);
  EXPECT_LE(mju_abs(model->geom_size[2]), 1);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, VolumeSmallAllowedShell) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  1 1 1e-6"
        face="0 1 2  2 1 3" inertia="shell"/>
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="example_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_LE(mju_abs(model->geom_size[0]), 1);
  EXPECT_LE(mju_abs(model->geom_size[1]), 1);
  EXPECT_LE(mju_abs(model->geom_size[2]), 1);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, VolumeNegativeThrowsError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      MESH_DEFINITIONS
    </asset>
    <worldbody>
      <body>
        GEOM_DEFINITIONS
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char bad_mesh[] = R"(
      <mesh name="bad_mesh%d" inertia="exact"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="3 0 2  0 3 1  1 3 2  0 1 2"/>\n"
  )";
  static constexpr char geom[] = R"(<geom type="mesh" mesh="bad_mesh%d"/>\n)";

  for (int nmesh : {3, 16, 17, 50}) {
    std::string mesh_definitions = "";
    for (int i = 1; i < nmesh+1; i++) {
      mesh_definitions += absl::StrFormat(bad_mesh, i);
    }

    std::string geom_definitions = "";
    for (int i = 1; i < nmesh+1; i++) {
      geom_definitions += absl::StrFormat(geom, i);
    }

    std::string xml_str = xml;
    absl::StrReplaceAll({{"MESH_DEFINITIONS", mesh_definitions},
                         {"GEOM_DEFINITIONS", geom_definitions}}, &xml_str);

    std::array<char, 1024> error;
    mjModel* model = LoadModelFromString(xml_str.c_str(),
                                         error.data(), error.size());
    EXPECT_THAT(model, IsNull());
    EXPECT_THAT(error.data(), HasSubstr("mesh volume is negative"));
  }
}

TEST_F(MjCMeshTest, MeshIgnoresDefaultDensity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <geom density="0" />
    </default>
    <asset>
      <mesh name="a" vertex="0 0 0 1 0 0 0 1 0 0 0 1" scale="10 10 10" />
    </asset>
    <worldbody/>
  </mujoco>)";
  char error[1024];
  mjSpec* spec = mj_parseXMLString(xml, 0, error, sizeof(error));
  EXPECT_THAT(spec, NotNull()) << error;
  mjModel* m1 = mj_compile(spec, nullptr);
  EXPECT_THAT(m1, NotNull());
  mj_deleteModel(m1);
  mjModel* m2 = mj_compile(spec, nullptr);
  EXPECT_THAT(m2, NotNull());
  mj_deleteModel(m2);
  mj_deleteSpec(spec);
}

// ------------- test concave and shell inertia --------------------------------

TEST_F(MjCMeshTest, ExactConcaveInertia) {
  const std::string xml_path = GetTestDataFilePath(kConcaveInertiaPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // analytic computation of 1x1x1 cube with a .8x.8x.9 hole
  // see https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  mjtNum density = 2.;
  mjtNum m_hole = .9 * .8 * .8 * density;
  mjtNum m_cube = 1. * density;
  mjtNum m_concave_cube = m_cube - m_hole;
  mjtNum I_cube = m_cube/6.;
  // due to the asymmetric hole, the com position has changed
  // so we need to use https://en.wikipedia.org/wiki/Parallel_axis_theorem
  mjtNum d_cube = .5 - model->body_ipos[5];
  mjtNum d_hole = .55 - model->body_ipos[5];
  mjtNum I1 = I_cube - m_hole*(.8*.8 + .8*.8)/12;
  mjtNum I2 = I_cube - m_hole*(.8*.8 + .9*.9)/12
            + m_cube*d_cube*d_cube - m_hole*d_hole*d_hole;
  EXPECT_NEAR(model->body_mass[1], m_concave_cube, kMaxAbsErr);
  EXPECT_NEAR(model->body_mass[2], m_concave_cube, kMaxAbsErr);
  EXPECT_NEAR(model->body_mass[3], m_concave_cube, kMaxAbsErr);
  EXPECT_NEAR(model->body_mass[4], m_concave_cube, kMaxAbsErr);
  for (int i = 3; i < 15; i += 3) {
    EXPECT_NEAR(model->body_inertia[i], I1, kMaxAbsErr);
    EXPECT_NEAR(model->body_inertia[i+1], I2, kMaxAbsErr);
    EXPECT_NEAR(model->body_inertia[i+2], I2, kMaxAbsErr);
  }
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, ExactConvexInertia) {
  const std::string xml_path = GetTestDataFilePath(kConvexInertiaPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  mjtNum m_solid_cube = 1.;
  mjtNum I_solid_cube = 1./6. * m_solid_cube;
  EXPECT_LE(mju_abs(model->body_mass[1] - m_solid_cube), kMaxAbsErr);
  EXPECT_LE(mju_abs(model->body_mass[2] - m_solid_cube), kMaxAbsErr);
  for (int i = 3; i < 9; i++) {
    EXPECT_LE(mju_abs(model->body_inertia[i] - I_solid_cube), kMaxAbsErr);
  }
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, ExactShellInertia) {
  const std::string xml_path = GetTestDataFilePath(kShellInertiaPath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // see https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  mjtNum m_hollow_cube = 6.;
  mjtNum I_hollow_cube = 5./18. * m_hollow_cube;
  EXPECT_LE(mju_abs(model->body_mass[1] - m_hollow_cube), kMaxAbsErr);
  EXPECT_LE(mju_abs(model->body_inertia[3] - I_hollow_cube), kMaxAbsErr);
  EXPECT_LE(mju_abs(model->body_inertia[4] - I_hollow_cube), kMaxAbsErr);
  EXPECT_LE(mju_abs(model->body_inertia[5] - I_hollow_cube), kMaxAbsErr);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, MeshPosQuat) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="pyramid" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>
    <worldbody>
      <geom type="mesh" name="geom1" mesh="pyramid"/>
      <geom type="mesh" name="geom2" pos="1 2 3" quat="0.5 0.5 0.5 0.5" mesh="pyramid"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  // loading the mesh results in an offset of the geom's pos and quat due to the
  // fact that the geom's center is not the volumetric center of the mesh. To
  // recover the geom's originally specified pose, the offset used is stored in
  // mesh_pos and mesh_quat. In order to recover the originally specified pose
  // and orientation, first invert the specified mesh_pos and mesh_quat
  mjtNum inverse_mesh_pos[3];
  mjtNum inverse_mesh_quat[4];
  mju_negPose(inverse_mesh_pos, inverse_mesh_quat,
              &model->mesh_pos[0], &model->mesh_quat[0]);

  // apply the inverted mesh_pos and inverted mesh_quat to the geom's pos and
  // quat. It should match the originally specified values
  mjtNum recovered_pos[3];
  mjtNum recovered_quat[4];
  mju_mulPose(recovered_pos, recovered_quat,
              &model->geom_pos[0], &model->geom_quat[0],
              inverse_mesh_pos, inverse_mesh_quat);
  EXPECT_NEAR(recovered_pos[0], 0, 1e-12);
  EXPECT_NEAR(recovered_pos[1], 0, 1e-12);
  EXPECT_NEAR(recovered_pos[2], 0, 1e-12);

  EXPECT_NEAR(recovered_quat[0], 1, 1e-12);
  EXPECT_NEAR(recovered_quat[1], 0, 1e-12);
  EXPECT_NEAR(recovered_quat[2], 0, 1e-12);
  EXPECT_NEAR(recovered_quat[3], 0, 1e-12);

  // same test on the other geom
  mju_negPose(inverse_mesh_pos, inverse_mesh_quat,
              &model->mesh_pos[0], &model->mesh_quat[0]);
  mju_mulPose(recovered_pos, recovered_quat,
              &model->geom_pos[3], &model->geom_quat[4],
              inverse_mesh_pos, inverse_mesh_quat);
  EXPECT_NEAR(recovered_pos[0], 1, 1e-12);
  EXPECT_NEAR(recovered_pos[1], 2, 1e-12);
  EXPECT_NEAR(recovered_pos[2], 3, 1e-12);

  EXPECT_NEAR(recovered_quat[0], 0.5, 1e-12);
  EXPECT_NEAR(recovered_quat[1], 0.5, 1e-12);
  EXPECT_NEAR(recovered_quat[2], 0.5, 1e-12);
  EXPECT_NEAR(recovered_quat[3], 0.5, 1e-12);

  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, MeshPosQuatShellInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="pyramid" vertex="0 0 0  1 0 0  0 1 0  0 0 1" inertia="shell"/>
    </asset>
    <worldbody>
      <geom type="mesh" name="geom1" mesh="pyramid"/>
      <geom type="mesh" name="geom2" pos="1 2 3" quat="0.5 0.5 0.5 0.5" mesh="pyramid"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  // loading the mesh results in an offset of the geom's pos and quat due to the
  // fact that the geom's center is not the volumetric center of the mesh. To
  // recover the geom's originally specified pose, the offset used is stored in
  // mesh_pos and mesh_quat. In order to recover the originally specified pose
  // and orientation, first invert the specified mesh_pos and mesh_quat
  mjtNum inverse_mesh_pos[3];
  mjtNum inverse_mesh_quat[4];
  mju_negPose(inverse_mesh_pos, inverse_mesh_quat,
              &model->mesh_pos[0], &model->mesh_quat[0]);

  // apply the inverted mesh_pos and inverted mesh_quat to the geom's pos and
  // quat. It should match the originally specified values
  mjtNum recovered_pos[3];
  mjtNum recovered_quat[4];
  mju_mulPose(recovered_pos, recovered_quat,
              &model->geom_pos[0], &model->geom_quat[0],
              inverse_mesh_pos, inverse_mesh_quat);
  EXPECT_NEAR(recovered_pos[0], 0, 1e-12);
  EXPECT_NEAR(recovered_pos[1], 0, 1e-12);
  EXPECT_NEAR(recovered_pos[2], 0, 1e-12);

  EXPECT_NEAR(recovered_quat[0], 1, 1e-12);
  EXPECT_NEAR(recovered_quat[1], 0, 1e-12);
  EXPECT_NEAR(recovered_quat[2], 0, 1e-12);
  EXPECT_NEAR(recovered_quat[3], 0, 1e-12);

  // same test on the other geom
  mju_negPose(inverse_mesh_pos, inverse_mesh_quat,
              &model->mesh_pos[0], &model->mesh_quat[0]);
  mju_mulPose(recovered_pos, recovered_quat,
              &model->geom_pos[3], &model->geom_quat[4],
              inverse_mesh_pos, inverse_mesh_quat);
  EXPECT_NEAR(recovered_pos[0], 1, 1e-12);
  EXPECT_NEAR(recovered_pos[1], 2, 1e-12);
  EXPECT_NEAR(recovered_pos[2], 3, 1e-12);

  EXPECT_NEAR(recovered_quat[0], 0.5, 1e-12);
  EXPECT_NEAR(recovered_quat[1], 0.5, 1e-12);
  EXPECT_NEAR(recovered_quat[2], 0.5, 1e-12);
  EXPECT_NEAR(recovered_quat[3], 0.5, 1e-12);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, MeshScale) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="pyramid" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="pyramid_scaled" vertex="0 0 0  1 0 0  0 1 0  0 0 1" scale="0.9 1 -1"/>
    </asset>
    <worldbody>
      <geom type="mesh" name="geom1" mesh="pyramid"/>
      <geom type="mesh" name="geom2" mesh="pyramid_scaled"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_THAT(AsVector(model->mesh_scale + 0, 3), ElementsAre(1, 1, 1));
  EXPECT_THAT(AsVector(model->mesh_scale + 3, 3), ElementsAre(0.9, 1, -1));
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, ShellInertiaTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="pyramid" vertex="0 0 0  1 0 0  0 1 0  0 0 1" inertia="shell"/>
      <mesh name="pyramid_scaled" vertex="0 0 0  1 0 0  0 1 0  0 0 1" scale="0.9 1 -1"/>
    </asset>
    <worldbody>
      <geom type="mesh" name="geom1" mesh="pyramid"/>
      <geom type="mesh" name="geom2" mesh="pyramid_scaled"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_THAT(AsVector(model->mesh_scale + 0, 3), ElementsAre(1, 1, 1));
  EXPECT_THAT(AsVector(model->mesh_scale + 3, 3), ElementsAre(0.9, 1, -1));
  mj_deleteModel(model);
}

// ----------------------------- texcoord -------------------------------------

TEST_F(MjCMeshTest, CreateFaceTexCoord) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            texcoord="0 0  0 0  0 0  0 0"/>
      </asset>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, UseFaceTexCoord) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  2 0 3  1 2 3"
            texcoord="0 0  .1 .1  .2 .2  .3 .3"/>
      </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 0]], .0);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 1]], .2);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 2]], .1);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 3]], .0);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 4]], .1);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 5]], .3);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 6]], .2);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 7]], .0);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 8]], .3);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[ 9]], .1);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[10]], .2);
  EXPECT_FLOAT_EQ(model->mesh_texcoord[2*model->mesh_facetexcoord[11]], .3);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, MissingTexCoord) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            texcoord="0 0"/>
      </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error.data(), HasSubstr("texcoord must be 2*nv"));
}

// ----------------------------- qhull ----------------------------------------

TEST_F(MjCMeshTest, NaNConvexHullDisallowed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh"vertex="nan 0 0  0 1 0  0 1 0  0 0 1"/>
    </asset>
  </mujoco>
    )";
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, testing::IsNull());
  EXPECT_THAT(error.data(), HasSubstr("vertex coordinate 0 is not finite"));
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, InvalidIndexInFace) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        normal="1 0 0  0 1 0  0 0 1  0.707 0 0.707"
        face="0 2 6  0 3 2" />
    </asset>
    <worldbody>
      <geom type="mesh" mesh="example_mesh"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("in face 0, vertex index 6 does not exist"));
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, QhullCache) {
  static constexpr char xml1[] = R"(
    <mujoco>
      <asset>
        <mesh name="box" file="cube_complete.obj"/>
      </asset>
      <worldbody>
        <geom type="mesh" pos="0 0 2" mesh="box" contype="0" conaffinity="0"/>
        <geom type="mesh" pos="0 0 0" mesh="box" contype="0" conaffinity="0"/>
      </worldbody>
    </mujoco>)";

    static constexpr char xml2[] = R"(
      <mujoco>
        <asset>
          <mesh name="box" file="cube_complete.obj"/>
        </asset>
        <worldbody>
          <geom type="mesh" pos="0 0 2" mesh="box"/>
          <geom type="mesh" pos="0 0 0" mesh="box"/>
        </worldbody>
      </mujoco>)";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addFileVFS(&vfs, "", GetTestDataFilePath(kCubeCompletePath).c_str());

  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml1, error.data(), error.size(), &vfs);
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  EXPECT_THAT(model->mesh_graphadr[0], -1);

  mj_deleteModel(model);

  model = LoadModelFromString(xml2, error.data(), error.size(), &vfs);
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  EXPECT_GT(model->mesh_graphadr[0], -1);

  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
}

TEST_F(MjCMeshTest, LoadSkin) {
  const std::string xml_path = GetTestDataFilePath(kCubeSkinPath);
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << error.data();
  mjModel* m1 = mj_compile(spec, 0);
  EXPECT_THAT(m1, NotNull());
  mj_deleteModel(m1);
  mjModel* m2 = mj_compile(spec, 0);
  EXPECT_THAT(m2, NotNull());
  mj_deleteModel(m2);
  mj_deleteSpec(spec);
}


}  // namespace
}  // namespace mujoco
