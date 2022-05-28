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
#include <ostream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjCMeshTest = MujocoTest;

static const char* const kDuplicateVerticesPath =
    "user/testdata/duplicate_vertices.xml";
static const char* const kCubePath =
    "user/testdata/cube.xml";
static const char* const kTorusPath =
    "user/testdata/torus.xml";
static const char* const kTorusQuadsPath =
    "user/testdata/torus_quads.xml";
static const char* const kTexturedTorusPath =
    "user/testdata/textured_torus.xml";
static const char* const kDuplicateOBJPath =
    "user/testdata/duplicate.xml";

using ::testing::HasSubstr;

// ------------- test vertex de-duplication (STL) ------------------------------

TEST_F(MjCMeshTest, DeDuplicateSTLVertices) {
  const std::string xml_path = GetTestDataFilePath(kDuplicateVerticesPath);
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error, error_sz);
  ASSERT_EQ(model->nmeshvert, 4);
  mj_deleteModel(model);
}

// ------------- test OBJ loading ----------------------------------------------

using MjCMeshTest = MujocoTest;

TEST_F(MjCMeshTest, LoadCube) {
  const std::string xml_path = GetTestDataFilePath(kCubePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, nullptr, 0);
  ASSERT_GT(model->ngeom, 0);
  ASSERT_EQ(model->nmeshvert, 8);
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
  ASSERT_EQ(model->nmeshvert, 12);
  mj_deleteModel(model);
}

TEST_F(MjCMeshTest, SaveMeshOnce) {
  const std::string xml_path = GetTestDataFilePath(kCubePath);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(testing::HasSubstr("vertex")));
  mj_deleteModel(model);
}

TEST_F(MujocoTest, TinyMeshLoads) {
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
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, testing::NotNull());
  mj_deleteModel(model);
}

TEST_F(MujocoTest, SmallInertiaLoads) {
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
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, testing::NotNull());
  mj_deleteModel(model);
}

TEST_F(MujocoTest, TinyInertiaFails) {
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

}  // namespace
}  // namespace mujoco
