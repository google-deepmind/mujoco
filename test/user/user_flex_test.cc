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

// Tests for user/user_model.cc.

#include <array>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"



namespace mujoco {
namespace {

using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::HasSubstr;
using UserFlexTest = MujocoTest;




TEST_F(UserFlexTest, ParentMustHaveName) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("required attribute missing: 'name'"));
}

TEST_F(UserFlexTest, InvalidDim) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" dim="4"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Invalid dim, must be between 1 and 3"));
}

TEST_F(UserFlexTest, CountTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" count="2 2 0"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Count too small"));
}

TEST_F(UserFlexTest, SpacingGreaterThanGeometry) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" spacing="0.5 0.5 0.5" radius="1"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Spacing must be larger than geometry size"));
}

TEST_F(UserFlexTest, ScaleMinValue) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" scale="0 1 1"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Scale must be larger than mjMINVAL"));
}

TEST_F(UserFlexTest, MassMinValue) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" mass="0"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Mass and inertiabox must be larger than mjMINVAL"));
}

TEST_F(UserFlexTest, PointSizeNotMultipleOf3) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" point="0 0 0 0"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Point size must be a multiple of 3"));
}

TEST_F(UserFlexTest, ElementSize) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" point="0 0 0" element="0 1 2" dim="3"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Element size must be a multiple of dim+1"));
}

TEST_F(UserFlexTest, PointAndElementNotInDirect) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Point and element required"));
}

TEST_F(UserFlexTest, UnknownFlexCompType) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="unknown"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("invalid keyword: 'unknown'"));
}

TEST_F(UserFlexTest, InvalidPinid) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct" point="0 0 0" element="0 1 2">
      <pin id="1"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("element 1 has point id 1, number of points is 1"));
}

TEST_F(UserFlexTest, MeshFileMissing) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="mesh"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("File is required"));
}

TEST_F(UserFlexTest, VertexOrFaceDataMissing) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/malformed_flex_nofaces.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Vertex and face data required"));
}

TEST_F(UserFlexTest, CreateBVHSuccess) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/robot_arm.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, RigidFlex) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/rigid_flex.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}
TEST_F(UserFlexTest, BoundingBoxCoordinates) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" pos="1 0 -1" type="grid"
              count="5 5 5" spacing="1 1 1" dim="3"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);

  EXPECT_EQ(m->nflexvert, 5*5*5);
  EXPECT_EQ(m->nflexelem, 4*4*4*6);
  EXPECT_EQ(m->flex_dim[0], 3);

  // Cartesian coordinates
  EXPECT_EQ(d->flexvert_xpos[0], -1);
  EXPECT_EQ(d->flexvert_xpos[1], -2);
  EXPECT_EQ(d->flexvert_xpos[2], -3);
  EXPECT_EQ(d->flexvert_xpos[3*m->nflexvert-3], 3);
  EXPECT_EQ(d->flexvert_xpos[3*m->nflexvert-2], 2);
  EXPECT_EQ(d->flexvert_xpos[3*m->nflexvert-1], 1);

  // bounding box coordinates
  EXPECT_EQ(m->flex_vert0[0], 0);
  EXPECT_EQ(m->flex_vert0[1], 0);
  EXPECT_EQ(m->flex_vert0[2], 0);
  EXPECT_EQ(m->flex_vert0[3*m->nflexvert-3], 1);
  EXPECT_EQ(m->flex_vert0[3*m->nflexvert-2], 1);
  EXPECT_EQ(m->flex_vert0[3*m->nflexvert-1], 1);

  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHBinary_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_binary_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHBinary_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_binary_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHSurfaceBinary_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_binary_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[1], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[2], 0 );

  // first element
  EXPECT_EQ(m->flex_elem[0], 9-1 );
  EXPECT_EQ(m->flex_elem[1], 4-1 );
  EXPECT_EQ(m->flex_elem[2], 3-1 );

  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHSurfaceBinary_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_binary_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[1], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[2], 0 );

  // first element
  EXPECT_EQ(m->flex_elem[0], 9-1 );
  EXPECT_EQ(m->flex_elem[1], 4-1 );
  EXPECT_EQ(m->flex_elem[2], 3-1 );

  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHBinaryFTETWILD_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/shark_22_binary_vol_fTetWild.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 429);
  EXPECT_EQ(m->nflexelem, 1073);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_ascii_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_ascii_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHSurfaceASCII_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_ascii_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[1], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[2], 0 );

  // first element
  EXPECT_EQ(m->flex_elem[0], 9-1 );
  EXPECT_EQ(m->flex_elem[1], 4-1 );
  EXPECT_EQ(m->flex_elem[2], 3-1 );

  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHSurfaceASCII_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_ascii_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[1], -0.5 );
  EXPECT_EQ(d->flexvert_xpos[2], 0 );

  // first element
  EXPECT_EQ(m->flex_elem[0], 9-1 );
  EXPECT_EQ(m->flex_elem[1], 4-1 );
  EXPECT_EQ(m->flex_elem[2], 3-1 );

  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHASCIIFTETWILD_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/shark_22_ascii_vol_fTetWild.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 425);
  EXPECT_EQ(m->nflexelem, 1070);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingNodeHeader_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_41_ascii_missing_node_header.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr(
        "XML Error: Error: All nodes must be in single block"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingNodeIndex_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_41_ascii_missing_node_index.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr(
        "XML Error: Error: Node tags must be sequential"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingElementHeader_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_41_ascii_missing_element_header.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr(
        "XML Error: Error: All elements must be in single block"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingElement_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_41_ascii_missing_element.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr(
        "XML Error: Error: Error reading Elements"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingNumNodes_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_22_ascii_missing_num_nodes.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // TODO(mohammadhamid): Replace with an assertion about the error message. For
  // some reason, on Windows the error message is different on GH Actions
  EXPECT_THAT(m, IsNull());
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingNode_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_22_ascii_missing_node.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr(
        "XML Error: Error: Error reading node tags"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingNumElements_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_shark_22_ascii_missing_num_elements.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // TODO(mohammadhamid): Replace with an assertion about the error message. For
  // some reason, on Windows the error message is different on GH Actions
  EXPECT_THAT(m, IsNull());
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingElement_Fail) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/malformed_cube_22_ascii_missing_element.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr(
        "XML Error: Error: Error reading Elements"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_dim_missing_in_xml) {
  const std::string xml_path =
      GetTestDataFilePath(
          "user/testdata/cube_22_ascii_vol_gmshApp_missing_dim.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
