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

TEST_F(UserFlexTest, UknownFlexCompType) {
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
  mjData* d = mj_makeData(m);
  EXPECT_THAT(m, NotNull()) << error.data();
  mj_step(m, d);
  mj_deleteModel(m);
  mj_deleteData(d);
}

}  // namespace
}  // namespace mujoco
