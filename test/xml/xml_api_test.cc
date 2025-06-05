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

// Tests for xml/xml_api.cc.

#include <array>
#include <cstddef>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include "src/xml/xml_api.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::StartsWith;
using ::testing::HasSubstr;

static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1"/>
      </body>
      <body>
        <joint/>
        <geom size="0.5"/>
      </body>
    </worldbody>
  </mujoco>
  )";

// ---------------------------- test mj_loadXML --------------------------------

using LoadXmlTest = MujocoTest;

TEST_F(LoadXmlTest, EmptyModel) {
  static constexpr char xml[] = "<mujoco/>";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->nq, 0);
  EXPECT_EQ(model->nv, 0);
  EXPECT_EQ(model->nu, 0);
  EXPECT_EQ(model->na, 0);
  EXPECT_EQ(model->nbody, 1);  // worldbody exists even in empty model

  mjData* data = mj_makeData(model);
  EXPECT_THAT(data, NotNull());
  mj_step(model, data);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(LoadXmlTest, InvalidXmlFailsToLoad) {
  static constexpr char invalid_xml[] = "<mujoc";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(invalid_xml, error.data(), error.size());
  EXPECT_THAT(model, IsNull()) << "Expected model loading to fail.";
  EXPECT_GT(std::strlen(error.data()), 0);
  if (model) {
    mj_deleteModel(model);
  }
}

TEST_F(LoadXmlTest, MultipleBodies) {
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  EXPECT_EQ(model->nbody, 3);

  mjData* data = mj_makeData(model);
  EXPECT_THAT(data, NotNull());
  mj_step(model, data);
  mj_deleteData(data);
  mj_deleteModel(model);
}
using SaveLastXmlTest = MujocoTest;

TEST_F(SaveLastXmlTest, EmptyModel) {
  static constexpr char xml[] = "<mujoco/>";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  mjData* data = mj_makeData(model);

  std::array<char, 1024> error;
  error.data()[0] = '\0';

  testing::internal::CaptureStdout();
  mj_saveLastXML(nullptr, model, error.data(), error.size());

  EXPECT_THAT(testing::internal::GetCapturedStdout(), StartsWith("<mujoco"));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MujocoTest, SaveXmlShortString) {
  std::array<char, 1000> error;

  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << "Failed to parse spec: " << error.data();
  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull()) << "Failed to compile model: " << error.data();

  std::array<char, 10> out;
  EXPECT_THAT(mj_saveXMLString(spec, out.data(), out.size(),
                               error.data(), error.size()), 273);
  EXPECT_STREQ(error.data(), "Output string too short, should be at least 274");

  mj_deleteSpec(spec);
  mj_deleteModel(model);
}

TEST_F(MujocoTest, SaveXml) {
  std::array<char, 1000> error;

  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << "Failed to parse spec: " << error.data();
  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull()) << "Failed to compile model: " << error.data();

  std::array<char, 274> out;
  EXPECT_THAT(mj_saveXMLString(NULL, out.data(), out.size(), error.data(),
                               error.size()), -1);
  EXPECT_STREQ(error.data(), "Cannot write empty model");
  EXPECT_THAT(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                               error.size()), 0) << error.data();

  mjSpec* saved_spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(saved_spec, NotNull()) << "Invalid saved spec: " << error.data();
  mjModel* saved_model = mj_compile(saved_spec, 0);
  EXPECT_THAT(saved_model, NotNull()) << "Invalid model: " << error.data();

  mj_deleteSpec(spec);
  mj_deleteSpec(saved_spec);
  mj_deleteModel(model);
  mj_deleteModel(saved_model);
}

TEST_F(MujocoTest, SaveXmlWithDefaultMesh) {
  static constexpr char xml[] = R"(
    <mujoco>
      <default>
        <mesh inertia="shell"/>
      </default>
      <asset>
        <mesh name="test_mesh" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/>
      </asset>
      <worldbody>
        <body>
          <geom mesh="test_mesh" type="mesh"/>
        </body>
      </worldbody>
    </mujoco>
    )";

  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << "Failed to parse spec: " << error.data();
  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull()) << "Failed to compile model: " << error.data();

  std::array<char, 1024> out;
  EXPECT_THAT(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                               error.size()), 0) << error.data();

  mjSpec* saved_spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(saved_spec, NotNull()) << "Invalid saved spec: " << error.data();
  mjModel* saved_model = mj_compile(saved_spec, 0);
  EXPECT_THAT(saved_model, NotNull()) << "Invalid model: " << error.data();

  // check that the mesh has inertia="shell"
  EXPECT_THAT(out.data(), HasSubstr(R"(<mesh inertia="shell"/>)"));

  mj_deleteSpec(spec);
  mj_deleteSpec(saved_spec);
  mj_deleteModel(model);
  mj_deleteModel(saved_model);
}

TEST_F(MujocoTest, FreeLastXml) {
  static constexpr char xml[] = "<mujoco/>";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
  ASSERT_NE(mj_saveLastXML(nullptr, nullptr, nullptr, 0), 0);
  mj_freeLastXML();
  ASSERT_EQ(mj_saveLastXML(nullptr, nullptr, nullptr, 0), 0);
}

}  // namespace
}  // namespace mujoco
