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

#include "src/xml/xml_api.h"

#include <array>
#include <cstddef>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::StartsWith;

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
  MjModelPtr model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model.get(), NotNull());
  EXPECT_EQ(model->nq, 0);
  EXPECT_EQ(model->nv, 0);
  EXPECT_EQ(model->nu, 0);
  EXPECT_EQ(model->na, 0);
  EXPECT_EQ(model->nbody, 1);  // worldbody exists even in empty model

  MjDataPtr data = MakeData(model);
  EXPECT_THAT(data, NotNull());
  mj_step(model.get(), data.get());
}

TEST_F(LoadXmlTest, InvalidXmlFailsToLoad) {
  static constexpr char invalid_xml[] = "<mujoc";
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(invalid_xml, error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull()) << "Expected model loading to fail.";
  EXPECT_GT(std::strlen(error.data()), 0);
  if (model.get()) {
  }
}

TEST_F(LoadXmlTest, MultipleBodies) {
  std::array<char, 1000> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());

  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();
  EXPECT_EQ(model->nbody, 3);

  MjDataPtr data = MakeData(model);
  EXPECT_THAT(data, NotNull());
  mj_step(model.get(), data.get());
}
using SaveLastXmlTest = MujocoTest;

TEST_F(SaveLastXmlTest, EmptyModel) {
  static constexpr char xml[] = "<mujoco/>";
  MjModelPtr model = LoadModelFromString(xml, 0, 0);
  MjDataPtr data = MakeData(model);

  std::array<char, 1024> error;
  error.data()[0] = '\0';

  testing::internal::CaptureStdout();
  mj_saveLastXML(nullptr, model.get(), error.data(), error.size());

  EXPECT_THAT(testing::internal::GetCapturedStdout(), StartsWith("<mujoco"));
}

TEST_F(LoadXmlTest, NullFileFails) {
  std::array<char, 1000> error;
  mjSpec* spec = mj_parseXML(nullptr, nullptr, error.data(), error.size());
  EXPECT_THAT(spec, IsNull()) << "Expected model loading to fail.";
  EXPECT_THAT(error.data(), HasSubstr("filename argument required"));
}

TEST_F(LoadXmlTest, InvalidFileFails) {
  std::array<char, 1000> error;
  mjSpec* spec = mj_parseXML("invalid", nullptr, error.data(), error.size());
  EXPECT_THAT(spec, IsNull()) << "Expected model loading to fail.";
  EXPECT_THAT(error.data(), HasSubstr("Error opening file"));
}

TEST_F(MujocoTest, SaveXmlShortString) {
  std::array<char, 1000> error;

  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << "Failed to parse spec: " << error.data();
  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull()) << "Failed to compile model: " << error.data();

  std::array<char, 10> out;
  EXPECT_THAT(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                               error.size()),
              223);
  EXPECT_STREQ(error.data(), "Output string too short, should be at least 224");

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
                               error.size()),
              -1);
  EXPECT_STREQ(error.data(), "Cannot write empty model");
  EXPECT_THAT(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                               error.size()),
              0)
      << error.data();

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
                               error.size()),
              0)
      << error.data();

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

TEST_F(MujocoTest, SaveXmlAfterAttachNeedsRecompile) {
  static constexpr char xml_parent[] = R"(
  <mujoco model="parent">
    <worldbody>
      <body name="p">
        <joint type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_child[] = R"(
  <mujoco model="child">
    <worldbody>
      <body name="c">
        <joint type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>)";

  std::array<char, 1024> error;
  mjSpec* parent = mj_parseXMLString(xml_parent, 0, error.data(), error.size());
  ASSERT_THAT(parent, NotNull()) << error.data();
  mjSpec* child = mj_parseXMLString(xml_child, 0, error.data(), error.size());
  ASSERT_THAT(child, NotNull()) << error.data();
  mjModel* model = mj_compile(parent, 0);
  ASSERT_THAT(model, NotNull()) << mjs_getError(parent);

  // attach to the compiled parent
  mjsFrame* frame = mjs_addFrame(mjs_findBody(parent, "world"), nullptr);
  mjsBody* body = mjs_findBody(child, "c");
  ASSERT_THAT(mjs_attach(frame->element, body->element, "c-", ""), NotNull());

  // saving as written needs no new compilation, saving compiled values does
  std::array<char, 2048> out;
  parent->compiler.savecompiled = 0;
  ASSERT_EQ(mj_saveXMLString(parent, out.data(), out.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  EXPECT_THAT(LoadModelFromString(out.data(), error.data(), error.size()),
              NotNull())
      << error.data();
  parent->compiler.savecompiled = 1;
  EXPECT_EQ(mj_saveXMLString(parent, out.data(), out.size(), error.data(),
                             error.size()),
            -1);
  EXPECT_THAT(error.data(), HasSubstr("must be recompiled"));

  // after it, the saved model has the attached body
  mjModel* recompiled = mj_compile(parent, 0);
  ASSERT_THAT(recompiled, NotNull()) << mjs_getError(parent);
  ASSERT_EQ(mj_saveXMLString(parent, out.data(), out.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  MjModelPtr saved =
      LoadModelFromString(out.data(), error.data(), error.size());
  ASSERT_THAT(saved.get(), NotNull()) << error.data();
  EXPECT_EQ(saved->nbody, 3);

  mj_deleteModel(recompiled);
  mj_deleteModel(model);
  mj_deleteSpec(child);
  mj_deleteSpec(parent);
}

TEST_F(MujocoTest, SaveXmlAfterDeleteNeedsRecompile) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="a">
        <freejoint/>
        <geom name="ga" size=".1"/>
      </body>
      <body name="b">
        <freejoint/>
        <geom name="gb" size=".1"/>
      </body>
      <body name="c">
        <freejoint/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <sensor>
      <contact geom1="ga" geom2="gb"/>
    </sensor>
  </mujoco>)";

  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  mjModel* model = mj_compile(spec, 0);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);

  // delete a body of the compiled spec
  ASSERT_EQ(mjs_delete(spec, mjs_findBody(spec, "c")->element), 0);

  // saving as written needs no new compilation, saving compiled values does
  std::array<char, 2048> out;
  spec->compiler.savecompiled = 0;
  ASSERT_EQ(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  EXPECT_THAT(LoadModelFromString(out.data(), error.data(), error.size()),
              NotNull())
      << error.data();
  spec->compiler.savecompiled = 1;
  EXPECT_EQ(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                             error.size()),
            -1);
  EXPECT_THAT(error.data(), HasSubstr("must be recompiled"));

  // after it, the saved model loads and has the sensor
  mjModel* recompiled = mj_compile(spec, 0);
  ASSERT_THAT(recompiled, NotNull()) << mjs_getError(spec);
  ASSERT_EQ(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  MjModelPtr saved =
      LoadModelFromString(out.data(), error.data(), error.size());
  ASSERT_THAT(saved.get(), NotNull()) << error.data();
  EXPECT_EQ(saved->nsensor, 1);

  mj_deleteModel(recompiled);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST_F(MujocoTest, SaveXmlAfterAddNeedsRecompile) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="a">
        <freejoint/>
        <geom size=".1"/>
        <site name="s"/>
      </body>
      <body name="b">
        <freejoint/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>)";

  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  mjModel* model = mj_compile(spec, 0);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);

  // add elements to the compiled spec
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), nullptr);
  body->pos[2] = 1;
  mjsGeom* geom = mjs_addGeom(body, nullptr);
  geom->size[0] = 0.25;
  mjsExclude* exclude = mjs_addExclude(spec);
  mjs_setString(exclude->bodyname1, "a");
  mjs_setString(exclude->bodyname2, "b");
  mjsSensor* sensor = mjs_addSensor(spec);
  sensor->type = mjSENS_ACCELEROMETER;
  sensor->objtype = mjOBJ_SITE;
  mjs_setString(sensor->objname, "s");

  // saving as written needs no new compilation, saving compiled values does
  std::array<char, 2048> out;
  spec->compiler.savecompiled = 0;
  ASSERT_EQ(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  EXPECT_THAT(LoadModelFromString(out.data(), error.data(), error.size()),
              NotNull())
      << error.data();
  spec->compiler.savecompiled = 1;
  EXPECT_EQ(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                             error.size()),
            -1);
  EXPECT_THAT(error.data(), HasSubstr("must be recompiled"));

  // after it, the saved model has the added elements
  mjModel* recompiled = mj_compile(spec, 0);
  ASSERT_THAT(recompiled, NotNull()) << mjs_getError(spec);
  ASSERT_EQ(mj_saveXMLString(spec, out.data(), out.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  MjModelPtr saved =
      LoadModelFromString(out.data(), error.data(), error.size());
  ASSERT_THAT(saved.get(), NotNull()) << error.data();
  EXPECT_EQ(saved->body_pos[3 * 3 + 2], 1);
  EXPECT_EQ(saved->geom_size[3 * 2], 0.25);
  EXPECT_EQ(saved->nexclude, 1);
  ASSERT_EQ(saved->nsensor, 1);
  EXPECT_EQ(saved->sensor_type[0], mjSENS_ACCELEROMETER);

  mj_deleteModel(recompiled);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST_F(MujocoTest, FreeLastXml) {
  static constexpr char xml[] = "<mujoco/>";
  MjModelPtr model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model.get(), NotNull());
  ASSERT_NE(mj_saveLastXML(nullptr, nullptr, nullptr, 0), 0);
  mj_freeLastXML();
  ASSERT_EQ(mj_saveLastXML(nullptr, nullptr, nullptr, 0), 0);
}

}  // namespace
}  // namespace mujoco
