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

// Tests for user/user_api.cc.

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include "src/xml/xml_api.h"
#include "src/xml/xml_numeric_format.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;


// -------------------------- test model manipulation  -------------------------

TEST_F(MujocoTest, GetSetData) {
  mjSpec* spec = mj_makeSpec();
  mjsBody* world = mjs_findBody(spec, "world");
  mjsBody* body = mjs_addBody(world, 0);
  mjsSite* site = mjs_addSite(body, 0);

  {
    double vec[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const char* str = "sitename";

    mjs_setString(site->name, str);
    mjs_setDouble(site->userdata, vec, 10);
  }

  EXPECT_THAT(mjs_getString(site->name), HasSubstr("sitename"));

  int nsize;
  const double* vec = mjs_getDouble(site->userdata, &nsize);
  for (int i = 0; i < nsize; ++i) {
    EXPECT_EQ(vec[i], i);
  }

  mj_deleteSpec(spec);
}

TEST_F(MujocoTest, TreeTraversal) {
  mjSpec* spec = mj_makeSpec();
  mjsBody* world = mjs_findBody(spec, "world");
  mjsBody* body = mjs_addBody(world, 0);

  mjsSite* site1 = mjs_addSite(body, 0);
  mjsGeom* geom1 = mjs_addGeom(body, 0);
  mjsGeom* geom2 = mjs_addGeom(body, 0);
  mjsSite* site2 = mjs_addSite(body, 0);
  mjsSite* site3 = mjs_addSite(body, 0);
  mjsGeom* geom3 = mjs_addGeom(body, 0);

  mjsElement* t_el1 = mjs_firstChild(body, mjOBJ_TENDON);
  mjsElement* s_el1 = mjs_firstChild(body, mjOBJ_SITE);
  mjsElement* s_el2 = mjs_nextChild(body, s_el1);
  mjsElement* s_el3 = mjs_nextChild(body, s_el2);
  mjsElement* s_el4 = mjs_nextChild(body, s_el3);
  mjsElement* g_el1 = mjs_firstChild(body, mjOBJ_GEOM);
  mjsElement* g_el2 = mjs_nextChild(body, g_el1);
  mjsElement* g_el3 = mjs_nextChild(body, g_el2);
  mjsElement* g_el4 = mjs_nextChild(body, g_el3);

  EXPECT_EQ(t_el1, nullptr);
  EXPECT_EQ(s_el1, site1->element);
  EXPECT_EQ(s_el2, site2->element);
  EXPECT_EQ(s_el3, site3->element);
  EXPECT_EQ(g_el1, geom1->element);
  EXPECT_EQ(g_el2, geom2->element);
  EXPECT_EQ(g_el3, geom3->element);
  EXPECT_EQ(g_el4, nullptr);
  EXPECT_EQ(s_el4, nullptr);

  mj_deleteSpec(spec);
}

// ------------------- test recompilation multiple files -----------------------
TEST_F(PluginTest, RecompileCompare) {
  mjtNum tol = 0;
  std::string field = "";

  // full precision float printing
  FullFloatPrecision increase_precision;

  // loop over all xml files in data
  std::vector<std::string> paths = {GetTestDataFilePath("."),
                                    GetModelPath(".")};
  std::string ext(".xml");
  for (auto const& path : paths) {
    for (auto &p : std::filesystem::recursive_directory_iterator(path)) {
      if (p.path().extension() == ext) {
        std::string xml = p.path().string();

        // if file is meant to fail, skip it
        if (absl::StrContains(p.path().string(), "malformed_") ||
            absl::StrContains(p.path().string(), "touch_grid") ||
            absl::StrContains(p.path().string(), "cow") ||
            absl::StrContains(p.path().string(), "discardvisual")) {
          continue;
        }

        // load spec
        std::array<char, 1000> err;
        mjSpec* s = mj_parseXML(xml.c_str(), 0, err.data(), err.size());

        ASSERT_THAT(s, NotNull())
            << "Failed to load " << xml << ": " << err.data();

        // copy spec
        mjSpec* s_copy = mj_copySpec(s);

        // compile twice and compare
        mjModel* m_old = mj_compile(s, nullptr);

        ASSERT_THAT(m_old, NotNull())
            << "Failed to compile " << xml << ": " << mjs_getError(s);

        mjModel* m_new = mj_compile(s, nullptr);
        mjModel* m_copy = mj_compile(s_copy, nullptr);

        ASSERT_THAT(m_new, NotNull())
            << "Failed to recompile " << xml << ": " << mjs_getError(s);
        ASSERT_THAT(m_copy, NotNull())
            << "Failed to compile " << xml << ": " << mjs_getError(s_copy);

        EXPECT_LE(CompareModel(m_old, m_new, field), tol)
            << "Compiled and recompiled models are different!\n"
            << "Affected file " << p.path().string() << '\n'
            << "Different field: " << field << '\n';

        EXPECT_LE(CompareModel(m_old, m_copy, field), tol)
            << "Original and copied models are different!\n"
            << "Affected file " << p.path().string() << '\n'
            << "Different field: " << field << '\n';

        // copy to a new spec, compile and compare
        mjSpec* s_copy2 = mj_copySpec(s);
        mjModel* m_copy2 = mj_compile(s_copy2, nullptr);

        ASSERT_THAT(m_copy2, NotNull())
            << "Failed to compile " << xml << ": " << mjs_getError(s_copy2);

        EXPECT_LE(CompareModel(m_old, m_copy2, field), tol)
            << "Original and re-copied models are different!\n"
            << "Affected file " << p.path().string() << '\n'
            << "Different field: " << field << '\n';

        // delete models
        mj_deleteSpec(s);
        mj_deleteSpec(s_copy);
        mj_deleteSpec(s_copy2);
        mj_deleteModel(m_old);
        mj_deleteModel(m_new);
        mj_deleteModel(m_copy);
        mj_deleteModel(m_copy2);
      }
    }
  }
}

// ------------------- test cache with modified assets -------------------------

TEST_F(PluginTest, RecompileCompareObjCache) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh file="cube.obj"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="cube"/>
    </worldbody>
  </mujoco>)";

  static constexpr char cube1[] = R"(
  v -0.500000 -0.500000  0.500000
  v  0.500000 -0.500000  0.500000
  v -0.500000  0.500000  0.500000
  v  0.500000  0.500000  0.500000
  v -0.500000  0.500000 -0.500000
  v  0.500000  0.500000 -0.500000
  v -0.500000 -0.500000 -0.500000
  v  0.500000 -0.500000 -0.500000)";

  static constexpr char cube2[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "cube.obj", cube1, sizeof(cube1));

  std::array<char, 1024> error;

  // load model once
  mjModel* m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_EQ(m->mesh_vert[0], -0.5);
  mj_deleteModel(m);

  // update cube.obj, load again
  mj_deleteFileVFS(vfs.get(), "cube.obj");
  mj_addBufferVFS(vfs.get(), "cube.obj", cube2, sizeof(cube2));
  m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_EQ(m->mesh_vert[0], -1);
  mj_deleteModel(m);

  mj_deleteVFS(vfs.get());
}

TEST_F(PluginTest, RecompileComparePngCache) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture content_type="image/png" file="tex.png" type="2d"/>
      <material name="material" texture="tex"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
)";

  // tiny RGB 2 x 3 PNG file
  static constexpr unsigned char tex1[] = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
    0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02,
    0x08, 0x02, 0x00, 0x00, 0x00, 0x12, 0x16, 0xf1, 0x4d, 0x00, 0x00, 0x00,
    0x1c, 0x49, 0x44, 0x41, 0x54, 0x08, 0xd7, 0x63, 0x78, 0xc1, 0xc0, 0xc0,
    0xc0, 0xf0, 0xbf, 0xb8, 0xb8, 0x98, 0x81, 0xe1, 0x3f, 0xc3, 0xff, 0xff,
    0xff, 0xc5, 0xc4, 0xc4, 0x00, 0x46, 0xd7, 0x07, 0x7f, 0xd2, 0x52, 0xa1,
    0x41, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60,
    0x82
  };

  // previous PNG file, but rotated by 180 degrees
  static constexpr unsigned char tex2[] = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
    0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02,
    0x08, 0x02, 0x00, 0x00, 0x00, 0x12, 0x16, 0xf1, 0x4d, 0x00, 0x00, 0x00,
    0x1c, 0x49, 0x44, 0x41, 0x54, 0x08, 0xd7, 0x63, 0x10, 0x13, 0x13, 0xfb,
    0xff, 0xff, 0x3f, 0xc3, 0x7f, 0x06, 0x96, 0xd8, 0xd8, 0x58, 0x46, 0x46,
    0x86, 0x17, 0x0c, 0x0c, 0x00, 0x49, 0x22, 0x06, 0x44, 0xe4, 0x91, 0xb8,
    0x83, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60,
    0x82
  };


  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "tex.png", tex1, sizeof(tex1));

  std::array<char, 1024> error;

  // load model once
  mjModel* m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_EQ(m->ntexdata, 18);  // w x h x rgb = 3 x 2 x 3
  mjtByte byte = m->tex_rgb[0];
  mj_deleteModel(m);

  // update tex.png, load again
  mj_deleteFileVFS(vfs.get(), "tex.png");
  mj_addBufferVFS(vfs.get(), "tex.png", tex2, sizeof(tex2));
  m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_NE(m->tex_rgb[0], byte);
  EXPECT_EQ(m->tex_rgb[15], byte);  // first pixel is now last pixel
  mj_deleteModel(m);

  mj_deleteVFS(vfs.get());
}

// -------------------------------- test attach --------------------------------

static constexpr char xml_child[] = R"(
  <mujoco>
    <default>
      <default class="cylinder">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <asset>
      <texture name="texture" type="2d" builtin="checker" width="32" height="32"/>
      <material name="material" texture="texture" texrepeat="1 1" texuniform="true"/>
    </asset>

    <worldbody>
      <frame name="pframe">
        <frame name="cframe">
          <body name="body">
            <joint type="hinge" name="hinge"/>
            <geom class="cylinder" material="material"/>
            <light mode="targetbody" target="targetbody"/>
            <body name="targetbody"/>
            <body/>
          </body>
        </frame>
      </frame>
      <body name="ignore"/>
      <frame name="frame" pos=".1 0 0" euler="0 90 0"/>
    </worldbody>

    <sensor>
      <framepos name="sensor" objtype="body" objname="body"/>
      <framepos name="ignore" objtype="body" objname="ignore"/>
    </sensor>

    <tendon>
      <fixed name="fixed">
        <joint joint="hinge" coef="2"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="hinge" joint="hinge"/>
      <position name="fixed" tendon="fixed"/>
    </actuator>

    <contact>
      <exclude body1="body" body2="targetbody"/>
    </contact>
  </mujoco>)";

TEST_F(MujocoTest, AttachSame) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <default>
      <default class="cylinder">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <asset>
      <texture name="texture" type="2d" builtin="checker" width="32" height="32"/>
      <material name="material" texture="texture" texrepeat="1 1" texuniform="true"/>
    </asset>

    <worldbody>
      <body name="body">
        <joint type="hinge" name="hinge"/>
        <geom class="cylinder" material="material"/>
        <light mode="targetbody" target="targetbody"/>
        <body name="targetbody"/>
        <body/>
      </body>
      <body name="ignore"/>
      <frame name="frame" pos=".1 0 0" euler="0 90 0">
        <body name="attached-body-1">
          <joint type="hinge" name="attached-hinge-1"/>
          <geom class="cylinder" material="material"/>
          <light mode="targetbody" target="attached-targetbody-1"/>
          <body name="attached-targetbody-1"/>
          <body/>
        </body>
      </frame>
    </worldbody>

    <sensor>
      <framepos name="sensor" objtype="body" objname="body"/>
      <framepos name="ignore" objtype="body" objname="ignore"/>
      <framepos name="attached-sensor-1" objtype="body" objname="attached-body-1"/>
    </sensor>

    <tendon>
      <fixed name="fixed">
        <joint joint="hinge" coef="2"/>
      </fixed>
      <fixed name="attached-fixed-1">
        <joint joint="attached-hinge-1" coef="2"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="hinge" joint="hinge"/>
      <position name="fixed" tendon="fixed"/>
      <position name="attached-hinge-1" joint="attached-hinge-1"/>
      <position name="attached-fixed-1" tendon="attached-fixed-1"/>
    </actuator>

    <contact>
      <exclude body1="body" body2="targetbody"/>
      <exclude body1="attached-body-1" body2="attached-targetbody-1"/>
    </contact>
  </mujoco>)";

  // create parent
  mjSpec* parent = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();

  // get frame
  mjsFrame* frame = mjs_findFrame(parent, "frame");
  EXPECT_THAT(frame, NotNull());

  // get subtree
  mjsBody* body = mjs_findBody(parent, "body");
  EXPECT_THAT(body, NotNull());

  // attach child to parent frame
  EXPECT_THAT(
      mjs_attachBody(frame, body, /*prefix=*/"attached-", /*suffix=*/"-1"), 0);

  // compile new model
  mjModel* m_attached = mj_compile(parent, 0);
  EXPECT_THAT(m_attached, NotNull());

  // check full name stored in mjModel
  EXPECT_STREQ(mj_id2name(m_attached, mjOBJ_BODY, 5), "attached-body-1");

  // check body 3 is attached to the world
  EXPECT_THAT(m_attached->body_parentid[4], 0);

  // compare with expected XML
  mjModel* m_expected = LoadModelFromString(xml_result, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(m_attached, m_expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';;

  // destroy everything
  mj_deleteSpec(parent);
  mj_deleteModel(m_attached);
  mj_deleteModel(m_expected);
}

TEST_F(MujocoTest, AttachDifferent) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_parent[] = R"(
  <mujoco>
    <default>
      <default class="geom_size">
        <geom size="0.1"/>
      </default>
    </default>

    <worldbody>
      <body name="sphere">
        <freejoint/>
        <geom class="geom_size"/>
        <frame name="frame" pos=".1 0 0" euler="0 90 0"/>
      </body>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <default>
      <default class="geom_size">
        <geom size="0.1"/>
      </default>
      <default class="attached-cylinder-1">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <asset>
      <texture name="attached-texture-1" type="2d" builtin="checker" width="32" height="32"/>
      <material name="attached-material-1" texture="attached-texture-1" texrepeat="1 1" texuniform="true"/>
    </asset>

    <worldbody>
      <body name="sphere">
        <freejoint/>
        <geom class="geom_size"/>
        <frame name="frame" pos=".1 0 0" euler="0 90 0">
          <body name="attached-body-1">
            <joint type="hinge" name="attached-hinge-1"/>
            <geom class="attached-cylinder-1" material="attached-material-1"/>
            <light mode="targetbody" target="attached-targetbody-1"/>
              <body name="attached-targetbody-1"/>
              <body/>
          </body>
        </frame>
      </body>
    </worldbody>

    <sensor>
      <framepos name="attached-sensor-1" objtype="body" objname="attached-body-1"/>
    </sensor>

    <tendon>
      <fixed name="attached-fixed-1">
        <joint joint="attached-hinge-1" coef="2"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="attached-hinge-1" joint="attached-hinge-1"/>
      <position name="attached-fixed-1" tendon="attached-fixed-1"/>
    </actuator>

    <contact>
      <exclude body1="attached-body-1" body2="attached-targetbody-1"/>
    </contact>
  </mujoco>)";

  // model with one free sphere and a frame
  mjSpec* parent = mj_parseXMLString(xml_parent, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();

  // get frame
  mjsFrame* frame = mjs_findFrame(parent, "frame");
  EXPECT_THAT(frame, NotNull());

  // model with one cylinder and a hinge
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  // get subtree
  mjsBody* body = mjs_findBody(child, "body");
  EXPECT_THAT(body, NotNull());

  // attach child to parent frame
  EXPECT_EQ(
      mjs_attachBody(frame, body, /*prefix=*/"attached-", /*suffix=*/"-1"), 0);

  // compile new model
  mjModel* m_attached = mj_compile(parent, 0);
  EXPECT_THAT(m_attached, NotNull());

  // check full name stored in mjModel
  EXPECT_STREQ(mj_id2name(m_attached, mjOBJ_BODY, 2), "attached-body-1");

  // check body 2 is attached to body 1
  EXPECT_THAT(m_attached->body_parentid[2], 1);

  // compare with expected XML
  mjModel* m_expected = LoadModelFromString(xml_result, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(m_attached, m_expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';;

  // destroy everything
  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  mj_deleteModel(m_attached);
  mj_deleteModel(m_expected);
}

TEST_F(MujocoTest, AttachFrame) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_parent[] = R"(
  <mujoco>
    <worldbody>
      <body name="sphere">
        <freejoint/>
        <geom size=".1"/>
        <frame name="frame" pos=".1 0 0" euler="0 90 0"/>
      </body>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <default>
      <default class="attached-cylinder-1">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <asset>
      <texture name="attached-texture-1" type="2d" builtin="checker" width="32" height="32"/>
      <material name="attached-material-1" texture="attached-texture-1" texrepeat="1 1" texuniform="true"/>
    </asset>

    <worldbody>
      <body name="sphere">
        <freejoint/>
        <geom size=".1"/>
        <frame name="frame" pos=".1 0 0" euler="0 90 0"/>
        <frame name="pframe">
          <frame name="cframe">
            <body name="attached-body-1">
              <joint type="hinge" name="attached-hinge-1"/>
              <geom class="attached-cylinder-1" material="attached-material-1"/>
              <light mode="targetbody" target="attached-targetbody-1"/>
                <body name="attached-targetbody-1"/>
                <body/>
            </body>
          </frame>
        </frame>
      </body>
    </worldbody>

    <sensor>
      <framepos name="attached-sensor-1" objtype="body" objname="attached-body-1"/>
    </sensor>

    <tendon>
      <fixed name="attached-fixed-1">
        <joint joint="attached-hinge-1" coef="2"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="attached-hinge-1" joint="attached-hinge-1"/>
      <position name="attached-fixed-1" tendon="attached-fixed-1"/>
    </actuator>

    <contact>
      <exclude body1="attached-body-1" body2="attached-targetbody-1"/>
    </contact>
  </mujoco>)";

  // model with one free sphere and a frame
  mjSpec* parent = mj_parseXMLString(xml_parent, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();

  // get frame
  mjsBody* body = mjs_findBody(parent, "sphere");
  EXPECT_THAT(body, NotNull());

  // model with one cylinder and a hinge
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  // get subtree
  mjsFrame* frame = mjs_findFrame(child, "pframe");
  EXPECT_THAT(frame, NotNull());

  // attach child to parent frame
  EXPECT_THAT(
      mjs_attachFrame(body, frame, /*prefix=*/"attached-", /*suffix=*/"-1"), 0);

  // compile new model
  mjModel* m_attached = mj_compile(parent, 0);
  EXPECT_THAT(m_attached, NotNull());

  // check full name stored in mjModel
  EXPECT_STREQ(mj_id2name(m_attached, mjOBJ_BODY, 2), "attached-body-1");

  // check body 2 is attached to body 1
  EXPECT_THAT(m_attached->body_parentid[2], 1);

  // compare with expected XML
  mjModel* m_expected = LoadModelFromString(xml_result, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(m_attached, m_expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';;

  // destroy everything
  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  mj_deleteModel(m_attached);
  mj_deleteModel(m_expected);
}

void TestDetachBody(bool compile) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" type="2d" builtin="checker" width="32" height="32"/>
      <material name="material" texture="texture" texrepeat="1 1" texuniform="true"/>
    </asset>

    <worldbody>
      <frame name="pframe">
        <frame name="cframe">
        </frame>
      </frame>
      <body name="ignore"/>
      <frame name="frame" pos=".1 0 0" euler="0 90 0"/>
    </worldbody>
    <sensor>
      <framepos name="ignore" objtype="body" objname="ignore"/>
    </sensor>
  </mujoco>)";

  // model with one cylinder and a hinge
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  // compile model (for testing double compilation)
  mjModel* m_child = compile ? mj_compile(child, 0) : nullptr;

  // get subtree
  mjsBody* body = mjs_findBody(child, "body");
  EXPECT_THAT(body, NotNull());

  // detach subtree
  EXPECT_THAT(mjs_detachBody(child, body), 0);

  // compile new model
  mjModel* m_detached = mj_compile(child, 0);
  EXPECT_THAT(m_detached, NotNull());

  // compare with expected XML
  mjModel* m_expected = LoadModelFromString(xml_result, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(m_detached, m_expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';

  // destroy everything
  mj_deleteSpec(child);
  mj_deleteModel(m_detached);
  mj_deleteModel(m_expected);
  if (m_child) mj_deleteModel(m_child);
}

TEST_F(MujocoTest, DetachBody) {
  TestDetachBody(/*compile=*/false);
  TestDetachBody(/*compile=*/true);
}

TEST_F(MujocoTest, PreserveState) {
  std::array<char, 1000> er;
  std::string field = "";

  static constexpr char xml_full[] = R"(
  <mujoco>
    <worldbody>
      <body name="detachable" pos="1 0 0">
        <joint type="hinge" axis="0 0 1" name="hinge"/>
        <geom type="sphere" size=".1"/>
      </body>
      <body name="persistent">
        <joint type="slide" axis="0 0 1" name="slide"/>
        <geom type="sphere" size=".2"/>
      </body>
    </worldbody>
    <actuator>
      <position name="hinge" joint="hinge" timeconst=".01"/>
      <position name="slide" joint="slide" timeconst=".01"/>
    </actuator>
  </mujoco>)";

  static constexpr char xml_expected[] = R"(
  <mujoco>
    <worldbody>
      <body name="persistent">
        <joint type="slide" axis="0 0 1" name="slide"/>
        <geom type="sphere" size=".2"/>
      </body>
      <body name="newbody" pos="2 0 0">
        <joint type="slide" axis="0 0 1"/>
        <geom type="sphere" size=".3"/>
      </body>
    </worldbody>
    <actuator>
      <position name="slide" joint="slide" timeconst=".01"/>
    </actuator>
  </mujoco>)";

  // load spec
  mjSpec* spec = mj_parseXMLString(xml_full, 0, er.data(), er.size());
  EXPECT_THAT(spec, NotNull()) << er.data();

  // compile models
  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull());
  mjModel* m_expected = LoadModelFromString(xml_expected, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull());

  // create data
  mjData* data = mj_makeData(model);
  EXPECT_THAT(data, NotNull());
  mjData* d_expected = mj_makeData(m_expected);
  EXPECT_THAT(d_expected, NotNull());

  // set ctrl
  data->ctrl[0] = 1;
  data->ctrl[1] = 2;
  d_expected->ctrl[0] = 2;

  // step models
  mj_step(model, data);
  mj_step(m_expected, d_expected);

  // detach subtree
  mjsBody* body = mjs_findBody(spec, "detachable");
  EXPECT_THAT(body, NotNull());
  EXPECT_THAT(mjs_detachBody(spec, body), 0);

  // add body
  mjsBody* newbody = mjs_addBody(mjs_findBody(spec, "world"), 0);
  EXPECT_THAT(newbody, NotNull());

  // add geom and joint
  mjsGeom* geom = mjs_addGeom(newbody, 0);
  mjsJoint* joint = mjs_addJoint(newbody, 0);

  // set properties
  newbody->pos[0] = 2;
  geom->size[0] = .3;
  joint->type = mjJNT_SLIDE;
  joint->axis[0] = 0;
  joint->axis[1] = 0;
  joint->axis[2] = 1;
  joint->ref = d_expected->qpos[m_expected->nq-1];

  // compile new model
  mj_recompile(spec, 0, model, data);
  EXPECT_THAT(model, NotNull());

  // compare qpos
  EXPECT_EQ(model->nq, m_expected->nq);
  for (int i = 0; i < model->nq; ++i) {
    EXPECT_EQ(data->qpos[i], d_expected->qpos[i]) << i;
  }

  // compare qvel
  EXPECT_EQ(model->nv, m_expected->nv);
  for (int i = 0; i < model->nv-1; ++i) {
    EXPECT_EQ(data->qvel[i], d_expected->qvel[i]) << i;
  }

  // second body was added after stepping so qvel should be zero
  EXPECT_EQ(data->qvel[model->nv-1], 0);

  // compare act
  EXPECT_EQ(model->na, m_expected->na);
  for (int i = 0; i < model->na; ++i) {
    EXPECT_EQ(data->act[i], d_expected->act[i]) << i;
  }

  // destroy everything
  mj_deleteData(data);
  mj_deleteData(d_expected);
  mj_deleteSpec(spec);
  mj_deleteModel(model);
  mj_deleteModel(m_expected);
}

}  // namespace
}  // namespace mujoco
