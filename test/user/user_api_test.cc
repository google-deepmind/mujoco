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
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include "src/cc/array_safety.h"
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include "src/xml/xml_api.h"
#include "src/xml/xml_numeric_format.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;
using ::testing::IsNull;


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
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <body name="body1">
          <site name="site4"/>
        </body>
        <site name="site1"/>
        <geom name="geom1" size="1"/>
        <geom name="geom2" size="1"/>
        <site name="site2"/>
        <site name="site3"/>
        <geom name="geom3" size="1"/>
      </body>
      <body name="body2">
        <site name="site5"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1000> err;
  mjSpec* spec = mj_parseXMLString(xml, 0, err.data(), err.size());
  ASSERT_THAT(spec, NotNull()) << err.data();

  mjsBody* world = mjs_findBody(spec, "world");
  mjsBody* body = mjs_findBody(spec, "body");
  mjsBody* body1 = mjs_findBody(spec, "body1");
  mjsBody* body2 = mjs_findBody(spec, "body2");
  mjsElement* site1 = mjs_findElement(spec, mjOBJ_SITE, "site1");
  mjsElement* site2 = mjs_findElement(spec, mjOBJ_SITE, "site2");
  mjsElement* site3 = mjs_findElement(spec, mjOBJ_SITE, "site3");
  mjsElement* site4 = mjs_findElement(spec, mjOBJ_SITE, "site4");
  mjsElement* site5 = mjs_findElement(spec, mjOBJ_SITE, "site5");
  mjsElement* geom1 = mjs_findElement(spec, mjOBJ_GEOM, "geom1");
  mjsElement* geom2 = mjs_findElement(spec, mjOBJ_GEOM, "geom2");
  mjsElement* geom3 = mjs_findElement(spec, mjOBJ_GEOM, "geom3");

  // test nonexistent
  EXPECT_EQ(mjs_firstElement(spec, mjOBJ_ACTUATOR), nullptr);
  EXPECT_EQ(mjs_firstElement(spec, mjOBJ_LIGHT), nullptr);
  EXPECT_EQ(mjs_firstChild(body, mjOBJ_CAMERA, /*recurse=*/true), nullptr);
  EXPECT_EQ(mjs_firstChild(body, mjOBJ_TENDON, /*recurse=*/true), nullptr);

  // test first, nonrecursive
  EXPECT_EQ(mjs_firstElement(spec, mjOBJ_BODY), world->element);
  EXPECT_EQ(site1, mjs_firstElement(spec, mjOBJ_SITE));
  EXPECT_EQ(site1, mjs_firstChild(body, mjOBJ_SITE, /*recurse=*/false));
  EXPECT_EQ(geom1, mjs_firstChild(body, mjOBJ_GEOM, /*recurse=*/false));
  EXPECT_EQ(site4, mjs_firstChild(body1, mjOBJ_SITE, /*recurse=*/false));
  EXPECT_EQ(site5, mjs_firstChild(body2, mjOBJ_SITE, /*recurse=*/false));

  // test first, recursive
  EXPECT_EQ(site1, mjs_firstChild(world, mjOBJ_SITE, /*recurse=*/true));
  EXPECT_EQ(geom1, mjs_firstChild(world, mjOBJ_GEOM, /*recurse=*/true));
  EXPECT_EQ(site4, mjs_firstChild(body1, mjOBJ_SITE, /*recurse=*/true));
  EXPECT_EQ(site5, mjs_firstChild(body2, mjOBJ_SITE, /*recurse=*/true));

  // text next, nonrecursive
  EXPECT_EQ(site2, mjs_nextChild(body, site1, /*recursive=*/false));
  EXPECT_EQ(site3, mjs_nextChild(body, site2, /*recursive=*/false));
  EXPECT_EQ(nullptr, mjs_nextChild(body, site3, /*recursive=*/false));
  EXPECT_EQ(geom2, mjs_nextChild(body, geom1, /*recursive=*/false));
  EXPECT_EQ(geom3, mjs_nextChild(body, geom2, /*recursive=*/false));
  EXPECT_EQ(nullptr, mjs_nextChild(body, geom3, /*recursive=*/false));
  EXPECT_EQ(mjs_nextElement(spec, site1), site2);
  EXPECT_EQ(mjs_nextElement(spec, site2), site3);
  EXPECT_EQ(mjs_nextElement(spec, site3), site4);
  EXPECT_EQ(mjs_nextElement(spec, site4), site5);
  EXPECT_EQ(mjs_nextElement(spec, site5), nullptr);
  EXPECT_EQ(mjs_nextElement(spec, geom1), geom2);
  EXPECT_EQ(mjs_nextElement(spec, geom2), geom3);
  EXPECT_EQ(mjs_nextElement(spec, geom3), nullptr);

  // text next, recursive
  EXPECT_EQ(site2, mjs_nextChild(body, site1, /*recursive=*/true));
  EXPECT_EQ(site3, mjs_nextChild(body, site2, /*recursive=*/true));
  EXPECT_EQ(site4, mjs_nextChild(body, site3, /*recursive=*/true));
  EXPECT_EQ(site4, mjs_nextChild(world, site3, /*recursive=*/true));
  EXPECT_EQ(site5, mjs_nextChild(world, site4, /*recursive=*/true));
  EXPECT_EQ(nullptr, mjs_nextChild(body, site5, /*recursive=*/true));
  EXPECT_EQ(geom2, mjs_nextChild(body, geom1, /*recursive=*/true));
  EXPECT_EQ(geom3, mjs_nextChild(body, geom2, /*recursive=*/true));
  EXPECT_EQ(nullptr, mjs_nextChild(body, geom3, /*recursive=*/true));

  // check compilation ordering of sites
  mjModel* model = mj_compile(spec, nullptr);
  EXPECT_THAT(model, NotNull());
  EXPECT_EQ(mjs_getId(site1), 0);
  EXPECT_EQ(mjs_getId(site2), 1);
  EXPECT_EQ(mjs_getId(site3), 2);
  EXPECT_EQ(mjs_getId(site4), 3);
  EXPECT_EQ(mjs_getId(site5), 4);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST_F(PluginTest, ActivatePlugin) {
  mjSpec* spec = mj_makeSpec();
  mjs_activatePlugin(spec, "mujoco.elasticity.cable");

  // associate plugin to body
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), 0);
  mjs_setString(body->plugin.plugin_name, "mujoco.elasticity.cable");
  body->plugin.element = mjs_addPlugin(spec)->element;
  body->plugin.active = true;
  mjsGeom* geom = mjs_addGeom(body, 0);
  geom->type = mjGEOM_BOX;
  geom->size[0] = 1;
  geom->size[1] = 1;
  geom->size[2] = 1;

  // compile and check that the plugin is present
  mjModel* model = mj_compile(spec, NULL);
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->nplugin, 1);
  EXPECT_THAT(model->body_plugin[1], 0);

  mj_deleteSpec(spec);
  mj_deleteModel(model);
}

TEST_F(PluginTest, DeletePlugin) {
  mjSpec* spec = mj_makeSpec();
  mjs_activatePlugin(spec, "mujoco.pid");

  // create body
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), 0);
  mjsJoint* joint = mjs_addJoint(body, 0);
  mjsGeom* geom = mjs_addGeom(body, 0);
  mjs_setString(joint->name, "j1");
  joint->type = mjJNT_SLIDE;
  geom->size[0] = 1;

  // add actuator
  mjsActuator* actuator = mjs_addActuator(spec, 0);
  mjs_setString(actuator->target, "j1");
  mjs_setString(actuator->plugin.plugin_name, "mujoco.pid");
  actuator->plugin.element = mjs_addPlugin(spec)->element;
  actuator->plugin.active = true;
  actuator->trntype = mjTRN_JOINT;

  // compile and check that the plugin is present
  mjModel* model = mj_compile(spec, NULL);
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->nu, 1);
  EXPECT_THAT(model->nplugin, 1);
  EXPECT_THAT(model->actuator_plugin[0], 0);

  // delete actuator
  mjs_delete(actuator->element);

  // recompile and check that the plugin is not present
  mjModel* newmodel = mj_compile(spec, NULL);
  EXPECT_THAT(newmodel, NotNull());
  EXPECT_THAT(newmodel->nu, 0);
  EXPECT_THAT(newmodel->nplugin, 0);

  mj_deleteSpec(spec);
  mj_deleteModel(model);
  mj_deleteModel(newmodel);
}

TEST_F(PluginTest, AttachPlugin) {
  static constexpr char xml_1[] = R"(
    <mujoco model="MuJoCo Model">
      <worldbody>
        <body name="body"/>
      </worldbody>
    </mujoco>)";

  static constexpr char xml_2[] = R"(
    <mujoco model="MuJoCo Model">
      <extension>
        <plugin plugin="mujoco.pid">
          <instance name="actuator-1">
            <config key="ki" value="4.0"/>
            <config key="slewmax" value="3.14159"/>
          </instance>
        </plugin>
      </extension>
      <worldbody>
        <body name="empty"/>
        <body name="body">
          <joint name="joint"/>
          <geom size="0.1"/>
        </body>
      </worldbody>
      <actuator>
        <plugin name="actuator-1" plugin="mujoco.pid" instance="actuator-1"
                joint="joint" actdim="2"/>
      </actuator>
    </mujoco>)";

  std::array<char, 1000> err;
  mjSpec* spec_1 = mj_parseXMLString(xml_1, 0, err.data(), err.size());
  ASSERT_THAT(spec_1, NotNull()) << err.data();
  mjSpec* spec_2 = mj_parseXMLString(xml_2, 0, err.data(), err.size());
  ASSERT_THAT(spec_2, NotNull()) << err.data();

  mjsBody* body_1 = mjs_findBody(spec_1, "body");
  EXPECT_THAT(body_1, NotNull());
  mjsFrame* attachment_frame = mjs_addFrame(body_1, 0);
  EXPECT_THAT(attachment_frame, NotNull());

  mjs_attachBody(attachment_frame, mjs_findBody(spec_2, "body"), "child-", "");
  mjModel* model_1 = mj_compile(spec_1, nullptr);
  EXPECT_THAT(model_1, NotNull());

  // attach it a second time to test namespacing
  mjs_attachBody(attachment_frame, mjs_findBody(spec_2, "body"), "copy-", "");
  mjModel* model_2 = mj_compile(spec_1, nullptr);
  EXPECT_THAT(model_2, NotNull());

  // attach a body not referencing the plugin
  mjs_attachBody(attachment_frame, mjs_findBody(spec_2, "empty"), "empty-", "");
  mjModel* model_3 = mj_compile(spec_1, nullptr);
  EXPECT_THAT(model_3, NotNull());

  mj_deleteModel(model_1);
  mj_deleteModel(model_2);
  mj_deleteModel(model_3);
  mj_deleteSpec(spec_1);
  mj_deleteSpec(spec_2);
}

TEST_F(PluginTest, ReplicatePlugin) {
  static constexpr char xml[] = R"(
    <mujoco>
      <extension>
        <plugin plugin="mujoco.sdf.torus">
          <instance name="torus" />
        </plugin>
      </extension>
      <asset>
        <mesh name="torus">
          <plugin instance="torus" />
        </mesh>
      </asset>
      <worldbody>
        <geom type="sdf" mesh="torus">
          <plugin instance="torus" />
        </geom>
        <replicate count="100">
          <body />
        </replicate>
      </worldbody>
    </mujoco>)";

  std::array<char, 1000> err;
  mjSpec* spec = mj_parseXMLString(xml, 0, err.data(), err.size());
  ASSERT_THAT(spec, NotNull()) << err.data();
  mjModel* model = mj_compile(spec, nullptr);
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->nplugin, 1);
  mj_deleteSpec(spec);
  mj_deleteModel(model);
}

TEST_F(MujocoTest, RecompileFails) {
  mjSpec* spec = mj_makeSpec();
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), 0);
  mjsGeom* geom = mjs_addGeom(body, 0);
  geom->type = mjGEOM_SPHERE;
  geom->size[0] = 1;

  mjModel* model = mj_compile(spec, 0);
  mjData* data = mj_makeData(model);

  mjsMaterial* mat1 = mjs_addMaterial(spec, 0);
  mjsMaterial* mat2 = mjs_addMaterial(spec, 0);
  mjs_setString(mat1->name, "yellow");
  mjs_setString(mat2->name, "yellow");

  EXPECT_EQ(mj_recompile(spec, 0, model, data), -1);
  EXPECT_STREQ(mjs_getError(spec), "Error: repeated name 'yellow' in material");

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

TEST_F(PluginTest, RecompileEdit) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1000> er;
  mjSpec *spec = mj_parseXMLString(xml, 0, er.data(), er.size());
  EXPECT_THAT(spec, NotNull()) << er.data();
  mjModel *m1 = mj_compile(spec, nullptr);
  EXPECT_THAT(m1, NotNull());

  // add a geom
  mjsBody *world = mjs_findBody(spec, "world");
  mjsGeom *geom = mjs_addGeom(world, nullptr);
  geom->size[0] = 1;

  // compile again
  mjModel *m2 = mj_compile(spec, nullptr);
  EXPECT_THAT(m2, NotNull());

  mj_deleteModel(m1);
  mj_deleteModel(m2);
  mj_deleteSpec(spec);
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

// tiny RGB 2 x 3 PNG file
static constexpr uint8_t tex1[] = {
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
static constexpr uint8_t tex2[] = {
  0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02,
  0x08, 0x02, 0x00, 0x00, 0x00, 0x12, 0x16, 0xf1, 0x4d, 0x00, 0x00, 0x00,
  0x1c, 0x49, 0x44, 0x41, 0x54, 0x08, 0xd7, 0x63, 0x10, 0x13, 0x13, 0xfb,
  0xff, 0xff, 0x3f, 0xc3, 0x7f, 0x06, 0x96, 0xd8, 0xd8, 0x58, 0x46, 0x46,
  0x86, 0x17, 0x0c, 0x0c, 0x00, 0x49, 0x22, 0x06, 0x44, 0xe4, 0x91, 0xb8,
  0x83, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60,
  0x82
};

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

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "tex.png", tex1, sizeof(tex1));

  std::array<char, 1024> error;

  // load model once
  mjModel* m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_EQ(m->ntexdata, 18);  // w x h x rgb = 3 x 2 x 3
  mjtByte byte = m->tex_data[0];
  mj_deleteModel(m);

  // update tex.png, load again
  mj_deleteFileVFS(vfs.get(), "tex.png");
  mj_addBufferVFS(vfs.get(), "tex.png", tex2, sizeof(tex2));
  m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_NE(m->tex_data[0], byte);
  EXPECT_EQ(m->tex_data[15], byte);  // first pixel is now last pixel
  mj_deleteModel(m);

  mj_deleteVFS(vfs.get());
}

// -------------------------------- test textures ------------------------------

TEST_F(PluginTest, TextureFromBuffer) {
  mjSpec* spec = mj_makeSpec();

  mjsTexture* t1 = mjs_addTexture(spec);
  mjs_setString(t1->name, "tex1");
  t1->type = mjTEXTURE_2D;
  t1->width = 3;
  t1->height = 2;
  t1->nchannel = 3;
  mjs_setBuffer(t1->data, (std::byte*)tex1, 18);

  mjsTexture* t2 = mjs_addTexture(spec);
  mjs_setString(t2->name, "tex2");
  t2->type = mjTEXTURE_2D;
  t2->width = 3;
  t2->height = 2;
  t2->nchannel = 3;
  mjs_setBuffer(t2->data, (std::byte*)tex2, 18);

  mjsMaterial* mat = mjs_addMaterial(spec, nullptr);
  mjs_setString(mat->name, "mat");
  mjs_setInStringVec(mat->textures, mjTEXROLE_RGB, "tex1");
  mjs_setInStringVec(mat->textures, mjTEXROLE_ORM, "tex2");

  mjsGeom* geom = mjs_addGeom(mjs_findBody(spec, "world"), nullptr);
  mjs_setString(geom->material, "mat");
  geom->size[0] = 1;

  mjModel* m = mj_compile(spec, nullptr);
  EXPECT_THAT(m, NotNull());
  EXPECT_EQ(m->ntex, 2);

  std::array<char, 1024> err;
  std::array<char, 1024> str;
  mj_saveXMLString(spec, str.data(), str.size(), err.data(), err.size());
  EXPECT_STREQ(err.data(), "XML Error: no support for buffer textures.");

  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// -------------------------------- test attach --------------------------------

static constexpr char xml_child[] = R"(
  <mujoco model="child">
    <default>
      <default class="cylinder">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <custom>
      <numeric data="10" name="constant"/>
    </custom>

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
      <body name="ignore">
        <geom size=".1"/>
        <joint type="slide"/>
      </body>
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
      <position name="hinge" joint="hinge" timeconst=".01"/>
      <position name="fixed" tendon="fixed" timeconst=".01"/>
    </actuator>

    <contact>
      <exclude body1="body" body2="targetbody"/>
    </contact>

    <keyframe>
      <key name="two" time="2" qpos="2 22" act="2 2" ctrl="2 2"/>
      <key name="three" time="3" qpos="3 33" act="3 3" ctrl="3 3"/>
    </keyframe>
  </mujoco>)";

TEST_F(MujocoTest, AttachSame) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_result[] = R"(
  <mujoco model="child">
    <default>
      <default class="cylinder">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <custom>
      <numeric data="10" name="constant"/>
    </custom>

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
      <body name="ignore">
        <geom size=".1"/>
        <joint type="slide"/>
      </body>
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
      <position name="hinge" joint="hinge" timeconst=".01"/>
      <position name="fixed" tendon="fixed" timeconst=".01"/>
      <position name="attached-hinge-1" joint="attached-hinge-1" timeconst=".01"/>
      <position name="attached-fixed-1" tendon="attached-fixed-1" timeconst=".01"/>
    </actuator>

    <contact>
      <exclude body1="body" body2="targetbody"/>
      <exclude body1="attached-body-1" body2="attached-targetbody-1"/>
    </contact>

    <keyframe>
      <key name="two" time="2" qpos="2 22 0" act="2 2 0 0" ctrl="2 2 0 0"/>
      <key name="three" time="3" qpos="3 33 0" act="3 3 0 0" ctrl="3 3 0 0"/>
      <key name="attached-two-1" time="2" qpos="0 22 2" act="0 0 2 2" ctrl="0 0 2 2"/>
      <key name="attached-three-1" time="3" qpos="0 33 3" act="0 0 3 3" ctrl="0 0 3 3"/>
    </keyframe>
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
  mjsBody* attached = mjs_attachBody(frame, body, "attached-", "-1");
  EXPECT_THAT(attached, mjs_findBody(parent, "attached-body-1"));

  // check that the spec was not copied
  EXPECT_THAT(mjs_findSpec(parent, "child"), IsNull());

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

    <keyframe>
      <key name="one" time="1" qpos="1 1 1 1 0 0 0"/>
    </keyframe>
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

    <custom>
      <numeric data="10" name="attached-constant-1"/>
    </custom>

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
      <position name="attached-hinge-1" joint="attached-hinge-1" timeconst=".01"/>
      <position name="attached-fixed-1" tendon="attached-fixed-1" timeconst=".01"/>
    </actuator>

    <contact>
      <exclude body1="attached-body-1" body2="attached-targetbody-1"/>
    </contact>

    <keyframe>
      <key name="one" time="1" qpos="1 1 1 1 0 0 0 0"/>
      <key name="attached-two-1" time="2" qpos="0 0 0 1 0 0 0 2" act="2 2" ctrl="2 2"/>
      <key name="attached-three-1" time="3" qpos="0 0 0 1 0 0 0 3" act="3 3" ctrl="3 3"/>
    </keyframe>
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
  mjsBody* attached = mjs_attachBody(frame, body, "attached-", "-1");
  EXPECT_THAT(attached, mjs_findBody(parent, "attached-body-1"));

  // check that the spec was copied
  EXPECT_THAT(mjs_findSpec(parent, "child"), NotNull());

  // compile new model
  mjModel* m_attached = mj_compile(parent, 0);
  EXPECT_THAT(m_attached, NotNull());

  // check full name stored in mjModel
  EXPECT_STREQ(mj_id2name(m_attached, mjOBJ_BODY, 2), "attached-body-1");

  // check body 2 is attached to body 1
  EXPECT_THAT(m_attached->body_parentid[2], 1);

  // check that the correct defaults are present
  EXPECT_THAT(mjs_findDefault(parent, "main"), NotNull());
  EXPECT_THAT(mjs_findDefault(parent, "geom_size"), NotNull());
  EXPECT_THAT(mjs_findDefault(parent, "attached-cylinder-1"), NotNull());

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

    <keyframe>
      <key name="one" time="1" qpos="1 1 1 1 0 0 0"/>
    </keyframe>
  </mujoco>)";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <default>
      <default class="attached-cylinder-1">
        <geom type="cylinder" size=".1 1 0"/>
      </default>
    </default>

    <custom>
      <numeric data="10" name="attached-constant-1"/>
    </custom>

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
      <position name="attached-hinge-1" joint="attached-hinge-1" timeconst=".01"/>
      <position name="attached-fixed-1" tendon="attached-fixed-1" timeconst=".01"/>
    </actuator>

    <contact>
      <exclude body1="attached-body-1" body2="attached-targetbody-1"/>
    </contact>

    <keyframe>
      <key name="one" time="1" qpos="1 1 1 1 0 0 0 0"/>
      <key name="attached-two-1" time="2" qpos="0 0 0 1 0 0 0 2" act="2 2" ctrl="2 2"/>
      <key name="attached-three-1" time="3" qpos="0 0 0 1 0 0 0 3" act="3 3" ctrl="3 3"/>
    </keyframe>
  </mujoco>)";

  // model with one free sphere and a frame
  mjSpec* parent = mj_parseXMLString(xml_parent, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();

  // get body
  mjsBody* body = mjs_findBody(parent, "sphere");
  EXPECT_THAT(body, NotNull());

  // model with one cylinder and a hinge
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  // get subtree
  mjsFrame* frame = mjs_findFrame(child, "pframe");
  EXPECT_THAT(frame, NotNull());

  // attach child frame to parent body
  mjsFrame* attached = mjs_attachFrame(body, frame, "attached-", "-1");
  EXPECT_THAT(attached, mjs_findFrame(parent, "attached-pframe-1"));

  // check that the spec was copied
  EXPECT_THAT(mjs_findSpec(parent, "child"), NotNull());

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
  <mujoco model="child">
    <asset>
      <texture name="texture" type="2d" builtin="checker" width="32" height="32"/>
      <material name="material" texture="texture" texrepeat="1 1" texuniform="true"/>
    </asset>

    <custom>
      <numeric data="10" name="constant"/>
    </custom>

    <worldbody>
      <frame name="pframe">
        <frame name="cframe">
        </frame>
      </frame>
      <body name="ignore">
        <geom size=".1"/>
        <joint type="slide"/>
      </body>
      <frame name="frame" pos=".1 0 0" euler="0 90 0"/>
    </worldbody>

    <sensor>
      <framepos name="ignore" objtype="body" objname="ignore"/>
    </sensor>

    <keyframe>
      <key name="two" time="2" qpos="22"/>
      <key name="three" time="3" qpos="33"/>
    </keyframe>
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

TEST_F(MujocoTest, AttachToSite) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_parent[] = R"(
  <mujoco>
    <worldbody>
      <site name="site" pos="1 0 0" quat="0 1 0 0"/>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_child[] = R"(
  <mujoco>
    <worldbody>
      <body name="sphere">
        <joint type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <worldbody>
      <site name="site" pos="1 0 0" quat="0 1 0 0"/>
      <frame pos="1 0 0" quat="0 1 0 0">
        <body name="attached-sphere-1">
          <joint type="slide"/>
          <geom size=".1"/>
        </body>
      </frame>
    </worldbody>
  </mujoco>)";

  mjSpec* parent = mj_parseXMLString(xml_parent, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  mjsBody* world = mjs_findBody(parent, "world");
  EXPECT_THAT(world, NotNull());
  mjsSite* site = mjs_asSite(mjs_firstChild(world, mjOBJ_SITE, 0));
  EXPECT_THAT(site, NotNull());
  mjsBody* body = mjs_findBody(child, "sphere");
  EXPECT_THAT(body, NotNull());
  mjsBody* attached = mjs_attachToSite(site, body, "attached-", "-1");
  EXPECT_THAT(attached, NotNull());

  mjModel* model = mj_compile(parent, 0);
  EXPECT_THAT(model, NotNull());
  mjModel* expected = LoadModelFromString(xml_result, er.data(), er.size());
  EXPECT_THAT(expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(model, expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';

  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  mj_deleteModel(model);
  mj_deleteModel(expected);
}

TEST_F(MujocoTest, AttachWorld) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml_parent[] = R"(
  <mujoco>
    <worldbody>
      <frame name="frame"/>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_child[] = R"(
  <mujoco>
    <worldbody>
      <body name="sphere">
        <joint type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>)";

  static constexpr char xml_result[] = R"(
  <mujoco>
    <worldbody>
      <frame name="frame">
        <frame name="attached-world-1">
          <body name="attached-sphere-1">
            <joint type="slide"/>
            <geom size=".1"/>
          </body>
        </frame>
      </frame>
    </worldbody>
  </mujoco>)";

  mjSpec* parent = mj_parseXMLString(xml_parent, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  mjsFrame* frame = mjs_findFrame(parent, "frame");
  EXPECT_THAT(frame, NotNull());
  mjsBody* world = mjs_findBody(child, "world");
  EXPECT_THAT(world, NotNull());
  mjsBody* child_world = mjs_attachBody(frame, world, "attached-", "-1");
  EXPECT_THAT(child_world, NotNull());
  mjsFrame* frame_world = mjs_bodyToFrame(&child_world);
  EXPECT_THAT(frame_world, NotNull());
  EXPECT_THAT(child_world, IsNull());

  mjModel* model = mj_compile(parent, 0);
  EXPECT_THAT(model, NotNull());
  mjModel* expected = LoadModelFromString(xml_result, er.data(), er.size());
  EXPECT_THAT(expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(model, expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';

  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  mj_deleteModel(model);
  mj_deleteModel(expected);
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
      <body name="mocap_detach" mocap="true"/>
      <body name="mocap" mocap="true"/>
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
      <body name="mocap" mocap="true"/>
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

  // set mocap
  data->mocap_pos[3] = 1;
  data->mocap_quat[4] = 0;
  data->mocap_quat[5] = 1;
  d_expected->mocap_pos[0] = 1;
  d_expected->mocap_quat[0] = 0;
  d_expected->mocap_quat[1] = 1;

  // step models
  mj_step(model, data);
  mj_step(m_expected, d_expected);
  EXPECT_THAT(data->time, model->opt.timestep);

  // detach subtree
  mjsBody* body = mjs_findBody(spec, "detachable");
  EXPECT_THAT(body, NotNull());
  EXPECT_THAT(mjs_detachBody(spec, body), 0);

  // detach mocap
  mjsBody* mocap_body = mjs_findBody(spec, "mocap_detach");
  EXPECT_THAT(mocap_body, NotNull());
  EXPECT_THAT(mjs_detachBody(spec, mocap_body), 0);

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
  EXPECT_THAT(data->time, model->opt.timestep);

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

  // compare mocap
  EXPECT_EQ(model->nmocap, m_expected->nmocap);
  for (int i = 0; i < model->nmocap; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(data->mocap_pos[3*i+j], d_expected->mocap_pos[3*i+j]) << i;
    }
    for (int j = 0; j < 4; ++j) {
      EXPECT_EQ(data->mocap_quat[4*i+j], d_expected->mocap_quat[4*i+j]) << i;
    }
  }

  // check that the function is callable with no data
  mj_deleteData(data);
  mj_recompile(spec, 0, model, nullptr);

  // destroy everything
  mj_deleteData(d_expected);
  mj_deleteSpec(spec);
  mj_deleteModel(model);
  mj_deleteModel(m_expected);
}

TEST_F(MujocoTest, RecompileAttach) {
  std::array<char, 1000> er;
  std::string field = "";

  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".2"/>
      </body>
    </worldbody>
  </mujoco>)";

  mjSpec* parent = mj_makeSpec();
  EXPECT_THAT(parent, NotNull());

  mjSpec* child = mj_parseXMLString(xml, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull());

  mjsFrame* frame1 = mjs_addFrame(mjs_findBody(parent, "world"), 0);
  mjs_attachBody(frame1, mjs_findBody(child, "body"), "child-", "-1");

  mjModel* model = mj_compile(parent, 0);
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->nq, 1);
  mjData* data = mj_makeData(model);
  EXPECT_THAT(data, NotNull());

  for (int i = 0; i < 100; i++) {
    mj_step(model, data);
  }

  mjsFrame* frame2 = mjs_addFrame(mjs_findBody(parent, "world"), 0);
  mjs_attachBody(frame2, mjs_findBody(child, "body"), "child-", "-2");

  EXPECT_EQ(mj_recompile(parent, 0, model, data), 0);
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->nq, 2);
  EXPECT_NE(data->qpos[0], data->qpos[1]);
  EXPECT_EQ(data->qpos[1], 0);

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(child);
  mj_deleteSpec(parent);
}

TEST_F(MujocoTest, AttachMocap) {
  std::array<char, 1000> er;
  mjtNum tol = 0;
  std::string field = "";

  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="1 1 1" quat="0 1 0 0" name="mocap" mocap="true"/>
    </worldbody>
    <keyframe>
      <key name="key" time="1" mpos="2 2 2" mquat="0 0 0 1"/>
    </keyframe>
  </mujoco>)";

  static constexpr char xml_expected[] = R"(
  <mujoco>
    <worldbody>
      <body pos="1 1 1" quat="0 1 0 0" name="mocap" mocap="true"/>
      <body pos="1 1 1" quat="0 1 0 0" name="attached-mocap-1" mocap="true"/>
    </worldbody>
    <keyframe>
      <key name="key" time="1" mpos="2 2 2 1 1 1" mquat="0 0 0 1 0 1 0 0"/>
      <key name="attached-key-1" time="1" mpos="1 1 1 2 2 2" mquat="0 1 0 0 0 0 0 1"/>
    </keyframe>
  </mujoco>)";

  mjSpec* spec = mj_parseXMLString(xml, 0, er.data(), er.size());
  EXPECT_THAT(spec, NotNull()) << er.data();

  mjsBody* body = mjs_findBody(spec, "mocap");
  EXPECT_THAT(body, NotNull());

  mjsBody* world = mjs_findBody(spec, "world");
  EXPECT_THAT(world, NotNull());

  mjsFrame* frame = mjs_addFrame(world, NULL);
  mjs_attachBody(frame, body, "attached-", "-1");

  mjsBody* attached_body = mjs_findBody(spec, "attached-mocap-1");
  EXPECT_THAT(attached_body, NotNull());

  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull());

  mjModel* m_expected = LoadModelFromString(xml_expected, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(model, m_expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';

  mj_deleteSpec(spec);
  mj_deleteModel(model);
  mj_deleteModel(m_expected);
}

TEST_F(MujocoTest, ReplicateKeyframe) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <replicate count="1" euler="0 0 1.8">
        <body name="body" pos="0 -1 0">
          <joint type="slide"/>
          <geom name="g" size="1"/>
        </body>
      </replicate>
    </worldbody>

    <keyframe>
      <key name="keyframe" qpos="1"/>
    </keyframe>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, testing::NotNull()) << error.data();
  EXPECT_THAT(m->ngeom, 1);
  EXPECT_THAT(m->nbody, 2);

  // check that the keyframe is resized
  EXPECT_THAT(m->nkey, 2);
  EXPECT_THAT(m->nq, 1);
  EXPECT_THAT(m->key_qpos[0], 1);
  EXPECT_THAT(m->key_qpos[1], 0);
  EXPECT_STREQ(mj_id2name(m, mjOBJ_KEY, 0), "keyframe0");
  EXPECT_STREQ(mj_id2name(m, mjOBJ_KEY, 1), "keyframe");

  mj_deleteModel(m);
}

TEST_F(MujocoTest, AttachUnnamedAssets) {
  static constexpr char cube[] = R"(
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
  mj_addBufferVFS(vfs.get(), "cube.obj", cube, sizeof(cube));

  mjSpec* child = mj_makeSpec();
  mjsMesh* mesh = mjs_addMesh(child, 0);
  mjsFrame* frame = mjs_addFrame(mjs_findBody(child, "world"), 0);
  mjsGeom* geom = mjs_addGeom(mjs_findBody(child, "world"), 0);
  mjs_setFrame(geom->element, frame);
  mjs_setString(mesh->file, "cube.obj");
  mjs_setString(geom->meshname, "cube");
  geom->type = mjGEOM_MESH;

  mjSpec* spec = mj_makeSpec();
  mjs_attachFrame(mjs_findBody(spec, "world"), frame, "_", "");

  mjModel* model = mj_compile(spec, vfs.get());
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->nmesh, 1);
  EXPECT_STREQ(mj_id2name(model, mjOBJ_MESH, 0), "_cube");

  mj_deleteVFS(vfs.get());
  mj_deleteSpec(child);
  mj_deleteSpec(spec);
  mj_deleteModel(model);
}

TEST_F(MujocoTest, InitTexture) {
  mjSpec* spec = mj_makeSpec();
  EXPECT_THAT(spec, NotNull());

  mjsTexture* texture = mjs_addTexture(spec);
  mjs_setString(texture->name, "checker");
  texture->type = mjTEXTURE_CUBE;
  texture->builtin = mjBUILTIN_CHECKER;
  texture->width = 300;
  texture->height = 300;

  mjsMaterial* material = mjs_addMaterial(spec, 0);
  mjs_setString(material->name, "floor");
  mjs_setInStringVec(material->textures, mjTEXROLE_RGB, "checker");

  mjsGeom* floor = mjs_addGeom(mjs_findBody(spec, "world"), 0);
  mjs_setString(floor->material, "floor");
  floor->type = mjGEOM_PLANE;
  floor->size[0] = 1;
  floor->size[1] = 1;
  floor->size[2] = 0.01;
  mjs_setString(floor->material, "floor");

  mjModel* model = mj_compile(spec, 0);
  EXPECT_THAT(model, NotNull());

  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

void AttachNestedKeyframe(bool compile) {
  static constexpr char parent_xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
      <frame name="frame"/>
    </worldbody>
  </mujoco>)";

  static constexpr char child_xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <joint name="joint" type="slide"/>
        <geom size=".2"/>
        <frame name="frame"/>
      </body>
    </worldbody>
    <keyframe>
      <key name="key2" time="2" qpos="2"/>
    </keyframe>
  </mujoco>)";

  static constexpr char gchild_xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <joint name="joint" type="slide"/>
        <geom size=".3"/>
      </body>
    </worldbody>
    <keyframe>
      <key name="key1" time="1" qpos="1"/>
    </keyframe>
  </mujoco>)";

  static constexpr char expected_xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
      <frame name="frame">
        <body name="child-body">
          <joint name="child-joint" type="slide"/>
          <geom size=".2"/>
          <frame name="child-frame">
            <body name="child-gchild-body">
              <joint name="child-gchild-joint" type="slide"/>
              <geom size=".3"/>
            </body>
          </frame>
        </body>
      </frame>
    </worldbody>
    <keyframe>
      <key name="child-key2" time="2" qpos="0 2 0"/>
      <key name="child-gchild-key1" time="1" qpos="0 0 1"/>
    </keyframe>
  </mujoco>)";

  static constexpr char expected_xml_uncompiled[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
      <frame name="frame">
        <body name="child-body">
          <joint name="child-joint" type="slide"/>
          <geom size=".2"/>
          <frame name="child-frame">
            <body name="child-gchild-body">
              <joint name="child-gchild-joint" type="slide"/>
              <geom size=".3"/>
            </body>
          </frame>
        </body>
      </frame>
    </worldbody>
    <keyframe>
      <key name="child-key2" time="2" qpos="0 2 0"/>
      <key name="gchild-key1" time="1" qpos="0 0 1"/>
    </keyframe>
  </mujoco>)";

  std::array<char, 1000> er;
  mjSpec* parent = mj_parseXMLString(parent_xml, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();
  mjSpec* child = mj_parseXMLString(child_xml, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();
  mjSpec* gchild = mj_parseXMLString(gchild_xml, 0, er.data(), er.size());
  EXPECT_THAT(gchild, NotNull()) << er.data();

  // attach gchild to child
  mjs_attachBody(mjs_findFrame(child, "frame"),
                 mjs_findBody(gchild, "body"), "gchild-", "");

  // compile required before further attachment
  mjModel* m_child = compile ? mj_compile(child, 0) : nullptr;

  // check warning is issued, empty for a compiled model
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
      util::strcpy_arr(warning, msg);
  };

  // attach child to parent
  mjs_attachBody(mjs_findFrame(parent, "frame"),
                 mjs_findBody(child, "body"), "child-", "");

  EXPECT_THAT(warning, HasSubstr(compile ? "" : "model has pending keyframes"));

  // compare models
  mjtNum tol = 0;
  std::string field = "";
  mjModel* m_attached = mj_compile(parent, 0);
  EXPECT_THAT(m_attached, NotNull());
  mjModel* m_expected = LoadModelFromString(
      compile ? expected_xml : expected_xml_uncompiled, er.data(), er.size());
  EXPECT_THAT(m_expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(m_attached, m_expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';;

  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  mj_deleteSpec(gchild);
  mj_deleteModel(m_expected);
  mj_deleteModel(m_attached);
  mj_deleteModel(m_child);
}

TEST_F(MujocoTest, TestAttachNestedKeyframe) {
  AttachNestedKeyframe(/*compile=*/true);
  AttachNestedKeyframe(/*compile=*/false);
}

TEST_F(MujocoTest, RepeatedAttachKeyframe) {
  static constexpr char xml_1[] = R"(
    <mujoco model="MuJoCo Model">
      <worldbody>
        <body name="body"/>
      </worldbody>
    </mujoco>)";

  static constexpr char xml_2[] = R"(
    <mujoco model="MuJoCo Model">
      <worldbody>
        <body name="b1">
          <joint/>
          <geom size="0.1"/>
        </body>
        <body name="b2">
          <joint/>
          <geom size="0.1"/>
        </body>
      </worldbody>
      <keyframe>
        <key name="home" qpos="1 2" />
      </keyframe>
    </mujoco>)";

    std::array<char, 1000> er;
    mjSpec* spec_1 = mj_parseXMLString(xml_1, 0, er.data(), er.size());
    EXPECT_THAT(spec_1, NotNull()) << er.data();
    mjSpec* spec_2 = mj_parseXMLString(xml_2, 0, er.data(), er.size());
    EXPECT_THAT(spec_2, NotNull()) << er.data();

    mjsBody* body_1 = mjs_findBody(spec_1, "body");
    mjsFrame* attachment_frame = mjs_addFrame(body_1, 0);
    mjs_attachBody(attachment_frame, mjs_findBody(spec_2, "b1"), "b1-", "");
    mjModel* model_1 = mj_compile(spec_1, 0);
    EXPECT_THAT(model_1, NotNull());
    mjs_attachBody(attachment_frame, mjs_findBody(spec_2, "b2"), "b2-", "");
    mjModel* model_2 = mj_compile(spec_1, 0);
    EXPECT_THAT(model_2, NotNull());

    EXPECT_EQ(model_1->nkey, 1);
    EXPECT_EQ(model_2->nkey, 2);
    EXPECT_STREQ(mj_id2name(model_2, mjOBJ_KEY, 0), "b1-home");
    EXPECT_STREQ(mj_id2name(model_2, mjOBJ_KEY, 1), "b2-home");

    mj_deleteSpec(spec_1);
    mj_deleteSpec(spec_2);
    mj_deleteModel(model_1);
    mj_deleteModel(model_2);
}

TEST_F(MujocoTest, ResizeParentKeyframe) {
  static constexpr char xml_parent[] = R"(
    <mujoco model="MuJoCo Model">
      <worldbody>
        <frame name="frame"/>
        <body name="body">
          <joint/>
          <geom size="0.1"/>
        </body>
      </worldbody>
      <keyframe>
        <key name="home" qpos="1"/>
      </keyframe>
    </mujoco>)";

  static constexpr char xml_child[] = R"(
    <mujoco model="MuJoCo Model">
      <worldbody>
        <body name="body">
          <joint/>
          <geom size="0.1"/>
        </body>
      </worldbody>
    </mujoco>)";

  static constexpr char xml_expected[] = R"(
    <mujoco model="MuJoCo Model">
      <worldbody>
        <body name="body">
          <joint/>
          <geom size="0.1"/>
        </body>
        <frame name="frame">
          <body name="child-body">
            <joint/>
            <geom size="0.1"/>
          </body>
        </frame>
      </worldbody>
      <keyframe>
        <key name="home" qpos="1 0"/>
      </keyframe>
    </mujoco>)";

  std::array<char, 1000> er;
  mjSpec* parent = mj_parseXMLString(xml_parent, 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();
  mjSpec* child = mj_parseXMLString(xml_child, 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  mjs_attachBody(mjs_findFrame(parent, "frame"), mjs_findBody(child, "body"),
                 "child-", "");

  mjModel* model = mj_compile(parent, 0);
  EXPECT_THAT(model, NotNull());

  mjtNum tol = 0;
  std::string field = "";
  mjModel* expected = LoadModelFromString(xml_expected, er.data(), er.size());
  EXPECT_THAT(expected, NotNull()) << er.data();
  EXPECT_LE(CompareModel(model, expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';

  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  mj_deleteModel(model);
  mj_deleteModel(expected);
}

TEST_F(MujocoTest, DifferentUnitsAllowed) {
  mjSpec* child = mj_makeSpec();
  child->compiler.degree = 1;
  mjsBody* body = mjs_addBody(mjs_findBody(child, "world"), 0);
  body->alt.type = mjORIENTATION_EULER;
  body->alt.euler[0] = 90;

  mjSpec* parent = mj_makeSpec();
  parent->compiler.degree = 0;
  mjsFrame* frame = mjs_addFrame(mjs_findBody(parent, "world"), 0);
  frame->alt.type = mjORIENTATION_EULER;
  frame->alt.euler[0] = -mjPI / 2;

  EXPECT_THAT(mjs_attachBody(frame, body, "child-", ""), NotNull());
  mjModel* model = mj_compile(parent, 0);
  EXPECT_THAT(model, NotNull());
  EXPECT_NEAR(model->body_quat[4], 1, 1e-12);
  EXPECT_NEAR(model->body_quat[5], 0, 1e-12);
  EXPECT_NEAR(model->body_quat[6], 0, 1e-12);
  EXPECT_NEAR(model->body_quat[7], 0, 1e-12);

  mjSpec* copy = mj_copySpec(parent);
  EXPECT_THAT(copy, NotNull());
  mj_deleteModel(model);
  mj_deleteSpec(child);
  mj_deleteSpec(parent);

  // check that deleting `parent` or `child` does not invalidate the copy
  mjModel* copy_model = mj_compile(copy, 0);
  EXPECT_THAT(copy_model, NotNull());
  EXPECT_NEAR(copy_model->body_quat[0], 1, 1e-12);
  EXPECT_NEAR(copy_model->body_quat[1], 0, 1e-12);
  EXPECT_NEAR(copy_model->body_quat[2], 0, 1e-12);
  EXPECT_NEAR(copy_model->body_quat[3], 0, 1e-12);

  mj_deleteModel(copy_model);
  mj_deleteSpec(copy);
}

TEST_F(MujocoTest, CopyAttachedSpec) {
  static constexpr char xml_parent[] = R"(
  <mujoco>
    <asset>
      <model name="child" file="xml_child.xml"/>
    </asset>
    <worldbody>
      <body name="parent">
        <geom name="geom" size="2"/>
        <attach model="child" body="body" prefix="other"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char xml_child[] = R"(
  <mujoco>
    <worldbody>
      <body name="body">
        <geom name="geom" size="1" pos="2 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "xml_child.xml", xml_child, sizeof(xml_child));

  std::array<char, 1024> er;
  mjSpec* spec = mj_parseXMLString(xml_parent, vfs.get(), er.data(), er.size());
  EXPECT_THAT(spec, NotNull()) << er.data();

  mjModel* model = mj_compile(spec, vfs.get());
  EXPECT_THAT(model, NotNull()) << er.data();

  mjSpec* child = mjs_findSpec(spec, "child");
  EXPECT_THAT(child, NotNull());

  mjSpec* copy = mj_copySpec(spec);
  EXPECT_THAT(copy, NotNull());

  mjSpec* child_copy = mjs_findSpec(copy, "child");
  EXPECT_THAT(child_copy, NotNull());
  EXPECT_NE(child_copy, child);

  mj_deleteSpec(spec);
  mj_deleteSpec(copy);
  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

}  // namespace
}  // namespace mujoco
