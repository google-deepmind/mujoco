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

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

// -------------------- test OS filesystem fallback ----------------------------

using VfsTest = MujocoTest;

TEST_F(VfsTest, HFieldPngWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hfield" file="unknown_file.png" size="0.5 0.5 1 0.1"/>
    </asset>

    <worldbody>
      <geom type="hfield" hfield="hfield" pos="-.4 .6 .05"/>
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
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
}

TEST_F(VfsTest, HFieldCustomWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hfield" file="unknown_file" size="0.5 0.5 1 0.1"/>
    </asset>

    <worldbody>
      <geom type="hfield" hfield="hfield" pos="-.4 .6 .05"/>
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
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
}

TEST_F(VfsTest, TexturePngWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" file="unknown_file.png" type="2d"/>
      <material name="material" texture="texture"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
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
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
 }

TEST_F(VfsTest, TextureCustomWithVFS) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" file="unknown_file" type="2d"/>
      <material name="material" texture="texture"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
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
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
 }

// ------------------------ test content_type attribute ------------------------

using ContentTypeTest = MujocoTest;

TEST_F(ContentTypeTest, HFieldPngWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hfield" content_type="image/png"
              file="some_file.png" size="0.5 0.5 1 0.1"/>
    </asset>

    <worldbody>
      <geom type="hfield" hfield="hfield" pos="-.4 .6 .05"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try loading the file
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
}

TEST_F(ContentTypeTest, HFieldCustomWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hfield" content_type="image/vnd.mujoco.hfield"
              file="some_file" size="0.5 0.5 1 0.1"/>
    </asset>

    <worldbody>
      <geom type="hfield" hfield="hfield" pos="-.4 .6 .05"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try loading the file
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
}

TEST_F(ContentTypeTest, HFieldWithContentTypeError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hfield" content_type="image/jpeg"
              file="some_file" size="0.5 0.5 1 0.1"/>
    </asset>

    <worldbody>
      <geom type="hfield" hfield="hfield" pos="-.4 .6 .05"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try loading the file
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("unsupported content type: 'image/jpeg'"));
}

TEST_F(ContentTypeTest, TexturePngWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" content_type="image/png" file="some_file" type="2d"/>
      <material name="material" texture="texture"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try loading the file
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
 }

TEST_F(ContentTypeTest, TextureCustomWithContentType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" content_type="image/vnd.mujoco.texture"
              file="some_file" type="2d"/>
      <material name="material" texture="texture"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try loading the file
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("resource not found via provider or OS filesystem"));
 }

TEST_F(ContentTypeTest, TextureWithContentTypeError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" content_type="image/jpeg"
              file="some_file" type="2d"/>
      <material name="material" texture="texture"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  size_t error_sz = 1024;

  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  // should try loading the file
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("unsupported content type: 'image/jpeg'"));
 }

TEST_F(ContentTypeTest, TextureLoadPng) {
  static constexpr char filename[] = "tiny";
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture content_type="image/png" file="tiny" type="2d"/>
      <material name="material" texture="tiny"/>
    </asset>

    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  // credit: https://www.mjt.me.uk/posts/smallest-png/
  static constexpr unsigned char tiny[] =
      { 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00,
        0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x03, 0x00,
        0x00, 0x00, 0x66, 0xBC, 0x3A, 0x25, 0x00, 0x00, 0x00,
        0x03, 0x50, 0x4C, 0x54, 0x45, 0xB5, 0xD0, 0xD0, 0x63,
        0x04, 0x16, 0xEA, 0x00, 0x00, 0x00, 0x1F, 0x49, 0x44,
        0x41, 0x54, 0x68, 0x81, 0xED, 0xC1, 0x01, 0x0D, 0x00,
        0x00, 0x00, 0xC2, 0xA0, 0xF7, 0x4F, 0x6D, 0x0E, 0x37,
        0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xBE, 0x0D, 0x21, 0x00, 0x00, 0x01, 0x9A, 0x60, 0xE1,
        0xD5, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44,
        0xAE, 0x42, 0x60, 0x82 };
  size_t tiny_sz = sizeof(tiny);

  char error[1024];
  size_t error_sz = 1024;


  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_makeEmptyFileVFS(vfs.get(), filename, 105);
  int i = mj_findFileVFS(vfs.get(), filename);
  memcpy(vfs->filedata[i], tiny, tiny_sz);

  // loading the file should be successful
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, NotNull());

  mj_deleteModel(model);
  mj_deleteFileVFS(vfs.get(), filename);
 }

// ------------------------ test keyframes -------------------------------------

using KeyframeTest = MujocoTest;

constexpr char kKeyframePath[] = "user/testdata/keyframe.xml";

TEST_F(KeyframeTest, CheckValues) {
  const std::string xml_path = GetTestDataFilePath(kKeyframePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->nkey, 7);
  EXPECT_EQ(model->key_time[0 * 1], 0.1);
  EXPECT_EQ(model->key_qpos[1 * model->nq], 0.2);
  EXPECT_EQ(model->key_qvel[2 * model->nv], 0.3);
  EXPECT_EQ(model->key_act[3 * model->na], 0.4);
  EXPECT_THAT(AsVector(model->key_ctrl + 4*model->nu, model->nu),
              ElementsAre(0.5, 0.6));
  EXPECT_THAT(AsVector(model->key_mpos + 3*model->nmocap*5, 3),
              ElementsAre(.1, .2, .3));
  EXPECT_THAT(AsVector(model->key_mquat + 4*model->nmocap*6, 4),
              ElementsAre(.5, .5, .5, .5));
  mj_deleteModel(model);
}

TEST_F(KeyframeTest, ResetDataKeyframe) {
  const std::string xml_path = GetTestDataFilePath(kKeyframePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  mj_resetDataKeyframe(model, data, 0);
  EXPECT_EQ(data->time, 0.1);

  mj_resetDataKeyframe(model, data, 1);
  EXPECT_EQ(data->qpos[0], 0.2);

  mj_resetDataKeyframe(model, data, 2);
  EXPECT_EQ(data->qvel[0], 0.3);

  mj_resetDataKeyframe(model, data, 3);
  EXPECT_EQ(data->act[0], 0.4);

  mj_resetDataKeyframe(model, data, 4);
  EXPECT_EQ(data->ctrl[0], 0.5);
  EXPECT_EQ(data->ctrl[1], 0.6);

  mj_resetDataKeyframe(model, data, 5);
  EXPECT_THAT(AsVector(data->mocap_pos, 3), ElementsAre(.1, .2, .3));

  mj_resetDataKeyframe(model, data, 6);
  EXPECT_THAT(AsVector(data->mocap_quat, 4), ElementsAre(.5, .5, .5, .5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(KeyframeTest, BadSize) {
  static constexpr char xml[] = R"(
  <mujoco>
    <keyframe>
      <key qpos="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = LoadModelFromString(xml, error, error_sz);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("invalid qpos size, expected length 0"));
}

// ------------- test relative frame sensor compilation-------------------------

using RelativeFrameSensorParsingTest = MujocoTest;

TEST_F(RelativeFrameSensorParsingTest, RefTypeNotRequired) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}

TEST_F(RelativeFrameSensorParsingTest, ReferenceBodyFound) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="xbody" objname="sensorized"
                reftype="xbody" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  ASSERT_EQ(model->sensor_reftype[0], mjOBJ_XBODY);
  ASSERT_EQ(model->sensor_refid[0], 1);
  mj_deleteModel(model);
}

TEST_F(RelativeFrameSensorParsingTest, MissingRefname) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"
                reftype="body" refname=""/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("missing name of reference frame"));
}

TEST_F(RelativeFrameSensorParsingTest, BadRefName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"
                reftype="body" refname="wrong_name"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("unrecognized name of reference frame"));
}

TEST_F(RelativeFrameSensorParsingTest, BadRefType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <light name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"
                reftype="light" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("reference frame object must be"));
}

// ------------- sensor compilation --------------------------------------------

using SensorTest = MujocoTest;
TEST_F(SensorTest, OjbtypeParsedButNotRequired) {
  static constexpr char xml[] = R"(
  <mujoco>
    <sensor>
      <user dim="3" needstage="vel" datatype="axis"/>
      <user dim="2" needstage="pos" objtype="body" objname="world"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->sensor_datatype[0], mjDATATYPE_AXIS);
  EXPECT_EQ(model->sensor_objtype[0], mjOBJ_UNKNOWN);
  EXPECT_EQ(model->sensor_dim[0], 3);
  EXPECT_EQ(model->sensor_datatype[1], mjDATATYPE_REAL);
  EXPECT_EQ(model->sensor_objtype[1], mjOBJ_BODY);
  EXPECT_EQ(model->sensor_objid[1], 0);
  mj_deleteModel(model);
}

// ------------- test capsule inertias -----------------------------------------

static const char* const kCapsuleInertiaPath =
    "user/testdata/capsule_inertia.xml";

using MjCGeomTest = MujocoTest;

static constexpr int kSphereBodyId = 1, kCylinderBodyId = 2,
                     kCapsuleBodyId = 3, kCapsuleGeomId = 2;

TEST_F(MjCGeomTest, CapsuleMass) {
  const std::string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  // Mass of capsule should equal mass of cylinder + mass of sphere.
  mjtNum sphere_cylinder_mass =
      model->body_mass[kSphereBodyId] + model->body_mass[kCylinderBodyId];
  mjtNum capsule_mass = model->body_mass[kCapsuleBodyId];
  EXPECT_DOUBLE_EQ(sphere_cylinder_mass, capsule_mass);
  mj_deleteModel(model);
}

TEST_F(MjCGeomTest, CapsuleInertiaZ) {
  const std::string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  // z-inertia of capsule should equal sphere + cylinder z-inertia.
  mjtNum sphere_cylinder_z_inertia =
      model->body_inertia[3*kSphereBodyId + 2] +
      model->body_inertia[3*kCylinderBodyId + 2];
  mjtNum capsule_z_inertia = model->body_inertia[3*kCapsuleBodyId + 2];
  EXPECT_DOUBLE_EQ(sphere_cylinder_z_inertia, capsule_z_inertia);
  mj_deleteModel(model);
}

TEST_F(MjCGeomTest, CapsuleInertiaX) {
  const std::string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);

  // The CoM of a solid hemisphere is 3/8*radius away from from the disk.
  mjtNum hs_com = model->geom_size[3*kCapsuleGeomId] * 3 / 8;

  // The mass of the two hemispherical end-caps is just the mass of the sphere.
  mjtNum sphere_mass = model->body_mass[1];

  // x-inertia of capsule should equal sphere + cylinder x-inertias, with
  // corrections from shifting the hemispheres using parallel axis theorem.
  mjtNum sphere_cylinder_x_inertia = model->body_inertia[3*kSphereBodyId] +
                                     model->body_inertia[3*kCylinderBodyId];

  // Parallel axis-theorem #1: translate the hemispheres in to the origin.
  mjtNum translate_in = hs_com;
  sphere_cylinder_x_inertia -= sphere_mass * translate_in*translate_in;

  // Parallel axis-theorem #2: translate the hemispheres out to the end caps.
  mjtNum cylinder_half_length = model->geom_size[3*kCapsuleGeomId + 1];
  mjtNum translate_out = cylinder_half_length + hs_com;
  sphere_cylinder_x_inertia += sphere_mass * translate_out*translate_out;

  // Compare native capsule inertia and computed inertia.
  mjtNum capsule_x_inertia = model->body_inertia[3*kCapsuleBodyId];
  EXPECT_DOUBLE_EQ(sphere_cylinder_x_inertia, capsule_x_inertia);
  mj_deleteModel(model);
}

// ------------- test inertiagrouprange ----------------------------------------

TEST_F(MjCGeomTest, IgnoreGeomOutsideInertiagrouprange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler inertiagrouprange="0 1"/>
    <worldbody>
      <body>
        <geom size="1" group="3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml, nullptr, 0);
  EXPECT_THAT(m->body_mass[1], 0);
  mj_deleteModel(m);
}

TEST_F(MjCGeomTest, IgnoreBadGeomOutsideInertiagrouprange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler inertiagrouprange="0 1"/>
    <asset>
      <mesh name="malformed_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="2 0 3  0 1 3  1 2 3  0 1 2" />
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="malformed_mesh" group="3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml, nullptr, 0);
  EXPECT_THAT(m->body_mass[1], 0);
  mj_deleteModel(m);
}

// ------------- test invalid size values --------------------------------------

TEST_F(MjCGeomTest, NanSize) {
  // even if the caller ignores warnings, models shouldn't compile with NaN
  // geom sizes
  mju_user_warning = nullptr;
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="box" size="1 1 nan" mass="1" />
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, testing::IsNull());
  ASSERT_THAT(error.data(), HasSubstr("nan"));
}

// ------------- test height fields --------------------------------------------

using MjCHFieldTest = MujocoTest;

TEST_F(MjCHFieldTest, PngMap) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/png_hfield.xml");
  std::array<char, 1024> error;
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->nhfield, 1);
  EXPECT_EQ(model->geom_type[0], mjGEOM_HFIELD);
  EXPECT_GT(model->nhfielddata, 0);

  float min_hfield = 1e7;
  float max_hfield = -1e7;
  for (int i = 0; i < model->nhfielddata; i++) {
    float v = model->hfield_data[i];
    min_hfield = std::min(min_hfield, v);
    max_hfield = std::max(max_hfield, v);
  }
  EXPECT_EQ(min_hfield, 0) << "hfield should be normalised to [0, 1]";
  EXPECT_EQ(max_hfield, 1) << "hfield should be normalised to [0, 1]";

  mj_deleteModel(model);
}

// ------------- test quaternion normalization----------------------------------

using QuatNorm = MujocoTest;

TEST_F(QuatNorm, QuatNotNormalized) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site quat="1 2 2 4"/>
      <camera quat="1 2 2 4"/>
      <body quat="1 2 2 4">
        <geom quat="1 2 2 4" size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(AsVector(m->body_quat+4, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  EXPECT_THAT(AsVector(m->geom_quat, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  EXPECT_THAT(AsVector(m->site_quat, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  EXPECT_THAT(AsVector(m->cam_quat, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  mj_deleteModel(m);
}

// ------------- test camera specifications ------------------------------------

using CameraSpecTest = MujocoTest;

TEST_F(CameraSpecTest, FovyLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <camera fovy="180"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("fovy too large"));
  mj_deleteModel(m);
}

TEST_F(CameraSpecTest, DuplicatedFocalIgnorePixel) {
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <map znear="0.01"/>
    </visual>
    <worldbody>
      <body>
        <geom size="1"/>
        <camera focal="8e-3 8e-3" focalpixel="100 100"
                resolution="1920 1200" sensorsize="9.6e-3 6e-3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_NEAR(m->cam_intrinsic[0], 5e-4, 1e-6);  // focal length in meters (x)
  EXPECT_NEAR(m->cam_intrinsic[1], 5e-4, 1e-6);  // focal length in meters (y)
  mj_deleteModel(m);
}

TEST_F(CameraSpecTest, FovyFromResolution) {
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <map znear="0.01"/>
    </visual>
    <worldbody>
      <body>
        <geom size="1"/>
        <!-- 8mm focal length lenses -->
        <camera focal="8e-3 8e-3" resolution="1920 1200" sensorsize="9.6e-3 6e-3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_NEAR(m->cam_fovy[0], 41.112, 1e-3);
  EXPECT_NEAR(m->cam_intrinsic[0], 8e-3, 1e-6);  // focal length in meters (x)
  EXPECT_NEAR(m->cam_intrinsic[1], 8e-3, 1e-6);  // focal length in meters (y)
  EXPECT_EQ(m->cam_intrinsic[2], 0);  // principal point in meters (x)
  EXPECT_EQ(m->cam_intrinsic[3], 0);  // principal point in meters (y)
  mj_deleteModel(m);
}

TEST_F(CameraSpecTest, FovyFromResolutionPixel) {
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <map znear="0.01"/>
    </visual>
    <worldbody>
      <body>
        <geom size="1"/>
        <!-- 8mm focal length lenses -->
        <camera focalpixel="1600 1600" resolution="1920 1200" sensorsize="9.6e-3 6e-3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_NEAR(m->cam_fovy[0], 41.112, 1e-3);
  EXPECT_NEAR(m->cam_intrinsic[0], 8e-3, 1e-6);  // focal length in meters (x)
  EXPECT_NEAR(m->cam_intrinsic[1], 8e-3, 1e-6);  // focal length in meters (y)
  EXPECT_EQ(m->cam_intrinsic[2], 0);  // principal point in meters (x)
  EXPECT_EQ(m->cam_intrinsic[3], 0);  // principal point in meters (y)
  mj_deleteModel(m);
}

// ------------- test actuator order -------------------------------------------

using ActuatorTest = MujocoTest;

TEST_F(ActuatorTest, ActuatorOrderDoesntMatter) {
  static constexpr char xml1[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="filter"/>
      <general joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  static constexpr char xml2[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge"/>
      <general joint="hinge" dyntype="filter"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model1 = LoadModelFromString(xml1, nullptr, 0);
  mjData* data1 = mj_makeData(model1);
  mjModel* model2 = LoadModelFromString(xml2, nullptr, 0);
  mjData* data2 = mj_makeData(model2);

  // check activation indexing
  EXPECT_EQ(model1->actuator_actadr[0], 0);
  EXPECT_EQ(model1->actuator_actadr[1], -1);
  EXPECT_EQ(model2->actuator_actadr[0], -1);
  EXPECT_EQ(model2->actuator_actadr[1], 0);

  // integrate both models, flipping the controls
  while (data1->time < 1) {
    data1->ctrl[0] = data1->time;
    data1->ctrl[1] = -data1->time;
    mj_step(model1, data1);
  }
  while (data2->time < 1) {
    data2->ctrl[0] = -data2->time;
    data2->ctrl[1] = data2->time;
    mj_step(model2, data2);
  }

  // expect states to match exactly
  EXPECT_EQ(data1->qpos[0], data2->qpos[0]);
  EXPECT_EQ(data1->qvel[0], data2->qvel[0]);
  EXPECT_EQ(data1->act[0], data2->act[0]);

  mj_deleteData(data2);
  mj_deleteModel(model2);
  mj_deleteData(data1);
  mj_deleteModel(model1);
}


// ------------- test actlimited and actrange fields ---------------------------

using ActRangeTest = MujocoTest;

TEST_F(ActRangeTest, ActRangeParsed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="integrator" joint="hinge" actlimited="true" actrange="-1 1.5"/>
    </actuator>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml, nullptr, 0);
  EXPECT_EQ(m->actuator_actlimited[0], 1);
  EXPECT_EQ(m->actuator_actrange[0], -1);
  EXPECT_EQ(m->actuator_actrange[1], 1.5);

  mj_deleteModel(m);
}

TEST_F(ActRangeTest, ActRangeBad) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="integrator" joint="hinge" actlimited="true" actrange="1 -1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid activation range"));
}

TEST_F(ActRangeTest, ActRangeUndefined) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="integrator" joint="hinge" actlimited="true"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid activation range"));
}

TEST_F(ActRangeTest, ActRangeNoDyntype) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("actrange specified but dyntype is 'none'"));
}

TEST_F(ActRangeTest, ActRangeDefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <default>
      <general dyntype="integrator" actlimited="true" actrange="-1 1"/>
      <default class="dclass">
        <general actlimited="false" actrange="2 3"/>
      </default>
    </default>
    <worldbody>
      <body>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide"/>
      <general joint="slide" class="dclass"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  // first actuator
  EXPECT_THAT(model->actuator_actlimited[0], 1);
  EXPECT_THAT(model->actuator_actrange[0], -1);
  EXPECT_THAT(model->actuator_actrange[1], 1);

  // // second actuator
  EXPECT_THAT(model->actuator_actlimited[1], 0);
  EXPECT_THAT(model->actuator_actrange[2], 2);
  EXPECT_THAT(model->actuator_actrange[3], 3);

  mj_deleteModel(model);
}

// ---------------------------- test actdim ------------------------------------

using ActDimTest = MujocoTest;

TEST_F(ActDimTest, BiggerThanOneOnlyForUser) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" actdim="2"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("actdim > 1 is only allowed for dyntype 'user'"));
}

TEST_F(ActDimTest, NonzeroNotAllowedInStateless) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" actdim="1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid actdim 1 in stateless"));
}

TEST_F(ActDimTest, ZeroNotAllowedInStateful) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="filter" actdim="0"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid actdim 0 in stateful"));
}

// ------------- test nuser_xxx fields -----------------------------------------

using UserDataTest = MujocoTest;

TEST_F(UserDataTest, NBodyTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_body="2"/>
    <worldbody>
      <body user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_body"));
}

TEST_F(UserDataTest, NJointTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_jnt="2"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint user="1 2 3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_jnt"));
}

TEST_F(UserDataTest, NGeomTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_geom="2"/>
    <worldbody>
      <geom size="1" user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_geom"));
}

TEST_F(UserDataTest, NSiteTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_site="2"/>
    <worldbody>
      <site user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_site"));
}

TEST_F(UserDataTest, NCameraTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_cam="2"/>
    <worldbody>
      <camera user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_cam"));
}

TEST_F(UserDataTest, NTendonTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_tendon="2"/>
    <worldbody>
      <site name="a"/>
      <site name="b"/>
    </worldbody>
    <tendon>
      <spatial user="1 2 3">
        <site site="a"/>
        <site site="b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_tendon"));
}

TEST_F(UserDataTest, NActuatorTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_actuator="2"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="a"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="a" user="1 2 3"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_actuator"));
}

TEST_F(UserDataTest, NSensorTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_sensor="2"/>
    <worldbody>
      <site name="a"/>
    </worldbody>
    <sensor>
      <accelerometer site="a" user="1 2 3"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_sensor"));
}

// ------------- test for auto parsing of *limited fields ----------------------

using LimitedTest = MujocoTest;

constexpr char kKeyAutoLimits[] = "user/testdata/auto_limits.xml";

// check joint limit values when automatically inferred based on range
TEST_F(LimitedTest, JointLimited) {
  const std::string xml_path = GetTestDataFilePath(kKeyAutoLimits);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());

  // see `user/testdata/auto_limits.xml` for expected values
  for (int i=0; i < model->njnt; i++) {
    EXPECT_EQ(model->jnt_limited[i], (mjtByte)model->jnt_user[i]);
  }

  mj_deleteModel(model);
}

TEST_F(LimitedTest, ErrorIfLimitedMissingOnJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint user="1" range="0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("limited"));
}

TEST_F(LimitedTest, ExplicitLimitedFalseIsOk) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint user="1" limited="false" range="0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  EXPECT_EQ(model->jnt_limited[0], 0);
  mj_deleteModel(model);
}

TEST_F(LimitedTest, ErrorIfLimitedMissingOnTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint type="slide"/>
        <geom size="1"/>
        <site name="s1"/>
      </body>
      <site name="s2"/>
    </worldbody>
    <tendon>
      <spatial range="-1 1">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
    </tendon>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("limited"));
  EXPECT_THAT(error.data(), HasSubstr("tendon"));
}

TEST_F(LimitedTest, ErrorIfForceLimitedMissingOnActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint type="slide" name="J1"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="J1" forcerange="0 100"/>
    </actuator>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("forcelimited"));
  EXPECT_THAT(error.data(), HasSubstr("forcerange"));
  EXPECT_THAT(error.data(), HasSubstr("actuator"));
}

// ------------- tests for tendon springrange ----------------------------------

using SpringrangeTest = MujocoTest;

TEST_F(SpringrangeTest, DefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <tendon springlength=".2 .5"/>
    </default>

    <worldbody>
      <site name="0"/>
      <site name="1" pos="1 0 0"/>
    </worldbody>

    <tendon>
      <spatial>
        <site site="0"/>
        <site site="1"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->tendon_lengthspring[0], .2);
  EXPECT_EQ(model->tendon_lengthspring[1], .5);
  mj_deleteModel(model);
}

TEST_F(SpringrangeTest, InvalidRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="0"/>
      <site name="1" pos="1 0 0"/>
    </worldbody>

    <tendon>
      <spatial springlength="1 0">
        <site site="0"/>
        <site site="1"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid springlength in tendon"));
}

// ------------- test frame ----------------------------------------------------
TEST_F(MujocoTest, Frame) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <frame euler="0 0 30">
        <geom size=".1" euler="0 0 20"/>
      </frame>

      <frame axisangle="0 0 1 90">
        <frame axisangle="0 1 0 90">
          <geom size=".1"/>
        </frame>
      </frame>

      <body>
        <frame pos="0 1 0">
          <geom size=".1" pos="0 1 0"/>
          <body pos="1 0 0">
            <geom size=".1" pos="0 0 1"/>
          </body>
        </frame>
      </body>

      <body>
        <geom size=".1"/>
        <frame euler="90 0 0">
          <joint type="hinge" axis="0 0 1"/>
        </frame>
      </body>
    </worldbody>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, testing::NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 4);

  // geom quat transformed to euler = 0 0 50
  EXPECT_NEAR(m->geom_quat[0], mju_cos(25. * mjPI / 180.), 1e-3);
  EXPECT_NEAR(m->geom_quat[1], 0, 0);
  EXPECT_NEAR(m->geom_quat[2], 0, 0);
  EXPECT_NEAR(m->geom_quat[3], mju_sin(25. * mjPI / 180.), 1e-3);

  // geom transformed to frame 0 1 0, 0 0 1, 1 0 0
  EXPECT_NEAR(m->geom_quat[4], .5, 1e-6);
  EXPECT_NEAR(m->geom_quat[5], .5, 1e-6);
  EXPECT_NEAR(m->geom_quat[6], .5, 1e-6);
  EXPECT_NEAR(m->geom_quat[7], .5, 1e-6);

  // geom pos transformed from 0 1 0 to 0 2 0
  EXPECT_EQ(m->geom_pos[6], 0);
  EXPECT_EQ(m->geom_pos[7], 2);
  EXPECT_EQ(m->geom_pos[8], 0);

  // body pos transformed from 1 0 0 to 1 1 0
  EXPECT_EQ(m->body_pos[6], 1);
  EXPECT_EQ(m->body_pos[7], 1);
  EXPECT_EQ(m->body_pos[8], 0);

  // nested geom pos not transformed
  EXPECT_EQ(m->geom_pos[ 9], 0);
  EXPECT_EQ(m->geom_pos[10], 0);
  EXPECT_EQ(m->geom_pos[11], 1);

  // joint axis transformed to 0 -1 0
  EXPECT_NEAR(m->jnt_axis[0],  0, 1e-6);
  EXPECT_NEAR(m->jnt_axis[1], -1, 1e-6);
  EXPECT_NEAR(m->jnt_axis[2],  0, 1e-6);

  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
