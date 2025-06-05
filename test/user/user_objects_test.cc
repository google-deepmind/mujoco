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
#include <cstring>
#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

constexpr double kInertiaTol = 1e-6;

using std::string;
using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Pointwise;

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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  EXPECT_THAT(error,
              HasSubstr("Error opening file"));
  mj_deleteVFS(vfs.get());
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
  mj_deleteVFS(vfs.get());
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

  // tiny RGB 2 x 3 PNG file
  static constexpr unsigned char tiny[] = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
    0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02,
    0x08, 0x02, 0x00, 0x00, 0x00, 0x12, 0x16, 0xf1, 0x4d, 0x00, 0x00, 0x00,
    0x1c, 0x49, 0x44, 0x41, 0x54, 0x08, 0xd7, 0x63, 0x78, 0xc1, 0xc0, 0xc0,
    0xc0, 0xf0, 0xbf, 0xb8, 0xb8, 0x98, 0x81, 0xe1, 0x3f, 0xc3, 0xff, 0xff,
    0xff, 0xc5, 0xc4, 0xc4, 0x00, 0x46, 0xd7, 0x07, 0x7f, 0xd2, 0x52, 0xa1,
    0x41, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60,
    0x82
  };

  size_t tiny_sz = sizeof(tiny);

  char error[1024];
  size_t error_sz = 1024;


  // load VFS on the heap
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), filename, tiny, tiny_sz);

  // loading the file should be successful
  mjModel* model = LoadModelFromString(xml, error, error_sz, vfs.get());
  EXPECT_THAT(model, NotNull());

  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

// ------------------------ test keyframes -------------------------------------

using KeyframeTest = MujocoTest;

constexpr char kKeyframePath[] = "user/testdata/keyframe.xml";

TEST_F(KeyframeTest, CheckValues) {
  const string xml_path = GetTestDataFilePath(kKeyframePath);
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
  const string xml_path = GetTestDataFilePath(kKeyframePath);
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

TEST_F(KeyframeTest, ResetDataKeyframeAcceptsNegativeKeyframe) {
  const string xml_path = GetTestDataFilePath(kKeyframePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  data->time = data->qpos[0] = data->qvel[0] = data->act[0] = data->ctrl[0] =
      data->mocap_pos[0] = data->mocap_quat[0] = 1337.0;

  mj_resetDataKeyframe(model, data, -1);

  EXPECT_EQ(data->time, 0.0);
  EXPECT_EQ(data->qpos[0], 0.0);
  EXPECT_EQ(data->qvel[0], 0.0);
  EXPECT_EQ(data->act[0], 0.0);
  EXPECT_EQ(data->ctrl[0], 0.0);
  EXPECT_EQ(data->mocap_pos[0], 0.0);
  EXPECT_EQ(data->mocap_quat[0], 1.0);

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
  EXPECT_THAT(error.data(),
              HasSubstr("unrecognized name 'wrong_name' of object"));
  EXPECT_THAT(error.data(), HasSubstr("line 8"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 8"));
}

TEST_F(RelativeFrameSensorParsingTest, BadObjName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
    </worldbody>
    <sensor>
      <framepos name="tom" objtype="site" objname="alessio" reftype="site" refname="a"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("unrecognized name 'alessio' of sensorized object"));
  EXPECT_THAT(error.data(), HasSubstr("name 'tom'"));
  EXPECT_THAT(error.data(), HasSubstr("line 7"));
}

TEST_F(RelativeFrameSensorParsingTest, BadObjRefName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
    </worldbody>
    <sensor>
      <framepos name="tom" objtype="site" objname="a" reftype="site" refname="alessio"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(
      error.data(),
      HasSubstr("unrecognized name 'alessio' of object"));
  EXPECT_THAT(error.data(), HasSubstr("name 'tom'"));
  EXPECT_THAT(error.data(), HasSubstr("line 7"));
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
  const string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  // Mass of capsule should equal mass of cylinder + mass of sphere.
  mjtNum sphere_cylinder_mass =
      model->body_mass[kSphereBodyId] + model->body_mass[kCylinderBodyId];
  mjtNum capsule_mass = model->body_mass[kCapsuleBodyId];
  EXPECT_DOUBLE_EQ(sphere_cylinder_mass, capsule_mass);
  mj_deleteModel(model);
}

TEST_F(MjCGeomTest, CapsuleInertiaZ) {
  const string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
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
  const string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
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

TEST_F(MjCGeomTest, ShellInertiaSphere) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="sphere"size="1.5" shellinertia="true"/>
      </body>
      <body>
        <!-- mass is difference of body 4 and 3 masses -->
        <geom type="sphere" size="1.5" mass="28.274333953857422" shellinertia="true"/>
      </body>
      <body>
        <geom type="sphere" size="1.5" density="1e8"/>
      </body>
      <body>
        <geom type="sphere" size="1.50000001" density="1e8"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  // radius
  mjtNum r = 1.5;
  mjtNum r2 = r * r;

  // body 1: shell inertia
  mjtNum mass1 = 4 * mjPI * r2 * 1000;  // surface area * surface density
  EXPECT_NEAR(m->body_mass[1], mass1, kInertiaTol);
  mjtNum I1 = 2 * mass1 * r2 / 3;
  EXPECT_NEAR(m->body_inertia[3], I1, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[4], I1, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[5], I1, kInertiaTol);

  // body 2: shell inertia, with specified mass
  mjtNum I2 = 2 * m->body_mass[2] * r2 / 3;
  EXPECT_NEAR(m->body_inertia[6], I2, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[7], I2, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[8], I2, kInertiaTol);

  mjtNum mass3 = m->body_mass[3];
  mjtNum mass4 = m->body_mass[4];
  EXPECT_FLOAT_EQ(mass4 - mass3, m->body_mass[2]);

  // compute approximate shell inertia by subtracting inertias of massive bodies
  // with small radius difference
  mjtNum* inertia3 = m->body_inertia + 9;
  mjtNum* inertia4 = m->body_inertia + 12;
  mjtNum shell_inertia[3];
  mju_sub3(shell_inertia, inertia4, inertia3);
  EXPECT_NEAR(shell_inertia[0], m->body_inertia[6], kInertiaTol);
  EXPECT_NEAR(shell_inertia[1], m->body_inertia[7], kInertiaTol);
  EXPECT_NEAR(shell_inertia[2], m->body_inertia[8], kInertiaTol);

  mj_deleteModel(m);
}

TEST_F(MjCGeomTest, ShellInertiaCapsule) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="capsule" size="0.1 0.25" shellinertia="true"/>
      </body>
      <!-- mass is difference of body 4 and 3 masses -->
      <body>
        <geom type="capsule" size="0.1 0.25" mass="4.3982325" shellinertia="true"/>
      </body>
      <body>
        <geom type="capsule" size="0.1 0.25" density="1e8"/>
      </body>
      <body>
        <geom type="capsule" size="0.1000001 0.25" density="1e8"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  // dimensions
  mjtNum r = 0.1;
  mjtNum r2 = r * r;
  mjtNum hh = 0.25;
  mjtNum h = 2 * hh;
  mjtNum h2 = h * h;

  // hemisphere
  mjtNum hs_com = r / 2;        // height of hemisphere center of mass
  mjtNum hs_pos = hh + hs_com;  // distance from origin to hemisphere com

  // surface area
  double Asphere = 4 * mjPI * r2;       // sphere
  double Acylinder = 2 * mjPI * r * h;  // cylinder
  double Atotal = Asphere + Acylinder;

  // body 1: shell inertia
  mjtNum mass1 = Atotal * 1000;  // surface area * surface density
  EXPECT_NEAR(m->body_mass[1], mass1, kInertiaTol);
  mjtNum mass1_sphere = mass1 * Asphere / Atotal;
  mjtNum mass1_cylinder = mass1 - mass1_sphere;
  double sphere1_inertia = 2 * mass1_sphere * r2 / 3;
  mjtNum I1x = mass1_cylinder * (6 * r2 + h2) / 12 + sphere1_inertia +
               mass1_sphere * (hs_pos * hs_pos - hs_com * hs_com);
  mjtNum I1z = mass1_cylinder * r2 + sphere1_inertia;
  EXPECT_NEAR(m->body_inertia[3], I1x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[4], I1x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[5], I1z, kInertiaTol);

  // body 2: shell inertia, with specified mass
  mjtNum mass2 = 4.3982325;
  EXPECT_NEAR(m->body_mass[2], mass2, kInertiaTol);
  EXPECT_FLOAT_EQ(m->body_mass[4] - m->body_mass[3], m->body_mass[2]);

  mjtNum mass2_sphere = mass2 * Asphere / Atotal;
  mjtNum mass2_cylinder = mass2 - mass2_sphere;
  double sphere2_inertia = 2 * mass2_sphere * r2 / 3;
  mjtNum I2x = mass2_cylinder * (6 * r2 + h2) / 12 + sphere2_inertia +
               mass2_sphere * (hs_pos * hs_pos - hs_com * hs_com);
  mjtNum I2z = mass2_cylinder * r2 + sphere2_inertia;
  EXPECT_NEAR(m->body_inertia[6], I2x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[7], I2x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[8], I2z, kInertiaTol);

  // compute approximate shell inertia by subtracting inertias of massive bodies
  // with small radius difference
  mjtNum* inertia3 = m->body_inertia + 9;
  mjtNum* inertia4 = m->body_inertia + 12;
  mjtNum shell_inertia[3];
  mju_sub3(shell_inertia, inertia4, inertia3);
  EXPECT_NEAR(shell_inertia[0], m->body_inertia[6], kInertiaTol);
  EXPECT_NEAR(shell_inertia[1], m->body_inertia[7], kInertiaTol);
  EXPECT_NEAR(shell_inertia[2], m->body_inertia[8], kInertiaTol);

  mj_deleteModel(m);
}

TEST_F(MjCGeomTest, ShellInertiaCylinder) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="cylinder" size="0.1 0.25" shellinertia="true"/>
      </body>
      <!-- mass is difference of body 4 and 3 masses -->
      <body>
        <geom type="cylinder" size="0.1 0.25" mass="3.7699139" shellinertia="true"/>
      </body>
      <body>
        <geom type="cylinder" size="0.1 0.25" density="1e8"/>
      </body>
      <body>
        <geom type="cylinder" size="0.1000001 0.2500001" density="1e8"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  // dimensions
  mjtNum r = 0.1;
  mjtNum hh = 0.25;
  mjtNum r2 = r * r;
  mjtNum h = 2 * hh;
  mjtNum h2 = h * h;

  // surface area
  double Adisk = mjPI * r2;             // disk
  double Acylinder = 2 * mjPI * r * h;  // cylinder
  double Atotal = 2 * Adisk + Acylinder;

  // body 1: shell inertia
  mjtNum mass1 = Atotal * 1000;  // surface area * surface density
  EXPECT_NEAR(m->body_mass[1], mass1, kInertiaTol);
  mjtNum mass1_disk = mass1 * Adisk / Atotal;
  mjtNum mass1_cylinder = mass1 - 2 * mass1_disk;
  mjtNum I1x = mass1_cylinder * (6 * r2 + h2) / 12 +
               2 * (mass1_disk * r2 / 4 + mass1_disk * hh * hh);
  mjtNum I1z = mass1_cylinder * r2 + mass1_disk * r2;
  EXPECT_NEAR(m->body_inertia[3], I1x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[4], I1x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[5], I1z, kInertiaTol);

  // body 2: shell inertia, with specified mass
  mjtNum mass2 = 3.7699139;
  EXPECT_NEAR(m->body_mass[2], mass2, kInertiaTol);
  EXPECT_FLOAT_EQ(m->body_mass[4] - m->body_mass[3], m->body_mass[2]);

  mjtNum mass2_disk = mass2 * Adisk / Atotal;
  mjtNum mass2_cylinder = mass2 - 2 * mass2_disk;
  mjtNum I2x = mass2_cylinder * (6 * r2 + h2) / 12 +
               2 * (mass2_disk * r2 / 4 + mass2_disk * hh * hh);
  mjtNum I2z = mass2_cylinder * r2 + mass2_disk * r2;
  EXPECT_NEAR(m->body_inertia[6], I2x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[7], I2x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[8], I2z, kInertiaTol);

  // compute approximate shell inertia by subtracting inertias of massive bodies
  // with small radius difference
  mjtNum* inertia3 = m->body_inertia + 9;
  mjtNum* inertia4 = m->body_inertia + 12;
  mjtNum shell_inertia[3];
  mju_sub3(shell_inertia, inertia4, inertia3);
  EXPECT_NEAR(shell_inertia[0], m->body_inertia[6], kInertiaTol);
  EXPECT_NEAR(shell_inertia[1], m->body_inertia[7], kInertiaTol);
  EXPECT_NEAR(shell_inertia[2], m->body_inertia[8], kInertiaTol);

  mj_deleteModel(m);
}

TEST_F(MjCGeomTest, ShellInertiaEllipsoid) {
  // test special case of ellipsoid with dimensions: a = b = c
  // TODO(taylorhowell): add test for ellipsoid with dimensions: a != b != c
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="ellipsoid" size="0.1 0.1 0.1" shellinertia="true"/>
      </body>
      <body>
        <!-- mass is difference of body 4 and 3 masses -->
        <geom type="ellipsoid" size="0.1 0.1 0.1" mass="0.12566371" shellinertia="true"/>
      </body>
      <body>
        <geom type="ellipsoid" size="0.1 0.1 0.1" density="1e8"/>
      </body>
      <body>
        <geom type="ellipsoid" size="0.10000001 0.10000001 0.10000001" density="1e8"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  // dimensions
  mjtNum r = 0.1;
  mjtNum r2 = r * r;

  // body 1: shell inertia
  mjtNum mass1 = 4 * mjPI * r2 * 1000;  // surface area * surface density
  EXPECT_NEAR(m->body_mass[1], mass1, kInertiaTol);
  mjtNum I1 = 2 * mass1 * r2 / 3;

  // note: increased tolerance, this is due to ellipsoid approximation
  EXPECT_NEAR(m->body_inertia[3], I1, 10 * kInertiaTol);
  EXPECT_NEAR(m->body_inertia[4], I1, 10 * kInertiaTol);
  EXPECT_NEAR(m->body_inertia[5], I1, 10 * kInertiaTol);

  // body 2: shell inertia, with specified mass
  mjtNum mass2 = 0.12566371;
  EXPECT_NEAR(m->body_mass[2], mass2, kInertiaTol);
  EXPECT_FLOAT_EQ(m->body_mass[4] - m->body_mass[3], m->body_mass[2]);

  mjtNum I2 = 2 * m->body_mass[2] * r2 / 3;
  EXPECT_NEAR(m->body_inertia[6], I2, 10 * kInertiaTol);
  EXPECT_NEAR(m->body_inertia[7], I2, 10 * kInertiaTol);
  EXPECT_NEAR(m->body_inertia[8], I2, 10 * kInertiaTol);

  // compute approximate shell inertia by subtracting inertias of massive bodies
  // with small radius difference
  mjtNum* inertia3 = m->body_inertia + 9;
  mjtNum* inertia4 = m->body_inertia + 12;
  mjtNum shell_inertia[3];
  mju_sub3(shell_inertia, inertia4, inertia3);
  EXPECT_NEAR(shell_inertia[0], m->body_inertia[6], 10 * kInertiaTol);
  EXPECT_NEAR(shell_inertia[1], m->body_inertia[7], 10 * kInertiaTol);
  EXPECT_NEAR(shell_inertia[2], m->body_inertia[8], 10 * kInertiaTol);

  mj_deleteModel(m);
}

TEST_F(MjCGeomTest, ShellInertiaBox) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="box" size="0.1 0.2 0.3" shellinertia="true"/>
      </body>
      <!-- mass is difference of body 4 and 3 masses -->
      <body>
        <geom type="box" size="0.1 0.2 0.3" mass="8.800005" shellinertia="true"/>
      </body>
      <body>
        <geom type="box" size="0.1 0.2 0.3" density="1e8"/>
      </body>
      <body>
        <geom type="box" size="0.1000001 0.2000001 0.3000001" density="1e8"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  // dimensions
  mjtNum dx = 0.1;
  mjtNum dy = 0.2;
  mjtNum dz = 0.3;

  // length
  mjtNum lx = 2 * dx;
  mjtNum ly = 2 * dy;
  mjtNum lz = 2 * dz;

  // surface area
  double A0 = lx * ly;
  double A1 = ly * lz;
  double A2 = lz * lx;
  double Atotal = 2 * (A0 + A1 + A2);

  // body 1: shell inertia
  mjtNum mass1 = Atotal * 1000;  // surface area * surface density
  EXPECT_NEAR(m->body_mass[1], mass1, kInertiaTol);

  mjtNum mass1_0 = mass1 * A0 / Atotal;
  mjtNum mass1_1 = mass1 * A1 / Atotal;
  mjtNum mass1_2 = mass1 * A2 / Atotal;
  mjtNum I1x = 2 * (mass1_0 * ly * ly / 12 + mass1_0 * dz * dz +
                    mass1_1 * (ly * ly + lz * lz) / 12 +
                    mass1_2 * lz * lz / 12 + mass1_2 * dy * dy);
  mjtNum I1y =
      2 * (mass1_0 * lx * lx / 12 + mass1_0 * dz * dz + mass1_1 * lz * lz / 12 +
           mass1_1 * dx * dx + mass1_2 * (lx * lx + lz * lz) / 12);
  mjtNum I1z =
      2 * (mass1_0 * (lx * lx + ly * ly) / 12 + mass1_1 * ly * ly / 12 +
           mass1_1 * dx * dx + mass1_2 * lx * lx / 12 + mass1_2 * dy * dy);

  EXPECT_NEAR(m->body_inertia[3], I1x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[4], I1y, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[5], I1z, kInertiaTol);

  // body 2: shell inertia, with specified mass
  mjtNum mass2 = 8.800005;
  EXPECT_NEAR(m->body_mass[2], mass2, 1e-6);
  EXPECT_FLOAT_EQ(m->body_mass[4] - m->body_mass[3], m->body_mass[2]);

  mjtNum mass2_0 = mass2 * A0 / Atotal;
  mjtNum mass2_1 = mass2 * A1 / Atotal;
  mjtNum mass2_2 = mass2 * A2 / Atotal;
  mjtNum I2x = 2 * (mass2_0 * ly * ly / 12 + mass2_0 * dz * dz +
                    mass2_1 * (ly * ly + lz * lz) / 12 +
                    mass2_2 * lz * lz / 12 + mass2_2 * dy * dy);
  mjtNum I2y =
      2 * (mass2_0 * lx * lx / 12 + mass2_0 * dz * dz + mass2_1 * lz * lz / 12 +
           mass2_1 * dx * dx + mass2_2 * (lx * lx + lz * lz) / 12);
  mjtNum I2z =
      2 * (mass2_0 * (lx * lx + ly * ly) / 12 + mass2_1 * ly * ly / 12 +
           mass2_1 * dx * dx + mass2_2 * lx * lx / 12 + mass2_2 * dy * dy);

  EXPECT_NEAR(m->body_inertia[6], I2x, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[7], I2y, kInertiaTol);
  EXPECT_NEAR(m->body_inertia[8], I2z, kInertiaTol);

  // compute approximate shell inertia by subtracting inertias of massive bodies
  // with small radius difference
  mjtNum* inertia3 = m->body_inertia + 9;
  mjtNum* inertia4 = m->body_inertia + 12;
  mjtNum shell_inertia[3];
  mju_sub3(shell_inertia, inertia4, inertia3);
  EXPECT_NEAR(shell_inertia[0], m->body_inertia[6], kInertiaTol);
  EXPECT_NEAR(shell_inertia[1], m->body_inertia[7], kInertiaTol);
  EXPECT_NEAR(shell_inertia[2], m->body_inertia[8], kInertiaTol);

  mj_deleteModel(m);
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
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
}

TEST_F(MjCGeomTest, BadMeshZeroMassDensityDoesntError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="bad_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="0 2 1"  inertia="shell"/>
    </asset>
    <worldbody>
      <body>
        <geom type="mesh" mesh="bad_mesh" mass="0"/>
      </body>
      <body>
        <geom type="mesh" mesh="bad_mesh" density="0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->body_mass[1], 0);
  EXPECT_EQ(model->body_mass[2], 0);
  mj_deleteModel(model);
}

// ------------- test joints --------------------------------------------------

using MjCJointTest = MujocoTest;

TEST_F(MjCJointTest, AlignFree) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/freejoint.xml");
  std::array<char, 1024> err;
  mjSpec* s = mj_parseXML(xml_path.c_str(), nullptr, err.data(), err.size());
  ASSERT_THAT(s, NotNull()) << err.data();
  s->compiler.alignfree = 1;  // auto-aligned free joint
  mjModel* m = mj_compile(s, nullptr);
  ASSERT_THAT(m, NotNull());

  // with alignfree the body has sameframe and simple and all dof are simple
  EXPECT_EQ(m->body_sameframe[1], 1);
  EXPECT_EQ(m->body_simple[1], 1);
  EXPECT_EQ(m->dof_simplenum[0], 6);

  // make unaligned model
  s->compiler.alignfree = 0;  // unaligned free joint
  mjModel* m_u = mj_compile(s, nullptr);
  ASSERT_THAT(m_u, NotNull());

  // no sameframe or simple
  EXPECT_EQ(m_u->body_sameframe[1], 0);
  EXPECT_EQ(m_u->body_simple[1], 0);

  // make datas for both models
  mjData* d = mj_makeData(m);
  mjData* d_u = mj_makeData(m_u);

  // call mj_forward
  mj_forward(m, d);
  mj_forward(m_u, d_u);

  // expect x-frames (sensors) to match to very high precision
  double eps = 1e-10;
  EXPECT_THAT(AsVector(d->sensordata, m->nsensordata),
      Pointwise(DoubleNear(eps), AsVector(d_u->sensordata, m->nsensordata)));

  // no frame sensors for lights, test separately
  EXPECT_THAT(AsVector(d->light_xpos, 3),
      Pointwise(DoubleNear(eps), AsVector(d_u->light_xpos, 3)));
  EXPECT_THAT(AsVector(d->light_xdir, 3),
      Pointwise(DoubleNear(eps), AsVector(d_u->light_xdir, 3)));

  // reduce timestep to 0.1ms and use RK4, simulate for 1 second
  m->opt.timestep = m_u->opt.timestep = 1e-4;
  m->opt.integrator = m_u->opt.integrator = mjINT_RK4;
  while (d->time < 1) {
    mj_step(m, d);
    mj_step(m_u, d_u);
  }

  // expect qpos to be significantly different, since the semantics are changed
  mj_markStack(d);
  int nq = m->nq;
  mjtNum* dqpos = mj_stackAllocNum(d, nq);
  mju_sub(dqpos, d->qpos, d_u->qpos, nq);
  EXPECT_GT(mju_norm(dqpos, nq), 1.0);
  mj_freeStack(d);


  // expect x-frames to match to reasonable precision
  eps = 1e-5;
  EXPECT_THAT(AsVector(d->sensordata, m->nsensordata),
      Pointwise(DoubleNear(eps), AsVector(d_u->sensordata, m->nsensordata)));
  EXPECT_THAT(AsVector(d->light_xpos, 3),
      Pointwise(DoubleNear(eps), AsVector(d_u->light_xpos, 3)));
  EXPECT_THAT(AsVector(d->light_xdir, 3),
      Pointwise(DoubleNear(eps), AsVector(d_u->light_xdir, 3)));

  mj_deleteData(d_u);
  mj_deleteData(d);
  mj_deleteModel(m_u);
  mj_deleteModel(m);
  mj_deleteSpec(s);
}


// ------------- test height fields --------------------------------------------

using MjCHFieldTest = MujocoTest;

TEST_F(MjCHFieldTest, PngMap) {
  const string xml_path =
      GetTestDataFilePath("user/testdata/hfield_png.xml");
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

// ------------- test textures -------------------------------------------------

using MjCTextureTest = MujocoTest;

TEST_F(MjCTextureTest, TexturesLoad) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="body" type="cube" builtin="flat" mark="cross" width="8"
       rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" markrgb="1 1 1"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  mj_deleteModel(m);
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
  EXPECT_THAT(error.data(), HasSubstr("line 6"));
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

TEST_F(CameraSpecTest, ParentTargetingNull) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom size="1"/>
      <camera mode="targetbody" target="world"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  mjData* data = mj_makeData(model);
  mj_forward(model, data);
  // orientation can't be decided so we get an arbitrary but valid xmat
  // (camera pointed towards the negative x-axis)
  EXPECT_THAT(AsVector(data->cam_xmat, 9),
              ElementsAre(0, 0, 1,
                          1, 0, 0,
                          0, 1, 0));
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(CameraSpecTest, ParentTargeting) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom size="1"/>
      <camera pos="1 1 0" mode="targetbody" target="world"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  mjData* data = mj_makeData(model);
  mj_forward(model, data);
  // expect negative z-axis to point from camera to world
  EXPECT_FLOAT_EQ(data->cam_xmat[2], mju_sqrt(0.5));
  EXPECT_FLOAT_EQ(data->cam_xmat[5], mju_sqrt(0.5));
  EXPECT_FLOAT_EQ(data->cam_xmat[8], 0);
  mj_deleteData(data);
  mj_deleteModel(model);
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

// ------------- test inheritrange attribute ----------------------------------

using InheritrangeTest = MujocoTest;

TEST_F(InheritrangeTest, ErrorIfTargetMissingRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="jnt"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" inheritrange="1"/>
    </actuator>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("target 'jnt' has no range defined"));
}

TEST_F(InheritrangeTest, WorksForDegrees) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="jnt" range="90 180"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" inheritrange="1"/>
    </actuator>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_DOUBLE_EQ(model->actuator_ctrlrange[0], mjPI/2);
  EXPECT_DOUBLE_EQ(model->actuator_ctrlrange[1], mjPI);

  mj_deleteModel(model);
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
  EXPECT_THAT(error.data(), HasSubstr("invalid actrange"));
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
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
  EXPECT_THAT(error.data(), HasSubstr("invalid actrange"));
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
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
  EXPECT_THAT(error.data(),
              HasSubstr("actrange specified but dyntype is 'none'"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 7"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 9"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 11"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 8"));
}

// ------------- test for auto parsing of *limited fields ----------------------

using LimitedTest = MujocoTest;

constexpr char kKeyAutoLimits[] = "user/testdata/auto_limits.xml";

// check joint limit values when automatically inferred based on range
TEST_F(LimitedTest, JointLimited) {
  const string path = GetTestDataFilePath(kKeyAutoLimits);
  std::array<char, 1024> err;
  mjModel* model = mj_loadXML(path.c_str(), nullptr, err.data(), err.size());
  ASSERT_THAT(model, NotNull()) << err.data();

  // see `user/testdata/auto_limits.xml` for expected values
  for (int i=0; i < model->njnt; i++) {
    EXPECT_EQ(model->jnt_limited[i], (mjtByte)model->jnt_user[i])
        << i << " " << (int)model->jnt_limited[i] << " "
        << (int)model->jnt_user[i];
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
  EXPECT_THAT(error.data(), HasSubstr("line 6"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 13"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 11"));
}

// ------------- tests for tendon ----------------------------------------------

using TendonTest = MujocoTest;

TEST_F(TendonTest, SiteBetweenPulleyNotAllowed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="1"/>
      <site name="2"/>
      <site name="3"/>
    </worldbody>
    <tendon>
      <spatial>
        <site site="1"/>
        <pulley divisor="1"/>
        <site site="3"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("needs a neighbor that is not a pulley"));
  EXPECT_THAT(error.data(), HasSubstr("line 9"));
}

TEST_F(TendonTest, ActuatorForceRangeNotAllowed) {
  std::string xml = R"(
  <mujoco>
    <worldbody>
      <site name="site0"/>
      <site name="site1"/>
    </worldbody>
    <tendon>
      <spatial name="spatial" actuatorfrclimited="true" actuatorfrcrange="{}">
        <site site="site0"/>
        <site site="site1"/>
      </spatial>
    </tendon>
    <actuator>
      <motor tendon="spatial"/>
    </actuator>
  </mujoco>
  )";

  std::array<char, 1024> error;
  std::string str_replace = "{}";
  size_t rng_ind = xml.find(str_replace);

  std::string xml0 = xml;
  std::string range0 = "-2 -1";
  xml0.replace(rng_ind, str_replace.length(), range0);
  mjModel* m0 = LoadModelFromString(xml0.c_str(), error.data(), error.size());
  EXPECT_THAT(m0, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid actuatorfrcrange in tendon"));

  std::string xml1 = xml;
  std::string range1 = "1 2";
  xml1.replace(rng_ind, str_replace.length(), range1);
  mjModel* m1 = LoadModelFromString(xml1.c_str(), error.data(), error.size());
  EXPECT_THAT(m1, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid actuatorfrcrange in tendon"));

  std::string xml2 = xml;
  std::string range2 = "1 0";
  xml2.replace(rng_ind, str_replace.length(), range2);
  mjModel* m2 = LoadModelFromString(xml2.c_str(), error.data(), error.size());
  EXPECT_THAT(m2, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid actuatorfrcrange in tendon"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 9"));
}

using UserObjectsTest = MujocoTest;

// ------------- test frame ----------------------------------------------------
TEST_F(UserObjectsTest, Frame) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <frame euler="0 0 30">
        <geom name="0" size=".1" euler="0 0 20"/>
      </frame>

      <frame axisangle="0 1 0 90">
        <frame axisangle="0 0 1 90">
          <geom name="1" size=".1"/>
        </frame>
      </frame>

      <frame pos="0 1 0" euler="0 20 0">
        <geom name="2" pos=".5 .6 .7" size=".1" euler="30 0 0"/>
      </frame>

      <body>
        <frame pos="0 1 0">
          <geom name="3" size=".1" pos="0 1 0"/>
          <body pos="1 0 0">
            <geom name="4" size=".1" pos="0 0 1"/>
          </body>
        </frame>
      </body>

      <body>
        <geom name="5" size=".1"/>
        <frame euler="90 0 0">
          <joint type="hinge" axis="0 0 1"/>
        </frame>
      </body>

      <body pos="0 1 0" euler="0 20 0">
        <geom name="6" pos=".5 .6 .7" size=".1" euler="30 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  constexpr mjtNum eps = 1e-14;
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, testing::NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 5);

  // geom quat transformed to euler = 0 0 50
  EXPECT_NEAR(m->geom_quat[0], mju_cos(25. * mjPI / 180.), 1e-3);
  EXPECT_NEAR(m->geom_quat[1], 0, 0);
  EXPECT_NEAR(m->geom_quat[2], 0, 0);
  EXPECT_NEAR(m->geom_quat[3], mju_sin(25. * mjPI / 180.), 1e-3);

  // geom transformed to frame 0 1 0, 0 0 1, 1 0 0
  EXPECT_NEAR(m->geom_quat[4], .5, eps);
  EXPECT_NEAR(m->geom_quat[5], .5, eps);
  EXPECT_NEAR(m->geom_quat[6], .5, eps);
  EXPECT_NEAR(m->geom_quat[7], .5, eps);

  // geom pos transformed from 0 1 0 to 0 2 0
  EXPECT_EQ(m->geom_pos[ 9], 0);
  EXPECT_EQ(m->geom_pos[10], 2);
  EXPECT_EQ(m->geom_pos[11], 0);

  // body pos transformed from 1 0 0 to 1 1 0
  EXPECT_EQ(m->body_pos[6], 1);
  EXPECT_EQ(m->body_pos[7], 1);
  EXPECT_EQ(m->body_pos[8], 0);

  // nested geom pos not transformed
  EXPECT_EQ(m->geom_pos[12], 0);
  EXPECT_EQ(m->geom_pos[13], 0);
  EXPECT_EQ(m->geom_pos[14], 1);

  // joint axis transformed to 0 -1 0
  EXPECT_NEAR(m->jnt_axis[0],  0, eps);
  EXPECT_NEAR(m->jnt_axis[1], -1, eps);
  EXPECT_NEAR(m->jnt_axis[2],  0, eps);

  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);

  // body and frame equivalence geom 2 vs 6
  EXPECT_NEAR(d->geom_xpos[6], d->geom_xpos[18], eps);
  EXPECT_NEAR(d->geom_xpos[7], d->geom_xpos[19], eps);
  EXPECT_NEAR(d->geom_xpos[8], d->geom_xpos[20], eps);
  EXPECT_NEAR(d->geom_xmat[18], d->geom_xmat[54], eps);
  EXPECT_NEAR(d->geom_xmat[19], d->geom_xmat[55], eps);
  EXPECT_NEAR(d->geom_xmat[20], d->geom_xmat[56], eps);
  EXPECT_NEAR(d->geom_xmat[21], d->geom_xmat[57], eps);

  mj_deleteModel(m);
  mj_deleteData(d);
}

TEST_F(UserObjectsTest, FrameTransformsLight) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <frame euler="0 45 0" pos="0 0 1">
        <light pos="-1 0 0" dir="1 0 -1"/>
      </frame>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_EQ(m->nlight, 1);

  constexpr mjtNum eps = 1e-14;
  EXPECT_NEAR(m->light_pos[0], -mju_sqrt(.5), eps);
  EXPECT_NEAR(m->light_pos[1], 0, eps);
  EXPECT_NEAR(m->light_pos[2], 1 + mju_sqrt(.5), eps);

  EXPECT_NEAR(m->light_dir[0], 0, eps);
  EXPECT_NEAR(m->light_dir[1], 0, eps);
  EXPECT_NEAR(m->light_dir[2], -1, eps);

  mj_deleteModel(m);
}

TEST_F(ContentTypeTest, ImageLightsReferenceTexture) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" type="cube" builtin="flat" mark="cross" width="8"
       rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" markrgb="1 1 1"/>
    </asset>

    <worldbody>
      <light type="image" texture="texture"/>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull());
  EXPECT_EQ(m->ntex, 1);
  EXPECT_EQ(m->nlight, 1);
  EXPECT_THAT(m->light_texid[0], 0);
  mj_deleteModel(m);
}


// ------------- test bvh ------------------------------------------------------
TEST_F(UserObjectsTest, RobustBVH) {
  static constexpr char xml1[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size=".1" pos="0 0 0"/>
        <geom size=".1" pos="0 1 0"/>
        <geom size=".1" pos="1 0 0"/>
        <geom size=".1" pos="1 1 0"/>
        <geom size=".1" pos="2 0 0"/>
        <geom size=".1" pos="2 1 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char xml2[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size=".1" pos="0 0 0"/>
        <geom size=".1" pos="0 1 0"/>
        <geom size=".1" pos="1.00000000000001 0 0"/>
        <geom size=".1" pos="1 1.00000000000001 0"/>
        <geom size=".1" pos="2 0 0"/>
        <geom size=".1" pos="2 1 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* m1 = LoadModelFromString(xml1, error.data(), error.size());
  EXPECT_THAT(m1, NotNull()) << error.data();

  mjModel* m2 = LoadModelFromString(xml2, error.data(), error.size());
  EXPECT_THAT(m2, NotNull()) << error.data();

  EXPECT_EQ(m1->nbvh, m2->nbvh);
  for (int i = 0; i < m1->nbvh; i++) {
    EXPECT_EQ(m1->bvh_nodeid[i], m2->bvh_nodeid[i]);
  }

  mj_deleteModel(m1);
  mj_deleteModel(m2);
}

// ------------- test equality compilation -------------------------------------

TEST_F(UserObjectsTest, BadConnect) {
  string base = R"(
  <mujoco>
    <worldbody>
      <site name="0" size="1"/>
      <body name="1" pos="0 0 1">
        <freejoint/>
        <geom size="1"/>
        <site name="1"/>
      </body>
    </worldbody>
    <equality>
      CONNECT
    </equality>
  </mujoco>
  )";
  int pos = base.find("CONNECT");
  int len = 7;

  // good model using body semantic
  string xml = base.replace(pos, len, "<connect body1='1' anchor='0 0 1'/>");
  char error[1024];
  mjModel* m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_THAT(AsVector(m->eq_data, 6), ElementsAre(0, 0, 1, 0, 0, 2));
  mj_deleteModel(m);

  // good model using site semantic
  xml = base.replace(pos, len, "<connect site1='0' site2='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_THAT(AsVector(m->eq_data, 6), ElementsAre(0, 0, 0, 0, 0, 0));
  mj_deleteModel(m);

  char error_missing[] = "either both body1 and anchor must be defined,"
      " or both site1 and site2 must be defined\nElement 'connect', line 12";

  // bad model (missing anchor)
  xml = base.replace(pos, len, "<connect body1='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_missing));

  char error_mixed[] = "body and site semantics cannot be mixed"
      "\nElement 'connect', line 12";

  // bad model (mixing body and site)
  xml = base.replace(pos, len, "<connect body1='1' site1='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_mixed));

  // load spec with no constraints
  xml = base.erase(pos, len);
  mjSpec* s = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(s, NotNull()) << error;

  // add a connect but don't set objtype
  mjsEquality* equality = mjs_addEquality(s, nullptr);
  equality->type = mjEQ_CONNECT;
  mjs_setString(equality->name1, "0");
  mjs_setString(equality->name2, "1");

  // expect compilation to fail
  m = mj_compile(s, nullptr);
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(mjs_getError(s),
              HasSubstr("connect constraint supports only sites and bodies"));

  // set objtype, expect compilation to succeed
  equality->objtype = mjOBJ_SITE;
  m = mj_compile(s, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(s);
  mj_deleteModel(m);
  mj_deleteSpec(s);
}

TEST_F(UserObjectsTest, BadWeld) {
  string base = R"(
  <mujoco>
    <worldbody>
      <site name="0" size="1"/>
      <body name="1" pos="0 0 1">
        <freejoint/>
        <geom size="1"/>
        <site name="1"/>
      </body>
    </worldbody>
    <equality>
      WELD
    </equality>
  </mujoco>
  )";
  int pos = base.find("WELD");
  int len = 4;

  // good model using body semantic
  string xml = base.replace(pos, len,
                            "<weld body1='1' anchor='0 0 1' torquescale='2'/>");
  char error[1024];
  mjModel* m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_THAT(AsVector(m->eq_data, 10),
              ElementsAre(0, 0, 1, 0, 0, 0, 1, 0, 0, 0));
  mj_deleteModel(m);

  // good model using site semantic
  xml = base.replace(pos, len, "<weld site1='0' site2='1' torquescale='2'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_THAT(AsVector(m->eq_data, 10),
              ElementsAre(0, 1, 0, 0, 0, 0, 0, 0, 0, 0));
  mj_deleteModel(m);

  char error_mixed[] =
      "body and site semantics cannot be mixed"
      "\nElement 'weld', line 12";

  // bad model (mixing body and site)
  xml = base.replace(pos, len, "<weld body1='1' site1='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_mixed));

  // bad model (mixing site and anchor)
  xml = base.replace(pos, len, "<weld anchor='0 0 1' site1='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_mixed));

  // bad model (mixing site and relpose)
  xml = base.replace(pos, len, "<weld relpose='0 0 0 1 0 0 0' site1='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_mixed));

  // bad model (body and site semantics are valid, but both are specified)
  xml = base.replace(pos, len, "<weld body1='1' site1='1' site2='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_mixed));

  char error_underspecified[] =
      "either body1 must be defined and optionally {body2, anchor, "
      "relpose}, or site1 and site2 must be defined\nElement 'weld', line 12";

  // bad model (underspecified body semantics)
  xml = base.replace(pos, len, "<weld anchor='0 0 1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_underspecified));

  // bad model (underspecified site semantics)
  xml = base.replace(pos, len, "<weld site2='1'/>");
  m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr(error_underspecified));

  // load spec with no constraints
  xml = base.erase(pos, len);
  mjSpec* s = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(s, NotNull()) << error;

  // add a weld but don't set objtype
  mjsEquality* equality = mjs_addEquality(s, nullptr);
  equality->type = mjEQ_WELD;
  mjs_setString(equality->name1, "0");
  mjs_setString(equality->name2, "1");

  // expect compilation to fail
  m = mj_compile(s, nullptr);
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(mjs_getError(s),
              HasSubstr("weld constraint supports only sites and bodies"));

  // set objtype, expect compilation to succeed
  equality->objtype = mjOBJ_SITE;
  m = mj_compile(s, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(s);
  mj_deleteModel(m);
  mj_deleteSpec(s);
}

TEST_F(UserObjectsTest, Inertial) {
  string xml = R"(
  <mujoco>
    <compiler angle="radian"/>
    <worldbody>
      <body>
        <inertial mass="1" pos="2 3 4" euler="3 4 5" diaginertia="4 5 6"/>
      </body>
      <body>
        <inertial mass="2" pos="1 2 3" fullinertia="4 3 2 0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_EQ(m->body_mass[1], 1);
  EXPECT_THAT(AsVector(m->body_ipos+3, 3), ElementsAre(2, 3, 4));
  EXPECT_THAT(AsVector(m->body_inertia+3, 3), ElementsAre(4, 5, 6));

  mjtNum quat[4];
  const mjtNum euler[3] = {3, 4, 5};
  mju_euler2Quat(quat, euler, "xyz");
  EXPECT_THAT(AsVector(m->body_iquat+4, 4),
              Pointwise(DoubleNear(1e-8), AsVector(quat, 4)));

  EXPECT_EQ(m->body_mass[2], 2);
  EXPECT_THAT(AsVector(m->body_ipos+6, 3), ElementsAre(1, 2, 3));
  EXPECT_THAT(AsVector(m->body_inertia+6, 3), ElementsAre(4, 3, 2));
  mj_deleteModel(m);

  string bad_xml1 = R"(
  <mujoco>
    <worldbody>
      <body>
        <inertial mass="1" pos="0 0 0" diaginertia="4 5 6" fullinertia="4 3 2 0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  m = LoadModelFromString(bad_xml1.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr("fullinertia and diagonal inertia cannot both"));

  string bad_xml2 = R"(
  <mujoco>
    <worldbody>
      <body>
        <inertial mass="1" pos="0 0 0" euler="4 5 6" fullinertia="4 3 2 0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  m = LoadModelFromString(bad_xml2.c_str(), error, sizeof(error));
  ASSERT_THAT(m, IsNull());
  EXPECT_THAT(error, HasSubstr("fullinertia and inertial orientation cannot"));
}

}  // namespace
}  // namespace mujoco
