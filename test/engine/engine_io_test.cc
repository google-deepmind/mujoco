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

// Tests for engine/{engine_io.c and engine_memory.c}.

#include "src/engine/engine_io.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>  // NOLINT
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest-spi.h>  // IWYU pragma: keep
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_memory.h"
#include "src/engine/engine_thread.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ContainsRegex;  // NOLINT(misc-unused-using-decls) asan only
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

using EngineIoTest = MujocoTest;

TEST_F(EngineIoTest, VerifySizeModel) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  std::filesystem::path temp_file =
      (std::filesystem::temp_directory_path() / "model.mjb");

  mj_saveModel(model.get(), temp_file.string().c_str(), NULL, 0);

  std::uintmax_t file_size = std::filesystem::file_size(temp_file);
  int model_size = mj_sizeModel(model.get());

  std::filesystem::remove(temp_file);

  EXPECT_EQ(file_size, model_size);
}

TEST_F(EngineIoTest, MakeDataLoadsQpos0) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();
  model->qpos0[0] = 1;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  EXPECT_EQ(data->qpos[0], 1);
}

TEST_F(EngineIoTest, MakeDataLoadsMocapBodies) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body mocap="true" pos="42 0 42">
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  EXPECT_EQ(data->mocap_pos[0], 42);
}

TEST_F(EngineIoTest, MakeDataReturnsNullOnFailure) {
  constexpr char xml[] = "<mujoco/>";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  // fail mj_makeData intentionally with a bad size
  model->nbody = -1;
  MockWarningHandler warning_handler;
  warning_handler.ExpectWarnings();
  MjDataPtr data = MakeData(model);
  EXPECT_THAT(data, IsNull());
}

TEST_F(EngineIoTest, ResetVariableSizes) {
  constexpr char xml[] = "<mujoco/>";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  MjDataPtr data = MakeData(model);
  ASSERT_THAT(model.get(), NotNull()) << "Failed to create mjData";

  // don't call mj_forward, vars should be reset
  EXPECT_EQ(data->ne, 0);
  EXPECT_EQ(data->nf, 0);
  EXPECT_EQ(data->nefc, 0);
  EXPECT_EQ(data->nJ, 0);
  EXPECT_EQ(data->nA, 0);
  EXPECT_EQ(data->ncon, 0);
}

TEST_F(EngineIoTest, MakeDataResetsAllArenaPointerSizes) {
  constexpr char xml[] = "<mujoco />";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  // without calling mj_forward, all array sizes should be zero.
  EXPECT_EQ(data->parena, 0) << "expecting empty arena";

#define X(type, name, nr, nc) \
  EXPECT_EQ(nr* nc, 0) << "expecting (" #nr " x " #nc ") to be zero";

#undef MJ_D
#undef MJ_M
#define MJ_D(name) data->name
#define MJ_M(name) model->name
  MJDATA_ARENA_POINTERS;
#undef MJ_D
#undef MJ_M
#define MJ_D(n) n
#define MJ_M(n) n
#undef X
}

TEST_F(EngineIoTest, MjvCopyModel) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="tet"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model1 = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model1.get(), NotNull()) << error;

  mjModel* model2 = mj_copyModel(nullptr, model1.get());
  ASSERT_THAT(model2, NotNull()) << error;

  model1->mesh_vert[0] = 0.1;
  model1->geom_rgba[0] = 0.2;
  mj_copyModel(model2, model1.get());

  EXPECT_FLOAT_EQ(model2->mesh_vert[0], 0.1);
  EXPECT_FLOAT_EQ(model2->geom_rgba[0], 0.2);

  model1->mesh_vert[0] = 0.3;
  model1->geom_rgba[0] = 0.4;
  mjv_copyModel(model2, model1.get());

  EXPECT_FLOAT_EQ(model2->mesh_vert[0], 0.1);  // unchanged
  EXPECT_FLOAT_EQ(model2->geom_rgba[0], 0.4);

  mj_deleteModel(model2);
}

TEST_F(EngineIoTest, MjvCopyData) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 0.8">
        <freejoint/>
        <geom type="sphere" size="0.1"/>
      </body>
      <geom type="plane" size="1 1 1"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  MjDataPtr data1 = MakeData(model);
  mj_forward(model.get(), data1.get());
  EXPECT_THAT(data1->efc_J, NotNull());

  mjData* data2 = mj_copyData(nullptr, model.get(), data1.get());
  EXPECT_THAT(data2->efc_J, NotNull());

  mj_deleteData(data2);
  data2 = mjv_copyData(nullptr, model.get(), data1.get());
  EXPECT_THAT(data2->efc_J, IsNull());

  mj_deleteData(data2);
}

using ValidateReferencesTest = MujocoTest;

TEST_F(ValidateReferencesTest, BodyReferences) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  model->jnt_bodyid[0] = 2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("jnt_bodyid"));
}

TEST_F(ValidateReferencesTest, AddressRange) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <joint/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  model->body_jntnum[1] = 3;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("body_jntadr"));
  model->body_jntnum[1] = 2;
  // Could be more strict and test for -1, but at the moment the code is a bit
  // lenient.
  model->body_jntadr[1] = -2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("body_jntadr"));
}

TEST_F(ValidateReferencesTest, AddressRangeNegativeNum) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <joint/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  // jntadr + jntnum is within safe range, but jntnum is negative.
  model->body_jntadr[1] += 5;
  model->body_jntnum[1] -= 5;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("body_jntnum"));
}

TEST_F(ValidateReferencesTest, GeomCondim) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1" condim="6"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());
  model->geom_condim[0] = 7;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("geom_condim"));
  model->geom_condim[0] = -1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("geom_condim"));
}

TEST_F(ValidateReferencesTest, HField) {
  static const char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="h" nrow="2" ncol="3" size="1 1 1 1" />
    </asset>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  model->hfield_adr[0] = -2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("hfield_adr"));
  model->hfield_adr[0] = 0;
  model->hfield_ncol[0] = 4;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("hfield_adr"));
}

TEST_F(ValidateReferencesTest, Texture) {
  static const char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="t" type="2d" width="2" height="3" builtin="flat" />
    </asset>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  model->tex_adr[0] = -2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("tex_adr"));
  model->tex_adr[0] = 0;
  model->tex_height[0] = 4;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("tex_adr"));
}

TEST_F(ValidateReferencesTest, GeomPairs) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom name="geom0" size="1"/>
      </body>
      <body>
        <geom name="geom1" size="1"/>
        <geom name="geom2" size="1"/>
        <geom name="geom3" size="1"/>
      </body>
    </worldbody>
    <contact>
      <pair geom1="geom0" geom2="geom3"/>
    </contact>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  // Invalid geomid=4
  model->pair_signature[0] = (1 << 16) | 5;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("pair_body1"));

  model->pair_signature[0] = (5 << 16) | 1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("pair_body2"));
}

TEST_F(ValidateReferencesTest, SensorsAddress) {
  // The test will likely only catch sensor size errors for the last sensor
  // in the model, so iterate over possible last sensors, instead of adding
  // them all into the same model.
  static const char xml_template[] = R"(
  <mujoco>
    <worldbody>
      <body name="body1">
        <joint name="slider" type="slide" axis="0 0 1"
            limited="true" range="-.2 .5"/>
        <geom name="geom1" size="1"/>
        <site name="site1"/>
      </body>
    </worldbody>
    <sensor>
      %s
    </sensor>
  </mujoco>
  )";
  std::vector<std::string> sensor_strings{
      "<framepos objtype='site' objname='site1'/>",
      "<rangefinder site='site1'/>",
      "<gyro site='site1'/>",
      "<touch site='site1'/>",
      "<force site='site1'/>",
      "<torque site='site1'/>",
      "<jointlimitfrc joint='slider'/>",
      "<accelerometer site='site1'/>",
      "<subtreeangmom body='body1'/>",
  };

  for (const std::string& sensor_string : sensor_strings) {
    std::string xml = absl::StrFormat(xml_template, sensor_string);
    std::array<char, 1024> error;
    MjModelPtr model =
        LoadModelFromString(xml.c_str(), error.data(), error.size());
    ASSERT_THAT(model.get(), NotNull())
        << "Failed to load model: " << error.data();

    EXPECT_THAT(mj_validateReferences(model.get()), IsNull());
  }
}

TEST_F(ValidateReferencesTest, SensorsAddressUser) {
  static const char xml[] = R"(
  <mujoco>
    <sensor>
      <user dim="3" user="1 2 3 4 5" />
      <user dim="2" />
      <user dim="1" />
    </sensor>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());
}

TEST_F(ValidateReferencesTest, SensorsObj) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body1">
        <joint/>
        <geom size="1"/>
        <site name="site1"/>
      </body>
    </worldbody>
    <sensor>
      <framepos objtype="site" objname="site1" reftype="body" refname="body1"/>
    </sensor>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  model->sensor_objtype[0] = -1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("sensor_objtype"));
  model->sensor_objtype[0] = mjOBJ_SITE;

  model->sensor_objid[0] = model->nsite;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("sensor_objid"));
  model->sensor_objid[0] = 0;

  model->sensor_reftype[0] = -1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("sensor_reftype"));
  model->sensor_reftype[0] = mjOBJ_BODY;

  model->sensor_refid[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("sensor_refid"));
}

TEST_F(ValidateReferencesTest, MoreBodiesThanGeoms) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom name="geom0" size="1"/>
      </body>
      <body>
        <geom name="geom1" size="1"/>
      </body>
    </worldbody>
    <contact>
      <pair geom1="geom1" geom2="geom0"/>
    </contact>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();
  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());
}

TEST_F(ValidateReferencesTest, BodyExcludes) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body1" />
      <body name="body2" />
    </worldbody>
    <contact>
      <exclude body1="body1" body2="body2"/>
    </contact>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  // Invalid bodyid=3
  model->exclude_signature[0] = (1 << 16) | 4;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("exclude_body1"));
  model->exclude_signature[0] = (4 << 16) | 2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("exclude_body2"));
}

TEST_F(ValidateReferencesTest, EqualityConstraints) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body1">
        <joint name="joint1"/>
        <geom name="geom1" size="1"/>
      </body>
      <body name="body2">
        <joint/>
        <joint name="joint2"/>
        <geom size="1"/>
        <geom size="1"/>
        <geom name="geom2" size="1"/>
      </body>
    </worldbody>
    <tendon>
      <fixed name="tendon1"><joint joint="joint1" coef="1"/></fixed>
      <fixed><joint joint="joint1" coef="1"/></fixed>
      <fixed><joint joint="joint1" coef="1"/></fixed>
      <fixed><joint joint="joint1" coef="1"/></fixed>
      <fixed name="tendon2"><joint joint="joint1" coef="1"/></fixed>
    </tendon>
    <equality>
      <connect anchor="0 0 0" body1="body1" />
      <weld body1="body1" body2="body2" />
      <joint joint1="joint1"/>
      <joint joint1="joint1" joint2="joint2"/>
      <tendon tendon1="tendon1"/>
      <tendon tendon1="tendon1" tendon2="tendon2"/>
    </equality>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  // connect constraint
  model->eq_obj1id[0] = -1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj1id"));
  model->eq_obj1id[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj1id"));
  model->eq_obj1id[0] = 1;

  model->eq_obj2id[0] = -2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj2id"));
  model->eq_obj2id[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj2id"));
  model->eq_obj2id[0] = 0;

  // weld constraint
  model->eq_obj1id[1] = -1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj1id"));
  model->eq_obj1id[1] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj1id"));
  model->eq_obj1id[1] = 1;

  model->eq_obj2id[1] = -2;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj2id"));
  model->eq_obj2id[1] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("eq_obj2id"));
  model->eq_obj2id[1] = model->nbody - 1;
}

TEST_F(ValidateReferencesTest, Tuples) {
  static const char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="body1">
        <joint name="joint1"/>
        <geom size="1"/>
      </body>
      <body name="body2">
        <joint/>
        <joint name="joint2"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <custom>
      <tuple name="tuple">
        <element objtype="body" objname="body1"/>
        <element objtype="joint" objname="joint2"/>
      </tuple>
    </custom>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model.get()), IsNull());

  model->tuple_objtype[0] = -1;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("tuple_objtype"));
  model->tuple_objtype[0] = mjOBJ_BODY;

  model->tuple_objid[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model.get()), HasSubstr("tuple_objid"));
  model->tuple_objid[0] = 1;
}

TEST_F(EngineIoTest, CanMarkAndFreeStack) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  auto pstack_before = data->pstack;
  mj_markStack(data.get());
  EXPECT_GT(data->pstack, pstack_before);
  mj_freeStack(data.get());
  EXPECT_EQ(data->pstack, pstack_before);
}

TEST_F(EngineIoTest, LargeMemory) {
  constexpr char xml[] = R"(
  <mujoco>
    <size memory="2400M"/>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  // allocate 2.3G of mjtNums
  mj_markStack(data.get());
  size_t num = 2300000000 / sizeof(mjtNum);
  mjtNum* testNum = mj_stackAllocNum(data.get(), num);
  testNum[num - 1] = 1;
  mj_freeStack(data.get());

  // allocate 2.3G of bytes
  mj_markStack(data.get());
  num = 2300000000;
  char* testByte = (char*)mj_stackAllocByte(data.get(), num, alignof(char));
  testByte[num - 1] = 1;
  mj_freeStack(data.get());
}

TEST_F(EngineIoTest, VeryLargeMemory) {
  constexpr char xml[] = R"(
  <mujoco>
    <size memory="8G"/>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  if (!model.get()) {
    // in some test environments, 8GB is too large
    EXPECT_THAT(error.data(), HasSubstr("Could not allocate memory"));
  } else {
    ASSERT_THAT(model.get(), NotNull())
        << "Failed to load model: " << error.data();
    MjDataPtr data = MakeData(model);
    ASSERT_THAT(data, NotNull());

    // allocate 7G of mjtNums
    mj_markStack(data.get());
    size_t num = 7516192768ull / sizeof(mjtNum);
    mjtNum* testNum = mj_stackAllocNum(data.get(), num);
    testNum[num - 1] = 1;
    mj_freeStack(data.get());

    // allocate 7G of bytes
    mj_markStack(data.get());
    num = 7516192768ull;
    char* testByte = (char*)mj_stackAllocByte(data.get(), num, alignof(char));
    testByte[num - 1] = 1;
    mj_freeStack(data.get());
  }
}

struct TestFunctionArgs {
  int stack_output[1000];
  int arena_output[1000];
};

void TestFunction(const mjModel* m, mjData* d, void* args, int i, int j) {
  TestFunctionArgs* test_args = static_cast<TestFunctionArgs*>(args);
  mj_markStack(d);
  int* test_ints = mj_stackAllocInt(d, 10);
  test_ints[0] = j;
  test_args->stack_output[j] = test_ints[0];
  mj_freeStack(d);
}

TEST_F(EngineIoTest, TestStackShardingForThreads) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  mju_threadpool(data.get(), 10);

  constexpr int kTasks = 1000;
  TestFunctionArgs test_function_args;
  mju_dispatch(model.get(), data.get(), TestFunction, &test_function_args,
               kTasks);

  for (int i = 0; i < kTasks; ++i) {
    EXPECT_EQ(i, test_function_args.stack_output[i]);
  }
}

#ifdef ADDRESS_SANITIZER
void MarkFreeStack(mjData* d, bool free) {
  mj_markStack(d);
  if (free) {
    mj_freeStack(d);
  }
}

TEST_F(EngineIoTest, CanDetectStackFrameLeakage) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  // MarkFreeStack correctly calls mj_freeStack, should not error.
  MarkFreeStack(data.get(), /* free= */ true);

  // MarkFreeStack calls mj_markStack without mj_freeStack, the next call to
  // mj_freeStack should detect the stack frame leakage.
  mj_markStack(data.get());
  MarkFreeStack(data.get(), /* free= */ false);
  EXPECT_THAT(
      MjuErrorMessageFrom(mj_freeStack)(data.get()),
      ContainsRegex(
          "mj_markStack in MarkFreeStack at .*engine_io_test\\.cc.* has no "
          "corresponding mj_freeStack"));

  // Dangling stack frames should be detected in mj_deleteData.
  mj_resetData(model.get(), data.get());
  mj_markStack(data.get());
  EXPECT_THAT(
      MjuErrorMessageFrom(mj_deleteData)(data.get()),
      ContainsRegex(
          "mj_markStack in .+EngineIoTest_CanDetectStackFrameLeakage.+ has no "
          "corresponding mj_freeStack"));

  mj_resetData(model.get(), data.get());
}

TEST_F(EngineIoTest, RedZoneAlignmentTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  mj_markStack(data.get());
  mj_stackAllocByte(data.get(), 1, 1);
  mj_stackAllocByte(data.get(), 1, 1);
  mj_freeStack(data.get());
}
#endif

// Regression test: crafted binary model with overflow-inducing sizes must be
// safely rejected (not cause heap overflow). This exercises the overflow checks
// in safeAddToBufferSize on all compilers including MSVC.
TEST_F(EngineIoTest, LoadModelBufferRejectsOverflowingSizes) {
  // construct a minimal valid-looking .mjb header
  int header[5];
  header[0] = 20;              // ID
  header[1] = sizeof(mjtNum);  // floating point size
  // We need the correct nsize and nptr from the current build.
  // Rather than hardcoding, we create and save a trivial model, then mutate
  // the sizes to trigger overflow.

  constexpr char xml[] = "<mujoco />";
  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull())
      << "Failed to load model: " << error.data();

  // save model to a buffer
  int bufsize = mj_sizeModel(model.get());
  ASSERT_GT(bufsize, 0);
  std::vector<char> buffer(bufsize);
  mj_saveModel(model.get(), nullptr, buffer.data(), bufsize);

  // locate the size fields in the buffer (after 5-int header)
  const int header_bytes = 5 * sizeof(int);
  ASSERT_GT(bufsize, header_bytes + 77 * (int)sizeof(mjtSize));

  // mutate a size field to an extremely large value that would overflow
  // when multiplied by sizeof(type). ntexdata is a good candidate since it
  // is a byte count field and gets multiplied by sizeof(mjtByte)==1, but other
  // fields multiply by sizeof(int) or sizeof(mjtNum), making overflow easier.
  // Use memcpy to avoid undefined behavior from unaligned access.
  const int size_offset = header_bytes + 49 * sizeof(mjtSize);

  // set to overflow-inducing value
  mjtSize overflow_val = static_cast<mjtSize>(SIZE_MAX / 2);
  std::memcpy(buffer.data() + size_offset, &overflow_val, sizeof(mjtSize));

  // also need to update the nbuffer field (last size) to avoid the early
  // nbuffer mismatch check — but the overflow should be caught earlier
  // in safeAddToBufferSize/mj_makeModel before we reach that check.

  // Intercept warnings to prevent them from failing the test.
  MockWarningHandler warning_handler;
  warning_handler.ExpectWarnings();

  // attempt to load — should return NULL, not crash
  mjModel* bad_model = mj_loadModelBuffer(buffer.data(), bufsize);
  EXPECT_THAT(bad_model, IsNull())
      << "Expected mj_loadModelBuffer to reject overflow-inducing sizes";

  // clean up if somehow it succeeded
  if (bad_model) {
    mj_deleteModel(bad_model);
  }
}

}  // namespace
}  // namespace mujoco
