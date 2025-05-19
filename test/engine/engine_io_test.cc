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

// Tests for engine/engine_io.c.

#include "src/engine/engine_io.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>  // IWYU pragma: keep
#include <absl/strings/str_format.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_errmem.h"
#include "src/thread/thread_pool.h"
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  std::filesystem::path temp_file = (
    std::filesystem::temp_directory_path() / "model.mjb");

  mj_saveModel(model, temp_file.string().c_str(), NULL, 0);

  std::uintmax_t file_size = std::filesystem::file_size(temp_file);
  int model_size = mj_sizeModel(model);

  std::filesystem::remove(temp_file);
  mj_deleteModel(model);

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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  model->qpos0[0] = 1;
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());
  EXPECT_EQ(data->qpos[0], 1);

  mj_deleteData(data);
  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());
  EXPECT_EQ(data->mocap_pos[0], 42);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, MakeDataReturnsNullOnFailure) {
  constexpr char xml[] = "<mujoco/>";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  // fail mj_makeData intentionally with a bad size
  model->nbody = -1;
  static bool warning;
  warning = false;
  mju_user_warning = [](const char* error) {
    warning = true;
  };
  mjData* data = mj_makeData(model);
  EXPECT_THAT(data, IsNull());
  EXPECT_TRUE(warning) << "Expecting warning to be triggered.";

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, ResetVariableSizes) {
  constexpr char xml[] = "<mujoco/>";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  ASSERT_THAT(model, NotNull()) << "Failed to create mjData";

  // don't call mj_forward, vars should be reset
  EXPECT_EQ(data->ne, 0);
  EXPECT_EQ(data->nf, 0);
  EXPECT_EQ(data->nefc, 0);
  EXPECT_EQ(data->nJ, 0);
  EXPECT_EQ(data->nA, 0);
  EXPECT_EQ(data->ncon, 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, MakeDataResetsAllArenaPointerSizes) {
  constexpr char xml[] = "<mujoco />";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  // without calling mj_forward, all array sizes should be zero.
  EXPECT_EQ(data->parena, 0) << "expecting empty arena";

#define X(type, name, nr, nc) \
  EXPECT_EQ(nr*nc, 0) << "expecting (" #nr " x " #nc ") to be zero";

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

  mj_deleteData(data);
  mj_deleteModel(model);
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
  mjModel* model1 = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model1, NotNull()) << error;

  mjModel* model2 = mj_copyModel(nullptr, model1);
  ASSERT_THAT(model2, NotNull()) << error;

  model1->mesh_vert[0] = 0.1;
  model1->geom_rgba[0] = 0.2;
  mj_copyModel(model2, model1);

  EXPECT_FLOAT_EQ(model2->mesh_vert[0], 0.1);
  EXPECT_FLOAT_EQ(model2->geom_rgba[0], 0.2);

  model1->mesh_vert[0] = 0.3;
  model1->geom_rgba[0] = 0.4;
  mjv_copyModel(model2, model1);

  EXPECT_FLOAT_EQ(model2->mesh_vert[0], 0.1);  // unchanged
  EXPECT_FLOAT_EQ(model2->geom_rgba[0], 0.4);

  mj_deleteModel(model2);
  mj_deleteModel(model1);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data1 = mj_makeData(model);
  mj_forward(model, data1);
  EXPECT_THAT(data1->efc_J, NotNull());

  mjData* data2 = mj_copyData(nullptr, model, data1);
  EXPECT_THAT(data2->efc_J, NotNull());

  mj_deleteData(data2);
  data2 = mjv_copyData(nullptr, model, data1);
  EXPECT_THAT(data2->efc_J, IsNull());

  mj_deleteData(data2);
  mj_deleteData(data1);
  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  model->jnt_bodyid[0] = 2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("jnt_bodyid"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  model->body_jntnum[1] = 3;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("body_jntadr"));
  model->body_jntnum[1] = 2;
  // Could be more strict and test for -1, but at the moment the code is a bit
  // lenient.
  model->body_jntadr[1] = -2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("body_jntadr"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  // jntadr + jntnum is within safe range, but jntnum is negative.
  model->body_jntadr[1] += 5;
  model->body_jntnum[1] -= 5;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("body_jntnum"));
  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());
  model->geom_condim[0] = 7;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("geom_condim"));
  model->geom_condim[0] = -1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("geom_condim"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  model->hfield_adr[0] = -2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("hfield_adr"));
  model->hfield_adr[0] = 0;
  model->hfield_ncol[0] = 4;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("hfield_adr"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  model->tex_adr[0] = -2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("tex_adr"));
  model->tex_adr[0] = 0;
  model->tex_height[0] = 4;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("tex_adr"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  // Invalid geomid=4
  model->pair_signature[0] = (1 << 16) | 5;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("pair_body1"));

  model->pair_signature[0] = (5 << 16) | 1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("pair_body2"));

  mj_deleteModel(model);
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
    mjModel* model =
        LoadModelFromString(xml.c_str(), error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

    EXPECT_THAT(mj_validateReferences(model), IsNull());
    mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());
  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  model->sensor_objtype[0] = -1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("sensor_objtype"));
  model->sensor_objtype[0] = mjOBJ_SITE;

  model->sensor_objid[0] = model->nsite;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("sensor_objid"));
  model->sensor_objid[0] = 0;

  model->sensor_reftype[0] = -1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("sensor_reftype"));
  model->sensor_reftype[0] = mjOBJ_BODY;

  model->sensor_refid[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("sensor_refid"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  EXPECT_THAT(mj_validateReferences(model), IsNull());
  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  // Invalid bodyid=3
  model->exclude_signature[0] = (1 << 16) | 4;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("exclude_body1"));
  model->exclude_signature[0] = (4 << 16) | 2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("exclude_body2"));

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  // connect constraint
  model->eq_obj1id[0] = -1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj1id"));
  model->eq_obj1id[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj1id"));
  model->eq_obj1id[0] = 1;

  model->eq_obj2id[0] = -2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj2id"));
  model->eq_obj2id[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj2id"));
  model->eq_obj2id[0] = 0;

  // weld constraint
  model->eq_obj1id[1] = -1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj1id"));
  model->eq_obj1id[1] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj1id"));
  model->eq_obj1id[1] = 1;

  model->eq_obj2id[1] = -2;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj2id"));
  model->eq_obj2id[1] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("eq_obj2id"));
  model->eq_obj2id[1] = model->nbody - 1;

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  EXPECT_THAT(mj_validateReferences(model), IsNull());

  model->tuple_objtype[0] = -1;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("tuple_objtype"));
  model->tuple_objtype[0] = mjOBJ_BODY;

  model->tuple_objid[0] = model->nbody;
  EXPECT_THAT(mj_validateReferences(model), HasSubstr("tuple_objid"));
  model->tuple_objid[0] = 1;
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, CanMarkAndFreeStack) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  auto pstack_before = data->pstack;
  mj_markStack(data);
  EXPECT_GT(data->pstack, pstack_before);
  mj_freeStack(data);
  EXPECT_EQ(data->pstack, pstack_before);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, LargeMemory) {
  constexpr char xml[] = R"(
  <mujoco>
    <size memory="2400M"/>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  // allocate 2.3G of mjtNums
  mj_markStack(data);
  size_t num = 2300000000 / sizeof(mjtNum);
  mjtNum* testNum = mj_stackAllocNum(data, num);
  testNum[num-1] = 1;
  mj_freeStack(data);

  // allocate 2.3G of bytes
  mj_markStack(data);
  num = 2300000000;
  char* testByte = (char*) mj_stackAllocByte(data, num, alignof(char));
  testByte[num-1] = 1;
  mj_freeStack(data);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, VeryLargeMemory) {
  constexpr char xml[] = R"(
  <mujoco>
    <size memory="8G"/>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  if (!model) {
    // in some test environments, 8GB is too large
    EXPECT_THAT(error.data(), HasSubstr("Could not allocate memory"));
  } else {
    ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
    mjData* data = mj_makeData(model);
    ASSERT_THAT(data, NotNull());

    // allocate 7G of mjtNums
    mj_markStack(data);
    size_t num = 7516192768ull / sizeof(mjtNum);
    mjtNum* testNum = mj_stackAllocNum(data, num);
    testNum[num-1] = 1;
    mj_freeStack(data);

    // allocate 7G of bytes
    mj_markStack(data);
    num = 7516192768ull;
    char* testByte = (char*) mj_stackAllocByte(data, num, alignof(char));
    testByte[num-1] = 1;
    mj_freeStack(data);

    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

struct TestFunctionArgs_ {
  mjData* d;
  int input;
  int stack_output;
  int arena_output;
  size_t output_thread_worker;
};
typedef TestFunctionArgs_ TestFunctionArgs;

void* TestFunction(void* args) {
  TestFunctionArgs* test_args = static_cast<TestFunctionArgs*>(args);
  test_args->output_thread_worker =
      mju_threadPoolCurrentWorkerId((mjThreadPool*)test_args->d->threadpool);
  mj_markStack(test_args->d);
  int* test_ints = mj_stackAllocInt(test_args->d, 10);
  test_ints[0] = test_args->input;
  test_args->stack_output = test_ints[0];

  int* test_arena_ints =
      (int*)mj_arenaAllocByte(test_args->d, sizeof(int) * 10, alignof(int));
  test_arena_ints[0] = test_args->input;
  test_args->arena_output = test_arena_ints[0];


  mj_freeStack(test_args->d);
  return nullptr;
}

TEST_F(EngineIoTest, TestStackShardingForThreads) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());
  mjThreadPool* thread_pool = mju_threadPoolCreate(10);
  mju_bindThreadPool(data, thread_pool);

  constexpr int kTasks = 1000;
  TestFunctionArgs test_function_args[kTasks];
  mjTask tasks[kTasks];
  for (int i = 0; i < kTasks; ++i) {
    test_function_args[i].d = data;
    test_function_args[i].input = i;
    mju_defaultTask(&tasks[i]);
    tasks[i].func = TestFunction;
    tasks[i].args = &test_function_args[i];
    mju_threadPoolEnqueue(thread_pool, &tasks[i]);
  }

  mj_markStack(data);
  int* test_ints = mj_stackAllocInt(data, 10);
  test_ints[0] = 1;
  mj_freeStack(data);

  for (int i = 0; i < kTasks; ++i) {
    mju_taskJoin(&tasks[i]);
  }

  for (int i = 0; i < kTasks; ++i) {
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].stack_output);
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].arena_output);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  mju_threadPoolDestroy(thread_pool);
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
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  // MarkFreeStack correctly calls mj_freeStack, should not error.
  MarkFreeStack(data, /* free= */ true);

  // MarkFreeStack calls mj_markStack without mj_freeStack, the next call to
  // mj_freeStack should detect the stack frame leakage.
  mj_markStack(data);
  MarkFreeStack(data, /* free= */ false);
  EXPECT_THAT(
      MjuErrorMessageFrom(mj_freeStack)(data),
      ContainsRegex(
          "mj_markStack in MarkFreeStack at .*engine_io_test\\.cc.* has no "
          "corresponding mj_freeStack"));

  // Dangling stack frames should be detected in mj_deleteData.
  mj_resetData(model, data);
  mj_markStack(data);
  EXPECT_THAT(
      MjuErrorMessageFrom(mj_deleteData)(data),
      ContainsRegex(
          "mj_markStack in .+EngineIoTest_CanDetectStackFrameLeakage.+ has no "
          "corresponding mj_freeStack"));

  mj_resetData(model, data);
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineIoTest, RedZoneAlignmentTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();

  mjData* data = mj_makeData(model);
  ASSERT_THAT(data, NotNull());

  mj_markStack(data);
  mj_stackAllocByte(data, 1, 1);
  mj_stackAllocByte(data, 1, 1);
  mj_freeStack(data);

  mj_deleteData(data);
  mj_deleteModel(model);
}
#endif

}  // namespace
}  // namespace mujoco
