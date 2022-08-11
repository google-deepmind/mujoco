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
#include <climits>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <mujoco/mjxmacro.h>
#include "src/engine/engine_util_errmem.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

using EngineIoTest = MujocoTest;

// Return an mjModel with just the ints set.
mjModel PartialModel(const mjModel* m) {
  mjModel partial_model = {0};
  #define X(var) partial_model.var = m->var;
  MJMODEL_INTS;
  #undef X
  partial_model.nbuffer = 0;
  return partial_model;
}

TEST_F(EngineIoTest, MakeDataFromPartialModel) {
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
  mjData* data_from_model = mj_makeData(model);
  ASSERT_THAT(data_from_model, NotNull());

  mjModel partial_model = PartialModel(model);
  mj_deleteModel(model);

  mjData* data_from_partial = mj_makeData(&partial_model);
  ASSERT_THAT(data_from_partial, NotNull());

  EXPECT_EQ(data_from_partial->nbuffer, data_from_model->nbuffer);
  // If there are no mocap bodies and qpos0 is all zero, mjData should be the
  // same whether it was made from the full model or the partial model.
  {
    MJDATA_POINTERS_PREAMBLE((&partial_model))
    #define X(type, name, nr, nc)                                              \
        EXPECT_EQ(std::memcmp(data_from_partial->name, data_from_model->name,  \
                              sizeof(type)*(partial_model.nr)*(nc)),           \
                  0) << "mjData::" #name " differs";
    MJDATA_POINTERS
    #undef X
  }

  mj_deleteData(data_from_model);
  mj_deleteData(data_from_partial);
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

TEST_F(EngineIoTest, CopyDataWithPartialModel) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size="1"/>
      </body>
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

  mjModel partial_model = PartialModel(model);
  mj_deleteModel(model);
  mjData* copy = mj_makeData(&partial_model);
  ASSERT_THAT(copy, NotNull());

  data->qpos[0] = 1;
  mj_copyData(copy, &partial_model, data);

  EXPECT_EQ(copy->nbuffer, data->nbuffer);
  EXPECT_EQ(copy->qpos[0], 1);
  {
    MJDATA_POINTERS_PREAMBLE((&partial_model))
    #define X(type, name, nr, nc)                                     \
        EXPECT_EQ(std::memcmp(copy->name, data->name,                 \
                              sizeof(type)*(partial_model.nr)*(nc)),  \
                  0) << "mjData::" #name " differs";
    MJDATA_POINTERS
    #undef X
  }

  mj_deleteData(data);
  mj_deleteData(copy);
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

}  // namespace
}  // namespace mujoco
