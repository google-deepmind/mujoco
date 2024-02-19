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

// Tests for xml/xml_native_reader.cc.

#include <array>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "src/engine/engine_util_errmem.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::string;
using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::IsNan;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::FloatEq;

using XMLReaderTest = MujocoTest;

TEST_F(XMLReaderTest, UniqueElementTest) {
  std::array<char, 1024> error;
  static constexpr char xml[] = R"(
  <mujoco>
   <option>
      <flag sensor="disable"/>
      <flag sensor="disable"/>
    </option>
  </mujoco>
  )";

  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("unique element 'flag' found 2 times"));
}

TEST_F(XMLReaderTest, MemorySize) {
  std::array<char, 1024> error;
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="512"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << error.data();
    EXPECT_EQ(model->narena, 512);
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1K "/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << error.data();
    EXPECT_EQ(model->narena, 1024);
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="  10K"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << error.data();
    EXPECT_EQ(model->narena, 10240);
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory=" 4M  "/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << error.data();
    EXPECT_EQ(model->narena, 4*1024*1024);
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1G"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << error.data();
    EXPECT_EQ(model->narena, 1024*1024*1024);
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1073741824"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, NotNull()) << error.data();
    EXPECT_EQ(model->narena, 1024*1024*1024);
    mj_deleteModel(model);
  }
}

TEST_F(XMLReaderTest, InvalidMemorySize) {
  std::array<char, 1024> error;
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="-3"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, IsNull());
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1 M"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, IsNull());
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="2X"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, IsNull());
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="K"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml, error.data(), error.size());
    ASSERT_THAT(model, IsNull());
  }
}

TEST_F(XMLReaderTest, InvalidNUserBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_body="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_body"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_jnt="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_jnt"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserGeom) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_geom="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_geom"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserSite) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_site="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_site"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserCamera) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_cam="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_cam"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_tendon="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_tendon"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_actuator="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_actuator"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, InvalidNUserSensor) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_sensor="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_sensor"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, CanParseInf) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" pos="5e-1 -INF iNf"/>
        <geom size="1" pos="inF -inf Inf"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  const double inf = std::numeric_limits<double>::infinity();
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->geom_pos[0], 0.5);
  EXPECT_EQ(model->geom_pos[1], -inf);
  EXPECT_THAT(model->geom_pos[2], inf);
  EXPECT_EQ(model->geom_pos[3], inf);
  EXPECT_EQ(model->geom_pos[4], -inf);
  EXPECT_EQ(model->geom_pos[5], inf);
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, CanParseNanAndRaisesWarning) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" pos="nan NaN NAN"/>
        <geom size="1" pos="1.0 0.0 nAn"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_THAT(warning, HasSubstr("XML contains a 'NaN'"));
  EXPECT_THAT(model->geom_pos[0], IsNan());
  EXPECT_THAT(model->geom_pos[1], IsNan());
  EXPECT_THAT(model->geom_pos[2], IsNan());
  EXPECT_EQ(model->geom_pos[3], 1);
  EXPECT_EQ(model->geom_pos[4], 0);
  EXPECT_THAT(model->geom_pos[5], IsNan());
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, InvalidArrayElement) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1.0 0.0 0.0 [[1]]"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("problem reading attribute 'axisangle'"));
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
}

TEST_F(XMLReaderTest, InvalidArrayLength) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1 0 0 0 1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("has too much data"));
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
}

TEST_F(XMLReaderTest, InvalidQuaternion) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" quat="0 0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("zero quaternion is not allowed"));
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
}

TEST_F(XMLReaderTest, InvalidNumber) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1 0.1.2.3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("problem reading attribute"));
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
}

TEST_F(XMLReaderTest, AllowsSpaces) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1 0   0 0 "/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, InvalidDoubleOrientation) {
  std::string prefix = "<mujoco><worldbody><";
  std::string suffix = "/></worldbody></mujoco>";
  std::vector<std::string> orientations = {
    R"( quat="0 1 0 0" )",
    R"( euler="1.7 2.9 0.1" )",
    R"( zaxis="1.7 2.9 0.1" )",
    R"( axisangle="1.7 2.9 0.1 0" )",
    R"( xyaxes="1.7 2.9 0.1 0.4 1.4 0.6" )",
  };
  std::vector<std::string> fields = {
    "geom", "body", "camera", "site"
  };
  for (auto const& field : fields) {
    for (auto const& orient1 : orientations) {
      for (auto const& orient2 : orientations) {
        if (orient1 == orient2) continue;
        std::string xml = prefix + field + orient1 + orient2 + suffix;
        std::array<char, 1024> error;
        mjModel* model =
            LoadModelFromString(xml.c_str(), error.data(), error.size());
        ASSERT_THAT(model, IsNull());
        EXPECT_THAT(
            error.data(),
            HasSubstr("multiple orientation specifiers are not allowed"));
      }
    }
  }
}

TEST_F(XMLReaderTest, RepeatedDefaultName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="sphere">
        <geom type="sphere" size="1"/>
      </default>
      <default class="sphere">
        <geom type="capsule" size="1 1"/>
      </default>
    </default>
    <worldbody>
      <body>
        <geom class="sphere"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("repeated default class name"));
}

// ------------------------ test including -------------------------------------

// credit: https://www.mjt.me.uk/posts/smallest-png/
static constexpr unsigned char kTinyPng[] =
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

TEST_F(XMLReaderTest, IncludeTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="1 1 1"/>
      <include file="model1.xml"/>
      <include file="model2.xml"/>
    </worldbody>
  </mujoco>)";

  static constexpr char xml1[] = R"(
  <mujoco>
    <geom name="box" type="box" size="1 1 1"/>
  </mujoco>)";

  static constexpr char xml2[]= R"(
  <mujoco>
    <geom name="ball" type="sphere" size="2"/>
    <include file="model3.xml"/>
  </mujoco>)";

  static constexpr char xml3[]= R"(
  <mujoco>
    <geom name="another_box" type="box" size="2 2 2"/>
  </mujoco>)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  mj_makeEmptyFileVFS(vfs.get(), "model1.xml", sizeof(xml1));
  std::memcpy(vfs->filedata[vfs->nfile - 1], xml1, sizeof(xml1));

  mj_makeEmptyFileVFS(vfs.get(), "model2.xml", sizeof(xml2));
  std::memcpy(vfs->filedata[vfs->nfile - 1], xml2, sizeof(xml2));

  mj_makeEmptyFileVFS(vfs.get(), "model3.xml", sizeof(xml3));
  std::memcpy(vfs->filedata[vfs->nfile - 1], xml3, sizeof(xml3));

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(),
                                       error.size(), vfs.get());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "ball"), 2);
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "another_box"), 3);
  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

TEST_F(XMLReaderTest, IncludeChildTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="1 1 1"/>
      <include file="model1.xml">
        <geom name="box" type="box" size="1 1 1"/>
      </include>
    </worldbody>
  </mujoco>)";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Include element cannot have children"));
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, IncludeSameFileTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <include file="model1.xml"/>
    <include file="model1.xml"/>
  </mujoco>)";

  static constexpr char xml1[] = R"(
  <mujoco>
    <geom name="box" type="box" size="1 1 1"/>
  </mujoco>)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());

  mj_makeEmptyFileVFS(vfs.get(), "model1.xml", sizeof(xml1));
  std::memcpy(vfs->filedata[vfs->nfile - 1], xml1, sizeof(xml1));

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size(),
                                       vfs.get());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("File 'model1.xml' already included"));
  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

TEST_F(XMLReaderTest, IncludePathTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="1 1 1"/>
      <include file="submodels/model1.xml"/>
      <include file="submodels/model2.xml"/>
    </worldbody>
  </mujoco>)";

  static constexpr char xml1[] = R"(
  <mujoco>
    <geom name="box" type="box" size="1 1 1"/>
  </mujoco>)";

  static constexpr char xml2[]= R"(
  <mujoco>
    <geom name="ball" type="sphere" size="2"/>
    <include file="subsubmodels/model3.xml"/>
  </mujoco>)";

  static constexpr char xml3[]= R"(
  <mujoco>
    <geom name="another_box" type="box" size="2 2 2"/>
  </mujoco>)";

  MockFilesystem fs("IncludePathTest");
  fs.AddFile("model.xml", (const unsigned char*) xml, sizeof(xml));
  std::string modelpath = fs.FullPath("model.xml");

  fs.ChangeDirectory("submodels/");
  fs.AddFile("model1.xml", (const unsigned char*) xml1, sizeof(xml1));
  fs.AddFile("model2.xml", (const unsigned char*) xml2, sizeof(xml2));
  fs.AddFile("subsubmodels/model3.xml", (const unsigned char*) xml3, sizeof(xml3));
  fs.ChangeDirectory("/");

  mjModel* model = mj_loadXML(modelpath.c_str(), nullptr,
                              nullptr, 0);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "ball"), 2);
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "another_box"), 3);
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, FallbackIncludePathTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="1 1 1"/>
      <include file="model1.xml"/>
      <include file="submodels/model2.xml"/>
    </worldbody>
  </mujoco>)";

  static constexpr char xml1[] = R"(
  <mujoco>
    <geom name="box" type="box" size="1 1 1"/>
  </mujoco>)";

  static constexpr char xml2[]= R"(
  <mujoco>
    <geom name="ball" type="sphere" size="2"/>
    <include file="subsubmodels/model3.xml"/>
  </mujoco>)";

  static constexpr char xml3[]= R"(
  <mujoco>
    <geom name="another_box" type="box" size="2 2 2"/>
  </mujoco>)";

  MockFilesystem fs("FallbackIncludePathTest");
  fs.AddFile("model.xml", (const unsigned char*) xml, sizeof(xml));
  std::string modelpath = fs.FullPath("model.xml");

  fs.AddFile("model1.xml", (const unsigned char*) xml1, sizeof(xml1));
  fs.AddFile("submodels/model2.xml", (const unsigned char*) xml2, sizeof(xml2));
  fs.AddFile("subsubmodels/model3.xml", (const unsigned char*) xml3,
   sizeof(xml3));

  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(modelpath.c_str(), nullptr,
                              error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "ball"), 2);
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "another_box"), 3);
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, IncludeAssetsTest) {
  static constexpr char xml[] = R"(
  <mujoco>
   <include file="assets/assets.xml"/>
    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";
  static constexpr char assets[] = R"(
  <mujoco>
    <asset>
      <texture file="tiny.png" type="2d"/>
      <material name="material" texture="tiny"/>
      <include file="subassets/assets.xml"/>
    </asset>
  </mujoco>
  )";
  static constexpr char subassets[] = R"(
  <mujoco>
    <texture file="subtiny.png" type="2d"/>
    <material name="submaterial" texture="subtiny"/>
  </mujoco>
  )";

  MockFilesystem fs("IncludeAssetsTest");
  fs.AddFile("assets/tiny.png", kTinyPng, sizeof(kTinyPng));
  fs.AddFile("assets/subassets/subtiny.png", kTinyPng, sizeof(kTinyPng));
  fs.AddFile("assets/assets.xml", (const unsigned char*) assets,
   sizeof(assets));
  fs.AddFile("assets/subassets/assets.xml", (const unsigned char*) subassets,
   sizeof(subassets));
  fs.AddFile("model.xml", (const unsigned char*) xml, sizeof(xml));
  std::string modelpath = fs.FullPath("model.xml");

  // loading the file should be successful
  mjModel* model = mj_loadXML(modelpath.c_str(), nullptr, nullptr, 0);

  EXPECT_THAT(model, NotNull());

  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, FallbackIncludeAssetsTest) {
  static constexpr char xml[] = R"(
  <mujoco>
   <include file="assets/assets.xml"/>
    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";
  static constexpr char assets[] = R"(
  <mujoco>
    <asset>
      <texture file="tiny.png" type="2d"/>
      <material name="material" texture="tiny"/>
      <include file="subassets/assets.xml"/>
    </asset>
  </mujoco>
  )";
  static constexpr char subassets[] = R"(
  <mujoco>
    <texture file="subtiny.png" type="2d"/>
    <material name="submaterial" texture="subtiny"/>
  </mujoco>
  )";

  MockFilesystem fs("FallbackIncludeAssetsTest");
  fs.AddFile("assets/tiny.png", kTinyPng, sizeof(kTinyPng));

  // need to fallback for backwards compatibility
  fs.AddFile("subtiny.png", kTinyPng, sizeof(kTinyPng));

  fs.AddFile("assets/assets.xml", (const unsigned char*) assets,
   sizeof(assets));
  fs.AddFile("assets/subassets/assets.xml", (const unsigned char*) subassets,
   sizeof(subassets));
  fs.AddFile("model.xml", (const unsigned char*) xml, sizeof(xml));
  std::string modelpath = fs.FullPath("model.xml");

  // loading the file should be successful
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(modelpath.c_str(), nullptr,
                              error.data(), error.size());
  EXPECT_THAT(model, NotNull());

  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, IncludeAbsoluteTest) {
  static constexpr char xml[] = R"(
  <mujoco>
   <include file="assets/assets.xml"/>
    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";
  static constexpr char assets[] = R"(
  <mujoco>
    <asset>
      <texture file="tiny.png" type="2d"/>
      <material name="material" texture="tiny"/>
      <include file="subassets/assets.xml"/>
    </asset>
  </mujoco>
  )";
  static constexpr char subassets[] = R"(
  <mujoco>
    <texture file="MjMock.IncludeAbsoluteTest:assets/subtiny.png" type="2d"/>
    <material name="submaterial" texture="subtiny"/>
  </mujoco>
  )";

  MockFilesystem fs("IncludeAbsoluteTest");
  fs.AddFile("assets/tiny.png", kTinyPng, sizeof(kTinyPng));
  fs.AddFile("assets/subtiny.png", kTinyPng, sizeof(kTinyPng));
  fs.AddFile("assets/assets.xml", (const unsigned char*) assets,
   sizeof(assets));
  fs.AddFile("assets/subassets/assets.xml", (const unsigned char*) subassets,
   sizeof(subassets));
  fs.AddFile("model.xml", (const unsigned char*) xml, sizeof(xml));
  std::string modelpath = fs.FullPath("model.xml");

  std::array<char, 1024> error;
  // loading the file should be successful
  mjModel* model = mj_loadXML(modelpath.c_str(), nullptr,
                              error.data(), error.size());
  EXPECT_THAT(model, NotNull());

  mj_deleteModel(model);
}
// ------------------------ test frame parsing ---------------------------------

TEST_F(XMLReaderTest, ParseFrame) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <frame euler="0 0 30">
        <geom size=".1" euler="0 0 20"/>
      </frame>

      <body>
        <frame pos="0 1 0">
          <geom size=".1" pos="0 1 0"/>
          <body pos="1 0 0">
            <geom size=".1" pos="0 0 1"/>
          </body>
        </frame>
      </body>

      <frame euler="0 0 30">
        <frame euler="0 0 20">
          <geom size=".1"/>
        </frame>
      </frame>
    </worldbody>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  mj_deleteModel(m);
}

// ----------------------- test camera parsing ---------------------------------

TEST_F(XMLReaderTest, CameraInvalidFovyAndSensorsize) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <camera fovy="1" sensorsize="1 1" resolution="100 100"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("either 'fovy' or 'sensorsize'"));
  EXPECT_THAT(error.data(), HasSubstr("line 6"));
}

TEST_F(XMLReaderTest, CameraPricipalRequiresSensorsize) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <camera principal="1 1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("attribute missing: 'sensorsize'"));
  EXPECT_THAT(error.data(), HasSubstr("line 6"));
}

TEST_F(XMLReaderTest, CameraSensorsizeRequiresResolution) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <camera sensorsize="1 1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("attribute missing: 'resolution'"));
  EXPECT_THAT(error.data(), HasSubstr("line 6"));
}

// ----------------------- test inertia parsing --------------------------------

TEST_F(XMLReaderTest, InvalidInertialOrientation) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <inertial pos="0 0 0" mass="1" quat="1 0 0 0" fullinertia="1 1 1 0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("multiple orientation specifiers are not allowed"));
}

TEST_F(XMLReaderTest, ReadShellParameter) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="0 2 1  2 0 3" />
    </asset>
    <worldbody>
      <geom type="mesh" mesh="example_mesh" shellinertia="true"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}


TEST_F(XMLReaderTest, ReadsSkinGroups) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <composite prefix="B0" type="box" count="4 4 4" spacing=".2">
          <geom size=".1" group="2"/>
          <skin group="4"/>
        </composite>
      </body>
      <body>
        <composite prefix="B1" type="box" count="4 4 4" spacing=".2">
          <geom size=".1" group="4"/>
          <skin group="2"/>
        </composite>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  int geomid1 = mj_name2id(model, mjOBJ_GEOM, "B0G0_0_0");
  int geomid2 = mj_name2id(model, mjOBJ_GEOM, "B1G0_0_0");
  EXPECT_THAT(model->geom_group[geomid1], 2);
  EXPECT_THAT(model->skin_group[0], 4);
  EXPECT_THAT(model->geom_group[geomid2], 4);
  EXPECT_THAT(model->skin_group[1], 2);
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, InvalidSkinGroup) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <composite prefix="B0" type="box" count="6 6 6" spacing=".2">
          <geom size=".1"/>
          <skin group="6"/>
        </composite>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(
      error.data(),
      HasSubstr("skin group must be between 0 and 5\nElement 'skin', line 7"));
  mj_deleteModel(model);
}

// ------------- test height-field parsing -------------------------------------

using HfieldParsingTest = MujocoTest;

TEST_F(HfieldParsingTest, NoData) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="4" ncol="3" size="0.5 0.5 1 0.1"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->hfield_nrow[0], 4);
  EXPECT_EQ(model->hfield_ncol[0], 3);
  EXPECT_EQ(model->hfield_size[0], 0.5);
  EXPECT_EQ(model->hfield_size[1], 0.5);
  EXPECT_EQ(model->hfield_size[2], 1);
  EXPECT_EQ(model->hfield_size[3], 0.1);
  mj_deleteModel(model);
}

TEST_F(HfieldParsingTest, HasDataBadSize) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="3" ncol="2" size="0.5 0.5 1 0.1"
              elevation="1 2
                         3 4
                         5 6 7"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("data length must match nrow*ncol"));
  EXPECT_THAT(error.data(), HasSubstr("line 4"));
}

TEST_F(HfieldParsingTest, HasData) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <hfield name="hf" nrow="3" ncol="2" size="0.5 0.5 1 0.1"
              elevation="1 2
                         3 4
                         5 6"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->hfield_nrow[0], 3);
  EXPECT_EQ(model->hfield_ncol[0], 2);
  EXPECT_EQ(model->hfield_size[0], 0.5);
  EXPECT_EQ(model->hfield_size[1], 0.5);
  EXPECT_EQ(model->hfield_size[2], 1);
  EXPECT_EQ(model->hfield_size[3], 0.1);

  // offset (minimum) and scaling (maximum) from normalizing operation
  float offset = 1.0;
  float scale = 6.0 - offset;

  // compare data, note: reverse row order
  EXPECT_THAT(model->hfield_data[0], FloatEq((5-offset)/scale));
  EXPECT_THAT(model->hfield_data[1], FloatEq((6-offset)/scale));
  EXPECT_THAT(model->hfield_data[2], FloatEq((3-offset)/scale));
  EXPECT_THAT(model->hfield_data[3], FloatEq((4-offset)/scale));
  EXPECT_THAT(model->hfield_data[4], FloatEq((1-offset)/scale));
  EXPECT_THAT(model->hfield_data[5], FloatEq((2-offset)/scale));
  mj_deleteModel(model);
}


// ------------- test relative frame sensor parsing ----------------------------

using RelativeFrameSensorParsingTest = MujocoTest;

TEST_F(RelativeFrameSensorParsingTest, RefNameButNoType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objname="sensorized" objtype="body" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("but reftype is missing"));
  EXPECT_THAT(error.data(), HasSubstr("line 8"));
}

TEST_F(RelativeFrameSensorParsingTest, RefTypeButNoName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objname="sensorized" objtype="body" reftype="site"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("attribute missing: 'refname'"));
  EXPECT_THAT(error.data(), HasSubstr("line 8"));
}

// ------------- test actlimited parsing ---------------------------------------

using ActuatorTest = MujocoTest;

TEST_F(ActuatorTest, InvalidActlimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="hinge" actlimited="invalid" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("unrecognized attribute"));
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
}

TEST_F(ActuatorTest, IncompleteActlimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" actlimited="true" dyntype="filter" actrange="-1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("attribute 'actrange' does not have enough data"));
}

TEST_F(ActuatorTest, ReadsByte) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="filter" actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(*(model->actuator_actlimited), (mjtByte)(1 & 0xFF));
  mj_deleteModel(model);
}

// ---------------- test actuator parsing --------------------------------------

using ActuatorParseTest = MujocoTest;

TEST_F(ActuatorParseTest, ReadsDamper) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" ctrlrange="0 1"/>
      <general joint="jnt" ctrllimited="true" ctrlrange="0 1" gaintype="affine" biastype="none"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_THAT(model->actuator_gaintype[0], Eq(mjGAIN_AFFINE));
  EXPECT_THAT(model->actuator_gaintype[1], Eq(mjGAIN_AFFINE));
  EXPECT_THAT(model->actuator_biastype[0], Eq(mjBIAS_NONE));
  EXPECT_THAT(model->actuator_biastype[1], Eq(mjBIAS_NONE));
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, DamperRequiresPositiveDamping) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" kv="-1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("damping coefficient cannot be negative"));
}

TEST_F(ActuatorParseTest, DamperRequiresControlRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10" limited="true"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" kv="1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid control range"));
  EXPECT_THAT(error.data(), HasSubstr("line = 10"));
}

TEST_F(ActuatorParseTest, DamperPositiveControlRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" kv="1" ctrlrange="-1 0"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("control range cannot be negative"));
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
}

TEST_F(ActuatorParseTest, ReadsPositionIntvelKv) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" kv="2"/>
      <intvelocity joint="jnt" actrange="-1 1" kv="3"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_THAT(model->actuator_biasprm[0*mjNBIAS + 2], Eq(-2.0));
  EXPECT_THAT(model->actuator_biasprm[1*mjNBIAS + 2], Eq(-3.0));
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, RequirePositiveKv) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" kv="-2"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("kv cannot be negative"));
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
}

TEST_F(ActuatorParseTest, PositionIntvelocityVelocityDefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="position">
        <position kp="3" kv="4" inheritrange="2"/>
      </default>
      <default class="intvelocity">
        <intvelocity kp="5" kv="6" inheritrange="0.5"/>
      </default>
      <default class="velocity">
        <velocity kv="7"/>
      </default>
    </default>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="0 2"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" class="position"/>
      <intvelocity joint="jnt" class="intvelocity"/>
      <velocity joint="jnt" class="velocity"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->actuator_gainprm[0*mjNGAIN + 0], 3.0);
  EXPECT_EQ(model->actuator_biasprm[0*mjNBIAS + 1], -3.0);
  EXPECT_EQ(model->actuator_biasprm[0*mjNBIAS + 2], -4.0);
  EXPECT_EQ(model->actuator_gainprm[1*mjNGAIN + 0], 5.0);
  EXPECT_EQ(model->actuator_biasprm[1*mjNBIAS + 1], -5.0);
  EXPECT_EQ(model->actuator_biasprm[1*mjNBIAS + 2], -6.0);
  EXPECT_EQ(model->actuator_gainprm[2*mjNGAIN + 0], 7.0);
  EXPECT_EQ(model->actuator_biasprm[2*mjNBIAS + 1], 0.0);
  EXPECT_EQ(model->actuator_biasprm[2*mjNBIAS + 2], -7.0);
  for (int i = 0; i < model->nu; i++) {
    for (int j = 3; j < mjNBIAS; j++) {
      EXPECT_EQ(model->actuator_biasprm[i*mjNBIAS + j], 0.0);
    }
    for (int j = 3; j < mjNGAIN; j++) {
      EXPECT_EQ(model->actuator_gainprm[i*mjNGAIN + j], 0.0);
    }
  }
  EXPECT_EQ(model->actuator_ctrlrange[0*2 + 0], -1.0);
  EXPECT_EQ(model->actuator_ctrlrange[0*2 + 1], 3.0);
  EXPECT_EQ(model->actuator_actrange[1*2 + 0], 0.5);
  EXPECT_EQ(model->actuator_actrange[1*2 + 1], 1.5);
  mj_deleteModel(model);
}


TEST_F(ActuatorParseTest, IntvelocityCheckEquivalence) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge" kp="2.5" actrange="-1.57 1.57"/>
      <general joint="hinge" actlimited="true" actrange="-1.57 1.57"
      dyntype="integrator" biastype="affine" gainprm="2.5" biasprm="0 -2.5 0"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  // same actlimited
  EXPECT_EQ(model->actuator_actlimited[0], 1);
  EXPECT_EQ(model->actuator_actlimited[1], 1);
  // same dyntype
  EXPECT_EQ(model->actuator_dyntype[0], mjDYN_INTEGRATOR);
  EXPECT_EQ(model->actuator_dyntype[1], mjDYN_INTEGRATOR);
  // same biastype
  EXPECT_EQ(model->actuator_biastype[0], mjBIAS_AFFINE);
  EXPECT_EQ(model->actuator_biastype[1], mjBIAS_AFFINE);
  // same gaintype
  EXPECT_EQ(model->actuator_gaintype[0], mjGAIN_FIXED);
  EXPECT_EQ(model->actuator_gaintype[1], mjGAIN_FIXED);
  // same gainprm
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[0], 2.5);
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN], 2.5);
  // same biasprm
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[0], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[1], -2.5);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[2], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[mjNBIAS], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[mjNBIAS + 1], -2.5);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[mjNBIAS + 2], 0.0);
  // same actrange
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 0], -1.57);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 1], 1.57);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 2], -1.57);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 3], 1.57);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, IntvelocityCheckDefaultsIfNotSpecified) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  // check that by default kp = 1
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[0], 1.0);
  // check that biasprm is (0, -1, 0)
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[0], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[1], -1.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[2], 0.0);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, IntvelocityNoActrangeThrowsError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid actrange for actuator"));
  EXPECT_THAT(error.data(), HasSubstr("line = 10"));
}

TEST_F(ActuatorParseTest, IntvelocityDefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <intvelocity kp="5"/>
      <default class="withactrange">
        <intvelocity kp="1" actrange="-1 1"/>
      </default>
    </default>
    <worldbody>
      <body>
        <joint name="hinge1"/>
        <joint name="hinge2"/>
        <geom type="box" size=".025 .025 .025"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge1" actrange="0 1"/>
      <intvelocity joint="hinge2" class="withactrange"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[0], 5);
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN], 1);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 0], 0);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 1], 1);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 2], -1);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 3], 1);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, AdhesionDefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <adhesion ctrlrange="0 3"/>
    </default>
    <worldbody>
      <body name="sphere">
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <adhesion body="sphere"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->actuator_ctrlrange[0], 0);
  EXPECT_EQ(model->actuator_ctrlrange[1], 3);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, ErrorBadAdhesionDefaults) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <adhesion ctrlrange="-1 0"/>
    </default>
    <worldbody>
      <body name="sphere">
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <adhesion body="sphere"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("adhesion control range cannot be negative"));
}

// make sure range requirement is not enforced at parse time
TEST_F(ActuatorParseTest, DampersDontRequireRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <damper ctrlrange="0 2"/>
    </default>
    <worldbody>
      <body name="sphere">
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->actuator_ctrlrange[0], 0);
  EXPECT_EQ(model->actuator_ctrlrange[1], 2);
  mj_deleteModel(model);
}

// adhesion actuators inherit from general defaults
TEST_F(ActuatorParseTest, AdhesionInheritsFromGeneral) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <general dyntype="filter" dynprm="123" gainprm="5"/>
      <adhesion ctrlrange="0 2"/>
    </default>
    <worldbody>
      <body name="sphere">
        <geom name="sphere" size="1"/>
      </body>
    </worldbody>
    <actuator>
      <adhesion name="adhere" body="sphere"/>
    </actuator>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  // expect that gainprm was inherited from the general default
  EXPECT_EQ(model->actuator_gainprm[0], 5);
  // expect that dynprm was inherited from the general default
  EXPECT_EQ(model->actuator_dynprm[0], 123);
  // expect that dyntype was inherited from the general default
  EXPECT_EQ(model->actuator_dyntype[0], mjDYN_FILTER);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, ActdimDefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <general actdim="2"/>
    </default>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="user"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  // expect that actdim was inherited from the general default
  EXPECT_EQ(model->actuator_actnum[0], 2);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, MusclesParseSmoothdyn) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge" limited="true" range="-1 1"/>
      </body>
    </worldbody>
    <actuator>
      <muscle joint="hinge"/>
      <muscle joint="hinge" tausmooth="0.4"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->actuator_dynprm[2], 0.0);
  EXPECT_EQ(model->actuator_dynprm[mjNDYN + 2], 0.4);
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, MusclesSmoothdynNegative) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge" limited="true" range="-1 1"/>
      </body>
    </worldbody>
    <actuator>
      <muscle joint="hinge" tausmooth="-0.4"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("muscle tausmooth cannot be negative"));
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
}

TEST_F(ActuatorParseTest, GroupDisable) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option actuatorgroupdisable="0 3 8 3"/>
    <!-- note: repeated numbers are okay -->
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->opt.disableactuator, (1<<0) + (1<<3) + (1<<8));
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, GroupDisableNegative) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option actuatorgroupdisable="0 -3 5"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("must be non-negative"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(ActuatorParseTest, GroupDisableTooBig) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option actuatorgroupdisable="0 31"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("cannot exceed 30"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

// ------------- test sensor parsing -------------------------------------------

using SensorParseTest = MujocoTest;

TEST_F(SensorParseTest, UserObjTypeNoName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <sensor>
      <user dim="1" needstage="vel" objtype="site"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("objtype 'site' given but"));
  EXPECT_THAT(error.data(), HasSubstr("line 4"));
}

TEST_F(SensorParseTest, UserObjNameNoType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <sensor>
      <user dim="1" needstage="vel" objname="kevin"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("objname 'kevin' given but"));
  EXPECT_THAT(error.data(), HasSubstr("line 4"));
}

TEST_F(SensorParseTest, UserNeedstageAcc) {
  static constexpr char xml[] = R"(
  <mujoco>
    <sensor>
      <user dim="1" needstage="vel"/>
      <user dim="1"/>
      <user dim="1" needstage="pos"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->sensor_needstage[0], mjSTAGE_VEL);
  EXPECT_EQ(model->sensor_needstage[1], mjSTAGE_ACC);
  EXPECT_EQ(model->sensor_needstage[2], mjSTAGE_POS);
  mj_deleteModel(model);
}

// ------------- test general parsing ------------------------------------------

TEST_F(XMLReaderTest, ZnearZeroNotAllowed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <map znear="0"/>
    </visual>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("znear must be strictly positive"));
  EXPECT_THAT(error.data(), HasSubstr("line 4"));
}

TEST_F(XMLReaderTest, ZnearNegativeNotAllowed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <map znear="-1"/>
    </visual>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("znear must be strictly positive"));
  EXPECT_THAT(error.data(), HasSubstr("line 4"));
}

TEST_F(XMLReaderTest, ExtentZeroNotAllowed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <statistic extent="0"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("extent must be strictly positive"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

TEST_F(XMLReaderTest, ExtentNegativeNotAllowed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <statistic extent="-1"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("extent must be strictly positive"));
  EXPECT_THAT(error.data(), HasSubstr("line 3"));
}

}  // namespace
}  // namespace mujoco
