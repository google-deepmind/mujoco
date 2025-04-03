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
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "src/engine/engine_util_errmem.h"
#include "src/xml/xml_api.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::string;
using ::testing::AllOf;
using ::testing::ElementsAre;
using ::testing::Eq;
using ::testing::FloatEq;
using ::testing::HasSubstr;
using ::testing::IsNan;
using ::testing::IsNull;
using ::testing::NotNull;

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
  EXPECT_THAT(error.data(), HasSubstr("bad format in attribute 'axisangle'"));
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
  EXPECT_THAT(error.data(), HasSubstr("bad format in attribute 'axisangle'"));
  EXPECT_THAT(error.data(), HasSubstr("line 5"));
}

TEST_F(XMLReaderTest, InvalidNumberRange) {
    static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh" file="mesh.stl" face="100000000000000000000000"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("number is too large in attribute 'face'"));
  EXPECT_THAT(error.data(), HasSubstr("line 4"));
}

TEST_F(XMLReaderTest, InvalidNumberOfAttributes) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 .3">
        <freejoint/>
        <geom name="ellipsoid" type="ellipsoid" size=".1 .1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("size 2 must be positive"));
  EXPECT_THAT(error.data(), HasSubstr("line 6"));
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

TEST_F(XMLReaderTest, ClassOverridesChildclass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="size2">
        <geom size="2"/>
      </default>
      <default class="size3">
        <geom size="3"/>
      </default>
    </default>
    <worldbody>
      <frame childclass="size2">
        <geom/>
        <geom class="size3"/>
        <body childclass="size2">
          <geom/>
          <geom class="size3"/>
        </body>
      </frame>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->geom_size[3*0], 2);
  EXPECT_EQ(model->geom_size[3*1], 3);
  EXPECT_EQ(model->geom_size[3*2], 2);
  EXPECT_EQ(model->geom_size[3*3], 3);
  mj_deleteModel(model);
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

TEST_F(XMLReaderTest, InvalidDefaultClassName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="sphere">
        <geom type="sphere" size="1"/>
      </default>
    </default>
    <worldbody>
      <body>
        <geom class="invalid"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              AllOf(HasSubstr("unknown default class name 'invalid'"),
                    HasSubstr("Element 'geom'"), HasSubstr("line 10")));
}

TEST_F(XMLReaderTest, InvalidTopDefaultClassName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default class="sphere">
      <geom type="sphere" size="1"/>
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
  EXPECT_THAT(error.data(),
              HasSubstr("top-level default class 'main' cannot be renamed"));
}

TEST_F(XMLReaderTest, ValidTopDefaultClassName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default class="main">
      <geom type="sphere" size="1"/>
    </default>
    <worldbody>
      <body>
        <geom class="main"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  mj_deleteModel(model);
}

// ------------------------ test including -------------------------------------

// tiny RGB 2 x 3 PNG file
static constexpr unsigned char kTinyPng[] = {
  0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02,
  0x08, 0x02, 0x00, 0x00, 0x00, 0x12, 0x16, 0xf1, 0x4d, 0x00, 0x00, 0x00,
  0x1c, 0x49, 0x44, 0x41, 0x54, 0x08, 0xd7, 0x63, 0x78, 0xc1, 0xc0, 0xc0,
  0xc0, 0xf0, 0xbf, 0xb8, 0xb8, 0x98, 0x81, 0xe1, 0x3f, 0xc3, 0xff, 0xff,
  0xff, 0xc5, 0xc4, 0xc4, 0x00, 0x46, 0xd7, 0x07, 0x7f, 0xd2, 0x52, 0xa1,
  0x41, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60,
  0x82
};

// mesh OBJ file of a cube
static constexpr char kTinyObj[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";


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

  mj_addBufferVFS(vfs.get(), "model1.xml", xml1, sizeof(xml1));
  mj_addBufferVFS(vfs.get(), "model2.xml", xml2, sizeof(xml2));
  mj_addBufferVFS(vfs.get(), "model3.xml", xml3, sizeof(xml3));

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(),
                                       error.size(), vfs.get());
  ASSERT_THAT(model, NotNull()) << error.data();
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

  mj_addBufferVFS(vfs.get(), "model1.xml", xml1, sizeof(xml1));

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

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "model.xml", xml, sizeof(xml));
  mj_addBufferVFS(&vfs, "submodels/model1.xml", xml1, sizeof(xml1));
  mj_addBufferVFS(&vfs, "submodels/model2.xml", xml2, sizeof(xml2));
  mj_addBufferVFS(&vfs, "submodels/subsubmodels/model3.xml", xml3,
                  sizeof(xml3));

  std::array<char, 1024> error;
  mjModel* model = mj_loadXML("model.xml", &vfs, error.data(),
                              error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "ball"), 2);
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "another_box"), 3);
  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
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

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "model.xml", xml, sizeof(xml));
  mj_addBufferVFS(&vfs, "model1.xml", xml1, sizeof(xml1));
  mj_addBufferVFS(&vfs, "submodels/model2.xml", xml2, sizeof(xml2));
  mj_addBufferVFS(&vfs, "subsubmodels/model3.xml", xml3, sizeof(xml3));

  std::array<char, 1024> error;
  mjModel* model = mj_loadXML("model.xml", &vfs,
                              error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "ball"), 2);
  EXPECT_EQ(mj_name2id(model, mjOBJ_GEOM, "another_box"), 3);
  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
}

TEST_F(XMLReaderTest, MaterialTextureTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture file="tiny0.png" type="2d" name="tiny0"/>
      <texture file="tiny1.png" type="2d" name="tiny1"/>
      <material name="material">
        <layer role="occlusion" texture="tiny0"/>
        <layer role="roughness" texture="tiny0"/>
        <layer role="metallic" texture="tiny0"/>
        <layer role="rgb" texture="tiny1"/>
      </material>
    </asset>
    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tiny0.png", kTinyPng, sizeof(kTinyPng));
  mj_addBufferVFS(&vfs, "tiny1.png", kTinyPng, sizeof(kTinyPng));
  mj_addBufferVFS(&vfs, "model.xml", xml, sizeof(xml));

  char error[1024];
  mjModel* model = mj_loadXML("model.xml", &vfs, error, 1024);

  EXPECT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->mat_texid[mjTEXROLE_RGB], 1);
  EXPECT_EQ(model->mat_texid[mjTEXROLE_METALLIC], 0);
  EXPECT_EQ(model->mat_texid[mjTEXROLE_ROUGHNESS], 0);
  EXPECT_EQ(model->mat_texid[mjTEXROLE_OCCLUSION], 0);

  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
}

TEST_F(XMLReaderTest, LegacyMaterialTextureTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture file="tiny0.png" type="2d" name="tiny0"/>
      <texture file="tiny1.png" type="2d" name="tiny1"/>
      <material name="material" texture="tiny1"/>
    </asset>
    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tiny0.png", kTinyPng, sizeof(kTinyPng));
  mj_addBufferVFS(&vfs, "tiny1.png", kTinyPng, sizeof(kTinyPng));
  mj_addBufferVFS(&vfs, "model.xml", xml, sizeof(xml));

  char error[1024];
  mjModel* model = mj_loadXML("model.xml", &vfs, error, 1024);

  EXPECT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->mat_texid[mjTEXROLE_RGB], 1);

  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
}

TEST_F(XMLReaderTest, MaterialTextureFailTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture file="tiny0.png" type="2d" name="tiny0"/>
      <texture file="tiny1.png" type="2d" name="tiny1"/>
      <material name="material" texture="tiny1">
        <layer role="rgb" texture="tiny1"/>
        <layer role="occlusion" texture="tiny0"/>
      </material>
    </asset>
    <worldbody>
      <geom type="plane" material="material" size="4 4 4"/>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("A material with a texture attribute "
                                      "cannot have layer sub-elements"));
}

TEST_F(XMLReaderTest, LargeTextureTest) {
  static constexpr char xml[] = R"(
  <mujoco>
  <asset>
    <!--
      Use a texture width that exceeds the maximum texture size. For cube
      textures, the height is ignored and set to width*6. The default number of
      channels is 3.
      The width in this test is chosen so that 6*width*width*3 is too large to
      be represented as an integer.
    -->
    <texture name="tex" builtin="gradient" width="10923" height="2"/>
  </asset>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  EXPECT_THAT(model, IsNull());
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, HugeTextureTest) {
  static constexpr char xml[] = R"(
  <mujoco>
  <asset>
    <!--
      Use a texture width that exceeds the maximum texture size. For cube
      textures, the height is ignored and set to width*6. The default number of
      channels is 3.
      The width in this test is chosen so that 6*width*width*3 is so large that
      it overflows and becomes a positive integer.
    -->
    <texture name="tex" builtin="gradient" width="15447" height="2"/>
  </asset>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  EXPECT_THAT(model, IsNull());
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
    <mesh name="cube" file="cube.obj"/>
    <material name="submaterial" texture="subtiny"/>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "assets/tiny.png", kTinyPng, sizeof(kTinyPng));
  mj_addBufferVFS(&vfs, "assets/subassets/subtiny.png", kTinyPng,
                  sizeof(kTinyPng));
  mj_addBufferVFS(&vfs, "assets/subassets/cube.obj", kTinyObj,
                  sizeof(kTinyObj));
  mj_addBufferVFS(&vfs, "assets/assets.xml", assets, sizeof(assets));
  mj_addBufferVFS(&vfs, "assets/subassets/assets.xml", subassets,
                  sizeof(subassets));
  mj_addBufferVFS(&vfs, "model.xml", xml, sizeof(xml));

  // loading the file should be successful
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML("model.xml", &vfs, error.data(),
                              error.size());

  ASSERT_THAT(model, NotNull()) << error.data();

  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
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

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "assets/tiny.png", kTinyPng, sizeof(kTinyPng));

  // need to fallback for backwards compatibility
  mj_addBufferVFS(&vfs, "subtiny.png", kTinyPng, sizeof(kTinyPng));

  mj_addBufferVFS(&vfs, "assets/assets.xml", assets, sizeof(assets));
  mj_addBufferVFS(&vfs, "assets/subassets/assets.xml", subassets,
                  sizeof(subassets));
  mj_addBufferVFS(&vfs, "model.xml", xml, sizeof(xml));

  // loading the file should be successful
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML("model.xml", &vfs,
                              error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  mj_deleteModel(model);
  mj_deleteVFS(&vfs);
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
  ASSERT_THAT(model, NotNull()) << error.data();

  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, IncludeAbsoluteMeshDirTest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <include file="assets.xml"/>
  </mujoco>
  )";
  static constexpr char assets[] = R"(
  <mujoco>
    <compiler meshdir="mjMock.IncludeAbsoluteMeshDirTest:/assets"/>
    <asset>
    <mesh file="cube.obj"/>
  </asset>
  </mujoco>
  )";

  static constexpr char cube[] = R"(
  v -0.500000 -0.500000  0.500000
  v  0.500000 -0.500000  0.500000
  v -0.500000  0.500000  0.500000
  v  0.500000  0.500000  0.500000
  v -0.500000  0.500000 -0.500000
  v  0.500000  0.500000 -0.500000
  v -0.500000 -0.500000 -0.500000
  v  0.500000 -0.500000 -0.500000)";

  MockFilesystem fs("IncludeAbsoluteMeshDirTest");
  fs.AddFile("/assets/cube.obj", (const unsigned char*) cube, sizeof(cube));
  fs.AddFile("assets.xml", (const unsigned char*) assets,
   sizeof(assets));
  fs.AddFile("model.xml", (const unsigned char*) xml, sizeof(xml));
  std::string modelpath = fs.FullPath("model.xml");

  std::array<char, 1024> error;
  // loading the file should be successful
  mjModel* model = mj_loadXML(modelpath.c_str(), nullptr,
                              error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, ParsePolycoef) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="0"/>
        <geom size="1"/>
      </body>
      <body>
        <joint name="1"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <equality>
      <joint joint1="0" joint2="1"/>
      <joint joint1="0" joint2="1" polycoef="2"/>
      <joint joint1="0" joint2="1" polycoef="3 4"/>
      <joint joint1="0" joint2="1" polycoef="5 6 7 8 9"/>
    </equality>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_THAT(AsVector(m->eq_data + 0*mjNEQDATA, 5),
              ElementsAre(0, 1, 0, 0, 0));
  EXPECT_THAT(AsVector(m->eq_data + 1*mjNEQDATA, 5),
              ElementsAre(2, 1, 0, 0, 0));
  EXPECT_THAT(AsVector(m->eq_data + 2*mjNEQDATA, 5),
              ElementsAre(3, 4, 0, 0, 0));
  EXPECT_THAT(AsVector(m->eq_data + 3*mjNEQDATA, 5),
              ElementsAre(5, 6, 7, 8, 9));
  mj_deleteModel(m);
}

TEST_F(XMLReaderTest, TendonArmature) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
      <body pos="1 0 0">
        <joint name="slide" type="slide"/>
        <geom size=".1"/>
        <site name="b"/>
      </body>
    </worldbody>

    <tendon>
      <spatial armature="1.5">
        <site site="a"/>
        <site site="b"/>
      </spatial>
      <fixed armature="2.5">
        <joint joint="slide" coef="1"/>
      </fixed>
      <fixed>
        <joint joint="slide" coef="2"/>
      </fixed>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_EQ(m->ntendon, 3);
  EXPECT_FLOAT_EQ(m->tendon_armature[0], 1.5);
  EXPECT_FLOAT_EQ(m->tendon_armature[1], 2.5);
  EXPECT_FLOAT_EQ(m->tendon_armature[2], 0);
  mj_deleteModel(m);
}

TEST_F(XMLReaderTest, TendonArmatureNegative) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
      <site name="b" pos="1 0 0"/>
    </worldbody>

    <tendon>
      <spatial armature="-1.5">
        <site site="a"/>
        <site site="b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("tendon armature cannot be negative"));
}

TEST_F(XMLReaderTest, TendonArmatureGeomWrap) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
      <geom name="g" type="sphere" size=".1"/>
      <site name="b" pos="1 0 0"/>
    </worldbody>

    <tendon>
      <spatial armature="1.5">
        <site site="a"/>
        <geom geom="g"/>
        <site site="b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("geom wrapping not supported"));
}

// ------------------------ test frame parsing ---------------------------------
TEST_F(XMLReaderTest, ParseFrame) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="frame">
        <geom size=".1"/>
      </default>
      <default class="body">
        <geom size=".2"/>
      </default>
      <default class="geom">
        <geom size=".3"/>
      </default>
    </default>

    <worldbody>
      <frame euler="0 0 30">
        <geom size=".5" euler="0 0 20"/>
      </frame>

      <body>
        <frame pos="0 1 0" childclass="frame">
          <geom pos="0 1 0"/>
          <body pos="1 0 0" childclass="body">
            <geom pos="0 0 1"/>
            <geom pos="0 0 1" class="geom"/>
          </body>
        </frame>
      </body>

      <frame euler="0 0 30">
        <frame euler="0 0 20">
          <geom size=".6"/>
        </frame>
      </frame>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_THAT(m->geom_size[ 0], .5);
  EXPECT_THAT(m->geom_size[ 3], .6);
  EXPECT_THAT(m->geom_size[ 6], .1);
  EXPECT_THAT(m->geom_size[ 9], .2);
  EXPECT_THAT(m->geom_size[12], .3);
  mj_deleteModel(m);
}

TEST_F(XMLReaderTest, DuplicateFrameName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <frame name="frame1" euler="0 0 30">
        <geom size=".1"/>
      </frame>

      <frame name="frame1" euler="0 0 30">
        <geom size=".1"/>
      </frame>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("repeated name 'frame1'"));
}


// ---------------------- test replicate parsing -------------------------------

TEST_F(XMLReaderTest, ParseReplicate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="texture" type="2d" builtin="checker" width="32" height="32"/>
      <material name="material" texture="texture" texrepeat="1 1" texuniform="true"/>
    </asset>

    <worldbody>
      <replicate count="101" offset="3 0 .1" euler="0 0 1.8">
        <body name="body" pos="0 -1 0">
          <joint type="slide"/>
          <geom name="g" size="1"/>
        </body>
      </replicate>

      <replicate count="2" offset="1 0 0">
        <replicate count="2" offset="0 1 0" sep="_">
          <geom name="geom" size="1" pos="0 0 1" material="material"/>
          <site name="site" pos="1 0 0"/>
        </replicate>
      </replicate>
    </worldbody>

    <sensor>
      <framepos name="sensor" objtype="site" objname="site"/>
    </sensor>

    <keyframe>
      <key name="keyframe" qpos="1"/>
    </keyframe>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, testing::NotNull()) << error.data();
  EXPECT_THAT(m->ngeom, 105);
  EXPECT_THAT(m->nsensor, 4);
  EXPECT_THAT(m->nbody, 102);

  // check that the separator is used correctly
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      char geom_name[mjMAXUINAME] = "";
      util::strcat_arr(geom_name, m->names + m->name_geomadr[2*i+j]);
      EXPECT_THAT(m->geom_pos[6*i+3*j+0], i);
      EXPECT_THAT(m->geom_pos[6*i+3*j+1], j);
      EXPECT_THAT(m->geom_pos[6*i+3*j+2], 1);
      EXPECT_THAT(std::string(geom_name),
                  "geom_" + std::to_string(j) + std::to_string(i));
    }
  }

  // check that 3 digits are used if count > 99
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 10; ++j) {
      for (int k = 0; k < 10; ++k) {
        int ngeom = 100*i+10*j+k;
        if (ngeom > 99) break;
        char geom_name[mjMAXUINAME] = "";
        util::strcat_arr(geom_name, m->names + m->name_geomadr[4+ngeom]);
        EXPECT_THAT(
            std::string(geom_name),
            "g" + std::to_string(i) + std::to_string(j) + std::to_string(k));
      }
    }
  }

  // check body positions
  mjtNum pos[2] = {0, 0};
  for (int i = 1; i < 102; ++i) {
    mjtNum theta = (i-1) * 1.8 * mjPI / 180;
    EXPECT_NEAR(m->body_pos[3*i+0], pos[0] + sin(theta), 1e-8) << i;
    EXPECT_NEAR(m->body_pos[3*i+1], pos[1] - cos(theta), 1e-8) << i;
    EXPECT_NEAR(m->body_pos[3*i+2], (i-1) * .1, 1e-8);
    pos[0] += 3 * cos(theta);
    pos[1] += 3 * sin(theta);
  }

  // check that the final pose is correct
  int n = m->nbody-1;
  EXPECT_NEAR(m->body_quat[4*n+0], 0, 1e-8);
  EXPECT_EQ(m->body_quat[4*n+1], 0);
  EXPECT_EQ(m->body_quat[4*n+2], 0);
  EXPECT_EQ(m->body_quat[4*n+3], 1);

  // check that the keyframe is resized
  EXPECT_THAT(m->nkey, 102);
  EXPECT_THAT(m->nq, 101);
  for (int i = 0; i < m->nkey; i++) {
    for (int j = 0; j < m->nq; j++) {
      EXPECT_THAT(m->key_qpos[i*m->nq+j], i == j ? 1 : 0) << i << " " << j;
    }
  }

  mj_deleteModel(m);
}

TEST_F(XMLReaderTest, ParseReplicatePartialReference) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="1 1 1"/>
      <site name="world"/>

      <replicate count="2" offset="1 0 0">
        <body name="replicated">
          <freejoint/>
          <site name="replicated"/>
          <geom name="replicated" size="1"/>
        </body>
      </replicate>
    </worldbody>

    <contact>
      <pair geom1="floor" geom2="replicated" condim="1"/>
      <exclude body1="world" body2="replicated" />
    </contact>

    <tendon>
      <spatial>
        <site site="world"/>
        <site site="replicated"/>
      </spatial>
    </tendon>

    <sensor>
      <framepos name="sensor" objtype="site" objname="replicated" reftype="site" refname="world"/>
    </sensor>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, testing::NotNull()) << error.data();

  EXPECT_THAT(m->nbody, 3);
  EXPECT_THAT(m->ngeom, 3);
  EXPECT_THAT(m->npair, 2);
  EXPECT_THAT(m->nexclude, 2);
  EXPECT_THAT(m->ntendon, 2);
  EXPECT_THAT(m->nsensor, 2);

  mj_deleteModel(m);
}

TEST_F(XMLReaderTest, ParseReplicateDefaultPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="body">
        <geom type="capsule"/>
      </default>
    </default>

    <worldbody>
      <replicate count="2" euler="0 0 16.36" sep="-">
        <body name="torso" pos="-5 0 1.282" childclass="body">
          <body name="head" pos="0 0 .19">
            <geom name="head" type="sphere" size=".09"/>
          </body>
        </body>
      </replicate>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << error.data();

  mjsBody* torso = mjs_findBody(spec, "torso-0");
  EXPECT_THAT(torso, NotNull());

  mjsDefault* def = mjs_getDefault(torso->element);
  EXPECT_THAT(def, NotNull());
  EXPECT_THAT(def->geom->type, mjGEOM_CAPSULE);

  mjsBody* head = mjs_findBody(spec, "head-0");
  EXPECT_THAT(head, NotNull());

  def = mjs_getDefault(head->element);
  EXPECT_THAT(def, NotNull());
  EXPECT_THAT(def->geom->type, mjGEOM_CAPSULE);

  mj_deleteSpec(spec);
}

TEST_F(XMLReaderTest, ParseReplicateRepeatedName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler>
      <lengthrange mode="all"/>
    </compiler>

    <worldbody>
      <light pos="0 0 10"/>

      <replicate offset="0 .1 0" count="2">
        <site name="a" size=".02"/>
        <body pos="1 0 1">
          <joint axis="0 -1 0" range="0 90"/>
          <geom type="capsule" size=".02" fromto="0 0 0 0 0 -1"/>
          <site name="b" pos="0 0 -1"/>
        </body>
      </replicate>
    </worldbody>

    <tendon>
      <spatial name="b">
        <site site="a"/>
        <site site="b"/>
      </spatial>
    </tendon>

    <actuator>
      <position name="b" tendon="b0" ctrlrange="0 3" kp="100" dampratio="1"/>
      <position name="b" tendon="b1" ctrlrange="0 3" kp="100" dampratio="1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("repeated name 'b' in actuator"));
  EXPECT_THAT(error.data(), HasSubstr("Element 'replicate'"));
}

TEST_F(XMLReaderTest, RepeatedPrefix) {
  static constexpr char parent[] = R"(
  <mujoco>
    <asset>
      <model name="1" file="child_1.xml" content_type="text/xml" />
      <model name="2" file="child_2.xml" content_type="text/xml" />
    </asset>

    <worldbody>
      <attach model="1" body="1" prefix="prefix-"/>
      <replicate count="1">
        <attach model="2" body="2" prefix="prefix-"/>
      </replicate>
    </worldbody>
  </mujoco>
  )";

  static constexpr char child_1[] = R"(
  <mujoco>
    <asset>
      <model name="2" file="child_2.xml" content_type="text/xml"/>
    </asset>

    <worldbody>
      <body name="1">
        <body name="2">
          <attach model="2" body="2" prefix="prefix2"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char child_2[] = R"(
  <mujoco>
    <worldbody>
      <body name="2"/>
    </worldbody>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "child_1.xml", child_1, sizeof(child_1));
  mj_addBufferVFS(vfs.get(), "child_2.xml", child_2, sizeof(child_2));

  std::array<char, 1024> err;
  mjSpec* c2 = mj_parseXMLString(child_2, 0, err.data(), err.size());
  EXPECT_THAT(c2, NotNull()) << err.data();
  mjSpec* c1 = mj_parseXMLString(child_1, vfs.get(), err.data(), err.size());
  EXPECT_THAT(c1, NotNull()) << err.data();
  mj_deleteSpec(c1);
  mj_deleteSpec(c2);

  mjSpec* spec = mj_parseXMLString(parent, vfs.get(), err.data(), err.size());
  EXPECT_THAT(spec, IsNull());
  EXPECT_THAT(err.data(), HasSubstr("mismatched parents"));
  mj_deleteSpec(spec);
  mj_deleteVFS(vfs.get());
}

TEST_F(XMLReaderTest, ParseReplicateExcludeTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="winch" pos="-.01 0 .35">
        <joint name="winch" damping="1"/>
        <geom type="cylinder" size=".015 .01"/>
        <site name="anchor" pos=".1 0 .04"/>
      </body>
      <site name="pulley" pos=".1 0 .32"/>
      <site name="hook_left" pos=".08 0 .3"/>
      <site name="hook_right" pos=".12 0 .3"/>
      <body name="sphere" pos=".1 0 .2">
        <freejoint/>
        <geom type="sphere" size=".03"/>
        <site name="pin_left" pos="-.025 0 .025"/>
        <site name="pin_right" pos=".025 0 .025"/>
      </body>

      <replicate count="4" offset=".025 0 0">
        <replicate count="4" offset="0 .025 0">
          <replicate count="4" offset="0 0 .025">
            <body pos=".06 -.04 .05">
              <geom type="sphere" size=".012"/>
            </body>
          </replicate>
        </replicate>
      </replicate>
    </worldbody>

    <tendon>
      <spatial range="0 .19" limited="true">
        <site site="anchor"/>
        <site site="pulley"/>
        <pulley divisor="3"/>
        <site site="pulley"/>
        <site site="hook_left"/>
        <site site="pin_left"/>
        <pulley divisor="3"/>
        <site site="pulley"/>
        <site site="hook_right"/>
        <site site="pin_right"/>
      </spatial>
    </tendon>

    <actuator>
      <position name="winch" joint="winch" ctrlrange="-.7 .5" ctrllimited="true" kp="10"/>
  </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << error.data();
  mjModel* m = mj_compile(spec, 0);
  EXPECT_THAT(m, NotNull());
  EXPECT_THAT(m->nbody, 67);
  EXPECT_THAT(m->ngeom, 66);
  EXPECT_THAT(m->nsite, 6);
  EXPECT_THAT(m->nu, 1);
  EXPECT_THAT(m->ntendon, 1);
  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

TEST_F(XMLReaderTest, ParseReplicateWithTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <replicate count="2" offset=".025 0 0">
        <replicate count="2" offset="0 .025 0">
          <replicate count="2" offset="0 0 .025">
            <body name="winch" pos="-.01 0 .35">
              <joint name="winch" damping="1"/>
              <geom type="cylinder" size=".015 .01"/>
              <site name="anchor" pos=".1 0 .04"/>
            </body>
            <site name="pulley" pos=".1 0 .32"/>
            <site name="hook_left" pos=".08 0 .3"/>
            <site name="hook_right" pos=".12 0 .3"/>
            <body name="sphere" pos=".1 0 .2">
              <freejoint/>
              <geom type="sphere" size=".03"/>
              <site name="pin_left" pos="-.025 0 .025"/>
              <site name="pin_right" pos=".025 0 .025"/>
            </body>
            <body pos=".06 -.04 .05">
              <geom type="sphere" size=".012"/>
            </body>
          </replicate>
        </replicate>
      </replicate>
    </worldbody>

    <tendon>
      <spatial range="0 .19" limited="true" name="tendon">
        <site site="anchor"/>
        <site site="pulley"/>
        <pulley divisor="3"/>
        <site site="pulley"/>
        <site site="hook_left"/>
        <site site="pin_left"/>
        <pulley divisor="3"/>
        <site site="pulley"/>
        <site site="hook_right"/>
        <site site="pin_right"/>
      </spatial>
    </tendon>

    <actuator>
      <position name="winch" joint="winch" ctrlrange="-.7 .5" ctrllimited="true" kp="10"/>
      <position name="tendon" tendon="tendon" ctrlrange="0 1" kp="100" dampratio="1"/>
  </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << error.data();
  mjModel* m = mj_compile(spec, 0);
  EXPECT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_THAT(m->nbody, 25);
  EXPECT_THAT(m->ngeom, 24);
  EXPECT_THAT(m->nsite, 48);
  EXPECT_THAT(m->nu, 16);
  EXPECT_THAT(m->ntendon, 8);
  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// ---------------------- test spec assets parsing -----------------------------

TEST_F(XMLReaderTest, ParseSpecAssets) {
  std::array<char, 1024> er;

  static const char* const kParentPath =
      "xml/testdata/parent.xml";
  static const char* const kChildPath =
      "xml/testdata/child.xml";

  const std::string xml_parent = GetTestDataFilePath(kParentPath);
  const std::string xml_child = GetTestDataFilePath(kChildPath);

  mjModel* parent = mj_loadXML(xml_parent.c_str(), 0, er.data(), er.size());
  EXPECT_THAT(parent, NotNull()) << er.data();

  mjModel* child = mj_loadXML(xml_child.c_str(), 0, er.data(), er.size());
  EXPECT_THAT(child, NotNull()) << er.data();

  mj_deleteModel(parent);
  mj_deleteModel(child);
}

TEST_F(XMLReaderTest, AttachSpecAssets) {
  static constexpr char xml_parent[] = R"(
  <mujoco>
    <asset>
      <model name="child" file="xml_child.xml"/>
    </asset>
    <worldbody>
      <body name="parent">
        <geom name="geom" size="2"/>
        <replicate count="2" offset="0 0 1">
          <attach model="child" body="body" prefix="other"/>
        </replicate>
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

  static constexpr char xml_expected[] = R"(
  <mujoco>
    <worldbody>
      <body name="parent">
        <geom name="geom" size="2"/>
        <body name="otherbody0">
          <geom name="othergeom0" size="1" pos="2 0 0"/>
        </body>
        <body name="otherbody1" pos="0 0 1">
          <geom name="othergeom1" size="1" pos="2 0 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "xml_child.xml", xml_child, sizeof(xml_child));

  std::array<char, 1024> er;
  mjModel* model =
      LoadModelFromString(xml_parent, er.data(), er.size(), vfs.get());
  EXPECT_THAT(model, NotNull()) << er.data();

  mjModel* expected = LoadModelFromString(xml_expected, er.data(), er.size());
  EXPECT_THAT(expected, NotNull()) << er.data();

  mjtNum tol = 0;
  std::string field = "";
  EXPECT_LE(CompareModel(model, expected, field), tol)
            << "Expected and attached models are different!\n"
            << "Different field: " << field << '\n';;

  mj_deleteModel(model);
  mj_deleteModel(expected);
  mj_deleteVFS(vfs.get());
}

TEST_F(XMLReaderTest, InvalidAttach) {
  static constexpr char xml_parent[] = R"(
  <mujoco>
    <asset>
      <model name="other" file="child.xml"/>
    </asset>
    <worldbody>
      <body name="parent">
        <joint name="joint1"/>
        <geom size="2"/>
        <attach model="other" body="body" prefix="_"/>
      </body>
    </worldbody>

    <actuator>
      <motor name="_actuator" joint="joint1"/>
    </actuator>
  </mujoco>
  )";

  static constexpr char xml_child[] = R"(
  <mujoco model="child">
    <worldbody>
      <body name="body">
        <joint name="joint2"/>
        <geom size="1" pos="2 0 0"/>
      </body>
    </worldbody>

    <actuator>
      <motor name="actuator" joint="joint2"/>
    </actuator>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "child.xml", xml_child, sizeof(xml_child));

  std::array<char, 1024> er;
  mjModel* model =
      LoadModelFromString(xml_parent, er.data(), er.size(), vfs.get());

  EXPECT_THAT(model, IsNull()) << er.data();
  EXPECT_THAT(er.data(), HasSubstr("repeated name '_actuator' in actuator"));
  EXPECT_THAT(er.data(), HasSubstr("Element 'attach'"));
  mj_deleteVFS(vfs.get());
}

TEST_F(XMLReaderTest, LookupCompilerOptionWithoutSpecCopy) {
  static constexpr char child_xml[] = R"(
  <mujoco>
    <compiler angle="radian"/>

    <worldbody>
      <body name="child">
        <geom type="box" size="1 1 1"/>
        <joint name="child_joint" range="-3.1415926 3.1415926"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char parent_xml[] = R"(
  <mujoco>
    <asset>
      <model name="child" file="child.xml"/>
    </asset>

    <worldbody>
      <body name="parent">
        <geom type="box" size="1 1 1"/>
        <joint name="parent_joint" range="-180 180"/>
        <attach model="child" body="child" prefix="child_"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "child.xml", child_xml, sizeof(child_xml));
  mj_addBufferVFS(vfs.get(), "parent.xml", parent_xml, sizeof(parent_xml));

  std::array<char, 1024> error;
  auto* spec = mj_parseXMLString(parent_xml, vfs.get(), error.data(),
                                 error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();

  mjModel* model = mj_compile(spec, vfs.get());
  EXPECT_THAT(model, NotNull());
  EXPECT_THAT(model->njnt, 2);
  EXPECT_NEAR(model->jnt_range[0], -3.14159, 1e-5);
  EXPECT_NEAR(model->jnt_range[1], 3.14159, 1e-5);
  EXPECT_NEAR(model->jnt_range[2], -3.14159, 1e-5);
  EXPECT_NEAR(model->jnt_range[3], 3.14159, 1e-5);

  mjSpec* copied_spec = mj_copySpec(spec);
  ASSERT_THAT(copied_spec, NotNull());
  mjModel* copied_model = mj_compile(copied_spec, vfs.get());
  EXPECT_THAT(copied_model, NotNull());
  EXPECT_THAT(copied_model->njnt, 2);
  EXPECT_NEAR(copied_model->jnt_range[0], -3.14159, 1e-5);
  EXPECT_NEAR(copied_model->jnt_range[1], 3.14159, 1e-5);
  EXPECT_NEAR(copied_model->jnt_range[2], -3.14159, 1e-5);
  EXPECT_NEAR(copied_model->jnt_range[3], 3.14159, 1e-5);

  mj_deleteSpec(spec);
  mj_deleteSpec(copied_spec);
  mj_deleteModel(model);
  mj_deleteModel(copied_model);

  mj_deleteVFS(vfs.get());
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

TEST_F(XMLReaderTest, CameraPrincipalRequiresSensorsize) {
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
  EXPECT_THAT(
      error.data(),
      HasSubstr(
          "fullinertia and inertial orientation cannot both be specified"));
}

TEST_F(XMLReaderTest, ReadShellParameter) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="0 2 1  2 0 3" inertia="shell"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="example_mesh"/>
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
      <body pos="1 -1 .6" name="B0_parent">
        <flexcomp name="B0" type="grid" count="4 4 1" spacing=".2 .2 .2" group="2" radius=".1" dim="2">
          <edge equality="true"/>
          <contact internal="false" selfcollide="none"/>
        </flexcomp>
      </body>
      <body pos="-1 1 .6" name="B1_parent">
        <flexcomp name="B1" type="grid" count="4 4 1" spacing=".2 .2 .2" group="4" radius=".1" dim="2">
          <edge equality="true"/>
          <contact internal="false" selfcollide="none"/>
        </flexcomp>
      </body>
    </worldbody>
    <deformable>
      <skin name="B0Skin" group="4" rgba="1 1 0 1" inflate="0.1"
        vertex="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
        face="0 4 5 0 5 1 1 5 6 1 6 2 2 6 7 2 7 3 4 8 9 4 9 5 5 9 10 5 10 6 6 10 11 6 11 7 8
              12 13 8 13 9 9 13 14 9 14 10 10 14 15 10 15 11 16 21 20 16 17 21 17 22 21 17 18
              22 18 23 22 18 19 23 20 25 24 20 21 25 21 26 25 21 22 26 22 27 26 22 23 27 24 29
              28 24 25 29 25 30 29 25 26 30 26 31 30 26 27 31 0 20 4 0 16 20 4 24 8 4 20 24 8
              28 12 8 24 28 3 7 23 3 23 19 7 11 27 7 27 23 11 15 31 11 31 27 0 1 17 0 17 16 1
              2 18 1 18 17 2 3 19 2 19 18 12 29 13 12 28 29 13 30 14 13 29 30 14 31 15 14 30 31">
        <bone body="B0_0" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0 16" vertweight="1 1"/>
        <bone body="B0_1" bindpos="0 0 0" bindquat="1 0 0 0" vertid="1 17" vertweight="1 1"/>
        <bone body="B0_2" bindpos="0 0 0" bindquat="1 0 0 0" vertid="2 18" vertweight="1 1"/>
        <bone body="B0_3" bindpos="0 0 0" bindquat="1 0 0 0" vertid="3 19" vertweight="1 1"/>
        <bone body="B0_4" bindpos="0 0 0" bindquat="1 0 0 0" vertid="4 20" vertweight="1 1"/>
        <bone body="B0_5" bindpos="0 0 0" bindquat="1 0 0 0" vertid="5 21" vertweight="1 1"/>
        <bone body="B0_6" bindpos="0 0 0" bindquat="1 0 0 0" vertid="6 22" vertweight="1 1"/>
        <bone body="B0_7" bindpos="0 0 0" bindquat="1 0 0 0" vertid="7 23" vertweight="1 1"/>
        <bone body="B0_8" bindpos="0 0 0" bindquat="1 0 0 0" vertid="8 24" vertweight="1 1"/>
        <bone body="B0_9" bindpos="0 0 0" bindquat="1 0 0 0" vertid="9 25" vertweight="1 1"/>
        <bone body="B0_10" bindpos="0 0 0" bindquat="1 0 0 0" vertid="10 26" vertweight="1 1"/>
        <bone body="B0_11" bindpos="0 0 0" bindquat="1 0 0 0" vertid="11 27" vertweight="1 1"/>
        <bone body="B0_12" bindpos="0 0 0" bindquat="1 0 0 0" vertid="12 28" vertweight="1 1"/>
        <bone body="B0_13" bindpos="0 0 0" bindquat="1 0 0 0" vertid="13 29" vertweight="1 1"/>
        <bone body="B0_14" bindpos="0 0 0" bindquat="1 0 0 0" vertid="14 30" vertweight="1 1"/>
        <bone body="B0_15" bindpos="0 0 0" bindquat="1 0 0 0" vertid="15 31" vertweight="1 1"/>
      </skin>
      <skin name="B1Skin" group="2" rgba="0 1 1 1" inflate="0.1"
        vertex="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
        face="0 4 5 0 5 1 1 5 6 1 6 2 2 6 7 2 7 3 4 8 9 4 9 5 5 9 10 5 10 6 6 10 11 6 11 7 8
              12 13 8 13 9 9 13 14 9 14 10 10 14 15 10 15 11 16 21 20 16 17 21 17 22 21 17 18
              22 18 23 22 18 19 23 20 25 24 20 21 25 21 26 25 21 22 26 22 27 26 22 23 27 24 29
              28 24 25 29 25 30 29 25 26 30 26 31 30 26 27 31 0 20 4 0 16 20 4 24 8 4 20 24 8
              28 12 8 24 28 3 7 23 3 23 19 7 11 27 7 27 23 11 15 31 11 31 27 0 1 17 0 17 16 1
              2 18 1 18 17 2 3 19 2 19 18 12 29 13 12 28 29 13 30 14 13 29 30 14 31 15 14 30 31">
        <bone body="B1_0" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0 16" vertweight="1 1"/>
        <bone body="B1_1" bindpos="0 0 0" bindquat="1 0 0 0" vertid="1 17" vertweight="1 1"/>
        <bone body="B1_2" bindpos="0 0 0" bindquat="1 0 0 0" vertid="2 18" vertweight="1 1"/>
        <bone body="B1_3" bindpos="0 0 0" bindquat="1 0 0 0" vertid="3 19" vertweight="1 1"/>
        <bone body="B1_4" bindpos="0 0 0" bindquat="1 0 0 0" vertid="4 20" vertweight="1 1"/>
        <bone body="B1_5" bindpos="0 0 0" bindquat="1 0 0 0" vertid="5 21" vertweight="1 1"/>
        <bone body="B1_6" bindpos="0 0 0" bindquat="1 0 0 0" vertid="6 22" vertweight="1 1"/>
        <bone body="B1_7" bindpos="0 0 0" bindquat="1 0 0 0" vertid="7 23" vertweight="1 1"/>
        <bone body="B1_8" bindpos="0 0 0" bindquat="1 0 0 0" vertid="8 24" vertweight="1 1"/>
        <bone body="B1_9" bindpos="0 0 0" bindquat="1 0 0 0" vertid="9 25" vertweight="1 1"/>
        <bone body="B1_10" bindpos="0 0 0" bindquat="1 0 0 0" vertid="10 26" vertweight="1 1"/>
        <bone body="B1_11" bindpos="0 0 0" bindquat="1 0 0 0" vertid="11 27" vertweight="1 1"/>
        <bone body="B1_12" bindpos="0 0 0" bindquat="1 0 0 0" vertid="12 28" vertweight="1 1"/>
        <bone body="B1_13" bindpos="0 0 0" bindquat="1 0 0 0" vertid="13 29" vertweight="1 1"/>
        <bone body="B1_14" bindpos="0 0 0" bindquat="1 0 0 0" vertid="14 30" vertweight="1 1"/>
        <bone body="B1_15" bindpos="0 0 0" bindquat="1 0 0 0" vertid="15 31" vertweight="1 1"/>
      </skin>
    </deformable>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  int flexid1 = mj_name2id(model, mjOBJ_FLEX, "B0");
  int flexid2 = mj_name2id(model, mjOBJ_FLEX, "B1");
  EXPECT_THAT(model->flex_group[flexid1], 2);
  EXPECT_THAT(model->skin_group[0], 4);
  EXPECT_THAT(model->flex_group[flexid2], 4);
  EXPECT_THAT(model->skin_group[1], 2);
  mj_deleteModel(model);
}

TEST_F(XMLReaderTest, InvalidSkinGroup) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <composite prefix="B0" type="grid" count="6 6 1">
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
}

TEST_F(XMLReaderTest, Orthographic) {
  static constexpr char xml[] = R"(
  <mujoco>
    <visual>
      <global orthographic="true" fovy="5"/>
      <map znear="0.01"/>
    </visual>

    <default>
      <camera orthographic="true"/>
    </default>

    <worldbody>
      <camera name="fovy=1" pos=".5 .5 2" orthographic="true" fovy="1"/>
      <camera name="fovy=2" pos="0 0 2" fovy="2"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();

  EXPECT_EQ(model->vis.global.orthographic, 1);
  EXPECT_EQ(model->cam_orthographic[0], 1);
  EXPECT_EQ(model->cam_orthographic[1], 1);
  EXPECT_EQ(model->cam_fovy[0], 1);
  EXPECT_EQ(model->cam_fovy[1], 2);

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

TEST_F(ActuatorParseTest, PositionTimeconst) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" timeconst="2"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  ASSERT_NEAR(model->actuator_dynprm[0], 2.0, 1e-6);
  EXPECT_THAT(model->actuator_dyntype[0], Eq(mjDYN_FILTEREXACT));
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, PositionTimeconstInheritrange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" range="-1 1"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" inheritrange="1" timeconst="2"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, PositionTimeconstDefault) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <position timeconst="1"/>
    </default>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  ASSERT_NEAR(model->actuator_dynprm[0], 1.0, 1e-6);
  EXPECT_THAT(model->actuator_dyntype[0], Eq(mjDYN_FILTEREXACT));
  mj_deleteModel(model);
}

TEST_F(ActuatorParseTest, PositionTimeconstDefaultOverride) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <position timeconst="1"/>
    </default>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" timeconst="0"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_FALSE(model->actuator_dynprm[0]);
  EXPECT_THAT(model->actuator_dyntype[0], Eq(mjDYN_NONE));
  mj_deleteModel(model);
}

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
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
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
  EXPECT_THAT(error.data(), HasSubstr("line 10"));
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

TEST_F(XMLReaderTest, LightRadius) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="r1">
        <light bulbradius="1"/>
      </default>
    </default>
    <worldbody>
      <light/>
      <light class="r1"/>
      <light class="r1" bulbradius="2"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_FLOAT_EQ(model->light_bulbradius[0], 0.02);
  EXPECT_FLOAT_EQ(model->light_bulbradius[1], 1);
  EXPECT_FLOAT_EQ(model->light_bulbradius[2], 2);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
