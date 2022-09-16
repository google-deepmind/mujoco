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

// Tests for xml/xml_native_writer.cc.

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>
#endif

#include <algorithm>
#include <array>
#include <clocale>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/container/flat_hash_set.h>
#include <absl/strings/match.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "src/xml/xml_numeric_format.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::Not;
using ::testing::NotNull;

using XMLWriterTest = MujocoTest;

TEST_F(XMLWriterTest, SavesDisableSensor) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag sensor="disable"/>
    </option>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("sensor=\"disable\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsEmptyClasses) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="empty_referenced"/>
      <default class="empty_unreferenced"/>
      <default class="regular">
        <geom size="0.3"/>
      </default>
    </default>
    <worldbody>
      <geom class="regular"/>
      <geom class="empty_referenced" size="0.2"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("default class=\"regular\""));
  EXPECT_THAT(saved_xml, HasSubstr("default class=\"empty_referenced\""));
  EXPECT_THAT(saved_xml, HasSubstr("default class=\"empty_unreferenced\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsExplicitInertial) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="0.2"/>
        <inertial pos="0 1 2" mass="3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("<inertial pos=\"0 1 2\" mass=\"3\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, NotAddsInertial) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="0.2"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("inertial")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DropsInertialIfFromGeom) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler inertiafromgeom="true"/>
    <worldbody>
      <body>
        <inertial pos="0 1 2" mass="3"/>
        <geom size="0.2"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("inertial")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepInferredJointLimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian" autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge" range="-1 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("limited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepExplicitJointLimitedIfAutoLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian" autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge" limited="true" range="-1 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("limited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsJointLimitedFalseIfAutoLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian" autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge" limited="false" range="-1 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("limited=\"false\" range=\"-1 1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepInferredTendonLimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian" autolimits="true" />
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
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("limited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepExplicitTendonLimitedIfAutoLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian" autolimits="true" />
    <worldbody>
      <body>
        <joint type="slide"/>
        <geom size="1"/>
        <site name="s1"/>
      </body>
      <site name="s2"/>
    </worldbody>
    <tendon>
      <spatial limited="true" range="-1 1">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("limited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsTendonLimitedFalseIfAutoLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian" autolimits="true" />
    <worldbody>
      <body>
        <joint type="slide"/>
        <geom size="1"/>
        <site name="s1"/>
      </body>
      <site name="s2"/>
    </worldbody>
    <tendon>
      <spatial limited="false" range="-1 1">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("limited=\"false\" range=\"-1 1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepInferredActlimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="filter" joint="hinge" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("actrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("actlimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepExplicitActlimitedIfAutoLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="filter" joint="hinge" actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("actrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("actlimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsActlimitedFalse) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="filter" joint="hinge" actlimited="false" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("actlimited=\"false\" actrange=\"-1 1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepInferredCtrllimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" ctrlrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("ctrlrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("ctrllimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepExplicitCtrllimitedIfAutoLimits) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" ctrllimited="true" ctrlrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("ctrlrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("ctrllimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsCtrllimitedFalse) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" ctrllimited="false" ctrlrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("ctrllimited=\"false\" ctrlrange=\"-1 1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepInferredForcelimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true" />
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" forcerange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("forcerange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("forcelimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesNotKeepExplicitForcelimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" forcelimited="true" forcerange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"true\""));
  EXPECT_THAT(saved_xml, HasSubstr("forcerange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("forcelimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsForcelimitedFalse) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" forcelimited="false" forcerange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("forcelimited=\"false\" forcerange=\"-1 1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, UndefinedMassDensity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="box" size=".05 .05 .05"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("density")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("mass")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesDefaults) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <geom density="100"/>
    </default>
    <worldbody>
      <body>
        <geom type="box" size=".05 .05 .05"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("mass")));
  EXPECT_THAT(saved_xml, HasSubstr("<geom density=\"100\"/>"));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesDensity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="box" size=".05 .05 .05" density="100"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("density=\"100\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("mass")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesMass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="box" size=".05 .05 .05" mass="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("density")));
  EXPECT_THAT(saved_xml, HasSubstr("mass=\"0.1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, ZeroMass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="box" size=".05 .05 .05" mass="0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("density")));
  EXPECT_THAT(saved_xml, HasSubstr("mass=\"0\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, OverwritesDensity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="0.2" density="100" mass="100"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("density")));
  EXPECT_THAT(saved_xml, HasSubstr("mass=\"100\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, UsesTwoSpaces) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("  "));
  EXPECT_THAT(saved_xml, Not(HasSubstr("    ")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesSkin) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B0_0" pos="0 0 0">
        <composite type="cloth" count="2 2 1" spacing="0.05">
          <skin texcoord="true"/>
          <geom type="ellipsoid" size="1 1 1"/>
        </composite>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjModel* mtemp = LoadModelFromString(SaveAndReadXml(model));
  EXPECT_THAT(model->nskin, 1);
  EXPECT_THAT(mtemp->nskin, 1);
  mj_deleteModel(model);
  mj_deleteModel(mtemp);
}

// check that no precision is lost when saving XMLs with FullFloatPrecision
TEST_F(XMLWriterTest, SetPrecision) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size="0.1 0.123456 0.1234567812345678"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  // save to XML and re-load, expect to lose precision
  mjModel* model_lo = LoadModelFromString(SaveAndReadXml(model));
  EXPECT_EQ(model->geom_size[1], model_lo->geom_size[1]);
  EXPECT_NE(model->geom_size[2], model_lo->geom_size[2]);
  {
    // save to XML and re-load with FullFloatPrecision, expect to maintain precision
    FullFloatPrecision increase_precision;
    mjModel* model_hi = LoadModelFromString(SaveAndReadXml(model));
    EXPECT_EQ(model->geom_size[2], model_hi->geom_size[2]);
    mj_deleteModel(model_hi);
  }
  mj_deleteModel(model_lo);
  mj_deleteModel(model);
}

class XMLWriterLocaleTest : public MujocoTest {
 protected:
  char* old_locale;
  void SetUp() override {
    this->old_locale = std::setlocale(LC_ALL, nullptr);
    if (!std::setlocale(LC_ALL, "de_DE.UTF-8")) {
      GTEST_SKIP() << "This system doesn't support the de_DE.UTF-8 locale";
    }
  }
  void TearDown() override {
    std::setlocale(LC_ALL, old_locale);
  }
};

TEST_F(XMLWriterLocaleTest, IgnoresLocale) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size="0.1 1.23 2.345"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("0.1 1.23 2.345"));
  mj_deleteModel(model);

  // Test that MuJoCo doesn't override locales for subsequent calls.
  char formatted[7];
  std::snprintf(formatted, sizeof(formatted), "%f", 3.9375);
  EXPECT_EQ(std::string(formatted), "3,9375");
}


// ------------------------ test loading and saving multiple files ---------------------------------
namespace mju = ::mujoco::util;
static constexpr int kFieldSize = 500;

// The maximum spacing between a normalised floating point number x and an
// adjacent normalised number is 2 epsilon |x|; a factor 10 is added accounting
// for losses during non-idempotent operations such as vector normalizations.
template<typename T = mjtNum> T Compare(T val1, T val2) {
  T error;
  if (mju_abs(val1) <= 1 || mju_abs(val2) <= 1) {
      // Asbolute precision for small numbers
      error = mju_abs(val1-val2);
  } else {
    // Relative precision for larger numbers
    T magnitude = mju_max(mju_abs(val1), mju_abs(val2));
    error = mju_abs(val1/magnitude - val2/magnitude) / magnitude;
  }
  return error < 2*10*std::numeric_limits<T>::epsilon() ? 0 : error;
}

mjtNum CompareModel(const mjModel* m1, const mjModel* m2, char (&field)[kFieldSize]) {
  mjtNum dif, maxdif = 0.0;

  // define symbols corresponding to number of columns (needed in MJMODEL_POINTERS)
  MJMODEL_POINTERS_PREAMBLE(m1);

  // compare ints
  #define X(name) \
    if (m1->name != m2->name) {maxdif = 1.0; mju::strcpy_arr(field, #name);}
    MJMODEL_INTS
  #undef X
  if (maxdif > 0) return maxdif;

  // compare arrays
  #define X(type, name, nr, nc)                                    \
    for (int r=0; r < m1->nr; r++)                                 \
      for (int c=0; c < nc; c++) {                                 \
        dif = Compare(m1->name[r*nc+c], m2->name[r*nc+c]);  \
        if (dif > maxdif) { maxdif = dif; mju::strcpy_arr(field, #name);} }
    MJMODEL_POINTERS
  #undef X

  // compare scalars in mjOption
  #define X(type, name)                                            \
    dif = Compare(m1->opt.name, m2->opt.name);                     \
    if (dif > maxdif) {maxdif = dif; mju::strcpy_arr(field, #name);}
    MJOPTION_SCALARS
  #undef X

  // compare arrays in mjOption
  #define X(name, n)                                             \
    for (int c=0; c < n; c++) {                                  \
      dif = Compare(m1->opt.name[c], m2->opt.name[c]);           \
      if (dif > maxdif) {maxdif = dif; mju::strcpy_arr(field, #name);} }
    MJOPTION_VECTORS
  #undef X

  // Return largest difference and field name
  return maxdif;
}

TEST_F(XMLWriterTest, WriteReadCompare) {
  FullFloatPrecision increase_precision;
  // Loop over all xml files in data
  std::vector<std::string> paths = {GetTestDataFilePath("."), GetModelPath(".")};
  std::string ext(".xml");
  for (auto const& path : paths) {
    for (auto &p : std::filesystem::recursive_directory_iterator(path)) {
      if (p.path().extension() == ext) {
        std::string xml = p.path().string();

        // if file is meant to fail, skip it
        if (absl::StrContains(p.path().string(), "malformed_")) {
          continue;
        }

        // load model
        std::array<char, 1000> error;
        mjModel* m = mj_loadXML(xml.c_str(), nullptr, error.data(), error.size());
        ASSERT_THAT(m, NotNull()) << "Failed to load " << xml.c_str() << ": " << error.data();

        // make data
        mjData* d = mj_makeData(m);
        ASSERT_THAT(d, testing::NotNull()) << "Failed to create data" << std::endl;

        // save and load back
        mjModel* mtemp = LoadModelFromString(SaveAndReadXml(m), error.data(), error.size());

        if (!mtemp) {
          // if failing because assets are missing, accept the test
          ASSERT_THAT(error.data(), HasSubstr("file")) << error.data();
        } else {
          // compare and delete
        char field[kFieldSize] = "";
        mjtNum result = CompareModel(m, mtemp, field);
        EXPECT_LE(result, 0) << "Loaded and saved models are different!" << std::endl
                             << "Affected file " << p.path().string() << std::endl
                             << "Different field: " << field << std::endl;
          mj_deleteModel(mtemp);
        }

        // delete original structures
        mj_deleteData(d);
        mj_deleteModel(m);
      }
    }
  }
}

}  // namespace
}  // namespace mujoco
