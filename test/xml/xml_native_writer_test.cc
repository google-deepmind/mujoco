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

#include <memory>
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>
#endif

#include <array>
#include <clocale>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "src/xml/xml_numeric_format.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::Not;
using ::testing::NotNull;
using ::testing::FloatEq;

using XMLWriterTest = PluginTest;

TEST_F(XMLWriterTest, EmptyModel) {
  static constexpr char xml[] = "<mujoco/>";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("default")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, SavesMemory) {
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory=" 1023 "/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml);
    ASSERT_THAT(model, NotNull());
    std::string saved_xml = SaveAndReadXml(model);
    EXPECT_THAT(saved_xml, HasSubstr("memory=\"1023\""));
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1024"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml);
    ASSERT_THAT(model, NotNull());
    std::string saved_xml = SaveAndReadXml(model);
    EXPECT_THAT(saved_xml, HasSubstr("memory=\"1K\""));
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="4096"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml);
    ASSERT_THAT(model, NotNull());
    std::string saved_xml = SaveAndReadXml(model);
    EXPECT_THAT(saved_xml, HasSubstr("memory=\"4K\""));
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1048576"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml);
    ASSERT_THAT(model, NotNull());
    std::string saved_xml = SaveAndReadXml(model);
    EXPECT_THAT(saved_xml, HasSubstr("memory=\"1M\""));
    mj_deleteModel(model);
  }
  {
    static constexpr char xml[] = R"(
    <mujoco>
      <size memory="1047552"/>
    </mujoco>
    )";
    mjModel* model = LoadModelFromString(xml);
    ASSERT_THAT(model, NotNull());
    std::string saved_xml = SaveAndReadXml(model);
    EXPECT_THAT(saved_xml, HasSubstr("memory=\"1023K\""));
    mj_deleteModel(model);
  }
}

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

TEST_F(XMLWriterTest, EmptyUserSensor) {
  static constexpr char xml[] = R"(
  <mujoco>
    <sensor>
      <user dim="2" needstage="vel"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("dim=\"2\""));
  EXPECT_THAT(saved_xml, HasSubstr("needstage=\"vel\""));
  EXPECT_THAT(saved_xml, HasSubstr("datatype=\"real\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("objtype")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("objname")));
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("inertial")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsBoundMassInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler boundmass="0.1" boundinertia="0.2"/>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("boundmass=\"0.1\""));
  EXPECT_THAT(saved_xml, HasSubstr("boundinertia=\"0.2\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DropsZeroBoundMassInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler boundmass="0" boundinertia="0"/>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("boundmass")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("boundinertia")));
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("inertial")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsAutoLimitsFalse) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("autolimits=\"false\""));
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("limited=\"true\"")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsExplicitJointLimited) {
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, HasSubstr("limited=\"true\""));
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("limited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsExplicitTendonLimitedIfAutoLimits) {
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1 1\""));
  EXPECT_THAT(saved_xml, HasSubstr("limited=\"true\""));
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("actrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("actlimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsExplicitActlimitedIfAutoLimits) {
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("actrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, HasSubstr("actlimited=\"true\""));
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("ctrlrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("ctrllimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsExplicitCtrllimitedIfAutoLimits) {
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("ctrlrange=\"-1 1\""));
  EXPECT_THAT(saved_xml, HasSubstr("ctrllimited=\"true\""));
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("forcerange=\"-1 1\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("forcelimited=\"true\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, KeepsExplicitForcelimited) {
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("autolimits=\"true\"")));
  EXPECT_THAT(saved_xml, HasSubstr("forcerange=\"-1 1\""));
  EXPECT_THAT(saved_xml, HasSubstr("forcelimited=\"true\""));
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml,
              HasSubstr("forcelimited=\"false\" forcerange=\"-1 1\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesGravComp) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body gravcomp=".25"/>
      <body/>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("gravcomp=\"0.25\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("gravcomp=\"0\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesBodyPos) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="1 2 3"/>
      <body pos="0 0 0"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("pos=\"1 2 3\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("pos=\"0 0 0\"")));
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("mass")));
  EXPECT_THAT(saved_xml, HasSubstr("<geom density=\"100\"/>"));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesActuatorDefaults) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="position">
        <position kp="3" kv="4" />
      </default>
      <default class="intvelocity">
        <intvelocity kp="5" kv="6" />
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
      <intvelocity joint="jnt" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("mass")));
  EXPECT_THAT(saved_xml, HasSubstr(
      "<general biastype=\"affine\" gainprm=\"3\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesFrameDefaults) {
  static constexpr char xml[] = R"(
  <mujoco model="test">
    <default>
      <default class="dframe">
        <geom size=".1"/>
      </default>
    </default>

    <worldbody>
      <frame name="f1" euler="0 0 30">
        <geom size=".5" euler="0 0 20"/>
      </frame>

      <body name="body">
        <frame pos="0 1 0" name="f2" childclass="dframe">
          <geom pos="0 1 0"/>
          <frame pos="0 1 0" name="f3">
            <frame pos="0 1 0" name="f4">
              <body pos="1 0 0">
                <geom pos="0 0 1"/>
              </body>
            </frame>
          </frame>
        </frame>
        <frame>
          <light pos="0 0 1"/>
        </frame>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char xml_expected[] = R"(<mujoco model="test">
  <compiler angle="radian"/>

  <default>
    <default class="dframe">
      <geom size="0.1 0 0"/>
    </default>
  </default>

  <worldbody>
    <body name="body">
      <frame name="f2" childclass="dframe">
        <geom pos="0 2 0"/>
        <frame name="f3" childclass="dframe">
          <frame name="f4" childclass="dframe">
            <body pos="1 3 0">
              <geom pos="0 0 1"/>
            </body>
          </frame>
        </frame>
      </frame>
      <frame>
        <light pos="0 0 1" dir="0 0 -1"/>
      </frame>
    </body>
    <frame name="f1">
      <geom size="0.5" quat="0.906308 0 0 0.422618"/>
    </frame>
  </worldbody>
</mujoco>
)";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_STREQ(saved_xml.c_str(), xml_expected);
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("density")));
  EXPECT_THAT(saved_xml, HasSubstr("mass=\"100\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, SaveDefaultMass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="2 0 3  0 1 3  1 2 3  0 2 1" />
    </asset>
    <default class="main">
      <geom type="mesh" mass="1"/>
    </default>
    <worldbody>
      <body>
        <geom mesh="example" size=".1 .2 .3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  EXPECT_THAT(model, NotNull());
  std::string content = SaveAndReadXml(model);
  EXPECT_THAT(content, HasSubstr("mass=\"1\""));
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("  "));
  EXPECT_THAT(saved_xml, Not(HasSubstr("    ")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesSkin) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <composite type="grid" count="2 2 1" spacing="0.05">
        <skin texcoord="true"/>
        <geom size=".01"/>
      </composite>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  EXPECT_THAT(model->nskin, 1);

  mjModel* mtemp = LoadModelFromString(SaveAndReadXml(model));
  ASSERT_THAT(mtemp, NotNull());
  EXPECT_THAT(mtemp->nskin, 1);

  mj_deleteModel(model);
  mj_deleteModel(mtemp);
}

TEST_F(XMLWriterTest, WritesHfield) {
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
  // load model
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  int size = model->hfield_nrow[0]*model->hfield_ncol[0];
  EXPECT_EQ(size, 6);

  // save and read, compare data
  mjModel* mtemp = LoadModelFromString(SaveAndReadXml(model));
  ASSERT_THAT(mtemp, NotNull());
  for (int i = 0; i < size; ++i) {
    EXPECT_THAT(mtemp->hfield_data[i], FloatEq(model->hfield_data[i]));
  }
  mj_deleteModel(mtemp);

  // modify data, save read and compare
  model->hfield_data[0] = 0.25;
  model->hfield_data[1] = 0.0;
  model->hfield_data[2] = 0.5;
  model->hfield_data[3] = 0.0;
  model->hfield_data[4] = 1.0;
  model->hfield_data[5] = 0.0;

  mtemp = LoadModelFromString(SaveAndReadXml(model));
  ASSERT_THAT(mtemp, NotNull());
  for (int i = 0; i < size; ++i) {
    EXPECT_THAT(mtemp->hfield_data[i], FloatEq(model->hfield_data[i]));
  }
  mj_deleteModel(mtemp);

  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesLight) {
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());

  // save and read, compare data
  mjModel* mtemp = LoadModelFromString(SaveAndReadXml(model));
  EXPECT_EQ(mtemp->nlight, 3);
  EXPECT_FLOAT_EQ(mtemp->light_bulbradius[0], 0.02);
  EXPECT_FLOAT_EQ(mtemp->light_bulbradius[1], 1);
  EXPECT_FLOAT_EQ(mtemp->light_bulbradius[2], 2);

  mj_deleteModel(mtemp);
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, WritesMaterial) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="mat">
        <material metallic="2" roughness="3"/>
      </default>
    </default>
    <asset>
      <material name="0" class="mat"/>
      <material name="1" class="mat" metallic="4" roughness="5"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  // save and read, compare data
  mjModel* mtemp = LoadModelFromString(SaveAndReadXml(model));
  EXPECT_EQ(mtemp->nmat, 2);
  EXPECT_EQ(mtemp->mat_metallic[0], 2);
  EXPECT_EQ(mtemp->mat_metallic[1], 4);
  EXPECT_EQ(mtemp->mat_roughness[0], 3);
  EXPECT_EQ(mtemp->mat_roughness[1], 5);

  mj_deleteModel(mtemp);
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, SpringlengthOneValue) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="1"/>
      <site name="2" pos="0 0 1"/>
    </worldbody>

    <tendon>
      <spatial springlength="0.5">
        <site site="1"/>
        <site site="2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("springlength=\"0.5\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, SpringlengthTwoValues) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="1"/>
      <site name="2" pos="0 0 1"/>
    </worldbody>

    <tendon>
      <spatial springlength="0 0.5">
        <site site="1"/>
        <site site="2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("springlength=\"0 0.5\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, Actdim) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="user" actdim="2"/>
      <general joint="hinge" dyntype="filter" dynprm="1"/>
      <motor joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("actdim=\"2\""));
  EXPECT_THAT(saved_xml, Not(HasSubstr("actdim=\"1\"")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("actdim=\"0\"")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, ActdimDefaults) {
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("actdim=\"2\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, TrimsDefaults) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" friction="0.9" solref="0.1" solimp="0.1 0.2"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="0 2"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" kp="3" kv="4"/>
      <damper joint="jnt" kv="10" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("friction=\"0.9\""));
  EXPECT_THAT(saved_xml, HasSubstr("solref=\"0.1\""));
  EXPECT_THAT(saved_xml, HasSubstr("solimp=\"0.1 0.2\""));
  EXPECT_THAT(saved_xml, HasSubstr("gainprm=\"3\" biasprm=\"0 -3 -4\""));
  EXPECT_THAT(saved_xml, HasSubstr("gainprm=\"0 0 -10\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, DoesntSaveGlobal) {
  static constexpr char xml[] = "<mujoco/>";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("global")));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, InheritrangeSavedAsRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="slide" type="slide" range="0 2"/>
      </body>
      <body>
        <geom size="1"/>
        <joint name="hinge" type="hinge" range="-2 0"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="slide" inheritrange="2"/>
      <intvelocity joint="hinge" inheritrange="0.5"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("ctrlrange=\"-1 3\""));
  EXPECT_THAT(saved_xml, HasSubstr("actrange=\"-1.5 -0.5\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, InheritedInheritrangeSavedAsRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <default class="position">
        <position kp="3" kv="4" inheritrange="2"/>
      </default>
      <default class="intvelocity">
        <intvelocity kp="5" kv="6" inheritrange="0.5"/>
      </default>
    </default>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" range="0 2"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt" class="position"/>
      <intvelocity joint="jnt" class="intvelocity"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("inheritrange")));
  EXPECT_THAT(saved_xml, HasSubstr("ctrlrange=\"-1 3\""));
  EXPECT_THAT(saved_xml, HasSubstr("actrange=\"0.5 1.5\""));
  mj_deleteModel(model);
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
  ASSERT_THAT(model, NotNull());

  // save to XML and re-load, expect to lose precision
  mjModel* model_lo = LoadModelFromString(SaveAndReadXml(model));
  ASSERT_THAT(model_lo, NotNull());

  EXPECT_EQ(model->geom_size[1], model_lo->geom_size[1]);
  EXPECT_NE(model->geom_size[2], model_lo->geom_size[2]);
  {
    // save to XML and reload with FullFloatPrecision
    // expect to maintain precision
    FullFloatPrecision increase_precision;
    mjModel* model_hi = LoadModelFromString(SaveAndReadXml(model));
    EXPECT_EQ(model->geom_size[2], model_hi->geom_size[2]);
    mj_deleteModel(model_hi);
  }
  mj_deleteModel(model_lo);
  mj_deleteModel(model);
}

class XMLWriterLocaleTest : public MujocoTest {
 public:
  XMLWriterLocaleTest() : old_locale_(std::setlocale(LC_ALL, nullptr)) {}

 protected:
  void SetUp() override {
    if (!std::setlocale(LC_ALL, "de_DE.UTF-8")) {
      GTEST_SKIP() << "This system doesn't support the de_DE.UTF-8 locale";
    }
  }

  void TearDown() override {
    std::setlocale(LC_ALL, old_locale_.c_str());
  }

 private:
  std::string old_locale_;
};

TEST_F(XMLWriterLocaleTest, IgnoresLocale) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size="0.1 1.23 2.345"/>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("0.1 1.23 2.345"));
  mj_deleteModel(model);

  // Test that MuJoCo doesn't override locales for subsequent calls.
  char formatted[8];
  std::snprintf(formatted, sizeof(formatted), "%.4f", 3.9375);
  EXPECT_EQ(std::string(formatted), "3,9375");
}


// ------------------- test loading and saving multiple files ------------------
TEST_F(XMLWriterTest, WriteReadCompare) {
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
        if (absl::StrContains(p.path().string(), "100_humanoids") ||
            absl::StrContains(p.path().string(), "malformed_") ||
            absl::StrContains(p.path().string(), "touch_grid") ||
            absl::StrContains(p.path().string(), "gmsh_") ||
            absl::StrContains(p.path().string(), "shark_") ||
            absl::StrContains(p.path().string(), "cow") ||
            absl::StrContains(p.path().string(), "frameless_contact_hfield") ||
            absl::StrContains(p.path().string(), "spheremesh")) {
          continue;
        }

        // load model
        std::array<char, 1000> error;
        mjModel* m = mj_loadXML(
            xml.c_str(), nullptr, error.data(), error.size());
        ASSERT_THAT(m, NotNull())
            << "Failed to load " << xml.c_str() << ": " << error.data();

        // make data
        mjData* d = mj_makeData(m);
        ASSERT_THAT(d, testing::NotNull()) << "Failed to create data\n";

        // save and load back
        mjModel* mtemp =
            LoadModelFromString(SaveAndReadXml(m), error.data(), error.size());

        if (!mtemp) {
          // if failing because assets are missing, accept the test
          ASSERT_THAT(error.data(), HasSubstr("file"))
              << error.data() << " from " << xml.c_str();
        } else {
          mjtNum tol = 0;

          // for particularly sensitive models, relax the tolerance
          if (absl::StrContains(p.path().string(), "belt.xml") ||
              absl::StrContains(p.path().string(), "cable.xml")) {
            tol = 1e-13;
          }

          // compare and delete
          std::string field = "";
          mjtNum result = CompareModel(m, mtemp, field);
          EXPECT_LE(result, tol)
              << "Loaded and saved models are different!\n"
              << "Affected file " << p.path().string() << '\n'
              << "Different field: " << field << '\n';
          mj_deleteModel(mtemp);
        }

        // check for stack memory leak
        mj_step(m, d);
        EXPECT_EQ(d->pstack, 0) << "mjData stack memory leak detected in " <<
            p.path().string() << '\n';

        // delete data
        mj_deleteData(d);

        // allocate buffer, save m into it
        size_t sz = mj_sizeModel(m);
        void* buffer = mju_malloc(sz);
        mj_saveModel(m, nullptr, buffer, sz);

        // make new VFS add buffer to it
        mjVFS* vfs = (mjVFS*)mju_malloc(sizeof(mjVFS));
        mj_defaultVFS(vfs);
        int failed = mj_addBufferVFS(vfs, "model.mjb", buffer, sz);
        EXPECT_EQ(failed, 0) << "Failed to add buffer to VFS";

        // load model from VFS
        mtemp = mj_loadModel("model.mjb", vfs);
        ASSERT_THAT(mtemp, NotNull());

        // compare with 0 tolerance
        std::string field = "";
        mjtNum result = CompareModel(m, mtemp, field);
        EXPECT_EQ(result, 0)
            << "Loaded and saved binary models are different!\n"
            << "Affected file " << p.path().string() << '\n'
            << "Different field: " << field << '\n';

        // clean up
        mj_deleteModel(mtemp);
        mj_deleteVFS(vfs);
        mju_free(vfs);
        mju_free(buffer);

        // delete model
        mj_deleteModel(m);
      }
    }
  }
}

// ---------------- test CopyBack functionality (decompiler) ------------------
using DecompilerTest = MujocoTest;
TEST_F(DecompilerTest, SavesStatitics) {
  static constexpr char xml[] = R"(
  <mujoco>
    <statistic meansize="2" extent="3" center="4 5 6" meanmass="7" meaninertia="8"/>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  model->stat.meansize = 7;
  model->stat.extent = 8;
  model->stat.center[0] = 9;
  model->stat.center[1] = 10;
  model->stat.center[2] = 11;
  model->stat.meanmass = 12;
  model->stat.meaninertia = 13;
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, HasSubstr("meansize=\"7\""));
  EXPECT_THAT(saved_xml, HasSubstr("extent=\"8\""));
  EXPECT_THAT(saved_xml, HasSubstr("center=\"9 10 11\""));
  EXPECT_THAT(saved_xml, HasSubstr("meanmass=\"12\""));
  EXPECT_THAT(saved_xml, HasSubstr("meaninertia=\"13\""));
  mj_deleteModel(model);
}

TEST_F(DecompilerTest, DoesntSaveInferredStatitics) {
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
  ASSERT_THAT(model, NotNull());
  std::string saved_xml = SaveAndReadXml(model);
  EXPECT_THAT(saved_xml, Not(HasSubstr("meansize")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("meanmass")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("meaninertia")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("center")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("extent")));
  EXPECT_THAT(saved_xml, Not(HasSubstr("statistic")));
  mj_deleteModel(model);
}

TEST_F(DecompilerTest, VeryLargeNumbers) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler angle="radian"/>
    <worldbody>
      <camera focal="16777217 1" sensorsize="1 1" resolution="100 100"/>
      <body pos="1e+20 0 0">
        <geom size="1"/>
        <joint axis="1 0 0" range="-1e+10 1e+10"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  std::string saved_xml = SaveAndReadXml(model);
  // note, focal is float and loses precision 16777217 -> 16777216
  EXPECT_THAT(saved_xml, HasSubstr("focal=\"16777216 1\""));
  EXPECT_THAT(saved_xml, HasSubstr("pos=\"1e+20 0 0\""));
  EXPECT_THAT(saved_xml, HasSubstr("range=\"-1e+10 1e+10\""));
  mj_deleteModel(model);
}

TEST_F(XMLWriterTest, ExpandAttach) {
  static constexpr char xml_parent[] = R"(
  <mujoco>
    <asset>
      <model name="b" file="b.xml" />
    </asset>

    <worldbody>
      <attach model="b" body="b" prefix="b" />
    </worldbody>
  </mujoco>
  )";

  static constexpr char xml_child[] = R"(
  <mujoco>
    <default>
      <default class="b"/>
    </default>

    <worldbody>
      <body name="b">
        <geom type="box" size="0.1 0.1 0.1"/>
        <joint name="b"/>
      </body>
    </worldbody>

    <actuator>
      <position joint="b" class="b"/>
    </actuator>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "b.xml", xml_child, sizeof(xml_child));

  std::array<char, 1024> er;
  mjModel* m = LoadModelFromString(xml_parent, er.data(), er.size(), vfs.get());
  ASSERT_THAT(m, NotNull()) << er.data();

  std::string saved_xml = SaveAndReadXml(m);
  EXPECT_THAT(saved_xml, HasSubstr("class=\"bb\""));
  mj_deleteModel(m);
  mj_deleteVFS(vfs.get());
}

}  // namespace
}  // namespace mujoco
