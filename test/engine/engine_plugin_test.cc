// Copyright 2022 DeepMind Technologies Limited
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

// Tests for plugin-related functionalities.
// TODO(b/247110452) add more comments to this file, or add sample plugins

#include "src/engine/engine_plugin.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {
using ::testing::HasSubstr;
using ::testing::NotNull;

constexpr int kNumFakePlugins = 30;
constexpr int kNumTestPlugins = 3;
const int kNumTruePlugins = mjp_pluginCount();

class BaseTestPlugin {
 public:
  static constexpr int kDefaultStride = 1;

  BaseTestPlugin(const mjModel* m, mjData* d, int instance)
      : reset_count(d->plugin_state[m->plugin_stateadr[instance]]),
        compute_count(d->plugin_state[m->plugin_stateadr[instance] + 1]),
        advance_count(d->plugin_state[m->plugin_stateadr[instance] + 2]) {
    {
      const char* s = mj_getPluginConfig(m, instance, "stride");
      if (*s) {
        std::stringstream(s) >> stride;
      } else {
        stride = kDefaultStride;
      }
    }

    reset_count = 0;
    compute_count = 0;
    advance_count = 0;
  }

  void Reset() {
    reset_count += stride;
    compute_count = 0;
    advance_count = 0;
  }

  void Compute() {
    compute_count += stride;
  }

  void Advance() {
    advance_count += stride;
  }

 protected:
  int stride;

  mjtNum& reset_count;
  mjtNum& compute_count;
  mjtNum& advance_count;
};

class TestSensor : public BaseTestPlugin {
 public:
  TestSensor(const mjModel* m, mjData* d, int instance)
      : BaseTestPlugin(m, d, instance) {
    for (int i = 0; i < m->nsensor; ++i) {
      if (m->sensor_type[i] == mjSENS_PLUGIN &&
          m->sensor_plugin[i] == instance) {
        sensors.push_back(
            reinterpret_cast<mjtNum(*)[4]>(&d->sensordata[m->sensor_adr[i]]));
      }
    }
  }

  static int& InitCount() {
    static int counter = 0;
    return counter;
  }

  static int& DestroyCount() {
    static int counter = 0;
    return counter;
  }

  void Reset() {
    BaseTestPlugin::Reset();
    WriteSensorData();
  }

  void Compute() {
    BaseTestPlugin::Compute();
    WriteSensorData();
  }

  void Advance() {
    BaseTestPlugin::Advance();
    WriteSensorData();
  }

 private:
  std::vector<mjtNum (*)[4]> sensors;

  void WriteSensorData() {
    for (auto* sensordata_ptr : sensors) {
      auto& sensordata = *sensordata_ptr;
      sensordata[0] = reset_count;
      sensordata[1] = compute_count;
      sensordata[2] = advance_count;
    }
  }
};

class TestActuator : public BaseTestPlugin {
 public:
  static constexpr mjtNum kDefaultMultiplier = 1.0;

  TestActuator(const mjModel* m, mjData* d, int instance)
      : BaseTestPlugin(m, d, instance) {
    const char* s = mj_getPluginConfig(m, instance, "multiplier");
    if (*s) {
        std::stringstream(s) >> multiplier;
      } else {
        multiplier = kDefaultMultiplier;
      }
    for (int i = 0; i < m->nu; ++i) {
      if (m->actuator_plugin[i] == instance) {
        actuators.push_back(&d->actuator_force[i]);
      }
    }
  }

  static int& InitCount() {
    static int counter = 0;
    return counter;
  }

  static int& DestroyCount() {
    static int counter = 0;
    return counter;
  }

  void Reset() {
    BaseTestPlugin::Reset();
    WriteActuatorForce();
  }

  void Compute() {
    BaseTestPlugin::Compute();
    WriteActuatorForce();
  }

  void Advance() {
    BaseTestPlugin::Advance();
    WriteActuatorForce();
  }

 private:
  mjtNum multiplier;
  std::vector<mjtNum*> actuators;

  void WriteActuatorForce() {
    for (mjtNum* actuator_force : actuators) {
      *actuator_force = advance_count * multiplier;
    }
  }
};

class TestPassive {
 public:
  TestPassive(const mjModel* m, mjData* d, int instance) {}
  void Reset() {}
  void Compute() {}
  void Advance() {}
};

int RegisterSensorPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.test.sensor";

  const char* attributes[] = {"stride"};
  plugin.nattribute = sizeof(attributes) / sizeof(*attributes);
  plugin.attributes = attributes;

  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  plugin.nstate = +[](const mjModel* m, int instance) { return 3; };
  plugin.nsensordata =
      +[](const mjModel* m, int instance, int sensor_id) { return 3; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* sensor = new TestSensor(m, d, instance);
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(sensor);
    TestSensor::InitCount()++;
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<TestSensor*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
    TestSensor::DestroyCount()++;
  };

  plugin.reset = +[](const mjModel* m, double* plugin_state, void* plugin_data,
                     int instance) {
    auto sensor = reinterpret_cast<TestSensor*>(plugin_data);
    sensor->Reset();
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    auto sensor = reinterpret_cast<TestSensor*>(d->plugin_data[instance]);
    sensor->Compute();
  };
  plugin.advance = +[](const mjModel* m, mjData* d, int instance) {
    auto sensor = reinterpret_cast<TestSensor*>(d->plugin_data[instance]);
    sensor->Advance();
  };

  return mjp_registerPlugin(&plugin);
}

int RegisterActuatorPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.test.actuator";

  const char* attributes[] = {"stride", "multiplier"};
  plugin.nattribute = sizeof(attributes) / sizeof(*attributes);
  plugin.attributes = attributes;

  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;

  plugin.nstate = +[](const mjModel* m, int instance) { return 3; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* actuator = new TestActuator(m, d, instance);
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(actuator);
    TestActuator::InitCount()++;
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<TestActuator*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
    TestActuator::DestroyCount()++;
  };

  plugin.reset = +[](const mjModel* m, double* plugin_state, void* plugin_data,
                     int instance) {
    auto actuator = reinterpret_cast<TestActuator*>(plugin_data);
    actuator->Reset();
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    auto actuator = reinterpret_cast<TestActuator*>(d->plugin_data[instance]);
    actuator->Compute();
  };
  plugin.advance = +[](const mjModel* m, mjData* d, int instance) {
    auto actuator = reinterpret_cast<TestActuator*>(d->plugin_data[instance]);
    actuator->Advance();
  };

  return mjp_registerPlugin(&plugin);
}

int RegisterPassivePlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.test.passive";

  const char* attributes[] = {"attribute"};
  plugin.nattribute = sizeof(attributes) / sizeof(*attributes);
  plugin.attributes = attributes;

  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* passive = new TestPassive(m, d, instance);
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(passive);
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<TestPassive*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel* m, double* plugin_state, void* plugin_data,
                     int instance) {
    auto passive = reinterpret_cast<TestPassive*>(plugin_data);
    passive->Reset();
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    auto passive = reinterpret_cast<TestPassive*>(d->plugin_data[instance]);
    passive->Compute();
  };

  return mjp_registerPlugin(&plugin);
}

class PluginTest : public MujocoTest {
 public:
  // register all plugins
  PluginTest() : MujocoTest() {
    RegisterSensorPlugin();

    for (int i = 1; i <= kNumFakePlugins; ++i) {
      mjpPlugin plugin;
      mjp_defaultPlugin(&plugin);
      std::string name = absl::StrFormat("mujoco.test.fake%u", i);
      plugin.name = name.c_str();
      mjp_registerPlugin(&plugin);
    }

    RegisterActuatorPlugin();
    RegisterPassivePlugin();
  }
};

constexpr char xml[] = R"(
<mujoco>
  <extension>
    <plugin plugin="mujoco.test.sensor">
      <instance name="twosensors"/>
      <instance name="threesensors">
        <config key="stride" value="3"/>
      </instance>
    </plugin>
    <plugin plugin="mujoco.test.actuator">
      <instance name="actuator2">
        <config key="stride" value="2"/>
        <config key="multiplier" value="0.125"/>
      </instance>
    </plugin>
    <plugin plugin="mujoco.test.passive"/>
  </extension>
  <worldbody>
    <body>
      <plugin plugin="mujoco.test.passive">
        <config key="attribute" value="0"/>
      </plugin>
      <geom type="capsule" size="0.1" fromto="-1 0 0 -1 0 -1"/>
      <joint name="h1" type="hinge"/>
    </body>
    <body>
      <geom type="capsule" size="0.1" fromto="1 0 0 1 0 -1"/>
      <joint name="h2" type="hinge"/>
    </body>
  </worldbody>
  <sensor>
    <plugin instance="twosensors"/>
    <plugin plugin="mujoco.test.sensor">
      <config key="stride" value="5"/>
    </plugin>
    <plugin instance="threesensors"/>
    <plugin instance="twosensors"/>
    <plugin instance="threesensors"/>
    <plugin instance="threesensors"/>
  </sensor>
  <actuator>
    <plugin joint="h1" plugin="mujoco.test.actuator">
      <config key="stride" value="4"/>
      <config key="multiplier" value="0.03125"/>
    </plugin>
    <plugin joint="h2" instance="actuator2"/>
  </actuator>
</mujoco>
)";

TEST_F(PluginTest, MultiplePluginTableBlocks) {
  EXPECT_EQ(mjp_pluginCount(), kNumTruePlugins + kNumFakePlugins + kNumTestPlugins);

  const mjpPlugin* last_plugin = nullptr;
  int table_count = 0;
  for (int i = 1; i <= kNumFakePlugins; ++i) {
    int slot;
    std::string name = absl::StrFormat("mujoco.test.fake%u", i);
    const mjpPlugin* plugin = mjp_getPlugin(name.c_str(), &slot);
    EXPECT_EQ(slot, kNumTruePlugins+i);
    EXPECT_THAT(plugin, NotNull());
    if (plugin - last_plugin != 1) {
      ++table_count;
    }
    last_plugin = plugin;
  }

  // Make sure that there are enough fake plugins to fill multiple table blocks.
  EXPECT_GT(table_count, 1);

  // Make sure that a block contains multiple plugins.
  EXPECT_LT(table_count, kNumFakePlugins);
}

TEST_F(PluginTest, RegisterIdenticalPlugin) {
  EXPECT_EQ(RegisterSensorPlugin(), kNumTruePlugins);
  EXPECT_EQ(RegisterActuatorPlugin(), kNumTruePlugins + kNumFakePlugins + 1);
  EXPECT_EQ(RegisterPassivePlugin(), kNumTruePlugins + kNumFakePlugins + 2);
  EXPECT_EQ(mjp_pluginCount(), kNumTruePlugins + kNumFakePlugins + kNumTestPlugins);
}

TEST_F(PluginTest, SaveXml) {
  char error[1024] = {0};

  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;

  std::string saved_xml = SaveAndReadXml(m);
  std::string_view expected_xml(xml);

  const std::string extension_open = "<extension>";
  const std::string extension_close = "</extension>";
  int extension_start = expected_xml.find(extension_open);
  ASSERT_NE(extension_start, std::string::npos);
  int extension_end =
      expected_xml.find(extension_close) + extension_close.size();
  ASSERT_NE(extension_end, std::string::npos);
  ASSERT_LE(extension_start, extension_end);
  auto expected_extension_section =
      expected_xml.substr(extension_start, extension_end - extension_start);

  const std::string sensor_open = "<sensor>";
  const std::string sensor_close = "</sensor>";
  int sensor_start = expected_xml.find(sensor_open);
  ASSERT_NE(sensor_start, std::string::npos);
  int sensor_end = expected_xml.find(sensor_close) + sensor_close.size();
  ASSERT_NE(sensor_end, std::string::npos);
  ASSERT_LE(sensor_start, sensor_end);
  auto expected_sensor_section =
      expected_xml.substr(sensor_start, sensor_end - sensor_start);

  const std::string actuator_open = "<actuator>";
  const std::string actuator_close = "</actuator>";
  int actuator_start = expected_xml.find(actuator_open);
  ASSERT_NE(actuator_start, std::string::npos);
  int actuator_end = expected_xml.find(actuator_close) + actuator_close.size();
  ASSERT_NE(actuator_end, std::string::npos);
  ASSERT_LE(actuator_start, actuator_end);
  auto expected_actuator_section =
      expected_xml.substr(actuator_start, actuator_end - actuator_start);

  EXPECT_THAT(saved_xml, HasSubstr(expected_extension_section));
  EXPECT_THAT(saved_xml, HasSubstr(expected_sensor_section));
  EXPECT_THAT(saved_xml, HasSubstr(expected_actuator_section));

  mj_deleteModel(m);

  // make sure that the saved XML can still be compiled
  mjModel* m2 = LoadModelFromString(saved_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mj_deleteModel(m2);
}

TEST_F(PluginTest, SensorPlugin) {
  int expected_init_count = TestSensor::InitCount();
  int expected_destroy_count = TestSensor::DestroyCount();
  EXPECT_EQ(expected_init_count, expected_destroy_count);

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;

  // mj_makeModel calls mj_makeData and mj_deleteData internally
  expected_init_count += 3;
  expected_destroy_count += 3;
  EXPECT_EQ(TestSensor::InitCount(), expected_init_count);
  EXPECT_EQ(TestSensor::DestroyCount(), expected_destroy_count);

  EXPECT_EQ(m->nplugin, 6);
  EXPECT_EQ(mj_name2id(m, mjOBJ_PLUGIN, "twosensors"), 0);
  EXPECT_EQ(mj_name2id(m, mjOBJ_PLUGIN, "threesensors"), 1);

  mjData* d = mj_makeData(m);
  expected_init_count += 3;
  EXPECT_EQ(TestSensor::InitCount(), expected_init_count);
  EXPECT_EQ(TestSensor::DestroyCount(), expected_destroy_count);

  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[3]>(d->plugin_state +
                                                  m->plugin_stateadr[0]),
                  testing::ElementsAreArray<int>({i+1, 2*j, j}));
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[3]>(d->plugin_state +
                                                  m->plugin_stateadr[1]),
                  testing::ElementsAreArray<int>({3*(i+1), 6*j, 3*j}));
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[3]>(d->plugin_state +
                                                  m->plugin_stateadr[4]),
                  testing::ElementsAreArray<int>({4*(i+1), 8*j, 4*j}));
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[18]>(d->sensordata),
                  testing::ElementsAreArray<int>({   i+1,   2*j,   j,
                                                  5*(i+1), 10*j, 5*j,
                                                  3*(i+1),  6*j, 3*j,
                                                     i+1,   2*j,   j,
                                                  3*(i+1),  6*j, 3*j,
                                                  3*(i+1),  6*j, 3*j}));
      mj_step(m, d);
      mj_forward(m, d);
    }
    mj_resetData(m, d);
  }

  mj_deleteData(d);
  expected_destroy_count += 3;
  EXPECT_EQ(TestSensor::InitCount(), expected_init_count);
  EXPECT_EQ(TestSensor::DestroyCount(), expected_destroy_count);

  mj_deleteModel(m);
  EXPECT_EQ(TestSensor::InitCount(), expected_init_count);
  EXPECT_EQ(TestSensor::DestroyCount(), expected_destroy_count);
}

TEST_F(PluginTest, ActuatorPlugin) {
  int expected_init_count = TestActuator::InitCount();
  int expected_destroy_count = TestActuator::DestroyCount();
  EXPECT_EQ(expected_init_count, expected_destroy_count);

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;

  // mj_makeModel calls mj_makeData and mj_deleteData internally
  expected_init_count += 2;
  expected_destroy_count += 2;
  EXPECT_EQ(TestActuator::InitCount(), expected_init_count);
  EXPECT_EQ(TestActuator::DestroyCount(), expected_destroy_count);

  EXPECT_EQ(m->nplugin, 6);
  EXPECT_EQ(mj_name2id(m, mjOBJ_PLUGIN, "actuator2"), 2);

  mjData* d = mj_makeData(m);
  expected_init_count += 2;
  EXPECT_EQ(TestActuator::InitCount(), expected_init_count);
  EXPECT_EQ(TestActuator::DestroyCount(), expected_destroy_count);

  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 10; ++j) {
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[3]>(d->plugin_state +
                                                  m->plugin_stateadr[2]),
                  testing::ElementsAreArray<int>({2*(i+1), 4*j, 2*j}));
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[3]>(d->plugin_state +
                                                  m->plugin_stateadr[3]),
                  testing::ElementsAreArray<int>({4*(i+1), 8*j, 4*j}));
      EXPECT_THAT(*reinterpret_cast<mjtNum(*)[2]>(d->actuator_force),
                  testing::ElementsAreArray<mjtNum>({0.125*j, 0.25*j}));
      mj_step(m, d);
      mj_forward(m, d);
    }
    mj_resetData(m, d);
  }

  mj_deleteData(d);
  expected_destroy_count += 2;
  EXPECT_EQ(TestActuator::InitCount(), expected_init_count);
  EXPECT_EQ(TestActuator::DestroyCount(), expected_destroy_count);

  mj_deleteModel(m);
  EXPECT_EQ(TestActuator::InitCount(), expected_init_count);
  EXPECT_EQ(TestActuator::DestroyCount(), expected_destroy_count);
}

}  // namespace
}  // namespace mujoco
