// Copyright 2023 DeepMind Technologies Limited
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

// Tests for thermal sensor plugin.

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

#include "test/fixture.h"

namespace mujoco {
namespace {

using ThermalPluginTest = PluginTest;

static constexpr const char* kThermalPath =
    "plugin/thermal/testdata/thermal_test.xml";
static constexpr const char* kThermalPluginName = "mujoco.sensor.thermal";

TEST_F(ThermalPluginTest, ValidThermalAttributes) {
  const std::string xml_path = GetTestDataFilePath(kThermalPath);
  char error[1024] = {0};
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  mjData* data = mj_makeData(model);
  ASSERT_THAT(model, testing::NotNull()) << error;
  int sensor_state = 1;
  EXPECT_THAT(model->sensor_dim[0], sensor_state);
  EXPECT_THAT(model->sensor_plugin[0], 0);
  int thermal_state_numbers = 1;
  EXPECT_THAT(model->plugin_statenum[0], thermal_state_numbers);

  const mjpPlugin* plugin = mjp_getPlugin(kThermalPluginName, nullptr);

  constexpr int kThermalAttributeCount = 7;
  EXPECT_THAT(plugin->nattribute, kThermalAttributeCount);

  const char* const thermal_attributes[] = {
      "C",
      "Rth",
      "RNorm",
      "TempCoeff",
      "Kt25",
      "Kt130",
      "G"
  };

  double thermal_config_value[] = {
      42.0,      // C
      3.4,       // Rth
      0.46,      // RNorm
      0.039,     // TempCoeff
      0.068,     // Kt25
      0.061,     // Kt130
      3141.59    // G
  };

  for (int i = 0; i < kThermalAttributeCount; i++) {
    const char* config_str = mj_getPluginConfig(model, 0, thermal_attributes[i]);
    double model_value = std::strtod(config_str, nullptr);
    EXPECT_NEAR(model_value, thermal_config_value[i], 1e-6);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ThermalPluginTest, TemperatureAfterMillionSteps) {
  const std::string xml_path = GetTestDataFilePath(kThermalPath);
  char error[1024] = {0};
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, testing::NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Run simulation for 1 million steps (2000 seconds at 0.002s timestep).
  constexpr int kNumSteps = 1000000;
  constexpr double kActuatorForce = 100.0;
  for (int i = 0; i < kNumSteps; ++i) {
    data->ctrl[0] = kActuatorForce;
    mj_step(model, data);
  }

  // Expected steady-state temperature in Kelvin.
  constexpr double kExpectedTemperature = 298.50;
  constexpr double kTolerance = 1e-2;
  EXPECT_NEAR(data->sensordata[0], kExpectedTemperature, kTolerance);

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
