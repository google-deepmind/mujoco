// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_PLUGIN_THERMAL_H_
#define MUJOCO_PLUGIN_THERMAL_H_

#include <memory>
#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

namespace mujoco::plugin::thermal {

struct ThermalConfig {
    double thermal_capacitance;
    double thermal_resistance;
    double electrical_nominal_resistance;
    double temperature_coefficient_of_resistance;
    // Motor torque constant at 25°C (Kt25).
    double torque_constant_25;
    // Motor torque constant at 130°C (Kt130).
    double torque_constant_130;
    double gear_ratio;

    static ThermalConfig FromModel(const mjModel* m, int instance);
};

class Thermal {
 public:
  static std::unique_ptr<Thermal> Create(const mjModel* m, mjData* d, int instance);
  static constexpr char kAmbientTemperature[] = "ambient_temperature";

  void Reset(const mjModel* m, mjtNum* plugin_state, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  static void RegisterPlugin();

 private:
  Thermal(ThermalConfig model, std::vector<int> sensors);
  
  ThermalConfig model_;  // Thermal model of the actuator.
  std::vector<int> sensors_;  // Actuators the plugin is managing.
};


}  // namespace mujoco::plugin::thermal

#endif  // MUJOCO_PLUGIN_THERMAL_H_
