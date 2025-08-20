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
    float thermal_capacitance;
    float thermal_resistance;
    float electrical_nominal_resistance;
    float temperature_coefficient;
    // Motor Constant at 25 C (Kt25)
    float motor_constant_25;
    // Motor Constant at 135 C (Kt130)
    float motor_constant_130;

} ;

class Thermal {
 public:
  static std::unique_ptr<Thermal>* Create(const mjModel* m, mjData* d, int instance);

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  int GetNumberOfStates();
  static void RegisterPlugin();

 private:
  Thermal(ThermalConfig model, std::vector<int> actuators);
  
  ThermalConfig _model; //Thermal model of the actuator
  std::vector<int> _actuators; // Actuators the plugin is managing
};


}  // namespace mujoco::plugin::actuator

#endif  // MUJOCO_PLUGIN_ACTUATOR_PID_H_
