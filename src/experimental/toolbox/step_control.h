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

#ifndef MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_STEP_CONTROL_H_
#define MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_STEP_CONTROL_H_

#include <chrono>
#include <string>

#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

using Seconds = std::chrono::duration<double>;
using Clock = std::chrono::steady_clock;

// State and logic for physics synchronization and stepping.
class StepControl {
 public:
  enum class Status {
    kOk,

    // Simulation diverged with autoreset enabled.
    kAutoReset,

    // Simulation diverged with autoreset disabled.
    // Note: Consider reporting mjData warning diagnostics in kDivergedWarnings.
    kDiverged,
  };

  // List of warnings that are checked to determine simulation divergence.
  static constexpr mjtWarning kDivergedWarnings[] = {
      mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS};

  // Steps physics forward, respecting speed settings and refresh budget.
  Status Advance(const mjModel* m, mjData* d);

  // Ensures the next call to Advance() will synchronize time and step once.
  void ForceSync();

  // Gets/sets the desired simulation speed as a percentage of real time.
  float GetSpeed() const;
  float GetSpeedMeasured() const;
  void SetSpeed(float speed);  // speed is clamped to [0.1%, 100%]

  // Gets/sets the control noise parameters applied before stepping.
  void GetNoiseParameters(float& noise_scale, float& noise_rate) const;
  void SetNoiseParameters(float noise_scale, float noise_rate);

 private:
  std::string AdvanceOneStep(const mjModel* m, mjData* d);

  void InjectNoise(const mjModel* m, mjData* d);

  // Control noise standard deviation
  double ctrl_noise_std_ = 0;

  // Control noise correlation rate
  double ctrl_noise_rate_ = 0;

  // Desired simulation speed as a percentage of real time
  float speed_ = 100;

  // Measured simulation speed as a percentage of real time
  float speed_measured_ = -1;

  // If true, the next call to Advance() will synchronize time step once.
  bool force_sync_ = true;

  // CPU time (aka wall time) of the last synchronization event
  std::chrono::time_point<Clock> sync_cpu_;

  // Simulation time of the last synchronization event
  mjtNum sync_sim_ = 0;

  // Maximum mis-alignment before re-sync (simulation seconds)
  double sync_misalign_ = .1;
};

}  // namespace mujoco::toolbox

#endif  // MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_STEP_CONTROL_H_
