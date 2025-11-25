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

#ifndef MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_PHYSICS_H_
#define MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_PHYSICS_H_

#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>
#include "experimental/toolbox/sim_history.h"
#include "experimental/toolbox/step_control.h"

namespace mujoco::toolbox {

// Owns the MuJoCo simulation state (e.g. mjModel and mjData) and is responsible
// for updating the state of the simulation.
class Physics {
 public:
  using OnModelLoadedFn = std::function<void(std::string_view)>;

  explicit Physics(OnModelLoadedFn on_model_loaded);
  ~Physics();

  Physics(const Physics&) = delete;
  Physics& operator=(const Physics&) = delete;

  StepControl& GetStepControl() { return step_control_; }
  SimHistory& GetSimHistory() { return sim_history_; }

  // Loads a model from the given path. An empty string will load an empty
  // scene.
  void LoadModel(std::string model_file, const mjVFS* vfs = nullptr);

  // Resets the simulation using mj_resetData
  void Reset();

  // Advances the state of the simulation.
  bool Update();

  // Sets the state of the simulation.
  bool UpdateState(mjtNum* state, unsigned int state_sig);

  // Loads a state from the history buffer at the given offset in the past.
  void LoadHistory(int offset);
  int GetHistoryIndex() const;

  // Returns the MuJoCo data structures owned by this Simulation object.
  mjModel* GetModel() { return model_; }
  mjData* GetData() { return data_; }

  // Returns the error message from the simulation.
  std::string_view GetError() const { return error_; }

  bool ProcessPendingLoad();

 private:
  // Clears the simulation, clearing all loaded state.
  void Clear();

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;

  OnModelLoadedFn on_model_loaded_;

  std::optional<std::string> pending_load_;
  const mjVFS* vfs_;

  std::string error_;

  StepControl step_control_;
  SimHistory sim_history_;
};

}  // namespace mujoco::toolbox

#endif  // MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_PHYSICS_H_
