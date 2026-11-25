// Copyright 2026 DeepMind Technologies Limited
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

// Viewer input for a batch of simulations of one mjModel: per-sim state rows,
// per-sim model fields, and how to lay them out. Shared by the scene that
// draws them (render/batched_model_scene.h) and the picking that ray-casts
// them (ux/interaction.h); free of any renderer dependency.
//
// The vocabulary is that of the batch API (mjbatch.h): a batch holds nsim
// simulations of one model, a simulation is a state row, and a model field
// with a value per simulation (mjb_expand) is a per-sim field.

#ifndef MUJOCO_STUDIO_SIM_BATCH_VIEW_H_
#define MUJOCO_STUDIO_SIM_BATCH_VIEW_H_

#include <span>  // NOLINT

#include <mujoco/mujoco.h>
#include "studio/sim/batched_model.h"

namespace mujoco::studio {

// Viewer input for a batch of simulations of a single mjModel.
struct BatchView {
  // Number of simulations in the batch; rows are indexed in [0, nsim).
  int nsim = 0;

  // Simulation mirrored by the viewer's own (model, data) and drawn with full
  // decorations, or -1 if (model, data) is separate from the batch rows.
  int active_sim = 0;

  // Whether to hide all simulations other than active_sim.
  bool others_hidden = false;

  // Whether to draw the other simulations with mjVIS_TRANSPARENT.
  bool others_ghosted = false;

  // Whether to draw decorations (contacts, frames, labels, forces, ...) on the
  // other simulations.
  bool others_decorated = false;

  // Row-major (nsim, 3) world-space scene translation per simulation.
  const float* offsets = nullptr;

  // Bitmask of mjtState components stored in each row of states.
  int state_sig = 0;

  // Row-major (nsim, mj_stateSize(model, state_sig)) simulation state rows.
  const mjtNum* states = nullptr;

  // Optional (nsim) status codes (mjtWarning); nonzero rows are drawn ghosted.
  const int* status = nullptr;

  // Per-simulation mjModel array overrides (only supported fields are applied).
  std::span<const BatchedModelField> model_fields;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_SIM_BATCH_VIEW_H_
