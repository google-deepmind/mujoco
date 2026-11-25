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

// Per-simulation mjModel array field overrides for viewing and picking a
// batch of simulations of one model; free of any renderer dependency.

#ifndef MUJOCO_STUDIO_SIM_BATCHED_MODEL_H_
#define MUJOCO_STUDIO_SIM_BATCHED_MODEL_H_

#include <cstddef>
#include <span>  // NOLINT
#include <string_view>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::studio {

// Per-simulation values of one mjModel array field.
struct BatchedModelField {
  // mjModel field name, e.g. "geom_rgba".
  std::string_view name;

  // Row-major buffer of shape (nsim, ...) in the field's native element type.
  const void* data = nullptr;

  // Bytes per simulation row; must equal the mjModel array's byte size.
  size_t sim_bytes = 0;
};

// Returns true if `name` is an mjModel field that can be applied per
// simulation.
bool IsBatchedModelField(std::string_view name);

// Writes the `fields` row for simulation `sim` into `model`. A local frame
// among `fields` (e.g. geom_pos) also clears `model`'s compile-time
// `*_sameframe` shortcuts so that mj_kinematics applies it. If `shape_row` is
// non-null, updates `*shape_row` with the fields the renderables are built from
// and returns true if they changed.
bool ApplyBatchedModelFields(mjModel* model, std::span<const BatchedModelField> fields, int sim,
                             std::vector<std::byte>* shape_row = nullptr);

// Copies `model`'s current values for the supported `fields` into `out`.
void SaveBatchedModelFields(const mjModel* model, std::span<const BatchedModelField> fields,
                            std::vector<std::byte>* out);

// Restores `model`'s fields from a buffer previously filled by
// SaveBatchedModelFields. Returns false without modifying `model` if `saved`'s
// size does not match.
bool RestoreBatchedModelFields(mjModel* model, std::span<const BatchedModelField> fields,
                               const std::vector<std::byte>& saved);

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_SIM_BATCHED_MODEL_H_
