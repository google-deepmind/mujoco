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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_LAYER_SINK_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_LAYER_SINK_H_

#include <vector>

#include <mujoco/mujoco.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>

namespace mujoco {
namespace usd {

// A sink for writing pose data to an SDF layer.
// Opinions will be authored on the edit target layer for the
// passed stage argument.
class LayerSink {
 public:
  explicit LayerSink(pxr::UsdStageRefPtr stage);
  void Update(const mjData* const data, std::vector<pxr::SdfPath> body_paths_);

 private:
  pxr::UsdStageRefPtr stage_;
};
}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_LAYER_SINK_H_
