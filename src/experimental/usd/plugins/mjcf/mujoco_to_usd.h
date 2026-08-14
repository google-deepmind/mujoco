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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_MUJOCO_TO_USD_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_MUJOCO_TO_USD_H_

#include <mujoco/mujoco.h>
#include <pxr/usd/sdf/declareHandles.h>

namespace mujoco {
namespace usd {
// Given an mjSpec, writes a USD representation of the data to the given layer.
//
// Args:
//   spec: mjSpec built programmatically or via parsed XML.
//   layer: SdfLayerRefPtr that will be written to.
//   skip_elems_from_usd: If true, skips over elements in the spec that have a usd_prim_path
//   custom attribute set.
bool WriteSpecToData(mjSpec* spec, pxr::SdfLayerRefPtr layer, bool skip_elems_from_usd = false);
}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_MUJOCO_TO_USD_H_
