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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_MATERIAL_PARSING_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_MATERIAL_PARSING_H_

#include <pxr/usd/usdShade/material.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace usd {
mjsMaterial* ParseMaterial(mjSpec* spec, const pxr::UsdShadeMaterial &material);
}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_MATERIAL_PARSING_H_
