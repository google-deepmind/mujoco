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

#ifndef MUJOCO_EXPERIMENTAL_SRC_USD_USD_TO_MJSPEC_H_
#define MUJOCO_EXPERIMENTAL_SRC_USD_USD_TO_MJSPEC_H_

#include <mujoco/mujoco.h>
#include <pxr/usd/usd/common.h>

// Given a USD stage, this function will do a best effort conversion to
// mjSpec.
//
// Particular care is taken for physics data to be lossless but visual
// data such as materials may be lossy.
MJAPI mjSpec* mj_parseUSDStage(pxr::UsdStageRefPtr stage);

#endif  // MUJOCO_EXPERIMENTAL_SRC_USD_USD_TO_MJSPEC_H_
