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

#ifndef MUJOCO_PLUGIN_USD_DECODER_COLLISION_FILTERING_H_
#define MUJOCO_PLUGIN_USD_DECODER_COLLISION_FILTERING_H_

#include <mujoco/mjspec.h>
#include <pxr/usd/usd/common.h>

struct Node;

// Parse pairwise collision filtering after all bodies and geoms are populated.
void ParseCollisionFiltering(const pxr::UsdStageRefPtr& stage, mjSpec* spec,
                             const Node& root);

#endif  // MUJOCO_PLUGIN_USD_DECODER_COLLISION_FILTERING_H_
