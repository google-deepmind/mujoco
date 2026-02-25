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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_SPEC_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_SPEC_H_

#include <functional>

#include <mujoco/mujoco.h>

namespace mujoco::platform {

using SpecElementCallbackFn = std::function<void(mjsElement*)>;

// UX for displaying the spec as a tree.
void SpecExplorerGui(mjsElement** element, mjSpec* spec,
                     const SpecElementCallbackFn& on_delete);

// UX for displaying the properties of an mjSpec element.
void ElementSpecGui(const mjSpec* spec, mjsElement* element);
void ElementDataGui(const mjData* data, mjsElement* element);
void ElementModelGui(const mjModel* model, mjsElement* element);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_SPEC_H_
