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

#include <string>

#include <mujoco/mujoco.h>

namespace mujoco::platform {

// The mode of the spec visualizer.
enum class SpecEditMode {
  kPlay,
  kEdit,
};

// Returns the name of the given element.
std::string ElementName(mjsElement* element);

// Displaying the mjSpec as a tree. `element` is the currently selected
// element and will be updated if a new element is selected. The function
// returns true if the spec was modified in any way.
bool SpecTreeGui(mjsElement** element, mjSpec* spec, SpecEditMode mode);

// Displays the properties of the given element in the table. Returns true if
// any value in the element was changed. The `ref_element` is used to highlight
// when a value differs from a reference element.
bool ElementSpecGui(mjsElement* element, mjsElement* ref_element,
                    SpecEditMode mode);

// Displays a (read-only) data table of the mjData values that correspond to the
// given element.
void ElementDataGui(const mjData* data, mjsElement* element);

// Displays a (read-only) data table of the mjModel values that correspond to
// the given element.
void ElementModelGui(const mjModel* model, mjsElement* element);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_GUI_SPEC_H_
