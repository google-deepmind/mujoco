// Copyright 2021 DeepMind Technologies Limited
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

#ifndef THIRD_PARTY_MUJOCO_SRC_UI_UI_MAIN_H
#define THIRD_PARTY_MUJOCO_SRC_UI_UI_MAIN_H

#include <mujoco/mjexport.h>
#include <mujoco/mjui.h>

#if defined(__cplusplus)
extern "C" {
#endif

// Get builtin UI theme spacing (ind: 0-1).
MJAPI mjuiThemeSpacing mjui_themeSpacing(int ind);

// Get builtin UI theme color (ind: 0-3).
MJAPI mjuiThemeColor mjui_themeColor(int ind);

// Add definitions to UI.
MJAPI void mjui_add(mjUI* ui, const mjuiDef* def);

// Add definitions to UI section.
MJAPI void mjui_addToSection(mjUI* ui, int sect, const mjuiDef* def);

// Compute UI sizes.
MJAPI void mjui_resize(mjUI* ui, const mjrContext* con);

// Update specific section/item; -1: update all.
MJAPI void mjui_update(int section, int item, const mjUI* ui,
                       const mjuiState* state, const mjrContext* con);

// Handle UI event, return pointer to changed item, NULL if no change.
MJAPI mjuiItem* mjui_event(mjUI* ui, mjuiState* state, const mjrContext* con);

// Copy UI image to current buffer.
MJAPI void mjui_render(mjUI* ui, const mjuiState* state, const mjrContext* con);

#if defined(__cplusplus)
}
#endif
#endif  // THIRD_PARTY_MUJOCO_SRC_UI_UI_MAIN_H
