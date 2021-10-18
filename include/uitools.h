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

#ifndef MUJOCO_UITOOLS_H_
#define MUJOCO_UITOOLS_H_


#include "mujoco.h"
#include "glfw3.h"


// this is a C-API
#if defined(__cplusplus)
extern "C"
{
#endif


// User-supplied callback function types.
typedef void (*uiEventFn)(mjuiState* state);
typedef void (*uiLayoutFn)(mjuiState* state);

// Container for GLFW window pointer.
struct _uiUserPointer
{
    mjuiState* state;
    uiEventFn uiEvent;
    uiLayoutFn uiLayout;
    double buffer2window;
};
typedef struct _uiUserPointer uiUserPointer;

// Set internal and user-supplied UI callbacks in GLFW window.
void uiSetCallback(GLFWwindow* wnd, mjuiState* state,
                   uiEventFn uiEvent, uiLayoutFn uiLayout);

// Clear UI callbacks in GLFW window.
void uiClearCallback(GLFWwindow* wnd);

// Compute suitable font scale.
int uiFontScale(GLFWwindow* wnd);

// Modify UI structure.
void uiModify(GLFWwindow* wnd, mjUI* ui, mjuiState* state, mjrContext* con);


#if defined(__cplusplus)
}
#endif

#endif  // MUJOCO_UITOOLS_H_
