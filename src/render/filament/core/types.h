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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_CORE_TYPES_H_
#define MUJOCO_SRC_RENDER_FILAMENT_CORE_TYPES_H_

#if defined(__cplusplus)
extern "C" {
#endif

// These types are only forward declared in the API. We define them concretely
// here to allow us to use them in the core classes.
struct mjrfContext_ {};
struct mjrfTexture_ {};
struct mjrfMesh_ {};
struct mjrfScene_ {};
struct mjrfLight_ {};
struct mjrfRenderable_ {};
struct mjrfRenderTarget_ {};

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // MUJOCO_SRC_RENDER_FILAMENT_CORE_TYPES_H_
