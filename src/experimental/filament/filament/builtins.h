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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUILTINS_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUILTINS_H_

#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"

// Generates buffers for built-in shapes.
namespace mujoco {

UniquePtr<mjrMesh> CreateLine(mjrfContext* ctx);
UniquePtr<mjrMesh> CreatePlane(mjrfContext* ctx, int nquad);
UniquePtr<mjrMesh> CreateTriangle(mjrfContext* ctx);
UniquePtr<mjrMesh> CreateBox(mjrfContext* ctx, int nquad);
UniquePtr<mjrMesh> CreateLineBox(mjrfContext* ctx);
UniquePtr<mjrMesh> CreateSphere(mjrfContext* ctx, int nstack, int nslice);
UniquePtr<mjrMesh> CreateTube(mjrfContext* ctx, int nstack, int nslice);
UniquePtr<mjrMesh> CreateDisk(mjrfContext* ctx, int nslice);
UniquePtr<mjrMesh> CreateDome(mjrfContext* ctx, int nstack, int nslice);
UniquePtr<mjrMesh> CreateCone(mjrfContext* ctx, int nstack, int nslice);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUILTINS_H_
