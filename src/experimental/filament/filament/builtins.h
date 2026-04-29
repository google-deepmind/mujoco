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

#include <memory>

#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/mesh.h"

// Generates buffers for built-in shapes.
namespace mujoco {

std::unique_ptr<Mesh> CreateLine(FilamentContext* ctx);
std::unique_ptr<Mesh> CreatePlane(FilamentContext* ctx, int nquad);
std::unique_ptr<Mesh> CreateTriangle(FilamentContext* ctx);
std::unique_ptr<Mesh> CreateBox(FilamentContext* ctx, int nquad);
std::unique_ptr<Mesh> CreateLineBox(FilamentContext* ctx);
std::unique_ptr<Mesh> CreateSphere(FilamentContext* ctx, int nstack, int nslice);
std::unique_ptr<Mesh> CreateTube(FilamentContext* ctx, int nstack, int nslice);
std::unique_ptr<Mesh> CreateDisk(FilamentContext* ctx, int nslice);
std::unique_ptr<Mesh> CreateDome(FilamentContext* ctx, int nstack, int nslice);
std::unique_ptr<Mesh> CreateCone(FilamentContext* ctx, int nstack, int nslice);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUILTINS_H_
