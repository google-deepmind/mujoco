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

#include <filament/Engine.h>
#include "experimental/filament/filament/buffer_util.h"

// Generates buffers for built-in shapes.
namespace mujoco {

MeshPtr CreateLine(filament::Engine* engine);
MeshPtr CreatePlane(filament::Engine* engine, int nquad);
MeshPtr CreateTriangle(filament::Engine* engine);
MeshPtr CreateBox(filament::Engine* engine, int nquad);
MeshPtr CreateLineBox(filament::Engine* engine);
MeshPtr CreateSphere(filament::Engine* engine, int nstack, int nslice);
MeshPtr CreateTube(filament::Engine* engine, int nstack, int nslice);
MeshPtr CreateDisk(filament::Engine* engine, int nslice);
MeshPtr CreateDome(filament::Engine* engine, int nstack, int nslice);
MeshPtr CreateCone(filament::Engine* engine, int nstack, int nslice);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUILTINS_H_
