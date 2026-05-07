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

#include <filament/Engine.h>
#include "experimental/filament/filament/mesh.h"

namespace mujoco {

// A collection of meshes that "built in" to the renderer.
class Builtins {
 public:
  Builtins(filament::Engine* engine, int nstack, int nslice, int nquad);

  const Mesh* Line();
  const Mesh* LineBox();
  const Mesh* Plane();
  const Mesh* Triangle();
  const Mesh* Box();
  const Mesh* Sphere();
  const Mesh* Cone();
  const Mesh* Disk();
  const Mesh* Dome();
  const Mesh* Tube();

 private:
  std::unique_ptr<Mesh> line_;
  std::unique_ptr<Mesh> line_box_;
  std::unique_ptr<Mesh> plane_;
  std::unique_ptr<Mesh> triangle_;
  std::unique_ptr<Mesh> box_;
  std::unique_ptr<Mesh> sphere_;
  std::unique_ptr<Mesh> cone_;
  std::unique_ptr<Mesh> disk_;
  std::unique_ptr<Mesh> dome_;
  std::unique_ptr<Mesh> tube_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUILTINS_H_
