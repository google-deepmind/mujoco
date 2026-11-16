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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_CORE_BUILTINS_H_
#define MUJOCO_SRC_RENDER_FILAMENT_CORE_BUILTINS_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include <filament/Engine.h>
#include "render/filament/core/mesh.h"

namespace mujoco {

// A collection of meshes (shapes) that are "built in" to the renderer.
class Builtins {
 public:
  explicit Builtins(filament::Engine* engine);

  // Strongly typed integers to prevent accidental mixing.
  enum class Quads : int {};
  enum class Slices : int {};
  enum class Stacks : int {};

  // Returns a mesh for the corresponding built-in shape.
  const Mesh* Line();
  const Mesh* LineBox();
  const Mesh* Plane(Quads nquad);
  const Mesh* Haze(Slices nslice);
  const Mesh* Triangle();
  const Mesh* Box(Quads nquad);
  const Mesh* Sphere(Stacks nstack, Slices nslice);
  const Mesh* Cone(Stacks nstack, Slices nslice);
  const Mesh* Disk(Slices nslice, Quads nquad);
  const Mesh* DomeTop(Stacks nstack, Slices nslice);
  const Mesh* DomeBottom(Stacks nstack, Slices nslice);
  const Mesh* Tube(Stacks nstack, Slices nslice);

 private:
  using MeshPtr = std::unique_ptr<Mesh>;
  using MeshMap = std::unordered_map<uint64_t, MeshPtr>;

  // Returns a mesh from the map with the key, creating one using the builder if
  // it is not found.
  template <typename Fn>
  const Mesh* GetMesh(MeshMap& map, uint64_t key, Fn&& builder);

  filament::Engine* engine_;
  MeshMap lines_;
  MeshMap line_boxes_;
  MeshMap planes_;
  MeshMap hazes_;
  MeshMap triangles_;
  MeshMap boxes_;
  MeshMap spheres_;
  MeshMap cones_;
  MeshMap disks_;
  MeshMap dome_tops_;
  MeshMap dome_bottoms_;
  MeshMap tubes_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_RENDER_FILAMENT_CORE_BUILTINS_H_
