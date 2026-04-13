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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLE_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLE_H_

#include <cstdint>
#include <vector>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <utils/Entity.h>
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/mesh.h"

namespace mujoco {

// Manages a collection of related filament Renderable Entities.
class Renderable {
 public:
  // Default filament values for priority and layer mask.
  static constexpr std::uint8_t kDefaultPriority = 4;
  static constexpr std::uint8_t kDefaultLayerMask = 0x01;

  Renderable(filament::Engine* engine);
  ~Renderable() noexcept;

  Renderable(const Renderable&) = delete;
  Renderable& operator=(const Renderable&) = delete;

  // Appends a new renderable entity built from the given mesh.
  void Append(const Mesh* mesh);
  void Append(MeshPtr mesh);

  // Updates the entity at the index with new mesh.
  void Update(int index, const Mesh* mesh);
  void Update(int index, MeshPtr mesh);

  // Removes the last entity.
  void RemoveLast();

  // Returns the entity at the given index.
  utils::Entity operator[](int index) { return entities_[index]; }

  // Returns the number of Entities that make up this renderable.
  int GetNumEntities() const { return entities_.size(); }

  // Hides all managed entities.
  void SetLayerMask(std::uint8_t mask);

  // Sets the priority of all managed entities.
  void SetPriority(std::uint8_t priority);

  // Disables the renderables from casting shadows.
  void SetCastShadows(bool cast_shadows);

  // Disables the renderables from receiving shadows.
  void SetReceiveShadows(bool receive_shadows);

  // If true, forces all entities to be rendered as lines.
  void SetWireframe(bool wireframe);

  // Adds all managed entities to the given filament Scene.
  void AddToScene(filament::Scene* scene);

  // Removes all managed entities from the given filament Scene.
  void RemoveFromScene(filament::Scene* scene);

  // Sets the material instance for all managed entities.
  void SetMaterialInstance(filament::MaterialInstance* material_instance);

  // Returns the material for the renderables.
  Material& GetMaterial();

  // Returns the filament Engine managing the renderables.
  filament::Engine* GetEngine();

 private:
  utils::Entity CreateEntity(const Mesh* mesh);
  void UpdateEntity(utils::Entity entity, const Mesh* mesh);
  void UpdateMeshes(int index, const Mesh* mesh, MeshPtr owned_mesh = nullptr);

  struct MeshWrapper {
    MeshPtr owned_mesh;
    const Mesh* mesh = nullptr;
  };

  Material material_;
  filament::Scene* assigned_scene_ = nullptr;
  filament::MaterialInstance* material_instance_ = nullptr;
  std::vector<utils::Entity> entities_;
  std::vector<MeshWrapper> meshes_;
  std::uint8_t priority_ = kDefaultPriority;
  std::uint8_t layer_mask_ = kDefaultLayerMask;
  bool wireframe_ = false;
  bool cast_shadows_ = true;
  bool receive_shadows_ = true;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLE_H_
