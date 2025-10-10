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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLES_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLES_H_

#include <optional>
#include <span>
#include <vector>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <utils/Entity.h>
#include "experimental/filament/filament/buffer_util.h"

namespace mujoco {

// Manages a collection of related filament Renderable Entities.
class Renderables {
 public:
  Renderables(filament::Engine* engine);
  ~Renderables() noexcept;

  Renderables(const Renderables&) = delete;
  Renderables& operator=(const Renderables&) = delete;

  // Appends a new renderable entity built from the given buffers.
  void Append(const FilamentBuffers& buffers);
  void Append(FilamentBuffers&& buffers);

  // Updates the entity at the index with new buffers.
  void Update(int index, const FilamentBuffers& buffers);
  void Update(int index, FilamentBuffers&& buffers);

  // Removes the last entity.
  void RemoveLast();

  // Returns the entity at the given index.
  utils::Entity operator[](int index) { return entities_[index]; }

  // Returns the owned buffers at the given index.
  int GetNumEntities() const { return entities_.size(); }

  // Hides all managed entities.
  void Hide();

  // Shows all managed entities.
  void Show();

  // Returns true if the entities are visible.
  bool IsVisible() const { return visible_; }

  // Adds all managed entities to the given filament Scene.
  void AddToScene(filament::Scene* scene);

  // Removes all managed entities from the given filament Scene.
  void RemoveFromScene(filament::Scene* scene);

  // Sets the material instance for all managed entities.
  void SetMaterialInstance(filament::MaterialInstance* material_instance);

  // Returns the filament Engine managing the entities in this collection.
  filament::Engine* GetEngine() { return engine_; }

 private:
  utils::Entity CreateEntity(const FilamentBuffers& buffers);
  void UpdateEntity(utils::Entity entity, const FilamentBuffers& buffers);
  void UpdateBuffers(int index, std::optional<FilamentBuffers> buffers);

  filament::Engine* engine_ = nullptr;
  filament::Scene* assigned_scene_ = nullptr;
  filament::MaterialInstance* material_instance_ = nullptr;
  std::vector<utils::Entity> entities_;
  std::vector<std::optional<FilamentBuffers>> owned_buffers_;
  bool visible_ = true;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLES_H_
