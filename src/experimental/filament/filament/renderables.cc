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

#include "experimental/filament/filament/renderables.h"

#include <optional>

#include <filament/Engine.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"

namespace mujoco {

Renderables::Renderables(filament::Engine* engine) : engine_(engine) {}

Renderables::~Renderables() {
  while (!entities_.empty()) {
    RemoveLast();
  }
}

void Renderables::RemoveLast() {
  if (entities_.empty()) {
    return;
  }

  utils::EntityManager& em = utils::EntityManager::get();
  utils::Entity entity = entities_.back();

  if (assigned_scene_) {
    assigned_scene_->remove(entity);
  }

  engine_->destroy(entity);
  em.destroy(entity);
  entities_.pop_back();

  UpdateBuffers(owned_buffers_.size() - 1, std::nullopt);
  owned_buffers_.pop_back();
}

void Renderables::Update(int index, const FilamentBuffers& buffers) {
  if (index < 0 || index >= entities_.size()) {
    mju_error("Invalid index %d for renderable.", index);
  }
  utils::Entity& entity = entities_[index];
  UpdateEntity(entity, buffers);
  UpdateBuffers(index, std::nullopt);
}

void Renderables::Update(int index, FilamentBuffers&& buffers) {
  if (index < 0 || index >= entities_.size()) {
    mju_error("Invalid index %d for renderable.", index);
  }
  utils::Entity& entity = entities_[index];
  UpdateEntity(entity, buffers);
  UpdateBuffers(index, buffers);
}

void Renderables::Append(const FilamentBuffers& buffers) {
  utils::Entity entity = CreateEntity(buffers);
  entities_.push_back(entity);
  owned_buffers_.push_back(std::nullopt);
}

void Renderables::Append(FilamentBuffers&& buffers) {
  utils::Entity entity = CreateEntity(buffers);
  entities_.push_back(entity);
  owned_buffers_.push_back(buffers);
}

utils::Entity Renderables::CreateEntity(const FilamentBuffers& buffers) {
  if (buffers.vertex_buffer == nullptr) {
    mju_error("Invalid (null) vertex buffer.");
  }
  if (buffers.index_buffer == nullptr) {
    mju_error("Invalid (null) index buffer.");
  }
  utils::Entity entity = utils::EntityManager::get().create();
  if (entity.isNull()) {
    mju_error("Failed to create entity.");
  }

  filament::RenderableManager::Builder builder(1);
  builder.geometry(0, buffers.type, buffers.vertex_buffer,
                   buffers.index_buffer);
  if (material_instance_) {
    builder.material(0, material_instance_);
  }
  builder.boundingBox(buffers.bounds)
      .culling(false)
      .castShadows(true)
      .receiveShadows(true)
      .layerMask(1, visible_ ? 1 : 0)
      .screenSpaceContactShadows(true);

  builder.build(*engine_, entity);
  if (assigned_scene_) {
    assigned_scene_->addEntity(entity);
  }
  return entity;
}

void Renderables::UpdateEntity(utils::Entity entity,
                               const FilamentBuffers& buffers) {
  if (buffers.vertex_buffer == nullptr) {
    mju_error("Invalid (null) vertex buffer.");
  }
  if (buffers.index_buffer == nullptr) {
    mju_error("Invalid (null) index buffer.");
  }
  filament::RenderableManager& rm = engine_->getRenderableManager();
  rm.setGeometryAt(rm.getInstance(entity), 0, buffers.type,
                   buffers.vertex_buffer, buffers.index_buffer, 0,
                   buffers.index_buffer->getIndexCount());
}

void Renderables::UpdateBuffers(int index,
                                std::optional<FilamentBuffers> buffers) {
  if (index < 0 || index >= owned_buffers_.size()) {
    mju_error("Invalid index %d for renderable.", index);
  }
  if (owned_buffers_[index].has_value()) {
    engine_->destroy(owned_buffers_[index]->vertex_buffer);
    engine_->destroy(owned_buffers_[index]->index_buffer);
  }
  owned_buffers_[index] = buffers;
}

void Renderables::AddToScene(filament::Scene* scene) {
  if (assigned_scene_) {
    if (assigned_scene_ != scene) {
      mju_error("Cannot add renderable to multiple scenes.");
    }
    // Entities are already added to the scene.
    return;
  }
  for (utils::Entity& entity : entities_) {
    scene->addEntity(entity);
  }
  assigned_scene_ = scene;
}

void Renderables::RemoveFromScene(filament::Scene* scene) {
  if (assigned_scene_ != scene) {
    mju_error("Attempting to remove renderable from wrong scene.");
  }
  for (utils::Entity& entity : entities_) {
    scene->remove(entity);
  }
  assigned_scene_ = nullptr;
}

void Renderables::SetMaterialInstance(
    filament::MaterialInstance* instance) {
  if (instance != material_instance_) {
    filament::RenderableManager& rm = engine_->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      filament::RenderableManager::Instance ri = rm.getInstance(entity);
      rm.setMaterialInstanceAt(ri, 0, instance);
    }
    material_instance_ = instance;
  }
}

void Renderables::Hide() {
  if (visible_) {
    filament::RenderableManager& rm = engine_->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setLayerMask(rm.getInstance(entity), 1, 0);
    }
    visible_ = false;
  }
}

void Renderables::Show() {
  if (!visible_) {
    filament::RenderableManager& rm = engine_->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setLayerMask(rm.getInstance(entity), 1, 1);
    }
    visible_ = true;
  }
}

}  // namespace mujoco
