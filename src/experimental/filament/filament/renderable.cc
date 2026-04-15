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

#include "experimental/filament/filament/renderable.h"

#include <cstdint>
#include <utility>

#include <filament/Engine.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/mesh.h"

namespace mujoco {

Renderable::Renderable(filament::Engine* engine) : material_(engine) {}

Renderable::~Renderable() noexcept {
  while (!entities_.empty()) {
    RemoveLastEntity();
  }
}

void Renderable::RemoveLastEntity() {
  if (entities_.empty()) {
    return;
  }

  utils::EntityManager& em = utils::EntityManager::get();
  utils::Entity entity = entities_.back();

  if (assigned_scene_) {
    assigned_scene_->remove(entity);
  }

  GetEngine()->destroy(entity);
  em.destroy(entity);
  entities_.pop_back();
  meshes_.pop_back();
}

void Renderable::UpdateMesh(int index, const Mesh* mesh, int elem_offset,
                            int elem_count) {
  MeshInfo& mesh_info = SetMesh(index, mesh, nullptr, elem_offset, elem_count);
  UpdateEntity(index, mesh_info);
}

void Renderable::UpdateMesh(int index, MeshPtr mesh, int elem_offset,
                            int elem_count) {
  MeshInfo& mesh_info =
      SetMesh(index, mesh.get(), std::move(mesh), elem_offset, elem_count);
  UpdateEntity(index, mesh_info);
}

void Renderable::AppendMesh(const Mesh* mesh, int elem_offset, int elem_count) {
  MeshInfo& mesh_info = SetMesh(-1, mesh, nullptr, elem_offset, elem_count);
  AppendEntity(mesh_info);
}

void Renderable::AppendMesh(MeshPtr mesh, int elem_offset, int elem_count) {
  MeshInfo& mesh_info =
      SetMesh(-1, mesh.get(), std::move(mesh), elem_offset, elem_count);
  AppendEntity(mesh_info);
}

void Renderable::AppendEntity(const MeshInfo& mesh_info) {
  const Mesh* mesh = mesh_info.mesh;
  filament::VertexBuffer* vertex_buffer = mesh->GetFilamentVertexBuffer();
  if (vertex_buffer == nullptr) {
    mju_error("Invalid (null) vertex buffer.");
  }

  filament::IndexBuffer* index_buffer = mesh->GetFilamentIndexBuffer();
  if (index_buffer == nullptr) {
    mju_error("Invalid (null) index buffer.");
  }

  utils::Entity entity = utils::EntityManager::get().create();
  if (entity.isNull()) {
    mju_error("Failed to create entity.");
  }

  filament::RenderableManager::Builder builder(1);
  builder.geometry(0, mesh->GetPrimitiveType(), vertex_buffer, index_buffer,
                   mesh_info.elem_offset, mesh_info.elem_count);
  if (mesh->HasBounds()) {
    builder.boundingBox(mesh->GetBounds());
  } else {
    builder.culling(false);
  }
  if (material_instance_) {
    builder.material(0, material_instance_);
  }
  builder.castShadows(cast_shadows_);
  builder.receiveShadows(receive_shadows_);
  builder.layerMask(0xff, layer_mask_);
  builder.priority(priority_);
  builder.blendOrder(0, blend_order_);
  builder.screenSpaceContactShadows(true);

  builder.build(*GetEngine(), entity);
  if (assigned_scene_) {
    assigned_scene_->addEntity(entity);
  }
  entities_.push_back(entity);
}

void Renderable::UpdateEntity(int index, const MeshInfo& mesh_info) {
  if (index < 0 || index >= entities_.size()) {
    mju_error("Invalid index %d for renderable.", index);
  }
  utils::Entity entity = entities_[index];

  const Mesh* mesh = mesh_info.mesh;
  filament::VertexBuffer* vertex_buffer = mesh->GetFilamentVertexBuffer();
  if (vertex_buffer == nullptr) {
    mju_error("Invalid (null) vertex buffer.");
  }

  filament::IndexBuffer* index_buffer = mesh->GetFilamentIndexBuffer();
  if (index_buffer == nullptr) {
    mju_error("Invalid (null) index buffer.");
  }

  filament::RenderableManager& rm = GetEngine()->getRenderableManager();
  rm.setGeometryAt(rm.getInstance(entity), 0, mesh->GetPrimitiveType(),
                   vertex_buffer, index_buffer, mesh_info.elem_offset,
                   mesh_info.elem_count);
}

Renderable::MeshInfo& Renderable::SetMesh(int index, const Mesh* mesh,
                                          MeshPtr owned_mesh, int elem_offset,
                                          int elem_count) {
  if (index == -1) {
    index = meshes_.size();
    meshes_.emplace_back();
  }
  if (index < 0 || index >= static_cast<int>(meshes_.size())) {
    mju_error("Invalid index %d for renderable.", index);
  }

  MeshInfo* mesh_info = &meshes_[index];
  mesh_info->owned_mesh = std::move(owned_mesh);
  mesh_info->mesh = mesh;
  mesh_info->elem_offset = elem_offset;
  mesh_info->elem_count = elem_count;
  if (mesh_info->elem_count == 0) {
    const int total =
        mesh_info->mesh->GetFilamentIndexBuffer()->getIndexCount();
    mesh_info->elem_count = total - mesh_info->elem_offset;
  }
  return *mesh_info;
}

void Renderable::AddToScene(filament::Scene* scene) {
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

void Renderable::RemoveFromScene(filament::Scene* scene) {
  if (assigned_scene_ != scene) {
    mju_error("Attempting to remove renderable from wrong scene.");
  }
  for (utils::Entity& entity : entities_) {
    scene->remove(entity);
  }
  assigned_scene_ = nullptr;
}

void Renderable::SetMaterialInstance(filament::MaterialInstance* instance) {
  if (instance != material_instance_) {
    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      filament::RenderableManager::Instance ri = rm.getInstance(entity);
      rm.setMaterialInstanceAt(ri, 0, instance);
    }
    material_instance_ = instance;
  }
}

std::uint8_t Renderable::SetLayerMask(std::uint8_t mask) {
  std::uint8_t prev = layer_mask_;
  if (mask != layer_mask_) {
    layer_mask_ = mask;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setLayerMask(rm.getInstance(entity), 0xff, layer_mask_);
    }
  }
  return prev;
}

std::uint8_t Renderable::SetPriority(std::uint8_t priority) {
  std::uint8_t prev = priority_;
  if (priority != priority_) {
    priority_ = priority;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setPriority(rm.getInstance(entity), priority_);
    }
  }
  return prev;
}

std::uint16_t Renderable::SetBlendOrder(std::uint16_t blend_order) {
  std::uint16_t prev = blend_order_;
  if (blend_order != blend_order_) {
    blend_order_ = blend_order;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setBlendOrderAt(rm.getInstance(entity), 0, blend_order_);
    }
  }
  return prev;
}

void Renderable::SetCastShadows(bool cast_shadows) {
  if (cast_shadows_ != cast_shadows) {
    cast_shadows_ = cast_shadows;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setCastShadows(rm.getInstance(entity), cast_shadows_);
    }
  }
}

void Renderable::SetReceiveShadows(bool receive_shadows) {
  if (receive_shadows_ != receive_shadows) {
    receive_shadows_ = receive_shadows;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (utils::Entity& entity : entities_) {
      rm.setReceiveShadows(rm.getInstance(entity), receive_shadows_);
    }
  }
}

void Renderable::SetWireframe(bool wireframe) {
  static constexpr auto kWireframeType =
      filament::RenderableManager::PrimitiveType::LINES;

  if (wireframe != wireframe_) {
    wireframe_ = wireframe;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (int i = 0; i < entities_.size(); ++i) {
      utils::Entity& entity = entities_[i];
      const Mesh* mesh = meshes_[i].mesh;
      filament::VertexBuffer* vertex_buffer = mesh->GetFilamentVertexBuffer();
      filament::IndexBuffer* index_buffer = mesh->GetFilamentIndexBuffer();
      rm.setGeometryAt(rm.getInstance(entity), 0,
                       wireframe_ ? kWireframeType : mesh->GetPrimitiveType(),
                       vertex_buffer, index_buffer, meshes_[i].elem_offset,
                       meshes_[i].elem_count);
    }
  }
}

Material& Renderable::GetMaterial() { return material_; }

filament::Engine* Renderable::GetEngine() { return material_.GetEngine(); }

}  // namespace mujoco
