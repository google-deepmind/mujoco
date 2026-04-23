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

#include <algorithm>
#include <cstdint>
#include <span>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <math/mat4.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/draw_mode.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

using filament::math::mat4f;

void mjr_defaultRenderableParams(mjrRenderableParams* params) {
  params->shading_model = mjSHADING_MODEL_SCENE_OBJECT;
}

Renderable::Renderable(ObjectManager* object_mgr, const mjrRenderableParams& params)
    : object_mgr_(object_mgr), params_(params) {}

Renderable::~Renderable() noexcept {
  filament::Engine* engine = GetEngine();
  utils::EntityManager& em = utils::EntityManager::get();

  for (Part& part : parts_) {
    if (assigned_scene_) {
      assigned_scene_->remove(part.entity);
    }
    engine->destroy(part.entity);
    em.destroy(part.entity);
  }
  for (int i = 0; i < kNumDrawModes; ++i) {
    if (instances_[i] != nullptr) {
      engine->destroy(instances_[i]);
      instances_[i] = nullptr;
    }
  }
}

void Renderable::SetMesh(const Mesh* mesh, int elem_offset, int elem_count) {
  if (mesh == nullptr) {
    mju_error("Cannot set mesh to nullptr.");
  }
  filament::VertexBuffer* vertex_buffer = mesh->GetFilamentVertexBuffer();
  if (vertex_buffer == nullptr) {
    mju_error("Invalid (null) vertex buffer.");
  }
  filament::IndexBuffer* index_buffer = mesh->GetFilamentIndexBuffer();
  if (index_buffer == nullptr) {
    mju_error("Invalid (null) index buffer.");
  }

  if (elem_count == 0) {
    elem_count = index_buffer->getIndexCount() - elem_offset;
  }

  if (parts_.empty()) {
    Part& part = parts_.emplace_back();
    part.mesh = mesh;
    part.elem_offset = elem_offset;
    part.elem_count = elem_count;
    InitPartEntity(part);
  } else if (parts_.size() == 1) {
    Part& part = parts_[0];
    part.mesh = mesh;
    part.elem_offset = elem_offset;
    part.elem_count = elem_count;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    rm.setGeometryAt(rm.getInstance(part.entity), 0,
                     part.mesh->GetPrimitiveType(), vertex_buffer, index_buffer,
                     part.elem_offset, part.elem_count);

  } else {
    mju_error("Cannot set mesh for renderable with multiple parts.");
  }
}

void Renderable::InitPartEntity(Part& part) {
  part.entity = utils::EntityManager::get().create();
  if (part.entity.isNull()) {
    mju_error("Failed to create entity.");
  }

  filament::VertexBuffer* vertex_buffer = part.mesh->GetFilamentVertexBuffer();
  filament::IndexBuffer* index_buffer = part.mesh->GetFilamentIndexBuffer();

  filament::RenderableManager::Builder builder(1);
  builder.geometry(0, part.mesh->GetPrimitiveType(), vertex_buffer, index_buffer,
                   part.elem_offset, part.elem_count);
  if (part.mesh->HasBounds()) {
    builder.boundingBox(part.mesh->GetBounds());
  } else {
    builder.culling(false);
  }
  if (instances_[static_cast<int>(draw_mode_)] != nullptr) {
    builder.material(0, instances_[static_cast<int>(draw_mode_)]);
  }
  builder.castShadows(cast_shadows_);
  builder.receiveShadows(receive_shadows_);
  builder.layerMask(0xff, layer_mask_);
  builder.priority(priority_);
  builder.blendOrder(0, blend_order_);
  builder.screenSpaceContactShadows(true);

  builder.build(*GetEngine(), part.entity);
  if (assigned_scene_) {
    assigned_scene_->addEntity(part.entity);
  }
}

void Renderable::SetTransform(const Trs& trs) {
  if (parts_.empty()) {
    transform_ = trs.ToTransform();
    return;
  }

  filament::TransformManager& tm = GetEngine()->getTransformManager();
  if (get_transform_fn_) {
    for (int i = 0; i < parts_.size(); ++i) {
      const mat4f& transform = get_transform_fn_(i, trs);
      tm.setTransform(tm.getInstance(parts_[i].entity), transform);
    }
  } else {
    for (Part& part : parts_) {
      tm.setTransform(tm.getInstance(part.entity), trs.ToTransform());
    }
  }
  transform_ = tm.getTransform(tm.getInstance(parts_[0].entity));
}

const mat4f& Renderable::GetTransform() const {
  return transform_;
}

void Renderable::SetMeshes(std::span<const Mesh*> meshes,
                           GetTransformFn get_transform_fn) {
  if (!parts_.empty()) {
    mju_error("Cannot set meshes for renderable with multiple parts.");
  }

  get_transform_fn_ = get_transform_fn;
  for (int i = 0; i < meshes.size(); ++i) {
    Part& part = parts_.emplace_back();
    part.mesh = meshes[i];
    part.elem_offset = 0;
    part.elem_count = part.mesh->GetFilamentIndexBuffer()->getIndexCount();
    InitPartEntity(part);
  }
}

void Renderable::AddToScene(filament::Scene* scene) {
  if (assigned_scene_) {
    if (assigned_scene_ != scene) {
      mju_error("Cannot add renderable to multiple scenes.");
    }
    // Entities are already added to the scene.
    return;
  }
  for (Part& part : parts_) {
    scene->addEntity(part.entity);
  }
  assigned_scene_ = scene;
}

void Renderable::RemoveFromScene(filament::Scene* scene) {
  if (assigned_scene_ != scene) {
    mju_error("Attempting to remove renderable from wrong scene.");
  }
  for (Part& part : parts_) {
    scene->remove(part.entity);
  }
  assigned_scene_ = nullptr;
}

void Renderable::UpdateMaterial(const MaterialParams& params,
                                const MaterialTextures& textures) {
  material_params_ = params;
  material_textures_ = textures;

  AssignMaterial(DrawMode::Color, GetColorMaterialType());
  if (params_.shading_model == mjSHADING_MODEL_SCENE_OBJECT) {
    AssignMaterial(DrawMode::Depth, ObjectManager::kUnlitDepth);
    AssignMaterial(DrawMode::Segmentation, ObjectManager::kUnlitSegmentation);
  }

  for (int i = 0; i < kNumDrawModes; ++i) {
    if (instances_[i]) {
      UpdateMaterialInstance(instances_[i], material_params_,
                             material_textures_, object_mgr_);
    }
  }
  SetDrawMode(draw_mode_);
}

void Renderable::AssignMaterial(DrawMode mode,
                                ObjectManager::MaterialType material_type) {
  const int index = static_cast<int>(mode);

  filament::Material* material = object_mgr_->GetMaterial(material_type);
  if (instances_[index]) {
    if (instances_[index]->getMaterial() == material) {
      // The correct material is already assigned, do nothing.
      return;
    } else {
      GetEngine()->destroy(instances_[index]);
      instances_[index] = nullptr;
    }
  }
  if (material) {
    instances_[index] = material->createInstance();
  }
}

const MaterialParams& Renderable::GetMaterialParams() const {
  return material_params_;
}

const MaterialTextures& Renderable::GetMaterialTextures() const {
  return material_textures_;
}

void Renderable::SetDrawMode(DrawMode mode) {
  // Only SceneObjects support non-color draw modes.
  if (params_.shading_model != mjSHADING_MODEL_SCENE_OBJECT) {
    mode = DrawMode::Color;
  }

  filament::MaterialInstance* instance = instances_[static_cast<int>(mode)];
  if (instance) {
    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      filament::RenderableManager::Instance ri = rm.getInstance(part.entity);
      rm.setMaterialInstanceAt(ri, 0, instance);
    }
  }
  draw_mode_ = mode;
}

std::uint8_t Renderable::SetLayerMask(std::uint8_t mask) {
  std::uint8_t prev = layer_mask_;
  if (mask != layer_mask_) {
    layer_mask_ = mask;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setLayerMask(rm.getInstance(part.entity), 0xff, layer_mask_);
    }
  }
  return prev;
}

std::uint8_t Renderable::SetPriority(std::uint8_t priority) {
  std::uint8_t prev = priority_;
  if (priority != priority_) {
    priority_ = priority;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setPriority(rm.getInstance(part.entity), priority_);
    }
  }
  return prev;
}

std::uint16_t Renderable::SetBlendOrder(std::uint16_t blend_order) {
  std::uint16_t prev = blend_order_;
  if (blend_order != blend_order_) {
    blend_order_ = blend_order;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setBlendOrderAt(rm.getInstance(part.entity), 0, blend_order_);
    }
  }
  return prev;
}

void Renderable::SetCastShadows(bool cast_shadows) {
  if (cast_shadows_ != cast_shadows) {
    cast_shadows_ = cast_shadows;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
  for (Part& part : parts_) {
        rm.setCastShadows(rm.getInstance(part.entity), cast_shadows_);
    }
  }
}

void Renderable::SetReceiveShadows(bool receive_shadows) {
  if (receive_shadows_ != receive_shadows) {
    receive_shadows_ = receive_shadows;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setReceiveShadows(rm.getInstance(part.entity), receive_shadows_);
    }
  }
}

void Renderable::SetWireframe(bool wireframe) {
  static constexpr auto kWireframeType =
      filament::RenderableManager::PrimitiveType::LINES;

  if (wireframe != wireframe_) {
    wireframe_ = wireframe;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      filament::VertexBuffer* vertex_buffer = part.mesh->GetFilamentVertexBuffer();
      filament::IndexBuffer* index_buffer = part.mesh->GetFilamentIndexBuffer();
      rm.setGeometryAt(rm.getInstance(part.entity), 0,
                       wireframe_ ? kWireframeType : part.mesh->GetPrimitiveType(),
                       vertex_buffer, index_buffer, part.elem_offset,
                       part.elem_count);
    }
  }
}

ObjectManager::MaterialType Renderable::GetColorMaterialType() const {
  if (params_.shading_model == mjSHADING_MODEL_DECOR_LINES) {
    return ObjectManager::kUnlitLine;
  } else if (params_.shading_model == mjSHADING_MODEL_DECOR) {
    return ObjectManager::kUnlitDecor;
  } else if (params_.shading_model == mjSHADING_MODEL_UX) {
    return ObjectManager::kUnlitUi;
  } else if (material_textures_.orm) {
    return ObjectManager::kPbrPacked;
  } else if (material_textures_.metallic) {
    return ObjectManager::kPbr;
  } else if (material_textures_.roughness) {
    return ObjectManager::kPbr;
  } else if (material_params_.metallic >= 0) {
    return ObjectManager::kPbr;
  } else if (material_params_.roughness >= 0) {
    return ObjectManager::kPbr;
  }

  // Check to see if we're dealing with a mesh with texture coordinates.
  // `data_id` is the id of the mesh in model (i.e. the geom has mesh
  // geometry) and `mesh_texcoordadr` stores the address of the mesh uvs if
  // it has them.
  bool has_texcoords = false;
  if (!parts_.empty()) {
    const auto attribs = parts_[0].mesh->GetVertexAttributes();
    auto it = std::find(attribs.begin(), attribs.end(),
                        filament::VertexAttribute::UV0);
    has_texcoords = (it != attribs.end());
  }

  if (material_textures_.color == nullptr) {
    if (material_params_.color.a < 1.0f) {
      return ObjectManager::kPhongColorFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhongColorReflect;
    } else {
      return ObjectManager::kPhongColor;
    }
  } else if (material_textures_.color->GetFilamentTexture()->getTarget() ==
              filament::Texture::Sampler::SAMPLER_CUBEMAP) {
    if (material_params_.color.a < 1.0f) {
      return ObjectManager::kPhongCubeFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhongCubeReflect;
    } else {
      return ObjectManager::kPhongCube;
    }
  } else if (has_texcoords) {
    if (material_params_.color.a < 1.0f) {
      return ObjectManager::kPhong2dUvFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhong2dUvReflect;
    } else {
      return ObjectManager::kPhong2dUv;
    }
  } else {
    if (material_params_.color.a < 1.0f) {
      return ObjectManager::kPhong2dFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhong2dReflect;
    } else {
      return ObjectManager::kPhong2d;
    }
  }
}

filament::Engine* Renderable::GetEngine() { return object_mgr_->GetEngine(); }

}  // namespace mujoco
