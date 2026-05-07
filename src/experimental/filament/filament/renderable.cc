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
#include <numbers>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament_util.h"
#include "experimental/filament/filament/builtins.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;

// An arbitrary scale factor for arrows.
static constexpr float kArrowScale = 1.f / 6.f;
static constexpr float kArrowHeadSize = 1.75f;

Renderable::Renderable(FilamentContext* ctx, const mjrRenderableParams& params)
    : object_mgr_(ctx->GetObjectManager()), params_(params) {
  mjr_defaultMaterialParams(&material_params_);
  mjr_defaultMaterialTextures(&material_textures_);
}

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
  for (int i = 0; i < mjNUM_DRAW_MODES; ++i) {
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
  builder.castShadows(params_.cast_shadows);
  builder.receiveShadows(params_.receive_shadows);
  builder.layerMask(0xff, params_.layer_mask);
  builder.priority(params_.priority);
  builder.blendOrder(0, params_.blend_order);
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

void Renderable::AppendMesh(const Mesh* mesh) {
  Part& part = parts_.emplace_back();
  part.mesh = Mesh::downcast(mesh);
  part.elem_offset = 0;
  part.elem_count = part.mesh->GetFilamentIndexBuffer()->getIndexCount();
  InitPartEntity(part);
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

void Renderable::UpdateMaterial(const mjrMaterialParams& params,
                                const mjrMaterialTextures& textures) {
  material_params_ = params;
  material_textures_ = textures;

  AssignMaterial(mjDRAW_MODE_COLOR, GetColorMaterialType());
  if (params_.shading_model == mjSHADING_MODEL_SCENE_OBJECT) {
    AssignMaterial(mjDRAW_MODE_DEPTH, ObjectManager::kUnlitDepth);
    AssignMaterial(mjDRAW_MODE_SEGMENTATION, ObjectManager::kUnlitSegmentation);
  }

  for (int i = 0; i < mjNUM_DRAW_MODES; ++i) {
    if (instances_[i]) {
      UpdateMaterialInstance(instances_[i], material_params_,
                             material_textures_, object_mgr_);
    }
  }
  SetDrawMode(draw_mode_);
}

void Renderable::AssignMaterial(mjrDrawMode mode,
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

const mjrMaterialParams& Renderable::GetMaterialParams() const {
  return material_params_;
}

const mjrMaterialTextures& Renderable::GetMaterialTextures() const {
  return material_textures_;
}

void Renderable::SetDrawMode(mjrDrawMode mode) {
  // Only SceneObjects support non-color draw modes.
  if (params_.shading_model != mjSHADING_MODEL_SCENE_OBJECT) {
    mode = mjDRAW_MODE_COLOR;
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
  std::uint8_t prev = params_.layer_mask;
  if (mask != params_.layer_mask) {
    params_.layer_mask = mask;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setLayerMask(rm.getInstance(part.entity), 0xff, params_.layer_mask);
    }
  }
  return prev;
}

std::uint8_t Renderable::SetPriority(std::uint8_t priority) {
  std::uint8_t prev = params_.priority;
  if (priority != params_.priority) {
    params_.priority = priority;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setPriority(rm.getInstance(part.entity), params_.priority);
    }
  }
  return prev;
}

std::uint16_t Renderable::SetBlendOrder(std::uint16_t blend_order) {
  std::uint16_t prev = params_.blend_order;
  if (blend_order != params_.blend_order) {
    params_.blend_order = blend_order;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setBlendOrderAt(rm.getInstance(part.entity), 0, params_.blend_order);
    }
  }
  return prev;
}

void Renderable::SetCastShadows(bool cast_shadows) {
  if (params_.cast_shadows != cast_shadows) {
    params_.cast_shadows = cast_shadows;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
  for (Part& part : parts_) {
        rm.setCastShadows(rm.getInstance(part.entity), params_.cast_shadows);
    }
  }
}

void Renderable::SetReceiveShadows(bool receive_shadows) {
  if (params_.receive_shadows != receive_shadows) {
    params_.receive_shadows = receive_shadows;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      rm.setReceiveShadows(rm.getInstance(part.entity), params_.receive_shadows);
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
  const Texture* color_texture = Texture::downcast(material_textures_.color);
  if (!parts_.empty()) {
    const auto attribs = parts_[0].mesh->GetVertexAttributes();
    auto it = std::find(attribs.begin(), attribs.end(),
                        filament::VertexAttribute::UV0);
    has_texcoords = (it != attribs.end());
  }

  if (color_texture == nullptr) {
    if (material_params_.color[3] < 1.0f) {
      return ObjectManager::kPhongColorFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhongColorReflect;
    } else {
      return ObjectManager::kPhongColor;
    }
  } else if (color_texture->GetTarget() == mjTEXTURE_CUBE) {
    if (material_params_.color[3] < 1.0f) {
      return ObjectManager::kPhongCubeFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhongCubeReflect;
    } else {
      return ObjectManager::kPhongCube;
    }
  } else if (has_texcoords) {
    if (material_params_.color[3] < 1.0f) {
      return ObjectManager::kPhong2dUvFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhong2dUvReflect;
    } else {
      return ObjectManager::kPhong2dUv;
    }
  } else {
    if (material_params_.color[3] < 1.0f) {
      return ObjectManager::kPhong2dFade;
    } else if (material_params_.reflective) {
      return ObjectManager::kPhong2dReflect;
    } else {
      return ObjectManager::kPhong2d;
    }
  }
}

void Renderable::SetGeomMesh(mjtGeom type, int nstack, int nslice, int nquad) {
  Builtins* builtins = object_mgr_->GetBuiltins(nstack, nslice, nquad);

  switch (type) {
    case mjGEOM_PLANE:
      AppendMesh(builtins->Plane());
      break;
    case mjGEOM_SPHERE:
      AppendMesh(builtins->Sphere());
      break;
    case mjGEOM_ELLIPSOID:
      AppendMesh(builtins->Sphere());
      break;
    case mjGEOM_BOX:
      AppendMesh(builtins->Box());
      break;
    case mjGEOM_CAPSULE:
      // Capsules are a tube with two domes at the ends.
      AppendMesh(builtins->Tube());
      AppendMesh(builtins->Dome());
      AppendMesh(builtins->Dome());

      get_transform_fn_ = [](int index, const Trs& trs) {
        // We apply an inverse scale to the domes to counteract the capsule's
        // overall scale so that the domes remain spherical in shape.
        const float xz_size = 0.5f * (trs.size.x + trs.size.y);
        if (index == 0) {
          return trs.ToTransform();
        } else if (index == 1) {
          // Move the first dome to the top of the capsule.
          mat4f top = mat4f(trs.rotation, trs.translation);
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(float3{trs.size.x, trs.size.y, xz_size});
          return top;
        } else if (index == 2) {
          // Move the second dome to the bottom of the capsule and rotate it 180
          // degrees so that it's facing the right way.
          mat4f bottom = mat4f(trs.rotation, trs.translation);
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          bottom *= mat4f::scaling(float3{trs.size.x, trs.size.y, xz_size});
          return bottom;
        } else {
          mju_error("Invalid index for capsule geom: %d (expected [0,2])", index);
          return trs.ToTransform();
        }
      };
      break;
    case mjGEOM_CYLINDER:
      // Cylinders are a tube with two disks at the ends.
      AppendMesh(builtins->Tube());
      AppendMesh(builtins->Disk());
      AppendMesh(builtins->Disk());

      get_transform_fn_ = [](int index, const Trs& trs) {
        if (index == 0) {
          return trs.ToTransform();
        } else if (index == 1) {
          // Move the first disk to the top of the cylinder.
          mat4f top = mat4f(trs.rotation, trs.translation);
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(trs.size);
          return top;
        } else if (index == 2) {
          // Move the second disk to the bottom of the cylinder. Rotate the disk
          // 180 degrees so that the normals point outwards.
          mat4f bottom = mat4f(trs.rotation, trs.translation);
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          bottom *= mat4f::scaling(trs.size);
          return bottom;
        } else {
          mju_error("Invalid index for cylinder geom: %d (expected [0,2])", index);
          return trs.ToTransform();
        }
      };
      break;
    case mjGEOM_ARROW:
      AppendMesh(builtins->Tube());
      AppendMesh(builtins->Cone());
      AppendMesh(builtins->Disk());
      AppendMesh(builtins->Disk());

      get_transform_fn_ = [](int index, const Trs& trs) {
        mat4f base = mat4f(trs.rotation, trs.translation);
        base *= mat4f::scaling(float3{1, 1, kArrowScale});
        base *= mat4f::translation(float3{0, 0, trs.size.z});
        if (index == 0) {
          return base * mat4f::scaling(trs.size);
        } else if (index == 1) {
          mat4f top = base;
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top * mat4f::scaling(trs.size);
        } else if (index == 2) {
          mat4f top_disk = base;
          top_disk *= mat4f::translation(float3{0, 0, trs.size.z});
          top_disk *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          top_disk *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top_disk * mat4f::scaling(trs.size);
        } else if (index == 3) {
          mat4f bottom = base;
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          return bottom * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow geom: %d (expected [0,3])", index);
          return trs.ToTransform();
        }
      };
      break;
    case mjGEOM_ARROW1:
      AppendMesh(builtins->Tube());
      AppendMesh(builtins->Cone());
      AppendMesh(builtins->Disk());

      get_transform_fn_ = [](int index, const Trs& trs) {
      mat4f base = mat4f(trs.rotation, trs.translation);
      base *= mat4f::scaling(float3{1, 1, kArrowScale});
      base *= mat4f::translation(float3{0, 0, trs.size.z});
        if (index == 0) {
          return base * mat4f::scaling(trs.size);
        } else if (index == 1) {
          mat4f top = base;
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          return top * mat4f::scaling(trs.size);
        } else if (index == 2) {
          mat4f bottom = base;
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          return bottom * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow1 geom: %d (expected [0,2])", index);
          return trs.ToTransform();
        }
      };
      break;
    case mjGEOM_ARROW2:
      AppendMesh(builtins->Tube());
      AppendMesh(builtins->Cone());
      AppendMesh(builtins->Cone());
      AppendMesh(builtins->Disk());
      AppendMesh(builtins->Disk());

      get_transform_fn_ = [](int index, const Trs& trs) {
        mat4f base = mat4f(trs.rotation, trs.translation);
        base *= mat4f::scaling(float3{1, 1, kArrowScale});
        base *= mat4f::translation(float3{0, 0, trs.size.z});
        if (index == 0) {
          return base * mat4f::scaling(trs.size);
        } else if (index == 1) {
          mat4f top = base;
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top * mat4f::scaling(trs.size);
        } else if (index == 2) {
          mat4f bottom = base;
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          bottom *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return bottom * mat4f::scaling(trs.size);
        } else if (index == 3) {
          mat4f top_disk = base;
          top_disk *= mat4f::translation(float3{0, 0, trs.size.z});
          top_disk *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          top_disk *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top_disk * mat4f::scaling(trs.size);
        } else if (index == 4) {
          mat4f bottom_disk = base;
          bottom_disk *= mat4f::translation(float3{0, 0, -trs.size.z});
          return bottom_disk * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow2 geom: %d (expected [0,4])", index);
          return trs.ToTransform();
        }
      };
      break;
    case mjGEOM_LINE:
      AppendMesh(builtins->Line());
      break;
    case mjGEOM_LINEBOX:
      AppendMesh(builtins->LineBox());
      break;
    case mjGEOM_TRIANGLE:
      AppendMesh(builtins->Triangle());
      break;
    default:
      mju_error("Unsupported geom type: %d", type);
      break;
  }
}

filament::Engine* Renderable::GetEngine() { return object_mgr_->GetEngine(); }

}  // namespace mujoco
