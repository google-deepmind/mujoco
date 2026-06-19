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

#include "render/filament/core/renderable.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <numbers>
#include <span>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <math/TMatHelpers.h>
#include <math/TVecHelpers.h>
#include <math/mat3.h>
#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "engine/engine_vis_visualize.h"
#include "render/filament/core/builtins.h"
#include "render/filament/core/material_manager.h"
#include "render/filament/core/mesh.h"
#include "render/filament/core/reflection_manager.h"
#include "render/filament/support/filament_util.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat3f;
using filament::math::mat4f;

// An arbitrary scale factor for arrows.
static constexpr float kArrowScale = 1.f / 6.f;
static constexpr float kArrowHeadSize = 1.75f;

Renderable::Renderable(filament::Engine* engine,
                       const mjrfRenderableParams& params,
                       MaterialManager* material_mgr)
    : material_mgr_(material_mgr), params_(params) {
  mjrf_defaultMaterial(&material_);
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
}

void Renderable::SetMesh(const Mesh* mesh, int elem_offset, int elem_count) {
  if (mesh == nullptr) {
    mju_error("Cannot set mesh to nullptr.");
  }

  // We use MESH, even though it could be any mesh-like geom type, e.g.
  // heightfields, flex, skin, sdf, etc.
  geom_type_ = mjGEOM_MESH;

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
  builder.geometry(0, part.mesh->GetPrimitiveType(), vertex_buffer,
                   index_buffer, part.elem_offset, part.elem_count);
  if (part.mesh->HasBounds()) {
    builder.boundingBox(part.mesh->GetBounds());
  } else {
    builder.culling(false);
  }
  builder.castShadows(params_.cast_shadows);
  builder.receiveShadows(params_.receive_shadows);
  builder.blendOrder(0, params_.blend_order);
  builder.screenSpaceContactShadows(true);

  builder.build(*GetEngine(), part.entity);
  if (assigned_scene_) {
    assigned_scene_->addEntity(part.entity);
  }
}

void Renderable::SetTransform(const float3& position, const mat3f& rotation) {
  trs_.translation = position;
  trs_.rotation = rotation;
  UpdateTransform();
}

void Renderable::SetSize(const float3& size) {
  trs_.size = size;
  UpdateTransform();
}

void Renderable::UpdateTransform() {
  if (parts_.empty()) {
    transform_ = trs_.ToTransform();
    return;
  }

  filament::TransformManager& tm = GetEngine()->getTransformManager();
  if (geom_type_ == mjGEOM_PLANE && (trs_.size[0] <= 0 || trs_.size[1] <= 0)) {
    infinite_plane_ = true;
    const mat4f transform =
        filament::math::mat4f(trs_.rotation, trs_.translation);
    for (Part& part : parts_) {
      tm.setTransform(tm.getInstance(part.entity), transform);
    }
  } else if (get_transform_fn_) {
    for (int i = 0; i < parts_.size(); ++i) {
      const mat4f transform = get_transform_fn_(i, trs_);
      tm.setTransform(tm.getInstance(parts_[i].entity), transform);
    }
  } else {
    const mat4f transform = trs_.ToTransform();
    for (Part& part : parts_) {
      tm.setTransform(tm.getInstance(part.entity), transform);
    }
  }
  transform_ = tm.getTransform(tm.getInstance(parts_[0].entity));
}

const mat4f& Renderable::GetTransform() const { return transform_; }

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

void Renderable::UpdateMaterial(const mjrfMaterial& material) {
  uint8_t layer_mask_ = kLayerMask_Object;
  if (material.decor_ux) {
    layer_mask_ = kLayerMask_Decor;
  }
  if (material.selected) {
    layer_mask_ |= kLayerMask_Outline;
  }
  SetLayerMask(layer_mask_);
  material_ = material;
}

const mjrfMaterial& Renderable::GetMaterial() const { return material_; }

void Renderable::Prepare(std::span<const mjrfRenderRequest*> requests,
                         ReflectionManager* reflection_mgr) {
  // We assume BindMaterialInstance will be called with the same requests in
  // the same order. As such, we'll just store the draw state in a deque rather
  // than trying to perform any kind of matching with the requests.
  draw_queue_.clear();
  curr_state_ = DrawState();

  for (const mjrfRenderRequest* request : requests) {
    DrawState draw_state;
    mjrfMaterial material = material_;
    material.selected = false;

    draw_state.wireframe = (request->draw_mode == mjDRAW_MODE_WIREFRAME);
    if (material.decor_ux) {
      draw_state.cast_shadows = false;
      draw_state.receive_shadows = false;
    }

    if (geom_type_ == mjGEOM_PLANE) {
      const float3 camera_pos = ReadFloat3(request->camera.pos);
      const float3 position = transform_[3].xyz;
      const float3 forward = transform_[2].xyz;
      const bool is_behind_camera = dot(camera_pos - position, forward) < 0;
      if (is_behind_camera) {
        material.color[3] *= 0.3;
        draw_state.receive_shadows = false;
      }
    }

    if (request->draw_mode == mjDRAW_MODE_SEGMENTATION_BY_COLOR) {
      constexpr double phi1 = 1.61803398874989484820;  // Cached Phi(1).
      constexpr double coef1 = 1.0 / phi1;
      const double index = static_cast<double>(material.segmentation_id);
      const double sample = std::fmod(0.5 + coef1 * index, 1.0);
      material.segmentation_id = 0x01000000 * sample;
    } else if (request->draw_mode != mjDRAW_MODE_SEGMENTATION_BY_ID) {
      // Clear out the segmentation color in order to take advantage of shared
      // materials.
      material.segmentation_id = 0;
    }

    if (request->draw_mode == mjDRAW_MODE_ISLANDS &&
        material.sleep_state != mjS_STATIC) {
      float hue = 1.0f;
      float saturation = 0.0f;
      float value = 0.7f;
      if (material.island_id >= 0) {
        const double h = static_cast<double>(material.island_id) + 1.0;
        hue = mju_Halton(h, 7);
        saturation = 0.5 + 0.5 * mju_Halton(h, 3);
        value = 0.6 + 0.4 * mju_Halton(h, 5);
      }
      if (material.sleep_state == mjS_ASLEEP) {
        value *= 0.6;
        saturation *= 0.7;
      }
      hsv2rgb(material.color, hue, saturation, value);
      material.color[3] = 1;

      // Remove textures so they don't interfere with the island color.
      material.color_texture = nullptr;
      material.metallic_texture = nullptr;
      material.roughness_texture = nullptr;
      material.occlusion_texture = nullptr;
      material.orm_texture = nullptr;
    }

    if (request->draw_mode == mjDRAW_MODE_DEFAULT_NO_TEXTURES) {
      material.color_texture = nullptr;
      material.metallic_texture = nullptr;
      material.roughness_texture = nullptr;
      material.occlusion_texture = nullptr;
      material.orm_texture = nullptr;
    }

    const bool reflective = request->draw_mode == mjDRAW_MODE_DEFAULT &&
                            request->enable_reflections &&
                            material.reflectance > 0.0;
    if (reflective) {
      material.reflection_texture = reflection_mgr->Register(
          this, request->viewport.width, request->viewport.height);
    }

    if (material.selected) {
      material.emissive += 0.3f;  // vis->global.glow
    }

    const Mesh* mesh = !parts_.empty() ? parts_[0].mesh : nullptr;
    draw_state.material_key = material_mgr_->PrepareMaterialInstance(
        material, static_cast<mjrDrawMode>(request->draw_mode), geom_type_,
        mesh);
    draw_queue_.push_back(draw_state);
  }
}

void Renderable::BindMaterialInstance(const mjrfRenderRequest& request) {
  if (draw_queue_.empty()) {
    mju_error("No material instances to bind.");
  }

  if (geom_type_ == mjGEOM_PLANE && infinite_plane_) {
    // Emulate an infinite plane by recentering a large quad in world space
    // relative to the camera. We use the shared mjMAXPLANEGRID value as the
    // size of the quad to ensure the texture scaling matches.
    static constexpr float kInfiniteScale = 0.5f * mjMAXPLANEGRID;
    const mat4f scaling =
        mat4f::scaling(float3{kInfiniteScale, kInfiniteScale, 1.0f});

    const float3 camera_pos = ReadFloat3(request.camera.pos);
    const float3 plane_origin = transform_[3].xyz;
    const mat3f plane_rotation = transform_.upperLeft();

    const float3 vec = camera_pos - plane_origin;
    const float3 plane_x = normalize(plane_rotation[0]);
    const float3 plane_y = normalize(plane_rotation[1]);

    // Project camera position onto the plane's local XY axes.
    float dx = dot(vec, plane_x);
    float dy = dot(vec, plane_y);

    // Quantize based on uv_scale.
    const float tile_size[] = {kInfiniteScale / material_.uv_scale[0],
                               kInfiniteScale / material_.uv_scale[1]};
    dx = tile_size[0] * mju_round(dx / tile_size[0]);
    dy = tile_size[1] * mju_round(dy / tile_size[1]);

    // Calculate the new center quad as a displacement from the plane origin.
    const float3 displacement = dx * plane_x + dy * plane_y;
    const float3 center = plane_origin + displacement;

    const mat4f transform = mat4f(plane_rotation, center) * scaling;
    filament::TransformManager& tm = GetEngine()->getTransformManager();
    for (Part& part : parts_) {
      tm.setTransform(tm.getInstance(part.entity), transform);
    }
  }

  const DrawState& state = draw_queue_.front();
  SetCastShadows(state.cast_shadows);
  SetReceiveShadows(state.receive_shadows);
  SetWireframe(state.wireframe);
  SetMaterialInstance(state.material_key);
  curr_state_ = state;
  draw_queue_.pop_front();
}

MaterialManager::MaterialKey Renderable::SetMaterialInstance(
    MaterialManager::MaterialKey key) {
  MaterialManager::MaterialKey prev = curr_state_.material_key;
  if (key != curr_state_.material_key) {
    filament::MaterialInstance* instance = material_mgr_->GetInstance(key);
    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      filament::RenderableManager::Instance ri = rm.getInstance(part.entity);
      rm.setMaterialInstanceAt(ri, 0, instance);
    }
  }
  return prev;
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
      rm.setReceiveShadows(rm.getInstance(part.entity),
                           params_.receive_shadows);
    }
  }
}

void Renderable::SetWireframe(bool wireframe) {
  static constexpr auto kWireframeType =
      filament::RenderableManager::PrimitiveType::LINES;

  if (wireframe != curr_state_.wireframe) {
    curr_state_.wireframe = wireframe;

    filament::RenderableManager& rm = GetEngine()->getRenderableManager();
    for (Part& part : parts_) {
      filament::VertexBuffer* vertex_buffer =
          part.mesh->GetFilamentVertexBuffer();
      filament::IndexBuffer* index_buffer = part.mesh->GetFilamentIndexBuffer();
      rm.setGeometryAt(
          rm.getInstance(part.entity), 0,
          wireframe ? kWireframeType : part.mesh->GetPrimitiveType(),
          vertex_buffer, index_buffer, part.elem_offset, part.elem_count);
    }
  }
}

void Renderable::SetGeomMesh(mjtGeom type, int nstack, int nslice, int nquad) {
  Builtins* builtins =
      material_mgr_->GetObjectManager()->GetBuiltins(nstack, nslice, nquad);
  geom_type_ = type;

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
          mju_error("Invalid index for capsule geom: %d (expected [0,2])",
                    index);
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
          mju_error("Invalid index for cylinder geom: %d (expected [0,2])",
                    index);
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
          top_disk *=
              mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
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
          mju_error("Invalid index for arrow1 geom: %d (expected [0,2])",
                    index);
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
          bottom *=
              mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return bottom * mat4f::scaling(trs.size);
        } else if (index == 3) {
          mat4f top_disk = base;
          top_disk *= mat4f::translation(float3{0, 0, trs.size.z});
          top_disk *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          top_disk *=
              mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top_disk * mat4f::scaling(trs.size);
        } else if (index == 4) {
          mat4f bottom_disk = base;
          bottom_disk *= mat4f::translation(float3{0, 0, -trs.size.z});
          return bottom_disk * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow2 geom: %d (expected [0,4])",
                    index);
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

filament::Engine* Renderable::GetEngine() { return material_mgr_->GetEngine(); }

}  // namespace mujoco
