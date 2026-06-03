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
#include <deque>
#include <functional>
#include <span>
#include <unordered_map>
#include <vector>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <math/mat3.h>
#include <math/mat4.h>
#include <math/vec3.h>
#include <utils/Entity.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/reflection_manager.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// A Renderable is effectively two things: a mesh and a material.
//
// The mesh describes the surface geometry of the object and the material
// describes how that surface interacts with light (i.e. the color of each point
// on the surface).
class Renderable : public mjrRenderable {
 public:
  Renderable(filament::Engine* engine, const mjrRenderableParams& params,
             ObjectManager* object_mgr);
  ~Renderable() noexcept;

  Renderable(const Renderable&) = delete;
  Renderable& operator=(const Renderable&) = delete;

  // Sets the mesh of this renderable. The elem_offset and elem_count parameters
  // can be used to specify a submesh within the mesh. If elem_count is 0,
  // assumes the entire mesh should be appended.
  void SetMesh(const Mesh* mesh, int elem_offset = 0, int elem_count = 0);

  // Sets the mesh of this renderable to a built-in mesh based on the geom type.
  void SetGeomMesh(mjtGeom type, int nstack, int nslice, int nquad);

  // Sets the position and rotation of this renderable.
  void SetTransform(const filament::math::float3& position,
                    const filament::math::mat3f& rotation);

  // Sets the size of this renderable. Note: this is effectively the same as a
  // scale for most renderables. However, for e.g. capsules, the spherical ends
  // are not scaled and remain fixed in size.
  void SetSize(const filament::math::float3& size);

  // Returns the current transform of this renderable.
  const filament::math::mat4f& GetTransform() const;

  // Sets the layer mask for this renderable. Layer masks can be used to
  // show/hide groups of renderables in scenes. Returns the previous layer mask.
  std::uint8_t SetLayerMask(std::uint8_t mask);

  // Sets the draw priority this renderable. The priority determines the order
  // in which renderables are rendered. Returns the previous priority.
  std::uint8_t SetPriority(std::uint8_t priority);

  // Sets the blend order for this renderable. This determines the order in
  // which transparent renderables are blended together. Returns the previous
  // blend order.
  std::uint16_t SetBlendOrder(std::uint16_t blend_order);

  // Disables this renderable from casting shadows.
  void SetCastShadows(bool cast_shadows);

  // Disables this renderable from receiving shadows.
  void SetReceiveShadows(bool receive_shadows);

  // If true, forces this renderable to use wireframe rendering.
  void SetWireframe(bool wireframe);

  // Adds this renderable to the filament Scene.
  void AddToScene(filament::Scene* scene);

  // Removes this renderable from the filament Scene.
  void RemoveFromScene(filament::Scene* scene);

  // Updates the parameters and textures of the material for this renderable.
  void UpdateMaterial(const mjrMaterial& material);

  // Returns this renderable's current material.
  const mjrMaterial& GetMaterial() const;

  // Prepares the material instances for the given render requests.
  void Prepare(std::span<const mjrRenderRequest*> requests,
               ReflectionManager* reflection_mgr);

  // Binds the material instance for the given render request.
  void BindMaterialInstance(const mjrRenderRequest& request);

  static Renderable* downcast(mjrRenderable* renderable) {
    return static_cast<Renderable*>(renderable);
  }
  static const Renderable* downcast(const mjrRenderable* renderable) {
    return static_cast<const Renderable*>(renderable);
  }

 private:
  // In most cases, a Renderable will be composed of a single filament Entity.
  // However, for some built-in geom types (e.g. capsules) we compose the
  // renderable out of multiple Entities.
  struct Part {
    utils::Entity entity;
    const Mesh* mesh = nullptr;
    int elem_offset = 0;
    int elem_count = 0;
  };

  // A tuple of translation, rotation, and size.
  struct Trs {
    filament::math::float3 translation{0.0f, 0.0f, 0.0f};
    filament::math::mat3f rotation;
    filament::math::float3 size{1.0f, 1.0f, 1.0f};
    filament::math::mat4f ToTransform() const {
      return filament::math::mat4f(rotation, translation) *
            filament::math::mat4f::scaling(size);
    }
  };

  // For each render request in a batch, we may need to render the object
  // differently. However, we cannot change the material instance during the
  // actual frame rendering (i.e. between beginFrame/endFrame). Instead, we can
  // switch out the material instance entirely for each request. We keep track
  // of which instance, as well as other render state, to use with each render
  // request.
  struct DrawState {
    MaterialKey material_key;
    bool cast_shadows = true;
    bool receive_shadows = true;
    bool wireframe = false;
  };

  // When composing a multi-part renderable, each Entity will have its own
  // transform offset based on the transform of the Renderable itself.
  using GetTransformFn = std::function<filament::math::mat4f(int, const Trs&)>;

  void AppendMesh(const Mesh* mesh);
  void InitPartEntity(Part& part);
  void UpdateTransform();

  MaterialKey PrepareMaterialInstance(const mjrMaterial& material,
                                      mjrDrawMode draw_mode);
  void SetMaterialInstance(MaterialKey key);

  filament::Engine* GetEngine();

  ObjectManager* object_mgr_;
  mjrRenderableParams params_;
  mjtGeom geom_type_ = mjGEOM_NONE;
  mjrMaterial material_;
  std::unordered_map<MaterialKey, filament::MaterialInstance*> instances_;
  std::deque<DrawState> draw_queue_;
  DrawState curr_state_;
  filament::Scene* assigned_scene_ = nullptr;
  std::vector<Part> parts_;
  filament::math::mat4f transform_;
  GetTransformFn get_transform_fn_;
  Trs trs_;
  bool infinite_plane_ = false;
  bool wireframe_ = false;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLE_H_
