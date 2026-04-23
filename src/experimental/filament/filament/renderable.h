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
#include <functional>
#include <span>
#include <vector>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <math/mat4.h>
#include <utils/Entity.h>
#include "experimental/filament/filament/draw_mode.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

// The shading model (material) for a Renderable.
typedef enum mjrShadingModel_ {
  mjSHADING_MODEL_SCENE_OBJECT,
  mjSHADING_MODEL_DECOR,
  mjSHADING_MODEL_DECOR_LINES,
  mjSHADING_MODEL_UX,
} mjrShadingModel;

// Configuration parameters for a Renderable.
struct mjrRenderableParams {
  mjrShadingModel shading_model;
};

void mjr_defaultRenderableParams(mjrRenderableParams* params);

// A Renderable is effectively two things: a mesh and a material.
//
// The mesh describes the surface geometry of the object and the material
// describes how that surface interacts with light (i.e. the color of each point
// on the surface).
//
// Defining the mesh is easy; just call SetMesh.
//
// Defining a Material happens in two stages. First, the user specifies the
// ShadingModel to use for Rendering. This describes the overall intent of
// how the Renderable will appear (e.g. lit, unlit, wireframe, etc.). Next,
// the user specifies the MaterialParams and MaterialTextures to use with the
// ShadingModel. Its these properties that ultimately define the actual material
// of the Renderable.
class Renderable {
 public:
  // Default filament values for priority and layer mask.
  static constexpr std::uint8_t kDefaultPriority = 4;
  static constexpr std::uint8_t kDefaultLayerMask = 0x01;

  Renderable(ObjectManager* object_mgr, const mjrRenderableParams& params);
  ~Renderable() noexcept;

  Renderable(const Renderable&) = delete;
  Renderable& operator=(const Renderable&) = delete;

  // Sets the mesh of the renderable. The elem_offset and elem_count parameters
  // can be used to specify a submesh within the mesh. If elem_count is 0,
  // assumes the entire mesh should be appended.
  void SetMesh(const Mesh* mesh, int elem_offset = 0, int elem_count = 0);

  // Sets the transform of the renderable.
  void SetTransform(const Trs& trs);

  // Returns the current transform of the renderable.
  const filament::math::mat4f& GetTransform() const;

  // Sets multiple meshes for a renderable. Users can optionally provide a
  // function that will be used to compute the transform for each (sub)mesh
  // relative to the transform of the renderable itself. This allows users to
  // construct compound (but rigid) objects from multiple meshes.
  using GetTransformFn = std::function<filament::math::mat4f(int, const Trs&)>;
  void SetMeshes(std::span<const Mesh*> meshes,
                 GetTransformFn get_transform = nullptr);

  // Sets the layer mask for the managed filament Entities. Layer masks can be
  // used to show/hide the renderable in different views. Returns the previous
  // layer mask.
  std::uint8_t SetLayerMask(std::uint8_t mask);

  // Sets the priority for the managed filament Entities. The priority
  // determines the order in which renderables are rendered. Returns the
  // previous priority.
  std::uint8_t SetPriority(std::uint8_t priority);

  // Sets the blend order of the managed filament entities. This determines the
  // order in which renderables are blended together. Returns the previous blend
  // order.
  std::uint16_t SetBlendOrder(std::uint16_t blend_order);

  // Disables the renderable from casting shadows.
  void SetCastShadows(bool cast_shadows);

  // Disables the renderable from receiving shadows.
  void SetReceiveShadows(bool receive_shadows);

  // If true, forces all meshes to be rendered using Lines primitives.
  void SetWireframe(bool wireframe);

  // Adds the renderable to the given filament Scene.
  void AddToScene(filament::Scene* scene);

  // Removes the renderable from the given filament Scene.
  void RemoveFromScene(filament::Scene* scene);

  // Further defines the material of the renderable. Only applies to renderables
  // with a SceneObject shading model.
  void SetDrawMode(DrawMode mode);

  // Updates the parameters for the material.
  void UpdateMaterial(const mjrMaterialParams& params,
                      const mjrMaterialTextures& textures);

  // Returns the current material parameters.
  const mjrMaterialParams& GetMaterialParams() const;

  // Returns the current material textures.
  const mjrMaterialTextures& GetMaterialTextures() const;

  // Returns the filament Engine managing the renderables.
  filament::Engine* GetEngine();

 private:
  struct Part {
    utils::Entity entity;
    const Mesh* mesh = nullptr;
    int elem_offset = 0;
    int elem_count = 0;
  };

  void InitPartEntity(Part& part);

  void AssignMaterial(DrawMode mode, ObjectManager::MaterialType material_type);

  ObjectManager::MaterialType GetColorMaterialType() const;

  ObjectManager* object_mgr_;
  mjrRenderableParams params_;
  filament::MaterialInstance* instances_[kNumDrawModes] = {nullptr};
  mjrMaterialParams material_params_;
  mjrMaterialTextures material_textures_;
  DrawMode draw_mode_ = DrawMode::Color;
  filament::Scene* assigned_scene_ = nullptr;
  std::vector<Part> parts_;
  filament::math::mat4f transform_;
  GetTransformFn get_transform_fn_;
  std::uint8_t priority_ = kDefaultPriority;
  std::uint8_t layer_mask_ = kDefaultLayerMask;
  std::uint16_t blend_order_ = 0;
  bool wireframe_ = false;
  bool cast_shadows_ = true;
  bool receive_shadows_ = true;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDERABLE_H_
