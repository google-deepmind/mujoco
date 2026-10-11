// Copyright 2026 DeepMind Technologies Limited
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

#include "render/filament/core/reflection_manager.h"

#include <memory>

#include <filament/Engine.h>
#include <math/mat4.h>
#include <math/vec3.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/core/material_manager.h"
#include "render/filament/core/mesh.h"
#include "render/filament/core/render_target.h"
#include "render/filament/support/filament_util.h"

namespace mujoco {

ReflectionManager::ReflectionManager(filament::Engine* engine,
                                     MaterialManager* material_mgr)
    : engine_(engine), material_mgr_(material_mgr) {}

ReflectionManager::~ReflectionManager() {}

void ReflectionManager::ClearRenderables() { entries_.clear(); }

void ReflectionManager::Register(mjrfRenderable* renderable, const Mesh* mesh,
                                 mjrfMaterial material, mjtGeom geom_type,
                                 const filament::math::float3& refl_normal,
                                 const mjrfRenderRequest* request) {
  const int width = request->viewport.width;
  const int height = request->viewport.height;
  if (targets_.size() == entries_.size()) {
    mjrfRenderTargetConfig config;
    mjrf_defaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_RGBA8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    targets_.push_back(std::make_unique<RenderTarget>(engine_, config));
  }
  RenderTarget* target = targets_[entries_.size()].get();
  target->Prepare(width, height);

  const auto draw_mode = static_cast<mjrDrawMode>(request->draw_mode);
  const filament::math::mat4f view_proj =
      GetReflectionViewProjectionMatrix(request->camera, width, height);
  WriteMat4(material.reflection_view_proj, view_proj);
  material.reflection_texture = target->GetColorTexture();
  material.reflection_normal[0] = refl_normal.x;
  material.reflection_normal[1] = refl_normal.y;
  material.reflection_normal[2] = refl_normal.z;
  const MaterialManager::MaterialKey material_key =
      material_mgr_->PrepareMaterialInstance(material, draw_mode, geom_type,
                                             mesh);
  entries_.push_back({.renderable = renderable, .material_key = material_key});
}

int ReflectionManager::GetNumRenderables() const { return entries_.size(); }

mjrfRenderable* ReflectionManager::GetRenderable(int index) const {
  return entries_[index].renderable;
}

const RenderTarget* ReflectionManager::GetRenderTarget(int index) const {
  return targets_[index].get();
}

MaterialManager::MaterialKey ReflectionManager::GetMaterialKey(
    int index) const {
  return entries_[index].material_key;
}

}  // namespace mujoco
