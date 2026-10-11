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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_CORE_REFLECTION_MANAGER_H_
#define MUJOCO_SRC_RENDER_FILAMENT_CORE_REFLECTION_MANAGER_H_

#include <memory>
#include <vector>

#include <filament/Engine.h>
#include <math/vec3.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/core/material_manager.h"
#include "render/filament/core/mesh.h"
#include "render/filament/core/render_target.h"

namespace mujoco {

// Manages the allocation of RenderTargets for reflective surfaces.
class ReflectionManager {
 public:
  ReflectionManager(filament::Engine* engine, MaterialManager* material_mgr);
  ~ReflectionManager();

  ReflectionManager(const ReflectionManager&) = delete;
  ReflectionManager& operator=(const ReflectionManager&) = delete;

  // Registers a Renderable as being reflective. Internally, this function will
  // create a RenderTarget for the reflection pass and prepare the reflective
  // material instance.
  void Register(mjrfRenderable* renderable, const Mesh* mesh,
                mjrfMaterial material, mjtGeom geom_type,
                const filament::math::float3& refl_normal,
                const mjrfRenderRequest* request);

  // Clears all previously registered renderables. This should be called at the
  // beginning of a frame.
  void ClearRenderables();

  // Returns the number of registered renderables
  int GetNumRenderables() const;

  // Returns the renderable at the given index.
  mjrfRenderable* GetRenderable(int index) const;

  // Returns the RenderTarget at the given index.
  const RenderTarget* GetRenderTarget(int index) const;

  // Returns the reflective material key at the given index.
  MaterialManager::MaterialKey GetMaterialKey(int index) const;

 private:
  struct Entry {
    mjrfRenderable* renderable = nullptr;
    MaterialManager::MaterialKey material_key = 0;
  };

  filament::Engine* engine_;
  MaterialManager* material_mgr_;
  std::vector<Entry> entries_;
  std::vector<std::unique_ptr<RenderTarget>> targets_;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_RENDER_FILAMENT_CORE_REFLECTION_MANAGER_H_
