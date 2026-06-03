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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_REFLECTION_MANAGER_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_REFLECTION_MANAGER_H_

#include <memory>
#include <vector>

#include <filament/Engine.h>
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Manages the allocation of RenderTargets for reflective surfaces.
class ReflectionManager {
 public:
  ReflectionManager(filament::Engine* engine);
  ~ReflectionManager();

  ReflectionManager(const ReflectionManager&) = delete;
  ReflectionManager& operator=(const ReflectionManager&) = delete;

  // Registers a Renderable as being reflective. Internally, this function will
  // create a RenderTarget of the given size and return the texture that the
  // Renderable can use as its reflection.
  mjrTexture* Register(mjrRenderable* renderable, int width, int height);

  // Clears all previously registered renderables. This should be called at the
  // beginning of a frame.
  void ClearRenderables();

  // Returns the number of registered renderables
  int GetNumRenderables() const;

  // Returns the renderable at the given index.
  mjrRenderable* GetRenderable(int index) const;

  // Returns the RenderTarget at the given index.
  const RenderTarget* GetRenderTarget(int index) const;

 private:
  filament::Engine* engine_;
  std::vector<mjrRenderable*> renderables_;
  std::vector<std::unique_ptr<RenderTarget>> targets_;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_REFLECTION_MANAGER_H_
