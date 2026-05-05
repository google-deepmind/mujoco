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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_CPP_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_CPP_H_

#include <memory>

#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// A unique pointer to a mujoco object.
template <typename T>
using UniquePtr = std::unique_ptr<T, void (*)(T*)>;

inline UniquePtr<mjrTexture> CreateTexture(mjrfContext* ctx,
                                           const mjrTextureConfig& config) {
  mjrTexture* texture = mjrf_createTexture(ctx, &config);
  return UniquePtr<mjrTexture>(texture, mjrf_destroyTexture);
}

inline UniquePtr<mjrMesh> CreateMesh(mjrfContext* ctx,
                                     const mjrMeshData& data) {
  mjrMesh* mesh = mjrf_createMesh(ctx, &data);
  return UniquePtr<mjrMesh>(mesh, mjrf_destroyMesh);
}

inline UniquePtr<mjrScene> CreateScene(mjrfContext* ctx,
                                       const mjrSceneParams& params) {
  mjrScene* scene = mjrf_createScene(ctx, &params);
  return UniquePtr<mjrScene>(scene, mjrf_destroyScene);
}

inline UniquePtr<mjrLight> CreateLight(mjrfContext* ctx,
                                       const mjrLightParams& params) {
  mjrLight* light = mjrf_createLight(ctx, &params);
  return UniquePtr<mjrLight>(light, mjrf_destroyLight);
}

inline UniquePtr<mjrRenderable> CreateRenderable(
    mjrfContext* ctx, const mjrRenderableParams& params) {
  mjrRenderable* renderable = mjrf_createRenderable(ctx, &params);
  return UniquePtr<mjrRenderable>(renderable, mjrf_destroyRenderable);
}

inline UniquePtr<mjrRenderTarget> CreateRenderTarget(
    mjrfContext* ctx, const mjrRenderTargetConfig& config) {
  mjrRenderTarget* render_target = mjrf_createRenderTarget(ctx, &config);
  return UniquePtr<mjrRenderTarget>(render_target, mjrf_destroyRenderTarget);
}

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_CPP_H_
