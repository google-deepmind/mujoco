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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_PRIVATE_TYPES_H_
#define MUJOCO_SRC_RENDER_FILAMENT_PRIVATE_TYPES_H_

#include <mujoco/mjrfilament.h>

// IMPORTANT: These types are only to be used by implementations of the API.
// They should not be used by clients of the API. Clients should only use the
// opaque types defined in mjrfilament.h.
struct mjrfVtable_;

struct mjrfContext_ {
  const mjrfVtable_* vtable = nullptr;
};

struct mjrfTexture_ {
  const mjrfVtable_* vtable = nullptr;
};

struct mjrfMesh_ {
  const mjrfVtable_* vtable = nullptr;
};

struct mjrfScene_ {
  const mjrfVtable_* vtable = nullptr;
};

struct mjrfLight_ {
  const mjrfVtable_* vtable = nullptr;
};

struct mjrfRenderable_ {
  const mjrfVtable_* vtable = nullptr;
};

struct mjrfRenderTarget_ {
  const mjrfVtable_* vtable = nullptr;
};

// Virtual table for all functions of the mjrf API (except for createContext).
// An implementation-specific createContext function should create a context
// which contains a pointer to the vtable for all other functions specific to
// that implementation. Furthermore, all create functions should also attach the
// owning context's vtable to the created object. This ensures that all mjrf
// functions will call the correct implementation-specific functions. This
// assumes that the vtable is immutable for the lifetime of the context and all
// objects created by it. This redirection allows for multiple implementations
// of the mjrf API to exist in the same binary.
struct mjrfVtable_ {
  // Context functions.
  decltype(mjrf_destroyContext)* destroyContext = nullptr;
  decltype(mjrf_getRendererInfo)* getRendererInfo = nullptr;
  decltype(mjrf_render)* render = nullptr;
  decltype(mjrf_waitForFrame)* waitForFrame = nullptr;
  decltype(mjrf_flush)* flush = nullptr;
  decltype(mjrf_setClearColor)* setClearColor = nullptr;
  decltype(mjrf_getFrameStats)* getFrameStats = nullptr;
  // Texture functions.
  decltype(mjrf_createTexture)* createTexture = nullptr;
  decltype(mjrf_destroyTexture)* destroyTexture = nullptr;
  decltype(mjrf_uploadTexture)* uploadTexture = nullptr;
  decltype(mjrf_getTextureWidth)* getTextureWidth = nullptr;
  decltype(mjrf_getTextureHeight)* getTextureHeight = nullptr;
  decltype(mjrf_getTextureSamplerType)* getTextureSamplerType = nullptr;
  // Mesh functions.
  decltype(mjrf_createMesh)* createMesh = nullptr;
  decltype(mjrf_destroyMesh)* destroyMesh = nullptr;
  decltype(mjrf_uploadMesh)* uploadMesh = nullptr;
  // Scene functions.
  decltype(mjrf_createScene)* createScene = nullptr;
  decltype(mjrf_destroyScene)* destroyScene = nullptr;
  // Light functions.
  decltype(mjrf_addSceneLight)* addSceneLight = nullptr;
  decltype(mjrf_removeSceneLight)* removeSceneLight = nullptr;
  decltype(mjrf_addSceneRenderable)* addSceneRenderable = nullptr;
  decltype(mjrf_removeSceneRenderable)* removeSceneRenderable = nullptr;
  decltype(mjrf_setSceneSkybox)* setSceneSkybox = nullptr;
  decltype(mjrf_configureSceneFromModel)* configureSceneFromModel = nullptr;
  decltype(mjrf_setSceneMsaa)* setSceneMsaa = nullptr;
  decltype(mjrf_createLight)* createLight = nullptr;
  decltype(mjrf_destroyLight)* destroyLight = nullptr;
  decltype(mjrf_setLightActive)* setLightActive = nullptr;
  decltype(mjrf_setLightCastShadow)* setLightCastShadow = nullptr;
  decltype(mjrf_setLightColor)* setLightColor = nullptr;
  decltype(mjrf_setLightIntensity)* setLightIntensity = nullptr;
  decltype(mjrf_setLightRange)* setLightRange = nullptr;
  decltype(mjrf_setLightCutoff)* setLightCutoff = nullptr;
  decltype(mjrf_setLightSoftness)* setLightSoftness = nullptr;
  decltype(mjrf_setLightBulbRadius)* setLightBulbRadius = nullptr;
  decltype(mjrf_setLightShadowMapSize)* setLightShadowMapSize = nullptr;
  decltype(mjrf_setLightPose)* setLightPose = nullptr;
  decltype(mjrf_getLightType)* getLightType = nullptr;
  // Renderable functions.
  decltype(mjrf_createRenderable)* createRenderable = nullptr;
  decltype(mjrf_destroyRenderable)* destroyRenderable = nullptr;
  decltype(mjrf_setRenderableMesh)* setRenderableMesh = nullptr;
  decltype(mjrf_setRenderableGeomMesh)* setRenderableGeomMesh = nullptr;
  decltype(mjrf_setRenderableMaterial)* setRenderableMaterial = nullptr;
  decltype(mjrf_getRenderableMaterial)* getRenderableMaterial = nullptr;
  decltype(mjrf_setRenderablePose)* setRenderablePose = nullptr;
  decltype(mjrf_setRenderableSize)* setRenderableSize = nullptr;
  decltype(mjrf_setRenderableCastShadow)* setRenderableCastShadow = nullptr;
  // Render target functions.
  decltype(mjrf_createRenderTarget)* createRenderTarget = nullptr;
  decltype(mjrf_destroyRenderTarget)* destroyRenderTarget = nullptr;
  decltype(mjrf_resizeRenderTarget)* resizeRenderTarget = nullptr;
};

#endif  // MUJOCO_SRC_RENDER_FILAMENT_PRIVATE_TYPES_H_
