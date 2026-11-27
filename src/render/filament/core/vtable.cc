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

#include <cstring>

#include <math/mathfwd.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/core/filament_context.h"
#include "render/filament/core/light.h"
#include "render/filament/core/mesh.h"
#include "render/filament/core/render_target.h"
#include "render/filament/core/renderable.h"
#include "render/filament/core/scene_view.h"
#include "render/filament/core/texture.h"
#include "render/filament/private/types.h"

namespace mujoco {

template <typename T, typename U>
static void CheckVtableMatches(const T* obj0, const U* obj1) {
  if (obj0 && obj1 && obj0->vtable != obj1->vtable) {
    mju_error("Vtable mismatch");
  }
}

static void destroyContext(mjrfContext* ctx) {
  delete FilamentContext::downcast(ctx);
}

static void getRendererInfo(mjrfContext* ctx, mjrRendererInfo* info) {
  memset(info, 0, sizeof(mjrRendererInfo));
  info->renderer = "filament";
  info->backend = ctx ? FilamentContext::downcast(ctx)->GetBackendName() : "";
}

static mjrfTexture* createTexture(mjrfContext* ctx,
                                  const mjrfTextureConfig* config) {
  if (!config) {
    mju_warning("Invalid (nullptr) config: createTexture");
    return nullptr;
  }
  if (!Texture::ValidateConfig(*config)) {
    return nullptr;
  }
  mjrfTexture* texture =
      new Texture(FilamentContext::downcast(ctx)->GetEngine(), *config);
  texture->vtable = ctx->vtable;
  return texture;
}

static void destroyTexture(mjrfTexture* texture) {
  delete Texture::downcast(texture);
}

static mjrfMesh* createMesh(mjrfContext* ctx, const mjrfMeshConfig* config) {
  if (!config) {
    mju_warning("Invalid (nullptr) config: createMesh");
    return nullptr;
  }
  if (!Mesh::ValidateConfig(*config)) {
    return nullptr;
  }
  mjrfMesh* mesh =
      new Mesh(FilamentContext::downcast(ctx)->GetEngine(), *config);
  mesh->vtable = ctx->vtable;
  return mesh;
}

static void destroyMesh(mjrfMesh* mesh) { delete Mesh::downcast(mesh); }

static mjrfScene* createScene(mjrfContext* ctx, const mjrfSceneParams* params) {
  mjrfScene* scene = new SceneView(
      FilamentContext::downcast(ctx)->GetObjectManager(),
      FilamentContext::downcast(ctx)->GetMaterialManager(), *params);
  scene->vtable = ctx->vtable;
  return scene;
}

static void destroyScene(mjrfScene* scene) {
  delete SceneView::downcast(scene);
}

static mjrfLight* createLight(mjrfContext* ctx, const mjrfLightParams* params) {
  if (!params) {
    mju_warning("Invalid (nullptr) params: createLight");
    return nullptr;
  }
  if (!Light::ValidateParams(*params)) {
    return nullptr;
  }
  mjrfLight* light =
      new Light(FilamentContext::downcast(ctx)->GetEngine(), *params);
  light->vtable = ctx->vtable;
  return light;
}

static void destroyLight(mjrfLight* light) { delete Light::downcast(light); }

static mjrfRenderable* createRenderable(mjrfContext* ctx,
                                        const mjrfRenderableParams* params) {
  mjrfRenderable* renderable =
      new Renderable(FilamentContext::downcast(ctx)->GetEngine(), *params,
                     FilamentContext::downcast(ctx)->GetMaterialManager());
  renderable->vtable = ctx->vtable;
  return renderable;
}

static void destroyRenderable(mjrfRenderable* renderable) {
  delete Renderable::downcast(renderable);
}

static mjrfRenderTarget* createRenderTarget(
    mjrfContext* ctx, const mjrfRenderTargetConfig* config) {
  if (!config) {
    mju_warning("Invalid (nullptr) config: createRenderTarget");
    return nullptr;
  }
  if (!RenderTarget::ValidateConfig(*config)) {
    return nullptr;
  }
  mjrfRenderTarget* render_target =
      new RenderTarget(FilamentContext::downcast(ctx)->GetEngine(), *config);
  render_target->vtable = ctx->vtable;
  return render_target;
}

static void destroyRenderTarget(mjrfRenderTarget* render_target) {
  delete RenderTarget::downcast(render_target);
}

static void uploadTexture(mjrfTexture* texture, const mjrfTextureData* data) {
  Texture::downcast(texture)->Upload(*data);
}

static void uploadMesh(mjrfMesh* mesh, const mjrfMeshData* data) {
  Mesh::downcast(mesh)->Upload(*data);
}

static int getTextureWidth(const mjrfTexture* texture) {
  return Texture::downcast(texture)->GetWidth();
}

static int getTextureHeight(const mjrfTexture* texture) {
  return Texture::downcast(texture)->GetHeight();
}

static int getTextureSamplerType(const mjrfTexture* texture) {
  return Texture::downcast(texture)->GetSamplerType();
}

static void setLightActive(mjrfLight* light, mjtBool active) {
  if (active) {
    Light::downcast(light)->Enable();
  } else {
    Light::downcast(light)->Disable();
  }
}

static void setLightCastShadow(mjrfLight* light, mjtBool enabled) {
  Light::downcast(light)->SetShadowsEnabled(enabled);
}

static void setLightColor(mjrfLight* light, const float color[3]) {
  Light::downcast(light)->SetColor({color[0], color[1], color[2]});
}

static void setLightIntensity(mjrfLight* light, float intensity) {
  Light::downcast(light)->SetIntensity(intensity);
}

static void setLightRange(mjrfLight* light, float range) {
  Light::downcast(light)->SetRange(range);
}

static void setLightCutoff(mjrfLight* light, float cutoff) {
  Light::downcast(light)->SetCutoffAngle(cutoff);
}

static void setLightSoftness(mjrfLight* light, float softness) {
  Light::downcast(light)->SetSoftness(softness);
}

static void setLightBulbRadius(mjrfLight* light, float radius) {
  Light::downcast(light)->SetBulbRadius(radius);
}

static void setLightShadowMapSize(mjrfLight* light, int map_size) {
  Light::downcast(light)->SetShadowMapSize(map_size);
}

static void setLightPose(mjrfLight* light, const float position[3],
                         const float direction[3]) {
  Light::downcast(light)->SetTransform(
      {position[0], position[1], position[2]},
      {direction[0], direction[1], direction[2]});
}

static int getLightType(const mjrfLight* light) {
  return Light::downcast(light)->GetType();
}

static void setRenderableMesh(mjrfRenderable* renderable, const mjrfMesh* mesh,
                              int elem_offset, int elem_count) {
  CheckVtableMatches(renderable, mesh);
  Renderable::downcast(renderable)
      ->SetMesh(Mesh::downcast(mesh), elem_offset, elem_count);
}

static void setRenderableGeomMesh(mjrfRenderable* renderable, int type,
                                  int nstack, int nslice, int nquad) {
  Renderable::downcast(renderable)
      ->SetGeomMesh(static_cast<mjtGeom>(type), nstack, nslice, nquad);
}

static void setRenderableMaterial(mjrfRenderable* renderable,
                                  const mjrfMaterial* material) {
  Renderable::downcast(renderable)->UpdateMaterial(*material);
}

static void getRenderableMaterial(mjrfRenderable* renderable,
                                  mjrfMaterial* material) {
  *material = Renderable::downcast(renderable)->GetMaterial();
}

static void setRenderablePose(mjrfRenderable* renderable,
                              const float position[3],
                              const float rotation[9]) {
  const filament::math::float3 fposition{position[0], position[1], position[2]};
  const filament::math::mat3f frotation{rotation[0], rotation[3], rotation[6],
                                        rotation[1], rotation[4], rotation[7],
                                        rotation[2], rotation[5], rotation[8]};
  Renderable::downcast(renderable)->SetTransform(fposition, frotation);
}

static void setRenderableSize(mjrfRenderable* renderable, const float size[3]) {
  const filament::math::float3 fsize{size[0], size[1], size[2]};
  Renderable::downcast(renderable)->SetSize(fsize);
}

static void setRenderableCastShadow(mjrfRenderable* renderable,
                                    mjtBool enabled) {
  Renderable::downcast(renderable)->SetCastShadows(enabled);
}

static void addSceneLight(mjrfScene* scene, mjrfLight* light) {
  CheckVtableMatches(scene, light);
  SceneView::downcast(scene)->AddToScene(Light::downcast(light));
}

static void removeSceneLight(mjrfScene* scene, mjrfLight* light) {
  CheckVtableMatches(scene, light);
  SceneView::downcast(scene)->RemoveFromScene(Light::downcast(light));
}

static void addSceneRenderable(mjrfScene* scene, mjrfRenderable* renderable) {
  CheckVtableMatches(scene, renderable);
  SceneView::downcast(scene)->AddToScene(Renderable::downcast(renderable));
}

static void removeSceneRenderable(mjrfScene* scene,
                                  mjrfRenderable* renderable) {
  CheckVtableMatches(scene, renderable);
  SceneView::downcast(scene)->RemoveFromScene(Renderable::downcast(renderable));
}

static void setSceneSkybox(mjrfScene* scene, const mjrfTexture* texture) {
  CheckVtableMatches(scene, texture);
  SceneView::downcast(scene)->SetSkybox(Texture::downcast(texture));
}

static void configureSceneFromModel(mjrfScene* scene, const mjModel* model) {
  SceneView::downcast(scene)->Configure(model);
}

static void setSceneMsaa(mjrfScene* scene, int samples) {
  SceneView::downcast(scene)->SetMsaa(samples);
}

static void resizeRenderTarget(mjrfRenderTarget* render_target, int width,
                               int height) {
  RenderTarget::downcast(render_target)->Prepare(width, height);
}

static mjrfFrameHandle render(mjrfContext* ctx, const mjrfRenderRequest* req,
                              int nreq, const mjrfReadRequest* read_req,
                              int nread_req) {
  return FilamentContext::downcast(ctx)->Render(
      {req, static_cast<size_t>(nreq)},
      {read_req, static_cast<size_t>(nread_req)});
}

static void waitForFrame(mjrfContext* ctx, mjrfFrameHandle frame) {
  FilamentContext::downcast(ctx)->WaitForFrame(frame);
}

static void flush(mjrfContext* ctx) { FilamentContext::downcast(ctx)->Flush(); }

static void setClearColor(mjrfContext* ctx, const float color[3]) {
  FilamentContext::downcast(ctx)->SetClearColor(
      {color[0], color[1], color[2], 1.0f});
}

static void getFrameStats(mjrfContext* ctx, mjrfFrameHandle frame,
                          mjrfFrameStats* stats_out) {
  FilamentContext::downcast(ctx)->GetFrameStats(frame, stats_out);
}

static const mjrfVtable_* GetVTable() {
  static const mjrfVtable_ vtable = {
      .destroyContext = destroyContext,
      .getRendererInfo = getRendererInfo,
      .render = render,
      .waitForFrame = waitForFrame,
      .flush = flush,
      .setClearColor = setClearColor,
      .getFrameStats = getFrameStats,
      .createTexture = createTexture,
      .destroyTexture = destroyTexture,
      .uploadTexture = uploadTexture,
      .getTextureWidth = getTextureWidth,
      .getTextureHeight = getTextureHeight,
      .getTextureSamplerType = getTextureSamplerType,
      .createMesh = createMesh,
      .destroyMesh = destroyMesh,
      .uploadMesh = uploadMesh,
      .createScene = createScene,
      .destroyScene = destroyScene,
      .addSceneLight = addSceneLight,
      .removeSceneLight = removeSceneLight,
      .addSceneRenderable = addSceneRenderable,
      .removeSceneRenderable = removeSceneRenderable,
      .setSceneSkybox = setSceneSkybox,
      .configureSceneFromModel = configureSceneFromModel,
      .setSceneMsaa = setSceneMsaa,
      .createLight = createLight,
      .destroyLight = destroyLight,
      .setLightActive = setLightActive,
      .setLightCastShadow = setLightCastShadow,
      .setLightColor = setLightColor,
      .setLightIntensity = setLightIntensity,
      .setLightRange = setLightRange,
      .setLightCutoff = setLightCutoff,
      .setLightSoftness = setLightSoftness,
      .setLightBulbRadius = setLightBulbRadius,
      .setLightShadowMapSize = setLightShadowMapSize,
      .setLightPose = setLightPose,
      .getLightType = getLightType,
      .createRenderable = createRenderable,
      .destroyRenderable = destroyRenderable,
      .setRenderableMesh = setRenderableMesh,
      .setRenderableGeomMesh = setRenderableGeomMesh,
      .setRenderableMaterial = setRenderableMaterial,
      .getRenderableMaterial = getRenderableMaterial,
      .setRenderablePose = setRenderablePose,
      .setRenderableSize = setRenderableSize,
      .setRenderableCastShadow = setRenderableCastShadow,
      .createRenderTarget = createRenderTarget,
      .destroyRenderTarget = destroyRenderTarget,
      .resizeRenderTarget = resizeRenderTarget,
  };
  return &vtable;
}

}  // namespace mujoco

extern "C" {

mjrfContext* mjrf_createContext(const mjrfContextConfig* config) {
  mjrfContext* ctx = new mujoco::FilamentContext(config);
  ctx->vtable = mujoco::GetVTable();
  return ctx;
}

}  // extern "C"
