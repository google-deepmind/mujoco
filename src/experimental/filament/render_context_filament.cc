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

#include "experimental/filament/render_context_filament.h"

#include <array>
#include <cstdint>
#include <cstring>

#include <math/mat3.h>
#include <math/vec3.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/mjr_filament_renderer.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture.h"


#if defined(TLS_FILAMENT_CONTEXT)
static thread_local mujoco::MjrFilamentRenderer* g_filament_context = nullptr;
#else
static mujoco::MjrFilamentRenderer* g_filament_context = nullptr;
#endif

static void CheckFilamentContext() {
  if (g_filament_context == nullptr) {
    mju_error("Missing context; did you call mjrf_makeFilamentContext?");
  }
}

template <int N>
static void setf(float (&arr)[N], const std::array<float, N>& values) {
  for (int i = 0; i < N; ++i) {
    arr[i] = values[i];
  }
}


extern "C" {

void mjrf_defaultFilamentConfig(mjrFilamentConfig* config) {
  memset(config, 0, sizeof(mjrFilamentConfig));
}

void mjr_defaultTextureData(mjrTextureData* data) {
  memset(data, 0, sizeof(mjrTextureData));
}

void mjr_defaultTextureConfig(mjrTextureConfig* config) {
  memset(config, 0, sizeof(mjrTextureConfig));
}

void mjr_defaultMeshData(mjrMeshData* data) {
  std::memset(data, 0, sizeof(mjrMeshData));
}

void mjr_defaultSceneParams(mjrSceneParams* params) {
  params->enable_post_processing = true;
  params->enable_reflections = true;
  params->enable_shadows = true;
  params->layer_mask = 0xff;
  params->reflection_layer_mask = 0xff;
}

void mjr_defaultLightParams(mjrLightParams* params) {
  params->type = mjLIGHT_POINT;
  params->texture = nullptr;
  params->color[0] = 0;
  params->color[1] = 0;
  params->color[2] = 0;
  params->intensity = 0.0f;
  params->cast_shadows = true;
  params->range = 10.0f;
  params->spot_cone_angle = 180.f;
  params->bulb_radius = 0.0f;
  params->shadow_map_size = 2048;
  params->vsm_blur_width = 0.0f;
}

void mjr_defaultMaterialTextures(mjrMaterialTextures* textures) {
  textures->color = nullptr;
  textures->normal = nullptr;
  textures->metallic = nullptr;
  textures->roughness = nullptr;
  textures->occlusion = nullptr;
  textures->orm = nullptr;
  textures->emissive = nullptr;
  textures->reflection = nullptr;
}

void mjr_defaultMaterialParams(mjrMaterialParams* params) {
  setf(params->color, {1.f, 1.f, 1.f, 1.f});
  setf(params->segmentation_color, {1, 1, 1, 1});
  setf(params->uv_scale, {1, 1, 1});
  setf(params->uv_offset, {0, 0, 0});
  setf(params->scissor, {0, 0, 0, 0});
  params->emissive = -1.0f;
  params->specular = -1.0f;
  params->glossiness = -1.0f;
  params->metallic = -1.0f;
  params->roughness = -1.0f;
  params->reflectance = 0.0f;
  params->tex_uniform = false;
  params->reflective = false;
}

void mjr_defaultRenderableParams(mjrRenderableParams* params) {
  params->shading_model = mjSHADING_MODEL_SCENE_OBJECT;
  params->cast_shadows = true;
  params->receive_shadows = true;
  params->layer_mask = 0x01;
  params->priority = 4;
  params->blend_order = 0;
}

void mjr_defaultRenderTargetConfig(mjrRenderTargetConfig* config) {
  memset(config, 0, sizeof(mjrRenderTargetConfig));
  config->color_format = mjPIXEL_FORMAT_RGBA8;
  config->depth_format = mjPIXEL_FORMAT_DEPTH32F;
}

void mjr_defaultRenderRequest(mjrRenderRequest* request) {
  memset(request, 0, sizeof(mjrRenderRequest));
}

void mjr_defaultReadPixelsRequest(mjrReadPixelsRequest* request) {
  memset(request, 0, sizeof(mjrReadPixelsRequest));
}

void mjr_defaultFrameStats(mjrFrameStats* stats) {
  memset(stats, 0, sizeof(mjrFrameStats));
}

mjrfContext* mjrf_createContext(const mjrFilamentConfig* config) {
  return new mujoco::FilamentContext(config);
}

void mjrf_destroyContext(mjrfContext* ctx) {
  delete mujoco::FilamentContext::downcast(ctx);
}

mjrTexture* mjrf_createTexture(mjrfContext* ctx, const mjrTextureConfig* cfg) {
  return new mujoco::Texture(
      mujoco::FilamentContext::downcast(ctx)->GetEngine(), *cfg);
}

void mjrf_destroyTexture(mjrTexture* texture) {
  delete mujoco::Texture::downcast(texture);
}

mjrMesh* mjrf_createMesh(mjrfContext* ctx, const mjrMeshData* data) {
  return new mujoco::Mesh(mujoco::FilamentContext::downcast(ctx)->GetEngine(),
                          *data);
}

void mjrf_destroyMesh(mjrMesh* mesh) { delete mujoco::Mesh::downcast(mesh); }

mjrScene* mjrf_createScene(mjrfContext* ctx, const mjrSceneParams* params) {
  return new mujoco::SceneView(mujoco::FilamentContext::downcast(ctx), *params);
}

void mjrf_destroyScene(mjrScene* scene) {
  delete mujoco::SceneView::downcast(scene);
}

mjrLight* mjrf_createLight(mjrfContext* ctx, const mjrLightParams* params) {
  return new mujoco::Light(mujoco::FilamentContext::downcast(ctx), *params);
}

void mjrf_destroyLight(mjrLight* light) {
  delete mujoco::Light::downcast(light);
}

mjrRenderable* mjrf_createRenderable(mjrfContext* ctx,
                                     const mjrRenderableParams* params) {
  return new mujoco::Renderable(mujoco::FilamentContext::downcast(ctx),
                                *params);
}

void mjrf_destroyRenderable(mjrRenderable* renderable) {
  delete mujoco::Renderable::downcast(renderable);
}

mjrRenderTarget* mjrf_createRenderTarget(mjrfContext* ctx,
                                         const mjrRenderTargetConfig* config) {
  return new mujoco::RenderTarget(
      mujoco::FilamentContext::downcast(ctx)->GetEngine(), *config);
}

void mjrf_destroyRenderTarget(mjrRenderTarget* render_target) {
  delete mujoco::RenderTarget::downcast(render_target);
}

void mjrf_setTextureData(mjrTexture* texture, const mjrTextureData* data) {
  mujoco::Texture::downcast(texture)->Upload(*data);
}

int mjrf_getTextureWidth(const mjrTexture* texture) {
  return mujoco::Texture::downcast(texture)->GetWidth();
}

int mjrf_getTextureHeight(const mjrTexture* texture) {
  return mujoco::Texture::downcast(texture)->GetHeight();
}

mjrTextureTarget mjrf_getTextureTarget(const mjrTexture* texture) {
  return mujoco::Texture::downcast(texture)->GetTarget();
}

void mjrf_setLightEnabled(mjrLight* light, bool enabled) {
  if (enabled) {
    mujoco::Light::downcast(light)->Enable();
  } else {
    mujoco::Light::downcast(light)->Disable();
  }
}

void mjrf_setLightIntensity(mjrLight* light, float intensity) {
  mujoco::Light::downcast(light)->SetIntensity(intensity);
}

void mjrf_setLightColor(mjrLight* light, const float color[3]) {
  mujoco::Light::downcast(light)->SetColor({color[0], color[1], color[2]});
}

void mjrf_setLightTransform(mjrLight* light, const float position[3],
                            const float direction[3]) {
  mujoco::Light::downcast(light)->SetTransform(
      {position[0], position[1], position[2]},
      {direction[0], direction[1], direction[2]});
}

mjrLightType mjrf_getLightType(const mjrLight* light) {
  return mujoco::Light::downcast(light)->GetType();
}

void mjrf_setRenderableMesh(mjrRenderable* renderable, const mjrMesh* mesh,
                            int elem_offset, int elem_count) {
  mujoco::Renderable::downcast(renderable)
      ->SetMesh(mujoco::Mesh::downcast(mesh), elem_offset, elem_count);
}

void mjrf_setRenderableGeomMesh(mjrRenderable* renderable, mjtGeom type,
                                int nstack, int nslice, int nquad) {
  mujoco::Renderable::downcast(renderable)->SetGeomMesh(type, nstack, nslice,
                                                        nquad);
}

void mjrf_setRenderableMaterial(mjrRenderable* renderable,
                                const mjrMaterialParams* params,
                                const mjrMaterialTextures* textures) {
  mujoco::Renderable::downcast(renderable)->UpdateMaterial(*params, *textures);
}

void mjrf_setRenderableTransform(mjrRenderable* renderable,
                                 const float position[3],
                                 const float rotation[9], const float size[3]) {
  const filament::math::float3 fposition{position[0], position[1], position[2]};
  const filament::math::float3 fsize{size[0], size[1], size[2]};
  const filament::math::mat3f frotation{rotation[0], rotation[3], rotation[6],
                                        rotation[1], rotation[4], rotation[7],
                                        rotation[2], rotation[5], rotation[8]};
  mujoco::Renderable::downcast(renderable)
      ->SetTransform({fposition, frotation, fsize});
}

void mjrf_setRenderableLayerMask(mjrRenderable* renderable,
                                 uint8_t layer_mask) {
  mujoco::Renderable::downcast(renderable)->SetLayerMask(layer_mask);
}

void mjrf_setRenderableWireframe(mjrRenderable* renderable, bool wireframe) {
  mujoco::Renderable::downcast(renderable)->SetWireframe(wireframe);
}

void mjrf_setRenderableCastShadows(mjrRenderable* renderable,
                                   bool cast_shadows) {
  mujoco::Renderable::downcast(renderable)->SetCastShadows(cast_shadows);
}

void mjrf_setRenderableReceiveShadows(mjrRenderable* renderable,
                                      bool receive_shadows) {
  mujoco::Renderable::downcast(renderable)->SetReceiveShadows(receive_shadows);
}

void mjrf_addLightToScene(mjrScene* scene, mjrLight* light) {
  mujoco::SceneView::downcast(scene)->AddToScene(
      mujoco::Light::downcast(light));
}

void mjrf_removeLightFromScene(mjrScene* scene, mjrLight* light) {
  mujoco::SceneView::downcast(scene)->RemoveFromScene(
      mujoco::Light::downcast(light));
}

void mjrf_addRenderableToScene(mjrScene* scene, mjrRenderable* renderable) {
  mujoco::SceneView::downcast(scene)->AddToScene(
      mujoco::Renderable::downcast(renderable));
}

void mjrf_removeRenderableFromScene(mjrScene* scene,
                                    mjrRenderable* renderable) {
  mujoco::SceneView::downcast(scene)->RemoveFromScene(
      mujoco::Renderable::downcast(renderable));
}

void mjrf_setSceneSkybox(mjrScene* scene, const mjrTexture* texture) {
  mujoco::SceneView::downcast(scene)->SetSkybox(
      mujoco::Texture::downcast(texture));
}

void mjrf_setSceneShadowsEnabled(mjrScene* scene, bool enabled) {
  if (enabled) {
    mujoco::SceneView::downcast(scene)->EnableShadows();
  } else {
    mujoco::SceneView::downcast(scene)->DisableShadows();
  }
}

void mjrf_setSceneReflectionsEnabled(mjrScene* scene, bool enabled) {
  if (enabled) {
    mujoco::SceneView::downcast(scene)->EnableReflections();
  } else {
    mujoco::SceneView::downcast(scene)->DisableReflections();
  }
}

void mjrf_configureSceneFromModel(mjrScene* scene, const mjModel* model) {
  mujoco::SceneView::downcast(scene)->Configure(model);
}

mjrFrameHandle mjrf_render(mjrfContext* ctx, const mjrRenderRequest* req,
                           int nreq, const mjrReadPixelsRequest* read_req,
                           int nread_req) {
  return mujoco::FilamentContext::downcast(ctx)->Render(
      {req, static_cast<size_t>(nreq)},
      {read_req, static_cast<size_t>(nread_req)});
}

void mjrf_waitForFrame(mjrfContext* ctx, mjrFrameHandle frame) {
  mujoco::FilamentContext::downcast(ctx)->WaitForFrame(frame);
}

void mjrf_getFrameStats(mjrfContext* ctx, mjrFrameHandle frame,
                        mjrFrameStats* stats_out) {
  mujoco::FilamentContext::downcast(ctx)->GetFrameStats(frame, stats_out);
}

// Legacy API, to be deprecated.

void mjrf_makeFilamentContext(const mjModel* m, mjrContext* con,
                              const mjrFilamentConfig* config) {
  // TODO: Support multiple contexts and multiple threads. For now, we'll just
  // assume a single, global context.
  if (g_filament_context != nullptr) {
    mju_error("Context already exists!");
  }
  g_filament_context = new mujoco::MjrFilamentRenderer(config);
  g_filament_context->Init(m);
}

void mjrf_defaultContext(mjrContext* con) {
  memset(con, 0, sizeof(mjrContext));
}

void mjrf_makeContext(const mjModel* m, mjrContext* con, int fontscale) {
  mjrf_freeContext(con);
  mjrFilamentConfig cfg;
  mjrf_defaultFilamentConfig(&cfg);
  cfg.width = m->vis.global.offwidth;
  cfg.height = m->vis.global.offheight;
  mjrf_makeFilamentContext(m, con, &cfg);
}

void mjrf_freeContext(mjrContext* con) {
  // mjr_freeContext may be called multiple times.
  if (g_filament_context) {
    delete g_filament_context;
    g_filament_context = nullptr;
  }
  mjrf_defaultContext(con);
}

void mjrf_renderScene(mjrRect viewport, mjvScene* scn, const mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->Render(viewport, scn);
}

void mjrf_uploadMesh(const mjModel* m, const mjrContext* con, int meshid) {
  CheckFilamentContext();
  g_filament_context->UploadMesh(m, meshid);
}

void mjrf_uploadTexture(const mjModel* m, const mjrContext* con, int texid) {
  CheckFilamentContext();
  g_filament_context->UploadTexture(m, texid);
}

void mjrf_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid) {
  CheckFilamentContext();
  g_filament_context->UploadHeightField(m, hfieldid);
}

void mjrf_setBuffer(int framebuffer, mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->SetFrameBuffer(framebuffer);
}

void mjrf_readPixels(unsigned char* rgb, float* depth, mjrRect viewport,
                          const mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->ReadPixels(viewport, rgb, depth);
}

uintptr_t mjrf_uploadGuiImage(uintptr_t tex_id, const unsigned char* pixels,
                             int width, int height, int bpp,
                             const mjrContext* con) {
  CheckFilamentContext();
  return g_filament_context->UploadGuiImage(tex_id, pixels, width, height, bpp);
}

double mjrf_getFrameRate(const mjrContext* con) {
  CheckFilamentContext();
  return g_filament_context->GetFrameRate();
}

void mjrf_updateGui(const mjrContext* con) {
  if (g_filament_context != nullptr) {
    g_filament_context->UpdateGui();
  }
}

}  // extern "C"
