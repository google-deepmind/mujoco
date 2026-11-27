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

#include <mujoco/mjrfilament.h>

#include <array>
#include <cmath>
#include <cstring>
#include <string_view>
#include <type_traits>
#include <utility>

#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "render/filament/private/types.h"

template <int N>
static void setf(float (&arr)[N], const std::array<float, N>& values) {
  for (int i = 0; i < N; ++i) {
    arr[i] = values[i];
  }
}

template <int N>
static void setf(float (&arr)[N], const float* values) {
  std::memcpy(arr, values, sizeof(arr));
}

// Helper class for reading values from mjModel custom fields.
struct ModelReader {
  explicit ModelReader(const mjModel* model) : model(model) {}

  // Reads a scalar value from the model.
  template <typename T>
  void operator()(T& value, const char* name) {
    const int id = mj_name2id(model, mjOBJ_NUMERIC, name);
    if (id < 0) {
      return;
    }
    if (model->numeric_size[id] != 1) {
      mju_warning("Invalid numeric size for '%s': expected 1, got %d", name,
                  model->numeric_size[id]);
      return;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    if constexpr (std::is_same_v<T, bool>) {
      value = static_cast<T>(*ptr != 0);
    } else if constexpr (std::is_arithmetic_v<T>) {
      value = static_cast<T>(*ptr);
    }
  }

  // Reads a vector value from the model.
  template <typename T, size_t N>
  void operator()(T (&arr)[N], const char* name) {
    const int id = mj_name2id(model, mjOBJ_NUMERIC, name);
    if (id < 0) {
      return;
    }
    if (model->numeric_size[id] != static_cast<int>(N)) {
      mju_warning("Invalid numeric size for '%s': expected %d, got %d", name,
                  static_cast<int>(N), model->numeric_size[id]);
      return;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    for (int i = 0; i < static_cast<int>(N); ++i) {
      arr[i] = static_cast<T>(ptr[i]);
    }
  }

  // Reads an enum value from the model (stored as a string).
  template <typename T>
  void operator()(int& value, const char* name, T e) {
    const int id = mj_name2id(model, mjOBJ_TEXT, name);
    if (id < 0) {
      return;
    }
    std::string_view txt = model->text_data + model->text_adr[id];
    if (txt.empty()) {
      return;
    }
    if constexpr (std::is_same_v<T, mjrShadowType>) {
      if (txt == "none") { value = mjSHADOW_TYPE_NONE; return; }
      if (txt == "hard") { value = mjSHADOW_TYPE_HARD; return; }
      if (txt == "soft") { value = mjSHADOW_TYPE_SOFT; return; }
      // legacy names
      if (txt == "pcf") { value = mjSHADOW_TYPE_HARD; return; }
      if (txt == "pcss") { value = mjSHADOW_TYPE_SOFT; return; }
    } else if constexpr (std::is_same_v<T, mjrToneMapper>) {
      if (txt == "aces") { value = mjTONE_MAPPER_ACES; return; }
      if (txt == "aces_legacy") { value = mjTONE_MAPPER_ACES_LEGACY; return; }
      if (txt == "filmic") { value = mjTONE_MAPPER_FILMIC; return; }
      if (txt == "linear") { value = mjTONE_MAPPER_LINEAR; return; }
      if (txt == "pbr_neutral") { value = mjTONE_MAPPER_PBR_NEUTRAL; return; }
    } else if constexpr (std::is_same_v<T, mjrColorSpace>) {
      if (txt == "linear") { value = mjCOLORSPACE_LINEAR; return; }
      if (txt == "srgb") { value = mjCOLORSPACE_SRGB; return; }
    } else if constexpr (std::is_same_v<T, mjrQuality>) {
      if (txt == "low") { value = mjQUALITY_LOW; return; }
      if (txt == "medium") { value = mjQUALITY_MEDIUM; return; }
      if (txt == "high") { value = mjQUALITY_HIGH; return; }
      if (txt == "ultra") { value = mjQUALITY_ULTRA; return; }
    } else if constexpr (std::is_same_v<T, mjrAmbientOcclusionType>) {
      if (txt == "none") { value = mjAMBIENT_OCCLUSION_TYPE_NONE; return; }
      if (txt == "scalable") { value = mjAMBIENT_OCCLUSION_TYPE_SCALABLE; return; }
      if (txt == "ground_truth") { value = mjAMBIENT_OCCLUSION_TYPE_GROUND_TRUTH; return; }
    } else if constexpr (std::is_same_v<T, mjrDitheringType>) {
      if (txt == "none") { value = mjDITHERING_TYPE_NONE; return; }
      if (txt == "temporal") { value = mjDITHERING_TYPE_TEMPORAL; return; }
    } else {
      static_assert(std::is_same_v<T, void>, "Unsupported enum type");
    }
    mju_warning("Unsupported enum value for %s: %s", name, txt.data());
  }

  const mjModel* model;
};

// Calls the function on the object. Warns if either object or function is null.
template <auto Function, typename T, typename... Args>
static auto Call(T* obj, const char* fname, Args&&... args) {
  using Result = decltype((obj->vtable->*Function)(obj, std::forward<Args>(args)...));
  if (obj == nullptr) {
    mju_warning("Invalid (nullptr) object: %s", fname);
    if constexpr (std::is_void_v<Result>) {
      return;
    } else {
      return Result{};
    }
  } else if (obj->vtable == nullptr) {
    mju_warning("Invalid vtable: %s", fname);
    if constexpr (std::is_void_v<Result>) {
      return;
    } else {
      return Result{};
    }
  } else if (obj->vtable->*Function == nullptr) {
    mju_warning("Not implemented: %s", fname);
    if constexpr (std::is_void_v<Result>) {
      return;
    } else {
      return Result{};
    }
  } else {
    return (obj->vtable->*Function)(obj, std::forward<Args>(args)...);
  }
}

#define MJRF_CALL(obj, func, ...)                           \
  Call<&std::remove_cvref_t<decltype(*obj->vtable)>::func>( \
      obj, #func __VA_OPT__(, ) __VA_ARGS__)

extern "C" {

void mjrf_defaultContextConfig(mjrfContextConfig* config) {
  memset(config, 0, sizeof(mjrfContextConfig));
}

void mjrf_defaultTextureData(mjrfTextureData* data) {
  memset(data, 0, sizeof(mjrfTextureData));
}

void mjrf_defaultTextureConfig(mjrfTextureConfig* config) {
  memset(config, 0, sizeof(mjrfTextureConfig));
}

void mjrf_defaultMeshConfig(mjrfMeshConfig* config) {
  memset(config, 0, sizeof(mjrfMeshConfig));
}

void mjrf_defaultMeshData(mjrfMeshData* data) {
  memset(data, 0, sizeof(mjrfMeshData));
}

void mjrf_defaultSceneParams(mjrfSceneParams* params) {
  memset(params, 0, sizeof(mjrfSceneParams));
}

void mjrf_defaultLightParams(mjrfLightParams* params) {
  memset(params, 0, sizeof(mjrfLightParams));
  params->type = mjLIGHT_POINT;
  params->texture = nullptr;
  params->color[0] = 0;
  params->color[1] = 0;
  params->color[2] = 0;
  params->intensity = 0.0f;
  params->cast_shadow = true;
  params->range = 10.0f;
  params->cutoff = 180.f;
  params->softness = 0.0f;
  params->bulb_radius = 0.0f;
  params->shadow_map_size = 2048;
  params->image_diffuse_scale = 1.0f;
}

void mjrf_defaultMaterial(mjrfMaterial* material) {
  memset(material, 0, sizeof(mjrfMaterial));
  setf(material->color, {1.f, 1.f, 1.f, 1.f});
  setf(material->uv_scale, {1, 1, 1});
  material->sleep_state = mjS_STATIC;
  material->island_id = -1;
  material->emissive = -1.0f;
  material->specular = -1.0f;
  material->glossiness = -1.0f;
  material->metallic = -1.0f;
  material->roughness = -1.0f;
  material->haze_ratio = -1.0f;
}

void mjrf_defaultRenderableParams(mjrfRenderableParams* params) {
  memset(params, 0, sizeof(mjrfRenderableParams));
  params->cast_shadow = true;
  params->receive_shadow = true;
  params->blend_order = 0;
}

void mjrf_defaultRenderTargetConfig(mjrfRenderTargetConfig* config) {
  memset(config, 0, sizeof(mjrfRenderTargetConfig));
  config->color_format = mjPIXEL_FORMAT_RGBA8;
  config->depth_format = mjPIXEL_FORMAT_DEPTH32F;
}

void mjrf_defaultRenderRequest(mjrfRenderRequest* request) {
  memset(request, 0, sizeof(mjrfRenderRequest));
  request->post_processing = true;
  request->reflections = true;
  request->shadows = true;
  request->skybox = true;
  request->haze = true;
}

void mjrf_defaultReadRequest(mjrfReadRequest* request) {
  memset(request, 0, sizeof(mjrfReadRequest));
}

void mjrf_defaultFrameStats(mjrfFrameStats* stats) {
  memset(stats, 0, sizeof(mjrfFrameStats));
}

void mjrf_defaultShadowOpts(mjrfShadowOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfShadowOpts));
  opts->type = mjSHADOW_TYPE_HARD;
}

void mjrf_defaultMaterialOpts(mjrfMaterialOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfMaterialOpts));
  opts->rgb_color_type = mjCOLORSPACE_LINEAR;
  opts->enable_textures = true;
}

void mjrf_defaultHeadlightOpts(mjrfHeadlightOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfHeadlightOpts));
}

void mjrf_defaultSkyboxOpts(mjrfSkyboxOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfSkyboxOpts));
  opts->enable = true;
}

void mjrf_defaultHazeOpts(mjrfHazeOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfHazeOpts));
  opts->enable = true;
  setf(opts->color, {1.0f, 1.0f, 1.0f, 1.0f});
  opts->ratio = 0.3f;
  opts->slices = 28;
}

void mjrf_defaultFogOpts(mjrfFogOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfFogOpts));
  opts->enable = false;
  setf(opts->color, {1.0f, 1.0f, 1.0f});
  opts->density = 0.1f;
  opts->max_opacity = 1.0f;
  opts->distance = 0.0f;
  opts->cutoff = INFINITY;
  opts->height = 0.0f;
  opts->height_falloff = 1.0f;
}

void mjrf_defaultBloomOpts(mjrfBloomOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfBloomOpts));
  opts->enable = false;
  opts->quality = mjQUALITY_LOW;
  opts->strength = 0.1f;
  opts->levels = 6;
  opts->lens_flare = false;
  opts->starburst = false;
}

void mjrf_defaultVignetteOpts(mjrfVignetteOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfVignetteOpts));
  opts->enable = false;
  setf(opts->color, {0.0f, 0.0f, 0.0f, 1.0f});
  opts->mid_point = 0.5f;
  opts->roundness = 0.5f;
  opts->feather = 0.5f;
}

void mjrf_defaultReflectionOpts(mjrfReflectionOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfReflectionOpts));
  opts->enable = true;
}

void mjrf_defaultDepthOfFieldOpts(mjrfDepthOfFieldOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfDepthOfFieldOpts));
  opts->enable = false;
  opts->blur_strength = 1.0f;
  opts->focus_distance = 1.0f;
  opts->aspect_ratio = 1.0f;
  opts->max_aperture_diameter = 0.01f;
}

void mjrf_defaultDitheringOpts(mjrfDitheringOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfDitheringOpts));
  opts->type = mjDITHERING_TYPE_TEMPORAL;
}

void mjrf_defaultColorGradingOpts(mjrfColorGradingOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfColorGradingOpts));
  opts->tone_mapper = mjTONE_MAPPER_PBR_NEUTRAL;
  opts->output_color_space = mjCOLORSPACE_SRGB;
  opts->exposure = 0.0f;
  opts->night_adaptation = 0.0f;
  opts->contrast = 1.0f;
  opts->vibrance = 1.0f;
  opts->saturation = 1.0f;
  opts->temperature = 0.0f;
  opts->tint = 0.0f;
  setf(opts->out_red, {1.0f, 0.0f, 0.0f});
  setf(opts->out_green, {0.0f, 1.0f, 0.0f});
  setf(opts->out_blue, {0.0f, 0.0f, 1.0f});
  setf(opts->shadows, {1.0f, 1.0f, 1.0f, 0.0f});
  setf(opts->midtones, {1.0f, 1.0f, 1.0f, 0.0f});
  setf(opts->highlights, {1.0f, 1.0f, 1.0f, 0.0f});
  setf(opts->tonal_ranges, {0.0f, 0.333f, 0.550f, 1.0f});
  setf(opts->slope, {1.0f, 1.0f, 1.0f});
  setf(opts->offset, {0.0f, 0.0f, 0.0f});
  setf(opts->power, {1.0f, 1.0f, 1.0f});
  setf(opts->shadow_gamma, {1.0f, 1.0f, 1.0f});
  setf(opts->mid_point, {1.0f, 1.0f, 1.0f});
  setf(opts->highlight_scale, {1.0f, 1.0f, 1.0f});
}

void mjrf_defaultAntialiasingOpts(mjrfAntialiasingOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfAntialiasingOpts));
  opts->fxaa_enable = true;
  opts->tsaa_enable = false;
  opts->msaa_samples = 0;
}

void mjrf_defaultAmbientOcclusionOpts(mjrfAmbientOcclusionOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfAmbientOcclusionOpts));
  opts->type = mjAMBIENT_OCCLUSION_TYPE_SCALABLE;
  opts->ssct_enable = false;
  opts->strength = 1.0f;
  opts->power = 1.0f;
  opts->radius = 0.3f;
  opts->quality = mjQUALITY_LOW;
  opts->low_pass_filter = mjQUALITY_MEDIUM;
  opts->upsampling = mjQUALITY_LOW;
  opts->bent_normals = false;
}

void mjrf_defaultRenderOpts(mjrfRenderOpts* opts) {
  std::memset(opts, 0, sizeof(mjrfRenderOpts));
  mjrf_defaultAmbientOcclusionOpts(&opts->ambient_occlusion);
  mjrf_defaultAntialiasingOpts(&opts->antialiasing);
  mjrf_defaultBloomOpts(&opts->bloom);
  mjrf_defaultColorGradingOpts(&opts->color_grading);
  mjrf_defaultDepthOfFieldOpts(&opts->depth_of_field);
  mjrf_defaultDitheringOpts(&opts->dithering);
  mjrf_defaultFogOpts(&opts->fog);
  mjrf_defaultHazeOpts(&opts->haze);
  mjrf_defaultHeadlightOpts(&opts->headlight);
  mjrf_defaultMaterialOpts(&opts->material);
  mjrf_defaultReflectionOpts(&opts->reflection);
  mjrf_defaultShadowOpts(&opts->shadow);
  mjrf_defaultSkyboxOpts(&opts->skybox);
  mjrf_defaultVignetteOpts(&opts->vignette);
  opts->post_processing = true;
}

void mjrf_destroyContext(mjrfContext* ctx) {
  MJRF_CALL(ctx, destroyContext);
}

void mjrf_getRendererInfo(mjrfContext* ctx, mjrRendererInfo* info) {
  memset(info, 0, sizeof(mjrRendererInfo));
  info->renderer = "filament";
  info->backend = "";
  MJRF_CALL(ctx, getRendererInfo, info);
}

mjrfTexture* mjrf_createTexture(mjrfContext* ctx,
                                const mjrfTextureConfig* config) {
  return MJRF_CALL(ctx, createTexture, config);
}

void mjrf_destroyTexture(mjrfTexture* texture) {
  MJRF_CALL(texture, destroyTexture);
}

mjrfMesh* mjrf_createMesh(mjrfContext* ctx, const mjrfMeshConfig* config) {
  return MJRF_CALL(ctx, createMesh, config);
}

void mjrf_destroyMesh(mjrfMesh* mesh) {
  MJRF_CALL(mesh, destroyMesh);
}

mjrfScene* mjrf_createScene(mjrfContext* ctx, const mjrfSceneParams* params) {
  return MJRF_CALL(ctx, createScene, params);
}

void mjrf_destroyScene(mjrfScene* scene) {
  MJRF_CALL(scene, destroyScene);
}

mjrfLight* mjrf_createLight(mjrfContext* ctx, const mjrfLightParams* params) {
  return MJRF_CALL(ctx, createLight, params);
}

void mjrf_destroyLight(mjrfLight* light) {
  MJRF_CALL(light, destroyLight);
}

mjrfRenderable* mjrf_createRenderable(mjrfContext* ctx,
                                      const mjrfRenderableParams* params) {
  return MJRF_CALL(ctx, createRenderable, params);
}

void mjrf_destroyRenderable(mjrfRenderable* renderable) {
  MJRF_CALL(renderable, destroyRenderable);
}

mjrfRenderTarget* mjrf_createRenderTarget(
    mjrfContext* ctx, const mjrfRenderTargetConfig* config) {
  return MJRF_CALL(ctx, createRenderTarget, config);
}

void mjrf_destroyRenderTarget(mjrfRenderTarget* render_target) {
  MJRF_CALL(render_target, destroyRenderTarget);
}

void mjrf_uploadTexture(mjrfTexture* texture, const mjrfTextureData* data) {
  MJRF_CALL(texture, uploadTexture, data);
}

void mjrf_uploadMesh(mjrfMesh* mesh, const mjrfMeshData* data) {
  MJRF_CALL(mesh, uploadMesh, data);
}

int mjrf_getTextureWidth(const mjrfTexture* texture) {
  return MJRF_CALL(texture, getTextureWidth);
}

int mjrf_getTextureHeight(const mjrfTexture* texture) {
  return MJRF_CALL(texture, getTextureHeight);
}

int mjrf_getTextureSamplerType(const mjrfTexture* texture) {
  return MJRF_CALL(texture, getTextureSamplerType);
}

void mjrf_setLightActive(mjrfLight* light, mjtBool active) {
  MJRF_CALL(light, setLightActive, active);
}

void mjrf_setLightCastShadow(mjrfLight* light, mjtBool enabled) {
  MJRF_CALL(light, setLightCastShadow, enabled);
}

void mjrf_setLightColor(mjrfLight* light, const float color[3]) {
  MJRF_CALL(light, setLightColor, color);
}

void mjrf_setLightIntensity(mjrfLight* light, float intensity) {
  MJRF_CALL(light, setLightIntensity, intensity);
}

void mjrf_setLightRange(mjrfLight* light, float range) {
  MJRF_CALL(light, setLightRange, range);
}

void mjrf_setLightCutoff(mjrfLight* light, float cutoff) {
  MJRF_CALL(light, setLightCutoff, cutoff);
}

void mjrf_setLightSoftness(mjrfLight* light, float softness) {
  MJRF_CALL(light, setLightSoftness, softness);
}

void mjrf_setLightBulbRadius(mjrfLight* light, float radius) {
  MJRF_CALL(light, setLightBulbRadius, radius);
}

void mjrf_setLightShadowMapSize(mjrfLight* light, int map_size) {
  MJRF_CALL(light, setLightShadowMapSize, map_size);
}

void mjrf_setLightPose(mjrfLight* light, const float position[3],
                       const float direction[3]) {
  MJRF_CALL(light, setLightPose, position, direction);
}

int mjrf_getLightType(const mjrfLight* light) {
  return MJRF_CALL(light, getLightType);
}

void mjrf_setRenderableMesh(mjrfRenderable* renderable, const mjrfMesh* mesh,
                            int elem_offset, int elem_count) {
  MJRF_CALL(renderable, setRenderableMesh, mesh, elem_offset, elem_count);
}

void mjrf_setRenderableGeomMesh(mjrfRenderable* renderable, int type,
                                int nstack, int nslice, int nquad) {
  MJRF_CALL(renderable, setRenderableGeomMesh, type, nstack, nslice, nquad);
}

void mjrf_setRenderableMaterial(mjrfRenderable* renderable,
                                const mjrfMaterial* material) {
  MJRF_CALL(renderable, setRenderableMaterial, material);
}

void mjrf_getRenderableMaterial(mjrfRenderable* renderable,
                                mjrfMaterial* material) {
  MJRF_CALL(renderable, getRenderableMaterial, material);
}

void mjrf_setRenderablePose(mjrfRenderable* renderable,
                            const float position[3],
                            const float rotation[9]) {
  MJRF_CALL(renderable, setRenderablePose, position, rotation);
}

void mjrf_setRenderableSize(mjrfRenderable* renderable, const float size[3]) {
  MJRF_CALL(renderable, setRenderableSize, size);
}

void mjrf_setRenderableCastShadow(mjrfRenderable* renderable, mjtBool enabled) {
  MJRF_CALL(renderable, setRenderableCastShadow, enabled);
}

void mjrf_addSceneLight(mjrfScene* scene, mjrfLight* light) {
  MJRF_CALL(scene, addSceneLight, light);
}

void mjrf_removeSceneLight(mjrfScene* scene, mjrfLight* light) {
  MJRF_CALL(scene, removeSceneLight, light);
}

void mjrf_addSceneRenderable(mjrfScene* scene, mjrfRenderable* renderable) {
  MJRF_CALL(scene, addSceneRenderable, renderable);
}

void mjrf_removeSceneRenderable(mjrfScene* scene,
                                mjrfRenderable* renderable) {
  MJRF_CALL(scene, removeSceneRenderable, renderable);
}

void mjrf_setSceneSkybox(mjrfScene* scene, const mjrfTexture* texture) {
  MJRF_CALL(scene, setSceneSkybox, texture);
}

void mjrf_configureSceneFromModel(mjrfScene* scene, const mjModel* model) {
  MJRF_CALL(scene, configureSceneFromModel, model);
}

void mjrf_setSceneMsaa(mjrfScene* scene, int samples) {
  MJRF_CALL(scene, setSceneMsaa, samples);
}

void mjrf_resizeRenderTarget(mjrfRenderTarget* render_target, int width,
                             int height) {
  MJRF_CALL(render_target, resizeRenderTarget, width, height);
}

mjrfFrameHandle mjrf_render(mjrfContext* ctx, const mjrfRenderRequest* req,
                            int nreq, const mjrfReadRequest* read_req,
                            int nread_req) {
  return MJRF_CALL(ctx, render, req, nreq, read_req, nread_req);
}

void mjrf_waitForFrame(mjrfContext* ctx, mjrfFrameHandle frame) {
  MJRF_CALL(ctx, waitForFrame, frame);
}

void mjrf_flush(mjrfContext* ctx) {
  MJRF_CALL(ctx, flush);
}

void mjrf_setClearColor(mjrfContext* ctx, const float color[3]) {
  MJRF_CALL(ctx, setClearColor, color);
}

void mjrf_getFrameStats(mjrfContext* ctx, mjrfFrameHandle frame,
                        mjrfFrameStats* stats_out) {
  MJRF_CALL(ctx, getFrameStats, frame, stats_out);
}

void mjrf_modelRenderOpts(mjrfRenderOpts* opts, const mjModel* model) {
  // Helper class for reading custom flags from the model.
  ModelReader read(model);

  mjrf_defaultRenderOpts(opts);

  // If there are no physical lights in the model, we use defaults that match the classic renderer.
  // Users can still override these defaults using custom flags.
  bool has_physical_lights = false;
  for (int i = 0; i < model->nlight; ++i) {
    if (model->light_intensity[i] > 0) {
      has_physical_lights = true;
      break;
    }
  }

  // Headlight is either off, on, or auto (default). We treat "auto" headlights differently depending on if we're trying
  // to match the classic renderer or not.
  auto& headlight = model->vis.headlight;

  float headlight_base_intensity = 150'000.f;
  read(headlight_base_intensity, "filament.fallback.head_light_intensity");
  const float headlight_strength =
      (headlight.diffuse[0] + headlight.diffuse[1] + headlight.diffuse[2]) /
      3.0f;

  if (model->vis.headlight.active == 1) {
    opts->headlight.enable = true;
    opts->headlight.intensity = headlight_base_intensity * headlight_strength;
  }
  if (!has_physical_lights) {
    // Classic renderer treats "auto" active headlights as always on.
    if (model->vis.headlight.active == 2) {
      opts->headlight.enable = true;
      opts->headlight.intensity = headlight_base_intensity * headlight_strength;
    }

    // Classic renderer has no color grading so use linear output.
    opts->color_grading.tone_mapper = mjTONE_MAPPER_LINEAR;
    opts->color_grading.output_color_space = mjCOLORSPACE_LINEAR;
  }

  // mjrfAmbientOcclusionOpts
  read(opts->ambient_occlusion.type, "filament.ao.enabled");  // backwards compatibility
  read(opts->ambient_occlusion.type, "filament.ao.type", mjrAmbientOcclusionType{});
  read(opts->ambient_occlusion.ssct_enable, "filament.ao.ssct");  // backwards compatibility
  read(opts->ambient_occlusion.ssct_enable, "filament.ao.ssct_enabled");
  read(opts->ambient_occlusion.strength, "filament.ao.strength");
  read(opts->ambient_occlusion.power, "filament.ao.power");
  read(opts->ambient_occlusion.quality, "filament.ao.quality", mjrQuality{});
  read(opts->ambient_occlusion.low_pass_filter, "filament.ao.low_pass_filter", mjrQuality{});
  read(opts->ambient_occlusion.upsampling, "filament.ao.upsampling", mjrQuality{});
  read(opts->ambient_occlusion.radius, "filament.ao.radius");
  read(opts->ambient_occlusion.bent_normals, "filament.ao.bent_normals");

  // mjrfAntialiasingOpts
  read(opts->antialiasing.fxaa_enable, "filament.aa.fxaa");
  read(opts->antialiasing.tsaa_enable, "filament.aa.tsaa");
  opts->antialiasing.msaa_samples = model->vis.quality.offsamples;

  // mjrfBloomOpts
  read(opts->bloom.enable, "filament.bloom.enabled");
  read(opts->bloom.quality, "filament.bloom.quality", mjrQuality{});
  read(opts->bloom.strength, "filament.bloom.strength");
  read(opts->bloom.levels, "filament.bloom.levels");
  read(opts->bloom.lens_flare, "filament.bloom.lens_flare");
  read(opts->bloom.starburst, "filament.bloom.starburst");

  // mjrfColorGradingOpts
  read(opts->color_grading.tone_mapper, "filament.cg.tone_mapping", mjrToneMapper{});
  read(opts->color_grading.output_color_space, "filament.cg.output_color_space", mjrColorSpace{});
  read(opts->color_grading.exposure, "filament.cg.exposure");
  read(opts->color_grading.night_adaptation, "filament.cg.night_adaptation");
  read(opts->color_grading.contrast, "filament.cg.contrast");
  read(opts->color_grading.vibrance, "filament.cg.vibrance");
  read(opts->color_grading.saturation, "filament.cg.saturation");
  read(opts->color_grading.temperature, "filament.cg.temperature");
  read(opts->color_grading.tint, "filament.cg.tint");
  read(opts->color_grading.out_red, "filament.cg.out_red");
  read(opts->color_grading.out_green, "filament.cg.out_green");
  read(opts->color_grading.out_blue, "filament.cg.out_blue");
  read(opts->color_grading.shadows, "filament.cg.shadows");
  read(opts->color_grading.midtones, "filament.cg.midtones");
  read(opts->color_grading.highlights, "filament.cg.highlights");
  read(opts->color_grading.tonal_ranges, "filament.cg.tonal_ranges");
  read(opts->color_grading.slope, "filament.cg.slope");
  read(opts->color_grading.offset, "filament.cg.offset");
  read(opts->color_grading.power, "filament.cg.power");
  read(opts->color_grading.shadow_gamma, "filament.cg.shadow_gamma");
  read(opts->color_grading.mid_point, "filament.cg.mid_point");
  read(opts->color_grading.highlight_scale, "filament.cg.highlight_scale");

  // mjrfDepthOfFieldOpts
  read(opts->depth_of_field.enable, "filament.dof.enabled");
  read(opts->depth_of_field.blur_strength, "filament.dof.blur_strength");
  read(opts->depth_of_field.focus_distance, "filament.dof.focus_distance");
  read(opts->depth_of_field.aspect_ratio, "filament.dof.aspect_ratio");
  read(opts->depth_of_field.max_aperture_diameter, "filament.dof.max_aperture_diameter");

  // mjrfDitheringOpts
  read(opts->dithering.type, "filament.dithering.type", mjrDitheringType{});

  // mjrfFogOpts
  read(opts->fog.enable, "filament.fog.enabled");
  read(opts->fog.color, "filament.fog.color");
  read(opts->fog.density, "filament.fog.density");
  read(opts->fog.max_opacity, "filament.fog.maximum_opacity");
  read(opts->fog.distance, "filament.fog.distance");
  read(opts->fog.cutoff, "filament.fog.cut_off_distance");
  read(opts->fog.height, "filament.fog.height");
  read(opts->fog.height_falloff, "filament.fog.height_falloff");

  // mjrfHazeOpts
  read(opts->haze.enable, "filament.haze.enabled");
  setf(opts->haze.color, model->vis.rgba.haze);
  opts->haze.ratio = model->vis.map.haze;
  opts->haze.slices = model->vis.quality.numslices;

  // mjrfHeadlightOpts, handled explicitly above.

  // mjrfMaterialOpts
  read(opts->material.rgb_color_type, "filament.material.base_color_type", mjrColorSpace{});
  read(opts->material.enable_textures, "filament.material.enable_textures");

  // mjrfReflectionOpts
  read(opts->reflection.enable, "filament.reflections.enabled");

  // mjrfShadowOpts
  read(opts->shadow.type, "filament.shadows.type", mjrShadowType{});

  // mjrfSkyboxOpts
  read(opts->skybox.enable, "filament.skybox.enabled");

  // mjrfVignetteOpts
  read(opts->vignette.enable, "filament.vignette.enabled");
  read(opts->vignette.color, "filament.vignette.color");
  read(opts->vignette.mid_point, "filament.vignette.midpoint");
  read(opts->vignette.roundness, "filament.vignette.roundness");
  read(opts->vignette.feather, "filament.vignette.feather");
}

}  // extern "C"
