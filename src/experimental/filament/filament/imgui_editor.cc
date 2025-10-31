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

#include "experimental/filament/filament/imgui_editor.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include <imgui.h>
#include <filament/ColorGrading.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Options.h>
#include <filament/View.h>
#include <math/mathfwd.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/scene_view.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4;

using NameList = std::span<const char*>;

NameList EnumNames(ToneMapperType v) {
  static const char* names[] = {"PBR Neutral", "ACES", "ACES Legacy", "Filmic",
                                "Linear"};
  return names;
}

NameList EnumNames(filament::QualityLevel v) {
  static const char* names[] = {"LOW", "MEDIUM", "HIGH", "ULTRA"};
  return names;
}

NameList EnumNames(filament::ShadowType v) {
  static const char* names[] = {"PCF", "VSM", "DPCF", "PCSS"};
  return names;
}

NameList EnumNames(filament::TemporalAntiAliasingOptions::BoxType v) {
  static const char* names[] = {"AABB", "VARIANCE", "AABB_VARIANCE"};
  return names;
}

NameList EnumNames(filament::TemporalAntiAliasingOptions::BoxClipping v) {
  static const char* names[] = {"ACCURATE", "CLAMP", "NONE"};
  return names;
}

NameList EnumNames(filament::TemporalAntiAliasingOptions::JitterPattern v) {
  static const char* names[] = {"RGSS_X4", "UNIFORM_HELIX_X4", "HALTON_23_X8",
                                "HALTON_23_X16", "HALTON_23_X32"};
  return names;
}

NameList EnumNames(filament::ColorGrading::LutFormat v) {
  static const char* names[] = {"Integer", "Float"};
  return names;
}

NameList EnumNames(filament::BloomOptions::BlendMode v) {
  static const char* names[] = {"Add", "Interpolate"};
  return names;
}

NameList EnumNames(filament::DepthOfFieldOptions::Filter v) {
  static const char* names[] = {"NONE", "UNUSED", "MEDIAN"};
  return names;
}

template <typename T>
struct UiOpts {
  std::optional<T> min;
  std::optional<T> max;
  std::optional<T> step;
  std::optional<T> fstep;
};

template <typename T>
bool Ui(std::string_view label, T* value, UiOpts<T> opts = {}) {
  bool changed = false;

  if constexpr (std::is_enum_v<T>) {
    int idx = static_cast<int>(*value);
    const NameList names = EnumNames(*value);
    changed = ImGui::Combo(label.data(), &idx, names.data(), names.size());
    *value = static_cast<T>(idx);
  } else if constexpr (std::is_same_v<T, bool>) {
    changed = ImGui::Checkbox(label.data(), value);
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    changed = ImGui::InputScalar(label.data(), ImGuiDataType_U8, value);
  } else if constexpr (std::is_same_v<T, uint16_t>) {
    changed = ImGui::InputScalar(label.data(), ImGuiDataType_U16, value);
  } else if constexpr (std::is_same_v<T, uint32_t>) {
    changed = ImGui::InputScalar(label.data(), ImGuiDataType_U32, value);
  } else if constexpr (std::is_same_v<T, int>) {
    if (opts.min.has_value() && opts.max.has_value()) {
      changed = ImGui::SliderInt(label.data(), value, *opts.min, *opts.max);
    } else {
      int step = opts.step.value_or(1);
      int fast_step = opts.fstep.value_or(100);
      changed = ImGui::InputInt(label.data(), value, step, fast_step);
    }
  } else if constexpr (std::is_same_v<T, float>) {
    if (opts.min.has_value() && opts.max.has_value()) {
      changed = ImGui::SliderFloat(label.data(), value, *opts.min, *opts.max);
    } else {
      float step = opts.step.value_or(0.f);
      float fast_step = opts.fstep.value_or(0.f);
      changed = ImGui::InputFloat(label.data(), value, step, fast_step);
    }
  } else if constexpr (std::is_same_v<T, float3>) {
    changed = ImGui::InputFloat3(label.data(), &value->x);
  } else if constexpr (std::is_same_v<T, float4>) {
    changed = ImGui::InputFloat4(label.data(), &value->x);
  } else {
    static_assert(false, "Unsupported type");
  }
  return changed;
}

void DrawAmbientOcclusionGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getAmbientOcclusionOptions();

  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Radius (m)", &opts.radius);
  changed |= Ui("Power Contrrast", &opts.power);
  changed |= Ui("Bias (m)", &opts.bias);
  changed |= Ui("Resolution Scale", &opts.resolution);
  changed |= Ui("Intensity", &opts.intensity);
  changed |= Ui("Bend Normals", &opts.bentNormals);
  changed |= Ui("Bilateral Threshold", &opts.bilateralThreshold);
  changed |= Ui("Min. Horizon Angle (Rad)", &opts.minHorizonAngleRad);
  changed |= Ui("Quality", &opts.quality);
  changed |= Ui("Low Pass Filter", &opts.lowPassFilter);
  changed |= Ui("Upsampling", &opts.upsampling);
  changed |= Ui("SSCT Enabled", &opts.ssct.enabled);
  changed |= Ui("Cone Angle (Rad)", &opts.ssct.lightConeRad);
  changed |= Ui("Shadow Distance (m)", &opts.ssct.shadowDistance);
  changed |= Ui("Max Contact Distance (m)", &opts.ssct.contactDistanceMax);
  changed |= Ui("Intensity", &opts.ssct.intensity);
  changed |= Ui("Deeth Bias", &opts.ssct.depthBias);
  changed |= Ui("Depth Slope Bias", &opts.ssct.depthSlopeBias);
  changed |= Ui("Sample Count", &opts.ssct.sampleCount);
  changed |= Ui("Raw Count", &opts.ssct.rayCount);
  changed |= Ui("Light Direction", &opts.ssct.lightDirection);

  if (changed) {
    opts.resolution = (opts.resolution < 0.75f) ? 0.f : 1.0f;
    opts.power = std::max(opts.power, 0.0f);
    opts.radius = std::clamp(opts.radius, 0.0f, 10.0f);
    opts.bias = std::clamp(opts.bias, 0.0f, 0.001f);
    opts.ssct.lightConeRad = std::clamp(opts.ssct.lightConeRad, 0.0f,
                                        filament::math::f::PI / 2.0f);

    view->setAmbientOcclusionOptions(opts);
  }
}

void DrawScreenSpaceGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto reflection = view->getScreenSpaceReflectionsOptions();
  bool refraction_enabled = view->isScreenSpaceRefractionEnabled();
  auto guard_band = view->getGuardBandOptions();

  bool changed = false;
  changed |= Ui("Reflection Enabled", &reflection.enabled);
  changed |= Ui("Refl. Thickness", &reflection.thickness);
  changed |= Ui("Refl. Bias", &reflection.bias);
  changed |= Ui("Refl. Max Distance", &reflection.maxDistance);
  changed |= Ui("Refl. Stride", &reflection.stride);
  if (changed) {
    view->setScreenSpaceReflectionsOptions(reflection);
  }
  if (Ui("Refraction", &refraction_enabled)) {
    view->setScreenSpaceRefractionEnabled(refraction_enabled);
  }
  if (Ui("Guard Band Enabled", &guard_band.enabled)) {
    view->setGuardBandOptions(guard_band);
  }
}

void DrawShadowingGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto vsm_opts = view->getVsmShadowOptions();
  auto soft_opts = view->getSoftShadowOptions();

  bool enabled = view->isShadowingEnabled();
  if (Ui("Shadowing", &enabled)) {
    view->setShadowingEnabled(enabled);
  }
  auto shadow_type = view->getShadowType();
  if (Ui("Shadow Type", &shadow_type)) {
    view->setShadowType(shadow_type);
  }

  bool changed = false;
  changed |= Ui("VSM Anisotropy", &vsm_opts.anisotropy);
  changed |= Ui("VSM Mipmapping", &vsm_opts.mipmapping);
  changed |= Ui("VSM MSAA Samples", &vsm_opts.msaaSamples);
  changed |= Ui("VSM High Precision", &vsm_opts.highPrecision);
  changed |= Ui("VSM Min Variance Scale", &vsm_opts.minVarianceScale);
  changed |= Ui("VSM Light Bleed Reduction", &vsm_opts.lightBleedReduction);
  if (changed) {
    vsm_opts.minVarianceScale = std::max(vsm_opts.minVarianceScale, 0.0f);
    vsm_opts.lightBleedReduction =
        std::clamp(vsm_opts.lightBleedReduction, 0.0f, 1.0f);

    view->setVsmShadowOptions(vsm_opts);
  }

  changed = false;
  changed |= Ui("Soft Penumbra Scale", &soft_opts.penumbraScale);
  changed |= Ui("Soft Penumbra Ratio Scale", &soft_opts.penumbraRatioScale);
  if (changed) {
    soft_opts.penumbraScale = std::max(soft_opts.penumbraScale, 0.0f);
    soft_opts.penumbraRatioScale = std::max(soft_opts.penumbraRatioScale, 1.0f);

    view->setSoftShadowOptions(soft_opts);
  }
}

void DrawPostProcessingGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  bool enabled = view->isPostProcessingEnabled();
  if (Ui("Enabled", &enabled)) {
    view->setPostProcessingEnabled(enabled);
  }
}

void DrawFxaaGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  bool enabled = view->getAntiAliasing() == filament::AntiAliasing::FXAA;
  if (Ui("Enabled", &enabled)) {
    view->setAntiAliasing(enabled ? filament::AntiAliasing::FXAA
                                  : filament::AntiAliasing::NONE);
  }
}

void DrawMsaaGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getMultiSampleAntiAliasingOptions();

  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Samples", &opts.sampleCount);
  changed |= Ui("Custom Resolve", &opts.customResolve);
  if (changed) {
    view->setMultiSampleAntiAliasingOptions(opts);
  }
}

void DrawTaaGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getTemporalAntiAliasingOptions();

  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Filter Width", &opts.filterWidth);
  changed |= Ui("Feedback", &opts.feedback, {.min = 0.f, .max = 1.f});
  changed |= Ui("Lod Bias", &opts.lodBias);
  changed |= Ui("Sharpness", &opts.sharpness);
  changed |= Ui("Upscaling", &opts.upscaling);
  changed |= Ui("Filter History", &opts.filterHistory);
  changed |= Ui("Filter Input", &opts.filterInput);
  changed |= Ui("Use YCoCg", &opts.useYCoCg);
  changed |= Ui("Box Type", &opts.boxType);
  changed |= Ui("Box Clipping", &opts.boxClipping);
  changed |= Ui("Jitter Pattern", &opts.jitterPattern);
  changed |= Ui("Variance Gamma", &opts.varianceGamma);
  changed |= Ui("Prevent Flickering", &opts.preventFlickering);

  if (changed) {
    opts.feedback = std::clamp(opts.feedback, 0.f, 1.f);
    opts.varianceGamma = std::clamp(opts.varianceGamma, 0.75f, 1.25f);
    view->setTemporalAntiAliasingOptions(opts);
  }
}

void DrawBloomGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getBloomOptions();
  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Strength", &opts.strength);
  changed |= Ui("Quality", &opts.quality);
  changed |= Ui("Resolution", &opts.resolution);
  changed |= Ui("Levels", &opts.levels);
  changed |= Ui("Blend Mode", &opts.blendMode);
  changed |= Ui("Threshold", &opts.threshold);
  changed |= Ui("Highlight", &opts.highlight);
  changed |= Ui("Lens Flare", &opts.lensFlare);
  changed |= Ui("Starburst", &opts.starburst);
  changed |= Ui("Ch. Aberration", &opts.chromaticAberration);
  changed |= Ui("Ghost Count", &opts.ghostCount);
  changed |= Ui("Ghost Spacing", &opts.ghostSpacing);
  changed |= Ui("Ghost Threshold", &opts.ghostThreshold);
  changed |= Ui("Halo Radius", &opts.haloRadius);
  changed |= Ui("Halo Thickness", &opts.haloThickness);
  changed |= Ui("Halo Threshold", &opts.haloThreshold);

  if (changed) {
    opts.strength = std::clamp(opts.strength, 0.0f, 1.0f);
    opts.levels = std::clamp<uint8_t>(opts.levels, 1, 11);
    opts.ghostSpacing = std::clamp(opts.ghostSpacing, 0.0f, 1.0f);
    opts.highlight = std::max(opts.highlight, 10.0f);
    opts.haloThickness = std::clamp(opts.haloThickness, 0.0f, 1.0f);
    opts.haloRadius = std::clamp(opts.haloRadius, 0.0f, 0.5f);
    view->setBloomOptions(opts);
  }
}

void DrawColorGradingGui(SceneView* scene_view) {
  auto opts = scene_view->GetColorGradingOptions();

  bool changed = false;
  changed |= Ui("Tone Mapper", &opts.tone_mapper);
  changed |= Ui("Lut Format", &opts.format);
  changed |= Ui("Lut Dimension", &opts.dimension);
  changed |= Ui("Luminance Scaling", &opts.luminance_scaling);
  changed |= Ui("Gamut Mapping", &opts.gamut_mapping);
  changed |= Ui("Exposure", &opts.exposure, {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Night Adaptation", &opts.night_adaptation,
                {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Contrast", &opts.contrast, {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Vibrance", &opts.vibrance, {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Saturation", &opts.saturation,
                {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Temperature", &opts.temperature,
                {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Tint", &opts.tint, {.step = 0.01f, .fstep = 0.1f});
  changed |= Ui("Out Red", &opts.out_red);
  changed |= Ui("Out Green", &opts.out_green);
  changed |= Ui("Out Blue", &opts.out_blue);
  changed |= Ui("Shadows", &opts.shadows);
  changed |= Ui("Midtones", &opts.midtones);
  changed |= Ui("Highlights", &opts.highlights);
  changed |= Ui("Tonal Ranges", &opts.tonal_ranges);
  changed |= Ui("Slope", &opts.slope);
  changed |= Ui("Offset", &opts.offset);
  changed |= Ui("Power", &opts.power);
  changed |= Ui("Shadow Gamma", &opts.shadow_gamma);
  changed |= Ui("Mid Point", &opts.mid_point);
  changed |= Ui("Highlight Scale", &opts.highlight_scale);

  opts.contrast = std::clamp(opts.contrast, 0.0f, 2.0f);
  opts.vibrance = std::clamp(opts.vibrance, 0.0f, 2.0f);
  opts.saturation = std::clamp(opts.saturation, 0.0f, 2.0f);
  opts.temperature = std::clamp(opts.temperature, -1.0f, 1.0f);
  opts.tint = std::clamp(opts.tint, -1.0f, 1.0f);
  opts.slope.x = std::max(0.000001f, opts.slope.x);
  opts.slope.y = std::max(0.000001f, opts.slope.y);
  opts.slope.z = std::max(0.000001f, opts.slope.z);
  opts.power.x = std::max(0.000001f, opts.power.x);
  opts.power.y = std::max(0.000001f, opts.power.y);
  opts.power.z = std::max(0.000001f, opts.power.z);
  opts.shadow_gamma.x = std::max(0.000001f, opts.shadow_gamma.x);
  opts.shadow_gamma.y = std::max(0.000001f, opts.shadow_gamma.y);
  opts.shadow_gamma.z = std::max(0.000001f, opts.shadow_gamma.z);
  opts.mid_point.x = std::max(0.000001f, opts.mid_point.x);
  opts.mid_point.y = std::max(0.000001f, opts.mid_point.y);
  opts.mid_point.z = std::max(0.000001f, opts.mid_point.z);
  if (opts.highlight_scale.x == 0.0f) opts.highlight_scale.x = 0.000001f;
  if (opts.highlight_scale.y == 0.0f) opts.highlight_scale.y = 0.000001f;
  if (opts.highlight_scale.z == 0.0f) opts.highlight_scale.z = 0.000001f;

  if (changed) {
    scene_view->SetColorGradingOptions(opts);
  }
}

void DrawDepthOfFieldGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getDepthOfFieldOptions();
  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Filter", &opts.filter);
  changed |= Ui("Max. Aperture", &opts.maxApertureDiameter);
  changed |= Ui("Resolution", &opts.nativeResolution);
  changed |= Ui("COC Scale", &opts.cocScale);
  changed |= Ui("COC Aspect Ratio", &opts.cocAspectRatio);
  changed |= Ui("Max COC (Foreground)", &opts.maxForegroundCOC);
  changed |= Ui("Max COC (Background)", &opts.maxBackgroundCOC);
  changed |= Ui("Forground Ring Count", &opts.foregroundRingCount);
  changed |= Ui("Background Ring Count", &opts.backgroundRingCount);
  changed |= Ui("Fast Gather Ring Count", &opts.fastGatherRingCount);
  if (changed) {
    view->setDepthOfFieldOptions(opts);
  }
}

void DrawDitheringGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  bool enabled = view->getDithering() != filament::Dithering::NONE;
  if (Ui("Enabled", &enabled)) {
    view->setDithering(enabled ? filament::Dithering::TEMPORAL
                               : filament::Dithering::NONE);
  }
}

void DrawFogGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getFogOptions();
  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Color", &opts.color);
  changed |= Ui("Use IBL Color", &opts.fogColorFromIbl);
  changed |= Ui("Density", &opts.density);
  changed |= Ui("Distance", &opts.distance);
  changed |= Ui("Cutoff", &opts.cutOffDistance);
  changed |= Ui("Max. Opacity", &opts.maximumOpacity);
  changed |= Ui("Height", &opts.height);
  changed |= Ui("Height Falloff", &opts.heightFalloff);
  changed |= Ui("InScattering Start", &opts.inScatteringStart);
  changed |= Ui("InScattering Size", &opts.inScatteringSize);
  if (changed) {
    opts.maximumOpacity = std::clamp(opts.maximumOpacity, 0.0f, 1.0f);
    view->setFogOptions(opts);
  }
}

void DrawVignetteGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto opts = view->getVignetteOptions();
  bool changed = false;
  changed |= Ui("Enabled", &opts.enabled);
  changed |= Ui("Color", &opts.color);
  changed |= Ui("Midpoint", &opts.midPoint);
  changed |= Ui("Roundness", &opts.roundness);
  changed |= Ui("Feature", &opts.feather);

  if (changed) {
    opts.midPoint = std::clamp(opts.midPoint, 0.0f, 1.0f);
    opts.roundness = std::clamp(opts.roundness, 0.0f, 1.0f);
    opts.feather = std::clamp(opts.feather, 0.0f, 1.0f);
    view->setVignetteOptions(opts);
  }
}

void DrawVisibleLayersGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  uint32_t layers = view->getVisibleLayers();
  bool changed = false;

  ImGui::Columns(2);
  changed |= ImGui::CheckboxFlags("Layer 0", &layers, 1 << 0);
  changed |= ImGui::CheckboxFlags("Layer 1", &layers, 1 << 1);
  changed |= ImGui::CheckboxFlags("Layer 2", &layers, 1 << 2);
  changed |= ImGui::CheckboxFlags("Layer 3", &layers, 1 << 3);
  ImGui::NextColumn();
  changed |= ImGui::CheckboxFlags("Layer 4", &layers, 1 << 4);
  changed |= ImGui::CheckboxFlags("Layer 5", &layers, 1 << 5);
  changed |= ImGui::CheckboxFlags("Layer 6", &layers, 1 << 6);
  changed |= ImGui::CheckboxFlags("Layer 7", &layers, 1 << 7);
  ImGui::Columns(1);
  if (changed) {
    view->setVisibleLayers(0xff, static_cast<uint8_t>(layers));
  }
}

void DrawCameraGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  filament::Camera& camera = view->getCamera();
  auto position = float3(camera.getPosition());
  Ui("Position", &position);
  auto direction = camera.getForwardVector();
  Ui("Direction", &direction);
}

void DrawIndirectLightGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  auto ibl = view->getScene()->getIndirectLight();

  float intensity = 10000.0f;
  if (ibl) {
    intensity = ibl->getIntensity();
    if (Ui("Intensity", &intensity, {.step = 1000.0f, .fstep = 10000.0f})) {
      ibl->setIntensity(intensity);
    }
  }

  static char filename[256];
  ImGui::InputText("Filename", filename, sizeof(filename));
  if (ImGui::Button("Load")) {
    scene_view->SetEnvironmentLight(filename, intensity);
  }
}

void DrawLightGui(filament::LightManager& lm,
                  filament::LightManager::Instance li) {
  auto color = lm.getColor(li);
  if (ImGui::ColorEdit3("Color", &color.x)) {
    lm.setColor(li, color);
  }
  auto position = lm.getPosition(li);
  if (Ui("Position", &position)) {
    lm.setPosition(li, position);
  }
  auto direction = lm.getDirection(li);
  if (Ui("Direction", &direction)) {
    lm.setDirection(li, direction);
  }
  float intensity = lm.getIntensity(li);
  if (Ui("Intensity", &intensity, {.step = 1000.0f, .fstep = 10000.0f})) {
    lm.setIntensityCandela(li, intensity);
  }
  float falloff = lm.getFalloff(li);
  if (Ui("Falloff Radius", &falloff)) {
    lm.setFalloff(li, falloff);
  }
  if (lm.isSpotLight(li)) {
    float spotLightInnerCone = lm.getSpotLightInnerCone(li);
    float spotLightOuterCone = lm.getSpotLightOuterCone(li);
    if (Ui("Spot Light Inner Cone", &spotLightInnerCone)) {
      lm.setSpotLightCone(li, spotLightInnerCone, spotLightOuterCone);
    }
    if (Ui("Spot Light Outer Cone", &spotLightOuterCone)) {
      lm.setSpotLightCone(li, spotLightInnerCone, spotLightOuterCone);
    }
  }
  ImGui::Columns(2);
  for (int i = 0; i < 8; ++i) {
    std::string name = "Channel " + std::to_string(i);
    bool enabled = lm.getLightChannel(li, i);
    if (i == 4) {
      ImGui::NextColumn();
    }
    if (Ui(name, &enabled)) {
      lm.setLightChannel(li, i, enabled);
    }
  }
  ImGui::Columns(1);
  if (ImGui::TreeNodeEx("Shadow Options")) {
    bool enabled = lm.isShadowCaster(li);
    if (Ui("Enabled", &enabled)) {
      lm.setShadowCaster(li, enabled);
    }

    filament::LightManager::ShadowOptions opts = lm.getShadowOptions(li);
    bool changed = false;
    changed |= Ui("Stable", &opts.stable);
    changed |= Ui("LiSPSM", &opts.lispsm);
    changed |= Ui("SS Contact Shadows", &opts.screenSpaceContactShadows);
    changed |= Ui("Map Size", &opts.mapSize);
    changed |= Ui("Bulb Radius", &opts.shadowBulbRadius);
    changed |= Ui("Step Count", &opts.stepCount);
    changed |= Ui("# Cascades", &opts.shadowCascades);
    changed |= Ui("Constant Bias", &opts.constantBias);
    changed |= Ui("Normal Bias", &opts.normalBias);
    if (changed) {
      lm.setShadowOptions(li, opts);
    }
    ImGui::TreePop();
  }
}

void DrawSceneViewGui(SceneView* scene_view) {
  filament::View* view = scene_view->GetDefaultRenderView();
  filament::Engine* engine = scene_view->GetEngine();
  filament::LightManager& lm = engine->getLightManager();

  if (ImGui::TreeNodeEx("Ambient Occlusion")) {
    DrawAmbientOcclusionGui(scene_view);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Screen Space")) {
    DrawScreenSpaceGui(scene_view);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Shadowing")) {
    DrawShadowingGui(scene_view);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Post Processing")) {
    DrawPostProcessingGui(scene_view);
    if (ImGui::TreeNodeEx("Anti Aliasing (FXAA)")) {
      DrawFxaaGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Anti Aliasing (MSAA)")) {
      DrawMsaaGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Anti Aliasing (Temporal)")) {
      DrawTaaGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Bloom")) {
      DrawBloomGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Color Grading")) {
      DrawColorGradingGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Depth of Field")) {
      DrawDepthOfFieldGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Dithering")) {
      DrawDitheringGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Fog")) {
      DrawFogGui(scene_view);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Vignette")) {
      DrawVignetteGui(scene_view);
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Visibility Layers")) {
    DrawVisibleLayersGui(scene_view);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Camera")) {
    DrawCameraGui(scene_view);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Lights")) {
    if (ImGui::TreeNodeEx("Indirect (Image-based) Light")) {
      DrawIndirectLightGui(scene_view);
      ImGui::TreePop();
    }
    view->getScene()->forEach([&](utils::Entity entity) {
      auto li = lm.getInstance(entity);
      if (!li.isValid()) {
        return;
      }

      const char* type = lm.isDirectional(li)  ? " (D)"
                         : lm.isPointLight(li) ? " (P)"
                                               : " (S)";
      const std::string name = "Light " + std::to_string(entity.getId()) + type;
      if (ImGui::TreeNodeEx(name.c_str())) {
        DrawLightGui(lm, li);
        ImGui::TreePop();
      }
    });
    ImGui::TreePop();
  }
}

void DrawGui(SceneView* scene_view) {
  static bool display = false;
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      ImGui::Separator();
      if (ImGui::MenuItem("Filament", "", display)) {
        display = !display;
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  if (!display) {
    return;
  }

  ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_Appearing);
  ImGui::Begin("Filament");
  DrawSceneViewGui(scene_view);
  ImGui::End();
}

}  // namespace mujoco
