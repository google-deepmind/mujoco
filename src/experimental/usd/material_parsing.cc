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

#include "experimental/usd/material_parsing.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lodepng.h"
#include <mujoco/mujoco.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/ar/asset.h>
#include <pxr/usd/ar/resolvedPath.h>
#include <pxr/usd/ar/resolver.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdShade/input.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/shader.h>
#include <pxr/usd/usdShade/types.h>
#include <pxr/usd/usdShade/udimUtils.h>

namespace mujoco {
namespace usd {

// Using to satisfy TF_DEFINE_PRIVATE_TOKENS macro below and avoid operating in
// PXR_NS.
using pxr::TfToken;
template <typename T>
using TfStaticData = pxr::TfStaticData<T>;

// clang-format off
TF_DEFINE_PRIVATE_TOKENS(kTokens,
                         ((auto_, "auto"))
                         ((diffuseColor, "diffuseColor"))
                         ((file, "file"))
                         ((metallic, "metallic"))
                         ((r, "r"))
                         ((raw, "raw"))
                         ((rgb, "rgb"))
                         ((roughness, "roughness"))
                         ((sourceColorSpace, "sourceColorSpace"))
                         ((srgb, "sRGB"))
                         ((UsdPreviewSurface, "UsdPreviewSurface"))
                         ((UsdUVTexture, "UsdUVTexture"))
                         );
// clang-format on

pxr::GfVec3f AsVec3f(pxr::GfVec3f val) { return val; }

pxr::GfVec3f AsVec3f(float val) { return pxr::GfVec3f(val, val, val); }

pxr::GfVec4f AsVec4f(pxr::GfVec4f val) { return val; }

pxr::GfVec4f AsVec4f(float val) { return pxr::GfVec4f(val, val, val, 1.0f); }

pxr::GfVec4f AsVec4f(pxr::GfVec3f vec3) {
  return pxr::GfVec4f(vec3[0], vec3[1], vec3[2], 1.0f);
}

pxr::TfToken GetShaderId(const pxr::UsdShadeShader& shader) {
  pxr::TfToken shader_id;
  shader.GetShaderId(&shader_id);
  return shader_id;
}

// Data read from a shader input.
template <typename T>
struct ResolvedShaderInput {
  // The value of the input if it is a constant.
  std::optional<T> value;

  // The value of the input if it is a texture.
  std::optional<mjsTexture*> sampler;
};

// Attempts to read an input from a USD ShadeInput as a specific type. Returns
// nullopt if the input is not found or is not of the given type.
template <typename T>
std::optional<T> ReadInput(pxr::UsdShadeInput usd_input) {
  T val;
  if (!usd_input.Get(&val)) {
    return std::nullopt;
  }
  return val;
}

// Attempts to read a UsdUVTexture shader and return a sampler.
template <typename T>
ResolvedShaderInput<T> ReadUsdUVTexture(mjSpec* spec,
                                        const pxr::UsdShadeShader shader,
                                        unsigned nchannels) {
  ResolvedShaderInput<T> out;
  mjsTexture* texture = mjs_addTexture(spec);
  texture->type = mjtTexture::mjTEXTURE_2D;
  mjs_setName(texture->element, shader.GetPath().GetAsString().c_str());

  pxr::TfToken source_color_space = kTokens->auto_;
  if (auto color_space_input = shader.GetInput(kTokens->sourceColorSpace)) {
    color_space_input.Get(&source_color_space);
  }

  if (source_color_space == kTokens->raw) {
    texture->colorspace = mjtColorSpace::mjCOLORSPACE_LINEAR;
  } else if (source_color_space == kTokens->srgb) {
    texture->colorspace = mjtColorSpace::mjCOLORSPACE_SRGB;
  } else if (source_color_space == kTokens->auto_) {
    texture->colorspace = mjtColorSpace::mjCOLORSPACE_AUTO;
  }

  pxr::ArResolver& resolver = pxr::ArGetResolver();
  pxr::SdfAssetPath resolved_texture_asset_path;
  if (auto file_input = shader.GetInput(kTokens->file)) {
    file_input.Get(&resolved_texture_asset_path);
  } else {
    mju_error("UsdUVTexture missing inputs:file.");
    return out;
  }

  std::string resolved_texture_path =
      resolved_texture_asset_path.GetResolvedPath();

  if (pxr::UsdShadeUdimUtils::IsUdimIdentifier(resolved_texture_path)) {
    mju_error("MuJoCo does not support UDIM textures: %s",
              resolved_texture_path.c_str());
    return out;
  }

  auto extension = resolver.GetExtension(resolved_texture_path);
  if (extension != "png") {
    mju_error("MuJoCo USD Parsing only supports PNG textures: %s",
              resolved_texture_path.c_str());
    return out;
  }

  FILE* fp = fopen(resolved_texture_path.c_str(), "r");
  if (fp) {
    mjs_setString(texture->content_type, "image/png");
    mjs_setString(texture->file, resolved_texture_path.c_str());
    out.sampler = texture;
    return out;
  }

  std::shared_ptr<pxr::ArAsset> texture_asset =
      resolver.OpenAsset(pxr::ArResolvedPath(resolved_texture_path));

  size_t texture_size = texture_asset->GetSize();

  uint32_t width, height;
  std::vector<unsigned char> image;
  lodepng::State state;
  if (nchannels == 3) {
    state.info_raw.colortype = LCT_RGB;
  } else if (nchannels == 1) {
    state.info_raw.colortype = LCT_GREY;
  } else {
    mju_error("MuJoCo USD Parsing only supports 1 or 3 channel textures.");
    return out;
  }

  unsigned error = lodepng::decode(
      image, width, height, state,
      reinterpret_cast<const unsigned char*>(texture_asset->GetBuffer().get()),
      texture_size);

  // check for errors
  if (error) {
    mju_error("LodePNG error %u: %s", error, lodepng_error_text(error));
    return out;
  }

  texture->width = width;
  texture->height = height;
  texture->nchannel = nchannels;
  mjs_setBuffer(texture->data, image.data(), image.size());

  out.sampler = texture;
  return out;
}

// Given an input to a shader, attempts to bake the shader and it's inputs
// into a singular value or texture sampler (ResolvedShaderInput).
template <typename T>
ResolvedShaderInput<T> ReadShaderInput(
    mjSpec* spec, const pxr::UsdShadeConnectableAPI source,
    const pxr::TfToken source_name,
    const pxr::UsdShadeAttributeType source_type) {
  ResolvedShaderInput<T> out;
  pxr::UsdShadeShader shader(source.GetPrim());
  pxr::TfToken shader_id = GetShaderId(shader);
  if (shader_id == kTokens->UsdUVTexture) {
    unsigned nchannels = -1;
    if (source_name == kTokens->rgb) {
      nchannels = 3;
    } else if (source_name == kTokens->r) {
      nchannels = 1;
    } else {
      mju_error("Unsupported texture channel: %s", source_name.GetText());
      return out;
    }

    return ReadUsdUVTexture<T>(spec, shader, nchannels);
  } else {
    mju_warning("Unsupported shader type: %s", shader_id.GetText());
  }

  return out;
}

// Reads UsdShadeInput as a value of type T or a texture sampler.
template <typename T>
ResolvedShaderInput<T> ReadShaderInput(mjSpec* spec, pxr::UsdShadeInput input) {
  ResolvedShaderInput<T> out;
  if (!input.GetPrim().IsValid()) {
    return out;
  }

  pxr::UsdShadeConnectableAPI source;
  pxr::TfToken source_name;
  pxr::UsdShadeAttributeType source_type;
  if (input.GetConnectedSource(&source, &source_name, &source_type)) {
    out = ReadShaderInput<T>(spec, source, source_name, source_type);
  } else {
    out.value = ReadInput<T>(input);
  }
  return out;
}

template <typename T, typename V>
void AssignShaderInput(mjSpec* spec, mjsMaterial* material, T* val,
                       mjtTextureRole texrole,
                       ResolvedShaderInput<V> resolved_input) {
  if (resolved_input.value.has_value()) {
    if constexpr (std::is_same_v<T, float>) {
      *val = resolved_input.value.value();
    } else if constexpr (std::is_same_v<T, float[3]>) {
      const pxr::GfVec3f resolved_value = AsVec3f(resolved_input.value.value());
      (*val)[0] = resolved_value[0];
      (*val)[1] = resolved_value[1];
      (*val)[2] = resolved_value[2];
    } else if constexpr (std::is_same_v<T, float[4]>) {
      const pxr::GfVec4f resolved_value = AsVec4f(resolved_input.value.value());
      (*val)[0] = resolved_value[0];
      (*val)[1] = resolved_value[1];
      (*val)[2] = resolved_value[2];
      (*val)[3] = resolved_value[3];
    }
  } else if (resolved_input.sampler.has_value()) {
    mjsTexture* texture = resolved_input.sampler.value();
    auto name = mjs_getName(texture->element);
    mjs_setInStringVec(material->textures, texrole, name->c_str());
  } else {
    mju_warning("No value or texture for shader input.");
  }
}

void ParsePreviewSurface(mjSpec* spec, mjsMaterial* material,
                         const pxr::UsdShadeShader& shader) {
  auto diffuse = ReadShaderInput<pxr::GfVec3f>(
      spec, shader.GetInput(kTokens->diffuseColor));
  AssignShaderInput(spec, material, &material->rgba, mjTEXROLE_RGB, diffuse);

  auto metallic =
      ReadShaderInput<float>(spec, shader.GetInput(kTokens->metallic));
  AssignShaderInput(spec, material, &material->metallic, mjTEXROLE_METALLIC,
                    metallic);

  auto roughness =
      ReadShaderInput<float>(spec, shader.GetInput(kTokens->roughness));
  AssignShaderInput(spec, material, &material->roughness, mjTEXROLE_ROUGHNESS,
                    roughness);
}

mjsMaterial* ParseMaterial(mjSpec* spec,
                           const pxr::UsdShadeMaterial& material) {
  pxr::UsdShadeShader surface_shader = material.ComputeSurfaceSource();
  if (!surface_shader.GetPrim().IsValid()) {
    mju_warning("Material %s has no surface output.",
                material.GetPath().GetAsString().c_str());
    return nullptr;
  }

  pxr::TfToken surface_shader_id = GetShaderId(surface_shader);
  if (surface_shader_id != kTokens->UsdPreviewSurface) {
    mju_warning("Mujoco only supports UsdPreviewSurface as surface output.");
    return nullptr;
  }

  mjsMaterial* mj_mat = mjs_addMaterial(spec, nullptr);
  mjs_setName(mj_mat->element, material.GetPath().GetAsString().c_str());
  ParsePreviewSurface(spec, mj_mat, surface_shader);

  return mj_mat;
}
}  // namespace usd
}  // namespace mujoco
