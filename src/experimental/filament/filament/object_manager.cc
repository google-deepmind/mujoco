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

#include "experimental/filament/filament/object_manager.h"

#include <cstdint>
#include <string>
#include <string_view>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/Skybox.h>
#include <math/mat3.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/texture_util.h"
#include "user/user_resource.h"

namespace mujoco {
namespace {

// Loads binary data from a file using mjrFilamentConfig callbacks.
struct Asset {
  explicit Asset(std::string_view filename) {
    std::string path = "filament:" + std::string(filename);

    resource = mju_openResource("", path.c_str(), nullptr, nullptr, 0);
    size = mju_readResource(resource, const_cast<const void**>(&payload));
  }

  ~Asset() {
    if (resource) {
      mju_closeResource(resource);
    }
  }

  Asset(const Asset&) = delete;
  Asset& operator=(const Asset&) = delete;

  int size = 0;
  void* payload = nullptr;
  mjResource* resource = nullptr;
};

}  // namespace

ObjectManager::ObjectManager(filament::Engine* engine)
    : engine_(engine) {
  auto LoadMaterial = [this](std::string_view filename) {
    Asset asset(filename);
    filament::Material::Builder material_builder;
    material_builder.package(asset.payload, asset.size);
    return material_builder.build(*this->engine_);
  };

  materials_[kPbr] = LoadMaterial("pbr.filamat");
  materials_[kPbrPacked] = LoadMaterial("pbr_packed.filamat");
  materials_[kPhong2d] = LoadMaterial("phong_2d.filamat");
  materials_[kPhong2dFade] = LoadMaterial("phong_2d_fade.filamat");
  materials_[kPhong2dReflect] = LoadMaterial("phong_2d_reflect.filamat");
  materials_[kPhong2dUv] = LoadMaterial("phong_2d_uv.filamat");
  materials_[kPhong2dUvFade] = LoadMaterial("phong_2d_uv_fade.filamat");
  materials_[kPhong2dUvReflect] = LoadMaterial("phong_2d_uv_reflect.filamat");
  materials_[kPhongColor] = LoadMaterial("phong_color.filamat");
  materials_[kPhongColorFade] = LoadMaterial("phong_color_fade.filamat");
  materials_[kPhongColorReflect] = LoadMaterial("phong_color_reflect.filamat");
  materials_[kPhongCube] = LoadMaterial("phong_cube.filamat");
  materials_[kPhongCubeFade] = LoadMaterial("phong_cube_fade.filamat");
  materials_[kPhongCubeReflect] = LoadMaterial("phong_cube_reflect.filamat");
  materials_[kUnlitSegmentation] = LoadMaterial("unlit_segmentation.filamat");
  materials_[kUnlitLine] = LoadMaterial("unlit_line.filamat");
  materials_[kUnlitDepth] = LoadMaterial("unlit_depth.filamat");
  materials_[kUnlitUi] = LoadMaterial("unlit_ui.filamat");

  static uint8_t black_rgb[3] = {0, 0, 0};
  fallback_black_ = Create2dTexture(engine_, 1, 1, 3, black_rgb, false);
  static uint8_t white_rgb[3] = {255, 255, 255};
  fallback_white_ = Create2dTexture(engine_, 1, 1, 3, white_rgb, false);
  static uint8_t normal_data[3] = {128, 128, 255};
  fallback_normal_ = Create2dTexture(engine_, 1, 1, 3, normal_data, false);
  static uint8_t orm_data[3] = {0, 255, 0};
  fallback_orm_ = Create2dTexture(engine_, 1, 1, 3, orm_data, false);

  fallback_textures_[mjTEXROLE_USER] = fallback_black_;
  fallback_textures_[mjTEXROLE_RGB] = fallback_white_;
  fallback_textures_[mjTEXROLE_OCCLUSION] = fallback_white_;
  fallback_textures_[mjTEXROLE_ROUGHNESS] = fallback_white_;
  fallback_textures_[mjTEXROLE_METALLIC] = fallback_black_;
  fallback_textures_[mjTEXROLE_NORMAL] = fallback_normal_;
  fallback_textures_[mjTEXROLE_EMISSIVE] = fallback_black_;
  fallback_textures_[mjTEXROLE_ORM] = fallback_orm_;

  LoadFallbackIndirectLight("ibl.ktx", 1.0f);
}

ObjectManager::~ObjectManager() {
  if (fallback_indirect_light_) {
    engine_->destroy(fallback_indirect_light_);
  }
  if (fallback_indirect_light_texture_) {
    engine_->destroy(fallback_indirect_light_texture_);
  }
  for (auto& iter : materials_) {
    engine_->destroy(iter);
  }
  // fallback_textures_ maps to these textures.
  engine_->destroy(fallback_white_);
  engine_->destroy(fallback_black_);
  engine_->destroy(fallback_normal_);
  engine_->destroy(fallback_orm_);
}

filament::Material* ObjectManager::GetMaterial(MaterialType type) const {
  if (type < 0 || type >= kNumMaterials) {
    mju_error("Invalid material type: %d", type);
  }
  return materials_[type];
}

const filament::Texture* ObjectManager::GetFallbackTexture(
    mjtTextureRole role) const {
  if (role < 0 || role >= mjNTEXROLE) {
    mju_error("Invalid texture role: %d", role);
  }
  return fallback_textures_[role];
}

filament::IndirectLight* ObjectManager::GetFallbackIndirectLight() {
  return fallback_indirect_light_;
}

void ObjectManager::LoadFallbackIndirectLight(
    std::string_view filename, float intensity) {
  if (fallback_indirect_light_texture_ != nullptr) {
    engine_->destroy(fallback_indirect_light_texture_);
    fallback_indirect_light_texture_ = nullptr;
  }
  if (fallback_indirect_light_ != nullptr) {
    engine_->destroy(fallback_indirect_light_);
    fallback_indirect_light_ = nullptr;
  }

  Asset asset(filename);
  if (asset.size == 0) {
    return;
  }

  filament::math::float3 spherical_harmonics[9];
  fallback_indirect_light_texture_ =
      CreateKtxTexture(engine_, reinterpret_cast<const uint8_t*>(asset.payload),
                       asset.size, spherical_harmonics);
  if (fallback_indirect_light_texture_ == nullptr) {
    return;
  }

  // Build the indirect light.
  filament::IndirectLight::Builder builder;
  builder.reflections(fallback_indirect_light_texture_);
  builder.irradiance(3, spherical_harmonics);
  builder.intensity(intensity);
  // Rotate the light to match mujoco's Z-up convention.
  builder.rotation(filament::math::mat3f::rotation(
      filament::math::f::PI / 2, filament::math::float3{1, 0, 0}));
  fallback_indirect_light_ = builder.build(*engine_);
}
}  // namespace mujoco
