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

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/Skybox.h>
#include <math/mat3.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/builtins.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/texture_util.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {
namespace {

// Loads binary data from a file using mjrFilamentConfig callbacks.
struct Asset {
  Asset(const char* filename, const mjrFilamentConfig* config) {
    const int error = config->load_asset(filename, config->load_asset_user_data,
                                         &payload, &size);
    if (error) {
      mju_error("Failed to load file: %s (error: %d)", filename, error);
    }
  }

  ~Asset() {
    if (payload) {
      free(payload);
      payload = nullptr;
    }
  }

  uint64_t size = 0;
  unsigned char* payload = nullptr;

  Asset(const Asset&) = delete;
  Asset& operator=(const Asset&) = delete;
};

}  // namespace

ObjectManager::ObjectManager(const mjModel* model, filament::Engine* engine,
                             const mjrFilamentConfig* config)
    : model_(model), engine_(engine), config_(config) {
  shapes_[kLine] = CreateLine(engine_, model_);
  shapes_[kBox] = CreateBox(engine_, model_);
  shapes_[kLineBox] = CreateLineBox(engine_, model_);
  shapes_[kCone] = CreateCone(engine_, model_);
  shapes_[kDisk] = CreateDisk(engine_, model_);
  shapes_[kDome] = CreateDome(engine_, model_);
  shapes_[kTube] = CreateTube(engine_, model_);
  shapes_[kPlane] = CreatePlane(engine_, model_);
  shapes_[kSphere] = CreateSphere(engine_, model_);

  auto LoadMaterial = [this](const char* filename) {
    Asset asset(filename, config_);
    filament::Material::Builder material_builder;
    material_builder.package(asset.payload, asset.size);
    return material_builder.build(*this->engine_);
  };

  materials_[kPbr] = LoadMaterial("pbr.filamat");
  materials_[kPbrPacked] = LoadMaterial("pbr_packed.filamat");
  materials_[kPhong2d] = LoadMaterial("phong_2d.filamat");
  materials_[kPhong2dFade] = LoadMaterial("phong_2d_fade.filamat");
  materials_[kPhong2dUv] = LoadMaterial("phong_2d_uv.filamat");
  materials_[kPhong2dUvFade] = LoadMaterial("phong_2d_uv_fade.filamat");
  materials_[kPhongColor] = LoadMaterial("phong_color.filamat");
  materials_[kPhongColorFade] = LoadMaterial("phong_color_fade.filamat");
  materials_[kPhongCube] = LoadMaterial("phong_cube.filamat");
  materials_[kPhongCubeFade] = LoadMaterial("phong_cube_fade.filamat");
  materials_[kUnlitSegmentation] = LoadMaterial("unlit_segmentation.filamat");
  materials_[kUnlitLine] = LoadMaterial("unlit_line.filamat");
  materials_[kUnlitDepth] = LoadMaterial("unlit_depth.filamat");
  materials_[kUnlitUi] = LoadMaterial("unlit_ui.filamat");

  for (int i = 0; i < model_->ntex; ++i) {
    UploadTexture(model_, i);
  }
  for (int i = 0; i < model_->nmesh; ++i) {
    UploadMesh(model_, i);
  }
  for (int i = 0; i < model_->nhfield; ++i) {
    UploadHeightField(model_, i);
  }

  static uint8_t black_rgb[3] = {0, 0, 0};
  fallback_black_ = Create2dTexture(engine_, 1, 1, 3, black_rgb, false);
  static uint8_t white_rgb[3] = {255, 255, 255};
  fallback_white_ = Create2dTexture(engine_, 1, 1, 3, white_rgb, false);
  static uint8_t normal_data[3] = {128, 128, 255};
  fallback_normal_ = Create2dTexture(engine_, 1, 1, 3, normal_data, false);
  static uint8_t orm_data[3] = {0, 255, 0};
  fallback_orm_ = Create2dTexture(engine_, 1, 1, 3, orm_data, false);

  fallback_textures_[mjTEXROLE_USER] = fallback_black_;
  fallback_textures_[mjTEXROLE_RGB] = fallback_black_;
  fallback_textures_[mjTEXROLE_OCCLUSION] = fallback_black_;
  fallback_textures_[mjTEXROLE_ROUGHNESS] = fallback_white_;
  fallback_textures_[mjTEXROLE_METALLIC] = fallback_black_;
  fallback_textures_[mjTEXROLE_NORMAL] = fallback_normal_;
  fallback_textures_[mjTEXROLE_EMISSIVE] = fallback_black_;
  fallback_textures_[mjTEXROLE_ORM] = fallback_orm_;

  fallback_indirect_light_ = LoadFallbackIndirectLight("ibl.ktx", 1.0f);
}

ObjectManager::~ObjectManager() {
  for (auto& iter : skyboxes_) {
    engine_->destroy(iter);
  }
  for (auto& iter : indirect_lights_) {
    engine_->destroy(iter);
  }
  for (auto& iter : materials_) {
    engine_->destroy(iter);
  }
  for (auto& iter : meshes_) {
    engine_->destroy(iter.second.vertex_buffer);
    engine_->destroy(iter.second.index_buffer);
  }
  for (auto& iter : shapes_) {
    engine_->destroy(iter.vertex_buffer);
    engine_->destroy(iter.index_buffer);
  }
  for (auto& iter : textures_) {
    engine_->destroy(iter.second);
  }
  // fallback_textures_ maps to these textures.
  engine_->destroy(fallback_white_);
  engine_->destroy(fallback_black_);
  engine_->destroy(fallback_normal_);
  engine_->destroy(fallback_orm_);
  for (auto& iter : fonts_) {
    engine_->destroy(iter.second);
  }
}

void ObjectManager::UploadMesh(const mjModel* model, int id) {
  if (model != model_) {
    mju_error("Model mismatch.");
  }
  if (id < 0 || id >=  model->nmesh) {
    mju_error("Invalid mesh index %d", id);
  }

  if (auto iter = meshes_.find(id); iter != meshes_.end()) {
    engine_->destroy(iter->second.vertex_buffer);
    engine_->destroy(iter->second.index_buffer);
  }
  if (auto iter = convex_hulls_.find(id); iter != convex_hulls_.end()) {
    engine_->destroy(iter->second.vertex_buffer);
    engine_->destroy(iter->second.index_buffer);
  }

  FilamentBuffers& buffers = meshes_[id];
  buffers.vertex_buffer =
      CreateVertexBuffer(engine_, model, id, MeshType::kNormal);
  buffers.index_buffer =
      CreateIndexBuffer(engine_, model, id, MeshType::kNormal);

  if (model->mesh_graphadr[id] >= 0) {
    FilamentBuffers& hull_buffers = convex_hulls_[id];
    hull_buffers.vertex_buffer =
        CreateVertexBuffer(engine_, model, id, MeshType::kConvexHull);
    hull_buffers.index_buffer =
        CreateIndexBuffer(engine_, model, id, MeshType::kConvexHull);
  }
}

void ObjectManager::UploadTexture(const mjModel* model, int id) {
  if (model != model_) {
    mju_error("Model mismatch.");
  }
  if (id < 0 || id >= model->ntex) {
    mju_error("Invalid texture index: %d", id);
  }

  if (auto iter = textures_.find(id); iter != textures_.end()) {
    engine_->destroy(iter->second);
  }

  const int texture_type = model->tex_type[id];
  if (model->tex_height[id] == 1) {
    const mjtByte* bytes = model->tex_data + model->tex_adr[id];
    const int num_bytes = model->tex_width[id];
    textures_[id] =
        CreateKtxTexture(engine_, bytes, num_bytes, spherical_harmonics_[id]);
  } else if (texture_type == mjTEXTURE_2D) {
    textures_[id] = CreateTexture(engine_, model, id, TextureType::kNormal2d);
  } else if (texture_type == mjTEXTURE_CUBE) {
    textures_[id] = CreateTexture(engine_, model, id, TextureType::kCube);
  } else if (texture_type == mjTEXTURE_SKYBOX) {
    textures_[id] = CreateTexture(engine_, model, id, TextureType::kCube);
  } else {
    mju_error("Unsupported: Texture type: %d", texture_type);
  }
}

void ObjectManager::UploadHeightField(const mjModel* model, int id) {
  if (model != model_) {
    mju_error("Model mismatch.");
  }
  if (id < 0 || id >= model->nhfield) {
    mju_error("Invalid height field index %d", id);
  }

  if (auto iter = height_fields_.find(id); iter != height_fields_.end()) {
    engine_->destroy(iter->second.vertex_buffer);
    engine_->destroy(iter->second.index_buffer);
  }

  FilamentBuffers& buffers = height_fields_[id];
  buffers.vertex_buffer =
      CreateVertexBuffer(engine_, model, id, MeshType::kHeightField);
  buffers.index_buffer =
      CreateIndexBuffer(engine_, model, id, MeshType::kHeightField);
}

void ObjectManager::UploadFont(const uint8_t* pixels, int width, int height,
                               int id) {
  if (auto iter = fonts_.find(id); iter != fonts_.end()) {
    engine_->destroy(iter->second);
  }
  fonts_[id] = Create2dTexture(engine_, width, height, 4, pixels, false);
}

filament::Material* ObjectManager::GetMaterial(MaterialType type) const {
  if (type < 0 || type >= kNumMaterials) {
    mju_error("Invalid material type: %d", type);
  }
  return materials_[type];
}

const FilamentBuffers* ObjectManager::GetMeshBuffer(int data_id) const {
  // As defined by mjv_updateScene:
  //   original mesh: mesh_id * 2
  //   convex hull: (mesh_id * 2) + 1
  const int mesh_id = data_id / 2;
  if (data_id % 2 == 0) {
    auto it = meshes_.find(mesh_id);
    return it != meshes_.end() ? &it->second : nullptr;
  } else {
    auto it = convex_hulls_.find(mesh_id);
    return it != convex_hulls_.end() ? &it->second : nullptr;
  }
}

const FilamentBuffers* ObjectManager::GetHeightFieldBuffer(
    int hfield_id) const {
  auto it = height_fields_.find(hfield_id);
  return it != height_fields_.end() ? &it->second : nullptr;
}

const FilamentBuffers* ObjectManager::GetShapeBuffer(ShapeType shape) const {
  if (shape < 0 || shape >= kNumShapes) {
    mju_error("Invalid shape type: %d", shape);
  }
  return &shapes_[shape];
}

const filament::Texture* ObjectManager::GetFont(int font_id) const {
  auto it = fonts_.find(font_id);
  return it != fonts_.end() ? it->second : nullptr;
}

const filament::Texture* ObjectManager::GetTexture(int tex_id) const {
  auto it = textures_.find(tex_id);
  return it != textures_.end() ? it->second : nullptr;
}

const filament::Texture* ObjectManager::GetTexture(int mat_id, int role) const {
  if (mat_id < 0 || mat_id >= model_->nmat || role < 0 || role >= mjNTEXROLE) {
    return nullptr;
  }
  const int tex_id = model_->mat_texid[mat_id * mjNTEXROLE + role];
  return GetTexture(tex_id);
}

const filament::Texture* ObjectManager::GetTextureWithFallback(int mat_id,
                                                               int role) const {
  if (auto texture = GetTexture(mat_id, role)) {
    return texture;
  }
  return GetFallbackTexture(role);
}

const filament::Texture* ObjectManager::GetFallbackTexture(int role) const {
  auto iter = fallback_textures_.find(role);
  if (iter != fallback_textures_.end()) {
    return iter->second;
  }
  return nullptr;
}

filament::IndirectLight* ObjectManager::GetFallbackIndirectLight() {
  return fallback_indirect_light_;
}

filament::IndirectLight* ObjectManager::CreateIndirectLight(int tex_id,
                                                            float intensity) {
  filament::Texture* texture = nullptr;
  auto texture_iter = textures_.find(tex_id);
  if (texture_iter != textures_.end()) {
    texture = texture_iter->second;
  }

  if (texture == nullptr) {
    return nullptr;
  }

  SphericalHarmonics* spherical_harmonics = nullptr;
  auto sh_iter = spherical_harmonics_.find(tex_id);
  if (sh_iter != spherical_harmonics_.end()) {
    spherical_harmonics = &sh_iter->second;
  }

  return CreateIndirectLight(texture, spherical_harmonics, intensity);
}

filament::IndirectLight* ObjectManager::LoadFallbackIndirectLight(
    std::string_view filename, float intensity) {
  Asset asset(std::string(filename).c_str(), config_);
  if (asset.size == 0) {
    return nullptr;
  }

  filament::math::float3 spherical_harmonics[9];
  filament::Texture* tex =
      CreateKtxTexture(engine_, asset.payload, asset.size, spherical_harmonics);
  // TODO: Revisit this to make it this work in WebGL
  #ifndef __EMSCRIPTEN__
  tex->generateMipmaps(*engine_);
  #endif

  return CreateIndirectLight(tex, &spherical_harmonics, intensity);
}

filament::IndirectLight* ObjectManager::CreateIndirectLight(
    filament::Texture* texture, SphericalHarmonics* spherical_harmonics,
    float intensity) {
  filament::IndirectLight::Builder builder;
  builder.reflections(texture);
  if (spherical_harmonics != nullptr) {
    builder.irradiance(3, *spherical_harmonics);
  }
  builder.intensity(intensity);
  // Rotate the light to match mujoco's Z-up convention.
  builder.rotation(filament::math::mat3f::rotation(
      filament::math::f::PI / 2, filament::math::float3{1, 0, 0}));
  filament::IndirectLight* indirect_light = builder.build(*engine_);
  indirect_lights_.push_back(indirect_light);
  return indirect_light;
}

filament::Skybox* ObjectManager::CreateSkybox() {
  filament::Texture* skybox_texture = nullptr;
  for (auto& iter : textures_) {
    const int texture_type = model_->tex_type[iter.first];
    if (texture_type == mjTEXTURE_SKYBOX) {
      skybox_texture = iter.second;
      break;
    }
  }

  if (skybox_texture == nullptr) {
    return nullptr;
  }

  filament::Skybox::Builder builder;
  builder.environment(skybox_texture);
  filament::Skybox* skybox = builder.build(*engine_);
  skyboxes_.push_back(skybox);
  return skybox;
}

}  // namespace mujoco
