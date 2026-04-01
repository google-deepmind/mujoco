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

#include "experimental/filament/filament/model_objects.h"

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/Skybox.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/builtins.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/texture.h"


namespace mujoco {

ModelObjects::ModelObjects(const mjModel* model, filament::Engine* engine)
    : model_(model), engine_(engine) {
  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;
  shapes_[kLine] = CreateLine(engine_);
  shapes_[kBox] = CreateBox(engine_, nquad);
  shapes_[kLineBox] = CreateLineBox(engine_);
  shapes_[kCone] = CreateCone(engine_, nstack, nslice);
  shapes_[kDisk] = CreateDisk(engine_, nslice);
  shapes_[kDome] = CreateDome(engine_, nstack / 2, nslice);
  shapes_[kTube] = CreateTube(engine_, nstack, nslice);
  shapes_[kPlane] = CreatePlane(engine_, nquad);
  shapes_[kSphere] = CreateSphere(engine_, nstack, nslice);
  shapes_[kTriangle] = CreateTriangle(engine_);

  for (int i = 0; i < model_->ntex; ++i) {
    UploadTexture(model_, i);
  }
  for (int i = 0; i < model_->nmesh; ++i) {
    UploadMesh(model_, i);
  }
  for (int i = 0; i < model_->nhfield; ++i) {
    UploadHeightField(model_, i);
  }

  specular_multiplier_ = ReadElement(
      model_, "filament.phong.specular_multiplier", specular_multiplier_);
  shininess_multiplier_ = ReadElement(
      model_, "filament.phong.shininess_multiplier", shininess_multiplier_);
  emissive_multiplier_ = ReadElement(
      model_, "filament.phong.emissive_multiplier", emissive_multiplier_);
}

ModelObjects::~ModelObjects() {
  for (auto& iter : skyboxes_) {
    engine_->destroy(iter);
  }
  for (auto& iter : indirect_lights_) {
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
  textures_.clear();
}

void ModelObjects::UploadMesh(const mjModel* model, int id) {
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
  buffers.vertex_buffer = CreateVertexBuffer(
      engine_, model, id, MeshType::kNormal, &buffers.bounds.emplace());
  buffers.index_buffer =
      CreateIndexBuffer(engine_, model, id, MeshType::kNormal);

  if (model->mesh_graphadr[id] >= 0) {
    FilamentBuffers& hull_buffers = convex_hulls_[id];
    hull_buffers.vertex_buffer =
        CreateVertexBuffer(engine_, model, id, MeshType::kConvexHull,
                           &hull_buffers.bounds.emplace());
    hull_buffers.index_buffer =
        CreateIndexBuffer(engine_, model, id, MeshType::kConvexHull);
  }
}

void ModelObjects::UploadTexture(const mjModel* model, int id) {
  if (model != model_) {
    mju_error("Model mismatch.");
  }
  if (id < 0 || id >= model->ntex) {
    mju_error("Invalid texture index: %d", id);
  }

  TextureConfig config;
  DefaultTextureConfig(&config);
  config.width = model->tex_width[id];
  config.height = model->tex_height[id];
  config.target = (mjtTexture)model->tex_type[id];
  config.color_space = (mjtColorSpace)model->tex_colorspace[id];
  switch (model->tex_nchannel[id]) {
    case 1:
      config.format = mjPIXEL_FORMAT_R8;
      break;
    case 3:
      config.format = mjPIXEL_FORMAT_RGB8;
      break;
    case 4:
      config.format = mjPIXEL_FORMAT_RGBA8;
      break;
    default:
      mju_error("Unsupported texture format: %d", model->tex_nchannel[id]);
      break;
  }
  if (config.height == 1 && model->tex_nchannel[id] == 1) {
    config.format = mjPIXEL_FORMAT_KTX;
  }


  TextureData payload;
  DefaultTextureData(&payload);
  payload.bytes = model->tex_data + model->tex_adr[id];
  payload.nbytes =
      model->tex_width[id] * model->tex_height[id] * model->tex_nchannel[id];
  // We assume that the model has the same lifetime as the engine.
  payload.user_data = nullptr;
  payload.release_callback = nullptr;

  auto texture = std::make_unique<Texture>(engine_, config);
  texture->Upload(payload);
  textures_[id] = std::move(texture);
}

void ModelObjects::UploadHeightField(const mjModel* model, int id) {
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
  buffers.vertex_buffer = CreateVertexBuffer(
      engine_, model, id, MeshType::kHeightField, &buffers.bounds.emplace());
  buffers.index_buffer =
      CreateIndexBuffer(engine_, model, id, MeshType::kHeightField);
}

const FilamentBuffers* ModelObjects::GetMeshBuffer(int data_id) const {
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

const FilamentBuffers* ModelObjects::GetHeightFieldBuffer(
    int hfield_id) const {
  auto it = height_fields_.find(hfield_id);
  return it != height_fields_.end() ? &it->second : nullptr;
}

const FilamentBuffers* ModelObjects::GetShapeBuffer(ShapeType shape) const {
  if (shape < 0 || shape >= kNumShapes) {
    mju_error("Invalid shape type: %d", shape);
  }
  return &shapes_[shape];
}

const Texture* ModelObjects::GetTexture(int tex_id) const {
  auto it = textures_.find(tex_id);
  return it != textures_.end() ? it->second.get() : nullptr;
}

const Texture* ModelObjects::GetTexture(int mat_id, int role) const {
  if (mat_id < 0 || mat_id >= model_->nmat || role < 0 || role >= mjNTEXROLE) {
    return nullptr;
  }
  const int tex_id = model_->mat_texid[mat_id * mjNTEXROLE + role];
  return GetTexture(tex_id);
}

filament::IndirectLight* ModelObjects::CreateIndirectLight(int tex_id,
                                                           float intensity) {
  filament::Texture* texture = nullptr;
  const Texture::SphericalHarmonics* spherical_harmonics = nullptr;
  auto texture_iter = textures_.find(tex_id);
  if (texture_iter != textures_.end()) {
    texture = texture_iter->second->GetFilamentTexture();
    spherical_harmonics = texture_iter->second->GetSphericalHarmonics();
  }

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

filament::Skybox* ModelObjects::CreateSkybox() {
  filament::Texture* skybox_texture = nullptr;
  for (auto& iter : textures_) {
    const int texture_type = model_->tex_type[iter.first];
    if (texture_type == mjTEXTURE_SKYBOX) {
      skybox_texture = iter.second->GetFilamentTexture();
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
