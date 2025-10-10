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

#include "experimental/filament/filament/geom_util.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <filament/Engine.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/vertex_util.h"

namespace mujoco {

static std::span<const float> GetPositions(const mjModel* model,
                                           const mjvScene* scene,
                                           const mjvGeom& geom) {
  if (geom.type == mjGEOM_FLEX) {
    const int num = 9 * scene->flexfaceused[geom.objid];
    const int addr = scene->flexfaceadr[geom.objid];
    const float* ptr = scene->flexface + (9 * addr);
    return {ptr, static_cast<size_t>(num)};
  } else {
    const int num = 3 * scene->skinvertnum[geom.objid];
    const int addr = scene->skinvertadr[geom.objid];
    const float* ptr = scene->skinvert + (3 * addr);
    return {ptr, static_cast<size_t>(num)};
  }
}

static std::span<const float> GetNormals(const mjModel* model,
                                         const mjvScene* scene,
                                         const mjvGeom& geom) {
  if (geom.type == mjGEOM_FLEX) {
    const int num = 9 * scene->flexfaceused[geom.objid];
    const int addr = scene->flexfaceadr[geom.objid];
    const float* ptr = scene->flexnormal + (9 * addr);
    return {ptr, static_cast<size_t>(num)};
  } else {
    const int num = 3 * scene->skinvertnum[geom.objid];
    const int addr = scene->skinvertadr[geom.objid];
    const float* ptr = scene->skinnormal + (3 * addr);
    return {ptr, static_cast<size_t>(num)};
  }
}

static std::span<const float> GetUvs(const mjModel* model,
                                     const mjvScene* scene,
                                     const mjvGeom& geom) {
  if (geom.type == mjGEOM_FLEX) {
    if (geom.texcoord && geom.matid >= 0) {
      const int num = 6 * scene->flexfaceused[geom.objid];
      const int addr = scene->flexfaceadr[geom.objid];
      const float* ptr = scene->flextexcoord + (6 * addr);
      return {ptr, static_cast<size_t>(num)};
    } else {
      const float* ptr = nullptr;
      return {ptr, 0};
    }
  } else {
    if (model->skin_texcoordadr[geom.objid] >= 0) {
      const int num = 3 * scene->skinvertnum[geom.objid];
      const int addr = model->skin_texcoordadr[geom.objid];
      const float* ptr = model->skin_texcoord + (2 * addr);
      return {ptr, static_cast<size_t>(num)};
    } else {
      const float* ptr = nullptr;
      return {ptr, 0};
    }
  }
}

static std::span<const int> GetIndices(const mjModel* model,
                                       const mjvScene* scene,
                                       const mjvGeom& geom) {
  if (geom.type == mjGEOM_FLEX) {
    const int* ptr = nullptr;
    return {ptr, 0};
  } else {
    const int num = 3 * model->skin_facenum[geom.objid];
    const int* ptr = model->skin_face + 3 * model->skin_faceadr[geom.objid];
    return {ptr, static_cast<size_t>(num)};
  }
}

template <typename T>
static void FillVertices(std::byte* buffer, std::size_t len,
                         std::span<const float> positions,
                         std::span<const float> normals,
                         std::span<const float> uvs) {
  const int num_vertices = len / sizeof(T);
  T* ptr = reinterpret_cast<T*>(buffer);
  for (int i = 0; i < num_vertices; ++i) {
    ptr->position = ReadFloat3(positions.data(), i);
    ptr->orientation = CalculateOrientation(ReadFloat3(normals.data(), i));
    if constexpr (T::kHasUv) {
      ptr->uv.x = uvs[i * 2];
      ptr->uv.y = uvs[i * 2 + 1];
    }
    ++ptr;
  }
}

static filament::VertexBuffer* BuildVertexBuffer(
    filament::Engine* engine, std::span<const float> positions,
    std::span<const float> normals, std::span<const float> uvs) {
  const int num_vertices = positions.size() / 3;
  if (uvs.data() != nullptr) {
    using VertexType = VertexWithUv;
    auto fill = [&](std::byte* buffer, std::size_t len) {
      FillVertices<VertexType>(buffer, len, positions, normals, uvs);
    };
    return CreateVertexBuffer<VertexType>(engine, num_vertices, fill);
  } else {
    using VertexType = VertexNoUv;
    auto fill = [&](std::byte* buffer, std::size_t len) {
      FillVertices<VertexType>(buffer, len, positions, normals, uvs);
    };
    return CreateVertexBuffer<VertexType>(engine, num_vertices, fill);
  }
}

static filament::IndexBuffer* BuildIndexBuffer(filament::Engine* engine,
                                               std::span<const int> indices,
                                               int num_indices) {
  if (indices.data() == nullptr) {
    auto fill_indices = FillSequence<uint32_t>;
    return CreateIndexBuffer<uint32_t>(engine, num_indices, fill_indices);
  } else {
    auto fill_indices = [&](std::byte* buffer, std::size_t len) {
      std::memcpy(buffer, indices.data(), len);
    };
    return CreateIndexBuffer<uint32_t>(engine, indices.size(), fill_indices);
  }
}

FilamentBuffers CreateGeomBuffers(filament::Engine* engine,
                                  const mjModel* model, const mjvScene* scene,
                                  const mjvGeom& geom) {
  auto positions = GetPositions(model, scene, geom);
  auto normals = GetNormals(model, scene, geom);
  auto uvs = GetUvs(model, scene, geom);
  auto indices = GetIndices(model, scene, geom);

  int num_indices = indices.size();
  if (num_indices == 0 && geom.type == mjGEOM_FLEX) {
    num_indices = 3 * scene->flexfaceused[geom.objid];
  }

  FilamentBuffers buffers;
  buffers.vertex_buffer = BuildVertexBuffer(engine, positions, normals, uvs);
  buffers.index_buffer = BuildIndexBuffer(engine, indices, num_indices);
  return buffers;
}

}  // namespace mujoco
