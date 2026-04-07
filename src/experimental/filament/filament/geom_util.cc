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
#include <cstring>
#include <memory>
#include <span>

#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/VertexBuffer.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/mesh.h"

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

MeshPtr CreateGeomBuffers(filament::Engine* engine, const mjModel* model,
                          const mjvScene* scene, const mjvGeom& geom) {
  auto positions = GetPositions(model, scene, geom);
  auto normals = GetNormals(model, scene, geom);
  auto uvs = GetUvs(model, scene, geom);
  auto indices = GetIndices(model, scene, geom);

  int num_indices = indices.size();
  if (num_indices == 0 && geom.type == mjGEOM_FLEX) {
    num_indices = 3 * scene->flexfaceused[geom.objid];
  }

  MeshData data;
  DefaultMeshData(&data);

  data.nattributes = uvs.data() ? 3 : 2;
  data.attributes[0].usage = mjVERTEX_ATTRIBUTE_POSITION;
  data.attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
  data.attributes[0].bytes = positions.data();
  data.attributes[1].usage = mjVERTEX_ATTRIBUTE_NORMAL;
  data.attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
  data.attributes[1].bytes = normals.data();
  data.attributes[2].usage = mjVERTEX_ATTRIBUTE_UV;
  data.attributes[2].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
  data.attributes[2].bytes = uvs.data();
  data.nvertices = positions.size() / 3;
  data.nindices = num_indices;
  data.indices = indices.data();
  data.index_type = mjINDEX_TYPE_UINT;
  data.primitive_type = mjPRIM_TYPE_TRIANGLES;
  data.compute_bounds = true;
  data.release_callback = nullptr;
  data.user_data = nullptr;
  return std::make_unique<Mesh>(engine, data);
}

}  // namespace mujoco
