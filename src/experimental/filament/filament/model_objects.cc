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
#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/Skybox.h>
#include <math/TVecHelpers.h>
#include <math/mat3.h>
#include <math/scalar.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/builtins.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/texture.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat3f;

enum class MeshType {
  kNormal,
  kConvexHull,
  kHeightField,
};

struct MeshBuilder {
  MeshBuilder(int nvertices) : nvertices(nvertices) {
    positions.reserve(nvertices);
    orientations.reserve(nvertices);
    uvs.reserve(nvertices);
  }

  void Append(const float3& position, const float4& orientation,
              const float2& uv) {
    positions.push_back(position);
    orientations.push_back(orientation);
    uvs.push_back(uv);
    bounds_min = min(bounds_min, position);
    bounds_max = max(bounds_max, position);
  }

  int nvertices = 0;
  float3 bounds_min = {FLT_MAX, FLT_MAX, FLT_MAX};
  float3 bounds_max = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
  std::vector<float3> positions;
  std::vector<float4> orientations;
  std::vector<float2> uvs;
};

static bool UseFaceNormal(const float3& face_normal,
                          const float3& mesh_normal) {
  // clang-format off
  return (face_normal[0] * mesh_normal[0] +
          face_normal[1] * mesh_normal[1] +
          face_normal[2] * mesh_normal[2]) < 0.8f;
  // clang-format on
}

static void FillConvexHullBuffer(MeshBuilder& builder, const mjModel* model,
                                 int meshid) {
  const int numvert = model->mesh_graph[model->mesh_graphadr[meshid]];
  const int numface = model->mesh_graph[model->mesh_graphadr[meshid] + 1];
  if (builder.nvertices != numface * 3) {
    mju_error("Invalid vertex count (%d vs %d).", builder.nvertices, numface * 3);
    return;
  }

  const int dataadr = model->mesh_graphadr[meshid] + 2;
  const int vertadr = model->mesh_vertadr[meshid];
  const float* vertices = model->mesh_vert + (3 * vertadr);
  const int texcoordadr = model->mesh_texcoordadr[meshid];
  const float* texcoords = texcoordadr >= 0 ? model->mesh_texcoord + (2 * texcoordadr) : nullptr;

  for (int face = 0; face < numface; ++face) {
    const int j = dataadr + (3 * numvert) + (3 * numface) + (3 * face);
    const float3 p1 = ReadFloat3(vertices, model->mesh_graph[j + 0]);
    const float3 p2 = ReadFloat3(vertices, model->mesh_graph[j + 1]);
    const float3 p3 = ReadFloat3(vertices, model->mesh_graph[j + 2]);
    const float4 orientation = CalculateOrientation(p1, p2, p3);
    const float2 uv1 = texcoords ? ReadFloat2(texcoords, model->mesh_graph[j + 0]) : float2(0, 0);
    const float2 uv2 = texcoords ? ReadFloat2(texcoords, model->mesh_graph[j + 1]) : float2(0, 0);
    const float2 uv3 = texcoords ? ReadFloat2(texcoords, model->mesh_graph[j + 2]) : float2(0, 0);
    builder.Append(p1, orientation, uv1);
    builder.Append(p2, orientation, uv2);
    builder.Append(p3, orientation, uv3);
  }
}

static void FillMeshBuffer(MeshBuilder& builder, const mjModel* model, int meshid) {
  const int faceadr = model->mesh_faceadr[meshid];
  const int facenum = model->mesh_facenum[meshid];
  if (builder.nvertices != facenum * 3) {
    mju_error("Invalid vertex count (%d vs %d).", builder.nvertices, facenum * 3);
    return;
  }

  const int vertadr = model->mesh_vertadr[meshid];
  const float* vertices = model->mesh_vert + (3 * vertadr);
  const int normaladr = model->mesh_normaladr[meshid];
  const float* normals = model->mesh_normal + 3 * normaladr;
  const int texcoordadr = model->mesh_texcoordadr[meshid];
  const float* texcoords = texcoordadr >= 0 ? model->mesh_texcoord + (2 * texcoordadr) : nullptr;

  for (int i = 0; i < facenum; ++i) {
    const int face = 3 * (faceadr + i);

    const float3 p1 = ReadFloat3(vertices, model->mesh_face[face + 0]);
    const float3 p2 = ReadFloat3(vertices, model->mesh_face[face + 1]);
    const float3 p3 = ReadFloat3(vertices, model->mesh_face[face + 2]);
    const float3 face_normal = CalculateNormal(p1, p2, p3);
    const float3 n1 = ReadFloat3(normals, model->mesh_facenormal[face + 0]);
    const float3 n2 = ReadFloat3(normals, model->mesh_facenormal[face + 1]);
    const float3 n3 = ReadFloat3(normals, model->mesh_facenormal[face + 2]);
    const float2 uv1 = texcoords ? ReadFloat2(texcoords, model->mesh_facetexcoord[face + 0]) : float2(0, 0);
    const float2 uv2 = texcoords ? ReadFloat2(texcoords, model->mesh_facetexcoord[face + 1]) : float2(0, 0);
    const float2 uv3 = texcoords ? ReadFloat2(texcoords, model->mesh_facetexcoord[face + 2]) : float2(0, 0);

    if (UseFaceNormal(face_normal, n1)) {
      builder.Append(p1, CalculateOrientation(face_normal), uv1);
    } else {
      builder.Append(p1, CalculateOrientation(n1), uv1);
    }

    if (UseFaceNormal(face_normal, n2)) {
      builder.Append(p2, CalculateOrientation(face_normal), uv2);
    } else {
      builder.Append(p2, CalculateOrientation(n2), uv2);
    }

    if (UseFaceNormal(face_normal, n3)) {
      builder.Append(p3, CalculateOrientation(face_normal), uv3);
    } else {
      builder.Append(p3, CalculateOrientation(n3), uv3);
    }
  }
}

static void FillHeightFieldBuffer(MeshBuilder& builder, const mjModel* model,
                                  int hfieldid) {
  auto append_tri = [&](float3 a, float3 b, float3 c) {
    float4 orientation = CalculateOrientation(a, b, c);
    builder.Append(a, orientation, float2(0, 0));
    builder.Append(b, orientation, float2(0, 0));
    builder.Append(c, orientation, float2(0, 0));
  };
  auto append_quad = [&](float3 a, float3 b, float3 c, float3 d) {
    append_tri(a, b, d);
    append_tri(d, b, c);
  };

  const float* data = model->hfield_data + model->hfield_adr[hfieldid];
  const int nrow = model->hfield_nrow[hfieldid];
  const int ncol = model->hfield_ncol[hfieldid];
  const float height = 0.5f * (nrow - 1);
  const float width = 0.5f * (ncol - 1);
  float sz[4];
  for (int i = 0; i < 4; ++i) {
    sz[i] = static_cast<float>(model->hfield_size[4 * hfieldid + i]);
  }

  auto get_pos = [=](int r, int c) {
    const float x = sz[0] * (c / width - 1.0f);
    const float y = sz[1] * (r / height - 1.0f);
    const float z = sz[2] * data[(r * ncol) + c];
    return float3{x, y, z};
  };

  // For each quad defined by 4 points in the height field, we will create 4
  // triangles by introducing a vertex in the middle of the quad.
  //     a---b
  //     |\ /|
  //     | m |
  //     |/ \|
  //     d---c
  for (int row = 0; row < nrow - 1; ++row) {
    for (int col = 0; col < ncol - 1; ++col) {
      const float3 a = get_pos(row, col);
      const float3 b = get_pos(row, col + 1);
      const float3 c = get_pos(row + 1, col + 1);
      const float3 d = get_pos(row + 1, col);

      const float mid_x = (a.x + b.x) * 0.5f;
      const float mid_y = (a.y + d.y) * 0.5f;

      // To determine the height of the middle vertex, we look at the heights
      // of the opposing corners (i.e. {a, c} and {b, d}). Our goal is to avoid
      // creating any odd bumps or valleys in the height field if possible.
      //
      // If one of the two opposing corners are of the same height, then we
      // set the middle vertex such that we're effectively rendering two
      // triangles, preventing an odd bump. Otherwise, we use the higher
      // midpoint between two opposing corners to prevent valleys.
      //     0---0      0---0      6---4
      //     |\  |      |  /|      |\ /|
      //     | 0 |      | 0 |      | 7 |
      //     |  \|      |/  |      |/ \|
      //     2---0      0---2      0---8
      float mid_z = 0;
      if (a.z == c.z && b.z != d.z) {
        mid_z = a.z;
      } else if (a.z != c.z && b.z == d.z) {
        mid_z = b.z;
      } else {
        const float mid_z_ac = (a.z + c.z) * 0.5f;
        const float mid_z_bd = (b.z + d.z) * 0.5f;
        mid_z = std::max(mid_z_ac, mid_z_bd);
      }

      const float3 mid = {mid_x, mid_y, mid_z};
      append_tri(a, b, mid);
      append_tri(b, c, mid);
      append_tri(c, d, mid);
      append_tri(d, a, mid);
    }
  }
  // Build the left edge.
  for (int row = 0; row < nrow - 1; ++row) {
    const float3 a = get_pos(row, 0);
    const float3 b = get_pos(row + 1, 0);
    const float3 c = {b.x, b.y, -sz[3]};
    const float3 d = {a.x, a.y, -sz[3]};
    append_quad(a, b, c, d);
  }
  // Build the right edge.
  for (int row = 0; row < nrow - 1; ++row) {
    const float3 a = get_pos(row + 1, ncol - 1);
    const float3 b = get_pos(row, ncol - 1);
    const float3 c = {b.x, b.y, -sz[3]};
    const float3 d = {a.x, a.y, -sz[3]};
    append_quad(a, b, c, d);
  }
  // Build the front edge.
  for (int col = 0; col < ncol - 1; ++col) {
    const float3 a = get_pos(0, col);
    const float3 b = get_pos(0, col + 1);
    const float3 c = {b.x, b.y, -sz[3]};
    const float3 d = {a.x, a.y, -sz[3]};
    append_quad(a, b, c, d);
  }
  // Build the back edge.
  for (int col = 0; col < ncol - 1; ++col) {
    const float3 a = get_pos(nrow - 1, col + 1);
    const float3 b = get_pos(nrow - 1, col);
    const float3 c = {b.x, b.y, -sz[3]};
    const float3 d = {a.x, a.y, -sz[3]};
    append_quad(a, b, c, d);
  }
  // Build the base. We use the visualization quality as the size rather than
  // the height field dimensions.
  const float base_width = (0.5f * model->vis.quality.numquads);
  const float base_height = (0.5f * model->vis.quality.numquads);
  for (int row = 0; row < model->vis.quality.numquads; ++row) {
    for (int col = 0; col < model->vis.quality.numquads; ++col) {
      const float x0 = sz[0] * ((col + 0) / base_width - 1.0f);
      const float x1 = sz[0] * ((col + 1) / base_width - 1.0f);
      const float y0 = sz[1] * ((row + 0) / base_height - 1.0f);
      const float y1 = sz[1] * ((row + 1) / base_height - 1.0f);
      append_quad({x0, y0, -sz[3]}, {x0, y1, -sz[3]}, {x1, y1, -sz[3]},
                  {x1, y0, -sz[3]});
    }
  }
}

static int CalculateHeightFieldVertexCount(const mjModel* model, int hfieldid) {
  const int nrow = model->hfield_nrow[hfieldid];
  const int ncol = model->hfield_ncol[hfieldid];

  // For details, see the logic in FillHeightFieldBuffer for how many vertices
  // we need. But, in general...

  // We use 4 triangles (i.e. 12 vertices) per quad.
  const int surface_count = 12 * (nrow - 1) * (ncol - 1);
  // We use 1 quad (i.e. 6 vertices) per edge element. We double this because
  // we have two edges per dimension (e.g. left/right and front/back).
  const int edge_count = (12 * (nrow - 1)) + (12 * (ncol - 1));
  // We use 1 quad (i.e. 6 vertices) per base element. We use the visualization
  // quality as the size rather than the height field dimensions.
  const int base_count =
      6 * model->vis.quality.numquads * model->vis.quality.numquads;

  const int total_count = surface_count + edge_count + base_count;
  return total_count;
}

static bool HasUvs(const mjModel* model, int id, MeshType mesh_type) {
  return mesh_type != MeshType::kHeightField &&
         model->mesh_texcoordadr[id] >= 0;
}

static bool IsValidIndex(const mjModel* model, int id, MeshType mesh_type) {
  switch (mesh_type) {
    case MeshType::kNormal:
      return id >= 0 && id < model->nmesh;
    case MeshType::kConvexHull:
      return id >= 0 && id < model->nmesh;
    case MeshType::kHeightField:
      return id >= 0 && id < model->nhfield;
  }
}

static int GetNumVertices(const mjModel* model, int id, MeshType mesh_type) {
  switch (mesh_type) {
    case MeshType::kNormal:
      return 3 * model->mesh_facenum[id];
    case MeshType::kConvexHull:
      return 3 * model->mesh_graph[model->mesh_graphadr[id] + 1];
    case MeshType::kHeightField:
      return CalculateHeightFieldVertexCount(model, id);
  }
}

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

static void UpdateMeshData(MeshData* data, const mjModel* model, int id,
                           MeshType mesh_type) {
  if (!IsValidIndex(model, id, mesh_type)) {
    mju_error("Invalid index %d for type %d", id, mesh_type);
    return;
  }

  const int num_vertices = GetNumVertices(model, id, mesh_type);
  const bool has_uvs = HasUvs(model, id, mesh_type);

  MeshBuilder* builder = new MeshBuilder(num_vertices);
  data->user_data = builder;
  data->release_callback = [](void* user_data) {
    delete static_cast<MeshBuilder*>(user_data);
  };

  switch (mesh_type) {
    case MeshType::kNormal:
      FillMeshBuffer(*builder, model, id);
      break;
    case MeshType::kConvexHull:
      FillConvexHullBuffer(*builder, model, id);
      break;
    case MeshType::kHeightField:
      FillHeightFieldBuffer(*builder, model, id);
      break;
  }

  data->primitive_type = mjPRIM_TYPE_TRIANGLES;
  data->nvertices = num_vertices;
  data->nindices = data->nvertices;
  data->indices = nullptr;
  data->index_type = data->nvertices >= std::numeric_limits<uint16_t>::max()
                         ? mjINDEX_TYPE_UINT
                         : mjINDEX_TYPE_USHORT;
  data->nattributes = has_uvs ? 3 : 2;
  data->attributes[0].usage = mjVERTEX_ATTRIBUTE_POSITION;
  data->attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
  data->attributes[0].bytes = builder->positions.data();
  data->attributes[1].usage = mjVERTEX_ATTRIBUTE_TANGENTS;
  data->attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT4;
  data->attributes[1].bytes = builder->orientations.data();
  if (has_uvs) {
    data->attributes[2].usage = mjVERTEX_ATTRIBUTE_UV;
    data->attributes[2].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
    data->attributes[2].bytes = builder->uvs.data();
  }
  data->bounds_min[0] = builder->bounds_min.x;
  data->bounds_min[1] = builder->bounds_min.y;
  data->bounds_min[2] = builder->bounds_min.z;
  data->bounds_max[0] = builder->bounds_max.x;
  data->bounds_max[1] = builder->bounds_max.y;
  data->bounds_max[2] = builder->bounds_max.z;
}

void UpdateSkinFlexMeshData(MeshData* data, const mjModel* model,
                            const mjvScene* scene, const mjvGeom& geom) {
  auto positions = GetPositions(model, scene, geom);
  auto normals = GetNormals(model, scene, geom);
  auto uvs = GetUvs(model, scene, geom);
  auto indices = GetIndices(model, scene, geom);

  int num_indices = indices.size();
  if (num_indices == 0 && geom.type == mjGEOM_FLEX) {
    num_indices = 3 * scene->flexfaceused[geom.objid];
  }

  data->nattributes = uvs.data() ? 3 : 2;
  data->attributes[0].usage = mjVERTEX_ATTRIBUTE_POSITION;
  data->attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
  data->attributes[0].bytes = positions.data();
  data->attributes[1].usage = mjVERTEX_ATTRIBUTE_NORMAL;
  data->attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
  data->attributes[1].bytes = normals.data();
  data->attributes[2].usage = mjVERTEX_ATTRIBUTE_UV;
  data->attributes[2].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
  data->attributes[2].bytes = uvs.data();
  data->nvertices = positions.size() / 3;
  data->nindices = num_indices;
  data->indices = indices.data();
  data->index_type = mjINDEX_TYPE_UINT;
  data->primitive_type = mjPRIM_TYPE_TRIANGLES;
  data->compute_bounds = true;
  data->release_callback = nullptr;
  data->user_data = nullptr;
}

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
  meshes_.clear();
  textures_.clear();
}

void ModelObjects::UploadMesh(const mjModel* model, int id) {
  if (model != model_) {
    mju_error("Model mismatch.");
  }
  if (id < 0 || id >= model->nmesh) {
    mju_error("Invalid mesh index %d", id);
  }
  meshes_.erase(id);
  convex_hulls_.erase(id);

  MeshData data;
  DefaultMeshData(&data);
  UpdateMeshData(&data, model, id, MeshType::kNormal);
  meshes_[id] = std::make_unique<Mesh>(engine_, data);

  if (model->mesh_graphadr[id] >= 0) {
    MeshData convex_hull_data;
    DefaultMeshData(&convex_hull_data);
    UpdateMeshData(&convex_hull_data, model, id, MeshType::kConvexHull);
    convex_hulls_[id] = std::make_unique<Mesh>(engine_, convex_hull_data);
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

  height_fields_.erase(id);

  MeshData data;
  DefaultMeshData(&data);
  UpdateMeshData(&data, model, id, MeshType::kHeightField);
  height_fields_[id] = std::make_unique<Mesh>(engine_, data);
}

void ModelObjects::CreateSkinFlexMesh(const mjvScene* scene, const mjvGeom& geom) {
  MeshData data;
  DefaultMeshData(&data);
  UpdateSkinFlexMeshData(&data, model_, scene, geom);
  dynamic_meshes_[geom.objid] = std::make_unique<Mesh>(engine_, data);
}

const Mesh* ModelObjects::GetMeshBuffer(int data_id) const {
  // As defined by mjv_updateScene:
  //   original mesh: mesh_id * 2
  //   convex hull: (mesh_id * 2) + 1
  const int mesh_id = data_id / 2;
  if (data_id % 2 == 0) {
    auto it = meshes_.find(mesh_id);
    return it != meshes_.end() ? it->second.get() : nullptr;
  } else {
    auto it = convex_hulls_.find(mesh_id);
    return it != convex_hulls_.end() ? it->second.get() : nullptr;
  }
}

const Mesh* ModelObjects::GetHeightFieldBuffer(int hfield_id) const {
  auto it = height_fields_.find(hfield_id);
  return it != height_fields_.end() ? it->second.get() : nullptr;
}

const Mesh* ModelObjects::GetShapeBuffer(ShapeType shape) const {
  if (shape < 0 || shape >= kNumShapes) {
    mju_error("Invalid shape type: %d", shape);
  }
  return shapes_[shape].get();
}

const Mesh* ModelObjects::GetFlexSkinGeomMesh(int geom_id) const {
  auto it = dynamic_meshes_.find(geom_id);
  return it != dynamic_meshes_.end() ? it->second.get() : nullptr;
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
  builder.rotation(mat3f::rotation(filament::math::f::PI / 2, float3{1, 0, 0}));
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
