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

#include "render/filament/support/mesh_util.h"

#include <cfloat>
#include <cstddef>
#include <cstring>
#include <vector>

#include <math/TVecHelpers.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjrfilament.h>
#include "engine/engine_vis_visualize.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/filament_util.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

// Vertex types that can be used to fill in mesh data using generic functions.
namespace {

struct VertexNoUv {
  float3 position;
  float4 orientation;
  static constexpr bool kHasUv = false;
};

struct VertexWithUv {
  float3 position;
  float4 orientation;
  float2 uv;
  static constexpr bool kHasUv = true;
};

}  // namespace


static void AccumulateNormal(float3* normals, const mjtNum* src_positions,
                             const int* indices, int i0, int i1, int i2) {
  const int idx0 = indices[i0];
  const int idx1 = indices[i1];
  const int idx2 = indices[i2];

  const float3 v0 = ReadFloat3(src_positions, idx0);
  const float3 v1 = ReadFloat3(src_positions, idx1);
  const float3 v2 = ReadFloat3(src_positions, idx2);
  const float3 normal = cross(v1 - v0, v2 - v0);

  normals[idx0] += normal;
  normals[idx1] += normal;
  normals[idx2] += normal;
}

template <typename T>
static void AddFlatFace(T* vertices, const mjtNum* src_positions, float radius,
                        const int* indices, int i0, int i1, int i2) {
  const float3 v0 = ReadFloat3(src_positions, indices[i0]);
  const float3 v1 = ReadFloat3(src_positions, indices[i1]);
  const float3 v2 = ReadFloat3(src_positions, indices[i2]);

  const float3 normal = normalize(cross(v1 - v0, v2 - v0));
  vertices[0].position = v0 + (radius * normal);
  vertices[1].position = v1 + (radius * normal);
  vertices[2].position = v2 + (radius * normal);

  const float4 orientation = CalculateOrientation(normal);
  vertices[0].orientation = orientation;
  vertices[1].orientation = orientation;
  vertices[2].orientation = orientation;
}

template <typename T>
static void AddSmoothFace(T* vertices, const mjtNum* src_positions,
                          const float3* src_normals, float radius,
                          const int* indices, int i0, int i1, int i2) {
  const int idx0 = indices[i0];
  const int idx1 = indices[i1];
  const int idx2 = indices[i2];

  const float3 v0 = ReadFloat3(src_positions, idx0);
  const float3 v1 = ReadFloat3(src_positions, idx1);
  const float3 v2 = ReadFloat3(src_positions, idx2);

  vertices[0].position = v0 + (radius * src_normals[idx0]);
  vertices[1].position = v1 + (radius * src_normals[idx1]);
  vertices[2].position = v2 + (radius * src_normals[idx2]);

  const float sign = radius > 0 ? 1.f : -1.f;
  vertices[0].orientation = CalculateOrientation(sign * src_normals[idx0]);
  vertices[1].orientation = CalculateOrientation(sign * src_normals[idx1]);
  vertices[2].orientation = CalculateOrientation(sign * src_normals[idx2]);
}

// Assumes T::orientation is already set to the "smoothed" normal.
template <typename T>
static void AddSideFace(T* vertices, const mjtNum* src_positions,
                        const float3* src_normals, float radius,
                        const int* indices, int i0, int i1) {
  const int idx0 = indices[i0];
  const int idx1 = indices[i1];

  const float3 v0 = ReadFloat3(src_positions, idx0);
  const float3 v1 = ReadFloat3(src_positions, idx1);
  const float3 v01 = v1 - v0;
  float3 normal = normalize(cross(v01, src_normals[idx1]));
  if (radius < 0) {
    normal = -normal;
  }

  vertices[0].position = v0 + radius * src_normals[idx0];
  vertices[1].position = v1 - radius * src_normals[idx1];
  vertices[2].position = v1 + radius * src_normals[idx1];

  vertices[0].orientation = CalculateOrientation(normal);
  vertices[1].orientation = vertices[0].orientation;
  vertices[2].orientation = vertices[0].orientation;
}

template <typename T>
static void AddFaceUvs(T* vertices, const float* src_uvs, const int* indices,
                       int t0, int t1, int t2) {
  if constexpr (T::kHasUv) {
    if (src_uvs) {
      vertices[0].uv = ReadFloat2(src_uvs, indices[t0]);
      vertices[1].uv = ReadFloat2(src_uvs, indices[t1]);
      vertices[2].uv = ReadFloat2(src_uvs, indices[t2]);
    }
  }
}

template <typename T>
static void UpdateBounds(float3* min_pt, float3* max_pt, const T* vertices) {
  *min_pt = min(*min_pt, vertices[0].position);
  *max_pt = max(*max_pt, vertices[0].position);
  *min_pt = min(*min_pt, vertices[1].position);
  *max_pt = max(*max_pt, vertices[1].position);
  *min_pt = min(*min_pt, vertices[2].position);
  *max_pt = max(*max_pt, vertices[2].position);
}

static int CalculateVertexCount(const mjModel* model, int flex_id,
                                int flex_layer, bool smooth_skinning) {
  const int dim = model->flex_dim[flex_id];

  int num_faces = 0;
  if (dim == 1) {
    // 1d flexes have no faces.
  } else if (smooth_skinning) {
    if (dim == 2) {
      num_faces += (2 * model->flex_elemnum[flex_id]);
      num_faces += (2 * model->flex_shellnum[flex_id]);
    } else {
      num_faces += model->flex_shellnum[flex_id];
    }
  } else {
    if (dim == 2) {
      num_faces += (2 * model->flex_elemnum[flex_id]);
    } else {
      for (int e = 0; e < model->flex_elemnum[flex_id]; e++) {
        if (model->flex_elemlayer[model->flex_elemadr[flex_id] + e] ==
            flex_layer) {
          num_faces += 4;
        }
      }
    }
  }
  return num_faces * 3;
}

template <typename T>
static void FillFlexVertices(T* vertices, const mjModel* model,
                             const mjData* data, int flex_id, float3* min_pt,
                             float3* max_pt, int flex_layer,
                             bool smooth_skinning) {
  *min_pt = float3(FLT_MAX);
  *max_pt = float3(FLT_MIN);

  const int num_vertices =
      CalculateVertexCount(model, flex_id, flex_layer, smooth_skinning);
  const int dim = model->flex_dim[flex_id];
  const float radius = (float)model->flex_radius[flex_id];
  const bool flat_shading = (bool)model->flex_flatskin[flex_id];

  const mjtNum* src_positions =
      data->flexvert_xpos + 3 * model->flex_vertadr[flex_id];

  const float* src_uvs = nullptr;
  if (model->flex_texcoordadr[flex_id] >= 0) {
    src_uvs = model->flex_texcoord + 2 * model->flex_texcoordadr[flex_id];
  }

  const int* edata = model->flex_elem + model->flex_elemdataadr[flex_id];
  const int* sdata = model->flex_shell + model->flex_shelldataadr[flex_id];
  const int* tdata =
      model->flex_elemtexcoord + model->flex_elemdataadr[flex_id];

  if (dim == 1) {
    // 1D - don't render?
    return;
  }

  else if (smooth_skinning) {
    // Accumulate normals in the `orientation` field.
    std::vector<float3> normals(num_vertices, float3(0, 0, 0));
    if (dim == 2) {
      for (int e = 0; e < model->flex_elemnum[flex_id]; ++e) {
        const int* indices = edata + e * (dim + 1);
        AccumulateNormal(normals.data(), src_positions, indices, 0, 1, 2);
      }
    } else {
      for (int s = 0; s < model->flex_shellnum[flex_id]; ++s) {
        const int* indices = sdata + s * dim;
        AccumulateNormal(normals.data(), src_positions, indices, 0, 1, 2);
      }
    }

    // Normalize the accumulated normals.
    for (float3& n : normals) {
      n = normalize(n);
    }

    if (dim == 2) {
      for (int e = 0; e < model->flex_elemnum[flex_id]; ++e) {
        const int* indices = edata + (e * (dim + 1));
        const int* tex_indices = tdata + (e * (dim + 1));

        if (flat_shading) {
          AddFlatFace(vertices, src_positions, radius, indices, 0, 1, 2);
        } else {
          AddSmoothFace(vertices, src_positions, normals.data(), radius,
                        indices, 0, 1, 2);
        }
        AddFaceUvs(vertices, src_uvs, tex_indices, 0, 1, 2);
        UpdateBounds(min_pt, max_pt, vertices);
        vertices += 3;

        if (flat_shading) {
          AddFlatFace(vertices, src_positions, -radius, indices, 0, 2, 1);
        } else {
          AddSmoothFace(vertices, src_positions, normals.data(), -radius,
                        indices, 0, 2, 1);
        }
        AddFaceUvs(vertices, src_uvs, tex_indices, 0, 2, 1);
        UpdateBounds(min_pt, max_pt, vertices);
        vertices += 3;
      }
      for (int s = 0; s < model->flex_shellnum[flex_id]; ++s) {
        const int* indices = sdata + (s * dim);

        AddSideFace(vertices, src_positions, normals.data(), radius, indices, 0,
                    1);
        AddFaceUvs(vertices, src_uvs, indices, 0, 1, 1);
        UpdateBounds(min_pt, max_pt, vertices);
        vertices += 3;

        AddSideFace(vertices, src_positions, normals.data(), -radius, indices,
                    1, 0);
        AddFaceUvs(vertices, src_uvs, indices, 1, 0, 0);
        UpdateBounds(min_pt, max_pt, vertices);
        vertices += 3;
      }
    } else {
      for (int s = 0; s < model->flex_shellnum[flex_id]; ++s) {
        const int* indices = sdata + s * dim;
        if (flat_shading) {
          AddFlatFace(vertices, src_positions, radius, indices, 0, 1, 2);
        } else {
          AddSmoothFace(vertices, src_positions, normals.data(), radius,
                        indices, 0, 1, 2);
        }
        AddFaceUvs(vertices, src_uvs, indices, 0, 1, 2);
        UpdateBounds(min_pt, max_pt, vertices);
        vertices += 3;
      }
    }
  }

  // 2D or 3D face: faces from elements, flat normals, texture
  else {
    for (int e = 0; e < model->flex_elemnum[flex_id]; e++) {
      // in 3D, show only elements in selected layer
      if (dim == 2 || model->flex_elemlayer[model->flex_elemadr[flex_id] + e] ==
                          flex_layer) {
        const int* edata2 = edata + e * (dim + 1);
        const int* tdata2 = tdata + e * (dim + 1);

        if (dim == 2) {
          AddFlatFace(vertices, src_positions, radius, edata2, 0, 1, 2);
          AddFaceUvs(vertices, src_uvs, tdata2, 0, 1, 2);
          UpdateBounds(min_pt, max_pt, vertices);
          vertices += 3;

          AddFlatFace(vertices, src_positions, radius, edata2, 0, 2, 1);
          AddFaceUvs(vertices, src_uvs, tdata2, 0, 2, 1);
          UpdateBounds(min_pt, max_pt, vertices);
          vertices += 3;
        } else {
          AddFlatFace(vertices, src_positions, radius, edata2, 0, 1, 2);
          AddFaceUvs(vertices, src_uvs, tdata2, 0, 1, 2);
          UpdateBounds(min_pt, max_pt, vertices);
          vertices += 3;

          AddFlatFace(vertices, src_positions, radius, edata2, 0, 2, 3);
          AddFaceUvs(vertices, src_uvs, tdata2, 0, 2, 3);
          UpdateBounds(min_pt, max_pt, vertices);
          vertices += 3;

          AddFlatFace(vertices, src_positions, radius, edata2, 0, 3, 1);
          AddFaceUvs(vertices, src_uvs, tdata2, 0, 3, 1);
          UpdateBounds(min_pt, max_pt, vertices);
          vertices += 3;

          AddFlatFace(vertices, src_positions, radius, edata2, 1, 3, 2);
          AddFaceUvs(vertices, src_uvs, tdata2, 1, 3, 2);
          UpdateBounds(min_pt, max_pt, vertices);
          vertices += 3;
        }
      }
    }
  }
}

template <typename T>
static void FillSkinVertices(T* vertices, const mjModel* model,
                             const mjData* data, int skin_id, float3* min_pt,
                             float3* max_pt) {
  const int vertadr = model->skin_vertadr[skin_id];
  const int faceadr = model->skin_faceadr[skin_id];
  const int facenum = model->skin_facenum[skin_id];
  const int boneadr = model->skin_boneadr[skin_id];
  const int bonenum = model->skin_bonenum[skin_id];

  // Accumulate positions from all bones.
  for (int bone_idx = boneadr; bone_idx < boneadr + bonenum; ++bone_idx) {
    mjtNum bind_pos[3] = {(mjtNum)model->skin_bonebindpos[3 * bone_idx + 0],
                          (mjtNum)model->skin_bonebindpos[3 * bone_idx + 1],
                          (mjtNum)model->skin_bonebindpos[3 * bone_idx + 2]};
    mjtNum bind_quat[4] = {(mjtNum)model->skin_bonebindquat[4 * bone_idx + 0],
                           (mjtNum)model->skin_bonebindquat[4 * bone_idx + 1],
                           (mjtNum)model->skin_bonebindquat[4 * bone_idx + 2],
                           (mjtNum)model->skin_bonebindquat[4 * bone_idx + 3]};

    const int body_id = model->skin_bonebodyid[bone_idx];
    const mjtNum* body_quat = data->xquat + 4 * body_id;
    const mjtNum* body_pos = data->xpos + 3 * body_id;

    // Apply the bone's current pose to the bind pose.
    mjtNum neg_bind_quat[4];
    mju_negQuat(neg_bind_quat, bind_quat);

    mjtNum quat[4];
    mju_mulQuat(quat, body_quat, neg_bind_quat);

    mjtNum rotate[9];
    mju_quat2Mat(rotate, quat);

    mjtNum translate[3];
    mju_mulMatVec3(translate, rotate, bind_pos);
    mju_sub3(translate, body_pos, translate);

    // Apply the bone's position to all vertices "connected" to the bone by
    // the weighting of the bone to the vertex.
    const int bonevertadr = model->skin_bonevertadr[bone_idx];
    const int bonevertnum = model->skin_bonevertnum[bone_idx];
    for (int i = bonevertadr; i < bonevertadr + bonevertnum; ++i) {
      const int vertex_id = model->skin_bonevertid[i];
      const mjtNum base_pos[3] = {
          (mjtNum)model->skin_vert[3 * (vertadr + vertex_id) + 0],
          (mjtNum)model->skin_vert[3 * (vertadr + vertex_id) + 1],
          (mjtNum)model->skin_vert[3 * (vertadr + vertex_id) + 2],
      };
      mjtNum unweighted_pos[3];
      mju_mulMatVec3(unweighted_pos, rotate, base_pos);
      mju_addTo3(unweighted_pos, translate);

      const float weight = model->skin_bonevertweight[i];
      vertices[vertex_id].position.x += weight * (float)unweighted_pos[0];
      vertices[vertex_id].position.y += weight * (float)unweighted_pos[1];
      vertices[vertex_id].position.z += weight * (float)unweighted_pos[2];
    }
  }

  // Compute normals for each face. For now, we'll store the normals in the
  // xyz components of the orientation field.
  for (int i = faceadr; i < faceadr + facenum; ++i) {
    const int i0 = model->skin_face[(3 * i) + 0];
    const int i1 = model->skin_face[(3 * i) + 1];
    const int i2 = model->skin_face[(3 * i) + 2];
    T& v0 = vertices[i0];
    T& v1 = vertices[i1];
    T& v2 = vertices[i2];

    const float3 vec01 = v1.position - v0.position;
    const float3 vec02 = v2.position - v0.position;
    const float4 normal = float4(cross(vec01, vec02), 0);

    v0.orientation += normal;
    v1.orientation += normal;
    v2.orientation += normal;
  }

  const float* uvs = nullptr;
  if (model->skin_texcoordadr[skin_id] >= 0) {
    const int uaddr = model->skin_texcoordadr[skin_id];
    uvs = model->skin_texcoord + (2 * uaddr);
  }

  // Perform final adjustments/corrections on all the vertices.
  const float inflate = model->skin_inflate[skin_id];
  const size_t num_vertices = model->skin_vertnum[skin_id];
  for (int i = 0; i < num_vertices; ++i) {
    // Ensure normals are normalized.
    vertices[i].orientation = normalize(vertices[i].orientation);

    // Inflate the vertex position in direction of normal (if applicable).
    if (inflate != 0.0f) {
      vertices[i].position += inflate * vertices[i].orientation.xyz;
    }

    // Convert the normals into orientations.
    vertices[i].orientation = CalculateOrientation(vertices[i].orientation.xyz);

    // Assign uvs (if applicable).
    if constexpr (T::kHasUv) {
      vertices[i].uv = ReadFloat2(uvs, i);
    }

    // Calculate the bounds of the vertex buffer.
    *min_pt = min(*min_pt, vertices[i].position);
    *max_pt = max(*max_pt, vertices[i].position);
  }
}

static mjrfMeshData PrepareMeshData(int num_vertices, bool has_uvs) {
  mjrfMeshData mesh_data;
  mjrf_defaultMeshData(&mesh_data);

  if (has_uvs) {
    const int nbytes = sizeof(VertexWithUv) * num_vertices;
    mesh_data.user_data = new char[nbytes];
    std::memset(mesh_data.user_data, 0, nbytes);
  } else {
    const int nbytes = sizeof(VertexNoUv) * num_vertices;
    mesh_data.user_data = new char[nbytes];
    std::memset(mesh_data.user_data, 0, nbytes);
  }
  mesh_data.release = [](void* user_data) {
    delete[] (char*)(user_data);
  };

  char* buf = reinterpret_cast<char*>(mesh_data.user_data);
  mesh_data.num_vertices = num_vertices;
  mesh_data.num_attributes = has_uvs ? 3 : 2;
  mesh_data.interleaved = true;
  mesh_data.attributes[0].usage = mjVERTEX_ATTRIBUTE_USAGE_POSITION;
  mesh_data.attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
  mesh_data.attributes[0].bytes = buf;
  mesh_data.attributes[1].usage = mjVERTEX_ATTRIBUTE_USAGE_TANGENTS;
  mesh_data.attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT4;
  mesh_data.attributes[1].bytes = buf + sizeof(float[3]);
  if (has_uvs) {
    mesh_data.attributes[2].usage = mjVERTEX_ATTRIBUTE_USAGE_UV;
    mesh_data.attributes[2].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
    mesh_data.attributes[2].bytes = buf + sizeof(float[7]);
  }
  mesh_data.primitive_type = mjMESH_PRIMITIVE_TYPE_TRIANGLES;
  mesh_data.num_indices = num_vertices;
  mesh_data.index_type = mjINDEX_TYPE_U32;
  mesh_data.indices = nullptr;
  return mesh_data;
}

static void SetBounds(mjrfMeshData* mesh_data, const float3& min_pt,
                      const float3& max_pt) {
  mesh_data->bounds_min[0] = min_pt.x;
  mesh_data->bounds_min[1] = min_pt.y;
  mesh_data->bounds_min[2] = min_pt.z;
  mesh_data->bounds_max[0] = max_pt.x;
  mesh_data->bounds_max[1] = max_pt.y;
  mesh_data->bounds_max[2] = max_pt.z;
}

UniquePtr<mjrfMesh> CreateFlexMesh(mjrfContext* ctx, const mjModel* model,
                                   const mjData* data, int flex_id,
                                   int flex_layer, bool smooth_skinning,
                                   bool generate_edges,
                                   bool generate_vertices) {
  // TODO: generate edges and vertices if requested.
  const int num_vertices =
      CalculateVertexCount(model, flex_id, flex_layer, smooth_skinning);
  const bool has_uvs = model->flex_texcoordadr[flex_id] >= 0;
  mjrfMeshData mesh_data = PrepareMeshData(num_vertices, has_uvs);

  float3 min_pt = float3(FLT_MAX);
  float3 max_pt = float3(FLT_MIN);
  if (has_uvs) {
    VertexWithUv* vertices = (VertexWithUv*)(mesh_data.user_data);
    FillFlexVertices(vertices, model, data, flex_id, &min_pt, &max_pt,
                     flex_layer, smooth_skinning);
  } else {
    VertexNoUv* vertices = (VertexNoUv*)(mesh_data.user_data);
    FillFlexVertices(vertices, model, data, flex_id, &min_pt, &max_pt,
                     flex_layer, smooth_skinning);
  }
  SetBounds(&mesh_data, min_pt, max_pt);
  return CreateMesh(ctx, mesh_data);
}

UniquePtr<mjrfMesh> CreateSkinMesh(mjrfContext* ctx, const mjModel* model,
                                  const mjData* data, int skin_id) {
  const int num_vertices = model->skin_vertnum[skin_id];
  const bool has_uvs = model->skin_texcoordadr[skin_id] >= 0;
  mjrfMeshData mesh_data = PrepareMeshData(num_vertices, has_uvs);

  float3 min_pt = float3(FLT_MAX);
  float3 max_pt = float3(FLT_MIN);
  if (has_uvs) {
    VertexWithUv* vertices = (VertexWithUv*)(mesh_data.user_data);
    FillSkinVertices(vertices, model, data, skin_id, &min_pt, &max_pt);
  } else {
    VertexNoUv* vertices = (VertexNoUv*)(mesh_data.user_data);
    FillSkinVertices(vertices, model, data, skin_id, &min_pt, &max_pt);
  }
  mesh_data.num_indices = 3 * model->skin_facenum[skin_id];
  mesh_data.indices = model->skin_face + 3 * model->skin_faceadr[skin_id];
  SetBounds(&mesh_data, min_pt, max_pt);
  return CreateMesh(ctx, mesh_data);
}

void GatherSpatialTendonPoints(const mjModel* model, const mjData* data,
                               int tendon_id, std::vector<float4>& points) {
  mjtNum length = 0.f;
  const bool is_catenary = mjv_isCatenary(model, data, tendon_id, &length);
  if (is_catenary) {
    const int max_segments = mjMIN(model->vis.quality.numslices + 1, 100);

    mjtNum x0[3];
    mju_copy3(x0, data->wrap_xpos + 3 * data->ten_wrapadr[tendon_id] + 0);

    mjtNum x1[3];
    mju_copy3(x1, data->wrap_xpos + 3 * data->ten_wrapadr[tendon_id] + 3);

    const float width = model->tendon_width[tendon_id];

    mjtNum pts[3 * 100];
    const int npoints =
        mjv_catenary(x0, x1, model->opt.gravity, length, pts, max_segments);

    for (int j = 0; j < npoints - 1; ++j) {
      points.emplace_back(ReadFloat3(pts, j), width);
      points.emplace_back(ReadFloat3(pts, j + 1), width);
    }
  } else {
    const int adr = data->ten_wrapadr[tendon_id];
    const int num = data->ten_wrapnum[tendon_id];
    for (int j = adr; j < adr + num - 1; j++) {
      if (data->wrap_obj[j] == -2 || data->wrap_obj[j + 1] == -2) {
        continue;
      }

      float width = model->tendon_width[tendon_id];
      if (data->wrap_obj[j] >= 0 && data->wrap_obj[j + 1] >= 0) {
        width *= 0.5;
      }
      points.emplace_back(ReadFloat3(data->wrap_xpos, j + 0), width);
      points.emplace_back(ReadFloat3(data->wrap_xpos, j + 1), width);
    }
  }
}
}  // namespace mujoco
