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

#include "experimental/filament/filament/model_util.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/texture_util.h"
#include "experimental/filament/filament/vertex_util.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

static bool UseFaceNormal(const float3& face_normal,
                          const float3& mesh_normal) {
  // clang-format off
  return face_normal[0] * mesh_normal[0] +
         face_normal[1] * mesh_normal[1] +
         face_normal[2] * mesh_normal[2] < 0.8f;
  // clang-format on
}

template <typename T>
static void FillConvexHullBuffer(T* ptr, std::size_t num, const mjModel* model,
                                int meshid) {
  const int numvert = model->mesh_graph[model->mesh_graphadr[meshid]];
  const int numface = model->mesh_graph[model->mesh_graphadr[meshid]+1];

  const int vertadr = model->mesh_vertadr[meshid];
  const float* vertices = model->mesh_vert + (3 * vertadr);
  const int texcoordadr = model->mesh_texcoordadr[meshid];
  const float* texcoords = model->mesh_texcoord + (2 * texcoordadr);

  if (num != numface * 3) {
    mju_error("Invalid vertex count.");
    return;
  }

  for (int face = 0; face < numface; ++face) {
    int j = model->mesh_graphadr[meshid] + 2 + 3*numvert + 3*numface + 3*face;

    const float3 p1 = ReadFloat3(vertices, model->mesh_graph[j + 0]);
    const float3 p2 = ReadFloat3(vertices, model->mesh_graph[j + 1]);
    const float3 p3 = ReadFloat3(vertices, model->mesh_graph[j + 2]);
    const float4 orientation = CalculateOrientation(p1, p2, p3);

    ptr->position = p1;
    ptr->orientation = orientation;
    if constexpr (T::kHasUv) {
      ptr->uv = ReadFloat2(texcoords, model->mesh_graph[j + 0]);
    }
    ++ptr;

    ptr->position = p2;
    ptr->orientation = orientation;
    if constexpr (T::kHasUv) {
      ptr->uv = ReadFloat2(texcoords, model->mesh_graph[j + 1]);
    }
    ++ptr;

    ptr->position = p3;
    ptr->orientation = orientation;
    if constexpr (T::kHasUv) {
      ptr->uv = ReadFloat2(texcoords, model->mesh_graph[j + 2]);
    }
    ++ptr;
  }
}

template <typename T>
static void FillMeshBuffer(T* ptr, std::size_t num, const mjModel* model,
                          int meshid) {
  const int faceadr = model->mesh_faceadr[meshid];
  const int facenum = model->mesh_facenum[meshid];
  if (num != facenum * 3) {
    mju_error("Invalid vertex count.");
    return;
  }

  const int vertadr = model->mesh_vertadr[meshid];
  const float* vertices = model->mesh_vert + (3 * vertadr);
  const int normaladr = model->mesh_normaladr[meshid];
  const float* normals = model->mesh_normal + 3 * normaladr;
  const int texcoordadr = model->mesh_texcoordadr[meshid];
  const float* texcoords = model->mesh_texcoord + (2 * texcoordadr);

  for (int i = 0; i < facenum; ++i) {
    const int face = 3 * (faceadr + i);

    const float3 p1 = ReadFloat3(vertices, model->mesh_face[face + 0]);
    const float3 p2 = ReadFloat3(vertices, model->mesh_face[face + 1]);
    const float3 p3 = ReadFloat3(vertices, model->mesh_face[face + 2]);
    const float3 face_normal = CalculateNormal(p1, p2, p3);

    const float3 n1 = ReadFloat3(normals, model->mesh_facenormal[face + 0]);
    const float3 n2 = ReadFloat3(normals, model->mesh_facenormal[face + 1]);
    const float3 n3 = ReadFloat3(normals, model->mesh_facenormal[face + 2]);

    ptr->position = p1;
    if constexpr (T::kHasUv) {
      ptr->orientation = CalculateOrientation(n1);
      ptr->uv = ReadFloat2(texcoords, model->mesh_facetexcoord[face + 0]);
    } else if (UseFaceNormal(face_normal, n1)) {
      ptr->orientation = CalculateOrientation(face_normal);
    } else {
      ptr->orientation = CalculateOrientation(n1);
    }
    ++ptr;

    ptr->position = p2;
    if constexpr (T::kHasUv) {
      ptr->orientation = CalculateOrientation(n2);
      ptr->uv = ReadFloat2(texcoords, model->mesh_facetexcoord[face + 1]);
    } else if (UseFaceNormal(face_normal, n2)) {
      ptr->orientation = CalculateOrientation(face_normal);
    } else {
      ptr->orientation = CalculateOrientation(n2);
    }
    ++ptr;

    ptr->position = p3;
    if constexpr (T::kHasUv) {
      ptr->orientation = CalculateOrientation(n3);
      ptr->uv = ReadFloat2(texcoords, model->mesh_facetexcoord[face + 2]);
    } else if (UseFaceNormal(face_normal, n3)) {
      ptr->orientation = CalculateOrientation(face_normal);
    } else {
      ptr->orientation = CalculateOrientation(n3);
    }
    ++ptr;
  }
}

static void FillHeightFieldBuffer(VertexNoUv* ptr, std::size_t num,
                                  const mjModel* model, int hfieldid) {
  int count = 0;
  auto append_tri = [&](float3 a, float3 b, float3 c) {
    float4 orientation = CalculateOrientation(a, b, c);
    ptr[count].position = a;
    ptr[count].orientation = orientation;
    ++count;
    ptr[count].position = b;
    ptr[count].orientation = orientation;
    ++count;
    ptr[count].position = c;
    ptr[count].orientation = orientation;
    ++count;
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
    sz[i] = static_cast<float>(model->hfield_size[4 * hfieldid  + i]);
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
    const float3 a = get_pos(row + 1, ncol-1);
    const float3 b = get_pos(row, ncol-1);
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
    const float3 a = get_pos(nrow-1, col + 1);
    const float3 b = get_pos(nrow-1, col);
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
      append_quad({x0, y0, -sz[3]},
                  {x0, y1, -sz[3]},
                  {x1, y1, -sz[3]},
                  {x1, y0, -sz[3]});
    }
  }
  if (count != num) {
    mju_error("Vertex count mismatch.");
  }
}

static int CalculateHeightFieldVertexCount(const mjModel* model, int hfieldid) {
  const int nrow = model->hfield_nrow[hfieldid];
  const int ncol = model->hfield_ncol[hfieldid];

  // For details, see the logic in FillHeightFieldBuffer for how many vertices
  // we need. But, in general...

  // We use 4 triangles (i.e. 12 vertices) per quad.
  const int surface_count = 12 * (nrow-1) * (ncol-1);
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

template <typename T, typename FillFn>
static filament::VertexBuffer* CreateVertexBuffer(filament::Engine* engine,
                                                  const mjModel* model, int id,
                                                  int vertex_count,
                                                  FillFn fill_fn) {
  return CreateVertexBuffer<T>(
      engine, vertex_count, [&](std::byte* buffer, std::size_t num_bytes) {
        auto* ptr = reinterpret_cast<T*>(buffer);
        fill_fn(ptr, num_bytes / sizeof(T), model, id);
      });
}

filament::VertexBuffer* CreateVertexBuffer(filament::Engine* engine,
                                           const mjModel* model, int id,
                                           MeshType mesh_type) {
  if (id < 0) {
    mju_error("Invalid mesh index %d", id);
    return nullptr;
  }

  int vertex_count = 0;
  switch (mesh_type) {
    case MeshType::kNormal:
      if (id >= model->nmesh) {
        mju_error("Invalid mesh index %d", id);
        return nullptr;
      }
      vertex_count = 3 * model->mesh_facenum[id];
      break;
    case MeshType::kConvexHull:
      if (id >= model->nmesh) {
        mju_error("Invalid mesh index %d", id);
        return nullptr;
      }
      vertex_count = 3 * model->mesh_graph[model->mesh_graphadr[id] + 1];
      break;
    case MeshType::kHeightField:
      if (id >= model->nhfield) {
        mju_error("Invalid height field index %d", id);
        return nullptr;
      }
      vertex_count = CalculateHeightFieldVertexCount(model, id);
      break;
  }

  if (vertex_count == 0) {
    mju_error("Vertex count is zero.");
    return nullptr;
  }

  const bool has_texcoords = mesh_type == MeshType::kHeightField
                                 ? false
                                 : model->mesh_texcoordadr[id] >= 0;
  if (has_texcoords) {
    using VertexType = VertexWithUv;
    switch (mesh_type) {
      case MeshType::kNormal:
        return CreateVertexBuffer<VertexType>(engine, model, id, vertex_count,
                                              FillMeshBuffer<VertexType>);
        break;
      case MeshType::kConvexHull:
        return CreateVertexBuffer<VertexType>(engine, model, id, vertex_count,
                                              FillConvexHullBuffer<VertexType>);
        break;
      case MeshType::kHeightField:
        mju_error("Height fields do not support UV coordinates.");
        return nullptr;
    }
  } else {
    using VertexType = VertexNoUv;
    switch (mesh_type) {
      case MeshType::kNormal:
        return CreateVertexBuffer<VertexType>(engine, model, id, vertex_count,
                                              FillMeshBuffer<VertexType>);
        break;
      case MeshType::kConvexHull:
        return CreateVertexBuffer<VertexType>(engine, model, id, vertex_count,
                                              FillConvexHullBuffer<VertexType>);
        break;
      case MeshType::kHeightField:
        return CreateVertexBuffer<VertexType>(engine, model, id, vertex_count,
                                              FillHeightFieldBuffer);
        break;
    }
  }
}

filament::IndexBuffer* CreateIndexBuffer(filament::Engine* engine,
                                         const mjModel* model, int id,
                                         MeshType mesh_type) {
  if (id < 0) {
    mju_error("Invalid index %d", id);
    return nullptr;
  }

  int index_count = 0;
  switch (mesh_type) {
    case MeshType::kNormal:
      if (id >= model->nmesh) {
        mju_error("Invalid mesh index %d", id);
        return nullptr;
      }
      index_count = 3 * model->mesh_facenum[id];
      break;
    case MeshType::kConvexHull:
      if (id >= model->nmesh) {
        mju_error("Invalid mesh index %d", id);
        return nullptr;
      }
      index_count = 3 * model->mesh_graph[model->mesh_graphadr[id] + 1];
      break;
    case MeshType::kHeightField:
      if (id >= model->nhfield) {
        mju_error("Invalid height field index %d", id);
        return nullptr;
      }
      index_count = CalculateHeightFieldVertexCount(model, id);
      break;
  }

  if (index_count == 0) {
    mju_error("Index count is zero.");
    return nullptr;
  }

  if (index_count >= std::numeric_limits<uint16_t>::max()) {
    return CreateIndexBuffer<uint32_t>(engine, index_count,
                                       FillSequence<uint32_t>);
  } else {
    return CreateIndexBuffer<uint16_t>(engine, index_count,
                                       FillSequence<uint16_t>);
  }
}

filament::Texture* CreateTexture(filament::Engine* engine, const mjModel* model,
                                 int id, TextureType texture_type) {
  if (id < 0 || id >= model->ntex) {
    mju_error("Invalid texture index %d", id);
  }

  const int width = model->tex_width[id];
  const int height = model->tex_height[id];
  const bool is_srgb = model->tex_colorspace[id] == mjCOLORSPACE_SRGB;
  const int num_channels = model->tex_nchannel[id];
  const mjtByte* data = model->tex_data + model->tex_adr[id];
  filament::Texture* texture =
      texture_type == TextureType::kNormal2d
          ? Create2dTexture(engine, width, height, num_channels, data, is_srgb)
          : CreateCubeTexture(engine, width, height, num_channels, data,
                              is_srgb);
  // TODO: Revisit this to make it this work in WebGL
  #ifndef __EMSCRIPTEN__
  texture->generateMipmaps(*engine);
  #endif
  return texture;
}
}  // namespace mujoco
