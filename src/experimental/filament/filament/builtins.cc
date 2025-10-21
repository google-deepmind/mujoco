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

#include "experimental/filament/filament/builtins.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <filament/Box.h>
#include <filament/Engine.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/vertex_util.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

static constexpr size_t kNumVerticesPerTriangle = 3;
static constexpr size_t kNumVerticesPerQuad = 4;
static constexpr size_t kNumIndicesPerTriangle = 3;
static constexpr size_t kNumIndicesPerQuad = 6;

static int AppendQuadIndices(uint16_t* ptr, int idx, uint16_t a, uint16_t b,
                             uint16_t c, uint16_t d) {
  ptr[idx++] = a;
  ptr[idx++] = b;
  ptr[idx++] = c;
  ptr[idx++] = a;
  ptr[idx++] = c;
  ptr[idx++] = d;
  return idx;
}

std::size_t NumVerticesPerSide(int num_quads_per_axis) {
  return (num_quads_per_axis + 1) * (num_quads_per_axis + 1);
}

std::size_t NumIndicesPerSide(int num_quads_per_axis) {
  return kNumIndicesPerQuad * num_quads_per_axis * num_quads_per_axis;
}

class LineBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::LINES;

  explicit LineBuilder() {}

  std::size_t NumVertices() const {
    return 2;
  }

  std::size_t NumIndices() const {
    return 2;
  }

  void GenerateVertices(VertexType* ptr, size_t num) const {
    constexpr float4 kOrientation = {0, 0, 0, 1};  // Unused for lines.
    ptr[0] = VertexType({0, 0, 0}, kOrientation);
    ptr[1] = VertexType({0, 0, 1}, kOrientation);
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    ptr[0] = 0;
    ptr[1] = 1;
  }

  filament::Box GetBounds() const {
    return {{-0.001, -0.001, 0}, {0.001, 0.001, 1}};
  }
};

class PlaneBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  explicit PlaneBuilder(int num_quads_per_axis)
      : num_quads_per_axis_(num_quads_per_axis),
        orientation_(CalculateOrientation({0, 0, 1})) {}

  std::size_t NumVertices() const {
    return NumVerticesPerSide(num_quads_per_axis_);
  }

  std::size_t NumIndices() const {
    return NumIndicesPerSide(num_quads_per_axis_);
  }

  void GenerateVertices(VertexType* ptr, size_t num) const {
    const float delta = 2.0f / num_quads_per_axis_;

    int idx = 0;
    for (int x = 0; x <= num_quads_per_axis_; ++x) {
      for (int y = 0; y <= num_quads_per_axis_; ++y) {
        const float dx = delta * static_cast<float>(x);
        const float dy = delta * static_cast<float>(y);
        ptr[idx++] = VertexType({dx - 1.0f, dy - 1.0f, 0}, orientation_);
      }
    }
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    int idx = 0;
    for (int x = 0; x < num_quads_per_axis_; ++x) {
      for (int y = 0; y < num_quads_per_axis_; ++y) {
        const int base_idx = x * (num_quads_per_axis_ + 1) + y;
        const int i0 = base_idx + 0;
        const int i1 = base_idx + 1;
        const int i2 = base_idx + num_quads_per_axis_ + 2;
        const int i3 = base_idx + num_quads_per_axis_ + 1;
        idx = AppendQuadIndices(ptr, idx, i0, i1, i2, i3);
      }
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, -0.001}, {1, 1, 0.001}}; }

 private:
  int num_quads_per_axis_;
  float4 orientation_;
};

class LineBoxBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::LINES;

  explicit LineBoxBuilder() {}

  std::size_t NumVertices() const {
    return 8;
  }

  std::size_t NumIndices() const {
    return 24;
  }

  void GenerateVertices(VertexType* ptr, size_t num) const {
    constexpr float4 kOrientation = {0, 0, 0, 1};  // Unused for lines.
    ptr[0] = VertexType({-1.0f, -1.0f, -1.0f}, kOrientation);
    ptr[1] = VertexType({ 1.0f, -1.0f, -1.0f}, kOrientation);
    ptr[2] = VertexType({-1.0f,  1.0f, -1.0f}, kOrientation);
    ptr[3] = VertexType({ 1.0f,  1.0f, -1.0f}, kOrientation);
    ptr[4] = VertexType({-1.0f, -1.0f,  1.0f}, kOrientation);
    ptr[5] = VertexType({ 1.0f, -1.0f,  1.0f}, kOrientation);
    ptr[6] = VertexType({-1.0f,  1.0f,  1.0f}, kOrientation);
    ptr[7] = VertexType({ 1.0f,  1.0f,  1.0f}, kOrientation);
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    // Bottom square (where z == -1).
    ptr[0] = 0;
    ptr[1] = 1;
    ptr[2] = 1;
    ptr[3] = 3;
    ptr[4] = 3;
    ptr[5] = 2;
    ptr[6] = 2;
    ptr[7] = 0;
    // Top square (where z == 1).
    ptr[8] = 4;
    ptr[9] = 5;
    ptr[10] = 5;
    ptr[11] = 7;
    ptr[12] = 7;
    ptr[13] = 6;
    ptr[14] = 6;
    ptr[15] = 4;
    // Connect edges from bottom to top.
    ptr[16] = 2;
    ptr[17] = 6;
    ptr[18] = 3;
    ptr[19] = 7;
    ptr[20] = 0;
    ptr[21] = 4;
    ptr[22] = 1;
    ptr[23] = 5;
  }

  filament::Box GetBounds() const { return {{-1, -1, -1}, {1, 1, 1}}; }
};

class BoxBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  static constexpr int kNumSides = 6;

  explicit BoxBuilder(int num_quads_per_axis)
      : num_quads_per_axis_(num_quads_per_axis) {
    quad_size_ = 2.0f / static_cast<float>(num_quads_per_axis_);
  }

  std::size_t NumVertices() const {
    return NumVerticesPerSide(num_quads_per_axis_) * kNumSides;
  }

  std::size_t NumIndices() const {
    return NumIndicesPerSide(num_quads_per_axis_) * kNumSides;
  }

  void GenerateVertices(VertexType* ptr, size_t num) const {
    int idx = 0;
    idx = GenerateVerticesForSide(ptr, idx, {0, 1, 0}, [](float2 pt) {
      return float3{pt.x, 1.0f, pt.y};
    });
    idx = GenerateVerticesForSide(ptr, idx, {0, -1, 0}, [](float2 pt) {
      return float3{pt.x, -1.0f, pt.y};
    });
    idx = GenerateVerticesForSide(ptr, idx, {1, 0, 0}, [](float2 pt) {
      return float3{1.0f, pt.x, pt.y};
    });
    idx = GenerateVerticesForSide(ptr, idx, {-1, 0, 0}, [](float2 pt) {
      return float3{-1.0f, pt.x, pt.y};
    });
    idx = GenerateVerticesForSide(ptr, idx, {0, 0, 1}, [](float2 pt) {
      return float3{pt.x, pt.y, 1.0f};
    });
    idx = GenerateVerticesForSide(ptr, idx, {0, 0, -1}, [](float2 pt) {
      return float3{pt.x, pt.y, -1.0f};
    });
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    const int vertices_per_side = NumVerticesPerSide(num_quads_per_axis_);

    int idx = 0;
    for (int i = 0; i < kNumSides; ++i) {
      for (int x = 0; x < num_quads_per_axis_; ++x) {
        for (int y = 0; y < num_quads_per_axis_; ++y) {
          const int base_idx =
              (i * vertices_per_side) + (x * (num_quads_per_axis_ + 1)) + y;
          const int i0 = base_idx + 0;
          const int i1 = base_idx + 1;
          const int i2 = base_idx + num_quads_per_axis_ + 2;
          const int i3 = base_idx + num_quads_per_axis_ + 1;
          idx = AppendQuadIndices(ptr, idx, i0, i1, i2, i3);
        }
      }
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, -1}, {1, 1, 1}}; }

 private:
  template <typename F>
  int GenerateVerticesForSide(VertexType* ptr, int idx, float3 normal,
                              const F& pt_gen) const {
    float4 orientation = CalculateOrientation(normal);
    for (int x = 0; x <= num_quads_per_axis_; ++x) {
      for (int y = 0; y <= num_quads_per_axis_; ++y) {
        const float dx = -1.0f + (quad_size_ * static_cast<float>(x));
        const float dy = -1.0f + (quad_size_ * static_cast<float>(y));
        const float3 position = pt_gen({dx, dy});
        ptr[idx++] = VertexType(position, orientation);
      }
    }
    return idx;
  }

  int num_quads_per_axis_;
  float quad_size_;
};

class TubeBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  TubeBuilder(int num_stacks, int num_slices)
      : num_stacks_(num_stacks), num_slices_(num_slices) {}

  std::size_t NumVertices() const {
    return num_slices_ * (num_stacks_ + 1);
  }

  std::size_t NumIndices() const {
    return kNumIndicesPerQuad * num_slices_ * num_stacks_;
  }

  void GenerateVertices(VertexType* ptr, size_t num) const {
    const float delta_angle = 2.f * M_PI / (float)num_slices_;
    const float delta_stack = 2.f / static_cast<float>(num_stacks_);

    int idx = 0;
    for (int i = 0; i < num_slices_; ++i) {
      const float angle = static_cast<float>(i) * delta_angle;
      const float2 pt{std::cos(angle), std::sin(angle)};
      const float4 orientation = CalculateOrientation({pt.x, pt.y, 0});
      for (int j = 0; j <= num_stacks_; ++j) {
        const float z = -1.0f + (static_cast<float>(j) * delta_stack);
        ptr[idx++] = VertexType({pt.x, pt.y, z}, orientation);
      }
    }
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    const int num_vertices = NumVertices();
    const int num_vertices_in_spine = num_stacks_ + 1;

    int idx = 0;
    for (int i = 0; i < num_slices_; ++i) {
      for (int j = 0; j < num_stacks_; ++j) {
        const int base_idx = (i * num_vertices_in_spine) + j;
        const int i0 = base_idx + 0;
        const int i1 = base_idx + 1;
        const int i2 = (base_idx + num_stacks_ + 2) % num_vertices;
        const int i3 = (base_idx + num_stacks_ + 1) % num_vertices;
        idx = AppendQuadIndices(ptr, idx, i0, i1, i2, i3);
      }
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, -1}, {1, 1, 1}}; }

 private:
  int num_stacks_;
  int num_slices_;
};

class ConeBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  ConeBuilder(int num_stacks, int num_slices)
      : num_stacks_(num_stacks), num_slices_(num_slices) {}

  std::size_t NumVertices() const {
    return (num_slices_ * kNumVerticesPerTriangle) +
           ((num_stacks_ - 1) * num_slices_ * kNumVerticesPerQuad);
  }

  std::size_t NumIndices() const {
    return (num_slices_ * kNumIndicesPerTriangle) +
           ((num_stacks_ - 1) * num_slices_ * kNumIndicesPerQuad);
  }
  void GenerateVertices(VertexType* ptr, std::size_t num) const {
    // pole: use triangles
    const float delta_angle = 2.0 * M_PI / static_cast<float>(num_slices_);
    const float delta_radius = 1.0f / static_cast<float>(num_stacks_);

    int idx = 0;
    for (int j = 0; j < num_slices_; ++j) {
      const float angle1 = (j+0) * delta_angle;
      const float angle2 = (j+1) * delta_angle;

      ptr[idx++] = MakeVert(angle1, delta_radius);
      ptr[idx++] = MakeVert(angle2, delta_radius);

      VertexType v3;
      v3.position = {0, 0, 1};
      v3.orientation = CalculateOrientation(v3.position);
      ptr[idx++] = v3;
    }

    // the rest: use quads
    for (int i = 1; i < num_stacks_; ++i) {
      const float radius1 = delta_radius * (i+0);
      const float radius2 = delta_radius * (i+1);

      for (int j = 0; j < num_slices_; ++j) {
        const float angle1 = (j+0) * delta_angle;
        const float angle2 = (j+1) * delta_angle;

        ptr[idx++] = MakeVert(angle1, radius2);
        ptr[idx++] = MakeVert(angle2, radius2);
        ptr[idx++] = MakeVert(angle2, radius1);
        ptr[idx++] = MakeVert(angle1, radius1);
      }
    }
  }

  void GenerateIndices(IndexType* ptr, std::size_t num) const {
    int idx = 0;
    for (int j = 0; j < num_slices_ * 3; ++j) {
      ptr[idx] = idx;
      ++idx;
    }

    int quad_idx = idx;
    for (int i = 1; i < num_stacks_; ++i) {
      for (int j = 0; j < num_slices_; ++j) {
        const int i0 = quad_idx + 0;
        const int i1 = quad_idx + 1;
        const int i2 = quad_idx + 2;
        const int i3 = quad_idx + 3;
        quad_idx += 4;
        idx = AppendQuadIndices(ptr, idx, i0, i1, i2, i3);
      }
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, 0}, {1, 1, 1}}; }

 private:
  static VertexType MakeVert(float theta, float radius) {
    static constexpr float kNormalScale = 0.70710678118f;
    const float cz = std::cos(theta);
    const float sz = std::sin(theta);
    const float3 pt{cz * radius, sz * radius, 1.f - radius};
    const float3 n{cz * kNormalScale, sz * kNormalScale, kNormalScale};
    return VertexType(pt, CalculateOrientation(n));
  }

  int num_stacks_;
  int num_slices_;
};

class DiskBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  explicit DiskBuilder(int num_slices) : num_slices_(num_slices) {
    orientation_ = CalculateOrientation({0, 0, 1});
  }

  std::size_t NumVertices() const {
    return num_slices_ + 1;
  }

  std::size_t NumIndices() const {
    return num_slices_ * kNumVerticesPerTriangle;
  }

  void GenerateVertices(VertexType* ptr, std::size_t num) const {
    const float delta_angle = 2.0 * M_PI / static_cast<float>(num_slices_);

    int idx = 0;
    ptr[idx++] = VertexType(float3{0, 0, 0}, orientation_);
    for (int i = 0; i < num_slices_; ++i) {
      const float angle = static_cast<float>(i) * delta_angle;
      const float x = std::cos(angle);
      const float y = std::sin(angle);
      ptr[idx++] = VertexType(float3{x, y, 0}, orientation_);
    }
  }

  void GenerateIndices(IndexType* ptr, std::size_t num) const {
    int idx = 0;
    for (int i = 0; i < num_slices_; ++i) {
      const int next = i < (num_slices_ - 1) ? i + 1 : 0;
      ptr[idx++] = 0;
      ptr[idx++] = 1 + i;
      ptr[idx++] = 1 + next;
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, -0.001}, {1, 1, 0.001}}; }

 private:
  int num_slices_;
  float4 orientation_;
};

class SphereBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  static constexpr IndexType kNorthPoleIndex = 0;
  static constexpr IndexType kSouthPoleIndex = 1;

  SphereBuilder(int num_stacks, int num_slices)
      : num_stacks_(num_stacks), num_slices_(num_slices) {}

  std::size_t NumVertices() const {
    return (num_stacks_ * num_slices_) + 2;  // +2 for poles
  }

  std::size_t NumIndices() const {
    const size_t num_tris_polar_cap = num_slices_;
    const size_t num_quads_body = num_slices_ * (num_stacks_ - 1);
    return (2 * num_tris_polar_cap * kNumIndicesPerTriangle) +
           (num_quads_body * kNumIndicesPerQuad);
  }

  void GenerateVertices(VertexType* ptr, size_t num) const {
    const float lat_angle_delta = M_PI / static_cast<float>(num_stacks_ + 1);
    const float lon_angle_delta = 2.0 * M_PI / static_cast<float>(num_slices_);

    // Add the north and south poles.
    int idx = 0;
    ptr[idx++] = MakeVert(0, 0, 1);
    ptr[idx++] = MakeVert(0, 0, -1);

    // Vertices by latitude.
    for (int lat = 0; lat < num_stacks_; ++lat) {
      // +1 because we handle the north pole (which would be at a lat angle of
      // 0-degrees) explicitly.
      const float lat_angle = static_cast<float>(lat + 1) * lat_angle_delta;
      const float cos_lat_angle = std::cos(lat_angle);
      const float sin_lat_angle = std::sin(lat_angle);
      const float z = cos_lat_angle;

      for (int lon = 0; lon < num_slices_; ++lon) {
        const float lon_angle = static_cast<float>(lon) * lon_angle_delta;

        const float cos_lon_angle = std::cos(lon_angle);
        const float sin_lon_angle = std::sin(lon_angle);

        const float x = sin_lat_angle * cos_lon_angle;
        const float y = sin_lat_angle * sin_lon_angle;
        ptr[idx++] = MakeVert(x, y, z);
      }
    }
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    int idx = 0;

    // The first two vertices are the poles, so the first vertex in the first
    // row starts at index 2.
    IndexType row_start = kSouthPoleIndex + 1;

    // North polar cap.
    for (int lon = 0; lon < num_slices_; ++lon) {
      const int next = lon < (num_slices_ - 1) ? lon + 1 : 0;
      ptr[idx++] = kNorthPoleIndex;
      ptr[idx++] = row_start + next;
      ptr[idx++] = row_start + lon;
    }

    // Latitudinal triangle strips.
    for (int lat = 0; lat < num_stacks_ - 1; lat++) {
      const IndexType north_start = row_start;
      const IndexType south_start = row_start + num_slices_;
      for (int lon = 0; lon < num_slices_; ++lon) {
        // The offset to the index that is adjacent to the current index.
        const int adjacent = lon < (num_slices_ - 1) ? lon + 1 : 0;

        const int i0 = (north_start + lon);
        const int i1 = (south_start + lon);
        const int i2 = (south_start + adjacent);
        const int i3 = (north_start + adjacent);
        idx = AppendQuadIndices(ptr, idx, i0, i1, i2, i3);
      }
      row_start += num_slices_;
    }

    // South polar cap.
    for (int lon = 0; lon < num_slices_; ++lon) {
      const int adjacent = lon < (num_slices_ - 1) ? lon + 1 : 0;
      ptr[idx++] = kSouthPoleIndex;
      ptr[idx++] = row_start + lon;
      ptr[idx++] = row_start + adjacent;
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, -1}, {1, 1, 1}}; }

 private:
  static VertexType MakeVert(float x, float y, float z) {
    const float3 pt{x, y, z};
    return VertexType(pt, CalculateOrientation(pt));
  }

  int num_stacks_;
  int num_slices_;
};

class DomeBuilder {
 public:
  using VertexType = VertexNoUv;
  using IndexType = uint16_t;
  static constexpr filament::RenderableManager::PrimitiveType kPrimitiveType =
      filament::RenderableManager::PrimitiveType::TRIANGLES;

  static constexpr IndexType kPoleIndex = 0;

  DomeBuilder(int num_stacks, int num_slices)
      : num_stacks_(num_stacks), num_slices_(num_slices) {}

  std::size_t NumVertices() const {
    return (num_stacks_ * num_slices_) + 1;  // +1 for poles
  }

  std::size_t NumIndices() const {
    const size_t num_tris_polar_cap = num_slices_;
    const size_t num_quads_body = num_slices_ * (num_stacks_ - 1);
    return (num_tris_polar_cap * kNumIndicesPerTriangle) +
           (num_quads_body * kNumIndicesPerQuad);
  }

    void GenerateVertices(VertexType* ptr, size_t num) const {
    const float lat_angle_delta = 0.5 * M_PI / static_cast<float>(num_stacks_);
    const float lon_angle_delta = 2.0 * M_PI / static_cast<float>(num_slices_);

    // Add the pole.
    int idx = 0;
    ptr[idx++] = MakeVert(0, 0, 1);

    // Vertices by latitude.
    for (int lat = 0; lat < num_stacks_; ++lat) {
      // +1 because we handle the north pole (which would be at a lat angle of
      // 0-degrees) explicitly.
      const float lat_angle = static_cast<float>(lat + 1) * lat_angle_delta;
      const float cos_lat_angle = std::cos(lat_angle);
      const float sin_lat_angle = std::sin(lat_angle);
      const float z = cos_lat_angle;

      for (int lon = 0; lon < num_slices_; ++lon) {
        const float lon_angle = static_cast<float>(lon) * lon_angle_delta;
        const float cos_lon_angle = std::cos(lon_angle);
        const float sin_lon_angle = std::sin(lon_angle);

        const float x = sin_lat_angle * cos_lon_angle;
        const float y = sin_lat_angle * sin_lon_angle;
        ptr[idx++] = MakeVert(x, y, z);
      }
    }
  }

  void GenerateIndices(IndexType* ptr, size_t num) const {
    int idx = 0;

    // The first vertex is the poles, so the first vertex in the first row
    // starts at index 1.
    IndexType row_start = kPoleIndex + 1;

    // North polar cap.
    for (int lon = 0; lon < num_slices_; ++lon) {
      const int next = lon < (num_slices_ - 1) ? lon + 1 : 0;

      ptr[idx++] = kPoleIndex;
      ptr[idx++] = row_start + next;
      ptr[idx++] = row_start + lon;
    }

    // Latitudinal quad strips.  The first "stack" was handled above, so we
    // only need to iterate over N-1 stacks.
    for (int lat = 0; lat < num_stacks_ - 1; lat++) {
      const int north_start = row_start;
      const int south_start = row_start + num_slices_;
      for (int lon = 0; lon < num_slices_; ++lon) {
        // The offset to the index that is adjacent to the current index.
        const int adjacent = lon < (num_slices_ - 1) ? lon + 1 : 0;

        const int i0 = (north_start + lon);
        const int i1 = (south_start + lon);
        const int i2 = (south_start + adjacent);
        const int i3 = (north_start + adjacent);
        idx = AppendQuadIndices(ptr, idx, i0, i1, i2, i3);
      }
      row_start += num_slices_;
    }
  }

  filament::Box GetBounds() const { return {{-1, -1, 0}, {1, 1, 1}}; }

 private:
  static VertexType MakeVert(float x, float y, float z) {
    const float3 pt{x, y, z};
    return VertexType(pt, CalculateOrientation(pt));
  }

  int num_stacks_;
  int num_slices_;
};


template <typename T>
FilamentBuffers CreateFromBuilder(filament::Engine* engine, const T& builder) {
  using VertexType = typename T::VertexType;
  using IndexType = typename T::IndexType;

  const int num_vertices = builder.NumVertices();
  const int num_indices = builder.NumIndices();
  if (num_vertices == 0 || num_indices == 0) {
    return {};
  }

  auto vertices = [&](std::byte* buffer, std::size_t len) {
    auto* ptr = reinterpret_cast<typename T::VertexType*>(buffer);
    if (sizeof(*ptr) * num_vertices != len) {
      mju_error("Buffer size mismatch.");
    }
    builder.GenerateVertices(ptr, num_vertices);
  };

  auto indices = [&](std::byte* buffer, std::size_t len) {
    auto* ptr = reinterpret_cast<typename T::IndexType*>(buffer);
    if (sizeof(*ptr) * num_indices != len) {
      mju_error("Buffer size mismatch.");
    }
    builder.GenerateIndices(ptr, num_indices);
  };

  auto vb = CreateVertexBuffer<VertexType>(engine, num_vertices, vertices);
  auto ib = CreateIndexBuffer<IndexType>(engine, num_indices, indices);
  return {ib, vb, builder.GetBounds(), T::kPrimitiveType};
}

FilamentBuffers CreateLine(filament::Engine* engine, const mjModel* model) {
  return CreateFromBuilder(engine, LineBuilder());
}

FilamentBuffers CreatePlane(filament::Engine* engine, const mjModel* model) {
  const int num_quads = model->vis.quality.numquads;
  return CreateFromBuilder(engine, PlaneBuilder(num_quads));
}

FilamentBuffers CreateBox(filament::Engine* engine, const mjModel* model) {
  const int num_quads = model->vis.quality.numquads;
  return CreateFromBuilder(engine, BoxBuilder(num_quads));
}

FilamentBuffers CreateLineBox(filament::Engine* engine, const mjModel* model) {
  return CreateFromBuilder(engine, LineBoxBuilder());
}

FilamentBuffers CreateSphere(filament::Engine* engine, const mjModel* model) {
  const int num_stacks = model->vis.quality.numstacks;
  const int num_slices = model->vis.quality.numslices;
  return CreateFromBuilder(engine, SphereBuilder(num_stacks, num_slices));
}

FilamentBuffers CreateTube(filament::Engine* engine, const mjModel* model) {
  const int num_stacks = model->vis.quality.numstacks;
  const int num_slices = model->vis.quality.numslices;
  return CreateFromBuilder(engine, TubeBuilder(num_stacks, num_slices));
}

FilamentBuffers CreateDisk(filament::Engine* engine, const mjModel* model) {
  const int num_slices = model->vis.quality.numslices;
  return CreateFromBuilder(engine, DiskBuilder(num_slices));
}

FilamentBuffers CreateDome(filament::Engine* engine, const mjModel* model) {
  const int num_stacks = model->vis.quality.numstacks / 2;
  const int num_slices = model->vis.quality.numslices;
  return CreateFromBuilder(engine, DomeBuilder(num_stacks, num_slices));
}

FilamentBuffers CreateCone(filament::Engine* engine, const mjModel* model) {
  const int num_stacks = model->vis.quality.numstacks;
  const int num_slices = model->vis.quality.numslices;
  return CreateFromBuilder(engine, ConeBuilder(num_stacks, num_slices));
}

}  // namespace mujoco
