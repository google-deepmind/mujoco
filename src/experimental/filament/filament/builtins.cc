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
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include "experimental/filament/filament_util.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

static constexpr size_t kNumVerticesPerTriangle = 3;
static constexpr size_t kNumVerticesPerQuad = 4;
static constexpr size_t kNumIndicesPerTriangle = 3;
static constexpr size_t kNumIndicesPerQuad = 6;

static void AppendQuadIndices(std::vector<uint16_t>& vec, uint16_t a,
                              uint16_t b, uint16_t c, uint16_t d) {
  vec.push_back(a);
  vec.push_back(b);
  vec.push_back(c);
  vec.push_back(a);
  vec.push_back(c);
  vec.push_back(d);
}

static std::size_t NumVerticesPerSide(int num_quads_per_axis) {
  return (num_quads_per_axis + 1) * (num_quads_per_axis + 1);
}

static std::size_t NumIndicesPerSide(int num_quads_per_axis) {
  return kNumIndicesPerQuad * num_quads_per_axis * num_quads_per_axis;
}

class BuiltinBuilder : public mjrMeshData {
 public:
  BuiltinBuilder() { mjr_defaultMeshData(this); }
  virtual ~BuiltinBuilder() = default;

  template <typename T, typename... Args>
  static std::unique_ptr<Mesh> Create(filament::Engine* engine,
                                      Args&&... args) {
    auto builder = new T(std::forward<Args>(args)...);
    mjrMeshData* mesh_data = builder->PrepareMeshData();
    mesh_data->release_callback = +[](void* user_data) {
      delete static_cast<BuiltinBuilder*>(user_data);
    };
    mesh_data->user_data = builder;
    return std::make_unique<Mesh>(engine, *mesh_data);
  }

  mjrMeshData* PrepareMeshData() {
    // Update the `mjrMeshData` fields.
    nattributes = 2;
    attributes[0].usage = mjVERTEX_ATTRIBUTE_USAGE_POSITION;
    attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT3;
    attributes[0].bytes = reinterpret_cast<const void*>(positions_.data());
    attributes[1].usage = mjVERTEX_ATTRIBUTE_USAGE_TANGENTS;
    attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT4;
    attributes[1].bytes = reinterpret_cast<const void*>(orientations_.data());
    nvertices = positions_.size();

    indices = indices_.data();
    nindices = indices_.size();
    index_type = mjINDEX_TYPE_U16;
    return this;
  }

 protected:
  void SetBounds(const float3& min, const float3& max) {
    bounds_min[0] = min.x;
    bounds_min[1] = min.y;
    bounds_min[2] = min.z;
    bounds_max[0] = max.x;
    bounds_max[1] = max.y;
    bounds_max[2] = max.z;
  }

  std::vector<float3> positions_;
  std::vector<float4> orientations_;
  std::vector<uint16_t> indices_;
};

class LineBuilder : public BuiltinBuilder {
 public:
  LineBuilder() {
    primitive_type = mjMESH_PRIMITIVE_TYPE_LINES;

    positions_.reserve(2);
    positions_.emplace_back(0, 0, 0);
    positions_.emplace_back(0, 0, 1);

    orientations_.resize(positions_.size(), {0, 0, 0, 1});

    indices_.reserve(2);
    indices_.push_back(0);
    indices_.push_back(1);

    SetBounds({0, 0, 0}, {0, 0, 1});
  }
};

class PlaneBuilder : public BuiltinBuilder {
 public:
  explicit PlaneBuilder(int num_quads_per_axis) {
    const int num_vertices = NumVerticesPerSide(num_quads_per_axis);
    positions_.reserve(num_vertices);

    const float delta = 2.0f / num_quads_per_axis;
    for (int x = 0; x <= num_quads_per_axis; ++x) {
      for (int y = 0; y <= num_quads_per_axis; ++y) {
        const float dx = delta * static_cast<float>(x);
        const float dy = delta * static_cast<float>(y);
        positions_.emplace_back(dx - 1.0f, dy - 1.0f, 0);
      }
    }

    orientations_.resize(positions_.size(), CalculateOrientation({0, 0, 1}));

    const int num_indices = NumIndicesPerSide(num_quads_per_axis);
    indices_.reserve(num_indices);
    for (int x = 0; x < num_quads_per_axis; ++x) {
      for (int y = 0; y < num_quads_per_axis; ++y) {
        const int base_idx = x * (num_quads_per_axis + 1) + y;
        const int i0 = base_idx + 0;
        const int i1 = base_idx + 1;
        const int i2 = base_idx + num_quads_per_axis + 2;
        const int i3 = base_idx + num_quads_per_axis + 1;
        AppendQuadIndices(indices_, i0, i3, i2, i1);
      }
    }

    SetBounds({-1, -1, -0.001}, {1, 1, 0.001});
  }
};

class TriangleBuilder : public BuiltinBuilder {
 public:
  TriangleBuilder() {
    positions_.reserve(3);
    positions_.emplace_back(0, 0, 0);
    positions_.emplace_back(1, 0, 0);
    positions_.emplace_back(0, 1, 0);

    orientations_.resize(positions_.size(),  CalculateOrientation({0, 0, 1}));

    indices_.reserve(3);
    indices_.emplace_back(0);
    indices_.emplace_back(1);
    indices_.emplace_back(2);

    SetBounds({-1, -1, -0.001}, {1, 1, 0.001});
  }
};

class LineBoxBuilder : public BuiltinBuilder {
 public:
  explicit LineBoxBuilder() {
    primitive_type = mjMESH_PRIMITIVE_TYPE_LINES;

    positions_.reserve(8);
    positions_.emplace_back(-1.0f, -1.0f, -1.0f);
    positions_.emplace_back( 1.0f, -1.0f, -1.0f);
    positions_.emplace_back(-1.0f,  1.0f, -1.0f);
    positions_.emplace_back( 1.0f,  1.0f, -1.0f);
    positions_.emplace_back(-1.0f, -1.0f,  1.0f);
    positions_.emplace_back( 1.0f, -1.0f,  1.0f);
    positions_.emplace_back(-1.0f,  1.0f,  1.0f);
    positions_.emplace_back( 1.0f,  1.0f,  1.0f);

    orientations_.resize(positions_.size(), {0, 0, 0, 1});

    indices_.reserve(24);
    indices_.push_back(0);
    indices_.push_back(1);
    indices_.push_back(1);
    indices_.push_back(3);
    indices_.push_back(3);
    indices_.push_back(2);
    indices_.push_back(2);
    indices_.push_back(0);
    // Top square (where z == 1).
    indices_.push_back(4);
    indices_.push_back(5);
    indices_.push_back(5);
    indices_.push_back(7);
    indices_.push_back(7);
    indices_.push_back(6);
    indices_.push_back(6);
    indices_.push_back(4);
    // Connect edges from bottom to top.
    indices_.push_back(2);
    indices_.push_back(6);
    indices_.push_back(3);
    indices_.push_back(7);
    indices_.push_back(0);
    indices_.push_back(4);
    indices_.push_back(1);
    indices_.push_back(5);

    SetBounds({-1, -1, -1}, {1, 1, 1});
  }
};

class BoxBuilder : public BuiltinBuilder {
 public:
  static constexpr int kNumSides = 6;

  explicit BoxBuilder(int num_quads_per_axis)
      : num_quads_per_axis_(num_quads_per_axis) {
    quad_size_ = 2.0f / static_cast<float>(num_quads_per_axis_);

    const int vertices_per_side = NumVerticesPerSide(num_quads_per_axis_);
    const int indices_per_side = NumIndicesPerSide(num_quads_per_axis_);
    const int num_vertices = vertices_per_side * kNumSides;
    const int num_indices = indices_per_side * kNumSides;

    positions_.reserve(num_vertices);
    orientations_.reserve(num_vertices);
    indices_.reserve(num_indices);

    GenerateVerticesForSide({0, 1, 0},
                            [](float2 pt) { return float3{pt.x, 1.0f, pt.y}; });
    GenerateVerticesForSide(
        {0, -1, 0}, [](float2 pt) { return float3{pt.x, -1.0f, pt.y}; });
    GenerateVerticesForSide({1, 0, 0},
                            [](float2 pt) { return float3{1.0f, pt.x, pt.y}; });
    GenerateVerticesForSide(
        {-1, 0, 0}, [](float2 pt) { return float3{-1.0f, pt.x, pt.y}; });
    GenerateVerticesForSide({0, 0, 1},
                            [](float2 pt) { return float3{pt.x, pt.y, 1.0f}; });
    GenerateVerticesForSide(
        {0, 0, -1}, [](float2 pt) { return float3{pt.x, pt.y, -1.0f}; });

    for (int i = 0; i < kNumSides; ++i) {
      for (int x = 0; x < num_quads_per_axis_; ++x) {
        for (int y = 0; y < num_quads_per_axis_; ++y) {
          const int base_idx =
              (i * vertices_per_side) + (x * (num_quads_per_axis_ + 1)) + y;
          const int i0 = base_idx + 0;
          const int i1 = base_idx + 1;
          const int i2 = base_idx + num_quads_per_axis_ + 2;
          const int i3 = base_idx + num_quads_per_axis_ + 1;
          if (i == 2 || i == 1 || i == 4) {
            AppendQuadIndices(indices_, i0, i3, i2, i1);
          } else {
            AppendQuadIndices(indices_, i0, i1, i2, i3);
          }
        }
      }
    }

    SetBounds({-1, -1, -1}, {1, 1, 1});
  }

 private:
  template <typename F>
  void GenerateVerticesForSide(float3 normal, const F& pt_gen) {
    float4 orientation = CalculateOrientation(normal);
    for (int x = 0; x <= num_quads_per_axis_; ++x) {
      for (int y = 0; y <= num_quads_per_axis_; ++y) {
        const float dx = -1.0f + (quad_size_ * static_cast<float>(x));
        const float dy = -1.0f + (quad_size_ * static_cast<float>(y));
        const float3 position = pt_gen({dx, dy});
        positions_.push_back(position);
        orientations_.push_back(orientation);
      }
    }
  }

  int num_quads_per_axis_;
  float quad_size_;
};

class TubeBuilder : public BuiltinBuilder {
 public:
  TubeBuilder(int num_stacks, int num_slices) {
    const int num_vertices = num_slices * (num_stacks + 1);
    positions_.reserve(num_vertices);
    orientations_.reserve(num_vertices);

    const float delta_angle = 2.f * std::numbers::pi / (float)num_slices;
    const float delta_stack = 2.f / static_cast<float>(num_stacks);
    for (int i = 0; i < num_slices; ++i) {
      const float angle = static_cast<float>(i) * delta_angle;
      const float2 pt{std::cos(angle), std::sin(angle)};
      const float4 orientation = CalculateOrientation({pt.x, pt.y, 0});
      for (int j = 0; j <= num_stacks; ++j) {
        const float z = -1.0f + (static_cast<float>(j) * delta_stack);
        positions_.emplace_back(pt.x, pt.y, z);
        orientations_.push_back(orientation);
      }
    }

    const int num_indices = kNumIndicesPerQuad * num_slices * num_stacks;
    indices_.reserve(num_indices);

    const int num_vertices_in_spine = num_stacks + 1;
    for (int i = 0; i < num_slices; ++i) {
      for (int j = 0; j < num_stacks; ++j) {
        const int base_idx = (i * num_vertices_in_spine) + j;
        const int i0 = base_idx + 0;
        const int i1 = base_idx + 1;
        const int i2 = (base_idx + num_stacks + 2) % num_vertices;
        const int i3 = (base_idx + num_stacks + 1) % num_vertices;
        AppendQuadIndices(indices_, i0, i3, i2, i1);
      }
    }

    SetBounds({-1, -1, -1}, {1, 1, 1});
  }
};

class ConeBuilder : public BuiltinBuilder {
 public:
  ConeBuilder(int num_stacks, int num_slices) {
    const int num_vertices =
        (num_slices * kNumVerticesPerTriangle) +
        ((num_stacks - 1) * num_slices * kNumVerticesPerQuad);
    positions_.reserve(num_vertices);
    orientations_.reserve(num_vertices);

    // pole: use triangles
    const float delta_angle =
        2.0 * std::numbers::pi / static_cast<float>(num_slices);
    const float delta_radius = 1.0f / static_cast<float>(num_stacks);

    for (int j = 0; j < num_slices; ++j) {
      const float angle1 = (j + 0) * delta_angle;
      const float angle2 = (j + 1) * delta_angle;

      AppendVert(angle1, delta_radius);
      AppendVert(angle2, delta_radius);

      positions_.emplace_back(0, 0, 1);
      orientations_.emplace_back(CalculateOrientation({0, 0, 1}));
    }

    // the rest: use quads
    for (int i = 1; i < num_stacks; ++i) {
      const float radius1 = delta_radius * (i + 0);
      const float radius2 = delta_radius * (i + 1);

      for (int j = 0; j < num_slices; ++j) {
        const float angle1 = (j + 0) * delta_angle;
        const float angle2 = (j + 1) * delta_angle;
        AppendVert(angle1, radius2);
        AppendVert(angle2, radius2);
        AppendVert(angle2, radius1);
        AppendVert(angle1, radius1);
      }
    }

    const int num_indices =
        (num_slices * kNumIndicesPerTriangle) +
        ((num_stacks - 1) * num_slices * kNumIndicesPerQuad);
    indices_.reserve(num_indices);
    for (int j = 0; j < num_slices * 3; ++j) {
      indices_.push_back(j);
    }

    int quad_idx = num_slices * 3;
    for (int i = 1; i < num_stacks; ++i) {
      for (int j = 0; j < num_slices; ++j) {
        const int i0 = quad_idx + 0;
        const int i1 = quad_idx + 1;
        const int i2 = quad_idx + 2;
        const int i3 = quad_idx + 3;
        quad_idx += 4;
        AppendQuadIndices(indices_, i0, i1, i2, i3);
      }
    }

    SetBounds({-1, -1, 0}, {1, 1, 1});
  }

 private:
  void AppendVert(float theta, float radius) {
    static constexpr float kNormalScale = 0.70710678118f;
    const float cz = std::cos(theta);
    const float sz = std::sin(theta);
    const float3 pt{cz * radius, sz * radius, 1.f - radius};
    const float3 n{cz * kNormalScale, sz * kNormalScale, kNormalScale};
    positions_.push_back(pt);
    orientations_.push_back(CalculateOrientation(n));
  }
};

class DiskBuilder : public BuiltinBuilder {
 public:
  explicit DiskBuilder(int num_slices) {
    const int num_vertices = num_slices + 1;
    positions_.reserve(num_vertices);
    const float delta_angle =
        2.0 * std::numbers::pi / static_cast<float>(num_slices);

    positions_.push_back({0, 0, 0});
    for (int i = 0; i < num_slices; ++i) {
      const float angle = static_cast<float>(i) * delta_angle;
      const float x = std::cos(angle);
      const float y = std::sin(angle);
      positions_.push_back({x, y, 0});
    }

    orientations_.resize(positions_.size(), CalculateOrientation({0, 0, 1}));

    const int num_indices = num_slices * kNumVerticesPerTriangle;
    indices_.reserve(num_indices);
    for (int i = 0; i < num_slices; ++i) {
      const int next = i < (num_slices - 1) ? i + 1 : 0;
      indices_.push_back(0);
      indices_.push_back(1 + i);
      indices_.push_back(1 + next);
    }

    SetBounds({-1, -1, -0.001}, {1, 1, 0.001});
  }
};

class SphereBuilder : public BuiltinBuilder {
 public:
  SphereBuilder(int num_stacks, int num_slices) {
    static constexpr uint16_t kNorthPoleIndex = 0;
    static constexpr uint16_t kSouthPoleIndex = 1;

    const int num_vertices = (num_stacks * num_slices) + 2;  // +2 for poles
    positions_.reserve(num_vertices);
    orientations_.reserve(num_vertices);

    const float lat_angle_delta =
        std::numbers::pi / static_cast<float>(num_stacks + 1);
    const float lon_angle_delta =
        2.0 * std::numbers::pi / static_cast<float>(num_slices);

    // Add the north and south poles.
    AppendVert(0, 0, 1);
    AppendVert(0, 0, -1);

    // Vertices by latitude.
    for (int lat = 0; lat < num_stacks; ++lat) {
      // +1 because we handle the north pole (which would be at a lat angle of
      // 0-degrees) explicitly.
      const float lat_angle = static_cast<float>(lat + 1) * lat_angle_delta;
      const float cos_lat_angle = std::cos(lat_angle);
      const float sin_lat_angle = std::sin(lat_angle);
      const float z = cos_lat_angle;

      for (int lon = 0; lon < num_slices; ++lon) {
        const float lon_angle = static_cast<float>(lon) * lon_angle_delta;

        const float cos_lon_angle = std::cos(lon_angle);
        const float sin_lon_angle = std::sin(lon_angle);

        const float x = sin_lat_angle * cos_lon_angle;
        const float y = sin_lat_angle * sin_lon_angle;
        AppendVert(x, y, z);
      }
    }

    const size_t num_tris_polar_cap = num_slices;
    const size_t num_quads_body = num_slices * (num_stacks - 1);
    const int num_indices = (2 * num_tris_polar_cap * kNumIndicesPerTriangle) +
                            (num_quads_body * kNumIndicesPerQuad);
    indices_.reserve(num_indices);

    // The first two vertices are the poles, so the first vertex in the first
    // row starts at index 2.
    uint16_t row_start = kSouthPoleIndex + 1;

    // North polar cap.
    for (int lon = 0; lon < num_slices; ++lon) {
      const int next = lon < (num_slices - 1) ? lon + 1 : 0;
      indices_.push_back(kNorthPoleIndex);
      indices_.push_back(row_start + lon);
      indices_.push_back(row_start + next);
    }

    // Latitudinal triangle strips.
    for (int lat = 0; lat < num_stacks - 1; lat++) {
      const uint16_t north_start = row_start;
      const uint16_t south_start = row_start + num_slices;
      for (int lon = 0; lon < num_slices; ++lon) {
        // The offset to the index that is adjacent to the current index.
        const int adjacent = lon < (num_slices - 1) ? lon + 1 : 0;

        const int i0 = (north_start + lon);
        const int i1 = (south_start + lon);
        const int i2 = (south_start + adjacent);
        const int i3 = (north_start + adjacent);
        AppendQuadIndices(indices_, i0, i1, i2, i3);
      }
      row_start += num_slices;
    }

    // South polar cap.
    for (int lon = 0; lon < num_slices; ++lon) {
      const int adjacent = lon < (num_slices - 1) ? lon + 1 : 0;
      indices_.push_back(kSouthPoleIndex);
      indices_.push_back(row_start + adjacent);
      indices_.push_back(row_start + lon);
    }

    SetBounds({-1, -1, -1}, {1, 1, 1});
  }

 private:
  void AppendVert(float x, float y, float z) {
    const float3 pt{x, y, z};
    positions_.push_back(pt);
    orientations_.push_back(CalculateOrientation(pt));
  }
};

class DomeBuilder : public BuiltinBuilder {
 public:
  DomeBuilder(int num_stacks, int num_slices) {
    static constexpr uint16_t kPoleIndex = 0;

    const int num_vertices = (num_stacks * num_slices) + 1;  // +1 for poles
    positions_.reserve(num_vertices);
    orientations_.reserve(num_vertices);

    const float lat_angle_delta =
        0.5 * std::numbers::pi / static_cast<float>(num_stacks);
    const float lon_angle_delta =
        2.0 * std::numbers::pi / static_cast<float>(num_slices);

    // Add the pole.
    AppendVert(0, 0, 1);

    // Vertices by latitude.
    for (int lat = 0; lat < num_stacks; ++lat) {
      // +1 because we handle the north pole (which would be at a lat angle of
      // 0-degrees) explicitly.
      const float lat_angle = static_cast<float>(lat + 1) * lat_angle_delta;
      const float cos_lat_angle = std::cos(lat_angle);
      const float sin_lat_angle = std::sin(lat_angle);
      const float z = cos_lat_angle;

      for (int lon = 0; lon < num_slices; ++lon) {
        const float lon_angle = static_cast<float>(lon) * lon_angle_delta;
        const float cos_lon_angle = std::cos(lon_angle);
        const float sin_lon_angle = std::sin(lon_angle);

        const float x = sin_lat_angle * cos_lon_angle;
        const float y = sin_lat_angle * sin_lon_angle;
        AppendVert(x, y, z);
      }
    }

    const size_t num_tris_polar_cap = num_slices;
    const size_t num_quads_body = num_slices * (num_stacks - 1);
    const int num_indices = (num_tris_polar_cap * kNumIndicesPerTriangle) +
                            (num_quads_body * kNumIndicesPerQuad);
    indices_.reserve(num_indices);

    // The first vertex is the poles, so the first vertex in the first row
    // starts at index 1.
    uint16_t row_start = kPoleIndex + 1;

    // North polar cap.
    for (int lon = 0; lon < num_slices; ++lon) {
      const int next = lon < (num_slices - 1) ? lon + 1 : 0;
      indices_.push_back(kPoleIndex);
      indices_.push_back(row_start + lon);
      indices_.push_back(row_start + next);
    }

    // Latitudinal quad strips.  The first "stack" was handled above, so we
    // only need to iterate over N-1 stacks.
    for (int lat = 0; lat < num_stacks - 1; lat++) {
      const int north_start = row_start;
      const int south_start = row_start + num_slices;
      for (int lon = 0; lon < num_slices; ++lon) {
        // The offset to the index that is adjacent to the current index.
        const int adjacent = lon < (num_slices - 1) ? lon + 1 : 0;

        const int i0 = (north_start + lon);
        const int i1 = (south_start + lon);
        const int i2 = (south_start + adjacent);
        const int i3 = (north_start + adjacent);
        AppendQuadIndices(indices_, i0, i1, i2, i3);
      }
      row_start += num_slices;
    }

    SetBounds({-1, -1, 0}, {1, 1, 1});
  }

 private:
  void AppendVert(float x, float y, float z) {
    const float3 pt{x, y, z};
    positions_.push_back(pt);
    orientations_.push_back(CalculateOrientation(pt));
  }
};

Builtins::Builtins(filament::Engine* engine, int nstack, int nslice, int nquad) {
  line_ = BuiltinBuilder::Create<LineBuilder>(engine);
  plane_ = BuiltinBuilder::Create<PlaneBuilder>(engine, nquad);
  triangle_ = BuiltinBuilder::Create<TriangleBuilder>(engine);
  box_ = BuiltinBuilder::Create<BoxBuilder>(engine, nquad);
  line_box_ = BuiltinBuilder::Create<LineBoxBuilder>(engine);
  sphere_ = BuiltinBuilder::Create<SphereBuilder>(engine, nstack, nslice);
  tube_ = BuiltinBuilder::Create<TubeBuilder>(engine, nstack, nslice);
  disk_ = BuiltinBuilder::Create<DiskBuilder>(engine, nslice);
  dome_ = BuiltinBuilder::Create<DomeBuilder>(engine, nstack, nslice);
  cone_ = BuiltinBuilder::Create<ConeBuilder>(engine, nstack, nslice);
}

const Mesh* Builtins::Line() { return line_.get(); }
const Mesh* Builtins::LineBox() { return line_box_.get(); }
const Mesh* Builtins::Plane() { return plane_.get(); }
const Mesh* Builtins::Triangle() { return triangle_.get(); }
const Mesh* Builtins::Box() { return box_.get(); }
const Mesh* Builtins::Sphere() { return sphere_.get(); }
const Mesh* Builtins::Cone() { return cone_.get(); }
const Mesh* Builtins::Disk() { return disk_.get(); }
const Mesh* Builtins::Dome() { return dome_.get(); }
const Mesh* Builtins::Tube() { return tube_.get(); }

}  // namespace mujoco
