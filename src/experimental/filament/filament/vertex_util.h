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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_VERTEX_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_VERTEX_UTIL_H_

#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>

namespace mujoco {

// Calculates the normal of a triangle given its three vertices.
filament::math::float3 CalculateNormal(
    const filament::math::float3& p1,
    const filament::math::float3& p2,
    const filament::math::float3& p3);

// Calculates the orientation of a vertex given just its normal.
filament::math::float4 CalculateOrientation(
    const filament::math::float3& normal);

// Calculates the orientation of a triangle given its three vertices.
filament::math::float4 CalculateOrientation(
    const filament::math::float3& p1,
    const filament::math::float3& p2,
    const filament::math::float3& p3);

// A standard vertex with no UV coordinates.
struct VertexNoUv {
  VertexNoUv() = default;
  VertexNoUv(filament::math::float3 position,
             filament::math::float4 orientation)
      : position(position), orientation(orientation) {}

  filament::math::float3 position;
  filament::math::float4 orientation;

  static constexpr bool kHasPosition = true;
  static constexpr bool kHasPosition2d = false;
  static constexpr bool kHasOrientation = true;
  static constexpr bool kHasUv = false;
  static constexpr bool kHasColor = false;
};

// A standard vertex with UV coordinates.
struct VertexWithUv {
  VertexWithUv() = default;
  VertexWithUv(filament::math::float3 position,
               filament::math::float4 orientation, filament::math::float2 uv)
      : position(position), orientation(orientation), uv(uv) {}

  filament::math::float3 position;
  filament::math::float4 orientation;
  filament::math::float2 uv;

  static constexpr bool kHasPosition = true;
  static constexpr bool kHasPosition2d = false;
  static constexpr bool kHasOrientation = true;
  static constexpr bool kHasUv = true;
  static constexpr bool kHasColor = false;
};

// A vertex for rendering GUI elements.
struct GuiVertex {
  GuiVertex() = default;
  GuiVertex(filament::math::float2 position, filament::math::float2 uv,
            filament::math::ubyte4 color)
      : position(position), uv(uv), color(color) {}

  filament::math::float2 position;
  filament::math::float2 uv;
  filament::math::ubyte4 color;

  static constexpr bool kHasPosition = false;
  static constexpr bool kHasPosition2d = true;
  static constexpr bool kHasOrientation = false;
  static constexpr bool kHasUv = true;
  static constexpr bool kHasColor = true;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_VERTEX_UTIL_H_
