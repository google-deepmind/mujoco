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

#include "experimental/filament/filament/math_util.h"

#include <math/mat4.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <math/TVecHelpers.h>

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4;

mat4 ToReflectionMatrix(const mat4& xform) {
  const float3 normal = xform[2].xyz;
  const float dist = dot(xform[3].xyz, normal);
  // clang-format off
  return mat4(
      1.0f - 2.0f * normal.x * normal.x,
      0.0f - 2.0f * normal.y * normal.x,
      0.0f - 2.0f * normal.z * normal.x,
      0.0f,
      0.0f - 2.0f * normal.x * normal.y,
      1.0f - 2.0f * normal.y * normal.y,
      0.0f - 2.0f * normal.z * normal.y,
      0.0f,
      0.0f - 2.0f * normal.x * normal.z,
      0.0f - 2.0f * normal.y * normal.z,
      1.0f - 2.0f * normal.z * normal.z,
      0.0f,
      2.0f * dist * normal.x,
      2.0f * dist * normal.y,
      2.0f * dist * normal.z,
      1.0f
  );
  // clang-format on
}

mat4 CalculateObliqueProjection(const mat4& projection, const float4& plane) {
  mat4 res = projection;
  auto sgn = [](float x) {
    return (x > 0.0f) ? 1.0f : x < 0.0f ? -1.0f : 0.0f;
  };

  // The plane should be oriented such that the side to be kept is positive.
  // The camera is at (0,0,0) in camera space. The value of the plane equation
  // at the camera origin is plane.w. The reflected scene is on the opposite
  // side of the plane from the camera. If plane.w is positive, the camera is
  // on the positive side, so the reflected scene is on the negative side.
  // We need to flip the plane in this case.
  float4 clip_plane = plane;
  if (plane.w > 0) {
    clip_plane = -plane;
  }

  float4 q;
  q.x = (sgn(clip_plane.x) + res[2][0]) / res[0][0];
  q.y = (sgn(clip_plane.y) + res[2][1]) / res[1][1];
  q.z = -1.0f;
  q.w = (1.0f + res[2][2]) / res[3][2];

  const float4 c = clip_plane * (2.0f / dot(clip_plane, q));
  res[0][2] = c.x;
  res[1][2] = c.y;
  res[2][2] = c.z;
  res[3][2] = c.w;
  return res;
}

}  // namespace mujoco
