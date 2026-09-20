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

#include "render/filament/support/filament_util.h"

#include <limits>

#include <math/TMatHelpers.h>
#include <math/TVecHelpers.h>
#include <math/mat3.h>
#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjrender.h>

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3f;
using filament::math::mat4;
using filament::math::mat4f;
using filament::math::quatf;

mat4 ToReflectionMatrix(const mat4& xform) {
  // Normalize normal so geom scaling does not collapse the reflection.
  const float3 normal = normalize(xform[2].xyz);
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

mat4f GetReflectionViewProjectionMatrix(const mjrCamera& cam, int viewport_width,
                                       int viewport_height) {
  const float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  const float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  const float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  const float3 cam_at = cam_pos + cam_fwd;

  const float bottom = cam.frustum_bottom;
  const float top = cam.frustum_top;
  const float near = cam.frustum_near;
  const float far = cam.frustum_far;
  const float aspect_ratio =
      static_cast<float>(viewport_width) / static_cast<float>(viewport_height);
  const float halfwidth =
      cam.frustum_width
          ? cam.frustum_width
          : 0.5f * aspect_ratio * (top - bottom);
  const float left = cam.frustum_center - halfwidth;
  const float right = cam.frustum_center + halfwidth;
  const mat4 view = inverse(mat4::lookAt(cam_pos, cam_at, cam_up));

  mat4 proj;
  if (cam.orthographic) {
    proj = mat4::ortho(left, right, bottom, top, near, far);
  } else {
    proj = mat4::frustum(left, right, bottom, top, near, far);
    proj[2][2] = -1.0f;
    proj[3][2] = -2.0f * near;
  }
  return mat4f(proj * view);
}

float4 CalculateOrientation(const float3& normal) {
  float3 tangent;
  float3 bitangent;
  if (normal.y < -1.0f + std::numeric_limits<float>::epsilon()) {
    // Handle the singularity.
    tangent = float3{-1.0f, 0.0f, 0.0f};
    bitangent = float3{0.0f, 0.0f, -1.0f};
  } else {
    const float a = 1.0f / (1.0f + normal.y);
    const float b = -normal.z * normal.x * a;
    tangent = float3(b, -normal.z, 1.0f - normal.z * normal.z * a);
    bitangent = float3(1.0f - normal.x * normal.x * a, -normal.x, b);
  }
  quatf orientation = mat3f::packTangentFrame({tangent, bitangent, normal});
  return float4(orientation.xyz, orientation.w);
}

float3 CalculateNormal(const filament::math::float3& p1,
                       const filament::math::float3& p2,
                       const filament::math::float3& p3) {
  const float3 v12 = p2 - p1;
  const float3 v13 = p3 - p1;
  return normalize(cross(v12, v13));
}

float4 CalculateOrientation(const filament::math::float3& p1,
                            const filament::math::float3& p2,
                            const filament::math::float3& p3) {
  return CalculateOrientation(CalculateNormal(p1, p2, p3));
}

}  // namespace mujoco
