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

#include "experimental/filament/filament/vertex_util.h"

#include <limits>

#include <math/TVecHelpers.h>
#include <math/mat3.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3f;
using filament::math::quatf;

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

float3 CalculateNormal(
    const filament::math::float3& p1,
    const filament::math::float3& p2,
    const filament::math::float3& p3) {
  const float3 v12 = p2 - p1;
  const float3 v13 = p3 - p1;
  return normalize(cross(v12, v13));
}

float4 CalculateOrientation(
    const filament::math::float3& p1,
    const filament::math::float3& p2,
    const filament::math::float3& p3) {
  return CalculateOrientation(CalculateNormal(p1, p2, p3));
}

}  // namespace mujoco
