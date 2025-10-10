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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATH_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATH_UTIL_H_

#include <math/mat3.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>

namespace mujoco {

// Reads a float2 from an array buffer in the model/scene.
template <typename T>
inline filament::math::float2 ReadFloat2(const T* arr, int index = 0) {
  const T* ptr = arr + (2 * index);
  return filament::math::float2(ptr[0], ptr[1]);
}

// Reads a float3 from an array buffer in the model/scene.
template <typename T>
inline filament::math::float3 ReadFloat3(const T* arr, int index = 0) {
  const T* ptr = arr + (3 * index);
  return filament::math::float3(ptr[0], ptr[1], ptr[2]);
}

// Reads a float4 from an array buffer in the model/scene.
template <typename T>
inline filament::math::float4 ReadFloat4(const T* arr, int index = 0) {
  const T* ptr = arr + (4 * index);
  return filament::math::float4(ptr[0], ptr[1], ptr[2], ptr[3]);
}

// Reads a mat3 from an array buffer in the model/scene.
template <typename T>
inline filament::math::mat3 ReadMat3(const T* arr, int index = 0) {
  // clang-format off
  const T* ptr = arr + (9 * index);
  return filament::math::mat3(ptr[0], ptr[3], ptr[6],
                              ptr[1], ptr[4], ptr[7],
                              ptr[2], ptr[5], ptr[8]);
  // clang-format on
}

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATH_UTIL_H_
