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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_UTIL_H_

#include <math/mat3.h>
#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

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
inline filament::math::mat3f ReadMat3(const T* arr, int index = 0) {
  // clang-format off
  const T* ptr = arr + (9 * index);
  return filament::math::mat3f(ptr[0], ptr[3], ptr[6],
                               ptr[1], ptr[4], ptr[7],
                               ptr[2], ptr[5], ptr[8]);
  // clang-format on
}

// A tuple of translation, rotation, and size.
struct Trs {
  filament::math::float3 translation{0.0f, 0.0f, 0.0f};
  filament::math::mat3f rotation;
  // Note: this is _slightly_ different than scale. For example, for capsules,
  // the size determines the length of the tube and the radius of the domes,
  // but the shape remains a capsule.
  filament::math::float3 size{1.0f, 1.0f, 1.0f};

  // Converts the TRS to a transform matrix.
  filament::math::mat4f ToTransform() const {
    return filament::math::mat4f(rotation, translation) *
           filament::math::mat4f::scaling(size);
  }
};

// Calculates a reflection matrix for a plane defined by its transform.
filament::math::mat4 ToReflectionMatrix(const filament::math::mat4& xform);

// Modifies a projection matrix so its near plane coincides with an arbitrary
// plane defined in camera space.
filament::math::mat4 CalculateObliqueProjection(
    const filament::math::mat4& projection,
    const filament::math::float4& plane);

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

// Reads a value with the given name from the mjModel's data sections. The
// default_value is returned if the named element is not found.
template <typename T>
T ReadElement(const mjModel* model, const char* name, T default_value = T()) {
  constexpr bool is_string =
      std::is_same_v<T, const char*> || std::is_same_v<T, std::string_view>;

  const int type = is_string ? mjOBJ_TEXT : mjOBJ_NUMERIC;
  const int id = mj_name2id(model, type, name);
  if (id < 0) {
    return default_value;
  }

  if constexpr (std::is_same_v<T, const char*>) {
    const char* ptr = model->text_data + model->text_adr[id];
    return ptr;
  } else if constexpr (std::is_same_v<T, std::string_view>) {
    const char* ptr = model->text_data + model->text_adr[id];
    // Do not include the null terminator in the string view.
    return std::string_view(ptr, model->text_size[id] - 1);
  } else if constexpr (std::is_arithmetic_v<T>) {
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return static_cast<T>(*ptr);
  } else if constexpr (std::is_enum_v<T>) {
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return static_cast<T>(static_cast<int>(*ptr));
  } else if constexpr (std::is_same_v<T, filament::math::float2>) {
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    if (model->numeric_size[id] != 2) mju_error("Invalid numeric size.");
    return T{ptr[0], ptr[1]};
  } else if constexpr (std::is_same_v<T, filament::math::float3>) {
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    if (model->numeric_size[id] != 3) mju_error("Invalid numeric size.");
    return T{ptr[0], ptr[1], ptr[2]};
  } else if constexpr (std::is_same_v<T, filament::math::float4>) {
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    if (model->numeric_size[id] != 4) mju_error("Invalid numeric size.");
    return T{ptr[0], ptr[1], ptr[2], ptr[3]};
  } else if constexpr (std::is_same_v<T, bool>) {
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return static_cast<T>(*ptr != 0);
  }
  return default_value;
}

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_UTIL_H_
