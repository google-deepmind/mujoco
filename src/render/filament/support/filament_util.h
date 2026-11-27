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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_FILAMENT_UTIL_H_
#define MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_FILAMENT_UTIL_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <string_view>
#include <type_traits>

#include <math/mat3.h>
#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

namespace mujoco {

// Combines the hash of *v into seed.
template <class T>
inline void Combine(uint64_t& seed, const T* v) {
  seed ^= std::hash<T>()(*v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// Hashes a struct by memory. Useful for hashing data types like mjrfMaterial
// and mjrfRenderOpts as these structs should be initialized using memset(0)
// in their default initialization functions.
template <typename T>
inline uint64_t StructHash(const T& obj) {
  static_assert(std::is_trivially_copyable_v<T>,
                "Only trivially copyable types are hashable.");
  int num_bytes = sizeof(T);
  const std::byte* ptr = reinterpret_cast<const std::byte*>(&obj);
  uint64_t seed = 0;
  while (num_bytes >= sizeof(uint64_t)) {
    uint64_t val;
    std::memcpy(&val, ptr, sizeof(val));
    Combine(seed, &val);
    ptr += sizeof(uint64_t);
    num_bytes -= sizeof(uint64_t);
  }
  while (num_bytes >= sizeof(uint32_t)) {
    uint32_t val;
    std::memcpy(&val, ptr, sizeof(val));
    Combine(seed, &val);
    ptr += sizeof(uint32_t);
    num_bytes -= sizeof(uint32_t);
  }
  while (num_bytes >= sizeof(uint16_t)) {
    uint16_t val;
    std::memcpy(&val, ptr, sizeof(val));
    Combine(seed, &val);
    ptr += sizeof(uint16_t);
    num_bytes -= sizeof(uint16_t);
  }
  while (num_bytes >= sizeof(uint8_t)) {
    uint8_t val;
    std::memcpy(&val, ptr, sizeof(val));
    Combine(seed, &val);
    ptr += sizeof(uint8_t);
    num_bytes -= sizeof(uint8_t);
  }
  return seed;
}

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
  return filament::math::mat3f(ptr[0], ptr[1], ptr[2],
                               ptr[3], ptr[4], ptr[5],
                               ptr[6], ptr[7], ptr[8]);
  // clang-format on
}

// Reads a mat4 from an array buffer in the model/scene.
template <typename T>
inline filament::math::mat4f ReadMat4(const T* arr, int index = 0) {
  // clang-format off
  const T* ptr = arr + (16 * index);
  return filament::math::mat4f(
      ptr[0],  ptr[1],  ptr[2],  ptr[3],
      ptr[4],  ptr[5],  ptr[6],  ptr[7],
      ptr[8],  ptr[9],  ptr[10], ptr[11],
      ptr[12], ptr[13], ptr[14], ptr[15]);
  // clang-format on
}

// Writes a mat4 into an array buffer in the model/scene.
template <typename T>
inline void WriteMat4(T* arr, const filament::math::mat4f& mat, int index = 0) {
  T* ptr = arr + (16 * index);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ptr[4 * i + j] = mat[i][j];
    }
  }
}

// Transforms a 3D vector from MuJoCo's Z-up coordinate frame to Filament's
// Y-up coordinate frame (equivalent to Rx(-pi/2) * v).
inline filament::math::float3 ToFilamentFrame(
    const filament::math::float3& v) {
  return filament::math::float3(v.x, v.z, -v.y);
}

// Calculates a reflection matrix for a plane defined by its transform.
filament::math::mat4 ToReflectionMatrix(const filament::math::mat4& xform);

// Returns the GL-convention `projection` with its near plane moved onto
// `plane`, given in view space with the camera on its negative side, so that
// the rasterizer clips everything behind the plane. The x, y and w rows are
// untouched. Returns `projection` unchanged if the far frustum corner is not
// in front of `plane`. Lengyel, "Oblique View Frustum Depth Projection and
// Clipping", Journal of Game Development 1(2), 2005.
filament::math::mat4 CalculateObliqueProjection(
    filament::math::mat4 projection, const filament::math::double4& plane);

// Calculates the view-projection matrix for reflection screen-space mapping.
filament::math::mat4f GetReflectionViewProjectionMatrix(
    const mjrCamera& cam, int viewport_width, int viewport_height);

// Calculates the normal of a triangle given its three vertices.
filament::math::float3 CalculateNormal(const filament::math::float3& p1,
                                       const filament::math::float3& p2,
                                       const filament::math::float3& p3);

// Calculates the orientation of a vertex given just its normal.
filament::math::float4 CalculateOrientation(
    const filament::math::float3& normal);

// Calculates the orientation of a triangle given its three vertices.
filament::math::float4 CalculateOrientation(const filament::math::float3& p1,
                                            const filament::math::float3& p2,
                                            const filament::math::float3& p3);

// Reads a value with the given name from the mjModel's data sections. The
// default_value is returned if the named element is not found. Supported types
// are: bool, enums, arithmetic types (int, float, etc.), filament math types,
// and strings (const char* or string_view).
template <typename T>
  requires(!std::is_pointer_v<T> || std::is_same_v<T, const char*>)
T ReadElement(const mjModel* model, const char* name, T default_value = T()) {
  if (!model) {
    return default_value;
  }
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
  } else if constexpr (std::is_same_v<T, bool>) {
    if (model->numeric_size[id] != 1) {
      mju_warning("Invalid numeric size for '%s': expected 1, got %d.", name,
                  model->numeric_size[id]);
      return default_value;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return static_cast<T>(*ptr != 0);
  } else if constexpr (std::is_arithmetic_v<T>) {
    if (model->numeric_size[id] != 1) {
      mju_warning("Invalid numeric size for '%s': expected 1, got %d.", name,
                  model->numeric_size[id]);
      return default_value;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return static_cast<T>(*ptr);
  } else if constexpr (std::is_enum_v<T>) {
    if (model->numeric_size[id] != 1) {
      mju_warning("Invalid numeric size for '%s': expected 1, got %d.", name,
                  model->numeric_size[id]);
      return default_value;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return static_cast<T>(static_cast<int>(*ptr));
  } else if constexpr (std::is_same_v<T, filament::math::float2>) {
    if (model->numeric_size[id] != 2) {
      mju_warning("Invalid numeric size for '%s': expected 2, got %d.", name,
                  model->numeric_size[id]);
      return default_value;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return T{ptr[0], ptr[1]};
  } else if constexpr (std::is_same_v<T, filament::math::float3>) {
    if (model->numeric_size[id] != 3) {
      mju_warning("Invalid numeric size for '%s': expected 3, got %d.", name,
                  model->numeric_size[id]);
      return default_value;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return T{ptr[0], ptr[1], ptr[2]};
  } else if constexpr (std::is_same_v<T, filament::math::float4>) {
    if (model->numeric_size[id] != 4) {
      mju_warning("Invalid numeric size for '%s': expected 4, got %d.", name,
                  model->numeric_size[id]);
      return default_value;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    return T{ptr[0], ptr[1], ptr[2], ptr[3]};
  }
  return default_value;
}

// ReadElement overload for reading C-style arrays. The array is left unchanged
// if the named element is not found.
template <typename T, int N>
void ReadElement(const mjModel* model, const char* name, T (&arr)[N]) {
  if (!model) {
    return;
  }
  const int id = mj_name2id(model, mjOBJ_NUMERIC, name);
  if (id >= 0) {
    if (model->numeric_size[id] != N) {
      mju_warning("Invalid numeric size for '%s': expected %d, got %d.", name, N,
                  model->numeric_size[id]);
      return;
    }
    const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
    for (int i = 0; i < N; ++i) {
      arr[i] = static_cast<T>(ptr[i]);
    }
  }
}

// Returns the full path to a filament asset based on its file name.
std::string ResolveFilamentAssetPath(const std::string& filename);

}  // namespace mujoco

#endif  // MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_FILAMENT_UTIL_H_
