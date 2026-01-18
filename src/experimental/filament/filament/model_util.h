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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MODEL_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MODEL_UTIL_H_

#include <string_view>

#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

namespace mujoco {

// The types of meshes stored in the mjModel.
enum class MeshType {
  kNormal,
  kConvexHull,
  kHeightField,
};

// The types of textures stored in the mjModel.
enum class TextureType {
  kNormal2d,
  kCube,
};

// Generates a filament VertexBuffer for a given mesh in the mjModel.
filament::VertexBuffer* CreateVertexBuffer(filament::Engine* engine,
                                           const mjModel* model, int id,
                                           MeshType mesh_type);

// Generates a filament IndexBuffer for a given mesh in the mjModel.
filament::IndexBuffer* CreateIndexBuffer(filament::Engine* engine,
                                         const mjModel* model, int id,
                                         MeshType mesh_type);

// Generates a filament Texture for a given 2D texture in the mjModel.
filament::Texture* CreateTexture(filament::Engine* engine, const mjModel* model,
                                 int id, TextureType texture_type);

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

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MODEL_UTIL_H_
