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

// NOLINTBEGIN(whitespace/line_length)
// NOLINTBEGIN(whitespace/semicolon)

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // NOLINT
#include <memory>
#include <optional>  // NOLINT
#include <sstream>
#include <string>    // NOLINT
#include <string_view>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "engine/engine_util_errmem.h"
#include "wasm/unpack.h"
#include "python/mujoco/indexer_xmacro.h"

namespace mujoco::wasm {

using emscripten::enum_;
using emscripten::function;
using emscripten::val;
using emscripten::typed_memory_view;
using emscripten::return_value_policy::reference;
using emscripten::return_value_policy::take_ownership;

EMSCRIPTEN_DECLARE_VAL_TYPE(NumberOrString);
EMSCRIPTEN_DECLARE_VAL_TYPE(NumberArray);
EMSCRIPTEN_DECLARE_VAL_TYPE(String);
EMSCRIPTEN_DECLARE_VAL_TYPE(StringOrNull);

// Macro to define accessors for different MuJoCo object types within mjModel.
// Each line calls X_ACCESSOR with the following arguments:
// 1.  NAME: The object type name in uppercase (e.g., ACTUATOR).
// 2.  Name: The object type name in CamelCase (e.g., Actuator).
// 3.  OBJTYPE: The corresponding mjOBJ_* enum value (e.g., mjOBJ_ACTUATOR).
// 4.  field: The name of the array field in mjModel (e.g., actuator).
// 5.  nfield: The name of the count field in mjModel (e.g., nu).
#define MJMODEL_ACCESSORS                                              \
  X_ACCESSOR( ACTUATOR, Actuator, mjOBJ_ACTUATOR, actuator, nu       ) \
  X_ACCESSOR( BODY,     Body,     mjOBJ_BODY,     body,     nbody    ) \
  X_ACCESSOR( CAMERA,   Camera,   mjOBJ_CAMERA,   cam,      ncam     ) \
  X_ACCESSOR( EQUALITY, Equality, mjOBJ_EQUALITY, eq,       neq      ) \
  X_ACCESSOR( EXCLUDE,  Exclude,  mjOBJ_EXCLUDE,  exclude,  nexclude ) \
  X_ACCESSOR( GEOM,     Geom,     mjOBJ_GEOM,     geom,     ngeom    ) \
  X_ACCESSOR( HFIELD,   Hfield,   mjOBJ_HFIELD,   hfield,   nhfield  ) \
  X_ACCESSOR( JOINT,    Joint,    mjOBJ_JOINT,    jnt,      njnt     ) \
  X_ACCESSOR( LIGHT,    Light,    mjOBJ_LIGHT,    light,    nlight   ) \
  X_ACCESSOR( MATERIAL, Material, mjOBJ_MATERIAL, mat,      nmat     ) \
  X_ACCESSOR( MESH,     Mesh,     mjOBJ_MESH,     mesh,     nmesh    ) \
  X_ACCESSOR( NUMERIC,  Numeric,  mjOBJ_NUMERIC,  numeric,  nnumeric ) \
  X_ACCESSOR( PAIR,     Pair,     mjOBJ_PAIR,     pair,     npair    ) \
  X_ACCESSOR( SENSOR,   Sensor,   mjOBJ_SENSOR,   sensor,   nsensor  ) \
  X_ACCESSOR( SITE,     Site,     mjOBJ_SITE,     site,     nsite    ) \
  X_ACCESSOR( SKIN,     Skin,     mjOBJ_SKIN,     skin,     nskin    ) \
  X_ACCESSOR( TENDON,   Tendon,   mjOBJ_TENDON,   tendon,   ntendon  ) \
  X_ACCESSOR( TEXTURE,  Texture,  mjOBJ_TEXTURE,  tex,      ntex     ) \
  X_ACCESSOR( TUPLE,    Tuple,    mjOBJ_TUPLE,    tuple,    ntuple   ) \
  X_ACCESSOR( KEYFRAME, Keyframe, mjOBJ_KEY,      key,      nkey     )

// Macro to define accessors for different MuJoCo object types within mjData.
// Each line calls X_ACCESSOR with the following arguments:
// 1.  NAME: The object type name in uppercase (e.g., ACTUATOR).
// 2.  Name: The object type name in CamelCase (e.g., Actuator).
// 3.  OBJTYPE: The corresponding mjOBJ_* enum value (e.g., mjOBJ_ACTUATOR).
// 4.  field: The name of the array field in mjData (e.g., actuator).
// 5.  nfield: The name of the count field in mjModel (e.g., nu).
#define MJDATA_ACCESSORS                                               \
  X_ACCESSOR( ACTUATOR, Actuator, mjOBJ_ACTUATOR, actuator, nu       ) \
  X_ACCESSOR( BODY,     Body,     mjOBJ_BODY,     body,     nbody    ) \
  X_ACCESSOR( CAMERA,   Camera,   mjOBJ_CAMERA,   cam,      ncam     ) \
  X_ACCESSOR( GEOM,     Geom,     mjOBJ_GEOM,     geom,     ngeom    ) \
  X_ACCESSOR( JOINT,    Joint,    mjOBJ_JOINT,    jnt,      njnt     ) \
  X_ACCESSOR( LIGHT,    Light,    mjOBJ_LIGHT,    light,    nlight   ) \
  X_ACCESSOR( SENSOR,   Sensor,   mjOBJ_SENSOR,   sensor,   nsensor  ) \
  X_ACCESSOR( SITE,     Site,     mjOBJ_SITE,     site,     nsite    ) \
  X_ACCESSOR( TENDON,   Tendon,   mjOBJ_TENDON,   tendon,   ntendon  )

// Raises an error if the given val is null or undefined.
// A macro is used so that the error contains the name of the variable.
// TODO(matijak): Remove this when we can handle strings using UNPACK_STRING?
#define CHECK_VAL(val)                                    \
  if (val.isNull()) {                                     \
    mju_error("Invalid argument: %s is null", #val);      \
  } else if (val.isUndefined()) {                         \
    mju_error("Invalid argument: %s is undefined", #val); \
  }

void ThrowMujocoErrorToJS(const char* msg) {
  // Get a handle to the JS global Error constructor function, create a new
  // object instance and then throw the object as an exception using the
  // val::throw_() helper function.
  val(val::global("Error").new_(val("MuJoCo Error: " + std::string(msg))))
      .throw_();
}
__attribute__((constructor)) void InitMuJoCoErrorHandler() {
  mju_user_error = ThrowMujocoErrorToJS;
}

// Generates a descriptive error message for when a key lookup fails.
// The message includes the invalid name and a list of valid names of the
// specified object type currently present in the model.
//
// Arguments:
//   model: Pointer to the mjModel.
//   objtype: The mjOBJ_* enum value representing the object type.
//   count: The number of objects of the given type in the model.
//   name: The invalid name that was looked up.
//
// Returns:
//   A string containing the error message.
std::string KeyErrorMessage(const mjModel* model, int objtype, int count,
                            std::string_view name, std::string_view accessor_name) {
  std::vector<std::string> valid_names;
  valid_names.reserve(count);
  for (int i = 0; i < count; ++i) {
    const char* n = mj_id2name(model, objtype, i);
    if (n) {
      valid_names.push_back(n);
    }
  }
  std::sort(valid_names.begin(), valid_names.end());

  std::ostringstream message;
  message << "Invalid name '" << name << "' for " << accessor_name
          << ". Valid names: [";
  for (size_t i = 0; i < valid_names.size(); ++i) {
    message << "'" << valid_names[i] << "'";
    if (i < valid_names.size() - 1) {
      message << ", ";
    }
  }
  message << "]";
  return message.str();
}

std::string IndexErrorMessage(int index, int count,
                              std::string_view accessor_name) {
  std::ostringstream message;
  message << "Invalid index " << index << " for " << accessor_name
          << ". Valid indices from 0 to " << count - 1;
  return message.str();
}

template <size_t N>
val MakeValArray(const char* (&strings)[N]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    result.call<void>("push", val(strings[i]));
  }
  return result;
}

template <size_t N, size_t M>
val MakeValArray3(const char* (&strings)[N][M]) {
  val result = val::array();
  for (int i = 0; i < N; i++) {
    val inner = val::array();
    for (int j = 0; j < M; j++) {
      inner.call<void>("push", val(strings[i][j]));
    }
    result.call<void>("push", inner);
  }
  return result;
}

template <typename WrapperType, typename ArrayType, typename SizeType>
std::vector<WrapperType> InitWrapperArray(ArrayType* array, SizeType size) {
  std::vector<WrapperType> result;
  result.reserve(size);
  for (int i = 0; i < size; ++i) {
    result.emplace_back(&array[i]);
  }
  return result;
}

val get_mjDISABLESTRING() { return MakeValArray(mjDISABLESTRING); }
val get_mjENABLESTRING() { return MakeValArray(mjENABLESTRING); }
val get_mjTIMERSTRING() { return MakeValArray(mjTIMERSTRING); }
val get_mjLABELSTRING() { return MakeValArray(mjLABELSTRING); }
val get_mjFRAMESTRING() { return MakeValArray(mjFRAMESTRING); }
val get_mjVISSTRING() { return MakeValArray3(mjVISSTRING); }
val get_mjRNDSTRING() { return MakeValArray3(mjRNDSTRING); }


// {{ ANONYMOUS_STRUCT_TYPEDEFS }}

#undef MJ_M
#define MJ_M(n) model_->n
// The X macro expands to a member function within an MjModel...Accessor struct.
// This function returns an emscripten::val, typically a typed memory view,
// providing access to array data within the underlying mjModel. The size and
// offset of the memory view are determined by the arguments:
// - type: The C++ type of the array elements (e.g., mjtNum, int).
// - prefix: The prefix of the field name in mjModel (e.g., jnt_, geom_).
// - var: The name of the field being accessed (e.g., qposadr, size).
// - dim0: Used to determine special indexing logic for dynamically sized arrays
//         like joints (nq, nv), hfields, textures, numerics, and tuples.
// - dim1: The fixed dimension of the array if not dynamically sized. If "1",
//         a single element is returned. Otherwise, it's used as the stride
//         for the typed memory view.
#define X(type, prefix, var, dim0, dim1)                                                         \
  emscripten::val get_##var() const {                                                            \
    if constexpr (std::string_view(#dim0) == "nq") {                                             \
      int start = model_->jnt_qposadr[id_];                                                      \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_qposadr[id_ + 1] : model_->nq;            \
      return val(typed_memory_view(end - start, model_->prefix##var + start));                   \
    } else if constexpr (std::string_view(#dim0) == "nv") {                                      \
      int start = model_->jnt_dofadr[id_];                                                       \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_dofadr[id_ + 1] : model_->nv;             \
      return val(typed_memory_view(end - start, model_->prefix##var + start));                   \
    } else if constexpr (std::string_view(#dim0) == "nhfielddata") {                             \
      int start = model_->hfield_adr[id_];                                                       \
      int count = model_->hfield_nrow[id_] * model_->hfield_ncol[id_];                           \
      return val(typed_memory_view(count, model_->hfield_data + start));                         \
    } else if constexpr (std::string_view(#dim0) == "ntexdata") {                                \
      int start = model_->tex_adr[id_];                                                          \
      int count = model_->tex_height[id_] * model_->tex_width[id_] * model_->tex_nchannel[id_];  \
      return val(typed_memory_view(count, model_->tex_data + start));                            \
    } else if constexpr (std::string_view(#dim0) == "nnumericdata") {                            \
      int start = model_->numeric_adr[id_];                                                      \
      int count = model_->numeric_size[id_];                                                     \
      return val(typed_memory_view(count, model_->numeric_data + start));                        \
    } else if constexpr (std::string_view(#dim0) == "ntupledata") {                              \
      int start = model_->tuple_adr[id_];                                                        \
      int count = model_->tuple_size[id_];                                                       \
      return val(typed_memory_view(count, model_->prefix##var + start));                         \
    } else {                                                                                     \
      if constexpr (std::string_view(#dim1) == "1") {                                            \
        return val(model_->prefix##var[id_]);                                                    \
      } else {                                                                                   \
        return val(typed_memory_view(dim1, model_->prefix##var + id_ * dim1));                   \
      }                                                                                          \
    }                                                                                            \
  }                                                                                              \
  void set_##var(const emscripten::val& value) {                                                 \
    if constexpr (std::string_view(#dim0) == "nq") {                                             \
      int start = model_->jnt_qposadr[id_];                                                      \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_qposadr[id_ + 1] : model_->nq;            \
      val(typed_memory_view(end - start, model_->prefix##var + start))                           \
          .call<void>("set", value);                                                             \
    } else if constexpr (std::string_view(#dim0) == "nv") {                                      \
      int start = model_->jnt_dofadr[id_];                                                       \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_dofadr[id_ + 1] : model_->nv;             \
      val(typed_memory_view(end - start, model_->prefix##var + start))                           \
          .call<void>("set", value);                                                             \
    } else if constexpr (std::string_view(#dim0) == "nhfielddata") {                             \
      int start = model_->hfield_adr[id_];                                                       \
      int count = model_->hfield_nrow[id_] * model_->hfield_ncol[id_];                           \
      val(typed_memory_view(count, model_->hfield_data + start))                                 \
          .call<void>("set", value);                                                             \
    } else if constexpr (std::string_view(#dim0) == "ntexdata") {                                \
      int start = model_->tex_adr[id_];                                                          \
      int count = model_->tex_height[id_] * model_->tex_width[id_] * model_->tex_nchannel[id_];  \
      val(typed_memory_view(count, model_->tex_data + start))                                    \
          .call<void>("set", value);                                                             \
    } else if constexpr (std::string_view(#dim0) == "nnumericdata") {                            \
      int start = model_->numeric_adr[id_];                                                      \
      int count = model_->numeric_size[id_];                                                     \
      val(typed_memory_view(count, model_->numeric_data + start))                                \
          .call<void>("set", value);                                                             \
    } else if constexpr (std::string_view(#dim0) == "ntupledata") {                              \
      int start = model_->tuple_adr[id_];                                                        \
      int count = model_->tuple_size[id_];                                                       \
      val(typed_memory_view(count, model_->prefix##var + start))                                 \
          .call<void>("set", value);                                                             \
    } else {                                                                                     \
      if constexpr (std::string_view(#dim1) == "1") {                                            \
        model_->prefix##var[id_] = value.as<type>();                                             \
      } else {                                                                                   \
        val(typed_memory_view(dim1, model_->prefix##var + id_ * dim1))                           \
            .call<void>("set", value);                                                           \
      }                                                                                          \
    }                                                                                            \
  }

// Expands to a struct definition for each object type in MJMODEL_ACCESSORS.
// Each struct, named `MjModel{Name}Accessor`, provides:
// - A constructor taking an `mjModel*` and an integer `id`.
// - An `id()` method to get the object's index.
// - A `name()` method to get the object's name using `mj_id2name`.
// - Member functions generated by the `MJMODEL_##NAME` macro, which in turn
//   uses the `X` macro to define accessors for fields within `mjModel`.
#define X_ACCESSOR(NAME, Name, OBJTYPE, field, nfield)                           \
  struct MjModel##Name##Accessor {                                               \
    MjModel##Name##Accessor(mjModel* model, int id) : model_(model), id_(id) {}  \
                                                                                 \
    int id() const { return id_; }                                               \
    std::string name() const {                                                   \
      const char* name = mj_id2name(model_, OBJTYPE, id_);                       \
      return name ? name : "";                                                   \
    }                                                                            \
                                                                                 \
    MJMODEL_##NAME                                                               \
                                                                                 \
   private:                                                                      \
    mjModel* model_;                                                             \
    int id_;                                                                     \
  };
MJMODEL_ACCESSORS
#undef X_ACCESSOR

// Expands to a struct definition for each object type in MJDATA_ACCESSORS.
// Each struct, named `MjData{Name}Accessor`, provides:
// - A constructor taking an `mjData*`, an `mjModel*`, and an integer `id`.
// - An `id()` method to get the object's index.
// - A `name()` method to get the object's name using `mj_id2name`.
// - Member functions generated by the `MJDATA_##NAME` macro, which in turn
//   uses the `X` macro to define accessors for fields within `mjData`.
#undef MJ_M
#define MJ_M(n) model_->n
#undef X
#define X(type, prefix, var, dim0, dim1)                                               \
  emscripten::val get_##var() const {                                                  \
    if constexpr (std::string_view(#dim0) == "nq") {                                   \
      int start = model_->jnt_qposadr[id_];                                            \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_qposadr[id_ + 1] : model_->nq;  \
      return val(typed_memory_view(end - start, data_->prefix##var + start));          \
    } else if constexpr (std::string_view(#dim0) == "nv") {                            \
      int start = model_->jnt_dofadr[id_];                                             \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_dofadr[id_ + 1] : model_->nv;   \
      if constexpr (std::string_view(#dim1) == "1") {                                  \
        return val(typed_memory_view(end - start, data_->prefix##var + start));        \
      } else {                                                                         \
        return val(typed_memory_view(                                                  \
            (end - start) * dim1, data_->prefix##var + start * dim1));                 \
      }                                                                                \
    } else if constexpr (std::string_view(#dim0) == "nsensordata") {                   \
      int start = model_->sensor_adr[id_];                                             \
      int count = model_->sensor_dim[id_];                                             \
      return val(typed_memory_view(count, data_->sensordata + start));                 \
    } else {                                                                           \
      if constexpr (std::string_view(#dim1) == "1") {                                  \
        return val(data_->prefix##var[id_]);                                           \
      } else {                                                                         \
        return val(typed_memory_view(dim1, data_->prefix##var + id_ * dim1));          \
      }                                                                                \
    }                                                                                  \
  }                                                                                    \
  void set_##var(const emscripten::val& value) {                                       \
    if constexpr (std::string_view(#dim0) == "nq") {                                   \
      int start = model_->jnt_qposadr[id_];                                            \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_qposadr[id_ + 1] : model_->nq;  \
      val(typed_memory_view(end - start, data_->prefix##var + start))                  \
          .call<void>("set", value);                                                   \
    } else if constexpr (std::string_view(#dim0) == "nv") {                            \
      int start = model_->jnt_dofadr[id_];                                             \
      int end = (id_ < model_->njnt - 1) ? model_->jnt_dofadr[id_ + 1] : model_->nv;   \
      if constexpr (std::string_view(#dim1) == "1") {                                  \
        val(typed_memory_view(end - start, data_->prefix##var + start))                \
            .call<void>("set", value);                                                 \
      } else {                                                                         \
        val(typed_memory_view((end - start) * dim1,                                    \
                                          data_->prefix##var + start * dim1))          \
            .call<void>("set", value);                                                 \
      }                                                                                \
    } else if constexpr (std::string_view(#dim0) == "nsensordata") {                   \
      int start = model_->sensor_adr[id_];                                             \
      int count = model_->sensor_dim[id_];                                             \
      val(typed_memory_view(count, data_->sensordata + start))                         \
          .call<void>("set", value);                                                   \
    } else {                                                                           \
      if constexpr (std::string_view(#dim1) == "1") {                                  \
        data_->prefix##var[id_] = value.as<type>();                                    \
      } else {                                                                         \
        val(typed_memory_view(dim1, data_->prefix##var + id_ * dim1))                  \
            .call<void>("set", value);                                                 \
      }                                                                                \
    }                                                                                  \
  }

#define X_ACCESSOR(NAME, Name, OBJTYPE, field, nfield)                                                     \
  struct MjData##Name##Accessor {                                                                          \
    MjData##Name##Accessor(mjData* data, mjModel* model, int id) : data_(data), model_(model), id_(id) {}  \
                                                                                                           \
    int id() const { return id_; }                                                                         \
    std::string name() const {                                                                             \
      const char* name = mj_id2name(model_, OBJTYPE, id_);                                                 \
      return name ? name : "";                                                                             \
    }                                                                                                      \
                                                                                                           \
    MJDATA_##NAME                                                                                          \
                                                                                                           \
   private:                                                                                                \
    mjData* data_;                                                                                         \
    mjModel* model_;                                                                                       \
    int id_;                                                                                               \
  };
MJDATA_ACCESSORS
#undef X_ACCESSOR

#undef X
#undef MJ_M

// {{ STRUCTS_HEADER }}

// {{ STRUCTS_SOURCE }}

struct MjvScene {
  MjvScene();
  MjvScene(MjModel *m, int maxgeom);
  ~MjvScene();
  std::unique_ptr<MjvScene> copy();
  int GetSumFlexFaces() const;

  mjvScene* get() const;
  void set(mjvScene* ptr);

  std::vector<MjvGeom> geoms() const;

  emscripten::val geomorder() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->ngeom, ptr_->geomorder));
  }
  emscripten::val flexedgeadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexedgeadr));
  }
  emscripten::val flexedgenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexedgenum));
  }
  emscripten::val flexvertadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexvertadr));
  }
  emscripten::val flexvertnum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexvertnum));
  }
  emscripten::val flexfaceadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfaceadr));
  }
  emscripten::val flexfacenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfacenum));
  }
  emscripten::val flexfaceused() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nflex, ptr_->flexfaceused));
  }
  emscripten::val flexedge() const {
    return emscripten::val(
        emscripten::typed_memory_view(2 * model->nflexedge, ptr_->flexedge));
  }
  emscripten::val flexvert() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nflexvert, ptr_->flexvert));
  }
  emscripten::val skinfacenum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinfacenum));
  }
  emscripten::val skinvertadr() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinvertadr));
  }
  emscripten::val skinvertnum() const {
    return emscripten::val(
        emscripten::typed_memory_view(ptr_->nskin, ptr_->skinvertnum));
  }
  emscripten::val skinvert() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nskinvert, ptr_->skinvert));
  }
  emscripten::val skinnormal() const {
    return emscripten::val(
        emscripten::typed_memory_view(3 * model->nskinvert, ptr_->skinnormal));
  }
  emscripten::val flexface() const {
    return emscripten::val(emscripten::typed_memory_view(
        9 * MjvScene::GetSumFlexFaces(), ptr_->flexface));
  }
  emscripten::val flexnormal() const {
    return emscripten::val(emscripten::typed_memory_view(
        9 * MjvScene::GetSumFlexFaces(), ptr_->flexnormal));
  }
  emscripten::val flextexcoord() const {
    return emscripten::val(emscripten::typed_memory_view(
        6 * MjvScene::GetSumFlexFaces(), ptr_->flextexcoord));
  }
  // INSERT-GENERATED-MjvScene-DECLARATION

 private:
  mjvScene* ptr_;
  bool owned_ = false;

 public:
  mjModel *model;
  std::vector<MjvLight> lights;
  std::vector<MjvGLCamera> camera;
};

struct MjVFS {
  MjVFS() : ptr_(new mjVFS) { mj_defaultVFS(ptr_); }
  ~MjVFS() {
    mj_deleteVFS(ptr_);
  }
  void AddBuffer(const std::string& name, const emscripten::val& buffer) {
    std::vector<uint8_t> vec = emscripten::vecFromJSArray<uint8_t>(buffer);
    int result = mj_addBufferVFS(ptr_, name.c_str(), vec.data(), vec.size());
    if (result != 0) {
      mju_error("Could not add buffer to VFS: %d", result);
    }
  }
  void DeleteFile(const std::string& filename) {
    mj_deleteFileVFS(ptr_, filename.c_str());
  }
  mjVFS* get() const { return ptr_; }

 private:
  mjVFS* ptr_;
};

MjModel::MjModel(mjModel* ptr)
    : ptr_(ptr), opt(&ptr->opt), vis(&ptr->vis), stat(&ptr->stat) {}

MjModel::MjModel(const MjModel& other)
    : ptr_(mj_copyModel(nullptr, other.get())),
      opt(&ptr_->opt),
      vis(&ptr_->vis),
      stat(&ptr_->stat) {}

MjModel::~MjModel() {
  if (ptr_) {
    mj_deleteModel(ptr_);
  }
}

mjModel* MjModel::get() const { return ptr_; }

void MjModel::set(mjModel* ptr) { ptr_ = ptr; }

MjData::MjData(MjModel* m) {
  model = m->get();
  ptr_ = mj_makeData(model);
  if (ptr_) {
    solver =
        InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}

MjData::MjData(const MjModel& model, const MjData& other)
    : ptr_(mj_copyData(nullptr, model.get(), other.get())), model(model.get()) {
  if (ptr_) {
    solver =
        InitWrapperArray<MjSolverStat>(get()->solver, mjNSOLVER * mjNISLAND);
    timer = InitWrapperArray<MjTimerStat>(get()->timer, mjNTIMER);
    warning = InitWrapperArray<MjWarningStat>(get()->warning, mjNWARNING);
  }
}

MjData::~MjData() {
  if (ptr_) {
    mj_deleteData(ptr_);
  }
}
mjData* MjData::get() const { return ptr_; }

void MjData::set(mjData* ptr) { ptr_ = ptr; }

std::vector<MjContact> MjData::contact() const {
  return InitWrapperArray<MjContact>(get()->contact, get()->ncon);
}

MjvScene::MjvScene() {
  owned_ = true;
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(nullptr, ptr_, 0);
  lights = InitWrapperArray<MjvLight>(ptr_->lights, mjMAXLIGHT);
  camera = InitWrapperArray<MjvGLCamera>(ptr_->camera, 2);
};

MjvScene::MjvScene(MjModel* m, int maxgeom) {
  owned_ = true;
  model = m->get();
  ptr_ = new mjvScene;
  mjv_defaultScene(ptr_);
  mjv_makeScene(model, ptr_, maxgeom);
  lights = InitWrapperArray<MjvLight>(ptr_->lights, mjMAXLIGHT);
  camera = InitWrapperArray<MjvGLCamera>(ptr_->camera, 2);
};

MjvScene::~MjvScene() {
  if (owned_ && ptr_) {
    mjv_freeScene(ptr_);
    delete ptr_;
  }
}

mjvScene* MjvScene::get() const { return ptr_; }
void MjvScene::set(mjvScene* ptr) { ptr_ = ptr; }

// Taken from the python mujoco bindings code for MjvScene Wrapper
int MjvScene::GetSumFlexFaces() const {
  int nflexface = 0;
  int flexfacenum = 0;
  for (int f = 0; f < model->nflex; f++) {
    if (model->flex_dim[f] == 0) {
      // 1D : 0
      flexfacenum = 0;
    } else if (model->flex_dim[f] == 2) {
      // 2D: 2*fragments + 2*elements
      flexfacenum = 2 * model->flex_shellnum[f] + 2 * model->flex_elemnum[f];
    } else {
      // 3D: max(fragments, 4*maxlayer)
      // find number of elements in biggest layer
      int maxlayer = 0, layer = 0, nlayer = 1;
      while (nlayer) {
        nlayer = 0;
        for (int e = 0; e < model->flex_elemnum[f]; e++) {
          if (model->flex_elemlayer[model->flex_elemadr[f] + e] == layer) {
            nlayer++;
          }
        }
        maxlayer = mjMAX(maxlayer, nlayer);
        layer++;
      }
      flexfacenum = mjMAX(model->flex_shellnum[f], 4 * maxlayer);
    }

    // accumulate over flexes
    nflexface += flexfacenum;
  }
  return nflexface;
}

std::vector<MjvGeom> MjvScene::geoms() const {
  return InitWrapperArray<MjvGeom>(ptr_->geoms, ptr_->ngeom);
}

MjSpec::MjSpec()
    : ptr_(mj_makeSpec()),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {
  owned_ = true;
  mjs_defaultSpec(ptr_);
};

MjSpec::MjSpec(mjSpec *ptr)
    : ptr_(ptr),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {}

MjSpec::MjSpec(const MjSpec &other)
    : ptr_(mj_copySpec(other.get())),
      element(ptr_->element),
      compiler(&ptr_->compiler),
      option(&ptr_->option),
      visual(&ptr_->visual),
      stat(&ptr_->stat) {
  owned_ = true;
}

MjSpec& MjSpec::operator=(const MjSpec &other) {
  if (this == &other) {
    return *this;
  }
  if (owned_ && ptr_) {
    mj_deleteSpec(ptr_);
  }
  ptr_ = mj_copySpec(other.get());
  owned_ = true;
  option.set(&ptr_->option);
  visual.set(&ptr_->visual);
  stat.set(&ptr_->stat);
  compiler.set(&ptr_->compiler);
  element.set(ptr_->element);
  return *this;
}

MjSpec::~MjSpec() {
  if (ptr_ && owned_) {
    mj_deleteSpec(ptr_);
  }
}

mjSpec *MjSpec::get() const { return ptr_; }
void MjSpec::set(mjSpec *ptr) { ptr_ = ptr; }

std::unique_ptr<MjModel> mj_loadXML_wrapper(std::string filename) {
  char error[1000];
  mjModel *model = mj_loadXML(filename.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    mju_error("Loading error: %s\n", error);
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

void mj_saveModel_wrapper(const MjModel& m, const StringOrNull& filename, const val& buffer) {
  UNPACK_NULLABLE_STRING(filename);
  UNPACK_NULLABLE_VALUE(uint8_t, buffer);
  mj_saveModel(m.get(), filename_.data(), buffer_.data(), static_cast<int>(buffer_.size()));
}

std::unique_ptr<MjModel> mj_loadModel_wrapper(std::string filename, const MjVFS& vfs) {
  mjModel *model = mj_loadModel(filename.c_str(), vfs.get());
  if (!model) {
    mju_error("Failed to load from mjb");
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjSpec> parseXMLString_wrapper(const std::string &xml) {
  char error[1000];
  mjSpec *ptr = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  if (!ptr) {
    mju_error("Could not create Spec from XML string: %s\n", error);
  }
  return std::unique_ptr<MjSpec>(new MjSpec(ptr));
}

std::unique_ptr<MjModel> mj_compile_wrapper_1(const MjSpec& spec) {
  mjSpec* spec_ptr = spec.get();
  mjModel* model = mj_compile(spec_ptr, nullptr);
  if (!model || mjs_isWarning(spec_ptr)) {
    mju_error("%s", mjs_getError(spec_ptr));
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

std::unique_ptr<MjModel> mj_compile_wrapper_2(const MjSpec& spec, const MjVFS& vfs) {
  mjSpec* spec_ptr = spec.get();
  mjVFS* vfs_ptr = vfs.get();
  mjModel* model = mj_compile(spec_ptr, vfs_ptr);
  if (!model || mjs_isWarning(spec_ptr)) {
    mju_error("%s", mjs_getError(spec_ptr));
  }
  return std::unique_ptr<MjModel>(new MjModel(model));
}

void error_wrapper(const String& msg) { mju_error("%s\n", msg.as<const std::string>().data()); }

int mj_saveLastXML_wrapper(const String& filename, const MjModel& m) {
  CHECK_VAL(filename);
  std::array<char, 1024> error;
  int result = mj_saveLastXML(filename.as<const std::string>().data(), m.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

int mj_setLengthRange_wrapper(const MjModel& m, const MjData& d, int index, const MjLROpt& opt) {
  std::array<char, 1024> error;
  int result = mj_setLengthRange(m.get(), d.get(), index, opt.get(), error.data(), error.size());
  if (!result) {
    mju_error("%s", error.data());
  }
  return result;
}

// {{ WRAPPER_FUNCTIONS }}

EMSCRIPTEN_BINDINGS(mujoco_bindings) {
  // {{ ENUM_BINDINGS }}

  // Bindings for the MjModel accessor classes.
  #define X(type, prefix, var, dim0, dim1) .property(#var, &Accessor::get_##var, &Accessor::set_##var)
  #define X_ACCESSOR(NAME, Name, OBJTYPE, field, nfield)        \
    {                                                           \
      using Accessor = MjModel##Name##Accessor;                 \
      emscripten::class_<Accessor>("MjModel" #Name "Accessor")  \
          .property("id", &Accessor::id)                        \
          .property("name", &Accessor::name)                    \
          MJMODEL_##NAME;                                       \
    }
    MJMODEL_ACCESSORS
  #undef X
  #undef X_ACCESSOR

  // Bindings for the MjData accessor classes.
  #define X(type, prefix, var, dim0, dim1) .property(#var, &Accessor::get_##var, &Accessor::set_##var)
  #define X_ACCESSOR(NAME, Name, OBJTYPE, field, nfield)       \
    {                                                          \
      using Accessor = MjData##Name##Accessor;                 \
      emscripten::class_<Accessor>("MjData" #Name "Accessor")  \
          .property("id", &Accessor::id)                       \
          .property("name", &Accessor::name)                   \
          MJDATA_##NAME;                                       \
    }
    MJDATA_ACCESSORS
  #undef X
  #undef X_ACCESSOR

  // {{ STRUCTS_BINDINGS }}

  emscripten::class_<MjVFS>("MjVFS")
      .constructor<>()
      .function("addBuffer", &MjVFS::AddBuffer)
      .function("deleteFile", &MjVFS::DeleteFile);

  // {{ FUNCTION_BINDINGS }}
  function("parseXMLString", &parseXMLString_wrapper, take_ownership());
  function("error", &error_wrapper);
  function("mj_saveModel", &mj_saveModel_wrapper);
  function("mj_saveLastXML", &mj_saveLastXML_wrapper);
  function("mj_setLengthRange", &mj_setLengthRange_wrapper);
  // mj_compile is bound using two overloads to handle the optional MjVFS argument,
  // as using std::optional<MjVFS> caused memory errors due to missing copy/move constructors.
  function("mj_compile", emscripten::select_overload<std::unique_ptr<MjModel>(const MjSpec&)>(&mj_compile_wrapper_1));
  function("mj_compile", emscripten::select_overload<std::unique_ptr<MjModel>(const MjSpec&, const MjVFS&)>(&mj_compile_wrapper_2));

  emscripten::class_<WasmBuffer<float>>("FloatBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<float>::FromArray)
      .function("GetPointer", &WasmBuffer<float>::GetPointer)
      .function("GetElementCount", &WasmBuffer<float>::GetElementCount)
      .function("GetView", &WasmBuffer<float>::GetView);

  emscripten::class_<WasmBuffer<double>>("DoubleBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<double>::FromArray)
      .function("GetPointer", &WasmBuffer<double>::GetPointer)
      .function("GetElementCount", &WasmBuffer<double>::GetElementCount)
      .function("GetView", &WasmBuffer<double>::GetView);

  emscripten::class_<WasmBuffer<int>>("IntBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<int>::FromArray)
      .function("GetPointer", &WasmBuffer<int>::GetPointer)
      .function("GetElementCount", &WasmBuffer<int>::GetElementCount)
      .function("GetView", &WasmBuffer<int>::GetView);

  emscripten::class_<WasmBuffer<uint8_t>>("Uint8Buffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<uint8_t>::FromArray)
      .function("GetPointer", &WasmBuffer<uint8_t>::GetPointer)
      .function("GetElementCount", &WasmBuffer<uint8_t>::GetElementCount)
      .function("GetView", &WasmBuffer<uint8_t>::GetView);

  emscripten::register_vector<std::string>("mjStringVec");
  emscripten::register_vector<int>("mjIntVec");
  emscripten::register_vector<mjIntVec>("mjIntVecVec");
  emscripten::register_vector<float>("mjFloatVec");
  emscripten::register_vector<mjFloatVec>("mjFloatVecVec");
  emscripten::register_vector<double>("mjDoubleVec");
  emscripten::register_vector<uint8_t>("mjByteVec");
  emscripten::register_vector<MjSolverStat>("MjSolverStatVec");
  emscripten::register_vector<MjTimerStat>("MjTimerStatVec");
  emscripten::register_vector<MjWarningStat>("MjWarningStatVec");
  emscripten::register_vector<MjContact>("MjContactVec");
  emscripten::register_vector<MjvLight>("MjvLightVec");
  emscripten::register_vector<MjvGLCamera>("MjvGLCameraVec");
  emscripten::register_vector<MjvGeom>("MjvGeomVec");

  // register_type() improves type information (val is mapped to any by default)
  // NumberOrString is used in functions returning accessors, allowing users to
  // get an accessor by name (string) or id (number).
  emscripten::register_type<NumberOrString>("number|string");
  emscripten::register_type<NumberArray>("number[]");
  emscripten::register_type<String>("string");
  emscripten::register_type<StringOrNull>("string|null");

  emscripten::constant("mjMAXCONPAIR", mjMAXCONPAIR);
  emscripten::constant("mjMAXIMP", mjMAXIMP);
  emscripten::constant("mjMAXLIGHT", mjMAXLIGHT);
  emscripten::constant("mjMAXLINE", mjMAXLINE);
  emscripten::constant("mjMAXLINEPNT", mjMAXLINEPNT);
  emscripten::constant("mjMAXOVERLAY", mjMAXOVERLAY);
  emscripten::constant("mjMAXPLANEGRID", mjMAXPLANEGRID);
  emscripten::constant("mjMAXVAL", mjMAXVAL);
  emscripten::constant("mjMINIMP", mjMINIMP);
  emscripten::constant("mjMINMU", mjMINMU);
  emscripten::constant("mjMINVAL", mjMINVAL);
  emscripten::constant("mjNBIAS", mjNBIAS);
  emscripten::constant("mjNDYN", mjNDYN);
  emscripten::constant("mjNEQDATA", mjNEQDATA);
  emscripten::constant("mjNGAIN", mjNGAIN);
  emscripten::constant("mjNGROUP", mjNGROUP);
  emscripten::constant("mjNIMP", mjNIMP);
  emscripten::constant("mjNREF", mjNREF);
  emscripten::constant("mjNSOLVER", mjNSOLVER);
  emscripten::constant("mjPI", mjPI);
  emscripten::constant("mjVERSION_HEADER", mjVERSION_HEADER);

  // These complex constants are bound using function() rather than constant()
  emscripten::function("get_mjDISABLESTRING", &get_mjDISABLESTRING);
  emscripten::function("get_mjENABLESTRING", &get_mjENABLESTRING);
  emscripten::function("get_mjFRAMESTRING", &get_mjFRAMESTRING);
  emscripten::function("get_mjLABELSTRING", &get_mjLABELSTRING);
  emscripten::function("get_mjRNDSTRING", &get_mjRNDSTRING);
  emscripten::function("get_mjTIMERSTRING", &get_mjTIMERSTRING);
  emscripten::function("get_mjVISSTRING", &get_mjVISSTRING);
}

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)
