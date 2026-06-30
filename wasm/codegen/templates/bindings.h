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

#ifndef MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
#define MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/em_asm.h>
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
#include <string>  // NOLINT
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
using emscripten::typed_memory_view;
using emscripten::val;
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

std::string KeyErrorMessage(const mjModel* model, int objtype, int count,
                            std::string_view name, std::string_view accessor_name);

std::string IndexErrorMessage(int index, int count,
                              std::string_view accessor_name);

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
  mjModel* model;
  std::vector<MjvLight> lights;
  std::vector<MjvGLCamera> camera;
};

struct MjVFS {
  MjVFS() : ptr_(new mjVFS) { mj_defaultVFS(ptr_); }
  ~MjVFS() { mj_deleteVFS(ptr_); }
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

}  // namespace mujoco::wasm
// NOLINTEND(whitespace/semicolon)
// NOLINTEND(whitespace/line_length)

#endif  // MUJOCO_WASM_CODEGEN_GENERATED_BINDINGS_H_
