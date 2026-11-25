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

#include "studio/sim/batched_model.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <span>  // NOLINT
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

namespace mujoco::studio {
namespace {

// Allowlist of per-sim model fields: everything the Filament renderables or
// render kinematics read, and nothing structural. This is a subset of what
// mjb_expand accepts and what the "mjModel changes" section of the programming
// guide allows to be written at runtime; the viewer only runs mj_fwdKinematics
// and mj_ray on the result, so clearing `*_sameframe` on local-frame overrides
// and refitting `geom_rbound` on `geom_size` / `geom_dataid` overrides suffices
// without running `mj_setConst`. Sizes and element types come from
// MJMODEL_POINTERS below so a renamed or retyped mjModel field fails to
// compile.
constexpr std::string_view kBatchedFields[] = {
    "qpos0",           "body_pos",       "body_quat",     "body_ipos",        "body_iquat",
    "jnt_pos",         "jnt_axis",       "geom_pos",      "geom_quat",        "geom_size",
    "geom_rgba",       "geom_group",     "geom_matid",    "geom_dataid",      "site_pos",
    "site_quat",       "site_size",      "site_rgba",     "site_group",       "site_matid",
    "cam_pos",         "cam_quat",       "cam_poscom0",   "cam_pos0",         "cam_mat0",
    "cam_fovy",        "cam_intrinsic",  "light_pos",     "light_dir",        "light_poscom0",
    "light_pos0",      "light_dir0",     "light_active",  "light_castshadow", "light_bulbradius",
    "light_intensity", "light_range",    "light_cutoff",  "light_softness",   "light_ambient",
    "light_diffuse",   "light_specular", "mat_texid",     "mat_texuniform",   "mat_texrepeat",
    "mat_emission",    "mat_specular",   "mat_shininess", "mat_reflectance",  "mat_metallic",
    "mat_roughness",   "mat_rgba",       "tendon_matid",  "tendon_group",     "tendon_width",
    "tendon_rgba",
};

constexpr bool AllBatchedFieldsInMjModel() {
  constexpr std::string_view kPointerNames[] = {
#define X(type, field, nr, nc) #field,
      MJMODEL_POINTERS
#undef X
  };
  for (std::string_view field : kBatchedFields) {
    bool found = false;
    for (std::string_view pointer : kPointerNames) {
      if (field == pointer) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}
static_assert(AllBatchedFieldsInMjModel(),
              "Every kBatchedFields entry must be an MJMODEL_POINTERS field.");

// Returns true if `ModelRenderables` reads `name` only at construction or on
// option changes and must be rebuilt when it changes per simulation.
constexpr bool IsShapeField(std::string_view name) {
  return name == "geom_size" || name == "geom_dataid" || name == "geom_group" ||
         name == "site_size" || name == "site_group" || name == "tendon_group";
}

// Returns true if `name` is a local frame that mj_kinematics skips for bodies'
// inertial frames, geoms and sites whose compiled frame is the identity (the
// `*_sameframe` shortcuts).
constexpr bool IsFrameField(std::string_view name) {
  return name == "body_ipos" || name == "body_iquat" || name == "geom_pos" || name == "geom_quat" ||
         name == "site_pos" || name == "site_quat";
}

// Returns true if `name` is in kBatchedFields. Only runs while FieldMap() is
// built, so a linear scan is fine.
constexpr bool IsAllowed(std::string_view name) {
  for (std::string_view field : kBatchedFields) {
    if (field == name) {
      return true;
    }
  }
  return false;
}

// How to read, write and size one allowlisted mjModel field.
struct FieldDesc {
  const void* (*data)(const mjModel* m);
  void* (*mutable_data)(mjModel* m);
  size_t (*byte_size)(const mjModel* m);
  bool is_shape;
  bool is_frame;
};

const std::unordered_map<std::string_view, FieldDesc>& FieldMap() {
  static const auto* kMap = [] {
    auto* map = new std::unordered_map<std::string_view, FieldDesc>();
#define X(type, field, nr, nc)                                                         \
  if (IsAllowed(#field)) {                                                             \
    map->emplace(#field, FieldDesc{                                                    \
                             [](const mjModel* m) -> const void* { return m->field; }, \
                             [](mjModel* m) -> void* { return m->field; },             \
                             [](const mjModel* m) -> size_t {                          \
                               MJMODEL_POINTERS_PREAMBLE(m)                            \
                               return sizeof(type) * m->nr * nc;                       \
                             },                                                        \
                             IsShapeField(#field),                                     \
                             IsFrameField(#field),                                     \
                         });                                                           \
  }
    MJMODEL_POINTERS
#undef X
    return map;
  }();
  return *kMap;
}

// Returns `field`'s descriptor if it is supported and sized for `model`, else
// nullptr. After this check field.sim_bytes is the mjModel array's byte size.
const FieldDesc* Resolve(const mjModel* model, const BatchedModelField& field) {
  if (field.data == nullptr) {
    return nullptr;
  }
  const auto& map = FieldMap();
  const auto it = map.find(field.name);
  if (it == map.end()) {
    return nullptr;
  }
  const size_t bytes = it->second.byte_size(model);
  return bytes != 0 && bytes == field.sim_bytes ? &it->second : nullptr;
}

// Refits `model->geom_rbound` for geoms whose bounding sphere depends on
// `geom_size` or `geom_dataid`, matching `mj_setConst` (`geomBound`) and
// `mjCGeom::GetRBound` so `mj_ray` (`ray_cull`) does not cull enlarged geoms.
void UpdateGeomRBound(mjModel* model, bool has_geom_size, bool has_geom_dataid) {
  if (!has_geom_size && !has_geom_dataid) {
    return;
  }
  for (int g = 0; g < model->ngeom; ++g) {
    const auto geom_type = static_cast<mjtGeom>(model->geom_type[g]);
    if (has_geom_size) {
      const mjtNum* size = model->geom_size + 3 * g;
      switch (geom_type) {
        case mjGEOM_PLANE:
          model->geom_rbound[g] = 0;
          break;
        case mjGEOM_SPHERE:
          model->geom_rbound[g] = size[0];
          break;
        case mjGEOM_CAPSULE:
          model->geom_rbound[g] = size[0] + size[1];
          break;
        case mjGEOM_CYLINDER:
          model->geom_rbound[g] = std::hypot(size[0], size[1]);
          break;
        case mjGEOM_ELLIPSOID:
          model->geom_rbound[g] = std::max({size[0], size[1], size[2]});
          break;
        case mjGEOM_BOX:
          model->geom_rbound[g] =
              std::sqrt(size[0] * size[0] + size[1] * size[1] + size[2] * size[2]);
          break;
        default:
          break;
      }
    }
    if (has_geom_dataid) {
      const int dataid = model->geom_dataid[g];
      if (geom_type == mjGEOM_HFIELD) {
        if (dataid >= 0 && dataid < model->nhfield) {
          const mjtNum* h = model->hfield_size + 4 * dataid;
          model->geom_rbound[g] =
              std::sqrt(h[0] * h[0] + h[1] * h[1] + std::max(h[2] * h[2], h[3] * h[3]));
        } else {
          model->geom_rbound[g] = 0;
        }
      } else if (geom_type == mjGEOM_MESH) {
        if (dataid >= 0 && dataid < model->nmesh && model->mesh_bvhadr[dataid] >= 0) {
          const mjtNum* aabb = model->bvh_aabb + 6 * model->mesh_bvhadr[dataid];
          const mjtNum hx = std::abs(aabb[0]) + aabb[3];
          const mjtNum hy = std::abs(aabb[1]) + aabb[4];
          const mjtNum hz = std::abs(aabb[2]) + aabb[5];
          model->geom_rbound[g] = std::sqrt(hx * hx + hy * hy + hz * hz);
        } else {
          model->geom_rbound[g] = 0;
        }
      }
    }
  }
}

}  // namespace

bool IsBatchedModelField(std::string_view name) { return FieldMap().contains(name); }

bool ApplyBatchedModelFields(mjModel* model, std::span<const BatchedModelField> fields, int sim,
                             std::vector<std::byte>* shape_row) {
  std::vector<std::byte> shape;
  bool has_frame_field = false;
  bool has_geom_size = false;
  bool has_geom_dataid = false;
  for (const BatchedModelField& field : fields) {
    const FieldDesc* desc = Resolve(model, field);
    if (desc == nullptr) {
      continue;
    }
    const std::byte* row = static_cast<const std::byte*>(field.data) + sim * field.sim_bytes;
    std::memcpy(desc->mutable_data(model), row, field.sim_bytes);
    if (shape_row != nullptr && desc->is_shape) {
      shape.insert(shape.end(), row, row + field.sim_bytes);
    }
    has_frame_field |= desc->is_frame;
    has_geom_size |= (field.name == "geom_size");
    has_geom_dataid |= (field.name == "geom_dataid");
  }
  // A per-sim local frame must be applied in full, so clear the compile-time
  // shortcuts that would copy the parent's pose instead. Nonzero to zero is
  // the safe direction of change for `*_sameframe`.
  if (has_frame_field) {
    std::memset(model->body_sameframe, mjSAMEFRAME_NONE, model->nbody);
    std::memset(model->geom_sameframe, mjSAMEFRAME_NONE, model->ngeom);
    std::memset(model->site_sameframe, mjSAMEFRAME_NONE, model->nsite);
  }
  UpdateGeomRBound(model, has_geom_size, has_geom_dataid);
  if (shape_row == nullptr || shape == *shape_row) {
    return false;
  }
  *shape_row = std::move(shape);
  return true;
}

void SaveBatchedModelFields(const mjModel* model, std::span<const BatchedModelField> fields,
                            std::vector<std::byte>* out) {
  out->clear();
  for (const BatchedModelField& field : fields) {
    const FieldDesc* desc = Resolve(model, field);
    if (desc == nullptr) {
      continue;
    }
    const auto* src = static_cast<const std::byte*>(desc->data(model));
    out->insert(out->end(), src, src + field.sim_bytes);
  }
}

bool RestoreBatchedModelFields(mjModel* model, std::span<const BatchedModelField> fields,
                               const std::vector<std::byte>& saved) {
  size_t expected = 0;
  for (const BatchedModelField& field : fields) {
    if (Resolve(model, field) != nullptr) {
      expected += field.sim_bytes;
    }
  }
  if (saved.size() != expected) {
    return false;
  }
  size_t offset = 0;
  bool has_geom_size = false;
  bool has_geom_dataid = false;
  for (const BatchedModelField& field : fields) {
    const FieldDesc* desc = Resolve(model, field);
    if (desc == nullptr) {
      continue;
    }
    std::memcpy(desc->mutable_data(model), saved.data() + offset, field.sim_bytes);
    offset += field.sim_bytes;
    has_geom_size |= (field.name == "geom_size");
    has_geom_dataid |= (field.name == "geom_dataid");
  }
  UpdateGeomRBound(model, has_geom_size, has_geom_dataid);
  return true;
}

}  // namespace mujoco::studio
