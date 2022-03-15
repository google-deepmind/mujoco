// Copyright 2022 DeepMind Technologies Limited
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

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "errors.h"
#include "indexers.h"
#include "mjdata_meta.h"
#include "raw.h"
#include <pybind11/pybind11.h>

namespace mujoco::python {

namespace py = ::pybind11;

namespace {
// Parses raw mjModel to create a mapping from names to array indices.
//
// Args:
//   count: Number of names in the map.
//   name_offsets: Array consisting of indices that correspond to the start of
//     each name in the `names` array.
//   names: Character arrays consisting of the concatenation of all names.
template <typename IntPtr, typename CharPtr>
NameToIDMap MakeMap(int count, IntPtr name_offsets, CharPtr names)
{
  NameToIDMap name_to_id;
  for (int index = 0; index < count; ++index) {
    const char* name = &names[name_offsets[index]];
    if (name[0] != '\0') {
      name_to_id.insert({name, index});
    }
  }
  return name_to_id;
}

// Makes an array view into an mjModel/mjData struct field at a given index.
//
// Template args:
//   MjSize: An (int M::*) specifying the number of entities of the same type
//     as the one whose array view is to be created. This is a way to indicate
//     the MuJoCo category of the entity itself, e.g. nbody indicates that the
//     field belongs to a body.
//   T: Scalar data type of the field.
//   M: Either raw::MjModel or MjDataMetadata.
//
// Args:
//   base_ptr: Pointer to the first entry in the entire field.
//   index: Entity ID at which a view is to be created.
//   shape: The shape of the array as defined by MuJoCo. If MjSize corresponds
//     to a ragged or indirect field, the shape may be prepended by additional
//     dimensions as appropriate, e.g. if MjSize is nv then the view gains an
//     additional dimension of the size len(qvel) of the particular joint.
//   m: Used for dereferencing MjSize.
//   owner: The base object whose lifetime is tied to the returned array.
template <auto MjSize, typename T, typename M>
py::array_t<T> MakeArray(T* base_ptr, int index, std::vector<int>&& shape,
                         const M& m, py::handle owner) {
  int offset;
  if (MjSize == &M::nq) {
    offset = m.jnt_qposadr[index];
    shape.insert(
        shape.begin(),
        ((index < m.njnt-1) ? m.jnt_qposadr[index+1] : m.nq) - offset);
  } else if (MjSize == &M::nv) {
    offset = m.jnt_dofadr[index];
    shape.insert(
        shape.begin(),
        ((index < m.njnt-1) ? m.jnt_dofadr[index+1] : m.nv) - offset);
  } else if (MjSize == &M::nhfielddata) {
    offset = m.hfield_adr[index];
    shape.insert(shape.begin(), m.hfield_ncol[index]);
    shape.insert(shape.begin(), m.hfield_nrow[index]);
  } else if (MjSize == &M::ntexdata) {
    offset = m.tex_adr[index];
    shape.insert(shape.begin(), m.tex_width[index]);
    shape.insert(shape.begin(), m.tex_height[index]);
  } else if (MjSize == &M::nsensordata) {
    offset = m.sensor_adr[index];
    shape.insert(shape.begin(), m.sensor_dim[index]);
  } else if (MjSize == &M::nnumericdata) {
    offset = m.numeric_adr[index];
    shape.insert(shape.begin(), m.numeric_size[index]);
  } else if (MjSize == &M::ntupledata) {
    offset = m.tuple_adr[index];
    shape.insert(shape.begin(), m.tuple_size[index]);
  } else {
    int size = 1;
    for (int s : shape) {
      size *= s;
    }
    offset = index * size;
  }
  return py::array_t<T>(std::move(shape), base_ptr + offset,
                        py::reinterpret_borrow<py::object>(owner));
}
}  // namespace

// M is either a raw::MjModel or MjDataMetadata.
template <typename M>
NameToIDMaps::NameToIDMaps(const M& m)
    : body(MakeMap(m.nbody, m.name_bodyadr, m.names)),
      jnt(MakeMap(m.njnt, m.name_jntadr, m.names)),
      geom(MakeMap(m.ngeom, m.name_geomadr, m.names)),
      site(MakeMap(m.nsite, m.name_siteadr, m.names)),
      cam(MakeMap(m.ncam, m.name_camadr, m.names)),
      light(MakeMap(m.nlight, m.name_lightadr, m.names)),
      mesh(MakeMap(m.nmesh, m.name_meshadr, m.names)),
      skin(MakeMap(m.nskin, m.name_skinadr, m.names)),
      hfield(MakeMap(m.nhfield, m.name_hfieldadr, m.names)),
      tex(MakeMap(m.ntex, m.name_texadr, m.names)),
      mat(MakeMap(m.nmat, m.name_matadr, m.names)),
      pair(MakeMap(m.npair, m.name_pairadr, m.names)),
      exclude(MakeMap(m.nexclude, m.name_excludeadr, m.names)),
      eq(MakeMap(m.neq, m.name_eqadr, m.names)),
      tendon(MakeMap(m.ntendon, m.name_tendonadr, m.names)),
      actuator(MakeMap(m.nu, m.name_actuatoradr, m.names)),
      sensor(MakeMap(m.nsensor, m.name_sensoradr, m.names)),
      numeric(MakeMap(m.nnumeric, m.name_numericadr, m.names)),
      text(MakeMap(m.ntext, m.name_textadr, m.names)),
      tuple(MakeMap(m.ntuple, m.name_tupleadr, m.names)),
      key(MakeMap(m.nkey, m.name_keyadr, m.names)) {}

MjModelIndexer::MjModelIndexer(raw::MjModel* m, py::handle owner)
    : m_(m),
      owner_(owner),
      name_to_id_(*m)
#define XGROUP(MjModelFieldGroupedViews, field, nfield, FIELD_XMACROS) \
  , field##_(m->nfield, std::nullopt)
      MJMODEL_VIEW_GROUPS
#undef XGROUP
{}

#define XGROUP(MjModelFieldGroupedViews, field, nfield, FIELD_XMACROS) \
  MjModelFieldGroupedViews& MjModelIndexer::field(int i) {             \
    if (i > field##_.size()) {                                         \
      throw py::index_error("index out of range");                     \
    }                                                                  \
    auto& indexer = field##_[i];                                       \
    if (!indexer.has_value()) {                                        \
      indexer.emplace(i, m_, owner_);                                  \
    }                                                                  \
    return *indexer;                                                   \
  }
MJMODEL_VIEW_GROUPS
#undef XGROUP

#define XGROUP(MjModelFieldGroupedViews, field, nfield, FIELD_XMACROS) \
  MjModelFieldGroupedViews& MjModelIndexer::field##_by_name(           \
      std::string_view name) {                                         \
    try {                                                              \
      return field(name_to_id_.field.at(name));                        \
    } catch (...) {                                                    \
      throw py::key_error(std::string(name));                          \
    }                                                                  \
  }
MJMODEL_VIEW_GROUPS
#undef XGROUP

MjDataIndexer::MjDataIndexer(raw::MjData* d, const MjDataMetadata* m,
                             py::handle owner)
    : d_(d),
      m_(m),
      owner_(owner),
      name_to_id_(*m)
#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS) \
  , field##_(m->nfield, std::nullopt)
      MJDATA_VIEW_GROUPS
#undef XGROUP
{}

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS) \
  MjDataGroupedViews& MjDataIndexer::field(int i) {              \
    if (i > field##_.size()) {                                   \
      throw py::index_error("index out of range");               \
    }                                                            \
    auto& indexer = field##_[i];                                 \
    if (!indexer.has_value()) {                                  \
      indexer.emplace(i, d_, m_, owner_);                        \
    }                                                            \
    return *indexer;                                             \
  }
MJDATA_VIEW_GROUPS
#undef XGROUP

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS) \
  MjDataGroupedViews& MjDataIndexer::field##_by_name(            \
      std::string_view name) {                                   \
    try {                                                        \
      return field(name_to_id_.field.at(name));                  \
    } catch (...) {                                              \
      throw py::key_error(std::string(name));                    \
    }                                                            \
  }
MJDATA_VIEW_GROUPS
#undef XGROUP

#define MAKE_SHAPE(dim)                                              \
  [n = (dim)]() -> std::vector<int> {                                \
    if constexpr (std::string_view(#dim) == std::string_view("1")) { \
      return {};                                                     \
    } else {                                                         \
      return {n};                                                    \
    }                                                                \
  }()

#undef MJ_M
#define MJ_M(n) m_->n
#define X(type, prefix, var, dim0, dim1)                            \
  py::array_t<type> XGROUP::var() {                                 \
    if (!var##_.has_value()) {                                      \
      var##_.emplace(MakeArray<&raw::MjModel::dim0>(                \
          m_->prefix##var, index_, MAKE_SHAPE(dim1), *m_, owner_)); \
    }                                                               \
    return *var##_;                                                 \
  }

#define XGROUP MjModelActuatorViews
MJMODEL_ACTUATOR
#undef XGROUP

#define XGROUP MjModelBodyViews
MJMODEL_BODY
#undef XGROUP

#define XGROUP MjModelCameraViews
MJMODEL_CAMERA
#undef XGROUP

#define XGROUP MjModelEqualityViews
MJMODEL_EQUALITY
#undef XGROUP

#define XGROUP MjModelExcludeViews
MJMODEL_EXCLUDE
#undef XGROUP

#define XGROUP MjModelGeomViews
MJMODEL_GEOM
#undef XGROUP

#define XGROUP MjModelHfieldViews
MJMODEL_HFIELD
#undef XGROUP

#define XGROUP MjModelJointViews
MJMODEL_JOINT
#undef XGROUP

#define XGROUP MjModelLightViews
MJMODEL_LIGHT
#undef XGROUP

#define XGROUP MjModelMaterialViews
MJMODEL_MATERIAL
#undef XGROUP

#define XGROUP MjModelMeshViews
MJMODEL_MESH
#undef XGROUP

#define XGROUP MjModelNumericViews
MJMODEL_NUMERIC
#undef XGROUP

#define XGROUP MjModelPairViews
MJMODEL_PAIR
#undef XGROUP

#define XGROUP MjModelSensorViews
MJMODEL_SENSOR
#undef XGROUP

#define XGROUP MjModelSiteViews
MJMODEL_SITE
#undef XGROUP

#define XGROUP MjModelSkinViews
MJMODEL_SKIN
#undef XGROUP

#define XGROUP MjModelTendonViews
MJMODEL_TENDON
#undef XGROUP

#define XGROUP MjModelTextureViews
MJMODEL_TEXTURE
#undef XGROUP

#define XGROUP MjModelTupleViews
MJMODEL_TUPLE
#undef XGROUP

#define XGROUP MjModelKeyframeViews
MJMODEL_KEYFRAME
#undef XGROUP

#undef X
#define MJ_M(n) m_->n
#define X(type, prefix, var, dim0, dim1)                            \
  py::array_t<type> XGROUP::var() {                               \
    if (!var##_.has_value()) {                                      \
      var##_.emplace(MakeArray<&MjDataMetadata::dim0>(              \
          d_->prefix##var, index_, MAKE_SHAPE(dim1), *m_, owner_)); \
    }                                                               \
    return *var##_;                                                 \
  }

#define XGROUP MjDataActuatorViews
MJDATA_ACTUATOR
#undef XGROUP

#define XGROUP MjDataBodyViews
MJDATA_BODY
#undef XGROUP

#define XGROUP MjDataCameraViews
MJDATA_CAMERA
#undef XGROUP

#define XGROUP MjDataGeomViews
MJDATA_GEOM
#undef XGROUP

#define XGROUP MjDataJointViews
MJDATA_JOINT
#undef XGROUP

#define XGROUP MjDataLightViews
MJDATA_LIGHT
#undef XGROUP

#define XGROUP MjDataSensorViews
MJDATA_SENSOR
#undef XGROUP

#define XGROUP MjDataSiteViews
MJDATA_SITE
#undef XGROUP

#define XGROUP MjDataTendonViews
MJDATA_TENDON
#undef XGROUP

#undef X
#undef MJ_M
#define MJ_M(n) n

}  // namespace mujoco::python
