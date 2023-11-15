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

#ifndef MUJOCO_PYTHON_INDEXERS_H_
#define MUJOCO_PYTHON_INDEXERS_H_

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <absl/container/flat_hash_map.h>
#include <mujoco/mjxmacro.h>
#include "indexer_xmacro.h"
#include "raw.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace mujoco::python {
using NameToID = absl::flat_hash_map<std::string, int>;
using IDToName = std::vector<std::string>;

struct NameToIDMappings {
  explicit NameToIDMappings(const raw::MjModel& m);

  NameToID body;
  NameToID jnt;
  NameToID geom;
  NameToID site;
  NameToID cam;
  NameToID light;
  NameToID mesh;
  NameToID skin;
  NameToID hfield;
  NameToID tex;
  NameToID mat;
  NameToID pair;
  NameToID exclude;
  NameToID eq;
  NameToID tendon;
  NameToID actuator;
  NameToID sensor;
  NameToID numeric;
  NameToID text;
  NameToID tuple;
  NameToID key;
};

struct IDToNameMappings {
  explicit IDToNameMappings(const raw::MjModel& m);

  IDToName body;
  IDToName jnt;
  IDToName geom;
  IDToName site;
  IDToName cam;
  IDToName light;
  IDToName mesh;
  IDToName skin;
  IDToName hfield;
  IDToName tex;
  IDToName mat;
  IDToName pair;
  IDToName exclude;
  IDToName eq;
  IDToName tendon;
  IDToName actuator;
  IDToName sensor;
  IDToName numeric;
  IDToName text;
  IDToName tuple;
  IDToName key;
};


// Base class for a collection of NumPy views into mjModel fields associated
// with the same "entity" (for example, fields corresponding to a particular
// geom, or a particular joint).
class MjModelGroupedViewsBase {
 public:
  MjModelGroupedViewsBase(int index, std::string_view name, raw::MjModel* m,
                          pybind11::handle owner)
      : index_(index), name_(name), m_(m), owner_(owner) {}

  int index() const { return index_; }
  const std::string& name() const { return name_; }
  raw::MjModel* ptr() { return m_; }

 protected:
  int index_;
  std::string name_;
  raw::MjModel* m_;
  pybind11::handle owner_;
};

#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS) \
  class MjModelGroupedViews : public MjModelGroupedViewsBase {    \
   public:                                                        \
    using MjModelGroupedViewsBase::MjModelGroupedViewsBase;       \
    FIELD_XMACROS                                                 \
  };

// Lazily instantiate a NumPy array view object when accessed from Python, but
// cache it once made so that we can return the same one if requested again.
#define X(type, prefix, var, dim0, dim1)         \
  pybind11::array_t<type> var();                 \
  std::optional<pybind11::array_t<type>> var##_;

MJMODEL_VIEW_GROUPS

#undef XGROUP
#undef X

// A class for accessing fields corresponding to a single mjModel "entity"
// (e.g. a particular geom or joint) either by name or by ID.
class MjModelIndexer {
 public:
  MjModelIndexer(raw::MjModel* m, pybind11::handle owner);

#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS) \
  MjModelGroupedViews& field(int i);                              \
  MjModelGroupedViews& field##_by_name(std::string_view name);

  MJMODEL_VIEW_GROUPS
#undef XGROUP

 private:
  raw::MjModel* m_;
  pybind11::handle owner_;
  NameToIDMappings name_to_id_;
  IDToNameMappings id_to_name_;

  // Lazily instantiate a grouped views object when accessed from Python, but
  // cache it once made so that we can return the same one if requested again.
#define XGROUP(MjModelGroupedViews, field, nfield, FIELD_XMACROS) \
  std::vector<std::optional<MjModelGroupedViews>> field##_;

  MJMODEL_VIEW_GROUPS
#undef XGROUP
};

// Base class for a collection of NumPy views into mjData fields associated
// with the same "entity" (for example, fields corresponding to a particular
// geom, or a particular joint).
class MjDataGroupedViewsBase {
 public:
  MjDataGroupedViewsBase(int index, std::string_view name, raw::MjData* d,
                         const raw::MjModel* m,
                         pybind11::handle owner)
      : index_(index), name_(name), d_(d), m_(m), owner_(owner) {}

  int index() const { return index_; }
  const std::string& name() const { return name_; }
  raw::MjData* ptr() { return d_; }

 protected:
  int index_;
  std::string name_;
  raw::MjData* d_;
  const raw::MjModel* m_;
  pybind11::handle owner_;
};

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS) \
  class MjDataGroupedViews : public MjDataGroupedViewsBase {     \
   public:                                                       \
    using MjDataGroupedViewsBase::MjDataGroupedViewsBase;        \
    FIELD_XMACROS                                                \
  };

// Lazily instantiate a NumPy array view object when accessed from Python, but
// cache it once made so that we can return the same one if requested again.
#define X(type, prefix, var, dim0, dim1)         \
  pybind11::array_t<type> var();                 \
  std::optional<pybind11::array_t<type>> var##_;

MJDATA_VIEW_GROUPS

#undef XGROUP
#undef X

// A class for accessing fields corresponding to a single mjModel "entity"
// (e.g. a particular geom or joint) either by name or by ID.
class MjDataIndexer {
 public:
  MjDataIndexer(raw::MjData* d, const raw::MjModel* m,
                pybind11::handle owner);

#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS) \
  MjDataGroupedViews& field(int i);                              \
  MjDataGroupedViews& field##_by_name(std::string_view name);

  MJDATA_VIEW_GROUPS
#undef XGROUP

 private:
  raw::MjData* d_;
  const raw::MjModel* m_;
  pybind11::handle owner_;
  NameToIDMappings name_to_id_;
  IDToNameMappings id_to_name_;

  // Lazily instantiate a grouped views object when accessed from Python, but
  // cache it once made so that we can return the same one if requested again.
#define XGROUP(MjDataGroupedViews, field, nfield, FIELD_XMACROS) \
  std::vector<std::optional<MjDataGroupedViews>> field##_;

  MJDATA_VIEW_GROUPS
#undef XGROUP
};

// Returns an error message when a nonexistent name is requested from an
// indexer, which includes all valid names.
std::string KeyErrorMessage(const NameToID& map, std::string_view name);
// Returns an error message when an invalid numeric index is provided.
std::string IndexErrorMessage(int index, int size);
}  // namespace mujoco::python

#endif  // MUJOCO_PYTHON_INDEXERS_H_
