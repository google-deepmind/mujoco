// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_PYTHON_MJDATA_META_H_
#define MUJOCO_PYTHON_MJDATA_META_H_

#include <mjxmacro.h>
#include "raw.h"

namespace mujoco::python {
namespace _impl {
template <typename T> class MjWrapper;
}  // namespace _impl

#define MJDATA_METADATA                 \
  X( int,  jnt_qposadr,      njnt     ) \
  X( int,  jnt_dofadr,       njnt     ) \
  X( int,  hfield_nrow,      nhfield  ) \
  X( int,  hfield_ncol,      nhfield  ) \
  X( int,  hfield_adr,       nhfield  ) \
  X( int,  tex_height,       ntex     ) \
  X( int,  tex_width,        ntex     ) \
  X( int,  tex_adr,          ntex     ) \
  X( int,  sensor_dim,       nsensor  ) \
  X( int,  sensor_adr,       nsensor  ) \
  X( int,  numeric_adr,      nnumeric ) \
  X( int,  numeric_size,     nnumeric ) \
  X( int,  tuple_adr,        ntuple   ) \
  X( int,  tuple_size,       ntuple   ) \
  X( int,  name_bodyadr,     nbody    ) \
  X( int,  name_jntadr,      njnt     ) \
  X( int,  name_geomadr,     ngeom    ) \
  X( int,  name_siteadr,     nsite    ) \
  X( int,  name_camadr,      ncam     ) \
  X( int,  name_lightadr,    nlight   ) \
  X( int,  name_meshadr,     nmesh    ) \
  X( int,  name_skinadr,     nskin    ) \
  X( int,  name_hfieldadr,   nhfield  ) \
  X( int,  name_texadr,      ntex     ) \
  X( int,  name_matadr,      nmat     ) \
  X( int,  name_pairadr,     npair    ) \
  X( int,  name_excludeadr,  nexclude ) \
  X( int,  name_eqadr,       neq      ) \
  X( int,  name_tendonadr,   ntendon  ) \
  X( int,  name_actuatoradr, nu       ) \
  X( int,  name_sensoradr,   nsensor  ) \
  X( int,  name_numericadr,  nnumeric ) \
  X( int,  name_textadr,     ntext    ) \
  X( int,  name_tupleadr,    ntuple   ) \
  X( int,  name_keyadr,      nkey     ) \
  X( char, names,            nnames   )

// A subset of mjModel fields that are required to reconstruct an MjDataWrapper.
struct MjDataMetadata {
 public:
  friend class _impl::MjWrapper<raw::MjData>;

#define X(var) int var;
  MJMODEL_INTS
#undef X
#define X(type, var, n) std::shared_ptr<type[]> var;
  MJDATA_METADATA
#undef X

 private:
  MjDataMetadata() = default;
  MjDataMetadata(const MjDataMetadata& other) = default;
  MjDataMetadata(MjDataMetadata&& other) = default;
  explicit MjDataMetadata(const raw::MjModel* m)
      :
#define X(var) var(m->var),
        MJMODEL_INTS
#undef X

#define X(dtype, var, n)                            \
  var(                                              \
      [](dtype* src, int len) {                     \
        dtype* dst = new dtype[len];                \
        std::memcpy(dst, src, len * sizeof(dtype)); \
        return dst;                                 \
      }(m->var, m->n)),

        MJDATA_METADATA
#undef X
        dummy_() {}

  bool dummy_;  // Dummy variable to terminate X macro sequences.
};

}  // namespace mujoco::python

#endif  // MUJOCO_PYTHON_MJDATA_META_H_
