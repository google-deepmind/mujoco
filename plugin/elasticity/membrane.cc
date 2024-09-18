// Copyright 2023 DeepMind Technologies Limited
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

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <utility>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "elasticity.h"
#include "membrane.h"


namespace mujoco::plugin::elasticity {

// factory function
std::optional<Membrane> Membrane::Create(const mjModel* m, mjData* d,
                                         int instance) {
  if (CheckAttr("face", m, instance)) {
    mjtNum damp =
            strtod(mj_getPluginConfig(m, instance, "damping"), nullptr);
    return Membrane(m, d, instance, damp);
  } else {
    mju_warning("Invalid parameter specification in shell plugin");
    return std::nullopt;
  }
}

// plugin constructor
Membrane::Membrane(const mjModel* m, mjData* d, int instance, mjtNum damp)
    : f0(-1), damping(damp) {
  // count plugin bodies
  nv = ne = 0;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      if (!nv++) {
        i0 = i;
      }
    }
  }

  // count flexes
  for (int i = 0; i < m->nflex; i++) {
    for (int j = 0; j < m->flex_vertnum[i]; j++) {
      if (m->flex_vertbodyid[m->flex_vertadr[i]+j] == i0) {
        f0 = i;
        nv = m->flex_vertnum[f0];
      }
    }
  }

  // loop over all triangles
  const int* elem = m->flex_elem + m->flex_elemdataadr[f0];
  for (int t = 0; t < m->flex_elemnum[f0]; t++) {
    const int* v = elem + (m->flex_dim[f0]+1) * t;
    for (int i = 0; i < Stencil2D::kNumVerts; i++) {
      int bi = m->flex_vertbodyid[m->flex_vertadr[f0]+v[i]];
      if (bi && m->body_plugin[bi] != instance) {
        mju_error("Body %d does not have plugin instance %d", bi, instance);
      }
    }
  }

  // allocate array
  ne = m->flex_edgenum[f0];
  elongation.assign(ne, 0);
  force.assign(3*nv, 0);
}

void Membrane::Compute(const mjModel* m, mjData* d, int instance) {
  mjtNum kD = damping / m->opt.timestep;

  // read edge lengths
  mjtNum* deformed = d->flexedge_length + m->flex_edgeadr[f0];
  mjtNum* ref = m->flexedge_length0 + m->flex_edgeadr[f0];

  // m->flexedge_length0 is not initialized when the plugin is constructed
  if (prev.empty()) {
    prev.assign(ne, 0);
    memcpy(prev.data(), ref, sizeof(mjtNum) * ne);
  }

  // we add generalized Rayleigh damping as decribed in Section 5.2 of
  // Kharevych et al., "Geometric, Variational Integrators for Computer
  // Animation" http://multires.caltech.edu/pubs/DiscreteLagrangian.pdf

  for (int idx = 0; idx < ne; idx++) {
    elongation[idx] =
        (deformed[idx] * deformed[idx] - prev[idx] * prev[idx]) * kD;
  }

  // compute gradient of elastic energy and insert into passive force
  int flex_vertadr = m->flex_vertadr[f0];
  mjtNum* xpos = d->flexvert_xpos + 3*flex_vertadr;
  mjtNum* qfrc = d->qfrc_passive;

  ComputeForce<Stencil2D>(force, elongation, m, f0, xpos);

  // insert into passive force
  AddFlexForce(qfrc, force, m, d, xpos, f0);

  // update stored lengths
  if (kD > 0) {
    memcpy(prev.data(), deformed, sizeof(mjtNum) * ne);
  }
}



void Membrane::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.membrane";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"face", "edge", "damping"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto elasticity_or_null = Membrane::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Membrane(std::move(*elasticity_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Membrane*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    auto* elasticity = reinterpret_cast<Membrane*>(d->plugin_data[instance]);
    elasticity->Compute(m, d, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::elasticity
