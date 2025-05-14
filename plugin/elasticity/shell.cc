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

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "elasticity.h"
#include "shell.h"


namespace mujoco::plugin::elasticity {
namespace {

// local tetrahedron numbering
constexpr int kNumVerts = Stencil2D::kNumVerts;



}  // namespace

// factory function
std::optional<Shell> Shell::Create(const mjModel* m, mjData* d, int instance) {
    return Shell(m, d, instance);
}

// plugin constructor
Shell::Shell(const mjModel* m, mjData* d, int instance)
    : f0(-1) {
  // count plugin bodies
  nv = 0;
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
        if (m->flex_dim[i] != 2) {  // SHOULD NOT OCCUR
          mju_error("mujoco.elasticity.shell requires a 2D mesh");
        }
      }
    }
  }

  // loop over all triangles
  for (int t = 0; t < m->flex_elemnum[f0]; t++) {
    int* v = m->flex_elem + 3*(t+m->flex_elemadr[f0]);
    for (int i = 0; i < kNumVerts; i++) {
      if (m->body_plugin[i0+v[i]] != instance) {
        mju_error("This body does not have the requested plugin instance");
      }
    }
  }

  // allocate array
  position.assign(nv*3, 0);

  // store previous positions
  mju_copy(position.data(), m->body_pos+3*i0, 3*nv);
}

void Shell::Compute(const mjModel* m, mjData* d, int instance) {
  for (int e = 0; e < m->flex_edgenum[f0]; e++) {
    int* edge = m->flex_edge + 2*(e+m->flex_edgeadr[f0]);
    int* flap = m->flex_edgeflap + 2*(e+m->flex_edgeadr[f0]);
    int v[4] = {edge[0], edge[1], flap[0], flap[1]};
    mjtNum force[12] = {0};
    if (v[3] == -1) {
      // skip boundary edges
      continue;
    }
    mjtNum* k = m->flex_bending + 16*m->flex_edgeadr[f0];
    for (int i = 0; i < StencilFlap::kNumVerts; i++) {
      for (int j = 0; j < StencilFlap::kNumVerts; j++) {
        for (int x = 0; x < 3; x++) {
          force[3*i+x] += k[16*e+4*i+j] * d->xpos[3*(i0+v[j])+x];
        }
      }
    }

    // update stored positions
    mju_copy(position.data(), d->xpos+3*i0, 3*nv);

    // insert into global force
    for (int i = 0; i < StencilFlap::kNumVerts; i++) {
      for (int x = 0; x < 3; x++) {
        d->qfrc_passive[m->body_dofadr[i0]+3*v[i]+x] -= force[3*i+x];
      }
    }
  }
}



void Shell::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.shell";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"damping"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto elasticity_or_null = Shell::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Shell(std::move(*elasticity_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Shell*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    auto* elasticity = reinterpret_cast<Shell*>(d->plugin_data[instance]);
    elasticity->Compute(m, d, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::elasticity
