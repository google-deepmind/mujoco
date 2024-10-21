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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <unordered_map>
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
constexpr int kNumEdges = Stencil2D::kNumEdges;
constexpr int kNumVerts = Stencil2D::kNumVerts;
constexpr int edge[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};

// cotangent between two edges
mjtNum cot(mjtNum* x, int v0, int v1, int v2) {
  mjtNum normal[3];
  mjtNum edge1[3];
  mjtNum edge2[3];

  mju_sub3(edge1, x+3*v1, x+3*v0);
  mju_sub3(edge2, x+3*v2, x+3*v0);
  mju_cross(normal, edge1, edge2);

  return mju_dot3(edge1, edge2) / mju_norm3(normal);
}

// area of a triangle
mjtNum ComputeVolume(const mjtNum* x, const int v[kNumVerts]) {
  mjtNum normal[3];
  mjtNum edge1[3];
  mjtNum edge2[3];

  mju_sub3(edge1, x+3*v[1], x+3*v[0]);
  mju_sub3(edge2, x+3*v[2], x+3*v[0]);
  mju_cross(normal, edge1, edge2);

  return mju_norm3(normal) / 2;
}

}  // namespace

// factory function
std::optional<Shell> Shell::Create(const mjModel* m, mjData* d, int instance) {
    if (CheckAttr("face", m, instance) &&
        CheckAttr("edge", m, instance) &&
        CheckAttr("poisson", m, instance) &&
        CheckAttr("young", m, instance) &&
        CheckAttr("thickness", m, instance)) {
      mjtNum nu = strtod(mj_getPluginConfig(m, instance, "poisson"), nullptr);
      mjtNum E = strtod(mj_getPluginConfig(m, instance, "young"), nullptr);
      mjtNum thick =
          strtod(mj_getPluginConfig(m, instance, "thickness"), nullptr);
      std::vector<int> face, edge;
      String2Vector(mj_getPluginConfig(m, instance, "face"), face);
      String2Vector(mj_getPluginConfig(m, instance, "edge"), edge);
      return Shell(m, d, instance, nu, E, thick, face, edge);
    } else {
      mju_warning("Invalid parameter specification in shell plugin");
      return std::nullopt;
    }
}

// create map from triangles to vertices and edges and from edges to vertices
void Shell::CreateStencils(const std::vector<int>& simplex,
                           const std::vector<int>& edgeidx) {
  // populate stencil
  nt = simplex.size() / kNumVerts;
  elements.resize(nt);
  for (int t = 0; t < nt; t++) {
    for (int v = 0; v < kNumVerts; v++) {
      elements[t].vertices[v] = simplex[kNumVerts*t+v];
    }
  }

  // map from edge vertices to their index in `edges` vector
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_indices;

  // loop over all triangles
  for (int t = 0; t < nt; t++) {
    int* v = elements[t].vertices;

    // compute edges to vertices map for fast computations
    for (int e = 0; e < kNumEdges; e++) {
      auto pair = std::pair(
        std::min(v[edge[e][0]], v[edge[e][1]]),
        std::max(v[edge[e][0]], v[edge[e][1]])
      );

      // if edge is already present in the vector only store its index
      auto [it, inserted] = edge_indices.insert({pair, ne});

      if (inserted) {
        StencilFlap flap;
        flap.vertices[0] = v[edge[e][0]];
        flap.vertices[1] = v[edge[e][1]];
        flap.vertices[2] = v[(edge[e][1]+1) % 3];
        flap.vertices[3] = -1;
        flaps.push_back(flap);
        elements[t].edges[e] = ne++;
      } else {
        elements[t].edges[e] = it->second;
        flaps[it->second].vertices[3] = v[(edge[e][1]+1) % 3];
      }

      if (!edgeidx.empty()) {
        assert(elements[t].edges[e] == edgeidx[kNumEdges*t+e]);
      }
    }
  }
}

// plugin constructor
Shell::Shell(const mjModel* m, mjData* d, int instance, mjtNum nu, mjtNum E,
             mjtNum thick, const std::vector<int>& face,
             const std::vector<int>& edgeidx)
    : thickness(thick) {
  // count plugin bodies
  nv = ne = 0;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      if (!nv++) {
        i0 = i;
      }
    }
  }

  // generate triangles from the vertices
  CreateStencils(face, edgeidx);

  // material parameters
  mjtNum mu = E / (2*(1+nu));

  // loop over all triangles
  for (int t = 0; t < nt; t++) {
    int* v = elements[t].vertices;
    for (int i = 0; i < kNumVerts; i++) {
      if (m->body_plugin[i0+v[i]] != instance) {
        mju_error("This body does not have the requested plugin instance");
      }
    }
  }

  // allocate array
  position.assign(nv*3, 0);
  bending.assign(ne*16, 0);

  // store previous positions
  mju_copy(position.data(), m->body_pos+3*i0, 3*nv);

  // assemble bending Hessian
  for (int e = 0; e < ne; e++)  {
    int* v = flaps[e].vertices;
    int vadj[3] = {v[1], v[0], v[3]};

    if (v[3]== -1) {
      // skip boundary edges
      continue;
    }

    // cotangent operator from Wardetzky at al., "Discrete Quadratic Curvature
    // Energies", https://cims.nyu.edu/gcl/papers/wardetzky2007dqb.pdf

    mjtNum a01 = cot(m->body_pos+3*i0, v[0], v[1], v[2]);
    mjtNum a02 = cot(m->body_pos+3*i0, v[0], v[3], v[1]);
    mjtNum a03 = cot(m->body_pos+3*i0, v[1], v[2], v[0]);
    mjtNum a04 = cot(m->body_pos+3*i0, v[1], v[0], v[3]);
    mjtNum c[4] = {a03 + a04, a01 + a02, -(a01 + a03), -(a02 + a04)};
    mjtNum volume = ComputeVolume(m->body_pos+3*i0, v) +
                    ComputeVolume(m->body_pos+3*i0, vadj);

    for (int v1 = 0; v1 < StencilFlap::kNumVerts; v1++) {
      for (int v2 = 0; v2 < StencilFlap::kNumVerts; v2++) {
        bending[16 * e + 4 * v1 + v2] +=
            1.5 * c[v1] * c[v2] / volume * mu * pow(thickness, 3) / 12;
      }
    }
  }
}

void Shell::Compute(const mjModel* m, mjData* d, int instance) {
  for (int e = 0; e < ne; e++) {
    int* v = flaps[e].vertices;
    mjtNum force[12] = {0};
    if (v[3] == -1) {
      // skip boundary edges
      continue;
    }
    for (int i = 0; i < StencilFlap::kNumVerts; i++) {
      for (int j = 0; j < StencilFlap::kNumVerts; j++) {
        for (int x = 0; x < 3; x++) {
          force[3*i+x] += bending[16*e+4*i+j] * d->xpos[3*(i0+v[j])+x];
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

  const char* attributes[] = {"face",    "edge",    "young",
                              "poisson", "thickness", "damping"};
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
