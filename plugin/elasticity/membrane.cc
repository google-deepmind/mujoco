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
#include <optional>
#include <utility>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "elasticity.h"
#include "membrane.h"


namespace mujoco::plugin::elasticity {
namespace {

// local tetrahedron numbering
constexpr int kNumEdges = Stencil2D::kNumEdges;
constexpr int kNumVerts = Stencil2D::kNumVerts;

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

// compute local basis
void ComputeBasis(mjtNum basis[9], const mjtNum* x, const int v[kNumVerts],
                  const int faceL[2], const int faceR[2], mjtNum area) {
  mjtNum basisL[3], basisR[3];
  mjtNum edgesL[3], edgesR[3];
  mjtNum normal[3];

  mju_sub3(edgesL, x+3*v[faceL[0]], x+3*v[faceL[1]]);
  mju_sub3(edgesR, x+3*v[faceR[1]], x+3*v[faceR[0]]);

  mju_cross(normal, edgesR, edgesL);
  mju_normalize3(normal);
  mju_cross(basisL, normal, edgesL);
  mju_cross(basisR, edgesR, normal);

  // we use as basis the symmetrized tensor products of the edge normals of the
  // other two edges; this is shown in Weischedel "A discrete geometric view on
  // shear-deformable shell models" in the remark at the end of section 4.1;
  // equivalent to linear finite elements but in a coordinate-free formulation.

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      basis[3*i+j] = ( basisL[i]*basisR[j] +
                       basisR[i]*basisL[j] ) / (8*area*area);
    }
  }
}

}  // namespace

// factory function
std::optional<Membrane> Membrane::Create(const mjModel* m, mjData* d,
                                         int instance) {
  if (CheckAttr("face", m, instance) && CheckAttr("poisson", m, instance) &&
      CheckAttr("young", m, instance) && CheckAttr("thickness", m, instance)) {
    mjtNum nu = strtod(mj_getPluginConfig(m, instance, "poisson"), nullptr);
    mjtNum E = strtod(mj_getPluginConfig(m, instance, "young"), nullptr);
    mjtNum thick =
        strtod(mj_getPluginConfig(m, instance, "thickness"), nullptr);
    mjtNum damp =
            strtod(mj_getPluginConfig(m, instance, "damping"), nullptr);
    std::vector<int> face, edge;
    String2Vector(mj_getPluginConfig(m, instance, "face"), face);
    String2Vector(mj_getPluginConfig(m, instance, "edge"), edge);
    return Membrane(m, d, instance, nu, E, thick, damp, face, edge);
  } else {
    mju_warning("Invalid parameter specification in shell plugin");
    return std::nullopt;
  }
}

// plugin constructor
Membrane::Membrane(const mjModel* m, mjData* d, int instance, mjtNum nu,
                   mjtNum E, mjtNum thick, mjtNum damp,
                   const std::vector<int>& simplex,
                   const std::vector<int>& edgeidx)
    : f0(-1), damping(damp), thickness(thick) {
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
      }
    }
  }

  // vertex positions
  mjtNum* body_pos =
      f0 < 0 ? m->body_pos + 3*i0 : m->flex_xvert0 + 3*m->flex_vertadr[f0];

  // generate triangles from the vertices
  nt = CreateStencils<Stencil2D>(elements, edges, simplex, edgeidx);

  // allocate metric induced by geometry
  metric.assign(kNumEdges*kNumEdges*nt, 0);

  // loop over all triangles
  for (int t = 0; t < nt; t++) {
    int* v = elements[t].vertices;
    for (int i = 0; i < kNumVerts; i++) {
      int bi = f0 < 0 ? i0+v[i] : m->flex_vertbodyid[m->flex_vertadr[f0]+v[i]];
      if (bi && m->body_plugin[bi] != instance) {
        mju_error("Body %d does not have plugin instance %d", bi, instance);
      }
    }

    // triangles area
    mjtNum volume = ComputeVolume(body_pos, v);

    // material parameters
    mjtNum mu = E / (2*(1+nu)) * mju_abs(volume) / 4 * thickness;
    mjtNum la = E*nu / ((1+nu)*(1-2*nu)) * mju_abs(volume) / 4 * thickness;

    // local geometric quantities
    mjtNum basis[kNumEdges][9] = {{0}, {0}, {0}};

    // compute edge basis
    for (int e = 0; e < kNumEdges; e++) {
      ComputeBasis(basis[e], body_pos, v,
                   Stencil2D::edge[Stencil2D::edge[e][0]],
                   Stencil2D::edge[Stencil2D::edge[e][1]], volume);
    }

    // compute metric tensor
    MetricTensor<Stencil2D>(metric, t, mu, la, basis);
  }

  // allocate array
  ne = edges.size();
  reference.assign(ne, 0);
  deformed.assign(ne, 0);
  previous.assign(ne, 0);
  elongation.assign(ne, 0);

  // compute edge lengths at equilibrium (m->flexedge_length0 not yet available)
  UpdateSquaredLengths(reference, edges, body_pos);

  // save previous lengths
  previous = reference;
}

void Membrane::Compute(const mjModel* m, mjData* d, int instance) {
  mjtNum kD = damping / m->opt.timestep;

  // update edge lengths
  if (f0 < 0) {
    UpdateSquaredLengths(deformed, edges, d->xpos+3*i0);
  } else {
    UpdateSquaredLengthsFlex(deformed,
                             d->flexedge_length + m->flex_edgeadr[f0]);
  }

  // we add generalized Rayleigh damping as decribed in Section 5.2 of
  // Kharevych et al., "Geometric, Variational Integrators for Computer
  // Animation" http://multires.caltech.edu/pubs/DiscreteLagrangian.pdf

  for (int idx = 0; idx < ne; idx++) {
    elongation[idx] = deformed[idx] - reference[idx] +
                    ( deformed[idx] -  previous[idx] ) * kD;
  }

  // compute gradient of elastic energy and insert into passive force
  int flex_vertadr = f0 < 0 ? -1 : m->flex_vertadr[f0];
  int* bodyid = f0 < 0 ? nullptr : m->flex_vertbodyid + flex_vertadr;
  mjtNum* xpos = f0 < 0 ? d->xpos + 3*i0 : d->flexvert_xpos + 3*flex_vertadr;
  mjtNum* qfrc = d->qfrc_passive + (f0 < 0 ? m->body_dofadr[i0] : 0);

  ComputeForce<Stencil2D>(qfrc, elements, metric, elongation, m, bodyid, xpos);

  // update stored lengths
  if (kD > 0) {
    previous = deformed;
  }
}



void Membrane::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.membrane";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"face", "edge", "young", "poisson", "thickness", "damping"};
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
