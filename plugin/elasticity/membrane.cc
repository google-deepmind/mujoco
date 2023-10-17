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
    std::vector<int> face, edge;
    String2Vector(mj_getPluginConfig(m, instance, "face"), face);
    String2Vector(mj_getPluginConfig(m, instance, "edge"), edge);
    return Membrane(m, d, instance, nu, E, thick, face, edge);
  } else {
    mju_warning("Invalid parameter specification in shell plugin");
    return std::nullopt;
  }
}

// plugin constructor
Membrane::Membrane(const mjModel* m, mjData* d, int instance, mjtNum nu,
                   mjtNum E, mjtNum thick, const std::vector<int>& simplex,
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

  // count flexes
  for (int i = 0; i < m->nflex; i++) {
    if (m->flex_vertbodyid[m->flex_vertadr[i]] == i0) {
      f0 = i;
      break;
    }
  }

  // generate triangles from the vertices
  nt = CreateStencils<Stencil2D>(elements, edges, simplex, edgeidx);

  // allocate metric induced by geometry
  metric.assign(kNumEdges*kNumEdges*nt, 0);

  // loop over all triangles
  for (int t = 0; t < nt; t++) {
    int* v = elements[t].vertices;
    for (int i = 0; i < kNumVerts; i++) {
      if (m->body_plugin[i0+v[i]] != instance) {
        mju_error("This body does not have the requested plugin instance");
      }
    }

    // triangles area
    mjtNum volume = ComputeVolume(m->body_pos+3*i0, v);

    // material parameters
    mjtNum mu = E / (2*(1+nu)) * mju_abs(volume) / 4 * thickness;
    mjtNum la = E*nu / ((1+nu)*(1-2*nu)) * mju_abs(volume) / 4 * thickness;

    // local geometric quantities
    mjtNum basis[kNumEdges][9] = {{0}, {0}, {0}};

    // compute edge basis
    for (int e = 0; e < kNumEdges; e++) {
      ComputeBasis(basis[e], m->body_pos+3*i0, v,
                   Stencil2D::edge[Stencil2D::edge[e][0]],
                   Stencil2D::edge[Stencil2D::edge[e][1]], volume);
    }

    // compute metric tensor
    MetricTensor<Stencil2D>(metric, t, mu, la, basis);
  }
}

void Membrane::Compute(const mjModel* m, mjData* d, int instance) {
  for (int t = 0; t < nt; t++)  {
    int* v = elements[t].vertices;

    // compute length gradient with respect to dofs
    mjtNum gradient[kNumEdges][2][3];
    GradSquaredLengths<Stencil2D>(gradient, d->xpos+3*i0, v);

    // compute elongation
    mjtNum elongation[kNumEdges];
    for (int e = 0; e < kNumEdges; e++) {
      int idx = elements[t].edges[e] + m->flex_edgeadr[f0];
      mjtNum deformed = d->flexedge_length[idx]*d->flexedge_length[idx];
      mjtNum reference = m->flexedge_length0[idx]*m->flexedge_length0[idx];
      elongation[e] = deformed - reference;
    }

    // we now multiply the elongations by the precomputed metric tensor,
    // notice that if metric=diag(1/reference) then this would yield a
    // mass-spring model

    // compute local force
    mjtNum force[kNumVerts*3] = {0};
    int offset = kNumEdges*kNumEdges;
    for (int ed1 = 0; ed1 < kNumEdges; ed1++) {
      for (int ed2 = 0; ed2 < kNumEdges; ed2++) {
        for (int i = 0; i < 2; i++) {
          for (int x = 0; x < 3; x++) {
            force[3 * Stencil2D::edge[ed2][i] + x] +=
                elongation[ed1] * gradient[ed2][i][x] *
                metric[offset * t + kNumEdges * ed1 + ed2];
          }
        }
      }
    }

    // insert into global force
    for (int i = 0; i < kNumVerts; i++) {
      for (int x = 0; x < 3; x++) {
        d->qfrc_passive[m->body_dofadr[i0]+3*v[i]+x] -= force[3*i+x];
      }
    }
  }
}



void Membrane::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.membrane";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"face", "edge", "young", "poisson", "thickness"};
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
