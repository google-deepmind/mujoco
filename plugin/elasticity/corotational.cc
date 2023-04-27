
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
#include <algorithm>
#include <cstddef>
#include <sstream>
#include <optional>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "corotational.h"

namespace mujoco::plugin::elasticity {
namespace {

// Gauss Legendre quadrature points in 1 dimension on the interval [a, b]
void quadratureGaussLegendre(mjtNum* points, mjtNum* weights,
                             const int order, const mjtNum a, const mjtNum b) {
  if (order > 2)
    mju_error("Integration order > 2 not yet supported.");
  // x is on [-1, 1], p on [a, b]
  mjtNum p0 = (a+b)/2.;
  mjtNum dpdx = (b-a)/2;
  points[0] = -dpdx/sqrt(3) + p0;
  points[1] =  dpdx/sqrt(3) + p0;
  weights[0] = dpdx;
  weights[1] = dpdx;
}

// evaluate 1-dimensional basis function
mjtNum phi(const mjtNum s, const mjtNum component) {
  if (component == 0) {
    return 1-s;
  } else {
    return s;
  }
}

// evaluate gradient fo 1-dimensional basis function
mjtNum dphi(const mjtNum s, const mjtNum component) {
  if (component == 0) {
    return -1;
  } else {
    return 1;
  }
}

// convert from joint to global coordinates
void evalFlexibleKinematics(mjtNum* jac, const int ncols) {
  // col 0
  jac[0*ncols] = 1/sqrt(3);
  jac[1*ncols] = 1/sqrt(3);
  jac[2*ncols] = 1/sqrt(3);
  // col 1
  jac[3*ncols+1] = -1/sqrt(3);
  jac[4*ncols+1] =  1/sqrt(3);
  jac[5*ncols+1] =  1/sqrt(3);
  // col 14
  jac[18*ncols+14] = 1/sqrt(3);
  jac[19*ncols+14] = 1/sqrt(3);
  jac[20*ncols+14] = 1/sqrt(3);
  // col 15
  jac[21*ncols+15] = -1/sqrt(3);
  jac[22*ncols+15] =  1/sqrt(3);
  jac[23*ncols+15] =  1/sqrt(3);
  // identity
  for (int k=0; k < 12; k++) {
    jac[(k+6)*ncols + (k+2)] = 1;
  }
}

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char *end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}
}  // namespace
// factory function
std::optional<Corotational> Corotational::Create(
  const mjModel* m, mjData* d, int instance) {
  if (CheckAttr("modulus", m, instance)) {
    return Corotational(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in solid plugin");
    return std::nullopt;
  }
}

// plugin constructor
Corotational::Corotational(const mjModel* m, mjData* d, int instance) {
  // parameters were validated by the factor function
  mjtNum E = strtod(mj_getPluginConfig(m, instance, "modulus"), nullptr);
  // count plugin bodies
  n = 0;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      if (!n++) {
        i0 = i;
      }
    }
  }
  // degrees of freedom
  nrows = 3*n;
  ncols = 16;
  // quadrature order
  int order = 2;
  if (pow(order, 3) != n) {
    mju_error("Order is not supported");
  }
  // allocate arrays
  std::vector<mjtNum> stiffness(n);    // material parameters
  K.assign(nrows*nrows, 0);            // stiffness matrix
  jac.assign(nrows*ncols, 0);          // jacobian matrix
  // compute quadrature points
  std::vector<mjtNum> points(order);     // quadrature points
  std::vector<mjtNum> weight(order);     // quadrature weights
  quadratureGaussLegendre(points.data(), weight.data(), order, 0, 1);
  // compute stiffness
  for (int b = 0; b < n; b++) {
    int i = i0 + b;
    if (m->body_plugin[i] != instance) {
      mju_error("This body does not have the requested plugin instance");
    }
    // compute physical parameters
    stiffness[b] = E;
  }
  // compute stiffness matrix
  mjtNum s, t, u, dvol;
  std::vector<mjtNum> F(nrows);
  mju_zero(K.data(), nrows * nrows);
  // loop over quadrature points
  for (int ps=0; ps < order; ps++) {
    for (int pt=0; pt < order; pt++) {
      for (int pu=0; pu < order; pu++) {
        s = points[ps];
        t = points[pt];
        u = points[pu];
        dvol = weight[ps]*weight[pt]*weight[pu];
        // cartesian product of basis functions
        for (int bx=0; bx < order; bx++) {
          for (int by=0; by < order; by++) {
            for (int bz=0; bz < order; bz++) {
              int basis_idx = (order*order*bx + order*by + bz);
              F[3*basis_idx+0] = dphi(s, bx) *  phi(t, by) *  phi(u, bz);
              F[3*basis_idx+1] =  phi(s, bx) * dphi(t, by) *  phi(u, bz);
              F[3*basis_idx+2] =  phi(s, bx) *  phi(t, by) * dphi(u, bz);
            }
          }
        }
        // tensor contraction of the gradients of elastic strains
        // (d(F+F')/dx : d(F+F')/dx)
        for (int i=0; i < n; i++) {
          for (int j=0; j < n; j++) {
            for (int d=0; d < 3; d++) {
              K[nrows*(3*i+d) + 3*j+d] -=
                1e2 * ( mju_dot(F.data()+3*i, F.data()+3*j, 3) +
                        mju_dot(F.data()+3*j, F.data()+3*i, 3) ) * dvol;
            }
            for (int k=0; k < 3; k++) {
              for (int l=0; l < 3; l++) {
                K[nrows*(3*i+k) + 3*j+l] -=
                  1e2 * ( F[3*i+k]*F[3*j+l] +
                          F[3*j+k]*F[3*i+l] ) * dvol;
              }
            }
          }
        }
      }
    }
  }
  // compute kinematics
  mju_zero(jac.data(), nrows * ncols);
  evalFlexibleKinematics(jac.data(), ncols);
}

void Corotational::Compute(const mjModel* m, mjData* d, int instance) {
  int offset = m->nq - ncols;
  mjtNum* xfrc_local = (mjtNum*) mju_malloc(sizeof(mjtNum)*nrows);
  mjtNum* xpos_local = (mjtNum*) mju_malloc(sizeof(mjtNum)*nrows);
  // elastic force as matrix-vector product
  mju_mulMatVec(xpos_local, jac.data(), d->qpos + offset, nrows, ncols);
  mju_mulMatVec(xfrc_local, K.data(), xpos_local, nrows, nrows);
  mju_mulMatTVec(
    d->qfrc_passive+offset - 1, jac.data(), xfrc_local, nrows, ncols);
  mju_free(xfrc_local);
  mju_free(xpos_local);
}

void Corotational::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.corotational";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"modulus"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto elasticity_or_null = Corotational::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Corotational(std::move(*elasticity_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Corotational*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* elasticity = reinterpret_cast<Corotational*>(d->plugin_data[instance]);
        elasticity->Compute(m, d, instance);
      };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::elasticity
