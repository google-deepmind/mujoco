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

#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "sdf.h"
#include "sdflib.h"

namespace mujoco::plugin::sdf {
namespace {

mjtNum boxProjection(mjtNum point[3], const mjtNum box[6]) {
  mjtNum r[3] = {point[0] - box[0], point[1] - box[1], point[2] - box[2]};
  mjtNum q[3] = {mju_abs(r[0]) - box[3], mju_abs(r[1]) - box[4],
                 mju_abs(r[2]) - box[5]};
  mjtNum dist_sqr = 0;
  mjtNum eps = 1e-6;

  // skip the projection if inside
  if (q[0] <= 0 && q[1] <= 0 && q[2] <= 0) {
    return mju_max(q[0], mju_max(q[1], q[2]));
  }

  // in-place projection inside the box if outside
  if ( q[0] >= 0 ) {
    dist_sqr += q[0] * q[0];
    point[0] -= r[0] > 0 ? (q[0]+eps) : -(q[0]+eps);
  }
  if ( q[1] >= 0 ) {
    dist_sqr += q[1] * q[1];
    point[1] -= r[1] > 0 ? (q[1]+eps) : -(q[1]+eps);
  }
  if ( q[2] >= 0 ) {
    dist_sqr += q[2] * q[2];
    point[2] -= r[2] > 0 ? (q[2]+eps) : -(q[2]+eps);
  }

  return mju_sqrt(dist_sqr);
}

// find the octree leaf containing the point p, return the index of the leaf and
// populate the weights of the interpolated function (if w is not null) and of
// its gradient (if dw is not null) using the vertices as degrees of freedom for
// trilinear interpolation.
static int findOct(mjtNum w[8], mjtNum dw[8][3], const mjtNum* oct_aabb,
                   const int* oct_child, const mjtNum p[3]) {
  std::vector<int> stack = {0};
  mjtNum eps = 1e-8;

  while (!stack.empty()) {
    int node = stack.back();
    stack.pop_back();
    mjtNum vmin[3], vmax[3];

    if (node == -1) {  // SHOULD NOT OCCUR
      mju_error("Invalid node number");
      return -1;
    }

    for (int j = 0; j < 3; j++) {
      vmin[j] = oct_aabb[6*node+j] - oct_aabb[6*node+3+j];
      vmax[j] = oct_aabb[6*node+j] + oct_aabb[6*node+3+j];
    }

    // check if the point is inside the aabb of the octree node
    if (p[0] + eps < vmin[0] || p[0] - eps > vmax[0] ||
        p[1] + eps < vmin[1] || p[1] - eps > vmax[1] ||
        p[2] + eps < vmin[2] || p[2] - eps > vmax[2]) {
      continue;
    }

    mjtNum coord[3] = {(p[0] - vmin[0]) / (vmax[0] - vmin[0]),
                       (p[1] - vmin[1]) / (vmax[1] - vmin[1]),
                       (p[2] - vmin[2]) / (vmax[2] - vmin[2])};

    // check if the node is a leaf
    if (oct_child[8*node+0] == -1 && oct_child[8*node+1] == -1 &&
        oct_child[8*node+2] == -1 && oct_child[8*node+3] == -1 &&
        oct_child[8*node+4] == -1 && oct_child[8*node+5] == -1 &&
        oct_child[8*node+6] == -1 && oct_child[8*node+7] == -1) {
      for (int j = 0; j < 8; j++) {
        if (w) {
          w[j] = (j & 1 ? coord[0] : 1 - coord[0]) *
                 (j & 2 ? coord[1] : 1 - coord[1]) *
                 (j & 4 ? coord[2] : 1 - coord[2]);
        }
        if (dw) {
          dw[j][0] = (j & 1 ? 1 : -1) *
                     (j & 2 ? coord[1] : 1 - coord[1]) *
                     (j & 4 ? coord[2] : 1 - coord[2]);
          dw[j][1] = (j & 1 ? coord[0] : 1 - coord[0]) *
                     (j & 2 ? 1 : -1) *
                     (j & 4 ? coord[2] : 1 - coord[2]);
          dw[j][2] = (j & 1 ? coord[0] : 1 - coord[0]) *
                     (j & 2 ? coord[1] : 1 - coord[1]) *
                     (j & 4 ? 1 : -1);
        }
      }
      return node;
    }

    // compute which of 8 children to visit next
    int x = coord[0] < .5 ? 1 : 0;
    int y = coord[1] < .5 ? 1 : 0;
    int z = coord[2] < .5 ? 1 : 0;
    stack.push_back(oct_child[8*node + 4*z + 2*y + x]);
  }

  mju_error("Node not found");  // SHOULD NOT OCCUR
  return -1;
}

}  // namespace

// factory function
std::optional<SdfLib> SdfLib::Create(const mjModel* m, mjData* d,
                                     int instance) {
  int geomid = 0;
  for (int i = 0; i < m->ngeom; ++i) {
    if (m->geom_plugin[i] == instance) {
      geomid = i;
      break;
    }
  }
  return SdfLib(m, m->geom_dataid[geomid]);
}

// plugin constructor
SdfLib::SdfLib(const mjModel* m, int meshid) {
  int octadr = m->mesh_octadr[meshid];
  int octnum = m->mesh_octnum[meshid];
  oct_aabb_.assign(m->oct_aabb + 6*octadr,
                   m->oct_aabb + 6*octadr + 6*octnum);
  oct_child_.assign(m->oct_child + 8 * octadr,
                    m->oct_child + 8 * octadr + 8 * octnum);
  sdf_coeff_.assign(8 * octnum, 0);
  memcpy(sdf_coeff_.data(), m->oct_coeff + 8*octadr, 8*octnum*sizeof(mjtNum));
  mju_copy(box_, m->oct_aabb + 6*octadr, 6);
}

// plugin computation
void SdfLib::Compute(const mjModel* m, mjData* d, int instance) {
  visualizer_.Next();
}

// plugin reset
void SdfLib::Reset() {
  visualizer_.Reset();
}

// plugin visualization
void SdfLib::Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                       mjvScene* scn, int instance) {
  visualizer_.Visualize(m, d, opt, scn, instance);
}

// sdf
mjtNum SdfLib::Distance(const mjtNum p[3]) const {
  mjtNum w[8];
  mjtNum sdf = 0;
  mjtNum point[3] = {p[0], p[1], p[2]};
  mjtNum boxDist = boxProjection(point, box_);
  if (boxDist > 0) {
    return boxDist;
  }
  int node = findOct(w, nullptr, oct_aabb_.data(), oct_child_.data(), point);
  for (int i = 0; i < 8; ++i) {
    sdf += w[i] * sdf_coeff_[8*node + i];
  }
  return sdf;
}

// gradient of sdf
void SdfLib::Gradient(mjtNum grad[3], const mjtNum point[3]) const {

  mjtNum p[3] = {point[0], point[1], point[2]};

  // analytic in the interior
  if (boxProjection(p, box_) <= 0) {
    mjtNum dw[8][3];
    mju_zero3(grad);
    int node = findOct(nullptr, dw, oct_aabb_.data(), oct_child_.data(), p);
    for (int i = 0; i < 8; ++i) {
      grad[0] += dw[i][0] * sdf_coeff_[8*node + i];
      grad[1] += dw[i][1] * sdf_coeff_[8*node + i];
      grad[2] += dw[i][2] * sdf_coeff_[8*node + i];
    }
    return;
  }

  // finite difference in the exterior
  mjtNum eps = 1e-8;
  mjtNum dist0 = Distance(point);
  mjtNum pointX[3] = {point[0]+eps, point[1], point[2]};
  mjtNum distX = Distance(pointX);
  mjtNum pointY[3] = {point[0], point[1]+eps, point[2]};
  mjtNum distY = Distance(pointY);
  mjtNum pointZ[3] = {point[0], point[1], point[2]+eps};
  mjtNum distZ = Distance(pointZ);

  grad[0] = (distX - dist0) / eps;
  grad[1] = (distY - dist0) / eps;
  grad[2] = (distZ - dist0) / eps;
}

// plugin registration
void SdfLib::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.sdflib";
  plugin.capabilityflags |= mjPLUGIN_SDF;

  const char* attributes[] = {"aabb"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto sdf_or_null = SdfLib::Create(m, d, instance);
    if (!sdf_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new SdfLib(std::move(*sdf_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<SdfLib*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                     int instance) {
    auto sdf = reinterpret_cast<SdfLib*>(plugin_data);
    sdf->Reset();
  };
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                         mjvScene* scn, int instance) {
    auto* sdf = reinterpret_cast<SdfLib*>(d->plugin_data[instance]);
    sdf->Visualize(m, d, opt, scn, instance);
  };
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* sdf = reinterpret_cast<SdfLib*>(d->plugin_data[instance]);
        sdf->Compute(m, d, instance);
      };
  plugin.sdf_distance =
      +[](const mjtNum point[3], const mjData* d, int instance) {
        auto* sdf = reinterpret_cast<SdfLib*>(d->plugin_data[instance]);
        return sdf->Distance(point);
      };
  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3],
                        const mjData* d, int instance) {
    auto* sdf = reinterpret_cast<SdfLib*>(d->plugin_data[instance]);
    sdf->visualizer_.AddPoint(point);
    sdf->Gradient(gradient, point);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sdf
