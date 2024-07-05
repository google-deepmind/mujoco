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
#include <optional>
#include <utility>
#include <vector>

#include <SdfLib/utils/Mesh.h>
#include <SdfLib/OctreeSdf.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "sdf.h"
#include "sdflib.h"

namespace mujoco::plugin::sdf {
namespace {

inline unsigned int* MakeNonConstUnsigned(const int* ptr) {
  return reinterpret_cast<unsigned int*>(const_cast<int*>(ptr));
}

mjtNum boxProjection(glm::vec3& point, const sdflib::BoundingBox& box) {
  glm::vec3 r = point - box.getCenter();
  glm::vec3 q = glm::abs(r) - 0.5f * box.getSize();
  mjtNum dist_sqr = 0;
  mjtNum eps = 1e-6;

  // skip the projection if inside
  if (q.x <= 0 && q.y <= 0 && q.z <= 0) {
    return glm::max(q.x, glm::max(q.y, q.z));
  }

  // in-place projection inside the box if outside
  if ( q.x >= 0 ) {
    dist_sqr += q.x * q.x;
    point.x -= r.x > 0 ? (q.x+eps) : -(q.x+eps);
  }
  if ( q.y >= 0 ) {
    dist_sqr += q.y * q.y;
    point.y -= r.y > 0 ? (q.y+eps) : -(q.y+eps);
  }
  if ( q.z >= 0 ) {
    dist_sqr += q.z * q.z;
    point.z -= r.z > 0 ? (q.z+eps) : -(q.z+eps);
  }

  return mju_sqrt(dist_sqr);
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
  int meshid = m->geom_dataid[geomid];
  int nvert = m->mesh_vertnum[meshid];
  int nface = m->mesh_facenum[meshid];
  int* indices = m->mesh_face + 3*m->mesh_faceadr[meshid];
  float* verts = m->mesh_vert + 3*m->mesh_vertadr[meshid];
  std::vector<glm::vec3> vertices(nvert);
  for (int i = 0; i < nvert; i++) {
    mjtNum vert[3] = {verts[3*i+0], verts[3*i+1], verts[3*i+2]};
    mju_rotVecQuat(vert, vert, m->mesh_quat + 4*meshid);
    mju_addTo3(vert, m->mesh_pos + 3*meshid);
    vertices[i].x = vert[0];
    vertices[i].y = vert[1];
    vertices[i].z = vert[2];
  }
  sdflib::Mesh mesh(vertices.data(), nvert,
                    MakeNonConstUnsigned(indices), 3*nface);
  mesh.computeBoundingBox();
  return SdfLib(std::move(mesh));
}

// plugin constructor
SdfLib::SdfLib(sdflib::Mesh&& mesh) {
  sdflib::BoundingBox box = mesh.getBoundingBox();
  const glm::vec3 modelBBsize = box.getSize();
  box.addMargin(
      0.1f * glm::max(glm::max(modelBBsize.x, modelBBsize.y), modelBBsize.z));
  sdf_func_ =
      sdflib::OctreeSdf(mesh, box, 8, 3, 1e-3,
                        sdflib::OctreeSdf::InitAlgorithm::CONTINUITY, 1);
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
  glm::vec3 point(p[0], p[1], p[2]);
  mjtNum boxDist = boxProjection(point, sdf_func_.getGridBoundingBox());
  return sdf_func_.getDistance(point) + (boxDist <= 0 ? 0 : boxDist);
}

// gradient of sdf
void SdfLib::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
  glm::vec3 gradient;
  glm::vec3 p(point[0], point[1], point[2]);

  // analytic in the interior
  if (boxProjection(p, sdf_func_.getGridBoundingBox()) <= 0) {
    sdf_func_.getDistance(p, gradient);
    grad[0] = gradient[0];
    grad[1] = gradient[1];
    grad[2] = gradient[2];
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
