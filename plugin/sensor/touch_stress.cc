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

#include "touch_stress.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::sensor {

namespace {

// Checks that a plugin config attribute exists.
bool CheckAttr(const std::string& input) {
  char* end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

// Converts a string into a numeric vector
template <typename T>
void ReadVector(std::vector<T>& output, const std::string& input) {
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while (getline(ss, item, delim)) {
    CheckAttr(item);
    output.push_back(strtod(item.c_str(), nullptr));
  }
}

// Evenly spaced numbers over a specified interval.
void LinSpace(mjtNum lower, mjtNum upper, int n, mjtNum array[]) {
  mjtNum increment = n > 1 ? (upper - lower) / (n - 1) : 0;
  for (int i = 0; i < n; ++i) {
    *array = lower;
    ++array;
    lower += increment;
  }
}

// Parametrized linear/quintic interpolated nonlinearity.
mjtNum Fovea(mjtNum x, mjtNum gamma) {
  // Quick return.
  if (!gamma) return x;

  // Foveal deformation.
  mjtNum g = mjMAX(0, mjMIN(1, gamma));
  return g*mju_pow(x, 5) + (1 - g)*x;
}

// Make bin edges.
void BinEdges(mjtNum* x_edges, mjtNum* y_edges, int size[2], mjtNum fov[2],
              mjtNum gamma) {
  // Make unit bin edges.
  LinSpace(-1, 1, size[0] + 1, x_edges);
  LinSpace(-1, 1, size[1] + 1, y_edges);

  // Apply foveal deformation.
  for (int i = 0; i < size[0] + 1; i++) {
    x_edges[i] = Fovea(x_edges[i], gamma);
  }
  for (int i = 0; i < size[1] + 1; i++) {
    y_edges[i] = Fovea(y_edges[i], gamma);
  }

  // Scale by field-of-view.
  mju_scl(x_edges, x_edges, fov[0]*mjPI / 180, size[0] + 1);
  mju_scl(y_edges, y_edges, fov[1]*mjPI / 180, size[1] + 1);
}

// Permute 3-vector from 0,1,2 to 2,0,1.
static void xyz2zxy(mjtNum* x) {
  mjtNum z = x[2];
  x[2] = x[1];
  x[1] = x[0];
  x[0] = z;
}

// Transform spherical (azimuth, elevation, radius) to Cartesian (x,y,z).
void SphericalToCartesian(const mjtNum aer[3], mjtNum xyz[3]) {
  mjtNum a = aer[0], e = aer[1], r = aer[2];
  xyz[0] = r * mju_cos(e) * mju_sin(a);
  xyz[1] = r * mju_sin(e);
  xyz[2] = -r * mju_cos(e) * mju_cos(a);
}

// Tangent frame in Cartesian coordinates.
void TangentFrame(const mjtNum aer[3], mjtNum mat[9]) {
  mjtNum a = aer[0], e = aer[1], r = aer[2];
  mjtNum ta[3] = {r * mju_cos(e) * mju_cos(a), 0,
                  r * mju_cos(e) * mju_sin(a)};
  mjtNum te[3] = {-r * mju_sin(e) * mju_sin(a), r * mju_cos(e),
                   r * mju_sin(e) * mju_cos(a)};
  mju_normalize3(ta);
  mju_normalize3(te);
  mju_copy3(mat, ta);
  mju_copy3(mat+3, te);
  mju_cross(mat+6, te, ta);
}

}  // namespace

// Creates a TouchStress instance if all config attributes are defined and
// within their allowed bounds.
TouchStress* TouchStress::Create(const mjModel* m, mjData* d,
                                                 int instance) {
  if (CheckAttr(std::string(mj_getPluginConfig(m, instance, "gamma"))) &&
      CheckAttr(std::string(mj_getPluginConfig(m, instance, "nchannel")))) {
    // nchannel
    int nchannel = strtod(mj_getPluginConfig(m, instance, "nchannel"), nullptr);
    if (!nchannel) nchannel = 1;
    if (nchannel < 1 || nchannel > 3) {
      mju_error("nchannel must be between 1 and 3");
      return nullptr;
    }

    // size
    std::vector<int> size;
    std::string size_str = std::string(mj_getPluginConfig(m, instance, "size"));
    ReadVector(size, size_str.c_str());
    if (size.size()!= 2) {
      mju_error("Both horizontal and vertical resolutions must be specified");
      return nullptr;
    }
    if (size[0] <= 0 || size[1] <= 0) {
      mju_error("Horizontal and vertical resolutions must be positive");
      return nullptr;
    }

    // field of view
    std::vector<mjtNum> fov;
    std::string fov_str = std::string(mj_getPluginConfig(m, instance, "fov"));
    ReadVector(fov, fov_str.c_str());
    if (fov.size()!= 2) {
      mju_error(
          "Both horizontal and vertical fields of view must be specified");
      return nullptr;
    }
    if (fov[0] <= 0 || fov[0] > 180) {
      mju_error("`fov[0]` must be a float between (0, 180] degrees");
      return nullptr;
    }
    if (fov[1] <= 0 || fov[1] > 90) {
      mju_error("`fov[1]` must be a float between (0, 90] degrees");
      return nullptr;
    }

    // gamma
    mjtNum gamma = strtod(mj_getPluginConfig(m, instance, "gamma"), nullptr);
    if (gamma < 0 || gamma > 1) {
      mju_error("`gamma` must be a nonnegative float between [0, 1]");
      return nullptr;
    }

    return new TouchStress(m, d, instance, nchannel, size.data(), fov.data(),
                         gamma);
  } else {
    mju_error("Invalid or missing parameters in touch_grid sensor plugin");
    return nullptr;
  }
}

TouchStress::TouchStress(const mjModel* m, mjData* d, int instance,
                         int nchannel, int size[2], mjtNum fov[2], mjtNum gamma)
    : nchannel_(nchannel),
      size_{size[0], size[1]},
      fov_{fov[0], fov[1]},
      gamma_(gamma) {
  // Make sure sensor is attached to a site.
  for (int i = 0; i < m->nsensor; ++i) {
    if (m->sensor_type[i] == mjSENS_PLUGIN && m->sensor_plugin[i] == instance) {
      if (m->sensor_objtype[i] != mjOBJ_SITE) {
        mju_error("Touch Grid sensor must be attached to a site");
      }
    }
  }

  // Get sensor id.
  for (id_ = 0; id_ < m->nsensor; ++id_) {
    if (m->sensor_type[id_] == mjSENS_PLUGIN &&
        m->sensor_plugin[id_] == instance) {
      break;
    }
  }

  // Get parent weld id.
  int site_id = m->sensor_objid[id_];
  int parent_body = m->body_weldid[m->site_bodyid[site_id]];
  parent_weld_ = m->body_weldid[parent_body];

  // Get geom id.
  if (m->body_geomnum[parent_body] != 1) {
    mju_error("Touch sensor must be attached to a body with exactly one geom");
  }
  geom_id_ = m->body_geomadr[parent_body];

  // Create bin edges.
  x_edges_.assign(size[0] + 1, 0);
  y_edges_.assign(size[1] + 1, 0);
  BinEdges(x_edges_.data(), y_edges_.data(), size_, fov_, gamma_);
  dist_.resize(size[0]*size[1], 0);
  pos_.resize(3*size[0]*size[1], 0);
  mat_.resize(9*size[0]*size[1], 0);

  // Precompute spherical coordinates.
  for (int i = 0; i < size[0]; i++) {
    for (int j = 0; j < size[1]; j++) {
      mjtNum aer[3];
      aer[0] = 0.5*(x_edges_[i+1]+x_edges_[i]);
      aer[1] = 0.5*(y_edges_[j+1]+y_edges_[j]);
      aer[2] = m->geom_size[3*geom_id_];
      SphericalToCartesian(aer, pos_.data() + 3 * (i * size[1] + j));
      dist_[i*size[1]+j] = mju_abs(aer[2]);
      TangentFrame(aer, mat_.data() + 9 * (i * size[1] + j));
    }
  }
}

void TouchStress::Reset(const mjModel* m, int instance) {}

void TouchStress::Compute(const mjModel* m, mjData* d, int instance) {
  mj_markStack(d);

  // Clear sensordata and distance matrix.
  mjtNum* sensordata = d->sensordata + m->sensor_adr[id_];
  mju_zero(sensordata, m->sensor_dim[id_]);

  // Get site id.
  int site_id = m->sensor_objid[id_];

  // Count contacts and get contact geom ids.
  std::unordered_set<int> contact_geom_ids;
  for (int i = 0; i < d->ncon; i++) {
    int body1 = m->body_weldid[m->geom_bodyid[d->contact[i].geom1]];
    int body2 = m->body_weldid[m->geom_bodyid[d->contact[i].geom2]];
    if (body1 == parent_weld_) {
      contact_geom_ids.insert(d->contact[i].geom2);
    }
    if (body2 == parent_weld_) {
      contact_geom_ids.insert(d->contact[i].geom1);
    }
  }

  // No contacts, return.
  if (contact_geom_ids.empty()) {
    mj_freeStack(d);
    return;
  }

  // All of the quadrature points are contact points.
  int ncon = size_[0]*size_[1];

  // Get site frame.
  mjtNum* site_pos = d->site_xpos + 3*site_id;
  mjtNum* site_mat = d->site_xmat + 9*site_id;

  // Allocate contact forces and positions.
  mjtNum* forces = mj_stackAllocNum(d, ncon*3);
  mjtNum* forcesT = mj_stackAllocNum(d, ncon*3);

  // Iterate over colliding geoms.
  for (auto geom : contact_geom_ids) {
    int body = m->geom_bodyid[geom];

    // Get sdf plugin of the geoms.
    int sdf_instance[2] = {-1, geom_id_};
    mjtGeom geomtype[2] = {mjGEOM_SDF, mjGEOM_SPHERE};
    const mjpPlugin* sdf_ptr[2] = {NULL, NULL};
    if (m->geom_type[geom] == mjGEOM_SDF) {
      sdf_instance[0] = m->geom_plugin[geom];
      sdf_ptr[0] = mjc_getSDF(m, geom);
    } else {
      sdf_instance[0] = geom;
      geomtype[0] = (mjtGeom)m->geom_type[geom];
    }

    // Set SDF parameters.
    mjSDF geom_sdf;
    geom_sdf.id = &sdf_instance[0];
    geom_sdf.type = mjSDFTYPE_SINGLE;
    geom_sdf.plugin = &sdf_ptr[0];
    geom_sdf.geomtype = &geomtype[0];

    mjSDF sensor_sdf;
    sensor_sdf.id = &sdf_instance[1];
    sensor_sdf.type = mjSDFTYPE_SINGLE;
    sensor_sdf.plugin = &sdf_ptr[1];
    sensor_sdf.geomtype = &geomtype[1];

    // Get forces and positions in spherical coordinates.
    int node = 0;
    for (int j = 0; j < size_[1]; j++) {
      for (int i = 0; i < size_[0]; i++) {
        // Position in site frame.
        mjtNum* pos = pos_.data() + 3*(i*size_[1] + j);
        mjtNum* mat = mat_.data() + 9*(i*size_[1] + j);

        // Position in global frame.
        mjtNum xpos[3];
        mju_mulMatVec3(xpos, site_mat, pos);
        mju_addTo3(xpos, site_pos);

        // Position in other geom frame.
        mjtNum lpos[3], tmp[3];
        mju_sub3(tmp, xpos, d->geom_xpos + 3*geom);
        mju_mulMatTVec3(lpos, d->geom_xmat + 9*geom, tmp);

        // Add mesh position if needed.
        if (m->geom_type[geom] == mjGEOM_MESH ||
            m->geom_type[geom] == mjGEOM_SDF) {
          mjtNum mesh_mat[9];
          mju_quat2Mat(mesh_mat, m->mesh_quat + 4 * m->geom_dataid[geom]);
          mju_mulMatVec3(lpos, mesh_mat, lpos);
          mju_addTo3(lpos, m->mesh_pos + 3 * m->geom_dataid[geom]);
        }

        // Compute distance.
        mjtNum depth = mju_min(mjc_distance(m, d, &geom_sdf, lpos), 0);
        if (depth == 0) {
          mju_zero3(forces + 3*node);
          node++;
          continue;
        }

        // Get velocity in global frame.
        mjtNum vel_sensor[6], vel_other[6], vel_rel[3];
        mju_transformSpatial(
            vel_sensor, d->cvel + 6 * parent_weld_, 0, xpos,
            d->subtree_com + 3 * m->body_rootid[parent_weld_], NULL);
        mju_transformSpatial(
            vel_other, d->cvel + 6 * body, 0, d->geom_xpos + 3 * geom,
            d->subtree_com + 3 * m->body_rootid[body], NULL);
        mju_sub3(vel_rel, vel_sensor+3, vel_other+3);

        // Get contact force/torque, rotate into node frame.
        mjtNum tmp_force[3], normal[3];
        mjtNum kMaxDepth = 0.05;
        mjtNum pressure = 1 / (kMaxDepth - depth) - 1 / kMaxDepth;
        mjc_gradient(m, d, &sensor_sdf, normal, pos);
        mju_scl3(tmp_force, normal, pressure);
        mju_mulMatTVec3(forces + 3*node, mat, tmp_force);
        forces[3*node+0] = mju_abs(mju_dot3(vel_rel, mat + 0));
        forces[3*node+1] = mju_abs(mju_dot3(vel_rel, mat + 3));

        // Permute forces from x,y,z to z,x,y (normal, tangent, tangent)
        xyz2zxy(forces + 3*node);
        node++;
      }
    }

    // Transpose forces.
    mju_transpose(forcesT, forces, ncon, 3);

    // Compute sensor output.
    for (int c = 0; c < nchannel_; c++) {
      if (!mju_isZero(forcesT + c*ncon, ncon)) {
        mju_addTo(sensordata + c*ncon, forcesT + c*ncon, size_[0]*size_[1]);
      }
    }
  }

  mj_freeStack(d);
}

// Thickness of taxel-visualization boxes relative to contact distance.
static const mjtNum kRelativeThickness = 0.02;

void TouchStress::Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                             mjvScene* scn, int instance) {
  mj_markStack(d);

  // Get sensor data.
  mjtNum* sensordata = d->sensordata + m->sensor_adr[id_];

  // Get maximum absolute normal force.
  mjtNum maxval = 0;
  int frame = size_[0]*size_[1];
  for (int j=0; j < frame; j++) {
    maxval = mju_max(maxval, mju_abs(sensordata[j]));
  }

  // If no normal force readings, quick return.
  if (!maxval) {
    mj_freeStack(d);
    return;
  }

  // Get site id and frame.
  int site_id = m->sensor_objid[id_];
  mjtNum* site_pos = d->site_xpos + 3*site_id;
  mjtNum* site_mat = d->site_xmat + 9*site_id;
  mjtNum site_quat[4];
  mju_mat2Quat(site_quat, site_mat);

  // Draw geoms.
  for (int i=0; i < size_[0]; i++) {
    for (int j=0; j < size_[1]; j++) {
      mjtNum dist = dist_[i*size_[1]+j];
      if (!dist) {
        continue;
      }
      if (scn->ngeom >= scn->maxgeom) {
        mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);
        mj_freeStack(d);
        return;
      } else {
        // size
        mjtNum size[3];
        size[0] = dist*0.5*(x_edges_[i+1]-x_edges_[i]);
        size[1] = dist*0.5*(y_edges_[j+1]-y_edges_[j]);
        size[2] = dist*kRelativeThickness;

        // position
        mjtNum pos[3];
        mjtNum aer[3];
        aer[0] = 0.5*(x_edges_[i+1]+x_edges_[i]);
        aer[1] = 0.5*(y_edges_[j+1]+y_edges_[j]);
        aer[2] = dist*(1-kRelativeThickness);
        SphericalToCartesian(aer, pos);
        mju_mulMatVec3(pos, site_mat, pos);
        mju_addTo3(pos, site_pos);

        // orientation
        mjtNum a_quat[4];
        mjtNum site_y[3] = {-site_mat[1], -site_mat[4], -site_mat[7]};
        mju_axisAngle2Quat(a_quat, site_y, aer[0]);
        mjtNum e_quat[4];
        mjtNum site_x[3] = {site_mat[0], site_mat[3], site_mat[6]};
        mju_axisAngle2Quat(e_quat, site_x, aer[1]);
        mjtNum quat[4];
        mju_mulQuat(quat, e_quat, site_quat);
        mju_mulQuat(quat, a_quat, quat);
        mjtNum mat[9];
        mju_quat2Mat(mat, quat);

        // color
        float rgba[4] = {1, 1, 1, 1.0};
        for (int k=0; k < mjMIN(nchannel_, 3); k++) {
          rgba[k] = mju_abs(sensordata[k*frame + j*size_[0] + i]) / maxval;
        }

        // draw box geom
        mjvGeom* thisgeom = scn->geoms + scn->ngeom;
        mjv_initGeom(thisgeom, mjGEOM_BOX, size, pos, mat, rgba);
        thisgeom->objtype = mjOBJ_UNKNOWN;
        thisgeom->objid = id_;
        thisgeom->category = mjCAT_DECOR;
        thisgeom->segid = scn->ngeom;
        scn->ngeom++;
      }
    }
  }

  mj_freeStack(d);
}


void TouchStress::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sensor.touch_stress";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  // Parameterized by 4 attributes.
  const char* attributes[] = {"nchannel", "size", "fov", "gamma"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  // Stateless.
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  // Sensor dimension = nchannel * size[0] * size[1]
  plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) {
    int nchannel = strtod(mj_getPluginConfig(m, instance, "nchannel"), nullptr);
    if (!nchannel) nchannel = 1;
    std::vector<int> size;
    std::string size_str = std::string(mj_getPluginConfig(m, instance, "size"));
    ReadVector(size, size_str.c_str());
    return nchannel * size[0] * size[1];
  };

  // Can only run after forces have been computed.
  plugin.needstage = mjSTAGE_ACC;

  // Initialization callback.
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* TouchStress = TouchStress::Create(m, d, instance);
    if (!TouchStress) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(TouchStress);
    return 0;
  };

  // Destruction callback.
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<TouchStress*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // Reset callback.
  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                     int instance) {
    auto* TouchStress = reinterpret_cast<class TouchStress*>(plugin_data);
    TouchStress->Reset(m, instance);
  };

  // Compute callback.
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* TouchStress =
            reinterpret_cast<class TouchStress*>(d->plugin_data[instance]);
        TouchStress->Compute(m, d, instance);
      };

  // Visualization callback.
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                         mjvScene* scn, int instance) {
    auto* TouchStress =
        reinterpret_cast<class TouchStress*>(d->plugin_data[instance]);
    TouchStress->Visualize(m, d, opt, scn, instance);
  };

  // Register the plugin.
  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sensor
