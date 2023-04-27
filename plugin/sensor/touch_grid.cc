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

#include "touch_grid.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::sensor {

namespace {

// Returns the index of the first value in `a` that x is less than or n if no
// such value exists. See: https://stackoverflow.com/a/39100135.
int LowerBound(const mjtNum a[], int n, mjtNum x) {
  int l = 0;
  int h = n;
  while (l < h) {
    int mid = (l + h) / 2;
    if (x <= a[mid]) {
      h = mid;
    } else {
      l = mid + 1;
    }
  }
  return l;
}

// Two dimensional histogram function.
void Histogram2D(const mjtNum x_data[], const mjtNum y_data[],
                 const mjtNum weights[], int n_data, const mjtNum x_edges[],
                 int n_x_edges, const mjtNum y_edges[], int n_y_edges,
                 mjtNum* histogram, int* counts) {
  for (int i = 0; i < n_data; ++i) {
    mjtNum x = x_data[i];
    mjtNum y = y_data[i];
    int x_idx = LowerBound(x_edges, n_x_edges, x);
    if (x_idx == 0 || x_idx == n_x_edges) {
      continue;
    }
    int y_idx = LowerBound(y_edges, n_y_edges, y);
    if (y_idx == 0 || y_idx == n_y_edges) {
      continue;
    }
    int index = (y_idx - 1)*(n_x_edges - 1) + (x_idx - 1);
    histogram[index] += weights[i];
    if (counts) {
      counts[index]++;
    }
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
void BinEdges(mjtNum* x_edges, mjtNum* y_edges, int size_x, int size_y,
              mjtNum fov_x, mjtNum fov_y, mjtNum gamma) {
  // Make unit bin edges.
  LinSpace(-1, 1, size_x + 1, x_edges);
  LinSpace(-1, 1, size_y + 1, y_edges);

  // Apply foveal deformation.
  for (int i = 0; i < size_x + 1; i++) {
    x_edges[i] = Fovea(x_edges[i], gamma);
  }
  for (int i = 0; i < size_y + 1; i++) {
    y_edges[i] = Fovea(y_edges[i], gamma);
  }

  // Scale by field-of-view.
  mju_scl(x_edges, x_edges, fov_x*mjPI / 180, size_x + 1);
  mju_scl(y_edges, y_edges, fov_y*mjPI / 180, size_y + 1);
}

// Permute 3-vector from 0,1,2 to 2,0,1.
static void xyz2zxy(mjtNum* x) {
  mjtNum z = x[2];
  x[2] = x[1];
  x[1] = x[0];
  x[0] = z;
}

// In the functions below transforming Cartesian <-> spherical:
//  - The frame points down the z-axis, so a=e=0 corresponds to (0, 0, -r).
//  - azimuth (a) corresponds to positive rotation around -y (towards +x).
//  - elevation (e) corresponds to positive rotation around +x (towards +y).

// Transform Cartesian (x,y,z) to spherical (azimuth, elevation, radius).
void CartesianToSpherical(const mjtNum xyz[3], mjtNum aer[3]) {
  mjtNum x = xyz[0], y = xyz[1], z = xyz[2];
  aer[0] = mju_atan2(x, -z);
  aer[1] = mju_atan2(y, mju_sqrt(x*x + z*z));
  aer[2] = mju_sqrt(x*x + z*z + y*y);
}

// Transform spherical (azimuth, elevation, radius) to Cartesian (x,y,z).
void SphericalToCartesian(const mjtNum aer[3], mjtNum xyz[3]) {
  mjtNum a = aer[0], e = aer[1], r = aer[2];
  xyz[0] = r * mju_cos(e) * mju_sin(a);
  xyz[1] = r * mju_sin(e);
  xyz[2] = -r * mju_cos(e) * mju_cos(a);
}

// Checks that a plugin config attribute exists.
bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char* end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

}  // namespace

// Creates a TouchGrid instance if all config attributes are defined and
// within their allowed bounds.
TouchGrid* TouchGrid::Create(const mjModel* m, mjData* d,
                                                 int instance) {
  if (CheckAttr("size_x", m, instance) && CheckAttr("size_y", m, instance) &&
      CheckAttr("fov_x", m, instance) && CheckAttr("fov_y", m, instance) &&
      CheckAttr("gamma", m, instance)) {
    // nchannel
    int nchannel = strtod(mj_getPluginConfig(m, instance, "nchannel"), nullptr);
    if (!nchannel) nchannel = 1;
    if (nchannel < 1 || nchannel > 6) {
      mju_warning("nchannel must be between 1 and 6");
      return nullptr;
    }

    // size_x, size_y
    int size_x = strtod(mj_getPluginConfig(m, instance, "size_x"), nullptr);
    int size_y = strtod(mj_getPluginConfig(m, instance, "size_y"), nullptr);
    if (size_x < 0 || size_y < 0) {
      mju_warning("Horizontal and vertical resolutions must be positive");
      return nullptr;
    }

    // fov_x, fov_y
    mjtNum fov_x = strtod(mj_getPluginConfig(m, instance, "fov_x"), nullptr);
    if (fov_x <= 0 || fov_x > 180) {
      mju_warning("`fov_x` must be a float between (0, 180] degrees");
      return nullptr;
    }
    mjtNum fov_y = strtod(mj_getPluginConfig(m, instance, "fov_y"), nullptr);
    if (fov_y <= 0 || fov_y > 90) {
      mju_warning("`fov_y` must be a float between (0, 90] degrees");
      return nullptr;
    }

    // gamma
    mjtNum gamma = strtod(mj_getPluginConfig(m, instance, "gamma"), nullptr);
    if (gamma < 0 || gamma > 1) {
      mju_warning("`gamma` must be a nonnegative float between [0, 1]");
      return nullptr;
    }

    return new TouchGrid(m, d, instance, nchannel, size_x, size_y,
                            fov_x, fov_y, gamma);
  } else {
    mju_warning("Invalid or missing parameters in touch_grid sensor plugin");
    return nullptr;
  }
}

TouchGrid::TouchGrid(const mjModel* m, mjData* d, int instance,
                           int nchannel, int size_x, int size_y, mjtNum fov_x,
                           mjtNum fov_y, mjtNum gamma)
    : nchannel_(nchannel),
      size_x_(size_x),
      size_y_(size_y),
      fov_x_(fov_x),
      fov_y_(fov_y),
      gamma_(gamma) {
  // Make sure sensor is attached to a site.
  for (int i = 0; i < m->nsensor; ++i) {
    if (m->sensor_type[i] == mjSENS_PLUGIN && m->sensor_plugin[i] == instance) {
      if (m->sensor_objtype[i] != mjOBJ_SITE) {
        mju_error("Touch Grid sensor must be attached to a site");
      }
    }
  }

  // Allocate distance array.
  distance_.resize(size_x*size_y, 0);
}

void TouchGrid::Reset(const mjModel* m, int instance) {}

void TouchGrid::Compute(const mjModel* m, mjData* d, int instance) {
  mjMARKSTACK;

  // Get sensor id.
  int id;
  for (id = 0; id < m->nsensor; ++id) {
    if (m->sensor_type[id] == mjSENS_PLUGIN &&
        m->sensor_plugin[id] == instance) {
      break;
    }
  }

  // Clear sensordata and distance matrix.
  mjtNum* sensordata = d->sensordata + m->sensor_adr[id];
  mju_zero(sensordata, m->sensor_dim[id]);
  int frame = size_x_*size_y_;
  mju_zero(distance_.data(), frame);

  // Get site id.
  int site_id = m->sensor_objid[id];

  // Count contacts.
  int ncon = 0;
  int parent_body = m->body_weldid[m->site_bodyid[site_id]];
  int parent_weld = m->body_weldid[parent_body];
  for (int i = 0; i < d->ncon; i++) {
    int body1 = m->body_weldid[m->geom_bodyid[d->contact[i].geom1]];
    int body2 = m->body_weldid[m->geom_bodyid[d->contact[i].geom2]];
    if (body1 == parent_weld || body2 == parent_weld) {
      ncon++;
    }
  }

  // No contacts, return.
  if (!ncon) {
    mjFREESTACK;
    return;
  }

  // Get site frame.
  mjtNum* site_pos = d->site_xpos + 3*site_id;
  mjtNum* site_mat = d->site_xmat + 9*site_id;

  // allocate contact forces and positions
  mjtNum* forces = mj_stackAlloc(d, ncon*6);
  mjtNum* positions = mj_stackAlloc(d, ncon*3);

  // Get forces and positions in spherical coordinates.
  int contact = 0;
  for (int i = 0; i < d->ncon; i++) {
    int body1 = m->geom_bodyid[d->contact[i].geom1];
    int weld1 = m->body_weldid[body1];
    int body2 = m->geom_bodyid[d->contact[i].geom2];
    int weld2 = m->body_weldid[body2];

    if (weld1 == parent_weld || weld2 == parent_weld) {
      // Get contact force/torque, rotate into world frame, then site frame.
      // Note that contact.frame is column major.
      mjtNum tmp_force[6], tmp1[3];
      mj_contactForce(m, d, i, tmp_force);
      mju_rotVecMatT(tmp1, tmp_force, d->contact[i].frame);
      mju_rotVecMatT(forces + 6*contact, tmp1, site_mat);
      mju_rotVecMatT(tmp1, tmp_force + 3, d->contact[i].frame);
      mju_rotVecMatT(forces + 6*contact + 3, tmp1, site_mat);

      // Forces point from the smaller to larger body, so flip sign if
      // the parent body has smaller id.
      if (parent_body < mjMAX(body1, body2)) {
        mju_scl(forces + 6*contact, forces + 6*contact, -1, 6);
      }

      // Permute forces from x,y,z to z,x,y (normal, tangent, tangent)
      xyz2zxy(forces + 6*contact);
      xyz2zxy(forces + 6*contact + 3);

      // Get position, rotate into contact frame.
      mjtNum tmp2[3];
      mju_sub3(tmp1, d->contact[i].pos, site_pos);
      mju_rotVecMatT(tmp2, tmp1, site_mat);

      // Transform to spherical coordinates, copy into positions array.
      CartesianToSpherical(tmp2, tmp1);
      for (int k = 0; k < 3; k++) {
        positions[k*ncon + contact] = tmp1[k];
      }
      contact++;
    }
  }

  // Transpose forces.
  mjtNum* forcesT = mj_stackAlloc(d, ncon*6);
  mju_transpose(forcesT, forces, ncon, 6);

  // Allocate bin edges.
  mjtNum* x_edges = mj_stackAlloc(d, size_x_ + 1);
  mjtNum* y_edges = mj_stackAlloc(d, size_y_ + 1);

  // Make bin edges.
  BinEdges(x_edges, y_edges, size_x_, size_y_, fov_x_, fov_y_, gamma_);

  // Compute sensor output.
  for (int i = 0; i < nchannel_; i++) {
    if (!mju_isZero(forcesT + i*ncon, ncon)) {
      Histogram2D(positions, positions + ncon, forcesT + i*ncon, ncon,
                  x_edges, size_x_ + 1, y_edges, size_y_ + 1,
                  sensordata + i*frame, nullptr);
    }
  }

  // Allocate count matrix.
  int* counts = mj_stackAllocInt(d, frame);
  for (int i=0; i < frame; i++) counts[i] = 0;

  // Compute distance matrix (unnormalized).
  Histogram2D(positions, positions + ncon, positions + 2*ncon, ncon, x_edges,
              size_x_ + 1, y_edges, size_y_ + 1, distance_.data(), counts);

  // Normalize distances
  for (int i=0; i < frame; i++) {
    if (counts[i]) {
      distance_.data()[i] /= counts[i];
    }
  }

  mjFREESTACK;
}

// Thickness of taxel-visualization boxes relative to contact distance.
static const mjtNum kRelativeThickness = 0.02;

void TouchGrid::Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                             mjvScene* scn, int instance) {
  mjMARKSTACK;

  // Get sensor id.
  int id;
  for (id=0; id < m->nsensor; ++id) {
    if (m->sensor_type[id] == mjSENS_PLUGIN &&
        m->sensor_plugin[id] == instance) {
      break;
    }
  }

  // Get sensor data.
  mjtNum* sensordata = d->sensordata + m->sensor_adr[id];

  // Get maximum absolute normal force.
  mjtNum maxval = 0;
  int frame = size_x_*size_y_;
  for (int j=0; j < frame; j++) {
    maxval = mju_max(maxval, mju_abs(sensordata[j]));
  }

  // If no normal force readings, quick return.
  if (!maxval) {
    mjFREESTACK;
    return;
  }

  // Get site id and frame.
  int site_id = m->sensor_objid[id];
  mjtNum* site_pos = d->site_xpos + 3*site_id;
  mjtNum* site_mat = d->site_xmat + 9*site_id;
  mjtNum site_quat[4];
  mju_mat2Quat(site_quat, site_mat);

  // Allocate bin edges.
  mjtNum* x_edges = mj_stackAlloc(d, size_x_ + 1);
  mjtNum* y_edges = mj_stackAlloc(d, size_y_ + 1);

  // Make bin edges.
  BinEdges(x_edges, y_edges, size_x_, size_y_, fov_x_, fov_y_, gamma_);

  // Draw geoms.
  for (int i=0; i < size_x_; i++) {
    for (int j=0; j < size_y_; j++) {
      mjtNum dist = distance_.data()[j*size_x_ + i];
      if (!dist) {
        continue;
      }
      if (scn->ngeom >= scn->maxgeom) {
        mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);
        mjFREESTACK;
        return;
      } else {
        // size
        mjtNum size[3] = {dist*0.5*(x_edges[i+1]-x_edges[i]),
                          dist*0.5*(y_edges[j+1]-y_edges[j]),
                          dist*kRelativeThickness};

        // position
        mjtNum pos[3];
        mjtNum aer[3] = {0.5*(x_edges[i+1]+x_edges[i]),
                         0.5*(y_edges[j+1]+y_edges[j]),
                         dist*(1-kRelativeThickness)};
        SphericalToCartesian(aer, pos);
        mju_rotVecMat(pos, pos, site_mat);
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
          rgba[k] = mju_abs(sensordata[k*frame + j*size_x_ + i]) / maxval;
        }

        // draw box geom
        mjvGeom* thisgeom = scn->geoms + scn->ngeom;
        mjv_initGeom(thisgeom, mjGEOM_BOX, size, pos, mat, rgba);
        thisgeom->objtype = mjOBJ_UNKNOWN;
        thisgeom->objid = id;
        thisgeom->category = mjCAT_DECOR;
        thisgeom->segid = scn->ngeom;
        scn->ngeom++;
      }
    }
  }

  mjFREESTACK;
}


void TouchGrid::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sensor.touch_grid";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  // Parameterized by 5 attributes.
  const char* attributes[] = {"nchannel", "size_x", "size_y", "fov_x", "fov_y",
                              "gamma"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  // Stateless.
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  // Sensor dimension = nchannel * size_x * size_y
  plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) {
    int nchannel = strtod(mj_getPluginConfig(m, instance, "nchannel"), nullptr);
    if (!nchannel) nchannel = 1;
    int size_x = strtod(mj_getPluginConfig(m, instance, "size_x"), nullptr);
    int size_y = strtod(mj_getPluginConfig(m, instance, "size_y"), nullptr);
    return nchannel * size_x * size_y;
  };

  // Can only run after forces have been computed.
  plugin.needstage = mjSTAGE_ACC;

  // Initialization callback.
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* TouchGrid = TouchGrid::Create(m, d, instance);
    if (!TouchGrid) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(TouchGrid);
    return 0;
  };

  // Destruction callback.
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<TouchGrid*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // Reset callback.
  plugin.reset = +[](const mjModel* m, double* plugin_state, void* plugin_data,
                     int instance) {
    auto* TouchGrid = reinterpret_cast<class TouchGrid*>(plugin_data);
    TouchGrid->Reset(m, instance);
  };

  // Compute callback.
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* TouchGrid =
            reinterpret_cast<class TouchGrid*>(d->plugin_data[instance]);
        TouchGrid->Compute(m, d, instance);
      };

  // Visualization callback.
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                         mjvScene* scn, int instance) {
    auto* TouchGrid =
        reinterpret_cast<class TouchGrid*>(d->plugin_data[instance]);
    TouchGrid->Visualize(m, d, opt, scn, instance);
  };

  // Register the plugin.
  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sensor
