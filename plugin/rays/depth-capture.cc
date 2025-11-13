// Copyright 2025 DeepMind Technologies Limited
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

#include "depth-capture.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::rays {

namespace {

// Helper function to parse configuration attributes
bool CheckAttr(const std::string& input) {
  if (input.empty()) return false;
  char* end;
  strtod(input.c_str(), &end);
  return end == input.data() + input.size();
}

}  // namespace

// Private constructor
DepthCapture::DepthCapture(int ncol, int nrow, mjtNum fov_x, mjtNum fov_y,
                           mjtNum max_distance)
    : ncol_(ncol),
      nrow_(nrow),
      fov_x_(fov_x),
      fov_y_(fov_y),
      max_distance_(max_distance) {
  // Pre-compute ray directions in sensor frame
  // Sensor frame: +X is right, +Y is up, -Z is forward
  int nrays = ncol * nrow;
  ray_directions_.resize(nrays * 3);

  // Convert FOV from degrees to radians
  mjtNum fov_x_rad = fov_x * mjPI / 180.0;
  mjtNum fov_y_rad = fov_y * mjPI / 180.0;

  // Generate rays in a grid pattern
  for (int row = 0; row < nrow; ++row) {
    for (int col = 0; col < ncol; ++col) {
      int idx = row * ncol + col;

      // Normalized coordinates in [-1, 1]
      mjtNum u = (ncol > 1) ? (2.0 * col / (ncol - 1) - 1.0) : 0.0;
      mjtNum v = (nrow > 1) ? (2.0 * row / (nrow - 1) - 1.0) : 0.0;

      // Convert to angles
      mjtNum angle_x = u * fov_x_rad / 2.0;
      mjtNum angle_y = v * fov_y_rad / 2.0;

      // Ray direction in sensor frame (looking down -Z)
      mjtNum* dir = &ray_directions_[idx * 3];
      dir[0] = std::tan(angle_x);       // X component
      dir[1] = std::tan(angle_y);       // Y component
      dir[2] = -1.0;                     // Z component (pointing forward)

      // Normalize the direction vector
      mjtNum len = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
      dir[0] /= len;
      dir[1] /= len;
      dir[2] /= len;
    }
  }
}

// Factory method
std::optional<DepthCapture> DepthCapture::Create(const mjModel* m, mjData* d,
                                                  int instance) {
  // Read configuration attributes
  const char* ncol_str = mj_getPluginConfig(m, instance, "ncol");
  const char* nrow_str = mj_getPluginConfig(m, instance, "nrow");
  const char* fov_x_str = mj_getPluginConfig(m, instance, "fov_x");
  const char* fov_y_str = mj_getPluginConfig(m, instance, "fov_y");
  const char* max_distance_str = mj_getPluginConfig(m, instance, "max_distance");

  // Parse ncol (default: 10)
  int ncol = 10;
  if (ncol_str && ncol_str[0]) {
    if (!CheckAttr(ncol_str)) {
      mju_error("Invalid 'ncol' attribute in depth-capture plugin");
      return std::nullopt;
    }
    ncol = std::atoi(ncol_str);
    if (ncol <= 0) {
      mju_error("'ncol' must be positive");
      return std::nullopt;
    }
  }

  // Parse nrow (default: 10)
  int nrow = 10;
  if (nrow_str && nrow_str[0]) {
    if (!CheckAttr(nrow_str)) {
      mju_error("Invalid 'nrow' attribute in depth-capture plugin");
      return std::nullopt;
    }
    nrow = std::atoi(nrow_str);
    if (nrow <= 0) {
      mju_error("'nrow' must be positive");
      return std::nullopt;
    }
  }

  // Parse fov_x (default: 45 degrees)
  mjtNum fov_x = 45.0;
  if (fov_x_str && fov_x_str[0]) {
    if (!CheckAttr(fov_x_str)) {
      mju_error("Invalid 'fov_x' attribute in depth-capture plugin");
      return std::nullopt;
    }
    fov_x = std::strtod(fov_x_str, nullptr);
    if (fov_x <= 0 || fov_x > 180) {
      mju_error("'fov_x' must be in range (0, 180] degrees");
      return std::nullopt;
    }
  }

  // Parse fov_y (default: 45 degrees)
  mjtNum fov_y = 45.0;
  if (fov_y_str && fov_y_str[0]) {
    if (!CheckAttr(fov_y_str)) {
      mju_error("Invalid 'fov_y' attribute in depth-capture plugin");
      return std::nullopt;
    }
    fov_y = std::strtod(fov_y_str, nullptr);
    if (fov_y <= 0 || fov_y > 180) {
      mju_error("'fov_y' must be in range (0, 180] degrees");
      return std::nullopt;
    }
  }

  // Parse max_distance (default: 10.0)
  mjtNum max_distance = 10.0;
  if (max_distance_str && max_distance_str[0]) {
    if (!CheckAttr(max_distance_str)) {
      mju_error("Invalid 'max_distance' attribute in depth-capture plugin");
      return std::nullopt;
    }
    max_distance = std::strtod(max_distance_str, nullptr);
    if (max_distance <= 0) {
      mju_error("'max_distance' must be positive");
      return std::nullopt;
    }
  }

  return DepthCapture(ncol, nrow, fov_x, fov_y, max_distance);
}

void DepthCapture::Reset() {
  // No state to reset
}

void DepthCapture::Compute(const mjModel* m, mjData* d, int instance) {
  // Find the sensor associated with this plugin instance
  int sensor_id = -1;
  for (int i = 0; i < m->nsensor; ++i) {
    if (m->sensor_type[i] == mjSENS_PLUGIN &&
        m->sensor_plugin[i] == instance) {
      sensor_id = i;
      break;
    }
  }

  if (sensor_id == -1) {
    mju_error("Could not find sensor for depth-capture plugin instance");
    return;
  }

  // Get sensor output pointer
  mjtNum* sensordata = d->sensordata + m->sensor_adr[sensor_id];
  int nrays = ncol_ * nrow_;

  // Get the site associated with this sensor
  int site_id = m->sensor_objid[sensor_id];

  // Get site position and orientation in world frame
  mjtNum* site_pos = d->site_xpos + 3 * site_id;
  mjtNum* site_mat = d->site_xmat + 9 * site_id;

  // Perform raycasting for each ray
  for (int i = 0; i < nrays; ++i) {
    // Get ray direction in sensor frame
    const mjtNum* dir_local = &ray_directions_[i * 3];

    // Transform ray direction to world frame
    mjtNum dir_world[3];
    mju_mulMatVec(dir_world, site_mat, dir_local, 3, 3);

    // Perform raycast
    int geomid = -1;
    mjtNum dist = mj_ray(m, d, site_pos, dir_world, nullptr, 1, -1, &geomid);

    // Store the distance (or max_distance if no hit)
    if (dist < 0 || dist > max_distance_) {
      sensordata[i] = max_distance_;
    } else {
      sensordata[i] = dist;
    }
  }
}

void DepthCapture::Visualize(const mjModel* m, mjData* d,
                             const mjvOption* opt, mjvScene* scn,
                             int instance) {
  // Optional: Visualize rays for debugging
  // This could be implemented to show the raycast directions and hits
  // For now, we leave it empty
}

// Plugin registration
void DepthCapture::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.rays.depth_capture";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  // Configuration attributes
  const char* attributes[] = {"ncol", "nrow", "fov_x", "fov_y", "max_distance"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  // Sensor should be computed after positions are updated
  plugin.needstage = mjSTAGE_POS;

  // No state needed
  plugin.nstate = +[](const mjModel* m, int instance) -> int {
    return 0;
  };

  // Sensor output dimension is ncol * nrow
  plugin.nsensordata = +[](const mjModel* m, int instance,
                            int sensor_id) -> int {
    const char* ncol_str = mj_getPluginConfig(m, instance, "ncol");
    const char* nrow_str = mj_getPluginConfig(m, instance, "nrow");

    int ncol = (ncol_str && ncol_str[0]) ? std::atoi(ncol_str) : 10;
    int nrow = (nrow_str && nrow_str[0]) ? std::atoi(nrow_str) : 10;

    return ncol * nrow;
  };

  // Initialize plugin instance
  plugin.init = +[](const mjModel* m, mjData* d, int instance) -> int {
    auto depthcapture_opt = DepthCapture::Create(m, d, instance);
    if (!depthcapture_opt.has_value()) {
      return -1;
    }

    // Check that sensor is attached to a site
    for (int i = 0; i < m->nsensor; ++i) {
      if (m->sensor_type[i] == mjSENS_PLUGIN &&
          m->sensor_plugin[i] == instance) {
        if (m->sensor_objtype[i] != mjOBJ_SITE) {
          mju_error("DepthCapture sensor must be attached to a site");
          return -1;
        }
      }
    }

    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(new DepthCapture(std::move(*depthcapture_opt)));
    return 0;
  };

  // Destroy plugin instance
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<DepthCapture*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // Reset plugin state
  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state,
                      void* plugin_data, int instance) {
    auto* depthcapture = reinterpret_cast<DepthCapture*>(plugin_data);
    depthcapture->Reset();
  };

  // Compute sensor output
  plugin.compute = +[](const mjModel* m, mjData* d, int instance,
                        int capability_bit) {
    auto* depthcapture = reinterpret_cast<DepthCapture*>(d->plugin_data[instance]);
    depthcapture->Compute(m, d, instance);
  };

  // Optional: visualize
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                          mjvScene* scn, int instance) {
    auto* depthcapture = reinterpret_cast<DepthCapture*>(d->plugin_data[instance]);
    depthcapture->Visualize(m, d, opt, scn, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::rays
