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

#ifndef MUJOCO_PLUGIN_RAYS_DEPTH_CAPTURE_H_
#define MUJOCO_PLUGIN_RAYS_DEPTH_CAPTURE_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::rays {

// A depth capture sensor is associated with a site and captures depth
// information by raycasting from the site's frame. The site's frame
// determines the orientation of the sensor with the same convention used
// for cameras: the sensor points in the frame's negative-Z direction,
// so the X and Y axes correspond to horizontal and vertical directions
// respectively.
//
// The sensor casts rays in a frustum-shaped pattern and returns the depth
// (distance) to the nearest surface for each ray. The output is an array
// of depth values, one for each ray.
//
// The sensor has 5 configurable parameters:
//  1. (int) ncol: Number of rays horizontally. Defaults to 10.
//  2. (int) nrow: Number of rays vertically. Defaults to 10.
//  3. (float) fov_x: Horizontal field-of-view, in degrees. Defaults to 45.
//  4. (float) fov_y: Vertical field-of-view, in degrees. Defaults to 45.
//  5. (float) max_distance: Maximum raycast distance. Defaults to 10.0.
//
// The total number of rays (and output dimension) is ncol * nrow.
class DepthCapture {
 public:
  // Factory method to create a DepthCapture instance
  static std::optional<DepthCapture> Create(const mjModel* m, mjData* d,
                                             int instance);

  DepthCapture(DepthCapture&&) = default;
  ~DepthCapture() = default;

  // Reset the sensor state
  void Reset();

  // Compute depth values by raycasting
  void Compute(const mjModel* m, mjData* d, int instance);

  // Visualize the rays (optional)
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);

  // Register this plugin with MuJoCo
  static void RegisterPlugin();

  // Sensor configuration
  int ncol_;           // number of rays horizontally
  int nrow_;           // number of rays vertically
  mjtNum fov_x_;       // horizontal field of view, in degrees
  mjtNum fov_y_;       // vertical field of view, in degrees
  mjtNum max_distance_; // maximum raycast distance

 private:
  DepthCapture(int ncol, int nrow, mjtNum fov_x, mjtNum fov_y,
               mjtNum max_distance);

  // Cached ray directions in sensor frame
  std::vector<mjtNum> ray_directions_;
};

}  // namespace mujoco::plugin::rays

#endif  // MUJOCO_PLUGIN_RAYS_DEPTH_CAPTURE_H_
