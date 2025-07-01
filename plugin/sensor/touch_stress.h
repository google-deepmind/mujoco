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

#ifndef MUJOCO_PLUGIN_SENSOR_TOUCH_STRESS_H_
#define MUJOCO_PLUGIN_SENSOR_TOUCH_STRESS_H_

#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::sensor {

// A touch grid sensor is associated with a site and senses contact stresses
// between the site's parent body and all other bodies. The site's
// frame determines the orientation of the sensor with the same convention used
// for cameras and lights: the sensor points in the frame's negative-Z
// direction, so the X and Y axes correspond to horizontal and vertical
// directions respectively.
//
// The output of the sensor is a stack of 3 "touch images" corresponding to
// forces in the local frame of the taxels. Forces are in the [z, x, y] order,
// corresponding to the ordering in contact frames: [normal, tangent, tangent].
//
// The sensor has 6 parameters:
//  1. (int) Number of channels [1-3]. Defaults to 1.
//  2. (int) Horizontal resolution.
//  3. (int) Vertical resolution.
//  4. (float) Horizontal field-of-view (fov_x), in degrees.
//  5. (float) Vertical field-of-view (fov_y), in degrees.
//  6. (float) Foveal deformation. Defaults to 0.
class TouchStress {
 public:
  static TouchStress* Create(const mjModel* m, mjData* d, int instance);
  TouchStress(TouchStress&&) = default;
  ~TouchStress() = default;

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);

  static void RegisterPlugin();

  int nchannel_;           // number of channels (1-3)
  int size_[2];            // horizontal and vertical resolution
  mjtNum fov_[2];          // horizontal and vertical field of view, in degrees
  mjtNum gamma_;           // foveal deformation

 private:
  TouchStress(const mjModel* m, mjData* d, int instance, int nchannel,
               int* size, mjtNum* fov_x, mjtNum gamma);

  std::vector<mjtNum> x_edges_;
  std::vector<mjtNum> y_edges_;
  std::vector<mjtNum> dist_;
  std::vector<mjtNum> pos_;
  std::vector<mjtNum> mat_;

  int id_;
  int parent_weld_;
  int geom_id_;
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SENSOR_TOUCH_STRESS_H_
