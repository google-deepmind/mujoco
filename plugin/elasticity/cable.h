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

#ifndef MUJOCO_SRC_PLUGIN_ELASTICITY_CABLE_H_
#define MUJOCO_SRC_PLUGIN_ELASTICITY_CABLE_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>


namespace mujoco::plugin::elasticity {

class Cable {
 public:
  // Creates a new Cable instance (allocated with `new`) or
  // returns null on failure.
  static std::optional<Cable> Create(const mjModel* m, mjData* d, int instance);
  Cable(Cable&&) = default;
  ~Cable() = default;

  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, mjvScene* scn, int instance);

  static void RegisterPlugin();

  int i0;                         // index of first body
  int n;                          // number of bodies in the cable
  std::vector<int> prev;          // indices of previous bodies   (n x 1)
  std::vector<int> next;          // indices of next bodies       (n x 1)
  std::vector<mjtNum> stiffness;  // stiffness parameters         (n x 4)
  std::vector<mjtNum> omega0;     // reference curvature          (n x 3)
  std::vector<mjtNum> stress;     // mechanical stress            (n x 3)
  mjtNum vmax;                    // max value in colormap

 private:
  Cable(const mjModel* m, mjData* d, int instance);
};

}  // namespace mujoco::plugin::elasticity

#endif  // MUJOCO_SRC_PLUGIN_ELASTICITY_CABLE_H_
