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

#ifndef MUJOCO_PLUGIN_SDF_GEAR_H_
#define MUJOCO_PLUGIN_SDF_GEAR_H_

#include <optional>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

struct GearAttribute {
  static constexpr int nattribute = 5;
  static constexpr char const* names[nattribute] = {"alpha", "diameter",
                                                    "teeth", "thickness",
                                                    "innerdiameter"};
  static constexpr mjtNum defaults[nattribute] = { 0, 2.8, 25, .2 , -1};
};

class Gear {
 public:
  // Creates a new Gear instance (allocated with `new`) or
  // returns null on failure.
  static std::optional<Gear> Create(const mjModel* m, mjData* d, int instance);
  Gear(Gear&&) = default;
  ~Gear() = default;

  void Reset();
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

  mjtNum attribute[GearAttribute::nattribute];

 private:
  Gear(const mjModel* m, mjData* d, int instance);

  SdfVisualizer visualizer_;
};

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_GEAR_H_
