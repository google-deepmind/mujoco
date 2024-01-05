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

#ifndef MUJOCO_PLUGIN_SDF_BOLT_H_
#define MUJOCO_PLUGIN_SDF_BOLT_H_

#include <optional>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

// this plugin implements a modification of the signed distance function
// from https://www.shadertoy.com/view/XtffzX  of a bolt with a hexagonal head

struct BoltAttribute {
  static constexpr int nattribute = 1;
  static constexpr char const* names[nattribute] = {"radius"};
  static constexpr mjtNum defaults[nattribute] = {0.26};
};

class Bolt {
 public:
  // Creates a new Bolt instance (allocated with `new`) or
  // returns null on failure.
  static std::optional<Bolt> Create(const mjModel* m, mjData* d, int instance);
  Bolt(Bolt&&) = default;
  ~Bolt() = default;

  void Reset();
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

  mjtNum attribute[BoltAttribute::nattribute];

 private:
  Bolt(const mjModel* m, mjData* d, int instance);

  SdfVisualizer visualizer_;
};

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_BOLT_H_
