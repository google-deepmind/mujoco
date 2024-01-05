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

#ifndef MUJOCO_PLUGIN_SDF_TORUS_H_
#define MUJOCO_PLUGIN_SDF_TORUS_H_

#include <optional>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

struct TorusAttribute {
  static constexpr int nattribute = 2;
  static constexpr char const* names[nattribute] = {"radius1", "radius2"};
  static constexpr mjtNum defaults[nattribute] = { .35, .15 };
};

class Torus {
 public:
  // Creates a new Torus instance or returns null on failure.
  static std::optional<Torus> Create(const mjModel* m, mjData* d, int instance);
  Torus(Torus&&) = default;
  ~Torus() = default;

  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

  mjtNum attribute[TorusAttribute::nattribute];

 private:
  Torus(const mjModel* m, mjData* d, int instance);
};

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_TORUS_H_
