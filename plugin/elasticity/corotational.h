
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

#ifndef THIRD_PARTY_MUJOCO_PLUGIN_ELASTICITY_SOLID_H_
#define THIRD_PARTY_MUJOCO_PLUGIN_ELASTICITY_SOLID_H_
#include <optional>
#include <vector>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
namespace mujoco::plugin::elasticity {
class Corotational {
 public:
  // Creates a new Corotational instance (allocated with `new`) or
  // returns null on failure.
  static std::optional<Corotational> Create(const mjModel* m, mjData* d,
                                          int instance);
  Corotational(Corotational&&) = default;
  ~Corotational() = default;
  void Compute(const mjModel* m, mjData* d, int instance);

  static void RegisterPlugin();
  
  int i0;                         // index of first body
  int n;                          // number of bodies (vertices)
  int nrows;                      // degrees of freedom
  int ncols;                      // deformation modes
  std::vector<mjtNum> K;          // stiffness matrix           (nrows x nrows)
  std::vector<mjtNum> jac;        // kinematic jacobian         (nrows x ncols)
 private:
  Corotational(const mjModel* m, mjData* d, int instance);
};
}  // namespace mujoco::plugin::elasticity
#endif  // THIRD_PARTY_MUJOCO_PLUGIN_ELASTICITY_SOLID_H_
