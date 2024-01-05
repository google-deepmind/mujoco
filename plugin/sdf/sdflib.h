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

#ifndef MUJOCO_PLUGIN_SDF_SDFLIB_H_
#define MUJOCO_PLUGIN_SDF_SDFLIB_H_

#include <optional>

#include <SdfLib/utils/Mesh.h>
#include <SdfLib/OctreeSdf.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {
class SdfLib {
 public:
  // Creates a new SdfLib instance or returns null on failure.
  static std::optional<SdfLib> Create(const mjModel* m, mjData* d,
                                      int instance);
  SdfLib(SdfLib&&) = default;
  ~SdfLib() = default;

  void Reset();
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

 private:
  SdfLib(sdflib::Mesh&& mesh);
  SdfVisualizer visualizer_;
  sdflib::OctreeSdf sdf_func_;
};

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_SDFLIB_H_
