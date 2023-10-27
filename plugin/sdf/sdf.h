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

#ifndef MUJOCO_PLUGIN_SDF_SDF_H_
#define MUJOCO_PLUGIN_SDF_SDF_H_

#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::sdf {

inline mjtNum Union(mjtNum a, mjtNum b) {
    return mju_min(a, b);
}

inline mjtNum Intersection(mjtNum a, mjtNum b) {
    return mju_max(a, b);
}

inline mjtNum Subtraction(mjtNum a, mjtNum b) {
    return mju_max(a, -b);
}

inline mjtNum Fract(mjtNum x) {
  return x - floor(x);
}

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance);

class SdfVisualizer {
 public:
  SdfVisualizer();

  void Visualize(const mjModel* m, const mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);

  void AddPoint(const mjtNum point[3]);
  void Reset();
  void Next();  // adds a new gradient descent trajectory to be visualized

 private:
  std::vector<mjtNum> points_;  // query points
  std::vector<int> npoints_;    // number of iterations from the starting point
};

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_SDF_H_
