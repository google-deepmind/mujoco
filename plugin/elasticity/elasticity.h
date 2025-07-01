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

#ifndef MUJOCO_PLUGIN_ELASTICITY_ELASTICITY_H_
#define MUJOCO_PLUGIN_ELASTICITY_ELASTICITY_H_

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::elasticity {

struct PairHash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

struct Stencil2D {
  static constexpr int kNumEdges = 3;
  static constexpr int kNumVerts = 3;
  static constexpr int kNumFaces = 2;
  static constexpr int edge[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  static constexpr int face[kNumVerts][2] = {{1, 2}, {2, 0}, {0, 1}};
  static constexpr int edge2face[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

struct Stencil3D {
  static constexpr int kNumEdges = 6;
  static constexpr int kNumVerts = 4;
  static constexpr int kNumFaces = 3;
  static constexpr int edge[kNumEdges][2] = {{0, 1}, {1, 2}, {2, 0},
                                             {2, 3}, {0, 3}, {1, 3}};
  static constexpr int face[kNumVerts][3] = {{2, 1, 0}, {0, 1, 3},
                                             {1, 2, 3}, {2, 0, 3}};
  static constexpr int edge2face[kNumEdges][2] = {{2, 3}, {1, 3}, {2, 1},
                                                  {1, 0}, {0, 2}, {0, 3}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

// copied from mjXUtil
void String2Vector(const std::string& txt, std::vector<int>& vec);

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance);

}  // namespace mujoco::plugin::elasticity

#endif  // MUJOCO_PLUGIN_ELASTICITY_ELASTICITY_H_
