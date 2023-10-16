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

#include <sstream>
#include <string>
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

inline mjtNum SquaredDist3(const mjtNum pos1[3], const mjtNum pos2[3]) {
  mjtNum dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  return dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2];
}

inline void UpdateSquaredLengths(std::vector<mjtNum>& len,
                                 const std::vector<std::pair<int, int> >& edges,
                                 const mjtNum* x) {
  for (int e = 0; e < len.size(); e++) {
    const mjtNum* p0 = x + 3*edges[e].first;
    const mjtNum* p1 = x + 3*edges[e].second;
    len[e] = SquaredDist3(p0, p1);
  }
}

// copied from mjXUtil
void String2Vector(const std::string& txt, std::vector<int>& vec);

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance);

}  // namespace mujoco::plugin::elasticity

#endif  // MUJOCO_PLUGIN_ELASTICITY_ELASTICITY_H_
