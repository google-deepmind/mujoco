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

#include "elasticity.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::elasticity {

template <typename T>
int CreateStencils(std::vector<T>& elements,
                   std::vector<std::pair<int, int>>& edges,
                   const std::vector<int>& simplex,
                   const std::vector<int>& edgeidx) {
  int ne = 0;
  int nt = simplex.size() / T::kNumVerts;
  elements.resize(nt);
  for (int t = 0; t < nt; t++) {
    for (int v = 0; v < T::kNumVerts; v++) {
      elements[t].vertices[v] = simplex[T::kNumVerts*t+v];
    }
  }

  // map from edge vertices to their index in `edges` vector
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_indices;

  // loop over all tetrahedra
  for (int t = 0; t < nt; t++) {
    int* v = elements[t].vertices;

    // compute edges to vertices map for fast computations
    for (int e = 0; e < T::kNumEdges; e++) {
      auto pair = std::pair(
        std::min(v[T::edge[e][0]], v[T::edge[e][1]]),
        std::max(v[T::edge[e][0]], v[T::edge[e][1]])
      );

      // if edge is already present in the vector only store its index
      auto [it, inserted] = edge_indices.insert({pair, ne});

      if (inserted) {
        edges.push_back(pair);
        elements[t].edges[e] = ne++;
      } else {
        elements[t].edges[e] = it->second;
      }

      if (!edgeidx.empty()) {  // SHOULD NOT OCCUR
        if (elements[t].edges[e] != edgeidx[T::kNumEdges*t+e]) {
          mju_error("edge ordering is incoherent between flex and plugin");
        }
      }
    }
  }

  return nt;
}

template int CreateStencils<Stencil2D>(std::vector<Stencil2D>& elements,
                                       std::vector<std::pair<int, int>>& edges,
                                       const std::vector<int>& simplex,
                                       const std::vector<int>& edgeidx);

template int CreateStencils<Stencil3D>(std::vector<Stencil3D>& elements,
                                       std::vector<std::pair<int, int>>& edges,
                                       const std::vector<int>& simplex,
                                       const std::vector<int>& edgeidx);

void String2Vector(const std::string& txt, std::vector<int>& vec) {
  std::stringstream strm(txt);
  vec.clear();

  while (!strm.eof()) {
    int num;
    strm >> num;
    if (strm.fail()) {
      break;
    } else {
      vec.push_back(num);
    }
  }
}

bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char* end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

}  // namespace mujoco::plugin::elasticity
