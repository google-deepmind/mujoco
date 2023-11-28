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

inline void UpdateSquaredLengthsFlex(std::vector<mjtNum>& len,
                                     const mjtNum* flexedge_length) {
  for (int e = 0; e < len.size(); e++) {
    len[e] = flexedge_length[e]*flexedge_length[e];
  }
}

struct Stencil2D {
  static constexpr int kNumEdges = 3;
  static constexpr int kNumVerts = 3;
  static constexpr int edge[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

struct Stencil3D {
  static constexpr int kNumEdges = 6;
  static constexpr int kNumVerts = 4;
  static constexpr int edge[kNumEdges][2] = {{0, 1}, {1, 2}, {2, 0},
                                             {2, 3}, {0, 3}, {1, 3}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

// gradients of edge lengths with respect to vertex positions
template <typename T>
void inline GradSquaredLengths(mjtNum gradient[T::kNumEdges][2][3],
                               const mjtNum* x,
                               const int v[T::kNumVerts]) {
  for (int e = 0; e < T::kNumEdges; e++) {
    for (int d = 0; d < 3; d++) {
      gradient[e][0][d] = x[3*v[T::edge[e][0]]+d] - x[3*v[T::edge[e][1]]+d];
      gradient[e][1][d] = x[3*v[T::edge[e][1]]+d] - x[3*v[T::edge[e][0]]+d];
    }
  }
}

template <typename T>
inline void ComputeForce(mjtNum* qfrc_passive,
                         const std::vector<T>& elements,
                         const std::vector<mjtNum>& metric,
                         const std::vector<mjtNum>& elongationglob,
                         const mjModel* m,
                         const int* vertbodyid,
                         const mjtNum* xpos) {
  for (int t = 0; t < elements.size(); t++)  {
    const int* v = elements[t].vertices;

    // compute length gradient with respect to dofs
    mjtNum gradient[T::kNumEdges][2][3];
    GradSquaredLengths<T>(gradient, xpos, v);

    // extract elongation of edges belonging to this element
    mjtNum elongation[T::kNumEdges];
    for (int e = 0; e < T::kNumEdges; e++) {
      int idx = elements[t].edges[e];
      elongation[e] = elongationglob[idx];
    }

    // we now multiply the elongations by the precomputed metric tensor,
    // notice that if metric=diag(1/reference) then this would yield a
    // mass-spring model

    // compute local force
    mjtNum force[T::kNumVerts*3] = {0};
    int offset = T::kNumEdges*T::kNumEdges;
    for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
      for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
        for (int i = 0; i < 2; i++) {
          for (int x = 0; x < 3; x++) {
            force[3 * T::edge[ed2][i] + x] +=
                elongation[ed1] * gradient[ed2][i][x] *
                metric[offset * t + T::kNumEdges * ed1 + ed2];
          }
        }
      }
    }

    // insert into global force
    for (int i = 0; i < T::kNumVerts; i++) {
      int body_dofnum = 3;
      int body_dofadr = 3*v[i];
      if (vertbodyid) {
        body_dofnum = m->body_dofnum[vertbodyid[v[i]]];
        body_dofadr = m->body_dofadr[vertbodyid[v[i]]];
      }
      for (int x = 0; x < body_dofnum; x++) {
        qfrc_passive[body_dofadr+x] -= force[3*i+x];
      }
    }
  }
}

// compute metric tensor of edge lengths inner product
template <typename T>
void inline MetricTensor(std::vector<mjtNum>& metric, int idx, mjtNum mu,
                         mjtNum la, const mjtNum basis[T::kNumEdges][9]) {
  mjtNum trE[T::kNumEdges] = {0};
  mjtNum trEE[T::kNumEdges*T::kNumEdges] = {0};

  // compute first invariant i.e. trace(strain)
  for (int e = 0; e < T::kNumEdges; e++) {
    for (int i = 0; i < 3; i++) {
      trE[e] += basis[e][4*i];
    }
  }

  // compute second invariant i.e. trace(strain^2)
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          trEE[T::kNumEdges*ed1+ed2] += basis[ed1][3*i+j] * basis[ed2][3*j+i];
        }
      }
    }
  }

  // assembly of strain metric tensor
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
      int index = T::kNumEdges*T::kNumEdges*idx + T::kNumEdges*ed1 + ed2;
      metric[index] = mu * trEE[T::kNumEdges * ed1 + ed2] +
                      la * trE[ed2] * trE[ed1];
    }
  }
}

// convert from Flex connectivity to stencils
template <typename T>
int CreateStencils(std::vector<T>& elements,
                   std::vector<std::pair<int, int>>& edges,
                   const std::vector<int>& simplex,
                   const std::vector<int>& edgeidx);

// copied from mjXUtil
void String2Vector(const std::string& txt, std::vector<int>& vec);

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance);

}  // namespace mujoco::plugin::elasticity

#endif  // MUJOCO_PLUGIN_ELASTICITY_ELASTICITY_H_
