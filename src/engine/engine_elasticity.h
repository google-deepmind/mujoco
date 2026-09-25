// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_ELASTICITY_H_
#define MUJOCO_SRC_ENGINE_ENGINE_ELASTICITY_H_

#include <mujoco/mjtype.h>
#include "engine/engine_util_blas.h"
#include "engine/engine_util_misc.h"

// element-local geometry, StVK material response, and stiffness contractions shared by
// passive forces and both solver paths; keep the helpers visible to the compiler so the
// small edge loops can be optimized together with their callers

// local edges for triangles and tetrahedra, in the compiler's stiffness ordering
static const int mj_stretchEdges[2][6][2] = {
  {{1, 2}, {2, 0}, {0, 1}, {0, 0}, {0, 0}, {0, 0}},
  {{0, 1}, {1, 2}, {2, 0}, {2, 3}, {0, 3}, {1, 3}}
};


// x_i - x_j: half the squared-length gradient at vertex i
static inline void mj_stretchEdgeVectors(mjtNum edgevec[6][3], const mjtNum* xpos,
                                         const int* vert, int dim) {
  int nedge = dim == 2 ? 3 : 6;
  const int (*edge)[2] = mj_stretchEdges[dim-2];
  for (int e = 0; e < nedge; e++) {
    for (int x = 0; x < 3; x++) {
      edgevec[e][x] = xpos[3*vert[edge[e][0]]+x] - xpos[3*vert[edge[e][1]]+x];
    }
  }
}


// unpack the symmetric StVK metric; triangles also use a 21-number element stride
static inline void mj_stretchMetric(mjtNum metric[36], const mjtNum* packed, int nedge) {
  int id = 0;
  for (int a = 0; a < nedge; a++) {
    for (int b = a; b < nedge; b++) {
      metric[nedge*a + b] = packed[id];
      metric[nedge*b + a] = packed[id++];
    }
  }
}


// gather squared-length differences in local edge order
static inline void mj_stretchElongation(mjtNum elongation[6], const int* edge,
                                        const mjtNum* length, const mjtNum* reference, int nedge) {
  for (int e = 0; e < nedge; e++) {
    int idx = edge[e];
    elongation[e] = length[idx]*length[idx] - reference[idx]*reference[idx];
  }
}


// StVK material response, also used for damping and stiffness-vector products
static inline void mj_stretchTension(mjtNum tension[6], const mjtNum metric[36],
                                     const mjtNum elongation[6], int nedge) {
  for (int e = 0; e < nedge; e++) {
    tension[e] = 0;
    for (int a = 0; a < nedge; a++) {
      tension[e] += metric[nedge*e + a]*elongation[a];
    }
  }
}


// accumulate world-frame vertex forces, independently of the body Jacobians
static inline void mj_stretchForce(mjtNum* force, const int* vert, const mjtNum tension[6],
                                   const mjtNum edgevec[6][3], int dim) {
  int nedge = dim == 2 ? 3 : 6;
  const int (*edge)[2] = mj_stretchEdges[dim-2];
  for (int e = 0; e < nedge; e++) {
    for (int i = 0; i < 2; i++) {
      for (int x = 0; x < 3; x++) {
        mjtNum gradient = i == 0 ? edgevec[e][x] : -edgevec[e][x];
        force[3*vert[edge[e][i]]+x] -= tension[e]*gradient;
      }
    }
  }
}


// prepare the material and geometric coefficients shared by both solver paths
static inline void mj_stretchStiffness(mjtNum metric[36], mjtNum tension[6], const mjtNum* packed,
                                       const int* edge, const mjtNum* length,
                                       const mjtNum* reference, int nedge) {
  mj_stretchMetric(metric, packed, nedge);
  mjtNum elongation[6];
  mj_stretchElongation(elongation, edge, length, reference, nedge);
  mj_stretchTension(tension, metric, elongation, nedge);

  // a compressed edge's geometric block is negative semidefinite; keep only the tensile
  // part for the SPD solver metric, exactly as in the original StVK operator and assembler
  // passive forces retain the full tension, and the material metric is not modified
  for (int e = 0; e < nedge; e++) {
    tension[e] = mju_max(tension[e], 0);
  }
}


// apply stiffness to edge-vector variations; scatter result with +/- edge signs
static inline void mj_stretchStiffnessMul(mjtNum result[6][3], const mjtNum metric[36],
                                          const mjtNum tension[6], const mjtNum edgevec[6][3],
                                          const mjtNum delta[6][3], int nedge, mjtNum scale) {
  mjtNum g[6], material[6];
  for (int e = 0; e < nedge; e++) {
    g[e] = 0;
    for (int x = 0; x < 3; x++) {
      g[e] += edgevec[e][x]*delta[e][x];
    }
  }
  mj_stretchTension(material, metric, g, nedge);
  for (int e = 0; e < nedge; e++) {
    mjtNum coef = material[e];
    coef *= 2*scale;
    for (int x = 0; x < 3; x++) {
      result[e][x] = coef*edgevec[e][x] + scale*tension[e]*delta[e][x];
    }
  }
}


// evaluate the corresponding 3x3 world-frame vertex-pair block for sparse assembly
static inline void mj_stretchStiffnessBlock(mjtNum block[9], const mjtNum metric[36],
                                            const mjtNum tension[6], const mjtNum edgevec[6][3],
                                            int dim, int i, int j, mjtNum scale) {
  int nedge = dim == 2 ? 3 : 6;
  const int (*edge)[2] = mj_stretchEdges[dim-2];
  mju_zero(block, 9);
  for (int a = 0; a < nedge; a++) {
    mjtNum sa = (i == edge[a][0]) ? 1 : ((i == edge[a][1]) ? -1 : 0);
    if (!sa) continue;
    for (int b = 0; b < nedge; b++) {
      mjtNum sb = (j == edge[b][0]) ? 1 : ((j == edge[b][1]) ? -1 : 0);
      if (!sb) continue;
      mjtNum w = 2*scale*metric[nedge*a + b]*sa*sb;
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          block[3*r+c] += w*edgevec[a][r]*edgevec[b][c];
        }
      }
    }
  }
  mjtNum geo = 0;
  for (int a = 0; a < nedge; a++) {
    mjtNum sa = (i == edge[a][0]) ? 1 : ((i == edge[a][1]) ? -1 : 0);
    mjtNum sb = (j == edge[a][0]) ? 1 : ((j == edge[a][1]) ? -1 : 0);
    if (!sa || !sb) continue;
    geo += tension[a]*sa*sb;
  }
  geo *= scale;
  block[0] += geo;
  block[4] += geo;
  block[8] += geo;
}

#endif  // MUJOCO_SRC_ENGINE_ENGINE_ELASTICITY_H_
