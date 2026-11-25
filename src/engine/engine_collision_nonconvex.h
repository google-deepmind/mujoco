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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_NONCONVEX_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_NONCONVEX_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#ifdef __cplusplus
extern "C" {
#endif

// compute overlapping hfield subgrid [rmin, rmax) x [cmin, cmax) and zmin in hfield frame;
// return 1 if overlap is non-empty, 0 if disjoint
MJAPI int mjc_hfieldSubgridGeom(const mjModel* m, const mjData* d, int g1, int g2, mjtNum margin,
                                int* rmin, int* rmax, int* cmin, int* cmax, mjtNum* zmin);
MJAPI int mjc_hfieldSubgridElem(const mjModel* m, const mjData* d, int g, int f, int e,
                                mjtNum margin, int* rmin, int* rmax, int* cmin, int* cmax,
                                mjtNum* zmin);

// encode heightfield prism id from grid cell (r, c), triangle index tri in {0, 1},
// and number of cell columns nc = ncol - 1 (assumes 2 * nr * nc <= INT_MAX)
static inline int mjc_hfieldPrismId(int r, int c, int tri, int nc) {
  return (r * nc + c) * 2 + tri;
}

// decode heightfield prism id into grid cell (r, c) and triangle index tri in {0, 1}
// given number of cell columns nc = ncol - 1
static inline void mjc_hfieldPrismCoords(int prism_id, int nc, int* r, int* c, int* tri) {
  int cell = prism_id / 2;
  *tri = prism_id % 2;
  *r = cell / nc;
  *c = cell % nc;
}

// return 2-bit mask of prisms in cell (r, c) with at least one top vertex >= zmin:
// bit 0 = tri 0 (upper-left), bit 1 = tri 1 (lower-right)
static inline int mjc_hfieldCellMask(const float* hdata, int ncol, mjtNum size2,
                                     int r, int c, mjtNum zmin) {
  mjtNum h00 = hdata[r*ncol + c] * size2;
  mjtNum h10 = hdata[(r+1)*ncol + c] * size2;
  mjtNum h01 = hdata[r*ncol + (c+1)] * size2;
  mjtNum h11 = hdata[(r+1)*ncol + (c+1)] * size2;
  int mask = 0;
  if (!(h10 < zmin && h00 < zmin && h11 < zmin)) mask |= 1;
  if (!(h00 < zmin && h01 < zmin && h11 < zmin)) mask |= 2;
  return mask;
}

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_NONCONVEX_H_
