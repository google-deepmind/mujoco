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

#include "engine/engine_collision_nonconvex.h"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include "engine/engine_collision_convex.h"
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_misc.h"


// convert hfield-local AAMM bounds (xmin, ymin, zmin, xmax, ymax, zmax) to cell subgrid
// [rmin, rmax) x [cmin, cmax); return 1 if overlapping, 0 if disjoint or empty
static int hfieldBoundsToSubgrid(const mjModel* m, int hid, const mjtNum aamm[6],
                                 int* rmin, int* rmax, int* cmin, int* cmax) {
  const mjtNum* size = m->hfield_size + 4*hid;
  int nr = m->hfield_nrow[hid] - 1;
  int nc = m->hfield_ncol[hid] - 1;
  if (nr < 1 || nc < 1) {
    return 0;
  }

  // AAMM box-box test
  if ((aamm[0] > size[0]) || (aamm[3] < -size[0]) ||
      (aamm[1] > size[1]) || (aamm[4] < -size[1]) ||
      (aamm[2] > size[2]) || (aamm[5] < -size[3])) {
    return 0;
  }

  // compute sub-grid cell bounds [rmin, rmax) x [cmin, cmax)
  *cmin = (int) mju_max(0, mju_floor((aamm[0] + size[0]) / (2.0 * size[0]) * nc));
  *cmax = (int) mju_min(nc, mju_ceil((aamm[3] + size[0]) / (2.0 * size[0]) * nc));
  *rmin = (int) mju_max(0, mju_floor((aamm[1] + size[1]) / (2.0 * size[1]) * nr));
  *rmax = (int) mju_min(nr, mju_ceil((aamm[4] + size[1]) / (2.0 * size[1]) * nr));

  // ensure non-empty range when AAMM lies exactly on a grid line (symmetric on interior lines)
  if (*cmin == *cmax) {
    if (*cmin > 0) (*cmin)--;
    if (*cmax < nc) (*cmax)++;
  }
  if (*rmin == *rmax) {
    if (*rmin > 0) (*rmin)--;
    if (*rmax < nr) (*rmax)++;
  }
  return 1;
}


// compute overlapping hfield subgrid [rmin, rmax) x [cmin, cmax) and zmin for geom g2;
// return 1 if overlap is non-empty, 0 if disjoint
int mjc_hfieldSubgridGeom(const mjModel* m, const mjData* d, int g1, int g2, mjtNum margin,
                          int* rmin, int* rmax, int* cmin, int* cmax, mjtNum* zmin_out) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;

  int hid = m->geom_dataid[g1];
  const mjtNum* size = m->hfield_size + 4*hid;

  // express geom2 pos in hfield frame
  mjtNum local_pos[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  mju_mulMatTVec3(local_pos, mat1, local_pos);

  // early return using box-sphere test
  mjtNum r = m->geom_rbound[g2] + margin;
  if ((size[0] < local_pos[0] - r) || (-size[0] > local_pos[0] + r) ||
      (size[1] < local_pos[1] - r) || (-size[1] > local_pos[1] + r) ||
      (size[2] < local_pos[2] - r) || (-size[3] > local_pos[2] + r)) {
    return 0;
  }

  // express geom2 in hfield frame and compute tight AABB via 6 support calls
  mjCCDObj obj2;
  mjc_initCCDObj(&obj2, m, d, g2, 0);
  mji_mulMatTMat3(obj2.mat, mat1, mat2);
  mji_copy3(obj2.pos, local_pos);

  // compute AAMM expanded by margin in -/+ x, y, and z directions
  mjtNum aamm[6], res[3], dir[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    dir[i] = 1;
    obj2.support(res, &obj2, dir);
    aamm[i + 3] = res[i] + margin;

    dir[i] = -1;
    obj2.support(res, &obj2, dir);
    aamm[i] = res[i] - margin;

    dir[i] = 0;
  }

  *zmin_out = aamm[2];
  return hfieldBoundsToSubgrid(m, hid, aamm, rmin, rmax, cmin, cmax);
}


// compute overlapping hfield subgrid [rmin, rmax) x [cmin, cmax) and zmin for flex element (f, e);
// return 1 if overlap is non-empty, 0 if disjoint
int mjc_hfieldSubgridElem(const mjModel* m, const mjData* d, int g, int f, int e, mjtNum margin,
                          int* rmin, int* rmax, int* cmin, int* cmax, mjtNum* zmin_out) {
  int hid = m->geom_dataid[g];
  const mjtNum* hpos = d->geom_xpos + 3*g;
  const mjtNum* hmat = d->geom_xmat + 9*g;

  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim + 1);
  mjtNum aamm[6] = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
  for (int i = 0; i <= dim; i++) {
    const mjtNum* v = d->flexvert_xpos + 3*(m->flex_vertadr[f] + edata[i]);
    mjtNum vec[3], evert[3];
    mji_sub3(vec, v, hpos);
    mji_mulMatTVec3(evert, hmat, vec);
    for (int k = 0; k < 3; k++) {
      aamm[k] = mju_min(aamm[k], evert[k]);
      aamm[k + 3] = mju_max(aamm[k + 3], evert[k]);
    }
  }

  mjtNum expand = m->flex_radius[f] + margin;
  for (int k = 0; k < 3; k++) {
    aamm[k] -= expand;
    aamm[k + 3] += expand;
  }

  *zmin_out = aamm[2];
  return hfieldBoundsToSubgrid(m, hid, aamm, rmin, rmax, cmin, cmax);
}
