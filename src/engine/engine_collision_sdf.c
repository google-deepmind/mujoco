// Copyright 2021 DeepMind Technologies Limited
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

#include "engine/engine_collision_sdf.h"

#include <math.h>
#include <stdio.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjtnum.h>
#include "engine/engine_collision_primitive.h"
#include "engine/engine_plugin.h"
#include "engine/engine_ray.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


//---------------------------- interpolated sdf -------------------------------------------

mjtNum boxProjection(mjtNum point[3], const mjtNum box[6]) {
  mjtNum r[3] = {point[0] - box[0], point[1] - box[1], point[2] - box[2]};
  mjtNum q[3] = {mju_abs(r[0]) - box[3], mju_abs(r[1]) - box[4],
                 mju_abs(r[2]) - box[5]};
  mjtNum dist_sqr = 0;
  mjtNum eps = 1e-6;

  // skip the projection if inside
  if (q[0] <= 0 && q[1] <= 0 && q[2] <= 0) {
    return mju_max(q[0], mju_max(q[1], q[2]));
  }

  // in-place projection inside the box if outside
  if ( q[0] >= 0 ) {
    dist_sqr += q[0] * q[0];
    point[0] -= r[0] > 0 ? (q[0]+eps) : -(q[0]+eps);
  }
  if ( q[1] >= 0 ) {
    dist_sqr += q[1] * q[1];
    point[1] -= r[1] > 0 ? (q[1]+eps) : -(q[1]+eps);
  }
  if ( q[2] >= 0 ) {
    dist_sqr += q[2] * q[2];
    point[2] -= r[2] > 0 ? (q[2]+eps) : -(q[2]+eps);
  }

  return mju_sqrt(dist_sqr);
}

// find the octree leaf containing the point p, return the index of the leaf and
// populate the weights of the interpolated function (if w is not null) and of
// its gradient (if dw is not null) using the vertices as degrees of freedom for
// trilinear interpolation.
static int findOct(mjtNum w[8], mjtNum dw[8][3], const mjtNum* oct_aabb,
                   const int* oct_child, const mjtNum p[3]) {
  int stack = 0;
  mjtNum eps = 1e-8;
  int niter = 100;

  while (niter-- > 0) {
    int node = stack;
    mjtNum vmin[3], vmax[3];

    if (node == -1) {  // SHOULD NOT OCCUR
      mju_error("Invalid node number");
      return -1;
    }

    for (int j = 0; j < 3; j++) {
      vmin[j] = oct_aabb[6*node+j] - oct_aabb[6*node+3+j];
      vmax[j] = oct_aabb[6*node+j] + oct_aabb[6*node+3+j];
    }

    // check if the point is inside the aabb of the octree node
    if (p[0] + eps < vmin[0] || p[0] - eps > vmax[0] ||
        p[1] + eps < vmin[1] || p[1] - eps > vmax[1] ||
        p[2] + eps < vmin[2] || p[2] - eps > vmax[2]) {
      continue;
    }

    mjtNum coord[3] = {(p[0] - vmin[0]) / (vmax[0] - vmin[0]),
                       (p[1] - vmin[1]) / (vmax[1] - vmin[1]),
                       (p[2] - vmin[2]) / (vmax[2] - vmin[2])};

    // check if the node is a leaf
    if (oct_child[8*node+0] == -1 && oct_child[8*node+1] == -1 &&
        oct_child[8*node+2] == -1 && oct_child[8*node+3] == -1 &&
        oct_child[8*node+4] == -1 && oct_child[8*node+5] == -1 &&
        oct_child[8*node+6] == -1 && oct_child[8*node+7] == -1) {
      for (int j = 0; j < 8; j++) {
        if (w) {
          w[j] = (j & 1 ? coord[0] : 1 - coord[0]) *
                 (j & 2 ? coord[1] : 1 - coord[1]) *
                 (j & 4 ? coord[2] : 1 - coord[2]);
        }
        if (dw) {
          dw[j][0] = (j & 1 ? 1 : -1) *
                     (j & 2 ? coord[1] : 1 - coord[1]) *
                     (j & 4 ? coord[2] : 1 - coord[2]);
          dw[j][1] = (j & 1 ? coord[0] : 1 - coord[0]) *
                     (j & 2 ? 1 : -1) *
                     (j & 4 ? coord[2] : 1 - coord[2]);
          dw[j][2] = (j & 1 ? coord[0] : 1 - coord[0]) *
                     (j & 2 ? coord[1] : 1 - coord[1]) *
                     (j & 4 ? 1 : -1);
        }
      }
      return node;
    }

    // compute which of 8 children to visit next
    int x = coord[0] < .5 ? 0 : 1;
    int y = coord[1] < .5 ? 0 : 1;
    int z = coord[2] < .5 ? 0 : 1;
    stack = oct_child[8 * node + 4*z + 2*y + x];
  }

  mju_error("Node not found");  // SHOULD NOT OCCUR
  return -1;
}

// sdf
mjtNum oct_distance(const mjModel* m, const mjtNum p[3], int meshid) {
  int octadr = m->mesh_octadr[meshid];
  int* oct_child = m->oct_child + 8*octadr;
  mjtNum* oct_aabb = m->oct_aabb + 6*octadr;
  mjtNum* oct_coeff = m->oct_coeff + 8*octadr;

  if (octadr == -1) {
    mjERROR("Octree not found in mesh %d", meshid);
    return 0;
  }

  mjtNum w[8];
  mjtNum sdf = 0;
  mjtNum point[3] = {p[0], p[1], p[2]};
  mjtNum boxDist = boxProjection(point, oct_aabb);
  int node = findOct(w, NULL, oct_aabb, oct_child, point);
  for (int i = 0; i < 8; ++i) {
    sdf += w[i] * oct_coeff[8*node + i];
  }
  return boxDist > 0 ? sdf + boxDist : sdf;
}

// gradient of sdf
void oct_gradient(const mjModel* m, mjtNum grad[3], const mjtNum point[3], int meshid) {
  mju_zero3(grad);
  mjtNum p[3] = {point[0], point[1], point[2]};

  int octadr = m->mesh_octadr[meshid];
  int* oct_child = m->oct_child + 8*octadr;
  mjtNum* oct_aabb = m->oct_aabb + 6*octadr;
  mjtNum* oct_coeff = m->oct_coeff + 8*octadr;

  if (octadr == -1) {
    mjERROR("Octree not found in mesh %d", meshid);
  }

  // analytic in the interior
  if (boxProjection(p, oct_aabb) <= 0) {
    mjtNum dw[8][3];
    int node = findOct(NULL, dw, oct_aabb, oct_child, p);
    for (int i = 0; i < 8; ++i) {
      grad[0] += dw[i][0] * oct_coeff[8*node + i];
      grad[1] += dw[i][1] * oct_coeff[8*node + i];
      grad[2] += dw[i][2] * oct_coeff[8*node + i];
    }
    return;
  }

  // finite difference in the exterior
  mjtNum eps = 1e-8;
  mjtNum dist0 = oct_distance(m, point, meshid);
  mjtNum pointX[3] = {point[0]+eps, point[1], point[2]};
  mjtNum distX = oct_distance(m, pointX, meshid);
  mjtNum pointY[3] = {point[0], point[1]+eps, point[2]};
  mjtNum distY = oct_distance(m, pointY, meshid);
  mjtNum pointZ[3] = {point[0], point[1], point[2]+eps};
  mjtNum distZ = oct_distance(m, pointZ, meshid);

  grad[0] = (distX - dist0) / eps;
  grad[1] = (distY - dist0) / eps;
  grad[2] = (distZ - dist0) / eps;
}


//---------------------------- primitives sdf ---------------------------------------------

static void radialField3d(mjtNum field[3], const mjtNum a[3], const mjtNum x[3],
                          const mjtNum size[3]) {
  field[0] = -size[0] / a[0];
  field[1] = -size[1] / a[1];
  field[2] = -size[2] / a[2];
  mju_normalize3(field);

  // flip sign if necessary
  if (x[0] < 0) field[0] = -field[0];
  if (x[1] < 0) field[1] = -field[1];
  if (x[2] < 0) field[2] = -field[2];
}

static mjtNum geomDistance(const mjModel* m, const mjData* d, const mjpPlugin* p,
                           int i, const mjtNum x[3], mjtGeom type) {
  mjtNum a[3], b[3];
  const mjtNum* size = m->geom_size+3*i;

  // see https://iquilezles.org/articles/distfunctions/
  switch (type) {
  case mjGEOM_PLANE:
    return x[2];
  case mjGEOM_SPHERE:
    return mju_norm3(x) - size[0];
  case mjGEOM_BOX:
    // compute shortest distance to box surface if outside, otherwise
    // intersect with a unit gradient that linearly rotates from radial to the face normals
    a[0] = mju_abs(x[0]) - size[0];
    a[1] = mju_abs(x[1]) - size[1];
    a[2] = mju_abs(x[2]) - size[2];
    if (a[0] >= 0 || a[1] >= 0 || a[2] >= 0) {
      b[0] = mju_max(a[0], 0);
      b[1] = mju_max(a[1], 0);
      b[2] = mju_max(a[2], 0);
      return mju_norm3(b) + mju_min(mju_max(a[0], mju_max(a[1], a[2])), 0);
    }
    radialField3d(b, a, x, size);
    mjtNum t[3];
    t[0] = -a[0] / mju_abs(b[0]);
    t[1] = -a[1] / mju_abs(b[1]);
    t[2] = -a[2] / mju_abs(b[2]);
    return -mju_min(t[0], mju_min(t[1], t[2])) * mju_norm3(b);
  case mjGEOM_CAPSULE:
    a[0] = x[0];
    a[1] = x[1];
    a[2] = x[2] - mju_clip(x[2], -size[1], size[1]);
    return mju_norm3(a) - size[0];
  case mjGEOM_ELLIPSOID:
    a[0] = x[0] / size[0];
    a[1] = x[1] / size[1];
    a[2] = x[2] / size[2];
    b[0] = a[0] / size[0];
    b[1] = a[1] / size[1];
    b[2] = a[2] / size[2];
    mjtNum k0 = mju_norm3(a);
    mjtNum k1 = mju_norm3(b);
    return k0 * (k0 - 1.0) / k1;
  case mjGEOM_CYLINDER:
    a[0] = mju_sqrt(x[0]*x[0]+x[1]*x[1]) - size[0];
    a[1] = mju_abs(x[2]) - size[1];
    b[0] = mju_max(a[0], 0);
    b[1] = mju_max(a[1], 0);
    return mju_min(mju_max(a[0], a[1]), 0) + mju_norm(b, 2);
  case mjGEOM_SDF:
    if (p) {
      return p->sdf_distance(x, d, i);
    } else {
      return oct_distance(m, x, i);
    }
  case mjGEOM_MESH:
    if (m->mesh_octnum[i]) {
      return oct_distance(m, x, i);
    } else {
      mju_mulMatVec3(a, d->geom_xmat + 9 * i, x);
      mju_addTo3(a, d->geom_xpos + 3 * i);
      mjtNum dir[3] = {-a[0], -a[1], -a[2]};
      mjtNum r = mju_norm3(dir);
      mjtNum dist = mj_rayMesh(m, d, i, a, dir);
      if (dist > r) {
        mju_scl3(dir, dir, -1);
        return -mj_rayMesh(m, d, i, a, dir);
      }
      return dist;
    }
  default:
    mjERROR("sdf collisions not available for geom type %d", type);
    return 0;
  }
}

static void geomGradient(mjtNum gradient[3], const mjModel* m, const mjData* d,
                         const mjpPlugin* p, int i, const mjtNum x[3],
                         mjtGeom type) {
  mjtNum a[3], b[3], c, e;
  const mjtNum* size = m->geom_size+3*i;

  // see https://iquilezles.org/articles/distfunctions/
  switch (type) {
  case mjGEOM_PLANE:
    mju_zero3(gradient);
    gradient[2] = 1;
    break;
  case mjGEOM_SPHERE:
    mju_copy3(gradient, x);
    c = mju_norm3(x);
    gradient[0] *= 1. / c;
    gradient[1] *= 1. / c;
    gradient[2] *= 1. / c;
    break;
  case mjGEOM_BOX:
    mju_zero3(gradient);
    a[0] = mju_abs(x[0]) - size[0];
    a[1] = mju_abs(x[1]) - size[1];
    a[2] = mju_abs(x[2]) - size[2];
    int k = a[0] > a[1] ? 0 : 1;
    int l = a[2] > a[k] ? 2 : k;
    if (a[l] < 0) {
      radialField3d(gradient, a, x, size);
    } else {
      b[0] = mju_max(a[0], 0);
      b[1] = mju_max(a[1], 0);
      b[2] = mju_max(a[2], 0);
      c = mju_norm3(b);
      gradient[0] = a[0] > 0 ? b[0] / c * x[0] / mju_abs(x[0]) : 0;
      gradient[1] = a[1] > 0 ? b[1] / c * x[1] / mju_abs(x[1]) : 0;
      gradient[2] = a[2] > 0 ? b[2] / c * x[2] / mju_abs(x[2]) : 0;
    }
    break;
  case mjGEOM_CAPSULE:
    a[0] = x[0];
    a[1] = x[1];
    a[2] = x[2] - mju_clip(x[2], -size[1], size[1]);
    c = mju_norm3(a);
    gradient[0] = a[0] / c;
    gradient[1] = a[1] / c;
    gradient[2] = a[2] / c;
    break;
  case mjGEOM_ELLIPSOID:
    a[0] = x[0] / size[0];
    a[1] = x[1] / size[1];
    a[2] = x[2] / size[2];
    b[0] = a[0] / size[0];
    b[1] = a[1] / size[1];
    b[2] = a[2] / size[2];
    mjtNum k0 = mju_norm3(a);
    mjtNum k1 = mju_norm3(b);
    mjtNum invK0 = 1. / k0;
    mjtNum invK1 = 1. / k1;
    mjtNum gk0[3] = {b[0]*invK0, b[1]*invK0, b[2]*invK0};
    mjtNum gk1[3] = {b[0]*invK1/(size[0]*size[0]),
                     b[1]*invK1/(size[1]*size[1]),
                     b[2]*invK1/(size[2]*size[2])};
    mjtNum df_dk0 = (2.*k0 - 1.) * invK1;
    mjtNum df_dk1 = k0*(k0 - 1.) * invK1 * invK1;
    gradient[0] = gk0[0]*df_dk0 - gk1[0]*df_dk1;
    gradient[1] = gk0[1]*df_dk0 - gk1[1]*df_dk1;
    gradient[2] = gk0[2]*df_dk0 - gk1[2]*df_dk1;
    mju_normalize3(gradient);
    break;
  case mjGEOM_CYLINDER:
    c = mju_sqrt(x[0]*x[0]+x[1]*x[1]);
    e = mju_abs(x[2]);
    a[0] = c - size[0];
    a[1] = e - size[1];
    mjtNum grada[3] = {x[0] / mju_max(c, 1. / mjMAXVAL),
                       x[1] / mju_max(c, 1. / mjMAXVAL),
                       x[2] / mju_max(e, 1. / mjMAXVAL)};
    int j = a[0] > a[1] ? 0 : 1;
    if (a[j] < 0) {
      gradient[0] = j == 0 ? grada[0] : 0;
      gradient[1] = j == 0 ? grada[1] : 0;
      gradient[2] = j == 1 ? grada[2] : 0;
    } else {
      b[0] = mju_max(a[0], 0);
      b[1] = mju_max(a[1], 0);
      mjtNum bnorm = mju_max(mju_norm(b, 2), 1./mjMAXVAL);
      gradient[0] = grada[0] * b[0] / bnorm;
      gradient[1] = grada[1] * b[0] / bnorm;
      gradient[2] = grada[2] * b[1] / bnorm;
    }
    break;
  case mjGEOM_SDF:
    if (p) {
      p->sdf_gradient(gradient, x, d, i);
    } else {
      oct_gradient(m, gradient, x, i);
    }
    break;
  case mjGEOM_MESH:
    if (m->mesh_octnum[i]) {
      oct_gradient(m, gradient, x, i);
    } else {
      mju_mulMatVec3(a, d->geom_xmat+9*i, x);
      mju_addTo3(a, d->geom_xpos+3*i);
      mjtNum dir[3] = {-a[0], -a[1], -a[2]};
      mjtNum r = mju_norm3(dir);
      mjtNum dist = mj_rayMesh(m, d, i, a, dir);
      gradient[0] = dist > r ? 1 : -1;
      gradient[1] = dist > r ? 1 : -1;
      gradient[2] = dist > r ? 1 : -1;
    }
    break;
  default:
    mjERROR("sdf collisions not available for geom type %d", type);
  }
}

//---------------------------- helper functions -------------------------------------------

// signed distance function
mjtNum mjc_distance(const mjModel* m, const mjData* d, const mjSDF* s, const mjtNum x[3]) {
  mjtNum y[3];

  switch (s->type) {
  case mjSDFTYPE_SINGLE:
    return geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
  case mjSDFTYPE_INTERSECTION:
    mju_mulMatVec3(y, s->relmat, x);
    mju_addTo3(y, s->relpos);
    return mju_max(geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]),
                   geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]));
  case mjSDFTYPE_MIDSURFACE:
    mju_mulMatVec3(y, s->relmat, x);
    mju_addTo3(y, s->relpos);
    return geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]) -
           geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
  case mjSDFTYPE_COLLISION:
    mju_mulMatVec3(y, s->relmat, x);
    mju_addTo3(y, s->relpos);
    mjtNum A = geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    mjtNum B = geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    return A + B + mju_abs(mju_max(A, B));
  default:
    mjERROR("SDF type not available");
    return 0;
  }
}

// gradient of sdf
void mjc_gradient(const mjModel* m, const mjData* d, const mjSDF* s,
                  mjtNum gradient[3], const mjtNum x[3]) {
  mjtNum y[3];
  const mjtNum* point[2] = {x, y};
  mjtNum grad1[3], grad2[3];

  switch (s->type) {
  case mjSDFTYPE_INTERSECTION:
    mju_mulMatVec3(y, s->relmat, x);
    mju_addTo3(y, s->relpos);
    int i = geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]) >
            geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]) ? 0 : 1;
    geomGradient(gradient, m, d, s->plugin[i], s->id[i], point[i], s->geomtype[i]);
    if (i == 1) {
      mju_mulMatTVec3(gradient, s->relmat, gradient);
    }
    break;
  case mjSDFTYPE_MIDSURFACE:
    mju_mulMatVec3(y, s->relmat, x);
    mju_addTo3(y, s->relpos);
    geomGradient(grad1, m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    mju_normalize3(grad1);
    geomGradient(grad2, m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    mju_mulMatTVec3(grad2, s->relmat, grad2);
    mju_normalize3(grad2);
    mju_sub3(gradient, grad1, grad2);
    mju_normalize3(gradient);
    break;
  case mjSDFTYPE_COLLISION:
    mju_mulMatVec3(y, s->relmat, x);
    mju_addTo3(y, s->relpos);
    mjtNum A = geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    mjtNum B = geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    geomGradient(grad1, m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    geomGradient(grad2, m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    mju_mulMatTVec3(grad2, s->relmat, grad2);
    gradient[0] = grad1[0] + grad2[0];
    gradient[1] = grad1[1] + grad2[1];
    gradient[2] = grad1[2] + grad2[2];
    mju_addToScl3(gradient, A > B ? grad1 : grad2, mju_max(A, B) > 0 ? 1 : -1);
    break;
  case mjSDFTYPE_SINGLE:
    geomGradient(gradient, m, d, s->plugin[0], s->id[0], point[0], s->geomtype[0]);
    break;
  default:
    mjERROR("SDF type not available");
  }
}

// get sdf from geom id
const mjpPlugin* mjc_getSDF(const mjModel* m, int id) {
  int instance = m->geom_plugin[id];
  const int nslot = mjp_pluginCount();
  const int slot = m->plugin[instance];
  const mjpPlugin* sdf = mjp_getPluginAtSlotUnsafe(slot, nslot);
  if (!sdf) mjERROR("invalid plugin slot: %d", slot);
  if (!(sdf->capabilityflags & mjPLUGIN_SDF)) {
    mjERROR("Plugin is not a signed distance field at slot %d", slot);
  }
  return sdf;
}

// map (pos12, mat12) as (xpos2, xmat2)^-1 \circ (xpos1, xmat1)
static void mapPose(const mjtNum xpos1[3], const mjtNum xquat1[4],
                    const mjtNum xpos2[3], const mjtNum xquat2[4],
                    mjtNum pos12[3], mjtNum mat12[9]) {
  mjtNum negpos[3], negquat[4], quat12[4];
  mju_negPose(negpos, negquat, xpos2, xquat2);
  mju_mulPose(pos12, quat12, negpos, negquat, xpos1, xquat1);
  mju_quat2Mat(mat12, quat12);
}

//---------------------------- narrow phase -----------------------------------------------

// check if the collision point already exists
static int isknown(const mjtNum* points, const mjtNum x[3], int cnt) {
  for (int i = 0; i < cnt; i++) {
    if (mju_dist3(x, points + 3*i) < mjMINVAL) {
      return 1;
    }
  }
  return 0;
}

// adds candidate point to result
static int addContact(mjtNum* points, mjContact* con, const mjtNum x[3],
                      const mjtNum pos2[3], const mjtNum quat2[4], mjtNum dist,
                      int cnt, const mjModel* m, const mjSDF* s, mjData* d) {
  // check if there is a collision
  if (dist > 0 || isknown(points, x, cnt)) {
    return cnt;
  } else {
    mju_copy3(points+3*cnt, x);
  }

  // compute normal in local coordinates
  mjtNum norm[3], vec[3];
  mjc_gradient(m, d, s, norm, x);
  mju_scl3(norm, norm, -1);

  // construct contact
  con[cnt].dist = dist;
  mju_rotVecQuat(con[cnt].frame, norm, quat2);
  mju_zero3(con[cnt].frame+3);
  mju_makeFrame(con[cnt].frame);
  mju_scl3(vec, con[cnt].frame, -con[cnt].dist/2);
  mju_rotVecQuat(con[cnt].pos, x, quat2);
  mju_addTo3(con[cnt].pos, pos2);
  mju_addTo3(con[cnt].pos, vec);

  return cnt+1;
}

// finds minimum using gradient descent
static mjtNum stepGradient(mjtNum x[3], const mjModel* m, const mjSDF* s,
                           mjData* d, int niter) {
  const mjtNum c = .1;       // reduction factor for the target decrease in the objective function
  const mjtNum rho = .5;     // reduction factor for the gradient scaling (alpha)
  const mjtNum amin = 1e-4;  // minimum value for alpha
  mjtNum dist = mjMAXVAL;

  for (int step=0; step < niter; step++) {
    mjtNum grad[3];
    mjtNum alpha = 2.;  // initial line search factor scaling the gradient
                        // the units of the gradient depend on s->type

    // evaluate gradient
    mjc_gradient(m, d, s, grad, x);

    // sanity check
    if (isnan(grad[0]) || grad[0] > mjMAXVAL || grad[0] < -mjMAXVAL ||
        isnan(grad[1]) || grad[1] > mjMAXVAL || grad[1] < -mjMAXVAL ||
        isnan(grad[2]) || grad[2] > mjMAXVAL || grad[2] < -mjMAXVAL) {
      return mjMAXVAL;
    }

    // save current solution
    mjtNum x0[] = {x[0], x[1], x[2]};

    // evaluate distance
    mjtNum dist0 = mjc_distance(m, d, s, x0);
    mjtNum wolfe = - c * alpha * mju_dot3(grad, grad);

    // backtracking line search
    do {
      alpha *= rho;
      wolfe *= rho;
      mju_addScl3(x, x0, grad, -alpha);
      dist = mjc_distance(m, d, s, x);
    } while (alpha > amin && dist - dist0 > wolfe);

    // if no improvement, early stop
    if (dist0 < dist) {
      return dist;
    }
  }

  // the distance will be used for the contact creation
  return dist;
}


//------------------------------ collision functions -----------------------------------------------

// collision between a height field and a signed distance field
int mjc_HFieldSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin) {
  mju_warning("HField vs SDF collision not yet supported!");
  return 0;
}

// collision between a mesh and a signed distance field
int mjc_MeshSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin) {
  return mjc_SDF(m, d, con, g1, g2, margin);
}

// collision between two SDFs
int mjc_SDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO;
  size1 = m->geom_aabb + 6*g1;
  size2 = m->geom_aabb + 6*g2;

  int cnt = 0;
  mjtNum x[3], y[3], dist, vec1[3], vec2[3];
  mjtNum aabb1[6] = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
  mjtNum aabb2[6] = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
  mjtNum aabb[6]  = {mjMAXVAL, mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL, -mjMAXVAL};

  // second geom must be an SDF
  if (m->geom_type[g2] != mjGEOM_SDF) {
    mjERROR("geom is not an SDF");
  }

  // compute transformations from/to g1 to/from g2
  mjtNum quat1[4], quat2[4];
  mjtNum offset21[3], rotation21[9], rotation12[9];
  mjtNum offset12[3], offset2[3], rotation2[9];
  mju_mat2Quat(quat1, mat1);
  mju_mat2Quat(quat2, mat2);
  mapPose(pos1, quat1, pos1, quat1, offset2, rotation2);
  mapPose(pos2, quat2, pos1, quat1, offset21, rotation21);
  mapPose(pos1, quat1, pos2, quat2, offset12, rotation12);

  // axis-aligned bounding boxes in g1 frame
  for (int i=0; i < 8; i++) {
    vec1[0] = (i&1 ? size1[0]+size1[3] : size1[0]-size1[3]);
    vec1[1] = (i&2 ? size1[1]+size1[4] : size1[1]-size1[4]);
    vec1[2] = (i&4 ? size1[2]+size1[5] : size1[2]-size1[5]);

    vec2[0] = (i&1 ? size2[0]+size2[3] : size2[0]-size2[3]);
    vec2[1] = (i&2 ? size2[1]+size2[4] : size2[1]-size2[4]);
    vec2[2] = (i&4 ? size2[2]+size2[5] : size2[2]-size2[5]);

    mju_mulMatVec3(vec2, rotation21, vec2);
    mju_addTo3(vec2, offset21);

    for (int k=0; k < 3; k++) {
      aabb1[0+k] = mju_min(aabb1[0+k], vec1[k]);
      aabb1[3+k] = mju_max(aabb1[3+k], vec1[k]);
      aabb2[0+k] = mju_min(aabb2[0+k], vec2[k]);
      aabb2[3+k] = mju_max(aabb2[3+k], vec2[k]);
    }
  }

  // intersection of aabbs
  for (int k=0; k < 3; k++) {
    aabb[0+k] = mju_max(aabb1[0+k], aabb2[0+k]);
    aabb[3+k] = mju_min(aabb1[3+k], aabb2[3+k]);
  }

  // no intersection if max < min
  if (aabb[3] < aabb[0] || aabb[4] < aabb[1] || aabb[5] < aabb[2]) {
    return cnt;
  }

  // create sdf pointers
  int instance[2];
  const mjpPlugin* sdf_ptr[2];
  mjtGeom geomtypes[2] = {m->geom_type[g2], m->geom_type[g1]};

  instance[0] = m->geom_plugin[g2];
  sdf_ptr[0] = instance[0] == -1 ? NULL : mjc_getSDF(m, g2);

  // get sdf plugins
  if (m->geom_type[g1] == mjGEOM_SDF) {
    instance[1] = m->geom_plugin[g1];
    sdf_ptr[1] = instance[1] == -1 ? NULL : mjc_getSDF(m, g1);
  } else {
    instance[1] = g1;
    sdf_ptr[1] = NULL;
  }

  // reset visualization count
  if (sdf_ptr[0]) {
    sdf_ptr[0]->reset(m, NULL, (void*)(d->plugin_data[instance[0]]), instance[0]);
  }

  // copy into sdf
  mjSDF sdf;
  instance[0] = instance[0] == -1 ? m->geom_dataid[g2] : instance[0];
  instance[1] = instance[1] == -1 ? m->geom_dataid[g1] : instance[1];
  sdf.id = instance;
  sdf.relpos = offset21;
  sdf.relmat = rotation21;
  sdf.plugin = sdf_ptr;
  sdf.geomtype = geomtypes;

  // minimize sdf intersection
  mjtNum contacts[3*mjMAXCONPAIR];

  int i = 0, j = 0;
  while (i < m->opt.sdf_initpoints) {
    x[0] = aabb[0] + (aabb[3]-aabb[0]) * mju_Halton(j, 2);
    x[1] = aabb[1] + (aabb[4]-aabb[1]) * mju_Halton(j, 3);
    x[2] = aabb[2] + (aabb[5]-aabb[2]) * mju_Halton(j, 5);

    mju_mulMatVec3(y, rotation2, x);
    mju_addTo3(y, offset2);

    mju_mulMatVec3(x, rotation12, y);
    mju_addTo3(x, offset12);

    j++;

    // here a criterion for rejecting points could be inserted

    i++;

    // start counters
    if (sdf_ptr[0]) {
      sdf_ptr[0]->compute(m, (mjData*)d, instance[0], mjPLUGIN_SDF);
    }

    // gradient descent - we use a special function of the two SDF as objective
    sdf.type = mjSDFTYPE_COLLISION;
    dist = stepGradient(x, m, &sdf, (mjData*)d, m->opt.sdf_iterations);

    // inexact SDFs can yield spurious collisions, filter them by projecting on the midsurface
    sdf.type = mjSDFTYPE_INTERSECTION;
    dist = stepGradient(x, m, &sdf, (mjData*)d, 1);

    // contact point and normal - we use the midsurface where SDF1=SDF2 as zero level set
    sdf.type = mjSDFTYPE_MIDSURFACE;
    cnt = addContact(contacts, con, x, pos2, quat2, dist, cnt, m, &sdf, (mjData*)d);

    // SHOULD NOT OCCUR
    if (cnt > mjMAXCONPAIR) {
      mjERROR("too many contact points");
    }
  }

  return cnt;
}

