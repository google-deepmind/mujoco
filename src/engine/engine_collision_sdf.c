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

#include <stdio.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjtnum.h>
#include "engine/engine_collision_primitive.h"
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sort.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


#define MAXSDFFACE 1300
#define MAXMESHPNT 500


//---------------------------- primitives sdf ---------------------------------------------

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
    a[0] = mju_abs(x[0]) - size[0];
    a[1] = mju_abs(x[1]) - size[1];
    a[2] = mju_abs(x[2]) - size[2];
    b[0] = mju_max(a[0], 0);
    b[1] = mju_max(a[1], 0);
    b[2] = mju_max(a[2], 0);
    return mju_norm3(b) + mju_min(mju_max(a[0], mju_max(a[1], a[2])), 0);
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
    return p->sdf_distance(x, d, i);
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
      gradient[l] = x[l] / mju_abs(x[l]);
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
    p->sdf_gradient(gradient, x, d, i);
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

// subtract mesh position from sdf transformation
static void undoTransformation(const mjModel* m, const mjData* d, int g,
                               mjtNum sdf_xpos[3], mjtNum sdf_quat[4]) {
  mjtNum* xpos = d->geom_xpos + 3 * g;
  mjtNum* xmat = d->geom_xmat + 9 * g;
  if (m->geom_type[g] == mjGEOM_MESH || m->geom_type[g] == mjGEOM_SDF) {
    mjtNum negpos[3], negquat[4], xquat[4];
    mjtNum* pos = m->mesh_pos + 3 * m->geom_dataid[g];
    mjtNum* quat = m->mesh_quat + 4 * m->geom_dataid[g];
    mju_mat2Quat(xquat, xmat);
    mju_negPose(negpos, negquat, pos, quat);
    mju_mulPose(sdf_xpos, sdf_quat, xpos, xquat, negpos, negquat);
  } else {
    mju_copy3(sdf_xpos, xpos);
    mju_mat2Quat(sdf_quat, xmat);
  }
}

//---------------------------- narrow phase -----------------------------------------------

// comparison function for contact sorting
static inline int distcmp(int* i, int* j, void* context) {
  mjtNum d1 = ((mjtNum*)context)[*i];
  mjtNum d2 = ((mjtNum*)context)[*j];
    if (d1 < d2) {
    return -1;
  } else if (d1 == d2) {
    return 0;
  } else {
    return 1;
  }
}

// define distSort function for contact sorting
mjSORT(distSort, int, distcmp)

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

// finds minimum of Frank-Wolfe objective
static mjtNum stepFrankWolfe(mjtNum x[3], const mjtNum* corners, int ncorners,
                             const mjModel* m, const mjSDF* sdf, mjData* d) {
  for (int step=0; step < m->opt.sdf_iterations; step++) {
    mjtNum best = 1e10, fun, s[3], grad[3];

    // evaluate gradient
    mjc_gradient(m, d, sdf, grad, x);

    // evaluate all corners
    for (int i=0; i < ncorners; i++) {
      // compute sdf
      fun = mju_dot3(corners + 3*i, grad);

      // save argmin
      if (fun < best) {
        best = fun;
        mju_copy3(s, corners + 3*i);
      }
    }

    // update collision point
    mju_subFrom3(s, x);
    mju_addToScl3(x, s, 2. / (step+2.));
  }

  // compute distance
  return mjc_distance(m, d, sdf, x);
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

//---------------------------- bounding box vs sdf -------------------------------------------------

// stricter triangle collision
static int triangleIntersect(const mjtNum triangle[9], const mjModel* m,
                             const mjSDF* sdf, mjData* d) {
  mjtNum edges[6];
  mjtNum normal[3], center[3];
  mjtNum v[9], cross[9], p[3];
  mjtNum kDistanceScl = 10.;

  // triangle normal
  mju_sub3(edges+0, triangle+3, triangle);
  mju_sub3(edges+3, triangle+6, triangle);
  mju_cross(normal, edges, edges+3);
  mju_normalize3(normal);

  // fourth point
  mju_scl3(p, triangle, 1./3.);
  mju_addToScl3(p, triangle+3, 1./3.);
  mju_addToScl3(p, triangle+6, 1./3.);
  mjtNum h = -mjc_distance(m, d, sdf, p)/kDistanceScl;
  mju_addToScl3(p, normal, -h);

  // circumsphere center
  mju_sub3(v+0, triangle+0, p);
  mju_sub3(v+3, triangle+3, p);
  mju_sub3(v+6, triangle+6, p);
  mju_cross(cross+0, v+3, v+6);
  mju_cross(cross+3, v+6, v+0);
  mju_cross(cross+6, v+0, v+3);
  mju_scl3(center, cross, mju_dot3(v, v));
  mju_addToScl3(center, cross+3, mju_dot3(v+3, v+3));
  mju_addToScl3(center, cross+6, mju_dot3(v+6, v+6));
  mju_scl3(center, center, 1./(2.*mju_dot3(v, cross)));

  // circumsphere radius
  mjtNum r = mju_sqrt(mju_dot3(center, center));

  // coordinate change
  mju_addTo3(center, p);

  return mjc_distance(m, d, sdf, center) < r;
}

// intersect with circumsphere of bounding box
static int boxIntersect(const mjtNum bvh[6], const mjtNum offset[3],
                        const mjtNum rotation[9], const mjModel* m,
                        const mjSDF* s, mjData* d) {
  mjtNum candidate[3];
  mjtNum r = mju_norm3(bvh+3);

  mju_mulMatVec3(candidate, rotation, bvh);
  mju_addTo3(candidate, offset);

  // check if inside the bounding box
  return mjc_distance(m, d, s, candidate) < r;
}

//---------------------------- mesh vs sdf broad phase --------------------------------------------

// tree vs sdf binary search
static void collideBVH(const mjModel* m, mjData* d, int g,
                       const mjtNum offset[3], const mjtNum rotation[9],
                       int* faces, int* npoints, int* n0,
                       const mjSDF* sdf) {
  const int bvhadr = m->mesh_bvhadr[m->geom_dataid[g]];
  const int* faceid = m->bvh_nodeid + bvhadr;
  const mjtNum* bvh = m->bvh_aabb + 6*bvhadr;
  const int* child = m->bvh_child + 2*bvhadr;
  mjtByte* bvh_active = m->vis.global.bvactive ? d->bvh_active + bvhadr : NULL;

  mj_markStack(d);
  // TODO(quaglino): Store bvh max depths to make this bound tighter.
  int max_stack = m->mesh_bvhnum[m->geom_dataid[g]];
  struct CollideTreeArgs_ {
    int node;
  };
  typedef struct CollideTreeArgs_ CollideTreeArgs;
  CollideTreeArgs* stack = mjSTACKALLOC(d, max_stack, CollideTreeArgs);
  int nstack = 0;
  stack[nstack].node = 0;
  nstack++;

  while (nstack) {
    (*n0)++;

    // pop from stack
    nstack--;
    int node = stack[nstack].node;

    // node1 is a leaf
    if (faceid[node] != -1) {
      if (boxIntersect(bvh+6*node, offset, rotation, m, sdf, d)) {
        faces[*npoints] = faceid[node];
        if (++(*npoints) == MAXSDFFACE) {
          mju_warning("mjc_MeshSDF: too many bounding volumes, some contacts may be missed");
          mj_freeStack(d);
          return;
        }
        if (bvh_active) bvh_active[node] = 1;
      }
      continue;
    }

    // if no intersection at intermediate levels, stop
    if (!boxIntersect(bvh+6*node, offset, rotation, m, sdf, d)) {
      continue;
    }

    if (bvh_active) bvh_active[node] = 1;

    // recursive call
    for (int i=0; i < 2; i++) {
      if (child[2*node+i] != -1) {
        if (nstack >= max_stack) mjERROR("BVH stack depth exceeded.");
        stack[nstack].node = child[2*node+i];
        nstack++;
      }
    }
  }

  mj_freeStack(d);
}

//------------------------------ collision functions -----------------------------------------------

// collision between a height field and a signed distance field
int mjc_HFieldSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin) {
  mju_warning("HField vs SDF collision not yet supported!");
  return 0;
}

// collision between a mesh and a signed distance field
int mjc_MeshSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin) {
  mjtNum* pos1 = d->geom_xpos + 3 * g1;
  mjtNum* mat1 = d->geom_xmat + 9 * g1;

  mjtNum offset[3], rotation[9], corners[9], x[3], depth;
  mjtNum points[3*MAXSDFFACE], dist[MAXMESHPNT], candidate[3*MAXMESHPNT];
  int vertadr = m->mesh_vertadr[m->geom_dataid[g1]];
  int faceadr = m->mesh_faceadr[m->geom_dataid[g1]];
  int cnt=0, npoints=0, ncandidate=0, n0=0, faces[MAXSDFFACE]={-1}, index[MAXMESHPNT];

  // get sdf plugin
  int instance = m->geom_plugin[g2];
  const mjpPlugin* sdf_ptr = mjc_getSDF(m, g2);
  mjtGeom geomtype = mjGEOM_SDF;

  // copy into data
  mjSDF sdf;
  sdf.id = &instance;
  sdf.type = mjSDFTYPE_SINGLE;
  sdf.plugin = &sdf_ptr;
  sdf.geomtype = &geomtype;

  // compute transformation from g1 to g2
  mjtNum pos2true[3], sdf_quat[4], quat1[4];
  mju_mat2Quat(quat1, mat1);
  undoTransformation(m, d, g2, pos2true, sdf_quat);
  mapPose(pos1, quat1, pos2true, sdf_quat, offset, rotation);

  // binary tree search
  collideBVH(m, (mjData*)d, g1, offset, rotation, faces, &npoints, &n0, &sdf);

  // Frank-Wolfe algorithm
  for (int i=0; i < npoints; i++) {
    int face = faceadr + faces[i];
    for (int v=0; v < 3; v++) {
      mjtNum vec[3] = {
        m->mesh_vert[3*(vertadr+m->mesh_face[3*face+v])+0],
        m->mesh_vert[3*(vertadr+m->mesh_face[3*face+v])+1],
        m->mesh_vert[3*(vertadr+m->mesh_face[3*face+v])+2],
      };

      // transform local 1 (mesh) to local 2 (sdf)
      mju_mulMatVec3(corners+3*v, rotation, vec);
      mju_addTo3(corners+3*v, offset);
    }

    // stricter culling
    if (!triangleIntersect(corners, m, &sdf, (mjData*)d)) {
      continue;
    }

    // starting point
    x[0] = (corners[0]+corners[3]+corners[6])/3;
    x[1] = (corners[1]+corners[4]+corners[7])/3;
    x[2] = (corners[2]+corners[5]+corners[8])/3;

    // SHOULD NOT OCCUR
    if (ncandidate == MAXMESHPNT)mjERROR("too many contact points");

    // Frank-Wolfe
    depth = stepFrankWolfe(x, corners, 3, m, &sdf, (mjData*)d);

    // store candidate if there is penetration
    if (depth < 0) {
      mju_copy3(candidate + 3*ncandidate, x);
      index[ncandidate] = ncandidate;
      dist[ncandidate++] = depth;
    }
  }

  // sort contacts using depth
  if (ncandidate > 1) {
    int buf[MAXMESHPNT];
    distSort(index, buf, ncandidate, dist);
  }

  // add only the first mjMAXCONPAIR pairs
  for (int i=0; i < mju_min(ncandidate, mjMAXCONPAIR); i++) {
    cnt = addContact(points, con, candidate + 3*index[i], pos2true, sdf_quat,
                     dist[index[i]], cnt, m, &sdf, (mjData*)d);
  }

  return cnt;
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
  mjtNum pos1true[3], offset21[3], rotation21[9], rotation12[9];
  mjtNum pos2true[3], offset1[3], rotation1[9], offset12[3];
  mjtNum offset2[3], rotation2[9], squat1[4], squat2[4];
  undoTransformation(m, d, g1, pos1true, squat1);
  undoTransformation(m, d, g2, pos2true, squat2);
  mju_mat2Quat(quat1, mat1);
  mju_mat2Quat(quat2, mat2);
  mapPose(pos2, quat2, pos1, quat1, offset1, rotation1);
  mapPose(pos1, quat1, pos1true, squat1, offset2, rotation2);
  mapPose(pos2true, squat2, pos1true, squat1, offset21, rotation21);
  mapPose(pos1true, squat1, pos2true, squat2, offset12, rotation12);

  // axis-aligned bounding boxes in g1 frame
  for (int i=0; i < 8; i++) {
    vec1[0] = (i&1 ? size1[0]+size1[3] : size1[0]-size1[3]);
    vec1[1] = (i&2 ? size1[1]+size1[4] : size1[1]-size1[4]);
    vec1[2] = (i&4 ? size1[2]+size1[5] : size1[2]-size1[5]);

    vec2[0] = (i&1 ? size2[0]+size2[3] : size2[0]-size2[3]);
    vec2[1] = (i&2 ? size2[1]+size2[4] : size2[1]-size2[4]);
    vec2[2] = (i&4 ? size2[2]+size2[5] : size2[2]-size2[5]);

    mju_mulMatVec3(vec2, rotation1, vec2);
    mju_addTo3(vec2, offset1);

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
  sdf_ptr[0] = mjc_getSDF(m, g2);

  // get sdf plugins
  if (m->geom_type[g1] == mjGEOM_SDF) {
    instance[1] = m->geom_plugin[g1];
    sdf_ptr[1] = mjc_getSDF(m, g1);
  } else {
    instance[1] = g1;
    sdf_ptr[1] = NULL;
  }

  // reset visualization count
  sdf_ptr[0]->reset(m, NULL, (void*)(d->plugin_data[instance[0]]), instance[0]);

  // copy into sdf
  mjSDF sdf;
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
    sdf_ptr[0]->compute(m, (mjData*)d, instance[0], mjPLUGIN_SDF);

    // gradient descent - we use a special function of the two SDF as objective
    sdf.type = mjSDFTYPE_COLLISION;
    dist = stepGradient(x, m, &sdf, (mjData*)d, m->opt.sdf_iterations);

    // inexact SDFs can yield spurious collisions, filter them by projecting on the midsurface
    sdf.type = mjSDFTYPE_INTERSECTION;
    dist = stepGradient(x, m, &sdf, (mjData*)d, 1);

    // contact point and normal - we use the midsurface where SDF1=SDF2 as zero level set
    sdf.type = mjSDFTYPE_MIDSURFACE;
    cnt = addContact(contacts, con, x, pos2true, squat2, dist, cnt, m, &sdf, (mjData*)d);

    // SHOULD NOT OCCUR
    if (cnt > mjMAXCONPAIR) {
      mjERROR("too many contact points");
    }
  }

  return cnt;
}

