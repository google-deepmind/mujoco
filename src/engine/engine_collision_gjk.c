// Copyright 2024 DeepMind Technologies Limited
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

#include "engine/engine_collision_gjk.h"

#include <float.h>
#include <stddef.h>
#include <stdlib.h>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_convex.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"

// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static void subdistance(mjtNum lambda[4], int n, const mjtNum s1[3], const mjtNum s2[3],
                        const mjtNum s3[3], const mjtNum s4[3]);

// compute the barycentric coordinates of the closest point to the origin in the n-simplex,
// where n = 3, 2, 1 respectively
static void S3D(mjtNum lambda[4], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3],
                const mjtNum s4[3]);
static void S2D(mjtNum lambda[3], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3]);
static void S1D(mjtNum lambda[2], const mjtNum s1[3], const mjtNum s2[3]);

// compute the support point for GJK
static void gjkSupport(Vertex* v, mjCCDObj* obj1, mjCCDObj* obj2,
                       const mjtNum x_k[3]);

// compute the linear combination of 1 - 4 3D vectors
static inline void lincomb(mjtNum res[3], const mjtNum* coef, int n, const mjtNum v1[3],
                           const mjtNum v2[3], const mjtNum v3[3], const mjtNum v4[3]);

// one face in a polytope
typedef struct {
  int verts[3];      // indices of the three vertices of the face in the polytope
  int adj[3];        // adjacent faces, one for each edge: [v1,v2], [v2,v3], [v3,v1]
  mjtNum v[3];       // projection of the origin on face, can be used as face normal
  mjtNum dist;       // norm of v; negative if deleted
  int index;         // index in map; -1: not in map, -2: deleted from polytope
} Face;

// polytope used in the Expanding Polytope Algorithm (EPA)
typedef struct {
  Vertex* verts;     // list of vertices that make up the polytope
  int nverts;        // number of vertices
  Face* faces;       // list of faces that make up the polytope
  int nfaces;        // number of faces
  int maxfaces;      // max number of faces that can be stored in polytope
  Face** map;        // linear map storing faces
  int nmap;          // number of faces in map
} Polytope;

// compute the support point for EPA
static int epaSupport(Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2,
                      const mjtNum d[3], mjtNum dnorm);

// make copy of vertex in polytope and return its index
static int insertVertex(Polytope* pt, const Vertex* v);

// attach a face to the polytope with the given vertex indices; return distance to origin
static mjtNum attachFace(Polytope* pt, int v1, int v2, int v3, int adj1, int adj2, int adj3);

// return 1 if objects are in contact; 0 if not; -1 if inconclusive
// status must have initial tetrahedrons
static int gjkIntersect(mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);

// create initial polytope for EPA for a 2, 3, or 4-simplex
static int polytope2(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);
static int polytope3(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);
static int polytope4(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);

// return a face of the expanded polytope that best approximates the pentration depth
// witness points are in status->{x1, x2}
static Face* epa(mjCCDStatus* status, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2);

// -------------------------------- inlined  3D vector utils --------------------------------------

// v1 == v2 up to 1e-15
static inline int equal3(const mjtNum v1[3], const mjtNum v2[3]) {
  return mju_abs(v1[0] - v2[0]) < mjMINVAL &&
         mju_abs(v1[1] - v2[1]) < mjMINVAL &&
         mju_abs(v1[2] - v2[2]) < mjMINVAL;
}

// v1 == v2
static inline int equalexact3(const mjtNum v1[3], const mjtNum v2[3]) {
  return v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2];
}

// res = v1 + v2
static inline void add3(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3]) {
  res[0] = v1[0] + v2[0], res[1] = v1[1] + v2[1], res[2] = v1[2] + v2[2];
}

// res = v1 - v2
static inline void sub3(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3]) {
  res[0] = v1[0] - v2[0], res[1] = v1[1] - v2[1], res[2] = v1[2] - v2[2];
}

// dot product
static inline mjtNum dot3(const mjtNum v1[3], const mjtNum v2[3]) {
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

// res = v
static inline void copy3(mjtNum res[3], const mjtNum v[3]) {
  res[0] = v[0], res[1] = v[1], res[2] = v[2];
}

// scalar product: res = s*v
static inline void scl3(mjtNum res[3], const mjtNum v[3], mjtNum s) {
  res[0] = s*v[0], res[1] = s*v[1], res[2] = s*v[2];
}

// cross product: res = v1 x v2
static inline void cross3(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3]) {
  res[0] = v1[1]*v2[2] - v1[2]*v2[1];
  res[1] = v1[2]*v2[0] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

// return determinant of the 3x3 matrix with columns v1, v2, v3
static inline mjtNum det3(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]) {
  // v1 * (v2 x v3)
  return v1[0]*(v2[1]*v3[2] - v2[2]*v3[1])
       + v1[1]*(v2[2]*v3[0] - v2[0]*v3[2])
       + v1[2]*(v2[0]*v3[1] - v2[1]*v3[0]);
}

// ---------------------------------------- GJK ---------------------------------------------------


// return true if both geoms are discrete shapes (i.e. meshes or boxes with no margin)
static int discreteGeoms(mjCCDObj* obj1, mjCCDObj* obj2) {
  // non-zero margin makes geoms smooth
  if (obj1->margin != 0 || obj2->margin != 0) return 0;

  int g1 = obj1->geom_type;
  int g2 = obj2->geom_type;
  return (g1 == mjGEOM_MESH || g1 == mjGEOM_BOX || g1 == mjGEOM_HFIELD) &&
         (g2 == mjGEOM_MESH || g2 == mjGEOM_BOX || g2 == mjGEOM_HFIELD);
}



// GJK algorithm
static void gjk(mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  int get_dist = status->dist_cutoff > 0;  // need to recover geom distances if not in contact
  int backup_gjk = !get_dist;              // use gjkIntersect if no geom distances needed
  Vertex* simplex = status->simplex;
  int n = 0;                               // number of vertices in the simplex
  int k = 0;                               // current iteration
  int kmax = status->max_iterations;       // max number of iterations
  mjtNum* x1_k = status->x1;               // the kth approximation point for obj1
  mjtNum* x2_k = status->x2;               // the kth approximation point for obj2
  mjtNum x_k[3];                           // the kth approximation point in Minkowski difference
  mjtNum lambda[4] = {1, 0, 0, 0};         // barycentric coordinates for x_k
  mjtNum cutoff2 = status->dist_cutoff * status->dist_cutoff;

  // if both geoms are discrete, finite convergence is guaranteed; set tolerance to 0
  mjtNum epsilon = discreteGeoms(obj1, obj2) ? 0 : status->tolerance * status->tolerance;

  // set initial guess
  sub3(x_k, x1_k, x2_k);

  for (; k < kmax; k++) {
    // compute the kth support point
    gjkSupport(simplex + n, obj1, obj2, x_k);
    mjtNum *s_k = simplex[n].vert;

    // stopping criteria using the Frank-Wolfe duality gap given by
    //  |f(x_k) - f(x_min)|^2 <= < grad f(x_k), (x_k - s_k) >
    mjtNum diff[3];
    sub3(diff, x_k, s_k);
    if (2*dot3(x_k, diff) < epsilon) {
      if (!k) n = 1;
      break;
    }

    // if the hyperplane separates the Minkowski difference and origin, the objects don't collide
    // if geom distance isn't requested, return early
    if (!get_dist) {
      if (dot3(x_k, s_k) > 0) {
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist = mjMAXVAL;
        return;
      }
    } else if (status->dist_cutoff < mjMAXVAL) {
      mjtNum vs = mju_dot3(x_k, s_k), vv = mju_dot3(x_k, x_k);
      if (mju_dot3(x_k, s_k) > 0 && (vs*vs / vv) >= cutoff2) {
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist = mjMAXVAL;
        return;
      }
    }

    // tetrahedron is generated and only need contact info; fallback to gjkIntersect to
    // determine contact
    if (n == 3 && backup_gjk) {
      status->gjk_iterations = k;
      int ret = gjkIntersect(status, obj1, obj2);
      if (ret != -1) {
        status->nx = 0;
        status->dist = ret > 0 ? 0 : mjMAXVAL;
        return;
      }
      k = status->gjk_iterations;
      backup_gjk = 0;
    }

    // run the distance subalgorithm to compute the barycentric coordinates
    // of the closest point to the origin in the simplex
    subdistance(lambda, n + 1, simplex[0].vert, simplex[1].vert, simplex[2].vert, simplex[3].vert);

    // remove vertices from the simplex no longer needed
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (lambda[i] == 0) continue;
      simplex[n] = simplex[i];
      lambda[n++] = lambda[i];
    }

    // get the next iteration of x_k
    mjtNum x_next[3];
    lincomb(x_next, lambda, n, simplex[0].vert, simplex[1].vert,
            simplex[2].vert, simplex[3].vert);

    // x_k has converged to minimum
    if (equal3(x_next, x_k)) {
      break;
    }

    // copy next iteration into x_k
    copy3(x_k, x_next);

    // we have a tetrahedron containing the origin so return early
    if (n == 4) {
      break;
    }
  }

  // compute the approximate witness points
  lincomb(x1_k, lambda, n, simplex[0].vert1, simplex[1].vert1, simplex[2].vert1,
          simplex[3].vert1);
  lincomb(x2_k, lambda, n, simplex[0].vert2, simplex[1].vert2, simplex[2].vert2,
          simplex[3].vert2);

  status->nx = 1;
  status->gjk_iterations = k;
  status->nsimplex = n;
  status->dist = mju_norm3(x_k);
}



// compute the support point in obj1 and obj2 for Minkowski difference
static inline void support(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                           const mjtNum dir[3], const mjtNum dir_neg[3]) {
  // obj1
  obj1->support(s1, obj1, dir);
  if (obj1->margin > 0 && obj1->geom >= 0) {
    mjtNum margin = 0.5 * obj1->margin;
    s1[0] += dir[0] * margin;
    s1[1] += dir[1] * margin;
    s1[2] += dir[2] * margin;
  }

  // obj2
  obj2->support(s2, obj2, dir_neg);
  if (obj2->margin > 0 && obj2->geom >= 0) {
    mjtNum margin = 0.5 * obj2->margin;
    s2[0] += dir_neg[0] * margin;
    s2[1] += dir_neg[1] * margin;
    s2[2] += dir_neg[2] * margin;
  }
}



// compute the support points in obj1 and obj2 for the kth approximation point
static void gjkSupport(Vertex* v, mjCCDObj* obj1, mjCCDObj* obj2,
                       const mjtNum x_k[3]) {
  mjtNum dir[3] = {-1, 0, 0}, dir_neg[3] = {1, 0, 0};

  // mjc_support requires a normalized direction
  mjtNum norm = dot3(x_k, x_k);
  if (norm > mjMINVAL*mjMINVAL) {
    norm = 1/mju_sqrt(norm);
    scl3(dir_neg, x_k, norm);
    scl3(dir, dir_neg, -1);
  }

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  support(v->vert1, v->vert2, obj1, obj2, dir, dir_neg);
  sub3(v->vert, v->vert1, v->vert2);
  // copy mesh indices
  if (obj1->vertindex >= 0) {
    v->index1 = obj1->vertindex;
  }
  if (obj2->vertindex >= 0) {
    v->index2 = obj2->vertindex;
  }
}



// compute support points in Minkowski difference, return index of new vertex in polytope
static int epaSupport(Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2,
                     const mjtNum d[3], mjtNum dnorm) {
  mjtNum dir[3] = {1, 0, 0}, dir_neg[3] = {-1, 0, 0};

  // mjc_support assumes a normalized direction
  if (dnorm > mjMINVAL) {
    dir[0] = d[0] / dnorm;
    dir[1] = d[1] / dnorm;
    dir[2] = d[2] / dnorm;
    scl3(dir_neg, dir, -1);
  }

  int n = 3*pt->nverts++;
  Vertex* v = pt->verts + n;

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  support(v->vert1, v->vert2, obj1, obj2, dir, dir_neg);
  sub3(v->vert, v->vert1, v->vert2);
  if (obj1->vertindex >= 0) {
    v->index1 = obj1->vertindex;
  }
  if (obj2->vertindex >= 0) {
    v->index2 = obj2->vertindex;
  }
  return n;
}



// compute the support point in the Minkowski difference for gjkIntersect (without normalization)
static void gjkIntersectSupport(Vertex* v, mjCCDObj* obj1, mjCCDObj* obj2,
                                const mjtNum dir[3]) {
  mjtNum dir_neg[3] = {-dir[0], -dir[1], -dir[2]};
  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  support(v->vert1, v->vert2, obj1, obj2, dir, dir_neg);
  sub3(v->vert, v->vert1, v->vert2);
  if (obj1->vertindex >= 0) {
    v->index1 = obj1->vertindex;
  }
  if (obj2->vertindex >= 0) {
    v->index2 = obj2->vertindex;
  }
}



// compute the signed distance of a face along with the normal
static inline mjtNum signedDistance(mjtNum normal[3], const Vertex* v1, const Vertex* v2,
                                    const Vertex* v3) {
  mjtNum diff1[3], diff2[3];
  sub3(diff1, v3->vert, v1->vert);
  sub3(diff2, v2->vert, v1->vert);
  cross3(normal, diff1, diff2);
  mjtNum norm = dot3(normal, normal);
  if (norm > mjMINVAL*mjMINVAL && norm < mjMAXVAL*mjMAXVAL) {
    norm = 1/mju_sqrt(norm);
    scl3(normal, normal, norm);
    return dot3(normal, v1->vert);
  }
  return mjMAXVAL;  // cannot recover normal (ignore face)
}



// return 1 if objects are in contact; 0 if not; -1 if inconclusive
static int gjkIntersect(mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  Vertex simplex[4] = {status->simplex[0], status->simplex[1],
                       status->simplex[2], status->simplex[3]};
  int s[4] = {0, 1, 2, 3};

  int k = status->gjk_iterations, kmax = status->max_iterations;
  for (; k < kmax; k++) {
    // compute the signed distance to each face in the simplex along with normals
    mjtNum dist[4], normals[12];
    dist[0] = signedDistance(&normals[0], simplex + s[2], simplex + s[1], simplex + s[3]);
    dist[1] = signedDistance(&normals[3], simplex + s[0], simplex + s[2], simplex + s[3]);
    dist[2] = signedDistance(&normals[6], simplex + s[1], simplex + s[0], simplex + s[3]);
    dist[3] = signedDistance(&normals[9], simplex + s[0], simplex + s[1], simplex + s[2]);

    // if origin is on any affine hull, convergence will fail
    if (!dist[3] || !dist[2] || !dist[1] || !dist[0]) {
      status->gjk_iterations = k;
      return -1;
    }

    // find the face with the smallest distance to the origin
    int i = (dist[0] < dist[1]) ? 0 : 1;
    int j = (dist[2] < dist[3]) ? 2 : 3;
    int index = (dist[i] < dist[j]) ? i : j;

    // origin inside of simplex (run EPA for contact information)
    if (dist[index] > 0) {
      status->nsimplex = 4;
      status->simplex[0] = simplex[s[0]];
      status->simplex[1] = simplex[s[1]];
      status->simplex[2] = simplex[s[2]];
      status->simplex[3] = simplex[s[3]];
      status->gjk_iterations = k;
      return 1;
    }

    // replace worst vertex (farthest from origin) with new candidate
    gjkIntersectSupport(simplex + s[index], obj1, obj2, normals + 3*index);

    // found origin outside the Minkowski difference (return no collision)
    if (dot3(&normals[3*index], simplex[s[index]].vert) < 0) {
      status->nsimplex = 0;
      status->gjk_iterations = k;
      return 0;
    }

    // swap vertices in the simplex to retain orientation
    i = (index + 1) & 3;
    j = (index + 2) & 3;
    int swap = s[i];
    s[i] = s[j];
    s[j] = swap;
  }
  status->gjk_iterations = k;
  return -1;  // never found origin
}



// linear combination of n 3D vectors
static inline void lincomb(mjtNum res[3], const mjtNum* coef, int n, const mjtNum v1[3],
                           const mjtNum v2[3], const mjtNum v3[3], const mjtNum v4[3]) {
  switch (n) {
    case 1:
      res[0] = coef[0]*v1[0];
      res[1] = coef[0]*v1[1];
      res[2] = coef[0]*v1[2];
      break;
    case 2:
      res[0] = coef[0]*v1[0] + coef[1]*v2[0];
      res[1] = coef[0]*v1[1] + coef[1]*v2[1];
      res[2] = coef[0]*v1[2] + coef[1]*v2[2];
      break;
    case 3:
      res[0] = coef[0]*v1[0] + coef[1]*v2[0] + coef[2]*v3[0];
      res[1] = coef[0]*v1[1] + coef[1]*v2[1] + coef[2]*v3[1];
      res[2] = coef[0]*v1[2] + coef[1]*v2[2] + coef[2]*v3[2];
      break;
    case 4:
      res[0] = coef[0]*v1[0] + coef[1]*v2[0] + coef[2]*v3[0] + coef[3]*v4[0];
      res[1] = coef[0]*v1[1] + coef[1]*v2[1] + coef[2]*v3[1] + coef[3]*v4[1];
      res[2] = coef[0]*v1[2] + coef[1]*v2[2] + coef[2]*v3[2] + coef[3]*v4[2];
      break;
  }
}



// res = origin projected onto plane defined by v1, v2, v3
static int projectOriginPlane(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3],
                              const mjtNum v3[3]) {
  mjtNum diff21[3], diff31[3], diff32[3], n[3], nv, nn;
  sub3(diff21, v2, v1);
  sub3(diff31, v3, v1);
  sub3(diff32, v3, v2);

  // n = (v1 - v2) x (v3 - v2)
  cross3(n, diff32, diff21);
  nv = dot3(n, v2);
  nn = dot3(n, n);
  if (nn == 0) return 1;
  if (nv != 0 && nn > mjMINVAL) {
    scl3(res, n, nv / nn);
    return 0;
  }

  // n = (v2 - v1) x (v3 - v1)
  cross3(n, diff21, diff31);
  nv = dot3(n, v1);
  nn = dot3(n, n);
  if (nn == 0) return 1;
  if (nv != 0 && nn > mjMINVAL) {
    scl3(res, n, nv / nn);
    return 0;
  }

  // n = (v1 - v3) x (v2 - v3)
  cross3(n, diff31, diff32);
  nv = dot3(n, v3);
  nn = dot3(n, n);
  scl3(res, n, nv / nn);
  return 0;
}



// res = origin projected onto line defined by v1, v2
static inline void projectOriginLine(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3]) {
  // res = v2 - <v2, v2 - v1> / <v2 - v1, v2 - v1> * (v2 - v1)
  mjtNum diff[3];
  sub3(diff, v2, v1);
  mjtNum scl = -(dot3(v2, diff) / dot3(diff, diff));
  res[0] = v2[0] + scl*diff[0];
  res[1] = v2[1] + scl*diff[1];
  res[2] = v2[2] + scl*diff[2];
}



// return 1 if both numbers are positive, -1 if both negative and 0 otherwise
static inline int sameSign2(mjtNum a, mjtNum b) {
  if (a > 0 && b > 0) return 1;
  if (a < 0 && b < 0) return -1;
  return 0;
}



// return 1 if all three numbers are positive, -1 if all negative and 0 otherwise
static inline int sameSign3(mjtNum a, mjtNum b, mjtNum c) {
  if (a > 0 && b > 0 && c > 0) return 1;
  if (a < 0 && b < 0 && c < 0) return -1;
  return 0;
}



// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static inline void subdistance(mjtNum lambda[4], int n, const mjtNum s1[3],
                               const mjtNum s2[3], const mjtNum s3[3], const mjtNum s4[3]) {
  lambda[0] = lambda[1] = lambda[2] = lambda[3] = 0;
  if (n == 4) {
    S3D(lambda, s1, s2, s3, s4);
  } else if (n == 3) {
    S2D(lambda, s1, s2, s3);
  } else if (n == 2) {
    S1D(lambda, s1, s2);
  } else {
    lambda[0] = 1;
  }
}



static void S3D(mjtNum lambda[4], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3],
                const mjtNum s4[3]) {
  // the matrix M is given by
  //  [[ s1_x, s2_x, s3_x, s4_x ],
  //   [ s1_y, s2_y, s3_y, s4_y ],
  //   [ s1_z, s2_z, s3_z, s4_z ],
  //   [ 1,    1,    1,    1    ]]
  // we want to solve M*lambda = P, where P = [p_x, p_y, p_z, 1] with [p_x, p_y, p_z] is the
  // origin projected onto the simplex

  // compute cofactors to find det(M)
  mjtNum C41 = -det3(s2, s3, s4);
  mjtNum C42 =  det3(s1, s3, s4);
  mjtNum C43 = -det3(s1, s2, s4);
  mjtNum C44 =  det3(s1, s2, s3);

  // note that m_det = 6*SignVol(simplex) with C4i corresponding to the volume of the 3-simplex
  // with vertices {s1, s2, s3, 0} - si
  mjtNum m_det = C41 + C42 + C43 + C44;

  int comp1 = sameSign2(m_det, C41),
      comp2 = sameSign2(m_det, C42),
      comp3 = sameSign2(m_det, C43),
      comp4 = sameSign2(m_det, C44);

  // if all signs are the same then the origin is inside the simplex
  if (comp1 && comp2 && comp3 && comp4) {
    lambda[0] = C41 / m_det;
    lambda[1] = C42 / m_det;
    lambda[2] = C43 / m_det;
    lambda[3] = C44 / m_det;
    return;
  }

  // find the smallest distance, and use the corresponding barycentric coordinates
  mjtNum dmin = mjMAXVAL;

  if (!comp1) {
    mjtNum lambda_2d[3], x[3];
    S2D(lambda_2d, s2, s3, s4);
    lincomb(x, lambda_2d, 3, s2, s3, s4, NULL);
    mjtNum d = dot3(x, x);
    lambda[0] = 0;
    lambda[1] = lambda_2d[0];
    lambda[2] = lambda_2d[1];
    lambda[3] = lambda_2d[2];
    dmin = d;
  }

  if (!comp2) {
    mjtNum lambda_2d[3], x[3];
    S2D(lambda_2d, s1, s3, s4);
    lincomb(x, lambda_2d, 3, s1, s3, s4, NULL);
    mjtNum d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = 0;
      lambda[2] = lambda_2d[1];
      lambda[3] = lambda_2d[2];
      dmin = d;
    }
  }

  if (!comp3) {
    mjtNum lambda_2d[3], x[3];
    S2D(lambda_2d, s1, s2, s4);
    lincomb(x, lambda_2d, 3, s1, s2, s4, NULL);
    mjtNum d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = 0;
      lambda[3] = lambda_2d[2];
      dmin = d;
    }
  }

  if (!comp4) {
    mjtNum lambda_2d[3], x[3];
    S2D(lambda_2d, s1, s2, s3);
    lincomb(x, lambda_2d, 3, s1, s2, s3, NULL);
    mjtNum d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = lambda_2d[2];
      lambda[3] = 0;
    }
  }
}



static void S2D(mjtNum lambda[3], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3]) {
  // project origin onto affine hull of the simplex
  mjtNum p_o[3];
  if (projectOriginPlane(p_o, s1, s2, s3)) {
    S1D(lambda, s1, s2);
    lambda[2] = 0;
    return;
  }

  // Below are the minors M_i4 of the matrix M given by
  // [[ s1_x, s2_x, s3_x, s4_x ],
  //  [ s1_y, s2_y, s3_y, s4_y ],
  //  [ s1_z, s2_z, s3_z, s4_z ],
  //  [ 1,    1,    1,    1    ]]
  mjtNum M_14 = s2[1]*s3[2] - s2[2]*s3[1] - s1[1]*s3[2] + s1[2]*s3[1] + s1[1]*s2[2] - s1[2]*s2[1];
  mjtNum M_24 = s2[0]*s3[2] - s2[2]*s3[0] - s1[0]*s3[2] + s1[2]*s3[0] + s1[0]*s2[2] - s1[2]*s2[0];
  mjtNum M_34 = s2[0]*s3[1] - s2[1]*s3[0] - s1[0]*s3[1] + s1[1]*s3[0] + s1[0]*s2[1] - s1[1]*s2[0];

  // exclude the axis with the largest projection of the simplex using the computed minors
  mjtNum M_max = 0;
  mjtNum s1_2D[2], s2_2D[2], s3_2D[2], p_o_2D[2];
  mjtNum mu1 = mju_abs(M_14), mu2 = mju_abs(M_24), mu3 = mju_abs(M_34);
  if (mu1 >= mu2 && mu1 >= mu3) {
    M_max = M_14;
    s1_2D[0] = s1[1];
    s1_2D[1] = s1[2];

    s2_2D[0] = s2[1];
    s2_2D[1] = s2[2];

    s3_2D[0] = s3[1];
    s3_2D[1] = s3[2];

    p_o_2D[0] = p_o[1];
    p_o_2D[1] = p_o[2];
  } else if (mu2 >= mu3) {
    M_max = M_24;
    s1_2D[0] = s1[0];
    s1_2D[1] = s1[2];

    s2_2D[0] = s2[0];
    s2_2D[1] = s2[2];

    s3_2D[0] = s3[0];
    s3_2D[1] = s3[2];

    p_o_2D[0] = p_o[0];
    p_o_2D[1] = p_o[2];
  } else {
    M_max = M_34;
    s1_2D[0] = s1[0];
    s1_2D[1] = s1[1];

    s2_2D[0] = s2[0];
    s2_2D[1] = s2[1];

    s3_2D[0] = s3[0];
    s3_2D[1] = s3[1];

    p_o_2D[0] = p_o[0];
    p_o_2D[1] = p_o[1];
  }

  // compute the cofactors C3i of the following matrix:
  // [[ s1_2D[0] - p_o_2D[0], s2_2D[0] - p_o_2D[0], s3_2D[0] - p_o_2D[0] ],
  //  [ s1_2D[1] - p_o_2D[1], s2_2D[1] - p_o_2D[1], s3_2D[1] - p_o_2D[1] ],
  //  [ 1,                    1,                    1                    ]]

  // C31 corresponds to the signed area of 2-simplex: (p_o_2D, s2_2D, s3_2D)
  mjtNum C31 = p_o_2D[0]*s2_2D[1] + p_o_2D[1]*s3_2D[0] + s2_2D[0]*s3_2D[1]
             - p_o_2D[0]*s3_2D[1] - p_o_2D[1]*s2_2D[0] - s3_2D[0]*s2_2D[1];

  // C32 corresponds to the signed area of 2-simplex: (_po_2D, s1_2D, s3_2D)
  mjtNum C32 = p_o_2D[0]*s3_2D[1] + p_o_2D[1]*s1_2D[0] + s3_2D[0]*s1_2D[1]
             - p_o_2D[0]*s1_2D[1] - p_o_2D[1]*s3_2D[0] - s1_2D[0]*s3_2D[1];

  // C33 corresponds to the signed area of 2-simplex: (p_o_2D, s1_2D, s2_2D)
  mjtNum C33 = p_o_2D[0]*s1_2D[1] + p_o_2D[1]*s2_2D[0] + s1_2D[0]*s2_2D[1]
             - p_o_2D[0]*s2_2D[1] - p_o_2D[1]*s1_2D[0] - s2_2D[0]*s1_2D[1];

  int comp1 = sameSign2(M_max, C31),
      comp2 = sameSign2(M_max, C32),
      comp3 = sameSign2(M_max, C33);

  // all the same sign, p_o is inside the 2-simplex
  if (comp1 && comp2 && comp3) {
    lambda[0] = C31 / M_max;
    lambda[1] = C32 / M_max;
    lambda[2] = C33 / M_max;
    return;
  }

  // find the smallest distance, and use the corresponding barycentric coordinates
  mjtNum dmin = mjMAXVAL;

  if (!comp1) {
    mjtNum lambda_1d[2], x[3];
    S1D(lambda_1d, s2, s3);
    lincomb(x, lambda_1d, 2, s2, s3, NULL, NULL);
    mjtNum d = dot3(x, x);
    lambda[0] = 0;
    lambda[1] = lambda_1d[0];
    lambda[2] = lambda_1d[1];
    dmin = d;
  }

  if (!comp2) {
    mjtNum lambda_1d[2], x[3];
    S1D(lambda_1d, s1, s3);
    lincomb(x, lambda_1d, 2, s1, s3, NULL, NULL);
    mjtNum d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_1d[0];
      lambda[1] = 0;
      lambda[2] = lambda_1d[1];
      dmin = d;
    }
  }

  if (!comp3) {
    mjtNum lambda_1d[2], x[3];
    S1D(lambda_1d, s1, s2);
    lincomb(x, lambda_1d, 2, s1, s2, NULL, NULL);
    mjtNum d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_1d[0];
      lambda[1] = lambda_1d[1];
      lambda[2] = 0;
    }
  }
}



static void S1D(mjtNum lambda[2], const mjtNum s1[3], const mjtNum s2[3]) {
  // find projection of origin onto the 1-simplex:
  mjtNum p_o[3];
  projectOriginLine(p_o, s1, s2);

  // find the axis with the largest projection "shadow" of the simplex
  mjtNum mu_max = 0;
  int index;
  for (int i = 0; i < 3; i++) {
    mjtNum mu = s1[i] - s2[i];
    if (mju_abs(mu) >= mju_abs(mu_max)) {
      mu_max = mu;
      index = i;
    }
  }

  mjtNum C1 = p_o[index] - s2[index];
  mjtNum C2 = s1[index] - p_o[index];

  // inside the simplex
  if (sameSign2(mu_max, C1) && sameSign2(mu_max, C2)) {
    lambda[0] = C1 / mu_max;
    lambda[1] = C2 / mu_max;
  } else {
    lambda[0] = 0;
    lambda[1] = 1;
  }
}


// ---------------------------------------- EPA ---------------------------------------------------

// replace a 3-simplex with one of its faces
static inline void replaceSimplex3(Polytope* pt, mjCCDStatus* status, int v1, int v2, int v3) {
  status->nsimplex = 3;
  Vertex* v = pt->verts;
  copy3(status->simplex[0].vert1, v[v1].vert1);
  copy3(status->simplex[1].vert1, v[v2].vert1);
  copy3(status->simplex[2].vert1, v[v3].vert1);

  copy3(status->simplex[0].vert2, v[v1].vert2);
  copy3(status->simplex[1].vert2, v[v2].vert2);
  copy3(status->simplex[2].vert2, v[v3].vert2);

  copy3(status->simplex[0].vert, v[v1].vert);
  copy3(status->simplex[1].vert, v[v2].vert);
  copy3(status->simplex[2].vert, v[v3].vert);

  pt->nfaces = 0;
  pt->nverts = 0;
}



// return 1 if the origin and p3 are on the same side of the plane defined by p0, p1, p2
static int sameSide(const mjtNum p0[3], const mjtNum p1[3],
                    const mjtNum p2[3], const mjtNum p3[3]) {
    mjtNum diff1[3], diff2[3], diff3[3], diff4[3], n[3];
    sub3(diff1, p1, p0);
    sub3(diff2, p2, p0);
    cross3(n, diff1, diff2);

    sub3(diff3, p3, p0);
    mjtNum dot1 = dot3(n, diff3);

    scl3(diff4, p0, -1);
    mjtNum dot2 = dot3(n, diff4);
    if (dot1 > 0 && dot2 > 0) return 1;
    if (dot1 < 0 && dot2 < 0) return 1;
    return 0;
}



// return 1 if the origin is contained in the tetrahedron, 0 otherwise
static int testTetra(const mjtNum p0[3], const mjtNum p1[3],
                     const mjtNum p2[3], const mjtNum p3[3]) {
  return sameSide(p0, p1, p2, p3)
      && sameSide(p1, p2, p3, p0)
      && sameSide(p2, p3, p0, p1)
      && sameSide(p3, p0, p1, p2);
}



// matrix for 120 degrees rotation around given axis
static void rotmat(mjtNum R[9], const mjtNum axis[3]) {
  mjtNum n = mju_norm3(axis);
  mjtNum u1 = axis[0] / n, u2 = axis[1] / n, u3 = axis[2] / n;
  const mjtNum sin = 0.86602540378;  // sin(120 deg)
  const mjtNum cos = -0.5;           // cos(120 deg)
  R[0] = cos + u1*u1*(1 - cos);
  R[1] = u1*u2*(1 - cos) - u3*sin;
  R[2] = u1*u3*(1 - cos) + u2*sin;
  R[3] = u2*u1*(1 - cos) + u3*sin;
  R[4] = cos + u2*u2*(1 - cos);
  R[5] = u2*u3*(1 - cos) - u1*sin;
  R[6] = u1*u3*(1 - cos) - u2*sin;
  R[7] = u2*u3*(1 - cos) + u1*sin;
  R[8] = cos + u3*u3*(1 - cos);
}



// create a polytope from a 1-simplex (returns 0 on success)
static int polytope2(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum *v1 = status->simplex[0].vert, *v2 = status->simplex[1].vert;

  mjtNum diff[3];
  sub3(diff, v2, v1);

  // find component with smallest magnitude (so cross product is largest)
  mjtNum value = mjMAXVAL;
  int index = 0;
  for (int i = 0; i < 3; i++) {
    if (mju_abs(diff[i]) < value) {
      value = mju_abs(diff[i]);
      index = i;
    }
  }

  // cross product with best coordinate axis
  mjtNum e[3] = {0, 0, 0};
  e[index] = 1;
  mjtNum d1[3], d2[3], d3[3];
  cross3(d1, e, diff);

  // rotate around the line segment to get three more points spaced 120 degrees apart
  mjtNum R[9];
  rotmat(R, diff);

  mju_mulMatVec3(d2, R, d1);
  mju_mulMatVec3(d3, R, d2);

  // save vertices and get indices for each one
  int v1i = insertVertex(pt, status->simplex + 0);
  int v2i = insertVertex(pt, status->simplex + 1);
  int v3i = epaSupport(pt, obj1, obj2, d1, mju_norm3(d1));
  int v4i = epaSupport(pt, obj1, obj2, d2, mju_norm3(d2));
  int v5i = epaSupport(pt, obj1, obj2, d3, mju_norm3(d3));

  mjtNum* v3 = pt->verts[v3i].vert;
  mjtNum* v4 = pt->verts[v4i].vert;
  mjtNum* v5 = pt->verts[v5i].vert;

  // build hexahedron
  if (attachFace(pt, v1i, v3i, v4i, 1, 3, 2) < mjMINVAL) {
    replaceSimplex3(pt, status, v1i, v3i, v4i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1i, v5i, v3i, 2, 4, 0) < mjMINVAL) {
    replaceSimplex3(pt, status, v1i, v5i, v3i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1i, v4i, v5i, 0, 5, 1) < mjMINVAL) {
    replaceSimplex3(pt, status, v1i, v4i, v5i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v4i, v3i, 5, 0, 4) < mjMINVAL) {
    replaceSimplex3(pt, status, v2i, v4i, v3i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v3i, v5i, 3, 1, 5) < mjMINVAL) {
    replaceSimplex3(pt, status, v2i, v3i, v5i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v5i, v4i, 4, 2, 3) < mjMINVAL) {
    replaceSimplex3(pt, status, v2i, v5i, v4i);
    return polytope3(pt, status, obj1, obj2);
  }

  // check that origin is in the hexahedron
  if (status->dist > 10*mjMINVAL && !testTetra(v1, v3, v4, v5) && !testTetra(v2, v3, v4, v5)) {
    return mjEPA_P2_MISSING_ORIGIN;
  }

  for (int i = 0; i < 6; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
  }
  pt->nmap = 6;

  // valid hexahedron for EPA
  return 0;
}



// compute the affine coordinates of p on the triangle v1v2v3
static void triAffineCoord(mjtNum lambda[3], const mjtNum v1[3], const mjtNum v2[3],
                           const mjtNum v3[3], const mjtNum p[3]) {
  // compute minors as in S2D
  mjtNum M_14 = v2[1]*v3[2] - v2[2]*v3[1] - v1[1]*v3[2] + v1[2]*v3[1] + v1[1]*v2[2] - v1[2]*v2[1];
  mjtNum M_24 = v2[0]*v3[2] - v2[2]*v3[0] - v1[0]*v3[2] + v1[2]*v3[0] + v1[0]*v2[2] - v1[2]*v2[0];
  mjtNum M_34 = v2[0]*v3[1] - v2[1]*v3[0] - v1[0]*v3[1] + v1[1]*v3[0] + v1[0]*v2[1] - v1[1]*v2[0];

  // exclude one of the axes with the largest projection of the simplex using the computed minors
  mjtNum M_max = 0;
  int x, y;
  mjtNum mu1 = mju_abs(M_14), mu2 = mju_abs(M_24), mu3 = mju_abs(M_34);
  if (mu1 >= mu2 && mu1 >= mu3) {
    M_max = M_14;
    x = 1;
    y = 2;
  } else if (mu2 >= mu3) {
    M_max = M_24;
    x = 0;
    y = 2;
  } else {
    M_max = M_34;
    x = 0;
    y = 1;
  }

  // C31 corresponds to the signed area of 2-simplex: (v, s2, s3)
  mjtNum C31 = p[x]*v2[y] + p[y]*v3[x] + v2[x]*v3[y]
             - p[x]*v3[y] - p[y]*v2[x] - v3[x]*v2[y];

  // C32 corresponds to the signed area of 2-simplex: (v, s1, s3)
  mjtNum C32 = p[x]*v3[y] + p[y]*v1[x] + v3[x]*v1[y]
             - p[x]*v1[y] - p[y]*v3[x] - v1[x]*v3[y];

  // C33 corresponds to the signed area of 2-simplex: (v, s1, s2)
  mjtNum C33 = p[x]*v1[y] + p[y]*v2[x] + v1[x]*v2[y]
             - p[x]*v2[y] - p[y]*v1[x] - v2[x]*v1[y];

  // compute affine coordinates
  lambda[0] = C31 / M_max;
  lambda[1] = C32 / M_max;
  lambda[2] = C33 / M_max;
}



// return true if point p and triangle v1v2v3 intersect
static int triPointIntersect(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3],
                             const mjtNum p[3]) {
  mjtNum lambda[3];
  triAffineCoord(lambda, v1, v2, v3, p);
  if (lambda[0] < 0 || lambda[1] < 0 || lambda[2] < 0) {
    return 0;
  }
  mjtNum pr[3], diff[3];
  pr[0] = v1[0]*lambda[0] + v2[0]*lambda[1] + v3[0]*lambda[2];
  pr[1] = v1[1]*lambda[0] + v2[1]*lambda[1] + v3[1]*lambda[2];
  pr[2] = v1[2]*lambda[0] + v2[2]*lambda[1] + v3[2]*lambda[2];
  sub3(diff, pr, p);
  return mju_norm3(diff) < mjMINVAL;
}



// create a polytope from a 2-simplex (returns 0 on success)
static int polytope3(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  // get vertices of simplex from GJK
  const mjtNum *v1 = status->simplex[0].vert,
               *v2 = status->simplex[1].vert,
               *v3 = status->simplex[2].vert;

  // get normals in both directions
  mjtNum diff1[3], diff2[3], n[3], n_neg[3];
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(n, diff1, diff2);
  mjtNum n_norm = mju_norm3(n);
  if (n_norm < mjMINVAL) {
    return mjEPA_P3_BAD_NORMAL;
  }

  // negative of triangle normal n
  scl3(n_neg, n, -1);

  // save vertices and get indices for each one
  int v1i = insertVertex(pt, status->simplex + 0);
  int v2i = insertVertex(pt, status->simplex + 1);
  int v3i = insertVertex(pt, status->simplex + 2);
  int v5i = epaSupport(pt, obj1, obj2, n_neg, n_norm);
  int v4i = epaSupport(pt, obj1, obj2, n, n_norm);
  mjtNum* v4 = pt->verts[v4i].vert;
  mjtNum* v5 = pt->verts[v5i].vert;

  // check that v4 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v4)) {
    return mjEPA_P3_INVALID_V4;
  }

  // check that v5 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v5)) {
    return mjEPA_P3_INVALID_V5;
  }

  // if origin does not lie on simplex then we need to check that the hexahedron contains the
  // origin
  //
  // TODO(kylebayes): It's possible for GJK to return a 2-simplex with the origin not contained in
  // it but within tolerance from it. In that case the hexahedron could possibly be constructed
  // that doesn't contain the origin, but nonetheless there is penetration depth.
  if (status->dist > 10*mjMINVAL && !testTetra(v1, v2, v3, v4) && !testTetra(v1, v2, v3, v5)) {
    return mjEPA_P3_MISSING_ORIGIN;
  }

  // create hexahedron for EPA
  attachFace(pt, v4i, v1i, v2i, 1, 3, 2);
  attachFace(pt, v4i, v3i, v1i, 2, 4, 0);
  attachFace(pt, v4i, v2i, v3i, 0, 5, 1);
  attachFace(pt, v5i, v2i, v1i, 5, 0, 4);
  attachFace(pt, v5i, v1i, v3i, 3, 1, 5);
  attachFace(pt, v5i, v3i, v2i, 4, 2, 3);


  // if the origin is on the affine hull of any of the faces then the origin is not in the
  //  hexahedron or the hexahedron is degenerate
  for (int i = 0; i < 6; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
    if (pt->faces[i].dist < mjMINVAL) {
      return mjEPA_P3_ORIGIN_ON_FACE;
    }
  }
  pt->nmap = 6;
  return 0;
}



// create a polytope from a 3-simplex (returns 0 on success)
static int polytope4(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  int v1 = insertVertex(pt, status->simplex + 0);
  int v2 = insertVertex(pt, status->simplex + 1);
  int v3 = insertVertex(pt, status->simplex + 2);
  int v4 = insertVertex(pt, status->simplex + 3);

  // if the origin is on a face, replace the 3-simplex with a 2-simplex
  if (attachFace(pt, v1, v2, v3, 1, 3, 2) < mjMINVAL) {
    replaceSimplex3(pt, status, v1, v2, v3);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v4, v2, 2, 3, 0) < mjMINVAL) {
    replaceSimplex3(pt, status, v1, v4, v2);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v3, v4, 0, 3, 1) < mjMINVAL) {
    replaceSimplex3(pt, status, v1, v3, v4);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v4, v3, v2, 2, 0, 1) < mjMINVAL) {
    replaceSimplex3(pt, status, v4, v3, v2);
    return polytope3(pt, status, obj1, obj2);
  }

  if (!testTetra(pt->verts[v1].vert, pt->verts[v2].vert, pt->verts[v3].vert, pt->verts[v4].vert)) {
    return mjEPA_P4_MISSING_ORIGIN;
  }

  for (int i = 0; i < 4; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
  }
  pt->nmap = 4;
  return 0;
}



// make a copy of vertex in polytope and return its index
static inline int insertVertex(Polytope* pt, const Vertex* v) {
  int n = 3*pt->nverts++;
  Vertex* new_v = pt->verts + n;
  copy3(new_v->vert1, v->vert1);
  copy3(new_v->vert2, v->vert2);
  new_v->index1 = v->index1;
  new_v->index2 = v->index2;
  sub3(new_v->vert, v->vert1, v->vert2);
  return n;
}


// delete face from map (return non-zero on error)
static void deleteFace(Polytope* pt, Face* face) {
  if (face->index >= 0) {
    pt->map[face->index] = pt->map[--pt->nmap];
    pt->map[face->index]->index = face->index;
  }
  face->index = -2;  // mark face as deleted from map and polytope
}



// return max number of faces that can be stored in polytope
static inline int maxFaces(Polytope* pt) {
  return pt->maxfaces - pt->nfaces;
}



// attach a face to the polytope with the given vertex indices; return distance to origin
static inline mjtNum attachFace(Polytope* pt, int v1, int v2, int v3,
                                int adj1, int adj2, int adj3) {
  Face* face = &pt->faces[pt->nfaces++];
  face->verts[0] = v1;
  face->verts[1] = v2;
  face->verts[2] = v3;

  // adjacent faces
  face->adj[0] = adj1;
  face->adj[1] = adj2;
  face->adj[2] = adj3;

  // compute witness point v
  int ret = projectOriginPlane(face->v, pt->verts[v3].vert, pt->verts[v2].vert, pt->verts[v1].vert);
  if (ret) return 0;
  face->dist = mju_sqrt(dot3(face->v, face->v));
  face->index = -1;

  return face->dist;
}



// horizon: polytope boundary edges that can be seen from w
typedef struct {
  Polytope* pt;  // polytope for which the horizon is defined
  int* indices;  // indices of faces on horizon
  int* edges;    // corresponding edge of each face on the horizon
  int nedges;    // number of edges in horizon
  mjtNum* w;     // point where horizon is created
} Horizon;



// add an edge to the horizon
static inline void addEdge(Horizon* h, int index, int edge) {
  h->edges[h->nedges] = edge;
  h->indices[h->nedges++] = index;
}



// get edge index where vertex lies
static inline int getEdge(Face* face, int vertex) {
  if (face->verts[0] == vertex) return 0;
  if (face->verts[1] == vertex) return 1;
  return 2;
}



// recursive call to build horizon; return 1 if face is visible from w otherwise 0
static int horizonRec(Horizon* h, Face* face, int e) {
    mjtNum dist2 = face->dist * face->dist;

    // v is visible from w so it is deleted and adjacent faces are checked
    if (dot3(face->v, h->w) >= dist2) {
      deleteFace(h->pt, face);

      // recursively search the adjacent faces on the next two edges
      for (int k = 1; k < 3; k++) {
        int i = (e + k) % 3;
        Face* adjFace = &h->pt->faces[face->adj[i]];
        if (adjFace->index > -2) {
          int adjEdge = getEdge(adjFace, face->verts[(i + 1) % 3]);
          if (!horizonRec(h, adjFace, adjEdge)) {
            addEdge(h, face->adj[i], adjEdge);
          }
        }
      }
      return 1;
    }
  return 0;
}



// create horizon given the face as starting point
static void horizon(Horizon* h, Face* face) {
  deleteFace(h->pt, face);

  // first edge
  Face* adjFace = &h->pt->faces[face->adj[0]];
  int adjEdge = getEdge(adjFace, face->verts[1]);
  if (!horizonRec(h, adjFace, adjEdge)) {
    addEdge(h, face->adj[0], adjEdge);
  }

  // second edge
  adjFace = &h->pt->faces[face->adj[1]];
  adjEdge = getEdge(adjFace, face->verts[2]);
  if (adjFace->index > -2 && !horizonRec(h, adjFace, adjEdge)) {
    addEdge(h, face->adj[1], adjEdge);
  }

  // third edge
  adjFace = &h->pt->faces[face->adj[2]];
  adjEdge = getEdge(adjFace, face->verts[0]);
  if (adjFace->index > -2 && !horizonRec(h, adjFace, adjEdge)) {
    addEdge(h, face->adj[2], adjEdge);
  }
}



// recover witness points from EPA polytope
static void epaWitness(const Polytope* pt, const Face* face, mjtNum x1[3], mjtNum x2[3]) {
  // compute affine coordinates for witness points on plane defined by face
  mjtNum lambda[3];
  mjtNum* v1 = pt->verts[face->verts[0]].vert;
  mjtNum* v2 = pt->verts[face->verts[1]].vert;
  mjtNum* v3 = pt->verts[face->verts[2]].vert;
  triAffineCoord(lambda, v1, v2, v3, face->v);

  // face on geom 1
  v1 = pt->verts[face->verts[0]].vert1;
  v2 = pt->verts[face->verts[1]].vert1;
  v3 = pt->verts[face->verts[2]].vert1;
  x1[0] = v1[0]*lambda[0] + v2[0]*lambda[1] + v3[0]*lambda[2];
  x1[1] = v1[1]*lambda[0] + v2[1]*lambda[1] + v3[1]*lambda[2];
  x1[2] = v1[2]*lambda[0] + v2[2]*lambda[1] + v3[2]*lambda[2];

  // face on geom 2
  v1 = pt->verts[face->verts[0]].vert2;
  v2 = pt->verts[face->verts[1]].vert2;
  v3 = pt->verts[face->verts[2]].vert2;
  x2[0] = v1[0]*lambda[0] + v2[0]*lambda[1] + v3[0]*lambda[2];
  x2[1] = v1[1]*lambda[0] + v2[1]*lambda[1] + v3[1]*lambda[2];
  x2[2] = v1[2]*lambda[0] + v2[2]*lambda[1] + v3[2]*lambda[2];
}



// return a face of the expanded polytope that best approximates the pentration depth
// witness points are in status->{x1, x2}
static Face* epa(mjCCDStatus* status, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum tolerance = status->tolerance, lower, upper = FLT_MAX;
  int k, kmax = status->max_iterations;
  mjData* d = (mjData*) obj1->data;
  Face* face, *pface;  // face closest to origin

  // initialize horizon
  Horizon h;
  mj_markStack(d);
  h.indices = mjSTACKALLOC(d, 6 + status->max_iterations, int);
  h.edges = mjSTACKALLOC(d, 6 + status->max_iterations, int);
  h.nedges = 0;
  h.pt = pt;

  for (k = 0; k < kmax; k++) {
    pface = face;

    // find the face closest to the origin (lower bound for penetration depth)
    lower = FLT_MAX;
    for (int i = 0; i < pt->nmap; i++) {
      if (pt->map[i]->dist < lower) {
        face = pt->map[i];
        lower = face->dist;
      }
    }

    // face not valid, return previous face
    if (lower > upper) {
      face = pface;
      break;
    }

    // check if lower bound is 0
    if (lower <= 0) {
      mju_warning("EPA: origin lies on affine hull of face");
      break;
    }

    // compute support point w from the closest face's normal
    int wi = epaSupport(pt, obj1, obj2, face->v, lower);
    mjtNum* w = pt->verts[wi].vert;
    mjtNum upper_k = dot3(face->v, w) / lower;  // upper bound for kth iteration
    if (upper_k < upper) upper = upper_k;
    if (upper - lower < tolerance) {
      break;
    }

    h.w = w;
    horizon(&h, face);

    // unrecoverable numerical issue; at least one face was deleted so nedges is 3 or more
    if (h.nedges < 3) {
      mj_freeStack(d);
      status->epa_iterations = k;
      status->nx = 0;
      status->dist = 0;
      return NULL;
    }

    // insert w as new vertex and attach faces along the horizon
    int nfaces = pt->nfaces, nedges = h.nedges;

    // check if there's enough memory to store new faces
    if (nedges > maxFaces(pt)) {
      mju_warning("EPA: out of memory for faces on expanding polytope");
      break;
    }

    // attach first face
    int horIndex = h.indices[0], horEdge = h.edges[0];
    Face* horFace = &pt->faces[horIndex];
    int v1 = horFace->verts[horEdge],
        v2 = horFace->verts[(horEdge + 1) % 3];
    horFace->adj[horEdge] = nfaces;
    mjtNum dist = attachFace(pt, wi, v2, v1, nfaces + nedges - 1, horIndex, nfaces + 1);

    // unrecoverable numerical issue
    if (dist == 0) {
      mj_freeStack(d);
      status->epa_iterations = k;
      status->nx = 0;
      status->dist = 0;
      return NULL;
    }

    // store face in map
    if (dist >= lower && dist <= upper) {
      int i = pt->nmap++;
      pt->map[i] = &pt->faces[pt->nfaces - 1];
      pt->map[i]->index = i;
    }

    // attach remaining faces
    for (int i = 1; i < nedges; i++) {
      int cur = nfaces + i;                  // index of attached face
      int next = nfaces + (i + 1) % nedges;  // index of next face

      horIndex = h.indices[i], horEdge = h.edges[i];
      horFace = &pt->faces[horIndex];
      v1 = horFace->verts[horEdge];
      v2 = horFace->verts[(horEdge + 1) % 3];
      horFace->adj[horEdge] = cur;
      dist = attachFace(pt, wi, v2, v1, cur - 1, horIndex, next);

      // unrecoverable numerical issue
      if (dist == 0) {
        mj_freeStack(d);
        status->epa_iterations = k;
        status->nx = 0;
        status->dist = 0;
        return NULL;
      }

      // store face in map
      if (dist >= lower && dist <= upper) {
        int idx = pt->nmap++;
        pt->map[idx] = &pt->faces[pt->nfaces - 1];
        pt->map[idx]->index = idx;
      }
    }
    h.nedges = 0;  // clear horizon

    // no face candidates left
    if (!pt->nmap) {
      break;
    }
  }

  mj_freeStack(d);
  epaWitness(pt, face, status->x1, status->x2);
  status->epa_iterations = k;
  status->nx = 1;
  status->dist = -face->dist;
  return face;
}


// ------------------------------------- MultiCCD -------------------------------------------------

// find the normal of a plane perpendicular to the face (given by its normal n) and intersecting the
// face edge (v1, v2)
static mjtNum planeNormal(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3],
                          const mjtNum n[3]) {
  mjtNum v3[3], diff1[3], diff2[3];
  add3(v3, v1, n);
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(res, diff1, diff2);
  return dot3(res, v1);
}



// find what side of a plane a point p lies
static int halfspace(const mjtNum a[3], const mjtNum n[3], const mjtNum p[3]) {
  mjtNum diff[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
  return dot3(diff, n) > 0;
}



// compute the intersection of a plane with a line segment (a, b)
static mjtNum planeIntersect(mjtNum res[3], const mjtNum pn[3], mjtNum pd,
                             const mjtNum a[3], const mjtNum b[3]) {
  mjtNum ab[3];
  sub3(ab, b, a);
  mjtNum temp = dot3(pn, ab);
  if (temp == 0.0) return mjMAXVAL;  // parallel; no intersection
  mjtNum t = (pd - dot3(pn, a)) / temp;
  if (t >= 0.0 && t <= 1.0) {
    res[0] = a[0] + t*ab[0];
    res[1] = a[1] + t*ab[1];
    res[2] = a[2] + t*ab[2];
  }
  return t;
}



// clip a polygon against another polygon
static void polygonClip(mjCCDStatus* status, const mjtNum face1[3 * mjMAX_SIDES], int nface1,
                       const mjtNum face2[3 * mjMAX_SIDES], int nface2, const mjtNum n[3],
                       const mjtNum dir[3]) {
  // compute plane normal and distance to plane for each vertex
  mjtNum pn[3 * mjMAX_SIDES], pd[mjMAX_SIDES];
  for (int i = 0; i < nface1 - 1; i++) {
    pd[i] = planeNormal(&pn[3*i], &face1[3*i], &face1[3*i + 3], n);
  }
  pd[nface1 - 1] = planeNormal(&pn[3*(nface1 - 1)], &face1[3*(nface1 - 1)], &face1[0], n);

  // reserve 2 * max_sides as max sides for a clipped polygon
  mjtNum polygon1[6 * mjMAX_SIDES], polygon2[6 * mjMAX_SIDES], *polygon, *clipped;
  int npolygon = nface2, nclipped = 0;
  polygon = polygon1;
  clipped = polygon2;

  for (int i = 0; i < nface2; i++) {
    copy3(polygon + 3*i, face2 + 3*i);
  }

  // clip the polygon by one edge e at a time
  for (int e = 0; e < (3 * nface1); e += 3) {
    for (int i = 0; i < npolygon; i++) {
      // get edge PQ of the polygon
      mjtNum *P = polygon + 3*i;
      mjtNum *Q = (i < npolygon - 1) ? polygon + 3*(i+1) : polygon;

      // determine if P and Q are in the halfspace of the clipping edge
      int inside1 = halfspace(face1 + e, pn + e, P);
      int inside2 = halfspace(face1 + e, pn + e, Q);

      // PQ entirely outside the clipping edge, skip
      if (!inside1 && !inside2) {
        continue;
      }

      // edge PQ is inside the clipping edge, add Q
      if (inside1 && inside2) {
        copy3(clipped + 3*nclipped++, Q);
        continue;
      }

      // add new vertex to clipped polygon where PQ intersects the clipping edge
      mjtNum t = planeIntersect(clipped + 3*nclipped++, pn + e, pd[e/3], P, Q);
      if (t < 0.0 || t > 1.0) {
        nclipped--;  // no intersection in PQ
      }

      // add Q as PQ is now back inside the clipping edge
      if (inside2) {
        copy3(clipped + 3*nclipped++, Q);
      }
    }

    // swap clipped and polygon
    mjtNum* tmp = polygon;
    polygon = clipped;
    clipped = tmp;
    npolygon = nclipped;
    nclipped = 0;
  }

  // copy final clipped polygon to status
  if (npolygon > 0) {
    status->nx = npolygon;
    for (int i = 0; i < 3*npolygon; i += 3) {
      copy3(status->x2 + i, polygon + i);
      sub3(status->x1 + i, status->x2 + i, dir);
    }
  }
}



// compute local coordinates of a global point (g1, g2, g3)
static inline void localcoord(mjtNum res[3], const mjtNum mat[9], const mjtNum pos[3],
                              mjtNum g1, mjtNum g2, mjtNum g3) {
  // perform matT * ((g1, g2, g3) - pos)
  if (pos) {
    g1 -= pos[0];
    g2 -= pos[1];
    g3 -= pos[2];
  }
  res[0] = mat[0]*g1 + mat[3]*g2 + mat[6]*g3;
  res[1] = mat[1]*g1 + mat[4]*g2 + mat[7]*g3;
  res[2] = mat[2]*g1 + mat[5]*g2 + mat[8]*g3;
}



// compute global coordinates of a local point (l1, l2, l3)
static inline void globalcoord(mjtNum res[3], const mjtNum mat[9], const mjtNum pos[3],
                              mjtNum l1, mjtNum l2, mjtNum l3) {
  // perform mat * (l1, l2, l3) + pos
  res[0] = mat[0]*l1 + mat[1]*l2 + mat[2]*l3;
  res[1] = mat[3]*l1 + mat[4]*l2 + mat[5]*l3;
  res[2] = mat[6]*l1 + mat[7]*l2 + mat[8]*l3;
  if (pos) {
    res[0] += pos[0];
    res[1] += pos[1];
    res[2] += pos[2];
  }
}



// compute possible face normals of a box given up to 3 vertices
static int boxNormals(mjtNum res[9], int resind[3], int dim, mjCCDObj* obj,
                      const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]) {
  // box data
  int g = 3*obj->geom;
  const mjtNum* mat = obj->data->geom_xmat + 3*g;
  const mjtNum* pos = obj->data->geom_xpos + g;

  // rotate global coordinates to geom local frame
  mjtNum v1_local[3], v2_local[3], v3_local[3];
  if (dim > 0) localcoord(v1_local, mat, pos, v1[0], v1[1], v1[2]);
  if (dim > 1) localcoord(v2_local, mat, pos, v2[0], v2[1], v2[2]);
  if (dim > 2) localcoord(v3_local, mat, pos, v3[0], v3[1], v3[2]);

  if (dim == 3) {
    int x = sameSign3(v1_local[0], v2_local[0], v3_local[0]);
    int y = sameSign3(v1_local[1], v2_local[1], v3_local[1]);
    int z = sameSign3(v1_local[2], v2_local[2], v3_local[2]);
    globalcoord(res, mat, NULL, x, y, z);
    int sgn = x + y + z;
    if (x) resind[0] = 0;
    if (y) resind[0] = 2;
    if (z) resind[0] = 4;
    if (sgn == -1) resind[0]++;
    return 1;
  }

  if (dim == 2) {
    int x = sameSign2(v1_local[0], v2_local[0]);
    int y = sameSign2(v1_local[1], v2_local[1]);
    int z = sameSign2(v1_local[2], v2_local[2]);
    if (x) {
      globalcoord(res, mat, NULL, x, 0, 0);
      resind[0] = (x > 0) ? 0 : 1;
    }
    if (y) {
      int i = (x ? 1 : 0);
      globalcoord(res + 3*i, mat, NULL, 0, y, 0);
      resind[i] = (y > 0) ? 2 : 3;
    }
    if (z) {
      globalcoord(res + 3, mat, NULL, 0, 0, z);
      resind[1] = (z > 0) ? 4 : 5;
    }
    return 2;
  }

  if (dim == 1) {
    mjtNum x = (v1_local[0] > 0) ? 1 : -1;
    mjtNum y = (v1_local[1] > 0) ? 1 : -1;
    mjtNum z = (v1_local[2] > 0) ? 1 : -1;
    globalcoord(res + 0, mat, NULL, x, 0, 0);
    globalcoord(res + 3, mat, NULL, 0, y, 0);
    globalcoord(res + 6, mat, NULL, 0, 0, z);
    resind[0] = (x > 0) ? 0 : 1;
    resind[1] = (y > 0) ? 2 : 3;
    resind[2] = (z > 0) ? 4 : 5;
    return 3;
  }
  return 0;
}



// recover face of a box from its index
static int boxFace(mjtNum res[12], mjCCDObj* obj, int idx) {
  // box data
  int g = 3*obj->geom;
  const mjtNum* mat = obj->data->geom_xmat + 3*g;
  const mjtNum* pos = obj->data->geom_xpos + g;
  const mjtNum* size = obj->model->geom_size + g;

  // compute global coordinates of the box face and face normal
  switch (idx) {
    case 0:  // right
      globalcoord(res + 0, mat, pos,  size[0],  size[1],  size[2]);
      globalcoord(res + 3, mat, pos,  size[0],  size[1], -size[2]);
      globalcoord(res + 6, mat, pos,  size[0], -size[1], -size[2]);
      globalcoord(res + 9, mat, pos,  size[0], -size[1],  size[2]);
      return 4;
     case 1:  // left
      globalcoord(res + 0, mat, pos, -size[0],  size[1], -size[2]);
      globalcoord(res + 3, mat, pos, -size[0],  size[1],  size[2]);
      globalcoord(res + 6, mat, pos, -size[0], -size[1],  size[2]);
      globalcoord(res + 9, mat, pos, -size[0], -size[1], -size[2]);
      return 4;
    case 2:  // top
      globalcoord(res + 0, mat, pos, -size[0],  size[1], -size[2]);
      globalcoord(res + 3, mat, pos,  size[0],  size[1], -size[2]);
      globalcoord(res + 6, mat, pos,  size[0],  size[1],  size[2]);
      globalcoord(res + 9, mat, pos, -size[0],  size[1],  size[2]);
      return 4;
    case 3:  // bottom
      globalcoord(res + 0, mat, pos, -size[0], -size[1],  size[2]);
      globalcoord(res + 3, mat, pos,  size[0], -size[1],  size[2]);
      globalcoord(res + 6, mat, pos,  size[0], -size[1], -size[2]);
      globalcoord(res + 9, mat, pos, -size[0], -size[1], -size[2]);
      return 4;
    case 4:  // front
      globalcoord(res + 0, mat, pos, -size[0],  size[1],  size[2]);
      globalcoord(res + 3, mat, pos,  size[0],  size[1],  size[2]);
      globalcoord(res + 6, mat, pos,  size[0], -size[1],  size[2]);
      globalcoord(res + 9, mat, pos, -size[0], -size[1],  size[2]);
      return 4;
    case 5:  // back
      globalcoord(res + 0, mat, pos,  size[0],  size[1], -size[2]);
      globalcoord(res + 3, mat, pos, -size[0],  size[1], -size[2]);
      globalcoord(res + 6, mat, pos, -size[0], -size[1], -size[2]);
      globalcoord(res + 9, mat, pos,  size[0], -size[1], -size[2]);
      return 4;
  }
  return 0;
}



static inline int compareNorms(int res[2], const mjtNum* v, int nv,
                               const mjtNum* w, int nw) {
  for (int i = 0; i < nv; i++) {
    for (int j = 0; j < nw; j++) {
      if (dot3(v + 3*i, w + 3*j) < -0.99999872) {
        res[0] = i;
        res[1] = j;
        return 1;
      }
    }
  }
  return 0;
}



// return number of dimensions of a feature (1, 2 or 3)
static inline int simplexDim(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]) {
  int i = 1;
  int same1 = equalexact3(v1, v2);
  int same2 = equalexact3(v1, v3);
  int same3 = equalexact3(v2, v3);
  if (!same1) i++;
  if (!same3 && !same2) i++;
  return i;
}



// recover multiple contacts from EPA polytope
static void multicontact(Polytope* pt, Face* face, mjCCDStatus* status,
                         mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum face1[mjMAX_SIDES * 3], face2[mjMAX_SIDES * 3];

  // get vertices of faces from EPA
  const mjtNum* v11 = pt->verts[face->verts[0]].vert1;
  const mjtNum* v12 = pt->verts[face->verts[1]].vert1;
  const mjtNum* v13 = pt->verts[face->verts[2]].vert1;
  const mjtNum* v21 = pt->verts[face->verts[0]].vert2;
  const mjtNum* v22 = pt->verts[face->verts[1]].vert2;
  const mjtNum* v23 = pt->verts[face->verts[2]].vert2;

  // get dimensions of features of geoms 1 and 2
  int nface1 = simplexDim(v11, v12, v13);
  int nface2 = simplexDim(v21, v22, v23);
  int nnorms1 = 0, nnorms2 = 0;
  mjtNum n1[9], n2[9];  // normals of possible face collisions
  int idx1[3], idx2[3];  // indices of faces, so they can be recovered later

  // get all possible face normals for each geom
  if (obj1->geom_type == mjGEOM_BOX) {
    nnorms1 = boxNormals(n1, idx1, nface1, obj1, v11, v12, v13);
  }
  if (obj2->geom_type == mjGEOM_BOX) {
    nnorms2 = boxNormals(n2, idx2, nface2, obj2, v21, v22, v23);
  }

  // determine if any two normals match
  int res[2];
  if (!compareNorms(res, n1, nnorms1, n2, nnorms2)) {
    return;
  }
  int i = res[0], j = res[1];

  // recover matching faces
  if (obj1->geom_type == mjGEOM_BOX) {
    nface1 = boxFace(face1, obj1, idx1[i]);
  }
  if (obj2->geom_type == mjGEOM_BOX) {
    nface2 = boxFace(face2, obj2, idx2[j]);
  }

  if (nface1 >= 3 && nface2 >= 3) {
    // TODO(kylebayes): this approximates the contact direction, by scaling the face normal by the
    // single contact direction's magnitude. This is effective, but polygonClip should compute
    // this for each contact point.
    mjtNum diff[3], approx_dir[3];
    sub3(diff, status->x2, status->x1);
    scl3(approx_dir, n2 + 3*j, mju_sqrt(dot3(diff, diff)));

    // clip the faces and store the results in status
    polygonClip(status, face1, nface1, face2, nface2, n1 + 3*i, approx_dir);
  }
}




// inflate a contact by margin
static inline void inflate(mjCCDStatus* status, mjtNum margin1, mjtNum margin2) {
  mjtNum n[3];
  sub3(n, status->x2, status->x1);
  mju_normalize3(n);
  if (margin1) {
    status->x1[0] += margin1 * n[0];
    status->x1[1] += margin1 * n[1];
    status->x1[2] += margin1 * n[2];
  }
  if (margin2) {
    status->x2[0] -= margin2 * n[0];
    status->x2[1] -= margin2 * n[1];
    status->x2[2] -= margin2 * n[2];
  }
  status->dist -= (margin1 + margin2);
}



// general convex collision detection
mjtNum mjc_ccd(const mjCCDConfig* config, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  // setup
  obj1->center(status->x1, obj1);
  obj2->center(status->x2, obj2);
  status->gjk_iterations = 0;
  status->epa_iterations = 0;
  status->epa_status = mjEPA_NOCONTACT;
  status->tolerance = config->tolerance;
  status->max_iterations = config->max_iterations;
  status->max_contacts = config->max_contacts;
  status->dist_cutoff = config->dist_cutoff;

  // special handling for sphere and capsule (shrink to point and line respectively)
  if (obj1->geom_type == mjGEOM_SPHERE || obj2->geom_type == mjGEOM_SPHERE ||
      obj1->geom_type == mjGEOM_CAPSULE || obj2->geom_type == mjGEOM_CAPSULE) {
    void (*support1)(mjtNum*, struct _mjCCDObj*, const mjtNum*) = obj1->support;
    void (*support2)(mjtNum*, struct _mjCCDObj*, const mjtNum*) = obj2->support;
    mjtNum margin1 = 0, margin2 = 0;

    if (obj1->geom_type == mjGEOM_SPHERE) {
      const mjModel* m = obj1->model;
      margin1 = m->geom_size[3*obj1->geom];
      support1 = obj1->support;
      obj1->support = mjc_pointSupport;
    } else if (obj1->geom_type == mjGEOM_CAPSULE) {
      const mjModel* m = obj1->model;
      margin1 = m->geom_size[3*obj1->geom];
      support1 = obj1->support;
      obj1->support = mjc_lineSupport;
    }

    if (obj2->geom_type == mjGEOM_SPHERE) {
      const mjModel* m = obj2->model;
      margin2 = m->geom_size[3*obj2->geom];
      support2 = obj2->support;
      obj2->support = mjc_pointSupport;
    } else if (obj2->geom_type == mjGEOM_CAPSULE) {
      const mjModel* m = obj2->model;
      margin2 = m->geom_size[3*obj2->geom];
      support2 = obj2->support;
      obj2->support = mjc_lineSupport;
    }

    status->dist_cutoff += margin1 + margin2;
    gjk(status, obj1, obj2);
    status->dist_cutoff = config->dist_cutoff;

    // shallow penetration, inflate contact
    if (status->dist > 0) {
      inflate(status, margin1, margin2);
      if (status->dist > status->dist_cutoff) {
        status->dist = mjMAXVAL;
      }
      return status->dist;
    }

    // contact not needed
    if (!config->max_contacts) {
      status->nx = 0;
      status->dist = 0;
      return 0;
    }

    // deep penetration, reset everything and run GJK again
    status->gjk_iterations = 0;
    obj1->support = support1;
    obj2->support = support2;
    obj1->center(status->x1, obj1);
    obj2->center(status->x2, obj2);
  }

  gjk(status, obj1, obj2);

  // penetration recovery for contacts not needed
  if (!config->max_contacts) {
    return status->dist;
  }

  if (status->dist <= config->tolerance && status->nsimplex > 1) {
    status->dist = 0;  // assume touching
    int N = status->max_iterations;
    mjData* d = (mjData*) obj1->data;
    mj_markStack((mjData*) obj1->data);

    Polytope pt;
    pt.nfaces = pt.nmap = pt.nverts = 0;

    // allocate memory for vertices
    pt.verts  = mjSTACKALLOC(d, 3*(5 + N), Vertex);

    // allocate memory for faces
    pt.maxfaces = (6*N > 1000) ? 6*N : 1000;  // use 1000 faces as lower bound
    size_t size1 = sizeof(Face) * pt.maxfaces;
    size_t size2 = sizeof(Face*) * pt.maxfaces;

    // since a generous upper bound is used, we need to rescale stack use if not enough
    // memory is available
    size_t max_size = mj_stackBytesAvailable(d) - 12*(N * sizeof(int));
    if (size1 + size2 > max_size) {
      pt.maxfaces = max_size / (sizeof(Face) + sizeof(Face*));
    }
    pt.faces = mjSTACKALLOC(d, pt.maxfaces, Face);
    pt.map = mjSTACKALLOC(d, pt.maxfaces, Face*);

    int ret;
    if (status->nsimplex == 2) {
      ret = polytope2(&pt, status, obj1, obj2);
    } else if (status->nsimplex == 3) {
      ret = polytope3(&pt, status, obj1, obj2);
    } else {
      ret = polytope4(&pt, status, obj1, obj2);
    }
    status->epa_status = ret;

    // simplex not on boundary (objects are penetrating)
    if (!ret) {
      Face* face = epa(status, &pt, obj1, obj2);
      if (config->max_contacts > 1 && face) {
        multicontact(&pt, face, status, obj1, obj2);
      }
    }
    mj_freeStack(d);
  }
  return status->dist;
}
