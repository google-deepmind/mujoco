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
#include <string.h>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_convex.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"

// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static void subdistance(mjtNum lambda[4], const mjtNum simplex[12], int n);

// compute the barycentric coordinates of the closest point to the origin in the n-simplex,
// where n = 3, 2, 1 respectively
static void S3D(mjtNum lambda[4], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3],
                const mjtNum s4[3]);
static void S2D(mjtNum lambda[3], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3]);
static void S1D(mjtNum lambda[2], const mjtNum s1[3], const mjtNum s2[3]);

// compute the support point for GJK
static void gjkSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                       const mjtNum x_k[3]);

// compute the support point for EPA
static void epaSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                    const mjtNum d[3], mjtNum dnorm);

// compute the linear combination of n 3D vectors
static void lincomb(mjtNum res[3], const mjtNum* coef, const mjtNum* v, int n);

// one face in a polytope
typedef struct {
  int verts[3];  // indices of the three vertices of the face in the polytope
  int adj[3];    // adjacent faces, one for each edge: [v1,v2], [v2,v3], [v3,v1]
  mjtNum v[3];   // projection of the origin on face, can be used as face normal
  mjtNum dist;   // norm of v; negative if deleted
  int index;     // index in map; -1: not in map, -2: deleted from polytope
} Face;

// polytope used in the Expanding Polytope Algorithm (EPA)
typedef struct {
  mjtNum* verts1;    // vertices of polytope in obj1
  mjtNum* verts2;    // vertices of polytope in obj2
  mjtNum* verts;     // v1 - v2; vertices in Minkowski sum making up polytope
  int nverts;        // number of vertices
  Face* faces;       // list of faces that make up the polytope
  int nfaces;        // number of faces
  int maxfaces;      // max number of faces that can be stored in polytope
  Face** map;        // linear map storing faces
  int nmap;          // number of faces in map
} Polytope;

// make copy of vertex in polytope and return its index
static int newVertex(Polytope* pt, const mjtNum v1[3], const mjtNum v2[3]);

// attach a face to the polytope with the given vertex indices; return distance to origin
static mjtNum attachFace(Polytope* pt, int v1, int v2, int v3, int adj1, int adj2, int adj3);

// return 1 if objects are in contact; 0 if not; -1 if inconclusive
// status must have initial tetrahedrons
static int gjkIntersect(mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);

// return the penetration depth of two convex objects; witness points are in status->{x1, x2}
static mjtNum epa(mjCCDStatus* status, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2);

// -------------------------------- inlined  3D vector utils --------------------------------------

// v1 == v2
static inline int equal3(const mjtNum v1[3], const mjtNum v2[3]) {
  return mju_abs(v1[0] - v2[0]) < mjMINVAL &&
         mju_abs(v1[1] - v2[1]) < mjMINVAL &&
         mju_abs(v1[2] - v2[2]) < mjMINVAL;
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
static mjtNum gjk(mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  int get_dist = status->dist_cutoff > 0;  // need to recover geom distances if not in contact
  int backup_gjk = !get_dist;              // use gjkIntersect if no geom distances needed
  mjtNum *simplex1 = status->simplex1;     // simplex for obj1
  mjtNum *simplex2 = status->simplex2;     // simplex for obj2
  mjtNum *simplex  = status->simplex;      // simplex in Minkowski difference
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
    mjtNum *s1_k = simplex1 + 3*n;  // the kth support point in obj1
    mjtNum *s2_k = simplex2 + 3*n;  // the kth support point in obj2
    mjtNum *s_k = simplex + 3*n;    // the kth support point of Minkowski difference

    // compute the kth support point
    gjkSupport(s1_k, s2_k, obj1, obj2, x_k);
    sub3(s_k, s1_k, s2_k);

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
        return status->dist;
      }
    } else if (status->dist_cutoff < mjMAXVAL) {
      mjtNum vs = mju_dot3(x_k, s_k), vv = mju_dot3(x_k, x_k);
      if (mju_dot3(x_k, s_k) > 0 && (vs*vs / vv) >= cutoff2) {
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist = mjMAXVAL;
        return status->dist;
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
        return status->dist;
      }
      k = status->gjk_iterations;
      backup_gjk = 0;
    }

    // run the distance subalgorithm to compute the barycentric coordinates
    // of the closest point to the origin in the simplex
    subdistance(lambda, simplex, n + 1);

    // remove vertices from the simplex no longer needed
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (lambda[i] == 0) continue;
      copy3(simplex1 + 3*n, simplex1 + 3*i);
      copy3(simplex2 + 3*n, simplex2 + 3*i);
      copy3(simplex  + 3*n, simplex  + 3*i);
      lambda[n++] = lambda[i];
    }

    // get the next iteration of x_k
    mjtNum x_next[3];
    lincomb(x_next, lambda, simplex, n);

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
  lincomb(x1_k, lambda, simplex1, n);
  lincomb(x2_k, lambda, simplex2, n);

  status->nx = 1;
  status->gjk_iterations = k;
  status->nsimplex = n;
  status->dist = mju_norm3(x_k);
  return status->dist;
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
static void gjkSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
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
  support(s1, s2, obj1, obj2, dir, dir_neg);
}



// compute the support point in the Minkowski difference for EPA
static void epaSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                       const mjtNum d[3], mjtNum dnorm) {
  mjtNum dir[3] = {1, 0, 0}, dir_neg[3] = {-1, 0, 0};

  // mjc_support assumes a normalized direction
  if (dnorm > mjMINVAL) {
    dir[0] = d[0] / dnorm;
    dir[1] = d[1] / dnorm;
    dir[2] = d[2] / dnorm;
    scl3(dir_neg, dir, -1);
  }

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  support(s1, s2, obj1, obj2, dir, dir_neg);
}



// compute the support point in the Minkowski difference for gjkIntersect (without normalization)
static void gjkIntersectSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                                const mjtNum dir[3]) {
  mjtNum dir_neg[3] = {-dir[0], -dir[1], -dir[2]};
  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  support(s1, s2, obj1, obj2, dir, dir_neg);
}



// compute the signed distance of a face along with the normal
static inline mjtNum signedDistance(mjtNum normal[3], const mjtNum v1[3], const mjtNum v2[3],
                                    const mjtNum v3[3]) {
  mjtNum diff1[3], diff2[3];
  sub3(diff1, v3, v1);
  sub3(diff2, v2, v1);
  cross3(normal, diff1, diff2);
  mjtNum norm = dot3(normal, normal);
  if (norm > mjMINVAL*mjMINVAL && norm < mjMAXVAL*mjMAXVAL) {
    norm = 1/mju_sqrt(norm);
    scl3(normal, normal, norm);
    return dot3(normal, v1);
  }
  return mjMAXVAL;  // cannot recover normal (ignore face)
}



// return 1 if objects are in contact; 0 if not; -1 if inconclusive
static int gjkIntersect(mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum simplex1[12], simplex2[12], simplex[12];
  memcpy(simplex1, status->simplex1, sizeof(mjtNum) * 12);
  memcpy(simplex2, status->simplex2, sizeof(mjtNum) * 12);
  memcpy(simplex, status->simplex, sizeof(mjtNum) * 12);
  int s[4] = {0, 3, 6, 9};

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
      for (int n = 0; n < 4; n++) {
        copy3(status->simplex + 3*n, simplex + s[n]);
        copy3(status->simplex1 + 3*n, simplex1 + s[n]);
        copy3(status->simplex2 + 3*n, simplex2 + s[n]);
      }
      status->gjk_iterations = k;
      return 1;
    }

    // replace worst vertex (farthest from origin) with new candidate
    gjkIntersectSupport(simplex1 + s[index], simplex2 + s[index], obj1, obj2, normals + 3*index);
    sub3(simplex + s[index], simplex1 + s[index], simplex2 + s[index]);

    // found origin outside the Minkowski difference (return no collision)
    if (dot3(&normals[3*index], simplex + s[index]) < 0) {
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
static inline void lincomb(mjtNum res[3], const mjtNum* coef, const mjtNum* v, int n) {
  res[0] = res[1] = res[2] = 0;
  for (int i = 0; i < n; i++) {
    res[0] += coef[i] * v[3*i + 0];
    res[1] += coef[i] * v[3*i + 1];
    res[2] += coef[i] * v[3*i + 2];
  }
}



// linear combination of 2 3D vectors
static inline void lincomb2(mjtNum res[3], const mjtNum coef[2], const mjtNum v1[3],
                     const mjtNum v2[3]) {
  res[0] = coef[0]*v1[0] + coef[1]*v2[0];
  res[1] = coef[0]*v1[1] + coef[1]*v2[1];
  res[2] = coef[0]*v1[2] + coef[1]*v2[2];
}



// linear combination of 3 3D vectors
static inline void lincomb3(mjtNum res[3], const mjtNum coef[3], const mjtNum v1[3],
                     const mjtNum v2[3], const mjtNum v3[3]) {
  res[0] = coef[0]*v1[0] + coef[1]*v2[0] + coef[2]*v3[0];
  res[1] = coef[0]*v1[1] + coef[1]*v2[1] + coef[2]*v3[1];
  res[2] = coef[0]*v1[2] + coef[1]*v2[2] + coef[2]*v3[2];
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



// return true only when a and b are both strictly positive or both strictly negative
static inline int sameSign(mjtNum a, mjtNum b) {
  if (a > 0 && b > 0) return 1;
  if (a < 0 && b < 0) return 1;
  return 0;
}



// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static inline void subdistance(mjtNum lambda[4], const mjtNum simplex[12], int n) {
  lambda[0] = lambda[1] = lambda[2] = lambda[3] = 0;
  const mjtNum* s1 = simplex;
  const mjtNum* s2 = simplex + 3;
  const mjtNum* s3 = simplex + 6;
  const mjtNum* s4 = simplex + 9;

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

  int comp1 = sameSign(m_det, C41),
      comp2 = sameSign(m_det, C42),
      comp3 = sameSign(m_det, C43),
      comp4 = sameSign(m_det, C44);

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
    lincomb3(x, lambda_2d, s2, s3, s4);
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
    lincomb3(x, lambda_2d, s1, s3, s4);
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
    lincomb3(x, lambda_2d, s1, s2, s4);
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
    lincomb3(x, lambda_2d, s1, s2, s3);
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

  int comp1 = sameSign(M_max, C31),
      comp2 = sameSign(M_max, C32),
      comp3 = sameSign(M_max, C33);

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
    lincomb2(x, lambda_1d, s2, s3);
    mjtNum d = dot3(x, x);
    lambda[0] = 0;
    lambda[1] = lambda_1d[0];
    lambda[2] = lambda_1d[1];
    dmin = d;
  }

  if (!comp2) {
    mjtNum lambda_1d[2], x[3];
    S1D(lambda_1d, s1, s3);
    lincomb2(x, lambda_1d, s1, s3);
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
    lincomb2(x, lambda_1d, s1, s2);
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
  if (sameSign(mu_max, C1) && sameSign(mu_max, C2)) {
    lambda[0] = C1 / mu_max;
    lambda[1] = C2 / mu_max;
  } else {
    lambda[0] = 0;
    lambda[1] = 1;
  }
}

// ---------------------------------------- EPA ---------------------------------------------------

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
static int polytope2(Polytope* pt, const mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum v1[3], v2[3];
  sub3(v1, status->simplex1 + 0, status->simplex2 + 0);
  sub3(v2, status->simplex1 + 3, status->simplex2 + 3);

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


  mjtNum v3a[3], v3b[3], v3[3];
  epaSupport(v3a, v3b, obj1, obj2, d1, mju_norm3(d1));
  sub3(v3, v3a, v3b);

  mjtNum v4a[3], v4b[3], v4[3];
  epaSupport(v4a, v4b, obj1, obj2, d2, mju_norm3(d2));
  sub3(v4, v4a, v4b);

  mjtNum v5a[3], v5b[3], v5[3];
  epaSupport(v5a, v5b, obj1, obj2, d3, mju_norm3(d3));
  sub3(v5, v5a, v5b);

  // check that all six faces are valid triangles (not collinear)
  if (mju_abs(det3(v1, v3, v4)) < mjMINVAL || mju_abs(det3(v1, v3, v5)) < mjMINVAL ||
      mju_abs(det3(v1, v3, v5)) < mjMINVAL || mju_abs(det3(v2, v3, v4)) < mjMINVAL ||
      mju_abs(det3(v2, v3, v5)) < mjMINVAL || mju_abs(det3(v2, v4, v5)) < mjMINVAL) {
    return 2;
  }

  // save vertices and get indices for each one
  int v1i = newVertex(pt, status->simplex1 + 0, status->simplex2 + 0);
  int v2i = newVertex(pt, status->simplex1 + 3, status->simplex2 + 3);
  int v3i = newVertex(pt, v3a, v3b);
  int v4i = newVertex(pt, v4a, v4b);
  int v5i = newVertex(pt, v5a, v5b);

  // build hexahedron
  attachFace(pt, v1i, v3i, v4i, 1, 3, 2);
  attachFace(pt, v1i, v5i, v3i, 2, 4, 0);
  attachFace(pt, v1i, v4i, v5i, 0, 5, 1);
  attachFace(pt, v2i, v4i, v3i, 5, 0, 4);
  attachFace(pt, v2i, v3i, v5i, 3, 1, 5);
  attachFace(pt, v2i, v5i, v4i, 4, 2, 3);

  // if the origin is on the affine hull of any of the faces then the origin is not in the
  //  hexahedron or the hexahedron is degenerate
  for (int i = 0; i < 6; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
    if (pt->faces[i].dist < mjMINVAL) {
      return 3;
    }
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
static int polytope3(Polytope* pt, const mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  // get vertices of simplex from GJK
  const mjtNum *v1 = status->simplex,
               *v2 = status->simplex + 3,
               *v3 = status->simplex + 6;

  // get normals in both directions
  mjtNum diff1[3], diff2[3], n[3], n_neg[3];
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(n, diff1, diff2);
  mjtNum n_norm = mju_norm3(n);
  if (n_norm < mjMINVAL) {
    return 4;
  }

  // negative of triangle normal n
  scl3(n_neg, n, -1);

  // get 4th vertex in n direction
  mjtNum v4a[3], v4b[3], v4[3];
  epaSupport(v4a, v4b, obj1, obj2, n, n_norm);
  sub3(v4, v4a, v4b);

  // check that v4 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v4)) {
    return 5;
  }

  // get 5th vertex in -n direction
  mjtNum v5a[3], v5b[3], v5[3];
  epaSupport(v5a, v5b, obj1, obj2, n_neg, n_norm);
  sub3(v5, v5a, v5b);

  // check that v5 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v5)) {
    return 6;
  }

  // if origin does not lie on simplex then we need to check that the hexahedron contains the
  // origin
  //
  // TODO(kylebayes): It's possible for GJK to return a 2-simplex with the origin not contained in
  // it but within tolerance from it. In that case the hexahedron could possibly be constructed
  // that doesn't contain the origin, but nonetheless there is penetration depth.
  if (status->dist > 10*mjMINVAL && !testTetra(v1, v2, v3, v4) && !testTetra(v1, v2, v3, v5)) {
    return 7;
  }

  // save vertices and get indices for each one
  int v1i = newVertex(pt, status->simplex1 + 0, status->simplex2 + 0);
  int v2i = newVertex(pt, status->simplex1 + 3, status->simplex2 + 3);
  int v3i = newVertex(pt, status->simplex1 + 6, status->simplex2 + 6);
  int v5i = newVertex(pt, v5a, v5b);
  int v4i = newVertex(pt, v4a, v4b);

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
      return 8;
    }
  }
  pt->nmap = 6;
  return 0;
}



// replace a 3-simplex with one of its faces
static inline void replaceSimplex3(Polytope* pt, mjCCDStatus* status, int v1, int v2, int v3) {
  status->nsimplex = 3;
  copy3(status->simplex1 + 0, pt->verts1 + v1);
  copy3(status->simplex1 + 3, pt->verts1 + v2);
  copy3(status->simplex1 + 6, pt->verts1 + v3);

  copy3(status->simplex2 + 0, pt->verts2 + v1);
  copy3(status->simplex2 + 3, pt->verts2 + v2);
  copy3(status->simplex2 + 6, pt->verts2 + v3);

  copy3(status->simplex + 0, pt->verts + v1);
  copy3(status->simplex + 3, pt->verts + v2);
  copy3(status->simplex + 6, pt->verts + v3);

  pt->nfaces = 0;
  pt->nverts = 0;
}



// create a polytope from a 3-simplex (returns 0 on success)
static int polytope4(Polytope* pt, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  int v1 = newVertex(pt, status->simplex1 + 0, status->simplex2 + 0);
  int v2 = newVertex(pt, status->simplex1 + 3, status->simplex2 + 3);
  int v3 = newVertex(pt, status->simplex1 + 6, status->simplex2 + 6);
  int v4 = newVertex(pt, status->simplex1 + 9, status->simplex2 + 9);

  // if the origin is on a face, replace the 3-simplex with a 2-simplex
  if (attachFace(pt, v1, v2, v3, 1, 3, 2) == 0.0) {
    replaceSimplex3(pt, status, v1, v2, v3);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v4, v2, 2, 3, 0) == 0.0) {
    replaceSimplex3(pt, status, v1, v4, v2);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v3, v4, 0, 3, 1) == 0.0) {
    replaceSimplex3(pt, status, v1, v3, v4);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v4, v3, v2, 2, 0, 1) == 0.0) {
    replaceSimplex3(pt, status, v4, v3, v2);
    return polytope3(pt, status, obj1, obj2);
  }

  for (int i = 0; i < 4; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
  }
  pt->nmap = 4;
  return 0;
}



// make a copy of vertex in polytope and return its index
static int newVertex(Polytope* pt, const mjtNum v1[3], const mjtNum v2[3]) {
  int n = 3*pt->nverts++;
  copy3(pt->verts1 + n, v1);
  copy3(pt->verts2 + n, v2);
  sub3(pt->verts + n, v1, v2);
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
  int ret = projectOriginPlane(face->v, pt->verts + v3, pt->verts + v2, pt->verts + v1);
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
  mjtNum* v1 = pt->verts + face->verts[0];
  mjtNum* v2 = pt->verts + face->verts[1];
  mjtNum* v3 = pt->verts + face->verts[2];
  triAffineCoord(lambda, v1, v2, v3, face->v);

  // face on geom 1
  v1 = pt->verts1 + face->verts[0];
  v2 = pt->verts1 + face->verts[1];
  v3 = pt->verts1 + face->verts[2];
  x1[0] = v1[0]*lambda[0] + v2[0]*lambda[1] + v3[0]*lambda[2];
  x1[1] = v1[1]*lambda[0] + v2[1]*lambda[1] + v3[1]*lambda[2];
  x1[2] = v1[2]*lambda[0] + v2[2]*lambda[1] + v3[2]*lambda[2];

  // face on geom 2
  v1 = pt->verts2 + face->verts[0];
  v2 = pt->verts2 + face->verts[1];
  v3 = pt->verts2 + face->verts[2];
  x2[0] = v1[0]*lambda[0] + v2[0]*lambda[1] + v3[0]*lambda[2];
  x2[1] = v1[1]*lambda[0] + v2[1]*lambda[1] + v3[1]*lambda[2];
  x2[2] = v1[2]*lambda[0] + v2[2]*lambda[1] + v3[2]*lambda[2];
}



// return the penetration depth of two convex objects; witness points are in status->{x1, x2}
static mjtNum epa(mjCCDStatus* status, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2) {
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
    mjtNum w1[3], w2[3], w[3];
    epaSupport(w1, w2, obj1, obj2, face->v, lower);
    sub3(w, w1, w2);
    mjtNum upper_k = dot3(face->v, w) / lower;  // upper bound for kth iteration
    if (upper_k < upper) upper = upper_k;
    if (upper - lower < tolerance) {
      break;
    }

    h.w = w;
    horizon(&h, face);

    // insert w as new vertex and attach faces along the horizon
    int wi = newVertex(pt, w1, w2), nfaces = pt->nfaces, nedges = h.nedges;

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
      return 0;
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
        return 0;
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
  return face->dist;
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
  // set up
  mjtNum dist;
  obj1->center(status->x1, obj1);
  obj2->center(status->x2, obj2);
  status->gjk_iterations = 0;
  status->epa_iterations = -1;
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
    dist = gjk(status, obj1, obj2);
    status->dist_cutoff = config->dist_cutoff;

    // shallow penetration, inflate contact
    if (dist > 0) {
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

  dist = gjk(status, obj1, obj2);

  // penetration recovery for contacts not needed
  if (!config->max_contacts) {
    return dist;
  }

  if (dist <= config->tolerance && status->nsimplex > 1) {
    int N = status->max_iterations;
    mjData* d = (mjData*) obj1->data;
    mj_markStack((mjData*) obj1->data);

    Polytope pt;
    pt.nfaces = pt.nmap = pt.nverts = 0;

    // allocate memory for vertices
    pt.verts  = mjSTACKALLOC(d, 3*(5 + N), mjtNum);
    pt.verts1 = mjSTACKALLOC(d, 3*(5 + N), mjtNum);
    pt.verts2 = mjSTACKALLOC(d, 3*(5 + N), mjtNum);

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

    // simplex not on boundary (objects are penetrating)
    if (!ret) {
      dist = -epa(status, &pt, obj1, obj2);
    } else {
      status->epa_iterations = -ret;
      dist = 0;
    }
    mj_freeStack(d);
  }
  return dist;
}
