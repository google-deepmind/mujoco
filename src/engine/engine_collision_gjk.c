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

#include <stddef.h>
#include <stdlib.h>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_convex.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"

// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static void subdistance(mjtNum lambda[4], const mjtNum simplex[12], int n);

// these internal functions compute the barycentric coordinates of the closest point
// to the origin in the n-simplex, where n = 3, 2, 1 respectively
static void S3D(mjtNum lambda[4], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3],
                const mjtNum s4[3]);
static void S2D(mjtNum lambda[3], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3]);
static void S1D(mjtNum lambda[2], const mjtNum s1[3], const mjtNum s2[3]);

// helper function to compute the support point in the Minkowski difference
static void support(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2, const mjtNum d[3]);

// support function tweaked for GJK by taking kth iteration point as input and setting both
// support points to recover witness points
static void gjkSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                       const mjtNum x_k[3]);

// linear algebra utility functions
static mjtNum det3(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]);
static void lincomb(mjtNum res[3], const mjtNum* coef, const mjtNum* v, int n);

// one face in a polytope
typedef struct {
  int verts[3];  // indices of the three vertices of the face in the polytope
  int adj[3];    // adjacent faces (one for each edge: [v1,v2], [v2,v3], [v3,v1])
  mjtNum v[3];   // the projection of the origin on the face (can be used as face normal)
  mjtNum dist;   // norm of v; negative if deleted
  int index;     // index in heap
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
  Face** heap;       // min heap storing faces
  int nheap;         // number of faces in heap
} Polytope;

// generates a polytope from a 1, 2, or 3-simplex respectively; return 1 if successful, 0 otherwise
static int polytope2(Polytope* pt, const mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);
static int polytope3(Polytope* pt, const mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);
static int polytope4(Polytope* pt, const mjCCDStatus* status);

// copies a vertex into the polytope and returns its index
static int newVertex(Polytope* pt, const mjtNum v1[3], const mjtNum v2[3]);

// attaches a face to the polytope with the given vertex indices; returns non-zero on error
static int attachFace(Polytope* pt, int v1, int v2, int v3, int adj1, int adj2, int adj3);

// returns the penetration depth of two convex objects; witness points are in status->{x1, x2}
static mjtNum epa(mjCCDStatus* status, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2);



// returns true if both geoms are discrete shapes (i.e. meshes or boxes with no margin)
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
  int get_dist = status->has_distances;  // need to recover geom distances if not in contact
  mjtNum *simplex1 = status->simplex1;   // simplex for obj1
  mjtNum *simplex2 = status->simplex2;   // simplex for obj2
  mjtNum simplex[12];                    // simplex in Minkowski difference
  int n = 0;                             // number of vertices in the simplex
  int k = 0;                             // current iteration
  int kmax = status->max_iterations;     // max number of iterations
  mjtNum* x1_k = status->x1;             // the kth approximation point for obj1
  mjtNum* x2_k = status->x2;             // the kth approximation point for obj2
  mjtNum x_k[3];                         // the kth approximation point in Minkowski difference
  mjtNum lambda[4];                      // barycentric coordinates for x_k

  // if both geoms are discrete, finite convergence is guaranteed; set tolerance to 0
  mjtNum epsilon = discreteGeoms(obj1, obj2) ? 0 : status->tolerance * status->tolerance;

  // set initial guess
  mju_sub3(x_k, x1_k, x2_k);

  for (; k < kmax; k++) {
    mjtNum *s1_k = simplex1 + 3*n;  // the kth support point in obj1
    mjtNum *s2_k = simplex2 + 3*n;  // the kth support point in obj2
    mjtNum *s_k = simplex + 3*n;    // the kth support point of Minkowski difference

    // compute the kth support point
    gjkSupport(s1_k, s2_k, obj1, obj2, x_k);
    mju_sub3(s_k, s1_k, s2_k);

    // stopping criteria using the Frank-Wolfe duality gap given by
    //  |f(x_k) - f(x_min)|^2 <= < grad f(x_k), (x_k - s_k) >
    mjtNum diff[3];
    mju_sub3(diff, x_k, s_k);
    if (2*mju_dot3(x_k, diff) < epsilon) {
      break;
    }

    // if the hyperplane separates the Minkowski difference and origin, the objects don't collide
    // if geom distance isn't requested, return early
    if (!get_dist && mju_dot3(x_k, s_k) > 0) {
      return mjMAXVAL;
    }

    // run the distance subalgorithm to compute the barycentric coordinates
    // of the closest point to the origin in the simplex
    subdistance(lambda, simplex, n + 1);

    // remove vertices from the simplex no longer needed
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (lambda[i] == 0) continue;
      mju_copy3(simplex1 + 3*n, simplex1 + 3*i);
      mju_copy3(simplex2 + 3*n, simplex2 + 3*i);
      mju_copy3(simplex  + 3*n, simplex  + 3*i);
      lambda[n++] = lambda[i];
    }

    // get the next iteration of x_k
    mjtNum x_next[3];
    lincomb(x_next, lambda, simplex, n);

    // x_k has converged to minimum
    if (mju_equal3(x_next, x_k)) {
      break;
    }

    // copy next iteration into x_k
    mju_copy3(x_k, x_next);

    // we have a tetrahedron containing the origin so return early
    if (n == 4) {
      break;
    }
  }

  // compute the approximate witness points
  lincomb(x1_k, lambda, simplex1, n);
  lincomb(x2_k, lambda, simplex2, n);

  status->gjk_iterations = k;
  status->nsimplex = n;
  return mju_norm3(x_k);
}



// computes the support points in obj1 and obj2 for the kth approximation point
static void gjkSupport(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                       const mjtNum x_k[3]) {
  mjtNum dir[3], dir_neg[3];
  mju_copy3(dir_neg, x_k);
  mju_normalize3(dir_neg);  // mjc_support assumes a normalized direction
  mju_scl3(dir, dir_neg, -1);

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  obj1->support(s1, obj1, dir);
  obj2->support(s2, obj2, dir_neg);
}



// helper function to compute the support point in the Minkowski difference
static void support(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                    const mjtNum d[3]) {
  mjtNum dir[3], dir_neg[3];
  mju_copy3(dir, d);
  mju_normalize3(dir);  // mjc_support assumes a normalized direction
  mju_scl3(dir_neg, dir, -1);

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  obj1->support(s1, obj1, dir);
  obj2->support(s2, obj2, dir_neg);
}



// linear combination of n 3D vectors
static inline void lincomb(mjtNum res[3], const mjtNum* coef, const mjtNum* v, int n) {
  mju_zero3(res);
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



// returns determinant of the 3x3 matrix with columns v1, v2, v3
static inline mjtNum det3(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]) {
  mjtNum temp[3];
  mju_cross(temp, v2, v3);
  return mju_dot3(v1, temp);
}


// res = origin projected onto plane defined by v1, v2, v3
static inline void projectOriginPlane(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3],
                                      const mjtNum v3[3]) {
  mjtNum diff21[3], diff31[3], diff32[3], n[3], nv, nn;
  mju_sub3(diff21, v2, v1);
  mju_sub3(diff31, v3, v1);
  mju_sub3(diff32, v3, v2);

  // n = (v1 - v2) x (v3 - v2)
  mju_cross(n, diff32, diff21);
  nv = mju_dot3(n, v2);
  nn = mju_dot3(n, n);
  if (nv != 0 && nn > mjMINVAL) {
    mju_scl3(res, n, nv / nn);
    return;
  }

  // n = (v2 - v1) x (v3 - v1)
  mju_cross(n, diff21, diff31);
  nv = mju_dot3(n, v1);
  nn = mju_dot3(n, n);
  if (nv != 0 && nn > mjMINVAL) {
    mju_scl3(res, n, nv / nn);
    return;
  }

  // n = (v1 - v3) x (v2 - v3)
  mju_cross(n, diff31, diff32);
  nv = mju_dot3(n, v3);
  nn = mju_dot3(n, n);
  mju_scl3(res, n, nv / nn);
}



// res = origin projected onto line defined by v1, v2
static inline void projectOriginLine(mjtNum res[3], const mjtNum v1[3], const mjtNum v2[3]) {
  // res = v2 - <v2, v2 - v1> / <v2 - v1, v2 - v1> * (v2 - v1)
  mjtNum diff[3];
  mju_sub3(diff, v2, v1);
  mjtNum temp1 = mju_dot3(v2, diff);
  mjtNum temp2 = mju_dot3(diff, diff);
  mju_addScl3(res, v2, diff, - temp1 / temp2);
}



// returns true only when a and b are both strictly positive or both strictly negative
static inline int sameSign(mjtNum a, mjtNum b) {
  if (a > 0 && b > 0) return 1;
  if (a < 0 && b < 0) return 1;
  return 0;
}



// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static inline void subdistance(mjtNum lambda[4], const mjtNum simplex[12], int n) {
  mju_zero4(lambda);
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
    mjtNum d = mju_dot3(x, x);
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
    mjtNum d = mju_dot3(x, x);
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
    mjtNum d = mju_dot3(x, x);
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
    mjtNum d = mju_dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = lambda_2d[2];
      lambda[3] = 0;
      dmin = d;
    }
  }
}



static void S2D(mjtNum lambda[3], const mjtNum s1[3], const mjtNum s2[3], const mjtNum s3[3]) {
  // project origin onto affine hull of the simplex
  mjtNum p_o[3];
  projectOriginPlane(p_o, s1, s2, s3);

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
    mjtNum d = mju_dot3(x, x);
    lambda[0] = 0;
    lambda[1] = lambda_1d[0];
    lambda[2] = lambda_1d[1];
    dmin = d;
  }

  if (!comp2) {
    mjtNum lambda_1d[2], x[3];
    S1D(lambda_1d, s1, s3);
    lincomb2(x, lambda_1d, s1, s3);
    mjtNum d = mju_dot3(x, x);
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
    mjtNum d = mju_dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_1d[0];
      lambda[1] = lambda_1d[1];
      lambda[2] = 0;
      dmin = d;
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

// returns 1 if the origin and p3 are on the same side of the plane defined by p0, p1, p2
static int sameSide(const mjtNum p0[3], const mjtNum p1[3],
                    const mjtNum p2[3], const mjtNum p3[3]) {
    mjtNum diff1[3], diff2[3], diff3[3], diff4[3], n[3];
    mju_sub3(diff1, p1, p0);
    mju_sub3(diff2, p2, p0);
    mju_cross(n, diff1, diff2);

    mju_sub3(diff3, p3, p0);
    mjtNum dot1 = mju_dot3(n, diff3);

    mju_scl3(diff4, p0, -1);
    mjtNum dot2 = mju_dot3(n, diff4);
    if (dot1 > 0 && dot2 > 0) return 1;
    if (dot1 < 0 && dot2 < 0) return 1;
    return 0;
}



// returns 1 if the origin is contained in the tetrahedron, 0 otherwise
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



// creates a polytope from a 1-simplex (2 points i.e. line segment)
static int polytope2(Polytope* pt, const mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum v1[3], v2[3];
  mju_sub3(v1, status->simplex1 + 0, status->simplex2 + 0);
  mju_sub3(v2, status->simplex1 + 3, status->simplex2 + 3);

  mjtNum diff[3];
  mju_sub3(diff, v2, v1);

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
  mju_cross(d1, e, diff);

  // rotate around the line segment to get three more points spaced 120 degrees apart
  mjtNum R[9];
  rotmat(R, diff);

  mju_mulMatVec3(d2, R, d1);
  mju_mulMatVec3(d3, R, d2);


  mjtNum v3a[3], v3b[3], v3[3];
  support(v3a, v3b, obj1, obj2, d1);
  mju_sub3(v3, v3a, v3b);

  mjtNum v4a[3], v4b[3], v4[3];
  support(v4a, v4b, obj1, obj2, d2);
  mju_sub3(v4, v4a, v4b);

  mjtNum v5a[3], v5b[3], v5[3];
  support(v5a, v5b, obj1, obj2, d3);
  mju_sub3(v5, v5a, v5b);

  // check that all six faces are valid triangles (not collinear)
  if (mju_abs(det3(v1, v3, v4)) < mjMINVAL || mju_abs(det3(v1, v3, v5)) < mjMINVAL ||
      mju_abs(det3(v1, v3, v5)) < mjMINVAL || mju_abs(det3(v2, v3, v4)) < mjMINVAL ||
      mju_abs(det3(v2, v3, v5)) < mjMINVAL || mju_abs(det3(v2, v4, v5)) < mjMINVAL) {
    return 0;
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
    if (pt->faces[i].dist < mjMINVAL) {
      return 0;
    }
  }

  // valid hexahedron for EPA
  return 1;
}



// computes the affine coordinates of p on the triangle v1v2v3
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



// returns true if point p and triangle v1v2v3 intersect
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
  mju_sub3(diff, pr, p);
  return mju_norm3(diff) < mjMINVAL;
}



// creates a polytope from a 2-simplex (3 points i.e. triangle)
static int polytope3(Polytope* pt, const mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  // get vertices of simplex from GJK
  mjtNum v1[3], v2[3], v3[3];
  mju_sub3(v1, status->simplex1 + 0, status->simplex2 + 0);
  mju_sub3(v2, status->simplex1 + 3, status->simplex2 + 3);
  mju_sub3(v3, status->simplex1 + 6, status->simplex2 + 6);

  // get normals in both directions
  mjtNum diff1[3], diff2[3], n[3], nn[3];
  mju_sub3(diff1, v2, v1);
  mju_sub3(diff2, v3, v1);
  mju_cross(n, diff1, diff2);
  if (mju_norm3(n) < mjMINVAL) {
    return 0;
  }

  // negative of triangle normal n
  mju_scl3(nn, n, -1);

  // get 4th vertex in n direction
  mjtNum v4a[3], v4b[3], v4[3];
  support(v4a, v4b, obj1, obj2, n);
  mju_sub3(v4, v4a, v4b);

  // check that v4 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v4)) {
    return 0;
  }

  // get 5th vertex in -n direction
  mjtNum v5a[3], v5b[3], v5[3];
  support(v5a, v5b, obj1, obj2, nn);
  mju_sub3(v5, v5a, v5b);

  // check that v5 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v5)) {
    return 0;
  }

  // if origin does not lie on simplex then we need to check that the hexahedron contains the
  // origin
  //
  // TODO(kylebayes): It's possible for GJK to return a 2-simplex with the origin not contained in
  // it but within tolerance from it. In that case the hexahedron could possibly be constructed
  // that doesn't contain the origin, but nonetheless there is penetration depth.
  mjtNum dir[3];
  mju_sub3(dir, status->x1, status->x2);
  if (mju_norm3(dir) > mjMINVAL && !testTetra(v1, v2, v3, v4) && !testTetra(v1, v2, v3, v5)) {
    return 0;
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
    if (pt->faces[i].dist < mjMINVAL) {
      return 0;
    }
  }
  return 1;
}



// creates a polytope from a 3-simplex (4 points i.e. tetrahedron)
static int polytope4(Polytope* pt, const mjCCDStatus* status) {
  int v1 = newVertex(pt, status->simplex1 + 0, status->simplex2 + 0);
  int v2 = newVertex(pt, status->simplex1 + 3, status->simplex2 + 3);
  int v3 = newVertex(pt, status->simplex1 + 6, status->simplex2 + 6);
  int v4 = newVertex(pt, status->simplex1 + 9, status->simplex2 + 9);

  attachFace(pt, v1, v2, v3, 1, 3, 2);
  attachFace(pt, v1, v4, v2, 2, 3, 0);
  attachFace(pt, v1, v3, v4, 0, 3, 1);
  attachFace(pt, v4, v3, v2, 2, 0, 1);
  return 1;
}



// copies a vertex into the polytope and returns its index
static int newVertex(Polytope* pt, const mjtNum v1[3], const mjtNum v2[3]) {
  int n = 3*pt->nverts++;
  mju_copy3(pt->verts1 + n, v1);
  mju_copy3(pt->verts2 + n, v2);
  mju_sub3(pt->verts + n, v1, v2);
  return n;
}



// swap two nodes in heap
static inline void swap(Polytope* pt, int i, int j) {
  Face* tmp = pt->heap[i];
  pt->heap[i] = pt->heap[j];
  pt->heap[j] = tmp;
  pt->heap[i]->index = i;
  pt->heap[j]->index = j;
}



// min heapify heap
void heapify(Polytope* pt, int i) {
  int l = 2*i + 1;    // left child
  int r = 2*(i + 1);  // right child
  int min = i, n = pt->nheap;
  if (l < n && pt->heap[l]->dist < pt->heap[i]->dist)
    min = l;
  if (r < n && pt->heap[r]->dist < pt->heap[min]->dist)
    min = r;
  if (min != i) {
    swap(pt, i, min);
    heapify(pt, min);
  }
}



// delete face from heap
void deleteFace(Polytope* pt, Face* face) {
  // SHOULD NOT OCCUR
  if (!pt->nheap) {
    mju_warning("EPA: trying to delete face from empty polytope");
    return;
  }

  face->dist = -1;
  pt->nheap--;

  // SHOULD NOT OCCUR
  // last face; nothing to do
  if (!pt->nheap) {
    return;  // EPA will flag a warning
  }

  // bubble up face to top of heap
  int i = face->index;
  while (i != 0) {
    int parent = (i - 1) >> 1;
    swap(pt, i, parent);
    i = parent;
  }

  // swap in last element and heapify
  pt->heap[0] = pt->heap[pt->nheap];
  pt->heap[0]->index = 0;
  heapify(pt, 0);
}



// attaches a face to the polytope with the given vertex indices; returns non-zero on error
static int attachFace(Polytope* pt, int v1, int v2, int v3, int adj1, int adj2, int adj3) {
  if (pt->nfaces >= pt->maxfaces) {
    mju_warning("EPA: out of memory for faces on expanding polytope");
    return 1;
  }
  Face* face = &pt->faces[pt->nfaces++];
  face->verts[0] = v1;
  face->verts[1] = v2;
  face->verts[2] = v3;

  // adjacent faces
  face->adj[0] = adj1;
  face->adj[1] = adj2;
  face->adj[2] = adj3;

  // compute witness point v
  projectOriginPlane(face->v, pt->verts + v1, pt->verts + v2, pt->verts + v3);
  face->dist = mju_norm3(face->v);

  // SHOULD NOT OCCUR
  if (pt->nheap == pt->maxfaces) {
    mju_warning("EPA: ran out of memory for faces on expanding polytope");
    return 1;
  }

  // store face on heap
  int i = pt->nheap++;
  face->index = i;
  pt->heap[i] = face;
  while (i != 0) {
    int parent = (i - 1) >> 1;
    if (pt->heap[parent]->dist <= pt->heap[i]->dist) {
      break;
    }
    swap(pt, i, parent);
    i = parent;
  }
  return 0;
}



// horizon: polytope boundary edges that can be seen from w
typedef struct {
  Polytope* pt;  // polytope for which the horizon is defined
  int* indices;  // indices of faces on horizon
  int* edges;    // corresponding edge of each face on the horizon
  int nedges;    // number of edges in horizon
  mjtNum* w;     // point where horizon is created
} Horizon;



// adds an edge to the horizon
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
    if (mju_dot3(face->v, h->w) >= dist2) {
      deleteFace(h->pt, face);

      // recursively search the adjacent faces on the next two edges
      for (int k = 1; k < 3; k++) {
        int i = (e + k) % 3;
        Face* adjFace = &h->pt->faces[face->adj[i]];
        if (adjFace->dist > 0) {
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



// creates horizon given the face as starting point
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
  if (adjFace->dist > 0 && !horizonRec(h, adjFace, adjEdge)) {
    addEdge(h, face->adj[1], adjEdge);
  }

  // third edge
  adjFace = &h->pt->faces[face->adj[2]];
  adjEdge = getEdge(adjFace, face->verts[0]);
  if (adjFace->dist > 0 && !horizonRec(h, adjFace, adjEdge)) {
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



// returns the penetration depth of two convex objects; witness points are in status->{x1, x2}
static mjtNum epa(mjCCDStatus* status, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum dist, tolerance = status->tolerance;
  int k, kmax = status->max_iterations;
  mjData* d = (mjData*) obj1->data;
  Face* face;  // face closest to origin

  // initialize horizon
  Horizon h;
  mj_markStack(d);
  h.indices = mj_stackAllocInt(d, 6 + status->max_iterations);
  h.edges = mj_stackAllocInt(d, 6 + status->max_iterations);
  h.nedges = 0;
  h.pt = pt;

  for (k = 0; k < kmax; k++) {
    // find the face closest to the origin
    if (!pt->nheap) {
      mju_warning("EPA: empty polytope");
      mj_freeStack(d);
      return 0;  // assume 0 depth
    }

    face = pt->heap[0];
    dist = face->dist;

    // check if dist is 0
    if (dist <= 0) {
      mju_warning("EPA: origin lies on affine hull of face");
    }

    // compute support point w from the closest face's normal
    mjtNum w1[3], w2[3], w[3];
    support(w1, w2, obj1, obj2, face->v);
    mju_sub3(w, w1, w2);
    mjtNum next_dist = mju_dot3(face->v, w) / dist;
    if (next_dist - dist < tolerance) {
      break;
    }

    h.w = w;
    horizon(&h, face);

    // insert w as new vertex and attach faces along the horizon
    int wi = newVertex(pt, w1, w2), nfaces = pt->nfaces, nedges = h.nedges;

    // attach first face
    int horIndex = h.indices[0], horEdge = h.edges[0];
    Face* horFace = &pt->faces[horIndex];
    int v1 = horFace->verts[horEdge],
        v2 = horFace->verts[(horEdge + 1) % 3];
    horFace->adj[horEdge] = nfaces;
    if (attachFace(pt, wi, v2, v1, nfaces + nedges - 1, horIndex, nfaces + 1)) {
      break;  // out of memory
    }

    // attach remaining faces
    int oom = 0;  // set to 1 if out of memory
    for (int i = 1; i < nedges; i++) {
      int cur = nfaces + i;  // index of attached face
      int next = nfaces + (i + 1) % nedges;  // index of next face

      horIndex = h.indices[i], horEdge = h.edges[i];
      horFace = &pt->faces[horIndex];
      v1 = horFace->verts[horEdge];
      v2 = horFace->verts[(horEdge + 1) % 3];
      horFace->adj[horEdge] = cur;
      if (attachFace(pt, wi, v2, v1, cur - 1, horIndex, next)) {
        oom = 1;
        break;
      }
    }
    if (oom) break;
    h.nedges = 0;  // clear horizon
  }

  mj_freeStack(d);
  epaWitness(pt, face, status->x1, status->x2);
  status->epa_iterations = k;
  return dist;
}



// general convex collision detection
mjtNum mjc_ccd(const mjCCDConfig* config, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2) {
  // set up
  obj1->center(status->x1, obj1);
  obj2->center(status->x2, obj2);
  status->epa_iterations = -1;
  status->tolerance = config->tolerance;
  status->max_iterations = config->max_iterations;
  status->has_contacts = config->contacts;
  status->has_distances = config->distances;

  mjtNum dist = gjk(status, obj1, obj2);

  // penetration recovery for contacts not needed
  if (!config->contacts) {
    return dist;
  }

  if (dist <= config->tolerance && status->nsimplex > 1) {
    int N = status->max_iterations;
    mjData* d = (mjData*) obj1->data;
    mj_markStack((mjData*) obj1->data);

    Polytope pt;
    pt.nfaces = pt.nheap = pt.nverts = 0;

    // allocate memory for faces
    pt.maxfaces = (6*N > 1000) ? 6*N : 1000;  // use 1000 faces as lower bound
    pt.faces = mj_stackAllocByte(d, sizeof(Face) * pt.maxfaces, _Alignof(Face));
    pt.heap = mj_stackAllocByte(d, sizeof(Face*) * pt.maxfaces, _Alignof(Face*));

    // allocate memory for vertices
    pt.verts  = mj_stackAllocNum(d, 3*(5 + N));
    pt.verts1 = mj_stackAllocNum(d, 3*(5 + N));
    pt.verts2 = mj_stackAllocNum(d, 3*(5 + N));

    int ret;
    if (status->nsimplex == 2) {
      ret = polytope2(&pt, status, obj1, obj2);
    } else if (status->nsimplex == 3) {
      ret = polytope3(&pt, status, obj1, obj2);
    } else {
      ret = polytope4(&pt, status);
    }

    // simplex not on boundary (objects are penetrating)
    if (ret) {
      dist = -epa(status, &pt, obj1, obj2);
    } else {
      dist = 0;
    }
    mj_freeStack(d);
  }
  return dist;
}
