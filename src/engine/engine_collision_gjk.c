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
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"

// Computes the shortest distance between the origin and an n-simplex (n <= 3) and returns the
// barycentric coordinates of the closest point in the simplex. This is the so called distance
// sub-algorithm of the original 1988 GJK algorithm.
//
//  We have adapted the Signed Volume method for our approach from the paper:
//     Improving the GJK Algorithm for Faster and More Reliable Distance Queries Between Two
//     Convex Objects, Montanari et al, ToG 2017.
static void signedVolume(mjtNum lambda[4], const mjtNum simplex[12], int n);

// these internal functions compute the barycentric coordinates of the closest point
// to the origin in the n-simplex, where n = 3, 2, 1 respectively
static void S3D(mjtNum lambda[4], const mjtNum simplex[12]);
static void S2D(mjtNum lambda[3], const mjtNum simplex[9]);
static void S1D(mjtNum lambda[2], const mjtNum simplex[6]);

// helper function to compute the support point in the Minkowski difference
static void support(mjtNum s[3], mjCCDObj* obj1, mjCCDObj* obj2, const mjtNum d[3]);

// support function tweaked for GJK by taking kth iteration point as input and setting both
// support points to recover witness points
static void gjk_support(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                        const mjtNum x_k[3]);

// linear algebra utility functions
static mjtNum det3(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]);
static void lincomb(mjtNum res[3], const mjtNum* coef, const mjtNum* v, int n);

typedef struct {
  int ignored;
  int verts[3];  // indices of the three vertices of the face in the polytope
  mjtNum v[3];
  mjtNum dist;
  mjtNum n[3];
} Face;

typedef struct {
  mjtNum* verts;
  int nverts;
  int vcap;
  Face* faces;
  int nfaces;
  int fcap;
} Polytope;

// generates a polytope from a 1-simplex, 2-simplex, or 3-simplex respectively
// returns true if the polytope can be generated, false otherwise
static int polytope2(Polytope* pt, const mjtNum simplex[6], mjCCDObj* obj1, mjCCDObj* obj2);
static int polytope3(Polytope* pt, const mjtNum simplex[9]);
static int polytope4(Polytope* pt, const mjtNum simplex[12]);

// initializes the polytope (faces and vertices must be freed by caller)
static void initPolytope(Polytope* pt);

// copies a vertex into the polytope and return its index
static int newVertex(Polytope* pt, const mjtNum v1[3]);

// attaches a face to the polytope with the given vertex indices in the polytope
static void attachFace(Polytope* pt, int v1, int v2, int v3);

// returns the penetration depth (negative distance) of the convex objects
static mjtNum epa(const mjCCDConfig* config, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2);

// internal data structure for the returning simplex from GJK
typedef struct {
  mjtNum verts[12];
  int nverts;
} Simplex;

// internal GJK with returned data for EPA
static mjtNum _gjk(const mjCCDConfig* config, mjCCDObj* obj1, mjCCDObj* obj2, Simplex* ret) {
  mjtNum simplex[12];  // our current simplex with max 4 vertices due to only 3 dimensions
  int n = 0;           // number of vertices in the simplex
  mjtNum x_k[3];       // the kth approximation point with initial value x_0

  // segregated simplices and points for the two objects to recover witness points
  mjtNum simplex1[12], simplex2[12];
  mjtNum* x1_k = obj1->x0;
  mjtNum* x2_k = obj2->x0;
  mju_sub3(x_k, x1_k, x2_k);

  int N = config->max_iterations;
  for (size_t k = 0; k < N; k++) {
    mjtNum s1[3], s2[3];  // the support points in obj1 and obj2
    mjtNum s_k[3];        // the kth support point of Minkowski difference
    mjtNum lambda[4];     // barycentric coordinates for x_k

    // compute the kth support point
    gjk_support(s1, s2, obj1, obj2, x_k);
    mju_sub3(s_k, s1, s2);

    // the stopping criteria relies on the Frank-Wolfe duality gap given by
    //  f(x_k) - f(x_min) <= < grad f(x_k), (x_k - s_k) >
    mjtNum diff[3];
    mju_sub3(diff, x_k, s_k);
    if (2*mju_dot3(x_k, diff) < config->tolerance) {
      break;
    }

    // TODO(kylebayes): signedVolume has been written to assume the first vertex is the latest
    // support to be added. Once the logic has been updated, then this hack should be removed.
    for (int i = n; i > 0; i--) {
      // shift the simplex vertices to the right
      mju_copy3(simplex  + 3*i, simplex  + 3*(i-1));
      mju_copy3(simplex1 + 3*i, simplex1 + 3*(i-1));
      mju_copy3(simplex2 + 3*i, simplex2 + 3*(i-1));
    }

    // copy new support point into the simplex
    mju_copy3(simplex, s_k);

    // copy new support point into the individual simplexes
    mju_copy3(simplex1, s1);
    mju_copy3(simplex2, s2);

    // run the distance subalgorithm to compute the barycentric coordinates
    // of the closest point to the origin in the simplex
    signedVolume(lambda, simplex, ++n);
    lincomb(x_k, lambda, simplex, 4);

    // compute the approximate witness points
    lincomb(x1_k, lambda, simplex1, 4);
    lincomb(x2_k, lambda, simplex2, 4);

    // for lambda[i] == 0, remove the ith vertex from the simplex
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (lambda[i] == 0) continue;
      // recover simplex for the two objects
      mju_copy3(simplex1 + 3*n, simplex1 + 3*i);
      mju_copy3(simplex2 + 3*n, simplex2 + 3*i);

      // simplex in Minkowski difference
      mju_copy3(simplex + 3*n++, simplex + 3*i);
    }
  }
  if (ret) {
    ret->nverts = n;
    for (int i = 0; i < n; i++) {
      mju_copy3(ret->verts + 3*i, simplex + 3*i);
    }
  }
  return mju_norm3(x_k);
}



// returns the distance between the two objects. The witness points are
// recoverable from the x_0 field in obj1 and obj2.
mjtNum mj_gjk(const mjCCDConfig* config, mjCCDObj* obj1, mjCCDObj* obj2) {
  return _gjk(config, obj1, obj2, NULL);
}



// Same as mj_gjk, but returns the penetration depth (negative distance) if the objects intersect.
mjtNum mj_gjkPenetration(const mjCCDConfig* config, mjCCDObj* obj1, mjCCDObj* obj2) {
  Simplex simplex;
  mjtNum dist = _gjk(config, obj1, obj2, &simplex);

  if (dist <= config->tolerance && simplex.nverts > 1) {
    Polytope pt;
    int ret;
    if (simplex.nverts == 2) {
      ret = polytope2(&pt, simplex.verts, obj1, obj2);
    } else if (simplex.nverts == 3) {
      ret = polytope3(&pt, simplex.verts);
    } else {
      ret = polytope4(&pt, simplex.verts);
    }

    // simplex not on boundary (objects are penetrating)
    if (ret) {
      dist = -epa(config, &pt, obj1, obj2);
    }
    mju_free(pt.faces);
    mju_free(pt.verts);
  }
  return dist;
}



// computes the support points in obj1 and obj2 for the kth approximation point
static void gjk_support(mjtNum s1[3], mjtNum s2[3], mjCCDObj* obj1, mjCCDObj* obj2,
                        const mjtNum x_k[3]) {
  mjtNum dir[3], dir_neg[3];
  mju_copy3(dir_neg, x_k);
  mju_normalize3(dir_neg);  // mjc_support assumes a normalized direction
  mju_scl3(dir, dir_neg, -1);

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  mjc_support(s1, obj1, dir);
  mjc_support(s2, obj2, dir_neg);
}



// helper function to compute the support point in the Minkowski difference
static void support(mjtNum s[3], mjCCDObj* obj1, mjCCDObj* obj2,
                    const mjtNum d[3]) {
  mjtNum dir[3], dir_neg[3], s1[3], s2[3];
  mju_copy3(dir, d);
  mju_normalize3(dir);  // mjc_support assumes a normalized direction
  mju_scl3(dir_neg, dir, -1);

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  mjc_support(s1, obj1, dir);
  mjc_support(s2, obj2, dir_neg);
  mju_sub3(s, s1, s2);
}



// linear combination of n 3D vectors:
//   res = coef[0]*v[0] + ... + coef[n-1]*v[3*(n-1)]
static void lincomb(mjtNum res[3], const mjtNum* coef, const mjtNum* v, int n) {
  mju_zero3(res);
  for (int i = 0; i < n; i++) {
    if (coef[i] == 0) continue;
    res[0] += coef[i] * v[3*i + 0];
    res[1] += coef[i] * v[3*i + 1];
    res[2] += coef[i] * v[3*i + 2];
  }
}



// returns determinant of the 3x3 matrix with columns v1, v2, v3
static mjtNum det3(const mjtNum v1[3], const mjtNum v2[3], const mjtNum v3[3]) {
  mjtNum temp[3];
  mju_cross(temp, v2, v3);
  return mju_dot3(v1, temp);
}



// returns true only when a and b are both strictly positive or both strictly negative
static int compareSigns(mjtNum a, mjtNum b) {
  if (a > 0 && b > 0) return 1;
  if (a < 0 && b < 0) return 1;
  return 0;
}



// computes the barycentric coordinates of the closest point to the origin in the n-simplex
void signedVolume(mjtNum lambda[4], const mjtNum simplex[12], int n) {
  int r = n - 1;  // spatial dimension of the simplex
  mju_zero4(lambda);

  if (r == 3) {
    S3D(lambda, simplex);
  } else if (r == 2) {
    S2D(lambda, simplex);
  } else if (r == 1) {
    S1D(lambda, simplex);
  } else {
    lambda[0] = 1;
  }
}



static void S3D(mjtNum lambda[4], const mjtNum simplex[12]) {
  // the four vertices of the 3-simplex that correspond to 4 support points
  const mjtNum* s1 = simplex;
  const mjtNum* s2 = simplex + 3;
  const mjtNum* s3 = simplex + 6;
  const mjtNum* s4 = simplex + 9;

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

  int comp1 = compareSigns(m_det, C41),
      comp2 = compareSigns(m_det, C42),
      comp3 = compareSigns(m_det, C43),
      comp4 = compareSigns(m_det, C44);

  // if all signs are the same then the origin is inside the simplex
  if (comp1 && comp2 && comp3 && comp4) {
    lambda[0] = C41 / m_det;
    lambda[1] = C42 / m_det;
    lambda[2] = C43 / m_det;
    lambda[3] = C44 / m_det;
    return;
  }

  // find the smallest distance, and use the corresponding barycentric coordinates
  mjtNum dist = mjMAXVAL;

  if (!comp2) {
    mjtNum lambda_2d[3], verts[9], x[3];
    mju_copy3(verts, s1);
    mju_copy3(verts + 3, s3);
    mju_copy3(verts + 6, s4);
    S2D(lambda_2d, verts);
    lincomb(x, lambda_2d, verts, 3);
    mjtNum d = mju_norm3(x);
    lambda[0] = lambda_2d[0];
    lambda[1] = 0;
    lambda[2] = lambda_2d[1];
    lambda[3] = lambda_2d[2];
    dist = d;
  }

  if (!comp3) {
    mjtNum lambda_2d[3], verts[9], x[3];
    mju_copy3(verts, s1);
    mju_copy3(verts + 3, s2);
    mju_copy3(verts + 6, s4);
    S2D(lambda_2d, verts);
    lincomb(x, lambda_2d, verts, 3);
    mjtNum d = mju_norm3(x);
    if (d < dist) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = 0;
      lambda[3] = lambda_2d[2];
      dist = d;
    }
  }

  if (!comp4) {
    mjtNum lambda_2d[3], verts[9], x[3];
    mju_copy3(verts, s1);
    mju_copy3(verts + 3, s2);
    mju_copy3(verts + 6, s3);
    S2D(lambda_2d, verts);
    lincomb(x, lambda_2d, verts, 3);
    mjtNum d = mju_norm3(x);
    if (d < dist) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = lambda_2d[2];
      lambda[3] = 0;
      dist = d;
    }
  }

  if (!comp1) {
    mjtNum lambda_2d[3], verts[9], x[3];
    mju_copy3(verts, s2);
    mju_copy3(verts + 3, s3);
    mju_copy3(verts + 6, s4);
    S2D(lambda_2d, verts);
    lincomb(x, lambda_2d, verts, 3);
    mjtNum d = mju_norm3(x);
    if (d < dist) {
      lambda[0] = 0;
      lambda[1] = lambda_2d[0];
      lambda[2] = lambda_2d[1];
      lambda[3] = lambda_2d[2];
      dist = d;
    }
  }
}



static void S2D(mjtNum lambda[3], const mjtNum simplex[9]) {
  // the three vertices of the 2-simplex that correspond to 3 support points
  const mjtNum* s1 = simplex;
  const mjtNum* s2 = simplex + 3;
  const mjtNum* s3 = simplex + 6;

  // compute normal
  mjtNum diff1[3], diff2[3], n[3];
  mju_sub3(diff1, s2, s1);
  mju_sub3(diff2, s3, s1);
  mju_cross(n, diff1, diff2);

  // project origin
  mjtNum p_o[3];
  mju_scl3(p_o, n, mju_dot3(n, s1) / mju_dot3(n, n));

  mjtNum mu_max = 0;

  //  Below are the minors M_i4 of the matrix M given by
  //  [[ s1_x, s2_x, s3_x, s4_x ],
  //   [ s1_y, s2_y, s3_y, s4_y ],
  //   [ s1_z, s2_z, s3_z, s4_z ],
  //   [ 1,    1,    1,    1    ]]
  mjtNum M_14 = s2[1]*s3[2] - s2[2]*s3[1] - s1[1]*s3[2] + s1[2]*s3[1] + s1[1]*s2[2] - s1[2]*s2[1];
  mjtNum M_24 = s2[0]*s3[2] - s2[2]*s3[0] - s1[0]*s3[2] + s1[2]*s3[0] + s1[0]*s2[2] - s1[2]*s2[0];
  mjtNum M_34 = s2[0]*s3[1] - s2[1]*s3[0] - s1[0]*s3[1] + s1[1]*s3[0] + s1[0]*s2[1] - s1[1]*s2[0];

  // exclude one of the axes with the largest projection of the simplex using the computed minors
  mjtNum s1_2D[2], s2_2D[2], s3_2D[2], p_o_2D[2];
  mjtNum mu1 = mju_abs(M_14), mu2 = mju_abs(M_24), mu3 = mju_abs(M_34);
  if (mu1 >= mu2 && mu1 >= mu3) {
    mu_max = mu1;
    s1_2D[0] = s1[1];
    s1_2D[1] = s1[2];

    s2_2D[0] = s2[1];
    s2_2D[1] = s2[2];

    s3_2D[0] = s3[1];
    s3_2D[1] = s3[2];

    p_o_2D[0] = p_o[1];
    p_o_2D[1] = p_o[2];
  } else if (mu2 >= mu3) {
    mu_max = mu2;
    s1_2D[0] = s1[0];
    s1_2D[1] = s1[2];

    s2_2D[0] = s2[0];
    s2_2D[1] = s2[2];

    s3_2D[0] = s3[0];
    s3_2D[1] = s3[2];

    p_o_2D[0] = p_o[0];
    p_o_2D[1] = p_o[2];
  } else {
    mu_max = mu3;
    s1_2D[0] = s1[0];
    s1_2D[1] = s1[1];

    s2_2D[0] = s2[0];
    s2_2D[1] = s2[1];

    s3_2D[0] = s3[0];
    s3_2D[1] = s3[1];

    p_o_2D[0] = p_o[0];
    p_o_2D[1] = p_o[1];
  }

  // substitute p_o as a vertex in simplex
  mjtNum C1 = p_o_2D[0]*s2_2D[1] + p_o_2D[1]*s3_2D[0] + s2_2D[0]*s3_2D[1]
            - p_o_2D[0]*s3_2D[1] - p_o_2D[1]*s2_2D[0] - s3_2D[0]*s2_2D[1];

  mjtNum C2 = p_o_2D[0]*s3_2D[1] + p_o_2D[1]*s1_2D[0] + s3_2D[0]*s1_2D[1]
            - p_o_2D[0]*s1_2D[1] - p_o_2D[1]*s3_2D[0] - s1_2D[0]*s3_2D[1];

  mjtNum C3 = p_o_2D[0]*s1_2D[1] + p_o_2D[1]*s2_2D[0] + s1_2D[0]*s2_2D[1]
            - p_o_2D[0]*s2_2D[1] - p_o_2D[1]*s1_2D[0] - s2_2D[0]*s1_2D[1];

  int comp1 = compareSigns(mu_max, C1),
      comp2 = compareSigns(mu_max, C2),
      comp3 = compareSigns(mu_max, C3);

  // inside the simplex
  if (comp1 && comp2 && comp3) {
    lambda[0] = C1 / mu_max;
    lambda[1] = C2 / mu_max;
    lambda[2] = C3 / mu_max;
    return;
  }

  // find the smallest distance, and use the corresponding barycentric coordinates
  mjtNum dist = mjMAXVAL;

  if (!comp2) {
    mjtNum lambda_1d[2], verts[6], x[3];
    mju_copy3(verts, s1);
    mju_copy3(verts + 3, s3);
    S1D(lambda_1d, verts);
    lincomb(x, lambda_1d, verts, 2);
    mjtNum d = mju_norm3(x);
    lambda[0] = lambda_1d[0];
    lambda[1] = 0;
    lambda[2] = lambda_1d[1];
    dist = d;
  }

  if (!comp3) {
    mjtNum lambda_1d[2], verts[6], x[3];
    mju_copy3(verts, s1);
    mju_copy3(verts + 3, s2);
    S1D(lambda_1d, verts);
    lincomb(x, lambda_1d, verts, 2);
    mjtNum d = mju_norm3(x);
    if (d < dist) {
      lambda[0] = lambda_1d[0];
      lambda[1] = lambda_1d[1];
      lambda[2] = 0;
      dist = d;
    }
  }

  if (!comp1) {
    mjtNum lambda_1d[2], verts[6], x[3];
    mju_copy3(verts, s2);
    mju_copy3(verts + 3, s3);
    S1D(lambda_1d, verts);
    lincomb(x, lambda_1d, verts, 2);
    mjtNum d = mju_norm3(x);
    if (d < dist) {
      lambda[1] = lambda_1d[0];
      lambda[2] = lambda_1d[1];
      lambda[0] = 0;
      dist = d;
    }
  }
}



static void S1D(mjtNum lambda[2], const mjtNum simplex[6]) {
  // the two vertices of the 1-simplex correspond to 2 support points
  const mjtNum* s1 = simplex;
  const mjtNum* s2 = simplex + 3;

  // find projection of origin onto the 1-simplex:
  //   p_o = s2 - <s2, s2 - s1> / <s2 - s1, s2 - s1> * (s2 - s1)
  mjtNum p_o[3];
  mjtNum diff[3];
  mju_sub3(diff, s2, s1);
  mjtNum temp1 = mju_dot3(s2, diff);
  mjtNum temp2 = mju_dot3(diff, diff);
  mju_addScl3(p_o, s2, diff, - temp1 / temp2);

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
  if (compareSigns(mu_max, C1) && compareSigns(mu_max, C2)) {
    lambda[0] = C1 / mu_max;
    lambda[1] = C2 / mu_max;
  } else {
    lambda[0] = 1;
    lambda[1] = 0;
  }
}



// helper function to test if the origin is in the same side of the plane formed by P0P1P2 as P3.
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



// determines if the origin is contained in the tetrahedron.
static int testTetra(const mjtNum p0[3], const mjtNum p1[3],
                     const mjtNum p2[3], const mjtNum p3[3]) {
  return sameSide(p0, p1, p2, p3)
      && sameSide(p1, p2, p3, p0)
      && sameSide(p2, p3, p0, p1)
      && sameSide(p3, p0, p1, p2);
}



// sets rotation matrix for 120 degrees along axis
static void rotmat(mjtNum R[9], const mjtNum axis[3]) {
  mjtNum n = mju_norm3(axis);
  mjtNum u1 = axis[0] / n, u2 = axis[1] / n, u3 = axis[2] / n;
  const mjtNum sin = 0.86602540378;  // sin(120 deg) = sqrt(3)/2 ~ 0.86602540378
  const mjtNum cos = -0.5;           // cos(120 deg) = -1/2
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
static int polytope2(Polytope* pt, const mjtNum simplex[6],
                              mjCCDObj* obj1, mjCCDObj* obj2) {
  initPolytope(pt);
  const mjtNum* s1 = simplex;
  const mjtNum* s2 = simplex + 3;
  mjtNum diff[3];
  mju_sub3(diff, s2, s1);

  // find component with largest magnitude (so cross product is largest)
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

  mju_mulMatVec(d2, R, d1, 3, 3);
  mju_mulMatVec(d3, R, d2, 3, 3);


  mjtNum v1[3], v2[3], v3[3];
  support(v1, obj1, obj2, d1);
  support(v2, obj1, obj2, d2);
  support(v3, obj1, obj2, d3);

  // points of a hexahedron (we test to see what half the origin is contained in)
  int s1i = newVertex(pt, s1);
  int v1i = newVertex(pt, v1);
  int v2i = newVertex(pt, v2);
  int v3i = newVertex(pt, v3);
  int s2i = newVertex(pt, s2);

  if (testTetra(s1, v1, v2, v3)) {
    attachFace(pt, s1i, v2i, v1i);
    attachFace(pt, s1i, v3i, v1i);
    attachFace(pt, s1i, v3i, v2i);
    attachFace(pt, v1i, v2i, v3i);
    return 1;
  }

  if (testTetra(s2, v1, v2, v3)) {
    attachFace(pt, s2i, v1i, v2i);
    attachFace(pt, s2i, v1i, v3i);
    attachFace(pt, s2i, v2i, v3i);
    attachFace(pt, v1i, v2i, v3i);
    return 1;
  }
  return 0;
}



// creates a polytope from a 2-simplex (3 points i.e. triangle)
static int polytope3(Polytope* pt, const mjtNum simplex[9]) {
  initPolytope(pt);

  const mjtNum* s1 = simplex;
  const mjtNum* s2 = simplex + 3;
  const mjtNum* s3 = simplex + 6;

  // form hexahedron from triangle and two face normals

  mjtNum diff1[3], diff2[3], n[3], neg_n[3];
  mju_sub3(diff1, s2, s1);
  mju_sub3(diff2, s3, s1);
  mju_cross(n, diff1, diff2);
  mju_scl3(neg_n, n, -1);

  int ni = newVertex(pt, n);
  int s1i = newVertex(pt, s1);
  int s2i = newVertex(pt, s2);
  int s3i = newVertex(pt, s3);
  int nni = newVertex(pt, neg_n);

  attachFace(pt, s1i, s2i, ni);
  attachFace(pt, s3i, s1i, ni);
  attachFace(pt, s2i, s3i, ni);

  attachFace(pt, s1i, s2i, nni);
  attachFace(pt, s3i, s1i, nni);
  attachFace(pt, s2i, s3i, nni);

  // TODO(kylebayes): check what side of the hexahedron the origin is on
  return 1;
}



// creates a polytope from a 3-simplex (4 points i.e. tetrahedron)
static int polytope4(Polytope* pt, const mjtNum simplex[12]) {
  initPolytope(pt);

  int v1 = newVertex(pt, simplex);
  int v2 = newVertex(pt, simplex + 3);
  int v3 = newVertex(pt, simplex + 6);
  int v4 = newVertex(pt, simplex + 9);

  attachFace(pt, v1, v2, v3);
  attachFace(pt, v1, v2, v4);
  attachFace(pt, v1, v4, v3);
  attachFace(pt, v4, v2, v3);

  // TODO(kylebayes): check if contains origin
  return 1;
}

#define mjMINCAP 100  // starting capacity for dynamic buffers

// an edge in the horizon
typedef struct {
  int v1;
  int v2;
  int ignore;  // deleted
} Edge;

// the horizon of the polytope
typedef struct {
  Edge* edges;   // edges in horizon
  int n;
  int capacity;
} Horizon;



// initializes the polytope (faces and vertices must be freed by caller)
static void initPolytope(Polytope* pt) {
  // vertices
  pt->nverts = 0;
  pt->vcap = mjMINCAP;
  pt->verts = (mjtNum*) mju_malloc(pt->vcap * 3 * sizeof(mjtNum));

  // faces
  pt->nfaces = 0;
  pt->fcap = mjMINCAP;
  pt->faces = (Face*) mju_malloc(pt->fcap * sizeof(Face));
}



// copies a vertex into the polytope and return its index
static int newVertex(Polytope* pt, const mjtNum v[3]) {
  int capacity = pt->vcap;
  int n = pt->nverts++;
  if (n == capacity) {
    capacity *= 2;
    pt->verts = (mjtNum*) realloc(pt->verts, capacity * 3 * sizeof(mjtNum));
    pt->vcap = capacity;
  }


  mju_copy3(pt->verts + 3*n, v);
  return n;
}



// attaches a face to the polytope with the given vertex indices in the polytope
static void attachFace(Polytope* pt, int v1, int v2, int v3) {
  int capacity = pt->fcap;
  if (pt->nfaces == capacity) {
    capacity *= 2;
    pt->faces = (Face*) realloc(pt->faces, capacity * sizeof(Face));
    pt->fcap = capacity;
  }

  Face* face = &pt->faces[pt->nfaces];
  face->ignored = 0;
  face->verts[0] = v1;
  face->verts[1] = v2;
  face->verts[2] = v3;

  // compute normal n
  mjtNum* pv1 = pt->verts + (v1 * 3);
  mjtNum* pv2 = pt->verts + (v2 * 3);
  mjtNum* pv3 = pt->verts + (v3 * 3);
  mjtNum diff1[3], diff2[3];
  mju_sub3(diff1, pv2, pv1);
  mju_sub3(diff2, pv3, pv1);
  mju_cross(face->n, diff1, diff2);
  mju_normalize3(face->n);

  // compute witness point v
  mju_scl3(face->v, face->n, mju_dot3(face->n, pv1));
  face->dist = mju_norm3(face->v);

  // orientation check
  if (mju_dot3(face->n, pv1) < 0) mju_scl3(face->n, face->n, -1);
  pt->nfaces++;
}



// initializes the horizon (edges must be freed by caller)
static void initHorizon(Horizon* h) {
  h->n = 0;
  h->capacity = mjMINCAP;
  h->edges = (Edge*) mju_malloc(h->capacity * sizeof(Edge));
}



// adds an edge to the horizon, if the edge is already in the horizon, it is
// deleted (marked ignored)
static void addEdgeIfUnique(Horizon* h, int v1, int v2) {
  int capacity = h->capacity;
  int n = h->n;

  for (int i = 0; i < n; i++) {
    if (h->edges[i].ignore) continue;
    int old_v1 = h->edges[i].v1;
    int old_v2 = h->edges[i].v2;
    if ((old_v1 == v1 && old_v2 == v2) || (old_v1 == v2 && old_v2 == v1)) {
      h->edges[i].ignore = 1;
      return;
    }
  }
  if (n == capacity) {
    capacity *= 2;
    h->edges = (Edge*) realloc(h->edges, capacity * sizeof(Edge));
    h->capacity = capacity;
  }
  h->edges[n].v1 = v1;
  h->edges[n].v2 = v2;
  h->edges[n].ignore = 0;
  h->n++;
}

#undef mjMINCAP


// returns the penetration depth (negative distance) of the convex objects
static mjtNum epa(const mjCCDConfig* config, Polytope* pt, mjCCDObj* obj1, mjCCDObj* obj2) {
  mjtNum dist = mjMAXVAL;
  Horizon h;
  initHorizon(&h);
  int N = config->max_iterations;
  mjtNum tolerance = config->tolerance;

  for (int j = 0; j < N; j++) {
    dist = mjMAXVAL;
    int index = -1;

    // find the closest face to the origin
    for (int i = 0; i < pt->nfaces; i++) {
      if (pt->faces[i].ignored) continue;
      if (pt->faces[i].dist < dist) {
        dist = pt->faces[i].dist;
        index = i;
      }
    }

    // compute support point w from the closest face's normal
    mjtNum w[3];
    support(w, obj1, obj2, pt->faces[index].v);
    mjtNum next_dist = mju_dot3(pt->faces[index].v, w) / dist;
    if (next_dist - dist < tolerance) {
      break;
    }

    // compute horizon for w
    for (int i = 0; i < pt->nfaces; i++) {
      Face* face = &pt->faces[i];
      if (face->ignored) continue;
      mjtNum dist2 = face->dist;
      mjtNum* v = face->v;  // use witness point as normal
      if (mju_dot3(v, w) >= dist2*dist2) {
        face->ignored = 1;
        addEdgeIfUnique(&h, face->verts[0], face->verts[1]);
        addEdgeIfUnique(&h, face->verts[1], face->verts[2]);
        addEdgeIfUnique(&h, face->verts[2], face->verts[0]);
      }
    }

    // insert w as new vertex and attach faces along the horizon
    int wi = newVertex(pt, w);
    for (int i = 0; i < h.n; i++) {
      if (h.edges[i].ignore) continue;
      attachFace(pt, wi, h.edges[i].v1, h.edges[i].v2);
    }

    h.n = 0;  // clear horizon
  }
  mju_free(h.edges);
  return dist;
}
