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

#include "engine/engine_collision_primitive.h"

#include <math.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_util_blas.h"
#include "engine/engine_util_spatial.h"


//--------------------------- plane collisions -----------------------------------------------------

// plane : sphere (actual implementation, can be called with modified parameters)
static int _PlaneSphere(mjContact* con, mjtNum margin,
                        mjtNum* pos1, mjtNum* mat1, mjtNum* size1,
                        mjtNum* pos2, mjtNum* mat2, mjtNum* size2) {
  mjtNum tmp[3];
  mjtNum cdist;

  // set normal
  con[0].frame[0] = mat1[2];
  con[0].frame[1] = mat1[5];
  con[0].frame[2] = mat1[8];

  // compute distance, return if too large
  mju_sub3(tmp, pos2, pos1);
  cdist = mju_dot3(tmp, con[0].frame);
  if (cdist > margin + size2[0]) {
    return 0;
  }

  // depth and position
  con[0].dist = cdist - size2[0];
  mju_scl3(tmp, con[0].frame, -con[0].dist/2 - size2[0]);
  mju_add3(con[0].pos, pos2, tmp);

  mju_zero3(con[0].frame+3);
  return 1;
}



// plane : sphere
int mjc_PlaneSphere(const mjModel* m, const mjData* d,
                    mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  return _PlaneSphere(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}



// plane : capsule
int mjc_PlaneCapsule(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum pos[3], axis[3], segment[3];
  int n1, n2;

  // get capsule axis, segment = scaled axis
  axis[0] = mat2[2];
  axis[1] = mat2[5];
  axis[2] = mat2[8];
  mju_scl3(segment, axis, size2[1]);

  // get point 1, do sphere-plane test
  mju_add3(pos, pos2, segment);
  n1 = _PlaneSphere(con, margin, pos1, mat1, size1, pos, mat2, size2);

  // get point 2, do sphere-plane test
  mju_sub3(pos, pos2, segment);
  n2 = _PlaneSphere(con+n1, margin, pos1, mat1, size1, pos, mat2, size2);

  // align contact frames with capsule axis
  if (n1) {
    mju_copy3(con->frame+3, axis);
  }
  if (n2) {
    mju_copy3((con+n1)->frame+3, axis);
  }

  return n1+n2;
}



// plane : cylinder
int mjc_PlaneCylinder(const mjModel* m, const mjData* d,
                      mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum normal[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum axis[3] = {mat2[2], mat2[5], mat2[8]};
  mjtNum vec[3], vec1[3];
  mjtNum len, scl, dist0, prjaxis, prjvec, prjvec1;
  int cnt = 0;

  // project, make sure axis points towards plane
  prjaxis = mju_dot3(normal, axis);
  if (prjaxis > 0) {
    mju_scl3(axis, axis, -1);
    prjaxis = -prjaxis;
  }

  // compute normal distance to cylinder center
  mju_sub3(vec, pos2, pos1);
  dist0 = mju_dot3(vec, normal);

  // remove component of -normal along axis, compute length
  mju_scl3(vec, axis, prjaxis);
  mju_subFrom3(vec, normal);
  len = mju_norm3(vec);

  // general configuration: normalize vector, scale by radius
  if (len >= mjMINVAL) {
    scl = size2[0]/len;
    vec[0] *= scl;
    vec[1] *= scl;
    vec[2] *= scl;
  }

  // disk parallel to plane: pick x-axis of cylinder, scale by radius
  else {
    vec[0] = mat2[0]*size2[0];
    vec[1] = mat2[3]*size2[0];
    vec[2] = mat2[6]*size2[0];
  }

  // project vector on normal
  prjvec = mju_dot3(vec, normal);

  // scale axis by half-length
  mju_scl3(axis, axis, size2[1]);
  prjaxis *= size2[1];

  // check first point, construct contact
  if (dist0 + prjaxis + prjvec <= margin) {
    con[cnt].dist = dist0 + prjaxis + prjvec;
    mju_add3(con[cnt].pos, pos2, vec);
    mju_addTo3(con[cnt].pos, axis);
    mju_addToScl3(con[cnt].pos, normal, -con[cnt].dist*0.5);
    mju_copy3(con[cnt].frame, normal);
    mju_zero3(con[cnt].frame+3);
    cnt++;
  } else {
    return 0;  // nearest point is above margin: no contacts
  }

  // check second point, construct contact
  if (dist0 - prjaxis + prjvec <= margin) {
    con[cnt].dist = dist0 - prjaxis + prjvec;
    mju_add3(con[cnt].pos, pos2, vec);
    mju_subFrom3(con[cnt].pos, axis);
    mju_addToScl3(con[cnt].pos, normal, -con[cnt].dist*0.5);
    mju_copy3(con[cnt].frame, normal);
    mju_zero3(con[cnt].frame+3);
    cnt++;
  }

  // try to add triangle points on side closer to plane
  prjvec1 = -prjvec*0.5;
  if (dist0 + prjaxis + prjvec1 <= margin) {
    // compute sideways vector: vec1
    mju_cross(vec1, vec, axis);
    mju_normalize3(vec1);
    mju_scl3(vec1, vec1, size2[0]*mju_sqrt(3.0)/2);

    // add point A
    con[cnt].dist = dist0 + prjaxis + prjvec1;
    mju_add3(con[cnt].pos, pos2, vec1);
    mju_addTo3(con[cnt].pos, axis);
    mju_addToScl3(con[cnt].pos, vec, -0.5);
    mju_addToScl3(con[cnt].pos, normal, -con[cnt].dist*0.5);
    mju_copy3(con[cnt].frame, normal);
    mju_zero3(con[cnt].frame+3);
    cnt++;

    // add point B
    con[cnt].dist = dist0 + prjaxis + prjvec1;
    mju_sub3(con[cnt].pos, pos2, vec1);
    mju_addTo3(con[cnt].pos, axis);
    mju_addToScl3(con[cnt].pos, vec, -0.5);
    mju_addToScl3(con[cnt].pos, normal, -con[cnt].dist*0.5);
    mju_copy3(con[cnt].frame, normal);
    mju_zero3(con[cnt].frame+3);
    cnt++;
  }

  return cnt;
}



// plane : box
int mjc_PlaneBox(const mjModel* m, const mjData* d,
                 mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  int cnt = 0;

  // get normal, difference between centers, normal distance
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3], vec[3], corner[3], dist, ldist;
  mju_sub3(dif, pos2, pos1);
  dist = mju_dot3(dif, norm);

  // test all corners, pick bottom 4
  for (int i=0; i<8; i++) {
    // get corner in local coordinates
    vec[0] = (i&1 ? size2[0] : -size2[0]);
    vec[1] = (i&2 ? size2[1] : -size2[1]);
    vec[2] = (i&4 ? size2[2] : -size2[2]);

    // get corner in global coordinates relative to box center
    mju_rotVecMat(corner, vec, mat2);

    // compute distance to plane, skip if too far or pointing up
    ldist = mju_dot3(norm, corner);
    if (dist + ldist > margin || ldist > 0) {
      continue;
    }

    // construct contact
    con[cnt].dist = dist + ldist;
    mju_copy3(con[cnt].frame, norm);
    mju_zero3(con[cnt].frame+3);
    mju_addTo3(corner, pos2);
    mju_scl3(vec, norm, -con[cnt].dist/2);
    mju_add3(con[cnt].pos, corner, vec);

    // count; max is 4
    if (++cnt >= 4) {
      return 4;
    }
  }

  return cnt;
}



//--------------------------- sphere and capsule collisions ----------------------------------------

// sphere : sphere (actual implementation, can be called with modified parameters)
static int _SphereSphere(mjContact* con, mjtNum margin,
                         mjtNum* pos1, mjtNum* mat1, mjtNum* size1,
                         mjtNum* pos2, mjtNum* mat2, mjtNum* size2) {
  mjtNum len, cdist;
  mjtNum axis1[3], axis2[3];

  // check bounding spheres (this is called from other functions)
  cdist = mju_dist3(pos1, pos2);
  if (cdist > margin + size1[0] + size2[0]) {
    return 0;
  }

  // depth and normal
  con[0].dist = cdist - size1[0] - size2[0];
  mju_sub3(con[0].frame, pos2, pos1);
  len = mju_normalize3(con[0].frame);

  // if centers are the same, norm = cross-product of z axes
  //  if z axes are parallel, norm = [1;0;0]
  if (len < mjMINVAL) {
    axis1[0] = mat1[2];
    axis1[1] = mat1[5];
    axis1[2] = mat1[8];
    axis2[0] = mat2[2];
    axis2[1] = mat2[5];
    axis2[2] = mat2[8];
    mju_cross(con[0].frame, axis1, axis2);
    mju_normalize3(con[0].frame);
  }

  // position
  mju_scl3(con[0].pos, con[0].frame, size1[0] + con[0].dist/2);
  mju_addTo3(con[0].pos, pos1);

  mju_zero3(con[0].frame+3);
  return 1;
}



// sphere : sphere
int mjc_SphereSphere(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  return  _SphereSphere(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}



// sphere : capsule
int mjc_SphereCapsule(const mjModel* m, const mjData* d,
                      mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum x, axis[3], vec[3];

  // get capsule axis (scaled)
  axis[0] = mat2[2] * size2[1];
  axis[1] = mat2[5] * size2[1];
  axis[2] = mat2[8] * size2[1];

  // find projection, clip to segment
  mju_sub3(vec, pos1, pos2);
  x = mju_dot3(axis, vec) / mju_dot3(axis, axis);
  if (x > 1) {
    x = 1;
  } else if (x < -1) {
    x = -1;
  }

  // find nearest point on segment, do sphere-sphere test
  mju_scl3(vec, axis, x);
  mju_addTo3(vec, pos2);
  return _SphereSphere(con, margin, pos1, mat1, size1, vec, mat2, size2);
}



// capsule : capsule
int mjc_CapsuleCapsule(const mjModel* m, const mjData* d,
                       mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum axis1[3], axis2[3], dif[3], vec1[3], vec2[3];
  mjtNum ma, mb, mc, u, v, det, x1, x2;
  int n1, n2, n3, n4;

  // get capsule axes (scaled) and center difference
  axis1[0] = mat1[2] * size1[1];
  axis1[1] = mat1[5] * size1[1];
  axis1[2] = mat1[8] * size1[1];
  axis2[0] = mat2[2] * size2[1];
  axis2[1] = mat2[5] * size2[1];
  axis2[2] = mat2[8] * size2[1];
  mju_sub3(dif, pos1, pos2);

  // compute matrix coefficients and determinant
  ma =  mju_dot3(axis1, axis1);
  mb = -mju_dot3(axis1, axis2);
  mc =  mju_dot3(axis2, axis2);
  u =  -mju_dot3(axis1, dif);
  v =   mju_dot3(axis2, dif);
  det = ma*mc - mb*mb;

  // general configuration (non-parallel axes)
  if (fabs(det) >= mjMINVAL) {
    // find projections, clip to segments
    x1 = (mc*u - mb*v) / det;
    x2 = (ma*v - mb*u) / det;

    if (x1 > 1) {
      x1 = 1;
      x2 = (v-mb)/mc;
    } else if (x1 < -1) {
      x1 = -1;
      x2 = (v+mb)/mc;
    }
    if (x2 > 1) {
      x2 = 1;
      x1 = (u-mb)/ma;
      if (x1 > 1) {
        x1 = 1;
      } else if (x1 < -1) {
        x1 = -1;
      }
    } else if (x2 < -1) {
      x2 = -1;
      x1 = (u+mb)/ma;
      if (x1 > 1) {
        x1 = 1;
      } else if (x1 < -1) {
        x1 = -1;
      }
    }

    // find nearest points, do sphere-sphere test
    mju_scl3(vec1, axis1, x1);
    mju_addTo3(vec1, pos1);
    mju_scl3(vec2, axis2, x2);
    mju_addTo3(vec2, pos2);

    return _SphereSphere(con, margin, vec1, mat1, size1, vec2, mat2, size2);
  }

  // parallel axes
  else {
    // x1 = 1
    mju_add3(vec1, pos1, axis1);
    x2 = (v - mb) / mc;
    if (x2 > 1) {
      x2 = 1;
    } else if (x2 < -1) {
      x2 = -1;
    }
    mju_scl3(vec2, axis2, x2);
    mju_addTo3(vec2, pos2);
    n1 = _SphereSphere(con, margin, vec1, mat1, size1, vec2, mat2, size2);

    // x1 = -1
    mju_sub3(vec1, pos1, axis1);
    x2 = (v + mb) / mc;
    if (x2 > 1) {
      x2 = 1;
    } else if (x2 < -1) {
      x2 = -1;
    }
    mju_scl3(vec2, axis2, x2);
    mju_addTo3(vec2, pos2);
    n2 = _SphereSphere(con+n1, margin, vec1, mat1, size1, vec2, mat2, size2);

    // return if two contacts already found
    if (n1+n2>=2) {
      return n1+n2;
    }

    // x2 = 1
    mju_add3(vec2, pos2, axis2);
    x1 = (u - mb) / ma;
    if (x1 > 1) {
      x1 = 1;
    } else if (x1 < -1) {
      x1 = -1;
    }
    mju_scl3(vec1, axis1, x1);
    mju_addTo3(vec1, pos1);
    n3 = _SphereSphere(con+n1+n2, margin, vec1, mat1, size1, vec2, mat2, size2);

    // return if two contacts already found
    if (n1+n2+n3>=2) {
      return n1+n2+n3;
    }

    // x2 = -1
    mju_sub3(vec2, pos2, axis2);
    x1 = (u + mb) / ma;
    if (x1 > 1) {
      x1 = 1;
    } else if (x1 < -1) {
      x1 = -1;
    }
    mju_scl3(vec1, axis1, x1);
    mju_addTo3(vec1, pos1);
    n4 = _SphereSphere(con+n1+n2+n3, margin, vec1, mat1, size1, vec2, mat2, size2);

    return n1+n2+n3+n4;
  }
}
