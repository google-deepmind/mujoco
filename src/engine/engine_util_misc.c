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

#include "engine/engine_util_misc.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"

//------------------------------ tendon wrapping ---------------------------------------------------

// check for intersection of two 2D line segments
static mjtByte is_intersect(const mjtNum* p1, const mjtNum* p2,
                            const mjtNum* p3, const mjtNum* p4) {
  mjtNum a, b;

  // compute determinant, check
  mjtNum det = (p4[1]-p3[1])*(p2[0]-p1[0]) - (p4[0]-p3[0])*(p2[1]-p1[1]);
  if (mju_abs(det) < mjMINVAL) {
    return 0;
  }

  // compute intersection point on each line
  a = ((p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])) / det;
  b = ((p2[0]-p1[0])*(p1[1]-p3[1]) - (p2[1]-p1[1])*(p1[0]-p3[0])) / det;

  return ((a >= 0 && a <= 1 && b >= 0 && b <= 1) ? 1 : 0);
}



// curve length along circle
static mjtNum length_circle(const mjtNum* p0, const mjtNum* p1, int ind, mjtNum radius) {
  mjtNum p0n[2] = {p0[0], p0[1]};
  mjtNum p1n[2] = {p1[0], p1[1]};

  // compute angle between 0 and pi
  mju_normalize(p0n, 2);
  mju_normalize(p1n, 2);
  mjtNum angle = mju_acos(mju_dot(p0n, p1n, 2));

  // flip if necessary
  mjtNum cross = p0[1]*p1[0]-p0[0]*p1[1];
  if ((cross > 0 && ind) || (cross < 0 && !ind)) {
    angle = 2*mjPI - angle;
  }

  return radius*angle;
}



// 2D circle wrap
//  input:  pair of 2D endpoints in end[4], optional 2D side point in side[2], radius
//  output: return length of circular wrap or -1
//          pair of 2D points in pnt[4]
static mjtNum wrap_circle(mjtNum pnt[4], const mjtNum end[4], const mjtNum* side, mjtNum radius) {
  mjtNum sqlen0 = end[0]*end[0] + end[1]*end[1];
  mjtNum sqlen1 = end[2]*end[2] + end[3]*end[3];
  mjtNum sqrad = radius*radius;

  // either point inside circle or circle too small: no wrap
  if (sqlen0 < sqrad || sqlen1 < sqrad || radius < mjMINVAL) {
    return -1;
  }

  // points too close: no wrap
  mjtNum dif[2] = {end[2]-end[0], end[3]-end[1]};
  mjtNum dd = dif[0]*dif[0] + dif[1]*dif[1];
  if (dd < mjMINVAL) {
    return -1;
  }

  // find nearest point on line segment to origin: a*dif + d0
  mjtNum a = -(dif[0]*end[0]+dif[1]*end[1])/dd;
  if (a < 0) {
    a = 0;
  } else if (a > 1) {
    a = 1;
  }

  // check for intersection and side
  mjtNum tmp[2] = {a*dif[0] + end[0], a*dif[1] + end[1]};
  if (tmp[0]*tmp[0]+tmp[1]*tmp[1] > sqrad && (!side || mju_dot(side, tmp, 2) >= 0)) {
    return -1;
  }

  mjtNum sqrt0 = mju_sqrt(sqlen0 - sqrad);
  mjtNum sqrt1 = mju_sqrt(sqlen1 - sqrad);

  // construct the two solutions, compute goodness
  mjtNum sol[2][2][2], good[2];
  for (int i=0; i < 2; i++) {
    int sgn = (i == 0 ? 1 : -1);

    sol[i][0][0] = (end[0]*sqrad + sgn*radius*end[1]*sqrt0)/sqlen0;
    sol[i][0][1] = (end[1]*sqrad - sgn*radius*end[0]*sqrt0)/sqlen0;
    sol[i][1][0] = (end[2]*sqrad - sgn*radius*end[3]*sqrt1)/sqlen1;
    sol[i][1][1] = (end[3]*sqrad + sgn*radius*end[2]*sqrt1)/sqlen1;

    // goodness: close to sd, or shorter path
    if (side) {
      mju_add(tmp, sol[i][0], sol[i][1], 2);
      mju_normalize(tmp, 2);
      good[i] = mju_dot(tmp, side, 2);
    } else {
      mju_sub(tmp, sol[i][0], sol[i][1], 2);
      good[i] = -mju_dot(tmp, tmp, 2);
    }

    // penalize for intersection
    if (is_intersect(end, sol[i][0], end+2, sol[i][1])) {
      good[i] = -10000;
    }
  }

  // select the better solution
  int i = (good[0] > good[1] ? 0 : 1);
  pnt[0] = sol[i][0][0];
  pnt[1] = sol[i][0][1];
  pnt[2] = sol[i][1][0];
  pnt[3] = sol[i][1][1];

  // check for intersection
  if (is_intersect(end, pnt, end+2, pnt+2)) {
    return -1;
  }

  // return curve length
  return length_circle(sol[i][0], sol[i][1], i, radius);
}



// 2D inside wrap
//  input: pair of 2D endpoints in end[4], radius
//  output: pair of 2D points in pnt[4]; return 0 if wrap, -1 if no wrap
static mjtNum wrap_inside(mjtNum pnt[4], const mjtNum end[4], mjtNum radius) {
  // algorithm parameters
  const int maxiter = 20;
  const mjtNum zinit = 1 - 1e-7;
  const mjtNum tolerance = 1e-6;

  // constants
  mjtNum len0 = mju_norm(end, 2);
  mjtNum len1 = mju_norm(end+2, 2);
  mjtNum dif[2] = {end[2]-end[0], end[3]-end[1]};
  mjtNum dd = dif[0]*dif[0] + dif[1]*dif[1];

  // either point inside circle or circle too small: no wrap
  if (len0 <= radius || len1 <= radius || radius < mjMINVAL || len0 < mjMINVAL || len1 < mjMINVAL) {
    return -1;
  }

  // segment-circle intersection: no wrap
  if (dd > mjMINVAL) {
    // find nearest point on line segment to origin: d0 + a*dif
    mjtNum a = -(dif[0]*end[0] + dif[1]*end[1]) / dd;

    // in segment
    if (a > 0 && a < 1) {
      mjtNum tmp[2];
      mju_addScl(tmp, end, dif, a, 2);
      if (mju_norm(tmp, 2) <= radius) {
        return -1;
      }
    }
  }

  // prepare default in case of numerical failure: average
  pnt[0] = 0.5*(end[0] + end[2]);
  pnt[1] = 0.5*(end[1] + end[3]);
  mju_normalize(pnt, 2);
  mju_scl(pnt, pnt, radius, 2);
  pnt[2] = pnt[0];
  pnt[3] = pnt[1];

  // compute function parameters: asin(A*z) + asin(B*z) - 2*asin(z) + G = 0
  mjtNum A = radius/len0;
  mjtNum B = radius/len1;
  mjtNum cosG = (len0*len0 + len1*len1 - dd) / (2*len0*len1);
  if (cosG < -1+mjMINVAL) {
    return -1;
  } else if (cosG > 1-mjMINVAL) {
    return 0;
  }
  mjtNum G = mju_acos(cosG);

  // init
  mjtNum z = zinit;
  mjtNum f = mju_asin(A*z) + mju_asin(B*z) - 2*mju_asin(z) + G;

  // make sure init is not on the other side
  if (f > 0) {
    return 0;
  }

  // Newton method
  int iter;
  for (iter=0; iter < maxiter && mju_abs(f) > tolerance; iter++) {
    // derivative
    mjtNum df = A/mju_max(mjMINVAL, mju_sqrt(1-z*z*A*A)) +
                B/mju_max(mjMINVAL, mju_sqrt(1-z*z*B*B)) -
                2/mju_max(mjMINVAL, mju_sqrt(1-z*z));

    // check sign; SHOULD NOT OCCUR
    if (df > -mjMINVAL) {
      return 0;
    }

    // new point
    mjtNum z1 = z - f/df;

    // make sure we are moving to the left; SHOULD NOT OCCUR
    if (z1 > z) {
      return 0;
    }

    // update solution
    z = z1;
    f = mju_asin(A*z) + mju_asin(B*z) - 2*mju_asin(z) + G;

    // exit if positive; SHOULD NOT OCCUR
    if (f > tolerance) {
      return 0;
    }
  }

  // check convergence
  if (iter >= maxiter) {
    return 0;
  }

  // finalize: rotation by ang from vec = a or b, depending on cross(a,b) sign
  mjtNum vec[2];
  mjtNum ang;
  if (end[0]*end[3] - end[1]*end[2] > 0) {
    mju_copy(vec, end, 2);
    ang = mju_asin(z) - mju_asin(A*z);
  } else {
    mju_copy(vec, end+2, 2);
    ang = mju_asin(z) - mju_asin(B*z);
  }
  mju_normalize(vec, 2);
  pnt[0] = radius*(mju_cos(ang)*vec[0] - mju_sin(ang)*vec[1]);
  pnt[1] = radius*(mju_sin(ang)*vec[0] + mju_cos(ang)*vec[1]);
  pnt[2] = pnt[0];
  pnt[3] = pnt[1];

  return 0;
}



// wrap tendons around spheres and cylinders
//  input:  x0, x1: pair of 3D endpoints
//          xpos, xmat, radius: position, orientation and radius of geom
//          type: wrap type (mjtWrap)
//          side: 3D position of sidesite
//  output: return wrap length, -1 if no wrap
//          wpnt: pair of 3D wrap points
mjtNum mju_wrap(mjtNum wpnt[6], const mjtNum x0[3], const mjtNum x1[3],
                const mjtNum xpos[3], const mjtNum xmat[9], mjtNum radius,
                int type, const mjtNum side[3]) {
  // check object type;  SHOULD NOT OCCUR
  if (type != mjWRAP_SPHERE && type != mjWRAP_CYLINDER) {
    mjERROR("unknown wrapping object type %d", type);
  }

  // map sites to wrap object's local frame
  mjtNum tmp[3];
  mju_sub3(tmp, x0, xpos);
  mjtNum p[2][3];
  mju_mulMatTVec3(p[0], xmat, tmp);
  mju_sub3(tmp, x1, xpos);
  mju_mulMatTVec3(p[1], xmat, tmp);

  // too close to origin: return
  if (mju_norm3(p[0]) < mjMINVAL || mju_norm3(p[1]) < mjMINVAL) {
    return -1;
  }

  // construct 2D frame for circle wrap
  mjtNum axis[2][3];
  if (type == mjWRAP_SPHERE) {
    // 1st axis = p0
    mju_copy3(axis[0], p[0]);
    mju_normalize3(axis[0]);

    // normal to p0-0-p1 plane = cross(p0, p1)
    mjtNum normal[3];
    mju_cross(normal, p[0], p[1]);
    mjtNum nrm = mju_normalize3(normal);

    // if (p0, p1) parallel: different normal
    if (nrm < mjMINVAL) {
      // find max component of axis0
      int i = 0;
      if (mju_abs(axis[0][1]) > mju_abs(axis[0][0]) &&
          mju_abs(axis[0][1]) > mju_abs(axis[0][2])) {
        i = 1;
      }
      if (mju_abs(axis[0][2]) > mju_abs(axis[0][0]) &&
          mju_abs(axis[0][2]) > mju_abs(axis[0][1])) {
        i = 2;
      }

      // init second axis: 0 at i; 1 elsewhere
      axis[1][0] = 1;
      axis[1][1] = 1;
      axis[1][2] = 1;
      axis[1][i] = 0;

      // recompute normal
      mju_cross(normal, axis[0], axis[1]);
      mju_normalize3(normal);
    }

    // 2nd axis = cross(normal, p0)
    mju_cross(axis[1], normal, axis[0]);
    mju_normalize3(axis[1]);
  } else {
    // 1st axis = x
    axis[0][0] = 1;
    axis[0][1] = axis[0][2] = 0;

    // 2nd axis = y
    axis[1][1] = 1;
    axis[1][0] = axis[1][2] = 0;
  }

  // project points in 2D frame: p => d
  mjtNum s[3], d[4], sd[2];
  d[0] = mju_dot3(p[0], axis[0]);
  d[1] = mju_dot3(p[0], axis[1]);
  d[2] = mju_dot3(p[1], axis[0]);
  d[3] = mju_dot3(p[1], axis[1]);

  // handle sidesite
  if (side) {
    // side point: apply same projection as x0, x1
    mju_sub3(tmp, side, xpos);
    mju_mulMatTVec3(s, xmat, tmp);

    // side point: project and rescale
    sd[0] = mju_dot3(s, axis[0]);
    sd[1] = mju_dot3(s, axis[1]);
    mju_normalize(sd, 2);
    mju_scl(sd, sd, radius, 2);
  }

  // apply inside wrap
  mjtNum wlen;
  mjtNum pnt[4];
  if (side && mju_norm3(s) < radius) {
    wlen = wrap_inside(pnt, d, radius);
  }

  // apply circle wrap
  else {
    wlen = wrap_circle(pnt, d, (side ? sd : NULL), radius);
  }

  // no wrap: return
  if (wlen < 0) {
    return -1;
  }

  // reconstruct 3D points in local frame: res
  mjtNum res[6];
  for (int i=0; i < 2; i++) {
    // res = axis0*d0 + axis1*d1
    mju_scl3(res+3*i, axis[0], pnt[2*i]);
    mju_scl3(tmp, axis[1], pnt[2*i+1]);
    mju_addTo3(res+3*i, tmp);
  }

  // cylinder: correct along z
  if (type == mjWRAP_CYLINDER) {
    // set vertical coordinates
    mjtNum L0 = mju_sqrt((p[0][0]-res[0])*(p[0][0]-res[0]) + (p[0][1]-res[1])*(p[0][1]-res[1]));
    mjtNum L1 = mju_sqrt((p[1][0]-res[3])*(p[1][0]-res[3]) + (p[1][1]-res[4])*(p[1][1]-res[4]));
    res[2] = p[0][2] + (p[1][2] - p[0][2])*L0 / (L0+wlen+L1);
    res[5] = p[0][2] + (p[1][2] - p[0][2])*(L0+wlen) / (L0+wlen+L1);

    // correct wlen for height
    mjtNum height = mju_abs(res[5] - res[2]);
    wlen = mju_sqrt(wlen*wlen + height*height);
  }

  // map back to global frame: wpnt
  mju_mulMatVec3(wpnt, xmat, res);
  mju_mulMatVec3(wpnt+3, xmat, res+3);
  mju_addTo3(wpnt, xpos);
  mju_addTo3(wpnt+3, xpos);

  return wlen;
}



// all 3 semi-axes of a geom
void mju_geomSemiAxes(const mjModel* m, int geom_id, mjtNum semiaxes[3]) {
  mjtNum* size = m->geom_size + 3*geom_id;
  switch ((mjtGeom) m->geom_type[geom_id]) {
  case mjGEOM_SPHERE:
    semiaxes[0] = size[0];
    semiaxes[1] = size[0];
    semiaxes[2] = size[0];
    break;

  case mjGEOM_CAPSULE:
    semiaxes[0] = size[0];
    semiaxes[1] = size[0];
    semiaxes[2] = size[1] + size[0];
    break;

  case mjGEOM_CYLINDER:
    semiaxes[0] = size[0];
    semiaxes[1] = size[0];
    semiaxes[2] = size[1];
    break;

  default:
    semiaxes[0] = size[0];
    semiaxes[1] = size[1];
    semiaxes[2] = size[2];
  }
}



// ----------------------------- Flex interpolation ------------------------------------------------

mjtNum static inline phi(mjtNum s, int i) {
  if (i == 0) {
    return 1-s;
  } else {
    return s;
  }
}

mjtNum static inline dphi(mjtNum s, int i) {
  if (i == 0) {
    return -1;
  } else {
    return 1;
  }
}

// evaluate the deformation gradient at p using the nodal dof values
void mju_defGradient(mjtNum res[9], const mjtNum p[3], const mjtNum* dof, int order) {
  mjtNum gradient[3];
  mju_zero(res, 9);
  for (int i = 0; i <= order; i++) {
    for (int j = 0; j <= order; j++) {
      for (int k = 0; k <= order; k++) {
        int idx = 4*i + 2*j + k;
        gradient[0] = dphi(p[0], i) *  phi(p[1], j) *  phi(p[2], k);
        gradient[1] =  phi(p[0], i) * dphi(p[1], j) *  phi(p[2], k);
        gradient[2] =  phi(p[0], i) *  phi(p[1], j) * dphi(p[2], k);
        res[0] += dof[3*idx+0] * gradient[0];
        res[1] += dof[3*idx+0] * gradient[1];
        res[2] += dof[3*idx+0] * gradient[2];
        res[3] += dof[3*idx+1] * gradient[0];
        res[4] += dof[3*idx+1] * gradient[1];
        res[5] += dof[3*idx+1] * gradient[2];
        res[6] += dof[3*idx+2] * gradient[0];
        res[7] += dof[3*idx+2] * gradient[1];
        res[8] += dof[3*idx+2] * gradient[2];
      }
    }
  }
}



//------------------------------ actuator models ---------------------------------------------------

// normalized muscle length-gain curve
mjtNum mju_muscleGainLength(mjtNum length, mjtNum lmin, mjtNum lmax) {
  if (lmin <= length && length <= lmax) {
    // mid-ranges (maximum is at 1.0)
    mjtNum a = 0.5*(lmin+1);
    mjtNum b = 0.5*(1+lmax);

    if (length <= a) {
      mjtNum x = (length-lmin) / mjMAX(mjMINVAL, a-lmin);
      return 0.5*x*x;
    } else if (length <= 1) {
      mjtNum x = (1-length) / mjMAX(mjMINVAL, 1-a);
      return 1 - 0.5*x*x;
    } else if (length <= b) {
      mjtNum x = (length-1) / mjMAX(mjMINVAL, b-1);
      return 1 - 0.5*x*x;
    } else {
      mjtNum x = (lmax-length) / mjMAX(mjMINVAL, lmax-b);
      return 0.5*x*x;
    }
  }

  return 0.0;
}



// muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
                      mjtNum acc0, const mjtNum prm[9]) {
  // unpack parameters
  mjtNum range[2] = {prm[0], prm[1]};
  mjtNum force    = prm[2];
  mjtNum scale    = prm[3];
  mjtNum lmin     = prm[4];
  mjtNum lmax     = prm[5];
  mjtNum vmax     = prm[6];
  mjtNum fvmax    = prm[8];

  // scale force if negative
  if (force < 0) {
    force = scale / mjMAX(mjMINVAL, acc0);
  }

  // optimum length
  mjtNum L0 = (lengthrange[1]-lengthrange[0]) / mjMAX(mjMINVAL, range[1]-range[0]);

  // normalized length and velocity
  mjtNum L = range[0] + (len-lengthrange[0]) / mjMAX(mjMINVAL, L0);
  mjtNum V = vel / mjMAX(mjMINVAL, L0*vmax);

  // length curve
  mjtNum FL = mju_muscleGainLength(L, lmin, lmax);

  // velocity curve
  mjtNum FV;
  mjtNum y = fvmax-1;
  if (V <= -1) {
    FV = 0;
  } else if (V <= 0) {
    FV = (V+1)*(V+1);
  } else if (V <= y) {
    FV = fvmax - (y-V)*(y-V) / mjMAX(mjMINVAL, y);
  } else {
    FV = fvmax;
  }

  // compute FVL and scale, make it negative
  return -force*FL*FV;
}



// muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
                      mjtNum acc0, const mjtNum prm[9]) {
  // unpack parameters
  mjtNum range[2] = {prm[0], prm[1]};
  mjtNum force    = prm[2];
  mjtNum scale    = prm[3];
  mjtNum lmax     = prm[5];
  mjtNum fpmax    = prm[7];

  // scale force if negative
  if (force < 0) {
    force = scale / mjMAX(mjMINVAL, acc0);
  }

  // optimum length
  mjtNum L0 = (lengthrange[1]-lengthrange[0]) / mjMAX(mjMINVAL, range[1]-range[0]);

  // normalized length
  mjtNum L = range[0] + (len-lengthrange[0]) / mjMAX(mjMINVAL, L0);

  // half-quadratic to (L0+lmax)/2, linear beyond
  mjtNum b = 0.5*(1+lmax);
  if (L <= 1) {
    return 0;
  } else if (L <= b) {
    mjtNum x = (L-1) / mjMAX(mjMINVAL, b-1);
    return -force*fpmax*0.5*x*x;
  } else {
    mjtNum x = (L-b) / mjMAX(mjMINVAL, b-1);
    return -force*fpmax*(0.5 + x);
  }
}



// muscle time constant with optional smoothing
mjtNum mju_muscleDynamicsTimescale(mjtNum dctrl, mjtNum tau_act, mjtNum tau_deact,
                                   mjtNum smoothing_width) {
  mjtNum tau;

  // hard switching
  if (smoothing_width < mjMINVAL) {
    tau = dctrl > 0 ? tau_act : tau_deact;
  }

  // smooth switching
  else {
    // scale by width, center around 0.5 midpoint, rescale to bounds
    tau = tau_deact + (tau_act-tau_deact)*mju_sigmoid(dctrl/smoothing_width + 0.5);
  }
  return tau;
}



// muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width)
mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[3]) {
  // clamp control
  mjtNum ctrlclamp = mju_clip(ctrl, 0, 1);

  // clamp activation
  mjtNum actclamp = mju_clip(act, 0, 1);

  // compute timescales as in Millard et al. (2013) https://doi.org/10.1115/1.4023390
  mjtNum tau_act = prm[0] * (0.5 + 1.5*actclamp);    // activation timescale
  mjtNum tau_deact = prm[1] / (0.5 + 1.5*actclamp);  // deactivation timescale
  mjtNum smoothing_width = prm[2];                   // width of smoothing sigmoid
  mjtNum dctrl = ctrlclamp - act;                    // excess excitation

  mjtNum tau = mju_muscleDynamicsTimescale(dctrl, tau_act, tau_deact, smoothing_width);

  // filter output
  return dctrl / mjMAX(mjMINVAL, tau);
}



//---------------------------------------- Base64 --------------------------------------------------

// decoding function for Base64
static uint32_t _decode(char ch) {
  if (ch >= 'A' && ch <= 'Z') {
    return ch - 'A';
  }

  if (ch >= 'a' && ch <= 'z') {
    return (ch - 'a') + 26;
  }

  if (ch >= '0' && ch <= '9') {
    return (ch - '0') + 52;
  }

  if (ch == '+') {
    return 62;
  }

  if (ch == '/') {
    return 63;
  }

  return 0;
}



// encode data as Base64 into buf (including padding and null char)
// returns number of chars written in buf: 4 * [(ndata + 2) / 3] + 1
size_t mju_encodeBase64(char* buf, const uint8_t* data, size_t ndata) {
  static const char *table =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  int i = 0, j = 0;

  // loop over 24 bit chunks
  while (i + 3 <= ndata) {
    // take next 24 bit chunk (3 bytes)
    uint32_t byte_1 = data[i++];
    uint32_t byte_2 = data[i++];
    uint32_t byte_3 = data[i++];

    // merge bytes into one 32 bit int
    uint32_t k = (byte_1 << 16) | (byte_2 << 8) | byte_3;

    // encode 6 bit chucks into four chars
    buf[j++] = table[(k >> 18) & 63];
    buf[j++] = table[(k >> 12) & 63];
    buf[j++] = table[(k >>  6) & 63];
    buf[j++] = table[(k >>  0) & 63];
  }

  // one byte left
  if (i + 1 == ndata) {
    uint32_t byte_1 = data[i];
    uint32_t k = byte_1 << 16;
    buf[j++] = table[(k >> 18) & 63];
    buf[j++] = table[(k >> 12) & 63];
    buf[j++] = '=';  // padding
    buf[j++] = '=';  // padding
  }

  // two bytes left
  if (i + 2 == ndata) {
    uint32_t byte_1 = data[i++];
    uint32_t byte_2 = data[i];

    uint32_t k = (byte_1 << 16) + (byte_2 << 8);

    buf[j++] = table[(k >> 18) & 63];
    buf[j++] = table[(k >> 12) & 63];
    buf[j++] = table[(k >>  6) & 63];
    buf[j++] = '=';  // padding
  }

  buf[j] = '\0';
  return 4 * ((ndata + 2) / 3) + 1;
}



// return size in decoded bytes if s is a valid Base64 encoding
// return 0 if s is empty or invalid Base64 encoding
size_t mju_isValidBase64(const char* s) {
  size_t i = 0;
  int pad = 0;  // 0, 1, or 2 zero padding at the end of s

  // validate chars
  for (; s[i] && s[i] != '='; i++) {
    if (!isalnum(s[i]) && s[i] != '/' && s[i] != '+') {
      return 0;
    }
  }

  // padding at end
  if (s[i] == '=') {
    if (!s[i + 1]) {
      pad = 1;  // one '=' padding at end
    } else if (s[i + 1] == '=' && !s[i + 2]) {
      pad = 2;  // two '=' padding at end
    } else {
      return 0;
    }
  }

  // strlen(s) must be a multiple of 4
  int len = i + pad;
  return len % 4 ? 0 : 3 * (len / 4) - pad;
}



// decode valid Base64 in string s into buf, undefined behavior if s is not valid Base64
// returns number of bytes decoded (upper limit of 3 * (strlen(s) / 4))
size_t mju_decodeBase64(uint8_t* buf, const char* s) {
  size_t i = 0, j = 0;

  // loop over 24 bit chunks
  while (s[i] != '\0') {
    // take next 24 bit chuck (4 chars; 6 bits each)
    uint32_t char_1 = _decode(s[i++]);
    uint32_t char_2 = _decode(s[i++]);
    uint32_t char_3 = _decode(s[i++]);
    uint32_t char_4 = _decode(s[i++]);

    // merge into 32 bit int
    uint32_t k = (char_1 << 18) | (char_2 << 12) | (char_3 << 6) | char_4;


    // write up to three bytes (exclude padding at end)
    buf[j++] = (k >> 16) & 0xFF;
    if (s[i - 2] != '=') {
      buf[j++] = (k >> 8) & 0xFF;
    }
    if (s[i - 1] != '=') {
      buf[j++] =  k & 0xFF;
    }
  }
  return j;
}



//------------------------------ miscellaneous -----------------------------------------------------

// convert contact force to pyramid representation
// the pyramid frame is: V0_i = N + mu_i*T_i
//                       V1_i = N - mu_i*T_i
void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force, const mjtNum* mu, int dim) {
  mjtNum a = force[0]/(dim-1), b;

  // arbitrary redundancy resolution:
  //  pyramid0_i + pyramid1_i = force_normal/(dim-1) = a
  //  pyramid0_i - pyramid1_i = force_tangent_i/mu_i = b
  for (int i=0; i < dim-1; i++) {
    b = mju_min(a, force[i+1]/mu[i]);
    pyramid[2*i] = 0.5*(a+b);
    pyramid[2*i+1] = 0.5*(a-b);
  }
}



// convert pyramid representation to contact force
void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid, const mjtNum* mu, int dim) {
  // special handling of frictionless contacts
  if (dim == 1) {
    force[0] = pyramid[0];
    return;
  }

  // force_normal = sum(pyramid0_i + pyramid1_i)
  force[0] = 0;
  for (int i=0; i < 2*(dim-1); i++) {
    force[0] += pyramid[i];
  }

  // force_tangent_i = (pyramid0_i - pyramid1_i) * mu_i
  for (int i=0; i < dim-1; i++) {
    force[i+1] = (pyramid[2*i] - pyramid[2*i+1]) * mu[i];
  }
}



// integrate spring-damper analytically, return pos(t)
mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum k, mjtNum b, mjtNum t) {
  mjtNum det, c1, c2, r1, r2, w;

  // determinant of characteristic equation
  det = b*b - 4*k;

  // overdamping
  //  pos(t) = c1*exp(r1*t) + c2*exp(r2*t);  r12 = (-b +- sqrt(det))/2
  if (det > mjMINVAL) {
    // compute w = sqrt(det)/2
    w = mju_sqrt(det)/2;

    // compute r1,r2
    r1 = -b/2 + w;
    r2 = -b/2 - w;

    // compute coefficients
    c1 = (pos0*r2-vel0) / (r2-r1);
    c2 = (pos0*r1-vel0) / (r1-r2);

    // evaluate result
    return c1*mju_exp(r1*t) + c2*mju_exp(r2*t);
  }

  // critical damping
  //  pos(t) = exp(-b*t/2) * (c1 + c2*t)
  else if (det <= mjMINVAL && det >= -mjMINVAL) {
    // compute coefficients
    c1 = pos0;
    c2 = vel0 + b*c1/2;

    // evaluate result
    return mju_exp(-b*t/2) * (c1 + c2*t);
  }

  // underdamping
  //  pos(t) = exp(-b*t/2) * (c1*cos(w*t) + c2*sin(w*t));  w = sqrt(abs(det))/2
  else {
    // compute w
    w = mju_sqrt(mju_abs(det))/2;

    // compute coefficients
    c1 = pos0;
    c2 = (vel0 + b*c1/2)/w;

    // evaluate result
    return mju_exp(-b*t/2) * (c1*mju_cos(w*t) + c2*mju_sin(w*t));
  }
}



// return 1 if point is outside box given by pos, mat, size * inflate
// return -1 if point is inside box given by pos, mat, size / inflate
// return 0 if point is between the inflated and deflated boxes
int mju_outsideBox(const mjtNum point[3], const mjtNum pos[3], const mjtNum mat[9],
                   const mjtNum size[3], mjtNum inflate) {
  // check inflation coefficient
  if (inflate < 1) {
    mjERROR("inflation coefficient must be >= 1")
  }

  // vector from pos to point, projected to box frame
  mjtNum vec[3] = {point[0]-pos[0], point[1]-pos[1], point[2]-pos[2]};
  mju_mulMatTVec3(vec, mat, vec);

  // big: inflated box
  mjtNum big[3] = {size[0], size[1], size[2]};
  if (inflate > 1) {
    mju_scl3(big, big, inflate);
  }

  // check if outside big box
  if (vec[0] > big[0] || vec[0] < -big[0] ||
      vec[1] > big[1] || vec[1] < -big[1] ||
      vec[2] > big[2] || vec[2] < -big[2]) {
    return 1;
  }

  // quick return if no inflation
  if (inflate == 1) {
    return -1;
  }

  // check if inside small (deflated) box
  mjtNum small[3] = {size[0]/inflate, size[1]/inflate, size[2]/inflate};
  if (vec[0] < small[0] && vec[0] > -small[0] &&
      vec[1] < small[1] && vec[1] > -small[1] &&
      vec[2] < small[2] && vec[2] > -small[2]) {
    return -1;
  }

  // within margin between small and big box
  return 0;
}



// print matrix to screen
void mju_printMat(const mjtNum* mat, int nr, int nc) {
  for (int r=0; r < nr; r++) {
    for (int c=0; c < nc; c++) {
      printf("%.8f ", mat[r*nc+c]);
    }
    printf("\n");
  }
  printf("\n");
}



// print sparse matrix to screen
void mju_printMatSparse(const mjtNum* mat, int nr,
                        const int* rownnz, const int* rowadr,
                        const int* colind) {
  for (int r=0; r < nr; r++) {
    for (int adr=rowadr[r]; adr < rowadr[r]+rownnz[r]; adr++) {
      printf("(%d %d): %9.6f  ", r, colind[adr], mat[adr]);
    }
    printf("\n");
  }
  printf("\n");
}



// min function, avoid re-evaluation
mjtNum mju_min(mjtNum a, mjtNum b) {
  if (a <= b) {
    return a;
  } else {
    return b;
  }
}



// max function, avoid re-evaluation
mjtNum mju_max(mjtNum a, mjtNum b) {
  if (a >= b) {
    return a;
  } else {
    return b;
  }
}



// clip x to the range [min, max]
mjtNum mju_clip(mjtNum x, mjtNum min, mjtNum max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  } else {
    return x;
  }
}



// sign function
mjtNum mju_sign(mjtNum x) {
  if (x < 0) {
    return -1;
  } else if (x > 0) {
    return 1;
  } else {
    return 0;
  }
}



// round to nearest integer
int mju_round(mjtNum x) {
  mjtNum lower = floor(x);
  mjtNum upper = ceil(x);

  if (x-lower < upper-x) {
    return (int)lower;
  } else {
    return (int)upper;
  }
}



// convert type id to type name
const char* mju_type2Str(int type) {
  switch ((mjtObj) type) {
  case mjOBJ_BODY:
    return "body";

  case mjOBJ_XBODY:
    return "xbody";

  case mjOBJ_JOINT:
    return "joint";

  case mjOBJ_DOF:
    return "dof";

  case mjOBJ_GEOM:
    return "geom";

  case mjOBJ_SITE:
    return "site";

  case mjOBJ_CAMERA:
    return "camera";

  case mjOBJ_LIGHT:
    return "light";

  case mjOBJ_FLEX:
    return "flex";

  case mjOBJ_MESH:
    return "mesh";

  case mjOBJ_SKIN:
    return "skin";

  case mjOBJ_HFIELD:
    return "hfield";

  case mjOBJ_TEXTURE:
    return "texture";

  case mjOBJ_MATERIAL:
    return "material";

  case mjOBJ_PAIR:
    return "pair";

  case mjOBJ_EXCLUDE:
    return "exclude";

  case mjOBJ_EQUALITY:
    return "equality";

  case mjOBJ_TENDON:
    return "tendon";

  case mjOBJ_ACTUATOR:
    return "actuator";

  case mjOBJ_SENSOR:
    return "sensor";

  case mjOBJ_NUMERIC:
    return "numeric";

  case mjOBJ_TEXT:
    return "text";

  case mjOBJ_TUPLE:
    return "tuple";

  case mjOBJ_KEY:
    return "key";

  case mjOBJ_PLUGIN:
    return "plugin";

  case mjOBJ_FRAME:
    return "frame";

  default:
    return 0;
  }
}



// convert type id to type name
int mju_str2Type(const char* str) {
  if (!strcmp(str, "body")) {
    return mjOBJ_BODY;
  }

  else if (!strcmp(str, "xbody")) {
    return mjOBJ_XBODY;
  }

  else if (!strcmp(str, "joint")) {
    return mjOBJ_JOINT;
  }

  else if (!strcmp(str, "dof")) {
    return mjOBJ_DOF;
  }

  else if (!strcmp(str, "geom")) {
    return mjOBJ_GEOM;
  }

  else if (!strcmp(str, "site")) {
    return mjOBJ_SITE;
  }

  else if (!strcmp(str, "camera")) {
    return mjOBJ_CAMERA;
  }

  else if (!strcmp(str, "light")) {
    return mjOBJ_LIGHT;
  }

  else if (!strcmp(str, "flex")) {
    return mjOBJ_FLEX;
  }

  else if (!strcmp(str, "mesh")) {
    return mjOBJ_MESH;
  }

  else if (!strcmp(str, "skin")) {
    return mjOBJ_SKIN;
  }

  else if (!strcmp(str, "hfield")) {
    return mjOBJ_HFIELD;
  }

  else if (!strcmp(str, "texture")) {
    return mjOBJ_TEXTURE;
  }

  else if (!strcmp(str, "material")) {
    return mjOBJ_MATERIAL;
  }

  else if (!strcmp(str, "pair")) {
    return mjOBJ_PAIR;
  }

  else if (!strcmp(str, "exclude")) {
    return mjOBJ_EXCLUDE;
  }

  else if (!strcmp(str, "equality")) {
    return mjOBJ_EQUALITY;
  }

  else if (!strcmp(str, "tendon")) {
    return mjOBJ_TENDON;
  }

  else if (!strcmp(str, "actuator")) {
    return mjOBJ_ACTUATOR;
  }

  else if (!strcmp(str, "sensor")) {
    return mjOBJ_SENSOR;
  }

  else if (!strcmp(str, "numeric")) {
    return mjOBJ_NUMERIC;
  }

  else if (!strcmp(str, "text")) {
    return mjOBJ_TEXT;
  }

  else if (!strcmp(str, "tuple")) {
    return mjOBJ_TUPLE;
  }

  else if (!strcmp(str, "key")) {
    return mjOBJ_KEY;
  }

  else if (!strcmp(str, "plugin")) {
    return mjOBJ_PLUGIN;
  }

  else {
    return mjOBJ_UNKNOWN;
  }
}



// return human readable number of bytes using standard letter suffix
const char* mju_writeNumBytes(size_t nbytes) {
  int i;
  static mjTHREADLOCAL char message[20];
  static const char suffix[] = " KMGTPE";
  for (i=0; i < 6; i++) {
    const size_t bits = (size_t)(1) << (10*(6-i));
    if (nbytes >= bits && !(nbytes & (bits - 1))) {
      break;
    }
  }
  if (i < 6) {
    mjSNPRINTF(message, "%zu%c", nbytes >> (10*(6-i)), suffix[6-i]);
  } else {
    mjSNPRINTF(message, "%zu", nbytes >> (10*(6-i)));
  }
  return message;
}



// warning text
const char* mju_warningText(int warning, size_t info) {
  static mjTHREADLOCAL char str[1000];

  switch ((mjtWarning) warning) {
  case mjWARN_INERTIA:
    mjSNPRINTF(str, "Inertia matrix is too close to singular at DOF %zu. Check model.", info);
    break;

  case mjWARN_CONTACTFULL:
    mjSNPRINTF(str,
               "Too many contacts. Either the arena memory is full, or nconmax is specified and is "
               "exceeded. Increase arena memory allocation, or increase/remove nconmax. "
               "(ncon = %zu)", info);
    break;

  case mjWARN_CNSTRFULL:
    mjSNPRINTF(str,
               "Insufficient arena memory for the number of constraints generated. "
               "Increase arena memory allocation above %s bytes.", mju_writeNumBytes(info));
    break;

  case mjWARN_VGEOMFULL:
    mjSNPRINTF(str, "Pre-allocated visual geom buffer is full. Increase maxgeom above %zu.", info);
    break;

  case mjWARN_BADQPOS:
    mjSNPRINTF(str, "Nan, Inf or huge value in QPOS at DOF %zu. The simulation is unstable.", info);
    break;

  case mjWARN_BADQVEL:
    mjSNPRINTF(str, "Nan, Inf or huge value in QVEL at DOF %zu. The simulation is unstable.", info);
    break;

  case mjWARN_BADQACC:
    mjSNPRINTF(str, "Nan, Inf or huge value in QACC at DOF %zu. The simulation is unstable.", info);
    break;

  case mjWARN_BADCTRL:
    mjSNPRINTF(str, "Nan, Inf or huge value in CTRL at ACTUATOR %zu. The simulation is unstable.",
               info);
    break;

  default:
    mjSNPRINTF(str, "Unknown warning type %d.", warning);
  }

  return str;
}



// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
int mju_isBad(mjtNum x) {
  return (x != x || x > mjMAXVAL || x < -mjMAXVAL);
}



// return 1 if all elements are 0
int mju_isZero(mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    if (vec[i] != 0) {
      return 0;
    }
  }

  return 1;
}



// set integer vector to 0
void mju_zeroInt(int* res, int n) {
  memset(res, 0, n*sizeof(int));
}


void mju_zeroSizeT(size_t* res, size_t n) {
  memset(res, 0, n*sizeof(size_t));
}


// copy int vector vec into res
void mju_copyInt(int* res, const int* vec, int n) {
  memcpy(res, vec, n*sizeof(int));
}



// standard normal random number generator (optional second number)
mjtNum mju_standardNormal(mjtNum* num2) {
  const mjtNum scale = 2.0/((mjtNum)RAND_MAX);
  mjtNum x1, x2, w;

  do {
    x1 = scale * (mjtNum)rand() - 1.0;
    x2 = scale * (mjtNum)rand() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w >= 1.0 || w == 0);

  w = mju_sqrt((-2.0 * mju_log(w)) / w);
  if (num2) {
    *num2 = x2 * w;
  }

  return (x1 * w);
}



// convert from float to mjtNum
void mju_f2n(mjtNum* res, const float* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (mjtNum) vec[i];
  }
}



// convert from mjtNum to float
void mju_n2f(float* res, const mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (float) vec[i];
  }
}


// convert from double to mjtNum
void mju_d2n(mjtNum* res, const double* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (mjtNum) vec[i];
  }
}



// convert from mjtNum to double
void mju_n2d(double* res, const mjtNum* vec, int n) {
  for (int i=0; i < n; i++) {
    res[i] = (double) vec[i];
  }
}



// gather
void mju_gather(mjtNum* restrict res, const mjtNum* restrict vec, const int* restrict ind, int n) {
  for (int i=0; i < n; i++) {
    res[i] = vec[ind[i]];
  }
}



// scatter
void mju_scatter(mjtNum* restrict res, const mjtNum* restrict vec, const int* restrict ind, int n) {
  for (int i=0; i < n; i++) {
    res[ind[i]] = vec[i];
  }
}



// gather integers
void mju_gatherInt(int* restrict res, const int* restrict vec, const int* restrict ind, int n) {
  for (int i=0; i < n; i++) {
    res[i] = vec[ind[i]];
  }
}



// scatter integers
void mju_scatterInt(int* restrict res, const int* restrict vec, const int* restrict ind, int n) {
  for (int i=0; i < n; i++) {
    res[ind[i]] = vec[i];
  }
}



// insertion sort, increasing order
void mju_insertionSort(mjtNum* list, int n) {
  for (int i=1; i < n; i++) {
    mjtNum x = list[i];
    int j = i-1;
    while (j >= 0 && list[j] > x) {
      list[j+1] = list[j];
      j--;
    }
    list[j+1] = x;
  }
}



// integer insertion sort, increasing order
void mju_insertionSortInt(int* list, int n) {
  for (int i=1; i < n; i++) {
    int x = list[i];
    int j = i-1;
    while (j >= 0 && list[j] > x) {
      list[j+1] = list[j];
      j--;
    }
    list[j+1] = x;
  }
}



// Halton sequence
mjtNum mju_Halton(int index, int base) {
  int n0 = index;
  mjtNum b = (mjtNum)base;
  mjtNum f = 1/b, hn = 0;

  while (n0 > 0) {
    int n1 = n0/base;
    int r = n0 - n1*base;
    hn += f*r;
    f /= b;
    n0 = n1;
  }

  return hn;
}



// Call strncpy, then set dst[n-1] = 0.
char* mju_strncpy(char *dst, const char *src, int n) {
  if (dst && src && n > 0) {
    strncpy(dst, src, n);
    dst[n-1] = 0;
  }

  return dst;
}



// sigmoid function over 0<=x<=1 using quintic polynomial
mjtNum mju_sigmoid(mjtNum x) {
  // fast return
  if (x <= 0) {
    return 0;
  }
  if (x >= 1) {
    return 1;
  }

  // sigmoid: f(x) = 6*x^5 - 15*x^4 + 10*x^3
  // solution of f(0) = f'(0) = f''(0) = 0, f(1) = 1, f'(1) = f''(1) = 0
  return x*x*x * (3*x * (2*x - 5) + 10);
}
