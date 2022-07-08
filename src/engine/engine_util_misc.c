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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"

//------------------------------ tendon wrapping ---------------------------------------------------

// check for intersection of two 2D line segmetns
static mjtByte is_intersect(const mjtNum* p1, const mjtNum* p2,
                            const mjtNum* p3, const mjtNum* p4) {
  mjtNum a, b;

  // compute determinant, check
  mjtNum det = (p4[1]-p3[1])*(p2[0]-p1[0]) - (p4[0]-p3[0])*(p2[1]-p1[1]);
  if (fabs(det)<mjMINVAL) {
    return 0;
  }

  // compute intersection point on each line
  a = ((p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])) / det;
  b = ((p2[0]-p1[0])*(p1[1]-p3[1]) - (p2[1]-p1[1])*(p1[0]-p3[0])) / det;

  return ((a>=0 && a<=1 && b>=0 && b<=1) ? 1 : 0);
}



// curve length along circle
static mjtNum length_circle(const mjtNum* p0, const mjtNum* p1, int ind, mjtNum rad) {
  mjtNum angle, cross;
  mjtNum p0n[2] = {p0[0], p0[1]};
  mjtNum p1n[2] = {p1[0], p1[1]};

  // compute angle between 0 and pi
  mju_normalize(p0n, 2);
  mju_normalize(p1n, 2);
  angle = mju_acos(mju_dot(p0n, p1n, 2));

  // flip if necessary
  cross = p0[1]*p1[0]-p0[0]*p1[1];
  if ((cross>0 && ind) || (cross<0 && !ind)) {
    angle = 2*mjPI - angle;
  }

  return rad*angle;
}



// 2D circle wrap
//  input: pair of 2D points in d[4], optional 2D side point in sd[2], radius
//  output: pair of 2D points in pnt[4], length of circular wrap or -1
static mjtNum wrap_circle(mjtNum* pnt, const mjtNum* d, const mjtNum* sd, mjtNum rad) {
  mjtNum sqlen0 = d[0]*d[0]+d[1]*d[1];
  mjtNum sqlen1 = d[2]*d[2]+d[3]*d[3];
  mjtNum sqrad = rad*rad;
  mjtNum dif[2] = {d[2]-d[0], d[3]-d[1]};
  mjtNum a, tmp[2], dd, sqrt0, sqrt1;
  mjtNum sol[2][2][2], good[2], wlen;
  int i, sgn;

  // either point inside circle or circle too small: no wrap
  if (sqlen0<sqrad || sqlen1<sqrad || rad<mjMINVAL) {
    return -1;
  }

  // points too close: no wrap
  dd = dif[0]*dif[0] + dif[1]*dif[1];
  if (dd<mjMINVAL) {
    return -1;
  }

  // find nearest point on line segment to origin: a*dif + d0
  a = -(dif[0]*d[0]+dif[1]*d[1])/dd;
  if (a<0) {
    a = 0;
  } else if (a>1) {
    a = 1;
  }
  tmp[0] = a*dif[0] + d[0];
  tmp[1] = a*dif[1] + d[1];

  // check for intersection and side
  if (tmp[0]*tmp[0]+tmp[1]*tmp[1]>sqrad && (!sd || mju_dot(sd, tmp, 2)>=0)) {
    return -1;
  }

  // construct the two solutions, compute goodness
  for (int i=0; i<2; i++) {
    sqrt0 = mju_sqrt(sqlen0 - sqrad);
    sqrt1 = mju_sqrt(sqlen1 - sqrad);

    sgn = (i==0 ? 1 : -1);

    sol[i][0][0] = (d[0]*sqrad + sgn*rad*d[1]*sqrt0)/sqlen0;
    sol[i][0][1] = (d[1]*sqrad - sgn*rad*d[0]*sqrt0)/sqlen0;
    sol[i][1][0] = (d[2]*sqrad - sgn*rad*d[3]*sqrt1)/sqlen1;
    sol[i][1][1] = (d[3]*sqrad + sgn*rad*d[2]*sqrt1)/sqlen1;

    // goodness: close to sd, or shorter path
    if (sd) {
      mju_add(tmp, sol[i][0], sol[i][1], 2);
      mju_normalize(tmp, 2);
      good[i] = mju_dot(tmp, sd, 2);
    } else {
      mju_sub(tmp, sol[i][0], sol[i][1], 2);
      good[i] = -mju_dot(tmp, tmp, 2);
    }

    // penalize for intersection
    if (is_intersect(d, sol[i][0], d+2, sol[i][1])) {
      good[i] = -10000;
    }
  }

  // select the better solution
  i = (good[0]>good[1] ? 0 : 1);
  pnt[0] = sol[i][0][0];
  pnt[1] = sol[i][0][1];
  pnt[2] = sol[i][1][0];
  pnt[3] = sol[i][1][1];

  // check for intersection
  if (is_intersect(d, pnt, d+2, pnt+2)) {
    return -1;
  }

  // compute curve length
  wlen = length_circle(sol[i][0], sol[i][1], i, rad);
  return wlen;
}



// 2D inside wrap
//  input: pair of 2D points in d[4], radius
//  output: pair of 2D points in pnt[4]; return 0 if wrap, -1 if no wrap
static mjtNum wrap_inside(mjtNum* pnt, const mjtNum* d, mjtNum rad) {
  // algorithm paramters
  const int maxiter = 20;
  const mjtNum zinit = 1 - 1e-7;
  const mjtNum tolerance = 1e-6;

  // constants
  mjtNum len0 = mju_norm(d, 2);
  mjtNum len1 = mju_norm(d+2, 2);
  mjtNum dif[2] = {d[2]-d[0], d[3]-d[1]};
  mjtNum dd = dif[0]*dif[0] + dif[1]*dif[1];

  // either point inside circle or circle too small: no wrap
  if (len0<=rad || len1<=rad || rad<mjMINVAL || len0<mjMINVAL || len1<mjMINVAL) {
    return -1;
  }

  // segment-circle intersection: no wrap
  if (dd>mjMINVAL) {
    // find nearest point on line segment to origin: d0 + a*dif
    mjtNum a = -(dif[0]*d[0]+dif[1]*d[1])/dd;

    // in segment
    if (a>0 && a<1) {
      mjtNum tmp[2];
      mju_addScl(tmp, d, dif, a, 2);
      if (mju_norm(tmp, 2)<=rad) {
        return -1;
      }
    }
  }

  // prepare default in case of numerical failure: average
  pnt[0] = 0.5*(d[0] + d[2]);
  pnt[1] = 0.5*(d[1] + d[3]);
  mju_normalize(pnt, 2);
  mju_scl(pnt, pnt, rad, 2);
  pnt[2] = pnt[0];
  pnt[3] = pnt[1];

  // compute function parameters: asin(A*z) + asin(B*z) - 2*asin(z) + G = 0
  mjtNum A = rad/len0;
  mjtNum B = rad/len1;
  mjtNum cosG = (len0*len0 + len1*len1 - dd) / (2*len0*len1);
  if (cosG<-1+mjMINVAL) {
    return -1;
  } else if (cosG>1-mjMINVAL) {
    return 0;
  }
  mjtNum G = mju_acos(cosG);

  // init
  mjtNum z = zinit;
  mjtNum f = mju_asin(A*z) + mju_asin(B*z) - 2*mju_asin(z) + G;

  // make sure init is not on the other side
  if (f>0) {
    return 0;
  }

  // Newton method
  int iter;
  for (iter=0; iter<maxiter && mju_abs(f)>tolerance; iter++) {
    // derivative
    mjtNum df = A/mju_max(mjMINVAL, mju_sqrt(1-z*z*A*A)) +
                B/mju_max(mjMINVAL, mju_sqrt(1-z*z*B*B)) -
                2/mju_max(mjMINVAL, mju_sqrt(1-z*z));

    // check sign; SHOULD NOT OCCUR
    if (df>-mjMINVAL) {
      return 0;
    }

    // new point
    mjtNum z1 = z - f/df;

    // make sure we are moving to the left; SHOULD NOT OCCUR
    if (z1>z) {
      return 0;
    }

    // update solution
    z = z1;
    f = mju_asin(A*z) + mju_asin(B*z) - 2*mju_asin(z) + G;

    // exit if positive; SHOULD NOT OCCUR
    if (f>tolerance) {
      return 0;
    }
  }

  // check convergence
  if (iter>=maxiter) {
    return 0;
  }

  // finalize: rotation by ang from vec = a or b, depending on cross(a,b) sign
  mjtNum vec[2];
  mjtNum ang;
  if (d[0]*d[3] - d[1]*d[2] > 0) {
    mju_copy(vec, d, 2);
    ang = mju_asin(z) - mju_asin(A*z);
  } else {
    mju_copy(vec, d+2, 2);
    ang = mju_asin(z) - mju_asin(B*z);
  }
  mju_normalize(vec, 2);
  pnt[0] = rad*(mju_cos(ang)*vec[0] - mju_sin(ang)*vec[1]);
  pnt[1] = rad*(mju_sin(ang)*vec[0] + mju_cos(ang)*vec[1]);
  pnt[2] = pnt[0];
  pnt[3] = pnt[1];

  return 0;
}



// wrap tendons around spheres and cylinders
mjtNum mju_wrap(mjtNum* wpnt, const mjtNum* x0, const mjtNum* x1,
                const mjtNum* xpos, const mjtNum* xmat, const mjtNum* size,
                int type, const mjtNum* side) {
  mjtNum tmp[3], normal[3], axis[2][3], p[2][3], s[3], d[4], sd[2], pnt[4];
  mjtNum res[6], wlen, height;
  mjtNum L0, L1;

  // check object type;  SHOULD NOT OCCUR
  if (type!=mjWRAP_SPHERE && type!=mjWRAP_CYLINDER) {
    mju_error_i("mju_wrap: unknown wrapping object type %d", type);
  }

  // map sites to wrap object's local frame
  mju_sub3(tmp, x0, xpos);
  mju_mulMatTVec(p[0], xmat, tmp, 3, 3);
  mju_sub3(tmp, x1, xpos);
  mju_mulMatTVec(p[1], xmat, tmp, 3, 3);

  // too close to origin: return
  if (mju_norm3(p[0])<mjMINVAL || mju_norm3(p[1])<mjMINVAL) {
    return -1;
  }

  // construct 2D frame for circle wrap
  if (type==mjWRAP_SPHERE) {
    // 1st axis = p0
    mju_copy3(axis[0], p[0]);
    mju_normalize3(axis[0]);

    // normal to p0-0-p1 plane = cross(p0, p1)
    mju_cross(normal, p[0], p[1]);
    mjtNum nrm = mju_normalize3(normal);

    // if (p0, p1) parallel: different normal
    if (nrm<mjMINVAL) {
      // find max component of axis0
      int i = 0;
      if (mju_abs(axis[0][1])>mju_abs(axis[0][0]) &&
          mju_abs(axis[0][1])>mju_abs(axis[0][2])) {
        i = 1;
      }
      if (mju_abs(axis[0][2])>mju_abs(axis[0][0]) &&
          mju_abs(axis[0][2])>mju_abs(axis[0][1])) {
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
    // normal = z
    normal[2] = 1;
    normal[0] = normal[1] = 0;

    // 1st axis = x
    axis[0][0] = 1;
    axis[0][1] = axis[0][2] = 0;

    // 2nd axis = y
    axis[1][1] = 1;
    axis[1][0] = axis[1][2] = 0;
  }

  // project points in 2D frame: p => d
  d[0] = mju_dot3(p[0], axis[0]);
  d[1] = mju_dot3(p[0], axis[1]);
  d[2] = mju_dot3(p[1], axis[0]);
  d[3] = mju_dot3(p[1], axis[1]);
  if (side) {
    // side point: apply same projection as x0, x1
    mju_sub3(tmp, side, xpos);
    mju_mulMatTVec(s, xmat, tmp, 3, 3);
    sd[0] = mju_dot3(s, axis[0]);
    sd[1] = mju_dot3(s, axis[1]);

    // map to circle if outside, set to (0,0) if inside
    if (mju_norm(sd, 2) >= size[0]) {
      mju_normalize(sd, 2);
      mju_scl(sd, sd, size[0], 2);
    } else {
      sd[0] = sd[1] = 0;
    }
  }

  // apply inside wrap
  if (side && sd[0]==0 && sd[1]==0) {
    wlen = wrap_inside(pnt, d, size[0]);
  }

  // apply circle wrap
  else {
    wlen = wrap_circle(pnt, d, (side ? sd : 0), size[0]);
  }

  // no wrap
  if (wlen<0) {
    return -1;
  }

  // reconstruct 3D points in local frame: res
  for (int i=0; i<2; i++) {
    // res = axis0*d0 + axis1*d1
    mju_scl3(res+3*i, axis[0], pnt[2*i]);
    mju_scl3(tmp, axis[1], pnt[2*i+1]);
    mju_addTo3(res+3*i, tmp);
  }

  // cylinder: correct along z
  if (type==mjWRAP_CYLINDER) {
    // set vertical coordinates
    L0 = mju_sqrt((p[0][0]-res[0])*(p[0][0]-res[0]) + (p[0][1]-res[1])*(p[0][1]-res[1]));
    L1 = mju_sqrt((p[1][0]-res[3])*(p[1][0]-res[3]) + (p[1][1]-res[4])*(p[1][1]-res[4]));
    res[2] = p[0][2] + (p[1][2]-p[0][2])*L0/(L0+wlen+L1);
    res[5] = p[0][2] + (p[1][2]-p[0][2])*(L0+wlen)/(L0+wlen+L1);

    // correct wlen for height
    height = mju_abs(res[5] - res[2]);
    wlen = mju_sqrt(wlen*wlen + height*height);
  }

  // map back to global frame: wpnt
  mju_mulMatVec(wpnt, xmat, res, 3, 3);
  mju_mulMatVec(wpnt+3, xmat, res+3, 3, 3);
  mju_addTo3(wpnt, xpos);
  mju_addTo3(wpnt+3, xpos);

  return wlen;
}



// all 3 semi-axes of a geom
void mju_geomSemiAxes(const mjModel* m, int geom_id, mjtNum semiaxes[3]) {
  mjtNum* size = m->geom_size + 3*geom_id;
  switch (m->geom_type[geom_id]) {
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



//------------------------------ actuator models ---------------------------------------------------

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
  if (force<0) {
    force = scale / mjMAX(mjMINVAL, acc0);
  }

  // mid-ranges
  mjtNum a = 0.5*(lmin+1);
  mjtNum b = 0.5*(1+lmax);
  mjtNum x;

  // optimum length
  mjtNum L0 = (lengthrange[1]-lengthrange[0]) / mjMAX(mjMINVAL, range[1]-range[0]);

  // normalized length and velocity
  mjtNum L = range[0] + (len-lengthrange[0]) / mjMAX(mjMINVAL, L0);
  mjtNum V = vel / mjMAX(mjMINVAL, L0*vmax);

  // length curve
  mjtNum FL = 0;
  if (L>=lmin && L<=a) {
    x = (L-lmin) / mjMAX(mjMINVAL, a-lmin);
    FL = 0.5*x*x;
  } else if (L<=1) {
    x = (1-L) / mjMAX(mjMINVAL, 1-a);
    FL = 1 - 0.5*x*x;
  } else if (L<=b) {
    x = (L-1) / mjMAX(mjMINVAL, b-1);
    FL = 1 - 0.5*x*x;
  } else if (L<=lmax) {
    x = (lmax-L) / mjMAX(mjMINVAL, lmax-b);
    FL = 0.5*x*x;
  }

  // velocity curve
  mjtNum FV;
  mjtNum y = fvmax-1;
  if (V<=-1) {
    FV = 0;
  } else if (V<=0) {
    FV = (V+1)*(V+1);
  } else if (V<=y) {
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
  if (force<0) {
    force = scale / mjMAX(mjMINVAL, acc0);
  }

  // optimum length
  mjtNum L0 = (lengthrange[1]-lengthrange[0]) / mjMAX(mjMINVAL, range[1]-range[0]);

  // normalized length
  mjtNum L = range[0] + (len-lengthrange[0]) / mjMAX(mjMINVAL, L0);

  // half-quadratic to (L0+lmax)/2, linear beyond
  mjtNum b = 0.5*(1+lmax);
  if (L<=1) {
    return 0;
  } else if (L<=b) {
    mjtNum x = (L-1) / mjMAX(mjMINVAL, b-1);
    return -force*fpmax*0.5*x*x;
  } else {
    mjtNum x = (L-b) / mjMAX(mjMINVAL, b-1);
    return -force*fpmax*(0.5 + x);
  }
}



// muscle activation dynamics, prm = (tau_act, tau_deact)
mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[2]) {
  // clamp control
  mjtNum ctrlclamp = mju_clip(ctrl, 0, 1);

  // clamp activation
  mjtNum actclamp = mju_clip(act, 0, 1);

  // compute time constant as in Millard et al. (2013) https://doi.org/10.1115/1.4023390
  mjtNum tau;
  if (ctrlclamp>act) {
    tau = prm[0] * (0.5 + 1.5*actclamp);
  } else {
    tau = prm[1] / (0.5 + 1.5*actclamp);
  }

  // filter output
  return (ctrlclamp-act) / mjMAX(mjMINVAL, tau);
}



//------------------------------ miscellaneous -----------------------------------------------------

// convert contact force to pyramid representation
// the pyramid frame is: V0_i = N + mu_i*T_i
//                       V1_i = N - mu_i*T_i
void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force, const mjtNum* mu, int dim) {
  mjtNum a = force[0]/(dim-1), b;

  // arbitary redundancy resolution:
  //  pyramid0_i + pyramid1_i = force_normal/(dim-1) = a
  //  pyramid0_i - pyramid1_i = force_tangent_i/mu_i = b
  for (int i=0; i<dim-1; i++) {
    b = mju_min(a, force[i+1]/mu[i]);
    pyramid[2*i] = 0.5*(a+b);
    pyramid[2*i+1] = 0.5*(a-b);
  }
}



// convert pyramid representation to contact force
void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid, const mjtNum* mu, int dim) {
  // special handling of frictionless contacts
  if (dim==1) {
    force[0] = pyramid[0];
    return;
  }

  // force_normal = sum(pyramid0_i + pyramid1_i)
  force[0] = 0;
  for (int i=0; i<2*(dim-1); i++) {
    force[0] += pyramid[i];
  }

  // force_tangent_i = (pyramid0_i - pyramid1_i) * mu_i
  for (int i=0; i<dim-1; i++) {
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
  if (det>mjMINVAL) {
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
  else if (det<=mjMINVAL && det>=-mjMINVAL) {
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



// print matrix to screen
void mju_printMat(const mjtNum* mat, int nr, int nc) {
  for (int r=0; r<nr; r++) {
    for (int c=0; c<nc; c++) {
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
  for (int r=0; r<nr; r++) {
    for (int adr=rowadr[r]; adr<rowadr[r]+rownnz[r]; adr++) {
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
  if (x<min) {
    return min;
  } else if (x>max) {
    return max;
  } else {
    return x;
  }
}



// sign function
mjtNum mju_sign(mjtNum x) {
  if (x<0) {
    return -1;
  } else if (x>0) {
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
  switch (type) {
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

  else {
    return mjOBJ_UNKNOWN;
  }
}



// warning text
const char* mju_warningText(int warning, int info) {
  static char str[1000];

  switch (warning) {
  case mjWARN_INERTIA:
    mjSNPRINTF(str, "Inertia matrix is too close to singular at DOF %d. Check model.", info);
    break;

  case mjWARN_CONTACTFULL:
    mjSNPRINTF(str, "Pre-allocated contact buffer is full. Increase nconmax above %d.", info);
    break;

  case mjWARN_CNSTRFULL:
    mjSNPRINTF(str, "Pre-allocated constraint buffer is full. Increase njmax above %d.", info);
    break;

  case mjWARN_VGEOMFULL:
    mjSNPRINTF(str, "Pre-allocated visual geom buffer is full. Increase maxgeom above %d.", info);
    break;

  case mjWARN_BADQPOS:
    mjSNPRINTF(str, "Nan, Inf or huge value in QPOS at DOF %d. The simulation is unstable.", info);
    break;

  case mjWARN_BADQVEL:
    mjSNPRINTF(str, "Nan, Inf or huge value in QVEL at DOF %d. The simulation is unstable.", info);
    break;

  case mjWARN_BADQACC:
    mjSNPRINTF(str, "Nan, Inf or huge value in QACC at DOF %d. The simulation is unstable.", info);
    break;

  case mjWARN_BADCTRL:
    mjSNPRINTF(str, "Nan, Inf or huge value in CTRL at ACTUATOR %d. The simulation is unstable.",
               info);
    break;

  default:
    mjSNPRINTF(str, "Unknown warning type %d.", warning);
  }

  return str;
}



// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
int mju_isBad(mjtNum x) {
  return (x!=x || x>mjMAXVAL || x<-mjMAXVAL);
}



// return 1 if all elements are 0
int mju_isZero(mjtNum* vec, int n) {
  for (int i=0; i<n; i++) {
    if (vec[i]!=0) {
      return 0;
    }
  }

  return 1;
}



// standard normal random number generator (optional second number)
mjtNum mju_standardNormal(mjtNum* num2) {
  const mjtNum scale = 2.0/((mjtNum)RAND_MAX);
  mjtNum x1, x2, w;

  do {
    x1 = scale * (mjtNum)rand() - 1.0;
    x2 = scale * (mjtNum)rand() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w>=1.0 || w==0);

  w = mju_sqrt((-2.0 * mju_log(w)) / w);
  if (num2) {
    *num2 = x2 * w;
  }

  return (x1 * w);
}



// convert from float to mjtNum
void mju_f2n(mjtNum* res, const float* vec, int n) {
  for (int i=0; i<n; i++) {
    res[i] = (mjtNum) vec[i];
  }
}



// convert from mjtNum to float
void mju_n2f(float* res, const mjtNum* vec, int n) {
  for (int i=0; i<n; i++) {
    res[i] = (float) vec[i];
  }
}


// convert from double to mjtNum
void mju_d2n(mjtNum* res, const double* vec, int n) {
  for (int i=0; i<n; i++) {
    res[i] = (mjtNum) vec[i];
  }
}



// convert from mjtNum to double
void mju_n2d(double* res, const mjtNum* vec, int n) {
  for (int i=0; i<n; i++) {
    res[i] = (double) vec[i];
  }
}



// insertion sort, increasing order
void mju_insertionSort(mjtNum* list, int n) {
  for (int i=1; i<n; i++) {
    mjtNum x = list[i];
    int j = i-1;
    while (j>=0 && list[j]>x) {
      list[j+1] = list[j];
      j--;
    }
    list[j+1] = x;
  }
}



// integer insertion sort, increasing order
void mju_insertionSortInt(int* list, int n) {
  for (int i=1; i<n; i++) {
    int x = list[i];
    int j = i-1;
    while (j>=0 && list[j]>x) {
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

  while (n0>0) {
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
  if (dst && src && n>0) {
    strncpy(dst, src, n);
    dst[n-1] = 0;
  }

  return dst;
}



// Sigmoid function over 0<=x<=1 constructed from half-quadratics.
mjtNum mju_sigmoid(mjtNum x) {
  // fast return
  if (x<=0) {
    return 0;
  }
  if (x>=1) {
    return 1;
  }
  if (x==0.5) {
    return 0.5;
  }

  // lower part
  if (x<0.5) {
    return 2*x*x;
  }

  // higher part
  return (1 - 2*(1-x)*(1-x));
}
