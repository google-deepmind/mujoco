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

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_misc.h"


//--------------------------- plane collisions -----------------------------------------------------

// raw plane : sphere
static int mjraw_PlaneSphere(mjPreContact* con, mjtNum margin,
                             const mjtNum* pos1, const mjtNum* mat1, const mjtNum* size1,
                             const mjtNum* pos2, const mjtNum* mat2, const mjtNum* size2) {
  // set normal
  con[0].normal[0] = mat1[2];
  con[0].normal[1] = mat1[5];
  con[0].normal[2] = mat1[8];

  // compute distance, return if too large
  mjtNum tmp[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  mjtNum cdist = mju_dot3(tmp, con[0].normal);
  if (cdist > margin + size2[0]) {
    return 0;
  }

  // depth and position
  con[0].dist = cdist - size2[0];
  mji_scl3(tmp, con[0].normal, -con[0].dist / 2 - size2[0]);
  mji_add3(con[0].pos, pos2, tmp);
  mji_zero3(con[0].tangent);
  return 1;
}


// plane : sphere
int mjc_PlaneSphere(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                    mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size1 = m->geom_size + 3*g1;
  const mjtNum* size2 = m->geom_size + 3*g2;
  return mjraw_PlaneSphere(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// plane : capsule
int mjc_PlaneCapsule(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                     mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size1 = m->geom_size + 3*g1;
  const mjtNum* size2 = m->geom_size + 3*g2;

  // get capsule axis, segment = scaled axis
  mjtNum axis[3] = {mat2[2], mat2[5], mat2[8]};
  mjtNum segment[3] = {size2[1]*axis[0], size2[1]*axis[1], size2[1]*axis[2]};

  // get point 1, do sphere-plane test
  mjtNum endpoint[3];
  mju_add3(endpoint, pos2, segment);
  int n1 = mjraw_PlaneSphere(con, margin, pos1, mat1, size1, endpoint, mat2, size2);

  // get point 2, do sphere-plane test
  mju_sub3(endpoint, pos2, segment);
  int n2 = mjraw_PlaneSphere(con + n1, margin, pos1, mat1, size1, endpoint, mat2, size2);

  // align contact frames with capsule axis
  if (n1) {
    mji_copy3(con[0].tangent, axis);
  }
  if (n2) {
    mji_copy3(con[n1].tangent, axis);
  }

  return n1 + n2;
}


// plane : cylinder
int mjc_PlaneCylinder(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                      mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size2 = m->geom_size + 3*g2;

  mjtNum normal[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum axis[3] = {mat2[2], mat2[5], mat2[8]};

  // project, make sure axis points towards plane
  mjtNum prjaxis = mju_dot3(normal, axis);
  if (prjaxis > 0) {
    mju_scl3(axis, axis, -1);
    prjaxis = -prjaxis;
  }

  // compute normal distance to cylinder center
  mjtNum vec[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  mjtNum dist0 = mju_dot3(vec, normal);

  // remove component of -normal along axis, compute length
  mju_scl3(vec, axis, prjaxis);
  mju_subFrom3(vec, normal);
  mjtNum len_sqr = mju_dot3(vec, vec);

  // general configuration: normalize vector, scale by radius
  if (len_sqr >= mjMINVAL*mjMINVAL) {
    mjtNum scl = size2[0]/mju_sqrt(len_sqr);
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
  mjtNum prjvec = mju_dot3(vec, normal);

  // scale axis by half-length
  mju_scl3(axis, axis, size2[1]);
  prjaxis *= size2[1];

  // check first point, construct contact
  int cnt = 0;
  if (dist0 + prjaxis + prjvec <= margin) {
    con[cnt].dist = dist0 + prjaxis + prjvec;
    mji_add3(con[cnt].pos, pos2, vec);
    mji_addTo3(con[cnt].pos, axis);
    mji_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    mji_copy3(con[cnt].normal, normal);
    mji_zero3(con[cnt].tangent);
    cnt++;
  } else {
    return 0;  // nearest point is above margin: no contacts
  }

  // check second point, construct contact
  if (dist0 - prjaxis + prjvec <= margin) {
    con[cnt].dist = dist0 - prjaxis + prjvec;
    mji_add3(con[cnt].pos, pos2, vec);
    mji_subFrom3(con[cnt].pos, axis);
    mji_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    mji_copy3(con[cnt].normal, normal);
    mji_zero3(con[cnt].tangent);
    cnt++;
  }

  // try to add triangle points on side closer to plane
  mjtNum prjvec1 = -prjvec*0.5;
  if (dist0 + prjaxis + prjvec1 <= margin) {
    // compute sideways vector: vec1
    mjtNum vec1[3];
    mji_cross(vec1, vec, axis);
    mju_normalize3(vec1);
    mju_scl3(vec1, vec1, size2[0] * mju_sqrt(3.0) / 2);

    // add point A
    con[cnt].dist = dist0 + prjaxis + prjvec1;
    mji_add3(con[cnt].pos, pos2, vec1);
    mji_addTo3(con[cnt].pos, axis);
    mji_addToScl3(con[cnt].pos, vec, -0.5);
    mji_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    mji_copy3(con[cnt].normal, normal);
    mji_zero3(con[cnt].tangent);
    cnt++;

    // add point B
    con[cnt].dist = dist0 + prjaxis + prjvec1;
    mji_sub3(con[cnt].pos, pos2, vec1);
    mji_addTo3(con[cnt].pos, axis);
    mji_addToScl3(con[cnt].pos, vec, -0.5);
    mji_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    mji_copy3(con[cnt].normal, normal);
    mji_zero3(con[cnt].tangent);
    cnt++;
  }

  return cnt;
}


// plane : box
int mjc_PlaneBox(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2, mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size2 = m->geom_size + 3*g2;

  // get normal, difference between centers, normal distance
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  mjtNum dist = mju_dot3(dif, norm);

  // test all corners, pick bottom 4
  int cnt = 0;
  for (int i=0; i < 8; i++) {
    // get corner in local coordinates
    mjtNum vec[3];
    vec[0] = (i&1 ? size2[0] : -size2[0]);
    vec[1] = (i&2 ? size2[1] : -size2[1]);
    vec[2] = (i&4 ? size2[2] : -size2[2]);

    // get corner in global coordinates relative to box center
    mjtNum corner[3];
    mju_mulMatVec3(corner, mat2, vec);

    // compute distance to plane, skip if too far or pointing up
    mjtNum ldist = mju_dot3(norm, corner);
    if (dist + ldist > margin || ldist > 0) {
      continue;
    }

    // construct contact
    con[cnt].dist = dist + ldist;
    mji_copy3(con[cnt].normal, norm);
    mji_addTo3(corner, pos2);
    mji_scl3(vec, norm, -con[cnt].dist / 2);
    mji_add3(con[cnt].pos, corner, vec);
    mji_zero3(con[cnt].tangent);

    // count; max is 4
    if (++cnt >= 4) {
      return 4;
    }
  }

  return cnt;
}


//--------------------------- sphere and capsule collisions ----------------------------------------

// sphere : sphere (actual implementation, can be called with modified parameters)
static int mjraw_SphereSphere(mjPreContact* con, mjtNum margin,
                              const mjtNum* pos1, const mjtNum* mat1, const mjtNum* size1,
                              const mjtNum* pos2, const mjtNum* mat2, const mjtNum* size2) {
  // check bounding spheres (this is called from other functions)
  mjtNum dif[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};
  mjtNum cdist_sqr = mju_dot3(dif, dif);
  mjtNum min_dist = margin + size1[0] + size2[0];
  if (cdist_sqr > min_dist*min_dist) {
    return 0;
  }

  // depth and normal
  con[0].dist = mju_sqrt(cdist_sqr) - size1[0] - size2[0];
  mji_sub3(con[0].normal, pos2, pos1);
  mjtNum len = mju_normalize3(con[0].normal);

  // if centers are the same, norm = cross-product of z axes
  //  if z axes are parallel, norm = [1;0;0]
  if (len < mjMINVAL) {
    mjtNum axis1[3] = {mat1[2], mat1[5], mat1[8]};
    mjtNum axis2[3] = {mat2[2], mat2[5], mat2[8]};
    mji_cross(con[0].normal, axis1, axis2);
    mju_normalize3(con[0].normal);
  }

  // position
  mji_scl3(con[0].pos, con[0].normal, size1[0] + con[0].dist / 2);
  mji_addTo3(con[0].pos, pos1);

  // axis
  mji_zero3(con[0].tangent);

  return 1;
}


// sphere : sphere
int mjc_SphereSphere(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                     mjtNum margin) {
  const mjtNum* pos1  = d->geom_xpos + 3*g1;
  const mjtNum* mat1  = d->geom_xmat + 9*g1;
  const mjtNum* size1 = m->geom_size + 3*g1;
  const mjtNum* pos2  = d->geom_xpos + 3*g2;
  const mjtNum* mat2  = d->geom_xmat + 9*g2;
  const mjtNum* size2 = m->geom_size + 3*g2;
  return mjraw_SphereSphere(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}



// raw sphere : capsule
int mjraw_SphereCapsule(mjPreContact* con, mjtNum margin,
                        const mjtNum* pos1, const mjtNum* mat1, const mjtNum* size1,
                        const mjtNum* pos2, const mjtNum* mat2, const mjtNum* size2) {
  // get capsule length and axis
  mjtNum len = size2[1];
  mjtNum axis[3] = {mat2[2], mat2[5], mat2[8]};

  // find projection, clip to segment
  mjtNum vec[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};
  mjtNum x = mju_clip(mju_dot3(axis, vec), -len, len);

  // find nearest point on segment, do sphere-sphere test
  mji_scl3(vec, axis, x);
  mji_addTo3(vec, pos2);
  return mjraw_SphereSphere(con, margin, pos1, mat1, size1, vec, mat2, size2);
}


// sphere : capsule
int mjc_SphereCapsule(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                      mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size1 = m->geom_size + 3*g1;
  const mjtNum* size2 = m->geom_size + 3*g2;
  return mjraw_SphereCapsule(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// sphere : cylinder
int mjc_SphereCylinder(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                       mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size1 = m->geom_size + 3*g1;
  const mjtNum* size2 = m->geom_size + 3*g2;

  // get cylinder sizes and axis
  mjtNum radius = size2[0];
  mjtNum height = size2[1];
  mjtNum axis[3] = {mat2[2], mat2[5], mat2[8]};

  // find sphere projection onto cylinder axis and plane
  mjtNum vec[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};
  mjtNum x = mju_dot3(axis, vec);
  mjtNum a_proj[3], p_proj[3];
  mji_scl3(a_proj, axis, x);
  mji_sub3(p_proj, vec, a_proj);
  mjtNum p_proj_sqr = mju_dot3(p_proj, p_proj);

  // get collision type
  int collide_side = mju_abs(x) < height;
  int collide_cap = p_proj_sqr < radius*radius;
  if (collide_side && collide_cap) {  // deep penetration (sphere origin inside cylinder)
    mjtNum dist_cap = height - mju_abs(x);
    mjtNum dist_radius = radius - mju_sqrt(p_proj_sqr);
    if (dist_cap < dist_radius) {  // disable one collision type
      collide_side = 0;
    } else {
      collide_cap = 0;
    }
  }

  // side collision: use sphere-sphere
  if (collide_side) {
    mji_addTo3(a_proj, pos2);
    return mjraw_SphereSphere(con, margin, pos1, mat1, size1, a_proj, mat2, size2);
  }

  // cap collision: use plane-sphere
  if (collide_cap) {
    const mjtNum flipmat[9] = {
      -mat2[0], mat2[1], -mat2[2],
      -mat2[3], mat2[4], -mat2[5],
      -mat2[6], mat2[7], -mat2[8]
    };
    const mjtNum* mat_cap;
    mjtNum pos_cap[3];
    if (x > 0) {  // top cap
      mju_addScl3(pos_cap, pos2, axis, height);
      mat_cap = mat2;
    } else {      // bottom cap
      mju_addScl3(pos_cap, pos2, axis, -height);
      mat_cap = flipmat;
    }
    int ncon = mjraw_PlaneSphere(con, margin, pos_cap, mat_cap, size2, pos1, mat1, size1);

    if (ncon) {
      // flip direction normal (because mjGEOM_PLANE < mjGEOM_SPHERE < mjGEOM_CYLINDER)
      mju_scl3(con[0].normal, con[0].normal, -1);
    }
    return ncon;
  }

  // otherwise corner collision: use sphere-sphere
  mju_scl3(p_proj, p_proj,
           size2[0] / mju_sqrt(p_proj_sqr));  // denominator cannot be 0
  mji_scl3(vec, axis, x > 0 ? height : -height);
  mji_addTo3(vec, p_proj);
  mji_addTo3(vec, pos2);

  // sphere-sphere with point sphere at the corner
  mjtNum size_zero[1] = {0};
  return mjraw_SphereSphere(con, margin, pos1, mat1, size1, vec, mat2, size_zero);
}


// raw capsule : capsule
int mjraw_CapsuleCapsule(mjPreContact* con, mjtNum margin,
                         const mjtNum* pos1, const mjtNum* mat1, const mjtNum* size1,
                         const mjtNum* pos2, const mjtNum* mat2, const mjtNum* size2) {
  // get capsule axes (scaled) and segment endpoints
  mjtNum axis1[3] = {mat1[2] * size1[1], mat1[5] * size1[1], mat1[8] * size1[1]};
  mjtNum axis2[3] = {mat2[2] * size2[1], mat2[5] * size2[1], mat2[8] * size2[1]};
  mjtNum p1[3], p2[3], q1[3], q2[3];
  mju_sub3(p1, pos1, axis1);
  mju_add3(p2, pos1, axis1);
  mju_sub3(q1, pos2, axis2);
  mju_add3(q2, pos2, axis2);

  mjtNum w[4];
  int n = mjc_segWeights(w, p1, p2, q1, q2);

  int ncon = 0;
  for (int i = 0; i < n; i++) {
    mjtNum vec1[3], vec2[3];
    for (int k = 0; k < 3; k++) {
      vec1[k] = p1[k] + w[2*i] * (p2[k] - p1[k]);
      vec2[k] = q1[k] + w[2*i + 1] * (q2[k] - q1[k]);
    }
    ncon += mjraw_SphereSphere(con + ncon, margin, vec1, mat1, size1, vec2, mat2, size2);
  }
  return ncon;
}


// capsule : capsule
int mjc_CapsuleCapsule(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2,
                        mjtNum margin) {
  const mjtNum* pos1 = d->geom_xpos + 3*g1;
  const mjtNum* pos2 = d->geom_xpos + 3*g2;
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  const mjtNum* mat2 = d->geom_xmat + 9*g2;
  const mjtNum* size1 = m->geom_size + 3*g1;
  const mjtNum* size2 = m->geom_size + 3*g2;
  return mjraw_CapsuleCapsule(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// compute the barycentric weights w of the closest point on triangle ABC to p
void mjc_triWeights(mjtNum w[3], const mjtNum p[3], const mjtNum a[3], const mjtNum b[3],
                    const mjtNum c[3]) {
  // compute barycentric coordinates via Voronoi regions (See Ericson 5.1.5)
  mjtNum ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  mjtNum ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  mjtNum ap[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
  mjtNum bp[3] = {p[0] - b[0], p[1] - b[1], p[2] - b[2]};
  mjtNum cp[3] = {p[0] - c[0], p[1] - c[1], p[2] - c[2]};

  mjtNum dot1 = mju_dot3(ab, ap);
  mjtNum dot2 = mju_dot3(ac, ap);
  mjtNum dot3 = mju_dot3(ab, bp);
  mjtNum dot4 = mju_dot3(ac, bp);
  mjtNum dot5 = mju_dot3(ab, cp);
  mjtNum dot6 = mju_dot3(ac, cp);

  // vertex region outside A
  if (dot1 <= 0 && dot2 <= 0) {
    w[0] = 1, w[1] = 0, w[2] = 0;
    return;
  }

  // vertex region outside B
  if (dot3 >= 0 && dot4 <= dot3) {
    w[0] = 0, w[1] = 1, w[2] = 0;
    return;
  }

  // vertex region outside C
  if (dot6 >= 0 && dot5 <= dot6) {
    w[0] = 0, w[1] = 0, w[2] = 1;
    return;
  }

  // signed areas of PBC, PCA, PAB scaled by 4*area(ABC)
  mjtNum va = dot3 * dot6 - dot5 * dot4;
  mjtNum vb = dot5 * dot2 - dot1 * dot6;
  mjtNum vc = dot1 * dot4 - dot3 * dot2;

  // edge region AB
  if (vc <= 0 && dot1 >= 0 && dot3 <= 0) {
    mjtNum t = dot1 / (dot1 - dot3);
    w[0] = 1 - t,  w[1] = t, w[2] = 0;
    return;
  }

  // edge region AC
  if (vb <= 0 && dot2 >= 0 && dot6 <= 0) {
    mjtNum t = dot2 / (dot2 - dot6);
    w[0] = 1 - t, w[1] = 0, w[2] = t;
    return;
  }

  // edge region BC
  if (va <= 0 && (dot4 - dot3) >= 0 && (dot5 - dot6) >= 0) {
    mjtNum t = (dot4 - dot3) / ((dot4 - dot3) + (dot5 - dot6));
    w[0] = 0, w[1] = 1 - t, w[2] = t;
    return;
  }

  // face region ABC
  mjtNum d = 1.0 / (va + vb + vc), t = vb * d, u = vc * d;
  w[0] = 1 - t - u, w[1] = t, w[2] = u;
}


// clamp t to [0, 1] and recompute s on segment AB (s0 at t=0, s1 at t=1)
static inline void clampSeg(mjtNum w[2], mjtNum s, mjtNum t, mjtNum s0, mjtNum s1) {
  if (t < 0) {
    w[0] = mju_clip(s0, 0, 1), w[1] = 0;
  } else if (t > 1) {
    w[0] = mju_clip(s1, 0, 1), w[1] = 1;
  } else {
    w[0] = s, w[1] = t;
  }
}


// compute the barycentric weights w = {s1, t1, s2, t2} in [0, 1] of the closest points
// A + s*(B - A) on segment AB and C + t*(D - C) on segment CD;
// return the number of closest point pairs (1, or 2 if parallel)
int mjc_segWeights(mjtNum w[4], const mjtNum a[3], const mjtNum b[3], const mjtNum c[3],
                   const mjtNum d[3]) {
  const mjtNum sin2 = 1e-8;  // sin^2(0.006 deg) cutoff for parallel segments

  // compute barycentric coordinates via Voronoi regions (See Ericson 5.1.9)
  mjtNum ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  mjtNum cd[3] = {d[0] - c[0], d[1] - c[1], d[2] - c[2]};
  mjtNum ca[3] = {a[0] - c[0], a[1] - c[1], a[2] - c[2]};

  mjtNum dot1 = mju_dot3(ab, ab);
  mjtNum dot2 = mju_dot3(cd, cd);
  mjtNum dot3 = mju_dot3(cd, ca);
  mjtNum dot4 = mju_dot3(ab, ca);
  mjtNum dot5 = mju_dot3(ab, cd);

  // both segments degenerate into points
  if (dot1 <= mjMINVAL && dot2 <= mjMINVAL) {
    w[0] = 0, w[1] = 0;
    return 1;
  }

  // first segment degenerates into a point
  if (dot1 <= mjMINVAL) {
    w[0] = 0, w[1] = mju_clip(dot3 / dot2, 0, 1);
    return 1;
  }

  // second segment degenerates into a point
  if (dot2 <= mjMINVAL) {
    w[0] = mju_clip(-dot4 / dot1, 0, 1), w[1] = 0;
    return 1;
  }

  // projections of C (t=0) and D (t=1) onto AB
  mjtNum s0 = -dot4 / dot1;
  mjtNum s1 = (dot5 - dot4) / dot1;

  // compute determinant via Lagrange's identity to avoid cancellation for parallel segments
  mjtNum cross[3];
  mji_cross(cross, ab, cd);
  mjtNum det = mju_dot3(cross, cross);

  // segments are considered non-parallel
  if (det > sin2 * dot1 * dot2) {
    // compute s via Binet-Cauchy identity to avoid cancellation for near-parallel segments
    mjtNum cross_ca[3];
    mji_cross(cross_ca, cd, ca);
    mjtNum s = mju_clip(mju_dot3(cross, cross_ca) / det, 0, 1);
    clampSeg(w, s, (dot5 * s + dot3) / dot2, s0, s1);
    return 1;
  }

  // segments are close to parallel (compute closest points on both segments)
  clampSeg(w, 0, dot3 / dot2, s0, s1);
  clampSeg(w + 2, 1, (dot5 + dot3) / dot2, s0, s1);
  return 2;
}
