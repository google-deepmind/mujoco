// Copyright 2026 DeepMind Technologies Limited
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

#include "engine/engine_collision_flex.h"

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_spatial.h"

// test a plane geom and a flex for collision, return number of contacts
int mjc_PlaneFlex(const mjModel* m, mjData* d, mjPreContact* con, int* vert, int g, int f,
                  mjtNum margin) {
  mjtNum radius = m->flex_radius[f];
  mjtNum* pos = d->geom_xpos + 3*g;
  mjtNum* mat = d->geom_xmat + 9*g;
  mjtNum normal[3] = {mat[2], mat[5], mat[8]};

  // prepare contact parameters (same for all vertices)
  int flex_vertnum = m->flex_vertnum[f];

  // collide all flex vertices with plane
  int ncon = 0;
  for (int i = 0; i < flex_vertnum; i++) {
    mjtNum* v = d->flexvert_xpos + 3*(m->flex_vertadr[f] + i);

    // distance from plane to vertex
    mjtNum dif[3] = {v[0] - pos[0], v[1] - pos[1], v[2] - pos[2]};
    mjtNum dist = mju_dot3(dif, normal);

    // no contact
    if (dist > margin + radius) {
      continue;
    }

    // create contact
    con[ncon].dist = dist - radius;
    mju_addScl3(con[ncon].pos, v, normal, -con[ncon].dist * 0.5 - radius);
    mju_copy3(con[ncon].normal, normal);
    mju_zero3(con[ncon].tangent);
    vert[ncon] = i;
    ncon++;
  }
  return ncon;
}


// filter contact based sphere-box test, treating sphere as box
static int filterSphereBox(const mjtNum s[3], mjtNum bound, const mjtNum aabb[6]) {
  if (s[0] + bound < aabb[0] - aabb[3]) return 1;
  if (s[1] + bound < aabb[1] - aabb[4]) return 1;
  if (s[2] + bound < aabb[2] - aabb[5]) return 1;
  if (s[0] - bound > aabb[0] + aabb[3]) return 1;
  if (s[1] - bound > aabb[1] + aabb[4]) return 1;
  if (s[2] - bound > aabb[2] + aabb[5]) return 1;
  return 0;
}


// filter contact based on global AABBs
static int filterBox(const mjtNum aabb1[6], const mjtNum aabb2[6], mjtNum margin) {
  if (aabb1[0] + aabb1[3] + margin < aabb2[0] - aabb2[3]) return 1;
  if (aabb1[1] + aabb1[4] + margin < aabb2[1] - aabb2[4]) return 1;
  if (aabb1[2] + aabb1[5] + margin < aabb2[2] - aabb2[5]) return 1;
  if (aabb2[0] + aabb2[3] + margin < aabb1[0] - aabb1[3]) return 1;
  if (aabb2[1] + aabb2[4] + margin < aabb1[1] - aabb1[4]) return 1;
  if (aabb2[2] + aabb2[5] + margin < aabb1[2] - aabb1[5]) return 1;
  return 0;
}


// make capsule from two flex vertices
static void makeCapsule(const mjModel* m, mjData* d, int f, const int vid[2],
                        mjtNum pos[3], mjtNum mat[9], mjtNum size[2]) {
  // get vertex positions
  mjtNum* v1 = d->flexvert_xpos + 3*(m->flex_vertadr[f] + vid[0]);
  mjtNum* v2 = d->flexvert_xpos + 3*(m->flex_vertadr[f] + vid[1]);

  // construct capsule from vertices
  mjtNum dif[3] = {v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2]};
  size[0] = m->flex_radius[f];
  size[1] = 0.5*mju_normalize3(dif);

  mju_add3(pos, v1, v2);
  mju_scl3(pos, pos, 0.5);

  mjtNum quat[4];
  mju_quatZ2Vec(quat, dif);
  mju_quat2Mat(mat, quat);
}


// test a geom and an elem for collision, return number of contacts
int mjc_GeomElem(const mjModel* m, mjData* d, mjPreContact* con, int g, int f, int e,
                 mjtNum margin) {
  int dim = m->flex_dim[f], type = m->geom_type[g], ncon;

  // bounding sphere test: only if midphase is disabled
  if (mjDISABLED(mjDSBL_MIDPHASE)) {
    int eglobal = m->flex_elemadr[f] + e;
    if (filterSphereBox(d->geom_xpos + 3*g, m->geom_rbound[g] + margin,
                        d->flexelem_aabb + 6*eglobal)) {
      return 0;
    }
  }

  // skip if element has vertices on the same body as geom
  int b = m->geom_bodyid[g];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim + 1);
  const int* bdata = m->flex_vertbodyid + m->flex_vertadr[f];
  for (int i = 0; i <= dim; i++) {
    if (b >= 0 && b == bdata[edata[i]]) {
      return 0;
    }
  }

  // sphere/capsule/box : capsule
  if (dim == 1 && (type == mjGEOM_SPHERE || type == mjGEOM_CAPSULE || type == mjGEOM_BOX)) {
    // make capsule from vertices
    mjtNum pos[3], mat[9], size[2];
    makeCapsule(m, d, f, m->flex_elem + m->flex_elemdataadr[f] + e*2, pos, mat, size);

    // call raw primitive for corresponding geom type
    switch (type) {
      case mjGEOM_SPHERE:
        ncon = mjraw_SphereCapsule(con, margin, d->geom_xpos + 3*g, d->geom_xmat+9*g,
                                   m->geom_size + 3*g, pos, mat, size);
        break;
      case mjGEOM_CAPSULE:
        ncon = mjraw_CapsuleCapsule(con, margin, d->geom_xpos + 3*g, d->geom_xmat + 9*g,
                                    m->geom_size + 3*g, pos, mat, size);
        break;
      case mjGEOM_BOX:
        ncon = mjraw_CapsuleBox(con, margin, pos, mat, size, d->geom_xpos + 3*g,
                                d->geom_xmat + 9*g, m->geom_size + 3*g);
        for (int i=0; i < ncon; i++) {
          mju_scl3(con[i].normal, con[i].normal, -1);
        }
        break;
      default:
        ncon = 0;
    }
  }

  // heightfield : elem
  else if (type == mjGEOM_HFIELD) {
    ncon = mjc_HFieldElem(m, d, con, g, f, e, margin);
  }

  // sphere : triangle
  else if (type == mjGEOM_SPHERE && dim == 2) {
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    ncon = mjraw_SphereTriangle(con, margin,
                               d->geom_xpos + 3*g, m->geom_size[3*g],
                               vertxpos + 3*edata[0], vertxpos + 3*edata[1],
                               vertxpos + 3*edata[2], m->flex_radius[f]);
  }

  // box : triangle
  else if (type == mjGEOM_BOX && dim == 2) {
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    ncon = mjraw_BoxTriangle(con, margin, d->geom_xpos + 3*g,
                             d->geom_xmat + 9*g, m->geom_size + 3*g,
                             vertxpos + 3*edata[0], vertxpos + 3*edata[1],
                             vertxpos + 3*edata[2], m->flex_radius[f]);
  }

  // capsule : triangle
  else if (type == mjGEOM_CAPSULE && dim == 2) {
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    ncon = mjraw_CapsuleTriangle(con, margin, d->geom_xpos + 3*g, d->geom_xmat + 9*g,
                                 m->geom_size + 3*g, vertxpos + 3*edata[0], vertxpos + 3*edata[1],
                                 vertxpos + 3*edata[2], m->flex_radius[f]);
  }

  // general geom : elem
  else {
    ncon = mjc_ConvexElem(m, d, con, g, -1, -1, -1, f, e, margin);
  }

  return ncon;
}


// test two elems for collision, return number of contacts
int mjc_ElemElem(const mjModel* m, mjData* d, mjPreContact* con, int f1, int e1,
                 int f2, int e2, mjtNum margin) {
  int dim1 = m->flex_dim[f1], dim2 = m->flex_dim[f2];
  int ncon = 0;

  // ignore margin and gap in self-collisions
  if (f1 == f2) {
    margin = 0;
  }

  // bounding box filter (not applied in midphase)
  if (filterBox(d->flexelem_aabb + 6*(m->flex_elemadr[f1] + e1),
                d->flexelem_aabb + 6*(m->flex_elemadr[f2] + e2), margin)) {
    return 0;
  }

  // skip if elements have vertices on the same body
  const int* edata1 = m->flex_elem + m->flex_elemdataadr[f1] + e1*(dim1 + 1);
  const int* edata2 = m->flex_elem + m->flex_elemdataadr[f2] + e2*(dim2 + 1);
  const int* bdata1 = m->flex_vertbodyid + m->flex_vertadr[f1];
  const int* bdata2 = m->flex_vertbodyid + m->flex_vertadr[f2];
  for (int i1 = 0; i1 <= dim1; i1++) {
    int b1 = bdata1[edata1[i1]];
    for (int i2 = 0; i2 <= dim2; i2++) {
      if (b1 >= 0 && b1 == bdata2[edata2[i2]]) {
        return 0;
      }
    }
  }

  // capsule : capsule
  if (dim1 == 1 && dim2 == 1) {
    // make capsules from vertices
    mjtNum pos1[3], mat1[9], size1[2];
    mjtNum pos2[3], mat2[9], size2[2];
    makeCapsule(m, d, f1, m->flex_elem + m->flex_elemdataadr[f1] + e1*2,
                pos1, mat1, size1);
    makeCapsule(m, d, f2, m->flex_elem + m->flex_elemdataadr[f2] + e2*2,
                pos2, mat2, size2);

    // raw primitive
    ncon = mjraw_CapsuleCapsule(con, margin, pos1, mat1, size1, pos2, mat2, size2);
  }

  // general convex collision
  else {
    ncon = mjc_ConvexElem(m, d, con, -1, f1, e1, -1, f2, e2, margin);
  }
  return ncon;
}


// test element and vertex for collision, return number of contacts
int mjc_ElemVert(const mjModel* m, mjData* d, mjPreContact* con, int f, int e, int v,
                 mjtNum margin) {
  mjtNum radius = m->flex_radius[f];
  const mjtNum* vert = d->flexvert_xpos + 3*(m->flex_vertadr[f] + v);
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim + 1);
  int ncon = 0;

  // box-box filter (sphere treated as box)
  const mjtNum* aabb = d->flexelem_aabb + 6*(m->flex_elemadr[f] + e);
  mjtNum rbound = margin + radius;
  if (aabb[0] - aabb[3] > vert[0] + rbound) return 0;
  if (aabb[1] - aabb[4] > vert[1] + rbound) return 0;
  if (aabb[2] - aabb[5] > vert[2] + rbound) return 0;
  if (aabb[0] + aabb[3] < vert[0] - rbound) return 0;
  if (aabb[1] + aabb[4] < vert[1] - rbound) return 0;
  if (aabb[2] + aabb[5] < vert[2] - rbound) return 0;

  // sphere : capsule
  if (dim == 1) {
    mjtNum pos[3], mat[9], size[2];
    mjtNum I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    makeCapsule(m, d, f, edata, pos, mat, size);
    ncon = mjraw_SphereCapsule(con, 0, vert, I, &radius, pos, mat, size);
  }

  // sphere : triangle
  else if (dim == 2) {
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    ncon = mjraw_SphereTriangle(con, 0, vert, radius,
                               vertxpos + 3*edata[0], vertxpos + 3*edata[1],
                               vertxpos + 3*edata[2], radius);
  }

  // sphere : tetrahedron
  else {
    ncon = mjc_ConvexElem(m, d, con, -1, f, -1, v, f, e, 0);
  }
  return ncon;
}
