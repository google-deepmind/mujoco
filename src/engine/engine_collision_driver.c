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

#include "engine/engine_collision_driver.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_callback.h"
#include "engine/engine_collision_continuous.h"
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_flex.h"
#include "engine/engine_collision_gjk.h"
#include "engine/engine_collision_nonconvex.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_collision_sdf.h"
#include "engine/engine_thread.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_inline.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_simd.h"
#include "engine/engine_sleep.h"
#include "engine/engine_sort.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"


// table of pairwise collision functions
#define mjCOLLISION_TABLE(X, _)                                                                                                                                         \
  /*                  PLANE  HFIELD  SPHERE            CAPSULE             ELLIPSOID         CYLINDER            BOX               MESH              SDF */             \
  X(mjGEOM_PLANE,     _,     _,      mjc_PlaneSphere,  mjc_PlaneCapsule,   mjc_PlaneConvex,  mjc_PlaneCylinder,  mjc_PlaneBox,     mjc_PlaneConvex,  mjc_PlaneConvex)   \
  X(mjGEOM_HFIELD,    _,     _,      mjc_ConvexHField, mjc_ConvexHField,   mjc_ConvexHField, mjc_ConvexHField,   mjc_ConvexHField, mjc_ConvexHField, mjc_HFieldSDF)     \
  X(mjGEOM_SPHERE,    _,     _,      mjc_SphereSphere, mjc_SphereCapsule,  mjc_Convex,       mjc_SphereCylinder, mjc_SphereBox,    mjc_Convex,       mjc_SDF)           \
  X(mjGEOM_CAPSULE,   _,     _,      _,                mjc_CapsuleCapsule, mjc_Convex,       mjc_Convex,         mjc_CapsuleBox,   mjc_Convex,       mjc_SDF)           \
  X(mjGEOM_ELLIPSOID, _,     _,      _,                _,                  mjc_Convex,       mjc_Convex,         mjc_Convex,       mjc_Convex,       mjc_SDF)           \
  X(mjGEOM_CYLINDER,  _,     _,      _,                _,                  _,                mjc_Convex,         mjc_Convex,       mjc_Convex,       mjc_SDF)           \
  X(mjGEOM_BOX,       _,     _,      _,                _,                  _,                _,                  mjc_BoxBox,       mjc_Convex,       mjc_SDF)           \
  X(mjGEOM_MESH,      _,     _,      _,                _,                  _,                _,                  _,                mjc_Convex,       mjc_MeshSDF)       \
  X(mjGEOM_SDF,       _,     _,      _,                _,                  _,                _,                  _,                _,                mjc_SDF)

#define MJ_COLLISION_ROW(t1, c0, c1, c2, c3, c4, c5, c6, c7, c8) \
  MJ_CELL(t1, 0, c0) MJ_CELL(t1, 1, c1) MJ_CELL(t1, 2, c2)       \
  MJ_CELL(t1, 3, c3) MJ_CELL(t1, 4, c4) MJ_CELL(t1, 5, c5)       \
  MJ_CELL(t1, 6, c6) MJ_CELL(t1, 7, c7) MJ_CELL(t1, 8, c8)
#define MJ_NONE(...) 0

#define MJ_ROW(t1, ...) {__VA_ARGS__},
mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES] = {mjCOLLISION_TABLE(MJ_ROW, 0)};
static const mjfCollision mjDEFAULTCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES]
    = {mjCOLLISION_TABLE(MJ_ROW, 0)};
#undef MJ_ROW

// return 1 if mjCOLLISIONFUNC has not been modified from defaults
static inline int isDefaultCollisionFunc(void) {
  return memcmp(mjCOLLISIONFUNC, mjDEFAULTCOLLISIONFUNC, sizeof(mjCOLLISIONFUNC)) == 0;
}



//------------------------------------ utility functions ------------------------------------------


// return the maximum number of contacts that can be generated between two geoms
// has_margin is ignored
int mj_maxContact(const mjModel* m, int g1, int g2, int has_margin) {
  int type1 = m->geom_type[g1];
  int type2 = m->geom_type[g2];

  if (type1 == mjGEOM_SDF || type2 == mjGEOM_SDF) {
    if (type1 == mjGEOM_PLANE || type2 == mjGEOM_PLANE) {
      return 4;
    }
    if (type1 == mjGEOM_MESH || type2 == mjGEOM_MESH) {
      return mjMAXCONPAIR;
    }
    if (type1 == mjGEOM_HFIELD || type2 == mjGEOM_HFIELD) {
      return 0;
    }
    return mjMIN(mjMAXCONPAIR, mjMAX(0, m->opt.sdf_initpoints));
  }

  if (type1 == mjGEOM_HFIELD || type2 == mjGEOM_HFIELD) {
    int type = (type1 == mjGEOM_HFIELD) ? type2 : type1;
    return (type != mjGEOM_PLANE && type != mjGEOM_HFIELD) ? mjMAXCONPAIR : 0;
  }

  // spheres and ellipsoids always generate a single contact
  if (type1 == mjGEOM_SPHERE || type1 == mjGEOM_ELLIPSOID ||
      type2 == mjGEOM_SPHERE || type2 == mjGEOM_ELLIPSOID) {
    return 1;
  }

  // box-box primitive collider
  if (type1 == mjGEOM_BOX && type2 == mjGEOM_BOX) {
    return 8;
  }

  // capsule-capsule primitive collider
  if (type1 == mjGEOM_CAPSULE && type2 == mjGEOM_CAPSULE) {
    return 2;
  }

  // capsule-box primitive collider
  if ((type1 == mjGEOM_CAPSULE && type2 == mjGEOM_BOX) ||
      (type1 == mjGEOM_BOX && type2 == mjGEOM_CAPSULE)) {
    return 4;
  }

  // geoms with flat faces
  int hasface1 = (type1 == mjGEOM_BOX || type1 == mjGEOM_MESH || type1 == mjGEOM_CYLINDER);
  int hasface2 = (type2 == mjGEOM_BOX || type2 == mjGEOM_MESH || type2 == mjGEOM_CYLINDER);

  // geoms with only edges
  int hasedge1 = (type1 == mjGEOM_CAPSULE);
  int hasedge2 = (type2 == mjGEOM_CAPSULE);

  // the remaining plane cases
  if (type1 == mjGEOM_PLANE || type2 == mjGEOM_PLANE) {
    if (hasface1 || hasface2) {
      return 4;
    }
    if (hasedge1 || hasedge2) {
      return 2;
    }
    return 0;
  }

  int is_multiccd = !mjDISABLED(mjDSBL_MULTICCD);
  if (!is_multiccd) {
    return 1;
  }

  if (mjDISABLED(mjDSBL_NATIVECCD)) {
    return 5;
  }

  // colliding geoms with flat faces
  if (hasface1 && hasface2) {
    return 4;
  }

  // colliding geoms with edges
  if ((hasedge1 && hasface2) || (hasface1 && hasedge2) || (hasedge1 && hasedge2)) {
    return 2;
  }

  return 1;
}


// return the margin for a given geom pair
static inline mjtNum getMargin(const mjModel* m, int g1, int g2, int ipair) {
  if (ipair >= 0) {
    return mj_assignMargin(m, m->pair_margin[ipair]);
  }
  return mj_assignMargin(m, m->geom_margin[g1] + m->geom_margin[g2]);
}


// return the gap for a given geom pair
static inline mjtNum getGap(const mjModel* m, int g1, int g2, int ipair) {
  if (ipair >= 0) {
    return m->pair_gap[ipair];
  }
  return m->geom_gap[g1] + m->geom_gap[g2];
}


// move arena pointer back to the end of the contact array
static inline void resetArena(mjData* d) {
  d->parena = d->ncon * sizeof(mjContact);
#ifdef mjUSEASAN
    ASAN_POISON_MEMORY_REGION(
      (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif
}


// realign the arena pointer to the specified alignment
static inline size_t alignArena(mjData* d, size_t alignment) {
  size_t misalignment = d->parena % alignment;
  if (misalignment) {
    d->parena += alignment - misalignment;
  }
  return d->parena;
}


// plane to geom_center squared distance, g1 is a plane
static inline mjtNum planeGeomDist(const mjModel* m, const mjData* d, int g1, int g2) {
  const mjtNum* mat1 = d->geom_xmat + 9*g1;
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3];

  mju_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
  return mju_dot3(dif, norm);
}


// filter contact based on type and affinity
static inline int filterBitmask(int contype1, int conaffinity1,
                                int contype2, int conaffinity2) {
  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}


// return 1 if geoms g1 and g2 cannot collide by bitmask, 0 otherwise
static inline int filterGeomPair(const mjModel* m, int g1, int g2) {
  return filterBitmask(m->geom_contype[g1], m->geom_conaffinity[g1],
                       m->geom_contype[g2], m->geom_conaffinity[g2]);
}


// return 1 if geom g and body b cannot collide by bitmask, 0 otherwise
static inline int filterGeomBody(const mjModel* m, int g, int b) {
  return filterBitmask(m->geom_contype[g], m->geom_conaffinity[g],
                       m->body_contype[b], m->body_conaffinity[b]);
}


// return 1 if geom g and flex f cannot collide by bitmask, 0 otherwise
static inline int filterGeomFlex(const mjModel* m, int g, int f) {
  return filterBitmask(m->geom_contype[g], m->geom_conaffinity[g],
                       m->flex_contype[f], m->flex_conaffinity[f]);
}


// return 1 if geom can collide, 0 otherwise
static inline int geomCollidable(const mjModel* m, int g) {
  return m->geom_contype[g] || m->geom_conaffinity[g];
}


// return 1 if flex can collide, 0 otherwise
static inline int flexCollidable(const mjModel* m, int f) {
  return m->flex_contype[f] || m->flex_conaffinity[f];
}


// return 1 if body can collide, 0 otherwise
static inline int bodyCollidable(const mjModel* m, int b) {
  return m->body_contype[b] || m->body_conaffinity[b];
}


// return 1 if bodyflex can collide, 0 otherwise
static inline int collidable(const mjModel* m, int bf) {
  return (bf < m->nbody) ? bodyCollidable(m, bf) : flexCollidable(m, bf - m->nbody);
}


// return 1 if two bodyflexes can collide, 0 otherwise
static inline int pairCollidable(const mjModel* m, int bf1, int bf2) {
  int f1 = bf1 - m->nbody, f2 = bf2 - m->nbody;
  int contype1 = (f1 < 0) ? m->body_contype[bf1] : m->flex_contype[f1];
  int conaffinity1 = (f1 < 0) ? m->body_conaffinity[bf1] : m->flex_conaffinity[f1];
  int contype2 = (f2 < 0) ? m->body_contype[bf2] : m->flex_contype[f2];
  int conaffinity2 = (f2 < 0) ? m->body_conaffinity[bf2] : m->flex_conaffinity[f2];
  return !filterBitmask(contype1, conaffinity1, contype2, conaffinity2);
}


// return 1 if body has collidable plane geom, 0 otherwise
static int hasPlane(const mjModel* m, int body) {
  int start = m->body_geomadr[body];
  int end = m->body_geomadr[body] + m->body_geomnum[body];

  // scan geoms belonging to body
  int g;
  for (g=start; g < end; g++) {
    if (m->geom_type[g] == mjGEOM_PLANE && (mjcb_contactfilter || geomCollidable(m, g))) {
      return 1;
    }
  }

  return 0;
}


// filter contact based on global AABBs
static inline int filterBox(const mjtNum aabb1[6], const mjtNum aabb2[6], mjtNum margin) {
  mjtNum4 c1 = v4_load3(aabb1);
  mjtNum4 s1 = v4_load3(aabb1+3);
  mjtNum4 c2 = v4_load3(aabb2);
  mjtNum4 s2 = v4_load3(aabb2+3);
  mjtNum4 m = v4_splat(margin);
  mjtMask4 sep = v4_or(v4_cmplt(v4_add(v4_add(c1, s1), m), v4_sub(c2, s2)),
                       v4_cmplt(v4_add(v4_add(c2, s2), m), v4_sub(c1, s1)));
  return (v4_movemask(sep) & 7) != 0;
}


// scalar global AABB filter (avoids YMM stack alignment in scalar callers)
static inline int filterBoxScalar(const mjtNum aabb1[6], const mjtNum aabb2[6], mjtNum margin) {
  if (aabb1[0]+aabb1[3]+margin < aabb2[0]-aabb2[3]) return 1;
  if (aabb1[1]+aabb1[4]+margin < aabb2[1]-aabb2[4]) return 1;
  if (aabb1[2]+aabb1[5]+margin < aabb2[2]-aabb2[5]) return 1;
  if (aabb2[0]+aabb2[3]+margin < aabb1[0]-aabb1[3]) return 1;
  if (aabb2[1]+aabb2[4]+margin < aabb1[1]-aabb1[4]) return 1;
  if (aabb2[2]+aabb2[5]+margin < aabb1[2]-aabb1[5]) return 1;
  return 0;
}


// filter contact based sphere-box test, treating sphere as box
static inline int filterSphereBox(const mjtNum s[3], mjtNum bound, const mjtNum aabb[6]) {
  if (s[0]+bound < aabb[0]-aabb[3]) return 1;
  if (s[1]+bound < aabb[1]-aabb[4]) return 1;
  if (s[2]+bound < aabb[2]-aabb[5]) return 1;
  if (s[0]-bound > aabb[0]+aabb[3]) return 1;
  if (s[1]-bound > aabb[1]+aabb[4]) return 1;
  if (s[2]-bound > aabb[2]+aabb[5]) return 1;
  return 0;
}


// return 1 if speculative collisions are enabled
static inline int speculativeEnabled(const mjModel* m) {
  return mjENABLED(mjENBL_SPECULATIVE) && !mjDISABLED(mjDSBL_NATIVECCD);
}


// return minimum AABB half-thickness of geom g
static inline mjtNum geomMinSize(const mjModel* m, int g) {
  const mjtNum* size = m->geom_aabb + 6*g + 3;
  return mju_min(size[0], mju_min(size[1], size[2]));
}


// return minimum half-thickness across a geom pair
static inline mjtNum pairMinSize(const mjModel* m, int g1, int g2) {
  if (m->geom_type[g1] == mjGEOM_PLANE) return geomMinSize(m, g2);
  if (m->geom_type[g2] == mjGEOM_PLANE) return geomMinSize(m, g1);
  return mju_min(geomMinSize(m, g1), geomMinSize(m, g2));
}


// return step sweep distance if geom g moves faster than min_size
static mjtNum geomStepSweep(const mjModel* m, const mjData* d, int g, mjtNum min_size,
                            mjtNum dx[3], mjtNum dw[3]) {
  if (dx) mji_zero3(dx);
  if (dw) mji_zero3(dw);
  if (g < 0 || m->geom_type[g] == mjGEOM_PLANE || !mjc_GeomSupported(m->geom_type[g])) {
    return 0;
  }
  mjtNum dx_copy[3], dw_copy[3];
  mjtNum s = mjc_geomStepMotion(m, d, g, m->opt.timestep, dx_copy, dw_copy);

  // if the geom is moving slower than min_size, we do not need to consider it
  // for continuous collision detection
  if (s < min_size) {
    return 0;
  }

  if (dx) mji_copy3(dx, dx_copy);
  if (dw) mji_copy3(dw, dw_copy);
  return s;
}


// return 1 if any geom in body pair (b1, b2) sweeps past the pair's minimum geom half-thickness
static int bodyPairHasFastGeom(const mjModel* m, const mjData* d, int b1, int b2) {
  int adr1 = m->body_geomadr[b1], num1 = m->body_geomnum[b1];
  int adr2 = m->body_geomadr[b2], num2 = m->body_geomnum[b2];
  mjtNum min_size = mjMAXVAL;
  for (int i = 0; i < num1; i++) {
    int g = adr1 + i;
    if ((!mjcb_contactfilter && filterGeomBody(m, g, b2)) ||
        m->geom_type[g] == mjGEOM_PLANE || !mjc_GeomSupported(m->geom_type[g])) {
      continue;
    }
    min_size = mju_min(min_size, geomMinSize(m, g));
  }
  for (int i = 0; i < num2; i++) {
    int g = adr2 + i;
    if ((!mjcb_contactfilter && filterGeomBody(m, g, b1)) ||
        m->geom_type[g] == mjGEOM_PLANE || !mjc_GeomSupported(m->geom_type[g])) {
      continue;
    }
    min_size = mju_min(min_size, geomMinSize(m, g));
  }
  if (min_size >= mjMAXVAL) {
    return 0;
  }
  for (int i = 0; i < num1; i++) {
    int g = adr1 + i;
    if ((mjcb_contactfilter || !filterGeomBody(m, g, b2)) &&
        geomStepSweep(m, d, g, min_size, NULL, NULL) > 0) {
      return 1;
    }
  }
  for (int i = 0; i < num2; i++) {
    int g = adr2 + i;
    if ((mjcb_contactfilter || !filterGeomBody(m, g, b1)) &&
        geomStepSweep(m, d, g, min_size, NULL, NULL) > 0) {
      return 1;
    }
  }
  return 0;
}


// filter contact based on static bounding sphere test
static inline int filterSphereStatic(const mjModel* m, const mjData* d,
                                     int g1, int g2, mjtNum margin) {
  mjtNum r1 = m->geom_rbound[g1];
  mjtNum r2 = m->geom_rbound[g2];

  // neither geom is a plane
  if (r1 > 0 && r2 > 0) {
    mjtNum bound = r1 + r2 + margin;
    mjtNum dif[3];
    mji_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
    return mju_dot3(dif, dif) > bound*bound;
  }

  // one geom is a plane
  if (m->geom_type[g1] == mjGEOM_PLANE && r2 > 0) {
    return planeGeomDist(m, d, g1, g2) > margin + r2;
  }
  if (m->geom_type[g2] == mjGEOM_PLANE && r1 > 0) {
    return planeGeomDist(m, d, g2, g1) > margin + r1;
  }
  return 0;
}


// filter contact based on bounding sphere test (swept over step for fast geoms)
static int mj_filterSphere(const mjModel* m, mjData* d, int g1, int g2, mjtNum margin) {
  // static bounding sphere test when speculative collisions are disabled
  if (!speculativeEnabled(m)) {
    return filterSphereStatic(m, d, g1, g2, margin);
  }

  // if speculative collisions are enabled, compute the step sweep distance for each geom
  mjtNum dx1[3], dx2[3];
  mjtNum min_size = pairMinSize(m, g1, g2);
  mjtNum s1 = geomStepSweep(m, d, g1, min_size, dx1, NULL);
  mjtNum s2 = geomStepSweep(m, d, g2, min_size, dx2, NULL);

  // no sweeping motion needed, check distance between centers
  if (s1 == 0 && s2 == 0) {
    return filterSphereStatic(m, d, g1, g2, margin);
  }

  // neither geom is a plane
  if (m->geom_rbound[g1] > 0 && m->geom_rbound[g2] > 0) {
    mjtNum bound = m->geom_rbound[g1] + m->geom_rbound[g2] + margin;
    const mjtNum* pos1 = d->geom_xpos + 3*g1;
    const mjtNum* pos2 = d->geom_xpos + 3*g2;
    mjtNum diff[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};

    // find closest approach time tau in [0, 1] (fraction of timestep) and check against
    // combined radius
    mjtNum v[3], closest[3];
    mju_sub3(v, dx2, dx1);
    mjtNum vv = mju_dot3(v, v);

    // tau is the minimum of |diff + t*v|^2 on the line segment [0, 1]
    mjtNum tau = (vv > mjMINVAL) ? mju_clip(-mju_dot3(diff, v) / vv, 0.0, 1.0) : 0.0;
    mji_addScl3(closest, diff, v, tau);
    return mju_dot3(closest, closest) > bound * bound;
  }

  // one geom is a plane
  if (m->geom_type[g1] == mjGEOM_PLANE && m->geom_rbound[g2] > 0) {
    mjtNum dist = planeGeomDist(m, d, g1, g2);
    if (s2 > 0) {
      const mjtNum* mat1 = d->geom_xmat + 9*g1;
      mjtNum n1[3] = {mat1[2], mat1[5], mat1[8]};
      dist = mju_min(dist, dist + mju_dot3(n1, dx2));
    }
    if (dist > margin + m->geom_rbound[g2]) {
      return 1;
    }
  }
  if (m->geom_type[g2] == mjGEOM_PLANE && m->geom_rbound[g1] > 0) {
    mjtNum dist = planeGeomDist(m, d, g2, g1);
    if (s1 > 0) {
      const mjtNum* mat2 = d->geom_xmat + 9*g2;
      mjtNum n2[3] = {mat2[2], mat2[5], mat2[8]};
      dist = mju_min(dist, dist + mju_dot3(n2, dx1));
    }
    if (dist > margin + m->geom_rbound[g1]) {
      return 1;
    }
  }
  return 0;
}


// filter body pair; 1: discard, 0: proceed
static int filterBodyPair(const mjModel* m, const mjData* d, int b1, int b2,
                          int dsbl_filterparent, int sleep_filter) {
  int weldbody1 = m->body_weldid[b1];
  int weldbody2 = m->body_weldid[b2];

  // same weldbody check
  if (weldbody1 == weldbody2) {
    return 1;
  }

  // both dof-less: no forces can act, skip
  if (m->body_dofnum[weldbody1] == 0 && m->body_dofnum[weldbody2] == 0) {
    return 1;
  }

  // sleep checks
  if (sleep_filter) {
    int asleep1 = (d->body_awake[b1] == mjS_ASLEEP);
    int asleep2 = (d->body_awake[b2] == mjS_ASLEEP);

    // both asleep check
    if (asleep1 && asleep2) {
      return 1;
    }

    // asleep and world-static check (mocap has weldbody != 0, still triggers wake)
    if ((asleep1 && !weldbody2) || (asleep2 && !weldbody1)) {
      return 1;
    }
  }

  // weldparent check
  if (!dsbl_filterparent && weldbody1 != 0 && weldbody2 != 0) {
    int weldparent1 = m->body_weldid[m->body_parentid[weldbody1]];
    int weldparent2 = m->body_weldid[m->body_parentid[weldbody2]];
    if (weldbody1 == weldparent2 || weldbody2 == weldparent1) {
      return 1;
    }
  }

  // all tests passed
  return 0;
}


// return 1 if bodyflex has dofs and they are awake, 0 otherwise
static int hasAwakeDofs(const mjModel* m, const mjData* d, int bf, int sleep_filter) {
  // body
  if (bf < m->nbody) {
    return m->body_treeid[bf] >= 0 && (!sleep_filter || d->body_awake[bf] == mjS_AWAKE);
  }

  // flex: static if none of its vertices (or nodes) are in a body with dofs
  mjtSleepState state = mj_sleepState(m, d, mjOBJ_FLEX, bf - m->nbody);
  return state != mjS_STATIC && (!sleep_filter || state == mjS_AWAKE);
}


// return 1 if element is active, 0 otherwise
int mj_isElemActive(const mjModel* m, int f, int e) {
  if (m->flex_dim[f] < 3) {
    return 1;
  } else {
    return (m->flex_elemlayer[m->flex_elemadr[f]+e] < m->flex_activelayers[f]);
  }
}


//----------------------------- collision detection entry point ------------------------------------

// internal structure for storing pairs meant for narrowphase collision detection
typedef struct {
  // type of collision pair
  enum {
    mjCPAIR_GEOM_GEOM = 0,  // mjCOLLISIONFUNC[type1][type2]
    mjCPAIR_GEOM_FLEX,      // mjc_PlaneFlex, mjc_FlexSDF
    mjCPAIR_GEOM_ELEM,      // mjc_SubGeomElem
    mjCPAIR_ELEM_ELEM,      // mjc_ElemElem
  } type;
  int conpos;
  int group;                // filter group (-1: no filtering)
  union {
    struct { int g1, g2, ipair, spec, sub_g1, sub_g2; } geom_geom;
    struct { int g, f; } geom_flex;
    struct { int g, f, e, sub_g; } geom_elem;
    struct { int f1, e1, f2, e2; } elem_elem;
  };
} mjcPair;


// stack-allocated accumulator for candidate pairs pushed onto the arena
typedef struct {
  size_t parena;  // byte offset in d->arena where mjcPair[] begins
  int npair;      // total candidate pairs attempted (may exceed arena on overflow)
  int ngroup;     // next filter group id
} mjcPairArena;


// initialize a default collision pair
static inline void defaultPair(mjcPair* pair, int type) {
  memset(pair, -1, sizeof(*pair));
  pair->type = type;
  if (type == mjCPAIR_GEOM_GEOM) {
    pair->geom_geom.spec = 0;
  }
}


// collision tree node
typedef struct {
  int node1;
  int node2;
} mjCollisionTree;

// binary search between two bodyflex trees
static void mj_collideTree(const mjModel* m, mjData* d, mjcPairArena* arena, int bf1, int bf2,
                           int merged, int startadr, int pairadr, int group,
                           mjCollisionTree* stack, int max_stack);

// struct for storing pair of 16-bit unsigned integers
typedef struct {
  uint16_t hi, lo;
} mjPacked32;

// broadphase collision detection; return number of bodyflex pairs
static int mj_broadphase(const mjModel* m, mjData* d, mjPacked32* bfpair, int maxpair);

// compute contacts for a batch of collision pairs on the arena starting at parena
static void mj_narrowphase(const mjModel* m, mjData* d, int npair, size_t parena);

// test active element self-collisions with SAP, pushing pairs onto the arena
static void mj_collideFlexSAPPairs(const mjModel* m, mjData* d, mjcPairArena* arena,
                                   int f, int group);

// test active element self-collisions of an interpolated flex by grouping elements into cells
static void mj_collideFlexCellPairs(const mjModel* m, mjData* d, mjcPairArena* arena,
                                    int f, int group);

// get colliding object IDs for a collision pair
static inline void pairObjects(const mjcPair* p, int* obj1, int* obj2) {
  switch (p->type) {
    case mjCPAIR_GEOM_GEOM:
      *obj1 = p->geom_geom.g1;
      *obj2 = p->geom_geom.g2;
      break;

    case mjCPAIR_GEOM_FLEX:
      *obj1 = p->geom_flex.g;
      *obj2 = p->geom_flex.f;
      break;

    case mjCPAIR_GEOM_ELEM:
      *obj1 = p->geom_elem.g;
      *obj2 = p->geom_elem.e;
      break;

    case mjCPAIR_ELEM_ELEM:
      *obj1 = p->elem_elem.e1;
      *obj2 = p->elem_elem.e2;
      break;

    default:
      *obj1 = -1;
      *obj2 = -1;
      break;
  }
}


// compare two collision pairs for ordering
static inline int pairCompare(const mjcPair* p1, const mjcPair* p2, void* context) {
  (void) context;

  // keep pairs in the same group contiguous and groups strictly increasing (-1 first)
  if (p1->group != p2->group) {
    return p1->group < p2->group ? -1 : 1;
  }

  // sort by type
  if (p1->type != p2->type) {
    return p1->type < p2->type ? -1 : 1;
  }

  // sort by colliding objects
  int obj1_1, obj1_2, obj2_1, obj2_2;
  pairObjects(p1, &obj1_1, &obj1_2);
  pairObjects(p2, &obj2_1, &obj2_2);

  if (obj1_1 < obj2_1) return -1;
  if (obj1_1 > obj2_1) return 1;
  if (obj1_2 < obj2_2) return -1;
  if (obj1_2 > obj2_2) return 1;

  // tie break on subgeom indices
  if (p1->type == mjCPAIR_GEOM_GEOM) {
    int sub_g1_1 = p1->geom_geom.sub_g1, sub_g1_2 = p1->geom_geom.sub_g2;
    int sub_g2_1 = p2->geom_geom.sub_g1, sub_g2_2 = p2->geom_geom.sub_g2;
    if (sub_g1_1 != sub_g2_1) return sub_g1_1 < sub_g2_1 ? -1 : 1;
    if (sub_g1_2 != sub_g2_2) return sub_g1_2 < sub_g2_2 ? -1 : 1;
  } else if (p1->type == mjCPAIR_GEOM_ELEM) {
    int sub_g1 = p1->geom_elem.sub_g, sub_g2 = p2->geom_elem.sub_g;
    if (sub_g1 != sub_g2) return sub_g1 < sub_g2 ? -1 : 1;
  }
  return 0;
}

// define pairSort function for sorting collision pairs
mjSORT(pairSort, mjcPair, pairCompare);




// push a candidate collision pair onto the arena
static void pushPairArena(mjData* d, mjcPairArena* arena, const mjcPair* pair) {
  // allocate geom pair on the arena
  mjcPair* new_pair = (mjcPair*) mj_arenaAllocByte(d, sizeof(mjcPair), _Alignof(mjcPair));
  if (new_pair) {
    *new_pair = *pair;
  }
  arena->npair++;
}


// assign a shared filter group to arena pairs from start to the current end when >= 2 pairs exist
static inline void assignGroup(mjData* d, mjcPairArena* arena, int start) {
  int end = (int)((d->parena - arena->parena) / sizeof(mjcPair));
  if (end - start >= 2) {
    int grp = arena->ngroup++;
    mjcPair* pairs = (mjcPair*)((char*)d->arena + arena->parena);
    for (int k = start; k < end; k++) {
      pairs[k].group = grp;
    }
  }
}


// return 1 if allocating up to n pairs fits within 25% of remaining free arena + narrowphase stack
static inline int canAllocPairs(const mjData* d, const mjcPairArena* arena, size_t n) {
  if (!n) {
    return 1;
  }
  size_t free_bytes = (d->parena + d->pstack < d->narena)
                      ? (d->narena - d->pstack - d->parena) : 0;

  // narrowphase copies pairs onto the stack while arena is live, then allocates ncon + con buffers
  size_t stack_per_pair = sizeof(mjcPair) + sizeof(int) + sizeof(mjPreContact);
  size_t existing_stack = (size_t)arena->npair * stack_per_pair;
  if (existing_stack >= free_bytes) {
    return 0;
  }

  // cap any single expansion to 25% of remaining net space so unrelated pairs and stack stay safe
  size_t budget = (free_bytes - existing_stack) / 4;
  size_t per_pair = sizeof(mjcPair) + stack_per_pair;
  return n <= budget / per_pair;
}


// push a geom-geom collision pair (or expand piece pairs) onto the arena
static void pushGeomGeom(const mjModel* m, mjData* d, mjcPairArena* arena,
                         int g1, int g2, int ipair) {
  if (m->geom_type[g1] > m->geom_type[g2]) {
    int tmp = g1;
    g1 = g2;
    g2 = tmp;
  }

  int type1 = m->geom_type[g1];
  int type2 = m->geom_type[g2];

  mjcPair pair;
  defaultPair(&pair, mjCPAIR_GEOM_GEOM);
  pair.geom_geom.g1 = g1;
  pair.geom_geom.g2 = g2;
  pair.geom_geom.ipair = ipair;
  pair.geom_geom.spec = 0;

  // expand heightfield vs convex geom into virtual triangular prism piece pairs
  if (mjCOLLISIONFUNC[type1][type2] == mjc_ConvexHField) {
    mjtNum margin = getMargin(m, g1, g2, ipair) + getGap(m, g1, g2, ipair);
    int rmin, rmax, cmin, cmax;
    mjtNum zmin;
    if (!mjc_hfieldSubgridGeom(m, d, g1, g2, margin, &rmin, &rmax, &cmin, &cmax, &zmin)) {
      return;
    }
    size_t max_prism = 2 * (size_t)(rmax - rmin) * (size_t)(cmax - cmin);
    if (!canAllocPairs(d, arena, max_prism)) {
      // fall back to single unexpanded pair (serial mjc_ConvexHField) to avoid arena/stack blowup
      pushPairArena(d, arena, &pair);
      return;
    }
    int start = (int)((d->parena - arena->parena) / sizeof(mjcPair));
    int hid = m->geom_dataid[g1];
    int ncol = m->hfield_ncol[hid];
    mjtNum size2 = m->hfield_size[4*hid + 2];
    const float* hdata = m->hfield_data + m->hfield_adr[hid];

    for (int r = rmin; r < rmax; r++) {
      for (int c = cmin; c < cmax; c++) {
        int mask = mjc_hfieldCellMask(hdata, ncol, size2, r, c, zmin);
        if (mask & 1) {
          pair.geom_geom.sub_g1 = mjc_hfieldPrismId(r, c, 0, ncol - 1);
          pushPairArena(d, arena, &pair);
        }
        if (mask & 2) {
          pair.geom_geom.sub_g1 = mjc_hfieldPrismId(r, c, 1, ncol - 1);
          pushPairArena(d, arena, &pair);
        }
      }
    }
    assignGroup(d, arena, start);
    return;
  }

  pushPairArena(d, arena, &pair);
}


// flex with few dofs (rigid or interpolated): its contacts can be reduced to mjMAXCONPAIR
static inline int flexReducible(const mjModel* m, int f) {
  return m->flex_rigid[f] || m->flex_interp[f];
}


// filter group of geom g in a body with first geom geomadr (-1: no filtering)
static inline int geomGroup(int group, int g, int geomadr) {
  return group < 0 ? -1 : group + g - geomadr;
}


// push a geom-flex collision pair onto the arena
static void pushGeomFlex(mjData* d, mjcPairArena* arena, int g, int f, int group) {
  mjcPair pair;
  defaultPair(&pair, mjCPAIR_GEOM_FLEX);
  pair.group = group;
  pair.geom_flex.g = g;
  pair.geom_flex.f = f;
  pushPairArena(d, arena, &pair);
}


// push a geom-elem collision pair (or expand piece pairs) onto the arena
static void pushGeomElem(const mjModel* m, mjData* d, mjcPairArena* arena,
                         int g, int f, int e, int group) {
  mjcPair pair;
  defaultPair(&pair, mjCPAIR_GEOM_ELEM);
  pair.group = group;
  pair.geom_elem.g = g;
  pair.geom_elem.f = f;
  pair.geom_elem.e = e;
  pair.geom_elem.sub_g = -1;

  // expand heightfield vs flex element into virtual triangular prism piece pairs
  if (m->geom_type[g] == mjGEOM_HFIELD) {
    // skip if element has a vertex on the same body as the heightfield
    int b = m->geom_bodyid[g];
    int dim = m->flex_dim[f];
    const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim + 1);
    const int* bdata = m->flex_vertbodyid + m->flex_vertadr[f];
    for (int i = 0; i <= dim; i++) {
      if (b >= 0 && b == bdata[edata[i]]) {
        return;
      }
    }

    mjtNum dist_bound = mj_assignMargin(m, m->geom_margin[g] + m->flex_margin[f]) +
                        m->geom_gap[g] + m->flex_gap[f];
    int rmin, rmax, cmin, cmax;
    mjtNum zmin;
    if (!mjc_hfieldSubgridElem(m, d, g, f, e, dist_bound, &rmin, &rmax, &cmin, &cmax, &zmin)) {
      return;
    }
    size_t max_prism = 2 * (size_t)(rmax - rmin) * (size_t)(cmax - cmin);
    if (!canAllocPairs(d, arena, max_prism)) {
      // fall back to single unexpanded pair (serial hfield-elem loop) to avoid arena/stack blowup
      pushPairArena(d, arena, &pair);
      return;
    }
    int start = (int)((d->parena - arena->parena) / sizeof(mjcPair));
    int hid = m->geom_dataid[g];
    int ncol = m->hfield_ncol[hid];
    mjtNum size2 = m->hfield_size[4*hid + 2];
    const float* hdata = m->hfield_data + m->hfield_adr[hid];

    for (int r = rmin; r < rmax; r++) {
      for (int c = cmin; c < cmax; c++) {
        int mask = mjc_hfieldCellMask(hdata, ncol, size2, r, c, zmin);
        if (mask & 1) {
          pair.geom_elem.sub_g = mjc_hfieldPrismId(r, c, 0, ncol - 1);
          pushPairArena(d, arena, &pair);
        }
        if (mask & 2) {
          pair.geom_elem.sub_g = mjc_hfieldPrismId(r, c, 1, ncol - 1);
          pushPairArena(d, arena, &pair);
        }
      }
    }
    if (group < 0) {
      assignGroup(d, arena, start);
    }
    return;
  }

  pushPairArena(d, arena, &pair);
}


// push an elem-elem collision pair onto the arena
static void pushElemElem(mjData* d, mjcPairArena* arena, int f1, int e1, int f2, int e2, int group) {
  mjcPair pair;
  defaultPair(&pair, mjCPAIR_ELEM_ELEM);
  pair.group = group;
  pair.elem_elem.f1 = f1;
  pair.elem_elem.e1 = e1;
  pair.elem_elem.f2 = f2;
  pair.elem_elem.e2 = e2;
  pushPairArena(d, arena, &pair);
}


// push an elem-elem self-collision pair of flex f onto the arena, unless it is excluded
static void pushSelfElemElem(const mjModel* m, mjData* d, mjcPairArena* arena,
                             int f, int e1, int e2, int group) {
  // interpolated flex: skip elements driven by a common grid node
  if (mjc_ElemSharedNode(m, f, e1, e2)) {
    return;
  }
  pushElemElem(d, arena, f, e1, f, e2, group);
}


// return 1 if (g1, g2) is in predefined pair range [startadr, pairadr), 0 otherwise
static inline int isPairMerged(const mjModel* m, int g1, int g2, int startadr, int pairadr) {
  for (int k=startadr; k < pairadr; k++) {
    if ((m->pair_geom1[k] == g1 && m->pair_geom2[k] == g2) ||
        (m->pair_geom1[k] == g2 && m->pair_geom2[k] == g1)) {
      return 1;
    }
  }
  return 0;
}


// filter candidate collision pair; return 0 if pair should be discarded
static int filterCollisionPair(const mjModel* m, mjData* d, int g1, int g2, int ipair,
                               int merged, int startadr, int pairadr) {
  // merged, find matching pair
  if (merged && isPairMerged(m, g1, g2, startadr, pairadr)) {
    return 0;
  }

  if (ipair >= 0) {
    if (mjENABLED(mjENBL_SLEEP)) {
      int b1 = m->geom_bodyid[g1];
      int b2 = m->geom_bodyid[g2];
      if (d->body_awake[b1] != mjS_AWAKE && d->body_awake[b2] != mjS_AWAKE) {
        return 0;
      }
    }
  }

  if (ipair < 0) {
    if (mjcb_contactfilter) {
      if (mj_callContactFilter(m, d, g1, g2)) {
        return 0;
      }
    } else if (filterGeomPair(m, g1, g2)) {
      return 0;
    }
  }

  // check collision function is well-defined
  int type1 = mjMIN(m->geom_type[g1], m->geom_type[g2]);
  int type2 = mjMAX(m->geom_type[g1], m->geom_type[g2]);
  if (!mjCOLLISIONFUNC[type1][type2]) {
    return 0;
  }

  // bounding sphere filter
  mjtNum margin = getMargin(m, g1, g2, ipair);
  mjtNum gap = getGap(m, g1, g2, ipair);
  return !mj_filterSphere(m, d, g1, g2, margin + gap);
}


// return single colliding geom id of body b if it has at most one, -1 otherwise
static inline int singleBodyGeom(const mjModel* m, int b) {
  if (m->body_geomnum[b] == 1) {
    return m->body_geomadr[b];
  }
  if (!mjDISABLED(mjDSBL_MIDPHASE) && !m->vis.global.bvactive &&
      !mjcb_contactfilter && m->body_bvhnum[b] == 1) {
    return m->bvh_nodeid[m->body_bvhadr[b]];
  }
  return -1;
}


// main collision function
void mj_collision(const mjModel* m, mjData* d) {
  TM_START1;

  int nexclude = m->nexclude, npair = m->npair, nbody = m->nbody;
  int nbodyflex = m->nbody + m->nflex;
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < nbody;

  // reset the size of the contact array and invalidate efc arrays
  d->ncon = 0;
  resetArena(d);
  mj_clearEfc(d);

  // return if disabled
  if (mjDISABLED(mjDSBL_CONSTRAINT) || mjDISABLED(mjDSBL_CONTACT) || nbodyflex < 2) {
    TM_END1(mjTIMER_POS_COLLISION);
    return;
  }

  // reset the visualization flags
  if (m->vis.global.bvactive) {
    memset(d->bvh_active, 0, m->nbvh);
  }

  mj_markStack(d);

  // broadphase collision detector
  TM_START;
  int nmaxpairs = (nbodyflex*(nbodyflex - 1))/2;
  mjPacked32* broadphasepair = mjSTACKALLOC(d, nmaxpairs, mjPacked32);
  int nbfpair = mj_broadphase(m, d, broadphasepair, nmaxpairs);
  unsigned int last_signature = -1;
  TM_END(mjTIMER_COL_BROAD);

  // narrowphase and midphase collision detector
  TM_RESTART;

  // process bodyflex pairs returned by broadphase, merge with predefined geom pairs
  int pairadr = 0, exadr = 0, g1, g2;

  // align the arena for candidate collision pairs for narrowphase
  mjcPairArena arena = {alignArena(d, _Alignof(mjcPair)), 0, 0};

  // reusable traversal stack for body:body midphase (lazily allocated once per step)
  mjCollisionTree* body_stack = NULL;
  int body_max_stack = 0;

  for (int i=0; i < nbfpair; i++) {
    // reconstruct bodyflex pair ids
    int bf1 = broadphasepair[i].hi;
    int bf2 = broadphasepair[i].lo;

    // compute signature for this bodyflex pair
    unsigned int signature = ((unsigned int)bf1 << 16) + bf2;
    // pairs come sorted by signature, but may not be unique
    // if signature is repeated, skip it
    if (signature == last_signature) {
      continue;
    }
    last_signature = signature;

    // merge predefined geom pairs
    int merged = 0;
    int startadr = pairadr;
    // test all predefined pairs for which pair_signature <= signature
    for (; pairadr < npair && m->pair_signature[pairadr] <= signature; pairadr++) {
      merged = (m->pair_signature[pairadr] == signature);
      g1 = m->pair_geom1[pairadr], g2 = m->pair_geom2[pairadr];
      if (filterCollisionPair(m, d, g1, g2, pairadr, 0, 0, 0)) {
        pushGeomGeom(m, d, &arena, g1, g2, pairadr);
      }
    }

    // under the ipc flag the IPC step resolves two dim-2 flexes itself
    if (bf1 >= nbody && bf2 >= nbody && mjc_ipcOwnsFlexFlex(m, bf1 - nbody, bf2 - nbody)) {
      continue;
    }

    // handle body pair exclusion
    if (nexclude) {
      // advance exadr while exclude_signature < signature
      while (exadr < nexclude && (unsigned int)m->exclude_signature[exadr] < signature) {
        exadr++;
      }

      // skip this bodyflex pair if its signature is found in exclude array
      if (exadr < nexclude && (unsigned int)m->exclude_signature[exadr] == signature) {
        continue;
      }
    }

    // get bodyflex info
    int isbody1 = (bf1 < nbody);
    int isbody2 = (bf2 < nbody);
    int bvh1 = (isbody1 ? m->body_bvhadr[bf1] : m->flex_bvhadr[bf1-nbody]);
    int bvh2 = (isbody2 ? m->body_bvhadr[bf2] : m->flex_bvhadr[bf2-nbody]);
    int geomadr1 = (isbody1 ? m->body_geomadr[bf1] : -1);
    int geomadr2 = (isbody2 ? m->body_geomadr[bf2] : -1);

    // process bodyflex pair: two single-colliding-geom bodies
    int cg1 = isbody1 ? singleBodyGeom(m, bf1) : -1;
    int cg2 = isbody2 ? singleBodyGeom(m, bf2) : -1;
    if (cg1 >= 0 && cg2 >= 0) {
      if (filterCollisionPair(m, d, cg1, cg2, -1, merged, startadr, pairadr)) {
        pushGeomGeom(m, d, &arena, cg1, cg2, -1);
      }
    }

    // process bodyflex pair: midphase
    else if (!mjDISABLED(mjDSBL_MIDPHASE) && bvh1 >= 0 && bvh2 >= 0 &&
             !(speculativeEnabled(m) && isbody1 && isbody2 &&
               bodyPairHasFastGeom(m, d, bf1, bf2))) {
      // lazily allocate reusable stack sized for the two largest body BVHs
      if (isbody1 && isbody2 && !body_stack) {
        // TODO(b/273737633): Store bvh max depths to make this bound tighter.
        int top1 = 0, top2 = 0;
        for (int b=0; b < nbody; b++) {
          int bn = m->body_bvhnum[b];
          if (bn >= top1) {
            top2 = top1;
            top1 = bn;
          } else if (bn > top2) {
            top2 = bn;
          }
        }
        body_max_stack = top1 + top2;
        body_stack = mjSTACKALLOC(d, body_max_stack, mjCollisionTree);
      }

      // filter groups for reducible flexes: one per geom for body:flex, one for flex:flex
      int group = -1;
      if (isbody1 && !isbody2) {
        if (flexReducible(m, bf2-nbody)) {
          group = arena.ngroup;
          arena.ngroup += m->body_geomnum[bf1];
        }
      } else if (!isbody1 && flexReducible(m, bf1-nbody) &&
                 flexReducible(m, bf2-nbody)) {
        group = arena.ngroup++;
      }
      mj_collideTree(m, d, &arena, bf1, bf2, merged, startadr, pairadr, group,
                     isbody1 && isbody2 ? body_stack : NULL,
                     isbody1 && isbody2 ? body_max_stack : 0);
    }

    // process bodyflex pair: all-to-all
    else {
      int geomadr_end1 = (isbody1 ? geomadr1 + m->body_geomnum[bf1] : -1);
      int geomadr_end2 = (isbody2 ? geomadr2 + m->body_geomnum[bf2] : -1);

      // body : body
      if (isbody1 && isbody2) {
        for (g1=geomadr1; g1 < geomadr_end1; g1++) {
          if (!mjcb_contactfilter && filterGeomBody(m, g1, bf2)) {
            continue;
          }
          for (g2=geomadr2; g2 < geomadr_end2; g2++) {
            if (!mjcb_contactfilter && filterGeomPair(m, g1, g2)) {
              continue;
            }
            if (filterCollisionPair(m, d, g1, g2, -1, merged, startadr, pairadr)) {
              pushGeomGeom(m, d, &arena, g1, g2, -1);
            }
          }
        }
      }

      // body : flex
      else if (isbody1) {
        int f = bf2 - nbody;

        // process body geoms
        for (int g=m->body_geomadr[bf1]; g < geomadr_end1; g++) {
          // bitmask filtering at the geom-flex level
          if (filterGeomFlex(m, g, f)) {
            continue;
          }

          if (mjc_ipcOwnsFlexGeom(m, f, g)) {
            continue;
          }

          int group = flexReducible(m, f) ? arena.ngroup++ : -1;

          // plane special processing
          if (m->geom_type[g] == mjGEOM_PLANE) {
            pushGeomFlex(d, &arena, g, f, group);
            continue;
          }

          // SDF special processing
          if (m->geom_type[g] == mjGEOM_SDF) {
            if (m->flex_dim[f] == 2) {
              pushGeomFlex(d, &arena, g, f, group);
            }
            continue;
          }

          // collide geom with flex elements
          int elemnum = m->flex_elemnum[f];
          for (int e=0; e < elemnum; e++) {
            if (mj_isElemActive(m, f, e)) {
              pushGeomElem(m, d, &arena, g, f, e, group);
            }
          }
        }
      }

      // flex : flex
      else {
        int f1 = bf1 - nbody;
        int f2 = bf2 - nbody;
        int group = (flexReducible(m, f1) && flexReducible(m, f2)) ? arena.ngroup++ : -1;

        // collide elements of two flexes
        for (int e1=0; e1 < m->flex_elemnum[f1]; e1++) {
          if (mj_isElemActive(m, f1, e1)) {
            for (int e2=0; e2 < m->flex_elemnum[f2]; e2++) {
              if (mj_isElemActive(m, f2, e2)) {
                pushElemElem(d, &arena, f1, e1, f2, e2, group);
              }
            }
          }
        }
      }
    }
  }
  mj_freeStack(d);

  // finish merging predefined geom pairs
  for (; pairadr < npair; pairadr++) {
    g1 = m->pair_geom1[pairadr], g2 = m->pair_geom2[pairadr];
    if (filterCollisionPair(m, d, g1, g2, pairadr, 0, 0, 0)) {
      pushGeomGeom(m, d, &arena, g1, g2, pairadr);
    }
  }

  // flex self-collisions
  for (int f=0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && (m->flex_contype[f] & m->flex_conaffinity[f])) {
      // skip if flex has no dofs or is asleep
      if (!hasAwakeDofs(m, d, nbody+f, sleep_filter)) continue;
      // under the ipc flag the IPC step resolves a dim-2 flex's self-contact itself
      if (mjc_ipcOwnsFlexFlex(m, f, f)) continue;

      // active element collisions; for an interpolated flex with at most 2 cells along every
      // axis, all cells are neighbors, so every pair is excluded by mjc_ElemSharedNode
      const int* cellnum = m->flex_cellnum + 3*f;
      int all_excluded = m->flex_interp[f] && cellnum[0] <= 2 && cellnum[1] <= 2 &&
                         cellnum[2] <= 2;
      if (m->flex_selfcollide[f] != mjFLEXSELF_NONE && !all_excluded) {
        int group = flexReducible(m, f) ? arena.ngroup++ : -1;

        // element-element: midphase
        if (!mjDISABLED(mjDSBL_MIDPHASE) &&
            m->flex_selfcollide[f] != mjFLEXSELF_NARROW &&
            (m->flex_bvhadr[f] >= 0 ||
             (m->flex_selfcollide[f] == mjFLEXSELF_AUTO && m->flex_interp[f]))) {
          // select midphase mode
          if (m->flex_selfcollide[f] == mjFLEXSELF_AUTO && m->flex_interp[f]) {
            mj_collideFlexCellPairs(m, d, &arena, f, group);
          } else if (m->flex_selfcollide[f] == mjFLEXSELF_BVH ||
                     (m->flex_selfcollide[f] == mjFLEXSELF_AUTO && m->flex_dim[f] == 3)) {
            mj_collideTree(m, d, &arena, nbody+f, nbody+f, 0, 0, 0, group, NULL, 0);
          } else {
            mj_collideFlexSAPPairs(m, d, &arena, f, group);
          }
        }

        // element-element: direct
        else {
          int flex_elemnum = m->flex_elemnum[f];
          for (int e1=0; e1 < flex_elemnum; e1++) {
            if (mj_isElemActive(m, f, e1)) {
              for (int e2=e1+1; e2 < flex_elemnum; e2++) {
                if (mj_isElemActive(m, f, e2)) {
                  pushSelfElemElem(m, d, &arena, f, e1, e2, group);
                }
              }
            }
          }
        }
      }
    }
  }

  // warn if candidate pairs overflowed the arena
  int npair_arena = (int)((d->parena - arena.parena) / sizeof(mjcPair));
  if (arena.npair > npair_arena) {
    mj_warning(d, mjWARN_CONTACTFULL, arena.npair);
  }

  // narrowphase: compute contacts for all candidate pairs in a single global call
  if (npair_arena > 0) {
    mj_narrowphase(m, d, npair_arena, arena.parena);
  }

  // end narrowphase and midphase timer
  TM_END(mjTIMER_COL_NARROW);
  TM_END1(mjTIMER_POS_COLLISION);
}


//------------------------------------ binary tree search ------------------------------------------

// return 3-bit mask of infinite AABB half-sizes
static inline int infMaskAABB(const mjtNum aabb[6]) {
  return (aabb[3] >= mjMAXVAL) | ((aabb[4] >= mjMAXVAL) << 1) | ((aabb[5] >= mjMAXVAL) << 2);
}


// return 1 if all three half-sizes of aabb are infinite
static inline int isInfiniteAABB(const mjtNum aabb[6]) {
  return aabb[3] >= mjMAXVAL && aabb[4] >= mjMAXVAL && aabb[5] >= mjMAXVAL;
}


// compute res = xmat^T * v
static inline void mulMatTVec3(mjtNum res[3], const mjtNum xmat[9], const mjtNum v[3]) {
  res[0] = xmat[0]*v[0] + xmat[3]*v[1] + xmat[6]*v[2];
  res[1] = xmat[1]*v[0] + xmat[4]*v[1] + xmat[7]*v[2];
  res[2] = xmat[2]*v[0] + xmat[5]*v[1] + xmat[8]*v[2];
}


// precomputed relative transform between two OBB frames
typedef struct {
  mjtNum r[3][3];   // rows of R = R1^T * R2
  mjtNum ar[3][3];  // rows of |R|
  mjtNum t1[3];     // R1^T * (xpos2 - xpos1)
  mjtNum t2[3];     // R2^T * (xpos2 - xpos1)
} mjOBBFrame;


// precompute relative OBB frame between (xpos1, xmat1) and (xpos2, xmat2)
static inline void initOBBFrame(mjOBBFrame* f,
                                const mjtNum xpos1[3], const mjtNum xmat1[9],
                                const mjtNum xpos2[3], const mjtNum xmat2[9]) {
  // compute R = R1^T * R2
  if (xmat1 && xmat2) {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        f->r[i][j] = xmat1[0+i]*xmat2[0+j] + xmat1[3+i]*xmat2[3+j] + xmat1[6+i]*xmat2[6+j];
      }
    }
  } else if (xmat1) {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        f->r[i][j] = xmat1[3*j+i];
      }
    }
  } else if (xmat2) {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        f->r[i][j] = xmat2[3*i+j];
      }
    }
  } else {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        f->r[i][j] = (i == j);
      }
    }
  }

  // precompute |R|
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      f->ar[i][j] = mju_abs(f->r[i][j]);
    }
  }

  // compute t1 = R1^T * (xpos2 - xpos1) and t2 = R2^T * (xpos2 - xpos1)
  mjtNum dp[3] = {
    (xpos2 ? xpos2[0] : 0) - (xpos1 ? xpos1[0] : 0),
    (xpos2 ? xpos2[1] : 0) - (xpos1 ? xpos1[1] : 0),
    (xpos2 ? xpos2[2] : 0) - (xpos1 ? xpos1[2] : 0),
  };
  if (xmat1) {
    mulMatTVec3(f->t1, xmat1, dp);
  } else {
    mji_copy3(f->t1, dp);
  }
  if (xmat2) {
    mulMatTVec3(f->t2, xmat2, dp);
  } else {
    mji_copy3(f->t2, dp);
  }
}


// test separation along box 1's face normals: |t1 + R*c2 - c1| > s1 + |R|*s2 + margin
static inline int sepOBBAxes1(const mjtNum r[3][3], const mjtNum ar[3][3], const mjtNum t1[3],
                              const mjtNum aabb1[6], const mjtNum aabb2[6], mjtNum margin) {
  for (int i=0; i < 3; i++) {
    mjtNum rc2 = r[i][0]*aabb2[0] + r[i][1]*aabb2[1] + r[i][2]*aabb2[2];
    mjtNum dist = (t1[i] + rc2) - aabb1[i];
    mjtNum rs2 = ar[i][0]*aabb2[3] + ar[i][1]*aabb2[4] + ar[i][2]*aabb2[5];
    if (aabb1[3+i] + rs2 + margin < mju_abs(dist)) {
      return 1;
    }
  }
  return 0;
}


// test separation along box 2's face normals: |t2 - R^T*c1 + c2| > s2 + |R^T|*s1 + margin
static inline int sepOBBAxes2(const mjtNum r[3][3], const mjtNum ar[3][3], const mjtNum t2[3],
                              const mjtNum aabb1[6], const mjtNum aabb2[6], mjtNum margin) {
  for (int i=0; i < 3; i++) {
    mjtNum rtc1 = r[0][i]*aabb1[0] + r[1][i]*aabb1[1] + r[2][i]*aabb1[2];
    mjtNum dist = (t2[i] - rtc1) + aabb2[i];
    mjtNum rs1 = ar[0][i]*aabb1[3] + ar[1][i]*aabb1[4] + ar[2][i]*aabb1[5];
    if (aabb2[3+i] + rs1 + margin < mju_abs(dist)) {
      return 1;
    }
  }
  return 0;
}


// test two OBBs using precomputed relative frame f; return 1 if overlapping, 0 if separated
static inline int collideOBBFrame(const mjtNum aabb1[6], const mjtNum aabb2[6],
                                  mjtNum margin, const mjOBBFrame* f) {
  // get infinite dimensions (planes only)
  int mask1 = infMaskAABB(aabb1);
  int mask2 = infMaskAABB(aabb2);

  // if a bounding box is infinite, there must be a collision
  if (mask1 == 7 || mask2 == 7) {
    return 1;
  }

  // check intersections along box 1's face normals (skip if box 2 is infinite)
  if (!mask2 && sepOBBAxes1(f->r, f->ar, f->t1, aabb1, aabb2, margin)) {
    return 0;
  }

  // check intersections along box 2's face normals (skip if box 1 is infinite)
  if (!mask1 && sepOBBAxes2(f->r, f->ar, f->t2, aabb1, aabb2, margin)) {
    return 0;
  }

  return 1;
}


// test two OBBs with non-NULL poses; reuses f->r/f->ar if f != NULL, else computes them
static inline int collideOBBLeaf(const mjtNum aabb1[6], const mjtNum aabb2[6],
                                 const mjtNum xpos1[3], const mjtNum xmat1[9],
                                 const mjtNum xpos2[3], const mjtNum xmat2[9],
                                 mjtNum margin, const mjOBBFrame* f) {
  // get infinite dimensions (planes only)
  int mask1 = infMaskAABB(aabb1);
  int mask2 = infMaskAABB(aabb2);

  // if a bounding box is infinite, there must be a collision
  if (mask1 == 7 || mask2 == 7 || (mask1 && mask2)) {
    return 1;
  }

  mjOBBFrame local;
  if (!f) {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        local.r[i][j] = xmat1[0+i]*xmat2[0+j] + xmat1[3+i]*xmat2[3+j] + xmat1[6+i]*xmat2[6+j];
        local.ar[i][j] = mju_abs(local.r[i][j]);
      }
    }
    f = &local;
  }

  mjtNum dp[3] = {xpos2[0] - xpos1[0], xpos2[1] - xpos1[1], xpos2[2] - xpos1[2]};

  // check intersections along box 1's face normals (skip if box 2 is infinite)
  if (!mask2) {
    mjtNum t1[3];
    mulMatTVec3(t1, xmat1, dp);
    if (sepOBBAxes1(f->r, f->ar, t1, aabb1, aabb2, margin)) {
      return 0;
    }
  }

  // check intersections along box 2's face normals (skip if box 1 is infinite)
  if (!mask1) {
    mjtNum t2[3];
    mulMatTVec3(t2, xmat2, dp);
    if (sepOBBAxes2(f->r, f->ar, t2, aabb1, aabb2, margin)) {
      return 0;
    }
  }

  return 1;
}


// test an OBB (box 1) against a world-aligned AABB (box 2); return 1 if overlapping, 0 if separated
static inline int collideOBBAABB(const mjtNum aabb1[6], const mjtNum xpos1[3],
                                 const mjtNum xmat1[9], const mjtNum aabb2[6], mjtNum margin) {
  int mask1 = infMaskAABB(aabb1);
  int mask2 = infMaskAABB(aabb2);
  if (mask1 == 7 || mask2 == 7 || (mask1 && mask2)) {
    return 1;
  }

  // check intersections along box 1's face normals
  if (!mask2) {
    mjtNum dp[3] = {aabb2[0] - xpos1[0], aabb2[1] - xpos1[1], aabb2[2] - xpos1[2]};
    for (int i=0; i < 3; i++) {
      mjtNum m0 = xmat1[0+i], m1 = xmat1[3+i], m2 = xmat1[6+i];
      mjtNum dist = (m0*dp[0] + m1*dp[1] + m2*dp[2]) - aabb1[i];
      mjtNum rs2 = mju_abs(m0)*aabb2[3] + mju_abs(m1)*aabb2[4] + mju_abs(m2)*aabb2[5];
      if (aabb1[3+i] + rs2 + margin < mju_abs(dist)) {
        return 0;
      }
    }
  }

  // check intersections along box 2's world axes
  if (!mask1) {
    for (int i=0; i < 3; i++) {
      const mjtNum* row = xmat1 + 3*i;
      mjtNum wc1 = xpos1[i] + row[0]*aabb1[0] + row[1]*aabb1[1] + row[2]*aabb1[2];
      mjtNum rs1 = mju_abs(row[0])*aabb1[3] + mju_abs(row[1])*aabb1[4] + mju_abs(row[2])*aabb1[5];
      if (aabb2[3+i] + rs1 + margin < mju_abs(aabb2[i] - wc1)) {
        return 0;
      }
    }
  }

  return 1;
}


// oriented bounding boxes collision (see Gottschalk et al.)
int mj_collideOBB(const mjtNum aabb1[6], const mjtNum aabb2[6],
                  const mjtNum xpos1[3], const mjtNum xmat1[9],
                  const mjtNum xpos2[3], const mjtNum xmat2[9], mjtNum margin,
                  mjtNum product[36], mjtNum offset[12], mjtBool* initialize) {
  // uncached single-shot test
  if (product == NULL || offset == NULL) {
    if (xpos1 && xmat1 && xpos2 && xmat2) {
      return collideOBBLeaf(aabb1, aabb2, xpos1, xmat1, xpos2, xmat2, margin, NULL);
    }
    if (xpos1 && xmat1 && !xpos2 && !xmat2) {
      return collideOBBAABB(aabb1, xpos1, xmat1, aabb2, margin);
    }
    mjOBBFrame f;
    initOBBFrame(&f, xpos1, xmat1, xpos2, xmat2);
    return collideOBBFrame(aabb1, aabb2, margin, &f);
  }

  // initialize or load cached relative frame
  mjOBBFrame f;
  if (*initialize) {
    initOBBFrame(&f, xpos1, xmat1, xpos2, xmat2);
    for (int i=0; i < 3; i++) {
      mji_copy3(product + 0 + 3*i, f.r[i]);
      mji_copy3(product + 9 + 3*i, f.ar[i]);
    }
    mji_copy3(offset + 0, f.t1);
    mji_copy3(offset + 3, f.t2);
    *initialize = 0;
  } else {
    for (int i=0; i < 3; i++) {
      mji_copy3(f.r[i],  product + 0 + 3*i);
      mji_copy3(f.ar[i], product + 9 + 3*i);
    }
    mji_copy3(f.t1, offset + 0);
    mji_copy3(f.t2, offset + 3);
  }

  return collideOBBFrame(aabb1, aabb2, margin, &f);
}


// return 1 if body b's inertial frame shares orientation with its body frame
static inline int bodyMatchesInertiaRot(const mjModel* m, int b) {
  return b == 0 || m->body_sameframe[b] == mjSAMEFRAME_BODY ||
         m->body_sameframe[b] == mjSAMEFRAME_BODYROT;
}


// return 1 if geom g shares orientation with its body's inertial frame
static inline int geomMatchesInertiaRot(const mjModel* m, int g, int body_same_rot) {
  mjtByte gsf = m->geom_sameframe[g];
  return gsf == mjSAMEFRAME_INERTIA || gsf == mjSAMEFRAME_INERTIAROT ||
         (body_same_rot && (gsf == mjSAMEFRAME_BODY || gsf == mjSAMEFRAME_BODYROT));
}


// report BVH stack overflow
static mjNOINLINE void bvhStackError(void) {
  mjERROR("BVH stack depth exceeded.");  // SHOULD NOT OCCUR
}


// sort n > 1 candidate pairs starting at start_pair
static mjNOINLINE void sortTreePairs(const mjModel* m, mjData* d, mjcPairArena* arena,
                                     int start_pair, int n) {
  // insertion sort for small pair counts (avoids stack allocation)
  void* ctx = (void*)m;
  mjcPair* pairs = (mjcPair*) ((char*) d->arena + arena->parena) + start_pair;
  if (n <= 8) {
    for (int i=1; i < n; i++) {
      mjcPair tmp = pairs[i];
      int j = i - 1;
      while (j >= 0 && pairCompare(pairs + j, &tmp, ctx) > 0) {
        pairs[j+1] = pairs[j];
        j--;
      }
      pairs[j+1] = tmp;
    }
    return;
  }

  // merge sort for larger pair counts
  mj_markStack(d);
  mjcPair* buf = mjSTACKALLOC(d, n, mjcPair);
  pairSort(pairs, buf, n, ctx);
  mj_freeStack(d);
}


// binary search between two bodyflex trees
//  body:flex pairs of geom g are in filter group: group + g - body_geomadr[bf1]
static void mj_collideTree(const mjModel* m, mjData* d, mjcPairArena* arena, int bf1, int bf2,
                           int merged, int startadr, int pairadr, int group,
                           mjCollisionTree* stack, int max_stack) {
  int nbody = m->nbody, nbvhstatic = m->nbvhstatic;
  mjtBool isbody1 = (bf1 < nbody);
  mjtBool isbody2 = (bf2 < nbody);

  // flex : body (broadphase orders bf1 < bf2, so body always precedes flex)
  if (!isbody1 && isbody2) {
    mju_error("BVH flex:body pair, expected body:flex ordering");  // SHOULD NOT OCCUR
  }

  int f1 = isbody1 ? -1 : bf1 - nbody;
  int f2 = isbody2 ? -1 : bf2 - nbody;
  int geomadr1 = isbody1 ? m->body_geomadr[bf1] : -1;
  int mark_active = m->vis.global.bvactive;
  int has_filter = (mjcb_contactfilter != NULL);
  const int bvhadr1 = isbody1 ? m->body_bvhadr[bf1] : m->flex_bvhadr[f1];
  const int bvhadr2 = isbody2 ? m->body_bvhadr[bf2] : m->flex_bvhadr[f2];
  const int* child1 = m->bvh_child + 2*bvhadr1;
  const int* child2 = m->bvh_child + 2*bvhadr2;
  const int* nodeid_arr1 = m->bvh_nodeid + bvhadr1;
  const int* nodeid_arr2 = m->bvh_nodeid + bvhadr2;
  const mjtNum* bvh1 = isbody1 ? (m->bvh_aabb     + 6*bvhadr1) :
                                 (d->bvh_aabb_dyn + 6*(bvhadr1 - nbvhstatic));
  const mjtNum* bvh2 = isbody2 ? (m->bvh_aabb     + 6*bvhadr2) :
                                 (d->bvh_aabb_dyn + 6*(bvhadr2 - nbvhstatic));
  int body1_same_rot = isbody1 ? bodyMatchesInertiaRot(m, bf1) : 0;
  int body2_same_rot = isbody2 ? bodyMatchesInertiaRot(m, bf2) : 0;

  // intermediate-level margin (hoisted out of traversal loop)
  mjtNum mid_margin;
  if (isbody1 && isbody2) {
    mid_margin = mj_assignMargin(m, m->body_margin[bf1] + m->body_margin[bf2]);
  } else if (isbody1) {
    mid_margin = mj_assignMargin(m, m->body_margin[bf1] + m->flex_margin[f2]) + m->flex_gap[f2];
  } else {
    mid_margin = mj_assignMargin(m, m->flex_margin[f1] + m->flex_margin[f2]) +
                 (bf1 != bf2 ? m->flex_gap[f1] + m->flex_gap[f2] : 0);
  }

  // used with rotated bounding boxes (when bodies are involved)
  mjOBBFrame frame;
  int frame_init = 0;

  // allocate local stack if caller did not provide a reusable buffer
  int local_stack = (stack == NULL);
  if (local_stack) {
    mj_markStack(d);

    // TODO(b/273737633): Store bvh max depths to make this bound tighter.
    max_stack = (isbody1 ? m->body_bvhnum[bf1] : m->flex_bvhnum[f1]) +
                (isbody2 ? m->body_bvhnum[bf2] : m->flex_bvhnum[f2]);
    stack = mjSTACKALLOC(d, max_stack, mjCollisionTree);
  }

  int nstack = 1;
  stack[0].node1 = stack[0].node2 = 0;
  int start_pair = (int)((d->parena - arena->parena) / sizeof(mjcPair));

  // for body:flex, if dof-less body has planes, call pushGeomFlex
  if (isbody1 && !isbody2 && m->body_dofnum[m->body_weldid[bf1]] == 0) {
    for (int i=geomadr1; i < geomadr1+m->body_geomnum[bf1]; i++) {
      if (m->geom_type[i] == mjGEOM_PLANE) {
        if (!filterGeomFlex(m, i, f2) && !mjc_ipcOwnsFlexGeom(m, f2, i)) {
          pushGeomFlex(d, arena, i, f2, geomGroup(group, i, geomadr1));
        }
      }
    }
  }

  // for body:flex, if body has SDFs, call pushGeomFlex
  if (isbody1 && !isbody2) {
    for (int i=geomadr1; i < geomadr1+m->body_geomnum[bf1]; i++) {
      if (m->geom_type[i] == mjGEOM_SDF) {
        if (m->flex_dim[f2] == 2 && !filterGeomFlex(m, i, f2)) {
          pushGeomFlex(d, arena, i, f2, geomGroup(group, i, geomadr1));
        }
      }
    }
  }

  // collide trees
  while (nstack) {
    // pop from stack
    nstack--;
    int node1 = stack[nstack].node1;
    int node2 = stack[nstack].node2;
    int c1_0 = child1[2*node1], c1_1 = child1[2*node1+1];
    int c2_0 = child2[2*node2], c2_1 = child2[2*node2+1];
    int isleaf1 = (c1_0 < 0) & (c1_1 < 0);
    int isleaf2 = (c2_0 < 0) & (c2_1 < 0);
    int nodeid1 = -1, nodeid2 = -1;
    if (isleaf1) {
      nodeid1 = nodeid_arr1[node1];
      if (nodeid1 < 0) {
        mju_error("BVH leaf has invalid node id");  // SHOULD NOT OCCUR
      }
    }
    if (isleaf2) {
      nodeid2 = nodeid_arr2[node2];
      if (nodeid2 < 0) {
        mju_error("BVH leaf has invalid node id");  // SHOULD NOT OCCUR
      }
    }

    // body : body
    if (isbody1 && isbody2) {
      // both are leaves
      if (isleaf1 && isleaf2) {
        if (merged && isPairMerged(m, nodeid1, nodeid2, startadr, pairadr)) {
          continue;
        }
        if (!has_filter && filterGeomPair(m, nodeid1, nodeid2)) {
          continue;
        }
        int gt1 = m->geom_type[nodeid1];
        int gt2 = m->geom_type[nodeid2];
        int type1 = mjMIN(gt1, gt2);
        int type2 = mjMAX(gt1, gt2);
        if (!mjCOLLISIONFUNC[type1][type2]) {
          continue;
        }

        // filter by bounding sphere, then OBB
        // caller guarantees !bodyPairHasFastGeom when speculative is enabled
        mjtNum margin = mj_assignMargin(m, m->geom_margin[nodeid1] + m->geom_margin[nodeid2]);
        mjtNum gap = m->geom_gap[nodeid1] + m->geom_gap[nodeid2];
        mjtNum bound = margin + gap;
        if (!filterSphereStatic(m, d, nodeid1, nodeid2, bound)) {
          // bounding sphere test is strictly tighter than OBB for sphere-sphere and plane-sphere
          int skip_obb = bound >= 0 && (
              (gt1 == mjGEOM_SPHERE && gt2 == mjGEOM_SPHERE &&
               m->geom_rbound[nodeid1] > 0 && m->geom_rbound[nodeid2] > 0) ||
              (gt1 == mjGEOM_PLANE && gt2 == mjGEOM_SPHERE && m->geom_rbound[nodeid2] > 0) ||
              (gt2 == mjGEOM_PLANE && gt1 == mjGEOM_SPHERE && m->geom_rbound[nodeid1] > 0));
          int hit = skip_obb;
          if (!hit) {
            // reuse precomputed body rotation when both geoms share inertial orientation
            const mjOBBFrame* leaf_frame =
                (frame_init && geomMatchesInertiaRot(m, nodeid1, body1_same_rot) &&
                 geomMatchesInertiaRot(m, nodeid2, body2_same_rot)) ? &frame : NULL;
            hit = collideOBBLeaf(m->geom_aabb + 6*nodeid1, m->geom_aabb + 6*nodeid2,
                                 d->geom_xpos + 3*nodeid1, d->geom_xmat + 9*nodeid1,
                                 d->geom_xpos + 3*nodeid2, d->geom_xmat + 9*nodeid2,
                                 bound, leaf_frame);
          }

          if (hit) {
            if (!has_filter || !mj_callContactFilter(m, d, nodeid1, nodeid2)) {
              pushGeomGeom(m, d, arena, nodeid1, nodeid2, -1);
            }
            if (mark_active) {
              d->bvh_active[node1 + bvhadr1] = true;
              d->bvh_active[node2 + bvhadr2] = true;
            }
          }
        }
        continue;
      }

      // prune subtree if leaf geom cannot collide with the other body's bitmask
      if (!has_filter &&
          ((isleaf1 && filterGeomBody(m, nodeid1, bf2)) ||
           (isleaf2 && filterGeomBody(m, nodeid2, bf1)))) {
        continue;
      }
    }

    // body : flex
    else if (isbody1) {
      // both are leaves
      if (isleaf1 && isleaf2) {
        mjtNum margin = mj_assignMargin(m, m->geom_margin[nodeid1] + m->flex_margin[f2]);
        mjtNum gap = m->geom_gap[nodeid1] + m->flex_gap[f2];

        if (!filterGeomFlex(m, nodeid1, f2) &&
            !filterSphereBox(d->geom_xpos + 3*nodeid1, m->geom_rbound[nodeid1] + margin + gap,
                             bvh2 + 6*node2)) {
          if (collideOBBAABB(m->geom_aabb + 6*nodeid1, d->geom_xpos + 3*nodeid1,
                             d->geom_xmat + 9*nodeid1, bvh2 + 6*node2, margin + gap)) {
            // collide unless geom is plane or SDF (handled separately)
            if (m->geom_type[nodeid1] != mjGEOM_PLANE &&
                m->geom_type[nodeid1] != mjGEOM_SDF) {
              if (!mjc_ipcOwnsFlexGeom(m, f2, nodeid1)) {
                pushGeomElem(m, d, arena, nodeid1, f2, nodeid2, geomGroup(group, nodeid1, geomadr1));
              }
            }
            if (mark_active) {
              d->bvh_active[node1 + bvhadr1] = true;
              d->bvh_active[node2 + bvhadr2] = true;
            }
          }
        }
        continue;
      }

      // prune flex subtree if leaf geom cannot collide with flex bitmask
      if (isleaf1 && filterGeomFlex(m, nodeid1, f2)) {
        continue;
      }
    }

    // flex : flex
    else {
      // self-collision: avoid repeated pairs
      if (bf1 == bf2 && node1 > node2) {
        continue;
      }

      // both are leaves: same box filter as mjc_ElemElem (margin and gap ignored in
      // self-collisions), bitmask filter applied earlier
      if (isleaf1 && isleaf2) {
        mjtNum margin = 0;
        if (bf1 != bf2) {
          margin = mj_assignMargin(m, m->flex_margin[f1] + m->flex_margin[f2]) +
                   m->flex_gap[f1] + m->flex_gap[f2];
        }
        if (!filterBoxScalar(d->flexelem_aabb + 6*(m->flex_elemadr[f1] + nodeid1),
                             d->flexelem_aabb + 6*(m->flex_elemadr[f2] + nodeid2), margin)) {
          size_t parena_old = d->parena;
          if (bf1 != bf2) {
            pushElemElem(d, arena, f1, nodeid1, f2, nodeid2, group);
          } else if (nodeid1 != nodeid2) {
            pushSelfElemElem(m, d, arena, f1, nodeid1, nodeid2, group);
          }
          if (mark_active && d->parena > parena_old) {
            d->bvh_active[node1 + bvhadr1] = true;
            d->bvh_active[node2 + bvhadr2] = true;
          }
        }
        continue;
      }
    }

    // if no intersection at intermediate levels, stop
    const mjtNum* aabb1 = bvh1 + 6*node1;
    const mjtNum* aabb2 = bvh2 + 6*node2;
    if (isbody1) {
      if (!isInfiniteAABB(aabb1) && !isInfiniteAABB(aabb2)) {
        if (!frame_init) {
          initOBBFrame(&frame, d->xipos + 3*bf1, d->ximat + 9*bf1,
                       isbody2 ? d->xipos + 3*bf2 : NULL,
                       isbody2 ? d->ximat + 9*bf2 : NULL);
          frame_init = 1;
        }
        if (!collideOBBFrame(aabb1, aabb2, mid_margin, &frame)) {
          continue;
        }
      }
    } else if (filterBoxScalar(aabb1, aabb2, mid_margin)) {
      continue;
    }

    if (mark_active) {
      d->bvh_active[node1 + bvhadr1] = true;
      d->bvh_active[node2 + bvhadr2] = true;
    }

    // keep traversing the tree: expand node1 or node2
    int expand1;
    if (!isleaf1 && isleaf2) {
      expand1 = 1;
    } else if (isleaf1 && !isleaf2) {
      expand1 = 0;
    } else {
      // compute surface areas of bounding boxes (center, size): use half-sizes
      mjtNum x1 = bvh1[6*node1+3], y1 = bvh1[6*node1+4], z1 = bvh1[6*node1+5];
      mjtNum x2 = bvh2[6*node2+3], y2 = bvh2[6*node2+4], z2 = bvh2[6*node2+5];
      mjtNum surface1 = x1*y1 + y1*z1 + z1*x1;
      mjtNum surface2 = x2*y2 + y2*z2 + z2*x2;
      expand1 = (surface1 > surface2);
    }

    int c0 = expand1 ? c1_0 : c2_0;
    int c1 = expand1 ? c1_1 : c2_1;
    if (nstack + (c0 != -1) + (c1 != -1) > max_stack) {
      bvhStackError();  // SHOULD NOT OCCUR
    }
    if (expand1) {
      if (c1_0 != -1) {
        stack[nstack].node1 = c1_0;
        stack[nstack].node2 = node2;
        nstack++;
      }
      if (c1_1 != -1) {
        stack[nstack].node1 = c1_1;
        stack[nstack].node2 = node2;
        nstack++;
      }
    } else {
      if (c2_0 != -1) {
        stack[nstack].node1 = node1;
        stack[nstack].node2 = c2_0;
        nstack++;
      }
      if (c2_1 != -1) {
        stack[nstack].node1 = node1;
        stack[nstack].node2 = c2_1;
        nstack++;
      }
    }
  }

  // sort pairs generated by this tree collision
  int end_pair = (int)((d->parena - arena->parena) / sizeof(mjcPair));
  int n = end_pair - start_pair;
  if (n > 1) {
    sortTreePairs(m, d, arena, start_pair, n);
  }

  if (local_stack) {
    mj_freeStack(d);
  }
}


//----------------------------- broad-phase collision detection ------------------------------------

// rounding slack for AAMM bounds (analogous to mjBOXBOX_SEPEPS in engine_collision_box.c),
// preventing flush zero-margin geoms from being pruned by ULP error in an aligned SAP frame
#ifdef mjUSESINGLE
  #define mjAAMM_EPS 1e-5f
#else
  #define mjAAMM_EPS 1e-12
#endif


// preloaded SAP projection frame (lane 3 = 0 for all mjtNum4 members)
typedef struct {
  mjtNum frame[9];   // rows are frame axes (e0, e1, e2)
  mjtNum4 c0;        // column 0 of frame
  mjtNum4 c1;        // column 1 of frame
  mjtNum4 c2;        // column 2 of frame
  int axis_aligned;  // 1 if frame is the identity matrix
} mjSAPFrame;


// horizontal minimum of 4 lanes
static inline mjtNum hmin4(mjtNum4 a) {
  mjtNum tmp[4];
  v4_store(tmp, a);
  mjtNum lo01 = mjMIN(tmp[0], tmp[1]);
  mjtNum lo23 = mjMIN(tmp[2], tmp[3]);
  return mjMIN(lo01, lo23);
}


// horizontal maximum of 4 lanes
static inline mjtNum hmax4(mjtNum4 a) {
  mjtNum tmp[4];
  v4_store(tmp, a);
  mjtNum hi01 = mjMAX(tmp[0], tmp[1]);
  mjtNum hi23 = mjMAX(tmp[2], tmp[3]);
  return mjMAX(hi01, hi23);
}


// project a 3D vector onto the 3 rows of sap_frame: out[j] = dot(v, frame_j)
static inline mjtNum4 projectSAP(const mjSAPFrame* sap_frame, const mjtNum v[3]) {
  return sap_frame->axis_aligned
         ? v4_load3(v)
         : v4_lincomb3(sap_frame->c0, sap_frame->c1, sap_frame->c2, v[0], v[1], v[2]);
}


// project 4 flex vertices onto 3 frame rows: t_r = f[3*r]*vx + f[3*r+1]*vy + f[3*r+2]*vz
static inline void projectFlex4(mjtNum4* tx, mjtNum4* ty, mjtNum4* tz,
                                const mjtNum* p, const mjtNum4 f[9]) {
  mjtNum4 vx, vy, vz;
  v4_load4x3(p, &vx, &vy, &vz);
  *tx = v4_add(v4_add(v4_mul(f[0], vx), v4_mul(f[1], vy)), v4_mul(f[2], vz));
  *ty = v4_add(v4_add(v4_mul(f[3], vx), v4_mul(f[4], vy)), v4_mul(f[5], vz));
  *tz = v4_add(v4_add(v4_mul(f[6], vx), v4_mul(f[7], vy)), v4_mul(f[8], vz));
}


// compute unpadded AAMM of flex vertices in sap_frame
static void flexAAMM(mjtNum aamm[6], const mjtNum* vbase, int flex_vertnum,
                     const mjSAPFrame* sap_frame) {
  mjtNum4 vmin, vmax;
  int i;

  // 4-wide SIMD main loop: v = frame * vbase[3*i + 0..2] for 4 vertices at a time
  if (flex_vertnum >= 4) {
    mjtNum4 f[9];
    for (int k=0; k < 9; k++) {
      f[k] = v4_splat(sap_frame->frame[k]);
    }
    mjtNum4 tx, ty, tz;
    projectFlex4(&tx, &ty, &tz, vbase, f);
    mjtNum4 minx = tx, maxx = tx;
    mjtNum4 miny = ty, maxy = ty;
    mjtNum4 minz = tz, maxz = tz;
    for (i=4; i+4 <= flex_vertnum; i += 4) {
      projectFlex4(&tx, &ty, &tz, vbase + 3*i, f);
      minx = v4_min(minx, tx);
      maxx = v4_max(maxx, tx);
      miny = v4_min(miny, ty);
      maxy = v4_max(maxy, ty);
      minz = v4_min(minz, tz);
      maxz = v4_max(maxz, tz);
    }

    // reduce 4-lane bounds
    vmin = v4_set(hmin4(minx), hmin4(miny), hmin4(minz), 0);
    vmax = v4_set(hmax4(maxx), hmax4(maxy), hmax4(maxz), 0);
  } else {
    vmin = vmax = projectSAP(sap_frame, vbase);
    i = 1;
  }

  // scalar tail for remaining vertices
  for (; i < flex_vertnum; i++) {
    mjtNum4 v = projectSAP(sap_frame, vbase + 3*i);
    vmin = v4_min(vmin, v);
    vmax = v4_max(vmax, v);
  }
  v4_store3(aamm, vmin);
  v4_store3(aamm+3, vmax);
}


// make AAMM (xmin[3], xmax[3]) for one bodyflex, in the frame whose axes are the rows of sap_frame
static void makeAAMM(const mjModel* m, mjData* d,
                     mjtNum* x_min, mjtNum* y_min, mjtNum* z_min,
                     mjtNum* x_max, mjtNum* y_max, mjtNum* z_max,
                     int bf, const mjSAPFrame* sap_frame) {
  mjtNum aamm[6] = {0};
  mjtNum override_margin = mjENABLED(mjENBL_OVERRIDE) ? 0.5 * m->opt.o_margin : 0;

  // body
  if (bf < m->nbody) {
    int body = bf;
    int body_geomnum = m->body_geomnum[body];
    int has_filter = (mjcb_contactfilter != NULL);
    int speculative = speculativeEnabled(m);
    mjtNum4 vzero = v4_splat(0);
    mjtNum4 vmin = vzero;
    mjtNum4 vmax = vzero;
    int first = 1;

    // process collidable body geoms (body is collidable, should have geoms)
    for (int i=0; i < body_geomnum; i++) {
      int geom = m->body_geomadr[body]+i;
      if (!has_filter && !geomCollidable(m, geom)) {
        continue;
      }
      mjtNum margin = override_margin ? override_margin
                                      : m->geom_margin[geom] + m->geom_gap[geom];

      const mjtNum* aabb = m->geom_aabb + 6*geom;
      const mjtNum* size = m->geom_aabb + 6*geom + 3;
      const mjtNum* xpos = d->geom_xpos + 3*geom;
      const mjtNum* xmat = d->geom_xmat + 9*geom;

      mjtNum r_half = m->geom_rbound[geom];
      mjtNum dx[3], dw[3];
      mjtNum s = speculative ? geomStepSweep(m, d, geom, 0, dx, dw) : 0;
      mjtNum s_rot = s > 0 ? mju_norm3(dw) * r_half : 0;

      // project geom center onto the 3 frame axes: r_cen[j] = dot(xpos, frame_j)
      mjtNum4 vr_cen = projectSAP(sap_frame, xpos);
      mjtNum4 vr_half = v4_splat(r_half);
      mjtNum4 vlo, vhi;

      // centered geom whose box encloses its bounding sphere (e.g. sphere):
      // |M_j0|*s0 + |M_j1|*s1 + |M_j2|*s2 >= r_half for any orthonormal M = frame*xmat
      int centered = (aabb[0] == 0 && aabb[1] == 0 && aabb[2] == 0);
      if (centered && size[0] >= r_half && size[1] >= r_half && size[2] >= r_half) {
        vlo = v4_sub(vr_cen, vr_half);
        vhi = v4_add(vr_cen, vr_half);
      } else {
        // compute center in global coordinates (skip when aabb center is zero)
        mjtNum4 vaabb_cen;
        if (centered) {
          vaabb_cen = vr_cen;
        } else {
          mjtNum pos[3];
          mji_mulMatVec3(pos, xmat, aabb);
          mju_addTo3(pos, xpos);
          vaabb_cen = projectSAP(sap_frame, pos);
        }

        // project geom axes onto frame: m_k[j] = dot(xmat col k, frame_j)
        mjtNum4 m0, m1, m2;
        if (sap_frame->axis_aligned) {
          m0 = v4_set(xmat[0], xmat[3], xmat[6], 0);
          m1 = v4_set(xmat[1], xmat[4], xmat[7], 0);
          m2 = v4_set(xmat[2], xmat[5], xmat[8], 0);
        } else {
          mjtNum4 c0 = sap_frame->c0, c1 = sap_frame->c1, c2 = sap_frame->c2;
          m0 = v4_lincomb3(c0, c1, c2, xmat[0], xmat[3], xmat[6]);
          m1 = v4_lincomb3(c0, c1, c2, xmat[1], xmat[4], xmat[7]);
          m2 = v4_lincomb3(c0, c1, c2, xmat[2], xmat[5], xmat[8]);
        }

        // aabb_half[j] = |m0[j]|*size[0] + |m1[j]|*size[1] + |m2[j]|*size[2] + s_rot
        mjtNum4 vaabb_half = v4_lincomb3(v4_abs(m0), v4_abs(m1), v4_abs(m2),
                                         size[0], size[1], size[2]);
        if (s > 0) {
          vaabb_half = v4_add(vaabb_half, v4_splat(s_rot));
        }

        // tighten interval using both bounding sphere and oriented box
        vlo = v4_max(v4_sub(vr_cen, vr_half), v4_sub(vaabb_cen, vaabb_half));
        vhi = v4_min(v4_add(vr_cen, vr_half), v4_add(vaabb_cen, vaabb_half));
      }

      // expand interval by speculative translation sweep
      if (s > 0) {
        mjtNum4 vd_proj = projectSAP(sap_frame, dx);
        vlo = v4_add(vlo, v4_min(vzero, vd_proj));
        vhi = v4_add(vhi, v4_max(vzero, vd_proj));
      }

      // expand by margin
      mjtNum4 vmargin = v4_splat(margin);
      vlo = v4_sub(vlo, vmargin);
      vhi = v4_add(vhi, vmargin);

      // update body aamm
      if (first) {
        vmin = vlo;
        vmax = vhi;
        first = 0;
      } else {
        vmin = v4_min(vmin, vlo);
        vmax = v4_max(vmax, vhi);
      }
    }

    // store accumulated body bounds
    if (!first) {
      v4_store3(aamm, vmin);
      v4_store3(aamm+3, vmax);
    }
  }

  // flex
  else {
    int f = bf - m->nbody;
    int flex_vertnum = m->flex_vertnum[f];
    const mjtNum* vbase = d->flexvert_xpos + 3*m->flex_vertadr[f];

    // process flex vertices
    if (flex_vertnum > 0) {
      flexAAMM(aamm, vbase, flex_vertnum, sap_frame);
    }

    // correct for flex radius and margin
    mjtNum margin = override_margin ? override_margin
                                    : m->flex_margin[f] + m->flex_gap[f];
    mjtNum box[2];
    mjtNum pad = mj_rodBoxSize(m, f, box) ? mju_max(m->flex_radius[f], mju_norm(box, 2))
                                          : m->flex_radius[f];
    mjtNum bound = pad + margin;
    aamm[0] -= bound;
    aamm[1] -= bound;
    aamm[2] -= bound;
    aamm[3] += bound;
    aamm[4] += bound;
    aamm[5] += bound;
  }

  // pad bounds by rounding slack so flush zero-margin geoms are not pruned
  for (int j=0; j < 3; j++) {
    mjtNum alo = mju_abs(aamm[j]);
    mjtNum ahi = mju_abs(aamm[j+3]);
    mjtNum tol = mjAAMM_EPS * (1 + mjMAX(alo, ahi));
    aamm[j]   -= tol;
    aamm[j+3] += tol;
  }

  // assign outputs
  *x_min = aamm[0];
  *y_min = aamm[1];
  *z_min = aamm[2];
  *x_max = aamm[3];
  *y_max = aamm[4];
  *z_max = aamm[5];
}


// add bodyflex pair in buffer; do not filter if m is NULL
static void addPair(const mjModel* m, int bf1, int bf2,
                     int* npair, mjPacked32* pair, int maxpair) {
  // contact filtering if m is not NULL
  if (m && !pairCollidable(m, bf1, bf2)) {
    return;
  }

  // add pair if there is room in buffer
  if ((*npair) < maxpair) {
    int n = (*npair)++;
    pair[n].hi = (bf1 < bf2) ? bf1 : bf2;
    pair[n].lo = (bf1 < bf2) ? bf2 : bf1;
  } else {
    mjERROR("broadphase buffer full");
  }
}


//----------------------------- general Sweep and Prune algorithm ----------------------------------

// helper structure for SAP sorting
struct _mjtSAP {
  union {
    float value;
    uint32_t key;
  };
  int id;
};
typedef struct _mjtSAP mjtSAP;


// comparison function for SAP
static inline int SAPcmp(const mjtSAP* obj1, const mjtSAP* obj2, void* context) {
  (void) context;
  if (obj1->value < obj2->value) {
    return -1;
  } else if (obj1->value > obj2->value) {
    return 1;
  } else {
    return 0;
  }
}

// define SAPsort function for sorting SAP sorting
mjSORT(SAPsort, mjtSAP, SAPcmp);


// radix/counting sort thresholds and 8-bit radix bucket count
enum {
  mjRADIX_BUCKETS     = 256,   // 2^8 buckets per 8-bit radix pass
  mjSAP_RADIX_MIN     = 96,    // minimum SAP endpoints for sapRadixSort
  mjBF_COUNT_MIN      = 64,    // minimum pairs for 2-pass counting sort
  mjBF_COUNT_MAXBOUND = 4096,  // maximum id bound for L1-resident counting sort
  mjBF_RADIX_MIN      = 128,   // minimum pairs for 4-pass 8-bit LSD radix sort
};


// in-place exclusive prefix sum on histogram c[0..n-1]
static inline void exclusivePrefixSum(int* c, int n) {
  int sum = 0;
  for (int i=0; i < n; i++) {
    int k = c[i];
    c[i] = sum;
    sum += k;
  }
}


// map IEEE-754 float to order-preserving uint32 key (-0.0f canonicalized to +0.0f)
static inline uint32_t floatToRadixKey(float f) {
  uint32_t bits;
  memcpy(&bits, &f, sizeof(bits));
  if (bits == 0x80000000u) {
    bits = 0;
  }

  // flip sign bit for >= 0, all bits for < 0
  uint32_t mask = ((uint32_t)0 - (bits >> 31)) | 0x80000000u;
  return bits ^ mask;
}


// inverse of floatToRadixKey
static inline float radixKeyToFloat(uint32_t key) {
  uint32_t mask = ((key >> 31) - 1u) | 0x80000000u;
  uint32_t bits = key ^ mask;
  float f;
  memcpy(&f, &bits, sizeof(f));
  return f;
}


// stable 4-pass 8-bit LSD radix sort for SAP endpoints
// outlined to keep mj_SAP stack frame small when unused
static mjNOINLINE void sapRadixSort(mjtSAP* sortbuf, mjtSAP* buf, const mjtNum* x_min, int n) {
  int cnt[4][mjRADIX_BUCKETS];
  memset(cnt, 0, sizeof(cnt));

  // encode float keys and accumulate all 4 byte histograms in one pass
  for (int i=0; i < n; i++) {
    uint32_t key = floatToRadixKey((float)x_min[i]);
    sortbuf[i].key = key;
    sortbuf[i].id = i;
    cnt[0][key & 0xFFu]++;
    cnt[1][(key >> 8) & 0xFFu]++;
    cnt[2][(key >> 16) & 0xFFu]++;
    cnt[3][key >> 24]++;
  }

  // find last active pass so key decoding folds into its scatter
  int last_pass = -1;
  uint32_t k0 = sortbuf[0].key;
  for (int pass=0; pass < 4; pass++) {
    if (cnt[pass][(k0 >> (8*pass)) & 0xFFu] < n) {
      last_pass = pass;
    }
  }

  // stable LSD radix passes, skipping any byte where all keys match
  mjtSAP* src = sortbuf;
  mjtSAP* dst = buf;
  for (int pass=0; pass < 4; pass++) {
    int shift = 8*pass;
    if (cnt[pass][(k0 >> shift) & 0xFFu] == n) {
      continue;
    }

    // exclusive prefix sum
    int* c = cnt[pass];
    exclusivePrefixSum(c, mjRADIX_BUCKETS);

    // scatter (decode keys back to float on final active pass)
    if (pass == last_pass) {
      for (int i=0; i < n; i++) {
        mjtSAP item = src[i];
        int idx = c[(item.key >> shift) & 0xFFu]++;
        dst[idx].value = radixKeyToFloat(item.key);
        dst[idx].id = item.id;
      }
    } else {
      for (int i=0; i < n; i++) {
        dst[c[(src[i].key >> shift) & 0xFFu]++] = src[i];
      }
    }

    mjtSAP* tmp = src;
    src = dst;
    dst = tmp;
  }

  // all keys equal: decode constant value
  if (last_pass < 0) {
    float val = radixKeyToFloat(k0);
    for (int i=0; i < n; i++) {
      sortbuf[i].value = val;
    }
  }

  // copy back if odd number of passes
  else if (src != sortbuf) {
    memcpy(sortbuf, src, n * sizeof(mjtSAP));
  }
}


// sort per-box SAP hits by sweep insertion rank and finalize (id1, id2) pairs
static inline void sapFinishOrdered(mjPacked32* pair, int start, int end, int id2) {
  for (int k=start+1; k < end; k++) {
    mjPacked32 tmp = pair[k];
    int p = k - 1;
    while (p >= start && pair[p].hi > tmp.hi) {
      pair[p+1] = pair[p];
      p--;
    }
    pair[p+1] = tmp;
  }
  for (int k=start; k < end; k++) {
    pair[k].hi = pair[k].lo;
    pair[k].lo = (uint16_t)id2;
  }
}


// given list of axis-aligned bounding boxes in AAMM (min[3], max[3]) format,
// return number of pairs that can collide, using sweep-and-prune along
// specified x axis (0, 1 or 2). If active is non-NULL, pairs where neither
// entry is active are skipped. When ordered is non-zero (with active == NULL),
// hits for each box are emitted in sweep insertion order. If more than maxpair
// overlaps exist, only the first maxpair are written to pair[] and maxpair + 1
// is returned.
static int mj_SAP(mjData* d, const mjtNum* aamm, const mjtByte* active, int n, int axis_x,
                  int ordered, mjPacked32* pair, int maxpair) {
  // check inputs
  if (n < 0 || n >= 0x10000 || axis_x < 0 || axis_x > 2 || maxpair < 1) {
    return -1;
  }
  if (n < 2) {
    return 0;
  }

  mj_markStack(d);

  // get AAMM pointers for primary "x" axis
  const mjtNum* x_min = aamm + n*(axis_x + 0);
  const mjtNum* x_max = aamm + n*(axis_x + 3);

  // init sortbuf with primary axis min values and sort along specified axis
  mjtSAP* sortbuf = mjSTACKALLOC(d, n, mjtSAP);
  {
    mj_markStack(d);
    mjtSAP* buf = mjSTACKALLOC(d, n, mjtSAP);

    // radix sort for large n
    if (n >= mjSAP_RADIX_MIN) {
      sapRadixSort(sortbuf, buf, x_min, n);
    }

    // comparison sort for small n
    else {
      for (int i=0; i < n; i++) {
        sortbuf[i].value = (float)x_min[i];
        sortbuf[i].id = i;
      }
      SAPsort(sortbuf, buf, n, NULL);
    }
    mj_freeStack(d);
  }

  // define the other two axes
  int axis_y, axis_z;
  if (axis_x == 0) {
    axis_y = 1;
    axis_z = 2;
  } else if (axis_x == 1) {
    axis_y = 0;
    axis_z = 2;
  } else {
    axis_y = 0;
    axis_z = 1;
  }

  // get AAMM pointers to secondary "y, z" axes
  const mjtNum* y_min = aamm + n*(axis_y + 0);
  const mjtNum* y_max = aamm + n*(axis_y + 3);
  const mjtNum* z_min = aamm + n*(axis_z + 0);
  const mjtNum* z_max = aamm + n*(axis_z + 3);

  // contiguous SoA active interval arrays (unordered, swap-removed on expiration)
  float* act_xmax = mjSTACKALLOC(d, n, float);
  mjtNum* act_ymin = mjSTACKALLOC(d, n, mjtNum);
  mjtNum* act_ymax = mjSTACKALLOC(d, n, mjtNum);
  mjtNum* act_zmin = mjSTACKALLOC(d, n, mjtNum);
  mjtNum* act_zmax = mjSTACKALLOC(d, n, mjtNum);
  uint32_t* act_id = mjSTACKALLOC(d, n, uint32_t);
  int keep_order = ordered && !active;

  // sweep and prune
  int cnt = 0;    // size of active list
  int npair = 0;  // number of pairs added
  for (int i=0; i < n; i++) {
    float x2_min = sortbuf[i].value;
    int id2 = sortbuf[i].id;
    float x2_max = (float)x_max[id2];
    if (x2_max < x2_min) {
      continue;
    }

    uint32_t passive2 = (active && !active[id2]) ? 0x10000u : 0u;
    int npair0 = npair;
    mjtNum y2_min = y_min[id2];
    mjtNum y2_max = y_max[id2];
    mjtNum z2_min = z_min[id2];
    mjtNum z2_max = z_max[id2];

    int j = 0;
    mjtFloat4 vx2_min = f4_splat(x2_min);
    mjtNum4 vy2_min = v4_splat(y2_min);
    mjtNum4 vy2_max = v4_splat(y2_max);
    mjtNum4 vz2_min = v4_splat(z2_min);
    mjtNum4 vz2_max = v4_splat(z2_max);
    while (j + 4 <= cnt) {
      int exp_mask = f4_movemask(f4_cmplt(f4_load(act_xmax+j), vx2_min));
      if (exp_mask) {
        int k = j + mju_ctz64((uint64_t)exp_mask);
        cnt--;
        if (k < cnt) {
          act_xmax[k] = act_xmax[cnt];
          act_ymin[k] = act_ymin[cnt];
          act_ymax[k] = act_ymax[cnt];
          act_zmin[k] = act_zmin[cnt];
          act_zmax[k] = act_zmax[cnt];
          act_id[k] = act_id[cnt];
        }
        continue;
      }

      mjtMask4 sep_y = v4_or(v4_cmpgt(v4_load(act_ymin+j), vy2_max),
                             v4_cmpgt(vy2_min, v4_load(act_ymax+j)));
      mjtMask4 sep_z = v4_or(v4_cmpgt(v4_load(act_zmin+j), vz2_max),
                             v4_cmpgt(vz2_min, v4_load(act_zmax+j)));
      int hit_mask = (~v4_movemask(v4_or(sep_y, sep_z))) & 0xF;
      while (hit_mask) {
        int bit = mju_ctz64((uint64_t)hit_mask);
        hit_mask &= hit_mask - 1;
        uint32_t raw = act_id[j+bit];
        if (passive2 && (raw & 0x10000u)) {
          continue;
        }
        if (npair >= maxpair) {
          if (keep_order) {
            sapFinishOrdered(pair, npair0, npair, id2);
          }
          mj_freeStack(d);
          return maxpair + 1;
        }
        pair[npair].hi = keep_order ? (uint16_t)(raw >> 16) : (uint16_t)(raw & 0xFFFFu);
        pair[npair].lo = keep_order ? (uint16_t)(raw & 0xFFFFu) : (uint16_t)id2;
        npair++;
      }
      j += 4;
    }

    // scalar tail
    while (j < cnt) {
      if (act_xmax[j] < x2_min) {
        cnt--;
        if (j < cnt) {
          act_xmax[j] = act_xmax[cnt];
          act_ymin[j] = act_ymin[cnt];
          act_ymax[j] = act_ymax[cnt];
          act_zmin[j] = act_zmin[cnt];
          act_zmax[j] = act_zmax[cnt];
          act_id[j] = act_id[cnt];
        }
        continue;
      }

      uint32_t raw = act_id[j];
      if (!(passive2 && (raw & 0x10000u)) &&
          !(act_ymin[j] > y2_max ||
            y2_min > act_ymax[j] ||
            act_zmin[j] > z2_max ||
            z2_min > act_zmax[j])) {
        if (npair >= maxpair) {
          if (keep_order) {
            sapFinishOrdered(pair, npair0, npair, id2);
          }
          mj_freeStack(d);
          return maxpair + 1;
        }
        pair[npair].hi = keep_order ? (uint16_t)(raw >> 16) : (uint16_t)(raw & 0xFFFFu);
        pair[npair].lo = keep_order ? (uint16_t)(raw & 0xFFFFu) : (uint16_t)id2;
        npair++;
      }
      j++;
    }

    if (keep_order && npair > npair0) {
      sapFinishOrdered(pair, npair0, npair, id2);
    }

    // add current box to active list
    act_xmax[cnt] = x2_max;
    act_ymin[cnt] = y2_min;
    act_ymax[cnt] = y2_max;
    act_zmin[cnt] = z2_min;
    act_zmax[cnt] = z2_max;
    act_id[cnt] = (uint32_t)id2 | (keep_order ? ((uint32_t)i << 16) : passive2);
    cnt++;
  }

  mj_freeStack(d);
  return npair;
}


// accumulator for shifted single-pass covariance (s[6]: 00, 01, 02, 11, 12, 22)
typedef struct {
  mjtNum p0[3];   // shift origin (first point)
  mjtNum sum[3];  // sum of shifted points
  mjtNum s[6];    // upper triangle of shifted outer-product sum
  int cnt;        // number of accumulated points
} mjCovAccum;


// accumulate one point into shifted single-pass covariance sums
static inline void addShiftedCov(mjCovAccum* acc, const mjtNum p[3]) {
  // first point sets shift origin
  if (!acc->cnt) {
    mji_copy3(acc->p0, p);
  }

  // accumulate shifted point
  else {
    mjtNum dx = p[0] - acc->p0[0];
    mjtNum dy = p[1] - acc->p0[1];
    mjtNum dz = p[2] - acc->p0[2];
    acc->sum[0] += dx;
    acc->sum[1] += dy;
    acc->sum[2] += dz;
    acc->s[0] += dx*dx;
    acc->s[1] += dx*dy;
    acc->s[2] += dx*dz;
    acc->s[3] += dy*dy;
    acc->s[4] += dy*dz;
    acc->s[5] += dz*dz;
  }
  acc->cnt++;
}


// multiply symmetric 3x3 matrix S[6] (00, 01, 02, 11, 12, 22) by vector v[3]
static inline void mulSym3Vec(mjtNum res[3], const mjtNum S[6], const mjtNum v[3]) {
  mjtNum r0 = S[0]*v[0] + S[1]*v[1] + S[2]*v[2];
  mjtNum r1 = S[1]*v[0] + S[3]*v[1] + S[4]*v[2];
  mjtNum r2 = S[2]*v[0] + S[4]*v[1] + S[5]*v[2];
  res[0] = r0;
  res[1] = r1;
  res[2] = r2;
}


// initialize sap_frame to the world identity frame
static inline void initIdentitySAPFrame(mjSAPFrame* sap_frame) {
  mju_eye(sap_frame->frame, 3);
  sap_frame->c0 = v4_set(1, 0, 0, 0);
  sap_frame->c1 = v4_set(0, 1, 0, 0);
  sap_frame->c2 = v4_set(0, 0, 1, 0);
  sap_frame->axis_aligned = 1;
}


// build a covariance-aligned orthonormal SAP frame via shifted single-pass covariance
// and power iteration; returns primary SAP axis (0, 1, or 2), or -1 if no points exist
static int makeSAPFrame(const mjModel* m, const mjData* d, mjSAPFrame* sap_frame, int has_filter) {
  int ngeom = m->ngeom, nflex = m->nflex;
  mjCovAccum acc = {0};

  // find center and shifted covariance of non-world collidable geoms in one pass
  for (int i=0; i < ngeom; i++) {
    if (m->geom_bodyid[i] && (has_filter || geomCollidable(m, i))) {
      addShiftedCov(&acc, d->geom_xpos + 3*i);
    }
  }

  // flex vertices
  for (int f=0; f < nflex; f++) {
    if (!flexCollidable(m, f)) {
      continue;
    }
    int vstart = m->flex_vertadr[f];
    int vend = vstart + m->flex_vertnum[f];
    for (int v=vstart; v < vend; v++) {
      if (m->flex_vertbodyid[v]) {
        addShiftedCov(&acc, d->flexvert_xpos + 3*v);
      }
    }
  }

  // no collidable points
  if (acc.cnt == 0) {
    return -1;
  }

  // centered covariance scaled by cnt: C = sum(d*d') - sum(d)*sum(d)'/cnt
  mjtNum inv_cnt = 1 / (mjtNum)acc.cnt;
  mjtNum mx = acc.sum[0]*inv_cnt, my = acc.sum[1]*inv_cnt, mz = acc.sum[2]*inv_cnt;
  mjtNum C[6] = {
    mju_max(0, acc.s[0] - acc.sum[0]*mx),
    acc.s[1] - acc.sum[0]*my,
    acc.s[2] - acc.sum[0]*mz,
    mju_max(0, acc.s[3] - acc.sum[1]*my),
    acc.s[4] - acc.sum[1]*mz,
    mju_max(0, acc.s[5] - acc.sum[2]*mz)
  };

  // use world frame directly when covariance is nearly axis-aligned
  // SAP is exact in any frame, a rough one prunes as well
  mjtNum dmax = mju_max(C[0], mju_max(C[3], C[5]));
  mjtNum off = mju_max(mju_abs(C[1]), mju_max(mju_abs(C[2]), mju_abs(C[4])));
  int best_axis = (C[0] >= C[3] && C[0] >= C[5]) ? 0 : (C[3] >= C[5] ? 1 : 2);
  if (dmax <= mjMINVAL || off <= 0.1*dmax) {
    initIdentitySAPFrame(sap_frame);
    return best_axis;
  }

  // construct covariance-aligned 3D frame: eigenvectors are rows of frame
  // normalize by trace so eigenvalues lie in [0, 1]
  mju_scl(C, C, 1 / (C[0] + C[3] + C[5]), 6);

  // power iteration for primary axis e0, starting from C * e_best_axis
  mjtNum e0[3];
  if (best_axis == 0) {
    e0[0] = C[0];
    e0[1] = C[1];
    e0[2] = C[2];
  } else if (best_axis == 1) {
    e0[0] = C[1];
    e0[1] = C[3];
    e0[2] = C[4];
  } else {
    e0[0] = C[2];
    e0[1] = C[4];
    e0[2] = C[5];
  }
  for (int iter=0; iter < 3; iter++) {
    mulSym3Vec(e0, C, e0);
  }

  // degenerate or non-finite covariance: fall back to identity frame
  if (!(mju_normalize3(e0) >= mjMINVAL)) {
    initIdentitySAPFrame(sap_frame);
    return best_axis;
  }

  // deflate C along e0: D = C - (e0' C e0) * e0 * e0'
  mjtNum ce0[3];
  mulSym3Vec(ce0, C, e0);
  mjtNum lam0 = mju_dot3(e0, ce0);
  mjtNum D[6] = {
    C[0] - lam0*e0[0]*e0[0],
    C[1] - lam0*e0[0]*e0[1],
    C[2] - lam0*e0[0]*e0[2],
    C[3] - lam0*e0[1]*e0[1],
    C[4] - lam0*e0[1]*e0[2],
    C[5] - lam0*e0[2]*e0[2]
  };

  // initial vector orthogonal to e0 from least-aligned world axis
  mjtNum u[3] = {0, 0, 0};
  mjtNum ax = mju_abs(e0[0]), ay = mju_abs(e0[1]), az = mju_abs(e0[2]);
  int min_axis = (ax <= ay && ax <= az) ? 0 : (ay <= az ? 1 : 2);
  u[min_axis] = 1;
  mji_addToScl3(u, e0, -e0[min_axis]);
  mju_normalize3(u);

  // power iteration on D for secondary axis e1
  mjtNum e1[3];
  mji_copy3(e1, u);
  for (int iter=0; iter < 2; iter++) {
    mulSym3Vec(e1, D, e1);
  }

  // re-orthogonalize e1 against e0
  mji_addToScl3(e1, e0, -mju_dot3(e1, e0));
  if (!(mju_normalize3(e1) >= 1e-6)) {
    mji_copy3(e1, u);
  }

  // complete right-handed orthonormal frame: e2 = e0 x e1
  mjtNum e2[3];
  mji_cross(e2, e0, e1);

  // store frame rows and preloaded columns
  mji_copy3(sap_frame->frame + 0, e0);
  mji_copy3(sap_frame->frame + 3, e1);
  mji_copy3(sap_frame->frame + 6, e2);
  sap_frame->c0 = v4_set(e0[0], e1[0], e2[0], 0);
  sap_frame->c1 = v4_set(e0[1], e1[1], e2[1], 0);
  sap_frame->c2 = v4_set(e0[2], e1[2], e2[2], 0);
  sap_frame->axis_aligned = 0;
  return 0;
}


// comparison function for packed 32-bit unsigned integers (lexicographical)
static inline int mjPacked32cmp(const mjPacked32* i, const mjPacked32* j, void* context) {
  if (i->hi < j->hi) return -1;
  if (i->hi > j->hi) return 1;
  if (i->lo < j->lo) return -1;
  if (i->lo > j->lo) return 1;
  return 0;
}

// define bfsort function for sorting bodyflex pairs
mjSORT(bfsort, mjPacked32, mjPacked32cmp);


// radix/counting sort for bodyflex pairs by (hi, lo); returns 1 if sorted, 0 if skipped
// outlined to keep mj_broadphase stack frame small when unused
static mjNOINLINE int bfRadixSort(mjData* d, mjPacked32* pair, mjPacked32* buf, int n, int maxid) {
  int bound = mjMIN(maxid, 0x10000);

  // 2-pass exact counting sort when bucket count fits in L1, is amortized by n,
  // and fits in the remaining arena stack
  if (bound > 0 && bound <= mjBF_COUNT_MAXBOUND && n >= (bound >> 1)) {
    size_t need = mj_stackFrameBytes() +
                  2 * mj_stackBytes((size_t)bound * sizeof(int), _Alignof(int));
    if (mj_stackBytesAvailable(d) >= need) {
      mj_markStack(d);
      int* cnt_lo = mjSTACKALLOC(d, bound, int);
      int* cnt_hi = mjSTACKALLOC(d, bound, int);
      mju_zeroInt(cnt_lo, bound);
      mju_zeroInt(cnt_hi, bound);

      // count lo and hi histograms in a single pass
      for (int i=0; i < n; i++) {
        cnt_lo[pair[i].lo]++;
        cnt_hi[pair[i].hi]++;
      }

      // exclusive prefix sums
      exclusivePrefixSum(cnt_lo, bound);
      exclusivePrefixSum(cnt_hi, bound);

      // pass 1: scatter by lo (pair -> buf)
      for (int i=0; i < n; i++) {
        buf[cnt_lo[pair[i].lo]++] = pair[i];
      }

      // pass 2: scatter by hi (buf -> pair)
      for (int i=0; i < n; i++) {
        pair[cnt_hi[buf[i].hi]++] = buf[i];
      }

      mj_freeStack(d);
      return 1;
    }
  }

  // 4-pass 8-bit LSD radix sort for larger id ranges
  if (n >= mjBF_RADIX_MIN) {
    int cnt[4][mjRADIX_BUCKETS];
    memset(cnt, 0, sizeof(cnt));

    // accumulate all 4 byte histograms in one pass
    for (int i=0; i < n; i++) {
      uint16_t lo = pair[i].lo, hi = pair[i].hi;
      cnt[0][lo & 0xFFu]++;
      cnt[1][lo >> 8]++;
      cnt[2][hi & 0xFFu]++;
      cnt[3][hi >> 8]++;
    }

    // stable LSD radix passes, skipping any byte where all keys match
    int n_hi = ((bound > 0 ? bound - 1 : 0xFFFF) >> 8) + 1;
    int lim[4] = {mjRADIX_BUCKETS, n_hi, mjRADIX_BUCKETS, n_hi};
    uint16_t lo0 = pair[0].lo, hi0 = pair[0].hi;
    int b0[4] = {lo0 & 0xFFu, lo0 >> 8, hi0 & 0xFFu, hi0 >> 8};
    mjPacked32* src = pair;
    mjPacked32* dst = buf;
    for (int p=0; p < 4; p++) {
      if (cnt[p][b0[p]] == n) {
        continue;
      }

      // exclusive prefix sum
      int* c = cnt[p];
      exclusivePrefixSum(c, lim[p]);

      // scatter
      switch (p) {
        case 0:
          for (int i=0; i < n; i++) {
            dst[c[src[i].lo & 0xFFu]++] = src[i];
          }
          break;
        case 1:
          for (int i=0; i < n; i++) {
            dst[c[src[i].lo >> 8]++] = src[i];
          }
          break;
        case 2:
          for (int i=0; i < n; i++) {
            dst[c[src[i].hi & 0xFFu]++] = src[i];
          }
          break;
        default:
          for (int i=0; i < n; i++) {
            dst[c[src[i].hi >> 8]++] = src[i];
          }
          break;
      }

      mjPacked32* tmp = src;
      src = dst;
      dst = tmp;
    }

    // copy back if odd number of passes
    if (src != pair) {
      memcpy(pair, src, n * sizeof(mjPacked32));
    }
    return 1;
  }

  return 0;
}


// return 1 if body b is paired against all collidable bodies/flexes without SAP, 0 otherwise
static inline int alwaysColliding(const mjModel* m, int b) {
  return (b == 0 && m->body_geomnum[b] > 0) ||
         (m->body_dofnum[m->body_weldid[b]] == 0 && hasPlane(m, b));
}


// broadphase collision detector
static int mj_broadphase(const mjModel* m, mjData* d, mjPacked32* bfpair, int maxpair) {
  int npair = 0, nbody = m->nbody, nflex = m->nflex;
  int nbodyflex = nbody + nflex;
  int dsbl_filterparent = mjDISABLED(mjDSBL_FILTERPARENT);
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < nbody;
  int has_filter = (mjcb_contactfilter != NULL);

  mj_markStack(d);
  mjtByte* skip_sap = mjSTACKALLOC(d, nbody, mjtByte);
  memset(skip_sap, 0, nbody);

  // init with pairs involving always-colliding bodies
  for (int b1=0; b1 < nbody; b1++) {
    if (!collidable(m, b1) || !alwaysColliding(m, b1)) {
      continue;
    }
    skip_sap[b1] = 1;

    // add b1:b2 pairs that are not welded together
    for (int b2=0; b2 < nbody; b2++) {
      if (!collidable(m, b2) ||
          filterBodyPair(m, d, b1, b2, dsbl_filterparent, sleep_filter)) {
        continue;
      }

      // add pair
      addPair(m, b1, b2, &npair, bfpair, maxpair);
    }

    // add body:flex pairs, skip if flex has no dofs (like b1) or is asleep
    for (int f=0; f < nflex; f++) {
      if (!flexCollidable(m, f) || !hasAwakeDofs(m, d, nbody+f, sleep_filter)) continue;
      addPair(m, b1, nbody+f, &npair, bfpair, maxpair);
    }
  }

  // allocate collidable non-always-colliding bodyflex ids and active flags
  int* bfid = mjSTACKALLOC(d, nbodyflex, int);
  mjtByte* active = mjSTACKALLOC(d, nbodyflex, mjtByte);
  int ncollide = 0, nactive = 0, nawake = 0, has_mocap = 0, has_asleep = 0;
  for (int i=1; i < nbodyflex; i++) {
    if (!collidable(m, i) || (i < nbody && skip_sap[i])) {
      continue;
    }
    int is_awake = hasAwakeDofs(m, d, i, sleep_filter);
    int is_mocap = (i < nbody && m->body_treeid[i] < 0 && m->body_weldid[i] != 0);
    if (sleep_filter && i < nbody && d->body_awake[i] == mjS_ASLEEP) {
      has_asleep = 1;
    }
    mjtByte act = (mjtByte)(is_awake || (sleep_filter && is_mocap));
    bfid[ncollide] = i;
    active[ncollide] = act;
    ncollide++;
    nactive += act;
    nawake += is_awake;
    has_mocap |= is_mocap;
  }

  if (ncollide > 1 && (nawake > 0 || (has_mocap && has_asleep))) {
    // construct SAP projection frame and primary sweep axis
    mjSAPFrame sap_frame;
    int sap_axis = makeSAPFrame(m, d, &sap_frame, has_filter);

    if (sap_axis >= 0) {
      // allocate and construct AAMMs for collidable only
      mjtNum* aamm = mjSTACKALLOC(d, 6*ncollide, mjtNum);
      for (int i=0; i < ncollide; i++) {
        // aamm is column-major (grouped per-axis rather than per-box for better cache locality)
        makeAAMM(m, d, aamm + 0*ncollide + i,
                       aamm + 1*ncollide + i,
                       aamm + 2*ncollide + i,
                       aamm + 3*ncollide + i,
                       aamm + 4*ncollide + i,
                       aamm + 5*ncollide + i, bfid[i], &sap_frame);
      }

      // call SAP (pass active mask only when passive entries exist)
      int maxsappair = ncollide*(ncollide-1)/2;
      mjPacked32* sappair = mjSTACKALLOC(d, maxsappair, mjPacked32);
      const mjtByte* sap_active = (nactive < ncollide) ? active : NULL;
      int nsappair = mj_SAP(d, aamm, sap_active, ncollide, sap_axis, 0, sappair, maxsappair);
      if (nsappair < 0) {
        mjERROR("SAP failed");
      }
      if (nsappair > maxsappair) {
        mj_warning(d, mjWARN_CONTACTFULL, maxsappair);
        nsappair = maxsappair;
      }

      // filter SAP pairs, convert to bodyflex pairs
      for (int i=0; i < nsappair; i++) {
        int bf1 = bfid[sappair[i].hi];
        int bf2 = bfid[sappair[i].lo];

        // body pair: prune based on sleep filter and weld filter
        if (bf1 < nbody && bf2 < nbody) {
          if (filterBodyPair(m, d, bf1, bf2, dsbl_filterparent, sleep_filter)) {
            continue;
          }
        }

        // flex pair: skip if neither side has awake dofs
        // (smaller id first: a body is cheaper to test than a flex)
        else if (!hasAwakeDofs(m, d, mjMIN(bf1, bf2), sleep_filter) &&
                 !hasAwakeDofs(m, d, mjMAX(bf1, bf2), sleep_filter)) {
          continue;
        }

        // add bodyflex pair if there is room in buffer
        addPair(m, bf1, bf2, &npair, bfpair, maxpair);
      }
    }
  }

  // sort bodyflex pairs by signature
  if (npair > 1) {
    mjPacked32* buf = mjSTACKALLOC(d, npair, mjPacked32);
    int use_radix = (npair >= mjBF_RADIX_MIN) ||
                    (npair >= mjBF_COUNT_MIN && npair >= (nbodyflex >> 1));
    if (!use_radix || !bfRadixSort(d, bfpair, buf, npair, nbodyflex)) {
      bfsort(bfpair, buf, npair, NULL);
    }
  }

  mj_freeStack(d);
  return npair;
}


//----------------------------- narrow-phase collision detection ----------------------------------


// set contact parameters
static void mj_setContact(const mjModel* m, const mjData* d, mjContact* con,
                          int condim, mjtNum includemargin,
                          const mjtNum* solref, const mjtNum* solreffriction,
                          const mjtNum* solimp, const mjtNum* friction,
                          mjtNum adhesion, int speculative) {
  // set parameters
  con->dim = condim;
  con->includemargin = includemargin;
  con->adhesion = adhesion;
  mj_assignRef(m, con->solref, solref);
  mj_assignRef(m, con->solreffriction, solreffriction);
  mj_assignImp(m, con->solimp, solimp);
  mj_assignFriction(m, con->friction, friction);

  // exclude in gap; adhesive and speculative contacts remain active when dist >= includemargin
  con->exclude = (con->dist >= includemargin) && !adhesion && !speculative;

  // in-gap adhesive contacts have no surface contact and therefore no
  // friction: reduce to normal-only rows (with tangential rows present, fast
  // slip would recruit normal force and brake tangential motion at a distance).
  // Speculative contacts represent a surface impact during this step (where
  // dist >= includemargin only indicates t >= 1 - b0), so they keep condim.
  if (adhesion && !speculative && con->dist >= includemargin) {
    con->dim = 1;
  }

  // complete frame
  mju_makeFrame(con->frame);

  // compute relative surface velocity in contact frame: (tangent1, tangent2, spin)
  mji_zero3(con->surfacevel);
  if (m->flg_surfacevel) {
    mjtNum svel[3] = {0, 0, 0}, sang[3] = {0, 0, 0};
    int active = 0;
    for (int side=0; side < 2; side++) {
      int g = con->geom[side];
      if (g < 0) {
        // TODO(team): support flex
        continue;
      }
      const mjtNum* sv = m->geom_surfacevel + 6*g;

      // skip geom with no surface velocity
      if (!sv[0] && !sv[1] && !sv[2] && !sv[3] && !sv[4] && !sv[5]) {
        continue;
      }
      active = 1;

      // rotate to world frame, add angular contribution at contact point
      mjtNum sgn = side ? 1 : -1;
      mjtNum vw[3], ww[3];
      mj_geomSurfaceVelocity(m, d, g, con->pos, vw, ww);
      mji_addToScl3(svel, vw, sgn);
      mji_addToScl3(sang, ww, sgn);
    }
    if (active) {
      con->surfacevel[0] = mju_dot3(con->frame + 3, svel);
      con->surfacevel[1] = mju_dot3(con->frame + 6, svel);
      con->surfacevel[2] = mju_dot3(con->frame + 0, sang);
    }
  }

  // clear fields that are computed later
  con->efc_address = -1;
  con->mu = 0;

  // set deprecated fields
  con->geom1 = con->geom[0];
  con->geom2 = con->geom[1];
}


// struct for collision task
typedef struct {
  mjPreContact* conbuffer;  // pre-contact buffer returned by collision functions
  int* flexbuffer;          // buffer for flex vertex or element IDs
  int* nconbuffer;          // contact count for each collision pair
  char* epabuffer;          // buffer for nativeccd
  int ccd_size;             // size of nativeccd buffer
  mjcPair* pairbuffer;      // collision pairs
  int npair;                // number of collision pairs
  int chunksize;            // number of pairs to process per task
  int maxcon;               // maximum number of contacts (size of conbuffer)
  int user_collider;        // 1 if mjCOLLISIONFUNC differs from mjDEFAULTCOLLISIONFUNC
} mjContactArg;


// return the maximum number of contacts for a collision pair
static inline int pairMaxContact(const mjModel* m, const mjcPair* pair) {
  switch (pair->type) {
    case mjCPAIR_GEOM_GEOM: {
      int g1 = pair->geom_geom.g1;
      int g2 = pair->geom_geom.g2;
      if (pair->geom_geom.sub_g1 >= 0 || pair->geom_geom.sub_g2 >= 0) {
        return 1;
      }
      int ipair = pair->geom_geom.ipair;
      mjtNum margin = getMargin(m, g1, g2, ipair);
      mjtNum gap = getGap(m, g1, g2, ipair);
      return mj_maxContact(m, g1, g2, margin + gap > 0);
    }

    case mjCPAIR_GEOM_FLEX: {
      int g = pair->geom_flex.g;
      int f = pair->geom_flex.f;
      if (m->geom_type[g] == mjGEOM_PLANE) {
        return m->flex_vertnum[f] * (mj_rodBoxSize(m, f, NULL) ? 2 : 1);
      }
      return mjMAXCONPAIR;
    }

    case mjCPAIR_GEOM_ELEM: {
      int g = pair->geom_elem.g;
      int f = pair->geom_elem.f;
      int dim = m->flex_dim[f];
      int type = m->geom_type[g];
      if (type == mjGEOM_HFIELD && pair->geom_elem.sub_g >= 0) {
        return 1;
      }
      if (type == mjGEOM_HFIELD ||
          (dim == 2 && (type == mjGEOM_BOX || type == mjGEOM_CAPSULE))) {
        return mjMAXCONPAIR;
      }
      if (dim == 1 && type == mjGEOM_BOX && mj_rodBoxSize(m, f, NULL)) {
        return mjMAXCONPAIR;
      }
      if (dim == 1 && (type == mjGEOM_CAPSULE || type == mjGEOM_BOX)) {
        return 2;
      }
      return 1;
    }

    case mjCPAIR_ELEM_ELEM: {
      int f1 = pair->elem_elem.f1;
      int f2 = pair->elem_elem.f2;
      int dim1 = m->flex_dim[f1];
      int dim2 = m->flex_dim[f2];
      if (dim1 == 1 && dim2 == 1 && mj_rodBoxSize(m, f1, NULL) && mj_rodBoxSize(m, f2, NULL)) {
        return mjMAXCONPAIR;
      }
      return (dim1 == 1 && dim2 == 1) ? 2 : 1;
    }

    default:
      return 0;
  }
}


// return 1 if collision pair may use nativeccd buffer, 0 otherwise
// returning 0 for a CCD pair only costs performance (mjc_penetration falls
// back to per-call stack allocation when no buffer is bound), so the denylist
// is safe and new analytical colliders don't need to be added for correctness
static inline int pairNeedsCCD(const mjModel* m, const mjcPair* pair, int user_collider) {
  if (mjDISABLED(mjDSBL_NATIVECCD)) {
    return 0;
  }

  switch (pair->type) {
    case mjCPAIR_GEOM_GEOM: {
      if (speculativeEnabled(m)) {
        return 1;
      }
      int type1 = m->geom_type[pair->geom_geom.g1];
      int type2 = m->geom_type[pair->geom_geom.g2];
      if (!user_collider) {
        switch ((type1 << 4) | type2) {
#define MJ_CELL(t1, t2, fn) \
          case ((t1) << 4) | (t2): return (fn == mjc_Convex || fn == mjc_ConvexHField);
          mjCOLLISION_TABLE(MJ_COLLISION_ROW, 0)
#undef MJ_CELL
          default:
            return 0;
        }
      }
      mjfCollision fn = mjCOLLISIONFUNC[type1][type2];
      if (!fn ||
          fn == mjc_PlaneSphere || fn == mjc_PlaneCapsule ||
          fn == mjc_PlaneCylinder || fn == mjc_PlaneBox ||
          fn == mjc_PlaneConvex || fn == mjc_SphereSphere ||
          fn == mjc_SphereCapsule || fn == mjc_SphereCylinder ||
          fn == mjc_SphereBox || fn == mjc_CapsuleCapsule ||
          fn == mjc_CapsuleBox || fn == mjc_BoxBox ||
          fn == mjc_SDF || fn == mjc_HFieldSDF || fn == mjc_MeshSDF) {
        return 0;
      }
      return 1;
    }

    case mjCPAIR_GEOM_ELEM:
    case mjCPAIR_ELEM_ELEM:
      return 1;

    default:
      return 0;
  }
}


static void collisionTask(const mjModel* m, mjData* d, void* arg, int thread_id, int idx) {
  mjContactArg* conargs = (mjContactArg*)arg;
  mjPreContact* conbuffer = conargs->conbuffer;
  char* epabuffer = conargs->epabuffer;
  int chunksize = conargs->chunksize;
  int globalidx = chunksize * idx;
  mjcPair* pair = conargs->pairbuffer + globalidx;
  int* ncon = conargs->nconbuffer + chunksize * idx;

  int npair = conargs->npair;
  int n = mjMIN(chunksize, npair - globalidx);
  int user_collider = conargs->user_collider;

  if (epabuffer) {
    mjc_setCCDBuffer(epabuffer + thread_id * conargs->ccd_size);
  }
  for (int i = 0; i < n; i++) {
    int conpos = pair[i].conpos;

    switch (pair[i].type) {
      case mjCPAIR_GEOM_GEOM: {
        int g1 = pair[i].geom_geom.g1;
        int g2 = pair[i].geom_geom.g2;
        int ipair = pair[i].geom_geom.ipair;
        int sub_g1 = pair[i].geom_geom.sub_g1;
        int sub_g2 = pair[i].geom_geom.sub_g2;
        int type1 = m->geom_type[g1];
        int type2 = m->geom_type[g2];
        mjtNum margin = getMargin(m, g1, g2, ipair);
        mjtNum gap = getGap(m, g1, g2, ipair);
        mjPreContact* con = conbuffer + conpos;

        // TODO(kylebayes): batch contiguous (g1, g2) sub-geom runs within this chunk to reuse
        // mjCCDObj setup and run GJK in the hfield/mesh local frame once per run
        if (sub_g1 >= 0 || sub_g2 >= 0) {
          ncon[i] = mjc_SubConvex(m, d, con, g1, sub_g1, g2, sub_g2, margin + gap);
        } else if (user_collider) {
          mjfCollision fn = mjCOLLISIONFUNC[type1][type2];
          ncon[i] = fn ? fn(m, d, con, g1, g2, margin + gap) : 0;
        } else {
          switch ((type1 << 4) | type2) {
#define MJ_CELL(t1, t2, fn) \
            case ((t1) << 4) | (t2): ncon[i] = fn(m, d, con, g1, g2, margin + gap); break;
            mjCOLLISION_TABLE(MJ_COLLISION_ROW, MJ_NONE)
#undef MJ_CELL
            default:
              ncon[i] = 0;
              break;
          }
        }
        if (speculativeEnabled(m) && sub_g1 < 0 && sub_g2 < 0) {
          // check for active surface contact (in-gap contacts are only replaced if
          // CCD finds a surface hit)
          int active = 0;
          for (int k = 0; k < ncon[i]; k++) {
            if (conbuffer[conpos + k].dist < margin) {
              active = 1;
              break;
            }
          }
          mjtNum min_size = pairMinSize(m, g1, g2);
          if (!active && (geomStepSweep(m, d, g1, min_size, NULL, NULL) > 0 ||
                          geomStepSweep(m, d, g2, min_size, NULL, NULL) > 0)) {
            int nccd = mjc_advanceGeomPair(m, d, g1, g2, margin, conbuffer + conpos,
                                           epabuffer + thread_id * conargs->ccd_size);
            if (nccd > 0) {
              ncon[i] = nccd;
              pair[i].geom_geom.spec = 1;
            }
          }
        }
        break;
      }

      case mjCPAIR_GEOM_FLEX: {
        int g = pair[i].geom_flex.g;
        int f = pair[i].geom_flex.f;
        mjtNum margin = mj_assignMargin(m, m->geom_margin[g] + m->flex_margin[f]);
        mjtNum gap = m->geom_gap[g] + m->flex_gap[f];
        if (m->geom_type[g] == mjGEOM_PLANE) {
          ncon[i] = mjc_PlaneFlex(m, d, conbuffer + conpos, conargs->flexbuffer + conpos,
                                  g, f, margin + gap);
        } else if (m->geom_type[g] == mjGEOM_SDF) {
          ncon[i] = mjc_FlexSDF(m, d, conbuffer + conpos, conargs->flexbuffer + conpos,
                                g, f, margin + gap);
        } else {
          ncon[i] = 0;
        }
        break;
      }

      case mjCPAIR_GEOM_ELEM: {
        int g = pair[i].geom_elem.g;
        int sub_g = pair[i].geom_elem.sub_g;
        int f = pair[i].geom_elem.f;
        int e = pair[i].geom_elem.e;
        mjtNum margin = mj_assignMargin(m, m->geom_margin[g] + m->flex_margin[f]);
        mjtNum gap = m->geom_gap[g] + m->flex_gap[f];
        ncon[i] = mjc_SubGeomElem(m, d, conbuffer + conpos, g, sub_g, f, e, margin + gap);
        break;
      }

      case mjCPAIR_ELEM_ELEM: {
        int f1 = pair[i].elem_elem.f1;
        int e1 = pair[i].elem_elem.e1;
        int f2 = pair[i].elem_elem.f2;
        int e2 = pair[i].elem_elem.e2;
        mjtNum margin = mj_assignMargin(m, m->flex_margin[f1] + m->flex_margin[f2]);
        mjtNum gap = m->flex_gap[f1] + m->flex_gap[f2];
        if (f1 == f2) {
          margin = 0;
        }
        ncon[i] = mjc_ElemElem(m, d, conbuffer + conpos, f1, e1, f2, e2, margin + gap);
        break;
      }

      default:
        ncon[i] = 0;
        break;
    }

    // SHOULD NOT OCCUR
    int expected_max = (globalidx + i + 1 < npair ? pair[i+1].conpos : conargs->maxcon) - conpos;
    if (ncon[i] > expected_max) {
      mjERROR("collision function returned %d contacts, "
              "expected at most %d from pairMaxContact", ncon[i], expected_max);
    }
  }
  if (epabuffer) {
    mjc_setCCDBuffer(NULL);
  }
}


// populate contacts from mjPreContact array; allocate in arena, return 0 if arena full
static int addPairContacts(const mjModel* m, mjData* d, const mjPreContact* precon, int ncon,
                           const mjcPair* pair, const int* elem, const int* vert,
                           int total_demand) {
  if (!ncon) {
    return 1;
  }

  size_t align = _Alignof(mjContact);
  mjContact* con = (mjContact*) mj_arenaAllocByte(d, sizeof(mjContact) * ncon, align);
  int full = 0;
  if (!con) {
    mj_warning(d, mjWARN_CONTACTFULL, total_demand);
    full = 1;
    size_t padding = (align - (d->parena % align)) % align;
    size_t avail = d->narena - d->pstack;
    ncon = (d->parena + padding < avail)
           ? (int)((avail - d->parena - padding) / sizeof(mjContact)) : 0;
    if (!ncon) {
      return 0;
    }
    con = (mjContact*) mj_arenaAllocByte(d, sizeof(mjContact) * ncon, align);
    if (!con) {
      return 0;
    }
  }
  d->ncon += ncon;

  int condim = 0;
  mjtNum margin = 0;
  mjtNum friction[5] = {0}, solref[mjNREF] = {0}, solimp[mjNIMP] = {0}, adhesion = 0;
  mjtNum solreffriction[mjNREF] = {0};

  int g1 = -1, g2 = -1, f1 = -1, f2 = -1;
  int e1 = -1, e2 = -1, v1 = -1, v2 = -1;
  int spec = 0;

  switch (pair->type) {
    case mjCPAIR_GEOM_GEOM: {
      g1 = pair->geom_geom.g1;
      g2 = pair->geom_geom.g2;
      int ipair = pair->geom_geom.ipair;
      spec = pair->geom_geom.spec;
      margin = getMargin(m, g1, g2, ipair);
      if (ipair >= 0) {
        condim = m->pair_dim[ipair];
        mju_copy(solref, m->pair_solref + mjNREF * ipair, mjNREF);
        mju_copy(solimp, m->pair_solimp + mjNIMP * ipair, mjNIMP);
        mju_copy(friction, m->pair_friction + 5 * ipair, 5);
        adhesion = m->pair_adhesion[ipair];
        if (m->pair_solreffriction[mjNREF * ipair] || m->pair_solreffriction[mjNREF * ipair + 1]) {
          mju_copy(solreffriction, m->pair_solreffriction + mjNREF * ipair, mjNREF);
        }
      } else {
        mj_contactParam(m, &condim, solref, solimp, friction, &adhesion, g1, g2, -1, -1);
      }
      break;
    }

    case mjCPAIR_GEOM_FLEX:
      g1 = pair->geom_flex.g;
      f2 = pair->geom_flex.f;
      margin = mj_assignMargin(m, m->geom_margin[g1] + m->flex_margin[f2]);
      mj_contactParam(m, &condim, solref, solimp, friction, &adhesion, g1, -1, -1, f2);
      break;

    case mjCPAIR_GEOM_ELEM:
      g1 = pair->geom_elem.g;
      f2 = pair->geom_elem.f;
      e2 = pair->geom_elem.e;
      margin = mj_assignMargin(m, m->geom_margin[g1] + m->flex_margin[f2]);
      mj_contactParam(m, &condim, solref, solimp, friction, &adhesion, g1, -1, -1, f2);
      break;

    case mjCPAIR_ELEM_ELEM:
      f1 = pair->elem_elem.f1;
      e1 = pair->elem_elem.e1;
      f2 = pair->elem_elem.f2;
      e2 = pair->elem_elem.e2;
      margin = (f1 == f2) ? 0 : mj_assignMargin(m, m->flex_margin[f1] + m->flex_margin[f2]);
      mj_contactParam(m, &condim, solref, solimp, friction, &adhesion, -1, -1, f1, f2);
      break;
  }

  for (int i = 0; i < ncon; i++) {
    con[i].dist = precon[i].dist;
    mji_copy3(con[i].pos, precon[i].pos);
    mji_copy3(con[i].frame + 0, precon[i].normal);
    mji_copy3(con[i].frame + 3, precon[i].tangent);

    con[i].geom[0] = g1;
    con[i].geom[1] = g2;
    con[i].flex[0] = f1;
    con[i].flex[1] = f2;
    con[i].elem[0] = e1;
    con[i].elem[1] = elem ? elem[i] : e2;
    con[i].vert[0] = v1;
    con[i].vert[1] = vert ? vert[i] : v2;

    mj_setContact(m, d, con + i, condim, margin, solref, solreffriction, solimp, friction,
                  adhesion, spec);
  }
  return !full;
}


// filter precontacts across a group of collision pairs down to mjMAXCONPAIR
// the pairs of a group are contiguous, the first one is at start
// removes unselected precontacts in-place, resets group to -1, and updates nconbuffer
static inline void filterPreContacts(mjData* d, mjContactArg* arg, int start, int group) {
  // find the end of the group, reset group and count precontacts
  int end = start;
  int count = 0;
  while (end < arg->npair && arg->pairbuffer[end].group == group) {
    arg->pairbuffer[end].group = -1;
    count += arg->nconbuffer[end];
    end++;
  }

  // if total contacts <= mjMAXCONPAIR, return
  if (count <= mjMAXCONPAIR) {
    return;
  }

  mj_markStack(d);
  mjtByte* selected = mjSTACKALLOC(d, count, mjtByte);
  mjtNum* min_dist = mjSTACKALLOC(d, count, mjtNum);
  memset(selected, 0, count);
  for (int i = 0; i < count; i++) {
    min_dist[i] = mjMAXVAL;
  }

  // start with deepest penetrating pre-contact (skip uninitialized pre-contacts)
  int best = -1, best_k = -1, best_c = -1;
  mjtNum bestdist = -mjMAXVAL;
  int idx = 0;
  for (int k = start; k < end; k++) {
    int conpos = arg->pairbuffer[k].conpos;
    for (int c = 0; c < arg->nconbuffer[k]; c++, idx++) {
      mjtNum dist = -arg->conbuffer[conpos + c].dist;
      if (dist > bestdist) {
        bestdist = dist;
        best = idx;
        best_k = k;
        best_c = c;
      }
    }
  }

  // greedy furthest-point selection
  int nselected = 0;
  while (nselected < mjMAXCONPAIR && best >= 0) {
    selected[best] = 1;
    const mjtNum* bestpos = arg->conbuffer[arg->pairbuffer[best_k].conpos + best_c].pos;

    int nextbest = -1, nextbest_k = -1, nextbest_c = -1;
    mjtNum nextbestdist = -1;
    idx = 0;
    for (int k = start; k < end; k++) {
      int conpos = arg->pairbuffer[k].conpos;
      for (int c = 0; c < arg->nconbuffer[k]; c++, idx++) {
        if (selected[idx]) {
          continue;
        }

        const mjtNum* pos = arg->conbuffer[conpos + c].pos;
        mjtNum dx = pos[0] - bestpos[0];
        mjtNum dy = pos[1] - bestpos[1];
        mjtNum dz = pos[2] - bestpos[2];
        mjtNum d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < min_dist[idx]) {
          min_dist[idx] = d2;
        }
        if (min_dist[idx] > nextbestdist) {
          nextbestdist = min_dist[idx];
          nextbest = idx;
          nextbest_k = k;
          nextbest_c = c;
        }
      }
    }

    nselected++;
    best = nextbest;
    best_k = nextbest_k;
    best_c = nextbest_c;
  }

  // compact selected precontacts in-place and update nconbuffer
  idx = 0;
  for (int k = start; k < end; k++) {
    int conpos = arg->pairbuffer[k].conpos;
    int new_ncon = 0;
    for (int c = 0; c < arg->nconbuffer[k]; c++, idx++) {
      if (selected[idx]) {
        if (new_ncon != c) {
          arg->conbuffer[conpos + new_ncon] = arg->conbuffer[conpos + c];
          if (arg->flexbuffer) {
            arg->flexbuffer[conpos + new_ncon] = arg->flexbuffer[conpos + c];
          }
        }
        new_ncon++;
      }
    }
    arg->nconbuffer[k] = new_ncon;
  }

  mj_freeStack(d);
}


// compute contacts for a batch of collision pairs on the arena starting at parena
static void mj_narrowphase(const mjModel* m, mjData* d, int npair, size_t parena) {
  int nthread = mju_numThread(d);
  // try to balance load of 5 chunks per thread
  int chunksize = (nthread == 1) ? npair : mjMAX(1, npair / (5 * nthread));

  int nchunk = (npair + chunksize - 1) / chunksize;

  const mjcPair* buffer = (const mjcPair*) ((char*) d->arena + parena);

  mj_markStack(d);

  // buffer for pair data
  mjcPair* pairbuffer = mjSTACKALLOC(d, npair, mjcPair);
  int maxcon = 0;
  int has_flex = 0;
  int has_ccd = 0;
  int user_collider = !isDefaultCollisionFunc();
  for (int i = 0; i < npair; i++) {
    pairbuffer[i] = buffer[i];
    pairbuffer[i].conpos = maxcon;
    maxcon += pairMaxContact(m, buffer + i);
    if (buffer[i].type == mjCPAIR_GEOM_FLEX) {
      has_flex = 1;
    }
    if (!has_ccd && pairNeedsCCD(m, buffer + i, user_collider)) {
      has_ccd = 1;
    }
  }

  int ccd_size = 0;
  if (has_ccd) {
    int npolygonmax = mjDISABLED(mjDSBL_MULTICCD) ? 0 : m->npolygonmax;
    int nmeshdegmax = mjDISABLED(mjDSBL_MULTICCD) ? 0 : m->nmeshdegmax;
    ccd_size = mjc_ccdSize(npolygonmax, nmeshdegmax, m->opt.ccd_iterations);
  }

  // buffer data has been copied to metadata on the stack;
  // reclaim arena space so contacts can overwrite the buffer region
  d->parena = parena;

  mjContactArg arg;
  arg.ccd_size = ccd_size;
  arg.pairbuffer = pairbuffer;
  arg.nconbuffer = mjSTACKALLOC(d, npair, int);
  arg.conbuffer = mjSTACKALLOC(d, maxcon, mjPreContact);
  arg.flexbuffer = has_flex ? mjSTACKALLOC(d, maxcon, int) : NULL;
  arg.npair = npair;
  arg.chunksize = chunksize;
  arg.maxcon = maxcon;
  arg.user_collider = user_collider;

  // dispatch narrowphase to threads with local stack allocation for EPA
  {
    mj_markStack(d);
    int nactive = (nchunk < 2) ? 1 : nthread;
    arg.epabuffer = has_ccd ? mj_stackAllocByte(d, ccd_size * nactive, sizeof(mjtNum)) : NULL;
    mju_dispatch(m, d, collisionTask, &arg, nchunk);
    mj_freeStack(d);
  }

  // filter contact groups and compute total unclamped contact demand
  int lastgroup = -1;
  int total_demand = d->ncon;
  for (int i = 0; i < npair; i++) {
    if (pairbuffer[i].group >= 0) {
      // groups are contiguous and increasing: SHOULD NOT OCCUR
      if (pairbuffer[i].group <= lastgroup) {
        mjERROR("filter group %d is not contiguous", pairbuffer[i].group);
      }
      lastgroup = pairbuffer[i].group;
      filterPreContacts(d, &arg, i, pairbuffer[i].group);
    }
    total_demand += arg.nconbuffer[i];
  }

  // fill in contact data in strict prefix order
  for (int i = 0; i < npair; i++) {
    int ncon = arg.nconbuffer[i];
    if (!ncon) {
      continue;
    }

    const int* vert = (pairbuffer[i].type == mjCPAIR_GEOM_FLEX &&
                       m->geom_type[pairbuffer[i].geom_flex.g] == mjGEOM_PLANE)
                      ? arg.flexbuffer + pairbuffer[i].conpos : NULL;
    const int* elem = (pairbuffer[i].type == mjCPAIR_GEOM_FLEX &&
                       m->geom_type[pairbuffer[i].geom_flex.g] == mjGEOM_SDF)
                      ? arg.flexbuffer + pairbuffer[i].conpos : NULL;
    if (!addPairContacts(m, d, arg.conbuffer + pairbuffer[i].conpos, ncon,
                         pairbuffer + i, elem, vert, total_demand)) {
      break;
    }
  }
  mj_freeStack(d);
}


// test active element self-collisions with SAP, pushing pairs onto the arena
static void mj_collideFlexSAPPairs(const mjModel* m, mjData* d, mjcPairArena* arena,
                                   int f, int group) {
  mj_markStack(d);

  // allocate and construct active element ids
  int* elid = mjSTACKALLOC(d, m->flex_elemnum[f], int);
  int nactive = 0;
  int flex_elemnum = m->flex_elemnum[f];
  for (int i=0; i < flex_elemnum; i++) {
    if (mj_isElemActive(m, f, i)) {
      elid[nactive++] = i;
    }
  }

  // nothing active
  if (nactive < 2) {
    mj_freeStack(d);
    return;
  }

  // allocate and construct AAMMs for active elements
  mjtNum* aamm = mjSTACKALLOC(d, 6*nactive, mjtNum);
  const mjtNum* elemaabb = d->flexelem_aabb + 6*m->flex_elemadr[f];
  for (int i=0; i < nactive; i++) {
    const mjtNum* center = elemaabb+6*elid[i];
    const mjtNum* radius = elemaabb+6*elid[i]+3;
    aamm[nactive*0 + i] = center[0] - radius[0];
    aamm[nactive*1 + i] = center[1] - radius[1];
    aamm[nactive*2 + i] = center[2] - radius[2];
    aamm[nactive*3 + i] = center[0] + radius[0];
    aamm[nactive*4 + i] = center[1] + radius[1];
    aamm[nactive*5 + i] = center[2] + radius[2];
  }

  // select largest axis from flex bvh
  const mjtNum* bvh = d->bvh_aabb_dyn + 6*(m->flex_bvhadr[f] - m->nbvhstatic);
  int axis = (bvh[3] > bvh[4] && bvh[3] > bvh[5]) ? 0 : (bvh[4] > bvh[5] ? 1 : 2);

  // call SAP; hard limit on number of pairs to avoid out-of-memory
  int maxsappair = mjMIN(nactive*(nactive-1)/2, 1000000);
  mjPacked32* sappair = mjSTACKALLOC(d, maxsappair, mjPacked32);
  int nsappair = mj_SAP(d, aamm, NULL, nactive, axis, 1, sappair, maxsappair);
  if (nsappair < 0) {
    mjERROR("SAP failed");
  }
  if (nsappair > maxsappair) {
    mj_warning(d, mjWARN_CONTACTFULL, maxsappair);
    nsappair = maxsappair;
  }

  // push SAP pairs
  for (int i=0; i < nsappair; i++) {
    int e1 = elid[sappair[i].hi];
    int e2 = elid[sappair[i].lo];
    pushSelfElemElem(m, d, arena, f, e1, e2, group);
  }

  mj_freeStack(d);
}


// test active element self-collisions of an interpolated flex by grouping elements into cells
static void mj_collideFlexCellPairs(const mjModel* m, mjData* d, mjcPairArena* arena,
                                    int f, int group) {
  const int* cellnum = m->flex_cellnum + 3*f;
  int nx = cellnum[0], ny = cellnum[1], nz = cellnum[2];
  int ncell = nx * ny * nz;
  int nvert = m->flex_vertnum[f];
  int nelem = m->flex_elemnum[f];
  int dim = m->flex_dim[f];
  if (nelem < 2 || ncell < 1) {
    return;
  }

  mj_markStack(d);
  int start_pair = (int)((d->parena - arena->parena) / sizeof(mjcPair));

  // compute cell range [lo, hi] once per vertex
  int* vert_lo = mjSTACKALLOC(d, 3*nvert, int);
  int* vert_hi = mjSTACKALLOC(d, 3*nvert, int);
  for (int v = 0; v < nvert; v++) {
    mjc_vertCellRange(m, f, v, vert_lo + 3*v, vert_hi + 3*v);
  }

  // assign each active element to the lo-cell of its first vertex, and record its [elo, ehi] range
  int* elem_cell = mjSTACKALLOC(d, nelem, int);
  int* elem_lo = mjSTACKALLOC(d, 3*nelem, int);
  int* elem_hi = mjSTACKALLOC(d, 3*nelem, int);
  int* cell_count = mjSTACKALLOC(d, ncell, int);
  mju_zeroInt(cell_count, ncell);

  const int* elemdata = m->flex_elem + m->flex_elemdataadr[f];
  int nactive = 0;
  for (int e = 0; e < nelem; e++) {
    if (!mj_isElemActive(m, f, e)) {
      elem_cell[e] = -1;
      continue;
    }
    const int* edata = elemdata + e*(dim + 1);
    int v0 = edata[0];
    int elo[3], ehi[3];
    for (int k = 0; k < 3; k++) {
      elo[k] = vert_lo[3*v0 + k];
      ehi[k] = vert_hi[3*v0 + k];
    }
    int c = (elo[0]*ny + elo[1])*nz + elo[2];
    for (int i = 1; i <= dim; i++) {
      int v = edata[i];
      for (int k = 0; k < 3; k++) {
        elo[k] = mjMIN(elo[k], vert_lo[3*v + k]);
        ehi[k] = mjMAX(ehi[k], vert_hi[3*v + k]);
      }
    }
    elem_cell[e] = c;
    for (int k = 0; k < 3; k++) {
      elem_lo[3*e + k] = elo[k];
      elem_hi[3*e + k] = ehi[k];
    }
    cell_count[c]++;
    nactive++;
  }

  if (nactive < 2) {
    mj_freeStack(d);
    return;
  }

  // compact non-empty cells and build CSR offsets into cell_elems
  int nnonempty = 0;
  for (int c = 0; c < ncell; c++) {
    if (cell_count[c] > 0) {
      nnonempty++;
    }
  }
  int* cell_slot = mjSTACKALLOC(d, ncell, int);
  int* cell_offset = mjSTACKALLOC(d, nnonempty + 1, int);
  int* cell_cursor = mjSTACKALLOC(d, nnonempty, int);
  int* cell_coord = mjSTACKALLOC(d, 3*nnonempty, int);
  int* cell_lo = mjSTACKALLOC(d, 3*nnonempty, int);
  int* cell_hi = mjSTACKALLOC(d, 3*nnonempty, int);
  mjtNum* cell_min = mjSTACKALLOC(d, 3*nnonempty, mjtNum);
  mjtNum* cell_max = mjSTACKALLOC(d, 3*nnonempty, mjtNum);
  mjtNum* cell_aabb = mjSTACKALLOC(d, 6*nnonempty, mjtNum);

  int offset = 0, slot = 0;
  for (int c = 0; c < ncell; c++) {
    if (cell_count[c] > 0) {
      cell_slot[c] = slot;
      cell_offset[slot] = offset;
      cell_cursor[slot] = offset;
      offset += cell_count[c];

      int cx = c / (ny * nz);
      int cy = (c / nz) % ny;
      int cz = c % nz;
      cell_coord[3*slot + 0] = cx;
      cell_coord[3*slot + 1] = cy;
      cell_coord[3*slot + 2] = cz;
      cell_lo[3*slot + 0] = cx;
      cell_lo[3*slot + 1] = cy;
      cell_lo[3*slot + 2] = cz;
      cell_hi[3*slot + 0] = cx;
      cell_hi[3*slot + 1] = cy;
      cell_hi[3*slot + 2] = cz;
      for (int k = 0; k < 3; k++) {
        cell_min[3*slot + k] = mjMAXVAL;
        cell_max[3*slot + k] = -mjMAXVAL;
      }
      slot++;
    } else {
      cell_slot[c] = -1;
    }
  }
  cell_offset[nnonempty] = offset;

  // populate cell_elems, cell_lo/hi, and cell bounding boxes in a single O(nelem) pass
  int* cell_elems = mjSTACKALLOC(d, nactive, int);
  const mjtNum* elemaabb = d->flexelem_aabb + 6*m->flex_elemadr[f];
  for (int e = 0; e < nelem; e++) {
    int c = elem_cell[e];
    if (c < 0) {
      continue;
    }
    int s = cell_slot[c];
    cell_elems[cell_cursor[s]++] = e;
    for (int k = 0; k < 3; k++) {
      cell_lo[3*s + k] = mjMIN(cell_lo[3*s + k], elem_lo[3*e + k]);
      cell_hi[3*s + k] = mjMAX(cell_hi[3*s + k], elem_hi[3*e + k]);
      mjtNum center = elemaabb[6*e + k];
      mjtNum radius = elemaabb[6*e + 3 + k];
      cell_min[3*s + k] = mju_min(cell_min[3*s + k], center - radius);
      cell_max[3*s + k] = mju_max(cell_max[3*s + k], center + radius);
    }
  }

  for (int s = 0; s < nnonempty; s++) {
    for (int k = 0; k < 3; k++) {
      cell_aabb[6*s + k] = 0.5 * (cell_max[3*s + k] + cell_min[3*s + k]);
      cell_aabb[6*s + 3 + k] = 0.5 * (cell_max[3*s + k] - cell_min[3*s + k]);
    }
  }

  // test pairs of non-empty cells (interpolated cages are coarse in practice, typically 3^3 to 6^3)
  for (int s1 = 0; s1 < nnonempty; s1++) {
    const int* coord1 = cell_coord + 3*s1;
    const int* lo1 = cell_lo + 3*s1;
    const int* hi1 = cell_hi + 3*s1;
    const mjtNum* aabb1 = cell_aabb + 6*s1;
    int start1 = cell_offset[s1], end1 = cell_offset[s1 + 1];

    // elements in the same cell s1 share a node via their first vertex (both have vert_lo == coord1)
    for (int s2 = s1 + 1; s2 < nnonempty; s2++) {
      const int* coord2 = cell_coord + 3*s2;

      // if first-vertex cells are neighbors (|coord1 - coord2|_inf <= 1), every pair is excluded
      if (mjc_cellRangeShared(coord1, coord1, coord2, coord2)) {
        continue;
      }

      // skip if cell AABBs do not overlap
      if (filterBox(aabb1, cell_aabb + 6*s2, 0)) {
        continue;
      }

      // if cell ranges are separated by > 1 along some axis, no pair in (s1, s2) shares a node
      const int* lo2 = cell_lo + 3*s2;
      const int* hi2 = cell_hi + 3*s2;
      int disjoint = !mjc_cellRangeShared(lo1, hi1, lo2, hi2);

      int start2 = cell_offset[s2], end2 = cell_offset[s2 + 1];
      for (int i1 = start1; i1 < end1; i1++) {
        int e1 = cell_elems[i1];
        const mjtNum* eaabb1 = elemaabb + 6*e1;
        for (int i2 = start2; i2 < end2; i2++) {
          int e2 = cell_elems[i2];
          if (!filterBox(eaabb1, elemaabb + 6*e2, 0)) {
            int emin = mjMIN(e1, e2);
            int emax = mjMAX(e1, e2);
            if (!disjoint) {
              // check if any vertex of e1 and any vertex of e2 share a cell node
              const int* edata1 = elemdata + e1*(dim + 1);
              const int* edata2 = elemdata + e2*(dim + 1);
              int shared = 0;
              for (int v1 = 0; v1 <= dim && !shared; v1++) {
                const int* vlo1 = vert_lo + 3*edata1[v1];
                const int* vhi1 = vert_hi + 3*edata1[v1];
                for (int v2 = 0; v2 <= dim; v2++) {
                  const int* vlo2 = vert_lo + 3*edata2[v2];
                  const int* vhi2 = vert_hi + 3*edata2[v2];
                  if (mjc_cellRangeShared(vlo1, vhi1, vlo2, vhi2)) {
                    shared = 1;
                    break;
                  }
                }
              }
              if (shared) {
                continue;
              }
            }
            pushElemElem(d, arena, f, emin, f, emax, group);
          }
        }
      }
    }
  }

  // sort pairs generated by this flex self-collision
  int end_pair = (int)((d->parena - arena->parena) / sizeof(mjcPair));
  int n = end_pair - start_pair;
  if (n > 1) {
    mjcPair* buf = mjSTACKALLOC(d, n, mjcPair);
    mjcPair* pairs = (mjcPair*) ((char*) d->arena + arena->parena);
    pairSort(pairs + start_pair, buf, n, (void*)m);
  }

  mj_freeStack(d);
}
