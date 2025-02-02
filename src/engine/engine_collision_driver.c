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
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_callback.h"
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_collision_sdf.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_sort.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_spatial.h"


// table of pair-wise collision functions
mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES] = {
  /*              PLANE  HFIELD  SPHERE            CAPSULE             ELLIPSOID         CYLINDER            BOX               MESH              SDF */
  /*PLANE     */ {0,     0,      mjc_PlaneSphere,  mjc_PlaneCapsule,   mjc_PlaneConvex,  mjc_PlaneCylinder,  mjc_PlaneBox,     mjc_PlaneConvex,  mjc_PlaneConvex},
  /*HFIELD    */ {0,     0,      mjc_ConvexHField, mjc_ConvexHField,   mjc_ConvexHField, mjc_ConvexHField,   mjc_ConvexHField, mjc_ConvexHField, mjc_HFieldSDF},
  /*SPHERE    */ {0,     0,      mjc_SphereSphere, mjc_SphereCapsule,  mjc_Convex,       mjc_SphereCylinder, mjc_SphereBox,    mjc_Convex,       mjc_SDF},
  /*CAPSULE   */ {0,     0,      0,                mjc_CapsuleCapsule, mjc_Convex,       mjc_Convex,         mjc_CapsuleBox,   mjc_Convex,       mjc_SDF},
  /*ELLIPSOID */ {0,     0,      0,                0,                  mjc_Convex,       mjc_Convex,         mjc_Convex,       mjc_Convex,       mjc_SDF},
  /*CYLINDER  */ {0,     0,      0,                0,                  0,                mjc_Convex,         mjc_Convex,       mjc_Convex,       mjc_SDF},
  /*BOX       */ {0,     0,      0,                0,                  0,                0,                  mjc_BoxBox,       mjc_Convex,       mjc_SDF},
  /*MESH      */ {0,     0,      0,                0,                  0,                0,                  0,                mjc_Convex,       mjc_MeshSDF},
  /*SDF       */ {0,     0,      0,                0,                  0,                0,                  0,                0,                mjc_SDF}
};



//------------------------------------ utility functions ------------------------------------------

// move arena pointer back to the end of the contact array
static inline void resetArena(mjData* d) {
  d->parena = d->ncon * sizeof(mjContact);
#ifdef ADDRESS_SANITIZER
  if (!d->threadpool) {
    ASAN_POISON_MEMORY_REGION(
      (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
  }
#endif
}



// plane to geom_center squared distance, g1 is a plane
static mjtNum planeGeomDist(const mjModel* m, mjData* d, int g1, int g2) {
  mjtNum* mat1 = d->geom_xmat + 9*g1;
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3];

  mju_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
  return mju_dot3(dif, norm);
}



// return 1 if body has plane geom, 0 otherwise
static int hasPlane(const mjModel* m, int body) {
  int start = m->body_geomadr[body];
  int end = m->body_geomadr[body] + m->body_geomnum[body];

  // scan geoms belonging to body
  int g;
  for (g=start; g < end; g++) {
    if (m->geom_type[g] == mjGEOM_PLANE) {
      return 1;
    }
  }

  return 0;
}



// filter contact based on type and affinity
static int filterBitmask(int contype1, int conaffinity1,
                         int contype2, int conaffinity2) {
  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}



// filter contact based on global AABBs
static int filterBox(const mjtNum aabb1[6], const mjtNum aabb2[6], mjtNum margin) {
  if (aabb1[0]+aabb1[3]+margin < aabb2[0]-aabb2[3]) return 1;
  if (aabb1[1]+aabb1[4]+margin < aabb2[1]-aabb2[4]) return 1;
  if (aabb1[2]+aabb1[5]+margin < aabb2[2]-aabb2[5]) return 1;
  if (aabb2[0]+aabb2[3]+margin < aabb1[0]-aabb1[3]) return 1;
  if (aabb2[1]+aabb2[4]+margin < aabb1[1]-aabb1[4]) return 1;
  if (aabb2[2]+aabb2[5]+margin < aabb1[2]-aabb1[5]) return 1;
  return 0;
}



// filter contact based sphere-box test, treating sphere as box
static int filterSphereBox(const mjtNum s[3], mjtNum bound, const mjtNum aabb[6]) {
  if (s[0]+bound < aabb[0]-aabb[3]) return 1;
  if (s[1]+bound < aabb[1]-aabb[4]) return 1;
  if (s[2]+bound < aabb[2]-aabb[5]) return 1;
  if (s[0]-bound > aabb[0]+aabb[3]) return 1;
  if (s[1]-bound > aabb[1]+aabb[4]) return 1;
  if (s[2]-bound > aabb[2]+aabb[5]) return 1;
  return 0;
}



// filter contact based on bounding sphere test (raw)
static int filterSphere(const mjtNum pos1[3], const mjtNum pos2[3], mjtNum bound) {
  mjtNum dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  mjtNum distsqr = dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2];

  return (distsqr > bound*bound);
}



// filter contact based on bounding sphere test
static int mj_filterSphere(const mjModel* m, mjData* d, int g1, int g2, mjtNum margin) {
  // neither geom is a plane
  if (m->geom_rbound[g1] > 0 && m->geom_rbound[g2] > 0) {
    return filterSphere(d->geom_xpos + 3*g1, d->geom_xpos + 3*g2,
                        m->geom_rbound[g1] + m->geom_rbound[g2] + margin);
  }

  // one geom is a plane
  if (m->geom_type[g1] == mjGEOM_PLANE && m->geom_rbound[g2] > 0
      && planeGeomDist(m, d, g1, g2) > margin + m->geom_rbound[g2]) {
    return 1;
  }
  if (m->geom_type[g2] == mjGEOM_PLANE && m->geom_rbound[g1] > 0
      && planeGeomDist(m, d, g2, g1) > margin + m->geom_rbound[g1]) {
    return 1;
  }
  return 0;
}



// filter body pair: 1- discard, 0- proceed
static int filterBodyPair(int weldbody1, int weldparent1, int weldbody2,
                          int weldparent2, int dsbl_filterparent) {
  // same weldbody check
  if (weldbody1 == weldbody2) {
    return 1;
  }

  // weldparent check
  if ((!dsbl_filterparent && weldbody1 != 0 && weldbody2 != 0) &&
      (weldbody1 == weldparent2 || weldbody2 == weldparent1)) {
    return 1;
  }

  // all tests passed
  return 0;
}



// return 1 if bodyflex can collide, 0 otherwise
static int canCollide(const mjModel* m, int bf) {
  if (bf < m->nbody) {
    return (m->body_contype[bf] || m->body_conaffinity[bf]);
  } else {
    int f = bf - m->nbody;
    return (m->flex_contype[f] || m->flex_conaffinity[f]);
  }
}



// return 1 if two bodyflexes can collide, 0 otherwise
static int canCollide2(const mjModel* m, int bf1, int bf2) {
  int nbody = m->nbody;
  int contype1 = (bf1 < nbody) ? m->body_contype[bf1] : m->flex_contype[bf1-nbody];
  int conaffinity1 = (bf1 < nbody) ? m->body_conaffinity[bf1] : m->flex_conaffinity[bf1-nbody];
  int contype2 = (bf2 < nbody) ? m->body_contype[bf2] : m->flex_contype[bf2-nbody];
  int conaffinity2 = (bf2 < nbody) ? m->body_conaffinity[bf2] : m->flex_conaffinity[bf2-nbody];

  // opposite of bitmask filter
  return (!filterBitmask(contype1, conaffinity1, contype2, conaffinity2));
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

// compare contact pairs by their geom/elem/vert IDs
static inline int contactcompare(const mjContact* c1, const mjContact* c2, void* context) {
  const mjModel* m = (const mjModel*) context;

  // get colliding object ids
  int con1_obj1 = c1->geom[0] >= 0 ? c1->geom[0] : (c1->elem[0] >= 0 ? c1->elem[0] : c1->vert[0]);
  int con1_obj2 = c1->geom[1] >= 0 ? c1->geom[1] : (c1->elem[1] >= 0 ? c1->elem[1] : c1->vert[1]);
  int con2_obj1 = c2->geom[0] >= 0 ? c2->geom[0] : (c2->elem[0] >= 0 ? c2->elem[0] : c2->vert[0]);
  int con2_obj2 = c2->geom[1] >= 0 ? c2->geom[1] : (c2->elem[1] >= 0 ? c2->elem[1] : c2->vert[1]);

  // for geom:geom, reproduce the order of contacts without mj_collideTree
  // normally sorted by (g1, g2), but in mj_collideGeoms, g1 and g2 are swapped based on geom_type
  // here we undo this swapping for the purpose of sorting - needs to be done for each mjContact
  if (c1->geom[0] >= 0 && c1->geom[1] >= 0 &&
      c2->geom[0] >= 0 && c2->geom[1] >= 0) {
    if (m->geom_type[con1_obj1] > m->geom_type[con1_obj2]) {
      int tmp = con1_obj1;
      con1_obj1 = con1_obj2;
      con1_obj2 = tmp;
    }
    if (m->geom_type[con2_obj1] > m->geom_type[con2_obj2]) {
      int tmp = con2_obj1;
      con2_obj1 = con2_obj2;
      con2_obj2 = tmp;
    }
  }
  if (con1_obj1 < con2_obj1) return -1;
  if (con1_obj1 > con2_obj1) return 1;
  if (con1_obj2 < con2_obj2) return -1;
  if (con1_obj2 > con2_obj2) return 1;
  return 0;
}

// define contactSort function for sorting contacts
mjSORT(contactSort, mjContact, contactcompare)



// main collision function
void mj_collision(const mjModel* m, mjData* d) {
  TM_START1;

  int nexclude = m->nexclude, npair = m->npair, nbody = m->nbody;
  int nbodyflex = m->nbody + m->nflex;

  // reset the size of the contact array and invalidate efc arrays
  d->ncon = 0;
  resetArena(d);
  mj_clearEfc(d);

  // reset the visualization flags
  if (m->vis.global.bvactive) {
    memset(d->bvh_active, 0, m->nbvh);
  }

  // return if disabled
  if (mjDISABLED(mjDSBL_CONSTRAINT) || mjDISABLED(mjDSBL_CONTACT)
      || m->nconmax == 0 || nbodyflex < 2) {
    return;
  }

  mj_markStack(d);

  // broadphase collision detector
  TM_START;
  int nmaxpairs = (nbodyflex*(nbodyflex - 1))/2;
  int* broadphasepair = mjSTACKALLOC(d, nmaxpairs, int);
  int nbfpair = mj_broadphase(m, d, broadphasepair, nmaxpairs);
  unsigned int last_signature = -1;
  TM_END(mjTIMER_COL_BROAD);

  // narrowphase and midphase collision detector
  TM_RESTART;

  // process bodyflex pairs returned by broadphase, merge with predefined geom pairs
  int pairadr = 0;
  for (int i=0; i < nbfpair; i++) {
    // reconstruct bodyflex pair ids
    int bf1 = (broadphasepair[i]>>16) & 0xFFFF;
    int bf2 = broadphasepair[i] & 0xFFFF;

    // compute signature for this bodyflex pair
    unsigned int signature = (bf1<<16) + bf2;
    // pairs come sorted by signature, but may not be unique
    // if signature is repeated, skip it
    if (signature == last_signature) {
      continue;
    }
    last_signature = signature;

    // merge predefined geom pairs
    int merged = 0;
    int startadr = pairadr;
    if (npair) {
      // test all predefined pairs for which pair_signature<=signature
      while (pairadr < npair && m->pair_signature[pairadr] <= signature) {
        if (m->pair_signature[pairadr] == signature) {
          merged = 1;
        }
        mj_collideGeoms(m, d, pairadr++, -1);
      }
    }

    // apply bitmask filtering at the bodyflex level
    if (!canCollide2(m, bf1, bf2)) {
      continue;
    }

    // handle body pair exclusion
    int exadr = 0;
    if (nexclude) {
      // advance exadr while exclude_signature < signature
      while (exadr < nexclude && m->exclude_signature[exadr] < signature) {
        exadr++;
      }

      // skip this bodyflex pair if its signature is found in exclude array
      if (exadr < nexclude && m->exclude_signature[exadr] == signature) {
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

    // process bodyflex pair: two single-geom bodies
    if (isbody1 && isbody2 && m->body_geomnum[bf1] == 1 && m->body_geomnum[bf2] == 1) {
      mj_collideGeomPair(m, d, geomadr1, geomadr2, merged, startadr, pairadr);
    }

    // process bodyflex pair: midphase
    else if (!mjDISABLED(mjDSBL_MIDPHASE) && bvh1 >= 0 && bvh2 >= 0) {
      int ncon_before = d->ncon;
      mj_collideTree(m, d, bf1, bf2, merged, startadr, pairadr);
      int ncon_after = d->ncon;

      // sort contacts
      int n = ncon_after - ncon_before;
      if (n > 1) {
        mj_markStack(d);
        mjContact* buf = mjSTACKALLOC(d, n, mjContact);
        contactSort(d->contact + ncon_before, buf, n, (void*)m);
        mj_freeStack(d);
      }
    }

    // process bodyflex pair: all-to-all
    else {
      int geomadr_end1 = geomadr1 + m->body_geomnum[bf1];
      int geomadr_end2 = geomadr2 + m->body_geomnum[bf2];

      // body : body
      if (isbody1 && isbody2) {
        for (int g1=geomadr1; g1 < geomadr_end1; g1++) {
          for (int g2=geomadr2; g2 < geomadr_end2; g2++) {
            mj_collideGeomPair(m, d, g1, g2, merged, startadr, pairadr);
          }
        }
      }

      // body : flex
      else if (isbody1) {
        int f = bf2 - nbody;

        // process body geoms
        for (int g=m->body_geomadr[bf1]; g < geomadr_end1; g++) {
          // bitmask filtering at the geom-flex level
          if (filterBitmask(m->geom_contype[g], m->geom_conaffinity[g],
                            m->flex_contype[f], m->flex_conaffinity[f])) {
            continue;
          }

          // plane special processing
          if (m->geom_type[g] == mjGEOM_PLANE) {
            mj_collidePlaneFlex(m, d, g, f);
            continue;
          }

          // collide geom with flex elements
          int elemnum = m->flex_elemnum[f];
          for (int e=0; e < elemnum; e++) {
            mj_collideGeomElem(m, d, g, f, e);
          }
        }
      }

      // flex : flex
      else {
        int f1 = bf1 - nbody;
        int f2 = bf2 - nbody;

        // collide elements of two flexes
        for (int e1=0; e1 < m->flex_elemnum[f1]; e1++) {
          for (int e2=0; e2 < m->flex_elemnum[f2]; e2++) {
            mj_collideElems(m, d, f1, e1, f2, e2);
          }
        }
      }
    }
  }

  // finish merging predefined geom pairs
  if (npair) {
    while (pairadr < npair) {
      mj_collideGeoms(m, d, pairadr++, -1);
    }
  }

  // flex self-collisions
  for (int f=0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && (m->flex_contype[f] & m->flex_conaffinity[f])) {
      // internal collisions
      if (m->flex_internal[f]) {
        mj_collideFlexInternal(m, d, f);
      }

      // active element collisions
      if (m->flex_selfcollide[f] != mjFLEXSELF_NONE) {
        // element-element: midphase
        if (!mjDISABLED(mjDSBL_MIDPHASE) &&
            m->flex_selfcollide[f] != mjFLEXSELF_NARROW &&
            m->flex_bvhadr[f] >= 0) {
          // select midphase mode
          if (m->flex_selfcollide[f] == mjFLEXSELF_BVH ||
              (m->flex_selfcollide[f] == mjFLEXSELF_AUTO && m->flex_dim[f] == 3)) {
            mj_collideTree(m, d, nbody+f, nbody+f, 0, 0, 0);
          } else {
            mj_collideFlexSAP(m, d, f);
          }
        }

        // element-element: direct
        else {
          int flex_elemnum = m->flex_elemnum[f];
          for (int e1=0; e1 < flex_elemnum; e1++) {
            if (mj_isElemActive(m, f, e1)) {
              for (int e2=e1+1; e2 < flex_elemnum; e2++) {
                if (mj_isElemActive(m, f, e2)) {
                  mj_collideElems(m, d, f, e1, f, e2);
                }
              }
            }
          }
        }
      }
    }
  }

  // end narrowphase and midphase timer
  TM_END(mjTIMER_COL_NARROW);

  mj_freeStack(d);
  TM_END1(mjTIMER_POS_COLLISION);
}



//------------------------------------ binary tree search ------------------------------------------

// collision tree node
struct mjCollisionTree_ {
  int node1;
  int node2;
};
typedef struct mjCollisionTree_ mjCollisionTree;


// checks if the proposed collision pair is already present in pair_geom and calls narrow phase
void mj_collideGeomPair(const mjModel* m, mjData* d, int g1, int g2, int merged,
                        int startadr, int pairadr) {
  // merged: make sure geom pair is not repeated
  if (merged) {
    // find matching pair
    int found = 0;
    for (int k=startadr; k < pairadr; k++) {
      if ((m->pair_geom1[k] == g1 && m->pair_geom2[k] == g2) ||
          (m->pair_geom1[k] == g2 && m->pair_geom2[k] == g1)) {
        found = 1;
        break;
      }
    }

    // not found: test
    if (!found) {
      mj_collideGeoms(m, d, g1, g2);
    }
  }

  // not merged: always test
  else {
    mj_collideGeoms(m, d, g1, g2);
  }
}



// oriented bounding boxes collision (see Gottschalk et al.)
int mj_collideOBB(const mjtNum aabb1[6], const mjtNum aabb2[6],
                  const mjtNum xpos1[3], const mjtNum xmat1[9],
                  const mjtNum xpos2[3], const mjtNum xmat2[9], mjtNum margin,
                  mjtNum product[36], mjtNum offset[12], mjtByte* initialize) {
  // get infinite dimensions (planes only)
  mjtByte inf1[3] = {aabb1[3] >= mjMAXVAL, aabb1[4] >= mjMAXVAL, aabb1[5] >= mjMAXVAL};
  mjtByte inf2[3] = {aabb2[3] >= mjMAXVAL, aabb2[4] >= mjMAXVAL, aabb2[5] >= mjMAXVAL};

  // if a bounding box is infinite, there must be a collision
  if ((inf1[0] && inf1[1] && inf1[2]) || (inf2[0] && inf2[1] && inf2[2])) {
    return 1;
  }

  const mjtNum* aabb[2] = {aabb1, aabb2};
  const mjtNum *xmat[2] = {xmat1, xmat2};
  const mjtNum *xpos[2] = {xpos1, xpos2};
  mjtNum xcenter[2][3], normal[2][3][3];
  mjtNum proj[2], radius[2];
  mjtByte infinite[2] = {inf1[0] || inf1[1] || inf1[2], inf2[0] || inf2[1] || inf2[2]};

  // compute centers in local coordinates
  if (product == NULL) {
    for (int i=0; i < 2; i++) {  // bounding boxes
      for (int j=0; j < 3; j++) {  // axes
        if (xmat[i]) {
          mju_mulMatVec3(xcenter[i], xmat[i], aabb[i]);
        } else {
          mju_copy3(xcenter[i], aabb[i]);
        }

        if (xpos[i]) {
          mju_addTo3(xcenter[i], xpos[i]);
        }
      }
    }
  }

  // compute normals in global coordinates
  for (int i=0; i < 2; i++) {  // bounding boxes
    for (int j=0; j < 3; j++) {  // faces
      for (int k=0; k < 3; k++) {  // world axes
        if (xmat[i]) {
          normal[i][j][k] = xmat[i][3*k+j];
        } else {
          normal[i][j][k] = (j == k);
        }
      }
    }
  }

  // precompute dot products
  if (product && offset && *initialize) {
    for (int i=0; i < 2; i++) {  // bodies
      for (int j=0; j < 2; j++) {  // bodies
        for (int k=0; k < 3; k++) {  // axes
          for (int l=0; l < 3; l++) {  // axes
            product[18*i + 9*j + 3*k + l] = mju_dot3(normal[i][l], normal[j][k]);
          }
          offset[6*i + 3*j + k] = xpos[i] ? mju_dot3(xpos[i], normal[j][k]) : 0;
        }
      }
    }
    *initialize = 0;
  }

  // check intersections
  for (int j=0; j < 2; j++) {  // bounding boxes
    if (infinite[1-j]) {
      continue;  // skip test against an infinite body
    }
    for (int k=0; k < 3; k++) {  // face
      for (int i=0; i < 2; i++) {  // bounding boxes
        if (product == NULL) {
          proj[i] = mju_dot3(xcenter[i], normal[j][k]);
          radius[i] = mju_abs(aabb[i][3]*mju_dot3(normal[i][0], normal[j][k])) +
                      mju_abs(aabb[i][4]*mju_dot3(normal[i][1], normal[j][k])) +
                      mju_abs(aabb[i][5]*mju_dot3(normal[i][2], normal[j][k]));
        } else {
          int adr = 18*i + 9*j + 3*k;
          proj[i] = aabb[i][0] * product[adr + 0] +
                    aabb[i][1] * product[adr + 1] +
                    aabb[i][2] * product[adr + 2] +
                    offset[6*i + 3*j + k];
          radius[i] = mju_abs(aabb[i][3]*product[adr + 0]) +
                      mju_abs(aabb[i][4]*product[adr + 1]) +
                      mju_abs(aabb[i][5]*product[adr + 2]);
        }
      }

      if (radius[0]+radius[1]+margin < mju_abs(proj[1]-proj[0])) {
        return 0;
      }
    }
  }

  return 1;
}



// binary search between two bodyflex trees
void mj_collideTree(const mjModel* m, mjData* d, int bf1, int bf2,
                    int merged, int startadr, int pairadr) {
  int nbody = m->nbody, nbvhstatic = m->nbvhstatic;
  mjtByte isbody1 = (bf1 < nbody);
  mjtByte isbody2 = (bf2 < nbody);
  int f1 = isbody1 ? -1 : bf1 - nbody;
  int f2 = isbody2 ? -1 : bf2 - nbody;
  int mark_active = m->vis.global.bvactive;
  const int bvhadr1 = isbody1 ? m->body_bvhadr[bf1] : m->flex_bvhadr[f1];
  const int bvhadr2 = isbody2 ? m->body_bvhadr[bf2] : m->flex_bvhadr[f2];
  const int* child1 = m->bvh_child + 2*bvhadr1;
  const int* child2 = m->bvh_child + 2*bvhadr2;
  const mjtNum* bvh1 = isbody1 ? (m->bvh_aabb     + 6*bvhadr1) :
                                 (d->bvh_aabb_dyn + 6*(bvhadr1 - nbvhstatic));
  const mjtNum* bvh2 = isbody2 ? (m->bvh_aabb     + 6*bvhadr2) :
                                 (d->bvh_aabb_dyn + 6*(bvhadr2 - nbvhstatic));

  // used with rotated bounding boxes (when bodies are involved)
  mjtNum product[36];  // 2 bb x 2 bb x 3 axes (body) x 3 axes (world)
  mjtNum offset[12];   // 2 bb x 2 bb x 3 axes (world)
  mjtByte initialize = 1;

  // bitmask filter for bodyflex pair
  if (!canCollide2(m, bf1, bf2)) {
    return;
  }

  mj_markStack(d);
  // TODO(b/273737633): Store bvh max depths to make this bound tighter.
  const int max_stack = (isbody1 ? m->body_bvhnum[bf1] : m->flex_bvhnum[f1]) +
                        (isbody2 ? m->body_bvhnum[bf2] : m->flex_bvhnum[f2]);
  mjCollisionTree* stack = mjSTACKALLOC(d, max_stack, mjCollisionTree);

  int nstack = 1;
  stack[0].node1 = stack[0].node2 = 0;

  // for body:flex, if body has planes, call mj_collidePlaneFlex directly
  if (isbody1 && !isbody2 && m->body_weldid[bf1] == 0) {
    for (int i=m->body_geomadr[bf1]; i < m->body_geomadr[bf1]+m->body_geomnum[bf1]; i++) {
      if (m->geom_type[i] == mjGEOM_PLANE) {
        mj_collidePlaneFlex(m, d, i, f2);
      }
    }
  }

  // collide trees
  while (nstack) {
    // pop from stack
    nstack--;
    int node1 = stack[nstack].node1;
    int node2 = stack[nstack].node2;
    mjtByte isleaf1 = (child1[2*node1] < 0) && (child1[2*node1+1] < 0);
    mjtByte isleaf2 = (child2[2*node2] < 0) && (child2[2*node2+1] < 0);
    int nodeid1 = m->bvh_nodeid[bvhadr1 + node1];
    int nodeid2 = m->bvh_nodeid[bvhadr2 + node2];

    // SHOULD NOT OCCUR
    if ((isleaf1 && nodeid1 < 0) || (isleaf2 && nodeid2 < 0)) {
      mju_error("BVH leaf has invalid node id");
    }

    // self-collision: avoid repeated pairs
    if (bf1 == bf2 && node1 > node2) {
      continue;
    }

    // body : body
    if (isbody1 && isbody2) {
      // both are leaves
      if (isleaf1 && isleaf2) {
        mjtNum maxmargin = mju_max(m->geom_margin[nodeid1], m->geom_margin[nodeid2]);
        mjtNum margin = mj_assignMargin(m, maxmargin);

        if (!mj_filterSphere(m, d, nodeid1, nodeid2, margin)) {
          if (mj_collideOBB(m->geom_aabb + 6*nodeid1, m->geom_aabb + 6*nodeid2,
                            d->geom_xpos + 3*nodeid1, d->geom_xmat + 9*nodeid1,
                            d->geom_xpos + 3*nodeid2, d->geom_xmat + 9*nodeid2,
                            margin, NULL, NULL, &initialize)) {
            mj_collideGeomPair(m, d, nodeid1, nodeid2, merged, startadr, pairadr);
            if (mark_active) {
              d->bvh_active[node1 + bvhadr1] = 1;
              d->bvh_active[node2 + bvhadr2] = 1;
            }
          }
        }
        continue;
      }

      // if no intersection at intermediate levels, stop
      mjtNum maxmargin = mju_max(m->body_margin[bf1], m->body_margin[bf2]);
      mjtNum margin = mj_assignMargin(m, maxmargin);
      if (!mj_collideOBB(bvh1 + 6*node1, bvh2 + 6*node2,
                         d->xipos + 3*bf1, d->ximat + 9*bf1,
                         d->xipos + 3*bf2, d->ximat + 9*bf2,
                         margin, product, offset, &initialize)) {
        continue;
      }
    }

    // body : flex
    else if (isbody1 && !isbody2) {
      // both are leaves
      if (isleaf1 && isleaf2) {
        mjtNum maxmargin = mju_max(m->geom_margin[nodeid1], m->flex_margin[f2]);
        mjtNum margin = mj_assignMargin(m, maxmargin);

        if (!filterBitmask(m->geom_contype[nodeid1], m->geom_conaffinity[nodeid1],
                           m->flex_contype[f2], m->flex_conaffinity[f2]) &&
            !filterSphereBox(d->geom_xpos + 3*nodeid1, m->geom_rbound[nodeid1] + margin,
                             bvh2 + 6*node2)) {
          if (mj_collideOBB(m->geom_aabb + 6*nodeid1, bvh2 + 6*node2,
                            d->geom_xpos + 3*nodeid1, d->geom_xmat + 9*nodeid1,
                            NULL, NULL,
                            margin, NULL, NULL, &initialize)) {
            // collide unless geom is plane (plane:flex handled separately)
            if (m->geom_type[nodeid1] != mjGEOM_PLANE) {
              mj_collideGeomElem(m, d, nodeid1, f2, nodeid2);
            }
            if (mark_active) {
              d->bvh_active[node1 + bvhadr1] = 1;
              d->bvh_active[node2 + bvhadr2] = 1;
            }
          }
        }
        continue;
      }

      // if no intersection at intermediate levels, stop
      mjtNum maxmargin = mju_max(m->body_margin[bf1], m->flex_margin[f2]);
      mjtNum margin = mj_assignMargin(m, maxmargin);
      if (!mj_collideOBB(bvh1 + 6*node1, bvh2 + 6*node2,
                         d->xipos + 3*bf1, d->ximat + 9*bf1,
                         NULL, NULL,
                         margin, product, offset, &initialize)) {
        continue;
      }
    }

    // flex : body  SHOULD NOT OCCUR
    else if (!isbody1 && isbody2) {
      mjERROR("BVH flex : body collision should not occur");
    }

    // flex : flex
    else {
      // both are leaves
      // box filter applied in mj_collideElems, bitmask filter applied earlier
      if (isleaf1 && isleaf2) {
        mj_collideElems(m, d, f1, nodeid1, f2, nodeid2);
        if (mark_active) {
          d->bvh_active[node1 + bvhadr1] = 1;
          d->bvh_active[node2 + bvhadr2] = 1;
        }
        continue;
      }

      // if no intersection at intermediate levels, stop
      mjtNum maxmargin = mju_max(m->flex_margin[f1], m->flex_margin[f2]);
      mjtNum margin = mj_assignMargin(m, maxmargin);
      if (filterBox(bvh1 + 6*node1, bvh2 + 6*node2, margin)) {
        continue;
      }
    }

    if (mark_active) {
      d->bvh_active[node1 + bvhadr1] = 1;
      d->bvh_active[node2 + bvhadr2] = 1;
    }

    // keep traversing the tree
    if (!isleaf1 && isleaf2) {
      for (int i=0; i < 2; i++) {
        if (child1[2*node1+i] != -1) {
          if (nstack >= max_stack) {
            mjERROR("BVH stack depth exceeded.");  // SHOULD NOT OCCUR
          }
          stack[nstack].node1 = child1[2*node1+i];
          stack[nstack].node2 = node2;
          nstack++;
        }
      }
    } else if (isleaf1 && !isleaf2) {
      for (int i=0; i < 2; i++) {
        if (child2[2*node2+i] != -1) {
          if (nstack >= max_stack) {
            mjERROR("BVH stack depth exceeded.");  // SHOULD NOT OCCUR
          }
          stack[nstack].node1 = node1;
          stack[nstack].node2 = child2[2*node2+i];
          nstack++;
        }
      }
    } else {
      // compute surface areas of bounding boxes
      mjtNum x1 = bvh1[6*node1+3]-bvh1[6*node1+0];
      mjtNum y1 = bvh1[6*node1+4]-bvh1[6*node1+1];
      mjtNum z1 = bvh1[6*node1+5]-bvh1[6*node1+2];
      mjtNum x2 = bvh2[6*node2+3]-bvh2[6*node2+0];
      mjtNum y2 = bvh2[6*node2+4]-bvh2[6*node2+1];
      mjtNum z2 = bvh2[6*node2+5]-bvh2[6*node2+2];
      mjtNum surface1 = x1*y1 + y1*z1 + z1*x1;
      mjtNum surface2 = x2*y2 + y2*z2 + z2*x2;

      // traverse the hierarchy whose bounding box has the larger surface area
      if (surface1 > surface2) {
        for (int i = 0; i < 2; i++) {
          if (child1[2 * node1 + i] != -1) {
            if (nstack >= max_stack) {
              mjERROR("BVH stack depth exceeded.");  // SHOULD NOT OCCUR
            }
            stack[nstack].node1 = child1[2 * node1 + i];
            stack[nstack].node2 = node2;
            nstack++;
          }
        }
      } else {
        for (int i = 0; i < 2; i++) {
          if (child2[2 * node2 + i] != -1) {
            if (nstack >= max_stack) {
              mjERROR("BVH stack depth exceeded.");  // SHOULD NOT OCCUR
            }
            stack[nstack].node1 = node1;
            stack[nstack].node2 = child2[2*node2+i];
            nstack++;
          }
        }
      }
    }
  }
  mj_freeStack(d);
}



//----------------------------- broad-phase collision detection ------------------------------------

// make AAMM (xmin[3], xmax[3]) for one bodyflex
static void makeAAMM(const mjModel* m, mjData* d, mjtNum* aamm, int bf, const mjtNum* frame) {
  // body
  if (bf < m->nbody) {
    int body = bf;
    int body_geomnum = m->body_geomnum[body];

    // process all body geoms (body is collidable, should have geoms)
    for (int i=0; i < body_geomnum; i++) {
      int geom = m->body_geomadr[body]+i;
      mjtNum margin = mjENABLED(mjENBL_OVERRIDE) ? 0.5*m->opt.o_margin : m->geom_margin[geom];
      mjtNum _aamm[6];

      // set _aamm for this geom
      for (int j=0; j < 3; j++) {
        mjtNum cen = mju_dot3(d->geom_xpos+3*geom, frame+3*j);
        _aamm[j]   = cen - m->geom_rbound[geom] - margin;
        _aamm[j+3] = cen + m->geom_rbound[geom] + margin;
      }

      // update body aamm
      if (i == 0) {
        mju_copy(aamm, _aamm, 6);
      } else {
        for (int j=0; j < 3; j++) {
          aamm[j]   = mju_min(aamm[j],   _aamm[j]);
          aamm[j+3] = mju_max(aamm[j+3], _aamm[j+3]);
        }
      }
    }
  }

  // flex
  else {
    int f = bf - m->nbody;
    int flex_vertnum = m->flex_vertnum[f];
    const mjtNum* vbase = d->flexvert_xpos + 3*m->flex_vertadr[f];

    // process flex vertices
    for (int i=0; i < flex_vertnum; i++) {
      mjtNum v[3];

      // compute vertex coordinates in given frame
      mju_mulMatVec(v, frame, vbase+3*i, 3, 3);

      // update aamm
      if (i == 0) {
        mju_copy3(aamm, v);
        mju_copy3(aamm+3, v);
      } else {
        for (int j=0; j < 3; j++) {
          aamm[j]   = mju_min(aamm[j], v[j]);
          aamm[j+3] = mju_max(aamm[j+3], v[j]);
        }
      }
    }

    // correct for flex radius and margin
    mjtNum margin = mjENABLED(mjENBL_OVERRIDE) ? 0.5*m->opt.o_margin : m->flex_margin[f];
    mjtNum bound = m->flex_radius[f] + margin;
    aamm[0] -= bound;
    aamm[1] -= bound;
    aamm[2] -= bound;
    aamm[3] += bound;
    aamm[4] += bound;
    aamm[5] += bound;
  }
}



// add bodyflex pair in buffer; do not filter if m is NULL
static void add_pair(const mjModel* m, int bf1, int bf2,
                     int* npair, int* pair, int maxpair) {
  // add pair if there is room in buffer
  if ((*npair) < maxpair) {
    // contact filtering if m is not NULL
    if (m) {
      int nbody = m->nbody;
      int contype1, conaffinity1, contype2, conaffinity2;

      // get contype and conaffinity for bodyflex 1
      if (bf1 < nbody) {
        int body_geomadr1 = m->body_geomadr[bf1];
        int body_geomnum1 = m->body_geomnum[bf1];
        contype1 = conaffinity1 = 0;
        for (int i=body_geomadr1; i < body_geomadr1+body_geomnum1; i++) {
          contype1 |= m->geom_contype[i];
          conaffinity1 |= m->geom_conaffinity[i];
        }
      } else {
        contype1 = m->flex_contype[bf1-nbody];
        conaffinity1 = m->flex_conaffinity[bf1-nbody];
      }

      // get contype and conaffinity for bodyflex 2
      if (bf2 < nbody) {
        int body_geomadr2 = m->body_geomadr[bf2];
        int body_geomnum2 = m->body_geomnum[bf2];
        contype2 = conaffinity2 = 0;
        for (int i=body_geomadr2; i < body_geomadr2+body_geomnum2; i++) {
          contype2 |= m->geom_contype[i];
          conaffinity2 |= m->geom_conaffinity[i];
        }
      } else {
        contype2 = m->flex_contype[bf2-nbody];
        conaffinity2 = m->flex_conaffinity[bf2-nbody];
      }

      // compatibility check
      if (!(contype1 & conaffinity2) && !(contype2 & conaffinity1)) {
        return;
      }
    }

    // add pair
    if (bf1 < bf2) {
      pair[*npair] = (bf1<<16) + bf2;
    } else {
      pair[*npair] = (bf2<<16) + bf1;
    }
    (*npair)++;
  } else {
    mjERROR("broadphase buffer full");
  }
}



//----------------------------- general Sweep and Prune algorithm ----------------------------------

// helper structure for SAP sorting
struct _mjtSAP {
  float value;
  int id_ismax;
};
typedef struct _mjtSAP mjtSAP;



// comparison function for SAP
static inline int SAPcmp(mjtSAP* obj1, mjtSAP* obj2, void* context) {
  if (obj1->value < obj2->value) {
    return -1;
  } else if (obj1->value == obj2->value) {
    return 0;
  } else {
    return 1;
  }
}

// define SAPsort function for sorting SAP sorting
mjSORT(SAPsort, mjtSAP, SAPcmp)


// given list of axis-aligned bounding boxes in AAMM (xmin[3], xmax[3]) format,
// return list of pairs (i, j) in format (i<<16 + j) that can collide,
// using sweep-and-prune along specified axis (0-2).
static int mj_SAP(mjData* d, const mjtNum* aamm, int n, int axis, int* pair, int maxpair) {
  // check inputs
  if (n >= 0x10000 || axis < 0 || axis > 2 || maxpair < 1) {
    return -1;
  }

  // allocate sort buffer
  mjtSAP* sortbuf = mjSTACKALLOC(d, 2*n, mjtSAP);
  mjtSAP* activebuf = mjSTACKALLOC(d, 2*n, mjtSAP);

  // init sortbuf with specified axis
  for (int i=0; i < n; i++) {
    sortbuf[2*i].id_ismax = i;
    sortbuf[2*i].value = (float)aamm[6*i+axis];
    sortbuf[2*i+1].id_ismax = i + 0x10000;
    sortbuf[2*i+1].value = (float)aamm[6*i+3+axis];
  }

  // sort along specified axis
  mjtSAP* buf = mjSTACKALLOC(d, 2*n, mjtSAP);
  SAPsort(sortbuf, buf, 2*n, NULL);

  // define the other two axes
  int axisA, axisB;
  if (axis == 0) {
    axisA = 1;
    axisB = 2;
  } else if (axis == 1) {
    axisA = 0;
    axisB = 2;
  } else {
    axisA = 0;
    axisB = 1;
  }

  // sweep and prune
  int cnt = 0;    // size of active list
  int npair = 0;  // number of pairs added
  for (int i=0; i < 2*n; i++) {
    // min value: collide with all in list, add
    if (!(sortbuf[i].id_ismax & 0x10000)) {
      for (int j=0; j < cnt; j++) {
        // get ids: no need to mask ismax because activebuf entries never have the ismax bit,
        // and sortbuf[i].id_ismax is tested above
        int id1 = activebuf[j].id_ismax;
        int id2 = sortbuf[i].id_ismax;

        // use the other two axes to prune if possible
        if (aamm[6*id1+axisA] > aamm[6*id2+axisA+3] ||
            aamm[6*id1+axisB] > aamm[6*id2+axisB+3] ||
            aamm[6*id2+axisA] > aamm[6*id1+axisA+3] ||
            aamm[6*id2+axisB] > aamm[6*id1+axisB+3]) {
          continue;
        }

        // add pair, check buffer size
        pair[npair++] = (id1<<16) + id2;
        if (npair >= maxpair) {
          return maxpair;
        }
      }

      // add to list
      activebuf[cnt] = sortbuf[i];
      cnt++;
    }

    // max value: remove corresponding min value from list
    else {
      int toremove = sortbuf[i].id_ismax & 0xFFFF;
      for (int j=0; j < cnt; j++) {
        if (activebuf[j].id_ismax == toremove) {
          if (j < cnt-1) {
            memmove(activebuf+j, activebuf+j+1, sizeof(mjtSAP)*(cnt-1-j));
          }
          cnt--;
          break;
        }
      }
    }
  }

  return npair;
}



// add vector to covariance
static void updateCov(mjtNum cov[9], const mjtNum vec[3], const mjtNum cen[3]) {
  mjtNum dif[3] = {vec[0]-cen[0], vec[1]-cen[1], vec[2]-cen[2]};
  mjtNum D00 = dif[0]*dif[0];
  mjtNum D01 = dif[0]*dif[1];
  mjtNum D02 = dif[0]*dif[2];
  mjtNum D11 = dif[1]*dif[1];
  mjtNum D12 = dif[1]*dif[2];
  mjtNum D22 = dif[2]*dif[2];
  cov[0] += D00;
  cov[1] += D01;
  cov[2] += D02;
  cov[3] += D01;
  cov[4] += D11;
  cov[5] += D12;
  cov[6] += D02;
  cov[7] += D12;
  cov[8] += D22;
}



// comparison function for unsigned ints
static inline int uintcmp(int* i, int* j, void* context) {
  if ((unsigned) *i < (unsigned) *j) {
    return -1;
  } else if (*i == *j) {
    return 0;
  } else {
    return 1;
  }
}

// define bfsort function for sorting bodyflex pairs
mjSORT(bfsort, int, uintcmp)


// broadphase collision detector
int mj_broadphase(const mjModel* m, mjData* d, int* bfpair, int maxpair) {
  int npair = 0, nbody = m->nbody, ngeom = m->ngeom;
  int nvert = m->nflexvert, nflex = m->nflex, nbodyflex = m->nbody + m->nflex;
  int dsbl_filterparent = mjDISABLED(mjDSBL_FILTERPARENT);
  mjtNum cov[9], cen[3], eigval[3], frame[9], quat[4];

  // init with pairs involving always-colliding bodies
  for (int b1=0; b1 < nbody; b1++) {
    // cannot collide
    if (!canCollide(m, b1)) {
      continue;
    }

    // b1 is world body with geoms, or world-welded body with plane
    if ((b1 == 0 && m->body_geomnum[b1] > 0) ||
        (m->body_weldid[b1] == 0 && hasPlane(m, b1))) {
      // add b1:body pairs that are not welded together
      for (int b2=0; b2 < nbody; b2++) {
        // cannot collide
        if (!canCollide(m, b2)) {
          continue;
        }

        // welded together
        int weld2 = m->body_weldid[b2];
        int parent_weld2 = m->body_weldid[m->body_parentid[weld2]];
        if (filterBodyPair(0, 0, weld2, parent_weld2, dsbl_filterparent)) {
          continue;
        }

        // add pair
        add_pair(m, b1, b2, &npair, bfpair, maxpair);
      }

      // add all b1:flex pairs
      for (int f=0; f < nflex; f++) {
        add_pair(m, b1, nbody+f, &npair, bfpair, maxpair);
      }
    }
  }

  // find center of non-world geoms and flex vertices; return if none
  int cnt = 0;
  mju_zero3(cen);
  for (int i=0; i < ngeom; i++) {
    if (m->geom_bodyid[i]) {
      mju_addTo3(cen, d->geom_xpos+3*i);
      cnt++;
    }
  }
  for (int i=0; i < nvert; i++) {
    if (m->flex_vertbodyid[i]) {
      mju_addTo3(cen, d->flexvert_xpos+3*i);
      cnt++;
    }
  }
  if (cnt == 0) {
    return npair;
  }
  mju_scl3(cen, cen, 1.0/cnt);

  // compute covariance
  mju_zero(cov, 9);
  for (int i=0; i < ngeom; i++) {
    if (m->geom_bodyid[i]) {
      updateCov(cov, d->geom_xpos+3*i, cen);
    }
  }
  for (int i=0; i < nvert; i++) {
    if (m->flex_vertbodyid[i]) {
      updateCov(cov, d->flexvert_xpos+3*i, cen);
    }
  }
  mju_scl(cov, cov, 1.0/cnt, 9);

  // construct covariance-aligned 3D frame
  mju_eig3(eigval, frame, quat, cov);

  // allocate collidable bodyflex ids, construct list
  mj_markStack(d);
  int* bfid = mjSTACKALLOC(d, nbodyflex, int);
  int ncollide = 0;
  for (int i=1; i < nbodyflex; i++) {
    if (canCollide(m, i)) {
      bfid[ncollide++] = i;
    }
  }

  if (ncollide > 1) {
    // allocate and construct AAMMs for collidable only
    mjtNum* aamm = mjSTACKALLOC(d, 6*ncollide, mjtNum);
    for (int i=0; i < ncollide; i++) {
      makeAAMM(m, d, aamm+6*i, bfid[i], frame);
    }

    // call SAP
    int maxsappair = ncollide*(ncollide-1)/2;
    int* sappair = mjSTACKALLOC(d, maxsappair, int);
    int nsappair = mj_SAP(d, aamm, ncollide, 0, sappair, maxsappair);
    if (nsappair < 0) {
      mjERROR("SAP failed");
    }

    // filter SAP pairs, convert to bodyflex pairs
    for (int i=0; i < nsappair; i++) {
      int bf1 = bfid[sappair[i] >> 16];
      int bf2 = bfid[sappair[i] & 0xFFFF];

      // body pair: prune based on weld filter
      if (bf1 < nbody && bf2 < nbody) {
        int weld1 = m->body_weldid[bf1];
        int weld2 = m->body_weldid[bf2];
        int parent_weld1 = m->body_weldid[m->body_parentid[weld1]];
        int parent_weld2 = m->body_weldid[m->body_parentid[weld2]];

        if (filterBodyPair(weld1, parent_weld1, weld2, parent_weld2,
                           dsbl_filterparent)) {
          continue;
        }
      }

      // add bodyflex pair if there is room in buffer
      add_pair(m, bf1, bf2, &npair, bfpair, maxpair);
    }
  }

  // sort bodyflex pairs by signature
  if (npair > 1) {
    int* buf = mjSTACKALLOC(d, npair, int);
    bfsort(bfpair, buf, npair, NULL);
  }

  mj_freeStack(d);
  return npair;
}



//----------------------------- narrow-phase collision detection -----------------------------------

// compute contact condim, gap, solref, solimp, friction
static void mj_contactParam(const mjModel* m, int* condim, mjtNum* gap,
                            mjtNum* solref, mjtNum* solimp, mjtNum* friction,
                            int g1, int g2, int f1, int f2) {
  mjtNum fri[3];

  // get parameters from geom1 or flex1
  int priority1 =           (f1 < 0) ? m->geom_priority[g1]     : m->flex_priority[f1];
  int condim1 =             (f1 < 0) ? m->geom_condim[g1]       : m->flex_condim[f1];
  mjtNum gap1 =             (f1 < 0) ? m->geom_gap[g1]          : m->flex_gap[f1];
  mjtNum solmix1 =          (f1 < 0) ? m->geom_solmix[g1]       : m->flex_solmix[f1];
  const mjtNum* solref1 =   (f1 < 0) ? m->geom_solref+g1*mjNREF : m->flex_solref+f1*mjNREF;
  const mjtNum* solimp1 =   (f1 < 0) ? m->geom_solimp+g1*mjNIMP : m->flex_solimp+f1*mjNIMP;
  const mjtNum* friction1 = (f1 < 0) ? m->geom_friction+g1*3    : m->flex_friction+f1*3;

  // get parameters from geom2 or flex2
  int priority2 =           (f2 < 0) ? m->geom_priority[g2]     : m->flex_priority[f2];
  int condim2 =             (f2 < 0) ? m->geom_condim[g2]       : m->flex_condim[f2];
  mjtNum gap2 =             (f2 < 0) ? m->geom_gap[g2]          : m->flex_gap[f2];
  mjtNum solmix2 =          (f2 < 0) ? m->geom_solmix[g2]       : m->flex_solmix[f2];
  const mjtNum* solref2 =   (f2 < 0) ? m->geom_solref+g2*mjNREF : m->flex_solref+f2*mjNREF;
  const mjtNum* solimp2 =   (f2 < 0) ? m->geom_solimp+g2*mjNIMP : m->flex_solimp+f2*mjNIMP;
  const mjtNum* friction2 = (f2 < 0) ? m->geom_friction+g2*3    : m->flex_friction+f2*3;

  // gap: max
  *gap = mju_max(gap1, gap2);

  // different priority: copy from item with higher priority
  if (priority1 > priority2) {
    *condim = condim1;
    mju_copy(solref, solref1, mjNREF);
    mju_copy(solimp, solimp1, mjNIMP);
    mju_copy(fri, friction1, 3);
  }
  else if (priority1 < priority2) {
    *condim = condim2;
    mju_copy(solref, solref2, mjNREF);
    mju_copy(solimp, solimp2, mjNIMP);
    mju_copy(fri, friction2, 3);
  }

  // same priority
  else {
    // condim: max
    *condim = mjMAX(condim1, condim2);

    // compute solver mix factor
    mjtNum mix;
    if (solmix1 >= mjMINVAL && solmix2 >= mjMINVAL) {
      mix = solmix1 / (solmix1 + solmix2);
    } else if (solmix1 < mjMINVAL && solmix2 < mjMINVAL) {
      mix = 0.5;
    } else if (solmix1 < mjMINVAL) {
      mix = 0.0;
    } else {
      mix = 1.0;
    }

    // reference standard: mix
    if (solref1[0] > 0 && solref2[0] > 0) {
      for (int i=0; i < mjNREF; i++) {
        solref[i] = mix*solref1[i] + (1-mix)*solref2[i];
      }
    }

    // reference direct: min
    else {
      for (int i=0; i < mjNREF; i++) {
        solref[i] = mju_min(solref1[i], solref2[i]);
      }
    }

    // impedance: mix
    for (int i=0; i < mjNIMP; i++) {
      solimp[i] = mix*solimp1[i] + (1-mix)*solimp2[i];
    }

    // friction: max
    for (int i=0; i < 3; i++) {
      fri[i] = mju_max(friction1[i], friction2[i]);
    }
  }

  // unpack 5D friction
  friction[0] = fri[0];
  friction[1] = fri[0];
  friction[2] = fri[1];
  friction[3] = fri[2];
  friction[4] = fri[2];

  // SHOULD NOT OCCUR
  if (*condim > 6 || *condim < 1) {
    mjERROR("Invalid condim value: %d", *condim);
  }
}



// set contact parameters
static void mj_setContact(const mjModel* m, mjContact* con,
                          int condim, mjtNum includemargin,
                          const mjtNum* solref, const mjtNum* solreffriction,
                          const mjtNum* solimp, const mjtNum* friction) {
  // set parameters
  con->dim = condim;
  con->includemargin = includemargin;
  mj_assignRef(m, con->solref, solref);
  mj_assignRef(m, con->solreffriction, solreffriction);
  mj_assignImp(m, con->solimp, solimp);
  mj_assignFriction(m, con->friction, friction);

  // exclude in gap
  con->exclude = (con->dist >= includemargin);

  // complete frame
  mju_makeFrame(con->frame);

  // clear fields that are computed later
  con->efc_address = -1;
  con->mu = 0;
  mju_zero(con->H, 36);

  // set deprecated fields
  con->geom1 = con->geom[0];
  con->geom2 = con->geom[1];
}



// make capsule from two flex vertices
static void mj_makeCapsule(const mjModel* m, mjData* d, int f, const int vid[2],
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



// test two geoms for collision, apply filters, add to contact list
void mj_collideGeoms(const mjModel* m, mjData* d, int g1, int g2) {
  int num, type1, type2, condim;
  mjtNum margin, gap, friction[5], solref[mjNREF], solimp[mjNIMP];
  mjtNum solreffriction[mjNREF] = {0};

  int ipair = (g2 < 0 ? g1 : -1);

  // get explicit geom ids from pair
  if (ipair >= 0) {
    g1 = m->pair_geom1[ipair];
    g2 = m->pair_geom2[ipair];
  }

  // order geoms by type
  if (m->geom_type[g1] > m->geom_type[g2]) {
    int i = g1;
    g1 = g2;
    g2 = i;
  }

  // copy types and bodies
  type1 = m->geom_type[g1];
  type2 = m->geom_type[g2];

  mjfCollision collisionFunc = mjCOLLISIONFUNC[type1][type2];

  // return if no collision function
  if (!collisionFunc) {
    return;
  }

  // apply filters if not predefined pair
  if (ipair < 0) {
    // user filter if defined
    if (mjcb_contactfilter) {
      if (mjcb_contactfilter(m, d, g1, g2)) {
        return;
      }
    }

    // otherwise built-in filter
    else if (filterBitmask(m->geom_contype[g1], m->geom_conaffinity[g1],
                           m->geom_contype[g2], m->geom_conaffinity[g2])) {
      return;
    }
  }

  // set margin: dynamic or pair
  if (ipair < 0) {
    margin = mj_assignMargin(m, mju_max(m->geom_margin[g1], m->geom_margin[g2]));
  } else {
    margin = mj_assignMargin(m, m->pair_margin[ipair]);
  }

  // bounding sphere filter
  if (mj_filterSphere(m, d, g1, g2, margin)) {
    return;
  }

  // allocate mjContact[mjMAXCONPAIR] on the arena
  mjContact* con =
    (mjContact*) mj_arenaAllocByte(d, sizeof(mjContact) * mjMAXCONPAIR, _Alignof(mjContact));
  if (!con) {
    mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
    return;
  }

  // call collision detector to generate contacts
  num = collisionFunc(m, d, con, g1, g2, margin);

  // check contacts
  if (!num) {
    resetArena(d);
    return;
  }

  // check number of contacts, SHOULD NOT OCCUR
  if (num > mjMAXCONPAIR) {
    mjERROR("too many contacts returned by collision function");
  }

  // remove bad and repeated contacts in box-box
  if (collisionFunc == mjc_BoxBox) {
    // use dim field to mark: -1: bad, 0: good
    for (int i=0; i < num; i++) {
      con[i].dim = 0;
    }

    // get box info
    const mjtNum* pos1 =  d->geom_xpos + 3 * g1;
    const mjtNum* mat1 =  d->geom_xmat + 9 * g1;
    const mjtNum* size1 = m->geom_size + 3 * g1;
    const mjtNum* pos2 =  d->geom_xpos + 3 * g2;
    const mjtNum* mat2 =  d->geom_xmat + 9 * g2;
    const mjtNum* size2 = m->geom_size + 3 * g2;

    // find bad: contacts outside one of the boxes
    for (int i=0; i < num; i++) {
      // box sizes with margin
      mjtNum sz1[3] = {size1[0] + margin, size1[1] + margin, size1[2] + margin};
      mjtNum sz2[3] = {size2[0] + margin, size2[1] + margin, size2[2] + margin};

      // relative distance from surface (1%) outside of which box-box contacts are removed
      static mjtNum kRemoveRatio = 1.01;

      // is the contact outside: 1, inside: -1, within the removal width: 0
      int out1 = mju_outsideBox(con[i].pos, pos1, mat1, sz1, kRemoveRatio);
      int out2 = mju_outsideBox(con[i].pos, pos2, mat2, sz2, kRemoveRatio);

      // mark as bad if outside one box and not inside the other box
      if ((out1 == 1 && out2 != -1) || (out2 == 1 && out1 != -1)) {
        con[i].dim = -1;
      }
    }

    // find duplicates
    for (int i=0; i < num-1; i++) {
      if (con[i].dim == -1) {
        continue;  // already marked bad: skip
      }
      for (int j=i+1; j < num; j++) {
        if (con[j].dim == -1) {
          continue;  // already marked bad: skip
        }
        if (con[i].pos[0] == con[j].pos[0] &&
            con[i].pos[1] == con[j].pos[1] &&
            con[i].pos[2] == con[j].pos[2]) {
          con[i].dim = -1;
          break;
        }
      }
    }

    // consolidate good
    int i = 0;
    for (int j=0; j < num; j++) {
      // good: maybe copy
      if (con[j].dim == 0) {
        // different: copy
        if (i < j) {
          con[i] = con[j];
        }

        // advance either way
        i++;
      }
    }

    // adjust size
    num = i;
  }

  // set condim, gap, solref, solimp, friction: dynamic
  if (ipair < 0) {
    mj_contactParam(m, &condim, &gap, solref, solimp, friction, g1, g2, -1, -1);
  }

  // set condim, gap, solref, solimp, friction: pair
  else {
    condim = m->pair_dim[ipair];
    gap = m->pair_gap[ipair];
    mju_copy(solref, m->pair_solref+mjNREF*ipair, mjNREF);
    mju_copy(solimp, m->pair_solimp+mjNIMP*ipair, mjNIMP);
    mju_copy(friction, m->pair_friction+5*ipair, 5);

    // reference, friction directions
    if (m->pair_solreffriction[mjNREF*ipair] || m->pair_solreffriction[mjNREF*ipair + 1]) {
      mju_copy(solreffriction, m->pair_solreffriction+mjNREF*ipair, mjNREF);
    }
  }

  // add contacts returned by collision detector
  for (int i=0; i < num; i++) {
    // set contact ids
    con[i].geom[0] = g1;
    con[i].geom[1] = g2;
    con[i].flex[0] = -1;
    con[i].flex[1] = -1;
    con[i].elem[0] = -1;
    con[i].elem[1] = -1;
    con[i].vert[0] = -1;
    con[i].vert[1] = -1;

    // set remaining contact parameters
    mj_setContact(m, con + i, condim, margin-gap, solref, solreffriction, solimp, friction);
  }

  // add to ncon
  d->ncon += num;

  // move arena pointer back to the end of the contact array
  resetArena(d);
}



// test a plane geom and a flex for collision, add to contact list
void mj_collidePlaneFlex(const mjModel* m, mjData* d, int g, int f) {
  mjContact con;
  mjtNum radius = m->flex_radius[f];
  mjtNum* pos = d->geom_xpos + 3*g;
  mjtNum* mat = d->geom_xmat + 9*g;
  mjtNum nrm[3] = {mat[2], mat[5], mat[8]};

  // prepare contact parameters (same for all vertices)
  mjtNum margin = mj_assignMargin(m, mju_max(m->geom_margin[g], m->flex_margin[f]));
  int condim;
  int flex_vertnum = m->flex_vertnum[f];
  mjtNum gap, solref[mjNREF], solimp[mjNIMP], friction[5];
  mjtNum solreffriction[mjNREF] = {0};
  mj_contactParam(m, &condim, &gap, solref, solimp, friction, g, -1, -1, f);

  // collide all flex vertices with plane
  for (int i=0; i < flex_vertnum; i++) {
    mjtNum* v = d->flexvert_xpos + 3*(m->flex_vertadr[f]+i);

    // distance from plane to vertex
    mjtNum dif[3] = {v[0]-pos[0], v[1]-pos[1], v[2]-pos[2]};
    mjtNum dist = mju_dot3(dif, nrm);

    // no contact
    if (dist > margin + radius) {
      continue;
    }

    // create contact
    con.dist = dist - radius;
    mju_addScl3(con.pos, v, nrm, -con.dist*0.5 - radius);
    mju_copy3(con.frame, nrm);
    mju_zero3(con.frame+3);

    // set contact ids
    con.geom[0] = g;
    con.geom[1] = -1;
    con.flex[0] = -1;
    con.flex[1] = f;
    con.elem[0] = -1;
    con.elem[1] = -1;
    con.vert[0] = -1;
    con.vert[1] = i;

    // set remaining contact parameters
    mj_setContact(m, &con, condim, margin-gap, solref, solreffriction, solimp, friction);

    // add to mjData, abort if too many contacts
    if (mj_addContact(m, d, &con)) {
      return;
    }
  }
}



// test single triangle plane : vertex
static int planeVertex(mjContact* con, const mjtNum* pos, mjtNum rad,
                       int t0, int t1, int t2, int v) {
  // make t0 the origin
  mjtNum e1[3], e2[3], ev[3];
  mju_sub3(e1, pos+3*t1, pos+3*t0);
  mju_sub3(e2, pos+3*t2, pos+3*t0);
  mju_sub3(ev, pos+3*v,  pos+3*t0);

  // compute normal
  mjtNum nrm[3];
  mju_cross(nrm, e1, e2);
  mju_normalize3(nrm);

  // project, check distance
  mjtNum dst = mju_dot3(ev, nrm);
  if (dst <= -2*rad) {
    return 0;
  }

  // construct contact
  con->dist = -dst-2*rad;
  mju_scl3(con->frame, nrm, -1);
  mju_zero3(con->frame+3);
  mju_addScl3(con->pos, pos+3*v, nrm, -0.5*dst);
  con->vert[1] = v;
  return 1;
}



// test for internal flex collisions, add to contact list
// ignore margin to avoid permament self-collision
void mj_collideFlexInternal(const mjModel* m, mjData* d, int f) {
  int flex_evpairnum = m->flex_evpairnum[f];

  // predefined element-vertex
  for (int i=0; i < flex_evpairnum; i++) {
    const int* ev = m->flex_evpair + 2*m->flex_evpairadr[f] + 2*i;
    mj_collideElemVert(m, d, f, ev[0], ev[1]);
  }

  // within-element for tetrahedral only
  if (m->flex_dim[f] != 3) {
    return;
  }

  // initialize contact
  mjContact con;
  con.geom[0] = con.geom[1] = -1;
  con.flex[0] = con.flex[1] = f;
  con.elem[1] = con.vert[0] = -1;

  // prepare contact parameters
  int condim;
  int flex_elemnum = m->flex_elemnum[f];
  mjtNum radius = m->flex_radius[f];
  mjtNum gap, solref[mjNREF], solimp[mjNIMP], friction[5];
  mjtNum solreffriction[mjNREF] = {0};
  mj_contactParam(m, &condim, &gap, solref, solimp, friction, -1, -1, f, f);
  condim = 1;

  // process all elements
  const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
  for (int e=0; e < flex_elemnum; e++) {
    const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*4;
    con.elem[0] = e;

    // face (0,1,2)
    if (planeVertex(&con, vertxpos, radius, edata[0], edata[1], edata[2], edata[3])) {
      mj_setContact(m, &con, condim, 0, solref, solreffriction, solimp, friction);
      if (mj_addContact(m, d, &con)) return;
    }

    // face (0,2,3)
    if (planeVertex(&con, vertxpos, radius, edata[0], edata[2], edata[3], edata[1])) {
      mj_setContact(m, &con, condim, 0, solref, solreffriction, solimp, friction);
      if (mj_addContact(m, d, &con)) return;
    }

    // face (0,3,1)
    if (planeVertex(&con, vertxpos, radius, edata[0], edata[3], edata[1], edata[2])) {
      mj_setContact(m, &con, condim, 0, solref, solreffriction, solimp, friction);
      if (mj_addContact(m, d, &con)) return;
    }

    // face (1,3,2)
    if (planeVertex(&con, vertxpos, radius, edata[1], edata[3], edata[2], edata[0])) {
      mj_setContact(m, &con, condim, 0, solref, solreffriction, solimp, friction);
      if (mj_addContact(m, d, &con)) return;
    }
  }
}



// test active element self-collisions with SAP
// ignore margin to avoid permanent self-collision
void mj_collideFlexSAP(const mjModel* m, mjData* d, int f) {
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
    mju_sub3(aamm+6*i+0, elemaabb+6*elid[i], elemaabb+6*elid[i]+3);
    mju_add3(aamm+6*i+3, elemaabb+6*elid[i], elemaabb+6*elid[i]+3);
  }

  // select largest axis from flex bvh
  const mjtNum* bvh = d->bvh_aabb_dyn + 6*(m->flex_bvhadr[f] - m->nbvhstatic);
  int axis = (bvh[3] > bvh[4] && bvh[3] > bvh[5]) ? 0 : (bvh[4] > bvh[5] ? 1 : 2);

  // call SAP; hard limit on number of pairs to avoid out-of-memory
  int maxsappair = mjMIN(nactive*(nactive-1)/2, 1000000);
  int* sappair = mjSTACKALLOC(d, maxsappair, int);
  int nsappair = mj_SAP(d, aamm, nactive, axis, sappair, maxsappair);
  if (nsappair < 0) {
    mjERROR("SAP failed");
  }

  // send SAP pairs to nearphase
  for (int i=0; i < nsappair; i++) {
    int e1 = elid[sappair[i] >> 16];
    int e2 = elid[sappair[i] & 0xFFFF];
    mj_collideElems(m, d, f, e1, f, e2);
  }

  mj_freeStack(d);
}



// test a geom and an elem for collision, add to contact list
void mj_collideGeomElem(const mjModel* m, mjData* d, int g, int f, int e) {
  mjtNum margin = mj_assignMargin(m, mju_max(m->geom_margin[g], m->flex_margin[f]));
  int dim = m->flex_dim[f], type = m->geom_type[g];
  int num;

  // bounding sphere test: only if midphase is disabled
  if (mjDISABLED(mjDSBL_MIDPHASE)) {
    int eglobal = m->flex_elemadr[f] + e;
    if (filterSphereBox(d->geom_xpos+3*g, m->geom_rbound[g]+margin,
                        d->flexelem_aabb+6*eglobal)) {
      return;
    }
  }

  // skip if element has vertices on the same body as geom
  int b = m->geom_bodyid[g];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
  const int* bdata = m->flex_vertbodyid + m->flex_vertadr[f];
  for (int i=0; i <= dim; i++) {
    if (b >= 0 && b == bdata[edata[i]]) {
      return;
    }
  }

  // allocate mjContact[mjMAXCONPAIR] on the arena
  mjContact* con =
    (mjContact*) mj_arenaAllocByte(d, sizeof(mjContact) * mjMAXCONPAIR, _Alignof(mjContact));
  if (!con) {
    mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
    return;
  }

  // sphere/capsule/box : capsule
  if (dim == 1 && (type == mjGEOM_SPHERE || type == mjGEOM_CAPSULE || type == mjGEOM_BOX)) {
    // make capsule from vertices
    mjtNum pos[3], mat[9], size[2];
    mj_makeCapsule(m, d, f, m->flex_elem + m->flex_elemdataadr[f] + e*2,
                   pos, mat, size);

    // call raw primitive for corresponding geom type
    if (type == mjGEOM_SPHERE) {
      num = mjraw_SphereCapsule(con, margin,
                                d->geom_xpos+3*g, d->geom_xmat+9*g, m->geom_size+3*g,
                                pos, mat, size);
    }
    else if (type == mjGEOM_CAPSULE) {
      num = mjraw_CapsuleCapsule(con, margin,
                                 d->geom_xpos+3*g, d->geom_xmat+9*g, m->geom_size+3*g,
                                 pos, mat, size);
    }
    else {
      num = mjraw_CapsuleBox(con, margin,
                             pos, mat, size,
                             d->geom_xpos+3*g, d->geom_xmat+9*g, m->geom_size+3*g);

      // reverse contact normals, since box geom is second
      for (int i=0; i < num; i++) {
        mju_scl3(con[i].frame, con[i].frame, -1);
      }
    }
  }

  // heightfield : elem
  else if (type == mjGEOM_HFIELD) {
    num = mjc_HFieldElem(m, d, con, g, f, e, margin);
  }

  // sphere : triangle
  else if (type == mjGEOM_SPHERE && dim == 2) {
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    num = mjraw_SphereTriangle(con, margin,
                               d->geom_xpos+3*g, m->geom_size[3*g],
                               vertxpos + 3*edata[0], vertxpos + 3*edata[1],
                               vertxpos + 3*edata[2], m->flex_radius[f]);
  }

  // general geom : elem
  else {
    num = mjc_ConvexElem(m, d, con, g, -1, -1, -1, f, e, margin);
  }

  // check contacts
  if (!num) {
    resetArena(d);
    return;
  }

  // get contact parameters
  int condim;
  mjtNum gap, friction[5], solref[mjNREF], solimp[mjNIMP];
  mjtNum solreffriction[mjNREF] = {0};
  mj_contactParam(m, &condim, &gap, solref, solimp, friction, g, -1, -1, f);

  // add contacts
  for (int i=0; i < num; i++) {
    // set contact ids
    con[i].geom[0] = g;
    con[i].geom[1] = -1;
    con[i].flex[0] = -1;
    con[i].flex[1] = f;
    con[i].elem[0] = -1;
    con[i].elem[1] = e;
    con[i].vert[0] = -1;
    con[i].vert[1] = -1;

    // set remaining contact parameters
    mj_setContact(m, con + i, condim, margin-gap, solref, solreffriction, solimp, friction);
  }

  // add to ncon
  d->ncon += num;

  // move arena pointer back to the end of the contact array
  resetArena(d);
}



// test two elems for collision, add to contact list
void mj_collideElems(const mjModel* m, mjData* d, int f1, int e1, int f2, int e2) {
  mjtNum margin = mj_assignMargin(m, mju_max(m->flex_margin[f1], m->flex_margin[f2]));
  int dim1 = m->flex_dim[f1], dim2 = m->flex_dim[f2];
  int num;

  // ignore margin in self-collisions
  if (f1 == f2) {
    margin = 0;
  }

  // bounding box filter (not applied in midphase)
  if (filterBox(d->flexelem_aabb+6*(m->flex_elemadr[f1]+e1),
                d->flexelem_aabb+6*(m->flex_elemadr[f2]+e2), margin)) {
    return;
  }

  // skip if elements have vertices on the same body
  const int* edata1 = m->flex_elem + m->flex_elemdataadr[f1] + e1*(dim1+1);
  const int* edata2 = m->flex_elem + m->flex_elemdataadr[f2] + e2*(dim2+1);
  const int* bdata1 = m->flex_vertbodyid + m->flex_vertadr[f1];
  const int* bdata2 = m->flex_vertbodyid + m->flex_vertadr[f2];
  for (int i1=0; i1 <= dim1; i1++) {
    int b1 = bdata1[edata1[i1]];
    for (int i2=0; i2 <= dim2; i2++) {
      if (b1 >= 0 && b1 == bdata2[edata2[i2]]) {
        return;
      }
    }
  }

  // allocate mjContact[mjMAXCONPAIR] on the arena
  mjContact* con =
    (mjContact*) mj_arenaAllocByte(d, sizeof(mjContact) * mjMAXCONPAIR, _Alignof(mjContact));
  if (!con) {
    mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
    return;
  }

  // capsule : capsule
  if (dim1 == 1 && dim2 == 1) {
    // make capsules from vertices
    mjtNum pos1[3], mat1[9], size1[2];
    mjtNum pos2[3], mat2[9], size2[2];
    mj_makeCapsule(m, d, f1, m->flex_elem + m->flex_elemdataadr[f1] + e1*2,
                   pos1, mat1, size1);
    mj_makeCapsule(m, d, f2, m->flex_elem + m->flex_elemdataadr[f2] + e2*2,
                   pos2, mat2, size2);

    // raw primitive
    num = mjraw_CapsuleCapsule(con, margin, pos1, mat1, size1, pos2, mat2, size2);
  }

  // general convex collision
  else {
    num = mjc_ConvexElem(m, d, con, -1, f1, e1, -1, f2, e2, margin);
  }

  // check contacts
  if (!num) {
    resetArena(d);
    return;
  }

  // get contact parameters
  int condim;
  mjtNum gap, friction[5], solref[mjNREF], solimp[mjNIMP];
  mjtNum solreffriction[mjNREF] = {0};
  mj_contactParam(m, &condim, &gap, solref, solimp, friction, -1, -1, f1, f2);

  // ignore gap in self collision, since margin is ignored
  if (f1 == f2) {
    gap = 0;
  }

  // add contacts
  for (int i=0; i < num; i++) {
    // set contact ids
    con[i].geom[0] = -1;
    con[i].geom[1] = -1;
    con[i].flex[0] = f1;
    con[i].flex[1] = f2;
    con[i].elem[0] = e1;
    con[i].elem[1] = e2;
    con[i].vert[0] = -1;
    con[i].vert[1] = -1;

    // set remaining contact parameters
    mj_setContact(m, con + i, condim, margin-gap, solref, solreffriction, solimp, friction);
  }

  // add to ncon
  d->ncon += num;

  // move arena pointer back to the end of the contact array
  resetArena(d);
}



// test element and vertex for collision, add to contact list
void mj_collideElemVert(const mjModel* m, mjData* d, int f, int e, int v) {
  mjtNum margin = mj_assignMargin(m, m->flex_margin[f]);
  mjtNum radius = m->flex_radius[f];
  const mjtNum* vert = d->flexvert_xpos + 3*(m->flex_vertadr[f] + v);
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
  int num;

  // box-box filter (sphere treated as box)
  const mjtNum* aabb = d->flexelem_aabb + 6*(m->flex_elemadr[f] + e);
  mjtNum rbound = margin + radius;
  if (aabb[0]-aabb[3] > vert[0]+rbound) return;
  if (aabb[1]-aabb[4] > vert[1]+rbound) return;
  if (aabb[2]-aabb[5] > vert[2]+rbound) return;
  if (aabb[0]+aabb[3] < vert[0]-rbound) return;
  if (aabb[1]+aabb[4] < vert[1]-rbound) return;
  if (aabb[2]+aabb[5] < vert[2]-rbound) return;

  // allocate mjContact[mjMAXCONPAIR] on the arena
  mjContact* con =
    (mjContact*) mj_arenaAllocByte(d, sizeof(mjContact) * mjMAXCONPAIR, _Alignof(mjContact));
  if (!con) {
    mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
    return;
  }

  // sphere : capsule
  if (dim == 1) {
    mjtNum pos[3], mat[9], size[2];
    mjtNum I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    mj_makeCapsule(m, d, f, edata, pos, mat, size);
    num = mjraw_SphereCapsule(con, 0, vert, I, &radius, pos, mat, size);
  }

  // sphere : triangle
  else if (dim == 2) {
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    num = mjraw_SphereTriangle(con, 0, vert, radius,
                               vertxpos + 3*edata[0], vertxpos + 3*edata[1],
                               vertxpos + 3*edata[2], radius);
  }

  // sphere : tetrahdron
  else {
    num = mjc_ConvexElem(m, d, con, -1, f, -1, v, f, e, 0);
  }

  // check contacts
  if (!num) {
    resetArena(d);
    return;
  }

  // get contact parameters
  int condim;
  mjtNum gap, friction[5], solref[mjNREF], solimp[mjNIMP];
  mjtNum solreffriction[mjNREF] = {0};
  mj_contactParam(m, &condim, &gap, solref, solimp, friction, -1, -1, f, f);

  // add contacts
  for (int i=0; i < num; i++) {
    // set contact ids
    con[i].geom[0] = -1;
    con[i].geom[1] = -1;
    con[i].flex[0] = f;
    con[i].flex[1] = f;
    con[i].elem[0] = -1;
    con[i].elem[1] = e;
    con[i].vert[0] = v;
    con[i].vert[1] = -1;

    // set remaining contact parameters
    mj_setContact(m, con + i, condim, 0, solref, solreffriction, solimp, friction);
  }

  // add to ncon
  d->ncon += num;

  // move arena pointer back to the end of the contact array
  resetArena(d);
}
