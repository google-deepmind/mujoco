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
#include "engine/engine_callback.h"
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_spatial.h"

// table of pair-wise collision functions
mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES] = {
  /*               PLANE  HFIELD  SPHERE              CAPSULE             ELLIPSOID           CYLINDER            BOX                 MESH      */
  /*PLANE     */  {0,     0,      mjc_PlaneSphere,    mjc_PlaneCapsule,   mjc_PlaneConvex,    mjc_PlaneCylinder,  mjc_PlaneBox,       mjc_PlaneConvex},
  /*HFIELD    */  {0,     0,      mjc_ConvexHField,   mjc_ConvexHField,   mjc_ConvexHField,   mjc_ConvexHField,   mjc_ConvexHField,   mjc_ConvexHField},
  /*SHPERE    */  {0,     0,      mjc_SphereSphere,   mjc_SphereCapsule,  mjc_Convex,         mjc_Convex,         mjc_SphereBox,      mjc_Convex},
  /*CAPSULE   */  {0,     0,      0,                  mjc_CapsuleCapsule, mjc_Convex,         mjc_Convex,         mjc_CapsuleBox,     mjc_Convex},
  /*ELLIPSOID */  {0,     0,      0,                  0,                  mjc_Convex,         mjc_Convex,         mjc_Convex,         mjc_Convex},
  /*CYLINDER  */  {0,     0,      0,                  0,                  0,                  mjc_Convex,         mjc_Convex,         mjc_Convex},
  /*BOX       */  {0,     0,      0,                  0,                  0,                  0,                  mjc_BoxBox,         mjc_Convex},
  /*MESH      */  {0,     0,      0,                  0,                  0,                  0,                  0,                  mjc_Convex}
};



//------------------------------------ static functions --------------------------------------------

// plane to geom_center squared distance, g1 is a plane
static mjtNum plane_geom(const mjModel* m, mjData* d, int g1, int g2) {
  mjtNum* mat1 = d->geom_xmat + 9*g1;
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3];

  mju_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
  return mju_dot3(dif, norm);
}

// squared Euclidean distance between 3D vectors
static inline mjtNum squaredDist3(const mjtNum pos1[3], const mjtNum pos2[3]) {
  mjtNum dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  return dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2];
}

// bounding-sphere collision
static int mj_collideSphere(const mjModel* m, mjData* d, int g1, int g2, mjtNum margin) {
  // neither geom is a plane
  if (m->geom_rbound[g1] > 0 && m->geom_rbound[g2] > 0) {
    mjtNum bound = m->geom_rbound[g1] + m->geom_rbound[g2] + margin;
    if (squaredDist3(d->geom_xpos+3*g1, d->geom_xpos+3*g2) > bound*bound) {
      return 0;
    }
  }

  // one geom is a plane
  if (m->geom_type[g1] == mjGEOM_PLANE && m->geom_rbound[g2] > 0
      && plane_geom(m, d, g1, g2) > margin+m->geom_rbound[g2]) {
    return 0;
  }
  if (m->geom_type[g2] == mjGEOM_PLANE && m->geom_rbound[g1] > 0
      && plane_geom(m, d, g2, g1) > margin+m->geom_rbound[g1]) {
    return 0;
  }
  return 1;
}


//------------------------------------ binary tree search ------------------------------------------

// checks if the proposed collision pair is already present in pair_geom and calls narrow phase
void mj_collidePair(const mjModel* m, mjData* d, int g1, int g2, int merged,
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
      mj_collideGeoms(m, d, g1, g2, 0, 0);
    }
  }

  // not merged: always test
  else {
    mj_collideGeoms(m, d, g1, g2, 0, 0);
  }
}

// oriented bounding boxes collision (see Gottschalk et al.)
int mj_collideOBB(const mjtNum aabb1[6], const mjtNum aabb2[6],
                  const mjtNum xpos1[3], const mjtNum xmat1[9],
                  const mjtNum xpos2[3], const mjtNum xmat2[9],
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
        mju_rotVecMat(xcenter[i], aabb[i], xmat[i]);
        mju_addTo3(xcenter[i], xpos[i]);
      }
    }
  }

  // compute normals in global coordinates
  for (int i=0; i < 2; i++) {  // bounding boxes
    for (int j=0; j < 3; j++) {  // faces
      for (int k=0; k < 3; k++) {  // world axes
        normal[i][j][k] = xmat[i][3*k+j];
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
          offset[6*i + 3*j + k] = mju_dot3(xpos[i], normal[j][k]);
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
          radius[i] = fabs(aabb[i][3]*mju_dot3(normal[i][0], normal[j][k])) +
                      fabs(aabb[i][4]*mju_dot3(normal[i][1], normal[j][k])) +
                      fabs(aabb[i][5]*mju_dot3(normal[i][2], normal[j][k]));
        } else {
          int adr = 18*i + 9*j + 3*k;
          proj[i] = aabb[i][0] * product[adr + 0] +
                    aabb[i][1] * product[adr + 1] +
                    aabb[i][2] * product[adr + 2] +
                    offset[6*i + 3*j + k];
          radius[i] = fabs(aabb[i][3]*product[adr + 0]) +
                      fabs(aabb[i][4]*product[adr + 1]) +
                      fabs(aabb[i][5]*product[adr + 2]);
        }
      }

      if (radius[0]+radius[1] < fabs(proj[1]-proj[0])) {
        return 0;
      }
    }
  }

  return 1;
}

static mjCollisionTree* mj_stackAllocTree(mjData* d, int max_stack) {
  // check that the quotient is an integer
  _Static_assert(sizeof(mjCollisionTree*) % sizeof(mjtNum) == 0,
                 "mjCollisionTree has a different size from mjtNum");
  return (mjCollisionTree*)mj_stackAlloc(
    d, max_stack * sizeof(mjCollisionTree*) / sizeof(mjtNum));
}

// binary search between two body trees
void mj_collideTree(const mjModel* m, mjData* d, int b1, int b2,
                    int merged, int startadr, int pairadr) {
  const int bvhadr1 = m->body_bvhadr[b1];
  const int bvhadr2 = m->body_bvhadr[b2];
  const mjtNum* bvh1 = m->bvh_aabb + 6 * bvhadr1;
  const mjtNum* bvh2 = m->bvh_aabb + 6 * bvhadr2;
  const int* child1 = m->bvh_child + 2 * bvhadr1;
  const int* child2 = m->bvh_child + 2 * bvhadr2;
  mjtNum product[36];  // 2 bb x 2 bb x 3 axes (body) x 3 axes (world)
  mjtNum offset[12];   // 2 bb x 2 bb x 3 axes (world)
  mjtByte initialize = 1;

  mjMARKSTACK;
  // TODO(b/273737633): Store bvh max depths to make this bound tighter.
  const int max_stack = m->body_bvhnum[b1] + m->body_bvhnum[b2];
  mjCollisionTree* stack = mj_stackAllocTree(d, max_stack);

  int nstack = 1;
  stack[0].node1 = stack[0].node2 = 0;

  while (nstack) {
    // pop from stack
    nstack--;
    int node1 = stack[nstack].node1;
    int node2 = stack[nstack].node2;
    mjtByte isleaf1 = (child1[2*node1] == -1) && (child1[2*node1+1] == -1);
    mjtByte isleaf2 = (child2[2*node2] == -1) && (child2[2*node2+1] == -1);
    int nodeid1 = m->bvh_geomid[bvhadr1 + node1];
    int nodeid2 = m->bvh_geomid[bvhadr2 + node2];

    // both are leaves
    if (isleaf1 && isleaf2 && nodeid1 != -1 && nodeid2 != -1) {
      if (mj_collideSphere(m, d, nodeid1, nodeid2, /*margin=*/ 0)) {
        if (mj_collideOBB(m->geom_aabb + 6*nodeid1, m->geom_aabb + 6*nodeid2,
                          d->geom_xpos + 3*nodeid1, d->geom_xmat + 9*nodeid1,
                          d->geom_xpos + 3*nodeid2, d->geom_xmat + 9*nodeid2,
                          NULL, NULL, &initialize)) {
          mj_collidePair(m, d, nodeid1, nodeid2, merged, startadr, pairadr);
          d->bvh_active[node1 + bvhadr1] = 1;
          d->bvh_active[node2 + bvhadr2] = 1;
        }
      }
      continue;
    }

    // if no intersection at intermediate levels, stop
    if (!mj_collideOBB(bvh1 + 6*node1, bvh2 + 6*node2,
                       d->xipos + 3*b1, d->ximat + 9*b1,
                       d->xipos + 3*b2, d->ximat + 9*b2,
                       product, offset, &initialize)) {
      continue;
    }

    d->bvh_active[node1 + bvhadr1] = 1;
    d->bvh_active[node2 + bvhadr2] = 1;

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
  mjFREESTACK;
}


//----------------------------- collision detection entry point ------------------------------------

// compare contact pairs by their geom IDs
quicksortfunc(contactcompare, context, el1, el2) {
  const mjModel* m = (const mjModel*) context;
  mjContact* con1 = (mjContact*)el1;
  mjContact* con2 = (mjContact*)el2;

  // reproduce the order contacts without mj_collideTree
  // normally sorted by (g1, g2), but in mj_collideGeoms, g1 and g2 are swapped based on geom_type.
  // here we undo this swapping for the purpose of sorting - needs to be done for each mjContact

  int con1_g1 = con1->geom1;
  int con1_g2 = con1->geom2;
  if (m->geom_type[con1_g1] > m->geom_type[con1_g2]) {
    int tmp = con1_g1;
    con1_g1 = con1_g2;
    con1_g2 = tmp;
  }
  int con2_g1 = con2->geom1;
  int con2_g2 = con2->geom2;
  if (m->geom_type[con2_g1] > m->geom_type[con2_g2]) {
    int tmp = con2_g1;
    con2_g1 = con2_g2;
    con2_g2 = tmp;
  }

  if (con1_g1 < con2_g1) return -1;
  if (con1_g1 > con2_g1) return 1;
  if (con1_g2 < con2_g2) return -1;
  if (con1_g2 > con2_g2) return 1;
  return 0;
}

void mj_collision(const mjModel* m, mjData* d) {
  int g1, g2, merged, b1 = 0, b2 = 0, exadr = 0, pairadr = 0, startadr;
  int nexclude = m->nexclude, npair = m->npair, nbodypair = ((m->nbody-1)*m->nbody)/2;
  int *broadphasepair = 0;
  mjMARKSTACK;

  // reset the size of the contact array
  d->ncon = 0;

  // reset diagnostics
  d->nbodypair_broad = 0;
  d->nbodypair_narrow = 0;
  d->ngeompair_mid = 0;
  d->ngeompair_narrow = 0;

  // reset the visualization flags
  memset(d->bvh_active, 0, m->nbvh);

  // return if disabled
  if (mjDISABLED(mjDSBL_CONSTRAINT) || mjDISABLED(mjDSBL_CONTACT)
      || m->nconmax == 0 || m->nbody < 2) {
    return;
  }

  // predefined only; ignore exclude
  if (m->opt.collision == mjCOL_PAIR) {
    d->nbodypair_broad = npair;
    for (pairadr=0; pairadr < npair; pairadr++) {
      int ngeompair_narrow_before = d->ngeompair_narrow;
      int ngeompair_mid_before = d->ngeompair_mid;
      mj_collideGeoms(m, d, pairadr, -1, 0, 0);
      if (d->ngeompair_narrow > ngeompair_narrow_before) d->nbodypair_narrow++;
      if (d->ngeompair_mid > ngeompair_mid_before) d->nbodypair_broad++;
    }
  }

  // dynamic only or merge; apply exclude
  else {
    // call broadphase collision detector
    int npairs = (m->nbody*(m->nbody - 1))/2;
    broadphasepair = mj_stackAllocInt(d, npairs);
    nbodypair = mj_broadphase(m, d, broadphasepair, npairs);
    unsigned int last_signature = -1;

    // loop over body pairs (broadphase or all)
    for (int i=0; i < nbodypair; i++) {
      // reconstruct body pair ids
      b1 = (broadphasepair[i]>>16) & 0xFFFF;
      b2 = broadphasepair[i] & 0xFFFF;

      // compute signature for this body pair
      unsigned int signature = ((b1+1)<<16) + (b2+1);
      // pairs come sorted by signature, but may not be unique
      // if signature is repeated, skip it
      if (signature == last_signature) {
        continue;
      }
      last_signature = signature;

      // merge predefined pairs
      merged = 0;
      startadr = pairadr;
      if (npair && m->opt.collision == mjCOL_ALL) {
        // test all predefined pairs for which pair_signature<=signature
        while (pairadr < npair && m->pair_signature[pairadr] <= signature) {
          if (m->pair_signature[pairadr] == signature) {
            merged = 1;
          }
          mj_collideGeoms(m, d, pairadr++, -1, 0, 0);
        }
      }

      // handle exclusion
      if (nexclude) {
        // advance exadr while exclude_signature < signature
        while (exadr < nexclude && m->exclude_signature[exadr] < signature) {
          exadr++;
        }

        // skip this body pair if its signature is found in exclude array
        if (exadr < nexclude && m->exclude_signature[exadr] == signature) {
          continue;
        }
      }

      int ngeompair_narrow_before = d->ngeompair_narrow;
      int ngeompair_mid_before = d->ngeompair_mid;

      // test all geom pairs within this body pair
      if (m->body_geomnum[b1] && m->body_geomnum[b2]) {
        if (!mjDISABLED(mjDSBL_MIDPHASE) && m->body_geomnum[b1]*m->body_geomnum[b2] > 1) {
          int ncon_before = d->ncon;
          mj_collideTree(m, d, b1, b2, merged, startadr, pairadr);
          int ncon_after = d->ncon;
          void* context = (void*) m;
          mjQUICKSORT(d->contact + ncon_before, ncon_after - ncon_before,
                      sizeof(mjContact), contactcompare, context);
        } else {
          for (g1=m->body_geomadr[b1]; g1 < m->body_geomadr[b1]+m->body_geomnum[b1]; g1++) {
            for (g2=m->body_geomadr[b2]; g2 < m->body_geomadr[b2]+m->body_geomnum[b2]; g2++) {
              mj_collidePair(m, d, g1, g2, merged, startadr, pairadr);
            }
          }
        }
      }
      if (d->ngeompair_narrow > ngeompair_narrow_before) d->nbodypair_narrow++;
      if (d->ngeompair_mid > ngeompair_mid_before) d->nbodypair_broad++;
    }

    // finish merging predefined pairs
    if (npair && m->opt.collision == mjCOL_ALL) {
      while (pairadr < npair) {
        mj_collideGeoms(m, d, pairadr++, -1, 0, 0);
      }
    }
  }

  mjFREESTACK;
}



//----------------------------- broad-phase collision detection ------------------------------------

// helper structure for SAP sorting
struct _mjtBroadphase {
  float value;
  int body_ismax;
};
typedef struct _mjtBroadphase mjtBroadphase;


// make AABB for one body
static void makeAABB(const mjModel* m, mjData* d, mjtNum* aabb, int body, const mjtNum* frame) {
  int geom;
  mjtNum _aabb[6], cen;

  // no geoms attached to body: set to 0
  if (m->body_geomnum[body] == 0) {
    mju_zero(aabb, 6);
    return;
  }

  // process all body geoms
  for (int i=0; i < m->body_geomnum[body]; i++) {
    // get geom id
    geom = m->body_geomadr[body]+i;

    // set _aabb for this geom
    for (int j=0; j < 3; j++) {
      cen = mju_dot3(d->geom_xpos+3*geom, frame+3*j);
      _aabb[2*j] = cen - m->geom_rbound[geom] - m->geom_margin[geom];
      _aabb[2*j+1] = cen + m->geom_rbound[geom] + m->geom_margin[geom];
    }

    // update body aabb
    if (i == 0) {
      mju_copy(aabb, _aabb, 6);
    } else {
      for (int j=0; j < 3; j++) {
        aabb[2*j] = mju_min(aabb[2*j], _aabb[2*j]);
        aabb[2*j+1] = mju_max(aabb[2*j+1], _aabb[2*j+1]);
      }
    }
  }
}



// return 1 if body has plane or hfield geom, 0 otherwise
static int has_plane_or_hfield(const mjModel* m, int body) {
  int start = m->body_geomadr[body];
  int end = m->body_geomadr[body] + m->body_geomnum[body];

  // scan geoms belonging to body
  int g;
  for (g=start; g < end; g++) {
    if (m->geom_type[g] == mjGEOM_PLANE || m->geom_type[g] == mjGEOM_HFIELD) {
      return 1;
    }
  }

  return 0;
}

// filter body pair: 1- discard, 0- proceed
static int body_pair_filter(int weldbody1, int weldparent1, int weldbody2,
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

// add body pair in buffer
static void add_pair(const mjModel* m, int b1, int b2, int* npair, int* pair, int maxpair) {
  // add pair if there is room in buffer
  if ((*npair) < maxpair) {
    // exlude based on contype and conaffinity
    if (m && m->body_geomnum[b1] == 1 && m->body_geomnum[b2] == 1) {
      // get contypes and conaffinities
      int contype1 = m->geom_contype[m->body_geomadr[b1]];
      int conaffinity1 = m->geom_conaffinity[m->body_geomadr[b1]];
      int contype2 = m->geom_contype[m->body_geomadr[b2]];
      int conaffinity2 = m->geom_conaffinity[m->body_geomadr[b2]];

      // compatibility check
      if (!(contype1 & conaffinity2) && !(contype2 & conaffinity1)) {
        return;
      }
    }

    // add pair
    if (b1 < b2) {
      pair[*npair] = (b1<<16) + b2;
    } else {
      pair[*npair] = (b2<<16) + b1;
    }

    (*npair)++;
  } else {
    mjERROR("broadphase buffer full");
  }
}



// comparison function for broadphase
quicksortfunc(broadcompare, context, el1, el2) {
  mjtBroadphase* b1 = (mjtBroadphase*)el1;
  mjtBroadphase* b2 = (mjtBroadphase*)el2;

  if (b1->value < b2->value) {
    return -1;
  } else if (b1->value == b2->value) {
    return 0;
  } else {
    return 1;
  }
}



// comparison function for pair sorting
quicksortfunc(paircompare, context, el1, el2) {
  int signature1 = *(int*)el1;
  int signature2 = *(int*)el2;

  if (signature1 < signature2) {
    return -1;
  } else if (signature1 == signature2) {
    return 0;
  } else {
    return 1;
  }
}



// does body have collidable geoms
static int can_collide(const mjModel* m, int b) {
  int g;

  // scan geoms; return if collidable
  for (g=0; g < m->body_geomnum[b]; g++) {
    int ind = m->body_geomadr[b] + g;
    if (m->geom_contype[ind] || m->geom_conaffinity[ind]) {
      return 1;
    }
  }

  // none found
  return 0;
}



// broadphase collision detector
int mj_broadphase(const mjModel* m, mjData* d, int* pair, int maxpair) {
  int b1, b2, toremove, cnt, npair = 0, nbody = m->nbody, ngeom = m->ngeom;
  mjtNum cov[9], cen[3], dif[3], eigval[3], frame[9], quat[4];
  mjtBroadphase *sortbuf, *activebuf;
  mjtNum *aabb;
  mjMARKSTACK;

  int dsbl_filterparent = mjDISABLED(mjDSBL_FILTERPARENT);
  // world with geoms, and body with plane or hfield, can collide all bodies
  for (b1=0; b1 < nbody; b1++) {
    // cannot colide
    if (!can_collide(m, b1)) {
      continue;
    }

    // world with geoms, or welded body with plane or hfield
    if ((b1 == 0 && m->body_geomnum[b1] > 0) ||
        (m->body_weldid[b1] == 0 && has_plane_or_hfield(m, b1))) {
      int weld1 = 0;
      int parent_weld1 = 0;
      for (b2=0; b2 < nbody; b2++) {
        int weld2 = m->body_weldid[b2];
        int parent_weld2 = m->body_weldid[m->body_parentid[weld2]];
        if (!body_pair_filter(weld1, parent_weld1, weld2, parent_weld2,
                              dsbl_filterparent)) {
          add_pair(NULL, b1, b2, &npair, pair, maxpair);
        }
      }
    }
  }

  // find center of non-world geoms; return if none
  cnt = 0;
  mju_zero3(cen);
  for (int i=0; i < ngeom; i++) {
    if (m->geom_bodyid[i]) {
      mju_addTo3(cen, d->geom_xpos+3*i);
      cnt++;
    }
  }
  if (cnt == 0) {
    return npair;
  } else {
    for (int i=0; i < 3; i++) {
      cen[i] /= cnt;
    }
  }

  // compute covariance
  mju_zero(cov, 9);
  for (int i=0; i < ngeom; i++) {
    if (m->geom_bodyid[i]) {
      mju_sub3(dif, d->geom_xpos+3*i, cen);
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
  }
  for (int i=0; i < 9; i++) {
    cov[i] /= cnt;
  }

  // construct covariance-aligned 3D frame
  mju_eig3(eigval, frame, quat, cov);

  // allocate AABB; clear world entry (not used)
  aabb = mj_stackAlloc(d, 6*nbody);
  mju_zero(aabb, 6);

  // construct body AABB for the aligned frame, count collidable
  int bufcnt = 0;
  for (int i=1; i < nbody; i++) {
    makeAABB(m, d, aabb+6*i, i, frame);

    if (can_collide(m, i)) {
      bufcnt++;
    }
  }

  // nothing collidable
  if (!bufcnt) {
    goto endbroad;
  }

  // allocate sort buffer
  int quot = sizeof(mjtBroadphase)/sizeof(mjtNum);
  int rem = sizeof(mjtBroadphase)%sizeof(mjtNum);
  sortbuf = (mjtBroadphase*)mj_stackAlloc(d, 2*bufcnt*(quot + (rem ? 1 : 0)));
  activebuf = (mjtBroadphase*)mj_stackAlloc(d, 2*bufcnt*(quot + (rem ? 1 : 0)));

  // init sortbuf with axis0
  int k = 0;
  for (int i=1; i < nbody; i++) {
    // cannot colide
    if (!can_collide(m, i)) {
      continue;
    }

    // init
    sortbuf[2*k].body_ismax = i;
    sortbuf[2*k].value = (float)aabb[6*i];
    sortbuf[2*k+1].body_ismax = i + 0x10000;
    sortbuf[2*k+1].value = (float)aabb[6*i+1];
    k++;
  }

  // sanity check; SHOULD NOT OCCUR
  if (k != bufcnt) {
    mjERROR("internal error: unexpected bufcnt");
  }

  // sort along axis0
  mjQUICKSORT(sortbuf, 2*bufcnt, sizeof(mjtBroadphase), broadcompare, 0);

  // sweep and prune
  cnt = 0;    // size of active list
  for (int i=0; i < 2*bufcnt; i++) {
    // min value: collide with all in list, add
    if (!(sortbuf[i].body_ismax & 0x10000)) {
      for (int j=0; j < cnt; j++) {
        // get body ids: no need to mask ismax because activebuf entries never have the ismax bit,
        // and sortbuf[i].body_ismax is tested above
        b1 = activebuf[j].body_ismax;
        b2 = sortbuf[i].body_ismax;

        int weld1 = m->body_weldid[b1];
        int weld2 = m->body_weldid[b2];
        int parent_weld1 = m->body_weldid[m->body_parentid[weld1]];
        int parent_weld2 = m->body_weldid[m->body_parentid[weld2]];

        if (body_pair_filter(weld1, parent_weld1, weld2, parent_weld2,
                             dsbl_filterparent)) {
          continue;
        }

        // use the other two axes to prune if possible
        if (aabb[6*b1+2] > aabb[6*b2+3] ||
            aabb[6*b1+3] < aabb[6*b2+2] ||
            aabb[6*b1+4] > aabb[6*b2+5] ||
            aabb[6*b1+5] < aabb[6*b2+4]) {
          continue;
        }

        // add body pair if there is room in buffer
        add_pair(m, b1, b2, &npair, pair, maxpair);
      }

      // add to list
      activebuf[cnt] = sortbuf[i];
      cnt++;
    }

    // max value: remove corresponding min value from list
    else {
      toremove = sortbuf[i].body_ismax & 0xFFFF;
      for (int j=0; j < cnt; j++) {
        if (activebuf[j].body_ismax == toremove) {
          if (j < cnt-1) {
            memmove(activebuf+j, activebuf+j+1, sizeof(mjtBroadphase)*(cnt-1-j));
          }
          cnt--;
          break;
        }
      }
    }
  }

endbroad:

  // sort pairs by signature
  if (npair) {
    mjQUICKSORT(pair, npair, sizeof(int), paircompare, 0);
  }

  mjFREESTACK;
  return npair;
}



//----------------------------- narrow-phase collision detection -----------------------------------


// test two geoms for collision, apply filters, add to contact list
//  flg_user disables filters and uses usermargin
void mj_collideGeoms(const mjModel* m, mjData* d, int g1, int g2, int flg_user, mjtNum usermargin) {
  int num, type1, type2, condim;
  mjtNum margin, gap, mix, friction[5], solref[mjNREF], solimp[mjNIMP];
  mjtNum solreffriction[mjNREF] = {0};
  mjContact con[mjMAXCONPAIR];
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

  // return if no collision function
  if (!mjCOLLISIONFUNC[type1][type2]) {
    return;
  }

  // apply filters if not predefined pair and not flg_user
  if (ipair < 0 && !flg_user) {
    // user filter if defined
    if (mjcb_contactfilter) {
      if (mjcb_contactfilter(m, d, g1, g2)) {
        return;
      }
    }

    // otherwise built-in filter
    else if (mj_contactFilter(m->geom_contype[g1], m->geom_conaffinity[g1],
                              m->geom_contype[g2], m->geom_conaffinity[g2])) {
      return;
    }
  }

  // set margin, gap, condim: dynamic
  if (ipair < 0) {
    // margin and gap: max
    margin = mju_max(m->geom_margin[g1], m->geom_margin[g2]);
    gap = mju_max(m->geom_gap[g1], m->geom_gap[g2]);

    // condim: priority or max
    if (m->geom_priority[g1] != m->geom_priority[g2]) {
      int gp = (m->geom_priority[g1] > m->geom_priority[g2] ? g1 : g2);
      condim = m->geom_condim[gp];
    } else {
      condim = mjMAX(m->geom_condim[g1], m->geom_condim[g2]);
    }
  }

  // set margin, gap, condim: pair
  else {
    margin = m->pair_margin[ipair];
    gap = m->pair_gap[ipair];
    condim = m->pair_dim[ipair];
  }

  // adjust margin
  if (flg_user) {
    margin = usermargin;
  } else {
    margin = mj_assignMargin(m, margin);
  }

  // bounding sphere filter
  if (!mj_collideSphere(m, d, g1, g2, margin)) {
    return;
  }

  // increment counter of expected collisions
  d->ngeompair_mid++;

  // call collision detector to generate contacts
  num = mjCOLLISIONFUNC[type1][type2](m, d, con, g1, g2, margin);

  // no contacts from near-phase
  if (!num) {
    return;
  }

  // increment counter of actual collisions
  d->ngeompair_narrow++;

  // check number of contacts, SHOULD NOT OCCUR
  if (num > mjMAXCONPAIR) {
    mjERROR("too many contacts returned by collision function");
  }

  // remove repeated contacts in box-box
  if (type1 == mjGEOM_BOX && type2 == mjGEOM_BOX) {
    // use dim field to mark: -1: bad, 0: good
    for (int i=0; i < num; i++) {
      con[i].dim = 0;
    }

    // find bad
    for (int i=0; i < num-1; i++) {
      for (int j=i+1; j < num; j++) {
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

  // set friction, solref, solimp: dynamic
  if (ipair < 0) {
    // different priority
    if (m->geom_priority[g1] != m->geom_priority[g2]) {
      int gp = (m->geom_priority[g1] > m->geom_priority[g2] ? g1 : g2);

      // friction
      for (int i=0; i < 3; i++) {
        friction[2*i] = m->geom_friction[3*gp+i];
      }

      // reference
      mju_copy(solref, m->geom_solref+mjNREF*gp, mjNREF);

      // impedance
      mju_copy(solimp, m->geom_solimp+mjNIMP*gp, mjNIMP);
    }

    // same priority
    else {
      // friction: max
      for (int i=0; i < 3; i++) {
        friction[2*i] = mju_max(m->geom_friction[3*g1+i], m->geom_friction[3*g2+i]);
      }

      // solver mix factor
      if (m->geom_solmix[g1] >= mjMINVAL && m->geom_solmix[g2] >= mjMINVAL) {
        mix = m->geom_solmix[g1] / (m->geom_solmix[g1] + m->geom_solmix[g2]);
      } else if (m->geom_solmix[g1] < mjMINVAL && m->geom_solmix[g2] < mjMINVAL) {
        mix = 0.5;
      } else if (m->geom_solmix[g1] < mjMINVAL) {
        mix = 0.0;
      } else {
        mix = 1.0;
      }

      // reference standard: mix
      if (m->geom_solref[mjNREF*g1] > 0 && m->geom_solref[mjNREF*g2] > 0) {
        for (int i=0; i < mjNREF; i++) {
          solref[i] = mix*m->geom_solref[mjNREF*g1+i] + (1-mix)*m->geom_solref[mjNREF*g2+i];
        }
      }

      // reference direct: min
      else {
        for (int i=0; i < mjNREF; i++) {
          solref[i] = mju_min(m->geom_solref[mjNREF*g1+i], m->geom_solref[mjNREF*g2+i]);
        }
      }

      // impedance: mix
      mju_scl(solimp, m->geom_solimp+mjNIMP*g1, mix, mjNIMP);
      mju_addToScl(solimp, m->geom_solimp+mjNIMP*g2, 1-mix, mjNIMP);
    }

    // unpack 5D friction
    friction[1] = friction[0];
    friction[3] = friction[4];
  }

  // set friction, solref, solimp: pair
  else {
    // friction
    for (int i=0; i < 5; i++) {
      friction[i] = m->pair_friction[5*ipair+i];
    }

    // reference, normal direction
    mju_copy(solref, m->pair_solref+mjNREF*ipair, mjNREF);

    // reference, friction directions
    if (m->pair_solreffriction[mjNREF*ipair] || m->pair_solreffriction[mjNREF*ipair + 1]) {
      mju_copy(solreffriction, m->pair_solreffriction+mjNREF*ipair, mjNREF);
    }

    // impedance
    mju_copy(solimp, m->pair_solimp+mjNIMP*ipair, mjNIMP);
  }

  // clamp friction to mjMINMU
  for (int i=0; i < 5; i++) {
    friction[i] = mju_max(mjMINMU, friction[i]);
  }

  // add contact returned by collision detector
  for (int i=0; i < num; i++) {
    if (condim > 6 || condim < 1) {
      mjERROR("invalid condim value: %d", i);  // SHOULD NOT OCCUR
    }

    // set contact data
    con[i].dim = condim;
    con[i].geom1 = g1;
    con[i].geom2 = g2;
    con[i].includemargin = margin-gap;
    mju_copy(con[i].friction, friction, 5);
    mj_assignRef(m, con[i].solref, solref);
    mj_assignRef(m, con[i].solreffriction, solreffriction);
    mj_assignImp(m, con[i].solimp, solimp);

    // exclude in gap
    if (con[i].dist < con[i].includemargin) {
      con[i].exclude = 0;
    } else {
      con[i].exclude = 1;
    }

    // complete frame
    mju_makeFrame(con[i].frame);

    // clear fields that are computed later
    con[i].efc_address = -1;
    con[i].mu = 0;
    mju_zero(con[i].H, 36);
    // add to mjData, abort if too many contacts
    if (mj_addContact(m, d, con + i)) {
      return;
    }
  }
}



// filter contacts: 1- discard, 0- proceed
int mj_contactFilter(int contype1, int conaffinity1,
                     int contype2, int conaffinity2) {
  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}
