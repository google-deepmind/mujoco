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
#include <mujoco/mjmodel.h>
#include "engine/engine_callback.h"
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
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



//----------------------------- collision detection entry point ------------------------------------

void mj_collision(const mjModel* m, mjData* d) {
  int g1, g2, signature, merged, b1 = 0, b2 = 0, exadr = 0, pairadr = 0, startadr;
  int nexclude = m->nexclude, npair = m->npair, nbodypair = ((m->nbody-1)*m->nbody)/2;
  int *broadphasepair = 0;
  mjMARKSTACK;

  // clear size
  d->ncon = 0;

  // return if disabled
  if (mjDISABLED(mjDSBL_CONSTRAINT) || mjDISABLED(mjDSBL_CONTACT)
      || m->nconmax==0 || m->nbody < 2) {
    return;
  }

  // predefined only; ignore exclude
  if (m->opt.collision==mjCOL_PAIR) {
    for (pairadr=0; pairadr<npair; pairadr++) {
      mj_collideGeoms(m, d, pairadr, -1, 0, 0);
    }
  }

  // dynamic only or merge; apply exclude
  else {
    // call broadphase collision detector
    broadphasepair = (int*)mj_stackAlloc(d, (m->nbody*(m->nbody-1))/2);
    nbodypair = mj_broadphase(m, d, broadphasepair, (m->nbody*(m->nbody-1))/2);

    // loop over body pairs (broadphase or all)
    for (int i=0; i<nbodypair; i++) {
      // reconstruct body pair ids
      b1 = (broadphasepair[i]>>16) & 0xFFFF;
      b2 = broadphasepair[i] & 0xFFFF;

      // compute signature for this body pair
      signature = ((b1+1)<<16) + (b2+1);

      // merge predefined pairs
      merged = 0;
      startadr = pairadr;
      if (npair && m->opt.collision==mjCOL_ALL) {
        // test all predefined pairs for which pair_signature<=signature
        while (pairadr<npair && m->pair_signature[pairadr]<=signature) {
          if (m->pair_signature[pairadr]==signature) {
            merged = 1;
          }
          mj_collideGeoms(m, d, pairadr++, -1, 0, 0);
        }
      }

      // handle exclusion
      if (nexclude) {
        // advance exadr while exclude_signature < signature
        while (exadr<nexclude && m->exclude_signature[exadr]<signature) {
          exadr++;
        }

        // skip this body pair if its signature is found in exclude array
        if (exadr<nexclude && m->exclude_signature[exadr]==signature) {
          continue;
        }
      }

      // test all geom pairs within this body pair
      if (m->body_geomnum[b1] && m->body_geomnum[b2]) {
        for (g1=m->body_geomadr[b1]; g1<m->body_geomadr[b1]+m->body_geomnum[b1]; g1++) {
          for (g2=m->body_geomadr[b2]; g2<m->body_geomadr[b2]+m->body_geomnum[b2]; g2++) {
            // merged: make sure geom pair is not repeated
            if (merged) {
              // find matching pair
              int found = 0;
              for (int k=startadr; k<pairadr; k++) {
                if ((m->pair_geom1[k]==g1 && m->pair_geom2[k]==g2) ||
                    (m->pair_geom1[k]==g2 && m->pair_geom2[k]==g1)) {
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
        }
      }
    }

    // finish merging predefined pairs
    if (npair && m->opt.collision==mjCOL_ALL) {
      while (pairadr<npair) {
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
  if (m->body_geomnum[body]==0) {
    mju_zero(aabb, 6);
    return;
  }

  // process all body geoms
  for (int i=0; i<m->body_geomnum[body]; i++) {
    // get geom id
    geom = m->body_geomadr[body]+i;

    // set _aabb for this geom
    for (int j=0; j<3; j++) {
      cen = mju_dot3(d->geom_xpos+3*geom, frame+3*j);
      _aabb[2*j] = cen - m->geom_rbound[geom] - m->geom_margin[geom];
      _aabb[2*j+1] = cen + m->geom_rbound[geom] + m->geom_margin[geom];
    }

    // update body aabb
    if (i==0) {
      mju_copy(aabb, _aabb, 6);
    } else {
      for (int j=0; j<3; j++) {
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
  for (g=start; g<end; g++) {
    if (m->geom_type[g]==mjGEOM_PLANE || m->geom_type[g]==mjGEOM_HFIELD) {
      return 1;
    }
  }

  return 0;
}



// add body pair in buffer
static void add_pair(const mjModel* m, int b1, int b2, int* npair, int* pair, int maxpair) {
  // add pair if there is room in buffer
  if ((*npair)<maxpair) {
    // exlude based on contype and conaffinity
    if (m && m->body_geomnum[b1]==1 && m->body_geomnum[b2]==1) {
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
    if (b1<b2) {
      pair[*npair] = (b1<<16) + b2;
    } else {
      pair[*npair] = (b2<<16) + b1;
    }

    (*npair)++;
  } else {
    mju_error("Broadphase buffer full");
  }
}



// comparison function for broadphase
quicksortfunc(broadcompare, context, el1, el2) {
  mjtBroadphase* b1 = (mjtBroadphase*)el1;
  mjtBroadphase* b2 = (mjtBroadphase*)el2;

  if (b1->value<b2->value) {
    return -1;
  } else if (b1->value==b2->value) {
    return 0;
  } else {
    return 1;
  }
}



// comparison function for pair sorting
quicksortfunc(paircompare, context, el1, el2) {
  int signature1 = *(int*)el1;
  int signature2 = *(int*)el2;

  if (signature1<signature2) {
    return -1;
  } else if (signature1==signature2) {
    return 0;
  } else {
    return 1;
  }
}



// does body have collidable geoms
static int can_collide(const mjModel* m, int b) {
  int g;

  // scan geoms; return if collidable
  for (g=0; g<m->body_geomnum[b]; g++) {
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
  int i, j, b1, b2, toremove, cnt, npair = 0, nbody = m->nbody, ngeom = m->ngeom;
  mjtNum cov[9], cen[3], dif[3], eigval[3], frame[9], quat[4];
  mjtBroadphase *sortbuf, *activebuf;
  mjtNum *aabb;
  mjMARKSTACK;

  // world with geoms, and body with plane or hfield, can collide all bodies
  for (b1=0; b1<nbody; b1++) {
    // cannot colide
    if (!can_collide(m, b1)) {
      continue;
    }

    // world with geoms, or welded body with plane or hfield
    if ((b1==0 && m->body_geomnum[b1]>0) || (m->body_weldid[b1]==0 && has_plane_or_hfield(m, b1))) {
      for (b2=0; b2<nbody; b2++) {
        if (b1!=b2) {
          add_pair(NULL, b1, b2, &npair, pair, maxpair);
        }
      }
    }
  }

  // find center of non-world geoms; return if none
  cnt = 0;
  mju_zero3(cen);
  for (i=0; i<ngeom; i++) {
    if (m->geom_bodyid[i]) {
      mju_addTo3(cen, d->geom_xpos+3*i);
      cnt++;
    }
  }
  if (cnt==0) {
    return npair;
  } else {
    for (i=0; i<3; i++) {
      cen[i] /= cnt;
    }
  }

  // compute covariance
  mju_zero(cov, 9);
  for (i=0; i<ngeom; i++) {
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
  for (i=0; i<9; i++) {
    cov[i] /= cnt;
  }

  // construct covariance-aligned 3D frame
  mju_eig3(eigval, frame, quat, cov);

  // allocate AABB; clear world entry (not used)
  aabb = mj_stackAlloc(d, 6*nbody);
  mju_zero(aabb, 6);

  // construct body AABB for the aligned frame, count collidable
  int bufcnt = 0;
  for (i=1; i<nbody; i++) {
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
  i = sizeof(mjtBroadphase)/sizeof(mjtNum);
  j = sizeof(mjtBroadphase)%sizeof(mjtNum);
  sortbuf = (mjtBroadphase*)mj_stackAlloc(d, 2*bufcnt*(i + (j ? 1 : 0)));
  activebuf = (mjtBroadphase*)mj_stackAlloc(d, 2*bufcnt*(i + (j ? 1 : 0)));

  // init sortbuf with axis0
  j = 0;
  for (i=1; i<nbody; i++) {
    // cannot colide
    if (!can_collide(m, i)) {
      continue;
    }

    // init
    sortbuf[2*j].body_ismax = i;
    sortbuf[2*j].value = (float)aabb[6*i];
    sortbuf[2*j+1].body_ismax = i + 0x10000;
    sortbuf[2*j+1].value = (float)aabb[6*i+1];
    j++;
  }

  // sanity check; SHOULD NOT OCCUR
  if (j!=bufcnt) {
    mju_error("Internal error in broadphase: unexpected bufcnt");
  }

  // sort along axis0
  mjQUICKSORT(sortbuf, 2*bufcnt, sizeof(mjtBroadphase), broadcompare, 0);

  // sweep and prune
  cnt = 0;    // size of active list
  for (i=0; i<2*bufcnt; i++) {
    // min value: collide with all in list, add
    if (!(sortbuf[i].body_ismax & 0x10000)) {
      for (j=0; j<cnt; j++) {
        // get body ids
        b1 = activebuf[j].body_ismax;
        b2 = sortbuf[i].body_ismax;

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
      for (j=0; j<cnt; j++) {
        if (activebuf[j].body_ismax==toremove) {
          if (j<cnt-1) {
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

// plane : geom_center distance, assuming g1 is plane
static mjtNum plane_geom(const mjModel* m, mjData* d, int g1, int g2) {
  mjtNum* mat1 = d->geom_xmat + 9*g1;
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3];

  mju_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
  return mju_dot3(dif, norm);
}


// test two geoms for collision, apply filters, add to contact list
//  flg_user disables filters and uses usermargin
void mj_collideGeoms(const mjModel* m, mjData* d, int g1, int g2, int flg_user, mjtNum usermargin) {
  int i, num, type1, type2, b1, b2, weld1, weld2, condim;
  mjtNum margin, gap, mix, friction[5], solref[mjNREF], solimp[mjNIMP];
  mjContact con[mjMAXCONPAIR];
  int ipair = (g2<0 ? g1 : -1);

  // get explicit geom ids from pair
  if (ipair>=0) {
    g1 = m->pair_geom1[ipair];
    g2 = m->pair_geom2[ipair];
  }

  // order geoms by type
  if (m->geom_type[g1] > m->geom_type[g2]) {
    i = g1;
    g1 = g2;
    g2 = i;
  }

  // copy types and bodies
  type1 = m->geom_type[g1];
  type2 = m->geom_type[g2];
  b1 = m->geom_bodyid[g1];
  b2 = m->geom_bodyid[g2];
  weld1 = m->body_weldid[b1];
  weld2 = m->body_weldid[b2];

  // return if no collision function
  if (!mjCOLLISIONFUNC[type1][type2]) {
    return;
  }

  // apply filters if not predefined pair and not flg_user
  if (ipair<0 && !flg_user) {
    // user filter if defined
    if (mjcb_contactfilter) {
      if (mjcb_contactfilter(m, d, g1, g2)) {
        return;
      }
    }

    // otherwise built-in filter
    else if (mj_contactFilter(
               type1, m->geom_contype[g1], m->geom_conaffinity[g1],
               weld1, m->body_weldid[m->body_parentid[weld1]],
               type2, m->geom_contype[g2], m->geom_conaffinity[g2],
               weld2, m->body_weldid[m->body_parentid[weld2]],
               !mjDISABLED(mjDSBL_FILTERPARENT) && weld1 && weld2)) {
      return;
    }
  }

  // set margin, gap, condim: dynamic
  if (ipair<0) {
    // margin and gap: max
    margin = mju_max(m->geom_margin[g1], m->geom_margin[g2]);
    gap = mju_max(m->geom_gap[g1], m->geom_gap[g2]);

    // condim: priority or max
    if (m->geom_priority[g1]!=m->geom_priority[g2]) {
      int gp = (m->geom_priority[g1]>m->geom_priority[g2] ? g1 : g2);
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
  if (m->geom_rbound[g1]>0 && m->geom_rbound[g2]>0 &&
      (mju_dist3(d->geom_xpos+3*g1, d->geom_xpos+3*g2) >
       m->geom_rbound[g1] + m->geom_rbound[g2] + margin)) {
    return;
  }

  // plane : bounding sphere filter
  if (m->geom_type[g1]==mjGEOM_PLANE && m->geom_rbound[g2]>0
      && plane_geom(m, d, g1, g2) > margin+m->geom_rbound[g2]) {
      return;
  }
  if (m->geom_type[g2]==mjGEOM_PLANE && m->geom_rbound[g1]>0
      && plane_geom(m, d, g2, g1) > margin+m->geom_rbound[g1]) {
      return;
  }

  // call collision detector to generate contacts
  num = mjCOLLISIONFUNC[type1][type2](m, d, con, g1, g2, margin);

  // no contacts from near-phase
  if (!num) {
    return;
  }

  // check number of contacts, SHOULD NOT OCCUR
  if (num>mjMAXCONPAIR) {
    mju_error("Too many contacts returned by collision function");
  }

  // remove repeated contacts in box-box
  if (type1==mjGEOM_BOX && type2==mjGEOM_BOX) {
    // use dim field to mark: -1: bad, 0: good
    for (i=0; i<num; i++) {
      con[i].dim = 0;
    }

    // find bad
    int j;
    for (i=0; i<num-1; i++) {
      for (j=i+1; j<num; j++) {
        if (con[i].pos[0]==con[j].pos[0] &&
            con[i].pos[1]==con[j].pos[1] &&
            con[i].pos[2]==con[j].pos[2]) {
          con[i].dim = -1;
          break;
        }
      }
    }

    // consolidate good
    i = 0;
    for (j=0; j<num; j++) {
      if (con[j].dim==0) {
        // different: copy
        if (i<j) {
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
  if (ipair<0) {
    // different priority
    if (m->geom_priority[g1]!=m->geom_priority[g2]) {
      int gp = (m->geom_priority[g1]>m->geom_priority[g2] ? g1 : g2);

      // friction
      for (i=0; i<3; i++) {
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
      for (i=0; i<3; i++) {
        friction[2*i] = mju_max(m->geom_friction[3*g1+i], m->geom_friction[3*g2+i]);
      }

      // solver mix factor
      if (m->geom_solmix[g1]>=mjMINVAL && m->geom_solmix[g2]>=mjMINVAL) {
        mix = m->geom_solmix[g1] / (m->geom_solmix[g1] + m->geom_solmix[g2]);
      } else if (m->geom_solmix[g1]<mjMINVAL && m->geom_solmix[g2]<mjMINVAL) {
        mix = 0.5;
      } else if (m->geom_solmix[g1]<mjMINVAL) {
        mix = 0.0;
      } else {
        mix = 1.0;
      }

      // reference standard: mix
      if (m->geom_solref[mjNREF*g1]>0 && m->geom_solref[mjNREF*g2]>0) {
        for (i=0; i<mjNREF; i++) {
          solref[i] = mix*m->geom_solref[mjNREF*g1+i] + (1-mix)*m->geom_solref[mjNREF*g2+i];
        }
      }

      // reference direct: min
      else {
        for (i=0; i<mjNREF; i++) {
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
    for (i=0; i<5; i++) {
      friction[i] = m->pair_friction[5*ipair+i];
    }

    // reference
    mju_copy(solref, m->pair_solref+mjNREF*ipair, mjNREF);

    // impedance
    mju_copy(solimp, m->pair_solimp+mjNIMP*ipair, mjNIMP);
  }

  // clamp friction to mjMINMU
  for (i=0; i<5; i++) {
    friction[i] = mju_max(mjMINMU, friction[i]);
  }

  // add contact returned by collision detector
  for (i=0; i<num; i++) {
    // set contact data
    if (condim > 6 || condim < 1) {  // SHOULD NOT OCCUR
      mju_error_i("Invalid condim value: %d", i);
    }
    con[i].dim = condim;
    con[i].geom1 = g1;
    con[i].geom2 = g2;
    con[i].includemargin = margin-gap;
    mju_copy(con[i].friction, friction, 5);
    mj_assignRef(m, con[i].solref, solref);
    mj_assignImp(m, con[i].solimp, solimp);

    // exclude in gap
    if (con[i].dist<con[i].includemargin) {
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
int mj_contactFilter(int type1, int contype1, int conaffinity1, int weldbody1, int weldparent1,
                     int type2, int contype2, int conaffinity2, int weldbody2, int weldparent2,
                     int filterparent) {
  // compatibility check
  if (!(contype1 & conaffinity2) && !(contype2 & conaffinity1)) {
    return 1;
  }

  // same weldbody check
  if (weldbody1==weldbody2) {
    return 1;
  }

  // weldparent check
  if (filterparent && (weldbody1==weldparent2 || weldbody2==weldparent1)) {
    return 1;
  }

  // all tests passed
  return 0;
}
