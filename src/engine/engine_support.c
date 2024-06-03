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

#include "engine/engine_support.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

#ifdef mjUSEPLATFORMSIMD
  #if defined(__AVX__) && defined(mjUSEDOUBLE)
    #define mjUSEAVX
    #include "immintrin.h"
  #endif
#endif

//-------------------------- Constants -------------------------------------------------------------

 #define mjVERSION 316
#define mjVERSIONSTRING "3.1.6"

// names of disable flags
const char* mjDISABLESTRING[mjNDISABLE] = {
  "Constraint",
  "Equality",
  "Frictionloss",
  "Limit",
  "Contact",
  "Passive",
  "Gravity",
  "Clampctrl",
  "Warmstart",
  "Filterparent",
  "Actuation",
  "Refsafe",
  "Sensor",
  "Midphase",
  "Eulerdamp"
};


// names of enable flags
const char* mjENABLESTRING[mjNENABLE] = {
  "Override",
  "Energy",
  "Fwdinv",
  "InvDiscrete",
  "MultiCCD",
  "Island"
};


// names of timers
const char* mjTIMERSTRING[mjNTIMER]= {
  "step",
  "forward",
  "inverse",
  "position",
  "velocity",
  "actuation",
  "constraint",
  "advance",
  "pos_kinematics",
  "pos_inertia",
  "pos_collision",
  "pos_make",
  "pos_project",
  "col_broadphase",
  "col_narrowphase"
};



//-------------------------- get/set state ---------------------------------------------------------

// return size of a single state element
static inline int mj_stateElemSize(const mjModel* m, mjtState spec) {
  switch (spec) {
  case mjSTATE_TIME:          return 1;
  case mjSTATE_QPOS:          return m->nq;
  case mjSTATE_QVEL:          return m->nv;
  case mjSTATE_ACT:           return m->na;
  case mjSTATE_WARMSTART:     return m->nv;
  case mjSTATE_CTRL:          return m->nu;
  case mjSTATE_QFRC_APPLIED:  return m->nv;
  case mjSTATE_XFRC_APPLIED:  return 6*m->nbody;
  case mjSTATE_EQ_ACTIVE:     return m->neq;    // mjtByte, stored as mjtNum in state vector
  case mjSTATE_MOCAP_POS:     return 3*m->nmocap;
  case mjSTATE_MOCAP_QUAT:    return 4*m->nmocap;
  case mjSTATE_USERDATA:      return m->nuserdata;
  case mjSTATE_PLUGIN:        return m->npluginstate;
  default:
    mjERROR("invalid state element %u", spec);
    return 0;
  }
}



// return pointer to a single state element
static inline mjtNum* mj_stateElemPtr(const mjModel* m, mjData* d, mjtState spec) {
  switch (spec) {
  case mjSTATE_TIME:          return &d->time;
  case mjSTATE_QPOS:          return d->qpos;
  case mjSTATE_QVEL:          return d->qvel;
  case mjSTATE_ACT:           return d->act;
  case mjSTATE_WARMSTART:     return d->qacc_warmstart;
  case mjSTATE_CTRL:          return d->ctrl;
  case mjSTATE_QFRC_APPLIED:  return d->qfrc_applied;
  case mjSTATE_XFRC_APPLIED:  return d->xfrc_applied;
  case mjSTATE_MOCAP_POS:     return d->mocap_pos;
  case mjSTATE_MOCAP_QUAT:    return d->mocap_quat;
  case mjSTATE_USERDATA:      return d->userdata;
  case mjSTATE_PLUGIN:        return d->plugin_state;
  default:
    mjERROR("invalid state element %u", spec);
    return NULL;
  }
}



static inline const mjtNum* mj_stateElemConstPtr(const mjModel* m, const mjData* d, mjtState spec) {
  return mj_stateElemPtr(m, (mjData*) d, spec);  // discard const qualifier from d
}



// get size of state specification
int mj_stateSize(const mjModel* m, unsigned int spec) {
  if (spec >= (1<<mjNSTATE)) {
    mjERROR("invalid state spec %u >= 2^mjNSTATE", spec);
  }

  int size = 0;
  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & spec) {
      size += mj_stateElemSize(m, element);
    }
  }

  return size;
}



// get state
void mj_getState(const mjModel* m, const mjData* d, mjtNum* state, unsigned int spec) {
  if (spec >= (1<<mjNSTATE)) {
    mjERROR("invalid state spec %u >= 2^mjNSTATE", spec);
  }

  int adr = 0;
  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & spec) {
      int size = mj_stateElemSize(m, element);

      // special handling of eq_active (mjtByte)
      if (element == mjSTATE_EQ_ACTIVE) {
        int neq = m->neq;
        for (int j=0; j < neq; j++) {
          state[adr++] = d->eq_active[j];
        }
      }

      // regular state components (mjtNum)
      else {
        const mjtNum* ptr = mj_stateElemConstPtr(m, d, element);
        mju_copy(state + adr, ptr, size);
        adr += size;
      }
    }
  }
}



// set state
void mj_setState(const mjModel* m, mjData* d, const mjtNum* state, unsigned int spec) {
  if (spec >= (1<<mjNSTATE)) {
    mjERROR("invalid state spec %u >= 2^mjNSTATE", spec);
  }

  int adr = 0;
  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & spec) {
      int size = mj_stateElemSize(m, element);

      // special handling of eq_active (mjtByte)
      if (element == mjSTATE_EQ_ACTIVE) {
        int neq = m->neq;
        for (int j=0; j < neq; j++) {
          d->eq_active[j] = state[adr++];
        }
      }

      // regular state components (mjtNum)
      else {
        mjtNum* ptr = mj_stateElemPtr(m, d, element);
        mju_copy(ptr, state + adr, size);
        adr += size;
      }
    }
  }
}



//-------------------------- sparse chains ---------------------------------------------------------

// merge dof chains for two bodies
int mj_mergeChain(const mjModel* m, int* chain, int b1, int b2) {
  int da1, da2, NV = 0;

  // skip fixed bodies
  while (b1 && !m->body_dofnum[b1]) {
    b1 = m->body_parentid[b1];
  }
  while (b2 && !m->body_dofnum[b2]) {
    b2 = m->body_parentid[b2];
  }

  // neither body is movable: empty chain
  if (b1 == 0 && b2 == 0) {
    return 0;
  }

  // intialize last dof address for each body
  da1 = m->body_dofadr[b1] + m->body_dofnum[b1] - 1;
  da2 = m->body_dofadr[b2] + m->body_dofnum[b2] - 1;

  // merge chains
  while (da1 >= 0 || da2 >= 0) {
    chain[NV] = mjMAX(da1, da2);
    if (da1 == chain[NV]) {
      da1 = m->dof_parentid[da1];
    }
    if (da2 == chain[NV]) {
      da2 = m->dof_parentid[da2];
    }
    NV++;
  }

  // reverse order of chain: make it increasing
  for (int i=0; i < NV/2; i++) {
    int tmp = chain[i];
    chain[i] = chain[NV-i-1];
    chain[NV-i-1] = tmp;
  }

  return NV;
}



// merge dof chains for two simple bodies
int mj_mergeChainSimple(const mjModel* m, int* chain, int b1, int b2) {
  // swap bodies if wrong order
  if (b1 > b2) {
    int tmp = b1;
    b1 = b2;
    b2 = tmp;
  }

  // init
  int n1 = m->body_dofnum[b1], n2 = m->body_dofnum[b2];

  // both fixed: nothing to do
  if (n1 == 0 && n2 == 0) {
    return 0;
  }

  // copy b1 dofs
  for (int i=0; i < n1; i++) {
    chain[i] = m->body_dofadr[b1] + i;
  }

  // copy b2 dofs
  for (int i=0; i < n2; i++) {
    chain[n1+i] = m->body_dofadr[b2] + i;
  }

  return (n1+n2);
}



// get body chain
int mj_bodyChain(const mjModel* m, int body, int* chain) {
  // simple body
  if (m->body_simple[body]) {
    int dofnum = m->body_dofnum[body];
    for (int i=0; i < dofnum; i++) {
      chain[i] = m->body_dofadr[body] + i;
    }
    return dofnum;
  }

  // general case
  else {
    // skip fixed bodies
    while (body && !m->body_dofnum[body]) {
      body = m->body_parentid[body];
    }

    // not movable: empty chain
    if (body == 0) {
      return 0;
    }

    // intialize last dof
    int da = m->body_dofadr[body] + m->body_dofnum[body] - 1;
    int NV = 0;

    // construct chain from child to parent
    while (da >= 0) {
      chain[NV++] = da;
      da = m->dof_parentid[da];
    }

    // reverse order of chain: make it increasing
    for (int i=0; i < NV/2; i++) {
      int tmp = chain[i];
      chain[i] = chain[NV-i-1];
      chain[NV-i-1] = tmp;
    }

    return NV;
  }
}



//-------------------------- Jacobians -------------------------------------------------------------

// compute 3/6-by-nv Jacobian of global point attached to given body
void mj_jac(const mjModel* m, const mjData* d,
            mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body) {
  int da, nv = m->nv;
  mjtNum offset[3], tmp[3], *cdof = d->cdof;

  // clear jacobians
  if (jacp) {
    mju_zero(jacp, 3*nv);
  }
  if (jacr) {
    mju_zero(jacr, 3*nv);
  }

  // compute point-com offset
  mju_sub3(offset, point, d->subtree_com+3*m->body_rootid[body]);

  // skip fixed bodies
  while (body && !m->body_dofnum[body]) {
    body = m->body_parentid[body];
  }

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  da = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // backward pass over dof ancestor chain
  while (da >= 0) {
    // construct rotation jacobian
    if (jacr) {
      jacr[da] = cdof[6*da];
      jacr[da+nv] = cdof[6*da+1];
      jacr[da+2*nv] = cdof[6*da+2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      mju_cross(tmp, cdof+6*da, offset);
      jacp[da] = cdof[6*da+3] + tmp[0];
      jacp[da+nv] = cdof[6*da+4] + tmp[1];
      jacp[da+2*nv] = cdof[6*da+5] + tmp[2];
    }

    // advance to parent dof
    da = m->dof_parentid[da];
  }
}



// compute body Jacobian
void mj_jacBody(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body) {
  mj_jac(m, d, jacp, jacr, d->xpos+3*body, body);
}



// compute body-com Jacobian
void mj_jacBodyCom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body) {
  mj_jac(m, d, jacp, jacr, d->xipos+3*body, body);
}



// compute subtree-com Jacobian
void mj_jacSubtreeCom(const mjModel* m, mjData* d, mjtNum* jacp, int body) {
  int nv = m->nv;
  mj_markStack(d);
  mjtNum* jacp_b = mj_stackAllocNum(d, 3*nv);

  // clear output
  mju_zero(jacp, 3*nv);

  // forward pass starting from body
  for (int b=body; b < m->nbody; b++) {
    // end of body subtree, break from the loop
    if (b > body && m->body_parentid[b] < body) {
      break;
    }

    // b is in the body subtree, add mass-weighted Jacobian into jacp
    mj_jac(m, d, jacp_b, NULL, d->xipos+3*b, b);
    mju_addToScl(jacp, jacp_b, m->body_mass[b], 3*nv);
  }

  // normalize by subtree mass
  mju_scl(jacp, jacp, 1/m->body_subtreemass[body], 3*nv);

  mj_freeStack(d);
}



// compute geom Jacobian
void mj_jacGeom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int geom) {
  mj_jac(m, d, jacp, jacr, d->geom_xpos + 3*geom, m->geom_bodyid[geom]);
}



// compute site Jacobian
void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site) {
  mj_jac(m, d, jacp, jacr, d->site_xpos + 3*site, m->site_bodyid[site]);
}



// compute translation Jacobian of point, and rotation Jacobian of axis
void mj_jacPointAxis(const mjModel* m, mjData* d, mjtNum* jacPoint, mjtNum* jacAxis,
                     const mjtNum point[3], const mjtNum axis[3], int body) {
  int nv = m->nv;

  // get full Jacobian of point
  mj_markStack(d);
  mjtNum* jacp = (jacPoint ? jacPoint : mj_stackAllocNum(d, 3*nv));
  mjtNum* jacr = mj_stackAllocNum(d, 3*nv);
  mj_jac(m, d, jacp, jacr, point, body);

  // jacAxis_col = cross(jacr_col, axis)
  if (jacAxis) {
    for (int i=0; i < nv; i++) {
      jacAxis[     i] = jacr[  nv+i]*axis[2] - jacr[2*nv+i]*axis[1];
      jacAxis[  nv+i] = jacr[2*nv+i]*axis[0] - jacr[     i]*axis[2];
      jacAxis[2*nv+i] = jacr[     i]*axis[1] - jacr[  nv+i]*axis[0];
    }
  }

  mj_freeStack(d);
}



// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void mj_jacSparse(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body,
                  int NV, const int* chain) {
  int da, ci;
  mjtNum offset[3], tmp[3], *cdof = d->cdof;

  // clear jacobians
  if (jacp) {
    mju_zero(jacp, 3*NV);
  }
  if (jacr) {
    mju_zero(jacr, 3*NV);
  }

  // compute point-com offset
  mju_sub3(offset, point, d->subtree_com+3*m->body_rootid[body]);

  // skip fixed bodies
  while (body && !m->body_dofnum[body]) {
    body = m->body_parentid[body];
  }

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  da = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // start and the end of the chain (chain is in increasing order)
  ci = NV-1;

  // backward pass over dof ancestor chain
  while (da >= 0) {
    // find chain index for this dof
    while (ci >= 0 && chain[ci] > da) {
      ci--;
    }

    // make sure we found it; SHOULD NOT OCCUR
    if (chain[ci] != da) {
      mjERROR("dof index %d not found in chain", da);
    }

    // construct rotation jacobian
    if (jacr) {
      jacr[ci] = cdof[6*da];
      jacr[ci+NV] = cdof[6*da+1];
      jacr[ci+2*NV] = cdof[6*da+2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      mju_cross(tmp, cdof+6*da, offset);

      jacp[ci] = cdof[6*da+3] + tmp[0];
      jacp[ci+NV] = cdof[6*da+4] + tmp[1];
      jacp[ci+2*NV] = cdof[6*da+5] + tmp[2];
    }

    // advance to parent dof
    da = m->dof_parentid[da];
  }
}



// sparse Jacobian difference for simple body contacts
void mj_jacSparseSimple(const mjModel* m, const mjData* d,
                        mjtNum* jacdifp, mjtNum* jacdifr, const mjtNum* point,
                        int body, int flg_second, int NV, int start) {
  mjtNum offset[3], tmp[3], *cdof = d->cdof;

  // compute point-com offset
  mju_sub3(offset, point, d->subtree_com+3*m->body_rootid[body]);

  // skip fixed body
  if (!m->body_dofnum[body]) {
    return;
  }

  // process dofs
  int ci = start;
  int end = m->body_dofadr[body] + m->body_dofnum[body];
  for (int da=m->body_dofadr[body]; da < end; da++) {
    // construct rotation jacobian
    if (jacdifr) {
      // plus sign
      if (flg_second) {
        jacdifr[ci] = cdof[6*da];
        jacdifr[ci+NV] = cdof[6*da+1];
        jacdifr[ci+2*NV] = cdof[6*da+2];
      }

      // minus sign
      else {
        jacdifr[ci] = -cdof[6*da];
        jacdifr[ci+NV] = -cdof[6*da+1];
        jacdifr[ci+2*NV] = -cdof[6*da+2];
      }
    }

    // construct translation jacobian (correct for rotation)
    if (jacdifp) {
      mju_cross(tmp, cdof+6*da, offset);

      // plus sign
      if (flg_second) {
        jacdifp[ci] = (cdof[6*da+3] + tmp[0]);
        jacdifp[ci+NV] = (cdof[6*da+4] + tmp[1]);
        jacdifp[ci+2*NV] = (cdof[6*da+5] + tmp[2]);
      }

      // plus sign
      else {
        jacdifp[ci] = -(cdof[6*da+3] + tmp[0]);
        jacdifp[ci+NV] = -(cdof[6*da+4] + tmp[1]);
        jacdifp[ci+2*NV] = -(cdof[6*da+5] + tmp[2]);
      }
    }

    // advance jacdif counter
    ci++;
  }
}



// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
int mj_jacDifPair(const mjModel* m, const mjData* d, int* chain,
                  int b1, int b2, const mjtNum pos1[3], const mjtNum pos2[3],
                  mjtNum* jac1p, mjtNum* jac2p, mjtNum* jacdifp,
                  mjtNum* jac1r, mjtNum* jac2r, mjtNum* jacdifr) {
  int issimple = (m->body_simple[b1] && m->body_simple[b2]);
  int issparse = mj_isSparse(m);
  int NV = m->nv;

  // skip if no DOFs
  if (!NV) {
    return 0;
  }

  // construct merged chain of body dofs
  if (issparse) {
    if (issimple) {
      NV = mj_mergeChainSimple(m, chain, b1, b2);
    } else {
      NV = mj_mergeChain(m, chain, b1, b2);
    }
  }

  // skip if empty chain
  if (!NV) {
    return 0;
  }

  // sparse case
  if (issparse) {
    // simple: fast processing
    if (issimple) {
      // first body
      mj_jacSparseSimple(m, d, jacdifp, jacdifr, pos1, b1, 0, NV,
                         b1 < b2 ? 0 : m->body_dofnum[b2]);

      // second body
      mj_jacSparseSimple(m, d, jacdifp, jacdifr, pos2, b2, 1, NV,
                         b2 < b1 ? 0 : m->body_dofnum[b1]);
    }

    // regular processing
    else {
      // Jacobians
      mj_jacSparse(m, d, jac1p, jac1r, pos1, b1, NV, chain);
      mj_jacSparse(m, d, jac2p, jac2r, pos2, b2, NV, chain);

      // differences
      if (jacdifp) {
        mju_sub(jacdifp, jac2p, jac1p, 3*NV);
      }
      if (jacdifr) {
        mju_sub(jacdifr, jac2r, jac1r, 3*NV);
      }
    }
  }

  // dense case
  else {
    // Jacobians
    mj_jac(m, d, jac1p, jac1r, pos1, b1);
    mj_jac(m, d, jac2p, jac2r, pos2, b2);

    // differences
    if (jacdifp) {
      mju_sub(jacdifp, jac2p, jac1p, 3*NV);
    }
    if (jacdifr) {
      mju_sub(jacdifr, jac2r, jac1r, 3*NV);
    }
  }

  return NV;
}



// dense or sparse weighted sum of multiple body Jacobians at same point
int mj_jacSum(const mjModel* m, mjData* d, int* chain,
              int n, const int* body, const mjtNum* weight,
              const mjtNum point[3], mjtNum* jac, int flg_rot) {
  int nv = m->nv, NV;
  mjtNum* jacp = jac;
  mjtNum* jacr = flg_rot ? jac + 3*nv : NULL;

  mj_markStack(d);
  mjtNum* jtmp = mj_stackAllocNum(d, flg_rot ? 6*nv : 3*nv);
  mjtNum* jp = jtmp;
  mjtNum* jr = flg_rot ? jtmp + 3*nv : NULL;

  // sparse
  if (mj_isSparse(m)) {
    mjtNum* buf = mj_stackAllocNum(d, flg_rot ? 6*nv : 3*nv);
    int* buf_ind = mj_stackAllocInt(d, nv);
    int* bodychain = mj_stackAllocInt(d, nv);

    // set first
    NV = mj_bodyChain(m, body[0], chain);
    if (NV) {
      // get Jacobian
      if (m->body_simple[body[0]]) {
        mj_jacSparseSimple(m, d, jacp, jacr, point, body[0], 1, NV, 0);
      } else {
        mj_jacSparse(m, d, jacp, jacr, point, body[0], NV, chain);
      }

      // apply weight
      mju_scl(jac, jac, weight[0], flg_rot ? 6*NV : 3*NV);
    }

    // accumulate remaining
    for (int i=1; i < n; i++) {
      // get body chain and Jacobian
      int bodyNV = mj_bodyChain(m, body[i], bodychain);
      if (!bodyNV) {
        continue;
      }
      if (m->body_simple[body[i]]) {
        mj_jacSparseSimple(m, d, jp, jr, point, body[i], 1, bodyNV, 0);
      } else {
        mj_jacSparse(m, d, jp, jr, point, body[i], bodyNV, bodychain);
      }

      // combine sparse matrices
      NV = mju_addToSparseMat(jac, jtmp, nv, flg_rot ? 6 : 3, weight[i],
                              NV, bodyNV, chain, bodychain, buf, buf_ind);
    }
  }

  // dense
  else {
    // set first
    mj_jac(m, d, jacp, jacr, point, body[0]);
    mju_scl(jac, jac, weight[0], flg_rot ? 6*nv : 3*nv);

    // accumulate remaining
    for (int i=1; i < n; i++) {
      mj_jac(m, d, jp, jr, point, body[i]);
      mju_addToScl(jac, jtmp, weight[i], flg_rot ? 6*nv : 3*nv);
    }

    NV = nv;
  }

  mj_freeStack(d);

  return NV;
}



// compute subtree angular momentum matrix
void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body) {
  int nv = m->nv;
  mj_markStack(d);

  // stack allocations
  mjtNum* jacp = mj_stackAllocNum(d, 3*nv);
  mjtNum* jacr = mj_stackAllocNum(d, 3*nv);
  mjtNum* term1 = mj_stackAllocNum(d, 3*nv);
  mjtNum* term2 = mj_stackAllocNum(d, 3*nv);

  // clear output
  mju_zero(mat, 3*nv);

  // save the location of the subtree COM
  mjtNum subtree_com[3];
  mju_copy3(subtree_com, d->subtree_com+3*body);

  for (int b=body; b < m->nbody; b++) {
    // end of body subtree, break from the loop
    if (b > body && m->body_parentid[b] < body) {
      break;
    }

    // linear and angular velocity Jacobian of the body COM (inertial frame)
    mj_jacBodyCom(m, d, jacp, jacr, b);

    // orientation of the COM (inertial) frame of b-th body
    mjtNum ximat[9];
    mju_copy(ximat, d->ximat+9*b, 9);

    // save the inertia matrix of b-th body
    mjtNum inertia[9] = {0};
    inertia[0] = m->body_inertia[3*b];   // inertia(1,1)
    inertia[4] = m->body_inertia[3*b+1]; // inertia(2,2)
    inertia[8] = m->body_inertia[3*b+2]; // inertia(3,3)

    // term1 = body angular momentum about self COM in world frame
    mjtNum tmp1[9], tmp2[9];
    mju_mulMatMat3(tmp1, ximat, inertia);          // tmp1  = ximat * inertia
    mju_mulMatMatT3(tmp2, tmp1, ximat);            // tmp2  = ximat * inertia * ximat^T
    mju_mulMatMat(term1, tmp2, jacr, 3, 3, nv);    // term1 = ximat * inertia * ximat^T * jacr

    // location of body COM w.r.t subtree COM
    mjtNum com[3];
    mju_sub3(com, d->xipos+3*b, subtree_com);

    // skew symmetric matrix representing body_com vector
    mjtNum com_mat[9] = {0};
    com_mat[1] = -com[2];
    com_mat[2] = com[1];
    com_mat[3] = com[2];
    com_mat[5] = -com[0];
    com_mat[6] = -com[1];
    com_mat[7] = com[0];

    // term2 = moment of linear momentum
    mju_mulMatMat(term2, com_mat, jacp, 3, 3, nv);   // term2 = com_mat * jacp
    mju_scl(term2, term2, m->body_mass[b], 3 * nv);  // term2 = com_mat * jacp * mass

    // mat += term1 + term2
    mju_addTo(mat, term1, 3*nv);
    mju_addTo(mat, term2, 3*nv);
  }

  mj_freeStack(d);
}



//-------------------------- name functions --------------------------------------------------------

// get number of objects and name addresses for given object type
static int _getnumadr(const mjModel* m, mjtObj type, int** padr, int* mapadr) {
  int num = -1;
  // map address starts at the end, subtract with explicit switch fallthrough below
  *mapadr = m->nnames_map;

  // get address list and size for object type
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    *mapadr -= mjLOAD_MULTIPLE*m->nbody;
    *padr = m->name_bodyadr;
    num = m->nbody;
    mjFALLTHROUGH;

  case mjOBJ_JOINT:
    *mapadr -= mjLOAD_MULTIPLE*m->njnt;
    if (num < 0) {
      *padr = m->name_jntadr;
      num = m->njnt;
    }
    mjFALLTHROUGH;

  case mjOBJ_GEOM:
    *mapadr -= mjLOAD_MULTIPLE*m->ngeom;
    if (num < 0) {
      *padr = m->name_geomadr;
      num = m->ngeom;
    }
    mjFALLTHROUGH;

  case mjOBJ_SITE:
    *mapadr -= mjLOAD_MULTIPLE*m->nsite;
    if (num < 0) {
      *padr = m->name_siteadr;
      num = m->nsite;
    }
    mjFALLTHROUGH;

  case mjOBJ_CAMERA:
    *mapadr -= mjLOAD_MULTIPLE*m->ncam;
    if (num < 0) {
      *padr = m->name_camadr;
      num = m->ncam;
    }
    mjFALLTHROUGH;

  case mjOBJ_LIGHT:
    *mapadr -= mjLOAD_MULTIPLE*m->nlight;
    if (num < 0) {
      *padr = m->name_lightadr;
      num = m->nlight;
    }
    mjFALLTHROUGH;

  case mjOBJ_FLEX:
    *mapadr -= mjLOAD_MULTIPLE*m->nflex;
    if (num < 0) {
      *padr = m->name_flexadr;
      num =  m->nflex;
    }
    mjFALLTHROUGH;

  case mjOBJ_MESH:
    *mapadr -= mjLOAD_MULTIPLE*m->nmesh;
    if (num < 0) {
      *padr = m->name_meshadr;
      num =  m->nmesh;
    }
    mjFALLTHROUGH;

  case mjOBJ_SKIN:
    *mapadr -= mjLOAD_MULTIPLE*m->nskin;
    if (num < 0) {
      *padr = m->name_skinadr;
      num = m->nskin;
    }
    mjFALLTHROUGH;

  case mjOBJ_HFIELD:
    *mapadr -= mjLOAD_MULTIPLE*m->nhfield;
    if (num < 0) {
      *padr = m->name_hfieldadr;
      num = m->nhfield;
    }
    mjFALLTHROUGH;

  case mjOBJ_TEXTURE:
    *mapadr -= mjLOAD_MULTIPLE*m->ntex;
    if (num < 0) {
      *padr = m->name_texadr;
      num = m->ntex;
    }
    mjFALLTHROUGH;

  case mjOBJ_MATERIAL:
    *mapadr -= mjLOAD_MULTIPLE*m->nmat;
    if (num < 0) {
      *padr = m->name_matadr;
      num = m->nmat;
    }
    mjFALLTHROUGH;

  case mjOBJ_PAIR:
    *mapadr -= mjLOAD_MULTIPLE*m->npair;
    if (num < 0) {
      *padr = m->name_pairadr;
      num = m->npair;
    }
    mjFALLTHROUGH;

  case mjOBJ_EXCLUDE:
    *mapadr -= mjLOAD_MULTIPLE*m->nexclude;
    if (num < 0) {
      *padr = m->name_excludeadr;
      num = m->nexclude;
    }
    mjFALLTHROUGH;

  case mjOBJ_EQUALITY:
    *mapadr -= mjLOAD_MULTIPLE*m->neq;
    if (num < 0) {
      *padr = m->name_eqadr;
      num = m->neq;
    }
    mjFALLTHROUGH;

  case mjOBJ_TENDON:
    *mapadr -= mjLOAD_MULTIPLE*m->ntendon;
    if (num < 0) {
      *padr = m->name_tendonadr;
      num = m->ntendon;
    }
    mjFALLTHROUGH;

  case mjOBJ_ACTUATOR:
    *mapadr -= mjLOAD_MULTIPLE*m->nu;
    if (num < 0) {
      *padr = m->name_actuatoradr;
      num = m->nu;
    }
    mjFALLTHROUGH;

  case mjOBJ_SENSOR:
    *mapadr -= mjLOAD_MULTIPLE*m->nsensor;
    if (num < 0) {
      *padr = m->name_sensoradr;
      num = m->nsensor;
    }
    mjFALLTHROUGH;

  case mjOBJ_NUMERIC:
    *mapadr -= mjLOAD_MULTIPLE*m->nnumeric;
    if (num < 0) {
      *padr = m->name_numericadr;
      num = m->nnumeric;
    }
    mjFALLTHROUGH;

  case mjOBJ_TEXT:
    *mapadr -= mjLOAD_MULTIPLE*m->ntext;
    if (num < 0) {
      *padr = m->name_textadr;
      num = m->ntext;
    }
    mjFALLTHROUGH;

  case mjOBJ_TUPLE:
    *mapadr -= mjLOAD_MULTIPLE*m->ntuple;
    if (num < 0) {
      *padr = m->name_tupleadr;
      num = m->ntuple;
    }
    mjFALLTHROUGH;

  case mjOBJ_KEY:
    *mapadr -= mjLOAD_MULTIPLE*m->nkey;
    if (num < 0) {
      *padr = m->name_keyadr;
      num = m->nkey;
    }
    mjFALLTHROUGH;

  case mjOBJ_PLUGIN:
    *mapadr -= mjLOAD_MULTIPLE*m->nplugin;
    if (num < 0) {
      *padr = m->name_pluginadr;
      num = m->nplugin;
    }
    mjFALLTHROUGH;

  default:
    if (num < 0) {
      *padr = 0;
      num = 0;
    }
  }

  return num;
}

// get string hash, see http://www.cse.yorku.ca/~oz/hash.html
uint64_t mj_hashString(const char* s, uint64_t n) {
  uint64_t h = 5381;
  int c;
  while ((c = *s++)) {
    h = ((h << 5) + h) ^ c;
  }
  return h % n;
}

// get id of object with the specified mjtObj type and name,
// returns -1 if id not found
int mj_name2id(const mjModel* m, int type, const char* name) {
  int mapadr;
  int* adr = 0;

  // get number of objects and name addresses
  int num = mjLOAD_MULTIPLE*_getnumadr(m, type, &adr, &mapadr);

  // search
  if (num) {    // look up at hash address
    uint64_t hash = mj_hashString(name, num);
    uint64_t i = hash;

    do {
      int j = m->names_map[mapadr + i];
      if (j < 0) {
        return -1;
      }

      if (!strncmp(name, m->names+adr[j], m->nnames-adr[j])) {
        return j;
      }
      if ((++i) == num)i = 0;
    } while (i != hash);
  }
  return -1;
}



// get name of object with the specified mjtObj type and id,
// returns NULL if name not found
const char* mj_id2name(const mjModel* m, int type, int id) {
  int mapadr;
  int* adr = 0;

  // get number of objects and name addresses
  int num = _getnumadr(m, type, &adr, &mapadr);

  // id is in [0, num) and the found name is not the empty string "\0"
  if (id >= 0 && id < num && m->names[adr[id]]) {
    return m->names+adr[id];
  }

  return NULL;
}



//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M) {
  int adr = 0, nv = m->nv;
  mju_zero(dst, nv*nv);

  for (int i=0; i < nv; i++) {
    int j = i;
    while (j >= 0) {
      dst[i*nv+j] = M[adr];
      dst[j*nv+i] = M[adr];
      j = m->dof_parentid[j];
      adr++;
    }
  }
}



// multiply vector by inertia matrix
void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  int nv = m->nv;
  const mjtNum* M = d->qM;
  const int* Madr = m->dof_Madr;
  const int* parentid = m->dof_parentid;
  const int* simplenum = m->dof_simplenum;

  mju_zero(res, nv);

  for (int i=0; i < nv; i++) {
#ifdef mjUSEAVX
    // simple: diagonal multiplication, AVX
    if (simplenum[i] >= 4) {
      // init
      __m256d result, val1, val2;

      // parallel computation
      val1 = _mm256_loadu_pd(vec+i);
      val2 = _mm256_set_pd(M[Madr[i+3]],
                           M[Madr[i+2]],
                           M[Madr[i+1]],
                           M[Madr[i+0]]);
      result = _mm256_mul_pd(val1, val2);

      // store result
      _mm256_storeu_pd(res+i, result);

      // skip rest of block
      i += 3;
      continue;
    }
#endif
    // address in M
    int adr = Madr[i];

    // compute diagonal
    res[i] = M[adr]*vec[i];

    // simple dof: continue
    if (simplenum[i]) {
      continue;
    }

    // compute off-diagonals
    int j = parentid[i];
    while (j >= 0) {
      adr++;
      res[i] += M[adr]*vec[j];
      res[j] += M[adr]*vec[i];

      // advance to parent
      j = parentid[j];
    }
  }
}



// multiply vector by inertia matrix for one dof island
void mj_mulM_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
                    int island, int flg_vecunc) {
  // if no island, call regular function
  if (island < 0) {
    mj_mulM(m, d, res, vec);
    return;
  }

  // local constants: general
  const mjtNum* M = d->qM;
  const int* Madr = m->dof_Madr;
  const int* parentid = m->dof_parentid;
  const int* simplenum = m->dof_simplenum;

  // local constants: island specific
  int ndof = d->island_dofnum[island];
  const int* dofind = d->island_dofind + d->island_dofadr[island];
  const int* islandind = d->dof_islandind;

  mju_zero(res, ndof);

  for (int k=0; k < ndof; k++) {
    // address in full dof vector
    int i = dofind[k];

    // address in M
    int adr = Madr[i];

    // diagonal
    if (flg_vecunc) {
      res[k] = M[adr]*vec[i];
    } else {
      res[k] = M[adr]*vec[k];
    }

    // simple dof: continue
    if (simplenum[i]) {
      continue;
    }

    // off-diagonal
    int j = parentid[i];
    while (j >= 0) {
      adr++;
      int l = islandind[j];
      if (flg_vecunc) {
        res[k] += M[adr]*vec[j];
        res[l] += M[adr]*vec[i];
      } else {
        res[k] += M[adr]*vec[l];
        res[l] += M[adr]*vec[k];
      }

      // advance to parent
      j = parentid[j];
    }
  }
}



// multiply vector by M^(1/2)
void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  int adr, nv = m->nv;
  const mjtNum* qLD = d->qLD;
  const mjtNum* qLDiagSqrtInv = d->qLDiagSqrtInv;
  const int* dofMadr = m->dof_Madr;

  mju_zero(res, nv);

  for (int i=0; i < nv; i++) {
#ifdef mjUSEAVX
    // simple: diagonal division, AVX
    if (m->dof_simplenum[i] >= 4) {
      // init
      __m256d result, val1, val2;

      // parallel computation
      val1 = _mm256_loadu_pd(vec+i);
      val2 = _mm256_set_pd(qLDiagSqrtInv[dofMadr[i+3]],
                           qLDiagSqrtInv[dofMadr[i+2]],
                           qLDiagSqrtInv[dofMadr[i+1]],
                           qLDiagSqrtInv[dofMadr[i+0]]);
      result = _mm256_div_pd(val1, val2);

      // store result
      _mm256_storeu_pd(res+i, result);

      // skip rest of block
      i += 3;
      continue;
    }
#endif

    // simple: diagonal division
    if (m->dof_simplenum[i]) {
      res[i] = vec[i]/qLDiagSqrtInv[i];
    }

    // regular: full multiplication
    else {
      // diagonal
      adr = dofMadr[i];
      res[i] += vec[i]/qLDiagSqrtInv[i];

      // off-diagonal
      int j = m->dof_parentid[i];
      adr++;
      while (j >= 0) {
        res[i] += qLD[adr]*vec[j];

        // advance to next element
        j = m->dof_parentid[j];
        adr++;
      }
    }
  }
}



// add inertia matrix to destination matrix
//  destination can be sparse uncompressed, or dense when all int* are NULL
void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
             int* rownnz, int* rowadr, int* colind) {
  // sparse
  if (rownnz && rowadr && colind) {
    int nv = m->nv;
    mj_markStack(d);
    // create sparse inertia matrix M
    int nnz = m->nD;  // use sparse dof-dof matrix
    int* M_rownnz = mj_stackAllocInt(d, nv);  // actual nnz count
    int* M_colind = mj_stackAllocInt(d, nnz);
    mjtNum* M = mj_stackAllocNum(d, nnz);

    mj_makeMSparse(m, d, M, M_rownnz, NULL, M_colind);
    mj_addMSparse(m, d, dst, rownnz, rowadr, colind, M,
                  M_rownnz, NULL, M_colind);
    mj_freeStack(d);
  }

  // dense
  else {
    mj_addMDense(m, d, dst);
  }
}



// make inertia matrix M
void mj_makeMSparse(const mjModel* m, mjData* d, mjtNum* M,
                    int* M_rownnz, int* M_rowadr, int* M_colind) {
  int nv = m->nv;
  // currently the sparse dof-dof matrix D row addresses are used, since D has
  // the same predetermined sparsity structure as M, however with simple bodies
  // M has less non-zeros and can be precounted for further memory reduction
  if (M_rowadr == NULL) {
    M_rowadr = d->D_rowadr;
  }

  // build M into sparse format, lower triangle
  for (int i = 0; i < nv; i++) {
    int Madr = m->dof_Madr[i];

    // simple, fill diagonal only
    if (m->dof_simplenum[i]) {
      M_rownnz[i] = 1;
      M[M_rowadr[i]] = d->qM[Madr];
      M_colind[M_rowadr[i]] = i;
      continue;
    }

    // backward pass over dofs: construct M_row(i) in reverse order
    int col = M_rowadr[i];  // current column in row i
    for (int j = i; j >= 0; j = m->dof_parentid[j]) {
      M[col] = d->qM[Madr++];
      M_colind[col++] = j;
    }

    // track nnz of lower triangle for row i
    int nnz = M_rownnz[i] = col - M_rowadr[i];

    // reverse order
    int end = nnz >> 1;
    for (int j = 0; j < end; j++) {
      int a1 = M_rowadr[i] + j;              // address 1
      int a2 = (M_rowadr[i] + nnz - 1) - j;  // address 2

      // swap M data on row i
      mjtNum val = M[a1];
      M[a1] = M[a2];
      M[a2] = val;

      // swap M column indices on row i
      int ind = M_colind[a1];
      M_colind[a1] = M_colind[a2];
      M_colind[a2] = ind;
    }
  }

  // fill upper triangle
  for (int i = 1; i < nv; i++) {
    int end = M_rowadr[i] + M_rownnz[i] - 1;
    for (int j = M_rowadr[i]; j < end; j++) {
      int a = M_rowadr[M_colind[j]] + M_rownnz[M_colind[j]]++;
      M[a] = M[j];
      M_colind[a] = i;
    }
  }
}



// add inertia matrix to sparse destination matrix
void mj_addMSparse(const mjModel* m, mjData* d, mjtNum* dst,
                   int* rownnz, int* rowadr, int* colind, mjtNum* M,
                   int* M_rownnz, int* M_rowadr, int* M_colind) {
  int nv = m->nv;
  // currently the sparse dof-dof matrix D row addresses are used, since D has
  // the same predetermined sparsity structure as M, however with simple bodies
  // M has less non-zeros and can be precounted for further memory reduction
  if (M_rowadr == NULL) {
    M_rowadr = d->D_rowadr;
  }

  mj_markStack(d);
  int* buf_ind = mj_stackAllocInt(d, nv);
  mjtNum* sparse_buf = mj_stackAllocNum(d, nv);

  // add to destination
  for (int i=0; i < nv; i++) {
    rownnz[i] = mju_combineSparse(dst + rowadr[i], M + M_rowadr[i], 1, 1,
                                  rownnz[i], M_rownnz[i], colind + rowadr[i],
                                  M_colind + M_rowadr[i], sparse_buf, buf_ind);
  }
  mj_freeStack(d);
}



// add inertia matrix to dense destination matrix
void mj_addMDense(const mjModel* m, mjData* d, mjtNum* dst) {
  int nv = m->nv;

  for (int i = 0; i < nv; i++) {
    int adr = m->dof_Madr[i];
    int j = i;
    while (j >= 0) {
      // add
      dst[i*nv+j] += d->qM[adr];
      if (j < i) {
        dst[j*nv+i] += d->qM[adr];
      }

      // only diagonal if simplenum
      if (m->dof_simplenum[i]) {
        break;
      }

      // advance
      j = m->dof_parentid[j];
      adr++;
    }
  }
}


//-------------------------- sparse system matrix conversion ---------------------------------------

// dst[D] = src[M], handle different sparsity representations
void mj_copyM2DSparse(const mjModel* m, mjData* d, mjtNum* dst, const mjtNum* src) {
  int nv = m->nv;
  mj_markStack(d);

  // init remaining
  int* remaining = mj_stackAllocInt(d, nv);
  mju_copyInt(remaining, d->D_rownnz, nv);

  // copy data
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    int adr = m->dof_Madr[i];
    remaining[i]--;
    dst[d->D_rowadr[i] + remaining[i]] = src[adr];
    adr++;

    // process below diagonal
    int j = i;
    while ((j = m->dof_parentid[j]) >= 0) {
      remaining[i]--;
      dst[d->D_rowadr[i] + remaining[i]] = src[adr];

      remaining[j]--;
      dst[d->D_rowadr[j] + remaining[j]] = src[adr];

      adr++;
    }
  }

  mj_freeStack(d);
}



// dst[M] = src[D lower], handle different sparsity representations
void mj_copyD2MSparse(const mjModel* m, mjData* d, mjtNum* dst, const mjtNum* src) {
  int nv = m->nv;

  // copy data
  for (int i = nv - 1; i >= 0; i--) {
    // find diagonal in qDeriv
    int j = 0;
    while (d->D_colind[d->D_rowadr[i] + j] < i) {
      j++;
    }

    // copy
    int adr = m->dof_Madr[i];
    while (j >= 0) {
      dst[adr] = src[d->D_rowadr[i] + j];
      adr++;
      j--;
    }
  }
}



//-------------------------- perturbations ---------------------------------------------------------

// add Cartesian force and torque to qfrc_target
void mj_applyFT(const mjModel* m, mjData* d,
                const mjtNum force[3], const mjtNum torque[3],
                const mjtNum point[3], int body, mjtNum* qfrc_target) {
  int nv = m->nv;

  // allocate local variables
  mj_markStack(d);
  mjtNum* jacp = force ? mj_stackAllocNum(d, 3*nv) : NULL;
  mjtNum* jacr = torque ? mj_stackAllocNum(d, 3*nv) : NULL;
  mjtNum* qforce = mj_stackAllocNum(d, nv);

  // make sure body is in range
  if (body < 0 || body >= m->nbody) {
    mjERROR("invalid body %d", body);
  }

  // sparse case
  if (mj_isSparse(m)) {
    // construct chain and sparse Jacobians
    int* chain = mj_stackAllocInt(d, nv);
    int NV = mj_bodyChain(m, body, chain);
    mj_jacSparse(m, d, jacp, jacr, point, body, NV, chain);

    // compute J'*f and accumulate
    if (force) {
      mju_mulMatTVec(qforce, jacp, force, 3, NV);
      for (int i=0; i < NV; i++) {
        qfrc_target[chain[i]] += qforce[i];
      }
    }
    if (torque) {
      mju_mulMatTVec(qforce, jacr, torque, 3, NV);
      for (int i=0; i < NV; i++) {
        qfrc_target[chain[i]] += qforce[i];
      }
    }
  }

  // dense case
  else {
    // compute Jacobians
    mj_jac(m, d, jacp, jacr, point, body);

    // compute J'*f and accumulate
    if (force) {
      mju_mulMatTVec(qforce, jacp, force, 3, nv);
      mju_addTo(qfrc_target, qforce, nv);
    }
    if (torque) {
      mju_mulMatTVec(qforce, jacr, torque, 3, nv);
      mju_addTo(qfrc_target, qforce, nv);
    }
  }

  mj_freeStack(d);
}



// accumulate xfrc_applied in qfrc
void mj_xfrcAccumulate(const mjModel* m, mjData* d, mjtNum* qfrc) {
  for (int i=1; i < m->nbody; i++) {
    if (!mju_isZero(d->xfrc_applied+6*i, 6)) {
      mj_applyFT(m, d, d->xfrc_applied+6*i, d->xfrc_applied+6*i+3, d->xipos+3*i, i, qfrc);
    }
  }
}



// compute object 6D velocity in object-centered frame, world/local orientation
void mj_objectVelocity(const mjModel* m, const mjData* d,
                       int objtype, int objid, mjtNum res[6], int flg_local) {
  int bodyid = 0;
  const mjtNum *pos = 0, *rot = 0;

  // body-inertial
  if (objtype == mjOBJ_BODY) {
    bodyid = objid;
    pos = d->xipos+3*objid;
    rot = (flg_local ? d->ximat+9*objid : 0);
  }

  // body-regular
  else if (objtype == mjOBJ_XBODY) {
    bodyid = objid;
    pos = d->xpos+3*objid;
    rot = (flg_local ? d->xmat+9*objid : 0);
  }

  // geom
  else if (objtype == mjOBJ_GEOM) {
    bodyid = m->geom_bodyid[objid];
    pos = d->geom_xpos+3*objid;
    rot = (flg_local ? d->geom_xmat+9*objid : 0);
  }

  // site
  else if (objtype == mjOBJ_SITE) {
    bodyid = m->site_bodyid[objid];
    pos = d->site_xpos+3*objid;
    rot = (flg_local ? d->site_xmat+9*objid : 0);
  }

  // camera
  else if (objtype == mjOBJ_CAMERA) {
    bodyid = m->cam_bodyid[objid];
    pos = d->cam_xpos+3*objid;
    rot = (flg_local ? d->cam_xmat+9*objid : 0);
  }

  // object without spatial frame
  else {
    mjERROR("invalid object type %d", objtype);
  }

  // transform velocity
  mju_transformSpatial(res, d->cvel+6*bodyid, 0, pos, d->subtree_com+3*m->body_rootid[bodyid], rot);
}



// compute object 6D acceleration in object-centered frame, world/local orientation
void mj_objectAcceleration(const mjModel* m, const mjData* d,
                           int objtype, int objid, mjtNum res[6], int flg_local) {
  int bodyid = 0;
  const mjtNum *pos = 0, *rot = 0;
  mjtNum correction[3], vel[6];

  // body-inertial
  if (objtype == mjOBJ_BODY) {
    bodyid = objid;
    pos = d->xipos+3*objid;
    rot = (flg_local ? d->ximat+9*objid : 0);
  }

  // body-regular
  else if (objtype == mjOBJ_XBODY) {
    bodyid = objid;
    pos = d->xpos+3*objid;
    rot = (flg_local ? d->xmat+9*objid : 0);
  }

  // geom
  else if (objtype == mjOBJ_GEOM) {
    bodyid = m->geom_bodyid[objid];
    pos = d->geom_xpos+3*objid;
    rot = (flg_local ? d->geom_xmat+9*objid : 0);
  }

  // site
  else if (objtype == mjOBJ_SITE) {
    bodyid = m->site_bodyid[objid];
    pos = d->site_xpos+3*objid;
    rot = (flg_local ? d->site_xmat+9*objid : 0);
  }

  // camera
  else if (objtype == mjOBJ_CAMERA) {
    bodyid = m->cam_bodyid[objid];
    pos = d->cam_xpos+3*objid;
    rot = (flg_local ? d->cam_xmat+9*objid : 0);
  }

  // object without spatial frame
  else {
    mjERROR("invalid object type %d", objtype);
  }

  // transform com-based velocity to local frame
  mju_transformSpatial(vel, d->cvel+6*bodyid, 0, pos, d->subtree_com+3*m->body_rootid[bodyid], rot);

  // transform com-based acceleration to local frame
  mju_transformSpatial(res, d->cacc+6*bodyid, 0, pos, d->subtree_com+3*m->body_rootid[bodyid], rot);

  // acc_tran += vel_rot x vel_tran
  mju_cross(correction, vel, vel+3);
  mju_addTo3(res+3, correction);
}



//-------------------------- miscellaneous ---------------------------------------------------------

// returns the smallest distance between two geoms
mjtNum mj_geomDistance(const mjModel* m, const mjData* d, int geom1, int geom2, mjtNum distmax,
                       mjtNum fromto[6]) {
  mjContact con[mjMAXCONPAIR];
  mjtNum dist = distmax;
  if (fromto) mju_zero(fromto, 6);

  // flip geom order if required
  int flip = m->geom_type[geom1] > m->geom_type[geom2];
  int g1 = flip ? geom2 : geom1;
  int g2 = flip ? geom1 : geom2;
  int type1 = m->geom_type[g1];
  int type2 = m->geom_type[g2];

  // call collision function if it exists
  if (!mjCOLLISIONFUNC[type1][type2]) {
    return dist;
  }
  int num = mjCOLLISIONFUNC[type1][type2](m, d, con, g1, g2, distmax);

  // find smallest distance
  int smallest = -1;
  for (int i=0; i < num; i++) {
    mjtNum dist_i = con[i].dist;
    if (dist_i < dist) {
      dist = dist_i;
      smallest = i;
    }
  }

  // write fromto if given and a collision has been found
  if (fromto && smallest >= 0) {
    mjtNum sign = flip ? -1 : 1;
    mju_addScl3(fromto+0, con[smallest].pos, con[smallest].frame, -0.5*sign*dist);
    mju_addScl3(fromto+3, con[smallest].pos, con[smallest].frame, 0.5*sign*dist);
  }

  return dist;
}



// extract 6D force:torque for one contact, in contact frame
void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]) {
  mjContact* con;

  // clear result
  mju_zero(result, 6);

  // make sure contact is valid
  if (id >= 0 && id < d->ncon && d->contact[id].efc_address >= 0) {
    // get contact pointer
    con = d->contact + id;

    if (mj_isPyramidal(m)) {
      mju_decodePyramid(result, d->efc_force + con->efc_address, con->friction, con->dim);
    } else {
      mju_copy(result, d->efc_force + con->efc_address, con->dim);
    }
  }
}



// compute velocity by finite-differencing two positions
void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                         const mjtNum* qpos1, const mjtNum* qpos2) {
  // loop over joints
  for (int j=0; j < m->njnt; j++) {
    // get addresses in qpos and qvel
    int padr = m->jnt_qposadr[j];
    int vadr = m->jnt_dofadr[j];

    switch ((mjtJoint) m->jnt_type[j]) {
    case mjJNT_FREE:
      for (int i=0; i < 3; i++) {
        qvel[vadr+i] = (qpos2[padr+i] - qpos1[padr+i]) / dt;
      }
      vadr += 3;
      padr += 3;

      // continute with rotations
      mjFALLTHROUGH;

    case mjJNT_BALL:
      // solve:  qpos1 * quat(qvel * dt) = qpos2
      mju_subQuat(qvel+vadr, qpos2+padr, qpos1+padr);
      mju_scl3(qvel+vadr, qvel+vadr, 1/dt);
      break;

    case mjJNT_HINGE:
    case mjJNT_SLIDE:
      qvel[vadr] = (qpos2[padr] - qpos1[padr]) / dt;
    }
  }
}



// integrate qpos with given qvel
void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt) {
  // loop over joints
  for (int j=0; j < m->njnt; j++) {
    // get addresses in qpos and qvel
    int padr = m->jnt_qposadr[j];
    int vadr = m->jnt_dofadr[j];

    switch ((mjtJoint) m->jnt_type[j]) {
    case mjJNT_FREE:
      // position update
      for (int i=0; i < 3; i++) {
        qpos[padr+i] += dt * qvel[vadr+i];
      }
      padr += 3;
      vadr += 3;

      // continue with rotation update
      mjFALLTHROUGH;

    case mjJNT_BALL:
      // quaternion update
      mju_quatIntegrate(qpos+padr, qvel+vadr, dt);
      break;

    case mjJNT_HINGE:
    case mjJNT_SLIDE:
      // scalar update: same for rotation and translation
      qpos[padr] += dt * qvel[vadr];
    }
  }
}



// normalize all quaternions in qpos-type vector
void mj_normalizeQuat(const mjModel* m, mjtNum* qpos) {
  // find quaternion fields and normalize
  for (int i=0; i < m->njnt; i++) {
    if (m->jnt_type[i] == mjJNT_BALL || m->jnt_type[i] == mjJNT_FREE) {
      mju_normalize4(qpos+m->jnt_qposadr[i]+3*(m->jnt_type[i] == mjJNT_FREE));
    }
  }
}



// map from body local to global Cartesian coordinates
void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9],
                     const mjtNum pos[3], const mjtNum quat[4],
                     int body, mjtByte sameframe) {
  mjtNum tmp[4];

  // position
  if (xpos && pos) {
    // compute
    if (sameframe == 0) {
      mju_rotVecMat(xpos, pos, d->xmat+9*body);
      mju_addTo3(xpos, d->xpos+3*body);
    }

    // copy body position
    else if (sameframe == 1) {
      mju_copy3(xpos, d->xpos+3*body);
    }

    // copy inertial body position
    else {
      mju_copy3(xpos, d->xipos+3*body);
    }
  }

  // orientation
  if (xmat && quat) {
    // compute
    if (sameframe == 0) {
      mju_mulQuat(tmp, d->xquat+4*body, quat);
      mju_quat2Mat(xmat, tmp);
    }

    // copy body orientation
    else if (sameframe == 1) {
      mju_copy(xmat, d->xmat+9*body, 9);
    }

    // copy inertial body orientation
    else {
      mju_copy(xmat, d->ximat+9*body, 9);
    }
  }
}

// return 1 if actuator i is disabled, 0 otherwise
int mj_actuatorDisabled(const mjModel* m, int i) {
  int group = m->actuator_group[i];
  if (group < 0 || group > 30) {
    return 0;
  } else {
    return m->opt.disableactuator & (1 << group) ? 1 : 0;
  }
}

// sum all body masses
mjtNum mj_getTotalmass(const mjModel* m) {
  mjtNum res = 0;

  for (int i=1; i < m->nbody; i++) {
    res += m->body_mass[i];
  }

  return res;
}



// scale all body masses and inertias to achieve specified total mass
void mj_setTotalmass(mjModel* m, mjtNum newmass) {
  // compute scale factor, avoid zeros
  mjtNum scale = mju_max(mjMINVAL, newmass / mju_max(mjMINVAL, mj_getTotalmass(m)));

  // scale all masses and inertias
  for (int i=1; i < m->nbody; i++) {
    m->body_mass[i] *= scale;
    m->body_inertia[3*i] *= scale;
    m->body_inertia[3*i+1] *= scale;
    m->body_inertia[3*i+2] *= scale;
  }

  // don't forget to call mj_set0 after changing masses
}



// count warnings, print only the first time
void mj_warning(mjData* d, int warning, int info) {
  // check type
  if (warning < 0 || warning >= mjNWARNING) {
    mjERROR("invalid warning type %d", warning);
  }

  // save info (override previous)
  d->warning[warning].lastinfo = info;

  // print message only the first time this warning is encountered
  if (!d->warning[warning].number) {
    mju_warning("%s Time = %.4f.", mju_warningText(warning, info), d->time);
  }

  // increase counter
  d->warning[warning].number++;
}



// version number
int mj_version(void) {
  return mjVERSION;
}



// current version of MuJoCo as a null-terminated string
const char* mj_versionString(void) {
  static const char versionstring[] = mjVERSIONSTRING;
  return versionstring;
}
