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

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_driver.h"
#include "engine/engine_collision_gjk.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

#ifdef mjUSEPLATFORMSIMD
  #if defined(__AVX__) && !defined(mjUSESINGLE)
    #define mjUSEAVX
    #include "immintrin.h"
  #endif
#endif

//-------------------------- Constants -------------------------------------------------------------

 #define mjVERSION 334
#define mjVERSIONSTRING "3.3.4"

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
  "Eulerdamp",
  "AutoReset",
  "NativeCCD"
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



// copy current state to the k-th model keyframe
void mj_setKeyframe(mjModel* m, const mjData* d, int k) {
  // check keyframe index
  if (k >= m->nkey) {
    mjERROR("index must be smaller than %d (keyframes allocated in model)", m->nkey);
  }
  if (k < 0) {
    mjERROR("keyframe index cannot be negative");
  }

  // copy state to model keyframe
  m->key_time[k] = d->time;
  mju_copy(m->key_qpos + k*m->nq, d->qpos, m->nq);
  mju_copy(m->key_qvel + k*m->nv, d->qvel, m->nv);
  mju_copy(m->key_act + k*m->na, d->act, m->na);
  mju_copy(m->key_mpos + k*3*m->nmocap, d->mocap_pos, 3*m->nmocap);
  mju_copy(m->key_mquat + k*4*m->nmocap, d->mocap_quat, 4*m->nmocap);
  mju_copy(m->key_ctrl + k*m->nu, d->ctrl, m->nu);
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

  // initialize last dof address for each body
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

    // initialize last dof
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
  int nv = m->nv;
  mjtNum offset[3];

  // clear jacobians, compute offset if required
  if (jacp) {
    mju_zero(jacp, 3*nv);
    mju_sub3(offset, point, d->subtree_com+3*m->body_rootid[body]);
  }
  if (jacr) {
    mju_zero(jacr, 3*nv);
  }

  // skip fixed bodies
  while (body && !m->body_dofnum[body]) {
    body = m->body_parentid[body];
  }

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  int i = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // backward pass over dof ancestor chain
  while (i >= 0) {
    mjtNum* cdof = d->cdof+6*i;

    // construct rotation jacobian
    if (jacr) {
      jacr[i+0*nv] = cdof[0];
      jacr[i+1*nv] = cdof[1];
      jacr[i+2*nv] = cdof[2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      mjtNum tmp[3];
      mju_cross(tmp, cdof, offset);
      jacp[i+0*nv] = cdof[3] + tmp[0];
      jacp[i+1*nv] = cdof[4] + tmp[1];
      jacp[i+2*nv] = cdof[5] + tmp[2];
    }

    // advance to parent dof
    i = m->dof_parentid[i];
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
  mjtNum* jacp_b = mjSTACKALLOC(d, 3*nv, mjtNum);

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
  mjtNum* jacp = (jacPoint ? jacPoint : mjSTACKALLOC(d, 3*nv, mjtNum));
  mjtNum* jacr = mjSTACKALLOC(d, 3*nv, mjtNum);
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
  // clear jacobians
  if (jacp) {
    mju_zero(jacp, 3*NV);
  }
  if (jacr) {
    mju_zero(jacr, 3*NV);
  }

  // compute point-com offset
  mjtNum offset[3];
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
  int da = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // start and the end of the chain (chain is in increasing order)
  int ci = NV-1;

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

    const mjtNum* cdof = d->cdof + 6*da;

    // construct rotation jacobian
    if (jacr) {
      jacr[ci+0*NV] = cdof[0];
      jacr[ci+1*NV] = cdof[1];
      jacr[ci+2*NV] = cdof[2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      mjtNum tmp[3];
      mju_cross(tmp, cdof, offset);

      jacp[ci+0*NV] = cdof[3] + tmp[0];
      jacp[ci+1*NV] = cdof[4] + tmp[1];
      jacp[ci+2*NV] = cdof[5] + tmp[2];
    }

    // advance to parent dof
    da = m->dof_parentid[da];
  }
}



// sparse Jacobian difference for simple body contacts
void mj_jacSparseSimple(const mjModel* m, const mjData* d,
                        mjtNum* jacdifp, mjtNum* jacdifr, const mjtNum* point,
                        int body, int flg_second, int NV, int start) {
  // compute point-com offset
  mjtNum offset[3];
  mju_sub3(offset, point, d->subtree_com+3*m->body_rootid[body]);

  // skip fixed body
  if (!m->body_dofnum[body]) {
    return;
  }

  // process dofs
  int ci = start;
  int end = m->body_dofadr[body] + m->body_dofnum[body];
  for (int da=m->body_dofadr[body]; da < end; da++) {
    mjtNum *cdof = d->cdof+6*da;

    // construct rotation jacobian
    if (jacdifr) {
      // plus sign
      if (flg_second) {
        jacdifr[ci+0*NV] = cdof[0];
        jacdifr[ci+1*NV] = cdof[1];
        jacdifr[ci+2*NV] = cdof[2];
      }

      // minus sign
      else {
        jacdifr[ci+0*NV] = -cdof[0];
        jacdifr[ci+1*NV] = -cdof[1];
        jacdifr[ci+2*NV] = -cdof[2];
      }
    }

    // construct translation jacobian (correct for rotation)
    if (jacdifp) {
      mjtNum tmp[3];
      mju_cross(tmp, cdof, offset);

      // plus sign
      if (flg_second) {
        jacdifp[ci+0*NV] = (cdof[3] + tmp[0]);
        jacdifp[ci+1*NV] = (cdof[4] + tmp[1]);
        jacdifp[ci+2*NV] = (cdof[5] + tmp[2]);
      }

      // plus sign
      else {
        jacdifp[ci+0*NV] = -(cdof[3] + tmp[0]);
        jacdifp[ci+1*NV] = -(cdof[4] + tmp[1]);
        jacdifp[ci+2*NV] = -(cdof[5] + tmp[2]);
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
  mjtNum* jtmp = mjSTACKALLOC(d, flg_rot ? 6*nv : 3*nv, mjtNum);
  mjtNum* jp = jtmp;
  mjtNum* jr = flg_rot ? jtmp + 3*nv : NULL;

  // sparse
  if (mj_isSparse(m)) {
    mjtNum* buf = mjSTACKALLOC(d, flg_rot ? 6*nv : 3*nv, mjtNum);
    int* buf_ind = mjSTACKALLOC(d, nv, int);
    int* bodychain = mjSTACKALLOC(d, nv, int);

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



// compute 3/6-by-nv Jacobian time derivative of global point attached to given body
void mj_jacDot(const mjModel* m, const mjData* d,
               mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body) {
  int nv = m->nv;
  mjtNum offset[3];
  mjtNum pvel[6];  // point velocity (rot:lin order)

  // clear jacobians, compute offset and pvel if required
  if (jacp) {
    mju_zero(jacp, 3*nv);
    const mjtNum* com = d->subtree_com+3*m->body_rootid[body];
    mju_sub3(offset, point, com);
    mju_transformSpatial(pvel, d->cvel+6*body, 0, point, com, 0);
  }
  if (jacr) {
    mju_zero(jacr, 3*nv);
  }

  // skip fixed bodies
  while (body && !m->body_dofnum[body]) {
    body = m->body_parentid[body];
  }

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  int i = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // backward pass over dof ancestor chain
  while (i >= 0) {
    mjtNum cdof_dot[6];
    mju_copy(cdof_dot, d->cdof_dot+6*i, 6);
    mjtNum* cdof = d->cdof+6*i;

    // check for quaternion
    mjtJoint type = m->jnt_type[m->dof_jntid[i]];
    int dofadr = m->jnt_dofadr[m->dof_jntid[i]];
    int is_quat = type == mjJNT_BALL || (type == mjJNT_FREE && i >= dofadr + 3);

    // compute cdof_dot for quaternion (use current body cvel)
    if (is_quat) {
      mju_crossMotion(cdof_dot, d->cvel+6*m->dof_bodyid[i], cdof);
    }

    // construct rotation jacobian
    if (jacr) {
      jacr[i+0*nv] += cdof_dot[0];
      jacr[i+1*nv] += cdof_dot[1];
      jacr[i+2*nv] += cdof_dot[2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      // first correction term, account for varying cdof
      mjtNum tmp1[3];
      mju_cross(tmp1, cdof_dot, offset);

      // second correction term, account for point translational velocity
      mjtNum tmp2[3];
      mju_cross(tmp2, cdof, pvel + 3);

      jacp[i+0*nv] += cdof_dot[3] + tmp1[0] + tmp2[0];
      jacp[i+1*nv] += cdof_dot[4] + tmp1[1] + tmp2[1];
      jacp[i+2*nv] += cdof_dot[5] + tmp1[2] + tmp2[2];
    }

    // advance to parent dof
    i = m->dof_parentid[i];
  }
}



// compute subtree angular momentum matrix
void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body) {
  int nv = m->nv;
  mj_markStack(d);

  // stack allocations
  mjtNum* jacp = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jacr = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* term1 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* term2 = mjSTACKALLOC(d, 3*nv, mjtNum);

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



// multiply vector by inertia matrix (implementation)
void mj_mulM_impl(mjtNum* res, const mjtNum* vec, int nv, const mjtNum* M,
                  const int* Madr, const int* parentid, const int* simplenum) {
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



// multiply vector by inertia matrix
void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  mj_mulM_impl(res, vec, m->nv, d->qM, m->dof_Madr, m->dof_parentid, m->dof_simplenum);
}



// multiply vector by M^(1/2)
void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  int  nv = m->nv;
  const mjtNum* qLD = d->qLD;

  mju_zero(res, nv);

  // res = L * vec
  for (int i=0; i < nv; i++) {
    // diagonal
    res[i] = vec[i];

    // non-simple: add off-diagonals
    if (!m->dof_simplenum[i]) {
      int adr = d->M_rowadr[i];
      res[i] += mju_dotSparse(qLD+adr, vec, d->M_rownnz[i] - 1, d->M_colind+adr);
    }
  }

  // res *= sqrt(D)
  for (int i=0; i < nv; i++) {
    int diag = d->M_rowadr[i] + d->M_rownnz[i] - 1;
    res[i] *= mju_sqrt(qLD[diag]);
  }
}



// add inertia matrix to destination matrix
//  destination can be sparse or dense when all int* are NULL
void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
             int* rownnz, int* rowadr, int* colind) {
  int nv = m->nv;
  // sparse
  if (rownnz && rowadr && colind) {
    mj_markStack(d);
    mjtNum* buf_val = mjSTACKALLOC(d, nv, mjtNum);
    int* buf_ind = mjSTACKALLOC(d, nv, int);

    mju_addToMatSparse(dst, rownnz, rowadr, colind, nv,
      d->M, d->M_rownnz, d->M_rowadr, d->M_colind,
      buf_val, buf_ind);

    mj_freeStack(d);
  }

  // dense
  else {
    mju_addToSymSparse(dst, d->M, nv, d->M_rownnz, d->M_rowadr, d->M_colind, /*flg_upper=*/ 1);
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
  mjtNum* jacp = force ? mjSTACKALLOC(d, 3*nv, mjtNum) : NULL;
  mjtNum* jacr = torque ? mjSTACKALLOC(d, 3*nv, mjtNum) : NULL;
  mjtNum* qforce = mjSTACKALLOC(d, nv, mjtNum);

  // make sure body is in range
  if (body < 0 || body >= m->nbody) {
    mjERROR("invalid body %d", body);
  }

  // sparse case
  if (mj_isSparse(m)) {
    // construct chain and sparse Jacobians
    int* chain = mjSTACKALLOC(d, nv, int);
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

// returns the smallest distance between two geoms (using nativeccd)
static mjtNum mj_geomDistanceCCD(const mjModel* m, const mjData* d, int g1, int g2,
                                 mjtNum distmax, mjtNum fromto[6]) {
  mjCCDConfig config;
  mjCCDStatus status;

  // set config
  config.max_iterations = m->opt.ccd_iterations;
  config.tolerance = m->opt.ccd_tolerance;
  config.max_contacts = 1;        // want contacts
  config.dist_cutoff = distmax;   // want geom distances

  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, m, d, g1, 0);
  mjc_initCCDObj(&obj2, m, d, g2, 0);

  mjtNum dist = mjc_ccd(&config, &status, &obj1, &obj2);

  if (fromto && status.nx > 0) {
    mju_copy3(fromto, status.x1);
    mju_copy3(fromto+3, status.x2);
  }

  return dist;
}



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

  mjfCollision func = mjCOLLISIONFUNC[type1][type2];

  // call collision function if it exists
  if (!func) {
    return dist;
  }

  // use nativecdd if flag is enabled
  if (!mjDISABLED(mjDSBL_NATIVECCD)) {
    if (func == mjc_Convex || func == mjc_BoxBox) {
      return mj_geomDistanceCCD(m, d, g1, g2, distmax, fromto);
    }
  }

  int num = func(m, d, con, g1, g2, distmax);

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
  mjtSameFrame sf = sameframe;

  // position
  if (xpos && pos) {
    switch (sf) {
    case mjSAMEFRAME_NONE:
    case mjSAMEFRAME_BODYROT:
    case mjSAMEFRAME_INERTIAROT:
      mju_mulMatVec3(xpos, d->xmat+9*body, pos);
      mju_addTo3(xpos, d->xpos+3*body);
      break;
    case mjSAMEFRAME_BODY:
      mju_copy3(xpos, d->xpos+3*body);
      break;
    case mjSAMEFRAME_INERTIA:
      mju_copy3(xpos, d->xipos+3*body);
      break;
    }
  }

  // orientation
  if (xmat && quat) {
    mjtNum tmp[4];
    switch (sf) {
    case mjSAMEFRAME_NONE:
      mju_mulQuat(tmp, d->xquat+4*body, quat);
      mju_quat2Mat(xmat, tmp);
      break;
    case mjSAMEFRAME_BODY:
    case mjSAMEFRAME_BODYROT:
      mju_copy(xmat, d->xmat+9*body, 9);
      break;
    case mjSAMEFRAME_INERTIA:
    case mjSAMEFRAME_INERTIAROT:
      mju_copy(xmat, d->ximat+9*body, 9);
      break;
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
