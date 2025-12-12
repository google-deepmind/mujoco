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
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_memory.h"
#include "engine/engine_memory.h"
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

 #define mjVERSION 341
#define mjVERSIONSTRING "3.4.1"

// names of disable flags
const char* mjDISABLESTRING[mjNDISABLE] = {
  "Constraint",
  "Equality",
  "Frictionloss",
  "Limit",
  "Contact",
  "Spring",
  "Damper",
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
  "NativeCCD",
  "Island"
};


// names of enable flags
const char* mjENABLESTRING[mjNENABLE] = {
  "Override",
  "Energy",
  "Fwdinv",
  "InvDiscrete",
  "MultiCCD",
  "Sleep"
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


// size of contact data fields
const int mjCONDATA_SIZE[mjNCONDATA] = {
  1,  // mjCONDATA_FOUND
  3,  // mjCONDATA_FORCE
  3,  // mjCONDATA_TORQUE
  1,  // mjCONDATA_DIST
  3,  // mjCONDATA_POS
  3,  // mjCONDATA_NORMAL
  3   // mjCONDATA_TANGENT
};


//-------------------------- get/set state ---------------------------------------------------------

// return size of a single state element
static inline int mj_stateElemSize(const mjModel* m, mjtState sig) {
  switch (sig) {
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
    mjERROR("invalid state element %u", sig);
    return 0;
  }
}


// return pointer to a single state element
static inline mjtNum* mj_stateElemPtr(const mjModel* m, mjData* d, mjtState sig) {
  switch (sig) {
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
    mjERROR("invalid state element %u", sig);
    return NULL;
  }
}


static inline const mjtNum* mj_stateElemConstPtr(const mjModel* m, const mjData* d, mjtState sig) {
  return mj_stateElemPtr(m, (mjData*) d, sig);  // discard const qualifier from d
}


// get size of state signature
int mj_stateSize(const mjModel* m, int sig) {
  if (sig < 0) {
    mjERROR("invalid state signature %d < 0", sig);
    return 0;
  }

  if (sig >= (1<<mjNSTATE)) {
    mjERROR("invalid state signature %d >= 2^mjNSTATE", sig);
    return 0;
  }

  int size = 0;
  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & sig) {
      size += mj_stateElemSize(m, element);
    }
  }

  return size;
}


// get state
void mj_getState(const mjModel* m, const mjData* d, mjtNum* state, int sig) {
  if (sig < 0) {
    mjERROR("invalid state signature %d < 0", sig);
    return;
  }

  if (sig >= (1<<mjNSTATE)) {
    mjERROR("invalid state signature %d >= 2^mjNSTATE", sig);
    return;
  }

  int adr = 0;
  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & sig) {
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


// extract a sub-state from a state
void mj_extractState(const mjModel* m, const mjtNum* src, int srcsig, mjtNum* dst, int dstsig) {
  if (srcsig < 0) {
    mjERROR("invalid srcsig %d < 0", srcsig);
    return;
  }

  if (srcsig >= (1<<mjNSTATE)) {
    mjERROR("invalid srcsig %d >= 2^mjNSTATE", srcsig);
    return;
  }

  if ((srcsig & dstsig) != dstsig) {
    mjERROR("dstsig is not a subset of srcsig");
    return;
  }

  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & srcsig) {
      int size = mj_stateElemSize(m, element);
      if (element & dstsig) {
        mju_copy(dst, src, size);
        dst += size;
      }
      src += size;
    }
  }
}


// set state
void mj_setState(const mjModel* m, mjData* d, const mjtNum* state, int sig) {
  if (sig < 0) {
    mjERROR("invalid state signature %d < 0", sig);
    return;
  }

  if (sig >= (1<<mjNSTATE)) {
    mjERROR("invalid state signature %d >= 2^mjNSTATE", sig);
    return;
  }

  int adr = 0;
  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & sig) {
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


// copy state from src to dst
void mj_copyState(const mjModel* m, const mjData* src, mjData* dst, int sig) {
  if (sig < 0) {
    mjERROR("invalid state signature %d < 0", sig);
    return;
  }

  if (sig >= (1<<mjNSTATE)) {
    mjERROR("invalid state signature %d >= 2^mjNSTATE", sig);
    return;
  }

  for (int i=0; i < mjNSTATE; i++) {
    mjtState element = 1<<i;
    if (element & sig) {
      int size = mj_stateElemSize(m, element);

      // special handling of eq_active (mjtByte)
      if (element == mjSTATE_EQ_ACTIVE) {
        int neq = m->neq;
        for (int j=0; j < neq; j++) {
          dst->eq_active[j] = src->eq_active[j];
        }
      }

      // regular state components (mjtNum)
      else {
        mjtNum* dst_ptr = mj_stateElemPtr(m, dst, element);
        const mjtNum* src_ptr = mj_stateElemConstPtr(m, src, element);
        mju_copy(dst_ptr, src_ptr, size);
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
  mju_mulSymVecSparse(res, d->M, vec, m->nv, m->M_rownnz, m->M_rowadr, m->M_colind);
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
      int adr = m->M_rowadr[i];
      res[i] += mju_dotSparse(qLD+adr, vec, m->M_rownnz[i] - 1, m->M_colind+adr);
    }
  }

  // res *= sqrt(D)
  for (int i=0; i < nv; i++) {
    int diag = m->M_rowadr[i] + m->M_rownnz[i] - 1;
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
      d->M, m->M_rownnz, m->M_rowadr, m->M_colind,
      buf_val, buf_ind);

    mj_freeStack(d);
  }

  // dense
  else {
    mju_addToSymSparse(dst, d->M, nv, m->M_rownnz, m->M_rowadr, m->M_colind, /*flg_upper*/ 0);
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
  int nbody = m->nbody;
  const mjtNum *xfrc = d->xfrc_applied;

  // quick return if identically zero (efficient memcmp implementation)
  if (mju_isZeroByte((const unsigned char*)(xfrc+6), 6*(nbody-1)*sizeof(mjtNum))) {
    return;
  }

  // some non-zero wrenches, apply them
  for (int i=1; i < nbody; i++) {
    if (!mju_isZero(xfrc+6*i, 6)) {
      mj_applyFT(m, d, xfrc+6*i, xfrc+6*i+3, d->xipos+3*i, i, qfrc);
    }
  }
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

  // witness points are only computed if dist <= distmax
  if (fromto && status.nx > 0) {
    mju_copy3(fromto, status.x1);
    mju_copy3(fromto+3, status.x2);
  }

  // clamp dist to distmax as mjc_ccd returns DBL_MAX if dist > distmax
  return dist < distmax ? dist : distmax;
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

  // use nativeccd if flag is enabled
  if (!mjDISABLED(mjDSBL_NATIVECCD)) {
    if (func == mjc_Convex || func == mjc_BoxBox) {
      return mj_geomDistanceCCD(m, d, geom1, geom2, distmax, fromto);
    }
  }

  // call collision function with distmax as margin
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


// integrate qpos with given qvel for given body indices
void mj_integratePosInd(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt,
                        const int* index, int nbody) {
  for (int b=1; b < nbody; b++) {
    int k = index ? index[b] : b;
    int start = m->body_jntadr[k];
    int end = start + m->body_jntnum[k];
    for (int j=start; j < end; j++) {
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
}


// integrate qpos with given qvel
void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt) {
  mj_integratePosInd(m, qpos, qvel, dt, NULL, m->nbody);
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


// version number
int mj_version(void) {
  return mjVERSION;
}


// current version of MuJoCo as a null-terminated string
const char* mj_versionString(void) {
  static const char versionstring[] = mjVERSIONSTRING;
  return versionstring;
}


// return total size of data in a contact sensor bitfield specification
int mju_condataSize(int dataspec) {
  int size = 0;
  for (int i=0; i < mjNCONDATA; i++) {
    if (dataspec & (1 << i)) {
      size += mjCONDATA_SIZE[i];
    }
  }
  return size;
}
