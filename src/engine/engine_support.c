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

#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
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

#define mjVERSION 222
#define mjVERSIONSTRING "2.2.2"

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
  "Sensor"
};


// names of enable flags
const char* mjENABLESTRING[mjNENABLE] = {
  "Override",
  "Energy",
  "Fwdinv",
  "Sensornoise",
  "MultiCCD"
};


// names of timers
const char* mjTIMERSTRING[mjNTIMER]= {
  "step",
  "forward",
  "inverse",
  "position",
  "velocity",
  "actuation",
  "acceleration",
  "constraint",
  "pos_kinematics",
  "pos_inertia",
  "pos_collision",
  "pos_make",
  "pos_project"
};



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
  while (da>=0) {
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
  mjMARKSTACK;
  mjtNum* jacp_b = mj_stackAlloc(d, 3*nv);

  // clear output
  mju_zero(jacp, 3*nv);

  // forward pass starting from body
  for (int b=body; b<m->nbody; b++) {
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

  mjFREESTACK;
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
  mjMARKSTACK;
  mjtNum* jacp = (jacPoint ? jacPoint : mj_stackAlloc(d, 3*nv));
  mjtNum* jacr = mj_stackAlloc(d, 3*nv);
  mj_jac(m, d, jacp, jacr, point, body);

  // jacAxis_col = cross(jacr_col, axis)
  if (jacAxis) {
    for (int i=0; i<nv; i++) {
      jacAxis[     i] = jacr[  nv+i]*axis[2] - jacr[2*nv+i]*axis[1];
      jacAxis[  nv+i] = jacr[2*nv+i]*axis[0] - jacr[     i]*axis[2];
      jacAxis[2*nv+i] = jacr[     i]*axis[1] - jacr[  nv+i]*axis[0];
    }
  }

  mjFREESTACK;
}



// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void mj_jacSparse(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body,
                  int NV, int* chain) {
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
  while (da>=0) {
    // find chain index for this dof
    while (ci>=0 && chain[ci]>da) {
      ci--;
    }

    // make sure we found it; SHOULD NOT OCCUR
    if (chain[ci]!=da) {
      mju_error_i("dof index %d not found in chain", da);
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
  for (int da=m->body_dofadr[body]; da<end; da++) {
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
                         b1<b2 ? 0 : m->body_dofnum[b2]);

      // second body
      mj_jacSparseSimple(m, d, jacdifp, jacdifr, pos2, b2, 1, NV,
                         b2<b1 ? 0 : m->body_dofnum[b1]);
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



//-------------------------- name functions --------------------------------------------------------

// get number of objects and name addresses for given object type
static int _getnumadr(const mjModel* m, mjtObj type, int** padr) {
  // get address list and size for object type
  switch (type) {
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    *padr = m->name_bodyadr;
    return m->nbody;
    break;

  case mjOBJ_JOINT:
    *padr = m->name_jntadr;
    return m->njnt;
    break;

  case mjOBJ_GEOM:
    *padr = m->name_geomadr;
    return m->ngeom;
    break;

  case mjOBJ_SITE:
    *padr = m->name_siteadr;
    return m->nsite;
    break;

  case mjOBJ_CAMERA:
    *padr = m->name_camadr;
    return m->ncam;
    break;

  case mjOBJ_LIGHT:
    *padr = m->name_lightadr;
    return m->nlight;
    break;

  case mjOBJ_MESH:
    *padr = m->name_meshadr;
    return m->nmesh;
    break;

  case mjOBJ_SKIN:
    *padr = m->name_skinadr;
    return m->nskin;
    break;

  case mjOBJ_HFIELD:
    *padr = m->name_hfieldadr;
    return m->nhfield;
    break;

  case mjOBJ_TEXTURE:
    *padr = m->name_texadr;
    return m->ntex;
    break;

  case mjOBJ_MATERIAL:
    *padr = m->name_matadr;
    return m->nmat;
    break;

  case mjOBJ_PAIR:
    *padr = m->name_pairadr;
    return m->npair;
    break;

  case mjOBJ_EXCLUDE:
    *padr = m->name_excludeadr;
    return m->nexclude;
    break;

  case mjOBJ_EQUALITY:
    *padr = m->name_eqadr;
    return m->neq;
    break;

  case mjOBJ_TENDON:
    *padr = m->name_tendonadr;
    return m->ntendon;
    break;

  case mjOBJ_ACTUATOR:
    *padr = m->name_actuatoradr;
    return m->nu;
    break;

  case mjOBJ_SENSOR:
    *padr = m->name_sensoradr;
    return m->nsensor;
    break;

  case mjOBJ_NUMERIC:
    *padr = m->name_numericadr;
    return m->nnumeric;
    break;

  case mjOBJ_TEXT:
    *padr = m->name_textadr;
    return m->ntext;
    break;

  case mjOBJ_TUPLE:
    *padr = m->name_tupleadr;
    return m->ntuple;
    break;

  case mjOBJ_KEY:
    *padr = m->name_keyadr;
    return m->nkey;
    break;

  default:
    *padr = 0;
    return 0;
  }
}



// get id of object with specified name; -1: not found
int mj_name2id(const mjModel* m, int type, const char* name) {
  int num = 0;
  int* adr = 0;

  // get number of objects and name addresses
  num = _getnumadr(m, type, &adr);

  // search
  if (num) {
    for (int i=0; i<num; i++) {
      if (!strncmp(name, m->names+adr[i], m->nnames-adr[i])) {
        return i;
      }
    }
  }

  return -1;
}



// get name of object with specified id; 0: invalid type or id, or null name
const char* mj_id2name(const mjModel* m, int type, int id) {
  int num = 0;
  int* adr = 0;

  // get number of objects and name addresses
  num = _getnumadr(m, type, &adr);

  if (id>=0 && id<num) {
    if (m->names[adr[id]]) {
      return m->names+adr[id];
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}



//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M) {
  int adr = 0, nv = m->nv;
  mju_zero(dst, nv*nv);

  for (int i=0; i<nv; i++) {
    int j = i;
    while (j>=0) {
      dst[i*nv+j] = M[adr];
      dst[j*nv+i] = M[adr];
      j = m->dof_parentid[j];
      adr++;
    }
  }
}



// multiply vector by inertia matrix
void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  int adr, nv = m->nv;
  const mjtNum* M = d->qM;
  const int* dofMadr = m->dof_Madr;

  mju_zero(res, nv);

  for (int i=0; i<nv; i++) {
#ifdef mjUSEAVX
    // simple: diagonal division, AVX
    if (m->dof_simplenum[i]>=4) {
      // init
      __m256d result, val1, val2;

      // parallel computation
      val1 = _mm256_loadu_pd(vec+i);
      val2 = _mm256_set_pd(M[dofMadr[i+3]],
                           M[dofMadr[i+2]],
                           M[dofMadr[i+1]],
                           M[dofMadr[i+0]]);
      result = _mm256_mul_pd(val1, val2);

      // store result
      _mm256_storeu_pd(res+i, result);

      // skip rest of block
      i += 3;
      continue;
    }
#endif

    // simple: diagonal muiltiplication
    if (m->dof_simplenum[i]) {
      res[i] = M[dofMadr[i]]*vec[i];
    }

    // regular: full multiplication
    else {
      // diagonal
      adr = dofMadr[i];
      res[i] += M[adr]*vec[i];

      // off-diagonal
      int j = m->dof_parentid[i];
      adr++;
      while (j>=0) {
        res[i] += M[adr]*vec[j];
        res[j] += M[adr]*vec[i];

        // advance to next element
        j = m->dof_parentid[j];
        adr++;
      }
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

  for (int i=0; i<nv; i++) {
#ifdef mjUSEAVX
    // simple: diagonal division, AVX
    if (m->dof_simplenum[i]>=4) {
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
      while (j>=0) {
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
  int adr, adr1, nv = m->nv;

  // sparse
  if (rownnz && rowadr && colind) {
    // special processing of simple dofs
    int simplecnt = 0;
    for (int i=0; i<nv; i++) {
      if (m->dof_simplenum[i]) {
        // count simple
        simplecnt++;

        // empty row: create entry
        if (!rownnz[i]) {
          colind[rowadr[i]] = i;
          dst[rowadr[i]] = d->qM[m->dof_Madr[i]];
          rownnz[i] = 1;
        }

        // non-empty row: assume dof is in dst (J'*D*J satisfies this)
        else {
          // find dof in row, add
          adr = rowadr[i];
          int end = adr + rownnz[i];
          while (adr<end)
            if (colind[adr]==i) {
              dst[adr] += d->qM[m->dof_Madr[i]];
              break;
            } else {
              adr++;
            }

          // not found: error
          if (adr>=end) {
            mju_error("mj_addM sparse: dst row expected to be empty");
          }
        }
      }
    }

    // done if all simple
    if (simplecnt==nv) {
      return;
    }

    // allocate space for sparse M
    mjMARKSTACK;
    mjtNum* M = mj_stackAlloc(d, nv*nv);
    int* M_rownnz = (int*) mj_stackAlloc(d, nv);
    int* M_rowadr = (int*) mj_stackAlloc(d, nv);
    int* M_colind = (int*) mj_stackAlloc(d, nv*nv);
    int* buf_ind = (int*) mj_stackAlloc(d, nv);
    mjtNum* sparse_buf = mj_stackAlloc(d, nv);

    // convert M into sparse format, lower-triangular
    for (int i=0; i<nv; i++) {
      if (!m->dof_simplenum[i]) {
        // backward pass over dofs: construct M_row(i) in reverse order
        adr = m->dof_Madr[i];
        int j = i;
        adr1 = 0;
        while (j>=0) {
          // assign
          M[i*nv+adr1] = d->qM[adr];
          M_colind[i*nv+adr1] = j;

          // count columns
          adr1++;

          // advance
          adr++;
          j = m->dof_parentid[j];
        }

        // assign row descriptors
        M_rownnz[i] = adr1;
        M_rowadr[i] = i*nv;

        // reverse order
        for (int j=0; j<adr1/2; j++) {
          mjtNum tmp = M[i*nv+j];
          M[i*nv+j] = M[i*nv+adr1-1-j];
          M[i*nv+adr1-1-j] = tmp;

          int tmpi = M_colind[i*nv+j];
          M_colind[i*nv+j] = M_colind[i*nv+adr1-1-j];
          M_colind[i*nv+adr1-1-j] = tmpi;
        }
      }
    }

    // make symmetric
    for (int i=1; i<nv; i++) {
      if (!m->dof_simplenum[i]) {
        for (int adr=nv*i; adr<nv*i+M_rownnz[i]-1; adr++) {
          // add to row given by column index
          adr1 = nv*M_colind[adr] + M_rownnz[M_colind[adr]]++;
          M[adr1] = M[adr];
          M_colind[adr1] = i;
        }
      }
    }

    // add to destination
    for (int i=0; i<nv; i++) {
      if (!m->dof_simplenum[i]) {
        int new_nnz =
          mju_combineSparse(dst + rowadr[i], M + M_rowadr[i], nv, 1, 1,
                            rownnz[i], M_rownnz[i],
                            colind + rowadr[i], M_colind + M_rowadr[i],
                            sparse_buf, buf_ind);

        rownnz[i] = new_nnz;
      }
    }

    mjFREESTACK;
  }

  // dense
  else {
    for (int i=0; i<nv; i++) {
      adr = m->dof_Madr[i];
      int j = i;
      while (j>=0) {
        // add
        dst[i*nv+j] += d->qM[adr];
        if (j<i) {
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
}



// construct sparse matrix representations matching qM
void mj_makeMSparse(const mjModel* m, mjData* d, int* rownnz, int* rowadr, int* colind) {
  int nv = m->nv;

  mjMARKSTACK;
  int *remaining = (int*) mj_stackAlloc(d, nv);

  // compute rownnz
  memset(rownnz, 0, nv*sizeof(int));
  for (int i=nv-1; i>=0; i--) {
    // init at diagonal
    int j = i;
    rownnz[i]++;

    // process below diagonal
    while ((j=m->dof_parentid[j]) >= 0) {
      rownnz[i]++;
      rownnz[j]++;
    }
  }

  // accumulate rowadr
  rowadr[0] = 0;
  for (int i=1; i<nv; i++) {
    rowadr[i] = rowadr[i-1] + rownnz[i-1];
  }

  // populate colind
  memcpy(remaining, rownnz, nv*sizeof(int));
  for (int i=nv-1; i>=0; i--) {
    // init at diagonal
    remaining[i]--;
    colind[rowadr[i] + remaining[i]] = i;

    // process below diagonal
    int j = i;
    while ((j = m->dof_parentid[j]) >= 0) {
      remaining[i]--;
      colind[rowadr[i] + remaining[i]] = j;

      remaining[j]--;
      colind[rowadr[j] + remaining[j]] = i;
    }
  }

  // sanity check; SHOULD NOT OCCUR
  for (int i=0; i<nv; i++) {
    if (remaining[i]!=0) {
      mju_error("Error in mj_makeMSparse: unexpected remaining");
    }
  }

  mjFREESTACK
}



// set dst = qM, handle different sparsity representations
void mj_setMSparse(const mjModel* m, mjData* d, mjtNum* dst,
                   const int *rownnz, const int *rowadr, const int *colind) {
  int nv = m->nv;

  mjMARKSTACK;
  int *remaining = (int*) mj_stackAlloc(d, nv);

  // copy data
  memcpy(remaining, rownnz, nv*sizeof(int));
  for (int i=nv-1; i>=0; i--) {
    // init at diagonal
    int adr = m->dof_Madr[i];
    remaining[i]--;
    dst[rowadr[i] + remaining[i]] = d->qM[adr];
    adr++;

    // process below diagonal
    int j = i;
    while ((j = m->dof_parentid[j]) >= 0) {
      remaining[i]--;
      dst[rowadr[i] + remaining[i]] = d->qM[adr];

      remaining[j]--;
      dst[rowadr[j] + remaining[j]] = d->qM[adr];

      adr++;
    }
  }

  mjFREESTACK
}



//-------------------------- perturbations ---------------------------------------------------------

// add cartesian force and torque to qfrc_target
void mj_applyFT(const mjModel* m, mjData* d,
                const mjtNum force[3], const mjtNum torque[3],
                const mjtNum point[3], int body, mjtNum* qfrc_target) {
  int nv = m->nv;

  // allocate local variables
  mjMARKSTACK;
  mjtNum* jacp = mj_stackAlloc(d, 3*nv);
  mjtNum* jacr = mj_stackAlloc(d, 3*nv);
  mjtNum* qforce = mj_stackAlloc(d, nv);

  // make sure body is in range
  if (body<0 || body>=m->nbody) {
    mju_error_i("Invalid body %d in applyFT", body);
  }

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

  mjFREESTACK;
}



// accumulate xfrc_applied in qfrc
void mj_xfrcAccumulate(const mjModel* m, mjData* d, mjtNum* qfrc) {
  for (int i=1; i<m->nbody; i++) {
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
  if (objtype==mjOBJ_BODY) {
    bodyid = objid;
    pos = d->xipos+3*objid;
    rot = (flg_local ? d->ximat+9*objid : 0);
  }

  // body-regular
  else if (objtype==mjOBJ_XBODY) {
    bodyid = objid;
    pos = d->xpos+3*objid;
    rot = (flg_local ? d->xmat+9*objid : 0);
  }

  // geom
  else if (objtype==mjOBJ_GEOM) {
    bodyid = m->geom_bodyid[objid];
    pos = d->geom_xpos+3*objid;
    rot = (flg_local ? d->geom_xmat+9*objid : 0);
  }

  // site
  else if (objtype==mjOBJ_SITE) {
    bodyid = m->site_bodyid[objid];
    pos = d->site_xpos+3*objid;
    rot = (flg_local ? d->site_xmat+9*objid : 0);
  }

  // camera
  else if (objtype==mjOBJ_CAMERA) {
    bodyid = m->cam_bodyid[objid];
    pos = d->cam_xpos+3*objid;
    rot = (flg_local ? d->cam_xmat+9*objid : 0);
  }

  // object without spatial frame
  else {
    mju_error_i("Invalid object type %d in mj_objectVelocity", objtype);
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
  if (objtype==mjOBJ_BODY) {
    bodyid = objid;
    pos = d->xipos+3*objid;
    rot = (flg_local ? d->ximat+9*objid : 0);
  }

  // body-regular
  else if (objtype==mjOBJ_XBODY) {
    bodyid = objid;
    pos = d->xpos+3*objid;
    rot = (flg_local ? d->xmat+9*objid : 0);
  }

  // geom
  else if (objtype==mjOBJ_GEOM) {
    bodyid = m->geom_bodyid[objid];
    pos = d->geom_xpos+3*objid;
    rot = (flg_local ? d->geom_xmat+9*objid : 0);
  }

  // site
  else if (objtype==mjOBJ_SITE) {
    bodyid = m->site_bodyid[objid];
    pos = d->site_xpos+3*objid;
    rot = (flg_local ? d->site_xmat+9*objid : 0);
  }

  // camera
  else if (objtype==mjOBJ_CAMERA) {
    bodyid = m->cam_bodyid[objid];
    pos = d->cam_xpos+3*objid;
    rot = (flg_local ? d->cam_xmat+9*objid : 0);
  }

  // object without spatial frame
  else {
    mju_error_i("Invalid object type %d in mj_objectAcceleration", objtype);
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

// extract 6D force:torque for one contact, in contact frame
void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]) {
  mjContact* con;

  // clear result
  mju_zero(result, 6);

  // make sure contact is valid
  if (id>=0 && id<d->ncon && d->contact[id].efc_address>=0) {
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
  int padr, vadr;
  mjtNum neg[4], dif[4];

  // loop over joints
  for (int j=0; j<m->njnt; j++) {
    // get addresses in qpos and qvel
    padr = m->jnt_qposadr[j];
    vadr = m->jnt_dofadr[j];

    switch (m->jnt_type[j]) {
    case mjJNT_FREE:
      for (int i=0; i<3; i++) {
        qvel[vadr+i] = (qpos2[padr+i] - qpos1[padr+i]) / dt;
      }
      vadr += 3;
      padr += 3;

    // continute with rotations

    case mjJNT_BALL:
      mju_negQuat(neg, qpos1+padr);           // solve:  qpos1 * dif = qpos2
      mju_mulQuat(dif, neg, qpos2+padr);
      mju_quat2Vel(qvel+vadr, dif, dt);
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
  for (int j=0; j<m->njnt; j++) {
    // get addresses in qpos and qvel
    int padr = m->jnt_qposadr[j];
    int vadr = m->jnt_dofadr[j];

    switch (m->jnt_type[j]) {
    case mjJNT_FREE:
      // position update
      for (int i=0; i<3; i++) {
        qpos[padr+i] += dt * qvel[vadr+i];
      }
      padr += 3;
      vadr += 3;

    // continue with rotation update

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
  for (int i=0; i<m->njnt; i++) {
    if (m->jnt_type[i]==mjJNT_BALL || m->jnt_type[i]==mjJNT_FREE) {
      mju_normalize4(qpos+m->jnt_qposadr[i]+3*(m->jnt_type[i]==mjJNT_FREE));
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
    if (sameframe==0) {
      mju_rotVecMat(xpos, pos, d->xmat+9*body);
      mju_addTo3(xpos, d->xpos+3*body);
    }

    // copy body position
    else if (sameframe==1) {
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
    if (sameframe==0) {
      mju_mulQuat(tmp, d->xquat+4*body, quat);
      mju_quat2Mat(xmat, tmp);
    }

    // copy body orientation
    else if (sameframe==1) {
      mju_copy(xmat, d->xmat+9*body, 9);
    }

    // copy inertial body orientation
    else {
      mju_copy(xmat, d->ximat+9*body, 9);
    }
  }
}



// sum all body masses
mjtNum mj_getTotalmass(const mjModel* m) {
  mjtNum res = 0;

  for (int i=1; i<m->nbody; i++) {
    res += m->body_mass[i];
  }

  return res;
}



// scale all body masses and inertias to achieve specified total mass
void mj_setTotalmass(mjModel* m, mjtNum newmass) {
  // compute scale factor, avoid zeros
  mjtNum scale = mjMAX(mjMINVAL, newmass / mjMAX(mjMINVAL, mj_getTotalmass(m)));

  // scale all masses and inertias
  for (int i=1; i<m->nbody; i++) {
    m->body_mass[i] *= scale;
    m->body_inertia[3*i] *= scale;
    m->body_inertia[3*i+1] *= scale;
    m->body_inertia[3*i+2] *= scale;
  }

  // don't forget to call mj_set0 after changing masses
}



// count warnings, print only the first time
void mj_warning(mjData* d, int warning, int info) {
  char str[1000];

  // check type
  if (warning<0 || warning>=mjNWARNING) {
    mju_error_i("Invalid warning type %d", warning);
  }

  // save info (override previous)
  d->warning[warning].lastinfo = info;

  // print message only the first time this warning is encountered
  if (!d->warning[warning].number) {
    mjSNPRINTF(str, "%s Time = %.4f.", mju_warningText(warning, info), d->time);
    mju_warning(str);
  }

  // increase counter
  d->warning[warning].number++;
}



// version number
int mj_version(void) {
  return mjVERSION;
}



// current version of MuJoCo as a null-terminated string
const char* mj_versionString() {
  static const char versionstring[] = mjVERSIONSTRING;
  return versionstring;
}
