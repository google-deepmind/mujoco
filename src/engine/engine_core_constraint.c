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

#include "engine/engine_core_constraint.h"

#include <stdio.h>
#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjxmacro.h>
#include "engine/engine_core_smooth.h"
#include "engine/engine_io.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#ifdef mjUSEPLATFORMSIMD
  #if defined(__AVX__) && !defined(mjUSESINGLE)
    #define mjUSEAVX
  #endif  // defined(__AVX__) && !defined(mjUSESINGLE)
#endif  // mjUSEPLATFORMSIMD


//-------------------------- utility functions -----------------------------------------------------



// allocate efc arrays on arena, return 1 on success, 0 on failure
static int arenaAllocEfc(const mjModel* m, mjData* d) {
#undef MJ_M
#define MJ_M(n) m->n
#undef MJ_D
#define MJ_D(n) d->n

  // move arena pointer to end of contact array
  d->parena = d->ncon * sizeof(mjContact);

  // poison remaining memory
#ifdef ADDRESS_SANITIZER
  ASAN_POISON_MEMORY_REGION(
    (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif

#define X(type, name, nr, nc)                                                 \
  d->name = mj_arenaAllocByte(d, sizeof(type) * (nr) * (nc), _Alignof(type)); \
  if (!d->name) {                                                             \
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);                               \
    mj_clearEfc(d);                                                           \
    d->parena = d->ncon * sizeof(mjContact);                                  \
    return 0;                                                                 \
  }

  MJDATA_ARENA_POINTERS_SOLVER
  if (mj_isDual(m)) {
    MJDATA_ARENA_POINTERS_DUAL
  }
#undef X

#undef MJ_M
#define MJ_M(n) n
#undef MJ_D
#define MJ_D(n) n

  return 1;
}



// determine type of friction cone
int mj_isPyramidal(const mjModel* m) {
  if (m->opt.cone == mjCONE_PYRAMIDAL) {
    return 1;
  } else {
    return 0;
  }
}



// determine type of constraint Jacobian
int mj_isSparse(const mjModel* m) {
  if (m->opt.jacobian == mjJAC_SPARSE ||
      (m->opt.jacobian == mjJAC_AUTO && m->nv >= 60)) {
    return 1;
  } else {
    return 0;
  }
}



// determine type of solver
int mj_isDual(const mjModel* m) {
  if (m->opt.solver == mjSOL_PGS || m->opt.noslip_iterations > 0) {
    return 1;
  } else {
    return 0;
  }
}



// assign/clamp contact friction parameters
void mj_assignFriction(const mjModel* m, mjtNum* target, const mjtNum* source) {
  if (mjENABLED(mjENBL_OVERRIDE)) {
    for (int i=0; i < 5; i++) {
      target[i] = mju_max(mjMINMU, m->opt.o_friction[i]);
    }
  } else {
    for (int i=0; i < 5; i++) {
      target[i] = mju_max(mjMINMU, source[i]);
    }
  }
}



// assign/override contact reference parameters
void mj_assignRef(const mjModel* m, mjtNum* target, const mjtNum* source) {
  if (mjENABLED(mjENBL_OVERRIDE)) {
    mju_copy(target, m->opt.o_solref, mjNREF);
  } else {
    mju_copy(target, source, mjNREF);
  }
}



// assign/override contact impedance parameters
void mj_assignImp(const mjModel* m, mjtNum* target, const mjtNum* source) {
  if (mjENABLED(mjENBL_OVERRIDE)) {
    mju_copy(target, m->opt.o_solimp, mjNIMP);
  } else {
    mju_copy(target, source, mjNIMP);
  }
}



// assign/override contact margin
mjtNum mj_assignMargin(const mjModel* m, mjtNum source) {
  if (mjENABLED(mjENBL_OVERRIDE)) {
    return m->opt.o_margin;
  } else {
    return source;
  }
}



// compute element bodies and weights for given contact point, return #bodies
// if v is one of the element vertices, reduce element to fragment
static int mj_elemBodyWeight(const mjModel* m, const mjData* d, int f, int e, int v,
                             const mjtNum point[3], int* body, mjtNum* weight) {
  // get flex info
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
  const mjtNum* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

  // compute inverse distances from contact point to element vertices
  // save body ids, find vertex v in element
  int vid = -1;
  for (int i=0; i <= dim; i++) {
    mjtNum dist = mju_dist3(point, vert+3*edata[i]);
    weight[i] = 1.0/(mju_max(mjMINVAL, dist));
    body[i] = m->flex_vertbodyid[m->flex_vertadr[f] + edata[i]];

    // check if element vertex matches v
    if (edata[i] == v) {
      vid = i;
    }
  }

  // v found in e: skip and shift remaining
  if (vid >= 0) {
    while (vid < dim) {
      weight[vid] = weight[vid+1];
      body[vid] = body[vid+1];
      vid++;
    }
    dim--;
  }

  // normalize weights
  mju_normalize(weight, dim+1);
  return dim+1;
}



// add contact to d->contact list; return 0 if success; 1 if buffer full
int mj_addContact(const mjModel* m, mjData* d, const mjContact* con) {
  // if nconmax is specified and ncon >= nconmax, warn and return error
  if (m->nconmax != -1 && d->ncon >= m->nconmax) {
    mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
    return 1;
  }

  // move arena pointer back to the end of the existing contact array and invalidate efc_ arrays
  d->parena = d->ncon * sizeof(mjContact);
#ifdef ADDRESS_SANITIZER
  ASAN_POISON_MEMORY_REGION(
    (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif
  mj_clearEfc(d);

  // copy contact
  mjContact* dst = mj_arenaAllocByte(d, sizeof(mjContact), _Alignof(mjContact));
  if (!dst) {
    mj_warning(d, mjWARN_CONTACTFULL, d->ncon);
    return 1;
  }
  *dst = *con;

  // increase counter, return success
  d->ncon++;
  return 0;
}



// add #size rows to constraint Jacobian; set pos, margin, frictionloss, type, id
static void mj_addConstraint(const mjModel* m, mjData* d,
                             const mjtNum* jac, const mjtNum* pos,
                             const mjtNum* margin, mjtNum frictionloss,
                             int size, int type, int id, int NV, const int* chain) {
  int empty, nv = m->nv, nefc = d->nefc;
  int *nnz = d->efc_J_rownnz, *adr = d->efc_J_rowadr, *ind = d->efc_J_colind;
  mjtNum *J = d->efc_J;

  // init empty guard for constraints other than contact
  if (type == mjCNSTR_CONTACT_FRICTIONLESS ||
      type == mjCNSTR_CONTACT_PYRAMIDAL ||
      type == mjCNSTR_CONTACT_ELLIPTIC) {
    empty = 0;
  } else {
    empty = 1;
  }

  // dense: copy entire Jacobian
  if (!mj_isSparse(m)) {
    // make sure jac is not empty
    if (empty) {
      for (int i=0; i < size*nv; i++) {
        if (jac[i]) {
          empty = 0;
          break;
        }
      }
    }

    // copy if not empty
    if (!empty) {
      mju_copy(J + nefc*nv, jac, size*nv);
    }
  }

  // sparse: copy chain
  else {
    // clamp NV (in case -1 was used in constraint construction)
    NV = mjMAX(0, NV);

    if (NV) {
      empty = 0;
    } else if (empty) {
      // all rows are empty, return early
      return;
    }

    // chain required in sparse mode
    if (NV && !chain) {
      mjERROR("called with dense arguments");
    }

    // process size elements
    for (int i=0; i < size; i++) {
      // set row address
      adr[nefc+i] = (nefc+i ? adr[nefc+i-1]+nnz[nefc+i-1] : 0);

      // set row descriptor
      nnz[nefc+i] = NV;

      // copy if not empty
      if (NV) {
        mju_copyInt(ind + adr[nefc+i], chain, NV);
        mju_copy(J + adr[nefc+i], jac + i*NV, NV);
      }
    }
  }

  // all rows empty: skip constraint
  if (empty) {
    return;
  }

  // set constraint pos, margin, frictionloss, type, id
  for (int i=0; i < size; i++) {
    d->efc_pos[nefc+i] = (pos ? pos[i] : 0);
    d->efc_margin[nefc+i] = (margin ? margin[i] : 0);
    d->efc_frictionloss[nefc+i] = frictionloss;
    d->efc_type[nefc+i] = type;
    d->efc_id[nefc+i] = id;
  }

  // increase counters
  d->nefc += size;
  if (type == mjCNSTR_EQUALITY) {
    d->ne += size;
  } else if (type == mjCNSTR_FRICTION_DOF || type == mjCNSTR_FRICTION_TENDON) {
    d->nf += size;
  } else if (type == mjCNSTR_LIMIT_JOINT || type == mjCNSTR_LIMIT_TENDON) {
    d->nl += size;
  }
}



// multiply Jacobian by vector
void mj_mulJacVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  // exit if no constraints
  if (!d->nefc) {
    return;
  }

  // sparse Jacobian
  if (mj_isSparse(m))
    mju_mulMatVecSparse(res, d->efc_J, vec, d->nefc,
                        d->efc_J_rownnz, d->efc_J_rowadr,
                        d->efc_J_colind, d->efc_J_rowsuper);

  // dense Jacobian
  else {
    mju_mulMatVec(res, d->efc_J, vec, d->nefc, m->nv);
  }
}



// multiply Jacobian by vector, for one island
//  flg_resunc and flg_vecunc denote whether res/vec are uncompressed
void mj_mulJacVec_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
                         int island, int flg_resunc, int flg_vecunc) {
  // no island, call regular function
  if (island < 0) {
    mj_mulJacVec(m, d, res, vec);
    return;
  }

  // sizes
  int vecnnz = d->island_dofnum[island];
  int resnnz = d->island_efcnum[island];

  // indices
  int* vecind = d->island_dofind + d->island_dofadr[island];
  int* resind = d->island_efcind + d->island_efcadr[island];

  // sparse Jacobian
  if (mj_isSparse(m)) {
    for (int i=0; i < resnnz; i++) {
      int row = resind[i];
      int Jnnz = d->efc_J_rownnz[row];
      int Jrowadr = d->efc_J_rowadr[row];
      int* Jind = d->efc_J_colind + Jrowadr;
      mjtNum* J = d->efc_J + Jrowadr;
      int j = flg_resunc ? row : i;
      res[j] = mju_dotSparse2(J, vec, Jnnz, Jind, vecnnz, vecind, flg_vecunc);
    }
  }

  // dense Jacobian
  else {
    int nv = m->nv;
    for (int i=0; i < resnnz; i++) {
      int row = resind[i];
      int j = flg_resunc ? row : i;
      res[j] = mju_dotSparse(vec, d->efc_J + nv*row, vecnnz, vecind, flg_vecunc);
    }
  }
}



// multiply JacobianT by vector
void mj_mulJacTVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec) {
  // exit if no constraints
  if (!d->nefc) {
    return;
  }

  // sparse Jacobian
  if (mj_isSparse(m))
    mju_mulMatVecSparse(res, d->efc_JT, vec, m->nv,
                        d->efc_JT_rownnz, d->efc_JT_rowadr,
                        d->efc_JT_colind, d->efc_JT_rowsuper);

  // dense Jacobian
  else {
    mju_mulMatTVec(res, d->efc_J, vec, d->nefc, m->nv);
  }
}



// multiply Jacobian transpose by vector, for one island
//  flg_resunc and flg_vecunc denote whether res/vec are uncompressed
void mj_mulJacTVec_island(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
                          int island, int flg_resunc, int flg_vecunc) {
  // no island, call regular function
  if (island < 0) {
    mj_mulJacTVec(m, d, res, vec);
    return;
  }

  // sizes
  int vecnnz = d->island_efcnum[island];
  int resnnz = d->island_dofnum[island];

  // indices
  int* vecind = d->island_efcind + d->island_efcadr[island];
  int* resind = d->island_dofind + d->island_dofadr[island];

  // sparse Jacobian
  if (mj_isSparse(m)) {
    for (int i=0; i < resnnz; i++) {
      int row = resind[i];
      int JTnnz = d->efc_JT_rownnz[row];
      int JTrowadr = d->efc_JT_rowadr[row];
      int* JTind = d->efc_JT_colind + JTrowadr;
      mjtNum* JT = d->efc_JT + JTrowadr;
      int j = flg_resunc ? row : i;
      res[j] = mju_dotSparse2(JT, vec, JTnnz, JTind, vecnnz, vecind, flg_vecunc);
    }
  }

  // dense Jacobian
  else {
    int nefc = d->nefc;
    for (int i=0; i < resnnz; i++) {
      int row = resind[i];
      int j = flg_resunc ? row : i;
      res[j] = mju_dotSparse(vec, d->efc_JT + nefc*row, vecnnz, vecind, flg_vecunc);
    }
  }
}



//--------------------- instantiate constraints by type --------------------------------------------

// equality constraints
void mj_instantiateEquality(const mjModel* m, mjData* d) {
  int issparse = mj_isSparse(m), nv = m->nv;
  int id[2], size, NV, NV2, *chain = NULL, *chain2 = NULL, *buf_ind = NULL;
  int flex_edgeadr, flex_edgenum;
  mjtNum cpos[6], pos[2][3], ref[2], dif, deriv;
  mjtNum quat[4], quat1[4], quat2[4], quat3[4], axis[3];
  mjtNum *jac[2], *jacdif, *data, *sparse_buf = NULL;

  // disabled or no equality constraints: return
  if (mjDISABLED(mjDSBL_EQUALITY) || m->nemax == 0) {
    return;
  }

  mj_markStack(d);

  // allocate space
  jac[0] = mjSTACKALLOC(d, 6*nv, mjtNum);
  jac[1] = mjSTACKALLOC(d, 6*nv, mjtNum);
  jacdif = mjSTACKALLOC(d, 6*nv, mjtNum);
  if (issparse) {
    chain = mjSTACKALLOC(d, nv, int);
    chain2 = mjSTACKALLOC(d, nv, int);
    buf_ind = mjSTACKALLOC(d, nv, int);
    sparse_buf = mjSTACKALLOC(d, nv, mjtNum);
  }

  // find active equality constraints
  for (int i=0; i < m->neq; i++) {
    if (d->eq_active[i]) {
      // get constraint data
      data = m->eq_data + mjNEQDATA*i;
      id[0] = m->eq_obj1id[i];
      id[1] = m->eq_obj2id[i];
      size = 0;
      NV = 0;
      NV2 = 0;
      int body_id[2];

      // process according to type
      switch ((mjtEq) m->eq_type[i]) {
      case mjEQ_CONNECT:              // connect bodies with ball joint
        // find global points, body semantic
        if (m->eq_objtype[i] == mjOBJ_BODY) {
          for (int j=0; j < 2; j++) {
            mju_mulMatVec3(pos[j], d->xmat + 9*id[j], data + 3*j);
            mju_addTo3(pos[j], d->xpos + 3*id[j]);
            body_id[j] = id[j];
          }
        }

        // find global points, site semantic
        else {
          for (int j=0; j < 2; j++) {
            mju_copy3(pos[j], d->site_xpos + 3*id[j]);
            body_id[j] = m->site_bodyid[id[j]];
          }
        }

        // compute position error
        mju_sub3(cpos, pos[0], pos[1]);

        // compute Jacobian difference (opposite of contact: 0 - 1)
        NV = mj_jacDifPair(m, d, chain, body_id[1], body_id[0], pos[1], pos[0],
                           jac[1], jac[0], jacdif, NULL, NULL, NULL);

        // copy difference into jac[0]
        mju_copy(jac[0], jacdif, 3*NV);

        size = 3;
        break;

      case mjEQ_WELD:                 // fix relative position and orientation
        // find global points, body semantic
        if (m->eq_objtype[i] == mjOBJ_BODY) {
          for (int j=0; j < 2; j++) {
            mjtNum* anchor = data + 3*(1-j);
            mju_mulMatVec3(pos[j], d->xmat + 9*id[j], anchor);
            mju_addTo3(pos[j], d->xpos + 3*id[j]);
            body_id[j] = id[j];
          }
        }

        // find global points, site semantic
        else {
          for (int j=0; j < 2; j++) {
            mju_copy3(pos[j], d->site_xpos + 3*id[j]);
            body_id[j] = m->site_bodyid[id[j]];
          }
        }

        // compute position error
        mju_sub3(cpos, pos[0], pos[1]);

        // get torquescale coefficient
        mjtNum torquescale = data[10];

        // compute error Jacobian (opposite of contact: 0 - 1)
        NV = mj_jacDifPair(m, d, chain, body_id[1], body_id[0], pos[1], pos[0],
                           jac[1], jac[0], jacdif,
                           jac[1]+3*nv, jac[0]+3*nv, jacdif+3*nv);

        // copy difference into jac[0], compress translation:rotation if sparse
        mju_copy(jac[0], jacdif, 3*NV);
        mju_copy(jac[0]+3*NV, jacdif+3*nv, 3*NV);

        // orientation, body semantic
        if (m->eq_objtype[i] == mjOBJ_BODY) {
          // compute orientation error: neg(q1) * q0 * relpose (axis components only)
          mjtNum* relpose = data+6;
          mju_mulQuat(quat, d->xquat+4*id[0], relpose);   // quat = q0*relpose
          mju_negQuat(quat1, d->xquat+4*id[1]);           // quat1 = neg(q1)
        }

        // orientation, site semantic
        else {
          mjtNum quat_site1[4];
          mju_mulQuat(quat, d->xquat+4*body_id[0], m->site_quat+4*id[0]);
          mju_mulQuat(quat_site1, d->xquat+4*body_id[1], m->site_quat+4*id[1]);
          mju_negQuat(quat1, quat_site1);
        }

        mju_mulQuat(quat2, quat1, quat);
        mju_scl3(cpos+3, quat2+1, torquescale);         // scale axis components by torquescale

        // correct rotation Jacobian: 0.5 * neg(q1) * (jac0-jac1) * q0 * relpose
        for (int j=0; j < NV; j++) {
          // axis = [jac0-jac1]_col(j)
          axis[0] = jac[0][3*NV+j];
          axis[1] = jac[0][4*NV+j];
          axis[2] = jac[0][5*NV+j];

          // apply formula
          mju_mulQuatAxis(quat2, quat1, axis);    // quat2 = neg(q1)*(jac0-jac1)
          mju_mulQuat(quat3, quat2, quat);        // quat3 = neg(q1)*(jac0-jac1)*q0*relpose

          // correct Jacobian
          jac[0][3*NV+j] = 0.5*quat3[1];
          jac[0][4*NV+j] = 0.5*quat3[2];
          jac[0][5*NV+j] = 0.5*quat3[3];
        }

        // scale rotational jacobian by torquescale
        mju_scl(jac[0]+3*NV, jac[0]+3*NV, torquescale, 3*NV);

        size = 6;
        break;

      case mjEQ_JOINT:                // couple joint values with cubic
      case mjEQ_TENDON:               // couple tendon lengths with cubic
        // get scalar positions and their Jacobians
        for (int j=0; j < 1+(id[1] >= 0); j++) {
          if (m->eq_type[i] == mjEQ_JOINT) {    // joint object
            pos[j][0] = d->qpos[m->jnt_qposadr[id[j]]];
            ref[j] = m->qpos0[m->jnt_qposadr[id[j]]];

            // make Jacobian: sparse or dense
            if (issparse) {
              // add first or second joint
              if (j == 0) {
                NV = 1;
                chain[0] = m->jnt_dofadr[id[j]];
                jac[j][0] = 1;
              } else {
                NV2 = 1;
                chain2[0] = m->jnt_dofadr[id[j]];
                jac[j][0] = 1;
              }
            } else {
              mju_zero(jac[j], nv);
              jac[j][m->jnt_dofadr[id[j]]] = 1;
            }
          } else {                            // tendon object
            pos[j][0] = d->ten_length[id[j]];
            ref[j] = m->tendon_length0[id[j]];

            // set tendon_efcadr
            if (d->tendon_efcadr[id[j]] == -1) {
              d->tendon_efcadr[id[j]] = i;
            }

            // copy Jacobian: sparse or dense
            if (issparse) {
              // add first or second chain
              if (j == 0) {
                NV = d->ten_J_rownnz[id[j]];
                mju_copyInt(chain, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV);
                mju_copy(jac[j], d->ten_J+d->ten_J_rowadr[id[j]], NV);
              } else {
                NV2 = d->ten_J_rownnz[id[j]];
                mju_copyInt(chain2, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV2);
                mju_copy(jac[j], d->ten_J+d->ten_J_rowadr[id[j]], NV2);
              }
            } else {
              mju_copy(jac[j], d->ten_J+id[j]*nv, nv);
            }
          }
        }

        // both objects defined
        if (id[1] >= 0) {
          // compute position error
          dif = pos[1][0] - ref[1];
          cpos[0] = pos[0][0] - ref[0] - data[0] -
                    (data[1]*dif + data[2]*dif*dif + data[3]*dif*dif*dif + data[4]*dif*dif*dif*dif);

          // compute derivative
          deriv = data[1] + 2*data[2]*dif + 3*data[3]*dif*dif + 4*data[4]*dif*dif*dif;

          // compute Jacobian: sparse or dense
          if (issparse) {
            NV = mju_combineSparse(jac[0], jac[1], 1, -deriv, NV, NV2, chain,
                                   chain2, sparse_buf, buf_ind);
          } else {
            mju_addToScl(jac[0], jac[1], -deriv, nv);
          }
        }

        // only one object defined
        else {
          // compute position error
          cpos[0] = pos[0][0] - ref[0] - data[0];

          // jac[0] already has the correct Jacobian
        }

        size = 1;
        break;

      case mjEQ_FLEX:
        flex_edgeadr = m->flex_edgeadr[id[0]];
        flex_edgenum = m->flex_edgenum[id[0]];
        // add one constraint per non-rigid edge
        for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
          // skip rigid
          if (m->flexedge_rigid[e]) {
            continue;
          }

          // position error
          cpos[0] = d->flexedge_length[e] - m->flexedge_length0[e];

          // add constraint: sparse or dense
          if (issparse) {
            mj_addConstraint(m, d, d->flexedge_J+d->flexedge_J_rowadr[e], cpos, 0, 0,
                             1, mjCNSTR_EQUALITY, i,
                             d->flexedge_J_rownnz[e],
                             d->flexedge_J_colind+d->flexedge_J_rowadr[e]);
          } else {
            mj_addConstraint(m, d, d->flexedge_J+e*nv, cpos, 0, 0,
                             1, mjCNSTR_EQUALITY, i,
                             0, NULL);
          }
        }
        break;

      default:                    // SHOULD NOT OCCUR
        mjERROR("invalid equality constraint type %d", m->eq_type[i]);
      }

      // add constraint
      if (size) {
        mj_addConstraint(m, d, jac[0], cpos, 0, 0,
                         size, mjCNSTR_EQUALITY, i,
                         issparse ? NV : 0,
                         issparse ? chain : NULL);
      }
    }
  }

  mj_freeStack(d);
}



// frictional dofs and tendons
void mj_instantiateFriction(const mjModel* m, mjData* d) {
  int nv = m->nv, issparse = mj_isSparse(m);
  mjtNum* jac;

  // disabled: return
  if (mjDISABLED(mjDSBL_FRICTIONLOSS)) {
    return;
  }

  mj_markStack(d);

  // allocate Jacobian
  jac = mjSTACKALLOC(d, nv, mjtNum);

  // find frictional dofs
  for (int i=0; i < nv; i++) {
    if (m->dof_frictionloss[i] > 0) {
      // prepare Jacobian: sparse or dense
      if (issparse) {
        jac[0] = 1;
      } else {
        mju_zero(jac, nv);
        jac[i] = 1;
      }

      // add constraint
      mj_addConstraint(m, d, jac, 0, 0, m->dof_frictionloss[i],
                       1, mjCNSTR_FRICTION_DOF, i,
                       issparse ? 1 : 0,
                       issparse ? &i : NULL);
    }
  }

  // find frictional tendons
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_frictionloss[i] > 0) {
      int efcadr = d->nefc;
      // add constraint
      mj_addConstraint(m, d, d->ten_J + (issparse ? d->ten_J_rowadr[i] : i*nv),
                       0, 0, m->tendon_frictionloss[i],
                       1, mjCNSTR_FRICTION_TENDON, i,
                       issparse ? d->ten_J_rownnz[i] : 0,
                       issparse ? d->ten_J_colind+d->ten_J_rowadr[i] : NULL);
      // set tendon_efcadr
      if (d->tendon_efcadr[i] == -1) {
        d->tendon_efcadr[i] = efcadr;
      }
    }
  }

  mj_freeStack(d);
}



// joint and tendon limits
void mj_instantiateLimit(const mjModel* m, mjData* d) {
  int side, nv = m->nv, issparse = mj_isSparse(m);
  mjtNum margin, value, dist, angleAxis[3];
  mjtNum *jac;

  // disabled: return
  if (mjDISABLED(mjDSBL_LIMIT)) {
    return;
  }

  mj_markStack(d);

  // allocate Jacobian
  jac = mjSTACKALLOC(d, nv, mjtNum);

  // find joint limits
  for (int i=0; i < m->njnt; i++) {
    if (m->jnt_limited[i]) {
      // get margin
      margin = m->jnt_margin[i];

      // HINGE or SLIDE joint
      if (m->jnt_type[i] == mjJNT_SLIDE || m->jnt_type[i] == mjJNT_HINGE) {
        // get joint value
        value = d->qpos[m->jnt_qposadr[i]];

        // process lower and upper limits
        for (side=-1; side <= 1; side+=2) {
          // compute distance (negative: penetration)
          dist = side * (m->jnt_range[2*i+(side+1)/2] - value);

          // detect joint limit
          if (dist < margin) {
            // prepare Jacobian: sparse or dense
            if (issparse) {
              jac[0] = -(mjtNum)side;
            } else {
              mju_zero(jac, nv);
              jac[m->jnt_dofadr[i]] = -(mjtNum)side;
            }

            // add constraint
            mj_addConstraint(m, d, jac, &dist, &margin, 0,
                             1, mjCNSTR_LIMIT_JOINT, i,
                             issparse ? 1 : 0,
                             issparse ? m->jnt_dofadr+i : NULL);
          }
        }
      }

      // BALL joint
      else if (m->jnt_type[i] == mjJNT_BALL) {
        // convert joint quaternion to axis-angle
        int adr = m->jnt_qposadr[i];
        mjtNum quat[4] = {d->qpos[adr], d->qpos[adr+1], d->qpos[adr+2], d->qpos[adr+3]};
        mju_normalize4(quat);
        mju_quat2Vel(angleAxis, quat, 1);

        // get rotation angle, normalize
        value = mju_normalize3(angleAxis);

        // compute distance, using max of range (negative: penetration)
        dist = mju_max(m->jnt_range[2*i], m->jnt_range[2*i+1]) - value;

        // detect joint limit
        if (dist < margin) {
          // sparse
          if (issparse) {
            // prepare dof index array
            int chain[3] = {
              m->jnt_dofadr[i],
              m->jnt_dofadr[i] + 1,
              m->jnt_dofadr[i] + 2
            };

            // prepare Jacobian
            mju_scl3(jac, angleAxis, -1);

            // add constraint
            mj_addConstraint(m, d, jac, &dist, &margin, 0,
                             1, mjCNSTR_LIMIT_JOINT, i, 3, chain);
          }

          // dense
          else {
            // prepare Jacobian
            mju_zero(jac, nv);
            mju_scl3(jac + m->jnt_dofadr[i], angleAxis, -1);

            // add constraint
            mj_addConstraint(m, d, jac, &dist, &margin, 0,
                             1, mjCNSTR_LIMIT_JOINT, i, 0, 0);
          }
        }
      }
    }
  }

  // find tendon limits
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_limited[i]) {
      // get value = length, margin
      value = d->ten_length[i];
      margin = m->tendon_margin[i];

      // process lower and upper limits
      for (side=-1; side <= 1; side+=2) {
        // compute distance (negative: penetration)
        dist = side * (m->tendon_range[2*i+(side+1)/2] - value);

        // detect tendon limit
        if (dist < margin) {
          // prepare Jacobian: sparse or dense
          if (issparse) {
            mju_scl(jac, d->ten_J+d->ten_J_rowadr[i], -side, d->ten_J_rownnz[i]);
          } else {
            mju_scl(jac, d->ten_J+i*nv, -side, nv);
          }

          // add constraint
          int efcadr = d->nefc;
          mj_addConstraint(m, d, jac, &dist, &margin, 0,
                           1, mjCNSTR_LIMIT_TENDON, i,
                           issparse ? d->ten_J_rownnz[i] : 0,
                           issparse ? d->ten_J_colind+d->ten_J_rowadr[i] : NULL);
          // set tendon_efcadr
          if (d->tendon_efcadr[i] == -1) {
            d->tendon_efcadr[i] = efcadr;
          }
        }
      }
    }
  }

  mj_freeStack(d);
}



// frictionless and frictional contacts
void mj_instantiateContact(const mjModel* m, mjData* d) {
  int ispyramid = mj_isPyramidal(m), issparse = mj_isSparse(m), ncon = d->ncon;
  int dim, NV, nv = m->nv, *chain = NULL;
  mjContact* con;
  mjtNum cpos[6], cmargin[6], *jac, *jacdif, *jacdifp, *jacdifr, *jac1p, *jac2p, *jac1r, *jac2r;

  if (mjDISABLED(mjDSBL_CONTACT) || ncon == 0 || nv == 0) {
    return;
  }

  mj_markStack(d);

  // allocate Jacobian
  jac = mjSTACKALLOC(d, 6*nv, mjtNum);
  jacdif = mjSTACKALLOC(d, 6*nv, mjtNum);
  jacdifp = jacdif;
  jacdifr = jacdif + 3*nv;
  jac1p = mjSTACKALLOC(d, 3*nv, mjtNum);
  jac2p = mjSTACKALLOC(d, 3*nv, mjtNum);
  jac1r = mjSTACKALLOC(d, 3*nv, mjtNum);
  jac2r = mjSTACKALLOC(d, 3*nv, mjtNum);
  if (issparse) {
    chain = mjSTACKALLOC(d, nv, int);
  }

  // find contacts to be included
  for (int i=0; i < ncon; i++) {
    if (!d->contact[i].exclude) {
      // get contact info, safe efc_address
      con = d->contact + i;
      dim = con->dim;
      con->efc_address = d->nefc;

      // special case: single body on each side
      if ((con->geom[0] >= 0 || con->vert[0] >= 0) &&
          (con->geom[1] >= 0 || con->vert[1] >= 0)) {
        // get bodies
        int bid[2];
        for (int side=0; side < 2; side++) {
          bid[side] = (con->geom[side] >= 0) ?
                      m->geom_bodyid[con->geom[side]] :
                      m->flex_vertbodyid[m->flex_vertadr[con->flex[side]] + con->vert[side]];
        }

        // compute Jacobian differences
        if (dim > 3) {
          NV = mj_jacDifPair(m, d, chain, bid[0], bid[1], con->pos, con->pos,
                             jac1p, jac2p, jacdifp, jac1r, jac2r, jacdifr);
        } else {
          NV = mj_jacDifPair(m, d, chain, bid[0], bid[1], con->pos, con->pos,
                             jac1p, jac2p, jacdifp, NULL, NULL, NULL);
        }
      }

      // general case: flex elements involved
      else {
        // get bodies and weights
        int nb = 0;
        int bid[8];
        mjtNum bweight[8];
        for (int side=0; side < 2; side++) {
          // geom
          if (con->geom[side] >= 0) {
            bid[nb] = m->geom_bodyid[con->geom[side]];
            bweight[nb] = side ? +1 : -1;
            nb++;
          }

          // flex vert
          else if (con->vert[side] >= 0) {
            bid[nb] = m->flex_vertbodyid[m->flex_vertadr[con->flex[side]] + con->vert[side]];
            bweight[nb] = side ? +1 : -1;
            nb++;
          }

          // flex elem
          else {
            int nw = mj_elemBodyWeight(m, d, con->flex[side], con->elem[side],
                                       con->vert[1-side], con->pos, bid+nb, bweight+nb);

            // negative sign for first side of contact
            if (side == 0) {
              mju_scl(bweight+nb, bweight+nb, -1, nw);
            }

            nb += nw;
          }
        }

        // combine weighted Jacobians
        NV = mj_jacSum(m, d, chain, nb, bid, bweight, con->pos, jacdif, dim > 3);
      }

      // skip contact if no DOFs affected
      if (NV == 0) {
        con->efc_address = -1;
        con->exclude = 3;
        continue;
      }

      // rotate Jacobian differences to contact frame
      mju_mulMatMat(jac, con->frame, jacdifp, dim > 1 ? 3 : 1, 3, NV);
      if (dim > 3) {
        mju_mulMatMat(jac + 3*NV, con->frame, jacdifr, dim-3, 3, NV);
      }

      // make frictionless contact
      if (dim == 1) {
        // add constraint
        mj_addConstraint(m, d, jac, &(con->dist), &(con->includemargin), 0,
                         1, mjCNSTR_CONTACT_FRICTIONLESS, i,
                         issparse ? NV : 0,
                         issparse ? chain : NULL);
      }

      // make pyramidal friction cone
      else if (ispyramid) {
        // pos = dist
        cpos[0] = cpos[1] = con->dist;
        cmargin[0] = cmargin[1] = con->includemargin;

        // one pair per friction dimension
        for (int k=1; k < con->dim; k++) {
          // Jacobian for pair of opposing pyramid edges
          mju_addScl(jacdifp, jac, jac + k*NV, con->friction[k-1], NV);
          mju_addScl(jacdifp + NV, jac, jac + k*NV, -con->friction[k-1], NV);

          // add constraint
          mj_addConstraint(m, d, jacdifp, cpos, cmargin, 0,
                           2, mjCNSTR_CONTACT_PYRAMIDAL, i,
                           issparse ? NV : 0,
                           issparse ? chain : NULL);
        }
      }

      // make elliptic friction cone
      else {
        // normal pos = dist, all others 0
        mju_zero(cpos, con->dim);
        mju_zero(cmargin, con->dim);
        cpos[0] = con->dist;
        cmargin[0] = con->includemargin;

        // add constraint
        mj_addConstraint(m, d, jac, cpos, cmargin, 0,
                         con->dim, mjCNSTR_CONTACT_ELLIPTIC, i,
                         issparse ? NV : 0,
                         issparse ? chain : NULL);
      }
    }
  }

  mj_freeStack(d);
}



//------------------------ compute constraint parameters -------------------------------------------

// compute diagApprox
void mj_diagApprox(const mjModel* m, mjData* d) {
  int id, dim, b1, b2, f, weldcnt = 0;
  int nefc = d->nefc;
  mjtNum tran, rot, fri, *dA = d->efc_diagApprox;
  mjContact* con = NULL;

  // loop over all constraints, compute approximate inverse inertia
  for (int i=0; i < nefc; i++) {
    // get constraint id
    id = d->efc_id[i];

    // process according to constraint type
    switch ((mjtConstraint) d->efc_type[i]) {
    case mjCNSTR_EQUALITY:
      // process according to equality-constraint type
      switch (m->eq_type[id]) {
      case mjEQ_CONNECT:
        b1 = m->eq_obj1id[id];
        b2 = m->eq_obj2id[id];

        // get body ids if using site semantics
        if (m->eq_objtype[id] == mjOBJ_SITE) {
          b1 = m->site_bodyid[b1];
          b2 = m->site_bodyid[b2];
        }

        // body translation
        dA[i] = m->body_invweight0[2*b1] + m->body_invweight0[2*b2];
        break;

      case mjEQ_WELD:  // distinguish translation and rotation inertia
        b1 = m->eq_obj1id[id];
        b2 = m->eq_obj2id[id];

        // get body ids if using site semantics
        if (m->eq_objtype[id] == mjOBJ_SITE) {
          b1 = m->site_bodyid[b1];
          b2 = m->site_bodyid[b2];
        }

        // body translation or rotation depending on weldcnt
        dA[i] = m->body_invweight0[2*b1 + (weldcnt > 2)] +
                m->body_invweight0[2*b2 + (weldcnt > 2)];
        weldcnt = (weldcnt + 1) % 6;
        break;

      case mjEQ_JOINT:
      case mjEQ_TENDON:
        // object 1 contribution
        dA[i] = (m->eq_type[id] == mjEQ_JOINT ?
                 m->dof_invweight0[m->jnt_dofadr[m->eq_obj1id[id]]] :
                 m->tendon_invweight0[m->eq_obj1id[id]]);

        // add object 2 contribution if present
        if (m->eq_obj2id[id] >= 0)
          dA[i] += (m->eq_type[id] == mjEQ_JOINT ?
                    m->dof_invweight0[m->jnt_dofadr[m->eq_obj2id[id]]] :
                    m->tendon_invweight0[m->eq_obj2id[id]]);
        break;

      case mjEQ_FLEX:
        // process all non-rigid edges for this flex
        f = m->eq_obj1id[id];
        int flex_edgeadr = m->flex_edgeadr[f];
        int flex_edgenum = m->flex_edgenum[f];
        for (int e=flex_edgeadr; e<flex_edgeadr+flex_edgenum; e++) {
          if (!m->flexedge_rigid[e]) {
            dA[i++] = m->flexedge_invweight0[e];
          }
        }

        // adjust constraint counter
        i--;
        break;

      default:
        mjERROR("unknown constraint type type %d", d->efc_type[i]);    // SHOULD NOT OCCUR
      }
      break;

    case mjCNSTR_FRICTION_DOF:
      dA[i] = m->dof_invweight0[id];
      break;

    case mjCNSTR_LIMIT_JOINT:
      dA[i] = m->dof_invweight0[m->jnt_dofadr[id]];
      break;

    case mjCNSTR_FRICTION_TENDON:
    case mjCNSTR_LIMIT_TENDON:
      dA[i] = m->tendon_invweight0[id];
      break;

    case mjCNSTR_CONTACT_FRICTIONLESS:
    case mjCNSTR_CONTACT_PYRAMIDAL:
    case mjCNSTR_CONTACT_ELLIPTIC:
      // get contact info
      con = d->contact + id;
      dim = con->dim;

      // add the average translation and rotation components from both sides
      tran = rot = 0;
      for (int side=0; side < 2; side++) {
        // get bodies and weights
        int nb, bid[4];
        mjtNum bweight[4];

        // geom
        if (con->geom[side] >= 0) {
          bid[0] = m->geom_bodyid[con->geom[side]];
          bweight[0] = 1;
          nb = 1;
        }

        // flex vert
        else if (con->vert[side] >= 0) {
          bid[0] = m->flex_vertbodyid[m->flex_vertadr[con->flex[side]] + con->vert[side]];
          bweight[0] = 1;
          nb = 1;
        }

        // flex elem
        else {
          nb = mj_elemBodyWeight(m, d, con->flex[side], con->elem[side],
                                 con->vert[1-side], con->pos, bid, bweight);
        }

        // add weighted average over bodies
        for (int k=0; k < nb; k++) {
          tran += m->body_invweight0[2*bid[k]] * bweight[k];
          rot += m->body_invweight0[2*bid[k]+1] * bweight[k];
        }
      }

      // set frictionless
      if (d->efc_type[i] == mjCNSTR_CONTACT_FRICTIONLESS) {
        dA[i] = tran;
      }

      // set elliptical
      else if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
        for (int j=0; j < dim; j++) {
          dA[i+j] = (j < 3 ? tran : rot);
        }

        // processed dim elements in one i-loop iteration; advance counter
        i += (dim-1);
      }

      // set pyramidal
      else {
        for (int j=0; j < dim-1; j++) {
          fri = con->friction[j];
          dA[i+2*j] = dA[i+2*j+1] = tran + fri*fri*(j < 2 ? tran : rot);
        }

        // processed 2*dim-2 elements in one i-loop iteration; advance counter
        i += (2*dim-3);
      }
    }
  }
}



// get solref, solimp for specified constraint
static void getsolparam(const mjModel* m, const mjData* d, int i,
                        mjtNum* solref, mjtNum* solreffriction, mjtNum* solimp) {
  // get constraint id
  int id = d->efc_id[i];

  // clear solreffriction (applies only to contacts)
  mju_zero(solreffriction, mjNREF);

  // extract solver parameters from corresponding model element
  switch ((mjtConstraint) d->efc_type[i]) {
  case mjCNSTR_EQUALITY:
    mju_copy(solref, m->eq_solref+mjNREF*id, mjNREF);
    mju_copy(solimp, m->eq_solimp+mjNIMP*id, mjNIMP);
    break;

  case mjCNSTR_LIMIT_JOINT:
    mju_copy(solref, m->jnt_solref+mjNREF*id, mjNREF);
    mju_copy(solimp, m->jnt_solimp+mjNIMP*id, mjNIMP);
    break;

  case mjCNSTR_FRICTION_DOF:
    mju_copy(solref, m->dof_solref+mjNREF*id, mjNREF);
    mju_copy(solimp, m->dof_solimp+mjNIMP*id, mjNIMP);
    break;

  case mjCNSTR_LIMIT_TENDON:
    mju_copy(solref, m->tendon_solref_lim+mjNREF*id, mjNREF);
    mju_copy(solimp, m->tendon_solimp_lim+mjNIMP*id, mjNIMP);
    break;

  case mjCNSTR_FRICTION_TENDON:
    mju_copy(solref, m->tendon_solref_fri+mjNREF*id, mjNREF);
    mju_copy(solimp, m->tendon_solimp_fri+mjNIMP*id, mjNIMP);
    break;

  case mjCNSTR_CONTACT_FRICTIONLESS:
  case mjCNSTR_CONTACT_PYRAMIDAL:
  case mjCNSTR_CONTACT_ELLIPTIC:
    mju_copy(solref, d->contact[id].solref, mjNREF);
    mju_copy(solreffriction, d->contact[id].solreffriction, mjNREF);
    mju_copy(solimp, d->contact[id].solimp, mjNIMP);
  }

  // check reference format: standard or direct, cannot be mixed
  if ((solref[0] > 0) ^ (solref[1] > 0)) {
    mju_warning("mixed solref format, replacing with default");
    mj_defaultSolRefImp(solref, NULL);
  }

  // integrator safety: impose ref[0]>=2*timestep for standard format
  if (!mjDISABLED(mjDSBL_REFSAFE) && solref[0] > 0) {
    solref[0] = mju_max(solref[0], 2*m->opt.timestep);
  }

  // check reference format: standard or direct, cannot be mixed
  if ((solreffriction[0] > 0) ^ (solreffriction[1] > 0)) {
    mju_warning("solreffriction values should have the same sign, replacing with default");
    mju_zero(solreffriction, mjNREF);  // default solreffriction is (0, 0)
  }

  // integrator safety: impose ref[0]>=2*timestep for standard format
  if (!mjDISABLED(mjDSBL_REFSAFE) && solreffriction[0] > 0) {
    solreffriction[0] = mju_max(solreffriction[0], 2*m->opt.timestep);
  }

  // enforce constraints on solimp
  solimp[0] = mju_min(mjMAXIMP, mju_max(mjMINIMP, solimp[0]));
  solimp[1] = mju_min(mjMAXIMP, mju_max(mjMINIMP, solimp[1]));
  solimp[2] = mju_max(0, solimp[2]);
  solimp[3] = mju_min(mjMAXIMP, mju_max(mjMINIMP, solimp[3]));
  solimp[4] = mju_max(1, solimp[4]);
}



// get pos and dim for specified constraint
static void getposdim(const mjModel* m, const mjData* d, int i, mjtNum* pos, int* dim) {
  // get id of constraint-related object
  int id = d->efc_id[i];

  // set (dim, pos) for common case
  *dim = 1;
  *pos = d->efc_pos[i];

  // change (dim, distance) for special cases
  switch ((mjtConstraint) d->efc_type[i]) {
  case mjCNSTR_CONTACT_ELLIPTIC:
    *dim = d->contact[id].dim;
    break;

  case mjCNSTR_CONTACT_PYRAMIDAL:
    *dim = 2*(d->contact[id].dim-1);
    break;

  case mjCNSTR_EQUALITY:
    if (m->eq_type[id] == mjEQ_WELD) {
      *dim = 6;
      *pos = mju_norm(d->efc_pos+i, 6);
    } else if (m->eq_type[id] == mjEQ_CONNECT) {
      *dim = 3;
      *pos = mju_norm(d->efc_pos+i, 3);
    }
    break;
  default:
    // already handled
    break;
  }
}



// return a to the power of b, quick return for powers 1 and 2
// solimp[4] == 2 is the default, so these branches are common
static mjtNum power(mjtNum a, mjtNum b) {
  if (b == 1) {
    return a;
  } else if (b == 2) {
    return a*a;
  }
  return mju_pow(a, b);
}



// compute impedance and derivative for one constraint
static void getimpedance(const mjtNum* solimp, mjtNum pos, mjtNum margin,
                         mjtNum* imp, mjtNum* impP) {
  // flat function
  if (solimp[0] == solimp[1] || solimp[2] <= mjMINVAL) {
    *imp = 0.5*(solimp[0] + solimp[1]);
    *impP = 0;
    return;
  }

  // x = abs((pos-margin) / width)
  mjtNum x = (pos-margin) / solimp[2];
  mjtNum sgn = 1;
  if (x < 0) {
    x = -x;
    sgn = -1;
  }

  // fully saturated
  if (x >= 1 || x <= 0) {
    *imp = (x >= 1 ? solimp[1] : solimp[0]);
    *impP = 0;
    return;
  }

  // linear
  mjtNum y, yP;
  if (solimp[4] == 1) {
    y = x;
    yP = 1;
  }

  // y(x) = a*x^p if x<=midpoint
  else if (x <= solimp[3]) {
    mjtNum a = 1/power(solimp[3], solimp[4]-1);
    y = a*power(x, solimp[4]);
    yP = solimp[4] * a*power(x, solimp[4]-1);
  }

  // y(x) = 1-b*(1-x)^p is x>midpoint
  else {
    mjtNum b = 1/power(1-solimp[3], solimp[4]-1);
    y = 1-b*power(1-x, solimp[4]);
    yP = solimp[4] * b*power(1-x, solimp[4]-1);
  }

  // scale
  *imp = solimp[0] + y*(solimp[1]-solimp[0]);
  *impP = yP * sgn * (solimp[1]-solimp[0]) / solimp[2];
}



// compute efc_R, efc_D, efc_KBIP, adjust efc_diagApprox
void mj_makeImpedance(const mjModel* m, mjData* d) {
  int dim, nefc = d->nefc;
  mjtNum *R = d->efc_R, *KBIP = d->efc_KBIP;
  mjtNum pos, imp, impP, Rpy, solref[mjNREF], solreffriction[mjNREF], solimp[mjNIMP];

  // set efc_R, efc_KBIP
  for (int i=0; i < nefc; i++) {
    // get solref and solimp
    getsolparam(m, d, i, solref, solreffriction, solimp);

    // get pos and dim
    getposdim(m, d, i, &pos, &dim);

    // get imp and impP
    getimpedance(solimp, pos, d->efc_margin[i], &imp, &impP);

    // set R and KBIP for all constraint dimensions
    for (int j=0; j < dim; j++) {
      // R = (1-imp)/imp * diagApprox
      R[i+j] = mju_max(mjMINVAL, (1-imp)*d->efc_diagApprox[i+j]/imp);

      // constraint type
      int tp = d->efc_type[i+j];

      // elliptic contacts use solreffriction in non-normal directions, if non-zero
      int elliptic_friction = (tp == mjCNSTR_CONTACT_ELLIPTIC) && (j > 0);
      mjtNum* ref = elliptic_friction && (solreffriction[0] || solreffriction[1]) ?
                    solreffriction : solref;

      // friction: K = 0
      if (tp == mjCNSTR_FRICTION_DOF || tp == mjCNSTR_FRICTION_TENDON || elliptic_friction) {
        KBIP[4*(i+j)] = 0;
      }

      // standard: K = 1 / (dmax^2 * timeconst^2 * dampratio^2)
      else if (ref[0] > 0)
        KBIP[4*(i+j)] = 1 / mju_max(mjMINVAL, solimp[1]*solimp[1] * ref[0]*ref[0] * ref[1]*ref[1]);

      // direct: K = -solref[0] / dmax^2
      else {
        KBIP[4*(i+j)] = -ref[0] / mju_max(mjMINVAL, solimp[1]*solimp[1]);
      }

      // standard: B = 2 / (dmax*timeconst)
      if (ref[1] > 0) {
        KBIP[4*(i+j)+1] = 2 / mju_max(mjMINVAL, solimp[1]*ref[0]);
      }

      // direct: B = -solref[1] / dmax
      else {
        KBIP[4*(i+j)+1] = -ref[1] / mju_max(mjMINVAL, solimp[1]);
      }

      // I = imp, P = imp'
      KBIP[4*(i+j)+2] = imp;
      KBIP[4*(i+j)+3] = impP;
    }

    // skip the rest of this constraint
    i += (dim-1);
  }

  // frictional contacts: adjust R in friction dimensions, set contact master mu
  for (int i=d->ne+d->nf; i < nefc; i++) {
    if (d->efc_type[i] == mjCNSTR_CONTACT_PYRAMIDAL ||
        d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
      // extract id, dim, mu
      int id = d->efc_id[i];
      dim = d->contact[id].dim;
      mjtNum* friction = d->contact[id].friction;

      // set R[1] = R[0]/impratio
      R[i+1] = R[i]/mju_max(mjMINVAL, m->opt.impratio);

      // set mu of regularized cone = mu[1]*sqrt(R[1]/R[0])
      d->contact[id].mu = friction[0] * mju_sqrt(R[i+1]/R[i]);

      // elliptic
      if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
        // set remaining R's such that R[j]*mu[j]^2 = R[1]*mu[1]^2
        for (int j=1; j < dim-1; j++) {
          R[i+j+1] = R[i+1]*friction[0]*friction[0]/(friction[j]*friction[j]);
        }

        // skip the rest of this contact
        i += (dim-1);
      }

      // pyramidal: common R matching friction impedance of elliptic model
      else {
        // D0_el = 2*(dim-1)*D_py : normal match
        // D0_el = 2*mu^2*D_py    : friction match
        Rpy = 2*d->contact[id].mu*d->contact[id].mu*R[i];

        // assign Rpy to all pyramidal R
        for (int j=0; j < 2*(dim-1); j++) {
          R[i+j] = Rpy;
        }

        // skip the rest of this contact
        i += 2*(dim-1) - 1;
      }
    }
  }

  // set D = 1 / R
  for (int i=0; i < nefc; i++) {
    d->efc_D[i] = 1 / R[i];
  }

  // adjust diagApprox so that R = (1-imp)/imp * diagApprox
  for (int i=0; i < nefc; i++) {
    d->efc_diagApprox[i] = R[i] * KBIP[4*i+2] / (1-KBIP[4*i+2]);
  }
}



//------------------------------------- constraint counting ----------------------------------------

// count the non-zero columns in the Jacobian difference of two bodies
static int mj_jacDifPairCount(const mjModel* m, int* chain,
                              int b1, int b2, int issparse) {
  if (!m->nv) {
    return 0;
  }

  if (issparse) {
    if (m->body_simple[b1] && m->body_simple[b2]) {
      return mj_mergeChainSimple(m, chain, b1, b2);
    }
    return mj_mergeChain(m, chain, b1, b2);
  }

  return m->nv;
}



// count the non-zero columns of the Jacobian returned by mj_jacSum
static int mj_jacSumCount(const mjModel* m, mjData* d, int* chain,
                          int n, const int* body) {
  int nv = m->nv, NV;

  mj_markStack(d);
  int* bodychain = mjSTACKALLOC(d, nv, int);
  int* tempchain = mjSTACKALLOC(d, nv, int);

  // set first
  NV = mj_bodyChain(m, body[0], chain);

  // accumulate remaining
  for (int i=1; i < n; i++) {
    // get body chain
    int bodyNV = mj_bodyChain(m, body[i], bodychain);
    if (!bodyNV) {
      continue;
    }

    // accumulate chains
    NV = mju_addChains(tempchain, nv, NV, bodyNV, chain, bodychain);
    if (NV) {
      mju_copyInt(chain, tempchain, NV);
    }
  }

  mj_freeStack(d);
  return NV;
}



// return number of constraint non-zeros, handle dense and dof-less cases
static inline int mj_addConstraintCount(const mjModel* m, int size, int NV) {
  // over count for dense allocation
  if (!mj_isSparse(m)) {
    return m->nv ? size : 0;
  }
  return mjMAX(0, NV) ? size : 0;
}



// count equality constraints, count Jacobian nonzeros if nnz is not NULL
static int mj_ne(const mjModel* m, mjData* d, int* nnz) {
  int ne = 0, nnze = 0;
  int nv = m->nv, neq = m->neq;
  int id[2], size, NV, NV2, *chain = NULL, *chain2 = NULL;
  int issparse = (nnz != NULL);
  int flex_edgeadr, flex_edgenum;

  // disabled or no equality constraints: return
  if (mjDISABLED(mjDSBL_EQUALITY) || m->nemax == 0) {
    return 0;
  }

  mj_markStack(d);

  if (nnz) {
    chain = mjSTACKALLOC(d, nv, int);
    chain2 = mjSTACKALLOC(d, nv, int);
  }

  // find active equality constraints
  for (int i=0; i < neq; i++) {
    if (d->eq_active[i]) {
      id[0] = m->eq_obj1id[i];
      id[1] = m->eq_obj2id[i];
      size = 0;
      NV = 0;
      NV2 = 0;

      // process according to type
      switch ((mjtEq) m->eq_type[i]) {
      case mjEQ_CONNECT:
        size = 3;
        if (!nnz) {
          break;
        }

        // get body ids if using site semantics
        if (m->eq_objtype[i] == mjOBJ_SITE) {
          id[0] = m->site_bodyid[id[0]];
          id[1] = m->site_bodyid[id[1]];
        }

        NV = mj_jacDifPairCount(m, chain, id[1], id[0], issparse);
        break;

      case mjEQ_WELD:
        size = 6;
        if (!nnz) {
          break;
        }

        // get body ids if using site semantics
        if (m->eq_objtype[i] == mjOBJ_SITE) {
          id[0] = m->site_bodyid[id[0]];
          id[1] = m->site_bodyid[id[1]];
        }

        NV = mj_jacDifPairCount(m, chain, id[1], id[0], issparse);
        break;

      case mjEQ_JOINT:
      case mjEQ_TENDON:
        size = 1;
        if (!nnz) {
          break;
        }

        for (int j=0; j < 1+(id[1] >= 0); j++) {
          if (m->eq_type[i] == mjEQ_JOINT) {
            if (!j) {
              NV = 1;
              chain[0] = m->jnt_dofadr[id[j]];
            } else {
              NV2 = 1;
              chain2[0] = m->jnt_dofadr[id[j]];
            }
          } else {
            if (!j) {
              NV = d->ten_J_rownnz[id[j]];
              mju_copyInt(chain, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV);
            } else {
              NV2 = d->ten_J_rownnz[id[j]];
              mju_copyInt(chain2, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV2);
            }
          }
        }

        if (id[1] >= 0) {
          NV = mju_combineSparseCount(NV, NV2, chain, chain2);
        }
        break;

      case mjEQ_FLEX:
        flex_edgeadr = m->flex_edgeadr[id[0]];
        flex_edgenum = m->flex_edgenum[id[0]];

        // init with all edges, subract rigid later
        size = flex_edgenum;

        // process edges of this flex
        for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
          // rigid: reduce size and skip
          if (m->flexedge_rigid[e]) {
            size--;
            continue;
          }

          // accumulate NV if needed
          if (nnz) {
            int b1 = m->flex_vertbodyid[m->flex_vertadr[id[0]] + m->flex_edge[2*e]];
            int b2 = m->flex_vertbodyid[m->flex_vertadr[id[0]] + m->flex_edge[2*e+1]];
            NV += mj_jacDifPairCount(m, chain, b1, b2, issparse);
          }
        }
        break;

      default:
        // might occur in case of the now-removed distance equality constraint
        mjERROR("unknown constraint type type %d", m->eq_type[i]);    // SHOULD NOT OCCUR
      }

      // accumulate counts; flex NV already accumulated
      ne += mj_addConstraintCount(m, size, NV);
      nnze += (m->eq_type[i] == mjEQ_FLEX) ? NV : size*NV;
    }
  }

  if (nnz) {
    *nnz += nnze;
  }

  mj_freeStack(d);
  return ne;
}



// count frictional constraints, count Jacobian nonzeros if nnz is not NULL
static int mj_nf(const mjModel* m, const mjData* d, int *nnz) {
  int nf = 0;
  int nv = m->nv, ntendon = m->ntendon;

  if (mjDISABLED(mjDSBL_FRICTIONLOSS)) {
    return 0;
  }

  for (int i=0; i < nv; i++) {
    if (m->dof_frictionloss[i] > 0) {
      nf += mj_addConstraintCount(m, 1, 1);
      if (nnz) *nnz += 1;
    }
  }

  for (int i=0; i < ntendon; i++) {
    if (m->tendon_frictionloss[i] > 0) {
      nf += mj_addConstraintCount(m, 1, d->ten_J_rownnz[i]);
      if (nnz) *nnz += d->ten_J_rownnz[i];
    }
  }

  return nf;
}



// count limit constraints, count Jacobian nonzeros if nnz is not NULL
static int mj_nl(const mjModel* m, const mjData* d, int *nnz) {
  int nl = 0;
  int ntendon = m->ntendon;
  int side;
  mjtNum margin, value, dist;

  // disabled: return
  if (mjDISABLED(mjDSBL_LIMIT)) {
    return 0;
  }


  for (int i=0; i < m->njnt; i++) {
    if (!m->jnt_limited[i]) {
      continue;
    }

    margin = m->jnt_margin[i];

    // slider and hinge joint limits can be bilateral, check both sides
    if (m->jnt_type[i] == mjJNT_SLIDE || m->jnt_type[i] == mjJNT_HINGE) {
      value = d->qpos[m->jnt_qposadr[i]];
      for (side=-1; side <= 1; side+=2) {
        dist = side * (m->jnt_range[2*i+(side+1)/2] - value);
        if (dist < margin) {
          nl += mj_addConstraintCount(m, 1, 1);
          if (nnz) *nnz += 1;
        }
      }
    }
    else if (m->jnt_type[i] == mjJNT_BALL) {
      mjtNum angleAxis[3];
      int adr = m->jnt_qposadr[i];
      mjtNum quat[4] = {d->qpos[adr], d->qpos[adr+1], d->qpos[adr+2], d->qpos[adr+3]};
      mju_normalize4(quat);
      mju_quat2Vel(angleAxis, quat, 1);
      value = mju_normalize3(angleAxis);
      dist = mju_max(m->jnt_range[2*i], m->jnt_range[2*i+1]) - value;
      if (dist < margin) {
        nl += mj_addConstraintCount(m, 1, 3);
        if (nnz) *nnz += 3;
      }
    }
  }

  for (int i=0; i < ntendon; i++) {
    if (m->tendon_limited[i]) {
      value = d->ten_length[i];
      margin = m->tendon_margin[i];

      // tendon limits can be bilateral, check both sides
      for (side=-1; side <= 1; side+=2) {
        dist = side * (m->tendon_range[2*i+(side+1)/2] - value);
        if (dist < margin) {
          nl += mj_addConstraintCount(m, 1, d->ten_J_rownnz[i]);
          if (nnz) *nnz += d->ten_J_rownnz[i];
        }
      }
    }
  }

  return nl;
}



// count contact constraints, count Jacobian nonzeros if nnz is not NULL
static int mj_nc(const mjModel* m, mjData* d, int* nnz) {
  int nnzc = 0, nc = 0;
  int ispyramid = mj_isPyramidal(m), ncon = d->ncon;

  if (mjDISABLED(mjDSBL_CONTACT) || !ncon) {
    return 0;
  }

  mj_markStack(d);
  int *chain = mjSTACKALLOC(d, m->nv, int);

  for (int i=0; i < ncon; i++) {
    mjContact* con = d->contact + i;

    // skip if excluded
    if (con->exclude) {
      continue;
    }

    // compute NV only if nnz requested
    int NV = 0;
    if (nnz) {
      // get bodies
      int nb = 0, bid[8];
      for (int side=0; side < 2; side++) {
        // geom
        if (con->geom[side] >= 0) {
          bid[nb++] = m->geom_bodyid[con->geom[side]];
        }

        // flex vert
        else if (con->vert[side] >= 0) {
          bid[nb++] = m->flex_vertbodyid[m->flex_vertadr[con->flex[side]] + con->vert[side]];
        }

        // flex elem
        else {
          int f = con->flex[side];
          int fdim = m->flex_dim[f];
          const int* edata = m->flex_elem + m->flex_elemdataadr[f] + con->elem[side]*(fdim+1);
          for (int k=0; k <= fdim; k++) {
            bid[nb++] = m->flex_vertbodyid[m->flex_vertadr[f] + edata[k]];
          }
        }
      }

      // count non-zeros in merged chain
      NV = mj_jacSumCount(m, d, chain, nb, bid);
      if (!NV) {
        continue;
      }
    }

    // count according to friction type
    int dim = con->dim;
    if (dim == 1) {
      nc++;
      nnzc += NV;
    } else if (ispyramid) {
      nc += 2*(dim-1);
      nnzc += 2*(dim-1)*NV;
    } else {
      nc += dim;
      nnzc += dim*NV;
    }
  }

  if (nnz) {
    *nnz += nnzc;
  }

  mj_freeStack(d);
  return nc;
}



//---------------------------- top-level API for constraint construction ---------------------------

// driver: call all functions above
void mj_makeConstraint(const mjModel* m, mjData* d) {
  // clear sizes
  d->ne = d->nf = d->nl = d->nefc = d->nJ = 0;

  // disabled or Jacobian not allocated: return
  if (mjDISABLED(mjDSBL_CONSTRAINT)) {
    return;
  }

  // precount sizes for constraint Jacobian matrices
  int *nnz = mj_isSparse(m) ? &(d->nJ) : NULL;
  int ne_allocated = mj_ne(m, d, nnz);
  int nf_allocated = mj_nf(m, d, nnz);
  int nl_allocated = mj_nl(m, d, nnz);
  int nefc_allocated = ne_allocated + nf_allocated + nl_allocated + mj_nc(m, d, nnz);
  if (!mj_isSparse(m)) {
    d->nJ = nefc_allocated * m->nv;
  }
  d->nefc = nefc_allocated;

  // allocate efc arrays on arena
  if (!arenaAllocEfc(m, d)) {
    return;
  }

  // clear tendon_efcadr
  for (int i=0; i < m->ntendon; i++) {
    d->tendon_efcadr[i] = -1;
  }

  // reset nefc for the instantiation functions,
  // and instantiate all elements of Jacobian
  d->nefc = 0;
  mj_instantiateEquality(m, d);
  mj_instantiateFriction(m, d);
  mj_instantiateLimit(m, d);
  mj_instantiateContact(m, d);

  // check sparse allocation
  if (mj_isSparse(m)) {
    if (d->ne != ne_allocated) {
      mjERROR("ne mis-allocation: found ne=%d but allocated %d", d->ne, ne_allocated);
    }

    if (d->nf != nf_allocated) {
      mjERROR("nf mis-allocation: found nf=%d but allocated %d", d->nf, nf_allocated);
    }

    if (d->nl != nl_allocated) {
      mjERROR("nl mis-allocation: found nl=%d but allocated %d", d->nl, nl_allocated);
    }

    // check that nefc was computed correctly
    if (d->nefc != nefc_allocated) {
      mjERROR("nefc mis-allocation: found nefc=%d but allocated %d", d->nefc, nefc_allocated);
    }

    // check that nJ was computed correctly
    if (d->nefc > 0) {
      int nJ = d->efc_J_rownnz[d->nefc - 1] + d->efc_J_rowadr[d->nefc - 1];
      if (d->nJ != nJ) {
        mjERROR("constraint Jacobian mis-allocation: found nJ=%d but allocated %d", nJ, d->nJ);
      }
    }
  } else if (d->nefc > nefc_allocated) {
    mjERROR("nefc under-allocation: found nefc=%d but allocated only %d",
            d->nefc, nefc_allocated);
  }

  // collect memory use statistics
  d->maxuse_con = mjMAX(d->maxuse_con, d->ncon);
  d->maxuse_efc = mjMAX(d->maxuse_efc, d->nefc);

  // no constraints: return
  if (!d->nefc) {
    return;
  }

  // transpose sparse Jacobian, make row supernodes
  if (mj_isSparse(m)) {
    // transpose
    mju_transposeSparse(d->efc_JT, d->efc_J, d->nefc, m->nv,
                        d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind,
                        d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);


#ifdef mjUSEAVX
    // compute supernodes of J; used by mju_mulMatVecSparse_avx
    mju_superSparse(d->nefc, d->efc_J_rowsuper,
                    d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
#else
  #ifdef MEMORY_SANITIZER
    // tell msan to treat the entire J rowsuper as uninitialized
    __msan_allocated_memory(d->efc_J_rowsuper, d->nefc);
  #endif  // MEMORY_SANITIZER
#endif  // mjUSEAVX

    // supernodes of JT
    mju_superSparse(m->nv, d->efc_JT_rowsuper,
                    d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind);
  } else {
    if (mjENABLED(mjENBL_ISLAND)) {
      mju_transpose(d->efc_JT, d->efc_J, d->nefc, m->nv);
    }
  }

  // compute diagApprox
  mj_diagApprox(m, d);

  // compute KBIP, D, R, adjust diagApprox
  mj_makeImpedance(m, d);
}



// compute efc_AR
void mj_projectConstraint(const mjModel* m, mjData* d) {
  int nefc = d->nefc, nv = m->nv;

  // nothing to do
  if (nefc == 0 || !mj_isDual(m)) {
    return;
  }

  mj_markStack(d);

  // space for backsubM2(J')' and its traspose
  mjtNum* JM2 = mjSTACKALLOC(d, nefc*nv, mjtNum);
  mjtNum* JM2T = mjSTACKALLOC(d, nv*nefc, mjtNum);

  // sparse
  if (mj_isSparse(m)) {
    // space for JM2 and JM2T indices
    int* rownnz = mjSTACKALLOC(d, nefc, int);
    int* rowadr = mjSTACKALLOC(d, nefc, int);
    int* colind = mjSTACKALLOC(d, nefc*nv, int);
    int* rowsuper = mjSTACKALLOC(d, nefc, int);
    int* rownnzT = mjSTACKALLOC(d, nv, int);
    int* rowadrT = mjSTACKALLOC(d, nv, int);
    int* colindT = mjSTACKALLOC(d, nv*nefc, int);

    // construct JM2 = backsubM2(J')' by rows
    for (int r=0; r < nefc; r++) {
      // init row
      int nnz = 0;
      int adr = (r > 0 ? rowadr[r-1]+rownnz[r-1] : 0);
      int remain = d->efc_J_rownnz[r];

      // complete chain in reverse
      while (1) {
        // assign row descriptor
        rownnz[r] = nnz;
        rowadr[r] = adr;

        // get previous dof in src and dst
        int prev_src = (remain > 0 ? d->efc_J_colind[d->efc_J_rowadr[r]+remain-1] : -1);
        int prev_dst = (nnz > 0 ? m->dof_parentid[colind[adr+nnz-1]] : -1);

        // both finished: break
        if (prev_src < 0 && prev_dst < 0) {
          break;
        }

        // add src
        else if (prev_src >= prev_dst) {
          colind[adr+nnz] = prev_src;
          JM2[adr+nnz] = d->efc_J[d->efc_J_rowadr[r]+remain-1];
          remain--;
          nnz++;
        }

        // add dst
        else {
          colind[adr+nnz] = prev_dst;
          JM2[adr+nnz] = 0;
          nnz++;
        }
      }

      // reverse order of chain: make it increasing
      for (int i=0; i < nnz/2; i++) {
        int tmp_col = colind[adr+i];
        colind[adr+i] = colind[adr+nnz-i-1];
        colind[adr+nnz-i-1] = tmp_col;

        mjtNum tmp_dat = JM2[adr+i];
        JM2[adr+i] = JM2[adr+nnz-i-1];
        JM2[adr+nnz-i-1] = tmp_dat;
      }

      // sparse backsubM2
      for (int i=nnz-1; i >= 0; i--) {
        // save x(i) and i-pointer
        mjtNum xi = JM2[adr+i];
        int pi = i;

        // process if not zero
        if (xi) {
          // x(i) /= sqrt(L(i,i))
          JM2[adr+i] *= d->qLDiagSqrtInv[colind[adr+i]];

          // x(j) -= L(i,j) * x(i)
          int Madr_ij = m->dof_Madr[colind[adr+i]]+1;
          int j = m->dof_parentid[colind[adr+i]];
          while (j >= 0) {
            // match dof id in sparse vector
            while (colind[adr+pi] > j) {
              pi--;
            }

            // scale
            JM2[adr+pi] -= d->qLD[Madr_ij++] * xi;

            // advance to parent
            j = m->dof_parentid[j];
          }
        }
      }
    }

    // construct JM2T
    mju_transposeSparse(JM2T, JM2, nefc, nv,
                        rownnzT, rowadrT, colindT, rownnz, rowadr, colind);

    // construct supernodes
    mju_superSparse(nefc, rowsuper, rownnz, rowadr, colind);

    // AR = JM2 * JM2'
    mju_sqrMatTDSparseInit(d->efc_AR_rownnz, d->efc_AR_rowadr, nefc, rownnzT,
                           rowadrT, colindT, rownnz, rowadr, colind, rowsuper, d);

    mju_sqrMatTDSparse(d->efc_AR, JM2T, JM2, NULL, nv, nefc,
                       d->efc_AR_rownnz, d->efc_AR_rowadr, d->efc_AR_colind,
                       rownnzT, rowadrT, colindT, NULL,
                       rownnz, rowadr, colind, rowsuper, d);

    // add R to diagonal of AR
    for (int i=0; i < nefc; i++) {
      for (int j=0; j < d->efc_AR_rownnz[i]; j++) {
        if (i == d->efc_AR_colind[d->efc_AR_rowadr[i]+j]) {
          d->efc_AR[d->efc_AR_rowadr[i]+j] += d->efc_R[i];
          break;
        }
      }
    }
  }

  // dense
  else {
    // JM2 = backsubM2(J')'
    mj_solveM2(m, d, JM2, d->efc_J, nefc);

    // construct JM2T
    mju_transpose(JM2T, JM2, nefc, nv);

    // AR = JM2 * JM2'
    mju_sqrMatTD(d->efc_AR, JM2T, NULL, nv, nefc);

    // add R to diagonal of AR
    for (int r=0; r < nefc; r++) {
      d->efc_AR[r*(nefc+1)] += d->efc_R[r];
    }
  }

  mj_freeStack(d);
}



// compute efc_vel, efc_aref
void mj_referenceConstraint(const mjModel* m, mjData* d) {
  int nefc = d->nefc;
  mjtNum* KBIP = d->efc_KBIP;

  // compute efc_vel
  mj_mulJacVec(m, d, d->efc_vel, d->qvel);

  // compute aref = -B*vel - K*I*(pos-margin)
  for (int i=0; i < nefc; i++) {
    d->efc_aref[i] = -KBIP[4*i+1]*d->efc_vel[i]
                     -KBIP[4*i]*KBIP[4*i+2]*(d->efc_pos[i]-d->efc_margin[i]);
  }
}



//---------------------------- update constraint state ---------------------------------------------

// compute efc_state, efc_force, qfrc_constraint, optionally restricted to one island
//  island < 0: update all d->nefc constraints
//  island >= 0: update only d->island_efcnum[island] constraints
//  jar = Jac*qacc-aref is restricted to the island, in the above sense
//  optional: cost(qacc) = shat(jar); cone Hessians
void mj_constraintUpdate_island(const mjModel* m, mjData* d, const mjtNum* jar,
                                mjtNum cost[1], int flg_coneHessian, int island) {
  int ne = d->ne, nf = d->nf;
  const mjtNum *D = d->efc_D, *R = d->efc_R, *floss = d->efc_frictionloss;
  mjtNum* force = d->efc_force;
  mjtNum s = 0;

  int nefc    = island < 0 ? d->nefc : d->island_efcnum[island];
  int* efcind = island < 0 ? NULL    : d->island_efcind + d->island_efcadr[island];

  // no constraints: clear qfrc_constraint and cost, return
  if (!nefc) {
    // can only occur for island == -1
    mju_zero(d->qfrc_constraint, m->nv);
    if (cost) {
      *cost = 0;
    }
    return;
  }

  // compute unconstrained efc_force
  for (int c=0; c < nefc; c++) {
    int i = efcind ? efcind[c] : c;
    force[i] = -D[i]*jar[c];
  }

  // update constraints
  for (int c=0; c < nefc; c++) {
    int i = efcind ? efcind[c] : c;

    // ==== equality
    if (i < ne) {
      if (cost) {
        s += 0.5*D[i]*jar[c]*jar[c];
      }
      d->efc_state[i] = mjCNSTRSTATE_QUADRATIC;
      continue;
    }

    // ==== friction
    if (i < ne + nf) {
      // linear negative
      if (jar[c] <= -R[i]*floss[i]) {
        if (cost) {
          s += -0.5*R[i]*floss[i]*floss[i] - floss[i]*jar[c];
        }

        force[i] = floss[i];

        d->efc_state[i] = mjCNSTRSTATE_LINEARNEG;
      }

      // linear positive
      else if (jar[c] >= R[i]*floss[i]) {
        if (cost) {
          s += -0.5*R[i]*floss[i]*floss[i] + floss[i]*jar[c];
        }

        force[i] = -floss[i];

        d->efc_state[i] = mjCNSTRSTATE_LINEARPOS;
      }

      // quadratic
      else {
        if (cost) {
          s += 0.5*D[i]*jar[c]*jar[c];
        }

        d->efc_state[i] = mjCNSTRSTATE_QUADRATIC;
      }
      continue;
    }

    // ==== contact

    // non-negative constraint
    if (d->efc_type[i] != mjCNSTR_CONTACT_ELLIPTIC) {
      // constraint is satisfied: no cost
      if (jar[c] >= 0) {
        force[i] = 0;

        d->efc_state[i] = mjCNSTRSTATE_SATISFIED;
      }

      // quadratic
      else {
        if (cost) {
          s += 0.5*D[i]*jar[c]*jar[c];
        }

        d->efc_state[i] = mjCNSTRSTATE_QUADRATIC;
      }
    }

    // contact with elliptic cone
    else {
      // get contact
      mjContact* con = d->contact + d->efc_id[i];
      mjtNum mu = con->mu, *friction = con->friction;
      int dim = con->dim;

      // map to regular dual cone space
      mjtNum U[6];
      U[0] = jar[c]*mu;
      for (int j=1; j < dim; j++) {
        U[j] = jar[c+j]*friction[j-1];
      }

      // decompose into normal and tangent
      mjtNum N = U[0];
      mjtNum T = mju_norm(U+1, dim-1);

      // top zone
      if (N >= mu*T || (T <= 0 && N >= 0)) {
        mju_zero(force+i, dim);

        d->efc_state[i] = mjCNSTRSTATE_SATISFIED;
      }

      // bottom zone
      else if (mu*N+T <= 0 || (T <= 0 && N < 0)) {
        if (cost) {
          for (int j=0; j < dim; j++) {
            s += 0.5*D[i+j]*jar[c+j]*jar[c+j];
          }
        }

        d->efc_state[i] = mjCNSTRSTATE_QUADRATIC;
      }

      // middle zone
      else {
        // cost: 0.5*D0/(mu*mu*(1+mu*mu))*(N-mu*T)^2
        mjtNum Dm = D[i]/(mu*mu*(1+mu*mu));
        mjtNum NmT = N - mu*T;

        if (cost) {
          s += 0.5*Dm*NmT*NmT;
        }

        // force: - ds/djar = dU/djar * ds/dU  (dU/djar = diag(mu, friction))
        force[i] = -Dm*NmT*mu;
        for (int j=1; j < dim; j++) {
          force[i+j] = -force[i]/T*U[j]*friction[j-1];
        }

        // set state
        d->efc_state[i] = mjCNSTRSTATE_CONE;

        // cone Hessian
        if (flg_coneHessian) {
          // get Hessian pointer
          mjtNum* H = d->contact[d->efc_id[i]].H;

          // set first row: (1, -mu/T * U)
          mjtNum scl = -mu/T;
          H[0] = 1;
          for (int j=1; j < dim; j++) {
            H[j] = scl*U[j];
          }

          // set upper block: mu*N/T^3 * U*U'
          scl = mu*N/(T*T*T);
          for (int k=1; k < dim; k++)
            for (int j=k; j < dim; j++) {
              H[k*dim+j] = scl*U[j]*U[k];
            }

          // add to diagonal: (mu^2 - mu*N/T) * I
          scl = mu*mu - mu*N/T;
          for (int j=1; j < dim; j++) {
            H[j*(dim+1)] += scl;
          }

          // pre and post multiply by diag(mu, friction), scale by Dm
          for (int k=0; k < dim; k++) {
            scl = Dm * (k == 0 ? mu : friction[k-1]);
            for (int j=k; j < dim; j++) {
              H[k*dim+j] *= scl * (j == 0 ? mu : friction[j-1]);
            }
          }

          // make symmetric: copy upper into lower
          for (int k=0; k < dim; k++) {
            for (int j=k+1; j < dim; j++) {
              H[j*dim+k] = H[k*dim+j];
            }
          }
        }
      }

      // replicate state in all cone dimensions
      for (int j=1; j < dim; j++) {
        d->efc_state[i+j] = d->efc_state[i];
      }

      // advance to end of contact
      c += (dim-1);
    }
  }

  // compute qfrc_constraint
  int flg_vecunc = 1;
  int flg_resunc = 1;
  mj_mulJacTVec_island(m, d, d->qfrc_constraint, d->efc_force, island, flg_vecunc, flg_resunc);

  // assign cost
  if (cost) {
    *cost = s;
  }
}



// compute efc_state, efc_force, qfrc_constraint
// optional: cost(qacc) = shat(jar) where jar = Jac*qacc-aref; cone Hessians
void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar,
                         mjtNum cost[1], int flg_coneHessian) {
  mj_constraintUpdate_island(m, d, jar, cost, flg_coneHessian, -1);
}
