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

#include "engine/engine_metric.h"

#include <float.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjtype.h>
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_derivative.h"
#include "engine/engine_memory.h"
#include "engine/engine_sleep.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"



//------------------- implicit effective metric Mtilde = M + h*D + h^2*K --------------------------
// The flex part is K = (h^2 + h*damping) * (K_bend + K_stretch), the PSD implicit flex stiffness,
// whose h^2 and h*damping parts enter only when the spring and damper forces are enabled;
// the diagonal part holds joint damping and stiffness (h*D + h^2*K per dof, clamped PSD). Built
// once per step on the arena by mj_effBuild (under integrator=discrete), then consumed
// uniformly: the smooth acceleration, the constraint solver and inverse dynamics all see the
// same metric.

// arena allocation with hard failure (mirrors stack overflow semantics)
static void* effAlloc(mjData* d, size_t bytes, size_t align) {
  void* p = mj_arenaAllocByte(d, bytes, align);
  if (!p) {
    mjERROR("arena overflow in implicit effective metric");
  }
  return p;
}
#define EFMALLOC(type, n) (type*) effAlloc(d, sizeof(type)*(size_t)(n), _Alignof(type))

// torsion term of hinge h of 1D DER rod f: scale * (v[dof1] - v[dof0])^2 on the twist dofs of the
// hinge's two edges (-1: clamped edge), scale = (s1 + s2*damping) * 2*kt; returns scale, 0 if none.
// The one definition of the twist chain's off-diagonals for the operator (class 3), the shift and
// effTwistSolve
static mjtNum effTwistHinge(const mjModel* m, int f, int h, mjtNum s1, mjtNum s2,
                            int* dof0, int* dof1) {
  const mjtNum* hinge = m->flex_bending + m->flex_bendingadr[f] + 1 + mjROD_HINGE_SIZE*h;
  mjtNum scale = (s1 + s2*mj_effFlexDamping(m, f)) * 2 * hinge[8];
  *dof0 = mj_rodChainTwistDof(m, f, h);
  *dof1 = mj_rodChainTwistDof(m, f, h + 1);
  if (scale <= 0 || (*dof0 < 0 && *dof1 < 0)) {
    return 0;
  }
  return scale;
}

// yield the next live rank-1 term of the metric; see the declaration for the contract
int mj_effRank1Next(const mjModel* m, const mjData* d, mjEffRank1Iter* it,
                    mjEffRank1* e, int flg_contact) {
  // class 0: one entry per tendon with metric terms
  while (it->cls == 0) {
    if (it->i >= d->nefmT) {
      it->cls = 1;
      it->i = 0;
      break;
    }
    int t = it->i++;
    mjtNum s = d->efm_ts[t];
    int id = d->efm_tid[t];
    int nnz = m->ten_J_rownnz[id];
    if (!s || !nnz) {
      continue;
    }
    e->val = d->ten_J + m->ten_J_rowadr[id];
    e->colind = m->ten_J_colind + m->ten_J_rowadr[id];
    e->nnz = nnz;
    e->scale = s;
    return 1;
  }

  // class 1: one entry per output row of each actuator with metric terms
  while (it->cls == 1) {
    if (it->i >= d->nefmA) {
      it->cls = 2;
      it->i = 0;
      it->k = 0;  // class 2 walks the packed rows by offset
      break;
    }
    int id = d->efm_aid[it->i];
    if (it->k >= m->actuator_outnum[id]) {
      it->i++;
      it->k = 0;
      continue;
    }
    mjtNum s = d->efm_as[it->i];
    int r = m->actuator_outadr[id] + it->k++;
    int nnz = d->moment_rownnz[r];
    if (!s || !nnz) {
      continue;
    }
    e->val = d->actuator_moment + d->moment_rowadr[r];
    e->colind = d->moment_colind + d->moment_rowadr[r];
    e->nnz = nnz;
    e->scale = s;
    return 1;
  }

  // class 2: one entry per passive flex contact, published by effContactBuild. Skipped
  // wholesale when the caller accounts for contact separately
  while (it->cls == 2) {
    if (!flg_contact || it->k >= d->nefmcon) {
      it->cls = 3;
      it->i = 0;
      it->k = 0;
      break;
    }
    int adr = it->k, nnz = d->efm_con_ind[adr];   // packed row: header then entries
    e->nnz = nnz;
    e->scale = d->efm_con_val[adr];
    e->colind = d->efm_con_ind + adr + 2;
    e->val = d->efm_con_val + adr + 2;
    it->k = adr + 2 + nnz;
    return 1;
  }

  // class 3: one entry per torsion hinge of 1D DER rods (tridiagonal coupling on per-edge twist
  // DOFs). The metric keeps the diagonal blocks of the rod's Gauss-Newton torsion term
  // 2kt (v; j_psi)(v; j_psi)': the twist-twist block 2kt v v' here (plus the bending stiffness of
  // each twist angle, a per-dof diagonal in efm_diag, see effDiagStiff) and the vertex-vertex
  // block 2kt j_psi' j_psi in the flex stiffness (rodHingeTorsionRow), omitting the cross blocks
  // 2kt v j_psi'. The result stays PSD, keeps every twist chain tridiagonal and in its own trees
  // (exact O(nedge) effTwistSolve), and over-stiffens only the zero-energy mode where the twist
  // angles cancel a change of reference twist (twist/writhe exchange), which is thus damped more
  // under discrete than the exact Hessian would
  if (it->cls == 3) {
    int passive = !(mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER));
    if (!passive || !mj_effCouplings(m)) {
      it->cls = 4;
      return 0;
    }
    mjtNum h_dt = m->opt.timestep;
    mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h_dt*h_dt;
    mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h_dt;
    int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
    while (it->i < m->nflex) {
      int f = it->i;
      if (it->k == 0 &&
          (m->flex_rigid[f] || !mj_rodHasTwist(m, f) ||
           (sleep_filter && mj_sleepState(m, d, mjOBJ_FLEX, f) != mjS_AWAKE))) {
        it->i++;
        continue;
      }
      int nhinge = (int)m->flex_bending[m->flex_bendingadr[f]];
      if (it->k >= nhinge) {
        it->i++;
        it->k = 0;
        continue;
      }
      int h = it->k++, dof0, dof1;
      mjtNum scale = effTwistHinge(m, f, h, s1, s2, &dof0, &dof1);
      if (!scale) {
        continue;
      }
      if (dof0 >= 0 && dof1 >= 0) {
        if (dof0 < dof1) {
          it->colind[0] = dof0;
          it->val[0] = -1;
          it->colind[1] = dof1;
          it->val[1] = 1;
        } else {
          it->colind[0] = dof1;
          it->val[0] = 1;
          it->colind[1] = dof0;
          it->val[1] = -1;
        }
        e->nnz = 2;
      } else if (dof1 >= 0) {
        it->colind[0] = dof1;
        it->val[0] = 1;
        e->nnz = 1;
      } else if (dof0 >= 0) {
        it->colind[0] = dof0;
        it->val[0] = -1;
        e->nnz = 1;
      } else {
        continue;
      }
      e->colind = it->colind;
      e->val = it->val;
      e->scale = scale;
      return 1;
    }
    it->cls = 4;
  }
  return 0;
}


// return 1 if i starts a 3-dof world-translation root body (its 3 dofs share one welded body at
// the root of its kinematic tree)
static int effSimpleTriple(const mjModel* m, int i) {
  if (i < 0 || i + 3 > m->nv) {
    return 0;
  }
  int b = m->body_weldid[m->dof_bodyid[i]];
  return m->body_dofadr[b] == i && m->body_dofnum[b] == 3 &&
         (m->body_simple[b] == 2 || mj_flexBodySimple(m, b));
}


// return 1 if every active flex can be assembled by the simple 3x3 vertex fast path
static int efmAllSimple(const mjModel* m) {
  if (!mjd_flexInterpAssemblable(m)) {
    return 0;
  }
  for (int f=0; f < m->nflex; f++) {
    if (!m->flex_interp[f] &&
        mjd_flexStiff_activeGen(m, f, /*flg_bend=*/1, /*flg_stretch=*/1, /*flg_general=*/1) &&
        !mj_flexSimple(m, f)) {
      return 0;
    }
  }
  return 1;
}


// values of d->efm_active: the metric is active, and its stiffness CSR d->efm_K is either the
// simple assembly (3x3 vertex blocks of simple flexes; the other flexes use the matrix-free
// operators) or the general one (every active flex pulled back through its attachment Jacobians,
// so no matrix-free operator applies). mj_effBuild records the choice, so that consumers need not
// rescan the flexes per product, and a CG solve on a copy of a Newton step (the solver fallback)
// still sees the matrix as it was built.
#define EFM_ACTIVE 1
#define EFM_ACTIVE_GENERAL 2

// return 1 if d->efm_K was assembled in general articulated mode
static inline int efmGeneralCSR(const mjData* d) {
  return d->efm_active == EFM_ACTIVE_GENERAL;
}


// rank-1 terms (tendon, actuator, 1D rod twist) and passive contacts: res += sum scale*r'*r*vec
static inline void effMulAddRank1(const mjModel* m, const mjData* d, mjtNum* res,
                                  const mjtNum* vec, int flg_contact) {
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/0)) {
    mjtNum dot = 0;
    for (int j=0; j < e.nnz; j++) {
      dot += e.val[j] * vec[e.colind[j]];
    }
    dot *= e.scale;
    for (int j=0; j < e.nnz; j++) {
      res[e.colind[j]] += dot * e.val[j];
    }
  }

  // contact class: the same rank-1 apply over the packed rows, walked inline; the iterator's
  // per-row call is measurable with thousands of published pairs
  if (flg_contact) {
    const int* ind = d->efm_con_ind;
    const mjtNum* val = d->efm_con_val;
    for (int adr=0; adr < d->nefmcon; ) {
      int nnz = ind[adr];
      const int* ci = ind + adr + 2;
      const mjtNum* rv = val + adr + 2;
      mjtNum dot = 0;
      for (int j=0; j < nnz; j++) {
        dot += rv[j] * vec[ci[j]];
      }
      dot *= val[adr];
      for (int j=0; j < nnz; j++) {
        res[ci[j]] += dot * rv[j];
      }
      adr += 2 + nnz;
    }
  }
}


// matrix-free flex stiffness fallback for flexes not assembled into d->efm_K
static void effMulAddMatrixFree(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec) {
  mjtNum h = m->opt.timestep;
  mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h*h;
  mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h;
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_interp[f] || m->flex_rigid[f]) continue;
    if (m->flex_dim[f] == 1 && m->flex_bendingadr[f] < 0) continue;
    if (!d->nefmK || !mj_flexSimple(m, f)) {
      mjd_flexBend_mulRange(m, d, res, vec, s1, s2, f, f+1);
      mjd_flexStretch_mulRange(m, d, res, vec, s1, s2, f, f+1);
    }
  }
  if (!d->nefmK || !mjd_flexInterpAssemblable(m)) {
    mjd_flexInterp_mul(m, d, res, vec, -s1, -s2, d->flexelem_krot);
  }
}


// res += B*vec, the stiffness part of the metric. The per-dof diagonal classes are applied
// from efm_diag (scales pre-folded); the stretch (and, when assemblable, bending and interp)
// part from the per-step CSR; terms not in the CSR fall back to the matrix-free operators.
void mj_effMulAdd(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec, int flg_contact) {
  if (d->efm_diag) {
    int nv = m->nv;
    for (int i=0; i < nv; i++) {
      res[i] += d->efm_diag[i] * vec[i];
    }
  }

  // fluid drag blocks: res += F*vec, F symmetric in M's lower-triangle pattern
  if (d->efm_fluid) {
    int nv = m->nv;
    for (int i=0; i < nv; i++) {
      int start = m->M_rowadr[i];
      int diag = start + m->M_rownnz[i] - 1;
      mjtNum acc = d->efm_fluid[diag] * vec[i];
      for (int a=start; a < diag; a++) {
        int c = m->M_colind[a];
        mjtNum F = d->efm_fluid[a];
        acc += F * vec[c];
        res[c] += F * vec[i];
      }
      res[i] += acc;
    }
  }

  // rank-1 terms (tendon, actuator, 1D rod twist) and passive flex contacts
  effMulAddRank1(m, d, res, vec, flg_contact);

  int gen_csr = efmGeneralCSR(d);
  if (d->nefmK && d->nefmdof && !gen_csr) {
    // the stiffness rows come in vertex triples with 3x3 blocks per neighbour (the assembly
    // writes the three dofs of each neighbour in turn), so one column index per block and the
    // three rows share the vector loads
    const int* rowadr = d->efm_K_rowadr;
    const int* rownnz = d->efm_K_rownnz;
    const int* colind = d->efm_K_colind;
    const mjtNum* val = d->efm_K_val;
    for (int k=0; k < d->nefmdof; k++) {
      int i = d->efm_dofid[k];
      const mjtNum* v0 = val + rowadr[i];
      const mjtNum* v1 = val + rowadr[i+1];
      const mjtNum* v2 = val + rowadr[i+2];
      const int* ci = colind + rowadr[i];
      int nn = rownnz[i] / 3;
      mjtNum r0 = 0, r1 = 0, r2 = 0;
      for (int j=0; j < nn; j++) {
        int c = ci[3*j];
        mjtNum x0 = vec[c], x1 = vec[c+1], x2 = vec[c+2];
        r0 += v0[3*j]*x0 + v0[3*j+1]*x1 + v0[3*j+2]*x2;
        r1 += v1[3*j]*x0 + v1[3*j+1]*x1 + v1[3*j+2]*x2;
        r2 += v2[3*j]*x0 + v2[3*j+1]*x1 + v2[3*j+2]*x2;
      }
      res[i] += r0;
      res[i+1] += r1;
      res[i+2] += r2;
    }
  } else if (d->nefmK) {
    int nv = m->nv;
    for (int i=0; i < nv; i++) {
      int nnz = d->efm_K_rownnz[i];
      if (!nnz) {
        continue;
      }
      res[i] += mju_dotSparse(d->efm_K_val + d->efm_K_rowadr[i], vec, nnz,
                              d->efm_K_colind + d->efm_K_rowadr[i]);
    }
  }

  // Under CG, a standard flex is assembled only when all its attachments have fixed-frame XYZ
  // mappings; under Newton, general articulated flexes are assembled into efm_K too. Dispatch
  // un-assembled flexes individually: another flex's CSR must not suppress their operators.
  if (!gen_csr) {
    effMulAddMatrixFree(m, d, res, vec);
  }
}


//------------------ sparse Cholesky metric preconditioner ----------------------------------------
// When every covered flex is a 1D rod or a 2D flex (effChol12D), the covered block of M + K is
// factored by a reverse-order sparse Cholesky, blocked by vertex (effCholApplies); 3D flexes and
// fallback flex dofs use the qH diagonal preconditioner.
// The ordering is the model's: mj_setConst orders the vertices of the model's efm_K pattern by
// minimum degree (mj_effCholSetConst, into efmC_perm). Each step analyses the pattern it
// assembled under that ordering (effCholReserve), on the vertex graph
// (mju_cholFactorSymbolicBlocked), and packs the analysis into the metric's buffer efm_L, next to
// the factor it describes:
//   [status][the analysis (effPackLayout)][nL factor values]
// so the qH diagonal remains the fallback whenever the factor is unusable. The numeric
// factorization runs lazily (effCholEnsure).

#define EFF_HDR 1
#define EFF_CHOL_PENDING 2
#define EFF_ISLAND_TAG -1
#define EFF_CHOL_MAXVERT 8192
#define EFF_CHOL_PIVOT 1e-12

typedef struct {
  int nb, n, nL;
  const int* dofid;
  int *perm, *L_rownnz, *L_rowadr, *L_colind, *BT_adr, *BT_row, *BT_off;
} mjEffChol;

static size_t effIntBytes(size_t n) {
  return mj_stackBytes(sizeof(int)*(n > 0 ? n : 1), _Alignof(int));
}

static size_t effNumBytes(size_t n) {
  return mj_stackBytes(sizeof(mjtNum)*n, _Alignof(mjtNum));
}

static size_t effBitBytes(size_t n) {
  return mj_stackBytes(sizeof(uint64_t)*(n > 0 ? n : 1), _Alignof(uint64_t));
}

static int* effInts(mjData* d, size_t n) {
  return mjSTACKALLOC(d, n > 0 ? n : 1, int);
}

static uint64_t* effBits(mjData* d, size_t n) {
  return mjSTACKALLOC(d, n > 0 ? n : 1, uint64_t);
}

// off-diagonal 3x3 blocks of a blocked factor with nL values: row r of vertex p holds 3 values per
// block of the vertex and r - 3*p + 1 in the diagonal block
static int effCholBlocks(int nb, int nL) {
  return (nL - 6*nb) / 9;
}

// the step's analysis, packed into efm_L before the factor values: nL, then perm (nb),
// L_rownnz, L_rowadr (n), L_colind (nL), BT_adr (nb+1), BT_row, BT_off (effCholBlocks).
// Holds no pointer, so mj_copyData carries it with efm_L
static void effPackLayout(int* A, int nb, const int* dofid, mjEffChol* c) {
  int n = 3*nb;
  c->nb = nb;
  c->n = n;
  c->nL = A[0];
  c->dofid = dofid;
  int* p = A + 1;
  c->perm = p;       p += nb;
  c->L_rownnz = p;   p += n;
  c->L_rowadr = p;   p += n;
  c->L_colind = p;   p += c->nL;
  c->BT_adr = p;     p += nb + 1;
  c->BT_row = p;     p += effCholBlocks(nb, c->nL);
  c->BT_off = p;
}

static size_t effPackNums(int nb, int nL) {
  size_t ints = 2 + 8*(size_t)nb + (size_t)nL + 2*(size_t)effCholBlocks(nb, nL);
  return (sizeof(int)*ints + sizeof(mjtNum) - 1) / sizeof(mjtNum);
}

static void effCholStep(const mjData* d, mjEffChol* c) {
  int nb = d->nefmdof;
  effPackLayout((int*)(d->efm_L + EFF_HDR), nb, d->efm_dofid, c);
}

static int effCholReserved(const mjData* d) {
  int nb = d->nefmdof;
  return nb && d->nefmL > 0 && d->efm_L[0] >= 0;
}

// per-island flex/triple lists, packed into efm_L when islands are active, IPC is disabled and no
// sparse factor is reserved (effCholReserve):
//   [EFF_ISLAND_TAG][nisland, nb, nbd, nflex, cov_adr, cov_ind, bend_adr, ...]
typedef struct {
  int nisland, nb, nbd, nflex;
  int *cov_adr, *cov_ind;    // covered block indices k in [0, nefmdof) per island
  int *bend_adr, *bend_ind;  // bending dof indices b in [0, nefm0dof) per island
  int *flex_adr, *flex_ind;  // matrix-free flex indices f in [0, nflex) per island
} mjEffIsland;

static int effIslandTail(const mjModel* m, const mjData* d, int nb) {
  int nisland = d->nisland;
  int nbd = nb ? 0 : m->nefm0dof;
  int nflex = m->nflex;
  size_t ints = 4 + 3*(size_t)(nisland + 1) + (size_t)nb + (size_t)nbd + (size_t)nflex;
  size_t npack = (sizeof(int)*ints + sizeof(mjtNum) - 1) / sizeof(mjtNum);
  return EFF_HDR + (int)npack;
}

static void effIslandPackLayout(int* A, mjEffIsland* isl) {
  isl->nisland = A[0];
  isl->nb = A[1];
  isl->nbd = A[2];
  isl->nflex = A[3];
  int* p = A + 4;
  isl->cov_adr = p;   p += isl->nisland + 1;
  isl->cov_ind = p;   p += isl->nb;
  isl->bend_adr = p;  p += isl->nisland + 1;
  isl->bend_ind = p;  p += isl->nbd;
  isl->flex_adr = p;  p += isl->nisland + 1;
  isl->flex_ind = p;
}

static int effIslandStep(const mjData* d, mjEffIsland* isl) {
  if (d->nefmL <= 0 || d->efm_L[0] != EFF_ISLAND_TAG) {
    return 0;
  }
  int* A = (int*)(d->efm_L + EFF_HDR);
  if (A[0] != d->nisland) {
    return 0;
  }
  effIslandPackLayout(A, isl);
  return 1;
}

static mjtNum* effCholStepValues(const mjData* d, const mjEffChol* c) {
  return d->efm_L + EFF_HDR + effPackNums(c->nb, c->nL);
}

// the covered blocks of a K pattern with row sizes rownnz (nv rows): each nonzero row on a
// standalone 3-dof slider tree starts a vertex triple, its 3 consecutive dofs
static int effCoveredBlocks(const mjModel* m, int nv, const int* rownnz, int* dofid) {
  int nb = 0;
  for (int i=0; i < nv; ) {
    if (rownnz[i] && effSimpleTriple(m, i)) {
      if (dofid) dofid[nb] = i;
      nb++;
      i += 3;
    } else {
      i++;
    }
  }
  return nb;
}

static void effCovered(int nv, int nb, const int* dofid, int* cov) {
  for (int i=0; i < nv; i++) cov[i] = -1;
  for (int k=0; k < nb; k++) {
    for (int a=0; a < 3; a++) cov[dofid[k]+a] = k;
  }
}

// greedy minimum-degree elimination on either the full vertex graph (kadr/knbr + eadr/enbr) or
// the merged step graph (madr/mcnt/mnbr), with dense bitset adjacency; first eliminated goes last
static void effCholMinDegree(mjData* d, int nb, const int* kadr, const int* knbr,
                             const int* eadr, const int* enbr,
                             const int* madr, const int* mcnt, const int* mnbr, int* perm) {
  int words = (nb + 63) / 64;
  uint64_t* adj = effBits(d, (size_t)nb*words);
  uint64_t* nz_bits = effBits(d, words);
  int* scratch = effInts(d, 5*(size_t)nb + words);
  memset(adj, 0, sizeof(uint64_t)*(size_t)nb*words);
  for (int k=0; k < nb; k++) {
    if (kadr) {
      for (int j=kadr[k]; j < kadr[k+1]; j++) {
        int q = knbr[j];
        adj[(size_t)k*words+(q>>6)] |= 1ULL << (q&63);
        adj[(size_t)q*words+(k>>6)] |= 1ULL << (k&63);
      }
      for (int j=eadr[k]; j < eadr[k+1]; j++) {
        int q = enbr[j];
        adj[(size_t)k*words+(q>>6)] |= 1ULL << (q&63);
        adj[(size_t)q*words+(k>>6)] |= 1ULL << (k&63);
      }
    } else {
      for (int j=madr[k]; j < madr[k] + mcnt[k]; j++) {
        int q = mnbr[j];
        adj[(size_t)k*words+(q>>6)] |= 1ULL << (q&63);
        adj[(size_t)q*words+(k>>6)] |= 1ULL << (k&63);
      }
    }
  }
  mju_minDegree(perm, adj, nb, NULL, NULL, NULL, 0, scratch, nz_bits);
}

// count or collect unordered covered-vertex block pairs coupled by passive flex contacts whose
// participating dofs are all covered (skipping mixed covered/uncovered contacts avoids a
// one-sided preconditioner term that squares the contact condition number)
static int effCholContactPairs(const mjData* d, const int* blk, int nb,
                               int* vmark, int* vlist, int* cpair) {
  int np = 0;
  for (int k=0; k < nb; k++) vmark[k] = -1;
  for (int adr=0, cid=0; adr < d->nefmcon; cid++) {
    int nnz = d->efm_con_ind[adr];
    mjtNum scale = d->efm_con_val[adr];
    const int* ci = d->efm_con_ind + adr + 2;
    adr += 2 + nnz;
    if (scale <= 0) {
      continue;
    }
    int mvert = 0, all_cov = 1;
    for (int a=0; a < nnz; a++) {
      int k = blk[ci[a]];
      if (k < 0) {
        all_cov = 0;
        break;
      }
      if (vmark[k] != cid) {
        vmark[k] = cid;
        vlist[mvert++] = k;
      }
    }
    if (!all_cov) {
      continue;
    }
    if (cpair) {
      for (int u=0; u < mvert; u++) {
        for (int v=u+1; v < mvert; v++) {
          cpair[2*np] = vlist[u];
          cpair[2*np+1] = vlist[v];
          np++;
        }
      }
    } else {
      np += mvert * (mvert - 1) / 2;
    }
  }
  return np;
}

static void effAllow(int* amark, const int* eadr, const int* enbr, int k) {
  for (int j=eadr[k]; j < eadr[k+1]; j++) {
    amark[enbr[j]] = k;
  }
}

static int effAllowed(const int* amark, int ne, int k, int q) {
  return !ne || amark[q] == k;
}

// return 1 if 2D flex f needs its 2-hop bending flap pairs in the sparse factor: true when
// the flap term is non-negligible relative to vertex mass.
// Dropping a flap block q and lumping |q| onto both diagonals shifts P^-1 A eigenvalues by at most
// 2*deg*q/mmin, so 0.05*mmin keeps cond(P^-1 A) <= ~1.6 on a 6-neighbor mesh without adding 2-hop
// fill-in to thin stretch+bend cloth.
static int effCholBendFlaps(const mjModel* m, int f) {
  if (m->flex_dim[f] != 2 || m->flex_rigid[f] || m->flex_interp[f] ||
      m->flex_bendingadr[f] < 0) {
    return 0;
  }
  mjtNum h = m->opt.timestep;
  mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h*h;
  mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h;
  mjtNum scale = s1 + s2*mj_effFlexDamping(m, f);
  if (scale <= 0) {
    return 0;
  }
  const mjtNum* bend = m->flex_bending + m->flex_bendingadr[f];
  int eadr = m->flex_edgeadr[f], vadr = m->flex_vertadr[f];
  for (int e=0; e < m->flex_edgenum[f]; e++) {
    const int* flap = m->flex_edgeflap + 2*(eadr + e);
    if (flap[1] == -1) {
      continue;
    }
    mjtNum q = scale * mju_abs(bend[17*e + 11]);
    int b0 = m->flex_vertbodyid[vadr + flap[0]];
    int b1 = m->flex_vertbodyid[vadr + flap[1]];
    mjtNum m0 = (b0 >= 0 && m->body_dofnum[b0] == 3) ? m->dof_M0[m->body_dofadr[b0]] : 0;
    mjtNum m1 = (b1 >= 0 && m->body_dofnum[b1] == 3) ? m->dof_M0[m->body_dofadr[b1]] : 0;
    mjtNum mmin = (m0 > 0 && m1 > 0) ? mju_min(m0, m1) : mju_max(m0, m1);
    if (mmin > 0 && q > 0.05 * mmin) {
      return 1;
    }
  }
  return 0;
}

// upper bound on the number of allowed flex vertex pairs (flex edges, 1D rod hinge spans, and
// 2D bending flap pairs where significant)
static size_t effCholFlexPairs(const mjModel* m) {
  size_t np = (size_t)m->nflexedge;
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_dim[f] == 1 && !m->flex_rigid[f] && m->flex_bendingadr[f] >= 0) {
      const mjtNum* bend = m->flex_bending + m->flex_bendingadr[f];
      for (int h=0; h < (int)bend[0]; h++) {
        np += mj_rodHingeVert(bend + 1 + mjROD_HINGE_SIZE*h) == 3 ? 1 : 3;
      }
    } else if (effCholBendFlaps(m, f)) {
      np += (size_t)m->flex_edgenum[f];
    }
  }
  return np;
}

// the sorted, deduplicated symmetric CSR (eadr/enbr) of the np vertex pairs apair, on the stack
static void effPairCSR(mjData* d, int nb, const int* apair, int np, int** eadr, int** enbr) {
  int* ea = *eadr = effInts(d, nb + 1);
  int* en = *enbr = effInts(d, 2*(size_t)np);
  int* cur = effInts(d, nb);
  for (int k=0; k <= nb; k++) ea[k] = 0;
  for (int x=0; x < 2*np; x++) ea[apair[x]+1]++;
  for (int k=0; k < nb; k++) {
    ea[k+1] += ea[k];
    cur[k] = ea[k];
  }
  for (int x=0; x < np; x++) {
    en[cur[apair[2*x]]++] = apair[2*x+1];
    en[cur[apair[2*x+1]]++] = apair[2*x];
  }
  int ne = 0, start = 0;
  for (int k=0; k < nb; k++) {
    int end = ea[k+1];
    for (int x=start+1; x < end; x++) {
      int v = en[x], y = x;
      for (; y > start && en[y-1] > v; y--) en[y] = en[y-1];
      en[y] = v;
    }
    ea[k] = ne;
    for (int x=start; x < end; x++) {
      if (x == start || en[x] != en[x-1]) en[ne++] = en[x];
    }
    start = end;
  }
  ea[nb] = ne;
}

// build the K vertex graph (kadr/knbr) and the allowed edge CSR (eadr/enbr: flex edges, 1D rod
// hinge spans, and significant 2D bending flap pairs between covered vertices, both ways, sorted
// and deduplicated)
static int effCholGraph(mjData* d, const mjModel* m, int nb, const int* dofid, const int* blk,
                        const int* K_rownnz, const int* K_rowadr, const int* K_colind,
                        int** kadr, int** knbr, int** eadr, int** enbr) {
  size_t kcap = 0;
  for (int k=0; k < nb; k++) kcap += K_rownnz[dofid[k]];
  int* ka = *kadr = effInts(d, nb + 1);
  int* kn = *knbr = effInts(d, kcap);
  ka[0] = 0;
  for (int k=0; k < nb; k++) {
    int i = dofid[k];
    ka[k+1] = ka[k];
    for (int j=K_rowadr[i]; j < K_rowadr[i] + K_rownnz[i]; j++) {
      int col = K_colind[j], q = blk[col];
      if (q >= 0 && q != k && col == dofid[q]) kn[ka[k+1]++] = q;
    }
  }
  int np = 0;
  int* apair = effInts(d, 2*effCholFlexPairs(m));
  for (int f=0; f < m->nflex; f++) {
    for (int e=0; e < m->flex_edgenum[f]; e++) {
      int ke[2];
      for (int s=0; s < 2; s++) {
        int v = m->flex_edge[2*(m->flex_edgeadr[f]+e)+s];
        int b = m->flex_vertbodyid[m->flex_vertadr[f]+v];
        int da = (b >= 0 && m->body_dofnum[b] == 3) ? m->body_dofadr[b] : -1;
        ke[s] = (da >= 0 && blk[da] >= 0 && dofid[blk[da]] == da) ? blk[da] : -1;
      }
      if (ke[0] >= 0 && ke[1] >= 0 && ke[0] != ke[1]) {
        apair[2*np] = ke[0];
        apair[2*np+1] = ke[1];
        np++;
      }
    }
    if (m->flex_dim[f] == 1 && !m->flex_rigid[f] && m->flex_bendingadr[f] >= 0) {
      const mjtNum* bend = m->flex_bending + m->flex_bendingadr[f];
      // vertex spans not covered by flex edges: (0, 2), plus (1, 3) and (0, 3) for a hinge pair
      static const int span[3][2] = {{0, 2}, {1, 3}, {0, 3}};
      for (int h=0; h < (int)bend[0]; h++) {
        const mjtNum* hinge = bend + 1 + mjROD_HINGE_SIZE*h;
        int nr = mj_rodHingeVert(hinge);
        int kv[4];
        for (int s=0; s < nr; s++) {
          int v = (int)hinge[s];
          int b = m->flex_vertbodyid[m->flex_vertadr[f]+v];
          int da = (b >= 0 && m->body_dofnum[b] == 3) ? m->body_dofadr[b] : -1;
          kv[s] = (da >= 0 && blk[da] >= 0 && dofid[blk[da]] == da) ? blk[da] : -1;
        }
        int nspan = (nr == 3) ? 1 : 3;
        for (int p=0; p < nspan; p++) {
          int k0 = kv[span[p][0]], k1 = kv[span[p][1]];
          if (k0 >= 0 && k1 >= 0 && k0 != k1) {
            apair[2*np] = k0;
            apair[2*np+1] = k1;
            np++;
          }
        }
      }
    } else if (effCholBendFlaps(m, f)) {
      for (int e=0; e < m->flex_edgenum[f]; e++) {
        const int* flap = m->flex_edgeflap + 2*(m->flex_edgeadr[f]+e);
        if (flap[1] == -1) {
          continue;
        }
        int kf[2];
        for (int s=0; s < 2; s++) {
          int b = m->flex_vertbodyid[m->flex_vertadr[f]+flap[s]];
          int da = (b >= 0 && m->body_dofnum[b] == 3) ? m->body_dofadr[b] : -1;
          kf[s] = (da >= 0 && blk[da] >= 0 && dofid[blk[da]] == da) ? blk[da] : -1;
        }
        if (kf[0] >= 0 && kf[1] >= 0 && kf[0] != kf[1]) {
          apair[2*np] = kf[0];
          apair[2*np+1] = kf[1];
          np++;
        }
      }
    }
  }
  effPairCSR(d, nb, apair, np, eadr, enbr);
  return 2*np;
}

// single-pass symbolic analysis on the stack: builds the vertex graph, runs minimum-degree
// ordering if order is nonzero (else perm is the caller's), and computes the blocked symbolic
// Cholesky counts
typedef struct {
  int nb, nL, ne, nfe;
  int *eadr, *enbr, *rownnz, *rowadr, *colind, *scratch;
  int *L_rownnz, *L_rowadr, *BT_adr;
} mjEffCholPass;

static int effCholBegin(const mjModel* m, mjData* d, int nb, const int* dofid, const int* blk,
                        const int* K_rownnz, const int* K_rowadr, const int* K_colind,
                        int np_con, int order, int* perm, mjEffCholPass* p) {
  if (nb <= 0 || nb > EFF_CHOL_MAXVERT) {
    return 0;
  }
  int n = 3*nb, *kadr, *knbr;
  p->nb = nb;
  p->nfe = effCholGraph(d, m, nb, dofid, blk, K_rownnz, K_rowadr, K_colind,
                        &kadr, &knbr, &p->eadr, &p->enbr);
  p->ne = p->eadr[nb];
  int* amark = effInts(d, nb);
  int* mark = effInts(d, nb);
  int* adr = effInts(d, nb);
  int* cnt = effInts(d, nb);
  int *cadr = NULL, *cnbr = NULL, nc = 0;
  if (np_con > 0) {
    int* cpair = effInts(d, 2*(size_t)np_con);
    effCholContactPairs(d, blk, nb, mark, adr, cpair);
    effPairCSR(d, nb, cpair, np_con, &cadr, &cnbr);
    nc = cadr[nb];
  }
  int* nbr = effInts(d, (size_t)kadr[nb] + p->ne + nc);
  for (int k=0; k < nb; k++) amark[k] = mark[k] = -1;
  int tot = 0, has_new_con = 0;
  for (int k=0; k < nb; k++) {
    adr[k] = tot;
    mark[k] = k;
    effAllow(amark, p->eadr, p->enbr, k);
    for (int j=kadr[k]; j < kadr[k+1]; j++) {
      int q = knbr[j];
      if (effAllowed(amark, p->ne, k, q) && mark[q] != k) {
        mark[q] = k;
        nbr[tot++] = q;
      }
    }
    for (int j=p->eadr[k]; j < p->eadr[k+1]; j++) {
      int q = p->enbr[j];
      if (mark[q] != k) {
        mark[q] = k;
        nbr[tot++] = q;
      }
    }
    if (cadr) {
      for (int j=cadr[k]; j < cadr[k+1]; j++) {
        int q = cnbr[j];
        if (mark[q] != k) {
          has_new_con = 1;
          mark[q] = k;
          nbr[tot++] = q;
        }
      }
    }
    cnt[k] = tot - adr[k];
  }
  if (order) {
    // model-level ordering (mj_effCholSetConst) includes the full K stencil (used by Newton too)
    effCholMinDegree(d, nb, kadr, knbr, p->eadr, p->enbr, NULL, NULL, NULL, perm);
  } else if (has_new_con) {
    effCholMinDegree(d, nb, NULL, NULL, NULL, NULL, adr, cnt, nbr, perm);
  }
  p->rownnz = effInts(d, nb);
  p->rowadr = effInts(d, nb);
  p->colind = effInts(d, (size_t)kadr[nb] + p->ne + nc);
  p->scratch = effInts(d, 3*(size_t)nb);
  for (int k=0; k < nb; k++) p->rownnz[perm[k]] = cnt[k];
  for (int pos=0, a=0; pos < nb; pos++) {
    p->rowadr[pos] = a;
    a += p->rownnz[pos];
  }
  for (int k=0; k < nb; k++) {
    int* row = p->colind + p->rowadr[perm[k]];
    for (int j=0; j < cnt[k]; j++) {
      int v = perm[nbr[adr[k]+j]], y = j;
      for (; y > 0 && row[y-1] > v; y--) row[y] = row[y-1];
      row[y] = v;
    }
  }
  p->L_rownnz = effInts(d, n);
  p->L_rowadr = effInts(d, n);
  p->BT_adr = effInts(d, nb + 1);
  p->nL = mju_cholFactorSymbolicBlocked(NULL, p->L_rownnz, p->L_rowadr, p->BT_adr, NULL, NULL,
                                        p->rownnz, p->rowadr, p->colind, nb, 3, p->scratch);
  return 1;
}

static void effCholFinish(const mjEffCholPass* p, mjEffChol* c) {
  int nb = p->nb, n = 3*nb;
  memcpy(c->L_rownnz, p->L_rownnz, sizeof(int)*n);
  memcpy(c->L_rowadr, p->L_rowadr, sizeof(int)*n);
  memcpy(c->BT_adr, p->BT_adr, sizeof(int)*(nb + 1));
  mju_cholFactorSymbolicBlocked(c->L_colind, c->L_rownnz, c->L_rowadr, c->BT_adr, c->BT_row,
                                c->BT_off, p->rownnz, p->rowadr, p->colind, nb, 3, p->scratch);
}

// return 1 if every covered flex qualifies for the sparse Cholesky factor or ordering: 1D rods,
// 2D flexes if allow2d, and 3D flexes if allow3d, not interpolated (currently all-or-nothing
// across covered flexes since efm_dofid and nefmCvert index all covered triples together).
// Flexes with articulated attachments are covered only by the general assembly (flg_general):
// otherwise they stay matrix-free and do not restrict the factor of the simple ones.
static int effCholDimOk(const mjModel* m, int allow2d, int allow3d, int flg_general) {
  for (int f=0; f < m->nflex; f++) {
    int covered = mjd_flexStiff_activeGen(m, f, /*flg_bend=*/1, /*flg_stretch=*/1, flg_general) ||
                  mjd_flexInterp_processed(m, f) || mj_effFlexContactPossible(m, f);
    if (!covered) {
      continue;
    }
    if (m->flex_interp[f]) {
      return 0;
    }
    if (m->flex_dim[f] != 1 && !(allow2d && m->flex_dim[f] == 2) &&
        !(allow3d && m->flex_dim[f] == 3)) {
      return 0;
    }
  }
  return 1;
}

// the step uses the sparse factor: 1D rods and 2D flexes, whatever the solver. It serves
// qacc_smooth (mj_effSolve) and Newton; the CG constraint solve preconditions with the folded
// per-dof diagonal (mj_effPrecFold) instead, and falls back to the factor only when it cannot fold
static int effCholApplies(const mjModel* m, const mjData* d, int nb, const int* dofid) {
  if (!nb || nb > EFF_CHOL_MAXVERT || !m->nefmCvert || m->efmC_perm[0] < 0 ||
      !effCholDimOk(m, /*allow2d=*/1, /*allow3d=*/0, efmGeneralCSR(d))) {
    return 0;
  }
  for (int f=0; f < m->nflex; f++) {
    if (mjd_flexInterp_processed(m, f)) {
      return 0;
    }
  }
  return dofid[nb-1] + 3 <= m->nv;
}

static size_t effCholKcap(const mjData* d, int nb, const int* dofid) {
  size_t cap = 0;
  for (int k=0; k < nb; k++) cap += d->efm_K_rownnz[dofid[k]];
  return cap;
}

static size_t effCholAnalysisBytes(const mjModel* m, size_t nb, size_t kcap, size_t np_con) {
  size_t ne = effCholFlexPairs(m);
  size_t nnbr = kcap + 2*ne + 2*np_con;
  size_t words = (nb + 63) / 64;
  size_t con_bytes = np_con ? (effIntBytes(nb + 1) + 2*effIntBytes(2*np_con) + effIntBytes(nb)) : 0;
  return mj_stackFrameBytes() + 2*effIntBytes(nb + 1) + effIntBytes(kcap) + 2*effIntBytes(2*ne) +
         effIntBytes(nb) + 4*effIntBytes(nb) + con_bytes + effIntBytes(nnbr) +
         effBitBytes(nb*words) + effBitBytes(words) + effIntBytes(5*nb + words) +
         2*effIntBytes(nb) + effIntBytes(nnbr) + effIntBytes(3*nb) + 2*effIntBytes(3*nb) +
         effIntBytes(nb + 1);
}

static size_t effFillBytes(size_t nb, size_t nv) {
  return mj_stackFrameBytes() + effNumBytes(9*nb) + effIntBytes(nv) + 4*effIntBytes(nb);
}

static size_t effCholFillBytes(const mjEffChol* c, int nv) {
  return effFillBytes(c->nb, nv);
}

static size_t effCholReserveBytes(int nb, int nv, int nL, size_t npack) {
  return effNumBytes(EFF_HDR + npack + (size_t)nL) + effNumBytes(EFF_HDR + (size_t)nL) +
         2*effIntBytes(nb) + effNumBytes(3*(size_t)nb) + effFillBytes(nb, nv);
}

// compute the model's minimum-degree vertex ordering for 1D rod and 2D/3D flex stencils
void mj_effCholSetConst(mjModel* m, mjData* d) {
  int ncap = m->nefmCvert, nv = m->nv;
  if (!ncap) {
    return;
  }
  for (int t=0; t < ncap; t++) m->efmC_perm[t] = -1;
  // the ordering is structural: computed for every model with non-interpolated flexes, whatever
  // the solver options (effCholApplies decides per step whether the metric factor is used).
  // Articulated flexes enter the stiffness pattern only under Newton (the general assembly of
  // mj_effBuild), so only then do they shape the ordering
  int gen = 0;
  for (int f=0; m->opt.solver == mjSOL_NEWTON && f < m->nflex; f++) {
    if (!m->flex_interp[f] &&
        mjd_flexStiff_activeGen(m, f, /*flg_bend=*/1, /*flg_stretch=*/1, /*flg_general=*/1) &&
        !mj_flexSimple(m, f)) {
      gen = 1;
      break;
    }
  }
  if (!effCholDimOk(m, /*allow2d=*/1, /*allow3d=*/1, gen)) {
    return;
  }
  mj_markStack(d);
  size_t avail = mj_stackBytesAvailable(d);
  size_t need = mj_stackFrameBytes() + 3*effIntBytes(nv) + effIntBytes(ncap + 1);

  // the general pattern passes need stack for the attachment chains; when it does not fit,
  // order the simple pattern instead, whose covered vertices the ordering also serves
  if (gen && avail < need + mjd_flexStiff_assembleGenStack(m, 1, 1, /*flg_interp=*/0)) {
    gen = 0;
  }
  int nb = 0, nK = 0, *K_rownnz = NULL, *K_rowadr = NULL, *blk = NULL, *dofid = NULL;
  size_t kcap = 0;
  if (avail >= need) {
    K_rownnz = effInts(d, nv);
    K_rowadr = effInts(d, nv);
    blk = effInts(d, nv);
    dofid = effInts(d, ncap + 1);
    nK = mjd_flexStiff_assembleGen(m, d, K_rownnz, K_rowadr, NULL, NULL, 0, 0, 1, 1, NULL, gen);
    for (int i=0; i < nv; i++) blk[i] = K_rownnz[i];
    for (int f=0; f < m->nflex; f++) {
      // edge equality (mjEQ_FLEX) on a non-interpolated 2D/3D flex
      if (m->flex_edgeequality[f] != 1 || m->flex_interp[f] || m->flex_rigid[f] ||
          m->flex_dim[f] < 2) {
        continue;
      }
      for (int v=0; v < m->flex_vertnum[f]; v++) {
        int b = m->flex_vertbodyid[m->flex_vertadr[f]+v];
        if (b >= 0 && m->body_dofnum[b] == 3) {
          int da = m->body_dofadr[b];
          blk[da] = blk[da+1] = blk[da+2] = 1;
        }
      }
    }
    nb = effCoveredBlocks(m, nv, blk, NULL);
    if (!nb || nb > ncap || nb > EFF_CHOL_MAXVERT) {
      mj_freeStack(d);
      return;
    }
    effCoveredBlocks(m, nv, blk, dofid);
    if (dofid[nb-1] + 3 > nv) {
      mj_freeStack(d);
      return;
    }
    for (int k=0; k < nb; k++) kcap += K_rownnz[dofid[k]];
    need += effIntBytes(nK) + effIntBytes(nb) + effCholAnalysisBytes(m, nb, kcap, /*np_con=*/0);
  }
  if (avail < need) {
    mj_freeStack(d);
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return;
  }
  int* K_colind = effInts(d, nK);
  int* perm = effInts(d, nb);
  mjd_flexStiff_assembleGen(m, d, K_rownnz, K_rowadr, K_colind, NULL, 0, 0, 1, 1, NULL, gen);
  effCovered(nv, nb, dofid, blk);
  mjEffCholPass pass;
  if (effCholBegin(m, d, nb, dofid, blk, K_rownnz, K_rowadr, K_colind, /*np_con=*/0, 1, perm,
                   &pass)) {
    for (int k=0; k < nb; k++) m->efmC_perm[perm[k]] = dofid[k];
  }
  mj_freeStack(d);
}

// numeric factorization by 3x3 vertex blocks: H = L'L backward elimination in place in L (the
// rows of a vertex are read before they are written, and only rows already factored are read)
static int effCholNumericBlocked(const mjEffChol* c, mjtNum* restrict L, mjtNum mindiag,
                                 mjtNum* restrict dense) {
  int nb = c->nb, rank = c->n;
  const int *rowadr = c->L_rowadr, *rownnz = c->L_rownnz, *colind = c->L_colind;
  for (int p=nb-1; p >= 0; p--) {
    int r0 = 3*p, nob = rownnz[r0] - 1, used_dense = 0;
    const int* ci = colind + rowadr[r0];
    mjtNum* restrict L0 = L + rowadr[r0];
    mjtNum* restrict L1 = L + rowadr[r0+1];
    mjtNum* restrict L2 = L + rowadr[r0+2];
    mjtNum* restrict La[3] = {L0, L1, L2};
    mjtNum d00 = L0[nob], d10 = L1[nob], d11 = L1[nob+1];
    mjtNum d20 = L2[nob], d21 = L2[nob+1], d22 = L2[nob+2];
    for (int k=c->BT_adr[p]; k < c->BT_adr[p+1]; k++) {
      int qrow = 3*c->BT_row[k], off = c->BT_off[k];
      const mjtNum *Q0 = L + rowadr[qrow], *Q1 = L + rowadr[qrow+1], *Q2 = L + rowadr[qrow+2];
      const int* qci = colind + rowadr[qrow];
      mjtNum l00 = Q0[off], l01 = Q0[off+1], l02 = Q0[off+2];
      mjtNum l10 = Q1[off], l11 = Q1[off+1], l12 = Q1[off+2];
      mjtNum l20 = Q2[off], l21 = Q2[off+1], l22 = Q2[off+2];
      if (!used_dense && (!off || qci[0] == ci[nob-off])) {
        int d = nob - off;
        for (int i=0; i < off; i++) {
          mjtNum y0 = Q0[i], y1 = Q1[i], y2 = Q2[i];
          L0[d+i] -= l00*y0 + l10*y1 + l20*y2;
          L1[d+i] -= l01*y0 + l11*y1 + l21*y2;
          L2[d+i] -= l02*y0 + l12*y1 + l22*y2;
        }
      } else {
        if (!used_dense) {
          used_dense = 1;
          for (int i=0; i < nob; i++) {
            mjtNum* dd = dense + 3*ci[i];
            dd[0] = L0[i];
            dd[1] = L1[i];
            dd[2] = L2[i];
          }
        }
        for (int i=0; i < off; i++) {
          mjtNum y0 = Q0[i], y1 = Q1[i], y2 = Q2[i];
          mjtNum* dd = dense + 3*qci[i];
          dd[0] -= l00*y0 + l10*y1 + l20*y2;
          dd[1] -= l01*y0 + l11*y1 + l21*y2;
          dd[2] -= l02*y0 + l12*y1 + l22*y2;
        }
      }
      d00 -= l00*l00 + l10*l10 + l20*l20;
      d10 -= l01*l00 + l11*l10 + l21*l20;
      d20 -= l02*l00 + l12*l10 + l22*l20;
      d11 -= l01*l01 + l11*l11 + l21*l21;
      d21 -= l02*l01 + l12*l11 + l22*l21;
      d22 -= l02*l02 + l12*l12 + l22*l22;
    }
    if (used_dense) {
      for (int i=0; i < nob; i++) {
        mjtNum* dd = dense + 3*ci[i];
        L0[i] = dd[0];
        L1[i] = dd[1];
        L2[i] = dd[2];
        dd[0] = dd[1] = dd[2] = 0;
      }
    }
    L0[nob] = d00;
    L1[nob] = d10;
    L1[nob+1] = d11;
    L2[nob] = d20;
    L2[nob+1] = d21;
    L2[nob+2] = d22;
    for (int a=2; a >= 0; a--) {
      mjtNum* restrict Ra = La[a];
      mjtNum diag = Ra[nob+a];
      int deficient = diag < mindiag;
      if (deficient) {
        diag = mindiag;
        rank--;
      }
      mjtNum Lrr = mju_sqrt(diag), inv = 1.0 / Lrr;
      if (deficient) {
        mju_zero(Ra, nob + a);
      } else {
        for (int i=0; i < nob + a; i++) Ra[i] *= inv;
      }
      Ra[nob+a] = Lrr;
      for (int e=0; e < a; e++) {
        mjtNum le = Ra[nob+e];
        if (!le) continue;
        mjtNum* restrict Re = La[e];
        for (int i=0; i <= nob + e; i++) Re[i] -= le * Ra[i];
      }
    }
  }
  return rank;
}

// assemble the covered block of M + K into values, in the factor's pattern, and factor it in
// place. A K block B_kq is exact where the factor has a block for the pair (an allowed pair or
// fill); where it has none, B_kq is dropped and replaced by its absolute row sums on the diagonal
// of k and its absolute column sums on that of q, which keeps the dropped part [D_r, -B; -B', D_c]
// diagonally dominant, so the assembled matrix stays positive definite and at least M + K
static void effCholFill(const mjModel* m, mjData* d, const mjEffChol* c, mjtNum* status,
                        mjtNum* values) {
  int nb = c->nb, n = c->n, nL = c->nL, nv = m->nv;
  mjtNum* H = values;
  *status = 0;
  mj_markStack(d);
  mju_zero(H, nL);
  mjtNum* scratch = mjSTACKALLOC(d, 3*n, mjtNum);
  mju_zero(scratch, 3*n);
  int* blk = effInts(d, nv);
  int* sc = effInts(d, 4*(size_t)nb);
  int *slot = sc, *qmark = sc + nb, *qlist = sc + 2*nb, *qpos = sc + 3*nb;
  effCovered(nv, nb, c->dofid, blk);
  for (int k=0; k < nb; k++) slot[k] = qmark[k] = -1;

  // the three rows of a covered triple share their column pattern, with a 3x3 block per
  // neighbour (mj_effMulAdd)
  const int* K_rowadr = d->efm_K_rowadr;
  for (int k=0; k < nb; k++) {
    int p = c->perm[k], i0 = c->dofid[k];
    const int* vc = c->L_colind + c->L_rowadr[3*p];
    int nvc = (c->L_rownnz[3*p] - 1) / 3, nq = 0;
    for (int i=0; i < nvc; i++) slot[vc[3*i]/3] = i;
    int ra[3], da[3];
    const mjtNum* Kv[3];
    for (int a=0; a < 3; a++) {
      ra[a] = c->L_rowadr[3*p+a];
      da[a] = ra[a] + 3*nvc;
      Kv[a] = d->efm_K_val + K_rowadr[i0+a];
    }
    const int* ci = d->efm_K_colind + K_rowadr[i0];
    int rownnz = d->efm_K_rownnz[i0];
    for (int j=0; j < rownnz; ) {
      int q = blk[ci[j]];
      if (q < 0) {
        j++;
        continue;
      }
      if (q == k) {
        for (int a=0; a < 3; a++) {
          for (int b=0; b <= a; b++) H[da[a]+b] += Kv[a][j+b];
        }
        j += 3;
        continue;
      }

      // each pair is taken from the row of the vertex eliminated first (the larger perm)
      int pq = c->perm[q];
      if (pq > p) {
        j += 3;
        continue;
      }
      int s = slot[pq];
      if (s >= 0) {
        for (int a=0; a < 3; a++) {
          mjtNum* h = H + ra[a] + 3*s;
          h[0] += Kv[a][j];
          h[1] += Kv[a][j+1];
          h[2] += Kv[a][j+2];
        }
      } else {
        if (qmark[q] != k) {
          qmark[q] = k;
          qlist[nq] = q;
          qpos[q] = nq++;
        }
        mjtNum* acc = scratch + 6*qpos[q];
        for (int a=0; a < 3; a++) {
          for (int b=0; b < 3; b++) {
            mjtNum av = mju_abs(Kv[a][j+b]);
            acc[a] += av;
            acc[3+b] += av;
          }
        }
      }
      j += 3;
    }
    for (int a=0; a < 3; a++) {
      int i = i0 + a;
      for (int j=m->M_rowadr[i]; j < m->M_rowadr[i] + m->M_rownnz[i]; j++) {
        int col = m->M_colind[j];
        if (col >= i0) {
          H[da[a]+col-i0] += d->M[j] + (d->efm_fluid ? d->efm_fluid[j] : 0);
        }
      }
      if (d->efm_diag) {
        H[da[a]+a] += d->efm_diag[i];
      }
    }
    for (int i=0; i < nvc; i++) slot[vc[3*i]/3] = -1;
    for (int idx=0; idx < nq; idx++) {
      int pk = 3*p, pq = 3*c->perm[qlist[idx]];
      mjtNum* acc = scratch + 6*idx;
      for (int a=0; a < 3; a++) {
        H[c->L_rowadr[pk+a] + c->L_rownnz[pk+a] - 1] += acc[a];
        H[c->L_rowadr[pq+a] + c->L_rownnz[pq+a] - 1] += acc[3+a];
      }
    }
    mju_zero(scratch, 6*nq);
  }

  // scatter single-dof tendon and actuator diagonal stiffness onto covered triples
  for (int t=0; t < d->nefmT; t++) {
    mjtNum s = d->efm_ts[t];
    int i = d->efm_tid[t];
    if (!s || m->ten_J_rownnz[i] != 1) {
      continue;
    }
    int j = m->ten_J_rowadr[i];
    int dof = m->ten_J_colind[j], k = blk[dof];
    if (k >= 0) {
      int row = 3*c->perm[k] + (dof - c->dofid[k]);
      H[c->L_rowadr[row] + c->L_rownnz[row] - 1] += s * d->ten_J[j] * d->ten_J[j];
    }
  }
  for (int t=0; t < d->nefmA; t++) {
    mjtNum s = d->efm_as[t];
    if (!s) {
      continue;
    }
    int i = d->efm_aid[t];
    int oadr = m->actuator_outadr[i];
    for (int m_out=0; m_out < m->actuator_outnum[i]; m_out++) {
      int r = oadr + m_out;
      if (d->moment_rownnz[r] != 1) {
        continue;
      }
      int j = d->moment_rowadr[r];
      int dof = d->moment_colind[j], k = blk[dof];
      if (k >= 0) {
        int row = 3*c->perm[k] + (dof - c->dofid[k]);
        H[c->L_rowadr[row] + c->L_rownnz[row] - 1] +=
            s * d->actuator_moment[j] * d->actuator_moment[j];
      }
    }
  }

  // scatter passive flex contact rank-1 terms (scale * row' * row) on covered triples
  if (d->nefmcon) {
    for (int k=0; k < nb; k++) qmark[k] = -1;
    for (int adr=0, cid=0; adr < d->nefmcon; cid++) {
      int nnz = d->efm_con_ind[adr];
      mjtNum scale = d->efm_con_val[adr];
      const int* ci = d->efm_con_ind + adr + 2;
      const mjtNum* rv = d->efm_con_val + adr + 2;
      adr += 2 + nnz;
      if (scale <= 0) {
        continue;
      }
      int mvert = 0, all_cov = 1;
      for (int a=0; a < nnz; a++) {
        int dof = ci[a], k = blk[dof];
        if (k < 0) {
          all_cov = 0;
          break;
        }
        int u;
        if (qmark[k] != cid) {
          qmark[k] = cid;
          qpos[k] = u = mvert++;
          qlist[u] = k;
          scratch[3*u] = scratch[3*u+1] = scratch[3*u+2] = 0;
        } else {
          u = qpos[k];
        }
        scratch[3*u+(dof-c->dofid[k])] += rv[a];
      }
      if (!all_cov) {
        mju_zero(scratch, 3*mvert);
        continue;
      }
      for (int u=0; u < mvert; u++) {
        int ku = qlist[u], pu = c->perm[ku];
        const mjtNum* wu = scratch + 3*u;
        mjtNum su0 = scale*wu[0], su1 = scale*wu[1], su2 = scale*wu[2];
        int nob = c->L_rownnz[3*pu] - 1;
        H[c->L_rowadr[3*pu]+nob]     += su0 * wu[0];
        H[c->L_rowadr[3*pu+1]+nob]   += su1 * wu[0];
        H[c->L_rowadr[3*pu+1]+nob+1] += su1 * wu[1];
        H[c->L_rowadr[3*pu+2]+nob]   += su2 * wu[0];
        H[c->L_rowadr[3*pu+2]+nob+1] += su2 * wu[1];
        H[c->L_rowadr[3*pu+2]+nob+2] += su2 * wu[2];
        for (int v=0; v < u; v++) {
          int kv = qlist[v], pv = c->perm[kv];
          int ph, pl;
          const mjtNum *wh, *wl;
          if (pu > pv) {
            ph = pu; pl = pv; wh = wu; wl = scratch + 3*v;
          } else {
            ph = pv; pl = pu; wh = scratch + 3*v; wl = wu;
          }
          const int* vc = c->L_colind + c->L_rowadr[3*ph];
          int nvc = (c->L_rownnz[3*ph] - 1) / 3;
          int lo = 0, hi = nvc;
          while (lo < hi) {
            int mid = (lo + hi) >> 1;
            if (vc[3*mid] < 3*pl) lo = mid + 1;
            else hi = mid;
          }
          if (lo < nvc && vc[3*lo] == 3*pl) {
            mjtNum* h0 = H + c->L_rowadr[3*ph] + 3*lo;
            mjtNum* h1 = H + c->L_rowadr[3*ph+1] + 3*lo;
            mjtNum* h2 = H + c->L_rowadr[3*ph+2] + 3*lo;
            mjtNum sh0 = scale*wh[0], sh1 = scale*wh[1], sh2 = scale*wh[2];
            h0[0] += sh0*wl[0]; h0[1] += sh0*wl[1]; h0[2] += sh0*wl[2];
            h1[0] += sh1*wl[0]; h1[1] += sh1*wl[1]; h1[2] += sh1*wl[2];
            h2[0] += sh2*wl[0]; h2[1] += sh2*wl[1]; h2[2] += sh2*wl[2];
          }
        }
      }
      mju_zero(scratch, 3*mvert);
    }
  }
  mjtNum maxdiag = 0;
  for (int r=0; r < n; r++) {
    maxdiag = mju_max(maxdiag, H[c->L_rowadr[r]+c->L_rownnz[r]-1]);
  }
  if (effCholNumericBlocked(c, H, mju_max(mjMINVAL, EFF_CHOL_PIVOT*maxdiag), scratch) == n) {
    *status = 1;
  }
  mj_freeStack(d);
}

static void effCholEnsure(const mjModel* m, mjData* d) {
  if (!effCholReserved(d) || d->efm_L[0] != EFF_CHOL_PENDING) {
    return;
  }
  d->efm_L[0] = 0;
  mjEffChol c;
  effCholStep(d, &c);
  if (mj_stackBytesAvailable(d) >= effCholFillBytes(&c, m->nv)) {
    effCholFill(m, d, &c, d->efm_L, effCholStepValues(d, &c));
  }
}

static void effCholSolveBlocked(mjtNum* restrict res, const mjtNum* restrict L,
                                const mjtNum* restrict vec, const mjEffChol* restrict c) {
  int nb = c->nb;
  const int* restrict rowadr = c->L_rowadr;
  const int* restrict rownnz = c->L_rownnz;
  const int* restrict colind = c->L_colind;
  mju_copy(res, vec, 3*nb);
  for (int p=nb-1; p >= 0; p--) {
    int nob = rownnz[3*p] - 1;
    const mjtNum* restrict L0 = L + rowadr[3*p];
    const mjtNum* restrict L1 = L + rowadr[3*p+1];
    const mjtNum* restrict L2 = L + rowadr[3*p+2];
    mjtNum* r = res + 3*p;
    mjtNum x2 = r[2] / L2[nob+2];
    mjtNum x1 = (r[1] - L2[nob+1]*x2) / L1[nob+1];
    mjtNum x0 = (r[0] - L2[nob]*x2 - L1[nob]*x1) / L0[nob];
    r[0] = x0;
    r[1] = x1;
    r[2] = x2;
    const int* restrict ci = colind + rowadr[3*p];
    for (int j=0; j < nob; j += 3) {
      mjtNum* rq = res + ci[j];
      rq[0] -= L0[j]*x0 + L1[j]*x1 + L2[j]*x2;
      rq[1] -= L0[j+1]*x0 + L1[j+1]*x1 + L2[j+1]*x2;
      rq[2] -= L0[j+2]*x0 + L1[j+2]*x1 + L2[j+2]*x2;
    }
  }
  for (int p=0; p < nb; p++) {
    int nob = rownnz[3*p] - 1;
    const mjtNum* restrict L0 = L + rowadr[3*p];
    const mjtNum* restrict L1 = L + rowadr[3*p+1];
    const mjtNum* restrict L2 = L + rowadr[3*p+2];
    const int* restrict ci = colind + rowadr[3*p];
    mjtNum s0 = 0, s1 = 0, s2 = 0;
    for (int j=0; j < nob; j += 3) {
      const mjtNum* rq = res + ci[j];
      mjtNum y0 = rq[0], y1 = rq[1], y2 = rq[2];
      s0 += L0[j]*y0 + L0[j+1]*y1 + L0[j+2]*y2;
      s1 += L1[j]*y0 + L1[j+1]*y1 + L1[j+2]*y2;
      s2 += L2[j]*y0 + L2[j+1]*y1 + L2[j+2]*y2;
    }
    mjtNum* r = res + 3*p;
    mjtNum x0 = (r[0] - s0) / L0[nob];
    mjtNum x1 = (r[1] - s1 - L1[nob]*x0) / L1[nob+1];
    mjtNum x2 = (r[2] - s2 - L2[nob]*x0 - L2[nob+1]*x1) / L2[nob+2];
    r[0] = x0;
    r[1] = x1;
    r[2] = x2;
  }
}

static void effCholApply(mjData* d, const mjEffChol* c, mjtNum* x, const mjtNum* rhs,
                         const mjtNum* Ls) {
  int nb = c->nb, n = c->n;
  mj_markStack(d);
  mjtNum* bc = mjSTACKALLOC(d, n, mjtNum);
  mjtNum* xc = mjSTACKALLOC(d, n, mjtNum);
  for (int k=0; k < nb; k++) {
    int i = c->dofid[k], p = 3*c->perm[k];
    bc[p] = rhs[i];
    bc[p+1] = rhs[i+1];
    bc[p+2] = rhs[i+2];
  }
  effCholSolveBlocked(xc, Ls, bc, c);
  for (int k=0; k < nb; k++) {
    int i = c->dofid[k], p = 3*c->perm[k];
    x[i] = xc[p];
    x[i+1] = xc[p+1];
    x[i+2] = xc[p+2];
  }
  mj_freeStack(d);
}

// analyse the step's efm_K (+ passive contact) pattern and allocate d->efm_L with the analysis
// and the factor
static mjtNum* effCholReserve(const mjModel* m, mjData* d, int nb, const int* dofid, int* ntail) {
  *ntail = 0;
  if (!effCholApplies(m, d, nb, dofid)) {
    return NULL;
  }
  int nv = m->nv;
  size_t kcap = effCholKcap(d, nb, dofid);
  size_t avail = mj_stackBytesAvailable(d);
  size_t base_bytes = mj_stackFrameBytes() + effIntBytes(nv) + effIntBytes(nb);
  size_t count_bytes = d->nefmcon ? (mj_stackFrameBytes() + 2*effIntBytes(nb)) : 0;
  size_t sym_bytes = base_bytes + effCholAnalysisBytes(m, nb, kcap, /*np_con=*/0);
  if (avail < mjMAX(sym_bytes, base_bytes + count_bytes)) {
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return NULL;
  }
  mj_markStack(d);
  int* blk = effInts(d, nv);
  int* perm = effInts(d, nb);
  effCovered(nv, nb, dofid, blk);
  int np_con = 0;
  if (d->nefmcon) {
    mj_markStack(d);
    int* vmark = effInts(d, nb);
    int* vlist = effInts(d, nb);
    np_con = effCholContactPairs(d, blk, nb, vmark, vlist, NULL);
    mj_freeStack(d);
    if (np_con > 0) {
      size_t con_sym = base_bytes + effCholAnalysisBytes(m, nb, kcap, (size_t)np_con);
      if (avail < con_sym) {
        mj_freeStack(d);
        mj_warning(d, mjWARN_CNSTRFULL, d->narena);
        return NULL;
      }
    }
  }
  int pos = 0;
  for (int t=0; t < m->nefmCvert && m->efmC_perm[t] >= 0; t++) {
    int k = m->efmC_perm[t] < nv ? blk[m->efmC_perm[t]] : -1;
    if (k >= 0 && dofid[k] == m->efmC_perm[t]) {
      perm[k] = pos++;
    }
  }
  mjEffCholPass pass;
  int ok = (pos == nb) &&
           effCholBegin(m, d, nb, dofid, blk, d->efm_K_rownnz, d->efm_K_rowadr, d->efm_K_colind,
                        np_con, /*order=*/0, perm, &pass) &&
           pass.nfe;
  if (!ok) {
    mj_freeStack(d);
    return NULL;
  }
  size_t npack = effPackNums(nb, pass.nL);
  if (avail < effCholReserveBytes(nb, nv, pass.nL, npack)) {
    mj_freeStack(d);
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return NULL;
  }
  int tail = EFF_HDR + (int)npack + pass.nL;
  if (mj_stackBytesAvailable(d) < effNumBytes((size_t)tail)) {
    mj_freeStack(d);
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return NULL;
  }
  mjtNum* B = (mjtNum*)effAlloc(d, sizeof(mjtNum)*(size_t)tail, _Alignof(mjtNum));
  if (!B) {
    mj_freeStack(d);
    return NULL;
  }
  B[0] = EFF_CHOL_PENDING;
  int* A = (int*)(B + EFF_HDR);
  A[0] = pass.nL;
  mjEffChol c;
  effPackLayout(A, nb, dofid, &c);
  memcpy(c.perm, perm, sizeof(int)*nb);
  effCholFinish(&pass, &c);
  mj_freeStack(d);
  *ntail = tail;
  return B;
}

static int effCholReady(const mjModel* m, mjData* d, int is_island, mjEffChol* cs) {
  if (!effCholReserved(d)) {
    return 0;
  }
  if (is_island && d->efm_L[0] == EFF_CHOL_PENDING && d->threadlock) {
    return 0;
  }
  effCholEnsure(m, d);
  if (d->efm_L[0] != 1) {
    return 0;
  }
  effCholStep(d, cs);
  if (mj_stackBytesAvailable(d) < mj_stackFrameBytes() + 2*effNumBytes(cs->n)) {
    return 0;
  }
  return 1;
}

static int effCholSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  mjEffChol cs;
  if (!effCholReady(m, d, /*is_island=*/0, &cs)) {
    return 0;
  }
  effCholApply(d, &cs, x, b, effCholStepValues(d, &cs));
  return 1;
}


// Record the covered vertex triples (d->efm_dofid, d->nefmdof) of the flex part of (M + K), and
// for 1D rods and 2D flexes reserve the sparse Cholesky factor of the covered block (effCholReserve).
static void effBlocks(const mjModel* m, mjData* d) {
  int nv = m->nv;

  // covered dofs come in contiguous triples (the 3 slide dofs of one flex point), but the first
  // one need not be at a multiple of 3: any joint declared before the flex shifts them. Walk the
  // covered rows rather than striding the dof index, which would straddle point boundaries.
  int nb = effCoveredBlocks(m, nv, d->efm_K_rownnz, NULL);
  d->nefmdof = 0;
  int* adr = (int*) effAlloc(d, sizeof(int)*(nb > 0 ? nb : 1), _Alignof(int));
  effCoveredBlocks(m, nv, d->efm_K_rownnz, adr);
  int ntail = 0;
  mjtNum* B = effCholReserve(m, d, nb, adr, &ntail);
  d->efm_L = B;
  d->efm_dofid = adr;
  d->nefmdof = nb;
  d->nefmL = ntail;
}

static int flexIsland(const mjModel* m, const mjData* d, int f);

// find k in sorted dofid[*lo..nb) with dofid[k] == dof, advancing *lo; returns -1 if absent
static int effFindDof(const int* dofid, int nb, int dof, int* lo) {
  int l = *lo, h = nb;
  while (l < h) {
    int mid = (l + h) >> 1;
    if (dofid[mid] < dof) {
      l = mid + 1;
    } else {
      h = mid;
    }
  }
  *lo = l;
  return (l < nb && dofid[l] == dof) ? l : -1;
}

// length of the per-dof diagonal fold for covered flex triples and 1D DER rod twist chains,
// 0 if none
int mj_effFoldDiagSize(const mjModel* m, const mjData* d, int island) {
  if (!d->efm_active) {
    return 0;
  }
  if (d->nefmdof > 0) {
    if (island < 0) {
      return m->nv;
    }
    mjEffIsland isl;
    if (effIslandStep(d, &isl)) {
      if (isl.cov_adr[island] < isl.cov_adr[island + 1]) {
        return d->island_nv[island];
      }
    } else {
      const int* idof2dof = d->map_idof2dof + d->island_idofadr[island];
      int inv = d->island_nv[island];
      for (int c = 0, lo = 0; c + 2 < inv; c++) {
        int i = idof2dof[c];
        if (d->efm_K_rownnz[i] > 0 && effFindDof(d->efm_dofid, d->nefmdof, i, &lo) >= 0) {
          return inv;
        }
      }
    }
  }
  int passive = !(mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER));
  if (!passive || !mj_effCouplings(m)) {
    return 0;
  }
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
  for (int f = 0; f < m->nflex; f++) {
    if (m->flex_rigid[f] || !mj_rodHasTwist(m, f)) {
      continue;
    }
    if (island >= 0) {
      if (flexIsland(m, d, f) == island) {
        return d->island_nv[island];
      }
    } else if (!sleep_filter || mj_sleepState(m, d, mjOBJ_FLEX, f) == mjS_AWAKE) {
      return m->nv;
    }
  }
  return 0;
}


// the twist dofs of 1D DER rod f lie on chain edges [*e0, *e1] (the clamped ends have none);
// returns 1 if each is a single-dof tree, so that effTwistSolve can solve the chain exactly
static int effTwistChain(const mjModel* m, int f, int* e0, int* e1) {
  int nhinge = (int)m->flex_bending[m->flex_bendingadr[f]];
  *e0 = -1;
  *e1 = -2;
  for (int e = 0; e <= nhinge; e++) {
    int dof = mj_rodChainTwistDof(m, f, e);
    if (dof < 0) {
      if (*e0 >= 0 && *e1 < *e0) {
        *e1 = e - 1;
      }
      continue;
    }
    if (*e1 >= *e0 && *e0 >= 0) {
      return 0;   // a gap in the chain
    }
    if (*e0 < 0) {
      *e0 = e;
    }
    if (m->M_rownnz[dof] != 1 || m->tree_dofnum[m->dof_treeid[dof]] != 1) {
      return 0;
    }
  }
  if (*e0 < 0) {
    return 0;
  }
  if (*e1 < *e0) {
    *e1 = nhinge;
  }
  return 1;
}


// exact O(nedge) symmetric tridiagonal LDL^T solve on 1D DER rod twist chains (island < 0: global)
static void effTwistSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* rhs, int island,
                          const mjtNum* tw_diag) {
  int passive = !(mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER));
  if (!passive || !mj_effCouplings(m) || !d->efm_sdiag) {
    return;
  }
  mjtNum h_dt = m->opt.timestep;
  mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h_dt*h_dt;
  mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h_dt;
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
  int idofadr = (island >= 0) ? d->island_idofadr[island] : 0;

  for (int f = 0; f < m->nflex; f++) {
    if (m->flex_rigid[f] || !mj_rodHasTwist(m, f)) {
      continue;
    }
    if (island >= 0) {
      if (flexIsland(m, d, f) != island) {
        continue;
      }
    } else if (sleep_filter && mj_sleepState(m, d, mjOBJ_FLEX, f) != mjS_AWAKE) {
      continue;
    }

    mjtNum scale_f = s1 + s2 * mj_effFlexDamping(m, f);
    const mjtNum* b = m->flex_bending + m->flex_bendingadr[f];
    int nhinge = (int)b[0];
    if (scale_f <= 0 || nhinge <= 0) {
      continue;
    }
    int e_start, e_end;
    if (!effTwistChain(m, f, &e_start, &e_end)) {
      continue;
    }
    int ntw = e_end - e_start + 1;

    size_t req = mj_stackFrameBytes() + mj_stackBytes(sizeof(int)*(size_t)ntw, _Alignof(int)) +
                 2*mj_stackBytes(sizeof(mjtNum)*(size_t)ntw, _Alignof(mjtNum)) +
                 (ntw > 1 ? mj_stackBytes(sizeof(mjtNum)*(size_t)(ntw - 1), _Alignof(mjtNum)) : 0);
    if (mj_stackBytesAvailable(d) < req) {
      continue;
    }
    mj_markStack(d);
    int* dofs = mjSTACKALLOC(d, ntw, int);
    mjtNum* diag = mjSTACKALLOC(d, ntw, mjtNum);
    mjtNum* off = (ntw > 1) ? mjSTACKALLOC(d, ntw - 1, mjtNum) : NULL;
    mjtNum* z = mjSTACKALLOC(d, ntw, mjtNum);
    int valid = 1;
    for (int k = 0; k < ntw; k++) {
      int dof = mj_rodChainTwistDof(m, f, e_start + k);
      if (island >= 0) {
        int lidof = d->map_dof2idof[dof] - idofadr;
        if (lidof < 0 || lidof >= d->island_nv[island]) {
          valid = 0;
          break;
        }
        dofs[k] = lidof;
        z[k] = rhs[lidof];
      } else {
        dofs[k] = dof;
        z[k] = rhs[dof];
      }
      int mdiag = m->M_rowadr[dof];
      mjtNum d_extra = tw_diag ? tw_diag[dofs[k]] : 0;
      diag[k] = mju_max(mjMINVAL, d->M[mdiag] + d->efm_sdiag[dof] + d_extra);
    }
    if (valid) {
      for (int k = 0; k < ntw - 1; k++) {
        int dof0, dof1;
        off[k] = -effTwistHinge(m, f, e_start + k, s1, s2, &dof0, &dof1);
      }
      for (int k = 0; k < ntw - 1; k++) {
        mjtNum lk = off[k] / diag[k];
        off[k] = lk;
        diag[k + 1] = mju_max(mjMINVAL, diag[k + 1] - lk * lk * diag[k]);
        z[k + 1] -= lk * z[k];
      }
      z[ntw - 1] /= diag[ntw - 1];
      for (int k = ntw - 2; k >= 0; k--) {
        z[k] = z[k] / diag[k] - off[k] * z[k + 1];
      }
      for (int k = 0; k < ntw; k++) {
        x[dofs[k]] = z[k];
      }
    }
    mj_freeStack(d);
  }
}


// L: folded per-dof diagonal (mj_effPrecFold), or NULL
static void effBlockApply(const mjModel* m, mjData* d, const mjtNum* L, mjtNum* x,
                          const mjtNum* b) {
  int nv = m->nv;
  int nbd = m->nefm0dof;
  int flg_bend = nbd && !d->nefmdof;

  // every dof a covered triple: folded diagonal (CG) or sparse factor (1D/2D) is the whole
  // preconditioner
  if (3*d->nefmdof == nv && !flg_bend) {
    if (L) {
      int sleep_dof = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
      if (sleep_dof) {
        for (int k = 0; k < d->nefmdof; k++) {
          int i = d->efm_dofid[k];
          if (d->tree_awake[m->dof_treeid[i]] == 0) {
            mju_zero3(x + i);
          } else {
            x[i]   = b[i]   * L[i];
            x[i+1] = b[i+1] * L[i+1];
            x[i+2] = b[i+2] * L[i+2];
          }
        }
      } else {
        for (int r = 0; r < nv; r++) {
          x[r] = b[r] * L[r];
        }
      }
      return;
    }
    if (effCholSolve(m, d, x, b)) {
      return;
    }
  }
  int need_stack = (x == b) || flg_bend;
  if (need_stack) {
    mj_markStack(d);
  }
  const mjtNum* rhs = b;
  if (x == b) {
    mjtNum* buf = mjSTACKALLOC(d, nv, mjtNum);
    mju_copy(buf, b, nv);   // b aliases x, which the sweep below overwrites
    rhs = buf;
  }
  mjEffChol cs;
  int use_chol = !L && effCholReady(m, d, /*is_island=*/0, &cs);
  int use_fold = (L != NULL) && (d->nefmdof > 0);

  // dofs no factor covers: backbone solve, using the qH backbone factor when diagonal
  // terms exist, else qLD. PCG requires symmetry, so dofs covered by L, effChol, or efm0_L are
  // zeroed before the sweep
  mju_copy(x, rhs, nv);
  if (use_chol || use_fold) {
    for (int k = 0; k < d->nefmdof; k++) {
      mju_zero(x + d->efm_dofid[k], 3);
    }
  }
  if (flg_bend) {
    for (int i = 0; i < nbd; i++) {
      x[m->efm0_dofid[i]] = 0;
    }
  }
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < nv;
  if (mj_effUseQH(d)) {
    mj_solveLD(x, d->qH, d->qHDiagInv, sleep_filter ? d->nv_awake : nv, 1,
               m->M_rownnz, m->M_rowadr, m->M_colind, sleep_filter ? d->dof_awake_ind : NULL);
    if (d->efm_sdiag) {
      effTwistSolve(m, d, x, rhs, -1, L);
    }
  } else {
    mj_solveLD(x, d->qLD, d->qLDiagInv, sleep_filter ? d->nv_awake : nv, 1,
               m->M_rownnz, m->M_rowadr, m->M_colind, sleep_filter ? d->dof_awake_ind : NULL);
  }

  // folded per-dof diagonal (CG) or per-step sparse factor (1D rods and 2D flexes)
  if (use_fold) {
    int sleep_dof = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
    for (int k = 0; k < d->nefmdof; k++) {
      int i = d->efm_dofid[k];
      if (sleep_dof && d->tree_awake[m->dof_treeid[i]] == 0) {
        mju_zero3(x + i);
      } else {
        x[i]   = rhs[i]   * L[i];
        x[i+1] = rhs[i+1] * L[i+1];
        x[i+2] = rhs[i+2] * L[i+2];
      }
    }
  } else if (use_chol) {
    effCholApply(d, &cs, x, rhs, effCholStepValues(d, &cs));
  }

  // bending-only: exact (M + K_bend)^-1 on the dofs the constant factor covers
  if (flg_bend) {
    mjtNum* bfr = mjSTACKALLOC(d, nbd, mjtNum);
    mjtNum* bfz = mjSTACKALLOC(d, nbd, mjtNum);
    for (int i = 0; i < nbd; i++) {
      bfr[i] = rhs[m->efm0_dofid[i]];
    }
    mju_cholSolveSparse(bfz, m->efm0_L, bfr, nbd,
                        m->efm0_L_rownnz, m->efm0_L_rowadr, m->efm0_L_colind);
    for (int i = 0; i < nbd; i++) {
      x[m->efm0_dofid[i]] = bfz[i];
    }
  }
  if (need_stack) {
    mj_freeStack(d);
  }
}


// forward 3x3-blocked scalar SGS sweep on covered triples: u = (D + L)^-1 * rhs
static void effBlk3Fwd(const mjData* d, int nb, const int* restrict dofid,
                       const int* restrict kdiag, const mjtNum* restrict invD,
                       mjtNum* restrict u, const mjtNum* restrict rhs) {
  const int *rowadr = d->efm_K_rowadr, *colind = d->efm_K_colind;
  const mjtNum* val = d->efm_K_val;
  for (int k=0; k < nb; k++) {
    int i = dofid[k], ra0 = rowadr[i], pos = kdiag[k];
    const int* ci = colind + ra0;
    const mjtNum *v0 = val + ra0, *v1 = val + rowadr[i+1], *v2 = val + rowadr[i+2];
    mjtNum s0 = 0, s1 = 0, s2 = 0;
    for (int j=0; j < pos; j += 3) {
      const mjtNum* uc = u + ci[j];
      mjtNum u0 = uc[0], u1 = uc[1], u2 = uc[2];
      s0 += v0[j]*u0 + v0[j+1]*u1 + v0[j+2]*u2;
      s1 += v1[j]*u0 + v1[j+1]*u1 + v1[j+2]*u2;
      s2 += v2[j]*u0 + v2[j+1]*u1 + v2[j+2]*u2;
    }
    mjtNum u0 = (rhs[i] - s0) * invD[i];
    mjtNum u1 = (rhs[i+1] - s1 - v1[pos]*u0) * invD[i+1];
    mjtNum u2 = (rhs[i+2] - s2 - v2[pos]*u0 - v2[pos+1]*u1) * invD[i+2];
    u[i]   = u0;
    u[i+1] = u1;
    u[i+2] = u2;
  }
}


// true residual r = (D + L)*rt on covered triples
static void effBlk3Ert(const mjData* d, int nb, const int* restrict dofid,
                       const int* restrict kdiag, const mjtNum* restrict D,
                       mjtNum* restrict r, const mjtNum* restrict rt) {
  const int *rowadr = d->efm_K_rowadr, *colind = d->efm_K_colind;
  const mjtNum* val = d->efm_K_val;
  for (int k=0; k < nb; k++) {
    int i = dofid[k], ra0 = rowadr[i], pos = kdiag[k];
    const int* ci = colind + ra0;
    const mjtNum *v0 = val + ra0, *v1 = val + rowadr[i+1], *v2 = val + rowadr[i+2];
    mjtNum s0 = 0, s1 = 0, s2 = 0;
    for (int j=0; j < pos; j += 3) {
      const mjtNum* rc = rt + ci[j];
      mjtNum r0 = rc[0], r1 = rc[1], r2 = rc[2];
      s0 += v0[j]*r0 + v0[j+1]*r1 + v0[j+2]*r2;
      s1 += v1[j]*r0 + v1[j+1]*r1 + v1[j+2]*r2;
      s2 += v2[j]*r0 + v2[j+1]*r1 + v2[j+2]*r2;
    }
    mjtNum r0 = rt[i], r1 = rt[i+1], r2 = rt[i+2];
    r[i]   = s0 + D[i]*r0;
    r[i+1] = s1 + v1[pos]*r0 + D[i+1]*r1;
    r[i+2] = s2 + v2[pos]*r0 + v2[pos+1]*r1 + D[i+2]*r2;
  }
}


// backward 3x3-blocked scalar SGS sweep on covered triples: t = (D + L^T)^-1 * p, w = p - Dsub*t
static void effBlk3Bwd(const mjData* d, int nb, const int* restrict dofid,
                       const int* restrict kdiag, const mjtNum* restrict invD,
                       const mjtNum* restrict Dsub, mjtNum* restrict t,
                       mjtNum* restrict w, const mjtNum* restrict p) {
  const int *rowadr = d->efm_K_rowadr, *rownnz = d->efm_K_rownnz, *colind = d->efm_K_colind;
  const mjtNum* val = d->efm_K_val;
  for (int k=nb-1; k >= 0; k--) {
    int i = dofid[k], ra0 = rowadr[i], nnz = rownnz[i], pos = kdiag[k];
    const int* ci = colind + ra0;
    const mjtNum *v0 = val + ra0, *v1 = val + rowadr[i+1], *v2 = val + rowadr[i+2];
    mjtNum s0 = 0, s1 = 0, s2 = 0;
    for (int j=nnz-3; j >= pos + 3; j -= 3) {
      const mjtNum* tc = t + ci[j];
      mjtNum t0 = tc[0], t1 = tc[1], t2 = tc[2];
      s0 += v0[j]*t0 + v0[j+1]*t1 + v0[j+2]*t2;
      s1 += v1[j]*t0 + v1[j+1]*t1 + v1[j+2]*t2;
      s2 += v2[j]*t0 + v2[j+1]*t1 + v2[j+2]*t2;
    }
    mjtNum p0 = p[i], p1 = p[i+1], p2 = p[i+2];
    mjtNum t2 = (p2 - s2) * invD[i+2];
    mjtNum t1 = (p1 - s1 - v1[pos+2]*t2) * invD[i+1];
    mjtNum t0 = (p0 - s0 - v0[pos+1]*t1 - v0[pos+2]*t2) * invD[i];
    t[i]   = t0;
    t[i+1] = t1;
    t[i+2] = t2;
    w[i]   = p0 - Dsub[i]*t0;
    w[i+1] = p1 - Dsub[i+1]*t1;
    w[i+2] = p2 - Dsub[i+2]*t2;
  }
}


// half-solve dst = L_U^-T * D_U^-1 * src on uncovered trees U (dst may alias src)
static void effUncovFwd(const mjModel* m, const mjData* d, int nU, const int* restrict u_dofs,
                        const mjtNum* restrict invD, int sleep_filter, mjtNum* dst,
                        const mjtNum* src) {
  for (int idx=0; idx < nU; idx++) {
    int i = u_dofs[idx];
    dst[i] = src[i];
  }
  for (int idx=nU-1; idx >= 0; idx--) {
    int i = u_dofs[idx];
    if (sleep_filter && d->tree_awake[m->dof_treeid[i]] == 0) {
      continue;
    }
    int mstart = m->M_rowadr[i], mdiag = mstart + m->M_rownnz[i] - 1;
    mjtNum vi = dst[i];
    if (mstart < mdiag && vi) {
      for (int a=mstart; a < mdiag; a++) {
        dst[m->M_colind[a]] -= d->qH[a] * vi;
      }
    }
  }
  for (int idx=0; idx < nU; idx++) {
    int i = u_dofs[idx];
    dst[i] *= invD[i];
  }
}


// solve (M + B) x = r_io by split-preconditioned CG using symmetric Gauss-Seidel (w=1) with
// Eisenstat's trick on standalone 3-dof flex triples (d->efm_dofid) and exact qH LDL^T on
// remaining trees (set U). Reuses d->efm_K in place with zero extra factor memory; non-CSR
// couplings (rank-1 tendons/actuators/contacts/twist, matrix-free flexes) are applied outside the
// split. On entry r_io holds b and x is zero; on return 1, x holds the solution, *capped is
// cleared on convergence or breakdown, and r_io holds the true residual b - A*x whenever *capped
// remains set or convergence was checked (not recomputed on early breakdown). Returns 0 if not
// applicable or stack is insufficient (caller falls back to standard PCG).
static int effSolveSGS(const mjModel* m, mjData* d, mjtNum* x, mjtNum* r_io, mjtNum bn,
                       mjtNum tol, int* capped) {
  int nv = m->nv, nb = d->nefmdof;
  if (!nb || efmGeneralCSR(d)) {
    return 0;
  }
  mjEffRank1Iter it_ext = {0};
  mjEffRank1 e_ext;
  size_t mf_scratch = mj_effMulAddScratch(m, d);
  int has_rank1 = mj_effRank1Next(m, d, &it_ext, &e_ext, /*flg_contact=*/1);
  int has_mf = mf_scratch > 0;
  int nU_max = nv - 3*nb;
  size_t need = mj_stackFrameBytes() + 8*effNumBytes(nv) + effIntBytes(nb) + effIntBytes(nU_max) +
                mj_stackBytes(sizeof(mjtByte)*(size_t)nv, _Alignof(mjtByte)) + mf_scratch;
  if (mj_stackBytesAvailable(d) < need) {
    return 0;
  }

  mj_markStack(d);
  const int* dofid = d->efm_dofid;
  const int *K_rowadr = d->efm_K_rowadr, *K_rownnz = d->efm_K_rownnz;
  const int* K_colind = d->efm_K_colind;
  const mjtNum* K_val = d->efm_K_val;
  mjtByte* in_C = mjSTACKALLOC(d, nv, mjtByte);
  memset(in_C, 0, sizeof(mjtByte)*(size_t)nv);
  int* kdiag = effInts(d, nb);

  // verify standalone 3-dof triples (decoupled from U, diagonal M + efm_fluid) and locate diagonal
  for (int k=0; k < nb; k++) {
    int i = dofid[k];
    if (m->tree_dofnum[m->dof_treeid[i]] != 3) {
      mj_freeStack(d);
      return 0;
    }
    for (int r=0; r < 3; r++) {
      int mstart = m->M_rowadr[i+r], mdiag = mstart + m->M_rownnz[i+r] - 1;
      for (int a=mstart; a < mdiag; a++) {
        if (d->M[a] != 0 || (d->efm_fluid && d->efm_fluid[a] != 0)) {
          mj_freeStack(d);
          return 0;
        }
      }
    }
    int nnz = K_rownnz[i];
    const int* ci = K_colind + K_rowadr[i];
    int lo = 0, hi = nnz / 3;
    while (lo < hi) {
      int mid = (lo + hi) >> 1;
      if (ci[3*mid] < i) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    int pos = 3*lo;
    if (pos + 2 >= nnz || ci[pos] != i) {
      mj_freeStack(d);
      return 0;
    }
    kdiag[k] = pos;
    in_C[i] = in_C[i+1] = in_C[i+2] = 1;
  }
  int nU = 0;
  int* u_dofs = effInts(d, nU_max);
  if (nU_max > 0) {
    for (int i=0; i < nv; i++) {
      if (!in_C[i]) {
        u_dofs[nU++] = i;
      }
    }
  }
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < nv;

  mjtNum* rt   = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* p    = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* t    = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* w    = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* q    = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* D    = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* invD = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* Dsub = mjSTACKALLOC(d, nv, mjtNum);

  // fold non-contact rank-1 diagonals into D, invD, Dsub = 2*D - D0 on covered triples
  // (passive contacts stay outside the split, matching qH in mj_effActuation: folding their
  // rank-1 outer-product diagonals without off-diagonals crushes orthogonal-mode eigenvalues)
  for (int k=0; k < nb; k++) {
    int i = dofid[k], pos = kdiag[k];
    for (int r=0; r < 3; r++) {
      int row = i + r, mdiag = m->M_rowadr[row] + m->M_rownnz[row] - 1;
      mjtNum diag_df = (d->efm_diag ? d->efm_diag[row] : 0) +
                       (d->efm_fluid ? d->efm_fluid[mdiag] : 0);
      mjtNum d0 = d->M[mdiag] + diag_df + K_val[K_rowadr[row] + pos + r];
      mjtNum extra = d->efm_sdiag ? mju_max(0, d->efm_sdiag[row] - diag_df) : 0;
      mjtNum di = mju_max(mjMINVAL, d0 + extra);
      D[row] = di;
      invD[row] = 1 / di;
      Dsub[row] = 2*di - d0;
    }
  }

  // uncovered trees U use exact qH factor L_U^T * D_U * L_U (E_U = L_U^T * D_U, E_U^T = D_U * L_U)
  for (int idx=0; idx < nU; idx++) {
    int i = u_dofs[idx];
    if (sleep_filter && d->tree_awake[m->dof_treeid[i]] == 0) {
      D[i] = 1;
      invD[i] = 1;
    } else {
      mjtNum inv = mju_max(mjMINVAL, d->qHDiagInv[i]);
      invD[i] = inv;
      D[i] = 1 / inv;
    }
  }

  // initial transformed residual rt = E^-1 * b
  effBlk3Fwd(d, nb, dofid, kdiag, invD, rt, r_io);
  if (nU > 0) {
    effUncovFwd(m, d, nU, u_dofs, invD, sleep_filter, rt, r_io);
  }

  // initial p = tilde_D * rt, rho = rt' * p
  mjtNum rho = 0;
  for (int i=0; i < nv; i++) {
    p[i] = D[i] * rt[i];
    rho += rt[i] * p[i];
  }
  mjtNum ratio = (rho > 0) ? (bn / rho) : 1;
  mjtNum rho_check = rho;

  for (int it=0; it < m->opt.iterations; it++) {
    // backward solve t = E^-T * p and Eisenstat RHS w = p - Dsub*t on C
    effBlk3Bwd(d, nb, dofid, kdiag, invD, Dsub, t, w, p);

    // uncovered trees U: t_U = L_U^-1 * D_U^-1 * p_U
    for (int idx=0; idx < nU; idx++) {
      int i = u_dofs[idx];
      w[i] = 0;
      if (sleep_filter && d->tree_awake[m->dof_treeid[i]] == 0) {
        t[i] = p[i];
        continue;
      }
      mjtNum ti = p[i] * invD[i];
      int mstart = m->M_rowadr[i], mdiag = mstart + m->M_rownnz[i] - 1;
      for (int a=mstart; a < mdiag; a++) {
        ti -= d->qH[a] * t[m->M_colind[a]];
      }
      t[i] = ti;
    }

    // non-CSR couplings on all dofs and (M + efm_fluid + efm_diag)*t on U
    if (has_rank1) {
      effMulAddRank1(m, d, w, t, /*flg_contact=*/1);
    }
    if (has_mf) {
      effMulAddMatrixFree(m, d, w, t);
    }
    for (int idx=0; idx < nU; idx++) {
      int i = u_dofs[idx];
      int mstart = m->M_rowadr[i], mdiag = mstart + m->M_rownnz[i] - 1;
      mjtNum ti = t[i];
      mjtNum acc = (d->M[mdiag] + (d->efm_diag ? d->efm_diag[i] : 0) +
                    (d->efm_fluid ? d->efm_fluid[mdiag] : 0)) * ti;
      for (int a=mstart; a < mdiag; a++) {
        int c = m->M_colind[a];
        mjtNum mv = d->efm_fluid ? (d->M[a] + d->efm_fluid[a]) : d->M[a];
        acc += mv * t[c];
        w[c] += mv * ti;
      }
      w[i] += acc;
    }

    // forward solve u = E^-1 * w on C, and q_U = E_U^-1 * (A*t)_U on U
    effBlk3Fwd(d, nb, dofid, kdiag, invD, q, w);
    if (nU > 0) {
      effUncovFwd(m, d, nU, u_dofs, invD, sleep_filter, q, w);
    }

    // q = t + u on C, inner product pq = p'*q = t'*A*t
    for (int k=0; k < nb; k++) {
      int i = dofid[k];
      q[i]   += t[i];
      q[i+1] += t[i+1];
      q[i+2] += t[i+2];
    }
    mjtNum pq = mju_dot(p, q, nv);
    if (pq <= 0) {
      *capped = 0;
      break;
    }
    mjtNum alpha = rho / pq;
    mjtNum rho_new = 0;
    for (int i=0; i < nv; i++) {
      x[i] += alpha * t[i];
      mjtNum rti = rt[i] - alpha * q[i];
      rt[i] = rti;
      rho_new += D[i] * rti * rti;
    }
    if (rho_new <= 0) {
      *capped = 0;
      break;
    }

    // adaptive true-residual check: r = E*rt when rho_new nears tol or drops 100x in norm
    if (ratio * rho_new < 4 * tol || rho_new < 1e-4 * rho_check ||
        it + 1 == m->opt.iterations) {
      effBlk3Ert(d, nb, dofid, kdiag, D, r_io, rt);
      for (int idx=0; idx < nU; idx++) {
        int i = u_dofs[idx];
        mjtNum ri = D[i] * rt[i];
        r_io[i] = ri;
        if (sleep_filter && d->tree_awake[m->dof_treeid[i]] == 0) {
          continue;
        }
        int mstart = m->M_rowadr[i], mdiag = mstart + m->M_rownnz[i] - 1;
        if (mstart < mdiag && ri) {
          for (int a=mstart; a < mdiag; a++) {
            r_io[m->M_colind[a]] += d->qH[a] * ri;
          }
        }
      }
      mjtNum rr = mju_dot(r_io, r_io, nv);
      if (rr < tol) {
        *capped = 0;
        break;
      }
      ratio = rr / rho_new;
      rho_check = rho_new;
    }
    mjtNum beta = rho_new / rho;
    for (int i=0; i < nv; i++) {
      p[i] = D[i]*rt[i] + beta*p[i];
    }
    rho = rho_new;
  }

  mj_freeStack(d);
  return 1;
}


// stack bytes mj_effLower takes on an island of size nv with nr1 rank-1 rows and nR nonzeros
size_t mj_effLowerScratch(int nv, int nr1, int nR) {
  if (!nr1 || !nR) {
    return 0;
  }
  size_t n_int = 2*(size_t)nv + (size_t)(nr1 + 2) + 2*(size_t)nR;
  size_t n_num = 2*(size_t)nR;
  return mj_stackFrameBytes() + effIntBytes(n_int) + effNumBytes(n_num);
}


// upper bound on nonzeros in lower-triangular CSR of S (or M + S if flg_addM); writes rank-1
// row count and nonzero count into *nr1_out and *nR_out
int mj_effLowerCap(const mjModel* m, mjData* d, int island, int flg_addM,
                   int* nr1_out, int* nR_out) {
  int nv = island >= 0 ? d->island_nv[island] : m->nv;
  int idofadr = island >= 0 ? d->island_idofadr[island] : 0;
  const int* idof2dof = island >= 0 ? d->map_idof2dof + idofadr : NULL;
  int nr1 = 0, nR = 0, tlow = 0;
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    nr1++;
    nR += e.nnz;
    tlow += e.nnz * (e.nnz + 1) / 2;
  }
  *nr1_out = nr1;
  *nR_out = nR;

  int kcap = d->nefmK / 2;
  if (island >= 0 && d->nefmK) {
    int ksum = 0;
    for (int i=0; i < nv; i++) {
      ksum += d->efm_K_rownnz[idof2dof[i]];
    }
    kcap = ksum / 2;
  }
  int m_offdiag = 0;
  if (flg_addM || d->efm_fluid) {
    if (island >= 0) {
      for (int i=0; i < nv; i++) {
        m_offdiag += m->M_rownnz[idof2dof[i]] - 1;
      }
    } else {
      m_offdiag = m->nC - nv;
    }
  }
  if (!nr1 || !nR) {
    return kcap + nv + m_offdiag;
  }

  // exact deduplicated lower-triangle count via sparse marker array when stack permits
  size_t count_ints = 3*(size_t)nv + (size_t)(nr1 + 1) + 2*(size_t)nR;
  if (mj_stackBytesAvailable(d) < mj_stackFrameBytes() + effIntBytes(count_ints)) {
    return kcap + tlow + nv + m_offdiag;
  }

  mj_markStack(d);
  int* cint      = effInts(d, count_ints);
  int* R_rowadr  = cint;
  int* R_colind  = R_rowadr + nr1 + 1;
  int* RT_rownnz = R_colind + nR;
  int* RT_rowadr = RT_rownnz + nv;
  int* RT_rowid  = RT_rowadr + nv;
  int* marker    = RT_rowid + nR;
  mju_zeroInt(RT_rownnz, nv);

  int radr = 0, rid = 0;
  it = (mjEffRank1Iter){0};
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    R_rowadr[rid++] = radr;
    for (int j=0; j < e.nnz; j++) {
      int r = e.colind[j];
      int lr = island >= 0 ? d->map_dof2idof[r] - idofadr : r;
      if (lr < 0 || lr >= nv) {
        mjERROR("metric rank-1 term spans islands on dof %d", r);  // SHOULD NOT OCCUR
      }
      R_colind[radr++] = lr;
      RT_rownnz[lr]++;
    }
  }
  R_rowadr[nr1] = radr;

  int tadr = 0;
  for (int i=0; i < nv; i++) {
    RT_rowadr[i] = tadr;
    tadr += RT_rownnz[i];
    RT_rownnz[i] = 0;
    marker[i] = -1;
  }
  for (int r=0; r < nr1; r++) {
    for (int a=R_rowadr[r]; a < R_rowadr[r+1]; a++) {
      int lr = R_colind[a];
      RT_rowid[RT_rowadr[lr] + RT_rownnz[lr]++] = r;
    }
  }

  int cap = 0;
  for (int i=0; i < nv; i++) {
    int gdof = island >= 0 ? idof2dof[i] : i;
    if (d->nefmK) {
      int fa = d->efm_K_rowadr[gdof], fe = fa + d->efm_K_rownnz[gdof];
      for (int a=fa; a < fe; a++) {
        int lc = island >= 0 ? d->map_dof2idof[d->efm_K_colind[a]] - idofadr : d->efm_K_colind[a];
        if (lc >= i) {
          break;
        }
        if (marker[lc] != i) {
          marker[lc] = i;
          cap++;
        }
      }
    }
    if (flg_addM || d->efm_fluid) {
      int ga = m->M_rowadr[gdof], ge = ga + m->M_rownnz[gdof];
      for (int a=ga; a < ge; a++) {
        int lc = island >= 0 ? d->map_dof2idof[m->M_colind[a]] - idofadr : m->M_colind[a];
        if (lc >= i) {
          break;
        }
        if (marker[lc] != i) {
          marker[lc] = i;
          cap++;
        }
      }
    }
    int te = RT_rowadr[i] + RT_rownnz[i];
    for (int k=RT_rowadr[i]; k < te; k++) {
      int r = RT_rowid[k];
      for (int a=R_rowadr[r]; a < R_rowadr[r+1]; a++) {
        int lc = R_colind[a];
        if (lc < i && marker[lc] != i) {
          marker[lc] = i;
          cap++;
        }
      }
    }
    cap++;  // diagonal slot
  }

  mj_freeStack(d);
  return cap;
}


// build lower-triangular + diagonal CSR of discrete metric S (or M + S if flg_addM), returning nnz
int mj_effLower(const mjModel* m, mjData* d, int island, int flg_addM, int nr1, int nR,
                int* S_rownnz, int* S_rowadr, int* S_colind, mjtNum* S_val) {
  int nv = island >= 0 ? d->island_nv[island] : m->nv;
  int idofadr = island >= 0 ? d->island_idofadr[island] : 0;
  const int* idof2dof = island >= 0 ? d->map_idof2dof + idofadr : NULL;

  // zero-scratch two-way merge when no rank-1 terms exist
  if (!nr1 || !nR) {
    int adr = 0;
    for (int i=0; i < nv; i++) {
      S_rowadr[i] = adr;
      int gdof = island >= 0 ? idof2dof[i] : i;
      mjtNum diag = d->efm_diag ? d->efm_diag[gdof] : 0;
      int fa = d->nefmK ? d->efm_K_rowadr[gdof] : 0;
      int fe = fa + (d->nefmK ? d->efm_K_rownnz[gdof] : 0);
      int ga = (flg_addM || d->efm_fluid) ? m->M_rowadr[gdof] : 0;
      int ge = ga + ((flg_addM || d->efm_fluid) ? m->M_rownnz[gdof] : 0);
      while (fa < fe || ga < ge) {
        int cf = fa < fe ?
                 (island >= 0 ? d->map_dof2idof[d->efm_K_colind[fa]] - idofadr
                              : d->efm_K_colind[fa]) : nv;
        int cg = ga < ge ?
                 (island >= 0 ? d->map_dof2idof[m->M_colind[ga]] - idofadr
                              : m->M_colind[ga]) : nv;
        int col = cf < cg ? cf : cg;
        mjtNum val = 0;
        if (cf == col) {
          val += d->efm_K_val[fa++];
        }
        if (cg == col) {
          if (flg_addM) {
            val += d->M[ga];
          }
          if (d->efm_fluid) {
            val += d->efm_fluid[ga];
          }
          ga++;
        }
        if (col < i) {
          S_colind[adr] = col;
          S_val[adr] = val;
          adr++;
        } else {
          if (col == i) {
            diag += val;
          }
          break;
        }
      }
      if (diag || flg_addM) {
        S_colind[adr] = i;
        S_val[adr] = diag;
        adr++;
      }
      S_rownnz[i] = adr - S_rowadr[i];
    }
    return adr;
  }

  size_t n_int = 2*(size_t)nv + (size_t)(nr1 + 2) + 2*(size_t)nR;
  size_t n_num = 2*(size_t)nR;

  mj_markStack(d);
  int* sint = effInts(d, n_int);
  mjtNum* snum = mjSTACKALLOC(d, n_num, mjtNum);

  int* RT_rownnz = sint;
  int* RT_rowadr = RT_rownnz + nv;
  int* R_rowadr  = RT_rowadr + nv + 1;
  int* R_colind  = R_rowadr + nr1 + 1;
  int* RT_rowid  = R_colind + nR;
  mjtNum* R_val  = snum;
  mjtNum* RT_sval = R_val + nR;
  mju_zeroInt(RT_rownnz, nv);

  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  int radr = 0, rid = 0;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    if (rid >= nr1 || radr + e.nnz > nR) {
      mjERROR("metric rank-1 count exceeded mj_effLowerCap (%d/%d rows, %d/%d nnz)",
              rid + 1, nr1, radr + e.nnz, nR);  // SHOULD NOT OCCUR
    }
    R_rowadr[rid++] = radr;
    for (int j=0; j < e.nnz; j++) {
      int r = e.colind[j];
      int lr = island >= 0 ? d->map_dof2idof[r] - idofadr : r;
      if (lr < 0 || lr >= nv) {
        mjERROR("metric rank-1 term spans islands on dof %d", r);  // SHOULD NOT OCCUR
      }
      R_colind[radr] = lr;
      R_val[radr] = e.val[j];
      radr++;
      RT_rownnz[lr]++;
    }
  }
  R_rowadr[rid] = radr;

  int tadr = 0;
  for (int i=0; i < nv; i++) {
    RT_rowadr[i] = tadr;
    tadr += RT_rownnz[i];
    RT_rownnz[i] = 0;
  }
  RT_rowadr[nv] = tadr;
  it = (mjEffRank1Iter){0};
  rid = 0;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    int r = rid++;
    for (int j=0; j < e.nnz; j++) {
      int lr = R_colind[R_rowadr[r] + j];
      int w = RT_rowadr[lr] + RT_rownnz[lr]++;
      RT_rowid[w] = r;
      RT_sval[w] = e.scale * e.val[j];
    }
  }

  // reuse RT_rownnz as per-column slot marker into S_val (-1: absent from current row)
  int* marker = RT_rownnz;
  for (int i=0; i < nv; i++) {
    marker[i] = -1;
  }

  // accumulate each row directly into S_colind/S_val, sorting only unique column indices
  int adr = 0;
  for (int i=0; i < nv; i++) {
    int start = adr;
    S_rowadr[i] = start;
    int gdof = island >= 0 ? idof2dof[i] : i;
    mjtNum diag = d->efm_diag ? d->efm_diag[gdof] : 0;

    if (d->nefmK) {
      int fa = d->efm_K_rowadr[gdof], fe = fa + d->efm_K_rownnz[gdof];
      for (int a=fa; a < fe; a++) {
        int lc = island >= 0 ? d->map_dof2idof[d->efm_K_colind[a]] - idofadr : d->efm_K_colind[a];
        mjtNum v = d->efm_K_val[a];
        if (lc < i) {
          int slot = adr++;
          marker[lc] = slot;
          S_colind[slot] = lc;
          S_val[slot] = v;
        } else {
          if (lc == i) {
            diag += v;
          }
          break;
        }
      }
    }

    if (flg_addM || d->efm_fluid) {
      int ga = m->M_rowadr[gdof], ge = ga + m->M_rownnz[gdof];
      for (int a=ga; a < ge; a++) {
        int lc = island >= 0 ? d->map_dof2idof[m->M_colind[a]] - idofadr : m->M_colind[a];
        mjtNum v = (flg_addM ? d->M[a] : 0) + (d->efm_fluid ? d->efm_fluid[a] : 0);
        if (lc < i) {
          int slot = marker[lc];
          if (slot < 0) {
            slot = adr++;
            marker[lc] = slot;
            S_colind[slot] = lc;
            S_val[slot] = v;
          } else {
            S_val[slot] += v;
          }
        } else {
          if (lc == i) {
            diag += v;
          }
          break;
        }
      }
    }

    int te = RT_rowadr[i+1];
    for (int k=RT_rowadr[i]; k < te; k++) {
      int r = RT_rowid[k];
      mjtNum sJr = RT_sval[k];
      for (int a=R_rowadr[r]; a < R_rowadr[r+1]; a++) {
        int lc = R_colind[a];
        if (lc < i) {
          mjtNum v = sJr * R_val[a];
          int slot = marker[lc];
          if (slot < 0) {
            slot = adr++;
            marker[lc] = slot;
            S_colind[slot] = lc;
            S_val[slot] = v;
          } else {
            S_val[slot] += v;
          }
        } else if (lc == i) {
          diag += sJr * R_val[a];
        }
      }
    }

    for (int a=start; a < adr; a++) {
      marker[S_colind[a]] = -1;
    }
    mju_sortSparseRow(adr - start, S_colind + start, S_val + start);

    if (diag || flg_addM) {
      S_colind[adr] = i;
      S_val[adr] = diag;
      adr++;
    }
    S_rownnz[i] = adr - S_rowadr[i];
  }

  mj_freeStack(d);
  return adr;
}


static int flexMatrixFree(const mjModel* m, const mjData* d, int f, int interp_asm, int gen_csr);


// return 1 if mj_effMulAdd applies any matrix-free flex operator not assembled into d->efm_K
static int effMatrixFree(const mjModel* m, const mjData* d) {
  int gen_csr = efmGeneralCSR(d);
  if (gen_csr || (mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER))) {
    return 0;
  }
  int interp_asm = d->nefmK && mjd_flexInterpAssemblable(m);
  for (int f=0; f < m->nflex; f++) {
    if (flexMatrixFree(m, d, f, interp_asm, gen_csr)) {
      return 1;
    }
  }
  return 0;
}


// return 1 if dof falls inside any 3-dof block covered by d->efm_dofid
static int effCholHasDof(const int* dofid, int nb, int dof) {
  int lo = 0, hi = nb - 1;
  while (lo <= hi) {
    int mid = lo + (hi - lo)/2;
    int start = dofid[mid];
    if (dof < start) {
      hi = mid - 1;
    } else if (dof >= start + 3) {
      lo = mid + 1;
    } else {
      return 1;
    }
  }
  return 0;
}


// return 1 if effChol + qH cover all covered-tree inertia blocks and active passive contact,
// tendon, and actuator rank-1 terms without omitting off-diagonals or covered-block diagonals
static int effCholCoversMetric(const mjModel* m, const mjData* d) {
  int nb = d->nefmdof;
  const int* dofid = d->efm_dofid;

  // covered vertex bodies with child joints have M couplings outside their 3x3 block
  for (int k=0; k < nb; k++) {
    if (m->tree_dofnum[m->dof_treeid[dofid[k]]] != 3) {
      return 0;
    }
  }

  // passive flex contacts: every dof of an active contact must lie in a covered block
  if (d->nefmcon && 3*nb < m->nv) {
    for (int adr=0; adr < d->nefmcon; ) {
      int nnz = d->efm_con_ind[adr];
      mjtNum scale = d->efm_con_val[adr];
      const int* ci = d->efm_con_ind + adr + 2;
      adr += 2 + nnz;
      if (scale <= 0) {
        continue;
      }
      for (int a=0; a < nnz; a++) {
        if (!effCholHasDof(dofid, nb, ci[a])) {
          return 0;
        }
      }
    }
  }

  // tendons: single-dof terms are exact in effChol/qH; multi-dof couplings need effSolveNewton
  for (int t=0; t < d->nefmT; t++) {
    if (d->efm_ts[t] && m->ten_J_rownnz[d->efm_tid[t]] > 1) {
      return 0;
    }
  }

  // actuators: single-dof terms are exact in effChol/qH; multi-dof couplings need effSolveNewton
  for (int t=0; t < d->nefmA; t++) {
    if (!d->efm_as[t]) {
      continue;
    }
    int i = d->efm_aid[t];
    int oadr = m->actuator_outadr[i];
    for (int k=0; k < m->actuator_outnum[i]; k++) {
      if (d->moment_rownnz[oadr + k] > 1) {
        return 0;
      }
    }
  }

  return 1;
}


// solve (M + S) x = b directly by sparse or dense Cholesky; returns 0 if stack space is exhausted
static int effSolveNewton(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  int nv = m->nv;
  int nr1 = 0, nR = 0;
  int nHcap = mj_effLowerCap(m, d, /*island=*/-1, /*flg_addM=*/1, &nr1, &nR);
  size_t lower_scratch = mj_effLowerScratch(nv, nr1, nR);
  size_t h_int_cnt = 2*(size_t)nv + (size_t)nHcap;
  size_t need = mj_stackFrameBytes() + effIntBytes(h_int_cnt) + effNumBytes((size_t)nHcap) +
                lower_scratch;
  if (mj_stackBytesAvailable(d) < need) {
    return 0;
  }

  mj_markStack(d);
  int* H_ints = effInts(d, h_int_cnt);
  mjtNum* H_val = mjSTACKALLOC(d, nHcap > 0 ? nHcap : 1, mjtNum);
  int* H_rownnz = H_ints;
  int* H_rowadr = H_ints + nv;
  int* H_colind = H_ints + 2*nv;
  int nH = mj_effLower(m, d, /*island=*/-1, /*flg_addM=*/1, nr1, nR,
                       H_rownnz, H_rowadr, H_colind, H_val);

  // dense Cholesky
  if (!mj_isSparse(m)) {
    size_t dense_nums = (size_t)nv * (size_t)nv + (x == b ? (size_t)nv : 0);
    if (mj_stackBytesAvailable(d) < effNumBytes(dense_nums)) {
      mj_freeStack(d);
      return 0;
    }
    mjtNum* dense_buf = mjSTACKALLOC(d, dense_nums > 0 ? dense_nums : 1, mjtNum);
    mjtNum* L = dense_buf;
    mju_zero(L, nv*nv);
    mju_addToSymSparse(L, H_val, nv, H_rownnz, H_rowadr, H_colind, /*flg_upper=*/0);
    mju_cholFactor(L, nv, mjMINVAL);
    const mjtNum* rhs = b;
    if (x == b) {
      mjtNum* rhs_buf = dense_buf + (size_t)nv*nv;
      mju_copy(rhs_buf, b, nv);
      rhs = rhs_buf;
    }
    mju_cholSolve(x, L, rhs, nv);
    mj_freeStack(d);
    return 1;
  }

  // transpose lower-triangle H to upper-triangle HT and run symbolic Cholesky counting
  size_t sym_int_cnt = 6*(size_t)nv + (size_t)nH;
  size_t chol_count_bytes = mj_stackFrameBytes() + 2*effIntBytes((size_t)nv);
  if (mj_stackBytesAvailable(d) < effIntBytes(sym_int_cnt) + chol_count_bytes) {
    mj_freeStack(d);
    return 0;
  }
  int* sym_ints  = effInts(d, sym_int_cnt);
  int* HT_rownnz = sym_ints;
  int* HT_rowadr = sym_ints + nv;
  int* L_rownnz  = sym_ints + 2*nv;
  int* L_rowadr  = sym_ints + 3*nv;
  int* LT_rownnz = sym_ints + 4*nv;
  int* LT_rowadr = sym_ints + 5*nv;
  int* HT_colind = sym_ints + 6*nv;
  mju_transposeSparse(NULL, NULL, nv, nv, HT_rownnz, HT_rowadr, HT_colind, NULL,
                      H_rownnz, H_rowadr, H_colind, NULL);
  int nL = mju_cholFactorSymbolic(NULL, L_rownnz, L_rowadr, NULL, LT_rownnz, LT_rowadr, NULL, NULL,
                                  HT_rownnz, HT_rowadr, HT_colind, nv, d);

  // allocate L structures, run symbolic fill, numeric Cholesky, and solve
  size_t nL_int = 3*(size_t)nL + 2*(size_t)nv;
  size_t nL_num = (size_t)nL + (x == b ? (size_t)nv : 0);
  size_t chol_fill_bytes = mj_stackFrameBytes() + 3*effIntBytes((size_t)nv);
  if (mj_stackBytesAvailable(d) < effIntBytes(nL_int) + effNumBytes(nL_num) + chol_fill_bytes) {
    mj_freeStack(d);
    return 0;
  }
  int* L_ints     = effInts(d, nL_int);
  mjtNum* L_nums  = mjSTACKALLOC(d, nL_num > 0 ? nL_num : 1, mjtNum);
  int* L_colind   = L_ints;
  int* LT_colind  = L_ints + nL;
  int* LT_map     = L_ints + 2*nL;
  int* L_super    = L_ints + 3*nL;
  int* cholscratch = L_ints + 3*nL + nv;
  mjtNum* L       = L_nums;

  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz, LT_rowadr, LT_map,
                         L_super, HT_rownnz, HT_rowadr, HT_colind, nv, d);
  mju_cholFactorNumeric(L, nv, mjMINVAL, L_rownnz, L_rowadr, L_colind,
                        LT_rownnz, LT_rowadr, LT_colind, LT_map, L_super,
                        H_val, H_rownnz, H_rowadr, H_colind, cholscratch);

  const mjtNum* rhs = b;
  if (x == b) {
    mjtNum* rhs_buf = L_nums + nL;
    mju_copy(rhs_buf, b, nv);
    rhs = rhs_buf;
  }
  mju_cholSolveSparse(x, L, rhs, nv, L_rownnz, L_rowadr, L_colind);

  mj_freeStack(d);
  return 1;
}


// accurate solve of (M + K) x = b: directly by Cholesky under solver=Newton (3x3-block effChol
// when it covers the metric, else general sparse or dense Cholesky via effSolveNewton), or by PCG
// with the metric preconditioner (with block-SGS under CG) converging the relative residual to
// opt.tolerance; used for qacc_smooth. Reaching opt.iterations means the metric is too
// ill-conditioned for the preconditioner: warn (mjWARN_INERTIA, worst-residual dof) and return x
// under-converged.
void mj_effSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  if (!d->efm_active) {
    mj_effPrec(m, d, x, b);
    return;
  }

  // backbone-only metric (no tendon, actuator, contact or flex couplings): the qH solve inside the
  // block-apply is the exact metric solve, skip the iteration. Beyond the saved iterations,
  // the direct solve is block-decoupled across trees, which sleeping trees rely on
  if (!d->nefmT && !d->nefmA && !d->nefmcon && !mj_effFlexAny(m)) {
    effBlockApply(m, d, NULL, x, b);
    return;
  }

  int nv = m->nv;
  mjtNum bn = mju_dot(b, b, nv);
  if (bn <= mjMINVAL) {
    mju_zero(x, nv);
    if (d->threadpool) {
      effCholEnsure(m, d);
    }
    return;
  }

  // Newton: solve (M + S) directly by Cholesky unless 3x3-block effChol already covers the metric
  // (falls through to PCG only if a flex remains matrix-free or stack space is exhausted);
  // defer effCholReady so d->efm_L is not filled when effSolveNewton will solve from scratch
  int chol_ready = -1;
  if (m->opt.solver == mjSOL_NEWTON && !effMatrixFree(m, d)) {
    int chol_covers = m->opt.iterations > 0 && effCholReserved(d) && !efmGeneralCSR(d) &&
                      effCholCoversMetric(m, d);
    if (chol_covers) {
      mjEffChol cs;
      chol_ready = effCholReady(m, d, /*is_island=*/0, &cs);
      chol_covers = chol_ready;
    }
    if (!chol_covers && effSolveNewton(m, d, x, b)) {
      return;
    }
  }
  if (chol_ready < 0) {
    mjEffChol cs;
    chol_ready = effCholReady(m, d, /*is_island=*/0, &cs);
  }

  mj_markStack(d);
  mjtNum* r = mjSTACKALLOC(d, nv, mjtNum);
  mju_copy(r, b, nv);   // before zeroing x: b may alias x
  mju_zero(x, nv);

  // converge the relative residual to opt.tolerance; both sides are squared norms
#ifdef mjUSESINGLE
  // float cannot reach a 1e-8 relative residual (eps ~1.2e-7): without a floor every step of
  // every covered model would run to opt.iterations and then warn.
  mjtNum tolerance = mju_max(m->opt.tolerance, 1e-5);
#else
  mjtNum tolerance = m->opt.tolerance;
#endif
  mjtNum tol = tolerance*tolerance*bn;
  int capped = 1;   // cleared by either exit below; still set means the cap was reached
  int use_sgs = d->nefmK && !chol_ready;
  if (!use_sgs || !effSolveSGS(m, d, x, r, bn, tol, &capped)) {
    mjtNum* z = mjSTACKALLOC(d, nv, mjtNum);
    mjtNum* p = mjSTACKALLOC(d, nv, mjtNum);
    mjtNum* Ap = mjSTACKALLOC(d, nv, mjtNum);
    effBlockApply(m, d, NULL, z, r);
    mju_copy(p, z, nv);
    mjtNum rz = mju_dot(r, z, nv);
    for (int it = 0; it < m->opt.iterations; it++) {
      mju_mulSymVecSparse(Ap, d->M, p, nv, m->M_rownnz, m->M_rowadr, m->M_colind);
      mj_effMulAdd(m, d, Ap, p, /*flg_contact=*/1);
      mjtNum pAp = mju_dot(p, Ap, nv);
      // curvature breakdown: the metric has no curvature along p, so no further progress is
      // possible and x is the best available. Not a budget failure, so it does not warn.
      if (pAp <= 0) { capped = 0; break; }
      mjtNum alpha = rz/pAp;
      mju_addToScl(x, p, alpha, nv);
      mju_addToScl(r, Ap, -alpha, nv);
      if (mju_dot(r, r, nv) < tol) { capped = 0; break; }
      effBlockApply(m, d, NULL, z, r);
      mjtNum rznew = mju_dot(r, z, nv);
      mju_addScl(p, z, p, rznew/rz, nv);
      rz = rznew;
    }
  }

  // Ran out of iterations with the residual still above tolerance: the metric is too
  // ill-conditioned for the blocks to solve within the budget. Blame the dof carrying the
  // largest residual.
  // mjWARN_INERTIA is the closest existing warning (reusing it avoids an ABI addition), but on
  // its own it points the user at their inertia when the cause is the flex stiffness, so say so
  // first. Gate on the same first-time condition mj_warning uses, or a model that fails every
  // step would print this thousands of times a second.
  if (capped && mju_dot(r, r, nv) >= tol) {
    int worst = 0;
    for (int i = 1; i < nv; i++) {
      if (mju_abs(r[i]) > mju_abs(r[worst])) worst = i;
    }
    if (!d->warning[mjWARN_INERTIA].number) {
      mju_warning("Flex stiffness is too ill-conditioned for the effective-metric block "
                  "preconditioner: the M+K solve ran out of iterations at a relative residual "
                  "of %.2e and qacc_smooth is under-converged. Reported as a singular inertia "
                  "below, because M+K is the effective inertia.",
                  mju_sqrt(mju_dot(r, r, nv)/bn));
    }
    mj_warning(d, mjWARN_INERTIA, worst);
  }
  mj_freeStack(d);
}

void mj_effPrec(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  int nv = m->nv;

  // active metric: apply the step's metric preconditioner
  if (d->efm_active) {
    effBlockApply(m, d, NULL, x, b);
    return;
  }

  // inactive metric: x = M \ b, which is exact
  if (x != b) {
    mju_copy(x, b, nv);
  }
  mj_solveLD(x, d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
}


//------------------ folded per-dof diagonal of the CG preconditioner ------------------------------
// The CG constraint solve iterates on M + K + rank-1 classes + J'*D*J. mj_effPrecFold folds the
// passive-contact and efc row diagonals (for every row with D > 0, active or not) into a
// solver-owned per-dof diagonal L (mj_effFoldDiagSize): covered flex triples store the inverted
// total diagonal 1 / (M_diag + efm_sdiag + K_diag + K_con_diag + J'DJ_diag), while 1D DER rod
// twist chains read the additive contact/efc diagonal in effTwistSolve.

static void effInvertTripleDiag(const mjModel* m, const mjData* d, int i, int li, mjtNum* L) {
  for (int r = 0; r < 3; r++) {
    int row = i + r, lr = li + r;
    mjtNum diag = d->M[m->M_rowadr[row] + m->M_rownnz[row] - 1] +
                  (d->efm_sdiag ? d->efm_sdiag[row] : 0) + L[lr];
    int adr = d->efm_K_rowadr[row], nnz = d->efm_K_rownnz[row];
    for (int j = adr; j < adr + nnz; j++) {
      if (d->efm_K_colind[j] == row) {
        diag += d->efm_K_val[j];
        break;
      }
    }
    L[lr] = 1 / mju_max(mjMINVAL, diag);
  }
}


// fold the passive-contact and efc row diagonals into L (mj_effFoldDiagSize) for covered flex
// triples and 1D DER rod twist chains; returns 1 when folded, 0 if none
int mj_effPrecFold(const mjModel* m, mjData* d, mjtNum* L, int island, int nefc,
                   const mjtNum* efc_D, int is_sparse, const mjtNum* J,
                   const int* J_rownnz, const int* J_rowadr, const int* J_colind) {
  int nfold = mj_effFoldDiagSize(m, d, island);
  int ncol = island >= 0 ? d->island_nv[island] : m->nv;
  if (!nfold || !d->efm_active || ncol <= 0) {
    return 0;
  }
  int idofadr = island >= 0 ? d->island_idofadr[island] : 0;
  mju_zero(L, nfold);

  // passive-contact diagonal
  mjEffRank1Iter it = {.cls = 2};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (it.cls != 2) {
      break;
    }
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    for (int a=0; a < e.nnz; a++) {
      int la = island >= 0 ? d->map_dof2idof[e.colind[a]] - idofadr : e.colind[a];
      L[la] += e.scale * e.val[a] * e.val[a];
    }
  }

  // efc rows: fold diag(J'*D*J) for every row with D > 0, active or satisfied
  for (int r=0; r < nefc; r++) {
    mjtNum D = efc_D[r];
    if (!D) {
      continue;
    }
    if (is_sparse) {
      int adr = J_rowadr[r], nnz = J_rownnz[r];
      for (int a=0; a < nnz; a++) {
        int la = J_colind[adr+a];
        L[la] += D * J[adr+a] * J[adr+a];
      }
    } else {
      const mjtNum* Jr = J + (size_t)r*ncol;
      for (int c=0; c < ncol; c++) {
        if (Jr[c]) {
          L[c] += D * Jr[c] * Jr[c];
        }
      }
    }
  }

  // convert covered flex triples (disjoint from 1-dof twist chains) to 1 / total_diag
  if (d->nefmdof > 0) {
    if (island < 0) {
      for (int k = 0; k < d->nefmdof; k++) {
        int i = d->efm_dofid[k];
        effInvertTripleDiag(m, d, i, i, L);
      }
    } else {
      mjEffIsland isl;
      if (effIslandStep(d, &isl)) {
        int q_end = isl.cov_adr[island + 1];
        for (int q = isl.cov_adr[island]; q < q_end; q++) {
          int i = d->efm_dofid[isl.cov_ind[q]];
          effInvertTripleDiag(m, d, i, d->map_dof2idof[i] - idofadr, L);
        }
      } else {
        const int* idof2dof = d->map_idof2dof + idofadr;
        for (int c = 0, lo = 0; c + 2 < ncol; ) {
          int i = idof2dof[c];
          if (d->efm_K_rownnz[i] > 0 && effFindDof(d->efm_dofid, d->nefmdof, i, &lo) >= 0) {
            effInvertTripleDiag(m, d, i, c, L);
            c += 3;
          } else {
            c++;
          }
        }
      }
    }
  }
  return 1;
}


// mj_effPrec with the folded blocks L (mj_effPrecFold) in place of the step's factor
void mj_effPrecBlocks(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b, const mjtNum* L) {
  if (d->efm_active) {
    effBlockApply(m, d, L, x, b);
    return;
  }
  mj_effPrec(m, d, x, b);
}


// the step's metric buffer holds the sparse factor, which leaves no room for the per-island lists
// (effIslandBuild) that keep the island-local metric operators off O(nefmdof) scans: a CG solve
// then runs monolithically, unless sleep needs the island path
int mj_effCGMonolithic(const mjModel* m, const mjData* d, int solver) {
  return solver == mjSOL_CG && d->efm_active && !mjENABLED(mjENBL_SLEEP) && effCholReserved(d);
}


// stack bytes of mj_effMulAdd and mj_effMulAddIsland, at most: the frames of the matrix-free
// flex operators they fall back to, one at a time, plus the island scatter/gather frame
size_t mj_effMulAddScratch(const mjModel* m, const mjData* d) {
  if (efmGeneralCSR(d)) {
    return 0;
  }
  size_t frame = mj_stackFrameBytes(), bytes = 0;
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_interp[f] || m->flex_rigid[f]) continue;
    if (m->flex_dim[f] == 1 && m->flex_bendingadr[f] < 0) continue;
    if (!d->nefmK || !mj_flexSimple(m, f)) {
      // bending and stretch: the gathered vector and result
      size_t b = frame + 2*mj_stackBytes(sizeof(mjtNum)*3*m->flex_vertnum[f], _Alignof(mjtNum));
      bytes = mjMAX(bytes, b);
    }
  }
  if (!d->nefmK || !mjd_flexInterpAssemblable(m)) {
    size_t b = mjd_flexInterp_mulBytes(m);
    bytes = mjMAX(bytes, b);
  }
  if (bytes && !mjDISABLED(mjDSBL_ISLAND) && !mjENABLED(mjENBL_IPC)) {
    bytes += frame + 2*mj_stackBytes(sizeof(mjtNum)*m->nv, _Alignof(mjtNum));
  }
  return bytes;
}


static size_t effCholApplyScratch(const mjModel* m, const mjData* d) {
  if (!effCholReserved(d)) {
    return 0;
  }
  mjEffChol c;
  effCholStep(d, &c);
  return mjMAX(mj_stackFrameBytes() + 2*effNumBytes(c.n), effCholFillBytes(&c, m->nv));
}


// stack bytes of mj_effPrec and mj_effPrecIsland, at most
size_t mj_effPrecScratch(const mjModel* m, const mjData* d) {
  size_t sn = sizeof(mjtNum), an = _Alignof(mjtNum);
  size_t twist_apply = 0;
  for (int f = 0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && mj_rodHasTwist(m, f)) {
      int ntw = (int)m->flex_bending[m->flex_bendingadr[f]] + 1;
      if (ntw >= 1) {
        size_t req = mj_stackFrameBytes() + effIntBytes(ntw) + 2*effNumBytes(ntw) +
                     (ntw > 1 ? effNumBytes(ntw - 1) : 0);
        twist_apply = mjMAX(twist_apply, req);
      }
    }
  }
  size_t apply = mj_stackFrameBytes() + mj_stackBytes(sn*m->nv, an) +
                 mjMAX(effCholApplyScratch(m, d), twist_apply);
  if (m->nefm0dof && !d->nefmdof) {
    apply += 2*mj_stackBytes(sn*m->nefm0dof, an);
  }
  return apply;
}


// can this dof contribute a damping term to the metric diagonal (model-level check;
// values are velocity-dependent and computed in mj_effShift)
static int dofDampPossible(const mjModel* m, int i) {
  if (mjDISABLED(mjDSBL_DAMPER)) {
    return 0;
  }
  return m->dof_damping[i] > 0 ||
         !mju_isZero(m->dof_dampingpoly + mjNPOLY*i, mjNPOLY) ||
         m->jnt_actuatorid[m->dof_jntid[i]] != -1;
}


// fill efm_ck with h * d(spring force)/d(displacement) per joint dof, clamped PSD-safe,
// mirroring the joint-spring structure of mj_springdamper (1D DER rod twist bending stiffness
// is filled by mj_flexPassiveBend1D in the passive pass before mj_effShift); returns any-nonzero
static int effDiagStiff(const mjModel* m, mjData* d) {
  int any = 0, njnt = m->njnt;
  mjtNum h = m->opt.timestep;
  mju_zero(d->efm_ck, m->nv);
  if (mjDISABLED(mjDSBL_SPRING)) {
    return 0;
  }

  for (int j=0; j < njnt; j++) {
    mjtNum stiffness = m->jnt_stiffness[j];
    const mjtNum* spoly = m->jnt_stiffnesspoly + mjNPOLY*j;
    if (stiffness == 0 && mju_isZero(spoly, mjNPOLY)) {
      continue;
    }

    int padr = m->jnt_qposadr[j];
    int dadr = m->jnt_dofadr[j];
    const mjtNum* qspring = m->jnt_plasticadr[j] >= 0 ?
                            d->plastic + m->jnt_plasticadr[j] :
                            m->qpos_spring + padr;
    mjtNum k;

    switch ((mjtJoint) m->jnt_type[j]) {
    case mjJNT_FREE:
      // translation: radial derivative of the spring, isotropic on the 3 slide dofs
      {
        mjtNum dif[3];
        mju_sub3(dif, d->qpos+padr, qspring);
        k = mju_max(0, mjd_xPolyForce(stiffness, spoly, mju_norm3(dif), mjNPOLY, 0));
        d->efm_ck[dadr] = d->efm_ck[dadr+1] = d->efm_ck[dadr+2] = h*k;
        any = any || k > 0;
      }

      // continue with rotations
      dadr += 3;
      padr += 3;
      qspring += 3;
      mjFALLTHROUGH;

    case mjJNT_BALL:
      // rotation: small-angle isotropic treatment of the quaternion spring
      {
        mjtNum dif[3], quat[4];
        mju_copy4(quat, d->qpos+padr);
        mju_normalize4(quat);
        mju_subQuat(dif, quat, qspring);
        k = mju_max(0, mjd_xPolyForce(stiffness, spoly, mju_norm3(dif), mjNPOLY, 0));
        d->efm_ck[dadr] = d->efm_ck[dadr+1] = d->efm_ck[dadr+2] = h*k;
        any = any || k > 0;
      }
      break;

    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      {
        mjtNum x = d->qpos[padr] - qspring[0];
        k = mju_max(0, mjd_xPolyForce(stiffness, spoly, x, mjNPOLY, 0));
        d->efm_ck[dadr] = h*k;
        any = any || k > 0;
      }
      break;
    }
  }

  return any;
}


// finalize efm_diag = h*D + h^2*K: the damping derivative is velocity-dependent,
// evaluated here at the velocity stage, matching mj_EulerSkip's coverage but clamped
static void effDiagDamp(const mjModel* m, mjData* d) {
  int nv = m->nv;
  mjtNum h = m->opt.timestep;
  int damper = !mjDISABLED(mjDSBL_DAMPER);

  for (int i=0; i < nv; i++) {
    mjtNum damp_deriv = 0;
    if (damper) {
      mjtNum poly[mjNPOLY];
      mju_copy(poly, m->dof_dampingpoly + mjNPOLY*i, mjNPOLY);
      mjtNum damping = m->dof_damping[i]
                       + mj_actuatorDamping(m, mjOBJ_JOINT, m->dof_jntid[i], poly);
      damp_deriv = mju_max(0, mjd_xPolyForce(damping, poly, d->qvel[i], mjNPOLY, 1));
    }
    d->efm_diag[i] = h*damp_deriv + h*d->efm_ck[i];
  }
}


// refresh the velocity-stage values of the active metric: the smooth-force shift
// c = -h*K*qvel and the diagonal (its damping part is velocity-dependent). No allocation
// here, mirroring the efc value refresh pattern. Called from the velocity stage only,
// after the derived velocities (ten_velocity, body velocities) it reads are computed;
// every consumer of the values runs at the actuation stage or later
void mj_effShift(const mjModel* m, mjData* d) {
  if (!d->efm_active) {
    return;
  }
  mjtNum h = m->opt.timestep;
  mju_zero(d->efm_c, m->nv);

  // flex stiffness shift, absent when the spring force is disabled
  if (!mjDISABLED(mjDSBL_SPRING)) {
    mjd_flexInterp_mul(m, d, d->efm_c, d->qvel, h, 0, d->flexelem_krot);
    mjd_flexBend_mul(m, d, d->efm_c, d->qvel, -h, 0);
    mjd_flexStretch_mul(m, d, d->efm_c, d->qvel, -h, 0);

    // 1D DER rod torsional coupling shift: -h * 2*kt * (qvel[dof1] - qvel[dof0]); the bending
    // stiffness of the twist angles is diagonal, in efm_ck
    if (mj_effCouplings(m)) {
      int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
      for (int f = 0; f < m->nflex; f++) {
        if (m->flex_rigid[f] || !mj_rodHasTwist(m, f) ||
            (sleep_filter && mj_sleepState(m, d, mjOBJ_FLEX, f) != mjS_AWAKE)) {
          continue;
        }
        const mjtNum* b = m->flex_bending + m->flex_bendingadr[f];
        int nhinge = (int)b[0];
        for (int hi = 0; hi < nhinge; hi++) {
          int dof0, dof1;
          mjtNum hk = effTwistHinge(m, f, hi, h, 0, &dof0, &dof1);
          if (!hk) {
            continue;
          }
          mjtNum w0 = (dof0 >= 0) ? d->qvel[dof0] : 0;
          mjtNum w1 = (dof1 >= 0) ? d->qvel[dof1] : 0;
          mjtNum ckv = hk * (w1 - w0);
          if (dof1 >= 0) {
            d->efm_c[dof1] -= ckv;
          }
          if (dof0 >= 0) {
            d->efm_c[dof0] += ckv;
          }
        }
      }
    }
  }

  // contact: -h*K_contact*v from the packed rank-1 rows (see effContactBuild), so the shift
  // and the operator apply one definition of contact. Each row's scale already carries the h^2
  // of the effective stiffness, so the shift's -h*K*v is -(1/h) * scale * row'(row.v)
  for (int adr=0; adr < d->nefmcon; ) {
    int nnz = d->efm_con_ind[adr];
    const int* colind = d->efm_con_ind + adr + 2;
    const mjtNum* val = d->efm_con_val + adr + 2;
    mjtNum dot = 0;
    for (int a=0; a < nnz; a++) {
      dot += val[a] * d->qvel[colind[a]];
    }
    dot *= -d->efm_con_val[adr] / h;
    for (int a=0; a < nnz; a++) {
      d->efm_c[colind[a]] += dot * val[a];
    }
    adr += 2 + nnz;
  }

  // per-dof diagonal classes
  if (d->efm_diag) {
    int nv = m->nv;
    effDiagDamp(m, d);

    // stiffness shift
    for (int i=0; i < nv; i++) {
      d->efm_c[i] -= d->efm_ck[i] * d->qvel[i];
    }
  }

  // tendon values: stiffness at the current deadband displacement, damping at the current
  // tendon velocity, both clamped PSD-safe, mirroring the passive-force expressions
  for (int t=0; t < d->nefmT; t++) {
    int i = d->efm_tid[t];

    mjtNum k = 0;
    if (!mjDISABLED(mjDSBL_SPRING)) {
      mjtNum length = d->ten_length[i];
      const mjtNum* lengthspring = m->tendon_plasticadr[i] >= 0 ?
                                   d->plastic + m->tendon_plasticadr[i] :
                                   m->tendon_lengthspring + 2*i;
      mjtNum lower = lengthspring[0];
      mjtNum upper = lengthspring[1];
      mjtNum x = (length > upper) ? length - upper : (length < lower) ? length - lower : 0;
      if (x) {
        k = mju_max(0, mjd_xPolyForce(m->tendon_stiffness[i],
                                      m->tendon_stiffnesspoly + mjNPOLY*i, x, mjNPOLY, 0));
      }
    }

    mjtNum b = 0;
    if (!mjDISABLED(mjDSBL_DAMPER)) {
      mjtNum dpoly[mjNPOLY];
      mju_copy(dpoly, m->tendon_dampingpoly + mjNPOLY*i, mjNPOLY);
      mjtNum damping = m->tendon_damping[i] + mj_actuatorDamping(m, mjOBJ_TENDON, i, dpoly);
      b = mju_max(0, mjd_xPolyForce(damping, dpoly, d->ten_velocity[i], mjNPOLY, 1));
    }

    d->efm_ts[t] = h*h*k + h*b;
    d->efm_tk[t] = h*k;

    // stiffness shift: c -= h*k * ten_velocity * J'
    if (k) {
      mjtNum ckv = d->efm_tk[t] * d->ten_velocity[i];
      int end = m->ten_J_rowadr[i] + m->ten_J_rownnz[i];
      for (int j=m->ten_J_rowadr[i]; j < end; j++) {
        d->efm_c[m->ten_J_colind[j]] -= ckv * d->ten_J[j];
      }
    }
  }


  // fluid drag blocks: assemble the drag-only passive-fluid velocity derivative into stack
  // scratch in qDeriv's shape (qDeriv itself is user-facing), gather into M's sparsity
  // pattern, scale by -h. The inertia-box drag derivative is negative semidefinite term by
  // term; the ellipsoid one is not (its symmetric part is indefinite for slender shapes) and
  // is projected per geom (mjd_ellipsoidFluid). Lift and added-mass terms are excluded and
  // integrate explicitly
  if (d->efm_fluid) {
    mj_markStack(d);
    mjtNum* scratch = mjSTACKALLOC(d, m->nD, mjtNum);
    mju_zero(scratch, m->nD);
    int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
    int nbody = sleep_filter ? d->nbody_awake : m->nbody;
    for (int b=0; b < nbody; b++) {
      int i = sleep_filter ? d->body_awake_ind[b] : b;
      if (m->body_mass[i] < mjMINVAL) {
        continue;
      }
      int use_ellipsoid_model = 0;
      for (int j=0; j < m->body_geomnum[i] && use_ellipsoid_model == 0; j++) {
        use_ellipsoid_model += (m->geom_fluid[mjNFLUID*(m->body_geomadr[i] + j)] > 0);
      }
      if (use_ellipsoid_model) {
        mjd_ellipsoidFluid(m, d, scratch, i, /*flg_dragonly=*/1);
      } else {
        mjd_inertiaBoxFluid(m, d, scratch, i);
      }
    }
    mju_gather(d->efm_fluid, scratch, m->mapD2M, m->nC);
    mju_scl(d->efm_fluid, d->efm_fluid, -m->opt.timestep, m->nC);
    mj_freeStack(d);
  }

  // velocity-dependent diagonal/fluid updates invalidate a previously factored effChol
  // (e.g. mj_forwardSkip re-running the velocity stage without mj_effBuild)
  if ((d->efm_diag || d->efm_fluid) && effCholReserved(d) && d->efm_L[0] == 1) {
    d->efm_L[0] = EFF_CHOL_PENDING;
  }
}


// actuation-stage refresh of the metric: actuator gain scalars (ctrl- and state-dependent,
// so evaluated after mj_fwdActuation), their smooth-force shift, and the backbone factor
// qH = M + diag(h*D + h^2*K) + tendon and actuator diagonals. The backbone is the part of
// the metric inside M's kinematic-tree sparsity -- every class's diagonal projection plus
// the fluid blocks -- so mj_factorI factors it with no fill-in; it is the exact metric
// when no coupling terms exist, the preconditioner otherwise, and the only metric the dual
// solvers ever see (mj_makeY). Idempotent: actuation-stage objects are rebuilt from
// scratch, so re-running the stage (mj_forwardSkip) is safe. Under sleep, rows of sleeping
// trees may hold stale M values: harmless, M and its factor are block-diagonal by tree and
// only awake rows are ever gathered or solved. If flg_factor is 0 the backbone is assembled
// (efm_sdiag reads its diagonal) but not factored, leaving qH unfactored: inverse dynamics
// only multiplies by the metric, and needs the factor only for the exact constraint diagonal
void mj_effActuation(const mjModel* m, mjData* d, int flg_factor) {
  if (!d->efm_active) {
    return;
  }
  int nv = m->nv;
  mjtNum h = m->opt.timestep;

  // actuator gains, clamped to the stabilizing sign
  d->nefmA = 0;
  if (d->efm_aid) {
    mju_zero(d->efm_ca, nv);
    int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
    for (int i=0; i < m->nactuator; i++) {
      if (!mj_effActuatorPossible(m, i) || mjd_actuatorDerivSkip(m, d, i, sleep_filter)) {
        continue;
      }
      mjtNum gv = mju_max(0, -mjd_actuatorVelDeriv(m, d, i));
      mjtNum gp = mju_max(0, -mjd_actuatorLenDeriv(m, d, i));
      if (!gv && !gp) {
        continue;
      }
      int t = d->nefmA++;
      d->efm_aid[t] = i;
      d->efm_as[t] = h*h*gp + h*gv;
      d->efm_ak[t] = h*gp;

      // stiffness shift: ca -= h*gp * actuator_velocity * moment'
      if (gp) {
        int oadr = m->actuator_outadr[i];
        for (int k=0; k < m->actuator_outnum[i]; k++) {
          int r = oadr + k;
          mjtNum ckv = d->efm_ak[t] * d->actuator_velocity[r];
          int end = d->moment_rowadr[r] + d->moment_rownnz[r];
          for (int j=d->moment_rowadr[r]; j < end; j++) {
            d->efm_ca[d->moment_colind[j]] -= ckv * d->actuator_moment[j];
          }
        }
      }
    }
  }

  // factor the backbone qH = M + all metric diagonals
  if (mj_effUseQH(d)) {
    mju_copy(d->qH, d->M, m->nC);
    if (d->efm_diag) {
      for (int i=0; i < nv; i++) {
        d->qH[m->M_rowadr[i] + m->M_rownnz[i] - 1] += d->efm_diag[i];
      }
      for (int t=0; t < d->nefmT; t++) {
        mjtNum s = d->efm_ts[t];
        if (!s) {
          continue;
        }
        int i = d->efm_tid[t];
        int end = m->ten_J_rowadr[i] + m->ten_J_rownnz[i];
        for (int j=m->ten_J_rowadr[i]; j < end; j++) {
          int c = m->ten_J_colind[j];
          d->qH[m->M_rowadr[c] + m->M_rownnz[c] - 1] += s * d->ten_J[j] * d->ten_J[j];
        }
      }
      for (int t=0; t < d->nefmA; t++) {
        mjtNum s = d->efm_as[t];
        int i = d->efm_aid[t];
        int oadr = m->actuator_outadr[i];
        for (int k=0; k < m->actuator_outnum[i]; k++) {
          int r = oadr + k;
          int end = d->moment_rowadr[r] + d->moment_rownnz[r];
          for (int j=d->moment_rowadr[r]; j < end; j++) {
            int c = d->moment_colind[j];
            d->qH[m->M_rowadr[c] + m->M_rownnz[c] - 1] +=
                s * d->actuator_moment[j] * d->actuator_moment[j];
          }
        }
      }
      // 1D DER rod torsional coupling diagonal
      mjEffRank1Iter it_tw = {.cls = 3};
      mjEffRank1 e_tw;
      while (mj_effRank1Next(m, d, &it_tw, &e_tw, /*flg_contact=*/0)) {
        for (int j=0; j < e_tw.nnz; j++) {
          int c = e_tw.colind[j];
          d->qH[m->M_rowadr[c] + m->M_rownnz[c] - 1] += e_tw.scale * e_tw.val[j] * e_tw.val[j];
        }
      }
      // fluid drag blocks share M's sparsity: add the full pattern
      if (d->efm_fluid) {
        mju_addTo(d->qH, d->efm_fluid, m->nC);
      }

      // record the diagonal additions for metric-consistent regularization
      for (int i=0; i < nv; i++) {
        int diag = m->M_rowadr[i] + m->M_rownnz[i] - 1;
        d->efm_sdiag[i] = d->qH[diag] - d->M[diag];
      }
    }

    // flex stiffness CSR diagonal on standalone 3-dof slider trees joins the qH backbone factor
    // (articulated trees keep M + efm_sdiag so parent-child inertia couplings are not distorted by
    // adding diag(efm_K) without efm_K's off-diagonals; added after recording efm_sdiag so
    // diagapprox efc_diagA/efc_R/efc_D are unchanged, while mj_makeYNumeric whitens against d->qH
    // so diagexact and dual AR include this diagonal)
    if (d->nefmK) {
      for (int i=0; i < nv; ) {
        if (effSimpleTriple(m, i) && m->tree_dofnum[m->dof_treeid[i]] == 3) {
          for (int r=0; r < 3; r++) {
            int row = i + r;
            int adr = d->efm_K_rowadr[row], nnz = d->efm_K_rownnz[row];
            for (int j=adr; j < adr + nnz; j++) {
              if (d->efm_K_colind[j] == row) {
                d->qH[m->M_rowadr[row]+m->M_rownnz[row]-1] += d->efm_K_val[j];
                break;
              }
            }
          }
          i += 3;
        } else {
          i++;
        }
      }
    }

    if (flg_factor) {
      int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < nv;
      mj_factorI(d->qH, d->qHDiagInv, sleep_filter ? d->nv_awake : nv,
                 m->M_rownnz, m->M_rowadr, m->M_colind,
                 sleep_filter ? d->dof_awake_ind : NULL);
    }
  }

  // actuation-stage diagonal updates invalidate a previously factored effChol
  // (e.g. mj_forwardSkip re-running only the actuation stage)
  if (d->nefmA && effCholReserved(d) && d->efm_L[0] == 1) {
    d->efm_L[0] = EFF_CHOL_PENDING;
  }
}


// island of flex f, or -1 if none of its bodies belong to an island. All awake trees of a
// metric-coupled flex share one island (unionConstraintTrees), so the first dynamic body in an
// island decides membership
static int flexIsland(const mjModel* m, const mjData* d, int f) {
  int num = m->flex_interp[f] ? m->flex_nodenum[f] : m->flex_vertnum[f];
  int adr = m->flex_interp[f] ? m->flex_nodeadr[f] : m->flex_vertadr[f];
  const int* bodyid = m->flex_interp[f] ? m->flex_nodebodyid : m->flex_vertbodyid;
  for (int j=0; j < num; j++) {
    int tree = m->body_treeid[bodyid[adr + j]];
    if (tree >= 0 && d->tree_island[tree] >= 0) {
      return d->tree_island[tree];
    }
  }
  if (mj_rodHasTwist(m, f)) {
    int nedge = (int)m->flex_bending[m->flex_bendingadr[f]] + 1;
    for (int e = 0; e < nedge; e++) {
      int jnt = mj_rodChainTwistJnt(m, f, e);
      if (jnt >= 0) {
        int tree = m->body_treeid[m->jnt_bodyid[jnt]];
        if (tree >= 0 && d->tree_island[tree] >= 0) {
          return d->tree_island[tree];
        }
      }
    }
  }
  return -1;
}


// does flex f require the matrix-free stiffness path (not covered by the CSR)?
static int flexMatrixFree(const mjModel* m, const mjData* d, int f, int interp_asm, int gen_csr) {
  if (gen_csr) {
    return 0;
  }
  if (m->flex_interp[f]) {
    return !interp_asm && mjd_flexInterp_processed(m, f);
  }
  return mjd_flexStiff_activeGen(m, f, /*flg_bend=*/1, /*flg_stretch=*/1, /*flg_general=*/1) &&
         (!d->nefmK || !mj_flexSimple(m, f));
}


// build CSR-style per-island lists of covered triples, bending-only dofs, and matrix-free flexes
// in the tail of d->efm_L once per step, avoiding O(nefmdof + nefm0dof + nflex) scans per island
// per CG iteration
static void effIslandBuild(const mjModel* m, mjData* d) {
  int nisland = d->nisland;
  int nb = d->nefmdof;
  int ntail = effIslandTail(m, d, nb);
  d->efm_L = EFMALLOC(mjtNum, ntail);
  d->nefmL = ntail;

  mjtNum* tail = d->efm_L;
  mju_zero(tail, ntail);
  tail[0] = EFF_ISLAND_TAG;
  int* A = (int*)(tail + EFF_HDR);
  A[0] = nisland;
  A[1] = nb;
  A[2] = nb ? 0 : m->nefm0dof;
  A[3] = m->nflex;
  mjEffIsland isl;
  effIslandPackLayout(A, &isl);

  mju_zeroInt(isl.cov_adr, nisland + 1);
  for (int k=0; k < nb; k++) {
    int island = d->dof_island[d->efm_dofid[k]];
    if (island >= 0 && island < nisland) {
      isl.cov_adr[island + 1]++;
    }
  }
  for (int c=0; c < nisland; c++) {
    isl.cov_adr[c + 1] += isl.cov_adr[c];
  }
  for (int k=0; k < nb; k++) {
    int island = d->dof_island[d->efm_dofid[k]];
    if (island >= 0 && island < nisland) {
      isl.cov_ind[isl.cov_adr[island]++] = k;
    }
  }
  for (int c=nisland; c > 0; c--) {
    isl.cov_adr[c] = isl.cov_adr[c - 1];
  }
  isl.cov_adr[0] = 0;

  mju_zeroInt(isl.bend_adr, nisland + 1);
  if (!nb && m->nefm0dof) {
    for (int i=0; i < m->nefm0dof; i++) {
      int island = d->dof_island[m->efm0_dofid[i]];
      if (island >= 0 && island < nisland) {
        isl.bend_adr[island + 1]++;
      }
    }
    for (int c=0; c < nisland; c++) {
      isl.bend_adr[c + 1] += isl.bend_adr[c];
    }
    for (int i=0; i < m->nefm0dof; i++) {
      int island = d->dof_island[m->efm0_dofid[i]];
      if (island >= 0 && island < nisland) {
        isl.bend_ind[isl.bend_adr[island]++] = i;
      }
    }
    for (int c=nisland; c > 0; c--) {
      isl.bend_adr[c] = isl.bend_adr[c - 1];
    }
    isl.bend_adr[0] = 0;
  }

  mju_zeroInt(isl.flex_adr, nisland + 1);
  int gen_csr = efmGeneralCSR(d);
  int interp_asm = d->nefmK && (gen_csr || mjd_flexInterpAssemblable(m));
  for (int f=0; f < m->nflex; f++) {
    if (!flexMatrixFree(m, d, f, interp_asm, gen_csr)) {
      continue;
    }
    int island = flexIsland(m, d, f);
    if (island >= 0 && island < nisland) {
      isl.flex_adr[island + 1]++;
    }
  }
  for (int c=0; c < nisland; c++) {
    isl.flex_adr[c + 1] += isl.flex_adr[c];
  }
  for (int f=0; f < m->nflex; f++) {
    if (!flexMatrixFree(m, d, f, interp_asm, gen_csr)) {
      continue;
    }
    int island = flexIsland(m, d, f);
    if (island >= 0 && island < nisland) {
      isl.flex_ind[isl.flex_adr[island]++] = f;
    }
  }
  for (int c=nisland; c > 0; c--) {
    isl.flex_adr[c] = isl.flex_adr[c - 1];
  }
  isl.flex_adr[0] = 0;
}


// island-local metric product res += S*vec: the diagonal classes, fluid blocks, rank-1 terms,
// and flex stiffness of the island, with vectors in island-local dof coordinates
void mj_effMulAddIsland(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                        int island) {
  int nv = d->island_nv[island];
  int idofadr = d->island_idofadr[island];
  const int* idof2dof = d->map_idof2dof + idofadr;

  if (d->efm_diag) {
    for (int k=0; k < nv; k++) {
      res[k] += d->efm_diag[idof2dof[k]] * vec[k];
    }
  }

  // fluid drag blocks: M-row columns are ancestor dofs, guaranteed in the same island
  if (d->efm_fluid) {
    for (int k=0; k < nv; k++) {
      int i = idof2dof[k];
      int start = m->M_rowadr[i];
      int diag = start + m->M_rownnz[i] - 1;
      mjtNum acc = d->efm_fluid[diag] * vec[k];
      for (int a=start; a < diag; a++) {
        int kc = d->map_dof2idof[m->M_colind[a]] - idofadr;
        mjtNum F = d->efm_fluid[a];
        acc += F * vec[kc];
        res[kc] += F * vec[k];
      }
      res[k] += acc;
    }
  }

  // the island's rank-1 terms: every tree an entry touches shares the island (union-find
  // merges the support), so the first column decides membership
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    mjtNum dot = 0;
    for (int j=0; j < e.nnz; j++) {
      dot += e.val[j] * vec[d->map_dof2idof[e.colind[j]] - idofadr];
    }
    dot *= e.scale;
    for (int j=0; j < e.nnz; j++) {
      res[d->map_dof2idof[e.colind[j]] - idofadr] += dot * e.val[j];
    }
  }

  mjEffIsland isl;
  int has_isl = effIslandStep(d, &isl);
  int gen_csr = efmGeneralCSR(d);

  // flex stiffness CSR restricted to the island's covered triples.
  // Invariant: all dynamic trees of a metric-coupled flex are awake or asleep together (sleeping
  // acts on whole islands in mj_sleep and wakes whole cycles in mj_wakeIsland, and
  // unionConstraintTrees unions all awake trees of a metric-coupled flex into one island), so
  // every column in row i belongs to the same island as i.
  if (d->nefmK && d->nefmdof && has_isl && !gen_csr) {
    const int* rowadr = d->efm_K_rowadr;
    const int* rownnz = d->efm_K_rownnz;
    const int* colind = d->efm_K_colind;
    const mjtNum* val = d->efm_K_val;
    int p_end = isl.cov_adr[island + 1];
    for (int p=isl.cov_adr[island]; p < p_end; p++) {
      int i = d->efm_dofid[isl.cov_ind[p]];
      int li = d->map_dof2idof[i] - idofadr;
      const mjtNum* v0 = val + rowadr[i];
      const mjtNum* v1 = val + rowadr[i+1];
      const mjtNum* v2 = val + rowadr[i+2];
      const int* ci = colind + rowadr[i];
      int nn = rownnz[i] / 3;
      mjtNum r0 = 0, r1 = 0, r2 = 0;
      for (int j=0; j < nn; j++) {
        int lc = d->map_dof2idof[ci[3*j]] - idofadr;
        if (lc < 0 || lc + 2 >= nv) {
          mjERROR("flex stiffness column outside island %d", island);  // SHOULD NOT OCCUR
        }
        mjtNum x0 = vec[lc], x1 = vec[lc+1], x2 = vec[lc+2];
        r0 += v0[3*j]*x0 + v0[3*j+1]*x1 + v0[3*j+2]*x2;
        r1 += v1[3*j]*x0 + v1[3*j+1]*x1 + v1[3*j+2]*x2;
        r2 += v2[3*j]*x0 + v2[3*j+1]*x1 + v2[3*j+2]*x2;
      }
      res[li] += r0;
      res[li+1] += r1;
      res[li+2] += r2;
    }
  } else if (d->nefmK) {
    for (int k=0; k < nv; k++) {
      int i = idof2dof[k];
      int nnz = d->efm_K_rownnz[i];
      if (!nnz) {
        continue;
      }
      const int* ci = d->efm_K_colind + d->efm_K_rowadr[i];
      const mjtNum* v = d->efm_K_val + d->efm_K_rowadr[i];
      mjtNum acc = 0;
      for (int j=0; j < nnz; j++) {
        int lc = d->map_dof2idof[ci[j]] - idofadr;
        if (lc < 0 || lc >= nv) {
          mjERROR("flex stiffness column outside island %d", island);  // SHOULD NOT OCCUR
        }
        acc += v[j] * vec[lc];
      }
      res[k] += acc;
    }
  }

  // matrix-free fallback for flexes in this island not covered by the CSR
  int has_mf = (!gen_csr && has_isl) ? (isl.flex_adr[island] < isl.flex_adr[island + 1]) : 0;
  int interp_asm = d->nefmK && (gen_csr || mjd_flexInterpAssemblable(m));
  if (!gen_csr && !has_isl) {
    for (int f=0; f < m->nflex; f++) {
      if (flexMatrixFree(m, d, f, interp_asm, gen_csr) && flexIsland(m, d, f) == island) {
        has_mf = 1;
        break;
      }
    }
  }
  if (has_mf) {
    mjtNum h = m->opt.timestep;
    mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h*h;
    mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h;
    mj_markStack(d);
    mjtNum* gvec = mjSTACKALLOC(d, m->nv, mjtNum);
    mjtNum* gres = mjSTACKALLOC(d, m->nv, mjtNum);
    mju_zero(gvec, m->nv);
    mju_zero(gres, m->nv);
    for (int k=0; k < nv; k++) {
      gvec[idof2dof[k]] = vec[k];
    }
    int n = has_isl ? (isl.flex_adr[island + 1] - isl.flex_adr[island]) : m->nflex;
    for (int p=0; p < n; p++) {
      int f = has_isl ? isl.flex_ind[isl.flex_adr[island] + p] : p;
      if (!has_isl && (!flexMatrixFree(m, d, f, interp_asm, gen_csr) ||
                       flexIsland(m, d, f) != island)) {
        continue;
      }
      if (m->flex_interp[f]) {
        mjd_flexInterp_mulRange(m, d, gres, gvec, -s1, -s2, d->flexelem_krot, f, f+1);
      } else {
        mjd_flexBend_mulRange(m, d, gres, gvec, s1, s2, f, f+1);
        mjd_flexStretch_mulRange(m, d, gres, gvec, s1, s2, f, f+1);
      }
    }
    for (int k=0; k < nv; k++) {
      res[k] += gres[idof2dof[k]];
    }
    mj_freeStack(d);
  }
}


// apply the step's sparse factor on the covered triples of island (vectors in island-local dof
// coordinates)
static void effCholApplyIsland(mjData* d, const mjEffChol* cs, int island, mjtNum* x,
                               const mjtNum* rhs) {
  int nb = d->nefmdof;
  const mjtNum* Ls = effCholStepValues(d, cs);
  int idofadr = d->island_idofadr[island];
  mj_markStack(d);
  mjtNum* bc = mjSTACKALLOC(d, cs->n, mjtNum);
  mjtNum* xc = mjSTACKALLOC(d, cs->n, mjtNum);
  for (int k=0; k < nb; k++) {
    int i = cs->dofid[k];
    if (d->dof_island[i] == island) {
      mju_copy3(bc + 3*cs->perm[k], rhs + (d->map_dof2idof[i] - idofadr));
    } else {
      mju_zero3(bc + 3*cs->perm[k]);
    }
  }
  effCholSolveBlocked(xc, Ls, bc, cs);
  for (int k=0; k < nb; k++) {
    int i = cs->dofid[k];
    if (d->dof_island[i] == island) {
      mju_copy3(x + (d->map_dof2idof[i] - idofadr), xc + 3*cs->perm[k]);
    }
  }
  mj_freeStack(d);
}


// island-local metric preconditioner x ~= (M + B)_island^-1 b, vectors in island-local dof
// coordinates; L: folded per-dof diagonal (mj_effPrecFold) or NULL;
// qLD/qLDiagInv/M_* are the island's gathered backbone factor
void mj_effPrecIsland(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b, int island,
                      const mjtNum* L, const mjtNum* qLD, const mjtNum* qLDiagInv,
                      const int* M_rownnz, const int* M_rowadr, const int* M_colind) {
  int nv = d->island_nv[island];
  int idofadr = d->island_idofadr[island];
  const int* idof2dof = d->map_idof2dof + idofadr;
  int nbd = m->nefm0dof;
  int flg_bend = nbd && !d->nefmdof;

  mjEffIsland isl;
  int has_isl = effIslandStep(d, &isl);
  int has_cov = has_isl && (isl.cov_adr[island] < isl.cov_adr[island + 1]);
  if (!has_isl && d->nefmdof && (L || effCholReserved(d))) {
    for (int c=0, lo=0; c + 2 < nv; c++) {
      int i = idof2dof[c];
      if (d->efm_K_rownnz[i] > 0 && effFindDof(d->efm_dofid, d->nefmdof, i, &lo) >= 0) {
        has_cov = 1;
        break;
      }
    }
  }
  mjEffChol cs;
  int use_chol = !L && has_cov && effCholReady(m, d, /*is_island=*/1, &cs);
  int use_fold = (L != NULL) && has_cov;
  int has_bend = flg_bend && has_isl && (isl.bend_adr[island] < isl.bend_adr[island + 1]);
  int has_twist = 0;
  if (d->efm_sdiag) {
    for (int f = 0; f < m->nflex; f++) {
      if (!m->flex_rigid[f] && mj_rodHasTwist(m, f) && flexIsland(m, d, f) == island) {
        has_twist = 1;
        break;
      }
    }
  }

  // no folded diagonal, sparse-factor, bending-only, or rod-twist blocks in this island: the
  // gathered backbone factor is all
  if (!use_chol && !use_fold && !has_bend && !has_twist) {
    if (x != b) {
      mju_copy(x, b, nv);
    }
    mj_solveLD(x, qLD, qLDiagInv, nv, 1, M_rownnz, M_rowadr, M_colind, NULL);
    return;
  }

  mj_markStack(d);
  mjtNum* rhs = mjSTACKALLOC(d, nv, mjtNum);
  mju_copy(rhs, b, nv);   // b may alias x

  // backbone solve on uncovered island dofs
  mju_copy(x, rhs, nv);
  if (use_chol || use_fold) {
    if (has_isl) {
      int q_end = isl.cov_adr[island + 1];
      for (int q=isl.cov_adr[island]; q < q_end; q++) {
        int i = d->efm_dofid[isl.cov_ind[q]];
        mju_zero(x + (d->map_dof2idof[i] - idofadr), 3);
      }
    } else {
      for (int c=0, lo=0; c + 2 < nv; ) {
        int i = idof2dof[c];
        if (d->efm_K_rownnz[i] > 0 && effFindDof(d->efm_dofid, d->nefmdof, i, &lo) >= 0) {
          mju_zero(x + c, 3);
          c += 3;
        } else {
          c++;
        }
      }
    }
  }
  if (has_bend) {
    int p_end = isl.bend_adr[island + 1];
    for (int p=isl.bend_adr[island]; p < p_end; p++) {
      int gdof = m->efm0_dofid[isl.bend_ind[p]];
      x[d->map_dof2idof[gdof] - idofadr] = 0;
    }
  }
  mj_solveLD(x, qLD, qLDiagInv, nv, 1, M_rownnz, M_rowadr, M_colind, NULL);
  if (has_twist) {
    effTwistSolve(m, d, x, rhs, island, L);
  }

  // folded per-dof diagonal (CG) or sparse factor on the island's covered triples
  if (use_fold) {
    if (has_isl) {
      int q_end = isl.cov_adr[island + 1];
      for (int q=isl.cov_adr[island]; q < q_end; q++) {
        int c = d->map_dof2idof[d->efm_dofid[isl.cov_ind[q]]] - idofadr;
        x[c]   = rhs[c]   * L[c];
        x[c+1] = rhs[c+1] * L[c+1];
        x[c+2] = rhs[c+2] * L[c+2];
      }
    } else {
      for (int c=0, lo=0; c + 2 < nv; ) {
        int i = idof2dof[c];
        if (d->efm_K_rownnz[i] > 0 && effFindDof(d->efm_dofid, d->nefmdof, i, &lo) >= 0) {
          x[c]   = rhs[c]   * L[c];
          x[c+1] = rhs[c+1] * L[c+1];
          x[c+2] = rhs[c+2] * L[c+2];
          c += 3;
        } else {
          c++;
        }
      }
    }
  } else if (use_chol) {
    effCholApplyIsland(d, &cs, island, x, rhs);
  }

  // bending-only constant factor restricted to the island
  if (has_bend) {
    mjtNum* bfr = mjSTACKALLOC(d, nbd, mjtNum);
    mjtNum* bfz = mjSTACKALLOC(d, nbd, mjtNum);
    mju_zero(bfr, nbd);
    int p_end = isl.bend_adr[island + 1];
    for (int p=isl.bend_adr[island]; p < p_end; p++) {
      int i = isl.bend_ind[p];
      int gdof = m->efm0_dofid[i];
      bfr[i] = rhs[d->map_dof2idof[gdof] - idofadr];
    }
    mju_cholSolveSparse(bfz, m->efm0_L, bfr, nbd,
                        m->efm0_L_rownnz, m->efm0_L_rowadr, m->efm0_L_colind);
    for (int p=isl.bend_adr[island]; p < p_end; p++) {
      int i = isl.bend_ind[p];
      int gdof = m->efm0_dofid[i];
      x[d->map_dof2idof[gdof] - idofadr] = bfz[i];
    }
  }
  mj_freeStack(d);
}


// publish passive flex contact as the metric's rank-1 contact class, one packed row per pair
// ([nnz, conid, colind...] / [scale, force, val...], see engine_derivative.h): matrix-free
// contact costs O(nnz) per pair where the stiffness CSR cost O(nnz^2), and the CSR's sparsity no
// longer grows with the contact set
static void effContactBuild(const mjModel* m, mjData* d, mjtNum scale) {
  d->nefmcon = 0;
  if (!d->ncon || !mjd_flexPassiveContact_any(m)) {
    return;
  }
  int nv = m->nv, issparse = mj_isSparse(m);
  mj_markStack(d);
  mjtNum* jacdif = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jac1 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jac2 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jacn = mjSTACKALLOC(d, 3*nv, mjtNum);
  int* chain = mjSTACKALLOC(d, nv, int);
  int* flex_ndyn = m->nflex ? mjSTACKALLOC(d, m->nflex, int) : NULL;
  for (int f = 0; f < m->nflex; f++) {
    flex_ndyn[f] = m->flex_interp[f] ? mjd_flexInterpDynNodes(m, f) : 0;
  }
  mjtNum* kcon = mjSTACKALLOC(d, d->ncon, mjtNum);

  // pass 1: packed length, two header entries plus nnz per row
  for (int i = 0; i < d->ncon; i++) {
    const mjContact* con = d->contact + i;
    if (con->exclude != 4) {
      kcon[i] = 0;
      continue;
    }
    kcon[i] = mjd_flexContactStiffness(m, d, con, flex_ndyn);
    if (kcon[i] <= 0) {
      continue;
    }
    int NV = mj_contactJacobian(m, d, con, con->dim, jacdif, NULL, jac1, jac2, NULL, NULL, chain);
    if (NV) {
      d->nefmcon += 2 + NV;   // an upper bound in a dense model, trimmed to `adr` below
    } else {
      d->contact[i].exclude = 3;   // affects no dofs, as the passive force used to mark it
    }
  }
  if (!d->nefmcon) {
    mj_freeStack(d);
    return;
  }
  d->efm_con_ind = EFMALLOC(int, d->nefmcon);
  d->efm_con_val = EFMALLOC(mjtNum, d->nefmcon);

  // pass 2: fill, rows packed back to back in both arrays
  int adr = 0;
  for (int i = 0; i < d->ncon; i++) {
    const mjContact* con = d->contact + i;
    mjtNum k = kcon[i];
    if (con->exclude != 4 || k <= 0) {
      continue;
    }
    int NV = mj_contactJacobian(m, d, con, con->dim, jacdif, NULL, jac1, jac2, NULL, NULL, chain);
    if (!NV) {
      continue;
    }
    mju_mulMatMat(jacn, con->frame, jacdif, con->dim > 1 ? 3 : 1, 3, NV);
    mjtNum s = mjd_flexContactSlack(k, con->dist, /*lam=*/0);
    int nnz = 0;
    for (int a = 0; a < NV; a++) {
      // a dense model gets no chain: every dof is returned, so compact the row on its nonzeros
      if (!issparse && jacn[a] == 0) {
        continue;
      }
      d->efm_con_ind[adr + 2 + nnz] = issparse ? chain[a] : a;
      d->efm_con_val[adr + 2 + nnz] = jacn[a];
      nnz++;
    }
    d->efm_con_ind[adr] = nnz;
    d->efm_con_ind[adr + 1] = i;
    d->efm_con_val[adr] = scale * k;
    d->efm_con_val[adr + 1] = -k*mjd_flexContactResidual(k, con->dist, s, /*lam=*/0);
    adr += 2 + nnz;
  }
  d->nefmcon = adr;   // the dense compaction can land below the bound counted above
  mj_freeStack(d);
}


// apply the published rows' forces, res += force * row: the passive stage takes its contact
// force from the rows the metric build published
void mj_effContactForce(const mjData* d, mjtNum* res) {
  const int* ind = d->efm_con_ind;
  const mjtNum* val = d->efm_con_val;
  for (int adr = 0; adr < d->nefmcon; ) {
    int nnz = ind[adr];
    mjtNum f = val[adr + 1];
    const int* colind = ind + adr + 2;
    const mjtNum* row = val + adr + 2;
    for (int a = 0; a < nnz; a++) {
      // fused multiply-add: rounding the product first moves the results by an ulp
      res[colind[a]] += f * row[a];
    }
    adr += 2 + nnz;
  }
}


// build the per-step implicit effective metric on the arena, or deactivate it. The gate
// decision (integrator=discrete) is the caller's: the metric module has no dependency on
// the solver configuration beyond what it is told here.
void mj_effBuild(const mjModel* m, mjData* d, int active, int flg_factor) {
  int nv = m->nv;
  d->efm_active = 0;
  d->nefmK = 0;
  d->nefmcon = 0;
  d->nefmT = 0;
  d->nefmdof = 0;
  d->nefmL = 0;
  d->nefmA = 0;
  d->efm_diag = NULL;
  d->efm_ck = NULL;
  d->efm_sdiag = NULL;
  d->efm_fluid = NULL;
  d->efm_tid = NULL;
  d->efm_ts = NULL;
  d->efm_tk = NULL;
  d->efm_aid = NULL;
  d->efm_as = NULL;
  d->efm_ak = NULL;
  d->efm_ca = NULL;
  d->efm_K_colind = NULL;
  d->efm_K_val = NULL;
  d->efm_dofid = NULL;
  d->efm_con_ind = NULL;
  d->efm_con_val = NULL;
  d->efm_L = NULL;
  if (!active) {
    return;
  }
  mjtNum h = m->opt.timestep;

  // corotated element stiffness cache (used by the shift and the matrix-free fallback)
  mju_zero(d->flexelem_krot, m->nflexstiffness);
  mjd_flexInterp_cacheKrot(m, d, d->flexelem_krot);

  // smooth-force shift c = h*K*qvel (values refreshed by mj_effShift in the velocity stage)
  d->efm_c = EFMALLOC(mjtNum, nv);

  // per-dof diagonal classes (joint damping/stiffness, joint-transmission actuator damping):
  // position-dependent stiffness assembled here; the velocity-dependent damping part and the
  // qH backbone factor are refreshed by mj_effShift in the velocity stage
  d->efm_ck = EFMALLOC(mjtNum, nv);
  int any_diag = effDiagStiff(m, d);
  for (int i=0; !any_diag && i < nv; i++) {
    any_diag = dofDampPossible(m, i);
  }

  // tendons with metric terms: id list here, values (velocity-dependent) in mj_effShift.
  // Sleeping tendons are excluded, matching the passive-force filter; the wake rule
  // (mj_wakeTendon) keeps metric-coupled tendon pairs awake or asleep together. Under the
  // dual solver the class is excluded (mj_effCouplings): an empty list removes its metric
  // terms and its shift together, so the tendon forces integrate explicitly
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
  if (m->ntendon && mj_effCouplings(m)) {
    d->efm_tid = EFMALLOC(int, m->ntendon);
    d->efm_ts  = EFMALLOC(mjtNum, m->ntendon);
    d->efm_tk  = EFMALLOC(mjtNum, m->ntendon);
    for (int i=0; i < m->ntendon; i++) {
      if (!mj_effTendonPossible(m, i)) {
        continue;
      }
      if (sleep_filter && mj_sleepState(m, d, mjOBJ_TENDON, i) != mjS_AWAKE) {
        continue;
      }
      d->efm_ts[d->nefmT] = 0;
      d->efm_tid[d->nefmT++] = i;
    }
  }

  // metric-possible actuators: allocation here, values (ctrl- and state-dependent) in
  // mj_effActuation at the actuation stage. Under the dual solver the class is excluded
  // (mj_effCouplings, as for tendons): no allocation, so mj_effActuation adds nothing
  int any_act = 0;
  if (mj_effCouplings(m)) {
    for (int i=0; i < m->nactuator; i++) {
      if (mj_effActuatorPossible(m, i)) {
        any_act = 1;
        break;
      }
    }
  }
  if (any_act) {
    d->efm_aid = EFMALLOC(int, m->nactuator);
    d->efm_as  = EFMALLOC(mjtNum, m->nactuator);
    d->efm_ak  = EFMALLOC(mjtNum, m->nactuator);
    d->efm_ca  = EFMALLOC(mjtNum, nv);
    mju_zero(d->efm_ca, nv);
  }

  // the tendon and actuator diagonals join the qH backbone: force the diagonal machinery on.
  // Fluid drag and passive flex contact enter only while their forces are applied: mj_passive
  // skips them, like every passive force, when both the spring and damper forces are disabled
  int passive = !(mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER));
  int any_fluid = passive && (m->opt.viscosity > 0 || m->opt.density > 0);
  int any_twist = 0;
  if (passive && mj_effCouplings(m)) {
    for (int f=0; f < m->nflex; f++) {
      if (!m->flex_rigid[f] && mj_rodHasTwist(m, f)) {
        any_twist = 1;
        break;
      }
    }
  }
  d->efm_diag = (any_diag || d->nefmT || any_act || any_fluid || any_twist) ? EFMALLOC(mjtNum, nv) : NULL;
  d->efm_sdiag = d->efm_diag ? EFMALLOC(mjtNum, nv) : NULL;
  d->efm_fluid = any_fluid ? EFMALLOC(mjtNum, m->nC) : NULL;
  if (d->efm_diag) {
    mju_zero(d->efm_diag, nv);
  }
  if (d->efm_fluid) {
    mju_zero(d->efm_fluid, m->nC);
  }

  // assemble the standard-flex part of B into CSR (constant during the step). With stretch or
  // assemblable interp present, assemble the FULL matrix (bending included): one CSR then
  // serves both the matvec and the per-step factor. Bending-only models keep the stencil
  // operator + the constant mj_setConst factor. The stiffness (s1) and damping (s2) parts enter
  // only when their forces are enabled; the structure does not depend on the flags
  mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h*h;
  mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h;

  // Newton needs every flex in its explicit Hessian: once any active flex is not simple, the
  // whole stiffness goes through the general (Jacobian pull-back) assembly, including the simple
  // flexes of the same model. Recorded now: the factor and island builds below read it.
  int gen_csr = (m->opt.solver == mjSOL_NEWTON) && !efmAllSimple(m);
  d->efm_active = gen_csr ? EFM_ACTIVE_GENERAL : EFM_ACTIVE;
  const mjtNum* krot = (gen_csr || mjd_flexInterpAssemblable(m)) ? d->flexelem_krot : NULL;
  d->efm_K_rownnz = EFMALLOC(int, nv);
  d->efm_K_rowadr = EFMALLOC(int, nv);

  // Newton consumes the metric as an explicit sparse matrix inside its Hessian: force full
  // CSR assembly, including bending-only flexes and general articulated flex attachments
  int assemble_any = mjd_flexStiff_any(m, krot != NULL) || mjd_flexPassiveContact_any(m);
  if (m->opt.solver == mjSOL_NEWTON) {
    for (int f=0; !assemble_any && f < m->nflex; f++) {
      assemble_any = mjd_flexStiff_activeGen(m, f, /*flg_bend=*/1, /*flg_stretch=*/1, gen_csr) ||
                     (gen_csr && mjd_flexInterp_processed(m, f));
    }
  }
  if (assemble_any && gen_csr) {
    // the general pattern is built once, with the CSR arrays allocated as soon as it is known
    d->nefmK = mjd_flexStiff_assembleGenArena(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                                              &d->efm_K_colind, &d->efm_K_val, s1, s2,
                                              /*bend*/ 1, /*stretch*/ 1, krot);
  } else if (assemble_any) {
    d->nefmK = mjd_flexStiff_assemble(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                                      NULL, NULL, s1, s2, /*bend*/ 1, /*stretch*/ 1, krot);
  }

  // passive flex contact is a rank-1 class, not CSR entries (see effContactBuild)
  if (passive) {
    effContactBuild(m, d, h*h);
  }
  if (d->nefmK) {
    if (!gen_csr) {
      d->efm_K_colind = EFMALLOC(int, d->nefmK);
      d->efm_K_val    = EFMALLOC(mjtNum, d->nefmK);
      mjd_flexStiff_assemble(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                             d->efm_K_colind, d->efm_K_val, s1, s2,
                             /*bend*/ 1, /*stretch*/ 1, krot);
    }
    // per-step factor of the flex block of (M + K): the stiffness is constant during the
    // step, so one factorization here turns every preconditioner application into a direct
    // solve (the stiff flex block stops being iterated on). Consumers that only multiply
    // (inverse dynamics) skip it.
    if (flg_factor) {
      effBlocks(m, d);
    }

  } else {
    mju_zeroInt(d->efm_K_rownnz, nv);
    mju_zeroInt(d->efm_K_rowadr, nv);
  }

  // per-island flex/triple lists for mj_effMulAddIsland and mj_effPrecIsland
  if (d->nisland > 0 && d->nefc > 0 && m->nflex > 0 &&
      !mjENABLED(mjENBL_IPC) && !effCholReserved(d)) {
    effIslandBuild(m, d);
  }
}
