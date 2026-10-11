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
      it->cls = 3;  // not a producer class: the cursor's done state
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
  return 0;
}


// res += B*vec, the stiffness part of the metric. The per-dof diagonal classes are applied
// from efm_diag (scales pre-folded); the stretch (and, when assemblable, bending and interp)
// part from the per-step CSR; terms not in the CSR fall back to the matrix-free operators.
void mj_effMulAdd(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec, int flg_contact) {
  mjtNum h = m->opt.timestep;
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

  // rank-1 terms (tendon, actuator): res += scale * row' * (row * vec)
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

  if (d->nefmK && d->nefmdof) {
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

  // A standard flex is assembled only when all its attachments have fixed-frame XYZ
  // mappings. Dispatch general flexes individually: another flex's CSR must not suppress
  // their operators. Both the stiffness and damping parts follow the passive-force flags.
  mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h*h;
  mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h;
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_interp[f] || m->flex_rigid[f] || m->flex_dim[f] < 2) continue;
    if (!d->nefmK || !mj_flexSimple(m, f)) {
      mjd_flexBend_mulRange(m, d, res, vec, s1, s2, f, f+1);
      mjd_flexStretch_mulRange(m, d, res, vec, s1, s2, f, f+1);
    }
  }
  if (!d->nefmK || !mjd_flexInterpAssemblable(m)) {
    mjd_flexInterp_mul(m, d, res, vec, -s1, -s2, d->flexelem_krot);
  }
}


// the unfactored 3x3 diagonal block of M + K for the covered dof triple starting at i
static void effBlockRaw(const mjModel* m, const mjData* d, int i, mjtNum* Bk) {
  mju_zero(Bk, 9);
  for (int r = 0; r < 3; r++) {
    int row = i + r;
    for (int a = m->M_rowadr[row]; a < m->M_rowadr[row] + m->M_rownnz[row]; a++) {
      int c = m->M_colind[a];
      if (c >= i && c < i+3) Bk[3*r + (c-i)] += d->M[a];
    }
    for (int a = d->efm_K_rowadr[row]; a < d->efm_K_rowadr[row] + d->efm_K_rownnz[row]; a++) {
      int c = d->efm_K_colind[a];
      if (c >= i && c < i+3) Bk[3*r + (c-i)] += d->efm_K_val[a];
    }
  }
}


//------------------ sparse Cholesky metric preconditioner ----------------------------------------
// Under mjENBL_IPC, when every covered flex is 2D (effChol2D), the covered block of M + K is
// factored by a reverse-order sparse Cholesky, blocked by vertex, rather than by its 3x3 diagonal
// blocks only.
// The ordering is the model's: mj_setConst orders the vertices of the model's efm_K pattern by
// minimum degree (mj_effCholSetConst, into efmC_perm). The symbolic factorization depends only on
// the efm_K pattern: each step analyses the pattern it assembled under that ordering
// (effCholReserve), on the vertex graph (mju_cholFactorSymbolicBlocked), and packs the analysis
// into the metric's buffer efm_L, next to the factor it describes:
//   [9*nb 3x3 blocks][status][the analysis (effPackLayout)][nL factor values]
// so the blocks remain the fallback whenever the factor is unusable. The numeric factorization
// runs lazily for the metric's copy (effCholEnsure) and per constraint solve for the solver's
// folded copy (mj_effCholFoldFactor): each solve analyses the pattern extended by the vertex
// pairs the contact terms couple, with its own minimum-degree ordering, and factors it on the
// stack of d in the solve's frame (mjEffFactor, mjEffFold.factor). When the arena cannot hold
// that factor, the copy uses the step's analysis in mjEffFold.F, with the pairs off its pattern
// complemented on the diagonal.

#define EFF_HDR 1
#define EFF_CHOL_PENDING 2
#define EFF_CHOL_MAXVERT 8192
#define EFF_CHOL_PIVOT 1e-12

typedef struct {
  int nefc;
  const mjtNum* D;
  int is_sparse;
  const mjtNum* J;
  const int *rownnz, *rowadr, *colind;
} mjEffConRows;

typedef struct {
  int nb, n, nL, ne;
  const int* dofid;
  int *perm, *L_rownnz, *L_rowadr, *L_colind, *LT_rownnz, *LT_rowadr, *LT_colind, *LT_map;
  int *eadr, *enbr;
} mjEffChol;

struct mjEffFactor_ {
  mjEffChol c;
  mjtNum* F;
};

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

static int effPopcount(uint64_t x) {
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_popcountll(x);
#else
  x = x - ((x >> 1) & 0x5555555555555555ULL);
  x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
  x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0FULL;
  return (int)((x * 0x0101010101010101ULL) >> 56);
#endif
}

static int effCtz64(uint64_t x) {
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_ctzll(x);
#else
  int b = 0;
  while (!((x >> b) & 1ULL)) b++;
  return b;
#endif
}

// the step's analysis, packed into efm_L before the factor values: nL, ne, then perm (nb),
// L_rownnz, L_rowadr (n), L_colind (nL), LT_rownnz, LT_rowadr (n), LT_colind, LT_map (nL),
// eadr (nb+1), enbr (ne). Holds no pointer, so mj_copyData carries it with efm_L
static void effPackLayout(int* A, int nb, const int* dofid, mjEffChol* c) {
  int n = 3*nb;
  c->nb = nb;
  c->n = n;
  c->nL = A[0];
  c->ne = A[1];
  c->dofid = dofid;
  int* p = A + 2;
  c->perm = p;       p += nb;
  c->L_rownnz = p;   p += n;
  c->L_rowadr = p;   p += n;
  c->L_colind = p;   p += c->nL;
  c->LT_rownnz = p;  p += n;
  c->LT_rowadr = p;  p += n;
  c->LT_colind = p;  p += c->nL;
  c->LT_map = p;     p += c->nL;
  c->eadr = p;       p += nb + 1;
  c->enbr = p;
}

static size_t effPackNums(int nb, int nL, int ne) {
  size_t ints = 3 + 14*(size_t)nb + 3*(size_t)nL + (size_t)ne;
  return (sizeof(int)*ints + sizeof(mjtNum) - 1) / sizeof(mjtNum);
}

static void effCholStep(const mjData* d, mjEffChol* c) {
  int nb = d->nefmdof;
  effPackLayout((int*)(d->efm_L + 9*nb + EFF_HDR), nb, d->efm_dofid, c);
}

static int effCholReserved(const mjData* d) {
  int nb = d->nefmdof;
  return nb && d->nefmL > 9*nb;
}

static mjtNum* effCholStepValues(const mjData* d, const mjEffChol* c) {
  return d->efm_L + 9*c->nb + EFF_HDR + effPackNums(c->nb, c->nL, c->ne);
}

// the covered blocks of a K pattern with row sizes rownnz (nv rows): each nonzero row starts a
// vertex triple, its 3 consecutive dofs
static int effCoveredBlocks(int nv, const int* rownnz, int* dofid) {
  int nb = 0;
  for (int i=0; i < nv; ) {
    if (rownnz[i]) {
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

// greedy minimum-degree elimination on the vertex graph (symmetric CSR adr/cnt/nbr), with dense
// bitset adjacency; the first eliminated vertex goes last (reverse-order Cholesky)
static void effCholMinDegree(mjData* d, int nb, const int* adr, const int* cnt, const int* nbr,
                             int* perm) {
  int words = (nb + 63) / 64;
  uint64_t* adj = effBits(d, (size_t)nb*words);
  uint64_t* alive = effBits(d, words);
  int* deg = effInts(d, nb);
  memset(adj, 0, sizeof(uint64_t)*(size_t)nb*words);
  memset(alive, 0, sizeof(uint64_t)*words);
  for (int k=0; k < nb; k++) {
    alive[k/64] |= 1ULL << (k%64);
    for (int j=0; j < cnt[k]; j++) {
      int q = nbr[adr[k]+j];
      adj[(size_t)k*words+q/64] |= 1ULL << (q%64);
      adj[(size_t)q*words+k/64] |= 1ULL << (k%64);
    }
  }
  for (int k=0; k < nb; k++) {
    adj[(size_t)k*words+k/64] &= ~(1ULL << (k%64));
    int dg = 0;
    for (int w=0; w < words; w++) dg += effPopcount(adj[(size_t)k*words+w]);
    deg[k] = dg;
  }
  for (int step=0; step < nb; step++) {
    int v = -1;
    for (int w=0; w < words; w++) {
      uint64_t aw = alive[w];
      while (aw) {
        int b = effCtz64(aw);
        aw &= aw - 1;
        int k = 64*w + b;
        if (v < 0 || deg[k] < deg[v]) v = k;
      }
    }
    perm[v] = nb - 1 - step;
    alive[v/64] &= ~(1ULL << (v%64));
    uint64_t* av = adj + (size_t)v*words;
    for (int w=0; w < words; w++) {
      uint64_t bits = av[w] & alive[w];
      while (bits) {
        int b = effCtz64(bits);
        bits &= bits - 1;
        int u = 64*w + b;
        uint64_t* au = adj + (size_t)u*words;
        int dg = 0;
        for (int x=0; x < words; x++) {
          au[x] |= av[x];
          if (x == u/64) au[x] &= ~(1ULL << (u%64));
          dg += effPopcount(au[x] & alive[x]);
        }
        deg[u] = dg;
      }
    }
  }
}

static int effCholFind(const int* L_rownnz, const int* L_rowadr, const int* L_colind, int r,
                       int col) {
  int adr = L_rowadr[r], nnz = L_rownnz[r];
  if (col == r) {
    return adr + nnz - 1;
  }
  int lo = 0, hi = nnz - 2;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    int cm = L_colind[adr+mid];
    if (cm == col) return adr + mid;
    if (cm < col) lo = mid + 1;
    else hi = mid - 1;
  }
  return -1;
}

static void effAllow(int* amark, const int* eadr, const int* enbr, int k) {
  for (int j=eadr[k]; j < eadr[k+1]; j++) {
    amark[enbr[j]] = k;
  }
}

static int effAllowed(const int* amark, int ne, int k, int q) {
  return !ne || amark[q] == k;
}

static int effPairCompare(const void* a, const void* b) {
  const int* x = (const int*)a;
  const int* y = (const int*)b;
  if (x[0] != y[0]) return x[0] < y[0] ? -1 : 1;
  if (x[1] != y[1]) return x[1] < y[1] ? -1 : 1;
  return 0;
}

// build the K vertex graph (kadr/knbr) and the allowed edge CSR (eadr/enbr: flex edges between
// covered vertices plus the nx extra pairs xpair, both ways, sorted and deduplicated)
static int effCholGraph(mjData* d, const mjModel* m, int nb, const int* dofid, const int* blk,
                        const int* K_rownnz, const int* K_rowadr, const int* K_colind,
                        const int* xpair, int nx, int** kadr, int** knbr, int** eadr,
                        int** enbr) {
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
  int* apair = effInts(d, 2*((size_t)m->nflexedge + nx));
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
  }
  int nfe = 2*np;
  for (int x=0; x < nx; x++) {
    apair[2*np] = xpair[2*x];
    apair[2*np+1] = xpair[2*x+1];
    np++;
  }
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
  return nfe;
}

// single-pass symbolic analysis on the stack: builds the vertex graph, runs minimum-degree
// ordering if order != 0, and computes the blocked symbolic Cholesky counts
typedef struct {
  int nb, nL, ne, nfe;
  int *eadr, *enbr, *rownnz, *rowadr, *colind, *scratch;
  int *L_rownnz, *L_rowadr, *LT_rownnz, *LT_rowadr;
} mjEffCholPass;

static int effCholBegin(const mjModel* m, mjData* d, int nb, const int* dofid, const int* blk,
                        const int* K_rownnz, const int* K_rowadr, const int* K_colind,
                        const int* xpair, int nx, int order, int* perm, mjEffCholPass* p) {
  if (nb <= 0 || nb > EFF_CHOL_MAXVERT) {
    return 0;
  }
  int n = 3*nb, *kadr, *knbr;
  p->nb = nb;
  p->nfe = effCholGraph(d, m, nb, dofid, blk, K_rownnz, K_rowadr, K_colind, xpair, nx,
                        &kadr, &knbr, &p->eadr, &p->enbr);
  p->ne = p->eadr[nb];
  int* amark = effInts(d, nb);
  int* mark = effInts(d, nb);
  int* adr = effInts(d, nb);
  int* cnt = effInts(d, nb);
  int* nbr = effInts(d, (size_t)kadr[nb] + p->ne);
  for (int k=0; k < nb; k++) amark[k] = mark[k] = -1;
  int tot = 0;
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
    cnt[k] = tot - adr[k];
  }
  if (order) {
    effCholMinDegree(d, nb, adr, cnt, nbr, perm);
  }
  p->rownnz = effInts(d, nb);
  p->rowadr = effInts(d, nb);
  p->colind = effInts(d, (size_t)kadr[nb] + p->ne);
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
  p->LT_rownnz = effInts(d, n);
  p->LT_rowadr = effInts(d, n);
  p->nL = mju_cholFactorSymbolicBlocked(NULL, p->L_rownnz, p->L_rowadr, NULL,
                                        p->LT_rownnz, p->LT_rowadr, NULL,
                                        p->rownnz, p->rowadr, p->colind, nb, 3, p->scratch);
  return 1;
}

static void effCholFinish(const mjEffCholPass* p, mjEffChol* c) {
  int nb = p->nb, n = 3*nb;
  memcpy(c->L_rownnz, p->L_rownnz, sizeof(int)*n);
  memcpy(c->L_rowadr, p->L_rowadr, sizeof(int)*n);
  memcpy(c->LT_rownnz, p->LT_rownnz, sizeof(int)*n);
  memcpy(c->LT_rowadr, p->LT_rowadr, sizeof(int)*n);
  memcpy(c->eadr, p->eadr, sizeof(int)*(nb + 1));
  if (p->ne) {
    memcpy(c->enbr, p->enbr, sizeof(int)*p->ne);
  }
  mju_cholFactorSymbolicBlocked(c->L_colind, c->L_rownnz, c->L_rowadr, c->LT_colind, c->LT_rownnz,
                                c->LT_rowadr, c->LT_map, p->rownnz, p->rowadr, p->colind,
                                nb, 3, p->scratch);
}

static int effChol2D(const mjModel* m) {
  for (int f=0; f < m->nflex; f++) {
    int covered = mjd_flexStiff_active(m, f, /*flg_bend=*/1, /*flg_stretch=*/1) ||
                  mjd_flexInterp_processed(m, f) || mj_effFlexContactPossible(m, f);
    if (covered && m->flex_dim[f] != 2) {
      return 0;
    }
  }
  return 1;
}

static int effCholApplies(const mjModel* m, int nb, const int* dofid) {
  if (!nb || nb > EFF_CHOL_MAXVERT || !m->nefmCvert || m->efmC_perm[0] < 0 ||
      !mjENABLED(mjENBL_IPC) || !effChol2D(m)) {
    return 0;
  }
  if (mjd_flexInterpAssemblable(m)) {
    for (int f=0; f < m->nflex; f++) {
      if (mjd_flexInterp_processed(m, f)) {
        return 0;
      }
    }
  }
  return dofid[nb-1] + 3 <= m->nv;
}

static size_t effCholKcap(const mjData* d, int nb, const int* dofid) {
  size_t cap = 0;
  for (int k=0; k < nb; k++) cap += d->efm_K_rownnz[dofid[k]];
  return cap;
}

static size_t effCholAnalysisBytes(const mjModel* m, size_t nb, size_t kcap, size_t nx) {
  size_t ne = (size_t)m->nflexedge + nx;
  size_t nnbr = kcap + 2*ne;
  size_t words = (nb + 63) / 64;
  return mj_stackFrameBytes() + 2*effIntBytes(nb + 1) + effIntBytes(kcap) + 2*effIntBytes(2*ne) +
         effIntBytes(nb) + 4*effIntBytes(nb) + effIntBytes(nnbr) + effBitBytes(nb*words) +
         effBitBytes(words) + effIntBytes(nb) + 2*effIntBytes(nb) + effIntBytes(nnbr) +
         effIntBytes(3*nb) + 4*effIntBytes(3*nb);
}

static size_t effFillBytes(size_t nb, size_t nv) {
  return mj_stackFrameBytes() + effNumBytes(9*nb) + effIntBytes(nv) + 4*effIntBytes(nb);
}

static size_t effCholFillBytes(const mjEffChol* c, int nv) {
  return effNumBytes(c->nL) + effFillBytes(c->nb, nv);
}

static size_t effConScratchBytes(size_t nb) {
  return 2*effIntBytes(nb) + effNumBytes(3*nb);
}

static size_t effCholReserveBytes(int nb, int nv, int nL, size_t npack) {
  return effNumBytes(EFF_HDR + npack + (size_t)nL) + effNumBytes(EFF_HDR + (size_t)nL) +
         effConScratchBytes(nb) + effNumBytes(nL) + effFillBytes(nb, nv);
}

// compute the model's minimum-degree vertex ordering for 2D flex metric stencils
void mj_effCholSetConst(mjModel* m, mjData* d) {
  int ncap = m->nefmCvert, nv = m->nv;
  if (!ncap) {
    return;
  }
  for (int t=0; t < ncap; t++) m->efmC_perm[t] = -1;
  if (!effChol2D(m)) {
    return;
  }
  mj_markStack(d);
  size_t avail = mj_stackBytesAvailable(d);
  size_t need = mj_stackFrameBytes() + 3*effIntBytes(nv) + effIntBytes(ncap + 1);
  int nb = 0, nK = 0, *K_rownnz = NULL, *K_rowadr = NULL, *blk = NULL, *dofid = NULL;
  size_t kcap = 0;
  if (avail >= need) {
    K_rownnz = effInts(d, nv);
    K_rowadr = effInts(d, nv);
    blk = effInts(d, nv);
    dofid = effInts(d, ncap + 1);
    nK = mjd_flexStiff_assemble(m, d, K_rownnz, K_rowadr, NULL, NULL, 0, 0, 1, 1, NULL);
    nb = effCoveredBlocks(nv, K_rownnz, dofid);
    if (!nb || nb > ncap || nb > EFF_CHOL_MAXVERT || dofid[nb-1] + 3 > nv) {
      mj_freeStack(d);
      return;
    }
    for (int k=0; k < nb; k++) kcap += K_rownnz[dofid[k]];
    need += effIntBytes(nK) + effIntBytes(nb) + effCholAnalysisBytes(m, nb, kcap, 0);
  }
  if (avail < need) {
    mj_freeStack(d);
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return;
  }
  int* K_colind = effInts(d, nK);
  int* perm = effInts(d, nb);
  mjd_flexStiff_assemble(m, d, K_rownnz, K_rowadr, K_colind, NULL, 0, 0, 1, 1, NULL);
  effCovered(nv, nb, dofid, blk);
  mjEffCholPass pass;
  if (effCholBegin(m, d, nb, dofid, blk, K_rownnz, K_rowadr, K_colind, NULL, 0, 1, perm, &pass)) {
    for (int k=0; k < nb; k++) m->efmC_perm[perm[k]] = dofid[k];
  }
  mj_freeStack(d);
}

// numeric factorization by 3x3 vertex blocks: H = L'L backward elimination
static int effCholNumericBlocked(const mjEffChol* c, mjtNum* L, const mjtNum* H, mjtNum mindiag,
                                 mjtNum* dense) {
  int nb = c->nb, rank = c->n;
  const int *rowadr = c->L_rowadr, *rownnz = c->L_rownnz, *colind = c->L_colind;
  for (int p=nb-1; p >= 0; p--) {
    int r0 = 3*p, nob = rownnz[r0] - 1;
    const int* ci = colind + rowadr[r0];
    for (int a=0; a < 3; a++) {
      const mjtNum* Ha = H + rowadr[r0+a];
      for (int i=0; i < nob; i++) dense[3*ci[i]+a] = Ha[i];
      for (int b=0; b <= a; b++) dense[3*(r0+b)+a] = Ha[nob+b];
    }
    int ltadr = c->LT_rowadr[r0], ltnnz = c->LT_rownnz[r0];
    for (int k=0; k < ltnnz; k++) {
      int qrow = c->LT_colind[ltadr+k];
      if (qrow < r0 + 3 || qrow % 3) continue;
      int off = c->LT_map[ltadr+k] - rowadr[qrow];
      const mjtNum *Q0 = L + rowadr[qrow], *Q1 = L + rowadr[qrow+1], *Q2 = L + rowadr[qrow+2];
      const int* qci = colind + rowadr[qrow];
      mjtNum l00 = Q0[off], l01 = Q0[off+1], l02 = Q0[off+2];
      mjtNum l10 = Q1[off], l11 = Q1[off+1], l12 = Q1[off+2];
      mjtNum l20 = Q2[off], l21 = Q2[off+1], l22 = Q2[off+2];
      for (int i=0; i <= off; i++) {
        mjtNum y0 = Q0[i], y1 = Q1[i], y2 = Q2[i];
        mjtNum* dd = dense + 3*qci[i];
        dd[0] -= l00*y0 + l10*y1 + l20*y2;
        dd[1] -= l01*y0 + l11*y1 + l21*y2;
        dd[2] -= l02*y0 + l12*y1 + l22*y2;
      }
      mjtNum y0 = Q0[off+1], y1 = Q1[off+1], y2 = Q2[off+1];
      mjtNum* dd = dense + 3*(r0 + 1);
      dd[1] -= l01*y0 + l11*y1 + l21*y2;
      dd[2] -= l02*y0 + l12*y1 + l22*y2;
      y0 = Q0[off+2];
      y1 = Q1[off+2];
      y2 = Q2[off+2];
      dense[3*(r0+2)+2] -= l02*y0 + l12*y1 + l22*y2;
    }
    for (int a=2; a >= 0; a--) {
      int r = r0 + a;
      mjtNum* La = L + rowadr[r];
      mjtNum diag = dense[3*r+a];
      int deficient = diag < mindiag;
      if (deficient) {
        diag = mindiag;
        rank--;
      }
      mjtNum Lrr = mju_sqrt(diag), inv = 1.0 / Lrr;
      if (deficient) {
        mju_zero(La, nob + a);
      } else {
        for (int i=0; i < nob; i++) La[i] = dense[3*ci[i]+a] * inv;
        for (int b=0; b < a; b++) La[nob+b] = dense[3*(r0+b)+a] * inv;
      }
      La[nob+a] = Lrr;
      for (int e=0; e < a; e++) {
        mjtNum le = La[nob+e];
        if (!le) continue;
        for (int i=0; i < nob; i++) dense[3*ci[i]+e] -= le * La[i];
        for (int b=0; b <= e; b++) dense[3*(r0+b)+e] -= le * La[nob+b];
      }
    }
    for (int i=0; i < nob; i++) {
      mjtNum* dd = dense + 3*ci[i];
      dd[0] = dd[1] = dd[2] = 0;
    }
    for (int b=0; b < 9; b++) dense[3*r0+b] = 0;
  }
  return rank;
}

// assemble and factor the covered block of M + K (plus H if pre-initialized with contact terms);
// each dropped off-pattern K block B_kq is replaced by ||B_kq||_F * I on both diagonal blocks
static void effCholFill(const mjModel* m, mjData* d, const mjEffChol* c, mjtNum* status,
                        mjtNum* values, mjtNum* H) {
  int nb = c->nb, n = c->n, nL = c->nL, ne = c->ne, nv = m->nv;
  *status = 0;
  mj_markStack(d);
  if (!H) {
    H = mjSTACKALLOC(d, nL, mjtNum);
    mju_zero(H, nL);
  }
  mjtNum* scratch = mjSTACKALLOC(d, 3*n, mjtNum);
  mju_zero(scratch, 3*n);
  int* blk = effInts(d, nv);
  int* sc = effInts(d, 4*(size_t)nb);
  int *amark = sc, *slot = sc + nb, *qmark = sc + 2*nb, *qlist = sc + 3*nb;
  effCovered(nv, nb, c->dofid, blk);
  for (int k=0; k < nb; k++) amark[k] = slot[k] = qmark[k] = -1;

  for (int k=0; k < nb; k++) {
    int p = c->perm[k], i0 = c->dofid[k];
    const int* vc = c->L_colind + c->L_rowadr[3*p];
    int nvc = (c->L_rownnz[3*p] - 1) / 3, nq = 0;
    for (int i=0; i < nvc; i++) slot[vc[3*i]/3] = i;
    effAllow(amark, c->eadr, c->enbr, k);
    for (int a=0; a < 3; a++) {
      int i = i0 + a, ra = c->L_rowadr[3*p+a], da = ra + 3*nvc;
      for (int j=d->efm_K_rowadr[i]; j < d->efm_K_rowadr[i] + d->efm_K_rownnz[i]; j++) {
        int col = d->efm_K_colind[j], q = blk[col];
        if (q < 0) continue;
        mjtNum v = d->efm_K_val[j];
        int b = col - c->dofid[q];
        if (q == k) {
          if (b <= a) H[da+b] += v;
        } else if (effAllowed(amark, ne, k, q)) {
          int pq = c->perm[q];
          if (pq < p) H[ra + 3*slot[pq] + b] += v;
        } else if (q > k) {
          if (qmark[q] != k) {
            qmark[q] = k;
            qlist[nq] = q;
            scratch[nq++] = 0;
          }
          for (int idx=nq-1; idx >= 0; idx--) {
            if (qlist[idx] == q) {
              scratch[idx] += v * v;
              break;
            }
          }
        }
      }
      for (int j=m->M_rowadr[i]; j < m->M_rowadr[i] + m->M_rownnz[i]; j++) {
        int col = m->M_colind[j];
        if (col >= i0) H[da+col-i0] += d->M[j];
      }
    }
    for (int i=0; i < nvc; i++) slot[vc[3*i]/3] = -1;
    for (int idx=0; idx < nq; idx++) {
      mjtNum f = mju_sqrt(scratch[idx]);
      scratch[idx] = 0;
      int pk = 3*p, pq = 3*c->perm[qlist[idx]];
      for (int a=0; a < 3; a++) {
        H[c->L_rowadr[pk+a] + c->L_rownnz[pk+a] - 1] += f;
        H[c->L_rowadr[pq+a] + c->L_rownnz[pq+a] - 1] += f;
      }
    }
  }
  mjtNum maxdiag = 0;
  for (int r=0; r < n; r++) {
    maxdiag = mju_max(maxdiag, H[c->L_rowadr[r]+c->L_rownnz[r]-1]);
  }
  if (effCholNumericBlocked(c, values, H, mju_max(mjMINVAL, EFF_CHOL_PIVOT*maxdiag),
                            scratch) == n) {
    *status = 1;
  }
  mj_freeStack(d);
}

static void effCholEnsure(const mjModel* m, mjData* d) {
  int nb = d->nefmdof;
  if (!effCholReserved(d) || d->efm_L[9*nb] != EFF_CHOL_PENDING) {
    return;
  }
  d->efm_L[9*nb] = 0;
  mjEffChol c;
  effCholStep(d, &c);
  if (mj_stackBytesAvailable(d) >= effCholFillBytes(&c, m->nv)) {
    effCholFill(m, d, &c, d->efm_L + 9*nb, effCholStepValues(d, &c), NULL);
  }
}

static void effCholSolveBlocked(mjtNum* res, const mjtNum* L, const mjtNum* vec,
                                const mjEffChol* c) {
  int nb = c->nb;
  const int *rowadr = c->L_rowadr, *rownnz = c->L_rownnz, *colind = c->L_colind;
  mju_copy(res, vec, 3*nb);
  for (int p=nb-1; p >= 0; p--) {
    int nob = rownnz[3*p] - 1;
    const mjtNum *L0 = L + rowadr[3*p], *L1 = L + rowadr[3*p+1], *L2 = L + rowadr[3*p+2];
    mjtNum* r = res + 3*p;
    mjtNum x2 = r[2] / L2[nob+2];
    mjtNum x1 = (r[1] - L2[nob+1]*x2) / L1[nob+1];
    mjtNum x0 = (r[0] - L2[nob]*x2 - L1[nob]*x1) / L0[nob];
    r[0] = x0;
    r[1] = x1;
    r[2] = x2;
    const int* ci = colind + rowadr[3*p];
    for (int j=0; j < nob; j += 3) {
      mjtNum* rq = res + ci[j];
      rq[0] -= L0[j]*x0 + L1[j]*x1 + L2[j]*x2;
      rq[1] -= L0[j+1]*x0 + L1[j+1]*x1 + L2[j+1]*x2;
      rq[2] -= L0[j+2]*x0 + L1[j+2]*x1 + L2[j+2]*x2;
    }
  }
  for (int p=0; p < nb; p++) {
    int nob = rownnz[3*p] - 1;
    const mjtNum *L0 = L + rowadr[3*p], *L1 = L + rowadr[3*p+1], *L2 = L + rowadr[3*p+2];
    const int* ci = colind + rowadr[3*p];
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

enum { EFF_CON_COUNT, EFF_CON_COLLECT, EFF_CON_FOLD };

typedef struct {
  int op;
  const mjEffChol* c;
  const int* blk;
  int *slot, *vk, *pair;
  mjtNum *v, *H;
  size_t npair, maxpair, ntotal, ndiag;
} mjEffConFold;

static int effConPair(const mjEffChol* c, int k, int q, int* pos) {
  int pk = c->perm[k], pq = c->perm[q];
  int lo_p = pk > pq ? pk : pq, hi_p = pk > pq ? pq : pk;
  int j = effCholFind(c->L_rownnz, c->L_rowadr, c->L_colind, 3*lo_p, 3*hi_p);
  if (j < 0) {
    return 0;
  }
  int off = j - c->L_rowadr[3*lo_p];
  for (int a=0; a < 3; a++) {
    for (int b=0; b < 3; b++) {
      int s = pk > pq ? a : b, t = pk > pq ? b : a;
      pos[3*a+b] = c->L_rowadr[3*lo_p+s] + off + t;
    }
  }
  return 1;
}

static void effConTerm(mjEffConFold* f, mjtNum s, int nnz, const int* colind,
                       const mjtNum* val) {
  if (s <= 0) {
    return;
  }
  const mjEffChol* c = f->c;
  int ns = 0;
  for (int a=0; a < nnz; a++) {
    int i = colind ? colind[a] : a;
    int k = f->blk[i];
    if (k < 0 || !val[a]) continue;
    int sl = f->slot[k];
    if (sl < 0) {
      sl = f->slot[k] = ns;
      f->vk[ns++] = k;
      mju_zero3(f->v + 3*sl);
    }
    f->v[3*sl+i-c->dofid[k]] += val[a];
  }
  if (f->op == EFF_CON_COUNT) {
    f->ndiag += ns;
    f->ntotal += (size_t)ns * (ns - 1) / 2;
    for (int i=0; i < ns; i++) f->slot[f->vk[i]] = -1;
    return;
  }
  if (f->op == EFF_CON_FOLD) {
    for (int i=0; i < ns; i++) {
      int pk = 3*c->perm[f->vk[i]];
      const mjtNum* vk = f->v + 3*i;
      for (int a=0; a < 3; a++) {
        int dk = c->L_rowadr[pk+a] + c->L_rownnz[pk+a] - 1 - a;
        for (int b=0; b <= a; b++) {
          f->H[dk+b] += s * vk[a] * vk[b];
        }
      }
    }
  }
  int pos[9];
  for (int i=0; i < ns; i++) {
    for (int j=i+1; j < ns; j++) {
      int k = f->vk[i], q = f->vk[j];
      if (f->op == EFF_CON_COLLECT) {
        if (f->npair < f->maxpair) {
          f->pair[2*f->npair] = k < q ? k : q;
          f->pair[2*f->npair+1] = k < q ? q : k;
          f->npair++;
        }
      } else {
        const mjtNum *vk = f->v + 3*i, *vq = f->v + 3*j;
        if (effConPair(c, k, q, pos)) {
          for (int a=0; a < 3; a++) {
            for (int b=0; b < 3; b++) f->H[pos[3*a+b]] += s * vk[a] * vq[b];
          }
        } else {
          int pk = 3*c->perm[k], pq = 3*c->perm[q];
          for (int a=0; a < 3; a++) {
            int dk = c->L_rowadr[pk+a] + c->L_rownnz[pk+a] - 1 - a;
            int dq = c->L_rowadr[pq+a] + c->L_rownnz[pq+a] - 1 - a;
            for (int b=0; b <= a; b++) {
              f->H[dk+b] += s * vk[a] * vk[b];
              f->H[dq+b] += s * vq[a] * vq[b];
            }
          }
        }
      }
    }
  }
  for (int i=0; i < ns; i++) f->slot[f->vk[i]] = -1;
}

static void effConTerms(const mjModel* m, const mjData* d, mjEffConFold* f, int op,
                        const mjEffConRows* rows) {
  f->op = op;
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    effConTerm(f, e.scale, e.nnz, e.colind, e.val);
  }
  for (int r=0; r < rows->nefc; r++) {
    if (!rows->D[r]) continue;
    if (rows->is_sparse) {
      int adr = rows->rowadr[r];
      effConTerm(f, rows->D[r], rows->rownnz[r], rows->colind + adr, rows->J + adr);
    } else {
      effConTerm(f, rows->D[r], m->nv, NULL, rows->J + (size_t)r*m->nv);
    }
  }
}

static void effConScratch(mjData* d, int nb, mjEffConFold* f) {
  f->slot = effInts(d, nb);
  f->vk = effInts(d, nb);
  f->v = mjSTACKALLOC(d, 3*nb, mjtNum);
  for (int k=0; k < nb; k++) f->slot[k] = -1;
}

// allocate on the stack of d, symbolically analyse, and factor this solve's contact-adaptive
// sparse factor (or fall back to the step's pattern in fold->F)
mjEffFactor* mj_effCholFoldFactor(const mjModel* m, mjData* d, const mjEffFold* fold,
                                  int nefc, const mjtNum* efc_D, int is_sparse,
                                  const mjtNum* J, const int* J_rownnz, const int* J_rowadr,
                                  const int* J_colind) {
  if (!fold || !fold->F) {
    return NULL;
  }
  fold->F[0] = 0;
  if (!effCholReserved(d)) {
    return NULL;
  }
  int nb = d->nefmdof, nv = m->nv;
  mjEffChol cs;
  effCholStep(d, &cs);
  size_t avail = mj_stackBytesAvailable(d);
  size_t con_bytes = effConScratchBytes(nb);
  if (avail < mj_stackFrameBytes() + effIntBytes(nv) + con_bytes) {
    return NULL;
  }
  mjEffConRows rows = {nefc, efc_D, is_sparse, J, J_rownnz, J_rowadr, J_colind};
  mj_markStack(d);
  int* blk = effInts(d, nv);
  effCovered(nv, nb, cs.dofid, blk);
  mjEffConFold con = {.c = &cs, .blk = blk};
  effConScratch(d, nb, &con);
  effConTerms(m, d, &con, EFF_CON_COUNT, &rows);
  mj_freeStack(d);

  size_t ntotal = con.ntotal, ndiag = con.ndiag;
  if (!ntotal && !ndiag) {
    fold->F[0] = EFF_CHOL_PENDING;
    return NULL;
  }

  mjEffFactor* dyn = NULL;
  if (ntotal > 0 && ntotal <= INT_MAX/2) {
    size_t kcap = effCholKcap(d, nb, cs.dofid);
    size_t ints_bytes = effIntBytes(nv) + effIntBytes(nb) + effIntBytes(2*ntotal);
    size_t sym_bytes = mj_stackFrameBytes() + con_bytes + effCholAnalysisBytes(m, nb, kcap, ntotal);
    size_t apply_bytes = (fold->nu ? effIntBytes(3*(size_t)m->ntree) : 0) +
                         mj_effFoldScratch(m, d, fold->nu);
    size_t mul_bytes = mj_effMulAddScratch(m, d);
    size_t tail = apply_bytes > mul_bytes ? apply_bytes : mul_bytes;
    if (avail < ints_bytes + (sym_bytes > tail ? sym_bytes : tail)) {
      mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    } else {
      blk = effInts(d, nv);
      int* perm = effInts(d, nb);
      int* pair = effInts(d, 2*ntotal);
      effCovered(nv, nb, cs.dofid, blk);
      mj_markStack(d);
      con.blk = blk;
      effConScratch(d, nb, &con);
      con.pair = pair;
      con.maxpair = ntotal;
      con.npair = 0;
      effConTerms(m, d, &con, EFF_CON_COLLECT, &rows);
      qsort(pair, con.npair, 2*sizeof(int), effPairCompare);
      int nx = 0;
      for (size_t x=0; x < con.npair; x++) {
        if (nx && pair[2*x] == pair[2*nx-2] && pair[2*x+1] == pair[2*nx-1]) continue;
        pair[2*nx] = pair[2*x];
        pair[2*nx+1] = pair[2*x+1];
        nx++;
      }
      mjEffCholPass pass;
      int ok = effCholBegin(m, d, nb, cs.dofid, blk, d->efm_K_rownnz, d->efm_K_rowadr,
                            d->efm_K_colind, pair, nx, /*order=*/1, perm, &pass) && pass.nfe;
      mj_freeStack(d);
      if (ok) {
        size_t npack = effPackNums(nb, pass.nL, pass.ne);
        size_t dyn_bytes = mj_stackBytes(sizeof(mjEffFactor), _Alignof(mjEffFactor)) +
                           effNumBytes(EFF_HDR + npack + (size_t)pass.nL);
        size_t pass2_bytes = effCholAnalysisBytes(m, nb, kcap, nx);
        size_t fill_bytes = mj_stackFrameBytes() + effIntBytes(nv) + con_bytes +
                            effCholFillBytes(&(mjEffChol){.nb = nb, .nL = pass.nL}, nv);
        size_t rest = pass2_bytes > fill_bytes ? pass2_bytes : fill_bytes;
        if (rest < tail) rest = tail;
        if (mj_stackBytesAvailable(d) < dyn_bytes + rest) {
          mj_warning(d, mjWARN_CNSTRFULL, d->narena);
        } else {
          dyn = mjSTACKALLOC(d, 1, mjEffFactor);
          dyn->F = mjSTACKALLOC(d, EFF_HDR + npack + pass.nL, mjtNum);
          dyn->F[0] = 0;
          int* A = (int*)(dyn->F + EFF_HDR);
          A[0] = pass.nL;
          A[1] = pass.ne;
          effPackLayout(A, nb, cs.dofid, &dyn->c);
          memcpy(dyn->c.perm, perm, sizeof(int)*nb);
          mj_markStack(d);
          effCholBegin(m, d, nb, cs.dofid, blk, d->efm_K_rownnz, d->efm_K_rowadr,
                       d->efm_K_colind, pair, nx, /*order=*/0, dyn->c.perm, &pass);
          effCholFinish(&pass, &dyn->c);
          mj_freeStack(d);
        }
      }
    }
  }

  mj_markStack(d);
  const mjEffChol* cf = NULL;
  mjtNum *F = NULL, *Fval = NULL;
  if (dyn && mj_stackBytesAvailable(d) >= effIntBytes(nv) + con_bytes +
                                          effCholFillBytes(&dyn->c, nv)) {
    cf = &dyn->c;
    F = dyn->F;
    Fval = dyn->F + EFF_HDR + effPackNums(nb, dyn->c.nL, dyn->c.ne);
  } else if (mj_stackBytesAvailable(d) >= effIntBytes(nv) + con_bytes +
                                          effCholFillBytes(&cs, nv)) {
    dyn = NULL;
    cf = &cs;
    F = fold->F;
    Fval = fold->F + EFF_HDR;
  }
  if (cf) {
    blk = effInts(d, nv);
    effCovered(nv, nb, cs.dofid, blk);
    con.blk = blk;
    effConScratch(d, nb, &con);
    con.c = cf;
    con.H = mjSTACKALLOC(d, cf->nL, mjtNum);
    mju_zero(con.H, cf->nL);
    effConTerms(m, d, &con, EFF_CON_FOLD, &rows);
    effCholFill(m, d, cf, F, Fval, con.H);
  }
  mj_freeStack(d);
  return (dyn && dyn->F[0] == 1) ? dyn : NULL;
}

// analyse the step's efm_K pattern and allocate d->efm_L with the analysis and the factor tail
static mjtNum* effCholReserve(const mjModel* m, mjData* d, int nb, const int* dofid, int* ntail) {
  *ntail = 0;
  if (!effCholApplies(m, nb, dofid)) {
    return NULL;
  }
  int nv = m->nv;
  size_t kcap = effCholKcap(d, nb, dofid);
  size_t blocks_bytes = mj_stackBytes(sizeof(mjtNum)*9*nb, _Alignof(mjtNum));
  size_t avail = mj_stackBytesAvailable(d);
  size_t sym_bytes = mj_stackFrameBytes() + effIntBytes(nv) + effIntBytes(nb) +
                     effCholAnalysisBytes(m, nb, kcap, 0);
  if (avail < blocks_bytes + sym_bytes) {
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return NULL;
  }
  mj_markStack(d);
  int* blk = effInts(d, nv);
  int* perm = effInts(d, nb);
  effCovered(nv, nb, dofid, blk);
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
                        NULL, 0, /*order=*/0, perm, &pass) &&
           pass.nfe;
  if (!ok) {
    mj_freeStack(d);
    return NULL;
  }
  size_t npack = effPackNums(nb, pass.nL, pass.ne);
  if (avail < blocks_bytes + effCholReserveBytes(nb, nv, pass.nL, npack)) {
    mj_freeStack(d);
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);
    return NULL;
  }
  int tail = EFF_HDR + (int)npack + pass.nL;
  mjtNum* B = (mjtNum*)effAlloc(d, sizeof(mjtNum)*(9*(size_t)nb + tail), _Alignof(mjtNum));
  if (!B) {
    mj_freeStack(d);
    return NULL;
  }
  B[9*nb] = EFF_CHOL_PENDING;
  int* A = (int*)(B + 9*nb + EFF_HDR);
  A[0] = pass.nL;
  A[1] = pass.ne;
  mjEffChol c;
  effPackLayout(A, nb, dofid, &c);
  memcpy(c.perm, perm, sizeof(int)*nb);
  effCholFinish(&pass, &c);
  mj_freeStack(d);
  *ntail = tail;
  return B;
}

static int effCholSolve(const mjModel* m, mjData* d, const mjEffFold* fold, mjtNum* x,
                        const mjtNum* b) {
  int nb = d->nefmdof;
  if (!effCholReserved(d)) {
    return 0;
  }
  mjEffChol cs;
  const mjEffChol* c = &cs;
  const mjtNum* Ls = NULL;
  if (fold) {
    if (fold->factor && fold->factor->F[0] == 1) {
      c = &fold->factor->c;
      Ls = fold->factor->F + EFF_HDR + effPackNums(nb, c->nL, c->ne);
    } else if (fold->F && fold->F[0] == 1) {
      effCholStep(d, &cs);
      Ls = fold->F + EFF_HDR;
    } else if (!fold->F || fold->F[0] != EFF_CHOL_PENDING) {
      return 0;
    }
  }
  if (!Ls) {
    effCholEnsure(m, d);
    if (d->efm_L[9*nb] != 1) {
      return 0;
    }
    effCholStep(d, &cs);
    Ls = effCholStepValues(d, &cs);
  }
  effCholApply(d, c, x, b, Ls);
  return 1;
}

int mj_effCholFoldSize(const mjData* d) {
  if (!effCholReserved(d)) {
    return 0;
  }
  mjEffChol c;
  effCholStep(d, &c);
  return EFF_HDR + c.nL;
}

static size_t effCholApplyScratch(const mjData* d) {
  if (!effCholReserved(d)) {
    return 0;
  }
  mjEffChol c;
  effCholStep(d, &c);
  return mj_stackFrameBytes() + 2*effNumBytes(c.n) + effCholFillBytes(&c, c.n);
}


// Build and factor the per-vertex 3x3 diagonal blocks of the flex part of (M + K), stored in
// d->efm_L, 9 numbers per covered vertex: O(n) to build and apply, approximate where the sparse
// factorization it replaces was exact. Both consumers use the blocks as a preconditioner: the CG
// constraint solver (Mgrad = Mtilde \ grad) and the qacc_smooth PCG in mj_effSolve, which
// supplies the accuracy. Under IPC the buffer can also hold a sparse factor of the covered
// block, which then replaces the blocks (effCholReserve).
static void effBlocks(const mjModel* m, mjData* d) {
  int nv = m->nv;

  // covered dofs come in contiguous triples (the 3 slide dofs of one flex point), but the first
  // one need not be at a multiple of 3: any joint declared before the flex shifts them. Walk the
  // covered rows rather than striding the dof index, which would straddle point boundaries.
  int nb = effCoveredBlocks(nv, d->efm_K_rownnz, NULL);
  d->nefmdof = 0;
  int* adr = (int*) effAlloc(d, sizeof(int)*(nb > 0 ? nb : 1), _Alignof(int));
  effCoveredBlocks(nv, d->efm_K_rownnz, adr);
  int ntail = 0;
  mjtNum* B = effCholReserve(m, d, nb, adr, &ntail);
  if (!B) {
    B = (mjtNum*) effAlloc(d, sizeof(mjtNum)*9*(nb > 0 ? nb : 1), _Alignof(mjtNum));
  }
  for (int k = 0; k < nb; k++) {
    mjtNum* Bk = B + 9*k;
    effBlockRaw(m, d, adr[k], Bk);
    mju_cholFactor(Bk, 3, mjMINVAL);
  }
  d->efm_L = B;
  d->efm_dofid = adr;
  d->nefmdof = nb;
  d->nefmL = 9*nb + ntail;
}

// Apply the metric preconditioner: the per-step 3x3 blocks when they exist, else the constant
// bending factor from mj_setConst, on the dofs they cover; M^-1 on all other dofs. PCG requires
// symmetry, so covered and uncovered dofs must not see each other: zeroing the covered entries
// of the right-hand side before the qLD sweep keeps the uncovered rows from reading them.
// x = (L L') \ b for one 3x3 factor from mju_cholFactor, in mju_cholSolve's arithmetic; b may
// alias x
static inline void chol3Solve(mjtNum* x, const mjtNum* L, const mjtNum* b) {
  mjtNum r0 = b[0] / L[0];
  mjtNum r1 = (b[1] - L[3]*r0) / L[4];
  mjtNum r2 = (b[2] - (L[6]*r0 + L[7]*r1)) / L[8];
  r2 /= L[8];
  r1 = (r1 - L[7]*r2) / L[4];
  r0 -= L[3]*r1;
  r0 -= L[6]*r2;
  r0 /= L[0];
  x[0] = r0;
  x[1] = r1;
  x[2] = r2;
}

// x[U] = S \ bu on each component of the fold's dense factors, bu = b[U] gathered by the
// caller and overwritten
static void effDenseApply(const mjEffFold* fold, mjtNum* x, mjtNum* bu) {
  for (int c = 0; c < fold->ncomp; c++) {
    int adr = fold->Uadr[c], n = fold->Uadr[c+1] - adr;
    mju_cholSolve(bu + adr, fold->S + fold->Sadr[c], bu + adr, n);
  }
  for (int j = 0; j < fold->nu; j++) {
    x[fold->U[j]] = bu[j];
  }
}

// x = (M + K)_covered \ b on the covered dofs: the sparse factor of fold (or of the metric, when
// fold is NULL) if valid, else the 3x3 blocks L; b may alias x
static void effCovSolve(const mjModel* m, mjData* d, const mjtNum* L, const mjEffFold* fold,
                        mjtNum* x, const mjtNum* b) {
  if (effCholSolve(m, d, fold, x, b)) {
    return;
  }
  for (int k = 0; k < d->nefmdof; k++) {
    int i = d->efm_dofid[k];
    chol3Solve(x + i, L + 9*k, b + i);
  }
}

// fold: the solver's folded copy (mj_effPrecFold) with L its blocks, or NULL
static void effBlockApply(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b,
                          const mjtNum* L, const mjEffFold* fold) {
  int nv = m->nv;
  int nbd = m->nefm0dof;
  int flg_bend = nbd && !d->nefmdof;
  int flg_dense = fold && fold->S_valid;

  // the fold's dense factors on all the uncovered dofs, the sparse factor or the 3x3 blocks on the
  // rest: no backbone
  if (flg_dense && !fold->partial) {
    mj_markStack(d);
    mjtNum* bu = mjSTACKALLOC(d, fold->nu, mjtNum);
    for (int j = 0; j < fold->nu; j++) {
      bu[j] = b[fold->U[j]];   // before the covered solve writes x: b may alias x
    }
    effCovSolve(m, d, L, fold, x, b);
    effDenseApply(fold, x, bu);
    mj_freeStack(d);
    return;
  }

  // every dof a covered triple: the blocks are the whole preconditioner
  if (3*d->nefmdof == nv && !flg_bend) {
    effCovSolve(m, d, L, fold, x, b);
    return;
  }
  mj_markStack(d);
  mjtNum* rhs = mjSTACKALLOC(d, nv, mjtNum);
  mju_copy(rhs, b, nv);   // b may alias x, which the sweep below overwrites

  // dofs no factor covers: backbone solve, using the qH backbone factor when diagonal
  // terms exist, else qLD
  mju_copy(x, rhs, nv);
  for (int k = 0; k < d->nefmdof; k++) {
    mju_zero(x + d->efm_dofid[k], 3);
  }
  if (flg_bend) {
    for (int i = 0; i < nbd; i++) {
      x[m->efm0_dofid[i]] = 0;
    }
  }
  // TODO(team): restrict solve to awake trees when island sleep filtering is active
  if (d->efm_diag) {
    mj_solveLD(x, d->qH, d->qHDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
  } else {
    mj_solveLD(x, d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
  }

  // per-step stiffness: the sparse factor or the 3x3 blocks
  effCovSolve(m, d, L, fold, x, rhs);

  // the fold's dense factors where some uncovered dofs are in no component: each component is
  // a union of whole trees, which the backbone solve keeps apart, so overwriting its dofs
  // leaves the backbone solve on the other trees as it was
  if (flg_dense) {
    mjtNum* bu = mjSTACKALLOC(d, fold->nu, mjtNum);
    for (int j = 0; j < fold->nu; j++) {
      bu[j] = rhs[fold->U[j]];
    }
    effDenseApply(fold, x, bu);
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
  mj_freeStack(d);
}


// accurate solve of (M + K) x = b by PCG with the 3x3 block preconditioner, converging the
// relative residual to opt.tolerance; used for qacc_smooth. Reaching opt.iterations means the
// metric is too ill-conditioned for the blocks: warn (mjWARN_INERTIA, worst-residual dof) and
// return x under-converged.
void mj_effSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  if (!d->efm_active) {
    mj_effPrec(m, d, x, b);
    return;
  }

  // backbone-only metric (no tendon, actuator or flex couplings): the qH solve inside the
  // block-apply is the exact metric solve, skip the iteration. Beyond the saved iterations,
  // the direct solve is block-decoupled across trees, which sleeping trees rely on
  int flex_any = 0;
  for (int f=0; f < m->nflex; f++) {
    flex_any = flex_any || mj_effFlexPossible(m, f);
  }
  if (!d->nefmT && !d->nefmA && !flex_any) {
    effBlockApply(m, d, x, b, d->efm_L, NULL);
    return;
  }
  int nv = m->nv;
  mj_markStack(d);
  mjtNum* r = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* z = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* p = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* Ap = mjSTACKALLOC(d, nv, mjtNum);
  mju_copy(r, b, nv);   // before zeroing x: b may alias x
  mju_zero(x, nv);
  mjtNum bn = mju_dot(r, r, nv);
  if (bn > mjMINVAL) {
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
    effBlockApply(m, d, z, r, d->efm_L, NULL);
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
      effBlockApply(m, d, z, r, d->efm_L, NULL);
      mjtNum rznew = mju_dot(r, z, nv);
      mju_addScl(p, z, p, rznew/rz, nv);
      rz = rznew;
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
  }
  mj_freeStack(d);
}

void mj_effPrec(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  int nv = m->nv;

  // active metric: the prefactored 3x3 blocks are the preconditioner
  if (d->efm_active) {
    effBlockApply(m, d, x, b, d->efm_L, NULL);
    return;
  }

  // inactive metric: x = M \ b, which is exact
  if (x != b) {
    mju_copy(x, b, nv);
  }
  mj_solveLD(x, d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
}


//------------- dense uncovered-dof blocks of the CG preconditioner --------------------------------
// The 3x3 blocks precondition only the flex vertex triples; every other dof (the articulated
// trees, free bodies) gets the backbone solve of M + diag alone, without the rank-1 classes and
// the solve's efc rows (dof friction, limits, contacts on the trees), and those stiff rows then
// set the CG iteration count. The solver's fold therefore also carries dense factors of the
// uncovered block
//   S = M_uu + diag + fluid + rank-1 classes + J_u' D J_u   (rows with D > 0),
// applied as block Jacobi next to the 3x3 blocks; S is SPD whenever the unfolded matrices are.
// S is block diagonal over its connected components: M and the fluid blocks stay within a
// tree, so only the efc rows and the rank-1 terms join trees. Each component is factored on its
// own, which is exact for S at sum(n_c^3)/6 multiply-adds per solve, and the dofs of the
// components over the cap keep the backbone solve. The metric's own preconditioner has none:
// the qacc_smooth PCG keeps the backbone solve.

// largest component of the dense blocks: its factorization per solve costs n^3/6 multiply-adds
// (36M here), which a solve that the block saves iterations on still recovers
#define EFF_DENSE_MAXNU 600

// union-find root of tree t, halving the path
static int effRoot(int* par, int t) {
  while (par[t] != t) {
    par[t] = par[par[t]];
    t = par[t];
  }
  return t;
}


// join the trees of the uncovered dofs (cov[i] < 0) among the n columns ind
static void effJoin(const mjModel* m, const int* cov, int* par, const int* ind, int n) {
  int r0 = -1;
  for (int a=0; a < n; a++) {
    if (cov[ind[a]] >= 0) {
      continue;
    }
    int r = effRoot(par, m->dof_treeid[ind[a]]);
    if (r0 < 0) {
      r0 = r;
    } else if (r != r0) {
      par[r] = r0;
    }
  }
}


// the connected components of S over the uncovered dofs (cov[i] < 0): their trees, joined by the
// efc rows with D > 0 and the rank-1 terms, kept when of at most EFF_DENSE_MAXNU dofs. Sets
// comp[t], the component of tree t (numbered in the order of their first dof), -1: no uncovered
// dofs or over the cap. Returns the components, with *nu their dofs, *nS the mjtNums of their
// packed factors and *partial set if uncovered dofs were left out. par, cnt: ntree ints scratch
static int effComponents(const mjModel* m, const mjData* d, const int* cov,
                         int nefc, const mjtNum* efc_D, const int* J_rownnz,
                         const int* J_rowadr, const int* J_colind,
                         int* comp, int* par, int* cnt, int* nu, int* nS, int* partial) {
  int nv = m->nv, ntree = m->ntree;
  for (int t=0; t < ntree; t++) {
    par[t] = t;
    cnt[t] = 0;
    comp[t] = -1;
  }
  for (int i=0; i < nv; i++) {
    if (cov[i] < 0) {
      cnt[m->dof_treeid[i]]++;
    }
  }

  // join the trees each rank-1 term and efc row couples
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    effJoin(m, cov, par, e.colind, e.nnz);
  }
  for (int r=0; r < nefc; r++) {
    if (efc_D[r] > 0) {
      effJoin(m, cov, par, J_colind + J_rowadr[r], J_rownnz[r]);
    }
  }

  // component sizes, at the roots
  for (int t=0; t < ntree; t++) {
    int r = effRoot(par, t);
    if (r != t) {
      cnt[r] += cnt[t];
    }
  }

  // number the components by their first dof, those within the cap (-2: over it)
  int ncomp = 0;
  *nu = *nS = *partial = 0;
  for (int i=0; i < nv; i++) {
    if (cov[i] >= 0) {
      continue;
    }
    int r = effRoot(par, m->dof_treeid[i]);
    if (comp[r] != -1) {
      continue;
    }
    int n = cnt[r];
    if (n <= EFF_DENSE_MAXNU && *nS <= INT_MAX - n*n) {
      comp[r] = ncomp++;
      *nu += n;
      *nS += n*n;
    } else {
      comp[r] = -2;
      *partial = 1;
    }
  }
  for (int t=0; t < ntree; t++) {
    int c = comp[effRoot(par, t)];
    comp[t] = c < 0 ? -1 : c;
  }
  return ncomp;
}


// size the dense blocks of a fold over the given efc rows, 0: none
int mj_effFoldDenseSize(const mjModel* m, mjData* d, int nefc, const mjtNum* efc_D,
                        int is_sparse, const int* J_rownnz, const int* J_rowadr,
                        const int* J_colind, int* nu, int* ncomp) {
  *nu = *ncomp = 0;
  if (!d->nefmdof || !is_sparse || 3*d->nefmdof == m->nv) {
    return 0;
  }
  int ntree = m->ntree, nS, partial;
  mj_markStack(d);
  int* cov = mjSTACKALLOC(d, m->nv, int);
  int* tint = mjSTACKALLOC(d, 3*ntree, int);
  effCovered(m->nv, d->nefmdof, d->efm_dofid, cov);
  *ncomp = effComponents(m, d, cov, nefc, efc_D, J_rownnz, J_rowadr, J_colind,
                         tint, tint + ntree, tint + 2*ntree, nu, &nS, &partial);
  mj_freeStack(d);
  return nS;
}


// stack bytes of mj_effPrecFold and mj_effPrecBlocks beyond the fold's arrays, at most: the
// fold's frame (block additions, covered marks and, with a dense block, the components'
// scratch, which also bounds mj_effFoldDenseSize) or the apply's (the backbone right-hand side
// and the dense right-hand side, or the bending-only factor's vectors)
size_t mj_effFoldScratch(const mjModel* m, const mjData* d, int nu) {
  size_t frame = mj_stackFrameBytes();
  size_t sn = sizeof(mjtNum), an = _Alignof(mjtNum), si = sizeof(int), ai = _Alignof(int);
  size_t fold = frame + mj_stackBytes(sn*9*d->nefmdof, an) + mj_stackBytes(si*m->nv, ai) +
                (nu ? mj_stackBytes(si*3*m->ntree, ai) : 0);
  size_t apply = frame + mj_stackBytes(sn*m->nv, an) + mj_stackBytes(sn*nu, an) +
                 effCholApplyScratch(d);
  if (m->nefm0dof && !d->nefmdof) {
    apply += 2*mj_stackBytes(sn*m->nefm0dof, an);
  }
  return mjMAX(fold, apply);
}


// stack bytes of mj_effMulAdd, at most: the frames of the matrix-free flex operators it falls
// back to, one at a time
size_t mj_effMulAddScratch(const mjModel* m, const mjData* d) {
  size_t frame = mj_stackFrameBytes(), bytes = 0;
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_interp[f] || m->flex_rigid[f] || m->flex_dim[f] < 2) continue;
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
  return bytes;
}


// assemble and factor the dense blocks of the components into fold->S; dof i is at loc[i]
// within its component comp[dof_treeid[i]], loc[i] = -1 outside the components. The efc rows
// are in the sparse Jacobian. A component holds every uncovered dof its rows and rank-1 terms
// reach (they joined its trees), and M and the fluid blocks stay within a tree
static void effDenseBlock(const mjModel* m, const mjData* d, mjEffFold* fold, const int* loc,
                          const int* comp, int nefc, const mjtNum* efc_D, const mjtNum* J,
                          const int* J_rownnz, const int* J_rowadr, const int* J_colind) {
  mju_zero(fold->S, fold->nS);

  // M_uu (lower rows, ancestors then the diagonal), the diagonal classes and the fluid blocks
  for (int c=0; c < fold->ncomp; c++) {
    int n = fold->Uadr[c+1] - fold->Uadr[c];
    mjtNum* S = fold->S + fold->Sadr[c];
    for (int j=0; j < n; j++) {
      int i = fold->U[fold->Uadr[c] + j];
      for (int a=m->M_rowadr[i]; a < m->M_rowadr[i] + m->M_rownnz[i]; a++) {
        int cu = loc[m->M_colind[a]];
        if (cu < 0) {
          continue;
        }
        mjtNum v = d->M[a] + (d->efm_fluid ? d->efm_fluid[a] : 0);
        S[j*n + cu] += v;
        if (cu != j) {
          S[cu*n + j] += v;
        }
      }
      if (d->efm_diag) {
        S[j*n + j] += d->efm_diag[i];
      }
    }
  }

  // rank-1 classes
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    for (int a=0; a < e.nnz; a++) {
      int ia = e.colind[a], ua = loc[ia];
      if (ua < 0) {
        continue;
      }
      int c = comp[m->dof_treeid[ia]], n = fold->Uadr[c+1] - fold->Uadr[c];
      mjtNum* S = fold->S + fold->Sadr[c];
      for (int b=0; b < e.nnz; b++) {
        int ib = e.colind[b], ub = loc[ib];
        if (ub >= 0 && comp[m->dof_treeid[ib]] == c) {
          S[ua*n + ub] += e.scale * e.val[a] * e.val[b];
        }
      }
    }
  }

  // efc rows: J_u' D J_u
  for (int r=0; r < nefc; r++) {
    mjtNum D = efc_D[r];
    if (D <= 0) {
      continue;
    }
    int adr = J_rowadr[r], nnz = J_rownnz[r];
    for (int a=0; a < nnz; a++) {
      int ia = J_colind[adr+a], ua = loc[ia];
      if (ua < 0) {
        continue;
      }
      int c = comp[m->dof_treeid[ia]], n = fold->Uadr[c+1] - fold->Uadr[c];
      mjtNum* S = fold->S + fold->Sadr[c];
      mjtNum DJa = D * J[adr+a];
      for (int b=0; b < nnz; b++) {
        int ib = J_colind[adr+b], ub = loc[ib];
        if (ub >= 0 && comp[m->dof_treeid[ib]] == c) {
          S[ua*n + ub] += DJa * J[adr+b];
        }
      }
    }
  }

  // factor each component
  int valid = 1;
  for (int c=0; c < fold->ncomp; c++) {
    int n = fold->Uadr[c+1] - fold->Uadr[c];
    valid = valid && mju_cholFactor(fold->S + fold->Sadr[c], n, mjMINVAL) == n;
  }
  fold->S_valid = valid;
}


// fold the metric's rank-1 classes and the efc rows (quadratic zone) into a copy of the
// preconditioner blocks, factored into fold->L (9*nefmdof); only each term's per-vertex 3x3
// diagonal survives, as for the elastic part. With fold->nu, also the dense blocks of the
// components of the uncovered dofs (effDenseBlock). Returns 0 when nothing is covered, leaving
// fold untouched. Stack: one frame (mj_effFoldScratch)
int mj_effPrecFold(const mjModel* m, mjData* d, mjEffFold* fold,
                   int nefc, const mjtNum* efc_D, int is_sparse,
                   const mjtNum* J, const int* J_rownnz, const int* J_rowadr,
                   const int* J_colind) {
  if (!d->nefmdof) {
    return 0;
  }
  mj_markStack(d);
  int nv = m->nv;
  mjtNum* L = fold->L;
  mjtNum* Badd = mjSTACKALLOC(d, 9*d->nefmdof, mjtNum);
  int* blk = mjSTACKALLOC(d, nv, int);
  mju_zero(Badd, 9*d->nefmdof);
  effCovered(nv, d->nefmdof, d->efm_dofid, blk);

  // rank-1 classes: scale * v v', restricted to each covered block. A term's coupling between
  // DIFFERENT vertices is off-diagonal and cannot be represented here; only its self-terms land
  mjEffRank1Iter it = {0};
  mjEffRank1 e;
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    for (int a=0; a < e.nnz; a++) {
      int ia = e.colind[a], k = blk[ia];
      if (k < 0) {
        continue;
      }
      int base = d->efm_dofid[k];
      for (int b=0; b < e.nnz; b++) {
        int ib = e.colind[b];
        if (blk[ib] == k) {
          Badd[9*k + 3*(ia-base) + (ib-base)] += e.scale * e.val[a] * e.val[b];
        }
      }
    }
  }

  // efc rows: the same blocks of J'*D*J, over every row the solve carries, active or not. The
  // rows are the pairs inside their wall, the likely active set, and rows switch state in
  // nearly every CG iteration, so a fold of the rows active at the start left the CG with a
  // preconditioner that matched neither iterate: on the reef knot the mean iterations per
  // solve fell from 212 to 59 when every row was folded
  for (int r=0; r < nefc; r++) {
    if (!efc_D[r]) {
      continue;
    }
    mjtNum D = efc_D[r];
    if (is_sparse) {
      int adr = J_rowadr[r], nnz = J_rownnz[r];
      for (int a=0; a < nnz; a++) {
        int ia = J_colind[adr+a], k = blk[ia];
        if (k < 0) {
          continue;
        }
        int base = d->efm_dofid[k];
        for (int b=0; b < nnz; b++) {
          int ib = J_colind[adr+b];
          if (blk[ib] == k) {
            Badd[9*k + 3*(ia-base) + (ib-base)] += D * J[adr+a] * J[adr+b];
          }
        }
      }
    } else {
      const mjtNum* Jr = J + (size_t)r*nv;
      for (int k=0; k < d->nefmdof; k++) {
        int base = d->efm_dofid[k];
        for (int a=0; a < 3; a++) {
          if (!Jr[base+a]) {
            continue;
          }
          for (int b=0; b < 3; b++) {
            Badd[9*k + 3*a + b] += D * Jr[base+a] * Jr[base+b];
          }
        }
      }
    }
  }

  for (int k=0; k < d->nefmdof; k++) {
    mjtNum* Bk = L + 9*k;
    effBlockRaw(m, d, d->efm_dofid[k], Bk);
    mju_addTo(Bk, Badd + 9*k, 9);
    mju_cholFactor(Bk, 3, mjMINVAL);
  }

  // dense blocks of the uncovered dofs: the components of the same rows mj_effFoldDenseSize
  // sized the fold for (a mismatch leaves the backbone solve), laid out by component in U
  fold->S_valid = 0;
  if (fold->nu && is_sparse) {
    int ntree = m->ntree, nu, nS, partial;
    int* tint = mjSTACKALLOC(d, 3*ntree, int);
    int* comp = tint;
    int* cur = tint + 2*ntree;
    int ncomp = effComponents(m, d, blk, nefc, efc_D, J_rownnz, J_rowadr, J_colind,
                              comp, tint + ntree, cur, &nu, &nS, &partial);
    if (nu == fold->nu && ncomp == fold->ncomp && nS == fold->nS) {
      // component offsets in U and S
      for (int c=0; c <= ncomp; c++) {
        fold->Uadr[c] = 0;
      }
      for (int i=0; i < nv; i++) {
        int c = blk[i] < 0 ? comp[m->dof_treeid[i]] : -1;
        if (c >= 0) {
          fold->Uadr[c+1]++;
        }
      }
      fold->Sadr[0] = 0;
      for (int c=0; c < ncomp; c++) {
        int n = fold->Uadr[c+1];
        fold->Uadr[c+1] += fold->Uadr[c];
        fold->Sadr[c+1] = fold->Sadr[c] + n*n;
        cur[c] = fold->Uadr[c];
      }

      // U by component, ascending within each; blk becomes the index within the component
      for (int i=0; i < nv; i++) {
        int c = blk[i] < 0 ? comp[m->dof_treeid[i]] : -1;
        if (c >= 0) {
          fold->U[cur[c]] = i;
          blk[i] = cur[c]++ - fold->Uadr[c];
        } else {
          blk[i] = -1;
        }
      }
      fold->partial = partial;
      effDenseBlock(m, d, fold, blk, comp, nefc, efc_D, J, J_rownnz, J_rowadr, J_colind);
    }
  }
  mj_freeStack(d);
  return 1;
}


// mj_effPrec against a solver-owned fold instead of the shared d->efm_L
void mj_effPrecBlocks(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b,
                      const mjEffFold* fold) {
  if (d->efm_active) {
    effBlockApply(m, d, x, b, fold->L, fold);
    return;
  }
  mj_effPrec(m, d, x, b);
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


// fill efm_ck with h * d(spring force)/d(displacement) per dof, clamped PSD-safe;
// mirrors the joint-spring structure of mj_springdamper; returns any-nonzero
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
    mjtNum k;

    switch ((mjtJoint) m->jnt_type[j]) {
    case mjJNT_FREE:
      // translation: radial derivative of the spring, isotropic on the 3 slide dofs
      {
        mjtNum dif[3];
        mju_sub3(dif, d->qpos+padr, m->qpos_spring+padr);
        k = mju_max(0, mjd_xPolyForce(stiffness, spoly, mju_norm3(dif), mjNPOLY, 0));
        d->efm_ck[dadr] = d->efm_ck[dadr+1] = d->efm_ck[dadr+2] = h*k;
        any = any || k > 0;
      }

      // continue with rotations
      dadr += 3;
      padr += 3;
      mjFALLTHROUGH;

    case mjJNT_BALL:
      // rotation: small-angle isotropic treatment of the quaternion spring
      {
        mjtNum dif[3], quat[4];
        mju_copy4(quat, d->qpos+padr);
        mju_normalize4(quat);
        mju_subQuat(dif, quat, m->qpos_spring + padr);
        k = mju_max(0, mjd_xPolyForce(stiffness, spoly, mju_norm3(dif), mjNPOLY, 0));
        d->efm_ck[dadr] = d->efm_ck[dadr+1] = d->efm_ck[dadr+2] = h*k;
        any = any || k > 0;
      }
      break;

    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      {
        mjtNum x = d->qpos[padr] - m->qpos_spring[padr];
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
      mjtNum lower = m->tendon_lengthspring[2*i];
      mjtNum upper = m->tendon_lengthspring[2*i+1];
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


  // fluid drag blocks: assemble the drag-only passive-fluid velocity derivative into the
  // assemble the drag derivatives into stack scratch in qDeriv's shape (qDeriv itself is
  // user-facing), gather into M's sparsity pattern, scale by -h. Drag derivatives are
  // symmetric dissipative by construction; lift and added-mass terms are excluded and
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
  if (d->efm_diag) {
    mju_copy(d->qH, d->M, m->nC);
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
    // fluid drag blocks share M's sparsity: add the full pattern
    if (d->efm_fluid) {
      mju_addTo(d->qH, d->efm_fluid, m->nC);
    }

    // record the diagonal additions for metric-consistent regularization, then factor
    for (int i=0; i < nv; i++) {
      int diag = m->M_rowadr[i] + m->M_rownnz[i] - 1;
      d->efm_sdiag[i] = d->qH[diag] - d->M[diag];
    }
    if (flg_factor) {
      mj_factorI(d->qH, d->qHDiagInv, nv, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
    }
  }
}


// island-local metric product res += S*vec: the diagonal classes and the island's tendons,
// with vectors in island-local dof coordinates. Flex terms never reach the island path:
// models with flex metric terms force a monolithic solve
void mj_effMulAddIsland(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec,
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

  // pass 1: packed length, two header entries plus nnz per row
  for (int i = 0; i < d->ncon; i++) {
    const mjContact* con = d->contact + i;
    if (con->exclude != 4 || mjd_flexContactStiffness(m, d, con) <= 0) {
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
    if (con->exclude != 4) {
      continue;
    }
    mjtNum k = mjd_flexContactStiffness(m, d, con);
    if (k <= 0) {
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
  d->efm_diag = (any_diag || d->nefmT || any_act || any_fluid) ? EFMALLOC(mjtNum, nv) : NULL;
  d->efm_sdiag = d->efm_diag ? EFMALLOC(mjtNum, nv) : NULL;
  d->efm_fluid = any_fluid ? EFMALLOC(mjtNum, m->nC) : NULL;

  // assemble the standard-flex part of B into CSR (constant during the step). With stretch or
  // assemblable interp present, assemble the FULL matrix (bending included): one CSR then
  // serves both the matvec and the per-step factor. Bending-only models keep the stencil
  // operator + the constant mj_setConst factor. The stiffness (s1) and damping (s2) parts enter
  // only when their forces are enabled; the structure does not depend on the flags
  mjtNum s1 = mjDISABLED(mjDSBL_SPRING) ? 0 : h*h;
  mjtNum s2 = mjDISABLED(mjDSBL_DAMPER) ? 0 : h;
  const mjtNum* krot = mjd_flexInterpAssemblable(m) ? d->flexelem_krot : NULL;
  d->efm_K_rownnz = EFMALLOC(int, nv);
  d->efm_K_rowadr = EFMALLOC(int, nv);

  // Newton consumes the metric as an explicit sparse matrix inside its Hessian: force full
  // CSR assembly, including bending-only flexes which otherwise keep the matrix-free stencil
  // plus the constant mj_setConst factor (non-assemblable interp flexes are rejected by
  // mj_checkDiscrete under Newton)
  int assemble_any = mjd_flexStiff_any(m, krot != NULL) || mjd_flexPassiveContact_any(m);
  if (m->opt.solver == mjSOL_NEWTON) {
    for (int f=0; !assemble_any && f < m->nflex; f++) {
      assemble_any = mjd_flexStiff_active(m, f, /*flg_bend=*/1, /*flg_stretch=*/1);
    }
  }
  if (assemble_any) {
    d->nefmK = mjd_flexStiff_assemble(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                                      NULL, NULL, s1, s2, /*bend*/ 1, /*stretch*/ 1, krot);
  }

  // passive flex contact is a rank-1 class, not CSR entries (see effContactBuild)
  if (passive) {
    effContactBuild(m, d, h*h);
  }
  if (d->nefmK) {
    d->efm_K_colind = EFMALLOC(int, d->nefmK);
    d->efm_K_val    = EFMALLOC(mjtNum, d->nefmK);
    mjd_flexStiff_assemble(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                           d->efm_K_colind, d->efm_K_val, s1, s2,
                           /*bend*/ 1, /*stretch*/ 1, krot);
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

  d->efm_active = 1;
}
