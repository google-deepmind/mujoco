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


// Build and factor the per-vertex 3x3 diagonal blocks of the flex part of (M + K), stored in
// d->efm_L, 9 numbers per covered vertex: O(n) to build and apply, approximate where the sparse
// factorization it replaces was exact. Both consumers use the blocks as a preconditioner: the CG
// constraint solver (Mgrad = Mtilde \ grad) and the qacc_smooth PCG in mj_effSolve, which
// supplies the accuracy.
static void effBlocks(const mjModel* m, mjData* d) {
  int nv = m->nv;

  // covered dofs come in contiguous triples (the 3 slide dofs of one flex point), but the first
  // one need not be at a multiple of 3: any joint declared before the flex shifts them. Walk the
  // covered rows rather than striding the dof index, which would straddle point boundaries.
  int nb = 0;
  for (int i = 0; i < nv; ) {
    if (d->efm_K_rownnz[i]) { nb++; i += 3; } else { i++; }
  }
  d->nefmdof = 0;
  mjtNum* B = (mjtNum*) effAlloc(d, sizeof(mjtNum)*9*(nb > 0 ? nb : 1), _Alignof(mjtNum));
  int* adr = (int*) effAlloc(d, sizeof(int)*(nb > 0 ? nb : 1), _Alignof(int));
  int k = 0;
  for (int i = 0; i < nv; ) {
    if (!d->efm_K_rownnz[i]) {
      i++;
      continue;
    }
    mjtNum* Bk = B + 9*k;
    effBlockRaw(m, d, i, Bk);
    mju_cholFactor(Bk, 3, mjMINVAL);
    adr[k++] = i;
    i += 3;
  }
  d->efm_L = B;
  d->efm_dofid = adr;
  d->nefmdof = nb;
  d->nefmL = 9*nb;
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

// fold: the solver's folded copy (mj_effPrecFold) with L its blocks, or NULL
static void effBlockApply(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b,
                          const mjtNum* L, const mjEffFold* fold) {
  int nv = m->nv;
  int nbd = m->nefm0dof;
  int flg_bend = nbd && !d->nefmdof;
  int flg_dense = fold && fold->S_valid;

  // the fold's dense factors on all the uncovered dofs, the 3x3 blocks on the rest: no backbone
  if (flg_dense && !fold->partial) {
    mj_markStack(d);
    mjtNum* bu = mjSTACKALLOC(d, fold->nu, mjtNum);
    for (int j = 0; j < fold->nu; j++) {
      bu[j] = b[fold->U[j]];   // before the blocks write x: b may alias x
    }
    for (int k = 0; k < d->nefmdof; k++) {
      int i = d->efm_dofid[k];
      chol3Solve(x + i, L + 9*k, b + i);
    }
    effDenseApply(fold, x, bu);
    mj_freeStack(d);
    return;
  }

  // every dof a covered triple: the blocks are the whole preconditioner
  if (3*d->nefmdof == nv && !flg_bend) {
    for (int k = 0; k < d->nefmdof; k++) {
      int i = d->efm_dofid[k];
      chol3Solve(x + i, L + 9*k, b + i);
    }
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

  // per-step stiffness: 3x3 blocks
  for (int k = 0; k < d->nefmdof; k++) {
    int i = d->efm_dofid[k];
    chol3Solve(x + i, L + 9*k, rhs + i);
  }

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


// mark the covered dofs: cov[i] = the 3x3 block of dof i, -1: uncovered
static void effCovered(const mjModel* m, const mjData* d, int* cov) {
  for (int i=0; i < m->nv; i++) {
    cov[i] = -1;
  }
  for (int k=0; k < d->nefmdof; k++) {
    for (int c=0; c < 3; c++) {
      cov[d->efm_dofid[k] + c] = k;
    }
  }
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
  effCovered(m, d, cov);
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
  size_t apply = frame + mj_stackBytes(sn*m->nv, an) + mj_stackBytes(sn*nu, an);
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
  effCovered(m, d, blk);

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
