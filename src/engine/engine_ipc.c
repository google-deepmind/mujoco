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

// This integrator implements barrier-free augmented-Lagrangian (AL) penetration-free contact
// (Li et al., arXiv:2512.12151): the log-barrier is replaced by an augmented Lagrangian
// with a per-pair multiplier + active-set, and intersection-freedom is maintained by advancing
// a CCD-bounded committed position. Flex-only: 2D flex nodal IPC (the rigid/affine path was
// removed).

#include "engine/engine_collision_continuous.h"
#include "engine/engine_ipc.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>  // qsort (sparse Hessian pattern build)

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include "engine/engine_forward.h"           // mj_Euler (fallback)
#include "engine/engine_collision_driver.h"  // mj_collision (re-run to source movable-rigid contacts)
#include "engine/engine_core_constraint.h"  // mj_makeConstraint / mj_referenceConstraint (reuse efc_aref/efc_R)
#include "engine/engine_support.h"  // mj_mulM, mj_integratePos, mj_differentiatePos (per-tree mass + manifold)
#include "engine/engine_derivative.h"  // effective-metric operators; mjExtraStiff
#include "engine/engine_core_smooth.h"  // mj_solveM, mj_kinematics, mj_comPos (per-tree M^-1, FK at trial q)
#include "engine/engine_core_util.h"  // mj_local2Global (anchor body-local -> world for the live gap)
#include "engine/engine_util_solve.h"    // mju_cholFactor, mju_cholSolve (dense Newton solve)
#include "engine/engine_util_blas.h"     // mju_dot3, mju_mulMatVec3
#include "engine/engine_util_spatial.h"  // mju_cross (flex bending)
#include "engine/engine_util_errmem.h"   // mju_malloc, mju_free
#include "engine/engine_memory.h"  // mj_markStack / mj_freeStack / mj_stackAlloc* (per-step scratch)







#define IPC_CDAMP_FLEX 0.1  // flex-contact normal dashpot coeff (cde = IPC_CDAMP_FLEX*m/h^2)
// ---- augmented-Lagrangian (AL) solve (pure-flex path) parameters ----
#define IPC_MU_SCALE 5e7  // per-vertex AL stiffness: mu_v = mass*IPC_MU_SCALE*h^2
#define IPC_ASET_AGE \
  25  // active-set eviction: drop a never-engaged pair after this many quiet updates
#define IPC_DECAY 0.3     // cnt-aging stiffness decay: scale = pow(IPC_DECAY,c)*mu
#define IPC_VEL_TOL \
  0.05  // newton velocity tolerance (m/s); abs dx tol = IPC_VEL_TOL*h (L-inf dx checker)
#define IPC_ALPHA_LB 1e-6  // advance xfree only if CCD alpha > this (alpha lower bound)
#define IPC_STALL_MAX \
  64  // consecutive outer iters with CCD alpha <= IPC_ALPHA_LB (no feasible advance) -> solve is
      // frozen
#define IPC_PCG_MAXITER \
  4000  // PCG hard cap (keep iterating to here; only reached on a badly ill-conditioned Hessian)
#define IPC_PCG_WARN \
  200  // WARN (but do NOT stop) once PCG needs more than this -> ill-conditioned, direction getting
       // costly
#define IPC_FLEX_MIN_ITER \
  1  // minimum Newton iterations. Terminate once beta is feasible
     // (beta ~ 1, complete advance) AND the articulated block converged AND the flex block
     // converged-or-full-step (contact-type-aware: rigid gcon contact needs convergence, AL
     // flex contact is stable at the alpha==1 bail). The persistent active set + per-element
     // PSD projection make the single-Newton flex step sound.
#define IPC_ASET_TOI \
  1e-6  // active-set update: admit a new broad-phase pair iff its CCD toi < 1 - IPC_ASET_TOI





// per-pair AL stiffness mu = min over the pair's flex vertices of mu_v.
// FORCE-FORM (this is the crux): the reference incremental-potential objective is K + h^2*Psi with
// K = 0.5*M*(x-xtil)^2 (RAW mass), so its mu_v = mass*IPC_MU_SCALE*h^2 gives mu/inertia =
// IPC_MU_SCALE*h^2 = 200 (a strong penalty). OUR objective is that IP divided by h^2 -> inertia =
// mass/h^2, elastic force-form, so the FORCE-FORM mu is the IP mu divided by h^2 =
// mass*IPC_MU_SCALE (the h^2 cancels). Returning the IP-form mu*h^2 verbatim (the old bug) made the
// penalty h^4 = 1250x too SOFT vs our inertia -> contacts couldn't hold (the CCD did all the work
// -> FROZEN) and the dual-ascent step lam-=craw*mu was 1250x too small (-> SLOW). mu =
// IPC_MU_SCALE*mass restores mu/inertia = 200, consistent with the dashpot in the same block (cde =
// IPC_CDAMP_FLEX*mass*ih2, also force-form). REPLACES scalar kappa.
static mjtNum ipc_muPair(const ipcCon* con, const mjtNum* mass, mjtNum ih2) {
  (void)ih2;  // force-form mu has no explicit h factor (the h^2 cancelled against the IP-form
              // objective)
  int vv[4], nvv;
  mjc_conVerts(con, vv, &nvv);
  // min over NONZERO masses: MuJoCo flex verts pinned to a rigid attachment carry mass 0 (their
  // inertia is in the rigid body); a plain min hits those -> mu=0 -> lam/mu = 0/0 = NaN. The
  // reference solver's flex verts all have mass; MuJoCo guard.
  mjtNum mmin = 1e30;
  for (int q=0; q < nvv; q++) {
    mjtNum mv = mass[vv[q]];
    if (mv > 0 && mv < mmin) mmin = mv;
  }
  if (mmin >= 1e29)
    mmin = 1e-9;  // all involved verts massless (degenerate) -> tiny mu, no division blow-up
  return IPC_MU_SCALE * mmin;
}

// cnt -> decay exponent c (AL normal-contact aging): c = cnt>=0 ? cnt : max(-cnt-6, 0).
static inline int ipc_cntExp(int cnt) { return cnt >= 0 ? cnt : (-cnt - 6 > 0 ? -cnt - 6 : 0); }

// stable per-pair hash for the persistent cnt store: contact type + sorted vertex/feature indices.
static unsigned long ipc_pairHash(const ipcCon* con) {
  int id[4];
  for (int k=0; k < 4; k++)
    id[k] = con->idx[k];
  for (int i=0; i < 3; i++)
    for (int j=0; j < 3 - i; j++)  // sort idx ascending (order-independent key)
      if (id[j] > id[j + 1]) {
        int t = id[j];
        id[j] = id[j + 1];
        id[j + 1] = t;
      }
  unsigned long hh = (unsigned long)con->type * 1000003ul + (unsigned long)(con->gi + 1);
  for (int k=0; k < 4; k++)
    hh = hh * 1000003ul + (unsigned long)(id[k] + 1);
  return hh;
}


// rigorous additive CCD (Li et al.): largest alpha in [0,1] s.t. advancing x by alpha*dxw keeps
// every candidate's surface gap above 20% of its current value -- conservative advancement, no
// tunneling. For self pairs the common (mean) displacement is removed so coherent motion (free
// fall) isn't throttled; geom features are fixed so only the flex side's speed bounds the
// gap-shrink rate. IMPORTANT: mean-removal is only valid when the pair can genuinely move
// together, which is why mjc_advance gates it on same-flex. IPC-specific: Li-et-al additive
// conservative-advancement TOI -- NOT mjc_ccd (GJK/EPA per-pair distance, despite the shared
// name); the engine has no time-of-impact routine.

// [P1c] The FEM machinery (ipcElem membrane elements, ipcBend flaps, ipcEdge soft rows) is gone:
// stretch is carried by the native edge-equality efc rows in the inner QP, and stretch+bending are
// carried by the native edge-equality efc rows; this solver has no elastic energy of its
// own (FEM elasticity is rejected at entry -- the CG integrators' effective metric owns it).

// slack update (loop step N2, at the optimizer x): materialize the AL slack s = max(0, c_raw -
// lam/mu) for every candidate so the subsequent assemble/energy (ipc_energy) use the
// exact slack-baked d = c_raw - s - lam/mu, and the lambda update (ipc_flexLamUpdate) can un-bake
// it. c_raw is the LINEARIZED gap at x (ld0 set this iter by linearize at xfree). Order is FIXED:
// linearize -> slack update -> assemble -> ... -> lambda.
static void ipc_updateSlack(ipcCon* cand, int ncand, const mjtNum* x, const mjtNum* xfree,
                            const mjtNum* rad, mjtNum ghat, const mjtNum* mass, mjtNum ih2) {
  for (int c=0; c < ncand; c++) {
    ipcCon* con = &cand[c];
    mjtNum mu = ipc_muPair(con, mass, ih2);
    mjtNum craw = con->ld0 - mjc_off(mjc_conGhat(
                                 con, rad, ghat));  // c_raw(x) = (ld0-delta) + d_grad.(x-xfree)
    for (int p=0; p < con->lniv; p++) {
      int v = con->liv[p];
      for (int k=0; k < 3; k++)
        craw += con->lcw[p] * con->ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    mjtNum t = craw - con->lam / mu;
    con->s =
        (t > 0)
            ? t
            : 0;  // s = max(0, c_raw - lam/mu); baked d = c_raw - s - lam/mu (in the assemble/energy)
  }
}

// AL contact multiplier per flex vertex: the cross-step warm-start
// store for the active-set contacts, whose LIVE multiplier rides in ipcCon.lam. Seeded into cand[].lam
// at step start, sunk back after the step (binding = max over a contact's free-point participants).
// npt-sized.

// Contact age, per FLEX VERTEX (d->flexvert_conage). A vertex counts as loaded on an update iff
// some pair touching it ended with a positive multiplier; the age then runs the same two-branch
// machine the per-pair counter used to (<0 loaded recently, >0 steps since, magnitude = consecutive
// quiet updates), and a pair reads the MIN over its vertices. The age drives only the penalty decay
// (ipc_cntExp); eviction is decided separately and statelessly in ipc_mergeActiveSet.
//
// Both halves must key on the SAME signal -- load at a vertex. Mixing them (age-driven decay with
// load-driven eviction, or the reverse) drops pairs at an arbitrary point on their decay ramp
// instead of at the bottom of it, and every such drop is a discontinuity the solve has to absorb.
//
// Keying on the vertex rather than on pair identity also keeps symmetric configurations symmetric:
// a pair's identity is the closest-feature pair, an arbitrary discrete label that mirror-image
// contacts need not share, whereas load at a vertex is a physical quantity.

// pair age = min over its flex vertices; 0 when none of them carries an age
static int ipc_conAge(const ipcCon* con, const int* age, const int* pt2vg) {
  int vv[4], nvv, a = INT_MAX;
  mjc_conVerts(con, vv, &nvv);
  for (int q=0; q < nvv; q++) {
    int g = age[pt2vg[vv[q]]];
    if (g < a) a = g;
  }
  return a == INT_MAX ? 0 : a;
}

// advance the per-vertex age one update: loaded iff some active pair touches the vertex
static void ipc_ageStep(int* age, const ipcCon* aset, int naset, const int* pt2vg, int npt,
                        char* loaded) {
  for (int i=0; i < npt; i++) loaded[i] = 0;
  for (int c=0; c < naset; c++) {
    if (aset[c].lam <= 0) continue;
    int vv[4], nvv;
    mjc_conVerts(&aset[c], vv, &nvv);
    for (int q=0; q < nvv; q++) loaded[vv[q]] = 1;
  }
  for (int i=0; i < npt; i++) {
    int g = pt2vg[i];
    if (loaded[i]) age[g] = (age[g] == 0 || age[g] > 5) ? 0 : -1;
    else           age[g] += (age[g] >= 0) ? 1 : -1;
  }
}

// lambda update + cnt state machine (loop step N8a, at the optimizer x): for every held candidate,
// un-bake d to the raw linearized gap c_raw, then the two-branch AL dual update (the slack
// condition c_raw>lam/mu) with the cnt aging machine (using the lambda value from BEFORE this
// update):
//   if (c_raw - lam/mu > 0)  { lam = 0;  cnt += (cnt>=0 ? +1 : -1); }                  // inactive
//   (slack>0) else                     { lam -= c_raw*mu;  cnt = (cnt==0 || cnt>5) ? 0 : -1; }   //
//   active: lam grows
// (NO max(0,.) clamp.) Then SINK lam into d->flexvert_lambda for the next step's warm start.
// cnt rides in cand[c].cnt and is persisted per-pair by the caller.
static void ipc_flexLamUpdate(const mjModel* m, const mjData* d, const mjtNum* x,
                              const mjtNum* xfree, const mjtNum* gv, const mjtNum* ge,
                              const mjtNum* rad, mjtNum ghat, const mjtNum* mass, mjtNum ih2,
                              ipcCon* cand, int ncand, mjtNum* pal, const int* pt2vg, int npt) {
  for (int c=0; c < ncand; c++) {
    ipcCon* con = &cand[c];
    mjtNum mu = ipc_muPair(con, mass, ih2);
    mjtNum lam0 = con->lam;  // lambda BEFORE this update (the dual update reads pre-update)
    mjtNum craw = con->ld0 - mjc_off(mjc_conGhat(
                                 con, rad, ghat));  // c_raw(x) (un-baked: == baked_d + s + lam/mu)
    for (int p=0; p < con->lniv; p++) {
      int v = con->liv[p];
      for (int k=0; k < 3; k++)
        craw += con->lcw[p] * con->ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    if (craw - lam0 / mu > 0) {  // inactive (slack s>0): multiplier off, age the inactive counter
      con->lam = 0;
      con->cnt += (con->cnt >= 0) ? 1 : -1;
    } else {  // active: ascend lam by c_raw*mu (grows since c_raw<lam/mu)
      con->lam = lam0 - craw * mu;
      con->cnt = (con->cnt == 0 || con->cnt > 5) ? 0 : -1;
    }
  }
  for (int p=0; p < npt; p++)
    pal[pt2vg[p]] = 0;  // sink: per-flex-vertex binding multiplier (warm-start src)
  for (int c=0; c < ncand; c++) {
    if (cand[c].lam <= 0) continue;
    int vv[4], nvv;
    mjc_conVerts(&cand[c], vv, &nvv);
    for (int q=0; q < nvv; q++)
      if (cand[c].lam > pal[pt2vg[vv[q]]]) pal[pt2vg[vv[q]]] = cand[c].lam;
  }
}

// active-set update (MERGE/AGING of the persistent active set).
// Inputs: the existing persistent set aset[0..*naset) (cnt rides in each ipcCon), and the fresh
// broad-phase candidates cand[0..ncand) with a per-candidate "closing this step" admission flag
// cadmit[c] (== the CCD time-of-impact < 1-1e-6). Rule:
//   - KEEP an existing aset pair iff it still carries load (lam > 0), else evict.
//   - ADD a new broad-phase candidate iff cadmit[c] AND its pairHash is not already present (dedup
//   by 64-bit-ish
//     pairHash). New entries seed cnt=0, lam=0; we re-hydrate cnt from the cross-step store and lam
//     from d->flexvert_lambda so the cnt aging and AL multiplier survive across steps
//     (our persistence path).
// The merged set is written back into aset/*naset. The new merged set's fields (ld0/ln/.../s) are
// refreshed by the next iter's linearize_constraints + update_slack, so only type/idx/gi/lam/cnt
// need to be carried here.
static void ipc_mergeActiveSet(mjData* d, ipcCon* aset, int* naset, const ipcCon* cand, int ncand,
                               const int* cadmit, ipcCon* amerge, const mjtNum* pal,
                               const int* conage, const int* pt2vg, int candmax) {
  // dedup hash over the merged set: open addressing keyed by pairHash (key 0 = empty slot; a true
  // hash of 0 is astronomically unlikely and at worst causes one duplicate, harmless).
  // Presence-only -> no value array needed. Zeroed on entry and never read across calls, so it is
  // per-call scratch: it lives on the mjData stack, not in a file-scope cache.
  int cap = 1;
  while (cap < 4 * (*naset + ncand) + 16)
    cap <<= 1;
  mj_markStack(d);
  unsigned long* mkey =
      (unsigned long*)mj_stackAllocByte(d, (size_t)cap * sizeof(unsigned long), sizeof(mjtNum));
  unsigned long mask = (unsigned long)(cap - 1);
  for (int i=0; i < cap; i++)
    mkey[i] = 0;
  int nm = 0;
  // 1) keep existing pairs that still carry load (stateless eviction).
  for (int c=0; c < *naset; c++) {
    // EVICTION. The age's SIGN says whether this pair has ever carried load: cnt < 0 means one of
    // its vertices has, cnt >= 0 means none has yet. Only the first group may be dropped for going
    // quiet. A pair that has never engaged is still climbing its multiplier off zero over the
    // outer iterations, and lam <= 0 cannot tell that state from "finished" -- testing it alone
    // culls approaching pairs before they develop any force, and the geometry passes through. The
    // never-engaged group still ages out on its own counter so it cannot accumulate.
    if ((aset[c].lam <= 0 && aset[c].cnt < 0) || aset[c].cnt > IPC_ASET_AGE) continue;
    unsigned long key = ipc_pairHash(&aset[c]), h = key & mask;
    while (mkey[h] != 0) {
      if (mkey[h] == key) break;
      h = (h + 1) & mask;
    }
    if (mkey[h] == key)
      continue;  // already present (shouldn't happen within the existing set, but safe)
    mkey[h] = key;
    amerge[nm++] = aset[c];
  }
  // 2) add new admitted broad-phase candidates not already present
  for (int c=0; c < ncand; c++) {
    if (!cadmit[c] || nm >= candmax) continue;
    unsigned long key = ipc_pairHash(&cand[c]), h = key & mask;
    while (mkey[h] != 0) {
      if (mkey[h] == key) break;
      h = (h + 1) & mask;
    }
    if (mkey[h] == key) continue;  // dedup: pair already in the merged set
    ipcCon con = cand[c];
    int vv[4], nvv;
    mjc_conVerts(&con, vv, &nvv);  // warm-start lam from the per-point binding multiplier
    mjtNum s = 0;
    for (int q=0; q < nvv; q++)
      if (pal[pt2vg[vv[q]]] > s) s = pal[pt2vg[vv[q]]];
    con.lam = s;
    con.cnt = ipc_conAge(&con, conage, pt2vg);
    con.s = 0;  // re-hydrate cnt across steps
    mkey[h] = key;
    amerge[nm++] = con;
  }
  for (int c=0; c < nm; c++)
    aset[c] = amerge[c];
  *naset = nm;
  mj_freeStack(d);
}


// IPC incremental-potential energy: inertia + edge-stretch penalty + AL contact merit over the
// the ACTIVE SET -- the same set the injected QP rows are built from (see the ONE SET note)
static mjtNum ipc_energy(const mjModel* m, const mjData* d, int nfv, const mjtNum* x,
                         const mjtNum* xtil, const int* fidx, const mjtNum* mass, mjtNum h,
                         const mjtNum* rad, mjtNum ghat, const mjtNum* gv,
                         const mjtNum* ge, const ipcCon* acon, int nacon, const mjtNum* xold,
                         const mjtNum* xfree) {
  mjtNum E = 0, ih2 = 1.0 / (h * h);
  for (int v=0; v < nfv; v++)
    if (fidx[v] >= 0) {
      mjtNum mh = mass[v] * ih2;
      for (int c=0; c < 3; c++) {
        mjtNum t = x[3 * v + c] - xtil[3 * v + c];
        E += 0.5 * mh * t * t;
      }
    }
  for (int c=0; c < nacon; c++) {
    // AL penalty energy E = 0.5*scale*d^2 over the slack-baked d: exact parity with
    // the injected row's gradient. d = c_raw - s - lam/mu, scale = pow(IPC_DECAY, cnt-exp)*mu_pair. Keep
    // mj's dashpot energy.
    mjtNum mu = ipc_muPair(&acon[c], mass, ih2);
    mjtNum craw = acon[c].ld0 - mjc_off(mjc_conGhat(&acon[c], rad, ghat));
    for (int p=0; p < acon[c].lniv; p++) {
      int v = acon[c].liv[p];
      for (int k=0; k < 3; k++)
        craw += acon[c].lcw[p] * acon[c].ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    mjtNum dd = craw - acon[c].s - acon[c].lam / mu;
    int cexp = ipc_cntExp(acon[c].cnt);
    mjtNum scale = mu;
    for (int e=0; e < cexp; e++)
      scale *= IPC_DECAY;
    E += 0.5 * scale * dd * dd;  // AL equality penalty (two-sided; slack carries the inequality)
    mjtNum dn = 0;               // matching one-sided dashpot energy (see the injected rows)
    for (int p=0; p < acon[c].lniv; p++) {
      int v = acon[c].liv[p];
      for (int k=0; k < 3; k++)
        dn += acon[c].lcw[p] * acon[c].ln[k] * (x[3 * v + k] - xold[3 * v + k]);
    }
    if (dd <= 0 && dn < 0) {
      mjtNum mmin = 1e30;
      for (int p=0; p < acon[c].lniv; p++) {
        mjtNum mv = mass[acon[c].liv[p]];
        if (mv < mmin) mmin = mv;
      }
      E += 0.5 * (IPC_CDAMP_FLEX * mmin / (h * h)) * dn * dn;
    }
  }
  return E;
}

// [ARCH-2] merit efc cost = EXACTLY what MuJoCo's QP minimizes over the efc rows (edge equalities,
// limits, friction, rigid-rigid and rigid-flex contacts -- all constraint states, impedances and
// cones), evaluated at the trial state via mj_constraintUpdate. Replaces the hand-replicated
// scorers (ipc_softEnergy/ipc_edgeEnergy/ ipc_gconEnergy): any replica divergence from the QP
// objective turns the QP's exact direction into a merit-ascent (the A!=merit bug class). The trial
// state maps to the QP variable by the same substitution the rows use: qacc = (v_new - v_old)/h,
// v_new = dp_local/h (flex slide dofs, world->local via R^T) and v_new = qdelta/h (appended
// articulated trees); untouched dofs stay at qacc_smooth (free flight).
// trial state -> the QP variable: qacc = (v_new - v_old)/h, with v_new = dp_local/h on the flex
// slide dofs (world->local via R^T) and qdelta/h on the appended articulated trees; dofs the step
// does not touch stay at qacc_smooth. Shared so every merit term prices the SAME point.
static void ipc_trialQacc(const mjModel* m, const mjData* d, mjtNum* qacc, const mjtNum* xt,
                          const mjtNum* xold, int npt, const int* fidx, const int* dofadr,
                          const int* pbody, const mjtNum* qdt, int na_artic, const int* atid,
                          const int* aoff, int N, mjtNum h) {
  mju_copy(qacc, d->qacc_smooth, m->nv);
  for (int v=0; v < npt; v++)
    if (fidx[v] >= 0) {
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum dpw[3], dpl[3];
      for (int c=0; c < 3; c++)
        dpw[c] = xt[3 * v + c] - xold[3 * v + c];
      mju_mulMatTVec3(dpl, R, dpw);
      int da = dofadr[v];
      for (int c=0; c < 3; c++)
        qacc[da + c] = (dpl[c] / h - d->qvel[da + c]) / h;
    }
  for (int a=0; a < na_artic; a++) {
    int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
    for (int i=0; i < nd; i++)
      qacc[da + i] = (qdt[o + i - N] / h - d->qvel[da + i]) / h;
  }
}


// Implicit-elasticity half of the Gauss term. The solver minimizes
// 0.5*(a-a*)^T Mtilde (a-a*) with Mtilde = M + K; the inertia terms above (ipc_energy's flex mass,
// ipc_articResid's tree mass) supply the M half, so this adds the K half and completes the metric.
// Without it the merit prices a different objective than the inner solve minimizes and the line
// search fights the QP direction. Zero unless the effective metric is active (edge-equality flexes
// carry their elasticity in efc rows, which ipc_efcCost already scores). h^2 puts it in the
// position-space units every other merit term uses.
static mjtNum ipc_metricCost(const mjModel* m, mjData* d, const mjtNum* xt, const mjtNum* xold,
                             int npt, const int* fidx, const int* dofadr, const int* pbody,
                             const mjtNum* qdt, int na_artic, const int* atid, const int* aoff,
                             int N, mjtNum h) {
  if (!d->efm_active) return 0;
  int nv = m->nv;
  mj_markStack(d);
  mjtNum* da = mj_stackAllocNum(d, nv);
  mjtNum* Kda = mj_stackAllocNum(d, nv);
  ipc_trialQacc(m, d, da, xt, xold, npt, fidx, dofadr, pbody, qdt, na_artic, atid, aoff, N, h);
  mju_subFrom(da, d->qacc_smooth, nv);  // a - a*
  mju_zero(Kda, nv);
  mjd_effMulAdd(m, d, Kda, da);  // K*(a-a*): the metric operator applies K, not M
  mjtNum E = 0.5 * h * h * mju_dot(da, Kda, nv);
  mj_freeStack(d);
  return E;
}


static mjtNum ipc_efcCost(const mjModel* m, mjData* d, const mjtNum* xt, const mjtNum* xold,
                          int npt, const int* fidx, const int* dofadr, const int* pbody,
                          const mjtNum* qdt, int na_artic, const int* atid, const int* aoff, int N,
                          mjtNum h, int nefc_qp) {
  int nv = m->nv,
      nefc = nefc_qp;  // score EXACTLY the rows the QP minimized (pure-flex trims native contacts)
  if (!nefc) return 0;
  mj_markStack(d);
  mjtNum* qacc = mj_stackAllocNum(d, nv);
  mjtNum* jar = mj_stackAllocNum(d, nefc);
  ipc_trialQacc(m, d, qacc, xt, xold, npt, fidx, dofadr, pbody, qdt, na_artic, atid, aoff, N, h);
  int savednefc = d->nefc;
  d->nefc = nefc;  // mj_mulJacVec/mj_constraintUpdate read d->nefc internally
  mj_mulJacVec(m, d, jar, qacc);
  mju_subFrom(jar, d->efc_aref, nefc);
  mjtNum cost = 0;
  mj_constraintUpdate(m, d, jar, &cost, 0);
  d->nefc = savednefc;
  mj_freeStack(d);
  return h * h * cost;  // the merit is the POSITION-space energy = h^2 * A (every other term is
                        // h^2-scaled: the verbatim edge replica is 0.5*h^2*efc_D*r^2, inertia is
                        // 0.5*m*ih2*dx^2) -- match it
}




// IPC-style variational integrator (integrator="ipc"): owns the full step, minimizing the per-step
// incremental potential with penetration-free contact by a barrier-free AUGMENTED-LAGRANGIAN method
// (paper arXiv 2512.12151) -- the contact multiplier carries the force at a fixed low stiffness (no
// log barrier, no kappa adaptation, no TOI-lock); the inner optimizer is MuJoCo's CG in the mass
// metric (stretch rides the native edge-equality efc rows; the solver carries no elastic
// energy) with the AL contact injected as extra-primal rows; the committed output is a
// conservative-CCD blend from the last intersection-free state. Covers flex self-contact
// (vertex-triangle + edge-edge) and flex-vs-geom.

// Soft-contact (k, gtarget) come from MuJoCo's per-contact efc reference via ipc_softCoef: gtarget
// = pos + h*vel + h^2*aref (efc_aref/efc_vel), k = imp/(1-imp)*ih2/invm (efc impedance + our EXACT
// contact-space mass). This replaced the old IPC_SOFTK magic number; refsafe + the impedance
// sigmoid + the solref format are baked into efc.

// APPENDED ARTICULATED kinetic residual for the flex-unify path: given the candidate generalized
// tangent qdc (length N_artic, indexed by appended solver offset), build q = q_n (+) qdc on the
// articulated trees, the inertial residual gr = q (-) q~ (filled over nv), and gMr = M*gr (frozen M
// at q_n). Returns the kinetic incremental potential 0.5/h^2 * gr^T M gr summed over the
// ARTICULATED dofs only (mirrors mj_ipcTree's term). The caller scatters ih2*gMr into the gradient.
// na_artic==0 -> returns 0 and touches nothing.
static mjtNum ipc_articResid(const mjModel* m, mjData* d, const mjtNum* qn, const mjtNum* qtil,
                             const mjtNum* qdc, int na_artic, const int* atid, const int* aoff,
                             int N_flex, mjtNum ih2, mjtNum* gdq, mjtNum* qcur, mjtNum* gr,
                             mjtNum* gMr) {
  if (!na_artic) return 0;
  for (int i=0; i < m->nv; i++)
    gdq[i] = 0;
  for (int a=0; a < na_artic; a++) {
    int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
    for (int i=0; i < nd; i++)
      gdq[da + i] = qdc[o + i - N_flex];
  }
  mju_copy(qcur, qn, m->nq);
  mj_integratePos(m, qcur, gdq, 1);             // q = q_n (+) qdc  (only the artic dofs move)
  mj_differentiatePos(m, gr, 1.0, qtil, qcur);  // gr = q (-) q~  (inertial residual)
  mj_mulM(m, d, gMr, gr);                       // gMr = M*gr (block-diagonal; artic blocks only)
  mjtNum hd = m->opt.timestep,
         ihd =
             1.0 /
             hd;  // IMPLICIT joint damping D=dof_damping: energy += 1/2 ih (q(-)q_n)^T D (q(-)q_n);
  mjtNum E = 0;   // gradient gMr += h*D*gdq so the caller's ih2*gMr = ih2*M*gr + ih*D*gdq
  for (int a=0; a < na_artic; a++) {
    int t = atid[a], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
    for (int i=0; i < nd; i++) {
      E += 0.5 * ih2 * gr[da + i] * gMr[da + i] +
           0.5 * ihd * m->dof_damping[da + i] * gdq[da + i] * gdq[da + i];
      gMr[da + i] += hd * m->dof_damping[da + i] * gdq[da + i];
    }
  }
  return E;
}

void mj_IPC(const mjModel* m, mjData* d) {
  mjtNum h = m->opt.timestep;
  // all dim-2 flexes participate in the IPC solve (was: only the first). Their vertices are
  // concatenated into the free-point array in flex order; fxadr[k] is the free-point offset of
  // dim-2 flex flist[k].
  int nfd = 0;
  for (int i=0; i < m->nflex; i++)
    if (m->flex_dim[i] == 2) nfd++;
  // nfd==0 (no 2D flex: pure rigid/articulated, e.g. the humanoid) falls through to the SAME
  // unified path: the appended articulated block carries the generalized tangent, and rigid contact
  // rides MuJoCo's native efc rows built just below.

  // SOURCE all movable-rigid-involved contacts from MuJoCo's OWN convex collision (geom-geom +
  // flex-vs-geom), snapshotted into a private buffer BEFORE any IPC scratch is allocated. The
  // forward's constraint stage was skipped for this integrator (ipcSkipConstraint in
  // engine_forward.c), so build the set here at q_n; the flex AL path never reads d->contact, so
  // this can't perturb it. mjDSBL_CONTACT / mjDSBL_CONSTRAINT are honoured rather than overridden:
  // mj_collision and mj_makeConstraint check them internally, so a model that disables either gets
  // an empty set here, same as under any other integrator. Ordering matters now that the scratch is
  // arena-backed: this efc build must happen BEFORE mj_markStack below, so its arena-side
  // allocations sit outside the per-step stack frame and stay live for the whole solve.
  // Mutually-exclusive filter: geom-geom (>=1 movable end) and flex-vs-MOVABLE-geom only; flex-flex
  // + flex-static stay on the AL path.
  // All per-step IPC scratch lives on the mjData stack from here to the matching mj_freeStack at
  // the end of this function. Marked AFTER the efc build above: those allocations come off the
  // arena end (d->parena) and must stay live for the whole solve, so they are not part of this
  // frame. The cnt map stays on the heap -- it survives
  // across steps and cannot live in a per-step frame.
  mj_markStack(d);
  int* flist = mj_stackAllocInt(d, nfd);  // the dim-2 flex ids
  int* fxadr = mj_stackAllocInt(d, nfd);  // free-point offset of each dim-2 flex
  int nfv = 0;                                       // total dim-2 flex verts (all flexes)
  for (int i=0, k = 0; i < m->nflex; i++)
    if (m->flex_dim[i] == 2) {
      flist[k] = i;
      fxadr[k] = nfv;
      nfv += m->flex_vertnum[i];
      k++;
    }
  // Detection band, model-independent (see IPC_GHAT). Per-pair narrowing still happens downstream:
  // mjc_conGhat mins it against the pair's own vertex radii, and mjc_candidates' bands min it
  // against the thinner participant.
  mjtNum ghat = IPC_GHAT;
  // free-point -> global flex-vertex index (into flex_vertbodyid / flexvert_xpos), and -> dim-2
  // flex slot k
  int* pt2vg = mj_stackAllocInt(d, (nfv > 0 ? nfv : 1));
  int* pt2flex = mj_stackAllocInt(d, (nfv > 0 ? nfv : 1));
  for (int k=0; k < nfd; k++) {
    int va_k = m->flex_vertadr[flist[k]], nv_k = m->flex_vertnum[flist[k]];
    for (int lv=0; lv < nv_k; lv++) {
      pt2vg[fxadr[k] + lv] = va_k + lv;
      pt2flex[fxadr[k] + lv] = k;
    }
  }

  // FLEX-ONLY point array. The point SoA holds ONLY flex vertices now: npt == nfv. Standalone
  // 3-slide bodies carrying a sphere geom are ordinary appended articulated trees (generalized
  // coords), not free points; the flex fidx assignment (0..nfree_flex-1) and the flex solver
  // arithmetic are the dense flex packing.
  char* isflexvert = (char*)mj_stackAllocByte(d, (m->nbody > 0 ? m->nbody : 1), 1);
  for (int b=0; b < m->nbody; b++)
    isflexvert[b] = 0;
  for (int v=0; v < nfv; v++)
    isflexvert[m->flex_vertbodyid[pt2vg[v]]] = 1;
  int npt = nfv;  // point array is flex-only
  int* dofadr = mj_stackAllocInt(d, (npt > 0 ? npt : 1));
  int* qpadr = mj_stackAllocInt(d, (npt > 0 ? npt : 1));  // qpos address (NOT dof address: differs after
  int* fidx = mj_stackAllocInt(d, (npt > 0 ? npt : 1));  // free/ball joints, which have more qpos than dof)
  mjtNum* mass = mj_stackAllocNum(d, (npt > 0 ? npt : 1));
  mjtNum* rad =
      mj_stackAllocNum(d, (npt > 0 ? npt : 1));  // per-point radius (flex_radius)
  int* pbody = mj_stackAllocInt(d, (npt > 0 ? npt : 1));  // body id, for the slide-frame rotation R
  int nfree = 0;
  for (int k=0; k < nfd; k++) {  // flex vertices, per dim-2 flex
    int fi = flist[k];
    mjtNum rk = m->flex_radius[fi];
    int va_k = m->flex_vertadr[fi], nv_k = m->flex_vertnum[fi];
    for (int lv=0; lv < nv_k; lv++) {
      int v = fxadr[k] + lv, bid = m->flex_vertbodyid[va_k + lv];
      dofadr[v] = -1;
      qpadr[v] = -1;
      fidx[v] = -1;
      mass[v] = 0;
      rad[v] = rk;
      pbody[v] = bid;
      if (m->body_dofnum[bid] == 3) {
        int da = m->body_dofadr[bid];
        dofadr[v] = da;
        qpadr[v] = m->jnt_qposadr[m->body_jntadr[bid]];
        fidx[v] = nfree++;
        mass[v] = d->M[m->M_rowadr[da] + m->M_rownnz[da] - 1];  // diagonal (point mass)
      }
    }
  }
  int nfree_flex = nfree;  // flex free-DOF count
  // STEP 4a: articulated trees (free root + hinges) the flex path historically ignored. Identified
  // by COMPLEMENT of the flex-vertex + slide-sphere bodies; each such tree's nv DOFs are APPENDED
  // to the solver vector after the flex/sphere packing at offset aoff[tree]. na_artic==0 ->
  // N_total==N (== N_flex) and every appended path is dead -> byte-identical to the pre-4a flex
  // solve. bodies the flex packing already carries: flex vertices (isflexvert). Anything else with
  // DOFs is an ARTICULATED tree to append.
  int ntree = m->ntree, na_artic = 0;
  char* isartictree = (char*)mj_stackAllocByte(d, (ntree > 0 ? ntree : 1), 1);
  for (int t=0; t < ntree; t++)
    isartictree[t] = 0;
  for (int b=1; b < m->nbody; b++) {
    if (m->body_dofnum[b] == 0 || isflexvert[b]) continue;
    isartictree[m->dof_treeid[m->body_dofadr[b]]] = 1;
  }
  int N = 3 * nfree_flex, N_artic = 0;  // N == N_flex: the flex dense packing
  int* aoff = mj_stackAllocInt(d, (ntree > 0 ? ntree : 1));
  int* atid =
      mj_stackAllocInt(d, (ntree > 0 ? ntree : 1));  // ids of the articulated trees
  for (int t=0; t < ntree; t++) {
    if (isartictree[t]) {
      aoff[t] = N + N_artic;
      atid[na_artic++] = t;
      N_artic += m->tree_dofnum[t];
    } else
      aoff[t] = -1;
  }
  int N_total = N + N_artic,
      Na = (N_total > 0 ? N_total : 1);  // Na sizes the full (flex+appended) solver vector
  // global dof -> appended solver slot (aoff[tree]+localdof), or -1 if its tree is not appended
  // (flex/slide-sphere/ static). Scatters the rigid-rigid contact's full-nv b into grad/p/Hp; only
  // ever indexed where b[i]!=0 (=> >=0). (slider-only simple-body diagonal-M fast solve DEFERRED:
  // body_mass IS the diagonal, but the fast branch diverged from mj_mulM in the flex+balls scene --
  // unexplained; the full mj_mulM/mj_solveM is correct.)
  int nstate = nfv;  // state-vector length (flex points)
  // No FEM element / edge-penalty / bending-flap arrays: stretch rides the native
  // edge-equality efc rows.
  int candmax = npt * 192 + 8192;  // capacity of the per-step candidate list (sized for the
                                   // geom-feature-heavy bag-in-bin contact: ~160k at npt~1100)
  ipcCon* cand = (ipcCon*)mj_stackAllocByte(d, (candmax) * sizeof(ipcCon), sizeof(mjtNum));
  mjtNum* cgap =
      mj_stackAllocNum(d, candmax);  // per-candidate gap at x (try->ccd/E0)
  // FLEX persistent active set (the persistent active-set manager): maintained ACROSS outer
  // iterations. Each iter: keep existing pairs that carry load, ADD new broad-phase
  // candidates with CCD toi<1-1e-6, dedup by pairHash. The assembled set (the injected QP rows
  // AND ipc_energy) is THIS persistent set -- bounded by the aging eviction, NOT the per-iter
  // broad-phase.
  ipcCon* aset = (ipcCon*)mj_stackAllocByte(d, (candmax) * sizeof(ipcCon), sizeof(mjtNum));  // persistent active pairs
  ipcCon* amerge =
      (ipcCon*)mj_stackAllocByte(d, (candmax) * sizeof(ipcCon), sizeof(mjtNum));  // merge scratch (new merged set built here)
  int naset = 0;
  // Per-candidate active-set scratch: appr is filled by mjc_advance (the pair closes this step),
  // actc is the admission mask handed to ipc_mergeActiveSet. The old per-point gamma carrier is
  // gone -- decay rides on the per-VERTEX contact age (d->flexvert_conage), live cnt in ipcCon.cnt.
  int* actc = mj_stackAllocInt(d, candmax);
  int* appr = mj_stackAllocInt(d, candmax);
  for (int c=0; c < candmax; c++) {
    actc[c] = 0;
    appr[c] = 0;
  }
  // precompute static-geom sharp features (vertices/edges) once per step (geoms are fixed here).
  // box -> 8 verts/12 edges; mesh -> all verts / all hull-poly edges. Cap = sum over colliding
  // geoms.
  int gvcap = 1, gecap = 1;
  for (int gi=0; gi < m->ngeom; gi++) {
    if (m->geom_contype[gi] == 0 && m->geom_conaffinity[gi] == 0) continue;  // skip non-colliding
    if (m->body_weldid[m->geom_bodyid[gi]] != 0)
      continue;  // STATIC geoms only (matches the type-2/3/4 barrier filter)
    int type = m->geom_type[gi];
    if (type == mjGEOM_BOX) {
      gvcap += 8;
      gecap += 12;
    } else if (type == mjGEOM_MESH) {
      int mid = m->geom_dataid[gi];
      gvcap += m->mesh_vertnum[mid];
      int pa = m->mesh_polyadr[mid], pn = m->mesh_polynum[mid];
      for (int p=0; p < pn; p++)
        gecap += m->mesh_polyvertnum[pa + p];  // upper bound (pre-dedup)
    }
  }
  int ngv = 0, nge = 0;
  mjtNum* gv = mj_stackAllocNum(d, 3 * gvcap);
  mjtNum* ge = mj_stackAllocNum(d, 6 * gecap);
  for (int gi=0; gi < m->ngeom; gi++) {
    if (m->geom_contype[gi] == 0 && m->geom_conaffinity[gi] == 0) continue;  // skip non-colliding
    if (m->body_weldid[m->geom_bodyid[gi]] != 0)
      continue;  // STATIC geoms only (matches the type-2/3/4 barrier filter)
    ngv += mjc_GeomVerts(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, gv + 3 * ngv);
    nge += mjc_GeomEdges(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, ge + 6 * nge);
  }
  // state vectors are sized 3*nstate (flex points 0..nfv-1), indexed by their state slot.
  mjtNum* x = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* xfree = mj_stackAllocNum(d, 3 * nstate);  // AL two-state: intersection-free output path (paper x[k])
  mjtNum* xtil = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* xold = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* xn = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* dx = mj_stackAllocNum(d, Na);

  // flex/sphere inertial prediction: q~ = q + h*v + h^2*qacc_smooth (point masses have no
  // Coriolis).
  mjtNum* qacc_pred = mj_stackAllocNum(d, (m->nv > 0 ? m->nv : 1));
  mju_copy(qacc_pred, d->qacc_smooth, m->nv);

  // STEP 4b: appended ARTICULATED kinetic state. qn_a = q_n; qtil_a = q_n (+) h*(v + h*qacc_smooth)
  // the free-flight predictor in generalized coords (mirrors mj_ipcTree); qdelta = accumulated
  // tangent from q_n (length N_artic). gradient/energy scratch (a_gdq/gr/gMr) are nv-sized; q*_a
  // are nq. All sized to 1 when na_artic==0 so the flex/bag path pays nothing (and stays
  // byte-identical).
  int nva = (na_artic ? m->nv : 1), nqa = (na_artic ? m->nq : 1),
      Nart = (N_artic > 0 ? N_artic : 1);
  mjtNum* qn_a = mj_stackAllocNum(d, nqa);
  mjtNum* qtil_a = mj_stackAllocNum(d, nqa);
  mjtNum* qcur_a = mj_stackAllocNum(d, nqa);
  mjtNum* qdelta = mj_stackAllocNum(d, Nart);
  mjtNum* qdtmp = mj_stackAllocNum(d, Nart);
  mjtNum* a_gdq = mj_stackAllocNum(d, nva);
  mjtNum* a_gr = mj_stackAllocNum(d, nva);
  mjtNum* a_gMr = mj_stackAllocNum(d, nva);
  for (int i=0; i < N_artic; i++)
    qdelta[i] = 0;
  if (na_artic) {
    mju_copy(qn_a, d->qpos, m->nq);  // q_n (d->qpos is at q_n throughout the flex solve)
    for (int i=0; i < m->nv; i++)
      a_gr[i] = m->dof_damping[i] *
                d->qvel[i];  // IMPLICIT joint damping: qacc_smooth has -M^-1 D v (explicit);
    mj_solveM(m, d, a_gMr, a_gr,
              1);  // add it BACK so the predictor is UNDAMPED -- the damping is now
    for (int i=0; i < m->nv; i++)
      a_gdq[i] = d->qvel[i] + h * qacc_pred[i] +
                 h * a_gMr[i];  // applied implicitly (M_eff) in the Newton solve
    mju_copy(qtil_a, qn_a, m->nq);
    mj_integratePos(m, qtil_a, a_gdq, h);  // q~ = q_n (+) h*(v + h*qacc_smooth)
  }

  const mjtNum* vx = d->flexvert_xpos;
  for (int v=0; v < npt; v++) {
    // xold position source: flex vertex from flexvert_xpos (the point array is flex-only now)
    for (int c=0; c < 3; c++)
      xold[3 * v + c] = vx[3 * pt2vg[v] + c];
    if (fidx[v] >= 0) {
      int da = dofadr[v];
      // qvel/qacc_smooth are in the body's local slide frame; map to WORLD via the body rotation R
      // (= d->xmat). R=I for unrotated bodies (cloth, the balls); non-identity for the rotated bag.
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum vw[3], aw[3];
      mju_mulMatVec3(vw, R, d->qvel + da);
      mju_mulMatVec3(aw, R, qacc_pred + da);
      for (int c=0; c < 3; c++)
        xtil[3 * v + c] = xold[3 * v + c] + h * vw[c] + h * h * aw[c];
    } else {
      for (int c=0; c < 3; c++)
        xtil[3 * v + c] = xold[3 * v + c];  // pinned: fixed
    }
  }
  for (int i=0; i < 3 * nstate; i++)
    x[i] = xold[i];  // start from last collision-free state (feasibility)

  mjtNum ih2 = 1.0 / (h * h);
  // The AL contact-multiplier warm start is d->flexvert_lambda, owned and zeroed by the mjData.
  char* ageload = (char*)mj_stackAllocByte(d, (size_t)(npt > 0 ? npt : 1), 1);  // ipc_ageStep temp
  // build the candidate-contact list once per step: detection threshold inflated by the predictor
  // displacement so any pair that could close during the step is captured (verified by gap checks)
  mjtNum maxdisp = 0;
  for (int v=0; v < npt; v++)
    if (fidx[v] >= 0) {
      mjtNum dd[3];
      for (int c=0; c < 3; c++)
        dd[c] = xtil[3 * v + c] - xold[3 * v + c];
      mjtNum L = sqrt(mju_dot3(dd, dd));
      if (L > maxdisp) maxdisp = L;
    }
  mjtNum thresh = 3 * ghat + 4 * maxdisp;
  // Static geom features (the bin) collide one-sided with the flex -- only the flex side moves, so
  // a pair closes at most at the flex speed. Their detection margin can therefore be half the
  // two-sided sphere/self margin (which budgets 2*maxdisp of approach from each side). Using the
  // full thresh here makes the 36-piece convex-decomposition bin (~1600 edges) generate ~600k
  // candidates that overflow candmax and drop the edge-edge contacts -> the bag sinks into the bin.
  // ghat + 2*maxdisp keeps the count ~160k.
  mjtNum threshGeom = ghat + 2.0 * maxdisp;
  int ncand = mjc_candidates(m, d, x, gv, ge, ngv, nge, rad, thresh, threshGeom, maxdisp, xold,
                             xtil, ghat, nfv, npt, fidx, flist, fxadr, nfd, pt2flex, cand, candmax);
  // A full candidate list means broad-phase pairs were DROPPED, and a dropped pair gets neither a
  // contact force nor CCD coverage, so geometry there can pass through. Report it rather than
  // tunnel quietly.
  if (ncand >= candmax) {
    mju_warning("IPC: candidate list full at %d pairs; contacts were dropped, so geometry there "
                "can pass through. Time = %.4f", ncand, d->time);
  }
  for (int c=0; c < ncand;
       c++) {  // AL: warm-start each candidate's lam from d->flexvert_lambda + cnt from the store
    int vv[4], nvv;
    mjc_conVerts(&cand[c], vv, &nvv);  // (binding = max over its free-point participants)
    mjtNum s = 0;
    for (int q=0; q < nvv; q++)
      if (d->flexvert_lambda[pt2vg[vv[q]]] > s) s = d->flexvert_lambda[pt2vg[vv[q]]];
    cand[c].lam = s;
    cand[c].cnt = ipc_conAge(&cand[c], d->flexvert_conage, pt2vg);
    cand[c].s = 0;
  }
  // warm start: with no candidate contacts within thresh, the predictor x~ is collision-free (the
  // thresh margin covers the step displacement), so it is a far better feasible initial guess than
  // xold and Newton converges in ~1 iteration instead of ~2 -- halving the cost of contact-free
  // steps.
  if (ncand == 0)
    for (int i=0; i < 3 * nstate; i++)
      x[i] = xtil[i];
  // mark active inter-flex candidates (gap already within their per-contact ghat). gap here
  // == iter-0 gap (x is unchanged until the Newton loop); geom contacts (types 2/3/4) add no
  // coupling so they're skipped.
  for (int c=0; c < ncand; c++) {
    if (cand[c].type >= 2) continue;
    mjtNum nn[3], cw[4];
    int iv[4], nidx;
    cgap[c] = mjc_conGap(&cand[c], m, d, x, gv, ge, rad, nn, iv, cw, &nidx, thresh);
  }
  // FLEX: SEED the persistent active set from the iter-0 broad-phase. addCand already pruned
  // cand to closing-or-distance-active pairs (per-pair closing-bound prune), so every iter-0
  // candidate is a valid initial active pair (seeding the active set from the first
  // discrete-collision-detection pass). lam/cnt were warm-started above. Aging + the per-iter
  // CCD-toi merge then maintain it (bounded, not holdall).
  naset = (ncand < candmax) ? ncand : candmax;
  for (int c=0; c < naset; c++) {
    aset[c] = cand[c];
  }
  for (int i=0; i < 3 * nstate; i++)
    xfree[i] = xold[i];  // intersection-free output path (paper x[k]), from feasible xold
  // Single AL Newton loop: inner_cap=1, beta ACCUMULATES to 1 (beta += (1-beta)*ac), terminate when
  // beta ~ 1 (complete advance) AND the articulated block converged AND (the flex block converged
  // OR the line search took a full step); no floor, no CFL cap, plain monotone-or-converged line
  // search.
  mjtNum beta = 0.0;
  int inner_cap = 1;           // single AL Newton iteration per outer
  int outer_cap = 1024;        // newton_max_iter
  int stall = 0, stalled = 0;  // CCD no-advance run-length; stalled==1 -> the feasible position
                               // froze (ill-conditioned)
  mjtNum last_ls_alpha =
      1.0;  // accepted line-search step of the last Newton iter; gates the outer termination
  int nefc_qp = 0;  // [ARCH-2] nefc as the QP saw it (post-trim); the merit's efc cost scores
                    // exactly these rows
  for (int outer=0; outer < outer_cap && N_total > 0;
       outer++) {  // OUTER loop (single AL loop; paper Alg.1). N_total (not N): the appended
                   // articulated block must run even with no flex packing (humanoid: N==0,
                   // N_total==nv)
    // WORKING SET: the PERSISTENT active set aset/naset maintained across outer iters by
    // ipc_mergeActiveSet (the persistent active-set manager). Assemble/energy/slack/lambda run over
    // the ENTIRE persistent set (the aging eviction -- not a per-iter ld0<ghat test -- bounds it).
    ipcCon* wcon = aset;
    int wn = naset;
    // N1 linearize_constraints (at xfree, Eq.10): ld0/ln/lcw/liv this iter -> c(x) is linear in x.
    for (int c=0; c < wn; c++)
      wcon[c].ld0 = mjc_conGap(&wcon[c], m, d, xfree, gv, ge, rad, wcon[c].ln, wcon[c].liv,
                               wcon[c].lcw, &wcon[c].lniv, ghat);
    // x PERSISTS across outer iters (libuipc/paper warm-start) -- NOT reset to xfree. The old
    // restart-from-xfree was a contact-set-explosion NaN firewall that ALSO prevented the primal
    // Newton from ever converging (one cold step per outer iter); with the faithful active-set +
    // the fixed broad-phase collar the NaN no longer fires, so persisting x lets the warm-started
    // Newton actually solve the equation of motion, which the restart was blocking.
    // N2 update_slack (at x): materialize s[c] and the slack-baked d the assemble /
    // lambda use.
    ipc_updateSlack(wcon, wn, x, xfree, rad, ghat, mass, ih2);
    int flex_converged_out = 0,
        artic_converged_out = 0;  // per-block convergence, carried to the outer termination
    int newton_converged_out = 0;  // both blocks converged on the last inner iteration
    for (int it=0; it < inner_cap && N_total > 0; it++) {
      // (The solveU-era gradient/Hessian assembly -- inertia, FEM stretch, bending, edge, soft,
      // gcon -- is gone: the QP computes the direction and the N7 merit scores the energy; only the
      // contact ASSEMBLY below remains, because it feeds the injected rows.) ONE SET: the whole
      // active set is assembled, and ipc_energy scores that same set. The set is already the
      // TOI-admitted one (ipc_mergeActiveSet), so "will collide this step" is structural and needs
      // no proximity test. The two proximity gates this replaces -- {gap < ghat} here and
      // {gap < ghat + |dx|} for the merit -- made the QP and the line search disagree about which
      // pairs exist: on a fast approach the model could hold ZERO rows while the merit scored
      // hundreds, so the merit rejected steps that were exactly optimal for the model, and those
      // truncated iterates were committed through the CCD blend (a 6x momentum error, caught by
      // IpcTest.SelfContactConservesMomentum).
      int nacon = wn;  // ONE SET: the injected rows ARE the active set, no copy and no cap
      // MuJoCo-solver injection (always on): swap ONLY the inner linear solve for MuJoCo's sparse
      // CG, KEEPING the AL/CCD outer loop (N8 dual-ascent + CCD stay in charge ->
      // penetration-free). Inject ALL barrier contacts (the active set: flex-flex + flex-sphere +
      // flex-static, no lniv filter) as extra-primal rows; native efc keeps the EDGES (equality)
      // but we DROP its contact rows (nefc -> ne+nf+nl) so they don't double-count our injection.
      // Then the converged qacc -> world x, and N8 + the next outer's re-linearize run normally on
      // it. pure-flex: inject ALL barrier contacts + drop native. pure-rigid: keep native. MIXED:
      // inject only flex-flex/self, keep native (rigid + flex-humanoid). MuJoCo's converged qacc is
      // used as the search DIRECTION for the shared N7 monotone-energy line search (below) -- a
      // converged step is big and needs the line search to stay stable.
      {
        int mixed = (na_artic > 0 && nfv > 0);
        int cap = 2 * nacon + 1;  // contact stiffness + dashpot pairs
        // NESTED frame: these arrays are sized by the live contact count and rebuilt every outer
        // iteration, so they must be released per iteration. Held in the outer frame instead they
        // would accumulate once per outer iteration, and esw is the largest of them.
        mj_markStack(d);
        // Contact as a FORCE plus a STIFFNESS: esbase/esw hold each pair's dof triples and their
        // weights, esD its stiffness, esf the constant force D*ref*J^T summed over pairs. The
        // curvature D*J^T*J rides with the metric operator (mj_extraStiffMulAdd), so it reaches
        // Ma/Mv and hence the gradient and both line-search coefficients with nothing re-derived
        // per solver iteration.
        int* esnpt = mj_stackAllocInt(d, cap);
        int* esbase = mj_stackAllocInt(d, mjNEXTRAPT * cap);
        mjtNum* esw = mj_stackAllocNum(d, 3 * mjNEXTRAPT * cap);
        mjtNum* esD = mj_stackAllocNum(d, cap);
        mjtNum* esf = mj_stackAllocNum(d, m->nv);
        mju_zero(esf, m->nv);
        int npair = 0;
        mjtNum h2 = 1.0 / ih2;
        for (int c=0; c < nacon; c++) {  // pure-flex: ALL barrier contacts (the flex CCD
          ipcCon* con = &wcon[c];  // tracks all -> all injected); mixed: only flex-flex/self
          if (mixed) {
            int othr = (con->type == 0)
                           ? con->idx[1]
                           : con->idx[2];  // mixed: inject only flex-flex/self contacts
            if (!(con->type <= 1 && con->idx[0] < nfv && othr < nfv)) continue;
          }
          mjtNum mu = ipc_muPair(con, mass, ih2);
          int cexp = ipc_cntExp(con->cnt);
          mjtNum D = mu;
          for (int e=0; e < cexp; e++)
            D *= IPC_DECAY;  // scale = mu*DECAY^cnt (the GN stiffness)
          mjtNum delta = mjc_off(mjc_conGhat(con, rad, ghat));
          mjtNum refc = -(con->ld0 - delta) + con->s + con->lam / mu;
          int np = 0;
          // This reference is EXACT: r(qacc(x)) = dd(x) algebraically (the h*Rqv terms cancel per
          // vertex against J*qacc; a h^2*qacc_smooth term would be spurious -- it cancels across
          // all-free pairs anyway since the weights sum to 0 and gravity is uniform, and adding it
          // costs deep backtracks on pin-adjacent pairs).
          for (int p=0; p < con->lniv; p++) {
            int v = con->liv[p];
            if (dofadr[v] < 0)
              continue;  // PINNED vertex: no dofs. Without this guard the pair read qvel[-1]/
                         // qacc_smooth[-1] (garbage reference) and wrote its J entries into dof
                         // columns -1,0,1 -- column -1 is OOB and 0,1 are the humanoid's free-root
                         // dofs: every pin-adjacent bag-string contact injected a phantom garbage
                         // coupling into the humanoid. (Reference contribution is exact to skip:
                         // pinned xfree==xold so its (xold-xfree) term is 0.)
            const mjtNum* R = d->xmat + 9 * pbody[v];
            const mjtNum* qv = d->qvel + dofadr[v];
            mjtNum Rtn[3], Rqv[3];
            mju_mulMatTVec3(Rtn, R, con->ln);
            mju_mulMatVec3(Rqv, R, qv);
            esbase[mjNEXTRAPT*npair + np] = dofadr[v];
            for (int k=0; k < 3; k++) {
              esw[3*mjNEXTRAPT*npair + 3*np + k] = con->lcw[p] * Rtn[k] * h2;
              refc -= con->lcw[p] * con->ln[k] * (xold[3 * v + k] + h * Rqv[k] - xfree[3 * v + k]);
            }
            np++;
          }
          // D = mu/h^2: MuJoCo's inertia is in ACCELERATION (1/h^2 heavier than ipc's position
          // inertia), so the contact stiffness is scaled by ih2 to match the contact/inertia RATIO.
          esnpt[npair] = np;
          esD[npair] = D * ih2;
          for (int q=0; q < np; q++) {
            int b = esbase[mjNEXTRAPT*npair + q];
            for (int k=0; k < 3; k++) {
              esf[b + k] += esD[npair] * refc * esw[3*mjNEXTRAPT*npair + 3*q + k];
            }
          }
          npair++;
        }
        int savednefc = d->nefc;
        if (na_artic == 0 && nfv > 0)
          d->nefc = d->ne + d->nf +
                    d->nl;  // pure-FLEX: drop native contacts (injection+CCD replace them);
        nefc_qp = d->nefc;  // [ARCH-2] the rows the QP actually minimizes -- the merit's efc cost
                            // scores exactly these
        // pure-RIGID + MIXED keep native contacts (rigid + flex-humanoid, w/ friction)
        // The injected penalty's cost/gradient/line-search curvature all live on the SHARED primal
        // path, so MuJoCo's matrix-free nonlinear CG solves the AL contact subproblem with NO
        // factorization -- it cannot hit the ill-conditioning that makes a direct Newton factor go
        // rank-deficient on the humanoid (stiff contact vs light flex mass, cond ~2e5). CG's exact
        // line search already carries the contact curvature Sum D*(val.p)^2, so each iterate's step
        // accounts for the contact even with M^-1 preconditioning (the paper's PCG route) -- robust
        // (no factorization to fail) and ~30-70x faster than the direct factor.
        // NONLINEAR CG inner solve (Polak-Ribiere, exact line search; the injected penalty's
        // cost/gradient/line-search curvature are on the shared primal path). A matrix-free
        // Newton direction by linear PCG over the same operator was tried and is slower here.
        // The outer AL converges at the same rate with a loosely solved
        // subproblem, so a fully converged Newton direction per outer is wasted work; the iteration
        // cap is the model's opt.iterations. Tightening the cap trades speed against occasional
        // deep line-search backtracks.
        // fwdConstraint zeroes solver_niter on entry, so each outer iteration would otherwise
        // overwrite the previous one and the step would report only its last solve. Save and add
        // back to report the step total, which is what the counter means for other integrators.
        int niter[mjNISLAND];
        for (int i=0; i < mjNISLAND; i++) niter[i] = d->solver_niter[i];
        mjExtraStiff es = {npair, esnpt, esbase, esw, esD, esf};
        // The constant contact force belongs in the SMOOTH force, not qfrc_constraint: the line
        // search's directional derivative is quadGauss[1] = v.Ma - v.qfrc_smooth and never reads
        // qfrc_constraint, so a force placed there is in the gradient but invisible to the line
        // search. qacc_smooth is deliberately left alone -- it enters only the reported cost,
        // never the gradient nor either line-search coefficient.
        mju_addTo(d->qfrc_smooth, esf, m->nv);
        d->efm_contact = (uintptr_t)&es;
        mj_fwdConstraintCG(
            m, d);  // efc_b + warmstart + mj_solCG (Gauss + efc + injected penalty contacts),
                    // pinned to CG over the monolithic problem regardless of the model's own solver
        d->efm_contact = 0;
        mju_subFrom(d->qfrc_smooth, esf, m->nv);   // restore: the caller's smooth force is shared
        for (int i=0; i < mjNISLAND; i++) d->solver_niter[i] += niter[i];
        d->nefc = savednefc;
        // Barrier-free AL contact energy is FINITE, so a CONVERGED inner step is big and MUST be
        // line-searched: converge+no-line-search explodes (step 145), converge+line-search is
        // stable (morph ladder on scene_ipc_balls_edge2, 2026-07-04). Treat MuJoCo's converged
        // solution as the search DIRECTION dx (free-dof [0,N) + articulated [N,N_total)); the
        // shared N7 monotone-energy line search below picks the step magnitude and commits
        // x/qdelta.
        for (int v=0; v < npt; v++)
          if (fidx[v] >= 0) {
            int fi = fidx[v];
            const mjtNum* R = d->xmat + 9 * pbody[v];
            const mjtNum* qv = d->qvel + dofadr[v];
            const mjtNum* qa = d->qacc + dofadr[v];
            mjtNum Rqv[3], Rqa[3];
            mju_mulMatVec3(Rqv, R, qv);
            mju_mulMatVec3(Rqa, R, qa);
            for (int k=0; k < 3; k++) {
              mjtNum xf = xold[3 * v + k] + h * Rqv[k] + h2 * Rqa[k];
              dx[3 * fi + k] = xf - x[3 * v + k];
            }  // search DIRECTION to MuJoCo's solution (NOT a commit)
          }
        for (int a=0; a < na_artic; a++) {
          int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
          for (int i=0; i < nd; i++)
            dx[N + (o - N + i)] = (h * d->qvel[da + i] + h2 * d->qacc[da + i]) - qdelta[o - N + i];
        }
        mj_freeStack(d);  // releases the ep* rows for this outer iteration
      }
      if (na_artic) {  // FK the articulated geoms at q_n (+) qdelta so type-3 soft contacts read
                       // live poses (qM/qLD
        for (int i=0; i < m->nv; i++)
          a_gdq[i] = 0;  // stay frozen at q_n -> mj_mulM/mj_solveM unaffected)
        for (int a=0; a < na_artic; a++) {
          int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
          for (int i=0; i < nd; i++)
            a_gdq[da + i] = qdelta[o + i - N];
        }
        mju_copy(d->qpos, qn_a, m->nq);
        mj_integratePos(m, d->qpos, a_gdq, 1);
        mj_kinematics(m, d);
      }
      // dx is MuJoCo's converged search direction (from the injection block above); fall through to
      // the N7 line search.
      // N6 newton_tolerance (L-infinity dx checker, <= velocity_tol*dt on the SOLVED dx BEFORE line
      // search), SPLIT at the flex/articulated boundary. The ARTICULATED block [N, N_total) must
      // truly converge before the outer loop may terminate: accepting an unconverged rigid step
      // injects energy through the explicit-aref gcon/soft contacts (the humanoid explosion). The
      // FLEX block [0, N) keeps the alpha==1 bail: AL flex contact is stable unconverged, and
      // forcing its convergence under load costs ~100+ outers (the AL multiplier's slow tail). Full
      // convergence (both blocks) still short-circuits the line search.
      int newton_converged = 0;
      {  // split L-infinity dx checker: a block is converged once its largest dx component is below
         // velocity_tol*dt.
        mjtNum maxf = 0, maxa = 0;
        for (int i=0; i < N; i++) {
          mjtNum a = dx[i] < 0 ? -dx[i] : dx[i];
          if (a > maxf) maxf = a;
        }
        for (int i=N; i < N_total; i++) {
          mjtNum a = dx[i] < 0 ? -dx[i] : dx[i];
          if (a > maxa) maxa = a;
        }
        flex_converged_out = (maxf <= IPC_VEL_TOL * h);   // vacuously 1 when N==0 (pure rigid)
        artic_converged_out = (maxa <= IPC_VEL_TOL * h);  // vacuously 1 when N_artic==0 (pure flex)
        newton_converged = flex_converged_out && artic_converged_out;
        newton_converged_out = newton_converged;
      }
      // [ARCH-2] N7 line search: the merit scores EXACTLY the QP objective with live flex-flex gaps
      // -- Gauss/inertia (ipc_energy inertia + ipc_articResid) + MuJoCo's own efc cost
      // (ipc_efcCost: edges, limits, rigid-rigid and rigid-flex contacts, no hand-replicated
      // scorers) + the AL flex-flex penalty (live gaps, the per-outer relinearized nonlinearity the
      // line search exists for). The line search itself is required: the paper line-searches the
      // nonlinear subproblem energy; accepting the linearized full step overshoots into penetration
      // and the N8 CCD advance collapses (beta grinds at the outer cap).
      mjtNum E0 = ipc_energy(m, d, npt, x, xtil, fidx, mass, h, rad, ghat, gv, ge, wcon, wn,
                             xold, xfree) +
                  ipc_articResid(m, d, qn_a, qtil_a, qdelta, na_artic, atid, aoff, N, ih2, a_gdq,
                                 qcur_a, a_gr, a_gMr) +
                  ipc_efcCost(m, d, x, xold, npt, fidx, dofadr, pbody, qdelta, na_artic, atid, aoff,
                              N, h, nefc_qp) +
                  ipc_metricCost(m, d, x, xold, npt, fidx, dofadr, pbody, qdelta, na_artic, atid,
                                 aoff, N, h);
      mjtNum alpha = 1.0;
      // Line search: plain monotone decrease + 1e-12 slop, OR newton_converged short-circuit; 8
      // backtracks /2; ALWAYS accept the final trial (no Armijo-break grind).
      for (int ls=0; ls < 8; ls++) {
        for (int i=0; i < 3 * nstate; i++)
          xn[i] = x[i];
        for (int v=0; v < npt; v++)
          if (fidx[v] >= 0) {
            int fi = fidx[v];
            for (int c=0; c < 3; c++)
              xn[3 * v + c] = x[3 * v + c] + alpha * dx[3 * fi + c];
          }
        for (int j=0; j < N_artic; j++)
          qdtmp[j] = qdelta[j] + alpha * dx[N + j];  // articulated trial tangent
        if (na_artic) {  // FK at the trial articulated config (next-iter gcon/soft reads live
                         // poses; qM/qLD stay @ q_n)
          for (int i=0; i < m->nv; i++)
            a_gdq[i] = 0;
          for (int a=0; a < na_artic; a++) {
            int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
            for (int i=0; i < nd; i++)
              a_gdq[da + i] = qdtmp[o + i - N];
          }
          mju_copy(d->qpos, qn_a, m->nq);
          mj_integratePos(m, d->qpos, a_gdq, 1);
          mj_kinematics(m, d);
        }
        mjtNum Etr = ipc_energy(m, d, npt, xn, xtil, fidx, mass, h, rad, ghat, gv, ge, wcon,
                                wn, xold, xfree) +
                     ipc_articResid(m, d, qn_a, qtil_a, qdtmp, na_artic, atid, aoff, N, ih2, a_gdq,
                                    qcur_a, a_gr, a_gMr) +
                     ipc_efcCost(m, d, xn, xold, npt, fidx, dofadr, pbody, qdtmp, na_artic, atid,
                                 aoff, N, h, nefc_qp) +
                     ipc_metricCost(m, d, xn, xold, npt, fidx, dofadr, pbody, qdtmp, na_artic, atid,
                                    aoff, N, h);
        if (Etr <= E0 + 1e-12 || newton_converged) break;
        alpha *= 0.5;
      }
      for (int i=0; i < 3 * nstate; i++)
        x[i] = xn[i];  // always accept the final trial
      for (int j=0; j < N_artic; j++)
        qdelta[j] = qdtmp[j];  // commit the accepted articulated tangent
      last_ls_alpha = alpha;  // accepted global line-search step (the outer loop terminates only on
                              // a full step)
    }
    // N8 non-penetration advance (every iter): dual ascent (lambda update + cnt) -> re-query@xfree
    // -> CCD -> advance xfree. Order is FIXED:
    //   lambda update (on the persistent set) -> prepare CCD -> detect trajectory candidates(1.0)
    //   -> alpha = CCD time-of-impact filter(1.0) -> active-set update (MERGE/AGING) -> advance
    //   non-penetrate positions(alpha)
    //   -> beta = beta + (1-beta)*alpha.
    // N8a lambda update + cnt on the ASSEMBLED persistent set (aset), then sink lam into
    // start).
    ipc_flexLamUpdate(m, d, x, xfree, gv, ge, rad, ghat, mass, ih2, aset, naset,
                      d->flexvert_lambda, pt2vg, npt);
    ipc_ageStep(d->flexvert_conage, aset, naset, pt2vg, npt, ageload);
    // N8b prepare CCD: disp = x - xfree (free-dof layout), base = xfree.
    for (int v=0; v < npt; v++)
      if (fidx[v] >= 0)
        for (int c=0; c < 3; c++)
          dx[3 * fidx[v] + c] = x[3 * v + c] - xfree[3 * v + c];
    // N8c detect trajectory candidates(1.0): re-query the broad-phase at xfree over the swept
    // segment xfree->x. FIXED collar (3*ghat, maxdisp=0, d_hat expansion + thickness): the swept
    // segment covers gross motion (no-tunnel), and maxdisp=0 keeps the count bounded regardless of
    // |x-xfree| (the old displacement-scaled collar ballooned to 238k candidates when x flung ->
    // NaN). This is the NEW-candidate source for the merge.
    ncand = mjc_candidates(m, d, xfree, gv, ge, ngv, nge, rad, 3 * ghat, 3 * ghat, 0.0, xfree, x,
                           ghat, nfv, npt, fidx, flist, fxadr, nfd, pt2flex, cand, candmax);
    if (ncand >= candmax) {
      mju_warning("IPC: candidate list full at %d pairs; contacts were dropped, so geometry there "
                  "can pass through. Time = %.4f", ncand, d->time);
    }
    for (int c=0; c < ncand; c++) {
      mjtNum nn[3], cw[4];
      int idv[4], ni;  // gaps at xfree (for CCD + admission)
      cgap[c] = mjc_conGap(&cand[c], m, d, xfree, gv, ge, rad, nn, idv, cw, &ni, ghat);
    }
    // N8d CCD time-of-impact filter(1.0): CCD over the trajectory candidates -> the advance alpha.
    // appr[c] flags each candidate whose full step closes its gap into the active zone (== its
    // individual CCD time-of-impact < 1).
    mjtNum ac =
        mjc_advance(m, d, xfree, dx, gv, ge, rad, nfv, fidx, cand, ncand, cgap, pt2flex, appr);
    // N8e update_active_set: MERGE the admitted broad-phase candidates into the persistent set
    // (keep existing with abs(cnt)<=25, add new with toi<1-1e-6, dedup). Admit a candidate iff it
    // is closing this step (appr) OR already distance-active (gap<=0): both are the CCD
    // time-of-impact < 1-1e-6 set. New entries seed lam(flexvert_lambda)/cnt(store).
    for (int c=0; c < ncand; c++)
      actc[c] = (appr[c] || cgap[c] <= 0.0) ? 1 : 0;
    ipc_mergeActiveSet(d, aset, &naset, cand, ncand, actc, amerge, d->flexvert_lambda,
                       d->flexvert_conage, pt2vg, candmax);
    if (naset >= candmax) {
      mju_warning("IPC: active set full at %d pairs; admitted contacts were dropped, so geometry "
                  "there can pass through. Time = %.4f", naset, d->time);
    }
    // N8f advance non-penetrate positions(alpha): advance the FLEX xfree, iff alpha > alpha lower
    // bound. beta -> 1.
    if (ac > IPC_ALPHA_LB)
      for (int i=0; i < 3 * npt; i++)
        xfree[i] = (1.0 - ac) * xfree[i] + ac * x[i];
    beta = beta + (1.0 - beta) * ac;
    // Terminate once the advance is COMPLETE and the full Newton step was accepted (the iterate is
    // in the step's basin) or both blocks converged. A complete advance is required to commit: beta
    // is the fraction of the step's motion the committed positions have absorbed, not a residual --
    // accepting beta<1 discards (1-beta) of the motion while time still advances by h (time-dilated,
    // slow-motion physics; sustained beta~0.9 commits in contact-rich phases read as a visible
    // freeze). Partial advances are fine WITHIN the loop; the stall path below is the escape when
    // CCD cannot complete, and warns at the end of the step.
    // The convergence test is on the ABSOLUTE Newton step, which for a moving body is its physical
    // motion h*v, so a body faster than IPC_VEL_TOL can never satisfy it on the first iteration.
    // Requiring it of the articulated block therefore cost a second pass on every step whose only
    // effect was to certify the first: measured maxa=4.6e-3 on pass 0 (== h*v for the falling
    // humanoid) and 8e-17 on pass 1. Both blocks now take the full-step bail.
    // ACFIX: terminate on THIS iteration's advance being complete (ac==1 => xfree == x, so the
    // committed state is the optimizer iterate) rather than on the cumulative beta ledger, which
    // SATURATES: once beta hits 1 the recurrence beta += (1-beta)*ac can never register that a new,
    // different x was only partially absorbed, and the commit is a stale blend.
    if (beta >= 1.0 - 1e-6 && (last_ls_alpha >= 1.0 - 1e-9 || newton_converged_out))
      break;
    if (ac > IPC_ALPHA_LB)
      stall = 0;
    else if (++stall >= IPC_STALL_MAX) {
      stalled = 1;
      break;
    }  // CCD froze -> stop grinding, warn below
  }  // OUTER loop close
  for (int i=0; i < 3 * nstate; i++)
    x[i] = xfree[i];  // commit the intersection-free output (readback uses x)
  // Restore the position-dependent fields to q_n. The solve FKs the articulated trees at trial
  // configurations (mj_kinematics above), which leaves xpos/xmat/subtree_com/cinert at a
  // configuration that was never committed; every other integrator leaves them at the pre-step
  // configuration, since mj_step computes them once in mj_forward and integrators only advance
  // qpos/qvel.
  if (na_artic) {
    mju_copy(d->qpos, qn_a, m->nq);
    mj_kinematics(m, d);
    mj_comPos(m, d);
  }
  // POSITION-level integrator: the solve yields q_{n+1} directly, so qpos and qvel are committed
  // below rather than through mj_advance's velocity/position stages. Routing them through it is
  // algebraically identical but not bitwise (the qacc round trip re-associates the arithmetic), and
  // under stiff contact that perturbation grows. qacc is published as the step's effective
  // acceleration, (v_new - v_old)/h, matching ipc_efcCost; dofs the solve does not own stay at
  // qacc_smooth (free flight).
  mjtNum* qacc_out = mj_stackAllocNum(d, m->nv);
  mju_copy(qacc_out, d->qacc_smooth, m->nv);
  for (int v=0; v < npt; v++)
    if (fidx[v] >= 0) {
      int da = dofadr[v];
      int qa = qpadr[v];  // qpos by joint qposadr; qvel/accel by dof address
      // x/xold are world; the slide dofs are in the body-local frame -> map the world displacement
      // back through R^T (= d->xmat^T). For an unrotated body this is the identity (dp_local ==
      // dp_world).
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum dpw[3], dpl[3];
      for (int c=0; c < 3; c++)
        dpw[c] = x[3 * v + c] - xold[3 * v + c];
      mju_mulMatTVec3(dpl, R, dpw);
      for (int c=0; c < 3; c++) {
        qacc_out[da + c] = (dpl[c] / h - d->qvel[da + c]) / h;  // effective qacc: reads v_old first
        d->qvel[da + c] = dpl[c] / h;
        d->qpos[qa + c] += dpl[c];
      }
    }
  // ARTICULATED readback: qdelta = accumulated generalized tangent from q_n. Commit q_{n+1}=q_n (+)
  // qdelta on each appended tree's qpos (per joint) and v=qdelta/h on its dofs (mirrors mj_ipcTree;
  // NO R^T -- generalized coords).
  if (na_artic) {
    for (int i=0; i < m->nv; i++)
      a_gdq[i] = 0;
    for (int a=0; a < na_artic; a++) {
      int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
      for (int i=0; i < nd; i++) {
        a_gdq[da + i] = qdelta[o + i - N];
        qacc_out[da + i] = (qdelta[o + i - N] / h - d->qvel[da + i]) / h;  // reads v_old
      }
    }
    mju_copy(qcur_a, qn_a, m->nq);
    mj_integratePos(m, qcur_a, a_gdq, 1);  // q_{n+1} = q_n (+) qdelta
    for (int a=0; a < na_artic; a++) {
      int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
      for (int bi=m->tree_bodyadr[t]; bi < m->tree_bodyadr[t] + m->tree_bodynum[t]; bi++)
        for (int j=m->body_jntadr[bi]; j < m->body_jntadr[bi] + m->body_jntnum[bi]; j++) {
          int qa = m->jnt_qposadr[j], nqj = (m->jnt_type[j] == mjJNT_FREE   ? 7
                                             : m->jnt_type[j] == mjJNT_BALL ? 4
                                                                            : 1);
          for (int k=0; k < nqj; k++)
            d->qpos[qa + k] = qcur_a[qa + k];
        }
      for (int i=0; i < nd; i++)
        d->qvel[da + i] = qdelta[o + i - N] / h;
    }
  }
  // Publish the step's effective acceleration: every other integrator hands mj_advance the qacc that
  // moved the system, and this is the equivalent for a position-level solve.
  mju_copy(d->qacc, qacc_out, m->nv);
  // Take the rest of the step from the shared tail. qacc/qvel are zeros so the velocity and
  // position stages are no-ops -- both are committed above -- while the history buffers, activation
  // dynamics, sleep, plugin states and time all run.
  // The warmstart save is deliberately undone: the inner mj_fwdConstraint solves a DIFFERENT
  // subproblem every outer iteration (the injected AL rows move with the active set), so the
  // previous step's acceleration is not a useful initial guess, and warmstarting makes the step
  // depend on solve history -- which costs the agreement with Euler that
  // IpcTest.RigidContactMatchesEuler asserts.
  mjtNum* zero_nv = mj_stackAllocNum(d, m->nv);
  mjtNum* ws_keep = mj_stackAllocNum(d, m->nv);
  mju_zero(zero_nv, m->nv);
  mju_copy(ws_keep, d->qacc_warmstart, m->nv);
  mj_advance(m, d, d->act_dot, zero_nv, zero_nv);
  mju_copy(d->qacc_warmstart, ws_keep, m->nv);
  mj_freeStack(d);  // releases every per-step array allocated after the mark above
  // A frozen CCD commits only beta of the motion while time still advances by h, which reads as
  // slow-motion physics rather than as a failure. Say so: the symptom is otherwise silent.
  if (stalled) {
    mju_warning("IPC: CCD stalled after %d iterations without a feasible advance; the step is "
                "incomplete (advanced %.3f of the motion). Time = %.4f",
                IPC_STALL_MAX, beta, d->time);
  }
}
