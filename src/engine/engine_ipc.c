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

#include <math.h>
#include <stddef.h>
#include <stdlib.h>  // qsort (sparse Hessian pattern build)

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include "engine/engine_forward.h"           // mj_Euler (fallback)
#include "engine/engine_collision_driver.h"  // mj_collision (re-run to source movable-rigid contacts)
#include "engine/engine_core_constraint.h"  // mj_makeConstraint / mj_referenceConstraint (reuse efc_aref/efc_R)
#include "engine/engine_solver.h"  // [M3] mj_setExtraPrimal / mjExtraPrimal (inject flex-flex contact into MuJoCo CG)
#include "engine/engine_support.h"  // mj_mulM, mj_integratePos, mj_differentiatePos (per-tree mass + manifold)
#include "engine/engine_core_smooth.h"  // mj_solveM, mj_kinematics, mj_comPos (per-tree M^-1, FK at trial q)
#include "engine/engine_core_util.h"  // mj_local2Global (anchor body-local -> world for the live gap)
#include "engine/engine_util_solve.h"    // mju_cholFactor, mju_cholSolve (dense Newton solve)
#include "engine/engine_util_blas.h"     // mju_dot3, mju_mulMatVec3
#include "engine/engine_util_spatial.h"  // mju_cross (flex bending)
#include "engine/engine_util_errmem.h"   // mju_malloc, mju_free







#define IPC_CDAMP_FLEX 0.1  // flex-contact normal dashpot coeff (cde = IPC_CDAMP_FLEX*m/h^2)
// ---- augmented-Lagrangian (AL) solve (pure-flex path) parameters ----
#define IPC_MU_SCALE 5e7  // per-vertex AL stiffness: mu_v = mass*IPC_MU_SCALE*h^2
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
#define IPC_ASET_AGE 25  // active-set update: evict a persistent pair once abs(cnt) > IPC_ASET_AGE
#define IPC_ASET_TOI \
  1e-6  // active-set update: admit a new broad-phase pair iff its CCD toi < 1 - IPC_ASET_TOI





// per-pair AL stiffness mu = min over the pair's flex/sphere vertices of mu_v.
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
  for (int q = 0; q < nvv; q++) {
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
  for (int k = 0; k < 4; k++)
    id[k] = con->idx[k];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3 - i; j++)  // sort idx ascending (order-independent key)
      if (id[j] > id[j + 1]) {
        int t = id[j];
        id[j] = id[j + 1];
        id[j + 1] = t;
      }
  unsigned long hh = (unsigned long)con->type * 1000003ul + (unsigned long)(con->gi + 1);
  for (int k = 0; k < 4; k++)
    hh = hh * 1000003ul + (unsigned long)(id[k] + 1);
  return hh;
}


// rigorous additive CCD (Li et al.): largest alpha in [0,1] s.t. advancing x by alpha*dxw keeps
// every candidate's surface gap above 20% of its current value -- conservative advancement, no
// tunneling. For self pairs the common (mean) displacement is removed so coherent motion (free
// fall) isn't throttled; geom features are fixed so only the flex side's speed bounds the
// gap-shrink rate. IMPORTANT: mean-removal is only valid when the pair can genuinely move together.
// A sphere-vs-flex contact (type 0 with the "vertex" a rigid point, idx[0]>=nfv) is NOT such a pair
// -- the flex side may be blocked (e.g. the bag bottom pinned against the bin floor) so it cannot
// follow the sphere. Removing the mean there underestimates the gap-shrink rate and lets the sphere
// punch through, so it is excluded. IPC-specific: Li-et-al additive conservative-advancement TOI --
// NOT mjc_ccd (GJK/EPA per-pair distance, despite the shared name); the engine has no
// time-of-impact routine.

// [P1c] The FEM machinery (ipcElem membrane elements, ipcBend flaps, ipcEdge soft rows) is gone:
// stretch is carried by the native edge-equality efc rows in the inner QP, and stretch+bending are
// carried by the native edge-equality efc rows; this solver has no elastic energy of its
// own (FEM elasticity is rejected at entry -- the CG integrators' effective metric owns it).

// re-test ONE held contact at the optimizer x and admit it to the assembled set (acon -> the
// injected QP rows). Linearized at xfree (ld0/ln/lcw/liv this iter): c_raw(x) = (ld0 - delta) +
// sum_p lcw*ln.(x-xfree). *gout refreshes the pair's maintained gap lower bound (the active-set
// re-test + the CCD's conservative cap). (The solveU-era gradient/GN-block assembly is gone: the QP
// owns the direction, the N7 merit owns the energy.)
static void ipc_try(ipcCon con, const mjtNum* x, const mjtNum* xfree, const mjtNum* rad,
                    mjtNum ghat, ipcCon* acon, int* nacon, int amax, mjtNum* gout) {
  mjtNum craw =
      con.ld0 - mjc_conGhat(&con, rad);  // c_raw(x) (un-baked linearized gap)
  for (int p = 0; p < con.lniv; p++) {
    int v = con.liv[p];
    for (int k = 0; k < 3; k++)
      craw += con.lcw[p] * con.ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
  }
  *gout = craw;  // linearized gap, for the inner active-set test (cgap < ghat)
  if (craw >= ghat || *nacon >= amax) return;  // beyond detection range -> not HELD; or list full
  acon[*nacon] = con;
  (*nacon)++;
}

// slack update (loop step N2, at the optimizer x): materialize the AL slack s = max(0, c_raw -
// lam/mu) for every held candidate so the subsequent assemble (ipc_try)/energy (ipc_energy) use the
// exact slack-baked d = c_raw - s - lam/mu, and the lambda update (ipc_flexLamUpdate) can un-bake
// it. c_raw is the LINEARIZED gap at x (ld0 set this iter by linearize at xfree). Order is FIXED:
// linearize -> slack update -> assemble -> ... -> lambda.
static void ipc_updateSlack(ipcCon* cand, int ncand, const int* held, const mjtNum* x,
                            const mjtNum* xfree, const mjtNum* rad, mjtNum ghat, const mjtNum* mass,
                            mjtNum ih2) {
  for (int c = 0; c < ncand; c++) {
    if (!held[c]) {
      cand[c].s = 0;
      continue;
    }
    ipcCon* con = &cand[c];
    mjtNum mu = ipc_muPair(con, mass, ih2);
    mjtNum craw = con->ld0 - mjc_conGhat(con, rad);  // c_raw(x) = (ld0-delta) + d_grad.(x-xfree)
    for (int p = 0; p < con->lniv; p++) {
      int v = con->liv[p];
      for (int k = 0; k < 3; k++)
        craw += con->lcw[p] * con->ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    mjtNum t = craw - con->lam / mu;
    con->s =
        (t > 0)
            ? t
            : 0;  // s = max(0, c_raw - lam/mu); baked d = c_raw - s - lam/mu (in ipc_try/energy)
  }
}

// AL contact multiplier per FREE POINT (flex vertex / rigid sphere): the cross-step warm-start
// store for the ipc_try contacts, whose LIVE multiplier rides in ipcCon.lam. Seeded into cand[].lam
// at step start, sunk back after the step (binding = max over a contact's free-point participants).
// npt-sized.
static mjtNum* g_pal = NULL;
static int g_palN = 0;

// per-pair cnt state machine (active-set aging + 0.3^c stiffness decay) persisted across the
// per-iter candidate re-query AND across steps. Two open-addressing hash tables keyed by
// ipc_pairHash (type+sorted idx+gi): g_cntOld* is read-only this step (last step's final cnts),
// g_cntNew* is written this step. They swap at step start (g_cntStepBegin) so the table never grows
// unbounded as contact identities churn (string-in-bag). A re-queried/new pair re-hydrates cnt from
// OLD (miss -> 0); after each update_lambda the cnt is sunk into NEW.
static unsigned long *g_cntKeyO = NULL, *g_cntKeyN = NULL;
static int *g_cntValO = NULL, *g_cntValN = NULL;
static int g_cntCap = 0, g_cntUsedN = 0;
static void ipc_cntStepBegin(
    int candmax) {  // size to ~4x the candidate ceiling (load factor < 0.25); swap+clear NEW
  int cap = 1;
  while (cap < 4 * candmax + 16)
    cap <<= 1;
  if (cap != g_cntCap) {
    mju_free(g_cntKeyO);
    mju_free(g_cntKeyN);
    mju_free(g_cntValO);
    mju_free(g_cntValN);
    g_cntCap = cap;
    g_cntKeyO = (unsigned long*)mju_malloc((size_t)cap * sizeof(unsigned long));
    g_cntKeyN = (unsigned long*)mju_malloc((size_t)cap * sizeof(unsigned long));
    g_cntValO = (int*)mju_malloc((size_t)cap * sizeof(int));
    g_cntValN = (int*)mju_malloc((size_t)cap * sizeof(int));
    for (int i = 0; i < cap; i++) {
      g_cntKeyO[i] = 0;
      g_cntValO[i] = 0;
    }
  } else {
    unsigned long* tk = g_cntKeyO;
    g_cntKeyO = g_cntKeyN;
    g_cntKeyN = tk;  // NEW (just written) becomes OLD
    int* tv = g_cntValO;
    g_cntValO = g_cntValN;
    g_cntValN = tv;
  }
  for (int i = 0; i < g_cntCap; i++) {
    g_cntKeyN[i] = 0;
    g_cntValN[i] = 0;
  }  // key 0 = empty slot
  g_cntUsedN = 0;
}
static int ipc_cntGet(unsigned long key) {  // re-hydrate cnt for a pair from OLD (miss -> 0)
  if (g_cntCap == 0 || key == 0) return 0;
  unsigned long mask = (unsigned long)(g_cntCap - 1), h = key & mask;
  for (int probe = 0; probe < g_cntCap; probe++) {
    if (g_cntKeyO[h] == 0) return 0;
    if (g_cntKeyO[h] == key) return g_cntValO[h];
    h = (h + 1) & mask;
  }
  return 0;
}
static void ipc_cntSet(unsigned long key,
                       int val) {  // sink updated cnt into NEW (insert/overwrite)
  if (g_cntCap == 0 || key == 0) return;
  unsigned long mask = (unsigned long)(g_cntCap - 1), h = key & mask;
  for (int probe = 0; probe < g_cntCap; probe++) {
    if (g_cntKeyN[h] == 0) {
      if (g_cntUsedN >= g_cntCap / 2)
        return;  // table near full (shouldn't happen at <0.25 load): drop, harmless
      g_cntKeyN[h] = key;
      g_cntValN[h] = val;
      g_cntUsedN++;
      return;
    }
    if (g_cntKeyN[h] == key) {
      g_cntValN[h] = val;
      return;
    }
    h = (h + 1) & mask;
  }
}

// lambda update + cnt state machine (loop step N8a, at the optimizer x): for every held candidate,
// un-bake d to the raw linearized gap c_raw, then the two-branch AL dual update (the slack
// condition c_raw>lam/mu) with the cnt aging machine (using the lambda value from BEFORE this
// update):
//   if (c_raw - lam/mu > 0)  { lam = 0;  cnt += (cnt>=0 ? +1 : -1); }                  // inactive
//   (slack>0) else                     { lam -= c_raw*mu;  cnt = (cnt==0 || cnt>5) ? 0 : -1; }   //
//   active: lam grows
// (NO max(0,.) clamp.) Then SINK lam into the per-free-point store g_pal for next-step warm start.
// cnt rides in cand[c].cnt and is persisted per-pair by the caller.
static void ipc_flexLamUpdate(const mjModel* m, const mjData* d, const mjtNum* x,
                              const mjtNum* xfree, const mjtNum* gv, const mjtNum* ge, mjtNum r,
                              const mjtNum* rad, mjtNum ghat, const mjtNum* mass, mjtNum ih2,
                              ipcCon* cand, int ncand, const int* held, mjtNum* g_pal, int npt) {
  for (int c = 0; c < ncand; c++) {
    if (!held[c]) {
      cand[c].lam = 0;
      continue;
    }
    ipcCon* con = &cand[c];
    mjtNum mu = ipc_muPair(con, mass, ih2);
    mjtNum lam0 = con->lam;  // lambda BEFORE this update (the dual update reads pre-update)
    mjtNum craw = con->ld0 - mjc_conGhat(con, rad);  // c_raw(x) (un-baked: == baked_d + s + lam/mu)
    for (int p = 0; p < con->lniv; p++) {
      int v = con->liv[p];
      for (int k = 0; k < 3; k++)
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
  for (int p = 0; p < npt; p++)
    g_pal[p] = 0;  // sink: per-free-point binding multiplier (warm-start src)
  for (int c = 0; c < ncand; c++) {
    if (cand[c].lam <= 0) continue;
    int vv[4], nvv;
    mjc_conVerts(&cand[c], vv, &nvv);
    for (int q = 0; q < nvv; q++)
      if (cand[c].lam > g_pal[vv[q]]) g_pal[vv[q]] = cand[c].lam;
  }
}

// active-set update (MERGE/AGING of the persistent active set).
// Inputs: the existing persistent set aset[0..*naset) (cnt rides in each ipcCon), and the fresh
// broad-phase candidates cand[0..ncand) with a per-candidate "closing this step" admission flag
// cadmit[c] (== the CCD time-of-impact < 1-1e-6). Rule:
//   - KEEP an existing aset pair iff abs(cnt) <= IPC_ASET_AGE (else evict).
//   - ADD a new broad-phase candidate iff cadmit[c] AND its pairHash is not already present (dedup
//   by 64-bit-ish
//     pairHash). New entries seed cnt=0, lam=0; we re-hydrate cnt from the cross-step store and lam
//     from the per-point warm-start g_pal so the cnt aging and AL multiplier survive across steps
//     (our persistence path).
// The merged set is written back into aset/*naset. The new merged set's fields (ld0/ln/.../s) are
// refreshed by the next iter's linearize_constraints + update_slack, so only type/idx/gi/lam/cnt
// need to be carried here.
static void ipc_mergeActiveSet(ipcCon* aset, int* naset, const ipcCon* cand, int ncand,
                               const int* cadmit, ipcCon* amerge, const mjtNum* g_pal,
                               int candmax) {
  // dedup hash over the merged set: open addressing keyed by pairHash (key 0 = empty slot; a true
  // hash of 0 is astronomically unlikely and at worst causes one duplicate, harmless).
  // Presence-only -> no value array needed.
  int cap = 1;
  while (cap < 4 * (*naset + ncand) + 16)
    cap <<= 1;
  static unsigned long* mkey = NULL;
  static int mcap = 0;
  if (cap > mcap) {
    mju_free(mkey);
    mcap = cap;
    mkey = (unsigned long*)mju_malloc((size_t)cap * sizeof(unsigned long));
  }
  unsigned long mask = (unsigned long)(cap - 1);
  for (int i = 0; i < cap; i++)
    mkey[i] = 0;
  int nm = 0;
  // 1) keep existing pairs with abs(cnt) <= IPC_ASET_AGE (aging eviction).
  for (int c = 0; c < *naset; c++) {
    int cnt = aset[c].cnt;
    if ((cnt < 0 ? -cnt : cnt) > IPC_ASET_AGE) continue;
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
  for (int c = 0; c < ncand; c++) {
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
    for (int q = 0; q < nvv; q++)
      if (g_pal[vv[q]] > s) s = g_pal[vv[q]];
    con.lam = s;
    con.cnt = ipc_cntGet(ipc_pairHash(&con));
    con.s = 0;  // re-hydrate cnt across steps
    mkey[h] = key;
    amerge[nm++] = con;
  }
  for (int c = 0; c < nm; c++)
    aset[c] = amerge[c];
  *naset = nm;
}


// RIGID-RIGID geom-geom contact (articulated body vs articulated body / static), anchored at q_n
// with a frozen normal + direction b=J_B^T n - J_A^T n; LIVE nonlinear gap each Newton iter
// (gconGap, defined below). One rank-1 k*b b^T per active contact carries both bodies' self+cross
// blocks (consumed by ipc_gconEnergy in the merit).
typedef struct {
  int bodyA, bodyB, movA, movB;
  mjtNum pA[3], pB[3];       // contact point in each body's local frame (anchors)
  mjtNum xA0[3], xB0[3];     // world anchors at q_n (== con->pos); used live for the static side
  mjtNum n[3];               // frozen unit normal, geom[0]->geom[1]
  mjtNum dist0, k, gtarget;  // gap at detection; penalty stiffness; one-step spring-damper target
                             // (drive gap -> gtarget)
} ipcGcon;

// IPC incremental-potential energy: inertia + edge-stretch penalty + AL contact merit over the
// per-iteration active contact list acon (cached so the line search doesn't re-enumerate all pairs)
static mjtNum ipc_energy(const mjModel* m, const mjData* d, int nfv, const mjtNum* x,
                         const mjtNum* xtil, const int* fidx, const mjtNum* mass, mjtNum h,
                         mjtNum r, const mjtNum* rad, mjtNum ghat, const mjtNum* gv,
                         const mjtNum* ge, const ipcCon* acon, int nacon, const mjtNum* xold,
                         const mjtNum* xfree) {
  mjtNum E = 0, ih2 = 1.0 / (h * h);
  for (int v = 0; v < nfv; v++)
    if (fidx[v] >= 0) {
      mjtNum mh = mass[v] * ih2;
      for (int c = 0; c < 3; c++) {
        mjtNum t = x[3 * v + c] - xtil[3 * v + c];
        E += 0.5 * mh * t * t;
      }
    }
  for (int c = 0; c < nacon; c++) {
    // AL penalty energy E = 0.5*scale*d^2 over the slack-baked d: exact parity with
    // ipc_try's gradient. d = c_raw - s - lam/mu, scale = pow(IPC_DECAY, cnt-exp)*mu_pair. Keep
    // mj's dashpot energy.
    mjtNum mu = ipc_muPair(&acon[c], mass, ih2);
    mjtNum craw = acon[c].ld0 - mjc_conGhat(&acon[c], rad);
    for (int p = 0; p < acon[c].lniv; p++) {
      int v = acon[c].liv[p];
      for (int k = 0; k < 3; k++)
        craw += acon[c].lcw[p] * acon[c].ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    mjtNum dd = craw - acon[c].s - acon[c].lam / mu;
    int cexp = ipc_cntExp(acon[c].cnt);
    mjtNum scale = mu;
    for (int e = 0; e < cexp; e++)
      scale *= IPC_DECAY;
    E += 0.5 * scale * dd * dd;  // AL equality penalty (two-sided; slack carries the inequality)
    mjtNum dn = 0;               // matching one-sided dashpot energy (see ipc_try)
    for (int p = 0; p < acon[c].lniv; p++) {
      int v = acon[c].liv[p];
      for (int k = 0; k < 3; k++)
        dn += acon[c].lcw[p] * acon[c].ln[k] * (x[3 * v + k] - xold[3 * v + k]);
    }
    if (dd <= 0 && dn < 0) {
      mjtNum mmin = 1e30;
      for (int p = 0; p < acon[c].lniv; p++) {
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
static mjtNum ipc_efcCost(const mjModel* m, mjData* d, const mjtNum* xt, const mjtNum* xold,
                          int npt, const int* fidx, const int* dofadr, const int* pbody,
                          const mjtNum* qdt, int na_artic, const int* atid, const int* aoff, int N,
                          mjtNum h, int nefc_qp) {
  int nv = m->nv,
      nefc = nefc_qp;  // score EXACTLY the rows the QP minimized (pure-flex trims native contacts)
  if (!nefc) return 0;
  mjtNum* qacc = (mjtNum*)mju_malloc(
      nv * sizeof(mjtNum));  // mju_malloc like the rest of this file (no engine_io stack here)
  mjtNum* jar = (mjtNum*)mju_malloc(nefc * sizeof(mjtNum));
  mju_copy(qacc, d->qacc_smooth, nv);
  for (int v = 0; v < npt; v++)
    if (fidx[v] >= 0) {
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum dpw[3], dpl[3];
      for (int c = 0; c < 3; c++)
        dpw[c] = xt[3 * v + c] - xold[3 * v + c];
      mju_mulMatTVec3(dpl, R, dpw);
      int da = dofadr[v];
      for (int c = 0; c < 3; c++)
        qacc[da + c] = (dpl[c] / h - d->qvel[da + c]) / h;
    }
  for (int a = 0; a < na_artic; a++) {
    int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
    for (int i = 0; i < nd; i++)
      qacc[da + i] = (qdt[o + i - N] / h - d->qvel[da + i]) / h;
  }
  int savednefc = d->nefc;
  d->nefc = nefc;  // mj_mulJacVec/mj_constraintUpdate read d->nefc internally
  mj_mulJacVec(m, d, jar, qacc);
  mju_subFrom(jar, d->efc_aref, nefc);
  mjtNum cost = 0;
  mj_constraintUpdate(m, d, jar, &cost, 0);
  d->nefc = savednefc;
  mju_free(qacc);
  mju_free(jar);
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

// A tracked GENERAL rigid penalty contact, ANCHORED at q_n, driven by MuJoCo's own collision result
// d->contact. End A = geom[0]'s body, end B = geom[1]'s body; either may be static (welded to
// world). The contact point con->pos is stored as a body-LOCAL anchor in each body (pA, pB). The
// FROZEN normal n (geom[0]->geom[1]) and the FROZEN nv gradient direction b = J_B^T n - J_A^T n
// (mj_jac, stored separately) fix the push-out direction and the Gauss-Newton Hessian; the SCALAR
// gap is recomputed LIVE through full nonlinear FK each Newton iter (the foot cannot tunnel). k =
// m_eff/h^2, m_eff = 1/(b^T M^-1 b) the contact-space effective mass (= the sphere's mass/h^2 as a
// special case, and the reduced mass for body-vs-body). The ipcGcon struct is defined above.

// rigid-rigid gap is LINEARIZED: gap = dist0 + b.(tangent from q_n), b = J^T n frozen at q_n (see
// ipc_gconEnergy + the gradient). This keeps the line-search energy consistent with the Newton
// direction under ROTATION -- the old live-FK gap made them diverge for rotating contacts (frozen b
// != true grad) -> energy injection. Bit-identical to the live gap for translation (slide bodies: b
// IS the true gradient).

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
  for (int i = 0; i < m->nv; i++)
    gdq[i] = 0;
  for (int a = 0; a < na_artic; a++) {
    int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
    for (int i = 0; i < nd; i++)
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
  for (int a = 0; a < na_artic; a++) {
    int t = atid[a], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
    for (int i = 0; i < nd; i++) {
      E += 0.5 * ih2 * gr[da + i] * gMr[da + i] +
           0.5 * ihd * m->dof_damping[da + i] * gdq[da + i] * gdq[da + i];
      gMr[da + i] += hd * m->dof_damping[da + i] * gdq[da + i];
    }
  }
  return E;
}

void mj_IPC(const mjModel* m, mjData* d) {
  // edge-constraint flexes only: FEM elasticity is the CG integrators' domain (the
  // effective-metric machinery); this solver deliberately knows nothing about it
  for (int f = 0; f < m->nflex; f++) {
    if (m->flex_dim[f] >= 2 &&
        ((m->flex_stiffnessadr[f] >= 0 && m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) ||
         m->flex_bendingadr[f] >= 0)) {
      mjERROR(
          "FEM elasticity is not supported under the IPC integrator: "
          "use edge equality constraints, or a CG-solver integrator for FEM (flex %d)",
          f);
    }
  }

  mjtNum h = m->opt.timestep;
  // all dim-2 flexes participate in the IPC solve (was: only the first). Their vertices are
  // concatenated into the free-point array in flex order; fxadr[k] is the free-point offset of
  // dim-2 flex flist[k].
  int nfd = 0;
  for (int i = 0; i < m->nflex; i++)
    if (m->flex_dim[i] == 2) nfd++;
  // nfd==0 (no 2D flex: pure rigid/articulated, e.g. the humanoid) now falls through to the SAME
  // unified path: the appended articulated block carries the generalized tangent and the ported
  // ipcGcon rigid-rigid contact.

  // SOURCE all movable-rigid-involved contacts from MuJoCo's OWN convex collision (geom-geom +
  // flex-vs-geom), snapshotted into a private buffer BEFORE any IPC scratch is allocated. The
  // predictor disables CONTACT for flex scenes (engine_forward.c), so re-run collision with
  // CONTACT|CONSTRAINT enabled at q_n; the flex AL path never reads d->contact, so this can't
  // perturb it (arena-safe: mj_IPC uses mju_malloc only). Mutually-exclusive filter: geom-geom (>=1
  // movable end) and flex-vs-MOVABLE-geom only; flex-flex + flex-static stay on the AL path.
  int saved_df = m->opt.disableflags;
  ((mjModel*)m)->opt.disableflags = saved_df & ~(mjDSBL_CONTACT | mjDSBL_CONSTRAINT);
  mj_collision(m, d);
  mj_makeConstraint(m, d);  // build efc (efc_J/pos/R via mj_makeImpedance) for the snapshot
                            // contacts -- ONCE per step
  mj_referenceConstraint(
      m, d);  // build efc_aref/efc_vel/efc_KBIP -- REUSE MuJoCo's exact reference, no re-derivation
  ((mjModel*)m)->opt.disableflags = saved_df;
  int* flist = (int*)mju_malloc(nfd * sizeof(int));  // the dim-2 flex ids
  int* fxadr = (int*)mju_malloc(nfd * sizeof(int));  // free-point offset of each dim-2 flex
  int nfv = 0;                                       // total dim-2 flex verts (all flexes)
  for (int i = 0, k = 0; i < m->nflex; i++)
    if (m->flex_dim[i] == 2) {
      flist[k] = i;
      fxadr[k] = nfv;
      nfv += m->flex_vertnum[i];
      k++;
    }
  int f =
      (nfd > 0) ? flist[0] : -1;  // flex 0 sets the global barrier scale (ghat) if any flex exists
  mjtNum r = (f >= 0) ? m->flex_radius[f] : 0,
         ghat = r;  // contact activates within ghat of the surface gap
  // free-point -> global flex-vertex index (into flex_vertbodyid / flexvert_xpos), and -> dim-2
  // flex slot k
  int* pt2vg = (int*)mju_malloc((nfv > 0 ? nfv : 1) * sizeof(int));
  int* pt2flex = (int*)mju_malloc((nfv > 0 ? nfv : 1) * sizeof(int));
  for (int k = 0; k < nfd; k++) {
    int va_k = m->flex_vertadr[flist[k]], nv_k = m->flex_vertnum[flist[k]];
    for (int lv = 0; lv < nv_k; lv++) {
      pt2vg[fxadr[k] + lv] = va_k + lv;
      pt2flex[fxadr[k] + lv] = k;
    }
  }

  // FLEX-ONLY point array. The point SoA holds ONLY flex vertices now: npt == nfv. Standalone
  // 3-slide bodies carrying a sphere geom are ordinary appended articulated trees (generalized
  // coords), not free points; the flex fidx assignment (0..nfree_flex-1) and the flex solver
  // arithmetic are the dense flex packing.
  char* isflexvert = (char*)mju_malloc((m->nbody > 0 ? m->nbody : 1) * sizeof(char));
  for (int b = 0; b < m->nbody; b++)
    isflexvert[b] = 0;
  for (int v = 0; v < nfv; v++)
    isflexvert[m->flex_vertbodyid[pt2vg[v]]] = 1;
  int npt = nfv;  // point array is flex-only
  int* dofadr = (int*)mju_malloc((npt > 0 ? npt : 1) * sizeof(int));
  int* qpadr = (int*)mju_malloc((npt > 0 ? npt : 1) *
                                sizeof(int));  // qpos address (NOT dof address: differs after
  int* fidx = (int*)mju_malloc((npt > 0 ? npt : 1) *
                               sizeof(int));  // free/ball joints, which have more qpos than dof)
  mjtNum* mass = (mjtNum*)mju_malloc((npt > 0 ? npt : 1) * sizeof(mjtNum));
  mjtNum* rad =
      (mjtNum*)mju_malloc((npt > 0 ? npt : 1) * sizeof(mjtNum));  // per-point radius (flex_radius)
  int* pbody = (int*)mju_malloc((npt > 0 ? npt : 1) *
                                sizeof(int));  // body id, for the slide-frame rotation R
  int nfree = 0;
  for (int k = 0; k < nfd; k++) {  // flex vertices, per dim-2 flex
    int fi = flist[k];
    mjtNum rk = m->flex_radius[fi];
    int va_k = m->flex_vertadr[fi], nv_k = m->flex_vertnum[fi];
    for (int lv = 0; lv < nv_k; lv++) {
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
  char* isartictree = (char*)mju_malloc((ntree > 0 ? ntree : 1) * sizeof(char));
  for (int t = 0; t < ntree; t++)
    isartictree[t] = 0;
  for (int b = 1; b < m->nbody; b++) {
    if (m->body_dofnum[b] == 0 || isflexvert[b]) continue;
    isartictree[m->dof_treeid[m->body_dofadr[b]]] = 1;
  }
  mju_free(isflexvert);
  int N = 3 * nfree_flex, N_artic = 0, NVTREE = 0;  // N == N_flex: the flex dense packing
  int* aoff = (int*)mju_malloc((ntree > 0 ? ntree : 1) * sizeof(int));
  int* atid =
      (int*)mju_malloc((ntree > 0 ? ntree : 1) * sizeof(int));  // ids of the articulated trees
  for (int t = 0; t < ntree; t++) {
    if (isartictree[t]) {
      aoff[t] = N + N_artic;
      atid[na_artic++] = t;
      N_artic += m->tree_dofnum[t];
      if (m->tree_dofnum[t] > NVTREE) NVTREE = m->tree_dofnum[t];
    } else
      aoff[t] = -1;
  }
  mju_free(isartictree);
  int N_total = N + N_artic,
      Na = (N_total > 0 ? N_total : 1);  // Na sizes the full (flex+appended) solver vector
  // global dof -> appended solver slot (aoff[tree]+localdof), or -1 if its tree is not appended
  // (flex/slide-sphere/ static). Scatters the rigid-rigid contact's full-nv b into grad/p/Hp; only
  // ever indexed where b[i]!=0 (=> >=0). (slider-only simple-body diagonal-M fast solve DEFERRED:
  // body_mass IS the diagonal, but the fast branch diverged from mj_mulM in the flex+balls scene --
  // unexplained; the full mj_mulM/mj_solveM is correct.)
  (void)NVTREE;      // consumed in 4c (mixed-contact side store)
  int nstate = nfv;  // state-vector length (flex points)
  // No FEM element / edge-penalty / bending-flap arrays: stretch rides the native
  // edge-equality efc rows.
  int amax = npt * 64 + 1024;  // capacity of the active-contact list
  ipcCon* acon = (ipcCon*)mju_malloc(amax * sizeof(ipcCon));
  int candmax = npt * 192 + 8192;  // capacity of the per-step candidate list (sized for the
                                   // geom-feature-heavy bag-in-bin contact: ~160k at npt~1100)
  ipcCon* cand = (ipcCon*)mju_malloc(candmax * sizeof(ipcCon));
  mjtNum* cgap =
      (mjtNum*)mju_malloc(candmax * sizeof(mjtNum));  // per-candidate gap at x (try->ccd/E0)
  mjtNum* minc = (mjtNum*)mju_malloc(
      (npt > 0 ? npt : 1) * sizeof(mjtNum));  // per-point min held gap (earliest-collision filter)
  int* held = (int*)mju_malloc(candmax * sizeof(int));  // candidate kept in the inner assembly?
  ipcCon* candLS =
      (ipcCon*)mju_malloc(candmax * sizeof(ipcCon));  // line-search subset (can activate this step)
  // FLEX persistent active set (the persistent active-set manager): maintained ACROSS outer
  // iterations. Each iter: keep existing pairs with abs(cnt)<=IPC_ASET_AGE, ADD new broad-phase
  // candidates with CCD toi<1-1e-6, dedup by pairHash. The assembled set (grad+Hessian in ipc_try,
  // AND ipc_energy) is THIS persistent set -- bounded by the aging eviction, NOT the per-iter
  // broad-phase. aheld is all-1 so the shared held-gated helpers assemble all of it.
  ipcCon* aset = (ipcCon*)mju_malloc(candmax * sizeof(ipcCon));  // persistent active pairs
  mjtNum* agap =
      (mjtNum*)mju_malloc(candmax * sizeof(mjtNum));  // per-aset-pair maintained gap lower bound
  int* aheld =
      (int*)mju_malloc(candmax * sizeof(int));  // all-1 mask for the held-gated assemble/energy
  ipcCon* amerge =
      (ipcCon*)mju_malloc(candmax * sizeof(ipcCon));  // merge scratch (new merged set built here)
  int naset = 0;
  for (int c = 0; c < candmax; c++)
    aheld[c] = 1;
  // Legacy active-set scratch. The current scheme replaces the old per-point gamma carrier with the
  // per-PAIR cnt state machine (g_cntKey/Val store; cnt rides in ipcCon.cnt). gam is unused now;
  // appr is filled by mjc_advance; actc/actpt are now unused but kept allocated to avoid churning the
  // frees.
  mjtNum* gam = (mjtNum*)mju_malloc(candmax * sizeof(mjtNum));
  int* actc = (int*)mju_malloc(candmax * sizeof(int));
  int* appr = (int*)mju_malloc(candmax * sizeof(int));
  int* actpt = (int*)mju_malloc((npt > 0 ? npt : 1) * sizeof(int));
  for (int c = 0; c < candmax; c++) {
    gam[c] = 1.0;
    actc[c] = 0;
    appr[c] = 0;
  }
  // precompute static-geom sharp features (vertices/edges) once per step (geoms are fixed here).
  // box -> 8 verts/12 edges; mesh -> all verts / all hull-poly edges. Cap = sum over colliding
  // geoms.
  int gvcap = 1, gecap = 1;
  for (int gi = 0; gi < m->ngeom; gi++) {
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
      for (int p = 0; p < pn; p++)
        gecap += m->mesh_polyvertnum[pa + p];  // upper bound (pre-dedup)
    }
  }
  int ngv = 0, nge = 0;
  mjtNum* gv = (mjtNum*)mju_malloc(3 * gvcap * sizeof(mjtNum));
  mjtNum* ge = (mjtNum*)mju_malloc(6 * gecap * sizeof(mjtNum));
  for (int gi = 0; gi < m->ngeom; gi++) {
    if (m->geom_contype[gi] == 0 && m->geom_conaffinity[gi] == 0) continue;  // skip non-colliding
    if (m->body_weldid[m->geom_bodyid[gi]] != 0)
      continue;  // STATIC geoms only (matches the type-2/3/4 barrier filter)
    ngv += mjc_GeomVerts(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, gv + 3 * ngv);
    nge += mjc_GeomEdges(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, ge + 6 * nge);
  }
  // state vectors are sized 3*nstate (flex points 0..nfv-1), indexed by their state slot.
  mjtNum* x = (mjtNum*)mju_malloc(3 * nstate * sizeof(mjtNum));
  mjtNum* xfree = (mjtNum*)mju_malloc(
      3 * nstate * sizeof(mjtNum));  // AL two-state: intersection-free output path (paper x[k])
  mjtNum* xtil = (mjtNum*)mju_malloc(3 * nstate * sizeof(mjtNum));
  mjtNum* xold = (mjtNum*)mju_malloc(3 * nstate * sizeof(mjtNum));
  mjtNum* xn = (mjtNum*)mju_malloc(3 * nstate * sizeof(mjtNum));
  mjtNum* dx = (mjtNum*)mju_malloc(Na * sizeof(mjtNum));

  // flex/sphere inertial prediction: q~ = q + h*v + h^2*qacc_smooth (point masses have no
  // Coriolis).
  mjtNum* qacc_pred = (mjtNum*)mju_malloc((m->nv > 0 ? m->nv : 1) * sizeof(mjtNum));
  mju_copy(qacc_pred, d->qacc_smooth, m->nv);

  // STEP 4b: appended ARTICULATED kinetic state. qn_a = q_n; qtil_a = q_n (+) h*(v + h*qacc_smooth)
  // the free-flight predictor in generalized coords (mirrors mj_ipcTree); qdelta = accumulated
  // tangent from q_n (length N_artic). gradient/energy scratch (a_gdq/gr/gMr) are nv-sized; q*_a
  // are nq. All sized to 1 when na_artic==0 so the flex/bag path pays nothing (and stays
  // byte-identical).
  int nva = (na_artic ? m->nv : 1), nqa = (na_artic ? m->nq : 1),
      Nart = (N_artic > 0 ? N_artic : 1);
  mjtNum* qn_a = (mjtNum*)mju_malloc(nqa * sizeof(mjtNum));
  mjtNum* qtil_a = (mjtNum*)mju_malloc(nqa * sizeof(mjtNum));
  mjtNum* qcur_a = (mjtNum*)mju_malloc(nqa * sizeof(mjtNum));
  mjtNum* qdelta = (mjtNum*)mju_malloc(Nart * sizeof(mjtNum));
  mjtNum* qdtmp = (mjtNum*)mju_malloc(Nart * sizeof(mjtNum));
  mjtNum* a_gdq = (mjtNum*)mju_malloc(nva * sizeof(mjtNum));
  mjtNum* a_gr = (mjtNum*)mju_malloc(nva * sizeof(mjtNum));
  mjtNum* a_gMr = (mjtNum*)mju_malloc(nva * sizeof(mjtNum));
  for (int i = 0; i < N_artic; i++)
    qdelta[i] = 0;
  if (na_artic) {
    mju_copy(qn_a, d->qpos, m->nq);  // q_n (d->qpos is at q_n throughout the flex solve)
    for (int i = 0; i < m->nv; i++)
      a_gr[i] = m->dof_damping[i] *
                d->qvel[i];  // IMPLICIT joint damping: qacc_smooth has -M^-1 D v (explicit);
    mj_solveM(m, d, a_gMr, a_gr,
              1);  // add it BACK so the predictor is UNDAMPED -- the damping is now
    for (int i = 0; i < m->nv; i++)
      a_gdq[i] = d->qvel[i] + h * qacc_pred[i] +
                 h * a_gMr[i];  // applied implicitly (M_eff) in the Newton solve
    mju_copy(qtil_a, qn_a, m->nq);
    mj_integratePos(m, qtil_a, a_gdq, h);  // q~ = q_n (+) h*(v + h*qacc_smooth)
  }

  const mjtNum* vx = d->flexvert_xpos;
  for (int v = 0; v < npt; v++) {
    // xold position source: flex vertex from flexvert_xpos (the point array is flex-only now)
    for (int c = 0; c < 3; c++)
      xold[3 * v + c] = vx[3 * pt2vg[v] + c];
    if (fidx[v] >= 0) {
      int da = dofadr[v];
      // qvel/qacc_smooth are in the body's local slide frame; map to WORLD via the body rotation R
      // (= d->xmat). R=I for unrotated bodies (cloth, the balls); non-identity for the rotated bag.
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum vw[3], aw[3];
      mju_mulMatVec3(vw, R, d->qvel + da);
      mju_mulMatVec3(aw, R, qacc_pred + da);
      for (int c = 0; c < 3; c++)
        xtil[3 * v + c] = xold[3 * v + c] + h * vw[c] + h * h * aw[c];
    } else {
      for (int c = 0; c < 3; c++)
        xtil[3 * v + c] = xold[3 * v + c];  // pinned: fixed
    }
  }
  for (int i = 0; i < 3 * nstate; i++)
    x[i] = xold[i];  // start from last collision-free state (feasibility)

  mjtNum ih2 = 1.0 / (h * h);
  {  // AL: per-free-point flex/sphere contact-multiplier warm-start store (persisted across steps).
    if (g_palN != npt) {
      mju_free(g_pal);
      g_palN = npt;
      g_pal = (mjtNum*)mju_malloc((size_t)(npt > 0 ? npt : 1) * sizeof(mjtNum));
      for (int i = 0; i < npt; i++)
        g_pal[i] = 0;
    }  // zero ONLY on resize -> warm-start persists
  }
  // build the candidate-contact list once per step: detection threshold inflated by the predictor
  // displacement so any pair that could close during the step is captured (verified by gap checks)
  mjtNum maxdisp = 0;
  for (int v = 0; v < npt; v++)
    if (fidx[v] >= 0) {
      mjtNum dd[3];
      for (int c = 0; c < 3; c++)
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
  ipc_cntStepBegin(
      candmax);  // swap+clear the per-pair cnt store for this step (OLD = last step's final cnts)
  int ncand = mjc_candidates(m, d, x, gv, ge, ngv, nge, r, rad, thresh, threshGeom, maxdisp, xold,
                             xtil, ghat, nfv, npt, fidx, flist, fxadr, nfd, pt2flex, cand, candmax);
  for (int c = 0; c < ncand;
       c++) {  // AL: warm-start each fresh candidate's multiplier from g_pal + cnt from the store
    int vv[4], nvv;
    mjc_conVerts(&cand[c], vv, &nvv);  // (binding = max over its free-point participants)
    mjtNum s = 0;
    for (int q = 0; q < nvv; q++)
      if (g_pal[vv[q]] > s) s = g_pal[vv[q]];
    cand[c].lam = s;
    cand[c].cnt = ipc_cntGet(ipc_pairHash(&cand[c]));
    cand[c].s = 0;
    gam[c] = 1.0;
  }
  // warm start: with no candidate contacts within thresh, the predictor x~ is collision-free (the
  // thresh margin covers the step displacement), so it is a far better feasible initial guess than
  // xold and Newton converges in ~1 iteration instead of ~2 -- halving the cost of contact-free
  // steps.
  if (ncand == 0)
    for (int i = 0; i < 3 * nstate; i++)
      x[i] = xtil[i];
  // mark active inter-flex/sphere candidates (gap already within their per-contact ghat). gap here
  // == iter-0 gap (x is unchanged until the Newton loop); geom contacts (types 2/3/4) add no
  // coupling so they're skipped.
  for (int c = 0; c < ncand; c++) {
    if (cand[c].type >= 2 && cand[c].type != 5) continue;
    mjtNum nn[3], cw[4];
    int iv[4], nidx;
    cgap[c] = mjc_conGap(&cand[c], m, d, x, gv, ge, r, rad, nn, iv, cw, &nidx, thresh);
  }
  // FLEX: SEED the persistent active set from the iter-0 broad-phase. addCand already pruned
  // cand to closing-or-distance-active pairs (per-pair closing-bound prune), so every iter-0
  // candidate is a valid initial active pair (seeding the active set from the first
  // discrete-collision-detection pass). lam/cnt were warm-started above. Aging + the per-iter
  // CCD-toi merge then maintain it (bounded, not holdall).
  naset = (ncand < candmax) ? ncand : candmax;
  for (int c = 0; c < naset; c++) {
    aset[c] = cand[c];
    agap[c] = cgap[c];
  }
  // active-set re-test: after the first Newton iter, only candidates with gap < ghat are re-tested
  // for the barrier (the contact interface). cgap is kept a valid LOWER BOUND for the rest by
  // decrementing it each iter by delta = 4*alpha*max|dx| (a bound on how fast any gap can shrink),
  // and refreshing it exactly when a candidate enters the set. So the lower bound crosses ghat no
  // later than the true gap (no active contact is ever missed in the barrier), and the CCD's
  // cgap-based step cap stays conservative (safe).
  for (int i = 0; i < 3 * nstate; i++)
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
  mjtNum last_maxdx =
      1e30;  // last inner Newton-direction magnitude, exposed to the outer early-out
  mjtNum last_ls_alpha =
      1.0;  // accepted line-search step of the last Newton iter; gates the outer termination
  int nefc_qp = 0;  // [ARCH-2] nefc as the QP saw it (post-trim); the merit's efc cost scores
                    // exactly these rows
  for (int outer = 0; outer < outer_cap && N_total > 0;
       outer++) {  // OUTER loop (single AL loop; paper Alg.1). N_total (not N): the appended
                   // articulated block must run even with no flex packing (humanoid: N==0,
                   // N_total==nv)
    // WORKING SET: the PERSISTENT active set aset/agap/aheld/naset maintained across outer iters by
    // ipc_mergeActiveSet (the persistent active-set manager). aheld is all-1:
    // assemble/energy/slack/lambda run over the ENTIRE persistent set (the aging eviction -- not a
    // per-iter ld0<ghat test -- bounds it).
    ipcCon* wcon = aset;
    mjtNum* wgap = agap;
    int* wheld = aheld;
    int wn = naset;
    // N1 linearize_constraints (at xfree, Eq.10): ld0/ln/lcw/liv this iter -> c(x) is linear in x.
    for (int c = 0; c < wn; c++)
      wcon[c].ld0 = mjc_conGap(&wcon[c], m, d, xfree, gv, ge, r, rad, wcon[c].ln, wcon[c].liv,
                               wcon[c].lcw, &wcon[c].lniv, ghat);
    // x PERSISTS across outer iters (libuipc/paper warm-start) -- NOT reset to xfree. The old
    // restart-from-xfree was a contact-set-explosion NaN firewall that ALSO prevented the primal
    // Newton from ever converging (one cold step per outer iter); with the faithful active-set +
    // the fixed broad-phase collar the NaN no longer fires, so persisting x lets the warm-started
    // Newton actually solve the equation of motion, which the restart was blocking.
    (void)minc;
    (void)appr;
    (void)gam;
    (void)actc;
    // N2 update_slack (at x): materialize s[c] and the slack-baked d the assemble (ipc_try) /
    // lambda use.
    ipc_updateSlack(wcon, wn, wheld, x, xfree, rad, ghat, mass, ih2);
    int flex_converged_out = 0,
        artic_converged_out = 0;  // per-block convergence, carried to the outer termination
    for (int it = 0; it < inner_cap && N_total > 0; it++) {
      // (The solveU-era gradient/Hessian assembly -- inertia, FEM stretch, bending, edge, soft,
      // gcon -- is gone: the QP computes the direction and the N7 merit scores the energy; only the
      // contact ASSEMBLY below remains, because it feeds the injected rows (acon) and maintains the
      // wgap lower bounds for the active-set logic.) assemble active contacts. First Newton iter:
      // scan all candidates (initializes cgap exactly). Later iters: only re-test the active set
      // {cgap < ghat} (cgap is a maintained lower bound, so this set contains every candidate that
      // is or could be active). acon gets the admitted pairs (-> the injected rows).
      int nacon = 0;
      if (it == 0) {
        for (int c = 0; c < wn; c++)
          if (wheld[c]) ipc_try(wcon[c], x, xfree, rad, ghat, acon, &nacon, amax, &wgap[c]);
      } else {
        for (int c = 0; c < wn; c++)
          if (wheld[c] && wgap[c] < ghat)
            ipc_try(wcon[c], x, xfree, rad, ghat, acon, &nacon, amax, &wgap[c]);
      }
      // MuJoCo-solver injection (always on): swap ONLY the inner linear solve for MuJoCo's sparse
      // CG, KEEPING the AL/CCD outer loop (N8 dual-ascent + CCD stay in charge ->
      // penetration-free). Inject ALL barrier contacts (acon: flex-flex + flex-sphere +
      // flex-static, no lniv filter) as extra-primal rows; native efc keeps the EDGES (equality)
      // but we DROP its contact rows (nefc -> ne+nf+nl) so they don't double-count our injection.
      // Then the converged qacc -> world x, and N8 + the next outer's re-linearize run normally on
      // it. pure-flex: inject ALL barrier contacts + drop native. pure-rigid: keep native. MIXED:
      // inject only flex-flex/self, keep native (rigid + flex-humanoid). MuJoCo's converged qacc is
      // used as the search DIRECTION for the shared N7 monotone-energy line search (below) -- a
      // converged step is big and needs the line search to stay stable.
      {
        int mixed = (na_artic > 0 && nfv > 0);
        int cap = 2 * nacon + 1;  // contact stiffness + dashpot rows
        int* epnnz = (int*)mju_malloc(cap * sizeof(int));
        int* epadr = (int*)mju_malloc(cap * sizeof(int));
        int* epcol = (int*)mju_malloc(cap * 12 * sizeof(int));
        mjtNum* epval = (mjtNum*)mju_malloc(cap * 12 * sizeof(mjtNum));
        mjtNum* epref = (mjtNum*)mju_malloc(cap * sizeof(mjtNum));
        mjtNum* epD = (mjtNum*)mju_malloc(cap * sizeof(mjtNum));
        int* epone = (int*)mju_malloc(cap * sizeof(int));
        int nrow = 0, nnz = 0;
        mjtNum h2 = 1.0 / ih2;
        for (int c = 0; c < nacon; c++) {  // pure-flex: ALL barrier contacts (the flex CCD
          ipcCon* con = &acon[c];  // tracks all -> all injected); mixed: only flex-flex/self
          if (mixed) {
            int othr = (con->type == 0)
                           ? con->idx[1]
                           : con->idx[2];  // mixed: inject only flex-flex/self contacts
            if (!(con->type <= 1 && con->idx[0] < nfv && othr < nfv)) continue;
          }
          mjtNum mu = ipc_muPair(con, mass, ih2);
          int cexp = ipc_cntExp(con->cnt);
          mjtNum D = mu;
          for (int e = 0; e < cexp; e++)
            D *= IPC_DECAY;  // scale = mu*DECAY^cnt (the GN stiffness)
          mjtNum delta = mjc_conGhat(con, rad);
          mjtNum refc = -(con->ld0 - delta) + con->s + con->lam / mu;
          epadr[nrow] = nnz;
          // This reference is EXACT: r(qacc(x)) = dd(x) algebraically (the h*Rqv terms cancel per
          // vertex against J*qacc; a h^2*qacc_smooth term would be spurious -- it cancels across
          // all-free pairs anyway since the weights sum to 0 and gravity is uniform, and adding it
          // costs deep backtracks on pin-adjacent pairs).
          for (int p = 0; p < con->lniv; p++) {
            int v = con->liv[p];
            if (dofadr[v] < 0)
              continue;  // PINNED vertex: no dofs. Without this guard the row read qvel[-1]/
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
            for (int k = 0; k < 3; k++) {
              epcol[nnz] = dofadr[v] + k;
              epval[nnz] = con->lcw[p] * Rtn[k] * h2;
              nnz++;
              refc -= con->lcw[p] * con->ln[k] * (xold[3 * v + k] + h * Rqv[k] - xfree[3 * v + k]);
            }
          }
          epnnz[nrow] = nnz - epadr[nrow];
          epref[nrow] = refc;
          // D = mu/h^2: MuJoCo's inertia is in ACCELERATION (1/h^2 heavier than ipc's position
          // inertia), so the injected contact stiffness must be scaled by ih2 to match the
          // contact/inertia RATIO (M2 only matched the energy).
          epD[nrow] = D * ih2;
          epone[nrow] =
              0;  // flex-flex is a PENALTY like the old path (ipc applies scale*dd to ALL held) ->
                  // TWO-SIDED; the slack/multiplier update carries the one-sidedness across outers
          nrow++;
        }
        mjExtraPrimal ep = {nrow, epnnz, epadr, epcol, epval, epref, epD, epone};
        int saved = m->opt.disableflags;
        int savedsol = m->opt.solver;
        int savednefc = d->nefc;
        if (na_artic == 0 && nfv > 0)
          d->nefc = d->ne + d->nf +
                    d->nl;  // pure-FLEX: drop native contacts (injection+CCD replace them);
        nefc_qp = d->nefc;  // [ARCH-2] the rows the QP actually minimizes -- the merit's efc cost
                            // scores exactly these
        int dflags = saved | mjDSBL_ISLAND;  // pure-RIGID + MIXED keep native contacts (rigid +
                                             // flex-humanoid, w/ friction)
        ((mjModel*)m)->opt.disableflags = dflags;
        // The injected penalty's cost/gradient/line-search curvature all live on the SHARED primal
        // path, so MuJoCo's matrix-free nonlinear CG solves the AL contact subproblem with NO
        // factorization -- it cannot hit the ill-conditioning that makes a direct Newton factor go
        // rank-deficient on the humanoid (stiff contact vs light flex mass, cond ~2e5). CG's exact
        // line search already carries the contact curvature Sum D*(val.p)^2, so each iterate's step
        // accounts for the contact even with M^-1 preconditioning (the paper's PCG route) -- robust
        // (no factorization to fail) and ~30-70x faster than the direct factor.
        // Default: matrix-free PCG Newton direction (mjSOL_NEWTON -> flg_pcg in mj_solPrimal): the
        // injected contact enters the H*p operator, solved with linear PCG (no factorization, no
        // stagnation). IPC_CG = old nonlinear CG. NONLINEAR CG inner solve (Polak-Ribiere, exact
        // line search; the injected penalty's cost/gradient/ line-search curvature are on the
        // shared primal path). The outer AL converges at the same rate with a loosely solved
        // subproblem, so a fully converged Newton direction per outer is wasted work; the iteration
        // cap is the model's opt.iterations. Tightening the cap trades speed against occasional
        // deep line-search backtracks.
        ((mjModel*)m)->opt.solver = mjSOL_CG;
        mj_setExtraPrimal(&ep);
        mj_fwdConstraint(
            m, d);  // efc_b + warmstart + mj_solCG (Gauss + efc + injected penalty contacts)
        mj_setExtraPrimal(NULL);
        d->nefc = savednefc;
        ((mjModel*)m)->opt.disableflags = saved;
        ((mjModel*)m)->opt.solver = savedsol;
        // Barrier-free AL contact energy is FINITE, so a CONVERGED inner step is big and MUST be
        // line-searched: converge+no-line-search explodes (step 145), converge+line-search is
        // stable (morph ladder on scene_ipc_balls_edge2, 2026-07-04). Treat MuJoCo's converged
        // solution as the search DIRECTION dx (free-dof [0,N) + articulated [N,N_total)); the
        // shared N7 monotone-energy line search below picks the step magnitude and commits
        // x/qdelta.
        for (int v = 0; v < npt; v++)
          if (fidx[v] >= 0) {
            int fi = fidx[v];
            const mjtNum* R = d->xmat + 9 * pbody[v];
            const mjtNum* qv = d->qvel + dofadr[v];
            const mjtNum* qa = d->qacc + dofadr[v];
            mjtNum Rqv[3], Rqa[3];
            mju_mulMatVec3(Rqv, R, qv);
            mju_mulMatVec3(Rqa, R, qa);
            for (int k = 0; k < 3; k++) {
              mjtNum xf = xold[3 * v + k] + h * Rqv[k] + h2 * Rqa[k];
              dx[3 * fi + k] = xf - x[3 * v + k];
            }  // search DIRECTION to MuJoCo's solution (NOT a commit)
          }
        for (int a = 0; a < na_artic; a++) {
          int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
          for (int i = 0; i < nd; i++)
            dx[N + (o - N + i)] = (h * d->qvel[da + i] + h2 * d->qacc[da + i]) - qdelta[o - N + i];
        }
        mju_free(epnnz);
        mju_free(epadr);
        mju_free(epcol);
        mju_free(epval);
        mju_free(epref);
        mju_free(epD);
        mju_free(epone);
      }
      if (na_artic) {  // FK the articulated geoms at q_n (+) qdelta so type-3 soft contacts read
                       // live poses (qM/qLD
        for (int i = 0; i < m->nv; i++)
          a_gdq[i] = 0;  // stay frozen at q_n -> mj_mulM/mj_solveM unaffected)
        for (int a = 0; a < na_artic; a++) {
          int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
          for (int i = 0; i < nd; i++)
            a_gdq[da + i] = qdelta[o + i - N];
        }
        mju_copy(d->qpos, qn_a, m->nq);
        mj_integratePos(m, d->qpos, a_gdq, 1);
        mj_kinematics(m, d);
      }
      // dx is MuJoCo's converged search direction (from the injection block above); fall through to
      // the N7 line search.
      mjtNum maxdx =
          0;  // max free-vertex displacement of the Newton direction (for the lower-bound decay)
      for (int v = 0; v < N / 3; v++) {
        mjtNum n2 =
            dx[3 * v] * dx[3 * v] + dx[3 * v + 1] * dx[3 * v + 1] + dx[3 * v + 2] * dx[3 * v + 2];
        if (n2 > maxdx) maxdx = n2;
      }
      maxdx = sqrt(maxdx);
      last_maxdx = maxdx;  // expose to the outer early-out (system-at-rest test)
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
        for (int i = 0; i < N; i++) {
          mjtNum a = dx[i] < 0 ? -dx[i] : dx[i];
          if (a > maxf) maxf = a;
        }
        for (int i = N; i < N_total; i++) {
          mjtNum a = dx[i] < 0 ? -dx[i] : dx[i];
          if (a > maxa) maxa = a;
        }
        flex_converged_out = (maxf <= IPC_VEL_TOL * h);   // vacuously 1 when N==0 (pure rigid)
        artic_converged_out = (maxa <= IPC_VEL_TOL * h);  // vacuously 1 when N_artic==0 (pure flex)
        newton_converged = flex_converged_out && artic_converged_out;
      }
      // line-search subset: a candidate can contribute to the barrier at xn = x + alpha*dx (alpha
      // in [0,cap], cap<=1) only if its gap can drop below ghat. Its gap drop over a full step is
      // bounded by the sum of |dx| over its flex vertices, so keep only c with cgap[c] < ghat +
      // that bound.
      int nls = 0;
      for (int c = 0; c < wn; c++) {
        int vv[4], nvv;
        mjc_conVerts(&wcon[c], vv, &nvv);
        mjtNum lub = 0;
        for (int q = 0; q < nvv; q++) {
          int fq = fidx[vv[q]];
          if (fq < 0) continue;
          lub += sqrt(dx[3 * fq] * dx[3 * fq] + dx[3 * fq + 1] * dx[3 * fq + 1] +
                      dx[3 * fq + 2] * dx[3 * fq + 2]);
        }
        if (wheld[c] && wgap[c] < ghat + lub)
          candLS[nls++] = wcon[c];  // assembled-set parity with the Newton Hessian
      }
      // [ARCH-2] N7 line search: the merit scores EXACTLY the QP objective with live flex-flex gaps
      // -- Gauss/inertia (ipc_energy inertia + ipc_articResid) + MuJoCo's own efc cost
      // (ipc_efcCost: edges, limits, rigid-rigid and rigid-flex contacts, no hand-replicated
      // scorers) + the AL flex-flex penalty (live gaps, the per-outer relinearized nonlinearity the
      // line search exists for). The line search itself is required: the paper line-searches the
      // nonlinear subproblem energy; accepting the linearized full step overshoots into penetration
      // and the N8 CCD advance collapses (beta grinds at the outer cap).
      mjtNum E0 = ipc_energy(m, d, npt, x, xtil, fidx, mass, h, r, rad, ghat, gv, ge, candLS, nls,
                             xold, xfree) +
                  ipc_articResid(m, d, qn_a, qtil_a, qdelta, na_artic, atid, aoff, N, ih2, a_gdq,
                                 qcur_a, a_gr, a_gMr) +
                  ipc_efcCost(m, d, x, xold, npt, fidx, dofadr, pbody, qdelta, na_artic, atid, aoff,
                              N, h, nefc_qp);
      mjtNum alpha = 1.0;
      // Line search: plain monotone decrease + 1e-12 slop, OR newton_converged short-circuit; 8
      // backtracks /2; ALWAYS accept the final trial (no Armijo-break grind).
      for (int ls = 0; ls < 8; ls++) {
        for (int i = 0; i < 3 * nstate; i++)
          xn[i] = x[i];
        for (int v = 0; v < npt; v++)
          if (fidx[v] >= 0) {
            int fi = fidx[v];
            for (int c = 0; c < 3; c++)
              xn[3 * v + c] = x[3 * v + c] + alpha * dx[3 * fi + c];
          }
        for (int j = 0; j < N_artic; j++)
          qdtmp[j] = qdelta[j] + alpha * dx[N + j];  // articulated trial tangent
        if (na_artic) {  // FK at the trial articulated config (next-iter gcon/soft reads live
                         // poses; qM/qLD stay @ q_n)
          for (int i = 0; i < m->nv; i++)
            a_gdq[i] = 0;
          for (int a = 0; a < na_artic; a++) {
            int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
            for (int i = 0; i < nd; i++)
              a_gdq[da + i] = qdtmp[o + i - N];
          }
          mju_copy(d->qpos, qn_a, m->nq);
          mj_integratePos(m, d->qpos, a_gdq, 1);
          mj_kinematics(m, d);
        }
        mjtNum Etr = ipc_energy(m, d, npt, xn, xtil, fidx, mass, h, r, rad, ghat, gv, ge, candLS,
                                nls, xold, xfree) +
                     ipc_articResid(m, d, qn_a, qtil_a, qdtmp, na_artic, atid, aoff, N, ih2, a_gdq,
                                    qcur_a, a_gr, a_gMr) +
                     ipc_efcCost(m, d, xn, xold, npt, fidx, dofadr, pbody, qdtmp, na_artic, atid,
                                 aoff, N, h, nefc_qp);
        if (Etr <= E0 + 1e-12 || newton_converged) break;
        alpha *= 0.5;
      }
      for (int i = 0; i < 3 * nstate; i++)
        x[i] = xn[i];  // always accept the final trial
      for (int j = 0; j < N_artic; j++)
        qdelta[j] = qdtmp[j];  // commit the accepted articulated tangent
      last_ls_alpha = alpha;  // accepted global line-search step (the outer loop terminates only on
                              // a full step)
      // keep wgap a valid lower bound: every gap can shrink by at most 4 vertices * the max vertex
      // step
      mjtNum dgap = 4.0 * alpha * maxdx;
      for (int c = 0; c < wn; c++)
        wgap[c] -= dgap;
    }
    // N8 non-penetration advance (every iter): dual ascent (lambda update + cnt) -> re-query@xfree
    // -> CCD -> advance xfree. Order is FIXED:
    //   lambda update (on the persistent set) -> prepare CCD -> detect trajectory candidates(1.0)
    //   -> alpha = CCD time-of-impact filter(1.0) -> active-set update (MERGE/AGING) -> advance
    //   non-penetrate positions(alpha)
    //   -> beta = beta + (1-beta)*alpha.
    // N8a lambda update + cnt on the ASSEMBLED persistent set (aset), then sink lam -> g_pal (warm
    // start).
    ipc_flexLamUpdate(m, d, x, xfree, gv, ge, r, rad, ghat, mass, ih2, aset, naset, aheld, g_pal,
                      npt);
    for (int c = 0; c < naset; c++)
      ipc_cntSet(ipc_pairHash(&aset[c]), aset[c].cnt);  // persist cnt across steps
    // N8b prepare CCD: disp = x - xfree (free-dof layout), base = xfree.
    for (int v = 0; v < npt; v++)
      if (fidx[v] >= 0)
        for (int c = 0; c < 3; c++)
          dx[3 * fidx[v] + c] = x[3 * v + c] - xfree[3 * v + c];
    // N8c detect trajectory candidates(1.0): re-query the broad-phase at xfree over the swept
    // segment xfree->x. FIXED collar (3*ghat, maxdisp=0, d_hat expansion + thickness): the swept
    // segment covers gross motion (no-tunnel), and maxdisp=0 keeps the count bounded regardless of
    // |x-xfree| (the old displacement-scaled collar ballooned to 238k candidates when x flung ->
    // NaN). This is the NEW-candidate source for the merge.
    ncand = mjc_candidates(m, d, xfree, gv, ge, ngv, nge, r, rad, 3 * ghat, 3 * ghat, 0.0, xfree, x,
                           ghat, nfv, npt, fidx, flist, fxadr, nfd, pt2flex, cand, candmax);
    for (int c = 0; c < ncand; c++) {
      mjtNum nn[3], cw[4];
      int idv[4], ni;  // gaps at xfree (for CCD + admission)
      cgap[c] = mjc_conGap(&cand[c], m, d, xfree, gv, ge, r, rad, nn, idv, cw, &ni, ghat);
    }
    // N8d CCD time-of-impact filter(1.0): CCD over the trajectory candidates -> the advance alpha.
    // appr[c] flags each candidate whose full step closes its gap into the active zone (== its
    // individual CCD time-of-impact < 1).
    mjtNum ac =
        mjc_advance(m, d, xfree, dx, gv, ge, r, rad, nfv, fidx, cand, ncand, cgap, pt2flex, appr);
    // N8e update_active_set: MERGE the admitted broad-phase candidates into the persistent set
    // (keep existing with abs(cnt)<=25, add new with toi<1-1e-6, dedup). Admit a candidate iff it
    // is closing this step (appr) OR already inside its skin (midsurface gap <= the pair's rest
    // separation r1+r2): compressed pairs are force-carrying. New entries seed lam(g_pal)/cnt(store).
    for (int c = 0; c < ncand; c++)
      actc[c] = (appr[c] || cgap[c] <= mjc_conGhat(&cand[c], rad)) ? 1 : 0;
    ipc_mergeActiveSet(aset, &naset, cand, ncand, actc, amerge, g_pal, candmax);
    // N8f advance non-penetrate positions(alpha): advance the FLEX xfree, iff alpha > alpha lower
    // bound. beta -> 1.
    if (ac > IPC_ALPHA_LB)
      for (int i = 0; i < 3 * npt; i++)
        xfree[i] = (1.0 - ac) * xfree[i] + ac * x[i];
    beta = beta + (1.0 - beta) * ac;
    // terminate: beta COMPLETE AND the articulated block truly converged AND the flex
    // block converged-or-full-step. Contact-type-aware: rigid (gcon/explicit-aref) contact NEEDS
    // convergence (energy injection otherwise); AL flex contact is stable at the alpha==1 bail and
    // forcing it is ~100x outers. A complete advance is required to commit: beta is the fraction of
    // the step's motion the committed positions have absorbed, not a residual -- accepting beta<1
    // discards (1-beta) of the motion while time still advances by h (time-dilated, slow-motion
    // physics; sustained beta~0.9 commits in contact-rich phases read as a visible freeze). Partial
    // advances are fine WITHIN the loop; the stall path below is the loud escape when CCD cannot
    // complete.
    if (beta >= 1.0 - 1e-6 && artic_converged_out &&
        (flex_converged_out || last_ls_alpha >= 1.0 - 1e-9))
      break;
    if (ac > IPC_ALPHA_LB)
      stall = 0;
    else if (++stall >= IPC_STALL_MAX) {
      stalled = 1;
      break;
    }  // CCD froze -> stop grinding, error below
    (void)last_maxdx;
  }  // OUTER loop close
  for (int i = 0; i < 3 * nstate; i++)
    x[i] = xfree[i];  // commit the intersection-free output (readback uses x)
  for (int v = 0; v < npt; v++)
    if (fidx[v] >= 0) {
      int da = dofadr[v];
      int qa = qpadr[v];  // qpos by joint qposadr; qvel/accel by dof address
      // x/xold are world; the slide dofs are in the body-local frame -> map the world displacement
      // back through R^T (= d->xmat^T). For an unrotated body this is the identity (dp_local ==
      // dp_world).
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum dpw[3], dpl[3];
      for (int c = 0; c < 3; c++)
        dpw[c] = x[3 * v + c] - xold[3 * v + c];
      mju_mulMatTVec3(dpl, R, dpw);
      for (int c = 0; c < 3; c++) {
        d->qvel[da + c] = dpl[c] / h;
        d->qpos[qa + c] += dpl[c];
      }
    }
  // ARTICULATED readback: qdelta = accumulated generalized tangent from q_n. Commit q_{n+1}=q_n (+)
  // qdelta on each appended tree's qpos (per joint) and v=qdelta/h on its dofs (mirrors mj_ipcTree;
  // NO R^T -- generalized coords).
  if (na_artic) {
    for (int i = 0; i < m->nv; i++)
      a_gdq[i] = 0;
    for (int a = 0; a < na_artic; a++) {
      int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
      for (int i = 0; i < nd; i++)
        a_gdq[da + i] = qdelta[o + i - N];
    }
    mju_copy(qcur_a, qn_a, m->nq);
    mj_integratePos(m, qcur_a, a_gdq, 1);  // q_{n+1} = q_n (+) qdelta
    for (int a = 0; a < na_artic; a++) {
      int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
      for (int bi = m->tree_bodyadr[t]; bi < m->tree_bodyadr[t] + m->tree_bodynum[t]; bi++)
        for (int j = m->body_jntadr[bi]; j < m->body_jntadr[bi] + m->body_jntnum[bi]; j++) {
          int qa = m->jnt_qposadr[j], nqj = (m->jnt_type[j] == mjJNT_FREE   ? 7
                                             : m->jnt_type[j] == mjJNT_BALL ? 4
                                                                            : 1);
          for (int k = 0; k < nqj; k++)
            d->qpos[qa + k] = qcur_a[qa + k];
        }
      for (int i = 0; i < nd; i++)
        d->qvel[da + i] = qdelta[o + i - N] / h;
    }
    // pure-articulated (no-flex) path: refresh FK + COM/cinert at q_{n+1} for downstream (the next
    // predictor's qacc_smooth + sensors), mirroring the old mj_ipcTree. Gated nfd==0 so the flex
    // bag stays byte-identical.
    if (nfd == 0) {
      mj_kinematics(m, d);
      mj_comPos(m, d);
    }
  }
  d->time += h;
  mju_free(dofadr);
  mju_free(qpadr);
  mju_free(fidx);
  mju_free(mass);
  mju_free(rad);
  mju_free(pbody);

  mju_free(flist);
  mju_free(fxadr);
  mju_free(pt2vg);
  mju_free(pt2flex);
  mju_free(acon);
  mju_free(cand);
  mju_free(cgap);
  mju_free(candLS);
  mju_free(minc);
  mju_free(held);
  mju_free(gam);
  mju_free(actc);
  mju_free(appr);
  mju_free(actpt);
  mju_free(aset);
  mju_free(agap);
  mju_free(aheld);
  mju_free(amerge);
  mju_free(gv);
  mju_free(ge);
  mju_free(x);
  mju_free(xfree);
  mju_free(xtil);
  mju_free(xold);
  mju_free(xn);
  mju_free(aoff);
  mju_free(atid);

  mju_free(qn_a);
  mju_free(qtil_a);
  mju_free(qcur_a);
  mju_free(qdelta);
  mju_free(qdtmp);
  mju_free(a_gdq);
  mju_free(a_gr);
  mju_free(a_gMr);
  mju_free(dx);
  mju_free(qacc_pred);
  (void)stalled;  // CCD-freeze detector; the diagnostic kill was env-gated and is removed
}
