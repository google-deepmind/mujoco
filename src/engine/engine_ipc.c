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

// The IPC contact mode of the discrete integrator: barrier-free augmented-Lagrangian contact for
// 2D flexes (Li et al., arXiv:2512.12151). A per-pair multiplier and a persistent active set
// replace the log barrier, and a CCD-bounded committed position keeps the configuration
// intersection-free. Elasticity is not solved here: elastic2d rides the effective metric and
// edge-equality rides the native constraint rows.

#include "engine/engine_ipc.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include "engine/engine_collision_continuous.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_derivative.h"
#include "engine/engine_forward.h"
#include "engine/engine_memory.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"

#define IPC_NPT 8                    // max vertices in one pair: point-triangle 1+3, edge-edge 2+2
#define IPC_CDAMP_FLEX 0.1           // normal dashpot coefficient (cde = IPC_CDAMP_FLEX*m/h^2)
#define IPC_ASET_AGE 25              // drop a never-engaged pair after this many quiet updates
#define IPC_DECAY 0.3                // stiffness decay per quiet update: scale = IPC_DECAY^age * mu
#define IPC_ALPHA_LB 1e-6            // advance xfree only when the CCD fraction exceeds this
#define IPC_STALL_MAX 64             // outer iterations without a feasible advance: the step froze

// The three constants below are absolute lengths and speeds, tuned on metre-scale models with
// millimetre-thick flexes at 1-2 ms steps; a model at another scale needs them rescaled.
// TODO(quaglino): compute the standoff per flex pair from the radii, as the engine's flex contact
// does with their sum, instead of a hard-coded cap, with a floor for a zero radius; the band then
// follows from it.
#define IPC_DELTACAP 0.001           // rest gap between midsurfaces (m): the standoff is the
                                     // thinner radius of the pair, capped at this
#define IPC_GHAT (3 * IPC_DELTACAP)  // detection band (m), narrowed per pair to the thinner radius
#define IPC_VEL_TOL 0.05             // step convergence (m/s): |dx|_inf <= IPC_VEL_TOL*h; a safety
                                     // criterion, the loop normally ends on a full line-search step

// Solver-side contact pair: the multiplier, the linearization at xfree, the active-set age and the
// slack. Its leading members mirror mjcFlexPair, so a pair is passed to the mjc_* API by cast.
typedef struct {
  mjcFlexPairType type;        // == mjcFlexPair.type
  int idx[4];                  // == mjcFlexPair.idx
  int gi;                      // == mjcFlexPair.g
  mjtNum lam;                  // AL multiplier
  mjtNum ld0, ln[3], lcw[4];   // per-outer linearization at xfree
  int liv[4], lniv;
  int cnt;                     // active-set state machine
  mjtNum s;                    // materialized AL slack
} ipcCon;
#define IPC_PAIR(con) ((const mjcFlexPair*)(con))
// IPC_PAIR is a layout pun, so the shared prefix must stay put: a reorder of either struct
// would otherwise compile clean and silently mis-address every pair handed to the mjc_* API
_Static_assert(offsetof(ipcCon, type) == offsetof(mjcFlexPair, type), "ipcCon prefix: type");
_Static_assert(offsetof(ipcCon, idx) == offsetof(mjcFlexPair, idx), "ipcCon prefix: idx");
_Static_assert(offsetof(ipcCon, gi) == offsetof(mjcFlexPair, g), "ipcCon prefix: g");
_Static_assert(sizeof(((ipcCon*)0)->idx) == sizeof(((mjcFlexPair*)0)->idx), "ipcCon prefix: idx[]");

// per-pair AL stiffness, in force form: mjFLEXCONTACT_OMEGA2 times the smallest nonzero mass
// among the pair's vertices. A vertex pinned to a body carries mass 0 (its inertia is in the
// body), and a zero mu would put 0/0 in lam/mu; an all-pinned pair gets a tiny mu instead.
static mjtNum ipc_muPair(const ipcCon* con, const mjtNum* mass) {
  int vv[4], nvv;
  nvv = mjc_pairVerts(vv, IPC_PAIR(con));
  mjtNum mmin = 1e30;
  for (int q=0; q < nvv; q++) {
    mjtNum mv = mass[vv[q]];
    if (mv > 0 && mv < mmin) mmin = mv;
  }
  if (mmin >= 1e29) mmin = 1e-9;
  return mjFLEXCONTACT_OMEGA2 * mmin;
}

// cnt -> decay exponent c (AL normal-contact aging): c = cnt>=0 ? cnt : max(-cnt-6, 0).
static inline int ipc_cntExp(int cnt) { return cnt >= 0 ? cnt : (-cnt - 6 > 0 ? -cnt - 6 : 0); }

// stable per-pair hash for the persistent cnt store: contact type + sorted vertex/feature indices.
static uint64_t ipc_pairHash(const mjcFlexPair* con) {
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
  uint64_t hh = (uint64_t)con->type * 1000003ull + (uint64_t)(con->g + 1);
  for (int k=0; k < 4; k++)
    hh = hh * 1000003ull + (uint64_t)(id[k] + 1);
  return hh;
}


// AL slack of every pair at the iterate x: s = max(0, c_raw - lam/mu), with c_raw the gap
// linearized at xfree. The slack stays frozen through the inner solve; the row build, the energy
// and the multiplier update all read it.
static void ipc_updateSlack(ipcCon* cand, int ncand, const mjtNum* x, const mjtNum* xfree,
                            const mjtNum* rad, mjtNum ghat, const mjtNum* mass) {
  for (int c=0; c < ncand; c++) {
    ipcCon* con = &cand[c];
    mjtNum mu = ipc_muPair(con, mass);
    mjtNum craw = con->ld0 - mjc_standoff(mjc_pairBand(IPC_PAIR(con), rad, ghat), IPC_DELTACAP);
    for (int p=0; p < con->lniv; p++) {
      int v = con->liv[p];
      for (int k=0; k < 3; k++)
        craw += con->lcw[p] * con->ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    con->s = mjd_flexContactSlack(mu, craw, con->lam);
  }
}

// Cross-step state lives per flex vertex: the multiplier (d->flexvert_lambda, the max over the
// pairs touching the vertex) and the contact age (d->flexvert_conage: < 0 loaded recently, > 0
// quiet updates since). A pair reads the min age over its vertices. The age drives the stiffness
// decay; eviction is decided in ipc_mergeActiveSet. Both key on the same signal, load at a vertex,
// so a pair is dropped at the bottom of its decay ramp rather than part way down, and mirror-image
// contacts stay symmetric (a pair's identity is an arbitrary closest-feature label; load is not).

// pair age: the min over its vertices, 0 when none carries one
static int ipc_conAge(const ipcCon* con, const int* age, const int* pt2vg) {
  int vv[4], nvv, a = INT_MAX;
  nvv = mjc_pairVerts(vv, IPC_PAIR(con));
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
    nvv = mjc_pairVerts(vv, IPC_PAIR(&aset[c]));
    for (int q=0; q < nvv; q++) loaded[vv[q]] = 1;
  }
  for (int i=0; i < npt; i++) {
    int g = pt2vg[i];
    if (loaded[i]) age[g] = (age[g] == 0 || age[g] > 5) ? 0 : -1;
    else           age[g] += (age[g] >= 0) ? 1 : -1;
  }
}

// Multiplier and age update at the iterate x, reading the multiplier from before the update:
//   inactive (c_raw - lam/mu > 0): lam = 0, the quiet counter advances
//   active:                        lam -= c_raw*mu, the counter marks the pair loaded
// with no clamp on lam. Then sink lam into the per-vertex store for the next step's warm start.
static void ipc_flexLamUpdate(const mjtNum* x, const mjtNum* xfree, const mjtNum* rad, mjtNum ghat,
                              const mjtNum* mass, ipcCon* cand, int ncand, mjtNum* pal,
                              const int* pt2vg, int npt) {
  for (int c=0; c < ncand; c++) {
    ipcCon* con = &cand[c];
    mjtNum mu = ipc_muPair(con, mass);
    mjtNum lam0 = con->lam;
    mjtNum craw = con->ld0 - mjc_standoff(mjc_pairBand(IPC_PAIR(con), rad, ghat), IPC_DELTACAP);
    for (int p=0; p < con->lniv; p++) {
      int v = con->liv[p];
      for (int k=0; k < 3; k++)
        craw += con->lcw[p] * con->ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    if (craw - lam0 / mu > 0) {  // inactive
      con->lam = 0;
      con->cnt += (con->cnt >= 0) ? 1 : -1;
    } else {  // active
      con->lam = lam0 - craw * mu;
      con->cnt = (con->cnt == 0 || con->cnt > 5) ? 0 : -1;
    }
  }
  for (int p=0; p < npt; p++)
    pal[pt2vg[p]] = 0;
  for (int c=0; c < ncand; c++) {
    if (cand[c].lam <= 0) continue;
    int vv[4], nvv;
    nvv = mjc_pairVerts(vv, IPC_PAIR(&cand[c]));
    for (int q=0; q < nvv; q++)
      if (cand[c].lam > pal[pt2vg[vv[q]]]) pal[pt2vg[vv[q]]] = cand[c].lam;
  }
}

// Merge the fresh candidates into the persistent active set: an existing pair is kept while it
// carries load or has not yet aged out (see below), and a candidate is added iff it was admitted
// (cadmit) and is not already present. New entries warm-start lam and the age from the per-vertex
// stores; the linearization fields are refreshed by the next outer iteration.
static void ipc_mergeActiveSet(mjData* d, ipcCon** aset, int* naset, const mjcFlexPair* cand,
                               int ncand, const int* cadmit, const mjtNum* pal, const int* conage,
                               const int* pt2vg) {
  // the merged set goes to a fresh arena block that replaces *aset, the old block being left to
  // the arena's next reset; its size is bounded by the old set plus the admitted candidates, both
  // known here, so the block never overflows
  int bound = *naset;
  for (int c=0; c < ncand; c++) bound += cadmit[c] ? 1 : 0;
  ipcCon* amerge = NULL;
  if (bound) {
    amerge = (ipcCon*)mj_arenaAllocByte(d, bound * sizeof(ipcCon), _Alignof(ipcCon));
    if (!amerge) {
      mjERROR("arena too small for the IPC active set (%d pairs): increase the model's memory",
              bound);
    }
  }
  const ipcCon* old = *aset;
  // presence hash over the merged set, open addressing keyed by pairHash (0 = empty; a true hash
  // of 0 would at worst admit one duplicate), per-call scratch on the mjData stack
  int cap = 1;
  while (cap < 4 * (*naset + ncand) + 16)
    cap <<= 1;
  mj_markStack(d);
  uint64_t* mkey =
      (uint64_t*)mj_stackAllocByte(d, (size_t)cap * sizeof(uint64_t), sizeof(mjtNum));
  uint64_t mask = (uint64_t)(cap - 1);
  for (int i=0; i < cap; i++)
    mkey[i] = 0;
  int nm = 0;
  // 1) keep existing pairs that still carry load
  for (int c=0; c < *naset; c++) {
    // A pair that has never carried load (cnt >= 0) is still raising its multiplier from zero, so
    // lam <= 0 cannot tell it from a finished pair: only engaged pairs (cnt < 0) are evicted for
    // going quiet, and never-engaged pairs age out on their own counter.
    if ((old[c].lam <= 0 && old[c].cnt < 0) || old[c].cnt > IPC_ASET_AGE) continue;
    uint64_t key = ipc_pairHash(IPC_PAIR(&old[c])), h = key & mask;
    while (mkey[h] != 0) {
      if (mkey[h] == key) break;
      h = (h + 1) & mask;
    }
    if (mkey[h] == key) continue;
    mkey[h] = key;
    amerge[nm++] = old[c];
  }
  // 2) add new admitted broad-phase candidates not already present
  for (int c=0; c < ncand; c++) {
    if (!cadmit[c]) continue;
    uint64_t key = ipc_pairHash(IPC_PAIR(&cand[c])), h = key & mask;
    while (mkey[h] != 0) {
      if (mkey[h] == key) break;
      h = (h + 1) & mask;
    }
    if (mkey[h] == key) continue;  // dedup: pair already in the merged set
    ipcCon con = {0};
    con.type = cand[c].type;
    for (int q=0; q < 4; q++) con.idx[q] = cand[c].idx[q];
    con.gi = cand[c].g;
    int vv[4], nvv;
    nvv = mjc_pairVerts(vv, IPC_PAIR(&con));  // warm start from the per-vertex stores
    mjtNum s = 0;
    for (int q=0; q < nvv; q++)
      if (pal[pt2vg[vv[q]]] > s) s = pal[pt2vg[vv[q]]];
    con.lam = s;
    con.cnt = ipc_conAge(&con, conage, pt2vg);
    con.s = 0;
    mkey[h] = key;
    amerge[nm++] = con;
  }
  *aset = amerge;
  *naset = nm;
  mj_freeStack(d);
}


// incremental potential over the flex points: inertia, the AL penalty over the active set (the
// same set the injected rows are built from) and the matching one-sided dashpot
static mjtNum ipc_energy(int nfv, const mjtNum* x, const mjtNum* xtil, const int* fidx,
                         const mjtNum* mass, mjtNum h, const mjtNum* rad, mjtNum ghat,
                         const ipcCon* acon, int nacon, const mjtNum* xold, const mjtNum* xfree) {
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
    // 0.5*scale*d^2 over the slack-baked residual, matching the injected row's gradient
    mjtNum mu = ipc_muPair(&acon[c], mass);
    mjtNum craw = acon[c].ld0 - mjc_standoff(mjc_pairBand(IPC_PAIR(&acon[c]), rad, ghat), IPC_DELTACAP);
    for (int p=0; p < acon[c].lniv; p++) {
      int v = acon[c].liv[p];
      for (int k=0; k < 3; k++)
        craw += acon[c].lcw[p] * acon[c].ln[k] * (x[3 * v + k] - xfree[3 * v + k]);
    }
    mjtNum dd = mjd_flexContactResidual(mu, craw, acon[c].s, acon[c].lam);
    int cexp = ipc_cntExp(acon[c].cnt);
    mjtNum scale = mu;
    for (int e=0; e < cexp; e++)
      scale *= IPC_DECAY;
    E += 0.5 * scale * dd * dd;  // the slack carries the inequality
    mjtNum dn = 0;               // one-sided dashpot
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

// trial state -> the QP variable: qacc = (v_new - v_old)/h, with v_new = dp_local/h on the flex
// slide dofs (world to body frame through R') and qdelta/h on the appended articulated trees;
// dofs the step does not touch stay at qacc_smooth. Shared so every merit term prices one point.
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


// the metric's stiffness half of the Gauss term, 0.5*(a-a*)' K (a-a*) in position units; the
// inertia half is scored by ipc_energy and ipc_articResid. Without it the merit prices a different
// objective than the inner solve minimizes. Contact is left out: the AL merit scores it itself.
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
  mjd_effMulAdd(m, d, Kda, da, /*flg_contact=*/0);  // K*(a-a*)
  mjtNum E = 0.5 * h * h * mju_dot(da, Kda, nv);
  mj_freeStack(d);
  return E;
}


// merit cost of the efc rows: exactly what the QP minimizes over them, evaluated at the trial
// state by mj_constraintUpdate and put in position units like every other merit term
static mjtNum ipc_efcCost(const mjModel* m, mjData* d, const mjtNum* xt, const mjtNum* xold,
                          int npt, const int* fidx, const int* dofadr, const int* pbody,
                          const mjtNum* qdt, int na_artic, const int* atid, const int* aoff, int N,
                          mjtNum h, int nefc_qp) {
  int nv = m->nv, nefc = nefc_qp;  // the rows the QP minimized
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
  return h * h * cost;
}




// The step: minimize the incremental potential over the flex vertices and the appended
// articulated trees with penetration-free contact. Each outer iteration linearizes the active
// pairs at the committed position xfree, runs one discrete solve with the pairs published as the
// metric's contact class, line-searches the result on the merit, updates the multipliers, and
// advances xfree along the iterate as far as conservative CCD allows. Covers flex self-contact
// (vertex-triangle and edge-edge) and flex-vs-static-geom; rigid contact stays on the native
// constraint rows.

// kinetic incremental potential of the appended articulated trees at a candidate tangent qdc:
// q = q_n (+) qdc, gr = q (-) q~, gMr = M*gr with M frozen at q_n. Returns 0.5/h^2 gr'M gr over
// the articulated dofs, with implicit joint damping; the caller scatters ih2*gMr into the gradient.
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
  mjtNum hd = m->opt.timestep, ihd = 1.0 / hd;
  mjtNum E = 0;
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
  // every dim-2 flex takes part; their vertices are concatenated in flex order and fxadr[k] is
  // the offset of flex flist[k]. A model with no dim-2 flex takes the same path with an empty
  // flex block: the appended articulated trees carry it, on the native efc rows.
  int nfd = 0;
  for (int i=0; i < m->nflex; i++)
    if (m->flex_dim[i] == 2) nfd++;

  // all per-step scratch lives on the mjData stack from here to the mj_freeStack at the end
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
  // detection band; per-pair narrowing happens in mjc_pairBand and mjc_candidates
  mjtNum ghat = IPC_GHAT;
  // free point -> global flex vertex, and -> dim-2 flex slot
  int* pt2vg = mj_stackAllocInt(d, (nfv > 0 ? nfv : 1));
  int* pt2flex = mj_stackAllocInt(d, (nfv > 0 ? nfv : 1));
  for (int k=0; k < nfd; k++) {
    int va_k = m->flex_vertadr[flist[k]], nv_k = m->flex_vertnum[flist[k]];
    for (int lv=0; lv < nv_k; lv++) {
      pt2vg[fxadr[k] + lv] = va_k + lv;
      pt2flex[fxadr[k] + lv] = k;
    }
  }

  // the point array holds flex vertices only; every other body with dofs is an appended
  // articulated tree (below)
  char* isflexvert = (char*)mj_stackAllocByte(d, (m->nbody > 0 ? m->nbody : 1), 1);
  for (int b=0; b < m->nbody; b++)
    isflexvert[b] = 0;
  for (int v=0; v < nfv; v++)
    isflexvert[m->flex_vertbodyid[pt2vg[v]]] = 1;
  int npt = nfv;
  int* dofadr = mj_stackAllocInt(d, (npt > 0 ? npt : 1));
  int* qpadr = mj_stackAllocInt(d, (npt > 0 ? npt : 1));  // qpos address, not the dof address
  int* fidx = mj_stackAllocInt(d, (npt > 0 ? npt : 1));   // free-point index, -1 when pinned
  mjtNum* mass = mj_stackAllocNum(d, (npt > 0 ? npt : 1));
  mjtNum* rad = mj_stackAllocNum(d, (npt > 0 ? npt : 1));
  int* pbody = mj_stackAllocInt(d, (npt > 0 ? npt : 1));  // body, for the slide-frame rotation
  int nfree = 0;
  for (int k=0; k < nfd; k++) {
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
        mass[v] = mjd_flexVertMass(m, d, va_k + lv);
      }
    }
  }
  int nfree_flex = nfree;  // flex free-DOF count
  // articulated trees: every tree with dofs that is not a flex vertex, appended to the solver
  // vector after the flex block at offset aoff[tree]
  int ntree = m->ntree, na_artic = 0;
  char* isartictree = (char*)mj_stackAllocByte(d, (ntree > 0 ? ntree : 1), 1);
  for (int t=0; t < ntree; t++)
    isartictree[t] = 0;
  for (int b=1; b < m->nbody; b++) {
    if (m->body_dofnum[b] == 0 || isflexvert[b]) continue;
    isartictree[m->dof_treeid[m->body_dofadr[b]]] = 1;
  }
  int N = 3 * nfree_flex, N_artic = 0;  // the flex block, then the appended trees
  int* aoff = mj_stackAllocInt(d, (ntree > 0 ? ntree : 1));
  int* atid = mj_stackAllocInt(d, (ntree > 0 ? ntree : 1));  // articulated tree ids
  for (int t=0; t < ntree; t++) {
    if (isartictree[t]) {
      aoff[t] = N + N_artic;
      atid[na_artic++] = t;
      N_artic += m->tree_dofnum[t];
    } else
      aoff[t] = -1;
  }
  int N_total = N + N_artic, Na = (N_total > 0 ? N_total : 1);  // full solver vector length
  int nstate = nfv;
  // the persistent active set, maintained across outer iterations by ipc_mergeActiveSet; the
  // injected rows and the merit are both built from it. It lives on the arena, sized exactly:
  // seeded from the first broad phase below, then replaced by each merge
  ipcCon* aset = NULL;
  int naset = 0;
  // static-geom sharp features (vertices, edges), collected once per step
  int gvcap = 1, gecap = 1;
  for (int gi=0; gi < m->ngeom; gi++) {
    if (m->geom_contype[gi] == 0 && m->geom_conaffinity[gi] == 0) continue;
    if (m->body_weldid[m->geom_bodyid[gi]] != 0) continue;  // static geoms only
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
    if (m->geom_contype[gi] == 0 && m->geom_conaffinity[gi] == 0) continue;
    if (m->body_weldid[m->geom_bodyid[gi]] != 0) continue;  // static geoms only
    ngv += mjc_GeomVerts(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, gv + 3 * ngv);
    nge += mjc_GeomEdges(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, ge + 6 * nge);
  }
  // state vectors are sized 3*nstate (flex points 0..nfv-1), indexed by their state slot.
  mjtNum* x = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* xfree = mj_stackAllocNum(d, 3 * nstate);  // the committed intersection-free position
  mjtNum* xtil = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* xold = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* xn = mj_stackAllocNum(d, 3 * nstate);
  mjtNum* dx = mj_stackAllocNum(d, Na);

  // free-flight predictor q~ = q + h*v + h^2*qacc_smooth
  mjtNum* qacc_pred = mj_stackAllocNum(d, (m->nv > 0 ? m->nv : 1));
  mju_copy(qacc_pred, d->qacc_smooth, m->nv);

  // articulated state: q_n, the predictor q~ = q_n (+) h*(v + h*qacc_smooth) in generalized
  // coordinates, and the accumulated tangent qdelta; sized 1 when there are no articulated trees
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
    mju_copy(qn_a, d->qpos, m->nq);
    // the predictor is undamped: remove the explicit -M^-1 D v that qacc_smooth carries, since
    // the solve applies joint damping implicitly (ipc_articResid)
    for (int i=0; i < m->nv; i++)
      a_gr[i] = m->dof_damping[i] * d->qvel[i];
    mj_solveM(m, d, a_gMr, a_gr, 1);
    for (int i=0; i < m->nv; i++)
      a_gdq[i] = d->qvel[i] + h * qacc_pred[i] + h * a_gMr[i];
    mju_copy(qtil_a, qn_a, m->nq);
    mj_integratePos(m, qtil_a, a_gdq, h);
  }

  const mjtNum* vx = d->flexvert_xpos;
  for (int v=0; v < npt; v++) {
    for (int c=0; c < 3; c++)
      xold[3 * v + c] = vx[3 * pt2vg[v] + c];
    if (fidx[v] >= 0) {
      int da = dofadr[v];
      // slide dofs are in the body frame; R = d->xmat maps them to world
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
    x[i] = xold[i];  // start from the committed, collision-free state

  mjtNum ih2 = 1.0 / (h * h);
  char* ageload = (char*)mj_stackAllocByte(d, (size_t)(npt > 0 ? npt : 1), 1);  // ipc_ageStep temp
  // candidates: the detection margin is inflated by the predictor displacement so every pair
  // that can close during the step is captured
  mjtNum maxdisp = 0;
  for (int v=0; v < npt; v++)
    if (fidx[v] >= 0) {
      mjtNum dd[3];
      for (int c=0; c < 3; c++)
        dd[c] = xtil[3 * v + c] - xold[3 * v + c];
      mjtNum L = mju_sqrt(mju_dot3(dd, dd));
      if (L > maxdisp) maxdisp = L;
    }
  mjtNum thresh = 3 * ghat + 4 * maxdisp;
  // static features move on the flex side only, so their margin is half the two-sided one; the
  // full margin on a many-piece convex bin produces hundreds of thousands of candidates
  mjtNum threshGeom = ghat + 2.0 * maxdisp;
  // the candidates live on the arena, pushed one at a time by the broad phase (mjc_candidates),
  // so every query yields an exactly sized array with nothing reserved up front
  mjcFlexPair* cand;
  int ncand = mjc_candidates(m, d, x, gv, ge, ngv, nge, rad, thresh, threshGeom, maxdisp, xold,
                             xtil, ghat, nfv, npt, fidx, flist, fxadr, nfd, pt2flex, &cand);
  // with no candidate within the margin the predictor is collision-free and a better start
  // than xold
  if (ncand == 0)
    for (int i=0; i < 3 * nstate; i++)
      x[i] = xtil[i];
  // seed the persistent active set from the first broad phase, warm-starting lam and the age
  // from the per-vertex stores
  naset = ncand;
  if (naset) {
    aset = (ipcCon*)mj_arenaAllocByte(d, naset * sizeof(ipcCon), _Alignof(ipcCon));
    if (!aset) {
      mjERROR("arena too small for the IPC active set (%d pairs): increase the model's memory",
              naset);
    }
  }
  for (int c=0; c < naset; c++) {
    ipcCon* a0 = &aset[c];
    memset(a0, 0, sizeof(ipcCon));
    a0->type = cand[c].type;
    for (int q=0; q < 4; q++) a0->idx[q] = cand[c].idx[q];
    a0->gi = cand[c].g;
    {
      int vv[4], nvv = mjc_pairVerts(vv, IPC_PAIR(a0));
      mjtNum s = 0;
      for (int q=0; q < nvv; q++)
        if (d->flexvert_lambda[pt2vg[vv[q]]] > s) s = d->flexvert_lambda[pt2vg[vv[q]]];
      a0->lam = s;
      a0->cnt = ipc_conAge(a0, d->flexvert_conage, pt2vg);
    }
  }
  for (int i=0; i < 3 * nstate; i++)
    xfree[i] = xold[i];
  // outer loop (the paper's Alg. 1): one discrete solve per iteration; beta accumulates the
  // fraction of the motion the committed position has absorbed
  mjtNum beta = 0.0;
  int inner_cap = 1;           // one solve per outer iteration
  int outer_cap = 1024;
  int stall = 0, stalled = 0;  // consecutive iterations without a feasible advance
  mjtNum last_ls_alpha = 1.0;  // accepted line-search step of the last iteration
  int nefc_qp = 0;             // nefc as the QP saw it; the merit scores exactly these rows
  for (int outer=0; outer < outer_cap && N_total > 0; outer++) {  // N_total: rigid-only runs too
    ipcCon* wcon = aset;  // the working set is the persistent active set
    int wn = naset;
    // linearize the pairs at xfree (Eq. 10): c(x) is linear in x for this iteration
    for (int c=0; c < wn; c++)
      wcon[c].ld0 = mjc_pairGap(IPC_PAIR(&wcon[c]), m, d, xfree, gv, ge, rad, wcon[c].ln, wcon[c].liv,
                               wcon[c].lcw, &wcon[c].lniv, ghat);
    // x persists across outer iterations (the paper's warm start); the slack is taken at x and
    // frozen for the inner solve
    ipc_updateSlack(wcon, wn, x, xfree, rad, ghat, mass);
    int flex_converged_out = 0, artic_converged_out = 0, newton_converged_out = 0;
    for (int it=0; it < inner_cap && N_total > 0; it++) {
      // one set: the injected rows and the merit are built from the same active set, which is
      // already the CCD-admitted one, so no proximity test decides membership
      // (IpcTest.SelfContactConservesMomentum guards the agreement)
      int nacon = wn;
      // Inner solve: one discrete constraint solve with every active pair published as the
      // metric's contact class, whether or not the model also has rigid bodies. In a pure-flex
      // model the native contact rows are dropped, since the pairs replace them; with rigid
      // bodies they stay (rigid contact with friction). The converged qacc is the search direction
      // for the merit line search below.
      {
        int cap = 2 * nacon + 1;  // contact stiffness + dashpot pairs
        // per-iteration frame: these arrays are sized by the live pair count
        mj_markStack(d);
        // per pair: dof triples and weights (esbase, esw), stiffness (esD) and force coefficient
        // (esfc); esf is the summed constant force
        int* esnpt = mj_stackAllocInt(d, cap);
        int* esbase = mj_stackAllocInt(d, IPC_NPT * cap);
        mjtNum* esw = mj_stackAllocNum(d, 3 * IPC_NPT * cap);
        mjtNum* esD = mj_stackAllocNum(d, cap);
        mjtNum* esfc = mj_stackAllocNum(d, cap);
        mjtNum* esf = mj_stackAllocNum(d, m->nv);
        int npair = 0;
        mjtNum h2 = 1.0 / ih2;
        for (int c=0; c < nacon; c++) {
          ipcCon* con = &wcon[c];
          mjtNum mu = ipc_muPair(con, mass);
          int cexp = ipc_cntExp(con->cnt);
          mjtNum D = mu;
          for (int e=0; e < cexp; e++)
            D *= IPC_DECAY;
          mjtNum delta = mjc_standoff(mjc_pairBand(IPC_PAIR(con), rad, ghat), IPC_DELTACAP);
          mjtNum refc = -mjd_flexContactResidual(mu, con->ld0 - delta, con->s, con->lam);
          int np = 0;
          // the reference is exact: the h*Rqv terms cancel per vertex against J*qacc
          for (int p=0; p < con->lniv; p++) {
            int v = con->liv[p];
            if (dofadr[v] < 0) continue;  // pinned: no dofs, and its reference term is zero
            const mjtNum* R = d->xmat + 9 * pbody[v];
            const mjtNum* qv = d->qvel + dofadr[v];
            mjtNum Rtn[3], Rqv[3];
            mju_mulMatTVec3(Rtn, R, con->ln);
            mju_mulMatVec3(Rqv, R, qv);
            esbase[IPC_NPT*npair + np] = dofadr[v];
            for (int k=0; k < 3; k++) {
              esw[3*IPC_NPT*npair + 3*np + k] = con->lcw[p] * Rtn[k] * h2;
              refc -= con->lcw[p] * con->ln[k] * (xold[3 * v + k] + h * Rqv[k] - xfree[3 * v + k]);
            }
            np++;
          }
          // D = mu/h^2: the inertia is in acceleration units, 1/h^2 heavier than in position form
          esnpt[npair] = np;
          esD[npair] = D * ih2;
          esfc[npair] = esD[npair] * refc;   // the pair's force along its row
          npair++;
        }
        int savednefc = d->nefc;
        if (na_artic == 0 && nfv > 0) d->nefc = d->ne + d->nf + d->nl;  // pure flex: no native contacts
        nefc_qp = d->nefc;
        // The subproblem is solved by MuJoCo's nonlinear CG on the shared primal path, with no
        // factorization to go rank-deficient under stiff contact. A fully converged direction per
        // outer iteration is wasted, since the outer AL converges at the same rate with a loosely
        // solved subproblem; the iteration cap is the model's opt.iterations.
        // solver_niter is zeroed per solve; accumulate it so the step reports its total
        int niter[mjNISLAND];
        for (int i=0; i < mjNISLAND; i++) niter[i] = d->solver_niter[i];
        // publish the pairs as the metric's contact class for the duration of the solve: one
        // packed row per pair, [nnz, conid, colind...] and [scale, force, val...]
        int* con_ind = mj_stackAllocInt(d, cap * (2 + 3*IPC_NPT));
        mjtNum* con_val = mj_stackAllocNum(d, cap * (2 + 3*IPC_NPT));
        int nrow = 0;
        for (int c=0; c < npair; c++) {
          int n = esnpt[c];
          con_ind[nrow] = 3*n;
          con_ind[nrow + 1] = -1;      // no mjContact behind an AL pair
          con_val[nrow] = esD[c];
          con_val[nrow + 1] = esfc[c];  // the pair's force along the row
          for (int q=0; q < n; q++) {
            int b = esbase[IPC_NPT*c + q];
            for (int k=0; k < 3; k++) {
              con_ind[nrow + 2 + 3*q + k] = b + k;
              con_val[nrow + 2 + 3*q + k] = esw[3*IPC_NPT*c + 3*q + k];
            }
          }
          nrow += 2 + 3*n;
        }
        int* con_ind_saved = d->efm_con_ind;
        mjtNum* con_val_saved = d->efm_con_val;
        int nefmcon_saved = d->nefmcon;
        // the constant force goes into qfrc_smooth, which the solver's line search reads;
        // qfrc_constraint would be invisible to it
        d->efm_con_ind = con_ind;
        d->efm_con_val = con_val;
        d->nefmcon = nrow;
        mju_zero(esf, m->nv);
        mjd_effContactForce(d, esf);
        mju_addTo(d->qfrc_smooth, esf, m->nv);
        mj_fwdConstraintCG(m, d);  // CG over the monolithic problem, whatever the model's solver
        d->efm_con_ind = con_ind_saved;
        d->efm_con_val = con_val_saved;
        d->nefmcon = nefmcon_saved;
        mju_subFrom(d->qfrc_smooth, esf, m->nv);
        for (int i=0; i < mjNISLAND; i++) d->solver_niter[i] += niter[i];
        d->nefc = savednefc;
        // the converged solution is a search direction, not a commit: the AL energy is finite,
        // so a converged step is large and must be line-searched
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
            }
          }
        for (int a=0; a < na_artic; a++) {
          int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
          for (int i=0; i < nd; i++)
            dx[N + (o - N + i)] = (h * d->qvel[da + i] + h2 * d->qacc[da + i]) - qdelta[o - N + i];
        }
        mj_freeStack(d);
      }
      if (na_artic) {  // FK at q_n (+) qdelta so rigid contacts read live poses; M stays at q_n
        for (int i=0; i < m->nv; i++)
          a_gdq[i] = 0;
        for (int a=0; a < na_artic; a++) {
          int t = atid[a], o = aoff[t], da = m->tree_dofadr[t], nd = m->tree_dofnum[t];
          for (int i=0; i < nd; i++)
            a_gdq[da + i] = qdelta[o + i - N];
        }
        mju_copy(d->qpos, qn_a, m->nq);
        mj_integratePos(m, d->qpos, a_gdq, 1);
        mj_kinematics(m, d);
      }
      // convergence per block on the solved dx: the articulated block must converge before the
      // outer loop may end (an unconverged rigid step injects energy through the explicit contact
      // reference), while the flex block may take the full-step bail (AL contact is stable
      // unconverged, and forcing it under load costs many outer iterations)
      int newton_converged = 0;
      {
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
      // line search on the merit: inertia, the articulated kinetic term, the native efc cost,
      // the metric's stiffness term and the AL penalty at live gaps. The full linearized step
      // overshoots into penetration and the CCD advance then collapses
      mjtNum E0 = ipc_energy(npt, x, xtil, fidx, mass, h, rad, ghat, wcon, wn, xold, xfree) +
                  ipc_articResid(m, d, qn_a, qtil_a, qdelta, na_artic, atid, aoff, N, ih2, a_gdq,
                                 qcur_a, a_gr, a_gMr) +
                  ipc_efcCost(m, d, x, xold, npt, fidx, dofadr, pbody, qdelta, na_artic, atid, aoff,
                              N, h, nefc_qp) +
                  ipc_metricCost(m, d, x, xold, npt, fidx, dofadr, pbody, qdelta, na_artic, atid,
                                 aoff, N, h);
      mjtNum alpha = 1.0;
      // monotone decrease with slop, up to 8 halvings, always accepting the final trial
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
        if (na_artic) {  // FK at the trial configuration
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
        mjtNum Etr = ipc_energy(npt, xn, xtil, fidx, mass, h, rad, ghat, wcon, wn, xold, xfree) +
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
      last_ls_alpha = alpha;
    }
    // non-penetration advance, in this order: multiplier update on the persistent set, re-query
    // the candidates at xfree over the swept segment xfree -> x, CCD fraction, merge the admitted
    // candidates, advance xfree by that fraction
    ipc_flexLamUpdate(x, xfree, rad, ghat, mass, aset, naset, d->flexvert_lambda, pt2vg, npt);
    ipc_ageStep(d->flexvert_conage, aset, naset, pt2vg, npt, ageload);
    // displacement x - xfree in the free-dof layout
    for (int v=0; v < npt; v++)
      if (fidx[v] >= 0)
        for (int c=0; c < 3; c++)
          dx[3 * fidx[v] + c] = x[3 * v + c] - xfree[3 * v + c];
    // re-query over the swept segment with a fixed collar (3*ghat, no displacement scaling), which
    // keeps the count bounded however far x flung
    ncand = mjc_candidates(m, d, xfree, gv, ge, ngv, nge, rad, 3 * ghat, 3 * ghat, 0.0, xfree, x,
                           ghat, nfv, npt, fidx, flist, fxadr, nfd, pt2flex, &cand);
    // per-query scratch: the gap at xfree, the closing flag from mjc_advance and the admission
    // mask handed to the merge
    mj_markStack(d);
    int nq = ncand > 0 ? ncand : 1;
    mjtNum* cgap = mjSTACKALLOC(d, nq, mjtNum);
    int* appr = mjSTACKALLOC(d, nq, int);
    int* actc = mjSTACKALLOC(d, nq, int);
    for (int c=0; c < ncand; c++) {
      mjtNum nn[3], cw[4];
      int idv[4], ni;  // gaps at xfree (for CCD + admission)
      cgap[c] = mjc_pairGap(IPC_PAIR(&cand[c]), m, d, xfree, gv, ge, rad, nn, idv, cw, &ni, ghat);
    }
    // conservative-advancement CCD (mjc_advance): the largest fraction of xfree -> x that keeps
    // every gap above a fraction of its current value; appr flags the pairs that close this step
    mjtNum ac =
        mjc_advance(m, d, xfree, dx, gv, ge, rad, nfv, fidx, cand, ncand, cgap, pt2flex, appr, NULL);
    // admit a candidate iff it closes this step or is already distance-active
    for (int c=0; c < ncand; c++)
      actc[c] = (appr[c] || cgap[c] <= 0.0) ? 1 : 0;
    ipc_mergeActiveSet(d, &aset, &naset, cand, ncand, actc, d->flexvert_lambda,
                       d->flexvert_conage, pt2vg);
    mj_freeStack(d);
    // advance xfree by the CCD fraction
    if (ac > IPC_ALPHA_LB)
      for (int i=0; i < 3 * npt; i++)
        xfree[i] = (1.0 - ac) * xfree[i] + ac * x[i];
    beta = beta + (1.0 - beta) * ac;
    // terminate once the advance is complete and the full step was accepted or both blocks
    // converged. A partial advance commits only part of the motion while time still advances by h,
    // which reads as slow motion; partial advances are fine within the loop, and a frozen CCD
    // leaves through the stall path below
    if (beta >= 1.0 - 1e-6 && (last_ls_alpha >= 1.0 - 1e-9 || newton_converged_out))
      break;
    if (ac > IPC_ALPHA_LB)
      stall = 0;
    else if (++stall >= IPC_STALL_MAX) {
      stalled = 1;
      break;
    }  // CCD froze: warn below
  }
  for (int i=0; i < 3 * nstate; i++)
    x[i] = xfree[i];  // commit the intersection-free position
  // restore the position-dependent fields to q_n: the solve ran FK at trial configurations, and
  // a step leaves them at the pre-step configuration
  if (na_artic) {
    mju_copy(d->qpos, qn_a, m->nq);
    mj_kinematics(m, d);
    mj_comPos(m, d);
  }
  // position-level commit: the solve yields q_{n+1}, so qpos and qvel are set here rather than
  // through mj_advance (algebraically the same, not bitwise); qacc is published as
  // (v_new - v_old)/h, matching ipc_efcCost, and dofs the solve does not own keep qacc_smooth
  mjtNum* qacc_out = mj_stackAllocNum(d, m->nv);
  mju_copy(qacc_out, d->qacc_smooth, m->nv);
  for (int v=0; v < npt; v++)
    if (fidx[v] >= 0) {
      int da = dofadr[v];
      int qa = qpadr[v];
      // world displacement back to the body-frame slide dofs through R'
      const mjtNum* R = d->xmat + 9 * pbody[v];
      mjtNum dpw[3], dpl[3];
      for (int c=0; c < 3; c++)
        dpw[c] = x[3 * v + c] - xold[3 * v + c];
      mju_mulMatTVec3(dpl, R, dpw);
      for (int c=0; c < 3; c++) {
        qacc_out[da + c] = (dpl[c] / h - d->qvel[da + c]) / h;  // reads v_old first
        d->qvel[da + c] = dpl[c] / h;
        d->qpos[qa + c] += dpl[c];
      }
    }
  // articulated readback: q_{n+1} = q_n (+) qdelta per joint, v = qdelta/h
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
  // publish the step's effective acceleration, as an integrator hands it to mj_advance
  mju_copy(d->qacc, qacc_out, m->nv);
  // the shared tail with zero qacc and qvel: the velocity and position stages are no-ops, the
  // committed values stand, and history, activation, sleep, plugins and time run. The warm start
  // is preserved rather than saved: every inner solve is a different subproblem, so the previous
  // acceleration is no guess for it
  mjtNum* zero_nv = mj_stackAllocNum(d, m->nv);
  mjtNum* ws_keep = mj_stackAllocNum(d, m->nv);
  mju_zero(zero_nv, m->nv);
  mju_copy(ws_keep, d->qacc_warmstart, m->nv);
  mj_advance(m, d, d->act_dot, zero_nv, zero_nv);
  mju_copy(d->qacc_warmstart, ws_keep, m->nv);
  mj_freeStack(d);
  // an incomplete advance commits only part of the motion while time advances by h, which reads
  // as slow motion rather than as a failure: say so on either exit, the symptom is otherwise
  // silent
  if (beta < 1.0 - 1e-6) {
    if (stalled) {
      mju_warning("IPC: CCD stalled after %d iterations without a feasible advance; the step is "
                  "incomplete (advanced %.3f of the motion). Time = %.4f",
                  IPC_STALL_MAX, beta, d->time);
    } else {
      mju_warning("IPC: outer iteration limit reached; the step is incomplete (advanced %.3f of "
                  "the motion). Time = %.4f", beta, d->time);
    }
  }
}
