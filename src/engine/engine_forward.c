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

#include "engine/engine_forward.h"

#include <stddef.h>
#include <stdio.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include "engine/engine_callback.h"
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_derivative.h"
#include "engine/engine_inverse.h"
#include "engine/engine_island.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_passive.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sensor.h"
#include "engine/engine_solver.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"
#include "thread/thread_pool.h"
#include "thread/thread_task.h"



//--------------------------- check values ---------------------------------------------------------

// check positions, reset if bad
void mj_checkPos(const mjModel* m, mjData* d) {
  for (int i=0; i < m->nq; i++) {
    if (mju_isBad(d->qpos[i])) {
      mj_warning(d, mjWARN_BADQPOS, i);
      mj_resetData(m, d);
      d->warning[mjWARN_BADQPOS].number++;
      d->warning[mjWARN_BADQPOS].lastinfo = i;
      return;
    }
  }
}



// check velocities, reset if bad
void mj_checkVel(const mjModel* m, mjData* d) {
  for (int i=0; i < m->nv; i++) {
    if (mju_isBad(d->qvel[i])) {
      mj_warning(d, mjWARN_BADQVEL, i);
      mj_resetData(m, d);
      d->warning[mjWARN_BADQVEL].number++;
      d->warning[mjWARN_BADQVEL].lastinfo = i;
      return;
    }
  }
}



// check accelerations, reset if bad
void mj_checkAcc(const mjModel* m, mjData* d) {
  for (int i=0; i < m->nv; i++) {
    if (mju_isBad(d->qacc[i])) {
      mj_warning(d, mjWARN_BADQACC, i);
      mj_resetData(m, d);
      d->warning[mjWARN_BADQACC].number++;
      d->warning[mjWARN_BADQACC].lastinfo = i;
      mj_forward(m, d);
      return;
    }
  }
}



//-------------------------- solver components -----------------------------------------------------

// args for internal functions in mj_fwdPosition
struct mjFwdPositionArgs_ {
  const mjModel* m;
  mjData* d;
};
typedef struct mjFwdPositionArgs_ mjFwdPositionArgs;

// wrapper for mj_crb and mj_factorM
void* mj_inertialThreaded(void* args) {
  mjFwdPositionArgs* forward_args = (mjFwdPositionArgs*) args;
  mj_crb(forward_args->m, forward_args->d);       // timed internally (POS_INERTIA)
  mj_factorM(forward_args->m, forward_args->d);   // timed internally (POS_INERTIA)
  return NULL;
}

// wrapper for mj_collision
void* mj_collisionThreaded(void* args) {
  mjFwdPositionArgs* forward_args = (mjFwdPositionArgs*) args;
  mj_collision(forward_args->m, forward_args->d);   // timed internally (POS_COLLISION)
  return NULL;
}



// position-dependent computations
void mj_fwdPosition(const mjModel* m, mjData* d) {
  TM_START1;

  TM_START;
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_camlight(m, d);
  mj_flex(m, d);
  mj_tendon(m, d);
  TM_END(mjTIMER_POS_KINEMATICS);

  // no threadpool: inertia and collision on main thread
  if (!d->threadpool) {
    mj_crb(m, d);        // timed internally (POS_INERTIA)
    mj_factorM(m, d);    // timed internally (POS_INERTIA)
    mj_collision(m, d);  // timed internally (POS_COLLISION)
  }

  // have threadpool: inertia and collision on seperate threads
  else {
    mjTask tasks[2];
    mjFwdPositionArgs forward_args;
    forward_args.m = m;
    forward_args.d = d;

    mju_defaultTask(&tasks[0]);
    tasks[0].func = mj_inertialThreaded;
    tasks[0].args = &forward_args;
    mju_threadPoolEnqueue((mjThreadPool*)d->threadpool, &tasks[0]);

    mju_defaultTask(&tasks[1]);
    tasks[1].func = mj_collisionThreaded;
    tasks[1].args = &forward_args;
    mju_threadPoolEnqueue((mjThreadPool*)d->threadpool, &tasks[1]);

    mju_taskJoin(&tasks[0]);
    mju_taskJoin(&tasks[1]);
  }

  TM_RESTART;
  mj_makeConstraint(m, d);
  if (mjENABLED(mjENBL_ISLAND)) {
    mj_island(m, d);
  }
  TM_END(mjTIMER_POS_MAKE);

  TM_RESTART;
  mj_transmission(m, d);
  TM_ADD(mjTIMER_POS_KINEMATICS);

  TM_RESTART;
  mj_projectConstraint(m, d);
  TM_END(mjTIMER_POS_PROJECT);

  TM_END1(mjTIMER_POSITION);
}



// velocity-dependent computations
void mj_fwdVelocity(const mjModel* m, mjData* d) {
  TM_START;

  // flexedge velocity: dense or sparse
  if (mj_isSparse(m)) {
    mju_mulMatVecSparse(d->flexedge_velocity, d->flexedge_J, d->qvel, m->nflexedge,
                        d->flexedge_J_rownnz, d->flexedge_J_rowadr, d->flexedge_J_colind, NULL);
  } else {
    mju_mulMatVec(d->flexedge_velocity, d->flexedge_J, d->qvel, m->nflexedge, m->nv);
  }

  // tendon velocity: dense or sparse
  if (mj_isSparse(m)) {
    mju_mulMatVecSparse(d->ten_velocity, d->ten_J, d->qvel, m->ntendon,
                        d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind, NULL);
  } else {
    mju_mulMatVec(d->ten_velocity, d->ten_J, d->qvel, m->ntendon, m->nv);
  }

  // actuator velocity: always dense
  mju_mulMatVec(d->actuator_velocity, d->actuator_moment, d->qvel, m->nu, m->nv);

  // com-based velocities, passive forces, constraint references
  mj_comVel(m, d);
  mj_passive(m, d);
  mj_referenceConstraint(m, d);

  // compute qfrc_bias with abbreviated RNE (without acceleration)
  mj_rne(m, d, 0, d->qfrc_bias);

  TM_END(mjTIMER_VELOCITY);
}



// returns the next act given the current act_dot, after clamping
static mjtNum nextActivation(const mjModel* m, const mjData* d,
                             int actuator_id, int act_adr, mjtNum act_dot) {
  mjtNum act = d->act[act_adr];

  if (m->actuator_dyntype[actuator_id] == mjDYN_FILTEREXACT) {
    // exact filter integration
    // act_dot(0) = (ctrl-act(0)) / tau
    // act(h) = act(0) + (ctrl-act(0)) (1 - exp(-h / tau))
    //        = act(0) + act_dot(0) * tau * (1 - exp(-h / tau))
    mjtNum tau = mju_max(mjMINVAL, m->actuator_dynprm[actuator_id * mjNDYN]);
    act = act + act_dot * tau * (1 - mju_exp(-m->opt.timestep / tau));
  } else {
    // Euler integration
    act = act + act_dot * m->opt.timestep;
  }

  // clamp to actrange
  if (m->actuator_actlimited[actuator_id]) {
    mjtNum* actrange = m->actuator_actrange + 2 * actuator_id;
    act = mju_clip(act, actrange[0], actrange[1]);
  }

  return act;
}



// clamp vector to range
static void clampVec(mjtNum* vec, const mjtNum* range, const mjtByte* limited, int n,
                      const int* index) {
  for (int i=0; i < n; i++) {
    int j = index ? index[i] : i;
    if (limited[i]) {
      vec[j] = mju_clip(vec[j], range[2*i], range[2*i + 1]);
    }
  }
}



// return number of dofs given joint type
static int jnt_dofnum(mjtJoint type) {
  if (type == mjJNT_FREE) {
    return 6;
  }

  if (type == mjJNT_BALL) {
    return 3;
  }

  return 1;
}



// (qpos, qvel, ctrl, act) => (qfrc_actuator, actuator_force, act_dot)
void mj_fwdActuation(const mjModel* m, mjData* d) {
  TM_START;
  int nv = m->nv, nu = m->nu;
  mjtNum gain, bias, tau;
  mjtNum *prm, *moment = d->actuator_moment, *force = d->actuator_force;

  // clear actuator_force
  mju_zero(force, nu);

  // disabled or no actuation: return
  if (nu == 0 || mjDISABLED(mjDSBL_ACTUATION)) {
    mju_zero(d->qfrc_actuator, nv);
    return;
  }

  // local, clamped copy of ctrl
  mj_markStack(d);
  mjtNum *ctrl = mj_stackAllocNum(d, nu);
  mju_copy(ctrl, d->ctrl, nu);
  if (!mjDISABLED(mjDSBL_CLAMPCTRL)) {
    clampVec(ctrl, m->actuator_ctrlrange, m->actuator_ctrllimited, nu, NULL);
  }

  // check controls, set all to 0 if any are bad
  for (int i=0; i < nu; i++) {
    if (mju_isBad(ctrl[i])) {
      mj_warning(d, mjWARN_BADCTRL, i);
      mju_zero(ctrl, nu);
      break;
    }
  }

  // act_dot for stateful actuators
  for (int i=0; i < nu; i++) {
    int act_first = m->actuator_actadr[i];
    if (act_first < 0) {
      continue;
    }

    // zero act_dot for actuator plugins
    if (m->actuator_actnum[i]) {
      mju_zero(d->act_dot + act_first, m->actuator_actnum[i]);
    }

    // extract info
    prm = m->actuator_dynprm + i*mjNDYN;

    // index into the last element in act. For most actuators it's also the
    // first element, but actuator plugins might store their own state in act.
    int act_last = act_first + m->actuator_actnum[i] - 1;

    // compute act_dot according to dynamics type
    switch ((mjtDyn) m->actuator_dyntype[i]) {
    case mjDYN_INTEGRATOR:          // simple integrator
      d->act_dot[act_last] = ctrl[i];
      break;

    case mjDYN_FILTER:              // linear filter: prm = tau
    case mjDYN_FILTEREXACT:
      tau = mju_max(mjMINVAL, prm[0]);
      d->act_dot[act_last] = (ctrl[i] - d->act[act_last]) / tau;
      break;

    case mjDYN_MUSCLE:              // muscle model: prm = (tau_act, tau_deact)
      d->act_dot[act_last] = mju_muscleDynamics(
          ctrl[i], d->act[act_last], prm);
      break;

    default:                        // user dynamics
      if (mjcb_act_dyn) {
        if (m->actuator_actnum[i] == 1) {
          // scalar activation dynamics, get act_dot
          d->act_dot[act_last] = mjcb_act_dyn(m, d, i);
        } else {
          // higher-order dynamics, mjcb_act_dyn writes into act_dot directly
          mjcb_act_dyn(m, d, i);
        }
      }
    }
  }

  // get act_dot from actuator plugins
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->capabilityflags & mjPLUGIN_ACTUATOR) {
        if (plugin->actuator_act_dot) {
          plugin->actuator_act_dot(m, d, i);
        }
      }
    }
  }

  // force = gain .* [ctrl/act] + bias
  for (int i=0; i < nu; i++) {
    // skip if disabled
    if (mj_actuatorDisabled(m, i)) {
      continue;
    }

    // skip actuator plugins -- these are handled after builtin actuator types
    if (m->actuator_plugin[i] >= 0) {
      continue;
    }

    // extract gain info
    prm = m->actuator_gainprm + mjNGAIN*i;

    // handle according to gain type
    switch ((mjtGain) m->actuator_gaintype[i]) {
    case mjGAIN_FIXED:              // fixed gain: prm = gain
      gain = prm[0];
      break;

    case mjGAIN_AFFINE:             // affine: prm = [const, kp, kv]
      gain = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
      break;

    case mjGAIN_MUSCLE:             // muscle gain
      gain = mju_muscleGain(d->actuator_length[i],
                            d->actuator_velocity[i],
                            m->actuator_lengthrange+2*i,
                            m->actuator_acc0[i],
                            prm);
      break;

    default:                        // user gain
      if (mjcb_act_gain) {
        gain = mjcb_act_gain(m, d, i);
      } else {
        gain = 1;
      }
    }

    // set force = gain .* [ctrl/act]
    if (m->actuator_actadr[i] == -1) {
      force[i] = gain * ctrl[i];
    } else {
      // use last activation variable associated with actuator i
      int act_adr = m->actuator_actadr[i] + m->actuator_actnum[i] - 1;

      mjtNum act;
      if (m->actuator_actearly[i]) {
        act = nextActivation(m, d, i, act_adr, d->act_dot[act_adr]);
      } else {
        act = d->act[act_adr];
      }
      force[i] = gain * act;
    }

    // extract bias info
    prm = m->actuator_biasprm + mjNBIAS*i;

    // handle according to bias type
    switch ((mjtBias) m->actuator_biastype[i]) {
    case mjBIAS_NONE:               // none
      bias = 0.0;
      break;

    case mjBIAS_AFFINE:             // affine: prm = [const, kp, kv]
      bias = prm[0] + prm[1]*d->actuator_length[i] + prm[2]*d->actuator_velocity[i];
      break;

    case mjBIAS_MUSCLE:             // muscle passive force
      bias =  mju_muscleBias(d->actuator_length[i],
                             m->actuator_lengthrange+2*i,
                             m->actuator_acc0[i],
                             prm);
      break;

    default:                        // user bias
      if (mjcb_act_bias) {
        bias = mjcb_act_bias(m, d, i);
      } else {
        bias = 0;
      }
    }

    // add bias
    force[i] += bias;
  }

  // handle actuator plugins
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->capabilityflags & mjPLUGIN_ACTUATOR) {
        if (!plugin->compute) {
          mjERROR("`compute` is a null function pointer for plugin at slot %d", slot);
        }
        plugin->compute(m, d, i, mjPLUGIN_ACTUATOR);
      }
    }
  }

  // clamp actuator_force
  clampVec(force, m->actuator_forcerange, m->actuator_forcelimited, nu, NULL);

  // qfrc_actuator = moment' * force
  mju_mulMatTVec(d->qfrc_actuator, moment, force, nu, nv);

  // actuator-level gravity compensation
  if (m->ngravcomp && !mjDISABLED(mjDSBL_GRAVITY) && mju_norm3(m->opt.gravity)) {
    int njnt = m->njnt;
    for (int i=0; i < njnt; i++) {
      // skip if gravcomp added as passive force
      if (!m->jnt_actgravcomp[i]) {
        continue;
      }

      // add gravcomp force
      int dofnum = jnt_dofnum(m->jnt_type[i]);
      int dofadr = m->jnt_dofadr[i];
      mju_addTo(d->qfrc_actuator + dofadr, d->qfrc_gravcomp + dofadr, dofnum);
    }
  }

  // clamp qfrc_actuator to joint-level actuator force limits
  clampVec(d->qfrc_actuator, m->jnt_actfrcrange, m->jnt_actfrclimited, m->njnt, m->jnt_dofadr);

  mj_freeStack(d);
  TM_END(mjTIMER_ACTUATION);
}



// add up all non-constraint forces, compute qacc_smooth
void mj_fwdAcceleration(const mjModel* m, mjData* d) {
  int nv = m->nv;

  // qfrc_smooth = sum of all non-constraint forces
  mju_sub(d->qfrc_smooth, d->qfrc_passive, d->qfrc_bias, nv);    // qfrc_bias is negative
  mju_addTo(d->qfrc_smooth, d->qfrc_applied, nv);
  mju_addTo(d->qfrc_smooth, d->qfrc_actuator, nv);
  mj_xfrcAccumulate(m, d, d->qfrc_smooth);

  // qacc_smooth = M \ qfrc_smooth
  mj_solveM(m, d, d->qacc_smooth, d->qfrc_smooth, 1);
}



// warmstart/init solver
static void warmstart(const mjModel* m, mjData* d) {
  int nv = m->nv, nefc = d->nefc;

  // warmstart with best of (qacc_warmstart, qacc_smooth)
  if (!mjDISABLED(mjDSBL_WARMSTART)) {
    mj_markStack(d);
    mjtNum* jar = mj_stackAllocNum(d, nefc);

    // start with qacc = qacc_warmstart
    mju_copy(d->qacc, d->qacc_warmstart, nv);

    // compute jar(qacc_warmstart)
    mj_mulJacVec(m, d, jar, d->qacc_warmstart);
    mju_subFrom(jar, d->efc_aref, nefc);

    // update constraints, save cost(qacc_warmstart)
    mjtNum cost_warmstart;
    mj_constraintUpdate(m, d, jar, &cost_warmstart, 0);

    // PGS
    if (m->opt.solver == mjSOL_PGS) {
      // cost(force_warmstart)
      mjtNum PGS_warmstart = mju_dot(d->efc_force, d->efc_b, nefc);
      mjtNum* ARf = mj_stackAllocNum(d, nefc);
      if (mj_isSparse(m))
        mju_mulMatVecSparse(ARf, d->efc_AR, d->efc_force, nefc,
                            d->efc_AR_rownnz, d->efc_AR_rowadr,
                            d->efc_AR_colind, NULL);
      else {
        mju_mulMatVec(ARf, d->efc_AR, d->efc_force, nefc, nefc);
      }
      PGS_warmstart += 0.5*mju_dot(d->efc_force, ARf, nefc);

      // use zero if better
      if (PGS_warmstart > 0) {
        mju_zero(d->efc_force, nefc);
        mju_zero(d->qfrc_constraint, nv);
      }
    }

    // non-PGS
    else {
      // add Gauss to cost(qacc_warmstart)
      mjtNum* Ma = mj_stackAllocNum(d, nv);
      mj_mulM(m, d, Ma, d->qacc_warmstart);
      for (int i=0; i < nv; i++) {
        cost_warmstart += 0.5*(Ma[i]-d->qfrc_smooth[i])*(d->qacc_warmstart[i]-d->qacc_smooth[i]);
      }

      // cost(qacc_smooth)
      mjtNum cost_smooth;
      mj_constraintUpdate(m, d, d->efc_b, &cost_smooth, 0);

      // use qacc_smooth if better
      if (cost_warmstart > cost_smooth) {
        mju_copy(d->qacc, d->qacc_smooth, nv);
      }
    }

    mj_freeStack(d);
  }

  // coldstart with qacc = qacc_smooth, efc_force = 0
  else {
    mju_copy(d->qacc, d->qacc_smooth, nv);
    mju_zero(d->efc_force, nefc);
  }
}



// struct encapsulating arguments to thread task
struct mjSolIslandArgs_ {
  const mjModel* m;
  mjData* d;
  int island;
};
typedef struct mjSolIslandArgs_ mjSolIslandArgs;

// extract arguments, pass to solver
void* mj_solCG_island_wrapper(void* args) {
  mjSolIslandArgs* solargs = (mjSolIslandArgs*) args;
  mj_solCG_island(solargs->m, solargs->d, solargs->island, solargs->m->opt.iterations);
  return NULL;
}




// CG solver, multi-threaded over islands
void mj_solCG_island_multithreaded(const mjModel* m, mjData* d) {
  mj_markStack(d);
  // allocate array of arguments to be passed to threads
  mjSolIslandArgs* sol_cg_island_args =
    mj_stackAllocByte(d, sizeof(mjSolIslandArgs) * d->nisland, _Alignof(mjSolIslandArgs));
  mjTask* tasks = mj_stackAllocByte(d, sizeof(mjTask)  * d->nisland, _Alignof(mjTask));

  for (int island = 0; island < d->nisland; ++island) {
    sol_cg_island_args[island].m = m;
    sol_cg_island_args[island].d = d;
    sol_cg_island_args[island].island = island;

    mju_defaultTask(&tasks[island]);
    tasks[island].func = mj_solCG_island_wrapper;
    tasks[island].args = &sol_cg_island_args[island];
    mju_threadPoolEnqueue((mjThreadPool*)d->threadpool, &tasks[island]);
  }

  for (int island = 0; island < d->nisland; ++island) {
    mju_taskJoin(&tasks[island]);
  }

  mj_freeStack(d);
}



// compute efc_b, efc_force, qfrc_constraint; update qacc
void mj_fwdConstraint(const mjModel* m, mjData* d) {
  TM_START;
  int nv = m->nv, nefc = d->nefc, nisland = d->nisland;

  // always clear qfrc_constraint
  mju_zero(d->qfrc_constraint, nv);

  // no constraints: copy unconstrained acc, clear forces, return
  if (!nefc) {
    mju_copy(d->qacc, d->qacc_smooth, nv);
    mju_copy(d->qacc_warmstart, d->qacc_smooth, nv);
    mju_zeroInt(d->solver_niter, mjNISLAND);
    TM_END(mjTIMER_CONSTRAINT);
    return;
  }

  // compute efc_b = J*qacc_smooth - aref
  mj_mulJacVec(m, d, d->efc_b, d->qacc_smooth);
  mju_subFrom(d->efc_b, d->efc_aref, nefc);

  // warmstart solver
  warmstart(m, d);
  mju_zeroInt(d->solver_niter, mjNISLAND);

  // check if islands are supported
  int islands_supported = mjENABLED(mjENBL_ISLAND)  &&
                          d->nisland > 0            &&
                          m->opt.solver == mjSOL_CG &&
                          m->opt.noslip_iterations == 0;

  // run solver over constraint islands
  if (islands_supported) {
    // no threadpool, loop over islands
    if (!d->threadpool) {
      for (int island=0; island < nisland; island++) {
        mj_solCG_island(m, d, island, m->opt.iterations);
      }
    }
    else {
      // solve using threads
      mj_solCG_island_multithreaded(m, d);
    }
    d->solver_nisland = nisland;
  }

  // run solver over all constraints
  else {
    switch ((mjtSolver) m->opt.solver) {
    case mjSOL_PGS:                     // PGS
      mj_solPGS(m, d, m->opt.iterations);
      break;

    case mjSOL_CG:                      // CG
      mj_solCG(m, d, m->opt.iterations);
      break;

    case mjSOL_NEWTON:                  // Newton
      mj_solNewton(m, d, m->opt.iterations);
      break;

    default:
      mjERROR("unknown solver type %d", m->opt.solver);
    }

    // one (monolithic) island
    d->solver_nisland = 1;
  }

  // save result for next step warmstart
  mju_copy(d->qacc_warmstart, d->qacc, nv);

  // run noslip solver if enabled
  if (m->opt.noslip_iterations > 0) {
    mj_solNoSlip(m, d, m->opt.noslip_iterations);
  }

  TM_END(mjTIMER_CONSTRAINT);
}



//-------------------------- integrators  ----------------------------------------------------------

// advance state and time given activation derivatives, acceleration, and optional velocity
static void mj_advance(const mjModel* m, mjData* d,
                       const mjtNum* act_dot, const mjtNum* qacc, const mjtNum* qvel) {
  // advance activations
  if (m->na && !mjDISABLED(mjDSBL_ACTUATION)) {
    int nu = m->nu;
    for (int i=0; i < nu; i++) {
      int actadr = m->actuator_actadr[i];
      int actadr_end = actadr + m->actuator_actnum[i];
      for (int j=actadr; j < actadr_end; j++) {
        // if disabled, set act_dot to 0
        d->act[j] = nextActivation(m, d, i, j, mj_actuatorDisabled(m, i) ? 0 : act_dot[j]);
      }
    }
  }

  // advance velocities
  mju_addToScl(d->qvel, qacc, m->opt.timestep, m->nv);

  // advance positions with qvel if given, d->qvel otherwise (semi-implicit)
  mj_integratePos(m, d->qpos, qvel ? qvel : d->qvel, m->opt.timestep);

  // advance time
  d->time += m->opt.timestep;

  // advance plugin states
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i = 0; i < m->nplugin; ++i) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->advance) {
        plugin->advance(m, d, i);
      }
    }
  }
}

// Euler integrator, semi-implicit in velocity, possibly skipping factorisation
void mj_EulerSkip(const mjModel* m, mjData* d, int skipfactor) {
  TM_START;
  int nv = m->nv, nM = m->nM;
  mj_markStack(d);
  mjtNum* qfrc = mj_stackAllocNum(d, nv);
  mjtNum* qacc = mj_stackAllocNum(d, nv);

  // check for dof damping if disable flag is not set
  int dof_damping = 0;
  if (!mjDISABLED(mjDSBL_EULERDAMP)) {
    for (int i=0; i < nv; i++) {
      if (m->dof_damping[i] > 0) {
        dof_damping = 1;
        break;
      }
    }
  }

  // no damping or disabled: explicit velocity integration
  if (!dof_damping) {
    mju_copy(qacc, d->qacc, nv);
  }

  // damping: integrate implicitly
  else {
    if (!skipfactor) {
      mjtNum* MhB = mj_stackAllocNum(d, nM);

      // MhB = M + h*diag(B)
      mju_copy(MhB, d->qM, m->nM);
      for (int i=0; i < nv; i++) {
        MhB[m->dof_Madr[i]] += m->opt.timestep * m->dof_damping[i];
      }

      // factor
      mj_factorI(m, d, MhB, d->qH, d->qHDiagInv, 0);
    }

    // solve
    mju_add(qfrc, d->qfrc_smooth, d->qfrc_constraint, nv);
    mju_copy(qacc, qfrc, m->nv);
    mj_solveLD(m, qacc, 1, d->qH, d->qHDiagInv);
  }

  // advance state and time
  mj_advance(m, d, d->act_dot, qacc, NULL);

  mj_freeStack(d);

  TM_END(mjTIMER_ADVANCE);
}



// Euler integrator, semi-implicit in velocity
void mj_Euler(const mjModel* m, mjData* d) {
  mj_EulerSkip(m, d, 0);
}



// RK4 tableau
const mjtNum RK4_A[9] = {
  0.5,    0,      0,
  0,      0.5,    0,
  0,      0,      1
};

const mjtNum RK4_B[4] = {
  1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0
};


// Runge Kutta explicit order-N integrator
//  (A,B) is the tableau, C is set to row_sum(A)
void mj_RungeKutta(const mjModel* m, mjData* d, int N) {
  int nv = m->nv, nq = m->nq, na = m->na;
  mjtNum h = m->opt.timestep, time = d->time;
  mjtNum C[9], T[9], *X[10], *F[10], *dX;
  const mjtNum* A = (N == 4 ? RK4_A : 0);
  const mjtNum* B = (N == 4 ? RK4_B : 0);

  // check order
  if (!A) {
    mjERROR("supported RK orders: N=4");
  }

  // allocate space for intermediate solutions
  mj_markStack(d);
  dX = mj_stackAllocNum(d, 2*nv+na);
  for (int i=0; i < N; i++) {
    X[i] = mj_stackAllocNum(d, nq+nv+na);
    F[i] = mj_stackAllocNum(d, nv+na);
  }

  // precompute C and T;  C,T,A have size (N-1)
  for (int i=1; i < N; i++) {
    // C(i) = sum_j A(i,j)
    C[i-1] = 0;
    for (int j=0; j < i; j++) {
      C[i-1] += A[(i-1)*(N-1)+j];
    }

    // compute T
    T[i-1] = d->time + C[i-1]*h;
  }

  // init X[0], F[0]; mj_forward() was already called
  mju_copy(X[0], d->qpos, nq);
  mju_copy(X[0]+nq, d->qvel, nv);
  mju_copy(F[0], d->qacc, nv);
  if (na) {
    mju_copy(X[0]+nq+nv, d->act, na);
    mju_copy(F[0]+nv, d->act_dot, na);
  }

  // compute the remaining X[i], F[i]
  for (int i=1; i < N; i++) {
    // compute dX
    mju_zero(dX, 2*nv+na);
    for (int j=0; j < i; j++) {
      mju_addToScl(dX, X[j]+nq, A[(i-1)*(N-1)+j], nv);
      mju_addToScl(dX+nv, F[j], A[(i-1)*(N-1)+j], nv+na);
    }

    // compute X[i] = X[0] '+' dX
    mju_copy(X[i], X[0], nq+nv+na);
    mj_integratePos(m, X[i], dX, h);
    mju_addToScl(X[i]+nq, dX+nv, h, nv+na);

    // set X[i], T[i-1] in mjData
    mju_copy(d->qpos, X[i], nq);
    mju_copy(d->qvel, X[i]+nq, nv);
    if (na) {
      mju_copy(d->act, X[i]+nq+nv, na);
    }
    d->time = T[i-1];

    // evaluate F[i]
    mj_forwardSkip(m, d, mjSTAGE_NONE, 1);  // 1: do not recompute sensors and energy
    mju_copy(F[i], d->qacc, nv);
    if (na) {
      mju_copy(F[i]+nv, d->act_dot, na);
    }
  }

  // compute dX for final update (using B instead of A)
  mju_zero(dX, 2*nv+na);
  for (int j=0; j < N; j++) {
    mju_addToScl(dX, X[j]+nq, B[j], nv);
    mju_addToScl(dX+nv, F[j], B[j], nv+na);
  }

  // reset state and time
  d->time = time;
  mju_copy(d->qpos, X[0], nq);
  mju_copy(d->qvel, X[0]+nq, nv);
  mju_copy(d->act, X[0]+nq+nv, na);

  // advance state and time
  mj_advance(m, d, dX+2*nv, dX+nv, dX);

  mj_freeStack(d);
}



// fully implicit in velocity, possibly skipping factorization
void mj_implicitSkip(const mjModel* m, mjData* d, int skipfactor) {
  TM_START;
  int nv = m->nv;

  mj_markStack(d);
  mjtNum* qfrc = mj_stackAllocNum(d, nv);
  mjtNum* qacc = mj_stackAllocNum(d, nv);

  // set qfrc = qfrc_smooth + qfrc_constraint
  mju_add(qfrc, d->qfrc_smooth, d->qfrc_constraint, nv);

  // IMPLICIT
  if (m->opt.integrator == mjINT_IMPLICIT) {
    if (!skipfactor) {
      // compute analytical derivative qDeriv
      mjd_smooth_vel(m, d, /* flg_bias = */ 1);

      // set qLU = qM
      mj_copyM2DSparse(m, d, d->qLU, d->qM);

      // set qLU = qM - dt*qDeriv
      mju_addToScl(d->qLU, d->qDeriv, -m->opt.timestep, m->nD);

      // factorize qLU
      int* scratch = mj_stackAllocInt(d, nv);
      mju_factorLUSparse(d->qLU, nv, scratch, d->D_rownnz, d->D_rowadr, d->D_colind);
    }

    // solve for qacc: (qM - dt*qDeriv) * qacc = qfrc
    mju_solveLUSparse(qacc, d->qLU, qfrc, nv, d->D_rownnz, d->D_rowadr, d->D_colind);
  }

  // IMPLICITFAST
  else if (m->opt.integrator == mjINT_IMPLICITFAST) {
    if (!skipfactor) {
      // compute analytical derivative qDeriv; skip rne derivative
      mjd_smooth_vel(m, d, /* flg_bias = */ 0);

      // modified mass matrix MhB = qDeriv[Lower]
      mjtNum* MhB = mj_stackAllocNum(d, m->nM);
      mj_copyD2MSparse(m, d, MhB, d->qDeriv);

      // set MhB = M - dt*qDeriv
      mju_addScl(MhB, d->qM, MhB, -m->opt.timestep, m->nM);

      // factorize
      mj_factorI(m, d, MhB, d->qH, d->qHDiagInv, NULL);
    }

    // solve for qacc: (qM - dt*qDeriv) * qacc = qfrc
    mju_copy(qacc, qfrc, m->nv);
    mj_solveLD(m, qacc, 1, d->qH, d->qHDiagInv);
  } else {
    mjERROR("integrator must be implicit or implicitfast");
  }

  // advance state and time
  mj_advance(m, d, d->act_dot, qacc, NULL);

  mj_freeStack(d);

  TM_END(mjTIMER_ADVANCE);
}



// fully implicit in velocity
void mj_implicit(const mjModel* m, mjData* d) {
  mj_implicitSkip(m, d, 0);
}



//-------------------------- top-level API ---------------------------------------------------------

// forward dynamics with skip; skipstage is mjtStage
void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
  TM_START;

  // position-dependent
  if (skipstage < mjSTAGE_POS) {
    mj_fwdPosition(m, d);
    if (!skipsensor) {
      mj_sensorPos(m, d);
    }
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyPos(m, d);
    }
  }

  // velocity-dependent
  if (skipstage < mjSTAGE_VEL) {
    mj_fwdVelocity(m, d);
    if (!skipsensor) {
      mj_sensorVel(m, d);
    }
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyVel(m, d);
    }
  }

  // acceleration-dependent
  if (mjcb_control && !mjDISABLED(mjDSBL_ACTUATION)) {
    mjcb_control(m, d);
  }

  mj_fwdActuation(m, d);
  mj_fwdAcceleration(m, d);
  mj_fwdConstraint(m, d);
  if (!skipsensor) {
    mj_sensorAcc(m, d);
  }

  TM_END(mjTIMER_FORWARD);
}



// forward dynamics
void mj_forward(const mjModel* m, mjData* d) {
  mj_forwardSkip(m, d, mjSTAGE_NONE, 0);
}



// advance simulation using control callback
void mj_step(const mjModel* m, mjData* d) {
  TM_START;

  // common to all integrators
  mj_checkPos(m, d);
  mj_checkVel(m, d);
  mj_forward(m, d);
  mj_checkAcc(m, d);

  // compare forward and inverse solutions if enabled
  if (mjENABLED(mjENBL_FWDINV)) {
    mj_compareFwdInv(m, d);
  }

  // use selected integrator
  switch ((mjtIntegrator) m->opt.integrator) {
  case mjINT_EULER:
    mj_Euler(m, d);
    break;

  case mjINT_RK4:
    mj_RungeKutta(m, d, 4);
    break;

  case mjINT_IMPLICIT:
  case mjINT_IMPLICITFAST:
    mj_implicit(m, d);
    break;

  default:
    mjERROR("invalid integrator");
  }

  TM_END(mjTIMER_STEP);
}



// advance simulation in two phases: before input is set by user
void mj_step1(const mjModel* m, mjData* d) {
  TM_START;
  mj_checkPos(m, d);
  mj_checkVel(m, d);
  mj_fwdPosition(m, d);
  mj_sensorPos(m, d);
  mj_energyPos(m, d);
  mj_fwdVelocity(m, d);
  mj_sensorVel(m, d);
  mj_energyVel(m, d);
  if (mjcb_control) {
    mjcb_control(m, d);
  }
  TM_END(mjTIMER_STEP);
}


//   >>>>   user can modify ctrl and q/xfrc_applied between step1 and step2   <<<<


// advance simulation in two phases: after input is set by user
void mj_step2(const mjModel* m, mjData* d) {
  TM_START;
  mj_fwdActuation(m, d);
  mj_fwdAcceleration(m, d);
  mj_fwdConstraint(m, d);
  mj_sensorAcc(m, d);
  mj_checkAcc(m, d);

  // compare forward and inverse solutions if enabled
  if (mjENABLED(mjENBL_FWDINV)) {
    mj_compareFwdInv(m, d);
  }

  // integrate with Euler or implicit; RK4 defaults to Euler
  if (m->opt.integrator == mjINT_IMPLICIT || m->opt.integrator == mjINT_IMPLICITFAST) {
    mj_implicit(m, d);
  } else {
    mj_Euler(m, d);
  }

  d->timer[mjTIMER_STEP].number--;
  TM_END(mjTIMER_STEP);
}
