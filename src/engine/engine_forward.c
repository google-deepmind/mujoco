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
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjplugin.h>
#include "engine/engine_callback.h"
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_derivative.h"
#include "engine/engine_inverse.h"
#include "engine/engine_island.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_passive.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sensor.h"
#include "engine/engine_sleep.h"
#include "engine/engine_solver.h"
#include "engine/engine_support.h"
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_thread.h"



//--------------------------- check values ---------------------------------------------------------

// check positions, reset if bad
void mj_checkPos(const mjModel* m, mjData* d) {
  int nq = m->nq;
  const mjtNum* qpos = d->qpos;
  for (int i=0; i < nq; i++) {
    if (mju_isBad(qpos[i])) {
      mj_warning(d, mjWARN_BADQPOS, i);
      if (!mjDISABLED(mjDSBL_AUTORESET)) {
        mj_resetData(m, d);
      }
      d->warning[mjWARN_BADQPOS].number++;
      d->warning[mjWARN_BADQPOS].lastinfo = i;
      return;
    }
  }
}


// check velocities, reset if bad
void mj_checkVel(const mjModel* m, mjData* d) {
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;

  for (int j=0; j < nv; j++) {
    int i = sleep_filter ? d->dof_awake_ind[j] : j;

    if (mju_isBad(d->qvel[i])) {
      mj_warning(d, mjWARN_BADQVEL, i);
      if (!mjDISABLED(mjDSBL_AUTORESET)) {
        mj_resetData(m, d);
      }
      d->warning[mjWARN_BADQVEL].number++;
      d->warning[mjWARN_BADQVEL].lastinfo = i;
      return;
    }
  }
}


// check accelerations, reset if bad
void mj_checkAcc(const mjModel* m, mjData* d) {
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;

  for (int j=0; j < nv; j++) {
    int i = sleep_filter ? d->dof_awake_ind[j] : j;

    if (mju_isBad(d->qacc[i])) {
      mj_warning(d, mjWARN_BADQACC, i);
      if (!mjDISABLED(mjDSBL_AUTORESET)) {
        mj_resetData(m, d);
      }
      d->warning[mjWARN_BADQACC].number++;
      d->warning[mjWARN_BADQACC].lastinfo = i;
      if (!mjDISABLED(mjDSBL_AUTORESET)) {
        mj_forward(m, d);
      }
      return;
    }
  }
}


//-------------------------- solver components -----------------------------------------------------

// kinematics-related computations
void mj_fwdKinematics(const mjModel* m, mjData* d) {
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_camlight(m, d);
  mj_flex(m, d);
  mj_tendon(m, d);
  if (mj_wakeTendon(m, d)) {
    mj_updateSleep(m, d);
  }
}

// position-dependent computations
void mj_fwdPosition(const mjModel* m, mjData* d) {
  TM_START1;

  // clear position-dependent flags for lazy evaluation
  d->flg_energypos = 0;

  TM_START;
  mj_fwdKinematics(m, d);

  TM_END(mjTIMER_POS_KINEMATICS);

  // inertia, timed internally (POS_INERTIA)
  mj_makeM(m, d);
  mj_factorM(m, d);

  // collision, timed internally (POS_COLLISION)
  mj_collision(m, d);

  if (mj_wakeCollision(m, d)) {
    mj_updateSleep(m, d);
    mj_collision(m, d);
  }

  if (mj_wakeEquality(m, d)) {
    mj_updateSleep(m, d);
  }

  TM_RESTART;
  mj_makeConstraint(m, d);
  mj_island(m, d);
  TM_END(mjTIMER_POS_MAKE);

  TM_RESTART;
  mj_projectConstraint(m, d);
  TM_END(mjTIMER_POS_PROJECT);

  TM_RESTART;
  mj_transmission(m, d);
  TM_ADD(mjTIMER_POS_KINEMATICS);

  // implicit effective metric Mtilde = M + K: build (or deactivate) for this step. Arena
  // lifetime and skip semantics mirror the constraint data: built once per position stage,
  // value-refreshed in the velocity stage, consumed downstream.
  mjd_effBuild(m, d, mj_flexCG(m), /*flg_factor=*/1);

  TM_END1(mjTIMER_POSITION);
}


// velocity-dependent computations
void mj_fwdVelocity(const mjModel* m, mjData* d) {
  TM_START;

  // clear velocity-dependent flags for lazy evaluation
  d->flg_subtreevel = 0;
  d->flg_energyvel = 0;

  // flexedge velocity: skip interp and rigid flexes (edge Jacobians are zero)
  mju_zero(d->flexedge_velocity, m->nflexedge);
  for (int f = 0; f < m->nflex; f++) {
    if (m->flex_rigid[f] || m->flex_interp[f]) continue;
    int adr = m->flex_edgeadr[f];
    int num = m->flex_edgenum[f];
    mju_mulMatVecSparse(d->flexedge_velocity + adr, d->flexedge_J, d->qvel, num,
                        m->flexedge_J_rownnz + adr, m->flexedge_J_rowadr + adr,
                        m->flexedge_J_colind, NULL);
  }

  // tendon velocity: always sparse
  mju_mulMatVecSparse(d->ten_velocity, d->ten_J, d->qvel, m->ntendon,
                      m->ten_J_rownnz, m->ten_J_rowadr, m->ten_J_colind, NULL);

  // actuator velocity: always sparse
  if (!mjDISABLED(mjDSBL_ACTUATION)) {
    mju_mulMatVecSparse(d->actuator_velocity, d->actuator_moment, d->qvel, m->nout,
                        d->moment_rownnz, d->moment_rowadr, d->moment_colind, NULL);
  } else {
    mju_zero(d->actuator_velocity, m->nout);
  }

  // com-based velocities, passive forces, constraint references
  mj_comVel(m, d);
  mj_passive(m, d);
  mj_referenceConstraint(m, d);

  // compute qfrc_bias with abbreviated RNE (without acceleration)
  mj_rne(m, d, 0, d->qfrc_bias);

  // add bias force due to tendon armature
  mj_tendonBias(m, d, d->qfrc_bias);

  mjd_effShift(m, d);

  TM_END(mjTIMER_VELOCITY);
}


// helper for DC motor: computes control voltage from PID state
static mjtNum dcmotorVoltage(mjtNum ctrl, mjtNum length, mjtNum velocity,
                             mjtNum x_I, const mjtNum* gainprm) {
  int input_mode = (int)gainprm[8];
  mjtNum Vmax = gainprm[7];
  mjtNum voltage;

  // get voltage
  if (input_mode > 0) {
    mjtNum kp = gainprm[4];  // proportional gain
    mjtNum ki = gainprm[5];  // integral gain
    mjtNum kd = gainprm[6];  // derivative gain

    if (input_mode == 1) {
      // position mode
      voltage = kp * (ctrl - length) + ki * x_I - kd * velocity;
    } else {
      // velocity mode
      voltage = kp * (ctrl - velocity) + ki * (x_I - length);
    }
  } else {
    voltage = ctrl;
  }

  // clip voltage
  if (Vmax > 0) voltage = mju_clip(voltage, -Vmax, Vmax);

  return voltage;
}


// clamp vector to range
static void clampVec(mjtNum* vec, const mjtNum* range, const mjtBool* limited, int n,
                      const int* index) {
  for (int i=0; i < n; i++) {
    int j = index ? index[i] : i;
    if (limited[i]) {
      vec[j] = mju_clip(vec[j], range[2*i], range[2*i + 1]);
    }
  }
}


// expmap (axis-angle) vector to quaternion
static void expmap2Quat(mjtNum quat[4], const mjtNum v[3]) {
  mjtNum angle = mju_norm3(v);
  if (angle < mjMINVAL) {
    quat[0] = 1;
    quat[1] = quat[2] = quat[3] = 0;
  } else {
    mjtNum axis[3] = {v[0]/angle, v[1]/angle, v[2]/angle};
    mju_axisAngle2Quat(quat, axis, angle);
  }
}


// period of the rotational transmission for wrap-eligible servo actuators, 0 otherwise
static mjtNum wrapPeriod(const mjModel* m, int i) {
  // servo shape: fixed gain, affine bias, matching kp, setpoint input
  mjtDyn dyntype = m->actuator_dyntype[i];
  if (m->actuator_gaintype[i] != mjGAIN_FIXED  ||
      m->actuator_biastype[i] != mjBIAS_AFFINE ||
      m->actuator_gainprm[mjNGAIN*i] != -m->actuator_biasprm[mjNBIAS*i+1] ||
      (dyntype != mjDYN_NONE && dyntype != mjDYN_INTEGRATOR)) {
    return 0;
  }

  const mjtNum* gear = m->actuator_gear+6*m->actuator_outadr[i];
  mjtTrn trntype = m->actuator_trntype[i];

  // site transmission with refsite and purely rotational gear
  if (trntype == mjTRN_SITE && m->actuator_trnid[2*i+1] >= 0 &&
      !gear[0] && !gear[1] && !gear[2]) {
    return 2*mjPI * mju_norm3(gear+3);
  }

  // joint transmission on a ball joint
  if ((trntype == mjTRN_JOINT || trntype == mjTRN_JOINTINPARENT) &&
      m->jnt_type[m->actuator_trnid[2*i]] == mjJNT_BALL) {
    return 2*mjPI * mju_norm3(gear);
  }

  return 0;
}


// representative of setpoint u nearest to length, given period
static mjtNum wrapSetpoint(mjtNum u, mjtNum length, mjtNum period) {
  mjtNum err = u - length;
  return u - period * mju_round(err / period);
}


// (qpos, qvel, ctrl, act) => (qfrc_actuator, actuator_force, act_dot)
void mj_fwdActuation(const mjModel* m, mjData* d) {
  TM_START;
  int nv = m->nv, nu = m->nu, nactuator = m->nactuator, nout = m->nout, ntendon = m->ntendon;
  mjtNum gain, bias, tau;
  mjtNum *force = d->actuator_force;

  // clear actuator_force
  mju_zero(force, nout);

  int sleep_filter = mjENABLED(mjENBL_SLEEP);

  // disabled or no actuation: return
  if (nactuator == 0 || mjDISABLED(mjDSBL_ACTUATION)) {
    mju_zero(d->qfrc_actuator, nv);
    return;
  }

  // any tendon transmission targets with force limits
  int tendon_frclimited = 0;

  // local copy of ctrl
  mj_markStack(d);
  mjtNum *ctrl = mjSTACKALLOC(d, nu, mjtNum);

  // read from ctrl or history buffer for delayed actuators
  for (int i = 0; i < nactuator; i++) {
    int adr = m->actuator_ctrladr[i];
    if (m->actuator_delay[i]) {
      // delayed: read from history buffer (scalar input)
      int interp = m->actuator_history[2*i+1];
      ctrl[adr] = mj_readCtrl(m, d, i, d->time, interp);
    } else {
      mju_copy(ctrl + adr, d->ctrl + adr, m->actuator_ctrlnum[i]);
    }
  }

  // clamp local copy
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
  for (int i=0; i < nactuator; i++) {
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_ACTUATOR, i) == mjS_ASLEEP) {
      continue;
    }

    int act_first = m->actuator_actadr[i];
    if (act_first < 0) {
      continue;
    }

    // addresses of the actuator's input and output blocks
    int uadr = m->actuator_ctrladr[i];
    int oadr = m->actuator_outadr[i];

    // zero act_dot for actuator plugins
    int actnum = m->actuator_actnum[i];
    if (actnum) {
      mju_zero(d->act_dot + act_first, actnum);
    }

    // extract info
    const mjtNum* dynprm = m->actuator_dynprm + i*mjNDYN;
    mjtDyn dyntype = m->actuator_dyntype[i];

    // index into the last element in act. For most actuators it's also the
    // first element, but actuator plugins might store their own state in act
    int act_last = act_first + actnum - 1;

    // compute act_dot according to dynamics type
    switch (dyntype) {
    case mjDYN_INTEGRATOR: {        // simple integrator, one per control
      int num = m->actuator_ctrlnum[i];
      for (int j=0; j < num; j++) {
        d->act_dot[act_last-num+1+j] = ctrl[uadr+j];
      }
      break;
    }

    case mjDYN_FILTER:              // linear filter: dynprm = tau
    case mjDYN_FILTEREXACT:
      tau = mju_max(mjMINVAL, dynprm[0]);
      d->act_dot[act_last] = (ctrl[uadr] - d->act[act_last]) / tau;
      break;

    case mjDYN_MUSCLE:              // muscle model: dynprm = (tau_act, tau_deact)
      d->act_dot[act_last] = mju_muscleDynamics(ctrl[uadr], d->act[act_last], dynprm);
      break;

    case mjDYN_DCMOTOR: {           // DC motor: up to 5 optional states
      const mjtNum* gainprm = m->actuator_gainprm + mjNGAIN*i;

      // verify allocated state size matches parameters; SHOULD NOT OCCUR
      if (mj_dcmotorSlots(dynprm, gainprm).num_slots != actnum) {
        mjERROR("inconsistent state array dimension in DC motor (actuator %d)", i);
      }

      int adr = act_first;
      mjtNum velocity = d->actuator_velocity[oadr];
      mjtNum R = gainprm[0];   // resistance
      mjtNum K = gainprm[1];   // motor constant
      mjtNum ki = gainprm[5];  // integral gain
      mjtNum te = dynprm[0];   // electrical time constant

      // slot order: slew, integral, temperature, bristle, current

      // controller state: slew rate limiting
      mjtNum slew_s = dynprm[7];  // slew rate limit
      if (slew_s > 0) {
        mjtNum u_prev = d->act[adr];
        mjtNum slew = slew_s * m->opt.timestep;
        mjtNum u_eff = mju_clip(ctrl[uadr], u_prev - slew, u_prev + slew);
        d->act_dot[adr] = (u_eff - u_prev) / m->opt.timestep;
        ctrl[uadr] = u_eff;
        adr++;
      }

      // controller state: integral state
      mjtNum x_I = 0;
      if (ki > 0) {
        x_I = d->act[adr];
        int input_mode = (int)gainprm[8];
        mjtNum Imax = dynprm[8];   // integral clamp
        mjtNum act_dot = ctrl[uadr];  // default raw accumulator for voltage and velocity modes

        // position mode
        if (input_mode == 1) {
          act_dot = ctrl[uadr] - d->actuator_length[oadr];
        }

        // clamp act_dot based on integral state
        if (Imax > 0) {
          if (x_I >= Imax) {
            act_dot = mju_min(act_dot, 0);
          } else if (x_I <= -Imax) {
            act_dot = mju_max(act_dot, 0);
          }
        }
        d->act_dot[adr] = act_dot;
        adr++;
      }

      // compute physical voltage to feed into current and temperature equations
      mjtNum V = dcmotorVoltage(ctrl[uadr], d->actuator_length[oadr], velocity, x_I, gainprm);

      // temperature: dT/dt = (R*i^2 - T/RT) / C, where T = delta above ambient
      mjtNum RT = dynprm[2];  // thermal resistance
      if (RT > 0) {
        mjtNum C = dynprm[3];       // thermal capacitance
        mjtNum Ta = dynprm[4];      // ambient temperature
        mjtNum alpha = gainprm[2];  // temperature coefficient
        mjtNum T0 = gainprm[3];     // reference temperature
        mjtNum T = d->act[adr];     // temperature rise above ambient
        R *= 1 + alpha * (T + Ta - T0);

        // get current: from act_last if stateful, from (V - K*omega)/R if stateless
        mjtNum current = (te > 0) ? d->act[act_last] : (V - K * velocity) / R;
        d->act_dot[adr] = (R*current*current - T / RT) / C;
        adr++;
      }

      // LuGre bristle state: dz/dt = v - sigma0 * |v| / g(v) * z
      mjtNum sigma0 = dynprm[5];  // bristle stiffness
      if (sigma0 > 0) {
        const mjtNum* biasprm = m->actuator_biasprm + mjNBIAS*i;
        mjtNum F_C = biasprm[3];  // Coulomb friction
        mjtNum F_S = biasprm[4];  // static friction
        mjtNum v_S = biasprm[5];  // Stribeck velocity
        mjtNum z = d->act[adr];   // bristle state
        mjtNum g = mj_lugreStribeck(velocity, F_C, F_S, v_S);
        mjtNum a = -sigma0 * mju_abs(velocity) / mju_max(mjMINVAL, g);
        d->act_dot[adr] = a * z + velocity;
        adr++;
      }

      // current state: di/dt = (V/R - K/R*omega - i) / te
      if (te > 0) {
        mjtNum dimax = dynprm[1];  // current rate limit (di/dt)_max
        mjtNum i_dot = (V/R - K/R*velocity - d->act[act_last]) / te;
        if (dimax > 0) {
          i_dot = mju_clip(i_dot, -dimax, dimax);
        }
        d->act_dot[act_last] = i_dot;
      }
      break;
    }

    default:                        // user dynamics
      if (mjcb_act_dyn) {
        if (actnum == 1) {
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
  for (int i=0; i < nactuator; i++) {
    // skip if sleeping
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_ACTUATOR, i) == mjS_ASLEEP) {
      continue;
    }

    // skip if disabled
    if (mj_actuatorDisabled(m, i)) {
      continue;
    }

    // skip actuator plugins -- these are handled after builtin actuator types
    if (m->actuator_plugin[i] >= 0) {
      continue;
    }

    // addresses of the actuator's input and output blocks
    int uadr = m->actuator_ctrladr[i];
    int oadr = m->actuator_outadr[i];

    // SO(3) geodesic servo: 3 or 4 inputs and 3 outputs on an SO3 transmission
    if (m->actuator_gaintype[i] == mjGAIN_SO3) {
      mjtNum q_tgt[4];

      // quat input: normalize ctrl directly (zero maps to the identity)
      if (m->actuator_ctrlspec[i] == mjCHART_QUAT) {
        mju_copy4(q_tgt, ctrl + uadr);
        mju_normalize4(q_tgt);
      }

      // expmap input: ctrl block (position) or act block (integrator)
      else {
        mjtNum u[3];
        if (m->actuator_dyntype[i] == mjDYN_NONE) {
          mju_copy3(u, ctrl + uadr);
        } else {
          int act_adr = m->actuator_actadr[i];
          if (m->actuator_actearly[i]) {
            for (int k=0; k < 3; k++) {
              u[k] = mj_nextActivation(m, d, i, act_adr+k, d->act_dot[act_adr+k]);
            }
          } else {
            mju_copy3(u, d->act + act_adr);
          }
        }
        expmap2Quat(q_tgt, u);
      }

      // error rotation from current to target: e = log(q_cur^-1 * q_tgt), in the local frame
      // of the transmission, matching the frame of the moment rows and of actuator_velocity
      // note: the force is invariant to the setpoint representative (exp is ray-periodic),
      // so no wrapping is required; act is re-anchored at integration time in mj_advance
      mjtNum q_cur[4], e[3];
      expmap2Quat(q_cur, d->actuator_length + oadr);
      mju_subQuat(e, q_tgt, q_cur);

      // output force: kp * error + constant - kv * velocity
      mjtNum kp = m->actuator_gainprm[mjNGAIN*i];
      const mjtNum* prm = m->actuator_biasprm + mjNBIAS*i;
      for (int k=0; k < 3; k++) {
        force[oadr+k] = kp*e[k] + prm[0] + prm[2]*d->actuator_velocity[oadr+k];
      }
      continue;
    }

    // check for tendon transmission with force limits
    if (ntendon && !tendon_frclimited && m->actuator_trntype[i] == mjTRN_TENDON) {
      tendon_frclimited = m->tendon_actfrclimited[m->actuator_trnid[2*i]];
    }

    // extract info
    const mjtNum* dynprm = m->actuator_dynprm + mjNDYN*i;
    const mjtNum* gainprm = m->actuator_gainprm + mjNGAIN*i;
    mjtGain gaintype = m->actuator_gaintype[i];
    int actnum = m->actuator_actnum[i];

    // handle SISO actuators according to gain type
    switch (gaintype) {
    case mjGAIN_FIXED:              // fixed gain: prm = gain
      gain = gainprm[0];
      break;

    case mjGAIN_AFFINE:             // affine: prm = [const, kp, kv]
      gain = gainprm[0] + gainprm[1]*d->actuator_length[oadr] +
             gainprm[2]*d->actuator_velocity[oadr];
      break;

    case mjGAIN_MUSCLE:             // muscle gain
      gain = mju_muscleGain(d->actuator_length[oadr],
                            d->actuator_velocity[oadr],
                            m->actuator_lengthrange+2*oadr,
                            m->actuator_acc0[oadr],
                            gainprm);
      break;

    case mjGAIN_DCMOTOR: {          // DC motor: gain = K or K/R
      mjtNum R = gainprm[0];  // resistance
      mjtNum K = gainprm[1];  // motor constant
      mjDCMotorSlots slots = mj_dcmotorSlots(dynprm, gainprm);

      // verify allocated state size matches parameters; SHOULD NOT OCCUR
      if (slots.num_slots != actnum) {
        mjERROR("inconsistent state array dimension in DC motor (actuator %d)", i);
      }

      int adr = m->actuator_actadr[i];

      // adjust R for temperature if enabled
      if (slots.temperature >= 0) {
        mjtNum T = d->act[adr + slots.temperature];
        mjtNum alpha = gainprm[2];  // temperature coefficient
        mjtNum T0 = gainprm[3];     // reference temperature
        mjtNum Ta = dynprm[4];      // ambient temperature
        R *= 1 + alpha * (T + Ta - T0);
      }

      // stateful current: gain = K, force = K * act[last] (generic path)
      // stateless: gain = K/R, force = K/R * ctrl (condition below)
      gain = (dynprm[0] > 0) ? K : K / mju_max(mjMINVAL, R);

      // controller: compute voltage, override ctrl[uadr] for force computation
      if ((int)gainprm[8] > 0) {
        mjtNum x_I = (slots.integral >= 0) ? d->act[adr + slots.integral] : 0;
        ctrl[uadr] = dcmotorVoltage(ctrl[uadr], d->actuator_length[oadr],
                                    d->actuator_velocity[oadr], x_I, gainprm);
      }
      break;
    }

    case mjGAIN_SO3:                // handled above via early continue
      mjERROR("mjGAIN_SO3 reached SISO switch (actuator %d)", i);
      break;

    default:                        // user gain
      if (mjcb_act_gain) {
        gain = mjcb_act_gain(m, d, i);
      } else {
        gain = 1;
      }
    }

    // set force = gain .* [ctrl/act]

    // DC motor without current state: use ctrl even if other activations exist
    int dcmotor_no_current = (gaintype == mjGAIN_DCMOTOR && dynprm[0] <= 0);
    if (actnum == 0 || dcmotor_no_current) {
      mjtNum input = ctrl[uadr];

      // rotational setpoint: use representative nearest the length (local, no state change)
      mjtNum period = wrapPeriod(m, i);
      if (period > 0) {
        input = wrapSetpoint(input, d->actuator_length[oadr], period);
      }
      force[oadr] = gain * input;
    } else {
      // use last activation variable associated with actuator i
      int act_adr = m->actuator_actadr[i] + actnum - 1;

      mjtNum act;
      if (m->actuator_actearly[i]) {
        act = mj_nextActivation(m, d, i, act_adr, d->act_dot[act_adr]);
      } else {
        act = d->act[act_adr];
      }

      // rotational setpoint: use representative nearest the length (local, no state change)
      mjtNum period = wrapPeriod(m, i);
      if (period > 0) {
        act = wrapSetpoint(act, d->actuator_length[oadr], period);
      }
      force[oadr] = gain * act;
    }

    // extract bias info
    const mjtNum* biasprm = m->actuator_biasprm + mjNBIAS*i;
    mjtBias biastype = m->actuator_biastype[i];

    // handle according to bias type
    switch (biastype) {
    case mjBIAS_NONE:               // none
      bias = 0.0;
      break;

    case mjBIAS_AFFINE:             // affine: biasprm = [const, kp, kv]
      bias = biasprm[0] + biasprm[1]*d->actuator_length[oadr] +
             biasprm[2]*d->actuator_velocity[oadr];
      break;

    case mjBIAS_MUSCLE:             // muscle passive force
      bias =  mju_muscleBias(d->actuator_length[oadr],
                             m->actuator_lengthrange+2*oadr,
                             m->actuator_acc0[oadr],
                             biasprm);
      break;

    case mjBIAS_DCMOTOR: {          // DC motor: back-EMF only (current-limited)
      bias = 0;

      // back-EMF (stateless only; for stateful current it's in the ODE)
      mjtNum te = m->actuator_dynprm[mjNDYN*i];  // electrical time constant
      if (te <= 0) {
        mjtNum K = gainprm[1];   // motor constant
        bias -= gain * K * d->actuator_velocity[oadr];
      }
      break;
    }

    default:                        // user bias
      if (mjcb_act_bias) {
        bias = mjcb_act_bias(m, d, i);
      } else {
        bias = 0;
      }
    }

    // add bias
    force[oadr] += bias;
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

  // clamp tendon total actuator force
  if (tendon_frclimited) {
    // compute total force for each tendon
    mjtNum* tendon_total_force = mjSTACKALLOC(d, ntendon, mjtNum);
    mju_zero(tendon_total_force, ntendon);
    for (int i=0; i < nactuator; i++) {
      if (m->actuator_trntype[i] == mjTRN_TENDON) {
        int tendon_id = m->actuator_trnid[2*i];
        if (m->tendon_actfrclimited[tendon_id]) {
          tendon_total_force[tendon_id] += force[m->actuator_outadr[i]];
        }
      }
    }

    // scale tendon actuator forces if limited and outside range
    for (int i=0; i < nactuator; i++) {
      if (m->actuator_trntype[i] != mjTRN_TENDON) {
        continue;
      }
      int tendon_id = m->actuator_trnid[2*i];
      mjtNum tendon_force = tendon_total_force[tendon_id];
      if (m->tendon_actfrclimited[tendon_id] && tendon_force) {
        const mjtNum* range = m->tendon_actfrcrange + 2 * tendon_id;
        if (tendon_force < range[0]) {
          force[m->actuator_outadr[i]] *= range[0] / tendon_force;
        } else if (tendon_force > range[1]) {
          force[m->actuator_outadr[i]] *= range[1] / tendon_force;
        }
      }
    }
  }

  // clamp actuator_force
  for (int i=0; i < nactuator; i++) {
    if (!m->actuator_forcelimited[i]) {
      continue;
    }
    const mjtNum* range = m->actuator_forcerange + 2*i;
    mjtNum* f = force + m->actuator_outadr[i];

    // SO3: clamp the norm of the output torque, preserving its direction
    if (m->actuator_gaintype[i] == mjGAIN_SO3) {
      mjtNum norm = mju_norm3(f);
      if (norm > range[1]) {
        mju_scl3(f, f, range[1]/norm);
      }
    }

    // otherwise: clamp each output
    else {
      int outnum = m->actuator_outnum[i];
      for (int j=0; j < outnum; j++) {
        f[j] = mju_clip(f[j], range[0], range[1]);
      }
    }
  }

  // add DC motor mechanical forces (not subject to current limits)
  for (int i=0; i < nactuator; i++) {
    if (m->actuator_biastype[i] != mjBIAS_DCMOTOR) {
      continue;
    }
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_ACTUATOR, i) == mjS_ASLEEP) {
      continue;
    }
    if (mj_actuatorDisabled(m, i) || m->actuator_plugin[i] >= 0) {
      continue;
    }

    const mjtNum* biasprm = m->actuator_biasprm + mjNBIAS*i;
    const mjtNum* dynprm = m->actuator_dynprm + mjNDYN*i;
    int oadr = m->actuator_outadr[i];

    // cogging torque
    mjtNum A = biasprm[0];
    if (A != 0) {
      mjtNum Np = biasprm[1];
      mjtNum phi = biasprm[2];
      force[oadr] += A * mju_sin(Np*d->actuator_length[oadr] + phi);
    }

    // LuGre friction
    mjtNum sigma0 = dynprm[5];
    if (sigma0 > 0) {
      mjtNum sigma1 = dynprm[6];
      mjDCMotorSlots slots = mj_dcmotorSlots(dynprm, m->actuator_gainprm + mjNGAIN*i);
      int adr = m->actuator_actadr[i] + slots.bristle;
      mjtNum z = d->act[adr];
      mjtNum z_dot = d->act_dot[adr];
      force[oadr] -= sigma0 * z + sigma1 * z_dot;
    }
  }

  // qfrc_actuator = moment' * force
  mju_mulMatTVecSparse(d->qfrc_actuator, d->actuator_moment, force, nout, nv,
                       d->moment_rownnz, d->moment_rowadr, d->moment_colind);

  // actuator-level gravity compensation
  if (m->flg_gravcomp && !mjDISABLED(mjDSBL_GRAVITY) && mju_norm3(m->opt.gravity)) {
    // number of dofs for each joint type: {mjJNT_FREE, mjJNT_BALL, mjJNT_SLIDE, mjJNT_HINGE}
    static const int jnt_dofnum[4] = {6, 3, 1, 1};
    int njnt = m->njnt;
    for (int i=0; i < njnt; i++) {
      // skip if gravcomp added as passive force
      if (!m->jnt_actgravcomp[i]) {
        continue;
      }

      // add gravcomp force
      int dofnum = jnt_dofnum[m->jnt_type[i]];
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
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < m->nv;
  int nv;
  const int* index;

  // qfrc_smooth = qfrc_passive - qfrc_bias + qfrc_applied + qfrc_actuator
  if (!sleep_filter) {
    nv = m->nv;
    index = NULL;
    mju_sub(d->qfrc_smooth, d->qfrc_passive, d->qfrc_bias, nv);
    mju_addTo(d->qfrc_smooth, d->qfrc_applied, nv);
    mju_addTo(d->qfrc_smooth, d->qfrc_actuator, nv);
  } else {
    nv = d->nv_awake;
    index = d->dof_awake_ind;
    mju_subInd(d->qfrc_smooth, d->qfrc_passive, d->qfrc_bias, index, nv);
    mju_addToInd(d->qfrc_smooth, d->qfrc_applied, index, nv);
    mju_addToInd(d->qfrc_smooth, d->qfrc_actuator, index, nv);
  }

  // qfrc_smooth += project(xfrc_applied)
  mj_xfrcAccumulate(m, d, d->qfrc_smooth);

  // implicit effective metric (built in mj_fwdPosition): the smooth acceleration is that of
  // the linearly-implicit dynamics, (M + K)*qacc_smooth = qfrc_smooth + c, so the constraint
  // solver, the no-constraint shortcut and the warmstart all see one consistent metric.
  if (d->efm_active) {
    mj_markStack(d);
    mjtNum* qfrc_eff = mjSTACKALLOC(d, nv, mjtNum);
    mju_add(qfrc_eff, d->qfrc_smooth, d->efm_c, nv);
    mjd_effSolve(m, d, d->qacc_smooth, qfrc_eff);
    mj_freeStack(d);
    return;
  }

  // copy for in-place solve: qacc_smooth = qfrc_smooth
  if (!sleep_filter) {
    mju_copy(d->qacc_smooth, d->qfrc_smooth, nv);
  } else {
    mju_copyInd(d->qacc_smooth, d->qfrc_smooth, index, nv);
  }

  // qacc_smooth = M \ qfrc_smooth
  mj_solveLD(d->qacc_smooth, d->qLD, d->qLDiagInv, nv, 1,
             m->M_rownnz, m->M_rowadr, m->M_colind, index);
}


// warmstart/init solver
static void warmstart(const mjModel* m, mjData* d) {
  int nv = m->nv, nefc = d->nefc;

  // warmstart with best of (qacc_warmstart, qacc_smooth)
  if (!mjDISABLED(mjDSBL_WARMSTART)) {
    mj_markStack(d);
    mjtNum* jar = mjSTACKALLOC(d, nefc, mjtNum);

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
      mjtNum* ARf = mjSTACKALLOC(d, nefc, mjtNum);
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
      mjtNum* Ma = mjSTACKALLOC(d, nv, mjtNum);
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

    // have island structure: unconstrained qacc = qacc_smooth
    if (d->nisland > 0) {
      // loop over unconstrained dofs in map_idof2dof[nidof, nv)
      for (int i=d->nidof; i < nv; i++) {
        int dof = d->map_idof2dof[i];
        d->qacc[dof] = d->qacc_smooth[dof];
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


// mju_dispatch callback: solve one island
static void solveIslandTask(const mjModel* m, mjData* d, void* arg, int thread_id, int island) {
  if (m->opt.solver == mjSOL_NEWTON) {
    mj_solNewton_island(m, d, island, m->opt.iterations);
  } else if (m->opt.solver == mjSOL_CG) {
    mj_solCG_island(m, d, island, m->opt.iterations);
  } else {
    mj_solPGS_island(m, d, island, m->opt.iterations);
  }
}


// compute efc_b, efc_force, qfrc_constraint; update qacc
void mj_fwdConstraint(const mjModel* m, mjData* d) {
  TM_START;
  int nv = m->nv, nefc = d->nefc, nisland = d->nisland, nidof;

  // always clear qfrc_constraint
  mju_zero(d->qfrc_constraint, nv);

  // no constraints: copy unconstrained acc, clear forces, return
  // (with the effective metric active, qacc_smooth is already the implicit answer)
  if (!nefc) {
    mju_copy(d->qacc, d->qacc_smooth, nv);
    mju_zeroInt(d->solver_niter, mjNISLAND);
    TM_END(mjTIMER_CONSTRAINT);
    return;
  }

  // compute efc_b = J*qacc_smooth - aref
  mj_mulJacVec(m, d, d->efc_b, d->qacc_smooth);
  mju_subFrom(d->efc_b, d->efc_aref, nefc);

  // check for invalid solver type
  if (m->opt.solver != mjSOL_PGS && m->opt.solver != mjSOL_CG && m->opt.solver != mjSOL_NEWTON) {
    mjERROR("unknown solver type %d", m->opt.solver);
  }

  // warmstart solver
  warmstart(m, d);
  mju_zeroInt(d->solver_niter, mjNISLAND);

  // check if islands are supported
  // TODO: support islands with the implicit effective metric and remove the mj_flexCG
  // condition. It is here because the metric machinery is monolithic: the efm_c shift and
  // the Ma/Mv/Mgrad operators (mjd_effMulAdd, mjd_effSolve) act on global dof vectors with
  // no island-local form. Discovery is already handled: findEdges unions the trees of every
  // stiffness-active flex, so a flex always lands in one island together with everything it
  // touches. Removal therefore needs only the solver side: apply the efm_c shift to that
  // island's dofs, gather/scatter its island-local vectors around the covered-compact
  // factor solves (the factors themselves need no change), and enable the metric path
  // (flg_flex) for the flex-containing island alone.
  int islands_supported = !mjDISABLED(mjDSBL_ISLAND) && nisland > 0 && !mj_flexCG(m);

  // run solver over constraint islands
  if (islands_supported) {
    switch ((mjtSolver) m->opt.solver) {
    case mjSOL_PGS:
      mju_dispatch(m, d, solveIslandTask, NULL, nisland);
      break;

    case mjSOL_CG:
    case mjSOL_NEWTON:
      // copy inputs to islands (vel+acc deps, pos-dependent already copied in mj_island)
      nidof = d->nidof;
      mju_gather(d->ifrc_smooth,     d->qfrc_smooth,     d->map_idof2dof, nidof);
      mju_gather(d->ifrc_constraint, d->qfrc_constraint, d->map_idof2dof, nidof);
      mju_gather(d->iacc_smooth,     d->qacc_smooth,     d->map_idof2dof, nidof);
      mju_gather(d->iacc,            d->qacc,            d->map_idof2dof, nidof);
      mju_gather(d->iefc_force,      d->efc_force,       d->map_iefc2efc, nefc);
      mju_gather(d->iefc_aref,       d->efc_aref,        d->map_iefc2efc, nefc);

      mju_dispatch(m, d, solveIslandTask, NULL, nisland);

      // copy back solver outputs (scatter dofs since ni <= nv)
      mju_scatter(d->qacc,            d->iacc,            d->map_idof2dof, nidof);
      mju_scatter(d->qfrc_constraint, d->ifrc_constraint, d->map_idof2dof, nidof);
      mju_gather(d->efc_force, d->iefc_force, d->map_efc2iefc, nefc);
      break;
    }

    // run noslip solver per island if enabled
    if (m->opt.noslip_iterations > 0) {
      for (int island=0; island < nisland; island++) {
        mj_solNoSlip_island(m, d, island, m->opt.noslip_iterations);
      }
    }
  }

  // run solver over all constraints (monolithic)
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
    }

    // run noslip solver if enabled
    if (m->opt.noslip_iterations > 0) {
      mj_solNoSlip(m, d, m->opt.noslip_iterations);
    }
  }

  // dual solvers: map efc_force to joint space (always monolithic)
  if (m->opt.solver == mjSOL_PGS || m->opt.noslip_iterations > 0) {
    mj_dualFinish(m, d);
  }

  TM_END(mjTIMER_CONSTRAINT);
}


//-------------------------- state advancement and integration  ------------------------------------

// advance state and time
//   act_dot: activation derivatives
//   qacc:    acceleration used to update d->qvel (d->qvel += h*qacc)
//   qvel:    optional velocity used for position integration; if NULL, use d->qvel
static void mj_advance(const mjModel* m, mjData* d,
                       const mjtNum* act_dot, const mjtNum* qacc, const mjtNum* qvel) {
  int nactuator = m->nactuator, nsensor = m->nsensor;

  // advance history buffers
  if (m->nhistory > 0) {
    // advance ctrl history buffers
    for (int i = 0; i < nactuator; i++) {
      int nsample = m->actuator_history[2*i];
      if (nsample == 0) continue;

      // get history buffer pointer and insert ctrl at current time
      mjtNum* buf = d->history + m->actuator_historyadr[i];
      *mju_historyInsert(buf, nsample, /*dim=*/1, d->time) = d->ctrl[m->actuator_ctrladr[i]];
    }

    // advance sensor history buffers
    for (int i = 0; i < nsensor; i++) {
      int nsample = m->sensor_history[2*i];
      if (nsample == 0) continue;

      // get history buffer parameters
      int dim = m->sensor_dim[i];
      mjtNum* buf = d->history + m->sensor_historyadr[i];
      mjtNum delay = m->sensor_delay[i];
      mjtNum interval = m->sensor_interval[2*i];

      if (interval > 0) {
        // interval mode: if condition is satisfied, compute; otherwise copy
        mjtNum time_prev = buf[0];  // first slot stores previous sensor tick
        if (time_prev + interval <= d->time) {
          buf[0] += interval;  // advance by exact interval (continuous time)
          mjtNum* slot = mju_historyInsert(buf, nsample, dim, d->time);
          if (delay > 0) {
            // have delay, compute sensor
            mj_computeSensor(m, d, i, slot);
          } else {
            // no delay, copy from sensordata (already computed)
            mju_copy(slot, d->sensordata + m->sensor_adr[i], dim);
          }
        }
      } else if (delay > 0) {
        // delay-only mode: always compute and insert
        mjtNum* slot = mju_historyInsert(buf, nsample, dim, d->time);
        mj_computeSensor(m, d, i, slot);
      } else {
        // history-only mode: copy from sensordata (already computed)
        mjtNum* slot = mju_historyInsert(buf, nsample, dim, d->time);
        mju_copy(slot, d->sensordata + m->sensor_adr[i], dim);
      }
    }
  }

  // advance activations
  if (m->na && !mjDISABLED(mjDSBL_ACTUATION)) {
    for (int i=0; i < nactuator; i++) {
      int actadr = m->actuator_actadr[i];
      int actadr_end = actadr + m->actuator_actnum[i];
      for (int j=actadr; j < actadr_end; j++) {
        // if disabled, set act_dot to 0
        d->act[j] = mj_nextActivation(m, d, i, j, mj_actuatorDisabled(m, i) ? 0 : act_dot[j]);
      }
    }

    // rotational setpoints stored in act: replace with an equivalent bounded representative,
    // like the actrange clamp above, this is a projection applied at integration time
    for (int i=0; i < nactuator; i++) {
      if (m->actuator_dyntype[i] != mjDYN_INTEGRATOR) {
        continue;
      }

      // per-axis servo: wrap act to the representative nearest the length
      mjtNum period = wrapPeriod(m, i);
      if (period > 0) {
        int adr = m->actuator_actadr[i] + m->actuator_actnum[i] - 1;
        d->act[adr] = wrapSetpoint(d->act[adr], d->actuator_length[m->actuator_outadr[i]], period);
      }

      // SO3 servo: re-anchor the act setpoint to the canonical representative
      else if (m->actuator_gaintype[i] == mjGAIN_SO3) {
        int adr = m->actuator_actadr[i];
        mjtNum angle = mju_norm3(d->act + adr);
        if (angle > mjPI) {
          mjtNum scale = (angle - 2*mjPI*mju_round(angle/(2*mjPI))) / angle;
          for (int k=0; k < 3; k++) {
            d->act[adr+k] *= scale;
          }
        }
      }
    }
  }

  // put islands to sleep according to velocity tolerance
  if (mj_sleep(m, d)) {
    // if any trees put to sleep (qvel set to 0), recompute all velocity-dependent quantities
    mj_forwardSkip(m, d, mjSTAGE_POS, 0);

    // update sleep indices
    mj_updateSleep(m, d);
  }

  // advance velocities
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
  if (sleep_filter) {
    mju_addToSclInd(d->qvel, qacc, d->dof_awake_ind, m->opt.timestep, d->nv_awake);
  } else {
    mju_addToScl(d->qvel, qacc, m->opt.timestep, m->nv);
  }

  // advance positions with qvel if given, d->qvel otherwise (semi-implicit)
  const int* index = sleep_filter ? d->body_awake_ind : NULL;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  mj_integratePosInd(m, d->qpos, qvel ? qvel : d->qvel, m->opt.timestep, index, nbody);

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

  // save qacc for next step warmstart
  mju_copy(d->qacc_warmstart, d->qacc, m->nv);
}

// Euler integrator, semi-implicit in velocity, possibly skipping factorisation
void mj_EulerSkip(const mjModel* m, mjData* d, int skipfactor) {
  TM_START;
  mj_markStack(d);
  mjtNum* qfrc = mjSTACKALLOC(d, m->nv, mjtNum);
  mjtNum* qacc = mjSTACKALLOC(d, m->nv, mjtNum);

  // sleep filtering
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;
  const int* dof_awake_ind = sleep_filter ? d->dof_awake_ind : NULL;

  // check for dof damping if disable flag is not set
  int dof_damping = 0;
  if (!mjDISABLED(mjDSBL_EULERDAMP) && !mjDISABLED(mjDSBL_DAMPER)) {
    for (int v=0; v < nv; v++) {
      int i = sleep_filter ? dof_awake_ind[v] : v;
      if (m->dof_damping[i] > 0 ||
          !mju_isZero(m->dof_dampingpoly + mjNPOLY*i, mjNPOLY) ||
          m->jnt_actuatorid[m->dof_jntid[i]] != -1) {
        dof_damping = 1;
        break;
      }
    }
  }

  // no damping or disabled: explicit velocity integration
  if (!dof_damping) {
    if (sleep_filter) {
      mju_copyInd(qacc, d->qacc, dof_awake_ind, nv);
    } else {
      mju_copy(qacc, d->qacc, nv);
    }
  }

  // damping: integrate implicitly
  else {
    if (!skipfactor) {
      // qH = M
      if (sleep_filter) {
        mju_copySparse(d->qH, d->M, m->M_rownnz, m->M_rowadr, dof_awake_ind, d->nv_awake);
      } else {
        mju_copy(d->qH, d->M, m->nC);
      }

      // qH += h*diag(B)
      for (int v=0; v < nv; v++) {
        int i = sleep_filter ? dof_awake_ind[v] : v;
        mjtNum qv = d->qvel[i];
        mjtNum poly[mjNPOLY];
        mju_copy(poly, m->dof_dampingpoly + mjNPOLY*i, mjNPOLY);
        mjtNum damping = m->dof_damping[i]
                         + mj_actuatorDamping(m, mjOBJ_JOINT, m->dof_jntid[i], poly);
        mjtNum damp_deriv = mjd_xPolyForce(damping, poly, qv, mjNPOLY, 1);
        d->qH[m->M_rowadr[i] + m->M_rownnz[i] - 1] += m->opt.timestep * damp_deriv;
      }

      // factorize in-place
      mj_factorI(d->qH, d->qHDiagInv, nv, m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
    }

    // solve
    if (sleep_filter) {
      mju_addInd(qfrc, d->qfrc_smooth, d->qfrc_constraint, dof_awake_ind, nv);
      mju_copyInd(qacc, qfrc, dof_awake_ind, nv);
    } else {
      mju_add(qfrc, d->qfrc_smooth, d->qfrc_constraint, nv);
      mju_copy(qacc, qfrc, nv);
    }
    mj_solveLD(qacc, d->qH, d->qHDiagInv, nv, 1,
               m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
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
  dX = mjSTACKALLOC(d, 2*nv+na, mjtNum);
  for (int i=0; i < N; i++) {
    X[i] = mjSTACKALLOC(d, nq+nv+na, mjtNum);
    F[i] = mjSTACKALLOC(d, nv+na, mjtNum);
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


// return 1 if any flex needs implicit stiffness treatment (interp or bending)
static mjtBool flex_has_implicit_stiffness(const mjModel* m) {
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_rigid[f]) {
      continue;
    }

    // interpolated flex with stiffness
    if (m->flex_interp[f] &&
        m->flex_edgeequality[f] != 3 &&
        m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) {
      return 1;
    }

    // standard flex with bending
    if (!m->flex_interp[f] && m->flex_dim[f] == 2 &&
        m->flex_bendingadr[f] >= 0) {
      return 1;
    }

    // standard flex with stretch
    if (!m->flex_interp[f] && m->flex_dim[f] >= 2 &&
        m->flex_stiffnessadr[f] >= 0 &&
        m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) {
      return 1;
    }
  }
  return 0;
}


// implicit-flex solve gate: with the CG solver, an implicit integrator and flex stiffness
// present, the CG solve carries the implicit flex stiffness itself -- K = (h^2+h*d) times the flex stiffness enters
// the objective/gradient/linesearch, and the preconditioned gradient becomes (M+K)\grad by
// linear matrix-free CG against the existing M factor (the in-solver form of the old post-hoc
// flexInterp_cgsolve treatment, no factorization anywhere); mj_implicitSkip then folds the
// implicit flex force of the solver's qacc into qfrc. When active with islands
// enabled, mj_fwdConstraint forces a monolithic solve (flex mesh coupling is invisible to
// constraint islanding). solver="Newton" keeps its exact-factorization semantics untouched.
// Models outside the gate integrate flex elasticity explicitly.
int mj_flexCG(const mjModel* m) {
  return m->opt.solver == mjSOL_CG &&
         (m->opt.integrator == mjINT_IMPLICIT || m->opt.integrator == mjINT_IMPLICITFAST) &&
         m->opt.cone != mjCONE_ELLIPTIC && !mjENABLED(mjENBL_SLEEP) &&
         flex_has_implicit_stiffness(m);
}


// fully implicit in velocity, possibly skipping factorization
void mj_implicitSkip(const mjModel* m, mjData* d, int skipfactor) {
  TM_START;
  int nD = m->nD, nC = m->nC, njnt = m->njnt;

  mj_markStack(d);
  mjtNum* qfrc = mjSTACKALLOC(d, m->nv, mjtNum);
  mjtNum* qacc = mjSTACKALLOC(d, m->nv, mjtNum);

  // sleep filtering
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < m->nv;
  int nv = sleep_filter ? d->nv_awake : m->nv;
  const int* dof_awake_ind = sleep_filter ? d->dof_awake_ind : NULL;

  // set qfrc = qfrc_smooth + qfrc_constraint
  if (sleep_filter) {
    mju_addInd(qfrc, d->qfrc_smooth, d->qfrc_constraint, dof_awake_ind, nv);
  } else {
    mju_add(qfrc, d->qfrc_smooth, d->qfrc_constraint, nv);
  }

  // implicit flex stiffness is carried by the constraint solver (see mj_flexCG): use the
  // solver's qacc directly. The qDeriv treatment is skipped for these models -- flex damping
  // is already implicit inside the solve (the s2 terms of B), joint damping and other velocity
  // derivatives integrate explicitly. This avoids both the qDeriv machinery and the
  // sequential flex-vs-qDeriv splitting. Models outside the gate (non-Newton solver, elliptic
  // cones, islands, sleep) integrate flex elasticity explicitly.
  int flexcg = !sleep_filter && mj_flexCG(m);

  // factorization
  if (!skipfactor && !flexcg) {
    // implicit
    if (m->opt.integrator == mjINT_IMPLICIT) {
      // compute analytical derivative qDeriv
      mjd_smooth_vel(m, d, /* flg_bias = */ 1);

      // gather qLU <- M (lower to full)
      mju_gatherMasked(d->qLU, d->M, m->mapM2D, nD);

      // set qLU = M - dt*qDeriv
      mju_addToScl(d->qLU, d->qDeriv, -m->opt.timestep, nD);
    }

    // implicitfast
    else if (m->opt.integrator == mjINT_IMPLICITFAST) {
      // compute analytical derivative qDeriv; skip rne derivative
      mjd_smooth_vel(m, d, /* flg_bias = */ 0);

      // modified mass matrix: gather qH <- qDeriv (full to lower)
      mju_gather(d->qH, d->qDeriv, m->mapD2M, nC);

      // set qH = M - dt*qDeriv
      mju_addScl(d->qH, d->M, d->qH, -m->opt.timestep, nC);

      // standalone free bodies: reset qH block rows to M; their qDeriv rows may be asymmetric and
      // are handled by the local LU solve; we reset to keep LTL well-defined
      for (int j=0; j < njnt; j++) {
        if (m->jnt_type[j] != mjJNT_FREE || !mj_isFreeBody(m, m->jnt_bodyid[j])) {
          continue;
        }
        int adr = m->jnt_dofadr[j];
        for (int r=0; r < 6; r++) {
          mju_copy(d->qH + m->M_rowadr[adr+r], d->M + m->M_rowadr[adr+r], m->M_rownnz[adr+r]);
        }
      }
    } else {
      mjERROR("integrator must be implicit or implicitfast");
    }

    // standard factorization (implicit / implicitfast)
    if (m->opt.integrator == mjINT_IMPLICIT) {
      int* scratch = mjSTACKALLOC(d, nv, int);
      mju_factorLUSparse(d->qLU, nv, scratch, m->D_rownnz, m->D_rowadr, m->D_colind, dof_awake_ind);
    } else {
      mj_factorI(d->qH, d->qHDiagInv, nv, m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
    }
  }

  // standard sparse solve
  if (flexcg) {
    // constraint solver's qacc already carries the implicit flex force
    mju_copy(qacc, d->qacc, m->nv);
  } else if (m->opt.integrator == mjINT_IMPLICIT) {
    mju_solveLUSparse(qacc, d->qLU, qfrc, nv, m->D_rownnz, m->D_rowadr, m->D_diag, m->D_colind,
                      dof_awake_ind);
  } else {
    // implicitfast
    if (sleep_filter) {
      mju_copyInd(qacc, qfrc, dof_awake_ind, nv);
    } else {
      mju_copy(qacc, qfrc, nv);
    }
    mj_solveLD(qacc, d->qH, d->qHDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, dof_awake_ind);
  }

  // implicitfast: local unsymmetric solve for standalone free bodies
  // adds the bias (gyroscopic) derivative, dropped from the global symmetric solve; the
  // 6x6 block of M - h*D is decoupled from the rest of the system (D sparsity is tree-local),
  // so overwriting these rows of qacc leaves all other DOFs unaffected
  if (m->opt.integrator == mjINT_IMPLICITFAST && !flexcg) {
    for (int j=0; j < m->njnt; j++) {
      mjtNum A[36];
      if (!mjd_freeMhat(m, d, j, m->opt.timestep, A)) {
        continue;
      }

      // solve A * qacc_block = qfrc_block
      int adr = m->jnt_dofadr[j];
      int pivot[6];
      if (mju_factorLU6(A, pivot)) {
        mjtNum x[6];  // local vector for guaranteed memory alignment
        mju_solveLU6(x, A, qfrc+adr, pivot);
        mji_copy6(qacc+adr, x);
      }
    }
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

    if (!d->flg_energypos) {
      if (mjENABLED(mjENBL_ENERGY)) {
        mj_energyPos(m, d);
      } else {
        d->energy[0] = d->energy[1] = 0;
      }
    }
  }

  // velocity-dependent
  if (skipstage < mjSTAGE_VEL) {
    mj_fwdVelocity(m, d);

    if (!skipsensor) {
      mj_sensorVel(m, d);
    }

    if (mjENABLED(mjENBL_ENERGY) && !d->flg_energyvel) {
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
    d->flg_rnepost = 0;  // clear flag for lazy evaluation
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

  if (!d->flg_energypos) {
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyPos(m, d);
    } else {
      d->energy[0] = d->energy[1] = 0;
    }
  }

  mj_fwdVelocity(m, d);
  mj_sensorVel(m, d);
  if (mjENABLED(mjENBL_ENERGY) && !d->flg_energyvel) {
    mj_energyVel(m, d);
  }

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
  d->flg_rnepost = 0;  // clear flag for lazy evaluation
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
