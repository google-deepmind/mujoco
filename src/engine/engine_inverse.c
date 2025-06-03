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

#include "engine/engine_inverse.h"

#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_derivative.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_forward.h"
#include "engine/engine_sensor.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"

// position-dependent computations
void mj_invPosition(const mjModel* m, mjData* d) {
  TM_START1;
  TM_START;

  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_camlight(m, d);
  mj_flex(m, d);
  mj_tendon(m, d);
  TM_END(mjTIMER_POS_KINEMATICS);

  mj_makeM(m, d);           // timed internally (POS_INERTIA)
  mj_factorM(m, d);         // timed internally (POS_INERTIA)

  mj_collision(m, d);  // timed internally (POS_COLLISION)

  TM_RESTART;
  mj_makeConstraint(m, d);
  TM_END(mjTIMER_POS_MAKE);

  TM_RESTART;
  mj_transmission(m, d);
  TM_ADD(mjTIMER_POS_KINEMATICS);

  TM_END1(mjTIMER_POSITION);
}



// velocity-dependent computations
void mj_invVelocity(const mjModel* m, mjData* d) {
  mj_fwdVelocity(m, d);
}



// convert discrete-time qacc to continuous-time qacc
static void mj_discreteAcc(const mjModel* m, mjData* d) {
  int nv = m->nv, nM = m->nM, nD = m->nD, dof_damping;
  mjtNum *qacc = d->qacc;

  mj_markStack(d);
  mjtNum* qfrc = mjSTACKALLOC(d, nv, mjtNum);

  // use selected integrator
  switch ((mjtIntegrator) m->opt.integrator) {
  case mjINT_RK4:
    // not supported by RK4
    mjERROR("discrete inverse dynamics is not supported by RK4 integrator");
    return;

  case mjINT_EULER:
    // check for dof damping if disable flag is not set
    dof_damping = 0;
    if (!mjDISABLED(mjDSBL_EULERDAMP)) {
      for (int i=0; i < nv; i++) {
        if (m->dof_damping[i] > 0) {
          dof_damping = 1;
          break;
        }
      }
    }

    // if disabled or no dof damping, nothing to do
    if (!dof_damping) {
      mj_freeStack(d);
      return;
    }

    // set qfrc = (M + h*diag(B)) * qacc
    mj_mulM(m, d, qfrc, qacc);
    for (int i=0; i < nv; i++) {
      qfrc[i] += m->opt.timestep * m->dof_damping[i] * d->qacc[i];
    }
    break;

  case mjINT_IMPLICIT:
    // compute qDeriv
    mjd_smooth_vel(m, d, /* flg_bias = */ 1);

    // gather qLU <- qM (lower to full)
    mju_gather(d->qLU, d->qM, d->mapM2D, nD);

    // set qLU = qM - dt*qDeriv
    mju_addToScl(d->qLU, d->qDeriv, -m->opt.timestep, m->nD);

    // set qfrc = qLU * qacc
    mju_mulMatVecSparse(qfrc, d->qLU, qacc, nv,
                        d->D_rownnz, d->D_rowadr, d->D_colind, /*rowsuper=*/NULL);
    break;

  case mjINT_IMPLICITFAST:
    // compute analytical derivative qDeriv; skip rne derivative
    mjd_smooth_vel(m, d, /* flg_bias = */ 0);

    // save mass matrix
    mjtNum* qMsave = mjSTACKALLOC(d, m->nM, mjtNum);
    mju_copy(qMsave, d->qM, m->nM);

    // set M = M - dt*qDeriv (reduced to M nonzeros)
    mjtNum* qDerivReduced = mjSTACKALLOC(d, m->nM, mjtNum);
    for (int i=0; i < nM; i++) {
      qDerivReduced[i] = d->qDeriv[d->mapD2M[i]];
    }
    mju_addToScl(d->qM, qDerivReduced, -m->opt.timestep, m->nM);

    // set qfrc = (M - dt*qDeriv) * qacc
    mj_mulM(m, d, qfrc, qacc);

    // restore mass matrix
    mju_copy(d->qM, qMsave, m->nM);
    break;
  }

  // solve for qacc: qfrc = M * qacc
  mj_solveM(m, d, qacc, qfrc, 1);

  mj_freeStack(d);
}



// inverse constraint solver
void mj_invConstraint(const mjModel* m, mjData* d) {
  TM_START;
  int nefc = d->nefc;

  // no constraints: clear, return
  if (!nefc) {
    mju_zero(d->qfrc_constraint, m->nv);
    TM_END(mjTIMER_CONSTRAINT);
    return;
  }

  mj_markStack(d);
  mjtNum* jar = mjSTACKALLOC(d, nefc, mjtNum);

  // compute jar = Jac*qacc - aref
  mj_mulJacVec(m, d, jar, d->qacc);
  mju_subFrom(jar, d->efc_aref, nefc);

  // call update function
  mj_constraintUpdate(m, d, jar, NULL, 0);

  mj_freeStack(d);
  TM_END(mjTIMER_CONSTRAINT);
}



// inverse dynamics with skip; skipstage is mjtStage
void mj_inverseSkip(const mjModel* m, mjData* d,
                    int skipstage, int skipsensor) {
  TM_START;
  mj_markStack(d);
  mjtNum* qacc;
  int nv = m->nv;

  // position-dependent
  if (skipstage < mjSTAGE_POS) {
    mj_invPosition(m, d);
    if (!skipsensor) {
      mj_sensorPos(m, d);
    }
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyPos(m, d);
    }
  }

  // velocity-dependent
  if (skipstage < mjSTAGE_VEL) {
    mj_invVelocity(m, d);
    if (!skipsensor) {
      mj_sensorVel(m, d);
    }
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyVel(m, d);
    }
  }

  if (mjENABLED(mjENBL_INVDISCRETE)) {
    // save current qacc
    qacc = mjSTACKALLOC(d, nv, mjtNum);
    mju_copy(qacc, d->qacc, nv);

    // modify qacc in-place
    mj_discreteAcc(m, d);
  }

  // acceleration-dependent
  mj_invConstraint(m, d);
  mj_rne(m, d, 1, d->qfrc_inverse);
  if (!skipsensor) {
    mj_sensorAcc(m, d);
  }

  // qfrc_inverse += armature*qacc - qfrc_passive - qfrc_constraint
  for (int i=0; i < nv; i++) {
    d->qfrc_inverse[i] += m->dof_armature[i]*d->qacc[i]
                          - d->qfrc_passive[i] - d->qfrc_constraint[i];
  }

  if (mjENABLED(mjENBL_INVDISCRETE)) {
    // restore qacc
    mju_copy(d->qacc, qacc, nv);
  }

  mj_freeStack(d);
  TM_END(mjTIMER_INVERSE);
}



// inverse dynamics
void mj_inverse(const mjModel* m, mjData* d) {
  mj_inverseSkip(m, d, mjSTAGE_NONE, 0);
}



// compare forward and inverse dynamics, without changing results of forward
//    fwdinv[0] = norm(qfrc_constraint(forward) - qfrc_constraint(inverse))
//    fwdinv[1] = norm(qfrc_applied(forward) - qfrc_inverse)
void mj_compareFwdInv(const mjModel* m, mjData* d) {
  int nv = m->nv, nefc = d->nefc;
  mjtNum *qforce, *dif, *save_qfrc_constraint, *save_efc_force;

  // clear result, return if no constraints
  d->solver_fwdinv[0] = d->solver_fwdinv[1] = 0;
  if (!nefc) {
    return;
  }

  // allocate
  mj_markStack(d);
  qforce = mjSTACKALLOC(d, nv, mjtNum);
  dif = mjSTACKALLOC(d, nv, mjtNum);
  save_qfrc_constraint = mjSTACKALLOC(d, nv, mjtNum);
  save_efc_force = mjSTACKALLOC(d, nefc, mjtNum);

  // qforce = qfrc_applied + J'*xfrc_applied + qfrc_actuator
  //  should equal result of inverse dynamics
  mju_add(qforce, d->qfrc_applied, d->qfrc_actuator, nv);
  mj_xfrcAccumulate(m, d, qforce);

  // save forward dynamics results that are about to be modified
  mju_copy(save_qfrc_constraint, d->qfrc_constraint, nv);
  mju_copy(save_efc_force, d->efc_force, nefc);

  // run inverse dynamics, do not update position and velocity,
  mj_inverseSkip(m, d, mjSTAGE_VEL, 1);  // 1: do not recompute sensors and energy

  // compute statistics
  mju_sub(dif, save_qfrc_constraint, d->qfrc_constraint, nv);
  d->solver_fwdinv[0] = mju_norm(dif, nv);
  mju_sub(dif, qforce, d->qfrc_inverse, nv);
  d->solver_fwdinv[1] = mju_norm(dif, nv);

  // restore forward dynamics results
  mju_copy(d->qfrc_constraint, save_qfrc_constraint, nv);
  mju_copy(d->efc_force, save_efc_force, nefc);

  mj_freeStack(d);
}
