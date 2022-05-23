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
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_sensor.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_sparse.h"

// position-dependent computations
void mj_invPosition(const mjModel* m, mjData* d) {
  TM_START1;
  TM_START;

  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_camlight(m, d);
  mj_tendon(m, d);
  mj_transmission(m, d);
  TM_END(mjTIMER_POS_KINEMATICS);

  TM_RESTART;
  mj_crb(m, d);
  mj_factorM(m, d);
  TM_END(mjTIMER_POS_INERTIA);

  TM_RESTART;
  mj_collision(m, d);
  TM_END(mjTIMER_POS_COLLISION);

  TM_RESTART;
  mj_makeConstraint(m, d);
  TM_END(mjTIMER_POS_MAKE);

  TM_END1(mjTIMER_POSITION);
}



// velocity-dependent computations
void mj_invVelocity(const mjModel* m, mjData* d) {
  TM_START;

  // tendon velocity: dense or sparse
  if (mj_isSparse(m)) {
    mju_mulMatVecSparse(d->ten_velocity, d->ten_J, d->qvel, m->ntendon,
                        d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind, NULL);
  } else {
    mju_mulMatVec(d->ten_velocity, d->ten_J, d->qvel, m->ntendon, m->nv);
  }

  // actuator velocity
  mju_mulMatVec(d->actuator_velocity, d->actuator_moment, d->qvel, m->nu, m->nv);

  // standard velocity computations
  mj_comVel(m, d);
  mj_passive(m, d);
  mj_referenceConstraint(m, d);

  // compute qfrc_bias with abbreviated RNE (without acceleration)
  mj_rne(m, d, 0, d->qfrc_bias);

  TM_END(mjTIMER_VELOCITY);
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

  mjMARKSTACK;
  mjtNum* jar = mj_stackAlloc(d, nefc);

  // compute jar = Jac*qacc - aref
  mj_mulJacVec(m, d, jar, d->qacc);
  mju_subFrom(jar, d->efc_aref, nefc);

  // call update function
  mj_constraintUpdate(m, d, jar, NULL, 0);

  mjFREESTACK;
  TM_END(mjTIMER_CONSTRAINT);
}



// inverse dynamics with skip; skipstage is mjtStage
void mj_inverseSkip(const mjModel* m, mjData* d,
                    int skipstage, int skipsensor) {
  TM_START;
  int nv = m->nv;

  // position-dependent
  if (skipstage<mjSTAGE_POS) {
    mj_invPosition(m, d);
    if (!skipsensor) {
      mj_sensorPos(m, d);
    }
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyPos(m, d);
    }
  }

  // velocity-dependent
  if (skipstage<mjSTAGE_VEL) {
    mj_invVelocity(m, d);
    if (!skipsensor) {
      mj_sensorVel(m, d);
    }
    if (mjENABLED(mjENBL_ENERGY)) {
      mj_energyVel(m, d);
    }
  }

  // acceleration-dependent
  mj_invConstraint(m, d);
  mj_rne(m, d, 1, d->qfrc_inverse);
  if (!skipsensor) {
    mj_sensorAcc(m, d);
  }

  // qfrc_inverse += artmature*qacc - qfrc_passive - qfrc_constraint
  for (int i=0; i<nv; i++) {
    d->qfrc_inverse[i] += m->dof_armature[i]*d->qacc[i]
                          - d->qfrc_passive[i] - d->qfrc_constraint[i];
  }

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
  mjMARKSTACK;

  // clear result, return if no constraints
  d->solver_fwdinv[0] = d->solver_fwdinv[1] = 0;
  if (!nefc) {
    return;
  }

  // allocate
  qforce = mj_stackAlloc(d, nv);
  dif = mj_stackAlloc(d, nv);
  save_qfrc_constraint = mj_stackAlloc(d, nv);
  save_efc_force = mj_stackAlloc(d, nefc);

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

  mjFREESTACK;
}
