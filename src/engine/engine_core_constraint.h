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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_CORE_CONSTRAINT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_CORE_CONSTRAINT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------- Jacobian-related ------------------------------------------------------

// determine type of friction cone
MJAPI int mj_isPyramidal(const mjModel* m);

// determine type of constraint Jacobian
MJAPI int mj_isSparse(const mjModel* m);

// determine type of solver
MJAPI int mj_isDual(const mjModel* m);

// multiply Jacobian by vector
MJAPI void mj_mulJacVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec);

// multiply JacobianT by vector
MJAPI void mj_mulJacTVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec);


//-------------------------- utility functions -----------------------------------------------------

// assign/override solver reference parameters
void mj_assignRef(const mjModel* m, mjtNum* target, const mjtNum* source);

// assign/override solver impedance parameters
void mj_assignImp(const mjModel* m, mjtNum* target, const mjtNum* source);

// assign/override geom/limit/tendon margin
mjtNum mj_assignMargin(const mjModel* m, mjtNum source);

// add contact to d->contact list; return 0 if success; 1 if buffer full
MJAPI int mj_addContact(const mjModel* m, mjData* d, const mjContact* con);

// add #size rows to constraint Jacobian; set pos, margin, frictionloss, type, id
//  result: 0=success; 1=buffer full
int mj_addConstraint(const mjModel* m, mjData* d,
                     const mjtNum* jac, const mjtNum* pos,
                     const mjtNum* margin, mjtNum frictionloss,
                     int size, int type, int id, int NV, const int* chain);

// merge dof chains for two bodies
int mj_mergeChain(const mjModel* m, int* chain, int b1, int b2);

// merge dof chains for two simple bodies
int mj_mergeChainSimple(const mjModel* m, int* chain, int b1, int b2);


//-------------------------- constraint instantiation ----------------------------------------------

// equality constraints
void mj_instantiateEquality(const mjModel* m, mjData* d);

// frictional dofs and tendons
void mj_instantiateFriction(const mjModel* m, mjData* d);

// joint and tendon limits
void mj_instantiateLimit(const mjModel* m, mjData* d);

// frictionelss and frictional contacts
void mj_instantiateContact(const mjModel* m, mjData* d);


//------------------------ parameter computation/extraction ----------------------------------------

// compute efc_diagApprox
void mj_diagApprox(const mjModel* m, mjData* d);

// compute efc_R, efc_D, efc_KDIP, adjust diagApprox
void mj_makeImpedance(const mjModel* m, mjData* d);


//---------------------------- top-level API for constraint construction ---------------------------

// main driver: call all functions above
MJAPI void mj_makeConstraint(const mjModel* m, mjData* d);

// compute efc_AR
MJAPI void mj_projectConstraint(const mjModel* m, mjData* d);

// compute efc_vel, efc_aref
MJAPI void mj_referenceConstraint(const mjModel* m, mjData* d);

// compute efc_state, efc_force, qfrc_constraint
// optional: cost(qacc) = shat(jar) where jar = Jac*qacc-aref; cone Hessians
MJAPI void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar,
                               mjtNum cost[1], int flg_coneHessian);
#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_CORE_CONSTRAINT_H_
