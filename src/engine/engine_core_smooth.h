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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
#define MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif
//-------------------------- position --------------------------------------------------------------

// forward kinematics
MJAPI void mj_kinematics(const mjModel* m, mjData* d);

// map inertias and motion dofs to global frame centered at CoM
MJAPI void mj_comPos(const mjModel* m, mjData* d);

// compute camera and light positions and orientations
MJAPI void mj_camlight(const mjModel* m, mjData* d);

// compute flex-related quantities
MJAPI void mj_flex(const mjModel* m, mjData* d);

// compute tendon lengths, velocities and moment arms
MJAPI void mj_tendon(const mjModel* m, mjData* d);

// compute actuator transmission lengths and moments
MJAPI void mj_transmission(const mjModel* m, mjData* d);


//-------------------------- inertia ---------------------------------------------------------------

// composite rigid body inertia algorithm
MJAPI void mj_crb(const mjModel* m, mjData* d);

// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
MJAPI void mj_factorI(const mjModel* m, mjData* d, const mjtNum* M, mjtNum* qLD, mjtNum* qLDiagInv,
                      mjtNum* qLDiagSqrtInv);

// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
MJAPI void mj_factorM(const mjModel* m, mjData* d);

// sparse backsubstitution:  x = inv(L'*D*L)*y
MJAPI void mj_solveLD(const mjModel* m, mjtNum* x, int n,
                      const mjtNum* qLD, const mjtNum* qLDiagInv);

// sparse backsubstitution:  x = inv(L'*D*L)*y, use factorization in d
MJAPI void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

// sparse backsubstitution for one island:  x = inv(L'*D*L)*x, use factorization in d
MJAPI void mj_solveM_island(const mjModel* m, const mjData* d, mjtNum* x, int island);

// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
MJAPI void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);


//-------------------------- velocity --------------------------------------------------------------

// compute cvel, cdof_dot
MJAPI void mj_comVel(const mjModel* m, mjData* d);

// subtree linear velocity and angular momentum
MJAPI void mj_subtreeVel(const mjModel* m, mjData* d);


//-------------------------- RNE -------------------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
MJAPI void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);

// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
MJAPI void mj_rnePostConstraint(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
