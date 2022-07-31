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

// compute tendon lengths, velocities and moment arms
MJAPI void mj_tendon(const mjModel* m, mjData* d);

// compute actuator transmission lengths and moments
MJAPI void mj_transmission(const mjModel* m, mjData* d);


//-------------------------- inertia ---------------------------------------------------------------

// composite rigid body inertia algorithm, with skip
void mj_crbSkip(const mjModel* m, mjData* d, int skipsimple);

// composite rigid body inertia algorithm
MJAPI void mj_crb(const mjModel* m, mjData* d);

// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
MJAPI void mj_factorI(const mjModel* m, mjData* d, const mjtNum* M, mjtNum* qLD, mjtNum* qLDiagInv,
                      mjtNum* qLDiagSqrtInv);

// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
MJAPI void mj_factorM(const mjModel* m, mjData* d);

// sparse backsubstitution:  x = inv(L'*D*L)*y
MJAPI void mj_solveLD(const mjModel* m, mjtNum* x, const mjtNum* y, int n,
                      const mjtNum* qLD, const mjtNum* qLDiagInv);

// sparse backsubstitution:  x = inv(L'*D*L)*y, use factorization in d
MJAPI void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
MJAPI void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);


//-------------------------- velocity --------------------------------------------------------------

// compute cvel, cdof_dot
MJAPI void mj_comVel(const mjModel* m, mjData* d);

// passive forces
MJAPI void mj_passive(const mjModel* m, mjData* d);

// subtree linear velocity and angular momentum
MJAPI void mj_subtreeVel(const mjModel* m, mjData* d);


//------------------------- fluid model ------------------------------------------------------------

// fluid forces based on inertia-box approximation
void mj_inertiaBoxFluidModel(const mjModel* m, mjData* d, int i);

// fluid forces based on ellipsoid approximation
void mj_ellipsoidFluidModel(const mjModel* m, mjData* d, int bodyid);

// compute forces due to added mass (potential flow)
void mj_addedMassForces(
    const mjtNum local_vels[6], const mjtNum local_accels[6],
    const mjtNum fluid_density, const mjtNum virtual_mass[3],
    const mjtNum virtual_inertia[3], mjtNum local_force[6]);

// compute forces due to viscous effects
void mj_viscousForces(
    const mjtNum local_vels[6], const mjtNum fluid_density,
    const mjtNum fluid_viscosity, const mjtNum size[3],
    const mjtNum magnus_lift_coef, const mjtNum kutta_lift_coef,
    const mjtNum blunt_drag_coef, const mjtNum slender_drag_coef,
    const mjtNum ang_drag_coef, mjtNum local_force[6]);

void readFluidGeomInteraction(const mjtNum * geom_fluid_coefs,
                              mjtNum * geom_fluid_coef,
                              mjtNum * blunt_drag_coef,
                              mjtNum * slender_drag_coef,
                              mjtNum * ang_drag_coef,
                              mjtNum * kutta_lift_coef,
                              mjtNum * magnus_lift_coef,
                              mjtNum virtual_mass[3],
                              mjtNum virtual_inertia[3]);

void writeFluidGeomInteraction (mjtNum * geom_fluid_coefs,
                                const mjtNum * geom_fluid_coef,
                                const mjtNum * blunt_drag_coef,
                                const mjtNum * slender_drag_coef,
                                const mjtNum * ang_drag_coef,
                                const mjtNum * kutta_lift_coef,
                                const mjtNum * magnus_lift_coef,
                                const mjtNum virtual_mass[3],
                                const mjtNum virtual_inertia[3]);


//-------------------------- RNE -------------------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
MJAPI void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);

// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
MJAPI void mj_rnePostConstraint(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
