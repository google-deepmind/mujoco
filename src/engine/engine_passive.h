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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_PASSIVE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_PASSIVE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------- passive forces ---------------------------------------------------------

// all passive forces
MJAPI void mj_passive(const mjModel* m, mjData* d);


//------------------------- fluid models -----------------------------------------------------------

// fluid forces based on inertia-box approximation
void mj_inertiaBoxFluidModel(const mjModel* m, mjData* d, int i);

// fluid forces based on ellipsoid approximation
void mj_ellipsoidFluidModel(const mjModel* m, mjData* d, int bodyid);

// compute forces due to added mass (potential flow)
void mj_addedMassForces(
    const mjtNum local_vels[6], const mjtNum local_accels[6],
    mjtNum fluid_density, const mjtNum virtual_mass[3],
    const mjtNum virtual_inertia[3], mjtNum local_force[6]);

// compute forces due to viscous effects
void mj_viscousForces(
    const mjtNum local_vels[6], mjtNum fluid_density,
    mjtNum fluid_viscosity, const mjtNum size[3],
    mjtNum magnus_lift_coef, mjtNum kutta_lift_coef,
    mjtNum blunt_drag_coef, mjtNum slender_drag_coef,
    mjtNum ang_drag_coef, mjtNum local_force[6]);

void readFluidGeomInteraction(const mjtNum* geom_fluid_coefs,
                              mjtNum* geom_fluid_coef,
                              mjtNum* blunt_drag_coef,
                              mjtNum* slender_drag_coef,
                              mjtNum* ang_drag_coef,
                              mjtNum* kutta_lift_coef,
                              mjtNum* magnus_lift_coef,
                              mjtNum virtual_mass[3],
                              mjtNum virtual_inertia[3]);

void writeFluidGeomInteraction (mjtNum* geom_fluid_coefs,
                                const mjtNum* geom_fluid_coef,
                                const mjtNum* blunt_drag_coef,
                                const mjtNum* slender_drag_coef,
                                const mjtNum* ang_drag_coef,
                                const mjtNum* kutta_lift_coef,
                                const mjtNum* magnus_lift_coef,
                                const mjtNum virtual_mass[3],
                                const mjtNum virtual_inertia[3]);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_PASSIVE_H_
