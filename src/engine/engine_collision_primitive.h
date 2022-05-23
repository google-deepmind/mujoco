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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_PRIMITIVE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_PRIMITIVE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

// define and extract geom info
#define mjGETINFO \
    mjtNum* pos1 = d->geom_xpos + 3*g1; \
    mjtNum* mat1 = d->geom_xmat + 9*g1; \
    mjtNum* size1= m->geom_size + 3*g1; \
    mjtNum* pos2 = d->geom_xpos + 3*g2; \
    mjtNum* mat2 = d->geom_xmat + 9*g2; \
    mjtNum* size2= m->geom_size + 3*g2; \
    (void) size1; (void) size2;

#ifdef __cplusplus
extern "C" {
#endif

// plane collisions
int mjc_PlaneSphere     (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_PlaneCapsule    (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_PlaneCylinder   (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_PlaneBox        (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);

// sphere and capsule collisions
int mjc_SphereSphere    (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_SphereCapsule   (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_CapsuleCapsule  (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);

// box collisions: from boxcollisions.c
int mjc_CapsuleBox      (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_SphereBox       (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);
int mjc_BoxBox          (const mjModel* m, const mjData* d,
                         mjContact* con, int g1, int g2, mjtNum margin);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_PRIMITIVE_H_
