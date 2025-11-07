// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SLEEP_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SLEEP_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// compute sleeping arrays from tree_asleep, if flg_staticawake is set, treat static bodies as awake
void mj_updateSleepInit(const mjModel* m, mjData* d, int flg_staticawake);

// compute {ntree,nbody,nv}_awake, {tree,body}_awake, {body,dof}_awake_ind from tree_asleep
MJAPI void mj_updateSleep(const mjModel* m, mjData* d);

// return the first tree in the sleep cycle that starts at i, -1 if error
int mj_sleepCycle(const int* tree_asleep, int ntree, int i);

// return the first tree in the sleep cycle that starts at i, -1 if error
int mj_sleepCycle(const int* tree_asleep, int ntree, int i);

// wake tree i and its related island cycle, return number of woke trees
MJAPI int mj_wakeTree(int* tree_asleep, int ntree, int i, int wakeval);

// wake trees with nonzero velocity or external forces, return number of woke trees
int mj_wake(const mjModel* m, mjData* d);

// wake sleeping trees that touch awake trees, return number of woke trees
int mj_wakeCollision(const mjModel* m, mjData* d);

// wake sleeping trees with a constrained tendon to a waking tree, return number of woke trees
int mj_wakeTendon(const mjModel* m, mjData* d);

// wake sleeping trees with an equality to a waking tree, return number of woke trees
int mj_wakeEquality(const mjModel* m, mjData* d);

// put trees to sleep according to tolerance, return number of slept trees
int mj_sleep(const mjModel* m, mjData* d);

// return sleep state of object i
mjtSleepState mj_sleepState(const mjModel* m, const mjData* d, mjtObj type, int i);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SLEEP_H_
