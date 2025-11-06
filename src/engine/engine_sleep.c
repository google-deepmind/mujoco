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

#include "engine/engine_sleep.h"

#include <stdio.h>
#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>


//-------------------------------- update ----------------------------------------------------------

// compute sleeping arrays from tree_asleep, if flg_staticawake is set treat static bodies as awake
void mj_updateSleepInit(const mjModel* m, mjData* d, int flg_staticawake) {
  int ntree = m->ntree, nbody = m->nbody, nv = m->nv;

  // input arrays
  const int* tree_asleep   = d->tree_asleep;  // sleep state source of truth
  const int* body_treeid   = m->body_treeid;
  const int* body_parentid = m->body_parentid;
  const int* body_mocapid  = m->body_mocapid;
  const int* dof_bodyid    = m->dof_bodyid;

  // output arrays
  int* tree_awake        = d->tree_awake;
  int* body_awake        = d->body_awake;
  int* dof_awake_ind     = d->dof_awake_ind;
  int* body_awake_ind    = d->body_awake_ind;
  int* parent_awake_ind  = d->parent_awake_ind;

  // tree_awake
  int ntree_awake = 0;
  for (int i=0; i < ntree; i++) {
    tree_awake[i] = tree_asleep[i] < 0;
    ntree_awake += tree_awake[i];
  }
  d->ntree_awake = ntree_awake;

  // {body,parent}_awake_ind
  int nbody_awake = 0;
  int nparent_awake = 0;
  for (int i=0; i < nbody; i++) {
    // static body
    if (body_treeid[i] < 0) {
      if (body_mocapid[i] >= 0) {
        // mocap body are always awake
        body_awake[i] = mjS_AWAKE;
      } else {
        // mark static body unless flg_staticawake is set
        body_awake[i] = flg_staticawake ? mjS_AWAKE : mjS_STATIC;
      }
    }

    // dynamic body
    else {
      body_awake[i] = tree_awake[body_treeid[i]] ? mjS_AWAKE : mjS_ASLEEP;
    }

    // body_awake_ind: list of awake and static bodies
    if (body_awake[i] != mjS_ASLEEP) {
      body_awake_ind[nbody_awake++] = i;
    }

    // parent_awake_ind: list of bodies with awake or static parents
    if (i && body_awake[body_parentid[i]] != mjS_ASLEEP) {
      parent_awake_ind[nparent_awake++] = i;
    }
  }
  d->nbody_awake = nbody_awake;
  d->nparent_awake = nparent_awake;

  // dof_awake_ind: list of awake degrees of freedom
  int nv_awake = 0;
  for (int i=0; i < nv; i++) {
    int bodyid = dof_bodyid[i];
    if (body_treeid[bodyid] >= 0 && body_awake[bodyid] == mjS_AWAKE) {
      dof_awake_ind[nv_awake++] = i;
    }
  }
  d->nv_awake = nv_awake;
}

// compute sleep arrays from tree_asleep
void mj_updateSleep(const mjModel* m, mjData* d) {
  mj_updateSleepInit(m, d, /*flg_staticawake*/0);
}
