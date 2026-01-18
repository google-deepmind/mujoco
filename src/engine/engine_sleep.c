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
#include "engine/engine_core_util.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

// uncomment to print sleep/wake events
// #define MJ_DEBUG_SLEEP

//-------------------------------- update ----------------------------------------------------------

// compute sleeping arrays from tree_asleep, if flg_staticawake is set treat static bodies as awake
void mj_updateSleepInit(const mjModel* m, mjData* d, int flg_staticawake) {
  int ntree = m->ntree, nbody = m->nbody, nv = m->nv;

  // input arrays
  const int* tree_asleep   = d->tree_asleep;  // sleep state source of truth
  const int* body_treeid   = m->body_treeid;
  const int* body_parentid = m->body_parentid;
  const int* body_rootid   = m->body_rootid;
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
      if (body_mocapid[body_rootid[i]] >= 0) {
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


//-------------------------------- utilities -------------------------------------------------------

// return 1 if the weighted infinity norm of vec is smaller than tol, 0 otherwise
static int isSmaller(const mjtNum* vec, const mjtNum* weight, int n, mjtNum tol) {
  mjtNum max = 0;
  for (int i=0; i < n; i++) {
    max = mju_max(max, weight[i] * mju_abs(vec[i]));
    if (max >= tol) {
      return 0;
    }
  }
  return 1;
}


// return 1 if tree i can sleep, 0 otherwise
static int treeCanSleep(const mjModel* m, const mjData* d, int i, mjtNum tol) {
  // check sleep policy
  if (m->tree_sleep_policy[i] == mjSLEEP_NEVER ||
      m->tree_sleep_policy[i] == mjSLEEP_AUTO_NEVER) {
    return 0;
  }

  // check xfrc_applied
  int adr = m->tree_bodyadr[i];
  int num = m->tree_bodynum[i];
  if (!mju_isZeroByte((const unsigned char*)(d->xfrc_applied+6*adr), 6*num*sizeof(mjtNum))) {
    return 0;
  }

  // check qfrc_applied
  adr = m->tree_dofadr[i];
  num = m->tree_dofnum[i];
  if (!mju_isZeroByte((const unsigned char*)(d->qfrc_applied+adr), num*sizeof(mjtNum))) {
    return 0;
  }

  // check qvel
  if (tol) {
    return isSmaller(d->qvel+adr, m->dof_length+adr, num, tol);
  } else {
    return mju_isZeroByte((const unsigned char*)(d->qvel+adr), num*sizeof(mjtNum));
  }
}


// return the first tree in the sleep cycle that starts at i, -1 if error
int mj_sleepCycle(const int* tree_asleep, int ntree, int i) {
  if (i < 0 || i >= ntree) {
    return -1;  // index i out of bounds
  }

  int smallest = i;
  int current = i;
  int count = 0;

  do {
    if (count > ntree) {
      return -1;  // cycle detection failed (too many steps)
    }

    int next = tree_asleep[current];

    if (next < 0 || next >= ntree) {
      return -1;  // next index out of bounds
    }

    if (next < smallest) {
      smallest = next;
    }

    current = next;
    count++;
  } while (current != i);

  return smallest;
}


//-------------------------------- wake ------------------------------------------------------------

// wake tree i and its associated cycle, return number of woke trees
int mj_wakeTree(int* tree_asleep, int ntree, int i, int wakeval) {
  int nwoke = 0;

  // i is invalid; SHOULD NOT OCCUR
  if (i < 0 || i >= ntree) {
    mjERROR("invalid tree %d", i);
    return nwoke;
  }

  // tree i already awake: set to wakeval if larger than current value
  int asleep_val = tree_asleep[i];
  if (asleep_val < 0) {
    tree_asleep[i] = mjMIN(wakeval, asleep_val);
    return nwoke;
  }

  // tree i asleep: wake up tree and its island cycle
  else {
    int current = i;
    do {
      // get the index of the next tree in the cycle
      int next = tree_asleep[current];

      // next is invalid; SHOULD NOT OCCUR
      if (next < 0 || next >= ntree) {
        mjERROR("invalid sleep state index %d when waking tree %d", next, i);
        return 0;
      }

      // wake the current tree, increment count, advance to next
      tree_asleep[current] = wakeval;
      nwoke++;
      current = next;
    } while (current != i && nwoke < ntree);

    // did not come back to tree i, not a cycle; SHOULD NOT OCCUR
    if (current != i) {
      mjERROR("tree %d is not in a cycle", i);
      return 0;
    }
  }

  return nwoke;
}


static int kAwake = -(1+mjMINAWAKE);  // tree_asleep value for fully awake tree

// wake sleeping trees due to changes by user, return number of woke trees
int mj_wake(const mjModel* m, mjData* d) {
  int ntree = m->ntree, nwoke = 0;

  // sleep disabled
  if (!mjENABLED(mjENBL_SLEEP)) {
    // sleep disabled but some trees still asleep: wake all
    if (d->ntree_awake < ntree) {
      mju_fillInt(d->tree_asleep, kAwake, ntree);
    }
    return ntree - d->ntree_awake;
  }

  // sweep over trees, wake if required
  for (int i=0; i < ntree; i++) {
    int asleep = d->tree_asleep[i] >= 0;

    // awake: nothing to do
    if (!asleep) {
      continue;
    }

    // if qpos mismatch or cannot sleep: wake up
    if (d->tree_awake[i] || !treeCanSleep(m, d, i, 0)) {
      int woke = mj_wakeTree(d->tree_asleep, ntree, i, kAwake);
      if (woke) {
        nwoke += woke;

        #ifdef MJ_DEBUG_SLEEP
        printf("woke tree %d due to perturbation at t=%g\n", i, d->time);
        #endif
      }
    }
  }

  return nwoke;
}


// wake sleeping trees that touch awake trees, return number of woke trees
int mj_wakeCollision(const mjModel* m, mjData* d) {
  int ntree = m->ntree, ncon = d->ncon, nwoke = 0;

  if (!mjENABLED(mjENBL_SLEEP)) {
    return nwoke;
  }

  // sweep over contacts, wake trees if required
  for (int i=0; i < ncon; i++) {
    const mjContact* con = d->contact + i;

    // only geom-geom contacts are handled
    if (con->geom[0] < 0 || con->geom[1] < 0) {
      continue;
    }

    int b1 = m->geom_bodyid[con->geom[0]];
    int b2 = m->geom_bodyid[con->geom[1]];
    int tree1 = m->body_treeid[b1];
    int tree2 = m->body_treeid[b2];

    // contact with static body, nothing to do
    if (tree1 < 0 || tree2 < 0) {
      continue;
    }

    int awake1 = d->tree_awake[tree1];
    int awake2 = d->tree_awake[tree2];

    // both trees awake, nothing to do
    if (awake1 && awake2) {
      continue;
    }

    // both trees asleep; SHOULD NOT OCCUR
    if (!awake1 && !awake2) {
      mjERROR("contact between sleeping bodies %d and %d", b1, b2);
    }

    // wake sleeping tree
    int sleeping_tree = awake1 ? tree2 : tree1;
    int wakeval = awake1 ? d->tree_asleep[tree1] : d->tree_asleep[tree2];
    nwoke += mj_wakeTree(d->tree_asleep, ntree, sleeping_tree, wakeval);

    #ifdef MJ_DEBUG_SLEEP
    printf("woke tree %d due to contact at t=%g\n", sleeping_tree, d->time);
    #endif
  }

  return nwoke;
}


// wake sleeping trees with a constrained tendon to a waking tree, return number of woke trees
int mj_wakeTendon(const mjModel* m, mjData* d) {
  int ntendon = m->ntendon, nwoke = 0;

  if (!mjENABLED(mjENBL_SLEEP)) {
    return nwoke;
  }

  // sweep over tendons, wake trees if required
  for (int i=0; i < ntendon; i++) {
    if (m->tendon_treenum[i] != 2 || !tendonLimit(m, d->ten_length, i)) {
      continue;
    }

    int tree1 = m->tendon_treeid[2*i];
    int tree2 = m->tendon_treeid[2*i + 1];
    int awake1 = d->tree_awake[tree1];
    int awake2 = d->tree_awake[tree2];
    if (awake1 != awake2) {
      int sleeping_tree = awake1 ? tree2 : tree1;
      int wakeval = awake1 ? d->tree_asleep[tree1] : d->tree_asleep[tree2];
      nwoke += mj_wakeTree(d->tree_asleep, m->ntree, sleeping_tree, wakeval);

      #ifdef MJ_DEBUG_SLEEP
      printf("woke tree %d due to tendon constraint at t=%g\n", sleeping_tree, d->time);
      #endif
    }
  }

  return nwoke;
}


// wake sleeping trees with an equality to a waking tree, return number of woke trees
int mj_wakeEquality(const mjModel* m, mjData* d) {
  int neq = m->neq, nwoke = 0;

  if (!mjENABLED(mjENBL_SLEEP)) {
    return nwoke;
  }

  // sweep over equalities, wake trees if required
  for (int i=0; i < neq; i++) {
    // skip inactive
    if (!d->eq_active[i]) continue;

    mjtEq eqtype = m->eq_type[i];
    int id1 = m->eq_obj1id[i];
    int id2 = m->eq_obj2id[i];
    int tree1, tree2;

    switch (eqtype) {
    case mjEQ_CONNECT:
    case mjEQ_WELD:
      if (m->eq_objtype[i] == mjOBJ_BODY) {
        tree1 = m->body_treeid[id1];
        tree2 = m->body_treeid[id2];
      } else {
        tree1 = m->body_treeid[m->site_bodyid[id1]];
        tree2 = m->body_treeid[m->site_bodyid[id2]];
      }
      break;
    case mjEQ_JOINT:
      tree1 = id1 >= 0 ? m->body_treeid[m->jnt_bodyid[id1]] : -1;
      tree2 = id2 >= 0 ? m->body_treeid[m->jnt_bodyid[id2]] : -1;
      break;
    case mjEQ_TENDON:
      mjERROR("tendon equality does not yet support sleeping");
      continue;
    case mjEQ_FLEX:
      mjERROR("flex equality does not yet support sleeping");
      continue;
    default:
      continue;
    }

    // get sleep state
    mjtSleepState s1 = tree1 >= 0 ? d->tree_awake[tree1] : mjS_STATIC;
    mjtSleepState s2 = tree2 >= 0 ? d->tree_awake[tree2] : mjS_STATIC;

    // neither is asleep, nothing to do
    if (s1 != mjS_ASLEEP && s2 != mjS_ASLEEP) {
      continue;
    }

    // one is static, nothing to do
    if (s1 == mjS_STATIC || s2 == mjS_STATIC) {
      continue;
    }

    // equality within the same tree, nothing to do
    if (tree1 == tree2) {
      continue;
    }

    // both are asleep, wake if in different islands
    if (s1 == mjS_ASLEEP && s2 == mjS_ASLEEP) {
      int cycle1 = mj_sleepCycle(d->tree_asleep, m->ntree, tree1);
      int cycle2 = mj_sleepCycle(d->tree_asleep, m->ntree, tree2);
      if (cycle1 != cycle2) {
        int nwoke1 = mj_wakeTree(d->tree_asleep, m->ntree, tree1, kAwake);
        int nwoke2 = mj_wakeTree(d->tree_asleep, m->ntree, tree2, kAwake);

        #ifdef MJ_DEBUG_SLEEP
        printf("woke trees %d, %d due to equality %d at t=%g\n", tree1, tree2, i, d->time);
        #endif

        nwoke += nwoke1 + nwoke2;
      }
      continue;
    }

    // one is asleep and one is awake, wake the sleeping tree
    int sleeping_tree = s1 == mjS_ASLEEP ? tree1 : tree2;
    nwoke += mj_wakeTree(d->tree_asleep, m->ntree, sleeping_tree, kAwake);

    #ifdef MJ_DEBUG_SLEEP
    printf("woke tree %d due to equality %d at t=%g\n", sleeping_tree, i, d->time);
    #endif
  }

  return nwoke;
}


//-------------------------------- sleep -----------------------------------------------------------

// put n trees to sleep (create cycle), set their velocity and acceleration to zero
static inline void sleepTrees(const mjModel* m, mjData* d, const int* tree, int n) {
  for (int i=0; i < n; i++) {
    // create cycle
    int current = tree[i];
    int next = (i == n - 1) ? tree[0] : tree[i + 1];
    if (d->tree_asleep[current] == -1) {
      d->tree_asleep[current] = next;
    }

    // SHOULD NOT OCCUR
    else if (d->tree_asleep[current] >= 0) {
      mjERROR("trying to sleep tree %d which is already asleep", i);
    } else {
      mjERROR("trying to sleep tree %d which is not ready to sleep", i);
    }

    // set tree velocity and acceleration to zero
    int adr = m->tree_dofadr[current];
    int num = m->tree_dofnum[current];
    mju_zero(d->qvel+adr, num);
    mju_zero(d->qacc+adr, num);
  }

  #ifdef MJ_DEBUG_SLEEP
  if (n == 1) {
    printf("tree %d put to sleep at t=%g\n", tree[0], d->time);
  } else if (n > 1) {
    printf("trees ");
    for (int i = 0; i < n; i++) {
      printf("%d%s", tree[i], (i == n - 1) ? "" : ", ");
    }
    printf(" put to sleep at t=%g\n", d->time);
  }
  #endif
}


// put trees to sleep according to tolerance, return number of slept trees
int mj_sleep(const mjModel* m, mjData* d) {
  int ntree = m->ntree, nisland = d->nisland, nslept = 0;

  // sleep disabled: nothing to do
  if (!mjENABLED(mjENBL_SLEEP)) {
    return nslept;
  }

  // have constraints but no island structure: can't sleep
  if (d->nefc && !nisland) {
    return nslept;
  }

  // sweep over awake trees, increment tree_asleep if under tolerance
  for (int i=0; i < ntree; i++) {
    // skip sleeping tree
    if (d->tree_asleep[i] >= 0) {
      continue;
    }

    // increment tree_asleep if tree can sleep, otherwise wake up
    if (treeCanSleep(m, d, i, m->opt.sleep_tolerance)) {
      d->tree_asleep[i] += (d->tree_asleep[i] < -1);
    } else {
      d->tree_asleep[i] = -(1+mjMINAWAKE);
    }
  }

  // sweep over islands, put to sleep if all trees are under tolerance
  for (int i=0; i < nisland; i++) {
    // check if all trees in the island can sleep
    int can_sleep = 1;
    int start = d->island_itreeadr[i];
    int end = start + d->island_ntree[i];
    for (int j=start; j < end; j++) {
      int tree_asleep = d->tree_asleep[d->map_itree2tree[j]];
      if (tree_asleep < -1) {
        can_sleep = 0;
        break;
      }

      // sleeping tree in an island; SHOULD NOT OCCUR
      else if (tree_asleep >= 0) {
        mjERROR("found sleeping tree %d in island %d", d->map_itree2tree[j], i);
      }
    }

    // put island to sleep
    if (can_sleep) {
      const int* tree = d->map_itree2tree + start;
      int n = d->island_ntree[i];
      sleepTrees(m, d, tree, n);
      nslept += n;
    }
  }

  // sleep unconstrained trees (with or without island structure)
  int start = nisland ? d->island_itreeadr[nisland-1] + d->island_ntree[nisland-1] : 0;
  for (int j=start; j < ntree; j++) {
    int i = nisland ? d->map_itree2tree[j] : j;
    if (d->tree_asleep[i] == -1) {
      sleepTrees(m, d, &i, 1);
      nslept++;
    }
  }

  return nslept;
}


//-------------------------------- sleep state -----------------------------------------------------

// return sleep state of tendon i
static mjtSleepState mj_tendonSleepState(const mjModel* m, const mjData* d, int i) {
  int treenum = m->tendon_treenum[i];

  // no trees: tendon is static
  if (treenum == 0) {
    return mjS_STATIC;
  }

  // single tree: awake if tree is awake, asleep otherwise
  int id1 = m->tendon_treeid[2*i];
  if (treenum == 1) {
    return d->tree_awake[id1] ? mjS_AWAKE : mjS_ASLEEP;
  }

  // two trees: asleep only if both are asleep
  int id2 = m->tendon_treeid[2*i+1];
  if (treenum == 2) {
    return (d->tree_awake[id1] || d->tree_awake[id2]) ? mjS_AWAKE : mjS_ASLEEP;
  }

  return mjS_AWAKE;
}


// return sleep state of actuator i
static mjtSleepState mj_actuatorSleepState(const mjModel* m, const mjData* d, int i) {
  mjtSleepState s1, s2;
  int trnid = m->actuator_trnid[i*2];

  switch ((mjtTrn)m->actuator_trntype[i]) {
  case mjTRN_JOINT:
  case mjTRN_JOINTINPARENT:
    return mj_sleepState(m, d, mjOBJ_JOINT, trnid);

  case mjTRN_SLIDERCRANK:
    s1 = mj_sleepState(m, d, mjOBJ_SITE, trnid);
    s2 = mj_sleepState(m, d, mjOBJ_SITE, m->actuator_trnid[i*2+1]);
    return (s1 == mjS_AWAKE || s2 == mjS_AWAKE) ? mjS_AWAKE : mjS_ASLEEP;

  case mjTRN_TENDON:
    return mj_tendonSleepState(m, d, trnid);

  case mjTRN_SITE:
    return mj_sleepState(m, d, mjOBJ_SITE, trnid);

  case mjTRN_BODY:
    return mj_sleepState(m, d, mjOBJ_BODY, trnid);

  case mjTRN_UNDEFINED:
    return mjS_AWAKE;
  }

  return mjS_AWAKE;
}


// return sleep state of equality i
static mjtSleepState mj_equalitySleepState(const mjModel* m, const mjData* d, int i) {
  mjtEq eqtype = m->eq_type[i];
  mjtObj objtype;

  switch (eqtype) {
    case mjEQ_CONNECT:
    case mjEQ_WELD:
      objtype = m->eq_objtype[i];
      break;
    case mjEQ_JOINT:
      objtype = mjOBJ_JOINT;
      break;
    case mjEQ_TENDON:
      objtype = mjOBJ_TENDON;
      break;
    case mjEQ_FLEX:
      objtype = mjOBJ_FLEX;
      break;
    default:
      return mjS_AWAKE;
  }

  int id1 = m->eq_obj1id[i];
  int id2 = m->eq_obj2id[i];
  mjtSleepState s1 = (id1 >= 0) ? mj_sleepState(m, d, objtype, id1) : mjS_STATIC;
  mjtSleepState s2 = (id2 >= 0) ? mj_sleepState(m, d, objtype, id2) : mjS_STATIC;

  // return ASLEEP if both objects are asleep or static, AWAKE otherwise
  int neither_awake = (s1 != mjS_AWAKE && s2 != mjS_AWAKE);
  return neither_awake ? mjS_ASLEEP : mjS_AWAKE;
}


// return sleep state of sensor i (AWAKE or ASLEEP, never STATIC)
static mjtSleepState mj_sensorSleepState(const mjModel* m, const mjData* d, int i) {
  mjtSensor type = m->sensor_type[i];
  mjtObj objtype = m->sensor_objtype[i];
  mjtObj reftype = m->sensor_reftype[i];

  // special handling for specific sensor types
  switch (type) {

  // USER and PLUGIN sensors: always awake
  case mjSENS_USER:
  case mjSENS_PLUGIN:
    return mjS_AWAKE;

  // contact sensors with site specifiers: always awake
  case mjSENS_CONTACT:
    // site used to define a volume: always awake
    if (objtype == mjOBJ_SITE || reftype == mjOBJ_SITE) {
      return mjS_AWAKE;
    }
    break;

  // rangefinder output does not depend on sleep state: always awake
  case mjSENS_RANGEFINDER:
    return mjS_AWAKE;

  default:
    break;
  }

  // get sleep state of the primary and reference objects
  mjtSleepState s_obj = mj_sleepState(m, d, objtype, m->sensor_objid[i]);
  mjtSleepState s_ref = mj_sleepState(m, d, reftype, m->sensor_refid[i]);

  // special handling for UNKNOWN objects

  // if both are UNKNOWN, return AWAKE
  if (objtype == mjOBJ_UNKNOWN && reftype == mjOBJ_UNKNOWN) {
    return mjS_AWAKE;
  }

  // if one is UNKNOWN, return the other's sleep state (if STATIC, return AWAKE)
  if (objtype == mjOBJ_UNKNOWN) {
    return s_ref == mjS_ASLEEP ? mjS_ASLEEP : mjS_AWAKE;
  } else if (reftype == mjOBJ_UNKNOWN) {
    return s_obj == mjS_ASLEEP ? mjS_ASLEEP : mjS_AWAKE;
  }

  // if either object is awake, return AWAKE
  if (s_obj == mjS_AWAKE || s_ref == mjS_AWAKE) {
    return mjS_AWAKE;
  }

  // otherwise return ASLEEP
  return mjS_ASLEEP;
}


// return sleep state of object i
mjtSleepState mj_sleepState(const mjModel* m, const mjData* d, mjtObj type, int i) {
  const char* typename;

  switch (type) {

  // simple types
  case mjOBJ_BODY:
  case mjOBJ_XBODY:
    return (mjtSleepState) d->body_awake[i];
  case mjOBJ_JOINT:
    return (mjtSleepState) d->body_awake[m->jnt_bodyid[i]];
  case mjOBJ_SITE:
    return (mjtSleepState) d->body_awake[m->site_bodyid[i]];
  case mjOBJ_DOF:
    return (mjtSleepState) d->body_awake[m->dof_bodyid[i]];
  case mjOBJ_GEOM:
    return (mjtSleepState) d->body_awake[m->geom_bodyid[i]];
  case mjOBJ_CAMERA:
    return (mjtSleepState) d->body_awake[m->cam_bodyid[i]];
  case mjOBJ_LIGHT:
    return (mjtSleepState) d->body_awake[m->light_bodyid[i]];

  // complex types
  case mjOBJ_EQUALITY:
    return mj_equalitySleepState(m, d, i);
  case mjOBJ_TENDON:
    return mj_tendonSleepState(m, d, i);
  case mjOBJ_ACTUATOR:
    return mj_actuatorSleepState(m, d, i);
  case mjOBJ_SENSOR:
    return mj_sensorSleepState(m, d, i);

  // always awake
  case mjOBJ_FLEX:
    return mjS_AWAKE;

  // undefined sleep state, return AWAKE
  case mjOBJ_UNKNOWN:
    return mjS_AWAKE;

  // unsupported
  default:
    typename = mju_type2Str(type);
    if (typename) {
      mjERROR("unsupported object type '%s'", typename);
    } else {
      mjERROR("unsupported object type %d", type);
    }
    return mjS_AWAKE;
  }
}


#ifdef MJ_DEBUG_SLEEP
  #undef MJ_DEBUG_SLEEP
#endif
