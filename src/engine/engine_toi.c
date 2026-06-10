// Copyright 2026 DeepMind Technologies Limited
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

#include "engine/engine_toi.h"

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_gjk.h"
#include "engine/engine_core_util.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"

// maximum number of conservative advancement iterations
#define mjMAXTOIITER 100

//---------------------------------- internal helpers ---------------------------------------------

// geom types supported by time-of-impact queries (compact convex shapes)
static int isTOISupported(mjtGeom type) {
  return type == mjGEOM_SPHERE   || type == mjGEOM_CAPSULE || type == mjGEOM_ELLIPSOID ||
         type == mjGEOM_CYLINDER || type == mjGEOM_BOX     || type == mjGEOM_MESH;
}


// advance obj pose to time t from initial pose (pos0, quat0) under constant world-frame
// velocity vel = [rot(3); lin(3)]: center moves linearly, rotation is about the moving center
static void advancePose(mjCCDObj* obj, const mjtNum pos0[3], const mjtNum quat0[4],
                        const mjtNum vel[6], mjtNum t) {
  // linear: pos = pos0 + t * v
  mju_addScl3(obj->pos, pos0, vel+3, t);

  // angular: quat = axisAngle(w_hat, |w|*t) * quat0 (left-multiply: w is world-frame)
  mjtNum speed = mju_norm3(vel);
  if (speed*t < mjMINVAL) {
    mju_quat2Mat(obj->mat, quat0);
  } else {
    mjtNum axis[3] = {vel[0]/speed, vel[1]/speed, vel[2]/speed};
    mjtNum qrot[4], quat[4];
    mju_axisAngle2Quat(qrot, axis, speed*t);
    mju_mulQuat(quat, qrot, quat0);
    mju_quat2Mat(obj->mat, quat);
  }
}


// upper bound on the closing speed of the separation along unit direction n (from obj1 to obj2):
// any surface point of obj i moves no faster than |v_i| + |w_i|*rbound_i, so the separation
// shrinks at a rate of at most (v1 - v2)'n + |w1|*rbound1 + |w2|*rbound2
static mjtNum closingSpeedBound(const mjtNum vel1[6], const mjtNum vel2[6],
                                mjtNum rbound1, mjtNum rbound2, const mjtNum n[3]) {
  mjtNum vrel[3];
  mju_sub3(vrel, vel1+3, vel2+3);
  return mju_dot3(vrel, n) + mju_norm3(vel1)*rbound1 + mju_norm3(vel2)*rbound2;
}


//---------------------------------- time of impact -----------------------------------------------

// conservative-advancement core, see header for semantics
mjtNum mjc_toi(const mjCCDConfig* config, mjCCDStatus* status,
               mjCCDObj* obj1, const mjtNum vel1[6], mjtNum rbound1,
               mjCCDObj* obj2, const mjtNum vel2[6], mjtNum rbound2,
               mjtNum horizon, mjtNum tolerance) {
  // save initial poses
  mjtNum pos0_1[3], pos0_2[3], quat0_1[4], quat0_2[4];
  mju_copy3(pos0_1, obj1->pos);
  mju_copy3(pos0_2, obj2->pos);
  mju_mat2Quat(quat0_1, obj1->mat);
  mju_mat2Quat(quat0_2, obj2->mat);

  // direction-independent bound on total closing speed
  mjtNum vrel[3];
  mju_sub3(vrel, vel1+3, vel2+3);
  mjtNum vmax = mju_norm3(vrel) + mju_norm3(vel1)*rbound1 + mju_norm3(vel2)*rbound2;

  mjtNum t = 0;
  for (int iter=0; iter < mjMAXTOIITER; iter++) {
    advancePose(obj1, pos0_1, quat0_1, vel1, t);
    advancePose(obj2, pos0_2, quat0_2, vel2, t);

    // distance query, cutoff at the largest gap still closable in the remaining time
    mjCCDConfig cfg = *config;
    cfg.dist_cutoff = vmax*(horizon - t) + tolerance;
    mjtNum dist = mjc_ccd(&cfg, status, obj1, obj2);

    // gap provably unreachable within horizon
    if (dist == mjMAX_LIMIT) {
      return -1;
    }

    // impact (covers t = 0 touching/penetration)
    if (dist <= tolerance) {
      return t;
    }

    // closing direction from witness points
    mjtNum n[3];
    mju_sub3(n, status->x2, status->x1);
    mju_scl3(n, n, 1/dist);

    // not approaching along the closing direction
    mjtNum vbound = closingSpeedBound(vel1, vel2, rbound1, rbound2, n);
    if (vbound <= mjMINVAL) {
      return -1;
    }

    // advance: objects cannot touch before t + dist/vbound
    t += dist/vbound;
    if (t > horizon) {
      return -1;
    }
  }

  // iteration cap: t is a conservative lower bound on the true TOI
  return t;
}


// time of impact between two geoms, see header for semantics
mjtNum mj_geomTOI(const mjModel* m, mjData* d, int geom1, int geom2, mjtNum horizon,
                  mjtNum fromto[6]) {
  if (fromto) {
    mju_zero(fromto, 6);
  }

  // check inputs
  if (geom1 < 0 || geom1 >= m->ngeom || geom2 < 0 || geom2 >= m->ngeom) {
    mjERROR("invalid geom id %d or %d", geom1, geom2);
  }
  if (horizon < 0) {
    mjERROR("negative horizon %g", horizon);
  }
  if (!isTOISupported(m->geom_type[geom1]) || !isTOISupported(m->geom_type[geom2])) {
    mjERROR("unsupported geom type: supported types are sphere, capsule, ellipsoid, "
            "cylinder, box, mesh");
  }

  mj_markStack(d);

  // set config; dist_cutoff is managed by mjc_toi
  mjCCDConfig config;
  mjCCDStatus status;
  config.max_iterations = m->opt.ccd_iterations;
  config.tolerance = m->opt.ccd_tolerance;
  config.max_contacts = 1;
  config.dist_cutoff = mjMAX_LIMIT;
  config.buffer = mj_stackAllocByte(d, mjc_ccdSize(config.max_iterations), sizeof(mjtNum));

  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, m, d, geom1, 0);
  mjc_initCCDObj(&obj2, m, d, geom2, 0);

  // world-frame velocities at the geom centers
  mjtNum vel1[6], vel2[6];
  mj_objectVelocity(m, d, mjOBJ_GEOM, geom1, vel1, /*flg_local=*/0);
  mj_objectVelocity(m, d, mjOBJ_GEOM, geom2, vel2, /*flg_local=*/0);

  mjtNum toi = mjc_toi(&config, &status, &obj1, vel1, m->geom_rbound[geom1],
                       &obj2, vel2, m->geom_rbound[geom2], horizon, m->opt.ccd_tolerance);

  // witness points at the impact poses
  if (toi >= 0 && fromto && status.nx > 0) {
    mju_copy3(fromto, status.x1);
    mju_copy3(fromto+3, status.x2);
  }

  mj_freeStack(d);
  return toi;
}
