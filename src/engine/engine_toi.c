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


// advancement context for one object: rigid initial pose, or a local flex element view that
// lets the support function read advanced vertex positions without touching mjData (all view
// arrays are indexed with flex id 0 and element 0 after rewiring)
typedef struct {
  int isflex;

  // rigid initial pose
  mjtNum pos0[3];
  mjtNum quat0[4];

  // local flex view, read by mjc_flexSupport/mjc_center through obj->data.flex
  int dim[1];
  int elemadr[1];
  int vertadr[1];
  int elemdataadr[1];
  int elem[4];
  mjtNum aabb[6];
  mjtNum xradius[1];
  mjtNum vert[12];      // advanced vertex positions
} mjTOIContext;


// initialize advancement context; flex objects are rewired to read the local view
static void toiInit(mjTOIContext* ctx, mjCCDObj* obj, const mjTOIMotion* motion) {
  ctx->isflex = motion->nvert > 0;

  // rigid: save initial pose
  if (!ctx->isflex) {
    mju_copy3(ctx->pos0, obj->pos);
    mju_mat2Quat(ctx->quat0, obj->mat);
    return;
  }

  // flex: copy the per-flex constants from the original arrays before rewiring
  int f = obj->flex;
  ctx->dim[0] = obj->data.flex.dim[f];
  ctx->xradius[0] = obj->data.flex.xradius[f];
  ctx->elemadr[0] = 0;
  ctx->vertadr[0] = 0;
  ctx->elemdataadr[0] = 0;
  for (int i=0; i < motion->nvert; i++) {
    ctx->elem[i] = i;
  }
  mju_zero(ctx->aabb, 6);

  // rewire the object to the local view
  obj->flex = 0;
  if (obj->elem >= 0) {
    obj->elem = 0;
  } else {
    obj->vert = 0;
  }
  obj->data.flex.dim = ctx->dim;
  obj->data.flex.aabb = ctx->aabb;
  obj->data.flex.elemadr = ctx->elemadr;
  obj->data.flex.vert_xpos = ctx->vert;
  obj->data.flex.vertadr = ctx->vertadr;
  obj->data.flex.xradius = ctx->xradius;
  obj->data.flex.elemdataadr = ctx->elemdataadr;
  obj->data.flex.elem = ctx->elem;
}


// advance object configuration to time t
static void toiAdvance(mjTOIContext* ctx, mjCCDObj* obj, const mjTOIMotion* motion, mjtNum t) {
  // flex: per-vertex linear motion; keep the aabb center (GJK seed) at the vertex mean
  if (ctx->isflex) {
    mju_zero3(ctx->aabb);
    for (int i=0; i < motion->nvert; i++) {
      mju_addScl3(ctx->vert + 3*i, motion->vertpos + 3*i, motion->vertvel + 3*i, t);
      mju_addTo3(ctx->aabb, ctx->vert + 3*i);
    }
    mju_scl3(ctx->aabb, ctx->aabb, 1.0/motion->nvert);
    return;
  }

  // rigid linear: pos = pos0 + t * v
  mju_addScl3(obj->pos, ctx->pos0, motion->vel+3, t);

  // rigid angular: quat = axisAngle(w_hat, |w|*t) * quat0 (left-multiply: w is world-frame)
  mjtNum speed = mju_norm3(motion->vel);
  if (speed*t < mjMINVAL) {
    mju_quat2Mat(obj->mat, ctx->quat0);
  } else {
    mjtNum axis[3] = {motion->vel[0]/speed, motion->vel[1]/speed, motion->vel[2]/speed};
    mjtNum qrot[4], quat[4];
    mju_axisAngle2Quat(qrot, axis, speed*t);
    mju_mulQuat(quat, qrot, ctx->quat0);
    mju_quat2Mat(obj->mat, quat);
  }
}


// upper bound on the speed of any support point of the object along unit direction n;
// flex support points move exactly with their vertex (the radius offset has constant
// magnitude along the query direction), rigid surface points are bounded by |w|*rbound
static mjtNum toiSpeedAlong(const mjTOIMotion* motion, const mjtNum n[3]) {
  if (motion->nvert > 0) {
    mjtNum best = mju_dot3(motion->vertvel, n);
    for (int i=1; i < motion->nvert; i++) {
      mjtNum dot = mju_dot3(motion->vertvel + 3*i, n);
      if (dot > best) {
        best = dot;
      }
    }
    return best;
  }
  return mju_dot3(motion->vel+3, n) + mju_norm3(motion->vel)*motion->rbound;
}


// upper bound on the relative speed of any pair of support points of the two objects
static mjtNum toiMaxRelSpeed(const mjTOIMotion* motion1, const mjTOIMotion* motion2) {
  int n1 = motion1->nvert ? motion1->nvert : 1;
  int n2 = motion2->nvert ? motion2->nvert : 1;
  mjtNum vmax = 0;
  for (int i=0; i < n1; i++) {
    const mjtNum* v1 = motion1->nvert ? motion1->vertvel + 3*i : motion1->vel+3;
    for (int j=0; j < n2; j++) {
      const mjtNum* v2 = motion2->nvert ? motion2->vertvel + 3*j : motion2->vel+3;
      mjtNum vrel[3];
      mju_sub3(vrel, v1, v2);
      mjtNum speed = mju_norm3(vrel);
      if (speed > vmax) {
        vmax = speed;
      }
    }
  }

  // rotational slack for rigid objects
  if (!motion1->nvert) {
    vmax += mju_norm3(motion1->vel)*motion1->rbound;
  }
  if (!motion2->nvert) {
    vmax += mju_norm3(motion2->vel)*motion2->rbound;
  }
  return vmax;
}


//---------------------------------- time of impact -----------------------------------------------

// conservative-advancement core, see header for semantics
mjtNum mjc_toi(const mjCCDConfig* config, mjCCDStatus* status,
               mjCCDObj* obj1, const mjTOIMotion* motion1,
               mjCCDObj* obj2, const mjTOIMotion* motion2,
               mjtNum horizon, mjtNum tolerance) {
  mjTOIContext ctx1, ctx2;
  toiInit(&ctx1, obj1, motion1);
  toiInit(&ctx2, obj2, motion2);

  // direction-independent bound on total closing speed
  mjtNum vmax = toiMaxRelSpeed(motion1, motion2);

  mjtNum t = 0;
  for (int iter=0; iter < mjMAXTOIITER; iter++) {
    toiAdvance(&ctx1, obj1, motion1, t);
    toiAdvance(&ctx2, obj2, motion2, t);

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
    mjtNum n[3], n_neg[3];
    mju_sub3(n, status->x2, status->x1);
    mju_scl3(n, n, 1/dist);
    mju_scl3(n_neg, n, -1);

    // not approaching along the closing direction
    mjtNum vbound = toiSpeedAlong(motion1, n) + toiSpeedAlong(motion2, n_neg);
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

  // rigid motion: world-frame velocities at the geom centers
  mjTOIMotion motion1 = {0}, motion2 = {0};
  mj_objectVelocity(m, d, mjOBJ_GEOM, geom1, motion1.vel, /*flg_local=*/0);
  mj_objectVelocity(m, d, mjOBJ_GEOM, geom2, motion2.vel, /*flg_local=*/0);
  motion1.rbound = m->geom_rbound[geom1];
  motion2.rbound = m->geom_rbound[geom2];

  mjtNum toi = mjc_toi(&config, &status, &obj1, &motion1, &obj2, &motion2,
                       horizon, m->opt.ccd_tolerance);

  // witness points at the impact poses
  if (toi >= 0 && fromto && status.nx > 0) {
    mju_copy3(fromto, status.x1);
    mju_copy3(fromto+3, status.x2);
  }

  mj_freeStack(d);
  return toi;
}


// fill a flex element motion descriptor: vertex positions and per-vertex linear velocities
static void flexElemMotion(const mjModel* m, const mjData* d, int f, int e, mjTOIMotion* motion) {
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);

  motion->nvert = dim + 1;
  for (int i=0; i <= dim; i++) {
    int gv = m->flex_vertadr[f] + edata[i];
    const mjtNum* vert = d->flexvert_xpos + 3*gv;
    mju_copy3(motion->vertpos + 3*i, vert);

    // vertex velocity: body velocity at the body frame origin, transported to the vertex
    int bid = m->flex_vertbodyid[gv];
    mjtNum vel6[6], arm[3], rotvel[3];
    mj_objectVelocity(m, d, mjOBJ_XBODY, bid, vel6, /*flg_local=*/0);
    mju_sub3(arm, vert, d->xpos + 3*bid);
    mju_cross(rotvel, vel6, arm);
    mju_add3(motion->vertvel + 3*i, vel6+3, rotvel);
  }
}


// time of impact between two flex elements, see header for semantics
mjtNum mj_flexTOI(const mjModel* m, mjData* d, int flex1, int elem1, int flex2, int elem2,
                  mjtNum horizon, mjtNum fromto[6]) {
  if (fromto) {
    mju_zero(fromto, 6);
  }

  // check inputs
  if (flex1 < 0 || flex1 >= m->nflex || flex2 < 0 || flex2 >= m->nflex) {
    mjERROR("invalid flex id %d or %d", flex1, flex2);
  }
  if (elem1 < 0 || elem1 >= m->flex_elemnum[flex1] ||
      elem2 < 0 || elem2 >= m->flex_elemnum[flex2]) {
    mjERROR("invalid element id %d or %d", elem1, elem2);
  }
  if (horizon < 0) {
    mjERROR("negative horizon %g", horizon);
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
  mjc_initCCDObj(&obj1, m, d, -1, 0);
  mjc_initCCDObj(&obj2, m, d, -1, 0);
  obj1.flex = flex1;
  obj1.elem = elem1;
  obj2.flex = flex2;
  obj2.elem = elem2;

  mjTOIMotion motion1 = {0}, motion2 = {0};
  flexElemMotion(m, d, flex1, elem1, &motion1);
  flexElemMotion(m, d, flex2, elem2, &motion2);

  mjtNum toi = mjc_toi(&config, &status, &obj1, &motion1, &obj2, &motion2,
                       horizon, m->opt.ccd_tolerance);

  // witness points at the impact positions
  if (toi >= 0 && fromto && status.nx > 0) {
    mju_copy3(fromto, status.x1);
    mju_copy3(fromto+3, status.x2);
  }

  mj_freeStack(d);
  return toi;
}
