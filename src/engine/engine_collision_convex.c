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

#include "engine/engine_collision_convex.h"

#include <float.h>
#include <stddef.h>

#include <ccd/ccd.h>
#include <ccd/vec3.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_gjk.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_io.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


// allocate callback for EPA in nativeccd
static void* ccd_allocate(void* data, size_t nbytes) {
  mj_markStack((mjData*)data);
  return mj_stackAllocByte((mjData*)data, nbytes, sizeof(mjtNum));
}

// free callback for EPA in nativeccd
static void ccd_free(void* data, void* buffer) {
  mj_freeStack((mjData*)data);
}

// call libccd or nativeccd to recover penetration info
static int mjc_penetration(const mjModel* m, mjCCDObj* obj1, mjCCDObj* obj2,
                           const ccd_t* ccd, ccd_real_t* depth, ccd_vec3_t* dir, ccd_vec3_t* pos) {
  // fallback to MPR
  if (mjDISABLED(mjDSBL_NATIVECCD)) {
    return ccdMPRPenetration(obj1, obj2, ccd, depth, dir, pos);
  }

  mjCCDConfig config;
  mjCCDStatus status;

  // set config
  config.max_iterations = ccd->max_iterations,
  config.tolerance = ccd->mpr_tolerance,
  config.max_contacts = 1;
  config.dist_cutoff = 0;  // no geom distances needed
  config.context = (void*)obj1->data;
  config.alloc = ccd_allocate;
  config.free = ccd_free;

  mjtNum dist = mjc_ccd(&config, &status, obj1, obj2);
  if (dist < 0) {
    if (depth) *depth = -dist;
    if (dir) {
      mju_sub3(dir->v, status.x1, status.x2);
      mju_normalize3(dir->v);
    }
    if (pos) {
      pos->v[0] = 0.5 * (status.x1[0] + status.x2[0]);
      pos->v[1] = 0.5 * (status.x1[1] + status.x2[1]);
      pos->v[2] = 0.5 * (status.x1[2] + status.x2[2]);
    }
    return 0;
  }
  if (depth) *depth = 0;
  if (dir) mju_zero3(dir->v);
  if (pos) mju_zero3(dir->v);
  return 1;
}



// ccd center function
void mjccd_center(const void *obj, ccd_vec3_t *center) {
  mjc_center(center->v, (const mjCCDObj*) obj);
}

// center function for convex collision algorithms
void mjc_center(mjtNum res[3], const mjCCDObj *obj) {
  int g = obj->geom;
  int f = obj->flex;
  int e = obj->elem;
  int v = obj->vert;

  // return geom position
  if (g >= 0) {
    mju_copy3(res, obj->data->geom_xpos + 3*g);
  }

  // return flex element position
  else if (e >= 0) {
    mju_copy3(res, obj->data->flexelem_aabb + 6*(obj->model->flex_elemadr[f]+e));
  }

  // return flex vertex position
  else {
    mju_copy3(res, obj->data->flexvert_xpos + 3*(obj->model->flex_vertadr[f]+v));
  }
}



// prism center function
static void mjc_prism_center(mjtNum res[3], const mjCCDObj* obj) {
  // compute mean
  mju_zero3(res);
  for (int i=0; i < 6; i++) {
    mju_addTo3(res, obj->prism[i]);
  }
  mju_scl3(res, res, 1.0/6.0);
}



// ccd prism center function
static void mjccd_prism_center(const void *obj, ccd_vec3_t *center) {
  mjc_prism_center(center->v, (const mjCCDObj*) obj);
}

// ------------------------------------ Support functions -----------------------------------------

// transform a vector from global to local frame
static inline void mulMatTVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum dir[3]) {
  // perform matT * dir
  res[0] = mat[0]*dir[0] + mat[3]*dir[1] + mat[6]*dir[2];
  res[1] = mat[1]*dir[0] + mat[4]*dir[1] + mat[7]*dir[2];
  res[2] = mat[2]*dir[0] + mat[5]*dir[1] + mat[8]*dir[2];
}



// transform a vector from local to global frame
static inline void localToGlobal(mjtNum res[3], const mjtNum mat[9], const mjtNum dir[3],
                                 const mjtNum pos[3]) {
  // perform mat * dir + pos
  res[0] = mat[0]*dir[0] + mat[1]*dir[1] + mat[2]*dir[2];
  res[1] = mat[3]*dir[0] + mat[4]*dir[1] + mat[5]*dir[2];
  res[2] = mat[6]*dir[0] + mat[7]*dir[1] + mat[8]*dir[2];
  res[0] += pos[0];
  res[1] += pos[1];
  res[2] += pos[2];
}



// point support function
void mjc_pointSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjtNum* pos = obj->data->geom_xpos + 3*obj->geom;
  res[0] = pos[0];
  res[1] = pos[1];
  res[2] = pos[2];
}



// sphere support function
static void mjc_sphereSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // sphere data
  const mjtNum* pos = d->geom_xpos + 3*obj->geom;
  mjtNum radius = m->geom_size[3*obj->geom];

  res[0] = radius*dir[0] + pos[0];
  res[1] = radius*dir[1] + pos[1];
  res[2] = radius*dir[2] + pos[2];
}



// line support function (capsule)
void mjc_lineSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // capsule data
  int i = 3*obj->geom;
  const mjtNum* mat = d->geom_xmat + 3*i;
  const mjtNum* pos = d->geom_xpos + i;
  mjtNum length = m->geom_size[i+1];

  // rotate dir to geom local frame
  mjtNum local_dir[3], tmp[3];
  mulMatTVec3(local_dir, mat, dir);

  tmp[0] = 0;
  tmp[1] = 0;
  tmp[2] = (local_dir[2] >= 0 ? length : -length);

  // transform result to global frame
  localToGlobal(res, mat, tmp, pos);
}



// capsule support function
static void mjc_capsuleSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // capsule data
  int i = 3*obj->geom;
  const mjtNum* mat = d->geom_xmat + 3*i;
  const mjtNum* pos = d->geom_xpos + i;
  mjtNum radius = m->geom_size[i];
  mjtNum length = m->geom_size[i+1];

  // rotate dir to geom local frame
  mjtNum local_dir[3], tmp[3];
  mulMatTVec3(local_dir, mat, dir);

  // start with sphere
  tmp[0] = local_dir[0] * radius;
  tmp[1] = local_dir[1] * radius;
  tmp[2] = local_dir[2] * radius;

  // add cylinder contribution
  tmp[2] += (local_dir[2] >= 0 ? length : -length);

  // transform result to global frame
  localToGlobal(res, mat, tmp, pos);
}



// ellipsoid support function
static void mjc_ellipsoidSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // ellipsoid data
  int i = 3*obj->geom;
  const mjtNum* mat = d->geom_xmat + 3*i;
  const mjtNum* pos = d->geom_xpos + i;
  const mjtNum* size = m->geom_size + i;

  // rotate dir to geom local frame
  mjtNum local_dir[3], tmp[3];
  mulMatTVec3(local_dir, mat, dir);

  // find support point on unit sphere: scale dir by ellipsoid sizes
  tmp[0] = local_dir[0] * size[0];
  tmp[1] = local_dir[1] * size[1];
  tmp[2] = local_dir[2] * size[2];

  mjtNum norm = mju_sqrt(tmp[0]*tmp[0] + tmp[1]*tmp[1] + tmp[2]*tmp[2]);

  // try normalizing and transform to ellipsoid
  if (norm < mjMINVAL) {
    tmp[0] = size[0];
    tmp[1] = 0;
    tmp[2] = 0;
  } else {
    mjtNum norm_inv = 1/norm;
    tmp[0] *= norm_inv * size[0];
    tmp[1] *= norm_inv * size[1];
    tmp[2] *= norm_inv * size[2];
  }

  // transform result to global frame
  localToGlobal(res, mat, tmp, pos);
}



// cylinder support function
static void mjc_cylinderSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // cylinder data
  int i = 3*obj->geom;
  const mjtNum* mat = d->geom_xmat + 3*i;
  const mjtNum* pos = d->geom_xpos + i;
  const mjtNum* size = m->geom_size + i;

  // rotate dir to geom local frame
  mjtNum local_dir[3], tmp[3];
  mulMatTVec3(local_dir, mat, dir);

  mjtNum n = local_dir[0]*local_dir[0] + local_dir[1]*local_dir[1];
  if (n > mjMINVAL*mjMINVAL) {
    n = size[0] / mju_sqrt(n);
    tmp[0] = local_dir[0] * n;
    tmp[1] = local_dir[1] * n;
  } else {
    tmp[0] = tmp[1] = 0;
  }

  // set result in Z direction
  tmp[2] = mju_sign(local_dir[2]) * size[1];

  // transform result to global frame
  localToGlobal(res, mat, tmp, pos);
}



// box support function
static void mjc_boxSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // box data
  int i = 3*obj->geom;
  const mjtNum* mat = d->geom_xmat + 3*i;
  const mjtNum* pos = d->geom_xpos + i;
  const mjtNum* size = m->geom_size + i;

  // rotate dir to geom local frame
  mjtNum local_dir[3], tmp[3];
  mulMatTVec3(local_dir, mat, dir);

  // find support point in local frame
  tmp[0] = (local_dir[0] >= 0 ? 1 : -1) * size[0];
  tmp[1] = (local_dir[1] >= 0 ? 1 : -1) * size[1];
  tmp[2] = (local_dir[2] >= 0 ? 1 : -1) * size[2];

  // mark the index of the corner of the box for fast lookup
  obj->vertindex = 0;
  if (tmp[0] > 0) obj->vertindex |= 1;
  if (tmp[1] > 0) obj->vertindex |= 2;
  if (tmp[2] > 0) obj->vertindex |= 4;

  // transform support point to global frame
  localToGlobal(res, mat, tmp, pos);
}



// dot product between mjtNum and float
static inline mjtNum dot3f(const mjtNum a[3], const float b[3]) {
  return a[0]*(mjtNum)b[0] + a[1]*(mjtNum)b[1] + a[2]*(mjtNum)b[2];
}



// mesh support function via exhaustive search
static void mjc_meshSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // mesh data
  int g = obj->geom;
  const mjtNum* mat = d->geom_xmat+9*g;
  const mjtNum* pos = d->geom_xpos+3*g;
  float* verts = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
  int nverts = m->mesh_vertnum[m->geom_dataid[g]];

  mjtNum local_dir[3];
  mulMatTVec3(local_dir, mat, dir);

  mjtNum max = -FLT_MAX;
  int imax = 0;

  // used cached results from previous search
  if (obj->vertindex >= 0) {
    imax = obj->vertindex;
    max = dot3f(local_dir, verts + 3*imax);
  }

  // search all vertices, find maximum dot product
  for (int i=0; i < nverts; i++) {
    mjtNum vdot = dot3f(local_dir, verts + 3*i);

    // update max
    if (vdot > max) {
      max = vdot;
      imax = i;
    }
  }

  // record vertex index of maximum
  obj->vertindex = imax;

  local_dir[0] = (mjtNum)verts[3*imax + 0];
  local_dir[1] = (mjtNum)verts[3*imax + 1];
  local_dir[2] = (mjtNum)verts[3*imax + 2];

  // transform result to global frame
  localToGlobal(res, mat, local_dir, pos);
}



// mesh support function via hill climbing
static void mjc_hillclimbSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;

  // get mesh info
  int g = obj->geom;
  int graphadr = m->mesh_graphadr[m->geom_dataid[g]];
  int numvert = m->mesh_graph[graphadr];
  int* vert_edgeadr = m->mesh_graph + graphadr + 2;
  int* vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
  int* edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;
  float* verts = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
  const mjtNum* mat = d->geom_xmat + 9*g;
  const mjtNum* pos = d->geom_xpos+3*g;

  // rotate dir to geom local frame
  mjtNum local_dir[3];
  mulMatTVec3(local_dir, mat, dir);

  // hillclimb until no change
  mjtNum max = -FLT_MAX;
  int prev = -1, imax = obj->meshindex < 0 ? 0 : obj->meshindex;
  do {
    prev = imax;
    for (int i = vert_edgeadr[imax]; edge_localid[i] >= 0; i++) {
      int idx = 3*vert_globalid[edge_localid[i]];
      mjtNum vdot = dot3f(local_dir, verts + idx);
      if (vdot > max) {
        max = vdot;
        imax = edge_localid[i];  // update maximum vertex index
      }
    }
  } while (imax != prev);

  // record vertex index of maximum (local id)
  obj->meshindex = imax;

  // get resulting support vertex
  imax = 3*vert_globalid[imax];
  obj->vertindex = imax / 3;
  local_dir[0] = (mjtNum)verts[imax + 0];
  local_dir[1] = (mjtNum)verts[imax + 1];
  local_dir[2] = (mjtNum)verts[imax + 2];

  // transform result to global frame
  localToGlobal(res, mat, local_dir, pos);
}



// prism support function
static void mjc_prism_support(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  int istart, ibest;
  mjtNum best, tmp;

  // find best vertex in halfspace determined by dir.z
  istart = dir[2] < 0 ? 0 : 3;
  ibest = istart;
  best = mju_dot3(obj->prism[istart], dir);
  for (int i=istart+1; i < istart+3; i++) {
    if ((tmp = mju_dot3(obj->prism[i], dir)) > best) {
      ibest = i;
      best = tmp;
    }
  }

  // copy best point
  mju_copy3(res, obj->prism[ibest]);
}



// flex support function
static void mjc_flexSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]) {
  const mjModel* m = obj->model;
  const mjData* d = obj->data;
  int f = obj->flex;
  int dim = m->flex_dim[f];

  // flex element
  if (obj->elem >= 0) {
    int e = obj->elem;
    const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
    const mjtNum* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

    // find element vertex with largest projection along dir
    mju_copy3(res, vert+3*edata[0]);
    mjtNum best = mju_dot3(res, dir);
    for (int i=1; i <= dim; i++) {
      mjtNum dot = mju_dot3(vert+3*edata[i], dir);

      // better vertex found: assign
      if (dot > best) {
        best = dot;
        mju_copy3(res, vert+3*edata[i]);
      }
    }

    // add radius and margin/2
    mju_addToScl3(res, dir, m->flex_radius[f] + 0.5*obj->margin);
    return;
  }

  // flex vertex
  else {
    const mjtNum* vert = d->flexvert_xpos + 3*(m->flex_vertadr[f] + obj->vert);
    mju_addScl3(res, vert, dir, m->flex_radius[f] + 0.5*obj->margin);
    return;
  }
}



// libccd support function
void mjccd_support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *vec) {
  mjCCDObj *obj = (mjCCDObj *)_obj;
  mjtNum *res = vec->v;
  const mjtNum *dir = _dir->v;
  const mjModel* m = obj->model;
  const mjData* d = obj->data;
  int g = obj->geom;

  if (g < 0) {
    int f = obj->flex;
    int dim = m->flex_dim[f];

    // flex element
    if (obj->elem >= 0) {
      int e = obj->elem;
      const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
      const mjtNum* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

      // find element vertex with largest projection along dir
      mju_copy3(res, vert+3*edata[0]);
      mjtNum best = mju_dot3(res, dir);
      for (int i=1; i <= dim; i++) {
        mjtNum dot = mju_dot3(vert+3*edata[i], dir);

        // better vertex found: assign
        if (dot > best) {
          best = dot;
          mju_copy3(res, vert+3*edata[i]);
        }
      }

      // add radius and margin/2
      mju_addToScl3(res, dir, m->flex_radius[f] + 0.5*obj->margin);
      return;
    }

    // flex vertex
    else {
      const mjtNum* vert = d->flexvert_xpos + 3*(m->flex_vertadr[f] + obj->vert);
      mju_addScl3(res, vert, dir, m->flex_radius[f] + 0.5*obj->margin);
      return;
    }
  }

  float* vertdata;
  int ibest, graphadr, numvert, change, locid;
  int *vert_edgeadr, *vert_globalid, *edge_localid;
  mjtNum tmp, vdot;

  const mjtNum* size = m->geom_size+3*g;  // geom sizes
  mjtNum local_dir[3];                    // direction in geom local frame

  // rotate dir to geom local frame
  mju_mulMatTVec3(local_dir, d->geom_xmat+9*g, dir);

  // compute result according to geom type
  switch ((mjtGeom) obj->geom_type) {
  case mjGEOM_SPHERE:
    mju_scl3(res, local_dir, size[0]);
    break;

  case mjGEOM_CAPSULE:
    // start with sphere
    mju_scl3(res, local_dir, size[0]);

    // add cylinder contribution
    res[2] += mju_sign(local_dir[2]) * size[1];
    break;

  case mjGEOM_ELLIPSOID:
    // find support point on unit sphere: scale dir by ellipsoid sizes and renormalize
    for (int i=0; i < 3; i++) {
      res[i] = local_dir[i] * size[i];
    }
    mju_normalize3(res);

    // transform to ellipsoid
    for (int i=0; i < 3; i++) {
      res[i] *= size[i];
    }
    break;

  case mjGEOM_CYLINDER:
    // set result in XY plane: support on circle
    tmp = mju_sqrt(local_dir[0]*local_dir[0] + local_dir[1]*local_dir[1]);
    if (tmp > mjMINVAL) {
      res[0] = local_dir[0]/tmp*size[0];
      res[1] = local_dir[1]/tmp*size[0];
    } else {
      res[0] = res[1] = 0;
    }

    // set result in Z direction
    res[2] = mju_sign(local_dir[2]) * size[1];
    break;

  case mjGEOM_BOX:
    for (int i=0; i < 3; i++) {
      res[i] = mju_sign(local_dir[i]) * size[i];
    }
    break;

  case mjGEOM_MESH:
  case mjGEOM_SDF:
    // init search
    vertdata = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
    tmp = -1E+10;
    ibest = -1;

    // no graph data: exhaustive search
    if (m->mesh_graphadr[m->geom_dataid[g]] < 0) {
      // search all vertices, find best
      for (int i=0; i < m->mesh_vertnum[m->geom_dataid[g]]; i++) {
        // vdot = dot(vertex, dir)
        vdot = local_dir[0] * (mjtNum)vertdata[3*i] +
               local_dir[1] * (mjtNum)vertdata[3*i+1] +
               local_dir[2] * (mjtNum)vertdata[3*i+2];

        // update best
        if (vdot > tmp) {
          tmp = vdot;
          ibest = i;
        }
      }

      // record best vertex index, in globalid format
      obj->meshindex = ibest;
    }

    // hill-climb using graph data
    else {
      // get info
      graphadr = m->mesh_graphadr[m->geom_dataid[g]];
      numvert = m->mesh_graph[graphadr];
      vert_edgeadr = m->mesh_graph + graphadr + 2;
      vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
      edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;

      // init with first vertex in convex hull or warmstart
      ibest = obj->meshindex < 0 ? 0 : obj->meshindex;
      tmp = local_dir[0] * (mjtNum)vertdata[3*vert_globalid[ibest]+0] +
            local_dir[1] * (mjtNum)vertdata[3*vert_globalid[ibest]+1] +
            local_dir[2] * (mjtNum)vertdata[3*vert_globalid[ibest]+2];

      // hill-climb until no change
      change = 1;
      while (change) {
        // look for improvement in ibest neighborhood
        change = 0;
        int i = vert_edgeadr[ibest];
        while ((locid=edge_localid[i]) >= 0) {
          // vdot = dot(vertex, local_dir)
          vdot = local_dir[0] * (mjtNum)vertdata[3*vert_globalid[locid]] +
                 local_dir[1] * (mjtNum)vertdata[3*vert_globalid[locid]+1] +
                 local_dir[2] * (mjtNum)vertdata[3*vert_globalid[locid]+2];

          // update best
          if (vdot > tmp) {
            tmp = vdot;
            ibest = locid;
            change = 1;
          }

          // advance to next edge
          i++;
        }
      }

      // record best vertex index, in locid format
      obj->meshindex = ibest;

      // map best index to globalid
      ibest = vert_globalid[ibest];
    }

    // sanity check, SHOULD NOT OCCUR
    if (ibest < 0) {
      mju_warning("mesh_support could not find support vertex");
      mju_zero3(res);
    }

    // copy best vertex
    else {
      for (int i=0; i < 3; i++) {
        res[i] = (mjtNum)vertdata[3*ibest + i];
      }
    }
    break;

  default:
    mjERROR("ccd support function is undefined for geom type %d", m->geom_type[g]);
  }

  // add local_dir*margin/2 to result
  for (int i=0; i < 3; i++) {
    res[i] += local_dir[i] * obj->margin/2;
  }

  // rotate result to global frame
  mju_mulMatVec3(res, d->geom_xmat+9*g, res);

  // add geom position
  mju_addTo3(res, d->geom_xpos+3*g);
}



// libccd prism support function
static void mjccd_prism_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  mjc_prism_support(vec->v, (mjCCDObj*) obj, dir->v);
}

// ------------------------------------------------------------------------------------------------

// initialize a CCD object
void mjc_initCCDObj(mjCCDObj* obj, const mjModel* m, const mjData* d, int g, mjtNum margin) {
  obj->model = m;
  obj->data = d;
  obj->geom = g;
  obj->margin = margin;
  obj->center = mjc_center;
  obj->vertindex = -1;
  obj->meshindex = -1;
  obj->flex = -1;
  obj->elem = -1;
  obj->vert = -1;
  mju_zero4(obj->rotate);
  obj->rotate[0] = 1;
  if (g >= 0) {
    obj->geom_type = m->geom_type[g];
    switch ((mjtGeom) obj->geom_type) {
    case mjGEOM_ELLIPSOID:
      obj->support = mjc_ellipsoidSupport;
      break;
    case mjGEOM_MESH:
    case mjGEOM_SDF:
      if (m->mesh_graphadr[m->geom_dataid[g]] < 0 ||
          m->mesh_vertnum[m->geom_dataid[g]] < mjMESH_HILLCLIMB_MIN) {
        obj->support = mjc_meshSupport;
      } else {
        obj->support = mjc_hillclimbSupport;
      }
      break;
    case mjGEOM_SPHERE:
      obj->support = mjc_sphereSupport;
      break;
    case mjGEOM_CAPSULE:
      obj->support = mjc_capsuleSupport;
      break;
    case mjGEOM_CYLINDER:
      obj->support = mjc_cylinderSupport;
      break;
    case mjGEOM_BOX:
      obj->support = mjc_boxSupport;
      break;
    case mjGEOM_HFIELD:
      obj->center = mjc_prism_center;
      obj->support = mjc_prism_support;
      break;
    default:
      obj->support = NULL;
      break;
    }
  } else {
    obj->geom_type = mjGEOM_FLEX;
    obj->support = mjc_flexSupport;
  }
}



// set flex data for CCD object
static void mjc_setCCDObjFlex(mjCCDObj* obj, int flex, int elem, int vert) {
  obj->flex = flex;
  obj->elem = elem;
  obj->vert = vert;
}



// initialize CCD structure
static void mjc_initCCD(ccd_t* ccd, const mjModel* m) {
  CCD_INIT(ccd);
  ccd->mpr_tolerance = m->opt.ccd_tolerance;
  ccd->epa_tolerance = m->opt.ccd_tolerance;  // use MPR tolerance for EPA
  ccd->max_iterations = m->opt.ccd_iterations;
}



// find convex-convex collision
static int mjc_CCDIteration(const mjModel* m, const mjData* d, mjCCDObj* obj1, mjCCDObj* obj2,
                            mjContact* con, int max_contacts, mjtNum margin) {
  if (!mjDISABLED(mjDSBL_NATIVECCD)) {
    mjCCDConfig config;
    mjCCDStatus status;

    // set config
    config.max_iterations = m->opt.ccd_iterations;
    config.tolerance = m->opt.ccd_tolerance;
    config.max_contacts = max_contacts;
    config.dist_cutoff = 0;  // no geom distances needed
    config.context = (void*)d;
    config.alloc = ccd_allocate;
    config.free = ccd_free;

    mjtNum dist = mjc_ccd(&config, &status, obj1, obj2);
    if (dist < 0) {
      for (int i = 0; i < status.nx; i++) {
        mjContact* c = con++;
        c->dist = margin + dist;
        mju_sub3(c->frame, status.x1 + 3*i, status.x2 + 3*i);
        mju_normalize3(c->frame);
        c->pos[0] = 0.5 * (status.x1[0 + 3*i] + status.x2[0 + 3*i]);
        c->pos[1] = 0.5 * (status.x1[1 + 3*i] + status.x2[1 + 3*i]);
        c->pos[2] = 0.5 * (status.x1[2 + 3*i] + status.x2[2 + 3*i]);
        mju_zero3(c->frame+3);
      }
      return status.nx;
    }
    return 0;
  }

  // init libccd structure
  ccd_t ccd;
  mjc_initCCD(&ccd, m);
  ccd.first_dir = ccdFirstDirDefault;
  ccd.center1 = mjccd_center;
  ccd.center2 = mjccd_center;
  ccd.support1 = mjccd_support;
  ccd.support2 = mjccd_support;

  ccd_vec3_t dir, pos;
  ccd_real_t depth;

  // call MPR from libccd
  if (ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos) == 0) {
    // contact is found but normal is undefined
    if (ccdVec3Eq(&dir, ccd_vec3_origin)) {
      return 0;
    }

    // fill in contact data
    con->dist = margin-depth;
    mju_copy3(con->frame, dir.v);
    mju_copy3(con->pos, pos.v);
    mju_zero3(con->frame+3);

    // both geoms: fix contact frame normal
    if (obj1->geom >= 0 && obj2->geom >= 0) {
      mjc_fixNormal(m, d, con, obj1->geom, obj2->geom);
    }

    return 1;
  }
  return 0;
}



// compare new contact to previous contacts, return 1 if it is far from all of them
static int mjc_isDistinctContact(mjContact* con, int ncon, mjtNum tolerance) {
  for (int i=0; i < ncon-1; i++) {
    if (mju_dist3(con[i].pos, con[ncon - 1].pos) <= tolerance) {
      return 0;
    }
  }
  return 1;
}



// in-place rotation of spatial frame around given point of origin
static void mju_rotateFrame(const mjtNum origin[3], const mjtNum rot[9],
                            mjtNum xmat[9], mjtNum xpos[3]) {
  mjtNum mat[9], vec[3], rel[3];

  // rotate frame: xmat = rot*xmat
  mju_mulMatMat3(mat, rot, xmat);
  mju_copy(xmat, mat, 9);

  // vector to rotation origin: rel = origin - xpos
  mju_sub3(rel, origin, xpos);

  // displacement of origin due to rotation: vec = rot*rel - rel
  mju_mulMatVec3(vec, rot, rel);
  mju_subFrom3(vec, rel);

  // correct xpos by subtracting displacement: xpos = xpos - vec
  mju_subFrom3(xpos, vec);
}



// return number of contacts supported by a single pass of narrowphase
static int maxContacts(const mjCCDObj* obj1, const mjCCDObj* obj2) {
  const mjModel* m = obj1->model;

  // single pass not supported for margins
  if (obj1->margin > 0 || obj2->margin > 0) {
    return 1;
  }

  // can return 8 contacts for box-box collision in one pass
  int type1 = m->geom_type[obj1->geom];
  int type2 = m->geom_type[obj2->geom];
  if (type1 == mjGEOM_BOX && type2 == mjGEOM_BOX) {
    return 8;
  }

  // reduce mesh collisions to 4 contacts max
  if (type1 == mjGEOM_BOX || type1 == mjGEOM_MESH) {
    if (type2 == mjGEOM_BOX || type2 == mjGEOM_MESH) {
      return mjENABLED(mjENBL_MULTICCD) ? 4 : 1;
    }
  }

  // not supported for other geom types
  return 1;
}



// multi-point convex-convex collision, using libccd
int mjc_Convex(const mjModel* m, const mjData* d,
               mjContact* con, int g1, int g2, mjtNum margin) {
  // init ccd objects
  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, m, d, g1, margin);
  mjc_initCCDObj(&obj2, m, d, g2, margin);
  int max_contacts = maxContacts(&obj1, &obj2);

  // find initial contact
  int ncon = mjc_CCDIteration(m, d, &obj1, &obj2, con, max_contacts, margin);


  // no additional contacts needed
  if (!mjDISABLED(mjDSBL_NATIVECCD) && max_contacts > 1) {
    return ncon;
  }

  // look for additional contacts
  if (ncon == 1 && mjENABLED(mjENBL_MULTICCD)  // TODO(tassa) leave as bitflag or make geom attribute (?)
      && m->geom_type[g1] != mjGEOM_ELLIPSOID && m->geom_type[g1] != mjGEOM_SPHERE
      && m->geom_type[g2] != mjGEOM_ELLIPSOID && m->geom_type[g2] != mjGEOM_SPHERE) {
    // multiCCD parameters
    const mjtNum relative_tolerance = 1e-3;
    const mjtNum perturbation_angle = 1e-3;

    // save positions and orientations of g1 and g2
    mjtNum xpos1[3], xmat1[9], xpos2[3], xmat2[9];
    mju_copy3(xpos1, d->geom_xpos+3*g1);
    mju_copy(xmat1, d->geom_xmat+9*g1, 9);
    mju_copy3(xpos2, d->geom_xpos+3*g2);
    mju_copy(xmat2, d->geom_xmat+9*g2, 9);

    // complete frame of initial contact
    mjtNum frame[9];
    mju_copy(frame, con[0].frame, 9);
    mju_makeFrame(frame);

    // tolerance for determining if newly found contacts are distinct
    const mjtNum tolerance = relative_tolerance * mju_min(m->geom_rbound[g1], m->geom_rbound[g2]);

    // axes and rotation angles for perturbation test
    mjtNum* axes[2] = {frame+3, frame+6};
    mjtNum angles[2] = {-perturbation_angle, perturbation_angle};

    // rotate both geoms, search for new contacts
    for (int axis_id = 0; axis_id < 2; ++axis_id) {
      for (int angle_id = 0; angle_id < 2; ++angle_id) {
        mjtNum* axis = axes[axis_id];
        mjtNum angle = angles[angle_id];

        // make rotation matrix rot
        mjtNum quat[4], rot[9];
        mju_axisAngle2Quat(quat, axis, angle);
        mju_quat2Mat(rot, quat);

        // rotate g1 around initial contact point
        mju_rotateFrame(con[0].pos, rot, d->geom_xmat+9*g1, d->geom_xpos+3*g1);

        // inversely rotate g2 around initial contact point
        mjtNum invrot[9];
        mju_transpose(invrot, rot, 3, 3);
        mju_rotateFrame(con[0].pos, invrot, d->geom_xmat+9*g2, d->geom_xpos+3*g2);

        // search for new contact
        int new_contact = mjc_CCDIteration(m, d, &obj1, &obj2, con+ncon, 1, margin);

        // check new contact
        if (new_contact && mjc_isDistinctContact(con, ncon + 1, tolerance)) {
          // set penetration of new point to equal that of initial point
          con[ncon].dist = con[0].dist;
          // add new point
          ncon += 1;
        }

        // reset positions and orientations of g1 and g2
        mju_copy3(d->geom_xpos+3*g1, xpos1);
        mju_copy(d->geom_xmat+9*g1, xmat1, 9);
        mju_copy3(d->geom_xpos+3*g2, xpos2);
        mju_copy(d->geom_xmat+9*g2, xmat2, 9);
      }
    }
  }
  return ncon;
}



// parameters for plane-mesh extra contacts
const int maxplanemesh = 3;
const mjtNum tolplanemesh = 0.3;

// add one plane-mesh contact
static int addplanemesh(mjContact* con, const float vertex[3],
                        const mjtNum pos1[3], const mjtNum normal1[3],
                        const mjtNum pos2[3], const mjtNum mat2[9],
                        const mjtNum first[3], mjtNum rbound) {
  // compute point in global coordinates
  mjtNum pnt[3], v[3] = {vertex[0], vertex[1], vertex[2]};
  mju_mulMatVec3(pnt, mat2, v);
  mju_addTo3(pnt, pos2);

  // skip if too close to first contact
  if (mju_dist3(pnt, first) < tolplanemesh*rbound) {
    return 0;
  }

  // pnt-pos difference vector
  mjtNum dif[3];
  mju_sub3(dif, pnt, pos1);

  // set distance
  con->dist = mju_dot3(normal1, dif);

  // set position
  mju_copy3(con->pos, pnt);
  mju_addToScl3(con->pos, normal1, -0.5*con->dist);

  // set frame
  mju_copy3(con->frame, normal1);
  mju_zero3(con->frame+3);

  return 1;
}



// plane-convex collision, using libccd
int mjc_PlaneConvex(const mjModel* m, const mjData* d,
                    mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum dist, dif[3], normal[3] = {mat1[2], mat1[5], mat1[8]};
  ccd_vec3_t dir, vec;
  mjCCDObj obj;
  mjc_initCCDObj(&obj, m, d, g2, 0);
  // get support point in -normal direction
  ccdVec3Set(&dir, -mat1[2], -mat1[5], -mat1[8]);
  mjccd_support(&obj, &dir, &vec);

  // compute normal distance, return if too far
  mju_sub3(dif, vec.v, pos1);
  dist = mju_dot3(normal, dif);
  if (dist > margin) {
    return 0;
  }

  // fill in contact data
  con->dist = dist;
  mju_copy3(con->pos, vec.v);
  mju_addToScl3(con->pos, normal, -0.5*dist);
  mju_copy3(con->frame, normal);
  mju_zero3(con->frame+3);

  //--------------- add all/connected vertices below margin
  float* vertdata;
  int graphadr, numvert, locid;
  int *vert_edgeadr, *vert_globalid, *edge_localid;
  mjtNum vdot;
  int count = 1, g = g2;

  // g is an ellipsoid: no need for further mesh-specific processing
  if (m->geom_dataid[g] == -1) {
    return count;
  }

  // init
  vertdata = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];

  // express dir in geom local frame
  mjtNum locdir[3];
  mju_mulMatTVec3(locdir, d->geom_xmat+9*g, dir.v);

  // inclusion threshold along locdir, relative to geom2 center
  mju_sub3(dif, pos2, pos1);
  mjtNum threshold = mju_dot3(normal, dif) - margin;

  // no graph data: exhaustive search
  if (m->mesh_graphadr[m->geom_dataid[g]] < 0) {
    // search all vertices, find best
    for (int i=0; i < m->mesh_vertnum[m->geom_dataid[g]] && count < maxplanemesh; i++) {
      // vdot = dot(vertex, dir)
      vdot = locdir[0] * (mjtNum)vertdata[3*i] +
             locdir[1] * (mjtNum)vertdata[3*i+1] +
             locdir[2] * (mjtNum)vertdata[3*i+2];

      // detect contact, skip best
      if (vdot > threshold && i != obj.meshindex) {
        count += addplanemesh(con+count, vertdata+3*i,
                              pos1, normal, pos2, mat2,
                              con->pos, m->geom_rbound[g2]);
      }
    }
  }

  // use graph data
  else if (obj.meshindex >= 0) {
    // get info
    graphadr = m->mesh_graphadr[m->geom_dataid[g]];
    numvert = m->mesh_graph[graphadr];
    vert_edgeadr = m->mesh_graph + graphadr + 2;
    vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
    edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;

    // look for contacts in ibest neighborhood
    int i = vert_edgeadr[obj.meshindex];
    while ((locid=edge_localid[i]) >= 0 && count < maxplanemesh) {
      // vdot = dot(vertex, dir)
      vdot = locdir[0] * (mjtNum)vertdata[3*vert_globalid[locid]] +
             locdir[1] * (mjtNum)vertdata[3*vert_globalid[locid]+1] +
             locdir[2] * (mjtNum)vertdata[3*vert_globalid[locid]+2];

      // detect contact
      if (vdot > threshold) {
        count += addplanemesh(con+count, vertdata+3*vert_globalid[locid],
                              pos1, normal, pos2, mat2,
                              con->pos, m->geom_rbound[g2]);
      }

      // advance to next edge
      i++;
    }
  }

  return count;
}



//----------------------------  heightfield collisions ---------------------------------------------

// ccd prism first dir
static void prism_firstdir(const void* o1, const void* o2, ccd_vec3_t *vec) {
  ccdVec3Set(vec, 0, 0, 1);
}


// add vertex to prism, count vertices
static void addVert(int* nvert, mjCCDObj* obj, mjtNum x, mjtNum y, mjtNum z) {
  // move old data
  mju_copy3(obj->prism[0], obj->prism[1]);
  mju_copy3(obj->prism[1], obj->prism[2]);
  mju_copy3(obj->prism[3], obj->prism[4]);
  mju_copy3(obj->prism[4], obj->prism[5]);

  // add new vertex at last position
  obj->prism[2][0] = obj->prism[5][0] = x;
  obj->prism[2][1] = obj->prism[5][1] = y;
  obj->prism[5][2] = z;

  // count
  (*nvert)++;
}



// entry point for heightfield collisions
int mjc_ConvexHField(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO_HFIELD
  mjtNum mat[9], savemat2[9], savepos2[3], pos[3], vec[3], r2, dx, dy;
  mjtNum xmin, xmax, ymin, ymax, zmin, zmax;
  int hid = m->geom_dataid[g1];
  int nrow = m->hfield_nrow[hid];
  int ncol = m->hfield_ncol[hid];
  int dr[2], cnt, rmin, rmax, cmin, cmax;
  const float* data = m->hfield_data + m->hfield_adr[hid];

  // ccd-related
  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, m, d, g1, 0);
  mjc_initCCDObj(&obj2, m, d, g2, 0);

  ccd_vec3_t dirccd, vecccd;
  ccd_real_t depth;
  ccd_t ccd;

  // point size1 to hfield size instead of geom1 size
  size1 = m->hfield_size + 4*hid;

  //------------------------------------- frame alignment, box-sphere test

  // express geom2 pos in heightfield frame
  mju_sub3(vec, pos2, pos1);
  mju_mulMatTVec(pos, mat1, vec, 3, 3);

  // get geom2 rbound
  r2 = m->geom_rbound[g2];

  // box-sphere test: horizontal plane
  for (int i=0; i < 2; i++) {
    if ((size1[i] < pos[i]-r2-margin) || (-size1[i] > pos[i]+r2+margin)) {
      return 0;
    }
  }

  // box-sphere test in: vertical direction
  if (size1[2] < pos[2]-r2-margin) {   // up
    return 0;
  }
  if (-size1[3] > pos[2]+r2+margin) {  // down
    return 0;
  }

  // express geom2 mat in heightfield frame
  mju_mulMatTMat3(mat, mat1, mat2);

  //------------------------------------- AABB computation, box-box test

  // save mat2 and pos2, replace with relative frame
  mju_copy(savemat2, mat2, 9);
  mju_copy3(savepos2, pos2);
  mju_copy(mat2, mat, 9);
  mju_copy3(pos2, pos);

  // get support point in +X
  ccdVec3Set(&dirccd, 1, 0, 0);
  mjccd_support(&obj2, &dirccd, &vecccd);
  xmax = vecccd.v[0];

  // get support point in -X
  ccdVec3Set(&dirccd, -1, 0, 0);
  mjccd_support(&obj2, &dirccd, &vecccd);
  xmin = vecccd.v[0];

  // get support point in +Y
  ccdVec3Set(&dirccd, 0, 1, 0);
  mjccd_support(&obj2, &dirccd, &vecccd);
  ymax = vecccd.v[1];

  // get support point in -Y
  ccdVec3Set(&dirccd, 0, -1, 0);
  mjccd_support(&obj2, &dirccd, &vecccd);
  ymin = vecccd.v[1];

  // get support point in +Z
  ccdVec3Set(&dirccd, 0, 0, 1);
  mjccd_support(&obj2, &dirccd, &vecccd);
  zmax = vecccd.v[2];

  // get support point in -Z
  ccdVec3Set(&dirccd, 0, 0, -1);
  mjccd_support(&obj2, &dirccd, &vecccd);
  zmin = vecccd.v[2];

  // box-box test
  if ((xmin-margin > size1[0]) || (xmax+margin < -size1[0]) ||
      (ymin-margin > size1[1]) || (ymax+margin < -size1[1]) ||
      (zmin-margin > size1[2]) || (zmax+margin < -size1[3])) {
    // restore mat2 and pos2
    mju_copy(mat2, savemat2, 9);
    mju_copy3(pos2, savepos2);

    return 0;
  }

  // compute sub-grid bounds
  cmin = (int) mju_floor((xmin + size1[0]) / (2*size1[0]) * (ncol-1));
  cmax = (int) mju_ceil ((xmax + size1[0]) / (2*size1[0]) * (ncol-1));
  rmin = (int) mju_floor((ymin + size1[1]) / (2*size1[1]) * (nrow-1));
  rmax = (int) mju_ceil ((ymax + size1[1]) / (2*size1[1]) * (nrow-1));
  cmin = mjMAX(0, cmin);
  cmax = mjMIN(ncol-1, cmax);
  rmin = mjMAX(0, rmin);
  rmax = mjMIN(nrow-1, rmax);

  //------------------------------------- collision testing

  // init ccd structure
  mjc_initCCD(&ccd, m);
  ccd.first_dir = prism_firstdir;
  ccd.center1 = mjccd_prism_center;
  ccd.center2 = mjccd_center;
  ccd.support1 = mjccd_prism_support;
  ccd.support2 = mjccd_support;

  // geom margin needed for actual collision test
  obj2.margin = margin;

  // compute real-valued grid step, and triangulation direction
  dx = (2.0*size1[0]) / (ncol-1);
  dy = (2.0*size1[1]) / (nrow-1);
  dr[0] = 1;
  dr[1] = 0;

  // set zbottom value using base size
  obj1.prism[0][2] = obj1.prism[1][2] = obj1.prism[2][2] = -size1[3];

  // process all prisms in sub-grid
  cnt = 0;
  for (int r=rmin; r < rmax; r++) {
    int nvert = 0;
    for (int c=cmin; c <= cmax; c++) {
      for (int i=0; i < 2; i++) {
        // send vertex to prism constructor
        addVert(&nvert, &obj1, dx*c-size1[0], dy*(r+dr[i])-size1[1],
                data[(r+dr[i])*ncol+c]*size1[2]+margin);

        // check for enough vertices
        if (nvert > 2) {
          // prism height test
          if (obj1.prism[3][2] < zmin && obj1.prism[4][2] < zmin
              && obj1.prism[5][2] < zmin) {
            continue;
          }

          // run penetration function, save contact
          if (mjc_penetration(m, &obj1, &obj2, &ccd, &depth, &dirccd, &vecccd) == 0
              && !ccdVec3Eq(&dirccd, ccd_vec3_origin)) {
            // fill in contact data, transform to global coordinates
            con[cnt].dist = -depth;
            mju_mulMatVec3(con[cnt].frame, mat1, dirccd.v);
            mju_mulMatVec3(con[cnt].pos, mat1, vecccd.v);
            mju_addTo3(con[cnt].pos, pos1);
            mju_zero3(con[cnt].frame+3);

            // count, stop if max number reached
            cnt++;
            if (cnt >= mjMAXCONPAIR) {
              r = rmax+1;
              c = cmax+1;
              i = 3;
              break;
            }
          }
        }
      }
    }
  }

  // restore mat2 and pos2
  mju_copy(mat2, savemat2, 9);
  mju_copy3(pos2, savepos2);

  // fix contact normals
  for (int i=0; i < cnt; i++) {
    mjc_fixNormal(m, d, con+i, g1, g2);
  }

  return cnt;
}



//--------------------------- fix contact frame normal ---------------------------------------------

// compute normal for point outside ellipsoid, using ray-projection SQP
static int mjc_ellipsoidInside(mjtNum nrm[3], const mjtNum pos[3], const mjtNum size[3]) {
  // algorithm constants
  const int maxiter = 30;
  const mjtNum tolerance = 1e-6;

  // precompute quantities
  mjtNum S2inv[3] = {1/(size[0]*size[0]), 1/(size[1]*size[1]), 1/(size[2]*size[2])};
  mjtNum C = pos[0]*pos[0]*S2inv[0] + pos[1]*pos[1]*S2inv[1] + pos[2]*pos[2]*S2inv[2] - 1;
  if (C > 0) {
    return 0;
  }

  // normalize initial normal (just in case)
  mju_normalize3(nrm);

  // main iteration
  int iter;
  for (iter=0; iter < maxiter; iter++) {
    // coefficients and determinant of quadratic
    mjtNum A = nrm[0]*nrm[0]*S2inv[0] + nrm[1]*nrm[1]*S2inv[1] + nrm[2]*nrm[2]*S2inv[2];
    mjtNum B = pos[0]*nrm[0]*S2inv[0] + pos[1]*nrm[1]*S2inv[1] + pos[2]*nrm[2]*S2inv[2];
    mjtNum det = B*B - A*C;
    if (det < mjMINVAL || A < mjMINVAL) {
      return (iter > 0);
    }

    // ray intersection with ellipse: pos + x*nrm, x>=0
    mjtNum x = (-B + mju_sqrt(det))/A;
    if (x < 0) {
      return (iter > 0);
    }

    // new point on ellipsoid
    mjtNum pnt[3];
    mju_addScl3(pnt, pos, nrm, x);

    // normal at new point
    mjtNum newnrm[3] = {pnt[0]*S2inv[0], pnt[1]*S2inv[1], pnt[2]*S2inv[2]};
    mju_normalize3(newnrm);

    // save change and assign
    mjtNum change = mju_dist3(nrm, newnrm);
    mju_copy3(nrm, newnrm);

    // terminate if converged
    if (change < tolerance) {
      break;
    }
  }

  return 1;
}



// compute normal for point inside ellipsoid, using diagonal QCQP
static int mjc_ellipsoidOutside(mjtNum nrm[3], const mjtNum pos[3], const mjtNum size[3]) {
  // algorithm constants
  const int maxiter = 30;
  const mjtNum tolerance = 1e-6;

  // precompute quantities
  mjtNum S2[3] = {size[0]*size[0], size[1]*size[1], size[2]*size[2]};
  mjtNum PS2[3] = {pos[0]*pos[0]*S2[0], pos[1]*pos[1]*S2[1], pos[2]*pos[2]*S2[2]};

  // main iteration
  mjtNum la = 0;
  int iter;
  for (iter=0; iter < maxiter; iter++) {
    // precompute 1/(s^2+la)
    mjtNum R[3] = {1/(S2[0]+la), 1/(S2[1]+la), 1/(S2[2]+la)};

    // value
    mjtNum val = PS2[0]*R[0]*R[0] + PS2[1]*R[1]*R[1] + PS2[2]*R[2]*R[2] - 1;
    if (val < tolerance) {
      break;
    }

    // derivative
    mjtNum deriv = -2*(PS2[0]*R[0]*R[0]*R[0] + PS2[1]*R[1]*R[1]*R[1] + PS2[2]*R[2]*R[2]*R[2]);
    if (deriv > -mjMINVAL) {
      break;
    }

    // delta
    mjtNum delta = -val/deriv;
    if (delta < tolerance) {
      break;
    }

    // update
    la += delta;
  }

  // compute normal given lambda
  nrm[0] = pos[0]/(S2[0]+la);
  nrm[1] = pos[1]/(S2[1]+la);
  nrm[2] = pos[2]/(S2[2]+la);
  mju_normalize3(nrm);

  return 1;
}



// entry point
void mjc_fixNormal(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2) {
  mjtNum dst1, dst2;

  // get geom ids and types
  int gid[2] = {g1, g2};
  mjtGeom type[2];
  for (int i=0; i < 2; i++) {
    type[i] = m->geom_type[gid[i]];

    // set to mjGEOM_NONE if type cannot be processed
    if (type[i] != mjGEOM_SPHERE    &&
        type[i] != mjGEOM_CAPSULE   &&
        type[i] != mjGEOM_ELLIPSOID &&
        type[i] != mjGEOM_CYLINDER) {
      type[i] = mjGEOM_NONE;
    }
  }

  // neither type can be processed: nothing to do
  if (type[0] == mjGEOM_NONE && type[1] == mjGEOM_NONE) {
    return;
  }

  // init normals
  mjtNum normal[2][3] = {
    {con->frame[0], con->frame[1], con->frame[2]},
    {-con->frame[0], -con->frame[1], -con->frame[2]}
  };

  // process geoms in type range
  int processed[2] = {0, 0};
  for (int i=0; i < 2; i++) {
    if (type[i] != mjGEOM_NONE) {
      // get geom mat and size
      mjtNum* mat = d->geom_xmat + 9*gid[i];
      mjtNum* size = m->geom_size + 3*gid[i];

      // map contact point and normal to local frame
      mjtNum dif[3], pos[3], nrm[3];
      mju_sub3(dif, con->pos, d->geom_xpos+3*gid[i]);
      mju_mulMatTVec3(pos, mat, dif);
      mju_mulMatTVec3(nrm, mat, normal[i]);

      // process according to type
      switch (type[i]) {
      case mjGEOM_SPHERE:
        mju_copy3(nrm, pos);
        processed[i] = 1;
        break;

      case mjGEOM_CAPSULE:
        // Z: bottom cap
        if (pos[2] < -size[1]) {
          nrm[2] = pos[2]+size[1];
        }

        // Z: top cap
        else if (pos[2] > size[1]) {
          nrm[2] = pos[2]-size[1];
        }

        // Z: cylinder
        else {
          nrm[2] = 0;
        }

        // copy XY
        nrm[0] = pos[0];
        nrm[1] = pos[1];
        processed[i] = 1;
        break;

      case mjGEOM_ELLIPSOID:
        // guard against invalid ellipsoid size (just in case)
        if (size[0] < mjMINVAL || size[1] < mjMINVAL || size[2] < mjMINVAL) {
          break;
        }

        // compute elliptic distance^2
        dst1 = pos[0]*pos[0]/(size[0]*size[0]) +
               pos[1]*pos[1]/(size[1]*size[1]) +
               pos[2]*pos[2]/(size[2]*size[2]);

        // dispatch to inside or outside solver
        if (dst1 <= 1) {
          processed[i] = mjc_ellipsoidInside(nrm, pos, size);
        } else {
          processed[i] = mjc_ellipsoidOutside(nrm, pos, size);
        }
        break;

      case mjGEOM_CYLINDER:
        // skip if within 5% length of flat wall
        if (mju_abs(pos[2]) > 0.95*size[1]) {
          break;
        }

        // compute distances to flat and round wall
        dst1 = mju_abs(size[1]-mju_abs(pos[2]));
        dst2 = mju_abs(size[0]-mju_norm(pos, 2));

        // require 4x closer to round than flat wall
        if (dst1 < 0.25*dst2) {
          break;
        }

        // set normal for round wall
        nrm[0] = pos[0];
        nrm[1] = pos[1];
        nrm[2] = 0;
        processed[i] = 1;
        break;
      default:
        // do nothing: only sphere, capsule, ellipsoid and cylinder are processed
        break;
      }

      // normalize and map normal to global frame
      if (processed[i]) {
        mju_normalize3(nrm);
        mju_mulMatVec3(normal[i], mat, nrm);
      }
    }
  }

  // both processed: average
  if (processed[0] && processed[1]) {
    mju_sub3(con->frame, normal[0], normal[1]);
    mju_normalize3(con->frame);
  }

  // first processed: copy
  else if (processed[0]) {
    mju_copy3(con->frame, normal[0]);
  }

  // second processed: copy reverse
  else if (processed[1]) {
    mju_scl3(con->frame, normal[1], -1);
  }

  // clear second frame axis if processed, just in case
  if (processed[0] || processed[1]) {
    mju_zero3(con->frame+3);
  }
}



//----------------------------  flex collisions ---------------------------------------------

// geom-elem or elem-elem or vert-elem convex collision using ccd
int mjc_ConvexElem(const mjModel* m, const mjData* d, mjContact* con,
                   int g1, int f1, int e1, int v1, int f2, int e2, mjtNum margin) {
  mjCCDObj obj1, obj2;
  mjc_initCCDObj(&obj1, m, d, g1, margin);
  mjc_initCCDObj(&obj2, m, d, -1, margin);
  mjc_setCCDObjFlex(&obj1, f1, e1, v1);
  mjc_setCCDObjFlex(&obj2, f2, e2, -1);

  // find contacts
  return mjc_CCDIteration(m, d, &obj1, &obj2, con, 1, margin);
}



// test a height field and a flex element for collision
int mjc_HFieldElem(const mjModel* m, const mjData* d, mjContact* con,
                   int g, int f, int e, mjtNum margin) {
  mjtNum vec[3], dx, dy;
  mjtNum xmin, xmax, ymin, ymax, zmin, zmax;
  int dr[2], cnt, rmin, rmax, cmin, cmax;
  mjCCDObj obj1;
  obj1.center = mjc_prism_center;
  obj1.support = mjc_prism_support;

  // get hfield info
  int hid = m->geom_dataid[g];
  int nrow = m->hfield_nrow[hid];
  int ncol = m->hfield_ncol[hid];
  mjtNum* hpos = d->geom_xpos + 3*g;
  mjtNum* hmat = d->geom_xmat + 9*g;
  mjtNum* hsize = m->hfield_size + 4*hid;
  const float* hdata = m->hfield_data + m->hfield_adr[hid];

  // get elem indo
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
  mjtNum* evert[4] = {NULL, NULL, NULL, NULL};
  for (int i=0; i <= dim; i++) {
    evert[i] = d->flexvert_xpos + 3*(m->flex_vertadr[f] + edata[i]);
  }
  mjtNum* ecenter = d->flexelem_aabb + 6*(m->flex_elemadr[f]+e);

  // ccd-related
  ccd_vec3_t dirccd, vecccd;
  ccd_real_t depth;
  mjCCDObj obj2;
  mjc_initCCDObj(&obj2, m, d, -1, margin);
  mjc_setCCDObjFlex(&obj2, f, e, -1);
  ccd_t ccd;

  //------------------------------------- AABB computation, box-box test

  // save elem vertices, transform to hfield frame
  mjtNum savevert[4][3];
  for (int i=0; i <= dim; i++) {
    mju_copy3(savevert[i], evert[i]);
    mju_sub3(vec, evert[i], hpos);
    mju_mulMatTVec(evert[i], hmat, vec, 3, 3);
  }

  // save elem center, transform to hfield frame
  mjtNum savecenter[3];
  mju_copy3(savecenter, ecenter);
  mju_sub3(vec, ecenter, hpos);
  mju_mulMatTVec(ecenter, hmat, vec, 3, 3);

  // compute elem bounding box (in hfield frame)
  xmin = xmax = evert[0][0];
  ymin = ymax = evert[0][1];
  zmin = zmax = evert[0][2];
  for (int i=1; i <= dim; i++) {
    xmin = mju_min(xmin, evert[i][0]);
    xmax = mju_max(xmax, evert[i][0]);
    ymin = mju_min(ymin, evert[i][1]);
    ymax = mju_max(ymax, evert[i][1]);
    zmin = mju_min(zmin, evert[i][2]);
    zmax = mju_max(zmax, evert[i][2]);
  }

  // box-box test
  if ((xmin-margin > hsize[0]) || (xmax+margin < -hsize[0]) ||
      (ymin-margin > hsize[1]) || (ymax+margin < -hsize[1]) ||
      (zmin-margin > hsize[2]) || (zmax+margin < -hsize[3])) {
    // restore vertices and center
    for (int i=0; i <= dim; i++) {
      mju_copy3(evert[i], savevert[i]);
    }
    mju_copy3(ecenter, savecenter);

    return 0;
  }

  // compute sub-grid bounds
  cmin = (int) mju_floor((xmin + hsize[0]) / (2*hsize[0]) * (ncol-1));
  cmax = (int) mju_ceil ((xmax + hsize[0]) / (2*hsize[0]) * (ncol-1));
  rmin = (int) mju_floor((ymin + hsize[1]) / (2*hsize[1]) * (nrow-1));
  rmax = (int) mju_ceil ((ymax + hsize[1]) / (2*hsize[1]) * (nrow-1));
  cmin = mjMAX(0, cmin);
  cmax = mjMIN(ncol-1, cmax);
  rmin = mjMAX(0, rmin);
  rmax = mjMIN(nrow-1, rmax);

  //------------------------------------- collision testing

  // init ccd structure
  CCD_INIT(&ccd);
  ccd.first_dir = prism_firstdir;
  ccd.center1 = mjccd_prism_center;
  ccd.center2 = mjccd_center;
  ccd.support1 = mjccd_prism_support;
  ccd.support2 = mjccd_support;

  // set ccd parameters
  ccd.max_iterations = m->opt.ccd_iterations;
  ccd.mpr_tolerance = m->opt.ccd_tolerance;

  // compute real-valued grid step, and triangulation direction
  dx = (2.0*hsize[0]) / (ncol-1);
  dy = (2.0*hsize[1]) / (nrow-1);
  dr[0] = 1;
  dr[1] = 0;

  // set zbottom value using base size
  obj1.prism[0][2] = obj1.prism[1][2] = obj1.prism[2][2] = -hsize[3];

  // process all prisms in sub-grid
  cnt = 0;
  for (int r=rmin; r < rmax; r++) {
    int nvert = 0;
    for (int c=cmin; c <= cmax; c++) {
      for (int k=0; k < 2; k++) {
        // send vertex to prism constructor
        addVert(&nvert, &obj1, dx*c-hsize[0], dy*(r+dr[k])-hsize[1],
                hdata[(r+dr[k])*ncol+c]*hsize[2]+margin);

        // check for enough vertices
        if (nvert > 2) {
          // prism height test
          if (obj1.prism[3][2] < zmin && obj1.prism[4][2] < zmin && obj1.prism[5][2] < zmin) {
            continue;
          }

          // run MPR, save contact
          if (mjc_penetration(m, &obj1, &obj2, &ccd, &depth, &dirccd, &vecccd) == 0) {
            if (!ccdVec3Eq(&dirccd, ccd_vec3_origin)) {
              // fill in contact data, transform to global coordinates
              con[cnt].dist = -depth;
              mju_mulMatVec3(con[cnt].frame, hmat, dirccd.v);
              mju_mulMatVec3(con[cnt].pos, hmat, vecccd.v);
              mju_addTo3(con[cnt].pos, hpos);
              mju_zero3(con[cnt].frame+3);

              // count, stop if max number reached
              cnt++;
              if (cnt >= mjMAXCONPAIR) {
                r = rmax+1;
                c = cmax+1;
                k = 3;
                break;
              }
            }
          }
        }
      }
    }
  }

  // restore elem vertices and center
  for (int i=0; i <= dim; i++) {
    mju_copy3(evert[i], savevert[i]);
  }
  mju_copy3(ecenter, savecenter);

  return cnt;
}
