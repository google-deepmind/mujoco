// Copyright 2023 DeepMind Technologies Limited
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

#include "engine/engine_vis_state.h"

#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include "engine/engine_core_constraint.h"
#include "engine/engine_plugin.h"
#include "engine/engine_support.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_vis_init.h"
#include "engine/engine_vis_interact.h"
#include "engine/engine_vis_visualize.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

// this source file needs to treat XMJV differently from other X macros
#undef XMJV



// round size up to multiples of 64-byte cache lines
static inline size_t roundUpToCacheLine(size_t n) {
  return 64 * ((n / 64) + (n % 64 ? 1 : 0));
}



// set default scene
void mjv_defaultSceneState(mjvSceneState* scnstate) {
  memset(scnstate, 0, sizeof(mjvSceneState));
  mjv_defaultScene(&scnstate->plugincache);
}



// allocate and init scene state
void mjv_makeSceneState(const mjModel* m, const mjData* d, mjvSceneState* scnstate, int maxgeom) {
  mjv_freeScene(&scnstate->plugincache);
  mju_free(scnstate->buffer);

#ifdef MEMORY_SANITIZER
  __msan_allocated_memory(scnstate, sizeof(mjvSceneState));
  mjv_defaultScene(&scnstate->plugincache);
#endif

  scnstate->nbuffer = 0;
  scnstate->maxgeom = maxgeom;

#define X(var)
#define XMJV(var) scnstate->model.var = m->var;
  MJMODEL_INTS
#undef XMJV
#undef X

#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) \
  scnstate->nbuffer += roundUpToCacheLine(sizeof(dtype) * m->dim0 * dim1);
  MJMODEL_POINTERS
#undef XMJV
#undef X

#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) \
  scnstate->nbuffer += roundUpToCacheLine(sizeof(dtype) * m->dim0 * dim1);
  MJDATA_POINTERS
#undef XMJV
#undef X

  int condimmax = mj_isPyramidal(m) ? 10 : 6;
  scnstate->nbuffer += roundUpToCacheLine(sizeof(mjContact) * maxgeom);
  scnstate->nbuffer += roundUpToCacheLine(sizeof(mjtNum) * maxgeom * condimmax);

  scnstate->buffer = mju_malloc(scnstate->nbuffer);

  char* ptr = scnstate->buffer;

#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) \
  scnstate->model.var = (dtype*)ptr; \
  ptr += roundUpToCacheLine(sizeof(dtype) * m->dim0 * dim1);
  MJMODEL_POINTERS
#undef XMJV
#undef X

#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) \
  scnstate->data.var = (dtype*)ptr;  \
  ptr += roundUpToCacheLine(sizeof(dtype) * m->dim0 * dim1);
  MJDATA_POINTERS
#undef XMJV
#undef X

  scnstate->data.contact = (mjContact*)ptr;
  ptr += roundUpToCacheLine(sizeof(mjContact) * scnstate->maxgeom);

  scnstate->data.efc_force = (mjtNum*)ptr;
  ptr += roundUpToCacheLine(sizeof(mjtNum) * scnstate->maxgeom * condimmax);

  // should not occur
  if (ptr - (char*)scnstate->buffer != scnstate->nbuffer) {
    mjERROR("mjvSceneState buffer is not fully used");
  }

  mjv_makeScene(m, &scnstate->plugincache, maxgeom);
}



// free scene state
void mjv_freeSceneState(mjvSceneState* scnstate) {
  mjv_freeScene(&scnstate->plugincache);
  mju_free(scnstate->buffer);
  mjv_defaultSceneState(scnstate);
}



// shallow copy scene state into model and data for use with mjv functions
void mjv_assignFromSceneState(const mjvSceneState* scnstate, mjModel* m, mjData* d) {
  if (m) {
    memset(m, 0, sizeof(mjModel));

#ifdef MEMORY_SANITIZER
    // Tell msan to treat the entire buffer as uninitialized
    __msan_allocated_memory(m, sizeof(mjModel));
#endif

#define X(var)
#define XMJV(var) m->var = scnstate->model.var;
    MJMODEL_INTS
#undef XMJV
#undef X

    m->opt = scnstate->model.opt;
    m->vis = scnstate->model.vis;
    m->stat = scnstate->model.stat;

#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) m->var = scnstate->model.var;
    MJMODEL_POINTERS
#undef XMJV
#undef X
  }

  if (d) {
    memset(d, 0, sizeof(mjData));

#ifdef MEMORY_SANITIZER
    // Tell msan to treat the entire buffer as uninitialized
    __msan_allocated_memory(d, sizeof(mjData));
#endif

    memcpy(d->warning, scnstate->data.warning, sizeof(d->warning));
    d->nefc = scnstate->data.nefc;
    d->ncon = scnstate->data.ncon;
    d->time = scnstate->data.time;

  #define X(dtype, var, dim0, dim1)
  #define XMJV(dtype, var, dim0, dim1) d->var = scnstate->data.var;
    MJDATA_POINTERS
  #undef XMJV
  #undef X

    d->contact = scnstate->data.contact;
    d->efc_force = scnstate->data.efc_force;
  }
}



// update entire scene from a scene state, return the number of new mjWARN_VGEOMFULL warnings
int mjv_updateSceneFromState(const mjvSceneState* scnstate, const mjvOption* opt,
                             const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn) {
  // shallow-copy scnstate pointers into mjModel and mjData
  mjModel m;
  mjData d;
  mjv_assignFromSceneState(scnstate, &m, &d);

  // save the number of mjWARN_VGEOMFULL warnings before the scene update
  int warning_start = d.warning[mjWARN_VGEOMFULL].number;

  // copy mjvGeoms added by plugins
  int nplugingeom = scnstate->plugincache.ngeom;
  if (nplugingeom > scn->maxgeom) {
    mj_warning(&d, mjWARN_VGEOMFULL, scn->maxgeom);
    scn->ngeom = scn->maxgeom;
  } else {
    scn->ngeom = nplugingeom;
  }
  memcpy(scn->geoms, scnstate->plugincache.geoms, sizeof(mjvGeom) * scn->ngeom);

  // add all categories
  mjv_addGeoms(&m, &d, opt, pert, catmask, scn);

  // add lights
  mjv_makeLights(&m, &d, scn);

  // update camera
  mjv_updateCamera(&m, &d, cam, scn);

  // update skins
  if (opt->flags[mjVIS_SKIN]) {
    mjv_updateActiveSkin(&m, &d, scn, opt);
  }

  // return the number of new mjWARN_VGEOMFULL warnings generated
  return d.warning[mjWARN_VGEOMFULL].number - warning_start;
}



// update a scene state from model and data
void mjv_updateSceneState(const mjModel* m, mjData* d, const mjvOption* opt,
                          mjvSceneState* scnstate) {
  // Check that mjModel sizes haven't changed.
#define X(var)
#define XMJV(var)                      \
  if (scnstate->model.var != m->var) { \
    mjERROR("m->%s changed", #var);  \
  }
  MJMODEL_INTS
#undef XMJV
#undef X

  // Update plugin visualization cache.
  scnstate->plugincache.ngeom = 0;
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    // iterate over plugins, call visualize if defined
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->visualize) {
        plugin->visualize(m, d, opt, &scnstate->plugincache, i);
      }
    }
  }

  // Copy variable-sized arrays in mjModel.
#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) \
  memcpy(scnstate->model.var, m->var, sizeof(dtype) * m->dim0 * dim1);
  MJMODEL_POINTERS
#undef XMJV
#undef X

  scnstate->model.opt = m->opt;
  scnstate->model.vis = m->vis;
  scnstate->model.stat = m->stat;

  // Copy mjData variables.
  memcpy(scnstate->data.warning, d->warning, sizeof(d->warning));
  scnstate->data.time = d->time;

  // Copy variable-sized arrays in mjData.
#define X(dtype, var, dim0, dim1)
#define XMJV(dtype, var, dim0, dim1) \
  memcpy(scnstate->data.var, d->var, sizeof(dtype) * m->dim0 * dim1);
  MJDATA_POINTERS
#undef XMJV
#undef X

  // Copy contacts.
  {
    if (d->ncon > scnstate->maxgeom) {
      mj_warning(d, mjWARN_VGEOMFULL, scnstate->maxgeom);
      scnstate->data.ncon = scnstate->maxgeom;
    } else {
      scnstate->data.ncon = d->ncon;
    }
    memcpy(scnstate->data.contact, d->contact, sizeof(mjContact) * scnstate->data.ncon);
  }

  // Copy only the entries in efc_force that correspond to contacts.
  {
    scnstate->data.nefc = 0;
    for (int i = 0; i < scnstate->data.ncon; ++i) {
      const mjContact* con = &d->contact[i];
      scnstate->data.nefc += con->dim;
    }

    int efc_address = 0;
    int ispyramid = mj_isPyramidal(m);
    for (int i = 0; i < scnstate->data.ncon; ++i) {
      mjContact* con = &scnstate->data.contact[i];
      int dim = con->dim;
      if (ispyramid && dim > 1){
        dim = 2*(dim - 1);
      }
      for (int j = 0; j < dim; ++j) {
        scnstate->data.efc_force[efc_address + j] = d->efc_force[con->efc_address + j];
      }
      con->efc_address = efc_address;
      efc_address += dim;
    }
  }
}



// move camera with mouse given a scene state; action is mjtMouse
MJAPI void mjv_moveCameraFromState(const mjvSceneState* scnstate, int action,
                                   mjtNum reldx, mjtNum reldy,
                                   const mjvScene* scn, mjvCamera* cam) {
  mjModel m;
  mjv_assignFromSceneState(scnstate, &m, NULL);
  mjv_moveCamera(&m, action, reldx, reldy, scn, cam);
}



// move perturb object with mouse given a scene state; action is mjtMouse
MJAPI void mjv_movePerturbFromState(const mjvSceneState* scnstate, int action,
                                    mjtNum reldx, mjtNum reldy,
                                    const mjvScene* scn, mjvPerturb* pert) {
  mjModel m;
  mjData d;
  mjv_assignFromSceneState(scnstate, &m, &d);
  mjv_movePerturb(&m, &d, action, reldx, reldy, scn, pert);
}
