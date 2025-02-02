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
  mjv_defaultScene(&scnstate->scratch);
}



// allocate and init scene state
void mjv_makeSceneState(const mjModel* m, const mjData* d, mjvSceneState* scnstate, int maxgeom) {
  mjv_freeScene(&scnstate->scratch);
  mju_free(scnstate->buffer);

#ifdef MEMORY_SANITIZER
  __msan_allocated_memory(scnstate, sizeof(mjvSceneState));
  mjv_defaultScene(&scnstate->scratch);
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

  // create an arena in the scnstate, to allow visualization code to use the stack.
  // TODO: Consider allocating way less than narena, since stack allocations in
  // visualization code are much smaller than the arena space required by the model,
  // typically.
  scnstate->nbuffer += roundUpToCacheLine(m->narena);
  // buffer space required for contacts
  int condimmax = mj_isPyramidal(m) ? 10 : 6;
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->contact) * maxgeom);
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->efc_force) * maxgeom * condimmax);

  // buffer space required for islands
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->island_dofadr) * m->ntree);
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->island_dofind) * m->nv);
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->dof_island) * m->nv);
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->efc_island) * maxgeom * condimmax);
  scnstate->nbuffer += roundUpToCacheLine(sizeof(*d->tendon_efcadr) * m->ntendon);

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

  scnstate->model.narena = m->narena;
  scnstate->data.arena = (void*)ptr;
  ptr += roundUpToCacheLine(m->narena);

  scnstate->data.contact = (mjContact*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.contact) * scnstate->maxgeom);

  scnstate->data.efc_force = (mjtNum*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.efc_force) * scnstate->maxgeom * condimmax);

  scnstate->data.island_dofadr = (int*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.island_dofadr) * scnstate->model.ntree);

  scnstate->data.island_dofind = (int*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.island_dofind) * scnstate->model.nv);

  scnstate->data.dof_island = (int*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.dof_island) * scnstate->model.nv);

  scnstate->data.efc_island = (int*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.efc_island) * scnstate->maxgeom * condimmax);

  scnstate->data.tendon_efcadr = (int*)ptr;
  ptr += roundUpToCacheLine(sizeof(*scnstate->data.tendon_efcadr) * m->ntendon);

  // should not occur
  if (ptr - (char*)scnstate->buffer != scnstate->nbuffer) {
    mjERROR("mjvSceneState buffer is not fully used");
  }

  mjv_makeScene(m, &scnstate->scratch, maxgeom);
}



// free scene state
void mjv_freeSceneState(mjvSceneState* scnstate) {
  mjv_freeScene(&scnstate->scratch);
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
    m->narena = scnstate->model.narena;

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
    d->threadpool = 0;
    d->nefc = scnstate->data.nefc;
    d->ncon = scnstate->data.ncon;
    d->nisland = scnstate->data.nisland;
    d->time = scnstate->data.time;
    d->narena = scnstate->model.narena;
    d->arena = scnstate->data.arena;
    d->parena = 0;
    d->pbase = 0;
    d->pstack = 0;

  #define X(dtype, var, dim0, dim1)
  #define XMJV(dtype, var, dim0, dim1) d->var = scnstate->data.var;
    MJDATA_POINTERS
  #undef XMJV
  #undef X

    d->contact = scnstate->data.contact;
    d->efc_force = scnstate->data.efc_force;
    d->island_dofadr = scnstate->data.island_dofadr;
    d->island_dofind = scnstate->data.island_dofind;
    d->dof_island = scnstate->data.dof_island;
    d->efc_island = scnstate->data.efc_island;
    d->tendon_efcadr = scnstate->data.tendon_efcadr;
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
  int nplugingeom = scnstate->scratch.ngeom;
  if (nplugingeom > scn->maxgeom) {
    mj_warning(&d, mjWARN_VGEOMFULL, scn->maxgeom);
    scn->ngeom = scn->maxgeom;
  } else {
    scn->ngeom = nplugingeom;
  }
  memcpy(scn->geoms, scnstate->scratch.geoms, sizeof(mjvGeom) * scn->ngeom);

  // add all categories
  mjv_addGeoms(&m, &d, opt, pert, catmask, scn);

  // update camera
  mjv_updateCamera(&m, &d, cam, scn);

  // add lights
  mjv_makeLights(&m, &d, scn);

  // update flexes
  if (opt->flags[mjVIS_FLEXVERT] || opt->flags[mjVIS_FLEXEDGE] ||
      opt->flags[mjVIS_FLEXFACE] || opt->flags[mjVIS_FLEXSKIN]) {
    mjv_updateActiveFlex(&m, &d, scn, opt);
  }

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
#define XMJV(var)                                                          \
  if (scnstate->model.var != m->var) {                                     \
    mjERROR("m->%s changed: %d vs %d", #var, scnstate->model.var, m->var); \
  }
  MJMODEL_INTS
#undef XMJV
#undef X

  // Update plugin visualization cache.
  scnstate->scratch.ngeom = 0;
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
        plugin->visualize(m, d, opt, &scnstate->scratch, i);
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
    memcpy(scnstate->data.contact, d->contact, sizeof(*d->contact) * scnstate->data.ncon);
  }

  // Copy only the entries in efc_force and efc_island that correspond to contacts.
  {
    scnstate->data.nefc = 0;
    for (int i = 0; i < scnstate->data.ncon; ++i) {
      const mjContact* con = &d->contact[i];
      scnstate->data.nefc += con->dim;
    }
    scnstate->data.nefc += scnstate->model.ntendon;

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
        if (d->nisland) {
          scnstate->data.efc_island[efc_address + j] = d->efc_island[con->efc_address + j];
        }
      }
      con->efc_address = efc_address;
      efc_address += dim;
    }
    if (d->nisland) {
      for (int i = 0; i < scnstate->model.ntendon; ++i) {
        int efcadr = d->tendon_efcadr[i];
        if (efcadr != -1) {
          scnstate->data.efc_island[efcadr] = d->efc_island[efcadr];
        }
      }
    }
  }

  // Copy island data.
  scnstate->data.nisland = d->nisland;
  if (d->nisland) {
    memcpy(scnstate->data.island_dofadr, d->island_dofadr, sizeof(*d->island_dofadr) * d->nisland);
    memcpy(scnstate->data.island_dofind, d->island_dofind, sizeof(*d->island_dofind) *  m->nv);
    memcpy(scnstate->data.dof_island, d->dof_island, sizeof(*d->dof_island) * m->nv);
    memcpy(scnstate->data.tendon_efcadr, d->tendon_efcadr, sizeof(*d->tendon_efcadr) * m->ntendon);
  }
}



// move camera with mouse given a scene state; action is mjtMouse
void mjv_moveCameraFromState(const mjvSceneState* scnstate, int action,
                             mjtNum reldx, mjtNum reldy,
                             const mjvScene* scn, mjvCamera* cam) {
  mjModel m;
  mjv_assignFromSceneState(scnstate, &m, NULL);
  mjv_moveCamera(&m, action, reldx, reldy, scn, cam);
}



// move perturb object with mouse given a scene state; action is mjtMouse
void mjv_movePerturbFromState(const mjvSceneState* scnstate, int action,
                              mjtNum reldx, mjtNum reldy,
                              const mjvScene* scn, mjvPerturb* pert) {
  mjModel m;
  mjData d;
  mjv_assignFromSceneState(scnstate, &m, &d);
  mjv_movePerturb(&m, &d, action, reldx, reldy, scn, pert);
}
