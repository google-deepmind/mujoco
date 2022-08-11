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

#include "engine/engine_vis_init.h"

#include <math.h>
#include <string.h>

#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

#ifdef _MSC_VER
  #pragma warning (disable: 4305)  // disable MSVC warning: truncation from 'double' to 'float'
#endif


//--------------------------------- Strings --------------------------------------------------------

// label names
const char* mjLABELSTRING[mjNLABEL] = {
  "None",
  "Body",
  "Joint",
  "Geom",
  "Site",
  "Camera",
  "Light",
  "Tendon",
  "Actuator",
  "Constraint",
  "Skin",
  "Selection",
  "SelPoint",
  "ContactForce"
};


// frame names
const char* mjFRAMESTRING[mjNFRAME] = {
  "None",
  "Body",
  "Geom",
  "Site",
  "Camera",
  "Light",
  "Contact",
  "World"
};


// visual opptions: {name, initial value, shortcut}
const char* mjVISSTRING[mjNVISFLAG][3] = {
  {"Convex &Hull",    "0", "H"},
  {"Te&xture",        "1", "X"},
  {"&Joint",          "0", "J"},
  {"Camera",          "0", "Q"},
  {"Act&uator",       "0", "U"},
  {"Act&ivation",     "0", ","},
  {"Light",           "0", "Z"},
  {"Tendon",          "1", "V"},
  {"Range Finder",    "1", "Y"},
  {"Co&nstraint",     "0", "N"},
  {"&Inertia",        "0", "I"},
  {"Scale Inertia",   "0", "'"},
  {"Pertur&b Force",  "0", "B"},
  {"Perturb &Object", "1", "O"},
  {"&Contact Point",  "0", "C"},
  {"Contact &Force",  "0", "F"},
  {"Contact S&plit",  "0", "P"},
  {"&Transparent",    "0", "T"},
  {"&Auto Connect",   "0", "A"},
  {"Center of &Mass", "0", "M"},
  {"S&elect Point",   "0", "E"},
  {"Static Bo&dy",    "1", "D"},
  {"Skin",            "1", ";"}
};


// render options: {name, initial value, shortcut}
const char* mjRNDSTRING[mjNRNDFLAG][3] = {
  {"Shadow",      "1", "S"},
  {"Wireframe",   "0", "W"},
  {"Reflection",  "1", "R"},
  {"Additive",    "0", "L"},
  {"Skybox",      "1", "K"},
  {"Fog",         "0", "G"},
  {"Haze",        "1", "/"},
  {"Segment",     "0", ","},
  {"Id Color",    "0", "."},
  {"Cull Face",   "1", ""}
};



//--------------------------------- Implementation -------------------------------------------------


// allocate and init abstract scene
void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom) {
  // free previous
  mjv_freeScene(scn);

  // allocate geom buffers
  if (maxgeom>0) {
    // allocate
    scn->maxgeom = maxgeom;
    scn->geoms = (mjvGeom*) mju_malloc(maxgeom*sizeof(mjvGeom));
    scn->geomorder = (int*) mju_malloc(maxgeom*sizeof(int));

    // check allocation
    if (!scn->geoms || !scn->geomorder) {
      mju_error("Could not allocate geom buffers");
    }
  }

  // set default OpenGL options
  for (int i=0; i<mjNRNDFLAG; i++) {
    scn->flags[i] = (mjRNDSTRING[i][1][0]=='1');
  }

  // set default model transformation
  scn->scale = 1;
  scn->rotate[0] = 1;

  // set number of skins
  if (m) {
    scn->nskin = m->nskin;
  } else {
    scn->nskin = 0;
  }

  // allocate skin data
  if (scn->nskin) {
    // compute number of vertices in all skins
    int nskin = m->nskin;
    int totvert = 0;
    for (int i=0; i<nskin; i++) {
      totvert += m->skin_vertnum[i];
    }

    // allocate
    scn->skinfacenum = (int*) mju_malloc(nskin*sizeof(int));
    scn->skinvertadr = (int*) mju_malloc(nskin*sizeof(int));
    scn->skinvertnum = (int*) mju_malloc(nskin*sizeof(int));
    scn->skinvert    = (float*) mju_malloc(3*totvert*sizeof(float));
    scn->skinnormal  = (float*) mju_malloc(3*totvert*sizeof(float));

    // check allocation
    if (!scn->skinfacenum ||
        !scn->skinvertadr ||
        !scn->skinvertnum ||
        !scn->skinvert    ||
        !scn->skinnormal) {
      mju_error("Could not allocate skin buffers");
    }

    // copy constant data
    for (int i=0; i<nskin; i++) {
      scn->skinfacenum[i] = m->skin_facenum[i];
      scn->skinvertadr[i] = m->skin_vertadr[i];
      scn->skinvertnum[i] = m->skin_vertnum[i];
    }
  }

  // mjvGeom, mjvLight, mjvGLCamera objects are invalid
}



// free abstract scene
void mjv_freeScene(mjvScene* scn) {
  // free buffers allocated by mjv_makeScene
  mju_free(scn->geoms);
  mju_free(scn->geomorder);
  mju_free(scn->skinfacenum);
  mju_free(scn->skinvertadr);
  mju_free(scn->skinvertnum);
  mju_free(scn->skinvert);
  mju_free(scn->skinnormal);

  // clear data structure
  mjv_defaultScene(scn);
}



// set default scene
void mjv_defaultScene(mjvScene* scn) {
  memset(scn, 0, sizeof(mjvScene));
}



// set default visualization options
void mjv_defaultOption(mjvOption* vopt) {
  vopt->label = mjLABEL_NONE;
  vopt->frame = mjFRAME_NONE;

  for (int i=0; i<mjNGROUP; i++) {
    int state = (i<3 ? 1 : 0);
    vopt->geomgroup[i] = state;
    vopt->sitegroup[i] = state;
    vopt->jointgroup[i] = state;
    vopt->tendongroup[i] = state;
    vopt->actuatorgroup[i] = state;
    vopt->skingroup[i] = state;
  }

  for (int i=0; i<mjNVISFLAG; i++) {
    vopt->flags[i] = (mjVISSTRING[i][1][0]=='1');
  }
}



// set default camera
void mjv_defaultCamera(mjvCamera* cam) {
  memset(cam, 0, sizeof(mjvCamera));

  cam->type        = mjCAMERA_FREE;
  cam->fixedcamid  = -1;
  cam->trackbodyid = -1;
  cam->distance    = 2;
  cam->azimuth     = 90;
  cam->elevation   = -45;
}



// set default free camera
void mjv_defaultFreeCamera(const mjModel* m, mjvCamera* cam) {
  memset(cam, 0, sizeof(mjvCamera));

  cam->type        = mjCAMERA_FREE;
  cam->fixedcamid  = -1;
  cam->trackbodyid = -1;
  cam->lookat[0]   = m->stat.center[0];
  cam->lookat[1]   = m->stat.center[1];
  cam->lookat[2]   = m->stat.center[2];
  cam->distance    = 1.5 * m->stat.extent;
  cam->azimuth     = m->vis.global.azimuth;
  cam->elevation   = m->vis.global.elevation;
}



// set default perturbation
void mjv_defaultPerturb(mjvPerturb* pert) {
  memset(pert, 0, sizeof(mjvPerturb));

  pert->skinselect = -1;
  pert->refquat[0] = 1;
  pert->scale = 1;
}



// predefined line colors
static const float _linergb[8][3] = {
  {1.0, 0.3, 0.3},
  {0.1, 1.0, 0.1},
  {0.3, 0.3, 1.0},
  {0.1, 1.0, 1.0},
  {1.0, 0.2, 1.0},
  {1.0, 1.0, 0.1},
  {1.0, 0.6, 0.2},
  {0.6, 0.7, 0.4}
};



// set default figure
void mjv_defaultFigure(mjvFigure* fig) {
  // set everything to zero
  memset(fig, 0, sizeof(mjvFigure));

  // disable highlight
  fig->highlightid = -1;

  // set enable flags
  fig->flg_legend = 1;
  fig->flg_ticklabel[0] = 1;
  fig->flg_ticklabel[1] = 1;
  fig->flg_extend = 1;

  // set style
  fig->linewidth = 3;
  fig->gridwidth = 1;
  fig->gridsize[0] = 2;
  fig->gridsize[1] = 2;
  fig->gridrgb[0] = 0.4f;
  fig->gridrgb[1] = 0.4f;
  fig->gridrgb[2] = 0.4f;
  fig->figurergba[3] = 1;
  fig->panergba[3] = 1;
  fig->legendrgba[3] = 0.3f;
  fig->textrgb[0] = 1;
  fig->textrgb[1] = 1;
  fig->textrgb[2] = 1;
  fig->range[0][0] = 0;
  fig->range[0][1] = 1;
  fig->range[1][0] = 0;
  fig->range[1][1] = 1;
  mjSTRNCPY(fig->xformat, "%.0f");
  mjSTRNCPY(fig->yformat, "%.2g");
  mjSTRNCPY(fig->minwidth, "XXX");

  // set line colors
  for (int n=0; n<mjMAXLINE; n++) {
    // predefined colors
    if (n<8) {
      fig->linergb[n][0] = _linergb[n][0];
      fig->linergb[n][1] = _linergb[n][1];
      fig->linergb[n][2] = _linergb[n][2];
    }

    // automatically generated colors: Halton sequence
    else {
      fig->linergb[n][0] = 0.1f + 0.8f*mju_Halton(n, 2);
      fig->linergb[n][1] = 0.1f + 0.8f*mju_Halton(n, 3);
      fig->linergb[n][2] = 0.1f + 0.8f*mju_Halton(n, 5);
    }
  }
}



// compute rbound for mjvGeom
float mjv_rbound(const mjvGeom* geom) {
  // model geom: return
  if (geom->objtype==mjOBJ_GEOM) {
    return geom->modelrbound;
  }

  // compute rbound according to type
  const float* s = geom->size;
  switch (geom->type) {
  case mjGEOM_SPHERE:
    return s[0];

  case mjGEOM_CAPSULE:
    return (s[0]+s[2]);

  case mjGEOM_CYLINDER:
    return sqrtf(s[0]*s[0] + s[2]*s[2]);

  case mjGEOM_BOX:
    return sqrtf(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
    break;

  default:  // not accurate for arrows, but they are not transparent
    return mjMAX(s[0], mjMAX(s[1], s[2]));
  }
}
