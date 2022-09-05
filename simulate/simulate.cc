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

#include "simulate.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <GLFW/glfw3.h>
#include "lodepng.h"
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include "glfw_dispatch.h"
#include "uitools.h"
#include "array_safety.h"

// When launched via an App Bundle on macOS, the working directory is the path to the App Bundle's
// resource directory. This causes files to be saved into the bundle, which is not the desired
// behavior. Instead, we open a save dialog box to ask the user where to put the file.
// Since the dialog box logic needs to be written in Objective-C, we separate it into a different
// source file.
#ifdef __APPLE__
std::string getSavePath(const char* filename);
#else
static std::string getSavePath(const char* filename) {
  return filename;
}
#endif

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using ::mujoco::Glfw;

//------------------------------------------- global -----------------------------------------------

const int maxgeom = 5000;            // preallocated geom array in mjvScene
const double zoom_increment = 0.02;  // ratio of one click-wheel zoom increment to vertical extent

// section ids
enum {
  // left ui
  SECT_FILE   = 0,
  SECT_OPTION,
  SECT_SIMULATION,
  SECT_WATCH,
  SECT_PHYSICS,
  SECT_RENDERING,
  SECT_GROUP,
  NSECT0,

  // right ui
  SECT_JOINT = 0,
  SECT_CONTROL,
  NSECT1
};

// file section of UI
const mjuiDef defFile[] = {
  {mjITEM_SECTION,   "File",          1, nullptr,                    "AF"},
  {mjITEM_BUTTON,    "Save xml",      2, nullptr,                    ""},
  {mjITEM_BUTTON,    "Save mjb",      2, nullptr,                    ""},
  {mjITEM_BUTTON,    "Print model",   2, nullptr,                    "CM"},
  {mjITEM_BUTTON,    "Print data",    2, nullptr,                    "CD"},
  {mjITEM_BUTTON,    "Quit",          1, nullptr,                    "CQ"},
  {mjITEM_BUTTON,    "Screenshot",    2, NULL,                       "CP"},
  {mjITEM_END}
};

// help strings
const char help_content[] =
  "Space\n"
  "+  -\n"
  "Right arrow\n"
  "[  ]\n"
  "Esc\n"
  "Double-click\n"
  "Page Up\n"
  "Right double-click\n"
  "Ctrl Right double-click\n"
  "Scroll, middle drag\n"
  "Left drag\n"
  "[Shift] right drag\n"
  "Ctrl [Shift] drag\n"
  "Ctrl [Shift] right drag\n"
  "F1\n"
  "F2\n"
  "F3\n"
  "F4\n"
  "F5\n"
  "UI right hold\n"
  "UI title double-click";

const char help_title[] =
  "Play / Pause\n"
  "Speed up / down\n"
  "Step\n"
  "Cycle cameras\n"
  "Free camera\n"
  "Select\n"
  "Select parent\n"
  "Center\n"
  "Tracking camera\n"
  "Zoom\n"
  "View rotate\n"
  "View translate\n"
  "Object rotate\n"
  "Object translate\n"
  "Help\n"
  "Info\n"
  "Profiler\n"
  "Sensors\n"
  "Full screen\n"
  "Show UI shortcuts\n"
  "Expand/collapse all";


//-------------------------------- profiler, sensor, info, watch -----------------------------------

// init profiler figures
void profilerinit(mj::Simulate* sim) {
  int i, n;

  // set figures to default
  mjv_defaultFigure(&sim->figconstraint);
  mjv_defaultFigure(&sim->figcost);
  mjv_defaultFigure(&sim->figtimer);
  mjv_defaultFigure(&sim->figsize);

  // titles
  mju::strcpy_arr(sim->figconstraint.title, "Counts");
  mju::strcpy_arr(sim->figcost.title, "Convergence (log 10)");
  mju::strcpy_arr(sim->figsize.title, "Dimensions");
  mju::strcpy_arr(sim->figtimer.title, "CPU time (msec)");

  // x-labels
  mju::strcpy_arr(sim->figconstraint.xlabel, "Solver iteration");
  mju::strcpy_arr(sim->figcost.xlabel, "Solver iteration");
  mju::strcpy_arr(sim->figsize.xlabel, "Video frame");
  mju::strcpy_arr(sim->figtimer.xlabel, "Video frame");

  // y-tick nubmer formats
  mju::strcpy_arr(sim->figconstraint.yformat, "%.0f");
  mju::strcpy_arr(sim->figcost.yformat, "%.1f");
  mju::strcpy_arr(sim->figsize.yformat, "%.0f");
  mju::strcpy_arr(sim->figtimer.yformat, "%.2f");

  // colors
  sim->figconstraint.figurergba[0] = 0.1f;
  sim->figcost.figurergba[2]       = 0.2f;
  sim->figsize.figurergba[0]       = 0.1f;
  sim->figtimer.figurergba[2]      = 0.2f;
  sim->figconstraint.figurergba[3] = 0.5f;
  sim->figcost.figurergba[3]       = 0.5f;
  sim->figsize.figurergba[3]       = 0.5f;
  sim->figtimer.figurergba[3]      = 0.5f;

  // legends
  mju::strcpy_arr(sim->figconstraint.linename[0], "total");
  mju::strcpy_arr(sim->figconstraint.linename[1], "active");
  mju::strcpy_arr(sim->figconstraint.linename[2], "changed");
  mju::strcpy_arr(sim->figconstraint.linename[3], "evals");
  mju::strcpy_arr(sim->figconstraint.linename[4], "updates");
  mju::strcpy_arr(sim->figcost.linename[0], "improvement");
  mju::strcpy_arr(sim->figcost.linename[1], "gradient");
  mju::strcpy_arr(sim->figcost.linename[2], "lineslope");
  mju::strcpy_arr(sim->figsize.linename[0], "dof");
  mju::strcpy_arr(sim->figsize.linename[1], "body");
  mju::strcpy_arr(sim->figsize.linename[2], "constraint");
  mju::strcpy_arr(sim->figsize.linename[3], "sqrt(nnz)");
  mju::strcpy_arr(sim->figsize.linename[4], "contact");
  mju::strcpy_arr(sim->figsize.linename[5], "iteration");
  mju::strcpy_arr(sim->figtimer.linename[0], "total");
  mju::strcpy_arr(sim->figtimer.linename[1], "collision");
  mju::strcpy_arr(sim->figtimer.linename[2], "prepare");
  mju::strcpy_arr(sim->figtimer.linename[3], "solve");
  mju::strcpy_arr(sim->figtimer.linename[4], "other");

  // grid sizes
  sim->figconstraint.gridsize[0] = 5;
  sim->figconstraint.gridsize[1] = 5;
  sim->figcost.gridsize[0] = 5;
  sim->figcost.gridsize[1] = 5;
  sim->figsize.gridsize[0] = 3;
  sim->figsize.gridsize[1] = 5;
  sim->figtimer.gridsize[0] = 3;
  sim->figtimer.gridsize[1] = 5;

  // minimum ranges
  sim->figconstraint.range[0][0] = 0;
  sim->figconstraint.range[0][1] = 20;
  sim->figconstraint.range[1][0] = 0;
  sim->figconstraint.range[1][1] = 80;
  sim->figcost.range[0][0] = 0;
  sim->figcost.range[0][1] = 20;
  sim->figcost.range[1][0] = -15;
  sim->figcost.range[1][1] = 5;
  sim->figsize.range[0][0] = -200;
  sim->figsize.range[0][1] = 0;
  sim->figsize.range[1][0] = 0;
  sim->figsize.range[1][1] = 100;
  sim->figtimer.range[0][0] = -200;
  sim->figtimer.range[0][1] = 0;
  sim->figtimer.range[1][0] = 0;
  sim->figtimer.range[1][1] = 0.4f;

  // init x axis on history figures (do not show yet)
  for (n=0; n<6; n++)
    for (i=0; i<mjMAXLINEPNT; i++) {
      sim->figtimer.linedata[n][2*i] = -i;
      sim->figsize.linedata[n][2*i] = -i;
    }
}

// update profiler figures
void profilerupdate(mj::Simulate* sim) {
  int i, n;

  // update constraint figure
  sim->figconstraint.linepnt[0] = mjMIN(mjMIN(sim->d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
  for (i=1; i<5; i++) {
    sim->figconstraint.linepnt[i] = sim->figconstraint.linepnt[0];
  }
  if (sim->m->opt.solver==mjSOL_PGS) {
    sim->figconstraint.linepnt[3] = 0;
    sim->figconstraint.linepnt[4] = 0;
  }
  if (sim->m->opt.solver==mjSOL_CG) {
    sim->figconstraint.linepnt[4] = 0;
  }
  for (i=0; i<sim->figconstraint.linepnt[0]; i++) {
    // x
    sim->figconstraint.linedata[0][2*i] = i;
    sim->figconstraint.linedata[1][2*i] = i;
    sim->figconstraint.linedata[2][2*i] = i;
    sim->figconstraint.linedata[3][2*i] = i;
    sim->figconstraint.linedata[4][2*i] = i;

    // y
    sim->figconstraint.linedata[0][2*i+1] = sim->d->nefc;
    sim->figconstraint.linedata[1][2*i+1] = sim->d->solver[i].nactive;
    sim->figconstraint.linedata[2][2*i+1] = sim->d->solver[i].nchange;
    sim->figconstraint.linedata[3][2*i+1] = sim->d->solver[i].neval;
    sim->figconstraint.linedata[4][2*i+1] = sim->d->solver[i].nupdate;
  }

  // update cost figure
  sim->figcost.linepnt[0] = mjMIN(mjMIN(sim->d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
  for (i=1; i<3; i++) {
    sim->figcost.linepnt[i] = sim->figcost.linepnt[0];
  }
  if (sim->m->opt.solver==mjSOL_PGS) {
    sim->figcost.linepnt[1] = 0;
    sim->figcost.linepnt[2] = 0;
  }

  for (i=0; i<sim->figcost.linepnt[0]; i++) {
    // x
    sim->figcost.linedata[0][2*i] = i;
    sim->figcost.linedata[1][2*i] = i;
    sim->figcost.linedata[2][2*i] = i;

    // y
    sim->figcost.linedata[0][2*i+1] = mju_log10(mju_max(mjMINVAL, sim->d->solver[i].improvement));
    sim->figcost.linedata[1][2*i+1] = mju_log10(mju_max(mjMINVAL, sim->d->solver[i].gradient));
    sim->figcost.linedata[2][2*i+1] = mju_log10(mju_max(mjMINVAL, sim->d->solver[i].lineslope));
  }

  // get timers: total, collision, prepare, solve, other
  mjtNum total = sim->d->timer[mjTIMER_STEP].duration;
  int number = sim->d->timer[mjTIMER_STEP].number;
  if (!number) {
    total = sim->d->timer[mjTIMER_FORWARD].duration;
    number = sim->d->timer[mjTIMER_FORWARD].number;
  }
  number = mjMAX(1, number);
  float tdata[5] = {
    static_cast<float>(total/number),
    static_cast<float>(sim->d->timer[mjTIMER_POS_COLLISION].duration/number),
    static_cast<float>(sim->d->timer[mjTIMER_POS_MAKE].duration/number) +
    static_cast<float>(sim->d->timer[mjTIMER_POS_PROJECT].duration/number),
    static_cast<float>(sim->d->timer[mjTIMER_CONSTRAINT].duration/number),
    0
  };
  tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

  // update figtimer
  int pnt = mjMIN(201, sim->figtimer.linepnt[0]+1);
  for (n=0; n<5; n++) {
    // shift data
    for (i=pnt-1; i>0; i--) {
      sim->figtimer.linedata[n][2*i+1] = sim->figtimer.linedata[n][2*i-1];
    }

    // assign new
    sim->figtimer.linepnt[n] = pnt;
    sim->figtimer.linedata[n][1] = tdata[n];
  }

  // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
  float sdata[6] = {
    static_cast<float>(sim->m->nv),
    static_cast<float>(sim->m->nbody),
    static_cast<float>(sim->d->nefc),
    static_cast<float>(mju_sqrt(sim->d->solver_nnz)),
    static_cast<float>(sim->d->ncon),
    static_cast<float>(sim->d->solver_iter)
  };

  // update figsize
  pnt = mjMIN(201, sim->figsize.linepnt[0]+1);
  for (n=0; n<6; n++) {
    // shift data
    for (i=pnt-1; i>0; i--) {
      sim->figsize.linedata[n][2*i+1] = sim->figsize.linedata[n][2*i-1];
    }

    // assign new
    sim->figsize.linepnt[n] = pnt;
    sim->figsize.linedata[n][1] = sdata[n];
  }
}

// show profiler figures
void profilershow(mj::Simulate* sim, mjrRect rect) {
  mjrRect viewport = {
    rect.left + rect.width - rect.width/4,
    rect.bottom,
    rect.width/4,
    rect.height/4
  };
  mjr_figure(viewport, &sim->figtimer, &sim->con);
  viewport.bottom += rect.height/4;
  mjr_figure(viewport, &sim->figsize, &sim->con);
  viewport.bottom += rect.height/4;
  mjr_figure(viewport, &sim->figcost, &sim->con);
  viewport.bottom += rect.height/4;
  mjr_figure(viewport, &sim->figconstraint, &sim->con);
}


// init sensor figure
void sensorinit(mj::Simulate* sim) {
  mjvFigure& figsensor = sim->figsensor;

  // set figure to default
  mjv_defaultFigure(&figsensor);
  figsensor.figurergba[3] = 0.5f;

  // set flags
  figsensor.flg_extend = 1;
  figsensor.flg_barplot = 1;
  figsensor.flg_symmetric = 1;

  // title
  mju::strcpy_arr(figsensor.title, "Sensor data");

  // y-tick nubmer format
  mju::strcpy_arr(figsensor.yformat, "%.0f");

  // grid size
  figsensor.gridsize[0] = 2;
  figsensor.gridsize[1] = 3;

  // minimum range
  figsensor.range[0][0] = 0;
  figsensor.range[0][1] = 0;
  figsensor.range[1][0] = -1;
  figsensor.range[1][1] = 1;
}

// update sensor figure
void sensorupdate(mj::Simulate* sim) {
  mjModel* m = sim->m;
  mjvFigure& figsensor = sim->figsensor;
  static const int maxline = 10;

  // clear linepnt
  for (int i=0; i<maxline; i++) {
    figsensor.linepnt[i] = 0;
  }

  // start with line 0
  int lineid = 0;

  // loop over sensors
  for (int n=0; n<m->nsensor; n++) {
    // go to next line if type is different
    if (n>0 && m->sensor_type[n]!=m->sensor_type[n-1]) {
      lineid = mjMIN(lineid+1, maxline-1);
    }

    // get info about this sensor
    mjtNum cutoff = (m->sensor_cutoff[n]>0 ? m->sensor_cutoff[n] : 1);
    int adr = m->sensor_adr[n];
    int dim = m->sensor_dim[n];

    // data pointer in line
    int p = figsensor.linepnt[lineid];

    // fill in data for this sensor
    for (int i=0; i<dim; i++) {
      // check size
      if ((p+2*i)>=mjMAXLINEPNT/2) {
        break;
      }

      // x
      figsensor.linedata[lineid][2*p+4*i] = adr+i;
      figsensor.linedata[lineid][2*p+4*i+2] = adr+i;

      // y
      figsensor.linedata[lineid][2*p+4*i+1] = 0;
      figsensor.linedata[lineid][2*p+4*i+3] = sim->d->sensordata[adr+i]/cutoff;
    }

    // update linepnt
    figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT-1,
                                       figsensor.linepnt[lineid]+2*dim);
  }
}

// show sensor figure
void sensorshow(mj::Simulate* sim, mjrRect rect) {
  // constant width with and without profiler
  int width = sim->profiler ? rect.width/3 : rect.width/4;

  // render figure on the right
  mjrRect viewport = {
    rect.left + rect.width - width,
    rect.bottom,
    width,
    rect.height/3
  };
  mjr_figure(viewport, &sim->figsensor, &sim->con);
}

// prepare info text
void infotext(mj::Simulate* sim,
              char (&title)[mj::Simulate::kMaxFilenameLength],
              char (&content)[mj::Simulate::kMaxFilenameLength],
              double interval) {
  mjModel* m = sim->m;
  mjData* d = sim->d;
  char tmp[20];

  // compute solver error
  mjtNum solerr = 0;
  if (d->solver_iter) {
    int ind = mjMIN(d->solver_iter-1, mjNSOLVER-1);
    solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
    if (solerr==0) {
      solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
    }
  }
  solerr = mju_log10(mju_max(mjMINVAL, solerr));

  // prepare info text
  mju::strcpy_arr(title, "Time\nSize\nCPU\nSolver   \nFPS\nstack\nconbuf\nefcbuf");
  mju::sprintf_arr(content,
                   "%-9.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f",
                   d->time,
                   d->nefc, d->ncon,
                   sim->run ?
                   d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) :
                   d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
                   solerr, d->solver_iter,
                   1/interval,
                   d->maxuse_stack/(double)d->nstack,
                   d->maxuse_con/(double)m->nconmax,
                   d->maxuse_efc/(double)m->njmax);

  // add Energy if enabled
  {
    if (mjENABLED(mjENBL_ENERGY)) {
      mju::sprintf_arr(tmp, "\n%.3f", d->energy[0]+d->energy[1]);
      mju::strcat_arr(content, tmp);
      mju::strcat_arr(title, "\nEnergy");
    }

    // add FwdInv if enabled
    if (mjENABLED(mjENBL_FWDINV)) {
      mju::sprintf_arr(tmp, "\n%.1f %.1f",
                       mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
                       mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])));
      mju::strcat_arr(content, tmp);
      mju::strcat_arr(title, "\nFwdInv");
    }
  }
}

// sprintf forwarding, to avoid compiler warning in x-macro
void printfield(char (&str)[mjMAXUINAME], void* ptr) {
  mju::sprintf_arr(str, "%g", *static_cast<mjtNum*>(ptr));
}

// update watch
void watch(mj::Simulate* sim) {
  // clear
  sim->ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
  mju::strcpy_arr(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

  // prepare symbols needed by xmacro
  MJDATA_POINTERS_PREAMBLE(sim->m);

  // find specified field in mjData arrays, update value
  #define X(TYPE, NAME, NR, NC)                                                                  \
    if (!mju::strcmp_arr(#NAME, sim->field) &&                                                   \
        !mju::strcmp_arr(#TYPE, "mjtNum")) {                                                     \
      if (sim->index >= 0 && sim->index < sim->m->NR * NC) {                                     \
        printfield(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], sim->d->NAME + sim->index);  \
      } else {                                                                                   \
        mju::strcpy_arr(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid index");       \
      }                                                                                          \
      return;                                                                                    \
    }

  MJDATA_POINTERS
#undef X
}


//---------------------------------- UI construction -----------------------------------------------

// make physics section of UI
void makephysics(mj::Simulate* sim, int oldstate) {
  int i;
  mjOption& opt = sim->m->opt;

  mjuiDef defPhysics[] = {
    {mjITEM_SECTION,   "Physics",       oldstate, nullptr,           "AP"},
    {mjITEM_SELECT,    "Integrator",    2, &(opt.integrator),        "Euler\nRK4\nimplicit"},
    {mjITEM_SELECT,    "Collision",     2, &(opt.collision),         "All\nPair\nDynamic"},
    {mjITEM_SELECT,    "Cone",          2, &(opt.cone),              "Pyramidal\nElliptic"},
    {mjITEM_SELECT,    "Jacobian",      2, &(opt.jacobian),          "Dense\nSparse\nAuto"},
    {mjITEM_SELECT,    "Solver",        2, &(opt.solver),            "PGS\nCG\nNewton"},
    {mjITEM_SEPARATOR, "Algorithmic Parameters", 1},
    {mjITEM_EDITNUM,   "Timestep",      2, &(opt.timestep),          "1 0 1"},
    {mjITEM_EDITINT,   "Iterations",    2, &(opt.iterations),        "1 0 1000"},
    {mjITEM_EDITNUM,   "Tolerance",     2, &(opt.tolerance),         "1 0 1"},
    {mjITEM_EDITINT,   "Noslip Iter",   2, &(opt.noslip_iterations), "1 0 1000"},
    {mjITEM_EDITNUM,   "Noslip Tol",    2, &(opt.noslip_tolerance),  "1 0 1"},
    {mjITEM_EDITINT,   "MRR Iter",      2, &(opt.mpr_iterations),    "1 0 1000"},
    {mjITEM_EDITNUM,   "MPR Tol",       2, &(opt.mpr_tolerance),     "1 0 1"},
    {mjITEM_EDITNUM,   "API Rate",      2, &(opt.apirate),           "1 0 1000"},
    {mjITEM_SEPARATOR, "Physical Parameters", 1},
    {mjITEM_EDITNUM,   "Gravity",       2, opt.gravity,              "3"},
    {mjITEM_EDITNUM,   "Wind",          2, opt.wind,                 "3"},
    {mjITEM_EDITNUM,   "Magnetic",      2, opt.magnetic,             "3"},
    {mjITEM_EDITNUM,   "Density",       2, &(opt.density),           "1"},
    {mjITEM_EDITNUM,   "Viscosity",     2, &(opt.viscosity),         "1"},
    {mjITEM_EDITNUM,   "Imp Ratio",     2, &(opt.impratio),          "1"},
    {mjITEM_SEPARATOR, "Disable Flags", 1},
    {mjITEM_END}
  };
  mjuiDef defEnableFlags[] = {
    {mjITEM_SEPARATOR, "Enable Flags", 1},
    {mjITEM_END}
  };
  mjuiDef defOverride[] = {
    {mjITEM_SEPARATOR, "Contact Override", 1},
    {mjITEM_EDITNUM,   "Margin",        2, &(opt.o_margin),          "1"},
    {mjITEM_EDITNUM,   "Sol Imp",       2, &(opt.o_solimp),          "5"},
    {mjITEM_EDITNUM,   "Sol Ref",       2, &(opt.o_solref),          "2"},
    {mjITEM_END}
  };

  // add physics
  mjui_add(&sim->ui0, defPhysics);

  // add flags programmatically
  mjuiDef defFlag[] = {
    {mjITEM_CHECKINT,  "", 2, nullptr, ""},
    {mjITEM_END}
  };
  for (i=0; i<mjNDISABLE; i++) {
    mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
    defFlag[0].pdata = sim->disable + i;
    mjui_add(&sim->ui0, defFlag);
  }
  mjui_add(&sim->ui0, defEnableFlags);
  for (i=0; i<mjNENABLE; i++) {
    mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
    defFlag[0].pdata = sim->enable + i;
    mjui_add(&sim->ui0, defFlag);
  }

  // add contact override
  mjui_add(&sim->ui0, defOverride);
}



// make rendering section of UI
void makerendering(mj::Simulate* sim, int oldstate) {
  int i, j;

  mjuiDef defRendering[] = {
    {
      mjITEM_SECTION,
      "Rendering",
      oldstate,
      nullptr,
      "AR"
    },
    {
      mjITEM_SELECT,
      "Camera",
      2,
      &(sim->camera),
      "Free\nTracking"
    },
    {
      mjITEM_SELECT,
      "Label",
      2,
      &(sim->vopt.label),
      "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\n"
      "Actuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce"
    },
    {
      mjITEM_SELECT,
      "Frame",
      2,
      &(sim->vopt.frame),
      "None\nBody\nGeom\nSite\nCamera\nLight\nContact\nWorld"
    },
    {
      mjITEM_BUTTON,
      "Copy camera",
      2,
      nullptr,
      ""
    },
    {
      mjITEM_SEPARATOR,
      "Model Elements",
      1
    },
    {
      mjITEM_END
    }
  };
  mjuiDef defOpenGL[] = {
    {mjITEM_SEPARATOR, "OpenGL Effects", 1},
    {mjITEM_END}
  };

  // add model cameras, up to UI limit
  for (i=0; i<mjMIN(sim->m->ncam, mjMAXUIMULTI-2); i++) {
    // prepare name
    char camname[mjMAXUITEXT] = "\n";
    if (sim->m->names[sim->m->name_camadr[i]]) {
      mju::strcat_arr(camname, sim->m->names+sim->m->name_camadr[i]);
    } else {
      mju::sprintf_arr(camname, "\nCamera %d", i);
    }

    // check string length
    if (mju::strlen_arr(camname) + mju::strlen_arr(defRendering[1].other)>=mjMAXUITEXT-1) {
      break;
    }

    // add camera
    mju::strcat_arr(defRendering[1].other, camname);
  }

  // add rendering standard
  mjui_add(&sim->ui0, defRendering);

  // add flags programmatically
  mjuiDef defFlag[] = {
    {mjITEM_CHECKBYTE,  "", 2, nullptr, ""},
    {mjITEM_END}
  };
  for (i=0; i<mjNVISFLAG; i++) {
    // set name, remove "&"
    mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);
    for (j=0; j<strlen(mjVISSTRING[i][0]); j++)
      if (mjVISSTRING[i][0][j]=='&') {
        mju_strncpy(
          defFlag[0].name+j, mjVISSTRING[i][0]+j+1, mju::sizeof_arr(defFlag[0].name)-j);
        break;
      }

    // set shortcut and data
    mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
    defFlag[0].pdata = sim->vopt.flags + i;
    mjui_add(&sim->ui0, defFlag);
  }
  mjui_add(&sim->ui0, defOpenGL);
  for (i=0; i<mjNRNDFLAG; i++) {
    mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);
    if (mjRNDSTRING[i][2][0]) {
      mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
    }
    defFlag[0].pdata = sim->scn.flags + i;
    mjui_add(&sim->ui0, defFlag);
  }
}



// make group section of UI
void makegroup(mj::Simulate* sim, int oldstate) {
  mjvOption& vopt = sim->vopt;
  mjuiDef defGroup[] = {
    {mjITEM_SECTION,    "Group enable",     oldstate, nullptr,          "AG"},
    {mjITEM_SEPARATOR,  "Geom groups",  1},
    {mjITEM_CHECKBYTE,  "Geom 0",           2, vopt.geomgroup,          " 0"},
    {mjITEM_CHECKBYTE,  "Geom 1",           2, vopt.geomgroup+1,        " 1"},
    {mjITEM_CHECKBYTE,  "Geom 2",           2, vopt.geomgroup+2,        " 2"},
    {mjITEM_CHECKBYTE,  "Geom 3",           2, vopt.geomgroup+3,        " 3"},
    {mjITEM_CHECKBYTE,  "Geom 4",           2, vopt.geomgroup+4,        " 4"},
    {mjITEM_CHECKBYTE,  "Geom 5",           2, vopt.geomgroup+5,        " 5"},
    {mjITEM_SEPARATOR,  "Site groups",  1},
    {mjITEM_CHECKBYTE,  "Site 0",           2, vopt.sitegroup,          "S0"},
    {mjITEM_CHECKBYTE,  "Site 1",           2, vopt.sitegroup+1,        "S1"},
    {mjITEM_CHECKBYTE,  "Site 2",           2, vopt.sitegroup+2,        "S2"},
    {mjITEM_CHECKBYTE,  "Site 3",           2, vopt.sitegroup+3,        "S3"},
    {mjITEM_CHECKBYTE,  "Site 4",           2, vopt.sitegroup+4,        "S4"},
    {mjITEM_CHECKBYTE,  "Site 5",           2, vopt.sitegroup+5,        "S5"},
    {mjITEM_SEPARATOR,  "Joint groups", 1},
    {mjITEM_CHECKBYTE,  "Joint 0",          2, vopt.jointgroup,         ""},
    {mjITEM_CHECKBYTE,  "Joint 1",          2, vopt.jointgroup+1,       ""},
    {mjITEM_CHECKBYTE,  "Joint 2",          2, vopt.jointgroup+2,       ""},
    {mjITEM_CHECKBYTE,  "Joint 3",          2, vopt.jointgroup+3,       ""},
    {mjITEM_CHECKBYTE,  "Joint 4",          2, vopt.jointgroup+4,       ""},
    {mjITEM_CHECKBYTE,  "Joint 5",          2, vopt.jointgroup+5,       ""},
    {mjITEM_SEPARATOR,  "Tendon groups",    1},
    {mjITEM_CHECKBYTE,  "Tendon 0",         2, vopt.tendongroup,        ""},
    {mjITEM_CHECKBYTE,  "Tendon 1",         2, vopt.tendongroup+1,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 2",         2, vopt.tendongroup+2,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 3",         2, vopt.tendongroup+3,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 4",         2, vopt.tendongroup+4,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 5",         2, vopt.tendongroup+5,      ""},
    {mjITEM_SEPARATOR,  "Actuator groups", 1},
    {mjITEM_CHECKBYTE,  "Actuator 0",       2, vopt.actuatorgroup,      ""},
    {mjITEM_CHECKBYTE,  "Actuator 1",       2, vopt.actuatorgroup+1,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 2",       2, vopt.actuatorgroup+2,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 3",       2, vopt.actuatorgroup+3,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 4",       2, vopt.actuatorgroup+4,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 5",       2, vopt.actuatorgroup+5,    ""},
    {mjITEM_SEPARATOR,  "Skin groups", 1},
    {mjITEM_CHECKBYTE,  "Skin 0",           2, vopt.skingroup,          ""},
    {mjITEM_CHECKBYTE,  "Skin 1",           2, vopt.skingroup+1,        ""},
    {mjITEM_CHECKBYTE,  "Skin 2",           2, vopt.skingroup+2,        ""},
    {mjITEM_CHECKBYTE,  "Skin 3",           2, vopt.skingroup+3,        ""},
    {mjITEM_CHECKBYTE,  "Skin 4",           2, vopt.skingroup+4,        ""},
    {mjITEM_CHECKBYTE,  "Skin 5",           2, vopt.skingroup+5,        ""},
    {mjITEM_END}
  };

  // add section
  mjui_add(&sim->ui0, defGroup);
}

// make joint section of UI
void makejoint(mj::Simulate* sim, int oldstate) {
  int i;

  mjuiDef defJoint[] = {
    {mjITEM_SECTION, "Joint", oldstate, nullptr, "AJ"},
    {mjITEM_END}
  };
  mjuiDef defSlider[] = {
    {mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"},
    {mjITEM_END}
  };

  // add section
  mjui_add(&sim->ui1, defJoint);
  defSlider[0].state = 4;

  // add scalar joints, exit if UI limit reached
  int itemcnt = 0;
  for (i=0; i<sim->m->njnt && itemcnt<mjMAXUIITEM; i++)
    if ((sim->m->jnt_type[i]==mjJNT_HINGE || sim->m->jnt_type[i]==mjJNT_SLIDE)) {
      // skip if joint group is disabled
      if (!sim->vopt.jointgroup[mjMAX(0, mjMIN(mjNGROUP-1, sim->m->jnt_group[i]))]) {
        continue;
      }

      // set data and name
      defSlider[0].pdata = sim->d->qpos + sim->m->jnt_qposadr[i];
      if (sim->m->names[sim->m->name_jntadr[i]]) {
        mju::strcpy_arr(defSlider[0].name, sim->m->names+sim->m->name_jntadr[i]);
      } else {
        mju::sprintf_arr(defSlider[0].name, "joint %d", i);
      }

      // set range
      if (sim->m->jnt_limited[i])
        mju::sprintf_arr(defSlider[0].other, "%.4g %.4g",
                         sim->m->jnt_range[2*i], sim->m->jnt_range[2*i+1]);
      else if (sim->m->jnt_type[i]==mjJNT_SLIDE) {
        mju::strcpy_arr(defSlider[0].other, "-1 1");
      } else {
        mju::strcpy_arr(defSlider[0].other, "-3.1416 3.1416");
      }

      // add and count
      mjui_add(&sim->ui1, defSlider);
      itemcnt++;
    }
}

// make control section of UI
void makecontrol(mj::Simulate* sim, int oldstate) {
  int i;

  mjuiDef defControl[] = {
    {mjITEM_SECTION, "Control", oldstate, nullptr, "AC"},
    {mjITEM_BUTTON,  "Clear all", 2},
    {mjITEM_END}
  };
  mjuiDef defSlider[] = {
    {mjITEM_SLIDERNUM, "", 2, nullptr, "0 1"},
    {mjITEM_END}
  };

  // add section
  mjui_add(&sim->ui1, defControl);
  defSlider[0].state = 2;

  // add controls, exit if UI limit reached (Clear button already added)
  int itemcnt = 1;
  for (i=0; i<sim->m->nu && itemcnt<mjMAXUIITEM; i++) {
    // skip if actuator group is disabled
    if (!sim->vopt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP-1, sim->m->actuator_group[i]))]) {
      continue;
    }

    // set data and name
    defSlider[0].pdata = sim->d->ctrl + i;
    if (sim->m->names[sim->m->name_actuatoradr[i]]) {
      mju::strcpy_arr(defSlider[0].name, sim->m->names+sim->m->name_actuatoradr[i]);
    } else {
      mju::sprintf_arr(defSlider[0].name, "control %d", i);
    }

    // set range
    if (sim->m->actuator_ctrllimited[i])
      mju::sprintf_arr(defSlider[0].other, "%.4g %.4g",
                       sim->m->actuator_ctrlrange[2*i], sim->m->actuator_ctrlrange[2*i+1]);
    else {
      mju::strcpy_arr(defSlider[0].other, "-1 1");
    }

    // add and count
    mjui_add(&sim->ui1, defSlider);
    itemcnt++;
  }
}

// make model-dependent UI sections
void makesections(mj::Simulate* sim) {
  int i;

  // get section open-close state, UI 0
  int oldstate0[NSECT0];
  for (i=0; i<NSECT0; i++) {
    oldstate0[i] = 0;
    if (sim->ui0.nsect>i) {
      oldstate0[i] = sim->ui0.sect[i].state;
    }
  }

  // get section open-close state, UI 1
  int oldstate1[NSECT1];
  for (i=0; i<NSECT1; i++) {
    oldstate1[i] = 0;
    if (sim->ui1.nsect>i) {
      oldstate1[i] = sim->ui1.sect[i].state;
    }
  }

  // clear model-dependent sections of UI
  sim->ui0.nsect = SECT_PHYSICS;
  sim->ui1.nsect = 0;

  // make
  makephysics(sim, oldstate0[SECT_PHYSICS]);
  makerendering(sim, oldstate0[SECT_RENDERING]);
  makegroup(sim, oldstate0[SECT_GROUP]);
  makejoint(sim, oldstate1[SECT_JOINT]);
  makecontrol(sim, oldstate1[SECT_CONTROL]);
}

//---------------------------------- utility functions ---------------------------------------------

// align and scale view
void alignscale(mj::Simulate* sim) {
  // use default free camera parameters
  mjv_defaultFreeCamera(sim->m, &sim->cam);
}


// copy qpos to clipboard as key
void copykey(mj::Simulate* sim) {
  char clipboard[5000] = "<key qpos='";
  char buf[200];

  // prepare string
  for (int i=0; i<sim->m->nq; i++) {
    mju::sprintf_arr(buf, i==sim->m->nq-1 ? "%g" : "%g ", sim->d->qpos[i]);
    mju::strcat_arr(clipboard, buf);
  }
  mju::strcat_arr(clipboard, "'/>");

  // copy to clipboard
  Glfw().glfwSetClipboardString(sim->window, clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer() {
  return 1000*Glfw().glfwGetTime();
}

// clear all times
void cleartimers(mjData* d) {
  for (int i=0; i<mjNTIMER; i++) {
    d->timer[i].duration = 0;
    d->timer[i].number = 0;
  }
}

// copy current camera to clipboard as MJCF specification
void copycamera(mj::Simulate* sim) {
  mjvGLCamera* camera = sim->scn.camera;

  char clipboard[500];
  mjtNum cam_right[3];
  mjtNum cam_forward[3];
  mjtNum cam_up[3];

  // get camera spec from the GLCamera
  mju_f2n(cam_forward, camera[0].forward, 3);
  mju_f2n(cam_up, camera[0].up, 3);
  mju_cross(cam_right, cam_forward, cam_up);

  // make MJCF camera spec
  mju::sprintf_arr(clipboard,
                   "<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f %.3f %.3f\"/>\n",
                   (camera[0].pos[0] + camera[1].pos[0]) / 2,
                   (camera[0].pos[1] + camera[1].pos[1]) / 2,
                   (camera[0].pos[2] + camera[1].pos[2]) / 2,
                   cam_right[0], cam_right[1], cam_right[2],
                   camera[0].up[0], camera[0].up[1], camera[0].up[2]);

  // copy spec into clipboard
  Glfw().glfwSetClipboardString(sim->window, clipboard);
}

// update UI 0 when MuJoCo structures change (except for joint sliders)
void updatesettings(mj::Simulate* sim) {
  int i;

  // physics flags
  for (i=0; i<mjNDISABLE; i++) {
    sim->disable[i] = ((sim->m->opt.disableflags & (1<<i)) !=0);
  }
  for (i=0; i<mjNENABLE; i++) {
    sim->enable[i] = ((sim->m->opt.enableflags & (1<<i)) !=0);
  }

  // camera
  if (sim->cam.type==mjCAMERA_FIXED) {
    sim->camera = 2 + sim->cam.fixedcamid;
  } else if (sim->cam.type==mjCAMERA_TRACKING) {
    sim->camera = 1;
  } else {
    sim->camera = 0;
  }

  // update UI
  mjui_update(-1, -1, &sim->ui0, &sim->uistate, &sim->con);
}


//---------------------------------- UI hooks (for uitools.c) --------------------------------------

// determine enable/disable item state given category
int uiPredicate(int category, void* userdata) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(userdata);

  switch (category) {
  case 2:                 // require model
    return (sim->m != nullptr);

  case 3:                 // require model and nkey
    return (sim->m && sim->m->nkey);

  case 4:                 // require model and paused
    return (sim->m && !sim->run);

  default:
    return 1;
  }
}

// set window layout
void uiLayout(mjuiState* state) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(state->userdata);
  mjrRect* rect = state->rect;

  // set number of rectangles
  state->nrect = 4;

  // rect 0: entire framebuffer
  rect[0].left = 0;
  rect[0].bottom = 0;
  Glfw().glfwGetFramebufferSize(sim->window, &rect[0].width, &rect[0].height);

  // rect 1: UI 0
  rect[1].left = 0;
  rect[1].width = sim->ui0_enable ? sim->ui0.width : 0;
  rect[1].bottom = 0;
  rect[1].height = rect[0].height;

  // rect 2: UI 1
  rect[2].width = sim->ui1_enable ? sim->ui1.width : 0;
  rect[2].left = mjMAX(0, rect[0].width - rect[2].width);
  rect[2].bottom = 0;
  rect[2].height = rect[0].height;

  // rect 3: 3D plot (everything else is an overlay)
  rect[3].left = rect[1].width;
  rect[3].width = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
  rect[3].bottom = 0;
  rect[3].height = rect[0].height;
}

// handle UI event
void uiEvent(mjuiState* state) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(state->userdata);
  mjModel* m = sim->m;
  mjData* d = sim->d;
  int i;
  char err[200];

  // call UI 0 if event is directed to it
  if ((state->dragrect==sim->ui0.rectid) ||
      (state->dragrect==0 && state->mouserect==sim->ui0.rectid) ||
      state->type==mjEVENT_KEY) {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui0, state, &sim->con);

    // file section
    if (it && it->sectionid==SECT_FILE) {
      switch (it->itemid) {
      case 0:             // Save xml
        {
          const std::string path = getSavePath("mjmodel.xml");
          if (!path.empty() && !mj_saveLastXML(path.c_str(), m, err, 200)) {
            std::printf("Save XML error: %s", err);
          }
        }
        break;

      case 1:             // Save mjb
        {
          const std::string path = getSavePath("mjmodel.mjb");
          if (!path.empty()) {
            mj_saveModel(m, path.c_str(), NULL, 0);
          }
        }
        break;

      case 2:             // Print model
        mj_printModel(m, "MJMODEL.TXT");
        break;

      case 3:             // Print data
        mj_printData(m, d, "MJDATA.TXT");
        break;

      case 4:             // Quit
        sim->exitrequest.store(1);
        break;

      case 5:             // Screenshot
        sim->screenshotrequest.store(true);
        break;
      }
    }

    // option section
    else if (it && it->sectionid==SECT_OPTION) {
      switch (it->itemid) {
      case 0:             // Spacing
        sim->ui0.spacing = mjui_themeSpacing(sim->spacing);
        sim->ui1.spacing = mjui_themeSpacing(sim->spacing);
        break;

      case 1:             // Color
        sim->ui0.color = mjui_themeColor(sim->color);
        sim->ui1.color = mjui_themeColor(sim->color);
        break;

      case 2:             // Font
        mjr_changeFont(50*(sim->font+1), &sim->con);
        break;

      case 9:             // Full screen
        if (Glfw().glfwGetWindowMonitor(sim->window)) {
          // restore window from saved data
          Glfw().glfwSetWindowMonitor(sim->window, nullptr, sim->windowpos[0], sim->windowpos[1],
                                      sim->windowsize[0], sim->windowsize[1], 0);
        }

        // currently windowed: switch to full screen
        else {
          // save window data
          Glfw().glfwGetWindowPos(sim->window, sim->windowpos, sim->windowpos+1);
          Glfw().glfwGetWindowSize(sim->window, sim->windowsize, sim->windowsize+1);

          // switch
          Glfw().glfwSetWindowMonitor(sim->window, Glfw().glfwGetPrimaryMonitor(), 0, 0,
                                      sim->vmode.width, sim->vmode.height, sim->vmode.refreshRate);
        }

        // reinstante vsync, just in case
        Glfw().glfwSwapInterval(sim->vsync);
        break;

      case 10:            // Vertical sync
        Glfw().glfwSwapInterval(sim->vsync);
        break;
      }

      // modify UI
      uiModify(sim->window, &sim->ui0, state, &sim->con);
      uiModify(sim->window, &sim->ui1, state, &sim->con);
    }

    // simulation section
    else if (it && it->sectionid==SECT_SIMULATION) {
      switch (it->itemid) {
      case 1:             // Reset
        if (m) {
          mj_resetData(m, d);
          mj_forward(m, d);
          profilerupdate(sim);
          sensorupdate(sim);
          updatesettings(sim);
        }
        break;

      case 2:             // Reload
        sim->uiloadrequest.fetch_add(1);
        break;

      case 3:             // Align
        alignscale(sim);
        updatesettings(sim);
        break;

      case 4:             // Copy pose
        copykey(sim);
        break;

      case 5:             // Adjust key
      case 6:             // Load key
        i = sim->key;
        d->time = m->key_time[i];
        mju_copy(d->qpos, m->key_qpos+i*m->nq, m->nq);
        mju_copy(d->qvel, m->key_qvel+i*m->nv, m->nv);
        mju_copy(d->act, m->key_act+i*m->na, m->na);
        mju_copy(d->mocap_pos, m->key_mpos+i*3*m->nmocap, 3*m->nmocap);
        mju_copy(d->mocap_quat, m->key_mquat+i*4*m->nmocap, 4*m->nmocap);
        mju_copy(d->ctrl, m->key_ctrl+i*m->nu, m->nu);
        mj_forward(m, d);
        profilerupdate(sim);
        sensorupdate(sim);
        updatesettings(sim);
        break;

      case 7:             // Save key
        i = sim->key;
        m->key_time[i] = d->time;
        mju_copy(m->key_qpos+i*m->nq, d->qpos, m->nq);
        mju_copy(m->key_qvel+i*m->nv, d->qvel, m->nv);
        mju_copy(m->key_act+i*m->na, d->act, m->na);
        mju_copy(m->key_mpos+i*3*m->nmocap, d->mocap_pos, 3*m->nmocap);
        mju_copy(m->key_mquat+i*4*m->nmocap, d->mocap_quat, 4*m->nmocap);
        mju_copy(m->key_ctrl+i*m->nu, d->ctrl, m->nu);
        break;
      }
    }

    // physics section
    else if (it && it->sectionid==SECT_PHYSICS) {
      // update disable flags in mjOption
      m->opt.disableflags = 0;
      for (i=0; i<mjNDISABLE; i++)
        if (sim->disable[i]) {
          m->opt.disableflags |= (1<<i);
        }

      // update enable flags in mjOption
      m->opt.enableflags = 0;
      for (i=0; i<mjNENABLE; i++)
        if (sim->enable[i]) {
          m->opt.enableflags |= (1<<i);
        }
    }

    // rendering section
    else if (it && it->sectionid==SECT_RENDERING) {
      // set camera in mjvCamera
      if (sim->camera==0) {
        sim->cam.type = mjCAMERA_FREE;
      } else if (sim->camera==1) {
        if (sim->pert.select>0) {
          sim->cam.type = mjCAMERA_TRACKING;
          sim->cam.trackbodyid = sim->pert.select;
          sim->cam.fixedcamid = -1;
        } else {
          sim->cam.type = mjCAMERA_FREE;
          sim->camera = 0;
          mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
        }
      } else {
        sim->cam.type = mjCAMERA_FIXED;
        sim->cam.fixedcamid = sim->camera - 2;
      }
      // copy camera spec to clipboard (as MJCF element)
      if (it->itemid == 3) {
        copycamera(sim);
      }
    }

    // group section
    else if (it && it->sectionid==SECT_GROUP) {
      // remake joint section if joint group changed
      if (it->name[0]=='J' && it->name[1]=='o') {
        sim->ui1.nsect = SECT_JOINT;
        makejoint(sim, sim->ui1.sect[SECT_JOINT].state);
        sim->ui1.nsect = NSECT1;
        uiModify(sim->window, &sim->ui1, state, &sim->con);
      }

      // remake control section if actuator group changed
      if (it->name[0]=='A' && it->name[1]=='c') {
        sim->ui1.nsect = SECT_CONTROL;
        makecontrol(sim, sim->ui1.sect[SECT_CONTROL].state);
        sim->ui1.nsect = NSECT1;
        uiModify(sim->window, &sim->ui1, state, &sim->con);
      }
    }

    // stop if UI processed event
    if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
      return;
    }
  }

  // call UI 1 if event is directed to it
  if ((state->dragrect==sim->ui1.rectid) ||
      (state->dragrect==0 && state->mouserect==sim->ui1.rectid) ||
      state->type==mjEVENT_KEY) {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui1, state, &sim->con);

    // control section
    if (it && it->sectionid==SECT_CONTROL) {
      // clear controls
      if (it->itemid==0) {
        mju_zero(d->ctrl, m->nu);
        mjui_update(SECT_CONTROL, -1, &sim->ui1, &sim->uistate, &sim->con);
      }
    }

    // stop if UI processed event
    if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
      return;
    }
  }

  // shortcut not handled by UI
  if (state->type==mjEVENT_KEY && state->key!=0) {
    switch (state->key) {
    case ' ':                   // Mode
      if (m) {
        sim->run = 1 - sim->run;
        sim->pert.active = 0;
        mjui_update(-1, -1, &sim->ui0, state, &sim->con);
      }
      break;

    case mjKEY_RIGHT:           // step forward
      if (m && !sim->run) {
        cleartimers(d);
        mj_step(m, d);
        profilerupdate(sim);
        sensorupdate(sim);
        updatesettings(sim);
      }
      break;

    case mjKEY_PAGE_UP:         // select parent body
      if (m && sim->pert.select>0) {
        sim->pert.select = m->body_parentid[sim->pert.select];
        sim->pert.skinselect = -1;

        // stop perturbation if world reached
        if (sim->pert.select<=0) {
          sim->pert.active = 0;
        }
      }

      break;

    case ']':                   // cycle up fixed cameras
      if (m && m->ncam) {
        sim->cam.type = mjCAMERA_FIXED;
        // simulate->camera = {0 or 1} are reserved for the free and tracking cameras
        if (sim->camera < 2 || sim->camera == 2 + m->ncam-1) {
          sim->camera = 2;
        } else {
          sim->camera += 1;
        }
        sim->cam.fixedcamid = sim->camera - 2;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
      }
      break;

    case '[':                   // cycle down fixed cameras
      if (m && m->ncam) {
        sim->cam.type = mjCAMERA_FIXED;
        // settings.camera = {0 or 1} are reserved for the free and tracking cameras
        if (sim->camera <= 2) {
          sim->camera = 2 + m->ncam-1;
        } else {
          sim->camera -= 1;
        }
        sim->cam.fixedcamid = sim->camera - 2;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
      }
      break;

    case mjKEY_F6:                   // cycle frame visualisation
      if (m) {
        sim->vopt.frame = (sim->vopt.frame + 1) % mjNFRAME;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
      }
      break;

    case mjKEY_F7:                   // cycle label visualisation
      if (m) {
        sim->vopt.label = (sim->vopt.label + 1) % mjNLABEL;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
      }
      break;

    case mjKEY_ESCAPE:          // free camera
      sim->cam.type = mjCAMERA_FREE;
      sim->camera = 0;
      mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
      break;

    case '-':                   // slow down
      {
        int numclicks = sizeof(sim->percentRealTime) / sizeof(sim->percentRealTime[0]);
        if (sim->realTimeIndex < numclicks-1 && !state->shift) {
          sim->realTimeIndex++;
          sim->speedChanged = true;
        }
      }
      break;

    case '=':                   // speed up
      if (sim->realTimeIndex > 0 && !state->shift) {
        sim->realTimeIndex--;
        sim->speedChanged = true;
      }
      break;
    }

    return;
  }

  // 3D scroll
  if (state->type==mjEVENT_SCROLL && state->mouserect==3 && m) {
    // emulate vertical mouse motion = 2% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -zoom_increment*state->sy, &sim->scn, &sim->cam);

    return;
  }

  // 3D press
  if (state->type==mjEVENT_PRESS && state->mouserect==3 && m) {
    // set perturbation
    int newperturb = 0;
    if (state->control && sim->pert.select>0) {
      // right: translate;  left: rotate
      if (state->right) {
        newperturb = mjPERT_TRANSLATE;
      } else if (state->left) {
        newperturb = mjPERT_ROTATE;
      }

      // perturbation onset: reset reference
      if (newperturb && !sim->pert.active) {
        mjv_initPerturb(m, d, &sim->scn, &sim->pert);
      }
    }
    sim->pert.active = newperturb;

    // handle double-click
    if (state->doubleclick) {
      // determine selection mode
      int selmode;
      if (state->button==mjBUTTON_LEFT) {
        selmode = 1;
      } else if (state->control) {
        selmode = 3;
      } else {
        selmode = 2;
      }

      // find geom and 3D click point, get corresponding body
      mjrRect r = state->rect[3];
      mjtNum selpnt[3];
      int selgeom, selskin;
      int selbody = mjv_select(m, d, &sim->vopt,
                               static_cast<mjtNum>(r.width)/r.height,
                               (state->x - r.left)/r.width,
                               (state->y - r.bottom)/r.height,
                               &sim->scn, selpnt, &selgeom, &selskin);

      // set lookat point, start tracking is requested
      if (selmode==2 || selmode==3) {
        // copy selpnt if anything clicked
        if (selbody>=0) {
          mju_copy3(sim->cam.lookat, selpnt);
        }

        // switch to tracking camera if dynamic body clicked
        if (selmode==3 && selbody>0) {
          // mujoco camera
          sim->cam.type = mjCAMERA_TRACKING;
          sim->cam.trackbodyid = selbody;
          sim->cam.fixedcamid = -1;

          // UI camera
          sim->camera = 1;
          mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->con);
        }
      }

      // set body selection
      else {
        if (selbody>=0) {
          // record selection
          sim->pert.select = selbody;
          sim->pert.skinselect = selskin;

          // compute localpos
          mjtNum tmp[3];
          mju_sub3(tmp, selpnt, d->xpos+3*sim->pert.select);
          mju_mulMatTVec(sim->pert.localpos, d->xmat+9*sim->pert.select, tmp, 3, 3);
        } else {
          sim->pert.select = 0;
          sim->pert.skinselect = -1;
        }
      }

      // stop perturbation on select
      sim->pert.active = 0;
    }

    return;
  }

  // 3D release
  if (state->type==mjEVENT_RELEASE && state->dragrect==3 && m) {
    // stop perturbation
    sim->pert.active = 0;

    return;
  }

  // 3D move
  if (state->type==mjEVENT_MOVE && state->dragrect==3 && m) {
    // determine action based on mouse button
    mjtMouse action;
    if (state->right) {
      action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (state->left) {
      action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
      action = mjMOUSE_ZOOM;
    }

    // move perturb or camera
    mjrRect r = state->rect[3];
    if (sim->pert.active)
      mjv_movePerturb(m, d, action, state->dx/r.height, -state->dy/r.height,
                      &sim->scn, &sim->pert);
    else
      mjv_moveCamera(m, action, state->dx/r.height, -state->dy/r.height,
                     &sim->scn, &sim->cam);

    return;
  }
}

void uiRender(mjuiState* state) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(state->userdata);
  sim->render();
}

// drop file callback
void drop(mj::Simulate* sim, int count, const char** paths) {
  // make sure list is non-empty
  if (count>0) {
    while (sim->droploadrequest.load()) {}
    mju::strcpy_arr(sim->dropfilename, paths[0]);
    sim->droploadrequest.store(true);
  }
}

void uiDrop(mjuiState* state, int count, const char** paths) {
  mj::Simulate* simulate = static_cast<mj::Simulate*>(state->userdata);
  drop(simulate, count, paths);
}

}  // namespace

namespace mujoco {
namespace mju = ::mujoco::sample_util;

//------------------------------------ apply pose perturbations ------------------------------------
void Simulate::applyposepertubations(int flg_paused) {
  if (this->m != nullptr) {
    mjv_applyPerturbPose(this->m, this->d, &this->pert, flg_paused);  // move mocap bodies only
  }
}

//----------------------------------- apply force perturbations ------------------------------------
void Simulate::applyforceperturbations() {
  if (this->m != nullptr) {
    mjv_applyPerturbForce(this->m, this->d, &this->pert);
  }
}

//------------------------- Tell the render thread to load a file and wait -------------------------
void Simulate::load(const char* file,
                    mjModel* mnew,
                    mjData* dnew,
                    bool delete_old_m_d) {
  this->mnew = mnew;
  this->dnew = dnew;
  this->delete_old_m_d = delete_old_m_d;
  mju::strcpy_arr(this->filename, file);

  {
    std::unique_lock<std::mutex> lock(mtx);
    this->loadrequest = 2;

    // Wait for the render thread to be done loading
    // so that we know the old model and data's memory can
    // be free'd by the other thread (sometimes python)
    cond_loadrequest.wait(lock, [this]() { return this->loadrequest == 0; });
  }
}

//------------------------------------- load mjb or xml model --------------------------------------
void Simulate::loadmodel() {
  if (this->delete_old_m_d) {
    // delete old model if requested
    if (this->d) {
      mj_deleteData(d);
    }
    if (this->m) {
      mj_deleteModel(m);
    }
  }

  this->m = this->mnew;
  this->d = this->dnew;

  // re-create scene and context
  mjv_makeScene(this->m, &this->scn, maxgeom);
  mjr_makeContext(this->m, &this->con, 50*(this->font+1));

  // clear perturbation state
  this->pert.active = 0;
  this->pert.select = 0;
  this->pert.skinselect = -1;

  // align and scale view unless reloading the same file
  if (mju::strcmp_arr(this->filename, this->previous_filename)) {
    alignscale(this);
    mju::strcpy_arr(this->previous_filename, this->filename);
  }

  // update scene
  mjv_updateScene(this->m, this->d, &this->vopt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);

  // set window title to model name
  if (this->window && this->m->names) {
    char title[200] = "Simulate : ";
    mju::strcat_arr(title, this->m->names);
    Glfw().glfwSetWindowTitle(this->window, title);
  }

  // set keyframe range and divisions
  this->ui0.sect[SECT_SIMULATION].item[5].slider.range[0] = 0;
  this->ui0.sect[SECT_SIMULATION].item[5].slider.range[1] = mjMAX(0, this->m->nkey - 1);
  this->ui0.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, this->m->nkey - 1);

  // rebuild UI sections
  makesections(this);

  // full ui update
  uiModify(this->window, &this->ui0, &this->uistate, &this->con);
  uiModify(this->window, &this->ui1, &this->uistate, &this->con);
  updatesettings(this);

  // clear request
  this->loadrequest = 0;
  cond_loadrequest.notify_all();
}


//------------------------------------------- rendering --------------------------------------------


// prepare to render
void Simulate::prepare() {
  // data for FPS calculation
  static double lastupdatetm = 0;

  // update interval, save update time
  double tmnow = Glfw().glfwGetTime();
  double interval = tmnow - lastupdatetm;
  interval = mjMIN(1, mjMAX(0.0001, interval));
  lastupdatetm = tmnow;

  // no model: nothing to do
  if (!this->m) {
    return;
  }

  // update scene
  mjv_updateScene(this->m, this->d, &this->vopt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);

  // update watch
  if (this->ui0_enable && this->ui0.sect[SECT_WATCH].state) {
    watch(this);
    mjui_update(SECT_WATCH, -1, &this->ui0, &this->uistate, &this->con);
  }

  // update joint
  if (this->ui1_enable && this->ui1.sect[SECT_JOINT].state) {
    mjui_update(SECT_JOINT, -1, &this->ui1, &this->uistate, &this->con);
  }

  // update info text
  if (this->info) {
    infotext(this, this->info_title, this->info_content, interval);
  }

  // update control
  if (this->ui1_enable && this->ui1.sect[SECT_CONTROL].state) {
    mjui_update(SECT_CONTROL, -1, &this->ui1, &this->uistate, &this->con);
  }

  // update profiler
  if (this->profiler && this->run) {
    profilerupdate(this);
  }

  // update sensor
  if (this->sensor && this->run) {
    sensorupdate(this);
  }

  // clear timers once profiler info has been copied
  cleartimers(this->d);
}

// render the ui to the window
void Simulate::render() {
  // get 3D rectangle and reduced for profiler
  mjrRect rect = this->uistate.rect[3];
  mjrRect smallrect = rect;
  if (this->profiler) {
    smallrect.width = rect.width - rect.width/4;
  }

  // no model
  if (!this->m) {
    // blank screen
    mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

    // label
    if (this->loadrequest) {
      mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, "loading", nullptr, &this->con);
    } else {
      char intro_message[Simulate::kMaxFilenameLength];
      mju::sprintf_arr(intro_message,
                       "MuJoCo version %s\nDrag-and-drop model file here", mj_versionString());
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, intro_message, 0, &this->con);
    }

    // show last loading error
    if (this->loadError[0]) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->loadError, 0, &this->con);
    }

    // render uis
    if (this->ui0_enable) {
      mjui_render(&this->ui0, &this->uistate, &this->con);
    }
    if (this->ui1_enable) {
      mjui_render(&this->ui1, &this->uistate, &this->con);
    }

    // finalize
    Glfw().glfwSwapBuffers(this->window);

    return;
  }

  // render scene
  mjr_render(rect, &this->scn, &this->con);

  // show last loading error
  if (this->loadError[0]) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->loadError, 0, &this->con);
  }

  // show pause/loading label
  if (!this->run || this->loadrequest) {
    mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                this->loadrequest ? "loading" : "pause", nullptr, &this->con);
  }

  // show realtime label
  if (this->run) {
    // get desired and actual percent-of-real-time
    float desiredRealtime = this->percentRealTime[this->realTimeIndex];
    float actualRealtime = 100 / this->measuredSlowdown;

    // check if real-time tracking is misaligned by more than than 10%
    bool misalignment = mju_abs(actualRealtime - desiredRealtime) > 0.1 * desiredRealtime;

    // display realtime overlay if not 100% or there is misalignment
    if (desiredRealtime != 100.0 || misalignment) {
      char overlay[30];
      if (misalignment) {
        std::snprintf(overlay, sizeof(overlay), "%g%% (%-.1f%%)", desiredRealtime, actualRealtime);
      } else {
        std::snprintf(overlay, sizeof(overlay), "%g%%", desiredRealtime);
      }
      mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, smallrect, overlay, nullptr, &this->con);
    }
  }

  // show ui 0
  if (this->ui0_enable) {
    mjui_render(&this->ui0, &this->uistate, &this->con);
  }

  // show ui 1
  if (this->ui1_enable) {
    mjui_render(&this->ui1, &this->uistate, &this->con);
  }

  // show help
  if (this->help) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &this->con);
  }

  // show info
  if (this->info) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->info_title, this->info_content, &this->con);
  }

  // show profiler
  if (this->profiler) {
    profilershow(this, rect);
  }

  // show sensor
  if (this->sensor) {
    sensorshow(this, smallrect);
  }

  // take screenshot, save to file
  if (this->screenshotrequest.exchange(false)) {
    const unsigned int h = uistate.rect[0].height;
    const unsigned int w = uistate.rect[0].width;
    std::unique_ptr<unsigned char[]> rgb(new unsigned char[3*w*h]);
    if (!rgb) {
      mju_error("could not allocate buffer for screenshot");
    }
    mjr_readPixels(rgb.get(), NULL, uistate.rect[0], &con);

    // flip up-down
    for (int r = 0; r < h/2; ++r) {
      unsigned char* top_row = &rgb[3*w*r];
      unsigned char* bottom_row = &rgb[3*w*(h-1-r)];
      std::swap_ranges(top_row, top_row+3*w, bottom_row);
    }

    // save as PNG
    // TODO(b/241577466): Parse the stem of the filename and use a .PNG extension.
    // Unfortunately, if we just yank ".xml"/".mjb" from the filename and append .PNG, the macOS
    // file dialog does not automatically open that location. Thus, we defer to a default
    // "screenshot.png" for now.
    const std::string path = getSavePath("screenshot.png");
    if (!path.empty()) {
      if (lodepng::encode(path, rgb.get(), w, h, LCT_RGB)) {
        mju_error("could not save screenshot");
      } else {
        std::printf("saved screenshot: %s\n", path.c_str());
      }
    }
  }

  // finalize
  Glfw().glfwSwapBuffers(this->window);
}


// clear callbacks registered in external structures
void Simulate::clearcallback() {
  uiClearCallback(this->window);
}

void Simulate::renderloop() {
  // Set timer callback (milliseconds)
  mjcb_time = timer;

  // multisampling
  Glfw().glfwWindowHint(GLFW_SAMPLES, 4);
  Glfw().glfwWindowHint(GLFW_VISIBLE, 1);

  // get videomode and save
  this->vmode = *Glfw().glfwGetVideoMode(Glfw().glfwGetPrimaryMonitor());

  // use videomode refreshrate if nonzero
  if (this->vmode.refreshRate) this->refreshRate = this->vmode.refreshRate;

  // create window
  this->window = Glfw().glfwCreateWindow((2*this->vmode.width)/3, (2*this->vmode.height)/3,
                                         "Simulate", nullptr, nullptr);
  if (!this->window) {
    Glfw().glfwTerminate();
    mju_error("could not create window");
  }

  // save window position and size
  Glfw().glfwGetWindowPos(this->window, this->windowpos, this->windowpos+1);
  Glfw().glfwGetWindowSize(this->window, this->windowsize, this->windowsize+1);

  // make context current, set v-sync
  Glfw().glfwMakeContextCurrent(this->window);
  Glfw().glfwSwapInterval(this->vsync);

  // init abstract visualization
  mjv_defaultCamera(&this->cam);
  mjv_defaultOption(&this->vopt);
  profilerinit(this);
  sensorinit(this);

  // make empty scene
  mjv_defaultScene(&this->scn);
  mjv_makeScene(nullptr, &this->scn, maxgeom);

  // select default font
  int fontscale = uiFontScale(this->window);
  this->font = fontscale/50 - 1;

  // make empty context
  mjr_defaultContext(&this->con);
  mjr_makeContext(nullptr, &this->con, fontscale);

  // init state and uis
  std::memset(&this->uistate, 0, sizeof(mjuiState));
  std::memset(&this->ui0, 0, sizeof(mjUI));
  std::memset(&this->ui1, 0, sizeof(mjUI));
  this->ui0.spacing = mjui_themeSpacing(this->spacing);
  this->ui0.color = mjui_themeColor(this->color);
  this->ui0.predicate = uiPredicate;
  this->ui0.rectid = 1;
  this->ui0.auxid = 0;
  this->ui1.spacing = mjui_themeSpacing(this->spacing);
  this->ui1.color = mjui_themeColor(this->color);
  this->ui1.predicate = uiPredicate;
  this->ui1.rectid = 2;
  this->ui1.auxid = 1;

  // set GLFW callbacks
  this->uistate.userdata = this;
  uiSetCallback(this->window, &this->uistate, uiEvent, uiLayout, uiRender, uiDrop);

  // populate uis with standard sections
  this->ui0.userdata = this;
  this->ui1.userdata = this;
  mjui_add(&this->ui0, defFile);
  mjui_add(&this->ui0, this->defOption);
  mjui_add(&this->ui0, this->defSimulation);
  mjui_add(&this->ui0, this->defWatch);
  uiModify(this->window, &this->ui0, &this->uistate, &this->con);
  uiModify(this->window, &this->ui1, &this->uistate, &this->con);

  // run event loop
  while (!Glfw().glfwWindowShouldClose(this->window) && !this->exitrequest.load()) {
    {
      const std::lock_guard<std::mutex> lock(this->mtx);

      // load model (not on first pass, to show "loading" label)
      if (this->loadrequest==1) {
        this->loadmodel();
      } else if (this->loadrequest>1) {
        this->loadrequest = 1;
      }

      // handle events (calls all callbacks)
      Glfw().glfwPollEvents();

      // prepare to render
      this->prepare();
    }  // std::lock_guard<std::mutex> (unblocks simulation thread)

    // render while simulation is running
    this->render();
  }

  this->exitrequest.store(true);

  this->clearcallback();
  mjv_freeScene(&this->scn);
  mjr_freeContext(&this->con);
}

}  // namespace mujoco
