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
#include <optional>
#include <ratio>
#include <string>
#include <type_traits>
#include <utility>

#include "lodepng.h"
#include <mujoco/mjdata.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "platform_ui_adapter.h"
#include "array_safety.h"

// When launched via an App Bundle on macOS, the working directory is the path to the App Bundle's
// resource directory. This causes files to be saved into the bundle, which is not the desired
// behavior. Instead, we open a save dialog box to ask the user where to put the file.
// Since the dialog box logic needs to be written in Objective-C, we separate it into a different
// source file.
#ifdef __APPLE__
std::string GetSavePath(const char* filename);
#else
static std::string GetSavePath(const char* filename) {
  return filename;
}
#endif

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

template <typename T>
inline bool IsDifferent(const T& a, const T& b) {
  if constexpr (std::is_array_v<T>) {
    static_assert(std::rank_v<T> == 1);
    for (int i = 0; i < std::extent_v<T>; ++i) {
      if (a[i] != b[i]) {
        return true;
      }
    }
    return false;
  } else {
    return a != b;
  }
}

template <typename T>
inline void CopyScalar(T& dst, const T& src) {
  dst = src;
}

template <typename T, int N>
inline void CopyArray(T (&dst)[N], const T (&src)[N]) {
  for (int i = 0; i < N; ++i) {
    dst[i] = src[i];
  }
}

template <typename T>
inline void Copy(T& dst, const T& src) {
  if constexpr (std::is_array_v<T>) {
    CopyArray(dst, src);
  } else {
    CopyScalar(dst, src);
  }
}

//------------------------------------------- global -----------------------------------------------

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
  SECT_VISUALIZATION,
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
  {mjITEM_BUTTON,    "Screenshot",    2, nullptr,                    "CP"},
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
void InitializeProfiler(mj::Simulate* sim) {
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
  for (int n=0; n<6; n++)
    for (int i=0; i<mjMAXLINEPNT; i++) {
      sim->figtimer.linedata[n][2*i] = -i;
      sim->figsize.linedata[n][2*i] = -i;
    }
}

// update profiler figures
void UpdateProfiler(mj::Simulate* sim, const mjModel* m, const mjData* d) {
  // update constraint figure
  sim->figconstraint.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
  for (int i=1; i<5; i++) {
    sim->figconstraint.linepnt[i] = sim->figconstraint.linepnt[0];
  }
  if (m->opt.solver==mjSOL_PGS) {
    sim->figconstraint.linepnt[3] = 0;
    sim->figconstraint.linepnt[4] = 0;
  }
  if (m->opt.solver==mjSOL_CG) {
    sim->figconstraint.linepnt[4] = 0;
  }
  for (int i=0; i<sim->figconstraint.linepnt[0]; i++) {
    // x
    sim->figconstraint.linedata[0][2*i] = i;
    sim->figconstraint.linedata[1][2*i] = i;
    sim->figconstraint.linedata[2][2*i] = i;
    sim->figconstraint.linedata[3][2*i] = i;
    sim->figconstraint.linedata[4][2*i] = i;

    // y
    sim->figconstraint.linedata[0][2*i+1] = d->nefc;
    sim->figconstraint.linedata[1][2*i+1] = d->solver[i].nactive;
    sim->figconstraint.linedata[2][2*i+1] = d->solver[i].nchange;
    sim->figconstraint.linedata[3][2*i+1] = d->solver[i].neval;
    sim->figconstraint.linedata[4][2*i+1] = d->solver[i].nupdate;
  }

  // update cost figure
  sim->figcost.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
  for (int i=1; i<3; i++) {
    sim->figcost.linepnt[i] = sim->figcost.linepnt[0];
  }
  if (m->opt.solver==mjSOL_PGS) {
    sim->figcost.linepnt[1] = 0;
    sim->figcost.linepnt[2] = 0;
  }

  for (int i=0; i<sim->figcost.linepnt[0]; i++) {
    // x
    sim->figcost.linedata[0][2*i] = i;
    sim->figcost.linedata[1][2*i] = i;
    sim->figcost.linedata[2][2*i] = i;

    // y
    sim->figcost.linedata[0][2*i+1] = mju_log10(mju_max(mjMINVAL, d->solver[i].improvement));
    sim->figcost.linedata[1][2*i+1] = mju_log10(mju_max(mjMINVAL, d->solver[i].gradient));
    sim->figcost.linedata[2][2*i+1] = mju_log10(mju_max(mjMINVAL, d->solver[i].lineslope));
  }

  // get timers: total, collision, prepare, solve, other
  mjtNum total = d->timer[mjTIMER_STEP].duration;
  int number = d->timer[mjTIMER_STEP].number;
  if (!number) {
    total = d->timer[mjTIMER_FORWARD].duration;
    number = d->timer[mjTIMER_FORWARD].number;
  }
  number = mjMAX(1, number);
  float tdata[5] = {
    static_cast<float>(total/number),
    static_cast<float>(d->timer[mjTIMER_POS_COLLISION].duration/number),
    static_cast<float>(d->timer[mjTIMER_POS_MAKE].duration/number) +
    static_cast<float>(d->timer[mjTIMER_POS_PROJECT].duration/number),
    static_cast<float>(d->timer[mjTIMER_CONSTRAINT].duration/number),
    0
  };
  tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

  // update figtimer
  int pnt = mjMIN(201, sim->figtimer.linepnt[0]+1);
  for (int n=0; n<5; n++) {
    // shift data
    for (int i=pnt-1; i>0; i--) {
      sim->figtimer.linedata[n][2*i+1] = sim->figtimer.linedata[n][2*i-1];
    }

    // assign new
    sim->figtimer.linepnt[n] = pnt;
    sim->figtimer.linedata[n][1] = tdata[n];
  }

  // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
  float sdata[6] = {
    static_cast<float>(m->nv),
    static_cast<float>(m->nbody),
    static_cast<float>(d->nefc),
    static_cast<float>(mju_sqrt(d->solver_nnz)),
    static_cast<float>(d->ncon),
    static_cast<float>(d->solver_iter)
  };

  // update figsize
  pnt = mjMIN(201, sim->figsize.linepnt[0]+1);
  for (int n=0; n<6; n++) {
    // shift data
    for (int i=pnt-1; i>0; i--) {
      sim->figsize.linedata[n][2*i+1] = sim->figsize.linedata[n][2*i-1];
    }

    // assign new
    sim->figsize.linepnt[n] = pnt;
    sim->figsize.linedata[n][1] = sdata[n];
  }
}

// show profiler figures
void ShowProfiler(mj::Simulate* sim, mjrRect rect) {
  mjrRect viewport = {
    rect.left + rect.width - rect.width/4,
    rect.bottom,
    rect.width/4,
    rect.height/4
  };
  mjr_figure(viewport, &sim->figtimer, &sim->platform_ui->mjr_context());
  viewport.bottom += rect.height/4;
  mjr_figure(viewport, &sim->figsize, &sim->platform_ui->mjr_context());
  viewport.bottom += rect.height/4;
  mjr_figure(viewport, &sim->figcost, &sim->platform_ui->mjr_context());
  viewport.bottom += rect.height/4;
  mjr_figure(viewport, &sim->figconstraint, &sim->platform_ui->mjr_context());
}


// init sensor figure
void InitializeSensor(mj::Simulate* sim) {
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
  mju::strcpy_arr(figsensor.yformat, "%.1f");

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
void UpdateSensor(mj::Simulate* sim, const mjModel* m, const mjData* d) {
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
      figsensor.linedata[lineid][2*p+4*i+3] = d->sensordata[adr+i]/cutoff;
    }

    // update linepnt
    figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT-1, figsensor.linepnt[lineid]+2*dim);
  }
}

// show sensor figure
void ShowSensor(mj::Simulate* sim, mjrRect rect) {
  // constant width with and without profiler
  int width = sim->profiler ? rect.width/3 : rect.width/4;

  // render figure on the right
  mjrRect viewport = {
    rect.left + rect.width - width,
    rect.bottom,
    width,
    rect.height/3
  };
  mjr_figure(viewport, &sim->figsensor, &sim->platform_ui->mjr_context());
}

// prepare info text
void UpdateInfoText(mj::Simulate* sim, const mjModel* m, const mjData* d,
              char (&title)[mj::Simulate::kMaxFilenameLength],
              char (&content)[mj::Simulate::kMaxFilenameLength]) {
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

  // format FPS text
  char fps[10];
  if (sim->fps_ < 1) {
    mju::sprintf_arr(fps, "%0.1f ", sim->fps_);
  } else {
    mju::sprintf_arr(fps, "%.0f ", sim->fps_);
  }

  // prepare info text
  mju::strcpy_arr(title, "Time\nSize\nPruning\nCPU\nSolver   \nFPS\nMemory");
  int broad_pruning = d->nbodypair_broad ? (100.0*d->nbodypair_narrow)/d->nbodypair_broad : 0;
  int mid_pruning = d->ngeompair_mid ? (100.0*d->nbodypair_narrow)/d->ngeompair_mid : 0;
  mju::sprintf_arr(content,
                   "%-9.3f\n%d  (%d con)\nb: %d%% m: %d%%\n%.3f\n%.1f  (%d it)\n%s\n%.2g of %s",
                   d->time,
                   d->nefc, d->ncon,
                   broad_pruning, mid_pruning,
                   sim->run ?
                   d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) :
                   d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
                   solerr, d->solver_iter,
                   fps,
                   d->maxuse_arena/(double)(d->nstack * sizeof(mjtNum)),
                   mju_writeNumBytes(d->nstack * sizeof(mjtNum)));

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
void PrintField(char (&str)[mjMAXUINAME], void* ptr) {
  mju::sprintf_arr(str, "%g", *static_cast<mjtNum*>(ptr));
}

// update watch
void UpdateWatch(mj::Simulate* sim, const mjModel* m, const mjData* d) {
  // clear
  sim->ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
  mju::strcpy_arr(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

  // prepare symbols needed by xmacro
  MJDATA_POINTERS_PREAMBLE(m);

  // find specified field in mjData arrays, update value
  #define X(TYPE, NAME, NR, NC)                                                                  \
    if (!mju::strcmp_arr(#NAME, sim->field) &&                                                   \
        !mju::strcmp_arr(#TYPE, "mjtNum")) {                                                     \
      if (sim->index >= 0 && sim->index < m->NR * NC) {                                          \
        PrintField(sim->ui0.sect[SECT_WATCH].item[2].multi.name[0], d->NAME + sim->index);       \
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
void MakePhysicsSection(mj::Simulate* sim, int oldstate) {
  mjOption* opt = sim->fully_managed_ ? &sim->m_->opt : &sim->scnstate_.model.opt;
  mjuiDef defPhysics[] = {
    {mjITEM_SECTION,   "Physics",       oldstate, nullptr,           "AP"},
    {mjITEM_SELECT,    "Integrator",    2, &(opt->integrator),        "Euler\nRK4\nimplicit\nimplicitfast"},
    {mjITEM_SELECT,    "Collision",     2, &(opt->collision),         "All\nPair\nDynamic"},
    {mjITEM_SELECT,    "Cone",          2, &(opt->cone),              "Pyramidal\nElliptic"},
    {mjITEM_SELECT,    "Jacobian",      2, &(opt->jacobian),          "Dense\nSparse\nAuto"},
    {mjITEM_SELECT,    "Solver",        2, &(opt->solver),            "PGS\nCG\nNewton"},
    {mjITEM_SEPARATOR, "Algorithmic Parameters", 1},
    {mjITEM_EDITNUM,   "Timestep",      2, &(opt->timestep),          "1 0 1"},
    {mjITEM_EDITINT,   "Iterations",    2, &(opt->iterations),        "1 0 1000"},
    {mjITEM_EDITNUM,   "Tolerance",     2, &(opt->tolerance),         "1 0 1"},
    {mjITEM_EDITINT,   "Noslip Iter",   2, &(opt->noslip_iterations), "1 0 1000"},
    {mjITEM_EDITNUM,   "Noslip Tol",    2, &(opt->noslip_tolerance),  "1 0 1"},
    {mjITEM_EDITINT,   "MRR Iter",      2, &(opt->mpr_iterations),    "1 0 1000"},
    {mjITEM_EDITNUM,   "MPR Tol",       2, &(opt->mpr_tolerance),     "1 0 1"},
    {mjITEM_EDITNUM,   "API Rate",      2, &(opt->apirate),           "1 0 1000"},
    {mjITEM_SEPARATOR, "Physical Parameters", 1},
    {mjITEM_EDITNUM,   "Gravity",       2, opt->gravity,              "3"},
    {mjITEM_EDITNUM,   "Wind",          2, opt->wind,                 "3"},
    {mjITEM_EDITNUM,   "Magnetic",      2, opt->magnetic,             "3"},
    {mjITEM_EDITNUM,   "Density",       2, &(opt->density),           "1"},
    {mjITEM_EDITNUM,   "Viscosity",     2, &(opt->viscosity),         "1"},
    {mjITEM_EDITNUM,   "Imp Ratio",     2, &(opt->impratio),          "1"},
    {mjITEM_SEPARATOR, "Disable Flags", 1},
    {mjITEM_END}
  };
  mjuiDef defEnableFlags[] = {
    {mjITEM_SEPARATOR, "Enable Flags", 1},
    {mjITEM_END}
  };
  mjuiDef defOverride[] = {
    {mjITEM_SEPARATOR, "Contact Override", 1},
    {mjITEM_EDITNUM,   "Margin",        2, &(opt->o_margin),          "1"},
    {mjITEM_EDITNUM,   "Sol Imp",       2, &(opt->o_solimp),          "5"},
    {mjITEM_EDITNUM,   "Sol Ref",       2, &(opt->o_solref),          "2"},
    {mjITEM_END}
  };

  // add physics
  mjui_add(&sim->ui0, defPhysics);

  // add flags programmatically
  mjuiDef defFlag[] = {
    {mjITEM_CHECKINT,  "", 2, nullptr, ""},
    {mjITEM_END}
  };
  for (int i=0; i<mjNDISABLE; i++) {
    mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
    defFlag[0].pdata = sim->disable + i;
    mjui_add(&sim->ui0, defFlag);
  }
  mjui_add(&sim->ui0, defEnableFlags);
  for (int i=0; i<mjNENABLE; i++) {
    mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
    defFlag[0].pdata = sim->enable + i;
    mjui_add(&sim->ui0, defFlag);
  }

  // add contact override
  mjui_add(&sim->ui0, defOverride);
}



// make rendering section of UI
void MakeRenderingSection(mj::Simulate* sim, const mjModel* m, int oldstate) {
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
      &(sim->opt.label),
      "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\n"
      "Actuator\nConstraint\nSkin\nSelection\nSel Pnt\nContact\nForce"
    },
    {
      mjITEM_SELECT,
      "Frame",
      2,
      &(sim->opt.frame),
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
  for (int i=0; i<mjMIN(m->ncam, mjMAXUIMULTI-2); i++) {
    // prepare name
    char camname[mjMAXUITEXT] = "\n";
    if (m->names[m->name_camadr[i]]) {
      mju::strcat_arr(camname, m->names+m->name_camadr[i]);
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
  for (int i=0; i<mjNVISFLAG; i++) {
    // set name, remove "&"
    mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);
    for (int j=0; j<strlen(mjVISSTRING[i][0]); j++) {
      if (mjVISSTRING[i][0][j]=='&') {
        mju_strncpy(
          defFlag[0].name+j, mjVISSTRING[i][0]+j+1, mju::sizeof_arr(defFlag[0].name)-j);
        break;
      }
    }

    // set shortcut and data
    if (mjVISSTRING[i][2][0]) {
      mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
    } else {
      mju::sprintf_arr(defFlag[0].other, "");
    }
    defFlag[0].pdata = sim->opt.flags + i;
    mjui_add(&sim->ui0, defFlag);
  }

  // create tree slider
  mjuiDef defTree[] = {
      {mjITEM_SLIDERINT, "Tree depth", 2, &sim->opt.bvh_depth, "0 15"},
      {mjITEM_END}
  };
  mjui_add(&sim->ui0, defTree);

  // add rendering flags
  mjui_add(&sim->ui0, defOpenGL);
  for (int i=0; i<mjNRNDFLAG; i++) {
    mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);
    if (mjRNDSTRING[i][2][0]) {
      mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
    } else {
      mju::sprintf_arr(defFlag[0].other, "");
    }
    defFlag[0].pdata = sim->scn.flags + i;
    mjui_add(&sim->ui0, defFlag);
  }
}

// make visualization section of UI
void MakeVisualizationSection(mj::Simulate* sim, const mjModel* m, int oldstate) {
  mjStatistic* stat = sim->fully_managed_ ? &sim->m_->stat : &sim->scnstate_.model.stat;
  mjVisual* vis = sim->fully_managed_ ? &sim->m_->vis : &sim->scnstate_.model.vis;

  mjuiDef defVisualization[] = {
    {mjITEM_SECTION,   "Visualization", oldstate, nullptr, "AV"},
    {mjITEM_SEPARATOR, "Headlight",  1},
    {mjITEM_RADIO,     "Active",          5, &(vis->headlight.active),     "Off\nOn"},
    {mjITEM_EDITFLOAT, "Ambient",         2, &(vis->headlight.ambient),    "3"},
    {mjITEM_EDITFLOAT, "Diffuse",         2, &(vis->headlight.diffuse),    "3"},
    {mjITEM_EDITFLOAT, "Specular",        2, &(vis->headlight.specular),   "3"},
    {mjITEM_SEPARATOR, "Initial Free Camera",  1},
    {mjITEM_EDITNUM,   "Center",          2, &(stat->center),              "3"},
    {mjITEM_EDITFLOAT, "Azimuth",         2, &(vis->global.azimuth),       "1"},
    {mjITEM_EDITFLOAT, "Elevation",       2, &(vis->global.elevation),     "1"},
    {mjITEM_BUTTON,    "Align",           2, nullptr,                      "CA"},
    {mjITEM_SEPARATOR, "Global",  1},
    {mjITEM_EDITNUM,   "Extent",          2, &(stat->extent),              "1"},
    {mjITEM_EDITFLOAT, "Field of view",   2, &(vis->global.fovy),          "1"},
    {mjITEM_RADIO,     "Inertia",         5, &(vis->global.ellipsoidinertia), "Box\nEllipsoid"},
    {mjITEM_SEPARATOR, "Map",  1},
    {mjITEM_EDITFLOAT, "Stiffness",       2, &(vis->map.stiffness),        "1"},
    {mjITEM_EDITFLOAT, "Rot stiffness",   2, &(vis->map.stiffnessrot),     "1"},
    {mjITEM_EDITFLOAT, "Force",           2, &(vis->map.force),            "1"},
    {mjITEM_EDITFLOAT, "Torque",          2, &(vis->map.torque),           "1"},
    {mjITEM_EDITFLOAT, "Alpha",           2, &(vis->map.alpha),            "1"},
    {mjITEM_EDITFLOAT, "Fog start",       2, &(vis->map.fogstart),         "1"},
    {mjITEM_EDITFLOAT, "Fog end",         2, &(vis->map.fogend),           "1"},
    {mjITEM_EDITFLOAT, "Z near",          2, &(vis->map.znear),            "1"},
    {mjITEM_EDITFLOAT, "Z far",           2, &(vis->map.zfar),             "1"},
    {mjITEM_EDITFLOAT, "Haze",            2, &(vis->map.haze),             "1"},
    {mjITEM_EDITFLOAT, "Shadow clip",     2, &(vis->map.shadowclip),       "1"},
    {mjITEM_EDITFLOAT, "Shadow scale",    2, &(vis->map.shadowscale),      "1"},
    {mjITEM_SEPARATOR, "Scale",  1},
    {mjITEM_EDITNUM,   "All [meansize]",  2, &(stat->meansize),            "1"},
    {mjITEM_EDITFLOAT, "Force width",     2, &(vis->scale.forcewidth),     "1"},
    {mjITEM_EDITFLOAT, "Contact width",   2, &(vis->scale.contactwidth),   "1"},
    {mjITEM_EDITFLOAT, "Contact height",  2, &(vis->scale.contactheight),  "1"},
    {mjITEM_EDITFLOAT, "Connect",         2, &(vis->scale.connect),        "1"},
    {mjITEM_EDITFLOAT, "Com",             2, &(vis->scale.com),            "1"},
    {mjITEM_EDITFLOAT, "Camera",          2, &(vis->scale.camera),         "1"},
    {mjITEM_EDITFLOAT, "Light",           2, &(vis->scale.light),          "1"},
    {mjITEM_EDITFLOAT, "Select point",    2, &(vis->scale.selectpoint),    "1"},
    {mjITEM_EDITFLOAT, "Joint length",    2, &(vis->scale.jointlength),    "1"},
    {mjITEM_EDITFLOAT, "Joint width",     2, &(vis->scale.jointwidth),     "1"},
    {mjITEM_EDITFLOAT, "Actuator length", 2, &(vis->scale.actuatorlength), "1"},
    {mjITEM_EDITFLOAT, "Actuator width",  2, &(vis->scale.actuatorwidth),  "1"},
    {mjITEM_EDITFLOAT, "Frame length",    2, &(vis->scale.framelength),    "1"},
    {mjITEM_EDITFLOAT, "Frame width",     2, &(vis->scale.framewidth),     "1"},
    {mjITEM_EDITFLOAT, "Constraint",      2, &(vis->scale.constraint),     "1"},
    {mjITEM_EDITFLOAT, "Slider-crank",    2, &(vis->scale.slidercrank),    "1"},
    {mjITEM_END}
  };

  // add rendering standard
  mjui_add(&sim->ui0, defVisualization);
}

// make group section of UI
void MakeGroupSection(mj::Simulate* sim, int oldstate) {
  mjuiDef defGroup[] = {
    {mjITEM_SECTION,    "Group enable",     oldstate, nullptr,          "AG"},
    {mjITEM_SEPARATOR,  "Geom groups",  1},
    {mjITEM_CHECKBYTE,  "Geom 0",           2, sim->opt.geomgroup,          " 0"},
    {mjITEM_CHECKBYTE,  "Geom 1",           2, sim->opt.geomgroup+1,        " 1"},
    {mjITEM_CHECKBYTE,  "Geom 2",           2, sim->opt.geomgroup+2,        " 2"},
    {mjITEM_CHECKBYTE,  "Geom 3",           2, sim->opt.geomgroup+3,        " 3"},
    {mjITEM_CHECKBYTE,  "Geom 4",           2, sim->opt.geomgroup+4,        " 4"},
    {mjITEM_CHECKBYTE,  "Geom 5",           2, sim->opt.geomgroup+5,        " 5"},
    {mjITEM_SEPARATOR,  "Site groups",  1},
    {mjITEM_CHECKBYTE,  "Site 0",           2, sim->opt.sitegroup,          "S0"},
    {mjITEM_CHECKBYTE,  "Site 1",           2, sim->opt.sitegroup+1,        "S1"},
    {mjITEM_CHECKBYTE,  "Site 2",           2, sim->opt.sitegroup+2,        "S2"},
    {mjITEM_CHECKBYTE,  "Site 3",           2, sim->opt.sitegroup+3,        "S3"},
    {mjITEM_CHECKBYTE,  "Site 4",           2, sim->opt.sitegroup+4,        "S4"},
    {mjITEM_CHECKBYTE,  "Site 5",           2, sim->opt.sitegroup+5,        "S5"},
    {mjITEM_SEPARATOR,  "Joint groups", 1},
    {mjITEM_CHECKBYTE,  "Joint 0",          2, sim->opt.jointgroup,         ""},
    {mjITEM_CHECKBYTE,  "Joint 1",          2, sim->opt.jointgroup+1,       ""},
    {mjITEM_CHECKBYTE,  "Joint 2",          2, sim->opt.jointgroup+2,       ""},
    {mjITEM_CHECKBYTE,  "Joint 3",          2, sim->opt.jointgroup+3,       ""},
    {mjITEM_CHECKBYTE,  "Joint 4",          2, sim->opt.jointgroup+4,       ""},
    {mjITEM_CHECKBYTE,  "Joint 5",          2, sim->opt.jointgroup+5,       ""},
    {mjITEM_SEPARATOR,  "Tendon groups",    1},
    {mjITEM_CHECKBYTE,  "Tendon 0",         2, sim->opt.tendongroup,        ""},
    {mjITEM_CHECKBYTE,  "Tendon 1",         2, sim->opt.tendongroup+1,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 2",         2, sim->opt.tendongroup+2,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 3",         2, sim->opt.tendongroup+3,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 4",         2, sim->opt.tendongroup+4,      ""},
    {mjITEM_CHECKBYTE,  "Tendon 5",         2, sim->opt.tendongroup+5,      ""},
    {mjITEM_SEPARATOR,  "Actuator groups", 1},
    {mjITEM_CHECKBYTE,  "Actuator 0",       2, sim->opt.actuatorgroup,      ""},
    {mjITEM_CHECKBYTE,  "Actuator 1",       2, sim->opt.actuatorgroup+1,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 2",       2, sim->opt.actuatorgroup+2,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 3",       2, sim->opt.actuatorgroup+3,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 4",       2, sim->opt.actuatorgroup+4,    ""},
    {mjITEM_CHECKBYTE,  "Actuator 5",       2, sim->opt.actuatorgroup+5,    ""},
    {mjITEM_SEPARATOR,  "Skin groups", 1},
    {mjITEM_CHECKBYTE,  "Skin 0",           2, sim->opt.skingroup,          ""},
    {mjITEM_CHECKBYTE,  "Skin 1",           2, sim->opt.skingroup+1,        ""},
    {mjITEM_CHECKBYTE,  "Skin 2",           2, sim->opt.skingroup+2,        ""},
    {mjITEM_CHECKBYTE,  "Skin 3",           2, sim->opt.skingroup+3,        ""},
    {mjITEM_CHECKBYTE,  "Skin 4",           2, sim->opt.skingroup+4,        ""},
    {mjITEM_CHECKBYTE,  "Skin 5",           2, sim->opt.skingroup+5,        ""},
    {mjITEM_END}
  };

  // add section
  mjui_add(&sim->ui0, defGroup);
}

// make joint section of UI
void MakeJointSection(mj::Simulate* sim, int oldstate) {
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
  for (int i=0; i < sim->jnt_type_.size() && itemcnt<mjMAXUIITEM; i++)
    if ((sim->jnt_type_[i]==mjJNT_HINGE || sim->jnt_type_[i]==mjJNT_SLIDE)) {
      // skip if joint group is disabled
      if (!sim->opt.jointgroup[mjMAX(0, mjMIN(mjNGROUP-1, sim->jnt_group_[i]))]) {
        continue;
      }

      // set data and name
      if (sim->fully_managed_) {
        defSlider[0].pdata = &sim->d_->qpos[sim->m_->jnt_qposadr[i]];
      } else {
        defSlider[0].pdata = &sim->qpos_[sim->jnt_qposadr_[i]];
      }
      if (!sim->jnt_names_[i].empty()) {
        mju::strcpy_arr(defSlider[0].name, sim->jnt_names_[i].c_str());
      } else {
        mju::sprintf_arr(defSlider[0].name, "joint %d", i);
      }

      // set range
      if (sim->jnt_range_[i].has_value())
        mju::sprintf_arr(defSlider[0].other, "%.4g %.4g",
                         sim->jnt_range_[i]->first, sim->jnt_range_[i]->second);
      else if (sim->jnt_type_[i]==mjJNT_SLIDE) {
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
void MakeControlSection(mj::Simulate* sim, int oldstate) {
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
  for (int i=0; i < sim->actuator_ctrlrange_.size() && itemcnt<mjMAXUIITEM; i++) {
    // skip if actuator group is disabled
    if (!sim->opt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP-1, sim->actuator_group_[i]))]) {
      continue;
    }

    // set data and name
    if (sim->fully_managed_) {
      defSlider[0].pdata = &sim->d_->ctrl[i];
    } else {
      defSlider[0].pdata = &sim->ctrl_[i];
    }
    if (!sim->actuator_names_[i].empty()) {
      mju::strcpy_arr(defSlider[0].name, sim->actuator_names_[i].c_str());
    } else {
      mju::sprintf_arr(defSlider[0].name, "control %d", i);
    }

    // set range
    if (sim->actuator_ctrlrange_[i].has_value())
      mju::sprintf_arr(defSlider[0].other, "%.4g %.4g",
                       sim->actuator_ctrlrange_[i]->first, sim->actuator_ctrlrange_[i]->second);
    else {
      mju::strcpy_arr(defSlider[0].other, "-1 1");
    }

    // add and count
    mjui_add(&sim->ui1, defSlider);
    itemcnt++;
  }
}

// make model-dependent UI sections
void MakeUiSections(mj::Simulate* sim, const mjModel* m, const mjData* d) {
  // get section open-close state, UI 0
  int oldstate0[NSECT0];
  for (int i=0; i<NSECT0; i++) {
    oldstate0[i] = 0;
    if (sim->ui0.nsect>i) {
      oldstate0[i] = sim->ui0.sect[i].state;
    }
  }

  // get section open-close state, UI 1
  int oldstate1[NSECT1];
  for (int i=0; i<NSECT1; i++) {
    oldstate1[i] = 0;
    if (sim->ui1.nsect>i) {
      oldstate1[i] = sim->ui1.sect[i].state;
    }
  }

  // clear model-dependent sections of UI
  sim->ui0.nsect = SECT_PHYSICS;
  sim->ui1.nsect = 0;

  // make
  MakePhysicsSection(sim, oldstate0[SECT_PHYSICS]);
  MakeRenderingSection(sim, m, oldstate0[SECT_RENDERING]);
  MakeVisualizationSection(sim, m, oldstate0[SECT_VISUALIZATION]);
  MakeGroupSection(sim, oldstate0[SECT_GROUP]);
  MakeJointSection(sim, oldstate1[SECT_JOINT]);
  MakeControlSection(sim, oldstate1[SECT_CONTROL]);
}

//---------------------------------- utility functions ---------------------------------------------

// align and scale view
void AlignAndScaleView(mj::Simulate* sim, const mjModel* m) {
  // use default free camera parameters
  mjv_defaultFreeCamera(m, &sim->cam);
}


// copy qpos to clipboard as key
void CopyPose(mj::Simulate* sim, const mjModel* m, const mjData* d) {
  char clipboard[5000] = "<key qpos='";
  char buf[200];

  // prepare string
  for (int i=0; i<m->nq; i++) {
    mju::sprintf_arr(buf, i==m->nq-1 ? "%g" : "%g ", d->qpos[i]);
    mju::strcat_arr(clipboard, buf);
  }
  mju::strcat_arr(clipboard, "'/>");

  // copy to clipboard
  sim->platform_ui->SetClipboardString(clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum Timer() {
  return Milliseconds(mj::Simulate::Clock::now().time_since_epoch()).count();
}

// clear all times
void ClearTimers(mjData* d) {
  for (int i=0; i<mjNTIMER; i++) {
    d->timer[i].duration = 0;
    d->timer[i].number = 0;
  }
}

// copy current camera to clipboard as MJCF specification
void CopyCamera(mj::Simulate* sim) {
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
  sim->platform_ui->SetClipboardString(clipboard);
}

// update UI 0 when MuJoCo structures change (except for joint sliders)
void UpdateSettings(mj::Simulate* sim, const mjModel* m) {
  // physics flags
  for (int i=0; i<mjNDISABLE; i++) {
    int new_value = ((m->opt.disableflags & (1<<i)) !=0);
    if (sim->disable[i] != new_value) {
      sim->disable[i] = new_value;
      sim->pending_.ui_update_physics = true;
    }
  }
  for (int i=0; i<mjNENABLE; i++) {
    int new_value = ((m->opt.enableflags & (1<<i)) !=0);
    if (sim->enable[i] != new_value) {
      sim->enable[i] = new_value;
      sim->pending_.ui_update_physics = true;
    }
  }

  // camera
  int old_camera = sim->camera;
  if (sim->cam.type==mjCAMERA_FIXED) {
    sim->camera = 2 + sim->cam.fixedcamid;
  } else if (sim->cam.type==mjCAMERA_TRACKING) {
    sim->camera = 1;
  } else {
    sim->camera = 0;
  }
  if (old_camera != sim->camera) {
    sim->pending_.ui_update_rendering = true;
  }
}

// Compute suitable font scale.
int ComputeFontScale(const mj::PlatformUIAdapter& platform_ui) {
  // compute framebuffer-to-window ratio
  auto [buf_width, buf_height] = platform_ui.GetFramebufferSize();
  auto [win_width, win_height] = platform_ui.GetWindowSize();
  double b2w = static_cast<double>(buf_width) / win_width;

  // compute PPI
  double PPI = b2w * platform_ui.GetDisplayPixelsPerInch();

  // estimate font scaling, guard against unrealistic PPI
  int fs;
  if (buf_width > win_width) {
    fs = mju_round(b2w * 100);
  } else if (PPI>50 && PPI<350) {
    fs = mju_round(PPI);
  } else {
    fs = 150;
  }
  fs = mju_round(fs * 0.02) * 50;
  fs = mjMIN(300, mjMAX(100, fs));

  return fs;
}


//---------------------------------- UI handlers ---------------------------------------------------

// determine enable/disable item state given category
int UiPredicate(int category, void* userdata) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(userdata);

  switch (category) {
  case 2:                 // require model
    return sim->m_ || !sim->fully_managed_;

  case 3:                 // require model and nkey
    return sim->fully_managed_ && sim->nkey_;

  case 4:                 // require model and paused
    return sim->m_ && !sim->run;

  case 5:                 // require model and fully managed mode
    return sim->fully_managed_ && sim->m_;

  default:
    return 1;
  }
}

// set window layout
void UiLayout(mjuiState* state) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(state->userdata);
  mjrRect* rect = state->rect;

  // set number of rectangles
  state->nrect = 4;

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

void UiModify(mjUI* ui, mjuiState* state, mjrContext* con) {
  mjui_resize(ui, con);
  mjr_addAux(ui->auxid, ui->width, ui->maxheight, ui->spacing.samples, con);
  UiLayout(state);
  mjui_update(-1, -1, ui, state, con);
}

// handle UI event
void UiEvent(mjuiState* state) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(state->userdata);

  // call UI 0 if event is directed to it
  if ((state->dragrect==sim->ui0.rectid) ||
      (state->dragrect==0 && state->mouserect==sim->ui0.rectid) ||
      state->type==mjEVENT_KEY) {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui0, state, &sim->platform_ui->mjr_context());

    // file section
    if (it && it->sectionid==SECT_FILE) {
      switch (it->itemid) {
      case 0:             // Save xml
        sim->pending_.save_xml = GetSavePath("mjmodel.xml");
        break;

      case 1:             // Save mjb
        sim->pending_.save_mjb = GetSavePath("mjmodel.mjb");
        break;

      case 2:             // Print model
        sim->pending_.print_model = GetSavePath("MJMODEL.TXT");
        break;

      case 3:             // Print data
        sim->pending_.print_data = GetSavePath("MJDATA.TXT");
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
        mjr_changeFont(50*(sim->font+1), &sim->platform_ui->mjr_context());
        break;

      case 9:             // Full screen
        sim->platform_ui->ToggleFullscreen();
        break;

      case 10:            // Vertical sync
        sim->platform_ui->SetVSync(sim->vsync);
        break;
      }

      // modify UI
      UiModify(&sim->ui0, state, &sim->platform_ui->mjr_context());
      UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
    }

    // simulation section
    else if (it && it->sectionid==SECT_SIMULATION) {
      switch (it->itemid) {
      case 1:             // Reset
        sim->pending_.reset = true;
        break;

      case 2:             // Reload
        sim->uiloadrequest.fetch_add(1);
        break;

      case 3:             // Align
        sim->pending_.align = true;
        break;

      case 4:             // Copy pose
        sim->pending_.copy_pose = true;
        break;

      case 5:             // Adjust key
      case 6:             // Load key
        sim->pending_.load_key = true;
        break;

      case 7:             // Save key
        sim->pending_.save_key = true;
        break;
      }
    }

    // physics section
    else if (it && it->sectionid==SECT_PHYSICS && sim->m_) {
      mjOption* opt = sim->fully_managed_ ? &sim->m_->opt : &sim->scnstate_.model.opt;

      // update disable flags in mjOption
      opt->disableflags = 0;
      for (int i=0; i<mjNDISABLE; i++)
        if (sim->disable[i]) {
          opt->disableflags |= (1<<i);
        }

      // update enable flags in mjOption
      opt->enableflags = 0;
      for (int i=0; i<mjNENABLE; i++)
        if (sim->enable[i]) {
          opt->enableflags |= (1<<i);
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
          mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate,
                      &sim->platform_ui->mjr_context());
        }
      } else {
        sim->cam.type = mjCAMERA_FIXED;
        sim->cam.fixedcamid = sim->camera - 2;
      }
      // copy camera spec to clipboard (as MJCF element)
      if (it->itemid == 3) {
        CopyCamera(sim);
      }
    }

    // visualization section
    else if (it && it->sectionid==SECT_VISUALIZATION) {
      if (!mju::strcmp_arr(it->name, "Align")) {
        sim->pending_.align = true;
      }
    }

    // group section
    else if (it && it->sectionid==SECT_GROUP) {
      // remake joint section if joint group changed
      if (it->name[0]=='J' && it->name[1]=='o') {
        sim->ui1.nsect = SECT_JOINT;
        MakeJointSection(sim, sim->ui1.sect[SECT_JOINT].state);
        sim->ui1.nsect = NSECT1;
        UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
      }

      // remake control section if actuator group changed
      if (it->name[0]=='A' && it->name[1]=='c') {
        sim->ui1.nsect = SECT_CONTROL;
        MakeControlSection(sim, sim->ui1.sect[SECT_CONTROL].state);
        sim->ui1.nsect = NSECT1;
        UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
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
    mjuiItem* it = mjui_event(&sim->ui1, state, &sim->platform_ui->mjr_context());

    // control section
    if (it && it->sectionid==SECT_CONTROL) {
      // clear controls
      if (it->itemid==0) {
        sim->pending_.zero_ctrl = true;
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
      if (sim->fully_managed_ && sim->m_) {
        sim->run = 1 - sim->run;
        sim->pert.active = 0;
        mjui_update(-1, -1, &sim->ui0, state, &sim->platform_ui->mjr_context());
      }
      break;

    case mjKEY_RIGHT:           // step forward
      if (sim->fully_managed_ && sim->m_ && !sim->run) {
        ClearTimers(sim->d_);
        mj_step(sim->m_, sim->d_);
        UpdateProfiler(sim, sim->m_, sim->d_);
        UpdateSensor(sim, sim->m_, sim->d_);
        UpdateSettings(sim, sim->m_);
      }
      break;

    case mjKEY_PAGE_UP:         // select parent body
      if ((sim->m_ || !sim->fully_managed_) && sim->pert.select > 0) {
        sim->pert.select = sim->body_parentid_[sim->pert.select];
        sim->pert.skinselect = -1;

        // stop perturbation if world reached
        if (sim->pert.select<=0) {
          sim->pert.active = 0;
        }
      }

      break;

    case ']':                   // cycle up fixed cameras
      if ((sim->m_ || !sim->fully_managed_) && sim->ncam_) {
        sim->cam.type = mjCAMERA_FIXED;
        // camera = {0 or 1} are reserved for the free and tracking cameras
        if (sim->camera < 2 || sim->camera == 2 + sim->ncam_ - 1) {
          sim->camera = 2;
        } else {
          sim->camera += 1;
        }
        sim->cam.fixedcamid = sim->camera - 2;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
      }
      break;

    case '[':                   // cycle down fixed cameras
      if ((sim->m_ || !sim->fully_managed_) && sim->ncam_) {
        sim->cam.type = mjCAMERA_FIXED;
        // camera = {0 or 1} are reserved for the free and tracking cameras
        if (sim->camera <= 2) {
          sim->camera = 2 + sim->ncam_-1;
        } else {
          sim->camera -= 1;
        }
        sim->cam.fixedcamid = sim->camera - 2;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
      }
      break;

    case mjKEY_F6:                   // cycle frame visualisation
      if (sim->m_ || !sim->fully_managed_) {
        sim->opt.frame = (sim->opt.frame + 1) % mjNFRAME;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
      }
      break;

    case mjKEY_F7:                   // cycle label visualisation
      if (sim->m_ || !sim->fully_managed_) {
        sim->opt.label = (sim->opt.label + 1) % mjNLABEL;
        mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
      }
      break;

    case mjKEY_ESCAPE:          // free camera
      sim->cam.type = mjCAMERA_FREE;
      sim->camera = 0;
      mjui_update(SECT_RENDERING, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
      break;

    case '-':                   // slow down
      if (sim->fully_managed_) {
        int numclicks = sizeof(sim->percentRealTime) / sizeof(sim->percentRealTime[0]);
        if (sim->real_time_index < numclicks-1 && !state->shift) {
          sim->real_time_index++;
          sim->speed_changed = true;
        }
      }
      break;

    case '=':                   // speed up
      if (sim->fully_managed_ && sim->real_time_index > 0 && !state->shift) {
        sim->real_time_index--;
        sim->speed_changed = true;
      }
      break;
    }

    return;
  }

  // 3D scroll
  if (state->type==mjEVENT_SCROLL && state->mouserect==3) {
    // emulate vertical mouse motion = 2% of window height
    if (sim->fully_managed_) {
      mjv_moveCamera(sim->m_, mjMOUSE_ZOOM, 0, -zoom_increment*state->sy, &sim->scn, &sim->cam);
    } else {
      mjv_moveCameraFromState(
          &sim->scnstate_, mjMOUSE_ZOOM, 0, -zoom_increment*state->sy, &sim->scn, &sim->cam);
    }
    return;
  }

  // 3D press
  if (state->type==mjEVENT_PRESS && state->mouserect==3) {
    // set perturbation
    int newperturb = 0;
    if (state->control && sim->pert.select>0 && (sim->m_ || !sim->fully_managed_)) {
      // right: translate;  left: rotate
      if (state->right) {
        newperturb = mjPERT_TRANSLATE;
      } else if (state->left) {
        newperturb = mjPERT_ROTATE;
      }
      if (newperturb && !sim->pert.active) {
        sim->pending_.newperturb = newperturb;
      }
    }

    // handle double-click
    if (state->doubleclick && (sim->m_ || !sim->fully_managed_)) {
      sim->pending_.select = true;
      std::memcpy(&sim->pending_.select_state, state, sizeof(sim->pending_.select_state));

      // stop perturbation on select
      sim->pert.active = 0;
      sim->pending_.newperturb = 0;
    }

    return;
  }

  // 3D release
  if (state->type==mjEVENT_RELEASE && state->dragrect==3 && (sim->m_ || !sim->fully_managed_)) {
    // stop perturbation
    sim->pert.active = 0;
    sim->pending_.newperturb = 0;
    return;
  }

  // 3D move
  if (state->type==mjEVENT_MOVE && state->dragrect==3 && (sim->m_ || !sim->fully_managed_)) {
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
    if (sim->pert.active) {
      if (sim->fully_managed_) {
        mjv_movePerturb(
            sim->m_, sim->d_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->pert);
      } else {
        mjv_movePerturbFromState(
            &sim->scnstate_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->pert);
      }
    } else {
      if (sim->fully_managed_) {
        mjv_moveCamera(
            sim->m_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->cam);
      } else {
        mjv_moveCameraFromState(
            &sim->scnstate_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->cam);
      }
    }
    return;
  }

  // Dropped files
  if (state->type == mjEVENT_FILESDROP && state->dropcount > 0 && sim->fully_managed_) {
    while (sim->droploadrequest.load()) {}
    mju::strcpy_arr(sim->dropfilename, state->droppaths[0]);
    sim->droploadrequest.store(true);
    return;
  }

  // Redraw
  if (state->type == mjEVENT_REDRAW) {
    sim->Render();
    return;
  }
}
}  // namespace

namespace mujoco {
namespace mju = ::mujoco::sample_util;

Simulate::Simulate(std::unique_ptr<PlatformUIAdapter> platform_ui,
                   mjvScene* scn, mjvCamera* cam,
                   mjvOption* opt, mjvPerturb* pert,
                   bool fully_managed)
    : fully_managed_(fully_managed),
      scn(*scn),
      cam(*cam),
      opt(*opt),
      pert(*pert),
      platform_ui(std::move(platform_ui)),
      uistate(this->platform_ui->state()) {
  mjv_defaultSceneState(&scnstate_);
}

void Simulate::Sync() {
  if (!m_) {
    return;
  }

  bool update_profiler = this->profiler && (this->run || !this->m_);
  bool update_sensor = this->sensor && (this->run || !this->m_);

  for (int i = 0; i < m_->njnt; ++i) {
    std::optional<std::pair<mjtNum, mjtNum>> range;
    if (m_->jnt_limited[i]) {
      range.emplace(m_->jnt_range[2*i], m_->jnt_range[2*i + 1]);
    }
    if (jnt_range_[i] != range) {
      pending_.ui_update_joint = true;
      jnt_range_[i].swap(range);
    }
  }

  for (int i = 0; i < m_->nu; ++i) {
    std::optional<std::pair<mjtNum, mjtNum>> range;
    if (m_->actuator_ctrllimited[i]) {
      range.emplace(m_->actuator_ctrlrange[2*i], m_->actuator_ctrlrange[2*i + 1]);
    }
    if (actuator_ctrlrange_[i] != range) {
      pending_.ui_update_ctrl = true;
      actuator_ctrlrange_[i].swap(range);
    }
  }

  for (int i = 0; i < m_->nq; ++i) {
    if (qpos_[i] != qpos_prev_[i]) {
      d_->qpos[i] = qpos_[i];
    } else {
      qpos_[i] = d_->qpos[i];
    }
    if (qpos_prev_[i] != qpos_[i]) {
      pending_.ui_update_joint = true;
      qpos_prev_[i] = qpos_[i];
    }
  }

  for (int i = 0; i < m_->nu; ++i) {
    if (ctrl_[i] != ctrl_prev_[i]) {
      d_->ctrl[i] = ctrl_[i];
    } else {
      ctrl_[i] = d_->ctrl[i];
    }
    if (ctrl_prev_[i] != ctrl_[i]) {
      pending_.ui_update_ctrl = true;
      ctrl_prev_[i] = ctrl_[i];
    }
  }

  if (!fully_managed_) {
    // synchronize m_->opt with changes made via the UI
#define X(name)                                                  \
  if (IsDifferent(scnstate_.model.opt.name, mjopt_prev_.name)) { \
    pending_.ui_update_physics = true;                           \
    Copy(m_->opt.name, scnstate_.model.opt.name);                \
  }

    X(timestep);
    X(apirate);
    X(impratio);
    X(tolerance);
    X(noslip_tolerance);
    X(mpr_tolerance);
    X(gravity);
    X(wind);
    X(magnetic);
    X(density);
    X(viscosity);
    X(o_margin);
    X(o_solref);
    X(o_solimp);
    X(integrator);
    X(collision);
    X(cone);
    X(jacobian);
    X(solver);
    X(iterations);
    X(noslip_iterations);
    X(mpr_iterations);
    X(disableflags);
    X(enableflags);

  #undef X

    // synchronize number of mjWARN_VGEOMFULL warnings
    if (scnstate_.data.warning[mjWARN_VGEOMFULL].number > warn_vgeomfull_prev_) {
      d_->warning[mjWARN_VGEOMFULL].number +=
          scnstate_.data.warning[mjWARN_VGEOMFULL].number - warn_vgeomfull_prev_;
    }
  }

  if (pending_.save_xml) {
    char err[200];
    if (!pending_.save_xml->empty() && !mj_saveLastXML(pending_.save_xml->c_str(), m_, err, 200)) {
      std::printf("Save XML error: %s", err);
    }
    pending_.save_xml = std::nullopt;
  }

  if (pending_.save_mjb) {
    if (!pending_.save_mjb->empty()) {
      mj_saveModel(m_, pending_.save_mjb->c_str(), nullptr, 0);
    }
    pending_.save_mjb = std::nullopt;
  }

  if (pending_.print_model) {
    if (!pending_.print_model->empty()) {
      mj_printModel(m_, pending_.print_model->c_str());
    }
    pending_.print_model = std::nullopt;
  }

  if (pending_.print_data) {
    if (!pending_.print_data->empty()) {
      mj_printData(m_, d_, pending_.print_data->c_str());
    }
    pending_.print_data = std::nullopt;
  }

  if (pending_.reset) {
    mj_resetData(m_, d_);
    mj_forward(m_, d_);
    update_profiler = true;
    update_sensor = true;
    pending_.reset = false;
  }

  if (pending_.align) {
    AlignAndScaleView(this, m_);
    pending_.align = false;
  }

  if (pending_.copy_pose) {
    CopyPose(this, m_, d_);
    pending_.copy_pose = false;
  }

  if (pending_.load_key) {
    int i = this->key;
    d_->time = m_->key_time[i];
    mju_copy(d_->qpos, m_->key_qpos + i*m_->nq, m_->nq);
    mju_copy(d_->qvel, m_->key_qvel + i*m_->nv, m_->nv);
    mju_copy(d_->act, m_->key_act + i*m_->na, m_->na);
    mju_copy(d_->mocap_pos, m_->key_mpos + i*3*m_->nmocap, 3*m_->nmocap);
    mju_copy(d_->mocap_quat, m_->key_mquat + i*4*m_->nmocap,
              4*m_->nmocap);
    mju_copy(d_->ctrl, m_->key_ctrl + i*m_->nu, m_->nu);
    mj_forward(m_, d_);
    update_profiler = true;
    update_sensor = true;
    pending_.load_key = false;
  }

  if (pending_.save_key) {
    int i = this->key;
    m_->key_time[i] = d_->time;
    mju_copy(m_->key_qpos + i*m_->nq, d_->qpos, m_->nq);
    mju_copy(m_->key_qvel + i*m_->nv, d_->qvel, m_->nv);
    mju_copy(m_->key_act + i*m_->na, d_->act, m_->na);
    mju_copy(m_->key_mpos + i*3*m_->nmocap, d_->mocap_pos, 3*m_->nmocap);
    mju_copy(m_->key_mquat + i*4*m_->nmocap, d_->mocap_quat, 4*m_->nmocap);
    mju_copy(m_->key_ctrl + i*m_->nu, d_->ctrl, m_->nu);
    pending_.save_key = false;
  }

  if (pending_.zero_ctrl) {
    mju_zero(d_->ctrl, m_->nu);
    pending_.zero_ctrl = false;
  }

  // perturbation onset: reset reference
  if (pending_.newperturb) {
    mjv_initPerturb(m_, d_, &this->scn, &this->pert);
    this->pert.active = pending_.newperturb;
    pending_.newperturb = 0;
  }

  if (pending_.select) {
    // determine selection mode
    int selmode;
    if (pending_.select_state.button==mjBUTTON_LEFT) {
      selmode = 1;
    } else if (pending_.select_state.control) {
      selmode = 3;
    } else {
      selmode = 2;
    }

    // find geom and 3D click point, get corresponding body
    mjrRect r = pending_.select_state.rect[3];
    mjtNum selpnt[3];
    int selgeom, selskin;
    int selbody = mjv_select(m_, d_, &this->opt,
                             static_cast<mjtNum>(r.width) / r.height,
                             (pending_.select_state.x - r.left) / r.width,
                             (pending_.select_state.y - r.bottom) / r.height,
                             &this->scn, selpnt, &selgeom, &selskin);

    // set lookat point, start tracking is requested
    if (selmode==2 || selmode==3) {
      // copy selpnt if anything clicked
      if (selbody>=0) {
        mju_copy3(this->cam.lookat, selpnt);
      }

      // switch to tracking camera if dynamic body clicked
      if (selmode==3 && selbody>0) {
        // mujoco camera
        this->cam.type = mjCAMERA_TRACKING;
        this->cam.trackbodyid = selbody;
        this->cam.fixedcamid = -1;

        // UI camera
        this->camera = 1;
        pending_.ui_update_rendering = true;
      }
    }

    // set body selection
    else {
      if (selbody>=0) {
        // record selection
        this->pert.select = selbody;
        this->pert.skinselect = selskin;

        // compute localpos
        mjtNum tmp[3];
        mju_sub3(tmp, selpnt, d_->xpos + 3*this->pert.select);
        mju_mulMatTVec(this->pert.localpos, d_->xmat + 9*this->pert.select, tmp, 3, 3);
      } else {
        this->pert.select = 0;
        this->pert.skinselect = -1;
      }
    }
    pending_.select = false;
  }

  // update scene
  if (fully_managed_) {
    mjv_updateScene(m_, d_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
  } else {
    mjv_updateSceneState(m_, d_, &this->opt, &scnstate_);
    mjopt_prev_ = scnstate_.model.opt;
    warn_vgeomfull_prev_ = scnstate_.data.warning[mjWARN_VGEOMFULL].number;
  }

  // update settings
  UpdateSettings(this, m_);

  // update watch
  if (this->ui0_enable && this->ui0.sect[SECT_WATCH].state) {
    UpdateWatch(this, m_, d_);
  }

  // update info text
  if (this->info) {
    UpdateInfoText(this, m_, d_, this->info_title, this->info_content);
  }
  if (update_profiler) { UpdateProfiler(this, m_, d_); }
  if (update_sensor) { UpdateSensor(this, m_, d_); }

  // clear timers once profiler info has been copied
  ClearTimers(d_);

  if (this->run || !this->fully_managed_) {
    // clear old perturbations, apply new
    mju_zero(d_->xfrc_applied, 6*m_->nbody);
    mjv_applyPerturbPose(m_, d_, &this->pert, 0);  // mocap bodies only
    mjv_applyPerturbForce(m_, d_, &this->pert);
  } else {
    mjv_applyPerturbPose(m_, d_, &this->pert, 1);  // mocap and dynamic bodies
  }
}

//------------------------- Tell the render thread to load a file and wait -------------------------
void Simulate::Load(mjModel* m,
                                mjData* d,
                                const char* displayed_filename) {
  this->mnew_ = m;
  this->dnew_ = d;
  mju::strcpy_arr(this->filename, displayed_filename);

  {
    MutexLock lock(mtx);
    this->loadrequest = 2;

    // Wait for the render thread to be done loading
    // so that we know the old model and data's memory can
    // be free'd by the other thread (sometimes python)
    cond_loadrequest.wait(lock, [this]() { return this->loadrequest == 0; });
  }
}

//------------------------------------- load mjb or xml model --------------------------------------
void Simulate::LoadOnRenderThread() {
  this->m_ = this->mnew_;
  this->d_ = this->dnew_;

  ncam_ = this->mnew_->ncam;
  nkey_ = this->mnew_->nkey;
  body_parentid_.resize(this->mnew_->nbody);
  std::memcpy(body_parentid_.data(), this->mnew_->body_parentid,
              sizeof(this->mnew_->body_parentid[0]) * this->mnew_->nbody);

  jnt_type_.resize(this->mnew_->njnt);
  std::memcpy(jnt_type_.data(), this->mnew_->jnt_type,
              sizeof(this->mnew_->jnt_type[0]) * this->mnew_->njnt);

  jnt_group_.resize(this->mnew_->njnt);
  std::memcpy(jnt_group_.data(), this->mnew_->jnt_group,
              sizeof(this->mnew_->jnt_group[0]) * this->mnew_->njnt);

  jnt_qposadr_.resize(this->mnew_->njnt);
  std::memcpy(jnt_qposadr_.data(), this->mnew_->jnt_qposadr,
              sizeof(this->mnew_->jnt_qposadr[0]) * this->mnew_->njnt);

  jnt_range_.clear();
  jnt_range_.reserve(this->mnew_->njnt);
  for (int i = 0; i < this->mnew_->njnt; ++i) {
    if (this->mnew_->jnt_limited[i]) {
      jnt_range_.push_back(
          std::make_pair(this->mnew_->jnt_range[2 * i], this->mnew_->jnt_range[2 * i + 1]));
    } else {
      jnt_range_.push_back(std::nullopt);
    }
  }

  jnt_names_.clear();
  jnt_names_.reserve(this->mnew_->njnt);
  for (int i = 0; i < this->mnew_->njnt; ++i) {
    jnt_names_.emplace_back(this->mnew_->names + this->mnew_->name_jntadr[i]);
  }

  actuator_group_.resize(this->mnew_->nu);
  std::memcpy(actuator_group_.data(), this->mnew_->actuator_group,
              sizeof(this->mnew_->actuator_group[0]) * this->mnew_->nu);

  actuator_ctrlrange_.clear();
  actuator_ctrlrange_.reserve(this->mnew_->nu);
  for (int i = 0; i < this->mnew_->nu; ++i) {
    if (this->mnew_->actuator_ctrllimited[i]) {
      actuator_ctrlrange_.push_back(std::make_pair(
          this->mnew_->actuator_ctrlrange[2 * i], this->mnew_->actuator_ctrlrange[2 * i + 1]));
    } else {
      actuator_ctrlrange_.push_back(std::nullopt);
    }
  }

  actuator_names_.clear();
  actuator_names_.reserve(this->mnew_->nu);
  for (int i = 0; i < this->mnew_->nu; ++i) {
    actuator_names_.emplace_back(this->mnew_->names + this->mnew_->name_actuatoradr[i]);
  }

  qpos_.resize(this->mnew_->nq);
  std::memcpy(qpos_.data(), this->dnew_->qpos, sizeof(this->dnew_->qpos[0]) * this->mnew_->nq);
  qpos_prev_ = qpos_;

  ctrl_.resize(this->mnew_->nu);
  std::memcpy(ctrl_.data(), this->dnew_->ctrl, sizeof(this->dnew_->ctrl[0]) * this->mnew_->nu);
  ctrl_prev_ = ctrl_;

  // re-create scene and context
  if (this->fully_managed_) {
    mjv_makeScene(this->mnew_, &this->scn, kMaxGeom);
  } else {
    mjopt_prev_ = mnew_->opt;
    opt_prev_ = opt;
    cam_prev_ = cam;
    warn_vgeomfull_prev_ = dnew_->warning[mjWARN_VGEOMFULL].number;
    mjv_makeSceneState(this->mnew_, this->dnew_, &this->scnstate_, kMaxGeom);
  }

  this->platform_ui->RefreshMjrContext(this->mnew_, 50*(this->font+1));
  UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());

  if (!this->platform_ui->IsGPUAccelerated()) {
    this->scn.flags[mjRND_SHADOW] = 0;
    this->scn.flags[mjRND_REFLECTION] = 0;
  }

  // clear perturbation state
  this->pert.active = 0;
  this->pert.select = 0;
  this->pert.skinselect = -1;

  // align and scale view unless reloading the same file
  if (this->filename[0] &&
      mju::strcmp_arr(this->filename, this->previous_filename)) {
    AlignAndScaleView(this, this->mnew_);
    mju::strcpy_arr(this->previous_filename, this->filename);
  }

  // update scene
  if (fully_managed_) {
    mjv_updateScene(this->mnew_, this->dnew_,
                    &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
  } else {
    mjv_updateSceneState(this->mnew_, this->dnew_, &this->opt, &this->scnstate_);
  }

  // set window title to model name
  if (this->mnew_->names) {
    char title[200] = "MuJoCo : ";
    mju::strcat_arr(title, this->mnew_->names);
    platform_ui->SetWindowTitle(title);
  }

  // set keyframe range and divisions
  this->ui0.sect[SECT_SIMULATION].item[5].slider.range[0] = 0;
  this->ui0.sect[SECT_SIMULATION].item[5].slider.range[1] = mjMAX(0, this->mnew_->nkey - 1);
  this->ui0.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, this->mnew_->nkey - 1);

  // rebuild UI sections
  MakeUiSections(this, this->mnew_, this->dnew_);

  // full ui update
  UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
  UpdateSettings(this, this->mnew_);

  // clear request
  this->loadrequest = 0;
  cond_loadrequest.notify_all();

  // set real time index
  int numclicks = sizeof(this->percentRealTime) / sizeof(this->percentRealTime[0]);
  float min_error = 1e6;
  float desired = mju_log(100*this->mnew_->vis.global.realtime);
  for (int click=0; click<numclicks; click++) {
    float error = mju_abs(mju_log(this->percentRealTime[click]) - desired);
    if (error < min_error) {
      min_error = error;
      this->real_time_index = click;
    }
  }

  this->mnew_ = nullptr;
  this->dnew_ = nullptr;
}


//------------------------------------------- rendering --------------------------------------------

// render the ui to the window
void Simulate::Render() {
  // update rendering context buffer size if required
  if (this->platform_ui->EnsureContextSize()) {
    UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
    UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
  }

  // get 3D rectangle and reduced for profiler
  mjrRect rect = this->uistate.rect[3];
  mjrRect smallrect = rect;
  if (this->profiler) {
    smallrect.width = rect.width - rect.width/4;
  }

  // no model
  if (this->fully_managed_ && !this->m_) {
    // blank screen
    mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

    // label
    if (this->loadrequest) {
      mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, "loading", nullptr,
                  &this->platform_ui->mjr_context());
    } else {
      char intro_message[Simulate::kMaxFilenameLength];
      mju::sprintf_arr(intro_message,
                       "MuJoCo version %s\nDrag-and-drop model file here", mj_versionString());
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, intro_message, 0,
                  &this->platform_ui->mjr_context());
    }

    // show last loading error
    if (this->load_error[0]) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->load_error, 0,
                  &this->platform_ui->mjr_context());
    }

    // render uis
    if (this->ui0_enable) {
      mjui_render(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
    }
    if (this->ui1_enable) {
      mjui_render(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }

    // finalize
    this->platform_ui->SwapBuffers();

    return;
  }

  // update UI sections from last sync
  if (this->ui0_enable && this->ui0.sect[SECT_WATCH].state) {
    mjui_update(SECT_WATCH, -1, &this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  }

  if (pending_.ui_update_physics) {
    if (this->ui0_enable && this->ui0.sect[SECT_PHYSICS].state) {
      mjui_update(SECT_PHYSICS, -1, &this->ui0, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_update_physics = false;
  }

  if (!fully_managed_) {
    if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state &&
        (cam_prev_.type != cam.type ||
         cam_prev_.fixedcamid != cam.fixedcamid ||
         cam_prev_.trackbodyid != cam.trackbodyid ||
         opt_prev_.label != opt.label || opt_prev_.frame != opt.frame ||
         IsDifferent(opt_prev_.flags, opt.flags))) {
      pending_.ui_update_rendering = true;
    }

    if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state &&
        (IsDifferent(opt_prev_.geomgroup, opt.geomgroup) ||
         IsDifferent(opt_prev_.sitegroup, opt.sitegroup) ||
         IsDifferent(opt_prev_.jointgroup, opt.jointgroup) ||
         IsDifferent(opt_prev_.tendongroup, opt.tendongroup) ||
         IsDifferent(opt_prev_.actuatorgroup, opt.actuatorgroup) ||
         IsDifferent(opt_prev_.skingroup, opt.skingroup))) {
      mjui_update(SECT_GROUP, -1, &this->ui0, &this->uistate,
                  &this->platform_ui->mjr_context());
    }

    opt_prev_ = opt;
    cam_prev_ = cam;
  }

  if (pending_.ui_update_rendering) {
    if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state) {
      mjui_update(SECT_RENDERING, -1, &this->ui0, &this->uistate,
                  &this->platform_ui->mjr_context());
    }
    pending_.ui_update_rendering = false;
  }

  if (pending_.ui_update_joint) {
    if (this->ui1_enable && this->ui1.sect[SECT_JOINT].state) {
      mjui_update(SECT_JOINT, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_update_joint = false;
  }

  if (pending_.ui_update_ctrl) {
    if (this->ui1_enable && this->ui1.sect[SECT_CONTROL].state) {
      mjui_update(SECT_CONTROL, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
    }
    pending_.ui_update_ctrl = false;
  }

  // render scene
  mjr_render(rect, &this->scn, &this->platform_ui->mjr_context());

  // show last loading error
  if (this->load_error[0]) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->load_error, 0,
                &this->platform_ui->mjr_context());
  }

  // make pause/loading label
  std::string pauseloadlabel;
  if (!this->run || this->loadrequest) {
    pauseloadlabel = this->loadrequest ? "loading" : "pause";
  }

  // get desired and actual percent-of-real-time
  float desiredRealtime = this->percentRealTime[this->real_time_index];
  float actualRealtime = 100 / this->measured_slowdown;

  // if running, check for misalignment of more than 10%
  float realtime_offset = mju_abs(actualRealtime - desiredRealtime);
  bool misaligned = this->run && realtime_offset > 0.1 * desiredRealtime;

  // make realtime overlay label
  char rtlabel[30] = {'\0'};
  if (desiredRealtime != 100.0 || misaligned) {
    // print desired realtime
    int labelsize = std::snprintf(rtlabel,
                                  sizeof(rtlabel), "%g%%", desiredRealtime);

    // if misaligned, append to label
    if (misaligned) {
      std::snprintf(rtlabel+labelsize,
                    sizeof(rtlabel)-labelsize, " (%-4.1f%%)", actualRealtime);
    }
  }

  // draw top left overlay
  if (!pauseloadlabel.empty() || rtlabel[0]) {
    std::string newline = !pauseloadlabel.empty() && rtlabel[0] ? "\n" : "";
    std::string topleftlabel = rtlabel + newline + pauseloadlabel;
    mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, smallrect,
                topleftlabel.c_str(), nullptr, &this->platform_ui->mjr_context());
  }


  // show ui 0
  if (this->ui0_enable) {
    mjui_render(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  }

  // show ui 1
  if (this->ui1_enable) {
    mjui_render(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
  }

  // show help
  if (this->help) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content,
                &this->platform_ui->mjr_context());
  }

  // show info
  if (this->info) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->info_title, this->info_content,
                &this->platform_ui->mjr_context());
  }

  // show profiler
  if (this->profiler) {
    ShowProfiler(this, rect);
  }

  // show sensor
  if (this->sensor) {
    ShowSensor(this, smallrect);
  }

  // take screenshot, save to file
  if (this->screenshotrequest.exchange(false)) {
    const unsigned int h = uistate.rect[0].height;
    const unsigned int w = uistate.rect[0].width;
    std::unique_ptr<unsigned char[]> rgb(new unsigned char[3*w*h]);
    if (!rgb) {
      mju_error("could not allocate buffer for screenshot");
    }
    mjr_readPixels(rgb.get(), nullptr, uistate.rect[0], &this->platform_ui->mjr_context());

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
    const std::string path = GetSavePath("screenshot.png");
    if (!path.empty()) {
      if (lodepng::encode(path, rgb.get(), w, h, LCT_RGB)) {
        mju_error("could not save screenshot");
      } else {
        std::printf("saved screenshot: %s\n", path.c_str());
      }
    }
  }

  // finalize
  this->platform_ui->SwapBuffers();
}



void Simulate::RenderLoop() {
  // Set timer callback (milliseconds)
  mjcb_time = Timer;

  // init abstract visualization
  mjv_defaultCamera(&this->cam);
  mjv_defaultOption(&this->opt);
  InitializeProfiler(this);
  InitializeSensor(this);

  // make empty scene
  if (fully_managed_) {
    mjv_defaultScene(&this->scn);
    mjv_makeScene(nullptr, &this->scn, kMaxGeom);
  }

  if (!this->platform_ui->IsGPUAccelerated()) {
    this->scn.flags[mjRND_SHADOW] = 0;
    this->scn.flags[mjRND_REFLECTION] = 0;
  }

  // select default font
  int fontscale = ComputeFontScale(*this->platform_ui);
  this->font = fontscale/50 - 1;

  // make empty context
  this->platform_ui->RefreshMjrContext(nullptr, fontscale);

  // init state and uis
  std::memset(&this->uistate, 0, sizeof(mjuiState));
  std::memset(&this->ui0, 0, sizeof(mjUI));
  std::memset(&this->ui1, 0, sizeof(mjUI));

  auto [buf_width, buf_height] = this->platform_ui->GetFramebufferSize();
  this->uistate.nrect = 1;
  this->uistate.rect[0].width = buf_width;
  this->uistate.rect[0].height = buf_height;

  this->ui0.spacing = mjui_themeSpacing(this->spacing);
  this->ui0.color = mjui_themeColor(this->color);
  this->ui0.predicate = UiPredicate;
  this->ui0.rectid = 1;
  this->ui0.auxid = 0;

  this->ui1.spacing = mjui_themeSpacing(this->spacing);
  this->ui1.color = mjui_themeColor(this->color);
  this->ui1.predicate = UiPredicate;
  this->ui1.rectid = 2;
  this->ui1.auxid = 1;

  // set GUI adapter callbacks
  this->uistate.userdata = this;
  this->platform_ui->SetEventCallback(UiEvent);
  this->platform_ui->SetLayoutCallback(UiLayout);

  // populate uis with standard sections
  this->ui0.userdata = this;
  this->ui1.userdata = this;
  mjui_add(&this->ui0, defFile);
  mjui_add(&this->ui0, this->def_option);
  mjui_add(&this->ui0, this->def_simulation);
  mjui_add(&this->ui0, this->def_watch);
  UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
  UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());

  // set VSync to initial value
  this->platform_ui->SetVSync(this->vsync);

  frames_ = 0;
  last_fps_update_ = mj::Simulate::Clock::now();

  // run event loop
  while (!this->platform_ui->ShouldCloseWindow() && !this->exitrequest.load()) {
    {
      const MutexLock lock(this->mtx);

      // load model (not on first pass, to show "loading" label)
      if (this->loadrequest==1) {
        this->LoadOnRenderThread();
      } else if (this->loadrequest>1) {
        this->loadrequest = 1;
      }

      // poll and handle events
      this->platform_ui->PollEvents();

      // update scene, doing a full sync if in fully managed mode
      if (this->fully_managed_) {
        Sync();
      } else {
        scnstate_.data.warning[mjWARN_VGEOMFULL].number += mjv_updateSceneFromState(
            &scnstate_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
      }
    }  // MutexLock (unblocks simulation thread)

    // render while simulation is running
    this->Render();

    // update FPS stat, at most 5 times per second
    auto now = mj::Simulate::Clock::now();
    double interval = Seconds(now - last_fps_update_).count();
    ++frames_;
    if (interval > 0.2) {
      last_fps_update_ = now;
      fps_ = frames_ / interval;
      frames_ = 0;
    }
  }

  if (fully_managed_){
    mjv_freeScene(&this->scn);
  } else {
    mjv_freeSceneState(&scnstate_);
  }

  this->exitrequest.store(2);
}
}  // namespace mujoco
