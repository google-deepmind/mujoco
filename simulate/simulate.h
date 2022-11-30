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

#ifndef MUJOCO_SIMULATE_SIMULATE_H_
#define MUJOCO_SIMULATE_SIMULATE_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#ifdef MJSIMULATE_STATIC
  // static library
  #define MJSIMULATEAPI
  #define MJSIMULATELOCAL
#else
  #ifdef MJSIMULATE_DLL_EXPORTS
    #define MJSIMULATEAPI MUJOCO_HELPER_DLL_EXPORT
  #else
    #define MJSIMULATEAPI MUJOCO_HELPER_DLL_IMPORT
  #endif
  #define MJSIMULATELOCAL MUJOCO_HELPER_DLL_LOCAL
#endif

namespace mujoco {

//-------------------------------- global -----------------------------------------------

// Simulate states not contained in MuJoCo structures
class MJSIMULATEAPI Simulate {
 public:
  // create object and initialize the simulate ui
  Simulate() = default;

  // Apply UI pose perturbations to model and data
  void applyposepertubations(int flg_paused);

  // Apply UI force perturbations to model and data
  void applyforceperturbations();

  // Request that the Simulate UI thread render a new model
  // optionally delete the old model and data when done
  void load(const char* file, mjModel* m, mjData* d);

  // functions below are used by the renderthread
  // load mjb or xml model that has been requested by load()
  void loadmodel();

  // prepare to render
  void prepare();

  // render the ui to the window
  void render();

  // clear callbacks registered in external structures
  void clearcallback();

  // loop to render the UI (must be called from main thread because of MacOS)
  // https://discourse.glfw.org/t/multithreading-glfw/573/5
  void renderloop();

  // constants
  static constexpr int kMaxFilenameLength = 1000;

  // model and data to be visualized
  mjModel* mnew = nullptr;
  mjData* dnew = nullptr;

  mjModel* m = nullptr;
  mjData* d = nullptr;
  std::mutex mtx;
  std::condition_variable cond_loadrequest;

  // options
  int spacing = 0;
  int color = 0;
  int font = 0;
  int ui0_enable = 1;
  int ui1_enable = 1;
  int help = 0;
  int info = 0;
  int profiler = 0;
  int sensor = 0;
  int fullscreen = 0;
  int vsync = 1;
  int busywait = 0;

  // keyframe index
  int key = 0;

  // simulation
  int run = 1;

  // atomics for cross-thread messages
  std::atomic_bool exitrequest = false;
  std::atomic_bool droploadrequest = false;
  std::atomic_bool screenshotrequest = false;
  std::atomic_int uiloadrequest = 0;

  // loadrequest
  //   2: render thread asked to update its model
  //   1: showing "loading" label, about to load
  //   0: model loaded or no load requested.
  int loadrequest = 0;

  // strings
  char loadError[kMaxFilenameLength] = "";
  char dropfilename[kMaxFilenameLength] = "";
  char filename[kMaxFilenameLength] = "";
  char previous_filename[kMaxFilenameLength] = "";

  // time synchronization
  int realTimeIndex;
  bool speedChanged = true;
  float measuredSlowdown = 1.0;
  // logarithmically spaced realtime slow-down coefficients (percent)
  static constexpr float percentRealTime[] = {
      100, 80, 66,  50,  40, 33,  25,  20, 16,  13,
      10,  8,  6.6, 5.0, 4,  3.3, 2.5, 2,  1.6, 1.3,
      1,  .8, .66, .5,  .4, .33, .25, .2, .16, .13,
     .1
  };

  // control noise
  double ctrlnoisestd = 0.0;
  double ctrlnoiserate = 0.0;

  // watch
  char field[mjMAXUITEXT] = "qpos";
  int index = 0;

  // physics: need sync
  int disable[mjNDISABLE] = {0};
  int enable[mjNENABLE] = {0};

  // rendering: need sync
  int camera = 0;

  // abstract visualization
  mjvScene scn = {};
  mjvCamera cam = {};
  mjvOption vopt = {};
  mjvPerturb pert = {};
  mjvFigure figconstraint = {};
  mjvFigure figcost = {};
  mjvFigure figtimer = {};
  mjvFigure figsize = {};
  mjvFigure figsensor = {};

  // OpenGL rendering and UI
  GLFWvidmode vmode = {};
  int refreshRate = 60;
  int windowpos[2] = {0};
  int windowsize[2] = {0};
  mjrContext con = {};
  GLFWwindow* window = nullptr;
  mjuiState uistate = {};
  mjUI ui0 = {};
  mjUI ui1 = {};

  // Constant arrays needed for the option section of UI and the UI interface
  // TODO setting the size here is not ideal
  const mjuiDef defOption[14] = {
    {mjITEM_SECTION,  "Option",        1, nullptr,           "AO"},
    {mjITEM_SELECT,   "Spacing",       1, &this->spacing,    "Tight\nWide"},
    {mjITEM_SELECT,   "Color",         1, &this->color,      "Default\nOrange\nWhite\nBlack"},
    {mjITEM_SELECT,   "Font",          1, &this->font,       "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
    {mjITEM_CHECKINT, "Left UI (Tab)", 1, &this->ui0_enable, " #258"},
    {mjITEM_CHECKINT, "Right UI",      1, &this->ui1_enable, "S#258"},
    {mjITEM_CHECKINT, "Help",          2, &this->help,       " #290"},
    {mjITEM_CHECKINT, "Info",          2, &this->info,       " #291"},
    {mjITEM_CHECKINT, "Profiler",      2, &this->profiler,   " #292"},
    {mjITEM_CHECKINT, "Sensor",        2, &this->sensor,     " #293"},
  #ifdef __APPLE__
    {mjITEM_CHECKINT, "Fullscreen",    0, &this->fullscreen, " #294"},
  #else
    {mjITEM_CHECKINT, "Fullscreen",    1, &this->fullscreen, " #294"},
  #endif
    {mjITEM_CHECKINT, "Vertical Sync", 1, &this->vsync,      ""},
    {mjITEM_CHECKINT, "Busy Wait",     1, &this->busywait,   ""},
    {mjITEM_END}
  };


  // simulation section of UI
  const mjuiDef defSimulation[12] = {
    {mjITEM_SECTION,   "Simulation",    1, nullptr,              "AS"},
    {mjITEM_RADIO,     "",              2, &this->run,           "Pause\nRun"},
    {mjITEM_BUTTON,    "Reset",         2, nullptr,              " #259"},
    {mjITEM_BUTTON,    "Reload",        2, nullptr,              "CL"},
    {mjITEM_BUTTON,    "Align",         2, nullptr,              "CA"},
    {mjITEM_BUTTON,    "Copy pose",     2, nullptr,              "CC"},
    {mjITEM_SLIDERINT, "Key",           3, &this->key,           "0 0"},
    {mjITEM_BUTTON,    "Load key",      3},
    {mjITEM_BUTTON,    "Save key",      3},
    {mjITEM_SLIDERNUM, "Noise scale",   2, &this->ctrlnoisestd,  "0 2"},
    {mjITEM_SLIDERNUM, "Noise rate",    2, &this->ctrlnoiserate, "0 2"},
    {mjITEM_END}
  };


  // watch section of UI
  const mjuiDef defWatch[5] = {
    {mjITEM_SECTION,   "Watch",         0, nullptr,              "AW"},
    {mjITEM_EDITTXT,   "Field",         2, this->field,          "qpos"},
    {mjITEM_EDITINT,   "Index",         2, &this->index,         "1"},
    {mjITEM_STATIC,    "Value",         2, nullptr,              " "},
    {mjITEM_END}
  };

  // info strings
  char info_title[Simulate::kMaxFilenameLength] = {0};
  char info_content[Simulate::kMaxFilenameLength] = {0};
};

// setup the glfw dispatch table
// if set, must be called prior to other Simulate functions
MJSIMULATEAPI void setglfwdlhandle(void* dlhandle);

}  // namespace mujoco

#endif
