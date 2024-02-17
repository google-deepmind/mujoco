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
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>
#include "platform_ui_adapter.h"

namespace mujoco {

// The viewer itself doesn't require a reentrant mutex, however we use it in
// order to provide a Python sync API that doesn't require separate locking
// (since sync is by far the most common operation), but that also won't
// deadlock if called when a lock is already held by the user script on the
// same thread.
class SimulateMutex : public std::recursive_mutex {};
using MutexLock = std::unique_lock<std::recursive_mutex>;

// Simulate states not contained in MuJoCo structures
class Simulate {
 public:
  using Clock = std::chrono::steady_clock;
  static_assert(std::ratio_less_equal_v<Clock::period, std::milli>);

  static constexpr int kMaxGeom = 20000;

  // create object and initialize the simulate ui
  Simulate(
      std::unique_ptr<PlatformUIAdapter> platform_ui_adapter,
      mjvCamera* cam, mjvOption* opt, mjvPerturb* pert, bool is_passive);

  // Synchronize mjModel and mjData state with UI inputs, and update
  // visualization.
  void Sync();

  void UpdateHField(int hfieldid);
  void UpdateMesh(int meshid);
  void UpdateTexture(int texid);

  // Request that the Simulate UI display a "loading" message
  // Called prior to Load or LoadMessageClear
  void LoadMessage(const char* displayed_filename);

  // Request that the Simulate UI thread render a new model
  void Load(mjModel* m, mjData* d, const char* displayed_filename);

  // Clear the loading message
  // Can be called instead of Load to clear the message without
  // requesting the UI load a model
  void LoadMessageClear(void);

  // functions below are used by the renderthread
  // load mjb or xml model that has been requested by load()
  void LoadOnRenderThread();

  // render the ui to the window
  void Render();

  // loop to render the UI (must be called from main thread because of MacOS)
  void RenderLoop();

  // add state to history buffer
  void AddToHistory();

  // constants
  static constexpr int kMaxFilenameLength = 1000;

  // whether the viewer is operating in passive mode, where it cannot assume
  // that it has exclusive access to mjModel, mjData, and various mjv objects
  bool is_passive_ = false;

  // model and data to be visualized
  mjModel* mnew_ = nullptr;
  mjData* dnew_ = nullptr;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  int ncam_ = 0;
  int nkey_ = 0;
  int state_size_ = 0;      // number of mjtNums in a history buffer state
  int nhistory_ = 0;        // number of states saved in history buffer
  int history_cursor_ = 0;  // cursor pointing at last saved state

  std::vector<int> body_parentid_;

  std::vector<int> jnt_type_;
  std::vector<int> jnt_group_;
  std::vector<int> jnt_qposadr_;
  std::vector<std::optional<std::pair<mjtNum, mjtNum>>> jnt_range_;
  std::vector<std::string> jnt_names_;

  std::vector<int> actuator_group_;
  std::vector<std::optional<std::pair<mjtNum, mjtNum>>> actuator_ctrlrange_;
  std::vector<std::string> actuator_names_;

  std::vector<mjtNum> history_;  // history buffer (nhistory x state_size)

  // mjModel and mjData fields that can be modified by the user through the GUI
  std::vector<mjtNum> qpos_;
  std::vector<mjtNum> qpos_prev_;
  std::vector<mjtNum> ctrl_;
  std::vector<mjtNum> ctrl_prev_;

  mjvSceneState scnstate_;
  mjOption mjopt_prev_;
  mjvOption opt_prev_;
  mjvCamera cam_prev_;

  int warn_vgeomfull_prev_;

  // pending GUI-driven actions, to be applied at the next call to Sync
  struct {
    std::optional<std::string> save_xml;
    std::optional<std::string> save_mjb;
    std::optional<std::string> print_model;
    std::optional<std::string> print_data;
    bool reset;
    bool align;
    bool copy_pose;
    bool load_from_history;
    bool load_key;
    bool save_key;
    bool zero_ctrl;
    int newperturb;
    bool select;
    mjuiState select_state;
    bool ui_update_simulation;
    bool ui_update_physics;
    bool ui_update_rendering;
    bool ui_update_joint;
    bool ui_update_ctrl;
    bool ui_remake_ctrl;
  } pending_ = {};

  SimulateMutex mtx;
  std::condition_variable_any cond_loadrequest;

  int frames_ = 0;
  std::chrono::time_point<Clock> last_fps_update_;
  double fps_ = 0;

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
  int pause_update = 1;
  int fullscreen = 0;
  int vsync = 1;
  int busywait = 0;

  // keyframe index
  int key = 0;

  // index of history-scrubber slider
  int scrub_index = 0;

  // simulation
  int run = 1;

  // atomics for cross-thread messages
  std::atomic_int exitrequest = 0;
  std::atomic_int droploadrequest = 0;
  std::atomic_int screenshotrequest = 0;
  std::atomic_int uiloadrequest = 0;

  // loadrequest
  //   3: display a loading message
  //   2: render thread asked to update its model
  //   1: showing "loading" label, about to load
  //   0: model loaded or no load requested.
  int loadrequest = 0;

  // strings
  char load_error[kMaxFilenameLength] = "";
  char dropfilename[kMaxFilenameLength] = "";
  char filename[kMaxFilenameLength] = "";
  char previous_filename[kMaxFilenameLength] = "";

  // time synchronization
  int real_time_index = 0;
  bool speed_changed = true;
  float measured_slowdown = 1.0;
  // logarithmically spaced real-time slow-down coefficients (percent)
  static constexpr float percentRealTime[] = {
      100, 80, 66,  50,  40, 33,  25,  20, 16,  13,
      10,  8,  6.6, 5.0, 4,  3.3, 2.5, 2,  1.6, 1.3,
      1,  .8, .66, .5,  .4, .33, .25, .2, .16, .13,
     .1
  };

  // control noise
  double ctrl_noise_std = 0.0;
  double ctrl_noise_rate = 0.0;

  // watch
  char field[mjMAXUITEXT] = "qpos";
  int index = 0;

  // physics: need sync
  int disable[mjNDISABLE] = {0};
  int enable[mjNENABLE] = {0};
  int enableactuator[mjNGROUP] = {0};

  // rendering: need sync
  int camera = 0;

  // abstract visualization
  mjvScene scn;
  mjvCamera& cam;
  mjvOption& opt;
  mjvPerturb& pert;
  mjvFigure figconstraint = {};
  mjvFigure figcost = {};
  mjvFigure figtimer = {};
  mjvFigure figsize = {};
  mjvFigure figsensor = {};

  // additional user-defined visualization geoms (used in passive mode)
  mjvScene* user_scn = nullptr;
  mjtByte user_scn_flags_prev_[mjNRNDFLAG];

  // OpenGL rendering and UI
  int refresh_rate = 60;
  int window_pos[2] = {0};
  int window_size[2] = {0};
  std::unique_ptr<PlatformUIAdapter> platform_ui;
  mjuiState& uistate;
  mjUI ui0 = {};
  mjUI ui1 = {};

  // Constant arrays needed for the option section of UI and the UI interface
  // TODO setting the size here is not ideal
  const mjuiDef def_option[13] = {
    {mjITEM_SECTION,  "Option",        1, nullptr,           "AO"},
    {mjITEM_CHECKINT, "Help",          2, &this->help,       " #290"},
    {mjITEM_CHECKINT, "Info",          2, &this->info,       " #291"},
    {mjITEM_CHECKINT, "Profiler",      2, &this->profiler,   " #292"},
    {mjITEM_CHECKINT, "Sensor",        2, &this->sensor,     " #293"},
    {mjITEM_CHECKINT, "Pause update",  2, &this->pause_update,    ""},
  #ifdef __APPLE__
    {mjITEM_CHECKINT, "Fullscreen",    0, &this->fullscreen, " #294"},
  #else
    {mjITEM_CHECKINT, "Fullscreen",    1, &this->fullscreen, " #294"},
  #endif
    {mjITEM_CHECKINT, "Vertical Sync", 1, &this->vsync,      ""},
    {mjITEM_CHECKINT, "Busy Wait",     1, &this->busywait,   ""},
    {mjITEM_SELECT,   "Spacing",       1, &this->spacing,    "Tight\nWide"},
    {mjITEM_SELECT,   "Color",         1, &this->color,      "Default\nOrange\nWhite\nBlack"},
    {mjITEM_SELECT,   "Font",          1, &this->font,       "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
    {mjITEM_END}
  };


  // simulation section of UI
  const mjuiDef def_simulation[14] = {
    {mjITEM_SECTION,   "Simulation",    1, nullptr,              "AS"},
    {mjITEM_RADIO,     "",              5, &this->run,           "Pause\nRun"},
    {mjITEM_BUTTON,    "Reset",         2, nullptr,              " #259"},
    {mjITEM_BUTTON,    "Reload",        5, nullptr,              "CL"},
    {mjITEM_BUTTON,    "Align",         2, nullptr,              "CA"},
    {mjITEM_BUTTON,    "Copy pose",     2, nullptr,              "CC"},
    {mjITEM_SLIDERINT, "Key",           3, &this->key,           "0 0"},
    {mjITEM_BUTTON,    "Load key",      3},
    {mjITEM_BUTTON,    "Save key",      3},
    {mjITEM_SLIDERNUM, "Noise scale",   5, &this->ctrl_noise_std,  "0 2"},
    {mjITEM_SLIDERNUM, "Noise rate",    5, &this->ctrl_noise_rate, "0 2"},
    {mjITEM_SEPARATOR, "History",       1},
    {mjITEM_SLIDERINT, "",              5, &this->scrub_index,     "0 0"},
    {mjITEM_END}
  };


  // watch section of UI
  const mjuiDef def_watch[5] = {
    {mjITEM_SECTION,   "Watch",         0, nullptr,              "AW"},
    {mjITEM_EDITTXT,   "Field",         2, this->field,          "qpos"},
    {mjITEM_EDITINT,   "Index",         2, &this->index,         "1"},
    {mjITEM_STATIC,    "Value",         2, nullptr,              " "},
    {mjITEM_END}
  };

  // info strings
  char info_title[Simulate::kMaxFilenameLength] = {0};
  char info_content[Simulate::kMaxFilenameLength] = {0};

  // pending uploads
  std::condition_variable_any cond_upload_;
  int texture_upload_ = -1;
  int mesh_upload_ = -1;
  int hfield_upload_ = -1;
};
}  // namespace mujoco

#endif
