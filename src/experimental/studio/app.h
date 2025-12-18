// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_APP_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_APP_H_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <ratio>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include "experimental/platform/gui.h"
#include "experimental/platform/helpers.h"
#include "experimental/platform/interaction.h"
#include "experimental/platform/renderer.h"
#include "experimental/platform/sim_history.h"
#include "experimental/platform/sim_profiler.h"
#include "experimental/platform/step_control.h"
#include "experimental/platform/window.h"

namespace mujoco::studio {

// Owns, updates, and renders a MuJoCo simulation.
class App {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Seconds = std::chrono::duration<double>;
  using Milliseconds = std::chrono::duration<double, std::milli>;

  App(int width, int height, std::string ini_path,
      const platform::LoadAssetFn& load_asset_fn);

  enum ContentType {
    kFilepath,  // Path to a model file.
    kModelXml,  // XML model string.
    kModelMjb,  // Binary model payload.
  };

  // Loads a model into the simulation.
  //
  // Note: Do not call this function from within Update() (i.e. while drawing
  // the UX). Call RequestModelLoad() instead.
  void LoadModel(std::string data, ContentType type);

  // Processes window events and advances the state of the simulation.
  bool Update();

  // Builds the GUI. We do this after Sync() to ensure we have the latest data
  // for building the GUI.
  void BuildGui();

  // Renders everything (e.g. the simulation and the GUI).
  void Render();

 private:
  static int LoadAssetCallback(const char* path, void* user_data,
                               unsigned char** out, std::uint64_t* out_size);

  // UI state that is persisted across application runs
  struct UiState {
    char watch_field[1000] = "qpos";
    int watch_index = 0;
    int camera_idx = platform::kTumbleCameraIdx;
    int key_idx = 0;
    platform::GuiTheme theme = platform::GuiTheme::kLight;

    using Dict = std::unordered_map<std::string, std::string>;
    Dict ToDict() const;
    void FromDict(const Dict& dict);
  };

  // UI state that is transient and only needed while the application runs
  struct UiTempState {
    bool should_exit = false;
    bool first_frame = true;

    // Windows.
    bool help = false;
    bool stats = false;
    bool chart_cpu_time = false;
    bool chart_dimensions = false;
    bool chart_solver = false;
    bool options_panel = true;
    bool inspector_panel = true;
    bool style_editor = false;
    bool imgui_demo = false;
    bool implot_demo = false;
    bool modal_open = false;
    bool load_popup = false;

    // Controls.
    bool perturb_active = false;
    int speed_index = 0;
    float cam_speed = 0.0f;

    // Cached data.
    float expected_label_width = 0;
    std::vector<std::string> camera_names;
    std::vector<std::string> speed_names;

    // Spec Properties.
    mjsElement* element = nullptr;
    int element_id = -1;

    // State.
    int state_sig = 0;
    std::vector<mjtNum> state;

    // File dialogs.
    char filename[1000] = "";
    std::string last_load_file;
    bool save_xml_popup = false;
    std::string last_save_xml_file;
    bool save_mjb_popup = false;
    std::string last_save_mjb_file;
    bool save_screenshot_popup = false;
    std::string last_save_screenshot_file;
    bool print_model_popup = false;
    std::string last_print_model_file;
    bool print_data_popup = false;
    std::string last_print_data_file;
  };

  void ClearModel();
  void ProcessPendingLoad();
  bool IsModelLoaded() const;
  void RequestModelLoad(std::string model_file);

  void ResetPhysics();
  void UpdatePhysics();

  void LoadSettings();
  void SaveSettings();

  void LoadHistory(int offset);

  void SetSpeedIndex(int idx);

  void HandleMouseEvents();
  void HandleKeyboardEvents();
  void MoveCamera(platform::CameraMotion motion, mjtNum reldx, mjtNum reldy);

  void SetupTheme(platform::GuiTheme theme);

  void MainMenuGui();
  void ToolBarGui();
  void StatusBarGui();
  void HelpGui();
  void FileDialogGui();
  void ModelOptionsGui();
  void DataInspectorGui();
  void SpecExplorerGui();
  void PropertiesGui();

  float GetExpectedLabelWidth();
  std::vector<const char*> GetCameraNames();

  std::string error_;
  std::string ini_path_;
  std::string model_name_;
  std::optional<std::string> pending_load_;

  std::unique_ptr<platform::Window> window_;
  std::unique_ptr<platform::Renderer> renderer_;
  platform::LoadAssetFn load_asset_fn_;
  platform::StepControl step_control_;
  platform::SimProfiler profiler_;
  platform::SimHistory history_;

  std::vector<std::string> search_paths_;

  mjSpec* spec_ = nullptr;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;

  mjvCamera camera_;
  mjvPerturb perturb_;
  mjvOption vis_options_;

  UiState ui_;
  UiTempState tmp_;

  int frames_ = 0;
  TimePoint last_fps_update_;
  double fps_ = 0;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_APP_H_
