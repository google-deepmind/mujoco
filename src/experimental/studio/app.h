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
#include <ratio>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include "experimental/toolbox/helpers.h"
#include "experimental/toolbox/physics.h"
#include "experimental/toolbox/renderer.h"
#include "experimental/toolbox/window.h"

namespace mujoco::studio {

// Owns, updates, and renders a MuJoCo simulation.
class App {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Seconds = std::chrono::duration<double>;
  using Milliseconds = std::chrono::duration<double, std::milli>;

  App(int width, int height, std::string ini_path,
      const toolbox::LoadAssetFn& load_asset_fn);

  // Loads a model into the simulation.
  void LoadModel(std::string model_file);

  // Processes window events and advances the state of the simulation.
  bool Update();

  // Syncs the state of the simulation with the renderer.
  void Sync();

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
    bool classic_ui = true;

    char watch_field[1000] = "qpos";
    int watch_index = 0;
    int camera_idx = 0;
    int key_idx = 0;
    int scrub_idx = 0;
    bool dark_mode = true;

    // UI visibility.
    bool simulation = false;
    bool physics = false;
    bool rendering = false;
    bool watch = false;
    bool visualization = false;
    bool groups = false;
    bool joints = false;
    bool controls = false;
    bool profiler = false;
    bool sensor = false;

    using Dict = std::unordered_map<std::string, std::string>;
    Dict ToDict() const;
    void FromDict(const Dict& dict);
  };

  // UI state that is transient and only needed while the application runs
  // TODO(matijak): Combine this with UiState and identify the list of transient
  // variables with a comment
  struct UiTempState {
    // bool paused = false;  // Application starts with the simulation running.
    bool help = false;
    bool info = false;
    bool modal_open = false;
    bool load_popup = false;
    bool save_xml_popup = false;
    bool save_mjb_popup = false;
    bool save_screenshot_popup = false;
    bool print_model_popup = false;
    bool print_data_popup = false;
    bool should_exit = false;
    // bool should_reload = false;
    bool style_editor = false;
    bool imgui_demo = false;
    bool perturb_active = false;

    int speed_index = 0;

    // Visibility, position and size of the left and right UI panels
    bool show_ui_lhs = true;
    bool show_ui_rhs = true;
    float pos_ui_lhs[2];
    float pos_ui_rhs[2];
    float size_ui_lhs[2];
    float size_ui_rhs[2];

    // Data for StateGui
    int state_sig = 0;
    std::vector<mjtNum> state;

    char filename[1000] = "";
  };

  void OnModelLoaded(std::string_view model_file);

  void LoadSettings();
  void SaveSettings();

  void SetCamera(int idx);

  void ChangeSpeed(int delta);

  void HandleMouseEvents();
  void HandleKeyboardEvents();

  void ClearProfilerData();
  void UpdateProfilerData();

  void BuildGuiWithWindows();
  void BuildGuiWithSections();

  void InfoGui();
  void HelpGui();
  void MainMenuGui();
  void FileDialogGui();
  void SimulationGui();
  void PhysicsGui();
  void RenderingGui();
  void VisualizationGui();
  void GroupsGui();
  void WatchGui();
  void SensorGui();
  void ProfilerGui();
  void StateGui();
  void JointsGui();
  void ControlsGui();

  mjModel* Model() { return physics_->GetModel(); };
  mjData* Data() { return physics_->GetData(); };

  std::string ini_path_;
  std::string model_file_;

  std::unique_ptr<toolbox::Window> window_;
  std::unique_ptr<toolbox::Renderer> renderer_;
  std::unique_ptr<toolbox::Physics> physics_;
  toolbox::LoadAssetFn load_asset_fn_;

  mjvCamera camera_;
  mjvPerturb perturb_;
  mjvOption vis_options_;

  // profiler data
  std::vector<float> cpu_total_;
  std::vector<float> cpu_collision_;
  std::vector<float> cpu_prepare_;
  std::vector<float> cpu_solve_;
  std::vector<float> cpu_other_;
  std::vector<float> dim_dof_;
  std::vector<float> dim_body_;
  std::vector<float> dim_constraint_;
  std::vector<float> dim_sqrt_nnz_;
  std::vector<float> dim_contact_;
  std::vector<float> dim_iteration_;

  UiState ui_;
  UiTempState tmp_;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_APP_H_
