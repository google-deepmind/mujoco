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

#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include "experimental/platform/gui.h"
#include "experimental/platform/interaction.h"
#include "experimental/platform/picture_gui.h"
#include "experimental/platform/renderer.h"
#include "experimental/platform/sim_history.h"
#include "experimental/platform/sim_profiler.h"
#include "experimental/platform/step_control.h"
#include "experimental/platform/window.h"

namespace mujoco::studio {

// Owns, updates, and renders a MuJoCo simulation.
class App {
 public:
  // Configuration/initialization options for the application.
  struct Config {
    // The original width and height of the window.
    int width = 0;
    int height = 0;

    // The path to the ini file containing the user settings.
    std::string ini_path;

    // By default, we render directly to the window surface. However, in some
    // cases, we may want to render to an (offscreen) texture and blit the
    // texture to the window surface.
    bool offscreen_mode = false;
  };

  explicit App(Config config);

  // Loads an empty mjModel.
  void InitEmptyModel();

  // Loads an mjModel from the given file. This extension should be one of:
  // .xml, .mjb, or .mjz.
  void LoadModelFromFile(const std::string& filepath);

  // Loads an mjModel from the given memory buffer. The content_type should be
  // one of: "text/xml", "application/mjb", or "application/mjz". For zip files,
  // a name is required in order to uniquely identify the model within the
  // archive.
  void LoadModelFromBuffer(std::span<const std::byte> buffer,
                           std::string_view content_type,
                           std::string_view name);

  // Processes window events and advances the state of the simulation.
  bool Update();

  // Builds the GUI. We do this after Sync() to ensure we have the latest data
  // for building the GUI.
  void BuildGui();

  // Renders everything (e.g. the simulation and the GUI).
  void Render();

 private:
  // The kind of model that is currently loaded.
  enum ModelKind {
    kEmptyModel,
    kModelFromFile,
    kModelFromBuffer,
  };

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
    bool chart_solver = false;
    bool chart_performance = false;
    bool picture_in_picture = false;
    bool options_panel = true;
    bool inspector_panel = true;
    bool full_screen = false;
    bool style_editor = false;
    bool imgui_demo = false;
    bool implot_demo = false;

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

    // Picture-in-Picture.
    std::vector<platform::PipState> pips;

    // File dialogs.
    enum FileDialog {
      FileDialog_None,
      FileDialog_Load,
      FileDialog_SaveXml,
      FileDialog_SaveMjb,
      FileDialog_PrintModel,
      FileDialog_PrintData,
      FileDialog_SaveScreenshot,
      NumFileDialogs,
    };
    FileDialog file_dialog = FileDialog_None;
    std::string last_path[NumFileDialogs];
    char filename[1000] = "";
  };

  // Requests that the model be loaded from the given file at the next update.
  void RequestModelLoad(std::string model_file);

  // Requests that the currently loaded model be reloaded at the next update.
  void RequestModelReload();

  // Clears the currently loaded model and all associated state.
  void ClearModel();

  // Updates the currently loaded model to the given model. If model is null,
  // then compile the spec to a model.
  void InitModel(mjModel* model, mjSpec* spec, mjVFS* vfs, std::string filename,
                 ModelKind model_kind);

  void SetLoadError(std::string error);
  void UpdateFilePaths(const std::string& resolved_path);

  void ResetPhysics();
  void UpdatePhysics();

  void LoadSettings();
  void SaveSettings();

  void LoadHistory(int offset);

  void SetSpeedIndex(int idx);

  void HandleWindowEvents();
  void HandleMouseEvents();
  void HandleKeyboardEvents();

  void ProcessPendingLoads();

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

  mjSpec* spec() { return spec_; }
  mjModel* model() { return model_; }
  mjData* data() { return data_; }
  bool has_spec() const { return spec_ != nullptr; }
  bool has_model() const { return model_ != nullptr; }
  bool has_data() const { return data_ != nullptr; }

  std::string ini_path_;
  std::string model_name_;  // Used if model_kind_ is kModelFromBuffer.
  std::string model_path_;
  std::string load_error_;
  std::string step_error_;
  std::optional<std::string> pending_load_;
  ModelKind model_kind_ = kEmptyModel;

  std::unique_ptr<platform::Window> window_;
  std::unique_ptr<platform::Renderer> renderer_;
  platform::StepControl step_control_;
  platform::SimProfiler profiler_;
  platform::SimHistory history_;

  std::vector<std::string> search_paths_;

  mjSpec* spec_ = nullptr;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::vector<std::byte> pixels_;

  mjvCamera camera_;
  mjvPerturb perturb_;
  mjvOption vis_options_;

  UiState ui_;
  UiTempState tmp_;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_APP_H_
