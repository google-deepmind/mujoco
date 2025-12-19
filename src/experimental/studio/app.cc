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

#include "experimental/studio/app.h"

#include <algorithm>
#include <array>
#include <cfloat>
#if defined(USE_CLASSIC_OPENGL)
#include <chrono>
#endif
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/gui.h"
#include "experimental/platform/helpers.h"
#include "experimental/platform/imgui_widgets.h"
#include "experimental/platform/interaction.h"
#include "experimental/platform/renderer.h"
#include "experimental/platform/step_control.h"
#include "experimental/platform/window.h"

#if defined(USE_FILAMENT_OPENGL) || defined(USE_FILAMENT_VULKAN)
#include "experimental/filament/render_context_filament.h"
#elif defined(USE_CLASSIC_OPENGL)
#include <backends/imgui_impl_opengl3.h>
#else
#error No rendering mode defined.
#endif

namespace mujoco::studio {

static constexpr platform::Window::Config kWindowConfig = {
#if defined(__EMSCRIPTEN__)
    .render_config = platform::Window::RenderConfig::kFilamentWebGL,
#elif defined(USE_FILAMENT_VULKAN)
    .render_config = platform::Window::RenderConfig::kFilamentVulkan,
#elif defined(USE_FILAMENT_OPENGL)
    .render_config = platform::Window::RenderConfig::kFilamentOpenGL,
#elif defined(USE_CLASSIC_OPENGL)
    .render_config = platform::Window::RenderConfig::kClassicOpenGL,
#endif
    .enable_keyboard = true,
};

static void ToggleFlag(mjtByte& flag) { flag = flag ? 0 : 1; }

static void ToggleWindow(bool& window) {
  window = !window;
  ImGui::GetIO().WantSaveIniSettings = true;
}

static void ShowPopup(bool& popup) { popup = true; }

static void SelectParentPerturb(const mjModel* model, mjvPerturb& perturb) {
  if (perturb.select > 0) {
    perturb.select = model->body_parentid[perturb.select];
    perturb.flexselect = -1;
    perturb.skinselect = -1;
    if (perturb.select <= 0) {
      perturb.active = 0;
    }
  }
}

static constexpr const char* ICON_PLAY = platform::ICON_FA_PLAY;
static constexpr const char* ICON_PAUSE = platform::ICON_FA_PAUSE;
static constexpr const char* ICON_COPY_CAMERA = platform::ICON_FA_COPY;
static constexpr const char* ICON_UNLOAD_MODEL = platform::ICON_FA_EJECT;
static constexpr const char* ICON_RELOAD_MODEL = platform::ICON_FA_REFRESH;
static constexpr const char* ICON_LABEL = platform::ICON_FA_COMMENT;
static constexpr const char* ICON_RESET_MODEL = platform::ICON_FA_UNDO;
static constexpr const char* ICON_FRAME = platform::ICON_FA_ARROWS;
static constexpr const char* ICON_CAMERA = platform::ICON_FA_CAMERA;
static constexpr const char* ICON_DARKMODE = platform::ICON_FA_MOON;
static constexpr const char* ICON_LIGHTMODE = platform::ICON_FA_SUN;
static constexpr const char* ICON_CLASSICMODE = platform::ICON_FA_DIAMOND;
static constexpr const char* ICON_PREV_FRAME = platform::ICON_FA_CARET_LEFT;
static constexpr const char* ICON_NEXT_FRAME = platform::ICON_FA_CARET_RIGHT;
static constexpr const char* ICON_CURR_FRAME = platform::ICON_FA_FAST_FORWARD;
static constexpr const char* ICON_SPEED = platform::ICON_FA_TACHOMETER;

// UI labels for mjtLabel.
static constexpr const char* kLabelNames[] = {
    "None",      "Body",    "Joint",    "Geom",       "Site",  "Camera",
    "Light",     "Tendon",  "Actuator", "Constraint", "Flex",  "Skin",
    "Selection", "Sel Pnt", "Contact",  "Force",      "Island"};

// UI labels for mjtFrame.
static constexpr const char* kFrameNames[] = {
    "None", "Body", "Geom", "Site", "Camera", "Light", "Contact", "World"};

// logarithmically spaced real-time slow-down coefficients (percent)
// clang-format off
static constexpr std::array<const char*, 31> kPercentRealTime = {
"100.0 ", " 80.0 ", " 66.0 ", " 50.0 ", " 40.0 ", " 33.0 ", " 25.0 ", " 20.0 ", " 16.0 ", " 13.0 ",
" 10.0 ", "  8.0 ", "  6.6 ", "  5.0 ", "  4.0 ", "  3.3 ", "  2.5 ", "  2.0 ", "  1.6 ", "  1.3 ",
"  1.0 ", "  0.8 ", "  0.7 ", "  0.5 ", "  0.4 ", "  0.33", "  0.25", "  0.2 ", "  0.16", "  0.13",
"  0.1 ",
};
// clang-format on

App::App(int width, int height, std::string ini_path,
         const platform::LoadAssetFn& load_asset_fn)
    : ini_path_(std::move(ini_path)), load_asset_fn_(load_asset_fn) {
  window_ = std::make_unique<platform::Window>("MuJoCo Studio", width, height,
                                               kWindowConfig, load_asset_fn);
  ImPlot::CreateContext();

  auto make_context_fn = [&](const mjModel* m, mjrContext* con) {
#if defined(USE_CLASSIC_OPENGL)
    mjr_makeContext(m, con, mjFONTSCALE_150);
#else
    mjrFilamentConfig render_config;
    mjr_defaultFilamentConfig(&render_config);
    render_config.native_window = window_->GetNativeWindowHandle();
    render_config.load_asset = &App::LoadAssetCallback;
    render_config.load_asset_user_data = this;
    render_config.enable_gui = true;
#if defined(USE_FILAMENT_OPENGL)
    render_config.graphics_api = mjGFX_OPENGL;
#elif defined(USE_FILAMENT_VULKAN)
    render_config.graphics_api = mjGFX_VULKAN;
#endif
    mjr_makeFilamentContext(m, con, &render_config);
#endif
  };
  renderer_ = std::make_unique<platform::Renderer>(make_context_fn);

  mjv_defaultPerturb(&perturb_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&vis_options_);

  profiler_.Clear();

#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_Init();
#endif
}

void App::ClearModel() {
  if (model_) {
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;

    if (spec_) {
      mj_deleteSpec(spec_);
      spec_ = nullptr;
    }
  }

  step_control_.SetSpeed(100.f);
  profiler_.Clear();
  tmp_ = UiTempState();
  error_ = "";
}

void App::RequestModelLoad(std::string model_file) {
  if (model_file.starts_with('[') || model_file.ends_with(']')) {
    pending_load_ = "";
  } else {
    pending_load_ = std::move(model_file);
  }
}

void App::LoadModel(std::string data, ContentType type) {
  // Delete the existing mjModel and mjData.
  ClearModel();

  if (!data.empty()) {
    char err[1000] = "";
    if (type == ContentType::kFilepath) {
      // Store the file path as the model name. Note that we use this model name
      // to perform reload operations.
      model_name_ = std::move(data);
      const std::string resolved_file =
        platform::ResolveFile(model_name_, search_paths_);
    if (resolved_file.ends_with(".mjb")) {
        model_ = mj_loadModel(resolved_file.c_str(), 0);
      } else if (resolved_file.ends_with(".xml")) {
        spec_ = mj_parseXML(resolved_file.c_str(), nullptr, err, sizeof(err));
        if (spec_ && err[0] == 0) {
          model_ = mj_compile(spec_, nullptr);
        }
      } else {
        error_ = "Unknown model file type; expected .mjb or .xml.";
      }
    } else if (type == ContentType::kModelXml) {
      model_name_ = "[xml]";
      spec_ = mj_parseXMLString(data.c_str(), nullptr, err, sizeof(err));
      if (spec_ && err[0] == 0) {
        model_ = mj_compile(spec_, nullptr);
      }
    } else if (type == ContentType::kModelMjb) {
      model_name_ = "[mjb]";
      model_ = mj_loadModelBuffer(data.data(), data.size());
    }

    if (err[0]) {
      error_ = err;
    }
  }

  // If no mjModel was loaded, load an empty mjModel.
  if (model_name_.empty() || model_ == nullptr) {
    spec_ = mj_makeSpec();
    model_ = mj_compile(spec_, 0);
    model_name_ = "";
  }
  if (!model_) {
    mju_error("Error loading model: %s", error_.c_str());
  }

  // Create the mjData for the mjModel.
  data_ = mj_makeData(model_);
  if (!data_) {
    mju_error("Error making data for model: %s", error_.c_str());
  }

  // Reset/reinitialize everything that depends on the new mjModel.
  renderer_->Init(model_);
  const int state_size = mj_stateSize(model_, mjSTATE_INTEGRATION);
  history_.Init(state_size);

  // Initialize the speed based on the model's default real-time setting.
  float min_error = FLT_MAX;
  const float desired = mju_log(100 * model_->vis.global.realtime);
  for (int i = 0; i < kPercentRealTime.size(); ++i) {
    const float speed = std::stof(kPercentRealTime[i]);
    const float error = mju_abs(mju_log(speed) - desired);
    if (error < min_error) {
      min_error = error;
      SetSpeedIndex(i);
    }
  }

  // Update the window title and update the file paths for saving files related
  // to the loaded model.
  std::string base_path = "/";
  std::string model_name = "model";
  if (!model_name_.empty() &&
      (model_name_.ends_with(".xml") || model_name_.ends_with(".mjb"))) {
    window_->SetTitle("MuJoCo Studio : " + model_name_);
    tmp_.last_load_file = std::string(model_name_);
    std::filesystem::path path(model_name_);
    base_path = path.parent_path().string() + "/";
    model_name = path.stem().string();
  } else {
    window_->SetTitle("MuJoCo Studio");
    tmp_.last_load_file = base_path;
  }

  tmp_.last_save_mjb_file = base_path + model_name + "_saved.mjb";
  tmp_.last_save_xml_file = base_path + model_name + "_saved.xml";
  tmp_.last_print_model_file = base_path + model_name + "_MJMODEL.TXT";
  tmp_.last_print_data_file = base_path + model_name + "_MJDATA.TXT";
  tmp_.last_save_screenshot_file = base_path + "screenshot.webp";
}

bool App::IsModelLoaded() const { return !model_name_.empty(); }

void App::ResetPhysics() {
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  error_ = "";
}

void App::UpdatePhysics() {
  if (pending_load_.has_value()) {
    std::string model_file = std::move(pending_load_.value());
    pending_load_.reset();
    LoadModel(model_file, ContentType::kFilepath);
  }
  if (!IsModelLoaded()) {
    return;
  }

  if (!step_control_.IsPaused()) {
    mju_zero(data_->xfrc_applied, 6 * model_->nbody);
    mjv_applyPerturbPose(model_, data_, &perturb_, 0);
    mjv_applyPerturbForce(model_, data_, &perturb_);
  } else {
    mjv_applyPerturbPose(model_, data_, &perturb_, 1);
  }

  if (data_) {
    for (int i = 0; i < mjNTIMER; i++) {
      data_->timer[i].duration = 0;
      data_->timer[i].number = 0;
    }
  }

  bool stepped = false;

  platform::StepControl::Status status = step_control_.Advance(model_, data_);
  if (status == platform::StepControl::Status::kPaused) {
    // do nothing
  } else if (status == platform::StepControl::Status::kOk) {
    stepped = true;
    // If we are adding to the history we didn't have a divergence error
    error_ = "";
  } else if (status == platform::StepControl::Status::kAutoReset) {
    ResetPhysics();
  } else if (status == platform::StepControl::Status::kDiverged) {
    stepped = true;
    for (mjtWarning w : platform::StepControl::kDivergedWarnings) {
      if (data_->warning[w].number > 0) {
        error_ = mju_warningText(w, data_->warning[w].lastinfo);
      }
    }
  }

  if (stepped) {
    profiler_.Update(model_, data_);
    std::span<mjtNum> state = history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    }
  }
}

void App::LoadHistory(int offset) {
  std::span<mjtNum> state = history_.SetIndex(offset);
  if (!state.empty()) {
    // Pause simulation when entering history mode.
    step_control_.Pause();

    // Load the state into the data buffer.
    mj_setState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    mj_forward(model_, data_);
  }
}

bool App::Update() {
  const platform::Window::Status status = window_->NewFrame();

#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_NewFrame();
#endif

  HandleMouseEvents();
  HandleKeyboardEvents();

  // Check to see if a model was dropped on the window.
  const std::string drop_file = window_->GetDropFile();
  if (!drop_file.empty()) {
    RequestModelLoad(drop_file);
  }

  // Only update the simulation if a popup window is not open. Note that the
  // simulation itself will only update if it is not paused.
  if (!tmp_.modal_open) {
    UpdatePhysics();
  }

  return status == platform::Window::Status::kRunning;
}

void App::Render() {
  const float width = window_->GetWidth();
  const float height = window_->GetHeight();
  const float scale = window_->GetScale();

  renderer_->Render(model_, data_, &perturb_, &camera_, &vis_options_,
                    width * scale, height * scale);

#ifdef USE_CLASSIC_OPENGL
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif

  // This call to EndFrame() is only needed if render_config.enable_gui is false
  window_->EndFrame();
  window_->Present();

  if (data_) {
    for (int i = 0; i < mjNTIMER; i++) {
      data_->timer[i].duration = 0;
      data_->timer[i].number = 0;
    }
  }

#ifdef USE_CLASSIC_OPENGL
  TimePoint now = std::chrono::steady_clock::now();
  TimePoint::duration delta_time = now - last_fps_update_;
  const double interval = std::chrono::duration<double>(delta_time).count();

  ++frames_;
  if (interval > 0.2) {  // only update FPS stat at most 5 times per second
    last_fps_update_ = now;
    fps_ = frames_ / interval;
    frames_ = 0;
  }
#else
  fps_ = mjr_getFrameRate(&renderer_->GetContext());
#endif
}

void App::HandleMouseEvents() {
  auto& io = ImGui::GetIO();
  if (io.WantCaptureMouse) {
    return;
  }

  if (!model_ || !data_) {
    return;
  }

  // Normalize mouse positions and movement to display size.
  const float mouse_x = io.MousePos.x / io.DisplaySize.x;
  const float mouse_y = io.MousePos.y / io.DisplaySize.y;
  const float mouse_dx = io.MouseDelta.x / io.DisplaySize.x;
  const float mouse_dy = io.MouseDelta.y / io.DisplaySize.y;
  const float mouse_scroll = io.MouseWheel / 50.0f;
  const bool is_mouse_moving = (mouse_dx != 0.0f || mouse_dy != 0.0f);
  const bool is_any_mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left) ||
                                 ImGui::IsMouseDown(ImGuiMouseButton_Right) ||
                                 ImGui::IsMouseDown(ImGuiMouseButton_Middle);
  const bool is_mouse_dragging = is_mouse_moving && is_any_mouse_down;

  // If no mouse buttons are down, end any active perturbations.
  if (!is_any_mouse_down) {
    perturb_.active = 0;
  }

  // Handle perturbation mouse actions.
  if (is_mouse_dragging && io.KeyCtrl) {
    if (perturb_.select > 0) {
      mjtMouse action = mjMOUSE_NONE;
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        action = mjMOUSE_ZOOM;
      }
      const mjtPertBit active =
          action == mjMOUSE_MOVE_V ? mjPERT_TRANSLATE : mjPERT_ROTATE;
      if (active != perturb_.active) {
        platform::InitPerturb(model_, data_, &camera_, &perturb_, active);
      }
      platform::MovePerturb(model_, data_, &camera_, &perturb_, action,
                            mouse_dx, mouse_dy);
    }
  }
  // Handle camera movement actions.
  else if (is_mouse_dragging) {
    if (ui_.camera_idx == platform::kFreeCameraIdx) {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        MoveCamera(platform::CameraMotion::PAN_TILT, mouse_dx, mouse_dy);
      }
    } else {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        MoveCamera(platform::CameraMotion::ORBIT, mouse_dx, mouse_dy);
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        MoveCamera(platform::CameraMotion::ZOOM, mouse_dx, mouse_dy);
      }
    }

    // Right mouse movement is relative to the horizontal and vertical planes.
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && io.KeyShift) {
      MoveCamera(platform::CameraMotion::PLANAR_MOVE_H, mouse_dx, mouse_dy);
    } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
      MoveCamera(platform::CameraMotion::PLANAR_MOVE_V, mouse_dx, mouse_dy);
    }
  }

  // Mouse scroll zooms the camera towards/away from the lookat point.
  // Ignored by user-centered free cameras which don't have a lookat point.
  if (mouse_scroll != 0.0f && ui_.camera_idx != platform::kFreeCameraIdx) {
    MoveCamera(platform::CameraMotion::ZOOM, 0, -mouse_scroll);
  }

  // Left double click.
  if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
    platform::PickResult picked =
        platform::Pick(model_, data_, &camera_, mouse_x, mouse_y,
                       window_->GetAspectRatio(), &vis_options_);
    if (picked.body >= 0) {
      perturb_.select = picked.body;
      perturb_.flexselect = picked.flex;
      perturb_.skinselect = picked.skin;

      // Compute the local position of the selected object in the world.
      mjtNum tmp[3];
      mju_sub3(tmp, picked.point, data_->xpos + 3 * picked.body);
      mju_mulMatTVec(perturb_.localpos, data_->xmat + 9 * picked.body, tmp, 3,
                     3);
    } else {
      perturb_.select = 0;
      perturb_.flexselect = -1;
      perturb_.skinselect = -1;
    }
  }

  // Right double click.
  if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right)) {
    platform::PickResult picked =
        platform::Pick(model_, data_, &camera_, mouse_x, mouse_y,
                       window_->GetAspectRatio(), &vis_options_);
    mju_copy3(camera_.lookat, picked.point);
    if (picked.body > 0 && io.KeyCtrl) {
      // Switch camera to tracking mode and track the selected body.
      camera_.type = mjCAMERA_TRACKING;
      camera_.trackbodyid = picked.body;
      camera_.fixedcamid = -1;
      ui_.camera_idx = platform::kTrackingCameraIdx;
    }
  }
}

void App::HandleKeyboardEvents() {
  using platform::ImGui_IsChordJustPressed;
  if (ImGui::GetIO().WantCaptureKeyboard) {
    return;
  }

  constexpr auto ImGuiMode_CtrlShift = ImGuiMod_Ctrl | ImGuiMod_Shift;

  bool is_freecam_wasd = ui_.camera_idx == platform::kFreeCameraIdx;

  // Menu shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_O | ImGuiMod_Ctrl)) {
    ShowPopup(tmp_.load_popup);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMode_CtrlShift)) {
    ShowPopup(tmp_.save_mjb_popup);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMod_Ctrl)) {
    ShowPopup(tmp_.save_xml_popup);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_M | ImGuiMod_Ctrl)) {
    ShowPopup(tmp_.print_model_popup);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_D | ImGuiMod_Ctrl)) {
    ShowPopup(tmp_.print_data_popup);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_P | ImGuiMod_Ctrl)) {
    ShowPopup(tmp_.save_screenshot_popup);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C | ImGuiMod_Ctrl)) {
    std::string keyframe = platform::KeyframeToString(model_, data_, false);
    platform::MaybeSaveToClipboard(keyframe);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_L | ImGuiMod_Ctrl)) {
    RequestModelLoad(model_name_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q | ImGuiMod_Ctrl)) {
    tmp_.should_exit = true;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_A | ImGuiMod_Ctrl)) {
    mjv_defaultFreeCamera(model_, &camera_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab | ImGuiMod_Shift)) {
    tmp_.inspector_panel = !tmp_.inspector_panel;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab)) {
    tmp_.options_panel = !tmp_.options_panel;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Minus)) {
    SetSpeedIndex(tmp_.speed_index + 1);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Equal)) {
    SetSpeedIndex(tmp_.speed_index - 1);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftArrow)) {
    if (step_control_.IsPaused()) {
      LoadHistory(history_.GetIndex() - 1);
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_RightArrow)) {
    if (step_control_.IsPaused()) {
      if (history_.GetIndex() == 0) {
        step_control_.RequestSingleStep();
      } else {
        LoadHistory(history_.GetIndex() + 1);
      }
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Space)) {
    step_control_.TogglePause();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backspace)) {
    ResetPhysics();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_PageUp)) {
    SelectParentPerturb(model_, perturb_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F1)) {
    ToggleWindow(tmp_.help);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F2)) {
    ToggleWindow(tmp_.stats);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F6)) {
    vis_options_.frame = (vis_options_.frame + 1) % mjNFRAME;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F7)) {
    vis_options_.label = (vis_options_.label + 1) % mjNLABEL;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F9)) {
    tmp_.chart_solver = !tmp_.chart_solver;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F10)) {
    tmp_.chart_cpu_time = !tmp_.chart_cpu_time;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F11)) {
    tmp_.chart_dimensions = !tmp_.chart_dimensions;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_H)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONVEXHULL]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_X)) {
    ToggleFlag(vis_options_.flags[mjVIS_TEXTURE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_J)) {
    ToggleFlag(vis_options_.flags[mjVIS_JOINT]);
  } else if (!is_freecam_wasd && ImGui_IsChordJustPressed(ImGuiKey_Q)) {
    ToggleFlag(vis_options_.flags[mjVIS_CAMERA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_U)) {
    ToggleFlag(vis_options_.flags[mjVIS_ACTUATOR]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Comma)) {
    ToggleFlag(vis_options_.flags[mjVIS_ACTIVATION]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Z)) {
    ToggleFlag(vis_options_.flags[mjVIS_LIGHT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_V)) {
    ToggleFlag(vis_options_.flags[mjVIS_TENDON]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Y)) {
    ToggleFlag(vis_options_.flags[mjVIS_RANGEFINDER]);
  } else if (!is_freecam_wasd && ImGui_IsChordJustPressed(ImGuiKey_E)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONSTRAINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_I)) {
    ToggleFlag(vis_options_.flags[mjVIS_INERTIA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Apostrophe)) {
    ToggleFlag(vis_options_.flags[mjVIS_SCLINERTIA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_B)) {
    ToggleFlag(vis_options_.flags[mjVIS_PERTFORCE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_O)) {
    ToggleFlag(vis_options_.flags[mjVIS_PERTOBJ]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONTACTPOINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_N)) {
    ToggleFlag(vis_options_.flags[mjVIS_ISLAND]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONTACTFORCE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_P)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONTACTSPLIT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_T)) {
    ToggleFlag(vis_options_.flags[mjVIS_TRANSPARENT]);
  } else if (!is_freecam_wasd && ImGui_IsChordJustPressed(ImGuiKey_A)) {
    ToggleFlag(vis_options_.flags[mjVIS_AUTOCONNECT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_M)) {
    ToggleFlag(vis_options_.flags[mjVIS_COM]);
  } else if (!is_freecam_wasd && ImGui_IsChordJustPressed(ImGuiKey_D)) {
    ToggleFlag(vis_options_.flags[mjVIS_STATIC]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Semicolon)) {
    ToggleFlag(vis_options_.flags[mjVIS_SKIN]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_GraveAccent)) {
    ToggleFlag(vis_options_.flags[mjVIS_BODYBVH]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backslash)) {
    ToggleFlag(vis_options_.flags[mjVIS_MESHBVH]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_0 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options_.sitegroup[0]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_1 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options_.sitegroup[1]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_2 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options_.sitegroup[2]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_3 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options_.sitegroup[3]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_4 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options_.sitegroup[4]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_5 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options_.sitegroup[5]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_0)) {
    ToggleFlag(vis_options_.geomgroup[0]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_1)) {
    ToggleFlag(vis_options_.geomgroup[1]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_2)) {
    ToggleFlag(vis_options_.geomgroup[2]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_3)) {
    ToggleFlag(vis_options_.geomgroup[3]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_4)) {
    ToggleFlag(vis_options_.geomgroup[4]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_5)) {
    ToggleFlag(vis_options_.geomgroup[5]);
  } else if (model_) {
    if (ImGui_IsChordJustPressed(ImGuiKey_Escape)) {
      ui_.camera_idx =
          platform::SetCamera(model_, &camera_, platform::kTumbleCameraIdx);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftBracket)) {
      ui_.camera_idx =
          platform::SetCamera(model_, &camera_, ui_.camera_idx - 1);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_RightBracket)) {
      ui_.camera_idx =
          platform::SetCamera(model_, &camera_, ui_.camera_idx + 1);
    }

    // WASD camera controls for free camera.
    if (is_freecam_wasd) {
      bool moved = false;

      // Move (dolly) forward/backward using W and S keys.
      if (ImGui::IsKeyDown(ImGuiKey_W)) {
        MoveCamera(platform::CameraMotion::TRUCK_DOLLY, 0, tmp_.cam_speed);
        moved = true;
      } else if (ImGui::IsKeyDown(ImGuiKey_S)) {
        MoveCamera(platform::CameraMotion::TRUCK_DOLLY, 0, -tmp_.cam_speed);
        moved = true;
      }

      // Strafe (truck) left/right using A and D keys.
      if (ImGui::IsKeyDown(ImGuiKey_A)) {
        MoveCamera(platform::CameraMotion::TRUCK_DOLLY, -tmp_.cam_speed, 0);
        moved = true;
      } else if (ImGui::IsKeyDown(ImGuiKey_D)) {
        MoveCamera(platform::CameraMotion::TRUCK_DOLLY, tmp_.cam_speed, 0);
        moved = true;
      }

      // Move (pedestal) up/down using Q and E keys.
      if (ImGui::IsKeyDown(ImGuiKey_Q)) {
        MoveCamera(platform::CameraMotion::TRUCK_PEDESTAL, 0, tmp_.cam_speed);
        moved = true;
      } else if (ImGui::IsKeyDown(ImGuiKey_E)) {
        MoveCamera(platform::CameraMotion::TRUCK_PEDESTAL, 0, -tmp_.cam_speed);
        moved = true;
      }

      if (moved) {
        tmp_.cam_speed += 0.001f;

        const float max_speed = ImGui::GetIO().KeyShift ? 0.1 : 0.01f;
        if (tmp_.cam_speed > max_speed) {
          tmp_.cam_speed = max_speed;
        }
      } else {
        tmp_.cam_speed = 0.001f;
      }
    }
  }
}

void App::LoadSettings() {
  if (!ini_path_.empty()) {
    std::string settings = platform::LoadText(ini_path_);
    if (!settings.empty()) {
      ui_.FromDict(platform::ReadIniSection(settings, "[Studio][UX]"));
      ImGui::LoadIniSettingsFromMemory(settings.data(), settings.size());
    }
  }
}

void App::SaveSettings() {
  if (!ini_path_.empty()) {
    std::string settings = ImGui::SaveIniSettingsToMemory();
    platform::AppendIniSection(settings, "[Studio][UX]", ui_.ToDict());
    platform::SaveText(settings, ini_path_);
  }
}

void App::SetSpeedIndex(int idx) {
  if (idx == tmp_.speed_index || kPercentRealTime.empty()) {
    return;
  }

  tmp_.speed_index = std::clamp<int>(idx, 0, kPercentRealTime.size() - 1);
  float speed = std::stof(kPercentRealTime[tmp_.speed_index]);
  step_control_.SetSpeed(speed);
}

void App::MoveCamera(platform::CameraMotion motion, mjtNum reldx,
                     mjtNum reldy) {
  platform::MoveCamera(model_, data_, &camera_, motion, reldx, reldy);
}

void App::BuildGui() {
  SetupTheme(ui_.theme);
  const ImVec4 workspace_rect = platform::ConfigureDockingLayout();

  // Place charts in bottom right corner of the workspace.
  const ImVec2 chart_size(250, 250);
  const ImVec2 chart_pos(workspace_rect.x + workspace_rect.z - chart_size.x,
                         workspace_rect.y + workspace_rect.w - chart_size.y);

  MainMenuGui();

  if (ImGui::Begin("ToolBar")) {
    ToolBarGui();
  }
  ImGui::End();

  {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_CellPadding, ImVec2(0, 0));
    style.Var(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
    style.Var(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    if (ImGui::Begin("StatusBar")) {
      StatusBarGui();
    }
    ImGui::End();
  }

  if (tmp_.options_panel) {
    if (ImGui::Begin("Options", &tmp_.options_panel)) {
      ModelOptionsGui();
    }
    ImGui::End();
  }

  if (tmp_.inspector_panel) {
    if (ImGui::Begin("Inspector", &tmp_.inspector_panel)) {
      DataInspectorGui();
    }
    ImGui::End();

    bool explorer_is_open = false;
    if (ImGui::Begin("Explorer", &tmp_.inspector_panel)) {
      explorer_is_open = true;
      SpecExplorerGui();
    }
    ImGui::End();

    if (explorer_is_open && tmp_.element != nullptr) {
      if (ImGui::Begin("Properties")) {
        PropertiesGui();
      }
      ImGui::End();
    }
  }

  if (tmp_.chart_cpu_time) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Cpu Time", &tmp_.chart_cpu_time)) {
      profiler_.CpuTimeGraph();
    }
    ImGui::End();
  }

  if (tmp_.chart_dimensions) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Dimensions", &tmp_.chart_dimensions)) {
      profiler_.DimensionsGraph();
    }
    ImGui::End();
  }

  if (tmp_.chart_solver) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Solver", &tmp_.chart_solver)) {
      platform::CountsGui(model_, data_);
      platform::ConvergenceGui(model_, data_);
    }
    ImGui::End();
  }

  if (tmp_.help) {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_Alpha, 0.6f);
    ImGui::SetNextWindowPos(ImVec2(workspace_rect.x, workspace_rect.y),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Help", &tmp_.help)) {
      HelpGui();
    }
    ImGui::End();
  }

  if (tmp_.stats) {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_Alpha, 0.6f);
    if (ImGui::Begin("Stats", &tmp_.stats)) {
      platform::StatsGui(model_, data_, step_control_.IsPaused(), fps_);
    }
    ImGui::End();
  }

  // Display a drag-and-drop message if no model is loaded.
  if (!IsModelLoaded()) {
    const char* text = "Load model file or drag-and-drop model file here.";

    const float width = window_->GetWidth();
    const float height = window_->GetHeight();
    const ImVec2 text_size = ImGui::CalcTextSize(text);

    ImGui::SetNextWindowPos(ImVec2((width - text_size.x) / 2, height / 2),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(text_size.x + 10, text_size.y + 10),
                             ImGuiCond_Always);
    const ImGuiWindowFlags kOverlayFlags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoDocking;
    if (ImGui::Begin("##Overlay", 0, kOverlayFlags)) {
      ImGui::Text("%s", text);
    }
    ImGui::End();
  }

  FileDialogGui();

  if (tmp_.imgui_demo) {
    ImGui::ShowDemoWindow();
  }
  if (tmp_.implot_demo) {
    ImPlot::ShowDemoWindow();
  }
  if (tmp_.style_editor) {
    if (ImGui::Begin("Style Editor", &tmp_.style_editor)) {
      ImGui::ShowStyleEditor();
    }
    ImGui::End();
  }

  ImGuiIO& io = ImGui::GetIO();
  if (tmp_.first_frame) {
    LoadSettings();
    tmp_.first_frame = false;
  }
  if (io.WantSaveIniSettings) {
    SaveSettings();
    io.WantSaveIniSettings = false;
  }
}

void App::SetupTheme(platform::GuiTheme theme) {
  if (!tmp_.style_editor) {
    platform::SetupTheme(theme);
    ui_.theme = theme;
    ImGui::GetIO().WantSaveIniSettings = true;
  }
}

void App::ModelOptionsGui() {
  const float min_width = GetExpectedLabelWidth();
  const ImGuiChildFlags child_flags =
      ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysAutoResize;
  const ImGuiTreeNodeFlags node_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  ImGui::BeginChild("PhysicsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Physics Settings", node_flags)) {
    platform::PhysicsGui(model_, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("RenderingGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Rendering Settings", node_flags)) {
    platform::RenderingGui(model_, &vis_options_, renderer_->GetRenderFlags(),
                           min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("GroupsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Visibility Groups", node_flags)) {
    platform::GroupsGui(model_, &vis_options_, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("VisualizationGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Visualization", node_flags)) {
    platform::VisualizationGui(model_, &vis_options_, &camera_, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();
}

void App::DataInspectorGui() {
  if (data_ == nullptr) {
    ImGui::Text("No mjData loaded.");
    return;
  }

  const float min_width = GetExpectedLabelWidth();
  const ImGuiChildFlags child_flags =
      ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysAutoResize;
  const ImGuiTreeNodeFlags node_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  ImGui::BeginChild("NoiseGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Noise", node_flags)) {
    float noise_scale = 0;
    float noise_rate = 0;
    step_control_.GetNoiseParameters(noise_scale, noise_rate);
    platform::NoiseGui(model_, data_, noise_scale, noise_rate);
    step_control_.SetNoiseParameters(noise_scale, noise_rate);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("JointsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Joints", node_flags)) {
    platform::JointsGui(model_, data_, &vis_options_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("ControlsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Controls", node_flags)) {
    platform::ControlsGui(model_, data_, &vis_options_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("SensorGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Sensor", node_flags)) {
    platform::SensorGui(model_, data_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("WatchGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Watch", node_flags)) {
    platform::WatchGui(model_, data_, ui_.watch_field, sizeof(ui_.watch_field),
                       ui_.watch_index);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("StateGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("State", node_flags)) {
    platform::StateGui(model_, data_, tmp_.state, tmp_.state_sig, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();
}

void DisplayElementTree(mjsElement* element) {
  const mjString* name = mjs_getName(element);
  if (name->empty()) {
    ImGui::Text("(unnamed)");
  } else {
    ImGui::Text("%s", name->c_str());
  }
}

void App::SpecExplorerGui() {
  if (spec_ == nullptr) {
    ImGui::Text("No mjSpec loaded.");
    return;
  }

  const ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  auto display_group = [this](mjtObj type, const std::string& prefix) {
    mjsElement* element = mjs_firstElement(spec_, type);
    while (element) {
      const int id = mjs_getId(element);

      const mjString* name = mjs_getName(element);
      std::string label = *name;
      if (label.empty()) {
        label = "(" + prefix + " " + std::to_string(id) + ")";
      }

      if (ImGui::Selectable(label.c_str(), false)) {
        tmp_.element = element;
        tmp_.element_id = id;
      }

      element = mjs_nextElement(spec_, element);
    }
  };

  if (ImGui::TreeNodeEx("Bodies", flags)) {
    // We don't use `display_group` here because we do additional selection
    // logic tied to the `perturb_` field.
    mjsElement* element = mjs_firstElement(spec_, mjOBJ_BODY);
    while (element) {
      const int id = mjs_getId(element);

      const mjString* name = mjs_getName(element);
      std::string label = *name;
      if (label.empty()) {
        label = "(Body " + std::to_string(id) + ")";
      }

      if (ImGui::Selectable(label.c_str(), (id == perturb_.select),
                            ImGuiSelectableFlags_AllowDoubleClick)) {
        tmp_.element = element;
        tmp_.element_id = id;
      }
      if (ImGui::IsItemHovered() &&
          ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        perturb_.select = id;
      }

      element = mjs_nextElement(spec_, element);
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Joints", flags)) {
    display_group(mjOBJ_JOINT, "Joint");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Sites", flags)) {
    display_group(mjOBJ_SITE, "Site");
    ImGui::TreePop();
  }
}

void App::PropertiesGui() {
  if (tmp_.element == nullptr) {
    ImGui::Text("No element selected.");
    return;
  }

  switch (tmp_.element->elemtype) {
    case mjOBJ_BODY:
      ImGui::Text("Body");
      ImGui::Separator();
      platform::BodyPropertiesGui(model_, data_, tmp_.element, tmp_.element_id);
      break;
    case mjOBJ_JOINT:
      ImGui::Text("Joint");
      ImGui::Separator();
      platform::JointPropertiesGui(model_, data_, tmp_.element,
                                   tmp_.element_id);
      break;
    case mjOBJ_SITE:
      ImGui::Text("Site");
      ImGui::Separator();
      platform::SitePropertiesGui(model_, data_, tmp_.element, tmp_.element_id);
      break;
    default:
      // ignore other types
      break;
  }
}

void App::HelpGui() {
  ImGui::Columns(4);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.35f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.15f);
  ImGui::SetColumnWidth(2, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(3, ImGui::GetWindowWidth() * 0.1f);

  ImGui::Text("Help");
  ImGui::Text("Stats");
  ImGui::Text("Cycle Frames");
  ImGui::Text("Cycle Labels");
  ImGui::Text("Free Camera");
  ImGui::Text("Toggle Pause");
  ImGui::Text("Reset Sim");
  ImGui::Text("Show/Hide UI");
  ImGui::Text("Speed Up");
  ImGui::Text("Speed Down");
  ImGui::Text("Prev Camera");
  ImGui::Text("Next Camera");
  ImGui::Text("Step Back");
  ImGui::Text("Step Forward");
  ImGui::Text("Select Parent");
  ImGui::Text("Align Camera");
  ImGui::Text("Copy Keyframe");
  ImGui::Text("Geom Group");
  ImGui::Text("Site Group");

  ImGui::NextColumn();
  ImGui::Text("F1");
  ImGui::Text("F2");
  ImGui::Text("F6");
  ImGui::Text("F7");
  ImGui::Text("Esc");
  ImGui::Text("Spc");
  ImGui::Text("Bksp");
  ImGui::Text("Tab");
  ImGui::Text("=");
  ImGui::Text("-");
  ImGui::Text("[");
  ImGui::Text("]");
  ImGui::Text("Left");
  ImGui::Text("Right");
  ImGui::Text("PgUp");
  ImGui::Text("Ctrl+A");
  ImGui::Text("Ctrl+C");
  ImGui::Text("0-5");
  ImGui::Text("Sh+0-5");

  ImGui::NextColumn();
  ImGui::Text("Activation");
  ImGui::Text("Auto Connect");
  ImGui::Text("Body Tree");
  ImGui::Text("Mesh Tree");
  ImGui::Text("Scale Inertia");
  ImGui::Text("Skin");
  ImGui::Text("Actuator");
  ImGui::Text("Camera");
  ImGui::Text("Center of Mass");
  ImGui::Text("Contact Force");
  ImGui::Text("Contact Point");
  ImGui::Text("Contact Split");
  ImGui::Text("Convex Hull");
  ImGui::Text("Constraint");
  ImGui::Text("Island");
  ImGui::Text("Joint");
  ImGui::Text("Light");
  ImGui::Text("Perturb Force");
  ImGui::Text("Perturb Object");
  ImGui::Text("Range Finder");
  ImGui::Text("Static Body");
  ImGui::Text("Tendon");
  ImGui::Text("Texture");
  ImGui::Text("Transparent");

  ImGui::NextColumn();
  ImGui::Text(",");
  ImGui::Text("K");
  ImGui::Text("`");
  ImGui::Text("\\");
  ImGui::Text("\"");
  ImGui::Text(";");
  ImGui::Text("U");
  ImGui::Text("L");
  ImGui::Text("M");
  ImGui::Text("F");
  ImGui::Text("C");
  ImGui::Text("P");
  ImGui::Text("H");
  ImGui::Text("N");
  ImGui::Text("I");
  ImGui::Text("J");
  ImGui::Text("Z");
  ImGui::Text("B");
  ImGui::Text("O");
  ImGui::Text("Y");
  ImGui::Text("G");
  ImGui::Text("V");
  ImGui::Text("X");
  ImGui::Text("T");

  ImGui::Columns();
}

void App::ToolBarGui() {
  if (ImGui::BeginTable("##ToolBarTable", 2)) {
    platform::ScopedStyle style;
    const ImColor red(220, 40, 40, 255);
    const ImColor green(40, 180, 40, 255);
    const ImColor yellow(250, 230, 10, 255);
    const int combo_flags = ImGuiComboFlags_NoArrowButton;

    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 520);

    ImGui::TableNextColumn();
    ImGui::Text("%s", " ");

    // Unload button.
    ImGui::SameLine();
    style.Color(ImGuiCol_ButtonHovered, red);
    if (ImGui::Button(ICON_UNLOAD_MODEL, ImVec2(48, 32))) {
      RequestModelLoad("");
    }
    ImGui::SetItemTooltip("%s", "Unload");
    style.Reset();

    // Reload button.
    ImGui::SameLine();
    if (ImGui::Button(ICON_RELOAD_MODEL, ImVec2(48, 32))) {
      RequestModelLoad(model_name_);
    }
    ImGui::SetItemTooltip("%s", "Reload");

    // Reset button.
    ImGui::SameLine();
    if (ImGui::Button(ICON_RESET_MODEL, ImVec2(48, 32))) {
      ResetPhysics();
    }
    ImGui::SetItemTooltip("%s", "Reset");

    // Play/pause button.
    ImGui::SameLine();
    const bool paused = step_control_.IsPaused();
    style.Color(ImGuiCol_Button, paused ? yellow : green);
    if (ImGui::Button(paused ? ICON_PLAY : ICON_PAUSE, ImVec2(120, 32))) {
      step_control_.TogglePause();
    }
    ImGui::SetItemTooltip("%s", paused ? "Play" : "Pause");
    style.Reset();

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Speed selection.
    ImGui::SameLine();
    ImGui::Text("%s", ICON_SPEED);
    ImGui::SetItemTooltip("%s", "Playback Speed");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(50);
    if (ImGui::BeginCombo("##Speed", kPercentRealTime[tmp_.speed_index],
                          combo_flags)) {
      for (int n = 0; n < kPercentRealTime.size(); n++) {
        if (ImGui::Selectable(kPercentRealTime[n], (tmp_.speed_index == n))) {
          SetSpeedIndex(n);
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("%s", "Playback Speed");

    // Camera selection.
    std::vector<const char*> cameras = GetCameraNames();
    ImGui::TableNextColumn();
    ImGui::Text("%s", ICON_CAMERA);
    ImGui::SetItemTooltip("%s", "Camera");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(GetExpectedLabelWidth());
    int camera_idx = ui_.camera_idx - platform::kTumbleCameraIdx;
    if (ImGui::BeginCombo("##Camera", cameras[camera_idx], combo_flags)) {
      for (int n = 0; n < cameras.size(); n++) {
        if (ImGui::Selectable(cameras[n], (camera_idx == n))) {
          ui_.camera_idx = ::mujoco::platform::SetCamera(
              model_, &camera_, camera_idx + platform::kTumbleCameraIdx);
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("%s", "Camera");
    ImGui::SameLine();
    if (ImGui::Button(ICON_COPY_CAMERA)) {
      std::string camera_string = platform::CameraToString(data_, &camera_);
      platform::MaybeSaveToClipboard(camera_string);
    }
    ImGui::SetItemTooltip("%s", "Copy Camera");

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Label selection.
    ImGui::SameLine();
    ImGui::Text("%s", ICON_LABEL);
    ImGui::SetItemTooltip("%s", "Label");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(GetExpectedLabelWidth());
    if (ImGui::BeginCombo("##Label", kLabelNames[vis_options_.label],
                          combo_flags)) {
      for (int n = 0; n < IM_ARRAYSIZE(kLabelNames); n++) {
        if (ImGui::Selectable(kLabelNames[n], (vis_options_.label == n))) {
          vis_options_.label = n;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("%s", "Label");

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Frame selection.
    ImGui::SameLine();
    ImGui::Text("%s", ICON_FRAME);
    ImGui::SetItemTooltip("%s", "Frame");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(GetExpectedLabelWidth());
    if (ImGui::BeginCombo("##Frame", kFrameNames[vis_options_.frame],
                          combo_flags)) {
      for (int n = 0; n < IM_ARRAYSIZE(kFrameNames); n++) {
        if (ImGui::Selectable(kFrameNames[n], (vis_options_.frame == n))) {
          vis_options_.frame = n;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("%s", "Frame");

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Style selection.
    ImGui::SameLine();
    switch (ui_.theme) {
      case platform::GuiTheme::kLight:
        if (ImGui::Button(ICON_LIGHTMODE)) {
          SetupTheme(platform::GuiTheme::kDark);
        }
        ImGui::SetItemTooltip("%s", "Switch to Dark Mode");
        break;
      case platform::GuiTheme::kDark:
        if (ImGui::Button(ICON_DARKMODE)) {
          SetupTheme(platform::GuiTheme::kClassic);
        }
        ImGui::SetItemTooltip("%s", "Switch to Classic Mode");
        break;
      case platform::GuiTheme::kClassic:
        if (ImGui::Button(ICON_CLASSICMODE)) {
          SetupTheme(platform::GuiTheme::kLight);
        }
        ImGui::SetItemTooltip("%s", "Switch to Light Mode");
        break;
    }

    ImGui::EndTable();
  }
}

void App::StatusBarGui() {
  if (ImGui::BeginTable("##StatusBarTable", 2)) {
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 520);

    ImGui::TableNextColumn();

    if (!IsModelLoaded()) {
      ImGui::Text("Not loaded");
    } else if (model_ == nullptr) {
      ImGui::Text("Not loaded");
    } else if (step_control_.IsPaused()) {
      ImGui::Text("Paused");
    } else {
      const float desired_realtime = step_control_.GetSpeed();
      const float measured_realtime = step_control_.GetSpeedMeasured();
      const float realtime_offset =
          mju_abs(measured_realtime - desired_realtime);
      const bool misaligned = realtime_offset > 0.1 * desired_realtime;
      if (misaligned) {
        ImGui::Text("Running: %g%% (%-4.1f%%)", desired_realtime,
                    measured_realtime);
      } else {
        ImGui::Text("Running: %g%%", desired_realtime);
      }
    }

    if (!error_.empty()) {
      ImGui::SameLine();
      ImGui::Text(" | Error: %s", error_.c_str());
    }

    ImGui::TableNextColumn();
    ImGui::Text("%s", " |");

    // Frame scrubber.
    platform::ScopedStyle style;

    style.Var(ImGuiStyleVar_FrameBorderSize, 0);
    style.Color(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_WindowBg]);
    ImGui::SameLine();
    if (ImGui::Button(ICON_PREV_FRAME)) {
      LoadHistory(history_.GetIndex() - 1);
    }
    ImGui::SetItemTooltip("%s", "Previous Frame");

    style.Reset();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(450);
    int index = history_.GetIndex();
    if (ImGui::SliderInt("##ScrubIndex", &index, 1 - history_.Size(), 0)) {
      LoadHistory(index);
    }

    style.Var(ImGuiStyleVar_FrameBorderSize, 0);
    style.Color(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_WindowBg]);
    ImGui::SameLine();
    if (ImGui::Button(ICON_NEXT_FRAME)) {
      if (history_.GetIndex() == 0) {
        step_control_.RequestSingleStep();
      } else {
        LoadHistory(history_.GetIndex() + 1);
      }
    }
    ImGui::SetItemTooltip("%s", "Next Frame");

    ImGui::SameLine();
    if (ImGui::Button(ICON_CURR_FRAME)) {
      LoadHistory(0);
    }
    ImGui::SetItemTooltip("%s", "Current Frame");

    ImGui::EndTable();
  }
}

void App::MainMenuGui() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open Model File", "Ctrl+O")) {
        ShowPopup(tmp_.load_popup);
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Save XML", "Ctrl+S")) {
        ShowPopup(tmp_.save_xml_popup);
      }
      if (ImGui::MenuItem("Save MJB", "Ctrl+Shift+S")) {
        ShowPopup(tmp_.save_mjb_popup);
      }
      if (ImGui::MenuItem("Save Screenshot", "Ctrl+P")) {
        ShowPopup(tmp_.save_screenshot_popup);
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Print Model", "Ctrl+M")) {
        ShowPopup(tmp_.print_model_popup);
      }
      if (ImGui::MenuItem("Print Data", "Ctrl+D")) {
        ShowPopup(tmp_.print_data_popup);
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Unload", "Ctrl+U")) {
        RequestModelLoad("");
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Quit", "Ctrl+Q")) {
        tmp_.should_exit = true;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Simulation")) {
      if (ImGui::MenuItem("Pause", "Space", step_control_.IsPaused())) {
        step_control_.TogglePause();
      }
      if (ImGui::MenuItem("Reset", "Backspace")) {
        ResetPhysics();
      }
      if (ImGui::MenuItem("Reload", "Ctrl+L")) {
        RequestModelLoad(model_name_);
      }
      ImGui::Separator();
      if (ImGui::BeginMenu("Keyframes")) {
        ImGui::SetNextItemWidth(200);
        ImGui::SliderInt("##Key", &ui_.key_idx, 0, model_->nkey);
        if (ImGui::MenuItem("Load")) {
          mj_resetDataKeyframe(model_, data_, ui_.key_idx);
          mj_forward(model_, data_);
        }
        if (ImGui::MenuItem("Save")) {
          mj_setKeyframe(model_, data_, ui_.key_idx);
        }
        if (ImGui::MenuItem("Copy")) {
          std::string str = platform::KeyframeToString(model_, data_, false);
          platform::MaybeSaveToClipboard(str);
        }
        ImGui::EndMenu();
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Save Config")) {
        SaveSettings();
      }
      if (ImGui::MenuItem("Reset Config")) {
        platform::SaveText("\n\n", ini_path_);
        LoadSettings();
      }
      ImGui::Separator();

      if (ImGui::MenuItem(tmp_.options_panel ? "Hide Options" : "Show Left UI",
                          "Tab")) {
        tmp_.options_panel = !tmp_.options_panel;
      }
      if (ImGui::MenuItem(
              tmp_.inspector_panel ? "Hide Inspector" : "Show Right UI",
              "Shift+Tab")) {
        tmp_.inspector_panel = !tmp_.inspector_panel;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Charts")) {
      if (ImGui::MenuItem("Solver", "F9")) {
        tmp_.chart_solver = !tmp_.chart_solver;
      }
      if (ImGui::MenuItem("CPU Time", "F10")) {
        tmp_.chart_cpu_time = !tmp_.chart_cpu_time;
      }
      if (ImGui::MenuItem("Dimensions", "F11")) {
        tmp_.chart_dimensions = !tmp_.chart_dimensions;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
      if (ImGui::MenuItem("Help", "F1", tmp_.help)) {
        ToggleWindow(tmp_.help);
      }
      if (ImGui::MenuItem("Stats", "F2", tmp_.stats)) {
        ToggleWindow(tmp_.stats);
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Style Editor", "", tmp_.style_editor)) {
        tmp_.style_editor = !tmp_.style_editor;
      }
      ImGui::Separator();
      if (ImGui::MenuItem("ImGui Demo")) {
        tmp_.imgui_demo = !tmp_.imgui_demo;
      }
      if (ImGui::MenuItem("ImPlot Demo")) {
        tmp_.implot_demo = !tmp_.implot_demo;
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
}

void App::FileDialogGui() {
  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

  if (tmp_.load_popup) {
    ImGui::OpenPopup("LoadModel");
    tmp_.load_popup = false;
    strncpy(tmp_.filename, tmp_.last_load_file.c_str(),
            tmp_.last_load_file.size());
    tmp_.filename[tmp_.last_load_file.size()] = 0;
  }
  if (tmp_.save_xml_popup) {
    ImGui::OpenPopup("SaveXML");
    tmp_.save_xml_popup = false;
    strncpy(tmp_.filename, tmp_.last_save_xml_file.c_str(),
            tmp_.last_save_xml_file.size());
    tmp_.filename[tmp_.last_save_xml_file.size()] = 0;
  }
  if (tmp_.save_mjb_popup) {
    ImGui::OpenPopup("SaveMJB");
    tmp_.save_mjb_popup = false;
    strncpy(tmp_.filename, tmp_.last_save_mjb_file.c_str(),
            tmp_.last_save_mjb_file.size());
    tmp_.filename[tmp_.last_save_mjb_file.size()] = 0;
  }
  if (tmp_.save_screenshot_popup) {
    ImGui::OpenPopup("SaveWebp");
    tmp_.save_screenshot_popup = false;
    strncpy(tmp_.filename, tmp_.last_save_screenshot_file.c_str(),
            tmp_.last_save_screenshot_file.size());
    tmp_.filename[tmp_.last_save_screenshot_file.size()] = 0;
  }
  if (tmp_.print_model_popup) {
    ImGui::OpenPopup("PrintModel");
    tmp_.print_model_popup = false;
    strncpy(tmp_.filename, tmp_.last_print_model_file.c_str(),
            tmp_.last_print_model_file.size());
    tmp_.filename[tmp_.last_print_model_file.size()] = 0;
  }
  if (tmp_.print_data_popup) {
    ImGui::OpenPopup("PrintData");
    tmp_.print_data_popup = false;
    strncpy(tmp_.filename, tmp_.last_print_data_file.c_str(),
            tmp_.last_print_data_file.size());
    tmp_.filename[tmp_.last_print_data_file.size()] = 0;
  }

  tmp_.modal_open =
      ImGui::IsPopupOpen("LoadModel") || ImGui::IsPopupOpen("SaveXML") ||
      ImGui::IsPopupOpen("SaveMJB") || ImGui::IsPopupOpen("SaveWebp") ||
      ImGui::IsPopupOpen("PrintModel") || ImGui::IsPopupOpen("PrintData");

  if (ImGui::BeginPopupModal("LoadModel", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (platform::ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      RequestModelLoad(tmp_.filename);
      tmp_.last_load_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveXML", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (platform::ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      char err[1000] = "";
      mj_saveLastXML(tmp_.filename, model_, err, 1000);
      tmp_.last_save_xml_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveMJB", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (platform::ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_saveModel(model_, tmp_.filename, nullptr, 0);
      tmp_.last_save_mjb_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveWebp", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (platform::ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      const int width = window_->GetWidth();
      const int height = window_->GetHeight();
      std::vector<std::byte> buffer(width * height * 3);
      renderer_->RenderToTexture(model_, data_, &camera_, width, height,
                                 buffer.data());
      platform::SaveToWebp(width, height, buffer.data(), tmp_.filename);
      tmp_.last_save_screenshot_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("PrintModel", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (platform::ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_printModel(model_, tmp_.filename);
      tmp_.last_print_model_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("PrintData", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (platform::ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_printData(model_, data_, tmp_.filename);
      tmp_.last_print_data_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
}

float App::GetExpectedLabelWidth() {
  // Find the longest label which we'll use to set the minimum toggle button
  // width. This isn't perfect because we may have labels that are longer, but
  // it's a good enough approximation.
  if (tmp_.expected_label_width == 0) {
    int longest = 0;
    const char* longest_label = "";
    for (int i = 0; i < mjNVISFLAG; ++i) {
      int length = static_cast<int>(strlen(mjVISSTRING[i][0]));
      if (length > longest) {
        longest_label = mjVISSTRING[i][0];
        longest = length;
      }
    }
    // Pad the width a bit to account for how the labels will be displayed
    // (e.g. as button labels or besides checkboxes).
    tmp_.expected_label_width = ImGui::CalcTextSize(longest_label).x + 16;
  }
  return tmp_.expected_label_width;
}

std::vector<const char*> App::GetCameraNames() {
  if (tmp_.camera_names.empty()) {
    tmp_.camera_names.reserve(model_->ncam + 3);

    tmp_.camera_names.push_back("Free: tumble");
    tmp_.camera_names.push_back("Free: wasd");
    tmp_.camera_names.push_back("Tracking (-1)");
    for (int i = 0; i < model_->ncam; i++) {
      if (model_->names[model_->name_camadr[i]]) {
        tmp_.camera_names.push_back(model_->names + model_->name_camadr[i]);
      } else {
        tmp_.camera_names.push_back("Unnamed");
      }
    }
  }

  // Update tracking camera name as this can change over time.
  tmp_.camera_names[2] =
      "Tracking (" + std::to_string(camera_.trackbodyid) + ")";

  std::vector<const char*> names;
  names.reserve(tmp_.camera_names.size());
  for (const auto& name : tmp_.camera_names) {
    names.push_back(name.c_str());
  }
  return names;
}

App::UiState::Dict App::UiState::ToDict() const {
  return {
    {"theme", std::to_string(static_cast<int>(theme))},
  };
}

void App::UiState::FromDict(const Dict& dict) {
  *this = UiState();
  theme = ReadIniValue(dict, "theme", theme);
}

int App::LoadAssetCallback(const char* path, void* user_data,
                           unsigned char** out, std::uint64_t* out_size) {
  App* app = static_cast<App*>(user_data);
  std::vector<std::byte> bytes = (app->load_asset_fn_)(path);
  if (bytes.empty()) {
    *out_size = 0;
    return 0;  // Empty file
  }

  *out_size = bytes.size();
  *out = reinterpret_cast<unsigned char*>(malloc(*out_size));
  if (*out == nullptr) {
    mju_error("Failed to allocate memory for file %s", path);
    return -1;
  }

  std::memcpy(*out, bytes.data(), *out_size);
  return 0;
}

}  // namespace mujoco::studio
