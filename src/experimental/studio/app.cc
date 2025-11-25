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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/toolbox/helpers.h"
#include "experimental/toolbox/imgui_widgets.h"
#include "experimental/toolbox/interaction.h"
#include "experimental/toolbox/renderer.h"
#include "experimental/toolbox/step_control.h"
#include "experimental/toolbox/window.h"

#if defined(USE_FILAMENT_OPENGL) || defined(USE_FILAMENT_VULKAN)
#include "experimental/filament/render_context_filament.h"
#elif defined(USE_CLASSIC_OPENGL)
#include <backends/imgui_impl_opengl3.h>
#else
#error No rendering mode defined.
#endif

namespace mujoco::studio {

static constexpr toolbox::Window::Config kWindowConfig = {
#ifdef EMSCRIPTEN
    .render_config = toolbox::Window::RenderConfig::kFilamentWebGL,
#elif defined(USE_FILAMENT_VULKAN)
    .render_config = toolbox::Window::RenderConfig::kFilamentVulkan,
#elif defined(USE_FILAMENT_OPENGL)
    .render_config = toolbox::Window::RenderConfig::kFilamentOpenGL,
#elif defined(USE_CLASSIC_OPENGL)
    .render_config = toolbox::Window::RenderConfig::kClassicOpenGL,
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

static ImVec2 GetFlexElementSize(int num_cols) {
  const float width = (ImGui::GetContentRegionAvail().x / num_cols) -
                      ImGui::GetStyle().FramePadding.x * 2;
  return ImVec2(width, 0);
}

// FontAwesome icon codes.
static constexpr const char* ICON_FA_PLAY = "\xef\x81\x8b";
static constexpr const char* ICON_FA_PAUSE = "\xef\x81\x8c";
static constexpr const char* ICON_FA_COPY = "\xef\x83\x85";
static constexpr const char* ICON_FA_EJECT = "\xef\x81\x92";
static constexpr const char* ICON_FA_REFRESH = "\xef\x80\xa1";
static constexpr const char* ICON_FA_COMMENT = "\xef\x83\xa5";
static constexpr const char* ICON_FA_UNDO = "\xef\x83\xa2";
static constexpr const char* ICON_FA_ARROWS = "\xef\x81\x87";
static constexpr const char* ICON_FA_CAMERA = "\xef\x80\xbd";
static constexpr const char* ICON_FA_MOON = "\xef\x86\x86";
static constexpr const char* ICON_FA_SUN = "\xef\x86\x85";
static constexpr const char* ICON_FA_CARET_LEFT = "\xef\x83\x99";
static constexpr const char* ICON_FA_CARET_RIGHT = "\xef\x83\x9a";
static constexpr const char* ICON_FA_FAST_FORWARD = "\xef\x81\x90";
static constexpr const char* ICON_FA_TACHOMETER = "\xef\x83\xa4";

static constexpr const char* ICON_PLAY = ICON_FA_PLAY;
static constexpr const char* ICON_PAUSE = ICON_FA_PAUSE;
static constexpr const char* ICON_COPY_CAMERA = ICON_FA_COPY;
static constexpr const char* ICON_UNLOAD_MODEL = ICON_FA_EJECT;
static constexpr const char* ICON_RELOAD_MODEL = ICON_FA_REFRESH;
static constexpr const char* ICON_LABEL = ICON_FA_COMMENT;
static constexpr const char* ICON_RESET_MODEL = ICON_FA_UNDO;
static constexpr const char* ICON_FRAME = ICON_FA_ARROWS;
static constexpr const char* ICON_CAMERA = ICON_FA_CAMERA;
static constexpr const char* ICON_DARKMODE = ICON_FA_MOON;
static constexpr const char* ICON_LIGHTMODE = ICON_FA_SUN;
static constexpr const char* ICON_PREV_FRAME = ICON_FA_CARET_LEFT;
static constexpr const char* ICON_NEXT_FRAME = ICON_FA_CARET_RIGHT;
static constexpr const char* ICON_CURR_FRAME = ICON_FA_FAST_FORWARD;
static constexpr const char* ICON_SPEED = ICON_FA_TACHOMETER;

static constexpr int kToolsBarHeight = 48;
static constexpr int kStatusBarHeight = 32;
static constexpr float kSettingsRelWidth = 0.22f;
static constexpr float kInspectorRelWidth = 0.18f;
static constexpr float kInfoRelHeight = 0.3f;

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

using toolbox::ImGui_FileDialog;
using toolbox::ImGui_Input;
using toolbox::ImGui_InputN;
using toolbox::ImGui_IsChordJustPressed;
using toolbox::ImGui_Slider;
using toolbox::ImGui_ButtonToggle;
using toolbox::ImGui_SwitchToggle;
using toolbox::ImGui_BitToggle;

App::App(int width, int height, std::string ini_path,
         const toolbox::LoadAssetFn& load_asset_fn)
    : ini_path_(std::move(ini_path)), load_asset_fn_(load_asset_fn) {
  window_ = std::make_unique<toolbox::Window>("MuJoCo Studio", width, height,
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
  renderer_ = std::make_unique<toolbox::Renderer>(make_context_fn);

  mjv_defaultPerturb(&perturb_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&vis_options_);

  profiler_.Clear();

#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_Init();
#endif
}

void App::LoadModel(std::string model_file) {
  pending_load_ = std::move(model_file);
}

void App::ProcessPendingLoad() {
  if (!pending_load_.has_value()) {
    return;
  }

  if (model_) {
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
    error_ = "";

    step_control_.SetSpeed(100.f);
  }

  std::string model_file = std::move(pending_load_.value());
  pending_load_.reset();

  model_ = toolbox::LoadMujocoModel(model_file, nullptr);
  if (!model_) {
    error_ = "Error loading model!";
    step_control_.Pause();
    model_ = toolbox::LoadMujocoModel("", nullptr);
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    error_ = "Error making data!";
    step_control_.Pause();
  }

  OnModelLoaded(model_file);
}

void App::OnModelLoaded(std::string_view model_file) {
  model_file_ = std::move(model_file);

  renderer_->Init(model_);
  tmp_ = UiTempState();
  mjv_defaultOption(&vis_options_);

  const int state_size = mj_stateSize(model_, mjSTATE_INTEGRATION);
  history_.Init(state_size);
  profiler_.Clear();

  std::string base_path = "/";
  std::string model_name = "model";

  if (!model_file.empty() &&
      (model_file.ends_with(".xml") || model_file.ends_with(".mjb"))) {
    window_->SetTitle("MuJoCo Studio : " + std::string(model_file));
    tmp_.last_load_file = std::string(model_file_);
    std::filesystem::path path(model_file_);
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

void App::ResetPhysics() {
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  error_ = "";
}

void App::UpdatePhysics() {
  ProcessPendingLoad();
  if (!model_ || !data_) {
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

  toolbox::StepControl::Status status = step_control_.Advance(model_, data_);
  if (status == toolbox::StepControl::Status::kPaused) {
    // do nothing
  } else if (status == toolbox::StepControl::Status::kOk) {
    std::span<mjtNum> state = history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model_, data_, state.data(), mjSTATE_INTEGRATION);
    }
    // If we are adding to the history we didn't have a divergence error
    error_ = "";
  } else if (status == toolbox::StepControl::Status::kAutoReset) {
    ResetPhysics();
  } else if (status == toolbox::StepControl::Status::kDiverged) {
    for (mjtWarning w : toolbox::StepControl::kDivergedWarnings) {
      if (data_->warning[w].number > 0) {
        error_ = mju_warningText(w, data_->warning[w].lastinfo);
      }
    }
  }

  profiler_.Update(model_, data_);
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
  const toolbox::Window::Status status = window_->NewFrame();

#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_NewFrame();
#endif

  HandleMouseEvents();
  HandleKeyboardEvents();

  // Check to see if a model was dropped on the window.
  const std::string drop_file = window_->GetDropFile();
  if (!drop_file.empty()) {
    LoadModel(drop_file);
  }

  // Only update the simulation if a popup window is not open. Note that the
  // simulation itself will only update if it is not paused.
  if (!tmp_.modal_open) {
    UpdatePhysics();
  }

  return status == toolbox::Window::Status::kRunning;
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
        toolbox::InitPerturb(model_, data_, &camera_, &perturb_, active);
      }
      toolbox::MovePerturb(model_, data_, &camera_, &perturb_, action,
                           mouse_dx, mouse_dy);
    }
  }
  // Handle camera movement actions.
  else if (is_mouse_dragging) {
    if (ui_.camera_idx == toolbox::kFreeCameraIdx) {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        MoveCamera(toolbox::CameraMotion::PAN_TILT, mouse_dx, mouse_dy);
      }
    } else {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        MoveCamera(toolbox::CameraMotion::ORBIT, mouse_dx, mouse_dy);
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        MoveCamera(toolbox::CameraMotion::ZOOM, mouse_dx, mouse_dy);
      }
    }

    // Right mouse movement is relative to the horizontal and vertical planes.
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && io.KeyShift) {
      MoveCamera(toolbox::CameraMotion::PLANAR_MOVE_H, mouse_dx, mouse_dy);
    } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
      MoveCamera(toolbox::CameraMotion::PLANAR_MOVE_V, mouse_dx, mouse_dy);
    }
  }

  // Mouse scroll zooms the mouse. Free cameras don't have the concept of
  // zooming, so we ignore the scroll in those cases.
  if (mouse_scroll != 0.0f && ui_.camera_idx != toolbox::kFreeCameraIdx) {
    MoveCamera(toolbox::CameraMotion::ZOOM, 0, mouse_scroll);
  }

  // Left double click.
  if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
    toolbox::PickResult picked =
        toolbox::Pick(model_, data_, &camera_, mouse_x, mouse_y,
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
    toolbox::PickResult picked =
        toolbox::Pick(model_, data_, &camera_, mouse_x, mouse_y,
                      window_->GetAspectRatio(), &vis_options_);
    mju_copy3(camera_.lookat, picked.point);
    if (picked.body > 0 && io.KeyCtrl) {
      // Switch camera to tracking mode and track the selected body.
      camera_.type = mjCAMERA_TRACKING;
      camera_.trackbodyid = picked.body;
      camera_.fixedcamid = -1;
      ui_.camera_idx = toolbox::kTrackingCameraIdx;
    }
  }
}

void App::HandleKeyboardEvents() {
  if (ImGui::GetIO().WantCaptureKeyboard) {
    return;
  }

  constexpr auto ImGuiMode_CtrlShift = ImGuiMod_Ctrl | ImGuiMod_Shift;

  // Menu shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_L | ImGuiMod_Ctrl)) {
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
    std::string keyframe = toolbox::KeyframeToString(model_, data_, false);
    toolbox::MaybeSaveToClipboard(keyframe);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_R | ImGuiMod_Ctrl)) {
    LoadModel(model_file_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q | ImGuiMod_Ctrl)) {
    tmp_.should_exit = true;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_A | ImGuiMod_Ctrl)) {
    mjv_defaultFreeCamera(model_, &camera_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab | ImGuiMod_Shift)) {
    tmp_.inspector_panel = !tmp_.inspector_panel;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab)) {
    tmp_.settings_panel = !tmp_.settings_panel;
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
    ToggleWindow(tmp_.info);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F6)) {
    vis_options_.frame = (vis_options_.frame + 1) % mjNFRAME;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F7)) {
    vis_options_.label = (vis_options_.label + 1) % mjNLABEL;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F9)) {
    tmp_.chart_counts = !tmp_.chart_counts;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F10)) {
    tmp_.chart_convergence = !tmp_.chart_convergence;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F11)) {
    tmp_.chart_dimensions = !tmp_.chart_dimensions;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F12)) {
    tmp_.chart_cpu_time = !tmp_.chart_cpu_time;
  // } else if (ImGui_IsChordJustPressed(ImGuiKey_Backquote)) {
  //   ToggleFlag(vis_options_.flags[mjVIS_BODYBVH]);
  // } else if (ImGui_IsChordJustPressed(ImGuiKey_Quote)) {
  //   ToggleFlag(vis_options_.flags[mjVIS_SCLINERTIA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Comma)) {
    ToggleFlag(vis_options_.flags[mjVIS_ACTIVATION]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backslash)) {
    ToggleFlag(vis_options_.flags[mjVIS_MESHBVH]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Semicolon)) {
    ToggleFlag(vis_options_.flags[mjVIS_SKIN]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_U)) {
    ToggleFlag(vis_options_.flags[mjVIS_ACTUATOR]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_L)) {
    ToggleFlag(vis_options_.flags[mjVIS_CAMERA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_M)) {
    ToggleFlag(vis_options_.flags[mjVIS_COM]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONTACTFORCE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONTACTPOINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_P)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONTACTSPLIT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_H)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONVEXHULL]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_N)) {
    ToggleFlag(vis_options_.flags[mjVIS_CONSTRAINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_I)) {
    ToggleFlag(vis_options_.flags[mjVIS_ISLAND]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_J)) {
    ToggleFlag(vis_options_.flags[mjVIS_JOINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Z)) {
    ToggleFlag(vis_options_.flags[mjVIS_LIGHT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_B)) {
    ToggleFlag(vis_options_.flags[mjVIS_PERTFORCE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_O)) {
    ToggleFlag(vis_options_.flags[mjVIS_PERTOBJ]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Y)) {
    ToggleFlag(vis_options_.flags[mjVIS_RANGEFINDER]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_V)) {
    ToggleFlag(vis_options_.flags[mjVIS_TENDON]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_X)) {
    ToggleFlag(vis_options_.flags[mjVIS_TEXTURE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_T)) {
    ToggleFlag(vis_options_.flags[mjVIS_TRANSPARENT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_K)) {
    ToggleFlag(vis_options_.flags[mjVIS_AUTOCONNECT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_G)) {
    ToggleFlag(vis_options_.flags[mjVIS_STATIC]);
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
      ui_.camera_idx = toolbox::SetCamera(model_, &camera_, toolbox::kTumbleCameraIdx);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftBracket)) {
      ui_.camera_idx = toolbox::SetCamera(model_, &camera_, ui_.camera_idx - 1);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_RightBracket)) {
      ui_.camera_idx = toolbox::SetCamera(model_, &camera_, ui_.camera_idx + 1);
    }

    // WASD camera controls for free camera.
    if (ui_.camera_idx == toolbox::kFreeCameraIdx) {
      bool moved = false;

      // Move (dolly) forward/backward using W and S keys.
      if (ImGui::IsKeyDown(ImGuiKey_W)) {
        MoveCamera(toolbox::CameraMotion::TRUCK_DOLLY, 0, tmp_.cam_speed);
        moved = true;
      } else if (ImGui::IsKeyDown(ImGuiKey_S)) {
        MoveCamera(toolbox::CameraMotion::TRUCK_DOLLY, 0, -tmp_.cam_speed);
        moved = true;
      }

      // Strafe (truck) left/right using A and D keys.
      if (ImGui::IsKeyDown(ImGuiKey_A)) {
        MoveCamera(toolbox::CameraMotion::TRUCK_DOLLY, -tmp_.cam_speed, 0);
        moved = true;
      } else if (ImGui::IsKeyDown(ImGuiKey_D)) {
        MoveCamera(toolbox::CameraMotion::TRUCK_DOLLY, tmp_.cam_speed, 0);
        moved = true;
      }

      // Move (pedestal) up/down using Q and E keys.
      if (ImGui::IsKeyDown(ImGuiKey_Q)) {
        MoveCamera(toolbox::CameraMotion::TRUCK_PEDESTAL, 0, tmp_.cam_speed);
        moved = true;
      } else if (ImGui::IsKeyDown(ImGuiKey_E)) {
        MoveCamera(toolbox::CameraMotion::TRUCK_PEDESTAL, 0, -tmp_.cam_speed);
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
    std::string settings = toolbox::LoadText(ini_path_);
    if (!settings.empty()) {
      ui_.FromDict(toolbox::ReadIniSection(settings, "[Simulate][Data]"));
      ImGui::LoadIniSettingsFromMemory(settings.data(), settings.size());
    }
  }
}

void App::SaveSettings() {
  if (!ini_path_.empty()) {
    std::string settings = ImGui::SaveIniSettingsToMemory();
    toolbox::AppendIniSection(settings, "[Simulate][Data]", ui_.ToDict());
    toolbox::SaveText(settings, ini_path_);
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

void App::MoveCamera(toolbox::CameraMotion motion, mjtNum reldx, mjtNum reldy) {
  toolbox::MoveCamera(model_, data_, &camera_, motion, reldx, reldy);
}

void App::BuildGui() {
  SetupStyle(ui_.style);
  const ImVec4 workspace_rect = ConfigureDockingLayout();

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
    toolbox::ScopedStyle style;
    style.Var(ImGuiStyleVar_CellPadding, ImVec2(0, 0));
    style.Var(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
    style.Var(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    if (ImGui::Begin("StatusBar")) {
      StatusBarGui();
    }
    ImGui::End();
  }

  if (tmp_.settings_panel) {
    if (ImGui::Begin("Settings", &tmp_.settings_panel)) {
      SettingsGui();
    }
    ImGui::End();
  }

  if (tmp_.inspector_panel) {
    if (ImGui::Begin("Inspector", &tmp_.inspector_panel)) {
      InspectorGui();
    }
    ImGui::End();
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

  if (tmp_.chart_convergence) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Convergence", &tmp_.chart_convergence)) {
      ImGui::Text("Coming soon!");
    }
    ImGui::End();
  }

  if (tmp_.chart_counts) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Counts", &tmp_.chart_counts)) {
      ImGui::Text("Coming soon!");
    }
    ImGui::End();
  }

  if (tmp_.help) {
    toolbox::ScopedStyle style;
    style.Var(ImGuiStyleVar_Alpha, 0.6f);
    ImGui::SetNextWindowPos(ImVec2(workspace_rect.x, workspace_rect.y),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Help", &tmp_.help)) {
      HelpGui();
    }
    ImGui::End();
  }

  if (tmp_.info) {
    toolbox::ScopedStyle style;
    style.Var(ImGuiStyleVar_Alpha, 0.6f);
    if (ImGui::Begin("Info", &tmp_.info)) {
      InfoGui();
    }
    ImGui::End();
  }

  // Display a drag-and-drop message if no model is loaded.
  if (model_file_.empty()) {
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
  if (io.WantSaveIniSettings) {
    SaveSettings();
    io.WantSaveIniSettings = false;
  }
}

void App::SetupStyle(Style style) {
  if (tmp_.style_editor) {
    return;
  }

  ImGuiStyle& s = ImGui::GetStyle();
  if (style == kDark) {
    ImGui::StyleColorsDark(&s);
  } else {
    ImGui::StyleColorsLight(&s);
  }
  s.FrameBorderSize = 1;
  ui_.style = style;
}

ImVec4 App::ConfigureDockingLayout() {
  ImGuiViewport* viewport = ImGui::GetMainViewport();

  const ImVec2 dockspace_pos{
      viewport->WorkPos.x,
      viewport->WorkPos.y + kToolsBarHeight
  };
  const ImVec2 dockspace_size{
      viewport->WorkSize.x,
      viewport->WorkSize.y - kToolsBarHeight - kStatusBarHeight
  };

  ImGuiID root = ImGui::GetID("Root");
  const bool first_time = (ImGui::DockBuilderGetNode(root) == nullptr);

  if (first_time) {
    ImGui::DockBuilderRemoveNode(root);
    ImGui::DockBuilderAddNode(root, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(root, dockspace_size);

    // Slice up the main dock space.
    ImGuiID main = root;

    ImGuiID settings = 0;
    ImGui::DockBuilderSplitNode(main, ImGuiDir_Left, kSettingsRelWidth,
                                &settings, &main);

    ImGuiID inspector = 0;
    ImGui::DockBuilderSplitNode(main, ImGuiDir_Right, kInspectorRelWidth,
                                &inspector, &main);

    ImGuiID info = 0;
    ImGui::DockBuilderSplitNode(inspector, ImGuiDir_Down, kInfoRelHeight,
                                &info, &inspector);

    ImGui::DockBuilderDockWindow("Dockspace", main);
    ImGui::DockBuilderDockWindow("Settings", settings);
    ImGui::DockBuilderDockWindow("Inspector", inspector);
    ImGui::DockBuilderDockWindow("Info", info);
    ImGui::DockBuilderFinish(root);
  }

  // Create a dummy window filling the entire workspace in which we can perform
  // docking.
  ImGui::SetNextWindowPos(dockspace_pos);
  ImGui::SetNextWindowSize(dockspace_size);
  ImGui::SetNextWindowViewport(viewport->ID);

  const ImGuiWindowFlags kWorkspaceFlags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground;

  toolbox::ScopedStyle style;
  style.Var(ImGuiStyleVar_WindowRounding, 0.0f);
  style.Var(ImGuiStyleVar_WindowBorderSize, 0.0f);
  style.Var(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::Begin("Dockspace", nullptr, kWorkspaceFlags);
  style.Reset();

  const ImGuiDockNodeFlags kDockSpaceFlags =
      ImGuiDockNodeFlags_PassthruCentralNode |
      ImGuiDockNodeFlags_NoDockingOverCentralNode |
      ImGuiDockNodeFlags_AutoHideTabBar;
  ImGui::DockSpace(root, ImVec2(0.0f, 0.0f), kDockSpaceFlags);
  ImGui::End();

  const ImGuiWindowFlags kFixedFlags = ImGuiWindowFlags_NoTitleBar |
                                       ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoResize |
                                       ImGuiWindowFlags_NoScrollbar |
                                       ImGuiWindowFlags_NoDocking;

  // Toolbar is fixed at the top.
  ImGui::SetNextWindowPos(viewport->WorkPos, ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kToolsBarHeight), ImGuiCond_Always);
  ImGui::Begin("ToolBar", nullptr, kFixedFlags);
  ImGui::End();

  // StatusBar is fixed at the bottom.
  ImGui::SetNextWindowPos(ImVec2(0, viewport->Size.y - kStatusBarHeight), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, kStatusBarHeight), ImGuiCond_Always);
  ImGui::Begin("StatusBar", nullptr, kFixedFlags);
  ImGui::End();

  const int settings_width = dockspace_size.x * kSettingsRelWidth;
  const int inspector_width = dockspace_size.x * kInspectorRelWidth;
  const float workspace_x = dockspace_pos.x + settings_width;
  const float workspace_y = dockspace_pos.y;
  const float workspace_w = dockspace_size.x - settings_width - inspector_width;
  const float workspace_h = dockspace_size.y;
  return ImVec4(workspace_x, workspace_y, workspace_w, workspace_h);
}

void App::SettingsGui() {
  ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  if (ImGui::TreeNodeEx("Physics Settings", flags)) {
    PhysicsGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Rendering Settings", flags)) {
    RenderingGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Visibility Groups", flags)) {
    GroupsGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Visualization", flags)) {
    VisualizationGui();
    ImGui::TreePop();
  }
}

void App::InspectorGui() {
  ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;
  if (ImGui::TreeNodeEx("Noise", flags)) {
    NoiseGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Joints", flags)) {
    JointsGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Controls", flags)) {
    ControlsGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Sensor", flags)) {
    SensorGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Watch", flags)) {
    WatchGui();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("State", flags)) {
    StateGui();
    ImGui::TreePop();
  }
}

void App::HelpGui() {
  ImGui::Columns(4);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.35f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.15f);
  ImGui::SetColumnWidth(2, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(3, ImGui::GetWindowWidth() * 0.1f);

  ImGui::Text("Help");
  ImGui::Text("Info");
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

void App::InfoGui() {
  const int num_islands = std::clamp(data_->nisland, 1, mjNISLAND);

  // compute solver error (maximum over islands)
  mjtNum solver_err = 0;
  int solver_iter = 0;
  for (int i = 0; i < num_islands; i++) {
    solver_iter += data_->solver_niter[i];

    mjtNum solerr_i = 0;
    if (data_->solver_niter[i]) {
      const int ind = mjMIN(data_->solver_niter[i], mjNSOLVER) - 1;
      const mjSolverStat* stat = data_->solver + i * mjNSOLVER + ind;
      solerr_i = mju_min(stat->improvement, stat->gradient);
      if (solerr_i == 0) {
        solerr_i = mju_max(stat->improvement, stat->gradient);
      }
    }
    solver_err = mju_max(solver_err, solerr_i);
  }
  solver_err = mju_log10(mju_max(mjMINVAL, solver_err));

  auto type = step_control_.IsPaused() ? mjTIMER_FORWARD : mjTIMER_STEP;
  auto cpu = data_->timer[type].duration / mjMAX(1, data_->timer[type].number);
  auto mempct = 100 * data_->maxuse_arena / (double)(data_->narena);
  auto memlimit = mju_writeNumBytes(data_->narena);

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.6f);

  ImGui::Text("Time");
  ImGui::Text("Size");
  ImGui::Text("CPU");
  ImGui::Text("Solver");
  ImGui::Text("FPS");
  ImGui::Text("Memory");
  if (model_->opt.enableflags & mjENBL_ENERGY) {
    ImGui::Text("Energy");
  }
  if (model_->opt.enableflags & mjENBL_FWDINV) {
    ImGui::Text("FwdInv");
  }
  if (!(model_->opt.disableflags & mjDSBL_ISLAND)) {
    ImGui::Text("Islands");
  }

  ImGui::NextColumn();
  ImGui::Text("%-9.3f", data_->time);
  ImGui::Text("%d (%d con)", data_->nefc, data_->ncon);
  ImGui::Text("%.3f", cpu);
  ImGui::Text("%.1f (%d it)", solver_err, solver_iter);
  ImGui::Text("%0.1f", renderer_->GetFrameRate());
  ImGui::Text("%.1f%% of %s", mempct, memlimit);
  if (model_->opt.enableflags & mjENBL_ENERGY) {
    ImGui::Text("%.3f", data_->energy[0] + data_->energy[1]);
  }
  if (model_->opt.enableflags & mjENBL_FWDINV) {
    ImGui::Text("%.1f %.1f",
                mju_log10(mju_max(mjMINVAL, data_->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL, data_->solver_fwdinv[1])));
  }
  if (!(model_->opt.disableflags & mjDSBL_ISLAND)) {
    ImGui::Text("%d", data_->nisland);
  }
  ImGui::Columns();
}


void App::ToolBarGui() {
  if (ImGui::BeginTable("##ToolBarTable", 2)) {
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 570);

    ImGui::TableNextColumn();

    // Play/pause button.
    const bool paused = step_control_.IsPaused();
    if (ImGui::Button(paused ? ICON_PLAY : ICON_PAUSE, ImVec2(144, 32))) {
      step_control_.TogglePause();
    }
    ImGui::SetItemTooltip("%s", paused ? "Play" : "Pause");

    // Reset/Reload/Unload.
    ImGui::SameLine();
    if (ImGui::Button(ICON_RESET_MODEL, ImVec2(48, 32))) {
      ResetPhysics();
    }
    ImGui::SetItemTooltip("%s", "Reset");

    ImGui::SameLine();
    if (ImGui::Button(ICON_RELOAD_MODEL, ImVec2(48, 32))) {
      LoadModel(model_file_);
    }
    ImGui::SetItemTooltip("%s", "Reload");

    ImGui::SameLine();
    if (ImGui::Button(ICON_UNLOAD_MODEL, ImVec2(48, 32))) {
      LoadModel("");
    }
    ImGui::SetItemTooltip("%s", "Unload");

    // Camera selection.
    ImGui::TableNextColumn();
    ImGui::Text("%s", ICON_CAMERA);
    ImGui::SetItemTooltip("%s", "Camera");
    ImGui::SameLine();
    std::vector<const char*> cameras = GetCameraNames();
    ImGui::SetNextItemWidth(GetExpectedLabelWidth());
    int camera_idx = ui_.camera_idx - toolbox::kTumbleCameraIdx;
    if (ImGui::Combo("##Camera", &camera_idx, cameras.data(), cameras.size())) {
      ui_.camera_idx = ::mujoco::toolbox::SetCamera(
          model_, &camera_, camera_idx + toolbox::kTumbleCameraIdx);
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_COPY_CAMERA)) {
      std::string camera_string = toolbox::CameraToString(data_, &camera_);
      toolbox::MaybeSaveToClipboard(camera_string);
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
    ImGui::Combo("##Label", &vis_options_.label, kLabelNames,
                IM_ARRAYSIZE(kLabelNames));

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Frame selection.
    ImGui::SameLine();
    ImGui::Text("%s", ICON_FRAME);
    ImGui::SetItemTooltip("%s", "Frame");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(GetExpectedLabelWidth());
    ImGui::Combo("##Frame", &vis_options_.frame, kFrameNames,
                IM_ARRAYSIZE(kFrameNames));

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Style selection.
    ImGui::SameLine();
    if (ImGui::Button(ui_.style == kDark ? ICON_DARKMODE : ICON_LIGHTMODE)) {
      if (ui_.style == kDark) {
        SetupStyle(kLight);
      } else {
        SetupStyle(kDark);
      }
    }
    ImGui::SetItemTooltip("%s", "Switch Style");

    ImGui::EndTable();
  }
}

void App::StatusBarGui() {
  if (ImGui::BeginTable("##StatusBarTable", 2)) {
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 670);

    ImGui::TableNextColumn();

    if (model_file_.empty()) {
      ImGui::Text("Not loaded");
    } else if (model_ == nullptr) {
      ImGui::Text("Not loaded");
    } else if (step_control_.IsPaused()) {
      ImGui::Text("Paused");
    } else {
      const float desired_realtime = step_control_.GetSpeed();
      const float measured_realtime = step_control_.GetSpeedMeasured();
      const float realtime_offset = mju_abs(measured_realtime - desired_realtime);
      const bool misaligned = realtime_offset > 0.1 * desired_realtime;
      if (misaligned) {
        ImGui::Text("Running: %g%% (%-4.1f%%)", desired_realtime, measured_realtime);
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

    // Speed selection.
    ImGui::SameLine();
    ImGui::Text("%s", ICON_SPEED);
    ImGui::SetItemTooltip("%s", "Playback Speed");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    int speed_index = tmp_.speed_index;
    if (ImGui::Combo("##Speed", &speed_index, kPercentRealTime.data(),
                     kPercentRealTime.size())) {
      SetSpeedIndex(speed_index);
    }

    ImGui::SameLine();
    ImGui::Text("%s", " |");

    // Frame scrubber.
    toolbox::ScopedStyle style;

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
      if (ImGui::MenuItem("Load Model", "Ctrl+L")) {
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
        LoadModel("");
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
      if (ImGui::MenuItem("Reload", "Ctrl+R")) {
        LoadModel(model_file_);
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
          std::string str = toolbox::KeyframeToString(model_, data_, false);
          toolbox::MaybeSaveToClipboard(str);
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
        toolbox::SaveText("\n\n", ini_path_);
        LoadSettings();
      }
      ImGui::Separator();

      if (ImGui::MenuItem(
              tmp_.settings_panel ? "Hide Settings" : "Show Left UI", "Tab")) {
        tmp_.settings_panel = !tmp_.settings_panel;
      }
      if (ImGui::MenuItem(
              tmp_.inspector_panel ? "Hide Inspector" : "Show Right UI",
              "Shift+Tab")) {
        tmp_.inspector_panel = !tmp_.inspector_panel;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Charts")) {
      if (ImGui::MenuItem("Counts", "F9")) {
        tmp_.chart_counts = !tmp_.chart_counts;
      }
      if (ImGui::MenuItem("Convergence", "F10")) {
        tmp_.chart_convergence = !tmp_.chart_convergence;
      }
      if (ImGui::MenuItem("Dimensions", "F11")) {
        tmp_.chart_dimensions = !tmp_.chart_dimensions;
      }
      if (ImGui::MenuItem("CPU Time", "F12")) {
        tmp_.chart_cpu_time = !tmp_.chart_cpu_time;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
      if (ImGui::MenuItem("Help", "F1", tmp_.help)) {
        ToggleWindow(tmp_.help);
      }
      if (ImGui::MenuItem("Info", "F2", tmp_.info)) {
        ToggleWindow(tmp_.info);
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
    strncpy(tmp_.filename, tmp_.last_load_file.c_str(), tmp_.last_load_file.size());
    tmp_.filename[tmp_.last_load_file.size()] = 0;
  }
  if (tmp_.save_xml_popup) {
    ImGui::OpenPopup("SaveXML");
    tmp_.save_xml_popup = false;
    strncpy(tmp_.filename, tmp_.last_save_xml_file.c_str(), tmp_.last_save_xml_file.size());
    tmp_.filename[tmp_.last_save_xml_file.size()] = 0;
  }
  if (tmp_.save_mjb_popup) {
    ImGui::OpenPopup("SaveMJB");
    tmp_.save_mjb_popup = false;
    strncpy(tmp_.filename, tmp_.last_save_mjb_file.c_str(), tmp_.last_save_mjb_file.size());
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
    strncpy(tmp_.filename, tmp_.last_print_model_file.c_str(), tmp_.last_print_model_file.size());
    tmp_.filename[tmp_.last_print_model_file.size()] = 0;
  }
  if (tmp_.print_data_popup) {
    ImGui::OpenPopup("PrintData");
    tmp_.print_data_popup = false;
    strncpy(tmp_.filename, tmp_.last_print_data_file.c_str(), tmp_.last_print_data_file.size());
    tmp_.filename[tmp_.last_print_data_file.size()] = 0;
  }

  tmp_.modal_open =
      ImGui::IsPopupOpen("LoadModel") || ImGui::IsPopupOpen("SaveXML") ||
      ImGui::IsPopupOpen("SaveMJB") || ImGui::IsPopupOpen("SaveWebp") ||
      ImGui::IsPopupOpen("PrintModel") || ImGui::IsPopupOpen("PrintData");

  if (ImGui::BeginPopupModal("LoadModel", NULL,
                            ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      LoadModel(tmp_.filename);
      tmp_.last_load_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveXML", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      char err[1000] = "";
      mj_saveLastXML(tmp_.filename, model_, err, 1000);
      tmp_.last_save_xml_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveMJB", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_saveModel(model_, tmp_.filename, nullptr, 0);
      tmp_.last_save_mjb_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveWebp", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      renderer_->SaveScreenshot(tmp_.filename, window_->GetWidth(),
                                window_->GetHeight());
      tmp_.last_save_screenshot_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("PrintModel", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_printModel(model_, tmp_.filename);
      tmp_.last_print_model_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("PrintData", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_printData(model_, data_, tmp_.filename);
      tmp_.last_print_data_file = tmp_.filename;
    }
    ImGui::EndPopup();
  }
}

void App::SensorGui() {
}

void App::StateGui() {
  const float min_width = GetExpectedLabelWidth();
  const float available_width =
      ImGui::GetContentRegionAvail().x - ImGui::GetTreeNodeToLabelSpacing();
  const int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 4);
  const ImVec2 size = GetFlexElementSize(num_cols);

  ImGui::Unindent(0.5f * ImGui::GetTreeNodeToLabelSpacing());
  // State component names and tooltips.
  static constexpr const char* name_and_tooltip[][2] = {
      {"TIME", "Time"},
      {"QPOS", "Position"},
      {"QVEL", "Velocity"},
      {"ACT", "Actuator activation"},
      {"WARMSTART", "Acceleration used for warmstart"},
      {"CTRL", "Control"},
      {"QFRC_APPLIED", "Applied generalized force"},
      {"XFRC_APPLIED", "Applied Cartesian force/torque"},
      {"EQ_ACTIVE", "Enable/disable constraints"},
      {"MOCAP_POS", "Positions of mocap bodies"},
      {"MOCAP_QUAT", "Orientations of mocap bodies"},
      {"USERDATA", "User data"},
      {"PLUGIN", "Plugin state"},
  };

  int prev_state_sig = tmp_.state_sig;

  // State component checkboxes.
  if (ImGui::BeginTable("##StateSignature", num_cols)) {
    for (int i = 0; i < mjNSTATE; ++i) {
      ImGui::TableNextColumn();
      bool checked = tmp_.state_sig & (1 << i);
      ImGui::Checkbox(name_and_tooltip[i][0], &checked);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", name_and_tooltip[i][1]);
      }
      tmp_.state_sig =
          checked ? (tmp_.state_sig | (1 << i)) : (tmp_.state_sig & ~(1 << i));
    }
    ImGui::EndTable();
  }

  // Buttons to select commonly used state signatures.
  if (ImGui::BeginTable("##CommonSignatures", num_cols)) {
    ImGui::TableNextColumn();
    if (ImGui::Button("Physics", size)) {
      tmp_.state_sig =
          (tmp_.state_sig == mjSTATE_PHYSICS) ? 0 : mjSTATE_PHYSICS;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Full Physics", size)) {
      tmp_.state_sig =
          (tmp_.state_sig == mjSTATE_FULLPHYSICS) ? 0 : mjSTATE_FULLPHYSICS;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("User", size)) {
      tmp_.state_sig = (tmp_.state_sig == mjSTATE_USER) ? 0 : mjSTATE_USER;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Integration", size)) {
      tmp_.state_sig =
          (tmp_.state_sig == mjSTATE_INTEGRATION) ? 0 : mjSTATE_INTEGRATION;
    }
    ImGui::EndTable();
  }

  if (tmp_.state_sig != prev_state_sig) {
    const int size = mj_stateSize(model_, tmp_.state_sig);
    tmp_.state.resize(size);
  }

  if (tmp_.state.empty()) {
    // The state size is 0, let the user know why.
    ImGui::Separator();
    ImGui::BeginDisabled();
    ImGui::TextWrapped(
        tmp_.state_sig == 0
            ? "No state components are selected."
            : "Selected state components do not exist in the model.");
    ImGui::EndDisabled();
  } else {
    mj_getState(model_, data_, tmp_.state.data(), tmp_.state_sig);
    bool changed = false;

    if (ImGui::BeginTable(
            "State", 3,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter |
                ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable |
                ImGuiTableFlags_ScrollY,
            ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 20))) {
      ImGui::TableSetupColumn("Index");
      ImGui::TableSetupColumn("Name");
      ImGui::TableSetupColumn("Value");
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableHeadersRow();

      ImGuiListClipper clipper;
      clipper.Begin(tmp_.state.size());
      while (clipper.Step()) {
        int global = 0;
        for (int i = 0; i < mjNSTATE; ++i) {
          if (tmp_.state_sig & (1 << i)) {
            for (int local = 0; local < mj_stateSize(model_, (1 << i));
                 ++local, ++global) {
              if (global < clipper.DisplayStart) {
                continue;
              }
              if (global >= clipper.DisplayEnd) {
                break;
              }
              ImGui::TableNextRow();

              ImGui::TableNextColumn();
              ImGui::Text("%d", global);

              ImGui::TableNextColumn();
              ImGui::Text("%s[%d]", name_and_tooltip[i][0], local);

              ImGui::TableNextColumn();
              float value = tmp_.state[global];
              ImGui::PushItemWidth(-std::numeric_limits<float>::min());
              ImGui::PushID(global);
              if (ImGui::DragFloat("##value", &value, 0.01f, 0, 0, "%.3f")) {
                changed = true;
              }
              ImGui::PopID();
              ImGui::PopItemWidth();
              tmp_.state[global] = value;
            }
          }
        }
      }
      ImGui::EndTable();
    }

    if (changed) {
      mj_setState(model_, data_, tmp_.state.data(), tmp_.state_sig);
    }
  }

  ImGui::Indent(0.5f * ImGui::GetTreeNodeToLabelSpacing());
}

void App::WatchGui() {
  ImGui::InputText("Field", ui_.watch_field, sizeof(ui_.watch_field));
  ImGui::InputInt("Index", &ui_.watch_index);
  const mjtNum* value = static_cast<const mjtNum*>(
      toolbox::GetValue(model_, data_, ui_.watch_field, ui_.watch_index));

  toolbox::ScopedStyle style;
  style.Color(ImGuiCol_FrameBg, ImGui::GetStyle().Colors[ImGuiCol_WindowBg]);

  if (value) {
    char buf[100];
    int size = std::snprintf(buf, sizeof(buf), "%0.3f", *value);
    ImGui::InputText("Value", buf, size, ImGuiInputTextFlags_ReadOnly);
  } else {
    ImGui::BeginDisabled();
    style.Color(ImGuiCol_Text, ImColor(255, 0, 0, 255));
    char buf[] = "Invalid field/index!";
    ImGui::InputText("Value", buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);
    ImGui::EndDisabled();
  }
}

void App::PhysicsGui() {
  const float min_width = GetExpectedLabelWidth();
  const float available_width =
      ImGui::GetContentRegionAvail().x - ImGui::GetTreeNodeToLabelSpacing();
  const int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 6);

  auto& opt = model_->opt;

  const char* opts0[] = {"Euler", "RK4", "implicit", "implicitfast"};
  ImGui::Combo("Integrator", &opt.integrator, opts0, IM_ARRAYSIZE(opts0));

  const char* opts1[] = {"Pyramidal", "Elliptic"};
  ImGui::Combo("Cone", &opt.cone, opts1, IM_ARRAYSIZE(opts1));

  const char* opts2[] = {"Dense", "Sparse", "Auto"};
  ImGui::Combo("Jacobian", &opt.jacobian, opts2, IM_ARRAYSIZE(opts2));

  const char* opts3[] = {"PGS", "CG", "Newton"};
  ImGui::Combo("Solver", &opt.solver, opts3, IM_ARRAYSIZE(opts3));

  if (ImGui::TreeNodeEx("Disable Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("##DisableFlagsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNDISABLE; ++i) {
        ImGui::TableNextColumn();
        ImGui_BitToggle(mjDISABLESTRING[i], &opt.disableflags, 1 << i, size);
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Enable Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("##EnableFlagsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNENABLE; ++i) {
        ImGui::TableNextColumn();
        ImGui_BitToggle(mjENABLESTRING[i], &opt.enableflags, 1 << i, size);
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Actuator Group Disable")) {
    if (ImGui::BeginTable("##EnableFlagsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < 6; ++i) {
        char label[64];
        std::snprintf(label, sizeof(label), "Act Group %d", i);
        ImGui::TableNextColumn();
        ImGui_BitToggle(label, &opt.disableactuator, 1 << i, size);
      }
      ImGui::EndTable();
    }
    ImGui::TreePop();
  };

  if (ImGui::TreeNodeEx("Algorithmic Parameters")) {
    float w = ImGui::GetWindowWidth() * .6f;
    ImGui_Input("Timestep", &opt.timestep, {0, 1, 0.01, 0.1, w});
    ImGui_Input("Iterations", &opt.iterations, {0, 1000, 1, 10, w});
    ImGui_Input("Tolerance", &opt.tolerance, {0, 1, 1e-7, 1e-6, w});
    ImGui_Input("LS Iter", &opt.ls_iterations, {0, 100, 1, 0.1, w});
    ImGui_Input("LS Tol", &opt.ls_tolerance, {0, 0.1, 0.01, 0.1, w});
    ImGui_Input("Noslip Iter", &opt.noslip_iterations, {0, 1000, 1, 100, w});
    ImGui_Input("Noslip Tol", &opt.noslip_tolerance, {0, 1, 0.01, 0.1, w});
    ImGui_Input("CCD Iter", &opt.ccd_iterations, {0, 1000, 1, 100, w});
    ImGui_Input("CCD Tol", &opt.ccd_tolerance, {0, 1, 0.01, 0.1, w});
    ImGui_Input("Sleep Tol", &opt.sleep_tolerance, {0, 1, 0.01, 0.1, w});
    ImGui_Input("SDF Iter", &opt.sdf_iterations, {1, 20, 1, 10, w});
    ImGui_Input("SDF Init", &opt.sdf_initpoints, {1, 100, 1, 10, w});
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Physical Parameters")) {
    float w = ImGui::GetWindowWidth() * .6f;
    ImGui_InputN("Gravity", opt.gravity, 3, {.width = w});
    ImGui_InputN("Wind", opt.wind, 3, {.width = w});
    ImGui_InputN("Magnetic", opt.magnetic, 3, {.width = w});
    ImGui_Input("Density", &opt.density, {.min = .1, .max = 1, .width = w});
    ImGui_Input("Viscosity", &opt.viscosity, {.min = .1, .max = 1, .width = w});
    ImGui_Input("Imp Ratio", &opt.impratio, {.min = .1, .max = 1, .width = w});
    ImGui::TreePop();
  };

  if (ImGui::TreeNodeEx("Contact Override")) {
    float w = ImGui::GetWindowWidth() * .6f;
    ImGui_Input("Margin", &opt.o_margin, {.min = 0.1, .max = 1, .width = w});
    ImGui_InputN("Sol Imp", opt.o_solimp, 5, {.width = w, .format = "%0.1f"});
    ImGui_InputN("Sol Ref", opt.o_solref, 2, {.width = w, .format = "%0.1f"});
    ImGui_InputN("Friction", opt.o_friction, 5, {.width = w, .format = "%.1f"});
    ImGui::TreePop();
  }
}

void App::VisualizationGui() {
  auto& vis = model_->vis;
  auto& stat = model_->stat;

  ImGui::SliderInt("Tree depth", &vis_options_.bvh_depth, 0, 20);
  ImGui::SliderInt("Flex layer", &vis_options_.flex_layer, 0, 10);

  if (ImGui::TreeNodeEx("Headlight")) {
    ImGui_SwitchToggle("Active", &vis.headlight.active);
    ImGui::ColorEdit3("Ambient", vis.headlight.ambient);
    ImGui::ColorEdit3("Diffuse", vis.headlight.diffuse);
    ImGui::ColorEdit3("Specular", vis.headlight.specular);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Free Camera")) {
    ImGui_SwitchToggle("Orthographic", &vis.global.orthographic);
    ImGui_Input("FOV", &vis.global.fovy, {.format = "%0.2f"});
    ImGui_InputN("Center", stat.center, 3, {.format = "%0.2f"});
    ImGui_Input("Azimuth", &vis.global.azimuth, {.format = "%0.2f"});
    ImGui_Input("Elevation", &vis.global.elevation, {.format = "%0.2f"});
    if (ImGui::Button("Align")) {
      mjv_defaultFreeCamera(model_, &camera_);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Global")) {
    ImGui_Input("Extent", &stat.extent);
    const char* opts[] = {"Box", "Ellipsoid"};
    ImGui::SliderInt("Inertia", &vis.global.ellipsoidinertia, 0, 1,
                     opts[vis.global.ellipsoidinertia]);
    ImGui_ButtonToggle("BVH active", &vis.global.bvactive);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Mapping")) {
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    ImGui_Input("Stiffness", &vis.map.stiffness);
    ImGui_Input("Rot stiffness", &vis.map.stiffnessrot);
    ImGui_Input("Force", &vis.map.force);
    ImGui_Input("Torque", &vis.map.torque);
    ImGui_Input("Alpha", &vis.map.alpha);
    ImGui_Input("Fog start", &vis.map.fogstart);
    ImGui_Input("Fog end", &vis.map.fogend);
    ImGui_Input("Z near", &vis.map.znear);
    ImGui_Input("Z far", &vis.map.zfar);
    ImGui_Input("Haze", &vis.map.haze);
    ImGui_Input("Shadow clip", &vis.map.shadowclip);
    ImGui_Input("Shadow scale", &vis.map.shadowscale);
    ImGui::PopItemWidth();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Scale")) {
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    ImGui_Input("All (meansize)", &stat.meansize, {.format = "%0.3f"});
    ImGui_Input("Force width", &vis.scale.forcewidth);
    ImGui_Input("Contact width", &vis.scale.contactwidth);
    ImGui_Input("Contact height", &vis.scale.contactheight);
    ImGui_Input("Connect", &vis.scale.connect);
    ImGui_Input("Com", &vis.scale.com);
    ImGui_Input("Camera", &vis.scale.camera);
    ImGui_Input("Light", &vis.scale.light);
    ImGui_Input("Select point", &vis.scale.selectpoint);
    ImGui_Input("Joint length", &vis.scale.jointlength);
    ImGui_Input("Joint width", &vis.scale.jointwidth);
    ImGui_Input("Actuator length", &vis.scale.actuatorlength);
    ImGui_Input("Actuator width", &vis.scale.actuatorwidth);
    ImGui_Input("Frame length", &vis.scale.framelength);
    ImGui_Input("Frame width", &vis.scale.framewidth);
    ImGui_Input("Constraint", &vis.scale.constraint);
    ImGui_Input("Slider-crank", &vis.scale.slidercrank);
    ImGui::PopItemWidth();
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Colors")) {
    ImGui::ColorEdit4("Fog", vis.rgba.fog);
    ImGui::ColorEdit4("Haze", vis.rgba.haze);
    ImGui::ColorEdit4("Force", vis.rgba.force);
    ImGui::ColorEdit4("Inertia", vis.rgba.inertia);
    ImGui::ColorEdit4("Joint", vis.rgba.joint);
    ImGui::ColorEdit4("Actuator", vis.rgba.actuator);
    ImGui::ColorEdit4("Act. Negative", vis.rgba.actuatornegative);
    ImGui::ColorEdit4("Act. Positive", vis.rgba.actuatorpositive);
    ImGui::ColorEdit4("Center of Mass", vis.rgba.com);
    ImGui::ColorEdit4("Camera", vis.rgba.camera);
    ImGui::ColorEdit4("Light", vis.rgba.light);
    ImGui::ColorEdit4("Select Point", vis.rgba.selectpoint);
    ImGui::ColorEdit4("Auto Connect", vis.rgba.connect);
    ImGui::ColorEdit4("Contact Point", vis.rgba.contactpoint);
    ImGui::ColorEdit4("Contact Force", vis.rgba.contactforce);
    ImGui::ColorEdit4("Contact Friction", vis.rgba.contactfriction);
    ImGui::ColorEdit4("Contact Torque", vis.rgba.contacttorque);
    ImGui::ColorEdit4("Contact Gap", vis.rgba.contactgap);
    ImGui::ColorEdit4("Range Finder", vis.rgba.rangefinder);
    ImGui::ColorEdit4("Constraint", vis.rgba.constraint);
    ImGui::ColorEdit4("Slider Crank", vis.rgba.slidercrank);
    ImGui::ColorEdit4("Crank Broken", vis.rgba.crankbroken);
    ImGui::ColorEdit4("Frustum", vis.rgba.frustum);
    ImGui::ColorEdit4("Bounding Vol.", vis.rgba.bv);
    ImGui::ColorEdit4("BV Active", vis.rgba.bvactive);
    ImGui::TreePop();
  }
}

void App::RenderingGui() {
  const float min_width = GetExpectedLabelWidth();
  const float available_width =
      ImGui::GetContentRegionAvail().x - ImGui::GetTreeNodeToLabelSpacing();
  const int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 6);

  if (ImGui::TreeNodeEx("Model Elements", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

    if (ImGui::BeginTable("##ModelElementsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNVISFLAG; ++i) {
        ImGui::TableNextColumn();
        ImGui_ButtonToggle(mjVISSTRING[i][0], &vis_options_.flags[i], size);
      }
      ImGui::EndTable();
    }

    ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Render Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

    if (ImGui::BeginTable("##RenderFlagsTable", num_cols)) {
      const ImVec2 size = GetFlexElementSize(num_cols);
      for (int i = 0; i < mjNRNDFLAG; ++i) {
        ImGui::TableNextColumn();
        mjtByte flag = renderer_->GetFlag(static_cast<mjtRndFlag>(i));
        ImGui_ButtonToggle(mjRNDSTRING[i][0], &flag, size);
        renderer_->SetFlag(static_cast<mjtRndFlag>(i), flag);
      }
      ImGui::EndTable();
    }

    ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
    ImGui::TreePop();
  }
}

void App::GroupsGui() {
  const float min_width = GetExpectedLabelWidth();
  const float available_width = ImGui::GetContentRegionAvail().x;
  // We limit the number of columns to 1, 2, 3, or 6 depending on how much
  // space the window has available.
  int num_cols = std::clamp(
      static_cast<int>(std::floor(available_width / min_width)), 1, 6);
  if (num_cols == 4 || num_cols == 5) {
    num_cols = 3;
  }

  auto GroupGui = [&](const char* name, mjtByte* group) {
    if (ImGui::TreeNodeEx(name, ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

      char label[64];
      std::snprintf(label, sizeof(label), "##%s", name);
      if (ImGui::BeginTable(label, num_cols)) {
        const ImVec2 size = GetFlexElementSize(num_cols);
        for (int i = 0; i < 6; ++i) {
          ImGui::TableNextColumn();
          std::snprintf(label, sizeof(label), "%s %d", name, i);
          ImGui_ButtonToggle(label, &group[i], size);
        }

        ImGui::EndTable();
      }

      ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
      ImGui::TreePop();
    }
  };

  GroupGui("Geoms", vis_options_.geomgroup);
  GroupGui("Sites", vis_options_.sitegroup);
  GroupGui("Joints", vis_options_.jointgroup);
  GroupGui("Tendons", vis_options_.tendongroup);
  GroupGui("Actuators", vis_options_.actuatorgroup);
  GroupGui("Flexes", vis_options_.flexgroup);
  GroupGui("Skins", vis_options_.skingroup);
}

void App::NoiseGui() {
  float noise_scale, noise_rate;
  step_control_.GetNoiseParameters(noise_scale, noise_rate);
  ImGui::SliderFloat("Scale", &noise_scale, 0, 1);
  ImGui::SliderFloat("Rate", &noise_rate, 0, 4);
  step_control_.SetNoiseParameters(noise_scale, noise_rate);
}

void App::JointsGui() {
  char name[100];
  for (int i = 0; i < model_->njnt; ++i) {
    if (model_->jnt_type[i] != mjJNT_HINGE &&
        model_->jnt_type[i] != mjJNT_SLIDE) {
      continue;
    }
    const int group = std::clamp(model_->jnt_group[i], 0, mjNGROUP - 1);
    if (!vis_options_.jointgroup[group]) {
      continue;
    }

    const char* jnt_name = model_->names + model_->name_jntadr[i];
    if (*jnt_name) {
      std::snprintf(name, sizeof(name), "%s", jnt_name);
    } else {
      std::snprintf(name, sizeof(name), "joint %d", i);
    }

    double min = -1.0;
    double max = 1.0;
    if (model_->jnt_limited[i]) {
      min = model_->jnt_range[2 * i + 0];
      max = model_->jnt_range[2 * i + 1];
    } else if (model_->jnt_type[i] == mjJNT_SLIDE) {
      min = -1.0;
      max = 1.0;
    } else {
      min = -3.1416;
      max = 3.1416;
    }

    const int data_adr = model_->jnt_qposadr[i];
    ImGui_Slider(name, &data_->qpos[data_adr], min, max);
  }
}

void App::ControlsGui() {
  if (ImGui::Button("Clear All")) {
    mju_zero(data_->ctrl, model_->nu);
  }

  char name[100];
  for (int i = 0; i < model_->nu; i++) {
    int group = std::clamp(model_->actuator_group[i], 0, mjNGROUP - 1);
    if (!vis_options_.actuatorgroup[group]) {
      continue;
    }
    if (group >= 0 && group <= 30 &&
        model_->opt.disableactuator & (1 << group)) {
      continue;
    }

    const char* ctrl_name = model_->names + model_->name_actuatoradr[i];
    if (*ctrl_name) {
      std::snprintf(name, sizeof(name), "%s", ctrl_name);
    } else {
      std::snprintf(name, sizeof(name), "control %d", i);
    }

    double min = -1.0;
    double max = 1.0;
    if (!model_->actuator_ctrllimited[i]) {
      min = model_->actuator_ctrlrange[2 * i + 0];
      max = model_->actuator_ctrlrange[2 * i + 1];
    }
    ImGui_Slider(name, &data_->ctrl[i], min, max);
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

    tmp_.camera_names.push_back("Tumble");
    tmp_.camera_names.push_back("Free");
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
  };
}

void App::UiState::FromDict(const Dict& dict) {
  *this = UiState();
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
