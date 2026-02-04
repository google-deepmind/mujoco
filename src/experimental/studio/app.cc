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
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
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
#include "experimental/platform/file_dialog.h"
#include "experimental/platform/helpers.h"
#include "experimental/platform/imgui_widgets.h"
#include "experimental/platform/interaction.h"
#include "experimental/platform/picture_gui.h"
#include "experimental/platform/plugin.h"
#include "experimental/platform/renderer.h"
#include "experimental/platform/step_control.h"
#include "experimental/platform/window.h"
#include "user/user_resource.h"

namespace mujoco::studio {

static void ToggleFlag(mjtByte& flag) { flag = flag ? 0 : 1; }

static void ToggleWindow(bool& window) {
  window = !window;
  ImGui::GetIO().WantSaveIniSettings = true;
}

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

App::App(Config config)
    : ini_path_(std::move(config.ini_path)) {
  platform::Window::Config window_config;
  window_config.renderer_backend = platform::Renderer::GetBackend();
  window_config.offscreen_mode = config.offscreen_mode;
  window_ = std::make_unique<platform::Window>("MuJoCo Studio", config.width,
                                               config.height, window_config);
  renderer_ =
      std::make_unique<platform::Renderer>(window_->GetNativeWindowHandle());

  ImPlot::CreateContext();
  mjv_defaultPerturb(&perturb_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&vis_options_);

  profiler_.Clear();
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

  window_->SetTitle("MuJoCo Studio");

  step_control_.SetSpeed(100.f);
  profiler_.Clear();
  tmp_ = UiTempState();
  load_error_ = "";
  step_error_ = "";
}

void App::RequestModelLoad(std::string model_file) {
  pending_load_ = std::move(model_file);
}

void App::RequestModelReload() {
  if (model_kind_ == kModelFromFile) {
    pending_load_ = model_path_;
  }
}

void App::InitEmptyModel() {
  mjSpec* spec = mj_makeSpec();
  mjModel* model = mj_compile(spec, nullptr);
  InitModel(model, spec, nullptr, "", kEmptyModel);
}

void App::LoadModelFromFile(const std::string& filepath) {
  mjModel* model = nullptr;
  mjSpec* spec = nullptr;
  mjVFS vfs;
  mj_defaultVFS(&vfs);

  const std::string resolved_file =
      platform::ResolveFile(filepath, search_paths_);

  if (resolved_file.empty()) {
    SetLoadError("File not found: " + filepath);
    return;
  }

  char err[1000] = "";
  if (resolved_file.ends_with(".mjb")) {
    model = mj_loadModel(resolved_file.c_str(), &vfs);
  } else {
    spec = mj_parse(resolved_file.c_str(), nullptr, &vfs, err, sizeof(err));
  }
  if (err[0]) {
    SetLoadError(err);
    return;
  }

  InitModel(model, spec, &vfs, filepath, kModelFromFile);
  UpdateFilePaths(resolved_file);
  window_->SetTitle("MuJoCo Studio : " + filepath);

  mj_deleteVFS(&vfs);
}

struct BufferProvider : public mjpResourceProvider {
  BufferProvider(std::span<const std::byte> buffer) : buffer(buffer) {
    mjp_defaultResourceProvider(this);
    open = [](mjResource* resource) {
      return 1;
    };
    read = [](mjResource* resource, const void** buffer) {
      BufferProvider* self = (BufferProvider*)resource->provider;
      *buffer = self->buffer.data();
      return static_cast<int>(self->buffer.size());
    };
    close = [](mjResource* resource) {};
  }
  std::span<const std::byte> buffer;
};

void App::LoadModelFromBuffer(std::span<const std::byte> buffer,
                              std::string_view content_type,
                              std::string_view filename) {
  mjModel* model = nullptr;
  mjSpec* spec = nullptr;

  mjVFS vfs;
  mj_defaultVFS(&vfs);

  char err[1000] = "";
  if (content_type == "text/xml") {
    const char* ptr = reinterpret_cast<const char*>(buffer.data());
    spec = mj_parseXMLString(ptr, nullptr, err, sizeof(err));
  } else if (content_type == "application/mjb") {
    model = mj_loadModelBuffer(buffer.data(), buffer.size());
  } else if (content_type == "application/zip") {
    BufferProvider provider(buffer);
    mjResource resource;
    memset(&resource, 0, sizeof(mjResource));
    resource.vfs = &vfs;
    resource.provider = &provider;
    resource.name = (char*)filename.data();
    spec = mju_decodeResource(&resource, content_type.data(), &vfs);
  } else {
    SetLoadError("Unknown content type; expected text/xml or application/mjb");
    return;
  }
  if (err[0]) {
    SetLoadError(err);
    return;
  }

  InitModel(model, spec, &vfs, std::string(filename), kModelFromBuffer);

  mj_deleteVFS(&vfs);
}

void App::InitModel(mjModel* model, mjSpec* spec, mjVFS* vfs,
                    std::string filename, ModelKind model_kind) {
  if (model_kind_ == kEmptyModel) {
    step_control_.Unpause();
  }
  ClearModel();

  model_path_ = std::move(filename);
  model_kind_ = model_kind;
  if (model_kind_ == kEmptyModel) {
    step_control_.Pause();
  }

  spec_ = spec;
  model_ = model;

  // If we have a spec but not a model, we need to compile the model from the spec.
  if (spec_ && !model_) {
    model_ = mj_compile(spec_, vfs);
    if (!model_) {
      SetLoadError("Error compiling model from spec.");
      return;
    }
  }

  if (!model_) {
    mju_error("Error making data for model: %s", model_path_.c_str());
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    mju_error("Error making data for model: %s", model_path_.c_str());
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

  platform::ForEachModelPlugin([&](platform::ModelPlugin* plugin) {
    if (plugin->post_model_loaded) {
      plugin->post_model_loaded(plugin, model_path_.c_str());
    }
  });
}

void App::UpdateFilePaths(const std::string& resolved_path) {
  if (!resolved_path.empty()) {
    const std::filesystem::path path(resolved_path);
    const std::string base_path = path.parent_path().string() + "/";
    const std::string model_name = path.stem().string();
    tmp_.last_path[UiTempState::FileDialog_Load] =
        base_path + model_name + ".xml";
    tmp_.last_path[UiTempState::FileDialog_SaveMjb] =
        base_path + model_name + "_saved.mjb";
    tmp_.last_path[UiTempState::FileDialog_SaveXml] =
        base_path + model_name + "_saved.xml";
    tmp_.last_path[UiTempState::FileDialog_PrintModel] =
        base_path + model_name + "_MJMODEL.TXT";
    tmp_.last_path[UiTempState::FileDialog_PrintData] =
        base_path + model_name + "_MJDATA.TXT";
    tmp_.last_path[UiTempState::FileDialog_SaveScreenshot] =
        base_path + "screenshot.webp";
  }
}

void App::SetLoadError(std::string error) {
  InitEmptyModel();
  load_error_ = std::move(error);
}

void App::ResetPhysics() {
  mj_resetData(model(), data());
  mj_forward(model(), data());
  step_error_ = "";
}

void App::UpdatePhysics() {
  if (!has_model()) {
    return;
  }

  bool stepped = false;
  platform::ForEachModelPlugin([&](platform::ModelPlugin* plugin) {
    if (plugin->do_update) {
      if (plugin->do_update(plugin, model(), data())) {
        stepped = true;
      }
    }
  });

  if (!stepped) {
    if (!step_control_.IsPaused()) {
      mju_zero(data()->xfrc_applied, 6 * model()->nbody);
      mjv_applyPerturbPose(model(), data(), &perturb_, 0);
      mjv_applyPerturbForce(model(), data(), &perturb_);
    } else {
      mjv_applyPerturbPose(model(), data(), &perturb_, 1);
    }

    if (has_data()) {
      for (int i = 0; i < mjNTIMER; i++) {
        data()->timer[i].duration = 0;
        data()->timer[i].number = 0;
      }
    }

    platform::StepControl::Status status =
        step_control_.Advance(model(), data());
    if (status == platform::StepControl::Status::kPaused) {
      // do nothing
    } else if (status == platform::StepControl::Status::kOk) {
      stepped = true;
      // If we are adding to the history we didn't have a divergence error
      step_error_ = "";
    } else if (status == platform::StepControl::Status::kAutoReset) {
      ResetPhysics();
    } else if (status == platform::StepControl::Status::kDiverged) {
      stepped = true;
      for (mjtWarning w : platform::StepControl::kDivergedWarnings) {
        if (data()->warning[w].number > 0) {
          step_error_ = mju_warningText(w, data()->warning[w].lastinfo);
        }
      }
    }
  }

  if (stepped) {
    profiler_.Update(model(), data());
    std::span<mjtNum> state = history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model(), data(), state.data(), mjSTATE_INTEGRATION);
    }
  }
}

void App::LoadHistory(int offset) {
  std::span<mjtNum> state = history_.SetIndex(offset);
  if (!state.empty()) {
    // Pause simulation when entering history mode.
    step_control_.Pause();

    // Load the state into the data buffer.
    mj_setState(model(), data(), state.data(), mjSTATE_INTEGRATION);
    mj_forward(model(), data());
  }
}

bool App::Update() {
  const platform::Window::Status status = window_->NewFrame();

  HandleWindowEvents();
  HandleMouseEvents();
  HandleKeyboardEvents();

  ProcessPendingLoads();

  // Only update the simulation if a popup window is not open. Note that the
  // simulation itself will only update if it is not paused.
  if (tmp_.file_dialog == UiTempState::FileDialog_None) {
    UpdatePhysics();
  }

  return status == platform::Window::Status::kRunning && !tmp_.should_exit;
}

void App::Render() {
  const float width = window_->GetWidth();
  const float height = window_->GetHeight();
  const float scale = window_->GetScale();

  if (window_->IsOffscreenMode()) {
    pixels_.resize(width * height * 3);
  } else {
    pixels_.clear();
  }
  renderer_->Render(model(), data(), &perturb_, &camera_, &vis_options_,
                    width * scale, height * scale, pixels_);

  window_->EndFrame();
  window_->Present(pixels_);

  if (has_data()) {
    for (int i = 0; i < mjNTIMER; i++) {
      data()->timer[i].duration = 0;
      data()->timer[i].number = 0;
    }
  }
}

void App::ProcessPendingLoads() {
  // Check to see if we need to load a new model.
  if (pending_load_.has_value()) {
    std::string load_data = std::move(pending_load_.value());
    pending_load_.reset();

    if (load_data.empty()) {
      InitEmptyModel();
    } else {
      LoadModelFromFile(load_data);
    }
  }

  // Check plugins to see if we need to load a new model.
  platform::ForEachModelPlugin([&](platform::ModelPlugin* plugin) {
    if (plugin->get_model_to_load) {
      char model_name[1000] = "";
      char content_type[1000] = "";
      int size = 0;
      const char* buf = plugin->get_model_to_load(
          plugin, &size, content_type, sizeof(content_type), model_name,
          sizeof(model_name));
      if (buf && size) {
        const std::byte* bytes = reinterpret_cast<const std::byte*>(buf);
        LoadModelFromBuffer({bytes, bytes + size}, content_type, model_name);
      }
    }
  });
}

void App::HandleWindowEvents() {
  const std::string drop_file = window_->GetDropFile();
  if (!drop_file.empty()) {
    RequestModelLoad(drop_file);
  }
}

void App::HandleMouseEvents() {
  auto& io = ImGui::GetIO();
  if (io.WantCaptureMouse) {
    return;
  }

  if (!has_model() || !has_data()) {
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
        platform::InitPerturb(model(), data(), &camera_, &perturb_, active);
      }
      platform::MovePerturb(model(), data(), &camera_, &perturb_, action,
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
        platform::Pick(model(), data(), &camera_, mouse_x, mouse_y,
                       window_->GetAspectRatio(), &vis_options_);
    if (picked.body >= 0) {
      perturb_.select = picked.body;
      perturb_.flexselect = picked.flex;
      perturb_.skinselect = picked.skin;

      // Compute the local position of the selected object in the world.
      mjtNum tmp[3];
      mju_sub3(tmp, picked.point, data()->xpos + 3 * picked.body);
      mju_mulMatTVec(perturb_.localpos, data()->xmat + 9 * picked.body, tmp, 3,
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
        platform::Pick(model(), data(), &camera_, mouse_x, mouse_y,
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
    tmp_.file_dialog = UiTempState::FileDialog_Load;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMode_CtrlShift)) {
    tmp_.file_dialog = UiTempState::FileDialog_SaveMjb;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_SaveXml;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_M | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_PrintModel;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_D | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_PrintData;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_P | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_SaveScreenshot;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C | ImGuiMod_Ctrl)) {
    std::string keyframe = platform::KeyframeToString(model(), data(), false);
    platform::MaybeSaveToClipboard(keyframe);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_L | ImGuiMod_Ctrl)) {
    RequestModelReload();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q | ImGuiMod_Ctrl)) {
    tmp_.should_exit = true;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_A | ImGuiMod_Ctrl)) {
    mjv_defaultFreeCamera(model(), &camera_);
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
    SelectParentPerturb(model(), perturb_);
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
    tmp_.chart_performance = !tmp_.chart_performance;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F11)) {
    tmp_.full_screen = !tmp_.full_screen;
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
  } else if (has_model()) {
    if (ImGui_IsChordJustPressed(ImGuiKey_Escape)) {
      ui_.camera_idx =
          platform::SetCamera(model(), &camera_, platform::kTumbleCameraIdx);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftBracket)) {
      ui_.camera_idx =
          platform::SetCamera(model(), &camera_, ui_.camera_idx - 1);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_RightBracket)) {
      ui_.camera_idx =
          platform::SetCamera(model(), &camera_, ui_.camera_idx + 1);
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

      platform::KeyValues plugin_names =
          platform::ReadIniSection(settings, "[Studio][Plugins]");
      platform::ForEachGuiPlugin([&](platform::GuiPlugin* plugin) {
        auto it = plugin_names.find(plugin->name);
        if (it != plugin_names.end()) {
          plugin->active = std::stoi(it->second) != 0;
        }
      });
    }
  }
}

void App::SaveSettings() {
  if (!ini_path_.empty()) {
    std::string settings = ImGui::SaveIniSettingsToMemory();
    platform::AppendIniSection(settings, "[Studio][UX]", ui_.ToDict());

    platform::KeyValues plugin_names;
    platform::ForEachGuiPlugin([&](platform::GuiPlugin* plugin) {
      plugin_names[plugin->name] = std::to_string((int)plugin->active);
    });
    platform::AppendIniSection(settings, "[Studio][Plugins]", plugin_names);

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
  platform::MoveCamera(model(), data(), &camera_, motion, reldx, reldy);
}

void App::BuildGui() {
  if (tmp_.full_screen) {
    return;
  }

  SetupTheme(ui_.theme);
  const ImVec4 workspace_rect = platform::ConfigureDockingLayout();

  // Place charts in bottom right corner of the workspace.
  const ImVec2 chart_size(250, 500);
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

  if (tmp_.chart_performance) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Performance", &tmp_.chart_performance)) {
      profiler_.CpuTimeGraph();
      profiler_.DimensionsGraph();
    }
    ImGui::End();
  }

  if (tmp_.chart_solver) {
    ImGui::SetNextWindowPos(chart_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(chart_size, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Solver", &tmp_.chart_solver)) {
      platform::CountsGui(model(), data());
      platform::ConvergenceGui(model(), data());
    }
    ImGui::End();
  }

  if (tmp_.picture_in_picture) {
    if (ImGui::Begin("Picture-in-Picture", &tmp_.picture_in_picture)) {
      PipGui(model(), data(), window_.get(), renderer_.get(), &tmp_.pips);
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
      const float fps = renderer_->GetFps();
      platform::StatsGui(model(), data(), step_control_.IsPaused(), fps);
    }
    ImGui::End();
  }

  // Display a drag-and-drop message if no model is loaded.
  if (model_kind_ == kEmptyModel) {
    #ifndef __EMSCRIPTEN__
    const char* text = "Load model file or drag-and-drop model file here.";

    const float width = window_->GetWidth() * ImGui::GetWindowDpiScale();
    const float height = window_->GetHeight() * ImGui::GetWindowDpiScale();
    const ImVec2 text_size = ImGui::CalcTextSize(text);

    ImGui::SetNextWindowPos(ImVec2((width - text_size.x) / 2, height / 2),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(text_size.x + 10, text_size.y + 10),
                             ImGuiCond_Always);
    const ImGuiWindowFlags kOverlayFlags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoDocking;
    if (ImGui::Begin("##Overlay", 0, kOverlayFlags)) {
      platform::ScopedStyle style;
      style.Color(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
      ImGui::Text("%s", text);
    }
    ImGui::End();
    #endif  // !__EMSCRIPTEN__
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

  platform::ForEachGuiPlugin([](platform::GuiPlugin* plugin) {
    if (!plugin->update) {
      return;
    }
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Plugins")) {
        if (ImGui::MenuItem(plugin->name, "", plugin->active)) {
          plugin->active = !plugin->active;
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    if (plugin->active) {
      ImGui::Begin(plugin->name);
      plugin->update(plugin);
      ImGui::End();
    }
  });

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
    platform::PhysicsGui(model(), min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("RenderingGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Rendering Settings", node_flags)) {
    platform::RenderingGui(model(), &vis_options_, renderer_->GetRenderFlags(),
                           min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("GroupsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Visibility Groups", node_flags)) {
    platform::GroupsGui(model(), &vis_options_, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("VisualizationGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Visualization", node_flags)) {
    platform::VisualizationGui(model(), &vis_options_, &camera_, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();
}

void App::DataInspectorGui() {
  if (!has_data()) {
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
    platform::NoiseGui(model(), data(), noise_scale, noise_rate);
    step_control_.SetNoiseParameters(noise_scale, noise_rate);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("JointsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Joints", node_flags)) {
    platform::JointsGui(model(), data(), &vis_options_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("ControlsGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Controls", node_flags)) {
    platform::ControlsGui(model(), data(), &vis_options_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("SensorGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Sensor", node_flags)) {
    platform::SensorGui(model(), data());
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("WatchGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("Watch", node_flags)) {
    platform::WatchGui(model(), data(), ui_.watch_field,
                       sizeof(ui_.watch_field), ui_.watch_index);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("StateGui", {0, 0}, child_flags);
  if (ImGui::TreeNodeEx("State", node_flags)) {
    platform::StateGui(model(), data(), tmp_.state, tmp_.state_sig, min_width);
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
  if (!has_spec()) {
    ImGui::Text("No mjSpec loaded.");
    return;
  }

  const ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  auto display_group = [this](mjtObj type, const std::string& prefix) {
    mjsElement* element = mjs_firstElement(spec(), type);
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

      element = mjs_nextElement(spec(), element);
    }
  };

  if (ImGui::TreeNodeEx("Bodies", flags)) {
    // We don't use `display_group` here because we do additional selection
    // logic tied to the `perturb_` field.
    mjsElement* element = mjs_firstElement(spec(), mjOBJ_BODY);
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

      element = mjs_nextElement(spec(), element);
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
      platform::BodyPropertiesGui(model(), data(), tmp_.element,
                                  tmp_.element_id);
      break;
    case mjOBJ_JOINT:
      ImGui::Text("Joint");
      ImGui::Separator();
      platform::JointPropertiesGui(model(), data(), tmp_.element,
                                   tmp_.element_id);
      break;
    case mjOBJ_SITE:
      ImGui::Text("Site");
      ImGui::Separator();
      platform::SitePropertiesGui(model(), data(), tmp_.element,
                                  tmp_.element_id);
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
  ImGui::Text("Solver Charts");
  ImGui::Text("Perf. Charts");
  ImGui::Text("Toggle Fullscreen");
  ImGui::Text("Free Camera");
  ImGui::Text("Toggle Pause");
  ImGui::Text("Reset Sim");
  ImGui::Text("Toggle Left UI");
  ImGui::Text("Toggle Right UI");
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
  ImGui::Text("F9");
  ImGui::Text("F10");
  ImGui::Text("F11");
  ImGui::Text("Esc");
  ImGui::Text("Spc");
  ImGui::Text("Bksp");
  ImGui::Text("Tab");
  ImGui::Text("Sh+Tab");
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

    const float scale = ImGui::GetWindowDpiScale();
    const float right_width = 520.f * scale;
    const ImVec2 button_size(48.f * scale, 32.f * scale);
    const ImVec2 play_button_size(120.f * scale, 32.f * scale);

    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, right_width);

    ImGui::TableNextColumn();
    ImGui::Text("%s", " ");

    // Unload button.
    ImGui::SameLine();
    style.Color(ImGuiCol_ButtonHovered, red);
    if (ImGui::Button(ICON_UNLOAD_MODEL, button_size)) {
      InitEmptyModel();
    }
    ImGui::SetItemTooltip("%s", "Unload");
    style.Reset();

    // Reload button.
    ImGui::SameLine();
    if (ImGui::Button(ICON_RELOAD_MODEL, button_size)) {
      RequestModelReload();
    }
    ImGui::SetItemTooltip("%s", "Reload");

    // Reset button.
    ImGui::SameLine();
    if (ImGui::Button(ICON_RESET_MODEL, button_size)) {
      ResetPhysics();
    }
    ImGui::SetItemTooltip("%s", "Reset");

    // Play/pause button.
    ImGui::SameLine();
    const bool paused = step_control_.IsPaused();
    style.Color(ImGuiCol_Button, paused ? yellow : green);
    if (ImGui::Button(paused ? ICON_PLAY : ICON_PAUSE, play_button_size)) {
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
    ImGui::SetNextItemWidth(50.0f * scale);
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
          ui_.camera_idx = platform::SetCamera(model(), &camera_,
                                               n + platform::kTumbleCameraIdx);
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("%s", "Camera");
    ImGui::SameLine();
    if (ImGui::Button(ICON_COPY_CAMERA)) {
      std::string camera_string = platform::CameraToString(data(), &camera_);
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

    if (!has_model()) {
      ImGui::Text("No model loaded");
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

    if (!step_error_.empty()) {
      ImGui::SameLine();
      ImGui::Text(" | Step Error: %s", step_error_.c_str());
    } else if (!load_error_.empty()) {
      ImGui::SameLine();
      ImGui::Text(" | Load Error: %s", load_error_.c_str());
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
      #ifndef __EMSCRIPTEN__
      if (ImGui::MenuItem("Open Model File", "Ctrl+O")) {
        tmp_.file_dialog = UiTempState::FileDialog_Load;
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Save XML", "Ctrl+S")) {
        tmp_.file_dialog = UiTempState::FileDialog_SaveXml;
      }
      if (ImGui::MenuItem("Save MJB", "Ctrl+Shift+S")) {
        tmp_.file_dialog = UiTempState::FileDialog_SaveMjb;
      }
      if (ImGui::MenuItem("Save Screenshot", "Ctrl+P")) {
        tmp_.file_dialog = UiTempState::FileDialog_SaveScreenshot;
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Print Model", "Ctrl+M")) {
        tmp_.file_dialog = UiTempState::FileDialog_PrintModel;
      }
      if (ImGui::MenuItem("Print Data", "Ctrl+D")) {
        tmp_.file_dialog = UiTempState::FileDialog_PrintData;
      }
      ImGui::Separator();
      #endif  // !__EMSCRIPTEN__
      if (ImGui::MenuItem("Unload", "Ctrl+U")) {
        InitEmptyModel();
      }
      #ifndef __EMSCRIPTEN__
      ImGui::Separator();
      if (ImGui::MenuItem("Quit", "Ctrl+Q")) {
        tmp_.should_exit = true;
      }
      #endif  // !__EMSCRIPTEN__
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
        RequestModelReload();
      }
      ImGui::Separator();
      if (ImGui::BeginMenu("Keyframes")) {
        ImGui::SetNextItemWidth(200);
        ImGui::SliderInt("##Key", &ui_.key_idx, 0, model()->nkey);
        if (ImGui::MenuItem("Load")) {
          mj_resetDataKeyframe(model(), data(), ui_.key_idx);
          mj_forward(model(), data());
        }
        if (ImGui::MenuItem("Save")) {
          mj_setKeyframe(model(), data(), ui_.key_idx);
        }
        if (ImGui::MenuItem("Copy")) {
          std::string str = platform::KeyframeToString(model(), data(), false);
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
      if (ImGui::MenuItem("Full Screen", "F11")) {
        tmp_.full_screen = !tmp_.full_screen;
      }
      ImGui::Separator();

      if (ImGui::MenuItem("Picture-in-Picture")) {
        tmp_.picture_in_picture = !tmp_.picture_in_picture;
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Charts")) {
      if (ImGui::MenuItem("Solver", "F9")) {
        tmp_.chart_solver = !tmp_.chart_solver;
      }
      if (ImGui::MenuItem("Performance", "F10")) {
        tmp_.chart_performance = !tmp_.chart_performance;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Plugins")) {
      // Placeholder menu item that will be populated by plugins later on. We
      // do this now in so that the menu is present at the right place.
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
  if (tmp_.file_dialog == UiTempState::FileDialog_None) {
    return;
  }

  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  if (ImGui::BeginPopupModal("FileDialog", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    const platform::DialogResult res =
        tmp_.file_dialog == UiTempState::FileDialog_Load
            ? platform::OpenFileDialog(tmp_.filename)
            : platform::SaveFileDialog(tmp_.filename);

    if (res.status == platform::DialogResult::kAccepted) {
      tmp_.last_path[tmp_.file_dialog] = res.path;
      switch (tmp_.file_dialog) {
        case UiTempState::FileDialog_Load:
          RequestModelLoad(res.path);
          break;
        case UiTempState::FileDialog_SaveXml:
          mj_saveLastXML(res.path.c_str(), model(), nullptr, 0);
          break;
        case UiTempState::FileDialog_SaveMjb:
          mj_saveModel(model(), res.path.c_str(), nullptr, 0);
          break;
        case UiTempState::FileDialog_SaveScreenshot: {
          const int width = window_->GetWidth();
          const int height = window_->GetHeight();
          std::vector<std::byte> buffer(width * height * 3);
          renderer_->RenderToTexture(model(), data(), &camera_, width, height,
                                     buffer.data());
          platform::SaveToWebp(width, height, buffer.data(), res.path);
          break;
        }
        case UiTempState::FileDialog_PrintModel:
          mj_printModel(model(), res.path.c_str());
          break;
        case UiTempState::FileDialog_PrintData:
          mj_printData(model(), data(), res.path.c_str());
          break;
        default:
          break;
      }
    }

    if (res.status != platform::DialogResult::kError) {
      strncpy(tmp_.filename, res.path.c_str(), res.path.size());
      tmp_.filename[res.path.size()] = 0;
    }
    if (res.status != platform::DialogResult::kPending) {
      tmp_.file_dialog = UiTempState::FileDialog_None;
      ImGui::CloseCurrentPopup();
      window_->EnableWindowResizing();
    }
    ImGui::EndPopup();
  }

  if (!ImGui::IsPopupOpen("FileDialog") &&
      tmp_.file_dialog != UiTempState::FileDialog_None) {
    const std::string& src = tmp_.last_path[tmp_.file_dialog];
    strncpy(tmp_.filename, src.c_str(), src.size());
    tmp_.filename[src.size()] = 0;
    ImGui::OpenPopup("FileDialog");
    window_->DisableWindowResizing();
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
    tmp_.camera_names.reserve(model()->ncam + 3);

    tmp_.camera_names.push_back("Free: tumble");
    tmp_.camera_names.push_back("Free: wasd");
    tmp_.camera_names.push_back("Tracking (-1)");
    for (int i = 0; i < model()->ncam; i++) {
      if (model()->names[model()->name_camadr[i]]) {
        tmp_.camera_names.push_back(model()->names + model()->name_camadr[i]);
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
}  // namespace mujoco::studio
