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
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/hal/renderer.h"
#include "experimental/platform/hal/window.h"
#include "experimental/platform/helpers.h"
#include "experimental/platform/sim/model_holder.h"
#include "experimental/platform/sim/step_control.h"
#include "experimental/platform/ux/file_dialog.h"
#include "experimental/platform/ux/gui.h"
#include "experimental/platform/ux/gui_spec.h"
#include "experimental/platform/ux/imgui_widgets.h"
#include "experimental/platform/ux/interaction.h"
#include "experimental/platform/ux/picture_gui.h"
#include "experimental/platform/ux/plugin.h"
#include "experimental/studio/llm/source_search.h"
#include "experimental/studio/mujoco_logo.h"

namespace mujoco::studio {

using PauseState = platform::StepControl::PauseState;

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
  // TODO: update selected element!
}

static constexpr const char* ICON_COPY_CAMERA = platform::ICON_FA_COPY;
static constexpr const char* ICON_RELOAD_MODEL = platform::ICON_FA_REFRESH;
static constexpr const char* ICON_RESET_MODEL = platform::ICON_FA_UNDO;
static constexpr const char* ICON_PREV_FRAME = platform::ICON_FA_CARET_LEFT;
static constexpr const char* ICON_NEXT_FRAME = platform::ICON_FA_CARET_RIGHT;
static constexpr const char* ICON_CURR_FRAME = platform::ICON_FA_FAST_FORWARD;
static constexpr const char* ICON_UNDO_SPEC = platform::ICON_FA_UNDO;
static constexpr const char* ICON_REDO_SPEC = platform::ICON_FA_REPEAT;

App::App(Config config)
    : app_title_(std::move(config.title)),
      ini_path_(std::move(config.ini_path)),
      gfx_mode_(config.gfx_mode),
      screenshot_path_(std::move(config.screenshot_path)),
      screenshot_frame_(config.screenshot_frame) {
  SwitchGraphicsMode(config.width, config.height, config.gfx_mode);

  if (config.initial_theme.has_value()) {
    ui_.theme = *config.initial_theme;
  }

  ImPlot::CreateContext();
  mjv_defaultPerturb(&perturb_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&vis_options_);

  RegisterToolWindows();
  RegisterLlmTools();
  profiler_.Clear();
}

App::~App() {
  // Stop the test engine while the ImGui context is still alive. We don't
  // Destroy() it (that must follow ImGui::DestroyContext, which the window never
  // calls); leaking the engine at process exit is harmless.
  test_runner_.Stop();
}

void App::SwitchGraphicsMode(int width, int height,
                             platform::GraphicsMode mode) {
  // The test engine is bound to the ImGui context that the window owns. Stop it
  // before the window is torn down. (We don't Destroy()/restart across a real
  // graphics switch yet -- not exercised here; the engine just stays stopped.)
  test_runner_.Stop();
  renderer_.reset();
  window_.reset();
  gfx_mode_ = mode;
  // The renderer owns GPU textures; force the logo to re-upload on the new one.
  logo_texture_ = 0;

  platform::Window::Config window_config;
  window_config.gfx_mode = gfx_mode_;
  window_ = std::make_unique<platform::Window>(app_title_, width, height,
                                               window_config);
  renderer_ = std::make_unique<platform::Renderer>(
      window_->GetNativeWindowHandle(), gfx_mode_);

  // TODO: Figure out why this breaks on some platforms.
  // LoadSettings();
  if (ui_.window_width > 0 && ui_.window_height > 0) {
    window_->Resize(ui_.window_width, ui_.window_height);
  }

  // The Window ctor created a fresh ImGui context; bind the test engine to it.
  test_runner_.Start();
}

void App::Recompile() {
  mj_recompile(model_holder_->spec(), model_holder_->vfs(),
               model_holder_->model(), model_holder_->data());
  const int state_size = mj_stateSize(model(), mjSTATE_INTEGRATION);
  sim_history_.Init(state_size);
}

void App::RequestModelLoad(std::string model_file) {
  pending_load_ = std::move(model_file);
}

void App::RequestModelReload() {
  if (model_kind_ == kModelFromFile ||
      (model_kind_ == kEmptyModel && !model_path_.empty())) {
    pending_load_ = model_path_;
    preserve_camera_on_load_ = true;
  }
}

void App::InitEmptyModel() {
  model_holder_ = platform::ModelHolder::FromSpec(mj_makeSpec());
  OnModelLoaded("", kEmptyModel);
  spec_editor_.Reset(*spec());
}

void App::LoadModelFromFile(const std::string& filepath) {
  const std::string resolved_file =
      platform::ResolveFile(filepath, search_paths_);
  model_holder_ = platform::ModelHolder::FromFile(resolved_file);
  if (model_holder_->ok()) {
    OnModelLoaded(filepath, kModelFromFile);
    spec_editor_.Reset(*spec());
    UpdateFilePaths(resolved_file);
    if (model() && model()->names) {
      // Assumes the first string in the model is the name of the model itself.
      window_->SetTitle(app_title_ + " : " + std::string(model()->names));
    } else {
      window_->SetTitle(app_title_ + " : " +
                        std::filesystem::path(filepath).stem().string());
    }
  } else {
    SetLoadError(std::string(model_holder_->error()));
    // Keep track of the attempted load in case the user fixes the error and
    // tries to reload the same file again.
    model_path_ = resolved_file;
  }
}

void App::LoadModelFromBuffer(std::span<const std::byte> buffer,
                              std::string_view content_type,
                              std::string_view filename) {
  model_holder_ =
      platform::ModelHolder::FromBuffer(buffer, content_type, filename);
  if (model_holder_->ok()) {
    OnModelLoaded(std::string(filename), kModelFromFile);
  } else {
    SetLoadError(std::string(model_holder_->error()));
  }
  spec_editor_.Reset(*spec());
}

void App::OnModelLoaded(std::string filename, ModelKind model_kind) {
  load_error_ = "";
  step_error_ = "";
  edit_error_ = "";

  model_path_ = std::move(filename);

  if (model_kind_ == kEmptyModel) {
    step_control_.SetPauseState(PauseState::kUnpaused);
  }
  model_kind_ = model_kind;
  if (model_kind_ == kEmptyModel) {
    step_control_.SetPauseState(PauseState::kNormalPaused);
  }

  // Reset/reinitialize everything that depends on the new mjModel.
  mjModel* model = model_holder_->model();
  renderer_->Init(model);
  const int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  sim_history_.Init(state_size);

  if (!preserve_camera_on_load_) {
    const int model_cam = model->vis.global.cameraid;
    if (model_cam >= 0 && model_cam < model->ncam) {
      ui_.camera_idx = platform::SetCamera(model, &camera_, model_cam);
    } else {
      mjv_defaultFreeCamera(model, &camera_);
    }
  }
  preserve_camera_on_load_ = false;

  // Initialize the speed based on the model's default real-time setting.
  float min_error = FLT_MAX;
  const float desired = mju_log(100 * model->vis.global.realtime);
  for (int i = 0; i < platform::kPercentRealTime.size(); ++i) {
    const float speed = std::stof(platform::kPercentRealTime[i]);
    const float error = mju_abs(mju_log(speed) - desired);
    if (error < min_error) {
      min_error = error;
      SetSpeedIndex(i);
    }
  }

  platform::ForEachPlugin<platform::ModelPlugin>([&](auto* plugin) {
    if (plugin->post_model_loaded) {
      plugin->post_model_loaded(plugin, model_path_.c_str());
    }
  });
  tmp_.update_threadpool = true;
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
  step_error_ = "";
  edit_error_ = "";
}

void App::ResetPhysics() {
  mj_resetData(model(), data());
  mj_forward(model(), data());
  step_error_ = "";
  edit_error_ = "";
}

void App::UpdatePhysics() {
  if (!has_model()) {
    return;
  }

  if (tmp_.update_threadpool) {
    mju_threadpool(data(), ui_.nthread);
    tmp_.update_threadpool = false;
  }

  bool stepped = false;
  platform::ForEachPlugin<platform::ModelPlugin>([&](auto* plugin) {
    if (plugin->do_update) {
      if (plugin->do_update(plugin, model(), data())) {
        stepped = true;
      }
    }
  });

  if (!stepped) {
    if (step_control_.GetPauseState() != PauseState::kNormalPaused) {
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
    std::span<mjtNum> state = sim_history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model(), data(), state.data(), mjSTATE_INTEGRATION);
    }
  }
}

void App::LoadHistory(int offset) {
  std::span<mjtNum> state = sim_history_.SetIndex(offset);
  if (!state.empty()) {
    // Pause simulation when entering history mode.
    step_control_.SetPauseState(PauseState::kNormalPaused);

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
  if (IsHeadless(window_->GetGraphicsMode())) {
    pixels_.resize(width * height * 3);
  } else {
    pixels_.clear();
  }
  renderer_->Render(model(), data(), &perturb_, &camera_, &vis_options_,
                    width * scale, height * scale, pixels_);

  window_->EndFrame();
  window_->Present(pixels_);

  // Tick the test engine after the swap so it can advance any queued UI program
  // (the run_ui_program tool) and perform clicks across frames.
  test_runner_.PostSwap();

  // Auto-screenshot: once enough frames have rendered for the model and GUI to
  // settle, dump the headless framebuffer (RGB888, scene + GUI) to a PPM and
  // request exit. Only valid in headless mode, where pixels_ is populated.
  if (!screenshot_path_.empty() && IsHeadless(window_->GetGraphicsMode())) {
    if (frame_count_ == screenshot_frame_) {
      const int w = static_cast<int>(width);
      const int h = static_cast<int>(height);
      std::FILE* f = std::fopen(screenshot_path_.c_str(), "wb");
      if (f != nullptr) {
        std::fprintf(f, "P6\n%d %d\n255\n", w, h);
        std::fwrite(pixels_.data(), 1, pixels_.size(), f);
        std::fclose(f);
        mju_warning("Saved screenshot to %s (%dx%d)", screenshot_path_.c_str(),
                    w, h);
      } else {
        mju_warning("Could not open screenshot file %s",
                    screenshot_path_.c_str());
      }
      tmp_.should_exit = true;
    }
    ++frame_count_;
  }

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

  if (pending_op_) {
    pending_op_();
    pending_op_ = nullptr;
  }

  // Allow plugins to edit the spec as well.
  platform::ForEachPlugin<platform::SpecEditorPlugin>([&](auto* plugin) {
    if (plugin->pre_compile) {
      if (plugin->pre_compile(plugin, spec(), model(), data(), &camera_)) {
        Recompile();
        if (plugin->post_compile) {
          plugin->post_compile(plugin, spec(), model(), data());
        }
      };
    }
  });

  // Check plugins to see if we need to load a new model.
  platform::ForEachPlugin<platform::ModelPlugin>([&](auto* plugin) {
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
        if (io.KeyAlt) {
          action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else {
          action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        action = mjMOUSE_ZOOM;
      }
      const mjtPertBit active =
          (action == mjMOUSE_MOVE_V || action == mjMOUSE_MOVE_H)
              ? mjPERT_TRANSLATE
              : mjPERT_ROTATE;
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

      // Select the corresponding element in the spec.
      tmp_.curr_element = nullptr;
      if (has_spec()) {
        mjsElement* element = mjs_firstElement(spec(), mjOBJ_BODY);
        while (element) {
          if (mjs_getId(element) == picked.body) {
            tmp_.curr_element = element;
            break;
          }
          element = mjs_nextElement(spec(), element);
        }
      }

      // Compute the local position of the selected object in the world.
      mjtNum tmp[3];
      mju_sub3(tmp, picked.point, data()->xpos + 3 * picked.body);
      mju_mulMatTVec(perturb_.localpos, data()->xmat + 9 * picked.body, tmp, 3,
                     3);
    } else {
      mjv_defaultPerturb(&perturb_);
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

  constexpr auto ImGuiMod_CtrlShift = ImGuiMod_Ctrl | ImGuiMod_Shift;

  bool is_freecam_wasd = ui_.camera_idx == platform::kFreeCameraIdx;

  // Menu shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_O | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_Load;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMod_CtrlShift)) {
    tmp_.file_dialog = UiTempState::FileDialog_SaveMjb;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_SaveXml;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_M | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_PrintModel;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_D | ImGuiMod_Ctrl)) {
    tmp_.file_dialog = UiTempState::FileDialog_PrintData;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_P | ImGuiMod_Ctrl)) {
    command_palette_.Toggle();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C | ImGuiMod_Ctrl)) {
    std::string keyframe = platform::KeyframeToString(model(), data(), false);
    platform::MaybeSaveToClipboard(keyframe);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_L | ImGuiMod_Ctrl)) {
    RequestModelReload();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q | ImGuiMod_Ctrl)) {
    tmp_.should_exit = true;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_A | ImGuiMod_Ctrl)) {
    mjv_defaultFreeCamera(model(), &camera_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab)) {
    tmp_.options_panel = !tmp_.options_panel;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Minus | ImGuiMod_Ctrl)) {
    float old_scale = ui_.font_scale;
    ui_.font_scale = std::clamp(ui_.font_scale - 0.1f, 0.5f, 3.0f);
    platform::RescaleDock(ui_.font_scale / old_scale);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Equal | ImGuiMod_Ctrl)) {
    float old_scale = ui_.font_scale;
    ui_.font_scale = std::clamp(ui_.font_scale + 0.1f, 0.5f, 3.0f);
    platform::RescaleDock(ui_.font_scale / old_scale);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Minus)) {
    SetSpeedIndex(tmp_.speed_index + 1);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Equal)) {
    SetSpeedIndex(tmp_.speed_index - 1);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftArrow)) {
    if (step_control_.GetPauseState() == PauseState::kNormalPaused) {
      LoadHistory(sim_history_.GetIndex() - 1);
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_RightArrow)) {
    if (step_control_.GetPauseState() == PauseState::kNormalPaused) {
      if (sim_history_.GetIndex() == 0) {
        step_control_.RequestSingleStep();
      } else {
        LoadHistory(sim_history_.GetIndex() + 1);
      }
    }
  } else if (ImGui_IsChordJustPressed(ImGuiMod_Ctrl | ImGuiKey_Space)) {
    if (step_control_.GetPauseState() == PauseState::kViscousPaused) {
      step_control_.SetPauseState(PauseState::kUnpaused);
    } else {
      step_control_.SetPauseState(PauseState::kViscousPaused);
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Space)) {
    if (step_control_.GetPauseState() == PauseState::kViscousPaused) {
      step_control_.SetPauseState(PauseState::kNormalPaused);
    } else if (step_control_.GetPauseState() == PauseState::kUnpaused) {
      step_control_.SetPauseState(PauseState::kNormalPaused);
    } else {
      step_control_.SetPauseState(PauseState::kUnpaused);
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backspace)) {
    ResetPhysics();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Delete)) {
    // TODO: SpecDeleteElement(tmp_.curr_element);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_PageUp)) {
    SelectParentPerturb(model(), perturb_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F1)) {
    ToggleWindow(tmp_.help);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F2)) {
    ToggleWindow(tmp_.stats);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F3)) {
    ToggleWindow(tmp_.profiler);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F6)) {
    vis_options_.frame = (vis_options_.frame + 1) % mjNFRAME;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F7)) {
    vis_options_.label = (vis_options_.label + 1) % mjNLABEL;
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
  } else if (has_model() && ImGui_IsChordJustPressed(ImGuiKey_Escape)) {
    ui_.camera_idx =
        platform::SetCamera(model(), &camera_, platform::kTumbleCameraIdx);
  } else if (has_model() && ImGui_IsChordJustPressed(ImGuiKey_LeftBracket)) {
    ui_.camera_idx = platform::SetCamera(model(), &camera_, ui_.camera_idx - 1);
  } else if (has_model() && ImGui_IsChordJustPressed(ImGuiKey_RightBracket)) {
    ui_.camera_idx = platform::SetCamera(model(), &camera_, ui_.camera_idx + 1);
    // WASD camera controls for free camera.
  } else if (is_freecam_wasd &&
             (ImGui::IsKeyDown(ImGuiKey_W) || ImGui::IsKeyDown(ImGuiKey_S) ||
              ImGui::IsKeyDown(ImGuiKey_A) || ImGui::IsKeyDown(ImGuiKey_D) ||
              ImGui::IsKeyDown(ImGuiKey_Q) || ImGui::IsKeyDown(ImGuiKey_E))) {
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
  } else {
    platform::ForEachPlugin<platform::KeyHandlerPlugin>([&](auto* plugin) {
      if (plugin->key_chord && plugin->on_key_pressed) {
        if (ImGui_IsChordJustPressed(plugin->key_chord)) {
          plugin->on_key_pressed(plugin);
        }
      }
    });
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
      platform::ForEachPlugin<platform::GuiPlugin>([&](auto* plugin) {
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
    if (window_) {
      ui_.window_width = window_->GetWidth();
      ui_.window_height = window_->GetHeight();
    }
    platform::AppendIniSection(settings, "[Studio][UX]", ui_.ToDict());

    platform::KeyValues plugin_names;
    platform::ForEachPlugin<platform::GuiPlugin>([&](auto* plugin) {
      plugin_names[plugin->name] = std::to_string((int)plugin->active);
    });
    platform::AppendIniSection(settings, "[Studio][Plugins]", plugin_names);

    platform::SaveText(settings, ini_path_);
  }
}

void App::SetSpeedIndex(int idx) {
  platform::SetSpeedIndex(&step_control_, tmp_.speed_index, idx);
}

void App::MoveCamera(platform::CameraMotion motion, mjtNum reldx,
                     mjtNum reldy) {
  platform::MoveCamera(model(), data(), &camera_, motion, reldx, reldy);
}

void App::BuildGui() {
  if (tmp_.full_screen) {
    return;
  }

  if (!tmp_.style_editor) {
    platform::SetupTheme(ui_.theme);
  }

  ImGui::GetIO().FontGlobalScale = ui_.font_scale;

  const ImVec4 workspace_rect = platform::ConfigureDockingLayout();

  MainMenuGui();

  // DCC-style translucent overlays on top of the viewport (transport + view
  // controls along the top, a vertical frame scrubber on the right) plus a
  // menu-bar-styled status bar pinned to the bottom.
  TopOverlayGui(workspace_rect);
  ScrubberOverlayGui(workspace_rect);
  StatusBarGui();

  // Feature documentation: the old "Options" and "Inspector" side panels
  // reserved two wide fixed columns even when their collapsing sections were
  // closed, wasting a lot of horizontal space. They are replaced by a thin
  // Photoshop-style icon rail on the left; each button opens that section as a
  // floating, movable tool window so the viewport stays large and tools appear
  // only on demand. The Explorer (spec tree) and Editor are rail tools too, so
  // no side panels stay docked.
  if (tmp_.options_panel) {
    ToolRailGui(workspace_rect);
    ToolWindowsGui(workspace_rect);
  }

  if (tmp_.profiler) {
    if (ImGui::Begin("Profiler", &tmp_.profiler,
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
      platform::ProfilerGui(model(), data(), &profiler_);
    }
    ImGui::End();
  }

  if (tmp_.picture_in_picture) {
    if (ImGui::Begin("Picture-in-Picture", &tmp_.picture_in_picture)) {
      platform::PipGui(model(), data(), window_->GetAspectRatio(),
                       renderer_.get(), &tmp_.pips);
    }
    ImGui::End();
  }

  if (tmp_.help) {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_Alpha, 0.8f);
    ImGui::SetNextWindowPos(ImVec2(workspace_rect.x, workspace_rect.y),
                            ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Appearing);
    if (ImGui::Begin("Help", &tmp_.help,
                     ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 5.0f));
      HelpGui();
      ImGui::PopStyleVar();
    }
    ImGui::End();
  }

  if (tmp_.stats) {
    // A small floating window. The "###" id keeps the docking layout's
    // name-based docking from capturing it over the rail (same trick as the
    // tool windows); first shown just to the right of the rail. Rendered with
    // plain text rather than the docked StatsGui (whose ImGui::Columns layout
    // doesn't play well in a small floating window).
    ImGui::SetNextWindowPos(
        ImVec2(workspace_rect.x + RailWidth() + 16.0f, workspace_rect.y + 16.0f),
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(200.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Stats###StatsWindow", &tmp_.stats,
                     ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("FPS        %.0f", renderer_->GetFps());
      if (has_data()) {
        const mjModel* m = model();
        const mjData* d = data();
        const bool paused =
            step_control_.GetPauseState() == PauseState::kNormalPaused;
        const auto timer = paused ? mjTIMER_FORWARD : mjTIMER_STEP;
        const double ms =
            d->timer[timer].duration / mjMAX(1, d->timer[timer].number);
        ImGui::Text("Step       %.3f ms", ms);
        ImGui::Text("Sim time   %.3f s", d->time);
        ImGui::Separator();
        ImGui::Text("Bodies     %d", m->nbody);
        ImGui::Text("DOFs       %d", m->nv);
        ImGui::Text("Contacts   %d", d->ncon);
      }
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

  // Ctrl+P command palette, drawn last so it sits on top. Commands are only
  // gathered while it is open. Centered on the screen, 10px below the top
  // transport overlay (so it lines up with that bar).
  ui_agent_.Poll();
  if (command_palette_.is_open()) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const ImVec4 palette_rect(vp->WorkPos.x, tmp_.top_overlay_bottom + 10.0f,
                              vp->WorkSize.x, workspace_rect.w);
    command_palette_.Draw(
        CollectCommands(), palette_rect,
        [this] { llm_panel_.Render(ui_agent_); },
        [this](const std::string& q) { ui_agent_.Ask(q); });
  }

  // Scripted GIF capture: advance the script and draw the synthetic cursor.
  if (capture_.active) {
    CaptureStep();
  }

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

  platform::ForEachPlugin<platform::GuiPlugin>([](auto* plugin) {
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

float App::RailWidth() const {
  const ImGuiStyle& style = ImGui::GetStyle();
  const float button = ImGui::GetFrameHeight() * 1.6f;
  return 2 * button + style.ItemSpacing.x + 2.0f * style.WindowPadding.x;
}

float App::ScrubberWidth() const {
  // A thin vertical strip: one frame wide plus a small padding on each side
  // (kScrubberPad in ScrubberOverlayGui).
  return ImGui::GetFrameHeight() + 8.0f;
}

void App::ToolRailGui(const ImVec4& workspace_rect) {
  const float button = ImGui::GetFrameHeight() * 1.6f;  // square icon button
  constexpr int kColumns = 2;

  // Float the rail vertically centered on the left, auto-sized to just contain
  // its buttons, and inset a little from the left edge (matches the scrubber).
  constexpr float kRailInsetX = 10.0f;
  ImGui::SetNextWindowPos(
      ImVec2(workspace_rect.x + kRailInsetX,
             workspace_rect.y + workspace_rect.w * 0.5f),
      ImGuiCond_Always, ImVec2(0.0f, 0.5f));
  ImGui::SetNextWindowBgAlpha(0.65f);  // translucent, like the other overlays

  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
      ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_AlwaysAutoResize;

  if (ImGui::Begin("ToolRail", nullptr, flags)) {
    // Lay out square icon buttons in a 2-column grid. `active` highlights the
    // button when its window/panel is open; returns true when clicked.
    int slot = 0;
    auto icon_button = [&](const char* icon, const char* title,
                           bool active) -> bool {
      if (slot % kColumns != 0) {
        ImGui::SameLine();
      }
      ++slot;
      if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button,
                              ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
      }
      // Explicit ### id so the test engine can reference the button as
      // "//ToolRail/<title>" (the visible label stays the icon glyph).
      const std::string label = std::string(icon) + "###" + title;
      const bool clicked = ImGui::Button(label.c_str(), ImVec2(button, button));
      // Record the button center for the capture script's cursor targeting.
      const ImVec2 lo = ImGui::GetItemRectMin();
      const ImVec2 hi = ImGui::GetItemRectMax();
      rail_button_center_[title] =
          ImVec2((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f);
      if (active) {
        ImGui::PopStyleColor();
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", title);
      }
      return clicked;
    };

    // Group 0: model actions (reload / reset). These are one-shot buttons, not
    // toggles, so they never show the "active" highlight.
    if (icon_button(ICON_RELOAD_MODEL, "Reload", false)) {
      RequestModelReload();
    }
    if (icon_button(ICON_RESET_MODEL, "Reset", false)) {
      ResetPhysics();
    }
    ImGui::Separator();
    slot = 0;

    // Group 1: registered tool windows (each opens a floating tool window).
    for (ToolWindow& tw : tool_windows_) {
      if (icon_button(tw.icon, tw.title.c_str(), tw.open)) {
        tw.open = !tw.open;
      }
    }

    // Group 2: view / diagnostics windows (toggle the app's own windows).
    ImGui::Separator();
    slot = 0;
    if (icon_button(platform::ICON_FA_TACHOMETER, "Profiler", tmp_.profiler)) {
      ToggleWindow(tmp_.profiler);
    }
    if (icon_button(platform::ICON_FA_BAR_CHART, "Stats", tmp_.stats)) {
      ToggleWindow(tmp_.stats);
    }
    if (icon_button(platform::ICON_FA_CLONE, "Picture-in-Picture",
                    tmp_.picture_in_picture)) {
      tmp_.picture_in_picture = !tmp_.picture_in_picture;
    }
    if (icon_button(platform::ICON_FA_QUESTION_CIRCLE, "Help", tmp_.help)) {
      ToggleWindow(tmp_.help);
    }
  }
  ImGui::End();
}

void App::ToolWindowsGui(const ImVec4& workspace_rect) {
  const float rail_width = RailWidth();

  for (int i = 0; i < static_cast<int>(tool_windows_.size()); ++i) {
    ToolWindow& tw = tool_windows_[i];
    if (!tw.open) {
      continue;
    }
    // Cascade newly opened windows just to the right of the rail.
    const float offset = 24.0f * i;
    ImGui::SetNextWindowPos(
        ImVec2(workspace_rect.x + rail_width + 8.0f + offset,
               workspace_rect.y + 8.0f + offset),
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 360), ImGuiCond_FirstUseEver);
    // Stable, unique ImGui id (via "###") so these windows are not captured by
    // ConfigureDockingLayout's name-based docking; they stay floating.
    char window_id[96];
    std::snprintf(window_id, sizeof(window_id), "%s###ToolWindow%d",
                  tw.title.c_str(), i);
    if (ImGui::Begin(window_id, &tw.open)) {
      tw.render();
    }
    // Record the window rect (x, y, w, h) for the capture script's cursor.
    const ImVec2 wp = ImGui::GetWindowPos();
    const ImVec2 ws = ImGui::GetWindowSize();
    tool_window_rect_[tw.title] = ImVec4(wp.x, wp.y, ws.x, ws.y);
    ImGui::End();
  }
}

void App::RegisterToolWindows() {
  // Wraps a render callback so it shows a placeholder until mjData is available.
  auto needs_data = [this](std::function<void()> fn) -> std::function<void()> {
    return [this, fn = std::move(fn)]() {
      if (!has_data()) {
        ImGui::TextUnformatted("No mjData loaded.");
        return;
      }
      fn();
    };
  };

  // Icons are FontAwesome 4 glyphs chosen to evoke each window's name. Adding a
  // window here is all it takes to get a rail button, a tooltip, and a
  // command-palette entry -- the intended extension point (incl. a future
  // Python API that would register windows whose render() calls back to script).
  tool_windows_.push_back({platform::ICON_FA_COGS, "Physics", [this] {
                             platform::PhysicsGui(
                                 model(), platform::GetExpectedLabelWidth());
                           }});
  tool_windows_.push_back({platform::ICON_FA_CUBE, "Rendering", [this] {
                             platform::RenderingGui(
                                 model(), &vis_options_,
                                 renderer_->GetRenderFlags(),
                                 platform::GetExpectedLabelWidth());
                           }});
  tool_windows_.push_back(
      {platform::ICON_FA_OBJECT_GROUP, "Visibility Groups", [this] {
         platform::GroupsGui(model(), &vis_options_,
                             platform::GetExpectedLabelWidth());
       }});
  tool_windows_.push_back({platform::ICON_FA_EYE, "Visualization", [this] {
                             platform::VisualizationGui(
                                 model(), &vis_options_, &camera_,
                                 platform::GetExpectedLabelWidth());
                           }});
  tool_windows_.push_back(
      {platform::ICON_FA_LINK, "Joints", needs_data([this] {
         platform::JointsGui(model(), data(), &vis_options_);
       })});
  tool_windows_.push_back(
      {platform::ICON_FA_SLIDERS, "Controls", needs_data([this] {
         float noise_scale = 0;
         float noise_rate = 0;
         step_control_.GetNoiseParameters(noise_scale, noise_rate);
         platform::NoiseGui(model(), data(), noise_scale, noise_rate);
         step_control_.SetNoiseParameters(noise_scale, noise_rate);
         ImGui::Separator();
         platform::ControlsGui(model(), data(), &vis_options_);
       })});
  tool_windows_.push_back(
      {platform::ICON_FA_RSS, "Sensor",
       needs_data([this] { platform::SensorGui(model(), data()); })});
  tool_windows_.push_back(
      {platform::ICON_FA_BINOCULARS, "Watch", needs_data([this] {
         platform::WatchGui(model(), data(), ui_.watch_field,
                            sizeof(ui_.watch_field), ui_.watch_index);
       })});
  tool_windows_.push_back(
      {platform::ICON_FA_TABLE, "State", needs_data([this] {
         platform::StateGui(model(), data(), tmp_.state, tmp_.state_sig,
                            platform::GetExpectedLabelWidth());
       })});
  tool_windows_.push_back(
      {platform::ICON_FA_SITEMAP, "Explorer", [this] { SpecExplorerGui(); }});
  tool_windows_.push_back(
      {platform::ICON_FA_PENCIL, "Editor", [this] { SpecEditorGui(); }});
}

std::vector<CommandPalette::Command> App::CollectCommands() {
  std::vector<CommandPalette::Command> commands;

  // Model actions.
  commands.push_back({"Reload", [this] { RequestModelReload(); }});
  commands.push_back({"Reset", [this] { ResetPhysics(); }});

  // Registered tool windows (index is stable for the process lifetime).
  for (int i = 0; i < static_cast<int>(tool_windows_.size()); ++i) {
    commands.push_back({tool_windows_[i].title, [this, i] {
                          tool_windows_[i].open = !tool_windows_[i].open;
                        }});
  }

  // View / diagnostics windows.
  commands.push_back({"Profiler", [this] { ToggleWindow(tmp_.profiler); }});
  commands.push_back({"Stats", [this] { ToggleWindow(tmp_.stats); }});
  commands.push_back({"Picture-in-Picture", [this] {
                        tmp_.picture_in_picture = !tmp_.picture_in_picture;
                      }});
  commands.push_back({"Help", [this] { ToggleWindow(tmp_.help); }});

  return commands;
}

void App::RegisterLlmTools() {
  // The sole actuator: the model drives the real UI through the ImGui Test
  // Engine by emitting a JSON op-program whose refs are ImGui item IDs (see
  // LLM_INTEGRATION_DESIGN.md and test_runner_).
  std::string panels = "Reload, Reset";
  for (const ToolWindow& tw : tool_windows_) panels += ", " + tw.title;
  panels += ", Profiler, Stats, Picture-in-Picture, Help";

  const std::string description =
      "Drive the MuJoCo Studio UI by running a short program of Dear ImGui Test "
      "Engine operations -- this clicks the real on-screen widgets. Use it for "
      "any UI action the user asks for.\n"
      "Reference items by their ImGui path. The left icon rail is the window "
      "'ToolRail'; each of its panel buttons is referenced as "
      "'//ToolRail/###<Panel>' (note the ### prefix on the panel name), where "
      "<Panel> is one of: " +
      panels +
      ". Clicking a panel button toggles that panel. Top menu-bar items use the "
      "'menu_click' op with a path like 'View/Tools'.\n"
      "Example -- open the Physics panel: "
      "{\"ops\":[{\"op\":\"item_click\",\"ref\":\"//ToolRail/###Physics\"}]}";

  ToolDef grep_source{
      "grep_source",
      "Case-insensitive substring search over the Studio C++ source. Use it to "
      "verify that an item id/label/name actually exists before referencing it "
      "in run_ui_program (don't guess refs), or to discover the exact spelling "
      "of a widget label, joint name, menu item, etc. Returns matching "
      "file:line: source lines.",
      "{\"type\":\"object\",\"properties\":{\"pattern\":{\"type\":\"string\","
      "\"description\":\"text to search for\"}},\"required\":[\"pattern\"]}"};

  ToolDef run_program{
      "run_ui_program", description,
      "{\"type\":\"object\",\"properties\":{\"ops\":{\"type\":\"array\","
      "\"items\":{\"type\":\"object\",\"properties\":{"
      "\"op\":{\"type\":\"string\",\"enum\":[\"item_click\",\"menu_click\","
      "\"item_check\",\"item_uncheck\",\"set_float\",\"set_int\",\"key_chars\","
      "\"set_ref\"]},"
      "\"ref\":{\"type\":\"string\",\"description\":\"ImGui item path, e.g. "
      "//ToolRail/###Physics\"},"
      "\"path\":{\"type\":\"string\",\"description\":\"menu path for "
      "menu_click\"},"
      "\"value\":{\"type\":\"number\"},\"text\":{\"type\":\"string\"}},"
      "\"required\":[\"op\"]}}},\"required\":[\"ops\"]}"};

  auto exec = [this](const std::string& name,
                     const std::string& json_args) -> std::string {
    if (name == "grep_source") {
      // Hard per-turn budget so the agent can't get stuck exploring (the system
      // prompt also asks it to grep sparingly).
      if (grep_calls_ >= 6) {
        return "Grep budget reached. Stop searching and emit a run_ui_program "
               "now with what you know (use keyboard shortcuts for anything you "
               "couldn't reference).";
      }
      ++grep_calls_;
      // Extract the "pattern" string argument.
      std::string pattern;
      size_t k = json_args.find("\"pattern\"");
      if (k != std::string::npos) {
        size_t colon = json_args.find(':', k);
        size_t q = (colon == std::string::npos)
                       ? std::string::npos
                       : json_args.find('"', colon + 1);
        size_t e = (q == std::string::npos) ? std::string::npos
                                            : json_args.find('"', q + 1);
        if (e != std::string::npos) pattern = json_args.substr(q + 1, e - q - 1);
      }
      std::fprintf(stderr, "[grep_source] %s\n", pattern.c_str());
      std::fflush(stderr);
      return GrepSource(pattern, 40);
    }
    if (name == "run_ui_program") {
      // Echo the program the LLM generated to the console before running it.
      std::fprintf(stderr, "[run_ui_program] %s\n", json_args.c_str());
      std::fflush(stderr);
      const int n = test_runner_.Run(json_args);
      return "Queued a " + std::to_string(n) + "-op UI program.";
    }
    return "Unknown tool: " + name;
  };

  ui_agent_.set_tools({grep_source, run_program}, exec);
  ui_agent_.set_on_ask([this] { grep_calls_ = 0; });
}

void App::SpecExplorerGui() {
  if (!has_spec()) {
    ImGui::Text("No mjSpec loaded.");
    return;
  }

  // Initialize the split height.
  const ImVec2 region = ImGui::GetContentRegionAvail();
  if (tmp_.explorer_split < 0) {
    tmp_.explorer_split = region.y * 0.7f;
  }
  tmp_.explorer_split = std::clamp(tmp_.explorer_split, 20.f, region.y - 40.f);

  mjsElement* element = tmp_.curr_element;
  bool open = element != nullptr;
  if (platform::ImGui_BeginHSplit("SpecExplorerTree", &tmp_.explorer_split,
                                  &open)) {
    platform::SpecTreeGui(&element, spec());

    if (element != tmp_.curr_element) {
      tmp_.curr_element = element;

      // A different element was selected, so select it for perturb.
      if (element) {
        open = true;
        const int element_id = mjs_getId(element);
        if (element->elemtype == mjOBJ_BODY && perturb_.select != element_id) {
          mjv_defaultPerturb(&perturb_);
          perturb_.select = element_id;
        }
      }
    }

    if (platform::ImGui_HSplit("SpecExplorerProperties", &tmp_.explorer_split,
                               &open)) {
      ImGui::Text("%s", mju_type2Str(tmp_.curr_element->elemtype));
      ImGui::SameLine();
      ImGui::Text("(%d)", mjs_getId(tmp_.curr_element));
      ImGui::SameLine(ImGui::GetContentRegionAvail().x - 100.0f);

      auto mode_button = [&](const char* label, const char* tooltip,
                             SpecPropertiesMode mode) {
        platform::ScopedStyle style;
        style.Var(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
        if (tmp_.spec_prop_mode == mode) {
          style.Color(ImGuiCol_Button, ImGuiCol_ButtonActive);
        }
        if (ImGui::Button(label, ImVec2(24.0f, 20.0f))) {
          tmp_.spec_prop_mode = mode;
        }
        ImGui::SetItemTooltip("%s", tooltip);
      };

      mode_button("S", "Spec", SpecPropertiesMode::kSpec);
      ImGui::SameLine();
      mode_button("M", "Model", SpecPropertiesMode::kModel);
      ImGui::SameLine();
      mode_button("D", "Data", SpecPropertiesMode::kData);

      ImGui::Separator();

      if (tmp_.spec_prop_mode == SpecPropertiesMode::kSpec) {
        platform::ElementSpecGui(element);
      } else if (tmp_.spec_prop_mode == SpecPropertiesMode::kModel) {
        platform::ElementModelGui(model(), tmp_.curr_element);
      } else {
        platform::ElementDataGui(data(), tmp_.curr_element);
      }
    }
    if (!open) {
      tmp_.curr_element = nullptr;
    }
  }
  platform::ImGui_EndHSplit(open);
}

void App::SpecEditorGui() {
  if (ImGui::BeginChild("SpecEditor", ImVec2(-1, 36))) {
    if (ImGui::BeginTable("##SpecEditorHeader", 3)) {
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 50.0f);
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 30.0f);

      ImGui::TableNextColumn();
      ImGui::BeginDisabled(!spec_editor_.CanUndo());
      if (ImGui::Button(ICON_UNDO_SPEC)) {
        spec_editor_.Undo();
      }
      ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::BeginDisabled(!spec_editor_.CanRedo());
      if (ImGui::Button(ICON_REDO_SPEC)) {
        spec_editor_.Redo();
      }
      ImGui::EndDisabled();

      ImGui::TableNextColumn();
      ImGui::PushStyleColor(ImGuiCol_Button, ImColor(40, 180, 40, 255).Value);
      if (ImGui::Button("Compile and Reload", ImVec2(-1, 0))) {
        pending_op_ = [this]() {
          auto tmp_holder = spec_editor_.Compile();
          if (tmp_holder->ok()) {
            model_holder_ = std::move(tmp_holder);
            OnModelLoaded(model_name_, model_kind_);
          } else {
            load_error_ = std::move(tmp_holder->error());
          }
        };
      }
      ImGui::PopStyleColor();

      ImGui::TableNextColumn();
      if (ImGui::Button(" + ")) {
        ImGui::OpenPopupOnItemClick("SpecAddElement", 0);
      }
      if (ImGui::BeginPopupContextItem("SpecAddElement")) {
        auto option = [&](const char* label, mjtObj type) {
          if (ImGui::Selectable(label)) {
            mjsElement* element = spec_editor_.AddElement(type);
            spec_editor_.SetActiveElement(element);
          }
        };

        option("Actuator", mjOBJ_ACTUATOR);
        option("Equality", mjOBJ_EQUALITY);
        option("Exclude", mjOBJ_EXCLUDE);
        option("Flex", mjOBJ_FLEX);
        option("Height Field", mjOBJ_HFIELD);
        option("Key", mjOBJ_KEY);
        option("Material", mjOBJ_MATERIAL);
        option("Mesh", mjOBJ_MESH);
        option("Numeric", mjOBJ_NUMERIC);
        option("Pair", mjOBJ_PAIR);
        option("Sensor", mjOBJ_SENSOR);
        option("Skin", mjOBJ_SKIN);
        option("Tendon", mjOBJ_TENDON);
        option("Text", mjOBJ_TEXT);
        option("Texture", mjOBJ_TEXTURE);
        option("Tuple", mjOBJ_TUPLE);
        ImGui::EndPopup();
      }

      ImGui::EndTable();
    }
  }
  ImGui::EndChild();

  // Initialize the split height.
  const ImVec2 region = ImGui::GetContentRegionAvail();
  if (tmp_.editor_split < 0) {
    tmp_.editor_split = region.y * 0.7f;
  }
  tmp_.editor_split = std::clamp(tmp_.editor_split, 20.0f, region.y - 40.0f);

  mjsElement* element = spec_editor_.GetActiveElement();

  bool open = element != nullptr;
  if (platform::ImGui_BeginHSplit("SpecEditorTree", &tmp_.editor_split,
                                  &open)) {
    platform::SpecTreeGui(&element, spec_editor_.GetActiveSpec(),
                          &spec_editor_);
    open = element != nullptr;
    spec_editor_.SetActiveElement(element);

    if (platform::ImGui_HSplit("SpecEditorProperties", &tmp_.editor_split,
                               &open)) {
      ImGui::Text("%s", mju_type2Str(element->elemtype));
      ImGui::SameLine();
      ImGui::Text("(%d)", mjs_getId(element));
      ImGui::Separator();

      platform::ElementSpecGui(element, &spec_editor_);
    }
    platform::ImGui_EndHSplit(open);
    if (!open) {
      spec_editor_.SetActiveElement(nullptr);
    }
  }
}

void App::HelpGui() {
  const float pad = ImGui::GetStyle().ItemSpacing.x;
  const float indent = pad * 2;
  const float col0 = ImGui::CalcTextSize("Toggle Visc Pause").x + pad;
  const float col1 = ImGui::CalcTextSize("Ctrl+Spc").x + pad + indent;
  const float col2 = ImGui::CalcTextSize("Center of Mass").x + pad + indent;
  const float col3 = ImGui::CalcTextSize("M").x + pad + indent;
  ImGui::Dummy(ImVec2(col0 + col1 + col2 + col3, 0));
  ImGui::Columns(4);
  ImGui::SetColumnWidth(0, col0);
  ImGui::SetColumnWidth(1, col1);
  ImGui::SetColumnWidth(2, col2);
  ImGui::SetColumnWidth(3, col3);

  ImGui::Text("Help");
  ImGui::Text("Stats");
  ImGui::Text("Profiler");
  ImGui::Text("Cycle Frames");
  ImGui::Text("Cycle Labels");
  ImGui::Text("Toggle Fullscreen");
  ImGui::Text("Free Camera");
  ImGui::Text("Toggle Pause");
  ImGui::Text("Toggle Visc Pause");
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
  ImGui::Indent(indent);
  ImGui::Text("F1");
  ImGui::Text("F2");
  ImGui::Text("F3");
  ImGui::Text("F6");
  ImGui::Text("F7");
  ImGui::Text("F11");
  ImGui::Text("Esc");
  ImGui::Text("Spc");
  ImGui::Text("Ctrl+Spc");
  ImGui::Text("Bksp");
  ImGui::Text("Tab");
  ImGui::Text("Sh+Tab");
  ImGui::Text("+");
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
  ImGui::Indent(indent);
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
  ImGui::Indent(indent);
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
  // Transport only: combined (Normal Pause, Viscous Pause, Play) widget + speed.
  // The view-display selectors (camera/label/frame) live on the menu bar, and
  // engine settings (threadpool size) live in Edit > Preferences.
  platform::StepControlGui(model(), &step_control_, tmp_.speed_index);
}

namespace {
// Shared flags for the translucent viewport overlays.
constexpr ImGuiWindowFlags kOverlayFlags =
    ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
    ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNavFocus;
constexpr float kOverlayAlpha = 0.65f;
}  // namespace

void App::TopOverlayGui(const ImVec4& workspace_rect) {
  const float margin = ImGui::GetStyle().ItemSpacing.x;
  // Auto-size to the packed controls and center horizontally on the actual
  // screen center (the rail and scrubber are thin overlays that don't sit at the
  // top center, so we don't bias around them). The (0.5, 0) pivot keeps it
  // centered regardless of its content width.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float center_x = vp->WorkPos.x + vp->WorkSize.x * 0.5f;
  ImGui::SetNextWindowPos(ImVec2(center_x, workspace_rect.y + margin),
                          ImGuiCond_Always, ImVec2(0.5f, 0.0f));
  ImGui::SetNextWindowBgAlpha(kOverlayAlpha);
  if (ImGui::Begin("##TopOverlay", nullptr,
                   kOverlayFlags | ImGuiWindowFlags_AlwaysAutoResize)) {
    ToolBarGui();
  }
  // Remember the bar's bottom edge so the command palette can sit just below it.
  tmp_.top_overlay_bottom = ImGui::GetWindowPos().y + ImGui::GetWindowSize().y;
  ImGui::End();
}

void App::ScrubberOverlayGui(const ImVec4& workspace_rect) {
  if (!has_model()) {
    return;
  }
  const ImGuiStyle& style = ImGui::GetStyle();
  const float scrub_w = ScrubberWidth();
  const float h = std::max(160.0f, workspace_rect.w * 0.6f);
  // Inset 10px from the right edge, mirroring the rail's 10px left inset.
  constexpr float kScrubberInsetX = 10.0f;
  const float x = workspace_rect.x + workspace_rect.z - scrub_w - kScrubberInsetX;
  const float y = workspace_rect.y + (workspace_rect.w - h) * 0.5f;

  ImGui::SetNextWindowPos(ImVec2(x, y));
  ImGui::SetNextWindowSize(ImVec2(scrub_w, h));
  ImGui::SetNextWindowBgAlpha(kOverlayAlpha);
  // Tight padding so the thin strip's content fills its width.
  constexpr float kScrubberPad = 4.0f;
  platform::ScopedStyle pad;
  pad.Var(ImGuiStyleVar_WindowPadding, ImVec2(kScrubberPad, kScrubberPad));
  if (ImGui::Begin("##Scrubber", nullptr, kOverlayFlags)) {
    const float col_w = ImGui::GetContentRegionAvail().x;
    const float step = ImGui::GetFrameHeight() + style.ItemSpacing.y;

    // Next frame (toward the current state) at the top.
    if (ImGui::Button(platform::ICON_FA_CARET_UP, ImVec2(col_w, 0))) {
      if (sim_history_.GetIndex() == 0) {
        step_control_.RequestSingleStep();
      } else {
        LoadHistory(sim_history_.GetIndex() + 1);
      }
    }
    ImGui::SetItemTooltip("%s", "Next frame");

    // Vertical history slider fills the middle (top = current, down = older).
    int index = sim_history_.GetIndex();
    const float slider_h =
        std::max(40.0f, ImGui::GetContentRegionAvail().y - 2.0f * step);
    if (ImGui::VSliderInt("##ScrubIndex", ImVec2(col_w, slider_h), &index,
                          1 - sim_history_.Size(), 0, "")) {
      LoadHistory(index);
    }
    ImGui::SetItemTooltip("Frame %d of %d", index, sim_history_.Size());

    // Previous frame (toward older states) below the slider.
    if (ImGui::Button(platform::ICON_FA_CARET_DOWN, ImVec2(col_w, 0))) {
      LoadHistory(sim_history_.GetIndex() - 1);
    }
    ImGui::SetItemTooltip("%s", "Previous frame");

    // Jump to the current (latest) frame.
    if (ImGui::Button(ICON_CURR_FRAME, ImVec2(col_w, 0))) {
      LoadHistory(0);
    }
    ImGui::SetItemTooltip("%s", "Current frame");
  }
  ImGui::End();
}

void App::StatusBarGui() {
  // Left: run state + any error message. Right: model/sim metrics.
  std::string left;
  if (!has_model()) {
    left = "No model loaded";
  } else if (step_control_.GetPauseState() == PauseState::kViscousPaused) {
    left = "Viscous Pause";
  } else if (step_control_.GetPauseState() == PauseState::kNormalPaused) {
    left = "Paused";
  } else {
    left = "Running";
  }
  if (!step_error_.empty()) {
    left += "   |   Step Error: " + step_error_;
  } else if (!load_error_.empty()) {
    left += "   |   Load Error: " + load_error_;
  } else if (!edit_error_.empty()) {
    left += "   |   Edit Error: " + edit_error_;
  }

  std::string right;
  if (has_model() && has_data()) {
    const mjModel* m = model();
    const mjData* d = data();
    const int step_num = d->timer[mjTIMER_STEP].number;
    const double ms_per_step =
        step_num > 0 ? d->timer[mjTIMER_STEP].duration / step_num : 0.0;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "bodies %d   dof %d   contacts %d      "
                  "t %.3f s   frame %d      "
                  "%.2f ms/step      %.0f fps",
                  m->nbody, m->nv, d->ncon, d->time, sim_history_.GetIndex(),
                  ms_per_step, renderer_->GetFps());
    right = buf;
  }

  // A full-width bar pinned to the bottom of the viewport, styled like the top
  // menu bar (same background colour, square corners). ConfigureDockingLayout
  // reserves the matching strip so it does not overlap the dockspace.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float h = ImGui::GetFrameHeight();
  const ImGuiStyle& s = ImGui::GetStyle();
  ImGui::SetNextWindowPos(
      ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - h));
  ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, h));

  platform::ScopedStyle style;
  style.Var(ImGuiStyleVar_WindowRounding, 0.0f);
  style.Var(ImGuiStyleVar_WindowBorderSize, 0.0f);
  style.Var(ImGuiStyleVar_WindowPadding,
            ImVec2(s.ItemSpacing.x, (h - ImGui::GetTextLineHeight()) * 0.5f));
  style.Color(ImGuiCol_WindowBg, ImGui::GetStyleColorVec4(ImGuiCol_MenuBarBg));

  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus;
  if (ImGui::Begin("##StatusBar", nullptr, flags)) {
    // Status message on the left.
    ImGui::TextUnformatted(left.c_str());
    // Metrics right-aligned on the same line.
    if (!right.empty()) {
      const float right_w = ImGui::CalcTextSize(right.c_str()).x;
      ImGui::SameLine();
      ImGui::SetCursorPosX(std::max(
          ImGui::GetCursorPosX(),
          ImGui::GetWindowWidth() - right_w - 2.0f * s.ItemSpacing.x));
      ImGui::TextUnformatted(right.c_str());
    }
  }
  ImGui::End();
}

void App::GraphicsModeMenu() {
#ifdef __linux__
  if (ImGui::BeginMenu("Graphics Mode (Experimental)")) {
    std::optional<platform::GraphicsMode> mode;
    if (ImGui::MenuItem("Classic OpenGL", nullptr,
                        gfx_mode_ == platform::GraphicsMode::ClassicOpenGl)) {
      mode = platform::GraphicsMode::ClassicOpenGl;
    }
    if (ImGui::MenuItem(
            "Classic OpenGL Headless", nullptr,
            gfx_mode_ == platform::GraphicsMode::ClassicOpenGlHeadless)) {
      mode = platform::GraphicsMode::ClassicOpenGlHeadless;
    }
    if (ImGui::MenuItem("Filament OpenGL", nullptr,
                        gfx_mode_ == platform::GraphicsMode::FilamentOpenGl)) {
      mode = platform::GraphicsMode::FilamentOpenGl;
    }
    if (ImGui::MenuItem(
            "Filament OpenGL Headless", nullptr,
            gfx_mode_ == platform::GraphicsMode::FilamentOpenGlHeadless)) {
      mode = platform::GraphicsMode::FilamentOpenGlHeadless;
    }
    if (ImGui::MenuItem(
            "Filament OpenGL Software", nullptr,
            gfx_mode_ == platform::GraphicsMode::FilamentOpenGlSoftware)) {
      mode = platform::GraphicsMode::FilamentOpenGlSoftware;
    }
    if (ImGui::MenuItem("Filament Vulkan", nullptr,
                        gfx_mode_ == platform::GraphicsMode::FilamentVulkan)) {
      mode = platform::GraphicsMode::FilamentVulkan;
    }
    if (ImGui::MenuItem(
            "Filament Vulkan Software", nullptr,
            gfx_mode_ == platform::GraphicsMode::FilamentVulkanSoftware)) {
      mode = platform::GraphicsMode::FilamentVulkanSoftware;
    }
    if (mode.has_value()) {
      pending_op_ = [=, this]() {
        const int width = window_->GetWidth();
        const int height = window_->GetHeight();
        SwitchGraphicsMode(width, height, *mode);
        // TODO: figure out why ImGui doesn't work unless we do this twice.
        if (IsClassic(*mode)) {
          SwitchGraphicsMode(width, height, *mode);
        }
        renderer_->Init(model());
      };
    }
    ImGui::EndMenu();
  }
#endif  // __linux__
}

void App::MainMenuGui() {
  if (ImGui::BeginMainMenuBar()) {
    // MuJoCo logo in a square to the left of the File menu. Uploaded lazily
    // (the renderer must exist) and kept square at the menu-bar height.
    if (logo_texture_ == 0) {
      logo_texture_ = renderer_->UploadImage(
          0, reinterpret_cast<const std::byte*>(kMujocoLogoRgba),
          kMujocoLogoWidth, kMujocoLogoHeight, 4);
    }
    const float logo_size = ImGui::GetFrameHeight();
    ImGui::Image(logo_texture_, ImVec2(logo_size, logo_size));
    ImGui::SameLine();

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
      if (ImGui::MenuItem("Save Screenshot")) {
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

    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::BeginMenu("Preferences")) {
        ImGui::TextUnformatted("Threadpool size");
        ImGui::SetNextItemWidth(180);
        ImGui::BeginDisabled(std::thread::hardware_concurrency() <= 1);
        if (ImGui::SliderInt("##NumThreads", &ui_.nthread, 0, 8, "%d threads")) {
          tmp_.update_threadpool = true;
        }
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("%s",
                              "Number of worker threads used to step the model");

        GraphicsModeMenu();

        ImGui::Separator();
        if (ImGui::MenuItem("Save Config")) {
          SaveSettings();
        }
        if (ImGui::MenuItem("Reset Config")) {
          platform::SaveText("\n\n", ini_path_);
          LoadSettings();
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Simulation")) {
      if (ImGui::MenuItem(
              "Pause", "Space",
              step_control_.GetPauseState() == PauseState::kNormalPaused)) {
        if (step_control_.GetPauseState() != PauseState::kNormalPaused) {
          step_control_.SetPauseState(PauseState::kNormalPaused);
        } else {
          step_control_.SetPauseState(PauseState::kUnpaused);
        }
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
      if (ImGui::MenuItem(tmp_.options_panel ? "Hide Tools" : "Show Tools",
                          "Tab")) {
        tmp_.options_panel = !tmp_.options_panel;
      }
      if (ImGui::MenuItem("Full Screen", "F11")) {
        tmp_.full_screen = !tmp_.full_screen;
      }
      ImGui::Separator();

      // Diagnostics / secondary windows.
      if (ImGui::MenuItem("Profiler", "F3", tmp_.profiler)) {
        ToggleWindow(tmp_.profiler);
      }
      if (ImGui::MenuItem("Stats", "F2", tmp_.stats)) {
        ToggleWindow(tmp_.stats);
      }
      if (ImGui::MenuItem("Picture-in-Picture", nullptr,
                          tmp_.picture_in_picture)) {
        tmp_.picture_in_picture = !tmp_.picture_in_picture;
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
      ImGui::Separator();
      if (ImGui::BeginMenu("Developer")) {
        if (ImGui::MenuItem("Style Editor", nullptr, tmp_.style_editor)) {
          tmp_.style_editor = !tmp_.style_editor;
        }
        if (ImGui::MenuItem("ImGui Demo", nullptr, tmp_.imgui_demo)) {
          tmp_.imgui_demo = !tmp_.imgui_demo;
        }
        if (ImGui::MenuItem("ImPlot Demo", nullptr, tmp_.implot_demo)) {
          tmp_.implot_demo = !tmp_.implot_demo;
        }
        ImGui::EndMenu();
      }
      ImGui::Separator();
      std::string version = "Version " + std::string(mj_versionString());
      ImGui::MenuItem(version.c_str());
      ImGui::EndMenu();
    }

    // Right-aligned cluster of view-display selectors followed by the theme
    // selector. CameraSelectionGui is [copy button + combo]; the Label/Frame
    // selectors are a single combo each (all combos use GetExpectedLabelWidth);
    // the theme button is one frame wide. Pre-compute the cluster width so it
    // hugs the right edge of the menu bar.
    const ImGuiStyle& menu_style = ImGui::GetStyle();
    const float frame_h = ImGui::GetFrameHeight();
    const float label_w = platform::GetExpectedLabelWidth();
    const float cluster_w =
        2.0f * frame_h + 3.0f * label_w + 3.0f * menu_style.ItemSpacing.x;
    ImGui::SameLine(ImGui::GetWindowWidth() - cluster_w -
                    menu_style.FramePadding.x * 2.0f);

    platform::CameraSelectionGui(model(), data(), camera_, ui_.camera_idx);
    ImGui::SameLine();
    platform::LabelSelectionGui(&vis_options_);
    ImGui::SameLine();
    platform::FrameSelectionGui(&vis_options_);
    ImGui::SameLine();
    // Theme selector: cycles Light -> Dark -> Classic and shows its icon.
    if (platform::ThemeSelectGui(&ui_.theme, ImVec2(frame_h, 0))) {
      platform::SetupTheme(ui_.theme);
      ImGui::GetIO().WantSaveIniSettings = true;
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

App::UiState::Dict App::UiState::ToDict() const {
  return {
      {"theme", std::to_string(static_cast<int>(theme))},
      {"font_scale", std::to_string(font_scale)},
      {"window_width", std::to_string(window_width)},
      {"window_height", std::to_string(window_height)},
      {"nthread", std::to_string(nthread)},
  };
}

void App::UiState::FromDict(const Dict& dict) {
  using platform::ReadIniValue;

  *this = UiState();
  theme = ReadIniValue(dict, "theme", theme);
  window_width = ReadIniValue(dict, "window_width", window_width);
  window_height = ReadIniValue(dict, "window_height", window_height);
  font_scale = ReadIniValue(dict, "font_scale", font_scale);
  nthread = ReadIniValue(dict, "nthread", nthread);
}
}  // namespace mujoco::studio
