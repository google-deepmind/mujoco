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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include "webp/encode.h"
#include "webp/types.h"
#include <mujoco/mujoco.h>
#include "experimental/platform/hal/classic_renderer.h"
#include "experimental/platform/hal/filament_renderer.h"
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

static std::string CheckPathForFile(const std::filesystem::path& path,
                                    const std::string& filename) {
  std::filesystem::path resolved = path / filename;
  if (std::filesystem::exists(resolved)) {
    return resolved.string();
  }
  resolved += ".xml";
  if (std::filesystem::exists(resolved)) {
    return resolved.string();
  }
  return "";
}

// Attempts to find a file with the given name by recursively searching the
// given search paths.
static std::string ResolveFile(const std::string& filename,
                               const std::vector<std::string>& search_paths) {
  if (std::filesystem::exists(filename)) {
    return filename;
  }

  std::string resolved;
  for (const std::string& path : search_paths) {
    if (!std::filesystem::exists(path) ||
        !std::filesystem::is_directory(path)) {
      continue;
    }

    resolved = CheckPathForFile(std::filesystem::path(path), filename);
    if (!resolved.empty()) {
      return resolved;
    }

    std::vector<std::filesystem::path> entries;
    for (const auto& it : std::filesystem::recursive_directory_iterator(path)) {
      entries.push_back(it.path());
    }
    std::sort(entries.begin(), entries.end());
    for (const auto& entry : entries) {
      resolved = CheckPathForFile(entry, filename);
      if (!resolved.empty()) {
        return resolved;
      }
    }
  }
  return filename;
}

// Exports the given image (assumed to be RGB888) to a webp file.
static void SaveToWebp(int width, int height, const std::byte* data,
                       const std::string& filename) {
  uint8_t* webp = nullptr;
  const size_t size = WebPEncodeLosslessRGB(
      reinterpret_cast<const uint8_t*>(data), width, height, width * 3, &webp);
  std::ofstream file(filename, std::ios::binary);
  file.write(reinterpret_cast<const char*>(webp), size);
  file.close();
  WebPFree(webp);
}

static constexpr const char* ICON_RELOAD_MODEL = platform::ICON_FA_REFRESH;
static constexpr const char* ICON_RESET_MODEL = platform::ICON_FA_UNDO;
static constexpr const char* ICON_UNDO_SPEC = platform::ICON_FA_UNDO;
static constexpr const char* ICON_REDO_SPEC = platform::ICON_FA_REPEAT;

App::App(Config config)
    : app_title_(std::move(config.title)),
      ini_path_(std::move(config.ini_path)),
      gfx_mode_(config.gfx_mode) {
  SwitchGraphicsMode(config.width, config.height, config.gfx_mode);

  if (config.initial_theme.has_value()) {
    ui_.theme = *config.initial_theme;
  }

  ImPlot::CreateContext();
  mjv_defaultPerturb(&perturb_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&vis_options_);
  std::memset(&plugin_scene_, 0, sizeof(mjvScene));
  mjv_makeScene(nullptr, &plugin_scene_, 2000);

  profiler_.Clear();

  step_control_.SetPreStepCallback(
      [this](const mjModel* m, mjData* d) { PreStep(m, d); });
  step_control_.SetPostStepCallback(
      [this](const mjModel* m, mjData* d) { PostStep(m, d); });
}

App::~App() {
  // ImGui's autosave is on a timer and covers only state it tracks, so recent
  // changes and plugin visibility would be lost. window_ owns the ImGui context
  // and outlives this body.
  SaveSettings();
  mjv_freeScene(&plugin_scene_);
}

void App::SwitchGraphicsMode(int width, int height,
                             platform::GraphicsMode mode) {
  renderer_.reset();
  window_.reset();
  gfx_mode_ = mode;

  platform::Window::Config window_config;
  window_config.gfx_mode = gfx_mode_;
  window_ = std::make_unique<platform::Window>(app_title_, width, height,
                                               window_config);
  if (platform::IsClassic(gfx_mode_)) {
    renderer_ = std::make_unique<platform::ClassicRenderer>(
        window_->GetNativeWindowHandle(), gfx_mode_);

  } else {
    renderer_ = std::make_unique<platform::FilamentRenderer>(
        window_->GetNativeWindowHandle(), gfx_mode_);
  }

  // TODO: Figure out why this breaks on some platforms.
  // LoadSettings();
  if (ui_.window_width > 0 && ui_.window_height > 0) {
    window_->Resize(ui_.window_width, ui_.window_height);
  }
}

void App::Recompile() {
  mj_recompile(model_holder_->spec(), model_holder_->vfs(),
               model_holder_->model(), model_holder_->data());
  const int state_size = mj_stateSize(model(), mjSTATE_INTEGRATION);
  sim_history_.Init(state_size);
  if (has_model() && has_data()) {
    std::span<mjtNum> state = sim_history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model(), data(), state.data(), mjSTATE_INTEGRATION);
    }
  }
  timeline_.sim_head_time = has_data() ? data()->time : 0.0;
  timeline_.lh_width = 0.0f;
  timeline_.rh_width = 0.0f;
  timeline_.scrubber_active = false;
  timeline_.scrubber_grab_offset = 0.0f;
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
  const std::string resolved_file = ResolveFile(filepath, search_paths_);
  model_holder_ = platform::ModelHolder::FromFile(resolved_file);
  if (model_holder_->ok()) {
    OnModelLoaded(filepath, kModelFromFile);
    if (spec()) {
      spec_editor_.Reset(*spec());
    } else {
      spec_editor_.Reset();
    }
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
  if (spec()) {
    spec_editor_.Reset(*spec());
  } else {
    spec_editor_.Reset();
  }
}

void App::OnModelLoaded(std::string filename, ModelKind model_kind) {
  load_error_ = "";
  step_error_ = "";
  edit_error_ = "";

  if (!model_holder_->warning().empty()) {
    load_error_ = model_holder_->warning();
  }
  model_path_ = std::move(filename);

  if (model_kind_ == kEmptyModel) {
    step_control_.SetPauseState(PauseState::kUnpaused);
  }
  model_kind_ = model_kind;
  if (model_kind_ == kEmptyModel || !load_error_.empty()) {
    step_control_.SetPauseState(PauseState::kNormalPaused);
  }

  // Reset/reinitialize everything that depends on the new mjModel.
  mjModel* model = model_holder_->model();
  renderer_->Init(model);
  const int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  sim_history_.Init(state_size);
  if (has_model() && has_data()) {
    std::span<mjtNum> state = sim_history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model, data(), state.data(), mjSTATE_INTEGRATION);
    }
  }
  timeline_.sim_head_time = has_data() ? data()->time : 0.0;
  timeline_.lh_width = 0.0f;
  timeline_.rh_width = 0.0f;
  timeline_.scrubber_active = false;
  timeline_.scrubber_grab_offset = 0.0f;

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
      plugin->post_model_loaded(plugin, model, model_path_.c_str());
    }
  });
  tmp_.update_threadpool = true;
  last_pause_state_ = step_control_.GetPauseState();
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
  if (has_model()) {
    const int state_size = mj_stateSize(model(), mjSTATE_INTEGRATION);
    sim_history_.Init(state_size);
  }
  if (has_model() && has_data()) {
    std::span<mjtNum> state = sim_history_.AddToHistory();
    if (!state.empty()) {
      mj_getState(model(), data(), state.data(), mjSTATE_INTEGRATION);
    }
  }
  timeline_.sim_head_time = has_data() ? data()->time : 0.0;
  timeline_.lh_width = 0.0f;
  timeline_.rh_width = 0.0f;
  timeline_.scrubber_active = false;
  timeline_.scrubber_grab_offset = 0.0f;
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

  bool plugin_stepped = false;
  platform::ForEachPlugin<platform::ModelPlugin>([&](auto* plugin) {
    if (plugin->do_update) {
      if (plugin->do_update(plugin, model(), data())) {
        plugin_stepped = true;
      }
    }
  });

  bool stepped = plugin_stepped;
  if (!plugin_stepped) {
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
      profiler_.Update(model(), data());
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
    if (plugin_stepped) {
      std::span<mjtNum> state = sim_history_.AddToHistory();
      if (!state.empty()) {
        mj_getState(model(), data(), state.data(), mjSTATE_INTEGRATION);
        timeline_.sim_head_time = data()->time;
      }
    }
  }
}

void App::PreStep(const mjModel* m, mjData* d) {
  platform::ForEachPlugin<platform::ModelPlugin>([&](auto* plugin) {
    if (plugin->pre_step) {
      plugin->pre_step(plugin, m, d);
    }
  });
}

void App::PostStep(const mjModel* m, mjData* d) {
  platform::ForEachPlugin<platform::ModelPlugin>([&](auto* plugin) {
    if (plugin->post_step) {
      plugin->post_step(plugin, m, d);
    }
  });

  std::span<mjtNum> state = sim_history_.AddToHistory();
  if (!state.empty()) {
    mj_getState(m, d, state.data(), mjSTATE_INTEGRATION);
    timeline_.sim_head_time = d->time;
  }
}

void App::LoadHistory(int offset) {
  platform::LoadHistoryFrame(sim_history_, step_control_, model(), data(),
                             offset);
}

bool App::Update() {
  // Must precede the first NewFrame: the dockspace is built lazily on the frame
  // that finds no root node, so the saved nodes have to be there already or the
  // default layout wins.
  if (tmp_.first_frame) {
    LoadSettings();
    tmp_.first_frame = false;
  }

  const platform::Window::Status status = window_->NewFrame();

  HandleWindowEvents();
  HandleMouseEvents();
  HandleKeyboardEvents();

  ProcessPendingLoads();

  PauseState current_pause = step_control_.GetPauseState();
  if (current_pause != last_pause_state_) {
    load_error_ = "";
    last_pause_state_ = current_pause;
  }

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

  plugin_scene_.ngeom = 0;
  platform::ForEachPlugin<platform::ScenePlugin>([&](auto* plugin) {
    if (plugin->enhance_scene) {
      plugin->enhance_scene(plugin, model(), data(), &plugin_scene_);
    }
  });

  renderer_->Render(model(), data(), &perturb_, &camera_, &vis_options_,
                    width * scale, height * scale, pixels_,
                    {plugin_scene_.geoms, (size_t)plugin_scene_.ngeom});

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
      if (buf && buf == model_name) {
        LoadModelFromFile(model_name);
      } else if (buf && size) {
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
        mjv_moveCamera(model(), mjMOUSE_TURN_H, mouse_dx, 0.f, &camera_);
        mjv_moveCamera(model(), mjMOUSE_TURN_V, 0.f, mouse_dy, &camera_);
      }
    } else {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        mjv_moveCamera(model(), mjMOUSE_ROTATE_H, mouse_dx, 0.f, &camera_);
        mjv_moveCamera(model(), mjMOUSE_ROTATE_V, 0.f, mouse_dy, &camera_);
      } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        mjv_moveCamera(model(), mjMOUSE_ZOOM, 0.f, mouse_dy, &camera_);
      }
    }

    // Right mouse movement is relative to the horizontal and vertical planes.
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right) && io.KeyShift) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_H, mouse_dx, mouse_dy, &camera_);
    } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_V, mouse_dx, mouse_dy, &camera_);
    }
  }

  // Mouse scroll zooms the camera towards/away from the lookat point.
  // Ignored by user-centered free cameras which don't have a lookat point.
  if (mouse_scroll != 0.0f && ui_.camera_idx != platform::kFreeCameraIdx) {
    mjv_moveCamera(model(), mjMOUSE_ZOOM, 0.f, -mouse_scroll, &camera_);
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
    tmp_.file_dialog = UiTempState::FileDialog_SaveScreenshot;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C | ImGuiMod_Ctrl)) {
    std::string keyframe = platform::KeyframeToString(model(), data(), false);
    platform::MaybeSaveToClipboard(keyframe);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_L | ImGuiMod_Ctrl)) {
    RequestModelReload();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q | ImGuiMod_Ctrl)) {
    tmp_.should_exit = true;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_A | ImGuiMod_Ctrl)) {
    const int cam_id = model()->vis.global.cameraid;
    if (cam_id >= 0 && cam_id < model()->ncam) {
      ui_.camera_idx = platform::SetCamera(model(), &camera_, cam_id);
    } else {
      mjv_defaultFreeCamera(model(), &camera_);
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab | ImGuiMod_Shift)) {
    tmp_.inspector_panel = !tmp_.inspector_panel;
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
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Space)) {
    if (step_control_.GetPauseState() == PauseState::kUnpaused) {
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
    ToggleWindow(tmp_.info);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F3)) {
    // Cycle through profiler views.
    if (!tmp_.profiler) {
      tmp_.profiler = true;
      tmp_.profiler_show_iter = false;
    } else if (!tmp_.profiler_show_iter) {
      tmp_.profiler_show_iter = true;
    } else {
      tmp_.profiler = false;
    }
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
  } else if (!is_freecam_wasd && ImGui_IsChordJustPressed(ImGuiKey_W)) {
    ToggleFlag(renderer_->GetRenderFlags()[mjRND_WIREFRAME]);
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
      mjv_moveCamera(model(), mjMOUSE_MOVE_H_REL, 0, tmp_.cam_speed, &camera_);
      moved = true;
    } else if (ImGui::IsKeyDown(ImGuiKey_S)) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_H_REL, 0, -tmp_.cam_speed, &camera_);
      moved = true;
    }

    // Strafe (truck) left/right using A and D keys.
    if (ImGui::IsKeyDown(ImGuiKey_A)) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_H_REL, -tmp_.cam_speed, 0, &camera_);
      moved = true;
    } else if (ImGui::IsKeyDown(ImGuiKey_D)) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_H_REL, tmp_.cam_speed, 0, &camera_);
      moved = true;
    }

    // Move (pedestal) up/down using Q and E keys.
    if (ImGui::IsKeyDown(ImGuiKey_Q)) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_V_REL, 0, tmp_.cam_speed, &camera_);
      moved = true;
    } else if (ImGui::IsKeyDown(ImGuiKey_E)) {
      mjv_moveCamera(model(), mjMOUSE_MOVE_V_REL, 0, -tmp_.cam_speed, &camera_);
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

      // Applied later: the owning windows do not exist until first submitted.
      window_state_storage_ =
          platform::ReadIniSection(settings, "[Studio][WindowStateStorage]");
    }
  }
}

// Key for one entry in state storage; the window name locates its owner on
// load.
static std::string WindowStateStorageKey(const char* window_name, ImGuiID id) {
  char id_str[16];
  std::snprintf(id_str, sizeof(id_str), "%08X", id);
  return std::string(window_name) + "/" + id_str;
}

void App::ApplyWindowStateStorage() {
  for (auto it = window_state_storage_.begin();
       it != window_state_storage_.end();) {
    const std::string::size_type sep = it->first.rfind('/');
    if (sep == std::string::npos) {
      it = window_state_storage_.erase(it);
      continue;
    }
    // Child windows are "<parent>/<child>_<id>": id follows the last '/'.
    ImGuiWindow* window =
        ImGui::FindWindowByName(it->first.substr(0, sep).c_str());
    if (window == nullptr) {
      ++it;  // Window not submitted yet; try again next frame.
      continue;
    }
    const ImGuiID id = std::strtoul(it->first.c_str() + sep + 1, nullptr, 16);
    window->StateStorage.SetInt(id, std::stoi(it->second) != 0);
    it = window_state_storage_.erase(it);
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

    // Pending entries first, so state for windows never opened this session is
    // not dropped, then let the live windows override. Merge into a local copy:
    // window_state_storage_ must stay pending-only, or a stale value could
    // re-apply over a newer toggle.
    platform::KeyValues window_state_storage = window_state_storage_;
    ImGuiContext& g = *ImGui::GetCurrentContext();
    for (ImGuiWindow* window : g.Windows) {
      for (const ImGuiStoragePair& pair : window->StateStorage.Data) {
        window_state_storage[WindowStateStorageKey(window->Name, pair.key)] =
            std::to_string(pair.val_i);
      }
    }
    platform::AppendIniSection(settings, "[Studio][WindowStateStorage]",
                               window_state_storage);

    platform::SaveText(settings, ini_path_);
  }
}

void App::SetSpeedIndex(int idx) {
  platform::SetSpeedIndex(&step_control_, tmp_.speed_index, idx);
}

void App::BuildGui() {
  if (tmp_.full_screen) {
    return;
  }

  if (!tmp_.style_editor) {
    platform::SetupTheme(ui_.theme);
  }

  ImGui::GetIO().FontGlobalScale = ui_.font_scale;

  const ImVec4 workspace_rect =
      platform::ConfigureDockingLayout(tmp_.toolbar, tmp_.status_bar);

  // Place charts in bottom right corner of the workspace.
  const ImVec2 chart_size(250, 500);
  const ImVec2 chart_pos(workspace_rect.x + workspace_rect.z - chart_size.x,
                         workspace_rect.y + workspace_rect.w - chart_size.y);

  MainMenuGui();

  if (tmp_.toolbar) {
    if (ImGui::Begin("ToolBar")) {
      ToolBarGui();
    }
    ImGui::End();
  }

  if (tmp_.status_bar) {
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
    if (ImGui::Begin("Explorer", &tmp_.inspector_panel)) {
      SpecExplorerGui();
    }
    ImGui::End();

    if (ImGui::Begin("Inspector", &tmp_.inspector_panel)) {
      DataInspectorGui();
    }
    ImGui::End();
  }

  if (tmp_.editor_panel) {
    if (ImGui::Begin("Editor", &tmp_.editor_panel)) {
      SpecEditorGui();
    }
    ImGui::End();
  }

  if (tmp_.profiler) {
    if (ImGui::Begin("Profiler", &tmp_.profiler,
                     ImGuiWindowFlags_NoScrollbar |
                         ImGuiWindowFlags_NoScrollWithMouse)) {
      platform::ProfilerGui(model(), data(), &profiler_,
                            tmp_.profiler_show_iter);
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
    if (ImGui::Begin("Help", &tmp_.help, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 5.0f));
      HelpGui();
      ImGui::PopStyleVar();
    }
    ImGui::End();
  }

  if (!load_error_.empty()) {
    const float scale = ImGui::GetWindowDpiScale();
    const float max_line_width = ImGui::CalcTextSize(load_error_.c_str()).x;
    const float padding = 30.0f * scale;
    const float max_workspace_width = std::max(0.0f, workspace_rect.z - 20.0f);
    const float min_target_width =
        std::min(350.0f * scale, max_workspace_width);
    const float target_width = std::clamp(
        max_line_width + padding, min_target_width, max_workspace_width);

    if (platform::BeginOverlay("WarningOverlay", platform::OverlayPos::kBottom,
                               workspace_rect, target_width, 0.8f)) {
      const bool is_dark = ImGui::GetStyle().Colors[ImGuiCol_WindowBg].x < 0.5f;
      if (!model_holder_->warning().empty()) {
        const ImVec4 warning_color = is_dark ? ImVec4(1.0f, 0.8f, 0.2f, 1.0f)
                                             : ImVec4(0.7f, 0.45f, 0.0f, 1.0f);
        ImGui::TextColored(warning_color, "Warning:");
      } else {
        const ImVec4 error_color = is_dark ? ImVec4(1.0f, 0.4f, 0.4f, 1.0f)
                                           : ImVec4(0.8f, 0.1f, 0.1f, 1.0f);
        ImGui::TextColored(error_color, "Compiler Error:");
      }
      ImGui::Separator();
      if (max_line_width + padding > max_workspace_width) {
        ImGui::PushTextWrapPos(max_workspace_width - padding);
        ImGui::TextUnformatted(load_error_.c_str());
        ImGui::PopTextWrapPos();
      } else {
        ImGui::TextUnformatted(load_error_.c_str());
      }
    }
    platform::EndOverlay();
  }

  if (tmp_.info) {
    const float scale = ImGui::GetWindowDpiScale();
    if (platform::BeginOverlay("Info", platform::OverlayPos::kBottomLeft,
                               workspace_rect, 180.0f * scale)) {
      const float fps = renderer_->GetFps();
      platform::InfoGui(
          model(), data(),
          step_control_.GetPauseState() == PauseState::kNormalPaused, fps);
    }
    platform::EndOverlay();
  }

  // pause overlay
  if (tmp_.info && has_model() &&
      step_control_.GetPauseState() == PauseState::kNormalPaused) {
    platform::TextOverlay("Pause", platform::OverlayPos::kTop, workspace_rect,
                          "PAUSE", ImVec4(0, 0, 0, 0), 3.0f);
  }

  // realtime factor overlay
  if (has_model() && step_control_.GetPauseState() == PauseState::kUnpaused) {
    const float desired = step_control_.GetSpeed();
    const float measured = step_control_.GetSpeedMeasured();
    if (desired != 100.0f || mju_abs(measured - desired) > 0.1f * desired) {
      char rtlabel[30];
      bool misaligned = mju_abs(measured - desired) > 0.1f * desired;
      if (misaligned) {
        std::snprintf(rtlabel, sizeof(rtlabel), "%g%% (%-4.1f%%)", desired,
                      measured);
      } else {
        std::snprintf(rtlabel, sizeof(rtlabel), "%g%%", desired);
      }
      platform::TextOverlay("Realtime", platform::OverlayPos::kTopLeft,
                            workspace_rect, rtlabel, ImVec4(0, 0, 0, 0), 2.0f);
    }
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
      if (ImGui::Begin(plugin->name, &plugin->active)) {
        plugin->update(plugin);
      }
      ImGui::End();
    }
  });

  // This frame's windows are submitted, so pending state storage can be
  // restored. Must precede the save below, which would otherwise write the
  // defaults.
  ApplyWindowStateStorage();

  ImGuiIO& io = ImGui::GetIO();
  if (io.WantSaveIniSettings) {
    SaveSettings();
    io.WantSaveIniSettings = false;
  }
}

void App::ModelOptionsGui() {
  const float min_width = platform::GetExpectedLabelWidth();
  const ImGuiChildFlags child_flags =
      ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysAutoResize;
  const ImGuiTreeNodeFlags node_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  const platform::SimulationGuiContext sim_ctx = {
      .model = model(),
      .data = data(),
      .step_control = &step_control_,
      .history = &sim_history_,
      .timeline = &timeline_,
      .speed_index = &tmp_.speed_index,
      .key_idx = &ui_.key_idx,
      .nthread = &ui_.nthread,
      .update_threadpool = &tmp_.update_threadpool,
      .reset = [this] { ResetPhysics(); },
      .reload = [this] { RequestModelReload(); },
      .align =
          [this] {
            const int cam_id = model()->vis.global.cameraid;
            if (cam_id >= 0 && cam_id < model()->ncam) {
              ui_.camera_idx = platform::SetCamera(model(), &camera_, cam_id);
            } else {
              mjv_defaultFreeCamera(model(), &camera_);
            }
          },
  };
  platform::SimulationGui(sim_ctx);

  ImGui::BeginChild("PhysicsGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Physics", node_flags, 0.65f)) {
    platform::PhysicsGui(model(), spec(), min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("RenderingGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Rendering", node_flags, 0.65f)) {
    ImGui::PushID("RenderingSection");

    // Compute combo width so labels align on the right.
    char frame_label_tmp[32];
    std::snprintf(frame_label_tmp, sizeof(frame_label_tmp), " %s  Camera",
                  platform::ICON_FA_CAMERA);
    const float combo_w = -ImGui::CalcTextSize(frame_label_tmp).x -
                          ImGui::GetStyle().ItemInnerSpacing.x;

    // Camera selector.
    {
      std::string cam_name =
          platform::GetCameraName(model(), camera_, ui_.camera_idx);

      char camera_text[32];
      std::snprintf(camera_text, sizeof(camera_text), "%s  Camera",
                    platform::ICON_FA_CAMERA);

      ImGui::SetNextItemWidth(combo_w);
      if (ImGui::BeginCombo(camera_text, cam_name.c_str())) {
        auto select = [&](int idx) {
          std::string name = platform::GetCameraName(model(), camera_, idx);
          if (ImGui::Selectable(name.c_str(), (ui_.camera_idx == idx))) {
            ui_.camera_idx = platform::SetCamera(model(), &camera_, idx);
          }
        };
        select(platform::kTumbleCameraIdx);
        select(platform::kFreeCameraIdx);
        select(platform::kTrackingCameraIdx);
        for (int c = 0; c < model()->ncam; c++) {
          select(c);
        }
        ImGui::EndCombo();
      }
    }

    // Label selector.
    {
      char label_text[32];
      std::snprintf(label_text, sizeof(label_text), "%s  Label",
                    platform::ICON_FA_COMMENT);
      ImGui::SetNextItemWidth(combo_w);
      if (ImGui::BeginCombo(label_text, mjLABELSTRING[vis_options_.label])) {
        for (int n = 0; n < mjNLABEL; n++) {
          if (ImGui::Selectable(mjLABELSTRING[n], (vis_options_.label == n))) {
            vis_options_.label = n;
          }
        }
        ImGui::EndCombo();
      }
    }

    // Frame selector.
    {
      char frame_text[32];
      std::snprintf(frame_text, sizeof(frame_text), "%s  Frame",
                    platform::ICON_FA_ARROWS);
      ImGui::SetNextItemWidth(combo_w);
      if (ImGui::BeginCombo(frame_text, mjFRAMESTRING[vis_options_.frame])) {
        for (int n = 0; n < mjNFRAME; n++) {
          if (ImGui::Selectable(mjFRAMESTRING[n], (vis_options_.frame == n))) {
            vis_options_.frame = n;
          }
        }
        ImGui::EndCombo();
      }
    }

    // Copy camera button.
    {
      char copy_cam_label[48];
      std::snprintf(copy_cam_label, sizeof(copy_cam_label), "%s  Copy Camera",
                    platform::ICON_FA_COPY);
      if (ImGui::Button(copy_cam_label)) {
        std::string camera_string = platform::CameraToString(data(), &camera_);
        platform::MaybeSaveToClipboard(camera_string);
      }
      ImGui::SetItemTooltip(
          "%s", "Copy the current camera pose as an MJCF XML string");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::PopID();

    platform::RenderingGui(model(), &vis_options_, renderer_->GetRenderFlags(),
                           min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("GroupsGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Visibility Groups", node_flags, 0.65f)) {
    platform::GroupsGui(model(), &vis_options_, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("VisualizationGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Visualization", node_flags, 0.65f)) {
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

  const float min_width = platform::GetExpectedLabelWidth();
  const ImGuiChildFlags child_flags =
      ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysAutoResize;
  const ImGuiTreeNodeFlags node_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  ImGui::BeginChild("JointsGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Joints", node_flags, 0.65f)) {
    platform::JointsGui(model(), data(), &vis_options_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("ControlsGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Controls", node_flags, 0.65f)) {
    platform::NoiseGui(&step_control_);
    ImGui::Separator();

    platform::ControlsGui(model(), data(), &vis_options_);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("SensorGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Sensor", node_flags, 0.65f)) {
    platform::SensorGui(model(), data());
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("WatchGui", {0, 0}, child_flags);
  if (platform::SectionHeader("Watch", node_flags, 0.65f)) {
    platform::WatchGui(model(), data(), ui_.watch_field,
                       sizeof(ui_.watch_field), ui_.watch_index);
    ImGui::TreePop();
  }
  ImGui::EndChild();

  ImGui::BeginChild("StateGui", {0, 0}, child_flags);
  if (platform::SectionHeader("State", node_flags, 0.65f)) {
    platform::StateGui(model(), data(), tmp_.state, tmp_.state_sig, min_width);
    ImGui::TreePop();
  }
  ImGui::EndChild();
}

void App::SpecExplorerGui() {
  if (!has_spec()) {
    ImGui::Text("No mjSpec loaded.");
    return;
  }

  // Initialize the split height.
  const ImVec2 region = ImGui::GetContentRegionAvail();
  if (region.y > 60.0f) {
    if (tmp_.explorer_split < 0) {
      tmp_.explorer_split = region.y * 0.7f;
    }
    tmp_.explorer_split =
        std::clamp(tmp_.explorer_split, 20.0f, region.y - 40.0f);
  }

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
      bool is_dark = ImGui::GetStyle().Colors[ImGuiCol_WindowBg].x < 0.5f;
      ImColor compile_green =
          is_dark ? ImColor(40, 125, 60, 255) : ImColor(40, 180, 40, 255);
      ImGui::PushStyleColor(ImGuiCol_Button, compile_green.Value);
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
  if (region.y > 60.0f) {
    if (tmp_.editor_split < 0) {
      tmp_.editor_split = region.y * 0.7f;
    }
    tmp_.editor_split = std::clamp(tmp_.editor_split, 20.0f, region.y - 40.0f);
  }

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
  // With ConfigMacOSXBehaviors, imgui swaps the Cmd and Ctrl keys, so
  // ImGuiMod_Ctrl chords and io.KeyCtrl match the Command key on macOS.
  const char* mod = ImGui::GetIO().ConfigMacOSXBehaviors ? "Cmd" : "Ctrl";

  const float pad = ImGui::GetStyle().ItemSpacing.x;
  const float indent = pad * 2;
  const float col0 = ImGui::CalcTextSize("Toggle Fullscreen").x + pad;
  const float col1 = ImGui::CalcTextSize("Ctrl+R-Dblclick").x + pad + indent;
  const float col2 = ImGui::CalcTextSize("Center of Mass").x + pad + indent;
  const float col3 = ImGui::CalcTextSize("M").x + pad + indent;
  ImGui::Dummy(ImVec2(col0 + col1 + col2 + col3, 0));
  ImGui::Columns(4);
  ImGui::SetColumnWidth(0, col0);
  ImGui::SetColumnWidth(1, col1);
  ImGui::SetColumnWidth(2, col2);
  ImGui::SetColumnWidth(3, col3);

  ImGui::SeparatorText("Windows & UI");
  ImGui::Text("Help");
  ImGui::Text("Info");
  ImGui::Text("Profiler");
  ImGui::Text("Toggle Fullscreen");
  ImGui::Text("Toggle Left UI");
  ImGui::Text("Toggle Right UI");
  ImGui::Text("Font Bigger");
  ImGui::Text("Font Smaller");
  ImGui::SeparatorText("Simulation");
  ImGui::Text("Toggle Pause");
  ImGui::Text("Reset Sim");
  ImGui::Text("Step Back");
  ImGui::Text("Step Forward");
  ImGui::Text("Speed Up");
  ImGui::Text("Speed Down");
  ImGui::Text("Copy Keyframe");
  ImGui::SeparatorText("Camera");
  ImGui::Text("Tumble Camera");
  ImGui::Text("Prev Camera");
  ImGui::Text("Next Camera");
  ImGui::Text("Align Camera");
  ImGui::Text("Orbit Camera");
  ImGui::Text("Pan Camera");
  ImGui::Text("Zoom Camera");
  ImGui::Text("Center Camera");
  ImGui::Text("Track Camera");
  ImGui::SeparatorText("Selection");
  ImGui::Text("Select");
  ImGui::Text("Select Parent");
  ImGui::Text("Rotate Object");
  ImGui::Text("Move Object");
  ImGui::SeparatorText("Visualization");
  ImGui::Text("Cycle Frames");
  ImGui::Text("Cycle Labels");
  ImGui::Text("Geom Group");
  ImGui::Text("Site Group");

  ImGui::NextColumn();
  ImGui::Indent(indent);
  ImGui::SeparatorText("");
  ImGui::Text("F1");
  ImGui::Text("F2");
  ImGui::Text("F3");
  ImGui::Text("F11");
  ImGui::Text("Tab");
  ImGui::Text("Sh+Tab");
  ImGui::Text("%s+=", mod);
  ImGui::Text("%s+-", mod);
  ImGui::SeparatorText("");
  ImGui::Text("Spc");
  ImGui::Text("Bksp");
  ImGui::Text("Left");
  ImGui::Text("Right");
  ImGui::Text("+");
  ImGui::Text("-");
  ImGui::Text("%s+C", mod);
  ImGui::SeparatorText("");
  ImGui::Text("Esc");
  ImGui::Text("[");
  ImGui::Text("]");
  ImGui::Text("%s+A", mod);
  ImGui::Text("L-Drag");
  ImGui::Text("[Sh+]R-Drag");
  ImGui::Text("Scroll");
  ImGui::Text("R-Dblclick");
  ImGui::Text("%s+R-Dblclick", mod);
  ImGui::SeparatorText("");
  ImGui::Text("Dblclick");
  ImGui::Text("PgUp");
  ImGui::Text("%s+L-Drag", mod);
  ImGui::Text("%s+R-Drag", mod);
  ImGui::SeparatorText("");
  ImGui::Text("F6");
  ImGui::Text("F7");
  ImGui::Text("0-5");
  ImGui::Text("Sh+0-5");

  ImGui::NextColumn();
  ImGui::Indent(indent);
  ImGui::SeparatorText("Flags");
  ImGui::Text("Activation");
  ImGui::Text("Actuator");
  ImGui::Text("Auto Connect");
  ImGui::Text("Body Tree");
  ImGui::Text("Camera");
  ImGui::Text("Center of Mass");
  ImGui::Text("Constraint");
  ImGui::Text("Contact Force");
  ImGui::Text("Contact Point");
  ImGui::Text("Contact Split");
  ImGui::Text("Convex Hull");
  ImGui::Text("Inertia");
  ImGui::Text("Island");
  ImGui::Text("Joint");
  ImGui::Text("Light");
  ImGui::Text("Mesh Tree");
  ImGui::Text("Perturb Force");
  ImGui::Text("Perturb Object");
  ImGui::Text("Range Finder");
  ImGui::Text("Scale Inertia");
  ImGui::Text("Skin");
  ImGui::Text("Static Body");
  ImGui::Text("Tendon");
  ImGui::Text("Texture");
  ImGui::Text("Transparent");
  ImGui::Text("Wireframe");

  ImGui::NextColumn();
  ImGui::Indent(indent);
  ImGui::SeparatorText("");
  ImGui::Text(",");
  ImGui::Text("U");
  ImGui::Text("A");
  ImGui::Text("`");
  ImGui::Text("Q");
  ImGui::Text("M");
  ImGui::Text("E");
  ImGui::Text("F");
  ImGui::Text("C");
  ImGui::Text("P");
  ImGui::Text("H");
  ImGui::Text("I");
  ImGui::Text("N");
  ImGui::Text("J");
  ImGui::Text("Z");
  ImGui::Text("\\");
  ImGui::Text("B");
  ImGui::Text("O");
  ImGui::Text("Y");
  ImGui::Text("'");
  ImGui::Text(";");
  ImGui::Text("D");
  ImGui::Text("V");
  ImGui::Text("X");
  ImGui::Text("T");
  ImGui::Text("W");

  ImGui::Columns();
}

void App::ToolBarGui() {
  ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(0, 0));
  if (ImGui::BeginTable("##ToolBarTable", 2)) {
    platform::ScopedStyle style;
    style.Var(ImGuiStyleVar_ItemSpacing,
              ImVec2(ImGui::GetStyle().ItemSpacing.x * 2.0f,
                     ImGui::GetStyle().ItemSpacing.y));

    const float label_width = platform::GetExpectedLabelWidth();
    const float copy_btn_width = ImGui::GetFrameHeight();
    const float sp = ImGui::GetStyle().ItemSpacing.x;
    const float right_width =
        copy_btn_width + label_width + sp + label_width + sp + label_width;
    const float separator_width = ImGui::GetFrameHeight() * .6f;

    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, right_width);

    ImGui::TableNextColumn();
    ImGui::Dummy(ImVec2(ImGui::GetStyle().WindowPadding.x, 0.0f));
    ImGui::SameLine(0.0f, 0.0f);

    const float btn_size = ImGui::GetFrameHeight();
    const ImVec2 square_size(btn_size, btn_size);

    // Reload button.
    {
      style.Var(ImGuiStyleVar_FrameRounding, 2.0f);
      if (ImGui::Button(ICON_RELOAD_MODEL, square_size)) {
        RequestModelReload();
      }
      ImGui::SetItemTooltip("%s", "Reload");
    }

    // Reset button.
    ImGui::SameLine(0, 0.5 * separator_width);
    if (ImGui::Button(ICON_RESET_MODEL, square_size)) {
      ResetPhysics();
    }
    ImGui::SetItemTooltip("%s", "Reset");

    // Combined (Pause, Play) widget and Speed selection.
    ImGui::SameLine(0, separator_width);
    platform::StepControlGui(&step_control_, tmp_.speed_index);

    ImGui::SameLine(0, separator_width);
    platform::TimelineScrubberGui(model(), data(), step_control_, sim_history_,
                                  timeline_);

    ImGui::TableNextColumn();

    platform::CameraSelectionGui(model(), data(), camera_, ui_.camera_idx);

    ImGui::SameLine();
    platform::LabelSelectionGui(&vis_options_);

    ImGui::SameLine();
    platform::FrameSelectionGui(&vis_options_);

    ImGui::EndTable();
  }
  ImGui::PopStyleVar();
}

void App::StatusBarGui() {
  if (ImGui::BeginTable("##StatusBarTable", 1)) {
    ImGui::TableNextColumn();

    if (!has_model()) {
      ImGui::Text("No model loaded");
    } else if (step_control_.GetPauseState() == PauseState::kNormalPaused) {
      ImGui::Text("Paused");
    } else {
      ImGui::Text("Running");
    }

    if (!step_error_.empty()) {
      ImGui::SameLine();
      ImGui::Text(" | Step Error: %s", step_error_.c_str());
    } else if (!edit_error_.empty()) {
      ImGui::SameLine();
      ImGui::Text(" | Edit Error: %s", edit_error_.c_str());
    }

    ImGui::EndTable();
  }
}

void App::MainMenuGui() {
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,
                      ImVec2(12.0f * ImGui::GetStyle().FontScaleDpi,
                             ImGui::GetStyle().ItemSpacing.y));
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
      if (ImGui::MenuItem("Save Config")) {
        SaveSettings();
      }
      if (ImGui::MenuItem("Reset Config")) {
        platform::ResetConfig(ini_path_);
        LoadSettings();
      }
      ImGui::Separator();

      if (ImGui::MenuItem(tmp_.options_panel ? "Hide Options" : "Show Options",
                          "Tab")) {
        tmp_.options_panel = !tmp_.options_panel;
      }
      if (ImGui::MenuItem(
              tmp_.inspector_panel ? "Hide Inspector" : "Show Inspector",
              "Shift+Tab")) {
        tmp_.inspector_panel = !tmp_.inspector_panel;
      }
      if (ImGui::MenuItem(tmp_.editor_panel ? "Hide Editor" : "Show Editor")) {
        tmp_.editor_panel = !tmp_.editor_panel;
        if (tmp_.editor_panel) {
          tmp_.inspector_panel = true;
        }
      }
      if (ImGui::MenuItem(tmp_.toolbar ? "Hide Toolbar" : "Show Toolbar")) {
        tmp_.toolbar = !tmp_.toolbar;
      }
      if (ImGui::MenuItem(tmp_.status_bar ? "Hide Status Bar"
                                          : "Show Status Bar")) {
        tmp_.status_bar = !tmp_.status_bar;
      }
      if (ImGui::MenuItem("Full Screen", "F11")) {
        tmp_.full_screen = !tmp_.full_screen;
      }
      ImGui::Separator();

      if (ImGui::MenuItem("Info", "F2", tmp_.info)) {
        ToggleWindow(tmp_.info);
      }
      if (ImGui::MenuItem("Profiler", "F3", tmp_.profiler)) {
        if (!tmp_.profiler) {
          tmp_.profiler = true;
          tmp_.profiler_show_iter = false;
        } else if (!tmp_.profiler_show_iter) {
          tmp_.profiler_show_iter = true;
        } else {
          tmp_.profiler = false;
        }
      }
      ImGui::Separator();

      if (ImGui::MenuItem("Picture-in-Picture")) {
        tmp_.picture_in_picture = !tmp_.picture_in_picture;
      }
      ImGui::Separator();

      platform::ThemeMenuGui(&ui_.theme);

#ifdef __linux__
      if (ImGui::BeginMenu("Graphics Mode (Experimental)")) {
        std::optional<platform::GraphicsMode> mode;
        if (ImGui::MenuItem(
                "Classic OpenGL", nullptr,
                gfx_mode_ == platform::GraphicsMode::ClassicOpenGl)) {
          mode = platform::GraphicsMode::ClassicOpenGl;
        }
        if (ImGui::MenuItem(
                "Classic OpenGL Headless", nullptr,
                gfx_mode_ == platform::GraphicsMode::ClassicOpenGlHeadless)) {
          mode = platform::GraphicsMode::ClassicOpenGlHeadless;
        }
        if (ImGui::MenuItem(
                "Filament OpenGL", nullptr,
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
        if (ImGui::MenuItem(
                "Filament Vulkan", nullptr,
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
      ImGui::Separator();
      std::string version = "Version " + std::string(mj_versionString());
      ImGui::MenuItem(version.c_str());
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  ImGui::PopStyleVar();
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
          SaveToWebp(width, height, buffer.data(), res.path);
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
