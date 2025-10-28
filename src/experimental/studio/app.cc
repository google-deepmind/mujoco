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
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "experimental/toolbox/helpers.h"
#include "experimental/toolbox/imgui_widgets.h"
#include "experimental/toolbox/interaction.h"
#include "experimental/toolbox/physics.h"
#include "experimental/toolbox/renderer.h"
#include "experimental/toolbox/window.h"

#if defined(USE_FILAMENT_OPENGL) || defined(USE_FILAMENT_VULKAN)
#include "experimental/filament/render_context_filament.h"
#elif defined(USE_CLASSIC_OPENGL)
#include <backends/imgui_impl_opengl3.h>
#else
#error No rendering mode defined.
#endif

namespace mujoco::studio {

// TO DO:
// - sensor graph
// - convergence profiler
// - solver iteration profiler
// - "passive" mode
// - async physics

static constexpr toolbox::Window::Config kWindowConfig = {
#if defined(USE_FILAMENT_VULKAN)
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

// FontAwesome icon codes.
static constexpr const char* ICON_PLAY = "\xef\x81\x8b";
static constexpr const char* ICON_PAUSE = "\xef\x81\x8c";
static constexpr const char* ICON_DARKMODE = "\xef\x86\x86";
static constexpr const char* ICON_LIGHTMODE = "\xef\x86\x85";

// UI labels for mjtLabel.
static constexpr const char* kLabelNames[] = {
    "None",      "Body",    "Joint",    "Geom",       "Site",  "Camera",
    "Light",     "Tendon",  "Actuator", "Constraint", "Flex",  "Skin",
    "Selection", "Sel Pnt", "Contact",  "Force",      "Island"};

// UI labels for mjtFrame.
static constexpr const char* kFrameNames[] = {
    "None", "Body", "Geom", "Site", "Camera", "Light", "Contact", "World"};

using toolbox::ImGui_FileDialog;
using toolbox::ImGui_Input;
using toolbox::ImGui_InputN;
using toolbox::ImGui_IsChordJustPressed;
using toolbox::ImGui_Slider;
using toolbox::Toggle;
using toolbox::ToggleBit;
using toolbox::ToggleKind;

App::App(int width, int height, std::string ini_path,
         const toolbox::LoadAssetFn& load_asset_fn)
    : ini_path_(std::move(ini_path)), load_asset_fn_(load_asset_fn) {
  window_ = std::make_unique<toolbox::Window>("MuJoCo Studio", width, height,
                                              kWindowConfig, load_asset_fn);

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

  physics_ = std::make_unique<toolbox::Physics>(
      [this](std::string_view model_file) { this->OnModelLoaded(model_file); });

  mjv_defaultPerturb(&perturb_);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&vis_options_);

  LoadSettings();
  ClearProfilerData();

#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_Init();
#endif
}

void App::LoadModel(std::string model_file) {
  model_file_ = std::move(model_file);
  physics_->LoadModel(model_file_);
}

void App::OnModelLoaded(std::string_view model_file) {
  renderer_->Init(Model());
  tmp_ = UiTempState();
  mjv_defaultOption(&vis_options_);
  ClearProfilerData();
  if (model_file.empty()) {
    window_->SetTitle("MuJoCo Studio");
  } else {
    window_->SetTitle("MuJoCo Studio : " + std::string(model_file));
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
    if (physics_->Update(&perturb_)) {
      UpdateProfilerData();
    }
  }

  return status == toolbox::Window::Status::kRunning;
}

void App::Sync() {
  renderer_->Sync(Model(), Data(), &perturb_, &camera_, &vis_options_);
}

void App::Render() {
  const float width = window_->GetWidth();
  const float height = window_->GetHeight();
  const float scale = window_->GetScale();

  renderer_->Render(Model(), Data(), &perturb_, &camera_, &vis_options_,
                    width * scale, height * scale);

#ifdef USE_CLASSIC_OPENGL
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif

  // This call to EndFrame() is only needed if render_config.enable_gui is false
  window_->EndFrame();
  window_->Present();

  if (Data()) {
    for (int i = 0; i < mjNTIMER; i++) {
      Data()->timer[i].duration = 0;
      Data()->timer[i].number = 0;
    }
  }
}

void App::HandleMouseEvents() {
  auto& io = ImGui::GetIO();
  if (io.WantCaptureMouse) {
    return;
  }

  mjModel* model = physics_->GetModel();
  mjData* data = physics_->GetData();
  mjvScene& scene = renderer_->GetScene();

  // Normalize mouse positions and movement to display size.
  const float mouse_x = io.MousePos.x / io.DisplaySize.x;
  const float mouse_y = io.MousePos.y / io.DisplaySize.y;
  const float mouse_dx = io.MouseDelta.x / io.DisplaySize.x;
  const float mouse_dy = io.MouseDelta.y / io.DisplaySize.y;
  const float mouse_scroll = io.MouseWheel / 50.0f;

  // Determine the mouse action based on which buttons are down.
  mjtMouse action = mjMOUSE_NONE;
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
    action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
    action = mjMOUSE_ZOOM;
  } else {
    // If no mouse buttons are down, end any active perturbations.
    perturb_.active = 0;
  }

  // Mouse scroll.
  if (model && mouse_scroll != 0.0f) {
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, mouse_scroll, &scene, &camera_);
  }

  // Mouse drag.
  if (model && data && action != mjMOUSE_NONE &&
      (mouse_dx != 0.0f || mouse_dy != 0.0f)) {
    // If ctrl is pressed, move the perturbation, otherwise move the camera_.
    if (io.KeyCtrl) {
      if (perturb_.select > 0) {
        const int active =
            action == mjMOUSE_MOVE_V ? mjPERT_TRANSLATE : mjPERT_ROTATE;
        if (active != perturb_.active) {
          mjv_initPerturb(model, data, &scene, &perturb_);
          perturb_.active = active;
        }
        mjv_movePerturb(model, data, action, mouse_dx, mouse_dy, &scene,
                        &perturb_);
      }
    } else {
      mjv_moveCamera(model, action, mouse_dx, mouse_dy, &scene, &camera_);
    }
  }

  // Left double click.
  if (data && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
    toolbox::PickResult picked = toolbox::Pick(
        model, data, &camera_, mouse_x, mouse_y, window_->GetAspectRatio(),
        &renderer_->GetScene(), &vis_options_);
    if (picked.body >= 0) {
      perturb_.select = picked.body;
      perturb_.flexselect = picked.flex;
      perturb_.skinselect = picked.skin;

      // Compute the local position of the selected object in the world.
      mjtNum tmp[3];
      mju_sub3(tmp, picked.point, data->xpos + 3 * picked.body);
      mju_mulMatTVec(perturb_.localpos, data->xmat + 9 * picked.body, tmp, 3, 3);
    } else {
      perturb_.select = 0;
      perturb_.flexselect = -1;
      perturb_.skinselect = -1;
    }
  }

  // Right double click.
  if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right)) {
    toolbox::PickResult picked = toolbox::Pick(
        model, data, &camera_, mouse_x, mouse_y, window_->GetAspectRatio(),
        &renderer_->GetScene(), &vis_options_);
    mju_copy3(camera_.lookat, picked.point);
    if (picked.body > 0 && io.KeyCtrl) {
      camera_.type = mjCAMERA_TRACKING;
      camera_.trackbodyid = picked.body;
      camera_.fixedcamid = -1;
      ui_.camera_idx = 1;
    }
  }
}

void App::ChangeSpeed(int delta) {
  if (delta == 0) {
    return;
  }

  // logarithmically spaced real-time slow-down coefficients (percent)
  // clang-format off
  static constexpr std::array<float, 31> kPercentRealTime = {
    100.0,  80.0,  66.00,  50.0,  40.0,  33.00,  25.00,  20.0,  16.00,  13.00,
     10.0,   8.0,   6.60,   5.0,   4.0,   3.30,   2.50,   2.0,   1.60,   1.30,
      1.0,    .8,    .66,    .5,    .4,    .33,    .25,    .2,    .16,    .13,
       .1};
  // clang-format on

  tmp_.speed_index += delta;
  tmp_.speed_index =
      std::clamp<int>(tmp_.speed_index, 0, kPercentRealTime.size() - 1);

  float speed = kPercentRealTime[tmp_.speed_index];
  physics_->GetStepControl().SetSpeed(speed);
}

void App::HandleKeyboardEvents() {
  if (ImGui::GetIO().WantCaptureKeyboard) {
    return;
  }

  mjModel* model = physics_->GetModel();

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
    std::string keyframe = toolbox::KeyframeToString(Model(), Data(), false);
    toolbox::MaybeSaveToClipboard(keyframe);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_R | ImGuiMod_Ctrl)) {
    LoadModel(model_file_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q | ImGuiMod_Ctrl)) {
    tmp_.should_exit = true;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_A | ImGuiMod_Ctrl)) {
    mjv_defaultFreeCamera(Model(), &camera_);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F1)) {
    ToggleWindow(tmp_.help);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F2)) {
    ToggleWindow(tmp_.info);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab | ImGuiMod_Shift)) {
    tmp_.show_ui_rhs = !tmp_.show_ui_rhs;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Tab)) {
    tmp_.show_ui_lhs = !tmp_.show_ui_lhs;
  }

  // Window toggles for non-classic UI.
  if (!ui_.classic_ui) {
    if (ImGui_IsChordJustPressed(ImGuiKey_S | ImGuiMod_Alt)) {
      ToggleWindow(ui_.simulation);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_W | ImGuiMod_Alt)) {
      ToggleWindow(ui_.watch);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_P | ImGuiMod_Alt)) {
      ToggleWindow(ui_.physics);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_R | ImGuiMod_Alt)) {
      ToggleWindow(ui_.rendering);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_V | ImGuiMod_Alt)) {
      ToggleWindow(ui_.visualization);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_G | ImGuiMod_Alt)) {
      ToggleWindow(ui_.groups);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_J | ImGuiMod_Alt)) {
      ToggleWindow(ui_.joints);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_C | ImGuiMod_Alt)) {
      ToggleWindow(ui_.controls);
    }
  }

  if (ImGui_IsChordJustPressed(ImGuiKey_Minus)) {
    ChangeSpeed(1);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Equal)) {
    ChangeSpeed(-1);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftArrow)) {
    if (physics_->IsPaused()) {
      physics_->LoadHistory(ui_.scrub_idx - 1);
    }
  } else if (ImGui_IsChordJustPressed(ImGuiKey_RightArrow)) {
    if (physics_->IsPaused()) {
      if (ui_.scrub_idx == 0) {
        physics_->RequestSingleStep();
      } else {
        physics_->LoadHistory(ui_.scrub_idx + 1);
      }
    }
  }

  // Camera wasd controls.
  if (!ui_.classic_ui) {
    mjvScene& scene = renderer_->GetScene();
    mjModel* model = Model();

    // Move (dolly) forward/backward using W and S keys.
    if (ImGui::IsKeyDown(ImGuiKey_W)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, 0, 0.01, &scene, &camera_);
    } else if (ImGui::IsKeyDown(ImGuiKey_S)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, 0, -0.01, &scene, &camera_);
    }

    // Strafe (truck) left/right using A dna D keys.
    if (ImGui::IsKeyDown(ImGuiKey_A)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, -0.01, 0, &scene, &camera_);
    } else if (ImGui::IsKeyDown(ImGuiKey_D)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_H_REL, 0.01, 0, &scene, &camera_);
    }

    // Move (pedestal) up/down using Q and E keys.
    if (ImGui::IsKeyDown(ImGuiKey_Q)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_V_REL, 0, 0.01, &scene, &camera_);
    } else if (ImGui::IsKeyDown(ImGuiKey_E)) {
      mjv_moveCamera(model, mjMOUSE_MOVE_V_REL, 0, -0.01, &scene, &camera_);
    }
  }

  // Physics control shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_Space)) {
    physics_->TogglePause();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backspace)) {
    physics_->Reset();
  }

  // Camera shortcuts.
  if (model) {
    if (ImGui_IsChordJustPressed(ImGuiKey_Escape)) {
      ui_.camera_idx = toolbox::SetCamera(model, &camera_, 0);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftBracket)) {
      ui_.camera_idx = toolbox::SetCamera(model, &camera_,ui_.camera_idx - 1);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_RightBracket)) {
      ui_.camera_idx = toolbox::SetCamera(model, &camera_, ui_.camera_idx + 1);
    }
  }

  // Perturb shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_PageUp)) {
    SelectParentPerturb(model, perturb_);
  }

  // Visualization shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_F6)) {
    vis_options_.frame = (vis_options_.frame + 1) % mjNFRAME;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F7)) {
    vis_options_.label = (vis_options_.label + 1) % mjNLABEL;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Comma)) {
    ToggleFlag(vis_options_.flags[mjVIS_ACTIVATION]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backslash)) {
    ToggleFlag(vis_options_.flags[mjVIS_MESHBVH]);
    // } else if (ImGui_IsChordJustPressed(ImGuiKey_Backquote)) {
    //   ToggleFlag(vis_options_.flags[mjVIS_BODYBVH]);
    // } else if (ImGui_IsChordJustPressed(ImGuiKey_Quote)) {
    //   ToggleFlag(vis_options_.flags[mjVIS_SCLINERTIA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Semicolon)) {
    ToggleFlag(vis_options_.flags[mjVIS_SKIN]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_U)) {
    ToggleFlag(vis_options_.flags[mjVIS_ACTUATOR]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q)) {
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
  } else if (ImGui_IsChordJustPressed(ImGuiKey_E)) {
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

void App::SetCamera(int idx) {
  if (Model()) {
    ui_.camera_idx = ::mujoco::toolbox::SetCamera(Model(), &camera_, idx);
  }
}

void App::ClearProfilerData() {
  constexpr int kProfilerMaxFrames = 200;
  cpu_total_.clear();
  cpu_collision_.clear();
  cpu_prepare_.clear();
  cpu_solve_.clear();
  cpu_other_.clear();
  dim_dof_.clear();
  dim_body_.clear();
  dim_constraint_.clear();
  dim_sqrt_nnz_.clear();
  dim_contact_.clear();
  dim_iteration_.clear();

  cpu_total_.resize(kProfilerMaxFrames, 0);
  cpu_collision_.resize(kProfilerMaxFrames, 0);
  cpu_prepare_.resize(kProfilerMaxFrames, 0);
  cpu_solve_.resize(kProfilerMaxFrames, 0);
  cpu_other_.resize(kProfilerMaxFrames, 0);
  dim_dof_.resize(kProfilerMaxFrames, 0);
  dim_body_.resize(kProfilerMaxFrames, 0);
  dim_constraint_.resize(kProfilerMaxFrames, 0);
  dim_sqrt_nnz_.resize(kProfilerMaxFrames, 0);
  dim_contact_.resize(kProfilerMaxFrames, 0);
  dim_iteration_.resize(kProfilerMaxFrames, 0);
}

void App::UpdateProfilerData() {
  // CPU timers.
  mjtNum total = Data()->timer[mjTIMER_STEP].duration;
  mjtNum number = static_cast<mjtNum>(Data()->timer[mjTIMER_STEP].number);
  if (number == 0.0) {
    total = Data()->timer[mjTIMER_FORWARD].duration;
    number = static_cast<mjtNum>(Data()->timer[mjTIMER_FORWARD].number);
  }
  if (number == 0.0) {
    // This can happen if the simulation is paused.
    return;
  }

  cpu_total_.erase(cpu_total_.begin());
  cpu_total_.push_back(total / number);

  mjtNum collision = Data()->timer[mjTIMER_POS_COLLISION].duration / number;
  cpu_collision_.erase(cpu_collision_.begin());
  cpu_collision_.push_back(collision);

  mjtNum prepare = (Data()->timer[mjTIMER_POS_MAKE].duration / number) +
                   (Data()->timer[mjTIMER_POS_PROJECT].duration / number);
  cpu_prepare_.erase(cpu_prepare_.begin());
  cpu_prepare_.push_back(prepare);

  mjtNum solve = Data()->timer[mjTIMER_CONSTRAINT].duration / number;
  cpu_solve_.erase(cpu_solve_.begin());
  cpu_solve_.push_back(solve);

  mjtNum other = total - collision - prepare - solve;
  cpu_other_.erase(cpu_other_.begin());
  cpu_other_.push_back(other);

  // Dimensions.
  mjtNum sqrt_nnz = 0;
  int solver_niter = 0;
  const int nisland = mjMAX(1, mjMIN(Data()->nisland, mjNISLAND));
  for (int island = 0; island < nisland; island++) {
    sqrt_nnz += mju_sqrt(Data()->solver_nnz[island]);
    solver_niter += Data()->solver_niter[island];
  }

  dim_dof_.erase(dim_dof_.begin());
  dim_dof_.push_back(Model()->nv);

  dim_body_.erase(dim_body_.begin());
  dim_body_.push_back(Model()->nbody);

  dim_constraint_.erase(dim_constraint_.begin());
  dim_constraint_.push_back(Data()->nefc);

  dim_sqrt_nnz_.erase(dim_sqrt_nnz_.begin());
  dim_sqrt_nnz_.push_back(sqrt_nnz);

  dim_contact_.erase(dim_contact_.begin());
  dim_contact_.push_back(Data()->ncon);

  dim_iteration_.erase(dim_iteration_.begin());
  dim_iteration_.push_back(static_cast<float>(solver_niter) / nisland);
}

void App::BuildGuiWithWindows() {
  if (!tmp_.show_ui_lhs) {
    return;
  }

  if (ui_.simulation) {
    if (ImGui::Begin("Simulation", &ui_.simulation)) {
      SimulationGui();
    }
    ImGui::End();
  }
  if (ui_.physics) {
    if (ImGui::Begin("Physics", &ui_.physics)) {
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
      PhysicsGui();
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }
  if (ui_.rendering) {
    if (ImGui::Begin("Rendering", &ui_.rendering)) {
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
      RenderingGui();
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }
  if (ui_.visualization) {
    if (ImGui::Begin("Visualization", &ui_.visualization)) {
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
      VisualizationGui();
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }
  if (ui_.groups) {
    if (ImGui::Begin("Groups", &ui_.groups)) {
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
      GroupsGui();
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }
  if (ui_.joints) {
    if (ImGui::Begin("Joints", &ui_.joints)) {
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
      JointsGui();
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }
  if (ui_.controls) {
    if (ImGui::Begin("Controls", &ui_.controls)) {
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
      ControlsGui();
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }
  if (ui_.watch) {
    if (ImGui::Begin("Watch", &ui_.watch)) {
      WatchGui();
    }
    ImGui::End();
  }
  if (ui_.profiler) {
    if (ImGui::Begin("Profiler", &ui_.profiler)) {
      ProfilerGui();
    }
    ImGui::End();
  }
  if (ui_.sensor) {
    if (ImGui::Begin("Sensor", &ui_.sensor)) {
      SensorGui();
    }
    ImGui::End();
  }
}

void Section(const char* name, ImGuiTreeNodeFlags flags,
             std::function<void()> content, float indent_factor = 1.f) {
  if (ImGui::TreeNodeEx(name, flags)) {
    if (indent_factor > 0.f) {
      ImGui::Unindent(indent_factor * ImGui::GetTreeNodeToLabelSpacing());
    }
    content();
    if (indent_factor > 0.f) {
      ImGui::Indent(indent_factor * ImGui::GetTreeNodeToLabelSpacing());
    }
    ImGui::TreePop();
  }
}

void App::BuildGuiWithSections() {
  // Pixel coordinates of the bottom of the menu bar (the last thing drawn)
  float viewport_top = ImGui::GetMainViewport()->WorkPos.y;
  float viewport_width = ImGui::GetMainViewport()->WorkSize.x;

  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
                                  ImGuiWindowFlags_NoMove |
                                  ImGuiWindowFlags_NoScrollbar;
  ImGuiTreeNodeFlags section_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  const float height = window_->GetHeight();
  auto set_next_window_size_constraint = [height, viewport_top]() {
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(250, height - viewport_top),
        ImVec2(std::numeric_limits<float>::max(), height - viewport_top));
  };

  if (tmp_.show_ui_lhs) {
    set_next_window_size_constraint();
    ImGui::SetNextWindowPos(ImVec2(0, viewport_top), ImGuiCond_Always);

    // Hide the triangular resize grip in the window corner. We do this rather
    // then passing ImGuiWindowFlags_NoResize to the window because we still
    // want the window to be resizable by dragging on the edge with the mouse.
    struct ScopeStyle {
      ScopeStyle() { ImGui::PushStyleColor(ImGuiCol_ResizeGrip, 0); }
      ~ScopeStyle() { ImGui::PopStyleColor(); }
    } style;

    if (ImGui::Begin("LHS_UI", &tmp_.show_ui_lhs, window_flags)) {
      ImGuiTreeNodeFlags sim_section_flags =
          section_flags | ImGuiTreeNodeFlags_DefaultOpen;
      Section("Simulation", sim_section_flags, [this] { SimulationGui(); });
      Section("Physics", section_flags, [this] { PhysicsGui(); });
      Section("Rendering", section_flags, [this] { RenderingGui(); });
      Section("Visualization", section_flags, [this] { VisualizationGui(); });
      Section("Groups", section_flags, [this] { GroupsGui(); });
      Section("Watch", section_flags, [this] { WatchGui(); });

      // Cache the size and position of the window before we end it
      tmp_.size_ui_lhs[0] = ImGui::GetWindowSize().x;
      tmp_.size_ui_lhs[1] = ImGui::GetWindowSize().y;
      tmp_.pos_ui_lhs[0] = ImGui::GetWindowPos().x;
      tmp_.pos_ui_lhs[1] = ImGui::GetWindowPos().y;
    }
    ImGui::End();
  } else {
    tmp_.size_ui_lhs[0] = 0;
    tmp_.size_ui_lhs[1] = height - viewport_top;
    tmp_.pos_ui_lhs[0] = 0;
    tmp_.pos_ui_lhs[1] = viewport_top;
  }

  if (tmp_.show_ui_rhs) {
    set_next_window_size_constraint();
    ImGui::SetNextWindowPos(ImVec2(viewport_width, viewport_top),
                            ImGuiCond_Always, ImVec2(1, 0));

    if (ImGui::Begin("RHS_UI", &tmp_.show_ui_rhs, window_flags)) {
      Section("Joints", section_flags, [this] { JointsGui(); });
      Section("Controls", section_flags, [this] { ControlsGui(); });
      Section("Sensor", section_flags, [this] { SensorGui(); });
      Section("Profiler", section_flags, [this] { ProfilerGui(); });
      Section("State", section_flags, [this] { StateGui(); }, .5f);

      // Cache the size and position of the window before we end it
      tmp_.size_ui_rhs[0] = ImGui::GetWindowSize().x;
      tmp_.size_ui_rhs[1] = ImGui::GetWindowSize().y;
      tmp_.pos_ui_rhs[0] = ImGui::GetWindowPos().x;
      tmp_.pos_ui_rhs[1] = ImGui::GetWindowPos().y;
    }
    ImGui::End();
  } else {
    tmp_.size_ui_rhs[0] = 0;
    tmp_.size_ui_rhs[1] = height - viewport_top;
    tmp_.pos_ui_rhs[0] = viewport_width;
    tmp_.pos_ui_rhs[1] = viewport_top;
  }
}

void App::BuildGui() {
  if (!ui_.classic_ui) {
    const int dock_flags = ImGuiDockNodeFlags_PassthruCentralNode |
                           ImGuiDockNodeFlags_NoDockingOverCentralNode;
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), dock_flags);
  }

  MainMenuGui();
  FileDialogGui();

  if (tmp_.imgui_demo) {
    ImGui::ShowDemoWindow();
    ImPlot::ShowDemoWindow();
  }
  if (tmp_.style_editor) {
    if (ImGui::Begin("Style Editor", &tmp_.style_editor)) {
      ImGui::ShowStyleEditor();
    }
    ImGui::End();
  }

  if (model_file_.empty()) {
    const float width = window_->GetWidth();
    const float height = window_->GetHeight();

    const int overlay_flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground;
    ImGui::SetNextWindowPos(ImVec2(0, height / 2), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, height / 2), ImGuiCond_Always);
    if (ImGui::Begin("##Overlay", 0, overlay_flags)) {
      const char* text = "Load model file or drag-and-drop model file here.";
      const auto text_size = ImGui::CalcTextSize(text);

      ImGui::SetCursorPos(ImVec2((width - text_size.x) / 2, 0.0f));
      ImGui::Text("%s", text);
    }
    ImGui::End();
  }

  if (ui_.classic_ui) {
    BuildGuiWithSections();
  } else {
    BuildGuiWithWindows();
  }

  struct ScopeOverlayStyle {
    ScopeOverlayStyle(bool dark_mode) {
      ImColor color = dark_mode ? IM_COL32_BLACK : IM_COL32_WHITE;
      color.Value.w = dark_mode ? .5f : .1f;

      ImGui::PushStyleColor(ImGuiCol_WindowBg, (ImU32)color);
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32_WHITE);
      ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    }
    ~ScopeOverlayStyle() {
      ImGui::PopStyleColor(2);
      ImGui::PopStyleVar();
    }

    const float margin_px = 5;
    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoTitleBar   // No title bar
        | ImGuiWindowFlags_NoMove     // No window moving
        | ImGuiWindowFlags_NoResize   // No window resizing
        | ImGuiWindowFlags_NoInputs;  // User can click through the window
  };

  // Draw the paused overlay
  if (physics_->IsPaused()) {
    ScopeOverlayStyle style(ui_.dark_mode);
    ImVec2 pivot_pos;
    pivot_pos.x = ImGui::GetMainViewport()->WorkPos.x +
                  ImGui::GetMainViewport()->WorkSize.x * 0.5f;
    pivot_pos.y = ImGui::GetMainViewport()->WorkPos.y + style.margin_px;
    ImVec2 pivot(.5f, .0f);  // Top middle
    ImGui::SetNextWindowPos(pivot_pos, ImGuiCond_Always, pivot);
    if (ImGui::Begin("##PausedOverlay", nullptr,
                     style.window_flags | ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::TextUnformatted("Paused");
    }
    ImGui::End();
  }

  // Draw the percent real-time simulation speed overlay
  {
    float desired_realtime = physics_->GetStepControl().GetSpeed();
    float measured_realtime = physics_->GetStepControl().GetSpeedMeasured();
    float realtime_offset = mju_abs(measured_realtime - desired_realtime);

    // If running, check for misalignment of more than 10%
    bool misaligned =
        !physics_->IsPaused() && realtime_offset > 0.1 * desired_realtime;

    if (desired_realtime != 100.0 || misaligned) {
      ScopeOverlayStyle style(ui_.dark_mode);
      ImVec2 pivot_pos;
      pivot_pos.x = tmp_.pos_ui_lhs[0] + tmp_.size_ui_lhs[0] + style.margin_px;
      pivot_pos.y = tmp_.pos_ui_lhs[1] + style.margin_px;
      ImGui::SetNextWindowPos(pivot_pos, ImGuiCond_Always);
      if (ImGui::Begin(
              "##RealTime", nullptr,
              style.window_flags | ImGuiWindowFlags_AlwaysAutoResize)) {
        if (misaligned) {
          ImGui::Text("%g%% (%-4.1f%%)", desired_realtime, measured_realtime);
        } else {
          ImGui::Text("%g%%", desired_realtime);
        }
      }
      ImGui::End();
    }
  }

  // Draw the simulation error overlay
  if (!physics_->GetError().empty()) {
    ScopeOverlayStyle style(ui_.dark_mode);
    ImVec2 pivot_pos;
    pivot_pos.x = tmp_.pos_ui_lhs[0] + tmp_.size_ui_lhs[0] + style.margin_px;
    pivot_pos.y = ImGui::GetMainViewport()->WorkPos.y +
                  ImGui::GetMainViewport()->WorkSize.y - style.margin_px;
    ImVec2 pivot(0.f, 1.f);  // Bottom left
    ImGui::SetNextWindowPos(pivot_pos, ImGuiCond_Always, pivot);
    if (ImGui::Begin("##SimError", nullptr,
                     style.window_flags | ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("%s", std::string(physics_->GetError()).c_str());
    }
    ImGui::End();
  }

  if (tmp_.help) {
    ScopeOverlayStyle style(ui_.dark_mode);
    ImGui::SetNextWindowPos(
        ImVec2(tmp_.pos_ui_lhs[0] + tmp_.size_ui_lhs[0] + style.margin_px,
               tmp_.pos_ui_lhs[1] + style.margin_px),
        ImGuiCond_Always);
    // TODO: Make the width scale with the content/font size
    ImGui::SetNextWindowSize(ImVec2(350, 0), ImGuiCond_Always);
    if (ImGui::Begin("Help", &tmp_.help, style.window_flags)) {
      HelpGui();
    }
    ImGui::End();
  }
  if (tmp_.info) {
    ScopeOverlayStyle style(ui_.dark_mode);
    ImGui::SetNextWindowPos(
        ImVec2(tmp_.pos_ui_lhs[0] + tmp_.size_ui_lhs[0] + style.margin_px,
               tmp_.pos_ui_lhs[1] + tmp_.size_ui_lhs[1] - style.margin_px),
        ImGuiCond_Always, ImVec2(0, 1));
    // TODO: Make the width scale with the content/font size
    ImGui::SetNextWindowSize(ImVec2(190, 0), ImGuiCond_Always);
    if (ImGui::Begin("Info", &tmp_.info, style.window_flags)) {
      InfoGui();
    }
    ImGui::End();
  }

  ImGuiIO& io = ImGui::GetIO();
  if (io.WantSaveIniSettings) {
    SaveSettings();
    io.WantSaveIniSettings = false;
  }
}

void App::MainMenuGui() {
  if (ImGui::BeginMainMenuBar()) {
    // Add a button to toggle light/dark mode.
    // TODO: Move this to the right side of the menu bar.
    {
      struct ScopeStyle {
        ScopeStyle() {
          ImColor button_color = ImGui::GetStyle().Colors[ImGuiCol_Button];
          button_color.Value.w = 0.0f;

          ImGui::PushStyleColor(ImGuiCol_Button, (ImU32)button_color);
          ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImU32)button_color);
          ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImU32)button_color);
          ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 2));
          ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0);
        }
        ~ScopeStyle() {
          ImGui::PopStyleColor(3);
          ImGui::PopStyleVar(2);
        }
      } style;

      if (ImGui::Button(ui_.dark_mode ? ICON_DARKMODE : ICON_LIGHTMODE,
                        ImVec2(ImGui::GetFontSize(), ImGui::GetFontSize()))) {
        ui_.dark_mode = !ui_.dark_mode;
        if (ui_.dark_mode) {
          ImGui::StyleColorsDark();
        } else {
          ImGui::StyleColorsLight();
        }
      }
    }

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
    if (!model_file_.empty()) {
      if (ImGui::BeginMenu("Simulation")) {
        if (ImGui::MenuItem("Pause", "Space", physics_->IsPaused())) {
          physics_->TogglePause();
        }
        if (ImGui::MenuItem("Reset", "Backspace")) {
          physics_->Reset();
        }
        if (ImGui::MenuItem("Reload", "Ctrl+R")) {
          LoadModel(model_file_);
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

        if (ui_.classic_ui) {
          if (ImGui::MenuItem(
                  tmp_.show_ui_lhs ? "Hide Left UI" : "Show Left UI", "Tab")) {
            tmp_.show_ui_lhs = !tmp_.show_ui_lhs;
          }
          if (ImGui::MenuItem(
                  tmp_.show_ui_rhs ? "Hide Right UI" : "Show Right UI",
                  "Shift+Tab")) {
            tmp_.show_ui_rhs = !tmp_.show_ui_rhs;
          }
        } else {
          if (ImGui::MenuItem(tmp_.show_ui_lhs ? "Hide UI" : "Show UI",
                              "Tab")) {
            tmp_.show_ui_lhs = !tmp_.show_ui_lhs;
            tmp_.show_ui_rhs = !tmp_.show_ui_rhs;
          }
        }

        // Developer option to toggle between dockable window UI and sections UI
        if (ImGui::MenuItem("Classic UI", "", ui_.classic_ui)) {
          ui_.classic_ui = !ui_.classic_ui;
        }

        if (!ui_.classic_ui) {
          ImGui::Separator();
          if (ImGui::MenuItem("Simulation", "Alt+S", ui_.simulation)) {
            ToggleWindow(ui_.simulation);
          }
          if (ImGui::MenuItem("Physics", "Alt+P", ui_.physics)) {
            ToggleWindow(ui_.physics);
          }
          if (ImGui::MenuItem("Rendering", "Alt+R", ui_.rendering)) {
            ToggleWindow(ui_.rendering);
          }
          if (ImGui::MenuItem("Visualization", "Alt+V", ui_.visualization)) {
            ToggleWindow(ui_.visualization);
          }
          if (ImGui::MenuItem("Groups", "Alt+G", ui_.groups)) {
            ToggleWindow(ui_.groups);
          }
          if (ImGui::MenuItem("Joints", "Alt+J", ui_.joints)) {
            ToggleWindow(ui_.joints);
          }
          if (ImGui::MenuItem("Controls", "Alt+C", ui_.controls)) {
            ToggleWindow(ui_.controls);
          }
          if (ImGui::MenuItem("Watch", "Alt+W", ui_.watch)) {
            ToggleWindow(ui_.watch);
          }
          if (ImGui::MenuItem("Profiler", "", ui_.profiler)) {
            ToggleWindow(ui_.profiler);
            ClearProfilerData();
          }
          if (ImGui::MenuItem("Sensor", "", ui_.sensor)) {
            ToggleWindow(ui_.sensor);
          }
        }
        ImGui::EndMenu();
      }
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
    tmp_.filename[0] = 0;
  }
  if (tmp_.save_xml_popup) {
    ImGui::OpenPopup("SaveXML");
    tmp_.save_xml_popup = false;
    tmp_.filename[0] = 0;
  }
  if (tmp_.save_mjb_popup) {
    ImGui::OpenPopup("SaveMJB");
    tmp_.save_mjb_popup = false;
    tmp_.filename[0] = 0;
  }
  if (tmp_.save_screenshot_popup) {
    ImGui::OpenPopup("SaveWebp");
    tmp_.save_screenshot_popup = false;
    tmp_.filename[0] = 0;
  }
  if (tmp_.print_model_popup) {
    ImGui::OpenPopup("PrintModel");
    tmp_.print_model_popup = false;
    tmp_.filename[0] = 0;
  }
  if (tmp_.print_data_popup) {
    ImGui::OpenPopup("PrintData");
    tmp_.print_data_popup = false;
    tmp_.filename[0] = 0;
  }

  tmp_.modal_open =
      ImGui::IsPopupOpen("SaveXML") || ImGui::IsPopupOpen("SaveMJB") ||
      ImGui::IsPopupOpen("SaveWebp") || ImGui::IsPopupOpen("PrintModel") ||
      ImGui::IsPopupOpen("PrintData");

  if (ImGui::BeginPopupModal("LoadModel", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      LoadModel(tmp_.filename);
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveXML", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      char err[1000] = "";
      mj_saveLastXML(tmp_.filename, Model(), err, 1000);
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveMJB", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_saveModel(Model(), tmp_.filename, nullptr, 0);
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("SaveWebp", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      renderer_->SaveScreenshot(tmp_.filename, window_->GetWidth(),
                                window_->GetHeight());
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("PrintModel", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_printModel(Model(), tmp_.filename);
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopupModal("PrintData", NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    if (ImGui_FileDialog(tmp_.filename, sizeof(tmp_.filename))) {
      mj_printData(Model(), Data(), tmp_.filename);
    }
    ImGui::EndPopup();
  }
}

struct ScopeGreyText {
  ScopeGreyText(bool dark_mode) {
    const float v = dark_mode ? .6f : .9f;
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(v, v, v, 1.f));
  }
  ~ScopeGreyText() { ImGui::PopStyleColor(); }
};

void App::HelpGui() {
  ImGui::Columns(4);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.35f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.15f);
  ImGui::SetColumnWidth(2, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(3, ImGui::GetWindowWidth() * 0.1f);

  {
    ScopeGreyText style(ui_.dark_mode);
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
  }

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
  {
    ScopeGreyText style(ui_.dark_mode);
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
    ImGui::Text("Equality");
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
  }

  ImGui::NextColumn();
  ImGui::Text(",");
  ImGui::Text("K");
  ImGui::Text("`");
  ImGui::Text("\\");
  ImGui::Text("\"");
  ImGui::Text(";");
  ImGui::Text("U");
  ImGui::Text("Q");
  ImGui::Text("M");
  ImGui::Text("F");
  ImGui::Text("C");
  ImGui::Text("P");
  ImGui::Text("H");
  ImGui::Text("E");
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

void App::SimulationGui() {
  float margin = ImGui::GetStyle().ItemSpacing.x;
  float indent = ImGui::GetStyle().IndentSpacing;
  float window_width = ImGui::GetWindowWidth();
  float play_offset_x = (ui_.classic_ui ? 1.f : 2.f) * margin;
  if (ImGui::Button(physics_->IsPaused() ? ICON_PLAY : ICON_PAUSE,
                    ImVec2(window_width - play_offset_x, 40))) {
    physics_->TogglePause();
  }

  if (ImGui::TreeNodeEx("History")) {
    ImGui::PushItemWidth(window_width - 2 * margin - 2 * indent);
    const int max_history =
        std::min<int>(physics_->GetStepCount(), physics_->GetHistorySize());
    if (ImGui::SliderInt("##ScrubIndex", &ui_.scrub_idx, -max_history, 0)) {
      physics_->LoadHistory(ui_.scrub_idx);
    }
    ImGui::PopItemWidth();
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Keyframes")) {
    ImGui::SliderInt("Key", &ui_.key_idx, 0, Model()->nkey);
    if (ImGui::Button("Load")) {
      mj_resetDataKeyframe(Model(), Data(), ui_.key_idx);
      mj_forward(Model(), Data());
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
      mj_setKeyframe(Model(), Data(), ui_.key_idx);
    }
    if (ImGui::Button("Copy to Clipboard")) {
      std::string keyframe = toolbox::KeyframeToString(Model(), Data(), false);
      toolbox::MaybeSaveToClipboard(keyframe);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Noise")) {
    float noise_scale, noise_rate;
    physics_->GetStepControl().GetNoiseParameters(noise_scale, noise_rate);
    ImGui::SliderFloat("Scale", &noise_scale, 0, 1);
    ImGui::SliderFloat("Rate", &noise_rate, 0, 4);
    physics_->GetStepControl().SetNoiseParameters(noise_scale, noise_rate);
    ImGui::TreePop();
  }
}

void App::SensorGui() {}

void App::StateGui() {
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
  if (ImGui::BeginTable("##StateSignature", 2)) {
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
  const ImVec2 button_size(ImGui::CalcTextSize("Full Physics").x +
                               2 * ImGui::GetStyle().FramePadding.x,
                           0);
  const int button_columns =
      (ImGui::GetContentRegionAvail().x + ImGui::GetStyle().ItemSpacing.x) /
      (button_size.x + ImGui::GetStyle().ItemSpacing.x);

  if (ImGui::BeginTable("##CommonSignatures", button_columns,
                        ImGuiTableFlags_None, ImVec2(0, 0))) {
    for (int i = 0; i < button_columns; ++i) {
      ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed,
                              button_size.x);
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Physics", button_size)) {
      tmp_.state_sig =
          (tmp_.state_sig == mjSTATE_PHYSICS) ? 0 : mjSTATE_PHYSICS;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Full Physics", button_size)) {
      tmp_.state_sig =
          (tmp_.state_sig == mjSTATE_FULLPHYSICS) ? 0 : mjSTATE_FULLPHYSICS;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("User", button_size)) {
      tmp_.state_sig = (tmp_.state_sig == mjSTATE_USER) ? 0 : mjSTATE_USER;
    }
    ImGui::TableNextColumn();
    if (ImGui::Button("Integration", button_size)) {
      tmp_.state_sig =
          (tmp_.state_sig == mjSTATE_INTEGRATION) ? 0 : mjSTATE_INTEGRATION;
    }
    ImGui::EndTable();
  }

  if (tmp_.state_sig != prev_state_sig) {
    const int size = mj_stateSize(Model(), tmp_.state_sig);
    tmp_.state.resize(size);
  }

  if (tmp_.state.empty()) {
    // The state size is 0, let the user know why.
    ImGui::Separator();
    ImGui::BeginDisabled();
    ImGui::TextWrapped(
        tmp_.state_sig == 0
            ? "State array is empty because no state components are selected."
            : "State array is empty because the selected state components do "
              "not exist in the model.");
    ImGui::EndDisabled();
  } else {
    mj_getState(Model(), Data(), tmp_.state.data(), tmp_.state_sig);
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
            for (int local = 0; local < mj_stateSize(Model(), (1 << i));
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
      mj_setState(Model(), Data(), tmp_.state.data(), tmp_.state_sig);
    }
  }
}

void App::ProfilerGui() {
  const int plot_flags = 0;

  if (ImPlot::BeginPlot("CPU Time", ImVec2(-1, 0), plot_flags)) {
    ImPlot::SetupAxis(ImAxis_X1, "frame", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxis(ImAxis_Y1, "msec", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.2f");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine("total", cpu_total_.data(), cpu_total_.size(), 1,
                     -(int)cpu_total_.size());
    ImPlot::PlotLine("prepare", cpu_prepare_.data(), cpu_prepare_.size(), 1,
                     -(int)cpu_prepare_.size());
    ImPlot::PlotLine("solve", cpu_solve_.data(), cpu_solve_.size(), 1,
                     -(int)cpu_solve_.size());
    ImPlot::PlotLine("collision", cpu_collision_.data(), cpu_collision_.size(),
                     1, -(int)cpu_collision_.size());
    ImPlot::PlotLine("other", cpu_other_.data(), cpu_other_.size(), 1,
                     -(int)cpu_other_.size());
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("Dimensions", ImVec2(-1, 0), plot_flags)) {
    ImPlot::SetupAxis(ImAxis_X1, "frame", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxis(ImAxis_Y1, "count", ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisFormat(ImAxis_Y1, "%.0f");
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
    ImPlot::SetupFinish();

    ImPlot::PlotLine("dof", dim_dof_.data(), dim_dof_.size(), 1,
                     -(int)dim_dof_.size());
    ImPlot::PlotLine("body", dim_body_.data(), dim_body_.size(), 1,
                     -(int)dim_body_.size());
    ImPlot::PlotLine("constraint", dim_constraint_.data(),
                     dim_constraint_.size(), 1, -(int)dim_constraint_.size());
    ImPlot::PlotLine("sqrt(nnz)", dim_sqrt_nnz_.data(), dim_sqrt_nnz_.size(), 1,
                     -(int)dim_sqrt_nnz_.size());
    ImPlot::PlotLine("contact", dim_contact_.data(), dim_contact_.size(), 1,
                     -(int)dim_contact_.size());
    ImPlot::PlotLine("iteration", dim_iteration_.data(), dim_iteration_.size(),
                     1, -(int)dim_iteration_.size());
    ImPlot::EndPlot();
  }
}

void App::InfoGui() {
  const int num_islands = std::clamp(Data()->nisland, 1, mjNISLAND);

  // compute solver error (maximum over islands)
  mjtNum solver_err = 0;
  int solver_iter = 0;
  for (int i = 0; i < num_islands; i++) {
    solver_iter += Data()->solver_niter[i];

    mjtNum solerr_i = 0;
    if (Data()->solver_niter[i]) {
      const int ind = mjMIN(Data()->solver_niter[i], mjNSOLVER) - 1;
      const mjSolverStat* stat = Data()->solver + i * mjNSOLVER + ind;
      solerr_i = mju_min(stat->improvement, stat->gradient);
      if (solerr_i == 0) {
        solerr_i = mju_max(stat->improvement, stat->gradient);
      }
    }
    solver_err = mju_max(solver_err, solerr_i);
  }
  solver_err = mju_log10(mju_max(mjMINVAL, solver_err));

  auto type = physics_->IsPaused() ? mjTIMER_FORWARD : mjTIMER_STEP;
  auto cpu =
      Data()->timer[type].duration / mjMAX(1, Data()->timer[type].number);
  auto mempct = 100 * Data()->maxuse_arena / (double)(Data()->narena);
  auto memlimit = mju_writeNumBytes(Data()->narena);

  ImGui::Columns(2);
  ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.4f);
  ImGui::SetColumnWidth(1, ImGui::GetWindowWidth() * 0.6f);

  {
    ScopeGreyText style(ui_.dark_mode);
    ImGui::Text("Time");
    ImGui::Text("Size");
    ImGui::Text("CPU");
    ImGui::Text("Solver");
    ImGui::Text("FPS");
    ImGui::Text("Memory");
    if (Model()->opt.enableflags & mjENBL_ENERGY) {
      ImGui::Text("Energy");
    }
    if (Model()->opt.enableflags & mjENBL_FWDINV) {
      ImGui::Text("FwdInv");
    }
    if (!(Model()->opt.disableflags & mjDSBL_ISLAND)) {
      ImGui::Text("Islands");
    }
  }

  ImGui::NextColumn();
  ImGui::Text("%-9.3f", Data()->time);
  ImGui::Text("%d (%d con)", Data()->nefc, Data()->ncon);
  ImGui::Text("%.3f", cpu);
  ImGui::Text("%.1f (%d it)", solver_err, solver_iter);
  ImGui::Text("%0.1f", renderer_->GetFrameRate());
  ImGui::Text("%.1f%% of %s", mempct, memlimit);
  if (Model()->opt.enableflags & mjENBL_ENERGY) {
    ImGui::Text("%.3f", Data()->energy[0] + Data()->energy[1]);
  }
  if (Model()->opt.enableflags & mjENBL_FWDINV) {
    ImGui::Text("%.1f %.1f",
                mju_log10(mju_max(mjMINVAL, Data()->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL, Data()->solver_fwdinv[1])));
  }
  if (!(Model()->opt.disableflags & mjDSBL_ISLAND)) {
    ImGui::Text("%d", Data()->nisland);
  }
  ImGui::Columns();
}

void App::WatchGui() {
  ImGui::InputText("Field", ui_.watch_field, sizeof(ui_.watch_field));
  ImGui::InputInt("Index", &ui_.watch_index);
  const mjtNum* value = static_cast<const mjtNum*>(
      toolbox::GetValue(Model(), Data(), ui_.watch_field, ui_.watch_index));
  if (value) {
    ImGui::Text("%0.3f", *(const mjtNum*)value);
  } else {
    ImGui::Text("Invalid field/index!");
  }
}

void App::PhysicsGui() {
  auto& opt = Model()->opt;

  const char* opts0[] = {"Euler", "RK4", "implicit", "implicitfast"};
  ImGui::Combo("Integrator", &opt.integrator, opts0, IM_ARRAYSIZE(opts0));

  const char* opts1[] = {"Pyramidal", "Elliptic"};
  ImGui::Combo("Cone", &opt.cone, opts1, IM_ARRAYSIZE(opts1));

  const char* opts2[] = {"Dense", "Sparse", "Auto"};
  ImGui::Combo("Jacobian", &opt.jacobian, opts2, IM_ARRAYSIZE(opts2));

  const char* opts3[] = {"PGS", "CG", "Newton"};
  ImGui::Combo("Solver", &opt.solver, opts3, IM_ARRAYSIZE(opts3));

  if (ImGui::TreeNodeEx("Algorithmic Parameters",
                        ImGuiTreeNodeFlags_DefaultOpen)) {
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
  if (ImGui::TreeNodeEx("Disable Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (int i = 0; i < mjNDISABLE; ++i) {
      ToggleBit(mjDISABLESTRING[i], opt.disableflags, 1 << i);
      if (i % 2 == 0 && i != mjNDISABLE - 1) {
        ImGui::SameLine();
      }
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Enable Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (int i = 0; i < mjNENABLE; ++i) {
      ToggleBit(mjENABLESTRING[i], opt.enableflags, 1 << i);
      if (i % 2 == 0 && i != mjNENABLE - 1) {
        ImGui::SameLine();
      }
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Actuator Group Disable")) {
    ToggleBit("Act Group 0", opt.disableactuator, 1 << 0);
    ImGui::SameLine();
    ToggleBit("Act Group 1", opt.disableactuator, 1 << 1);
    ToggleBit("Act Group 2", opt.disableactuator, 1 << 2);
    ImGui::SameLine();
    ToggleBit("Act Group 3", opt.disableactuator, 1 << 3);
    ToggleBit("Act Group 4", opt.disableactuator, 1 << 4);
    ImGui::SameLine();
    ToggleBit("Act Group 5", opt.disableactuator, 1 << 5);
    ImGui::TreePop();
  };
}

void App::VisualizationGui() {
  auto& vis = Model()->vis;
  auto& stat = Model()->stat;
  if (ImGui::TreeNodeEx("Headlight")) {
    Toggle("Active", vis.headlight.active, ToggleKind::kSlider);
    // TODO: These are not working
    ImGui::ColorEdit3("Ambient", vis.headlight.ambient);
    ImGui::ColorEdit3("Diffuse", vis.headlight.diffuse);
    ImGui::ColorEdit3("Specular", vis.headlight.specular);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Free Camera")) {
    Toggle("Orthographic", vis.global.orthographic, ToggleKind::kSlider);
    ImGui_Input("FOV", &vis.global.fovy, {.format = "%0.2f"});
    ImGui_InputN("Center", stat.center, 3, {.format = "%0.2f"});
    ImGui_Input("Azimuth", &vis.global.azimuth, {.format = "%0.2f"});
    ImGui_Input("Elevation", &vis.global.elevation, {.format = "%0.2f"});
    if (ImGui::Button("Align")) {
      mjv_defaultFreeCamera(Model(), &camera_);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Global")) {
    ImGui_Input("Extent", &stat.extent);
    const char* opts[] = {"Box", "Ellipsoid"};
    ImGui::SliderInt("Inertia", &vis.global.ellipsoidinertia, 0, 1,
                     opts[vis.global.ellipsoidinertia]);
    Toggle("BVH active", vis.global.bvactive);
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Map")) {
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
    ImGui::ColorEdit4("fog", vis.rgba.fog);
    ImGui::ColorEdit4("haze", vis.rgba.haze);
    ImGui::ColorEdit4("force", vis.rgba.force);
    ImGui::ColorEdit4("inertia", vis.rgba.inertia);
    ImGui::ColorEdit4("joint", vis.rgba.joint);
    ImGui::ColorEdit4("actuator", vis.rgba.actuator);
    ImGui::ColorEdit4("actnegative", vis.rgba.actuatornegative);
    ImGui::ColorEdit4("actpositive", vis.rgba.actuatorpositive);
    ImGui::ColorEdit4("com", vis.rgba.com);
    ImGui::ColorEdit4("camera", vis.rgba.camera);
    ImGui::ColorEdit4("light", vis.rgba.light);
    ImGui::ColorEdit4("selectpoint", vis.rgba.selectpoint);
    ImGui::ColorEdit4("connect", vis.rgba.connect);
    ImGui::ColorEdit4("contactpoint", vis.rgba.contactpoint);
    ImGui::ColorEdit4("contactforce", vis.rgba.contactforce);
    ImGui::ColorEdit4("contactfriction", vis.rgba.contactfriction);
    ImGui::ColorEdit4("contacttorque", vis.rgba.contacttorque);
    ImGui::ColorEdit4("contactgap", vis.rgba.contactgap);
    ImGui::ColorEdit4("rangefinder", vis.rgba.rangefinder);
    ImGui::ColorEdit4("constraint", vis.rgba.constraint);
    ImGui::ColorEdit4("slidercrank", vis.rgba.slidercrank);
    ImGui::ColorEdit4("crankbroken", vis.rgba.crankbroken);
    ImGui::ColorEdit4("frustum", vis.rgba.frustum);
    ImGui::ColorEdit4("bv", vis.rgba.bv);
    ImGui::ColorEdit4("bvactive", vis.rgba.bvactive);
    ImGui::TreePop();
  }
}

void App::RenderingGui() {
  // Generate a list of camera names dynamically.
  std::vector<const char*> camera_names;
  camera_names.push_back("Free");
  camera_names.push_back("Tracking");
  for (int i = 0; i < Model()->ncam; i++) {
    if (Model()->names[Model()->name_camadr[i]]) {
      camera_names.push_back(Model()->names + Model()->name_camadr[i]);
    } else {
      camera_names.push_back("(Unnamed)");
    }
  }

  ImGui::Combo("Label", &vis_options_.label, kLabelNames,
               IM_ARRAYSIZE(kLabelNames));
  ImGui::Combo("Frame", &vis_options_.frame, kFrameNames,
               IM_ARRAYSIZE(kFrameNames));
  if (ImGui::Combo("Camera", &ui_.camera_idx, camera_names.data(),
                   camera_names.size())) {
    SetCamera(ui_.camera_idx);
  }
  if (ImGui::Button("Copy Camera")) {
    std::string camera_string = toolbox::CameraToString(Data(), &camera_);
    toolbox::MaybeSaveToClipboard(camera_string);
  }

  ImGui::SliderInt("Tree depth", &vis_options_.bvh_depth, 0, 20);
  ImGui::SliderInt("Flex layer", &vis_options_.flex_layer, 0, 10);

  if (ImGui::TreeNodeEx("Model Elements", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

    for (int i = 0; i < mjNVISFLAG; ++i) {
      Toggle(mjVISSTRING[i][0], vis_options_.flags[i]);
      if (i % 2 == 0 && i != mjNVISFLAG - 1) {
        ImGui::SameLine();
      }
    }
    ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Render Flags", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

    for (int i = 0; i < mjNRNDFLAG; ++i) {
      mjtByte flag = renderer_->GetFlag(static_cast<mjtRndFlag>(i));
      Toggle(mjRNDSTRING[i][0], flag);
      renderer_->SetFlag(static_cast<mjtRndFlag>(i), flag);

      if (i % 2 == 0 && i != mjNRNDFLAG - 1) {
        ImGui::SameLine();
      }
    }

    ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing() / 2);
    ImGui::TreePop();
  }
}

void App::GroupsGui() {
  auto GroupGui = [](const char* name, mjtByte* group) {
    if (ImGui::TreeNodeEx(name, ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing() / 2);

      for (int i = 0; i < 6; ++i) {
        char label[64];
        std::snprintf(label, sizeof(label), "%s %d", name, i);

        Toggle(label, group[i]);

        if (i % 2 == 0) ImGui::SameLine();
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

void App::JointsGui() {
  char name[100];
  for (int i = 0; i < Model()->njnt; ++i) {
    if (Model()->jnt_type[i] != mjJNT_HINGE &&
        Model()->jnt_type[i] != mjJNT_SLIDE) {
      continue;
    }
    const int group = std::clamp(Model()->jnt_group[i], 0, mjNGROUP - 1);
    if (!vis_options_.jointgroup[group]) {
      continue;
    }

    const char* jnt_name = Model()->names + Model()->name_jntadr[i];
    if (*jnt_name) {
      std::snprintf(name, sizeof(name), "%s", jnt_name);
    } else {
      std::snprintf(name, sizeof(name), "joint %d", i);
    }

    double min = -1.0;
    double max = 1.0;
    if (Model()->jnt_limited[i]) {
      min = Model()->jnt_range[2 * i + 0];
      max = Model()->jnt_range[2 * i + 1];
    } else if (Model()->jnt_type[i] == mjJNT_SLIDE) {
      min = -1.0;
      max = 1.0;
    } else {
      min = -3.1416;
      max = 3.1416;
    }

    const int data_adr = Model()->jnt_qposadr[i];
    ImGui_Slider(name, &Data()->qpos[data_adr], min, max);
  }
}

void App::ControlsGui() {
  if (ImGui::Button("Clear All")) {
    mju_zero(Data()->ctrl, Model()->nu);
  }

  char name[100];
  for (int i = 0; i < Model()->nu; i++) {
    int group = std::clamp(Model()->actuator_group[i], 0, mjNGROUP - 1);
    if (!vis_options_.actuatorgroup[group]) {
      continue;
    }
    if (group >= 0 && group <= 30 &&
        Model()->opt.disableactuator & (1 << group)) {
      continue;
    }

    const char* ctrl_name = Model()->names + Model()->name_actuatoradr[i];
    if (*ctrl_name) {
      std::snprintf(name, sizeof(name), "%s", ctrl_name);
    } else {
      std::snprintf(name, sizeof(name), "control %d", i);
    }

    double min = -1.0;
    double max = 1.0;
    if (!Model()->actuator_ctrllimited[i]) {
      min = Model()->actuator_ctrlrange[2 * i + 0];
      max = Model()->actuator_ctrlrange[2 * i + 1];
    }
    ImGui_Slider(name, &Data()->ctrl[i], min, max);
  }
}

App::UiState::Dict App::UiState::ToDict() const {
  return {
      {"Simulation", simulation ? "1" : "0"},
      {"Physics", physics ? "1" : "0"},
      {"Rendering", rendering ? "1" : "0"},
      {"Watch", watch ? "1" : "0"},
      {"Visualization", visualization ? "1" : "0"},
      {"Groups", groups ? "1" : "0"},
      {"Joints", joints ? "1" : "0"},
      {"Controls", controls ? "1" : "0"},
      {"Profiler", profiler ? "1" : "0"},
      {"Sensor", sensor ? "1" : "0"},
  };
}

void App::UiState::FromDict(const Dict& dict) {
  auto read_bool = [&](const char* key, bool& value) {
    auto iter = dict.find(key);
    if (iter != dict.end()) {
      value = iter->second == "1";
    }
  };

  *this = UiState();

  read_bool("Simulation", simulation);
  read_bool("Physics", physics);
  read_bool("Rendering", rendering);
  read_bool("Watch", watch);
  read_bool("Visualization", visualization);
  read_bool("Groups", groups);
  read_bool("Joints", joints);
  read_bool("Controls", controls);
  read_bool("Profiler", profiler);
  read_bool("Sensor", sensor);
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
