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

#include "experimental/toolbox/interaction.h"
#include <algorithm>

#include <imgui.h>
#include "experimental/toolbox/imgui_widgets.h"
#include "experimental/toolbox/physics.h"
#include "experimental/toolbox/renderer.h"
#include "experimental/toolbox/window.h"
#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

static void ToggleFlag(mjtByte& flag) { flag = flag ? 0 : 1; }

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

PickResult Pick(float x, float y, Window* window, Renderer* renderer,
                Physics* physics, const mjvOption& vis_options) {
  const float w = static_cast<float>(window->GetWidth());
  const float h = static_cast<float>(window->GetHeight());
  const float aspect_ratio = w / h;

  PickResult result;
  result.body =
      mjv_select(physics->GetModel(), physics->GetData(), &vis_options,
                 aspect_ratio, x, 1.0f - y, &renderer->GetScene(), result.point,
                 &result.geom, &result.flex, &result.skin);
  return result;
}

int SetCamera(const mjModel& model, mjvCamera& camera, int request_idx) {
  // 0 = free, 1 = tracking, 2+ = fixed
  int camera_idx = std::clamp(request_idx, 0, std::max(model.ncam + 1, 0));
  if (camera_idx == 0) {
    camera.type = mjCAMERA_FREE;
  } else if (camera_idx == 1) {
    if (camera.trackbodyid >= 0) {
      camera.type = mjCAMERA_TRACKING;
      camera.fixedcamid = -1;
    } else {
      camera.type = mjCAMERA_FREE;
      camera_idx = 0;
    }
  } else {
    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = camera_idx - 2;
  }

  return camera_idx;
}

void HandleMouseEvents(Window* window, Renderer* renderer,
                       Physics* physics, mjvPerturb& perturb,
                       mjvOption& vis_options, mjvCamera& camera,
                       int& camera_idx) {
  auto& io = ImGui::GetIO();

  if (io.WantCaptureMouse) {
    return;
  }

  // Normalize mouse positions and movement to display size.
  const float mouse_x = io.MousePos.x / io.DisplaySize.x;
  const float mouse_y = io.MousePos.y / io.DisplaySize.y;
  const float mouse_dx = io.MouseDelta.x / io.DisplaySize.x;
  const float mouse_dy = io.MouseDelta.y / io.DisplaySize.y;
  const float mouse_scroll = io.MouseWheel / 50.0f;

  mjModel* model = physics->GetModel();
  mjData* data = physics->GetData();
  mjvScene& scene = renderer->GetScene();

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
    perturb.active = 0;
  }

  // Mouse scroll.
  if (model && mouse_scroll != 0.0f) {
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, mouse_scroll, &scene, &camera);
  }

  // Mouse drag.
  if (model && data && action != mjMOUSE_NONE &&
      (mouse_dx != 0.0f || mouse_dy != 0.0f)) {
    // If ctrl is pressed, move the perturbation, otherwise move the camera.
    if (io.KeyCtrl) {
      if (perturb.select > 0) {
        const int active =
            action == mjMOUSE_MOVE_V ? mjPERT_TRANSLATE : mjPERT_ROTATE;
        if (active != perturb.active) {
          mjv_initPerturb(model, data, &scene, &perturb);
          perturb.active = active;
        }
        mjv_movePerturb(model, data, action, mouse_dx, mouse_dy, &scene,
                        &perturb);
      }
    } else {
      mjv_moveCamera(model, action, mouse_dx, mouse_dy, &scene, &camera);
    }
  }

  // Left double click.
  if (data && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
    PickResult picked =
        Pick(mouse_x, mouse_y, window, renderer, physics, vis_options);
    if (picked.body >= 0) {
      perturb.select = picked.body;
      perturb.flexselect = picked.flex;
      perturb.skinselect = picked.skin;

      // Compute the local position of the selected object in the world.
      mjtNum tmp[3];
      mju_sub3(tmp, picked.point, data->xpos + 3 * picked.body);
      mju_mulMatTVec(perturb.localpos, data->xmat + 9 * picked.body, tmp, 3, 3);
    } else {
      perturb.select = 0;
      perturb.flexselect = -1;
      perturb.skinselect = -1;
    }
  }

  // Right double click.
  if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right)) {
    PickResult picked =
        Pick(mouse_x, mouse_y, window, renderer, physics, vis_options);
    mju_copy3(camera.lookat, picked.point);
    if (picked.body > 0 && io.KeyCtrl) {
      camera.type = mjCAMERA_TRACKING;
      camera.trackbodyid = picked.body;
      camera.fixedcamid = -1;
      camera_idx = 1;
    }
  }
}

void HandleKeyboardEvents(Window* window, Renderer* renderer,
                          Physics* physics, mjvPerturb& perturb,
                          mjvOption& vis_options, mjvCamera& camera,
                          int& camera_idx) {
  if (ImGui::GetIO().WantCaptureKeyboard) {
    return;
  }

  mjModel* model = physics->GetModel();

  // Physics control shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_Space)) {
    physics->TogglePause();
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backspace)) {
    physics->Reset();
  }

  // Camera shortcuts.
  if (model) {
    if (ImGui_IsChordJustPressed(ImGuiKey_Escape)) {
      camera_idx = SetCamera(*model, camera, 0);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_LeftBracket)) {
      camera_idx = SetCamera(*model, camera, camera_idx - 1);
    } else if (ImGui_IsChordJustPressed(ImGuiKey_RightBracket)) {
      camera_idx = SetCamera(*model, camera, camera_idx + 1);
    }
  }

  // Perturb shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_PageUp)) {
    SelectParentPerturb(model, perturb);
  }

  // Visualization shortcuts.
  if (ImGui_IsChordJustPressed(ImGuiKey_F6)) {
    vis_options.frame = (vis_options.frame + 1) % mjNFRAME;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F7)) {
    vis_options.label = (vis_options.label + 1) % mjNLABEL;
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Comma)) {
    ToggleFlag(vis_options.flags[mjVIS_ACTIVATION]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Backslash)) {
    ToggleFlag(vis_options.flags[mjVIS_MESHBVH]);
    // } else if (ImGui_IsChordJustPressed(ImGuiKey_Backquote)) {
    //   ToggleFlag(vis_options.flags[mjVIS_BODYBVH]);
    // } else if (ImGui_IsChordJustPressed(ImGuiKey_Quote)) {
    //   ToggleFlag(vis_options.flags[mjVIS_SCLINERTIA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Semicolon)) {
    ToggleFlag(vis_options.flags[mjVIS_SKIN]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_U)) {
    ToggleFlag(vis_options.flags[mjVIS_ACTUATOR]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Q)) {
    ToggleFlag(vis_options.flags[mjVIS_CAMERA]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_M)) {
    ToggleFlag(vis_options.flags[mjVIS_COM]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_F)) {
    ToggleFlag(vis_options.flags[mjVIS_CONTACTFORCE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_C)) {
    ToggleFlag(vis_options.flags[mjVIS_CONTACTPOINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_P)) {
    ToggleFlag(vis_options.flags[mjVIS_CONTACTSPLIT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_H)) {
    ToggleFlag(vis_options.flags[mjVIS_CONVEXHULL]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_E)) {
    ToggleFlag(vis_options.flags[mjVIS_CONSTRAINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_I)) {
    ToggleFlag(vis_options.flags[mjVIS_ISLAND]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_J)) {
    ToggleFlag(vis_options.flags[mjVIS_JOINT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Z)) {
    ToggleFlag(vis_options.flags[mjVIS_LIGHT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_B)) {
    ToggleFlag(vis_options.flags[mjVIS_PERTFORCE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_O)) {
    ToggleFlag(vis_options.flags[mjVIS_PERTOBJ]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_Y)) {
    ToggleFlag(vis_options.flags[mjVIS_RANGEFINDER]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_V)) {
    ToggleFlag(vis_options.flags[mjVIS_TENDON]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_X)) {
    ToggleFlag(vis_options.flags[mjVIS_TEXTURE]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_T)) {
    ToggleFlag(vis_options.flags[mjVIS_TRANSPARENT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_K)) {
    ToggleFlag(vis_options.flags[mjVIS_AUTOCONNECT]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_G)) {
    ToggleFlag(vis_options.flags[mjVIS_STATIC]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_0 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options.sitegroup[0]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_1 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options.sitegroup[1]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_2 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options.sitegroup[2]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_3 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options.sitegroup[3]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_4 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options.sitegroup[4]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_5 | ImGuiMod_Shift)) {
    ToggleFlag(vis_options.sitegroup[5]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_0)) {
    ToggleFlag(vis_options.geomgroup[0]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_1)) {
    ToggleFlag(vis_options.geomgroup[1]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_2)) {
    ToggleFlag(vis_options.geomgroup[2]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_3)) {
    ToggleFlag(vis_options.geomgroup[3]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_4)) {
    ToggleFlag(vis_options.geomgroup[4]);
  } else if (ImGui_IsChordJustPressed(ImGuiKey_5)) {
    ToggleFlag(vis_options.geomgroup[5]);
  }
}

}  // namespace mujoco::toolbox
