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

#include "experimental/platform/picture_gui.h"

#include <cstddef>
#include <vector>

#include <imgui.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/imgui_widgets.h"
#include "experimental/platform/renderer.h"
#include "experimental/platform/window.h"

namespace mujoco::platform {

// Returns false if the user requests that this picture-in-picture widget be
// removed from the GUI.
static bool PipGuiImpl(const mjModel* model, mjData* data,
                       platform::Window* window, platform::Renderer* renderer,
                       PipState* pip) {
  bool result = true;

  auto get_camera_name = [model](int i) -> const char* {
    if (model->names[model->name_camadr[i]]) {
      return model->names + model->name_camadr[i];
    } else {
      return "Unnamed";
    }
  };

  const int width = ImGui::GetContentRegionAvail().x;
  const int height = width / window->GetAspectRatio();
  std::vector<std::byte> output(width * height * 3);

  const int combo_width = (width-30) / 2;

  ImGui::PushID(pip);
  ImGui::SetNextItemWidth(combo_width);
  if (ImGui::BeginCombo("##PipCamera", get_camera_name(pip->camera))) {
    for (int i = 0; i < model->ncam; i++) {
      if (ImGui::Selectable(get_camera_name(i), (pip->camera == i))) {
        pip->camera = i;
      }
    }
    ImGui::EndCombo();
  }

  mjvCamera camera;
  mjv_defaultCamera(&camera);
  camera.type = mjCAMERA_FIXED;
  camera.fixedcamid = pip->camera;

  const char* mode_names[] = {"Color", "Depth", "Segmentation"};
  ImGui::SameLine();
  ImGui::SetNextItemWidth(combo_width);
  if (ImGui::BeginCombo("##PipMode", mode_names[pip->mode])) {
    for (int i = 0; i < 3; i++) {
      if (ImGui::Selectable(mode_names[i], (pip->mode == i))) {
        pip->mode = static_cast<PipState::Mode>(i);
      }
    }
    ImGui::EndCombo();
  }

  ImGui::SameLine();
  if (ImGui::Button(ICON_FA_TRASH_CAN)) {
    result = false;
  }

  mjtByte* flags = renderer->GetRenderFlags();
  const int prev_depth = flags[mjRND_DEPTH];
  const int prev_segment = flags[mjRND_SEGMENT];
  flags[mjRND_DEPTH] = (pip->mode == PipState::Depth) ? 1 : 0;
  flags[mjRND_SEGMENT] = (pip->mode == PipState::Segmentation) ? 1 : 0;
  renderer->RenderToTexture(model, data, &camera, width, height, output.data());
  pip->texture =
      renderer->UploadImage(pip->texture, output.data(), width, height, 3);

  // Restore previous render flags.
  flags[mjRND_DEPTH] = prev_depth;
  flags[mjRND_SEGMENT] = prev_segment;

  ImGui::Image(pip->texture, {(float)width, (float)height});
  ImGui::PopID();
  return result;
}

void PipGui(const mjModel* model, mjData* data, platform::Window* window,
            platform::Renderer* renderer, std::vector<PipState>* pips) {
  if (pips->empty()) {
    pips->emplace_back();
  }

  std::vector<int> to_delete;
  for (int i = 0; i < pips->size(); ++i) {
    PipState& pip = pips->at(i);
    if (PipGuiImpl(model, data, window, renderer, &pip) == false) {
      to_delete.push_back(i);
    };
    ImGui::Separator();
  }
  for (int i = to_delete.size() - 1; i >= 0; --i) {
    pips->erase(pips->begin() + to_delete[i]);
  }
  if (ImGui::Button("+")) {
    pips->emplace_back();
  };
}

}  // namespace mujoco::platform
