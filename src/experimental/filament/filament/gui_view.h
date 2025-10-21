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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_GUI_VIEW_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_GUI_VIEW_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Scene.h>
#include <filament/Texture.h>
#include <filament/View.h>
#include <mujoco/mjrender.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

// A filament::View that contains a filament::Scene used for rendering the GUI.
class GuiView {
 public:
  GuiView(filament::Engine* engine, ObjectManager* object_mgr);
  ~GuiView();

  // Prepares the UX scene renderable using data from the current ImGui state.
  // This function must be called once per frame to ensure ImGui state is
  // correctly synced.
  bool PrepareRenderable();

  // Returns the filament::View used to render the UX scene.
  filament::View* PrepareRenderView();

 private:
  // Returns the filament::MaterialInstance configured to draw into the given
  // scissor rect.
  filament::MaterialInstance* GetMaterialInstance(mjrRect rect);

  // Clears the filament::Scene of the UX renderable and releases all buffers.
  void ResetRenderable();

  ObjectManager* object_mgr_ = nullptr;
  filament::Engine* engine_ = nullptr;
  filament::Scene* scene_ = nullptr;
  filament::Camera* camera_ = nullptr;
  filament::View* view_ = nullptr;
  filament::Material* material_ = nullptr;
  utils::Entity renderable_;
  std::vector<FilamentBuffers> buffers_;
  std::unordered_map<uint64_t, filament::MaterialInstance*> instances_;
};

// Draws text at the given screen coordinates in clip space (i.e. [-1,-1,-1] to
// [1,1,1]).
void DrawTextAt(const char* text, float x, float y, float z);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_GUI_VIEW_H_
