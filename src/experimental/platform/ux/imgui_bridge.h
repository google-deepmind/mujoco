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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_UX_IMGUI_BRIDGE_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_UX_IMGUI_BRIDGE_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <imgui.h>
#include <mujoco/mjrfilament.h>
#include "render/filament/mjrfilament_cpp.h"

namespace mujoco {

// Creates and manages a SceneView using data read from ImGui.
class ImguiBridge {
 public:
  explicit ImguiBridge(mjrfContext* ctx);
  ~ImguiBridge();

  // Prepares the Renderables using data from the current ImGui state. This
  // function must be called once per frame to ensure ImGui state is correctly
  // synced.
  void Update();

  // Returns the managed UX scene.
  mjrfScene* GetScene() const;
  mjrCamera GetCamera(int width, int height) const;

  // Uploads texture to be used with ImGui's Image and ImageButton functions.
  uintptr_t UploadImage(uintptr_t tex_id, const uint8_t* pixels, int width,
                        int height, int bpp);

  ImguiBridge(const ImguiBridge&) = delete;
  ImguiBridge& operator=(const ImguiBridge&) = delete;

 private:
  // Ensures exactly `count` Renderables exist, creating or destroying them as
  // needed.
  void PrepareRenderables(int count);

  void CreateTexture(ImTextureData* data);
  void UpdateTexture(ImTextureData* data);
  void DestroyTexture(ImTextureData* data);
  mjrfTexture* GetTexture(uintptr_t tex_id) const;

  mjrfContext* ctx_ = nullptr;
  UniquePtr<mjrfScene> scene_{nullptr, nullptr};
  std::vector<UniquePtr<mjrfRenderable>> renderables_;
  std::vector<UniquePtr<mjrfMesh>> meshes_;
  std::unordered_map<uintptr_t, UniquePtr<mjrfTexture>> textures_;
  uintptr_t next_tex_id_ = 1;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_UX_IMGUI_BRIDGE_H_
