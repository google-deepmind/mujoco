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

#include "experimental/filament/filament/imgui_bridge.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <imgui.h>
#include <math/mat3.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture.h"

namespace mujoco {

using filament::math::float3;
using filament::math::mat3f;

ImguiBridge::ImguiBridge(ObjectManager* object_mgr)
    : object_mgr_(object_mgr) {
  scene_view_ = std::make_unique<SceneView>(object_mgr_->GetEngine());
  scene_view_->DisableShadows();
  scene_view_->DisableReflections();
  scene_view_->DisablePostProcessing();
}

ImguiBridge::~ImguiBridge() { PrepareRenderables(0); }

uintptr_t ImguiBridge::UploadImage(uintptr_t tex_id, const uint8_t* pixels,
                                   int width, int height, int bpp) {
  if (bpp != 4 && bpp != 3) {
    mju_error("Unsupported image bpp. Got %d, wanted 3 or 4", bpp);
  }

  if (pixels == nullptr) {
    // If the pixels are nullptr, we destroy the texture.
    if (tex_id != 0) {
      textures_.erase(tex_id);
    }
    return 0;
  }

  // Assign a new texture ID.
  if (tex_id == 0) {
    tex_id = textures_.size() + 1;
  }

  std::unique_ptr<Texture>& texture = textures_[tex_id];

  // If the texture does not exist or the dimensions have changed, we create a
  // new texture.
  if (texture == nullptr || texture->GetWidth() != width ||
      texture->GetHeight() != height) {
    mjrTextureConfig config;
    mjr_defaultTextureConfig(&config);
    config.width = width;
    config.height = height;
    config.target = mjTEXTURE_2D;
    config.format = bpp == 4 ? mjPIXEL_FORMAT_RGBA8 : mjPIXEL_FORMAT_RGB8;
    config.color_space = mjCOLORSPACE_LINEAR;
    texture = std::make_unique<Texture>(scene_view_->GetEngine(), config);
  }

  // Create a copy of the image to pass it to filament as we don't know the
  // lifetime of the data.
  const size_t num_bytes = width * height * bpp;
  std::byte* bytes = new std::byte[num_bytes];
  const auto callback =
      +[](void* user) { delete[] reinterpret_cast<std::byte*>(user); };

  mjrTextureData texture_data;
  mjr_defaultTextureData(&texture_data);
  texture_data.bytes = bytes;
  texture_data.nbytes = num_bytes;
  texture_data.user_data = bytes;
  texture_data.release_callback = callback;

  std::memcpy(bytes, pixels, num_bytes);
  texture->Upload(texture_data);
  return tex_id;
}

void ImguiBridge::CreateTexture(ImTextureData* data) {
  if (data->Format != ImTextureFormat_RGBA32) {
    mju_error("Unsupported texture format.");
  }

  mjrTextureConfig config;
  mjr_defaultTextureConfig(&config);
  config.width = data->Width;
  config.height = data->Height;
  config.target = mjTEXTURE_2D;
  config.format = mjPIXEL_FORMAT_RGBA8;
  config.color_space = mjCOLORSPACE_LINEAR;

  const uintptr_t tex_id = textures_.size() + 1;
  textures_[tex_id] =
      std::make_unique<Texture>(scene_view_->GetEngine(), config);
  data->SetTexID((ImTextureID)tex_id);
  UpdateTexture(data);
}

void ImguiBridge::UpdateTexture(ImTextureData* data) {
  auto iter = textures_.find(data->TexID);
  if (iter == textures_.end()) {
    mju_error("Texture not found: %llu", data->TexID);
  }

  mjrTextureData texture_data;
  mjr_defaultTextureData(&texture_data);
  texture_data.bytes = data->GetPixels();
  texture_data.nbytes = data->Width * data->Height * 4;
  texture_data.user_data = nullptr;
  texture_data.release_callback = nullptr;
  iter->second->Upload(texture_data);
  data->SetStatus(ImTextureStatus_OK);
}

void ImguiBridge::DestroyTexture(ImTextureData* data) {
  auto iter = textures_.find(data->TexID);
  if (iter != textures_.end()) {
    textures_.erase(data->TexID);
    data->SetTexID(ImTextureID_Invalid);
    data->SetStatus(ImTextureStatus_Destroyed);
  }
}

void ImguiBridge::Update() {
  if (!ImGui::GetCurrentContext()) {
    PrepareRenderables(0);
    return;
  }

  // Prepare the imgui draw commands. We must call this function even if we do
  // not plan on rendering anything to ensure imgui state is updated.
  ImGui::Render();

  ImGuiIO& io = ImGui::GetIO();
  const ImVec2& size = io.DisplaySize;
  const ImVec2& scale = io.DisplayFramebufferScale;
  ImDrawData* commands = ImGui::GetDrawData();
  if (!commands || size.x == 0 || size.y == 0) {
    PrepareRenderables(0);
    return;
  }
  commands->ScaleClipRects(scale);

  // 2 floats for position, 2 floats for uv, 4 bytes for color.
  constexpr size_t kExpectedVertexSize =
      sizeof(float) * 4 + sizeof(uint8_t) * 4;

  int num_elements = 0;
  for (int n = 0; n < commands->CmdListsCount; ++n) {
    const ImDrawList* cmds = commands->CmdLists[n];
    if (kExpectedVertexSize != sizeof(cmds->VtxBuffer.Data[0])) {
      mju_error("Invalid vertex buffer size.");
    }
    if (sizeof(uint16_t) != sizeof(cmds->IdxBuffer.Data[0])) {
      mju_error("Invalid index buffer size.");
    }
    num_elements += cmds->CmdBuffer.size();
  }

  if (commands->Textures != nullptr) {
    for (ImTextureData* tex : *commands->Textures) {
      if (tex->Status == ImTextureStatus_OK) {
        // ImGui's lifecycle is independent of the filament context lifecycle.
        // As such, it is possible to destroy and create a new filament context
        // while ImGui is still expecting the "OK" textures to work. In this
        // case, we simply recreate the texture.
        if (textures_.find(tex->TexID) == textures_.end()) {
          CreateTexture(tex);
        }
      } else if (tex->Status == ImTextureStatus_WantCreate) {
        CreateTexture(tex);
      } else if (tex->Status == ImTextureStatus_WantUpdates) {
        if (textures_.find(tex->TexID) == textures_.end()) {
          CreateTexture(tex);
        } else {
          UpdateTexture(tex);
        }
      } else if (tex->Status == ImTextureStatus_WantDestroy &&
                 tex->UnusedFrames > 0) {
        DestroyTexture(tex);
      }
    }
  }

  PrepareRenderables(num_elements);
  if (num_elements == 0) {
    return;
  }

  meshes_.clear();
  int renderable_index = 0;
  for (int n = 0; n < commands->CmdListsCount; ++n) {
    const ImDrawList* cmds = commands->CmdLists[n];

    MeshData data;
    DefaultMeshData(&data);
    data.nattributes = 3;
    data.attributes[0].usage = mjVERTEX_ATTRIBUTE_POSITION;
    data.attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
    data.attributes[0].bytes = cmds->VtxBuffer.Data;
    data.attributes[1].usage = mjVERTEX_ATTRIBUTE_UV;
    data.attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
    data.attributes[1].bytes = cmds->VtxBuffer.Data + sizeof(float) * 2;
    data.attributes[2].usage = mjVERTEX_ATTRIBUTE_COLOR;
    data.attributes[2].type = mjVERTEX_ATTRIBUTE_TYPE_UBYTE4;
    data.attributes[2].bytes = cmds->VtxBuffer.Data + sizeof(float) * 4;
    data.interleaved = true;
    data.nvertices = cmds->VtxBuffer.Size;
    data.nindices = cmds->IdxBuffer.Size;
    data.indices = cmds->IdxBuffer.Data;
    data.index_type = mjINDEX_TYPE_USHORT;
    data.primitive_type = mjPRIM_TYPE_TRIANGLES;
    meshes_.push_back(std::make_unique<Mesh>(scene_view_->GetEngine(), data));

    const Mesh* mesh = meshes_.back().get();

    int index_offset = 0;
    for (const ImDrawCmd& command : cmds->CmdBuffer) {
      const int width = size.x * scale.x;
      const int height = size.y * scale.y;

      auto& renderable = renderables_[renderable_index];
      renderable->SetMesh(mesh, index_offset, command.ElemCount);

      MaterialTextures textures;
      textures.color = textures_[command.GetTexID()].get();

      MaterialParams properties;
      properties.scissor[0] = command.ClipRect.x;
      properties.scissor[1] = height - command.ClipRect.w;
      properties.scissor[2] = command.ClipRect.z - command.ClipRect.x;
      properties.scissor[3] = command.ClipRect.w - command.ClipRect.y;
      // Modal dialogs try to cover the whole window, but also a little outside
      // of it. This doesn't work well with filament's scissor test, so we clip
      // them to the window.
      if (properties.scissor[0] < 0 || properties.scissor[1] < 0) {
        properties.scissor[0] = 0;
        properties.scissor[1] = 0;
        properties.scissor[2] = width;
        properties.scissor[3] = height;
      }
      renderable->UpdateMaterial(properties, textures);
      renderable->SetTransform(
          {float3{0, 0, 0}, mat3f(), float3(scale.x, scale.y, 1.0f)});

      index_offset += command.ElemCount;
      ++renderable_index;
    }
  }
}

void ImguiBridge::PrepareRenderables(int count) {
  while (renderables_.size() < count) {
    RenderableParams config;
    DefaultRenderableParams(&config);
    config.shading_model = ShadingModel::Ux;
    auto& r = renderables_.emplace_back(
        std::make_unique<Renderable>(object_mgr_, config));
    r->SetCastShadows(false);
    r->SetReceiveShadows(false);
    r->SetBlendOrder(static_cast<std::uint16_t>(renderables_.size()));
    scene_view_->AddToScene(r.get());
  }
  while (renderables_.size() > count) {
    scene_view_->RemoveFromScene(renderables_.back().get());
    renderables_.pop_back();
  }
}

static ImVec2 ClipSpaceToWindowCoordinates(float x, float y) {
  const ImVec2& display_size = ImGui::GetIO().DisplaySize;
  const float pos_x = display_size.x * ((x + 1) * 0.5f);
  const float pos_y = display_size.y * (1.0f - ((y + 1) * 0.5f));
  return ImVec2(pos_x, pos_y);
}

void DrawTextAt(const char* text, float x, float y, float z) {
  if (x < -1 || y < -1 || x > 1 || y > 1 || z < -1 || z > 1) {
    return;
  }

  const ImVec2 center_pos = ClipSpaceToWindowCoordinates(x, y);
  const ImVec2 size = ImGui::CalcTextSize(text);

  const ImVec2 pos = ImVec2(center_pos.x - size.x / 2, center_pos.y);
  const ImVec2 shadow_pos = ImVec2(pos.x + 2, pos.y + 2);
  const int flags = ImGuiWindowFlags_NoBringToFrontOnFocus |
                    ImGuiWindowFlags_NoFocusOnAppearing |
                    ImGuiWindowFlags_NoBackground |
                    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
                    ImGuiWindowFlags_NoNav;

  ImGui::Begin("labels", nullptr, flags);
  ImGui::BeginChild("labels", ImGui::GetIO().DisplaySize, 0, flags);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddText(shadow_pos, IM_COL32_BLACK, text);
  draw_list->AddText(pos, IM_COL32_WHITE, text);
  ImGui::EndChild();
  ImGui::End();
}

}  // namespace mujoco
