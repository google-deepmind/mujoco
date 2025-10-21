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

#include "experimental/filament/filament/gui_view.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <imgui.h>
#include <filament/Engine.h>
#include <filament/RenderableManager.h>
#include <filament/TextureSampler.h>
#include <filament/Viewport.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mjrender.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/vertex_util.h"

namespace mujoco {

using filament::math::float4;

static constexpr auto kTriangles =
    filament::RenderableManager::PrimitiveType::TRIANGLES;

GuiView::GuiView(filament::Engine* engine, ObjectManager* object_mgr)
    : object_mgr_(object_mgr), engine_(engine) {
  auto& em = utils::EntityManager::get();
  scene_ = engine_->createScene();
  camera_ = engine_->createCamera(em.create());
  view_ = engine_->createView();
  view_->setScene(scene_);
  view_->setCamera(camera_);
  view_->setPostProcessingEnabled(false);

  material_ = object_mgr_->GetMaterial(ObjectManager::kUnlitUi);

  // Upload the ImGui font as a texture that is sampled by the UX material.
  int font_width = 0;
  int font_height = 0;
  int font_bpp = 0;
  unsigned char* pixels = nullptr;
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->GetTexDataAsRGBA32(&pixels, &font_width, &font_height, &font_bpp);
  object_mgr_->UploadFont(pixels, font_width, font_height, 0);
}

GuiView::~GuiView() {
  ResetRenderable();
  for (auto& instance : instances_) {
    engine_->destroy(instance.second);
  }
  engine_->destroyCameraComponent(camera_->getEntity());
  engine_->destroy(view_);
  engine_->destroy(scene_);
}

void GuiView::ResetRenderable() {
  auto& em = utils::EntityManager::get();
  if (!renderable_.isNull()) {
    scene_->remove(renderable_);
    auto& rm = engine_->getRenderableManager();
    rm.destroy(renderable_);
    em.destroy(renderable_);
    renderable_ = utils::Entity();
  }

  for (auto& buffer : buffers_) {
    engine_->destroy(buffer.vertex_buffer);
    engine_->destroy(buffer.index_buffer);
  }
  buffers_.clear();
}

bool GuiView::PrepareRenderable() {
  ResetRenderable();

  // Prepare the imgui draw commands. We must call this function even if we do
  // not plan on rendering anything to ensure imgui state is updated.
  ImGui::Render();

  ImGuiIO& io = ImGui::GetIO();
  const ImVec2& size = io.DisplaySize;
  const ImVec2& scale = io.DisplayFramebufferScale;
  ImDrawData* commands = ImGui::GetDrawData();
  commands->ScaleClipRects(scale);

  int num_elements = 0;
  for (int n = 0; n < commands->CmdListsCount; ++n) {
    const ImDrawList* cmds = commands->CmdLists[n];
    if (sizeof(GuiVertex) != sizeof(cmds->VtxBuffer.Data[0])) {
      mju_error("Invalid vertex buffer size.");
    }
    if (sizeof(uint16_t) != sizeof(cmds->IdxBuffer.Data[0])) {
      mju_error("Invalid index buffer size.");
    }
    num_elements += cmds->CmdBuffer.size();
  }

  if (size.x == 0 || size.y == 0 || num_elements == 0) {
    return false;
  }

  view_->setViewport(
      filament::Viewport(0.f, 0.f, size.x * scale.x, size.y * scale.y));
  camera_->setProjection(filament::Camera::Projection::ORTHO, 0.0, size.x,
                         size.y, 0.0, 0.0, 1.0);

  filament::RenderableManager::Builder builder(num_elements);
  builder.boundingBox({{-100, -100, -100}, {100, 100, 100}});
  builder.culling(false);

  int drawable_index = 0;
  for (int n = 0; n < commands->CmdListsCount; ++n) {
    const ImDrawList* cmds = commands->CmdLists[n];
    auto vfill = [&](std::byte* dst, std::size_t size) {
      if (size != cmds->VtxBuffer.size_in_bytes()) {
        mju_error("Invalid vertex buffer size.");
      }
      std::memcpy(dst, cmds->VtxBuffer.Data, size);
    };
    auto ifill = [&](std::byte* dst, std::size_t size) {
      if (size != cmds->IdxBuffer.size_in_bytes()) {
        mju_error("Invalid index buffer size.");
      }
      std::memcpy(dst, cmds->IdxBuffer.Data, size);
    };
    const mujoco::FilamentBuffers& buffer = buffers_.emplace_back(
        CreateIndexBuffer<uint16_t>(engine_, cmds->IdxBuffer.Size, ifill),
        CreateVertexBuffer<GuiVertex>(engine_, cmds->VtxBuffer.Size, vfill));

    int index_offset = 0;
    for (const ImDrawCmd& command : cmds->CmdBuffer) {
      const int width = size.x * scale.x;
      const int height = size.y * scale.y;

      int clip_left = command.ClipRect.x;
      int clip_bottom = height - command.ClipRect.w;
      int clip_width = command.ClipRect.z - command.ClipRect.x;
      int clip_height = command.ClipRect.w - command.ClipRect.y;
      // Modal dialogs try to cover the whole window, but also a little outside
      // of it. This doesn't work well with filament's scissor test, so we clip
      // them to the window.
      if (clip_left < 0 || clip_bottom < 0) {
        clip_left = 0;
        clip_bottom = 0;
        clip_width = width;
        clip_height = height;
      }

      mjrRect clip_rect(clip_left, clip_bottom, clip_width, clip_height);
      builder.material(drawable_index, GetMaterialInstance(clip_rect));
      builder.geometry(drawable_index, kTriangles, buffer.vertex_buffer,
                      buffer.index_buffer, index_offset,
                      command.ElemCount);
      builder.blendOrder(drawable_index, drawable_index);

      index_offset += command.ElemCount;
      ++drawable_index;
    }
  }

  auto& em = utils::EntityManager::get();
  renderable_ = em.create();
  builder.build(*engine_, renderable_);
  scene_->addEntity(renderable_);
  return true;
}

// Maps a rect into a 64-bit key to assist with lookup.
static uint64_t MakeRectKey(mjrRect rect) {
  return static_cast<uint64_t>(rect.left & 0xff) << 24 |
         static_cast<uint64_t>(rect.bottom & 0xff) << 16 |
         static_cast<uint64_t>(rect.width & 0xff) << 8 |
         static_cast<uint64_t>(rect.height & 0xff);
}

filament::MaterialInstance* GuiView::GetMaterialInstance(mjrRect rect) {
  const uint64_t key = MakeRectKey(rect);
  auto iter = instances_.find(key);
  if (iter != instances_.end()) {
    return iter->second;
  }

  auto instance = material_->createInstance();
  instance->setScissor(rect.left, rect.bottom, rect.width, rect.height);
  filament::TextureSampler sampler;
  instance->setParameter("glyph", object_mgr_->GetFont(0), sampler);
  instances_[key] = instance;
  return instance;
}

filament::View* GuiView::PrepareRenderView() { return view_; }

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
                    ImGuiWindowFlags_NoDecoration |
                    ImGuiWindowFlags_NoInputs |
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
