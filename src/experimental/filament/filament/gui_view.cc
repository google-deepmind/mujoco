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
#include <utility>
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
  renderable_ = em.create();
  view_->setScene(scene_);
  view_->setCamera(camera_);
  view_->setPostProcessingEnabled(false);

  material_ = object_mgr_->GetMaterial(ObjectManager::kUnlitUi);
}

GuiView::~GuiView() {
  if (num_elements_ > 0) {
    scene_->remove(renderable_);
    auto& rm = engine_->getRenderableManager();
    rm.destroy(renderable_);
  }
  auto& em = utils::EntityManager::get();
  em.destroy(renderable_);
  for (auto& buffer : buffers_) {
    engine_->destroy(buffer.vertex_buffer);
    engine_->destroy(buffer.index_buffer);
  }
  for (auto& instance : instances_) {
    engine_->destroy(instance);
  }
  for (auto& texture : textures_) {
    engine_->destroy(texture.second);
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

uintptr_t GuiView::UploadImage(uintptr_t tex_id, const uint8_t* pixels,
                               int width, int height, int bpp) {
  if (bpp != 4 && bpp != 3) {
    mju_error("Unsupported image bpp. Got %d, wanted 3 or 4", bpp);
  }

  const auto internal_format =
      bpp == 4 ? filament::Texture::InternalFormat::RGBA8
               : filament::Texture::InternalFormat::RGB8;
  const auto texture_format = bpp == 4 ? filament::Texture::Format::RGBA
                                       : filament::Texture::Format::RGB;

  filament::Engine* engine = object_mgr_->GetEngine();

  filament::Texture* texture = nullptr;
  if (tex_id == 0) {
    texture = filament::Texture::Builder()
                  .width(width)
                  .height(height)
                  .levels(1)
                  .format(internal_format)
                  .sampler(filament::Texture::Sampler::SAMPLER_2D)
                  .build(*engine);
    tex_id = textures_.size() + 1;
    textures_[tex_id] = texture;
  } else {
    auto iter = textures_.find(tex_id);
    if (iter == textures_.end()) {
      mju_error("Texture not found: %lu", tex_id);
    }
    texture = iter->second;

    if (pixels == nullptr) {
      // A nullptr implies that the user wants to destroy the texture.
      engine->destroy(texture);
      textures_.erase(tex_id);
      return 0;
    } else if (texture->getWidth() != width || texture->getHeight() != height) {
      // Recreate the texture if the dimensions have changed.
      engine->destroy(texture);
      texture = filament::Texture::Builder()
                    .width(width)
                    .height(height)
                    .levels(1)
                    .format(internal_format)
                    .sampler(filament::Texture::Sampler::SAMPLER_2D)
                    .build(*engine);
      textures_[tex_id] = texture;
    }
  }

  // Create a copy of the image to pass it to filament as we don't know the
  // lifetime of the data.
  const int num_bytes = width * height * bpp;
  std::byte* bytes = new std::byte[num_bytes];
  std::memcpy(bytes, pixels, num_bytes);
  const auto callback = [](void* buffer, size_t size, void* user) {
    auto* ptr = reinterpret_cast<std::byte*>(user);
    delete[] ptr;
  };
  filament::Texture::PixelBufferDescriptor pb(bytes, num_bytes, texture_format,
                                              filament::Texture::Type::UBYTE,
                                              callback);
  texture->setImage(*engine, 0, std::move(pb));
  return tex_id;
}

void GuiView::CreateTexture(ImTextureData* data) {
  filament::Engine* engine = object_mgr_->GetEngine();
  if (data->Format != ImTextureFormat_RGBA32) {
    mju_error("Unsupported texture format.");
  }

  filament::Texture* texture =
      filament::Texture::Builder()
          .width(data->Width)
          .height(data->Height)
          .levels(1)
          .format(filament::Texture::InternalFormat::RGBA8)
          .sampler(filament::Texture::Sampler::SAMPLER_2D)
          .build(*engine);

  const uintptr_t tex_id = textures_.size() + 1;
  textures_[tex_id] = texture;
  data->SetTexID((ImTextureID)tex_id);
  UpdateTexture(data);
}

void GuiView::UpdateTexture(ImTextureData* data) {
  const int size = data->Width * data->Height * 4;
  filament::Texture::PixelBufferDescriptor pb(data->GetPixels(), size,
                                              filament::Texture::Format::RGBA,
                                              filament::Texture::Type::UBYTE);
  auto iter = textures_.find(data->TexID);
  if (iter == textures_.end()) {
    mju_error("Texture not found: %llu", data->TexID);
  }

  filament::Texture* texture = iter->second;
  texture->setImage(*object_mgr_->GetEngine(), 0, std::move(pb));
  data->SetStatus(ImTextureStatus_OK);
}

void GuiView::DestroyTexture(ImTextureData* data) {
  auto iter = textures_.find(data->TexID);
  if (iter != textures_.end()) {
    object_mgr_->GetEngine()->destroy(iter->second);
    textures_.erase(data->TexID);
    data->SetTexID(ImTextureID_Invalid);
    data->SetStatus(ImTextureStatus_Destroyed);
  }
}

bool GuiView::PrepareRenderable() {
  // Prepare the imgui draw commands. We must call this function even if we do
  // not plan on rendering anything to ensure imgui state is updated.
  ImGui::Render();
  auto& rm = engine_->getRenderableManager();

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

  if (size.x == 0 || size.y == 0 || num_elements == 0) {
    return false;
  }

  view_->setViewport(
      filament::Viewport(0.f, 0.f, size.x * scale.x, size.y * scale.y));
  camera_->setProjection(filament::Camera::Projection::ORTHO, 0.0, size.x,
                         size.y, 0.0, 0.0, 1.0);

  if (num_elements != num_elements_) {
    if (num_elements_ > 0) {
      scene_->remove(renderable_);
      rm.destroy(renderable_);
    }

    num_elements_ = num_elements;

    filament::RenderableManager::Builder builder(num_elements_);
    builder.boundingBox({{-100, -100, -100}, {100, 100, 100}});
    builder.culling(false);
    builder.build(*engine_, renderable_);
    scene_->addEntity(renderable_);
  }

  for (auto& buffer : buffers_) {
    engine_->destroy(buffer.vertex_buffer);
    engine_->destroy(buffer.index_buffer);
  }
  buffers_.clear();

  auto ri = rm.getInstance(renderable_);

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
    buffers_.push_back(
        {CreateIndexBuffer<uint16_t>(engine_, cmds->IdxBuffer.Size, ifill),
         CreateVertexBuffer<GuiVertex>(engine_, cmds->VtxBuffer.Size, vfill)});
    const mujoco::FilamentBuffers& buffer = buffers_.back();

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

      mjrRect clip_rect{clip_left, clip_bottom, clip_width, clip_height};
      rm.setMaterialInstanceAt(
          ri, drawable_index,
          GetMaterialInstance(drawable_index, clip_rect, command.GetTexID()));
      rm.setGeometryAt(ri, drawable_index, kTriangles, buffer.vertex_buffer,
                       buffer.index_buffer, index_offset, command.ElemCount);
      rm.setBlendOrderAt(ri, drawable_index, drawable_index);

      index_offset += command.ElemCount;
      ++drawable_index;
    }
  }
  return true;
}

filament::MaterialInstance* GuiView::GetMaterialInstance(int index,
                                                         mjrRect rect,
                                                         uintptr_t texture_id) {
  while (index >= instances_.size()) {
    instances_.push_back(material_->createInstance());
  }

  auto iter = textures_.find(texture_id);
  if (iter == textures_.end()) {
    mju_error("Texture not found: %lu", texture_id);
  }

  filament::MaterialInstance* instance = instances_[index];
  instance->setParameter("glyph", iter->second, filament::TextureSampler());
  instance->setScissor(rect.left, rect.bottom, rect.width, rect.height);
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
