// Copyright 2026 DeepMind Technologies Limited
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

#include "src/render/filament/support/renderer.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "src/render/filament/support/mjrf_ptr.h"
#include "src/render/filament/support/model_resources.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using RendererTest = MujocoTest;

static constexpr uint8_t kRed[3] = {255, 0, 0};
static constexpr uint8_t kGreen[3] = {0, 255, 0};
static constexpr uint8_t kBlue[3] = {0, 0, 255};
static constexpr uint8_t kWhite[3] = {255, 255, 255};

static UniquePtr<mjrfTexture> MakeTexture(mjrfContext* ctx,
                                          const uint8_t* color) {
  mjrfTextureConfig config;
  mjrf_defaultTextureConfig(&config);
  config.width = 1;
  config.height = 1;
  config.format = mjPIXEL_FORMAT_RGB8;
  UniquePtr<mjrfTexture> texture = CreateTexture(ctx, config);

  mjrfTextureData data;
  mjrf_defaultTextureData(&data);
  data.bytes = color;
  data.num_bytes = 3;
  mjrf_uploadTexture(texture.get(), &data);
  return texture;
}

static UniquePtr<mjrfMesh> MakeQuadMesh(mjrfContext* ctx, bool small = false) {
  static const float vertices[] = {
      -256.0f, -256.0f, 256.0f, -256.0f, -256.0f, 256.0f, 256.0f, 256.0f,
  };
  static const float small_vertices[] = {
      -4.0f, -4.0f, 4.0f, -4.0f, -4.0f, 4.0f, 4.0f, 4.0f,
  };
  static const float uvs[] = {
      0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
  };
  static const uint8_t colors[] = {
      255, 255, 255, 255, 255, 255, 255, 255,
      255, 255, 255, 255, 255, 255, 255, 255,
  };
  static const uint16_t indices[] = {0, 1, 2, 1, 3, 2};

  mjrfMeshConfig config;
  mjrf_defaultMeshConfig(&config);
  config.num_attributes = 3;
  config.attributes[0].usage = mjVERTEX_ATTRIBUTE_USAGE_POSITION;
  config.attributes[0].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
  config.attributes[1].usage = mjVERTEX_ATTRIBUTE_USAGE_UV;
  config.attributes[1].type = mjVERTEX_ATTRIBUTE_TYPE_FLOAT2;
  config.attributes[2].usage = mjVERTEX_ATTRIBUTE_USAGE_COLOR;
  config.attributes[2].type = mjVERTEX_ATTRIBUTE_TYPE_UBYTE4;
  config.max_vertices = 4;
  config.max_indices = 6;
  config.interleaved = false;
  config.index_type = mjINDEX_TYPE_U16;
  config.primitive_type = mjMESH_PRIMITIVE_TYPE_TRIANGLES;
  UniquePtr<mjrfMesh> mesh = CreateMesh(ctx, config);

  mjrfMeshData data;
  mjrf_defaultMeshData(&data);
  data.num_vertices = 4;
  data.vertices[0] = small ? small_vertices : vertices;
  data.vertices[1] = uvs;
  data.vertices[2] = colors;
  data.num_indices = 6;
  data.indices = indices;
  mjrf_uploadMesh(mesh.get(), &data);

  return mesh;
}

static UniquePtr<mjrfRenderable> MakeRenderable(mjrfContext* ctx,
                                                mjrfMesh* mesh,
                                                mjrfTexture* texture) {
  mjrfMaterial material;
  mjrf_defaultMaterial(&material);
  material.decor = true;
  material.color_texture = texture;

  mjrfRenderableParams params;
  mjrf_defaultRenderableParams(&params);
  params.cast_shadow = false;
  params.receive_shadow = false;
  auto renderable = CreateRenderable(ctx, params);
  mjrf_setRenderableMesh(renderable.get(), mesh, 0, 0);
  mjrf_setRenderableMaterial(renderable.get(), &material);
  return renderable;
}

mjrfContextConfig MakeOffscreenConfig() {
  mjrfContextConfig config;
  mjrf_defaultContextConfig(&config);
  config.native_window = nullptr;
  config.software_rendering = true;
  config.graphics_api = mjGRAPHICS_API_VULKAN;
  return config;
}

void SetupView(Renderer::View* view, Renderer::SceneHandle scene,
               Renderer::RenderTargetHandle target, int width, int height) {
  view->scene = scene;
  view->render_target = target;
  view->post_processing = false;
  view->shadows = false;
  view->reflections = false;
  view->camera.orthographic = true;
  view->camera.pos[0] = 0.0f;
  view->camera.pos[1] = 0.0f;
  view->camera.pos[2] = 1.0f;
  view->camera.forward[0] = 0.0f;
  view->camera.forward[1] = 0.0f;
  view->camera.forward[2] = -1.0f;
  view->camera.up[0] = 0.0f;
  view->camera.up[1] = 1.0f;
  view->camera.up[2] = 0.0f;
  view->camera.frustum_top = 0.0f;
  view->camera.frustum_near = 0.0f;
  view->camera.frustum_far = 1.0f;
  view->camera.frustum_center = width / 2.0f;
  view->camera.frustum_width = width / 2.0f;
  view->camera.frustum_bottom = height;
}

TEST_F(RendererTest, ContextCreationAndDefaults) {
  Renderer renderer(MakeOffscreenConfig());
  ASSERT_NE(renderer.GetContext(), nullptr);
  mjrf_flush(renderer.GetContext());
  EXPECT_FLOAT_EQ(renderer.GetGpuFrameMs(), 0.0f);
}

TEST_F(RendererTest, SceneManagement) {
  Renderer renderer(MakeOffscreenConfig());

  // Unknown scene names and invalid handles return invalid/null results.
  EXPECT_EQ(renderer.GetSceneHandle("main"), Renderer::kInvalidScene);
  EXPECT_EQ(renderer.GetScene(Renderer::kInvalidScene), nullptr);
  EXPECT_EQ(renderer.GetScene(Renderer::SceneHandle{-5}), nullptr);
  EXPECT_EQ(renderer.GetScene(Renderer::SceneHandle{0}), nullptr);
  EXPECT_EQ(renderer.GetScene(Renderer::SceneHandle{100}), nullptr);

  // GetScene lazily creates a scene and assigns a valid handle.
  mjrfScene* scene_a = renderer.GetScene("scene_a");
  ASSERT_NE(scene_a, nullptr);
  Renderer::SceneHandle handle_a = renderer.GetSceneHandle("scene_a");
  EXPECT_NE(handle_a, Renderer::kInvalidScene);
  EXPECT_EQ(renderer.GetScene(handle_a), scene_a);

  // Requesting the same scene name again returns the existing scene instance.
  EXPECT_EQ(renderer.GetScene("scene_a"), scene_a);
  EXPECT_EQ(renderer.GetSceneHandle("scene_a"), handle_a);

  // Creating a second scene produces a distinct handle and pointer.
  mjrfScene* scene_b = renderer.GetScene("scene_b");
  ASSERT_NE(scene_b, nullptr);
  EXPECT_NE(scene_b, scene_a);
  Renderer::SceneHandle handle_b = renderer.GetSceneHandle("scene_b");
  EXPECT_NE(handle_b, Renderer::kInvalidScene);
  EXPECT_NE(handle_b, handle_a);
  EXPECT_EQ(renderer.GetScene(handle_b), scene_b);
  EXPECT_EQ(renderer.GetScene(handle_a), scene_a);
}

TEST_F(RendererTest, RenderTargetLifecycleAndResize) {
  Renderer renderer(MakeOffscreenConfig());

  EXPECT_EQ(renderer.GetRenderTargetHandle("color"),
            Renderer::kInvalidRenderTarget);
  EXPECT_TRUE(renderer.GetPixels(Renderer::kInvalidRenderTarget).empty());
  EXPECT_TRUE(renderer.GetPixels(Renderer::kWindowRenderTarget).empty());
  EXPECT_TRUE(renderer.GetPixels(Renderer::RenderTargetHandle{42}).empty());

  // Resizing invalid or out-of-bounds handles is a safe no-op.
  renderer.ResizeRenderTarget(Renderer::kInvalidRenderTarget, 64, 64);
  renderer.ResizeRenderTarget(Renderer::RenderTargetHandle{42}, 64, 64);
  renderer.ResizeRenderTarget(Renderer::kWindowRenderTarget, 128, 128);
  EXPECT_TRUE(renderer.GetPixels(Renderer::kWindowRenderTarget).empty());

  // Creating a target with positive dimensions immediately allocates buffer.
  mjrfRenderTargetConfig config;
  mjrf_defaultRenderTargetConfig(&config);
  config.width = 64;
  config.height = 32;
  config.color_format = mjPIXEL_FORMAT_RGBA8;
  config.depth_format = mjPIXEL_FORMAT_DEPTH32F;

  Renderer::RenderTargetHandle color_target =
      renderer.CreateRenderTarget("color", config);
  EXPECT_NE(color_target, Renderer::kInvalidRenderTarget);
  EXPECT_EQ(renderer.GetRenderTargetHandle("color"), color_target);
  EXPECT_EQ(renderer.GetPixels(color_target).size(), 64 * 32 * 4);

  // Resizing updates the pixel buffer capacity accordingly.
  renderer.ResizeRenderTarget(color_target, 128, 64);
  EXPECT_EQ(renderer.GetPixels(color_target).size(), 128 * 64 * 4);

  renderer.ResizeRenderTarget(color_target, 16, 8);
  EXPECT_EQ(renderer.GetPixels(color_target).size(), 16 * 8 * 4);

  renderer.ResizeRenderTarget(color_target, 0, 0);
  EXPECT_TRUE(renderer.GetPixels(color_target).empty());

  // Creating a target with zero initial dimensions defers pixel allocation
  // until ResizeRenderTarget is called.
  mjrfRenderTargetConfig deferred_config;
  mjrf_defaultRenderTargetConfig(&deferred_config);
  deferred_config.width = 0;
  deferred_config.height = 0;
  deferred_config.color_format = mjPIXEL_FORMAT_RGB8;

  Renderer::RenderTargetHandle deferred_target =
      renderer.CreateRenderTarget("deferred", deferred_config);
  EXPECT_NE(deferred_target, Renderer::kInvalidRenderTarget);
  EXPECT_NE(deferred_target, color_target);
  EXPECT_TRUE(renderer.GetPixels(deferred_target).empty());

  renderer.ResizeRenderTarget(deferred_target, 32, 24);
  EXPECT_EQ(renderer.GetPixels(deferred_target).size(), 32 * 24 * 3);
}

TEST_F(RendererTest, RenderTargetPixelFormatsAndDuplicateNames) {
  Renderer renderer(MakeOffscreenConfig());

  struct FormatCase {
    const char* name;
    mjrPixelFormat format;
    size_t bytes_per_pixel;
  };
  constexpr FormatCase kFormats[] = {
      {"r8", mjPIXEL_FORMAT_R8, 1},
      {"rgb8", mjPIXEL_FORMAT_RGB8, 3},
      {"rgba8", mjPIXEL_FORMAT_RGBA8, 4},
      {"r32f", mjPIXEL_FORMAT_R32F, 4},
  };

  constexpr int kWidth = 20;
  constexpr int kHeight = 10;
  for (const FormatCase& tc : kFormats) {
    mjrfRenderTargetConfig config;
    mjrf_defaultRenderTargetConfig(&config);
    config.width = kWidth;
    config.height = kHeight;
    config.color_format = tc.format;

    Renderer::RenderTargetHandle handle =
        renderer.CreateRenderTarget(tc.name, config);
    EXPECT_NE(handle, Renderer::kInvalidRenderTarget) << tc.name;
    EXPECT_EQ(renderer.GetRenderTargetHandle(tc.name), handle) << tc.name;
    EXPECT_EQ(renderer.GetPixels(handle).size(),
              kWidth * kHeight * tc.bytes_per_pixel)
        << tc.name;
  }

  // Creating a render target with an already-registered name raises mju_error.
  EXPECT_THAT(MjuErrorMessageFrom(+[](Renderer* r) {
                mjrfRenderTargetConfig cfg;
                mjrf_defaultRenderTargetConfig(&cfg);
                r->CreateRenderTarget("rgb8", cfg);
              })(&renderer),
              HasSubstr("Render target 'rgb8' already exists"));
}

TEST_F(RendererTest, RenderFullscreen) {
  Renderer renderer(MakeOffscreenConfig());

  mjrfContext* ctx = renderer.GetContext();
  UniquePtr<mjrfMesh> mesh = MakeQuadMesh(ctx);
  UniquePtr<mjrfTexture> texture = MakeTexture(ctx, kRed);
  UniquePtr<mjrfRenderable> renderable =
      MakeRenderable(ctx, mesh.get(), texture.get());

  mjrfRenderTargetConfig target_config;
  mjrf_defaultRenderTargetConfig(&target_config);
  target_config.width = 32;
  target_config.height = 32;
  target_config.color_format = mjPIXEL_FORMAT_RGB8;
  target_config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
  Renderer::RenderTargetHandle target =
      renderer.CreateRenderTarget("offscreen", target_config);
  ASSERT_NE(target, Renderer::kInvalidRenderTarget);

  mjrfScene* scene = renderer.GetScene("scene");
  ASSERT_NE(scene, nullptr);
  mjrf_addSceneRenderable(scene, renderable.get());

  Renderer::View views[1];
  SetupView(&views[0], renderer.GetSceneHandle("scene"), target, 32, 32);

  renderer.Render(views);

  std::span<const std::byte> pixels = renderer.GetPixels(target);
  EXPECT_EQ(pixels.size(), target_config.width * target_config.height * 3);
  EXPECT_NE(pixels.data(), nullptr);
  for (int i = 0; i < pixels.size(); i += 3) {
    EXPECT_EQ((uint8_t)pixels[i + 0], kRed[0]);
    EXPECT_EQ((uint8_t)pixels[i + 1], kRed[1]);
    EXPECT_EQ((uint8_t)pixels[i + 2], kRed[2]);
  }
  mjrf_removeSceneRenderable(scene, renderable.get());
}

TEST_F(RendererTest, RenderGrid) {
  Renderer renderer(MakeOffscreenConfig());

  mjrfContext* ctx = renderer.GetContext();

  UniquePtr<mjrfMesh> mesh = MakeQuadMesh(ctx);
  std::vector<UniquePtr<mjrfTexture>> textures;
  textures.push_back(MakeTexture(ctx, kRed));
  textures.push_back(MakeTexture(ctx, kGreen));
  textures.push_back(MakeTexture(ctx, kBlue));
  textures.push_back(MakeTexture(ctx, kWhite));

  std::vector<UniquePtr<mjrfRenderable>> renderables;
  for (int i = 0; i < textures.size(); ++i) {
    renderables.push_back(MakeRenderable(ctx, mesh.get(), textures[i].get()));
  }

  mjrfRenderTargetConfig target_config;
  mjrf_defaultRenderTargetConfig(&target_config);
  target_config.width = 32;
  target_config.height = 32;
  target_config.color_format = mjPIXEL_FORMAT_RGB8;
  target_config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
  Renderer::RenderTargetHandle target =
      renderer.CreateRenderTarget("offscreen", target_config);
  ASSERT_NE(target, Renderer::kInvalidRenderTarget);

  mjrfScene* scene0 = renderer.GetScene("scene0");
  mjrf_addSceneRenderable(scene0, renderables[0].get());
  mjrfScene* scene1 = renderer.GetScene("scene1");
  mjrf_addSceneRenderable(scene1, renderables[1].get());
  mjrfScene* scene2 = renderer.GetScene("scene2");
  mjrf_addSceneRenderable(scene2, renderables[2].get());
  mjrfScene* scene3 = renderer.GetScene("scene3");
  mjrf_addSceneRenderable(scene3, renderables[3].get());

  Renderer::View views[4];

  SetupView(&views[0], renderer.GetSceneHandle("scene0"), target, 32, 32);
  views[0].render_target = target;
  views[0].scene = renderer.GetSceneHandle("scene0");
  views[0].rel_viewport[0] = 0.0f;
  views[0].rel_viewport[1] = 0.0f;
  views[0].rel_viewport[2] = 0.5f;
  views[0].rel_viewport[3] = 0.5f;

  SetupView(&views[1], renderer.GetSceneHandle("scene1"), target, 32, 32);
  views[1].rel_viewport[0] = 0.0f;
  views[1].rel_viewport[1] = 0.5f;
  views[1].rel_viewport[2] = 0.5f;
  views[1].rel_viewport[3] = 0.5f;

  SetupView(&views[2], renderer.GetSceneHandle("scene2"), target, 32, 32);
  views[2].rel_viewport[0] = 0.5f;
  views[2].rel_viewport[1] = 0.0f;
  views[2].rel_viewport[2] = 0.5f;
  views[2].rel_viewport[3] = 0.5f;

  SetupView(&views[3], renderer.GetSceneHandle("scene3"), target, 32, 32);
  views[3].rel_viewport[0] = 0.5f;
  views[3].rel_viewport[1] = 0.5f;
  views[3].rel_viewport[2] = 0.5f;
  views[3].rel_viewport[3] = 0.5f;

  renderer.Render(views);

  std::span<const std::byte> pixels = renderer.GetPixels(target);
  EXPECT_EQ(pixels.size(), target_config.width * target_config.height * 3);
  EXPECT_NE(pixels.data(), nullptr);
  auto get_pixel = [&](int x, int y) {
    int index = (y * target_config.width + x) * 3;
    return std::make_tuple((uint8_t)pixels[index + 0],
                           (uint8_t)pixels[index + 1],
                           (uint8_t)pixels[index + 2]);
  };

  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 16; ++y) {
      auto pixel = get_pixel(x, y);
      EXPECT_EQ(std::get<0>(pixel), kGreen[0]);
      EXPECT_EQ(std::get<1>(pixel), kGreen[1]);
      EXPECT_EQ(std::get<2>(pixel), kGreen[2]);
    }
  }
  for (int x = 16; x < 32; ++x) {
    for (int y = 0; y < 16; ++y) {
      auto pixel = get_pixel(x, y);
      EXPECT_EQ(std::get<0>(pixel), kWhite[0]);
      EXPECT_EQ(std::get<1>(pixel), kWhite[1]);
      EXPECT_EQ(std::get<2>(pixel), kWhite[2]);
    }
  }
  for (int x = 0; x < 16; ++x) {
    for (int y = 16; y < 32; ++y) {
      auto pixel = get_pixel(x, y);
      EXPECT_EQ(std::get<0>(pixel), kRed[0]);
      EXPECT_EQ(std::get<1>(pixel), kRed[1]);
      EXPECT_EQ(std::get<2>(pixel), kRed[2]);
    }
  }
  for (int x = 16; x < 32; ++x) {
    for (int y = 16; y < 32; ++y) {
      auto pixel = get_pixel(x, y);
      EXPECT_EQ(std::get<0>(pixel), kBlue[0]);
      EXPECT_EQ(std::get<1>(pixel), kBlue[1]);
      EXPECT_EQ(std::get<2>(pixel), kBlue[2]);
    }
  }
  mjrf_removeSceneRenderable(scene0, renderables[0].get());
  mjrf_removeSceneRenderable(scene1, renderables[1].get());
  mjrf_removeSceneRenderable(scene2, renderables[2].get());
  mjrf_removeSceneRenderable(scene3, renderables[3].get());
}

// a texture with an unsupported number of channels is skipped with a warning,
// rather than an mju_error which would longjmp past the objects being built
TEST_F(RendererTest, UnsupportedTextureChannelsSkipped) {
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(mj_makeSpec(),
                                                         mj_deleteSpec);
  mjsTexture* texture = mjs_addTexture(spec.get(), nullptr);
  mjs_setName(texture->element, "texture");
  texture->type = mjTEXTURE_2D;
  texture->width = 2;
  texture->height = 2;
  texture->nchannel = 2;
  const std::vector<uint8_t> data(8, 255);
  mjs_setBuffer(texture->data, data.data(), data.size());
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_compile(spec.get(), nullptr), mj_deleteModel);
  ASSERT_NE(model, nullptr) << mjs_getError(spec.get());
  ASSERT_EQ(model->tex_nchannel[0], 2);

  Renderer renderer(MakeOffscreenConfig());
  mock_warning_handler.ExpectWarnings("2-channel");
  ModelResources resources(model.get(), renderer.GetContext());
  EXPECT_EQ(resources.GetTexture(0), nullptr);
}

TEST_F(RendererTest, TextureGeneratesMipmaps) {
  Renderer renderer(MakeOffscreenConfig());
  mjrfContext* ctx = renderer.GetContext();

  // 64x64 texture where 1 in every 4x4 block is 240 and all other texels are 0.
  // The box-filtered mip average at level >= 2 is 240 / 16 = 15, whereas
  // sampling level 0 at the 4x4 render target pixel centers (u = 8, 24, 40, 56)
  // would bilinearly sample one 240 texel and three 0 texels (240 / 4 = 60).
  constexpr int kTexSize = 64;
  std::vector<uint8_t> tex_bytes(kTexSize * kTexSize * 3, 0);
  for (int y = 0; y < kTexSize; y += 4) {
    for (int x = 0; x < kTexSize; x += 4) {
      int idx = (y * kTexSize + x) * 3;
      tex_bytes[idx + 0] = 240;
      tex_bytes[idx + 1] = 240;
      tex_bytes[idx + 2] = 240;
    }
  }

  mjrfTextureConfig tex_config;
  mjrf_defaultTextureConfig(&tex_config);
  tex_config.width = kTexSize;
  tex_config.height = kTexSize;
  tex_config.format = mjPIXEL_FORMAT_RGB8;
  tex_config.color_space = mjCOLORSPACE_LINEAR;
  UniquePtr<mjrfTexture> texture = CreateTexture(ctx, tex_config);

  mjrfTextureData tex_data;
  mjrf_defaultTextureData(&tex_data);
  tex_data.bytes = tex_bytes.data();
  tex_data.num_bytes = tex_bytes.size();
  mjrf_uploadTexture(texture.get(), &tex_data);

  UniquePtr<mjrfMesh> mesh = MakeQuadMesh(ctx, /*small=*/true);
  mjrfMaterial material;
  mjrf_defaultMaterial(&material);
  material.metallic = 0.0f;
  material.emissive_texture = texture.get();

  mjrfRenderableParams params;
  mjrf_defaultRenderableParams(&params);
  params.cast_shadow = false;
  params.receive_shadow = false;
  UniquePtr<mjrfRenderable> renderable = CreateRenderable(ctx, params);
  mjrf_setRenderableMesh(renderable.get(), mesh.get(), 0, 0);
  mjrf_setRenderableMaterial(renderable.get(), &material);

  mjrfRenderTargetConfig target_config;
  mjrf_defaultRenderTargetConfig(&target_config);
  target_config.width = 4;
  target_config.height = 4;
  target_config.color_format = mjPIXEL_FORMAT_RGB8;
  target_config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
  Renderer::RenderTargetHandle target =
      renderer.CreateRenderTarget("offscreen", target_config);
  ASSERT_NE(target, Renderer::kInvalidRenderTarget);

  mjrfScene* scene = renderer.GetScene("scene");
  mjrf_addSceneRenderable(scene, renderable.get());

  Renderer::View views[1];
  SetupView(&views[0], renderer.GetSceneHandle("scene"), target, 4, 4);
  views[0].camera.frustum_bottom = 0.0f;
  views[0].camera.frustum_top = 4.0f;
  renderer.Render(views);

  std::span<const std::byte> pixels = renderer.GetPixels(target);
  ASSERT_EQ(pixels.size(), 4 * 4 * 3);
  for (size_t i = 0; i < pixels.size(); ++i) {
    EXPECT_NEAR(static_cast<uint8_t>(pixels[i]), 15, 2);
  }

  // UI (decor) materials sample level 0 directly without mipmapping.
  mjrf_removeSceneRenderable(scene, renderable.get());
  UniquePtr<mjrfRenderable> decor_renderable =
      MakeRenderable(ctx, mesh.get(), texture.get());
  mjrf_addSceneRenderable(scene, decor_renderable.get());
  SetupView(&views[0], renderer.GetSceneHandle("scene"), target, 4, 4);
  renderer.Render(views);

  pixels = renderer.GetPixels(target);
  ASSERT_EQ(pixels.size(), 4 * 4 * 3);
  for (size_t i = 0; i < pixels.size(); ++i) {
    EXPECT_NEAR(static_cast<uint8_t>(pixels[i]), 60, 2);
  }
  mjrf_removeSceneRenderable(scene, decor_renderable.get());
}
// A texture destroyed before Filament has read its data releases the data once,
// after the upload, without touching the destroyed texture.
TEST_F(RendererTest, TextureDestroyedBeforeUploadReleasesData) {
  static int released;
  released = 0;
  {
    Renderer renderer(MakeOffscreenConfig());
    mjrfTextureConfig config;
    mjrf_defaultTextureConfig(&config);
    config.width = 4;
    config.height = 4;
    config.format = mjPIXEL_FORMAT_RGB8;
    UniquePtr<mjrfTexture> texture =
        CreateTexture(renderer.GetContext(), config);

    auto* bytes = new std::vector<uint8_t>(4 * 4 * 3, 255);
    mjrfTextureData data;
    mjrf_defaultTextureData(&data);
    data.bytes = bytes->data();
    data.num_bytes = bytes->size();
    data.user_data = bytes;
    data.release = +[](void* user_data) {
      delete static_cast<std::vector<uint8_t>*>(user_data);
      released++;
    };
    mjrf_uploadTexture(texture.get(), &data);
    texture.reset();
  }
  EXPECT_EQ(released, 1);
}

}  // namespace
}  // namespace mujoco
