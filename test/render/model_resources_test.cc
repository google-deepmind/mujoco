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

#include "src/render/filament/support/model_resources.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "absl/strings/substitute.h"
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "src/render/filament/support/mjrf_ptr.h"
#include "src/render/filament/support/model_lights.h"
#include "src/render/filament/support/model_renderables.h"
#include "src/render/filament/support/renderer.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Return;

mjrfContextConfig MakeOffscreenConfig() {
  mjrfContextConfig config;
  mjrf_defaultContextConfig(&config);
  config.native_window = nullptr;
  config.software_rendering = true;
  config.graphics_api = mjGRAPHICS_API_VULKAN;
  return config;
}

// A model whose texture files are in test/render/testdata, compiled eager or
// with <compiler passthrough="textures"/>.
static MjModelPtr LoadTexturedModel(std::string_view assets,
                                    std::string_view body, bool by_reference) {
  std::string xml =
      absl::Substitute(R"(
  <mujoco>
    <compiler texturedir="$0" passthrough="$1"/>
    <asset>
      $2
    </asset>
    <worldbody>
      $3
    </worldbody>
  </mujoco>
  )",
                       GetTestDataFilePath("render/testdata"),
                       by_reference ? "textures" : "none", assets, body);
  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  return model;
}

// Renders the model of the given resources into a 64x64 RGB image, from a
// camera on the -y axis looking at the origin.
static std::vector<uint8_t> RenderModel(Renderer& renderer,
                                        ModelResources& resources) {
  const mjModel* model = resources.GetModel();
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjrfScene* scene = renderer.GetScene("scene");
  ModelRenderables renderables(scene, &resources);
  ModelLights lights(scene, &resources);
  renderables.Update(data);
  lights.Update(data);
  mj_deleteData(data);

  Renderer::RenderTargetHandle target =
      renderer.GetRenderTargetHandle("target");
  if (target == Renderer::kInvalidRenderTarget) {
    mjrfRenderTargetConfig config;
    mjrf_defaultRenderTargetConfig(&config);
    config.width = 64;
    config.height = 64;
    config.color_format = mjPIXEL_FORMAT_RGB8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    target = renderer.CreateRenderTarget("target", config);
  }

  Renderer::View views[1];
  Renderer::View& view = views[0];
  view.render_target = target;
  view.scene = renderer.GetSceneHandle("scene");
  view.camera.pos[1] = -4.0f;
  view.camera.forward[1] = 1.0f;
  view.camera.up[2] = 1.0f;
  view.camera.frustum_bottom = -0.05f;
  view.camera.frustum_top = 0.05f;
  view.camera.frustum_near = 0.1f;
  view.camera.frustum_far = 50.0f;
  view.post_processing = false;
  view.shadows = false;
  view.reflections = false;
  renderer.Render(views);
  std::span<const std::byte> pixels = renderer.GetPixels(target);
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(pixels.data());
  return std::vector<uint8_t>(bytes, bytes + pixels.size());
}

using ModelTextureTest = MujocoTest;

static constexpr char kPngAndKtxAssets[] = R"(
      <texture name="pattern" type="2d" file="test-pattern.png"/>
      <texture name="ibl" type="2d" file="ibl.ktx"/>
      <material name="pattern" texture="pattern"/>
)";
static constexpr char kPngAndKtxBody[] = R"(
      <light type="image" texture="ibl" intensity="10000"/>
      <geom type="box" size="1 1 1" material="pattern"/>
)";

// A PNG texture loaded with mj_loadTexture and a KTX read as bytes from the
// resource system render the same image as their eager counterparts.
TEST_F(ModelTextureTest, ByReferenceRendersLikeEager) {
  MjModelPtr eager = LoadTexturedModel(kPngAndKtxAssets, kPngAndKtxBody, false);
  MjModelPtr byref = LoadTexturedModel(kPngAndKtxAssets, kPngAndKtxBody, true);
  ASSERT_THAT(eager, NotNull());
  ASSERT_THAT(byref, NotNull());
  ASSERT_EQ(byref->tex_adr[0], -1);
  ASSERT_EQ(byref->tex_adr[1], -1);

  Renderer renderer(MakeOffscreenConfig());
  ModelResources eager_resources(eager.get(), renderer.GetContext());
  std::vector<uint8_t> expected = RenderModel(renderer, eager_resources);

  ModelResources byref_resources(byref.get(), renderer.GetContext());
  EXPECT_THAT(byref_resources.GetTexture(0), NotNull());
  EXPECT_THAT(byref_resources.GetTexture(1), NotNull());
  std::vector<uint8_t> actual = RenderModel(renderer, byref_resources);

  // the box fills the image with the pattern, which is not a flat color
  EXPECT_NE(std::count(expected.begin(), expected.end(), expected[0]),
            static_cast<std::ptrdiff_t>(expected.size()));
  EXPECT_EQ(actual, expected);
}

// A by-reference PNG authored colorspace="auto" keeps auto in the model; the
// renderer uploads it with the colorspace the file resolves to (sRGB here).
TEST_F(ModelTextureTest, ByReferenceResolvesAutoColorspace) {
  static constexpr char kAuto[] = R"(
      <texture name="pattern" type="2d" file="test-pattern.png"/>
      <material name="pattern" texture="pattern"/>
  )";
  static constexpr char kLinear[] = R"(
      <texture name="pattern" type="2d" file="test-pattern.png"
          colorspace="linear"/>
      <material name="pattern" texture="pattern"/>
  )";
  static constexpr char kBody[] = R"(
      <geom type="box" size="1 1 1" material="pattern"/>
  )";
  MjModelPtr eager = LoadTexturedModel(kAuto, kBody, false);
  MjModelPtr byref = LoadTexturedModel(kAuto, kBody, true);
  MjModelPtr linear = LoadTexturedModel(kLinear, kBody, true);
  ASSERT_THAT(eager, NotNull());
  ASSERT_THAT(byref, NotNull());
  ASSERT_THAT(linear, NotNull());
  ASSERT_EQ(eager->tex_colorspace[0], mjCOLORSPACE_SRGB);
  ASSERT_EQ(byref->tex_colorspace[0], mjCOLORSPACE_AUTO);

  Renderer renderer(MakeOffscreenConfig());
  ModelResources eager_resources(eager.get(), renderer.GetContext());
  ModelResources byref_resources(byref.get(), renderer.GetContext());
  ModelResources linear_resources(linear.get(), renderer.GetContext());
  std::vector<uint8_t> expected = RenderModel(renderer, eager_resources);
  EXPECT_EQ(RenderModel(renderer, byref_resources), expected);
  EXPECT_NE(RenderModel(renderer, linear_resources), expected);
}

// Two by-reference textures with the same file and decode parameters share one
// GPU texture; different parameters or embedded textures do not.
TEST_F(ModelTextureTest, ByReferenceSharesUploads) {
  static constexpr char kAssets[] = R"(
      <texture name="a" type="2d" file="test-pattern.png"/>
      <texture name="b" type="2d" file="test-pattern.png"/>
      <texture name="rgba" type="2d" file="test-pattern.png" nchannel="4"/>
      <texture name="linear" type="2d" file="test-pattern.png"
          colorspace="linear"/>
      <texture name="ktx" type="2d" file="ibl.ktx"/>
      <texture name="ktx2" type="2d" file="ibl.ktx" content_type="image/ktx"/>
  )";
  MjModelPtr eager = LoadTexturedModel(kAssets, "", false);
  MjModelPtr byref = LoadTexturedModel(kAssets, "", true);
  ASSERT_THAT(eager, NotNull());
  ASSERT_THAT(byref, NotNull());

  Renderer renderer(MakeOffscreenConfig());
  ModelResources eager_resources(eager.get(), renderer.GetContext());
  EXPECT_NE(eager_resources.GetTexture(0), eager_resources.GetTexture(1));

  ModelResources byref_resources(byref.get(), renderer.GetContext());
  const mjrfTexture* a = byref_resources.GetTexture(0);
  ASSERT_THAT(a, NotNull());
  EXPECT_EQ(byref_resources.GetTexture(1), a);
  EXPECT_NE(byref_resources.GetTexture(2), a);
  EXPECT_NE(byref_resources.GetTexture(3), a);
  // a KTX file is uploaded as the file's bytes; the content type is part of the
  // key as authored, so an explicit one is a different key
  EXPECT_THAT(byref_resources.GetTexture(4), NotNull());
  EXPECT_THAT(byref_resources.GetTexture(5), NotNull());
  EXPECT_NE(byref_resources.GetTexture(5), byref_resources.GetTexture(4));

  // re-uploading an id reads the file again, for every id which shares it
  byref_resources.UploadTexture(byref.get(), 0);
  const mjrfTexture* reloaded = byref_resources.GetTexture(0);
  EXPECT_THAT(reloaded, NotNull());
  EXPECT_NE(reloaded, a);
  EXPECT_EQ(byref_resources.GetTexture(1), reloaded);
  EXPECT_NE(byref_resources.GetTexture(2), reloaded);

  // an id whose key changed leaves the ids of its old key
  byref->tex_colorspace[1] = mjCOLORSPACE_LINEAR;
  byref_resources.UploadTexture(byref.get(), 1);
  EXPECT_THAT(byref_resources.GetTexture(1), NotNull());
  EXPECT_NE(byref_resources.GetTexture(1), reloaded);
  EXPECT_EQ(byref_resources.GetTexture(0), reloaded);
  byref_resources.UploadTexture(byref.get(), 0);
  EXPECT_NE(byref_resources.GetTexture(0), byref_resources.GetTexture(1));
}

// A by-reference texture whose file is missing warns once and leaves the
// material untextured, rendering as if the material had no texture.
TEST_F(ModelTextureTest, MissingByReferenceTextureWarns) {
  static constexpr char kMissing[] = R"(
      <texture name="missing" type="2d" file="missing.png"/>
      <material name="mat" texture="missing"/>
  )";
  static constexpr char kUntextured[] = R"(
      <material name="mat"/>
  )";
  static constexpr char kBody[] = R"(
      <geom type="box" size="1 1 1" material="mat"/>
  )";
  MjModelPtr missing = LoadTexturedModel(kMissing, kBody, true);
  MjModelPtr untextured = LoadTexturedModel(kUntextured, kBody, false);
  ASSERT_THAT(missing, NotNull());
  ASSERT_THAT(untextured, NotNull());

  Renderer renderer(MakeOffscreenConfig());
  ModelResources untextured_resources(untextured.get(), renderer.GetContext());
  std::vector<uint8_t> expected = RenderModel(renderer, untextured_resources);

  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("texture 0 not loaded, left untextured")))
      .WillOnce(Return());
  ModelResources missing_resources(missing.get(), renderer.GetContext());
  EXPECT_THAT(missing_resources.GetTexture(0), IsNull());
  EXPECT_EQ(RenderModel(renderer, missing_resources), expected);
}

TEST_F(ModelTextureTest, InvalidObjectConfigsWarnAndReturnNull) {
  Renderer renderer(MakeOffscreenConfig());
  mjrfContext* ctx = renderer.GetContext();

  // Default texture config has 0x0 dimensions.
  mjrfTextureConfig tex_cfg;
  mjrf_defaultTextureConfig(&tex_cfg);
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Invalid texture dimensions")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createTexture(ctx, &tex_cfg), IsNull());

  // Unknown pixel format.
  tex_cfg.width = 4;
  tex_cfg.height = 4;
  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Unsupported format")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createTexture(ctx, &tex_cfg), IsNull());

  // Non-RGB cubemap and non-1x6 cubemap dimensions.
  tex_cfg.sampler_type = mjTEXTURE_CUBE;
  tex_cfg.format = mjPIXEL_FORMAT_RGBA8;
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Only support RGB cubemaps")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createTexture(ctx, &tex_cfg), IsNull());

  tex_cfg.format = mjPIXEL_FORMAT_RGB8;
  tex_cfg.height = 8;
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Cube maps must contain 6 square images")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createTexture(ctx, &tex_cfg), IsNull());

  // Invalid mesh configs: 0 vertices, no positions, normals + interleaved.
  mjrfMeshConfig mesh_cfg;
  mjrf_defaultMeshConfig(&mesh_cfg);
  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Mesh has no vertices")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createMesh(ctx, &mesh_cfg), IsNull());

  mesh_cfg.max_vertices = 3;
  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Mesh has no positions")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createMesh(ctx, &mesh_cfg), IsNull());

  mesh_cfg.num_attributes = 2;
  mesh_cfg.interleaved = true;
  mesh_cfg.attributes[0] = {mjVERTEX_ATTRIBUTE_USAGE_POSITION,
                            mjVERTEX_ATTRIBUTE_TYPE_FLOAT3};
  mesh_cfg.attributes[1] = {mjVERTEX_ATTRIBUTE_USAGE_NORMAL,
                            mjVERTEX_ATTRIBUTE_TYPE_FLOAT3};
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr(
                  "Cannot support normals with interleaved vertex attributes")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createMesh(ctx, &mesh_cfg), IsNull());

  // Invalid render target config and light params.
  mjrfRenderTargetConfig rt_cfg;
  mjrf_defaultRenderTargetConfig(&rt_cfg);
  rt_cfg.color_format = mjPIXEL_FORMAT_KTX;
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Unsupported render target color format")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createRenderTarget(ctx, &rt_cfg), IsNull());

  mjrfLightParams light_params;
  mjrf_defaultLightParams(&light_params);
  light_params.type = 99;
  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Unsupported light type")))
      .WillOnce(Return());
  EXPECT_THAT(mjrf_createLight(ctx, &light_params), IsNull());
}

TEST_F(ModelTextureTest, MismatchedUploadsWarnAndReleasePayload) {
  Renderer renderer(MakeOffscreenConfig());
  mjrfContext* ctx = renderer.GetContext();

  mjrfTextureConfig tex_cfg;
  mjrf_defaultTextureConfig(&tex_cfg);
  tex_cfg.width = 2;
  tex_cfg.height = 2;
  tex_cfg.format = mjPIXEL_FORMAT_RGB8;
  UniquePtr<mjrfTexture> tex = CreateTexture(ctx, tex_cfg);
  ASSERT_THAT(tex.get(), NotNull());

  int tex_released = 0;
  auto* heap_bytes = new uint8_t[3]{1, 2, 3};
  struct ReleaseCtx {
    uint8_t* ptr;
    int* count;
  } tex_release_ctx{heap_bytes, &tex_released};

  mjrfTextureData tex_data;
  mjrf_defaultTextureData(&tex_data);
  tex_data.bytes = heap_bytes;
  tex_data.num_bytes = 3;  // Expected 2 * 2 * 3 = 12.
  tex_data.user_data = &tex_release_ctx;
  tex_data.release = +[](void* user) {
    auto* c = static_cast<ReleaseCtx*>(user);
    delete[] c->ptr;
    ++(*c->count);
  };

  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Texture size does not match data size")))
      .WillOnce(Return());
  mjrf_uploadTexture(tex.get(), &tex_data);
  EXPECT_EQ(tex_released, 1);

  mjrfMeshConfig mesh_cfg;
  mjrf_defaultMeshConfig(&mesh_cfg);
  mesh_cfg.max_vertices = 3;
  mesh_cfg.max_indices = 3;
  mesh_cfg.num_attributes = 1;
  mesh_cfg.attributes[0] = {mjVERTEX_ATTRIBUTE_USAGE_POSITION,
                            mjVERTEX_ATTRIBUTE_TYPE_FLOAT3};
  UniquePtr<mjrfMesh> mesh = CreateMesh(ctx, mesh_cfg);
  ASSERT_THAT(mesh.get(), NotNull());

  int mesh_released = 0;
  auto* heap_verts = new uint8_t[36]{};
  ReleaseCtx mesh_release_ctx{heap_verts, &mesh_released};

  mjrfMeshData mesh_data;
  mjrf_defaultMeshData(&mesh_data);
  mesh_data.num_vertices = 2;  // Mismatch with max_vertices = 3.
  mesh_data.vertices[0] = heap_verts;
  mesh_data.user_data = &mesh_release_ctx;
  mesh_data.release = +[](void* user) {
    auto* c = static_cast<ReleaseCtx*>(user);
    delete[] c->ptr;
    ++(*c->count);
  };

  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Vertex count does not match config")))
      .WillOnce(Return());
  mjrf_uploadMesh(mesh.get(), &mesh_data);
  EXPECT_EQ(mesh_released, 1);
}

TEST_F(ModelTextureTest, InvalidUploadIndicesAndModelMismatchWarn) {
  static constexpr char kAssets[] = R"(
      <texture name="srgb_r8" type="2d" file="test-pattern.png"
          nchannel="1" colorspace="sRGB"/>
      <hfield name="hf" nrow="2" ncol="2" size="1 1 1 0.1"/>
  )";
  MjModelPtr model = LoadTexturedModel(kAssets, "", false);
  MjModelPtr other = LoadTexturedModel("", "", false);
  ASSERT_THAT(model, NotNull());
  ASSERT_THAT(other, NotNull());

  Renderer renderer(MakeOffscreenConfig());
  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Unsupported format")))
      .WillOnce(Return());
  ModelResources resources(model.get(), renderer.GetContext());
  EXPECT_THAT(resources.GetTexture(0), IsNull());

  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Model mismatch")))
      .Times(3)
      .WillRepeatedly(Return());
  resources.UploadMesh(other.get(), 0);
  resources.UploadTexture(other.get(), 0);
  resources.UploadHeightField(other.get(), 0);

  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Invalid mesh index")))
      .WillOnce(Return());
  resources.UploadMesh(model.get(), -1);
  EXPECT_CALL(mock_warning_handler, Warn(HasSubstr("Invalid texture index")))
      .WillOnce(Return());
  resources.UploadTexture(model.get(), 99);
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Invalid height field index")))
      .WillOnce(Return());
  resources.UploadHeightField(model.get(), -1);
}

}  // namespace
}  // namespace mujoco
