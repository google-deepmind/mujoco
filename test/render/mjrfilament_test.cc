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

#include <mujoco/mjrfilament.h>

#include <array>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/render/filament/private/types.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Return;
using ::testing::StrEq;

using MjrFilamentTest = MujocoTest;

TEST_F(MjrFilamentTest, NullObjectWarnsForVoidFunctions) {
  EXPECT_CALL(mock_warning_handler, Warn("Invalid (nullptr) object: flush"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: destroyTexture"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: destroyMesh"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: destroyScene"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: destroyLight"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: destroyRenderable"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: destroyRenderTarget"))
      .WillOnce(Return());

  mjrf_flush(nullptr);
  mjrf_destroyTexture(nullptr);
  mjrf_destroyMesh(nullptr);
  mjrf_destroyScene(nullptr);
  mjrf_destroyLight(nullptr);
  mjrf_destroyRenderable(nullptr);
  mjrf_destroyRenderTarget(nullptr);
}

TEST_F(MjrFilamentTest, NullObjectWarnsAndReturnsDefault) {
  mjrfTextureConfig config;
  mjrf_defaultTextureConfig(&config);

  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: createTexture"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: getTextureWidth"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: getLightType"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid (nullptr) object: render"))
      .WillOnce(Return());

  EXPECT_THAT(mjrf_createTexture(nullptr, &config), IsNull());
  EXPECT_EQ(mjrf_getTextureWidth(nullptr), 0);
  EXPECT_EQ(mjrf_getLightType(nullptr), 0);
  EXPECT_EQ(mjrf_render(nullptr, nullptr, 0, nullptr, 0), 0);
}

TEST_F(MjrFilamentTest, NullVtableWarnsForVoidFunctions) {
  mjrfContext ctx{};
  mjrfTexture texture{};
  mjrfMesh mesh{};
  mjrfScene scene{};
  mjrfLight light{};
  mjrfRenderable renderable{};
  mjrfRenderTarget target{};

  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: flush"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: destroyTexture"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: destroyMesh"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: destroyScene"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: destroyLight"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: destroyRenderable"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: destroyRenderTarget"))
      .WillOnce(Return());

  mjrf_flush(&ctx);
  mjrf_destroyTexture(&texture);
  mjrf_destroyMesh(&mesh);
  mjrf_destroyScene(&scene);
  mjrf_destroyLight(&light);
  mjrf_destroyRenderable(&renderable);
  mjrf_destroyRenderTarget(&target);
}

TEST_F(MjrFilamentTest, NullVtableWarnsAndReturnsDefault) {
  mjrfContext ctx{};
  mjrfTexture texture{};
  mjrfLight light{};
  mjrfTextureConfig config;
  mjrf_defaultTextureConfig(&config);

  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: createTexture"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: getTextureWidth"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: getLightType"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: render"))
      .WillOnce(Return());

  EXPECT_THAT(mjrf_createTexture(&ctx, &config), IsNull());
  EXPECT_EQ(mjrf_getTextureWidth(&texture), 0);
  EXPECT_EQ(mjrf_getLightType(&light), 0);
  EXPECT_EQ(mjrf_render(&ctx, nullptr, 0, nullptr, 0), 0);
}

TEST_F(MjrFilamentTest, UnimplementedFunctionWarnsForVoidFunctions) {
  mjrfVtable_ vtable{};
  mjrfContext ctx{.vtable = &vtable};
  mjrfTexture texture{.vtable = &vtable};
  mjrfMesh mesh{.vtable = &vtable};
  mjrfScene scene{.vtable = &vtable};
  mjrfLight light{.vtable = &vtable};
  mjrfRenderable renderable{.vtable = &vtable};
  mjrfRenderTarget target{.vtable = &vtable};

  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: flush"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: destroyTexture"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: destroyMesh"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: destroyScene"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: destroyLight"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: destroyRenderable"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn("Not implemented: destroyRenderTarget"))
      .WillOnce(Return());

  mjrf_flush(&ctx);
  mjrf_destroyTexture(&texture);
  mjrf_destroyMesh(&mesh);
  mjrf_destroyScene(&scene);
  mjrf_destroyLight(&light);
  mjrf_destroyRenderable(&renderable);
  mjrf_destroyRenderTarget(&target);
}

TEST_F(MjrFilamentTest, UnimplementedFunctionWarnsAndReturnsDefault) {
  mjrfVtable_ vtable{};
  mjrfContext ctx{.vtable = &vtable};
  mjrfTexture texture{.vtable = &vtable};
  mjrfLight light{.vtable = &vtable};
  mjrfTextureConfig config;
  mjrf_defaultTextureConfig(&config);

  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: createTexture"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: getTextureWidth"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: getLightType"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: render"))
      .WillOnce(Return());

  EXPECT_THAT(mjrf_createTexture(&ctx, &config), IsNull());
  EXPECT_EQ(mjrf_getTextureWidth(&texture), 0);
  EXPECT_EQ(mjrf_getLightType(&light), 0);
  EXPECT_EQ(mjrf_render(&ctx, nullptr, 0, nullptr, 0), 0);
}

TEST_F(MjrFilamentTest, GetRendererInfoSetsDefaultsOnFailure) {
  mjrfContext null_vtable_ctx{};
  mjrfVtable_ empty_vtable{};
  mjrfContext unimplemented_ctx{.vtable = &empty_vtable};

  EXPECT_CALL(mock_warning_handler,
              Warn("Invalid (nullptr) object: getRendererInfo"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Invalid vtable: getRendererInfo"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: getRendererInfo"))
      .WillOnce(Return());

  mjrRendererInfo info_null = {"sentinel", "sentinel"};
  mjrf_getRendererInfo(nullptr, &info_null);
  EXPECT_THAT(info_null.renderer, StrEq("filament"));
  EXPECT_THAT(info_null.backend, StrEq(""));

  mjrRendererInfo info_null_vtable = {"sentinel", "sentinel"};
  mjrf_getRendererInfo(&null_vtable_ctx, &info_null_vtable);
  EXPECT_THAT(info_null_vtable.renderer, StrEq("filament"));
  EXPECT_THAT(info_null_vtable.backend, StrEq(""));

  mjrRendererInfo info_unimplemented = {"sentinel", "sentinel"};
  mjrf_getRendererInfo(&unimplemented_ctx, &info_unimplemented);
  EXPECT_THAT(info_unimplemented.renderer, StrEq("filament"));
  EXPECT_THAT(info_unimplemented.backend, StrEq(""));
}

TEST_F(MjrFilamentTest, PartialVtableDispatchesImplementedAndWarnsOnMissing) {
  struct FakeContext : mjrfContext_ {
    int flush_calls = 0;
    const mjrfTextureConfig* last_config = nullptr;
    mjrfTexture* next_texture = nullptr;
  };
  struct FakeTexture : mjrfTexture_ {
    int width = 0;
  };

  mjrfVtable_ vtable{};
  vtable.flush =
      +[](mjrfContext* ctx) { static_cast<FakeContext*>(ctx)->flush_calls++; };
  vtable.createTexture =
      +[](mjrfContext* ctx, const mjrfTextureConfig* config) {
        auto* fake = static_cast<FakeContext*>(ctx);
        fake->last_config = config;
        return fake->next_texture;
      };
  vtable.getTextureWidth = +[](const mjrfTexture* texture) {
    return static_cast<const FakeTexture*>(texture)->width;
  };
  vtable.render =
      +[](mjrfContext*, const mjrfRenderRequest*, int, const mjrfReadRequest*,
          int) -> mjrfFrameHandle { return 42; };

  FakeTexture texture;
  texture.vtable = &vtable;
  texture.width = 128;

  FakeContext ctx;
  ctx.vtable = &vtable;
  ctx.next_texture = &texture;

  mjrfTextureConfig config;
  mjrf_defaultTextureConfig(&config);

  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: waitForFrame"))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler, Warn("Not implemented: getTextureHeight"))
      .WillOnce(Return());

  // Implemented vtable entries dispatch without warning.
  mjrf_flush(&ctx);
  EXPECT_EQ(ctx.flush_calls, 1);
  EXPECT_EQ(mjrf_createTexture(&ctx, &config), &texture);
  EXPECT_EQ(ctx.last_config, &config);
  EXPECT_EQ(mjrf_getTextureWidth(&texture), 128);
  EXPECT_EQ(mjrf_render(&ctx, nullptr, 0, nullptr, 0), 42);

  // Unimplemented entries on the same vtable warn and return default values.
  mjrf_waitForFrame(&ctx, 42);
  EXPECT_EQ(mjrf_getTextureHeight(&texture), 0);
}

TEST_F(MjrFilamentTest, MalformedNumericSizesWarnAndKeepDefaults) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <custom>
      <numeric name="filament.bloom.enabled" data="1 0"/>
      <numeric name="filament.fog.density" data="0.5 0.6"/>
      <numeric name="filament.fog.color" data="0.2 0.3"/>
      <numeric name="filament.vignette.color" data="0.1 0.2 0.3"/>
    </custom>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(kXml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  mjrfRenderOpts defaults;
  mjrf_defaultRenderOpts(&defaults);

  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Invalid numeric size for "
                             "'filament.bloom.enabled': expected 1, got 2")))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Invalid numeric size for 'filament.fog.density': "
                             "expected 1, got 2")))
      .WillOnce(Return());
  EXPECT_CALL(
      mock_warning_handler,
      Warn(HasSubstr(
          "Invalid numeric size for 'filament.fog.color': expected 3, got 2")))
      .WillOnce(Return());
  EXPECT_CALL(mock_warning_handler,
              Warn(HasSubstr("Invalid numeric size for "
                             "'filament.vignette.color': expected 4, got 3")))
      .WillOnce(Return());

  mjrfRenderOpts opts;
  mjrf_modelRenderOpts(&opts, model.get());

  EXPECT_EQ(opts.bloom.enable, defaults.bloom.enable);
  EXPECT_FLOAT_EQ(opts.fog.density, defaults.fog.density);
  EXPECT_FLOAT_EQ(opts.fog.color[0], defaults.fog.color[0]);
  EXPECT_FLOAT_EQ(opts.fog.color[1], defaults.fog.color[1]);
  EXPECT_FLOAT_EQ(opts.fog.color[2], defaults.fog.color[2]);
  EXPECT_FLOAT_EQ(opts.vignette.color[0], defaults.vignette.color[0]);
  EXPECT_FLOAT_EQ(opts.vignette.color[1], defaults.vignette.color[1]);
  EXPECT_FLOAT_EQ(opts.vignette.color[2], defaults.vignette.color[2]);
  EXPECT_FLOAT_EQ(opts.vignette.color[3], defaults.vignette.color[3]);
}

}  // namespace
}  // namespace mujoco
