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

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "src/render/filament/support/model_lights.h"
#include "src/render/filament/support/model_renderables.h"
#include "src/render/filament/support/model_resources.h"
#include "src/render/filament/support/renderer.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using SceneViewTest = MujocoTest;

constexpr int kSize = 128;

// Sixteen pairs of boxes, each box in a color of its own. The top faces of a
// pair lie in one plane, so where they overlap, a pixel shows whichever face
// is drawn last.
std::string CoplanarBoxes() {
  std::string geoms;
  for (int i = 0; i < 16; ++i) {
    const double x = (i % 4 - 1.5) * 0.5;
    const double y = (i / 4 - 1.5) * 0.5;
    const std::string g = std::to_string(0.1 + 0.05 * i);
    geoms += "<geom type='box' pos='" + std::to_string(x) + " " +
             std::to_string(y) + " 0' size='.2 .2 .1' rgba='.9 " + g +
             " .1 1'/>";
    geoms += "<geom type='box' pos='" + std::to_string(x + 0.05) + " " +
             std::to_string(y + 0.05) + " 0' size='.1 .1 .1' rgba='.1 " + g +
             " .9 1'/>";
  }
  return "<mujoco><worldbody>"
         "<light directional='true' pos='0 0 3' dir='0 0 -1'/>" +
         geoms + "</worldbody></mujoco>";
}

// A renderer of its own, with a scene of the model.
class ModelRenderer {
 public:
  ModelRenderer(const mjrfContextConfig& config, const mjModel* model,
                const mjData* data)
      : renderer_(config),
        resources_(model, renderer_.GetContext()),
        lights_(renderer_.GetScene("scene"), &resources_),
        renderables_(renderer_.GetScene("scene"), &resources_) {
    lights_.Update(data);
    renderables_.Update(data);
    mjrfRenderTargetConfig target;
    mjrf_defaultRenderTargetConfig(&target);
    target.width = kSize;
    target.height = kSize;
    target.color_format = mjPIXEL_FORMAT_RGB8;
    renderer_.CreateRenderTarget("target", target);
  }

  std::vector<std::byte> Draw(const mjrCamera& camera) {
    Renderer::View view;
    view.scene = renderer_.GetSceneHandle("scene");
    view.render_target = renderer_.GetRenderTargetHandle("target");
    view.camera = camera;
    renderer_.Render({&view, 1});
    auto pixels = renderer_.GetPixels(view.render_target);
    return {pixels.begin(), pixels.end()};
  }

 private:
  Renderer renderer_;
  ModelResources resources_;
  ModelLights lights_;
  ModelRenderables renderables_;
};

// The number of RGB pixels whose channel `a` exceeds their channel `b` by more
// than 100.
int CountPixelsWhere(const std::vector<std::byte>& rgb, int a, int b) {
  int count = 0;
  for (size_t i = 0; i + 2 < rgb.size(); i += 3) {
    const int excess =
        std::to_integer<int>(rgb[i + a]) - std::to_integer<int>(rgb[i + b]);
    count += excess > 100;
  }
  return count;
}

int CountDifferentBytes(const std::vector<std::byte>& a,
                        const std::vector<std::byte>& b) {
  int count = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    count += a[i] != b[i];
  }
  return count;
}

TEST_F(SceneViewTest, RenderersBuiltAlikeDrawCoplanarFacesAlike) {
  MjModelPtr model = LoadModelFromString(CoplanarBoxes());
  ASSERT_NE(model, nullptr);
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  mjvCamera view;
  mjv_defaultFreeCamera(model.get(), &view);
  view.lookat[2] = 0.1;
  view.distance = 3;
  view.azimuth = 90;
  view.elevation = -90;
  const mjrCamera camera = mjv_camera2GLCamera(model.get(), data.get(), &view);

  mjrfContextConfig config;
  mjrf_defaultContextConfig(&config);
  config.software_rendering = true;
  config.graphics_api = mjGRAPHICS_API_VULKAN;
  // All alive at once, so that none of them reuses another's memory.
  std::vector<std::unique_ptr<ModelRenderer>> renderers;
  for (int i = 0; i < 4; ++i) {
    renderers.push_back(
        std::make_unique<ModelRenderer>(config, model.get(), data.get()));
  }

  const std::vector<std::byte> first = renderers[0]->Draw(camera);
  ASSERT_EQ(first.size(), static_cast<size_t>(kSize * kSize * 3));
  EXPECT_GT(CountPixelsWhere(first, 0, 2), 0);  // Red faces show.
  EXPECT_GT(CountPixelsWhere(first, 2, 0), 0);  // Blue faces show.
  for (int i = 1; i < 4; ++i) {
    EXPECT_EQ(CountDifferentBytes(renderers[i]->Draw(camera), first), 0)
        << "renderer " << i;
  }
}

}  // namespace
}  // namespace mujoco
