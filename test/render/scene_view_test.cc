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

#include "src/render/filament/core/scene_view.h"

#include <filament/Options.h>
#include <filament/View.h>
#include <gtest/gtest.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "src/render/filament/support/renderer.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using SceneViewTest = MujocoTest;

TEST_F(SceneViewTest, ConfigureReadsAmbientOcclusionBilateralThreshold) {
  mjrfContextConfig config;
  mjrf_defaultContextConfig(&config);
  config.software_rendering = true;
  config.graphics_api = mjGRAPHICS_API_VULKAN;
  Renderer renderer(config);

  // The threshold a scene configured from the model passes to Filament.
  auto threshold = [&](const char* scene_name, const char* xml) {
    MjModelPtr model = LoadModelFromString(xml);
    mjrfScene* scene = renderer.GetScene(scene_name);
    mjrf_configureSceneFromModel(scene, model.get());
    return SceneView::downcast(scene)
        ->GetDefaultRenderView()
        ->getAmbientOcclusionOptions()
        .bilateralThreshold;
  };

  static constexpr char kDeclared[] = R"(
    <mujoco>
      <custom>
        <numeric name="filament.ao.bilateral_threshold" data="0.5"/>
      </custom>
    </mujoco>
  )";
  EXPECT_FLOAT_EQ(threshold("declared", kDeclared), 0.5f);
  EXPECT_FLOAT_EQ(threshold("undeclared", "<mujoco/>"),
                  filament::AmbientOcclusionOptions().bilateralThreshold);
}

}  // namespace
}  // namespace mujoco
