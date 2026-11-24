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

#include "studio/app.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <span>
#include <string>

#include "testing/base/public/gunit.h"
#include <imgui.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/support/renderer.h"
#include "studio/hal/graphics_mode.h"
#include "studio/hal/window.h"
#include "studio/render/model_scene.h"

namespace mujoco::studio {

struct AppTestPeer {
  static mjSpec* spec(App& app) { return app.has_spec() ? app.spec() : nullptr; }
  static mjModel* model(App& app) { return app.has_model() ? app.model() : nullptr; }
  static mjData* data(App& app) { return app.has_data() ? app.data() : nullptr; }
  static mjvCamera* camera(App& app) { return &app.camera_; }
  static mjvOption* vis_options(App& app) { return &app.vis_options_; }
  static Window* window(App& app) { return app.window_.get(); }
  static ModelScene* model_scene(App& app) { return app.model_scene_.get(); }
  static Renderer* renderer(App& app) { return app.renderer_.get(); }
  static std::unique_lock<std::mutex> LockPhysics(App& app) {
    return std::unique_lock<std::mutex>(app.physics_mutex_);
  }
  static Renderer::View GetSceneView(App& app) {
    std::unique_lock<std::mutex> lock(app.physics_mutex_);
    return app.GetSceneView();
  }
  static mjsElement* curr_element(const App& app) { return app.tmp_.curr_element; }
  static void set_curr_element(App& app, mjsElement* element) { app.tmp_.curr_element = element; }
};

namespace {

TEST(AppTest, CurrElementResetOnModelReloadAndEmptyInit) {
  setenv("SDL_VIDEODRIVER", "dummy", 1);

  App::Config config;
  config.width = 640;
  config.height = 480;
  config.ini_path = std::string(::testing::TempDir()) + "/curr_element_mujoco.ini";
  config.gfx_mode = GraphicsMode::FilamentOpenGlSoftware;
  App app(config);

  static constexpr char kXml1[] = R"(
  <mujoco>
    <worldbody>
      <body name="body1">
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char kXml2[] = R"(
  <mujoco>
    <worldbody>
      <body name="body2">
        <geom type="box" size="0.1 0.1 0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::span<const std::byte> buf1(reinterpret_cast<const std::byte*>(kXml1), sizeof(kXml1) - 1);
  std::span<const std::byte> buf2(reinterpret_cast<const std::byte*>(kXml2), sizeof(kXml2) - 1);

  app.LoadModelFromBuffer(buf1, "text/xml", "m1.xml");
  ASSERT_NE(AppTestPeer::spec(app), nullptr);
  mjsBody* b1 = mjs_findBody(AppTestPeer::spec(app), "body1");
  ASSERT_NE(b1, nullptr);
  mjsElement* el = b1->element;
  ASSERT_NE(el, nullptr);

  AppTestPeer::set_curr_element(app, el);
  EXPECT_EQ(AppTestPeer::curr_element(app), el);

  app.LoadModelFromBuffer(buf2, "text/xml", "m2.xml");
  EXPECT_EQ(AppTestPeer::curr_element(app), nullptr);

  mjsBody* b2 = mjs_findBody(AppTestPeer::spec(app), "body2");
  ASSERT_NE(b2, nullptr);
  AppTestPeer::set_curr_element(app, b2->element);
  EXPECT_EQ(AppTestPeer::curr_element(app), b2->element);

  app.InitEmptyModel();
  EXPECT_EQ(AppTestPeer::curr_element(app), nullptr);
}

TEST(AppTest, PrimaryViewportReinterpretsFovyAsFovxOnRealtimeResizeOnly) {
  setenv("SDL_VIDEODRIVER", "dummy", 1);

  App::Config config;
  config.width = 400;
  config.height = 200;
  config.ini_path = std::string(::testing::TempDir()) + "/portrait_fov_mujoco.ini";
  config.gfx_mode = GraphicsMode::FilamentOpenGlSoftware;
  App app(config);

  // Camera at (0, 0, 2) looking along -z with fovy = 90 deg (tan(45 deg) = 1).
  // At z = 0 (distance 2), a 1:1 viewport spans x in [-2, 2], y in [-2, 2].
  // With portrait fovx = 90 deg at 200x400 (aspect 0.5), the primary viewport
  // spans x in [-2, 2] and y in [-4, 4]:
  //   - A red sphere at (1.5, 0, 0) stays visible near x_ndc = +0.75 in the
  //     primary viewport, whereas in an unadjusted 200x400 PiP render
  //     (x in [-1, 1]) it lies completely outside the frustum.
  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="90"/>
      <headlight ambient="1 1 1" diffuse="1 1 1"/>
    </visual>
    <worldbody>
      <camera name="main" pos="0 0 2" fovy="90"/>
      <camera name="calibrated" pos="0 0 2" sensorsize="0.04 0.04" focal="0.02 0.02"/>
      <body name="edge_body" pos="1.5 0 0">
        <geom name="edge_sphere" type="sphere" size="0.2" rgba="1 0 0 1"/>
      </body>
      <body name="top_body" pos="0 3.0 0">
        <geom name="top_sphere" type="sphere" size="0.2" rgba="0 1 0 1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::span<const std::byte> buf(reinterpret_cast<const std::byte*>(kXml), sizeof(kXml) - 1);
  app.LoadModelFromBuffer(buf, "text/xml", "portrait.xml");

  mjModel* model = AppTestPeer::model(app);
  mjData* data = AppTestPeer::data(app);
  mjvCamera* camera = AppTestPeer::camera(app);
  Window* window = AppTestPeer::window(app);
  ASSERT_NE(model, nullptr);
  ASSERT_NE(data, nullptr);

  mjv_defaultFreeCamera(model, camera);

  mjrCamera base;
  {
    auto lock = AppTestPeer::LockPhysics(app);
    base = mjv_camera2GLCamera(model, data, camera);
  }
  ASSERT_GT(base.frustum_top, 0.0f);

  const auto step_frame = [&app, window]() {
    app.Update();
    window->EndFrame();
  };

  // Initial 400x200 landscape: fovy is vertical.
  step_frame();
  EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_top, base.frustum_top);
  EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_bottom, base.frustum_bottom);

  // Realtime resize to 200x200 (1:1 boundary): continuous, no jump.
  window->Resize(200, 200);
  step_frame();
  EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_top, base.frustum_top);
  EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_bottom, base.frustum_bottom);

  // Realtime resize to 200x400 portrait (aspect 0.5): frustum_top doubles so
  // horizontal halfwidth 0.5 * 0.5 * (top - bottom) == base.frustum_top.
  window->Resize(200, 400);
  step_frame();
  const mjrCamera portrait_cam = AppTestPeer::GetSceneView(app).camera;
  EXPECT_FLOAT_EQ(portrait_cam.frustum_top, 2.0f * base.frustum_top);
  EXPECT_FLOAT_EQ(portrait_cam.frustum_bottom, 2.0f * base.frustum_bottom);

  // Tracking camera also expands vertically in portrait.
  camera->type = mjCAMERA_TRACKING;
  camera->trackbodyid = 1;
  step_frame();
  const mjrCamera tracking_cam = AppTestPeer::GetSceneView(app).camera;
  EXPECT_FLOAT_EQ(tracking_cam.frustum_top, 2.0f * base.frustum_top);
  EXPECT_FLOAT_EQ(tracking_cam.frustum_bottom, 2.0f * base.frustum_bottom);

  // Realtime resize back to 400x200 landscape restores standard vertical fovy.
  camera->type = mjCAMERA_FREE;
  window->Resize(400, 200);
  step_frame();
  EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_top, base.frustum_top);
  EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_bottom, base.frustum_bottom);

  // Fixed model cameras (both fovy-only and intrinsic-calibrated) are left
  // unchanged in portrait so they stay faithful to offscreen rendering.
  camera->type = mjCAMERA_FIXED;
  window->Resize(200, 400);
  for (int cam_id : {0, 1}) {
    camera->fixedcamid = cam_id;
    mjrCamera fixed_base;
    {
      auto lock = AppTestPeer::LockPhysics(app);
      fixed_base = mjv_camera2GLCamera(model, data, camera);
    }
    step_frame();
    EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_top, fixed_base.frustum_top);
    EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_bottom,
                    fixed_base.frustum_bottom);
    EXPECT_FLOAT_EQ(AppTestPeer::GetSceneView(app).camera.frustum_width, fixed_base.frustum_width);
  }
}

TEST(AppTest, ModelSceneSyncProjectsLabelsWithPortraitFov) {
  setenv("SDL_VIDEODRIVER", "dummy", 1);

  App::Config config;
  config.width = 200;
  config.height = 200;
  config.ini_path = std::string(::testing::TempDir()) + "/model_scene_labels_mujoco.ini";
  config.gfx_mode = GraphicsMode::FilamentOpenGlSoftware;
  App app(config);

  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="90"/>
    </visual>
    <worldbody>
      <camera name="main" pos="0 0 2" fovy="90"/>
      <body name="origin_body" pos="0 0 0">
        <geom type="sphere" size="0.01"/>
      </body>
      <body name="edge_body" pos="1.5 0 0">
        <geom type="sphere" size="0.1"/>
      </body>
      <body name="top_body" pos="0 3.0 0">
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::span<const std::byte> buf(reinterpret_cast<const std::byte*>(kXml), sizeof(kXml) - 1);
  app.LoadModelFromBuffer(buf, "text/xml", "labels.xml");

  mjModel* model = AppTestPeer::model(app);
  mjData* data = AppTestPeer::data(app);
  Renderer* renderer = AppTestPeer::renderer(app);
  ASSERT_NE(model, nullptr);
  ASSERT_NE(data, nullptr);

  std::map<std::string, std::array<float, 3>> labels;
  mjrfScene* label_scene = renderer->GetScene("label_test");
  ModelScene model_scene(
      renderer->GetContext(), label_scene, model,
      [&labels](const char* label, float x, float y, float z) { labels[label] = {x, y, z}; });

  mjvOption opt;
  mjv_defaultOption(&opt);
  opt.label = mjLABEL_BODY;

  // Free camera at (0, 0, 2) looking along -z with right=+x, up=+y.
  mjvCamera cam;
  mjv_defaultFreeCamera(model, &cam);
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 0.0;
  cam.distance = 2.0;
  cam.azimuth = 90.0;
  cam.elevation = -90.0;

  // Square 200x200: edge_body at (1.5, 0, 0) -> x_ndc = 0.75; top_body at
  // (0, 3.0, 0) -> y_ndc = 1.5 (outside [-1, 1]).
  auto lock = AppTestPeer::LockPhysics(app);
  labels.clear();
  model_scene.Sync(model, data, {0, 0, 200, 200}, &opt, nullptr, &cam, {});
  ASSERT_TRUE(labels.contains("edge_body"));
  ASSERT_TRUE(labels.contains("top_body"));
  EXPECT_NEAR(labels["edge_body"][0], 0.75f, 1e-4f);
  EXPECT_NEAR(labels["edge_body"][1], 0.0f, 1e-4f);
  EXPECT_NEAR(labels["top_body"][1], 1.5f, 1e-4f);

  // Portrait 100x200 (aspect 0.5): edge_body stays at x_ndc = 0.75 (fovx = 90)
  // and top_body maps into the expanded vertical view at y_ndc = 0.75.
  labels.clear();
  model_scene.Sync(model, data, {0, 0, 100, 200}, &opt, nullptr, &cam, {});
  ASSERT_TRUE(labels.contains("edge_body"));
  ASSERT_TRUE(labels.contains("top_body"));
  EXPECT_NEAR(labels["edge_body"][0], 0.75f, 1e-4f);
  EXPECT_NEAR(labels["edge_body"][1], 0.0f, 1e-4f);
  EXPECT_NEAR(labels["top_body"][0], 0.0f, 1e-4f);
  EXPECT_NEAR(labels["top_body"][1], 0.75f, 1e-4f);

  // Tracking camera also projects labels with portrait FOV.
  mjvCamera tracking_cam = cam;
  tracking_cam.type = mjCAMERA_TRACKING;
  tracking_cam.trackbodyid = mj_name2id(model, mjOBJ_BODY, "origin_body");
  labels.clear();
  model_scene.Sync(model, data, {0, 0, 100, 200}, &opt, nullptr, &tracking_cam, {});
  ASSERT_TRUE(labels.contains("edge_body"));
  ASSERT_TRUE(labels.contains("top_body"));
  EXPECT_NEAR(labels["edge_body"][0], 0.75f, 1e-4f);
  EXPECT_NEAR(labels["top_body"][1], 0.75f, 1e-4f);

  // Fixed model camera keeps vertical fovy in portrait: edge_body projects to
  // x_ndc = 1.5 (cropped) and top_body stays at y_ndc = 1.5.
  mjvCamera fixed_cam;
  mjv_defaultCamera(&fixed_cam);
  fixed_cam.type = mjCAMERA_FIXED;
  fixed_cam.fixedcamid = 0;
  labels.clear();
  model_scene.Sync(model, data, {0, 0, 100, 200}, &opt, nullptr, &fixed_cam, {});
  ASSERT_TRUE(labels.contains("edge_body"));
  ASSERT_TRUE(labels.contains("top_body"));
  EXPECT_NEAR(labels["edge_body"][0], 1.5f, 1e-4f);
  EXPECT_NEAR(labels["top_body"][1], 1.5f, 1e-4f);
}

TEST(AppTest, SceneLabelsCoverFullViewportAndStayBehindUiAfterPortraitToLandscapeResize) {
  setenv("SDL_VIDEODRIVER", "dummy", 1);

  const std::string ini_path =
      std::string(::testing::TempDir()) + "/resize_labels_regression_mujoco.ini";
  {
    std::ofstream ini_file(ini_path);
    ini_file << "[Window][labels]\nPos=60,60\nSize=150,400\nCollapsed=0\n\n";
  }

  App::Config config;
  config.width = 200;
  config.height = 600;
  config.ini_path = ini_path;
  config.gfx_mode = GraphicsMode::FilamentOpenGlSoftware;
  App app(config);

  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <camera name="main" pos="0 0 2" fovy="90"/>
      <body name="left_body" pos="-1.5 0 0">
        <geom type="sphere" size="0.1"/>
      </body>
      <body name="right_body" pos="2.2 0 0">
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::span<const std::byte> buf(reinterpret_cast<const std::byte*>(kXml), sizeof(kXml) - 1);
  app.LoadModelFromBuffer(buf, "text/xml", "welcome_labels.xml");

  mjModel* model = AppTestPeer::model(app);
  mjData* data = AppTestPeer::data(app);
  mjvCamera* camera = AppTestPeer::camera(app);
  mjvOption* vis_options = AppTestPeer::vis_options(app);
  Window* window = AppTestPeer::window(app);
  ModelScene* model_scene = AppTestPeer::model_scene(app);
  ASSERT_NE(model, nullptr);
  ASSERT_NE(data, nullptr);
  ASSERT_NE(model_scene, nullptr);

  camera->type = mjCAMERA_FIXED;
  camera->fixedcamid = 0;
  vis_options->label = mjLABEL_BODY;

  const auto run_ui_and_labels_frame = [&]() {
    app.Update();
    app.BuildGui();
    {
      auto lock = AppTestPeer::LockPhysics(app);
      model_scene->Sync(model, data, {0, 0, window->GetWidth(), window->GetHeight()}, vis_options,
                        nullptr, camera, {});
    }
    ImGui::Render();
    window->EndFrame();
  };

  for (int i = 0; i < 3; ++i) {
    run_ui_and_labels_frame();
  }

  // Resize from 200x600 portrait to 900x600 landscape (aspect 1.5, horizontal
  // half-span at z=0 is 3.0, so right_body at x=2.2 projects to x_ndc ≈ +0.733,
  // screen x ≈ 780 >> 200).
  window->Resize(900, 600);
  run_ui_and_labels_frame();

  const ImDrawData* draw_data = ImGui::GetDrawData();
  ASSERT_NE(draw_data, nullptr);

  int labels_idx = -1;
  int options_idx = -1;
  const ImDrawList* labels_list = nullptr;
  for (int i = 0; i < draw_data->CmdLists.Size; ++i) {
    const ImDrawList* list = draw_data->CmdLists[i];
    if (!list->_OwnerName) {
      continue;
    }
    if (std::strncmp(list->_OwnerName, "labels", 6) == 0) {
      labels_idx = i;
      labels_list = list;
    } else if (std::strncmp(list->_OwnerName, "Options", 7) == 0) {
      options_idx = i;
    }
  }

  ASSERT_NE(labels_list, nullptr);
  EXPECT_GE(labels_idx, 0);
  EXPECT_GT(options_idx, labels_idx);
  ASSERT_EQ(labels_list->CmdBuffer.Size, 1);
  EXPECT_FLOAT_EQ(labels_list->CmdBuffer[0].ClipRect.x, 0.0f);
  EXPECT_FLOAT_EQ(labels_list->CmdBuffer[0].ClipRect.y, 0.0f);
  EXPECT_FLOAT_EQ(labels_list->CmdBuffer[0].ClipRect.z, 900.0f);
  EXPECT_FLOAT_EQ(labels_list->CmdBuffer[0].ClipRect.w, 600.0f);

  float max_vtx_x = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < labels_list->VtxBuffer.Size; ++i) {
    max_vtx_x = std::max(max_vtx_x, labels_list->VtxBuffer[i].pos.x);
  }
  EXPECT_GT(max_vtx_x, 750.0f);
}

}  // namespace
}  // namespace mujoco::studio
