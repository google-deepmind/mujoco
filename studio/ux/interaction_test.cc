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

#include "studio/ux/interaction.h"

#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "testing/base/public/gunit.h"
#include <mujoco/mujoco.h>
#include "studio/sim/batch_view.h"
#include "studio/sim/batched_model.h"
#include "studio/sim/model_holder.h"

namespace mujoco::studio {
namespace {

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

struct TestScene {
  ModelPtr model{nullptr, mj_deleteModel};
  DataPtr data{nullptr, mj_deleteData};
};

// Returns a null model and data, and fails the test, if `xml` does not compile.
TestScene MakeTestScene(const char* xml) {
  char error[1000] = "";
  mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
  if (!spec) {
    ADD_FAILURE() << error;
    return {};
  }
  mjModel* m = mj_compile(spec, nullptr);
  if (!m) {
    ADD_FAILURE() << mjs_getError(spec);
    mj_deleteSpec(spec);
    return {};
  }
  mj_deleteSpec(spec);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  return {ModelPtr(m, mj_deleteModel), DataPtr(d, mj_deleteData)};
}

// Returns the height of the visible area at the lookat point.
mjtNum ViewHeightAtLookat(const mjModel* m, const mjvCamera* cam) {
  float zver[2] = {0, 0}, zclip[2] = {0, 0};
  mjv_cameraFrustum(zver, nullptr, zclip, m, cam);
  if (cam->orthographic) {
    return zver[0] + zver[1];
  }
  return (zver[0] + zver[1]) * cam->distance / zclip[0];
}

constexpr char kOrthographicTestXml[] = R"(
<mujoco>
  <worldbody>
    <body name="box" pos="0.5 -0.3 0.2">
      <freejoint/>
      <geom type="box" size="0.5 0.5 0.5"/>
    </body>
  </worldbody>
</mujoco>
)";

TEST(InteractionTest, OrthographicViewHeightFollowsDistance) {
  auto [m, d] = MakeTestScene(kOrthographicTestXml);
  ASSERT_NE(m, nullptr);

  mjvCamera cam;
  mjv_defaultFreeCamera(m.get(), &cam);
  cam.distance = 3.0;
  const mjtNum perspective_height = ViewHeightAtLookat(m.get(), &cam);

  // Toggling orthographic preserves the view size at the lookat point.
  cam.orthographic = 1;
  EXPECT_NEAR(ViewHeightAtLookat(m.get(), &cam), perspective_height, 1e-5);

  // Zooming scales the orthographic view.
  cam.distance = 0.8 * 3.0;
  EXPECT_NEAR(ViewHeightAtLookat(m.get(), &cam), 0.8 * perspective_height, 1e-5);

  // The render camera agrees with the frustum.
  const mjvGLCamera gl_camera = mjv_camera2GLCamera(m.get(), d.get(), &cam);
  EXPECT_TRUE(gl_camera.orthographic);
  EXPECT_NEAR(gl_camera.frustum_top - gl_camera.frustum_bottom, 0.8 * perspective_height, 1e-5);
}

TEST(InteractionTest, OrthographicEyeIsBehindModelWhenZoomedIn) {
  auto [m, d] = MakeTestScene(kOrthographicTestXml);
  ASSERT_NE(m, nullptr);

  mjvCamera cam;
  mjv_defaultFreeCamera(m.get(), &cam);
  cam.orthographic = 1;
  cam.distance = 0.01 * m->stat.extent;  // Unadjusted eye would be in the box.

  const mjvGLCamera gl_camera = mjv_camera2GLCamera(m.get(), d.get(), &cam);
  mjtNum eye[3], forward[3], to_center[3];
  mju_f2n(eye, gl_camera.pos, 3);
  mju_f2n(forward, gl_camera.forward, 3);
  mju_sub3(to_center, m->stat.center, eye);
  // The whole model is in front of the near plane and behind the far plane.
  EXPECT_GT(mju_dot3(to_center, forward) - m->stat.extent, gl_camera.frustum_near);
  EXPECT_LT(mju_dot3(to_center, forward) + m->stat.extent, gl_camera.frustum_far);
}

TEST(InteractionTest, OrthographicPickHitsFrontFaceWhenZoomedIn) {
  auto [m, d] = MakeTestScene(kOrthographicTestXml);
  ASSERT_NE(m, nullptr);
  const int box_id = mj_name2id(m.get(), mjOBJ_BODY, "box");
  ASSERT_GT(box_id, 0);

  // Look along +x at the center of the box, zoomed in so far that an
  // unadjusted eye would be inside the box.
  mjvCamera cam;
  mjv_defaultFreeCamera(m.get(), &cam);
  cam.orthographic = 1;
  cam.azimuth = 0;
  cam.elevation = 0;
  mju_copy3(cam.lookat, d->xpos + 3 * box_id);
  cam.distance = 0.01 * m->stat.extent;

  mjvOption opt;
  mjv_defaultOption(&opt);
  const PickResult picked = Pick(m.get(), d.get(), &cam, 0.5f, 0.5f,
                                 /*aspect_ratio=*/1.5f, &opt);
  EXPECT_EQ(picked.body, box_id);
  EXPECT_NEAR(picked.point[0], d->xpos[3 * box_id] - 0.5, 1e-5);
}

TEST(InteractionTest, OrthographicPlanarMoveAndPerturbUseViewHeight) {
  auto [m, d] = MakeTestScene(kOrthographicTestXml);
  ASSERT_NE(m, nullptr);

  mjvCamera cam;
  mjv_defaultFreeCamera(m.get(), &cam);
  cam.orthographic = 1;
  cam.azimuth = 90.0;
  cam.elevation = -90.0 + 1e-3;  // Looking straight down.
  cam.distance = 2.0;
  const mjtNum view_height = ViewHeightAtLookat(m.get(), &cam);

  // Moving the mouse by the full window height moves the lookat point by the
  // full view height.
  mjtNum lookat_before[3];
  mju_copy3(lookat_before, cam.lookat);
  MoveCamera(m.get(), d.get(), &cam, CameraMotion::PLANAR_MOVE_H, 0, 1.0);
  mjtNum moved[3];
  mju_sub3(moved, cam.lookat, lookat_before);
  EXPECT_NEAR(mju_norm3(moved), view_height, 1e-3 * view_height);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);
  pert.select = mj_name2id(m.get(), mjOBJ_BODY, "box");
  ASSERT_GT(pert.select, 0);
  InitPerturb(m.get(), d.get(), &cam, &pert, mjPERT_TRANSLATE);
  EXPECT_NEAR(pert.scale, view_height, 1e-6);
}

TEST(InteractionTest, OrthographicTrackingCameraLooksAtBodyNotStaleLookat) {
  auto [m, d] = MakeTestScene(kOrthographicTestXml);
  ASSERT_NE(m, nullptr);
  const int box_id = mj_name2id(m.get(), mjOBJ_BODY, "box");
  ASSERT_GT(box_id, 0);

  // A tracking camera whose lookat was not updated since the body moved.
  mjvCamera cam;
  mjv_defaultFreeCamera(m.get(), &cam);
  cam.type = mjCAMERA_TRACKING;
  cam.trackbodyid = box_id;
  cam.orthographic = 1;
  cam.lookat[0] = cam.lookat[1] = cam.lookat[2] = 10;

  // The body's center of mass is on the render camera's axis.
  const mjvGLCamera gl_camera = mjv_camera2GLCamera(m.get(), d.get(), &cam);
  mjtNum eye[3], forward[3], to_body[3], off_axis[3];
  mju_f2n(eye, gl_camera.pos, 3);
  mju_f2n(forward, gl_camera.forward, 3);
  mju_sub3(to_body, d->subtree_com + 3 * box_id, eye);
  mju_cross(off_axis, to_body, forward);
  EXPECT_NEAR(mju_norm3(off_axis), 0, 1e-5);

  // Picking at the center of the view hits the body.
  mjvOption opt;
  mjv_defaultOption(&opt);
  const PickResult picked = Pick(m.get(), d.get(), &cam, 0.5f, 0.5f,
                                 /*aspect_ratio=*/1.5f, &opt);
  EXPECT_EQ(picked.body, box_id);
}

TEST(InteractionTest, PickSkinDoesNotCrashOnUninflatedSkin) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="b0" pos="0 0 1"/>
    </worldbody>
    <deformable>
      <skin name="skin" vertex="0 0 1 1 0 1 0 1 1" face="0 1 2">
        <bone body="b0" bindpos="0 0 1" bindquat="1 0 0 0" vertid="0 1 2"
          vertweight="1 1 1"/>
      </skin>
    </deformable>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(m, nullptr);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  cam.lookat[0] = 0.33;
  cam.lookat[1] = 0.33;
  cam.lookat[2] = 1.0;
  cam.distance = 2.0;

  mjvOption opt;
  mjv_defaultOption(&opt);

  PickResult res = Pick(m, d, &cam, 0.5f, 0.5f, 1.0f, &opt);
  EXPECT_EQ(res.skin, 0);
  EXPECT_EQ(res.body, 1);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(InteractionTest, ViewCameraReinterpretsFovyAlongShorterAxisForFreeAndTrackingOnly) {
  static constexpr char kCamXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="60"/>
    </visual>
    <worldbody>
      <geom size="1"/>
      <camera name="fovy_only" pos="0 0 3" mode="fixed" fovy="90"/>
      <camera name="ortho" pos="0 0 3" mode="fixed" projection="orthographic" fovy="4"/>
      <camera name="intrinsic" pos="0 0 3" mode="fixed"
              sensorsize="0.036 0.024" focal="0.05 0.05"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(kCamXml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* model = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(model, nullptr);
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  mjvCamera cam;
  mjv_defaultFreeCamera(model, &cam);
  const mjrCamera base = mjv_camera2GLCamera(model, data, &cam);
  ASSERT_EQ(base.frustum_width, 0.0f);
  ASSERT_GT(base.frustum_top, 0.0f);

  // Landscape (> 1:1) and square (1:1): unchanged.
  const mjrCamera landscape = ViewCamera(model, data, &cam, 16.0f / 9.0f);
  EXPECT_FLOAT_EQ(landscape.frustum_top, base.frustum_top);
  EXPECT_FLOAT_EQ(landscape.frustum_bottom, base.frustum_bottom);

  const mjrCamera square = ViewCamera(model, data, &cam, 1.0f);
  EXPECT_FLOAT_EQ(square.frustum_top, base.frustum_top);
  EXPECT_FLOAT_EQ(square.frustum_bottom, base.frustum_bottom);

  // Portrait (< 1:1): vertical frustum expands by 1 / aspect so horizontal
  // halfwidth 0.5 * aspect * (top - bottom) equals base.frustum_top.
  constexpr float kPortraitAspect = 9.0f / 16.0f;
  const mjrCamera portrait = ViewCamera(model, data, &cam, kPortraitAspect);
  EXPECT_FLOAT_EQ(portrait.frustum_top, base.frustum_top / kPortraitAspect);
  EXPECT_FLOAT_EQ(portrait.frustum_bottom, base.frustum_bottom / kPortraitAspect);
  const float portrait_halfwidth =
      0.5f * kPortraitAspect * (portrait.frustum_top - portrait.frustum_bottom);
  EXPECT_NEAR(portrait_halfwidth, base.frustum_top, 1e-6f);

  // Tracking camera also expands in portrait.
  mjvCamera tracking_cam = cam;
  tracking_cam.type = mjCAMERA_TRACKING;
  tracking_cam.trackbodyid = 0;
  const mjrCamera tracking_portrait = ViewCamera(model, data, &tracking_cam, kPortraitAspect);
  EXPECT_FLOAT_EQ(tracking_portrait.frustum_top, base.frustum_top / kPortraitAspect);
  EXPECT_FLOAT_EQ(tracking_portrait.frustum_bottom, base.frustum_bottom / kPortraitAspect);

  // Orthographic free camera also expands vertically in portrait.
  mjvCamera ortho_free = cam;
  ortho_free.orthographic = 1;
  const mjrCamera ortho_base = mjv_camera2GLCamera(model, data, &ortho_free);
  ASSERT_GT(ortho_base.frustum_top, 0.0f);
  const mjrCamera ortho_gl = ViewCamera(model, data, &ortho_free, 0.5f);
  EXPECT_FLOAT_EQ(ortho_gl.frustum_top, ortho_base.frustum_top / 0.5f);
  EXPECT_FLOAT_EQ(ortho_gl.frustum_bottom, ortho_base.frustum_bottom / 0.5f);

  // Fixed model cameras (fovy-only, orthographic, and intrinsic-calibrated)
  // are all left untouched in portrait so they match offscreen rendering.
  for (int cam_id = 0; cam_id < model->ncam; ++cam_id) {
    mjvCamera fixed_cam;
    mjv_defaultCamera(&fixed_cam);
    fixed_cam.type = mjCAMERA_FIXED;
    fixed_cam.fixedcamid = cam_id;
    const mjrCamera fixed_base = mjv_camera2GLCamera(model, data, &fixed_cam);
    const mjrCamera fixed_portrait = ViewCamera(model, data, &fixed_cam, 0.5f);
    EXPECT_FLOAT_EQ(fixed_portrait.frustum_top, fixed_base.frustum_top);
    EXPECT_FLOAT_EQ(fixed_portrait.frustum_bottom, fixed_base.frustum_bottom);
    EXPECT_FLOAT_EQ(fixed_portrait.frustum_width, fixed_base.frustum_width);
    EXPECT_FLOAT_EQ(fixed_portrait.frustum_center, fixed_base.frustum_center);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST(InteractionTest, PickWithViewCameraMatchesPortraitFovWhileMjvOverloadStaysStandard) {
  // Free/tracking camera at (0, 0, 2) looking along -z with fovy = 90 deg
  // (tan(45 deg) = 1). At z = 0 (distance 2):
  //   - 1:1 viewport covers x in [-2, 2], y in [-2, 2].
  //   - 0.5 portrait viewport with ViewCamera covers x in [-2, 2] (fovx = 90)
  //     and y in [-4, 4] (expanded vertical FOV).
  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="90"/>
    </visual>
    <worldbody>
      <camera name="main" pos="0 0 2" fovy="90"/>
      <camera name="calibrated" pos="0 0 2" sensorsize="0.04 0.04" focal="0.02 0.02"/>
      <body name="origin_body" pos="0 0 0">
        <geom name="origin_geom" type="sphere" size="0.01"/>
      </body>
      <body name="right_edge" pos="1.8 0 0">
        <geom name="right_geom" type="sphere" size="0.1"/>
      </body>
      <body name="top_expanded" pos="0 3.2 0">
        <geom name="top_geom" type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(kXml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(m, nullptr);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  const int origin_body = mj_name2id(m, mjOBJ_BODY, "origin_body");
  const int right_body = mj_name2id(m, mjOBJ_BODY, "right_edge");
  const int top_body = mj_name2id(m, mjOBJ_BODY, "top_expanded");
  ASSERT_GT(origin_body, 0);
  ASSERT_GT(right_body, 0);
  ASSERT_GT(top_body, 0);

  // Free camera positioned at (0, 0, 2) looking along -z with right=+x, up=+y.
  mjvCamera cam;
  mjv_defaultFreeCamera(m, &cam);
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 0.0;
  cam.distance = 2.0;
  cam.azimuth = 90.0;
  cam.elevation = -90.0;

  mjvOption opt;
  mjv_defaultOption(&opt);

  // Normalized screen coords: x = (1.8 / 2.0 + 1) / 2 = 0.95;
  // y (top-down) for y_world = 3.2 in [-4, 4] is (1 - 3.2 / 4.0) / 2 = 0.1.
  const mjrCamera square_view = ViewCamera(m, d, &cam, 1.0f);
  EXPECT_EQ(Pick(m, d, square_view, 0.95f, 0.5f, 1.0f, &opt).body, right_body);
  EXPECT_EQ(Pick(m, d, square_view, 0.5f, 0.1f, 1.0f, &opt).body, -1);

  const mjrCamera portrait_view = ViewCamera(m, d, &cam, 0.5f);
  EXPECT_EQ(Pick(m, d, portrait_view, 0.95f, 0.5f, 0.5f, &opt).body, right_body);
  EXPECT_EQ(Pick(m, d, portrait_view, 0.5f, 0.1f, 0.5f, &opt).body, top_body);

  // The mjvCamera* overload does not implicitly apply portrait FOV so
  // non-Studio callers using unmodified mjv_camera2GLCamera keep matching their
  // render.
  EXPECT_EQ(Pick(m, d, &cam, 0.95f, 0.5f, 0.5f, &opt).body, -1);
  EXPECT_EQ(Pick(m, d, &cam, 0.5f, 0.1f, 0.5f, &opt).body, -1);

  // Tracking camera with ViewCamera also expands in portrait.
  mjvCamera tracking_cam = cam;
  tracking_cam.type = mjCAMERA_TRACKING;
  tracking_cam.trackbodyid = origin_body;
  const mjrCamera tracking_view = ViewCamera(m, d, &tracking_cam, 0.5f);
  EXPECT_EQ(Pick(m, d, tracking_view, 0.95f, 0.5f, 0.5f, &opt).body, right_body);
  EXPECT_EQ(Pick(m, d, tracking_view, 0.5f, 0.1f, 0.5f, &opt).body, top_body);

  // Orthographic free camera with ViewCamera also expands in portrait. Its
  // view is as tall as the perspective view at the lookat point (z = 0).
  mjvCamera ortho_free = cam;
  ortho_free.orthographic = 1;
  const mjrCamera ortho_view = ViewCamera(m, d, &ortho_free, 0.5f);
  EXPECT_EQ(Pick(m, d, ortho_view, 0.95f, 0.5f, 0.5f, &opt).body, right_body);
  EXPECT_EQ(Pick(m, d, ortho_view, 0.5f, 0.1f, 0.5f, &opt).body, top_body);

  // Fixed model cameras (both fovy-only and intrinsic-calibrated) keep their
  // standard frustums in portrait.
  mjvCamera fixed_cam;
  mjv_defaultCamera(&fixed_cam);
  fixed_cam.type = mjCAMERA_FIXED;
  fixed_cam.fixedcamid = 0;
  const mjrCamera fixed_view = ViewCamera(m, d, &fixed_cam, 0.5f);
  EXPECT_EQ(Pick(m, d, fixed_view, 0.95f, 0.5f, 0.5f, &opt).body, -1);
  EXPECT_EQ(Pick(m, d, fixed_view, 0.5f, 0.1f, 0.5f, &opt).body, -1);

  fixed_cam.fixedcamid = 1;
  const mjrCamera calibrated_view = ViewCamera(m, d, &fixed_cam, 0.5f);
  EXPECT_EQ(Pick(m, d, calibrated_view, 0.95f, 0.5f, 0.5f, &opt).body, right_body);
  EXPECT_EQ(Pick(m, d, calibrated_view, 0.5f, 0.1f, 0.5f, &opt).body, -1);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(InteractionTest, MovePerturbScalesVerticalDragAndIsotropicRotationInPortrait) {
  // Free/tracking camera at (0, -2, 0) looking along +y with fovy = 90 deg.
  // At y = 0 (depth 2): horizontal world span in 0.5 portrait is 4.0 and
  // vertical world span is 8.0.
  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="90"/>
    </visual>
    <worldbody>
      <camera name="front" pos="0 -2 0" xyaxes="1 0 0 0 0 1" fovy="90"/>
      <camera name="calibrated" pos="0 -2 0" xyaxes="1 0 0 0 0 1"
              sensorsize="0.04 0.04" focal="0.02 0.02"/>
      <camera name="ortho" pos="0 -2 0" xyaxes="1 0 0 0 0 1"
              projection="orthographic" fovy="4"/>
      <body name="box" pos="0 0 0">
        <freejoint/>
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(kXml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(m, nullptr);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // Free camera at (0, -2, 0) looking along +y with right=+x, up=+z.
  mjvCamera cam;
  mjv_defaultFreeCamera(m, &cam);
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 0.0;
  cam.distance = 2.0;
  cam.azimuth = 90.0;
  cam.elevation = 0.0;

  // Equal 20-pixel drag in a 200x400 portrait viewport (dx = 0.1, dy = -0.05)
  // produces equal world displacement (0.4 m) horizontally and vertically.
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);
  pert.select = 1;
  InitPerturb(m, d, &cam, &pert, mjPERT_TRANSLATE);
  MovePerturb(m, d, &cam, &pert, mjMOUSE_MOVE_V, 0.1, -0.05, 0.5f);
  EXPECT_NEAR(pert.refpos[0], 0.4, 1e-5);
  EXPECT_NEAR(pert.refpos[1], 0.0, 1e-5);
  EXPECT_NEAR(pert.refpos[2], 0.4, 1e-5);

  // Equal 20-pixel horizontal vs vertical rotation drag in 200x400 portrait
  // produces identical rotation angle and an unskewed 45-deg axis.
  mjvPerturb rot_h;
  mjv_defaultPerturb(&rot_h);
  rot_h.select = 1;
  InitPerturb(m, d, &cam, &rot_h, mjPERT_ROTATE);
  MovePerturb(m, d, &cam, &rot_h, mjMOUSE_ROTATE_V, 0.1, 0.0, 0.5f);

  mjvPerturb rot_v;
  mjv_defaultPerturb(&rot_v);
  rot_v.select = 1;
  InitPerturb(m, d, &cam, &rot_v, mjPERT_ROTATE);
  MovePerturb(m, d, &cam, &rot_v, mjMOUSE_ROTATE_V, 0.0, 0.05, 0.5f);
  EXPECT_NEAR(rot_h.refquat[0], rot_v.refquat[0], 1e-6);

  mjvPerturb rot_diag;
  mjv_defaultPerturb(&rot_diag);
  rot_diag.select = 1;
  InitPerturb(m, d, &cam, &rot_diag, mjPERT_ROTATE);
  MovePerturb(m, d, &cam, &rot_diag, mjMOUSE_ROTATE_V, 0.1, 0.05, 0.5f);
  EXPECT_NEAR(rot_diag.refquat[1], rot_diag.refquat[3], 1e-6);

  // Tracking camera also scales vertical perturbation in portrait.
  mjvCamera tracking_cam = cam;
  tracking_cam.type = mjCAMERA_TRACKING;
  tracking_cam.trackbodyid = 1;
  mjvPerturb track_pert;
  mjv_defaultPerturb(&track_pert);
  track_pert.select = 1;
  InitPerturb(m, d, &tracking_cam, &track_pert, mjPERT_TRANSLATE);
  MovePerturb(m, d, &tracking_cam, &track_pert, mjMOUSE_MOVE_V, 0.1, -0.05, 0.5f);
  EXPECT_NEAR(track_pert.refpos[0], 0.4, 1e-5);
  EXPECT_NEAR(track_pert.refpos[2], 0.4, 1e-5);

  // Fixed model cameras (both fovy-only and intrinsic-calibrated) leave
  // vertical perturbation unscaled in portrait.
  mjvCamera fixed_cam;
  mjv_defaultCamera(&fixed_cam);
  fixed_cam.type = mjCAMERA_FIXED;
  for (int cam_id : {0, 1}) {
    fixed_cam.fixedcamid = cam_id;
    mjvPerturb fixed_pert;
    mjv_defaultPerturb(&fixed_pert);
    fixed_pert.select = 1;
    InitPerturb(m, d, &fixed_cam, &fixed_pert, mjPERT_TRANSLATE);
    MovePerturb(m, d, &fixed_cam, &fixed_pert, mjMOUSE_MOVE_V, 0.1, -0.1, 0.5f);
    EXPECT_NEAR(fixed_pert.refpos[0], fixed_pert.refpos[2], 1e-5);
  }

  // MoveCamera scales vertical planar pan in portrait.
  mjvCamera free_cam;
  mjv_defaultFreeCamera(m, &free_cam);
  mjvCamera expected_cam = free_cam;
  MoveCamera(m, d, &free_cam, CameraMotion::PLANAR_MOVE_V, 0.1, 0.2, 0.5f);
  mjv_moveCamera(m, mjMOUSE_MOVE_V, 0.1, 0.4, &expected_cam);
  EXPECT_NEAR(free_cam.lookat[0], expected_cam.lookat[0], 1e-6);
  EXPECT_NEAR(free_cam.lookat[1], expected_cam.lookat[1], 1e-6);
  EXPECT_NEAR(free_cam.lookat[2], expected_cam.lookat[2], 1e-6);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(InteractionTest, PickAndPerturbHandleFixedOrthographicCamera) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <camera name="ortho" pos="0 0 2" projection="orthographic" fovy="4"/>
      <body name="top_edge" pos="0 1.6 0">
        <freejoint/>
        <geom name="top_geom" type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(kXml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(m, nullptr);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  const int top_body = mj_name2id(m, mjOBJ_BODY, "top_edge");
  ASSERT_GT(top_body, 0);

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = 0;

  mjvOption opt;
  mjv_defaultOption(&opt);

  // Orthographic vertical extent is [-2, 2], so y_world = 1.6 is at
  // top-down y = (1 - 1.6 / 2.0) / 2 = 0.1.
  const PickResult picked = Pick(m, d, &cam, 0.5f, 0.1f, 1.0f, &opt);
  EXPECT_EQ(picked.body, top_body);

  // Orthographic movement scale equals the vertical frustum height (fovy).
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);
  pert.select = top_body;
  InitPerturb(m, d, &cam, &pert, mjPERT_TRANSLATE);
  EXPECT_NEAR(pert.scale, 4.0, 1e-5);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(InteractionTest, PickUsesRealBottomAnchorForAsymmetricFrustum) {
  // sensorsize="0.04 0.04", focal="0.02 0.02", principal="0 0.01":
  // At z = 0 (depth 2), vertical frustum spans y in [-3, +1], so the viewport
  // midpoint (x = 0.5, y = 0.5) projects to (0, -1, 0), not (0, +1, 0).
  static constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <camera name="shifted" pos="0 0 2" sensorsize="0.04 0.04"
              focal="0.02 0.02" principal="0 0.01"/>
      <body name="at_neg_y" pos="0 -1 0">
        <geom type="sphere" size="0.1"/>
      </body>
      <body name="at_pos_y" pos="0 1 0">
        <geom type="sphere" size="0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(kXml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(m, nullptr);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  const int neg_body = mj_name2id(m, mjOBJ_BODY, "at_neg_y");
  ASSERT_GT(neg_body, 0);

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = 0;

  mjvOption opt;
  mjv_defaultOption(&opt);

  const PickResult picked = Pick(m, d, &cam, 0.5f, 0.5f, 1.0f, &opt);
  EXPECT_EQ(picked.body, neg_body);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(InteractionTest, PickToleranceSelectsNearbyGeomAndPrioritizesStrictVisibleBounds) {
  // Camera at (0, 0, 2) looking along -z with fovy = 90 deg on a 400x800
  // portrait smartphone viewport (aspect 0.5):
  //   - At z = 0 (dist 2): x_world in [-2, 2], so 1 px = 0.01 m.
  //   - At z = 1 (dist 1): x_world in [-1, 1], so 1 px = 0.005 m.
  //   - At z = -1 (dist 3): x_world in [-3, 3], so 1 px = 0.015 m.
  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="90"/>
    </visual>
    <worldbody>
      <geom name="ground" type="plane" pos="0 0 -1" size="10 10 0.1"/>
      <body name="small_near" pos="0 0 1">
        <geom name="small_near_geom" type="sphere" size="0.02"/>
      </body>
      <body name="far_visible" pos="0.30 0 -1">
        <geom name="far_visible_geom" type="box" size="0.21 0.5 0.05"/>
      </body>
      <body name="occluded_back" pos="-1.0 0 -0.5">
        <geom name="occluded_geom" type="sphere" size="0.06"/>
      </body>
      <body name="occluder_front" pos="-0.4 0 1.0">
        <geom name="occluder_geom" type="box" size="0.15 0.15 0.05"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(kXml, nullptr, error.data(), error.size());
  ASSERT_NE(spec, nullptr) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  ASSERT_NE(m, nullptr);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  const int ground_geom = mj_name2id(m, mjOBJ_GEOM, "ground");
  const int small_near_body = mj_name2id(m, mjOBJ_BODY, "small_near");
  const int far_visible_body = mj_name2id(m, mjOBJ_BODY, "far_visible");
  const int occluded_body = mj_name2id(m, mjOBJ_BODY, "occluded_back");
  const int occluder_body = mj_name2id(m, mjOBJ_BODY, "occluder_front");
  ASSERT_GE(ground_geom, 0);
  ASSERT_GT(small_near_body, 0);
  ASSERT_GT(far_visible_body, 0);
  ASSERT_GT(occluded_body, 0);
  ASSERT_GT(occluder_body, 0);

  mjvCamera cam;
  mjv_defaultFreeCamera(m, &cam);
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 0.0;
  cam.distance = 2.0;
  cam.azimuth = 90.0;
  cam.elevation = -90.0;

  mjvOption opt;
  mjv_defaultOption(&opt);

  constexpr float kWidth = 400.0f;
  constexpr float kHeight = 800.0f;
  constexpr float kAspect = kWidth / kHeight;
  constexpr float kTol = kTouchPickTolerancePx / kHeight;
  const mjrCamera view_cam = ViewCamera(m, d, &cam, kAspect);

  // 1. Touch 14 px left of small_near center (px = 186, outside its 4 px
  // radius, over the ground plane):
  //   - Desktop (tolerance = 0) hits the ground plane (body == 0).
  //   - Touch (22 px tolerance) selects small_near.
  const float near_miss_x = 186.0f / kWidth;
  const PickResult desktop_miss = Pick(m, d, view_cam, near_miss_x, 0.5f, kAspect, &opt);
  EXPECT_EQ(desktop_miss.geom, ground_geom);
  EXPECT_EQ(desktop_miss.body, 0);

  const PickResult phone_hit = Pick(m, d, view_cam, near_miss_x, 0.5f, kAspect, &opt, kTol);
  EXPECT_EQ(phone_hit.body, small_near_body);

  // Beyond the 22 px radius (30 px right of far_visible's right edge at
  // px = 234 -> px = 264, clear of all non-plane geoms), touch tolerance
  // falls back to the center ground-plane hit.
  const PickResult phone_outside_tol =
      Pick(m, d, view_cam, 264.0f / kWidth, 0.5f, kAspect, &opt, kTol);
  EXPECT_EQ(phone_outside_tol.geom, ground_geom);
  EXPECT_EQ(phone_outside_tol.body, 0);

  // 2. Strict visible-bounds priority:
  // small_near spans screen px [196, 204] at depth ~1.0.
  // far_visible spans x_world [0.09, 0.51] at z = -1 (depth 3.0), which
  // projects to screen px [206, 234].
  // Touching at px = 210 is strictly inside the visible portion of far_visible
  // AND only 6 px outside small_near (well inside small_near's 22 px tolerance
  // halo, with small_near closer in 3D depth). Pick MUST prioritize
  // far_visible because the touch already lies strictly within its visible
  // bounds.
  const PickResult strict_priority =
      Pick(m, d, view_cam, 210.0f / kWidth, 0.5f, kAspect, &opt, kTol);
  EXPECT_EQ(strict_priority.body, far_visible_body);

  // 3. True 3D occlusion:
  // occluded_back at (-1.0, 0, -0.5) (dist 2.5) projects to px = 120 with
  // 8 px radius [112, 128], completely hidden behind occluder_front at
  // (-0.4, 0, 1.0) (dist 1.0) which spans px [90, 150].
  // Touching at px = 82 (8 px left of occluder_front, 30 px from occluded
  // center; or px = 100 directly over occluder_front) must NEVER select
  // occluded_back through occluder_front.
  const PickResult over_occluder = Pick(m, d, view_cam, 120.0f / kWidth, 0.5f, kAspect, &opt, kTol);
  EXPECT_EQ(over_occluder.body, occluder_body);

  const PickResult beside_occluder =
      Pick(m, d, view_cam, 82.0f / kWidth, 0.5f, kAspect, &opt, kTol);
  EXPECT_EQ(beside_occluder.body, occluder_body);
  EXPECT_NE(beside_occluder.body, occluded_body);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(InteractionTest, PickToleranceStaticFallbackPrefersClosestDepthAndIgnoresNaN) {
  static constexpr char kXml[] = R"(
  <mujoco>
    <visual>
      <global fovy="45"/>
    </visual>
    <worldbody>
      <geom name="ground" type="plane" pos="0 0 -2" size="10 10 0.1"/>
      <geom name="far_right" type="sphere" pos="0.08 0 -0.5" size="0.05"/>
      <geom name="near_left" type="sphere" pos="-0.035 0 1.0" size="0.025"/>
    </worldbody>
  </mujoco>
  )";
  auto [m, d] = MakeTestScene(kXml);
  ASSERT_NE(m, nullptr);

  const int ground_geom = mj_name2id(m.get(), mjOBJ_GEOM, "ground");
  const int near_left_geom = mj_name2id(m.get(), mjOBJ_GEOM, "near_left");
  ASSERT_GE(ground_geom, 0);
  ASSERT_GE(near_left_geom, 0);

  mjvCamera cam;
  mjv_defaultFreeCamera(m.get(), &cam);
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 0.0;
  cam.distance = 2.0;
  cam.azimuth = 90.0;
  cam.elevation = -90.0;

  mjvOption opt;
  mjv_defaultOption(&opt);

  const mjrCamera view_cam = ViewCamera(m.get(), d.get(), &cam, /*aspect_ratio=*/1.0f);

  // Center ray hits the ground plane; ring 1 hits `far_right` at angle 0 before
  // `near_left` at angle pi, so closest depth on the innermost hit ring must
  // win.
  const PickResult picked = Pick(m.get(), d.get(), view_cam, 0.5f, 0.5f, 1.0f, &opt, 0.2f);
  EXPECT_EQ(picked.geom, near_left_geom);

  // NaN tolerance disables ring sampling and returns the exact center hit.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const PickResult nan_picked = Pick(m.get(), d.get(), view_cam, 0.5f, 0.5f, 1.0f, &opt, nan);
  EXPECT_EQ(nan_picked.geom, ground_geom);
}

// An infinite floor and a free box at z = 2 whose x is the single slide joint,
// plus a top-down orthographic fixed camera spanning world X in [-2, +10].
constexpr const char* kSceneXml = R"xml(
<mujoco>
  <statistic extent="1.5" center="0 0 1"/>
  <worldbody>
    <camera name="top" pos="4 0 6" xyaxes="1 0 0 0 1 0"
            projection="orthographic" fovy="12"/>
    <geom name="floor" type="plane" size="0 0 1"/>
    <body name="box" pos="0 0 2">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <geom name="box" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
)xml";

class PickBatchTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char error[256] = "";
    mjSpec* spec = mj_parseXMLString(kSceneXml, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;
    holder_ = ModelHolder::FromSpec(spec);
    ASSERT_TRUE(holder_->ok()) << holder_->error();
    m_ = holder_->model();
    d_ = holder_->data();
    scratch_data_.reset(mj_makeData(m_));
    box_body_ = mj_name2id(m_, mjOBJ_BODY, "box");
    floor_ = mj_name2id(m_, mjOBJ_GEOM, "floor");
    box_ = mj_name2id(m_, mjOBJ_GEOM, "box");
    mjv_defaultOption(&opt_);
    mjv_defaultFreeCamera(m_, &camera_);
    camera_.distance = 2.0;
    camera_.azimuth = 90.0;  // Looking along -y, x to the right.
    camera_.elevation = -20.0;

    // Three simulations in a row along x, one slide unit apart; sim 0 active at
    // the origin, simulation w's box at local x = w.
    for (int w = 0; w < 3; ++w) {
      offsets_.push_back(3.0f * w);
      offsets_.push_back(0.0f);
      offsets_.push_back(0.0f);
      states_.push_back(w);  // qpos (mjSTATE_QPOS has nq = 1 entries).
    }
    batch_.nsim = 3;
    batch_.active_sim = 0;
    batch_.offsets = offsets_.data();
    batch_.state_sig = mjSTATE_QPOS;
    batch_.states = states_.data();
    mj_forward(m_, d_);
  }

  // Aims the camera's lookat at `point` in simulation `w` and picks the center.
  PickResult PickAt(int w, double x, double y, double z, int* sim) {
    camera_.lookat[0] = x + offsets_[3 * w];
    camera_.lookat[1] = y + offsets_[3 * w + 1];
    camera_.lookat[2] = z + offsets_[3 * w + 2];
    return PickBatch(m_, d_, scratch_data_.get(), batch_, &camera_, 0.5f, 0.5f, 1.0f, &opt_, sim);
  }

  std::unique_ptr<ModelHolder> holder_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  std::unique_ptr<mjData, void (*)(mjData*)> scratch_data_{nullptr, mj_deleteData};
  std::vector<float> offsets_;
  std::vector<mjtNum> states_;
  BatchView batch_;
  mjvCamera camera_{};
  mjvOption opt_{};
  int box_body_ = -1;
  int floor_ = -1;
  int box_ = -1;
};

TEST_F(PickBatchTest, HitsOtherSimInItsOwnCoordinates) {
  int sim = -1;
  PickResult picked = PickAt(2, 2.0, 0.0, 2.0, &sim);
  EXPECT_EQ(sim, 2);
  EXPECT_EQ(picked.body, box_body_);
  EXPECT_NEAR(picked.point[0], 2.0, 0.15);
  EXPECT_NEAR(picked.point[2], 2.0, 0.15);
}

TEST_F(PickBatchTest, OtherSimsInfinitePlaneIsNotPicked) {
  // Bare floor inside sim 2's cell: the one floor is picked, for sim 0.
  camera_.elevation = -90.0;
  int sim = -1;
  PickResult picked = PickAt(2, 0.8, 0.8, 0.0, &sim);
  EXPECT_EQ(sim, 0);
  EXPECT_EQ(picked.geom, floor_);
  // And the model is left as it was.
  EXPECT_GE(m_->geom_rgba[4 * floor_ + 3], 1.0f);
}

TEST_F(PickBatchTest, AppliesPerSimModelFieldsAndRestoresActive) {
  // Sim 1's box is five times bigger; aim 0.35 off its center, which the
  // base size would miss.
  std::vector<mjtNum> geom_size(3 * 3 * m_->ngeom);
  for (int w = 0; w < 3; ++w) {
    for (int i = 0; i < 3 * m_->ngeom; ++i) {
      geom_size[w * 3 * m_->ngeom + i] = m_->geom_size[i];
    }
  }
  for (int i = 0; i < 3; ++i) {
    geom_size[1 * 3 * m_->ngeom + 3 * box_ + i] = 0.5;
  }
  BatchedModelField fields[1] = {
      {"geom_size", geom_size.data(), static_cast<size_t>(3 * m_->ngeom) * sizeof(mjtNum)}};
  batch_.model_fields = fields;

  camera_.elevation = 0.0;
  int sim = -1;
  PickResult picked = PickAt(1, 1.35, 0.0, 2.0, &sim);
  EXPECT_EQ(sim, 1);
  EXPECT_EQ(picked.body, box_body_);
  // Active simulation's again.
  EXPECT_NEAR(m_->geom_size[3 * box_], 0.1, 1e-6);
}

TEST_F(PickBatchTest, OthersHiddenPicksActiveSimOnly) {
  batch_.others_hidden = true;
  int sim = -1;
  PickResult picked = PickAt(2, 2.0, 0.0, 2.0, &sim);
  EXPECT_EQ(sim, 0);
  EXPECT_EQ(picked.geom, floor_);  // Sim 0's floor behind sim 2's box.
  batch_.others_hidden = false;
  EXPECT_EQ(PickAt(2, 2.0, 0.0, 2.0, &sim).body, box_body_);
  EXPECT_EQ(sim, 2);
}

TEST_F(PickBatchTest, FixedCameraPicksAcrossBatch) {
  const int top_cam = mj_name2id(m_, mjOBJ_CAMERA, "top");
  ASSERT_GE(top_cam, 0);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = top_cam;

  // Orthographic camera at x = 4 with fovy = 12 maps world_x = 4 + (2*x - 1)*6:
  // sim 0 at world_x = 0 (x = 1/6), sim 1 at world_x = 4 (x = 0.5), and sim 2
  // at world_x = 8 (x = 5/6). PickResult::point is in the hit simulation's
  // local (un-offset) frame.
  int sim = -1;
  PickResult p0 = PickBatch(m_, d_, scratch_data_.get(), batch_, &camera_, 1.0f / 6.0f, 0.5f, 1.0f,
                            &opt_, &sim);
  EXPECT_EQ(sim, 0);
  EXPECT_EQ(p0.body, box_body_);
  EXPECT_NEAR(p0.point[0], 0.0, 1e-4);

  PickResult p1 =
      PickBatch(m_, d_, scratch_data_.get(), batch_, &camera_, 0.5f, 0.5f, 1.0f, &opt_, &sim);
  EXPECT_EQ(sim, 1);
  EXPECT_EQ(p1.body, box_body_);
  EXPECT_NEAR(p1.point[0], 1.0, 1e-4);

  PickResult p2 = PickBatch(m_, d_, scratch_data_.get(), batch_, &camera_, 5.0f / 6.0f, 0.5f, 1.0f,
                            &opt_, &sim);
  EXPECT_EQ(sim, 2);
  EXPECT_EQ(p2.body, box_body_);
  EXPECT_NEAR(p2.point[0], 2.0, 1e-4);

  // Also works when active_sim == -1.
  batch_.active_sim = -1;
  PickResult p2_no_active = PickBatch(m_, d_, scratch_data_.get(), batch_, &camera_, 5.0f / 6.0f,
                                      0.5f, 1.0f, &opt_, &sim);
  EXPECT_EQ(sim, 2);
  EXPECT_EQ(p2_no_active.body, box_body_);
  EXPECT_NEAR(p2_no_active.point[0], 2.0, 1e-4);
}

TEST_F(PickBatchTest, PicksInitiallyAsleepBodiesButNotOtherSimsStaticGeoms) {
  constexpr const char* kSleepXml = R"xml(
  <mujoco>
    <option>
      <flag sleep="enable"/>
    </option>
    <statistic extent="1.5" center="0 0 1"/>
    <worldbody>
      <geom name="floor" type="plane" size="0 0 1"/>
      <geom name="pillar" type="box" pos="0 0 2" size="0.1 0.1 0.1"/>
      <body name="sleeper" pos="0 0 2" sleep="init">
        <joint name="slide_x" type="slide" axis="1 0 0"/>
        <geom name="sleeper_box" type="box" size="0.1 0.1 0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )xml";

  char error[256] = "";
  mjSpec* spec = mj_parseXMLString(kSleepXml, nullptr, error, sizeof(error));
  ASSERT_NE(spec, nullptr) << error;
  holder_ = ModelHolder::FromSpec(spec);
  ASSERT_TRUE(holder_->ok()) << holder_->error();
  m_ = holder_->model();
  d_ = holder_->data();
  scratch_data_.reset(mj_makeData(m_));
  mj_forward(m_, d_);

  const int pillar = mj_name2id(m_, mjOBJ_GEOM, "pillar");
  const int sleeper = mj_name2id(m_, mjOBJ_BODY, "sleeper");
  ASSERT_GE(pillar, 0);
  ASSERT_GE(sleeper, 0);

  const float offsets[6] = {0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f};
  const mjtNum states[2] = {0.0, 0.8};
  std::vector<mjtNum> geom_pos(2 * 3 * m_->ngeom, 0.0);
  for (int w = 0; w < 2; ++w) {
    for (int i = 0; i < 3 * m_->ngeom; ++i) {
      geom_pos[w * 3 * m_->ngeom + i] = m_->geom_pos[i];
    }
  }
  geom_pos[1 * 3 * m_->ngeom + 3 * pillar + 0] = -0.8;

  BatchedModelField fields[1] = {
      {"geom_pos", geom_pos.data(), static_cast<size_t>(3 * m_->ngeom) * sizeof(mjtNum)},
  };

  BatchView batch;
  batch.nsim = 2;
  batch.active_sim = 0;
  batch.offsets = offsets;
  batch.state_sig = mjSTATE_QPOS;
  batch.states = states;
  batch.model_fields = fields;

  mjv_defaultFreeCamera(m_, &camera_);
  camera_.distance = 2.0;
  camera_.azimuth = 90.0;
  camera_.elevation = 0.0;

  // Sim 1's shifted static world geom at local x = -0.8 (world x = 2.2) is
  // not drawn, so it is not picked.
  camera_.lookat[0] = 3.0 - 0.8;
  camera_.lookat[1] = 0.0;
  camera_.lookat[2] = 2.0;
  int sim = 7;
  PickResult miss_pillar =
      PickBatch(m_, d_, scratch_data_.get(), batch, &camera_, 0.5f, 0.5f, 1.0f, &opt_, &sim);
  EXPECT_EQ(sim, -1);
  EXPECT_LT(miss_pillar.dist, 0);

  // Hit sim 1's sleep="init" dynamic body at qpos = +0.8 (world x = 3.8).
  camera_.lookat[0] = 3.0 + 0.8;
  PickResult hit_sleeper =
      PickBatch(m_, d_, scratch_data_.get(), batch, &camera_, 0.5f, 0.5f, 1.0f, &opt_, &sim);
  EXPECT_EQ(sim, 1);
  EXPECT_EQ(hit_sleeper.body, sleeper);
}

TEST_F(PickBatchTest, WithoutAnActiveSimEveryRowIsPickedAndTheModelKept) {
  // (m, d) is not a row: the three rows are all "others", and a hit on (m, d)
  // reports sim -1. Sim 1's box is enlarged through a field; afterwards the
  // model holds its own size again, not sim 0's row.
  batch_.active_sim = -1;
  std::vector<mjtNum> geom_size(3 * 3 * m_->ngeom);
  for (int w = 0; w < 3; ++w) {
    for (int i = 0; i < 3 * m_->ngeom; ++i) {
      geom_size[w * 3 * m_->ngeom + i] = m_->geom_size[i];
    }
  }
  for (int i = 0; i < 3; ++i) {
    geom_size[1 * 3 * m_->ngeom + 3 * box_ + i] = 0.5;
  }
  BatchedModelField fields[1] = {
      {"geom_size", geom_size.data(), static_cast<size_t>(3 * m_->ngeom) * sizeof(mjtNum)}};
  batch_.model_fields = fields;
  m_->geom_size[3 * box_] = 0.2;  // The model's own, unlike any row.

  camera_.elevation = 0.0;
  int sim = -1;
  EXPECT_EQ(PickAt(1, 1.35, 0.0, 2.0, &sim).body, box_body_);
  EXPECT_EQ(sim, 1);
  EXPECT_EQ(PickAt(2, 2.0, 0.0, 2.0, &sim).body, box_body_);
  EXPECT_EQ(sim, 2);
  camera_.elevation = -90.0;
  sim = 7;
  EXPECT_EQ(PickAt(2, 0.8, 0.8, 0.0, &sim).geom, floor_);
  EXPECT_EQ(sim, -1);
  EXPECT_NEAR(m_->geom_size[3 * box_], 0.2, 1e-6);
}

TEST_F(PickBatchTest, MissSetsSimToMinusOneAndAllowsNullSim) {
  PickResult hit_null_sim = PickAt(2, 2.0, 0.0, 2.0, nullptr);
  EXPECT_EQ(hit_null_sim.body, box_body_);

  // Aim into empty sky above all boxes and the floor.
  camera_.elevation = 45.0;
  int sim = 7;
  PickResult miss = PickAt(0, 0.0, 0.0, 5.0, &sim);
  EXPECT_LT(miss.dist, 0);
  EXPECT_EQ(sim, -1);
  EXPECT_LT(PickAt(0, 0.0, 0.0, 5.0, nullptr).dist, 0);

  // Also check the early-return path when others_hidden is set.
  batch_.others_hidden = true;
  sim = 7;
  PickResult solo_miss = PickAt(0, 0.0, 0.0, 5.0, &sim);
  EXPECT_LT(solo_miss.dist, 0);
  EXPECT_EQ(sim, -1);
}

TEST_F(PickBatchTest, PicksPosedActiveSimAtItsGridCell) {
  // Make sim 2 active at offsets_[2] = (6, 0, 0): its local box at (2, 0, 2)
  // sits at world (8, 0, 2).
  batch_.active_sim = 2;
  d_->qpos[0] = 2.0;
  mj_forward(m_, d_);

  int sim = -1;
  PickResult picked = PickAt(2, 2.0, 0.0, 2.0, &sim);
  EXPECT_EQ(sim, 2);
  EXPECT_EQ(picked.body, box_body_);
  EXPECT_NEAR(picked.point[0], 2.0, 0.15);
  EXPECT_NEAR(picked.point[1], 0.0, 0.15);
  EXPECT_NEAR(picked.point[2], 2.0, 0.15);

  // Sim 0 (now passive at the origin) is still picked at (0, 0, 2).
  PickResult hit0 = PickAt(0, 0.0, 0.0, 2.0, &sim);
  EXPECT_EQ(sim, 0);
  EXPECT_EQ(hit0.body, box_body_);
  EXPECT_NEAR(hit0.point[0], 0.0, 0.15);

  // Fixed camera on posed active sim sees active sim 2's box at local x = 2
  // (camera center is local x = 4, span [-2, 10] -> x = 1/3).
  const int top_cam = mj_name2id(m_, mjOBJ_CAMERA, "top");
  ASSERT_GE(top_cam, 0);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = top_cam;
  PickResult fixed_hit = PickBatch(m_, d_, scratch_data_.get(), batch_, &camera_, 1.0f / 3.0f, 0.5f,
                                   1.0f, &opt_, &sim);
  EXPECT_EQ(sim, 2);
  EXPECT_EQ(fixed_hit.body, box_body_);
  EXPECT_NEAR(fixed_hit.point[0], 2.0, 1e-4);
}

}  // namespace
}  // namespace mujoco::studio
