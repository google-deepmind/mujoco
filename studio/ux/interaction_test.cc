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
#include <memory>

#include "testing/base/public/gunit.h"
#include <mujoco/mujoco.h>

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

  // The mjvCamera* overload does not implicitly apply portrait FOV so non-Studio
  // callers using unmodified mjv_camera2GLCamera keep matching their render.
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

}  // namespace
}  // namespace mujoco::studio
