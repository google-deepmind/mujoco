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

#include "studio/ux/camera_control.h"

#include <cmath>
#include <initializer_list>

#include "testing/base/public/gunit.h"
#include <mujoco/mujoco.h>

namespace mujoco::studio {
namespace {

constexpr char kXml[] = R"(
<mujoco>
  <worldbody>
    <geom size="1"/>
  </worldbody>
</mujoco>
)";

// A mouse drag's motion, as a fraction of the view size.
constexpr float kDx = 0.1f;
constexpr float kDy = 0.2f;

class CameraControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char error[1000] = "";
    mjSpec* spec = mj_parseXMLString(kXml, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;
    model_ = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    ASSERT_NE(model_, nullptr);
    mjv_defaultFreeCamera(model_, &camera_);
  }

  void TearDown() override { mj_deleteModel(model_); }

  // One mjv_moveCamera call.
  struct Motion {
    mjtMouse action;
    mjtNum dx;
    mjtNum dy;
  };

  // Returns the camera as moved by mjv_moveCamera with `motions`, in order.
  mjvCamera Moved(std::initializer_list<Motion> motions) const {
    mjvCamera camera = camera_;
    for (const Motion& motion : motions) {
      mjv_moveCamera(model_, motion.action, motion.dx, motion.dy, &camera);
    }
    return camera;
  }

  static void ExpectSameCamera(const mjvCamera& actual, const mjvCamera& expected) {
    EXPECT_DOUBLE_EQ(actual.lookat[0], expected.lookat[0]);
    EXPECT_DOUBLE_EQ(actual.lookat[1], expected.lookat[1]);
    EXPECT_DOUBLE_EQ(actual.lookat[2], expected.lookat[2]);
    EXPECT_DOUBLE_EQ(actual.distance, expected.distance);
    EXPECT_DOUBLE_EQ(actual.azimuth, expected.azimuth);
    EXPECT_DOUBLE_EQ(actual.elevation, expected.elevation);
  }

  static CameraInput Drag(float dx, float dy) {
    CameraInput input;
    input.mouse_dx = dx;
    input.mouse_dy = dy;
    input.mouse_focus = true;
    return input;
  }

  mjModel* model_ = nullptr;
  mjvCamera camera_;
};

TEST_F(CameraControlTest, LeftDragOrbits) {
  CameraInput input = Drag(kDx, kDy);
  input.left_button = kButtonPressed;
  const mjvCamera expected = Moved({{mjMOUSE_ROTATE_H, kDx, 0}, {mjMOUSE_ROTATE_V, 0, kDy}});

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected);
}

TEST_F(CameraControlTest, LeftDragTurnsTheFlyCamera) {
  CameraInput input = Drag(kDx, kDy);
  input.left_button = kButtonPressed;
  const mjvCamera expected = Moved({{mjMOUSE_TURN_H, kDx, 0}, {mjMOUSE_TURN_V, 0, kDy}});

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_, /*fly_camera=*/true, input));
  ExpectSameCamera(camera_, expected);
}

TEST_F(CameraControlTest, MiddleDragZoomsExceptTheFlyCamera) {
  CameraInput input = Drag(kDx, kDy);
  input.middle_button = kButtonPressed;
  const mjvCamera unmoved = camera_;
  const mjvCamera expected = Moved({{mjMOUSE_ZOOM, 0, kDy}});

  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/true, input));
  ExpectSameCamera(camera_, unmoved);
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected);
}

TEST_F(CameraControlTest, RightDragMovesInThePlaneShiftPicks) {
  CameraInput input = Drag(kDx, kDy);
  input.right_button = kButtonPressed;
  input.shift = true;
  const mjvCamera expected = Moved({{mjMOUSE_MOVE_H, kDx, kDy}});

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected);
}

TEST_F(CameraControlTest, RightDragScalesVerticalPlanarPanInPortrait) {
  constexpr float kPortraitAspect = 0.5f;

  // Vertical-plane pan (Shift released) scales dy by 1 / aspect_ratio < 1.
  CameraInput portrait_v = Drag(kDx, kDy);
  portrait_v.right_button = kButtonPressed;
  portrait_v.aspect_ratio = kPortraitAspect;
  const mjvCamera expected_v = Moved({{mjMOUSE_MOVE_V, kDx, kDy / kPortraitAspect}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_, /*fly_camera=*/false, portrait_v));
  ExpectSameCamera(camera_, expected_v);

  // Horizontal-plane pan (Shift held) also scales dy by 1 / aspect_ratio < 1.
  mjv_defaultFreeCamera(model_, &camera_);
  CameraInput portrait_h = Drag(kDx, kDy);
  portrait_h.right_button = kButtonPressed;
  portrait_h.shift = true;
  portrait_h.aspect_ratio = kPortraitAspect;
  const mjvCamera expected_h = Moved({{mjMOUSE_MOVE_H, kDx, kDy / kPortraitAspect}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_, /*fly_camera=*/false, portrait_h));
  ExpectSameCamera(camera_, expected_h);

  // Landscape (> 1:1) leaves dy unchanged.
  mjv_defaultFreeCamera(model_, &camera_);
  CameraInput landscape = Drag(kDx, kDy);
  landscape.right_button = kButtonPressed;
  landscape.aspect_ratio = 16.0f / 9.0f;
  const mjvCamera expected_landscape = Moved({{mjMOUSE_MOVE_V, kDx, kDy}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_, /*fly_camera=*/false, landscape));
  ExpectSameCamera(camera_, expected_landscape);
}

TEST_F(CameraControlTest, RightDragMovesByTheOrthographicViewHeight) {
  camera_.orthographic = 1;
  CameraInput input = Drag(0, kDy);
  input.right_button = kButtonPressed;
  input.shift = true;
  const mjvCamera before = camera_;
  // The view is as tall as the perspective view at the lookat point.
  const mjtNum view_height = 2 * camera_.distance * mju_tan(model_->vis.global.fovy * mjPI / 360.0);

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  mjtNum moved[3];
  mju_sub3(moved, camera_.lookat, before.lookat);
  EXPECT_NEAR(mju_norm3(moved), kDy * view_height, 1e-5 * view_height);
}

TEST_F(CameraControlTest, CtrlDragMovesNothing) {
  CameraInput input = Drag(kDx, kDy);
  input.left_button = kButtonPressed;
  input.ctrl = true;
  const mjvCamera unmoved = camera_;

  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, unmoved);
}

TEST_F(CameraControlTest, ReleasedButtonMovesNothing) {
  CameraInput input = Drag(kDx, kDy);
  input.left_button = kButtonReleased;
  const mjvCamera unmoved = camera_;

  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, unmoved);
}

TEST_F(CameraControlTest, MouseWithoutFocusMovesNothing) {
  CameraInput input = Drag(kDx, kDy);
  input.left_button = kButtonPressed;
  input.mouse_wheel = 1;
  input.mouse_focus = false;
  const mjvCamera unmoved = camera_;

  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, unmoved);
}

TEST_F(CameraControlTest, WheelZoomsExceptTheFlyCamera) {
  CameraInput input = Drag(0, 0);
  input.mouse_wheel = 2;
  const mjvCamera unmoved = camera_;
  const mjvCamera expected = Moved({{mjMOUSE_ZOOM, 0, -2 / 50.0f}});

  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/true, input));
  ExpectSameCamera(camera_, unmoved);
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected);
}

TEST_F(CameraControlTest, PinchZoomsTumbleAndDolliesFlyCamera) {
  constexpr float kPinch = 0.6f;
  CameraInput input = Drag(0, 0);
  input.pinch_zoom = kPinch;

  const mjvCamera expected_tumble = Moved({{mjMOUSE_ZOOM, 0, kPinch / 3.0f}});
  const mjtNum initial_distance = camera_.distance;
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected_tumble);
  EXPECT_LT(camera_.distance, initial_distance);

  mjv_defaultFreeCamera(model_, &camera_);
  const mjvCamera expected_fly = Moved({{mjMOUSE_MOVE_H_REL, 0, model_->stat.extent * kPinch}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/true, input));
  ExpectSameCamera(camera_, expected_fly);
}

TEST_F(CameraControlTest, TwoFingerPanMovesInPlaneShiftPicks) {
  CameraInput input = Drag(0, 0);
  input.pan_dx = kDx;
  input.pan_dy = kDy;
  const mjvCamera expected_v = Moved({{mjMOUSE_MOVE_V, kDx, kDy}});

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected_v);

  mjv_defaultFreeCamera(model_, &camera_);
  input.shift = true;
  const mjvCamera expected_h = Moved({{mjMOUSE_MOVE_H, kDx, kDy}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected_h);

  // Portrait viewports (< 1:1) scale pan_dy by 1 / aspect_ratio, matching right drag.
  constexpr float kPortraitAspect = 0.5f;
  mjv_defaultFreeCamera(model_, &camera_);
  input.shift = false;
  input.aspect_ratio = kPortraitAspect;
  const mjvCamera expected_portrait = Moved({{mjMOUSE_MOVE_V, kDx, kDy / kPortraitAspect}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, expected_portrait);
}

TEST_F(CameraControlTest, TouchPreservesOneFingerOrbitAndTwoFingerPan) {
  CameraInput one_finger = Drag(kDx, kDy);
  one_finger.left_button = kButtonPressed;
  const mjvCamera expected_orbit = Moved({{mjMOUSE_ROTATE_H, kDx, 0}, {mjMOUSE_ROTATE_V, 0, kDy}});

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, one_finger));
  ExpectSameCamera(camera_, expected_orbit);

  mjv_defaultFreeCamera(model_, &camera_);
  constexpr float kPortraitAspect = 0.5f;
  CameraInput two_finger_pan = Drag(0, 0);
  two_finger_pan.pan_dx = kDx;
  two_finger_pan.pan_dy = kDy;
  two_finger_pan.aspect_ratio = kPortraitAspect;
  const mjvCamera expected_pan = Moved({{mjMOUSE_MOVE_V, kDx, kDy / kPortraitAspect}});

  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, two_finger_pan));
  ExpectSameCamera(camera_, expected_pan);
}

TEST_F(CameraControlTest, TwoFingerTwistSpinsTurntableWeightedByElevation) {
  constexpr float kTwistRad = 0.4f;  // Clockwise screen twist (~22.92 deg).

  // 1. Ctrl-held ignores twist_angle (left to perturbation).
  camera_.azimuth = 10.0;
  camera_.elevation = -60.0;
  CameraInput ctrl_twist = Drag(0, 0);
  ctrl_twist.twist_angle = kTwistRad;
  ctrl_twist.ctrl = true;
  const mjvCamera unmoved = camera_;
  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/false, ctrl_twist));
  ExpectSameCamera(camera_, unmoved);

  // 2. Top-down view (elevation = -89 deg): clockwise twist increases azimuth
  // nearly 1:1 (-sin(-89 deg) * twist_deg > 0).
  camera_.azimuth = 10.0;
  camera_.elevation = -89.0;
  CameraInput touch_twist = Drag(0, 0);
  touch_twist.twist_angle = kTwistRad;
  const mjtNum top_reldx =
      static_cast<mjtNum>(std::sin(-89.0 * (mjPI / 180.0)) * (kTwistRad / mjPI));
  const mjvCamera expected_top = Moved({{mjMOUSE_ROTATE_H, top_reldx, 0}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, touch_twist));
  ExpectSameCamera(camera_, expected_top);
  EXPECT_NEAR(static_cast<double>(camera_.azimuth),
              10.0 - std::sin(-89.0 * (mjPI / 180.0)) * (kTwistRad * 180.0 / mjPI), 1e-4);

  // 3. Bottom-up view (elevation = +89 deg): clockwise twist flips azimuth sign
  // (-sin(+89 deg) * twist_deg < 0) so the ceiling still turns with the fingers.
  camera_.azimuth = 10.0;
  camera_.elevation = 89.0;
  const mjtNum bottom_reldx =
      static_cast<mjtNum>(std::sin(89.0 * (mjPI / 180.0)) * (kTwistRad / mjPI));
  const mjvCamera expected_bottom = Moved({{mjMOUSE_ROTATE_H, bottom_reldx, 0}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, touch_twist));
  ExpectSameCamera(camera_, expected_bottom);
  EXPECT_LT(camera_.azimuth, 10.0);

  // 4. Horizon view (elevation = 0 deg): -sin(0) == 0, so twist moves nothing.
  camera_.azimuth = 10.0;
  camera_.elevation = 0.0;
  const mjvCamera horizon_unmoved = camera_;
  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/false, touch_twist));
  ExpectSameCamera(camera_, horizon_unmoved);

  // 5. Fly camera turns azimuth with mjMOUSE_TURN_H, and simultaneous pan +
  // pinch + twist in tumble mode composes all three gestures.
  mjv_defaultFreeCamera(model_, &camera_);
  camera_.elevation = -45.0;
  const mjtNum fly_reldx =
      static_cast<mjtNum>(std::sin(-45.0 * (mjPI / 180.0)) * (kTwistRad / mjPI));
  const mjvCamera expected_fly = Moved({{mjMOUSE_TURN_H, fly_reldx, 0}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/true, touch_twist));
  ExpectSameCamera(camera_, expected_fly);

  mjv_defaultFreeCamera(model_, &camera_);
  camera_.elevation = -45.0;
  CameraInput combined = touch_twist;
  combined.pan_dx = kDx;
  combined.pan_dy = kDy;
  combined.pinch_zoom = 0.6f;
  const mjvCamera expected_combined = Moved({{mjMOUSE_MOVE_V, kDx, kDy},
                                             {mjMOUSE_ZOOM, 0, 0.6f / 3.0f},
                                             {mjMOUSE_ROTATE_H, fly_reldx, 0}});
  EXPECT_TRUE(MoveCameraWithMouse(model_, &camera_,
                                  /*fly_camera=*/false, combined));
  ExpectSameCamera(camera_, expected_combined);
}

TEST_F(CameraControlTest, PinchAndPanWithoutFocusMoveNothing) {
  CameraInput input;
  input.pinch_zoom = 0.5f;
  input.pan_dx = kDx;
  input.pan_dy = kDy;
  input.mouse_focus = false;
  const mjvCamera unmoved = camera_;

  EXPECT_FALSE(MoveCameraWithMouse(model_, &camera_,
                                   /*fly_camera=*/false, input));
  ExpectSameCamera(camera_, unmoved);
}

TEST_F(CameraControlTest, KeysFlyFasterWhileHeldAndResetOnRelease) {
  CameraInput input;
  input.keyboard_focus = true;
  input.fly_forward = 1;
  const mjvCamera expected = Moved({{mjMOUSE_MOVE_H_REL, 0, kMinFlySpeed}});
  float speed = kMinFlySpeed;

  EXPECT_TRUE(MoveCameraWithKeys(model_, &camera_, input, &speed));
  ExpectSameCamera(camera_, expected);
  EXPECT_GT(speed, kMinFlySpeed);

  input.fly_forward = 0;
  EXPECT_FALSE(MoveCameraWithKeys(model_, &camera_, input, &speed));
  EXPECT_EQ(speed, kMinFlySpeed);
}

TEST_F(CameraControlTest, KeysFlyInAllHeldDirections) {
  CameraInput input;
  input.keyboard_focus = true;
  input.fly_right = -1;
  input.fly_forward = 1;
  input.fly_up = -1;
  const mjvCamera expected = Moved(
      {{mjMOUSE_MOVE_H_REL, -kMinFlySpeed, kMinFlySpeed}, {mjMOUSE_MOVE_V_REL, 0, -kMinFlySpeed}});
  float speed = kMinFlySpeed;

  EXPECT_TRUE(MoveCameraWithKeys(model_, &camera_, input, &speed));
  ExpectSameCamera(camera_, expected);
}

TEST_F(CameraControlTest, ShiftRaisesTheFlySpeedCap) {
  CameraInput input;
  input.keyboard_focus = true;
  input.fly_up = 1;
  float speed = kMinFlySpeed;
  for (int i = 0; i < 1000; ++i) {
    MoveCameraWithKeys(model_, &camera_, input, &speed);
  }
  const float max_speed = speed;

  input.shift = true;
  for (int i = 0; i < 1000; ++i) {
    MoveCameraWithKeys(model_, &camera_, input, &speed);
  }
  EXPECT_GT(speed, max_speed);
}

TEST_F(CameraControlTest, KeysWithoutFocusMoveNothing) {
  CameraInput input;
  input.fly_forward = 1;
  const mjvCamera unmoved = camera_;
  float speed = kMinFlySpeed;

  EXPECT_FALSE(MoveCameraWithKeys(model_, &camera_, input, &speed));
  ExpectSameCamera(camera_, unmoved);
}

}  // namespace
}  // namespace mujoco::studio
