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

#include "studio/ux/touch_gesture.h"

#include <cmath>
#include <limits>

#include "testing/base/public/gunit.h"

namespace mujoco::studio {
namespace {

constexpr float kPortraitW = 390.0f;
constexpr float kPortraitH = 844.0f;

TEST(TouchGestureTest, SingleFingerAllowsMouseSynthesisAndEmitsNoGesture) {
  TouchGestureTracker tracker;
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, 0.5f, 0.5f);
  tracker.OnFingerMotion(1, 0.6f, 0.6f);
  tracker.EndFrame();

  EXPECT_FALSE(tracker.SuppressTouchMouse());
  EXPECT_FALSE(tracker.state().active);
  EXPECT_FALSE(tracker.state().just_started);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dx_px, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dy_px, 0.0f);

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerUp(1);
  tracker.EndFrame();
  EXPECT_FALSE(tracker.SuppressTouchMouse());
}

TEST(TouchGestureTest, CoalescesTwoFingerPinchAndMidpointPanPerFrame) {
  TouchGestureTracker tracker;

  // Frame 1: two fingers touch down 100 px apart horizontally around (200, 400).
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, 150.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerDown(2, 250.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();

  EXPECT_TRUE(tracker.SuppressTouchMouse());
  EXPECT_TRUE(tracker.state().active);
  EXPECT_TRUE(tracker.state().just_started);
  EXPECT_FLOAT_EQ(tracker.state().origin_x, 200.0f);
  EXPECT_FLOAT_EQ(tracker.state().origin_y, 400.0f);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);

  // Frame 2: spread from 100 px to 200 px AND translate midpoint by (+15, -25) px,
  // arriving as two separate finger-motion events in the same frame.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(1, 115.0f / kPortraitW, 375.0f / kPortraitH);
  tracker.OnFingerMotion(2, 315.0f / kPortraitW, 375.0f / kPortraitH);
  tracker.EndFrame();

  EXPECT_TRUE(tracker.SuppressTouchMouse());
  EXPECT_TRUE(tracker.state().active);
  EXPECT_FALSE(tracker.state().just_started);
  EXPECT_FLOAT_EQ(tracker.state().origin_x, 200.0f);
  EXPECT_FLOAT_EQ(tracker.state().origin_y, 400.0f);
  EXPECT_NEAR(tracker.state().pinch_log_scale, std::log(2.0f), 1e-5f);
  EXPECT_NEAR(tracker.state().pan_dx_px, 15.0f, 1e-4f);
  EXPECT_NEAR(tracker.state().pan_dy_px, -25.0f, 1e-4f);
}

TEST(TouchGestureTest, PureRotationOnPortraitViewportTracksTwistWithZeroPinchZoom) {
  TouchGestureTracker tracker;
  const float cx = 195.0f;
  const float cy = 422.0f;
  const float r = 60.0f;  // Constant 120 px finger span.
  constexpr float kDegToRad = 3.1415926535f / 180.0f;

  // Start horizontal (0 deg).
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, (cx - r) / kPortraitW, cy / kPortraitH);
  tracker.OnFingerDown(2, (cx + r) / kPortraitW, cy / kPortraitH);
  tracker.EndFrame();
  EXPECT_FLOAT_EQ(tracker.state().twist_angle, 0.0f);

  // Rotate clockwise through 30, 45, 60, 90, 170, and 190 (-170) deg on a
  // 390x844 portrait screen; normalized [0, 1] Euclidean distance shrinks by
  // >2x, but pixel span stays 120 px and twist wraps smoothly across +/-180 deg.
  float prev_deg = 0.0f;
  for (float angle_deg : {30.0f, 45.0f, 60.0f, 90.0f, 170.0f, 190.0f, 150.0f}) {
    const float rad = angle_deg * kDegToRad;
    const float dx = r * std::cos(rad);
    const float dy = r * std::sin(rad);
    tracker.BeginFrame(kPortraitW, kPortraitH);
    tracker.OnFingerMotion(1, (cx - dx) / kPortraitW, (cy - dy) / kPortraitH);
    tracker.OnFingerMotion(2, (cx + dx) / kPortraitW, (cy + dy) / kPortraitH);
    tracker.EndFrame();
    EXPECT_NEAR(tracker.state().pinch_log_scale, 0.0f, 1e-5f);
    EXPECT_NEAR(tracker.state().pan_dx_px, 0.0f, 1e-4f);
    EXPECT_NEAR(tracker.state().pan_dy_px, 0.0f, 1e-4f);
    EXPECT_NEAR(tracker.state().twist_angle, (angle_deg - prev_deg) * kDegToRad, 1e-5f);
    prev_deg = angle_deg;
  }
}

TEST(TouchGestureTest, LiftingCooldownSuppressesTrailingFingerAndReanchorsCleanly) {
  TouchGestureTracker tracker;

  // Frame 1: 1 finger down (single-touch orbit allowed).
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, 100.0f / kPortraitW, 300.0f / kPortraitH);
  tracker.EndFrame();
  EXPECT_FALSE(tracker.SuppressTouchMouse());

  // Frame 2: 2nd finger down -> enters PinchPan and suppresses touch mouse.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(2, 200.0f / kPortraitW, 300.0f / kPortraitH);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.SuppressTouchMouse());
  EXPECT_TRUE(tracker.state().active);
  EXPECT_TRUE(tracker.state().just_started);

  // Frame 3: 2nd finger lifts, 1st finger still moves -> LiftingCooldown keeps
  // suppressing touch mouse and emits no camera delta.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerUp(2);
  tracker.OnFingerMotion(1, 140.0f / kPortraitW, 350.0f / kPortraitH);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.SuppressTouchMouse());
  EXPECT_FALSE(tracker.state().active);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dx_px, 0.0f);

  // Frame 4: 2nd finger touches back down at a new span (260 px from finger 1)
  // -> re-anchors without a delta jump.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(3, 400.0f / kPortraitW, 350.0f / kPortraitH);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.SuppressTouchMouse());
  EXPECT_TRUE(tracker.state().active);
  EXPECT_TRUE(tracker.state().just_started);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dx_px, 0.0f);

  // Frame 5: both fingers lift -> still suppresses trailing mouse-up on this
  // frame, then returns to idle on Frame 6.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerUp(1);
  tracker.OnFingerUp(3);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.SuppressTouchMouse());
  EXPECT_FALSE(tracker.state().active);

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.EndFrame();
  EXPECT_FALSE(tracker.SuppressTouchMouse());
}

TEST(TouchGestureTest, ThreeToTwoFingerLiftReanchorsWithoutJump) {
  TouchGestureTracker tracker;

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, 100.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerDown(2, 200.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerDown(3, 350.0f / kPortraitW, 500.0f / kPortraitH);
  tracker.EndFrame();

  // Finger 1 lifts, promoting (2, 3) as the primary pair; no jump emitted.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerUp(1);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.state().active);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dx_px, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dy_px, 0.0f);
}

TEST(TouchGestureTest, IgnoresDegenerateSmallSpansAndResetsOnFocusLoss) {
  TouchGestureTracker tracker;

  // Two fingers only 4 px apart (< kMinPinchSpanPx = 12 px): neither pinch nor
  // twist is emitted at degenerate spans.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, 200.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerDown(2, 204.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(2, 200.0f / kPortraitW, 408.0f / kPortraitH);
  tracker.EndFrame();
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().twist_angle, 0.0f);

  // Focus loss clears tracked fingers and mouse-suppression state immediately.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  EXPECT_TRUE(tracker.SuppressTouchMouse());
  tracker.Reset();
  EXPECT_FALSE(tracker.SuppressTouchMouse());
  tracker.EndFrame();
  EXPECT_FALSE(tracker.state().active);
  EXPECT_FALSE(tracker.SuppressTouchMouse());

  // Next frame allows normal single-finger taps again.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(10, 0.5f, 0.5f);
  tracker.EndFrame();
  EXPECT_FALSE(tracker.SuppressTouchMouse());
}

TEST(TouchGestureTest, ClampsCrossThresholdPinchWithoutDroppingOrRatcheting) {
  TouchGestureTracker tracker;

  // Start 100 px apart (150..250).
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(1, 150.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerDown(2, 250.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();

  // Fast pinch across kMinPinchSpanPx (12 px) from 100 px down to 8 px in one
  // frame: clamps at 12 px instead of dropping the frame's zoom.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(1, 196.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerMotion(2, 204.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();
  const float zoom_in = tracker.state().pinch_log_scale;
  EXPECT_NEAR(zoom_in, std::log(TouchGestureTracker::kMinPinchSpanPx / 100.0f), 1e-5f);

  // Spread back out slowly across two steps (8 -> 30 -> 100 px): net log-scale
  // over the closed cycle must sum to zero (no ratchet drift).
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(1, 185.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerMotion(2, 215.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();
  const float zoom_out_1 = tracker.state().pinch_log_scale;

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(1, 150.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerMotion(2, 250.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();
  const float zoom_out_2 = tracker.state().pinch_log_scale;

  EXPECT_NEAR(zoom_in + zoom_out_1 + zoom_out_2, 0.0f, 1e-5f);
}

TEST(TouchGestureTest, MidGestureWindowResizeKeepsStationaryFingersWithoutJump) {
  TouchGestureTracker tracker;

  // Start at CSS pixels (100, 400) and (300, 400) on 400x800 (span = 200 px).
  tracker.BeginFrame(400.0f, 800.0f);
  tracker.OnFingerDown(1, 100.0f / 400.0f, 400.0f / 800.0f);
  tracker.OnFingerDown(2, 300.0f / 400.0f, 400.0f / 800.0f);
  tracker.EndFrame();

  // Mid-frame: Finger 1 moves horizontally to x = 80, then the mobile address
  // bar shrinks the viewport height to 700 while both fingers stay at y = 400
  // CSS px (re-normalized by SDL as 400/700), and Finger 2 moves to x = 320.
  tracker.BeginFrame(400.0f, 800.0f);
  tracker.OnFingerMotion(1, 80.0f / 400.0f, 400.0f / 800.0f);
  tracker.SetWindowSize(400.0f, 700.0f);
  tracker.OnFingerMotion(2, 320.0f / 400.0f, 400.0f / 700.0f);
  tracker.EndFrame();

  EXPECT_TRUE(tracker.state().active);
  EXPECT_NEAR(tracker.state().pinch_log_scale, std::log(240.0f / 200.0f), 1e-5f);
  EXPECT_NEAR(tracker.state().pan_dx_px, 0.0f, 1e-4f);
  EXPECT_NEAR(tracker.state().pan_dy_px, 0.0f, 1e-4f);

  // Next frame: stationary Finger 1 reports its re-normalized y = 400/700.
  // Because SetWindowSize preserved Finger 1's CSS pixel anchor, no spurious
  // pan or zoom jump is injected.
  tracker.BeginFrame(400.0f, 700.0f);
  tracker.OnFingerMotion(1, 80.0f / 400.0f, 400.0f / 700.0f);
  tracker.EndFrame();

  EXPECT_TRUE(tracker.state().active);
  EXPECT_NEAR(tracker.state().pinch_log_scale, 0.0f, 1e-5f);
  EXPECT_NEAR(tracker.state().pan_dx_px, 0.0f, 1e-4f);
  EXPECT_NEAR(tracker.state().pan_dy_px, 0.0f, 1e-4f);
}

TEST(TouchGestureTest, RejectsNonFiniteInputsAndHandlesDuplicateAndSubframeEvents) {
  TouchGestureTracker tracker;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.SetWindowSize(inf, nan);
  tracker.OnFingerDown(1, 100.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerDown(2, 200.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.OnFingerMotion(1, nan, 0.5f);
  tracker.OnFingerMotion(2, 0.5f, inf);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.state().active);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);

  // Duplicate OnFingerDown for an existing primary finger re-anchors cleanly
  // after flushing prior motion rather than injecting a jump.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerDown(2, 300.0f / kPortraitW, 400.0f / kPortraitH);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.state().active);
  EXPECT_FLOAT_EQ(tracker.state().pinch_log_scale, 0.0f);
  EXPECT_FLOAT_EQ(tracker.state().pan_dx_px, 0.0f);

  // Sub-frame motion -> FingerUp (2->1) -> FingerDown (1->2) -> motion preserves
  // both pre-lift and post-reanchor deltas without jump.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(2, 350.0f / kPortraitW, 400.0f / kPortraitH);  // 200 -> 250 px
  tracker.OnFingerUp(2);
  tracker.OnFingerDown(3, 200.0f / kPortraitW, 400.0f / kPortraitH);    // Re-anchor at 100 px
  tracker.OnFingerMotion(3, 250.0f / kPortraitW, 400.0f / kPortraitH);  // 100 -> 150 px
  tracker.EndFrame();
  EXPECT_TRUE(tracker.state().active);
  EXPECT_FALSE(tracker.state().just_started);
  EXPECT_NEAR(tracker.state().pinch_log_scale,
              std::log(250.0f / 200.0f) + std::log(150.0f / 100.0f), 1e-5f);

  // Sub-frame motion -> FingerUp (2->1) at gesture end still delivers the final
  // motion delta on that frame, then deactivates on the next frame.
  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.OnFingerMotion(3, 300.0f / kPortraitW, 400.0f / kPortraitH);  // 150 -> 200 px
  tracker.OnFingerUp(3);
  tracker.EndFrame();
  EXPECT_TRUE(tracker.state().active);
  EXPECT_NEAR(tracker.state().pinch_log_scale, std::log(200.0f / 150.0f), 1e-5f);

  tracker.BeginFrame(kPortraitW, kPortraitH);
  tracker.EndFrame();
  EXPECT_FALSE(tracker.state().active);
  EXPECT_TRUE(tracker.SuppressTouchMouse());
}

}  // namespace
}  // namespace mujoco::studio
