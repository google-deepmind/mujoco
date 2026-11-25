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

#ifndef MUJOCO_STUDIO_UX_TOUCH_GESTURE_H_
#define MUJOCO_STUDIO_UX_TOUCH_GESTURE_H_

#include <cstdint>
#include <vector>

namespace mujoco::studio {

// Snapshot of multi-touch pinch, two-finger pan, and twist state for the
// current frame.
struct TouchGestureState {
  bool active = false;        // True while 2+ fingers are down (PinchPan).
  bool just_started = false;  // True on the frame the gesture entered PinchPan.
  float origin_x = 0.0f;      // Initial 2-finger midpoint (logical pixels).
  float origin_y = 0.0f;
  float pinch_log_scale = 0.0f;  // ln(span_curr / span_prev); >0 zooms in.
  float pan_dx_px = 0.0f;        // Midpoint delta this frame (logical pixels).
  float pan_dy_px = 0.0f;
  float twist_angle = 0.0f;  // Clockwise 2-finger angle delta (radians).
};

// Tracks direct-touch contacts in isotropic logical window pixels, coalescing
// multi-finger updates once per frame and arbitrating against SDL2's
// single-finger touch-to-mouse synthesis (SDL_TOUCH_MOUSEID).
class TouchGestureTracker {
 public:
  // Minimum finger separation (in logical pixels) required to compute a pinch
  // zoom ratio; smaller spans are ignored to avoid singularities.
  static constexpr float kMinPinchSpanPx = 12.0f;

  // Starts a new frame with the window's logical pixel dimensions. Call before
  // feeding this frame's finger events.
  void BeginFrame(float width, float height);

  // Updates the window's logical pixel dimensions mid-frame (e.g. on resize)
  // before subsequent finger events are scaled.
  void SetWindowSize(float width, float height);

  // Feeds normalized [0, 1] direct-touch events for this frame.
  void OnFingerDown(int64_t finger_id, float norm_x, float norm_y);
  void OnFingerMotion(int64_t finger_id, float norm_x, float norm_y);
  void OnFingerUp(int64_t finger_id);

  // Finalizes this frame's coalesced pinch zoom and midpoint pan deltas.
  void EndFrame();

  // Clears all tracked fingers and resets to idle (e.g. on focus loss).
  void Reset();

  // True when SDL's synthesized touch-mouse events (SDL_TOUCH_MOUSEID) must be
  // suppressed for this frame (during PinchPan and LiftingCooldown).
  bool SuppressTouchMouse() const { return suppress_touch_mouse_; }

  const TouchGestureState& state() const { return state_; }

 private:
  enum class Mode {
    kIdle,      // 0 fingers down.
    kSingle,    // 1 finger down (SDL mouse passthrough active).
    kPinchPan,  // 2+ fingers down (pinch & midpoint pan active).
    kCooldown,  // 1 finger remaining after a pinch; suppress mouse until 0.
  };

  struct Finger {
    int64_t id = 0;
    float x_px = 0.0f;
    float y_px = 0.0f;
  };

  struct PairMetrics {
    float dx = 0.0f;
    float dy = 0.0f;
    float span = 0.0f;
    float mid_x = 0.0f;
    float mid_y = 0.0f;
  };

  PairMetrics MeasurePrimaryPair() const;
  void AnchorPrimaryPair();
  void FlushPinchDelta();

  Mode mode_ = Mode::kIdle;
  float width_ = 0.0f;
  float height_ = 0.0f;
  std::vector<Finger> fingers_;
  float prev_dx_ = 0.0f;
  float prev_dy_ = 0.0f;
  float prev_span_ = 0.0f;
  float prev_mid_x_ = 0.0f;
  float prev_mid_y_ = 0.0f;
  bool suppress_touch_mouse_ = false;
  bool active_this_frame_ = false;
  TouchGestureState state_;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_UX_TOUCH_GESTURE_H_
