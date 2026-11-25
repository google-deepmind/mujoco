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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mujoco::studio {

void TouchGestureTracker::BeginFrame(float width, float height) {
  SetWindowSize(width, height);
  state_.just_started = false;
  state_.pinch_log_scale = 0.0f;
  state_.pan_dx_px = 0.0f;
  state_.pan_dy_px = 0.0f;
  state_.twist_angle = 0.0f;
  active_this_frame_ = (mode_ == Mode::kPinchPan);
  state_.active = active_this_frame_;
  suppress_touch_mouse_ = (mode_ == Mode::kPinchPan || mode_ == Mode::kCooldown);
}

void TouchGestureTracker::SetWindowSize(float width, float height) {
  if (!std::isfinite(width) || !std::isfinite(height)) {
    return;
  }
  width_ = std::max(0.0f, width);
  height_ = std::max(0.0f, height);
}

TouchGestureTracker::PairMetrics TouchGestureTracker::MeasurePrimaryPair() const {
  const Finger& f0 = fingers_[0];
  const Finger& f1 = fingers_[1];
  const float dx = f1.x_px - f0.x_px;
  const float dy = f1.y_px - f0.y_px;
  return {
      .dx = dx,
      .dy = dy,
      .span = std::hypot(dx, dy),
      .mid_x = 0.5f * (f0.x_px + f1.x_px),
      .mid_y = 0.5f * (f0.y_px + f1.y_px),
  };
}

void TouchGestureTracker::AnchorPrimaryPair() {
  const PairMetrics curr = MeasurePrimaryPair();
  prev_dx_ = curr.dx;
  prev_dy_ = curr.dy;
  prev_span_ = curr.span;
  prev_mid_x_ = curr.mid_x;
  prev_mid_y_ = curr.mid_y;
}

void TouchGestureTracker::FlushPinchDelta() {
  if (mode_ != Mode::kPinchPan || fingers_.size() < 2) {
    return;
  }
  const PairMetrics curr = MeasurePrimaryPair();
  const float clamped_prev = std::max(prev_span_, kMinPinchSpanPx);
  const float clamped_curr = std::max(curr.span, kMinPinchSpanPx);
  state_.pinch_log_scale += std::log(clamped_curr / clamped_prev);
  state_.pan_dx_px += curr.mid_x - prev_mid_x_;
  state_.pan_dy_px += curr.mid_y - prev_mid_y_;
  if (prev_span_ >= kMinPinchSpanPx && curr.span >= kMinPinchSpanPx) {
    const float cross = prev_dx_ * curr.dy - prev_dy_ * curr.dx;
    const float dot = prev_dx_ * curr.dx + prev_dy_ * curr.dy;
    state_.twist_angle += std::atan2(cross, dot);
  }
  prev_dx_ = curr.dx;
  prev_dy_ = curr.dy;
  prev_span_ = curr.span;
  prev_mid_x_ = curr.mid_x;
  prev_mid_y_ = curr.mid_y;
}

void TouchGestureTracker::OnFingerDown(int64_t finger_id, float norm_x, float norm_y) {
  if (!std::isfinite(norm_x) || !std::isfinite(norm_y) || width_ <= 0.0f || height_ <= 0.0f) {
    return;
  }
  for (const Finger& f : fingers_) {
    if (f.id == finger_id) {
      OnFingerUp(finger_id);
      break;
    }
  }

  const float x_px = norm_x * width_;
  const float y_px = norm_y * height_;
  fingers_.push_back({.id = finger_id, .x_px = x_px, .y_px = y_px});

  if (fingers_.size() == 1 && mode_ == Mode::kIdle) {
    mode_ = Mode::kSingle;
  } else if (fingers_.size() >= 2 && mode_ != Mode::kPinchPan) {
    mode_ = Mode::kPinchPan;
    suppress_touch_mouse_ = true;
    AnchorPrimaryPair();
    state_.active = true;
    if (!active_this_frame_) {
      active_this_frame_ = true;
      state_.just_started = true;
      state_.origin_x = prev_mid_x_;
      state_.origin_y = prev_mid_y_;
    }
  }
}

void TouchGestureTracker::OnFingerMotion(int64_t finger_id, float norm_x, float norm_y) {
  if (!std::isfinite(norm_x) || !std::isfinite(norm_y) || width_ <= 0.0f || height_ <= 0.0f) {
    return;
  }
  for (Finger& f : fingers_) {
    if (f.id == finger_id) {
      f.x_px = norm_x * width_;
      f.y_px = norm_y * height_;
      return;
    }
  }
}

void TouchGestureTracker::OnFingerUp(int64_t finger_id) {
  auto it = std::find_if(fingers_.begin(), fingers_.end(),
                         [finger_id](const Finger& f) { return f.id == finger_id; });
  if (it == fingers_.end()) {
    return;
  }
  const size_t idx = static_cast<size_t>(it - fingers_.begin());
  const bool primary_changed = (idx < 2);

  if (mode_ == Mode::kPinchPan && fingers_.size() >= 2 && primary_changed) {
    FlushPinchDelta();
  }

  fingers_.erase(it);

  if (fingers_.empty()) {
    mode_ = Mode::kIdle;
    state_.active = false;
  } else if (fingers_.size() == 1) {
    if (mode_ == Mode::kPinchPan) {
      mode_ = Mode::kCooldown;
      suppress_touch_mouse_ = true;
      state_.active = false;
    }
  } else if (mode_ == Mode::kPinchPan && primary_changed) {
    // 3+ -> 2+ fingers where one of the primary pair lifted: re-anchor to the
    // remaining primary pair without emitting a jump.
    AnchorPrimaryPair();
  }
}

void TouchGestureTracker::EndFrame() {
  if (mode_ == Mode::kPinchPan && fingers_.size() >= 2) {
    FlushPinchDelta();
    state_.active = true;
  } else {
    state_.active = active_this_frame_ && (state_.just_started || state_.pinch_log_scale != 0.0f ||
                                           state_.pan_dx_px != 0.0f || state_.pan_dy_px != 0.0f ||
                                           state_.twist_angle != 0.0f);
  }
}

void TouchGestureTracker::Reset() {
  fingers_.clear();
  mode_ = Mode::kIdle;
  prev_dx_ = 0.0f;
  prev_dy_ = 0.0f;
  prev_span_ = 0.0f;
  prev_mid_x_ = 0.0f;
  prev_mid_y_ = 0.0f;
  suppress_touch_mouse_ = false;
  active_this_frame_ = false;
  state_ = {};
}

}  // namespace mujoco::studio
