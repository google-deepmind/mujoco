// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_STUDIO_UX_INTERACTION_H_
#define MUJOCO_STUDIO_UX_INTERACTION_H_

#include <mujoco/mujoco.h>
#include "studio/sim/batch_view.h"

namespace mujoco::studio {

// Touch selection tolerance radius in logical CSS pixels for direct touch
// input, matching the 44x44 pt / CSS px minimum touch target diameter (22 px
// radius) recommended by Apple Human Interface Guidelines and WCAG 2.2 (2.5.5).
inline constexpr float kTouchPickTolerancePx = 22.0f;

// The result of a pick operation.
struct PickResult {
  mjtNum point[3] = {0, 0, 0};  // World coordinates
  mjtNum dist = -1;             // Distance from the camera.
  int body = -1;
  int geom = -1;
  int flex = -1;
  int skin = -1;
};

// Returns the vertical frustum expansion factor for `camera` in a viewport with
// the given `aspect_ratio` (width / height). For free and tracking cameras in
// portrait viewports (0 < aspect_ratio < 1), the authored fovy is applied along
// the shorter (horizontal) viewport axis, expanding the vertical frustum by
// 1 / aspect_ratio; otherwise returns 1.
float PortraitFovScale(const mjvCamera* camera, float aspect_ratio);

// Derives the GL camera for `camera` in a Studio primary viewport of the given
// `aspect_ratio` (width / height), applying the shorter-axis FOV rule for free
// and tracking cameras in portrait viewports. Fixed model cameras are left
// unchanged so they remain faithful to offscreen rendering.
mjrCamera ViewCamera(const mjModel* m, const mjData* d, const mjvCamera* camera,
                     float aspect_ratio);

// Returns information about the object (if any) under the mouse/touch cursor
// `(x, y)` in normalized `[0, 1]` viewport coordinates using an explicit GL
// camera (such as one returned by ViewCamera).
//
// When `tolerance > 0` (normalized vertical viewport radius as a fraction of
// viewport height, e.g. `kTouchPickTolerancePx / height`; horizontal radius is
// derived as `tolerance / aspect_ratio`):
//   1. An exact ray at `(x, y)` is cast first. If it hits the visible bounds of
//      any finite object (non-plane geom, flex, or skin), that hit is ALWAYS
//      returned immediately, even if `(x, y)` also lies within the padded
//      tolerance halo of another closer object.
//   2. Otherwise, concentric rings up to `(tolerance / aspect_ratio, tolerance)`
//      are sampled outward with true camera rays (respecting 3D occlusion),
//      preferring dynamic/articulated objects (`body > 0`) at the smallest
//      angular/screen-space offset from `(x, y)` and falling back to static
//      non-plane geoms (`body == 0`) or the exact center ray.
PickResult Pick(const mjModel* m, const mjData* d, const mjrCamera& camera, float x, float y,
                float aspect_ratio, const mjvOption* vis_options, float tolerance = 0.0f);

// Returns information about the object (if any) under the mouse/touch cursor
// using the unmodified mjv_camera2GLCamera projection for `camera`.
PickResult Pick(const mjModel* m, const mjData* d, const mjvCamera* camera, float x, float y,
                float aspect_ratio, const mjvOption* vis_options, float tolerance = 0.0f);

// Like Pick, but across all visible simulations in `batch`. If `sim` is
// non-null, writes the hit simulation index (or -1 on a miss).
PickResult PickBatch(mjModel* m, const mjData* d, mjData* scratch_data, const BatchView& batch,
                     const mjvCamera* camera, float x, float y, float aspect_ratio,
                     const mjvOption* vis_options, int* sim);

// Indices for cameras that are not defined in the model.
static constexpr int kTumbleCameraIdx = -3;
static constexpr int kFreeCameraIdx = -2;
static constexpr int kTrackingCameraIdx = -1;

// Updates the camera according to the requested index.
//
// The function returns the new index of the camera which may differ from the
// request if the request was invalid (e.g. request was out of range).
int SetCamera(const mjModel* m, mjvCamera* camera, int request_idx);

// Camera motions are either relative to a target or the camera itself.
//
// We use the following camera nomenclature:
//   - Truck: moves the camera left/right along a horizontal plane.
//   - Pedestal: moves the camera up/down along a vertical plane.
//   - Dolly: moves the camera forward/backward along a horizontal plane.
//   - Pan: turns the camera left/right.
//   - Tilt: turns the camera upwards/downwards.
//   - Zoom: moves the camera closer to or away from the target. This is
//           different from dolly in that the movement is relative to the
//           target. (It's also not actually a camera zoom, which is an
//           action of lens of the camera, rather than the camera itself.)
//   - Orbit: moves the camera around the target.
//   - Planer: creates a horizontal or vertical plane based on the cameras
//             position and orientation, then moves the camera along that plane.
enum class CameraMotion {
  ZOOM,
  ORBIT,
  TRUCK_PEDESTAL,
  TRUCK_DOLLY,
  PAN_TILT,
  PLANAR_MOVE_H,
  PLANAR_MOVE_V,
};

void MoveCamera(const mjModel* m, const mjData* d, mjvCamera* cam, CameraMotion motion, mjtNum dx,
                mjtNum dy, float aspect_ratio = 1.0f);

void InitPerturb(const mjModel* m, const mjData* d, const mjvCamera* cam, mjvPerturb* pert,
                 mjtPertBit active);

void MovePerturb(const mjModel* m, const mjData* d, const mjvCamera* cam, mjvPerturb* pert,
                 mjtMouse action, mjtNum reldx, mjtNum reldy, float aspect_ratio = 1.0f);

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_UX_INTERACTION_H_
