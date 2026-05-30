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

#include "experimental/filament/compat/scene_bridge.h"

#include <memory>
#include <optional>
#include <utility>

#include <math/TMatHelpers.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/TVecHelpers.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/light_manager.h"
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/compat/scene_geom_util.h"
#include "experimental/filament/filament_util.h"
#include "experimental/filament/render_context_filament_cpp.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3;
using filament::math::mat4;

SceneBridge::SceneBridge(mjrfContext* ctx, const mjModel* model)
    : ctx_(ctx) {
  mjrSceneParams params;
  mjr_defaultSceneParams(&params);
  params.layer_mask = mjCAT_ALL;
  params.reflection_layer_mask = mjCAT_DYNAMIC | mjCAT_STATIC;
  scene_ = CreateScene(ctx_, params);
  model_objects_ = std::make_unique<ModelObjects>(model, ctx_);

  mjrf_configureSceneFromModel(scene_.get(), model);

  auto clear_color = ReadElement(model, "filament.clearColor",
                                 filament::math::float4(0, 0, 0, 1));
  mjrf_setClearColor(ctx_, &clear_color[0]);

  light_manager_ =
      std::make_unique<LightManager>(ctx_, scene_.get(), model_objects_.get());
}

SceneBridge::~SceneBridge() {
  light_manager_.reset();
  for (auto& iter : renderables_) {
    mjrf_removeRenderableFromScene(scene_.get(), iter.get());
  }
  renderables_.clear();
}

std::optional<float3> SceneBridge::ClipFromWorld(const float3& pos) const{
  const float4 clip_pos = clip_from_world_ * float4(pos, 1.0f);
  if (clip_pos.w == 0.0f) {
    return std::nullopt;
  }
  return clip_pos.xyz / clip_pos.w;
}

mat4 CalculateClipFromWorld(const mjrRect& viewport, const mjrCamera& cam) {
  const float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  const float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  const float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  const float3 cam_at = cam_pos + cam_fwd;
  const float aspect_ratio = (float)viewport.width / (float)viewport.height;
  const float halfwidth =
      cam.frustum_width
          ? cam.frustum_width
          : 0.5f * aspect_ratio * (cam.frustum_top - cam.frustum_bottom);
  const float left = cam.frustum_center - halfwidth;
  const float right = cam.frustum_center + halfwidth;

  mat4 projection;
  if (cam.orthographic) {
    projection = mat4::ortho(left, right, cam.frustum_bottom, cam.frustum_top,
                             cam.frustum_near, cam.frustum_far);
  } else {
    projection = mat4::frustum(left, right, cam.frustum_bottom, cam.frustum_top,
                               cam.frustum_near, cam.frustum_far);
    projection[2][2] = -1.0f;
    projection[3][2] = -2.0f * cam.frustum_near;
  }
  mat4 look_at = mat4::lookAt(cam_pos, cam_at, cam_up);
  return projection * inverse(look_at);
}

void SceneBridge::Update(const mjrRect& viewport, const mjvScene* scene) {
  mjtNum hpos[3], hfwd[3];
  float headpos[3], gazedir[3];
  mjv_cameraInModel(hpos, hfwd, nullptr, scene);
  mju_n2f(headpos, hpos, 3);
  mju_n2f(gazedir, hfwd, 3);

  camera_ = mjv_averageCamera(scene->camera, scene->camera + 1);
  clip_from_world_ = CalculateClipFromWorld(viewport, camera_);

  // Remove all drawables from previous render and prepare new ones.
  for (auto& iter : renderables_) {
    mjrf_removeRenderableFromScene(scene_.get(), iter.get());
  }
  renderables_.clear();
  for (int i = 0; i < scene->ngeom; ++i) {
    const mjvGeom* geom = scene->geoms + i;

    if (draw_text_callback_ && geom->label[0] != 0) {
      if (auto pos = ClipFromWorld(ReadFloat3(geom->pos))) {
        draw_text_callback_(geom->label, pos->x, pos->y, pos->z);
      }
    }

    if (geom->type == mjGEOM_FLEX || geom->type == mjGEOM_SKIN) {
      model_objects_->CreateSkinFlexMesh(scene, *geom);
    }

    UniquePtr<mjrRenderable> renderable =
        CreateGeomRenderable(*geom, ctx_, model_objects_.get(), scene->flags);

    mjrf_addRenderableToScene(scene_.get(), renderable.get());
    renderables_.push_back(std::move(renderable));
  }

  mjrLight* headlight = nullptr;
  bool headlight_enabled = false;
  for (int i = 0; i < scene->nlight; ++i) {
    const mjvLight& scene_light = scene->lights[i];
    if (scene_light.id < 0 && scene_light.headlight) {
      // The headlight, if it exists, is assigned the id `scene->nlight`.
      headlight = light_manager_->GetLight(scene->nlight);
      if (!headlight) {
        continue;
      }
      // We position the headlight slightly behind the camera to avoid some
      // odd clipping issues.
      headlight_enabled = true;
      headpos[0] -= gazedir[0] * 0.05f;
      headpos[1] -= gazedir[1] * 0.05f;
      headpos[2] -= gazedir[2] * 0.05f;

      mjrf_setLightColor(headlight, scene_light.diffuse);
      mjrf_setLightTransform(headlight, headpos, gazedir);
      continue;
    } else if (mjrLight* light = light_manager_->GetLight(scene_light.id)) {
      mjrf_setLightColor(light, scene_light.diffuse);
      mjrf_setLightTransform(light, scene_light.pos, scene_light.dir);
    } else {
      mju_error("Unexpected light id: %d", scene_light.id);
    }
  }

  // Enable/disable the headlight based on whether or not it's in the scene.
  if (headlight) {
    mjrf_setLightEnabled(headlight, headlight_enabled);
  }
}

void SceneBridge::UploadMesh(const mjModel* model, int id) {
  model_objects_->UploadMesh(model, id);
}

void SceneBridge::UploadTexture(const mjModel* model, int id) {
  model_objects_->UploadTexture(model, id);
}

void SceneBridge::UploadHeightField(const mjModel* model, int id) {
  model_objects_->UploadHeightField(model, id);
}

void SceneBridge::SetDrawTextFunction(DrawTextAtFn fn) {
  draw_text_callback_ = std::move(fn);
}

mjrScene* SceneBridge::GetScene() const { return scene_.get(); }

mjrCamera SceneBridge::GetCamera() const { return camera_; }

}  // namespace mujoco
