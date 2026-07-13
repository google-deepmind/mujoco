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

#include "experimental/filament/compat/scene_decorator.h"

#include <algorithm>
#include <cstring>
#include <span>
#include <utility>

#include <math/TMatHelpers.h>
#include <math/mat4.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4;

SceneDecorator::SceneDecorator(mjrfScene* scene, ModelObjects* model_objects,
                               int num_geoms)
    : scene_(scene), model_objects_(model_objects) {
  std::memset(&mjv_scene_, 0, sizeof(mjvScene));
  mjv_makeScene(model_objects_->GetModel(), &mjv_scene_, 2000);
}

SceneDecorator::~SceneDecorator() {
  mjv_freeScene(&mjv_scene_);
}

static mat4 CalcClipFromWorld(const mjModel* model, const mjData* data,
                              mjvCamera mjv_camera, const mjrRect& viewport) {
  if (mjv_camera.type == mjCAMERA_TRACKING) {
    int bid = mjv_camera.trackbodyid;
    if (bid < 0 || bid >= model->nbody) {
      mju_error("track body id is outside valid range");
    }
    mju_copy3(mjv_camera.lookat, data->subtree_com + 3 * bid);
  }

  mjtNum headpos[3], forward[3], up[3];
  mjv_cameraFrame(headpos, forward, up, nullptr, data, &mjv_camera);
  float zver[2], zhor[2], zclip[2] = {0, 0};
  mjv_cameraFrustum(zver, zhor, zclip, model, &mjv_camera);

  const float frustum_top = zver[0];
  const float frustum_bottom = -zver[1];
  const float frustum_center = (zhor[1] - zhor[0]) / 2;
  const float frustum_width = (zhor[1] + zhor[0]) / 2;
  const float frustum_near = zclip[0];
  const float frustum_far = zclip[1];

  bool orthographic = model->vis.global.orthographic;
  if (mjv_camera.type == mjCAMERA_FIXED) {
    const int cid = mjv_camera.fixedcamid;
    if (cid < 0 || cid >= model->ncam) {
      mju_error("fixed camera id is outside valid range");
    }
    orthographic = model->cam_projection[cid] == mjPROJ_ORTHOGRAPHIC;
  }

  const float3 cam_pos{headpos[0], headpos[1], headpos[2]};
  const float3 cam_fwd{forward[0], forward[1], forward[2]};
  const float3 cam_up{up[0], up[1], up[2]};
  const float3 cam_at = cam_pos + cam_fwd;
  const float aspect_ratio = (float)viewport.width / (float)viewport.height;
  const float halfwidth =
      frustum_width ? frustum_width
                    : 0.5f * aspect_ratio * (frustum_top - frustum_bottom);
  const float left = frustum_center - halfwidth;
  const float right = frustum_center + halfwidth;

  mat4 projection;
  if (orthographic) {
    projection = mat4::ortho(left, right, frustum_bottom, frustum_top,
                             frustum_near, frustum_far);
  } else {
    projection = mat4::frustum(left, right, frustum_bottom, frustum_top,
                               frustum_near, frustum_far);
    projection[2][2] = -1.0f;
    projection[3][2] = -2.0f * frustum_near;
  }
  const mat4 look_at = mat4::lookAt(cam_pos, cam_at, cam_up);
  return projection * inverse(look_at);
}

void SceneDecorator::Update(mjData* data, const mjvOption* vis_option,
                            const mjvPerturb* perturb, mjvCamera* camera,
                            const mjrRect& viewport,
                            DrawTextAtFn draw_text_at_fn,
                            std::span<const mjvGeom> extra_geoms) {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();
  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;

  // For decor rendering, we go through the mjvScene. Ideally, we would only
  // update the mjvGeoms in the scene, but the API is not capable of that.
  mjv_updateScene(model, data, vis_option, perturb, camera, mjCAT_ALL,
                  &mjv_scene_);
  const int num_extra_geoms =
      std::min<int>(extra_geoms.size(), mjv_scene_.maxgeom - mjv_scene_.ngeom);
  for (int i = 0; i < num_extra_geoms; ++i) {
    mjv_scene_.geoms[mjv_scene_.ngeom] = extra_geoms[i];
    mjv_scene_.geoms[mjv_scene_.ngeom].category = mjCAT_DECOR;
    ++mjv_scene_.ngeom;
  }

  // Remove all decorative elements from previous render and prepare new ones.
  for (auto& iter : decorations_) {
    mjrf_removeRenderableFromScene(scene_, iter.get());
  }
  decorations_.clear();

  const mat4 clip_from_world =
      CalcClipFromWorld(model, data, *camera, viewport);
  for (int i = 0; i < mjv_scene_.ngeom; ++i) {
    const mjvGeom& geom = mjv_scene_.geoms[i];
    if (draw_text_at_fn && geom.label[0] != 0) {
      const float4 clip_pos =
          clip_from_world *
          float4(geom.pos[0], geom.pos[1], geom.pos[2], 1.0f);
      if (clip_pos.w != 0.0f) {
        const float3 pos = clip_pos.xyz / clip_pos.w;
        draw_text_at_fn(geom.label, pos.x, pos.y, pos.z);
      }
    }
    if (geom.category != mjCAT_DECOR) {
      continue;
    }

    const mjtGeom geom_type = (mjtGeom)geom.type;
    if (geom_type == mjGEOM_NONE || geom_type == mjGEOM_LABEL) {
      continue;
    }

    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    auto renderable = CreateRenderable(ctx, params);

    mjrf_setRenderableGeomMesh(renderable.get(), geom_type, nstack, nslice, nquad);
    mjrf_setRenderableTransform(renderable.get(), geom.pos, geom.mat);
    if (geom_type == mjGEOM_PLANE) {
      // Planes only define an xy size, so set the z-dimension to 1.0f.
      const float size[3] = {geom.size[0], geom.size[1], 1.0f};
      mjrf_setRenderableSize(renderable.get(), size);
    } else {
      mjrf_setRenderableSize(renderable.get(), geom.size);
    }

    mjrfMaterial material;
    mjrf_defaultMaterial(&material);
    material.decor_ux = true;
    material.color[0] = geom.rgba[0];
    material.color[1] = geom.rgba[1];
    material.color[2] = geom.rgba[2];
    material.color[3] = geom.rgba[3];
    mjrf_setRenderableMaterial(renderable.get(), &material);

    mjrf_addRenderableToScene(scene_, renderable.get());
    decorations_.push_back(std::move(renderable));
  }
}

}  // namespace mujoco
