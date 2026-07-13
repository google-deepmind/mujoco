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

#include "render/filament/support/renderable_manager.h"

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <math/mat3.h>
#include <math/quat.h>
#include <math/TVecHelpers.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "engine/engine_sleep.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/filament_util.h"
#include "render/filament/support/mesh_util.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3f;
using filament::math::quatf;

// Converts an array of numbers into an array of floats.
template <typename T>
void xtof(float* dst, const T* src, int n) {
  for (int i = 0; i < n; ++i) {
    dst[i] = static_cast<float>(src[i]);
  }
}

// Returns the tile size for infinite plane texture alignment.
// This is duplicated from engine_vis_visualize.c (re-center infinite plane)
// to ensure UV scaling matches the re-centering increments.
static float GetPlaneTileSize(const mjModel* model, int matid,
                              float texrepeat) {
  if (matid >= 0 && texrepeat > 0) {
    return 2.0f / texrepeat;
  } else {
    const float zfar = model->vis.map.zfar * model->stat.extent;
    return 2.1f * zfar / (mjMAXPLANEGRID - 2);
  }
}

static mjtCatBit GetBodyCategory(const mjModel* m, int bodyid) {
  if (m->body_weldid[bodyid] == 0 &&
      m->body_mocapid[m->body_rootid[bodyid]] == -1) {
    return mjCAT_STATIC;
  } else {
    return mjCAT_DYNAMIC;
  }
}

static void Connect(mjrfRenderable* renderable, const float3 p0,
                    const float3 p1, const float width) {
  const float3 vec = p1 - p0;
  const float len = length(vec);
  const float3 pos = (p0 + p1) * 0.5f;
  const float3 size = {width, width, 0.5f * len};
  const quatf rot =
      quatf::fromDirectedRotation(normalize(vec), float3{0, 0, 1});
  const mat3f mat = mat3f(rot);
  mjrf_setRenderableTransform(renderable, pos.v, mat.asArray());
  mjrf_setRenderableSize(renderable, size.v);
}

static bool GetSize(const mjModel* model, mjtGeom type, const mjtNum* size,
                    float out[3]) {
  out[0] = size[0];
  out[1] = size[1];
  out[2] = size[2];
  switch (type) {
    case mjGEOM_SPHERE:
      out[2] = size[0];
      out[1] = size[0];
      return true;
    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
      out[2] = size[1];
      out[1] = size[0];
      return true;
    case mjGEOM_TRIANGLE:
    case mjGEOM_PLANE:
      out[2] = 1.0f;
      return true;
    case mjGEOM_ELLIPSOID:
    case mjGEOM_BOX:
    case mjGEOM_ARROW:
    case mjGEOM_ARROW1:
    case mjGEOM_ARROW2:
    case mjGEOM_LINE:
    case mjGEOM_LINEBOX:
      return true;
    case mjGEOM_MESH:
    case mjGEOM_SDF:
    case mjGEOM_HFIELD:
    case mjGEOM_FLEX:
    case mjGEOM_SKIN:
    case mjGEOM_NONE:
    case mjGEOM_LABEL:
      return false;
    case mjNGEOMTYPES:
      mju_warning("Unsupported geom type: %d", type);
      return false;
  }
}

static void SetGeomMesh(mjrfRenderable* renderable, ModelObjects* model_objs,
                        mjtGeom type, int geom_index = -1) {
  const mjModel* model = model_objs->GetModel();
  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;

  switch (type) {
    case mjGEOM_MESH:
    case mjGEOM_SDF: {
      const int data_id = model->geom_dataid[geom_index] * 2;
      mjrf_setRenderableMesh(renderable, model_objs->GetMesh(data_id), 0, 0);
      break;
    }
    case mjGEOM_HFIELD: {
      const int data_id = model->geom_dataid[geom_index];
      mjrf_setRenderableMesh(renderable, model_objs->GetHeightField(data_id), 0, 0);
      break;
    }
    case mjGEOM_FLEX:
      mju_error("Flex meshes should be handled separately.");
      break;
    case mjGEOM_SKIN:
      mju_error("Skin meshes should be handled separately.");
      break;
    case mjGEOM_PLANE:
    case mjGEOM_SPHERE:
    case mjGEOM_ELLIPSOID:
    case mjGEOM_BOX:
    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
    case mjGEOM_ARROW:
    case mjGEOM_ARROW1:
    case mjGEOM_ARROW2:
    case mjGEOM_LINE:
    case mjGEOM_LINEBOX:
    case mjGEOM_TRIANGLE:
      mjrf_setRenderableGeomMesh(renderable, type, nstack, nslice, nquad);
      break;
    case mjGEOM_NONE:
    case mjGEOM_LABEL:
      // Do nothing.
      break;
    case mjNGEOMTYPES:
      mju_warning("Unsupported geom type: %d", type);
      break;
  }
}

RenderableManager::RenderableManager(mjrfScene* scene,
                                     ModelObjects* model_objects)
    : scene_(scene), model_objects_(model_objects) {
  mjv_defaultOption(&vopts_);

  AddGeomGeoms();
  AddSiteGeoms();
  AddFlexGeoms();
  AddSkinGeoms();
  AddSliderCrankGeoms();
  // A tendon is composed of a collection of renderables which are managed
  // every frame. For now, all we do is reserve space for the collections.
  tendons_.resize(model_objects_->GetModel()->ntendon);
}

RenderableManager::~RenderableManager() {
  for (auto& renderable : geoms_) {
    mjrf_removeRenderableFromScene(scene_, renderable.get());
  }
  for (auto& renderable : sites_) {
    mjrf_removeRenderableFromScene(scene_, renderable.get());
  }
  for (auto& renderable : flexes_) {
    mjrf_removeRenderableFromScene(scene_, renderable.get());
  }
  for (auto& renderable : skins_) {
    mjrf_removeRenderableFromScene(scene_, renderable.get());
  }
  for (auto& segments : tendons_) {
    for (auto& renderable : segments) {
      mjrf_removeRenderableFromScene(scene_, renderable.get());
    }
  }
  for (auto& renderable : sliders_) {
    mjrf_removeRenderableFromScene(scene_, renderable.get());
  }
  for (auto& renderable : cranks_) {
    mjrf_removeRenderableFromScene(scene_, renderable.get());
  }
}

void RenderableManager::Update(const mjData* data) {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();

  for (int i = 0; i < model->ngeom; ++i) {
    const float3 pos = ReadFloat3(data->geom_xpos, i);
    const mat3f mat = ReadMat3(data->geom_xmat, i);
    mjrf_setRenderableTransform(geoms_[i].get(), pos.v, mat.asArray());
  }

  for (int i = 0; i < model->nsite; ++i) {
    const float3 pos = ReadFloat3(data->site_xpos, i);
    const mat3f mat = ReadMat3(data->site_xmat, i);
    mjrf_setRenderableTransform(sites_[i].get(), pos.v, mat.asArray());
  }

  for (int i = 0; i < model->nflex; ++i) {
    const int flex_layer = vopts_.flex_layer;
    const bool smooth_skinning = vopts_.flags[mjVIS_FLEXSKIN];
    const bool edges = !smooth_skinning && vopts_.flags[mjVIS_FLEXEDGE];
    const bool vertices = !smooth_skinning && vopts_.flags[mjVIS_FLEXVERT];
    auto mesh = CreateFlexMesh(ctx, model, data, i, flex_layer,
                               smooth_skinning, edges, vertices);
    mjrf_setRenderableMesh(flexes_[i].get(), mesh.get(), 0, 0);
    flex_meshes_[i] = std::move(mesh);
  }

  for (int i = 0; i < model->nskin; ++i) {
    auto mesh = CreateSkinMesh(ctx, model, data, i);
    mjrf_setRenderableMesh(skins_[i].get(), mesh.get(), 0, 0);
    skin_meshes_[i] = std::move(mesh);
  }

  for (int i = 0; i < model->ntendon; i++) {
    UpdateSpatialTendons(data, i);
  }

  int renderable_index = 0;
  for (int i = 0; i < model->nu; i++) {
    if (model->actuator_trntype[i] != mjTRN_SLIDERCRANK) {
      continue;
    }
    UpdateSliderCranks(data, i, renderable_index++);
  }

  if (vopts_.flags[mjVIS_ISLAND]) {
    const bool sleep_enabled = model->opt.enableflags & mjENBL_SLEEP;

    for (int i = 0; i < model->ngeom; ++i) {
      const int weld_id = model->body_weldid[model->geom_bodyid[i]];
      if (!model->body_dofnum[weld_id]) {
        continue;
      }

      const int awake = data->body_awake[model->geom_bodyid[i]];
      const int dof = model->body_dofadr[weld_id];
      const int island = data->nisland ? data->dof_island[dof] : -1;
      int island_id = island >= 0 ? data->island_dofadr[island] : -1;
      if (island_id == -1 && sleep_enabled) {
        int tree = model->dof_treeid[dof];
        if (!awake) {
          tree = mj_sleepCycle(data->tree_asleep, model->ntree, tree);
        }
        island_id = model->tree_dofadr[tree];
      }

      mjrfMaterial material;
      mjrf_getRenderableMaterial(geoms_[i].get(), &material);
      material.island_id = island_id;
      material.sleep_state = awake ? mjS_AWAKE : mjS_ASLEEP;
      mjrf_setRenderableMaterial(geoms_[i].get(), &material);
    }
    for (int i = 0; i < model->nflex; ++i) {
      int bodyid = -1;
      if (model->flex_interp[i]) {
        int nodeadr = model->flex_nodeadr[i];
        for (int j = 0; j < model->flex_nodenum[i] && bodyid < 0; j++) {
          int b = model->flex_nodebodyid[nodeadr+j];
          if (model->body_treeid[b] >= 0) bodyid = b;
        }
      } else {
        int vertadr = model->flex_vertadr[i];
        for (int j=0; j < model->flex_vertnum[i] && bodyid < 0; j++) {
          int b = model->flex_vertbodyid[vertadr+j];
          if (model->body_treeid[b] >= 0) bodyid = b;
        }
      }
      if (bodyid < 0) {
        continue;
      }
      int weld_id = model->body_weldid[bodyid];
      int dof = model->body_dofadr[weld_id];
      int island = data->nisland ? data->dof_island[dof] : -1;
      int island_id = island >= 0 ? data->island_dofadr[island] : -1;
      int awake = data->body_awake[bodyid];
      if (island_id == -1 && sleep_enabled) {
        int tree = model->dof_treeid[dof];
        if (!awake) {
          tree = mj_sleepCycle(data->tree_asleep, model->ntree, tree);
        }
        island_id = model->tree_dofadr[tree];
      }

      mjrfMaterial material;
      mjrf_getRenderableMaterial(flexes_[i].get(), &material);
      material.island_id = island_id;
      material.sleep_state = awake ? mjS_AWAKE : mjS_ASLEEP;
      mjrf_setRenderableMaterial(flexes_[i].get(), &material);
    }
  }
}

mjrfRenderable* RenderableManager::GetRenderable(mjtObj obj_type,
                                                 int obj_index) {
  switch (obj_type) {
    case mjOBJ_GEOM:
      if (obj_index >= 0 && obj_index < geoms_.size()) {
        return geoms_[obj_index].get();
      }
      break;
    case mjOBJ_SITE:
      if (obj_index >= 0 && obj_index < sites_.size()) {
        return sites_[obj_index].get();
      }
      break;
    case mjOBJ_FLEX:
      if (obj_index >= 0 && obj_index < flexes_.size()) {
        return flexes_[obj_index].get();
      }
      break;
    case mjOBJ_SKIN:
      if (obj_index >= 0 && obj_index < skins_.size()) {
        return skins_[obj_index].get();
      }
      break;
    case mjOBJ_ACTUATOR:
      if (obj_index >= 0 && obj_index < sliders_.size()) {
        return sliders_[obj_index].get();
      }
      break;
    case mjOBJ_BODY: {
      const mjModel* model = model_objects_->GetModel();
      for (int i = 0; i < model->ngeom; ++i) {
        if (model->geom_bodyid[i] == obj_index) {
          return geoms_[i].get();
        }
      }
      for (int i = 0; i < model->nsite; ++i) {
        if (model->site_bodyid[i] == obj_index) {
          return sites_[i].get();
        }
      }
      break;
    }
    default:
      break;
  }
  return nullptr;
}

void RenderableManager::AddGeomGeoms() {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();

  geoms_.reserve(model->ngeom);
  for (int i = 0; i < model->ngeom; ++i) {
    const mjtGeom type = (mjtGeom)model->geom_type[i];

    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    auto renderable = CreateRenderable(ctx, params);

    SetGeomMesh(renderable.get(), model_objects_, type, i);

    mjrfMaterial material = GetDefaultMaterial(mjOBJ_GEOM, i);
    mjrf_setRenderableMaterial(renderable.get(), &material);

    float size[3];
    if (GetSize(model, type, model->geom_size + (3 * i), size)) {
      mjrf_setRenderableSize(renderable.get(), size);
    }

    if (vopts_.geomgroup[model->geom_group[i]]) {
      mjrf_addRenderableToScene(scene_, renderable.get());
    }
    geoms_.emplace_back(std::move(renderable));
  }
}

void RenderableManager::AddSiteGeoms() {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();

  sites_.reserve(model->nsite);
  for (int i = 0; i < model->nsite; ++i) {
    const mjtGeom type = (mjtGeom)model->site_type[i];

    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    auto renderable = CreateRenderable(ctx, params);

    SetGeomMesh(renderable.get(), model_objects_, type);

    mjrfMaterial material = GetDefaultMaterial(mjOBJ_SITE, i);
    mjrf_setRenderableMaterial(renderable.get(), &material);

    float size[3];
    if (GetSize(model, type, model->site_size + (3 * i), size)) {
      mjrf_setRenderableSize(renderable.get(), size);
    }

    if (vopts_.sitegroup[model->site_group[i]]) {
      mjrf_addRenderableToScene(scene_, renderable.get());
    }
    sites_.push_back(std::move(renderable));
  }
}

void RenderableManager::AddFlexGeoms() {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();

  flexes_.reserve(model->nflex);
  flex_meshes_.reserve(model->nflex);
  for (int i = 0; i < model->nflex; ++i) {
    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    auto renderable = CreateRenderable(ctx, params);

    mjrfMaterial material = GetDefaultMaterial(mjOBJ_FLEX, i);
    mjrf_setRenderableMaterial(renderable.get(), &material);

    if (vopts_.flexgroup[model->flex_group[i]]) {
      mjrf_addRenderableToScene(scene_, renderable.get());
    }
    flexes_.emplace_back(std::move(renderable));
    flex_meshes_.emplace_back(nullptr, nullptr);
  }
}

void RenderableManager::AddSkinGeoms() {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();

  skins_.reserve(model->nskin);
  skin_meshes_.reserve(model->nskin);
  for (int i = 0; i < model->nskin; i++) {
    mjrfRenderableParams params;
    mjrf_defaultRenderableParams(&params);
    auto renderable = CreateRenderable(ctx, params);

    mjrfMaterial material = GetDefaultMaterial(mjOBJ_SKIN, i);
    mjrf_setRenderableMaterial(renderable.get(), &material);

    if (vopts_.skingroup[model->skin_group[i]]) {
      mjrf_addRenderableToScene(scene_, renderable.get());
    }
    skins_.emplace_back(std::move(renderable));
    skin_meshes_.emplace_back(nullptr, nullptr);
  }
}

void RenderableManager::AddSliderCrankGeoms() {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();
  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;

  mjrfRenderableParams params;
  mjrf_defaultRenderableParams(&params);

  sliders_.reserve(model->nu);
  cranks_.reserve(model->nu);
  for (int i = 0; i < model->nu; i++) {
    if (model->actuator_trntype[i] != mjTRN_SLIDERCRANK) {
      continue;
    }

    // Create two renderables, one for the slider and the other for the crank.
    auto slider = CreateRenderable(ctx, params);
    mjrf_setRenderableGeomMesh(slider.get(), mjGEOM_CYLINDER, nstack, nslice,
                               nquad);
    mjrf_addRenderableToScene(scene_, slider.get());
    sliders_.emplace_back(std::move(slider));

    auto crank = CreateRenderable(ctx, params);
    mjrf_setRenderableGeomMesh(crank.get(), mjGEOM_CAPSULE, nstack, nslice,
                               nquad);
    mjrf_addRenderableToScene(scene_, crank.get());
    cranks_.emplace_back(std::move(crank));
  }
}

void RenderableManager::UpdateSliderCranks(const mjData* data, int actuator_id,
                                           int index) {
  const mjModel* model = model_objects_->GetModel();
  const float scale = model->stat.meansize;
  const float slider_width = scale * model->vis.scale.slidercrank;
  const float crank_width = scale * model->vis.scale.slidercrank / 2.0;

  const int crank_id = model->actuator_trnid[2 * actuator_id];
  const int slider_id = model->actuator_trnid[2 * actuator_id + 1];
  const float3 crank_pos = ReadFloat3(data->site_xpos, crank_id);
  const float3 slider_pos = ReadFloat3(data->site_xpos, slider_id);

  const float3 vec = crank_pos - slider_pos;

  float3 axis;
  axis[0] = data->site_xmat[9 * slider_id + 2];
  axis[1] = data->site_xmat[9 * slider_id + 5];
  axis[2] = data->site_xmat[9 * slider_id + 8];
  float len = dot(vec, axis);

  const float rod = model->actuator_cranklength[actuator_id];
  const float det = (len * len) + (rod * rod) - dot(vec, vec);
  const bool broken = (det < 0);
  if (det > 0) {
    len -= std::sqrt(det);
  }

  const float3 end = slider_pos + (axis * len);

  mjrfRenderable* slider = sliders_[index].get();
  mjrfRenderable* crank = cranks_[index].get();
  Connect(slider, slider_pos, end, slider_width);
  Connect(crank, end, crank_pos, crank_width);

  mjrfMaterial material = GetDefaultMaterial(mjOBJ_ACTUATOR, actuator_id);

  xtof(material.color, model->vis.rgba.slidercrank, 4);
  mjrf_setRenderableMaterial(slider, &material);

  if (broken) {
    xtof(material.color, model->vis.rgba.crankbroken, 4);
  }
  mjrf_setRenderableMaterial(crank, &material);
}

void RenderableManager::AppendSegmentToTendon(int tendon_id) {
  mjrfContext* ctx = model_objects_->GetContext();
  const mjModel* model = model_objects_->GetModel();

  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;

  mjrfRenderableParams params;
  mjrf_defaultRenderableParams(&params);
  auto renderable = CreateRenderable(ctx, params);
  mjrf_setRenderableGeomMesh(renderable.get(), mjGEOM_CAPSULE, nstack, nslice,
                             nquad);
  if (vopts_.tendongroup[model->tendon_group[tendon_id]]) {
    mjrf_addRenderableToScene(scene_, renderable.get());
  }

  auto& segments = tendons_[tendon_id];
  segments.emplace_back(std::move(renderable));
}

void RenderableManager::RemoveSegmentFromTendon(int tendon_id) {
  auto& segments = tendons_[tendon_id];
  mjrfRenderable* renderable = segments.back().get();
  mjrf_removeRenderableFromScene(scene_, renderable);
  segments.pop_back();
}

void RenderableManager::UpdateSpatialTendons(const mjData* data, int tendon_id) {
  const mjModel* model = model_objects_->GetModel();

  // Gather the points that define the tendon. We'll use a simple cache to avoid
  // excessive reallocations.
  point_cache_.clear();
  GatherSpatialTendonPoints(model, data, tendon_id, point_cache_);

  // Tendons are composed on N "segments" represented by cylinder geoms.
  // We adjust the number of segments we need to match the number of points.
  const int nsegments = static_cast<int>(point_cache_.size()) / 2;
  std::vector<UniquePtr<mjrfRenderable>>& segments = tendons_[tendon_id];
  while (tendons_[tendon_id].size() < nsegments) {
    AppendSegmentToTendon(tendon_id);
  }
  while (tendons_[tendon_id].size() > nsegments) {
    RemoveSegmentFromTendon(tendon_id);
  }

  mjrfMaterial material = GetDefaultMaterial(mjOBJ_TENDON, tendon_id);

  // If tendon has no explicit color then color it using limit impedance.
  if (model->tendon_matid[tendon_id] == -1 && material.color[0] == 0.5 &&
      material.color[1] == 0.5 && material.color[2] == 0.5 &&
      material.color[3] == 1) {
    mjtNum imp = 0;
    int efc_start = data->ne + data->nf;
    int efc_end = efc_start + data->nl;
    for (int k = efc_start; k < efc_end; k++) {
      if (data->efc_type[k] == mjCNSTR_LIMIT_TENDON &&
          data->efc_id[k] == tendon_id) {
        imp = data->efc_KBIP[4 * k + 2];
      }
    }
    const float scale = (1 - imp);
    const float* constraint = model->vis.rgba.constraint;
    material.color[0] = scale * material.color[0] + imp * constraint[0];
    material.color[1] = scale * material.color[1] + imp * constraint[1];
    material.color[2] = scale * material.color[2] + imp * constraint[2];
  }

  if (vopts_.flags[mjVIS_ISLAND]) {
    const int ecf = data->tendon_efcadr[tendon_id];
    if (data->nisland && ecf >= 0) {
      material.island_id = data->island_dofadr[data->efc_island[ecf]];
      material.sleep_state = mjS_AWAKE;
    }
  }

  for (int i = 0; i < point_cache_.size(); i += 2) {
    mjrfRenderable* renderable = segments[i / 2].get();
    const float width = point_cache_[i].w;
    const float3 p0 = point_cache_[i + 0].xyz;
    const float3 p1 = point_cache_[i + 1].xyz;
    Connect(renderable, p0, p1, width);
    mjrf_setRenderableMaterial(renderable, &material);
  }
}

int RenderableManager::GetSegmentationId(mjtObj obj_type, int obj_index) {
  const mjModel* model = model_objects_->GetModel();

  int id = 0;
  if (obj_type == mjOBJ_GEOM) {
    return id + obj_index;
  } else {
    id += model->ngeom;
  }
  if (obj_type == mjOBJ_SITE) {
    return id + obj_index;
  } else {
    id += model->nsite;
  }
  if (obj_type == mjOBJ_FLEX) {
    return id + obj_index;
  } else {
    id += model->nflex;
  }
  if (obj_type == mjOBJ_SKIN) {
    return id + obj_index;
  } else {
    id += model->nskin;
  }
  if (obj_type == mjOBJ_TENDON) {
    return id + obj_index;
  } else {
    id += model->ntendon;
  }
  if (obj_type == mjOBJ_ACTUATOR) {
    return id + obj_index;
  } else {
    id += model->nu;
  }
  mju_error("Unsupported object type: %d", obj_type);
  return -1;
}

mjrfMaterial RenderableManager::GetDefaultMaterial(mjtObj obj_type,
                                                   int obj_index) {
  const mjModel* model = model_objects_->GetModel();

  mjrfMaterial material;
  mjrf_defaultMaterial(&material);

  int matid = -1;
  mjtGeom geom_type = mjGEOM_NONE;
  const float* rgba = nullptr;
  const mjtNum* size = nullptr;

  switch (obj_type) {
    case mjOBJ_GEOM:
      geom_type = (mjtGeom)model->geom_type[obj_index];
      rgba = model->geom_rgba + (4 * obj_index);
      size = model->geom_size + (3 * obj_index);
      matid = model->geom_matid[obj_index];
      break;
    case mjOBJ_SITE:
      geom_type = (mjtGeom)model->site_type[obj_index];
      rgba = model->site_rgba + (4 * obj_index);
      size = model->site_size + (3 * obj_index);
      matid = model->site_matid[obj_index];
      break;
    case mjOBJ_FLEX:
      geom_type = mjGEOM_FLEX;
      rgba = model->flex_rgba + (4 * obj_index);
      size = model->flex_size + (3 * obj_index);
      matid = model->flex_matid[obj_index];
      break;
    case mjOBJ_SKIN:
      geom_type = mjGEOM_SKIN;
      rgba = model->skin_rgba + (4 * obj_index);
      matid = model->skin_matid[obj_index];
      break;
    case mjOBJ_TENDON:
      geom_type = mjGEOM_CAPSULE;
      rgba = model->tendon_rgba + (4 * obj_index);
      size = model->tendon_width + obj_index;
      matid = model->tendon_matid[obj_index];
      break;
    case mjOBJ_ACTUATOR:
      geom_type = mjGEOM_CYLINDER;
      rgba = model->vis.rgba.slidercrank;
      // size = model->tendon_width + obj_index;
      break;
    default:
      mju_error("Unsupported object type: %d", obj_type);
      break;
  }

  xtof(material.color, rgba, 4);

  if (matid >= 0 && matid < model->nmat) {
    auto get_texture = [&](int role) -> const mjrfTexture* {
      const int tex_id = model->mat_texid[matid * mjNTEXROLE + role];
      return tex_id >= 0 ? model_objects_->GetTexture(tex_id) : nullptr;
    };
    material.color_texture = get_texture(mjTEXROLE_RGB);
    material.normal_texture = get_texture(mjTEXROLE_NORMAL);
    material.emissive_texture = get_texture(mjTEXROLE_EMISSIVE);
    material.orm_texture = get_texture(mjTEXROLE_ORM);
    material.metallic_texture = get_texture(mjTEXROLE_METALLIC);
    material.roughness_texture = get_texture(mjTEXROLE_ROUGHNESS);
    material.occlusion_texture = get_texture(mjTEXROLE_OCCLUSION);

    material.emissive = model->mat_emission[matid];
    material.specular = model->mat_specular[matid];
    material.glossiness = model->mat_shininess[matid];
    material.reflectance = model->mat_reflectance[matid];
    material.metallic = model->mat_metallic[matid];
    material.roughness = model->mat_roughness[matid];

    const float* rgba = model->mat_rgba + 4 * matid;
    if (rgba[0] != 0.5f || rgba[1] != 0.5f || rgba[2] != 0.5f ||
        rgba[3] != 1.0f) {
      xtof(material.color, rgba, 4);
    }
  } else {
    material.emissive = 0;
    material.specular = 0.5;
    material.glossiness = 0.5;
    material.reflectance = 0;
  }

  // UvScale only applies to objects that don't have explicit UV coordinates
  // in their vertex buffer. Instead, we set the UV coordinate to be the same
  // as the vertex position.
  //
  // The material's `texuniform` and `texrepeat` parameters allow us to scale
  // the programmatic UVs.

  if (material.color_texture) {
    float fsize[3] = {1.0f, 1.0f, 1.0f};
    if (size) {
      xtof(fsize, size, 3);
      if (geom_type != mjGEOM_NONE) {
        switch (geom_type) {
          case mjGEOM_SPHERE:
            fsize[2] = size[0];
            fsize[1] = size[0];
            break;
          case mjGEOM_CAPSULE:
          case mjGEOM_CYLINDER:
            fsize[2] = size[1];
            fsize[1] = size[0];
            break;
          case mjGEOM_TRIANGLE:
          case mjGEOM_PLANE:
            fsize[2] = 1.0f;
            break;
          default:
            break;
        }
      }
    }

    const bool tex_uniform = model->mat_texuniform[matid];
    if (mjrf_getTextureSamplerType(material.color_texture) == mjTEXTURE_2D) {
      // For 2D textures, `tex_repeat` specifies how many times the texture
      // image is repeated. The `tex_uniform` flag determines if the repetition
      // is applied at in object space (false) or in world space (true).
      float tex_repeat[2];
      tex_repeat[0] = model->mat_texrepeat[(matid * 2) + 0];
      tex_repeat[1] = model->mat_texrepeat[(matid * 2) + 1];
      material.uv_scale[0] = tex_repeat[0];
      material.uv_scale[1] = tex_repeat[1];

      if (geom_type == mjGEOM_MESH || geom_type == mjGEOM_HFIELD ||
          geom_type == mjGEOM_SDF) {
        if (fsize[0] > mjMINVAL) {
          material.uv_scale[0] /= fsize[0];
        }
        if (fsize[1] > mjMINVAL) {
          material.uv_scale[1] /= fsize[1];
        }
      }

      if (tex_uniform) {
        if (fsize[0] > 0) {
          material.uv_scale[0] *= fsize[0];
        }
        if (fsize[1] > 0) {
          material.uv_scale[1] *= fsize[1];
        }
      }
      const bool is_infinite_plane =
          geom_type == mjGEOM_PLANE && (fsize[0] <= 0 || fsize[1] <= 0);
      if (is_infinite_plane) {
        // Infinite planes are scaled to match the tile size used by
        // re-centering in engine_vis_visualize.c.
        const float plane_scale = static_cast<float>(mjMAXPLANEGRID) / 2.0f;
        const float tile_size_x = GetPlaneTileSize(model, matid, tex_repeat[0]);
        const float tile_size_y = GetPlaneTileSize(model, matid, tex_repeat[1]);
        material.uv_scale[0] = 2.0f * plane_scale / tile_size_x;
        material.uv_scale[1] = 2.0f * plane_scale / tile_size_y;
      }

      // We want to do the equivalent of:
      //   mjr_setf4(splane, 0.5 * scl.x,  0, 0, -0.5);
      //   mjr_setf4(tplane, 0, -0.5 * scl.y, 0, -0.5);
      //   glTexGenfv(GL_S, GL_OBJECT_PLANE, splane);
      //   glTexGenfv(GL_T, GL_OBJECT_PLANE, tplane);
      material.uv_scale[0] = 0.5f * material.uv_scale[0];
      material.uv_scale[1] = -0.5f * material.uv_scale[1];
      material.uv_offset[0] = -0.5f;
      material.uv_offset[1] = -0.5f;
    } else {
      // For cube maps, if `tex_uniform` is true, then scale the texture so that
      // it covers a 1x1 area of world space rather than the area of the object.
      if (tex_uniform) {
        material.uv_scale[0] = 1.0f / (fsize[0] ? fsize[0] : 1.0f);
        material.uv_scale[1] = 1.0f / (fsize[1] ? fsize[1] : 1.0f);
        material.uv_scale[2] = 1.0f / (fsize[2] ? fsize[2] : 1.0f);
      }
    }
  }

  // Apply material multipliers from the model.
  material.emissive *= model_objects_->GetEmissiveMultiplier();
  material.specular *= model_objects_->GetSpecularMultiplier();
  material.glossiness *= model_objects_->GetShininessMultiplier();

  material.segmentation_id = GetSegmentationId(obj_type, obj_index);

  return material;
}

void RenderableManager::SelectObject(mjtObj obj_type, int obj_index) {
  if (obj_type != selected_obj_type_ || obj_index != selected_obj_index_) {
    mjrfMaterial material;

    mjrfRenderable* prev_renderable =
        GetRenderable(selected_obj_type_, selected_obj_index_);
    if (prev_renderable) {
      mjrf_getRenderableMaterial(prev_renderable, &material);
      material.selected = 0;
      mjrf_setRenderableMaterial(prev_renderable, &material);
    }

    selected_obj_type_ = obj_type;
    selected_obj_index_ = obj_index;

    mjrfRenderable* curr_renderable =
        GetRenderable(selected_obj_type_, selected_obj_index_);
    if (curr_renderable) {
      mjrf_getRenderableMaterial(curr_renderable, &material);
      material.selected = 1;
      mjrf_setRenderableMaterial(curr_renderable, &material);
    }
  }
}

void RenderableManager::SetVisibility(mjtObj obj_type, bool visible,
                                 std::optional<int> group) {
  const mjModel* model = model_objects_->GetModel();

  switch (obj_type) {
    case mjOBJ_GEOM:
      if (group.has_value()) {
        if (vopts_.geomgroup[group.value()] == visible) {
          return;
        }
        for (int i = 0; i < model->ngeom; ++i) {
          if (model->geom_group[i] != group.value()) {
            continue;
          }
          mjrfRenderable* renderable = geoms_[i].get();
          if (visible) {
            mjrf_addRenderableToScene(scene_, renderable);
          } else {
            mjrf_removeRenderableFromScene(scene_, renderable);
          }
        }
      } else {
        mju_error("Unsupported object type: %d", obj_type);
      }
      break;
    case mjOBJ_SITE:
      if (group.has_value()) {
        if (vopts_.sitegroup[group.value()] == visible) {
          return;
        }
        for (int i = 0; i < model->nsite; ++i) {
          if (model->site_group[i] != group.value()) {
            continue;
          }
          mjrfRenderable* renderable = sites_[i].get();
          if (visible) {
            mjrf_addRenderableToScene(scene_, renderable);
          } else {
            mjrf_removeRenderableFromScene(scene_, renderable);
          }
        }
      } else {
        mju_error("Unsupported object type: %d", obj_type);
      }
      break;
    case mjOBJ_JOINT:
      break;
    case mjOBJ_TENDON:
      break;
    case mjOBJ_ACTUATOR:
      break;
    case mjOBJ_FLEX:
      break;
    case mjOBJ_SKIN:
      break;
    default:
      mju_error("Unsupported object type: %d", obj_type);
      break;
  }
}

void RenderableManager::Apply(const mjvOption& vopts) {
  const mjModel* model = model_objects_->GetModel();
  for (int i = 0; i < mjNGROUP; ++i) {
    SetVisibility(mjOBJ_GEOM, vopts.geomgroup[i], i);
    SetVisibility(mjOBJ_SITE, vopts.sitegroup[i], i);
    SetVisibility(mjOBJ_JOINT, vopts.jointgroup[i], i);
    SetVisibility(mjOBJ_TENDON, vopts.tendongroup[i], i);
    SetVisibility(mjOBJ_ACTUATOR, vopts.actuatorgroup[i], i);
    SetVisibility(mjOBJ_FLEX, vopts.flexgroup[i], i);
    SetVisibility(mjOBJ_SKIN, vopts.skingroup[i], i);
  }
  SetVisibility(mjOBJ_JOINT, vopts.flags[mjVIS_JOINT]);
  SetVisibility(mjOBJ_TENDON, vopts.flags[mjVIS_TENDON]);
  SetVisibility(mjOBJ_ACTUATOR, vopts.flags[mjVIS_ACTUATOR]);
  SetVisibility(mjOBJ_SKIN, vopts.flags[mjVIS_SKIN]);
  SetVisibility(mjOBJ_FLEX, vopts.flags[mjVIS_FLEXSKIN]);

  // Swap geoms between convex and non-convex meshes based on flags.
  if (vopts.flags[mjVIS_CONVEXHULL] != vopts_.flags[mjVIS_CONVEXHULL]) {
    for (int i = 0; i < model->ngeom; ++i) {
      const mjtGeom geom_type = (mjtGeom)model->geom_type[i];
      if (geom_type == mjGEOM_MESH || geom_type == mjGEOM_SDF) {
        mjrfRenderable* renderable = geoms_[i].get();
        const int mesh_id =
            model->geom_dataid[i] * 2 + (vopts.flags[mjVIS_CONVEXHULL] ? 1 : 0);
        const mjrfMesh* mesh = model_objects_->GetMesh(mesh_id);
        if (mesh) {
          mjrf_setRenderableMesh(renderable, mesh, 0, 0);
        }
      }
    }
  }

  // Adjust alpha of dynamic geoms based on transparent flag.
  if (vopts.flags[mjVIS_TRANSPARENT] != vopts_.flags[mjVIS_TRANSPARENT]) {
    float multiplier = model->vis.map.alpha;
    if (!vopts.flags[mjVIS_TRANSPARENT]) {
      multiplier = 1.0f / multiplier;
    }

    for (int i = 0; i < model->ngeom; ++i) {
      const int category = GetBodyCategory(model, model->geom_bodyid[i]);
      if (category == mjCAT_DYNAMIC) {
        mjrfMaterial material;
        mjrf_getRenderableMaterial(geoms_[i].get(), &material);
        material.color[3] *= multiplier;
        mjrf_setRenderableMaterial(geoms_[i].get(), &material);
      }
    }
  }

  vopts_ = vopts;
}

}  // namespace mujoco
