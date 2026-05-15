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

#include "experimental/filament/compat/scene_geom_util.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"

namespace mujoco {

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

static bool IsBehind(const float* headpos, const float* pos, const float* mat) {
  return ((headpos[0] - pos[0]) * mat[2] + (headpos[1] - pos[1]) * mat[5] +
              (headpos[2] - pos[2]) * mat[8] <
          0.0f);
}

static void PrepareGeomMeshes(mjrRenderable* renderable, const mjvGeom& geom,
                              ModelObjects* model_objs) {
  const mjModel* model = model_objs->GetModel();
  const int nstack = model->vis.quality.numstacks;
  const int nslice = model->vis.quality.numslices;
  const int nquad = model->vis.quality.numquads;

  float position[3];
  std::memcpy(position, &geom.pos, 3 * sizeof(float));
  float rotation[9];
  std::memcpy(rotation, &geom.mat, 9 * sizeof(float));
  float size[3];
  std::memcpy(size, &geom.size, 3 * sizeof(float));

  const mjtGeom geom_type = (mjtGeom)geom.type;
  switch (geom_type) {
    case mjGEOM_MESH:
    case mjGEOM_SDF:
      mjrf_setRenderableMesh(renderable, model_objs->GetMesh(geom.dataid), 0, 0);
      // Ignore size for meshes.
      size[0] = 1.f;
      size[1] = 1.f;
      size[2] = 1.f;
      break;
    case mjGEOM_HFIELD:
      mjrf_setRenderableMesh(renderable, model_objs->GetHeightField(geom.dataid), 0, 0);
      // Ignore size for meshes.
      size[0] = 1.f;
      size[1] = 1.f;
      size[2] = 1.f;
      break;
    case mjGEOM_PLANE: {
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);

      const bool is_infinite = !(size[0] > 0 && size[1] > 0);
      if (is_infinite) {
        // Infinite planes are scaled to match the tile size used by
        // re-centering in engine_vis_visualize.c.
        const float plane_scale = static_cast<float>(mjMAXPLANEGRID) / 2.0f;
        size[0] = plane_scale;
        size[1] = plane_scale;
      }
      // Planes only define an xy size, so set the z-dimension to 1.0f.
      size[2] = 1.0f;
      break;
    }
    case mjGEOM_SPHERE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_ELLIPSOID:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_BOX:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_CAPSULE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_CYLINDER:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_ARROW:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_ARROW1:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_ARROW2:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_LINE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_LINEBOX:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_TRIANGLE:
      mjrf_setRenderableGeomMesh(renderable, geom_type, nstack, nslice, nquad);
      break;
    case mjGEOM_FLEX:
      mjrf_setRenderableMesh(renderable, model_objs->GetFlexSkinMesh(geom.objid), 0, 0);
      // Flexes are defined in global space.
      std::memset(position, 0, sizeof(position));
      std::memset(rotation, 0, sizeof(rotation));
      rotation[0] = 1.f;
      rotation[4] = 1.f;
      rotation[8] = 1.f;
      size[0] = 1.f;
      size[1] = 1.f;
      size[2] = 1.f;
      break;
    case mjGEOM_SKIN:
      mjrf_setRenderableMesh(renderable, model_objs->GetFlexSkinMesh(geom.objid), 0, 0);
      // Skins are defined in global space.
      std::memset(position, 0, sizeof(position));
      std::memset(rotation, 0, sizeof(rotation));
      rotation[0] = 1.f;
      rotation[4] = 1.f;
      rotation[8] = 1.f;
      size[0] = 1.f;
      size[1] = 1.f;
      size[2] = 1.f;
      break;
    case mjGEOM_NONE:
    case mjGEOM_LABEL:
      // Do nothing.
      break;
    case mjNGEOMTYPES:
      mju_warning("Unsupported geom type: %d", geom.type);
      break;
  }

  mjrf_setRenderableTransform(renderable, position, rotation, size);
}

static void UpdateGeomMaterial(mjrRenderable* renderable, const mjvGeom& geom,
                               ModelObjects* model_objs, const float headpos[3],
                               const mjtByte render_flags[mjNRNDFLAG]) {
  const mjModel* model = model_objs->GetModel();

  mjrMaterial material;
  mjr_defaultMaterial(&material);

  if (geom.category == mjCAT_DECOR) {
    material.decor_ux = true;
  }

  material.color[0] = geom.rgba[0];
  material.color[1] = geom.rgba[1];
  material.color[2] = geom.rgba[2];
  material.color[3] = geom.rgba[3];
  if (geom.type == mjGEOM_PLANE) {
    if (IsBehind(headpos, geom.pos, geom.mat)) {
      material.color[3] *= 0.3;
      mjrf_setRenderableReceiveShadows(renderable, false);
      material.reflective = false;
    } else {
      mjrf_setRenderableReceiveShadows(renderable, true);
      material.reflective = geom.reflectance > 0 && material.color[3] == 1.0f;
    }
  }
  mjrf_setRenderableLayerMask(renderable, geom.category);
  if (geom.category == mjCAT_DECOR) {
    mjrf_setRenderableCastShadows(renderable, false);
    mjrf_setRenderableReceiveShadows(renderable, false);
  }

  if (geom.matid >= 0 && geom.matid < model->nmat) {
    auto get_texture = [&](int role) -> const mjrTexture* {
      const int tex_id = model->mat_texid[geom.matid * mjNTEXROLE + role];
      return tex_id >= 0 ? model_objs->GetTexture(tex_id) : nullptr;
    };
    material.color_texture = get_texture(mjTEXROLE_RGB);
    material.normal_texture = get_texture(mjTEXROLE_NORMAL);
    material.emissive_texture = get_texture(mjTEXROLE_EMISSIVE);
    material.orm_texture = get_texture(mjTEXROLE_ORM);
    material.metallic_texture = get_texture(mjTEXROLE_METALLIC);
    material.roughness_texture = get_texture(mjTEXROLE_ROUGHNESS);
    material.occlusion_texture = get_texture(mjTEXROLE_OCCLUSION);
  }

  material.reflectance = geom.reflectance;
  material.emissive = geom.emission;
  material.specular = geom.specular;
  material.glossiness = geom.shininess;
  if (geom.matid >= 0) {
    material.metallic = model->mat_metallic[geom.matid];
    material.roughness = model->mat_roughness[geom.matid];
  }

  if (geom.segid >= 0) {
    uint32_t segmentation_color = geom.segid + 1;
    const bool use_segid_color = render_flags[mjRND_IDCOLOR];
    if (!use_segid_color) {
      constexpr double phi1 = 1.61803398874989484820;  // Cached Phi(1).
      constexpr double coef1 = 1.0 / phi1;
      const double index = static_cast<double>(geom.segid);
      const double sample = std::fmod(0.5 + coef1 * index, 1.0);
      segmentation_color = 0x01000000 * sample;
    }
    material.segmentation_color[0] = (segmentation_color >> 0);
    material.segmentation_color[1] = (segmentation_color >> 8);
    material.segmentation_color[2] = (segmentation_color >> 16);
  }

  // UvScale only applies to objects that don't have explicit UV coordinates
  // in their vertex buffer. Instead, we set the UV coordinate to be the same
  // as the vertex position.
  //
  // The material's `texuniform` and `texrepeat` parameters allow us to scale
  // the programmatic UVs.

  if (material.color_texture) {
    const bool tex_uniform = model->mat_texuniform[geom.matid];
    if (mjrf_getSamplerType(material.color_texture) == mjTEXTURE_2D) {
      // For 2D textures, `tex_repeat` specifies how many times the texture
      // image is repeated. The `tex_uniform` flag determines if the repetition
      // is applied at in object space (false) or in world space (true).
      float tex_repeat[2];
      tex_repeat[0] = model->mat_texrepeat[(geom.matid * 2) + 0];
      tex_repeat[1] = model->mat_texrepeat[(geom.matid * 2) + 1];
      material.uv_scale[0] = tex_repeat[0];
      material.uv_scale[1] = tex_repeat[1];

      if (geom.dataid >= 0 && geom.type != mjGEOM_PLANE) {
        if (geom.size[0] > mjMINVAL) {
          material.uv_scale[0] /= geom.size[0];
        }
        if (geom.size[1] > mjMINVAL) {
          material.uv_scale[1] /= geom.size[1];
        }
      }

      if (tex_uniform) {
        if (geom.size[0] > 0) {
          material.uv_scale[0] *= geom.size[0];
        }
        if (geom.size[1] > 0) {
          material.uv_scale[1] *= geom.size[1];
        }
      }
      const bool is_infinite_plane =
          geom.type == mjGEOM_PLANE && (geom.size[0] <= 0 || geom.size[1] <= 0);
      if (is_infinite_plane) {
        // Infinite planes are scaled to match the tile size used by
        // re-centering in engine_vis_visualize.c.
        const float plane_scale = static_cast<float>(mjMAXPLANEGRID) / 2.0f;
        const float tile_size_x =
            GetPlaneTileSize(model, geom.matid, tex_repeat[0]);
        const float tile_size_y =
            GetPlaneTileSize(model, geom.matid, tex_repeat[1]);
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
        material.uv_scale[0] = 1.0f / (geom.size[0] ? geom.size[0] : 1.0f);
        material.uv_scale[1] = 1.0f / (geom.size[1] ? geom.size[1] : 1.0f);
        material.uv_scale[2] = 1.0f / (geom.size[2] ? geom.size[2] : 1.0f);
      }
    }
  }

  // Apply material multipliers from the model.
  material.emissive *= model_objs->GetEmissiveMultiplier();
  material.specular *= model_objs->GetSpecularMultiplier();
  material.glossiness *= model_objs->GetShininessMultiplier();

  mjrf_setRenderableMaterial(renderable, &material);
}

UniquePtr<mjrRenderable> CreateGeomRenderable(
    const mjvGeom& geom, mjrfContext* ctx, ModelObjects* model_objs,
    const float headpos[3], const mjtByte render_flags[mjNRNDFLAG]) {
  mjrRenderableParams params;
  mjr_defaultRenderableParams(&params);
  auto renderable = CreateRenderable(ctx, params);
  PrepareGeomMeshes(renderable.get(), geom, model_objs);
  UpdateGeomMaterial(renderable.get(), geom, model_objs, headpos, render_flags);
  return renderable;
}
}  // namespace mujoco
