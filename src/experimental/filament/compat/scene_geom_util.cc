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
#include <memory>
#include <numbers>
#include <vector>

#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/mesh.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;

// An arbitrary scale factor for arrows.
static constexpr float kArrowScale = 1.f / 6.f;
static constexpr float kArrowHeadSize = 1.75f;

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

static const Mesh* GetMesh(ModelObjects* model_objs, int data_id) {
  const Mesh* mesh = model_objs->GetMeshBuffer(data_id);
  if (mesh == nullptr) {
    mju_error("Unknown mesh %d", data_id);
  }
  return mesh;
}

static const Mesh* GetSkinFlexMesh(ModelObjects* model_objs, int objid) {
  return model_objs->GetFlexSkinGeomMesh(objid);
}

static const Mesh* GetHeightField(ModelObjects* model_objs, int hfield_id) {
  const Mesh* mesh = model_objs->GetHeightFieldBuffer(hfield_id);
  if (mesh == nullptr) {
    mju_error("Unknown height field %d", hfield_id);
  }
  return mesh;
}

static const Mesh* GetShape(ModelObjects* model_objs,
                            ModelObjects::ShapeType shape_type) {
  const Mesh* mesh = model_objs->GetShapeBuffer(shape_type);
  if (mesh == nullptr) {
    mju_error("Unknown shape %d", shape_type);
  }
  return mesh;
}

static void PrepareGeomMeshes(Renderable& renderable, const mjvGeom& geom,
                              const mjvScene* scene,
                              ModelObjects* model_objects) {
  std::vector<const Mesh*> meshes;
  Renderable::GetTransformFn get_transforms;

  Trs trs = {
      .translation = ReadFloat3(geom.pos),
      .rotation = ReadMat3(geom.mat),
      .size = ReadFloat3(geom.size),
  };

  switch ((mjtGeom)geom.type) {
    case mjGEOM_MESH:
      meshes.push_back(GetMesh(model_objects, geom.dataid));
      // Ignore size for meshes.
      trs.size = float3{1.0f, 1.0f, 1.0f};
      break;
    case mjGEOM_HFIELD:
      meshes.push_back(GetHeightField(model_objects, geom.dataid));
      // Ignore size for height fields.
      trs.size = float3{1.0f, 1.0f, 1.0f};
      break;
    case mjGEOM_PLANE: {
      meshes.push_back(GetShape(model_objects, ModelObjects::kPlane));
      const bool is_infinite = !(trs.size.x > 0 && trs.size.y > 0);
      if (is_infinite) {
        // Infinite planes are scaled to match the tile size used by
        // re-centering in engine_vis_visualize.c.
        const float plane_scale = static_cast<float>(mjMAXPLANEGRID) / 2.0f;
        trs.size.x = plane_scale;
        trs.size.y = plane_scale;
      }
      // Planes only define an xy size, so set the z-dimension to 1.0f.
      trs.size.z = 1.0f;
      break;
    }
    case mjGEOM_SPHERE:
      meshes.push_back(GetShape(model_objects, ModelObjects::kSphere));
      break;
    case mjGEOM_ELLIPSOID:
      meshes.push_back(GetShape(model_objects, ModelObjects::kSphere));
      break;
    case mjGEOM_BOX:
      meshes.push_back(GetShape(model_objects, ModelObjects::kBox));
      break;
    case mjGEOM_CAPSULE: {
      // Capsules are a tube with two domes at the ends.
      meshes.push_back(GetShape(model_objects, ModelObjects::kTube));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDome));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDome));

      get_transforms = [](int index, const Trs& trs) {
        // We apply an inverse scale to the domes to counteract the capsule's
        // overall scale so that the domes remain spherical in shape.
        const float xz_size = 0.5f * (trs.size.x + trs.size.y);
        if (index == 0) {
          return trs.ToTransform();
        } else if (index == 1) {
          // Move the first dome to the top of the capsule.
          mat4f top = mat4f(trs.rotation, trs.translation);
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(float3{trs.size.x, trs.size.y, xz_size});
          return top;
        } else if (index == 2) {
          // Move the second dome to the bottom of the capsule and rotate it 180
          // degrees so that it's facing the right way.
          mat4f bottom = mat4f(trs.rotation, trs.translation);
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          bottom *= mat4f::scaling(float3{trs.size.x, trs.size.y, xz_size});
          return bottom;
        } else {
          mju_error("Invalid index for capsule geom: %d (expected [0,2])", index);
          return trs.ToTransform();
        }
      };
      break;
    }
    case mjGEOM_CYLINDER: {
      // Cylinders are a tube with two disks at the ends.
      meshes.push_back(GetShape(model_objects, ModelObjects::kTube));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));

      get_transforms = [](int index, const Trs& trs) {
        if (index == 0) {
          return trs.ToTransform();
        } else if (index == 1) {
          // Move the first disk to the top of the cylinder.
          mat4f top = mat4f(trs.rotation, trs.translation);
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(trs.size);
          return top;
        } else if (index == 2) {
          // Move the second disk to the bottom of the cylinder. Rotate the disk
          // 180 degrees so that the normals point outwards.
          mat4f bottom = mat4f(trs.rotation, trs.translation);
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          bottom *= mat4f::scaling(trs.size);
          return bottom;
        } else {
          mju_error("Invalid index for cylinder geom: %d (expected [0,2])", index);
          return trs.ToTransform();
        }
      };
      break;
    }
    case mjGEOM_ARROW: {
      meshes.push_back(GetShape(model_objects, ModelObjects::kTube));
      meshes.push_back(GetShape(model_objects, ModelObjects::kCone));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));

      get_transforms = [](int index, const Trs& trs) {
        mat4f base = mat4f(trs.rotation, trs.translation);
        base *= mat4f::scaling(float3{1, 1, kArrowScale});
        base *= mat4f::translation(float3{0, 0, trs.size.z});
        if (index == 0) {
          return base * mat4f::scaling(trs.size);
        } else if (index == 1) {
          mat4f top = base;
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top * mat4f::scaling(trs.size);
        } else if (index == 2) {
          mat4f top_disk = base;
          top_disk *= mat4f::translation(float3{0, 0, trs.size.z});
          top_disk *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          top_disk *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top_disk * mat4f::scaling(trs.size);
        } else if (index == 3) {
          mat4f bottom = base;
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          return bottom * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow geom: %d (expected [0,3])", index);
          return trs.ToTransform();
        }
      };
      break;
    }
    case mjGEOM_ARROW1: {
      meshes.push_back(GetShape(model_objects, ModelObjects::kTube));
      meshes.push_back(GetShape(model_objects, ModelObjects::kCone));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));

      get_transforms = [](int index, const Trs& trs) {
      mat4f base = mat4f(trs.rotation, trs.translation);
      base *= mat4f::scaling(float3{1, 1, kArrowScale});
      base *= mat4f::translation(float3{0, 0, trs.size.z});
        if (index == 0) {
          return base * mat4f::scaling(trs.size);
        } else if (index == 1) {
          mat4f top = base;
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          return top * mat4f::scaling(trs.size);
        } else if (index == 2) {
          mat4f bottom = base;
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          return bottom * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow1 geom: %d (expected [0,2])", index);
          return trs.ToTransform();
        }
      };
      break;
    }
    case mjGEOM_ARROW2: {
      meshes.push_back(GetShape(model_objects, ModelObjects::kTube));
      meshes.push_back(GetShape(model_objects, ModelObjects::kCone));
      meshes.push_back(GetShape(model_objects, ModelObjects::kCone));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));
      meshes.push_back(GetShape(model_objects, ModelObjects::kDisk));

      get_transforms = [](int index, const Trs& trs) {
        mat4f base = mat4f(trs.rotation, trs.translation);
        base *= mat4f::scaling(float3{1, 1, kArrowScale});
        base *= mat4f::translation(float3{0, 0, trs.size.z});
        if (index == 0) {
          return base * mat4f::scaling(trs.size);
        } else if (index == 1) {
          mat4f top = base;
          top *= mat4f::translation(float3{0, 0, trs.size.z});
          top *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top * mat4f::scaling(trs.size);
        } else if (index == 2) {
          mat4f bottom = base;
          bottom *= mat4f::translation(float3{0, 0, -trs.size.z});
          bottom *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          bottom *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return bottom * mat4f::scaling(trs.size);
        } else if (index == 3) {
          mat4f top_disk = base;
          top_disk *= mat4f::translation(float3{0, 0, trs.size.z});
          top_disk *= mat4f::rotation(std::numbers::pi, float3{1, 0, 0});
          top_disk *= mat4f::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
          return top_disk * mat4f::scaling(trs.size);
        } else if (index == 4) {
          mat4f bottom_disk = base;
          bottom_disk *= mat4f::translation(float3{0, 0, -trs.size.z});
          return bottom_disk * mat4f::scaling(trs.size);
        } else {
          mju_error("Invalid index for arrow2 geom: %d (expected [0,4])", index);
          return trs.ToTransform();
        }
      };
      break;
    }
    case mjGEOM_LINE:
      meshes.push_back(GetShape(model_objects, ModelObjects::kLine));
      break;
    case mjGEOM_LINEBOX:
      meshes.push_back(GetShape(model_objects, ModelObjects::kLineBox));
      break;
    case mjGEOM_TRIANGLE:
      meshes.push_back(GetShape(model_objects, ModelObjects::kTriangle));
      break;
    case mjGEOM_FLEX:
      meshes.push_back(GetSkinFlexMesh(model_objects, geom.objid));
      // Flexes are defined in global space.
      trs = Trs();
      break;
    case mjGEOM_SKIN:
      meshes.push_back(GetSkinFlexMesh(model_objects, geom.objid));
      // Skins are defined in global space.
      trs = Trs();
      break;
    case mjGEOM_NONE:
    case mjGEOM_LABEL:
      // Do nothing.
      break;
    case mjGEOM_SDF:
    case mjNGEOMTYPES:
      mju_warning("Unsupported geom type: %d", geom.type);
      break;
  }

  renderable.SetMeshes(meshes, get_transforms);
  renderable.SetTransform(trs);
}

static void UpdateGeomMaterial(Renderable& renderable, const mjvGeom& geom,
                               const mjvScene* scene, ModelObjects* model_objs,
                               const float headpos[3]) {
  const mjModel* model = model_objs->GetModel();

  const bool use_segid_color = scene->flags[mjRND_IDCOLOR];
  mjrMaterialParams params;
  mjr_defaultMaterialParams(&params);
  params.color[0] = geom.rgba[0];
  params.color[1] = geom.rgba[1];
  params.color[2] = geom.rgba[2];
  params.color[3] = geom.rgba[3];
  if (geom.type == mjGEOM_PLANE) {
    if (IsBehind(headpos, geom.pos, geom.mat)) {
      params.color[3] *= 0.3;
      renderable.SetReceiveShadows(false);
      params.reflective = false;
    } else {
      renderable.SetReceiveShadows(true);
      params.reflective = geom.reflectance > 0 && params.color[3] == 1.0f;
    }
  }
  renderable.SetLayerMask(geom.category);
  if (geom.category == mjCAT_DECOR) {
    renderable.SetCastShadows(false);
    renderable.SetReceiveShadows(false);
  } else {
    renderable.SetWireframe(scene->flags[mjRND_WIREFRAME]);
  }

  mjrMaterialTextures textures;
  mjr_defaultMaterialTextures(&textures);
  if (geom.matid >= 0) {
    textures.color = model_objs->GetTexture(geom.matid, mjTEXROLE_RGB);
    textures.normal = model_objs->GetTexture(geom.matid, mjTEXROLE_NORMAL);
    textures.emissive = model_objs->GetTexture(geom.matid, mjTEXROLE_EMISSIVE);
    textures.orm = model_objs->GetTexture(geom.matid, mjTEXROLE_ORM);
    textures.metallic = model_objs->GetTexture(geom.matid, mjTEXROLE_METALLIC);
    textures.roughness =
        model_objs->GetTexture(geom.matid, mjTEXROLE_ROUGHNESS);
    textures.occlusion =
        model_objs->GetTexture(geom.matid, mjTEXROLE_OCCLUSION);
  }

  params.reflectance = geom.reflectance;
  params.emissive = geom.emission;
  params.specular = geom.specular;
  params.glossiness = geom.shininess;
  if (geom.matid >= 0) {
    params.metallic = model->mat_metallic[geom.matid];
    params.roughness = model->mat_roughness[geom.matid];
    params.tex_uniform = model->mat_texuniform[geom.matid];
    params.tex_repeat[0] = model->mat_texrepeat[(geom.matid * 2) + 0];
    params.tex_repeat[1] = model->mat_texrepeat[(geom.matid * 2) + 1];
  }

  if (geom.segid >= 0) {
    uint32_t segmentation_color = geom.segid + 1;
    if (!use_segid_color) {
      constexpr double phi1 = 1.61803398874989484820;  // Cached Phi(1).
      constexpr double coef1 = 1.0 / phi1;
      const double index = static_cast<double>(geom.segid);
      const double sample = std::fmod(0.5 + coef1 * index, 1.0);
      segmentation_color = 0x01000000 * sample;
    }

    const uint8_t red = (segmentation_color >> 0) & 0xff;
    const uint8_t green = (segmentation_color >> 8) & 0xff;
    const uint8_t blue = (segmentation_color >> 16) & 0xff;
    params.segmentation_color[0] = static_cast<float>(red) / 255.0f;
    params.segmentation_color[1] = static_cast<float>(green) / 255.0f;
    params.segmentation_color[2] = static_cast<float>(blue) / 255.0f;
  }

  // UvScale only applies to objects that don't have explicit UV coordinates
  // in their vertex buffer. Instead, we set the UV coordinate to be the same
  // as the vertex position.
  //
  // The material's `texuniform` and `texrepeat` parameters allow us to scale
  // the programmatic UVs.

  if (textures.color) {
    if (Texture::downcast(textures.color)->GetTarget() == mjTEXTURE_2D) {
      // For 2D textures, `tex_repeat` specifies how many times the texture
      // image is repeated. The `tex_uniform` flag determines if the repetition
      // is applied at in object space (false) or in world space (true).
      params.uv_scale[0] = params.tex_repeat[0];
      params.uv_scale[1] = params.tex_repeat[1];

      if (geom.dataid >= 0 && geom.type != mjGEOM_PLANE) {
        if (geom.size[0] > mjMINVAL) {
          params.uv_scale[0] /= geom.size[0];
        }
        if (geom.size[1] > mjMINVAL) {
          params.uv_scale[1] /= geom.size[1];
        }
      }
      if (params.tex_uniform) {
        if (geom.size[0] > 0) {
          params.uv_scale[0] *= geom.size[0];
        }
        if (geom.size[1] > 0) {
          params.uv_scale[1] *= geom.size[1];
        }
      }
      const bool is_infinite_plane =
          geom.type == mjGEOM_PLANE && (geom.size[0] <= 0 || geom.size[1] <= 0);
      if (is_infinite_plane) {
        // Infinite planes are scaled to match the tile size used by
        // re-centering in engine_vis_visualize.c.
        const float plane_scale = static_cast<float>(mjMAXPLANEGRID) / 2.0f;
        const float tile_size_x =
            GetPlaneTileSize(model, geom.matid, params.tex_repeat[0]);
        const float tile_size_y =
            GetPlaneTileSize(model, geom.matid, params.tex_repeat[1]);
        params.uv_scale[0] = 2.0f * plane_scale / tile_size_x;
        params.uv_scale[1] = 2.0f * plane_scale / tile_size_y;
      }

      // We want to do the equivalent of:
      //   mjr_setf4(splane, 0.5 * scl.x,  0, 0, -0.5);
      //   mjr_setf4(tplane, 0, -0.5 * scl.y, 0, -0.5);
      //   glTexGenfv(GL_S, GL_OBJECT_PLANE, splane);
      //   glTexGenfv(GL_T, GL_OBJECT_PLANE, tplane);
      params.uv_scale[0] = 0.5f * params.uv_scale[0];
      params.uv_scale[1] = -0.5f * params.uv_scale[1];
      params.uv_offset[0] = -0.5f;
      params.uv_offset[1] = -0.5f;
    } else {
      // For cube maps, if `tex_uniform` is true, then scale the texture so that
      // it covers a 1x1 area of world space rather than the area of the object.
      if (params.tex_uniform) {
        params.uv_scale[0] = 1.0f / (geom.size[0] ? geom.size[0] : 1.0f);
        params.uv_scale[1] = 1.0f / (geom.size[1] ? geom.size[1] : 1.0f);
        params.uv_scale[2] = 1.0f / (geom.size[2] ? geom.size[2] : 1.0f);
      }
    }
  }

  // Apply material multipliers from the model.
  params.emissive *= model_objs->GetEmissiveMultiplier();
  params.specular *= model_objs->GetSpecularMultiplier();
  params.glossiness *= model_objs->GetShininessMultiplier();

  renderable.UpdateMaterial(params, textures);
}

std::unique_ptr<Renderable> CreateGeomRenderable(
    const mjvGeom& geom, const mjvScene* scene, FilamentContext* ctx,
    ModelObjects* model_objs, const float headpos[3]) {
  mjrShadingModel shading_model = mjSHADING_MODEL_SCENE_OBJECT;
  if (geom.type == mjGEOM_LINE || geom.type == mjGEOM_LINEBOX) {
    shading_model = mjSHADING_MODEL_DECOR_LINES;
  } else if (geom.category == mjCAT_DECOR) {
    shading_model = mjSHADING_MODEL_DECOR;
  }

  mjrRenderableParams params;
  mjr_defaultRenderableParams(&params);
  params.shading_model = shading_model;
  auto renderable = std::make_unique<Renderable>(ctx, params);
  PrepareGeomMeshes(*renderable, geom, scene, model_objs);
  UpdateGeomMaterial(*renderable, geom, scene, model_objs, headpos);
  return renderable;
}
}  // namespace mujoco
