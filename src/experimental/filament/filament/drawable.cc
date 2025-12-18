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

#include "experimental/filament/filament/drawable.h"

#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

#include <filament/Material.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/Texture.h>
#include <filament/TransformManager.h>
#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/Entity.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/geom_util.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat4;

// Planes with a 0-size dimension should be infinitely large. However, we
// don't support that directly so we use a large but finite size instead.
static constexpr float kInfinitePlaneFakeSize = 1000.0f;

// An arbitrary scale factor for arrows.
static constexpr float kArrowScale = 1.f / 6.f;
static constexpr float kArrowHeadSize = 1.75f;

// Some built-in geometries are actually composed of multiple simple shapes. A
// capsule, for example, is a open-ended tube with two dome ends. We use these
// constants to help identify which entity (by index) represents which part of
// the overall shape.
static constexpr int kCapsuleTopDome = 1;
static constexpr int kCapsuleBottomDome = 2;
static constexpr int kCylinderTopDisk = 1;
static constexpr int kCylinderBottomDisk = 2;
static constexpr int kArrow0Cone = 1;
static constexpr int kArrow0ConeDisk = 2;
static constexpr int kArrow0BottomDisk = 3;
static constexpr int kArrow1Cone = 1;
static constexpr int kArrow1BottomDisk = 2;
static constexpr int kArrow2TopCone = 1;
static constexpr int kArrow2BottomCone = 2;
static constexpr int kArrow2TopConeDisk = 3;
static constexpr int kArrow2BottomConeDisk = 4;

Drawable::Drawable(ObjectManager* object_mgr, const mjvGeom& geom)
    : material_(object_mgr), renderables_(object_mgr->GetEngine()) {
  if (geom.category == mjCAT_DECOR) {
    renderables_.DisableShadows();
  }

  switch ((mjtGeom)geom.type) {
    case mjGEOM_MESH:
      AddMesh(geom.dataid);
      break;
    case mjGEOM_HFIELD:
      AddHeightField(geom.dataid);
      break;
    case mjGEOM_PLANE:
      AddShape(ObjectManager::kPlane);
      break;
    case mjGEOM_SPHERE:
      AddShape(ObjectManager::kSphere);
      break;
    case mjGEOM_ELLIPSOID:
      AddShape(ObjectManager::kSphere);
      break;
    case mjGEOM_BOX:
      AddShape(ObjectManager::kBox);
      break;
    case mjGEOM_CAPSULE:
      AddShape(ObjectManager::kTube);
      AddShape(ObjectManager::kDome);
      AddShape(ObjectManager::kDome);
      break;
    case mjGEOM_CYLINDER:
      AddShape(ObjectManager::kTube);
      AddShape(ObjectManager::kDisk);
      AddShape(ObjectManager::kDisk);
      break;
    case mjGEOM_ARROW:
      AddShape(ObjectManager::kTube);
      AddShape(ObjectManager::kCone);
      AddShape(ObjectManager::kDisk);
      break;
    case mjGEOM_ARROW1:
      AddShape(ObjectManager::kTube);
      AddShape(ObjectManager::kCone);
      AddShape(ObjectManager::kDisk);
      AddShape(ObjectManager::kDisk);
      break;
    case mjGEOM_ARROW2:
      AddShape(ObjectManager::kTube);
      AddShape(ObjectManager::kCone);
      AddShape(ObjectManager::kCone);
      AddShape(ObjectManager::kDisk);
      AddShape(ObjectManager::kDisk);
      break;
    case mjGEOM_LINE:
      AddShape(ObjectManager::kLine);
      break;
    case mjGEOM_LINEBOX:
      AddShape(ObjectManager::kLineBox);
      break;
    case mjGEOM_TRIANGLE:
      AddShape(ObjectManager::kTriangle);
      break;
    case mjGEOM_FLEX:
    case mjGEOM_SKIN:
      // Flex and skin geometries are dynamically updated every frame.
      break;
    case mjGEOM_NONE:
    case mjGEOM_LABEL:
      // Do nothing .
      break;
    case mjGEOM_SDF:
    case mjNGEOMTYPES:
      mju_warning("Unsupported geom type: %d", geom.type);
      break;
  }
}

void Drawable::Update(const mjModel* model, const mjvScene* scene,
                      const mjvGeom& geom) {
  if (geom.type == mjGEOM_FLEX || geom.type == mjGEOM_SKIN) {
    // Flex geometry is updated every frame with new vertex data.
    filament::Engine* engine = renderables_.GetEngine();
    FilamentBuffers buffers = CreateGeomBuffers(engine, model, scene, geom);

    if (renderables_.GetNumEntities() == 0) {
      renderables_.Append(std::move(buffers));
    } else {
      renderables_.Update(0, std::move(buffers));
    }
  }

  SetTransform(geom);
  UpdateMaterial(geom, scene->flags[mjRND_IDCOLOR]);
}

void Drawable::AddMesh(int data_id) {
  ObjectManager* object_mgr = material_.GetObjectManager();
  const FilamentBuffers* buffers = object_mgr->GetMeshBuffer(data_id);
  if (buffers == nullptr) {
    mju_error("Unknown mesh %d", data_id);
  }
  renderables_.Append(*buffers);
}

void Drawable::AddHeightField(int hfield_id) {
  ObjectManager* object_mgr = material_.GetObjectManager();
  const FilamentBuffers* buffers = object_mgr->GetHeightFieldBuffer(hfield_id);
  if (buffers == nullptr) {
    mju_error("Unknown height field %d", hfield_id);
  }
  renderables_.Append(*buffers);
}

void Drawable::AddShape(ObjectManager::ShapeType shape_type) {
  ObjectManager* object_mgr = material_.GetObjectManager();
  const FilamentBuffers* buffers = object_mgr->GetShapeBuffer(shape_type);
  if (buffers == nullptr) {
    mju_error("Unknown shape %d", shape_type);
  }
  renderables_.Append(*buffers);
}

void Drawable::AddToScene(filament::Scene* scene) {
  renderables_.AddToScene(scene);
}

void Drawable::RemoveFromScene(filament::Scene* scene) {
  renderables_.RemoveFromScene(scene);
}

void Drawable::SetDrawMode(Material::DrawMode mode) {
  renderables_.SetMaterialInstance(material_.GetMaterialInstance(mode));
}

void Drawable::SetTransform(const mjvGeom& geom) {
  // Flex and skin geometries are in global space.
  if (geom.type == mjGEOM_FLEX || geom.type == mjGEOM_SKIN) {
    return;
  }

  const mat4 transform(ReadMat3(geom.mat), ReadFloat3(geom.pos));

  float3 size = ReadFloat3(geom.size);
  filament::TransformManager& tm =
      renderables_.GetEngine()->getTransformManager();
  for (int j = 0; j < renderables_.GetNumEntities(); ++j) {
    const utils::Entity& entity = renderables_[j];

    // Update object transform.
    mat4 entity_transform = transform;

    // Some built-in drawables are composed of multiple entities. For example,
    // capsules are a combination of a open tube and two dome end caps.

    if (geom.type == mjGEOM_CYLINDER) {
      // Cylinders are a tube with two disks at the ends. The "bottom" disk is
      // rotated so that the normals point outwards.
      if (j == kCylinderTopDisk) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
      } else if (j == kCylinderBottomDisk) {
        entity_transform *= mat4::translation(float3{0, 0, -size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
      }
    } else if (geom.type == mjGEOM_CAPSULE) {
      // Capsules are a tube with two domes at the ends. We apply an inverse
      // scale to the domes to "counteract" the capsule's overall scale so that
      // the domes remain spherical in shape.
      const float xz_size = 0.5f * (size.x + size.y);
      if (j == kCapsuleTopDome) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
        entity_transform *= mat4::scaling(float3{1, 1, xz_size / size.z});
      } else if (j == kCapsuleBottomDome) {
        entity_transform *= mat4::translation(float3{0, 0, -size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
        entity_transform *= mat4::scaling(float3{1, 1, xz_size / size.z});
      }
    } else if (geom.type == mjGEOM_ARROW) {
      // An arrow is a tube with a cone at the end and a disk cap at the other
      // end. Because the cone head's base is larger than the tube, an extra
      // disk is added to the base of the cone. This disk is rotated such that
      // its normal points outwards.
      entity_transform *= mat4::scaling(float3{1, 1, kArrowScale});
      entity_transform *= mat4::translation(float3{0, 0, size.z});
      if (j == kArrow0Cone) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
        entity_transform *=
            mat4::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
      } else if (j == kArrow0ConeDisk) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
        entity_transform *=
            mat4::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
      } else if (j == kArrow0BottomDisk) {
        entity_transform *= mat4::translation(float3{0, 0, -size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
      }
    } else if (geom.type == mjGEOM_ARROW1) {
      // An arrow1 is a tube with a cone at the end and a disk cap at the other
      // end.
      entity_transform *= mat4::scaling(float3{1, 1, kArrowScale});
      entity_transform *= mat4::translation(float3{0, 0, size.z});
      if (j == kArrow1Cone) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
      } else if (j == kArrow1BottomDisk) {
        entity_transform *= mat4::translation(float3{0, 0, -size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
      }
    } else if (geom.type == mjGEOM_ARROW2) {
      // An arrow2 is a tube with a cone at both ends.  Like the standard arrow,
      // an extra disk is added to the base of each cone.
      entity_transform *= mat4::scaling(float3{1, 1, kArrowScale});
      entity_transform *= mat4::translation(float3{0, 0, size.z});
      if (j == kArrow2TopCone) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
        entity_transform *=
            mat4::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
      } else if (j == kArrow2BottomCone) {
        entity_transform *= mat4::translation(float3{0, 0, -size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
        entity_transform *=
            mat4::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
      } else if (j == kArrow2TopConeDisk) {
        entity_transform *= mat4::translation(float3{0, 0, size.z});
        entity_transform *= mat4::rotation(std::numbers::pi, float3{1, 0, 0});
        entity_transform *=
            mat4::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
      } else if (j == kArrow2BottomConeDisk) {
        entity_transform *= mat4::translation(float3{0, 0, -size.z});
        entity_transform *=
            mat4::scaling(float3{kArrowHeadSize, kArrowHeadSize, 1.0f});
      }
    } else if (geom.type == mjGEOM_PLANE) {
      // A plane with 0-size in any dimension is considered to be an infinite
      // plane, but we don't really support that so just scale it so that its
      // very large.
      if (size.x == 0 && size.y == 0) {
        size = float3(kInfinitePlaneFakeSize, kInfinitePlaneFakeSize, size.z);
      }
    }

    if (geom.type != mjGEOM_MESH && geom.type != mjGEOM_HFIELD) {
      entity_transform *= mat4::scaling(size);
    }
    tm.setTransform(tm.getInstance(entity), entity_transform);
  }
}

void Drawable::UpdateMaterial(const mjvGeom& geom, bool use_segid_color) {
  ObjectManager* object_mgr = material_.GetObjectManager();
  const mjModel* model = object_mgr->GetModel();

  Material::Textures textures;
  if (geom.matid >= 0) {
    textures.color = object_mgr->GetTexture(geom.matid, mjTEXROLE_RGB);
    textures.normal = object_mgr->GetTexture(geom.matid, mjTEXROLE_NORMAL);
    textures.emissive = object_mgr->GetTexture(geom.matid, mjTEXROLE_EMISSIVE);
    textures.orm = object_mgr->GetTexture(geom.matid, mjTEXROLE_ORM);
    textures.metallic = object_mgr->GetTexture(geom.matid, mjTEXROLE_METALLIC);
    textures.roughness =
        object_mgr->GetTexture(geom.matid, mjTEXROLE_ROUGHNESS);
    textures.occlusion =
        object_mgr->GetTexture(geom.matid, mjTEXROLE_OCCLUSION);
    material_.UpdateTextures(textures);
  }

  if (geom.type == mjGEOM_LINE || geom.type == mjGEOM_LINEBOX) {
    material_.SetNormalMaterialType(ObjectManager::kUnlitLine);
  } else {
    bool material_assigned = false;
    if (geom.matid >= 0) {
      material_assigned = true;
      if (textures.orm) {
        material_.SetNormalMaterialType(ObjectManager::kPbrPacked);
      } else if (textures.metallic) {
        material_.SetNormalMaterialType(ObjectManager::kPbr);
      } else if (textures.roughness) {
        material_.SetNormalMaterialType(ObjectManager::kPbr);
      } else if (model->mat_metallic[geom.matid] >= 0) {
        material_.SetNormalMaterialType(ObjectManager::kPbr);
      } else if (model->mat_roughness[geom.matid] >= 0) {
        material_.SetNormalMaterialType(ObjectManager::kPbr);
      } else {
        material_assigned = false;
      }
    }

    if (!material_assigned) {
      // Check to see if we're dealing with a mesh with texture coordinates.
      // `data_id` is the id of the mesh in model (i.e. the geom has mesh
      // geometry) and `mesh_texcoordadr` stores the address of the mesh uvs if
      // it has them.
      bool has_texcoords = false;
      if ((geom.type == mjGEOM_MESH || geom.type == mjGEOM_SDF) &&
          geom.dataid >= 0 && model->mesh_texcoordadr[geom.dataid / 2] >= 0) {
        has_texcoords = true;
      }

      if (textures.color == nullptr) {
        if (geom.rgba[3] < 1.0f) {
          material_.SetNormalMaterialType(ObjectManager::kPhongColorFade);
        } else {
          material_.SetNormalMaterialType(ObjectManager::kPhongColor);
        }
      } else if (textures.color->getTarget() ==
                filament::Texture::Sampler::SAMPLER_CUBEMAP) {
        if (geom.rgba[3] < 1.0f) {
          material_.SetNormalMaterialType(ObjectManager::kPhongCubeFade);
        } else {
          material_.SetNormalMaterialType(ObjectManager::kPhongCube);
        }
      } else if (has_texcoords) {
        if (geom.rgba[3] < 1.0f) {
          material_.SetNormalMaterialType(ObjectManager::kPhong2dUvFade);
        } else {
          material_.SetNormalMaterialType(ObjectManager::kPhong2dUv);
        }
      } else {
        if (geom.rgba[3] < 1.0f) {
          material_.SetNormalMaterialType(ObjectManager::kPhong2dFade);
        } else {
          material_.SetNormalMaterialType(ObjectManager::kPhong2d);
        }
      }
    }
  }

  Material::Params params;
  params.color = ReadFloat4(geom.rgba);
  params.emissive = geom.emission;
  params.specular = geom.specular;
  params.glossiness = geom.shininess;
  if (geom.matid >= 0) {
    params.metallic = model->mat_metallic[geom.matid];
    params.roughness = model->mat_roughness[geom.matid];
    params.tex_uniform = model->mat_texuniform[geom.matid];
    params.tex_repeat = ReadFloat2(model->mat_texrepeat, geom.matid);
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
    params.segmentation_color.x = static_cast<float>(red) / 255.0f;
    params.segmentation_color.y = static_cast<float>(green) / 255.0f;
    params.segmentation_color.z = static_cast<float>(blue) / 255.0f;
  }

  // UvScale only applies to objects that don't have explicit UV coordinates
  // in their vertex buffer. Instead, we set the UV coordinate to be the same
  // as the vertex position.
  //
  // The material's `texuniform` and `texrepeat` parameters allow us to scale
  // the programmatic UVs.

  if (textures.color) {
    if (textures.color->getTarget() == filament::Texture::Sampler::SAMPLER_2D) {
      // For 2D textures, `tex_repeat` specifies how many times the texture
      // image is repeated. The `tex_uniform` flag determines if the repetition
      // is applied at in object space (false) or in world space (true).
      params.uv_scale.x = params.tex_repeat.x;
      params.uv_scale.y = params.tex_repeat.y;

      if (geom.dataid >= 0 && geom.type != mjGEOM_PLANE) {
        if (geom.size[0] > mjMINVAL) {
          params.uv_scale.x /= geom.size[0];
        }
        if (geom.size[1] > mjMINVAL) {
          params.uv_scale.y /= geom.size[1];
        }
      }
      if (params.tex_uniform) {
        if (geom.size[0] > 0) {
          params.uv_scale.x *= geom.size[0];
        }
        if (geom.size[1] > 0) {
          params.uv_scale.y *= geom.size[1];
        }
      }
      if (geom.type == mjGEOM_PLANE) {
        if (geom.size[0] == 0) {
          params.uv_scale.x = kInfinitePlaneFakeSize;
        }
        if (geom.size[1] == 0) {
          params.uv_scale.y = kInfinitePlaneFakeSize;
        }
      }

      params.uv_scale.x = 0.5f * params.uv_scale.x;
      params.uv_scale.y = 0.5f * params.uv_scale.y;
    } else {
      // For cube maps, if `tex_uniform` is true, then scale the texture so that
      // it covers a 1x1 area of world space rather than the area of the object.
      if (params.tex_uniform) {
        params.uv_scale.x = 1.0f / (geom.size[0] ? geom.size[0] : 1.0f);
        params.uv_scale.y = 1.0f / (geom.size[1] ? geom.size[1] : 1.0f);
        params.uv_scale.z = 1.0f / (geom.size[2] ? geom.size[2] : 1.0f);
      }
    }
  }
  material_.UpdateParams(params);
}
}  // namespace mujoco
