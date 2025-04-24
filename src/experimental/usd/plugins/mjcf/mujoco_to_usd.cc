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

#include "mjcf/mujoco_to_usd.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjcf/utils.h"
#include <pxr/base/arch/attributes.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/callContext.h>
#include <pxr/base/tf/diagnostic.h>
#include <pxr/base/tf/diagnosticHelper.h>
#include <pxr/base/tf/enum.h>
#include <pxr/base/tf/registryManager.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/stringUtils.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/dictionary.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/sdf/abstractData.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdLux/tokens.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <pxr/usd/usdShade/tokens.h>
#include <pxr/usdImaging/usdImaging/tokens.h>

namespace {

// The ID of the World in mjModel and mjData.
static constexpr int kWorldIndex = 0;

// Using to satisfy TF_DEFINE_PRIVATE_TOKENS macro below and avoid operating in
// PXR_NS.
using pxr::TfToken;
template <typename T>
using TfStaticData = pxr::TfStaticData<T>;

// clang-format off
TF_DEFINE_PRIVATE_TOKENS(kTokens,
                         // Xform ops
                         ((body, "Body"))
                         ((body_name, "mujoco:body_name"))
                         ((geom, "Geom"))
                         ((light, "Light"))
                         ((meshScope, "MeshSources"))
                         ((materialsScope, "Materials"))
                         ((surface, "PreviewSurface"))
                         ((world, "World"))
                         ((xformOpTransform, "xformOp:transform"))
                         ((xformOpScale, "xformOp:scale"))
                         (st)
                         ((primvarsSt, "primvars:st"))
                         ((outputsSt, "outputs:st"))
                         ((inputsSt, "inputs:st"))
                         ((inputsVarname, "inputs:varname"))
                         ((inputsFile, "inputs:file"))
                         ((inputsWrapS, "inputs:wrapS"))
                         ((inputsWrapT, "inputs:wrapT"))
                         ((inputsDiffuseColor, "inputs:diffuseColor"))
                         ((outputsRgb, "outputs:rgb"))
                         ((inputsMetallic, "inputs:metallic"))
                         (repeat)
                         );

// Using to satisfy TF_REGISTRY_FUNCTION macro below and avoid operating in PXR_NS.
using pxr::TfEnum;
using pxr::Tf_RegistryStaticInit;
using pxr::Tf_RegistryInit;
using pxr::TfEnum;
template <typename T>
using Arch_PerLibInit = pxr::Arch_PerLibInit<T>;
enum ErrorCodes { UnsupportedGeomTypeError, MujocoCompilationError };

TF_REGISTRY_FUNCTION(pxr::TfEnum) {
  TF_ADD_ENUM_NAME(UnsupportedGeomTypeError, "UsdGeom type is unsupported.")
  TF_ADD_ENUM_NAME(MujocoCompilationError, "Mujoco spec failed to compile.")
}

// Usings to satisfy TF_ERROR macro.
using pxr::TfCallContext;
using pxr::Tf_PostErrorHelper;
// clang-format on

using mujoco::usd::AddAttributeConnection;
using mujoco::usd::AddPrimInherit;
using mujoco::usd::AddPrimReference;
using mujoco::usd::ApplyApiSchema;
using mujoco::usd::CreateAttributeSpec;
using mujoco::usd::CreateClassSpec;
using mujoco::usd::CreatePrimSpec;
using mujoco::usd::CreateRelationshipSpec;
using mujoco::usd::SetAttributeDefault;
using mujoco::usd::SetAttributeMetadata;
using mujoco::usd::SetLayerMetadata;
using mujoco::usd::SetPrimKind;
using mujoco::usd::SetPrimMetadata;
using mujoco::usd::SetPrimPurpose;

pxr::GfMatrix4d MujocoPosQuatToTransform(double *pos, double *quat) {
  pxr::GfQuatd quaternion = pxr::GfQuatd::GetIdentity();
  quaternion.SetReal(quat[0]);
  quaternion.SetImaginary(quat[1], quat[2], quat[3]);
  pxr::GfRotation rotation(quaternion);

  pxr::GfVec3d translation(0.0, 0.0, 0.0);
  translation.Set(pos[0], pos[1], pos[2]);

  pxr::GfMatrix4d transform;
  transform.SetTransform(rotation, translation);
  return transform;
}

}  // namespace

class ModelWriter {
 public:
  ModelWriter(mjSpec *spec, mjModel *model, pxr::SdfAbstractDataRefPtr &data)
      : spec_(spec), model_(model), data_(data), class_path_("/Bad_Path") {
    body_paths_ = std::vector<pxr::SdfPath>(model->nbody);
    body_xforms_ = std::vector<pxr::GfMatrix4d>(model->nbody);
  }
  ~ModelWriter() { mj_deleteModel(model_); }

  void Write(bool write_physics) {
    // Create top level class holder.
    class_path_ = CreateClassSpec(data_, pxr::SdfPath::AbsoluteRootPath(),
                                  pxr::TfToken("__class__"));

    // Create the world body.
    body_paths_[kWorldIndex] = WriteWorldBody(kWorldIndex);
    body_xforms_[kWorldIndex] = pxr::GfMatrix4d().SetIdentity();

    SetLayerMetadata(data_, pxr::SdfFieldKeys->Documentation,
                     "Generated by mujoco model writer.");
    // Mujoco is Z up by default.
    SetLayerMetadata(data_, pxr::UsdGeomTokens->upAxis, pxr::UsdGeomTokens->z);
    // Mujoco is authored in meters by default.
    SetLayerMetadata(data_, pxr::UsdGeomTokens->metersPerUnit,
                     pxr::UsdGeomLinearUnits::meters);

    // Set the world body to be the default prim for referencing/payloads.
    SetLayerMetadata(data_, pxr::SdfFieldKeys->DefaultPrim,
                     body_paths_[kWorldIndex].GetNameToken());

    // Author mesh scope + mesh prims to be referenced.
    WriteMeshes();
    WriteMaterials();
    WriteBodies(write_physics);
  }

 private:
  mjSpec *spec_;
  mjModel *model_;

  // This is a handle to the Sdf data to be written into the generated USD
  // layer.
  pxr::SdfAbstractDataRefPtr &data_;
  // Path to top level class spec that all classes should be children of.
  pxr::SdfPath class_path_;
  // Mapping from Mujoco body id to SdfPath.
  std::vector<pxr::SdfPath> body_paths_;
  // Mapping from Mujoco body id to world space transform.
  std::vector<pxr::GfMatrix4d> body_xforms_;
  // Mapping from mesh names to Mesh prim path.
  std::unordered_map<std::string, pxr::SdfPath> mesh_paths_;

  // Given a name index and a parent prim path this returns a
  // token such that appending it to the parent prim path does not
  // identify an existing prim spec.
  //
  // This is necessary since mujoco does not require names for elements
  // so we must differentiate between elements of the same type.
  //
  // For example:
  // <mujoco model="model with two planes">
  //    <worldbody>
  //      <geom type="plane"/>
  //      <geom type="plane"/>
  //    </worldbody>
  //  </mujoco>
  //
  //  We expect that the occurrence of this happens little enough that linear
  //  searching is plenty efficient.
  pxr::TfToken GetAvailablePrimName(const std::string base_name,
                                    const pxr::TfToken fallback_name,
                                    const pxr::SdfPath &parent_path) {
    const auto valid_base_name = pxr::TfMakeValidIdentifier(
        base_name.empty() ? fallback_name : base_name);
    std::string name = valid_base_name;
    pxr::SdfPath test_path = parent_path.AppendChild(pxr::TfToken(name));
    int count = 1;
    while (data_->HasSpec(test_path) &&
           data_->GetSpecType(test_path) == pxr::SdfSpecType::SdfSpecTypePrim) {
      name = pxr::TfStringPrintf("%s_%d", valid_base_name.c_str(), count++);
      test_path = parent_path.AppendChild(pxr::TfToken(name));
    }
    return pxr::TfToken(name);
  }

  // This function, conversely to GetAvailablePrimName will not handle
  // collisions. This is useful when looking up a prim path that might exist or
  // a path that you know must be unique.
  pxr::TfToken GetValidPrimName(const std::string name) {
    return pxr::TfToken(pxr::TfMakeValidIdentifier(name));
  }

  struct BodyPathComponents {
    pxr::SdfPath parent_path;
    pxr::TfToken body_name;
  };

  pxr::SdfPath CreateParentIfNotExists(mjsBody *body,
                                       const pxr::SdfPath &world_path,
                                       pxr::SdfAbstractDataRefPtr &data) {
    // To allow for easier scene authoring and modification, we want to
    // place MJCF bodies belonging to the same kinematic chain under some
    // identity parent Xform prim. This allows users to move the entire
    // asset.
    //
    // We cannot simply recreate the MJCF kinematic tree structure
    // because in USD it is assumed that children move rigidly with their
    // parents. This is not true in MJCF if you have joints. We could perhaps
    // use a more complex heuristic where we evaluate a common tree prefix
    // in MJCF that is effectively welded together but for now we choose
    // simplicity.

    // In the trivial case where the parent of body is already the world
    // body we want to create a parent xform of the same name.
    // So if the MJCF has a child of the world body called "root" we will
    // create a parent Xform at /World/root and the actual body will be
    // created at /World/root/root.
    mjsBody *last_parent = body;
    mjsBody *parent = mjs_getParent(body->element);
    while (mjs_getId(parent->element) != kWorldIndex) {
      last_parent = parent;
      parent = mjs_getParent(parent->element);
    }

    pxr::TfToken last_parent_name = GetValidPrimName(*last_parent->name);
    pxr::SdfPath parent_xform_path = world_path.AppendChild(last_parent_name);
    if (!data->HasSpec(parent_xform_path)) {
      pxr::SdfPath prim_path = CreatePrimSpec(
          data, world_path, last_parent_name, pxr::UsdGeomTokens->Xform);

      SetPrimKind(data_, prim_path, pxr::KindTokens->component);
    }
    return parent_xform_path;
  }

  void WriteScaleXformOp(const pxr::SdfPath &prim_path,
                         const pxr::GfVec3f &scale) {
    pxr::SdfPath scale_attr_path =
        CreateAttributeSpec(data_, prim_path, kTokens->xformOpScale,
                            pxr::SdfValueTypeNames->Float3);
    SetAttributeDefault(data_, scale_attr_path, scale);
  }

  void WriteTransformXformOp(const pxr::SdfPath &prim_path,
                             const pxr::GfMatrix4d &transform) {
    pxr::SdfPath transform_op_path =
        CreateAttributeSpec(data_, prim_path, kTokens->xformOpTransform,
                            pxr::SdfValueTypeNames->Matrix4d);
    SetAttributeDefault(data_, transform_op_path, transform);
  }

  void WriteXformOpOrder(const pxr::SdfPath &prim_path,
                         const pxr::VtArray<pxr::TfToken> &order) {
    pxr::SdfPath xform_op_order_path =
        CreateAttributeSpec(data_, prim_path, pxr::UsdGeomTokens->xformOpOrder,
                            pxr::SdfValueTypeNames->TokenArray);
    SetAttributeDefault(data_, xform_op_order_path, order);
  }

  void PrependToXformOpOrder(const pxr::SdfPath &prim_path,
                             const pxr::VtArray<pxr::TfToken> &order) {
    auto xform_op_order_path =
        prim_path.AppendProperty(pxr::UsdGeomTokens->xformOpOrder);
    if (!data_->HasSpec(xform_op_order_path)) {
      WriteXformOpOrder(prim_path, order);
      return;
    }

    auto existing_order =
        data_->Get(xform_op_order_path, pxr::SdfFieldKeys->Default)
            .UncheckedGet<pxr::VtArray<pxr::TfToken>>();

    pxr::VtArray<pxr::TfToken> new_order(order.size() + existing_order.size());
    std::copy(order.begin(), order.end(), new_order.begin());
    std::copy(existing_order.begin(), existing_order.end(),
              new_order.begin() + order.size());

    SetAttributeDefault(data_, xform_op_order_path, new_order);
  }

  void WriteMesh(const mjsMesh *mesh, const pxr::SdfPath &parent_path) {
    auto name = GetAvailablePrimName(*mesh->name, pxr::UsdGeomTokens->Mesh,
                                     parent_path);
    pxr::SdfPath subcomponent_path =
        CreatePrimSpec(data_, parent_path, name, pxr::UsdGeomTokens->Xform);
    pxr::SdfPath mesh_path =
        CreatePrimSpec(data_, subcomponent_path, pxr::UsdGeomTokens->Mesh,
                       pxr::UsdGeomTokens->Mesh);
    mesh_paths_[*mesh->name] = subcomponent_path;

    // NOTE: The geometry data taken from the spec is the post-compilation
    // data after it has been mjCMesh::Compile'd. So don't be surprised if
    // things like user defined vertices have moved due to re-centering to
    // CoM and other modifications (see mjCMesh::Process for other xforms).
    int mesh_id = mjs_getId(mesh->element);
    int vert_start_offset = model_->mesh_vertadr[mesh_id] * 3;
    int nvert = model_->mesh_vertnum[mesh_id];
    pxr::VtArray<pxr::GfVec3f> points;
    points.reserve(nvert);
    for (int i = vert_start_offset; i < vert_start_offset + nvert * 3; i += 3) {
      points.emplace_back(&model_->mesh_vert[i]);
    }

    pxr::SdfPath points_attr_path =
        CreateAttributeSpec(data_, mesh_path, pxr::UsdGeomTokens->points,
                            pxr::SdfValueTypeNames->Vector3fArray);
    SetAttributeDefault(data_, points_attr_path, points);

    // NOTE: nface is never 0.
    int nface = model_->mesh_facenum[mesh_id];
    pxr::VtArray<int> faces;
    faces.reserve(nface * 3);
    int face_start_offset = model_->mesh_faceadr[mesh_id] * 3;
    for (int i = face_start_offset; i < face_start_offset + nface * 3; i += 3) {
      faces.push_back(model_->mesh_face[i]);
      faces.push_back(model_->mesh_face[i + 1]);
      faces.push_back(model_->mesh_face[i + 2]);
    }
    pxr::SdfPath face_vertex_idx_attr_path = CreateAttributeSpec(
        data_, mesh_path, pxr::UsdGeomTokens->faceVertexIndices,
        pxr::SdfValueTypeNames->IntArray);
    SetAttributeDefault(data_, face_vertex_idx_attr_path, faces);

    pxr::VtArray<int> vertex_counts;
    for (int i = 0; i < nface; ++i) {
      // Mujoco is always triangles.
      vertex_counts.push_back(3);
    }
    pxr::SdfPath face_vertex_counts_attr_path = CreateAttributeSpec(
        data_, mesh_path, pxr::UsdGeomTokens->faceVertexCounts,
        pxr::SdfValueTypeNames->IntArray);
    SetAttributeDefault(data_, face_vertex_counts_attr_path, vertex_counts);

    if (model_->mesh_normalnum[mesh_id]) {
      // We have to convert from Mujoco's indexed normals to USD's faceVarying
      // normals.
      pxr::VtArray<pxr::GfVec3f> normals;
      normals.reserve(nface * 3);
      int normal_start_adr = model_->mesh_normaladr[mesh_id];
      int face_start_offset = model_->mesh_faceadr[mesh_id] * 3;
      for (int i = face_start_offset; i < face_start_offset + nface * 3; ++i) {
        int normal_adr = normal_start_adr + model_->mesh_facenormal[i];
        normals.emplace_back(&model_->mesh_normal[normal_adr * 3]);
      }
      pxr::SdfPath normals_attr_path =
          CreateAttributeSpec(data_, mesh_path, pxr::UsdGeomTokens->normals,
                              pxr::SdfValueTypeNames->Vector3fArray);
      SetAttributeDefault(data_, normals_attr_path, normals);
      SetAttributeMetadata(data_, normals_attr_path,
                           pxr::UsdGeomTokens->interpolation,
                           pxr::UsdGeomTokens->faceVarying);
    }

    if (model_->mesh_texcoordnum[mesh_id]) {
      // We have to convert from Mujoco's indexed texcoords to USD's faceVarying
      // texcoords.
      pxr::VtArray<pxr::GfVec2f> texcoords;
      texcoords.reserve(nface * 3);
      int texcoord_start_adr = model_->mesh_texcoordadr[mesh_id];
      int face_start_offset = model_->mesh_faceadr[mesh_id] * 3;
      for (int i = face_start_offset; i < face_start_offset + nface * 3; ++i) {
        int texcoord_adr = texcoord_start_adr + model_->mesh_facetexcoord[i];
        // Invert the V coordinate, Mujoco assumes OpenGL 0,0 is top left.
        // But USD UVs use image bottom left 0,0 convention.
        pxr::GfVec2f uv(&model_->mesh_texcoord[texcoord_adr * 2]);
        uv[1] = 1.0f - uv[1];
        texcoords.push_back(uv);
      }

      pxr::SdfPath texcoords_attr_path =
          CreateAttributeSpec(data_, mesh_path, kTokens->primvarsSt,
                              pxr::SdfValueTypeNames->TexCoord2fArray);
      SetAttributeDefault(data_, texcoords_attr_path, texcoords);
      SetAttributeMetadata(data_, texcoords_attr_path,
                           pxr::UsdGeomTokens->interpolation,
                           pxr::UsdGeomTokens->faceVarying);
    }

    // Default subdivision scheme is catmull clark so explicitly set it
    // to none here.
    pxr::SdfPath subdivision_scheme_path = CreateAttributeSpec(
        data_, mesh_path, pxr::UsdGeomTokens->subdivisionScheme,
        pxr::SdfValueTypeNames->Token);
    SetAttributeDefault(data_, subdivision_scheme_path,
                        pxr::UsdGeomTokens->none);
  }

  void WriteMeshes() {
    // Create a scope for the meshes to keep things organized
    pxr::SdfPath scope_path =
        CreatePrimSpec(data_, body_paths_[kWorldIndex], kTokens->meshScope,
                       pxr::UsdGeomTokens->Scope);

    // Make the mesh scope invisible since they will be referenced by the bits
    // that should be visible.
    SetPrimMetadata(data_, scope_path, pxr::SdfFieldKeys->Active, false);

    mjsMesh *mesh = mjs_asMesh(mjs_firstElement(spec_, mjOBJ_MESH));
    while (mesh) {
      WriteMesh(mesh, scope_path);
      mesh = mjs_asMesh(mjs_nextElement(spec_, mesh->element));
    }
  }

  pxr::SdfPath AddTextureShader(const pxr::SdfPath &material_path,
                                const char *texture_file) {
    // Shader "uvmap"
    pxr::SdfPath uvmap_shader_path =
        CreatePrimSpec(data_, material_path, pxr::TfToken("uvmap"),
                       pxr::UsdShadeTokens->Shader);

    pxr::SdfPath uvmap_info_id_attr = CreateAttributeSpec(
        data_, uvmap_shader_path, pxr::UsdShadeTokens->infoId,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
    SetAttributeDefault(data_, uvmap_info_id_attr,
                        pxr::UsdImagingTokens->UsdPrimvarReader_float2);

    pxr::SdfPath uvmap_varname_attr =
        CreateAttributeSpec(data_, uvmap_shader_path, kTokens->inputsVarname,
                            pxr::SdfValueTypeNames->Token);
    SetAttributeDefault(data_, uvmap_varname_attr, kTokens->st);

    pxr::SdfPath uvmap_st_output_attr =
        CreateAttributeSpec(data_, uvmap_shader_path, kTokens->outputsSt,
                            pxr::SdfValueTypeNames->Float2);

    // Shader "texture"
    pxr::SdfPath texture_shader_path =
        CreatePrimSpec(data_, material_path, pxr::TfToken("texture"),
                       pxr::UsdShadeTokens->Shader);
    pxr::SdfPath texture_info_id_attr = CreateAttributeSpec(
        data_, texture_shader_path, pxr::UsdShadeTokens->infoId,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
    SetAttributeDefault(data_, texture_info_id_attr,
                        pxr::UsdImagingTokens->UsdUVTexture);

    pxr::SdfPath texture_file_attr =
        CreateAttributeSpec(data_, texture_shader_path, kTokens->inputsFile,
                            pxr::SdfValueTypeNames->Asset);
    SetAttributeDefault(data_, texture_file_attr,
                        pxr::SdfAssetPath(texture_file));

    pxr::SdfPath texture_st_input_attr =
        CreateAttributeSpec(data_, texture_shader_path, kTokens->inputsSt,
                            pxr::SdfValueTypeNames->Float2);
    AddAttributeConnection(data_, texture_st_input_attr, uvmap_st_output_attr);

    pxr::SdfPath texture_wrap_s_attr =
        CreateAttributeSpec(data_, texture_shader_path, kTokens->inputsWrapS,
                            pxr::SdfValueTypeNames->Token);
    SetAttributeDefault(data_, texture_wrap_s_attr, kTokens->repeat);

    pxr::SdfPath texture_wrap_t_attr =
        CreateAttributeSpec(data_, texture_shader_path, kTokens->inputsWrapT,
                            pxr::SdfValueTypeNames->Token);
    SetAttributeDefault(data_, texture_wrap_t_attr, kTokens->repeat);

    pxr::SdfPath texture_rgb_output_attr =
        CreateAttributeSpec(data_, texture_shader_path, kTokens->outputsRgb,
                            pxr::SdfValueTypeNames->Float3);

    return texture_rgb_output_attr;
  }

  void WriteMaterial(mjsMaterial *material, const pxr::SdfPath &parent_path) {
    auto name = GetAvailablePrimName(
        *material->name, pxr::UsdShadeTokens->Material, parent_path);
    pxr::SdfPath material_path =
        CreatePrimSpec(data_, parent_path, name, pxr::UsdShadeTokens->Material);

    // Shader "PreviewSurface"
    pxr::SdfPath preview_surface_shader_path = CreatePrimSpec(
        data_, material_path, kTokens->surface, pxr::UsdShadeTokens->Shader);

    pxr::SdfPath info_id_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, pxr::UsdShadeTokens->infoId,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
    SetAttributeDefault(data_, info_id_attr,
                        pxr::UsdImagingTokens->UsdPreviewSurface);

    pxr::SdfPath surface_output_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, pxr::UsdShadeTokens->outputsSurface,
        pxr::SdfValueTypeNames->Token);

    pxr::SdfPath displacement_output_attr =
        CreateAttributeSpec(data_, preview_surface_shader_path,
                            pxr::UsdShadeTokens->outputsDisplacement,
                            pxr::SdfValueTypeNames->Token);

    pxr::SdfPath diffuse_color_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, kTokens->inputsDiffuseColor,
        pxr::SdfValueTypeNames->Color3f);

    // Find the main texture if specified.
    std::string main_texture_name = (*material->textures)[mjTEXROLE_RGB];
    mjsTexture *main_texture = mjs_asTexture(
        mjs_findElement(spec_, mjOBJ_TEXTURE, main_texture_name.c_str()));
    if (main_texture) {
      // Create the texture shader and connect it to the diffuse color
      // attribute.
      pxr::SdfPath texture_rgb_output_attr =
          AddTextureShader(material_path, main_texture->file->c_str());
      AddAttributeConnection(data_, diffuse_color_attr,
                             texture_rgb_output_attr);
    } else {
      // If no texture is specified, use the rgba diffuse color.
      SetAttributeDefault(data_, diffuse_color_attr,
                          pxr::GfVec3f(material->rgba[0], material->rgba[1],
                                       material->rgba[2]));
    }

    pxr::SdfPath metallic_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, kTokens->inputsMetallic,
        pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, metallic_attr, material->metallic);

    pxr::SdfPath material_surface_output_attr = CreateAttributeSpec(
        data_, material_path, pxr::UsdShadeTokens->outputsSurface,
        pxr::SdfValueTypeNames->Token);

    AddAttributeConnection(data_, material_surface_output_attr,
                           surface_output_attr);

    pxr::SdfPath material_displacement_output_attr = CreateAttributeSpec(
        data_, material_path, pxr::UsdShadeTokens->outputsDisplacement,
        pxr::SdfValueTypeNames->Token);

    AddAttributeConnection(data_, material_displacement_output_attr,
                           displacement_output_attr);
  }

  void WriteMaterials() {
    // Create a scope for the meshes to keep things organized
    pxr::SdfPath scope_path =
        CreatePrimSpec(data_, body_paths_[kWorldIndex], kTokens->materialsScope,
                       pxr::UsdGeomTokens->Scope);

    mjsMaterial *material =
        mjs_asMaterial(mjs_firstElement(spec_, mjOBJ_MATERIAL));
    while (material) {
      WriteMaterial(material, scope_path);
      material = mjs_asMaterial(mjs_nextElement(spec_, material->element));
    }
  }

  pxr::SdfPath WriteMeshGeom(const mjsGeom *geom,
                             const pxr::SdfPath &body_path) {
    std::string mj_name = geom->name->empty() ? *geom->meshname : *geom->name;
    auto name =
        GetAvailablePrimName(mj_name, pxr::UsdGeomTokens->Mesh, body_path);
    pxr::SdfPath subcomponent_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Xform);

    // Reference the mesh asset written in WriteMeshes.
    AddPrimReference(data_, subcomponent_path, mesh_paths_[*geom->meshname]);

    return subcomponent_path;
  }

  pxr::SdfPath WriteSiteGeom(const mjsSite *site,
                             const pxr::SdfPath &body_path) {
    auto name =
        GetAvailablePrimName(*site->name, pxr::UsdGeomTokens->Cube, body_path);

    int site_idx = mjs_getId(site->element);
    const mjtNum *size = &model_->site_size[site_idx * 3];
    pxr::SdfPath site_path;
    switch (site->type) {
      case mjGEOM_BOX:
        site_path = WriteBox(name, size, body_path);
        break;
      case mjGEOM_SPHERE:
        site_path = WriteSphere(name, size, body_path);
        break;
      case mjGEOM_CAPSULE:
        site_path = WriteCapsule(name, size, body_path);
        break;
      case mjGEOM_CYLINDER:
        site_path = WriteCylinder(name, size, body_path);
        break;
      case mjGEOM_ELLIPSOID:
        site_path = WriteEllipsoid(name, size, body_path);
        break;
      default:
        break;
    }

    return site_path;
  }

  pxr::SdfPath WriteBox(const pxr::TfToken &name, const mjtNum *size,
                        const pxr::SdfPath &body_path) {
    pxr::SdfPath box_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Cube);
    // MuJoCo uses half sizes.
    pxr::SdfPath size_attr_path =
        CreateAttributeSpec(data_, box_path, pxr::UsdGeomTokens->size,
                            pxr::SdfValueTypeNames->Float);
    pxr::GfVec3f scale(static_cast<float>(size[0]), static_cast<float>(size[1]),
                       static_cast<float>(size[2]));
    SetAttributeDefault(data_, size_attr_path, 2.0);

    pxr::SdfPath extent_attr_path =
        CreateAttributeSpec(data_, box_path, pxr::UsdGeomTokens->extent,
                            pxr::SdfValueTypeNames->Float3Array);
    SetAttributeDefault(data_, extent_attr_path,
                        pxr::VtArray<pxr::GfVec3f>({
                            pxr::GfVec3f(-size[0], -size[1], -size[2]),
                            pxr::GfVec3f(size[0], size[1], size[2]),
                        }));

    WriteScaleXformOp(box_path, scale);
    WriteXformOpOrder(box_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpScale});
    return box_path;
  }

  pxr::SdfPath WriteBoxGeom(const mjsGeom *geom,
                            const pxr::SdfPath &body_path) {
    auto name =
        GetAvailablePrimName(*geom->name, pxr::UsdGeomTokens->Cube, body_path);

    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WriteBox(name, geom_size, body_path);
  }

  pxr::SdfPath WriteCapsule(const pxr::TfToken name, const mjtNum *size,
                            const pxr::SdfPath &body_path) {
    pxr::SdfPath capsule_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Capsule);

    // MuJoCo uses half sizes.
    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, capsule_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, radius_attr_path, size[0] * 2);

    pxr::SdfPath height_attr_path =
        CreateAttributeSpec(data_, capsule_path, pxr::UsdGeomTokens->height,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, height_attr_path, size[1] * 2);
    return capsule_path;
  }

  pxr::SdfPath WriteCapsuleGeom(const mjsGeom *geom,
                                const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*geom->name, pxr::UsdGeomTokens->Capsule,
                                     body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];

    return WriteCapsule(name, geom_size, body_path);
  }

  pxr::SdfPath WriteCylinder(const pxr::TfToken name, const mjtNum *size,
                             const pxr::SdfPath &body_path) {
    pxr::SdfPath cylinder_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Cylinder);

    // MuJoCo uses half sizes.
    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, cylinder_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, radius_attr_path, size[0] * 2);

    pxr::SdfPath height_attr_path =
        CreateAttributeSpec(data_, cylinder_path, pxr::UsdGeomTokens->height,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, height_attr_path, size[1] * 2);
    return cylinder_path;
  }

  pxr::SdfPath WriteCylinderGeom(const mjsGeom *geom,
                                 const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*geom->name, pxr::UsdGeomTokens->Cylinder,
                                     body_path);

    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WriteCylinder(name, geom_size, body_path);
  }

  pxr::SdfPath WriteEllipsoid(const pxr::TfToken name, const mjtNum *size,
                              const pxr::SdfPath &body_path) {
    pxr::SdfPath ellipsoid_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Sphere);

    pxr::GfVec3f scale = {static_cast<float>(size[0] * 2),
                          static_cast<float>(size[1] * 2),
                          static_cast<float>(size[2] * 2)};

    // MuJoCo uses half sizes.
    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, ellipsoid_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, radius_attr_path, 1.0f);

    WriteScaleXformOp(ellipsoid_path, scale);
    WriteXformOpOrder(ellipsoid_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpScale});
    return ellipsoid_path;
  }

  pxr::SdfPath WriteEllipsoidGeom(const mjsGeom *geom,
                                  const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*geom->name, pxr::UsdGeomTokens->Sphere,
                                     body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];

    return WriteEllipsoid(name, geom_size, body_path);
  }

  pxr::SdfPath WriteSphere(const pxr::TfToken name, const mjtNum *size,
                           const pxr::SdfPath &body_path) {
    pxr::SdfPath sphere_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Sphere);

    // MuJoCo uses half sizes.
    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, sphere_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, radius_attr_path, size[0] * 2);
    return sphere_path;
  }

  pxr::SdfPath WriteSphereGeom(const mjsGeom *geom,
                               const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*geom->name, pxr::UsdGeomTokens->Sphere,
                                     body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WriteSphere(name, geom_size, body_path);
  }

  pxr::SdfPath WritePlane(const pxr::TfToken &name, const mjtNum *size,
                          const pxr::SdfPath &body_path) {
    pxr::SdfPath plane_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Plane);

    // MuJoCo uses half sizes.
    // Note that UsdGeomPlane is infinite for simulation purposes but can have
    // width/length for visualization, same as MuJoCo.
    double width = size[0] * 2.0;
    double length = size[1] * 2.0;

    pxr::SdfPath width_attr_path =
        CreateAttributeSpec(data_, plane_path, pxr::UsdGeomTokens->width,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, width_attr_path, width);

    pxr::SdfPath length_attr_path =
        CreateAttributeSpec(data_, plane_path, pxr::UsdGeomTokens->length,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, length_attr_path, length);

    // MuJoCo plane is always a XY plane with +Z up.
    // UsdGeomPlane is also a XY plane if axis is 'Z', which is default.
    // So no need to set axis attribute explicitly.

    return plane_path;
  }

  pxr::SdfPath WritePlaneGeom(const mjsGeom *geom,
                              const pxr::SdfPath &body_path) {
    auto name =
        GetAvailablePrimName(*geom->name, pxr::UsdGeomTokens->Plane, body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WritePlane(name, geom_size, body_path);
  }

  void WriteSite(mjsSite *site, const mjsBody *body) {
    const int body_id = mjs_getId(body->element);
    const auto &body_path = body_paths_[body_id];
    auto name =
        GetAvailablePrimName(*site->name, pxr::UsdGeomTokens->Xform, body_path);

    // Create a geom primitive and set its purpose to guide so it won't be
    // rendered.
    pxr::SdfPath site_path = WriteSiteGeom(site, body_path);
    SetPrimPurpose(data_, site_path, pxr::UsdGeomTokens->guide);

    int site_id = mjs_getId(site->element);
    auto transform = MujocoPosQuatToTransform(&model_->site_pos[3 * site_id],
                                              &model_->site_quat[4 * site_id]);
    WriteTransformXformOp(site_path, transform);

    PrependToXformOpOrder(
        site_path, pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});
  }

  void WriteGeom(mjsGeom *geom, const mjsBody *body) {
    const int body_id = mjs_getId(body->element);
    const auto &body_path = body_paths_[body_id];

    pxr::SdfPath geom_path;
    int geom_id = mjs_getId(geom->element);
    switch (geom->type) {
      case mjGEOM_PLANE:
        geom_path = WritePlaneGeom(geom, body_path);
        break;
      case mjGEOM_MESH:
        geom_path = WriteMeshGeom(geom, body_path);
        break;
      case mjGEOM_BOX:
        geom_path = WriteBoxGeom(geom, body_path);
        break;
      case mjGEOM_CAPSULE:
        geom_path = WriteCapsuleGeom(geom, body_path);
        break;
      case mjGEOM_CYLINDER:
        geom_path = WriteCylinderGeom(geom, body_path);
        break;
      case mjGEOM_ELLIPSOID:
        geom_path = WriteEllipsoidGeom(geom, body_path);
        break;
      case mjGEOM_SPHERE:
        geom_path = WriteSphereGeom(geom, body_path);
        break;
      default:
        TF_WARN(UnsupportedGeomTypeError, "Unsupported geom type for geom %d",
                geom_id);
        return;
    }

    mjsDefault *spec_default = mjs_getDefault(geom->element);
    pxr::TfToken valid_class_name = GetValidPrimName(*spec_default->name);
    pxr::SdfPath geom_class_path = class_path_.AppendChild(valid_class_name);
    if (!data_->HasSpec(geom_class_path)) {
      pxr::SdfPath class_path =
          CreateClassSpec(data_, class_path_, valid_class_name);
      auto visibility_attr =
          CreateAttributeSpec(data_, class_path, pxr::UsdGeomTokens->visibility,
                              pxr::SdfValueTypeNames->Token);
      SetAttributeDefault(data_, visibility_attr,
                          pxr::UsdGeomTokens->inherited);
    }

    // Bind material if it exists.
    if (!geom->material->empty()) {
      pxr::SdfPath material_path =
          body_paths_[kWorldIndex]
              .AppendChild(kTokens->materialsScope)
              .AppendChild(GetValidPrimName(*geom->material));
      if (data_->HasSpec(material_path)) {
        ApplyApiSchema(data_, geom_path,
                       pxr::UsdShadeTokens->MaterialBindingAPI);
        // Bind the material to this geom.
        CreateRelationshipSpec(data_, geom_path,
                               pxr::UsdShadeTokens->materialBinding,
                               material_path, pxr::SdfVariabilityUniform);
      }
    }

    // If geom rgba is not the default (0.5, 0.5, 0.5, 1), then set the
    // displayColor attribute.
    // No effort is made to properly handle the interaction between geom rgba
    // and the material if both are specified.
    if (geom->rgba[0] != 0.5f || geom->rgba[1] != 0.5f ||
        geom->rgba[2] != 0.5f || geom->rgba[3] != 1.0f) {
      // Set the displayColor attribute.
      pxr::SdfPath display_color_attr = CreateAttributeSpec(
          data_, geom_path, pxr::UsdGeomTokens->primvarsDisplayColor,
          pxr::SdfValueTypeNames->Color3fArray);
      SetAttributeDefault(data_, display_color_attr,
                          pxr::VtArray<pxr::GfVec3f>{
                              {geom->rgba[0], geom->rgba[1], geom->rgba[2]}});
      // Set the displayOpacity attribute, only if the opacity is not 1.
      if (geom->rgba[3] != 1.0f) {
        pxr::SdfPath display_opacity_attr = CreateAttributeSpec(
            data_, geom_path, pxr::UsdGeomTokens->primvarsDisplayOpacity,
            pxr::SdfValueTypeNames->FloatArray);
        SetAttributeDefault(data_, display_opacity_attr,
                            pxr::VtArray<float>{geom->rgba[3]});
      }
    }

    if (body_id == kWorldIndex) {
      SetPrimKind(data_, geom_path, pxr::KindTokens->component);
    }
    // Inherit from class.
    AddPrimInherit(data_, geom_path, geom_class_path);

    auto transform = MujocoPosQuatToTransform(&model_->geom_pos[3 * geom_id],
                                              &model_->geom_quat[4 * geom_id]);
    WriteTransformXformOp(geom_path, transform);

    PrependToXformOpOrder(
        geom_path, pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});
  }

  void WriteSites(mjsBody *body) {
    mjsSite *site = mjs_asSite(mjs_firstChild(body, mjOBJ_SITE, false));
    while (site) {
      WriteSite(site, body);
      site = mjs_asSite(mjs_nextChild(body, site->element, false));
    }
  }

  void WriteGeoms(mjsBody *body) {
    mjsGeom *geom = mjs_asGeom(mjs_firstChild(body, mjOBJ_GEOM, false));
    while (geom) {
      WriteGeom(geom, body);
      geom = mjs_asGeom(mjs_nextChild(body, geom->element, false));
    }
  }

  void WriteCamera(mjsCamera *spec_cam, const mjsBody *body) {
    const auto &body_path = body_paths_[mjs_getId(body->element)];
    auto name = GetAvailablePrimName(*spec_cam->name,
                                     pxr::UsdGeomTokens->Camera, body_path);
    // Create a root Xform for the world body with the model name if it exists
    // otherwise called 'World'.
    pxr::SdfPath camera_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Camera);

    int cam_id = mjs_getId(spec_cam->element);
    auto transform = MujocoPosQuatToTransform(&model_->cam_pos[3 * cam_id],
                                              &model_->cam_quat[4 * cam_id]);
    WriteTransformXformOp(camera_path, transform);
    WriteXformOpOrder(camera_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});

    // If the camera intrinsics are specified, then it is important that we
    // reproduce the code in mujoco/src/engine/engine_vis_visualize.c
    const float *cam_sensorsize = &model_->cam_sensorsize[cam_id * 2];
    bool use_intrinsic = cam_sensorsize[1] > 0.0f;
    float znear = spec_->visual.map.znear * model_->stat.extent * 100;
    float zfar = spec_->visual.map.zfar * model_->stat.extent * 100;
    mjtNum fovy = model_->cam_fovy[cam_id];
    const float *cam_intrinsic = &model_->cam_intrinsic[cam_id * 4];

    const float aspect_ratio =
        use_intrinsic ? cam_sensorsize[0] / cam_sensorsize[1] : 4.0f / 3;
    float vertical_apperture =
        2 * znear *
        (use_intrinsic ? 1.0f / cam_intrinsic[1] *
                             (cam_sensorsize[1] / 2.f - cam_intrinsic[3])
                       : mju_tan((fovy / 2) * (M_PI / 180.0)));
    float horizontal_aperture =
        use_intrinsic ? 2 * znear / cam_intrinsic[0] *
                            (cam_sensorsize[0] / 2.f - cam_intrinsic[2])
                      : vertical_apperture * aspect_ratio;

    pxr::SdfPath clipping_range_attr_path = CreateAttributeSpec(
        data_, camera_path, pxr::UsdGeomTokens->clippingRange,
        pxr::SdfValueTypeNames->Float2);
    SetAttributeDefault(data_, clipping_range_attr_path,
                        pxr::GfVec2f(znear, zfar));
    pxr::SdfPath focal_length_attr_path =
        CreateAttributeSpec(data_, camera_path, pxr::UsdGeomTokens->focalLength,
                            pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, focal_length_attr_path, znear);

    pxr::SdfPath vertical_aperture_attr_path = CreateAttributeSpec(
        data_, camera_path, pxr::UsdGeomTokens->verticalAperture,
        pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, vertical_aperture_attr_path, vertical_apperture);

    pxr::SdfPath horizontal_aperture_attr_path = CreateAttributeSpec(
        data_, camera_path, pxr::UsdGeomTokens->horizontalAperture,
        pxr::SdfValueTypeNames->Float);
    SetAttributeDefault(data_, horizontal_aperture_attr_path,
                        horizontal_aperture);
  }

  void WriteCameras(mjsBody *body) {
    mjsCamera *cam = mjs_asCamera(mjs_firstChild(body, mjOBJ_CAMERA, false));
    while (cam) {
      WriteCamera(cam, body);
      cam = mjs_asCamera(mjs_nextChild(body, cam->element, false));
    }
  }

  void WriteLight(mjsLight *light, const mjsBody *body) {
    const auto &body_path = body_paths_[mjs_getId(body->element)];
    auto name = GetAvailablePrimName(*light->name, kTokens->light, body_path);
    // Create a root Xform for the world body with the model name if it exists
    // otherwise called 'World'.
    pxr::SdfPath light_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdLuxTokens->SphereLight);

    int light_id = mjs_getId(light->element);
    auto transform = MujocoPosQuatToTransform(&model_->light_pos[3 * light_id],
                                              &model_->light_dir[4 * light_id]);
    WriteTransformXformOp(light_path, transform);
    WriteXformOpOrder(light_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});
  }

  void WriteLights(mjsBody *body) {
    mjsLight *light = mjs_asLight(mjs_firstChild(body, mjOBJ_LIGHT, false));
    while (light) {
      WriteLight(light, body);
      light = mjs_asLight(mjs_nextChild(body, light->element, false));
    }
  }

  void WriteBody(mjsBody *body, bool write_physics) {
    int body_id = mjs_getId(body->element);
    pxr::SdfPath parent_path =
        CreateParentIfNotExists(body, body_paths_[kWorldIndex], data_);
    pxr::TfToken body_name = GetValidPrimName(*body->name);

    // Create Xform prim for body.
    pxr::SdfPath body_path = CreatePrimSpec(data_, parent_path, body_name,
                                            pxr::UsdGeomTokens->Xform);
    // The parent_path will be a component which makes the actual articulated
    // bodies subcomponents.
    SetPrimKind(data_, body_path, pxr::KindTokens->subcomponent);

    // Apply the PhysicsRigidBodyAPI schema if we are writing physics.
    if (write_physics) {
      ApplyApiSchema(data_, body_path,
                     pxr::UsdPhysicsTokens->PhysicsRigidBodyAPI);
    }

    // Create classes if necessary
    mjsDefault *spec_default = mjs_getDefault(body->element);

    pxr::TfToken body_class_name = GetValidPrimName(*spec_default->name);
    pxr::SdfPath body_class_path = class_path_.AppendChild(body_class_name);
    if (!data_->HasSpec(body_class_path)) {
      CreateClassSpec(data_, class_path_, body_class_name);
    }

    // Create XformOp attribute for body transform.
    pxr::SdfPath xform_op_path =
        CreateAttributeSpec(data_, body_path, kTokens->xformOpTransform,
                            pxr::SdfValueTypeNames->Matrix4d);

    // Make sure to account for the parent since UsdPhysics doesn't support
    // nested bodies!
    auto parent_xform = body_xforms_[model_->body_parentid[body_id]];
    // mjModel will have all frames already accounted for so no need to worry
    // about them here.
    body_xforms_[body_id] =
        MujocoPosQuatToTransform(&model_->body_pos[body_id * 3],
                                 &model_->body_quat[body_id * 4]) *
        parent_xform;
    SetAttributeDefault(data_, xform_op_path, body_xforms_[body_id]);

    // Create XformOpOrder attribute for body transform order.
    // For us this is simply the transform we authored above.
    WriteXformOpOrder(body_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});

    pxr::VtDictionary customData;
    customData[kTokens->body_name] = *body->name;
    SetPrimMetadata(data_, body_path, pxr::SdfFieldKeys->CustomData,
                    customData);

    body_paths_[body_id] = body_path;
  }

  void WriteBodies(bool write_physics) {
    mjsBody *body = mjs_asBody(mjs_firstElement(spec_, mjOBJ_BODY));
    while (body) {
      // Only write a rigidbody if we are not the world body.
      // We fall through since the world body might have static
      // geom children.
      if (mjs_getId(body->element) != kWorldIndex) {
        WriteBody(body, write_physics);
      }
      WriteSites(body);
      WriteGeoms(body);
      WriteCameras(body);
      WriteLights(body);
      body = mjs_asBody(mjs_nextElement(spec_, body->element));
    }
  }

  pxr::SdfPath WriteWorldBody(const size_t body_index) {
    // Create a root Xform for the world body with the model name if it exists
    // otherwise called 'World'.
    auto name = GetAvailablePrimName(*spec_->modelname, kTokens->world,
                                     pxr::SdfPath::AbsoluteRootPath());
    pxr::SdfPath world_group_path =
        CreatePrimSpec(data_, pxr::SdfPath::AbsoluteRootPath(), name,
                       pxr::UsdGeomTokens->Xform);
    SetPrimKind(data_, world_group_path, pxr::KindTokens->group);
    return world_group_path;
  }
};

namespace mujoco {
namespace usd {

bool WriteSpecToData(mjSpec *spec, pxr::SdfAbstractDataRefPtr &data,
                     bool write_physics) {
  // Create pseudo root first.
  data->CreateSpec(pxr::SdfPath::AbsoluteRootPath(),
                   pxr::SdfSpecTypePseudoRoot);

  mjModel *model = mj_compile(spec, nullptr);
  if (model == nullptr) {
    TF_ERROR(MujocoCompilationError, "%s", mjs_getError(spec));
    return false;
  }

  ModelWriter(spec, model, data).Write(write_physics);

  return true;
}

}  // namespace usd
}  // namespace mujoco
