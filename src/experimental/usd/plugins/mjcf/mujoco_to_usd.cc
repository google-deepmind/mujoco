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
#include <unordered_set>
#include <utility>
#include <vector>

#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "mjcf/utils.h"
#include <pxr/base/arch/attributes.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatf.h>
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
#include <pxr/base/vt/types.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/kind/registry.h>
#include <pxr/usd/sdf/abstractData.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdLux/tokens.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
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
                         ((body, "Body"))
                         ((body_name, "mujoco:body_name"))
                         ((geom, "Geom"))
                         ((light, "Light"))
                         ((meshScope, "MeshSources"))
                         ((materialsScope, "Materials"))
                         ((physicsMaterialsScope, "PhysicsMaterials"))
                         ((previewSurface, "PreviewSurface"))
                         ((keyframesScope, "Keyframes"))
                         ((actuatorsScope, "Actuators"))
                         ((keyframe, "Keyframe"))
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
                         ((inputsEmissiveColor, "inputs:emissiveColor"))
                         ((outputsRgb, "outputs:rgb"))
                         ((outputsR, "outputs:r"))
                         ((outputsG, "outputs:g"))
                         ((outputsB, "outputs:b"))
                         ((inputsMetallic, "inputs:metallic"))
                         ((inputsOcclusion, "inputs:occlusion"))
                         ((inputsRoughness, "inputs:roughness"))
                         (repeat)
                         ((sourceMesh, pxr::UsdGeomTokens->Mesh))
                         ((inputsNormal, "inputs:normal"))
                         ((joint, "Joint"))
                        );

// Using to satisfy TF_REGISTRY_FUNCTION macro below and avoid operating in PXR_NS.
using pxr::TfEnum;
using pxr::Tf_RegistryStaticInit;
using pxr::Tf_RegistryInit;
using pxr::TfEnum;
template <typename T>
using Arch_PerLibInit = pxr::Arch_PerLibInit<T>;
#if defined(ARCH_OS_DARWIN)
using Arch_ConstructorEntry = pxr::Arch_ConstructorEntry;
#endif
enum ErrorCodes { UnsupportedActuatorTypeError, UnsupportedGeomTypeError, MujocoCompilationError };

TF_REGISTRY_FUNCTION(pxr::TfEnum) {
  TF_ADD_ENUM_NAME(UnsupportedGeomTypeError, "UsdGeom type is unsupported.")
  TF_ADD_ENUM_NAME(MujocoCompilationError, "Mujoco spec failed to compile.")
}

// Usings to satisfy TF_ERROR macro.
using pxr::TfCallContext;
using pxr::Tf_PostErrorHelper;
// clang-format on

using pxr::MjcPhysicsTokens;

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
using mujoco::usd::SetAttributeTimeSample;
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
    site_paths_ = std::vector<pxr::SdfPath>(model->nsite);
    joint_paths_ = std::vector<pxr::SdfPath>(model->njnt);
  }
  ~ModelWriter() { mj_deleteModel(model_); }

  void Write() {
    // Create top level class holder.
    class_path_ = CreateClassSpec(data_, pxr::SdfPath::AbsoluteRootPath(),
                                  pxr::TfToken("__class__"));

    // Create the world body.
    body_paths_[kWorldIndex] = WriteWorldBody(kWorldIndex);

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

    WritePhysicsScene();

    // Author mesh scope + mesh prims to be referenced.
    WriteMeshes();
    WriteMaterials();
    WriteBodies();
    WriteActuators();
    WriteKeyframes();
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
  // Mapping from Mujoco site id to SdfPath.
  std::vector<pxr::SdfPath> site_paths_;
  // Mapping from Mujoco joint id to SdfPath.
  std::vector<pxr::SdfPath> joint_paths_;
  // Mapping from mesh names to Mesh prim path.
  std::unordered_map<std::string, pxr::SdfPath> mesh_paths_;
  // Set of body ids that have had the articulation root API applied.
  std::unordered_set<int> articulation_roots_;

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

  template <typename T>
  void WriteUniformAttribute(const pxr::SdfPath &prim_path,
                             const pxr::SdfValueTypeName &value_type_name,
                             const pxr::TfToken &token, const T &value) {
    pxr::SdfPath attr_path = CreateAttributeSpec(
        data_, prim_path, token, value_type_name, pxr::SdfVariabilityUniform);
    SetAttributeDefault(data_, attr_path, value);
  }

  template <typename T>
  void WriteColorAndOpacityAttributes(const pxr::SdfPath &prim_path,
                                      const T &element) {
    // If rgba is not the default (0.5, 0.5, 0.5, 1), then set the
    // displayColor attribute.
    // No effort is made to properly handle the interaction between rgba
    // and the material if both are specified.
    if (element->rgba[0] != 0.5f || element->rgba[1] != 0.5f ||
        element->rgba[2] != 0.5f || element->rgba[3] != 1.0f) {
      // Set the displayColor attribute.
      pxr::SdfPath display_color_attr = CreateAttributeSpec(
          data_, prim_path, pxr::UsdGeomTokens->primvarsDisplayColor,
          pxr::SdfValueTypeNames->Color3fArray);
      SetAttributeDefault(
          data_, display_color_attr,
          pxr::VtArray<pxr::GfVec3f>{
              {element->rgba[0], element->rgba[1], element->rgba[2]}});
      // Set the displayOpacity attribute, only if the opacity is not 1.
      if (element->rgba[3] != 1.0f) {
        pxr::SdfPath display_opacity_attr = CreateAttributeSpec(
            data_, prim_path, pxr::UsdGeomTokens->primvarsDisplayOpacity,
            pxr::SdfValueTypeNames->FloatArray);
        SetAttributeDefault(data_, display_opacity_attr,
                            pxr::VtArray<float>{element->rgba[3]});
      }
    }
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
    auto name = GetAvailablePrimName(*mjs_getName(mesh->element),
                                     pxr::UsdGeomTokens->Mesh, parent_path);
    pxr::SdfPath subcomponent_path =
        CreatePrimSpec(data_, parent_path, name, pxr::UsdGeomTokens->Xform);
    pxr::SdfPath mesh_path =
        CreatePrimSpec(data_, subcomponent_path, kTokens->sourceMesh,
                       pxr::UsdGeomTokens->Mesh);
    mesh_paths_[*mjs_getName(mesh->element)] = subcomponent_path;

    ApplyApiSchema(data_, mesh_path, MjcPhysicsTokens->MjcMeshCollisionAPI);

    pxr::TfToken inertia = MjcPhysicsTokens->legacy;
    if (mesh->inertia == mjtMeshInertia::mjMESH_INERTIA_EXACT) {
      inertia = MjcPhysicsTokens->exact;
    } else if (mesh->inertia == mjtMeshInertia::mjMESH_INERTIA_CONVEX) {
      inertia = MjcPhysicsTokens->convex;
    } else if (mesh->inertia == mjtMeshInertia::mjMESH_INERTIA_SHELL) {
      inertia = MjcPhysicsTokens->shell;
    }

    WriteUniformAttribute(mesh_path, pxr::SdfValueTypeNames->Token,
                          MjcPhysicsTokens->mjcInertia, inertia);

    WriteUniformAttribute(mesh_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcMaxhullvert, mesh->maxhullvert);

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

  void WritePhysicsScene() {
    pxr::SdfPath physics_scene_path = CreatePrimSpec(
        data_, body_paths_[kWorldIndex], pxr::UsdPhysicsTokens->PhysicsScene,
        pxr::UsdPhysicsTokens->PhysicsScene);

    ApplyApiSchema(data_, physics_scene_path, MjcPhysicsTokens->MjcSceneAPI);

    const std::vector<std::pair<pxr::TfToken, double>>
        option_double_attributes = {
            {MjcPhysicsTokens->mjcOptionTimestep, spec_->option.timestep},
            {MjcPhysicsTokens->mjcOptionTolerance, spec_->option.tolerance},
            {MjcPhysicsTokens->mjcOptionLs_tolerance,
             spec_->option.ls_tolerance},
            {MjcPhysicsTokens->mjcOptionNoslip_tolerance,
             spec_->option.noslip_tolerance},
            {MjcPhysicsTokens->mjcOptionCcd_tolerance,
             spec_->option.ccd_tolerance},
            {MjcPhysicsTokens->mjcOptionApirate, spec_->option.apirate},
            {MjcPhysicsTokens->mjcOptionImpratio, spec_->option.impratio},
            {MjcPhysicsTokens->mjcOptionDensity, spec_->option.density},
            {MjcPhysicsTokens->mjcOptionViscosity, spec_->option.viscosity},
            {MjcPhysicsTokens->mjcOptionO_margin, spec_->option.o_margin},
        };
    for (const auto &[token, value] : option_double_attributes) {
      WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Double,
                            token, value);
    }

    const std::vector<std::pair<pxr::TfToken, int>> option_int_attributes = {
        {MjcPhysicsTokens->mjcOptionIterations, spec_->option.iterations},
        {MjcPhysicsTokens->mjcOptionLs_iterations, spec_->option.ls_iterations},
        {MjcPhysicsTokens->mjcOptionNoslip_iterations,
         spec_->option.noslip_iterations},
        {MjcPhysicsTokens->mjcOptionCcd_iterations,
         spec_->option.ccd_iterations},
        {MjcPhysicsTokens->mjcOptionSdf_iterations,
         spec_->option.sdf_iterations},
        {MjcPhysicsTokens->mjcOptionSdf_initpoints,
         spec_->option.sdf_initpoints},
    };
    for (const auto &[token, value] : option_int_attributes) {
      WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Int,
                            token, value);
    }

    pxr::SdfPath cone_attr = CreateAttributeSpec(
        data_, physics_scene_path, MjcPhysicsTokens->mjcOptionCone,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);

    switch (spec_->option.cone) {
      case mjCONE_PYRAMIDAL:
        SetAttributeDefault(data_, cone_attr, MjcPhysicsTokens->pyramidal);
        break;
      case mjCONE_ELLIPTIC:
        SetAttributeDefault(data_, cone_attr, MjcPhysicsTokens->elliptic);
        break;
      default:
        break;
    }

    pxr::SdfPath jacobian_attr = CreateAttributeSpec(
        data_, physics_scene_path, MjcPhysicsTokens->mjcOptionJacobian,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);

    switch (spec_->option.jacobian) {
      case mjJAC_AUTO:
        SetAttributeDefault(data_, jacobian_attr, MjcPhysicsTokens->auto_);
        break;
      case mjJAC_DENSE:
        SetAttributeDefault(data_, jacobian_attr, MjcPhysicsTokens->dense);
        break;
      case mjJAC_SPARSE:
        SetAttributeDefault(data_, jacobian_attr, MjcPhysicsTokens->sparse);
        break;
      default:
        break;
    }

    pxr::SdfPath solver_attr = CreateAttributeSpec(
        data_, physics_scene_path, MjcPhysicsTokens->mjcOptionSolver,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
    switch (spec_->option.solver) {
      case mjSOL_NEWTON:
        SetAttributeDefault(data_, solver_attr, MjcPhysicsTokens->newton);
        break;
      case mjSOL_PGS:
        SetAttributeDefault(data_, solver_attr, MjcPhysicsTokens->pgs);
        break;
      case mjSOL_CG:
        SetAttributeDefault(data_, solver_attr, MjcPhysicsTokens->cg);
        break;
      default:
        break;
    }

    pxr::GfVec3f gravity(spec_->option.gravity[0], spec_->option.gravity[1],
                         spec_->option.gravity[2]);
    // Normalize will normalize gravity in place and return the magnitude before
    // normalization.
    float gravity_magnitude = gravity.Normalize();

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Float,
                          pxr::UsdPhysicsTokens->physicsGravityMagnitude,
                          gravity_magnitude);
    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Vector3f,
                          pxr::UsdPhysicsTokens->physicsGravityDirection,
                          gravity);

    pxr::GfVec3d wind(spec_->option.wind[0], spec_->option.wind[1],
                      spec_->option.wind[2]);
    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Double3,
                          MjcPhysicsTokens->mjcOptionWind, wind);

    pxr::GfVec3d magnetic(spec_->option.magnetic[0], spec_->option.magnetic[1],
                          spec_->option.magnetic[2]);
    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Double3,
                          MjcPhysicsTokens->mjcOptionMagnetic, magnetic);

    pxr::VtArray<double> o_solref(spec_->option.o_solref,
                                  spec_->option.o_solref + mjNREF);
    WriteUniformAttribute(physics_scene_path,
                          pxr::SdfValueTypeNames->DoubleArray,
                          MjcPhysicsTokens->mjcOptionO_solref, o_solref);

    pxr::VtArray<double> o_solimp(spec_->option.o_solimp,
                                  spec_->option.o_solimp + mjNIMP);
    WriteUniformAttribute(physics_scene_path,
                          pxr::SdfValueTypeNames->DoubleArray,
                          MjcPhysicsTokens->mjcOptionO_solimp, o_solimp);

    pxr::VtArray<double> o_friction(spec_->option.o_friction,
                                    spec_->option.o_friction + 5);
    WriteUniformAttribute(physics_scene_path,
                          pxr::SdfValueTypeNames->DoubleArray,
                          MjcPhysicsTokens->mjcOptionO_friction, o_friction);

    pxr::SdfPath integrator_attr = CreateAttributeSpec(
        data_, physics_scene_path, MjcPhysicsTokens->mjcOptionIntegrator,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
    switch (spec_->option.integrator) {
      case mjINT_EULER:
        SetAttributeDefault(data_, integrator_attr, MjcPhysicsTokens->euler);
        break;
      case mjINT_RK4:
        SetAttributeDefault(data_, integrator_attr, MjcPhysicsTokens->rk4);
        break;
      default:
        break;
    }

    auto create_flag_attr = [&](pxr::TfToken token, int flag, bool enable) {
      int flags =
          enable ? spec_->option.enableflags : spec_->option.disableflags;
      bool value = enable ? (flags & flag) : !(flags & flag);
      WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                            token, value);
    };

    const std::vector<std::pair<pxr::TfToken, int>> enable_flags = {
        {MjcPhysicsTokens->mjcFlagMulticcd, mjENBL_MULTICCD},
        {MjcPhysicsTokens->mjcFlagFwdinv, mjENBL_FWDINV},
        {MjcPhysicsTokens->mjcFlagEnergy, mjENBL_ENERGY},
        {MjcPhysicsTokens->mjcFlagOverride, mjENBL_OVERRIDE},
        {MjcPhysicsTokens->mjcFlagInvdiscrete, mjENBL_INVDISCRETE}};
    for (const auto &[token, flag] : enable_flags) {
      create_flag_attr(token, flag, true);
    }

    const std::vector<std::pair<pxr::TfToken, int>> disable_flags = {
        {MjcPhysicsTokens->mjcFlagConstraint, mjDSBL_CONSTRAINT},
        {MjcPhysicsTokens->mjcFlagEquality, mjDSBL_EQUALITY},
        {MjcPhysicsTokens->mjcFlagFrictionloss, mjDSBL_FRICTIONLOSS},
        {MjcPhysicsTokens->mjcFlagLimit, mjDSBL_LIMIT},
        {MjcPhysicsTokens->mjcFlagContact, mjDSBL_CONTACT},
        {MjcPhysicsTokens->mjcFlagSpring, mjDSBL_SPRING},
        {MjcPhysicsTokens->mjcFlagDamper, mjDSBL_DAMPER},
        {MjcPhysicsTokens->mjcFlagGravity, mjDSBL_GRAVITY},
        {MjcPhysicsTokens->mjcFlagClampctrl, mjDSBL_CLAMPCTRL},
        {MjcPhysicsTokens->mjcFlagWarmstart, mjDSBL_WARMSTART},
        {MjcPhysicsTokens->mjcFlagFilterparent, mjDSBL_FILTERPARENT},
        {MjcPhysicsTokens->mjcFlagActuation, mjDSBL_ACTUATION},
        {MjcPhysicsTokens->mjcFlagRefsafe, mjDSBL_REFSAFE},
        {MjcPhysicsTokens->mjcFlagSensor, mjDSBL_SENSOR},
        {MjcPhysicsTokens->mjcFlagMidphase, mjDSBL_MIDPHASE},
        {MjcPhysicsTokens->mjcFlagEulerdamp, mjDSBL_EULERDAMP},
        {MjcPhysicsTokens->mjcFlagAutoreset, mjDSBL_AUTORESET},
        {MjcPhysicsTokens->mjcFlagNativeccd, mjDSBL_NATIVECCD},
        {MjcPhysicsTokens->mjcFlagIsland, mjDSBL_ISLAND}};
    for (const auto &[token, flag] : disable_flags) {
      create_flag_attr(token, flag, false);
    }

    // Compiler attributes
    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerAutoLimits,
                          (bool)spec_->compiler.autolimits);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Double,
                          MjcPhysicsTokens->mjcCompilerBoundMass,
                          spec_->compiler.boundmass);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Double,
                          MjcPhysicsTokens->mjcCompilerBoundInertia,
                          spec_->compiler.boundinertia);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Double,
                          MjcPhysicsTokens->mjcCompilerSetTotalMass,
                          spec_->compiler.settotalmass);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerUseThread,
                          (bool)spec_->compiler.usethread);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerBalanceInertia,
                          (bool)spec_->compiler.balanceinertia);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Token,
                          MjcPhysicsTokens->mjcCompilerAngle,
                          spec_->compiler.degree ? MjcPhysicsTokens->degree
                                                 : MjcPhysicsTokens->radian);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerFitAABB,
                          (bool)spec_->compiler.fitaabb);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerFuseStatic,
                          (bool)spec_->compiler.fusestatic);

    pxr::TfToken inertiafromgeom_token = MjcPhysicsTokens->auto_;
    if (spec_->compiler.inertiafromgeom ==
        mjINERTIAFROMGEOM_TRUE) {  // mjINERTIA_TRUE
      inertiafromgeom_token = MjcPhysicsTokens->true_;
    } else if (spec_->compiler.inertiafromgeom ==
               mjINERTIAFROMGEOM_FALSE) {  // mjINERTIA_FALSE
      inertiafromgeom_token = MjcPhysicsTokens->false_;
    }
    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Token,
                          MjcPhysicsTokens->mjcCompilerInertiaFromGeom,
                          inertiafromgeom_token);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerAlignFree,
                          (bool)spec_->compiler.alignfree);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcCompilerInertiaGroupRangeMin,
                          spec_->compiler.inertiagrouprange[0]);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcCompilerInertiaGroupRangeMax,
                          spec_->compiler.inertiagrouprange[1]);

    WriteUniformAttribute(physics_scene_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcCompilerSaveInertial,
                          (bool)spec_->compiler.saveinertial);
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

  pxr::SdfPath AddUVTextureShader(const pxr::SdfPath &material_path,
                                  const pxr::TfToken &name) {
    pxr::SdfPath uvmap_shader_path =
        CreatePrimSpec(data_, material_path, name, pxr::UsdShadeTokens->Shader);

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

    return uvmap_st_output_attr;
  }

  std::vector<pxr::SdfPath> AddTextureShader(
      const pxr::SdfPath &material_path, const char *texture_file,
      const pxr::TfToken &name, const pxr::SdfPath &uvmap_st_output_attr,
      const std::vector<pxr::TfToken> &output_channels) {
    pxr::SdfPath texture_shader_path =
        CreatePrimSpec(data_, material_path, name, pxr::UsdShadeTokens->Shader);
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

    std::vector<pxr::SdfPath> texture_output_attrs;
    for (const auto &output_channel : output_channels) {
      pxr::SdfValueTypeName value_type;
      if (output_channel == kTokens->outputsRgb) {
        value_type = pxr::SdfValueTypeNames->Float3;
      } else {
        // Assume the other specified channels are outputR, outputG, outputB.
        value_type = pxr::SdfValueTypeNames->Float;
      }
      texture_output_attrs.push_back(CreateAttributeSpec(
          data_, texture_shader_path, output_channel, value_type));
    }
    return texture_output_attrs;
  }

  pxr::SdfPath WritePhysicsMaterial(mjsGeom *geom) {
    pxr::SdfPath scope_path =
        body_paths_[kWorldIndex].AppendChild(kTokens->physicsMaterialsScope);

    if (!data_->HasSpec(scope_path)) {
      CreatePrimSpec(data_, body_paths_[kWorldIndex],
                     kTokens->physicsMaterialsScope, pxr::UsdGeomTokens->Scope);
    }

    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdShadeTokens->Material, scope_path);
    pxr::SdfPath material_path =
        CreatePrimSpec(data_, scope_path, name, pxr::UsdShadeTokens->Material);

    ApplyApiSchema(data_, material_path,
                   pxr::UsdPhysicsTokens->PhysicsMaterialAPI);
    ApplyApiSchema(data_, material_path, MjcPhysicsTokens->MjcMaterialAPI);

    mjsGeom *geom_default = mjs_getDefault(geom->element)->geom;
    if (geom->friction[0] != geom_default->friction[0]) {
      // Since MuJoCo has no concept of static friction, only write dynamic
      // friction to remain truthful to how MuJoCo perceives the data.
      WriteUniformAttribute(material_path, pxr::SdfValueTypeNames->Float,
                            pxr::UsdPhysicsTokens->physicsDynamicFriction,
                            (float)geom->friction[0]);
    }
    if (geom->friction[1] != geom_default->friction[1]) {
      WriteUniformAttribute(material_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcTorsionalfriction,
                            geom->friction[1]);
    }
    if (geom->friction[2] != geom_default->friction[2]) {
      WriteUniformAttribute(material_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcRollingfriction,
                            geom->friction[2]);
    }

    return material_path;
  }

  void WriteMaterial(mjsMaterial *material, const pxr::SdfPath &parent_path) {
    // Create a Material prim.
    auto name =
        GetAvailablePrimName(*mjs_getName(material->element),
                             pxr::UsdShadeTokens->Material, parent_path);
    pxr::SdfPath material_path =
        CreatePrimSpec(data_, parent_path, name, pxr::UsdShadeTokens->Material);

    // Create a Shader prim "PreviewSurface" under the Material prim.
    pxr::SdfPath preview_surface_shader_path =
        CreatePrimSpec(data_, material_path, kTokens->previewSurface,
                       pxr::UsdShadeTokens->Shader);

    // Set the Shader'sinfoId attribute to UsdPreviewSurface, a standard surface
    // shader.
    pxr::SdfPath info_id_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, pxr::UsdShadeTokens->infoId,
        pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
    SetAttributeDefault(data_, info_id_attr,
                        pxr::UsdImagingTokens->UsdPreviewSurface);

    // Connect material's surface output to the preview surface's surface
    // output.
    pxr::SdfPath surface_output_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, pxr::UsdShadeTokens->outputsSurface,
        pxr::SdfValueTypeNames->Token);
    pxr::SdfPath material_surface_output_attr = CreateAttributeSpec(
        data_, material_path, pxr::UsdShadeTokens->outputsSurface,
        pxr::SdfValueTypeNames->Token);
    AddAttributeConnection(data_, material_surface_output_attr,
                           surface_output_attr);

    // Connect material's displacement output to the preview surface's
    // displacement output.
    pxr::SdfPath displacement_output_attr =
        CreateAttributeSpec(data_, preview_surface_shader_path,
                            pxr::UsdShadeTokens->outputsDisplacement,
                            pxr::SdfValueTypeNames->Token);
    pxr::SdfPath material_displacement_output_attr = CreateAttributeSpec(
        data_, material_path, pxr::UsdShadeTokens->outputsDisplacement,
        pxr::SdfValueTypeNames->Token);
    AddAttributeConnection(data_, material_displacement_output_attr,
                           displacement_output_attr);

    // Add an st (uv) Shader, a prim var reader for the UV coordinates.
    const pxr::SdfPath &uvmap_st_output_attr =
        AddUVTextureShader(material_path, pxr::TfToken("uvmap"));
    const mjStringVec &textures = *(material->textures);

    // Set the values of metallic, roughness and occlusion. These can come from
    // an ORM packed texture, as individual textures, or as a values defined in
    // mjsMaterial_ (with the exception of occlusion).
    pxr::SdfPath metallic_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, kTokens->inputsMetallic,
        pxr::SdfValueTypeNames->Float);
    pxr::SdfPath roughness_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, kTokens->inputsRoughness,
        pxr::SdfValueTypeNames->Float);
    // Find the occlusion, roughness, and metallic textures.
    if (mjTEXROLE_ORM < textures.size()) {
      std::string orm_texture_name = textures[mjTEXROLE_ORM];
      mjsTexture *orm_texture = mjs_asTexture(
          mjs_findElement(spec_, mjOBJ_TEXTURE, orm_texture_name.c_str()));
      std::string occlusion_texture_name = textures[mjTEXROLE_OCCLUSION];
      mjsTexture *occlusion_texture = mjs_asTexture(mjs_findElement(
          spec_, mjOBJ_TEXTURE, occlusion_texture_name.c_str()));
      std::string roughness_texture_name = textures[mjTEXROLE_ROUGHNESS];
      mjsTexture *roughness_texture = mjs_asTexture(mjs_findElement(
          spec_, mjOBJ_TEXTURE, roughness_texture_name.c_str()));
      std::string metallic_texture_name = textures[mjTEXROLE_METALLIC];
      mjsTexture *metallic_texture = mjs_asTexture(
          mjs_findElement(spec_, mjOBJ_TEXTURE, metallic_texture_name.c_str()));
      if (orm_texture) {
        // Create the ORM shader and connect its output to the preview
        // surface ORM attrs.
        const std::vector<pxr::SdfPath> orm_output_attrs = AddTextureShader(
            material_path, orm_texture->file->c_str(),
            pxr::TfToken("orm_packed"), uvmap_st_output_attr,
            {kTokens->outputsR, kTokens->outputsG, kTokens->outputsB});
        if (orm_output_attrs.size() == 3) {
          pxr::SdfPath occlusion_attr = CreateAttributeSpec(
              data_, preview_surface_shader_path, kTokens->inputsOcclusion,
              pxr::SdfValueTypeNames->Float);
          AddAttributeConnection(data_, occlusion_attr, orm_output_attrs[0]);
          AddAttributeConnection(data_, roughness_attr, orm_output_attrs[1]);
          AddAttributeConnection(data_, metallic_attr, orm_output_attrs[2]);
        }
      } else {
        if (metallic_texture) {
          const std::vector<pxr::SdfPath> metallic_output_attrs =
              AddTextureShader(material_path, metallic_texture->file->c_str(),
                               pxr::TfToken("metallic"), uvmap_st_output_attr,
                               {kTokens->outputsRgb});
          if (metallic_output_attrs.size() == 1) {
            AddAttributeConnection(data_, metallic_attr,
                                   metallic_output_attrs[0]);
          }
        } else {
          SetAttributeDefault(data_, metallic_attr, material->metallic);
        }
        if (roughness_texture) {
          const std::vector<pxr::SdfPath> roughness_output_attrs =
              AddTextureShader(material_path, roughness_texture->file->c_str(),
                               pxr::TfToken("roughness"), uvmap_st_output_attr,
                               {kTokens->outputsRgb});
          if (roughness_output_attrs.size() == 1) {
            AddAttributeConnection(data_, roughness_attr,
                                   roughness_output_attrs[0]);
          }
        } else {
          SetAttributeDefault(data_, roughness_attr, material->roughness);
        }
        if (occlusion_texture) {
          pxr::SdfPath occlusion_attr = CreateAttributeSpec(
              data_, preview_surface_shader_path, kTokens->inputsOcclusion,
              pxr::SdfValueTypeNames->Float);
          const std::vector<pxr::SdfPath> occlusion_output_attrs =
              AddTextureShader(material_path, occlusion_texture->file->c_str(),
                               pxr::TfToken("occlusion"), uvmap_st_output_attr,
                               {kTokens->outputsRgb});
          if (occlusion_output_attrs.size() == 1) {
            AddAttributeConnection(data_, occlusion_attr,
                                   occlusion_output_attrs[0]);
          }
        }
      }
    }
    // Find the normal texture if specified.
    if (mjTEXROLE_NORMAL < textures.size()) {
      std::string normal_texture_name = textures[mjTEXROLE_NORMAL];
      mjsTexture *normal_texture = mjs_asTexture(
          mjs_findElement(spec_, mjOBJ_TEXTURE, normal_texture_name.c_str()));
      if (normal_texture) {
        pxr::SdfPath normal_attr = CreateAttributeSpec(
            data_, preview_surface_shader_path, kTokens->inputsNormal,
            pxr::SdfValueTypeNames->Normal3f);
        // Create the normal map shader and connect its output to the preview
        // surface normal attr.
        const std::vector<pxr::SdfPath> normal_map_output_attrs =
            AddTextureShader(material_path, normal_texture->file->c_str(),
                             pxr::TfToken("normal"), uvmap_st_output_attr,
                             {kTokens->outputsRgb});
        if (normal_map_output_attrs.size() == 1) {
          AddAttributeConnection(data_, normal_attr,
                                 normal_map_output_attrs[0]);
        }
      }
    }

    // Connect an emissive texture if specified.
    if (mjTEXROLE_EMISSIVE < textures.size()) {
      std::string emissive_texture_name = textures[mjTEXROLE_EMISSIVE];
      mjsTexture *emissive_texture = mjs_asTexture(
          mjs_findElement(spec_, mjOBJ_TEXTURE, emissive_texture_name.c_str()));
      if (emissive_texture) {
        pxr::SdfPath emissive_attr = CreateAttributeSpec(
            data_, preview_surface_shader_path, kTokens->inputsEmissiveColor,
            pxr::SdfValueTypeNames->Color3f);
        const std::vector<pxr::SdfPath> emissive_map_output_attrs =
            AddTextureShader(material_path, emissive_texture->file->c_str(),
                             pxr::TfToken("emissive"), uvmap_st_output_attr,
                             {kTokens->outputsRgb});
        if (emissive_map_output_attrs.size() == 1) {
          AddAttributeConnection(data_, emissive_attr,
                                 emissive_map_output_attrs[0]);
        }
      }
    }

    // Set the value of diffuse color. This can come from a diffuse texture
    // or as a value defined in mjsMaterial_.
    pxr::SdfPath diffuse_color_attr = CreateAttributeSpec(
        data_, preview_surface_shader_path, kTokens->inputsDiffuseColor,
        pxr::SdfValueTypeNames->Color3f);

    // Find the main texture if specified.
    std::string main_texture_name = textures[mjTEXROLE_RGB];
    mjsTexture *main_texture = mjs_asTexture(
        mjs_findElement(spec_, mjOBJ_TEXTURE, main_texture_name.c_str()));
    if (main_texture) {
      // Create the texture shader and connect it to the diffuse color
      // attribute.
      const std::vector<pxr::SdfPath> texture_diffuse_output_attrs =
          AddTextureShader(material_path, main_texture->file->c_str(),
                           pxr::TfToken("diffuse"), uvmap_st_output_attr,
                           {kTokens->outputsRgb});
      if (texture_diffuse_output_attrs.size() == 1) {
        AddAttributeConnection(data_, diffuse_color_attr,
                               texture_diffuse_output_attrs[0]);
      }
    } else {
      // If no texture is specified, use the rgba diffuse color.
      SetAttributeDefault(data_, diffuse_color_attr,
                          pxr::GfVec3f(material->rgba[0], material->rgba[1],
                                       material->rgba[2]));
    }
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

  void WriteKeyframesWithName(const std::string &name,
                              const std::vector<mjsKey *> &keyframes,
                              const pxr::SdfPath &parent_path) {
    if (keyframes.empty()) {
      return;
    }
    const auto keyframe_name = pxr::TfToken(pxr::TfMakeValidIdentifier(
        name.empty() ? MjcPhysicsTokens->MjcKeyframe : name));
    pxr::SdfPath keyframe_path = parent_path.AppendChild(keyframe_name);
    if (!data_->HasSpec(keyframe_path)) {
      CreatePrimSpec(data_, parent_path, keyframe_name,
                     pxr::MjcPhysicsTokens->MjcKeyframe);
    }
    auto set_attribute_data = [&](const pxr::SdfPath &attr_path,
                                  const pxr::VtDoubleArray &value,
                                  mjsKey *keyframe) {
      // If the keyframe time is the default, and there are no other keyframes
      // set the attribute at the default time code.
      if (keyframe->time == 0 && keyframes.size() == 1) {
        SetAttributeDefault(data_, attr_path, value);
      } else {
        SetAttributeTimeSample(data_, attr_path, keyframe->time, value);
      }
    };

    for (auto *keyframe : keyframes) {
      pxr::SdfPath qpos_attr_path =
          CreateAttributeSpec(data_, keyframe_path, MjcPhysicsTokens->mjcQpos,
                              pxr::SdfValueTypeNames->DoubleArray);
      set_attribute_data(
          qpos_attr_path,
          pxr::VtDoubleArray(keyframe->qpos->begin(), keyframe->qpos->end()),
          keyframe);

      pxr::SdfPath qvel_attr_path =
          CreateAttributeSpec(data_, keyframe_path, MjcPhysicsTokens->mjcQvel,
                              pxr::SdfValueTypeNames->DoubleArray);
      set_attribute_data(
          qvel_attr_path,
          pxr::VtDoubleArray(keyframe->qvel->begin(), keyframe->qvel->end()),
          keyframe);

      pxr::SdfPath act_attr_path =
          CreateAttributeSpec(data_, keyframe_path, MjcPhysicsTokens->mjcAct,
                              pxr::SdfValueTypeNames->DoubleArray);
      set_attribute_data(
          act_attr_path,
          pxr::VtDoubleArray(keyframe->act->begin(), keyframe->act->end()),
          keyframe);

      pxr::SdfPath ctrl_attr_path =
          CreateAttributeSpec(data_, keyframe_path, MjcPhysicsTokens->mjcCtrl,
                              pxr::SdfValueTypeNames->DoubleArray);
      set_attribute_data(
          ctrl_attr_path,
          pxr::VtDoubleArray(keyframe->ctrl->begin(), keyframe->ctrl->end()),
          keyframe);

      pxr::SdfPath mpos_attr_path =
          CreateAttributeSpec(data_, keyframe_path, MjcPhysicsTokens->mjcMpos,
                              pxr::SdfValueTypeNames->DoubleArray);
      set_attribute_data(
          mpos_attr_path,
          pxr::VtDoubleArray(keyframe->mpos->begin(), keyframe->mpos->end()),
          keyframe);

      pxr::SdfPath mquat_attr_path =
          CreateAttributeSpec(data_, keyframe_path, MjcPhysicsTokens->mjcMquat,
                              pxr::SdfValueTypeNames->DoubleArray);
      set_attribute_data(
          mquat_attr_path,
          pxr::VtDoubleArray(keyframe->mquat->begin(), keyframe->mquat->end()),
          keyframe);
    }
  }

  void WriteKeyframes() {
    std::unordered_map<std::string, std::vector<mjsKey *>> keyframes_map;
    mjsKey *keyframe = mjs_asKey(mjs_firstElement(spec_, mjOBJ_KEY));
    while (keyframe) {
      std::string keyframe_name = mjs_getName(keyframe->element)->empty()
                                      ? kTokens->keyframe
                                      : *mjs_getName(keyframe->element);
      keyframes_map[keyframe_name].push_back(keyframe);
      keyframe = mjs_asKey(mjs_nextElement(spec_, keyframe->element));
    }

    pxr::SdfPath scope_path =
        CreatePrimSpec(data_, body_paths_[kWorldIndex], kTokens->keyframesScope,
                       pxr::UsdGeomTokens->Scope);
    for (const auto &[keyframe_name, keyframes] : keyframes_map) {
      WriteKeyframesWithName(keyframe_name, keyframes, scope_path);
    }
  }

  void WriteActuator(mjsActuator *actuator, const pxr::SdfPath &parent_path) {
    pxr::TfToken valid_name = GetValidPrimName(*mjs_getName(actuator->element));
    pxr::SdfPath actuator_path = parent_path.AppendChild(valid_name);
    if (!data_->HasSpec(actuator_path)) {
      CreatePrimSpec(data_, parent_path, valid_name,
                     pxr::MjcPhysicsTokens->MjcActuator);
    }

    pxr::SdfPath target_path;
    if (actuator->trntype == mjtTrn::mjTRN_BODY) {
      int body_id = mj_name2id(model_, mjOBJ_BODY, actuator->target->c_str());
      target_path = body_paths_[body_id];
    } else if (actuator->trntype == mjtTrn::mjTRN_SITE ||
               actuator->trntype == mjtTrn::mjTRN_SLIDERCRANK) {
      int site_id = mj_name2id(model_, mjOBJ_SITE, actuator->target->c_str());
      target_path = site_paths_[site_id];
    } else if (actuator->trntype == mjtTrn::mjTRN_JOINT) {
      int joint_id = mj_name2id(model_, mjOBJ_JOINT, actuator->target->c_str());
      target_path = joint_paths_[joint_id];
    } else {
      TF_WARN(UnsupportedActuatorTypeError,
              "Unsupported transmission type for actuator %d",
              mjs_getId(actuator->element));
      return;
    }

    CreateRelationshipSpec(data_, actuator_path, MjcPhysicsTokens->mjcTarget,
                           target_path, pxr::SdfVariabilityUniform);

    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcGroup, actuator->group);

    if (!actuator->refsite->empty()) {
      int refsite_id =
          mj_name2id(model_, mjOBJ_SITE, actuator->refsite->c_str());
      pxr::SdfPath refsite_path = site_paths_[refsite_id];
      CreateRelationshipSpec(data_, actuator_path, MjcPhysicsTokens->mjcRefSite,
                             refsite_path, pxr::SdfVariabilityUniform);
    }

    if (!actuator->slidersite->empty()) {
      int slidersite_id =
          mj_name2id(model_, mjOBJ_SITE, actuator->slidersite->c_str());
      pxr::SdfPath slidersite_path = site_paths_[slidersite_id];
      CreateRelationshipSpec(data_, actuator_path,
                             MjcPhysicsTokens->mjcSliderSite, slidersite_path,
                             pxr::SdfVariabilityUniform);
    }

    const std::vector<std::pair<pxr::TfToken, int>> limited_attributes = {
        {MjcPhysicsTokens->mjcCtrlLimited, actuator->ctrllimited},
        {MjcPhysicsTokens->mjcForceLimited, actuator->forcelimited},
        {MjcPhysicsTokens->mjcActLimited, actuator->actlimited},
    };
    for (const auto &[token, value] : limited_attributes) {
      pxr::TfToken limited_token = pxr::MjcPhysicsTokens->auto_;
      if (value == mjLIMITED_TRUE) {
        limited_token = pxr::MjcPhysicsTokens->true_;
      } else if (value == mjLIMITED_FALSE) {
        limited_token = pxr::MjcPhysicsTokens->false_;
      }
      WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Token, token,
                            limited_token);
    }

    const std::vector<std::pair<pxr::TfToken, double>>
        actuator_double_attributes = {
            {MjcPhysicsTokens->mjcCtrlRangeMin, actuator->ctrlrange[0]},
            {MjcPhysicsTokens->mjcCtrlRangeMax, actuator->ctrlrange[1]},
            {MjcPhysicsTokens->mjcForceRangeMin, actuator->forcerange[0]},
            {MjcPhysicsTokens->mjcForceRangeMax, actuator->forcerange[1]},
            {MjcPhysicsTokens->mjcActRangeMin, actuator->actrange[0]},
            {MjcPhysicsTokens->mjcActRangeMax, actuator->actrange[1]},
            {MjcPhysicsTokens->mjcLengthRangeMin, actuator->lengthrange[0]},
            {MjcPhysicsTokens->mjcLengthRangeMax, actuator->lengthrange[1]},
            {MjcPhysicsTokens->mjcCrankLength, actuator->cranklength},
        };
    for (const auto &[token, value] : actuator_double_attributes) {
      WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Double,
                            token, value);
    }

    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcActDim, actuator->actdim);
    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Bool,
                          MjcPhysicsTokens->mjcActEarly,
                          (bool)actuator->actearly);
    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Double,
                          MjcPhysicsTokens->mjcInheritRange,
                          actuator->inheritrange);

    WriteUniformAttribute(
        actuator_path, pxr::SdfValueTypeNames->DoubleArray,
        MjcPhysicsTokens->mjcGear,
        pxr::VtDoubleArray(actuator->gear, actuator->gear + 6));

    pxr::TfToken dyn_type;
    if (actuator->dyntype == mjtDyn::mjDYN_NONE) {
      dyn_type = MjcPhysicsTokens->none;
    } else if (actuator->dyntype == mjtDyn::mjDYN_INTEGRATOR) {
      dyn_type = MjcPhysicsTokens->integrator;
    } else if (actuator->dyntype == mjtDyn::mjDYN_FILTER) {
      dyn_type = MjcPhysicsTokens->filter;
    } else if (actuator->dyntype == mjtDyn::mjDYN_FILTEREXACT) {
      dyn_type = MjcPhysicsTokens->filterexact;
    } else if (actuator->dyntype == mjtDyn::mjDYN_MUSCLE) {
      dyn_type = MjcPhysicsTokens->muscle;
    } else if (actuator->dyntype == mjtDyn::mjDYN_USER) {
      dyn_type = MjcPhysicsTokens->user;
    }
    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Token,
                          MjcPhysicsTokens->mjcDynType, dyn_type);
    WriteUniformAttribute(
        actuator_path, pxr::SdfValueTypeNames->DoubleArray,
        MjcPhysicsTokens->mjcDynPrm,
        pxr::VtDoubleArray(actuator->dynprm, actuator->dynprm + 10));

    pxr::TfToken gain_type;
    if (actuator->gaintype == mjtGain::mjGAIN_FIXED) {
      gain_type = MjcPhysicsTokens->fixed;
    } else if (actuator->gaintype == mjtGain::mjGAIN_AFFINE) {
      gain_type = MjcPhysicsTokens->affine;
    } else if (actuator->gaintype == mjtGain::mjGAIN_MUSCLE) {
      gain_type = MjcPhysicsTokens->muscle;
    } else if (actuator->gaintype == mjtGain::mjGAIN_USER) {
      gain_type = MjcPhysicsTokens->user;
    }
    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Token,
                          MjcPhysicsTokens->mjcGainType, gain_type);
    WriteUniformAttribute(
        actuator_path, pxr::SdfValueTypeNames->DoubleArray,
        MjcPhysicsTokens->mjcGainPrm,
        pxr::VtDoubleArray(actuator->gainprm, actuator->gainprm + 10));

    pxr::TfToken bias_type;
    if (actuator->biastype == mjtBias::mjBIAS_NONE) {
      bias_type = MjcPhysicsTokens->fixed;
    } else if (actuator->biastype == mjtBias::mjBIAS_AFFINE) {
      bias_type = MjcPhysicsTokens->affine;
    } else if (actuator->biastype == mjtBias::mjBIAS_MUSCLE) {
      bias_type = MjcPhysicsTokens->muscle;
    } else if (actuator->biastype == mjtBias::mjBIAS_USER) {
      bias_type = MjcPhysicsTokens->user;
    }
    WriteUniformAttribute(actuator_path, pxr::SdfValueTypeNames->Token,
                          MjcPhysicsTokens->mjcBiasType, bias_type);
    WriteUniformAttribute(
        actuator_path, pxr::SdfValueTypeNames->DoubleArray,
        MjcPhysicsTokens->mjcBiasPrm,
        pxr::VtDoubleArray(actuator->biasprm, actuator->biasprm + 10));
  }

  void WriteActuators() {
    pxr::SdfPath scope_path =
        CreatePrimSpec(data_, body_paths_[kWorldIndex], kTokens->actuatorsScope,
                       pxr::UsdGeomTokens->Scope);
    mjsActuator *actuator =
        mjs_asActuator(mjs_firstElement(spec_, mjOBJ_ACTUATOR));
    while (actuator) {
      WriteActuator(actuator, scope_path);
      actuator = mjs_asActuator(mjs_nextElement(spec_, actuator->element));
    }
  }

  pxr::SdfPath WriteMeshGeom(const mjsGeom *geom,
                             const pxr::SdfPath &body_path) {
    std::string mj_name = mjs_getName(geom->element)->empty()
                              ? *geom->meshname
                              : *mjs_getName(geom->element);
    auto name =
        GetAvailablePrimName(mj_name, pxr::UsdGeomTokens->Mesh, body_path);
    pxr::SdfPath subcomponent_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Xform);

    // Reference the mesh asset written in WriteMeshes.
    AddPrimReference(data_, subcomponent_path, mesh_paths_[*geom->meshname]);

    // We want to use instancing with meshes, and it requires creating a parent
    // scope to be referenced, with the Mesh prim as a child.
    // To be able to actually manipulate the Mesh prim, we need to create and
    // return the corresponding `over` prim as a child of the referencing prim.
    pxr::SdfPath over_mesh_path =
        CreatePrimSpec(data_, subcomponent_path, kTokens->sourceMesh,
                       pxr::UsdGeomTokens->Mesh, pxr::SdfSpecifierOver);

    return over_mesh_path;
  }

  pxr::SdfPath WriteSiteGeom(const mjsSite *site,
                             const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*mjs_getName(site->element),
                                     pxr::UsdGeomTokens->Cube, body_path);

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
    // MuJoCo uses half sizes. Always set size to 2 (and correspondingly extent
    // from -1 to 1), and let scale determine the actual size.
    pxr::SdfPath size_attr_path =
        CreateAttributeSpec(data_, box_path, pxr::UsdGeomTokens->size,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, size_attr_path, 2.0);

    pxr::SdfPath extent_attr_path =
        CreateAttributeSpec(data_, box_path, pxr::UsdGeomTokens->extent,
                            pxr::SdfValueTypeNames->Float3Array);
    SetAttributeDefault(data_, extent_attr_path,
                        pxr::VtArray<pxr::GfVec3f>(
                            {pxr::GfVec3f(-1, -1, -1), pxr::GfVec3f(1, 1, 1)}));

    pxr::GfVec3f scale(static_cast<float>(size[0]), static_cast<float>(size[1]),
                       static_cast<float>(size[2]));
    WriteScaleXformOp(box_path, scale);
    WriteXformOpOrder(box_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpScale});
    return box_path;
  }

  pxr::SdfPath WriteBoxGeom(const mjsGeom *geom,
                            const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdGeomTokens->Cube, body_path);

    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WriteBox(name, geom_size, body_path);
  }

  pxr::SdfPath WriteCapsule(const pxr::TfToken name, const mjtNum *size,
                            const pxr::SdfPath &body_path) {
    pxr::SdfPath capsule_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Capsule);

    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, capsule_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, radius_attr_path, (double)size[0]);

    pxr::SdfPath height_attr_path =
        CreateAttributeSpec(data_, capsule_path, pxr::UsdGeomTokens->height,
                            pxr::SdfValueTypeNames->Double);
    // MuJoCo uses half sizes.
    SetAttributeDefault(data_, height_attr_path, (double)(size[1] * 2));
    return capsule_path;
  }

  pxr::SdfPath WriteCapsuleGeom(const mjsGeom *geom,
                                const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdGeomTokens->Capsule, body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];

    return WriteCapsule(name, geom_size, body_path);
  }

  pxr::SdfPath WriteCylinder(const pxr::TfToken name, const mjtNum *size,
                             const pxr::SdfPath &body_path) {
    pxr::SdfPath cylinder_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Cylinder);

    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, cylinder_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, radius_attr_path, (double)size[0]);

    pxr::SdfPath height_attr_path =
        CreateAttributeSpec(data_, cylinder_path, pxr::UsdGeomTokens->height,
                            pxr::SdfValueTypeNames->Double);
    // MuJoCo uses half sizes.
    SetAttributeDefault(data_, height_attr_path, (double)(size[1] * 2));
    return cylinder_path;
  }

  pxr::SdfPath WriteCylinderGeom(const mjsGeom *geom,
                                 const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdGeomTokens->Cylinder, body_path);

    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WriteCylinder(name, geom_size, body_path);
  }

  pxr::SdfPath WriteEllipsoid(const pxr::TfToken name, const mjtNum *size,
                              const pxr::SdfPath &body_path) {
    pxr::SdfPath ellipsoid_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Sphere);

    pxr::GfVec3f scale = {static_cast<float>(size[0]),
                          static_cast<float>(size[1]),
                          static_cast<float>(size[2])};

    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, ellipsoid_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, radius_attr_path, 1.0);

    WriteScaleXformOp(ellipsoid_path, scale);
    WriteXformOpOrder(ellipsoid_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpScale});
    return ellipsoid_path;
  }

  pxr::SdfPath WriteEllipsoidGeom(const mjsGeom *geom,
                                  const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdGeomTokens->Sphere, body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];

    return WriteEllipsoid(name, geom_size, body_path);
  }

  pxr::SdfPath WriteSphere(const pxr::TfToken name, const mjtNum *size,
                           const pxr::SdfPath &body_path) {
    pxr::SdfPath sphere_path =
        CreatePrimSpec(data_, body_path, name, pxr::UsdGeomTokens->Sphere);

    pxr::SdfPath radius_attr_path =
        CreateAttributeSpec(data_, sphere_path, pxr::UsdGeomTokens->radius,
                            pxr::SdfValueTypeNames->Double);
    SetAttributeDefault(data_, radius_attr_path, (double)size[0]);
    return sphere_path;
  }

  pxr::SdfPath WriteSphereGeom(const mjsGeom *geom,
                               const pxr::SdfPath &body_path) {
    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdGeomTokens->Sphere, body_path);
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
    auto name = GetAvailablePrimName(*mjs_getName(geom->element),
                                     pxr::UsdGeomTokens->Plane, body_path);
    int geom_idx = mjs_getId(geom->element);
    mjtNum *geom_size = &model_->geom_size[geom_idx * 3];
    return WritePlane(name, geom_size, body_path);
  }

  void WriteSite(mjsSite *site, const mjsBody *body) {
    const int body_id = mjs_getId(body->element);
    const auto &body_path = body_paths_[body_id];
    auto name = GetAvailablePrimName(*mjs_getName(site->element),
                                     pxr::UsdGeomTokens->Xform, body_path);

    // Create a geom primitive and set its purpose to guide so it won't be
    // rendered.
    pxr::SdfPath site_path = WriteSiteGeom(site, body_path);
    SetPrimPurpose(data_, site_path, pxr::UsdGeomTokens->guide);

    ApplyApiSchema(data_, site_path, MjcPhysicsTokens->MjcSiteAPI);

    WriteUniformAttribute(site_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcGroup, site->group);

    WriteColorAndOpacityAttributes(site_path, site);

    int site_id = mjs_getId(site->element);
    auto transform = MujocoPosQuatToTransform(&model_->site_pos[3 * site_id],
                                              &model_->site_quat[4 * site_id]);
    WriteTransformXformOp(site_path, transform);

    PrependToXformOpOrder(
        site_path, pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});

    site_paths_[site_id] = site_path;
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

    WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Int,
                          MjcPhysicsTokens->mjcGroup, geom->group);

    if (model_->geom_contype[geom_id] == 0 &&
        model_->geom_conaffinity[geom_id] == 0) {
      // If the geom is purely visual, apply the imageable API.
      ApplyApiSchema(data_, geom_path, MjcPhysicsTokens->MjcImageableAPI);
    }

    // Apply the physics schemas if we are writing physics and the
    // geom participates in collisions.
    if (model_->geom_contype[geom_id] != 0 ||
        model_->geom_conaffinity[geom_id] != 0) {
      ApplyApiSchema(data_, geom_path,
                     pxr::UsdPhysicsTokens->PhysicsCollisionAPI);
      ApplyApiSchema(data_, geom_path, MjcPhysicsTokens->MjcCollisionAPI);

      WriteUniformAttribute(
          geom_path, pxr::SdfValueTypeNames->Bool,
          MjcPhysicsTokens->mjcShellinertia,
          geom->typeinertia == mjtGeomInertia::mjINERTIA_SHELL);

      WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Int,
                            MjcPhysicsTokens->mjcPriority, geom->priority);

      WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Int,
                            MjcPhysicsTokens->mjcCondim, geom->condim);

      WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcSolmix, geom->solmix);

      WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcSolmix, geom->solmix);

      WriteUniformAttribute(
          geom_path, pxr::SdfValueTypeNames->DoubleArray,
          MjcPhysicsTokens->mjcSolref,
          pxr::VtArray<double>(geom->solref, geom->solref + mjNREF));

      WriteUniformAttribute(
          geom_path, pxr::SdfValueTypeNames->DoubleArray,
          MjcPhysicsTokens->mjcSolimp,
          pxr::VtArray<double>(geom->solimp, geom->solimp + mjNIMP));

      WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcMargin, geom->margin);

      WriteUniformAttribute(geom_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcGap, geom->gap);

      if (geom->mass >= mjMINVAL || geom->density >= mjMINVAL) {
        ApplyApiSchema(data_, geom_path, pxr::UsdPhysicsTokens->PhysicsMassAPI);
      }

      if (geom->mass >= mjMINVAL) {
        pxr::SdfPath mass_attr = CreateAttributeSpec(
            data_, geom_path, pxr::UsdPhysicsTokens->physicsMass,
            pxr::SdfValueTypeNames->Float, pxr::SdfVariabilityUniform);

        // Make sure to cast to float here since mjtNum might be a double.
        SetAttributeDefault(data_, mass_attr, (float)geom->mass);
      }

      // Even though density is not used for mass computation when mass exists
      // we want to retain the information anyways.
      if (geom->density >= mjMINVAL) {
        pxr::SdfPath density_attr = CreateAttributeSpec(
            data_, geom_path, pxr::UsdPhysicsTokens->physicsDensity,
            pxr::SdfValueTypeNames->Float, pxr::SdfVariabilityUniform);

        // Make sure to cast to float here since mjtNum might be a double.
        SetAttributeDefault(data_, density_attr, (float)geom->density);
      }

      mjsDefault *geom_default = mjs_getDefault(geom->element);

      if (geom->friction[0] != geom_default->geom->friction[0] ||
          geom->friction[1] != geom_default->geom->friction[1] ||
          geom->friction[2] != geom_default->geom->friction[2]) {
        pxr::SdfPath physics_material_path = WritePhysicsMaterial(geom);
        ApplyApiSchema(data_, geom_path,
                       pxr::UsdShadeTokens->MaterialBindingAPI);
        // Bind the material to this geom.
        CreateRelationshipSpec(
            data_, geom_path, pxr::UsdShadeTokens->materialBinding,
            physics_material_path, pxr::SdfVariabilityUniform);
      }

      // For meshes, also apply PhysicsMeshCollisionAPI and set the
      // approximation attribute.
      if (geom->type == mjGEOM_MESH) {
        ApplyApiSchema(data_, geom_path,
                       pxr::UsdPhysicsTokens->PhysicsMeshCollisionAPI);

        // Note: MuJoCo documentation states that for collision purposes, meshes
        // are always replaced with their convex hulls. Therefore, we set the
        // approximation attribute to convexHull explicitly.
        pxr::SdfPath approximation_attr = CreateAttributeSpec(
            data_, geom_path, pxr::UsdPhysicsTokens->physicsApproximation,
            pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
        SetAttributeDefault(data_, approximation_attr,
                            pxr::UsdPhysicsTokens->convexHull);
      }
    } else {
      // Currently imageable only has a group API. But since it's the same
      // naming in MjcCollisionsAPI we've already set it earlier in this
      // function.
      ApplyApiSchema(data_, geom_path, MjcPhysicsTokens->MjcImageableAPI);
    }

    mjsDefault *spec_default = mjs_getDefault(geom->element);
    pxr::TfToken valid_class_name =
        GetValidPrimName(*mjs_getName(spec_default->element));
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

    WriteColorAndOpacityAttributes(geom_path, geom);

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

  void WriteJoints(mjsBody *body) {
    int body_id = mjs_getId(body->element);
    if (body_id == kWorldIndex) return;

    mjsJoint *joint = mjs_asJoint(mjs_firstChild(body, mjOBJ_JOINT, false));

    if (!joint) {
      // If no joint is found, then we pass nullptr to create a FixedJoint.
      // WriteJoint properly handles the case where the parent is the worldbody.
      WriteJoint(nullptr, body);
    } else {
      WriteJoint(joint, body);
      if (mjs_asJoint(mjs_nextChild(body, joint->element, false))) {
        TF_WARN(
            "Multiple joints found for body %d. Only writing the first one.",
            body_id);
      }
    }
  }

  // Write the joint. If null, then a FixedJoint is created.
  void WriteJoint(mjsJoint *joint, const mjsBody *parent_mj_body) {
    // Default to fixed joint if joint is null.
    pxr::TfToken joint_prim_type = pxr::UsdPhysicsTokens->PhysicsFixedJoint;

    int joint_id = -1;
    if (joint) {
      joint_id = mjs_getId(joint->element);
      mjtJoint type = (mjtJoint)model_->jnt_type[joint_id];
      switch (type) {
        case mjJNT_FREE:
          // Free joints are guaranteed to only ever be on the top-level body so
          // we just write no joint. As a top-level body with no joint it will
          // be considered a floating-base body.
          return;
        case mjJNT_HINGE:
          joint_prim_type = pxr::UsdPhysicsTokens->PhysicsRevoluteJoint;
          break;
        case mjJNT_SLIDE:
          joint_prim_type = pxr::UsdPhysicsTokens->PhysicsPrismaticJoint;
          break;
        default:
          TF_WARN("Unsupported joint type '%d' for joint '%s'. Skipping.",
                  (int)type, mjs_getName(joint->element)->c_str());
          return;
      }
    }

    int body_id = mjs_getId(parent_mj_body->element);

    // the joint connects the current body as body1, to its parent body as
    // body0.
    int body1_id_usd = body_id;
    int body0_id_usd = model_->body_parentid[body_id];

    const pxr::SdfPath &body1_path_usd = body_paths_[body1_id_usd];
    auto joint_name = joint ? *mjs_getName(joint->element) : "FixedJoint";
    pxr::TfToken joint_name_token =
        GetAvailablePrimName(joint_name, kTokens->joint, body1_path_usd);
    pxr::SdfPath joint_path = CreatePrimSpec(data_, body1_path_usd,
                                             joint_name_token, joint_prim_type);

    // Set body0 and body1 relationships
    // For the initial joints that connect to the world, we signal this by
    // keeping the body0 relationship empty.
    if (body0_id_usd != kWorldIndex) {
      const pxr::SdfPath &body0_path_usd = body_paths_[body0_id_usd];
      CreateRelationshipSpec(data_, joint_path,
                             pxr::UsdPhysicsTokens->physicsBody0,
                             body0_path_usd, pxr::SdfVariabilityUniform);
    }
    CreateRelationshipSpec(data_, joint_path,
                           pxr::UsdPhysicsTokens->physicsBody1, body1_path_usd,
                           pxr::SdfVariabilityUniform);

    // Joint frame in MuJoCo is defined by jnt_pos and jnt_axis in body1's frame
    // For FixedJoint, these are both unity.
    pxr::GfVec3d mj_jnt_pos = pxr::GfVec3d(0.0);
    pxr::GfVec3d mj_jnt_axis = pxr::GfVec3d(0.0, 0.0, 1.0);
    if (joint) {
      mj_jnt_pos = pxr::GfVec3d(&model_->jnt_pos[joint_id * 3]);
      mj_jnt_axis = pxr::GfVec3d(&model_->jnt_axis[joint_id * 3]);
    }

    // Local joint frame for body1
    pxr::GfVec3f local_pos1(mj_jnt_pos);
    pxr::GfQuatf local_rot1(
        pxr::GfRotation(pxr::GfVec3f::ZAxis(), mj_jnt_axis).GetQuat());

    SetAttributeDefault(
        data_,
        CreateAttributeSpec(data_, joint_path,
                            pxr::UsdPhysicsTokens->physicsLocalPos1,
                            pxr::SdfValueTypeNames->Float3),
        local_pos1);
    if (joint_prim_type == pxr::UsdPhysicsTokens->PhysicsRevoluteJoint ||
        joint_prim_type == pxr::UsdPhysicsTokens->PhysicsPrismaticJoint) {
      SetAttributeDefault(
          data_,
          CreateAttributeSpec(data_, joint_path,
                              pxr::UsdPhysicsTokens->physicsLocalRot1,
                              pxr::SdfValueTypeNames->Quatf),
          local_rot1);
    } else {
      SetAttributeDefault(
          data_,
          CreateAttributeSpec(data_, joint_path,
                              pxr::UsdPhysicsTokens->physicsLocalRot1,
                              pxr::SdfValueTypeNames->Quatf),
          pxr::GfQuatf::GetIdentity());
    }

    // Calculate local joint frame for body0
    pxr::GfMatrix4d body1_transform_local =
        MujocoPosQuatToTransform(&model_->body_pos[body1_id_usd * 3],
                                 &model_->body_quat[body1_id_usd * 4]);
    pxr::GfVec3d jnt_pos_parent_local =
        body1_transform_local.Transform(mj_jnt_pos);
    pxr::GfVec3d jnt_axis_parent_local =
        body1_transform_local.TransformDir(mj_jnt_axis);

    pxr::GfVec3f local_pos0(jnt_pos_parent_local);

    SetAttributeDefault(
        data_,
        CreateAttributeSpec(data_, joint_path,
                            pxr::UsdPhysicsTokens->physicsLocalPos0,
                            pxr::SdfValueTypeNames->Float3),
        local_pos0);

    if (joint_prim_type == pxr::UsdPhysicsTokens->PhysicsRevoluteJoint ||
        joint_prim_type == pxr::UsdPhysicsTokens->PhysicsPrismaticJoint) {
      pxr::GfQuatf other_rot0(
          pxr::GfRotation(pxr::GfVec3f::ZAxis(), jnt_axis_parent_local)
              .GetQuat());

      SetAttributeDefault(
          data_,
          CreateAttributeSpec(data_, joint_path,
                              pxr::UsdPhysicsTokens->physicsLocalRot0,
                              pxr::SdfValueTypeNames->Quatf),
          other_rot0);
    } else {
      // Fixed joints have no frame and no axis per se. We simply need the
      // rotation quaternion of the body its on.
      SetAttributeDefault(
          data_,
          CreateAttributeSpec(data_, joint_path,
                              pxr::UsdPhysicsTokens->physicsLocalRot0,
                              pxr::SdfValueTypeNames->Quatf),
          body1_transform_local.ExtractRotationQuat());
    }

    if (joint) {
      mjtJoint type = (mjtJoint)model_->jnt_type[joint_id];

      // Joint-specific attributes
      if (type == mjJNT_HINGE || type == mjJNT_SLIDE) {
        // The joint motion occurs around/along the Z-axis of the joint frame
        // established by localRot0/1.
        SetAttributeDefault(
            data_,
            CreateAttributeSpec(data_, joint_path,
                                pxr::UsdPhysicsTokens->physicsAxis,
                                pxr::SdfValueTypeNames->Token),
            pxr::UsdPhysicsTokens->z);  // "Z" axis
      }

      if (model_->jnt_limited[joint_id]) {
        float lower_limit = model_->jnt_range[joint_id * 2];
        float upper_limit = model_->jnt_range[joint_id * 2 + 1];

        if (type == mjJNT_HINGE) {
          // Convert radians to degrees for USD
          // As per the XML Reference, "mjModel always uses radians"
          lower_limit *= (180.0 / mjPI);
          upper_limit *= (180.0 / mjPI);
          SetAttributeDefault(
              data_,
              CreateAttributeSpec(data_, joint_path,
                                  pxr::UsdPhysicsTokens->physicsLowerLimit,
                                  pxr::SdfValueTypeNames->Float),
              lower_limit);
          SetAttributeDefault(
              data_,
              CreateAttributeSpec(data_, joint_path,
                                  pxr::UsdPhysicsTokens->physicsUpperLimit,
                                  pxr::SdfValueTypeNames->Float),
              upper_limit);
        } else if (type == mjJNT_SLIDE) {
          SetAttributeDefault(
              data_,
              CreateAttributeSpec(data_, joint_path,
                                  pxr::UsdPhysicsTokens->physicsLowerLimit,
                                  pxr::SdfValueTypeNames->Float),
              lower_limit);
          SetAttributeDefault(
              data_,
              CreateAttributeSpec(data_, joint_path,
                                  pxr::UsdPhysicsTokens->physicsUpperLimit,
                                  pxr::SdfValueTypeNames->Float),
              upper_limit);
        }
      }

      // Finally write the mjcPhysicsJointAPI attributes.
      ApplyApiSchema(data_, joint_path, MjcPhysicsTokens->MjcJointAPI);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Int,
                            MjcPhysicsTokens->mjcGroup, joint->group);

      WriteUniformAttribute(
          joint_path, pxr::SdfValueTypeNames->DoubleArray,
          MjcPhysicsTokens->mjcSpringdamper,
          pxr::VtArray<double>(joint->springdamper, joint->springdamper + 2));

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->DoubleArray,
                            MjcPhysicsTokens->mjcSolreflimit,
                            pxr::VtArray<double>(joint->solref_limit,
                                                 joint->solref_limit + mjNREF));

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->DoubleArray,
                            MjcPhysicsTokens->mjcSolimplimit,
                            pxr::VtArray<double>(joint->solimp_limit,
                                                 joint->solimp_limit + mjNIMP));

      WriteUniformAttribute(
          joint_path, pxr::SdfValueTypeNames->DoubleArray,
          MjcPhysicsTokens->mjcSolreffriction,
          pxr::VtArray<double>(joint->solref_friction,
                               joint->solref_friction + mjNREF));

      WriteUniformAttribute(
          joint_path, pxr::SdfValueTypeNames->DoubleArray,
          MjcPhysicsTokens->mjcSolimpfriction,
          pxr::VtArray<double>(joint->solimp_friction,
                               joint->solimp_friction + mjNIMP));

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcStiffness, joint->stiffness);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcActuatorfrcrangeMin,
                            joint->actfrcrange[0]);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcActuatorfrcrangeMax,
                            joint->actfrcrange[1]);

      pxr::TfToken actuatorfrclimited_token = MjcPhysicsTokens->auto_;
      if (joint->actfrclimited == mjLIMITED_TRUE) {
        actuatorfrclimited_token = MjcPhysicsTokens->true_;
      } else if (joint->actfrclimited == mjLIMITED_FALSE) {
        actuatorfrclimited_token = MjcPhysicsTokens->false_;
      }
      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Token,
                            MjcPhysicsTokens->mjcActuatorfrclimited,
                            actuatorfrclimited_token);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Bool,
                            MjcPhysicsTokens->mjcActuatorgravcomp,
                            static_cast<bool>(joint->actgravcomp));

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcMargin, joint->margin);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcRef, joint->ref);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcSpringref, joint->springref);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcArmature, joint->armature);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcDamping, joint->damping);

      WriteUniformAttribute(joint_path, pxr::SdfValueTypeNames->Double,
                            MjcPhysicsTokens->mjcFrictionloss,
                            joint->frictionloss);
    }
    if (joint_id >= 0) {
      joint_paths_[joint_id] = joint_path;
    }
  }

  void WriteCamera(mjsCamera *spec_cam, const mjsBody *body) {
    const auto &body_path = body_paths_[mjs_getId(body->element)];
    auto name = GetAvailablePrimName(*mjs_getName(spec_cam->element),
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

    WriteUniformAttribute(camera_path, pxr::SdfValueTypeNames->Float2,
                          pxr::UsdGeomTokens->clippingRange,
                          pxr::GfVec2f(znear, zfar));
    WriteUniformAttribute(camera_path, pxr::SdfValueTypeNames->Float,
                          pxr::UsdGeomTokens->focalLength, znear);
    WriteUniformAttribute(camera_path, pxr::SdfValueTypeNames->Float,
                          pxr::UsdGeomTokens->verticalAperture,
                          vertical_apperture);
    WriteUniformAttribute(camera_path, pxr::SdfValueTypeNames->Float,
                          pxr::UsdGeomTokens->horizontalAperture,
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
    auto name = GetAvailablePrimName(*mjs_getName(light->element),
                                     kTokens->light, body_path);
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

  void WriteBody(mjsBody *body) {
    int body_id = mjs_getId(body->element);
    // This should be safe as we process parent bodies before children.
    mjsBody *parent = mjs_getParent(body->element);
    int parent_id = mjs_getId(parent->element);
    pxr::SdfPath parent_path = body_paths_[parent_id];
    pxr::TfToken body_name = GetValidPrimName(*mjs_getName(body->element));

    // Create Xform prim for body.
    pxr::SdfPath body_path = CreatePrimSpec(data_, parent_path, body_name,
                                            pxr::UsdGeomTokens->Xform);
    // The parent_path will be a component which makes the actual articulated
    // bodies subcomponents.
    auto kind = parent_id == kWorldIndex ? pxr::KindTokens->component
                                         : pxr::KindTokens->subcomponent;
    SetPrimKind(data_, body_path, kind);

    // If the parent is not the world body, but is child of the world body
    // then we need to apply the articulation root API.
    if (parent_id != kWorldIndex) {
      int parent_parent_id = mjs_getId(mjs_getParent(parent->element)->element);
      // We guard against applying the API more than once, which can happen when
      // there are multiple children.
      if (parent_parent_id == kWorldIndex &&
          articulation_roots_.find(parent_id) == articulation_roots_.end()) {
        ApplyApiSchema(data_, parent_path,
                       pxr::UsdPhysicsTokens->PhysicsArticulationRootAPI);
        articulation_roots_.insert(parent_id);
      }
    }

    // If the body had a mass specified then it must have either inertia or
    // fullinertia specified per inertia element XML documentation.
    // Therefore it is sufficient to check if the mass is non-zero to see if
    // we should set inertial attributes on the body.
    //
    // Note that if the user has NOT specified any inertial properties then
    // we don't want to pull values from the compiled model since coming back
    // into Mujoco would take those values instead of computing them
    // automatically from the subtree.
    if (body->mass > 0) {
      // User might have specified the inertia via fullinertia and the
      // compiler has extracted all values properly. So leverage those
      // instead of doing the computation ourselves here.
      ApplyApiSchema(data_, body_path, pxr::UsdPhysicsTokens->PhysicsMassAPI);
      WriteUniformAttribute(body_path, pxr::SdfValueTypeNames->Float,
                            pxr::UsdPhysicsTokens->physicsMass,
                            (float)model_->body_mass[body_id]);

      mjtNum *body_ipos = &model_->body_ipos[body_id * 3];
      pxr::GfVec3f inertial_pos(body_ipos[0], body_ipos[1], body_ipos[2]);
      WriteUniformAttribute(body_path, pxr::SdfValueTypeNames->Point3f,
                            pxr::UsdPhysicsTokens->physicsCenterOfMass,
                            inertial_pos);

      mjtNum *body_iquat = &model_->body_iquat[body_id * 4];
      pxr::GfQuatf inertial_frame(body_iquat[0], body_iquat[1], body_iquat[2],
                                  body_iquat[3]);
      WriteUniformAttribute(body_path, pxr::SdfValueTypeNames->Quatf,
                            pxr::UsdPhysicsTokens->physicsPrincipalAxes,
                            inertial_frame);

      mjtNum *inertia = &model_->body_inertia[body_id * 3];
      pxr::GfVec3f diag_inertia(inertia[0], inertia[1], inertia[2]);
      WriteUniformAttribute(body_path, pxr::SdfValueTypeNames->Float3,
                            pxr::UsdPhysicsTokens->physicsDiagonalInertia,
                            diag_inertia);
    }

    ApplyApiSchema(data_, body_path,
                   pxr::UsdPhysicsTokens->PhysicsRigidBodyAPI);

    // Create classes if necessary
    mjsDefault *spec_default = mjs_getDefault(body->element);

    pxr::TfToken body_class_name =
        GetValidPrimName(*mjs_getName(spec_default->element));
    pxr::SdfPath body_class_path = class_path_.AppendChild(body_class_name);
    if (!data_->HasSpec(body_class_path)) {
      CreateClassSpec(data_, class_path_, body_class_name);
    }

    // Create XformOp attribute for body transform.

    pxr::SdfPath xform_op_path =
        CreateAttributeSpec(data_, body_path, kTokens->xformOpTransform,
                            pxr::SdfValueTypeNames->Matrix4d);
    // mjModel will have all frames already accounted for so no need to worry
    // about them here.
    auto body_xform = MujocoPosQuatToTransform(&model_->body_pos[body_id * 3],
                                               &model_->body_quat[body_id * 4]);
    SetAttributeDefault(data_, xform_op_path, body_xform);

    // Create XformOpOrder attribute for body transform order.
    // For us this is simply the transform we authored above.
    WriteXformOpOrder(body_path,
                      pxr::VtArray<pxr::TfToken>{kTokens->xformOpTransform});

    pxr::VtDictionary customData;
    customData[kTokens->body_name] = *mjs_getName(body->element);
    SetPrimMetadata(data_, body_path, pxr::SdfFieldKeys->CustomData,
                    customData);

    body_paths_[body_id] = body_path;
  }

  void WriteBodies() {
    mjsBody *body = mjs_asBody(mjs_firstElement(spec_, mjOBJ_BODY));
    while (body) {
      // Only write a rigidbody if we are not the world body.
      // We fall through since the world body might have static
      // geom children.
      if (mjs_getId(body->element) != kWorldIndex) {
        WriteBody(body);
      }
      WriteSites(body);
      WriteGeoms(body);
      WriteJoints(body);
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

bool WriteSpecToData(mjSpec *spec, pxr::SdfAbstractDataRefPtr &data) {
  // Create pseudo root first.
  data->CreateSpec(pxr::SdfPath::AbsoluteRootPath(),
                   pxr::SdfSpecTypePseudoRoot);

  mjModel *model = mj_compile(spec, nullptr);
  if (model == nullptr) {
    TF_ERROR(MujocoCompilationError, "%s", mjs_getError(spec));
    return false;
  }

  ModelWriter(spec, model, data).Write();

  return true;
}

}  // namespace usd
}  // namespace mujoco
