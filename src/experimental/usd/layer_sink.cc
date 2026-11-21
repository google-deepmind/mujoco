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

#include <mujoco/experimental/usd/layer_sink.h>

#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include <pxr/base/gf/declare.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/childrenPolicies.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformCache.h>

PXR_NAMESPACE_OPEN_SCOPE
TF_DEFINE_PRIVATE_TOKENS(_layer_sink_tokens,
                         ((xformOpTranslate, "xformOp:translate:mujoco"))
                         ((xformOpOrient, "xformOp:orient:mujoco")));
PXR_NAMESPACE_CLOSE_SCOPE

namespace {
pxr::SdfAttributeSpecHandle GetOrCreateAttribute(
    pxr::SdfLayerHandle layer, pxr::SdfPrimSpecHandle prim_spec,
    const pxr::SdfPath& path, const pxr::TfToken& token,
    const pxr::SdfValueTypeName& type, pxr::SdfVariability variability) {
  // Find or create transform attribute.
  pxr::SdfAttributeSpecHandle attr =
      layer->GetAttributeAtPath(path.AppendProperty(token));

  if (!attr) {
    attr = pxr::SdfAttributeSpec::New(prim_spec, token, type, variability);
  }
  return attr;
}
}  // namespace

namespace mujoco {
namespace usd {
LayerSink::LayerSink(pxr::UsdStageRefPtr stage) : stage_(stage) {}

void LayerSink::Update(const mjData* const data,
                       std::vector<pxr::SdfPath> body_paths) {
  pxr::SdfLayerHandle layer = stage_->GetEditTarget().GetLayer();

  pxr::UsdTimeCode timecode = data->time * layer->GetTimeCodesPerSecond();
  pxr::UsdGeomXformCache xform_cache(timecode);

  // Compute world transforms for all valid body prims.
  std::unordered_map<pxr::SdfPath, pxr::GfMatrix4d, pxr::SdfPath::Hash>
      world_transforms;
  std::vector<pxr::GfMatrix4d> body_world_xforms(body_paths.size());
  for (int i = 0; i < body_paths.size(); i++) {
    const pxr::SdfPath& path = body_paths[i];
    // Some bodies such as the world body don't have mapped USD prims to
    // transform.
    if (path.IsEmpty()) {
      continue;
    }

    // Convert pose data.
    const mjtNum* xpos = &data->xpos[i * 3];
    const mjtNum* xquat = &data->xquat[i * 4];
    pxr::GfMatrix4d world_xform(
        pxr::GfRotation(
            pxr::GfQuatd(xquat[0], xquat[1], xquat[2], xquat[3])
                .GetNormalized()),
        pxr::GfVec3d(xpos[0], xpos[1], xpos[2]));
    body_world_xforms[i] = world_xform;
    world_transforms[path] = world_xform;
  }

  // Compute relative transforms outside of SdfChangeBlock. Use MuJoCo world
  // poses directly for ancestor bodies, and query static ancestor prims before
  // authoring layer edits.
  std::vector<pxr::GfVec3d> relative_translations(body_paths.size());
  std::vector<pxr::GfQuatd> relative_orients(body_paths.size());
  for (int i = 0; i < body_paths.size(); i++) {
    const pxr::SdfPath& path = body_paths[i];
    if (path.IsEmpty()) {
      continue;
    }

    pxr::GfMatrix4d parent_xform(1.0);
    bool has_static_ancestor = false;
    for (pxr::SdfPath ancestor_path = path.GetParentPath();
         !ancestor_path.IsEmpty() &&
         ancestor_path != pxr::SdfPath::AbsoluteRootPath();
         ancestor_path = ancestor_path.GetParentPath()) {
      auto it = world_transforms.find(ancestor_path);
      if (it != world_transforms.end()) {
        parent_xform *= it->second;
        break;
      }
      pxr::UsdPrim ancestor_prim = stage_->GetPrimAtPath(ancestor_path);
      if (!ancestor_prim) {
        break;
      }
      bool resets_xform_stack = false;
      parent_xform *= xform_cache.GetLocalTransformation(ancestor_prim,
                                                         &resets_xform_stack);
      has_static_ancestor = true;
      if (resets_xform_stack) {
        break;
      }
    }
    pxr::GfMatrix4d relative_xform =
        body_world_xforms[i] * parent_xform.GetInverse();
    relative_translations[i] = relative_xform.ExtractTranslation();
    relative_orients[i] = (has_static_ancestor
                               ? relative_xform.RemoveScaleShear()
                               : relative_xform)
                              .ExtractRotationQuat()
                              .GetNormalized();
  }

  pxr::SdfChangeBlock change_block;
  const pxr::VtValue xformop_order(
      pxr::VtTokenArray{pxr::_layer_sink_tokens->xformOpTranslate,
                        pxr::_layer_sink_tokens->xformOpOrient});
  // TODO(shaves) this can be parallelised.
  for (int i = 0; i < body_paths.size(); i++) {
    const pxr::SdfPath& path = body_paths[i];
    if (path.IsEmpty()) {
      continue;
    }

    pxr::SdfPrimSpecHandle prim_spec = pxr::SdfCreatePrimInLayer(layer, path);

    pxr::SdfAttributeSpecHandle translate_attr = GetOrCreateAttribute(
        layer, prim_spec, path, pxr::_layer_sink_tokens->xformOpTranslate,
        pxr::SdfValueTypeNames->Double3, pxr::SdfVariabilityVarying);
    layer->SetTimeSample(translate_attr->GetPath(), timecode.GetValue(),
                         relative_translations[i]);

    pxr::SdfAttributeSpecHandle orient_attr = GetOrCreateAttribute(
        layer, prim_spec, path, pxr::_layer_sink_tokens->xformOpOrient,
        pxr::SdfValueTypeNames->Quatd, pxr::SdfVariabilityVarying);
    layer->SetTimeSample(orient_attr->GetPath(), timecode.GetValue(),
                         relative_orients[i]);

    // Create xformop_order attribute if missing.
    pxr::SdfAttributeSpecHandle op_order_attr = GetOrCreateAttribute(
        layer, prim_spec, path, pxr::UsdGeomTokens->xformOpOrder,
        pxr::SdfValueTypeNames->TokenArray, pxr::SdfVariabilityUniform);

    // Override all local transforms so that this is the absolute position.
    // We don't reset the xform stack here, so parent transforms will still
    // affect the world pose.
    op_order_attr->SetDefaultValue(xformop_order);
  }
}
}  // namespace usd
}  // namespace mujoco
