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

#include <vector>

#include <mujoco/mujoco.h>
#include <pxr/base/gf/declare.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quaternion.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/tf/staticTokens.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/childrenPolicies.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformCache.h>

PXR_NAMESPACE_OPEN_SCOPE
TF_DEFINE_PRIVATE_TOKENS(_layer_sink_tokens,
                         ((xformOpTransform,
                           "xformOp:transform:mujoco"))(Xform));
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

  pxr::SdfChangeBlock change_block;
  // TODO(shaves) this can be parallelised.
  for (int i = 0; i < body_paths.size(); i++) {
    const pxr::SdfPath& path = body_paths[i];

    // Author transform timesample
    // Convert pose data.
    mjtNum* xpos = &data->xpos[i * 3];
    mjtNum* xquat = &data->xquat[i * 4];
    pxr::GfVec3d translation(xpos[0], xpos[1], xpos[2]);
    pxr::GfVec3d imag_part(xquat[1], xquat[2], xquat[3]);
    // Some bodies such as the world body don't have mapped USD prims to
    // transform.
    if (path.IsEmpty()) {
      continue;
    }

    pxr::SdfPrimSpecHandle prim_spec = pxr::SdfCreatePrimInLayer(layer, path);

    // Find or create transform attribute.
    pxr::SdfAttributeSpecHandle xform_attr = GetOrCreateAttribute(
        layer, prim_spec, path, pxr::_layer_sink_tokens->xformOpTransform,
        pxr::SdfValueTypeNames->Matrix4d, pxr::SdfVariabilityVarying);

    pxr::GfQuaternion quat(xquat[0], imag_part);
    pxr::GfRotation rotation(quat);

    // Identities
    pxr::GfVec3d scale(1.0);
    pxr::GfVec3d pivot_position(0);
    pxr::GfRotation pivot_rotation(pxr::GfQuaternion::GetIdentity());

    // Set time sample
    pxr::GfTransform xform(translation, rotation, scale, pivot_position,
                           pivot_rotation);
    pxr::GfMatrix4d parent_xform =
        xform_cache.GetParentToWorldTransform(stage_->GetPrimAtPath(path));
    pxr::GfMatrix4d relative_xform =
        xform.GetMatrix() * parent_xform.GetInverse();
    layer->SetTimeSample(xform_attr->GetPath(), timecode.GetValue(),
                         relative_xform);

    // Create xformop_order attribute if missing.
    pxr::SdfAttributeSpecHandle op_order_attr = GetOrCreateAttribute(
        layer, prim_spec, path, pxr::UsdGeomTokens->xformOpOrder,
        pxr::SdfValueTypeNames->TokenArray, pxr::SdfVariabilityUniform);

    // Override all local transforms so that this is the absolute position.
    // We don't reset the xform stack here, so parent transforms will still
    // affect the world pose.
    const pxr::VtTokenArray xformop_order(
        {pxr::_layer_sink_tokens->xformOpTransform});

    op_order_attr->SetDefaultValue(pxr::VtValue(xformop_order));
  }
}
}  // namespace usd
}  // namespace mujoco
