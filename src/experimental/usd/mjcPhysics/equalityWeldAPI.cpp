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

#include <mujoco/experimental/usd/mjcPhysics/equalityWeldAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsEqualityWeldAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsEqualityWeldAPI::~MjcPhysicsEqualityWeldAPI() {}

/* static */
MjcPhysicsEqualityWeldAPI MjcPhysicsEqualityWeldAPI::Get(
    const UsdStagePtr& stage, const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsEqualityWeldAPI();
  }
  return MjcPhysicsEqualityWeldAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsEqualityWeldAPI::_GetSchemaKind() const {
  return MjcPhysicsEqualityWeldAPI::schemaKind;
}

/* static */
bool MjcPhysicsEqualityWeldAPI::CanApply(const UsdPrim& prim,
                                         std::string* whyNot) {
  return prim.CanApplyAPI<MjcPhysicsEqualityWeldAPI>(whyNot);
}

/* static */
MjcPhysicsEqualityWeldAPI MjcPhysicsEqualityWeldAPI::Apply(
    const UsdPrim& prim) {
  if (prim.ApplyAPI<MjcPhysicsEqualityWeldAPI>()) {
    return MjcPhysicsEqualityWeldAPI(prim);
  }
  return MjcPhysicsEqualityWeldAPI();
}

/* static */
const TfType& MjcPhysicsEqualityWeldAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsEqualityWeldAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsEqualityWeldAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsEqualityWeldAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsEqualityWeldAPI::GetTorqueScaleAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcTorqueScale);
}

UsdAttribute MjcPhysicsEqualityWeldAPI::CreateTorqueScaleAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcTorqueScale, SdfValueTypeNames->Float,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

namespace {
static inline TfTokenVector _ConcatenateAttributeNames(
    const TfTokenVector& left, const TfTokenVector& right) {
  TfTokenVector result;
  result.reserve(left.size() + right.size());
  result.insert(result.end(), left.begin(), left.end());
  result.insert(result.end(), right.begin(), right.end());
  return result;
}
}  // namespace

/*static*/
const TfTokenVector& MjcPhysicsEqualityWeldAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcTorqueScale,
  };
  static TfTokenVector allNames = _ConcatenateAttributeNames(
      UsdAPISchemaBase::GetSchemaAttributeNames(true), localNames);

  if (includeInherited)
    return allNames;
  else
    return localNames;
}

PXR_NAMESPACE_CLOSE_SCOPE

// ===================================================================== //
// Feel free to add custom code below this line. It will be preserved by
// the code generator.
//
// Just remember to wrap code in the appropriate delimiters:
// 'PXR_NAMESPACE_OPEN_SCOPE', 'PXR_NAMESPACE_CLOSE_SCOPE'.
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--
