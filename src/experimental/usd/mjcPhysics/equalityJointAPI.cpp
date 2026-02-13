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

#include <mujoco/experimental/usd/mjcPhysics/equalityJointAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsEqualityJointAPI,
                 TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsEqualityJointAPI::~MjcPhysicsEqualityJointAPI() {}

/* static */
MjcPhysicsEqualityJointAPI MjcPhysicsEqualityJointAPI::Get(
    const UsdStagePtr& stage, const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsEqualityJointAPI();
  }
  return MjcPhysicsEqualityJointAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsEqualityJointAPI::_GetSchemaKind() const {
  return MjcPhysicsEqualityJointAPI::schemaKind;
}

/* static */
bool MjcPhysicsEqualityJointAPI::CanApply(const UsdPrim& prim,
                                          std::string* whyNot) {
  return prim.CanApplyAPI<MjcPhysicsEqualityJointAPI>(whyNot);
}

/* static */
MjcPhysicsEqualityJointAPI MjcPhysicsEqualityJointAPI::Apply(
    const UsdPrim& prim) {
  if (prim.ApplyAPI<MjcPhysicsEqualityJointAPI>()) {
    return MjcPhysicsEqualityJointAPI(prim);
  }
  return MjcPhysicsEqualityJointAPI();
}

/* static */
const TfType& MjcPhysicsEqualityJointAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsEqualityJointAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsEqualityJointAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsEqualityJointAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsEqualityJointAPI::GetCoef0Attr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCoef0);
}

UsdAttribute MjcPhysicsEqualityJointAPI::CreateCoef0Attr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCoef0, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsEqualityJointAPI::GetCoef1Attr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCoef1);
}

UsdAttribute MjcPhysicsEqualityJointAPI::CreateCoef1Attr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCoef1, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsEqualityJointAPI::GetCoef2Attr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCoef2);
}

UsdAttribute MjcPhysicsEqualityJointAPI::CreateCoef2Attr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCoef2, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsEqualityJointAPI::GetCoef3Attr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCoef3);
}

UsdAttribute MjcPhysicsEqualityJointAPI::CreateCoef3Attr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCoef3, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsEqualityJointAPI::GetCoef4Attr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCoef4);
}

UsdAttribute MjcPhysicsEqualityJointAPI::CreateCoef4Attr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCoef4, SdfValueTypeNames->Double,
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
const TfTokenVector& MjcPhysicsEqualityJointAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcCoef0, MjcPhysicsTokens->mjcCoef1,
      MjcPhysicsTokens->mjcCoef2, MjcPhysicsTokens->mjcCoef3,
      MjcPhysicsTokens->mjcCoef4,
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
