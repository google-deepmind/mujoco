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

#include <mujoco/experimental/usd/mjcPhysics/equalityAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsEqualityAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsEqualityAPI::~MjcPhysicsEqualityAPI() {}

/* static */
MjcPhysicsEqualityAPI MjcPhysicsEqualityAPI::Get(const UsdStagePtr& stage,
                                                 const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsEqualityAPI();
  }
  return MjcPhysicsEqualityAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsEqualityAPI::_GetSchemaKind() const {
  return MjcPhysicsEqualityAPI::schemaKind;
}

/* static */
bool MjcPhysicsEqualityAPI::CanApply(const UsdPrim& prim, std::string* whyNot) {
  return prim.CanApplyAPI<MjcPhysicsEqualityAPI>(whyNot);
}

/* static */
MjcPhysicsEqualityAPI MjcPhysicsEqualityAPI::Apply(const UsdPrim& prim) {
  if (prim.ApplyAPI<MjcPhysicsEqualityAPI>()) {
    return MjcPhysicsEqualityAPI(prim);
  }
  return MjcPhysicsEqualityAPI();
}

/* static */
const TfType& MjcPhysicsEqualityAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsEqualityAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsEqualityAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsEqualityAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsEqualityAPI::GetSolRefAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolref);
}

UsdAttribute MjcPhysicsEqualityAPI::CreateSolRefAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolref, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsEqualityAPI::GetSolImpAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolimp);
}

UsdAttribute MjcPhysicsEqualityAPI::CreateSolImpAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolimp, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdRelationship MjcPhysicsEqualityAPI::GetMjcTargetRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcTarget);
}

UsdRelationship MjcPhysicsEqualityAPI::CreateMjcTargetRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcTarget,
                                      /* custom = */ false);
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
const TfTokenVector& MjcPhysicsEqualityAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcSolref,
      MjcPhysicsTokens->mjcSolimp,
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
