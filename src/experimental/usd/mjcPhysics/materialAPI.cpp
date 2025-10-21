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

#include <mujoco/experimental/usd/mjcPhysics/materialAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsMaterialAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsMaterialAPI::~MjcPhysicsMaterialAPI() {}

/* static */
MjcPhysicsMaterialAPI MjcPhysicsMaterialAPI::Get(const UsdStagePtr& stage,
                                                 const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsMaterialAPI();
  }
  return MjcPhysicsMaterialAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsMaterialAPI::_GetSchemaKind() const {
  return MjcPhysicsMaterialAPI::schemaKind;
}

/* static */
bool MjcPhysicsMaterialAPI::CanApply(const UsdPrim& prim, std::string* whyNot) {
  return prim.CanApplyAPI<MjcPhysicsMaterialAPI>(whyNot);
}

/* static */
MjcPhysicsMaterialAPI MjcPhysicsMaterialAPI::Apply(const UsdPrim& prim) {
  if (prim.ApplyAPI<MjcPhysicsMaterialAPI>()) {
    return MjcPhysicsMaterialAPI(prim);
  }
  return MjcPhysicsMaterialAPI();
}

/* static */
const TfType& MjcPhysicsMaterialAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsMaterialAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsMaterialAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsMaterialAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsMaterialAPI::GetTorsionalFrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcTorsionalfriction);
}

UsdAttribute MjcPhysicsMaterialAPI::CreateTorsionalFrictionAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcTorsionalfriction, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsMaterialAPI::GetRollingFrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcRollingfriction);
}

UsdAttribute MjcPhysicsMaterialAPI::CreateRollingFrictionAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcRollingfriction, SdfValueTypeNames->Double,
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
const TfTokenVector& MjcPhysicsMaterialAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcTorsionalfriction,
      MjcPhysicsTokens->mjcRollingfriction,
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
