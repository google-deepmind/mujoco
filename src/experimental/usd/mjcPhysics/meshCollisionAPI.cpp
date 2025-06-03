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

#include "./meshCollisionAPI.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsMeshCollisionAPI,
                 TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsMeshCollisionAPI::~MjcPhysicsMeshCollisionAPI() {}

/* static */
MjcPhysicsMeshCollisionAPI MjcPhysicsMeshCollisionAPI::Get(
    const UsdStagePtr &stage, const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsMeshCollisionAPI();
  }
  return MjcPhysicsMeshCollisionAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsMeshCollisionAPI::_GetSchemaKind() const {
  return MjcPhysicsMeshCollisionAPI::schemaKind;
}

/* static */
bool MjcPhysicsMeshCollisionAPI::CanApply(const UsdPrim &prim,
                                          std::string *whyNot) {
  return prim.CanApplyAPI<MjcPhysicsMeshCollisionAPI>(whyNot);
}

/* static */
MjcPhysicsMeshCollisionAPI MjcPhysicsMeshCollisionAPI::Apply(
    const UsdPrim &prim) {
  if (prim.ApplyAPI<MjcPhysicsMeshCollisionAPI>()) {
    return MjcPhysicsMeshCollisionAPI(prim);
  }
  return MjcPhysicsMeshCollisionAPI();
}

/* static */
const TfType &MjcPhysicsMeshCollisionAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsMeshCollisionAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsMeshCollisionAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsMeshCollisionAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsMeshCollisionAPI::GetInertiaAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcInertia);
}

UsdAttribute MjcPhysicsMeshCollisionAPI::CreateInertiaAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcInertia, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

namespace {
static inline TfTokenVector _ConcatenateAttributeNames(
    const TfTokenVector &left, const TfTokenVector &right) {
  TfTokenVector result;
  result.reserve(left.size() + right.size());
  result.insert(result.end(), left.begin(), left.end());
  result.insert(result.end(), right.begin(), right.end());
  return result;
}
}  // namespace

/*static*/
const TfTokenVector &MjcPhysicsMeshCollisionAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcInertia,
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
