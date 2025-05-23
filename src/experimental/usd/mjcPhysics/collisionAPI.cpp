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

#include "./collisionAPI.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsCollisionAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsCollisionAPI::~MjcPhysicsCollisionAPI() {}

/* static */
MjcPhysicsCollisionAPI MjcPhysicsCollisionAPI::Get(const UsdStagePtr &stage,
                                                   const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsCollisionAPI();
  }
  return MjcPhysicsCollisionAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsCollisionAPI::_GetSchemaKind() const {
  return MjcPhysicsCollisionAPI::schemaKind;
}

/* static */
bool MjcPhysicsCollisionAPI::CanApply(const UsdPrim &prim,
                                      std::string *whyNot) {
  return prim.CanApplyAPI<MjcPhysicsCollisionAPI>(whyNot);
}

/* static */
MjcPhysicsCollisionAPI MjcPhysicsCollisionAPI::Apply(const UsdPrim &prim) {
  if (prim.ApplyAPI<MjcPhysicsCollisionAPI>()) {
    return MjcPhysicsCollisionAPI(prim);
  }
  return MjcPhysicsCollisionAPI();
}

/* static */
const TfType &MjcPhysicsCollisionAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsCollisionAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsCollisionAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsCollisionAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsCollisionAPI::GetShellInertiaAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcShellinertia);
}

UsdAttribute MjcPhysicsCollisionAPI::CreateShellInertiaAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcShellinertia, SdfValueTypeNames->Bool,
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
const TfTokenVector &MjcPhysicsCollisionAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcShellinertia,
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
