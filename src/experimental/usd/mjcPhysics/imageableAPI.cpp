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

#include <mujoco/experimental/usd/mjcPhysics/imageableAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsImageableAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsImageableAPI::~MjcPhysicsImageableAPI() {}

/* static */
MjcPhysicsImageableAPI MjcPhysicsImageableAPI::Get(const UsdStagePtr& stage,
                                                   const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsImageableAPI();
  }
  return MjcPhysicsImageableAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsImageableAPI::_GetSchemaKind() const {
  return MjcPhysicsImageableAPI::schemaKind;
}

/* static */
bool MjcPhysicsImageableAPI::CanApply(const UsdPrim& prim,
                                      std::string* whyNot) {
  return prim.CanApplyAPI<MjcPhysicsImageableAPI>(whyNot);
}

/* static */
MjcPhysicsImageableAPI MjcPhysicsImageableAPI::Apply(const UsdPrim& prim) {
  if (prim.ApplyAPI<MjcPhysicsImageableAPI>()) {
    return MjcPhysicsImageableAPI(prim);
  }
  return MjcPhysicsImageableAPI();
}

/* static */
const TfType& MjcPhysicsImageableAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsImageableAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsImageableAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsImageableAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsImageableAPI::GetGroupAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGroup);
}

UsdAttribute MjcPhysicsImageableAPI::CreateGroupAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGroup, SdfValueTypeNames->Int,
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
const TfTokenVector& MjcPhysicsImageableAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcGroup,
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
