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

#include <mujoco/experimental/usd/mjcPhysics/equalityConnectAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsEqualityConnectAPI,
                 TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsEqualityConnectAPI::~MjcPhysicsEqualityConnectAPI() {}

/* static */
MjcPhysicsEqualityConnectAPI MjcPhysicsEqualityConnectAPI::Get(
    const UsdStagePtr& stage, const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsEqualityConnectAPI();
  }
  return MjcPhysicsEqualityConnectAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsEqualityConnectAPI::_GetSchemaKind() const {
  return MjcPhysicsEqualityConnectAPI::schemaKind;
}

/* static */
bool MjcPhysicsEqualityConnectAPI::CanApply(const UsdPrim& prim,
                                            std::string* whyNot) {
  return prim.CanApplyAPI<MjcPhysicsEqualityConnectAPI>(whyNot);
}

/* static */
MjcPhysicsEqualityConnectAPI MjcPhysicsEqualityConnectAPI::Apply(
    const UsdPrim& prim) {
  if (prim.ApplyAPI<MjcPhysicsEqualityConnectAPI>()) {
    return MjcPhysicsEqualityConnectAPI(prim);
  }
  return MjcPhysicsEqualityConnectAPI();
}

/* static */
const TfType& MjcPhysicsEqualityConnectAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsEqualityConnectAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsEqualityConnectAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsEqualityConnectAPI::_GetTfType() const {
  return _GetStaticTfType();
}

/*static*/
const TfTokenVector& MjcPhysicsEqualityConnectAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames;
  static TfTokenVector allNames =
      UsdAPISchemaBase::GetSchemaAttributeNames(true);

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
