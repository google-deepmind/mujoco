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

#include "./siteAPI.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsSiteAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsSiteAPI::~MjcPhysicsSiteAPI() {}

/* static */
MjcPhysicsSiteAPI MjcPhysicsSiteAPI::Get(const UsdStagePtr &stage,
                                         const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsSiteAPI();
  }
  return MjcPhysicsSiteAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsSiteAPI::_GetSchemaKind() const {
  return MjcPhysicsSiteAPI::schemaKind;
}

/* static */
bool MjcPhysicsSiteAPI::CanApply(const UsdPrim &prim, std::string *whyNot) {
  return prim.CanApplyAPI<MjcPhysicsSiteAPI>(whyNot);
}

/* static */
MjcPhysicsSiteAPI MjcPhysicsSiteAPI::Apply(const UsdPrim &prim) {
  if (prim.ApplyAPI<MjcPhysicsSiteAPI>()) {
    return MjcPhysicsSiteAPI(prim);
  }
  return MjcPhysicsSiteAPI();
}

/* static */
const TfType &MjcPhysicsSiteAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsSiteAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsSiteAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsSiteAPI::_GetTfType() const {
  return _GetStaticTfType();
}

/*static*/
const TfTokenVector &MjcPhysicsSiteAPI::GetSchemaAttributeNames(
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
