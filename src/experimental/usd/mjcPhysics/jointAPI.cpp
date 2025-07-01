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

#include <mujoco/experimental/usd/mjcPhysics/jointAPI.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsJointAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsJointAPI::~MjcPhysicsJointAPI() {}

/* static */
MjcPhysicsJointAPI MjcPhysicsJointAPI::Get(const UsdStagePtr &stage,
                                           const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsJointAPI();
  }
  return MjcPhysicsJointAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsJointAPI::_GetSchemaKind() const {
  return MjcPhysicsJointAPI::schemaKind;
}

/* static */
bool MjcPhysicsJointAPI::CanApply(const UsdPrim &prim, std::string *whyNot) {
  return prim.CanApplyAPI<MjcPhysicsJointAPI>(whyNot);
}

/* static */
MjcPhysicsJointAPI MjcPhysicsJointAPI::Apply(const UsdPrim &prim) {
  if (prim.ApplyAPI<MjcPhysicsJointAPI>()) {
    return MjcPhysicsJointAPI(prim);
  }
  return MjcPhysicsJointAPI();
}

/* static */
const TfType &MjcPhysicsJointAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsJointAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsJointAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsJointAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsJointAPI::GetMjcSpringdamperAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSpringdamper);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcSpringdamperAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSpringdamper, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcSolreflimitAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolreflimit);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcSolreflimitAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolreflimit, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcSolimplimitAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolimplimit);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcSolimplimitAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolimplimit, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcSolreffrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolreffriction);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcSolreffrictionAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolreffriction, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcSolimpfrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolimpfriction);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcSolimpfrictionAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolimpfriction, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcStiffnessAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcStiffness);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcStiffnessAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcStiffness, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcActuatorfrcrangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorfrcrangeMin);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcActuatorfrcrangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorfrcrangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcActuatorfrcrangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorfrcrangeMax);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcActuatorfrcrangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorfrcrangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcActuatorfrclimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorfrclimited);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcActuatorfrclimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorfrclimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcActuatorgravcompAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorgravcomp);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcActuatorgravcompAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorgravcomp, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcMarginAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcMargin);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcMarginAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcMargin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcRefAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcRef);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcRefAttr(VtValue const &defaultValue,
                                                  bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcRef, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcSpringrefAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSpringref);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcSpringrefAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSpringref, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcArmatureAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcArmature);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcArmatureAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcArmature, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcDampingAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDamping);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcDampingAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDamping, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsJointAPI::GetMjcFrictionlossAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFrictionloss);
}

UsdAttribute MjcPhysicsJointAPI::CreateMjcFrictionlossAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFrictionloss, SdfValueTypeNames->Double,
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
const TfTokenVector &MjcPhysicsJointAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcSpringdamper,
      MjcPhysicsTokens->mjcSolreflimit,
      MjcPhysicsTokens->mjcSolimplimit,
      MjcPhysicsTokens->mjcSolreffriction,
      MjcPhysicsTokens->mjcSolimpfriction,
      MjcPhysicsTokens->mjcStiffness,
      MjcPhysicsTokens->mjcActuatorfrcrangeMin,
      MjcPhysicsTokens->mjcActuatorfrcrangeMax,
      MjcPhysicsTokens->mjcActuatorfrclimited,
      MjcPhysicsTokens->mjcActuatorgravcomp,
      MjcPhysicsTokens->mjcMargin,
      MjcPhysicsTokens->mjcRef,
      MjcPhysicsTokens->mjcSpringref,
      MjcPhysicsTokens->mjcArmature,
      MjcPhysicsTokens->mjcDamping,
      MjcPhysicsTokens->mjcFrictionloss,
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
