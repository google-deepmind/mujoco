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

#include "./actuatorAPI.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsActuatorAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsActuatorAPI::~MjcPhysicsActuatorAPI() {}

/* static */
MjcPhysicsActuatorAPI MjcPhysicsActuatorAPI::Get(const UsdStagePtr &stage,
                                                 const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsActuatorAPI();
  }
  return MjcPhysicsActuatorAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsActuatorAPI::_GetSchemaKind() const {
  return MjcPhysicsActuatorAPI::schemaKind;
}

/* static */
bool MjcPhysicsActuatorAPI::CanApply(const UsdPrim &prim, std::string *whyNot) {
  return prim.CanApplyAPI<MjcPhysicsActuatorAPI>(whyNot);
}

/* static */
MjcPhysicsActuatorAPI MjcPhysicsActuatorAPI::Apply(const UsdPrim &prim) {
  if (prim.ApplyAPI<MjcPhysicsActuatorAPI>()) {
    return MjcPhysicsActuatorAPI(prim);
  }
  return MjcPhysicsActuatorAPI();
}

/* static */
const TfType &MjcPhysicsActuatorAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsActuatorAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsActuatorAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsActuatorAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcCtrlLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlLimited);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcCtrlLimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcForceLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceLimited);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcForceLimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcActLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActLimited);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcActLimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcCtrlRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlRangeMin);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcCtrlRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcCtrlRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlRangeMax);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcCtrlRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcForceRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceRangeMin);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcForceRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcForceRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceRangeMax);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcForceRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcActRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActRangeMin);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcActRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcActRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActRangeMax);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcActRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcLengthRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLengthRangeMin);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcLengthRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLengthRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcLengthRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLengthRangeMax);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcLengthRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLengthRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcGearAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGear);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcGearAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGear, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcCrankLengthAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCrankLength);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcCrankLengthAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCrankLength, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcJointInParentAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcJointInParent);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcJointInParentAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcJointInParent, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcActDimAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActDim);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcActDimAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActDim, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcDynTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDynType);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcDynTypeAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDynType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcGainTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGainType);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcGainTypeAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGainType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcBiasTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcBiasType);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcBiasTypeAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcBiasType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcDynPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDynPrm);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcDynPrmAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDynPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcGainPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGainPrm);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcGainPrmAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGainPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcBiasPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcBiasPrm);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcBiasPrmAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcBiasPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuatorAPI::GetMjcActEarlyAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActEarly);
}

UsdAttribute MjcPhysicsActuatorAPI::CreateMjcActEarlyAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActEarly, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdRelationship MjcPhysicsActuatorAPI::GetMjcRefSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcRefSite);
}

UsdRelationship MjcPhysicsActuatorAPI::CreateMjcRefSiteRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcRefSite,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsActuatorAPI::GetMjcCrankSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcCrankSite);
}

UsdRelationship MjcPhysicsActuatorAPI::CreateMjcCrankSiteRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcCrankSite,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsActuatorAPI::GetMjcSliderSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcSliderSite);
}

UsdRelationship MjcPhysicsActuatorAPI::CreateMjcSliderSiteRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcSliderSite,
                                      /* custom = */ false);
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
const TfTokenVector &MjcPhysicsActuatorAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcCtrlLimited,    MjcPhysicsTokens->mjcForceLimited,
      MjcPhysicsTokens->mjcActLimited,     MjcPhysicsTokens->mjcCtrlRangeMin,
      MjcPhysicsTokens->mjcCtrlRangeMax,   MjcPhysicsTokens->mjcForceRangeMin,
      MjcPhysicsTokens->mjcForceRangeMax,  MjcPhysicsTokens->mjcActRangeMin,
      MjcPhysicsTokens->mjcActRangeMax,    MjcPhysicsTokens->mjcLengthRangeMin,
      MjcPhysicsTokens->mjcLengthRangeMax, MjcPhysicsTokens->mjcGear,
      MjcPhysicsTokens->mjcCrankLength,    MjcPhysicsTokens->mjcJointInParent,
      MjcPhysicsTokens->mjcActDim,         MjcPhysicsTokens->mjcDynType,
      MjcPhysicsTokens->mjcGainType,       MjcPhysicsTokens->mjcBiasType,
      MjcPhysicsTokens->mjcDynPrm,         MjcPhysicsTokens->mjcGainPrm,
      MjcPhysicsTokens->mjcBiasPrm,        MjcPhysicsTokens->mjcActEarly,
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
