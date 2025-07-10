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

#include <mujoco/experimental/usd/mjcPhysics/transmission.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsTransmission, TfType::Bases<UsdTyped> >();

  // Register the usd prim typename as an alias under UsdSchemaBase. This
  // enables one to call
  // TfType::Find<UsdSchemaBase>().FindDerivedByName("MjcTransmission")
  // to find TfType<MjcPhysicsTransmission>, which is how IsA queries are
  // answered.
  TfType::AddAlias<UsdSchemaBase, MjcPhysicsTransmission>("MjcTransmission");
}

/* virtual */
MjcPhysicsTransmission::~MjcPhysicsTransmission() {}

/* static */
MjcPhysicsTransmission MjcPhysicsTransmission::Get(const UsdStagePtr &stage,
                                                   const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsTransmission();
  }
  return MjcPhysicsTransmission(stage->GetPrimAtPath(path));
}

/* static */
MjcPhysicsTransmission MjcPhysicsTransmission::Define(const UsdStagePtr &stage,
                                                      const SdfPath &path) {
  static TfToken usdPrimTypeName("MjcTransmission");
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsTransmission();
  }
  return MjcPhysicsTransmission(stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind MjcPhysicsTransmission::_GetSchemaKind() const {
  return MjcPhysicsTransmission::schemaKind;
}

/* static */
const TfType &MjcPhysicsTransmission::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsTransmission>();
  return tfType;
}

/* static */
bool MjcPhysicsTransmission::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsTransmission::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsTransmission::GetGroupAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGroup);
}

UsdAttribute MjcPhysicsTransmission::CreateGroupAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGroup, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcCtrlLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlLimited);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcCtrlLimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcForceLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceLimited);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcForceLimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcActLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActLimited);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcActLimitedAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcCtrlRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlRangeMin);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcCtrlRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcCtrlRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlRangeMax);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcCtrlRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcForceRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceRangeMin);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcForceRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcForceRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceRangeMax);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcForceRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcActRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActRangeMin);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcActRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcActRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActRangeMax);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcActRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcLengthRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLengthRangeMin);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcLengthRangeMinAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLengthRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcLengthRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLengthRangeMax);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcLengthRangeMaxAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLengthRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcGearAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGear);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcGearAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGear, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcCrankLengthAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCrankLength);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcCrankLengthAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCrankLength, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcJointInParentAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcJointInParent);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcJointInParentAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcJointInParent, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcActDimAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActDim);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcActDimAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActDim, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcDynTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDynType);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcDynTypeAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDynType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcGainTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGainType);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcGainTypeAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGainType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcBiasTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcBiasType);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcBiasTypeAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcBiasType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcDynPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDynPrm);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcDynPrmAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDynPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcGainPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGainPrm);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcGainPrmAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGainPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcBiasPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcBiasPrm);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcBiasPrmAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcBiasPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTransmission::GetMjcActEarlyAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActEarly);
}

UsdAttribute MjcPhysicsTransmission::CreateMjcActEarlyAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActEarly, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdRelationship MjcPhysicsTransmission::GetMjcTargetRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcTarget);
}

UsdRelationship MjcPhysicsTransmission::CreateMjcTargetRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcTarget,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsTransmission::GetMjcRefSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcRefSite);
}

UsdRelationship MjcPhysicsTransmission::CreateMjcRefSiteRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcRefSite,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsTransmission::GetMjcSliderSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcSliderSite);
}

UsdRelationship MjcPhysicsTransmission::CreateMjcSliderSiteRel() const {
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
const TfTokenVector &MjcPhysicsTransmission::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcGroup,          MjcPhysicsTokens->mjcCtrlLimited,
      MjcPhysicsTokens->mjcForceLimited,   MjcPhysicsTokens->mjcActLimited,
      MjcPhysicsTokens->mjcCtrlRangeMin,   MjcPhysicsTokens->mjcCtrlRangeMax,
      MjcPhysicsTokens->mjcForceRangeMin,  MjcPhysicsTokens->mjcForceRangeMax,
      MjcPhysicsTokens->mjcActRangeMin,    MjcPhysicsTokens->mjcActRangeMax,
      MjcPhysicsTokens->mjcLengthRangeMin, MjcPhysicsTokens->mjcLengthRangeMax,
      MjcPhysicsTokens->mjcGear,           MjcPhysicsTokens->mjcCrankLength,
      MjcPhysicsTokens->mjcJointInParent,  MjcPhysicsTokens->mjcActDim,
      MjcPhysicsTokens->mjcDynType,        MjcPhysicsTokens->mjcGainType,
      MjcPhysicsTokens->mjcBiasType,       MjcPhysicsTokens->mjcDynPrm,
      MjcPhysicsTokens->mjcGainPrm,        MjcPhysicsTokens->mjcBiasPrm,
      MjcPhysicsTokens->mjcActEarly,
  };
  static TfTokenVector allNames = _ConcatenateAttributeNames(
      UsdTyped::GetSchemaAttributeNames(true), localNames);

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
