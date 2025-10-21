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

#include <mujoco/experimental/usd/mjcPhysics/actuator.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsActuator, TfType::Bases<UsdTyped> >();

  // Register the usd prim typename as an alias under UsdSchemaBase. This
  // enables one to call
  // TfType::Find<UsdSchemaBase>().FindDerivedByName("MjcActuator")
  // to find TfType<MjcPhysicsActuator>, which is how IsA queries are
  // answered.
  TfType::AddAlias<UsdSchemaBase, MjcPhysicsActuator>("MjcActuator");
}

/* virtual */
MjcPhysicsActuator::~MjcPhysicsActuator() {}

/* static */
MjcPhysicsActuator MjcPhysicsActuator::Get(const UsdStagePtr& stage,
                                           const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsActuator();
  }
  return MjcPhysicsActuator(stage->GetPrimAtPath(path));
}

/* static */
MjcPhysicsActuator MjcPhysicsActuator::Define(const UsdStagePtr& stage,
                                              const SdfPath& path) {
  static TfToken usdPrimTypeName("MjcActuator");
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsActuator();
  }
  return MjcPhysicsActuator(stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind MjcPhysicsActuator::_GetSchemaKind() const {
  return MjcPhysicsActuator::schemaKind;
}

/* static */
const TfType& MjcPhysicsActuator::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsActuator>();
  return tfType;
}

/* static */
bool MjcPhysicsActuator::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsActuator::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsActuator::GetGroupAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGroup);
}

UsdAttribute MjcPhysicsActuator::CreateGroupAttr(VtValue const& defaultValue,
                                                 bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGroup, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcCtrlLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlLimited);
}

UsdAttribute MjcPhysicsActuator::CreateMjcCtrlLimitedAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcForceLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceLimited);
}

UsdAttribute MjcPhysicsActuator::CreateMjcForceLimitedAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcActLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActLimited);
}

UsdAttribute MjcPhysicsActuator::CreateMjcActLimitedAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcCtrlRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlRangeMin);
}

UsdAttribute MjcPhysicsActuator::CreateMjcCtrlRangeMinAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcCtrlRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCtrlRangeMax);
}

UsdAttribute MjcPhysicsActuator::CreateMjcCtrlRangeMaxAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCtrlRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcForceRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceRangeMin);
}

UsdAttribute MjcPhysicsActuator::CreateMjcForceRangeMinAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcForceRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcForceRangeMax);
}

UsdAttribute MjcPhysicsActuator::CreateMjcForceRangeMaxAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcForceRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcActRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActRangeMin);
}

UsdAttribute MjcPhysicsActuator::CreateMjcActRangeMinAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcActRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActRangeMax);
}

UsdAttribute MjcPhysicsActuator::CreateMjcActRangeMaxAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcLengthRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLengthRangeMin);
}

UsdAttribute MjcPhysicsActuator::CreateMjcLengthRangeMinAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLengthRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcLengthRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLengthRangeMax);
}

UsdAttribute MjcPhysicsActuator::CreateMjcLengthRangeMaxAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLengthRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcGearAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGear);
}

UsdAttribute MjcPhysicsActuator::CreateMjcGearAttr(VtValue const& defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGear, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcCrankLengthAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcCrankLength);
}

UsdAttribute MjcPhysicsActuator::CreateMjcCrankLengthAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcCrankLength, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcJointInParentAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcJointInParent);
}

UsdAttribute MjcPhysicsActuator::CreateMjcJointInParentAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcJointInParent, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcActDimAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActDim);
}

UsdAttribute MjcPhysicsActuator::CreateMjcActDimAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActDim, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcDynTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDynType);
}

UsdAttribute MjcPhysicsActuator::CreateMjcDynTypeAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDynType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcGainTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGainType);
}

UsdAttribute MjcPhysicsActuator::CreateMjcGainTypeAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGainType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcBiasTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcBiasType);
}

UsdAttribute MjcPhysicsActuator::CreateMjcBiasTypeAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcBiasType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcDynPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDynPrm);
}

UsdAttribute MjcPhysicsActuator::CreateMjcDynPrmAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDynPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcGainPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGainPrm);
}

UsdAttribute MjcPhysicsActuator::CreateMjcGainPrmAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGainPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcBiasPrmAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcBiasPrm);
}

UsdAttribute MjcPhysicsActuator::CreateMjcBiasPrmAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcBiasPrm, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcActEarlyAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActEarly);
}

UsdAttribute MjcPhysicsActuator::CreateMjcActEarlyAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActEarly, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsActuator::GetMjcInheritRangeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcInheritRange);
}

UsdAttribute MjcPhysicsActuator::CreateMjcInheritRangeAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcInheritRange, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdRelationship MjcPhysicsActuator::GetMjcTargetRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcTarget);
}

UsdRelationship MjcPhysicsActuator::CreateMjcTargetRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcTarget,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsActuator::GetMjcRefSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcRefSite);
}

UsdRelationship MjcPhysicsActuator::CreateMjcRefSiteRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcRefSite,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsActuator::GetMjcSliderSiteRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcSliderSite);
}

UsdRelationship MjcPhysicsActuator::CreateMjcSliderSiteRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcSliderSite,
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
const TfTokenVector& MjcPhysicsActuator::GetSchemaAttributeNames(
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
      MjcPhysicsTokens->mjcActEarly,       MjcPhysicsTokens->mjcInheritRange,
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
