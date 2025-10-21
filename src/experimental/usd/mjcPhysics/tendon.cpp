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

#include <mujoco/experimental/usd/mjcPhysics/tendon.h>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsTendon, TfType::Bases<UsdTyped> >();

  // Register the usd prim typename as an alias under UsdSchemaBase. This
  // enables one to call
  // TfType::Find<UsdSchemaBase>().FindDerivedByName("MjcTendon")
  // to find TfType<MjcPhysicsTendon>, which is how IsA queries are
  // answered.
  TfType::AddAlias<UsdSchemaBase, MjcPhysicsTendon>("MjcTendon");
}

/* virtual */
MjcPhysicsTendon::~MjcPhysicsTendon() {}

/* static */
MjcPhysicsTendon MjcPhysicsTendon::Get(const UsdStagePtr& stage,
                                       const SdfPath& path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsTendon();
  }
  return MjcPhysicsTendon(stage->GetPrimAtPath(path));
}

/* static */
MjcPhysicsTendon MjcPhysicsTendon::Define(const UsdStagePtr& stage,
                                          const SdfPath& path) {
  static TfToken usdPrimTypeName("MjcTendon");
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsTendon();
  }
  return MjcPhysicsTendon(stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind MjcPhysicsTendon::_GetSchemaKind() const {
  return MjcPhysicsTendon::schemaKind;
}

/* static */
const TfType& MjcPhysicsTendon::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsTendon>();
  return tfType;
}

/* static */
bool MjcPhysicsTendon::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType& MjcPhysicsTendon::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsTendon::GetTypeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcType);
}

UsdAttribute MjcPhysicsTendon::CreateTypeAttr(VtValue const& defaultValue,
                                              bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcType, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetMjcSideSitesIndicesAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSideSitesIndices);
}

UsdAttribute MjcPhysicsTendon::CreateMjcSideSitesIndicesAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSideSitesIndices, SdfValueTypeNames->IntArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetMjcPathSegmentsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPathSegments);
}

UsdAttribute MjcPhysicsTendon::CreateMjcPathSegmentsAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPathSegments, SdfValueTypeNames->IntArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetMjcPathDivisorsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPathDivisors);
}

UsdAttribute MjcPhysicsTendon::CreateMjcPathDivisorsAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPathDivisors, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetMjcPathCoefAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPathCoef);
}

UsdAttribute MjcPhysicsTendon::CreateMjcPathCoefAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPathCoef, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetGroupAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcGroup);
}

UsdAttribute MjcPhysicsTendon::CreateGroupAttr(VtValue const& defaultValue,
                                               bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcGroup, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcLimited);
}

UsdAttribute MjcPhysicsTendon::CreateLimitedAttr(VtValue const& defaultValue,
                                                 bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcLimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetActuatorFrcLimitedAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorfrclimited);
}

UsdAttribute MjcPhysicsTendon::CreateActuatorFrcLimitedAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorfrclimited, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcRangeMin);
}

UsdAttribute MjcPhysicsTendon::CreateRangeMinAttr(VtValue const& defaultValue,
                                                  bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcRangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcRangeMax);
}

UsdAttribute MjcPhysicsTendon::CreateRangeMaxAttr(VtValue const& defaultValue,
                                                  bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcRangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetActuatorFrcRangeMinAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorfrcrangeMin);
}

UsdAttribute MjcPhysicsTendon::CreateActuatorFrcRangeMinAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorfrcrangeMin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetActuatorFrcRangeMaxAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcActuatorfrcrangeMax);
}

UsdAttribute MjcPhysicsTendon::CreateActuatorFrcRangeMaxAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcActuatorfrcrangeMax, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetSolRefLimitAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolreflimit);
}

UsdAttribute MjcPhysicsTendon::CreateSolRefLimitAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolreflimit, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetSolImpLimitAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolimplimit);
}

UsdAttribute MjcPhysicsTendon::CreateSolImpLimitAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolimplimit, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetSolRefFrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolreffriction);
}

UsdAttribute MjcPhysicsTendon::CreateSolRefFrictionAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolreffriction, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetSolImpFrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSolimpfriction);
}

UsdAttribute MjcPhysicsTendon::CreateSolImpFrictionAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSolimpfriction, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetMarginAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcMargin);
}

UsdAttribute MjcPhysicsTendon::CreateMarginAttr(VtValue const& defaultValue,
                                                bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcMargin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetFrictionLossAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFrictionloss);
}

UsdAttribute MjcPhysicsTendon::CreateFrictionLossAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFrictionloss, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetWidthAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcWidth);
}

UsdAttribute MjcPhysicsTendon::CreateWidthAttr(VtValue const& defaultValue,
                                               bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcWidth, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetRgbaAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcRgba);
}

UsdAttribute MjcPhysicsTendon::CreateRgbaAttr(VtValue const& defaultValue,
                                              bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcRgba, SdfValueTypeNames->Color4f,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetSpringLengthAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcSpringlength);
}

UsdAttribute MjcPhysicsTendon::CreateSpringLengthAttr(
    VtValue const& defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcSpringlength, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetStiffnessAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcStiffness);
}

UsdAttribute MjcPhysicsTendon::CreateStiffnessAttr(VtValue const& defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcStiffness, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetDampingAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcDamping);
}

UsdAttribute MjcPhysicsTendon::CreateDampingAttr(VtValue const& defaultValue,
                                                 bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcDamping, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsTendon::GetArmatureAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcArmature);
}

UsdAttribute MjcPhysicsTendon::CreateArmatureAttr(VtValue const& defaultValue,
                                                  bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcArmature, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdRelationship MjcPhysicsTendon::GetMjcPathRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcPath);
}

UsdRelationship MjcPhysicsTendon::CreateMjcPathRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcPath,
                                      /* custom = */ false);
}

UsdRelationship MjcPhysicsTendon::GetMjcSideSitesRel() const {
  return GetPrim().GetRelationship(MjcPhysicsTokens->mjcSideSites);
}

UsdRelationship MjcPhysicsTendon::CreateMjcSideSitesRel() const {
  return GetPrim().CreateRelationship(MjcPhysicsTokens->mjcSideSites,
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
const TfTokenVector& MjcPhysicsTendon::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcType,
      MjcPhysicsTokens->mjcSideSitesIndices,
      MjcPhysicsTokens->mjcPathSegments,
      MjcPhysicsTokens->mjcPathDivisors,
      MjcPhysicsTokens->mjcPathCoef,
      MjcPhysicsTokens->mjcGroup,
      MjcPhysicsTokens->mjcLimited,
      MjcPhysicsTokens->mjcActuatorfrclimited,
      MjcPhysicsTokens->mjcRangeMin,
      MjcPhysicsTokens->mjcRangeMax,
      MjcPhysicsTokens->mjcActuatorfrcrangeMin,
      MjcPhysicsTokens->mjcActuatorfrcrangeMax,
      MjcPhysicsTokens->mjcSolreflimit,
      MjcPhysicsTokens->mjcSolimplimit,
      MjcPhysicsTokens->mjcSolreffriction,
      MjcPhysicsTokens->mjcSolimpfriction,
      MjcPhysicsTokens->mjcMargin,
      MjcPhysicsTokens->mjcFrictionloss,
      MjcPhysicsTokens->mjcWidth,
      MjcPhysicsTokens->mjcRgba,
      MjcPhysicsTokens->mjcSpringlength,
      MjcPhysicsTokens->mjcStiffness,
      MjcPhysicsTokens->mjcDamping,
      MjcPhysicsTokens->mjcArmature,
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
