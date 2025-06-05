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

#include "./sceneAPI.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
  TfType::Define<MjcPhysicsSceneAPI, TfType::Bases<UsdAPISchemaBase> >();
}

/* virtual */
MjcPhysicsSceneAPI::~MjcPhysicsSceneAPI() {}

/* static */
MjcPhysicsSceneAPI MjcPhysicsSceneAPI::Get(const UsdStagePtr &stage,
                                           const SdfPath &path) {
  if (!stage) {
    TF_CODING_ERROR("Invalid stage");
    return MjcPhysicsSceneAPI();
  }
  return MjcPhysicsSceneAPI(stage->GetPrimAtPath(path));
}

/* virtual */
UsdSchemaKind MjcPhysicsSceneAPI::_GetSchemaKind() const {
  return MjcPhysicsSceneAPI::schemaKind;
}

/* static */
bool MjcPhysicsSceneAPI::CanApply(const UsdPrim &prim, std::string *whyNot) {
  return prim.CanApplyAPI<MjcPhysicsSceneAPI>(whyNot);
}

/* static */
MjcPhysicsSceneAPI MjcPhysicsSceneAPI::Apply(const UsdPrim &prim) {
  if (prim.ApplyAPI<MjcPhysicsSceneAPI>()) {
    return MjcPhysicsSceneAPI(prim);
  }
  return MjcPhysicsSceneAPI();
}

/* static */
const TfType &MjcPhysicsSceneAPI::_GetStaticTfType() {
  static TfType tfType = TfType::Find<MjcPhysicsSceneAPI>();
  return tfType;
}

/* static */
bool MjcPhysicsSceneAPI::_IsTypedSchema() {
  static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
  return isTyped;
}

/* virtual */
const TfType &MjcPhysicsSceneAPI::_GetTfType() const {
  return _GetStaticTfType();
}

UsdAttribute MjcPhysicsSceneAPI::GetTimestepAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionTimestep);
}

UsdAttribute MjcPhysicsSceneAPI::CreateTimestepAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionTimestep, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetApiRateAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionApirate);
}

UsdAttribute MjcPhysicsSceneAPI::CreateApiRateAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionApirate, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetImpRatioAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionImpratio);
}

UsdAttribute MjcPhysicsSceneAPI::CreateImpRatioAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionImpratio, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetWindAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionWind);
}

UsdAttribute MjcPhysicsSceneAPI::CreateWindAttr(VtValue const &defaultValue,
                                                bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionWind, SdfValueTypeNames->Double3,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetMagneticAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionMagnetic);
}

UsdAttribute MjcPhysicsSceneAPI::CreateMagneticAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionMagnetic, SdfValueTypeNames->Double3,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetDensityAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionDensity);
}

UsdAttribute MjcPhysicsSceneAPI::CreateDensityAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionDensity, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetViscosityAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionViscosity);
}

UsdAttribute MjcPhysicsSceneAPI::CreateViscosityAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionViscosity, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOMarginAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionO_margin);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOMarginAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionO_margin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOSolRefAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionO_solref);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOSolRefAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionO_solref, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOSolImpAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionO_solimp);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOSolImpAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionO_solimp, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOFrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionO_friction);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOFrictionAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionO_friction, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetIntegratorAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionIntegrator);
}

UsdAttribute MjcPhysicsSceneAPI::CreateIntegratorAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionIntegrator, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetConeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionCone);
}

UsdAttribute MjcPhysicsSceneAPI::CreateConeAttr(VtValue const &defaultValue,
                                                bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionCone, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetJacobianAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionJacobian);
}

UsdAttribute MjcPhysicsSceneAPI::CreateJacobianAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionJacobian, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSolverAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionSolver);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSolverAttr(VtValue const &defaultValue,
                                                  bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionSolver, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionIterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionIterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionTolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionTolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetLSIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionLs_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateLSIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionLs_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetLSToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionLs_tolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateLSToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionLs_tolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetNoslipIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionNoslip_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateNoslipIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionNoslip_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetNoslipToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionNoslip_tolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateNoslipToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionNoslip_tolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetCCDIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionCcd_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateCCDIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionCcd_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetCCDToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionCcd_tolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateCCDToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionCcd_tolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSDFIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionSdf_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSDFIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionSdf_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSDFInitPointsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcOptionSdf_initpoints);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSDFInitPointsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionSdf_initpoints, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetActuatorGroupDisableAttr() const {
  return GetPrim().GetAttribute(
      MjcPhysicsTokens->mjcOptionActuatorgroupdisable);
}

UsdAttribute MjcPhysicsSceneAPI::CreateActuatorGroupDisableAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcOptionActuatorgroupdisable,
      SdfValueTypeNames->IntArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetConstraintFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagConstraint);
}

UsdAttribute MjcPhysicsSceneAPI::CreateConstraintFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagConstraint, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetEqualityFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagEquality);
}

UsdAttribute MjcPhysicsSceneAPI::CreateEqualityFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagEquality, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetFrictionLossFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagFrictionloss);
}

UsdAttribute MjcPhysicsSceneAPI::CreateFrictionLossFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagFrictionloss, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetLimitFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagLimit);
}

UsdAttribute MjcPhysicsSceneAPI::CreateLimitFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagLimit, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetContactFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagContact);
}

UsdAttribute MjcPhysicsSceneAPI::CreateContactFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagContact, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetPassiveFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagPassive);
}

UsdAttribute MjcPhysicsSceneAPI::CreatePassiveFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagPassive, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetGravityFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagGravity);
}

UsdAttribute MjcPhysicsSceneAPI::CreateGravityFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagGravity, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetClampCtrlFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagClampctrl);
}

UsdAttribute MjcPhysicsSceneAPI::CreateClampCtrlFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagClampctrl, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetWarmStartFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagWarmstart);
}

UsdAttribute MjcPhysicsSceneAPI::CreateWarmStartFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagWarmstart, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetFilterParentFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagFilterparent);
}

UsdAttribute MjcPhysicsSceneAPI::CreateFilterParentFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagFilterparent, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetActuationFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagActuation);
}

UsdAttribute MjcPhysicsSceneAPI::CreateActuationFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagActuation, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetRefSafeFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagRefsafe);
}

UsdAttribute MjcPhysicsSceneAPI::CreateRefSafeFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagRefsafe, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSensorFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagSensor);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSensorFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagSensor, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetMidPhaseFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagMidphase);
}

UsdAttribute MjcPhysicsSceneAPI::CreateMidPhaseFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagMidphase, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetNativeCCDFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagNativeccd);
}

UsdAttribute MjcPhysicsSceneAPI::CreateNativeCCDFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagNativeccd, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetEulerDampFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagEulerdamp);
}

UsdAttribute MjcPhysicsSceneAPI::CreateEulerDampFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagEulerdamp, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetAutoResetFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagAutoreset);
}

UsdAttribute MjcPhysicsSceneAPI::CreateAutoResetFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagAutoreset, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOverrideFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagOverride);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOverrideFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagOverride, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetEnergyFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagEnergy);
}

UsdAttribute MjcPhysicsSceneAPI::CreateEnergyFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagEnergy, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetFwdinvFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagFwdinv);
}

UsdAttribute MjcPhysicsSceneAPI::CreateFwdinvFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagFwdinv, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetInvDiscreteFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagInvdiscrete);
}

UsdAttribute MjcPhysicsSceneAPI::CreateInvDiscreteFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagInvdiscrete, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetMultiCCDFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagMulticcd);
}

UsdAttribute MjcPhysicsSceneAPI::CreateMultiCCDFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagMulticcd, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetIslandFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcFlagIsland);
}

UsdAttribute MjcPhysicsSceneAPI::CreateIslandFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcFlagIsland, SdfValueTypeNames->Bool,
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
const TfTokenVector &MjcPhysicsSceneAPI::GetSchemaAttributeNames(
    bool includeInherited) {
  static TfTokenVector localNames = {
      MjcPhysicsTokens->mjcOptionTimestep,
      MjcPhysicsTokens->mjcOptionApirate,
      MjcPhysicsTokens->mjcOptionImpratio,
      MjcPhysicsTokens->mjcOptionWind,
      MjcPhysicsTokens->mjcOptionMagnetic,
      MjcPhysicsTokens->mjcOptionDensity,
      MjcPhysicsTokens->mjcOptionViscosity,
      MjcPhysicsTokens->mjcOptionO_margin,
      MjcPhysicsTokens->mjcOptionO_solref,
      MjcPhysicsTokens->mjcOptionO_solimp,
      MjcPhysicsTokens->mjcOptionO_friction,
      MjcPhysicsTokens->mjcOptionIntegrator,
      MjcPhysicsTokens->mjcOptionCone,
      MjcPhysicsTokens->mjcOptionJacobian,
      MjcPhysicsTokens->mjcOptionSolver,
      MjcPhysicsTokens->mjcOptionIterations,
      MjcPhysicsTokens->mjcOptionTolerance,
      MjcPhysicsTokens->mjcOptionLs_iterations,
      MjcPhysicsTokens->mjcOptionLs_tolerance,
      MjcPhysicsTokens->mjcOptionNoslip_iterations,
      MjcPhysicsTokens->mjcOptionNoslip_tolerance,
      MjcPhysicsTokens->mjcOptionCcd_iterations,
      MjcPhysicsTokens->mjcOptionCcd_tolerance,
      MjcPhysicsTokens->mjcOptionSdf_iterations,
      MjcPhysicsTokens->mjcOptionSdf_initpoints,
      MjcPhysicsTokens->mjcOptionActuatorgroupdisable,
      MjcPhysicsTokens->mjcFlagConstraint,
      MjcPhysicsTokens->mjcFlagEquality,
      MjcPhysicsTokens->mjcFlagFrictionloss,
      MjcPhysicsTokens->mjcFlagLimit,
      MjcPhysicsTokens->mjcFlagContact,
      MjcPhysicsTokens->mjcFlagPassive,
      MjcPhysicsTokens->mjcFlagGravity,
      MjcPhysicsTokens->mjcFlagClampctrl,
      MjcPhysicsTokens->mjcFlagWarmstart,
      MjcPhysicsTokens->mjcFlagFilterparent,
      MjcPhysicsTokens->mjcFlagActuation,
      MjcPhysicsTokens->mjcFlagRefsafe,
      MjcPhysicsTokens->mjcFlagSensor,
      MjcPhysicsTokens->mjcFlagMidphase,
      MjcPhysicsTokens->mjcFlagNativeccd,
      MjcPhysicsTokens->mjcFlagEulerdamp,
      MjcPhysicsTokens->mjcFlagAutoreset,
      MjcPhysicsTokens->mjcFlagOverride,
      MjcPhysicsTokens->mjcFlagEnergy,
      MjcPhysicsTokens->mjcFlagFwdinv,
      MjcPhysicsTokens->mjcFlagInvdiscrete,
      MjcPhysicsTokens->mjcFlagMulticcd,
      MjcPhysicsTokens->mjcFlagIsland,
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
