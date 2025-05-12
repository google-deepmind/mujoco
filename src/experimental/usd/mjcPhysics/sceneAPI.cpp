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
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsTimestep);
}

UsdAttribute MjcPhysicsSceneAPI::CreateTimestepAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsTimestep, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetApiRateAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsApirate);
}

UsdAttribute MjcPhysicsSceneAPI::CreateApiRateAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsApirate, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetImpRatioAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsImpratio);
}

UsdAttribute MjcPhysicsSceneAPI::CreateImpRatioAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsImpratio, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetWindAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsWind);
}

UsdAttribute MjcPhysicsSceneAPI::CreateWindAttr(VtValue const &defaultValue,
                                                bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsWind, SdfValueTypeNames->Double3,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetMagneticAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsMagnetic);
}

UsdAttribute MjcPhysicsSceneAPI::CreateMagneticAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsMagnetic, SdfValueTypeNames->Double3,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetDensityAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsDensity);
}

UsdAttribute MjcPhysicsSceneAPI::CreateDensityAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsDensity, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetViscosityAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsViscosity);
}

UsdAttribute MjcPhysicsSceneAPI::CreateViscosityAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsViscosity, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOMarginAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsO_margin);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOMarginAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsO_margin, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOSolRefAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsO_solref);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOSolRefAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsO_solref, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOSolImpAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsO_solimp);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOSolImpAttr(VtValue const &defaultValue,
                                                   bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsO_solimp, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOFrictionAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsO_friction);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOFrictionAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsO_friction, SdfValueTypeNames->DoubleArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetIntegratorAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsIntegrator);
}

UsdAttribute MjcPhysicsSceneAPI::CreateIntegratorAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsIntegrator, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetConeAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsCone);
}

UsdAttribute MjcPhysicsSceneAPI::CreateConeAttr(VtValue const &defaultValue,
                                                bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsCone, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetJacobianAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsJacobian);
}

UsdAttribute MjcPhysicsSceneAPI::CreateJacobianAttr(VtValue const &defaultValue,
                                                    bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsJacobian, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSolverAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsSolver);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSolverAttr(VtValue const &defaultValue,
                                                  bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsSolver, SdfValueTypeNames->Token,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsIterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsIterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsTolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsTolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetLSIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsLs_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateLSIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsLs_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetLSToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsLs_tolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateLSToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsLs_tolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetNoslipIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsNoslip_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateNoslipIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsNoslip_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetNoslipToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsNoslip_tolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateNoslipToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsNoslip_tolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetCCDIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsCcd_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateCCDIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsCcd_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetCCDToleranceAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsCcd_tolerance);
}

UsdAttribute MjcPhysicsSceneAPI::CreateCCDToleranceAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsCcd_tolerance, SdfValueTypeNames->Double,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSDFIterationsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsSdf_iterations);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSDFIterationsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsSdf_iterations, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSDFInitPointsAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsSdf_initpoints);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSDFInitPointsAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsSdf_initpoints, SdfValueTypeNames->Int,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetActuatorGroupDisableAttr() const {
  return GetPrim().GetAttribute(
      MjcPhysicsTokens->mjcPhysicsActuatorgroupdisable);
}

UsdAttribute MjcPhysicsSceneAPI::CreateActuatorGroupDisableAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsActuatorgroupdisable,
      SdfValueTypeNames->IntArray,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetConstraintFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagConstraint);
}

UsdAttribute MjcPhysicsSceneAPI::CreateConstraintFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagConstraint, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetEqualityFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagEquality);
}

UsdAttribute MjcPhysicsSceneAPI::CreateEqualityFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagEquality, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetFrictionLossFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagFrictionloss);
}

UsdAttribute MjcPhysicsSceneAPI::CreateFrictionLossFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagFrictionloss, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetLimitFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagLimit);
}

UsdAttribute MjcPhysicsSceneAPI::CreateLimitFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagLimit, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetContactFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagContact);
}

UsdAttribute MjcPhysicsSceneAPI::CreateContactFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagContact, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetPassiveFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagPassive);
}

UsdAttribute MjcPhysicsSceneAPI::CreatePassiveFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagPassive, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetGravityFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagGravity);
}

UsdAttribute MjcPhysicsSceneAPI::CreateGravityFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagGravity, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetClampCtrlFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagClampctrl);
}

UsdAttribute MjcPhysicsSceneAPI::CreateClampCtrlFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagClampctrl, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetWarmStartFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagWarmstart);
}

UsdAttribute MjcPhysicsSceneAPI::CreateWarmStartFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagWarmstart, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetFilterParentFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagFilterparent);
}

UsdAttribute MjcPhysicsSceneAPI::CreateFilterParentFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagFilterparent, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetActuationFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagActuation);
}

UsdAttribute MjcPhysicsSceneAPI::CreateActuationFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagActuation, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetRefSafeFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagRefsafe);
}

UsdAttribute MjcPhysicsSceneAPI::CreateRefSafeFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagRefsafe, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetSensorFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagSensor);
}

UsdAttribute MjcPhysicsSceneAPI::CreateSensorFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagSensor, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetMidPhaseFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagMidphase);
}

UsdAttribute MjcPhysicsSceneAPI::CreateMidPhaseFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagMidphase, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetNativeCCDFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagNativeccd);
}

UsdAttribute MjcPhysicsSceneAPI::CreateNativeCCDFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagNativeccd, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetEulerDampFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagEulerdamp);
}

UsdAttribute MjcPhysicsSceneAPI::CreateEulerDampFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagEulerdamp, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetAutoResetFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagAutoreset);
}

UsdAttribute MjcPhysicsSceneAPI::CreateAutoResetFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagAutoreset, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetOverrideFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagOverride);
}

UsdAttribute MjcPhysicsSceneAPI::CreateOverrideFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagOverride, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetEnergyFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagEnergy);
}

UsdAttribute MjcPhysicsSceneAPI::CreateEnergyFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagEnergy, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetFwdinvFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagFwdinv);
}

UsdAttribute MjcPhysicsSceneAPI::CreateFwdinvFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagFwdinv, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetInvDiscreteFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagInvdiscrete);
}

UsdAttribute MjcPhysicsSceneAPI::CreateInvDiscreteFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagInvdiscrete, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetMultiCCDFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagMulticcd);
}

UsdAttribute MjcPhysicsSceneAPI::CreateMultiCCDFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagMulticcd, SdfValueTypeNames->Bool,
      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute MjcPhysicsSceneAPI::GetIslandFlagAttr() const {
  return GetPrim().GetAttribute(MjcPhysicsTokens->mjcPhysicsFlagIsland);
}

UsdAttribute MjcPhysicsSceneAPI::CreateIslandFlagAttr(
    VtValue const &defaultValue, bool writeSparsely) const {
  return UsdSchemaBase::_CreateAttr(
      MjcPhysicsTokens->mjcPhysicsFlagIsland, SdfValueTypeNames->Bool,
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
      MjcPhysicsTokens->mjcPhysicsTimestep,
      MjcPhysicsTokens->mjcPhysicsApirate,
      MjcPhysicsTokens->mjcPhysicsImpratio,
      MjcPhysicsTokens->mjcPhysicsWind,
      MjcPhysicsTokens->mjcPhysicsMagnetic,
      MjcPhysicsTokens->mjcPhysicsDensity,
      MjcPhysicsTokens->mjcPhysicsViscosity,
      MjcPhysicsTokens->mjcPhysicsO_margin,
      MjcPhysicsTokens->mjcPhysicsO_solref,
      MjcPhysicsTokens->mjcPhysicsO_solimp,
      MjcPhysicsTokens->mjcPhysicsO_friction,
      MjcPhysicsTokens->mjcPhysicsIntegrator,
      MjcPhysicsTokens->mjcPhysicsCone,
      MjcPhysicsTokens->mjcPhysicsJacobian,
      MjcPhysicsTokens->mjcPhysicsSolver,
      MjcPhysicsTokens->mjcPhysicsIterations,
      MjcPhysicsTokens->mjcPhysicsTolerance,
      MjcPhysicsTokens->mjcPhysicsLs_iterations,
      MjcPhysicsTokens->mjcPhysicsLs_tolerance,
      MjcPhysicsTokens->mjcPhysicsNoslip_iterations,
      MjcPhysicsTokens->mjcPhysicsNoslip_tolerance,
      MjcPhysicsTokens->mjcPhysicsCcd_iterations,
      MjcPhysicsTokens->mjcPhysicsCcd_tolerance,
      MjcPhysicsTokens->mjcPhysicsSdf_iterations,
      MjcPhysicsTokens->mjcPhysicsSdf_initpoints,
      MjcPhysicsTokens->mjcPhysicsActuatorgroupdisable,
      MjcPhysicsTokens->mjcPhysicsFlagConstraint,
      MjcPhysicsTokens->mjcPhysicsFlagEquality,
      MjcPhysicsTokens->mjcPhysicsFlagFrictionloss,
      MjcPhysicsTokens->mjcPhysicsFlagLimit,
      MjcPhysicsTokens->mjcPhysicsFlagContact,
      MjcPhysicsTokens->mjcPhysicsFlagPassive,
      MjcPhysicsTokens->mjcPhysicsFlagGravity,
      MjcPhysicsTokens->mjcPhysicsFlagClampctrl,
      MjcPhysicsTokens->mjcPhysicsFlagWarmstart,
      MjcPhysicsTokens->mjcPhysicsFlagFilterparent,
      MjcPhysicsTokens->mjcPhysicsFlagActuation,
      MjcPhysicsTokens->mjcPhysicsFlagRefsafe,
      MjcPhysicsTokens->mjcPhysicsFlagSensor,
      MjcPhysicsTokens->mjcPhysicsFlagMidphase,
      MjcPhysicsTokens->mjcPhysicsFlagNativeccd,
      MjcPhysicsTokens->mjcPhysicsFlagEulerdamp,
      MjcPhysicsTokens->mjcPhysicsFlagAutoreset,
      MjcPhysicsTokens->mjcPhysicsFlagOverride,
      MjcPhysicsTokens->mjcPhysicsFlagEnergy,
      MjcPhysicsTokens->mjcPhysicsFlagFwdinv,
      MjcPhysicsTokens->mjcPhysicsFlagInvdiscrete,
      MjcPhysicsTokens->mjcPhysicsFlagMulticcd,
      MjcPhysicsTokens->mjcPhysicsFlagIsland,
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
