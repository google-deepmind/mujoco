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

#include <boost/python.hpp>
#include <string>

#include "./sceneAPI.h"
#include "pxr/base/tf/pyAnnotatedBoolResult.h"
#include "pxr/base/tf/pyContainerConversions.h"
#include "pxr/base/tf/pyResultConversions.h"
#include "pxr/base/tf/pyUtils.h"
#include "pxr/base/tf/wrapTypeHelpers.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/usd/pyConversions.h"
#include "pxr/usd/usd/schemaBase.h"

using namespace boost::python;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

#define WRAP_CUSTOM    \
  template <class Cls> \
  static void _CustomWrapCode(Cls &_class)

// fwd decl.
WRAP_CUSTOM;

static UsdAttribute _CreateTimestepAttr(MjcPhysicsSceneAPI &self,
                                        object defaultVal, bool writeSparsely) {
  return self.CreateTimestepAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateApiRateAttr(MjcPhysicsSceneAPI &self,
                                       object defaultVal, bool writeSparsely) {
  return self.CreateApiRateAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateImpRatioAttr(MjcPhysicsSceneAPI &self,
                                        object defaultVal, bool writeSparsely) {
  return self.CreateImpRatioAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateWindAttr(MjcPhysicsSceneAPI &self, object defaultVal,
                                    bool writeSparsely) {
  return self.CreateWindAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double3),
      writeSparsely);
}

static UsdAttribute _CreateMagneticAttr(MjcPhysicsSceneAPI &self,
                                        object defaultVal, bool writeSparsely) {
  return self.CreateMagneticAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double3),
      writeSparsely);
}

static UsdAttribute _CreateDensityAttr(MjcPhysicsSceneAPI &self,
                                       object defaultVal, bool writeSparsely) {
  return self.CreateDensityAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateViscosityAttr(MjcPhysicsSceneAPI &self,
                                         object defaultVal,
                                         bool writeSparsely) {
  return self.CreateViscosityAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateOMarginAttr(MjcPhysicsSceneAPI &self,
                                       object defaultVal, bool writeSparsely) {
  return self.CreateOMarginAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateOSolRefAttr(MjcPhysicsSceneAPI &self,
                                       object defaultVal, bool writeSparsely) {
  return self.CreateOSolRefAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->DoubleArray),
      writeSparsely);
}

static UsdAttribute _CreateOSolImpAttr(MjcPhysicsSceneAPI &self,
                                       object defaultVal, bool writeSparsely) {
  return self.CreateOSolImpAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->DoubleArray),
      writeSparsely);
}

static UsdAttribute _CreateOFrictionAttr(MjcPhysicsSceneAPI &self,
                                         object defaultVal,
                                         bool writeSparsely) {
  return self.CreateOFrictionAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->DoubleArray),
      writeSparsely);
}

static UsdAttribute _CreateIntegratorAttr(MjcPhysicsSceneAPI &self,
                                          object defaultVal,
                                          bool writeSparsely) {
  return self.CreateIntegratorAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static UsdAttribute _CreateConeAttr(MjcPhysicsSceneAPI &self, object defaultVal,
                                    bool writeSparsely) {
  return self.CreateConeAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static UsdAttribute _CreateJacobianAttr(MjcPhysicsSceneAPI &self,
                                        object defaultVal, bool writeSparsely) {
  return self.CreateJacobianAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static UsdAttribute _CreateSolverAttr(MjcPhysicsSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
  return self.CreateSolverAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}

static UsdAttribute _CreateIterationsAttr(MjcPhysicsSceneAPI &self,
                                          object defaultVal,
                                          bool writeSparsely) {
  return self.CreateIterationsAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static UsdAttribute _CreateToleranceAttr(MjcPhysicsSceneAPI &self,
                                         object defaultVal,
                                         bool writeSparsely) {
  return self.CreateToleranceAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateLSIterationsAttr(MjcPhysicsSceneAPI &self,
                                            object defaultVal,
                                            bool writeSparsely) {
  return self.CreateLSIterationsAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static UsdAttribute _CreateLSToleranceAttr(MjcPhysicsSceneAPI &self,
                                           object defaultVal,
                                           bool writeSparsely) {
  return self.CreateLSToleranceAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateNoslipIterationsAttr(MjcPhysicsSceneAPI &self,
                                                object defaultVal,
                                                bool writeSparsely) {
  return self.CreateNoslipIterationsAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static UsdAttribute _CreateNoslipToleranceAttr(MjcPhysicsSceneAPI &self,
                                               object defaultVal,
                                               bool writeSparsely) {
  return self.CreateNoslipToleranceAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateCCDIterationsAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateCCDIterationsAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static UsdAttribute _CreateCCDToleranceAttr(MjcPhysicsSceneAPI &self,
                                            object defaultVal,
                                            bool writeSparsely) {
  return self.CreateCCDToleranceAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Double), writeSparsely);
}

static UsdAttribute _CreateSDFIterationsAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateSDFIterationsAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static UsdAttribute _CreateSDFInitPointsAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateSDFInitPointsAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}

static UsdAttribute _CreateActuatorGroupDisableAttr(MjcPhysicsSceneAPI &self,
                                                    object defaultVal,
                                                    bool writeSparsely) {
  return self.CreateActuatorGroupDisableAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray),
      writeSparsely);
}

static UsdAttribute _CreateConstraintFlagAttr(MjcPhysicsSceneAPI &self,
                                              object defaultVal,
                                              bool writeSparsely) {
  return self.CreateConstraintFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateEqualityFlagAttr(MjcPhysicsSceneAPI &self,
                                            object defaultVal,
                                            bool writeSparsely) {
  return self.CreateEqualityFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateFrictionLossFlagAttr(MjcPhysicsSceneAPI &self,
                                                object defaultVal,
                                                bool writeSparsely) {
  return self.CreateFrictionLossFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateLimitFlagAttr(MjcPhysicsSceneAPI &self,
                                         object defaultVal,
                                         bool writeSparsely) {
  return self.CreateLimitFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateContactFlagAttr(MjcPhysicsSceneAPI &self,
                                           object defaultVal,
                                           bool writeSparsely) {
  return self.CreateContactFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreatePassiveFlagAttr(MjcPhysicsSceneAPI &self,
                                           object defaultVal,
                                           bool writeSparsely) {
  return self.CreatePassiveFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateGravityFlagAttr(MjcPhysicsSceneAPI &self,
                                           object defaultVal,
                                           bool writeSparsely) {
  return self.CreateGravityFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateClampCtrlFlagAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateClampCtrlFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateWarmStartFlagAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateWarmStartFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateFilterParentFlagAttr(MjcPhysicsSceneAPI &self,
                                                object defaultVal,
                                                bool writeSparsely) {
  return self.CreateFilterParentFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateActuationFlagAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateActuationFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateRefSafeFlagAttr(MjcPhysicsSceneAPI &self,
                                           object defaultVal,
                                           bool writeSparsely) {
  return self.CreateRefSafeFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateSensorFlagAttr(MjcPhysicsSceneAPI &self,
                                          object defaultVal,
                                          bool writeSparsely) {
  return self.CreateSensorFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateMidPhaseFlagAttr(MjcPhysicsSceneAPI &self,
                                            object defaultVal,
                                            bool writeSparsely) {
  return self.CreateMidPhaseFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateNativeCCDFlagAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateNativeCCDFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateEulerDampFlagAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateEulerDampFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateAutoResetFlagAttr(MjcPhysicsSceneAPI &self,
                                             object defaultVal,
                                             bool writeSparsely) {
  return self.CreateAutoResetFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateOverrideFlagAttr(MjcPhysicsSceneAPI &self,
                                            object defaultVal,
                                            bool writeSparsely) {
  return self.CreateOverrideFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateEnergyFlagAttr(MjcPhysicsSceneAPI &self,
                                          object defaultVal,
                                          bool writeSparsely) {
  return self.CreateEnergyFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateFwdinvFlagAttr(MjcPhysicsSceneAPI &self,
                                          object defaultVal,
                                          bool writeSparsely) {
  return self.CreateFwdinvFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateInvDiscreteFlagAttr(MjcPhysicsSceneAPI &self,
                                               object defaultVal,
                                               bool writeSparsely) {
  return self.CreateInvDiscreteFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateMultiCCDFlagAttr(MjcPhysicsSceneAPI &self,
                                            object defaultVal,
                                            bool writeSparsely) {
  return self.CreateMultiCCDFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static UsdAttribute _CreateIslandFlagAttr(MjcPhysicsSceneAPI &self,
                                          object defaultVal,
                                          bool writeSparsely) {
  return self.CreateIslandFlagAttr(
      UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string _Repr(const MjcPhysicsSceneAPI &self) {
  std::string primRepr = TfPyRepr(self.GetPrim());
  return TfStringPrintf("MjcPhysics.SceneAPI(%s)", primRepr.c_str());
}

struct MjcPhysicsSceneAPI_CanApplyResult
    : public TfPyAnnotatedBoolResult<std::string> {
  MjcPhysicsSceneAPI_CanApplyResult(bool val, std::string const &msg)
      : TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static MjcPhysicsSceneAPI_CanApplyResult _WrapCanApply(const UsdPrim &prim) {
  std::string whyNot;
  bool result = MjcPhysicsSceneAPI::CanApply(prim, &whyNot);
  return MjcPhysicsSceneAPI_CanApplyResult(result, whyNot);
}

}  // anonymous namespace

void wrapMjcPhysicsSceneAPI() {
  typedef MjcPhysicsSceneAPI This;

  MjcPhysicsSceneAPI_CanApplyResult::Wrap<MjcPhysicsSceneAPI_CanApplyResult>(
      "_CanApplyResult", "whyNot");

  class_<This, bases<UsdAPISchemaBase> > cls("SceneAPI");

  cls.def(init<UsdPrim>(arg("prim")))
      .def(init<UsdSchemaBase const &>(arg("schemaObj")))
      .def(TfTypePythonClass())

      .def("Get", &This::Get, (arg("stage"), arg("path")))
      .staticmethod("Get")

      .def("CanApply", &_WrapCanApply, (arg("prim")))
      .staticmethod("CanApply")

      .def("Apply", &This::Apply, (arg("prim")))
      .staticmethod("Apply")

      .def("GetSchemaAttributeNames", &This::GetSchemaAttributeNames,
           arg("includeInherited") = true,
           return_value_policy<TfPySequenceToList>())
      .staticmethod("GetSchemaAttributeNames")

      .def("_GetStaticTfType", (TfType const &(*)())TfType::Find<This>,
           return_value_policy<return_by_value>())
      .staticmethod("_GetStaticTfType")

      .def(!self)

      .def("GetTimestepAttr", &This::GetTimestepAttr)
      .def("CreateTimestepAttr", &_CreateTimestepAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetApiRateAttr", &This::GetApiRateAttr)
      .def("CreateApiRateAttr", &_CreateApiRateAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetImpRatioAttr", &This::GetImpRatioAttr)
      .def("CreateImpRatioAttr", &_CreateImpRatioAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetWindAttr", &This::GetWindAttr)
      .def("CreateWindAttr", &_CreateWindAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetMagneticAttr", &This::GetMagneticAttr)
      .def("CreateMagneticAttr", &_CreateMagneticAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetDensityAttr", &This::GetDensityAttr)
      .def("CreateDensityAttr", &_CreateDensityAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetViscosityAttr", &This::GetViscosityAttr)
      .def("CreateViscosityAttr", &_CreateViscosityAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetOMarginAttr", &This::GetOMarginAttr)
      .def("CreateOMarginAttr", &_CreateOMarginAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetOSolRefAttr", &This::GetOSolRefAttr)
      .def("CreateOSolRefAttr", &_CreateOSolRefAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetOSolImpAttr", &This::GetOSolImpAttr)
      .def("CreateOSolImpAttr", &_CreateOSolImpAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetOFrictionAttr", &This::GetOFrictionAttr)
      .def("CreateOFrictionAttr", &_CreateOFrictionAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetIntegratorAttr", &This::GetIntegratorAttr)
      .def("CreateIntegratorAttr", &_CreateIntegratorAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetConeAttr", &This::GetConeAttr)
      .def("CreateConeAttr", &_CreateConeAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetJacobianAttr", &This::GetJacobianAttr)
      .def("CreateJacobianAttr", &_CreateJacobianAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetSolverAttr", &This::GetSolverAttr)
      .def("CreateSolverAttr", &_CreateSolverAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetIterationsAttr", &This::GetIterationsAttr)
      .def("CreateIterationsAttr", &_CreateIterationsAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetToleranceAttr", &This::GetToleranceAttr)
      .def("CreateToleranceAttr", &_CreateToleranceAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetLSIterationsAttr", &This::GetLSIterationsAttr)
      .def("CreateLSIterationsAttr", &_CreateLSIterationsAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetLSToleranceAttr", &This::GetLSToleranceAttr)
      .def("CreateLSToleranceAttr", &_CreateLSToleranceAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetNoslipIterationsAttr", &This::GetNoslipIterationsAttr)
      .def("CreateNoslipIterationsAttr", &_CreateNoslipIterationsAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetNoslipToleranceAttr", &This::GetNoslipToleranceAttr)
      .def("CreateNoslipToleranceAttr", &_CreateNoslipToleranceAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetCCDIterationsAttr", &This::GetCCDIterationsAttr)
      .def("CreateCCDIterationsAttr", &_CreateCCDIterationsAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetCCDToleranceAttr", &This::GetCCDToleranceAttr)
      .def("CreateCCDToleranceAttr", &_CreateCCDToleranceAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetSDFIterationsAttr", &This::GetSDFIterationsAttr)
      .def("CreateSDFIterationsAttr", &_CreateSDFIterationsAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetSDFInitPointsAttr", &This::GetSDFInitPointsAttr)
      .def("CreateSDFInitPointsAttr", &_CreateSDFInitPointsAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetActuatorGroupDisableAttr", &This::GetActuatorGroupDisableAttr)
      .def("CreateActuatorGroupDisableAttr", &_CreateActuatorGroupDisableAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetConstraintFlagAttr", &This::GetConstraintFlagAttr)
      .def("CreateConstraintFlagAttr", &_CreateConstraintFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetEqualityFlagAttr", &This::GetEqualityFlagAttr)
      .def("CreateEqualityFlagAttr", &_CreateEqualityFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetFrictionLossFlagAttr", &This::GetFrictionLossFlagAttr)
      .def("CreateFrictionLossFlagAttr", &_CreateFrictionLossFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetLimitFlagAttr", &This::GetLimitFlagAttr)
      .def("CreateLimitFlagAttr", &_CreateLimitFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetContactFlagAttr", &This::GetContactFlagAttr)
      .def("CreateContactFlagAttr", &_CreateContactFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetPassiveFlagAttr", &This::GetPassiveFlagAttr)
      .def("CreatePassiveFlagAttr", &_CreatePassiveFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetGravityFlagAttr", &This::GetGravityFlagAttr)
      .def("CreateGravityFlagAttr", &_CreateGravityFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetClampCtrlFlagAttr", &This::GetClampCtrlFlagAttr)
      .def("CreateClampCtrlFlagAttr", &_CreateClampCtrlFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetWarmStartFlagAttr", &This::GetWarmStartFlagAttr)
      .def("CreateWarmStartFlagAttr", &_CreateWarmStartFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetFilterParentFlagAttr", &This::GetFilterParentFlagAttr)
      .def("CreateFilterParentFlagAttr", &_CreateFilterParentFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetActuationFlagAttr", &This::GetActuationFlagAttr)
      .def("CreateActuationFlagAttr", &_CreateActuationFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetRefSafeFlagAttr", &This::GetRefSafeFlagAttr)
      .def("CreateRefSafeFlagAttr", &_CreateRefSafeFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetSensorFlagAttr", &This::GetSensorFlagAttr)
      .def("CreateSensorFlagAttr", &_CreateSensorFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetMidPhaseFlagAttr", &This::GetMidPhaseFlagAttr)
      .def("CreateMidPhaseFlagAttr", &_CreateMidPhaseFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetNativeCCDFlagAttr", &This::GetNativeCCDFlagAttr)
      .def("CreateNativeCCDFlagAttr", &_CreateNativeCCDFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetEulerDampFlagAttr", &This::GetEulerDampFlagAttr)
      .def("CreateEulerDampFlagAttr", &_CreateEulerDampFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetAutoResetFlagAttr", &This::GetAutoResetFlagAttr)
      .def("CreateAutoResetFlagAttr", &_CreateAutoResetFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetOverrideFlagAttr", &This::GetOverrideFlagAttr)
      .def("CreateOverrideFlagAttr", &_CreateOverrideFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetEnergyFlagAttr", &This::GetEnergyFlagAttr)
      .def("CreateEnergyFlagAttr", &_CreateEnergyFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetFwdinvFlagAttr", &This::GetFwdinvFlagAttr)
      .def("CreateFwdinvFlagAttr", &_CreateFwdinvFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetInvDiscreteFlagAttr", &This::GetInvDiscreteFlagAttr)
      .def("CreateInvDiscreteFlagAttr", &_CreateInvDiscreteFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetMultiCCDFlagAttr", &This::GetMultiCCDFlagAttr)
      .def("CreateMultiCCDFlagAttr", &_CreateMultiCCDFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("GetIslandFlagAttr", &This::GetIslandFlagAttr)
      .def("CreateIslandFlagAttr", &_CreateIslandFlagAttr,
           (arg("defaultValue") = object(), arg("writeSparsely") = false))

      .def("__repr__", ::_Repr);

  _CustomWrapCode(cls);
}

// ===================================================================== //
// Feel free to add custom code below this line, it will be preserved by
// the code generator.  The entry point for your custom code should look
// minimally like the following:
//
// WRAP_CUSTOM {
//     _class
//         .def("MyCustomMethod", ...)
//     ;
// }
//
// Of course any other ancillary or support code may be provided.
//
// Just remember to wrap code in the appropriate delimiters:
// 'namespace {', '}'.
//
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--

namespace {

WRAP_CUSTOM {}

}  // namespace
