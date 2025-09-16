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

#ifndef MJCPHYSICS_TOKENS_H
#define MJCPHYSICS_TOKENS_H

/// \file mjcPhysics/tokens.h

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//
// This is an automatically generated file (by usdGenSchema.py).
// Do not hand-edit!
//
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#include <vector>

#include <mujoco/experimental/usd/mjcPhysics/api.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/pxr.h>

PXR_NAMESPACE_OPEN_SCOPE

/// \class MjcPhysicsTokensType
///
/// \link MjcPhysicsTokens \endlink provides static, efficient
/// \link TfToken TfTokens\endlink for use in all public USD API.
///
/// These tokens are auto-generated from the module's schema, representing
/// property names, for when you need to fetch an attribute or relationship
/// directly by name, e.g. UsdPrim::GetAttribute(), in the most efficient
/// manner, and allow the compiler to verify that you spelled the name
/// correctly.
///
/// MjcPhysicsTokens also contains all of the \em allowedTokens values
/// declared for schema builtin attributes of 'token' scene description type.
/// Use MjcPhysicsTokens like so:
///
/// \code
///     gprim.GetMyTokenValuedAttr().Set(MjcPhysicsTokens->affine);
/// \endcode
struct MjcPhysicsTokensType {
  MJCPHYSICS_API MjcPhysicsTokensType();
  /// \brief "affine"
  ///
  /// Possible value for MjcPhysicsActuator::GetMjcBiasTypeAttr(), Possible
  /// value for MjcPhysicsActuator::GetMjcGainTypeAttr()
  const TfToken affine;
  /// \brief "auto"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetInertiaFromGeomAttr(), Fallback
  /// value for MjcPhysicsSceneAPI::GetJacobianAttr(), Fallback value for
  /// MjcPhysicsActuator::GetMjcActLimitedAttr(), Fallback value for
  /// MjcPhysicsActuator::GetMjcCtrlLimitedAttr(), Fallback value for
  /// MjcPhysicsActuator::GetMjcForceLimitedAttr(), Fallback value for
  /// MjcPhysicsJointAPI::GetMjcActuatorfrclimitedAttr(),  This token represents
  /// the auto constraint Jacobian and matrices computed from it.
  const TfToken auto_;
  /// \brief "cg"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetSolverAttr(),  This token
  /// represents the CG constraint solver algorithm.
  const TfToken cg;
  /// \brief "convex"
  ///
  /// Possible value for MjcPhysicsMeshCollisionAPI::GetInertiaAttr()
  const TfToken convex;
  /// \brief "degree"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetAngleAttr()
  const TfToken degree;
  /// \brief "dense"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetJacobianAttr(),  This token
  /// represents the dense constraint Jacobian and matrices computed from it.
  const TfToken dense;
  /// \brief "elliptic"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetConeAttr(),  This token
  /// represents the elliptic contact friction cone type.
  const TfToken elliptic;
  /// \brief "euler"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetIntegratorAttr(),  This token
  /// represents the Euler numerical integrator.
  const TfToken euler;
  /// \brief "exact"
  ///
  /// Possible value for MjcPhysicsMeshCollisionAPI::GetInertiaAttr()
  const TfToken exact;
  /// \brief "false"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetInertiaFromGeomAttr(), Possible
  /// value for MjcPhysicsActuator::GetMjcActLimitedAttr(), Possible value for
  /// MjcPhysicsActuator::GetMjcCtrlLimitedAttr(), Possible value for
  /// MjcPhysicsActuator::GetMjcForceLimitedAttr(), Possible value for
  /// MjcPhysicsJointAPI::GetMjcActuatorfrclimitedAttr()
  const TfToken false_;
  /// \brief "filter"
  ///
  /// Possible value for MjcPhysicsActuator::GetMjcDynTypeAttr()
  const TfToken filter;
  /// \brief "filterexact"
  ///
  /// Possible value for MjcPhysicsActuator::GetMjcDynTypeAttr()
  const TfToken filterexact;
  /// \brief "fixed"
  ///
  /// Fallback value for MjcPhysicsActuator::GetMjcGainTypeAttr()
  const TfToken fixed;
  /// \brief "implicit"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetIntegratorAttr(),  This token
  /// represents the implicit numerical integrator.
  const TfToken implicit;
  /// \brief "implicitfast"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetIntegratorAttr(),  This token
  /// represents the implicitfast numerical integrator.
  const TfToken implicitfast;
  /// \brief "integrator"
  ///
  /// Possible value for MjcPhysicsActuator::GetMjcDynTypeAttr()
  const TfToken integrator;
  /// \brief "legacy"
  ///
  /// Fallback value for MjcPhysicsMeshCollisionAPI::GetInertiaAttr()
  const TfToken legacy;
  /// \brief "mjc:act"
  ///
  /// MjcPhysicsKeyframe
  const TfToken mjcAct;
  /// \brief "mjc:actDim"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcActDim;
  /// \brief "mjc:actEarly"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcActEarly;
  /// \brief "mjc:actLimited"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcActLimited;
  /// \brief "mjc:actRange:max"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcActRangeMax;
  /// \brief "mjc:actRange:min"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcActRangeMin;
  /// \brief "mjc:actuatorfrclimited"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcActuatorfrclimited;
  /// \brief "mjc:actuatorfrcrange:max"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcActuatorfrcrangeMax;
  /// \brief "mjc:actuatorfrcrange:min"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcActuatorfrcrangeMin;
  /// \brief "mjc:actuatorgravcomp"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcActuatorgravcomp;
  /// \brief "mjc:armature"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcArmature;
  /// \brief "mjc:biasPrm"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcBiasPrm;
  /// \brief "mjc:biasType"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcBiasType;
  /// \brief "mjc:compiler:alignFree"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerAlignFree;
  /// \brief "mjc:compiler:angle"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerAngle;
  /// \brief "mjc:compiler:autoLimits"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerAutoLimits;
  /// \brief "mjc:compiler:balanceInertia"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerBalanceInertia;
  /// \brief "mjc:compiler:boundInertia"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerBoundInertia;
  /// \brief "mjc:compiler:boundMass"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerBoundMass;
  /// \brief "mjc:compiler:fitAABB"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerFitAABB;
  /// \brief "mjc:compiler:fuseStatic"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerFuseStatic;
  /// \brief "mjc:compiler:inertiaFromGeom"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerInertiaFromGeom;
  /// \brief "mjc:compiler:inertiaGroupRange:max"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerInertiaGroupRangeMax;
  /// \brief "mjc:compiler:inertiaGroupRange:min"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerInertiaGroupRangeMin;
  /// \brief "mjc:compiler:saveInertial"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerSaveInertial;
  /// \brief "mjc:compiler:setTotalMass"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerSetTotalMass;
  /// \brief "mjc:compiler:useThread"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcCompilerUseThread;
  /// \brief "mjc:condim"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcCondim;
  /// \brief "mjc:crankLength"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcCrankLength;
  /// \brief "mjc:ctrl"
  ///
  /// MjcPhysicsKeyframe
  const TfToken mjcCtrl;
  /// \brief "mjc:ctrlLimited"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcCtrlLimited;
  /// \brief "mjc:ctrlRange:max"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcCtrlRangeMax;
  /// \brief "mjc:ctrlRange:min"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcCtrlRangeMin;
  /// \brief "mjc:damping"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcDamping;
  /// \brief "mjc:dynPrm"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcDynPrm;
  /// \brief "mjc:dynType"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcDynType;
  /// \brief "mjc:flag:actuation"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagActuation;
  /// \brief "mjc:flag:autoreset"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagAutoreset;
  /// \brief "mjc:flag:clampctrl"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagClampctrl;
  /// \brief "mjc:flag:constraint"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagConstraint;
  /// \brief "mjc:flag:contact"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagContact;
  /// \brief "mjc:flag:damper"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagDamper;
  /// \brief "mjc:flag:energy"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagEnergy;
  /// \brief "mjc:flag:equality"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagEquality;
  /// \brief "mjc:flag:eulerdamp"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagEulerdamp;
  /// \brief "mjc:flag:filterparent"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagFilterparent;
  /// \brief "mjc:flag:frictionloss"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagFrictionloss;
  /// \brief "mjc:flag:fwdinv"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagFwdinv;
  /// \brief "mjc:flag:gravity"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagGravity;
  /// \brief "mjc:flag:invdiscrete"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagInvdiscrete;
  /// \brief "mjc:flag:island"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagIsland;
  /// \brief "mjc:flag:limit"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagLimit;
  /// \brief "mjc:flag:midphase"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagMidphase;
  /// \brief "mjc:flag:multiccd"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagMulticcd;
  /// \brief "mjc:flag:nativeccd"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagNativeccd;
  /// \brief "mjc:flag:override"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagOverride;
  /// \brief "mjc:flag:refsafe"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagRefsafe;
  /// \brief "mjc:flag:sensor"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagSensor;
  /// \brief "mjc:flag:spring"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagSpring;
  /// \brief "mjc:flag:warmstart"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagWarmstart;
  /// \brief "mjc:forceLimited"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcForceLimited;
  /// \brief "mjc:forceRange:max"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcForceRangeMax;
  /// \brief "mjc:forceRange:min"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcForceRangeMin;
  /// \brief "mjc:frictionloss"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcFrictionloss;
  /// \brief "mjc:gainPrm"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcGainPrm;
  /// \brief "mjc:gainType"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcGainType;
  /// \brief "mjc:gap"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcGap;
  /// \brief "mjc:gear"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcGear;
  /// \brief "mjc:group"
  ///
  /// MjcPhysicsSiteAPI, MjcPhysicsImageableAPI, MjcPhysicsCollisionAPI,
  /// MjcPhysicsActuator, MjcPhysicsJointAPI
  const TfToken mjcGroup;
  /// \brief "mjc:inertia"
  ///
  /// MjcPhysicsMeshCollisionAPI
  const TfToken mjcInertia;
  /// \brief "mjc:inheritRange"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcInheritRange;
  /// \brief "mjc:jointInParent"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcJointInParent;
  /// \brief "mjc:lengthRange:max"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcLengthRangeMax;
  /// \brief "mjc:lengthRange:min"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcLengthRangeMin;
  /// \brief "mjc:margin"
  ///
  /// MjcPhysicsCollisionAPI, MjcPhysicsJointAPI
  const TfToken mjcMargin;
  /// \brief "mjc:maxhullvert"
  ///
  /// MjcPhysicsMeshCollisionAPI
  const TfToken mjcMaxhullvert;
  /// \brief "mjc:mpos"
  ///
  /// MjcPhysicsKeyframe
  const TfToken mjcMpos;
  /// \brief "mjc:mquat"
  ///
  /// MjcPhysicsKeyframe
  const TfToken mjcMquat;
  /// \brief "mjc:option:actuatorgroupdisable"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionActuatorgroupdisable;
  /// \brief "mjc:option:apirate"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionApirate;
  /// \brief "mjc:option:ccd_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionCcd_iterations;
  /// \brief "mjc:option:ccd_tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionCcd_tolerance;
  /// \brief "mjc:option:cone"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionCone;
  /// \brief "mjc:option:density"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionDensity;
  /// \brief "mjc:option:impratio"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionImpratio;
  /// \brief "mjc:option:integrator"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionIntegrator;
  /// \brief "mjc:option:iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionIterations;
  /// \brief "mjc:option:jacobian"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionJacobian;
  /// \brief "mjc:option:ls_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionLs_iterations;
  /// \brief "mjc:option:ls_tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionLs_tolerance;
  /// \brief "mjc:option:magnetic"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionMagnetic;
  /// \brief "mjc:option:noslip_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionNoslip_iterations;
  /// \brief "mjc:option:noslip_tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionNoslip_tolerance;
  /// \brief "mjc:option:o_friction"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionO_friction;
  /// \brief "mjc:option:o_margin"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionO_margin;
  /// \brief "mjc:option:o_solimp"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionO_solimp;
  /// \brief "mjc:option:o_solref"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionO_solref;
  /// \brief "mjc:option:sdf_initpoints"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionSdf_initpoints;
  /// \brief "mjc:option:sdf_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionSdf_iterations;
  /// \brief "mjc:option:solver"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionSolver;
  /// \brief "mjc:option:timestep"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionTimestep;
  /// \brief "mjc:option:tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionTolerance;
  /// \brief "mjc:option:viscosity"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionViscosity;
  /// \brief "mjc:option:wind"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcOptionWind;
  /// \brief "mjc:priority"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcPriority;
  /// \brief "mjc:qpos"
  ///
  /// MjcPhysicsKeyframe
  const TfToken mjcQpos;
  /// \brief "mjc:qvel"
  ///
  /// MjcPhysicsKeyframe
  const TfToken mjcQvel;
  /// \brief "mjc:ref"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcRef;
  /// \brief "mjc:refSite"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcRefSite;
  /// \brief "mjc:rollingfriction"
  ///
  /// MjcPhysicsMaterialAPI
  const TfToken mjcRollingfriction;
  /// \brief "mjc:shellinertia"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcShellinertia;
  /// \brief "mjc:sliderSite"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcSliderSite;
  /// \brief "mjc:solimp"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcSolimp;
  /// \brief "mjc:solimpfriction"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcSolimpfriction;
  /// \brief "mjc:solimplimit"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcSolimplimit;
  /// \brief "mjc:solmix"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcSolmix;
  /// \brief "mjc:solref"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcSolref;
  /// \brief "mjc:solreffriction"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcSolreffriction;
  /// \brief "mjc:solreflimit"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcSolreflimit;
  /// \brief "mjc:springdamper"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcSpringdamper;
  /// \brief "mjc:springref"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcSpringref;
  /// \brief "mjc:stiffness"
  ///
  /// MjcPhysicsJointAPI
  const TfToken mjcStiffness;
  /// \brief "mjc:target"
  ///
  /// MjcPhysicsActuator
  const TfToken mjcTarget;
  /// \brief "mjc:torsionalfriction"
  ///
  /// MjcPhysicsMaterialAPI
  const TfToken mjcTorsionalfriction;
  /// \brief "muscle"
  ///
  /// Possible value for MjcPhysicsActuator::GetMjcBiasTypeAttr(), Possible
  /// value for MjcPhysicsActuator::GetMjcDynTypeAttr(), Possible value for
  /// MjcPhysicsActuator::GetMjcGainTypeAttr()
  const TfToken muscle;
  /// \brief "newton"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetSolverAttr(),  This token
  /// represents the Newton constraint solver algorithm.
  const TfToken newton;
  /// \brief "none"
  ///
  /// Fallback value for MjcPhysicsActuator::GetMjcBiasTypeAttr(), Fallback
  /// value for MjcPhysicsActuator::GetMjcDynTypeAttr()
  const TfToken none;
  /// \brief "pgs"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetSolverAttr(),  This token
  /// represents the PGS constraint solver algorithm.
  const TfToken pgs;
  /// \brief "pyramidal"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetConeAttr(),  This token
  /// represents the pyramidal contact friction cone type.
  const TfToken pyramidal;
  /// \brief "radian"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetAngleAttr()
  const TfToken radian;
  /// \brief "rk4"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetIntegratorAttr(),  This token
  /// represents the RK4 numerical integrator.
  const TfToken rk4;
  /// \brief "shell"
  ///
  /// Possible value for MjcPhysicsMeshCollisionAPI::GetInertiaAttr()
  const TfToken shell;
  /// \brief "sparse"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetJacobianAttr(),  This token
  /// represents the sparse constraint Jacobian and matrices computed from it.
  const TfToken sparse;
  /// \brief "true"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetInertiaFromGeomAttr(), Possible
  /// value for MjcPhysicsActuator::GetMjcActLimitedAttr(), Possible value for
  /// MjcPhysicsActuator::GetMjcCtrlLimitedAttr(), Possible value for
  /// MjcPhysicsActuator::GetMjcForceLimitedAttr(), Possible value for
  /// MjcPhysicsJointAPI::GetMjcActuatorfrclimitedAttr()
  const TfToken true_;
  /// \brief "user"
  ///
  /// Possible value for MjcPhysicsActuator::GetMjcBiasTypeAttr(), Possible
  /// value for MjcPhysicsActuator::GetMjcDynTypeAttr(), Possible value for
  /// MjcPhysicsActuator::GetMjcGainTypeAttr()
  const TfToken user;
  /// \brief "MjcActuator"
  ///
  /// Schema identifier and family for MjcPhysicsActuator
  const TfToken MjcActuator;
  /// \brief "MjcCollisionAPI"
  ///
  /// Schema identifier and family for MjcPhysicsCollisionAPI
  const TfToken MjcCollisionAPI;
  /// \brief "MjcImageableAPI"
  ///
  /// Schema identifier and family for MjcPhysicsImageableAPI
  const TfToken MjcImageableAPI;
  /// \brief "MjcJointAPI"
  ///
  /// Schema identifier and family for MjcPhysicsJointAPI
  const TfToken MjcJointAPI;
  /// \brief "MjcKeyframe"
  ///
  /// Schema identifier and family for MjcPhysicsKeyframe
  const TfToken MjcKeyframe;
  /// \brief "MjcMaterialAPI"
  ///
  /// Schema identifier and family for MjcPhysicsMaterialAPI
  const TfToken MjcMaterialAPI;
  /// \brief "MjcMeshCollisionAPI"
  ///
  /// Schema identifier and family for MjcPhysicsMeshCollisionAPI
  const TfToken MjcMeshCollisionAPI;
  /// \brief "MjcSceneAPI"
  ///
  /// Schema identifier and family for MjcPhysicsSceneAPI
  const TfToken MjcSceneAPI;
  /// \brief "MjcSiteAPI"
  ///
  /// Schema identifier and family for MjcPhysicsSiteAPI
  const TfToken MjcSiteAPI;
  /// A vector of all of the tokens listed above.
  const std::vector<TfToken> allTokens;
};

/// \var MjcPhysicsTokens
///
/// A global variable with static, efficient \link TfToken TfTokens\endlink
/// for use in all public USD API.  \sa MjcPhysicsTokensType
extern MJCPHYSICS_API TfStaticData<MjcPhysicsTokensType> MjcPhysicsTokens;

PXR_NAMESPACE_CLOSE_SCOPE

#endif
