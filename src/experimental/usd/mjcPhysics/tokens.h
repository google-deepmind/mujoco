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

#include "./api.h"
#include "pxr/base/tf/staticData.h"
#include "pxr/base/tf/token.h"
#include "pxr/pxr.h"

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
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcBiasTypeAttr(), Possible
  /// value for MjcPhysicsActuatorAPI::GetMjcGainTypeAttr()
  const TfToken affine;
  /// \brief "auto"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetJacobianAttr(), Fallback value
  /// for MjcPhysicsActuatorAPI::GetMjcActLimitedAttr(), Fallback value for
  /// MjcPhysicsActuatorAPI::GetMjcCtrlLimitedAttr(), Fallback value for
  /// MjcPhysicsActuatorAPI::GetMjcForceLimitedAttr(),  This token represents
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
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcActLimitedAttr(), Possible
  /// value for MjcPhysicsActuatorAPI::GetMjcCtrlLimitedAttr(), Possible value
  /// for MjcPhysicsActuatorAPI::GetMjcForceLimitedAttr()
  const TfToken false_;
  /// \brief "filter"
  ///
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcDynTypeAttr()
  const TfToken filter;
  /// \brief "filterexact"
  ///
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcDynTypeAttr()
  const TfToken filterexact;
  /// \brief "fixed"
  ///
  /// Fallback value for MjcPhysicsActuatorAPI::GetMjcGainTypeAttr()
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
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcDynTypeAttr()
  const TfToken integrator;
  /// \brief "legacy"
  ///
  /// Fallback value for MjcPhysicsMeshCollisionAPI::GetInertiaAttr()
  const TfToken legacy;
  /// \brief "mjc:actDim"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcActDim;
  /// \brief "mjc:actEarly"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcActEarly;
  /// \brief "mjc:actLimited"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcActLimited;
  /// \brief "mjc:actRange:max"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcActRangeMax;
  /// \brief "mjc:actRange:min"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcActRangeMin;
  /// \brief "mjc:biasPrm"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcBiasPrm;
  /// \brief "mjc:biasType"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcBiasType;
  /// \brief "mjc:crankLength"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcCrankLength;
  /// \brief "mjc:crankSite"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcCrankSite;
  /// \brief "mjc:ctrlLimited"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcCtrlLimited;
  /// \brief "mjc:ctrlRange:max"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcCtrlRangeMax;
  /// \brief "mjc:ctrlRange:min"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcCtrlRangeMin;
  /// \brief "mjc:dynPrm"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcDynPrm;
  /// \brief "mjc:dynType"
  ///
  /// MjcPhysicsActuatorAPI
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
  /// \brief "mjc:flag:passive"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagPassive;
  /// \brief "mjc:flag:refsafe"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagRefsafe;
  /// \brief "mjc:flag:sensor"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagSensor;
  /// \brief "mjc:flag:warmstart"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcFlagWarmstart;
  /// \brief "mjc:forceLimited"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcForceLimited;
  /// \brief "mjc:forceRange:max"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcForceRangeMax;
  /// \brief "mjc:forceRange:min"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcForceRangeMin;
  /// \brief "mjc:gainPrm"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcGainPrm;
  /// \brief "mjc:gainType"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcGainType;
  /// \brief "mjc:gear"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcGear;
  /// \brief "mjc:inertia"
  ///
  /// MjcPhysicsMeshCollisionAPI
  const TfToken mjcInertia;
  /// \brief "mjc:jointInParent"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcJointInParent;
  /// \brief "mjc:lengthRange:max"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcLengthRangeMax;
  /// \brief "mjc:lengthRange:min"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcLengthRangeMin;
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
  /// \brief "mjc:refSite"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcRefSite;
  /// \brief "mjc:shellinertia"
  ///
  /// MjcPhysicsCollisionAPI
  const TfToken mjcShellinertia;
  /// \brief "mjc:sliderSite"
  ///
  /// MjcPhysicsActuatorAPI
  const TfToken mjcSliderSite;
  /// \brief "muscle"
  ///
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcBiasTypeAttr(), Possible
  /// value for MjcPhysicsActuatorAPI::GetMjcDynTypeAttr(), Possible value for
  /// MjcPhysicsActuatorAPI::GetMjcGainTypeAttr()
  const TfToken muscle;
  /// \brief "newton"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetSolverAttr(),  This token
  /// represents the Newton constraint solver algorithm.
  const TfToken newton;
  /// \brief "none"
  ///
  /// Fallback value for MjcPhysicsActuatorAPI::GetMjcBiasTypeAttr(), Fallback
  /// value for MjcPhysicsActuatorAPI::GetMjcDynTypeAttr()
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
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcActLimitedAttr(), Possible
  /// value for MjcPhysicsActuatorAPI::GetMjcCtrlLimitedAttr(), Possible value
  /// for MjcPhysicsActuatorAPI::GetMjcForceLimitedAttr()
  const TfToken true_;
  /// \brief "user"
  ///
  /// Possible value for MjcPhysicsActuatorAPI::GetMjcBiasTypeAttr(), Possible
  /// value for MjcPhysicsActuatorAPI::GetMjcDynTypeAttr(), Possible value for
  /// MjcPhysicsActuatorAPI::GetMjcGainTypeAttr()
  const TfToken user;
  /// \brief "CollisionAPI"
  ///
  /// Schema identifer and family for MjcPhysicsCollisionAPI
  const TfToken CollisionAPI;
  /// \brief "MeshCollisionAPI"
  ///
  /// Schema identifer and family for MjcPhysicsMeshCollisionAPI
  const TfToken MeshCollisionAPI;
  /// \brief "PhysicsActuatorAPI"
  ///
  /// Schema identifer and family for MjcPhysicsActuatorAPI
  const TfToken PhysicsActuatorAPI;
  /// \brief "SceneAPI"
  ///
  /// Schema identifer and family for MjcPhysicsSceneAPI
  const TfToken SceneAPI;
  /// \brief "SiteAPI"
  ///
  /// Schema identifer and family for MjcPhysicsSiteAPI
  const TfToken SiteAPI;
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
