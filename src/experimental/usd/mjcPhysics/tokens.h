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
///     gprim.GetMyTokenValuedAttr().Set(MjcPhysicsTokens->auto_);
/// \endcode
struct MjcPhysicsTokensType {
  MJCPHYSICS_API MjcPhysicsTokensType();
  /// \brief "auto"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetJacobianAttr(),  This token
  /// represents the auto constraint Jacobian and matrices computed from it.
  const TfToken auto_;
  /// \brief "cg"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetSolverAttr(),  This token
  /// represents the CG constraint solver algorithm.
  const TfToken cg;
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
  /// \brief "mjc:physics:actuatorgroupdisable"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsActuatorgroupdisable;
  /// \brief "mjc:physics:apirate"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsApirate;
  /// \brief "mjc:physics:ccd_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsCcd_iterations;
  /// \brief "mjc:physics:ccd_tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsCcd_tolerance;
  /// \brief "mjc:physics:cone"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsCone;
  /// \brief "mjc:physics:density"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsDensity;
  /// \brief "mjc:physics:flag:actuation"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagActuation;
  /// \brief "mjc:physics:flag:autoreset"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagAutoreset;
  /// \brief "mjc:physics:flag:clampctrl"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagClampctrl;
  /// \brief "mjc:physics:flag:constraint"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagConstraint;
  /// \brief "mjc:physics:flag:contact"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagContact;
  /// \brief "mjc:physics:flag:energy"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagEnergy;
  /// \brief "mjc:physics:flag:equality"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagEquality;
  /// \brief "mjc:physics:flag:eulerdamp"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagEulerdamp;
  /// \brief "mjc:physics:flag:filterparent"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagFilterparent;
  /// \brief "mjc:physics:flag:frictionloss"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagFrictionloss;
  /// \brief "mjc:physics:flag:fwdinv"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagFwdinv;
  /// \brief "mjc:physics:flag:gravity"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagGravity;
  /// \brief "mjc:physics:flag:invdiscrete"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagInvdiscrete;
  /// \brief "mjc:physics:flag:island"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagIsland;
  /// \brief "mjc:physics:flag:limit"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagLimit;
  /// \brief "mjc:physics:flag:midphase"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagMidphase;
  /// \brief "mjc:physics:flag:multiccd"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagMulticcd;
  /// \brief "mjc:physics:flag:nativeccd"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagNativeccd;
  /// \brief "mjc:physics:flag:override"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagOverride;
  /// \brief "mjc:physics:flag:passive"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagPassive;
  /// \brief "mjc:physics:flag:refsafe"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagRefsafe;
  /// \brief "mjc:physics:flag:sensor"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagSensor;
  /// \brief "mjc:physics:flag:warmstart"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsFlagWarmstart;
  /// \brief "mjc:physics:impratio"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsImpratio;
  /// \brief "mjc:physics:integrator"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsIntegrator;
  /// \brief "mjc:physics:iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsIterations;
  /// \brief "mjc:physics:jacobian"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsJacobian;
  /// \brief "mjc:physics:ls_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsLs_iterations;
  /// \brief "mjc:physics:ls_tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsLs_tolerance;
  /// \brief "mjc:physics:magnetic"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsMagnetic;
  /// \brief "mjc:physics:noslip_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsNoslip_iterations;
  /// \brief "mjc:physics:noslip_tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsNoslip_tolerance;
  /// \brief "mjc:physics:o_friction"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsO_friction;
  /// \brief "mjc:physics:o_margin"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsO_margin;
  /// \brief "mjc:physics:o_solimp"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsO_solimp;
  /// \brief "mjc:physics:o_solref"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsO_solref;
  /// \brief "mjc:physics:sdf_initpoints"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsSdf_initpoints;
  /// \brief "mjc:physics:sdf_iterations"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsSdf_iterations;
  /// \brief "mjc:physics:solver"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsSolver;
  /// \brief "mjc:physics:timestep"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsTimestep;
  /// \brief "mjc:physics:tolerance"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsTolerance;
  /// \brief "mjc:physics:viscosity"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsViscosity;
  /// \brief "mjc:physics:wind"
  ///
  /// MjcPhysicsSceneAPI
  const TfToken mjcPhysicsWind;
  /// \brief "newton"
  ///
  /// Fallback value for MjcPhysicsSceneAPI::GetSolverAttr(),  This token
  /// represents the Newton constraint solver algorithm.
  const TfToken newton;
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
  /// \brief "sparse"
  ///
  /// Possible value for MjcPhysicsSceneAPI::GetJacobianAttr(),  This token
  /// represents the sparse constraint Jacobian and matrices computed from it.
  const TfToken sparse;
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
