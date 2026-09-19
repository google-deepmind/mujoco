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

// Shared static tokens for Newton USD Schemas.

#ifndef MUJOCO_PLUGIN_USD_DECODER_NEWTON_TOKENS_H_
#define MUJOCO_PLUGIN_USD_DECODER_NEWTON_TOKENS_H_

#include <pxr/base/tf/staticTokens.h>
#include <pxr/pxr.h>

PXR_NAMESPACE_OPEN_SCOPE

// clang-format off
#define NEWTON_TOKENS \
  ((NewtonMaterialAPI, "NewtonMaterialAPI")) \
  ((NewtonMeshCollisionAPI, "NewtonMeshCollisionAPI")) \
  ((NewtonJointAPI, "NewtonJointAPI")) \
  ((NewtonMassAPI, "NewtonMassAPI")) \
  ((NewtonSiteAPI, "NewtonSiteAPI")) \
  ((NewtonMimicAPI, "NewtonMimicAPI")) \
  ((NewtonCollisionAPI, "NewtonCollisionAPI")) \
  ((NewtonArticulationRootAPI, "NewtonArticulationRootAPI")) \
  ((newtonMassModel, "newton:massModel")) \
  ((newtonMaxSolverIterations, "newton:maxSolverIterations")) \
  ((newtonTimeStepsPerSecond, "newton:timeStepsPerSecond")) \
  ((newtonGravityEnabled, "newton:gravityEnabled")) \
  ((newtonContactMargin, "newton:contactMargin")) \
  ((newtonContactGap, "newton:contactGap")) \
  ((newtonMaxHullVertices, "newton:maxHullVertices")) \
  ((newtonTorsionalFriction, "newton:torsionalFriction")) \
  ((newtonRollingFriction, "newton:rollingFriction")) \
  ((newtonArmature, "newton:armature")) \
  ((newtonDamping, "newton:damping")) \
  ((newtonFriction, "newton:friction")) \
  ((newtonMimicEnabled, "newton:mimicEnabled")) \
  ((newtonMimicJoint, "newton:mimicJoint")) \
  ((newtonMimicCoef0, "newton:mimicCoef0")) \
  ((newtonMimicCoef1, "newton:mimicCoef1")) \
  ((newtonInertia, "newton:inertia")) \
  ((newtonContactAdhesion, "newton:contactAdhesion")) \
  ((newtonJointsAddMobility, "newton:jointsAddMobility")) \
  ((newtonSelfCollisionEnabled, "newton:selfCollisionEnabled"))
// clang-format on

TF_DECLARE_PUBLIC_TOKENS(NewtonTokens, NEWTON_TOKENS);

PXR_NAMESPACE_CLOSE_SCOPE

namespace mujoco {

using pxr::NewtonTokens;

}  // namespace mujoco

#endif  // MUJOCO_PLUGIN_USD_DECODER_NEWTON_TOKENS_H_
