// Copyright 2026 DeepMind Technologies Limited
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

#include "collision_filtering.h"

#include <iterator>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include "kinematic_tree.h"
#include "newton_tokens.h"
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primFlags.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/filteredPairsAPI.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

namespace {

using PathSet = std::set<pxr::SdfPath>;
using BodyGeoms = std::map<pxr::SdfPath, PathSet>;

struct FilterContext {
  BodyGeoms body_geoms;
  std::map<pxr::SdfPath, pxr::SdfPath> geom_bodies;
  std::map<pxr::SdfPath, int> body_components;
  std::map<pxr::SdfPath, int> joint_components;
  std::vector<PathSet> components;
  std::set<std::pair<pxr::SdfPath, pxr::SdfPath>> excludes;
};

bool JointEnabled(const pxr::UsdStageRefPtr& stage,
                  const pxr::SdfPath& joint_path) {
  bool enabled = true;
  pxr::UsdPhysicsJoint::Get(stage, joint_path)
      .GetJointEnabledAttr()
      .Get(&enabled);
  return enabled;
}

// The world never joins components. Excluded joints are already absent from the
// kinematic tree; disabled joints must not connect articulation members either.
void CollectBodies(const pxr::UsdStageRefPtr& stage, mjSpec* spec,
                   const Node& node, int component, FilterContext& context) {
  if (!node.body_path.IsEmpty()) {
    bool connected = false;
    for (const auto& joint : node.joints) {
      connected |= JointEnabled(stage, joint);
    }
    if (component < 0 || !connected) {
      component = context.components.size();
      context.components.emplace_back();
    }
    context.body_components[node.body_path] = component;
    context.components[component].insert(node.body_path);
    for (const auto& joint : node.joints) {
      if (JointEnabled(stage, joint)) {
        context.joint_components[joint] = component;
      }
    }
  }

  auto& geoms = context.body_geoms[node.body_path];
  // Check the parsed collision masks rather than the USD geom category when
  // deciding whether an endpoint covers a whole body. An exclude must not
  // suppress collisions which the USD relationship did not mention.
  for (const auto* paths : {&node.colliders, &node.visual_gprims}) {
    for (const auto& path : *paths) {
      auto* geom =
          mjs_asGeom(mjs_findElement(spec, mjOBJ_GEOM, path.GetText()));
      if (geom) {
        context.geom_bodies[path] = node.body_path;
        if (geom->contype || geom->conaffinity) {
          geoms.insert(path);
        }
      }
    }
  }
  for (const auto& child : node.children) {
    CollectBodies(stage, spec, *child, component, context);
  }
}

std::set<int> ArticulationComponents(const pxr::UsdPrim& prim,
                                     const FilterContext& context) {
  const auto& path = prim.GetPath();
  if (prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()) {
    auto body = context.body_components.find(path);
    return body == context.body_components.end() ? std::set<int>{}
                                                 : std::set<int>{body->second};
  }
  if (prim.IsA<pxr::UsdPhysicsJoint>()) {
    auto joint = context.joint_components.find(path);
    return joint == context.joint_components.end()
               ? std::set<int>{}
               : std::set<int>{joint->second};
  }

  // An ancestor prim can identify multiple articulations. Seed from bodies and
  // root joints in its namespace, then include their entire joint components,
  // including bodies outside that namespace.
  std::set<int> components;
  for (const auto& [body, component] : context.body_components) {
    if (body.HasPrefix(path)) {
      components.insert(component);
    }
  }
  for (const auto& [joint, component] : context.joint_components) {
    if (joint.HasPrefix(path)) {
      components.insert(component);
    }
  }
  return components;
}

bool ResolveEndpoint(const pxr::UsdPrim& prim, const FilterContext& context,
                     BodyGeoms& geoms) {
  if (!prim) {
    return false;
  }
  if (prim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>()) {
    for (int component : ArticulationComponents(prim, context)) {
      for (const auto& body : context.components[component]) {
        geoms[body] = context.body_geoms.at(body);
      }
    }
    return true;
  }
  if (prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()) {
    auto body = context.body_geoms.find(prim.GetPath());
    if (body == context.body_geoms.end()) {
      return false;
    }
    geoms.insert(*body);
    return true;
  }
  if (prim.HasAPI<pxr::UsdPhysicsCollisionAPI>()) {
    auto owner = context.geom_bodies.find(prim.GetPath());
    if (owner == context.geom_bodies.end()) {
      return false;
    }
    if (context.body_geoms.at(owner->second).count(prim.GetPath())) {
      geoms[owner->second].insert(prim.GetPath());
    }
    return true;
  }
  return false;
}

void AddExclude(mjSpec* spec, pxr::SdfPath body1, pxr::SdfPath body2,
                FilterContext& context) {
  if (body1 == body2) {
    return;
  }
  if (body2 < body1) {
    std::swap(body1, body2);
  }
  if (!context.excludes.emplace(body1, body2).second) {
    return;
  }
  mjsExclude* exclude = mjs_addExclude(spec);
  mjs_setString(exclude->bodyname1,
                body1.IsEmpty() ? "world" : body1.GetText());
  mjs_setString(exclude->bodyname2,
                body2.IsEmpty() ? "world" : body2.GetText());
}

void ParseFilteredPairs(const pxr::UsdPrim& prim, mjSpec* spec,
                        FilterContext& context) {
  pxr::SdfPathVector targets;
  pxr::UsdPhysicsFilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets(
      &targets);
  if (targets.empty()) {
    return;
  }
  BodyGeoms source;
  if (!ResolveEndpoint(prim, context, source)) {
    mju_warning("Cannot resolve filtered pair endpoint '%s'; skipping.",
                prim.GetPath().GetText());
    return;
  }
  for (const auto& target : targets) {
    BodyGeoms destination;
    if (!ResolveEndpoint(prim.GetStage()->GetPrimAtPath(target), context,
                         destination)) {
      mju_warning(
          "Cannot resolve filtered pair endpoint '%s' on '%s'; skipping.",
          target.GetText(), prim.GetPath().GetText());
      continue;
    }
    bool partial = false;
    for (const auto& [body1, geoms1] : source) {
      for (const auto& [body2, geoms2] : destination) {
        if (body1 == body2 || geoms1.empty() || geoms2.empty()) {
          continue;
        }
        if (geoms1 == context.body_geoms.at(body1) &&
            geoms2 == context.body_geoms.at(body2)) {
          AddExclude(spec, body1, body2, context);
        } else {
          partial = true;
        }
      }
    }
    if (partial) {
      mju_warning(
          "Cannot represent filtered pair '%s' -> '%s' with body "
          "excludes without filtering other colliders; skipping "
          "collider-level pairs.",
          prim.GetPath().GetText(), target.GetText());
    }
  }
}

void ParseNewtonArticulation(const pxr::UsdPrim& prim, mjSpec* spec,
                             FilterContext& context) {
  auto add_mobility_attr =
      prim.GetAttribute(mujoco::NewtonTokens->newtonJointsAddMobility);
  bool add_mobility = true;
  if (add_mobility_attr && add_mobility_attr.HasAuthoredValue() &&
      add_mobility_attr.Get(&add_mobility) && !add_mobility) {
    mju_warning(
        "Prim '%s' has newton:jointsAddMobility set to false, but MuJoCo "
        "only supports reduced-coordinate formulations where joints add "
        "mobility. Joints will be treated as adding mobility.",
        prim.GetPath().GetText());
  }

  auto self_collision_attr =
      prim.GetAttribute(mujoco::NewtonTokens->newtonSelfCollisionEnabled);
  bool self_collision = true;
  if (!self_collision_attr || !self_collision_attr.Get(&self_collision) ||
      self_collision) {
    return;
  }
  for (int component : ArticulationComponents(prim, context)) {
    const auto& bodies = context.components[component];
    for (auto body1 = bodies.begin(); body1 != bodies.end(); ++body1) {
      for (auto body2 = std::next(body1); body2 != bodies.end(); ++body2) {
        AddExclude(spec, *body1, *body2, context);
      }
    }
  }
}

}  // namespace

void ParseCollisionFiltering(const pxr::UsdStageRefPtr& stage, mjSpec* spec,
                             const Node& root) {
  FilterContext context;
  CollectBodies(stage, spec, root, -1, context);
  for (const auto& prim : pxr::UsdPrimRange(
           stage->GetPseudoRoot(), pxr::UsdTraverseInstanceProxies())) {
    if (prim.HasAPI<pxr::UsdPhysicsFilteredPairsAPI>()) {
      ParseFilteredPairs(prim, spec, context);
    }
    if (prim.HasAPI(mujoco::NewtonTokens->NewtonArticulationRootAPI)) {
      ParseNewtonArticulation(prim, spec, context);
    }
  }
}
