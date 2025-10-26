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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_KINEMATIC_TREE_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_KINEMATIC_TREE_H_

#include <memory>
#include <vector>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdPhysics/joint.h>

namespace mujoco {
namespace usd {

// A struct to represent a node in the kinematic tree.
// Using a struct with a vector of children preserves the order of bodies,
// which is important for things like keyframes and policy compatibility.
struct Node {
  pxr::SdfPath body_path;
  pxr::SdfPath physics_scene;
  std::vector<pxr::SdfPath> actuators;
  std::vector<pxr::SdfPath> joints;
  std::vector<pxr::SdfPath> visual_gprims;
  std::vector<pxr::SdfPath> colliders;
  std::vector<pxr::SdfPath> sites;
  std::vector<pxr::SdfPath> tendons;
  std::vector<pxr::SdfPath> keyframes;
  std::vector<std::unique_ptr<Node>> children;
};

// A kinematic edge represents a joint.
using JointVec = std::vector<pxr::UsdPhysicsJoint>;

// Builds a single kinematic tree from a list of directed edges.
// The DFS order of bodies in the tree is determined by the order of bodies in
// `all_body_paths`.
// All bodies, including static and floating-base bodies, are organized under a
// single world root. An empty 'from' path in an edge represents the world body.
// Returns the root of the kinematic tree, or `nullptr` for invalid structures.
std::unique_ptr<Node> BuildKinematicTree(const pxr::UsdStageRefPtr stage);

}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_KINEMATIC_TREE_H_
