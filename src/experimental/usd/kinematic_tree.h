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
struct KinematicNode {
  pxr::SdfPath body_path;
  pxr::SdfPath joint_path;  // Joint connecting this node to its parent.
  std::vector<std::unique_ptr<KinematicNode>> children;
};

// Builds a single kinematic tree from a list of joints.
// The DFS order of bodies in the tree is determined by the order of bodies in
// `all_body_paths`.
// All bodies, including static and floating-base bodies, are organized under a
// single world root. An empty 'from' path in an edge represents the world body.
// Returns the root of the kinematic tree, or `nullptr` for invalid structures.
std::unique_ptr<KinematicNode> BuildKinematicTree(
    const std::vector<pxr::UsdPhysicsJoint>& joints,
    const std::vector<pxr::SdfPath>& all_body_paths,
    const pxr::SdfPath& default_prim_path);

}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_KINEMATIC_TREE_H_
