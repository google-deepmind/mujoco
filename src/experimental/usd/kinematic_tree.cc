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

#include "experimental/usd/kinematic_tree.h"

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdPhysics/joint.h>

namespace mujoco {
namespace usd {

bool GetJointBodies(const pxr::UsdPhysicsJoint& joint,
                    const pxr::SdfPath& default_prim_path, pxr::SdfPath* from,
                    pxr::SdfPath* to) {
  pxr::SdfPathVector body1_paths;
  joint.GetBody1Rel().GetTargets(&body1_paths);
  if (body1_paths.empty()) {
    mju_warning("Joint %s does not have body1 rel. Skipping.",
                joint.GetPath().GetAsString().c_str());
    return false;
  } else if (body1_paths.size() > 1) {
    mju_warning("Joint %s has multiple body1 rels. Skipping.",
                joint.GetPath().GetAsString().c_str());
    return false;
  }
  *to = body1_paths[0];

  pxr::SdfPathVector body0_paths;
  joint.GetBody0Rel().GetTargets(&body0_paths);
  if (body0_paths.size() > 1) {
    mju_warning("Joint %s has multiple body0 rels. Skipping.",
                joint.GetPath().GetAsString().c_str());
    return false;
  }
  // Empty body0, or body0 pointing to the default prim means we'll attach
  // to the worldbody.
  if (body0_paths.empty() || body0_paths[0] == default_prim_path) {
    *from = pxr::SdfPath();
  } else {
    *from = body0_paths[0];
  }
  return true;
}

std::unique_ptr<KinematicNode> BuildKinematicTree(
    const std::vector<pxr::UsdPhysicsJoint>& joints,
    const std::vector<pxr::SdfPath>& all_body_paths,
    const pxr::SdfPath& default_prim_path) {
  std::map<pxr::SdfPath, std::vector<pxr::SdfPath>> children_map;
  std::map<pxr::SdfPath, pxr::SdfPath> parent_map;
  std::map<std::pair<pxr::SdfPath, pxr::SdfPath>, pxr::SdfPath>
      edge_to_joint_map;
  std::set<pxr::SdfPath> all_nodes(all_body_paths.begin(),
                                   all_body_paths.end());

  for (const auto& joint : joints) {
    pxr::SdfPath from, to;
    if (!GetJointBodies(joint, default_prim_path, &from, &to)) {
      continue;
    }

    auto edge_key = std::make_pair(from, to);
    auto it = edge_to_joint_map.find(edge_key);
    if (it == edge_to_joint_map.end()) {
      edge_to_joint_map[edge_key] = joint.GetPath();
    } else {
      mju_warning(
          "Multiple explicit joints defined between body %s and body %s. "
          "Joint1: %s, Joint2: %s. Keeping the first one found: %s",
          (from.IsEmpty() ? "<worldbody>" : from.GetString()).c_str(),
          to.GetString().c_str(), it->second.GetString().c_str(),
          joint.GetPath().GetString().c_str(), it->second.GetString().c_str());
      continue;
    }

    if (from == to) {
      mju_error("Self-loop detected at node %s", to.GetString().c_str());
      return nullptr;
    }
    if (parent_map.count(to)) {
      mju_error("Node %s has multiple parents ('%s' and '%s').",
                to.GetString().c_str(), parent_map.at(to).GetString().c_str(),
                from.GetString().c_str());
      return nullptr;
    }
    children_map[from].push_back(to);
    parent_map[to] = from;
    all_nodes.insert(from);
    all_nodes.insert(to);
  }

  // Sort children in children_map to respect the DFS order from the stage.
  for (auto& [_, children] : children_map) {
    std::sort(
        children.begin(), children.end(),
        [&v = all_body_paths](const auto& a, const auto& b) {
          return std::distance(v.begin(), std::find(v.begin(), v.end(), a)) <
                 std::distance(v.begin(), std::find(v.begin(), v.end(), b));
        });
  }

  // The world body is represented by an empty SdfPath.
  auto world_root = std::make_unique<KinematicNode>();
  std::map<pxr::SdfPath, KinematicNode*> node_map;
  node_map[pxr::SdfPath()] = world_root.get();

  // Use a deque for traversal. We will add roots to the back and children
  // to the front to perform a DFS on each root's tree.
  std::deque<pxr::SdfPath> q;

  // Add roots (floating-base bodies and children of the world) to the queue,
  // preserving the DFS order from the USD stage.
  for (const auto& body_path : all_body_paths) {
    if (!body_path.IsEmpty()) {
      const auto it = parent_map.find(body_path);
      // A root is a body that has no parent, or its parent is the world.
      if (it == parent_map.end() || it->second.IsEmpty()) {
        q.push_back(body_path);
      }
    }
  }

  while (!q.empty()) {
    pxr::SdfPath current_path = q.front();
    q.pop_front();

    pxr::SdfPath parent_path = parent_map.count(current_path)
                                   ? parent_map.at(current_path)
                                   : pxr::SdfPath();
    KinematicNode* parent_node = node_map.at(parent_path);

    auto new_node = std::make_unique<KinematicNode>();
    new_node->body_path = current_path;
    if (edge_to_joint_map.count({parent_path, current_path})) {
      new_node->joint_path = edge_to_joint_map.at({parent_path, current_path});
    }
    node_map[current_path] = new_node.get();
    parent_node->children.push_back(std::move(new_node));

    if (children_map.count(current_path)) {
      const auto& children = children_map.at(current_path);
      // Add children to the front of the queue in reverse order to ensure
      // they are processed in the correct order by the DFS.
      for (auto it = children.rbegin(); it != children.rend(); ++it) {
        q.push_front(*it);
      }
    }
  }

  // After traversal, check for unvisited nodes.
  // Unvisited nodes at this point imply a cycle.
  for (const auto& node : all_nodes) {
    if (!node.IsEmpty() && !node_map.count(node)) {
      mju_error("Cycle detected involving node %s.", node.GetString().c_str());
      return nullptr;
    }
  }

  return world_root;
}
}  // namespace usd
}  // namespace mujoco
