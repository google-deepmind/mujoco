# Copyright 2026 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.types import ConstraintType
from mujoco.mjx.third_party.mujoco_warp._src.types import EqType
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType


@wp.kernel
def _tree_edges(
  # Model:
  nv: int,
  body_treeid: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  dof_treeid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  eq_type: wp.array(dtype=int),
  eq_obj1id: wp.array(dtype=int),
  eq_obj2id: wp.array(dtype=int),
  eq_objtype: wp.array(dtype=int),
  # Data in:
  nefc_in: wp.array(dtype=int),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  njmax_in: int,
  # Out:
  tree_tree: wp.array3d(dtype=int),  # kernel_analyzer: off
):
  """Find tree edges. Launch: (nworld, njmax)."""
  worldid, efcid = wp.tid()

  # skip if beyond active constraints
  if efcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  efc_type = efc_type_in[worldid, efcid]
  efc_id = efc_id_in[worldid, efcid]

  tree0 = int(-1)
  tree1 = int(-1)
  use_generic = int(0)

  # equality (connect/weld)
  if efc_type == ConstraintType.EQUALITY:
    eq_t = eq_type[efc_id]

    if eq_t == EqType.CONNECT or eq_t == EqType.WELD:
      b1 = eq_obj1id[efc_id]
      b2 = eq_obj2id[efc_id]

      # site semantics
      if eq_objtype[efc_id] == ObjType.SITE:
        b1 = site_bodyid[b1]
        b2 = site_bodyid[b2]

      tree0 = body_treeid[b1]
      tree1 = body_treeid[b2]
    else:
      # JOINT, TENDON, FLEX
      use_generic = 1

  # joint friction
  elif efc_type == ConstraintType.FRICTION_DOF:
    tree0 = dof_treeid[efc_id]

  # joint limit
  elif efc_type == ConstraintType.LIMIT_JOINT:
    tree0 = dof_treeid[jnt_dofadr[efc_id]]

  # contact
  elif (
    efc_type == ConstraintType.CONTACT_FRICTIONLESS
    or efc_type == ConstraintType.CONTACT_PYRAMIDAL
    or efc_type == ConstraintType.CONTACT_ELLIPTIC
  ):
    geom_pair = contact_geom_in[efc_id]
    g1 = geom_pair[0]
    g2 = geom_pair[1]

    # flex contacts have negative geom ids
    if g1 >= 0 and g2 >= 0:
      tree0 = body_treeid[geom_bodyid[g1]]
      tree1 = body_treeid[geom_bodyid[g2]]
    else:
      use_generic = 1

  # generic
  else:
    use_generic = 1

  # handle static bodies
  if use_generic == 0:
    # swap so tree0 is non-negative if possible
    if tree0 < 0 and tree1 >= 0:
      tree0 = tree1
      tree1 = -1

    # mark the edge
    if tree0 >= 0:
      if tree1 < 0 or tree0 == tree1:
        # self-edge
        wp.atomic_max(tree_tree, worldid, tree0, tree0, 1)
      else:
        # cross-tree edge
        t1 = wp.min(tree0, tree1)
        t2 = wp.max(tree0, tree1)
        wp.atomic_max(tree_tree, worldid, t1, t2, 1)
        wp.atomic_max(tree_tree, worldid, t2, t1, 1)
    return

  # generic: scan Jacobian row
  first_tree = int(-1)
  has_cross_edge = int(0)

  for dof in range(nv):
    # TODO(team): sparse efc_J
    # TODO(team): tree dof skip
    J_val = efc_J_in[worldid, efcid, dof]
    if J_val != 0.0:
      tree = dof_treeid[dof]
      if tree < 0:
        continue
      if first_tree == -1:
        first_tree = tree
      elif tree != first_tree:
        t1 = wp.min(first_tree, tree)
        t2 = wp.max(first_tree, tree)
        wp.atomic_max(tree_tree, worldid, t1, t2, 1)
        has_cross_edge = 1

  if first_tree >= 0 and has_cross_edge == 0:
    wp.atomic_max(tree_tree, worldid, first_tree, first_tree, 1)


def tree_edges(m: types.Model, d: types.Data, tree_tree: wp.array3d(dtype=int)):
  """Compute tree-tree adjacency matrix."""
  tree_tree.zero_()
  wp.launch(
    kernel=_tree_edges,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.nv,
      m.body_treeid,
      m.jnt_dofadr,
      m.dof_treeid,
      m.geom_bodyid,
      m.site_bodyid,
      m.eq_type,
      m.eq_obj1id,
      m.eq_obj2id,
      m.eq_objtype,
      d.nefc,
      d.contact.geom,
      d.efc.type,
      d.efc.id,
      d.efc.J,
      d.njmax,
    ],
    outputs=[tree_tree],
  )
