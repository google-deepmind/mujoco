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
from mujoco.mjx.third_party.mujoco_warp._src.types import IslandSolverContext
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope


@wp.kernel
def _tree_edges(
  # Model:
  nv: int,
  body_treeid: wp.array[int],
  jnt_dofadr: wp.array[int],
  dof_treeid: wp.array[int],
  geom_bodyid: wp.array[int],
  site_bodyid: wp.array[int],
  eq_type: wp.array[int],
  eq_obj1id: wp.array[int],
  eq_obj2id: wp.array[int],
  eq_objtype: wp.array[int],
  is_sparse: bool,
  # Data in:
  nefc_in: wp.array[int],
  contact_geom_in: wp.array[wp.vec2i],
  efc_type_in: wp.array2d[int],
  efc_id_in: wp.array2d[int],
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  njmax_in: int,
  # Out:
  tree_tree: wp.array3d[int],  # kernel_analyzer: off
):
  """Find tree edges."""
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

  count = nv
  rowadr = 0
  if is_sparse:
    count = efc_J_rownnz_in[worldid, efcid]
    rowadr = efc_J_rowadr_in[worldid, efcid]

  for i in range(count):
    dof = i
    if is_sparse:
      sparseid = rowadr + i
      dof = efc_J_colind_in[worldid, 0, sparseid]
    else:
      J_val = efc_J_in[worldid, efcid, dof]
      if J_val == 0.0:
        continue

    tree = dof_treeid[dof]
    if tree < 0:
      continue
    if first_tree == -1:
      first_tree = tree
    elif tree != first_tree:
      t1 = wp.min(first_tree, tree)
      t2 = wp.max(first_tree, tree)
      wp.atomic_max(tree_tree, worldid, t1, t2, 1)
      wp.atomic_max(tree_tree, worldid, t2, t1, 1)
      has_cross_edge = 1

  if first_tree >= 0 and has_cross_edge == 0:
    wp.atomic_max(tree_tree, worldid, first_tree, first_tree, 1)


@event_scope
def tree_edges(m: types.Model, d: types.Data, tree_tree: wp.array3d[int]):
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
      m.is_sparse,
      d.nefc,
      d.contact.geom,
      d.efc.type,
      d.efc.id,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.J,
      d.njmax,
    ],
    outputs=[tree_tree],
  )


@wp.kernel
def _flood_fill(
  # Model:
  ntree: int,
  # In:
  tree_tree_in: wp.array3d[int],
  labels_in: wp.array2d[int],
  stack_in: wp.array2d[int],
  # Data out:
  nisland_out: wp.array[int],
  tree_island_out: wp.array2d[int],
  # Out:
  stack_out: wp.array2d[int],
):
  """DFS flood fill to discover islands using tree_tree matrix."""
  worldid = wp.tid()
  nisland = int(0)

  # iterate over trees
  for i in range(ntree):
    # already assigned
    if labels_in[worldid, i] != -1:
      continue

    # check if tree has any edges
    has_edge = int(0)
    for j in range(ntree):
      if tree_tree_in[worldid, i, j] != 0:
        has_edge = 1
        break
    if has_edge == 0:
      continue

    # DFS: push i onto stack
    nstack = int(0)
    stack_out[worldid, nstack] = i
    nstack = nstack + 1

    while nstack > 0:
      # pop v from stack
      nstack = nstack - 1
      v = stack_in[worldid, nstack]

      # already assigned
      if labels_in[worldid, v] != -1:
        continue

      # assign to current island
      tree_island_out[worldid, v] = nisland

      # push neighbors
      for neighbor in range(ntree):
        if tree_tree_in[worldid, v, neighbor] != 0:
          if labels_in[worldid, neighbor] == -1:
            stack_out[worldid, nstack] = neighbor
            nstack = nstack + 1

    # island filled
    nisland = nisland + 1

  nisland_out[worldid] = nisland


@event_scope
def flood_fill(m: types.Model, d: types.Data, tree_tree: wp.array3d[int]):
  d.tree_island.fill_(-1)
  stack_scratch = wp.empty((d.nworld, m.ntree * m.ntree), dtype=int)

  wp.launch(
    _flood_fill,
    dim=d.nworld,
    inputs=[m.ntree, tree_tree, d.tree_island, stack_scratch],
    outputs=[d.nisland, d.tree_island, stack_scratch],
  )


@event_scope
def island(m: types.Model, d: types.Data):
  """Discover constraint islands."""
  if m.ntree == 0:
    d.nisland.zero_()
    return

  # Step 1: Find tree edges
  tree_tree = wp.zeros((d.nworld, m.ntree, m.ntree), dtype=int)
  tree_edges(m, d, tree_tree)

  # Step 2: DFS flood fill
  flood_fill(m, d, tree_tree)


@wp.kernel
def _island_count_dofs(
  dof_treeid: wp.array[int],
  tree_island_in: wp.array2d[int],
  dof_island_out: wp.array2d[int],
  island_nv_out: wp.array2d[int],
):
  worldid, dofid = wp.tid()

  island_id = tree_island_in[worldid, dof_treeid[dofid]]
  dof_island_out[worldid, dofid] = island_id
  if island_id >= 0:
    wp.atomic_add(island_nv_out, worldid, island_id, 1)


@wp.kernel
def _island_scan_sizes(
  nisland_in: wp.array[int],
  island_idofadr_out: wp.array2d[int],
  island_nv_inout: wp.array2d[int],
  island_nefc_inout: wp.array2d[int],
  island_iefcadr_out: wp.array2d[int],
  nidof_out: wp.array[int],
):
  worldid = wp.tid()

  nisland = nisland_in[worldid]
  if nisland == 0:
    nidof_out[worldid] = 0
    return

  # Scan DOFs and Constraints
  island_idofadr_out[worldid, 0] = 0
  island_iefcadr_out[worldid, 0] = 0
  for i in range(1, nisland):
    island_idofadr_out[worldid, i] = island_idofadr_out[worldid, i - 1] + island_nv_inout[worldid, i - 1]
    island_iefcadr_out[worldid, i] = island_iefcadr_out[worldid, i - 1] + island_nefc_inout[worldid, i - 1]

  nidof = island_idofadr_out[worldid, nisland - 1] + island_nv_inout[worldid, nisland - 1]
  nidof_out[worldid] = nidof

  # Reset for recount
  for i in range(nisland):
    island_nv_inout[worldid, i] = 0
    island_nefc_inout[worldid, i] = 0


@wp.kernel
def _island_map_dofs(
  nv: int,
  dof_island_in: wp.array2d[int],
  island_idofadr_in: wp.array2d[int],
  nidof_in: wp.array[int],
  island_nv_inout: wp.array2d[int],
  map_dof2idof_out: wp.array2d[int],
  map_idof2dof_out: wp.array2d[int],
  idof_islandid_out: wp.array2d[int],
  unconstrained_cnt_inout: wp.array2d[int],
):
  worldid, dofid = wp.tid()

  nidof = nidof_in[worldid]
  island_id = dof_island_in[worldid, dofid]

  if island_id >= 0:
    local_idx = wp.atomic_add(island_nv_inout, worldid, island_id, 1)
    idof = island_idofadr_in[worldid, island_id] + local_idx
    idof_islandid_out[worldid, idof] = island_id
  else:
    cnt = wp.atomic_add(unconstrained_cnt_inout, worldid, 0, 1)
    idof = nidof + cnt

  map_dof2idof_out[worldid, dofid] = idof
  map_idof2dof_out[worldid, idof] = dofid


@wp.kernel
def _island_count_constraints(
  nefc_in: wp.array[int],
  njmax_in: int,
  efc_tree_in: wp.array2d[int],
  tree_island_in: wp.array2d[int],
  efc_type_in: wp.array2d[int],
  efc_island_out: wp.array2d[int],
  island_nefc_out: wp.array2d[int],
  island_ne_out: wp.array2d[int],
  island_nf_out: wp.array2d[int],
):
  worldid, efcid = wp.tid()
  if efcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  efc_tree = efc_tree_in[worldid, efcid]
  if efc_tree < 0:
    efc_island_out[worldid, efcid] = -1
    return
  island_id = tree_island_in[worldid, efc_tree]
  efc_island_out[worldid, efcid] = island_id

  if island_id >= 0:
    wp.atomic_add(island_nefc_out, worldid, island_id, 1)

    efc_type = efc_type_in[worldid, efcid]
    if efc_type == ConstraintType.EQUALITY:
      wp.atomic_add(island_ne_out, worldid, island_id, 1)
    elif efc_type == ConstraintType.FRICTION_DOF or efc_type == ConstraintType.FRICTION_TENDON:
      wp.atomic_add(island_nf_out, worldid, island_id, 1)


@wp.kernel
def _island_map_constraints(
  nefc_in: wp.array[int],
  njmax_in: int,
  efc_island_in: wp.array2d[int],
  island_iefcadr_in: wp.array2d[int],
  island_ne_in: wp.array2d[int],
  island_nf_in: wp.array2d[int],
  efc_type_in: wp.array2d[int],
  # Counters (inout):
  island_ne_mapped_inout: wp.array2d[int],
  island_nf_mapped_inout: wp.array2d[int],
  island_nother_mapped_inout: wp.array2d[int],
  island_nefc_inout: wp.array2d[int],
  # Out:
  map_efc2iefc_out: wp.array2d[int],
  map_iefc2efc_out: wp.array2d[int],
  iefc_islandid_out: wp.array2d[int],
):
  worldid, efcid = wp.tid()
  if efcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  island_id = efc_island_in[worldid, efcid]
  if island_id >= 0:
    efc_type = efc_type_in[worldid, efcid]

    # 1. Determine local index and absolute index ic based on category
    if efc_type == ConstraintType.EQUALITY:
      local_idx = wp.atomic_add(island_ne_mapped_inout, worldid, island_id, 1)
      ic = island_iefcadr_in[worldid, island_id] + local_idx
    elif efc_type == ConstraintType.FRICTION_DOF or efc_type == ConstraintType.FRICTION_TENDON:
      local_idx = wp.atomic_add(island_nf_mapped_inout, worldid, island_id, 1)
      ic = island_iefcadr_in[worldid, island_id] + island_ne_in[worldid, island_id] + local_idx
    else:
      local_idx = wp.atomic_add(island_nother_mapped_inout, worldid, island_id, 1)
      ic = (
        island_iefcadr_in[worldid, island_id] + island_ne_in[worldid, island_id] + island_nf_in[worldid, island_id] + local_idx
      )

    # 2. Increment overall island_nefc counter to reconstruct d.island_nefc
    wp.atomic_add(island_nefc_inout, worldid, island_id, 1)

    # 3. Store mappings
    map_efc2iefc_out[worldid, efcid] = ic
    map_iefc2efc_out[worldid, ic] = efcid
    iefc_islandid_out[worldid, ic] = island_id


@wp.kernel
def _island_scan_sparse_rows(
  nisland_in: wp.array[int],
  efc_J_rownnz_in: wp.array2d[int],
  island_nefc_in: wp.array2d[int],
  island_iefcadr_in: wp.array2d[int],
  map_iefc2efc_in: wp.array2d[int],
  iefc_J_rownnz_out: wp.array2d[int],
  iefc_J_rowadr_out: wp.array2d[int],
):
  worldid = wp.tid()

  nisland = nisland_in[worldid]
  if nisland == 0:
    return

  total_gathered_efc = island_iefcadr_in[worldid, nisland - 1] + island_nefc_in[worldid, nisland - 1]

  running_rowadr = int(0)
  for ic in range(total_gathered_efc):
    c = map_iefc2efc_in[worldid, ic]
    rownnz = efc_J_rownnz_in[worldid, c]
    iefc_J_rowadr_out[worldid, ic] = running_rowadr
    iefc_J_rownnz_out[worldid, ic] = rownnz
    running_rowadr += rownnz


@wp.kernel
def _compute_efc_tree(
  # Model:
  nv: int,
  body_treeid: wp.array[int],
  jnt_dofadr: wp.array[int],
  dof_treeid: wp.array[int],
  geom_bodyid: wp.array[int],
  site_bodyid: wp.array[int],
  eq_type: wp.array[int],
  eq_obj1id: wp.array[int],
  eq_obj2id: wp.array[int],
  eq_objtype: wp.array[int],
  is_sparse: bool,
  # Data in:
  nefc_in: wp.array[int],
  contact_geom_in: wp.array[wp.vec2i],
  efc_type_in: wp.array2d[int],
  efc_id_in: wp.array2d[int],
  efc_J_in: wp.array3d[float],
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  njmax_in: int,
  # Out:
  efc_tree_out: wp.array2d[int],
):
  """Compute the first non-negative tree for each constraint."""
  worldid, efcid = wp.tid()

  if efcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  efc_type = efc_type_in[worldid, efcid]
  efc_id = efc_id_in[worldid, efcid]

  tree = int(-1)
  use_generic = int(0)

  # equality (connect/weld)
  if efc_type == ConstraintType.EQUALITY:
    eq_t = eq_type[efc_id]

    if eq_t == EqType.CONNECT or eq_t == EqType.WELD:
      b1 = eq_obj1id[efc_id]
      b2 = eq_obj2id[efc_id]

      if eq_objtype[efc_id] == ObjType.SITE:
        b1 = site_bodyid[b1]
        b2 = site_bodyid[b2]

      t1 = body_treeid[b1]
      t2 = body_treeid[b2]
      if t1 >= 0:
        tree = t1
      else:
        tree = t2
    else:
      # JOINT, TENDON, FLEX: generic scan
      use_generic = 1

  # joint friction
  elif efc_type == ConstraintType.FRICTION_DOF:
    tree = dof_treeid[efc_id]

  # joint limit
  elif efc_type == ConstraintType.LIMIT_JOINT:
    tree = dof_treeid[jnt_dofadr[efc_id]]

  # contact
  elif (
    efc_type == ConstraintType.CONTACT_FRICTIONLESS
    or efc_type == ConstraintType.CONTACT_PYRAMIDAL
    or efc_type == ConstraintType.CONTACT_ELLIPTIC
  ):
    geom_pair = contact_geom_in[efc_id]
    g1 = geom_pair[0]
    g2 = geom_pair[1]

    if g1 >= 0 and g2 >= 0:
      t1 = body_treeid[geom_bodyid[g1]]
      t2 = body_treeid[geom_bodyid[g2]]
      if t1 >= 0:
        tree = t1
      else:
        tree = t2
    else:
      # flex contacts: generic scan
      use_generic = 1

  else:
    # generic: scan Jacobian row
    use_generic = 1

  if use_generic:
    count = nv
    rowadr = 0
    if is_sparse:
      count = efc_J_rownnz_in[worldid, efcid]
      rowadr = efc_J_rowadr_in[worldid, efcid]

    for i in range(count):
      dof = i
      if is_sparse:
        sparseid = rowadr + i
        dof = efc_J_colind_in[worldid, 0, sparseid]
      else:
        J_val = efc_J_in[worldid, efcid, dof]
        if J_val == 0.0:
          continue

      t = dof_treeid[dof]
      if t >= 0:
        tree = t
        break

  efc_tree_out[worldid, efcid] = tree


@wp.kernel
def _gather_efc_and_jacobian(
  # Model:
  is_sparse: bool,
  # Data in:
  nefc_in: wp.array[int],
  efc_D_in: wp.array2d[float],
  efc_type_in: wp.array2d[int],
  efc_id_in: wp.array2d[int],
  efc_frictionloss_in: wp.array2d[float],
  efc_aref_in: wp.array2d[float],
  efc_J_in: wp.array3d[float],
  # Sparse Jacobian arrays:
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  iefc_J_rowadr_in: wp.array2d[int],
  # In:
  njmax_in: int,
  map_iefc2efc_in: wp.array2d[int],
  map_idof2dof_in: wp.array2d[int],
  map_dof2idof_in: wp.array2d[int],
  nidof_in: wp.array[int],
  # Out:
  iefc_D_out: wp.array2d[float],
  iefc_type_out: wp.array2d[int],
  iefc_id_out: wp.array2d[int],
  iefc_frictionloss_out: wp.array2d[float],
  iefc_aref_out: wp.array2d[float],
  iefc_J_out: wp.array3d[float],
  iefc_J_colind_out: wp.array3d[int],
):
  """Gather constraint arrays and Jacobian into island-local dense order."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  c = map_iefc2efc_in[worldid, iefcid]

  # Scalar constraint fields
  iefc_D_out[worldid, iefcid] = efc_D_in[worldid, c]
  iefc_type_out[worldid, iefcid] = efc_type_in[worldid, c]
  iefc_id_out[worldid, iefcid] = efc_id_in[worldid, c]
  iefc_frictionloss_out[worldid, iefcid] = efc_frictionloss_in[worldid, c]
  iefc_aref_out[worldid, iefcid] = efc_aref_in[worldid, c]

  # Jacobian: convert to island-local dense format
  nid = nidof_in[worldid]

  if is_sparse:
    rownnz = efc_J_rownnz_in[worldid, c]
    rowadr_in = efc_J_rowadr_in[worldid, c]
    rowadr_out = iefc_J_rowadr_in[worldid, iefcid]
    for i in range(rownnz):
      sparseid_in = rowadr_in + i
      sparseid_out = rowadr_out + i
      dof = efc_J_colind_in[worldid, 0, sparseid_in]
      idof = map_dof2idof_in[worldid, dof]
      # Store in sparse iefc_J_out and iefc_J_colind_out
      iefc_J_out[worldid, 0, sparseid_out] = efc_J_in[worldid, 0, sparseid_in]
      iefc_J_colind_out[worldid, 0, sparseid_out] = idof
  else:
    # Dense path: reorder rows by iefc, columns by idof
    for idof in range(nid):
      dof = map_idof2dof_in[worldid, idof]
      iefc_J_out[worldid, iefcid, idof] = efc_J_in[worldid, c, dof]


@wp.kernel
def _gather_dof_arrays(
  # Data in:
  qacc_in: wp.array2d[float],
  qacc_smooth_in: wp.array2d[float],
  qfrc_smooth_in: wp.array2d[float],
  # In:
  nidof_in: wp.array[int],
  map_idof2dof_in: wp.array2d[int],
  # Out:
  iacc_out: wp.array2d[float],
  iacc_smooth_out: wp.array2d[float],
  ifrc_smooth_out: wp.array2d[float],
):
  """Gather DOF arrays into island-local order."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  dof = map_idof2dof_in[worldid, idofid]
  iacc_out[worldid, idofid] = qacc_in[worldid, dof]
  iacc_smooth_out[worldid, idofid] = qacc_smooth_in[worldid, dof]
  ifrc_smooth_out[worldid, idofid] = qfrc_smooth_in[worldid, dof]


@wp.kernel
def _scatter_dof_arrays(
  # In:
  qacc_smooth_in: wp.array2d[float],
  qfrc_smooth_in: wp.array2d[float],
  dof_island_in: wp.array2d[int],
  iacc_in: wp.array2d[float],
  ifrc_constraint_in: wp.array2d[float],
  iMa_in: wp.array2d[float],
  map_dof2idof_in: wp.array2d[int],
  scatter_Ma: bool,
  # Out:
  qacc_out: wp.array2d[float],
  qfrc_constraint_out: wp.array2d[float],
  Ma_out: wp.array2d[float],
):
  """Scatter island results to global arrays, copy qacc_smooth for non-island DOFs."""
  worldid, dofid = wp.tid()

  if dof_island_in[worldid, dofid] < 0:
    qacc_out[worldid, dofid] = qacc_smooth_in[worldid, dofid]
    qfrc_constraint_out[worldid, dofid] = 0.0
    if scatter_Ma:
      Ma_out[worldid, dofid] = qfrc_smooth_in[worldid, dofid]
  else:
    idof = map_dof2idof_in[worldid, dofid]
    qacc_out[worldid, dofid] = iacc_in[worldid, idof]
    qfrc_constraint_out[worldid, dofid] = ifrc_constraint_in[worldid, idof]
    if scatter_Ma:
      Ma_out[worldid, dofid] = iMa_in[worldid, idof]


@wp.kernel
def _scatter_efc_arrays(
  # In:
  nefc_in: wp.array[int],
  njmax_in: int,
  map_iefc2efc_in: wp.array2d[int],
  iefc_force_in: wp.array2d[float],
  iefc_state_in: wp.array2d[int],
  # Out:
  efc_force_out: wp.array2d[float],
  efc_state_out: wp.array2d[int],
):
  """Scatter island-local constraint results back to global order."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  c = map_iefc2efc_in[worldid, iefcid]
  efc_force_out[worldid, c] = iefc_force_in[worldid, iefcid]
  efc_state_out[worldid, c] = iefc_state_in[worldid, iefcid]


@wp.kernel
def _init_island_arrays(
  island_idofadr_out: wp.array2d[int],
  island_nv_out: wp.array2d[int],
  island_nefc_out: wp.array2d[int],
  island_ne_out: wp.array2d[int],
  island_nf_out: wp.array2d[int],
  island_iefcadr_out: wp.array2d[int],
  nidof_out: wp.array[int],
):
  worldid, islandid = wp.tid()
  island_nv_out[worldid, islandid] = 0
  island_nefc_out[worldid, islandid] = 0
  island_ne_out[worldid, islandid] = 0
  island_nf_out[worldid, islandid] = 0
  island_idofadr_out[worldid, islandid] = 0
  island_iefcadr_out[worldid, islandid] = 0
  if islandid == 0:
    nidof_out[worldid] = 0


@wp.kernel
def _init_dof_arrays(
  dof_island_out: wp.array2d[int],
  map_dof2idof_out: wp.array2d[int],
  map_idof2dof_out: wp.array2d[int],
  idof_islandid_out: wp.array2d[int],
):
  worldid, dofid = wp.tid()
  dof_island_out[worldid, dofid] = -1
  map_dof2idof_out[worldid, dofid] = 0
  map_idof2dof_out[worldid, dofid] = 0
  idof_islandid_out[worldid, dofid] = -1


@wp.kernel
def _init_efc_arrays(
  efc_island_out: wp.array2d[int],
  map_efc2iefc_out: wp.array2d[int],
  map_iefc2efc_out: wp.array2d[int],
  iefc_islandid_out: wp.array2d[int],
  efc_tree_out: wp.array2d[int],
):
  worldid, efcid = wp.tid()
  efc_island_out[worldid, efcid] = -1
  map_efc2iefc_out[worldid, efcid] = 0
  map_iefc2efc_out[worldid, efcid] = 0
  iefc_islandid_out[worldid, efcid] = -1
  efc_tree_out[worldid, efcid] = -1


@event_scope
def compute_island_mapping(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Compute DOF/constraint island mappings after island discovery.

  Populates island solver context arrays via ctx: nv, nefc, ne, nf,
  iefcadr, nidof, map_dof2idof, map_idof2dof, dof_islandid, map_efc2iefc,
  map_iefc2efc, efc_islandid. Also populates d.dof_island, d.efc.island,
  and d.island_dofadr.

  Args:
    m: Model.
    d: Data.
    ctx: IslandSolverContext.
  """
  # Ensure dof_islandid / efc_islandid are allocated at the right shape
  if d.dof_islandid.shape[1] != m.nv:
    d.dof_islandid = wp.empty((d.nworld, m.nv), dtype=int)
  if d.efc_islandid.shape[1] != d.njmax:
    d.efc_islandid = wp.empty((d.nworld, d.njmax), dtype=int)

  # Ensure island-local DOF arrays are allocated at the right shape
  if d.iqacc.shape[1] != m.nv:
    nw = d.nworld
    d.iqacc = wp.empty((nw, m.nv), dtype=float)
    d.iqacc_smooth = wp.empty((nw, m.nv), dtype=float)
    d.iqfrc_smooth = wp.empty((nw, m.nv), dtype=float)
    d.iqfrc_constraint = wp.empty((nw, m.nv), dtype=float)
  wp.launch(
    _init_island_arrays,
    dim=(d.nworld, m.ntree),
    inputs=[],
    outputs=[
      d.island_dofadr,
      d.island_nv,
      d.island_nefc,
      d.island_ne,
      d.island_nf,
      d.island_efcadr,
      d.nidof,
    ],
  )
  wp.launch(
    _init_dof_arrays,
    dim=(d.nworld, m.nv),
    inputs=[],
    outputs=[d.dof_island, d.map_dof2idof, d.map_idof2dof, d.dof_islandid],
  )
  efc_tree = wp.empty((d.nworld, d.njmax), dtype=int)
  wp.launch(
    _init_efc_arrays,
    dim=(d.nworld, d.njmax),
    inputs=[],
    outputs=[d.efc.island, d.map_efc2iefc, d.map_iefc2efc, d.efc_islandid, efc_tree],
  )

  wp.launch(
    _compute_efc_tree,
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
      m.is_sparse,
      d.nefc,
      d.contact.geom,
      d.efc.type,
      d.efc.id,
      d.efc.J,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.njmax,
    ],
    outputs=[efc_tree],
  )

  # 1. Count DOFs per island
  wp.launch(
    _island_count_dofs,
    dim=(d.nworld, m.nv),
    inputs=[m.dof_treeid, d.tree_island],
    outputs=[d.dof_island, d.island_nv],
  )

  # 2. Count Constraints per island
  wp.launch(
    _island_count_constraints,
    dim=(d.nworld, d.njmax),
    inputs=[d.nefc, d.njmax, efc_tree, d.tree_island, d.efc.type],
    outputs=[d.efc.island, d.island_nefc, d.island_ne, d.island_nf],
  )

  # 3. Scan sizes and reset counters for mapping
  wp.launch(
    _island_scan_sizes,
    dim=d.nworld,
    inputs=[d.nisland],
    outputs=[d.island_dofadr, d.island_nv, d.island_nefc, d.island_efcadr, d.nidof],
  )

  # 4. Map DOFs
  unconstrained_cnt = wp.zeros((d.nworld, 1), dtype=int)
  wp.launch(
    _island_map_dofs,
    dim=(d.nworld, m.nv),
    inputs=[m.nv, d.dof_island, d.island_dofadr, d.nidof],
    outputs=[d.island_nv, d.map_dof2idof, d.map_idof2dof, d.dof_islandid, unconstrained_cnt],
  )

  # 5. Map Constraints
  ne_mapped = wp.zeros((d.nworld, m.ntree), dtype=int)
  nf_mapped = wp.zeros((d.nworld, m.ntree), dtype=int)
  nother_mapped = wp.zeros((d.nworld, m.ntree), dtype=int)

  wp.launch(
    _island_map_constraints,
    dim=(d.nworld, d.njmax),
    inputs=[
      d.nefc,
      d.njmax,
      d.efc.island,
      d.island_efcadr,
      d.island_ne,
      d.island_nf,
      d.efc.type,
      ne_mapped,
      nf_mapped,
      nother_mapped,
    ],
    outputs=[d.island_nefc, d.map_efc2iefc, d.map_iefc2efc, d.efc_islandid],
  )

  # 6. Scan Sparse Rows (if sparse)
  if m.is_sparse:
    wp.launch(
      _island_scan_sparse_rows,
      dim=d.nworld,
      inputs=[d.nisland, d.efc.J_rownnz, d.island_nefc, d.island_efcadr, d.map_iefc2efc],
      outputs=[d.efc.iJ_rownnz, d.efc.iJ_rowadr],
    )


@event_scope
def gather_island_inputs(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Gather constraint and DOF arrays into island-local order.

  Populates d.iefc (D, type, id, frictionloss, aref, J, J_colind) and
  d.iqacc, d.iqacc_smooth, d.iqfrc_smooth.

  Must be called after compute_island_mapping() and before per-island solving.

  Args:
    m: Model.
    d: Data.
    ctx: IslandSolverContext whose arrays are populated.
  """
  # Gather constraint arrays and dense Jacobian (fused)
  wp.launch(
    _gather_efc_and_jacobian,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.is_sparse,
      d.nefc,
      d.efc.D,
      d.efc.type,
      d.efc.id,
      d.efc.frictionloss,
      d.efc.aref,
      d.efc.J,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.iJ_rowadr,
      d.njmax,
      d.map_iefc2efc,
      d.map_idof2dof,
      d.map_dof2idof,
      d.nidof,
    ],
    outputs=[
      d.efc.iD,
      d.efc.itype,
      d.efc.iid,
      d.efc.ifrictionloss,
      d.efc.iaref,
      d.efc.iJ,
      d.efc.iJ_colind,
    ],
  )

  # Gather DOF arrays
  wp.launch(
    _gather_dof_arrays,
    dim=(d.nworld, m.nv),
    inputs=[
      d.qacc,
      d.qacc_smooth,
      d.qfrc_smooth,
      d.nidof,
      d.map_idof2dof,
    ],
    outputs=[
      d.iqacc,
      d.iqacc_smooth,
      d.iqfrc_smooth,
    ],
  )


@event_scope
def scatter_island_results(m: types.Model, d: types.Data, ctx: IslandSolverContext, scatter_Ma: bool):
  """Scatter island-local solver results back to global arrays.

  Reads ctx qacc, qfrc_constraint, Ma, d.efc iforce, istate and
  writes them back to d.qacc, d.qfrc_constraint, d.efc.Ma, d.efc.force,
  d.efc.state. Unconstrained DOFs receive qacc_smooth and zero qfrc.

  Args:
    m: Model.
    d: Data.
    ctx: IslandSolverContext which contains results.
    scatter_Ma: Whether to scatter Ma for Euler/implicit integrators.
  """
  # Scatter DOF results (and optionally Ma for Euler/implicit integrators)
  wp.launch(
    _scatter_dof_arrays,
    dim=(d.nworld, m.nv),
    inputs=[
      d.qacc_smooth,
      d.qfrc_smooth,
      d.dof_island,
      d.iqacc,
      d.iqfrc_constraint,
      ctx.Ma,
      d.map_dof2idof,
      scatter_Ma,
    ],
    outputs=[
      d.qacc,
      d.qfrc_constraint,
      d.efc.Ma,
    ],
  )

  # Scatter constraint results (force and state from island-local arrays)
  wp.launch(
    _scatter_efc_arrays,
    dim=(d.nworld, d.njmax),
    inputs=[
      d.nefc,
      d.njmax,
      d.map_iefc2efc,
      d.efc.iforce,
      d.efc.istate,
    ],
    outputs=[
      d.efc.force,
      d.efc.state,
    ],
  )
