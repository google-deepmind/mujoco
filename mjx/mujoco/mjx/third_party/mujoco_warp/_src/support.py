# Copyright 2025 The Newton Developers
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

from typing import Optional, Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.math import motion_cross
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import State
from mujoco.mjx.third_party.mujoco_warp._src.types import TileSet
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import nested_kernel

wp.set_module_options({"enable_backward": False})


@cache_kernel
def mul_m_sparse_diag(check_skip: bool):
  @nested_kernel(module="unique", enable_backward=False)
  def _mul_m_sparse_diag(
    # Model:
    dof_Madr: wp.array(dtype=int),
    # Data in:
    qM_in: wp.array3d(dtype=float),
    # In:
    vec: wp.array2d(dtype=float),
    skip: wp.array(dtype=bool),
    # Out:
    res: wp.array2d(dtype=float),
  ):
    """Diagonal update for sparse matmul."""
    worldid, dofid = wp.tid()

    if wp.static(check_skip):
      if skip[worldid]:
        return

    res[worldid, dofid] = qM_in[worldid, 0, dof_Madr[dofid]] * vec[worldid, dofid]

  return _mul_m_sparse_diag


@cache_kernel
def mul_m_sparse_ij(check_skip: bool):
  @nested_kernel(module="unique", enable_backward=False)
  def _mul_m_sparse_ij(
    # Model:
    qM_mulm_i: wp.array(dtype=int),
    qM_mulm_j: wp.array(dtype=int),
    qM_madr_ij: wp.array(dtype=int),
    # Data in:
    qM_in: wp.array3d(dtype=float),
    # In:
    vec: wp.array2d(dtype=float),
    skip: wp.array(dtype=bool),
    # Out:
    res: wp.array2d(dtype=float),
  ):
    """Off-diagonal update for sparse matmul."""
    worldid, elementid = wp.tid()

    if wp.static(check_skip):
      if skip[worldid]:
        return

    i = qM_mulm_i[elementid]
    j = qM_mulm_j[elementid]
    madr_ij = qM_madr_ij[elementid]

    qM_ij = qM_in[worldid, 0, madr_ij]

    wp.atomic_add(res[worldid], i, qM_ij * vec[worldid, j])
    wp.atomic_add(res[worldid], j, qM_ij * vec[worldid, i])

  return _mul_m_sparse_ij


@cache_kernel
def mul_m_dense(tile: TileSet, check_skip: bool):
  """Returns a matmul kernel for some tile size."""

  @nested_kernel(module="unique", enable_backward=False)
  def _mul_m_dense(
    # Data In:
    qM_in: wp.array3d(dtype=float),
    # In:
    adr: wp.array(dtype=int),
    vec: wp.array3d(dtype=float),
    skip: wp.array(dtype=bool),
    # Out:
    res: wp.array3d(dtype=float),
  ):
    worldid, nodeid = wp.tid()
    TILE_SIZE = wp.static(tile.size)

    if wp.static(check_skip):
      if skip[worldid]:
        return

    dofid = adr[nodeid]
    qM_tile = wp.tile_load(qM_in[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(dofid, dofid), bounds_check=False)
    vec_tile = wp.tile_load(vec[worldid], shape=(TILE_SIZE, 1), offset=(dofid, 0), bounds_check=False)
    res_tile = wp.tile_matmul(qM_tile, vec_tile)
    wp.tile_store(res[worldid], res_tile, offset=(dofid, 0), bounds_check=False)

  return _mul_m_dense


@event_scope
def mul_m(
  m: Model,
  d: Data,
  res: wp.array2d(dtype=float),
  vec: wp.array2d(dtype=float),
  skip: Optional[wp.array] = None,
  M: Optional[wp.array] = None,
):
  """Multiply vectors by inertia matrix; optionally skip per world.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    res: Result: qM @ vec.
    vec: Input vector to multiply by qM.
    skip: Per-world bitmask to skip computing output.
    M: Input matrix: M @ vec.
  """
  check_skip = skip is not None
  skip = skip or wp.empty(0, dtype=bool)

  if M is None:
    M = d.qM

  if m.opt.is_sparse:
    wp.launch(
      mul_m_sparse_diag(check_skip),
      dim=(d.nworld, m.nv),
      inputs=[m.dof_Madr, M, vec, skip],
      outputs=[res],
    )

    wp.launch(
      mul_m_sparse_ij(check_skip),
      dim=(d.nworld, m.qM_madr_ij.size),
      inputs=[m.qM_mulm_i, m.qM_mulm_j, m.qM_madr_ij, M, vec, skip],
      outputs=[res],
    )

  else:
    for tile in m.qM_tiles:
      wp.launch_tiled(
        mul_m_dense(tile, check_skip),
        dim=(d.nworld, tile.adr.size),
        inputs=[
          M,
          tile.adr,
          # note reshape: tile_matmul expects 2d input
          vec.reshape(vec.shape + (1,)),
          skip,
        ],
        outputs=[res.reshape(res.shape + (1,))],
        block_dim=m.block_dim.mul_m_dense,
      )


@wp.kernel
def _apply_ft(
  # Model:
  nbody: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  ft_in: wp.array2d(dtype=wp.spatial_vector),
  flg_add: bool,
  # Out:
  qfrc_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  cdof = cdof_in[worldid, dofid]
  rotational_cdof = wp.vec3(cdof[0], cdof[1], cdof[2])
  jac = wp.spatial_vector(cdof[3], cdof[4], cdof[5], cdof[0], cdof[1], cdof[2])

  dofbodyid = dof_bodyid[dofid]
  accumul = float(0.0)

  for bodyid in range(dofbodyid, nbody):
    ft_body = ft_in[worldid, bodyid]
    if ft_body == wp.spatial_vector():
      continue
    # any body that is in the subtree of dofbodyid is part of the jacobian
    parentid = bodyid
    while parentid != 0 and parentid != dofbodyid:
      parentid = body_parentid[parentid]
    if parentid == 0:
      continue  # body is not part of the subtree
    offset = xipos_in[worldid, bodyid] - subtree_com_in[worldid, body_rootid[bodyid]]
    cross_term = wp.cross(rotational_cdof, offset)
    accumul += wp.dot(jac, ft_body) + wp.dot(cross_term, wp.spatial_top(ft_body))

  if flg_add:
    qfrc_out[worldid, dofid] += accumul
  else:
    qfrc_out[worldid, dofid] = accumul


def apply_ft(m: Model, d: Data, ft: wp.array2d(dtype=wp.spatial_vector), qfrc: wp.array2d(dtype=float), flg_add: bool):
  wp.launch(
    kernel=_apply_ft,
    dim=(d.nworld, m.nv),
    inputs=[m.nbody, m.body_parentid, m.body_rootid, m.dof_bodyid, d.xipos, d.subtree_com, d.cdof, ft, flg_add],
    outputs=[qfrc],
  )


@event_scope
def xfrc_accumulate(m: Model, d: Data, qfrc: wp.array2d(dtype=float)):
  """Map applied forces at each body via Jacobians to dof space and accumulate.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    qfrc: Total applied force mapped to dof space.
  """
  apply_ft(m, d, d.xfrc_applied, qfrc, True)


@wp.func
def all_same(v0: wp.vec3, v1: wp.vec3) -> wp.bool:
  dx = abs(v0[0] - v1[0])
  dy = abs(v0[1] - v1[1])
  dz = abs(v0[2] - v1[2])

  return (
    (dx <= 1.0e-9 or dx <= max(abs(v0[0]), abs(v1[0])) * 1.0e-9)
    and (dy <= 1.0e-9 or dy <= max(abs(v0[1]), abs(v1[1])) * 1.0e-9)
    and (dz <= 1.0e-9 or dz <= max(abs(v0[2]), abs(v1[2])) * 1.0e-9)
  )


@wp.func
def any_different(v0: wp.vec3, v1: wp.vec3) -> wp.bool:
  dx = abs(v0[0] - v1[0])
  dy = abs(v0[1] - v1[1])
  dz = abs(v0[2] - v1[2])

  return (
    (dx > 1.0e-9 and dx > max(abs(v0[0]), abs(v1[0])) * 1.0e-9)
    or (dy > 1.0e-9 and dy > max(abs(v0[1]), abs(v1[1])) * 1.0e-9)
    or (dz > 1.0e-9 and dz > max(abs(v0[2]), abs(v1[2])) * 1.0e-9)
  )


@wp.func
def _decode_pyramid(
  njmax_in: int, pyramid: wp.array(dtype=float), efc_address: int, mu: vec5, condim: int
) -> wp.spatial_vector:
  """Converts pyramid representation to contact force."""
  force = wp.spatial_vector()

  if condim == 1:
    force[0] = pyramid[efc_address]
    return force

  force[0] = float(0.0)
  for i in range(condim - 1):
    adr = 2 * i + efc_address
    if adr < njmax_in:
      dir1 = pyramid[adr]
    else:
      dir1 = 0.0
    if adr + 1 < njmax_in:
      dir2 = pyramid[adr + 1]
    else:
      dir2 = 0.0
    force[0] += dir1 + dir2
    force[i + 1] = (dir1 - dir2) * mu[i]

  return force


@wp.func
def contact_force_fn(
  # Model:
  opt_cone: int,
  # Data in:
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  # In:
  worldid: int,
  contact_id: int,
  to_world_frame: bool,
) -> wp.spatial_vector:
  """Extract 6D force:torque for one contact, in contact frame by default."""
  force = wp.spatial_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  condim = contact_dim_in[contact_id]
  efc_address = contact_efc_address_in[contact_id, 0]

  if contact_id >= 0 and contact_id <= nacon_in[0] and efc_address >= 0:
    if opt_cone == ConeType.PYRAMIDAL:
      force = _decode_pyramid(
        njmax_in,
        efc_force_in[worldid],
        efc_address,
        contact_friction_in[contact_id],
        condim,
      )
    else:
      for i in range(condim):
        if contact_efc_address_in[contact_id, i] < njmax_in:
          force[i] = efc_force_in[worldid, contact_efc_address_in[contact_id, i]]

  if to_world_frame:
    # Transform both top and bottom parts of spatial vector by the full contact frame matrix
    t = wp.spatial_top(force) @ contact_frame_in[contact_id]
    b = wp.spatial_bottom(force) @ contact_frame_in[contact_id]
    force = wp.spatial_vector(t, b)

  return force


@wp.kernel
def contact_force_kernel(
  # Model:
  opt_cone: int,
  # Data in:
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  # In:
  contact_ids: wp.array(dtype=int),
  to_world_frame: bool,
  # Out:
  out: wp.array(dtype=wp.spatial_vector),
):
  tid = wp.tid()

  contactid = contact_ids[tid]

  if contactid >= nacon_in[0]:
    return

  worldid = contact_worldid_in[contactid]

  out[tid] = contact_force_fn(
    opt_cone,
    contact_frame_in,
    contact_friction_in,
    contact_dim_in,
    contact_efc_address_in,
    efc_force_in,
    njmax_in,
    nacon_in,
    worldid,
    contactid,
    to_world_frame,
  )


def contact_force(
  m: Model, d: Data, contact_ids: wp.array(dtype=int), to_world_frame: bool, force: wp.array(dtype=wp.spatial_vector)
):
  """Compute forces for contacts in Data.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    contact_ids: IDs for each contact.
    to_world_frame: If True, map force from contact to world frame.
    force: Contact forces.
  """
  wp.launch(
    contact_force_kernel,
    dim=contact_ids.size,
    inputs=[
      m.opt.cone,
      d.contact.frame,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.contact.worldid,
      d.efc.force,
      d.njmax,
      d.nacon,
      contact_ids,
      to_world_frame,
    ],
    outputs=[force],
  )


@wp.func
def transform_force(force: wp.vec3, torque: wp.vec3, offset: wp.vec3) -> wp.spatial_vector:
  return wp.spatial_vector(torque - wp.cross(offset, force), force)


@wp.func
def transform_force(frc: wp.spatial_vector, offset: wp.vec3) -> wp.spatial_vector:
  force = wp.spatial_top(frc)
  torque = wp.spatial_bottom(frc)
  return transform_force(force, torque, offset)


@wp.func
def jac(
  # Model:
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  # Data in:
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  point: wp.vec3,
  bodyid: int,
  dofid: int,
  worldid: int,
) -> Tuple[wp.vec3, wp.vec3]:
  dof_bodyid_ = dof_bodyid[dofid]
  in_tree = int(dof_bodyid_ == 0)
  parentid = bodyid
  while parentid != 0:
    if parentid == dof_bodyid_:
      in_tree = 1
      break
    parentid = body_parentid[parentid]

  if not in_tree:
    return wp.vec3(0.0), wp.vec3(0.0)

  offset = point - wp.vec3(subtree_com_in[worldid, body_rootid[bodyid]])

  cdof = cdof_in[worldid, dofid]
  cdof_ang = wp.spatial_top(cdof)
  cdof_lin = wp.spatial_bottom(cdof)

  jacp = cdof_lin + wp.cross(cdof_ang, offset)
  jacr = cdof_ang

  return jacp, jacr


@wp.func
def jac_dot(
  # Model:
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  jnt_type: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  dof_jntid: wp.array(dtype=int),
  # Data in:
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  cdof_dot_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  point: wp.vec3,
  bodyid: int,
  dofid: int,
  worldid: int,
) -> Tuple[wp.vec3, wp.vec3]:
  dof_bodyid_ = dof_bodyid[dofid]
  in_tree = int(dof_bodyid_ == 0)
  parentid = bodyid
  while parentid != 0:
    if parentid == dof_bodyid_:
      in_tree = 1
      break
    parentid = body_parentid[parentid]

  if not in_tree:
    return wp.vec3(0.0), wp.vec3(0.0)

  com = subtree_com_in[worldid, body_rootid[bodyid]]
  offset = point - com

  # transform spatial
  cvel = cvel_in[worldid, bodyid]
  pvel_lin = wp.spatial_bottom(cvel) - wp.cross(offset, wp.spatial_top(cvel))

  cdof = cdof_in[worldid, dofid]
  cdof_dot = cdof_dot_in[worldid, dofid]

  # check for quaternion
  dofjntid = dof_jntid[dofid]
  jnttype = jnt_type[dofjntid]
  jntdofadr = jnt_dofadr[dofjntid]

  if (jnttype == JointType.BALL) or ((jnttype == JointType.FREE) and dofid >= jntdofadr + 3):
    # compute cdof_dot for quaternion (use current body cvel)
    cvel = cvel_in[worldid, dof_bodyid[dofid]]
    cdof_dot = motion_cross(cvel, cdof)

  cdof_dot_ang = wp.spatial_top(cdof_dot)
  cdof_dot_lin = wp.spatial_bottom(cdof_dot)

  # construct translational Jacobian (correct for rotation)
  # first correction term, account for varying cdof
  correction1 = wp.cross(cdof_dot_ang, offset)

  # second correction term, account for point translational velocity
  correction2 = wp.cross(wp.spatial_top(cdof), pvel_lin)

  jacp = cdof_dot_lin + correction1 + correction2
  jacr = cdof_dot_ang

  return jacp, jacr


def get_state(m: Model, d: Data, state: wp.array2d(dtype=float), sig: int, active: Optional[wp.array] = None):
  """Copy concatenated state components specified by sig from Data into state.

  The bits of the integer sig correspond to element fields of State.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output information (device).
    state: Concatenation of state components.
    sig: Bitflag specifying state components.
    active: Per-world bitmask for getting state.
  """
  if sig >= (1 << State.NSTATE):
    raise ValueError(f"invalid state signature {sig} >= 2^mjNSTATE")

  @nested_kernel(module="unique", enable_backward=False)
  def _get_state(
    # Model:
    nq: int,
    nv: int,
    nu: int,
    na: int,
    nbody: int,
    neq: int,
    nmocap: int,
    # Data in:
    time_in: wp.array(dtype=float),
    qpos_in: wp.array2d(dtype=float),
    qvel_in: wp.array2d(dtype=float),
    act_in: wp.array2d(dtype=float),
    qacc_warmstart_in: wp.array2d(dtype=float),
    ctrl_in: wp.array2d(dtype=float),
    qfrc_applied_in: wp.array2d(dtype=float),
    xfrc_applied_in: wp.array2d(dtype=wp.spatial_vector),
    eq_active_in: wp.array2d(dtype=bool),
    mocap_pos_in: wp.array2d(dtype=wp.vec3),
    mocap_quat_in: wp.array2d(dtype=wp.quat),
    # In:
    sig_in: int,
    active_in: wp.array(dtype=bool),
    # Out:
    state_out: wp.array2d(dtype=float),
  ):
    worldid = wp.tid()

    if wp.static(active is not None):
      if not active_in[worldid]:
        return

    adr = int(0)
    for i in range(State.NSTATE.value):
      element = 1 << i
      if element & sig_in:
        if element == State.TIME:
          state_out[worldid, adr] = time_in[worldid]
          adr += 1
        elif element == State.QPOS:
          for j in range(nq):
            state_out[worldid, adr + j] = qpos_in[worldid, j]
          adr += nq
        elif element == State.QVEL:
          for j in range(nv):
            state_out[worldid, adr + j] = qvel_in[worldid, j]
          adr += nv
        elif element == State.ACT:
          for j in range(na):
            state_out[worldid, adr + j] = act_in[worldid, j]
          adr += na
        elif element == State.WARMSTART:
          for j in range(nv):
            state_out[worldid, adr + j] = qacc_warmstart_in[worldid, j]
          adr += nv
        elif element == State.CTRL:
          for j in range(nu):
            state_out[worldid, adr + j] = ctrl_in[worldid, j]
          adr += nu
        elif element == State.QFRC_APPLIED:
          for j in range(nv):
            state_out[worldid, adr + j] = qfrc_applied_in[worldid, j]
          adr += nv
        elif element == State.XFRC_APPLIED:
          for j in range(nbody):
            xfrc = xfrc_applied_in[worldid, j]
            state_out[worldid, adr + 0] = xfrc[0]
            state_out[worldid, adr + 1] = xfrc[1]
            state_out[worldid, adr + 2] = xfrc[2]
            state_out[worldid, adr + 3] = xfrc[3]
            state_out[worldid, adr + 4] = xfrc[4]
            state_out[worldid, adr + 5] = xfrc[5]
            adr += 6
        elif element == State.EQ_ACTIVE:
          for j in range(neq):
            state_out[worldid, adr + j] = float(eq_active_in[worldid, j])
          adr += j
        elif element == State.MOCAP_POS:
          for j in range(nmocap):
            pos = mocap_pos_in[worldid, j]
            state_out[worldid, adr + 0] = pos[0]
            state_out[worldid, adr + 1] = pos[1]
            state_out[worldid, adr + 2] = pos[2]
            adr += 3
        elif element == State.MOCAP_QUAT:
          for j in range(nmocap):
            quat = mocap_quat_in[worldid, j]
            state_out[worldid, adr + 0] = quat[0]
            state_out[worldid, adr + 1] = quat[1]
            state_out[worldid, adr + 2] = quat[2]
            state_out[worldid, adr + 3] = quat[3]
            adr += 4

  wp.launch(
    _get_state,
    dim=d.nworld,
    inputs=[
      m.nq,
      m.nv,
      m.nu,
      m.na,
      m.nbody,
      m.neq,
      m.nmocap,
      d.time,
      d.qpos,
      d.qvel,
      d.act,
      d.qacc_warmstart,
      d.ctrl,
      d.qfrc_applied,
      d.xfrc_applied,
      d.eq_active,
      d.mocap_pos,
      d.mocap_quat,
      int(sig),
      active or wp.ones(d.nworld, dtype=bool),
    ],
    outputs=[state],
  )


def set_state(m: Model, d: Data, state: wp.array2d(dtype=float), sig: int, active: Optional[wp.array] = None):
  """Copy concatenated state components specified by sig from state into Data.

  The bits of the integer sig correspond to element fields of State.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output information (device).
    state: Concatenation of state components.
    sig: Bitflag specifying state components.
    active: Per-world bitmask for setting state.
  """
  if sig >= (1 << State.NSTATE):
    raise ValueError(f"invalid state signature {sig} >= 2^mjNSTATE")

  @nested_kernel(module="unique", enable_backward=False)
  def _set_state(
    # Model:
    nq: int,
    nv: int,
    nu: int,
    na: int,
    nbody: int,
    neq: int,
    nmocap: int,
    # In:
    sig_in: int,
    active_in: wp.array(dtype=bool),
    state_in: wp.array2d(dtype=float),
    # Data out:
    time_out: wp.array(dtype=float),
    qpos_out: wp.array2d(dtype=float),
    qvel_out: wp.array2d(dtype=float),
    act_out: wp.array2d(dtype=float),
    qacc_warmstart_out: wp.array2d(dtype=float),
    ctrl_out: wp.array2d(dtype=float),
    qfrc_applied_out: wp.array2d(dtype=float),
    xfrc_applied_out: wp.array2d(dtype=wp.spatial_vector),
    eq_active_out: wp.array2d(dtype=bool),
    mocap_pos_out: wp.array2d(dtype=wp.vec3),
    mocap_quat_out: wp.array2d(dtype=wp.quat),
  ):
    worldid = wp.tid()

    if wp.static(active is not None):
      if not active_in[worldid]:
        return

    adr = int(0)
    for i in range(State.NSTATE.value):
      element = 1 << i
      if element & sig_in:
        if element == State.TIME:
          time_out[worldid] = state_in[worldid, adr]
          adr += 1
        elif element == State.QPOS:
          for j in range(nq):
            qpos_out[worldid, j] = state_in[worldid, adr + j]
          adr += nq
        elif element == State.QVEL:
          for j in range(nv):
            qvel_out[worldid, j] = state_in[worldid, adr + j]
          adr += nv
        elif element == State.ACT:
          for j in range(na):
            act_out[worldid, j] = state_in[worldid, adr + j]
          adr += na
        elif element == State.WARMSTART:
          for j in range(nv):
            qacc_warmstart_out[worldid, j] = state_in[worldid, adr + j]
          adr += nv
        elif element == State.CTRL:
          for j in range(nu):
            ctrl_out[worldid, j] = state_in[worldid, adr + j]
          adr += nu
        elif element == State.QFRC_APPLIED:
          for j in range(nv):
            qfrc_applied_out[worldid, j] = state_in[worldid, adr + j]
          adr += nv
        elif element == State.XFRC_APPLIED:
          for j in range(nbody):
            xfrc = wp.spatial_vector(
              state_in[worldid, adr + 0],
              state_in[worldid, adr + 1],
              state_in[worldid, adr + 2],
              state_in[worldid, adr + 3],
              state_in[worldid, adr + 4],
              state_in[worldid, adr + 5],
            )
            xfrc_applied_out[worldid, j] = xfrc
            adr += 6
        elif element == State.EQ_ACTIVE:
          for j in range(neq):
            eq_active_out[worldid, j] = bool(state_in[worldid, adr + j])
          adr += j
        elif element == State.MOCAP_POS:
          for j in range(nmocap):
            pos = wp.vec3(
              state_in[worldid, adr + 1],
              state_in[worldid, adr + 0],
              state_in[worldid, adr + 2],
            )
            mocap_pos_out[worldid, j] = pos
            adr += 3
        elif element == State.MOCAP_QUAT:
          for j in range(nmocap):
            quat = wp.quat(
              state_in[worldid, adr + 0],
              state_in[worldid, adr + 1],
              state_in[worldid, adr + 2],
              state_in[worldid, adr + 3],
            )
            mocap_quat_out[worldid, j] = quat
            adr += 4

  wp.launch(
    _set_state,
    dim=d.nworld,
    inputs=[
      m.nq,
      m.nv,
      m.nu,
      m.na,
      m.nbody,
      m.neq,
      m.nmocap,
      int(sig),
      active or wp.ones(d.nworld, dtype=bool),
      state,
    ],
    outputs=[
      d.time,
      d.qpos,
      d.qvel,
      d.act,
      d.qacc_warmstart,
      d.ctrl,
      d.qfrc_applied,
      d.xfrc_applied,
      d.eq_active,
      d.mocap_pos,
      d.mocap_quat,
    ],
  )
