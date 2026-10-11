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
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DynType
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import State
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False, "default_grid_stride": False})


# TODO(team): kernel analyzer array slice?
@wp.func
def next_act(
  # Model:
  opt_timestep: float,  # kernel_analyzer: ignore
  actuator_dyntype: int,  # kernel_analyzer: ignore
  actuator_dynprm: vec10,  # kernel_analyzer: ignore
  actuator_actrange: wp.vec2,  # kernel_analyzer: ignore
  # Data In:
  act_in: float,  # kernel_analyzer: ignore
  act_dot_in: float,  # kernel_analyzer: ignore
  # In:
  act_dot_scale: float,
  clamp: bool,
) -> float:
  # advance actuation
  if actuator_dyntype == DynType.FILTEREXACT:
    tau = wp.max(MJ_MINVAL, actuator_dynprm[0])
    act = act_in + act_dot_scale * act_dot_in * tau * (1.0 - wp.exp(-opt_timestep / tau))
  elif actuator_dyntype == DynType.USER:
    return act_in
  else:
    act = act_in + act_dot_scale * act_dot_in * opt_timestep

  # clamp to actrange
  if clamp:
    act = wp.clamp(act, actuator_actrange[0], actuator_actrange[1])

  return act


@wp.func
def mat33_to_quat_polar(F: wp.mat33) -> wp.quat:
  cell_quat = wp.quat(0.0, 0.0, 0.0, 1.0)
  for _iter in range(50):
    rot = wp.quat_to_matrix(cell_quat)
    rot_t = wp.transpose(rot)
    col1_rot = rot_t[0]
    col2_rot = rot_t[1]
    col3_rot = rot_t[2]
    F_t = wp.transpose(F)
    col1_mat = F_t[0]
    col2_mat = F_t[1]
    col3_mat = F_t[2]

    omega = wp.cross(col1_rot, col1_mat) + wp.cross(col2_rot, col2_mat) + wp.cross(col3_rot, col3_mat)
    denom = wp.abs(wp.dot(col1_rot, col1_mat) + wp.dot(col2_rot, col2_mat) + wp.dot(col3_rot, col3_mat)) + 1.0e-10
    omega = omega / denom

    w = wp.length(omega)
    if w < 1.0e-6:
      break

    axis = omega / w
    half_w = 0.5 * w
    qrot = wp.quat(
      axis[0] * wp.sin(half_w),
      axis[1] * wp.sin(half_w),
      axis[2] * wp.sin(half_w),
      wp.cos(half_w),
    )
    cell_quat = wp.normalize(qrot * cell_quat)
  return cell_quat


@wp.func
def compute_interp_cell_quat(
  # Data in:
  flexnode_xpos_in: wp.array2d[wp.vec3],
  # In:
  order: int,
  ci: int,
  cj: int,
  ck: int,
  cy: int,
  cz: int,
  ny_g: int,
  nz_g: int,
  nstart: int,
  worldid: int,
) -> wp.quat:
  """Computes corotational cell quaternion from deformation gradient at cell center."""
  npc = (order + 1) * (order + 1) * (order + 1)
  F = wp.mat33(0.0)
  idx = int(0)
  for li in range(order + 1):
    for lj in range(order + 1):
      for lk in range(order + 1):
        if idx < npc:
          gi = ci * order + li
          gj = cj * order + lj
          gk = ck * order + lk
          gidx = gi * ny_g * nz_g + gj * nz_g + gk

          node_pos = flexnode_xpos_in[worldid, nstart + gidx]

          dphi_x = float(-1) if li == 0 else float(1)
          dphi_y = float(-1) if lj == 0 else float(1)
          dphi_z = float(-1) if lk == 0 else float(1)
          phi_x = float(0.5)
          phi_y = float(0.5)
          phi_z = float(0.5)

          grad_x = dphi_x * phi_y * phi_z
          grad_y = phi_x * dphi_y * phi_z
          grad_z = phi_x * phi_y * dphi_z

          for r in range(3):
            F[r, 0] += node_pos[r] * grad_x
            F[r, 1] += node_pos[r] * grad_y
            F[r, 2] += node_pos[r] * grad_z

          idx += 1

  return mat33_to_quat_polar(F)


@cache_kernel
def mul_m_kernel(check_skip: bool):
  @wp.kernel(module="unique", grid_stride=False)
  def _mul_m(
    # Model:
    M_mulm_rowadr: wp.array[int],
    M_mulm_col: wp.array[int],
    M_mulm_madr: wp.array[int],
    # Data in:
    M_in: wp.array2d[float],
    # In:
    vec: wp.array2d[float],
    skip: wp.array[bool],
    # Out:
    res: wp.array2d[float],
  ):
    """Sparse matmul: one thread per DOF, gather-based (no atomics)."""
    worldid, dofid = wp.tid()

    if wp.static(check_skip):
      if skip[worldid]:
        return

    # Gather all contributions (diagonal + off-diagonal).
    acc = float(0.0)
    start = M_mulm_rowadr[dofid]
    end = M_mulm_rowadr[dofid + 1]
    for k in range(start, end):
      col = M_mulm_col[k]
      madr = M_mulm_madr[k]
      acc += M_in[worldid, madr] * vec[worldid, col]

    res[worldid, dofid] = acc

  return _mul_m


@cache_kernel
def mul_m_dense(nv: int, check_skip: bool):
  @wp.kernel(module="unique", grid_stride=False)
  def _mul_m_dense(
    # Data in:
    M_in: wp.array3d[float],  # kernel_analyzer: ignore
    # In:
    vec: wp.array2d[float],
    skip: wp.array[bool],
    # Out:
    res: wp.array2d[float],
  ):
    """Dense matmul for the compact active-DOF inertia block (nworld, nv, nv)."""
    worldid, i = wp.tid()

    if wp.static(check_skip):
      if skip[worldid]:
        return

    acc = float(0.0)
    for j in range(wp.static(nv)):
      acc += M_in[worldid, i, j] * vec[worldid, j]
    res[worldid, i] = acc

  return _mul_m_dense


@event_scope
def mul_m(
  m: Model,
  d: Data,
  res: wp.array2d[float],
  vec: wp.array2d[float],
  skip: Optional[wp.array] = None,
  M: Optional[wp.array] = None,
):
  """Multiply vectors by inertia matrix; optionally skip per world.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    res: Result: M @ vec.
    vec: Input vector to multiply by M.
    skip: Per-world bitmask to skip computing output.
    M: Input matrix: M @ vec.
  """
  check_skip = skip is not None
  skip = skip or wp.empty(0, dtype=bool)

  if M is None:
    M = d.M

  if M.ndim == 3:
    # Dense compact active-DOF block (nworld, nv, nv) used by the compact solver.
    wp.launch(
      mul_m_dense(m.nv, check_skip),
      dim=(d.nworld, m.nv),
      inputs=[M, vec, skip],
      outputs=[res],
    )
  else:
    wp.launch(
      mul_m_kernel(check_skip),
      dim=(d.nworld, m.nv),
      inputs=[m.M_mulm_rowadr, m.M_mulm_col, m.M_mulm_madr, M, vec, skip],
      outputs=[res],
    )


@wp.kernel
def _apply_ft(
  # Model:
  nbody: int,
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  dof_bodyid: wp.array[int],
  # Data in:
  xipos_in: wp.array2d[wp.vec3],
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  # In:
  ft_in: wp.array2d[wp.spatial_vector],
  flg_add: bool,
  # Out:
  qfrc_out: wp.array2d[float],
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


def apply_ft(m: Model, d: Data, ft: wp.array2d[wp.spatial_vector], qfrc: wp.array2d[float], flg_add: bool):
  wp.launch(
    kernel=_apply_ft,
    dim=(d.nworld, m.nv),
    inputs=[m.nbody, m.body_parentid, m.body_rootid, m.dof_bodyid, d.xipos, d.subtree_com, d.cdof, ft, flg_add],
    outputs=[qfrc],
  )


@event_scope
def xfrc_accumulate(m: Model, d: Data, qfrc: wp.array2d[float]):
  """Map applied forces at each body via Jacobians to dof space and accumulate.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    qfrc: Total applied force mapped to dof space.
  """
  apply_ft(m, d, d.xfrc_applied, qfrc, True)


@wp.func
def _decode_pyramid(njmax_in: int, pyramid: wp.array[float], efc_address: int, mu: vec5, condim: int) -> wp.spatial_vector:
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
  contact_frame_in: wp.array[wp.mat33],
  contact_friction_in: wp.array[vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  contact_adhesion_in: wp.array[float],
  efc_force_in: wp.array2d[float],
  njmax_in: int,
  nacon_in: wp.array[int],
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

    # report net interface force: solver cone force minus adhesive pull
    force[0] -= contact_adhesion_in[contact_id]

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
  contact_frame_in: wp.array[wp.mat33],
  contact_friction_in: wp.array[vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  contact_worldid_in: wp.array[int],
  contact_adhesion_in: wp.array[float],
  efc_force_in: wp.array2d[float],
  njmax_in: int,
  nacon_in: wp.array[int],
  # In:
  contact_ids: wp.array[int],
  to_world_frame: bool,
  # Out:
  out: wp.array[wp.spatial_vector],
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
    contact_adhesion_in,
    efc_force_in,
    njmax_in,
    nacon_in,
    worldid,
    contactid,
    to_world_frame,
  )


def contact_force(m: Model, d: Data, contact_ids: wp.array[int], to_world_frame: bool, force: wp.array[wp.spatial_vector]):
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
      d.contact.adhesion,
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
def _compute_jacp(cdof_clip: wp.spatial_vector, offset: wp.vec3, affect: int) -> wp.vec3:
  if affect == 0:
    return wp.vec3(0.0)
  cdof_lin = wp.spatial_bottom(cdof_clip)
  cdof_ang = wp.spatial_top(cdof_clip)
  return cdof_lin + wp.cross(cdof_ang, offset)


@wp.func
def _compute_jacr(cdof_clip: wp.spatial_vector, affect: int) -> wp.vec3:
  if affect == 0:
    return wp.vec3(0.0)
  return wp.spatial_top(cdof_clip)


@wp.func
def jac_dof(
  # Model:
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  dof_bodyid: wp.array[int],
  body_isdofancestor: wp.array2d[int],
  # Data in:
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  # In:
  point: wp.vec3,
  bodyid: int,
  dofid: int,
  worldid: int,
) -> Tuple[wp.vec3, wp.vec3]:
  if body_isdofancestor[bodyid, dofid] == 0:
    return wp.vec3(0.0), wp.vec3(0.0)

  offset = point - wp.vec3(subtree_com_in[worldid, body_rootid[bodyid]])

  cdof = cdof_in[worldid, dofid]
  cdof_ang = wp.spatial_top(cdof)
  cdof_lin = wp.spatial_bottom(cdof)

  jacp = cdof_lin + wp.cross(cdof_ang, offset)
  jacr = cdof_ang

  return jacp, jacr


@cache_kernel
def _make_jac_kernel(has_jacp: bool, has_jacr: bool):
  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def _jac(
    # Model:
    body_parentid: wp.array[int],
    body_rootid: wp.array[int],
    dof_bodyid: wp.array[int],
    body_isdofancestor: wp.array2d[int],
    # Data in:
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    # In:
    point_in: wp.array[wp.vec3],
    bodyid_in: wp.array[int],
    # Out:
    jacp_out: wp.array3d[float],
    jacr_out: wp.array3d[float],
  ):
    worldid, dofid = wp.tid()

    jacp_val, jacr_val = jac_dof(
      body_parentid,
      body_rootid,
      dof_bodyid,
      body_isdofancestor,
      subtree_com_in,
      cdof_in,
      point_in[worldid],
      bodyid_in[worldid],
      dofid,
      worldid,
    )

    if wp.static(has_jacp):
      jacp_out[worldid, 0, dofid] = jacp_val[0]
      jacp_out[worldid, 1, dofid] = jacp_val[1]
      jacp_out[worldid, 2, dofid] = jacp_val[2]

    if wp.static(has_jacr):
      jacr_out[worldid, 0, dofid] = jacr_val[0]
      jacr_out[worldid, 1, dofid] = jacr_val[1]
      jacr_out[worldid, 2, dofid] = jacr_val[2]

  return _jac


@event_scope
def jac(
  m: Model,
  d: Data,
  jacp: wp.array | None,  # wp.array3d[float]
  jacr: wp.array | None,  # wp.array3d[float]
  point: wp.array[wp.vec3],
  body: wp.array[int],
):
  """Compute translational and rotational Jacobian for point on body.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state (device).
    jacp: Output translational Jacobian (optional).
    jacr: Output rotational Jacobian (optional).
    point: 3D point in global coordinates.
    body: Body ID for each world.
  """
  kernel = _make_jac_kernel(jacp is not None, jacr is not None)

  jacp_arr = jacp or wp.empty((0, 0, 0), dtype=float)
  jacr_arr = jacr or wp.empty((0, 0, 0), dtype=float)

  wp.launch(
    kernel,
    dim=(d.nworld, m.nv),
    inputs=[m.body_parentid, m.body_rootid, m.dof_bodyid, m.body_isdofancestor, d.subtree_com, d.cdof, point, body],
    outputs=[jacp_arr, jacr_arr],
  )


@wp.func
def jac_dot_dof(
  # Model:
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  jnt_type: wp.array[int],
  jnt_dofadr: wp.array[int],
  dof_bodyid: wp.array[int],
  dof_jntid: wp.array[int],
  body_isdofancestor: wp.array2d[int],
  # Data in:
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  cvel_in: wp.array2d[wp.spatial_vector],
  cdof_dot_in: wp.array2d[wp.spatial_vector],
  # In:
  point: wp.vec3,
  bodyid: int,
  dofid: int,
  worldid: int,
) -> Tuple[wp.vec3, wp.vec3]:
  if body_isdofancestor[bodyid, dofid] == 0:
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


def get_state(m: Model, d: Data, state: wp.array2d[float], sig: int, active: Optional[wp.array] = None):
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

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def _get_state(
    # Model:
    nq: int,
    nv: int,
    nu: int,
    na: int,
    nbody: int,
    neq: int,
    nmocap: int,
    nuserdata: int,
    nhistory: int,
    # Data in:
    time_in: wp.array[float],
    qpos_in: wp.array2d[float],
    qvel_in: wp.array2d[float],
    act_in: wp.array2d[float],
    history_in: wp.array2d[float],
    qacc_warmstart_in: wp.array2d[float],
    ctrl_in: wp.array2d[float],
    qfrc_applied_in: wp.array2d[float],
    xfrc_applied_in: wp.array2d[wp.spatial_vector],
    eq_active_in: wp.array2d[bool],
    mocap_pos_in: wp.array2d[wp.vec3],
    mocap_quat_in: wp.array2d[wp.quat],
    userdata_in: wp.array2d[float],
    # In:
    sig_in: int,
    active_in: wp.array[bool],
    # Out:
    state_out: wp.array2d[float],
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
        elif element == State.HISTORY:
          for j in range(nhistory):
            state_out[worldid, adr + j] = history_in[worldid, j]
          adr += nhistory
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
          adr += neq
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
        elif element == State.USERDATA:
          for j in range(nuserdata):
            state_out[worldid, adr + j] = userdata_in[worldid, j]
          adr += nuserdata

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
      m.nuserdata,
      m.nhistory,
      d.time,
      d.qpos,
      d.qvel,
      d.act,
      d.history,
      d.qacc_warmstart,
      d.ctrl,
      d.qfrc_applied,
      d.xfrc_applied,
      d.eq_active,
      d.mocap_pos,
      d.mocap_quat,
      d.userdata,
      int(sig),
      active or wp.ones(d.nworld, dtype=bool),
    ],
    outputs=[state],
  )


def set_state(m: Model, d: Data, state: wp.array2d[float], sig: int, active: Optional[wp.array] = None):
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

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def _set_state(
    # Model:
    nq: int,
    nv: int,
    nu: int,
    na: int,
    nbody: int,
    neq: int,
    nmocap: int,
    nuserdata: int,
    nhistory: int,
    # In:
    sig_in: int,
    active_in: wp.array[bool],
    state_in: wp.array2d[float],
    # Data out:
    time_out: wp.array[float],
    qpos_out: wp.array2d[float],
    qvel_out: wp.array2d[float],
    act_out: wp.array2d[float],
    history_out: wp.array2d[float],
    qacc_warmstart_out: wp.array2d[float],
    ctrl_out: wp.array2d[float],
    qfrc_applied_out: wp.array2d[float],
    xfrc_applied_out: wp.array2d[wp.spatial_vector],
    eq_active_out: wp.array2d[bool],
    mocap_pos_out: wp.array2d[wp.vec3],
    mocap_quat_out: wp.array2d[wp.quat],
    userdata_out: wp.array2d[float],
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
        elif element == State.HISTORY:
          for j in range(nhistory):
            history_out[worldid, j] = state_in[worldid, adr + j]
          adr += nhistory
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
          adr += neq
        elif element == State.MOCAP_POS:
          for j in range(nmocap):
            pos = wp.vec3(
              state_in[worldid, adr + 0],
              state_in[worldid, adr + 1],
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
        elif element == State.USERDATA:
          for j in range(nuserdata):
            userdata_out[worldid, j] = state_in[worldid, adr + j]
          adr += nuserdata

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
      m.nuserdata,
      m.nhistory,
      int(sig),
      active or wp.ones(d.nworld, dtype=bool),
      state,
    ],
    outputs=[
      d.time,
      d.qpos,
      d.qvel,
      d.act,
      d.history,
      d.qacc_warmstart,
      d.ctrl,
      d.qfrc_applied,
      d.xfrc_applied,
      d.eq_active,
      d.mocap_pos,
      d.mocap_quat,
      d.userdata,
    ],
  )


@wp.func
def _phi(s: float, i: int) -> float:
  """1D trilinear basis function (order=1 only).

  phi(s, 0) = 1 - s
  phi(s, 1) = s
  """
  if i == 0:
    return 1.0 - s
  return s


@wp.func
def eval_basis_trilinear(local: wp.vec3, node_idx: int) -> float:
  """Evaluate trilinear basis function for node_idx at local coords [0,1]^3.

  For order=1 (trilinear), node_idx encodes (i,j,k) via bits:
    k = node_idx & 1, j = (node_idx >> 1) & 1, i = (node_idx >> 2) & 1
  """
  k = node_idx & 1
  j = (node_idx >> 1) & 1
  i = (node_idx >> 2) & 1
  return _phi(local[0], i) * _phi(local[1], j) * _phi(local[2], k)


@wp.func
def select_top4_weights(
  # In:
  W_mat: wp.mat33,
  b_mat: wp.mat33,
) -> tuple[wp.vec4i, wp.vec4]:
  """Selects top 4 weights and their corresponding body IDs from 8 voxel corners."""
  selected_b = wp.vec4i(-1, -1, -1, -1)
  selected_W = wp.vec4(0.0, 0.0, 0.0, 0.0)

  local_W = W_mat
  for p in range(4):
    max_w = -1.0
    max_b = -1
    max_r = -1
    max_c = -1
    for r in range(3):
      for c in range(3):
        idx = 3 * r + c
        if idx < 8:
          w = local_W[r, c]
          if w > max_w:
            max_w = w
            max_b = int(b_mat[r, c])
            max_r = r
            max_c = c
    # Record top choice for this pass and mark it as visited
    if max_r >= 0:
      local_W[max_r, max_c] = -1.0

      if p == 0:
        selected_b = wp.vec4i(max_b, -1, -1, -1)
        selected_W = wp.vec4(max_w, 0.0, 0.0, 0.0)
      elif p == 1:
        selected_b = wp.vec4i(selected_b[0], max_b, -1, -1)
        selected_W = wp.vec4(selected_W[0], max_w, 0.0, 0.0)
      elif p == 2:
        selected_b = wp.vec4i(selected_b[0], selected_b[1], max_b, -1)
        selected_W = wp.vec4(selected_W[0], selected_W[1], max_w, 0.0)
      else:
        selected_b = wp.vec4i(selected_b[0], selected_b[1], selected_b[2], max_b)
        selected_W = wp.vec4(selected_W[0], selected_W[1], selected_W[2], max_w)

  # Normalize selected weights
  sum_W = selected_W[0] + selected_W[1] + selected_W[2] + selected_W[3]
  if sum_W > 1.0e-5:
    selected_W = wp.vec4(
      selected_W[0] / sum_W,
      selected_W[1] / sum_W,
      selected_W[2] / sum_W,
      selected_W[3] / sum_W,
    )

  return selected_b, selected_W


@wp.func
def get_face_metadata(
  # In:
  cellnum_x: int,
  cellnum_y: int,
  cellnum_z: int,
  face_elem_idx: int,
  order_abs: int,
) -> Tuple[int, int, int, int, int, int]:
  size01 = cellnum_y * cellnum_z
  size23 = cellnum_x * cellnum_z
  size45 = cellnum_x * cellnum_y

  face_id = 0
  within_face = 0

  if face_elem_idx < size01:
    face_id = 0
    within_face = face_elem_idx
  elif face_elem_idx < 2 * size01:
    face_id = 1
    within_face = face_elem_idx - size01
  elif face_elem_idx < 2 * size01 + size23:
    face_id = 2
    within_face = face_elem_idx - 2 * size01
  elif face_elem_idx < 2 * size01 + 2 * size23:
    face_id = 3
    within_face = face_elem_idx - 2 * size01 - size23
  elif face_elem_idx < 2 * size01 + 2 * size23 + size45:
    face_id = 4
    within_face = face_elem_idx - 2 * size01 - 2 * size23
  else:
    face_id = 5
    within_face = face_elem_idx - 2 * size01 - 2 * size23 - size45

  normal_axis = face_id // 2

  c1 = 0
  if face_id == 0 or face_id == 1:
    c1 = cellnum_z
  elif face_id == 2 or face_id == 3:
    c1 = cellnum_x
  else:
    c1 = cellnum_y

  fixed_dim = 0
  if normal_axis == 0:
    fixed_dim = cellnum_x
  elif normal_axis == 1:
    fixed_dim = cellnum_y
  else:
    fixed_dim = cellnum_z
  g_fixed = (face_id % 2) * fixed_dim * order_abs

  q0 = within_face // c1
  q1 = within_face % c1

  ny_g = cellnum_y * order_abs + 1
  nz_g = cellnum_z * order_abs + 1

  return normal_axis, g_fixed, q0, q1, ny_g, nz_g


@wp.func
def gather_face_node_index_fast(
  # In:
  normal_axis: int,
  g_fixed: int,
  q0: int,
  q1: int,
  ny_g: int,
  nz_g: int,
  local_idx: int,
  order_abs: int,
) -> int:
  l0 = local_idx // (order_abs + 1)
  l1 = local_idx % (order_abs + 1)

  g = wp.vec3i(0, 0, 0)
  if normal_axis == 0:
    g = wp.vec3i(g_fixed, q0 * order_abs + l0, q1 * order_abs + l1)
  elif normal_axis == 1:
    g = wp.vec3i(q1 * order_abs + l1, g_fixed, q0 * order_abs + l0)
  else:
    g = wp.vec3i(q0 * order_abs + l0, q1 * order_abs + l1, g_fixed)

  return g[0] * ny_g * nz_g + g[1] * nz_g + g[2]


@wp.func
def gather_face_node_index(
  # In:
  cellnum_x: int,
  cellnum_y: int,
  cellnum_z: int,
  face_elem_idx: int,
  local_idx: int,
  order_abs: int,
) -> int:
  normal_axis, g_fixed, q0, q1, ny_g, nz_g = get_face_metadata(cellnum_x, cellnum_y, cellnum_z, face_elem_idx, order_abs)
  return gather_face_node_index_fast(normal_axis, g_fixed, q0, q1, ny_g, nz_g, local_idx, order_abs)


@wp.func
def compute_interp_face_quat(
  # Data in:
  flexnode_xpos_in: wp.array2d[wp.vec3],
  # In:
  cellnum_x: int,
  cellnum_y: int,
  cellnum_z: int,
  face_elem_idx: int,
  nstart: int,
  order_abs: int,
  worldid: int,
) -> wp.quat:
  normal_axis, g_fixed, q0, q1, ny_g, nz_g = get_face_metadata(
    cellnum_x,
    cellnum_y,
    cellnum_z,
    face_elem_idx,
    order_abs,
  )

  t1 = wp.vec3(0.0)
  t2 = wp.vec3(0.0)

  npc = (order_abs + 1) * (order_abs + 1)

  for local_idx in range(9):
    if local_idx < npc:
      gidx = gather_face_node_index_fast(
        normal_axis,
        g_fixed,
        q0,
        q1,
        ny_g,
        nz_g,
        local_idx,
        order_abs,
      )
      node_pos = flexnode_xpos_in[worldid, nstart + gidx]

      l0 = local_idx // (order_abs + 1)
      l1 = local_idx % (order_abs + 1)

      dphi0 = -1.0 + 2.0 * float(l0)
      dphi1 = -1.0 + 2.0 * float(l1)
      phi0 = 0.5
      phi1 = 0.5

      grad0 = dphi0 * phi1
      grad1 = phi0 * dphi1

      t1 += node_pos * grad0
      t2 += node_pos * grad1

  normal = wp.cross(t1, t2)

  F = wp.mat33(0.0)
  if normal_axis == 0:
    F = wp.mat33(
      normal[0],
      t1[0],
      t2[0],
      normal[1],
      t1[1],
      t2[1],
      normal[2],
      t1[2],
      t2[2],
    )
  elif normal_axis == 1:
    F = wp.mat33(
      t2[0],
      normal[0],
      t1[0],
      t2[1],
      normal[1],
      t1[1],
      t2[2],
      normal[2],
      t1[2],
    )
  else:
    F = wp.mat33(
      t1[0],
      t2[0],
      normal[0],
      t1[1],
      t2[1],
      normal[1],
      t1[2],
      t2[2],
      normal[2],
    )

  return mat33_to_quat_polar(F)


@wp.func
def flex_phi(s: float, i: int, order: int) -> float:
  return 1.0 - s if i == 0 else s


@wp.func
def flex_dphi(s: float, i: int, order: int) -> float:
  return -1.0 if i == 0 else 1.0


@wp.func
def dphi2D(s0: float, l0: int, s1: float, l1: int, order: int, direction: int) -> float:
  if direction == 0:
    return flex_dphi(s0, l0, order) * flex_phi(s1, l1, order)
  else:
    return flex_phi(s0, l0, order) * flex_dphi(s1, l1, order)
