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

from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import Geom
from mujoco.mjx.third_party.mujoco_warp._src.math import motion_cross
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import TileSet
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec6
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel

wp.set_module_options({"enable_backward": False})


@wp.kernel
def mul_m_sparse_diag(
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

  if skip[worldid]:
    return

  res[worldid, dofid] = qM_in[worldid, 0, dof_Madr[dofid]] * vec[worldid, dofid]


@wp.kernel
def mul_m_sparse_ij(
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

  if skip[worldid]:
    return

  i = qM_mulm_i[elementid]
  j = qM_mulm_j[elementid]
  madr_ij = qM_madr_ij[elementid]

  qM_ij = qM_in[worldid, 0, madr_ij]

  wp.atomic_add(res[worldid], i, qM_ij * vec[worldid, j])
  wp.atomic_add(res[worldid], j, qM_ij * vec[worldid, i])


@cache_kernel
def mul_m_dense(tile: TileSet):
  """Returns a matmul kernel for some tile size"""

  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
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

    if skip[worldid]:
      return

    dofid = adr[nodeid]
    qM_tile = wp.tile_load(qM_in[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(dofid, dofid))
    vec_tile = wp.tile_load(vec[worldid], shape=(TILE_SIZE, 1), offset=(dofid, 0))
    res_tile = wp.tile_matmul(qM_tile, vec_tile)
    wp.tile_store(res[worldid], res_tile, offset=(dofid, 0))

  return kernel


@event_scope
def mul_m(
  m: Model,
  d: Data,
  res: wp.array2d(dtype=float),
  vec: wp.array2d(dtype=float),
  skip: wp.array(dtype=bool),
  M: wp.array3d(dtype=float) = None,
):
  """Multiply vectors by inertia matrix.

  Args:
    m (Model): The model containing kinematic and dynamic information (device).
    d (Data): The data object containing the current state and output arrays (device).
    res (wp.array2d(dtype=float)): Result: qM @ vec.
    vec (wp.array2d(dtype=float)): Input vector to multiply by qM.
    skip (wp.array(dtype=flooat)): Skip output.
    M (wp.array3d(dtype=float), optional): Input matrix: M @ vec.
  """

  if M is None:
    M = d.qM

  if m.opt.is_sparse:
    wp.launch(
      mul_m_sparse_diag,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_Madr, M, vec, skip],
      outputs=[res],
    )

    wp.launch(
      mul_m_sparse_ij,
      dim=(d.nworld, m.qM_madr_ij.size),
      inputs=[m.qM_mulm_i, m.qM_mulm_j, m.qM_madr_ij, M, vec, skip],
      outputs=[res],
    )

  else:
    for tile in m.qM_tiles:
      wp.launch_tiled(
        mul_m_dense(tile),
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
  """
  Map applied forces at each body via Jacobians to dof space and accumulate.

  Args:
    m (Model): The model containing kinematic and dynamic information (device).
    d (Data): The data object containing the current state and output arrays (device).
    qfrc (wp.array2d(dtype=float)): Total applied force mapped to dof space.
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
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
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
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
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
    njmax_in,
    nacon_in,
    contact_frame_in,
    contact_friction_in,
    contact_dim_in,
    contact_efc_address_in,
    efc_force_in,
    worldid,
    contactid,
    to_world_frame,
  )


def contact_force(
  m: Model,
  d: Data,
  contact_ids: wp.array(dtype=int),
  to_world_frame: bool,
  force: wp.array(dtype=wp.spatial_vector),
):
  """
  Compute forces for contacts in Data.

  Args:
    m (Model): The model containing kinematic and dynamic information (device).
    d (Data): The data object containing the current state and output arrays (device).
    contact_ids (wp.array(dtype=int)): IDs for each contact.
    to_world_frame (bool): If True, map force from contact to world frame.
    force (wp.array(dtype=wp.spatial_vector)): Contact forces.
  """
  wp.launch(
    contact_force_kernel,
    dim=(contact_ids.size,),
    inputs=[
      m.opt.cone,
      d.njmax,
      d.nacon,
      d.contact.frame,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.contact.worldid,
      d.efc.force,
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
