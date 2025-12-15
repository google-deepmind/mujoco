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


import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import util_misc
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import CamLightType
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType
from mujoco.mjx.third_party.mujoco_warp._src.types import TileSet
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType
from mujoco.mjx.third_party.mujoco_warp._src.types import WrapType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10
from mujoco.mjx.third_party.mujoco_warp._src.types import vec11
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import nested_kernel

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _kinematics_level(
  # Model:
  qpos0: wp.array2d(dtype=float),
  body_parentid: wp.array(dtype=int),
  body_mocapid: wp.array(dtype=int),
  body_jntnum: wp.array(dtype=int),
  body_jntadr: wp.array(dtype=int),
  body_pos: wp.array2d(dtype=wp.vec3),
  body_quat: wp.array2d(dtype=wp.quat),
  body_ipos: wp.array2d(dtype=wp.vec3),
  body_iquat: wp.array2d(dtype=wp.quat),
  jnt_type: wp.array(dtype=int),
  jnt_qposadr: wp.array(dtype=int),
  jnt_pos: wp.array2d(dtype=wp.vec3),
  jnt_axis: wp.array2d(dtype=wp.vec3),
  # Data in:
  qpos_in: wp.array2d(dtype=float),
  mocap_pos_in: wp.array2d(dtype=wp.vec3),
  mocap_quat_in: wp.array2d(dtype=wp.quat),
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  xpos_out: wp.array2d(dtype=wp.vec3),
  xquat_out: wp.array2d(dtype=wp.quat),
  xmat_out: wp.array2d(dtype=wp.mat33),
  xipos_out: wp.array2d(dtype=wp.vec3),
  ximat_out: wp.array2d(dtype=wp.mat33),
  xanchor_out: wp.array2d(dtype=wp.vec3),
  xaxis_out: wp.array2d(dtype=wp.vec3),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  jntadr = body_jntadr[bodyid]
  jntnum = body_jntnum[bodyid]
  qpos = qpos_in[worldid]
  body_pos_id = worldid % body_pos.shape[0]
  body_quat_id = worldid % body_quat.shape[0]
  jnt_axis_id = worldid % jnt_axis.shape[0]

  free_joint = False
  if jntnum == 1:
    jnt_type_ = jnt_type[jntadr]
    free_joint = jnt_type_ == JointType.FREE

  if free_joint:
    # free joint
    qadr = jnt_qposadr[jntadr]
    xpos = wp.vec3(qpos[qadr], qpos[qadr + 1], qpos[qadr + 2])
    xquat = wp.quat(qpos[qadr + 3], qpos[qadr + 4], qpos[qadr + 5], qpos[qadr + 6])
    xquat = wp.normalize(xquat)
    xanchor_out[worldid, jntadr] = xpos
    xaxis_out[worldid, jntadr] = jnt_axis[jnt_axis_id, jntadr]
  else:
    # regular or no joints
    # apply fixed translation and rotation relative to parent
    qpos0_id = worldid % qpos0.shape[0]
    jnt_pos_id = worldid % jnt_pos.shape[0]
    pid = body_parentid[bodyid]

    # mocap bodies have world body as parent
    mocapid = body_mocapid[bodyid]
    if mocapid >= 0:
      xpos = mocap_pos_in[worldid, mocapid]
      xquat = mocap_quat_in[worldid, mocapid]
    else:
      xpos = body_pos[body_pos_id, bodyid]
      xquat = body_quat[body_quat_id, bodyid]

    if pid >= 0:
      xpos = xmat_in[worldid, pid] @ xpos + xpos_in[worldid, pid]
      xquat = math.mul_quat(xquat_in[worldid, pid], xquat)

    for _ in range(jntnum):
      qadr = jnt_qposadr[jntadr]
      jnt_type_ = jnt_type[jntadr]
      jnt_axis_ = jnt_axis[jnt_axis_id, jntadr]
      xanchor = math.rot_vec_quat(jnt_pos[jnt_pos_id, jntadr], xquat) + xpos
      xaxis = math.rot_vec_quat(jnt_axis_, xquat)

      if jnt_type_ == JointType.BALL:
        qloc = wp.quat(qpos[qadr + 0], qpos[qadr + 1], qpos[qadr + 2], qpos[qadr + 3])
        qloc = wp.normalize(qloc)
        xquat = math.mul_quat(xquat, qloc)
        # correct for off-center rotation
        xpos = xanchor - math.rot_vec_quat(jnt_pos[jnt_pos_id, jntadr], xquat)
      elif jnt_type_ == JointType.SLIDE:
        xpos += xaxis * (qpos[qadr] - qpos0[qpos0_id, qadr])
      elif jnt_type_ == JointType.HINGE:
        qpos0_ = qpos0[qpos0_id, qadr]
        qloc_ = math.axis_angle_to_quat(jnt_axis_, qpos[qadr] - qpos0_)
        xquat = math.mul_quat(xquat, qloc_)
        # correct for off-center rotation
        xpos = xanchor - math.rot_vec_quat(jnt_pos[jnt_pos_id, jntadr], xquat)

      xanchor_out[worldid, jntadr] = xanchor
      xaxis_out[worldid, jntadr] = xaxis
      jntadr += 1

  xpos_out[worldid, bodyid] = xpos
  xquat = wp.normalize(xquat)
  xquat_out[worldid, bodyid] = xquat
  xmat_out[worldid, bodyid] = math.quat_to_mat(xquat)
  xipos_out[worldid, bodyid] = xpos + math.rot_vec_quat(body_ipos[worldid % body_ipos.shape[0], bodyid], xquat)
  ximat_out[worldid, bodyid] = math.quat_to_mat(math.mul_quat(xquat, body_iquat[worldid % body_iquat.shape[0], bodyid]))


@wp.kernel
def _geom_local_to_global(
  # Model:
  body_rootid: wp.array(dtype=int),
  body_weldid: wp.array(dtype=int),
  body_mocapid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  geom_pos: wp.array2d(dtype=wp.vec3),
  geom_quat: wp.array2d(dtype=wp.quat),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  # Data out:
  geom_xpos_out: wp.array2d(dtype=wp.vec3),
  geom_xmat_out: wp.array2d(dtype=wp.mat33),
):
  worldid, geomid = wp.tid()
  bodyid = geom_bodyid[geomid]

  if body_weldid[bodyid] == 0 and body_mocapid[body_rootid[bodyid]] == -1:
    # geoms attached to the world are static (unless they are descended from mcocap bodies)
    # for such static geoms, geom_xpos and geom_xquat are computed only once during make_data
    return

  xpos = xpos_in[worldid, bodyid]
  xquat = xquat_in[worldid, bodyid]
  geom_xpos_out[worldid, geomid] = xpos + math.rot_vec_quat(geom_pos[worldid % geom_pos.shape[0], geomid], xquat)
  geom_xmat_out[worldid, geomid] = math.quat_to_mat(math.mul_quat(xquat, geom_quat[worldid % geom_quat.shape[0], geomid]))


@wp.kernel
def _site_local_to_global(
  # Model:
  site_bodyid: wp.array(dtype=int),
  site_pos: wp.array2d(dtype=wp.vec3),
  site_quat: wp.array2d(dtype=wp.quat),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  # Data out:
  site_xpos_out: wp.array2d(dtype=wp.vec3),
  site_xmat_out: wp.array2d(dtype=wp.mat33),
):
  worldid, siteid = wp.tid()
  bodyid = site_bodyid[siteid]
  xpos = xpos_in[worldid, bodyid]
  xquat = xquat_in[worldid, bodyid]
  site_xpos_out[worldid, siteid] = xpos + math.rot_vec_quat(site_pos[worldid % site_pos.shape[0], siteid], xquat)
  site_xmat_out[worldid, siteid] = math.quat_to_mat(math.mul_quat(xquat, site_quat[worldid % site_quat.shape[0], siteid]))


@wp.kernel
def _flex_vertices(
  # Model:
  flex_vertbodyid: wp.array(dtype=int),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  flexvert_xpos_out: wp.array2d(dtype=wp.vec3),
):
  worldid, vertid = wp.tid()
  flexvert_xpos_out[worldid, vertid] = xpos_in[worldid, flex_vertbodyid[vertid]]


@wp.kernel
def _flex_edges(
  # Model:
  nv: int,
  nflex: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_dofadr: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_edgeadr: wp.array(dtype=int),
  flex_edgenum: wp.array(dtype=int),
  flex_vertbodyid: wp.array(dtype=int),
  flex_edge: wp.array(dtype=wp.vec2i),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  flexedge_J_out: wp.array3d(dtype=float),
  flexedge_length_out: wp.array2d(dtype=float),
  flexedge_velocity_out: wp.array2d(dtype=float),
):
  worldid, edgeid = wp.tid()
  for i in range(nflex):
    locid = edgeid - flex_edgeadr[i]
    if locid >= 0 and locid < flex_edgenum[i]:
      f = i
      break
  vbase = flex_vertadr[f]
  v = flex_edge[edgeid]
  pos1 = flexvert_xpos_in[worldid, vbase + v[0]]
  pos2 = flexvert_xpos_in[worldid, vbase + v[1]]
  vec = pos2 - pos1
  vecnorm = wp.length(vec)
  flexedge_length_out[worldid, edgeid] = vecnorm
  # TODO(quaglino): use Jacobian
  b1 = flex_vertbodyid[vbase + v[0]]
  b2 = flex_vertbodyid[vbase + v[1]]
  i = body_dofadr[b1]
  j = body_dofadr[b2]
  vel1 = wp.vec3(qvel_in[worldid, i], qvel_in[worldid, i + 1], qvel_in[worldid, i + 2])
  vel2 = wp.vec3(qvel_in[worldid, j], qvel_in[worldid, j + 1], qvel_in[worldid, j + 2])
  edge = wp.normalize(vec)
  flexedge_velocity_out[worldid, edgeid] = wp.dot(vel2 - vel1, edge)
  # Edge jacobian
  for k in range(nv):
    jacp1, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, pos1, b1, k, worldid)
    jacp2, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, pos2, b2, k, worldid)
    jacdif = jacp2 - jacp1
    flexedge_J_out[worldid, edgeid, k] = wp.dot(jacdif, edge)


@event_scope
def kinematics(m: Model, d: Data):
  """Computes forward kinematics for all bodies, sites, geoms, and flexible elements.

  This function updates the global positions and orientations of all bodies, as well as the
  derived positions and orientations of geoms, sites, and flexible elements, based on the
  current joint positions and any attached mocap bodies.
  """
  for i in range(1, len(m.body_tree)):
    body_tree = m.body_tree[i]
    wp.launch(
      _kinematics_level,
      dim=(d.nworld, body_tree.size),
      inputs=[
        m.qpos0,
        m.body_parentid,
        m.body_mocapid,
        m.body_jntnum,
        m.body_jntadr,
        m.body_pos,
        m.body_quat,
        m.body_ipos,
        m.body_iquat,
        m.jnt_type,
        m.jnt_qposadr,
        m.jnt_pos,
        m.jnt_axis,
        d.qpos,
        d.mocap_pos,
        d.mocap_quat,
        d.xpos,
        d.xquat,
        d.xmat,
        body_tree,
      ],
      outputs=[d.xpos, d.xquat, d.xmat, d.xipos, d.ximat, d.xanchor, d.xaxis],
    )

  wp.launch(
    _geom_local_to_global,
    dim=(d.nworld, m.ngeom),
    inputs=[m.body_rootid, m.body_weldid, m.body_mocapid, m.geom_bodyid, m.geom_pos, m.geom_quat, d.xpos, d.xquat],
    outputs=[d.geom_xpos, d.geom_xmat],
  )

  wp.launch(
    _site_local_to_global,
    dim=(d.nworld, m.nsite),
    inputs=[m.site_bodyid, m.site_pos, m.site_quat, d.xpos, d.xquat],
    outputs=[d.site_xpos, d.site_xmat],
  )


@event_scope
def flex(m: Model, d: Data):
  wp.launch(_flex_vertices, dim=(d.nworld, m.nflexvert), inputs=[m.flex_vertbodyid, d.xpos], outputs=[d.flexvert_xpos])
  wp.launch(
    _flex_edges,
    dim=(d.nworld, m.nflexedge),
    inputs=[
      m.nv,
      m.nflex,
      m.body_parentid,
      m.body_rootid,
      m.body_dofadr,
      m.dof_bodyid,
      m.flex_vertadr,
      m.flex_edgeadr,
      m.flex_edgenum,
      m.flex_vertbodyid,
      m.flex_edge,
      d.qvel,
      d.subtree_com,
      d.cdof,
      d.flexvert_xpos,
    ],
    outputs=[d.flexedge_J, d.flexedge_length, d.flexedge_velocity],
  )


@wp.kernel
def _subtree_com_init(
  # Model:
  body_mass: wp.array2d(dtype=float),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  subtree_com_out: wp.array2d(dtype=wp.vec3),
):
  worldid, bodyid = wp.tid()
  subtree_com_out[worldid, bodyid] = xipos_in[worldid, bodyid] * body_mass[worldid % body_mass.shape[0], bodyid]


@wp.kernel
def _subtree_com_acc(
  # Model:
  body_parentid: wp.array(dtype=int),
  # Data in:
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  subtree_com_out: wp.array2d(dtype=wp.vec3),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  pid = body_parentid[bodyid]
  if bodyid != 0:
    wp.atomic_add(subtree_com_out, worldid, pid, subtree_com_in[worldid, bodyid])


@wp.kernel
def _subtree_div(
  # Model:
  body_subtreemass: wp.array2d(dtype=float),
  # Data in:
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  subtree_com_out: wp.array2d(dtype=wp.vec3),
):
  worldid, bodyid = wp.tid()
  com = subtree_com_in[worldid, bodyid]
  mass = body_subtreemass[worldid % body_subtreemass.shape[0], bodyid]
  if mass != 0.0:
    subtree_com_out[worldid, bodyid] = com / mass


@wp.kernel
def _cinert(
  # Model:
  body_rootid: wp.array(dtype=int),
  body_mass: wp.array2d(dtype=float),
  body_inertia: wp.array2d(dtype=wp.vec3),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  cinert_out: wp.array2d(dtype=vec10),
):
  worldid, bodyid = wp.tid()
  mat = ximat_in[worldid, bodyid]
  inert = body_inertia[worldid % body_inertia.shape[0], bodyid]
  mass = body_mass[worldid % body_mass.shape[0], bodyid]
  dif = xipos_in[worldid, bodyid] - subtree_com_in[worldid, body_rootid[bodyid]]
  # express inertia in com-based frame (mju_inertCom)

  res = vec10()
  # res_rot = mat * diag(inert) * mat'
  tmp = mat @ wp.diag(inert) @ wp.transpose(mat)
  res[0] = tmp[0, 0]
  res[1] = tmp[1, 1]
  res[2] = tmp[2, 2]
  res[3] = tmp[0, 1]
  res[4] = tmp[0, 2]
  res[5] = tmp[1, 2]
  # res_rot -= mass * dif_cross * dif_cross
  res[0] += mass * (dif[1] * dif[1] + dif[2] * dif[2])
  res[1] += mass * (dif[0] * dif[0] + dif[2] * dif[2])
  res[2] += mass * (dif[0] * dif[0] + dif[1] * dif[1])
  res[3] -= mass * dif[0] * dif[1]
  res[4] -= mass * dif[0] * dif[2]
  res[5] -= mass * dif[1] * dif[2]
  # res_tran = mass * dif
  res[6] = mass * dif[0]
  res[7] = mass * dif[1]
  res[8] = mass * dif[2]
  # res_mass = mass
  res[9] = mass

  cinert_out[worldid, bodyid] = res


@wp.kernel
def _cdof(
  # Model:
  body_rootid: wp.array(dtype=int),
  jnt_type: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  jnt_bodyid: wp.array(dtype=int),
  # Data in:
  xmat_in: wp.array2d(dtype=wp.mat33),
  xanchor_in: wp.array2d(dtype=wp.vec3),
  xaxis_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  cdof_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, jntid = wp.tid()
  bodyid = jnt_bodyid[jntid]
  dofid = jnt_dofadr[jntid]
  jnt_type_ = jnt_type[jntid]
  xaxis = xaxis_in[worldid, jntid]
  xmat = wp.transpose(xmat_in[worldid, bodyid])

  # compute com-anchor vector
  offset = subtree_com_in[worldid, body_rootid[bodyid]] - xanchor_in[worldid, jntid]

  res = cdof_out[worldid]
  if jnt_type_ == JointType.FREE:
    res[dofid + 0] = wp.spatial_vector(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    res[dofid + 1] = wp.spatial_vector(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    res[dofid + 2] = wp.spatial_vector(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    # I_3 rotation in child frame (assume no subsequent rotations)
    res[dofid + 3] = wp.spatial_vector(xmat[0], wp.cross(xmat[0], offset))
    res[dofid + 4] = wp.spatial_vector(xmat[1], wp.cross(xmat[1], offset))
    res[dofid + 5] = wp.spatial_vector(xmat[2], wp.cross(xmat[2], offset))
  elif jnt_type_ == JointType.BALL:  # ball
    # I_3 rotation in child frame (assume no subsequent rotations)
    res[dofid + 0] = wp.spatial_vector(xmat[0], wp.cross(xmat[0], offset))
    res[dofid + 1] = wp.spatial_vector(xmat[1], wp.cross(xmat[1], offset))
    res[dofid + 2] = wp.spatial_vector(xmat[2], wp.cross(xmat[2], offset))
  elif jnt_type_ == JointType.SLIDE:
    res[dofid] = wp.spatial_vector(wp.vec3(0.0), xaxis)
  elif jnt_type_ == JointType.HINGE:  # hinge
    res[dofid] = wp.spatial_vector(xaxis, wp.cross(xaxis, offset))


@event_scope
def com_pos(m: Model, d: Data):
  """Computes subtree center of mass positions.

  Transforms inertia and motion to global frame centered at subtree CoM. Accumulates the
  mass-weighted positions up the kinematic tree, divides by total mass, and computes composite
  inertias and motion degrees of freedom in the subtree CoM frame.
  """
  wp.launch(_subtree_com_init, dim=(d.nworld, m.nbody), inputs=[m.body_mass, d.xipos], outputs=[d.subtree_com])

  for i in reversed(range(len(m.body_tree))):
    body_tree = m.body_tree[i]
    wp.launch(
      _subtree_com_acc,
      dim=(d.nworld, body_tree.size),
      inputs=[m.body_parentid, d.subtree_com, body_tree],
      outputs=[d.subtree_com],
    )

  wp.launch(_subtree_div, dim=(d.nworld, m.nbody), inputs=[m.body_subtreemass, d.subtree_com], outputs=[d.subtree_com])
  wp.launch(
    _cinert,
    dim=(d.nworld, m.nbody),
    inputs=[m.body_rootid, m.body_mass, m.body_inertia, d.xipos, d.ximat, d.subtree_com],
    outputs=[d.cinert],
  )
  wp.launch(
    _cdof,
    dim=(d.nworld, m.njnt),
    inputs=[m.body_rootid, m.jnt_type, m.jnt_dofadr, m.jnt_bodyid, d.xmat, d.xanchor, d.xaxis, d.subtree_com],
    outputs=[d.cdof],
  )


@wp.kernel
def _cam_local_to_global(
  # Model:
  cam_mode: wp.array(dtype=int),
  cam_bodyid: wp.array(dtype=int),
  cam_targetbodyid: wp.array(dtype=int),
  cam_pos: wp.array2d(dtype=wp.vec3),
  cam_quat: wp.array2d(dtype=wp.quat),
  cam_poscom0: wp.array2d(dtype=wp.vec3),
  cam_pos0: wp.array2d(dtype=wp.vec3),
  cam_mat0: wp.array2d(dtype=wp.mat33),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  cam_xpos_out: wp.array2d(dtype=wp.vec3),
  cam_xmat_out: wp.array2d(dtype=wp.mat33),
):
  worldid, camid = wp.tid()
  cam_pos_id = worldid % cam_pos.shape[0]
  cam_quat_id = worldid % cam_quat.shape[0]
  is_target_cam = (cam_mode[camid] == CamLightType.TARGETBODY) or (cam_mode[camid] == CamLightType.TARGETBODYCOM)
  invalid_target = is_target_cam and (cam_targetbodyid[camid] < 0)
  if invalid_target:
    bodyid = cam_bodyid[camid]
    xpos = xpos_in[worldid, bodyid]
    xquat = xquat_in[worldid, bodyid]
    cam_xpos_out[worldid, camid] = xpos + math.rot_vec_quat(cam_pos[cam_pos_id, camid], xquat)
    cam_xmat_out[worldid, camid] = math.quat_to_mat(math.mul_quat(xquat, cam_quat[cam_quat_id, camid]))
  elif cam_mode[camid] == CamLightType.TRACK:
    cam_xmat_out[worldid, camid] = cam_mat0[worldid % cam_mat0.shape[0], camid]
    body_xpos = xpos_in[worldid, cam_bodyid[camid]]
    cam_xpos_out[worldid, camid] = body_xpos + cam_pos0[worldid % cam_pos0.shape[0], camid]
  elif cam_mode[camid] == CamLightType.TRACKCOM:
    cam_xmat_out[worldid, camid] = cam_mat0[worldid % cam_mat0.shape[0], camid]
    cam_xpos_out[worldid, camid] = (
      subtree_com_in[worldid, cam_bodyid[camid]] + cam_poscom0[worldid % cam_poscom0.shape[0], camid]
    )
  elif cam_mode[camid] == CamLightType.TARGETBODY or cam_mode[camid] == CamLightType.TARGETBODYCOM:
    bodyid = cam_bodyid[camid]
    xpos = xpos_in[worldid, bodyid]
    xquat = xquat_in[worldid, bodyid]
    cam_xpos_out[worldid, camid] = xpos + math.rot_vec_quat(cam_pos[cam_pos_id, camid], xquat)
    pos = xpos_in[worldid, cam_targetbodyid[camid]]
    if cam_mode[camid] == CamLightType.TARGETBODYCOM:
      pos = subtree_com_in[worldid, cam_targetbodyid[camid]]
    # zaxis = -desired camera direction, in global frame
    mat_3 = wp.normalize(cam_xpos_out[worldid, camid] - pos)
    # xaxis: orthogonal to zaxis and to (0,0,1)
    mat_1 = wp.normalize(wp.cross(wp.vec3(0.0, 0.0, 1.0), mat_3))
    mat_2 = wp.normalize(wp.cross(mat_3, mat_1))
    # fmt: off
    cam_xmat_out[worldid, camid] = wp.mat33(
      mat_1[0], mat_2[0], mat_3[0],
      mat_1[1], mat_2[1], mat_3[1],
      mat_1[2], mat_2[2], mat_3[2]
    )
    # fmt: on
  else:
    bodyid = cam_bodyid[camid]
    xpos = xpos_in[worldid, bodyid]
    xquat = xquat_in[worldid, bodyid]
    cam_xpos_out[worldid, camid] = xpos + math.rot_vec_quat(cam_pos[cam_pos_id, camid], xquat)
    cam_xmat_out[worldid, camid] = math.quat_to_mat(math.mul_quat(xquat, cam_quat[cam_quat_id, camid]))


@wp.kernel
def _light_local_to_global(
  # Model:
  light_mode: wp.array(dtype=int),
  light_bodyid: wp.array(dtype=int),
  light_targetbodyid: wp.array(dtype=int),
  light_pos: wp.array2d(dtype=wp.vec3),
  light_dir: wp.array2d(dtype=wp.vec3),
  light_poscom0: wp.array2d(dtype=wp.vec3),
  light_pos0: wp.array2d(dtype=wp.vec3),
  light_dir0: wp.array2d(dtype=wp.vec3),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  light_xpos_out: wp.array2d(dtype=wp.vec3),
  light_xdir_out: wp.array2d(dtype=wp.vec3),
):
  worldid, lightid = wp.tid()
  light_pos_id = worldid % light_pos.shape[0]
  light_dir_id = worldid % light_dir.shape[0]
  is_target_light = (light_mode[lightid] == CamLightType.TARGETBODY) or (light_mode[lightid] == CamLightType.TARGETBODYCOM)
  invalid_target = is_target_light and (light_targetbodyid[lightid] < 0)
  if invalid_target:
    bodyid = light_bodyid[lightid]
    xpos = xpos_in[worldid, bodyid]
    xquat = xquat_in[worldid, bodyid]
    light_xpos_out[worldid, lightid] = xpos + math.rot_vec_quat(light_pos[light_pos_id, lightid], xquat)
    light_xdir_out[worldid, lightid] = math.rot_vec_quat(light_dir[light_dir_id, lightid], xquat)
    return
  elif light_mode[lightid] == CamLightType.TRACK:
    light_xdir_out[worldid, lightid] = light_dir0[worldid % light_dir0.shape[0], lightid]
    body_xpos = xpos_in[worldid, light_bodyid[lightid]]
    light_xpos_out[worldid, lightid] = body_xpos + light_pos0[worldid % light_pos0.shape[0], lightid]
  elif light_mode[lightid] == CamLightType.TRACKCOM:
    light_xdir_out[worldid, lightid] = light_dir0[worldid % light_dir0.shape[0], lightid]
    light_xpos_out[worldid, lightid] = (
      subtree_com_in[worldid, light_bodyid[lightid]] + light_poscom0[worldid % light_poscom0.shape[0], lightid]
    )
  elif light_mode[lightid] == CamLightType.TARGETBODY or light_mode[lightid] == CamLightType.TARGETBODYCOM:
    bodyid = light_bodyid[lightid]
    xpos = xpos_in[worldid, bodyid]
    xquat = xquat_in[worldid, bodyid]
    light_xpos_out[worldid, lightid] = xpos + math.rot_vec_quat(light_pos[light_pos_id, lightid], xquat)
    pos = xpos_in[worldid, light_targetbodyid[lightid]]
    if light_mode[lightid] == CamLightType.TARGETBODYCOM:
      pos = subtree_com_in[worldid, light_targetbodyid[lightid]]
    light_xdir_out[worldid, lightid] = pos - light_xpos_out[worldid, lightid]
  else:
    bodyid = light_bodyid[lightid]
    xpos = xpos_in[worldid, bodyid]
    xquat = xquat_in[worldid, bodyid]
    light_xpos_out[worldid, lightid] = xpos + math.rot_vec_quat(light_pos[light_pos_id, lightid], xquat)
    light_xdir_out[worldid, lightid] = math.rot_vec_quat(light_dir[light_dir_id, lightid], xquat)

  light_xdir_out[worldid, lightid] = wp.normalize(light_xdir_out[worldid, lightid])


@event_scope
def camlight(m: Model, d: Data):
  """Computes camera and light positions and orientations.

  Updates the global positions and orientations for all cameras and lights in the model,
  including special handling for tracking and target modes.
  """
  wp.launch(
    _cam_local_to_global,
    dim=(d.nworld, m.ncam),
    inputs=[
      m.cam_mode,
      m.cam_bodyid,
      m.cam_targetbodyid,
      m.cam_pos,
      m.cam_quat,
      m.cam_poscom0,
      m.cam_pos0,
      m.cam_mat0,
      d.xpos,
      d.xquat,
      d.subtree_com,
    ],
    outputs=[d.cam_xpos, d.cam_xmat],
  )
  wp.launch(
    _light_local_to_global,
    dim=(d.nworld, m.nlight),
    inputs=[
      m.light_mode,
      m.light_bodyid,
      m.light_targetbodyid,
      m.light_pos,
      m.light_dir,
      m.light_poscom0,
      m.light_pos0,
      m.light_dir0,
      d.xpos,
      d.xquat,
      d.subtree_com,
    ],
    outputs=[d.light_xpos, d.light_xdir],
  )


@wp.kernel
def _crb_accumulate(
  # Model:
  body_parentid: wp.array(dtype=int),
  # Data in:
  crb_in: wp.array2d(dtype=vec10),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  crb_out: wp.array2d(dtype=vec10),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  pid = body_parentid[bodyid]
  if pid == 0:
    return
  wp.atomic_add(crb_out, worldid, pid, crb_in[worldid, bodyid])


@wp.kernel
def _qM_sparse(
  # Model:
  dof_bodyid: wp.array(dtype=int),
  dof_parentid: wp.array(dtype=int),
  dof_Madr: wp.array(dtype=int),
  dof_armature: wp.array2d(dtype=float),
  # Data in:
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  crb_in: wp.array2d(dtype=vec10),
  # Data out:
  qM_out: wp.array3d(dtype=float),
):
  worldid, dofid = wp.tid()
  madr_ij = dof_Madr[dofid]  # dof_Madr is not batched
  bodyid = dof_bodyid[dofid]

  # init M(i,i) with armature inertia
  qM_out[worldid, 0, madr_ij] = dof_armature[worldid, dofid]

  # precompute buf = crb_body_i * cdof_i
  buf = math.inert_vec(crb_in[worldid, bodyid], cdof_in[worldid, dofid])

  # sparse backward pass over ancestors
  while dofid >= 0:
    qM_out[worldid, 0, madr_ij] += wp.dot(cdof_in[worldid, dofid], buf)
    madr_ij += 1
    dofid = dof_parentid[dofid]


@wp.kernel
def _qM_dense(
  # Model:
  dof_bodyid: wp.array(dtype=int),
  dof_parentid: wp.array(dtype=int),
  dof_armature: wp.array2d(dtype=float),
  # Data in:
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  crb_in: wp.array2d(dtype=vec10),
  # Data out:
  qM_out: wp.array3d(dtype=float),
):
  worldid, dofid = wp.tid()
  bodyid = dof_bodyid[dofid]
  # init M(i,i) with armature inertia.
  M = dof_armature[worldid % dof_armature.shape[0], dofid]

  # precompute buf = crb_body_i * cdof_i
  buf = math.inert_vec(crb_in[worldid, bodyid], cdof_in[worldid, dofid])
  M += wp.dot(cdof_in[worldid, dofid], buf)

  qM_out[worldid, dofid, dofid] = M

  # sparse backward pass over ancestors
  dofidi = dofid
  dofid = dof_parentid[dofid]
  while dofid >= 0:
    qMij = wp.dot(cdof_in[worldid, dofid], buf)
    qM_out[worldid, dofidi, dofid] += qMij
    qM_out[worldid, dofid, dofidi] += qMij
    dofid = dof_parentid[dofid]


@event_scope
def crb(m: Model, d: Data):
  """Computes composite rigid body inertias for each body and the joint-space inertia matrix.

  Accumulates composite rigid body inertias up the kinematic tree and computes the
  joint-space inertia matrix in either sparse or dense format, depending on model options.
  """
  wp.copy(d.crb, d.cinert)

  for i in reversed(range(len(m.body_tree))):
    body_tree = m.body_tree[i]
    wp.launch(_crb_accumulate, dim=(d.nworld, body_tree.size), inputs=[m.body_parentid, d.crb, body_tree], outputs=[d.crb])

  d.qM.zero_()
  if m.opt.is_sparse:
    wp.launch(
      _qM_sparse,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_bodyid, m.dof_parentid, m.dof_Madr, m.dof_armature, d.cdof, d.crb],
      outputs=[d.qM],
    )
  else:
    wp.launch(
      _qM_dense, dim=(d.nworld, m.nv), inputs=[m.dof_bodyid, m.dof_parentid, m.dof_armature, d.cdof, d.crb], outputs=[d.qM]
    )


@wp.kernel
def _tendon_armature(
  # Model:
  opt_is_sparse: bool,
  dof_parentid: wp.array(dtype=int),
  dof_Madr: wp.array(dtype=int),
  tendon_armature: wp.array2d(dtype=float),
  # Data in:
  ten_J_in: wp.array3d(dtype=float),
  # Data out:
  qM_out: wp.array3d(dtype=float),
):
  worldid, tenid, dofid = wp.tid()

  if opt_is_sparse:  # opt_is_sparse is not batched
    madr_ij = dof_Madr[dofid]

  armature = tendon_armature[worldid, tenid]

  if armature == 0.0:
    return

  ten_Ji = ten_J_in[worldid, tenid, dofid]

  if ten_Ji == 0.0:
    return

  # sparse backward pass over ancestors
  dofidi = dofid
  while dofid >= 0:
    if dofid != dofidi:
      ten_Jj = ten_J_in[worldid, tenid, dofid]
    else:
      ten_Jj = ten_Ji

    qMij = armature * ten_Jj * ten_Ji

    if opt_is_sparse:
      wp.atomic_add(qM_out[worldid, 0], madr_ij, qMij)
      madr_ij += 1
    else:
      wp.atomic_add(qM_out[worldid, dofidi], dofid, qMij)
      if dofidi != dofid:
        wp.atomic_add(qM_out[worldid, dofid], dofidi, qMij)

    dofid = dof_parentid[dofid]


@event_scope
def tendon_armature(m: Model, d: Data):
  """Add tendon armature to qM."""
  wp.launch(
    _tendon_armature,
    dim=(d.nworld, m.ntendon, m.nv),
    inputs=[m.opt.is_sparse, m.dof_parentid, m.dof_Madr, m.tendon_armature, d.ten_J],
    outputs=[d.qM],
  )


@wp.kernel
def _copy_CSR(
  # Model:
  mapM2M: wp.array(dtype=int),
  # In:
  M_in: wp.array3d(dtype=float),
  # Out:
  L_out: wp.array3d(dtype=float),
):
  worldid, ind = wp.tid()
  L_out[worldid, 0, ind] = M_in[worldid, 0, mapM2M[ind]]


@wp.kernel
def _qLD_acc(
  # Model:
  M_rownnz: wp.array(dtype=int),
  M_rowadr: wp.array(dtype=int),
  # In:
  qLD_updates_: wp.array(dtype=wp.vec3i),
  L_in: wp.array3d(dtype=float),
  # Out:
  L_out: wp.array3d(dtype=float),
):
  worldid, nodeid = wp.tid()
  update = qLD_updates_[nodeid]
  i, k, Madr_ki = update[0], update[1], update[2]
  Madr_i = M_rowadr[i]  # Address of row being updated
  diag_k = M_rowadr[k] + M_rownnz[k] - 1  # Address of diagonal element of k
  # tmp = M(k,i) / M(k,k)
  tmp = L_out[worldid, 0, Madr_ki] / L_out[worldid, 0, diag_k]
  for j in range(M_rownnz[i]):
    # M(i,j) -= M(k,j) * tmp
    wp.atomic_sub(L_out[worldid, 0], Madr_i + j, L_in[worldid, 0, M_rowadr[k] + j] * tmp)
  # M(k,i) = tmp
  L_out[worldid, 0, Madr_ki] = tmp


@wp.kernel
def _qLDiag_div(
  # Model:
  M_rownnz: wp.array(dtype=int),
  M_rowadr: wp.array(dtype=int),
  # In:
  L_in: wp.array3d(dtype=float),
  # Out:
  D_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  diag_i = M_rowadr[dofid] + M_rownnz[dofid] - 1  # Address of diagonal element of i
  D_out[worldid, dofid] = 1.0 / L_in[worldid, 0, diag_i]


def _factor_i_sparse(m: Model, d: Data, M: wp.array3d(dtype=float), L: wp.array3d(dtype=float), D: wp.array2d(dtype=float)):
  """Sparse L'*D*L factorization of inertia-like matrix M, assumed spd."""
  wp.launch(_copy_CSR, dim=(d.nworld, m.nC), inputs=[m.mapM2M, M], outputs=[L])

  for i in reversed(range(len(m.qLD_updates))):
    qLD_updates = m.qLD_updates[i]
    wp.launch(_qLD_acc, dim=(d.nworld, qLD_updates.size), inputs=[m.M_rownnz, m.M_rowadr, qLD_updates, L], outputs=[L])

  wp.launch(_qLDiag_div, dim=(d.nworld, m.nv), inputs=[m.M_rownnz, m.M_rowadr, L], outputs=[D])


@cache_kernel
def _tile_cholesky_factorize(tile: TileSet):
  """Returns a kernel for dense Cholesky factorization of a tile."""

  @nested_kernel(module="unique", enable_backward=False)
  def cholesky_factorize(
    # Data In:
    qM_in: wp.array3d(dtype=float),
    # In:
    adr: wp.array(dtype=int),
    # Out:
    L_out: wp.array3d(dtype=float),
  ):
    worldid, nodeid = wp.tid()
    TILE_SIZE = wp.static(tile.size)

    dofid = adr[nodeid]
    M_tile = wp.tile_load(qM_in[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(dofid, dofid))
    L_tile = wp.tile_cholesky(M_tile)
    wp.tile_store(L_out[worldid], L_tile, offset=(dofid, dofid))

  return cholesky_factorize


def _factor_i_dense(m: Model, d: Data, M: wp.array, L: wp.array):
  """Dense Cholesky factorization of inertia-like matrix M, assumed spd."""
  for tile in m.qM_tiles:
    wp.launch_tiled(
      _tile_cholesky_factorize(tile),
      dim=(d.nworld, tile.adr.size),
      inputs=[M, tile.adr],
      outputs=[L],
      block_dim=m.block_dim.cholesky_factorize,
    )


@event_scope
def factor_m(m: Model, d: Data):
  """Factorization of inertia-like matrix M, assumed spd."""
  if m.opt.is_sparse:
    _factor_i_sparse(m, d, d.qM, d.qLD, d.qLDiagInv)
  else:
    _factor_i_dense(m, d, d.qM, d.qLD)


@wp.kernel
def _cacc_world(
  # In:
  gravity: wp.array(dtype=wp.vec3),
  # Data out:
  cacc_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid = wp.tid()
  cacc_out[worldid, 0] = wp.spatial_vector(wp.vec3(0.0), -gravity[worldid % gravity.shape[0]])


def _rne_cacc_world(m: Model, d: Data):
  if m.opt.disableflags & DisableBit.GRAVITY:
    d.cacc.zero_()
  else:
    wp.launch(_cacc_world, dim=[d.nworld], inputs=[m.opt.gravity], outputs=[d.cacc])


@wp.kernel
def _cacc(
  # Model:
  body_parentid: wp.array(dtype=int),
  body_dofnum: wp.array(dtype=int),
  body_dofadr: wp.array(dtype=int),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  qacc_in: wp.array2d(dtype=float),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  cdof_dot_in: wp.array2d(dtype=wp.spatial_vector),
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  body_tree_: wp.array(dtype=int),
  flg_acc: bool,
  # Data out:
  cacc_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  dofnum = body_dofnum[bodyid]
  pid = body_parentid[bodyid]
  dofadr = body_dofadr[bodyid]
  local_cacc = cacc_in[worldid, pid]
  for i in range(dofnum):
    local_cacc += cdof_dot_in[worldid, dofadr + i] * qvel_in[worldid, dofadr + i]
    if flg_acc:
      local_cacc += cdof_in[worldid, dofadr + i] * qacc_in[worldid, dofadr + i]
  cacc_out[worldid, bodyid] = local_cacc


def _rne_cacc_forward(m: Model, d: Data, flg_acc: bool = False):
  for body_tree in m.body_tree:
    wp.launch(
      _cacc,
      dim=(d.nworld, body_tree.size),
      inputs=[m.body_parentid, m.body_dofnum, m.body_dofadr, d.qvel, d.qacc, d.cdof, d.cdof_dot, d.cacc, body_tree, flg_acc],
      outputs=[d.cacc],
    )


@wp.kernel
def _cfrc(
  # Data in:
  cinert_in: wp.array2d(dtype=vec10),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  cacc_in: wp.array2d(dtype=wp.spatial_vector),
  cfrc_ext_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  flg_cfrc_ext: bool,
  # Data out:
  cfrc_int_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, bodyid = wp.tid()
  bodyid += 1  # skip world body
  cacc = cacc_in[worldid, bodyid]
  cinert = cinert_in[worldid, bodyid]
  cvel = cvel_in[worldid, bodyid]
  frc = math.inert_vec(cinert, cacc)
  frc += math.motion_cross_force(cvel, math.inert_vec(cinert, cvel))
  if flg_cfrc_ext:
    frc -= cfrc_ext_in[worldid, bodyid]

  cfrc_int_out[worldid, bodyid] = frc


def _rne_cfrc(m: Model, d: Data, flg_cfrc_ext: bool = False):
  wp.launch(
    _cfrc, dim=[d.nworld, m.nbody - 1], inputs=[d.cinert, d.cvel, d.cacc, d.cfrc_ext, flg_cfrc_ext], outputs=[d.cfrc_int]
  )


@wp.kernel
def _cfrc_backward(
  # Model:
  body_parentid: wp.array(dtype=int),
  # Data in:
  cfrc_int_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  cfrc_int_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  pid = body_parentid[bodyid]
  if bodyid != 0:
    wp.atomic_add(cfrc_int_out[worldid], pid, cfrc_int_in[worldid, bodyid])


def _rne_cfrc_backward(m: Model, d: Data):
  for body_tree in reversed(m.body_tree):
    wp.launch(
      _cfrc_backward, dim=[d.nworld, body_tree.size], inputs=[m.body_parentid, d.cfrc_int, body_tree], outputs=[d.cfrc_int]
    )


@wp.kernel
def _qfrc_bias(
  # Model:
  dof_bodyid: wp.array(dtype=int),
  # Data in:
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  cfrc_int_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  qfrc_bias_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  bodyid = dof_bodyid[dofid]
  qfrc_bias_out[worldid, dofid] = wp.dot(cdof_in[worldid, dofid], cfrc_int_in[worldid, bodyid])


@event_scope
def rne(m: Model, d: Data, flg_acc: bool = False):
  """Computes inverse dynamics using the recursive Newton-Euler algorithm.

  Computes the bias forces (`qfrc_bias`) and internal forces (`cfrc_int`) for the current state,
  including the effects of gravity and optionally joint accelerations.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    flg_acc: If True, includes joint accelerations in the computation.
  """
  _rne_cacc_world(m, d)
  _rne_cacc_forward(m, d, flg_acc=flg_acc)
  _rne_cfrc(m, d)
  _rne_cfrc_backward(m, d)
  wp.launch(_qfrc_bias, dim=[d.nworld, m.nv], inputs=[m.dof_bodyid, d.cdof, d.cfrc_int], outputs=[d.qfrc_bias])


@wp.kernel
def _cfrc_ext(
  # Model:
  body_rootid: wp.array(dtype=int),
  # Data in:
  xfrc_applied_in: wp.array2d(dtype=wp.spatial_vector),
  xipos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  cfrc_ext_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, bodyid = wp.tid()
  if bodyid == 0:
    cfrc_ext_out[worldid, 0] = wp.spatial_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  else:
    xfrc_applied = xfrc_applied_in[worldid, bodyid]
    subtree_com = subtree_com_in[worldid, body_rootid[bodyid]]
    xipos = xipos_in[worldid, bodyid]
    cfrc_ext_out[worldid, bodyid] = support.transform_force(xfrc_applied, subtree_com - xipos)


@wp.kernel
def _cfrc_ext_equality(
  # Model:
  body_rootid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  site_pos: wp.array2d(dtype=wp.vec3),
  eq_obj1id: wp.array(dtype=int),
  eq_obj2id: wp.array(dtype=int),
  eq_objtype: wp.array(dtype=int),
  eq_data: wp.array2d(dtype=vec11),
  # Data in:
  xpos_in: wp.array2d(dtype=wp.vec3),
  xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  efc_id_in: wp.array2d(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  ne_connect_in: wp.array(dtype=int),
  ne_weld_in: wp.array(dtype=int),
  # Data out:
  cfrc_ext_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, eqid = wp.tid()

  ne_connect = ne_connect_in[worldid]
  ne_weld = ne_weld_in[worldid]
  num_connect = ne_connect // 3

  if eqid >= num_connect + ne_weld // 6:
    return

  is_connect = eqid < num_connect
  if is_connect:
    efcid = 3 * eqid
    cfrc_torque = wp.vec3(0.0, 0.0, 0.0)  # no torque from connect
  else:
    efcid = 6 * eqid - ne_connect
    cfrc_torque = wp.vec3(efc_force_in[worldid, efcid + 3], efc_force_in[worldid, efcid + 4], efc_force_in[worldid, efcid + 5])

  cfrc_force = wp.vec3(
    efc_force_in[worldid, efcid + 0],
    efc_force_in[worldid, efcid + 1],
    efc_force_in[worldid, efcid + 2],
  )

  id = efc_id_in[worldid, efcid]
  eq_data_ = eq_data[worldid, id]
  body_semantic = eq_objtype[id] == ObjType.BODY

  obj1 = eq_obj1id[id]
  obj2 = eq_obj2id[id]

  if body_semantic:
    bodyid1 = obj1
    bodyid2 = obj2
  else:
    bodyid1 = site_bodyid[obj1]
    bodyid2 = site_bodyid[obj2]

  # body 1
  if bodyid1:
    if body_semantic:
      if is_connect:
        offset = wp.vec3(eq_data_[0], eq_data_[1], eq_data_[2])
      else:
        offset = wp.vec3(eq_data_[3], eq_data_[4], eq_data_[5])
    else:
      offset = site_pos[worldid, obj1]

    # transform point on body1: local -> global
    pos = xmat_in[worldid, bodyid1] @ offset + xpos_in[worldid, bodyid1]

    # subtree CoM-based torque_force vector
    newpos = subtree_com_in[worldid, body_rootid[bodyid1]]

    dif = newpos - pos
    cfrc_com = wp.spatial_vector(cfrc_torque - wp.cross(dif, cfrc_force), cfrc_force)

    # apply (opposite for body 1)
    wp.atomic_add(cfrc_ext_out[worldid], bodyid1, cfrc_com)

  # body 2
  if bodyid2:
    if body_semantic:
      if is_connect:
        offset = wp.vec3(eq_data_[3], eq_data_[4], eq_data_[5])
      else:
        offset = wp.vec3(eq_data_[0], eq_data_[1], eq_data_[2])
    else:
      offset = site_pos[worldid, obj2]

    # transform point on body2: local -> global
    pos = xmat_in[worldid, bodyid2] @ offset + xpos_in[worldid, bodyid2]

    # subtree CoM-based torque_force vector
    newpos = subtree_com_in[worldid, body_rootid[bodyid2]]

    dif = newpos - pos
    cfrc_com = wp.spatial_vector(cfrc_torque - wp.cross(dif, cfrc_force), cfrc_force)

    # apply
    wp.atomic_sub(cfrc_ext_out[worldid], bodyid2, cfrc_com)


@wp.func
def transform_force(force: wp.vec3, torque: wp.vec3, offset: wp.vec3) -> wp.spatial_vector:
  torque -= wp.cross(offset, force)
  return wp.spatial_vector(torque, force)


@wp.kernel
def _cfrc_ext_contact(
  # Model:
  opt_cone: int,
  body_rootid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  # Data in:
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_friction_in: wp.array(dtype=vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_force_in: wp.array2d(dtype=float),
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  # Data out:
  cfrc_ext_out: wp.array2d(dtype=wp.spatial_vector),
):
  contactid = wp.tid()

  if contactid >= nacon_in[0]:
    return

  geom = contact_geom_in[contactid]
  id1 = geom_bodyid[geom[0]]
  id2 = geom_bodyid[geom[1]]

  if id1 == 0 and id2 == 0:
    return

  worldid = contact_worldid_in[contactid]

  # contact force in world frame
  force = support.contact_force_fn(
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
    to_world_frame=True,
  )

  pos = contact_pos_in[contactid]

  # contact force on bodies
  if id1:
    com1 = subtree_com_in[worldid, body_rootid[id1]]
    wp.atomic_sub(cfrc_ext_out[worldid], id1, support.transform_force(force, com1 - pos))

  if id2:
    com2 = subtree_com_in[worldid, body_rootid[id2]]
    wp.atomic_add(cfrc_ext_out[worldid], id2, support.transform_force(force, com2 - pos))


@event_scope
def rne_postconstraint(m: Model, d: Data):
  """Computes the recursive Newton-Euler algorithm after constraints are applied.

  Computes `cacc`, `cfrc_ext`, and `cfrc_int`, including the effects of applied forces, equality
  constraints, and contacts.
  """
  # cfrc_ext = perturb
  wp.launch(
    _cfrc_ext,
    dim=(d.nworld, m.nbody),
    inputs=[m.body_rootid, d.xfrc_applied, d.xipos, d.subtree_com],
    outputs=[d.cfrc_ext],
  )

  wp.launch(
    _cfrc_ext_equality,
    dim=(d.nworld, m.neq),
    inputs=[
      m.body_rootid,
      m.site_bodyid,
      m.site_pos,
      m.eq_obj1id,
      m.eq_obj2id,
      m.eq_objtype,
      m.eq_data,
      d.xpos,
      d.xmat,
      d.subtree_com,
      d.efc.id,
      d.efc.force,
      d.ne_connect,
      d.ne_weld,
    ],
    outputs=[d.cfrc_ext],
  )

  # cfrc_ext += contacts
  wp.launch(
    _cfrc_ext_contact,
    dim=(d.naconmax,),
    inputs=[
      m.opt.cone,
      m.body_rootid,
      m.geom_bodyid,
      d.subtree_com,
      d.contact.pos,
      d.contact.frame,
      d.contact.friction,
      d.contact.dim,
      d.contact.geom,
      d.contact.efc_address,
      d.contact.worldid,
      d.efc.force,
      d.njmax,
      d.nacon,
    ],
    outputs=[d.cfrc_ext],
  )

  # forward pass over bodies: compute cacc, cfrc_int
  _rne_cacc_world(m, d)
  _rne_cacc_forward(m, d, flg_acc=True)

  # cfrc_body = cinert * cacc + cvel x (cinert * cvel)
  _rne_cfrc(m, d, flg_cfrc_ext=True)

  # backward pass over bodies: accumulate cfrc_int from children
  _rne_cfrc_backward(m, d)


@wp.kernel
def _tendon_dot(
  # Model:
  nv: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  jnt_type: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  dof_jntid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  tendon_adr: wp.array(dtype=int),
  tendon_num: wp.array(dtype=int),
  tendon_armature: wp.array2d(dtype=float),
  wrap_type: wp.array(dtype=int),
  wrap_objid: wp.array(dtype=int),
  wrap_prm: wp.array(dtype=float),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  cdof_dot_in: wp.array2d(dtype=wp.spatial_vector),
  # Out:
  ten_Jdot_out: wp.array3d(dtype=float),
):
  worldid, tenid = wp.tid()

  armature = tendon_armature[worldid, tenid]
  if armature == 0.0:
    return

  # fixed tendon has zero Jdot
  adr = tendon_adr[tenid]
  if wrap_type[adr] == WrapType.JOINT:
    return

  # process spatial tendon
  divisor = float(1.0)
  num = tendon_num[tenid]
  j = int(0)
  while j < num - 1:
    # get 1st and 2nd object
    type0 = wrap_type[adr + j + 0]
    type1 = wrap_type[adr + j + 1]
    id0 = wrap_objid[adr + j + 0]
    id1 = wrap_objid[adr + j + 1]

    # pulley
    pulley = WrapType.PULLEY
    if (type0 == pulley) or (type1 == pulley):
      # get divisor
      if type0 == pulley:
        divisor = wrap_prm[adr + j]

      j += 1
      continue

    # init sequence; assume it start with site
    wpnt0 = site_xpos_in[worldid, id0]

    bodyid0 = site_bodyid[id0]
    pos0 = site_xpos_in[worldid, id0]
    cvel0 = cvel_in[worldid, bodyid0]
    subtree_com0 = subtree_com_in[worldid, body_rootid[bodyid0]]
    dif0 = pos0 - subtree_com0
    wvel0 = wp.spatial_bottom(cvel0) - wp.cross(dif0, wp.spatial_top(cvel0))
    wbody0 = site_bodyid[id0]

    # second object is geom: process site-geom-site
    if (type1 == WrapType.SPHERE) or (type1 == WrapType.CYLINDER):
      # TODO(team): derivatives of util_misc.wrap
      return

    # complete sequence
    wbody1 = site_bodyid[id1]
    wpnt1 = site_xpos_in[worldid, id1]

    bodyid1 = site_bodyid[id1]
    pos1 = site_xpos_in[worldid, id1]
    cvel1 = cvel_in[worldid, bodyid1]
    subtree_com1 = subtree_com_in[worldid, body_rootid[bodyid1]]
    dif1 = pos1 - subtree_com1
    wvel1 = wp.spatial_bottom(cvel1) - wp.cross(dif1, wp.spatial_top(cvel1))

    # accumulate moments if consecutive points are in different bodies
    if wbody0 != wbody1:
      # dpnt = 3D position difference, normalize
      dpnt, norm = math.normalize_with_norm(wpnt1 - wpnt0)

      # dvel = d / dt (dpnt)
      dvel = wvel1 - wvel0
      dot = wp.dot(dpnt, dvel)
      dvel += dpnt * (-dot)
      if norm > MJ_MINVAL:
        dvel /= norm
      else:
        dvel = wp.vec3(0.0)

      # get endpoint Jacobian time derivatives, subtract
      # TODO(team): parallelize?
      for i in range(nv):
        jac1, _ = support.jac_dot(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          wpnt0,
          wbody0,
          i,
          worldid,
        )
        jac2, _ = support.jac_dot(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          wpnt1,
          wbody1,
          i,
          worldid,
        )
        jacdif = jac2 - jac1

        # chain rule, first term: Jdot += d / dt (jac2 - jac1) * dpnt
        Jdot = wp.dot(jacdif, dpnt)

        # get endpoint Jacobians, subtract
        jac1, _ = support.jac(
          body_parentid,
          body_rootid,
          dof_bodyid,
          subtree_com_in,
          cdof_in,
          wpnt0,
          wbody0,
          i,
          worldid,
        )
        jac2, _ = support.jac(
          body_parentid,
          body_rootid,
          dof_bodyid,
          subtree_com_in,
          cdof_in,
          wpnt1,
          wbody1,
          i,
          worldid,
        )
        jacdif = jac2 - jac1

        # chain rule, second term: Jdot += (jac2 - jac1) * d / dt (dpnt)
        Jdot += wp.dot(jacdif, dvel)

        ten_Jdot_out[worldid, tenid, i] += math.safe_div(Jdot, divisor)

    # TODO(team): j += 2 if geom wrapping
    j += 1


@wp.kernel
def _tendon_bias_coef(
  # Model:
  tendon_armature: wp.array2d(dtype=float),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  # In:
  ten_Jdot_in: wp.array3d(dtype=float),
  # Out:
  ten_bias_coef_out: wp.array2d(dtype=float),
):
  worldid, tenid, dofid = wp.tid()

  armature = tendon_armature[worldid, tenid]
  if armature == 0.0:
    return

  ten_Jdot = ten_Jdot_in[worldid, tenid, dofid]
  if ten_Jdot == 0.0:
    return

  wp.atomic_add(ten_bias_coef_out[worldid], tenid, ten_Jdot * qvel_in[worldid, dofid])


@wp.kernel
def _tendon_bias_qfrc(
  # Model:
  tendon_armature: wp.array2d(dtype=float),
  # Data in:
  ten_J_in: wp.array3d(dtype=float),
  # In:
  ten_bias_coef_in: wp.array2d(dtype=float),
  # Out:
  qfrc_out: wp.array2d(dtype=float),
):
  worldid, tenid, dofid = wp.tid()

  armature = tendon_armature[worldid, tenid]
  if armature == 0.0:
    return

  ten_J = ten_J_in[worldid, tenid, dofid]
  if ten_J == 0.0:
    return

  wp.atomic_add(qfrc_out[worldid], dofid, ten_J * armature * ten_bias_coef_in[worldid, tenid])


@event_scope
def tendon_bias(m: Model, d: Data, qfrc: wp.array2d(dtype=float)):
  """Add bias force due to tendon armature.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    qfrc: Force.
  """
  # time derivative of tendon Jacobian
  ten_Jdot = wp.zeros((d.nworld, m.ntendon, m.nv), dtype=float)
  wp.launch(
    _tendon_dot,
    dim=(d.nworld, m.ntendon),
    inputs=[
      m.nv,
      m.body_parentid,
      m.body_rootid,
      m.jnt_type,
      m.jnt_dofadr,
      m.dof_bodyid,
      m.dof_jntid,
      m.site_bodyid,
      m.tendon_adr,
      m.tendon_num,
      m.tendon_armature,
      m.wrap_type,
      m.wrap_objid,
      m.wrap_prm,
      d.site_xpos,
      d.subtree_com,
      d.cdof,
      d.cvel,
      d.cdof_dot,
    ],
    outputs=[ten_Jdot],
  )

  # tendon bias force coefficients
  ten_bias_coef = wp.zeros((d.nworld, m.ntendon), dtype=float)
  wp.launch(
    _tendon_bias_coef,
    dim=(d.nworld, m.ntendon, m.nv),
    inputs=[m.tendon_armature, d.qvel, ten_Jdot],
    outputs=[ten_bias_coef],
  )

  wp.launch(
    _tendon_bias_qfrc,
    dim=(d.nworld, m.ntendon, m.nv),
    inputs=[m.tendon_armature, d.ten_J, ten_bias_coef],
    outputs=[qfrc],
  )


@wp.kernel
def _comvel_root(cvel_out: wp.array2d(dtype=wp.spatial_vector)):
  worldid, elementid = wp.tid()
  cvel_out[worldid, 0][elementid] = 0.0


@wp.kernel
def _comvel_level(
  # Model:
  body_parentid: wp.array(dtype=int),
  body_jntnum: wp.array(dtype=int),
  body_jntadr: wp.array(dtype=int),
  body_dofadr: wp.array(dtype=int),
  jnt_type: wp.array(dtype=int),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  cvel_out: wp.array2d(dtype=wp.spatial_vector),
  cdof_dot_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  dofid = body_dofadr[bodyid]
  jntid = body_jntadr[bodyid]
  jntnum = body_jntnum[bodyid]
  pid = body_parentid[bodyid]

  if jntnum == 0:
    cvel_out[worldid, bodyid] = cvel_in[worldid, pid]
    return

  cvel = cvel_in[worldid, pid]
  qvel = qvel_in[worldid]
  cdof = cdof_in[worldid]

  for j in range(jntid, jntid + jntnum):
    jnttype = jnt_type[j]

    if jnttype == JointType.FREE:
      cvel += cdof[dofid + 0] * qvel[dofid + 0]
      cvel += cdof[dofid + 1] * qvel[dofid + 1]
      cvel += cdof[dofid + 2] * qvel[dofid + 2]

      cdof_dot_out[worldid, dofid + 3] = math.motion_cross(cvel, cdof[dofid + 3])
      cdof_dot_out[worldid, dofid + 4] = math.motion_cross(cvel, cdof[dofid + 4])
      cdof_dot_out[worldid, dofid + 5] = math.motion_cross(cvel, cdof[dofid + 5])

      cvel += cdof[dofid + 3] * qvel[dofid + 3]
      cvel += cdof[dofid + 4] * qvel[dofid + 4]
      cvel += cdof[dofid + 5] * qvel[dofid + 5]

      dofid += 6
    elif jnttype == JointType.BALL:
      cdof_dot_out[worldid, dofid + 0] = math.motion_cross(cvel, cdof[dofid + 0])
      cdof_dot_out[worldid, dofid + 1] = math.motion_cross(cvel, cdof[dofid + 1])
      cdof_dot_out[worldid, dofid + 2] = math.motion_cross(cvel, cdof[dofid + 2])

      cvel += cdof[dofid + 0] * qvel[dofid + 0]
      cvel += cdof[dofid + 1] * qvel[dofid + 1]
      cvel += cdof[dofid + 2] * qvel[dofid + 2]

      dofid += 3
    else:
      cdof_dot_out[worldid, dofid] = math.motion_cross(cvel, cdof[dofid])
      cvel += cdof[dofid] * qvel[dofid]

      dofid += 1

  cvel_out[worldid, bodyid] = cvel


@event_scope
def com_vel(m: Model, d: Data):
  """Computes the spatial velocities (cvel) and the derivative `cdof_dot` for all bodies.

  Propagates velocities down the kinematic tree, updating the spatial velocity and
  derivative for each body.
  """
  wp.launch(_comvel_root, dim=(d.nworld, 6), inputs=[], outputs=[d.cvel])

  for body_tree in m.body_tree:
    wp.launch(
      _comvel_level,
      dim=(d.nworld, body_tree.size),
      inputs=[m.body_parentid, m.body_jntnum, m.body_jntadr, m.body_dofadr, m.jnt_type, d.qvel, d.cdof, d.cvel, body_tree],
      outputs=[d.cvel, d.cdof_dot],
    )


@wp.kernel
def _transmission(
  # Model:
  nv: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_weldid: wp.array(dtype=int),
  body_dofnum: wp.array(dtype=int),
  body_dofadr: wp.array(dtype=int),
  jnt_type: wp.array(dtype=int),
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  dof_parentid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  site_quat: wp.array2d(dtype=wp.quat),
  tendon_adr: wp.array(dtype=int),
  tendon_num: wp.array(dtype=int),
  wrap_type: wp.array(dtype=int),
  wrap_objid: wp.array(dtype=int),
  actuator_trntype: wp.array(dtype=int),
  actuator_trnid: wp.array(dtype=wp.vec2i),
  actuator_gear: wp.array2d(dtype=wp.spatial_vector),
  actuator_cranklength: wp.array(dtype=float),
  # Data in:
  qpos_in: wp.array2d(dtype=float),
  xquat_in: wp.array2d(dtype=wp.quat),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  site_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  ten_J_in: wp.array3d(dtype=float),
  ten_length_in: wp.array2d(dtype=float),
  # Data out:
  actuator_length_out: wp.array2d(dtype=float),
  actuator_moment_out: wp.array3d(dtype=float),
):
  worldid, actid = wp.tid()
  trntype = actuator_trntype[actid]
  actuator_gear_id = worldid % actuator_gear.shape[0]
  gear = actuator_gear[actuator_gear_id, actid]
  if trntype == TrnType.JOINT or trntype == TrnType.JOINTINPARENT:
    qpos = qpos_in[worldid]
    jntid = actuator_trnid[actid][0]
    jnt_typ = jnt_type[jntid]
    qadr = jnt_qposadr[jntid]
    vadr = jnt_dofadr[jntid]
    if jnt_typ == JointType.FREE:
      actuator_length_out[worldid, actid] = 0.0
      if trntype == TrnType.JOINTINPARENT:
        quat = wp.normalize(wp.quat(qpos[qadr + 3], qpos[qadr + 4], qpos[qadr + 5], qpos[qadr + 6]))
        quat_neg = math.quat_inv(quat)
        gearaxis = math.rot_vec_quat(wp.spatial_bottom(gear), quat_neg)
        actuator_moment_out[worldid, actid, vadr + 0] = gear[0]
        actuator_moment_out[worldid, actid, vadr + 1] = gear[1]
        actuator_moment_out[worldid, actid, vadr + 2] = gear[2]
        actuator_moment_out[worldid, actid, vadr + 3] = gearaxis[0]
        actuator_moment_out[worldid, actid, vadr + 4] = gearaxis[1]
        actuator_moment_out[worldid, actid, vadr + 5] = gearaxis[2]
      else:
        for i in range(6):
          actuator_moment_out[worldid, actid, vadr + i] = gear[i]
    elif jnt_typ == JointType.BALL:
      q = wp.quat(qpos[qadr + 0], qpos[qadr + 1], qpos[qadr + 2], qpos[qadr + 3])
      q = wp.normalize(q)
      axis_angle = math.quat_to_vel(q)
      gearaxis = wp.spatial_top(gear)  # [:3]
      if trntype == TrnType.JOINTINPARENT:
        quat_neg = math.quat_inv(q)
        gearaxis = math.rot_vec_quat(gearaxis, quat_neg)
      actuator_length_out[worldid, actid] = wp.dot(axis_angle, gearaxis)
      for i in range(3):
        actuator_moment_out[worldid, actid, vadr + i] = gearaxis[i]
    elif jnt_typ == JointType.SLIDE or jnt_typ == JointType.HINGE:
      actuator_length_out[worldid, actid] = qpos[qadr] * gear[0]
      actuator_moment_out[worldid, actid, vadr] = gear[0]
    else:
      wp.printf("unrecognized joint type")
  elif trntype == TrnType.SLIDERCRANK:
    # get data
    trnid = actuator_trnid[actid]
    id = trnid[0]
    idslider = trnid[1]
    gear0 = gear[0]
    rod = actuator_cranklength[actid]
    site_xmat = site_xmat_in[worldid, idslider]
    axis = wp.vec3(site_xmat[0, 2], site_xmat[1, 2], site_xmat[2, 2])
    site_xpos_id = site_xpos_in[worldid, id]
    site_xpos_idslider = site_xpos_in[worldid, idslider]
    vec = site_xpos_id - site_xpos_idslider

    # compute length and determinant
    #  length = a' * v - sqrt(det);  det = (a' * v)^2 + r^2 - v' * v
    av = wp.dot(vec, axis)
    det = av * av + rod * rod - wp.dot(vec, vec)
    ok = 1
    if det <= 0.0:
      ok = 0
      sdet = 0.0
      length = av
    else:
      sdet = wp.sqrt(det)
      length = av - sdet

    actuator_length_out[worldid, actid] = length * gear0

    # compute derivatives of length w.r.t. vec and axis
    if ok == 1:
      scale = 1.0 - math.safe_div(av, sdet)
      dldv = axis * scale + math.safe_div(vec, sdet)
      dlda = vec * scale
    else:
      dldv = axis
      dlda = vec

    # apply chain rule
    # TODO(team): parallelize?
    for i in range(nv):
      # get Jacobians of axis(jacA) and vec(jac)
      # mj_jacPointAxis
      jacp, jacr = support.jac(
        body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_xpos_idslider, site_bodyid[idslider], i, worldid
      )
      jacS = jacp
      jacA = wp.cross(jacr, axis)

      # mj_jacSite
      jac, _ = support.jac(
        body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_xpos_id, site_bodyid[id], i, worldid
      )
      jac -= jacS

      # apply the chain rule
      moment = wp.dot(dlda, jacA) + wp.dot(dldv, jac)
      actuator_moment_out[worldid, actid, i] = moment * gear0
  elif trntype == TrnType.TENDON:
    tenid = actuator_trnid[actid][0]

    gear0 = gear[0]
    actuator_length_out[worldid, actid] = ten_length_in[worldid, tenid] * gear0

    # fixed
    adr = tendon_adr[tenid]
    if wrap_type[adr] == WrapType.JOINT:
      ten_num = tendon_num[tenid]
      for i in range(ten_num):
        dofadr = jnt_dofadr[wrap_objid[adr + i]]
        actuator_moment_out[worldid, actid, dofadr] = ten_J_in[worldid, tenid, dofadr] * gear0
    else:  # spatial
      for dofadr in range(nv):
        actuator_moment_out[worldid, actid, dofadr] = ten_J_in[worldid, tenid, dofadr] * gear0
  elif trntype == TrnType.BODY:
    # cannot compute meaningful length, set to zero
    actuator_length_out[worldid, actid] = 0.0

    # initialize moment
    for i in range(nv):
      actuator_moment_out[worldid, actid, i] = 0.0

    # moment computed by _transmission_body_moment and _transmission_body_moment_scale
  elif trntype == TrnType.SITE:
    trnid = actuator_trnid[actid]
    siteid = trnid[0]
    refid = trnid[1]

    gear = actuator_gear[worldid, actid]
    site_quat_id = worldid % site_quat.shape[0]
    gear_translation = wp.spatial_top(gear)
    gear_rotational = wp.spatial_bottom(gear)

    # reference site undefined
    if refid == -1:
      # wrench: gear expressed in global frame
      site_xmat = site_xmat_in[worldid, siteid]
      wrench_translation = site_xmat @ gear_translation
      wrench_rotation = site_xmat @ gear_rotational

      # moment: global Jacobian projected on wrench
      # TODO(team): parallelize
      for i in range(nv):
        jacp, jacr = support.jac(
          body_parentid,
          body_rootid,
          dof_bodyid,
          subtree_com_in,
          cdof_in,
          site_xpos_in[worldid, siteid],
          site_bodyid[siteid],
          i,
          worldid,
        )
        actuator_length_out[worldid, actid] = 0.0
        actuator_moment_out[worldid, actid, i] = wp.dot(jacp, wrench_translation) + wp.dot(jacr, wrench_rotation)
    # reference site defined
    else:
      # initialize last dof address for each body
      bodyid = site_bodyid[siteid]
      bodyrefid = site_bodyid[refid]
      b0 = body_weldid[bodyid]
      b1 = body_weldid[bodyrefid]
      dofadr0 = body_dofadr[b0] + body_dofnum[b0] - 1
      dofadr1 = body_dofadr[b1] + body_dofnum[b1] - 1

      # find common ancestral dof, if any
      dofadr_common = -1
      if dofadr0 >= 0 and dofadr1 >= 0:
        # traverse up the tree until common ancestral dof is found
        while dofadr0 != dofadr1:
          if dofadr0 < dofadr1:
            dofadr1 = dof_parentid[dofadr1]
          else:
            dofadr0 = dof_parentid[dofadr0]

          if dofadr0 == -1 or dofadr1 == -1:
            # reached tree root, no common ancestral dof
            break

        # found common ancestral dof
        if dofadr0 == dofadr1:
          dofadr_common = dofadr0

      translational_transmission = not (gear[0] == 0.0 and gear[1] == 0.0 and gear[2] == 0.0)
      rotational_transmission = not (gear[3] == 0.0 and gear[4] == 0.0 and gear[5] == 0.0)

      site_xpos = site_xpos_in[worldid, siteid]
      ref_xpos = site_xpos_in[worldid, refid]
      ref_xmat = site_xmat_in[worldid, refid]

      length = float(0.0)

      if translational_transmission:
        # vec: site position in reference site frame
        vec = wp.transpose(ref_xmat) @ (site_xpos - ref_xpos)
        length += wp.dot(vec, gear_translation)

        wrench_translation = ref_xmat @ gear_translation

      if rotational_transmission:
        # get site and refsite quats from parent bodies (avoid converting matrix to quat)
        quat = math.mul_quat(site_quat[site_quat_id, siteid], xquat_in[worldid, bodyid])
        refquat = math.mul_quat(site_quat[site_quat_id, refid], xquat_in[worldid, bodyrefid])

        # convert difference to expmap (axis-angle)
        vec = math.quat_sub(quat, refquat)
        length += wp.dot(vec, gear_rotational)

        wrench_rotation = ref_xmat @ gear_rotational

      actuator_length_out[worldid, actid] = length

      # TODO(team): parallelize
      for i in range(nv):
        jacp, jacr = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_xpos, site_bodyid[siteid], i, worldid
        )

        # jacref: global Jacobian of reference site
        jacpref, jacrref = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, ref_xpos, site_bodyid[refid], i, worldid
        )

        jacpdif = jacp - jacpref
        jacrdif = jacr - jacrref

        # if common ancestral dof was found, clear the columns of its parental chain
        da = dofadr_common
        while da >= 0:
          if da == i:
            jacpdif = wp.vec3(0.0)
            jacrdif = wp.vec3(0.0)
            break
          da = dof_parentid[da]

        # moment: global Jacobian projected on wrench
        moment = float(0.0)

        if translational_transmission:
          moment += wp.dot(jacpdif, wrench_translation)
        if rotational_transmission:
          moment += wp.dot(jacrdif, wrench_rotation)

        actuator_moment_out[worldid, actid, i] = moment
  else:
    wp.printf("unhandled transmission type %d\n", trntype)


@wp.kernel
def _transmission_body_moment(
  # Model:
  opt_cone: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  actuator_trnid: wp.array(dtype=wp.vec2i),
  actuator_trntype_body_adr: wp.array(dtype=int),
  # Data in:
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  contact_dist_in: wp.array(dtype=float),
  contact_pos_in: wp.array(dtype=wp.vec3),
  contact_frame_in: wp.array(dtype=wp.mat33),
  contact_includemargin_in: wp.array(dtype=float),
  contact_dim_in: wp.array(dtype=int),
  contact_geom_in: wp.array(dtype=wp.vec2i),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  nacon_in: wp.array(dtype=int),
  # Data out:
  actuator_moment_out: wp.array3d(dtype=float),
  # Out:
  actuator_trntype_body_ncon_out: wp.array2d(dtype=int),
):
  trnbodyid, conid, dofid = wp.tid()
  actid = actuator_trntype_body_adr[trnbodyid]
  bodyid = actuator_trnid[actid][0]

  if conid >= nacon_in[0]:
    return

  worldid = contact_worldid_in[conid]

  # get geom ids
  geom = contact_geom_in[conid]
  g1 = geom[0]
  g2 = geom[1]

  # contact involving flex, continue
  if g1 < 0 or g2 < 0:
    return

  # get body ids
  b1 = geom_bodyid[g1]
  b2 = geom_bodyid[g2]

  # irrelevant contact, continue
  if b1 != bodyid and b2 != bodyid:
    return

  contact_exclude = int(contact_dist_in[conid] >= contact_includemargin_in[conid])

  if dofid == 0:
    wp.atomic_add(actuator_trntype_body_ncon_out[worldid], trnbodyid, 1)

  # mark contact normals in efc_force
  if contact_exclude == 0:
    contact_dim = contact_dim_in[conid]
    contact_efc_address = contact_efc_address_in[conid]

    if contact_dim == 1 or opt_cone == ConeType.ELLIPTIC:
      efc_force = 1.0
      efcid0 = contact_efc_address[0]
      wp.atomic_add(actuator_moment_out[worldid, actid], dofid, efc_J_in[worldid, efcid0, dofid] * efc_force)

    else:
      npyramid = contact_dim - 1  # number of frictional directions
      efc_force = 0.5 / float(npyramid)

      for j in range(2 * npyramid):
        efcid = contact_efc_address[j]
        wp.atomic_add(actuator_moment_out[worldid, actid], dofid, efc_J_in[worldid, efcid, dofid] * efc_force)

  # excluded contact in gap: get Jacobian, accumulate
  elif contact_exclude == 1:
    contact_pos = contact_pos_in[conid]
    contact_frame = contact_frame_in[conid]
    normal = wp.vec3(contact_frame[0, 0], contact_frame[0, 1], contact_frame[0, 2])

    # get Jacobian difference
    jacp1, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, contact_pos, b1, dofid, worldid)
    jacp2, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, contact_pos, b2, dofid, worldid)
    jacdif = jacp2 - jacp1

    # project Jacobian along the normal of the contact frame
    wp.atomic_add(actuator_moment_out[worldid, actid], dofid, wp.dot(normal, jacdif))


@wp.kernel
def _transmission_body_moment_scale(
  # Model:
  actuator_trntype_body_adr: wp.array(dtype=int),
  # In:
  actuator_trntype_body_ncon_in: wp.array2d(dtype=int),
  # Data out:
  actuator_moment_out: wp.array3d(dtype=float),
):
  worldid, trnbodyid, dofid = wp.tid()

  ncon = actuator_trntype_body_ncon_in[worldid, trnbodyid]

  if ncon > 0:
    actid = actuator_trntype_body_adr[trnbodyid]
    actuator_moment_out[worldid, actid, dofid] /= -float(ncon)


@event_scope
def transmission(m: Model, d: Data):
  """Computes actuator/transmission lengths and moments.

  Updates the actuator length and moments for all actuators in the model, including joint
  and tendon transmissions.
  """
  wp.launch(
    _transmission,
    dim=[d.nworld, m.nu],
    inputs=[
      m.nv,
      m.body_parentid,
      m.body_rootid,
      m.body_weldid,
      m.body_dofnum,
      m.body_dofadr,
      m.jnt_type,
      m.jnt_qposadr,
      m.jnt_dofadr,
      m.dof_bodyid,
      m.dof_parentid,
      m.site_bodyid,
      m.site_quat,
      m.tendon_adr,
      m.tendon_num,
      m.wrap_type,
      m.wrap_objid,
      m.actuator_trntype,
      m.actuator_trnid,
      m.actuator_gear,
      m.actuator_cranklength,
      d.qpos,
      d.xquat,
      d.site_xpos,
      d.site_xmat,
      d.subtree_com,
      d.cdof,
      d.ten_J,
      d.ten_length,
    ],
    outputs=[d.actuator_length, d.actuator_moment],
  )

  if m.nacttrnbody:
    # compute moments
    ncon = wp.zeros((d.nworld, m.nacttrnbody), dtype=int)
    wp.launch(
      _transmission_body_moment,
      dim=(m.nacttrnbody, d.naconmax, m.nv),
      inputs=[
        m.opt.cone,
        m.body_parentid,
        m.body_rootid,
        m.dof_bodyid,
        m.geom_bodyid,
        m.actuator_trnid,
        m.actuator_trntype_body_adr,
        d.subtree_com,
        d.cdof,
        d.contact.dist,
        d.contact.pos,
        d.contact.frame,
        d.contact.includemargin,
        d.contact.dim,
        d.contact.geom,
        d.contact.efc_address,
        d.contact.worldid,
        d.efc.J,
        d.nacon,
      ],
      outputs=[d.actuator_moment, ncon],
    )

    # scale moments
    wp.launch(
      _transmission_body_moment_scale,
      dim=(d.nworld, m.nacttrnbody, m.nv),
      inputs=[m.actuator_trntype_body_adr, ncon],
      outputs=[d.actuator_moment],
    )


@wp.kernel
def _solve_LD_sparse_x_acc_up(
  # In:
  L: wp.array3d(dtype=float),
  qLD_updates_: wp.array(dtype=wp.vec3i),
  # Out:
  x: wp.array2d(dtype=float),
):
  worldid, nodeid = wp.tid()
  update = qLD_updates_[nodeid]
  i, k, Madr_ki = update[0], update[1], update[2]
  wp.atomic_sub(x[worldid], i, L[worldid, 0, Madr_ki] * x[worldid, k])


@wp.kernel
def _solve_LD_sparse_qLDiag_mul(
  # In:
  D: wp.array2d(dtype=float),
  # Out:
  out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  out[worldid, dofid] *= D[worldid, dofid]


@wp.kernel
def _solve_LD_sparse_x_acc_down(
  # In:
  L: wp.array3d(dtype=float),
  qLD_updates_: wp.array(dtype=wp.vec3i),
  # Out:
  x: wp.array2d(dtype=float),
):
  worldid, nodeid = wp.tid()
  update = qLD_updates_[nodeid]
  i, k, Madr_ki = update[0], update[1], update[2]
  wp.atomic_sub(x[worldid], k, L[worldid, 0, Madr_ki] * x[worldid, i])


def _solve_LD_sparse(
  m: Model,
  d: Data,
  L: wp.array3d(dtype=float),
  D: wp.array2d(dtype=float),
  x: wp.array2d(dtype=float),
  y: wp.array2d(dtype=float),
):
  """Computes sparse backsubstitution: x = inv(L'*D*L)*y."""
  wp.copy(x, y)
  for qLD_updates in reversed(m.qLD_updates):
    wp.launch(_solve_LD_sparse_x_acc_up, dim=(d.nworld, qLD_updates.size), inputs=[L, qLD_updates], outputs=[x])

  wp.launch(_solve_LD_sparse_qLDiag_mul, dim=(d.nworld, m.nv), inputs=[D], outputs=[x])

  for qLD_updates in m.qLD_updates:
    wp.launch(_solve_LD_sparse_x_acc_down, dim=(d.nworld, qLD_updates.size), inputs=[L, qLD_updates], outputs=[x])


@cache_kernel
def _tile_cholesky_solve(tile: TileSet):
  """Returns a kernel for dense Cholesky backsubstitution of a tile."""

  @nested_kernel(module="unique", enable_backward=False)
  def cholesky_solve(
    # In:
    L: wp.array3d(dtype=float),
    y: wp.array2d(dtype=float),
    adr: wp.array(dtype=int),
    # Out:
    x: wp.array2d(dtype=float),
  ):
    worldid, nodeid = wp.tid()
    TILE_SIZE = wp.static(tile.size)

    dofid = adr[nodeid]
    y_slice = wp.tile_load(y[worldid], shape=(TILE_SIZE,), offset=(dofid,))
    L_tile = wp.tile_load(L[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(dofid, dofid))
    x_slice = wp.tile_cholesky_solve(L_tile, y_slice)
    wp.tile_store(x[worldid], x_slice, offset=(dofid,))

  return cholesky_solve


def _solve_LD_dense(m: Model, d: Data, L: wp.array3d(dtype=float), x: wp.array2d(dtype=float), y: wp.array2d(dtype=float)):
  """Computes dense backsubstitution: x = inv(L'*L)*y."""
  for tile in m.qM_tiles:
    wp.launch_tiled(
      _tile_cholesky_solve(tile),
      dim=(d.nworld, tile.adr.size),
      inputs=[L, y, tile.adr],
      outputs=[x],
      block_dim=m.block_dim.cholesky_solve,
    )


def solve_LD(
  m: Model,
  d: Data,
  L: wp.array3d(dtype=float),
  D: wp.array2d(dtype=float),
  x: wp.array2d(dtype=float),
  y: wp.array2d(dtype=float),
):
  """Computes backsubstitution to solve a linear system of the form x = inv(L'*D*L) * y.

  L and D are the factors from the Cholesky factorization of the inertia matrix.

  This function dispatches to either a sparse or dense solver depending on Model options.

  Args:
    m: The model containing factorization and sparsity information.
    d: The data object containing workspace and factorization results.
    L: Lower-triangular factor from the factorization (sparse or dense).
    D: Diagonal factor from the factorization (only used for sparse).
    x: Output array for the solution.
    y: Input right-hand side array.
  """
  if m.opt.is_sparse:
    _solve_LD_sparse(m, d, L, D, x, y)
  else:
    _solve_LD_dense(m, d, L, x, y)


@event_scope
def solve_m(m: Model, d: Data, x: wp.array2d(dtype=float), y: wp.array2d(dtype=float)):
  """Computes backsubstitution: x = qLD * y.

  Args:
    m: The model containing inertia and factorization information.
    d: The data object containing factorization results.
    x: Output array for the solution.
    y: Input right-hand side array.
  """
  solve_LD(m, d, d.qLD, d.qLDiagInv, x, y)


@cache_kernel
def _tile_cholesky_factorize_solve(tile: TileSet):
  """Returns a kernel for dense Cholesky factorization and backsubstitution of a tile."""

  @nested_kernel(module="unique", enable_backward=False)
  def cholesky_factorize_solve(
    # In:
    M: wp.array3d(dtype=float),
    y: wp.array2d(dtype=float),
    adr: wp.array(dtype=int),
    # Out:
    x: wp.array2d(dtype=float),
    L: wp.array3d(dtype=float),
  ):
    worldid, nodeid = wp.tid()
    TILE_SIZE = wp.static(tile.size)

    dofid = adr[nodeid]
    M_tile = wp.tile_load(M[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(dofid, dofid))
    y_slice = wp.tile_load(y[worldid], shape=(TILE_SIZE,), offset=(dofid,))

    L_tile = wp.tile_cholesky(M_tile)
    wp.tile_store(L[worldid], L_tile, offset=(dofid, dofid))
    x_slice = wp.tile_cholesky_solve(L_tile, y_slice)
    wp.tile_store(x[worldid], x_slice, offset=(dofid,))

  return cholesky_factorize_solve


def _factor_solve_i_dense(
  m: Model,
  d: Data,
  M: wp.array3d(dtype=float),
  x: wp.array2d(dtype=float),
  y: wp.array2d(dtype=float),
  L: wp.array3d(dtype=float),
):
  for tile in m.qM_tiles:
    wp.launch_tiled(
      _tile_cholesky_factorize_solve(tile),
      dim=(d.nworld, tile.adr.size),
      inputs=[M, y, tile.adr],
      outputs=[x, L],
      block_dim=m.block_dim.cholesky_factorize_solve,
    )


def factor_solve_i(m, d, M, L, D, x, y):
  """Factorizes and solves the linear system: x = inv(L'*D*L) * y or x = inv(L'*L) * y.

  M is an inertia-like matrix and L, D are its Cholesky-like factors.

  This function first factorizes the matrix M (sparse or dense depending on model options),
  then solves the system for x given right-hand side y.

  Args:
    m: The model containing factorization and sparsity information.
    d: The data object containing workspace and factorization results.
    M: The inertia-like matrix to factorize.
    L: Output lower-triangular factor from the factorization (sparse or dense).
    D: Output diagonal factor from the factorization (only used for sparse).
    x: Output array for the solution.
    y: Input right-hand side array.
  """
  if m.opt.is_sparse:
    _factor_i_sparse(m, d, M, L, D)
    _solve_LD_sparse(m, d, L, D, x, y)
  else:
    _factor_solve_i_dense(m, d, M, x, y, L)


@wp.kernel
def _subtree_vel_forward(
  # Model:
  body_rootid: wp.array(dtype=int),
  body_mass: wp.array2d(dtype=float),
  body_inertia: wp.array2d(dtype=wp.vec3),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  subtree_linvel_out: wp.array2d(dtype=wp.vec3),
  subtree_angmom_out: wp.array2d(dtype=wp.vec3),
  subtree_bodyvel_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, bodyid = wp.tid()
  body_mass_id = worldid % body_mass.shape[0]
  body_inertia_id = worldid % body_inertia.shape[0]

  cvel = cvel_in[worldid, bodyid]
  ang = wp.spatial_top(cvel)
  lin = wp.spatial_bottom(cvel)
  xipos = xipos_in[worldid, bodyid]
  ximat = ximat_in[worldid, bodyid]
  subtree_com_root = subtree_com_in[worldid, body_rootid[bodyid]]

  # update linear velocity
  lin -= wp.cross(xipos - subtree_com_root, ang)

  subtree_linvel_out[worldid, bodyid] = body_mass[body_mass_id, bodyid] * lin
  dv = wp.transpose(ximat) @ ang
  dv[0] *= body_inertia[body_inertia_id, bodyid][0]
  dv[1] *= body_inertia[body_inertia_id, bodyid][1]
  dv[2] *= body_inertia[body_inertia_id, bodyid][2]
  subtree_angmom_out[worldid, bodyid] = ximat @ dv
  subtree_bodyvel_out[worldid, bodyid] = wp.spatial_vector(ang, lin)


@wp.kernel
def _linear_momentum(
  # Model:
  body_parentid: wp.array(dtype=int),
  body_subtreemass: wp.array2d(dtype=float),
  # Data in:
  subtree_linvel_in: wp.array2d(dtype=wp.vec3),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  subtree_linvel_out: wp.array2d(dtype=wp.vec3),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]
  if bodyid:
    pid = body_parentid[bodyid]
    wp.atomic_add(subtree_linvel_out[worldid], pid, subtree_linvel_in[worldid, bodyid])
  subtree_linvel_out[worldid, bodyid] /= wp.max(MJ_MINVAL, body_subtreemass[worldid % body_subtreemass.shape[0], bodyid])


@wp.kernel
def _angular_momentum(
  # Model:
  body_parentid: wp.array(dtype=int),
  body_mass: wp.array2d(dtype=float),
  body_subtreemass: wp.array2d(dtype=float),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  subtree_linvel_in: wp.array2d(dtype=wp.vec3),
  subtree_bodyvel_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  body_tree_: wp.array(dtype=int),
  # Data out:
  subtree_angmom_out: wp.array2d(dtype=wp.vec3),
):
  worldid, nodeid = wp.tid()
  bodyid = body_tree_[nodeid]

  if bodyid == 0:
    return

  pid = body_parentid[bodyid]

  xipos = xipos_in[worldid, bodyid]
  com = subtree_com_in[worldid, bodyid]
  com_parent = subtree_com_in[worldid, pid]
  vel = subtree_bodyvel_in[worldid, bodyid]
  linvel = subtree_linvel_in[worldid, bodyid]
  linvel_parent = subtree_linvel_in[worldid, pid]  # Data field
  mass = body_mass[worldid % body_mass.shape[0], bodyid]
  subtreemass = body_subtreemass[worldid % body_subtreemass.shape[0], bodyid]

  # momentum wrt body i
  dx = xipos - com
  dv = wp.spatial_bottom(vel) - linvel
  dp = dv * mass
  dL = wp.cross(dx, dp)

  # add to subtree i
  subtree_angmom_out[worldid, bodyid] += dL

  # add to parent
  wp.atomic_add(subtree_angmom_out[worldid], pid, subtree_angmom_out[worldid, bodyid])

  # momentum wrt parent
  dx = com - com_parent
  dv = linvel - linvel_parent
  dv *= subtreemass
  dL = wp.cross(dx, dv)
  wp.atomic_add(subtree_angmom_out[worldid], pid, dL)


def subtree_vel(m: Model, d: Data):
  """Computes subtree linear velocity and angular momentum.

  Computes the linear momentum and angular momentum for each subtree, accumulating
  contributions up the kinematic tree.
  """
  # bodywise quantities
  wp.launch(
    _subtree_vel_forward,
    dim=(d.nworld, m.nbody),
    inputs=[m.body_rootid, m.body_mass, m.body_inertia, d.xipos, d.ximat, d.subtree_com, d.cvel],
    outputs=[d.subtree_linvel, d.subtree_angmom, d.subtree_bodyvel],
  )

  # sum body linear momentum recursively up the kinematic tree
  for body_tree in reversed(m.body_tree):
    wp.launch(
      _linear_momentum,
      dim=[d.nworld, body_tree.size],
      inputs=[m.body_parentid, m.body_subtreemass, d.subtree_linvel, body_tree],
      outputs=[d.subtree_linvel],
    )

  for body_tree in reversed(m.body_tree):
    wp.launch(
      _angular_momentum,
      dim=[d.nworld, body_tree.size],
      inputs=[
        m.body_parentid,
        m.body_mass,
        m.body_subtreemass,
        d.xipos,
        d.subtree_com,
        d.subtree_linvel,
        d.subtree_bodyvel,
        body_tree,
      ],
      outputs=[d.subtree_angmom],
    )


@wp.kernel
def _joint_tendon(
  # Model:
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  wrap_objid: wp.array(dtype=int),
  wrap_prm: wp.array(dtype=float),
  tendon_jnt_adr: wp.array(dtype=int),
  wrap_jnt_adr: wp.array(dtype=int),
  # Data in:
  qpos_in: wp.array2d(dtype=float),
  # Data out:
  ten_J_out: wp.array3d(dtype=float),
  ten_length_out: wp.array2d(dtype=float),
):
  worldid, wrapid = wp.tid()

  tendon_jnt_adr_ = tendon_jnt_adr[wrapid]
  wrap_jnt_adr_ = wrap_jnt_adr[wrapid]

  wrap_objid_ = wrap_objid[wrap_jnt_adr_]
  prm = wrap_prm[wrap_jnt_adr_]

  # add to length
  L = prm * qpos_in[worldid, jnt_qposadr[wrap_objid_]]
  # TODO(team): compare atomic_add and for loop
  wp.atomic_add(ten_length_out[worldid], tendon_jnt_adr_, L)

  # add to moment
  ten_J_out[worldid, tendon_jnt_adr_, jnt_dofadr[wrap_objid_]] = prm


@wp.kernel
def _spatial_site_tendon(
  # Model:
  nv: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  wrap_objid: wp.array(dtype=int),
  tendon_site_pair_adr: wp.array(dtype=int),
  wrap_site_pair_adr: wp.array(dtype=int),
  wrap_pulley_scale: wp.array(dtype=float),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  ten_J_out: wp.array3d(dtype=float),
  ten_length_out: wp.array2d(dtype=float),
):
  worldid, elementid = wp.tid()

  # site pairs
  site_pair_adr = wrap_site_pair_adr[elementid]
  ten_adr = tendon_site_pair_adr[elementid]

  # pulley scaling
  pulley_scale = wrap_pulley_scale[site_pair_adr]

  id0 = wrap_objid[site_pair_adr + 0]
  id1 = wrap_objid[site_pair_adr + 1]

  pnt0 = site_xpos_in[worldid, id0]
  pnt1 = site_xpos_in[worldid, id1]
  dif = pnt1 - pnt0
  vec, length = math.normalize_with_norm(dif)
  wp.atomic_add(ten_length_out[worldid], ten_adr, length * pulley_scale)

  if length < MJ_MINVAL:
    vec = wp.vec3(1.0, 0.0, 0.0)

  body0 = site_bodyid[id0]
  body1 = site_bodyid[id1]
  if body0 != body1:
    # TODO(team): parallelize
    for i in range(nv):
      jacp1, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, pnt0, body0, i, worldid)
      jacp2, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, pnt1, body1, i, worldid)

      J = wp.dot(jacp2 - jacp1, vec)
      if J:
        wp.atomic_add(ten_J_out[worldid, ten_adr], i, J * pulley_scale)


@wp.kernel
def _spatial_geom_tendon(
  # Model:
  nv: int,
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  dof_bodyid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  site_bodyid: wp.array(dtype=int),
  wrap_type: wp.array(dtype=int),
  wrap_objid: wp.array(dtype=int),
  wrap_prm: wp.array(dtype=float),
  tendon_geom_adr: wp.array(dtype=int),
  wrap_geom_adr: wp.array(dtype=int),
  wrap_pulley_scale: wp.array(dtype=float),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  ten_J_out: wp.array3d(dtype=float),
  ten_length_out: wp.array2d(dtype=float),
  # Out:
  wrap_geom_xpos_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid, elementid = wp.tid()
  wrap_adr = wrap_geom_adr[elementid]
  ten_adr = tendon_geom_adr[elementid]

  # pulley scaling
  pulley_scale = wrap_pulley_scale[wrap_adr]

  # site-geom-site
  wrap_objid_site0 = wrap_objid[wrap_adr - 1]
  wrap_objid_geom = wrap_objid[wrap_adr + 0]
  wrap_objid_site1 = wrap_objid[wrap_adr + 1]

  # get site positions before and after geom
  site_pnt0 = site_xpos_in[worldid, wrap_objid_site0]
  site_pnt1 = site_xpos_in[worldid, wrap_objid_site1]

  # get geom information
  geom_xpos = geom_xpos_in[worldid, wrap_objid_geom]
  geom_xmat = geom_xmat_in[worldid, wrap_objid_geom]
  geomsize = geom_size[worldid, wrap_objid_geom][0]
  geom_type = wrap_type[wrap_adr]

  # get body ids for site-geom-site instances
  bodyid_site0 = site_bodyid[wrap_objid_site0]
  bodyid_geom = geom_bodyid[wrap_objid_geom]
  bodyid_site1 = site_bodyid[wrap_objid_site1]

  # find wrap object sidesite (if it exists)
  sideid = int(wp.round(wrap_prm[wrap_adr]))
  if sideid >= 0:
    side = site_xpos_in[worldid, sideid]
  else:
    side = wp.vec3(wp.inf)

  # compute geom wrap length and connect points (if wrap occurs)
  length_geomgeom, geom_pnt0, geom_pnt1 = util_misc.wrap(site_pnt0, site_pnt1, geom_xpos, geom_xmat, geomsize, geom_type, side)

  # store geom points
  wrap_geom_xpos_out[worldid, elementid] = wp.spatial_vector(geom_pnt0, geom_pnt1)

  if length_geomgeom >= 0.0:
    dif_sitegeom = geom_pnt0 - site_pnt0
    dif_geomsite = site_pnt1 - geom_pnt1
    vec_sitegeom, length_sitegeom = math.normalize_with_norm(dif_sitegeom)
    vec_geomsite, length_geomsite = math.normalize_with_norm(dif_geomsite)

    # length
    length_sitegeomsite = length_sitegeom + length_geomgeom + length_geomsite

    if length_sitegeomsite:
      wp.atomic_add(ten_length_out[worldid], ten_adr, length_sitegeomsite * pulley_scale)

    # moment
    if length_sitegeom < MJ_MINVAL:
      vec_sitegeom = wp.vec3(1.0, 0.0, 0.0)

    if length_geomsite < MJ_MINVAL:
      vec_geomsite = wp.vec3(1.0, 0.0, 0.0)

    dif_body_sitegeom = bodyid_site0 != bodyid_geom
    dif_body_geomsite = bodyid_geom != bodyid_site1

    # TODO(team): parallelize
    for i in range(nv):
      J = float(0.0)
      # site-geom
      if dif_body_sitegeom:
        jacp_site0, _ = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_pnt0, bodyid_site0, i, worldid
        )

        jacp_geom0, _ = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, geom_pnt0, bodyid_geom, i, worldid
        )

        J += wp.dot(jacp_geom0 - jacp_site0, vec_sitegeom)

      # geom-site
      if dif_body_geomsite:
        jacp_geom1, _ = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, geom_pnt1, bodyid_geom, i, worldid
        )

        jacp_site1, _ = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_pnt1, bodyid_site1, i, worldid
        )

        J += wp.dot(jacp_site1 - jacp_geom1, vec_geomsite)

      if J:
        wp.atomic_add(ten_J_out[worldid, ten_adr], i, J * pulley_scale)
  else:
    dif_sitesite = site_pnt1 - site_pnt0
    vec_sitesite, length_sitesite = math.normalize_with_norm(dif_sitesite)

    # length
    if length_sitesite:
      wp.atomic_add(ten_length_out[worldid], ten_adr, length_sitesite * pulley_scale)

    # moment
    if length_sitesite < MJ_MINVAL:
      vec_sitesite = wp.vec3(1.0, 0.0, 0.0)

    if bodyid_site0 != bodyid_site1:
      # TODO(team): parallelize
      for i in range(nv):
        jacp1, _ = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_pnt0, bodyid_site0, i, worldid
        )
        jacp2, _ = support.jac(
          body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, site_pnt1, bodyid_site1, i, worldid
        )

        J = wp.dot(jacp2 - jacp1, vec_sitesite)

        if J:
          wp.atomic_add(ten_J_out[worldid, ten_adr], i, J * pulley_scale)


@wp.kernel
def _spatial_tendon_wrap(
  # Model:
  ntendon: int,
  tendon_adr: wp.array(dtype=int),
  tendon_num: wp.array(dtype=int),
  wrap_type: wp.array(dtype=int),
  wrap_objid: wp.array(dtype=int),
  # Data in:
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  # In:
  wrap_geom_xpos_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  ten_wrapadr_out: wp.array2d(dtype=int),
  ten_wrapnum_out: wp.array2d(dtype=int),
  wrap_obj_out: wp.array2d(dtype=wp.vec2i),
  wrap_xpos_out: wp.array2d(dtype=wp.spatial_vector),
):
  worldid = wp.tid()

  wrapcount = int(0)
  wrapgeomid = int(0)
  for i in range(ntendon):
    adr = tendon_adr[i]
    ten_wrapadr_out[worldid, i] = wrapcount
    wrapnum = int(0)
    tendonnum = tendon_num[i]

    # process fixed tendon
    if wrap_type[adr] == WrapType.JOINT:
      continue

    # process spatial tendon
    j = int(0)
    while j < tendonnum - 1:
      # get 1st and 2nd object
      type0 = wrap_type[adr + j + 0]
      type1 = wrap_type[adr + j + 1]
      id0 = wrap_objid[adr + j + 0]
      id1 = wrap_objid[adr + j + 1]

      # pulley
      pulley0 = type0 == WrapType.PULLEY
      if pulley0 or type1 == WrapType.PULLEY:
        if pulley0:
          row = wrapcount // 2
          col = wrapcount % 2
          wrap_xpos_out[worldid, row][3 * col + 0] = 0.0
          wrap_xpos_out[worldid, row][3 * col + 1] = 0.0
          wrap_xpos_out[worldid, row][3 * col + 2] = 0.0

          wrap_obj_out[worldid, row][col] = -2

          wrapnum += 1
          wrapcount += 1

        # move to next
        j += 1
        continue

      # init sequence; assume it starts with site
      wpnt_site0 = site_xpos_in[worldid, id0]

      # second object is geom: process site-geom-site
      if type1 == WrapType.SPHERE or type1 == WrapType.CYLINDER:
        wrap_geom_xpos = wrap_geom_xpos_in[worldid, wrapgeomid]
        wpnt_geom0 = wp.spatial_top(wrap_geom_xpos)
        wrapgeomid += 1

        wrapid = id1
        id1 = wrap_objid[adr + j + 2]
        if wp.norm_l2(wpnt_geom0) < wp.inf:
          wpnt_geom1 = wp.spatial_bottom(wrap_geom_xpos)
          wpnt_site1 = site_xpos_in[worldid, id1]

          # assign to wrap
          row0 = (wrapcount + 0) // 2
          col0 = (wrapcount + 0) % 2
          row1 = (wrapcount + 1) // 2
          col1 = (wrapcount + 1) % 2
          row2 = (wrapcount + 2) // 2
          col2 = (wrapcount + 2) % 2
          row3 = (wrapcount + 3) // 2
          col3 = (wrapcount + 3) % 2

          wrap_xpos_out[worldid, row0][3 * col0 + 0] = wpnt_site0[0]
          wrap_xpos_out[worldid, row0][3 * col0 + 1] = wpnt_site0[1]
          wrap_xpos_out[worldid, row0][3 * col0 + 2] = wpnt_site0[2]

          wrap_xpos_out[worldid, row1][3 * col1 + 0] = wpnt_geom0[0]
          wrap_xpos_out[worldid, row1][3 * col1 + 1] = wpnt_geom0[1]
          wrap_xpos_out[worldid, row1][3 * col1 + 2] = wpnt_geom0[2]

          wrap_xpos_out[worldid, row2][3 * col2 + 0] = wpnt_geom1[0]
          wrap_xpos_out[worldid, row2][3 * col2 + 1] = wpnt_geom1[1]
          wrap_xpos_out[worldid, row2][3 * col2 + 2] = wpnt_geom1[2]

          wrap_xpos_out[worldid, row3][3 * col3 + 0] = wpnt_site1[0]
          wrap_xpos_out[worldid, row3][3 * col3 + 1] = wpnt_site1[1]
          wrap_xpos_out[worldid, row3][3 * col3 + 2] = wpnt_site1[2]

          wrap_obj_out[worldid, row0][col0] = -1
          wrap_obj_out[worldid, row1][col1] = wrapid
          wrap_obj_out[worldid, row2][col2] = wrapid

          wrapnum += 3
          wrapcount += 3
          j += 2

        else:
          row0 = (wrapcount + 0) // 2
          col0 = (wrapcount + 0) % 2

          wrap_xpos_out[worldid, row0][3 * col0 + 0] = wpnt_site0[0]
          wrap_xpos_out[worldid, row0][3 * col0 + 1] = wpnt_site0[1]
          wrap_xpos_out[worldid, row0][3 * col0 + 2] = wpnt_site0[2]

          wrap_obj_out[worldid, row0][col0] = -1

          wrapnum += 1
          wrapcount += 1
          j += 2

      else:
        row0 = (wrapcount + 0) // 2
        col0 = (wrapcount + 0) % 2

        wrap_xpos_out[worldid, row0][3 * col0 + 0] = wpnt_site0[0]
        wrap_xpos_out[worldid, row0][3 * col0 + 1] = wpnt_site0[1]
        wrap_xpos_out[worldid, row0][3 * col0 + 2] = wpnt_site0[2]

        wrap_obj_out[worldid, row0][col0] = -1

        wrapnum += 1
        wrapcount += 1
        j += 1

      # assign last site before pulley or tendon end
      if adr + j + 1 < wrap_type.shape[0]:
        last_before_pulley = wrap_type[adr + j + 1] == WrapType.PULLEY
      else:
        last_before_pulley = False

      if j == tendonnum - 1 or last_before_pulley:
        row0 = (wrapcount + 0) // 2
        col0 = (wrapcount + 0) % 2

        wpnt_site1 = site_xpos_in[worldid, id1]
        wrap_xpos_out[worldid, row0][3 * col0 + 0] = wpnt_site1[0]
        wrap_xpos_out[worldid, row0][3 * col0 + 1] = wpnt_site1[1]
        wrap_xpos_out[worldid, row0][3 * col0 + 2] = wpnt_site1[2]

        wrap_obj_out[worldid, row0][col0] = -1
        wrapnum += 1
        wrapcount += 1

    ten_wrapnum_out[worldid, i] = wrapnum


def tendon(m: Model, d: Data):
  """Computes tendon lengths and moments.

  Updates the tendon length and moment arrays for all tendons in the model, including joint,
  site, and geom tendons.
  """
  if not m.ntendon:
    return

  d.ten_length.zero_()
  d.ten_J.zero_()

  # Cartesian 3D points fro geom wrap points
  wrap_geom_xpos = wp.empty((d.nworld, m.nwrap), dtype=wp.spatial_vector)

  # process joint tendons
  wp.launch(
    _joint_tendon,
    dim=(d.nworld, m.wrap_jnt_adr.size),
    inputs=[m.jnt_qposadr, m.jnt_dofadr, m.wrap_objid, m.wrap_prm, m.tendon_jnt_adr, m.wrap_jnt_adr, d.qpos],
    outputs=[d.ten_J, d.ten_length],
  )

  spatial_site = m.wrap_site_pair_adr.size > 0
  spatial_geom = m.wrap_geom_adr.size > 0

  if spatial_site or spatial_geom:
    d.wrap_xpos.zero_()
    d.wrap_obj.zero_()

  # process spatial site tendons
  wp.launch(
    _spatial_site_tendon,
    dim=(d.nworld, m.wrap_site_pair_adr.size),
    inputs=[
      m.nv,
      m.body_parentid,
      m.body_rootid,
      m.dof_bodyid,
      m.site_bodyid,
      m.wrap_objid,
      m.tendon_site_pair_adr,
      m.wrap_site_pair_adr,
      m.wrap_pulley_scale,
      d.site_xpos,
      d.subtree_com,
      d.cdof,
    ],
    outputs=[d.ten_J, d.ten_length],
  )

  # process spatial geom tendons
  wp.launch(
    _spatial_geom_tendon,
    dim=(d.nworld, m.wrap_geom_adr.size),
    inputs=[
      m.nv,
      m.body_parentid,
      m.body_rootid,
      m.dof_bodyid,
      m.geom_bodyid,
      m.geom_size,
      m.site_bodyid,
      m.wrap_type,
      m.wrap_objid,
      m.wrap_prm,
      m.tendon_geom_adr,
      m.wrap_geom_adr,
      m.wrap_pulley_scale,
      d.geom_xpos,
      d.geom_xmat,
      d.site_xpos,
      d.subtree_com,
      d.cdof,
    ],
    outputs=[d.ten_J, d.ten_length, wrap_geom_xpos],
  )

  if spatial_site or spatial_geom:
    wp.launch(
      _spatial_tendon_wrap,
      dim=d.nworld,
      inputs=[m.ntendon, m.tendon_adr, m.tendon_num, m.wrap_type, m.wrap_objid, d.site_xpos, wrap_geom_xpos],
      outputs=[d.ten_wrapadr, d.ten_wrapnum, d.wrap_obj, d.wrap_xpos],
    )
