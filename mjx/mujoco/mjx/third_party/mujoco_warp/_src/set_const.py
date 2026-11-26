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


import dataclasses
import warnings

import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math as mjmath
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType

wp.set_module_options({"default_grid_stride": False})


# TODO: Update kernel_analyzer to support Model fields as outputs so it can be enabled here.
# kernel_analyzer: off
@wp.kernel
def _subtreemass(
  body_subtreenum: wp.array[int],
  body_mass_in: wp.array2d[float],
  body_subtreemass_out: wp.array2d[float],
):
  """Sums body_mass over the subtree, the contiguous body range [b, b + body_subtreenum[b])."""
  worldid, bodyid = wp.tid()
  body_mass_id = worldid % body_mass_in.shape[0]
  mass = float(0.0)
  for j in range(bodyid, bodyid + body_subtreenum[bodyid]):
    mass += body_mass_in[body_mass_id, j]
  body_subtreemass_out[worldid % body_subtreemass_out.shape[0], bodyid] = mass


@wp.kernel
def _copy_qpos0_to_qpos(
  qpos0: wp.array2d[float],
  qpos_out: wp.array2d[float],
):
  worldid, i = wp.tid()
  qpos0_id = worldid % qpos0.shape[0]
  qpos_out[worldid, i] = qpos0[qpos0_id, i]


@wp.kernel
def _copy_body_to_mocap(
  body_mocapid: wp.array[int],
  body_pos: wp.array2d[wp.vec3],
  body_quat: wp.array2d[wp.quat],
  mocap_pos_out: wp.array2d[wp.vec3],
  mocap_quat_out: wp.array2d[wp.quat],
):
  worldid, bodyid = wp.tid()
  mocapid = body_mocapid[bodyid]
  if mocapid >= 0:
    mocap_pos_out[worldid, mocapid] = body_pos[worldid % body_pos.shape[0], bodyid]
    mocap_quat_out[worldid, mocapid] = body_quat[worldid % body_quat.shape[0], bodyid]


@wp.kernel
def _copy_tendon_length0(
  ten_length_in: wp.array2d[float],
  tendon_length0_out: wp.array2d[float],
):
  worldid, tenid = wp.tid()
  tendon_length0_id = worldid % tendon_length0_out.shape[0]
  tendon_length0_out[tendon_length0_id, tenid] = ten_length_in[worldid, tenid]


@wp.kernel
def _compute_eq_data0(
  # Model:
  eq_type: wp.array[int],
  eq_obj1id: wp.array[int],
  eq_obj2id: wp.array[int],
  eq_objtype: wp.array[int],
  # Data in:
  xpos_in: wp.array2d[wp.vec3],
  xquat_in: wp.array2d[wp.quat],
  # Out:
  eq_data_out: wp.array2d[types.vec12],
):
  """Compute the data of equality constraints between bodies which follows from qpos0.

  The anchor of connects in the second body, and the relative pose of welds which are marked in
  data[11] for it to be computed. Kinematics must have been evaluated at qpos0.
  """
  worldid, eqid = wp.tid()

  if eq_objtype[eqid] != int(types.ObjType.BODY.value):
    return

  eqtype = eq_type[eqid]
  eq_data_id = worldid % eq_data_out.shape[0]
  data = eq_data_out[eq_data_id, eqid]
  obj1id = eq_obj1id[eqid]
  obj2id = eq_obj2id[eqid]

  if eqtype == int(types.EqType.CONNECT.value):
    # data[0:3] = anchor in body1 local frame; map to global frame
    anchor1 = wp.vec3(data[0], data[1], data[2])
    pos = xpos_in[worldid, obj1id] + mjmath.quat_to_mat(xquat_in[worldid, obj1id]) @ anchor1

    # data[3:6] = anchor position in body2 local frame
    anchor2 = wp.transpose(mjmath.quat_to_mat(xquat_in[worldid, obj2id])) @ (pos - xpos_in[worldid, obj2id])
    data[3] = anchor2[0]
    data[4] = anchor2[1]
    data[5] = anchor2[2]
    eq_data_out[eq_data_id, eqid] = data
  elif eqtype == int(types.EqType.WELD.value) and data[11] != 0.0:
    # data[0:3] = anchor in body2 local frame; map to global frame
    anchor2 = wp.vec3(data[0], data[1], data[2])
    pos = xpos_in[worldid, obj2id] + mjmath.quat_to_mat(xquat_in[worldid, obj2id]) @ anchor2

    # data[3:6] = anchor position in body1 local frame
    anchor1 = wp.transpose(mjmath.quat_to_mat(xquat_in[worldid, obj1id])) @ (pos - xpos_in[worldid, obj1id])
    data[3] = anchor1[0]
    data[4] = anchor1[1]
    data[5] = anchor1[2]

    # data[6:10] = neg(xquat1) * xquat2 = "xquat2 - xquat1" in body1 local frame
    relquat = mjmath.mul_quat(mjmath.quat_inv(xquat_in[worldid, obj1id]), xquat_in[worldid, obj2id])
    data[6] = relquat[0]
    data[7] = relquat[1]
    data[8] = relquat[2]
    data[9] = relquat[3]
    eq_data_out[eq_data_id, eqid] = data


@wp.kernel
def _compute_geom_bound(
  # Model:
  geom_type: wp.array[int],
  geom_dataid: wp.array2d[int],
  hfield_size: wp.array[wp.vec4],
  # Out:
  geom_size_out: wp.array2d[wp.vec3],
  geom_aabb_out: wp.array3d[wp.vec3],
  geom_rbound_out: wp.array2d[float],
):
  """Compute the bounding box and sphere of geoms whose bounds follow from their size.

  The size of a height field geom follows its height field. Mesh and SDF geoms keep the bounds of
  their mesh.
  """
  worldid, geomid = wp.tid()
  gtype = geom_type[geomid]
  if gtype == int(types.GeomType.MESH.value) or gtype == int(types.GeomType.SDF.value):
    return

  geom_size_id = worldid % geom_size_out.shape[0]
  size = geom_size_out[geom_size_id, geomid]
  lo = wp.vec3(0.0)
  hi = wp.vec3(0.0)
  rbound = float(0.0)

  if gtype == int(types.GeomType.HFIELD.value):
    hsize = hfield_size[geom_dataid[worldid % geom_dataid.shape[0], geomid]]
    geom_size_out[geom_size_id, geomid] = wp.vec3(hsize[0], hsize[1], 0.25 * hsize[2] + 0.5 * hsize[3])
    lo = wp.vec3(-hsize[0], -hsize[1], -hsize[3])
    hi = wp.vec3(hsize[0], hsize[1], hsize[2])
    rbound = wp.sqrt(hsize[0] * hsize[0] + hsize[1] * hsize[1] + wp.max(hsize[2] * hsize[2], hsize[3] * hsize[3]))
  elif gtype == int(types.GeomType.PLANE.value):
    lo = wp.vec3(-types.MJ_MAXVAL)
    hi = wp.vec3(types.MJ_MAXVAL, types.MJ_MAXVAL, 0.0)
  else:
    if gtype == int(types.GeomType.SPHERE.value):
      hi = wp.vec3(size[0])
      rbound = size[0]
    elif gtype == int(types.GeomType.CAPSULE.value):
      hi = wp.vec3(size[0], size[0], size[0] + size[1])
      rbound = size[0] + size[1]
    elif gtype == int(types.GeomType.CYLINDER.value):
      hi = wp.vec3(size[0], size[0], size[1])
      rbound = wp.sqrt(size[0] * size[0] + size[1] * size[1])
    elif gtype == int(types.GeomType.ELLIPSOID.value):
      hi = size
      rbound = wp.max(wp.max(size[0], size[1]), size[2])
    elif gtype == int(types.GeomType.BOX.value):
      hi = size
      rbound = wp.length(size)
    else:
      hi = size
    lo = -hi

  geom_aabb_id = worldid % geom_aabb_out.shape[0]
  geom_aabb_out[geom_aabb_id, geomid, 0] = 0.5 * (hi + lo)
  geom_aabb_out[geom_aabb_id, geomid, 1] = 0.5 * (hi - lo)
  geom_rbound_out[worldid % geom_rbound_out.shape[0], geomid] = rbound


@wp.func
def _minv_entry(
  dof_treeid: wp.array[int],
  tree_dofadr: wp.array[int],
  tree_dofnum: wp.array[int],
  Minv_block_adr: wp.array[int],
  Minv_in: wp.array2d[float],
  worldid: int,
  i: int,
  j: int,
) -> float:
  """Returns entry (i, j) of inv(M) from the blocks computed by smooth.inv_m."""
  treeid = dof_treeid[i]
  if dof_treeid[j] != treeid:
    return 0.0
  start = tree_dofadr[treeid]
  return Minv_in[worldid, Minv_block_adr[i] + (i - start) * tree_dofnum[treeid] + (j - start)]


@wp.kernel
def _finalize_dof_invweight0(
  dof_jntid: wp.array[int],
  jnt_type: wp.array[int],
  jnt_dofadr: wp.array[int],
  dof_treeid: wp.array[int],
  tree_dofadr: wp.array[int],
  tree_dofnum: wp.array[int],
  Minv_block_adr: wp.array[int],
  Minv_in: wp.array2d[float],
  dof_invweight0_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  jntid = dof_jntid[dofid]
  jtype = jnt_type[jntid]
  dofadr = jnt_dofadr[jntid]

  # average the diagonal of inv(M) over the translational or rotational dofs of FREE and BALL joints
  first = dofid
  num = int(1)
  if jtype == int(types.JointType.FREE.value):
    first = wp.where(dofid < dofadr + 3, dofadr, dofadr + 3)
    num = 3
  elif jtype == int(types.JointType.BALL.value):
    first = dofadr
    num = 3

  avg = float(0.0)
  for i in range(first, first + num):
    avg += _minv_entry(dof_treeid, tree_dofadr, tree_dofnum, Minv_block_adr, Minv_in, worldid, i, i)
  dof_invweight0_out[worldid % dof_invweight0_out.shape[0], dofid] = avg / float(num)


@wp.kernel
def _compute_body_invweight0(
  body_rootid: wp.array[int],
  body_weldid: wp.array[int],
  body_dofnum: wp.array[int],
  body_dofadr: wp.array[int],
  dof_parentid: wp.array[int],
  dof_treeid: wp.array[int],
  tree_dofadr: wp.array[int],
  tree_dofnum: wp.array[int],
  Minv_block_adr: wp.array[int],
  xipos_in: wp.array2d[wp.vec3],
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  Minv_in: wp.array2d[float],
  body_invweight0_out: wp.array2d[wp.vec2],
):
  worldid, bodyid = wp.tid()
  body_invweight0_id = worldid % body_invweight0_out.shape[0]

  # World body and static bodies have zero invweight
  weldid = body_weldid[bodyid]
  dofnum = body_dofnum[weldid]
  if dofnum == 0:
    body_invweight0_out[body_invweight0_id, bodyid] = wp.vec2(0.0, 0.0)
    return

  # Compute offset from point (xipos) to subtree_com of root body
  offset = xipos_in[worldid, bodyid] - subtree_com_in[worldid, body_rootid[bodyid]]

  # Get last dof that affects this body. The chain lies in one tree, so its inv(M) is one block.
  last_dofid = body_dofadr[weldid] + dofnum - 1
  treeid = dof_treeid[last_dofid]
  start = tree_dofadr[treeid]
  size = tree_dofnum[treeid]
  block_adr = Minv_block_adr[last_dofid]

  # diagonal of A = J * inv(M) * J', where J is the body Jacobian on the dof ancestor chain;
  # inv(M) is symmetric, so each off-diagonal pair is visited once and counted twice
  A_trans = wp.vec3(0.0)
  A_rot = wp.vec3(0.0)
  dofid_a = last_dofid
  while dofid_a >= 0:
    cdof_a = cdof_in[worldid, dofid_a]
    jac_rot_a = wp.spatial_top(cdof_a)
    jac_trans_a = wp.spatial_bottom(cdof_a) + wp.cross(jac_rot_a, offset)
    row_adr = block_adr + (dofid_a - start) * size
    sub_trans = wp.vec3(0.0)
    sub_rot = wp.vec3(0.0)
    dofid_b = dof_parentid[dofid_a]
    while dofid_b >= 0:
      minv = Minv_in[worldid, row_adr + (dofid_b - start)]
      cdof_b = cdof_in[worldid, dofid_b]
      jac_rot_b = wp.spatial_top(cdof_b)
      jac_trans_b = wp.spatial_bottom(cdof_b) + wp.cross(jac_rot_b, offset)
      sub_trans += minv * jac_trans_b
      sub_rot += minv * jac_rot_b
      dofid_b = dof_parentid[dofid_b]
    minv_aa = Minv_in[worldid, row_adr + (dofid_a - start)]
    A_trans += wp.cw_mul(jac_trans_a, minv_aa * jac_trans_a + 2.0 * sub_trans)
    A_rot += wp.cw_mul(jac_rot_a, minv_aa * jac_rot_a + 2.0 * sub_rot)
    dofid_a = dof_parentid[dofid_a]

  # Average diagonal: trans = (A[0,0]+A[1,1]+A[2,2])/3, rot = (A[3,3]+A[4,4]+A[5,5])/3
  inv_trans = wp.static(1.0 / 3.0) * (A_trans[0] + A_trans[1] + A_trans[2])
  inv_rot = wp.static(1.0 / 3.0) * (A_rot[0] + A_rot[1] + A_rot[2])

  body_invweight0_out[body_invweight0_id, bodyid] = wp.vec2(inv_trans, inv_rot)


@wp.kernel
def _compute_tendon_invweight0(
  ten_J_rownnz: wp.array[int],
  ten_J_rowadr: wp.array[int],
  ten_J_colind: wp.array[int],
  dof_treeid: wp.array[int],
  tree_dofadr: wp.array[int],
  tree_dofnum: wp.array[int],
  Minv_block_adr: wp.array[int],
  ten_J_in: wp.array2d[float],
  Minv_in: wp.array2d[float],
  tendon_invweight0_out: wp.array2d[float],
):
  worldid, tenid = wp.tid()
  rownnz = ten_J_rownnz[tenid]
  rowadr = ten_J_rowadr[tenid]

  # J_t * inv(M) * J_t'
  value = float(0.0)
  for p in range(rownnz):
    dofid_p = ten_J_colind[rowadr + p]
    jac_p = ten_J_in[worldid, rowadr + p]
    for q in range(rownnz):
      dofid_q = ten_J_colind[rowadr + q]
      minv = _minv_entry(dof_treeid, tree_dofadr, tree_dofnum, Minv_block_adr, Minv_in, worldid, dofid_p, dofid_q)
      value += jac_p * minv * ten_J_in[worldid, rowadr + q]
  tendon_invweight0_out[worldid % tendon_invweight0_out.shape[0], tenid] = value


@wp.kernel
def _compute_cam_pos0(
  cam_bodyid: wp.array[int],
  cam_targetbodyid: wp.array[int],
  cam_pos: wp.array2d[wp.vec3],
  cam_quat: wp.array2d[wp.quat],
  xpos_in: wp.array2d[wp.vec3],
  xquat_in: wp.array2d[wp.quat],
  subtree_com_in: wp.array2d[wp.vec3],
  cam_pos0_out: wp.array2d[wp.vec3],
  cam_poscom0_out: wp.array2d[wp.vec3],
  cam_mat0_out: wp.array2d[wp.mat33],
):
  worldid, camid = wp.tid()
  bodyid = cam_bodyid[camid]
  targetid = cam_targetbodyid[camid]
  xpos = xpos_in[worldid, bodyid]
  xquat = xquat_in[worldid, bodyid]

  # Fixed-mode camera pose at qpos0
  rel_pos = mjmath.rot_vec_quat(cam_pos[worldid % cam_pos.shape[0], camid], xquat)
  cam_xpos = xpos + rel_pos
  cam_xmat = mjmath.quat_to_mat(mjmath.mul_quat(xquat, cam_quat[worldid % cam_quat.shape[0], camid]))

  cam_pos0_out[worldid % cam_pos0_out.shape[0], camid] = rel_pos
  comid = wp.where(targetid >= 0, targetid, bodyid)
  cam_poscom0_out[worldid % cam_poscom0_out.shape[0], camid] = cam_xpos - subtree_com_in[worldid, comid]
  cam_mat0_out[worldid % cam_mat0_out.shape[0], camid] = cam_xmat


@wp.kernel
def _compute_light_pos0(
  light_bodyid: wp.array[int],
  light_targetbodyid: wp.array[int],
  light_pos: wp.array2d[wp.vec3],
  light_dir: wp.array2d[wp.vec3],
  xpos_in: wp.array2d[wp.vec3],
  xquat_in: wp.array2d[wp.quat],
  subtree_com_in: wp.array2d[wp.vec3],
  light_pos0_out: wp.array2d[wp.vec3],
  light_poscom0_out: wp.array2d[wp.vec3],
  light_dir0_out: wp.array2d[wp.vec3],
):
  worldid, lightid = wp.tid()
  bodyid = light_bodyid[lightid]
  targetid = light_targetbodyid[lightid]
  xpos = xpos_in[worldid, bodyid]
  xquat = xquat_in[worldid, bodyid]

  # Fixed-mode light pose at qpos0
  rel_pos = mjmath.rot_vec_quat(light_pos[worldid % light_pos.shape[0], lightid], xquat)
  light_xpos = xpos + rel_pos
  light_xdir = wp.normalize(mjmath.rot_vec_quat(light_dir[worldid % light_dir.shape[0], lightid], xquat))

  light_pos0_out[worldid % light_pos0_out.shape[0], lightid] = rel_pos
  comid = wp.where(targetid >= 0, targetid, bodyid)
  light_poscom0_out[worldid % light_poscom0_out.shape[0], lightid] = light_xpos - subtree_com_in[worldid, comid]
  light_dir0_out[worldid % light_dir0_out.shape[0], lightid] = light_xdir


@wp.kernel
def _compute_actuator_acc0(
  nv: int,
  dof_treeid: wp.array[int],
  tree_dofadr: wp.array[int],
  tree_dofnum: wp.array[int],
  Minv_block_adr: wp.array[int],
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  Minv_in: wp.array2d[float],
  actuator_acc0_out: wp.array2d[float],
):
  worldid, actid = wp.tid()
  rownnz = moment_rownnz_in[worldid, actid]
  rowadr = moment_rowadr_in[worldid, actid]

  # inv(M) * moment is nonzero only on the trees of the moment's dofs
  dofid_lo = nv
  dofid_hi = int(0)
  for q in range(rownnz):
    treeid = dof_treeid[moment_colind_in[worldid, rowadr + q]]
    dofadr = tree_dofadr[treeid]
    dofid_lo = wp.min(dofid_lo, dofadr)
    dofid_hi = wp.max(dofid_hi, dofadr + tree_dofnum[treeid])

  # ||inv(M) * moment||, accumulating each entry of inv(M) * moment in turn
  norm_sq = float(0.0)
  for i in range(dofid_lo, dofid_hi):
    value = float(0.0)
    for q in range(rownnz):
      sparseid = rowadr + q
      dofid = moment_colind_in[worldid, sparseid]
      minv = _minv_entry(dof_treeid, tree_dofadr, tree_dofnum, Minv_block_adr, Minv_in, worldid, i, dofid)
      value += minv * actuator_moment_in[worldid, sparseid]
    norm_sq += value * value
  actuator_acc0_out[worldid, actid] = wp.sqrt(norm_sq)


@wp.kernel
def _set_length_range(
  actuator_trntype: wp.array[int],
  actuator_trnid: wp.array[wp.vec2i],
  actuator_gear: wp.array2d[wp.spatial_vector],
  jnt_limited: wp.array[int],
  jnt_range: wp.array2d[wp.vec2],
  tendon_limited: wp.array[int],
  tendon_range: wp.array2d[wp.vec2],
  ntendon: int,
  actuator_lengthrange_out: wp.array2d[wp.vec2],
):
  worldid, actid = wp.tid()
  trntype = actuator_trntype[actid]
  id0 = actuator_trnid[actid][0]
  gear0 = actuator_gear[worldid % actuator_gear.shape[0], actid][0]

  lr = wp.vec2(0.0, 0.0)

  if trntype == TrnType.JOINT or trntype == TrnType.JOINTINPARENT:
    if jnt_limited[id0]:
      rng = jnt_range[worldid % jnt_range.shape[0], id0]
      if gear0 > 0.0:
        lr = wp.vec2(rng[0] * gear0, rng[1] * gear0)
      else:
        lr = wp.vec2(rng[1] * gear0, rng[0] * gear0)
  elif trntype == TrnType.TENDON:
    if ntendon > 0 and tendon_limited[id0]:
      rng = tendon_range[worldid % tendon_range.shape[0], id0]
      if gear0 > 0.0:
        lr = wp.vec2(rng[0] * gear0, rng[1] * gear0)
      else:
        lr = wp.vec2(rng[1] * gear0, rng[0] * gear0)

  actuator_lengthrange_out[worldid, actid] = lr


# kernel_analyzer: on


def _set_bound(m: types.Model):
  """Compute geom bounding boxes and spheres."""
  nworld_geom = np.max([m.geom_dataid.shape[0], m.geom_size.shape[0], m.geom_aabb.shape[0], m.geom_rbound.shape[0]])
  wp.launch(
    _compute_geom_bound,
    dim=(nworld_geom, m.ngeom),
    inputs=[m.geom_type, m.geom_dataid, m.hfield_size],
    outputs=[m.geom_size, m.geom_aabb, m.geom_rbound],
  )


def _set_fixed(m: types.Model):
  """Compute fixed quantities (do not depend on qpos0)."""
  nworld_subtreemass = m.body_subtreemass.shape[0]
  wp.launch(
    _subtreemass,
    dim=(nworld_subtreemass, m.nbody),
    inputs=[m.body_subtreenum, m.body_mass],
    outputs=[m.body_subtreemass],
  )


def _kinematics0(m: types.Model, d: types.Data):
  """Run kinematics and tendon lengths in qpos0."""
  wp.launch(_copy_qpos0_to_qpos, dim=(d.nworld, m.nq), inputs=[m.qpos0], outputs=[d.qpos])
  if m.nmocap:
    wp.launch(
      _copy_body_to_mocap,
      dim=(d.nworld, m.nbody),
      inputs=[m.body_mocapid, m.body_pos, m.body_quat],
      outputs=[d.mocap_pos, d.mocap_quat],
    )
  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.tendon(m, d)


def _set_kinematics0(m: types.Model, d: types.Data):
  """Compute kinematic quantities in qpos0 (requires _kinematics0)."""
  wp.launch(_copy_tendon_length0, dim=(m.tendon_length0.shape[0], m.ntendon), inputs=[d.ten_length], outputs=[m.tendon_length0])

  wp.launch(
    _compute_eq_data0,
    dim=(m.eq_data.shape[0], m.neq),
    inputs=[m.eq_type, m.eq_obj1id, m.eq_obj2id, m.eq_objtype, d.xpos, d.xquat],
    outputs=[m.eq_data],
  )

  nworld_cam = np.max([m.cam_pos0.shape[0], m.cam_poscom0.shape[0], m.cam_mat0.shape[0]])
  wp.launch(
    _compute_cam_pos0,
    dim=(nworld_cam, m.ncam),
    inputs=[m.cam_bodyid, m.cam_targetbodyid, m.cam_pos, m.cam_quat, d.xpos, d.xquat, d.subtree_com],
    outputs=[m.cam_pos0, m.cam_poscom0, m.cam_mat0],
  )

  nworld_light = np.max([m.light_pos0.shape[0], m.light_poscom0.shape[0], m.light_dir0.shape[0]])
  wp.launch(
    _compute_light_pos0,
    dim=(nworld_light, m.nlight),
    inputs=[m.light_bodyid, m.light_targetbodyid, m.light_pos, m.light_dir, d.xpos, d.xquat, d.subtree_com],
    outputs=[m.light_pos0, m.light_poscom0, m.light_dir0],
  )


def _set_inertia0(m: types.Model, d: types.Data):
  """Compute inertial quantities in qpos0 (requires _kinematics0)."""
  smooth.crb(m, d)
  smooth.tendon_armature(m, d)
  smooth.factor_m(m, d)
  smooth.transmission(m, d)

  Minv = wp.empty((d.nworld, m.Minv_block_total), dtype=float)
  smooth.inv_m(m, d, Minv)

  # dof_invweight0: computed per joint with averaging for multi-DOF joints
  # FREE: 6 DOFs, trans gets mean(A[0:3]), rot gets mean(A[3:6])
  # BALL: 3 DOFs, all get mean(A[0:3])
  # HINGE/SLIDE: 1 DOF, gets A[0,0]
  wp.launch(
    _finalize_dof_invweight0,
    dim=(m.dof_invweight0.shape[0], m.nv),
    inputs=[m.dof_jntid, m.jnt_type, m.jnt_dofadr, m.dof_treeid, m.tree_dofadr, m.tree_dofnum, m.Minv_block_adr, Minv],
    outputs=[m.dof_invweight0],
  )

  # body_invweight0: computed as mean diagonal of J * inv(M) * J'
  # where J is the 6xnv body Jacobian (3 rows translation, 3 rows rotation)
  wp.launch(
    _compute_body_invweight0,
    dim=(m.body_invweight0.shape[0], m.nbody),
    inputs=[
      m.body_rootid,
      m.body_weldid,
      m.body_dofnum,
      m.body_dofadr,
      m.dof_parentid,
      m.dof_treeid,
      m.tree_dofadr,
      m.tree_dofnum,
      m.Minv_block_adr,
      d.xipos,
      d.subtree_com,
      d.cdof,
      Minv,
    ],
    outputs=[m.body_invweight0],
  )

  # tendon_invweight0[t] = J_t * inv(M) * J_t'
  wp.launch(
    _compute_tendon_invweight0,
    dim=(m.tendon_invweight0.shape[0], m.ntendon),
    inputs=[
      m.ten_J_rownnz,
      m.ten_J_rowadr,
      m.ten_J_colind,
      m.dof_treeid,
      m.tree_dofadr,
      m.tree_dofnum,
      m.Minv_block_adr,
      d.ten_J,
      Minv,
    ],
    outputs=[m.tendon_invweight0],
  )

  # actuator_acc0[i] = ||inv(M) * actuator_moment[i]|| - acceleration from unit actuator force
  wp.launch(
    _compute_actuator_acc0,
    dim=(m.actuator_acc0.shape[0], m.nactuator),
    inputs=[
      m.nv,
      m.dof_treeid,
      m.tree_dofadr,
      m.tree_dofnum,
      m.Minv_block_adr,
      d.moment_rownnz,
      d.moment_rowadr,
      d.moment_colind,
      d.actuator_moment,
      Minv,
    ],
    outputs=[m.actuator_acc0],
  )


def _restore(m: types.Model, d: types.Data, groups: int):
  """Restore state fields in d to correspond to d.qpos."""
  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.tendon(m, d)
  if groups & types.SetConstGroup.INERT0:
    smooth.crb(m, d)
    smooth.tendon_armature(m, d)
    smooth.factor_m(m, d)
    smooth.transmission(m, d)


def set_const(
  m: types.Model,
  d: types.Data,
  groups: int = types.SetConstGroup.ALL,
  restore: bool = True,
):
  """Recomputes qpos0-dependent and fixed constant model fields.

  This function propagates changes from some model fields to derived fields,
  allowing modifications that would otherwise be unsafe. It should be called
  after modifying model parameters at runtime.

  Model fields that can be modified safely with set_const:

  ==================================  ==============================================
  Field                               Notes
  ==================================  ==============================================
  qpos0
  body_mass, body_inertia,            Mass and inertia are usually scaled together
  body_ipos, body_iquat               since inertia is sum(m * r^2).
  body_pos, body_quat                 Unsafe for static bodies (invalidates BVH).
  dof_armature, tendon_armature
  actuator_armature, actuator_gear    Actuator armature is added to joint and tendon
                                      armature. Not if changing from/to zero, since
                                      jnt_actuatorid and tendon_actuatorid are set
                                      by put_model.
  eq_data                             Connect anchors, marked weld poses.
  geom_size, hfield_size              Bounds of primitive and height field geoms.
  cam_pos, cam_quat                   Camera pose in body frame.
  light_pos, light_dir                Light position and direction in body frame.
  tendon_stiffness, tendon_damping    Only if changing from/to zero.
  ==================================  ==============================================

  For selective updates, pass SetConstGroup flags to groups:

  ====================  =================================================
  Modified Field        Groups
  ====================  =================================================
  body_mass             SetConstGroup.ALL
  geom_size             SetConstGroup.FIXED (or ALL if tendon wraps geom)
  cam_pos, cam_quat     SetConstGroup.KIN0
  light_pos, light_dir  SetConstGroup.KIN0
  eq_data               SetConstGroup.KIN0
  body_inertia          SetConstGroup.INERT0
  dof_armature          SetConstGroup.INERT0
  qpos0                 SetConstGroup.KIN0 | SetConstGroup.INERT0
  ====================  =================================================

  Computes:
    - Fixed quantities (SetConstGroup.FIXED):
      - body_subtreemass: mass of body and all descendants
      - geom_aabb, geom_rbound, geom_size (hfield): geom bounds
    - Kinematic qpos0 quantities (SetConstGroup.KIN0):
      - tendon_length0: tendon resting lengths
      - eq_data: connect anchors and marked weld poses (eq_data[11])
      - cam_pos0, cam_poscom0, cam_mat0: camera references
      - light_pos0, light_poscom0, light_dir0: light references
    - Inertial qpos0 quantities (SetConstGroup.INERT0):
      - dof_invweight0: inverse inertia for DOFs
      - body_invweight0: inverse spatial inertia for bodies
      - tendon_invweight0: inverse weight for tendons
      - actuator_acc0: acceleration from unit actuator force

  Skips:
    - actuator_length0: not in mjwarp
    - dof_length, flexedge_length0, flexedge_invweight0: unbatched in mjwarp
    - tendon_lengthspring (from qpos_spring): SPRING group unsupported in mjwarp

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    groups: Bitmask of SetConstGroup flags selecting which groups to compute.
    restore: Whether to restore state fields to correspond to d.qpos.
  """
  if groups & types.SetConstGroup.FIXED:
    _set_bound(m)
    _set_fixed(m)

  if groups & (types.SetConstGroup.KIN0 | types.SetConstGroup.INERT0):
    opt = dataclasses.replace(m.opt, enableflags=m.opt.enableflags & ~types.EnableBit.SLEEP)
    m = dataclasses.replace(m, opt=opt)
    qpos_saved = wp.clone(d.qpos)
    mocap_pos_saved = wp.clone(d.mocap_pos) if m.nmocap else None
    mocap_quat_saved = wp.clone(d.mocap_quat) if m.nmocap else None

    # the constants do not depend on the contacts of d, which transmission averages for adhesion
    # actuators: hide them while the constants are computed, as mj_setConst clears them
    nacon_saved = wp.clone(d.nacon) if m.nacttrnbody else None
    if m.nacttrnbody:
      d.nacon.zero_()

    _kinematics0(m, d)
    if groups & types.SetConstGroup.KIN0:
      _set_kinematics0(m, d)
    if groups & types.SetConstGroup.INERT0:
      _set_inertia0(m, d)

    wp.copy(d.qpos, qpos_saved)
    if m.nmocap:
      wp.copy(d.mocap_pos, mocap_pos_saved)
      wp.copy(d.mocap_quat, mocap_quat_saved)
    if m.nacttrnbody:
      wp.copy(d.nacon, nacon_saved)

    if restore:
      _restore(m, d, groups)


# TODO(team): remove in future version
def set_const_fixed(m: types.Model, d: types.Data):
  """Recomputes qpos0-independent constant model fields (deprecated)."""
  warnings.warn(
    "set_const_fixed is deprecated, use set_const(m, d, groups=SetConstGroup.FIXED) instead.",
    DeprecationWarning,
    stacklevel=2,
  )
  set_const(m, d, groups=types.SetConstGroup.FIXED, restore=False)


# TODO(team): remove in future version
def set_const_0(m: types.Model, d: types.Data, restore: bool = True):
  """Recomputes qpos0-dependent constant model fields (deprecated)."""
  warnings.warn(
    "set_const_0 is deprecated, use set_const(m, d, groups=SetConstGroup.KIN0 | SetConstGroup.INERT0) instead.",
    DeprecationWarning,
    stacklevel=2,
  )
  set_const(
    m,
    d,
    groups=types.SetConstGroup.KIN0 | types.SetConstGroup.INERT0,
    restore=restore,
  )


def set_length_range(m: types.Model, d: types.Data, index: int = -1):
  """Compute feasible actuator length ranges from joint/tendon limits.

  For joint and tendon transmissions with limits, copies the range directly
  from jnt_range or tendon_range scaled by gear. Actuators without limits
  keep (0, 0). This covers the common robotics use case; simulation-based
  computation for general transmissions is not yet implemented.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object (unused, kept for API compatibility with MuJoCo C).
    index: Actuator index to compute for, or -1 for all actuators.
  """
  wp.launch(
    _set_length_range,
    dim=(d.nworld, m.nactuator),
    inputs=[
      m.actuator_trntype,
      m.actuator_trnid,
      m.actuator_gear,
      m.jnt_limited,
      m.jnt_range,
      m.tendon_limited,
      m.tendon_range,
      m.ntendon,
    ],
    outputs=[m.actuator_lengthrange],
  )
