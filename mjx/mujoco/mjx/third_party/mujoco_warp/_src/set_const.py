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


import mujoco
import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math as mjmath
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10

wp.set_module_options({"default_grid_stride": False})


# TODO: Update kernel_analyzer to support Model fields as outputs so it can be enabled here.
# kernel_analyzer: off
@wp.kernel
def _init_subtreemass(
  body_mass_in: wp.array2d[float],
  body_subtreemass_out: wp.array2d[float],
):
  worldid, bodyid = wp.tid()
  body_mass_id = worldid % body_mass_in.shape[0]
  body_subtreemass_id = worldid % body_subtreemass_out.shape[0]
  body_subtreemass_out[body_subtreemass_id, bodyid] = body_mass_in[body_mass_id, bodyid]


@wp.kernel
def _accumulate_subtreemass(
  body_parentid: wp.array[int],
  body_subtreemass_io: wp.array2d[float],
  body_tree_: wp.array[int],
):
  worldid, nodeid = wp.tid()
  body_subtreemass_id = worldid % body_subtreemass_io.shape[0]
  bodyid = body_tree_[nodeid]
  parentid = body_parentid[bodyid]
  if bodyid != 0:
    wp.atomic_add(body_subtreemass_io, body_subtreemass_id, parentid, body_subtreemass_io[body_subtreemass_id, bodyid])


@wp.kernel
def _copy_qpos0_to_qpos(
  qpos0: wp.array2d[float],
  qpos_out: wp.array2d[float],
):
  worldid, i = wp.tid()
  qpos0_id = worldid % qpos0.shape[0]
  qpos_out[worldid, i] = qpos0[qpos0_id, i]


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
  xmat_in: wp.array2d[wp.mat33],
  # Out:
  eq_data_out: wp.array2d[types.vec11],
):
  """Compute eq_data for connect/weld constraints.

  Kinematics must have been evaluated at qpos0 so the constraint is satisfied at qpos0.
  """
  worldid, eqid = wp.tid()
  eq_data_id = worldid % eq_data_out.shape[0]

  eqtype = eq_type[eqid]
  objtype = eq_objtype[eqid]
  data = eq_data_out[eq_data_id, eqid]

  if eqtype == int(types.EqType.CONNECT.value):
    if objtype == int(types.ObjType.BODY.value):
      obj1id = eq_obj1id[eqid]
      obj2id = eq_obj2id[eqid]

      # data[0:3] = anchor in body1 local frame; map to global frame
      anchor1 = wp.vec3(data[0], data[1], data[2])
      pos = xpos_in[worldid, obj1id] + xmat_in[worldid, obj1id] @ anchor1

      # data[3:6] = anchor position in body2 local frame
      anchor2 = wp.transpose(xmat_in[worldid, obj2id]) @ (pos - xpos_in[worldid, obj2id])
      data[3] = anchor2[0]
      data[4] = anchor2[1]
      data[5] = anchor2[2]
      eq_data_out[eq_data_id, eqid] = data
    elif objtype == int(types.ObjType.SITE.value):
      # site-based connect, eq_data is unused
      eq_data_out[eq_data_id, eqid] = types.vec11(0.0)
  elif eqtype == int(types.EqType.WELD.value):
    if objtype == int(types.ObjType.BODY.value):
      quat = wp.quat(data[6], data[7], data[8], data[9])
      if wp.length_sq(quat) > 0.0:
        # user has set quaternion data: normalize it and keep the remaining data
        quat = wp.normalize(quat)
        data[6] = quat[0]
        data[7] = quat[1]
        data[8] = quat[2]
        data[9] = quat[3]
        eq_data_out[eq_data_id, eqid] = data
      else:
        obj1id = eq_obj1id[eqid]
        obj2id = eq_obj2id[eqid]

        # data[0:3] = anchor in body2 local frame; map to global frame
        anchor2 = wp.vec3(data[0], data[1], data[2])
        pos = xpos_in[worldid, obj2id] + xmat_in[worldid, obj2id] @ anchor2

        # data[3:6] = anchor position in body1 local frame
        anchor1 = wp.transpose(xmat_in[worldid, obj1id]) @ (pos - xpos_in[worldid, obj1id])
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
def _resolve_tendon_lengthspring(
  ten_length_in: wp.array2d[float],
  tendon_lengthspring_out: wp.array2d[wp.vec2],
):
  worldid, tenid = wp.tid()
  tendon_lengthspring_id = worldid % tendon_lengthspring_out.shape[0]
  val = tendon_lengthspring_out[tendon_lengthspring_id, tenid]
  if val[0] == -1.0 and val[1] == -1.0:
    l = ten_length_in[worldid, tenid]
    tendon_lengthspring_out[tendon_lengthspring_id, tenid] = wp.vec2(l, l)


@wp.kernel
def _compute_meaninertia(
  nv: int,
  M_rownnz_in: wp.array[int],
  M_rowadr_in: wp.array[int],
  M_in: wp.array2d[float],
  meaninertia_out: wp.array[float],
):
  """Compute mean diagonal inertia from M at qpos0."""
  worldid = wp.tid()

  if nv == 0:
    meaninertia_out[worldid % meaninertia_out.shape[0]] = 1.0  # Default from MuJoCo
    return

  total = float(0.0)
  for i in range(nv):
    # CSR row diagonal is the last entry: M_rowadr_in[i] + M_rownnz_in[i] - 1
    madr = M_rowadr_in[i] + M_rownnz_in[i] - 1
    total += M_in[worldid, madr]

  meaninertia_out[worldid % meaninertia_out.shape[0]] = total / float(nv)


@wp.kernel
def _set_unit_vector(
  dofid_target: int,
  unit_vec_out: wp.array2d[float],
):
  worldid = wp.tid()
  nv = unit_vec_out.shape[1]
  for i in range(nv):
    if i == dofid_target:
      unit_vec_out[worldid, i] = 1.0
    else:
      unit_vec_out[worldid, i] = 0.0


@wp.kernel
def _extract_dof_A_diag(
  dofid: int,
  result_vec_in: wp.array2d[float],
  dof_A_diag_out: wp.array2d[float],
):
  worldid = wp.tid()
  dof_A_diag_id = worldid % dof_A_diag_out.shape[0]
  dof_A_diag_out[dof_A_diag_id, dofid] = result_vec_in[worldid, dofid]


@wp.kernel
def _finalize_dof_invweight0(
  dof_jntid: wp.array[int],
  jnt_type: wp.array[int],
  jnt_dofadr: wp.array[int],
  dof_A_diag_in: wp.array2d[float],
  dof_invweight0_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  dof_invweight0_id = worldid % dof_invweight0_out.shape[0]
  dof_A_diag_id = worldid % dof_A_diag_in.shape[0]

  jntid = dof_jntid[dofid]
  jtype = jnt_type[jntid]
  dofadr = jnt_dofadr[jntid]

  if jtype == int(types.JointType.FREE.value):
    # FREE joint: 6 DOFs, average first 3 (trans) and last 3 (rot) separately
    if dofid < dofadr + 3:
      avg = wp.static(1.0 / 3.0) * (
        dof_A_diag_in[dof_A_diag_id, dofadr + 0]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 1]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 2]
      )
    else:
      avg = wp.static(1.0 / 3.0) * (
        dof_A_diag_in[dof_A_diag_id, dofadr + 3]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 4]
        + dof_A_diag_in[dof_A_diag_id, dofadr + 5]
      )
    dof_invweight0_out[dof_invweight0_id, dofid] = avg
  elif jtype == int(types.JointType.BALL.value):
    # BALL joint: 3 DOFs, average all
    avg = wp.static(1.0 / 3.0) * (
      dof_A_diag_in[dof_A_diag_id, dofadr + 0]
      + dof_A_diag_in[dof_A_diag_id, dofadr + 1]
      + dof_A_diag_in[dof_A_diag_id, dofadr + 2]
    )
    dof_invweight0_out[dof_invweight0_id, dofid] = avg
  else:
    # HINGE/SLIDE: 1 DOF, no averaging
    dof_invweight0_out[dof_invweight0_id, dofid] = dof_A_diag_in[dof_A_diag_id, dofid]


@wp.kernel
def _compute_body_jac_row(
  nv: int,
  bodyid_target: int,
  row_idx: int,
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  body_dofadr: wp.array[int],
  body_dofnum: wp.array[int],
  dof_parentid: wp.array[int],
  subtree_com_in: wp.array2d[wp.vec3],
  xipos_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  body_jac_row_out: wp.array2d[float],
):
  worldid = wp.tid()

  for i in range(nv):
    body_jac_row_out[worldid, i] = 0.0

  bodyid = bodyid_target
  while bodyid > 0 and body_dofnum[bodyid] == 0:
    bodyid = body_parentid[bodyid]

  if bodyid == 0:
    return

  # Compute offset from point (xipos) to subtree_com of root body
  point = xipos_in[worldid, bodyid_target]
  offset = point - subtree_com_in[worldid, body_rootid[bodyid_target]]

  # Get last dof that affects this body
  dofid = body_dofadr[bodyid] + body_dofnum[bodyid] - 1

  # Backward pass over dof ancestor chain
  while dofid >= 0:
    cdof = cdof_in[worldid, dofid]
    cdof_ang = wp.spatial_top(cdof)
    cdof_lin = wp.spatial_bottom(cdof)

    if row_idx < 3:
      tmp = wp.cross(cdof_ang, offset)
      if row_idx == 0:
        body_jac_row_out[worldid, dofid] = cdof_lin[0] + tmp[0]
      elif row_idx == 1:
        body_jac_row_out[worldid, dofid] = cdof_lin[1] + tmp[1]
      else:
        body_jac_row_out[worldid, dofid] = cdof_lin[2] + tmp[2]
    else:
      if row_idx == 3:
        body_jac_row_out[worldid, dofid] = cdof_ang[0]
      elif row_idx == 4:
        body_jac_row_out[worldid, dofid] = cdof_ang[1]
      else:
        body_jac_row_out[worldid, dofid] = cdof_ang[2]

    dofid = dof_parentid[dofid]


@wp.kernel
def _compute_body_A_diag_entry(
  nv: int,
  bodyid_target: int,
  row_idx: int,
  body_jac_row_in: wp.array2d[float],
  result_vec_in: wp.array2d[float],
  body_A_diag_out: wp.array3d[float],
):
  worldid = wp.tid()
  body_A_diag_id = worldid % body_A_diag_out.shape[0]
  # A[row,row] = J[row] · inv(M) · J[row]' = J[row] · result_vec
  dot_prod = float(0.0)
  for i in range(nv):
    dot_prod += body_jac_row_in[worldid, i] * result_vec_in[worldid, i]
  body_A_diag_out[body_A_diag_id, bodyid_target, row_idx] = dot_prod


@wp.kernel
def _finalize_body_invweight0(
  body_weldid: wp.array[int],
  body_A_diag_in: wp.array3d[float],
  body_invweight0_out: wp.array2d[wp.vec2],
):
  worldid, bodyid = wp.tid()
  body_invweight0_id = worldid % body_invweight0_out.shape[0]
  body_A_diag_id = worldid % body_A_diag_in.shape[0]

  # World body and static bodies have zero invweight
  if bodyid == 0 or body_weldid[bodyid] == 0:
    body_invweight0_out[body_invweight0_id, bodyid] = wp.vec2(0.0, 0.0)
    return

  # Average diagonal: trans = (A[0,0]+A[1,1]+A[2,2])/3, rot = (A[3,3]+A[4,4]+A[5,5])/3
  inv_trans = wp.static(1.0 / 3.0) * (
    body_A_diag_in[body_A_diag_id, bodyid, 0]
    + body_A_diag_in[body_A_diag_id, bodyid, 1]
    + body_A_diag_in[body_A_diag_id, bodyid, 2]
  )
  inv_rot = wp.static(1.0 / 3.0) * (
    body_A_diag_in[body_A_diag_id, bodyid, 3]
    + body_A_diag_in[body_A_diag_id, bodyid, 4]
    + body_A_diag_in[body_A_diag_id, bodyid, 5]
  )

  # Prevent degenerate constraints: if one component is near zero, use the other as fallback
  if inv_trans < mujoco.mjMINVAL and inv_rot > mujoco.mjMINVAL:
    inv_trans = inv_rot  # use rotation as fallback for translation
  elif inv_rot < mujoco.mjMINVAL and inv_trans > mujoco.mjMINVAL:
    inv_rot = inv_trans  # use translation as fallback for rotation

  body_invweight0_out[body_invweight0_id, bodyid] = wp.vec2(inv_trans, inv_rot)


@wp.kernel
def _copy_tendon_jacobian(
  tenid_target: int,
  ten_J_rownnz: wp.array[int],
  ten_J_rowadr: wp.array[int],
  ten_J_colind: wp.array[int],
  ten_J_in: wp.array2d[float],
  ten_J_vec_out: wp.array2d[float],
):
  worldid = wp.tid()
  nv = ten_J_in.shape[2]
  rownnz = ten_J_rownnz[tenid_target]
  rowadr = ten_J_rowadr[tenid_target]
  for i in range(rownnz):
    colind = ten_J_colind[rowadr + i]
    ten_J_vec_out[worldid, colind] = ten_J_in[worldid, rowadr + i]


@wp.kernel
def _compute_tendon_dot_product(
  # Model:
  ten_J_rownnz: wp.array[int],
  ten_J_rowadr: wp.array[int],
  ten_J_colind: wp.array[int],
  # In:
  tenid_target: int,
  ten_J_in: wp.array2d[float],
  result_vec_in: wp.array2d[float],
  # Out:
  tendon_invweight0_out: wp.array2d[float],
):
  worldid = wp.tid()
  tendon_invweight0_id = worldid % tendon_invweight0_out.shape[0]
  dot_prod = float(0.0)

  rownnz = ten_J_rownnz[tenid_target]
  rowadr = ten_J_rowadr[tenid_target]
  for i in range(rownnz):
    sparseid = rowadr + i
    colind = ten_J_colind[sparseid]
    dot_prod += ten_J_in[worldid, sparseid] * result_vec_in[worldid, colind]

  tendon_invweight0_out[tendon_invweight0_id, tenid_target] = dot_prod


@wp.kernel
def _compute_cam_pos0(
  cam_bodyid: wp.array[int],
  cam_targetbodyid: wp.array[int],
  cam_xpos_in: wp.array2d[wp.vec3],
  cam_xmat_in: wp.array2d[wp.mat33],
  xpos_in: wp.array2d[wp.vec3],
  subtree_com_in: wp.array2d[wp.vec3],
  cam_pos0_out: wp.array2d[wp.vec3],
  cam_poscom0_out: wp.array2d[wp.vec3],
  cam_mat0_out: wp.array2d[wp.mat33],
):
  worldid, camid = wp.tid()
  cam_pos0_id = worldid % cam_pos0_out.shape[0]
  bodyid = cam_bodyid[camid]
  targetid = cam_targetbodyid[camid]
  cam_xpos = cam_xpos_in[worldid, camid]

  cam_pos0_out[cam_pos0_id, camid] = cam_xpos - xpos_in[worldid, bodyid]
  if targetid >= 0:
    cam_poscom0_out[cam_pos0_id, camid] = cam_xpos - subtree_com_in[worldid, targetid]
  else:
    cam_poscom0_out[cam_pos0_id, camid] = cam_xpos - subtree_com_in[worldid, bodyid]
  cam_mat0_out[cam_pos0_id, camid] = cam_xmat_in[worldid, camid]


@wp.kernel
def _compute_light_pos0(
  light_bodyid: wp.array[int],
  light_targetbodyid: wp.array[int],
  light_xpos_in: wp.array2d[wp.vec3],
  light_xdir_in: wp.array2d[wp.vec3],
  xpos_in: wp.array2d[wp.vec3],
  subtree_com_in: wp.array2d[wp.vec3],
  light_pos0_out: wp.array2d[wp.vec3],
  light_poscom0_out: wp.array2d[wp.vec3],
  light_dir0_out: wp.array2d[wp.vec3],
):
  worldid, lightid = wp.tid()
  light_pos0_id = worldid % light_pos0_out.shape[0]
  bodyid = light_bodyid[lightid]
  targetid = light_targetbodyid[lightid]
  light_xpos = light_xpos_in[worldid, lightid]

  light_pos0_out[light_pos0_id, lightid] = light_xpos - xpos_in[worldid, bodyid]
  if targetid >= 0:
    light_poscom0_out[light_pos0_id, lightid] = light_xpos - subtree_com_in[worldid, targetid]
  else:
    light_poscom0_out[light_pos0_id, lightid] = light_xpos - subtree_com_in[worldid, bodyid]
  light_dir0_out[light_pos0_id, lightid] = light_xdir_in[worldid, lightid]


@wp.kernel
def _copy_actuator_moment(
  actid_target: int,
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  act_moment_vec_out: wp.array2d[float],
):
  worldid = wp.tid()
  nv = act_moment_vec_out.shape[1]
  for i in range(nv):
    act_moment_vec_out[worldid, i] = 0.0
  rownnz = moment_rownnz_in[worldid, actid_target]
  rowadr = moment_rowadr_in[worldid, actid_target]
  for i in range(rownnz):
    sparseid = rowadr + i
    col = moment_colind_in[worldid, sparseid]
    act_moment_vec_out[worldid, col] = actuator_moment_in[worldid, sparseid]


@wp.kernel
def _compute_actuator_acc0(
  actid_target: int,
  nv: int,
  result_vec_in: wp.array2d[float],
  actuator_acc0_out: wp.array2d[float],
):
  worldid = wp.tid()
  norm_sq = float(0.0)
  for i in range(nv):
    norm_sq += result_vec_in[worldid, i] * result_vec_in[worldid, i]
  actuator_acc0_out[worldid, actid_target] = wp.sqrt(norm_sq)


@wp.kernel
def _compute_dof_M0(
  dof_bodyid: wp.array[int],
  dof_armature: wp.array2d[float],
  cdof_in: wp.array2d[wp.spatial_vector],
  crb_in: wp.array2d[vec10],
  dof_M0_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  bodyid = dof_bodyid[dofid]
  armature = dof_armature[worldid % dof_armature.shape[0], dofid]
  buf = mjmath.inert_vec(crb_in[worldid, bodyid], cdof_in[worldid, dofid])
  dof_M0_out[worldid, dofid] = armature + wp.dot(cdof_in[worldid, dofid], buf)


@wp.kernel
def _resolve_dampratio(
  actuator_biastype: wp.array[int],
  actuator_gainprm: wp.array2d[types.vec10],
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  dof_M0_in: wp.array2d[float],
  nv: int,
  actuator_biasprm: wp.array2d[types.vec10],
):
  worldid, actid = wp.tid()
  biastype = actuator_biastype[actid]

  # only affine bias (position actuators)
  if biastype != BiasType.AFFINE:
    return

  gainprm_id = worldid % actuator_gainprm.shape[0]
  biasprm_id = worldid % actuator_biasprm.shape[0]
  kp = actuator_gainprm[gainprm_id, actid][0]

  biasprm = actuator_biasprm[biasprm_id, actid]
  # dampratio condition: gainprm[0] == -biasprm[1] and biasprm[2] > 0
  if wp.abs(kp + biasprm[1]) > MJ_MINVAL:
    return
  if biasprm[2] <= 0.0:
    return

  dampratio = biasprm[2]

  # compute reflected mass: sum(dof_M0[j] / moment[i,j]^2) for active DOFs
  mass = float(0.0)
  rownnz = moment_rownnz_in[worldid, actid]
  rowadr = moment_rowadr_in[worldid, actid]
  for k in range(rownnz):
    sparseid = rowadr + k
    j = moment_colind_in[worldid, sparseid]
    moment = actuator_moment_in[worldid, sparseid]
    if wp.abs(moment) > MJ_MINVAL:
      mass += dof_M0_in[worldid, j] / (moment * moment)

  damping = dampratio * 2.0 * wp.sqrt(kp * mass)

  # write -damping to biasprm[2]
  new_biasprm = biasprm
  new_biasprm[2] = -damping
  actuator_biasprm[biasprm_id, actid] = new_biasprm


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


def set_const_fixed(m: types.Model, d: types.Data):
  """Compute fixed quantities (independent of qpos0).

  Computes:
    - body_subtreemass: mass of body and all descendants (depends on body_mass)

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
  """
  nworld_subtreemass = m.body_subtreemass.shape[0]
  wp.launch(_init_subtreemass, dim=(nworld_subtreemass, m.nbody), inputs=[m.body_mass], outputs=[m.body_subtreemass])
  for i in reversed(range(len(m.body_tree))):
    body_tree = m.body_tree[i]
    wp.launch(
      _accumulate_subtreemass,
      dim=(nworld_subtreemass, body_tree.size),
      inputs=[m.body_parentid, m.body_subtreemass, body_tree],
    )


def set_const_0(m: types.Model, d: types.Data, restore: bool = True):
  """Compute quantities that depend on qpos0.

  Computes:
    - tendon_length0: tendon resting lengths
    - eq_data: connect/weld anchor data, recomputed so the constraint is
      satisfied at qpos0
    - dof_invweight0: inverse inertia for DOFs
    - body_invweight0: inverse spatial inertia for bodies
    - tendon_invweight0: inverse weight for tendons
    - cam_pos0, cam_poscom0, cam_mat0: camera references
    - light_pos0, light_poscom0, light_dir0: light references
    - actuator_acc0: acceleration from unit actuator force
    - actuator_biasprm[2] (dampratio resolution): for position actuators where
      gainprm[0] == -biasprm[1] and biasprm[2] > 0, converts dampratio to
      damping via biasprm[2] = -dampratio * 2 * sqrt(kp * reflected_mass)

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    restore: Whether to restore state fields to correspond to d.qpos.
  """
  qpos_saved = wp.clone(d.qpos)

  wp.launch(_copy_qpos0_to_qpos, dim=(d.nworld, m.nq), inputs=[m.qpos0], outputs=[d.qpos])

  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.camlight(m, d)
  smooth.flex(m, d)
  smooth.tendon(m, d)
  smooth.crb(m, d)
  smooth.tendon_armature(m, d)
  smooth.factor_m(m, d)
  smooth.transmission(m, d)

  # Compute meaninertia from M diagonal at qpos0
  wp.launch(
    _compute_meaninertia,
    dim=m.stat.meaninertia.shape[0],
    inputs=[m.nv, m.M_rownnz, m.M_rowadr, d.M],
    outputs=[m.stat.meaninertia],
  )

  wp.launch(_copy_tendon_length0, dim=(m.tendon_length0.shape[0], m.ntendon), inputs=[d.ten_length], outputs=[m.tendon_length0])

  wp.launch(
    _compute_eq_data0,
    dim=(m.eq_data.shape[0], m.neq),
    inputs=[m.eq_type, m.eq_obj1id, m.eq_obj2id, m.eq_objtype, d.xpos, d.xquat, d.xmat],
    outputs=[m.eq_data],
  )

  # dof_invweight0: computed per joint with averaging for multi-DOF joints
  # FREE: 6 DOFs, trans gets mean(A[0:3]), rot gets mean(A[3:6])
  # BALL: 3 DOFs, all get mean(A[0:3])
  # HINGE/SLIDE: 1 DOF, gets A[0,0]
  if m.nv > 0:
    unit_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    result_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    dof_A_diag = wp.zeros((d.nworld, m.nv), dtype=float)

    # TODO(team): more efficient approach instead of looping over nv?
    for dofid in range(m.nv):
      wp.launch(_set_unit_vector, dim=d.nworld, inputs=[dofid], outputs=[unit_vec])
      smooth.solve_m(m, d, result_vec, unit_vec)
      wp.launch(_extract_dof_A_diag, dim=d.nworld, inputs=[dofid, result_vec], outputs=[dof_A_diag])

    wp.launch(
      _finalize_dof_invweight0,
      dim=(m.dof_invweight0.shape[0], m.nv),
      inputs=[m.dof_jntid, m.jnt_type, m.jnt_dofadr, dof_A_diag],
      outputs=[m.dof_invweight0],
    )

  # body_invweight0: computed as mean diagonal of J * inv(M) * J'
  # where J is the 6xnv body Jacobian (3 rows translation, 3 rows rotation)
  if m.nv > 0:
    body_jac_row = wp.zeros((d.nworld, m.nv), dtype=float)
    body_result_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    body_A_diag = wp.zeros((d.nworld, m.nbody, 6), dtype=float)

    # TODO(team): more efficient approach instead of nested iterations?
    for bodyid in range(1, m.nbody):
      for row_idx in range(6):
        wp.launch(
          _compute_body_jac_row,
          dim=d.nworld,
          inputs=[
            m.nv,
            bodyid,
            row_idx,
            m.body_parentid,
            m.body_rootid,
            m.body_dofadr,
            m.body_dofnum,
            m.dof_parentid,
            d.subtree_com,
            d.xipos,
            d.cdof,
          ],
          outputs=[body_jac_row],
        )
        smooth.solve_m(m, d, body_result_vec, body_jac_row)
        wp.launch(
          _compute_body_A_diag_entry,
          dim=d.nworld,
          inputs=[m.nv, bodyid, row_idx, body_jac_row, body_result_vec],
          outputs=[body_A_diag],
        )

    wp.launch(
      _finalize_body_invweight0,
      dim=(m.body_invweight0.shape[0], m.nbody),
      inputs=[m.body_weldid, body_A_diag],
      outputs=[m.body_invweight0],
    )
  else:
    m.body_invweight0.zero_()

  # tendon_invweight0[t] = J_t * inv(M) * J_t'
  if m.ntendon > 0:
    ten_J_vec = wp.empty((d.nworld, m.nv), dtype=float)
    ten_result_vec = wp.empty((d.nworld, m.nv), dtype=float)

    for tenid in range(m.ntendon):
      ten_J_vec.zero_()
      wp.launch(
        _copy_tendon_jacobian,
        dim=d.nworld,
        inputs=[tenid, m.ten_J_rownnz, m.ten_J_rowadr, m.ten_J_colind, d.ten_J],
        outputs=[ten_J_vec],
      )
      smooth.solve_m(m, d, ten_result_vec, ten_J_vec)
      wp.launch(
        _compute_tendon_dot_product,
        dim=m.tendon_invweight0.shape[0],
        inputs=[m.ten_J_rownnz, m.ten_J_rowadr, m.ten_J_colind, tenid, d.ten_J, ten_result_vec],
        outputs=[m.tendon_invweight0],
      )

  nworld_cam = np.max([m.cam_pos0.shape[0], m.cam_poscom0.shape[0], m.cam_mat0.shape[0]])
  wp.launch(
    _compute_cam_pos0,
    dim=(nworld_cam, m.ncam),
    inputs=[m.cam_bodyid, m.cam_targetbodyid, d.cam_xpos, d.cam_xmat, d.xpos, d.subtree_com],
    outputs=[m.cam_pos0, m.cam_poscom0, m.cam_mat0],
  )

  nworld_light = np.max([m.light_pos0.shape[0], m.light_poscom0.shape[0], m.light_dir0.shape[0]])
  wp.launch(
    _compute_light_pos0,
    dim=(nworld_light, m.nlight),
    inputs=[m.light_bodyid, m.light_targetbodyid, d.light_xpos, d.light_xdir, d.xpos, d.subtree_com],
    outputs=[m.light_pos0, m.light_poscom0, m.light_dir0],
  )

  # actuator_acc0[i] = ||inv(M) * actuator_moment[i]|| - acceleration from unit actuator force
  if m.nu > 0 and m.nv > 0:
    act_moment_vec = wp.zeros((d.nworld, m.nv), dtype=float)
    act_result_vec = wp.zeros((d.nworld, m.nv), dtype=float)

    for actid in range(m.nu):
      wp.launch(
        _copy_actuator_moment,
        dim=d.nworld,
        inputs=[actid, d.moment_rownnz, d.moment_rowadr, d.moment_colind, d.actuator_moment],
        outputs=[act_moment_vec],
      )
      smooth.solve_m(m, d, act_result_vec, act_moment_vec)
      wp.launch(
        _compute_actuator_acc0, dim=m.actuator_acc0.shape[0], inputs=[actid, m.nv, act_result_vec], outputs=[m.actuator_acc0]
      )

  # resolve dampratio: compute dof_M0, then convert dampratio to damping
  if m.nu > 0 and m.nv > 0:
    dof_M0 = wp.zeros((d.nworld, m.nv), dtype=float)
    wp.launch(
      _compute_dof_M0,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_bodyid, m.dof_armature, d.cdof, d.crb],
      outputs=[dof_M0],
    )
    wp.launch(
      _resolve_dampratio,
      dim=(m.actuator_biasprm.shape[0], m.nu),
      inputs=[
        m.actuator_biastype,
        m.actuator_gainprm,
        d.moment_rownnz,
        d.moment_rowadr,
        d.moment_colind,
        d.actuator_moment,
        dof_M0,
        m.nv,
      ],
      outputs=[m.actuator_biasprm],
    )

  wp.copy(d.qpos, qpos_saved)

  if restore:
    smooth.kinematics(m, d)
    smooth.com_pos(m, d)
    smooth.camlight(m, d)
    smooth.flex(m, d)
    smooth.tendon(m, d)
    smooth.crb(m, d)
    smooth.tendon_armature(m, d)
    smooth.factor_m(m, d)
    smooth.transmission(m, d)


def set_const_spring(m: types.Model, d: types.Data, restore: bool = True):
  """Compute quantities that depend on qpos_spring.

  Computes:
    - tendon_lengthspring: spring resting length range
  """
  if m.ntendon == 0:
    return

  qpos_saved = wp.clone(d.qpos)

  wp.launch(_copy_qpos0_to_qpos, dim=(d.nworld, m.nq), inputs=[m.qpos_spring], outputs=[d.qpos])

  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.tendon(m, d)
  smooth.transmission(m, d)

  wp.launch(
    _resolve_tendon_lengthspring,
    dim=(m.tendon_lengthspring.shape[0], m.ntendon),
    inputs=[d.ten_length],
    outputs=[m.tendon_lengthspring],
  )

  wp.copy(d.qpos, qpos_saved)

  if restore:
    smooth.kinematics(m, d)
    smooth.com_pos(m, d)
    smooth.tendon(m, d)
    smooth.transmission(m, d)


def set_const(m: types.Model, d: types.Data, restore: bool = True):
  """Recomputes qpos0-dependent constant model fields.

  This function propagates changes from some model fields to derived fields,
  allowing modifications that would otherwise be unsafe. It should be called
  after modifying model parameters at runtime.

  Model fields that can be modified safely with set_const:

  ==================================  ==============================================
  Field                               Notes
  ==================================  ==============================================
  qpos0, qpos_spring
  body_mass, body_inertia,            Mass and inertia are usually scaled together
  body_ipos, body_iquat               since inertia is sum(m * r^2).
  body_pos, body_quat                 Unsafe for static bodies (invalidates BVH).
  body_gravcomp                       If changing from 0 to >0 bodies, required.
  dof_armature
  eq_data                             For connect/weld, offsets computed if not set.
  hfield_size
  tendon_stiffness, tendon_damping    Only if changing from/to zero.
  actuator_gainprm, actuator_biasprm  For position actuators with dampratio.
  ==================================  ==============================================

  For selective updates, use the sub-functions directly based on what changed:

  ==============  ===============
  Modified Field  Call
  ==============  ===============
  body_mass       set_const
  body_gravcomp   set_const_fixed
  body_inertia    set_const_0
  qpos0           set_const_0
  ==============  ===============

  Computes:
    - Fixed quantities (via set_const_fixed):
      - body_subtreemass: mass of body and all descendants
    - qpos0-dependent quantities (via set_const_0):
      - tendon_length0: tendon resting lengths
      - dof_invweight0: inverse inertia for DOFs
      - body_invweight0: inverse spatial inertia for bodies
      - tendon_invweight0: inverse weight for tendons
      - cam_pos0, cam_poscom0, cam_mat0: camera references
      - light_pos0, light_poscom0, light_dir0: light references
      - actuator_acc0: acceleration from unit actuator force
      - actuator_biasprm[2] (dampratio resolution)

  Skips: actuator_length0 (not in mjwarp).

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    restore: Whether to restore state fields to correspond to d.qpos.
  """
  set_const_fixed(m, d)
  set_const_0(m, d, restore=False)
  set_const_spring(m, d, restore=False)

  if restore:
    smooth.kinematics(m, d)
    smooth.com_pos(m, d)
    smooth.camlight(m, d)
    smooth.flex(m, d)
    smooth.tendon(m, d)
    smooth.crb(m, d)
    smooth.tendon_armature(m, d)
    smooth.factor_m(m, d)
    smooth.transmission(m, d)


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
  if m.nu == 0:
    return

  wp.launch(
    _set_length_range,
    dim=(d.nworld, m.nu),
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
