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
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.types import ConstraintType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec11
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.config.enable_backward = False


@wp.kernel
def zero_constraint_counts(
  # Data out:
  ne_out: wp.array(dtype=int),
  ne_connect_out: wp.array(dtype=int),
  ne_weld_out: wp.array(dtype=int),
  ne_jnt_out: wp.array(dtype=int),
  ne_ten_out: wp.array(dtype=int),
  nf_out: wp.array(dtype=int),
  nl_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
):
  worldid = wp.tid()

  # Zero all constraint counters
  ne_out[worldid] = 0
  ne_connect_out[worldid] = 0
  ne_weld_out[worldid] = 0
  ne_jnt_out[worldid] = 0
  ne_ten_out[worldid] = 0
  nf_out[worldid] = 0
  nl_out[worldid] = 0
  nefc_out[worldid] = 0


@wp.func
def _update_efc_row(
  # In:
  worldid: int,
  timestep: float,
  refsafe: int,
  efcid: int,
  pos_aref: float,
  pos_imp: float,
  invweight: float,
  solref: wp.vec2,
  solimp: vec5,
  margin: float,
  vel: float,
  frictionloss: float,
  type: int,
  id: int,
  # Data out:
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  # Calculate kbi
  timeconst = solref[0]
  dampratio = solref[1]
  dmin = solimp[0]
  dmax = solimp[1]
  width = solimp[2]
  mid = solimp[3]
  power = solimp[4]

  # TODO(team): wp.static?
  if not refsafe:
    timeconst = wp.max(timeconst, 2.0 * timestep)

  dmin = wp.clamp(dmin, types.MJ_MINIMP, types.MJ_MAXIMP)
  dmax = wp.clamp(dmax, types.MJ_MINIMP, types.MJ_MAXIMP)
  width = wp.max(types.MJ_MINVAL, width)
  mid = wp.clamp(mid, types.MJ_MINIMP, types.MJ_MAXIMP)
  power = wp.max(1.0, power)

  # See https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters
  k = 1.0 / (dmax * dmax * timeconst * timeconst * dampratio * dampratio)
  b = 2.0 / (dmax * timeconst)
  k = wp.where(solref[0] <= 0, -solref[0] / (dmax * dmax), k)
  b = wp.where(solref[1] <= 0, -solref[1] / dmax, b)

  imp_x = wp.abs(pos_imp) / width
  imp_a = (1.0 / wp.pow(mid, power - 1.0)) * wp.pow(imp_x, power)
  imp_b = 1.0 - (1.0 / wp.pow(1.0 - mid, power - 1.0)) * wp.pow(1.0 - imp_x, power)
  imp_y = wp.where(imp_x < mid, imp_a, imp_b)
  imp = dmin + imp_y * (dmax - dmin)
  imp = wp.clamp(imp, dmin, dmax)
  imp = wp.where(imp_x > 1.0, dmax, imp)

  # Update constraints
  efc_D_out[worldid, efcid] = 1.0 / wp.max(invweight * (1.0 - imp) / imp, types.MJ_MINVAL)
  efc_vel_out[worldid, efcid] = vel
  efc_aref_out[worldid, efcid] = -k * imp * pos_aref - b * vel
  efc_pos_out[worldid, efcid] = pos_aref + margin
  efc_margin_out[worldid, efcid] = margin
  efc_frictionloss_out[worldid, efcid] = frictionloss
  efc_type_out[worldid, efcid] = type
  efc_id_out[worldid, efcid] = id


@wp.kernel
def _efc_equality_connect(
  # Model:
  nv: int,
  nsite: int,
  opt_timestep: wp.array(dtype=float),
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_invweight0: wp.array2d(dtype=wp.vec2),
  dof_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  eq_obj1id: wp.array(dtype=int),
  eq_obj2id: wp.array(dtype=int),
  eq_objtype: wp.array(dtype=int),
  eq_solref: wp.array2d(dtype=wp.vec2),
  eq_solimp: wp.array2d(dtype=vec5),
  eq_data: wp.array2d(dtype=vec11),
  eq_connect_adr: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  qvel_in: wp.array2d(dtype=float),
  eq_active_in: wp.array2d(dtype=bool),
  xpos_in: wp.array2d(dtype=wp.vec3),
  xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  refsafe_in: int,
  # Data out:
  ne_connect_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  """Calculates constraint rows for connect equality constraints."""

  worldid, i_eq_connect_adr = wp.tid()
  timestep = opt_timestep[worldid]
  i_eq = eq_connect_adr[i_eq_connect_adr]

  if not eq_active_in[worldid, i_eq]:
    return

  wp.atomic_add(ne_connect_out, worldid, 3)
  efcid = wp.atomic_add(nefc_out, worldid, 3)

  if efcid + 3 >= njmax_in:
    return

  data = eq_data[worldid, i_eq]
  anchor1 = wp.vec3f(data[0], data[1], data[2])
  anchor2 = wp.vec3f(data[3], data[4], data[5])

  obj1id = eq_obj1id[i_eq]
  obj2id = eq_obj2id[i_eq]

  if nsite and eq_objtype[i_eq] == wp.static(types.ObjType.SITE.value):
    # body1id stores the index of site_bodyid.
    body1id = site_bodyid[obj1id]
    body2id = site_bodyid[obj2id]
    pos1 = site_xpos_in[worldid, obj1id]
    pos2 = site_xpos_in[worldid, obj2id]
  else:
    body1id = obj1id
    body2id = obj2id
    pos1 = xpos_in[worldid, body1id] + xmat_in[worldid, body1id] @ anchor1
    pos2 = xpos_in[worldid, body2id] + xmat_in[worldid, body2id] @ anchor2

  # error is difference in global positions
  pos = pos1 - pos2

  # compute Jacobian difference (opposite of contact: 0 - 1)
  Jqvel = wp.vec3f(0.0, 0.0, 0.0)
  for dofid in range(nv):  # TODO: parallelize
    jacp1, _ = support.jac(
      body_parentid,
      body_rootid,
      dof_bodyid,
      subtree_com_in,
      cdof_in,
      pos1,
      body1id,
      dofid,
      worldid,
    )
    jacp2, _ = support.jac(
      body_parentid,
      body_rootid,
      dof_bodyid,
      subtree_com_in,
      cdof_in,
      pos2,
      body2id,
      dofid,
      worldid,
    )
    j1mj2 = jacp1 - jacp2
    efc_J_out[worldid, efcid + 0, dofid] = j1mj2[0]
    efc_J_out[worldid, efcid + 1, dofid] = j1mj2[1]
    efc_J_out[worldid, efcid + 2, dofid] = j1mj2[2]
    Jqvel += j1mj2 * qvel_in[worldid, dofid]

  invweight = body_invweight0[worldid, body1id][0] + body_invweight0[worldid, body2id][0]
  pos_imp = wp.length(pos)

  solref = eq_solref[worldid, i_eq]
  solimp = eq_solimp[worldid, i_eq]

  for i in range(3):
    efcidi = efcid + i

    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcidi,
      pos[i],
      pos_imp,
      invweight,
      solref,
      solimp,
      0.0,
      Jqvel[i],
      0.0,
      ConstraintType.EQUALITY.value,
      i_eq,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _efc_equality_joint(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  qpos0: wp.array2d(dtype=float),
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  dof_invweight0: wp.array2d(dtype=float),
  eq_obj1id: wp.array(dtype=int),
  eq_obj2id: wp.array(dtype=int),
  eq_solref: wp.array2d(dtype=wp.vec2),
  eq_solimp: wp.array2d(dtype=vec5),
  eq_data: wp.array2d(dtype=vec11),
  eq_jnt_adr: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  qpos_in: wp.array2d(dtype=float),
  qvel_in: wp.array2d(dtype=float),
  eq_active_in: wp.array2d(dtype=bool),
  # In:
  refsafe_in: int,
  # Data out:
  ne_jnt_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, i_eq_joint_adr = wp.tid()
  timestep = opt_timestep[worldid]
  i_eq = eq_jnt_adr[i_eq_joint_adr]
  if not eq_active_in[worldid, i_eq]:
    return

  wp.atomic_add(ne_jnt_out, worldid, 1)
  efcid = wp.atomic_add(nefc_out, worldid, 1)

  if efcid >= njmax_in:
    return

  for i in range(nv):
    efc_J_out[worldid, efcid, i] = 0.0

  jntid_1 = eq_obj1id[i_eq]
  jntid_2 = eq_obj2id[i_eq]
  data = eq_data[worldid, i_eq]
  dofadr1 = jnt_dofadr[jntid_1]
  qposadr1 = jnt_qposadr[jntid_1]
  efc_J_out[worldid, efcid, dofadr1] = 1.0

  if jntid_2 > -1:
    # Two joint constraint
    qposadr2 = jnt_qposadr[jntid_2]
    dofadr2 = jnt_dofadr[jntid_2]
    dif = qpos_in[worldid, qposadr2] - qpos0[worldid, qposadr2]

    # Horner's method for polynomials
    rhs = data[0] + dif * (data[1] + dif * (data[2] + dif * (data[3] + dif * data[4])))
    deriv_2 = data[1] + dif * (2.0 * data[2] + dif * (3.0 * data[3] + dif * 4.0 * data[4]))

    pos = qpos_in[worldid, qposadr1] - qpos0[worldid, qposadr1] - rhs
    Jqvel = qvel_in[worldid, dofadr1] - qvel_in[worldid, dofadr2] * deriv_2
    invweight = dof_invweight0[worldid, dofadr1] + dof_invweight0[worldid, dofadr2]

    efc_J_out[worldid, efcid, dofadr2] = -deriv_2
  else:
    # Single joint constraint
    pos = qpos_in[worldid, qposadr1] - qpos0[worldid, qposadr1] - data[0]
    Jqvel = qvel_in[worldid, dofadr1]
    invweight = dof_invweight0[worldid, dofadr1]

  # Update constraint parameters
  _update_efc_row(
    worldid,
    timestep,
    refsafe_in,
    efcid,
    pos,
    pos,
    invweight,
    eq_solref[worldid, i_eq],
    eq_solimp[worldid, i_eq],
    0.0,
    Jqvel,
    0.0,
    ConstraintType.EQUALITY.value,
    i_eq,
    efc_type_out,
    efc_id_out,
    efc_pos_out,
    efc_margin_out,
    efc_D_out,
    efc_vel_out,
    efc_aref_out,
    efc_frictionloss_out,
  )


@wp.kernel
def _efc_equality_tendon(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  eq_obj1id: wp.array(dtype=int),
  eq_obj2id: wp.array(dtype=int),
  eq_solref: wp.array2d(dtype=wp.vec2),
  eq_solimp: wp.array2d(dtype=vec5),
  eq_data: wp.array2d(dtype=vec11),
  eq_ten_adr: wp.array(dtype=int),
  tendon_length0: wp.array2d(dtype=float),
  tendon_invweight0: wp.array2d(dtype=float),
  # Data in:
  njmax_in: int,
  qvel_in: wp.array2d(dtype=float),
  eq_active_in: wp.array2d(dtype=bool),
  ten_length_in: wp.array2d(dtype=float),
  ten_J_in: wp.array3d(dtype=float),
  # In:
  refsafe_in: int,
  # Data out:
  ne_ten_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, tenid = wp.tid()
  timestep = opt_timestep[worldid]
  eqid = eq_ten_adr[tenid]

  if not eq_active_in[worldid, eqid]:
    return

  wp.atomic_add(ne_ten_out, worldid, 1)
  efcid = wp.atomic_add(nefc_out, worldid, 1)

  if efcid >= njmax_in:
    return

  obj1id = eq_obj1id[eqid]
  obj2id = eq_obj2id[eqid]

  data = eq_data[worldid, eqid]
  solref = eq_solref[worldid, eqid]
  solimp = eq_solimp[worldid, eqid]
  pos1 = ten_length_in[worldid, obj1id] - tendon_length0[worldid, obj1id]
  jac1 = ten_J_in[worldid, obj1id]

  if obj2id > -1:
    invweight = tendon_invweight0[worldid, obj1id] + tendon_invweight0[worldid, obj2id]

    pos2 = ten_length_in[worldid, obj2id] - tendon_length0[worldid, obj2id]
    jac2 = ten_J_in[worldid, obj2id]

    dif = pos2
    dif2 = dif * dif
    dif3 = dif2 * dif
    dif4 = dif3 * dif

    pos = pos1 - (data[0] + data[1] * dif + data[2] * dif2 + data[3] * dif3 + data[4] * dif4)
    deriv = data[1] + 2.0 * data[2] * dif + 3.0 * data[3] * dif2 + 4.0 * data[4] * dif3
  else:
    invweight = tendon_invweight0[worldid, obj1id]
    pos = pos1 - data[0]
    deriv = 0.0

  Jqvel = float(0.0)
  for i in range(nv):
    if deriv != 0.0:
      J = jac1[i] + jac2[i] * -deriv
    else:
      J = jac1[i]
    efc_J_out[worldid, efcid, i] = J
    Jqvel += J * qvel_in[worldid, i]

  _update_efc_row(
    worldid,
    timestep,
    refsafe_in,
    efcid,
    pos,
    pos,
    invweight,
    solref,
    solimp,
    0.0,
    Jqvel,
    0.0,
    ConstraintType.EQUALITY.value,
    eqid,
    efc_type_out,
    efc_id_out,
    efc_pos_out,
    efc_margin_out,
    efc_D_out,
    efc_vel_out,
    efc_aref_out,
    efc_frictionloss_out,
  )


@wp.kernel
def _efc_friction_dof(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  dof_invweight0: wp.array2d(dtype=float),
  dof_frictionloss: wp.array2d(dtype=float),
  dof_solimp: wp.array2d(dtype=vec5),
  dof_solref: wp.array2d(dtype=wp.vec2),
  # Data in:
  njmax_in: int,
  qvel_in: wp.array2d(dtype=float),
  # In:
  refsafe_in: int,
  # Data out:
  nf_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  timestep = opt_timestep[worldid]

  if dof_frictionloss[worldid, dofid] <= 0.0:
    return

  efcid = wp.atomic_add(nefc_out, worldid, 1)

  if efcid >= njmax_in:
    return

  wp.atomic_add(nf_out, worldid, 1)

  for i in range(nv):
    efc_J_out[worldid, efcid, i] = 0.0

  efc_J_out[worldid, efcid, dofid] = 1.0
  Jqvel = qvel_in[worldid, dofid]

  _update_efc_row(
    worldid,
    timestep,
    refsafe_in,
    efcid,
    0.0,
    0.0,
    dof_invweight0[worldid, dofid],
    dof_solref[worldid, dofid],
    dof_solimp[worldid, dofid],
    0.0,
    Jqvel,
    dof_frictionloss[worldid, dofid],
    ConstraintType.FRICTION_DOF.value,
    dofid,
    efc_type_out,
    efc_id_out,
    efc_pos_out,
    efc_margin_out,
    efc_D_out,
    efc_vel_out,
    efc_aref_out,
    efc_frictionloss_out,
  )


@wp.kernel
def _efc_friction_tendon(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  tendon_solref_fri: wp.array2d(dtype=wp.vec2),
  tendon_solimp_fri: wp.array2d(dtype=vec5),
  tendon_frictionloss: wp.array2d(dtype=float),
  tendon_invweight0: wp.array2d(dtype=float),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  ten_J_in: wp.array3d(dtype=float),
  # In:
  refsafe_in: int,
  # Data out:
  nf_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, tenid = wp.tid()
  timestep = opt_timestep[worldid]

  frictionloss = tendon_frictionloss[worldid, tenid]
  if frictionloss <= 0.0:
    return

  efcid = wp.atomic_add(nefc_out, worldid, 1)
  wp.atomic_add(nf_out, worldid, 1)

  Jqvel = float(0.0)

  # TODO(team): parallelize
  for i in range(nv):
    J = ten_J_in[worldid, tenid, i]
    efc_J_out[worldid, efcid, i] = J
    Jqvel += J * qvel_in[worldid, i]

  _update_efc_row(
    worldid,
    timestep,
    refsafe_in,
    efcid,
    0.0,
    0.0,
    tendon_invweight0[worldid, tenid],
    tendon_solref_fri[worldid, tenid],
    tendon_solimp_fri[worldid, tenid],
    0.0,
    Jqvel,
    frictionloss,
    ConstraintType.FRICTION_TENDON.value,
    tenid,
    efc_type_out,
    efc_id_out,
    efc_pos_out,
    efc_margin_out,
    efc_D_out,
    efc_vel_out,
    efc_aref_out,
    efc_frictionloss_out,
  )


@wp.kernel
def _efc_equality_weld(
  # Model:
  nv: int,
  nsite: int,
  opt_timestep: wp.array(dtype=float),
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_invweight0: wp.array2d(dtype=wp.vec2),
  dof_bodyid: wp.array(dtype=int),
  site_bodyid: wp.array(dtype=int),
  site_quat: wp.array2d(dtype=wp.quat),
  eq_obj1id: wp.array(dtype=int),
  eq_obj2id: wp.array(dtype=int),
  eq_objtype: wp.array(dtype=int),
  eq_solref: wp.array2d(dtype=wp.vec2),
  eq_solimp: wp.array2d(dtype=vec5),
  eq_data: wp.array2d(dtype=vec11),
  eq_wld_adr: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  qvel_in: wp.array2d(dtype=float),
  eq_active_in: wp.array2d(dtype=bool),
  xpos_in: wp.array2d(dtype=wp.vec3),
  xquat_in: wp.array2d(dtype=wp.quat),
  xmat_in: wp.array2d(dtype=wp.mat33),
  site_xpos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  refsafe_in: int,
  # Data out:
  ne_weld_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, i_eq_weld_adr = wp.tid()
  timestep = opt_timestep[worldid]
  i_eq = eq_wld_adr[i_eq_weld_adr]
  if not eq_active_in[worldid, i_eq]:
    return

  wp.atomic_add(ne_weld_out, worldid, 6)
  efcid = wp.atomic_add(nefc_out, worldid, 6)

  if efcid + 6 >= njmax_in:
    return

  is_site = eq_objtype[i_eq] == wp.static(types.ObjType.SITE.value) and nsite > 0

  obj1id = eq_obj1id[i_eq]
  obj2id = eq_obj2id[i_eq]

  data = eq_data[worldid, i_eq]
  anchor1 = wp.vec3(data[0], data[1], data[2])
  anchor2 = wp.vec3(data[3], data[4], data[5])
  relpose = wp.quat(data[6], data[7], data[8], data[9])
  torquescale = data[10]

  if is_site:
    # body1id stores the index of site_bodyid.
    body1id = site_bodyid[obj1id]
    body2id = site_bodyid[obj2id]
    pos1 = site_xpos_in[worldid, obj1id]
    pos2 = site_xpos_in[worldid, obj2id]

    quat = math.mul_quat(xquat_in[worldid, body1id], site_quat[worldid, obj1id])
    quat1 = math.quat_inv(math.mul_quat(xquat_in[worldid, body2id], site_quat[worldid, obj2id]))

  else:
    body1id = obj1id
    body2id = obj2id
    pos1 = xpos_in[worldid, body1id] + xmat_in[worldid, body1id] @ anchor2
    pos2 = xpos_in[worldid, body2id] + xmat_in[worldid, body2id] @ anchor1

    quat = math.mul_quat(xquat_in[worldid, body1id], relpose)
    quat1 = math.quat_inv(xquat_in[worldid, body2id])

  # compute Jacobian difference (opposite of contact: 0 - 1)
  Jqvelp = wp.vec3f(0.0, 0.0, 0.0)
  Jqvelr = wp.vec3f(0.0, 0.0, 0.0)

  for dofid in range(nv):  # TODO: parallelize
    jacp1, jacr1 = support.jac(
      body_parentid,
      body_rootid,
      dof_bodyid,
      subtree_com_in,
      cdof_in,
      pos1,
      body1id,
      dofid,
      worldid,
    )
    jacp2, jacr2 = support.jac(
      body_parentid,
      body_rootid,
      dof_bodyid,
      subtree_com_in,
      cdof_in,
      pos2,
      body2id,
      dofid,
      worldid,
    )

    jacdifp = jacp1 - jacp2
    for i in range(wp.static(3)):
      efc_J_out[worldid, efcid + i, dofid] = jacdifp[i]

    jacdifr = (jacr1 - jacr2) * torquescale
    jacdifrq = math.mul_quat(math.quat_mul_axis(quat1, jacdifr), quat)
    jacdifr = 0.5 * wp.vec3(jacdifrq[1], jacdifrq[2], jacdifrq[3])

    for i in range(wp.static(3)):
      efc_J_out[worldid, efcid + 3 + i, dofid] = jacdifr[i]

    Jqvelp += jacdifp * qvel_in[worldid, dofid]
    Jqvelr += jacdifr * qvel_in[worldid, dofid]

  # error is difference in global position and orientation
  cpos = pos1 - pos2

  crotq = math.mul_quat(quat1, quat)  # copy axis components
  crot = wp.vec3(crotq[1], crotq[2], crotq[3]) * torquescale

  invweight_t = body_invweight0[worldid, body1id][0] + body_invweight0[worldid, body2id][0]

  pos_imp = wp.sqrt(wp.length_sq(cpos) + wp.length_sq(crot))

  solref = eq_solref[worldid, i_eq]
  solimp = eq_solimp[worldid, i_eq]

  for i in range(3):
    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid + i,
      cpos[i],
      pos_imp,
      invweight_t,
      solref,
      solimp,
      0.0,
      Jqvelp[i],
      0.0,
      ConstraintType.EQUALITY.value,
      i_eq,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )

  invweight_r = body_invweight0[worldid, body1id][1] + body_invweight0[worldid, body2id][1]

  for i in range(3):
    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid + 3 + i,
      crot[i],
      pos_imp,
      invweight_r,
      solref,
      solimp,
      0.0,
      Jqvelr[i],
      0.0,
      ConstraintType.EQUALITY.value,
      i_eq,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _efc_limit_slide_hinge(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  jnt_solref: wp.array2d(dtype=wp.vec2),
  jnt_solimp: wp.array2d(dtype=vec5),
  jnt_range: wp.array2d(dtype=wp.vec2),
  jnt_margin: wp.array2d(dtype=float),
  jnt_limited_slide_hinge_adr: wp.array(dtype=int),
  dof_invweight0: wp.array2d(dtype=float),
  # Data in:
  njmax_in: int,
  qpos_in: wp.array2d(dtype=float),
  qvel_in: wp.array2d(dtype=float),
  # In:
  refsafe_in: int,
  # Data out:
  nl_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, jntlimitedid = wp.tid()
  timestep = opt_timestep[worldid]
  jntid = jnt_limited_slide_hinge_adr[jntlimitedid]
  jntrange = jnt_range[worldid, jntid]

  qpos = qpos_in[worldid, jnt_qposadr[jntid]]
  jntmargin = jnt_margin[worldid, jntid]
  dist_min, dist_max = qpos - jntrange[0], jntrange[1] - qpos
  pos = wp.min(dist_min, dist_max) - jntmargin
  active = pos < 0

  if active:
    wp.atomic_add(nl_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    for i in range(nv):
      efc_J_out[worldid, efcid, i] = 0.0

    dofadr = jnt_dofadr[jntid]

    J = float(dist_min < dist_max) * 2.0 - 1.0
    efc_J_out[worldid, efcid, dofadr] = J
    Jqvel = J * qvel_in[worldid, dofadr]

    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid,
      pos,
      pos,
      dof_invweight0[worldid, dofadr],
      jnt_solref[worldid, jntid],
      jnt_solimp[worldid, jntid],
      jntmargin,
      Jqvel,
      0.0,
      ConstraintType.LIMIT_JOINT.value,
      jntid,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _efc_limit_ball(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  jnt_solref: wp.array2d(dtype=wp.vec2),
  jnt_solimp: wp.array2d(dtype=vec5),
  jnt_range: wp.array2d(dtype=wp.vec2),
  jnt_margin: wp.array2d(dtype=float),
  jnt_limited_ball_adr: wp.array(dtype=int),
  dof_invweight0: wp.array2d(dtype=float),
  # Data in:
  njmax_in: int,
  qpos_in: wp.array2d(dtype=float),
  qvel_in: wp.array2d(dtype=float),
  # In:
  refsafe_in: int,
  # Data out:
  nl_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, jntlimitedid = wp.tid()
  timestep = opt_timestep[worldid]
  jntid = jnt_limited_ball_adr[jntlimitedid]
  qposadr = jnt_qposadr[jntid]

  qpos = qpos_in[worldid]
  jnt_quat = wp.quat(qpos[qposadr + 0], qpos[qposadr + 1], qpos[qposadr + 2], qpos[qposadr + 3])
  jnt_quat = wp.normalize(jnt_quat)
  axis_angle = math.quat_to_vel(jnt_quat)
  jntrange = jnt_range[worldid, jntid]
  axis, angle = math.normalize_with_norm(axis_angle)
  jntmargin = jnt_margin[worldid, jntid]

  pos = wp.max(jntrange[0], jntrange[1]) - angle - jntmargin
  active = pos < 0

  if active:
    wp.atomic_add(nl_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    for i in range(nv):
      efc_J_out[worldid, efcid, i] = 0.0

    dofadr = jnt_dofadr[jntid]

    efc_J_out[worldid, efcid, dofadr + 0] = -axis[0]
    efc_J_out[worldid, efcid, dofadr + 1] = -axis[1]
    efc_J_out[worldid, efcid, dofadr + 2] = -axis[2]

    Jqvel = -axis[0] * qvel_in[worldid, dofadr + 0]
    Jqvel -= axis[1] * qvel_in[worldid, dofadr + 1]
    Jqvel -= axis[2] * qvel_in[worldid, dofadr + 2]

    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid,
      pos,
      pos,
      dof_invweight0[worldid, dofadr],
      jnt_solref[worldid, jntid],
      jnt_solimp[worldid, jntid],
      jntmargin,
      Jqvel,
      0.0,
      ConstraintType.LIMIT_JOINT.value,
      jntid,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _efc_limit_tendon(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  jnt_dofadr: wp.array(dtype=int),
  tendon_adr: wp.array(dtype=int),
  tendon_num: wp.array(dtype=int),
  tendon_limited_adr: wp.array(dtype=int),
  tendon_solref_lim: wp.array2d(dtype=wp.vec2),
  tendon_solimp_lim: wp.array2d(dtype=vec5),
  tendon_range: wp.array2d(dtype=wp.vec2),
  tendon_margin: wp.array2d(dtype=float),
  tendon_invweight0: wp.array2d(dtype=float),
  wrap_objid: wp.array(dtype=int),
  wrap_type: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  qvel_in: wp.array2d(dtype=float),
  ten_length_in: wp.array2d(dtype=float),
  ten_J_in: wp.array3d(dtype=float),
  # In:
  refsafe_in: int,
  # Data out:
  nl_out: wp.array(dtype=int),
  nefc_out: wp.array(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  worldid, tenlimitedid = wp.tid()
  timestep = opt_timestep[worldid]
  tenid = tendon_limited_adr[tenlimitedid]

  tenrange = tendon_range[worldid, tenid]
  length = ten_length_in[worldid, tenid]
  dist_min, dist_max = length - tenrange[0], tenrange[1] - length
  tenmargin = tendon_margin[worldid, tenid]
  pos = wp.min(dist_min, dist_max) - tenmargin
  active = pos < 0

  if active:
    wp.atomic_add(nl_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    for i in range(nv):
      efc_J_out[worldid, efcid, i] = 0.0

    Jqvel = float(0.0)
    scl = float(dist_min < dist_max) * 2.0 - 1.0

    adr = tendon_adr[tenid]
    if wrap_type[adr] == wp.static(types.WrapType.JOINT.value):
      ten_num = tendon_num[tenid]
      for i in range(ten_num):
        dofadr = jnt_dofadr[wrap_objid[adr + i]]
        J = scl * ten_J_in[worldid, tenid, dofadr]
        efc_J_out[worldid, efcid, dofadr] = J
        Jqvel += J * qvel_in[worldid, dofadr]
    else:
      for i in range(nv):
        J = scl * ten_J_in[worldid, tenid, i]
        efc_J_out[worldid, efcid, i] = J
        Jqvel += J * qvel_in[worldid, i]

    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid,
      pos,
      pos,
      tendon_invweight0[worldid, tenid],
      tendon_solref_lim[worldid, tenid],
      tendon_solimp_lim[worldid, tenid],
      tenmargin,
      Jqvel,
      0.0,
      ConstraintType.LIMIT_TENDON.value,
      tenid,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _efc_contact_pyramidal(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  opt_impratio: wp.array(dtype=float),
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_invweight0: wp.array2d(dtype=wp.vec2),
  dof_bodyid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  ncon_in: wp.array(dtype=int),
  qvel_in: wp.array2d(dtype=float),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  refsafe_in: int,
  dist_in: wp.array(dtype=float),
  condim_in: wp.array(dtype=int),
  includemargin_in: wp.array(dtype=float),
  worldid_in: wp.array(dtype=int),
  geom_in: wp.array(dtype=wp.vec2i),
  pos_in: wp.array(dtype=wp.vec3),
  frame_in: wp.array(dtype=wp.mat33),
  friction_in: wp.array(dtype=vec5),
  solref_in: wp.array(dtype=wp.vec2),
  solimp_in: wp.array(dtype=vec5),
  # Data out:
  nefc_out: wp.array(dtype=int),
  contact_efc_address_out: wp.array2d(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  conid, dimid = wp.tid()

  if conid >= ncon_in[0]:
    return

  condim = condim_in[conid]

  if condim == 1 and dimid > 0:
    return
  elif condim > 1 and dimid >= 2 * (condim - 1):
    return

  includemargin = includemargin_in[conid]
  pos = dist_in[conid] - includemargin
  active = pos < 0

  if active:
    worldid = worldid_in[conid]
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    timestep = opt_timestep[worldid]
    impratio = opt_impratio[worldid]
    contact_efc_address_out[conid, dimid] = efcid

    geom = geom_in[conid]
    body1 = geom_bodyid[geom[0]]
    body2 = geom_bodyid[geom[1]]

    con_pos = pos_in[conid]
    frame = frame_in[conid]

    # pyramidal has common invweight across all edges
    invweight = body_invweight0[worldid, body1][0] + body_invweight0[worldid, body2][0]

    if condim > 1:
      dimid2 = dimid / 2 + 1

      friction = friction_in[conid]
      fri0 = friction[0]
      frii = friction[dimid2 - 1]
      invweight = invweight + fri0 * fri0 * invweight
      invweight = invweight * 2.0 * fri0 * fri0 / impratio

    Jqvel = float(0.0)
    for i in range(nv):
      J = float(0.0)
      Ji = float(0.0)
      jac1p, jac1r = support.jac(
        body_parentid,
        body_rootid,
        dof_bodyid,
        subtree_com_in,
        cdof_in,
        con_pos,
        body1,
        i,
        worldid,
      )
      jac2p, jac2r = support.jac(
        body_parentid,
        body_rootid,
        dof_bodyid,
        subtree_com_in,
        cdof_in,
        con_pos,
        body2,
        i,
        worldid,
      )
      jacp_dif = jac2p - jac1p
      for xyz in range(3):
        J += frame[0, xyz] * jacp_dif[xyz]

        if condim > 1:
          if dimid2 < 3:
            Ji += frame[dimid2, xyz] * jacp_dif[xyz]
          else:
            Ji += frame[dimid2 - 3, xyz] * (jac2r[xyz] - jac1r[xyz])

      if condim > 1:
        if dimid % 2 == 0:
          J += Ji * frii
        else:
          J -= Ji * frii

      efc_J_out[worldid, efcid, i] = J
      Jqvel += J * qvel_in[worldid, i]

    if condim == 1:
      efc_type = int(ConstraintType.CONTACT_FRICTIONLESS.value)
    else:
      efc_type = int(ConstraintType.CONTACT_PYRAMIDAL.value)

    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid,
      pos,
      pos,
      invweight,
      solref_in[conid],
      solimp_in[conid],
      includemargin,
      Jqvel,
      0.0,
      efc_type,
      conid,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _efc_contact_elliptic(
  # Model:
  nv: int,
  opt_timestep: wp.array(dtype=float),
  opt_impratio: wp.array(dtype=float),
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_invweight0: wp.array2d(dtype=wp.vec2),
  dof_bodyid: wp.array(dtype=int),
  geom_bodyid: wp.array(dtype=int),
  # Data in:
  njmax_in: int,
  ncon_in: wp.array(dtype=int),
  qvel_in: wp.array2d(dtype=float),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # In:
  refsafe_in: int,
  dist_in: wp.array(dtype=float),
  condim_in: wp.array(dtype=int),
  includemargin_in: wp.array(dtype=float),
  worldid_in: wp.array(dtype=int),
  geom_in: wp.array(dtype=wp.vec2i),
  pos_in: wp.array(dtype=wp.vec3),
  frame_in: wp.array(dtype=wp.mat33),
  friction_in: wp.array(dtype=vec5),
  solref_in: wp.array(dtype=wp.vec2),
  solreffriction_in: wp.array(dtype=wp.vec2),
  solimp_in: wp.array(dtype=vec5),
  # Data out:
  nefc_out: wp.array(dtype=int),
  contact_efc_address_out: wp.array2d(dtype=int),
  efc_type_out: wp.array2d(dtype=int),
  efc_id_out: wp.array2d(dtype=int),
  efc_J_out: wp.array3d(dtype=float),
  efc_pos_out: wp.array2d(dtype=float),
  efc_margin_out: wp.array2d(dtype=float),
  efc_D_out: wp.array2d(dtype=float),
  efc_vel_out: wp.array2d(dtype=float),
  efc_aref_out: wp.array2d(dtype=float),
  efc_frictionloss_out: wp.array2d(dtype=float),
):
  conid, dimid = wp.tid()

  if conid >= ncon_in[0]:
    return

  condim = condim_in[conid]

  if dimid > condim - 1:
    return

  includemargin = includemargin_in[conid]
  pos = dist_in[conid] - includemargin
  active = pos < 0.0

  if active:
    worldid = worldid_in[conid]
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    timestep = opt_timestep[worldid]
    impratio = opt_impratio[worldid]
    contact_efc_address_out[conid, dimid] = efcid

    geom = geom_in[conid]
    body1 = geom_bodyid[geom[0]]
    body2 = geom_bodyid[geom[1]]

    cpos = pos_in[conid]
    frame = frame_in[conid]

    # TODO(team): parallelize J and Jqvel computation?
    Jqvel = float(0.0)
    for i in range(nv):
      J = float(0.0)
      jac1p, jac1r = support.jac(
        body_parentid,
        body_rootid,
        dof_bodyid,
        subtree_com_in,
        cdof_in,
        cpos,
        body1,
        i,
        worldid,
      )
      jac2p, jac2r = support.jac(
        body_parentid,
        body_rootid,
        dof_bodyid,
        subtree_com_in,
        cdof_in,
        cpos,
        body2,
        i,
        worldid,
      )
      for xyz in range(3):
        if dimid < 3:
          jac_dif = jac2p[xyz] - jac1p[xyz]
          J += frame[dimid, xyz] * jac_dif
        else:
          jac_dif = jac2r[xyz] - jac1r[xyz]
          J += frame[dimid - 3, xyz] * jac_dif

      efc_J_out[worldid, efcid, i] = J
      Jqvel += J * qvel_in[worldid, i]

    invweight = body_invweight0[worldid, body1][0] + body_invweight0[worldid, body2][0]

    ref = solref_in[conid]
    pos_aref = pos

    if dimid > 0:
      solreffriction = solreffriction_in[conid]

      # non-normal directions use solreffriction (if non-zero)
      if solreffriction[0] or solreffriction[1]:
        ref = solreffriction

      # TODO(team): precompute 1 / impratio
      invweight = invweight / impratio
      friction = friction_in[conid]

      if dimid > 1:
        fri0 = friction[0]
        frii = friction[dimid - 1]
        fri = fri0 * fri0 / (frii * frii)
        invweight *= fri

      pos_aref = 0.0

    if condim == 1:
      efc_type = int(ConstraintType.CONTACT_FRICTIONLESS.value)
    else:
      efc_type = int(ConstraintType.CONTACT_ELLIPTIC.value)

    _update_efc_row(
      worldid,
      timestep,
      refsafe_in,
      efcid,
      pos_aref,
      pos,
      invweight,
      ref,
      solimp_in[conid],
      includemargin,
      Jqvel,
      0.0,
      efc_type,
      conid,
      efc_type_out,
      efc_id_out,
      efc_pos_out,
      efc_margin_out,
      efc_D_out,
      efc_vel_out,
      efc_aref_out,
      efc_frictionloss_out,
    )


@wp.kernel
def _num_equality(
  # Data in:
  ne_connect_in: wp.array(dtype=int),
  ne_weld_in: wp.array(dtype=int),
  ne_jnt_in: wp.array(dtype=int),
  ne_ten_in: wp.array(dtype=int),
  # Data out:
  ne_out: wp.array(dtype=int),
):
  worldid = wp.tid()
  ne = ne_connect_in[worldid] + ne_weld_in[worldid] + ne_jnt_in[worldid] + ne_ten_in[worldid]
  ne_out[worldid] = ne


@event_scope
def make_constraint(m: types.Model, d: types.Data):
  """Creates constraint jacobians and other supporting data."""

  wp.launch(
    zero_constraint_counts,
    dim=d.nworld,
    inputs=[
      d.ne,
      d.ne_connect,
      d.ne_weld,
      d.ne_jnt,
      d.ne_ten,
      d.nf,
      d.nl,
      d.nefc,
    ],
  )

  if not (m.opt.disableflags & types.DisableBit.CONSTRAINT.value):
    refsafe = m.opt.disableflags & types.DisableBit.REFSAFE

    if not (m.opt.disableflags & types.DisableBit.EQUALITY.value):
      wp.launch(
        _efc_equality_connect,
        dim=(d.nworld, m.eq_connect_adr.size),
        inputs=[
          m.nv,
          m.nsite,
          m.opt.timestep,
          m.body_parentid,
          m.body_rootid,
          m.body_invweight0,
          m.dof_bodyid,
          m.site_bodyid,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_objtype,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.eq_connect_adr,
          d.njmax,
          d.qvel,
          d.eq_active,
          d.xpos,
          d.xmat,
          d.site_xpos,
          d.subtree_com,
          d.cdof,
          refsafe,
        ],
        outputs=[
          d.ne_connect,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
        ],
      )
      wp.launch(
        _efc_equality_weld,
        dim=(d.nworld, m.eq_wld_adr.size),
        inputs=[
          m.nv,
          m.nsite,
          m.opt.timestep,
          m.body_parentid,
          m.body_rootid,
          m.body_invweight0,
          m.dof_bodyid,
          m.site_bodyid,
          m.site_quat,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_objtype,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.eq_wld_adr,
          d.njmax,
          d.qvel,
          d.eq_active,
          d.xpos,
          d.xquat,
          d.xmat,
          d.site_xpos,
          d.subtree_com,
          d.cdof,
          refsafe,
        ],
        outputs=[
          d.ne_weld,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
        ],
      )
      wp.launch(
        _efc_equality_joint,
        dim=(d.nworld, m.eq_jnt_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.qpos0,
          m.jnt_qposadr,
          m.jnt_dofadr,
          m.dof_invweight0,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.eq_jnt_adr,
          d.njmax,
          d.qpos,
          d.qvel,
          d.eq_active,
          refsafe,
        ],
        outputs=[
          d.ne_jnt,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
        ],
      )
      wp.launch(
        _efc_equality_tendon,
        dim=(d.nworld, m.eq_ten_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.eq_ten_adr,
          m.tendon_length0,
          m.tendon_invweight0,
          d.njmax,
          d.qvel,
          d.eq_active,
          d.ten_length,
          d.ten_J,
          refsafe,
        ],
        outputs=[
          d.ne_ten,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
        ],
      )

      wp.launch(
        _num_equality,
        dim=(d.nworld,),
        inputs=[
          d.ne_connect,
          d.ne_weld,
          d.ne_jnt,
          d.ne_ten,
        ],
        outputs=[
          d.ne,
        ],
      )

    if not (m.opt.disableflags & types.DisableBit.FRICTIONLOSS.value):
      wp.launch(
        _efc_friction_dof,
        dim=(d.nworld, m.nv),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.dof_invweight0,
          m.dof_frictionloss,
          m.dof_solimp,
          m.dof_solref,
          d.njmax,
          d.qvel,
          refsafe,
        ],
        outputs=[
          d.nf,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
        ],
      )

      wp.launch(
        _efc_friction_tendon,
        dim=(d.nworld, m.ntendon),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.tendon_solref_fri,
          m.tendon_solimp_fri,
          m.tendon_frictionloss,
          m.tendon_invweight0,
          d.qvel,
          d.ten_J,
          refsafe,
        ],
        outputs=[
          d.nf,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
        ],
      )

    # limit
    if not (m.opt.disableflags & types.DisableBit.LIMIT.value):
      limit_ball = m.jnt_limited_ball_adr.size > 0
      if limit_ball:
        wp.launch(
          _efc_limit_ball,
          dim=(d.nworld, m.jnt_limited_ball_adr.size),
          inputs=[
            m.nv,
            m.opt.timestep,
            m.jnt_qposadr,
            m.jnt_dofadr,
            m.jnt_solref,
            m.jnt_solimp,
            m.jnt_range,
            m.jnt_margin,
            m.jnt_limited_ball_adr,
            m.dof_invweight0,
            d.njmax,
            d.qpos,
            d.qvel,
            refsafe,
          ],
          outputs=[
            d.nl,
            d.nefc,
            d.efc.type,
            d.efc.id,
            d.efc.J,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )

      limit_slide_hinge = m.jnt_limited_slide_hinge_adr.size > 0
      if limit_slide_hinge:
        wp.launch(
          _efc_limit_slide_hinge,
          dim=(d.nworld, m.jnt_limited_slide_hinge_adr.size),
          inputs=[
            m.nv,
            m.opt.timestep,
            m.jnt_qposadr,
            m.jnt_dofadr,
            m.jnt_solref,
            m.jnt_solimp,
            m.jnt_range,
            m.jnt_margin,
            m.jnt_limited_slide_hinge_adr,
            m.dof_invweight0,
            d.njmax,
            d.qpos,
            d.qvel,
            refsafe,
          ],
          outputs=[
            d.nl,
            d.nefc,
            d.efc.type,
            d.efc.id,
            d.efc.J,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )

      limit_tendon = m.tendon_limited_adr.size > 0
      if limit_tendon:
        wp.launch(
          _efc_limit_tendon,
          dim=(d.nworld, m.tendon_limited_adr.size),
          inputs=[
            m.nv,
            m.opt.timestep,
            m.jnt_dofadr,
            m.tendon_adr,
            m.tendon_num,
            m.tendon_limited_adr,
            m.tendon_solref_lim,
            m.tendon_solimp_lim,
            m.tendon_range,
            m.tendon_margin,
            m.tendon_invweight0,
            m.wrap_objid,
            m.wrap_type,
            d.njmax,
            d.qvel,
            d.ten_length,
            d.ten_J,
            refsafe,
          ],
          outputs=[
            d.nl,
            d.nefc,
            d.efc.type,
            d.efc.id,
            d.efc.J,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )

    # contact
    if not (m.opt.disableflags & types.DisableBit.CONTACT.value):
      if m.opt.cone == types.ConeType.PYRAMIDAL.value:
        wp.launch(
          _efc_contact_pyramidal,
          dim=(d.nconmax, 2 * (m.condim_max - 1) if m.condim_max > 1 else 1),
          inputs=[
            m.nv,
            m.opt.timestep,
            m.opt.impratio,
            m.body_parentid,
            m.body_rootid,
            m.body_invweight0,
            m.dof_bodyid,
            m.geom_bodyid,
            d.njmax,
            d.ncon,
            d.qvel,
            d.subtree_com,
            d.cdof,
            refsafe,
            d.contact.dist,
            d.contact.dim,
            d.contact.includemargin,
            d.contact.worldid,
            d.contact.geom,
            d.contact.pos,
            d.contact.frame,
            d.contact.friction,
            d.contact.solref,
            d.contact.solimp,
          ],
          outputs=[
            d.nefc,
            d.contact.efc_address,
            d.efc.type,
            d.efc.id,
            d.efc.J,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )
      elif m.opt.cone == types.ConeType.ELLIPTIC.value:
        wp.launch(
          _efc_contact_elliptic,
          dim=(d.nconmax, m.condim_max),
          inputs=[
            m.nv,
            m.opt.timestep,
            m.opt.impratio,
            m.body_parentid,
            m.body_rootid,
            m.body_invweight0,
            m.dof_bodyid,
            m.geom_bodyid,
            d.njmax,
            d.ncon,
            d.qvel,
            d.subtree_com,
            d.cdof,
            refsafe,
            d.contact.dist,
            d.contact.dim,
            d.contact.includemargin,
            d.contact.worldid,
            d.contact.geom,
            d.contact.pos,
            d.contact.frame,
            d.contact.friction,
            d.contact.solref,
            d.contact.solreffriction,
            d.contact.solimp,
          ],
          outputs=[
            d.nefc,
            d.contact.efc_address,
            d.efc.type,
            d.efc.id,
            d.efc.J,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )
