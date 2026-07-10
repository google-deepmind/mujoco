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

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.types import ConstraintType
from mujoco.mjx.third_party.mujoco_warp._src.types import ContactType
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.types import vec11
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _zero_constraint_counts(
  # Data out:
  ne_out: wp.array[int],
  nf_out: wp.array[int],
  nl_out: wp.array[int],
  nefc_out: wp.array[int],
  efc_jtdaj_nblock_out: wp.array[int],
  # Out:
  efc_nnz_out: wp.array[int],
):
  worldid = wp.tid()

  # Zero all constraint counters
  ne_out[worldid] = 0
  nf_out[worldid] = 0
  nl_out[worldid] = 0
  nefc_out[worldid] = 0
  efc_jtdaj_nblock_out[worldid] = 0
  efc_nnz_out[worldid] = 0


@wp.func
def _efc_row(
  # Model:
  opt_disableflags: int,
  # In:
  worldid: int,
  timestep: float,
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
  # Out:
  type_out: wp.array2d[int],
  id_out: wp.array2d[int],
  pos_out: wp.array2d[float],
  margin_out: wp.array2d[float],
  D_out: wp.array2d[float],
  vel_out: wp.array2d[float],
  aref_out: wp.array2d[float],
  frictionloss_out: wp.array2d[float],
):
  # calculate kbi
  timeconst = solref[0]
  dampratio = solref[1]
  dmin = solimp[0]
  dmax = solimp[1]
  width = solimp[2]
  mid = solimp[3]
  power = solimp[4]

  if not (opt_disableflags & DisableBit.REFSAFE):
    timeconst = wp.max(timeconst, 2.0 * timestep)

  dmin = wp.clamp(dmin, types.MJ_MINIMP, types.MJ_MAXIMP)
  dmax = wp.clamp(dmax, types.MJ_MINIMP, types.MJ_MAXIMP)
  width = wp.max(types.MJ_MINVAL, width)
  mid = wp.clamp(mid, types.MJ_MINIMP, types.MJ_MAXIMP)
  power = wp.max(1.0, power)

  # see https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters
  dmax_sq = dmax * dmax
  k = 1.0 / (dmax_sq * timeconst * timeconst * dampratio * dampratio)
  b = 2.0 / (dmax * timeconst)
  k = wp.where(solref[0] <= 0, -solref[0] / dmax_sq, k)
  b = wp.where(solref[1] <= 0, -solref[1] / dmax, b)

  imp_x = wp.abs(pos_imp) / width
  imp_a = (1.0 / wp.pow(mid, power - 1.0)) * wp.pow(imp_x, power)
  imp_b = 1.0 - (1.0 / wp.pow(1.0 - mid, power - 1.0)) * wp.pow(1.0 - imp_x, power)
  imp_y = wp.where(imp_x < mid, imp_a, imp_b)
  imp = dmin + imp_y * (dmax - dmin)
  imp = wp.clamp(imp, dmin, dmax)
  imp = wp.where(imp_x > 1.0, dmax, imp)

  # set outputs
  D_out[worldid, efcid] = 1.0 / wp.max(invweight * (1.0 - imp) / imp, types.MJ_MINVAL)
  vel_out[worldid, efcid] = vel
  aref_out[worldid, efcid] = -k * imp * pos_aref - b * vel
  pos_out[worldid, efcid] = pos_aref + margin
  margin_out[worldid, efcid] = margin
  frictionloss_out[worldid, efcid] = frictionloss
  type_out[worldid, efcid] = type
  id_out[worldid, efcid] = id


@cache_kernel
def _equality_connect(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    nsite: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    body_parentid: wp.array[int],
    body_rootid: wp.array[int],
    body_weldid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    body_invweight0: wp.array2d[wp.vec2],
    jnt_type: wp.array[int],
    jnt_dofadr: wp.array[int],
    dof_bodyid: wp.array[int],
    dof_jntid: wp.array[int],
    dof_parentid: wp.array[int],
    site_bodyid: wp.array[int],
    eq_obj1id: wp.array[int],
    eq_obj2id: wp.array[int],
    eq_objtype: wp.array[int],
    eq_solref: wp.array2d[wp.vec2],
    eq_solimp: wp.array2d[vec5],
    eq_data: wp.array2d[vec11],
    body_isdofancestor: wp.array2d[int],
    eq_connect_adr: wp.array[int],
    # Data in:
    qvel_in: wp.array2d[float],
    eq_active_in: wp.array2d[bool],
    xpos_in: wp.array2d[wp.vec3],
    xmat_in: wp.array2d[wp.mat33],
    site_xpos_in: wp.array2d[wp.vec3],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    cvel_in: wp.array2d[wp.spatial_vector],
    cdof_dot_in: wp.array2d[wp.spatial_vector],
    subtree_linvel_in: wp.array2d[wp.vec3],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    ne_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    """Calculates constraint rows for connect equality constraints."""
    worldid, eqconnectid = wp.tid()
    eqid = eq_connect_adr[eqconnectid]

    if not eq_active_in[worldid, eqid]:
      return

    wp.atomic_add(ne_out, worldid, 3)
    efcid = wp.atomic_add(nefc_out, worldid, 3)

    if efcid >= njmax_in - 3:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 3

    efcid0 = efcid + 0
    efcid1 = efcid + 1
    efcid2 = efcid + 2

    data = eq_data[worldid % eq_data.shape[0], eqid]
    anchor1 = wp.vec3f(data[0], data[1], data[2])
    anchor2 = wp.vec3f(data[3], data[4], data[5])

    obj1id = eq_obj1id[eqid]
    obj2id = eq_obj2id[eqid]

    if nsite > 0 and eq_objtype[eqid] == types.ObjType.SITE:
      body1 = site_bodyid[obj1id]
      body2 = site_bodyid[obj2id]
      pos1 = site_xpos_in[worldid, obj1id]
      pos2 = site_xpos_in[worldid, obj2id]
    else:
      body1 = obj1id
      body2 = obj2id
      pos1 = xpos_in[worldid, body1] + xmat_in[worldid, body1] @ anchor1
      pos2 = xpos_in[worldid, body2] + xmat_in[worldid, body2] @ anchor2

    # error is difference in global positions
    pos = pos1 - pos2

    # compute Jacobian difference (opposite of contact: 0 - 1)
    Jqvel = wp.vec3f(0.0, 0.0, 0.0)
    Jdotv = wp.vec3f(0.0, 0.0, 0.0)

    if wp.static(is_sparse):
      # TODO(team): pre-compute number of non-zeros
      body1 = body_weldid[body1]
      body2 = body_weldid[body2]

      da1 = int(body_dofadr[body1] + body_dofnum[body1] - 1)
      da2 = int(body_dofadr[body2] + body_dofnum[body2] - 1)

      # count non-zeros
      pda1 = da1
      pda2 = da2
      rownnz = int(0)
      while pda1 >= 0 or pda2 >= 0:
        da = wp.max(pda1, pda2)
        if pda1 == da:
          pda1 = dof_parentid[pda1]
        if pda2 == da:
          pda2 = dof_parentid[pda2]
        rownnz += 1

      # get rowadr
      rowadr = wp.atomic_add(efc_nnz_out, worldid, 3 * rownnz)
      if rowadr + 3 * rownnz > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid0] = rowadr
      efc_J_rowadr_out[worldid, efcid1] = rowadr + rownnz
      efc_J_rowadr_out[worldid, efcid2] = rowadr + 2 * rownnz

      efc_J_rownnz_out[worldid, efcid0] = rownnz
      efc_J_rownnz_out[worldid, efcid1] = rownnz
      efc_J_rownnz_out[worldid, efcid2] = rownnz

      # compute J and colind
      nnz = int(0)
      while da1 >= 0 or da2 >= 0:
        da = wp.max(da1, da2)
        if da1 == da:
          da1 = dof_parentid[da1]
        if da2 == da:
          da2 = dof_parentid[da2]

        jacp1, _ = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos1,
          body1,
          da,
          worldid,
        )
        jacp2, _ = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos2,
          body2,
          da,
          worldid,
        )
        j1mj2 = jacp1 - jacp2

        jacp1_dot, _ = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos1,
          body1,
          da,
          worldid,
        )
        jacp2_dot, _ = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos2,
          body2,
          da,
          worldid,
        )
        j1mj2_dot = jacp1_dot - jacp2_dot

        sparseid0 = rowadr + nnz
        sparseid1 = rowadr + rownnz + nnz
        sparseid2 = rowadr + 2 * rownnz + nnz

        efc_J_colind_out[worldid, 0, sparseid0] = da
        efc_J_colind_out[worldid, 0, sparseid1] = da
        efc_J_colind_out[worldid, 0, sparseid2] = da

        efc_J_out[worldid, 0, sparseid0] = j1mj2[0]
        efc_J_out[worldid, 0, sparseid1] = j1mj2[1]
        efc_J_out[worldid, 0, sparseid2] = j1mj2[2]

        qvel = qvel_in[worldid, da]
        Jqvel += j1mj2 * qvel
        Jdotv += j1mj2_dot * qvel

        nnz += 1
    else:
      # TODO(team): dof tree traversal
      for dofid in range(nv):
        jacp1, _ = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos1,
          body1,
          dofid,
          worldid,
        )
        jacp2, _ = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos2,
          body2,
          dofid,
          worldid,
        )
        j1mj2 = jacp1 - jacp2

        jacp1_dot, _ = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos1,
          body1,
          dofid,
          worldid,
        )
        jacp2_dot, _ = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos2,
          body2,
          dofid,
          worldid,
        )
        j1mj2_dot = jacp1_dot - jacp2_dot

        efc_J_out[worldid, efcid0, dofid] = j1mj2[0]
        efc_J_out[worldid, efcid1, dofid] = j1mj2[1]
        efc_J_out[worldid, efcid2, dofid] = j1mj2[2]

        qvel = qvel_in[worldid, dofid]
        Jqvel += j1mj2 * qvel
        Jdotv += j1mj2_dot * qvel

    body_invweight0_id = worldid % body_invweight0.shape[0]
    invweight = body_invweight0[body_invweight0_id, body1][0] + body_invweight0[body_invweight0_id, body2][0]
    pos_imp = wp.length(pos)

    solref = eq_solref[worldid % eq_solref.shape[0], eqid]
    solimp = eq_solimp[worldid % eq_solimp.shape[0], eqid]
    timestep = opt_timestep[worldid % opt_timestep.shape[0]]

    for i in range(3):
      efcidi = efcid + i

      _efc_row(
        opt_disableflags,
        worldid,
        timestep,
        efcidi,
        pos[i],
        pos_imp,
        invweight,
        solref,
        solimp,
        0.0,
        Jqvel[i],
        0.0,
        ConstraintType.EQUALITY,
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

      efc_aref_out[worldid, efcidi] -= Jdotv[i]

  return kernel


@cache_kernel
def _equality_joint(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    qpos0: wp.array2d[float],
    jnt_qposadr: wp.array[int],
    jnt_dofadr: wp.array[int],
    dof_invweight0: wp.array2d[float],
    eq_obj1id: wp.array[int],
    eq_obj2id: wp.array[int],
    eq_solref: wp.array2d[wp.vec2],
    eq_solimp: wp.array2d[vec5],
    eq_data: wp.array2d[vec11],
    eq_jnt_adr: wp.array[int],
    # Data in:
    qpos_in: wp.array2d[float],
    qvel_in: wp.array2d[float],
    eq_active_in: wp.array2d[bool],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    ne_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, eqjntid = wp.tid()
    eqid = eq_jnt_adr[eqjntid]

    if not eq_active_in[worldid, eqid]:
      return

    wp.atomic_add(ne_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 1

    jntid_1 = eq_obj1id[eqid]
    jntid_2 = eq_obj2id[eqid]
    data = eq_data[worldid % eq_data.shape[0], eqid]
    dofadr1 = jnt_dofadr[jntid_1]
    qposadr1 = jnt_qposadr[jntid_1]
    qpos0_id = worldid % qpos0.shape[0]
    dof_invweight0_id = worldid % dof_invweight0.shape[0]

    if wp.static(is_sparse):
      if jntid_2 > -1:
        rownnz = 2
      else:
        rownnz = 1
      efc_J_rownnz_out[worldid, efcid] = rownnz
      rowadr = wp.atomic_add(efc_nnz_out, worldid, rownnz)
      if rowadr + rownnz > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid] = rowadr
      efc_J_colind_out[worldid, 0, rowadr] = dofadr1
      efc_J_out[worldid, 0, rowadr] = 1.0
    else:
      for i in range(nv):
        efc_J_out[worldid, efcid, i] = 0.0
      efc_J_out[worldid, efcid, dofadr1] = 1.0

    if jntid_2 > -1:
      # Two joint constraint
      qposadr2 = jnt_qposadr[jntid_2]
      dofadr2 = jnt_dofadr[jntid_2]
      dif = qpos_in[worldid, qposadr2] - qpos0[qpos0_id, qposadr2]

      # Horner's method for polynomials
      rhs = data[0] + dif * (data[1] + dif * (data[2] + dif * (data[3] + dif * data[4])))
      deriv_2 = data[1] + dif * (2.0 * data[2] + dif * (3.0 * data[3] + dif * 4.0 * data[4]))

      pos = qpos_in[worldid, qposadr1] - qpos0[qpos0_id, qposadr1] - rhs
      Jqvel = qvel_in[worldid, dofadr1] - qvel_in[worldid, dofadr2] * deriv_2
      invweight = dof_invweight0[dof_invweight0_id, dofadr1] + dof_invweight0[dof_invweight0_id, dofadr2]

      if wp.static(is_sparse):
        sparseid = rowadr + 1
        efc_J_colind_out[worldid, 0, sparseid] = dofadr2
        efc_J_out[worldid, 0, sparseid] = -deriv_2
      else:
        efc_J_out[worldid, efcid, dofadr2] = -deriv_2
    else:
      # Single joint constraint
      pos = qpos_in[worldid, qposadr1] - qpos0[qpos0_id, qposadr1] - data[0]
      Jqvel = qvel_in[worldid, dofadr1]
      invweight = dof_invweight0[dof_invweight0_id, dofadr1]

    # Update constraint parameters
    _efc_row(
      opt_disableflags,
      worldid,
      opt_timestep[worldid % opt_timestep.shape[0]],
      efcid,
      pos,
      pos,
      invweight,
      eq_solref[worldid % eq_solref.shape[0], eqid],
      eq_solimp[worldid % eq_solimp.shape[0], eqid],
      0.0,
      Jqvel,
      0.0,
      ConstraintType.EQUALITY,
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

  return kernel


@cache_kernel
def _equality_tendon(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    eq_obj1id: wp.array[int],
    eq_obj2id: wp.array[int],
    eq_solref: wp.array2d[wp.vec2],
    eq_solimp: wp.array2d[vec5],
    eq_data: wp.array2d[vec11],
    ten_J_rownnz: wp.array[int],
    ten_J_rowadr: wp.array[int],
    ten_J_colind: wp.array[int],
    tendon_length0: wp.array2d[float],
    tendon_invweight0: wp.array2d[float],
    eq_ten_adr: wp.array[int],
    # Data in:
    qvel_in: wp.array2d[float],
    eq_active_in: wp.array2d[bool],
    ten_J_in: wp.array2d[float],
    ten_length_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    ne_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, eqtenid = wp.tid()
    eqid = eq_ten_adr[eqtenid]

    if not eq_active_in[worldid, eqid]:
      return

    wp.atomic_add(ne_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 1

    obj1id = eq_obj1id[eqid]
    obj2id = eq_obj2id[eqid]

    data = eq_data[worldid % eq_data.shape[0], eqid]
    solref = eq_solref[worldid % eq_solref.shape[0], eqid]
    solimp = eq_solimp[worldid % eq_solimp.shape[0], eqid]
    tendon_length0_id = worldid % tendon_length0.shape[0]
    tendon_invweight0_id = worldid % tendon_invweight0.shape[0]
    pos1 = ten_length_in[worldid, obj1id] - tendon_length0[tendon_length0_id, obj1id]

    if obj2id > -1:
      invweight = tendon_invweight0[tendon_invweight0_id, obj1id] + tendon_invweight0[tendon_invweight0_id, obj2id]

      pos2 = ten_length_in[worldid, obj2id] - tendon_length0[tendon_length0_id, obj2id]

      dif = pos2
      dif2 = dif * dif
      dif3 = dif2 * dif
      dif4 = dif3 * dif

      pos = pos1 - (data[0] + data[1] * dif + data[2] * dif2 + data[3] * dif3 + data[4] * dif4)
      deriv = data[1] + 2.0 * data[2] * dif + 3.0 * data[3] * dif2 + 4.0 * data[4] * dif3
    else:
      invweight = tendon_invweight0[tendon_invweight0_id, obj1id]
      pos = pos1 - data[0]
      deriv = 0.0

    rownnz1 = ten_J_rownnz[obj1id]
    rowadr1 = ten_J_rowadr[obj1id]
    rownnz2 = 0
    rowadr2 = 0

    if deriv != 0.0:
      rownnz2 = ten_J_rownnz[obj2id]
      rowadr2 = ten_J_rowadr[obj2id]

    if wp.static(is_sparse):
      # TODO(team): pre-compute rownnz
      # count unique dofs
      p1, p2 = int(0), int(0)
      rownnz = int(0)
      while p1 < rownnz1 or p2 < rownnz2:
        col1 = nv
        col2 = nv
        if p1 < rownnz1:
          col1 = ten_J_colind[rowadr1 + p1]
        if p2 < rownnz2:
          col2 = ten_J_colind[rowadr2 + p2]
        if col1 <= col2:
          p1 += 1
        if col2 <= col1:
          p2 += 1
        rownnz += 1

      rowadr = wp.atomic_add(efc_nnz_out, worldid, rownnz)
      if rowadr + rownnz > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid] = rowadr

    ptr1 = int(0)
    ptr2 = int(0)

    Jqvel = float(0.0)

    nnz = int(0)
    for i in range(nv):
      J1 = float(0.0)
      if ptr1 < rownnz1:
        sparseid1 = rowadr1 + ptr1
        if ten_J_colind[sparseid1] == i:
          J1 = ten_J_in[worldid, sparseid1]
          ptr1 += 1

      J = J1
      if deriv != 0.0:
        J2 = float(0.0)
        if ptr2 < rownnz2:
          sparseid2 = rowadr2 + ptr2
          if ten_J_colind[sparseid2] == i:
            J2 = ten_J_in[worldid, sparseid2]
            ptr2 += 1
        J += J2 * -deriv

      if wp.static(is_sparse):
        if J != 0.0:
          sparseid = rowadr + nnz
          efc_J_colind_out[worldid, 0, sparseid] = i
          efc_J_out[worldid, 0, sparseid] = J
          nnz += 1
      else:
        efc_J_out[worldid, efcid, i] = J

      Jqvel += J * qvel_in[worldid, i]

    if wp.static(is_sparse):
      efc_J_rownnz_out[worldid, efcid] = nnz

    _efc_row(
      opt_disableflags,
      worldid,
      opt_timestep[worldid % opt_timestep.shape[0]],
      efcid,
      pos,
      pos,
      invweight,
      solref,
      solimp,
      0.0,
      Jqvel,
      0.0,
      ConstraintType.EQUALITY,
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

  return kernel


@cache_kernel
def _equality_flex(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    flex_interp: wp.array[int],
    flex_edgeadr: wp.array[int],
    flex_edgenum: wp.array[int],
    flexedge_length0: wp.array[float],
    flexedge_invweight0: wp.array[float],
    flexedge_J_rownnz: wp.array[int],
    flexedge_J_rowadr: wp.array[int],
    flexedge_J_colind: wp.array[int],
    eq_obj1id: wp.array[int],
    eq_solref: wp.array2d[wp.vec2],
    eq_solimp: wp.array2d[vec5],
    eq_flex_adr: wp.array[int],
    # Data in:
    qvel_in: wp.array2d[float],
    eq_active_in: wp.array2d[bool],
    flexedge_J_in: wp.array2d[float],
    flexedge_length_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    ne_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, eqflexid, edgeid = wp.tid()
    eqid = eq_flex_adr[eqflexid]

    if not eq_active_in[worldid, eqid]:
      return

    flexid = eq_obj1id[eqid]

    # skip edge-based equality for interpolated (trilinear/quadratic) flex
    if flex_interp[flexid] != 0:
      return

    if edgeid < flex_edgeadr[flexid] or edgeid >= flex_edgeadr[flexid] + flex_edgenum[flexid]:
      return

    wp.atomic_add(ne_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 1

    pos = flexedge_length_in[worldid, edgeid] - flexedge_length0[edgeid]
    solref = eq_solref[worldid % eq_solref.shape[0], eqid]
    solimp = eq_solimp[worldid % eq_solimp.shape[0], eqid]

    Jqvel = float(0.0)

    rownnz = flexedge_J_rownnz[edgeid]
    flex_rowadr = flexedge_J_rowadr[edgeid]

    if wp.static(is_sparse):
      efc_J_rownnz_out[worldid, efcid] = rownnz
      efc_rowadr = wp.atomic_add(efc_nnz_out, worldid, rownnz)
      if efc_rowadr + rownnz > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid] = efc_rowadr
      for i in range(rownnz):
        flex_sparseid = flex_rowadr + i
        efc_sparseid = efc_rowadr + i
        colind = flexedge_J_colind[flex_sparseid]
        J = flexedge_J_in[worldid, flex_sparseid]
        efc_J_colind_out[worldid, 0, efc_sparseid] = colind
        efc_J_out[worldid, 0, efc_sparseid] = J
        Jqvel += J * qvel_in[worldid, colind]
    else:
      for i in range(nv):
        efc_J_out[worldid, efcid, i] = 0.0
      for i in range(rownnz):
        flex_sparseid = flex_rowadr + i
        colind = flexedge_J_colind[flex_sparseid]
        J = flexedge_J_in[worldid, flex_sparseid]
        efc_J_out[worldid, efcid, colind] = J
        Jqvel += J * qvel_in[worldid, colind]

    _efc_row(
      opt_disableflags,
      worldid,
      opt_timestep[worldid % opt_timestep.shape[0]],
      efcid,
      pos,
      pos,
      flexedge_invweight0[edgeid],
      solref,
      solimp,
      0.0,
      Jqvel,
      0.0,
      ConstraintType.EQUALITY,
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

  return kernel


@cache_kernel
def _equality_weld(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    nsite: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    body_parentid: wp.array[int],
    body_rootid: wp.array[int],
    body_weldid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    body_invweight0: wp.array2d[wp.vec2],
    jnt_type: wp.array[int],
    jnt_dofadr: wp.array[int],
    dof_bodyid: wp.array[int],
    dof_jntid: wp.array[int],
    dof_parentid: wp.array[int],
    site_bodyid: wp.array[int],
    site_quat: wp.array2d[wp.quat],
    eq_obj1id: wp.array[int],
    eq_obj2id: wp.array[int],
    eq_objtype: wp.array[int],
    eq_solref: wp.array2d[wp.vec2],
    eq_solimp: wp.array2d[vec5],
    eq_data: wp.array2d[vec11],
    body_isdofancestor: wp.array2d[int],
    eq_wld_adr: wp.array[int],
    # Data in:
    qvel_in: wp.array2d[float],
    eq_active_in: wp.array2d[bool],
    xpos_in: wp.array2d[wp.vec3],
    xquat_in: wp.array2d[wp.quat],
    xmat_in: wp.array2d[wp.mat33],
    site_xpos_in: wp.array2d[wp.vec3],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    cvel_in: wp.array2d[wp.spatial_vector],
    cdof_dot_in: wp.array2d[wp.spatial_vector],
    subtree_linvel_in: wp.array2d[wp.vec3],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    ne_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, eqweldid = wp.tid()
    eqid = eq_wld_adr[eqweldid]

    if not eq_active_in[worldid, eqid]:
      return

    wp.atomic_add(ne_out, worldid, 6)
    efcid = wp.atomic_add(nefc_out, worldid, 6)

    if efcid >= njmax_in - 6:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 6

    efcid0 = efcid + 0
    efcid1 = efcid + 1
    efcid2 = efcid + 2
    efcid3 = efcid + 3
    efcid4 = efcid + 4
    efcid5 = efcid + 5

    is_site = eq_objtype[eqid] == types.ObjType.SITE and nsite > 0

    obj1id = eq_obj1id[eqid]
    obj2id = eq_obj2id[eqid]

    data = eq_data[worldid % eq_data.shape[0], eqid]
    anchor1 = wp.vec3(data[0], data[1], data[2])
    anchor2 = wp.vec3(data[3], data[4], data[5])
    relpose = wp.quat(data[6], data[7], data[8], data[9])
    torquescale = data[10]

    if is_site:
      body1 = site_bodyid[obj1id]
      body2 = site_bodyid[obj2id]
      pos1 = site_xpos_in[worldid, obj1id]
      pos2 = site_xpos_in[worldid, obj2id]

      site_quat_id = worldid % site_quat.shape[0]
      quat = math.mul_quat(xquat_in[worldid, body1], site_quat[site_quat_id, obj1id])
      quat1 = math.quat_inv(math.mul_quat(xquat_in[worldid, body2], site_quat[site_quat_id, obj2id]))

    else:
      body1 = obj1id
      body2 = obj2id
      pos1 = xpos_in[worldid, body1] + xmat_in[worldid, body1] @ anchor2
      pos2 = xpos_in[worldid, body2] + xmat_in[worldid, body2] @ anchor1

      quat = math.mul_quat(xquat_in[worldid, body1], relpose)
      quat1 = math.quat_inv(xquat_in[worldid, body2])

      # quat1 = quat_inv(xquat_in[worldid, body2])
      q2 = xquat_in[worldid, body2]
      quat1 = wp.quat(q2[0], -q2[1], -q2[2], -q2[3])

    # compute rotational Jdotv helper quaternions
    omega1 = wp.spatial_top(cvel_in[worldid, body1])
    omega2 = wp.spatial_top(cvel_in[worldid, body2])
    domega = omega1 - omega2

    omega1_q = wp.quat(0.0, omega1[0], omega1[1], omega1[2])
    omega2_q = wp.quat(0.0, omega2[0], omega2[1], omega2[2])
    domega_q = wp.quat(0.0, domega[0], domega[1], domega[2])

    if is_site:
      qdot0r = math.mul_quat(omega1_q, quat) * 0.5
      qfull1 = math.mul_quat(xquat_in[worldid, body2], site_quat[site_quat_id, obj2id])
      qdot1 = math.mul_quat(omega2_q, qfull1) * 0.5

      negqdot1 = wp.quat(-qdot1[0], -qdot1[1], -qdot1[2], -qdot1[3])
      negq1 = wp.quat(qfull1[0], -qfull1[1], -qfull1[2], -qfull1[3])

    else:
      # qdot0 = mul_quat(xquat_in[worldid, body1], omega1_q) * 0.5
      u7 = xquat_in[worldid, body1]
      qdot0 = math.mul_quat(omega1_q, xquat_in[worldid, body1]) * 0.5
      qdot0r = math.mul_quat(qdot0, relpose)
      q1_non_site = xquat_in[worldid, body2]
      qdot1 = math.mul_quat(omega2_q, q1_non_site) * 0.5

      negqdot1 = wp.quat(-qdot1[0], -qdot1[1], -qdot1[2], -qdot1[3])
      negq1 = wp.quat(q1_non_site[0], -q1_non_site[1], -q1_non_site[2], -q1_non_site[3])

    # compute Jacobian difference (opposite of contact: 0 - 1)
    Jqvelp = wp.vec3f(0.0, 0.0, 0.0)
    Jqvelr = wp.vec3f(0.0, 0.0, 0.0)
    Jdotv_p = wp.vec3f(0.0, 0.0, 0.0)
    Jdotv_r0 = wp.vec3f(0.0, 0.0, 0.0)

    if wp.static(is_sparse):
      # TODO(team): pre-compute number of non-zeros
      body1 = body_weldid[body1]
      body2 = body_weldid[body2]

      da1 = int(body_dofadr[body1] + body_dofnum[body1] - 1)
      da2 = int(body_dofadr[body2] + body_dofnum[body2] - 1)

      # count non-zeros
      pda1 = da1
      pda2 = da2
      rownnz = int(0)
      while pda1 >= 0 or pda2 >= 0:
        da = wp.max(pda1, pda2)
        if pda1 == da:
          pda1 = dof_parentid[da]
        if pda2 == da:
          pda2 = dof_parentid[da]
        rownnz += 1

      # get rowadr
      rowadr = wp.atomic_add(efc_nnz_out, worldid, 6 * rownnz)
      if rowadr + 6 * rownnz > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid0] = rowadr
      efc_J_rowadr_out[worldid, efcid1] = rowadr + rownnz
      efc_J_rowadr_out[worldid, efcid2] = rowadr + 2 * rownnz
      efc_J_rowadr_out[worldid, efcid3] = rowadr + 3 * rownnz
      efc_J_rowadr_out[worldid, efcid4] = rowadr + 4 * rownnz
      efc_J_rowadr_out[worldid, efcid5] = rowadr + 5 * rownnz

      efc_J_rownnz_out[worldid, efcid0] = rownnz
      efc_J_rownnz_out[worldid, efcid1] = rownnz
      efc_J_rownnz_out[worldid, efcid2] = rownnz
      efc_J_rownnz_out[worldid, efcid3] = rownnz
      efc_J_rownnz_out[worldid, efcid4] = rownnz
      efc_J_rownnz_out[worldid, efcid5] = rownnz

      # compute J and colind
      nnz = int(0)
      while da1 >= 0 or da2 >= 0:
        da = wp.max(da1, da2)
        if da1 == da:
          da1 = dof_parentid[da]
        if da2 == da:
          da2 = dof_parentid[da]

        jacp1, jacr1 = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos1,
          body1,
          da,
          worldid,
        )
        jacp2, jacr2 = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos2,
          body2,
          da,
          worldid,
        )

        jacdifp = jacp1 - jacp2

        jacdifr = (jacr1 - jacr2) * torquescale
        jacdifrq = math.mul_quat(math.quat_mul_axis(quat1, jacdifr), quat)
        jacdifr = 0.5 * wp.vec3(jacdifrq[1], jacdifrq[2], jacdifrq[3])

        jacp1_dot, jacr1_dot = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos1,
          body1,
          da,
          worldid,
        )
        jacp2_dot, jacr2_dot = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos2,
          body2,
          da,
          worldid,
        )

        jacdifp_dot = jacp1_dot - jacp2_dot
        jacdifr_dot = jacr1_dot - jacr2_dot

        sparseid0 = rowadr + nnz
        sparseid1 = rowadr + rownnz + nnz
        sparseid2 = rowadr + 2 * rownnz + nnz
        sparseid3 = rowadr + 3 * rownnz + nnz
        sparseid4 = rowadr + 4 * rownnz + nnz
        sparseid5 = rowadr + 5 * rownnz + nnz

        efc_J_colind_out[worldid, 0, sparseid0] = da
        efc_J_colind_out[worldid, 0, sparseid1] = da
        efc_J_colind_out[worldid, 0, sparseid2] = da
        efc_J_colind_out[worldid, 0, sparseid3] = da
        efc_J_colind_out[worldid, 0, sparseid4] = da
        efc_J_colind_out[worldid, 0, sparseid5] = da

        efc_J_out[worldid, 0, sparseid0] = jacdifp[0]
        efc_J_out[worldid, 0, sparseid1] = jacdifp[1]
        efc_J_out[worldid, 0, sparseid2] = jacdifp[2]
        efc_J_out[worldid, 0, sparseid3] = jacdifr[0]
        efc_J_out[worldid, 0, sparseid4] = jacdifr[1]
        efc_J_out[worldid, 0, sparseid5] = jacdifr[2]

        Jqvelp += jacdifp * qvel_in[worldid, da]
        Jqvelr += jacdifr * qvel_in[worldid, da]
        Jdotv_p += jacdifp_dot * qvel_in[worldid, da]
        Jdotv_r0 += jacdifr_dot * qvel_in[worldid, da]

        nnz += 1
    else:
      for dofid in range(nv):
        jacp1, jacr1 = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos1,
          body1,
          dofid,
          worldid,
        )
        jacp2, jacr2 = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          pos2,
          body2,
          dofid,
          worldid,
        )

        jacdifp = jacp1 - jacp2

        efc_J_out[worldid, efcid0, dofid] = jacdifp[0]
        efc_J_out[worldid, efcid1, dofid] = jacdifp[1]
        efc_J_out[worldid, efcid2, dofid] = jacdifp[2]

        jacdifr = (jacr1 - jacr2) * torquescale
        jacdifrq = math.mul_quat(math.quat_mul_axis(quat1, jacdifr), quat)
        jacdifr = 0.5 * wp.vec3(jacdifrq[1], jacdifrq[2], jacdifrq[3])

        jacp1_dot, jacr1_dot = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos1,
          body1,
          dofid,
          worldid,
        )
        jacp2_dot, jacr2_dot = support.jac_dot_dof(
          body_parentid,
          body_rootid,
          jnt_type,
          jnt_dofadr,
          dof_bodyid,
          dof_jntid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          cvel_in,
          cdof_dot_in,
          pos2,
          body2,
          dofid,
          worldid,
        )

        jacdifp_dot = jacp1_dot - jacp2_dot
        jacdifr_dot = jacr1_dot - jacr2_dot

        efc_J_out[worldid, efcid3, dofid] = jacdifr[0]
        efc_J_out[worldid, efcid4, dofid] = jacdifr[1]
        efc_J_out[worldid, efcid5, dofid] = jacdifr[2]

        Jqvelp += jacdifp * qvel_in[worldid, dofid]
        Jqvelr += jacdifr * qvel_in[worldid, dofid]
        Jdotv_p += jacdifp_dot * qvel_in[worldid, dofid]
        Jdotv_r0 += jacdifr_dot * qvel_in[worldid, dofid]

    # error is difference in global position and orientation
    cpos = pos1 - pos2

    crotq = math.mul_quat(quat1, quat)  # copy axis components
    crot = wp.vec3(crotq[1], crotq[2], crotq[3]) * torquescale

    body_invweight0_id = worldid % body_invweight0.shape[0]
    invweight_t = body_invweight0[body_invweight0_id, body1][0] + body_invweight0[body_invweight0_id, body2][0]

    pos_imp = wp.sqrt(wp.length_sq(cpos) + wp.length_sq(crot))

    solref = eq_solref[worldid % eq_solref.shape[0], eqid]
    solimp = eq_solimp[worldid % eq_solimp.shape[0], eqid]

    timestep = opt_timestep[worldid % opt_timestep.shape[0]]

    djrdv_q = wp.quat(0.0, Jdotv_r0[0], Jdotv_r0[1], Jdotv_r0[2])

    # Term 1: negqdot1 * domega * q0r
    t1a = math.mul_quat(negqdot1, domega_q)
    t1 = math.mul_quat(t1a, quat)

    # Term 2: negq1 * djrdv * q0r
    t2a = math.mul_quat(negq1, djrdv_q)
    t2 = math.mul_quat(t2a, quat)

    # Term 3: negq1 * domega * qdot0r
    t3a = math.mul_quat(negq1, domega_q)
    t3 = math.mul_quat(t3a, qdot0r)

    Jdotv_r = wp.vec3(t1[1] + t2[1] + t3[1], t1[2] + t2[2] + t3[2], t1[3] + t2[3] + t3[3]) * 0.5 * torquescale

    for i in range(3):
      _efc_row(
        opt_disableflags,
        worldid,
        timestep,
        efcid + i,
        cpos[i],
        pos_imp,
        invweight_t,
        solref,
        solimp,
        0.0,
        Jqvelp[i],
        0.0,
        ConstraintType.EQUALITY,
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

      efc_aref_out[worldid, efcid + i] -= Jdotv_p[i]

    invweight_r = body_invweight0[body_invweight0_id, body1][1] + body_invweight0[body_invweight0_id, body2][1]

    for i in range(3):
      _efc_row(
        opt_disableflags,
        worldid,
        timestep,
        efcid + 3 + i,
        crot[i],
        pos_imp,
        invweight_r,
        solref,
        solimp,
        0.0,
        Jqvelr[i],
        0.0,
        ConstraintType.EQUALITY,
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

      efc_aref_out[worldid, efcid + 3 + i] -= Jdotv_r[i]

  return kernel


@cache_kernel
def _equality_flexstrain(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    body_parentid: wp.array[int],
    body_rootid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    body_invweight0: wp.array2d[wp.vec2],
    dof_bodyid: wp.array[int],
    flex_interp: wp.array[int],
    flex_cellnum: wp.array[wp.vec3i],
    flex_nodeadr: wp.array[int],
    flex_nodenum: wp.array[int],
    flex_stiffnessadr: wp.array[int],
    flex_nodebodyid: wp.array[int],
    flex_node: wp.array[wp.vec3],
    flex_node0: wp.array[wp.vec3],
    flex_stiffness: wp.array[float],
    flex_centered: wp.array[bool],
    eq_obj1id: wp.array[int],
    eq_solref: wp.array2d[wp.vec2],
    eq_solimp: wp.array2d[vec5],
    eq_data: wp.array2d[vec11],
    body_isdofancestor: wp.array2d[int],
    eq_flexstrain_adr: wp.array[int],
    flexstrain_J_rownnz: wp.array[int],
    flexstrain_J_rowadr: wp.array[int],
    flexstrain_J_colind: wp.array[int],
    # Data in:
    qvel_in: wp.array2d[float],
    eq_active_in: wp.array2d[bool],
    xpos_in: wp.array2d[wp.vec3],
    xmat_in: wp.array2d[wp.mat33],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    njmax_in: int,
    njmax_nnz_in: int,
    flexnode_xpos_in: wp.array2d[wp.vec3],
    # Data out:
    ne_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, eqstrainid = wp.tid()
    eqid = eq_flexstrain_adr[eqstrainid]

    if not eq_active_in[worldid, eqid]:
      return

    f = eq_obj1id[eqid]
    order = flex_interp[f]
    if order <= 0:
      return

    # nodes per cell
    npc = (order + 1) * (order + 1) * (order + 1)

    # cell indices from eq_data
    data = eq_data[worldid % eq_data.shape[0], eqid]
    ci = int(data[0])
    cj = int(data[1])
    ck = int(data[2])

    cellnum = flex_cellnum[f]
    cy = cellnum[1]
    cz = cellnum[2]
    nstart = flex_nodeadr[f]
    ny_g = cy * order + 1
    nz_g = cz * order + 1

    ndof_cell = 3 * npc

    # read eigenmode data from flex_stiffness
    cell_idx = ci * cy * cz + cj * cz + ck
    k_base = flex_stiffnessadr[f] + cell_idx * ndof_cell * ndof_cell
    neig = int(flex_stiffness[k_base])

    if neig == 0:
      return

    solref = eq_solref[worldid % eq_solref.shape[0], eqid]
    solimp = eq_solimp[worldid % eq_solimp.shape[0], eqid]

    # gather cell node positions and reference positions (order==1: npc=8)
    # We compute the corotational quaternion from the deformation gradient
    # at the cell center (0.5, 0.5, 0.5)

    cell_quat = support.compute_interp_cell_quat(flexnode_xpos_in, order, ci, cj, ck, cy, cz, ny_g, nz_g, nstart, worldid)
    cell_quat_inv = wp.quat(-cell_quat[0], -cell_quat[1], -cell_quat[2], cell_quat[3])

    # Compute average invweight across cell nodes (translation component)
    avg_invweight = float(0.0)
    idx_iw = int(0)
    for li_iw in range(order + 1):
      for lj_iw in range(order + 1):
        for lk_iw in range(order + 1):
          if idx_iw < npc:
            gi_iw = ci * order + li_iw
            gj_iw = cj * order + lj_iw
            gk_iw = ck * order + lk_iw
            gidx_iw = gi_iw * ny_g * nz_g + gj_iw * nz_g + gk_iw
            bodyid_iw = flex_nodebodyid[nstart + gidx_iw]
            avg_invweight += body_invweight0[worldid % body_invweight0.shape[0], bodyid_iw][0]
            idx_iw += 1
    avg_invweight /= float(npc)

    # Loop over eigenmodes
    for eig in range(neig):
      wp.atomic_add(ne_out, worldid, 1)
      efcid = wp.atomic_add(nefc_out, worldid, 1)

      if efcid >= njmax_in:
        return

      if wp.static(is_sparse and newton):
        jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
        efc_jtdaj_adr_out[worldid, jgid] = efcid
        efc_jtdaj_nrow_out[worldid, jgid] = 6

      # Read eigenvector from stiffness data
      eigvec_base = k_base + 1 + eig * ndof_cell

      # Compute constraint residual: dot(eigvec, displacement_in_corot_frame)
      residual = float(0.0)
      idx2 = int(0)
      for li2 in range(order + 1):
        for lj2 in range(order + 1):
          for lk2 in range(order + 1):
            if idx2 < npc:
              gi2 = ci * order + li2
              gj2 = cj * order + lj2
              gk2 = ck * order + lk2
              gidx2 = gi2 * ny_g * nz_g + gj2 * nz_g + gk2

              xpos_n = flexnode_xpos_in[worldid, nstart + gidx2]
              refpos_n = flex_node0[nstart + gidx2]

              # Rotate xpos to corotational frame
              xrot = wp.quat_rotate(cell_quat_inv, xpos_n)
              displ = wp.vec3(xrot[0] - refpos_n[0], xrot[1] - refpos_n[1], xrot[2] - refpos_n[2])

              for c in range(3):
                residual += flex_stiffness[eigvec_base + 3 * idx2 + c] * displ[c]

              idx2 += 1

      efc_pos_out[worldid, efcid] = residual

      # Compute Jacobian: J_q = dSdx_world * dxpos/dq
      # dSdx_world = R_inv * eigvec (rotate eigenvector to world frame)
      # Then contract with per-node body Jacobians

      Jqvel = float(0.0)
      invweight = float(0.0)

      rownnz = flexstrain_J_rownnz[eqstrainid]
      fs_rowadr = flexstrain_J_rowadr[eqstrainid]

      efc_rowadr = int(0)
      if wp.static(is_sparse):
        efc_J_rownnz_out[worldid, efcid] = rownnz
        efc_rowadr = wp.atomic_add(efc_nnz_out, worldid, rownnz)
        if efc_rowadr + rownnz > njmax_nnz_in:
          return
        efc_J_rowadr_out[worldid, efcid] = efc_rowadr
      else:
        for q in range(nv):
          efc_J_out[worldid, efcid, q] = 0.0

      for sparseid in range(rownnz):
        q = flexstrain_J_colind[fs_rowadr + sparseid]
        J_val = float(0.0)
        idx3 = int(0)
        for li3 in range(order + 1):
          for lj3 in range(order + 1):
            for lk3 in range(order + 1):
              if idx3 < npc:
                gi3 = ci * order + li3
                gj3 = cj * order + lj3
                gk3 = ck * order + lk3
                gidx3 = gi3 * ny_g * nz_g + gj3 * nz_g + gk3

                bodyid3 = flex_nodebodyid[nstart + gidx3]
                xpos_n3 = flexnode_xpos_in[worldid, nstart + gidx3]

                if body_isdofancestor[bodyid3, q] != 0:
                  # Rotate eigenvector component to world frame
                  eig_local = wp.vec3(
                    flex_stiffness[eigvec_base + 3 * idx3 + 0],
                    flex_stiffness[eigvec_base + 3 * idx3 + 1],
                    flex_stiffness[eigvec_base + 3 * idx3 + 2],
                  )
                  eig_world = wp.quat_rotate(cell_quat, eig_local)

                  jacp, jacr = support.jac_dof(
                    body_parentid,
                    body_rootid,
                    dof_bodyid,
                    body_isdofancestor,
                    subtree_com_in,
                    cdof_in,
                    xpos_n3,
                    bodyid3,
                    q,
                    worldid,
                  )
                  J_val += wp.dot(eig_world, jacp)
                idx3 += 1

        if wp.static(is_sparse):
          efc_J_colind_out[worldid, 0, efc_rowadr + sparseid] = q
          efc_J_out[worldid, 0, efc_rowadr + sparseid] = J_val
        else:
          efc_J_out[worldid, efcid, q] = J_val
        Jqvel += J_val * qvel_in[worldid, q]

      invweight = avg_invweight

      _efc_row(
        opt_disableflags,
        worldid,
        opt_timestep[worldid % opt_timestep.shape[0]],
        efcid,
        residual,
        residual,
        invweight,
        solref,
        solimp,
        0.0,
        Jqvel,
        0.0,
        ConstraintType.EQUALITY,
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

  return kernel


@cache_kernel
def _friction_dof(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    dof_solref: wp.array2d[wp.vec2],
    dof_solimp: wp.array2d[vec5],
    dof_frictionloss: wp.array2d[float],
    dof_invweight0: wp.array2d[float],
    # Data in:
    qvel_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    nf_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, dofid = wp.tid()

    dof_frictionloss_id = worldid % dof_frictionloss.shape[0]

    if dof_frictionloss[dof_frictionloss_id, dofid] <= 0.0:
      return

    wp.atomic_add(nf_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 1

    if wp.static(is_sparse):
      efc_J_rownnz_out[worldid, efcid] = 1
      rowadr = wp.atomic_add(efc_nnz_out, worldid, 1)
      if rowadr + 1 > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid] = rowadr
      efc_J_colind_out[worldid, 0, rowadr] = dofid
      efc_J_out[worldid, 0, rowadr] = 1.0
    else:
      for i in range(nv):
        efc_J_out[worldid, efcid, i] = 0.0
      efc_J_out[worldid, efcid, dofid] = 1.0

    Jqvel = qvel_in[worldid, dofid]

    dof_invweight0_id = worldid % dof_invweight0.shape[0]
    dof_solref_id = worldid % dof_solref.shape[0]
    dof_solimp_id = worldid % dof_solimp.shape[0]
    _efc_row(
      opt_disableflags,
      worldid,
      opt_timestep[worldid % opt_timestep.shape[0]],
      efcid,
      0.0,
      0.0,
      dof_invweight0[dof_invweight0_id, dofid],
      dof_solref[dof_solref_id, dofid],
      dof_solimp[dof_solimp_id, dofid],
      0.0,
      Jqvel,
      dof_frictionloss[dof_frictionloss_id, dofid],
      ConstraintType.FRICTION_DOF,
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

  return kernel


@cache_kernel
def _friction_tendon(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    ten_J_rownnz: wp.array[int],
    ten_J_rowadr: wp.array[int],
    ten_J_colind: wp.array[int],
    tendon_solref_fri: wp.array2d[wp.vec2],
    tendon_solimp_fri: wp.array2d[vec5],
    tendon_frictionloss: wp.array2d[float],
    tendon_invweight0: wp.array2d[float],
    # Data in:
    qvel_in: wp.array2d[float],
    ten_J_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    nf_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, tenid = wp.tid()

    tendon_frictionloss_id = worldid % tendon_frictionloss.shape[0]

    frictionloss = tendon_frictionloss[tendon_frictionloss_id, tenid]
    if frictionloss <= 0.0:
      return

    wp.atomic_add(nf_out, worldid, 1)
    efcid = wp.atomic_add(nefc_out, worldid, 1)

    if efcid >= njmax_in:
      return

    if wp.static(is_sparse and newton):
      jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
      efc_jtdaj_adr_out[worldid, jgid] = efcid
      efc_jtdaj_nrow_out[worldid, jgid] = 1

    Jqvel = float(0.0)

    rownnz_tenJ = ten_J_rownnz[tenid]
    rowadr_tenJ = ten_J_rowadr[tenid]
    if wp.static(is_sparse):
      efc_J_rownnz_out[worldid, efcid] = rownnz_tenJ
      rowadr_efc = wp.atomic_add(efc_nnz_out, worldid, rownnz_tenJ)
      if rowadr_efc + rownnz_tenJ > njmax_nnz_in:
        return
      efc_J_rowadr_out[worldid, efcid] = rowadr_efc

      for i in range(rownnz_tenJ):
        sparseid_ten = rowadr_tenJ + i
        sparseid_efc = rowadr_efc + i
        colind = ten_J_colind[sparseid_ten]
        J = ten_J_in[worldid, sparseid_ten]
        efc_J_colind_out[worldid, 0, sparseid_efc] = colind
        efc_J_out[worldid, 0, sparseid_efc] = J
        Jqvel += J * qvel_in[worldid, colind]
    else:
      nnz = int(0)
      colind = ten_J_colind[rowadr_tenJ]
      for i in range(nv):
        if nnz < rownnz_tenJ and i == colind:
          J = ten_J_in[worldid, rowadr_tenJ + nnz]
          efc_J_out[worldid, efcid, i] = J
          Jqvel += J * qvel_in[worldid, i]
          nnz += 1
          if nnz < rownnz_tenJ:
            colind = ten_J_colind[rowadr_tenJ + nnz]
        else:
          efc_J_out[worldid, efcid, i] = 0.0

    tendon_invweight0_id = worldid % tendon_invweight0.shape[0]
    tendon_solref_fri_id = worldid % tendon_solref_fri.shape[0]
    tendon_solimp_fri_id = worldid % tendon_solimp_fri.shape[0]
    _efc_row(
      opt_disableflags,
      worldid,
      opt_timestep[worldid % opt_timestep.shape[0]],
      efcid,
      0.0,
      0.0,
      tendon_invweight0[tendon_invweight0_id, tenid],
      tendon_solref_fri[tendon_solref_fri_id, tenid],
      tendon_solimp_fri[tendon_solimp_fri_id, tenid],
      0.0,
      Jqvel,
      frictionloss,
      ConstraintType.FRICTION_TENDON,
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

  return kernel


@cache_kernel
def _limit_slide_hinge(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    jnt_qposadr: wp.array[int],
    jnt_dofadr: wp.array[int],
    jnt_solref: wp.array2d[wp.vec2],
    jnt_solimp: wp.array2d[vec5],
    jnt_range: wp.array2d[wp.vec2],
    jnt_margin: wp.array2d[float],
    dof_invweight0: wp.array2d[float],
    jnt_limited_slide_hinge_adr: wp.array[int],
    # Data in:
    qpos_in: wp.array2d[float],
    qvel_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    nl_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, jntlimitedid = wp.tid()
    jntid = jnt_limited_slide_hinge_adr[jntlimitedid]
    jnt_range_id = worldid % jnt_range.shape[0]
    jntrange = jnt_range[jnt_range_id, jntid]

    qpos = qpos_in[worldid, jnt_qposadr[jntid]]
    jnt_margin_id = worldid % jnt_margin.shape[0]
    jntmargin = jnt_margin[jnt_margin_id, jntid]
    dist_min, dist_max = qpos - jntrange[0], jntrange[1] - qpos
    pos = wp.min(dist_min, dist_max) - jntmargin
    active = pos < 0

    if active:
      wp.atomic_add(nl_out, worldid, 1)
      efcid = wp.atomic_add(nefc_out, worldid, 1)

      if efcid >= njmax_in:
        return

      if wp.static(is_sparse and newton):
        jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
        efc_jtdaj_adr_out[worldid, jgid] = efcid
        efc_jtdaj_nrow_out[worldid, jgid] = 1

      dofadr = jnt_dofadr[jntid]

      J = float(dist_min < dist_max) * 2.0 - 1.0

      if wp.static(is_sparse):
        efc_J_rownnz_out[worldid, efcid] = 1
        rowadr = wp.atomic_add(efc_nnz_out, worldid, 1)
        if rowadr + 1 > njmax_nnz_in:
          return
        efc_J_rowadr_out[worldid, efcid] = rowadr
        efc_J_colind_out[worldid, 0, rowadr] = dofadr
        efc_J_out[worldid, 0, rowadr] = J
      else:
        for i in range(nv):
          efc_J_out[worldid, efcid, i] = 0.0
        efc_J_out[worldid, efcid, dofadr] = J

      Jqvel = J * qvel_in[worldid, dofadr]

      dof_invweight0_id = worldid % dof_invweight0.shape[0]
      jnt_solref_id = worldid % jnt_solref.shape[0]
      jnt_solimp_id = worldid % jnt_solimp.shape[0]
      _efc_row(
        opt_disableflags,
        worldid,
        opt_timestep[worldid % opt_timestep.shape[0]],
        efcid,
        pos,
        pos,
        dof_invweight0[dof_invweight0_id, dofadr],
        jnt_solref[jnt_solref_id, jntid],
        jnt_solimp[jnt_solimp_id, jntid],
        jntmargin,
        Jqvel,
        0.0,
        ConstraintType.LIMIT_JOINT,
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

  return kernel


@cache_kernel
def _limit_ball(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    jnt_qposadr: wp.array[int],
    jnt_dofadr: wp.array[int],
    jnt_solref: wp.array2d[wp.vec2],
    jnt_solimp: wp.array2d[vec5],
    jnt_range: wp.array2d[wp.vec2],
    jnt_margin: wp.array2d[float],
    dof_invweight0: wp.array2d[float],
    jnt_limited_ball_adr: wp.array[int],
    # Data in:
    qpos_in: wp.array2d[float],
    qvel_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    nl_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, jntlimitedid = wp.tid()
    jntid = jnt_limited_ball_adr[jntlimitedid]
    qposadr = jnt_qposadr[jntid]

    qpos = qpos_in[worldid]
    jnt_quat = wp.quat(qpos[qposadr + 0], qpos[qposadr + 1], qpos[qposadr + 2], qpos[qposadr + 3])
    jnt_quat = wp.normalize(jnt_quat)
    axis_angle = math.quat_to_vel(jnt_quat)
    jnt_range_id = worldid % jnt_range.shape[0]
    jntrange = jnt_range[jnt_range_id, jntid]
    axis, angle = math.normalize_with_norm(axis_angle)
    jnt_margin_id = worldid % jnt_margin.shape[0]
    jntmargin = jnt_margin[jnt_margin_id, jntid]

    pos = wp.max(jntrange[0], jntrange[1]) - angle - jntmargin
    active = pos < 0

    if active:
      wp.atomic_add(nl_out, worldid, 1)
      efcid = wp.atomic_add(nefc_out, worldid, 1)

      if efcid >= njmax_in:
        return

      if wp.static(is_sparse and newton):
        jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
        efc_jtdaj_adr_out[worldid, jgid] = efcid
        efc_jtdaj_nrow_out[worldid, jgid] = 1

      dofadr = jnt_dofadr[jntid]
      dof0 = dofadr + 0
      dof1 = dofadr + 1
      dof2 = dofadr + 2

      if wp.static(is_sparse):
        efc_J_rownnz_out[worldid, efcid] = 3
        rowadr = wp.atomic_add(efc_nnz_out, worldid, 3)
        if rowadr + 3 > njmax_nnz_in:
          return
        efc_J_rowadr_out[worldid, efcid] = rowadr

        sparseid0 = rowadr + 0
        sparseid1 = rowadr + 1
        sparseid2 = rowadr + 2

        efc_J_colind_out[worldid, 0, sparseid0] = dof0
        efc_J_colind_out[worldid, 0, sparseid1] = dof1
        efc_J_colind_out[worldid, 0, sparseid2] = dof2

        efc_J_out[worldid, 0, sparseid0] = -axis[0]
        efc_J_out[worldid, 0, sparseid1] = -axis[1]
        efc_J_out[worldid, 0, sparseid2] = -axis[2]
      else:
        for i in range(nv):
          efc_J_out[worldid, efcid, i] = 0.0
        efc_J_out[worldid, efcid, dof0] = -axis[0]
        efc_J_out[worldid, efcid, dof1] = -axis[1]
        efc_J_out[worldid, efcid, dof2] = -axis[2]

      Jqvel = -axis[0] * qvel_in[worldid, dof0]
      Jqvel -= axis[1] * qvel_in[worldid, dof1]
      Jqvel -= axis[2] * qvel_in[worldid, dof2]

      dof_invweight0_id = worldid % dof_invweight0.shape[0]
      jnt_solref_id = worldid % jnt_solref.shape[0]
      jnt_solimp_id = worldid % jnt_solimp.shape[0]
      _efc_row(
        opt_disableflags,
        worldid,
        opt_timestep[worldid % opt_timestep.shape[0]],
        efcid,
        pos,
        pos,
        dof_invweight0[dof_invweight0_id, dofadr],
        jnt_solref[jnt_solref_id, jntid],
        jnt_solimp[jnt_solimp_id, jntid],
        jntmargin,
        Jqvel,
        0.0,
        ConstraintType.LIMIT_JOINT,
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

  return kernel


@cache_kernel
def _limit_tendon(is_sparse: bool, newton: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    ten_J_rownnz: wp.array[int],
    ten_J_rowadr: wp.array[int],
    ten_J_colind: wp.array[int],
    tendon_solref_lim: wp.array2d[wp.vec2],
    tendon_solimp_lim: wp.array2d[vec5],
    tendon_range: wp.array2d[wp.vec2],
    tendon_margin: wp.array2d[float],
    tendon_invweight0: wp.array2d[float],
    tendon_limited_adr: wp.array[int],
    # Data in:
    qvel_in: wp.array2d[float],
    ten_J_in: wp.array2d[float],
    ten_length_in: wp.array2d[float],
    njmax_in: int,
    njmax_nnz_in: int,
    # Data out:
    nl_out: wp.array[int],
    nefc_out: wp.array[int],
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    worldid, tenlimitedid = wp.tid()
    tenid = tendon_limited_adr[tenlimitedid]

    tendon_range_id = worldid % tendon_range.shape[0]
    tenrange = tendon_range[tendon_range_id, tenid]
    length = ten_length_in[worldid, tenid]
    dist_min, dist_max = length - tenrange[0], tenrange[1] - length
    tendon_margin_id = worldid % tendon_margin.shape[0]
    tenmargin = tendon_margin[tendon_margin_id, tenid]
    pos = wp.min(dist_min, dist_max) - tenmargin
    active = pos < 0

    if active:
      wp.atomic_add(nl_out, worldid, 1)
      efcid = wp.atomic_add(nefc_out, worldid, 1)

      if efcid >= njmax_in:
        return

      if wp.static(is_sparse and newton):
        jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
        efc_jtdaj_adr_out[worldid, jgid] = efcid
        efc_jtdaj_nrow_out[worldid, jgid] = 1

      Jqvel = float(0.0)
      scl = float(dist_min < dist_max) * 2.0 - 1.0

      rownnz_tenJ = ten_J_rownnz[tenid]
      rowadr_tenJ = ten_J_rowadr[tenid]
      if wp.static(is_sparse):
        efc_J_rownnz_out[worldid, efcid] = rownnz_tenJ
        rowadr_efc = wp.atomic_add(efc_nnz_out, worldid, rownnz_tenJ)
        if rowadr_efc + rownnz_tenJ > njmax_nnz_in:
          return
        efc_J_rowadr_out[worldid, efcid] = rowadr_efc

        for i in range(rownnz_tenJ):
          sparseid_ten = rowadr_tenJ + i
          sparseid_efc = rowadr_efc + i
          colind = ten_J_colind[sparseid_ten]
          J = scl * ten_J_in[worldid, sparseid_ten]
          efc_J_colind_out[worldid, 0, sparseid_efc] = colind
          efc_J_out[worldid, 0, sparseid_efc] = J
          Jqvel += J * qvel_in[worldid, colind]
      else:
        nnz = int(0)
        colind = ten_J_colind[rowadr_tenJ]
        for i in range(nv):
          if nnz < rownnz_tenJ and i == colind:
            J = scl * ten_J_in[worldid, rowadr_tenJ + nnz]
            efc_J_out[worldid, efcid, i] = J
            Jqvel += J * qvel_in[worldid, i]
            nnz += 1
            if nnz < rownnz_tenJ:
              colind = ten_J_colind[rowadr_tenJ + nnz]
          else:
            efc_J_out[worldid, efcid, i] = 0.0

      tendon_invweight0_id = worldid % tendon_invweight0.shape[0]
      tendon_solref_lim_id = worldid % tendon_solref_lim.shape[0]
      tendon_solimp_lim_id = worldid % tendon_solimp_lim.shape[0]
      _efc_row(
        opt_disableflags,
        worldid,
        opt_timestep[worldid % opt_timestep.shape[0]],
        efcid,
        pos,
        pos,
        tendon_invweight0[tendon_invweight0_id, tenid],
        tendon_solref_lim[tendon_solref_lim_id, tenid],
        tendon_solimp_lim[tendon_solimp_lim_id, tenid],
        tenmargin,
        Jqvel,
        0.0,
        ConstraintType.LIMIT_TENDON,
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

  return kernel


@wp.func
def _get_contact_bodies_and_weights(
  # Model:
  geom_bodyid: wp.array[int],
  flex_dim: wp.array[int],
  flex_cellnum: wp.array[wp.vec3i],
  flex_nodeadr: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_shelldataadr: wp.array[int],
  flex_nodebodyid: wp.array[int],
  flex_vertbodyid: wp.array[int],
  flex_elem: wp.array[int],
  flex_shell: wp.array[int],
  flex_vert0: wp.array[wp.vec3],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  # In:
  conid: int,
  side: int,
  geom: wp.vec2i,
  flex: wp.vec2i,
  elem: wp.vec2i,
  vert: wp.vec2i,
  con_pos: wp.vec3,
  worldid: int,
) -> Tuple[wp.vec4i, wp.vec4]:
  geom_id = geom[side]
  flex_id = flex[side]
  elem_id = elem[side]
  vert_id = vert[side]

  # Rigid Geom Side
  if geom_id >= 0:
    return wp.vec4i(geom_bodyid[geom_id], -1, -1, -1), wp.vec4(1.0, 0.0, 0.0, 0.0)

  # Plane-Vertex or Vertex-only flex contact
  flex_vert_start = flex_vertadr[flex_id]
  if vert_id >= 0:
    body = flex_vertbodyid[flex_vert_start + vert_id]
    if body >= 0:
      return wp.vec4i(body, -1, -1, -1), wp.vec4(1.0, 0.0, 0.0, 0.0)

    # Interpolated flex vertex contact: interpolate to the 8 corner nodes
    v_adr = flex_vert_start + vert_id
    coord = flex_vert0[v_adr]
    cn = flex_cellnum[flex_id]
    cx = cn[0]
    cy = cn[1]
    cz = cn[2]

    ci = wp.min(int(coord[0] * float(cx)), cx - 1)
    ci = wp.max(ci, 0)
    cj = wp.min(int(coord[1] * float(cy)), cy - 1)
    cj = wp.max(cj, 0)
    ck = wp.min(int(coord[2] * float(cz)), cz - 1)
    ck = wp.max(ck, 0)

    local_x = wp.clamp(coord[0] * float(cx) - float(ci), 0.0, 1.0)
    local_y = wp.clamp(coord[1] * float(cy) - float(cj), 0.0, 1.0)
    local_z = wp.clamp(coord[2] * float(cz) - float(ck), 0.0, 1.0)
    local = wp.vec3(local_x, local_y, local_z)

    ny_g = cy + 1
    nz_g = cz + 1
    nstart = flex_nodeadr[flex_id]

    body_ids = wp.vec4i(-1, -1, -1, -1)
    weights = wp.vec4(0.0, 0.0, 0.0, 0.0)
    n_collected = int(0)

    for li in range(2):
      for lj in range(2):
        for lk in range(2):
          w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
          if w > 1.0e-5 and n_collected < 4:
            gi = ci + li
            gj = cj + lj
            gk = ck + lk
            node_idx = gi * ny_g * nz_g + gj * nz_g + gk
            b = flex_nodebodyid[nstart + node_idx]

            # Put into body_ids and weights
            if n_collected == 0:
              body_ids = wp.vec4i(b, -1, -1, -1)
              weights = wp.vec4(w, 0.0, 0.0, 0.0)
            elif n_collected == 1:
              body_ids = wp.vec4i(body_ids[0], b, -1, -1)
              weights = wp.vec4(weights[0], w, 0.0, 0.0)
            elif n_collected == 2:
              body_ids = wp.vec4i(body_ids[0], body_ids[1], b, -1)
              weights = wp.vec4(weights[0], weights[1], w, 0.0)
            else:
              body_ids = wp.vec4i(body_ids[0], body_ids[1], body_ids[2], b)
              weights = wp.vec4(weights[0], weights[1], weights[2], w)

            n_collected += 1

    # Normalize weights so they sum to 1.0
    w_sum = weights[0] + weights[1] + weights[2] + weights[3]
    if w_sum > 1.0e-5:
      weights = wp.vec4(weights[0] / w_sum, weights[1] / w_sum, weights[2] / w_sum, weights[3] / w_sum)

    return body_ids, weights

  # Element contact: Retrieve local vertices
  dim = flex_dim[flex_id]

  if dim == 2:
    elem_data_start = flex_elemdataadr[flex_id] + elem_id * 3
    v0 = flex_elem[elem_data_start + 0]
    v1 = flex_elem[elem_data_start + 1]
    v2 = flex_elem[elem_data_start + 2]

    x0 = flexvert_xpos_in[worldid, flex_vert_start + v0]
    x1 = flexvert_xpos_in[worldid, flex_vert_start + v1]
    x2 = flexvert_xpos_in[worldid, flex_vert_start + v2]

    d0 = wp.length(con_pos - x0)
    d1 = wp.length(con_pos - x1)
    d2 = wp.length(con_pos - x2)

    w0 = 1.0 / wp.max(types.MJ_MINVAL, d0)
    w1 = 1.0 / wp.max(types.MJ_MINVAL, d1)
    w2 = 1.0 / wp.max(types.MJ_MINVAL, d2)

    w_sum = w0 + w1 + w2
    w0 = w0 / w_sum
    w1 = w1 / w_sum
    w2 = w2 / w_sum

    b0 = flex_vertbodyid[flex_vert_start + v0]
    b1 = flex_vertbodyid[flex_vert_start + v1]
    b2 = flex_vertbodyid[flex_vert_start + v2]

    return wp.vec4i(b0, b1, b2, -1), wp.vec4(w0, w1, w2, 0.0)

  elif dim == 3:
    elem_data_start = flex_elemdataadr[flex_id] + elem_id * 4
    v0 = flex_elem[elem_data_start + 0]
    v1 = flex_elem[elem_data_start + 1]
    v2 = flex_elem[elem_data_start + 2]
    v3 = flex_elem[elem_data_start + 3]

    x0 = flexvert_xpos_in[worldid, flex_vert_start + v0]
    x1 = flexvert_xpos_in[worldid, flex_vert_start + v1]
    x2 = flexvert_xpos_in[worldid, flex_vert_start + v2]
    x3 = flexvert_xpos_in[worldid, flex_vert_start + v3]

    d0 = wp.length(con_pos - x0)
    d1 = wp.length(con_pos - x1)
    d2 = wp.length(con_pos - x2)
    d3 = wp.length(con_pos - x3)

    w0 = 1.0 / wp.max(types.MJ_MINVAL, d0)
    w1 = 1.0 / wp.max(types.MJ_MINVAL, d1)
    w2 = 1.0 / wp.max(types.MJ_MINVAL, d2)
    w3 = 1.0 / wp.max(types.MJ_MINVAL, d3)

    w_sum = w0 + w1 + w2 + w3
    w0 = w0 / w_sum
    w1 = w1 / w_sum
    w2 = w2 / w_sum
    w3 = w3 / w_sum

    b0 = flex_vertbodyid[flex_vert_start + v0]
    if b0 >= 0:
      b1 = flex_vertbodyid[flex_vert_start + v1]
      b2 = flex_vertbodyid[flex_vert_start + v2]
      b3 = flex_vertbodyid[flex_vert_start + v3]
      return wp.vec4i(b0, b1, b2, b3), wp.vec4(w0, w1, w2, w3)
    else:
      # Interpolated 3D flex element contact: accumulate node weights and select top 4
      coord0 = flex_vert0[flex_vert_start + v0]
      cn = flex_cellnum[flex_id]
      cx = cn[0]
      cy = cn[1]
      cz = cn[2]

      ci = wp.min(int(coord0[0] * float(cx)), cx - 1)
      ci = wp.max(ci, 0)
      cj = wp.min(int(coord0[1] * float(cy)), cy - 1)
      cj = wp.max(cj, 0)
      ck = wp.min(int(coord0[2] * float(cz)), cz - 1)
      ck = wp.max(ck, 0)

      ny_g = cy + 1
      nz_g = cz + 1
      nstart = flex_nodeadr[flex_id]

      # 8 accumulated weights for the voxel corners
      W0 = float(0.0)
      W1 = float(0.0)
      W2 = float(0.0)
      W3 = float(0.0)
      W4 = float(0.0)
      W5 = float(0.0)
      W6 = float(0.0)
      W7 = float(0.0)

      # Loop over the 4 vertices of the tetrahedron
      for k in range(4):
        v_idx = v0
        w_k = w0
        if k == 1:
          v_idx = v1
          w_k = w1
        elif k == 2:
          v_idx = v2
          w_k = w2
        elif k == 3:
          v_idx = v3
          w_k = w3

        coord = flex_vert0[flex_vert_start + v_idx]
        local_x = wp.clamp(coord[0] * float(cx) - float(ci), 0.0, 1.0)
        local_y = wp.clamp(coord[1] * float(cy) - float(cj), 0.0, 1.0)
        local_z = wp.clamp(coord[2] * float(cz) - float(ck), 0.0, 1.0)
        local = wp.vec3(local_x, local_y, local_z)

        # Accumulate trilinear weights
        W0 += w_k * support.eval_basis_trilinear(local, 0)
        W1 += w_k * support.eval_basis_trilinear(local, 1)
        W2 += w_k * support.eval_basis_trilinear(local, 2)
        W3 += w_k * support.eval_basis_trilinear(local, 3)
        W4 += w_k * support.eval_basis_trilinear(local, 4)
        W5 += w_k * support.eval_basis_trilinear(local, 5)
        W6 += w_k * support.eval_basis_trilinear(local, 6)
        W7 += w_k * support.eval_basis_trilinear(local, 7)

      # 8 node body IDs corresponding to the 8 voxel corners
      b0_node = flex_nodebodyid[nstart + (ci + 0) * ny_g * nz_g + (cj + 0) * nz_g + (ck + 0)]
      b1_node = flex_nodebodyid[nstart + (ci + 0) * ny_g * nz_g + (cj + 0) * nz_g + (ck + 1)]
      b2_node = flex_nodebodyid[nstart + (ci + 0) * ny_g * nz_g + (cj + 1) * nz_g + (ck + 0)]
      b3_node = flex_nodebodyid[nstart + (ci + 0) * ny_g * nz_g + (cj + 1) * nz_g + (ck + 1)]
      b4_node = flex_nodebodyid[nstart + (ci + 1) * ny_g * nz_g + (cj + 0) * nz_g + (ck + 0)]
      b5_node = flex_nodebodyid[nstart + (ci + 1) * ny_g * nz_g + (cj + 0) * nz_g + (ck + 1)]
      b6_node = flex_nodebodyid[nstart + (ci + 1) * ny_g * nz_g + (cj + 1) * nz_g + (ck + 0)]
      b7_node = flex_nodebodyid[nstart + (ci + 1) * ny_g * nz_g + (cj + 1) * nz_g + (ck + 1)]

      W_mat = wp.mat33(W0, W1, W2, W3, W4, W5, W6, W7, -1.0)
      b_mat = wp.mat33(
        float(b0_node),
        float(b1_node),
        float(b2_node),
        float(b3_node),
        float(b4_node),
        float(b5_node),
        float(b6_node),
        float(b7_node),
        -1.0,
      )
      return support.select_top4_weights(W_mat, b_mat)

  else:
    return wp.vec4i(-1, -1, -1, -1), wp.vec4(0.0, 0.0, 0.0, 0.0)


# TODO(flex): Add interpolated flex self-collision support. Currently, interpolated
# flex contacts only handle flex-vs-geom contacts (e.g., flex on ground plane).
# Self-collision requires distributing both sides of the contact across cell nodes.

# TODO(flex): Replace inverse-distance contact weights with barycentric weights
# (mj_elemBodyWeight-style) for element contacts on interpolated flex.


@cache_kernel
def _efc_contact_init(cone_type: types.ConeType, is_sparse: bool, newton: bool):
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  IS_SPARSE = is_sparse

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_weldid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    dof_parentid: wp.array[int],
    geom_bodyid: wp.array[int],
    # Data in:
    njmax_in: int,
    njmax_nnz_in: int,
    nacon_in: wp.array[int],
    # In:
    dist_in: wp.array[float],
    condim_in: wp.array[int],
    includemargin_in: wp.array[float],
    worldid_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    type_in: wp.array[int],
    # Data out:
    nefc_out: wp.array[int],
    contact_efc_address_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    conid = wp.tid()

    if conid >= nacon_in[0]:
      return

    if not type_in[conid] & ContactType.CONSTRAINT:
      return

    condim = condim_in[conid]

    includemargin = includemargin_in[conid]
    pos = dist_in[conid] - includemargin
    active = pos < 0

    if not active:
      return

    if wp.static(IS_ELLIPTIC):
      ndim = condim
    else:
      if condim == 1:
        ndim = 1
      else:
        ndim = 2 * (condim - 1)

    worldid = worldid_in[conid]

    # Allocate contiguous block of efcids for all dimids
    base_efcid = wp.atomic_add(nefc_out, worldid, ndim)
    for dim in range(ndim):
      efcid = base_efcid + dim
      if efcid >= njmax_in:
        contact_efc_address_out[conid, dim] = -1
      else:
        contact_efc_address_out[conid, dim] = efcid
        # This is redundant with the _efc_row call later but needed for the jac calculation
        efc_id_out[worldid, efcid] = conid

    if wp.static(is_sparse and newton):
      if base_efcid < njmax_in:
        jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
        efc_jtdaj_adr_out[worldid, jgid] = base_efcid
        efc_jtdaj_nrow_out[worldid, jgid] = wp.min(ndim, njmax_in - base_efcid)

    if wp.static(IS_SPARSE):
      geom = geom_in[conid]
      body1 = body_weldid[geom_bodyid[geom[0]]]
      body2 = body_weldid[geom_bodyid[geom[1]]]

      da1 = int(body_dofadr[body1] + body_dofnum[body1] - 1)
      da2 = int(body_dofadr[body2] + body_dofnum[body2] - 1)

      # count non-zeros
      rownnz = int(0)
      while da1 >= 0 or da2 >= 0:
        da = wp.max(da1, da2)
        # skip common dofs
        if da1 == da and da2 == da:
          break
        if da1 == da:
          da1 = dof_parentid[da1]
        if da2 == da:
          da2 = dof_parentid[da2]
        rownnz += 1

      rowadr = wp.atomic_add(efc_nnz_out, worldid, rownnz * ndim)
      if rowadr + rownnz * ndim > njmax_nnz_in:
        return
      for dim in range(ndim):
        efcid = base_efcid + dim
        if efcid < njmax_in:
          efc_J_rowadr_out[worldid, efcid] = rowadr + dim * rownnz
          efc_J_rownnz_out[worldid, efcid] = rownnz

  return kernel


@cache_kernel
def _efc_contact_init_flex(cone_type: types.ConeType, is_sparse: bool, newton: bool):
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  IS_SPARSE = is_sparse
  HAS_FLEX = True

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_parentid: wp.array[int],
    body_weldid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    dof_parentid: wp.array[int],
    geom_bodyid: wp.array[int],
    flex_dim: wp.array[int],
    flex_interp: wp.array[int],
    flex_cellnum: wp.array[wp.vec3i],
    flex_nodeadr: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_shelldataadr: wp.array[int],
    flex_nodebodyid: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_shell: wp.array[int],
    flex_vert0: wp.array[wp.vec3],
    # Data in:
    flexvert_xpos_in: wp.array2d[wp.vec3],
    njmax_in: int,
    njmax_nnz_in: int,
    nacon_in: wp.array[int],
    # In:
    dist_in: wp.array[float],
    condim_in: wp.array[int],
    includemargin_in: wp.array[float],
    worldid_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    flex_in: wp.array[wp.vec2i],
    elem_in: wp.array[wp.vec2i],
    vert_in: wp.array[wp.vec2i],
    pos_in: wp.array[wp.vec3],
    type_in: wp.array[int],
    # Data out:
    nefc_out: wp.array[int],
    contact_efc_address_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_jtdaj_adr_out: wp.array2d[int],
    efc_jtdaj_nrow_out: wp.array2d[int],
    efc_jtdaj_nblock_out: wp.array[int],
    efc_J_rownnz_out: wp.array2d[int],
    efc_J_rowadr_out: wp.array2d[int],
    # Out:
    efc_nnz_out: wp.array[int],
  ):
    conid = wp.tid()

    if conid >= nacon_in[0]:
      return

    if not type_in[conid] & ContactType.CONSTRAINT:
      return

    condim = condim_in[conid]

    includemargin = includemargin_in[conid]
    pos = dist_in[conid] - includemargin
    active = pos < 0

    if not active:
      return

    if wp.static(IS_ELLIPTIC):
      ndim = condim
    else:
      if condim == 1:
        ndim = 1
      else:
        ndim = 2 * (condim - 1)

    worldid = worldid_in[conid]

    # Allocate contiguous block of efcids for all dimids
    base_efcid = wp.atomic_add(nefc_out, worldid, ndim)
    for dim in range(ndim):
      efcid = base_efcid + dim
      if efcid >= njmax_in:
        contact_efc_address_out[conid, dim] = -1
      else:
        contact_efc_address_out[conid, dim] = efcid
        # This is redundant with the _efc_row call later but needed for the jac calculation
        efc_id_out[worldid, efcid] = conid

    if wp.static(is_sparse and newton):
      if base_efcid < njmax_in:
        jgid = wp.atomic_add(efc_jtdaj_nblock_out, worldid, 1)
        efc_jtdaj_adr_out[worldid, jgid] = base_efcid
        efc_jtdaj_nrow_out[worldid, jgid] = wp.min(ndim, njmax_in - base_efcid)

    if wp.static(IS_SPARSE):
      geom = geom_in[conid]
      flex = flex_in[conid]
      elem = elem_in[conid]
      vert = vert_in[conid]
      con_pos = pos_in[conid]

      body_ids1, weights1 = _get_contact_bodies_and_weights(
        geom_bodyid,
        flex_dim,
        flex_cellnum,
        flex_nodeadr,
        flex_vertadr,
        flex_elemdataadr,
        flex_shelldataadr,
        flex_nodebodyid,
        flex_vertbodyid,
        flex_elem,
        flex_shell,
        flex_vert0,
        flexvert_xpos_in,
        conid,
        0,
        geom,
        flex,
        elem,
        vert,
        con_pos,
        worldid,
      )
      body_ids2, weights2 = _get_contact_bodies_and_weights(
        geom_bodyid,
        flex_dim,
        flex_cellnum,
        flex_nodeadr,
        flex_vertadr,
        flex_elemdataadr,
        flex_shelldataadr,
        flex_nodebodyid,
        flex_vertbodyid,
        flex_elem,
        flex_shell,
        flex_vert0,
        flexvert_xpos_in,
        conid,
        1,
        geom,
        flex,
        elem,
        vert,
        con_pos,
        worldid,
      )

      is_interp = False
      if wp.static(HAS_FLEX):
        if geom[0] < 0 and flex[0] >= 0 and (vert[0] >= 0 or elem[0] >= 0):
          if flex_interp[flex[0]] != 0:
            is_interp = True
        if geom[1] < 0 and flex[1] >= 0 and (vert[1] >= 0 or elem[1] >= 0):
          if flex_interp[flex[1]] != 0:
            is_interp = True

      if is_interp:
        # Interpolated flex: sum of all contributing body dofnums
        rownnz = int(0)
        for side in range(2):
          if geom[side] >= 0:
            b = body_weldid[geom_bodyid[geom[side]]]
            rownnz += body_dofnum[b]
          elif flex[side] >= 0 and vert[side] >= 0:
            f = flex[side]
            if flex_interp[f] != 0:
              # Compute parametric coordinate from flex_vert0
              v_adr = flex_vertadr[f] + vert[side]
              coord = flex_vert0[v_adr]
              cn = flex_cellnum[f]
              cx = cn[0]
              cy = cn[1]
              cz = cn[2]

              # Cell lookup
              ci = wp.min(int(coord[0] * float(cx)), cx - 1)
              ci = wp.max(ci, 0)
              cj = wp.min(int(coord[1] * float(cy)), cy - 1)
              cj = wp.max(cj, 0)
              ck = wp.min(int(coord[2] * float(cz)), cz - 1)
              ck = wp.max(ck, 0)

              # Local parametric coordinates
              local_x = wp.clamp(coord[0] * float(cx) - float(ci), 0.0, 1.0)
              local_y = wp.clamp(coord[1] * float(cy) - float(cj), 0.0, 1.0)
              local_z = wp.clamp(coord[2] * float(cz) - float(ck), 0.0, 1.0)
              local = wp.vec3(local_x, local_y, local_z)

              # Node grid dimensions
              ny_g = cy + 1
              nz_g = cz + 1
              nstart = flex_nodeadr[f]

              # Loop over 8 trilinear nodes
              for li in range(2):
                for lj in range(2):
                  for lk in range(2):
                    w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                    if w > 1.0e-5:
                      gi = ci + li
                      gj = cj + lj
                      gk = ck + lk
                      node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                      b = body_weldid[flex_nodebodyid[nstart + node_idx]]
                      rownnz += body_dofnum[b]
            else:
              b = body_weldid[flex_vertbodyid[flex_vertadr[f] + vert[side]]]
              rownnz += body_dofnum[b]
          elif flex[side] >= 0 and elem[side] >= 0:
            # Elem contact: use blended coordinate from distance weighting
            f = flex[side]
            e = elem[side]
            dim_f = flex_dim[f]
            edata_adr = flex_elemdataadr[f] + e * (dim_f + 1)
            vert_adr_f = flex_vertadr[f]
            contact_pos = pos_in[conid]

            total_inv_dist = float(0.0)
            blended_coord = wp.vec3(0.0, 0.0, 0.0)
            for vi in range(4):
              if vi <= dim_f:
                v_idx = flex_elem[edata_adr + vi]
                vpos = flexvert_xpos_in[worldid, vert_adr_f + v_idx]
                dist_v = wp.length(contact_pos - vpos)
                w_inv = 1.0 / wp.max(1.0e-10, dist_v)
                total_inv_dist += w_inv
                blended_coord += flex_vert0[vert_adr_f + v_idx] * w_inv
            if total_inv_dist > 1.0e-10:
              blended_coord = blended_coord / total_inv_dist

            if flex_interp[f] != 0:
              cn = flex_cellnum[f]
              cx = cn[0]
              cy = cn[1]
              cz = cn[2]
              ci = wp.min(int(blended_coord[0] * float(cx)), cx - 1)
              ci = wp.max(ci, 0)
              cj = wp.min(int(blended_coord[1] * float(cy)), cy - 1)
              cj = wp.max(cj, 0)
              ck = wp.min(int(blended_coord[2] * float(cz)), cz - 1)
              ck = wp.max(ck, 0)
              local_x = wp.clamp(blended_coord[0] * float(cx) - float(ci), 0.0, 1.0)
              local_y = wp.clamp(blended_coord[1] * float(cy) - float(cj), 0.0, 1.0)
              local_z = wp.clamp(blended_coord[2] * float(cz) - float(ck), 0.0, 1.0)
              local = wp.vec3(local_x, local_y, local_z)
              ny_g = cy + 1
              nz_g = cz + 1
              nstart = flex_nodeadr[f]
              for li in range(2):
                for lj in range(2):
                  for lk in range(2):
                    w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                    if w > 1.0e-5:
                      gi = ci + li
                      gj = cj + lj
                      gk = ck + lk
                      node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                      b = body_weldid[flex_nodebodyid[nstart + node_idx]]
                      rownnz += body_dofnum[b]
      else:
        # Standard path (including elements up to 4 bodies)
        b1_0 = body_weldid[body_ids1[0]]
        b1_1 = body_weldid[body_ids1[1]] if body_ids1[1] >= 0 else -1
        b1_2 = body_weldid[body_ids1[2]] if body_ids1[2] >= 0 else -1
        b1_3 = body_weldid[body_ids1[3]] if body_ids1[3] >= 0 else -1

        b2_0 = body_weldid[body_ids2[0]]
        b2_1 = body_weldid[body_ids2[1]] if body_ids2[1] >= 0 else -1
        b2_2 = body_weldid[body_ids2[2]] if body_ids2[2] >= 0 else -1
        b2_3 = body_weldid[body_ids2[3]] if body_ids2[3] >= 0 else -1

        dof1_0 = int(body_dofadr[b1_0] + body_dofnum[b1_0] - 1) if b1_0 >= 0 else -1
        dof1_1 = int(body_dofadr[b1_1] + body_dofnum[b1_1] - 1) if b1_1 >= 0 else -1
        dof1_2 = int(body_dofadr[b1_2] + body_dofnum[b1_2] - 1) if b1_2 >= 0 else -1
        dof1_3 = int(body_dofadr[b1_3] + body_dofnum[b1_3] - 1) if b1_3 >= 0 else -1

        dof2_0 = int(body_dofadr[b2_0] + body_dofnum[b2_0] - 1) if b2_0 >= 0 else -1
        dof2_1 = int(body_dofadr[b2_1] + body_dofnum[b2_1] - 1) if b2_1 >= 0 else -1
        dof2_2 = int(body_dofadr[b2_2] + body_dofnum[b2_2] - 1) if b2_2 >= 0 else -1
        dof2_3 = int(body_dofadr[b2_3] + body_dofnum[b2_3] - 1) if b2_3 >= 0 else -1

        # count non-zeros
        rownnz = int(0)
        while (
          dof1_0 >= 0 or dof1_1 >= 0 or dof1_2 >= 0 or dof1_3 >= 0 or dof2_0 >= 0 or dof2_1 >= 0 or dof2_2 >= 0 or dof2_3 >= 0
        ):
          da1_max = wp.max(dof1_0, wp.max(dof1_1, wp.max(dof1_2, dof1_3)))
          da2_max = wp.max(dof2_0, wp.max(dof2_1, wp.max(dof2_2, dof2_3)))
          da = wp.max(da1_max, da2_max)

          if dof1_0 == da:
            dof1_0 = dof_parentid[dof1_0]
          if dof1_1 == da:
            dof1_1 = dof_parentid[dof1_1]
          if dof1_2 == da:
            dof1_2 = dof_parentid[dof1_2]
          if dof1_3 == da:
            dof1_3 = dof_parentid[dof1_3]

          if dof2_0 == da:
            dof2_0 = dof_parentid[dof2_0]
          if dof2_1 == da:
            dof2_1 = dof_parentid[dof2_1]
          if dof2_2 == da:
            dof2_2 = dof_parentid[dof2_2]
          if dof2_3 == da:
            dof2_3 = dof_parentid[dof2_3]

          rownnz += 1

      rowadr = wp.atomic_add(efc_nnz_out, worldid, rownnz * ndim)
      if rowadr + rownnz * ndim > njmax_nnz_in:
        return
      for dim in range(ndim):
        efcid = base_efcid + dim
        if efcid < njmax_in:
          efc_J_rowadr_out[worldid, efcid] = rowadr + dim * rownnz
          efc_J_rownnz_out[worldid, efcid] = rownnz

  return kernel


@cache_kernel
def _efc_contact_jac_sparse(cone_type: types.ConeType):
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_parentid: wp.array[int],
    body_rootid: wp.array[int],
    body_weldid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    dof_bodyid: wp.array[int],
    dof_parentid: wp.array[int],
    geom_bodyid: wp.array[int],
    body_isdofancestor: wp.array2d[int],
    # Data in:
    qvel_in: wp.array2d[float],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    contact_efc_address_in: wp.array2d[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    nacon_in: wp.array[int],
    # In:
    condim_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    pos_in: wp.array[wp.vec3],
    frame_in: wp.array2d[wp.vec3],
    friction_in: wp.array2d[float],
    worldid_in: wp.array[int],
    # Data out:
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_Jqvel_out: wp.array2d[float],
  ):
    conid, dimid = wp.tid()

    if conid >= nacon_in[0]:
      return

    efcid = contact_efc_address_in[conid, dimid]
    if efcid < 0:
      return

    worldid = worldid_in[conid]
    condim = condim_in[conid]

    geom = geom_in[conid]
    body1 = body_weldid[geom_bodyid[geom[0]]]
    body2 = body_weldid[geom_bodyid[geom[1]]]

    con_pos = pos_in[conid]

    if not wp.static(IS_ELLIPTIC):
      frame_0 = frame_in[conid, 0]
      if condim > 1:
        dimid2 = dimid / 2 + 1
        frii = friction_in[conid, dimid2 - 1]

    da1 = int(body_dofadr[body1] + body_dofnum[body1] - 1)
    da2 = int(body_dofadr[body2] + body_dofnum[body2] - 1)
    da = wp.max(da1, da2)

    rowadr = efc_J_rowadr_in[worldid, efcid]
    rownnz = efc_J_rownnz_in[worldid, efcid]

    Jqvel = float(0.0)
    nnz = int(0)
    dofid = int(da)

    while True:
      if nnz >= rownnz:
        break

      if dofid == da:
        jac1p, jac1r = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          con_pos,
          body1,
          dofid,
          worldid,
        )
        jac2p, jac2r = support.jac_dof(
          body_parentid,
          body_rootid,
          dof_bodyid,
          body_isdofancestor,
          subtree_com_in,
          cdof_in,
          con_pos,
          body2,
          dofid,
          worldid,
        )

        jacp_dif = jac2p - jac1p
        jacr_dif = jac2r - jac1r

        if wp.static(IS_ELLIPTIC):
          J = float(0.0)
          if dimid < 3:
            frame_row = frame_in[conid, dimid]
            for xyz in range(3):
              J += frame_row[xyz] * jacp_dif[xyz]
          else:
            frame_row = frame_in[conid, dimid - 3]
            for xyz in range(3):
              J += frame_row[xyz] * jacr_dif[xyz]
        else:
          J = float(0.0)
          Ji = float(0.0)

          for xyz in range(3):
            J += frame_0[xyz] * jacp_dif[xyz]

            if condim > 1:
              if dimid2 < 3:
                Ji += frame_in[conid, dimid2][xyz] * jacp_dif[xyz]
              else:
                Ji += frame_in[conid, dimid2 - 3][xyz] * jacr_dif[xyz]

          if condim > 1:
            if dimid % 2 == 0:
              J += Ji * frii
            else:
              J -= Ji * frii

        sparseid = rowadr + nnz
        efc_J_colind_out[worldid, 0, sparseid] = dofid
        efc_J_out[worldid, 0, sparseid] = J
        nnz += 1
        Jqvel += J * qvel_in[worldid, dofid]

        # Advance tree pointers and recompute da for next iteration
        if da1 == da:
          da1 = dof_parentid[da1]
        if da2 == da:
          da2 = dof_parentid[da2]
        da = wp.max(da1, da2)
        dofid = da

    efc_Jqvel_out[worldid, efcid] = Jqvel

  return kernel


@cache_kernel
def _efc_contact_jac_sparse_flex(cone_type: types.ConeType):
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  HAS_FLEX = True

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_parentid: wp.array[int],
    body_rootid: wp.array[int],
    body_weldid: wp.array[int],
    body_dofnum: wp.array[int],
    body_dofadr: wp.array[int],
    dof_bodyid: wp.array[int],
    dof_parentid: wp.array[int],
    geom_bodyid: wp.array[int],
    flex_dim: wp.array[int],
    flex_interp: wp.array[int],
    flex_cellnum: wp.array[wp.vec3i],
    flex_nodeadr: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_shelldataadr: wp.array[int],
    flex_nodebodyid: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_shell: wp.array[int],
    flex_vert0: wp.array[wp.vec3],
    body_isdofancestor: wp.array2d[int],
    # Data in:
    qvel_in: wp.array2d[float],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    contact_efc_address_in: wp.array2d[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    nacon_in: wp.array[int],
    # In:
    condim_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    flex_in: wp.array[wp.vec2i],
    elem_in: wp.array[wp.vec2i],
    vert_in: wp.array[wp.vec2i],
    pos_in: wp.array[wp.vec3],
    frame_in: wp.array2d[wp.vec3],
    friction_in: wp.array2d[float],
    worldid_in: wp.array[int],
    # Data out:
    efc_J_colind_out: wp.array3d[int],
    efc_J_out: wp.array3d[float],
    efc_Jqvel_out: wp.array2d[float],
  ):
    conid, dimid = wp.tid()

    if conid >= nacon_in[0]:
      return

    efcid = contact_efc_address_in[conid, dimid]
    if efcid < 0:
      return

    worldid = worldid_in[conid]
    condim = condim_in[conid]

    geom = geom_in[conid]
    flex = flex_in[conid]
    elem = elem_in[conid]
    vert = vert_in[conid]
    con_pos = pos_in[conid]

    is_interp = False
    if wp.static(HAS_FLEX):
      if geom[0] < 0 and flex[0] >= 0 and (vert[0] >= 0 or elem[0] >= 0):
        if flex_interp[flex[0]] != 0:
          is_interp = True
      if geom[1] < 0 and flex[1] >= 0 and (vert[1] >= 0 or elem[1] >= 0):
        if flex_interp[flex[1]] != 0:
          is_interp = True

    if not wp.static(IS_ELLIPTIC):
      frame_0 = frame_in[conid, 0]
      if condim > 1:
        dimid2 = dimid / 2 + 1
        frii = friction_in[conid, dimid2 - 1]

    rowadr = efc_J_rowadr_in[worldid, efcid]
    rownnz = efc_J_rownnz_in[worldid, efcid]

    if is_interp:
      # Interpolated flex path: iterate over bodies per side, accumulate weighted Jacobians
      nnz = int(0)
      Jqvel = float(0.0)

      for side in range(2):
        sign = float(-1.0) if side == 0 else float(1.0)

        if geom[side] >= 0:
          # Geom side: single body
          b = body_weldid[geom_bodyid[geom[side]]]
          dof_start = body_dofadr[b]
          ndof = body_dofnum[b]
          for di in range(ndof):
            dofid = dof_start + di
            jacp, jacr = support.jac_dof(
              body_parentid,
              body_rootid,
              dof_bodyid,
              body_isdofancestor,
              subtree_com_in,
              cdof_in,
              con_pos,
              b,
              dofid,
              worldid,
            )

            if wp.static(IS_ELLIPTIC):
              J = float(0.0)
              if dimid < 3:
                frame_row = frame_in[conid, dimid]
                for xyz in range(3):
                  J += frame_row[xyz] * jacp[xyz] * sign
              else:
                frame_row = frame_in[conid, dimid - 3]
                for xyz in range(3):
                  J += frame_row[xyz] * jacr[xyz] * sign
            else:
              J = float(0.0)
              Ji = float(0.0)
              for xyz in range(3):
                J += frame_0[xyz] * jacp[xyz] * sign
                if condim > 1:
                  if dimid2 < 3:
                    Ji += frame_in[conid, dimid2][xyz] * jacp[xyz] * sign
                  else:
                    Ji += frame_in[conid, dimid2 - 3][xyz] * jacr[xyz] * sign
              if condim > 1:
                if dimid % 2 == 0:
                  J += Ji * frii
                else:
                  J -= Ji * frii

            if nnz < rownnz:
              sparseid = rowadr + nnz
              efc_J_colind_out[worldid, 0, sparseid] = dofid
              efc_J_out[worldid, 0, sparseid] = J
              Jqvel += J * qvel_in[worldid, dofid]
              nnz += 1

        elif flex[side] >= 0 and vert[side] >= 0:
          f = flex[side]
          if flex_interp[f] != 0:
            # Interpolated flex side: compute cell node bodies and weights
            v_adr = flex_vertadr[f] + vert[side]
            coord = flex_vert0[v_adr]
            cn = flex_cellnum[f]
            cx = cn[0]
            cy = cn[1]
            cz = cn[2]

            ci = wp.min(int(coord[0] * float(cx)), cx - 1)
            ci = wp.max(ci, 0)
            cj = wp.min(int(coord[1] * float(cy)), cy - 1)
            cj = wp.max(cj, 0)
            ck = wp.min(int(coord[2] * float(cz)), cz - 1)
            ck = wp.max(ck, 0)

            local_x = wp.clamp(coord[0] * float(cx) - float(ci), 0.0, 1.0)
            local_y = wp.clamp(coord[1] * float(cy) - float(cj), 0.0, 1.0)
            local_z = wp.clamp(coord[2] * float(cz) - float(ck), 0.0, 1.0)
            local = wp.vec3(local_x, local_y, local_z)

            ny_g = cy + 1
            nz_g = cz + 1
            nstart = flex_nodeadr[f]

            for li in range(2):
              for lj in range(2):
                for lk in range(2):
                  w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                  if w > 1.0e-5:
                    gi = ci + li
                    gj = cj + lj
                    gk = ck + lk
                    node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                    b = body_weldid[flex_nodebodyid[nstart + node_idx]]
                    w_sign = w * sign

                    dof_start = body_dofadr[b]
                    ndof = body_dofnum[b]
                    for di in range(ndof):
                      dofid = dof_start + di
                      jacp, jacr = support.jac_dof(
                        body_parentid,
                        body_rootid,
                        dof_bodyid,
                        body_isdofancestor,
                        subtree_com_in,
                        cdof_in,
                        con_pos,
                        b,
                        dofid,
                        worldid,
                      )

                      if wp.static(IS_ELLIPTIC):
                        J = float(0.0)
                        if dimid < 3:
                          frame_row = frame_in[conid, dimid]
                          for xyz in range(3):
                            J += frame_row[xyz] * jacp[xyz] * w_sign
                        else:
                          frame_row = frame_in[conid, dimid - 3]
                          for xyz in range(3):
                            J += frame_row[xyz] * jacr[xyz] * w_sign
                      else:
                        J = float(0.0)
                        Ji = float(0.0)
                        for xyz in range(3):
                          J += frame_0[xyz] * jacp[xyz] * w_sign
                          if condim > 1:
                            if dimid2 < 3:
                              Ji += frame_in[conid, dimid2][xyz] * jacp[xyz] * w_sign
                            else:
                              Ji += frame_in[conid, dimid2 - 3][xyz] * jacr[xyz] * w_sign
                        if condim > 1:
                          if dimid % 2 == 0:
                            J += Ji * frii
                          else:
                            J -= Ji * frii

                      if nnz < rownnz:
                        sparseid = rowadr + nnz
                        efc_J_colind_out[worldid, 0, sparseid] = dofid
                        efc_J_out[worldid, 0, sparseid] = J
                        Jqvel += J * qvel_in[worldid, dofid]
                        nnz += 1
          else:
            # Non-interpolated flex: single body
            b = body_weldid[flex_vertbodyid[flex_vertadr[f] + vert[side]]]
            dof_start = body_dofadr[b]
            ndof = body_dofnum[b]
            for di in range(ndof):
              dofid = dof_start + di
              jacp, jacr = support.jac_dof(
                body_parentid,
                body_rootid,
                dof_bodyid,
                body_isdofancestor,
                subtree_com_in,
                cdof_in,
                con_pos,
                b,
                dofid,
                worldid,
              )

              if wp.static(IS_ELLIPTIC):
                J = float(0.0)
                if dimid < 3:
                  frame_row = frame_in[conid, dimid]
                  for xyz in range(3):
                    J += frame_row[xyz] * jacp[xyz] * sign
                else:
                  frame_row = frame_in[conid, dimid - 3]
                  for xyz in range(3):
                    J += frame_row[xyz] * jacr[xyz] * sign
              else:
                J = float(0.0)
                Ji = float(0.0)
                for xyz in range(3):
                  J += frame_0[xyz] * jacp[xyz] * sign
                  if condim > 1:
                    if dimid2 < 3:
                      Ji += frame_in[conid, dimid2][xyz] * jacp[xyz] * sign
                    else:
                      Ji += frame_in[conid, dimid2 - 3][xyz] * jacr[xyz] * sign
                if condim > 1:
                  if dimid % 2 == 0:
                    J += Ji * frii
                  else:
                    J -= Ji * frii

              if nnz < rownnz:
                sparseid = rowadr + nnz
                efc_J_colind_out[worldid, 0, sparseid] = dofid
                efc_J_out[worldid, 0, sparseid] = J
                Jqvel += J * qvel_in[worldid, dofid]
                nnz += 1

        elif flex[side] >= 0 and elem[side] >= 0:
          # Elem contact: compute blended coordinate from distance weighting
          f = flex[side]
          e = elem[side]
          dim_f = flex_dim[f]
          edata_adr = flex_elemdataadr[f] + e * (dim_f + 1)
          vert_adr_f = flex_vertadr[f]

          total_inv_dist = float(0.0)
          blended_coord = wp.vec3(0.0, 0.0, 0.0)
          for vi in range(4):
            if vi <= dim_f:
              v_idx = flex_elem[edata_adr + vi]
              vpos = flexvert_xpos_in[worldid, vert_adr_f + v_idx]
              dist_v = wp.length(con_pos - vpos)
              w_inv = 1.0 / wp.max(1.0e-10, dist_v)
              total_inv_dist += w_inv
              blended_coord += flex_vert0[vert_adr_f + v_idx] * w_inv
          if total_inv_dist > 1.0e-10:
            blended_coord = blended_coord / total_inv_dist

          if flex_interp[f] != 0:
            cn = flex_cellnum[f]
            cx = cn[0]
            cy = cn[1]
            cz = cn[2]
            ci = wp.min(int(blended_coord[0] * float(cx)), cx - 1)
            ci = wp.max(ci, 0)
            cj = wp.min(int(blended_coord[1] * float(cy)), cy - 1)
            cj = wp.max(cj, 0)
            ck = wp.min(int(blended_coord[2] * float(cz)), cz - 1)
            ck = wp.max(ck, 0)
            local_x = wp.clamp(blended_coord[0] * float(cx) - float(ci), 0.0, 1.0)
            local_y = wp.clamp(blended_coord[1] * float(cy) - float(cj), 0.0, 1.0)
            local_z = wp.clamp(blended_coord[2] * float(cz) - float(ck), 0.0, 1.0)
            local = wp.vec3(local_x, local_y, local_z)
            ny_g = cy + 1
            nz_g = cz + 1
            nstart = flex_nodeadr[f]

            for li in range(2):
              for lj in range(2):
                for lk in range(2):
                  w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                  if w > 1.0e-5:
                    gi = ci + li
                    gj = cj + lj
                    gk = ck + lk
                    node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                    b = body_weldid[flex_nodebodyid[nstart + node_idx]]
                    w_sign = w * sign

                    dof_start = body_dofadr[b]
                    ndof = body_dofnum[b]
                    for di in range(ndof):
                      dofid = dof_start + di
                      jacp, jacr = support.jac_dof(
                        body_parentid,
                        body_rootid,
                        dof_bodyid,
                        body_isdofancestor,
                        subtree_com_in,
                        cdof_in,
                        con_pos,
                        b,
                        dofid,
                        worldid,
                      )

                      if wp.static(IS_ELLIPTIC):
                        J = float(0.0)
                        if dimid < 3:
                          frame_row = frame_in[conid, dimid]
                          for xyz in range(3):
                            J += frame_row[xyz] * jacp[xyz] * w_sign
                        else:
                          frame_row = frame_in[conid, dimid - 3]
                          for xyz in range(3):
                            J += frame_row[xyz] * jacr[xyz] * w_sign
                      else:
                        J = float(0.0)
                        Ji = float(0.0)
                        for xyz in range(3):
                          J += frame_0[xyz] * jacp[xyz] * w_sign
                          if condim > 1:
                            if dimid2 < 3:
                              Ji += frame_in[conid, dimid2][xyz] * jacp[xyz] * w_sign
                            else:
                              Ji += frame_in[conid, dimid2 - 3][xyz] * jacr[xyz] * w_sign
                        if condim > 1:
                          if dimid % 2 == 0:
                            J += Ji * frii
                          else:
                            J -= Ji * frii

                      if nnz < rownnz:
                        sparseid = rowadr + nnz
                        efc_J_colind_out[worldid, 0, sparseid] = dofid
                        efc_J_out[worldid, 0, sparseid] = J
                        Jqvel += J * qvel_in[worldid, dofid]
                        nnz += 1

      efc_Jqvel_out[worldid, efcid] = Jqvel

    else:
      # Non-interpolated flex path: use HEAD's 4-body lockstep tree-walk
      body_ids1, weights1 = _get_contact_bodies_and_weights(
        geom_bodyid,
        flex_dim,
        flex_cellnum,
        flex_nodeadr,
        flex_vertadr,
        flex_elemdataadr,
        flex_shelldataadr,
        flex_nodebodyid,
        flex_vertbodyid,
        flex_elem,
        flex_shell,
        flex_vert0,
        flexvert_xpos_in,
        conid,
        0,
        geom,
        flex,
        elem,
        vert,
        con_pos,
        worldid,
      )
      body_ids2, weights2 = _get_contact_bodies_and_weights(
        geom_bodyid,
        flex_dim,
        flex_cellnum,
        flex_nodeadr,
        flex_vertadr,
        flex_elemdataadr,
        flex_shelldataadr,
        flex_nodebodyid,
        flex_vertbodyid,
        flex_elem,
        flex_shell,
        flex_vert0,
        flexvert_xpos_in,
        conid,
        1,
        geom,
        flex,
        elem,
        vert,
        con_pos,
        worldid,
      )

      # skip fixed bodies
      b1_0 = body_weldid[body_ids1[0]]
      b1_1 = body_weldid[body_ids1[1]] if body_ids1[1] >= 0 else -1
      b1_2 = body_weldid[body_ids1[2]] if body_ids1[2] >= 0 else -1
      b1_3 = body_weldid[body_ids1[3]] if body_ids1[3] >= 0 else -1

      b2_0 = body_weldid[body_ids2[0]]
      b2_1 = body_weldid[body_ids2[1]] if body_ids2[1] >= 0 else -1
      b2_2 = body_weldid[body_ids2[2]] if body_ids2[2] >= 0 else -1
      b2_3 = body_weldid[body_ids2[3]] if body_ids2[3] >= 0 else -1

      dof1_0 = int(body_dofadr[b1_0] + body_dofnum[b1_0] - 1) if b1_0 >= 0 else -1
      dof1_1 = int(body_dofadr[b1_1] + body_dofnum[b1_1] - 1) if b1_1 >= 0 else -1
      dof1_2 = int(body_dofadr[b1_2] + body_dofnum[b1_2] - 1) if b1_2 >= 0 else -1
      dof1_3 = int(body_dofadr[b1_3] + body_dofnum[b1_3] - 1) if b1_3 >= 0 else -1
      da1 = wp.max(dof1_0, wp.max(dof1_1, wp.max(dof1_2, dof1_3)))

      dof2_0 = int(body_dofadr[b2_0] + body_dofnum[b2_0] - 1) if b2_0 >= 0 else -1
      dof2_1 = int(body_dofadr[b2_1] + body_dofnum[b2_1] - 1) if b2_1 >= 0 else -1
      dof2_2 = int(body_dofadr[b2_2] + body_dofnum[b2_2] - 1) if b2_2 >= 0 else -1
      dof2_3 = int(body_dofadr[b2_3] + body_dofnum[b2_3] - 1) if b2_3 >= 0 else -1
      da2 = wp.max(dof2_0, wp.max(dof2_1, wp.max(dof2_2, dof2_3)))

      da = wp.max(da1, da2)

      Jqvel = float(0.0)
      nnz = int(0)
      dofid = int(da)

      while True:
        if nnz >= rownnz:
          break

        if dofid == da:
          jac1p = wp.vec3(0.0)
          jac1r = wp.vec3(0.0)
          if dof1_0 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b1_0, dofid, worldid
            )
            jac1p += jp * weights1[0]
            jac1r += jr * weights1[0]
          if dof1_1 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b1_1, dofid, worldid
            )
            jac1p += jp * weights1[1]
            jac1r += jr * weights1[1]
          if dof1_2 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b1_2, dofid, worldid
            )
            jac1p += jp * weights1[2]
            jac1r += jr * weights1[2]
          if dof1_3 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b1_3, dofid, worldid
            )
            jac1p += jp * weights1[3]
            jac1r += jr * weights1[3]

          jac2p = wp.vec3(0.0)
          jac2r = wp.vec3(0.0)
          if dof2_0 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b2_0, dofid, worldid
            )
            jac2p += jp * weights2[0]
            jac2r += jr * weights2[0]
          if dof2_1 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b2_1, dofid, worldid
            )
            jac2p += jp * weights2[1]
            jac2r += jr * weights2[1]
          if dof2_2 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b2_2, dofid, worldid
            )
            jac2p += jp * weights2[2]
            jac2r += jr * weights2[2]
          if dof2_3 == da:
            jp, jr = support.jac_dof(
              body_parentid, body_rootid, dof_bodyid, body_isdofancestor, subtree_com_in, cdof_in, con_pos, b2_3, dofid, worldid
            )
            jac2p += jp * weights2[3]
            jac2r += jr * weights2[3]

          jacp_dif = jac2p - jac1p
          jacr_dif = jac2r - jac1r

          if wp.static(IS_ELLIPTIC):
            J = float(0.0)
            if dimid < 3:
              frame_row = frame_in[conid, dimid]
              for xyz in range(3):
                J += frame_row[xyz] * jacp_dif[xyz]
            else:
              frame_row = frame_in[conid, dimid - 3]
              for xyz in range(3):
                J += frame_row[xyz] * jacr_dif[xyz]
          else:
            J = float(0.0)
            Ji = float(0.0)

            for xyz in range(3):
              J += frame_0[xyz] * jacp_dif[xyz]

              if condim > 1:
                if dimid2 < 3:
                  Ji += frame_in[conid, dimid2][xyz] * jacp_dif[xyz]
                else:
                  Ji += frame_in[conid, dimid2 - 3][xyz] * jacr_dif[xyz]

            if condim > 1:
              if dimid % 2 == 0:
                J += Ji * frii
              else:
                J -= Ji * frii

          sparseid = rowadr + nnz
          efc_J_colind_out[worldid, 0, sparseid] = dofid
          efc_J_out[worldid, 0, sparseid] = J
          nnz += 1
          Jqvel += J * qvel_in[worldid, dofid]

          # Advance tree pointers
          if dof1_0 == da:
            dof1_0 = dof_parentid[dof1_0]
          if dof1_1 == da:
            dof1_1 = dof_parentid[dof1_1]
          if dof1_2 == da:
            dof1_2 = dof_parentid[dof1_2]
          if dof1_3 == da:
            dof1_3 = dof_parentid[dof1_3]
          da1 = wp.max(dof1_0, wp.max(dof1_1, wp.max(dof1_2, dof1_3)))

          if dof2_0 == da:
            dof2_0 = dof_parentid[dof2_0]
          if dof2_1 == da:
            dof2_1 = dof_parentid[dof2_1]
          if dof2_2 == da:
            dof2_2 = dof_parentid[dof2_2]
          if dof2_3 == da:
            dof2_3 = dof_parentid[dof2_3]
          da2 = wp.max(dof2_0, wp.max(dof2_1, wp.max(dof2_2, dof2_3)))

          da = wp.max(da1, da2)
          dofid = da

      efc_Jqvel_out[worldid, efcid] = Jqvel

  return kernel


@cache_kernel
def _efc_contact_jac_dense(tile_size: int, cone_type: types.ConeType):
  TILE_SIZE = tile_size
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_rootid: wp.array[int],
    geom_bodyid: wp.array[int],
    body_isdofancestor: wp.array2d[int],
    # Data in:
    ne_in: wp.array[int],
    nf_in: wp.array[int],
    nl_in: wp.array[int],
    nefc_in: wp.array[int],
    qvel_in: wp.array2d[float],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    contact_efc_address_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    njmax_in: int,
    # In:
    nv_padded: int,
    condim_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    pos_in: wp.array[wp.vec3],
    frame_in: wp.array2d[wp.vec3],
    friction_in: wp.array2d[float],
    # Data out:
    efc_J_out: wp.array3d[float],
    efc_Jqvel_out: wp.array2d[float],
  ):
    worldid, dof_block_id, tid = wp.tid()

    dof_start = dof_block_id * wp.static(TILE_SIZE)
    if dof_start >= nv_padded:
      return

    cdof_tile = wp.tile_load(cdof_in[worldid], shape=TILE_SIZE, offset=dof_start, bounds_check=True)
    qvel_tile = wp.tile_load(qvel_in[worldid], shape=TILE_SIZE, offset=dof_start, bounds_check=True)

    efcid_start = ne_in[worldid] + nf_in[worldid] + nl_in[worldid]
    efcid_end = wp.min(nefc_in[worldid], njmax_in)

    prev_conid = int(-1)
    condim = int(0)

    for efcid in range(efcid_start, efcid_end):
      conid = efc_id_in[worldid, efcid]

      # Recompute per-contact data only when contact changes
      if conid != prev_conid:
        prev_conid = conid
        condim = condim_in[conid]

        geom = geom_in[conid]
        body1 = geom_bodyid[geom[0]]
        body2 = geom_bodyid[geom[1]]

        con_pos = pos_in[conid]
        offset1 = con_pos - subtree_com_in[worldid, body_rootid[body1]]
        affects1_tile = wp.tile_load(body_isdofancestor[body1], shape=TILE_SIZE, offset=dof_start, bounds_check=True)
        jacp1_tile = wp.tile_map(support._compute_jacp, cdof_tile, offset1, affects1_tile)
        jacr1_tile = wp.tile_map(support._compute_jacr, cdof_tile, affects1_tile)

        offset2 = con_pos - subtree_com_in[worldid, body_rootid[body2]]
        affects2_tile = wp.tile_load(body_isdofancestor[body2], shape=TILE_SIZE, offset=dof_start, bounds_check=True)
        jacp2_tile = wp.tile_map(support._compute_jacp, cdof_tile, offset2, affects2_tile)
        jacr2_tile = wp.tile_map(support._compute_jacr, cdof_tile, affects2_tile)

        jacp_dif_tile = wp.tile_map(wp.sub, jacp2_tile, jacp1_tile)
        jacr_dif_tile = wp.tile_map(wp.sub, jacr2_tile, jacr1_tile)

        if not wp.static(IS_ELLIPTIC):
          frame_0 = frame_in[conid, 0]
          Ji_0p_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_0)

          if condim > 1:
            Ji_0r_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_0)
            frame_1 = frame_in[conid, 1]
            Ji_1p_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_1)
            Ji_1r_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_1)
            frame_2 = frame_in[conid, 2]
            Ji_2p_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_2)
            Ji_2r_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_2)

      if wp.static(IS_ELLIPTIC):
        dimid = efcid - contact_efc_address_in[conid, 0]
        if dimid < 3:
          frame_idx = dimid
        else:
          frame_idx = dimid - 3

        frame_row = frame_in[conid, frame_idx]

        if dimid < 3:
          J_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_row)
        else:
          J_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_row)
      else:
        J_tile = Ji_0p_tile
        if condim > 1:
          dimid = efcid - contact_efc_address_in[conid, 0]
          dimid2 = dimid / 2 + 1
          frii = friction_in[conid, dimid2 - 1]
          frii_sign = frii * (1.0 - 2.0 * float(dimid & 1))

          if dimid2 == 1:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_1p_tile, frii_sign))
          elif dimid2 == 2:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_2p_tile, frii_sign))
          elif dimid2 == 3:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_0r_tile, frii_sign))
          elif dimid2 == 4:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_1r_tile, frii_sign))
          else:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_2r_tile, frii_sign))

      wp.tile_store(efc_J_out[worldid, efcid], J_tile, offset=dof_start, bounds_check=True)

      Jqvel_tile = wp.tile_map(wp.mul, J_tile, qvel_tile)
      Jqvel_sum = wp.tile_reduce(wp.add, Jqvel_tile)
      if tid == 0:
        wp.atomic_add(efc_Jqvel_out[worldid], efcid, wp.tile_extract(Jqvel_sum, 0))

  return kernel


@cache_kernel
def _efc_contact_jac_dense_flex(tile_size: int, cone_type: types.ConeType):
  TILE_SIZE = tile_size
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_rootid: wp.array[int],
    geom_bodyid: wp.array[int],
    flex_dim: wp.array[int],
    flex_cellnum: wp.array[wp.vec3i],
    flex_nodeadr: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_shelldataadr: wp.array[int],
    flex_nodebodyid: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_shell: wp.array[int],
    flex_vert0: wp.array[wp.vec3],
    body_isdofancestor: wp.array2d[int],
    # Data in:
    ne_in: wp.array[int],
    nf_in: wp.array[int],
    nl_in: wp.array[int],
    nefc_in: wp.array[int],
    qvel_in: wp.array2d[float],
    subtree_com_in: wp.array2d[wp.vec3],
    cdof_in: wp.array2d[wp.spatial_vector],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    contact_efc_address_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    njmax_in: int,
    # In:
    nv_padded: int,
    condim_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    flex_in: wp.array[wp.vec2i],
    elem_in: wp.array[wp.vec2i],
    vert_in: wp.array[wp.vec2i],
    pos_in: wp.array[wp.vec3],
    frame_in: wp.array2d[wp.vec3],
    friction_in: wp.array2d[float],
    # Data out:
    efc_J_out: wp.array3d[float],
    efc_Jqvel_out: wp.array2d[float],
  ):
    worldid, dof_block_id, tid = wp.tid()

    dof_start = dof_block_id * wp.static(TILE_SIZE)
    if dof_start >= nv_padded:
      return

    cdof_tile = wp.tile_load(cdof_in[worldid], shape=TILE_SIZE, offset=dof_start, bounds_check=True)
    qvel_tile = wp.tile_load(qvel_in[worldid], shape=TILE_SIZE, offset=dof_start, bounds_check=True)

    efcid_start = ne_in[worldid] + nf_in[worldid] + nl_in[worldid]
    efcid_end = wp.min(nefc_in[worldid], njmax_in)

    prev_conid = int(-1)
    condim = int(0)

    for efcid in range(efcid_start, efcid_end):
      conid = efc_id_in[worldid, efcid]

      # Recompute per-contact data only when contact changes
      if conid != prev_conid:
        prev_conid = conid
        condim = condim_in[conid]

        geom = geom_in[conid]
        flex = flex_in[conid]
        elem = elem_in[conid]
        vert = vert_in[conid]
        con_pos = pos_in[conid]

        body_ids1, weights1 = _get_contact_bodies_and_weights(
          geom_bodyid,
          flex_dim,
          flex_cellnum,
          flex_nodeadr,
          flex_vertadr,
          flex_elemdataadr,
          flex_shelldataadr,
          flex_nodebodyid,
          flex_vertbodyid,
          flex_elem,
          flex_shell,
          flex_vert0,
          flexvert_xpos_in,
          conid,
          0,
          geom,
          flex,
          elem,
          vert,
          con_pos,
          worldid,
        )
        body_ids2, weights2 = _get_contact_bodies_and_weights(
          geom_bodyid,
          flex_dim,
          flex_cellnum,
          flex_nodeadr,
          flex_vertadr,
          flex_elemdataadr,
          flex_shelldataadr,
          flex_nodebodyid,
          flex_vertbodyid,
          flex_elem,
          flex_shell,
          flex_vert0,
          flexvert_xpos_in,
          conid,
          1,
          geom,
          flex,
          elem,
          vert,
          con_pos,
          worldid,
        )

        b1_0 = body_ids1[0]
        b1_1 = body_ids1[1]
        b1_2 = body_ids1[2]
        b1_3 = body_ids1[3]

        b2_0 = body_ids2[0]
        b2_1 = body_ids2[1]
        b2_2 = body_ids2[2]
        b2_3 = body_ids2[3]

        # Weighted jacp for side 1
        jacp1_tile = wp.tile_map(
          support._compute_jacp,
          cdof_tile,
          con_pos - subtree_com_in[worldid, body_rootid[b1_0]],
          wp.tile_load(body_isdofancestor[b1_0], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
        )
        jacp1_tile = wp.tile_map(wp.mul, jacp1_tile, weights1[0])
        if b1_1 >= 0:
          t_jacp = wp.tile_map(
            support._compute_jacp,
            cdof_tile,
            con_pos - subtree_com_in[worldid, body_rootid[b1_1]],
            wp.tile_load(body_isdofancestor[b1_1], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacp1_tile = wp.tile_map(wp.add, jacp1_tile, wp.tile_map(wp.mul, t_jacp, weights1[1]))
        if b1_2 >= 0:
          t_jacp = wp.tile_map(
            support._compute_jacp,
            cdof_tile,
            con_pos - subtree_com_in[worldid, body_rootid[b1_2]],
            wp.tile_load(body_isdofancestor[b1_2], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacp1_tile = wp.tile_map(wp.add, jacp1_tile, wp.tile_map(wp.mul, t_jacp, weights1[2]))
        if b1_3 >= 0:
          t_jacp = wp.tile_map(
            support._compute_jacp,
            cdof_tile,
            con_pos - subtree_com_in[worldid, body_rootid[b1_3]],
            wp.tile_load(body_isdofancestor[b1_3], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacp1_tile = wp.tile_map(wp.add, jacp1_tile, wp.tile_map(wp.mul, t_jacp, weights1[3]))

        # Weighted jacp for side 2
        jacp2_tile = wp.tile_map(
          support._compute_jacp,
          cdof_tile,
          con_pos - subtree_com_in[worldid, body_rootid[b2_0]],
          wp.tile_load(body_isdofancestor[b2_0], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
        )
        jacp2_tile = wp.tile_map(wp.mul, jacp2_tile, weights2[0])
        if b2_1 >= 0:
          t_jacp = wp.tile_map(
            support._compute_jacp,
            cdof_tile,
            con_pos - subtree_com_in[worldid, body_rootid[b2_1]],
            wp.tile_load(body_isdofancestor[b2_1], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacp2_tile = wp.tile_map(wp.add, jacp2_tile, wp.tile_map(wp.mul, t_jacp, weights2[1]))
        if b2_2 >= 0:
          t_jacp = wp.tile_map(
            support._compute_jacp,
            cdof_tile,
            con_pos - subtree_com_in[worldid, body_rootid[b2_2]],
            wp.tile_load(body_isdofancestor[b2_2], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacp2_tile = wp.tile_map(wp.add, jacp2_tile, wp.tile_map(wp.mul, t_jacp, weights2[2]))
        if b2_3 >= 0:
          t_jacp = wp.tile_map(
            support._compute_jacp,
            cdof_tile,
            con_pos - subtree_com_in[worldid, body_rootid[b2_3]],
            wp.tile_load(body_isdofancestor[b2_3], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacp2_tile = wp.tile_map(wp.add, jacp2_tile, wp.tile_map(wp.mul, t_jacp, weights2[3]))

        jacp_dif_tile = wp.tile_map(wp.sub, jacp2_tile, jacp1_tile)

        # Weighted jacr for side 1
        jacr1_tile = wp.tile_map(
          support._compute_jacr,
          cdof_tile,
          wp.tile_load(body_isdofancestor[b1_0], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
        )
        jacr1_tile = wp.tile_map(wp.mul, jacr1_tile, weights1[0])
        if b1_1 >= 0:
          t_jacr = wp.tile_map(
            support._compute_jacr,
            cdof_tile,
            wp.tile_load(body_isdofancestor[b1_1], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacr1_tile = wp.tile_map(wp.add, jacr1_tile, wp.tile_map(wp.mul, t_jacr, weights1[1]))
        if b1_2 >= 0:
          t_jacr = wp.tile_map(
            support._compute_jacr,
            cdof_tile,
            wp.tile_load(body_isdofancestor[b1_2], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacr1_tile = wp.tile_map(wp.add, jacr1_tile, wp.tile_map(wp.mul, t_jacr, weights1[2]))
        if b1_3 >= 0:
          t_jacr = wp.tile_map(
            support._compute_jacr,
            cdof_tile,
            wp.tile_load(body_isdofancestor[b1_3], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacr1_tile = wp.tile_map(wp.add, jacr1_tile, wp.tile_map(wp.mul, t_jacr, weights1[3]))

        # Weighted jacr for side 2
        jacr2_tile = wp.tile_map(
          support._compute_jacr,
          cdof_tile,
          wp.tile_load(body_isdofancestor[b2_0], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
        )
        jacr2_tile = wp.tile_map(wp.mul, jacr2_tile, weights2[0])
        if b2_1 >= 0:
          t_jacr = wp.tile_map(
            support._compute_jacr,
            cdof_tile,
            wp.tile_load(body_isdofancestor[b2_1], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacr2_tile = wp.tile_map(wp.add, jacr2_tile, wp.tile_map(wp.mul, t_jacr, weights2[1]))
        if b2_2 >= 0:
          t_jacr = wp.tile_map(
            support._compute_jacr,
            cdof_tile,
            wp.tile_load(body_isdofancestor[b2_2], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacr2_tile = wp.tile_map(wp.add, jacr2_tile, wp.tile_map(wp.mul, t_jacr, weights2[2]))
        if b2_3 >= 0:
          t_jacr = wp.tile_map(
            support._compute_jacr,
            cdof_tile,
            wp.tile_load(body_isdofancestor[b2_3], shape=TILE_SIZE, offset=dof_start, bounds_check=True),
          )
          jacr2_tile = wp.tile_map(wp.add, jacr2_tile, wp.tile_map(wp.mul, t_jacr, weights2[3]))

        jacr_dif_tile = wp.tile_map(wp.sub, jacr2_tile, jacr1_tile)

        if not wp.static(IS_ELLIPTIC):
          frame_0 = frame_in[conid, 0]
          Ji_0p_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_0)

          if condim > 1:
            Ji_0r_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_0)
            frame_1 = frame_in[conid, 1]
            Ji_1p_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_1)
            Ji_1r_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_1)
            frame_2 = frame_in[conid, 2]
            Ji_2p_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_2)
            Ji_2r_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_2)

      if wp.static(IS_ELLIPTIC):
        dimid = efcid - contact_efc_address_in[conid, 0]
        if dimid < 3:
          frame_idx = dimid
        else:
          frame_idx = dimid - 3

        frame_row = frame_in[conid, frame_idx]

        if dimid < 3:
          J_tile = wp.tile_map(wp.dot, jacp_dif_tile, frame_row)
        else:
          J_tile = wp.tile_map(wp.dot, jacr_dif_tile, frame_row)
      else:
        J_tile = Ji_0p_tile
        if condim > 1:
          dimid = efcid - contact_efc_address_in[conid, 0]
          dimid2 = dimid / 2 + 1
          frii = friction_in[conid, dimid2 - 1]
          frii_sign = frii * (1.0 - 2.0 * float(dimid & 1))

          if dimid2 == 1:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_1p_tile, frii_sign))
          elif dimid2 == 2:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_2p_tile, frii_sign))
          elif dimid2 == 3:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_0r_tile, frii_sign))
          elif dimid2 == 4:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_1r_tile, frii_sign))
          else:
            J_tile = wp.tile_map(wp.add, J_tile, wp.tile_map(wp.mul, Ji_2r_tile, frii_sign))

      wp.tile_store(efc_J_out[worldid, efcid], J_tile, offset=dof_start, bounds_check=True)

      Jqvel_tile = wp.tile_map(wp.mul, J_tile, qvel_tile)
      Jqvel_sum = wp.tile_reduce(wp.add, Jqvel_tile)
      if tid == 0:
        wp.atomic_add(efc_Jqvel_out[worldid], efcid, wp.tile_extract(Jqvel_sum, 0))

  return kernel


@cache_kernel
def _efc_contact_update(cone_type: types.ConeType):
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    opt_impratio_invsqrt: wp.array[float],
    body_invweight0: wp.array2d[wp.vec2],
    geom_bodyid: wp.array[int],
    # Data in:
    contact_efc_address_in: wp.array2d[int],
    efc_Jqvel_in: wp.array2d[float],
    nacon_in: wp.array[int],
    # In:
    dist_in: wp.array[float],
    condim_in: wp.array[int],
    includemargin_in: wp.array[float],
    worldid_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    friction_in: wp.array[vec5],
    solref_in: wp.array[wp.vec2],
    solreffriction_in: wp.array[wp.vec2],
    solimp_in: wp.array[vec5],
    type_in: wp.array[int],
    # Data out:
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
  ):
    conid, dimid = wp.tid()

    if conid >= nacon_in[0]:
      return

    if not type_in[conid] & ContactType.CONSTRAINT:
      return

    condim = condim_in[conid]

    if wp.static(IS_ELLIPTIC):
      if dimid > condim - 1:
        return
    else:
      if condim == 1 and dimid > 0:
        return
      elif condim > 1 and dimid >= 2 * (condim - 1):
        return

    efcid = contact_efc_address_in[conid, dimid]
    if efcid < 0:
      return

    worldid = worldid_in[conid]
    timestep = opt_timestep[worldid % opt_timestep.shape[0]]
    impratio_invsqrt = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

    includemargin = includemargin_in[conid]
    pos = dist_in[conid] - includemargin

    geom = geom_in[conid]
    Jqvel = efc_Jqvel_in[worldid, efcid]

    body1 = geom_bodyid[geom[0]]
    body2 = geom_bodyid[geom[1]]

    body_invweight0_id = worldid % body_invweight0.shape[0]
    invweight = body_invweight0[body_invweight0_id, body1][0] + body_invweight0[body_invweight0_id, body2][0]

    ref = solref_in[conid]
    pos_aref = pos

    if wp.static(IS_ELLIPTIC):
      if dimid > 0:
        solreffriction = solreffriction_in[conid]

        # non-normal directions use solreffriction (if non-zero)
        if solreffriction[0] or solreffriction[1]:
          ref = solreffriction

        invweight = invweight * impratio_invsqrt * impratio_invsqrt
        friction = friction_in[conid]

        if dimid > 1:
          fri0 = friction[0]
          frii = friction[dimid - 1]
          fri = fri0 * fri0 / (frii * frii)
          invweight *= fri

        pos_aref = 0.0
    else:
      if condim > 1:
        friction = friction_in[conid]
        fri0 = friction[0]
        invweight = invweight + fri0 * fri0 * invweight
        invweight = invweight * 2.0 * fri0 * fri0 * impratio_invsqrt * impratio_invsqrt

    if condim == 1:
      efc_type = ConstraintType.CONTACT_FRICTIONLESS
    elif wp.static(IS_ELLIPTIC):
      efc_type = ConstraintType.CONTACT_ELLIPTIC
    else:
      efc_type = ConstraintType.CONTACT_PYRAMIDAL

    _efc_row(
      opt_disableflags,
      worldid,
      timestep,
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

  return kernel


@cache_kernel
def _efc_contact_update_flex(cone_type: types.ConeType):
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_timestep: wp.array[float],
    opt_disableflags: int,
    opt_impratio_invsqrt: wp.array[float],
    body_invweight0: wp.array2d[wp.vec2],
    geom_bodyid: wp.array[int],
    flex_dim: wp.array[int],
    flex_interp: wp.array[int],
    flex_cellnum: wp.array[wp.vec3i],
    flex_nodeadr: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_shelldataadr: wp.array[int],
    flex_nodebodyid: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_shell: wp.array[int],
    flex_vert0: wp.array[wp.vec3],
    # Data in:
    flexvert_xpos_in: wp.array2d[wp.vec3],
    contact_efc_address_in: wp.array2d[int],
    efc_Jqvel_in: wp.array2d[float],
    nacon_in: wp.array[int],
    # In:
    dist_in: wp.array[float],
    pos_in: wp.array[wp.vec3],
    condim_in: wp.array[int],
    includemargin_in: wp.array[float],
    worldid_in: wp.array[int],
    geom_in: wp.array[wp.vec2i],
    flex_in: wp.array[wp.vec2i],
    elem_in: wp.array[wp.vec2i],
    vert_in: wp.array[wp.vec2i],
    friction_in: wp.array[vec5],
    solref_in: wp.array[wp.vec2],
    solreffriction_in: wp.array[wp.vec2],
    solimp_in: wp.array[vec5],
    type_in: wp.array[int],
    # Data out:
    efc_type_out: wp.array2d[int],
    efc_id_out: wp.array2d[int],
    efc_pos_out: wp.array2d[float],
    efc_margin_out: wp.array2d[float],
    efc_D_out: wp.array2d[float],
    efc_vel_out: wp.array2d[float],
    efc_aref_out: wp.array2d[float],
    efc_frictionloss_out: wp.array2d[float],
  ):
    conid, dimid = wp.tid()

    if conid >= nacon_in[0]:
      return

    if not type_in[conid] & ContactType.CONSTRAINT:
      return

    condim = condim_in[conid]

    if wp.static(IS_ELLIPTIC):
      if dimid > condim - 1:
        return
    else:
      if condim == 1 and dimid > 0:
        return
      elif condim > 1 and dimid >= 2 * (condim - 1):
        return

    efcid = contact_efc_address_in[conid, dimid]
    if efcid < 0:
      return

    worldid = worldid_in[conid]
    timestep = opt_timestep[worldid % opt_timestep.shape[0]]
    impratio_invsqrt = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

    includemargin = includemargin_in[conid]
    pos = dist_in[conid] - includemargin

    geom = geom_in[conid]
    flex = flex_in[conid]
    elem = elem_in[conid]
    vert = vert_in[conid]
    con_pos = pos_in[conid]
    Jqvel = efc_Jqvel_in[worldid, efcid]

    body_invweight0_id = worldid % body_invweight0.shape[0]
    invweight1 = float(0.0)

    if geom[0] >= 0:
      b1 = geom_bodyid[geom[0]]
      invweight1 = body_invweight0[body_invweight0_id, b1][0]
    elif flex[0] >= 0:
      f1 = flex[0]
      if flex_interp[f1] != 0:
        if vert[0] >= 0:
          v_adr = flex_vertadr[f1] + vert[0]
          coord = flex_vert0[v_adr]
          cn = flex_cellnum[f1]
          cx = cn[0]
          cy = cn[1]
          cz = cn[2]

          ci = wp.min(int(coord[0] * float(cx)), cx - 1)
          ci = wp.max(ci, 0)
          cj = wp.min(int(coord[1] * float(cy)), cy - 1)
          cj = wp.max(cj, 0)
          ck = wp.min(int(coord[2] * float(cz)), cz - 1)
          ck = wp.max(ck, 0)

          local_x = wp.clamp(coord[0] * float(cx) - float(ci), 0.0, 1.0)
          local_y = wp.clamp(coord[1] * float(cy) - float(cj), 0.0, 1.0)
          local_z = wp.clamp(coord[2] * float(cz) - float(ck), 0.0, 1.0)
          local = wp.vec3(local_x, local_y, local_z)

          ny_g = cy + 1
          nz_g = cz + 1
          nstart = flex_nodeadr[f1]

          for li in range(2):
            for lj in range(2):
              for lk in range(2):
                w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                if w > 1.0e-5:
                  gi = ci + li
                  gj = cj + lj
                  gk = ck + lk
                  node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                  b = flex_nodebodyid[nstart + node_idx]
                  invweight1 += body_invweight0[body_invweight0_id, b][0] * w
        elif elem[0] >= 0:
          e = elem[0]
          dim_f = flex_dim[f1]
          edata_adr = flex_elemdataadr[f1] + e * (dim_f + 1)
          vert_adr_f = flex_vertadr[f1]

          total_inv_dist = float(0.0)
          blended_coord = wp.vec3(0.0, 0.0, 0.0)
          for vi in range(4):
            if vi <= dim_f:
              v_idx = flex_elem[edata_adr + vi]
              vpos = flexvert_xpos_in[worldid, vert_adr_f + v_idx]
              dist_v = wp.length(con_pos - vpos)
              w_inv = 1.0 / wp.max(1.0e-10, dist_v)
              total_inv_dist += w_inv
              blended_coord += flex_vert0[vert_adr_f + v_idx] * w_inv

          if total_inv_dist > 1.0e-10:
            blended_coord = blended_coord / total_inv_dist

          cn = flex_cellnum[f1]
          cx = cn[0]
          cy = cn[1]
          cz = cn[2]

          ci = wp.min(int(blended_coord[0] * float(cx)), cx - 1)
          ci = wp.max(ci, 0)
          cj = wp.min(int(blended_coord[1] * float(cy)), cy - 1)
          cj = wp.max(cj, 0)
          ck = wp.min(int(blended_coord[2] * float(cz)), cz - 1)
          ck = wp.max(ck, 0)

          local_x = wp.clamp(blended_coord[0] * float(cx) - float(ci), 0.0, 1.0)
          local_y = wp.clamp(blended_coord[1] * float(cy) - float(cj), 0.0, 1.0)
          local_z = wp.clamp(blended_coord[2] * float(cz) - float(ck), 0.0, 1.0)
          local = wp.vec3(local_x, local_y, local_z)

          ny_g = cy + 1
          nz_g = cz + 1
          nstart = flex_nodeadr[f1]

          for li in range(2):
            for lj in range(2):
              for lk in range(2):
                w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                if w > 1.0e-5:
                  gi = ci + li
                  gj = cj + lj
                  gk = ck + lk
                  node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                  b = flex_nodebodyid[nstart + node_idx]
                  invweight1 += body_invweight0[body_invweight0_id, b][0] * w
      else:
        body_ids, weights = _get_contact_bodies_and_weights(
          geom_bodyid,
          flex_dim,
          flex_cellnum,
          flex_nodeadr,
          flex_vertadr,
          flex_elemdataadr,
          flex_shelldataadr,
          flex_nodebodyid,
          flex_vertbodyid,
          flex_elem,
          flex_shell,
          flex_vert0,
          flexvert_xpos_in,
          conid,
          0,
          geom,
          flex,
          elem,
          vert,
          con_pos,
          worldid,
        )
        b0 = body_ids[0]
        b1 = body_ids[1]
        b2 = body_ids[2]
        b3 = body_ids[3]
        invweight1 = weights[0] * body_invweight0[body_invweight0_id, b0][0]
        if b1 >= 0:
          invweight1 += weights[1] * body_invweight0[body_invweight0_id, b1][0]
        if b2 >= 0:
          invweight1 += weights[2] * body_invweight0[body_invweight0_id, b2][0]
        if b3 >= 0:
          invweight1 += weights[3] * body_invweight0[body_invweight0_id, b3][0]

    invweight2 = float(0.0)

    if geom[1] >= 0:
      b2 = geom_bodyid[geom[1]]
      invweight2 = body_invweight0[body_invweight0_id, b2][0]
    elif flex[1] >= 0:
      f2 = flex[1]
      if flex_interp[f2] != 0:
        if vert[1] >= 0:
          v_adr = flex_vertadr[f2] + vert[1]
          coord = flex_vert0[v_adr]
          cn = flex_cellnum[f2]
          cx = cn[0]
          cy = cn[1]
          cz = cn[2]

          ci = wp.min(int(coord[0] * float(cx)), cx - 1)
          ci = wp.max(ci, 0)
          cj = wp.min(int(coord[1] * float(cy)), cy - 1)
          cj = wp.max(cj, 0)
          ck = wp.min(int(coord[2] * float(cz)), cz - 1)
          ck = wp.max(ck, 0)

          local_x = wp.clamp(coord[0] * float(cx) - float(ci), 0.0, 1.0)
          local_y = wp.clamp(coord[1] * float(cy) - float(cj), 0.0, 1.0)
          local_z = wp.clamp(coord[2] * float(cz) - float(ck), 0.0, 1.0)
          local = wp.vec3(local_x, local_y, local_z)

          ny_g = cy + 1
          nz_g = cz + 1
          nstart = flex_nodeadr[f2]

          for li in range(2):
            for lj in range(2):
              for lk in range(2):
                w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                if w > 1.0e-5:
                  gi = ci + li
                  gj = cj + lj
                  gk = ck + lk
                  node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                  b = flex_nodebodyid[nstart + node_idx]
                  invweight2 += body_invweight0[body_invweight0_id, b][0] * w
        elif elem[1] >= 0:
          e = elem[1]
          dim_f = flex_dim[f2]
          edata_adr = flex_elemdataadr[f2] + e * (dim_f + 1)
          vert_adr_f = flex_vertadr[f2]

          total_inv_dist = float(0.0)
          blended_coord = wp.vec3(0.0, 0.0, 0.0)
          for vi in range(4):
            if vi <= dim_f:
              v_idx = flex_elem[edata_adr + vi]
              vpos = flexvert_xpos_in[worldid, vert_adr_f + v_idx]
              dist_v = wp.length(con_pos - vpos)
              w_inv = 1.0 / wp.max(1.0e-10, dist_v)
              total_inv_dist += w_inv
              blended_coord += flex_vert0[vert_adr_f + v_idx] * w_inv

          if total_inv_dist > 1.0e-10:
            blended_coord = blended_coord / total_inv_dist

          cn = flex_cellnum[f2]
          cx = cn[0]
          cy = cn[1]
          cz = cn[2]

          ci = wp.min(int(blended_coord[0] * float(cx)), cx - 1)
          ci = wp.max(ci, 0)
          cj = wp.min(int(blended_coord[1] * float(cy)), cy - 1)
          cj = wp.max(cj, 0)
          ck = wp.min(int(blended_coord[2] * float(cz)), cz - 1)
          ck = wp.max(ck, 0)

          local_x = wp.clamp(blended_coord[0] * float(cx) - float(ci), 0.0, 1.0)
          local_y = wp.clamp(blended_coord[1] * float(cy) - float(cj), 0.0, 1.0)
          local_z = wp.clamp(blended_coord[2] * float(cz) - float(ck), 0.0, 1.0)
          local = wp.vec3(local_x, local_y, local_z)

          ny_g = cy + 1
          nz_g = cz + 1
          nstart = flex_nodeadr[f2]

          for li in range(2):
            for lj in range(2):
              for lk in range(2):
                w = support.eval_basis_trilinear(local, li * 4 + lj * 2 + lk)
                if w > 1.0e-5:
                  gi = ci + li
                  gj = cj + lj
                  gk = ck + lk
                  node_idx = gi * ny_g * nz_g + gj * nz_g + gk
                  b = flex_nodebodyid[nstart + node_idx]
                  invweight2 += body_invweight0[body_invweight0_id, b][0] * w
      else:
        body_ids, weights = _get_contact_bodies_and_weights(
          geom_bodyid,
          flex_dim,
          flex_cellnum,
          flex_nodeadr,
          flex_vertadr,
          flex_elemdataadr,
          flex_shelldataadr,
          flex_nodebodyid,
          flex_vertbodyid,
          flex_elem,
          flex_shell,
          flex_vert0,
          flexvert_xpos_in,
          conid,
          1,
          geom,
          flex,
          elem,
          vert,
          con_pos,
          worldid,
        )
        b0 = body_ids[0]
        b1 = body_ids[1]
        b2 = body_ids[2]
        b3 = body_ids[3]
        invweight2 = weights[0] * body_invweight0[body_invweight0_id, b0][0]
        if b1 >= 0:
          invweight2 += weights[1] * body_invweight0[body_invweight0_id, b1][0]
        if b2 >= 0:
          invweight2 += weights[2] * body_invweight0[body_invweight0_id, b2][0]
        if b3 >= 0:
          invweight2 += weights[3] * body_invweight0[body_invweight0_id, b3][0]

    invweight = invweight1 + invweight2

    ref = solref_in[conid]
    pos_aref = pos

    if wp.static(IS_ELLIPTIC):
      if dimid > 0:
        solreffriction = solreffriction_in[conid]

        # non-normal directions use solreffriction (if non-zero)
        if solreffriction[0] or solreffriction[1]:
          ref = solreffriction

        invweight = invweight * impratio_invsqrt * impratio_invsqrt
        friction = friction_in[conid]

        if dimid > 1:
          fri0 = friction[0]
          frii = friction[dimid - 1]
          fri = fri0 * fri0 / (frii * frii)
          invweight *= fri

        pos_aref = 0.0
    else:
      if condim > 1:
        friction = friction_in[conid]
        fri0 = friction[0]
        invweight = invweight + fri0 * fri0 * invweight
        invweight = invweight * 2.0 * fri0 * fri0 * impratio_invsqrt * impratio_invsqrt

    if condim == 1:
      efc_type = ConstraintType.CONTACT_FRICTIONLESS
    elif wp.static(IS_ELLIPTIC):
      efc_type = ConstraintType.CONTACT_ELLIPTIC
    else:
      efc_type = ConstraintType.CONTACT_PYRAMIDAL

    _efc_row(
      opt_disableflags,
      worldid,
      timestep,
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

  return kernel


@event_scope
def make_constraint(m: types.Model, d: types.Data):
  """Creates constraint jacobians and other supporting data."""
  newton = m.opt.solver == types.SolverType.NEWTON
  efc_nnz = wp.empty((d.nworld,), dtype=int)

  wp.launch(
    _zero_constraint_counts,
    dim=d.nworld,
    inputs=[d.ne, d.nf, d.nl, d.nefc, d.efc.jtdaj_nblock, efc_nnz],
  )

  if not (m.opt.disableflags & types.DisableBit.CONSTRAINT):
    if not (m.opt.disableflags & types.DisableBit.EQUALITY):
      wp.launch(
        _equality_connect(m.is_sparse, newton),
        dim=(d.nworld, m.eq_connect_adr.size),
        inputs=[
          m.nv,
          m.nsite,
          m.opt.timestep,
          m.opt.disableflags,
          m.body_parentid,
          m.body_rootid,
          m.body_weldid,
          m.body_dofnum,
          m.body_dofadr,
          m.body_invweight0,
          m.jnt_type,
          m.jnt_dofadr,
          m.dof_bodyid,
          m.dof_jntid,
          m.dof_parentid,
          m.site_bodyid,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_objtype,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.body_isdofancestor,
          m.eq_connect_adr,
          d.qvel,
          d.eq_active,
          d.xpos,
          d.xmat,
          d.site_xpos,
          d.subtree_com,
          d.cdof,
          d.cvel,
          d.cdof_dot,
          d.subtree_linvel,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.ne,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )
      wp.launch(
        _equality_weld(m.is_sparse, newton),
        dim=(d.nworld, m.eq_wld_adr.size),
        inputs=[
          m.nv,
          m.nsite,
          m.opt.timestep,
          m.opt.disableflags,
          m.body_parentid,
          m.body_rootid,
          m.body_weldid,
          m.body_dofnum,
          m.body_dofadr,
          m.body_invweight0,
          m.jnt_type,
          m.jnt_dofadr,
          m.dof_bodyid,
          m.dof_jntid,
          m.dof_parentid,
          m.site_bodyid,
          m.site_quat,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_objtype,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.body_isdofancestor,
          m.eq_wld_adr,
          d.qvel,
          d.eq_active,
          d.xpos,
          d.xquat,
          d.xmat,
          d.site_xpos,
          d.subtree_com,
          d.cdof,
          d.cvel,
          d.cdof_dot,
          d.subtree_linvel,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.ne,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )
      wp.launch(
        _equality_joint(m.is_sparse, newton),
        dim=(d.nworld, m.eq_jnt_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
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
          d.qpos,
          d.qvel,
          d.eq_active,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.ne,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )
      wp.launch(
        _equality_tendon(m.is_sparse, newton),
        dim=(d.nworld, m.eq_ten_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_solref,
          m.eq_solimp,
          m.eq_data,
          m.ten_J_rownnz,
          m.ten_J_rowadr,
          m.ten_J_colind,
          m.tendon_length0,
          m.tendon_invweight0,
          m.eq_ten_adr,
          d.qvel,
          d.eq_active,
          d.ten_J,
          d.ten_length,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.ne,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )

      if m.nflex > 0:
        wp.launch(
          _equality_flex(m.is_sparse, newton),
          dim=(d.nworld, m.eq_flex_adr.size, m.nflexedge),
          inputs=[
            m.nv,
            m.opt.timestep,
            m.opt.disableflags,
            m.flex_interp,
            m.flex_edgeadr,
            m.flex_edgenum,
            m.flexedge_length0,
            m.flexedge_invweight0,
            m.flexedge_J_rownnz,
            m.flexedge_J_rowadr,
            m.flexedge_J_colind,
            m.eq_obj1id,
            m.eq_solref,
            m.eq_solimp,
            m.eq_flex_adr,
            d.qvel,
            d.eq_active,
            d.flexedge_J,
            d.flexedge_length,
            d.njmax,
            d.njmax_nnz,
          ],
          outputs=[
            d.ne,
            d.nefc,
            d.efc.type,
            d.efc.id,
            d.efc.jtdaj_adr,
            d.efc.jtdaj_nrow,
            d.efc.jtdaj_nblock,
            d.efc.J_rownnz,
            d.efc.J_rowadr,
            d.efc.J_colind,
            d.efc.J,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
            efc_nnz,
          ],
        )

        if m.eq_flexstrain_adr.size:
          wp.launch(
            _equality_flexstrain(m.is_sparse, newton),
            dim=(d.nworld, m.eq_flexstrain_adr.size),
            inputs=[
              m.nv,
              m.opt.timestep,
              m.opt.disableflags,
              m.body_parentid,
              m.body_rootid,
              m.body_dofnum,
              m.body_dofadr,
              m.body_invweight0,
              m.dof_bodyid,
              m.flex_interp,
              m.flex_cellnum,
              m.flex_nodeadr,
              m.flex_nodenum,
              m.flex_stiffnessadr,
              m.flex_nodebodyid,
              m.flex_node,
              m.flex_node0,
              m.flex_stiffness,
              m.flex_centered,
              m.eq_obj1id,
              m.eq_solref,
              m.eq_solimp,
              m.eq_data,
              m.body_isdofancestor,
              m.eq_flexstrain_adr,
              m.flexstrain_J_rownnz,
              m.flexstrain_J_rowadr,
              m.flexstrain_J_colind,
              d.qvel,
              d.eq_active,
              d.xpos,
              d.xmat,
              d.subtree_com,
              d.cdof,
              d.njmax,
              d.njmax_nnz,
              d.flexnode_xpos,
            ],
            outputs=[
              d.ne,
              d.nefc,
              d.efc.type,
              d.efc.id,
              d.efc.jtdaj_adr,
              d.efc.jtdaj_nrow,
              d.efc.jtdaj_nblock,
              d.efc.J_rownnz,
              d.efc.J_rowadr,
              d.efc.J_colind,
              d.efc.J,
              d.efc.pos,
              d.efc.margin,
              d.efc.D,
              d.efc.vel,
              d.efc.aref,
              d.efc.frictionloss,
              efc_nnz,
            ],
          )

    if not (m.opt.disableflags & types.DisableBit.FRICTIONLOSS):
      wp.launch(
        _friction_dof(m.is_sparse, newton),
        dim=(d.nworld, m.nv),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
          m.dof_solref,
          m.dof_solimp,
          m.dof_frictionloss,
          m.dof_invweight0,
          d.qvel,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.nf,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )

      wp.launch(
        _friction_tendon(m.is_sparse, newton),
        dim=(d.nworld, m.ntendon),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
          m.ten_J_rownnz,
          m.ten_J_rowadr,
          m.ten_J_colind,
          m.tendon_solref_fri,
          m.tendon_solimp_fri,
          m.tendon_frictionloss,
          m.tendon_invweight0,
          d.qvel,
          d.ten_J,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.nf,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )

    # limit
    if not (m.opt.disableflags & types.DisableBit.LIMIT):
      wp.launch(
        _limit_ball(m.is_sparse, newton),
        dim=(d.nworld, m.jnt_limited_ball_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
          m.jnt_qposadr,
          m.jnt_dofadr,
          m.jnt_solref,
          m.jnt_solimp,
          m.jnt_range,
          m.jnt_margin,
          m.dof_invweight0,
          m.jnt_limited_ball_adr,
          d.qpos,
          d.qvel,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.nl,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )

      wp.launch(
        _limit_slide_hinge(m.is_sparse, newton),
        dim=(d.nworld, m.jnt_limited_slide_hinge_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
          m.jnt_qposadr,
          m.jnt_dofadr,
          m.jnt_solref,
          m.jnt_solimp,
          m.jnt_range,
          m.jnt_margin,
          m.dof_invweight0,
          m.jnt_limited_slide_hinge_adr,
          d.qpos,
          d.qvel,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.nl,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )

      wp.launch(
        _limit_tendon(m.is_sparse, newton),
        dim=(d.nworld, m.tendon_limited_adr.size),
        inputs=[
          m.nv,
          m.opt.timestep,
          m.opt.disableflags,
          m.ten_J_rownnz,
          m.ten_J_rowadr,
          m.ten_J_colind,
          m.tendon_solref_lim,
          m.tendon_solimp_lim,
          m.tendon_range,
          m.tendon_margin,
          m.tendon_invweight0,
          m.tendon_limited_adr,
          d.qvel,
          d.ten_J,
          d.ten_length,
          d.njmax,
          d.njmax_nnz,
        ],
        outputs=[
          d.nl,
          d.nefc,
          d.efc.type,
          d.efc.id,
          d.efc.jtdaj_adr,
          d.efc.jtdaj_nrow,
          d.efc.jtdaj_nblock,
          d.efc.J_rownnz,
          d.efc.J_rowadr,
          d.efc.J_colind,
          d.efc.J,
          d.efc.pos,
          d.efc.margin,
          d.efc.D,
          d.efc.vel,
          d.efc.aref,
          d.efc.frictionloss,
          efc_nnz,
        ],
      )

    # contact
    if not (m.opt.disableflags & types.DisableBit.CONTACT):
      nmaxdim = int(m.nmaxpyramid) if m.opt.cone == types.ConeType.PYRAMIDAL else int(m.nmaxcondim)

      # Reinterpret to avoid unnecessary loads
      contact_frame_2d = wp.array(
        ptr=d.contact.frame.ptr,
        dtype=wp.vec3,
        shape=(d.naconmax, 3),
        device=d.contact.frame.device,
        copy=False,
      )
      contact_friction_2d = wp.array(
        ptr=d.contact.friction.ptr,
        dtype=float,
        shape=(d.naconmax, 5),
        device=d.contact.friction.device,
        copy=False,
      )

      has_flex = m.nflex > 0
      if has_flex:
        wp.launch(
          _efc_contact_init_flex(m.opt.cone, m.is_sparse, newton),
          dim=d.naconmax,
          inputs=[
            m.body_parentid,
            m.body_weldid,
            m.body_dofnum,
            m.body_dofadr,
            m.dof_parentid,
            m.geom_bodyid,
            m.flex_dim,
            m.flex_interp,
            m.flex_cellnum,
            m.flex_nodeadr,
            m.flex_vertadr,
            m.flex_elemdataadr,
            m.flex_shelldataadr,
            m.flex_nodebodyid,
            m.flex_vertbodyid,
            m.flex_elem,
            m.flex_shell,
            m.flex_vert0,
            d.flexvert_xpos,
            d.njmax,
            d.njmax_nnz,
            d.nacon,
            d.contact.dist,
            d.contact.dim,
            d.contact.includemargin,
            d.contact.worldid,
            d.contact.geom,
            d.contact.flex,
            d.contact.elem,
            d.contact.vert,
            d.contact.pos,
            d.contact.type,
          ],
          outputs=[
            d.nefc,
            d.contact.efc_address,
            d.efc.id,
            d.efc.jtdaj_adr,
            d.efc.jtdaj_nrow,
            d.efc.jtdaj_nblock,
            d.efc.J_rownnz,
            d.efc.J_rowadr,
            efc_nnz,
          ],
        )
      else:
        wp.launch(
          _efc_contact_init(m.opt.cone, m.is_sparse, newton),
          dim=d.naconmax,
          inputs=[
            m.body_weldid,
            m.body_dofnum,
            m.body_dofadr,
            m.dof_parentid,
            m.geom_bodyid,
            d.njmax,
            d.njmax_nnz,
            d.nacon,
            d.contact.dist,
            d.contact.dim,
            d.contact.includemargin,
            d.contact.worldid,
            d.contact.geom,
            d.contact.type,
          ],
          outputs=[
            d.nefc,
            d.contact.efc_address,
            d.efc.id,
            d.efc.jtdaj_adr,
            d.efc.jtdaj_nrow,
            d.efc.jtdaj_nblock,
            d.efc.J_rownnz,
            d.efc.J_rowadr,
            efc_nnz,
          ],
        )

      if m.is_sparse:
        if has_flex:
          wp.launch(
            _efc_contact_jac_sparse_flex(m.opt.cone),
            dim=(d.naconmax, nmaxdim),
            inputs=[
              m.body_parentid,
              m.body_rootid,
              m.body_weldid,
              m.body_dofnum,
              m.body_dofadr,
              m.dof_bodyid,
              m.dof_parentid,
              m.geom_bodyid,
              m.flex_dim,
              m.flex_interp,
              m.flex_cellnum,
              m.flex_nodeadr,
              m.flex_vertadr,
              m.flex_elemdataadr,
              m.flex_shelldataadr,
              m.flex_nodebodyid,
              m.flex_vertbodyid,
              m.flex_elem,
              m.flex_shell,
              m.flex_vert0,
              m.body_isdofancestor,
              d.qvel,
              d.subtree_com,
              d.cdof,
              d.flexvert_xpos,
              d.contact.efc_address,
              d.efc.J_rownnz,
              d.efc.J_rowadr,
              d.nacon,
              d.contact.dim,
              d.contact.geom,
              d.contact.flex,
              d.contact.elem,
              d.contact.vert,
              d.contact.pos,
              contact_frame_2d,
              contact_friction_2d,
              d.contact.worldid,
            ],
            outputs=[
              d.efc.J_colind,
              d.efc.J,
              d.efc.Jqvel,
            ],
          )
        else:
          wp.launch(
            _efc_contact_jac_sparse(m.opt.cone),
            dim=(d.naconmax, nmaxdim),
            inputs=[
              m.body_parentid,
              m.body_rootid,
              m.body_weldid,
              m.body_dofnum,
              m.body_dofadr,
              m.dof_bodyid,
              m.dof_parentid,
              m.geom_bodyid,
              m.body_isdofancestor,
              d.qvel,
              d.subtree_com,
              d.cdof,
              d.contact.efc_address,
              d.efc.J_rownnz,
              d.efc.J_rowadr,
              d.nacon,
              d.contact.dim,
              d.contact.geom,
              d.contact.pos,
              contact_frame_2d,
              contact_friction_2d,
              d.contact.worldid,
            ],
            outputs=[
              d.efc.J_colind,
              d.efc.J,
              d.efc.Jqvel,
            ],
          )
      else:
        d.efc.Jqvel.zero_()
        tile_size = m.block_dim.contact_jac_tiled
        n_dof_blocks = (m.nv_pad + tile_size - 1) // tile_size

        if has_flex:
          wp.launch_tiled(
            _efc_contact_jac_dense_flex(tile_size, m.opt.cone),
            dim=(d.nworld, n_dof_blocks),
            inputs=[
              m.body_rootid,
              m.geom_bodyid,
              m.flex_dim,
              m.flex_cellnum,
              m.flex_nodeadr,
              m.flex_vertadr,
              m.flex_elemdataadr,
              m.flex_shelldataadr,
              m.flex_nodebodyid,
              m.flex_vertbodyid,
              m.flex_elem,
              m.flex_shell,
              m.flex_vert0,
              m.body_isdofancestor,
              d.ne,
              d.nf,
              d.nl,
              d.nefc,
              d.qvel,
              d.subtree_com,
              d.cdof,
              d.flexvert_xpos,
              d.contact.efc_address,
              d.efc.id,
              d.njmax,
              m.nv_pad,
              d.contact.dim,
              d.contact.geom,
              d.contact.flex,
              d.contact.elem,
              d.contact.vert,
              d.contact.pos,
              contact_frame_2d,
              contact_friction_2d,
            ],
            outputs=[
              d.efc.J,
              d.efc.Jqvel,
            ],
            block_dim=tile_size,
          )
        else:
          wp.launch_tiled(
            _efc_contact_jac_dense(tile_size, m.opt.cone),
            dim=(d.nworld, n_dof_blocks),
            inputs=[
              m.body_rootid,
              m.geom_bodyid,
              m.body_isdofancestor,
              d.ne,
              d.nf,
              d.nl,
              d.nefc,
              d.qvel,
              d.subtree_com,
              d.cdof,
              d.contact.efc_address,
              d.efc.id,
              d.njmax,
              m.nv_pad,
              d.contact.dim,
              d.contact.geom,
              d.contact.pos,
              contact_frame_2d,
              contact_friction_2d,
            ],
            outputs=[
              d.efc.J,
              d.efc.Jqvel,
            ],
            block_dim=tile_size,
          )

      if has_flex:
        wp.launch(
          _efc_contact_update_flex(m.opt.cone),
          dim=(d.naconmax, nmaxdim),
          inputs=[
            m.opt.timestep,
            m.opt.disableflags,
            m.opt.impratio_invsqrt,
            m.body_invweight0,
            m.geom_bodyid,
            m.flex_dim,
            m.flex_interp,
            m.flex_cellnum,
            m.flex_nodeadr,
            m.flex_vertadr,
            m.flex_elemdataadr,
            m.flex_shelldataadr,
            m.flex_nodebodyid,
            m.flex_vertbodyid,
            m.flex_elem,
            m.flex_shell,
            m.flex_vert0,
            d.flexvert_xpos,
            d.contact.efc_address,
            d.efc.Jqvel,
            d.nacon,
            d.contact.dist,
            d.contact.pos,
            d.contact.dim,
            d.contact.includemargin,
            d.contact.worldid,
            d.contact.geom,
            d.contact.flex,
            d.contact.elem,
            d.contact.vert,
            d.contact.friction,
            d.contact.solref,
            d.contact.solreffriction,
            d.contact.solimp,
            d.contact.type,
          ],
          outputs=[
            d.efc.type,
            d.efc.id,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )
      else:
        wp.launch(
          _efc_contact_update(m.opt.cone),
          dim=(d.naconmax, nmaxdim),
          inputs=[
            m.opt.timestep,
            m.opt.disableflags,
            m.opt.impratio_invsqrt,
            m.body_invweight0,
            m.geom_bodyid,
            d.contact.efc_address,
            d.efc.Jqvel,
            d.nacon,
            d.contact.dist,
            d.contact.dim,
            d.contact.includemargin,
            d.contact.worldid,
            d.contact.geom,
            d.contact.friction,
            d.contact.solref,
            d.contact.solreffriction,
            d.contact.solimp,
            d.contact.type,
          ],
          outputs=[
            d.efc.type,
            d.efc.id,
            d.efc.pos,
            d.efc.margin,
            d.efc.D,
            d.efc.vel,
            d.efc.aref,
            d.efc.frictionloss,
          ],
        )
