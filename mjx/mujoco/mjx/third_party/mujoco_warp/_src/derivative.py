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
from mujoco.mjx.third_party.mujoco_warp._src import util_misc
from mujoco.mjx.third_party.mujoco_warp._src.support import next_act
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import DynType
from mujoco.mjx.third_party.mujoco_warp._src.types import GainType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10f
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _qderiv_actuator_passive_vel(
  # Model:
  opt_timestep: wp.array[float],
  actuator_dyntype: wp.array[int],
  actuator_gaintype: wp.array[int],
  actuator_biastype: wp.array[int],
  actuator_actadr: wp.array[int],
  actuator_actnum: wp.array[int],
  actuator_forcelimited: wp.array[bool],
  actuator_actlimited: wp.array[bool],
  actuator_dynprm: wp.array2d[vec10f],
  actuator_gainprm: wp.array2d[vec10f],
  actuator_biasprm: wp.array2d[vec10f],
  actuator_actearly: wp.array[bool],
  actuator_forcerange: wp.array2d[wp.vec2],
  actuator_actrange: wp.array2d[wp.vec2],
  # Data in:
  act_in: wp.array2d[float],
  ctrl_in: wp.array2d[float],
  act_dot_in: wp.array2d[float],
  actuator_force_in: wp.array2d[float],
  # Out:
  vel_out: wp.array2d[float],
):
  worldid, actid = wp.tid()

  actuator_gainprm_id = worldid % actuator_gainprm.shape[0]
  actuator_biasprm_id = worldid % actuator_biasprm.shape[0]

  bias = float(0.0)

  if actuator_gaintype[actid] == GainType.AFFINE:
    gain = actuator_gainprm[actuator_gainprm_id, actid][2]
  elif actuator_gaintype[actid] == GainType.DCMOTOR:
    gain = 0.0
    dynprm = actuator_dynprm[worldid % actuator_dynprm.shape[0], actid]
    gainprm = actuator_gainprm[actuator_gainprm_id, actid]
    te = dynprm[0]

    # controller velocity derivative: dV/dω
    input_mode = int(gainprm[8])
    dVdw = 0.0
    if input_mode == 1:
      dVdw = -gainprm[6]  # position: -kd
    elif input_mode == 2:
      dVdw = -gainprm[4]  # velocity: -kp

    if te > 0.0:
      # stateful current with actearly: d(K*next_act)/dω
      # includes both back-EMF (-K) and controller (dVdw) through act_dot
      R = wp.max(MJ_MINVAL, gainprm[0])
      K = gainprm[1]
      s = 1.0 - wp.exp(-opt_timestep[worldid % opt_timestep.shape[0]] / te)
      bias += K * (dVdw - K) * s / R
    elif dVdw != 0.0:
      # stateless: controller terms only (back-EMF handled in bias block)
      R = wp.max(MJ_MINVAL, gainprm[0])
      K = gainprm[1]
      bias += K * dVdw / R

    # LuGre: force includes -sigma1*z_dot, z_dot = a*z + v
    # d(sigma1*z_dot)/dv = sigma1*(da/dv*z + 1), ignoring higher-order da/dv*z
    sigma1 = dynprm[6]
    if sigma1 > 0.0:
      bias -= sigma1
  else:
    gain = 0.0

  if actuator_biastype[actid] == BiasType.AFFINE:
    bias += actuator_biasprm[actuator_biasprm_id, actid][2]
  elif actuator_biastype[actid] == BiasType.DCMOTOR:
    dynprm = actuator_dynprm[worldid % actuator_dynprm.shape[0], actid]
    te = dynprm[0]
    if te <= 0.0:
      gainprm = actuator_gainprm[actuator_gainprm_id, actid]
      R = gainprm[0]
      K = gainprm[1]

      slots = util_misc.dcmotor_slots(dynprm, gainprm)
      slot_Ta = slots[2]

      if slot_Ta >= 0:
        adr = actuator_actadr[actid] + slot_Ta
        T = act_in[worldid, adr]
        alpha = gainprm[2]
        T0 = gainprm[3]
        Ta = dynprm[4]
        R *= 1.0 + alpha * (T + Ta - T0)

      bias += -K * K / wp.max(MJ_MINVAL, R)

  if bias == 0.0 and gain == 0.0:
    vel_out[worldid, actid] = 0.0
    return

  # skip if force is clamped by forcerange
  if actuator_forcelimited[actid]:
    force = actuator_force_in[worldid, actid]
    forcerange = actuator_forcerange[worldid % actuator_forcerange.shape[0], actid]
    if force <= forcerange[0] or force >= forcerange[1]:
      vel_out[worldid, actid] = 0.0
      return

  vel = float(bias)
  if actuator_dyntype[actid] != DynType.NONE:
    if gain != 0.0:
      act_adr = actuator_actadr[actid] + actuator_actnum[actid] - 1

      # use next activation if actearly is set (matching forward pass)
      if actuator_actearly[actid]:
        act = next_act(
          opt_timestep[worldid % opt_timestep.shape[0]],
          actuator_dyntype[actid],
          actuator_dynprm[worldid % actuator_dynprm.shape[0], actid],
          actuator_actrange[worldid % actuator_actrange.shape[0], actid],
          act_in[worldid, act_adr],
          act_dot_in[worldid, act_adr],
          1.0,
          actuator_actlimited[actid],
        )
      else:
        act = act_in[worldid, act_adr]

      vel += gain * act
  else:
    if gain != 0.0:
      vel += gain * ctrl_in[worldid, actid]

  vel_out[worldid, actid] = vel


@wp.func
def _nonzero_mask(x: float) -> float:
  """Returns 1.0 for non-zero input, 0.0 otherwise."""
  if x != 0.0:
    return 1.0
  return 0.0


@wp.kernel
def _qderiv_actuator_passive_actuation_dense(
  # Model:
  nu: int,
  # Data in:
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  # In:
  vel_in: wp.array2d[float],
  Mi: wp.array[int],
  Mj: wp.array[int],
  # Out:
  qDeriv_out: wp.array3d[float],
):
  worldid, elemid = wp.tid()

  dofiid = Mi[elemid]
  dofjid = Mj[elemid]
  qderiv_contrib = float(0.0)
  for actid in range(nu):
    vel = vel_in[worldid, actid]
    if vel == 0.0:
      continue

    # TODO(team): restructure sparse version for better parallelism?
    moment_i = float(0.0)
    moment_j = float(0.0)

    rownnz = moment_rownnz_in[worldid, actid]
    rowadr = moment_rowadr_in[worldid, actid]
    for i in range(rownnz):
      sparseid = rowadr + i
      colind = moment_colind_in[worldid, sparseid]
      if colind == dofiid:
        moment_i = actuator_moment_in[worldid, sparseid]
      if colind == dofjid:
        moment_j = actuator_moment_in[worldid, sparseid]
      if moment_i != 0.0 and moment_j != 0.0:
        break

    if moment_i == 0 and moment_j == 0:
      continue

    qderiv_contrib += moment_i * moment_j * vel

  qDeriv_out[worldid, dofiid, dofjid] = qderiv_contrib
  if dofiid != dofjid:
    qDeriv_out[worldid, dofjid, dofiid] = qderiv_contrib


@wp.kernel
def _qderiv_actuator_passive_actuation_sparse(
  # Model:
  M_elemid: wp.array2d[int],
  # Data in:
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  # In:
  vel_in: wp.array2d[float],
  # Out:
  qDeriv_out: wp.array3d[float],
):
  worldid, actid = wp.tid()

  vel = vel_in[worldid, actid]
  if vel == 0.0:
    return

  rownnz = moment_rownnz_in[worldid, actid]
  rowadr = moment_rowadr_in[worldid, actid]

  for i in range(rownnz):
    rowadri = rowadr + i
    moment_i = actuator_moment_in[worldid, rowadri]
    if moment_i == 0.0:
      continue
    dofi = moment_colind_in[worldid, rowadri]

    for j in range(i + 1):
      rowadrj = rowadr + j
      moment_j = actuator_moment_in[worldid, rowadrj]
      if moment_j == 0.0:
        continue
      dofj = moment_colind_in[worldid, rowadrj]

      elemid = M_elemid[dofi, dofj]
      if elemid >= 0:
        contrib = moment_i * moment_j * vel
        wp.atomic_add(qDeriv_out[worldid, 0], elemid, contrib)


@wp.kernel
def _qderiv_actuator_passive(
  # Model:
  opt_timestep: wp.array[float],
  opt_disableflags: int,
  dof_damping: wp.array2d[float],
  dof_dampingpoly: wp.array2d[wp.vec2],
  is_sparse: bool,
  M_elemid: wp.array2d[int],
  # Data in:
  qvel_in: wp.array2d[float],
  M_in: wp.array3d[float],
  # In:
  Mi: wp.array[int],
  Mj: wp.array[int],
  qDeriv_in: wp.array3d[float],
  # Out:
  qDeriv_out: wp.array3d[float],
):
  worldid, elemid = wp.tid()

  dofiid = Mi[elemid]
  dofjid = Mj[elemid]

  madr = M_elemid[dofiid, dofjid]

  if is_sparse:
    if madr >= 0:
      qderiv = qDeriv_in[worldid, 0, madr]
    else:
      qderiv = 0.0
  else:
    qderiv = qDeriv_in[worldid, dofiid, dofjid]

  if not (opt_disableflags & DisableBit.DAMPER) and dofiid == dofjid:
    damping = dof_damping[worldid % dof_damping.shape[0], dofiid]
    dpoly = dof_dampingpoly[worldid % dof_dampingpoly.shape[0], dofiid]
    v = qvel_in[worldid, dofiid]
    qderiv -= util_misc._poly_force_deriv(damping, dpoly, v, 1)

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  if is_sparse:
    if madr >= 0:
      qDeriv_out[worldid, 0, madr] = M_in[worldid, 0, madr] - qderiv
  else:
    M = M_in[worldid, dofiid, dofjid] - qderiv
    qDeriv_out[worldid, dofiid, dofjid] = M
    if dofiid != dofjid:
      qDeriv_out[worldid, dofjid, dofiid] = M


# TODO(team): improve performance with tile operations?
@wp.kernel
def _qderiv_tendon_damping(
  # Model:
  ntendon: int,
  opt_timestep: wp.array[float],
  ten_J_rownnz: wp.array[int],
  ten_J_rowadr: wp.array[int],
  ten_J_colind: wp.array[int],
  tendon_damping: wp.array2d[float],
  tendon_dampingpoly: wp.array2d[wp.vec2],
  is_sparse: bool,
  M_elemid: wp.array2d[int],
  # Data in:
  ten_J_in: wp.array2d[float],
  ten_velocity_in: wp.array2d[float],
  # In:
  Mi: wp.array[int],
  Mj: wp.array[int],
  # Out:
  qDeriv_out: wp.array3d[float],
):
  worldid, elemid = wp.tid()
  dofiid = Mi[elemid]
  dofjid = Mj[elemid]

  qderiv = float(0.0)
  tendon_damping_id = worldid % tendon_damping.shape[0]
  for tenid in range(ntendon):
    damping = tendon_damping[tendon_damping_id, tenid]
    dpoly = tendon_dampingpoly[worldid % tendon_dampingpoly.shape[0], tenid]
    if damping == 0.0 and dpoly[0] == 0.0 and dpoly[1] == 0.0:
      continue

    rownnz = ten_J_rownnz[tenid]
    rowadr = ten_J_rowadr[tenid]
    Ji = float(0.0)
    Jj = float(0.0)
    for k in range(rownnz):
      if Ji != 0.0 and Jj != 0.0:
        break
      sparseid = rowadr + k
      colind = ten_J_colind[sparseid]
      if colind == dofiid:
        Ji = ten_J_in[worldid, sparseid]
      if colind == dofjid:
        Jj = ten_J_in[worldid, sparseid]

    v = ten_velocity_in[worldid, tenid]
    qderiv -= Ji * Jj * util_misc._poly_force_deriv(damping, dpoly, v, 1)

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  madr = M_elemid[dofiid, dofjid]

  if is_sparse:
    if madr >= 0:
      qDeriv_out[worldid, 0, madr] -= qderiv
  else:
    qDeriv_out[worldid, dofiid, dofjid] -= qderiv
    if dofiid != dofjid:
      qDeriv_out[worldid, dofjid, dofiid] -= qderiv


@wp.kernel
def deriv_rne_cvel_cdof_dot(
  # Model:
  body_parentid: wp.array[int],
  body_jntnum: wp.array[int],
  body_jntadr: wp.array[int],
  body_dofadr: wp.array[int],
  jnt_type: wp.array[int],
  # Data in:
  cdof_in: wp.array2d[wp.spatial_vector],
  # In:
  body_tree_: wp.array[int],
  # Out:
  Dcvel_out: wp.array3d[wp.spatial_vector],
  Dcdof_dot_out: wp.array3d[wp.spatial_vector],
):
  """Forward pass: compute d(cvel)/d(qvel_k) and d(cdof_dot)/d(qvel_k).

  Mirrors the accumulation order of comvel for each joint type.

  Dcdof_dot for rotation DOFs of free joints (dofid+0..2) is zero because the
  forward pass sets cdof_dot[dofid+0..2] = 0.  The Dcdof_dot array is
  zero-initialized so no explicit write is needed.
  """
  worldid, nodeid, dofid = wp.tid()
  bodyid = body_tree_[nodeid]
  dofadr = body_dofadr[bodyid]
  jntid = body_jntadr[bodyid]
  jntnum = body_jntnum[bodyid]
  pid = body_parentid[bodyid]

  cdof = cdof_in[worldid]

  # Initialize from parent
  cvel_k = Dcvel_out[worldid, pid, dofid]

  if jntnum == 0:
    Dcvel_out[worldid, bodyid, dofid] = cvel_k
    return

  dof_i = dofadr

  for j in range(jntid, jntid + jntnum):
    jnttype = jnt_type[j]

    if jnttype == 0:  # FREE
      # rotation DOFs (dof_i+0..2) contribute to cvel
      if dofid >= dof_i and dofid < dof_i + 3:
        cvel_k += cdof[dofid]

      # cdof_dot for rotation DOFs is zero (set in forward kinematics),
      # so Dcdof_dot for rotation DOFs is zero (from wp.zeros init)

      # derivative of cdof_dot for translation DOFs 3,4,5
      Dcdof_dot_out[worldid, dof_i + 3, dofid] = math.motion_cross(cvel_k, cdof[dof_i + 3])
      Dcdof_dot_out[worldid, dof_i + 4, dofid] = math.motion_cross(cvel_k, cdof[dof_i + 4])
      Dcdof_dot_out[worldid, dof_i + 5, dofid] = math.motion_cross(cvel_k, cdof[dof_i + 5])

      # translation DOFs (dof_i+3..5) contribute to cvel
      if dofid >= dof_i + 3 and dofid < dof_i + 6:
        cvel_k += cdof[dofid]

      dof_i += 6

    elif jnttype == 1:  # BALL
      Dcdof_dot_out[worldid, dof_i + 0, dofid] = math.motion_cross(cvel_k, cdof[dof_i + 0])
      Dcdof_dot_out[worldid, dof_i + 1, dofid] = math.motion_cross(cvel_k, cdof[dof_i + 1])
      Dcdof_dot_out[worldid, dof_i + 2, dofid] = math.motion_cross(cvel_k, cdof[dof_i + 2])

      if dofid >= dof_i and dofid < dof_i + 3:
        cvel_k += cdof[dofid]

      dof_i += 3
    else:  # HINGE or SLIDE
      Dcdof_dot_out[worldid, dof_i, dofid] = math.motion_cross(cvel_k, cdof[dof_i])

      if dofid == dof_i:
        cvel_k += cdof[dof_i]

      dof_i += 1

  Dcvel_out[worldid, bodyid, dofid] = cvel_k


@wp.kernel
def deriv_rne_cacc_cfrcbody_forward(
  # Model:
  body_parentid: wp.array[int],
  body_dofnum: wp.array[int],
  body_dofadr: wp.array[int],
  # Data in:
  qvel_in: wp.array2d[float],
  cinert_in: wp.array2d[vec10],
  cvel_in: wp.array2d[wp.spatial_vector],
  cdof_dot_in: wp.array2d[wp.spatial_vector],
  # In:
  body_tree_: wp.array[int],
  Dcvel_in: wp.array3d[wp.spatial_vector],
  Dcdof_dot_in: wp.array3d[wp.spatial_vector],
  # Out:
  Dcacc_out: wp.array3d[wp.spatial_vector],
  Dcfrcbody_out: wp.array3d[wp.spatial_vector],
):
  """Forward pass: compute d(cacc)/d(qvel_k) and d(cfrc_body)/d(qvel_k)."""
  worldid, nodeid, dofid = wp.tid()
  bodyid = body_tree_[nodeid]
  dofadr = body_dofadr[bodyid]
  dofnum = body_dofnum[bodyid]
  pid = body_parentid[bodyid]

  qvel = qvel_in[worldid]

  dcacc = Dcacc_out[worldid, pid, dofid]

  for j in range(dofadr, dofadr + dofnum):
    # Term 1: d(cdof_dot * qvel)/d(qvel_k) when j == dofid
    if j == dofid:
      dcacc += cdof_dot_in[worldid, j]

    # Term 2: cdof_dot depends on cvel which depends on qvel_k
    dcdofdot = Dcdof_dot_in[worldid, j, dofid]
    dcacc += dcdofdot * qvel[j]

  Dcacc_out[worldid, bodyid, dofid] = dcacc

  # d(cfrc_body)/d(qvel_k)
  cinert = cinert_in[worldid, bodyid]
  cvel = cvel_in[worldid, bodyid]
  dcvel = Dcvel_in[worldid, bodyid, dofid]

  # term1 = cinert * d(cacc)/d(qvel_k)
  term1 = math.inert_vec(cinert, dcacc)

  # term2 = d(cvel x* (cinert * cvel))/d(qvel_k)
  cinert_cvel = math.inert_vec(cinert, cvel)
  cinert_dcvel = math.inert_vec(cinert, dcvel)
  term2 = math.motion_cross_force(dcvel, cinert_cvel) + math.motion_cross_force(cvel, cinert_dcvel)

  Dcfrcbody_out[worldid, bodyid, dofid] = term1 + term2


@wp.kernel
def deriv_rne_cfrcbody_backward(
  # Model:
  body_parentid: wp.array[int],
  # In:
  body_tree_: wp.array[int],
  # Out:
  Dcfrcbody_out: wp.array3d[wp.spatial_vector],
):
  """Backward pass: accumulate d(cfrc_body) from children to parents."""
  worldid, nodeid, dofid = wp.tid()
  bodyid = body_tree_[nodeid]
  pid = body_parentid[bodyid]

  # body_tree never contains bodyid=0 (worldbody), so pid >= 0 is always valid.
  # Siblings at the same level may share a parent; atomic_add handles this.
  val = Dcfrcbody_out[worldid, bodyid, dofid]
  wp.atomic_add(Dcfrcbody_out[worldid, pid], dofid, val)


@wp.kernel
def deriv_rne_body2jnt_sparse(
  # Model:
  dof_bodyid: wp.array[int],
  # Data in:
  cdof_in: wp.array2d[wp.spatial_vector],
  # In:
  timestep: wp.array[float],
  Di: wp.array[int],
  Dj: wp.array[int],
  Dcfrcbody_in: wp.array3d[wp.spatial_vector],
  flg_subtract: bool,
  # Out:
  qDeriv_out: wp.array3d[float],
):
  """Project body-space RNE derivatives into joint-space qDeriv (sparse)."""
  worldid, elemid = wp.tid()
  dt = timestep[worldid % timestep.shape[0]]

  i = Di[elemid]
  j = Dj[elemid]

  body_i = dof_bodyid[i]
  dcfrc = Dcfrcbody_in[worldid, body_i, j]
  term = wp.dot(cdof_in[worldid, i], dcfrc)

  if flg_subtract:
    wp.atomic_sub(qDeriv_out[worldid, 0], elemid, dt * term)
  else:
    wp.atomic_add(qDeriv_out[worldid, 0], elemid, dt * term)


def deriv_rne_vel(m: Model, d: Data, out: wp.array3d[float], flg_subtract: bool = False):
  """Compute RNE velocity derivatives and add/subtract from the output.

  Implements the analytical derivative of inverse-dynamics Coriolis/centrifugal
  forces with respect to joint velocities.

  Args:
    m: The model (device).
    d: The data (device).
    out: D-structure output array (nworld, 1, nD) to accumulate RNE terms into.
    flg_subtract: If True, subtract the RNE derivatives from output instead of adding them.
  """
  # TODO(team): consider caching these allocations
  Dcvel = wp.zeros((d.nworld, m.nbody, m.nv), dtype=wp.spatial_vector)
  Dcdof_dot = wp.zeros((d.nworld, m.nv, m.nv), dtype=wp.spatial_vector)
  Dcacc = wp.zeros((d.nworld, m.nbody, m.nv), dtype=wp.spatial_vector)
  Dcfrcbody = wp.zeros((d.nworld, m.nbody, m.nv), dtype=wp.spatial_vector)

  # Forward pass 1: compute Dcvel and Dcdof_dot
  for body_tree in m.body_tree:
    wp.launch(
      deriv_rne_cvel_cdof_dot,
      dim=(d.nworld, body_tree.size, m.nv),
      inputs=[
        m.body_parentid,
        m.body_jntnum,
        m.body_jntadr,
        m.body_dofadr,
        m.jnt_type,
        d.cdof,
        body_tree,
      ],
      outputs=[Dcvel, Dcdof_dot],
    )

  # Forward pass 2: compute Dcacc and Dcfrcbody
  for body_tree in m.body_tree:
    wp.launch(
      deriv_rne_cacc_cfrcbody_forward,
      dim=(d.nworld, body_tree.size, m.nv),
      inputs=[
        m.body_parentid,
        m.body_dofnum,
        m.body_dofadr,
        d.qvel,
        d.cinert,
        d.cvel,
        d.cdof_dot,
        body_tree,
        Dcvel,
        Dcdof_dot,
      ],
      outputs=[Dcacc, Dcfrcbody],
    )

  # Backward pass: accumulate Dcfrcbody from children to parents
  for body_tree in reversed(m.body_tree):
    wp.launch(
      deriv_rne_cfrcbody_backward,
      dim=(d.nworld, body_tree.size, m.nv),
      inputs=[m.body_parentid, body_tree],
      outputs=[Dcfrcbody],
    )

  # Project body-space derivatives into joint-space qDeriv (always sparse D-structure)
  wp.launch(
    deriv_rne_body2jnt_sparse,
    dim=(d.nworld, m.qD_fullm_i.size),
    inputs=[m.dof_bodyid, d.cdof, m.opt.timestep, m.qD_fullm_i, m.qD_fullm_j, Dcfrcbody, flg_subtract],
    outputs=[out],
  )


@event_scope
def deriv_smooth_vel(m: Model, d: Data, out: wp.array2d[float]):
  """Analytical derivative of smooth forces w.r.t. velocities.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    out: M - dt * qDeriv (derivatives of smooth forces w.r.t velocities).
  """
  Mi = m.M_fullm_i
  Mj = m.M_fullm_j

  if ~(m.opt.disableflags & (DisableBit.ACTUATION | DisableBit.DAMPER)):
    # TODO(team): only clear elements not set by _qderiv_actuator_passive
    out.zero_()
    if m.nu > 0 and not (m.opt.disableflags & DisableBit.ACTUATION):
      vel = wp.empty((d.nworld, m.nu), dtype=float)
      wp.launch(
        _qderiv_actuator_passive_vel,
        dim=(d.nworld, m.nu),
        inputs=[
          m.opt.timestep,
          m.actuator_dyntype,
          m.actuator_gaintype,
          m.actuator_biastype,
          m.actuator_actadr,
          m.actuator_actnum,
          m.actuator_forcelimited,
          m.actuator_actlimited,
          m.actuator_dynprm,
          m.actuator_gainprm,
          m.actuator_biasprm,
          m.actuator_actearly,
          m.actuator_forcerange,
          m.actuator_actrange,
          d.act,
          d.ctrl,
          d.act_dot,
          d.actuator_force,
        ],
        outputs=[vel],
      )
      if m.is_sparse:
        wp.launch(
          _qderiv_actuator_passive_actuation_sparse,
          dim=(d.nworld, m.nu),
          inputs=[
            m.M_elemid,
            d.moment_rownnz,
            d.moment_rowadr,
            d.moment_colind,
            d.actuator_moment,
            vel,
          ],
          outputs=[out],
        )
      else:
        wp.launch(
          _qderiv_actuator_passive_actuation_dense,
          dim=(d.nworld, Mi.size),
          inputs=[m.nu, d.moment_rownnz, d.moment_rowadr, d.moment_colind, d.actuator_moment, vel, Mi, Mj],
          outputs=[out],
        )
    wp.launch(
      _qderiv_actuator_passive,
      dim=(d.nworld, Mi.size),
      inputs=[
        m.opt.timestep,
        m.opt.disableflags,
        m.dof_damping,
        m.dof_dampingpoly,
        m.is_sparse,
        m.M_elemid,
        d.qvel,
        d.M,
        Mi,
        Mj,
        out,
      ],
      outputs=[out],
    )
  else:
    # TODO(team): directly utilize M for these settings
    wp.copy(out, d.M)

  if not (m.opt.disableflags & DisableBit.DAMPER):
    wp.launch(
      _qderiv_tendon_damping,
      dim=(d.nworld, Mi.size),
      inputs=[
        m.ntendon,
        m.opt.timestep,
        m.ten_J_rownnz,
        m.ten_J_rowadr,
        m.ten_J_colind,
        m.tendon_damping,
        m.tendon_dampingpoly,
        m.is_sparse,
        m.M_elemid,
        d.ten_J,
        d.ten_velocity,
        Mi,
        Mj,
      ],
      outputs=[out],
    )
