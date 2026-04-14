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

from mujoco.mjx.third_party.mujoco_warp._src.support import next_act
from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import DynType
from mujoco.mjx.third_party.mujoco_warp._src.types import GainType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10f
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _qderiv_actuator_passive_vel(
  # Model:
  opt_timestep: wp.array(dtype=float),
  actuator_dyntype: wp.array(dtype=int),
  actuator_gaintype: wp.array(dtype=int),
  actuator_biastype: wp.array(dtype=int),
  actuator_actadr: wp.array(dtype=int),
  actuator_actnum: wp.array(dtype=int),
  actuator_forcelimited: wp.array(dtype=bool),
  actuator_actlimited: wp.array(dtype=bool),
  actuator_dynprm: wp.array2d(dtype=vec10f),
  actuator_gainprm: wp.array2d(dtype=vec10f),
  actuator_biasprm: wp.array2d(dtype=vec10f),
  actuator_actearly: wp.array(dtype=bool),
  actuator_forcerange: wp.array2d(dtype=wp.vec2),
  actuator_actrange: wp.array2d(dtype=wp.vec2),
  # Data in:
  act_in: wp.array2d(dtype=float),
  ctrl_in: wp.array2d(dtype=float),
  act_dot_in: wp.array2d(dtype=float),
  actuator_force_in: wp.array2d(dtype=float),
  # Out:
  vel_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()

  actuator_gainprm_id = worldid % actuator_gainprm.shape[0]
  actuator_biasprm_id = worldid % actuator_biasprm.shape[0]

  if actuator_gaintype[actid] == GainType.AFFINE:
    gain = actuator_gainprm[actuator_gainprm_id, actid][2]
  else:
    gain = 0.0

  if actuator_biastype[actid] == BiasType.AFFINE:
    bias = actuator_biasprm[actuator_biasprm_id, actid][2]
  else:
    bias = 0.0

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
  moment_rownnz_in: wp.array2d(dtype=int),
  moment_rowadr_in: wp.array2d(dtype=int),
  moment_colind_in: wp.array2d(dtype=int),
  actuator_moment_in: wp.array2d(dtype=float),
  # In:
  vel_in: wp.array2d(dtype=float),
  qMi: wp.array(dtype=int),
  qMj: wp.array(dtype=int),
  # Out:
  qDeriv_out: wp.array3d(dtype=float),
):
  worldid, elemid = wp.tid()

  dofiid = qMi[elemid]
  dofjid = qMj[elemid]
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
  M_rownnz: wp.array(dtype=int),
  M_rowadr: wp.array(dtype=int),
  # Data in:
  moment_rownnz_in: wp.array2d(dtype=int),
  moment_rowadr_in: wp.array2d(dtype=int),
  moment_colind_in: wp.array2d(dtype=int),
  actuator_moment_in: wp.array2d(dtype=float),
  # In:
  vel_in: wp.array2d(dtype=float),
  qMj: wp.array(dtype=int),
  # Out:
  qDeriv_out: wp.array3d(dtype=float),
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

      contrib = moment_i * moment_j * vel

      # Search the corresponding elemid
      # TODO: This could be precalculated for improved performance
      row = dofi
      col = dofj
      row_startk = M_rowadr[row] - 1
      row_nnz = M_rownnz[row]
      for k in range(row_nnz):
        row_startk += 1
        if qMj[row_startk] == col:
          wp.atomic_add(qDeriv_out[worldid, 0], row_startk, contrib)
          break


@wp.kernel
def _qderiv_actuator_passive(
  # Model:
  opt_timestep: wp.array(dtype=float),
  opt_disableflags: int,
  dof_damping: wp.array2d(dtype=float),
  is_sparse: bool,
  # Data in:
  qM_in: wp.array3d(dtype=float),
  # In:
  qMi: wp.array(dtype=int),
  qMj: wp.array(dtype=int),
  qDeriv_in: wp.array3d(dtype=float),
  # Out:
  qDeriv_out: wp.array3d(dtype=float),
):
  worldid, elemid = wp.tid()

  dofiid = qMi[elemid]
  dofjid = qMj[elemid]

  if is_sparse:
    qderiv = qDeriv_in[worldid, 0, elemid]
  else:
    qderiv = qDeriv_in[worldid, dofiid, dofjid]

  if not (opt_disableflags & DisableBit.DAMPER) and dofiid == dofjid:
    qderiv -= dof_damping[worldid % dof_damping.shape[0], dofiid]

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  if is_sparse:
    qDeriv_out[worldid, 0, elemid] = qM_in[worldid, 0, elemid] - qderiv
  else:
    qM = qM_in[worldid, dofiid, dofjid] - qderiv
    qDeriv_out[worldid, dofiid, dofjid] = qM
    if dofiid != dofjid:
      qDeriv_out[worldid, dofjid, dofiid] = qM


# TODO(team): improve performance with tile operations?
@wp.kernel
def _qderiv_tendon_damping(
  # Model:
  ntendon: int,
  opt_timestep: wp.array(dtype=float),
  ten_J_rownnz: wp.array(dtype=int),
  ten_J_rowadr: wp.array(dtype=int),
  ten_J_colind: wp.array(dtype=int),
  tendon_damping: wp.array2d(dtype=float),
  is_sparse: bool,
  # Data in:
  ten_J_in: wp.array2d(dtype=float),
  # In:
  qMi: wp.array(dtype=int),
  qMj: wp.array(dtype=int),
  # Out:
  qDeriv_out: wp.array3d(dtype=float),
):
  worldid, elemid = wp.tid()
  dofiid = qMi[elemid]
  dofjid = qMj[elemid]

  qderiv = float(0.0)
  tendon_damping_id = worldid % tendon_damping.shape[0]
  for tenid in range(ntendon):
    damping = tendon_damping[tendon_damping_id, tenid]
    if damping == 0.0:
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
    qderiv -= Ji * Jj * damping

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  if is_sparse:
    qDeriv_out[worldid, 0, elemid] -= qderiv
  else:
    qDeriv_out[worldid, dofiid, dofjid] -= qderiv
    if dofiid != dofjid:
      qDeriv_out[worldid, dofjid, dofiid] -= qderiv


@event_scope
def deriv_smooth_vel(m: Model, d: Data, out: wp.array2d(dtype=float)):
  """Analytical derivative of smooth forces w.r.t. velocities.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    out: qM - dt * qDeriv (derivatives of smooth forces w.r.t velocities).
  """
  qMi = m.qM_fullm_i
  qMj = m.qM_fullm_j

  # TODO(team): implicit requires different sparsity structure

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
          inputs=[m.M_rownnz, m.M_rowadr, d.moment_rownnz, d.moment_rowadr, d.moment_colind, d.actuator_moment, vel, qMj],
          outputs=[out],
        )
      else:
        wp.launch(
          _qderiv_actuator_passive_actuation_dense,
          dim=(d.nworld, qMi.size),
          inputs=[m.nu, d.moment_rownnz, d.moment_rowadr, d.moment_colind, d.actuator_moment, vel, qMi, qMj],
          outputs=[out],
        )
    wp.launch(
      _qderiv_actuator_passive,
      dim=(d.nworld, qMi.size),
      inputs=[
        m.opt.timestep,
        m.opt.disableflags,
        m.dof_damping,
        m.is_sparse,
        d.qM,
        qMi,
        qMj,
        out,
      ],
      outputs=[out],
    )
  else:
    # TODO(team): directly utilize qM for these settings
    wp.copy(out, d.qM)

  if not (m.opt.disableflags & DisableBit.DAMPER):
    wp.launch(
      _qderiv_tendon_damping,
      dim=(d.nworld, qMi.size),
      inputs=[
        m.ntendon,
        m.opt.timestep,
        m.ten_J_rownnz,
        m.ten_J_rowadr,
        m.ten_J_colind,
        m.tendon_damping,
        m.is_sparse,
        d.ten_J,
        qMi,
        qMj,
      ],
      outputs=[out],
    )

  # TODO(team): rne derivative
