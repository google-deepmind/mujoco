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

from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import DynType
from mujoco.mjx.third_party.mujoco_warp._src.types import GainType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10f
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


# TODO(team): improve performance with tile operations?
@wp.kernel
def _qderiv_actuator_passive(
  # Model:
  nu: int,
  opt_timestep: wp.array(dtype=float),
  opt_disableflags: int,
  opt_is_sparse: bool,
  dof_damping: wp.array2d(dtype=float),
  actuator_dyntype: wp.array(dtype=int),
  actuator_gaintype: wp.array(dtype=int),
  actuator_biastype: wp.array(dtype=int),
  actuator_actadr: wp.array(dtype=int),
  actuator_actnum: wp.array(dtype=int),
  actuator_gainprm: wp.array2d(dtype=vec10f),
  actuator_biasprm: wp.array2d(dtype=vec10f),
  # Data in:
  act_in: wp.array2d(dtype=float),
  ctrl_in: wp.array2d(dtype=float),
  actuator_moment_in: wp.array3d(dtype=float),
  qM_in: wp.array3d(dtype=float),
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
  if not opt_disableflags & DisableBit.ACTUATION:
    actuator_gainprm_id = worldid % actuator_gainprm.shape[0]
    actuator_biasprm_id = worldid % actuator_biasprm.shape[0]

    for actid in range(nu):
      if actuator_gaintype[actid] == GainType.AFFINE:
        gain = actuator_gainprm[actuator_gainprm_id, actid][2]
      else:
        gain = 0.0

      if actuator_biastype[actid] == BiasType.AFFINE:
        bias = actuator_biasprm[actuator_biasprm_id, actid][2]
      else:
        bias = 0.0

      if actuator_dyntype[actid] != DynType.NONE:
        act_first = actuator_actadr[actid]
        act_last = act_first + actuator_actnum[actid] - 1
        vel = bias + gain * act_in[worldid, act_last]
      else:
        vel = bias + gain * ctrl_in[worldid, actid]

      qderiv += actuator_moment_in[worldid, actid, dofiid] * actuator_moment_in[worldid, actid, dofjid] * vel

  # TODO(team): fluid model derivative

  if not opt_disableflags & DisableBit.DAMPER and dofiid == dofjid:
    qderiv -= dof_damping[worldid % dof_damping.shape[0], dofiid]

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  if opt_is_sparse:
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
  opt_is_sparse: bool,
  tendon_damping: wp.array2d(dtype=float),
  # Data in:
  ten_J_in: wp.array3d(dtype=float),
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
    qderiv -= ten_J_in[worldid, tenid, dofiid] * ten_J_in[worldid, tenid, dofjid] * tendon_damping[tendon_damping_id, tenid]

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  if opt_is_sparse:
    qDeriv_out[worldid, 0, elemid] -= qderiv
  else:
    qDeriv_out[worldid, dofiid, dofjid] -= qderiv
    if dofiid != dofjid:
      qDeriv_out[worldid, dofjid, dofiid] -= qderiv


@event_scope
def deriv_smooth_vel(m: Model, d: Data, qDeriv: wp.array2d(dtype=float)):
  """Analytical derivative of smooth forces w.r.t. velocities.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    qDeriv: Analytical derivative of smooth forces w.r.t. velocity.
  """
  qMi = m.qM_fullm_i if m.opt.is_sparse else m.dof_tri_row
  qMj = m.qM_fullm_j if m.opt.is_sparse else m.dof_tri_col

  if ~(m.opt.disableflags & (DisableBit.ACTUATION | DisableBit.DAMPER)):
    wp.launch(
      _qderiv_actuator_passive,
      dim=(d.nworld, qMi.size),
      inputs=[
        m.nu,
        m.opt.timestep,
        m.opt.disableflags,
        m.opt.is_sparse,
        m.dof_damping,
        m.actuator_dyntype,
        m.actuator_gaintype,
        m.actuator_biastype,
        m.actuator_actadr,
        m.actuator_actnum,
        m.actuator_gainprm,
        m.actuator_biasprm,
        d.act,
        d.ctrl,
        d.actuator_moment,
        d.qM,
        qMi,
        qMj,
      ],
      outputs=[qDeriv],
    )
  else:
    # TODO(team): directly utilize qM for these settings
    wp.copy(qDeriv, d.qM)

  if not m.opt.disableflags & DisableBit.DAMPER:
    wp.launch(
      _qderiv_tendon_damping,
      dim=(d.nworld, qMi.size),
      inputs=[m.ntendon, m.opt.timestep, m.opt.is_sparse, m.tendon_damping, d.ten_J, qMi, qMj],
      outputs=[qDeriv],
    )

  # TODO(team): rne derivative
