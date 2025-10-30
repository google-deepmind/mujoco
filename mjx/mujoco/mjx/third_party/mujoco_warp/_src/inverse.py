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

from mujoco.mjx.third_party.mujoco_warp._src import derivative
from mujoco.mjx.third_party.mujoco_warp._src import forward
from mujoco.mjx.third_party.mujoco_warp._src import sensor
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import solver
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import EnableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _qfrc_eulerdamp(
  # Model:
  opt_timestep: wp.array(dtype=float),
  dof_damping: wp.array2d(dtype=float),
  # Data in:
  qacc_in: wp.array2d(dtype=float),
  # Out:
  qfrc_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  timestep = opt_timestep[worldid]
  qfrc_out[worldid, dofid] += timestep * dof_damping[worldid, dofid] * qacc_in[worldid, dofid]


@wp.kernel
def _qfrc_inverse(
  # Data in:
  qfrc_bias_in: wp.array2d(dtype=float),
  qfrc_passive_in: wp.array2d(dtype=float),
  qfrc_constraint_in: wp.array2d(dtype=float),
  # In:
  Ma: wp.array2d(dtype=float),
  # Data out:
  qfrc_inverse_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  qfrc_inverse = qfrc_bias_in[worldid, dofid]
  qfrc_inverse += Ma[worldid, dofid]
  qfrc_inverse -= qfrc_passive_in[worldid, dofid]
  qfrc_inverse -= qfrc_constraint_in[worldid, dofid]

  qfrc_inverse_out[worldid, dofid] = qfrc_inverse


def discrete_acc(m: Model, d: Data, qacc: wp.array2d(dtype=float), qfrc: wp.array2d(dtype=float)):
  """Convert discrete-time qacc to continuous-time qacc."""

  if m.opt.integrator == IntegratorType.RK4:
    raise NotImplementedError("discrete inverse dynamics is not supported by RK4 integrator")
  elif m.opt.integrator == IntegratorType.EULER:
    if m.opt.disableflags & DisableBit.EULERDAMP:
      wp.copy(qacc, d.qacc)
      return

    # TODO(team): qacc = d.qacc if (m.dof_damping == 0.0).all()

    # set qfrc = (d.qM + m.opt.timestep * diag(m.dof_damping)) * d.qacc

    # d.qM @ d.qacc
    support.mul_m(m, d, qfrc, d.qacc, d.inverse_mul_m_skip)

    # qfrc += m.opt.timestep * m.dof_damping * d.qacc
    wp.launch(
      _qfrc_eulerdamp,
      dim=(d.nworld, m.nv),
      inputs=[m.opt.timestep, m.dof_damping, d.qacc],
      outputs=[qfrc],
    )
  elif m.opt.integrator == IntegratorType.IMPLICITFAST:
    derivative.deriv_smooth_vel(m, d, flg_forward=False)
    smooth.factor_solve_i(m, d, d.qM, d.qLD, d.qLDiagInv, qacc, qfrc)
  else:
    raise NotImplementedError(f"integrator {m.opt.integrator} not implemented.")

  # solve for qacc: qfrc = d.qM @ d.qacc
  smooth.solve_m(m, d, qacc, qfrc)


def inv_constraint(m: Model, d: Data):
  """Inverse constraint solver."""

  # no constraints
  if d.njmax == 0:
    d.qfrc_constraint.zero_()
    return

  # update
  solver.create_context(m, d, grad=False)


def inverse(m: Model, d: Data):
  """Inverse dynamics."""
  forward.fwd_position(m, d)
  sensor.sensor_pos(m, d)
  forward.fwd_velocity(m, d)
  sensor.sensor_vel(m, d)

  invdiscrete = m.opt.enableflags & EnableBit.INVDISCRETE
  if invdiscrete:
    # save discrete-time qacc and compute continuous-time qacc
    wp.copy(d.qacc_discrete, d.qacc)
    discrete_acc(m, d, d.qacc, d.qfrc_integration)

  inv_constraint(m, d)
  smooth.rne(m, d)
  smooth.tendon_bias(m, d, d.qfrc_bias)
  sensor.sensor_acc(m, d)

  support.mul_m(m, d, d.qfrc_inverse, d.qacc, d.inverse_mul_m_skip)

  wp.launch(
    _qfrc_inverse,
    dim=(d.nworld, m.nv),
    inputs=[
      d.qfrc_bias,
      d.qfrc_passive,
      d.qfrc_constraint,
      d.qfrc_inverse,
    ],
    outputs=[d.qfrc_inverse],
  )

  if invdiscrete:
    # restore discrete-time qacc
    wp.copy(d.qacc, d.qacc_discrete)
