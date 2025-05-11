# Copyright 2025 DeepMind Technologies Limited
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
"""Inverse dynamics functions."""

from jax import numpy as jp
from mujoco.mjx._src import derivative
from mujoco.mjx._src import forward
from mujoco.mjx._src import sensor
from mujoco.mjx._src import smooth
from mujoco.mjx._src import solver
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import EnableBit
from mujoco.mjx._src.types import IntegratorType
from mujoco.mjx._src.types import Model


def discrete_acc(m: Model, d: Data) -> Data:
  """Convert discrete-time qacc to continuous-time qacc."""

  if m.opt.integrator == IntegratorType.RK4:
    raise RuntimeError(
        'discrete inverse dynamics is not supported by RK4 integrator'
    )
  elif m.opt.integrator == IntegratorType.EULER:
    dsbl_eulerdamp = m.opt.disableflags & DisableBit.EULERDAMP
    no_dof_damping = (m.dof_damping == 0).all()
    if dsbl_eulerdamp or no_dof_damping:
      return d

    # set qfrc = (M + h*diag(B)) * qacc
    qfrc = support.mul_m(m, d, d.qacc)
    qfrc += m.opt.timestep * m.dof_damping * d.qacc
  elif m.opt.integrator == IntegratorType.IMPLICITFAST:
    qm = support.full_m(m, d)

    # compute analytical derivative qDeriv; skip rne derivative
    qderiv = derivative.deriv_smooth_vel(m, d)
    if qderiv is not None:
      # M = M - dt*qDeriv
      qm -= m.opt.timestep * qderiv

    # set qfrc = (M - dt*qDeriv) * qacc
    qfrc = qm @ d.qacc
  else:
    raise NotImplementedError(f'integrator {m.opt.integrator} not implemented.')

  # solve for qacc: qfrc = M * qacc
  qacc = smooth.solve_m(m, d, qfrc)

  return d.replace(qacc=qacc)


def inv_constraint(m: Model, d: Data) -> Data:
  """Inverse constraint solver."""

  # no constraints
  if d._impl.efc_J.size == 0:  # pytype: disable=attribute-error
    return d.replace(qfrc_constraint=jp.zeros(m.nv))

  # update
  ctx = solver.Context.create(m, d, grad=False)

  return d.tree_replace({
      'qfrc_constraint': ctx.qfrc_constraint,
      '_impl.efc_force': ctx.efc_force,
  })


def inverse(m: Model, d: Data) -> Data:
  """Inverse dynamics."""
  d = forward.fwd_position(m, d)
  d = sensor.sensor_pos(m, d)
  d = forward.fwd_velocity(m, d)
  d = sensor.sensor_vel(m, d)

  qacc = d.qacc
  if m.opt.enableflags & EnableBit.INVDISCRETE:
    d = discrete_acc(m, d)

  d = inv_constraint(m, d)
  d = smooth.rne(m, d, flg_acc=True)
  d = sensor.sensor_acc(m, d)

  qfrc_inverse = (
      d.qfrc_bias + m.dof_armature * d.qacc - d.qfrc_passive - d.qfrc_constraint
  )

  if m.opt.enableflags & EnableBit.INVDISCRETE:
    return d.replace(qfrc_inverse=qfrc_inverse, qacc=qacc)
  else:
    return d.replace(qfrc_inverse=qfrc_inverse)
