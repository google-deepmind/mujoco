# Copyright 2023 DeepMind Technologies Limited
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
"""Forward step functions."""

import functools
from typing import Optional, Sequence

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import constraint
from mujoco.mjx._src import derivative
from mujoco.mjx._src import math
from mujoco.mjx._src import passive
from mujoco.mjx._src import scan
from mujoco.mjx._src import sensor
from mujoco.mjx._src import smooth
from mujoco.mjx._src import solver
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import BiasType
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DataJAX
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import DynType
from mujoco.mjx._src.types import GainType
from mujoco.mjx._src.types import IntegratorType
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import ModelJAX
from mujoco.mjx._src.types import TrnType
# pylint: enable=g-importing-member
import numpy as np

# RK4 tableau
_RK4_A = np.array([
    [0.5, 0.0, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 1.0],
])
_RK4_B = np.array([1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0])


def named_scope(fn, name: str = ''):
  @functools.wraps(fn)
  def wrapper(*args, **kwargs):
    with jax.named_scope(name or getattr(fn, '__name__')):
      res = fn(*args, **kwargs)
    return res

  return wrapper


@named_scope
def fwd_position(m: Model, d: Data) -> Data:
  """Position-dependent computations."""
  # TODO(robotics-simulation): tendon
  d = smooth.kinematics(m, d)
  d = smooth.com_pos(m, d)
  d = smooth.camlight(m, d)
  d = smooth.tendon(m, d)
  d = smooth.crb(m, d)
  d = smooth.tendon_armature(m, d)
  d = smooth.factor_m(m, d)
  d = collision_driver.collision(m, d)
  d = constraint.make_constraint(m, d)
  d = smooth.transmission(m, d)
  return d


@named_scope
def fwd_velocity(m: Model, d: Data) -> Data:
  """Velocity-dependent computations."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('fwd_velocity requires JAX backend implementation.')

  d = d.tree_replace({
      '_impl.actuator_velocity': d._impl.actuator_moment @ d.qvel,
      '_impl.ten_velocity': d._impl.ten_J @ d.qvel,
  })
  d = smooth.com_vel(m, d)
  d = passive.passive(m, d)
  d = smooth.rne(m, d)
  d = smooth.tendon_bias(m, d)
  return d


@named_scope
def fwd_actuation(m: Model, d: Data) -> Data:
  """Actuation-dependent computations."""
  if not m.nu or m.opt.disableflags & DisableBit.ACTUATION:
    return d.replace(
        act_dot=jp.zeros((m.na,)),
        qfrc_actuator=jp.zeros((m.nv,)),
    )

  ctrl = d.ctrl
  if not m.opt.disableflags & DisableBit.CLAMPCTRL:
    ctrlrange = jp.where(
        m.actuator_ctrllimited[:, None],
        m.actuator_ctrlrange,
        jp.array([-jp.inf, jp.inf]),
    )
    ctrl = jp.clip(ctrl, ctrlrange[:, 0], ctrlrange[:, 1])

  # act_dot for stateful actuators
  def get_act_dot(dyn_typ, dyn_prm, ctrl, act):
    if dyn_typ == DynType.NONE:
      act_dot = jp.array(0.0)
    elif dyn_typ == DynType.INTEGRATOR:
      act_dot = ctrl
    elif dyn_typ in (DynType.FILTER, DynType.FILTEREXACT):
      act_dot = (ctrl - act) / jp.clip(dyn_prm[0], mujoco.mjMINVAL)
    elif dyn_typ == DynType.MUSCLE:
      act_dot = support.muscle_dynamics(ctrl, act, dyn_prm)
    else:
      raise NotImplementedError(f'dyntype {dyn_typ.name} not implemented.')
    return act_dot

  act_dot = jp.zeros((m.na,))
  if m.na:
    act_dot = scan.flat(
        m,
        get_act_dot,
        'uuua',
        'a',
        m.actuator_dyntype,
        m.actuator_dynprm,
        ctrl,
        d.act,
        group_by='u',
    )

  ctrl_act = ctrl
  if m.na:
    act_last_dim = d.act[m.actuator_actadr + m.actuator_actnum - 1]
    ctrl_act = jp.where(m.actuator_actadr == -1, ctrl, act_last_dim)

  def get_force(*args):
    gain_t, gain_p, bias_t, bias_p, len_, vel, ctrl_act, len_range, acc0 = args

    typ, prm = GainType(gain_t), gain_p
    if typ == GainType.FIXED:
      gain = prm[0]
    elif typ == GainType.AFFINE:
      gain = prm[0] + prm[1] * len_ + prm[2] * vel
    elif typ == GainType.MUSCLE:
      gain = support.muscle_gain(len_, vel, len_range, acc0, prm)
    else:
      raise RuntimeError(f'unrecognized gaintype {typ.name}.')

    typ, prm = BiasType(bias_t), bias_p
    bias = jp.array(0.0)
    if typ == BiasType.AFFINE:
      bias = prm[0] + prm[1] * len_ + prm[2] * vel
    elif typ == BiasType.MUSCLE:
      bias = support.muscle_bias(len_, len_range, acc0, prm)

    return gain * ctrl_act + bias

  force = scan.flat(
      m,
      get_force,
      'uuuuuuuuu',
      'u',
      m.actuator_gaintype,
      m.actuator_gainprm,
      m.actuator_biastype,
      m.actuator_biasprm,
      d._impl.actuator_length,
      d._impl.actuator_velocity,
      ctrl_act,
      jp.array(m.actuator_lengthrange),
      jp.array(m.actuator_acc0),
      group_by='u',
  )

  # tendon total force clamping
  if np.any(m.tendon_actfrclimited):
    (tendon_actfrclimited_id,) = np.nonzero(m.tendon_actfrclimited)
    actuator_tendon = m.actuator_trntype == TrnType.TENDON

    force_mask = [
        actuator_tendon & (m.actuator_trnid[:, 0] == tendon_id)
        for tendon_id in tendon_actfrclimited_id
    ]
    force_ids = np.concatenate([np.nonzero(mask)[0] for mask in force_mask])
    force_mat = np.array(force_mask)[:, force_ids]
    tendon_total_force = force_mat @ force[force_ids]

    force_scaling = jp.where(
        tendon_total_force < m.tendon_actfrcrange[tendon_actfrclimited_id, 0],
        m.tendon_actfrcrange[tendon_actfrclimited_id, 0] / tendon_total_force,
        1,
    )
    force_scaling = jp.where(
        tendon_total_force > m.tendon_actfrcrange[tendon_actfrclimited_id, 1],
        m.tendon_actfrcrange[tendon_actfrclimited_id, 1] / tendon_total_force,
        force_scaling,
    )

    tendon_forces = force[force_ids] * (force_mat.T @ force_scaling)
    force = force.at[force_ids].set(tendon_forces)

  forcerange = jp.where(
      m.actuator_forcelimited[:, None],
      m.actuator_forcerange,
      jp.array([-jp.inf, jp.inf]),
  )
  force = jp.clip(force, forcerange[:, 0], forcerange[:, 1])

  qfrc_actuator = d._impl.actuator_moment.T @ force

  if m.ngravcomp:
    # actuator-level gravity compensation, skip if added as passive force
    qfrc_actuator += d.qfrc_gravcomp * m.jnt_actgravcomp[m.dof_jntid]

  # clamp qfrc_actuator
  actfrcrange = jp.where(
      m.jnt_actfrclimited[:, None],
      m.jnt_actfrcrange,
      jp.array([-jp.inf, jp.inf]),
  )
  actfrcrange = actfrcrange[m.dof_jntid]
  qfrc_actuator = jp.clip(qfrc_actuator, actfrcrange[:, 0], actfrcrange[:, 1])

  d = d.replace(
      act_dot=act_dot, qfrc_actuator=qfrc_actuator, actuator_force=force
  )
  return d


@named_scope
def fwd_acceleration(m: Model, d: Data) -> Data:
  """Add up all non-constraint forces, compute qacc_smooth."""
  qfrc_applied = d.qfrc_applied + support.xfrc_accumulate(m, d)
  qfrc_smooth = d.qfrc_passive - d.qfrc_bias + d.qfrc_actuator + qfrc_applied
  qacc_smooth = smooth.solve_m(m, d, qfrc_smooth)
  d = d.replace(qfrc_smooth=qfrc_smooth, qacc_smooth=qacc_smooth)
  return d


@named_scope
def _integrate_pos(
    jnt_typs: Sequence[str], qpos: jax.Array, qvel: jax.Array, dt: jax.Array
) -> jax.Array:
  """Integrate position given velocity."""
  qs, qi, vi = [], 0, 0

  for jnt_typ in jnt_typs:
    if jnt_typ == JointType.FREE:
      pos = qpos[qi : qi + 3] + dt * qvel[vi : vi + 3]
      quat = math.quat_integrate(
          qpos[qi + 3 : qi + 7], qvel[vi + 3 : vi + 6], dt
      )
      qs.append(jp.concatenate([pos, quat]))
      qi, vi = qi + 7, vi + 6
    elif jnt_typ == JointType.BALL:
      quat = math.quat_integrate(qpos[qi : qi + 4], qvel[vi : vi + 3], dt)
      qs.append(quat)
      qi, vi = qi + 4, vi + 3
    elif jnt_typ in (JointType.HINGE, JointType.SLIDE):
      pos = qpos[qi] + dt * qvel[vi]
      qs.append(pos[None])
      qi, vi = qi + 1, vi + 1
    else:
      raise RuntimeError(f'unrecognized joint type: {jnt_typ}')

  return jp.concatenate(qs) if qs else jp.empty((0,))


def _next_activation(m: Model, d: Data, act_dot: jax.Array) -> jax.Array:
  """Returns the next act given the current act_dot, after clamping."""
  act = d.act

  if not m.na:
    return act

  actrange = jp.where(
      m.actuator_actlimited[:, None],
      m.actuator_actrange,
      jp.array([-jp.inf, jp.inf]),
  )

  def fn(dyntype, dynprm, act, act_dot, actrange):
    if dyntype == DynType.FILTEREXACT:
      tau = jp.clip(dynprm[0], a_min=mujoco.mjMINVAL)
      act = act + act_dot * tau * (1 - jp.exp(-m.opt.timestep / tau))
    else:
      act = act + act_dot * m.opt.timestep
    act = jp.clip(act, actrange[0], actrange[1])
    return act

  args = (m.actuator_dyntype, m.actuator_dynprm, act, act_dot, actrange)
  act = scan.flat(m, fn, 'uuaau', 'a', *args, group_by='u')

  return act.reshape(m.na)


@named_scope
def _advance(
    m: Model,
    d: Data,
    act_dot: jax.Array,
    qacc: jax.Array,
    qvel: Optional[jax.Array] = None,
) -> Data:
  """Advance state and time given activation derivatives and acceleration."""
  act = _next_activation(m, d, act_dot)

  # advance velocities
  d = d.replace(qvel=d.qvel + qacc * m.opt.timestep)

  # advance positions with qvel if given, d.qvel otherwise (semi-implicit)
  qvel = d.qvel if qvel is None else qvel
  integrate_fn = lambda *args: _integrate_pos(*args, dt=m.opt.timestep)
  qpos = scan.flat(m, integrate_fn, 'jqv', 'q', m.jnt_type, d.qpos, qvel)

  # advance time
  time = d.time + m.opt.timestep

  return d.replace(act=act, qpos=qpos, time=time)


@named_scope
def euler(m: Model, d: Data) -> Data:
  """Euler integrator, semi-implicit in velocity."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('euler requires JAX backend implementation.')

  # integrate damping implicitly
  qacc = d.qacc
  if not m.opt.disableflags & DisableBit.EULERDAMP:
    if support.is_sparse(m):
      qM = d._impl.qM.at[m.dof_Madr].add(m.opt.timestep * m.dof_damping)
    else:
      qM = d._impl.qM + jp.diag(m.opt.timestep * m.dof_damping)
    dh = d.tree_replace({'_impl.qM': qM})
    dh = smooth.factor_m(m, dh)
    qfrc = d.qfrc_smooth + d.qfrc_constraint
    qacc = smooth.solve_m(m, dh, qfrc)
  return _advance(m, d, d.act_dot, qacc)


@named_scope
def rungekutta4(m: Model, d: Data) -> Data:
  """Runge-Kutta explicit order 4 integrator."""
  d0 = d
  # pylint: disable=invalid-name
  A, B = _RK4_A, _RK4_B
  C = jp.tril(A).sum(axis=0)  # C(i) = sum_j A(i,j)
  T = d.time + C * m.opt.timestep
  # pylint: enable=invalid-name

  kqvel = d.qvel  # intermediate RK solution
  # RK solutions sum
  qvel, qacc, act_dot = jax.tree_util.tree_map(
      lambda k: B[0] * k, (kqvel, d.qacc, d.act_dot)
  )
  integrate_fn = lambda *args: _integrate_pos(*args, dt=m.opt.timestep)

  def f(carry, x):
    qvel, qacc, act_dot, kqvel, d = carry
    a, b, t = x  # tableau numbers
    dqvel, dqacc, dact_dot = jax.tree_util.tree_map(
        lambda k: a * k, (kqvel, d.qacc, d.act_dot)
    )
    # get intermediate RK solutions
    kqpos = scan.flat(m, integrate_fn, 'jqv', 'q', m.jnt_type, d0.qpos, dqvel)
    kact = d0.act + dact_dot * m.opt.timestep
    kqvel = d0.qvel + dqacc * m.opt.timestep
    d = d.replace(qpos=kqpos, qvel=kqvel, act=kact, time=t)
    d = forward(m, d)

    qvel += b * kqvel
    qacc += b * d.qacc
    act_dot += b * d.act_dot

    return (qvel, qacc, act_dot, kqvel, d), None

  abt = jp.vstack([jp.diag(A), B[1:4], T]).T
  out, _ = jax.lax.scan(f, (qvel, qacc, act_dot, kqvel, d), abt, unroll=3)
  qvel, qacc, act_dot, _, d1 = out

  d = d1.replace(qpos=d0.qpos, qvel=d0.qvel, act=d0.act, time=d0.time)
  d = _advance(m, d, act_dot, qacc, qvel)
  return d


@named_scope
def implicit(m: Model, d: Data) -> Data:
  """Integrates fully implicit in velocity."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('implicit requires JAX backend implementation.')

  qderiv = derivative.deriv_smooth_vel(m, d)

  qacc = d.qacc
  if qderiv is not None:
    # TODO(robotics-simulation): use smooth.factor_m / solve_m here:
    qm = support.full_m(m, d) if support.is_sparse(m) else d._impl.qM
    qm -= m.opt.timestep * qderiv
    qh, _ = jax.scipy.linalg.cho_factor(qm)
    qfrc = d.qfrc_smooth + d.qfrc_constraint
    qacc = jax.scipy.linalg.cho_solve((qh, False), qfrc)

  return _advance(m, d, d.act_dot, qacc)


@named_scope
def forward(m: Model, d: Data) -> Data:
  """Forward dynamics."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('forward requires JAX backend implementation.')

  d = fwd_position(m, d)
  d = sensor.sensor_pos(m, d)
  d = fwd_velocity(m, d)
  d = sensor.sensor_vel(m, d)
  d = fwd_actuation(m, d)
  d = fwd_acceleration(m, d)
  d = sensor.sensor_acc(m, d)

  if d._impl.efc_J.size == 0:
    d = d.replace(qacc=d.qacc_smooth)
    return d

  d = named_scope(solver.solve)(m, d)

  return d


@named_scope
def step(m: Model, d: Data) -> Data:
  """Advance simulation."""
  d = forward(m, d)

  if m.opt.integrator == IntegratorType.EULER:
    d = euler(m, d)
  elif m.opt.integrator == IntegratorType.RK4:
    d = rungekutta4(m, d)
  elif m.opt.integrator == IntegratorType.IMPLICITFAST:
    d = implicit(m, d)
  else:
    raise NotImplementedError(f'integrator {m.opt.integrator} not implemented.')

  return d
