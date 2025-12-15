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

from typing import Optional

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import collision_driver
from mujoco.mjx.third_party.mujoco_warp._src import constraint
from mujoco.mjx.third_party.mujoco_warp._src import derivative
from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import passive
from mujoco.mjx.third_party.mujoco_warp._src import sensor
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import solver
from mujoco.mjx.third_party.mujoco_warp._src import util_misc
from mujoco.mjx.third_party.mujoco_warp._src.support import xfrc_accumulate
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import DynType
from mujoco.mjx.third_party.mujoco_warp._src.types import EnableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import GainType
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import TileSet
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10f
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import nested_kernel

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _next_position(
  # Model:
  opt_timestep: wp.array(dtype=float),
  jnt_type: wp.array(dtype=int),
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  # Data in:
  qpos_in: wp.array2d(dtype=float),
  qvel_in: wp.array2d(dtype=float),
  # In:
  qvel_scale_in: float,
  # Data out:
  qpos_out: wp.array2d(dtype=float),
):
  worldid, jntid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  jnttype = jnt_type[jntid]
  qpos_adr = jnt_qposadr[jntid]
  dof_adr = jnt_dofadr[jntid]
  qpos = qpos_in[worldid]
  qpos_next = qpos_out[worldid]
  qvel = qvel_in[worldid]

  if jnttype == JointType.FREE:
    qpos_pos = wp.vec3(qpos[qpos_adr], qpos[qpos_adr + 1], qpos[qpos_adr + 2])
    qvel_lin = wp.vec3(qvel[dof_adr], qvel[dof_adr + 1], qvel[dof_adr + 2]) * qvel_scale_in

    qpos_new = qpos_pos + timestep * qvel_lin

    qpos_quat = wp.quat(
      qpos[qpos_adr + 3],
      qpos[qpos_adr + 4],
      qpos[qpos_adr + 5],
      qpos[qpos_adr + 6],
    )
    qvel_ang = wp.vec3(qvel[dof_adr + 3], qvel[dof_adr + 4], qvel[dof_adr + 5]) * qvel_scale_in

    qpos_quat_new = math.quat_integrate(qpos_quat, qvel_ang, timestep)

    qpos_next[qpos_adr + 0] = qpos_new[0]
    qpos_next[qpos_adr + 1] = qpos_new[1]
    qpos_next[qpos_adr + 2] = qpos_new[2]
    qpos_next[qpos_adr + 3] = qpos_quat_new[0]
    qpos_next[qpos_adr + 4] = qpos_quat_new[1]
    qpos_next[qpos_adr + 5] = qpos_quat_new[2]
    qpos_next[qpos_adr + 6] = qpos_quat_new[3]

  elif jnttype == JointType.BALL:
    qpos_quat = wp.quat(qpos[qpos_adr + 0], qpos[qpos_adr + 1], qpos[qpos_adr + 2], qpos[qpos_adr + 3])
    qvel_ang = wp.vec3(qvel[dof_adr], qvel[dof_adr + 1], qvel[dof_adr + 2]) * qvel_scale_in

    qpos_quat_new = math.quat_integrate(qpos_quat, qvel_ang, timestep)

    qpos_next[qpos_adr + 0] = qpos_quat_new[0]
    qpos_next[qpos_adr + 1] = qpos_quat_new[1]
    qpos_next[qpos_adr + 2] = qpos_quat_new[2]
    qpos_next[qpos_adr + 3] = qpos_quat_new[3]

  else:  # if jnt_type in (JointType.HINGE, JointType.SLIDE):
    qpos_next[qpos_adr] = qpos[qpos_adr] + timestep * qvel[dof_adr] * qvel_scale_in


@wp.kernel
def _next_velocity(
  # Model:
  opt_timestep: wp.array(dtype=float),
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  qacc_in: wp.array2d(dtype=float),
  # In:
  qacc_scale_in: float,
  # Data out:
  qvel_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]
  qvel_out[worldid, dofid] = qvel_in[worldid, dofid] + qacc_scale_in * qacc_in[worldid, dofid] * timestep


# TODO(team): kernel analyzer array slice?
@wp.func
def _next_act(
  # Model:
  opt_timestep: float,  # kernel_analyzer: ignore
  actuator_dyntype: int,  # kernel_analyzer: ignore
  actuator_dynprm: vec10f,  # kernel_analyzer: ignore
  actuator_actrange: wp.vec2,  # kernel_analyzer: ignore
  # Data In:
  act_in: float,  # kernel_analyzer: ignore
  act_dot_in: float,  # kernel_analyzer: ignore
  # In:
  act_dot_scale: float,
  clamp: bool,
) -> float:
  # advance actuation
  if actuator_dyntype == DynType.FILTEREXACT:
    tau = wp.max(MJ_MINVAL, actuator_dynprm[0])
    act = act_in + act_dot_scale * act_dot_in * tau * (1.0 - wp.exp(-opt_timestep / tau))
  else:
    act = act_in + act_dot_scale * act_dot_in * opt_timestep

  # clamp to actrange
  if clamp:
    act = wp.clamp(act, actuator_actrange[0], actuator_actrange[1])

  return act


@wp.kernel
def _next_activation(
  # Model:
  opt_timestep: wp.array(dtype=float),
  actuator_dyntype: wp.array(dtype=int),
  actuator_actlimited: wp.array(dtype=bool),
  actuator_dynprm: wp.array2d(dtype=vec10f),
  actuator_actrange: wp.array2d(dtype=wp.vec2),
  # Data in:
  act_in: wp.array2d(dtype=float),
  act_dot_in: wp.array2d(dtype=float),
  # In:
  act_dot_scale: float,
  limit: bool,
  # Data out:
  act_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()
  opt_timestep_id = worldid % opt_timestep.shape[0]
  actuator_dynprm_id = worldid % actuator_dynprm.shape[0]
  actuator_actrange_id = worldid % actuator_actrange.shape[0]
  act = _next_act(
    opt_timestep[opt_timestep_id],
    actuator_dyntype[actid],
    actuator_dynprm[actuator_dynprm_id, actid],
    actuator_actrange[actuator_actrange_id, actid],
    act_in[worldid, actid],
    act_dot_in[worldid, actid],
    act_dot_scale,
    limit and actuator_actlimited[actid],
  )
  act_out[worldid, actid] = act


@wp.kernel
def _next_time(
  # Model:
  opt_timestep: wp.array(dtype=float),
  # Data in:
  nefc_in: wp.array(dtype=int),
  time_in: wp.array(dtype=float),
  nworld_in: int,
  naconmax_in: int,
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  ncollision_in: wp.array(dtype=int),
  # Data out:
  time_out: wp.array(dtype=float),
):
  worldid = wp.tid()
  time_out[worldid] = time_in[worldid] + opt_timestep[worldid % opt_timestep.shape[0]]
  nefc = nefc_in[worldid]

  if nefc > njmax_in:
    wp.printf("nefc overflow - please increase njmax to %u\n", nefc)

  if worldid == 0:
    ncollision = ncollision_in[0]
    if ncollision > naconmax_in:
      nconmax = int(wp.ceil(float(ncollision) / float(nworld_in)))
      wp.printf("broadphase overflow - please increase nconmax to %u or naconmax to %u\n", nconmax, ncollision)

    if nacon_in[0] > naconmax_in:
      nconmax = int(wp.ceil(float(nacon_in[0]) / float(nworld_in)))
      wp.printf("narrowphase overflow - please increase nconmax to %u or naconmax to %u\n", nconmax, nacon_in[0])


def _advance(m: Model, d: Data, qacc: wp.array, qvel: Optional[wp.array] = None):
  """Advance state and time given activation derivatives and acceleration."""
  # TODO(team): can we assume static timesteps?

  # advance activations
  wp.launch(
    _next_activation,
    dim=(d.nworld, m.na),
    inputs=[
      m.opt.timestep,
      m.actuator_dyntype,
      m.actuator_actlimited,
      m.actuator_dynprm,
      m.actuator_actrange,
      d.act,
      d.act_dot,
      1.0,
      True,
    ],
    outputs=[d.act],
  )

  wp.launch(
    _next_velocity,
    dim=(d.nworld, m.nv),
    inputs=[m.opt.timestep, d.qvel, qacc, 1.0],
    outputs=[d.qvel],
  )

  # advance positions with qvel if given, d.qvel otherwise (semi-implicit)
  qvel_in = qvel or d.qvel

  wp.launch(
    _next_position,
    dim=(d.nworld, m.njnt),
    inputs=[m.opt.timestep, m.jnt_type, m.jnt_qposadr, m.jnt_dofadr, d.qpos, qvel_in, 1.0],
    outputs=[d.qpos],
  )

  wp.launch(
    _next_time,
    dim=d.nworld,
    inputs=[m.opt.timestep, d.nefc, d.time, d.nworld, d.naconmax, d.njmax, d.nacon, d.ncollision],
    outputs=[d.time],
  )

  wp.copy(d.qacc_warmstart, d.qacc)


@wp.kernel
def _euler_damp_qfrc_sparse(
  # Model:
  opt_timestep: wp.array(dtype=float),
  dof_Madr: wp.array(dtype=int),
  dof_damping: wp.array2d(dtype=float),
  # Out:
  qM_integration_out: wp.array3d(dtype=float),
):
  worldid, tid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  adr = dof_Madr[tid]
  qM_integration_out[worldid, 0, adr] += timestep * dof_damping[worldid, tid]


@cache_kernel
def _tile_euler_dense(tile: TileSet):
  @nested_kernel(module="unique", enable_backward=False)
  def euler_dense(
    # Model:
    dof_damping: wp.array2d(dtype=float),
    opt_timestep: wp.array(dtype=float),
    # Data in:
    qM_in: wp.array3d(dtype=float),
    efc_Ma_in: wp.array2d(dtype=float),
    # In:
    adr_in: wp.array(dtype=int),
    # Out:
    qacc_out: wp.array2d(dtype=float),
  ):
    worldid, nodeid = wp.tid()
    timestep = opt_timestep[worldid % opt_timestep.shape[0]]
    TILE_SIZE = wp.static(tile.size)

    dofid = adr_in[nodeid]
    M_tile = wp.tile_load(qM_in[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(dofid, dofid))
    damping_tile = wp.tile_load(dof_damping[worldid % dof_damping.shape[0]], shape=(TILE_SIZE,), offset=(dofid,))
    damping_scaled = damping_tile * timestep
    qm_integration_tile = wp.tile_diag_add(M_tile, damping_scaled)

    Ma_tile = wp.tile_load(efc_Ma_in[worldid], shape=(TILE_SIZE,), offset=(dofid,))
    L_tile = wp.tile_cholesky(qm_integration_tile)
    qacc_tile = wp.tile_cholesky_solve(L_tile, Ma_tile)
    wp.tile_store(qacc_out[worldid], qacc_tile, offset=(dofid))

  return euler_dense


@event_scope
def euler(m: Model, d: Data):
  """Euler integrator, semi-implicit in velocity."""
  # integrate damping implicitly
  if not m.opt.disableflags & (DisableBit.EULERDAMP | DisableBit.DAMPER):
    qacc = wp.empty((d.nworld, m.nv), dtype=float)
    if m.opt.is_sparse:
      qM = wp.clone(d.qM)
      qLD = wp.empty((d.nworld, 1, m.nC), dtype=float)
      qLDiagInv = wp.empty((d.nworld, m.nv), dtype=float)
      wp.launch(
        _euler_damp_qfrc_sparse,
        dim=(d.nworld, m.nv),
        inputs=[m.opt.timestep, m.dof_Madr, m.dof_damping],
        outputs=[qM],
      )
      smooth.factor_solve_i(m, d, qM, qLD, qLDiagInv, qacc, d.efc.Ma)
    else:
      for tile in m.qM_tiles:
        wp.launch_tiled(
          _tile_euler_dense(tile),
          dim=(d.nworld, tile.adr.size),
          inputs=[m.dof_damping, m.opt.timestep, d.qM, d.efc.Ma, tile.adr],
          outputs=[qacc],
          block_dim=m.block_dim.euler_dense,
        )
    _advance(m, d, qacc)
  else:
    _advance(m, d, d.qacc)


def _rk_perturb_state(
  m: Model,
  d: Data,
  scale: float,
  qpos_t0: wp.array2d(dtype=float),
  qvel_t0: wp.array2d(dtype=float),
  act_t0: Optional[wp.array] = None,
):
  # position
  wp.launch(
    _next_position,
    dim=(d.nworld, m.njnt),
    inputs=[m.opt.timestep, m.jnt_type, m.jnt_qposadr, m.jnt_dofadr, qpos_t0, d.qvel, scale],
    outputs=[d.qpos],
  )

  # velocity
  wp.launch(
    _next_velocity,
    dim=(d.nworld, m.nv),
    inputs=[m.opt.timestep, qvel_t0, d.qacc, scale],
    outputs=[d.qvel],
  )

  # activation
  if m.na and act_t0 is not None:
    wp.launch(
      _next_activation,
      dim=(d.nworld, m.na),
      inputs=[m.opt.timestep, act_t0, d.act_dot, scale, False],
      outputs=[d.act],
    )


@wp.kernel
def _rk_accumulate_velocity_acceleration(
  # Data in:
  qvel_in: wp.array2d(dtype=float),
  qacc_in: wp.array2d(dtype=float),
  # In:
  scale: float,
  # Data out:
  qvel_out: wp.array2d(dtype=float),
  qacc_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  qvel_out[worldid, dofid] += scale * qvel_in[worldid, dofid]
  qacc_out[worldid, dofid] += scale * qacc_in[worldid, dofid]


@wp.kernel
def _rk_accumulate_activation_velocity(
  # Data in:
  act_dot_in: wp.array2d(dtype=float),
  # In:
  scale: float,
  # Data out:
  act_dot_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()
  act_dot_out[worldid, actid] += scale * act_dot_in[worldid, actid]


def _rk_accumulate(
  m: Model,
  d: Data,
  scale: float,
  qvel_rk: wp.array2d(dtype=float),
  qacc_rk: wp.array2d(dtype=float),
  act_dot_rk: Optional[wp.array] = None,
):
  """Computes one term of 1/6 k_1 + 1/3 k_2 + 1/3 k_3 + 1/6 k_4."""
  wp.launch(
    _rk_accumulate_velocity_acceleration,
    dim=(d.nworld, m.nv),
    inputs=[d.qvel, d.qacc, scale],
    outputs=[qvel_rk, qacc_rk],
  )

  if m.na and act_dot_rk is not None:
    wp.launch(
      _rk_accumulate_activation_velocity,
      dim=(d.nworld, m.na),
      inputs=[d.act_dot, scale],
      outputs=[act_dot_rk],
    )


@event_scope
def rungekutta4(m: Model, d: Data):
  """Runge-Kutta explicit order 4 integrator."""
  # RK4 tableau
  A = [0.5, 0.5, 1.0]  # diagonal only
  B = [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0]

  qpos_t0 = wp.clone(d.qpos)
  qvel_t0 = wp.clone(d.qvel)
  qvel_rk = wp.zeros((d.nworld, m.nv), dtype=float)
  qacc_rk = wp.zeros((d.nworld, m.nv), dtype=float)

  if m.na:
    act_t0 = wp.clone(d.act)
    act_dot_rk = wp.zeros((d.nworld, m.na), dtype=float)
  else:
    act_t0 = None
    act_dot_rk = None

  _rk_accumulate(m, d, B[0], qvel_rk, qacc_rk, act_dot_rk)

  for i in range(3):
    a, b = float(A[i]), B[i + 1]
    _rk_perturb_state(m, d, a, qpos_t0, qvel_t0, act_t0)
    forward(m, d)
    _rk_accumulate(m, d, b, qvel_rk, qacc_rk, act_dot_rk)

  wp.copy(d.qpos, qpos_t0)
  wp.copy(d.qvel, qvel_t0)

  if m.na:
    wp.copy(d.act, act_t0)
    wp.copy(d.act_dot, act_dot_rk)

  _advance(m, d, qacc_rk, qvel_rk)


@event_scope
def implicit(m: Model, d: Data):
  """Integrates fully implicit in velocity."""
  if ~(m.opt.disableflags | ~(DisableBit.ACTUATION | DisableBit.SPRING | DisableBit.DAMPER)):
    if m.opt.is_sparse:
      qDeriv = wp.empty((d.nworld, 1, m.nM), dtype=float)
      qLD = wp.empty((d.nworld, 1, m.nC), dtype=float)
    else:
      qDeriv = wp.empty((d.nworld, m.nv, m.nv), dtype=float)
      qLD = wp.empty((d.nworld, m.nv, m.nv), dtype=float)
    qLDiagInv = wp.empty((d.nworld, m.nv), dtype=float)
    derivative.deriv_smooth_vel(m, d, qDeriv)
    qacc = wp.empty((d.nworld, m.nv), dtype=float)
    smooth.factor_solve_i(m, d, qDeriv, qLD, qLDiagInv, qacc, d.efc.Ma)
    _advance(m, d, qacc)
  else:
    _advance(m, d, d.qacc)


@event_scope
def fwd_position(m: Model, d: Data, factorize: bool = True):
  """Position-dependent computations.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    factorize: Flag to factorize interia matrix.
  """
  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.camlight(m, d)
  smooth.flex(m, d)
  smooth.tendon(m, d)
  smooth.crb(m, d)
  smooth.tendon_armature(m, d)
  if factorize:
    smooth.factor_m(m, d)
  if m.opt.run_collision_detection:
    collision_driver.collision(m, d)
  constraint.make_constraint(m, d)
  smooth.transmission(m, d)


# TODO(team): sparse actuator_moment version
@cache_kernel
def _actuator_velocity(nv: int):
  @nested_kernel(module="unique", enable_backward=False)
  def actuator_velocity(
    # Data in:
    qvel_in: wp.array2d(dtype=float),
    actuator_moment_in: wp.array3d(dtype=float),
    # Data out:
    actuator_velocity_out: wp.array2d(dtype=float),
  ):
    worldid, actid = wp.tid()
    moment_tile = wp.tile_load(actuator_moment_in[worldid, actid], shape=wp.static(nv))
    qvel_tile = wp.tile_load(qvel_in[worldid], shape=wp.static(nv))
    moment_qvel_tile = wp.tile_map(wp.mul, moment_tile, qvel_tile)
    actuator_velocity_tile = wp.tile_reduce(wp.add, moment_qvel_tile)
    actuator_velocity_out[worldid, actid] = actuator_velocity_tile[0]

  return actuator_velocity


@cache_kernel
def _tendon_velocity(nv: int):
  @nested_kernel(module="unique", enable_backward=False)
  def tendon_velocity(
    # Data in:
    qvel_in: wp.array2d(dtype=float),
    ten_J_in: wp.array3d(dtype=float),
    # Data out:
    ten_velocity_out: wp.array2d(dtype=float),
  ):
    worldid, tenid = wp.tid()
    ten_J_tile = wp.tile_load(ten_J_in[worldid, tenid], shape=wp.static(nv))
    qvel_tile = wp.tile_load(qvel_in[worldid], shape=wp.static(nv))
    ten_J_qvel_tile = wp.tile_map(wp.mul, ten_J_tile, qvel_tile)
    ten_velocity_tile = wp.tile_reduce(wp.add, ten_J_qvel_tile)
    ten_velocity_out[worldid, tenid] = ten_velocity_tile[0]

  return tendon_velocity


@event_scope
def fwd_velocity(m: Model, d: Data):
  """Velocity-dependent computations."""
  wp.launch_tiled(
    _actuator_velocity(m.nv),
    dim=(d.nworld, m.nu),
    inputs=[d.qvel, d.actuator_moment],
    outputs=[d.actuator_velocity],
    block_dim=m.block_dim.actuator_velocity,
  )

  # TODO(team): sparse version
  wp.launch_tiled(
    _tendon_velocity(m.nv),
    dim=(d.nworld, m.ntendon),
    inputs=[d.qvel, d.ten_J],
    outputs=[d.ten_velocity],
    block_dim=m.block_dim.tendon_velocity,
  )

  smooth.com_vel(m, d)
  passive.passive(m, d)
  smooth.rne(m, d)
  smooth.tendon_bias(m, d, d.qfrc_bias)


@wp.kernel
def _actuator_force(
  # Model:
  na: int,
  opt_timestep: wp.array(dtype=float),
  actuator_dyntype: wp.array(dtype=int),
  actuator_gaintype: wp.array(dtype=int),
  actuator_biastype: wp.array(dtype=int),
  actuator_actadr: wp.array(dtype=int),
  actuator_actnum: wp.array(dtype=int),
  actuator_ctrllimited: wp.array(dtype=bool),
  actuator_forcelimited: wp.array(dtype=bool),
  actuator_actlimited: wp.array(dtype=bool),
  actuator_dynprm: wp.array2d(dtype=vec10f),
  actuator_gainprm: wp.array2d(dtype=vec10f),
  actuator_biasprm: wp.array2d(dtype=vec10f),
  actuator_actearly: wp.array(dtype=bool),
  actuator_ctrlrange: wp.array2d(dtype=wp.vec2),
  actuator_forcerange: wp.array2d(dtype=wp.vec2),
  actuator_actrange: wp.array2d(dtype=wp.vec2),
  actuator_acc0: wp.array(dtype=float),
  actuator_lengthrange: wp.array(dtype=wp.vec2),
  # Data in:
  act_in: wp.array2d(dtype=float),
  ctrl_in: wp.array2d(dtype=float),
  actuator_length_in: wp.array2d(dtype=float),
  actuator_velocity_in: wp.array2d(dtype=float),
  # In:
  dsbl_clampctrl: int,
  # Data out:
  act_dot_out: wp.array2d(dtype=float),
  actuator_force_out: wp.array2d(dtype=float),
):
  worldid, uid = wp.tid()

  actuator_ctrlrange_id = worldid % actuator_ctrlrange.shape[0]

  ctrl = ctrl_in[worldid, uid]

  if actuator_ctrllimited[uid] and not dsbl_clampctrl:
    ctrlrange = actuator_ctrlrange[actuator_ctrlrange_id, uid]
    ctrl = wp.clamp(ctrl, ctrlrange[0], ctrlrange[1])
  ctrl_act = ctrl

  act_first = actuator_actadr[uid]
  if na and act_first >= 0:
    act_last = act_first + actuator_actnum[uid] - 1
    dyntype = actuator_dyntype[uid]
    dynprm = actuator_dynprm[worldid % actuator_dynprm.shape[0], uid]

    if dyntype == DynType.INTEGRATOR:
      act_dot = ctrl
    elif dyntype == DynType.FILTER or dyntype == DynType.FILTEREXACT:
      act = act_in[worldid, act_last]
      act_dot = (ctrl - act) / wp.max(dynprm[0], MJ_MINVAL)
    elif dyntype == DynType.MUSCLE:
      dynprm = actuator_dynprm[worldid, uid]
      act = act_in[worldid, act_last]
      act_dot = util_misc.muscle_dynamics(ctrl, act, dynprm)
    else:  # DynType.NONE
      act_dot = 0.0

    act_dot_out[worldid, act_last] = act_dot

    if actuator_actearly[uid]:
      if dyntype == DynType.INTEGRATOR or dyntype == DynType.NONE:
        act = act_in[worldid, act_last]

      ctrl_act = _next_act(
        opt_timestep[worldid % opt_timestep.shape[0]],
        dyntype,
        dynprm,
        actuator_actrange[worldid % actuator_actrange.shape[0], uid],
        act,
        act_dot,
        1.0,
        actuator_actlimited[uid],
      )
    else:
      ctrl_act = act_in[worldid, act_last]

  length = actuator_length_in[worldid, uid]
  velocity = actuator_velocity_in[worldid, uid]

  # gain
  gaintype = actuator_gaintype[uid]
  gainprm = actuator_gainprm[worldid % actuator_gainprm.shape[0], uid]

  gain = 0.0
  if gaintype == GainType.FIXED:
    gain = gainprm[0]
  elif gaintype == GainType.AFFINE:
    gain = gainprm[0] + gainprm[1] * length + gainprm[2] * velocity
  elif gaintype == GainType.MUSCLE:
    acc0 = actuator_acc0[uid]
    lengthrange = actuator_lengthrange[uid]
    gain = util_misc.muscle_gain(length, velocity, lengthrange, acc0, gainprm)

  # bias
  biastype = actuator_biastype[uid]
  biasprm = actuator_biasprm[worldid % actuator_biasprm.shape[0], uid]

  bias = 0.0  # BiasType.NONE
  if biastype == BiasType.AFFINE:
    bias = biasprm[0] + biasprm[1] * length + biasprm[2] * velocity
  elif biastype == BiasType.MUSCLE:
    acc0 = actuator_acc0[uid]
    lengthrange = actuator_lengthrange[uid]
    bias = util_misc.muscle_bias(length, lengthrange, acc0, biasprm)

  force = gain * ctrl_act + bias

  if actuator_forcelimited[uid]:
    forcerange = actuator_forcerange[worldid % actuator_forcerange.shape[0], uid]
    force = wp.clamp(force, forcerange[0], forcerange[1])

  actuator_force_out[worldid, uid] = force


@wp.kernel
def _tendon_actuator_force(
  # Model:
  actuator_trntype: wp.array(dtype=int),
  actuator_trnid: wp.array(dtype=wp.vec2i),
  # Data in:
  actuator_force_in: wp.array2d(dtype=float),
  # Out:
  ten_actfrc_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()

  if actuator_trntype[actid] == TrnType.TENDON:
    tenid = actuator_trnid[actid][0]
    # TODO(team): only compute for tendons with force limits?
    wp.atomic_add(ten_actfrc_out[worldid], tenid, actuator_force_in[worldid, actid])


@wp.kernel
def _tendon_actuator_force_clamp(
  # Model:
  tendon_actfrclimited: wp.array(dtype=bool),
  tendon_actfrcrange: wp.array2d(dtype=wp.vec2),
  actuator_trntype: wp.array(dtype=int),
  actuator_trnid: wp.array(dtype=wp.vec2i),
  # In:
  ten_actfrc_in: wp.array2d(dtype=float),
  # Data out:
  actuator_force_out: wp.array2d(dtype=float),
):
  worldid, actid = wp.tid()

  if actuator_trntype[actid] == TrnType.TENDON:
    tenid = actuator_trnid[actid][0]
    if tendon_actfrclimited[tenid]:
      ten_actfrc = ten_actfrc_in[worldid, tenid]
      actfrcrange = tendon_actfrcrange[worldid % tendon_actfrcrange.shape[0], tenid]

      if ten_actfrc < actfrcrange[0]:
        actuator_force_out[worldid, actid] *= actfrcrange[0] / ten_actfrc
      elif ten_actfrc > actfrcrange[1]:
        actuator_force_out[worldid, actid] *= actfrcrange[1] / ten_actfrc


@wp.kernel
def _qfrc_actuator(
  # Model:
  nu: int,
  ngravcomp: int,
  jnt_actfrclimited: wp.array(dtype=bool),
  jnt_actgravcomp: wp.array(dtype=int),
  jnt_actfrcrange: wp.array2d(dtype=wp.vec2),
  dof_jntid: wp.array(dtype=int),
  # Data in:
  actuator_moment_in: wp.array3d(dtype=float),
  qfrc_gravcomp_in: wp.array2d(dtype=float),
  actuator_force_in: wp.array2d(dtype=float),
  # Data out:
  qfrc_actuator_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  qfrc = float(0.0)
  for uid in range(nu):
    qfrc += actuator_moment_in[worldid, uid, dofid] * actuator_force_in[worldid, uid]

  jntid = dof_jntid[dofid]

  # actuator-level gravity compensation, skip if added as passive force
  if ngravcomp and jnt_actgravcomp[jntid]:
    qfrc += qfrc_gravcomp_in[worldid, dofid]

  if jnt_actfrclimited[jntid]:
    frcrange = jnt_actfrcrange[worldid % jnt_actfrcrange.shape[0], jntid]
    qfrc = wp.clamp(qfrc, frcrange[0], frcrange[1])

  qfrc_actuator_out[worldid, dofid] = qfrc


@event_scope
def fwd_actuation(m: Model, d: Data):
  """Actuation-dependent computations."""
  if not m.nu or (m.opt.disableflags & DisableBit.ACTUATION):
    d.act_dot.zero_()
    d.qfrc_actuator.zero_()
    return

  wp.launch(
    _actuator_force,
    dim=(d.nworld, m.nu),
    inputs=[
      m.na,
      m.opt.timestep,
      m.actuator_dyntype,
      m.actuator_gaintype,
      m.actuator_biastype,
      m.actuator_actadr,
      m.actuator_actnum,
      m.actuator_ctrllimited,
      m.actuator_forcelimited,
      m.actuator_actlimited,
      m.actuator_dynprm,
      m.actuator_gainprm,
      m.actuator_biasprm,
      m.actuator_actearly,
      m.actuator_ctrlrange,
      m.actuator_forcerange,
      m.actuator_actrange,
      m.actuator_acc0,
      m.actuator_lengthrange,
      d.act,
      d.ctrl,
      d.actuator_length,
      d.actuator_velocity,
      m.opt.disableflags & DisableBit.CLAMPCTRL,
    ],
    outputs=[d.act_dot, d.actuator_force],
  )

  if m.ntendon:
    # total actuator force at tendon
    ten_actfrc = wp.zeros((d.nworld, m.ntendon), dtype=float)
    wp.launch(
      _tendon_actuator_force,
      dim=(d.nworld, m.nu),
      inputs=[m.actuator_trntype, m.actuator_trnid, d.actuator_force],
      outputs=[ten_actfrc],
    )

    wp.launch(
      _tendon_actuator_force_clamp,
      dim=(d.nworld, m.nu),
      inputs=[m.tendon_actfrclimited, m.tendon_actfrcrange, m.actuator_trntype, m.actuator_trnid, ten_actfrc],
      outputs=[d.actuator_force],
    )

  wp.launch(
    _qfrc_actuator,
    dim=(d.nworld, m.nv),
    inputs=[
      m.nu,
      m.ngravcomp,
      m.jnt_actfrclimited,
      m.jnt_actgravcomp,
      m.jnt_actfrcrange,
      m.dof_jntid,
      d.actuator_moment,
      d.qfrc_gravcomp,
      d.actuator_force,
    ],
    outputs=[d.qfrc_actuator],
  )


@wp.kernel
def _qfrc_smooth(
  # Data in:
  qfrc_applied_in: wp.array2d(dtype=float),
  qfrc_bias_in: wp.array2d(dtype=float),
  qfrc_passive_in: wp.array2d(dtype=float),
  qfrc_actuator_in: wp.array2d(dtype=float),
  # Data out:
  qfrc_smooth_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  qfrc_smooth_out[worldid, dofid] = (
    qfrc_passive_in[worldid, dofid]
    - qfrc_bias_in[worldid, dofid]
    + qfrc_actuator_in[worldid, dofid]
    + qfrc_applied_in[worldid, dofid]
  )


@event_scope
def fwd_acceleration(m: Model, d: Data, factorize: bool = False):
  """Add up all non-constraint forces, compute qacc_smooth.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    factorize: Flag to factorize inertia matrix.
  """
  wp.launch(
    _qfrc_smooth,
    dim=(d.nworld, m.nv),
    inputs=[d.qfrc_applied, d.qfrc_bias, d.qfrc_passive, d.qfrc_actuator],
    outputs=[d.qfrc_smooth],
  )
  xfrc_accumulate(m, d, d.qfrc_smooth)

  if factorize:
    smooth.factor_solve_i(m, d, d.qM, d.qLD, d.qLDiagInv, d.qacc_smooth, d.qfrc_smooth)
  else:
    smooth.solve_m(m, d, d.qacc_smooth, d.qfrc_smooth)


@event_scope
def forward(m: Model, d: Data):
  """Forward dynamics."""
  energy = m.opt.enableflags & EnableBit.ENERGY

  fwd_position(m, d, factorize=False)
  d.sensordata.zero_()
  sensor.sensor_pos(m, d)
  if energy:
    if m.sensor_e_potential == 0:  # not computed by sensor
      sensor.energy_pos(m, d)
  else:
    d.energy.zero_()

  fwd_velocity(m, d)
  sensor.sensor_vel(m, d)

  if energy:
    if m.sensor_e_kinetic == 0:  # not computed by sensor
      sensor.energy_vel(m, d)

  fwd_actuation(m, d)
  fwd_acceleration(m, d, factorize=True)

  solver.solve(m, d)
  sensor.sensor_acc(m, d)


@event_scope
def step(m: Model, d: Data):
  """Advance simulation."""
  # TODO(team): mj_checkPos
  # TODO(team): mj_checkVel
  forward(m, d)
  # TODO(team): mj_checkAcc

  if m.opt.integrator == IntegratorType.EULER:
    euler(m, d)
  elif m.opt.integrator == IntegratorType.RK4:
    rungekutta4(m, d)
  elif m.opt.integrator == IntegratorType.IMPLICITFAST:
    implicit(m, d)
  else:
    raise NotImplementedError(f"integrator {m.opt.integrator} not implemented.")


@event_scope
def step1(m: Model, d: Data):
  """Advance simulation in two phases: before input is set by user."""
  energy = m.opt.enableflags & EnableBit.ENERGY
  # TODO(team): mj_checkPos
  # TODO(team): mj_checkVel
  fwd_position(m, d)
  sensor.sensor_pos(m, d)

  if energy:
    if m.sensor_e_potential == 0:  # not computed by sensor
      sensor.energy_pos(m, d)
  else:
    d.energy.zero_()

  fwd_velocity(m, d)
  sensor.sensor_vel(m, d)

  if energy:
    if m.sensor_e_kinetic == 0:  # not computed by sensor
      sensor.energy_vel(m, d)


@event_scope
def step2(m: Model, d: Data):
  """Advance simulation in two phases: after input is set by user."""
  fwd_actuation(m, d)
  fwd_acceleration(m, d)
  solver.solve(m, d)
  sensor.sensor_acc(m, d)
  # TODO(team): mj_checkAcc

  # integrate with Euler or implicitfast
  # TODO(team): implicit
  if m.opt.integrator == IntegratorType.IMPLICITFAST:
    implicit(m, d)
  else:
    # note: RK4 defaults to Euler
    euler(m, d)
