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
from mujoco.mjx.third_party.mujoco_warp._src import history
from mujoco.mjx.third_party.mujoco_warp._src import island
from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import passive
from mujoco.mjx.third_party.mujoco_warp._src import sensor
from mujoco.mjx.third_party.mujoco_warp._src import sleep
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import solver
from mujoco.mjx.third_party.mujoco_warp._src import util_misc
from mujoco.mjx.third_party.mujoco_warp._src.support import next_act
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
from mujoco.mjx.third_party.mujoco_warp._src.types import OverflowType
from mujoco.mjx.third_party.mujoco_warp._src.types import TrnType
from mujoco.mjx.third_party.mujoco_warp._src.types import mat66
from mujoco.mjx.third_party.mujoco_warp._src.types import vec6
from mujoco.mjx.third_party.mujoco_warp._src.types import vec10
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _next_position(
  # Model:
  opt_timestep: wp.array[float],
  jnt_type: wp.array[int],
  jnt_qposadr: wp.array[int],
  jnt_dofadr: wp.array[int],
  # Data in:
  qpos_in: wp.array2d[float],
  qvel_in: wp.array2d[float],
  # In:
  qvel_scale_in: float,
  # Data out:
  qpos_out: wp.array2d[float],
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
  opt_timestep: wp.array[float],
  # Data in:
  qvel_in: wp.array2d[float],
  qacc_in: wp.array2d[float],
  # In:
  qacc_scale_in: float,
  # Data out:
  qvel_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]
  qvel_out[worldid, dofid] = qvel_in[worldid, dofid] + qacc_scale_in * qacc_in[worldid, dofid] * timestep


@wp.kernel
def _next_activation(
  # Model:
  opt_timestep: wp.array[float],
  actuator_dyntype: wp.array[int],
  actuator_actadr: wp.array[int],
  actuator_actnum: wp.array[int],
  actuator_dynprm: wp.array2d[vec10],
  actuator_gainprm: wp.array2d[vec10],
  actuator_biasprm: wp.array2d[vec10],
  actuator_actlimited: wp.array[bool],
  actuator_actrange: wp.array2d[wp.vec2],
  # Data in:
  act_in: wp.array2d[float],
  act_dot_in: wp.array2d[float],
  actuator_velocity_in: wp.array2d[float],
  # In:
  act_dot_scale: float,
  limit: bool,
  # Data out:
  act_out: wp.array2d[float],
):
  worldid, uid = wp.tid()
  opt_timestep_id = worldid % opt_timestep.shape[0]
  actuator_dynprm_id = worldid % actuator_dynprm.shape[0]
  actuator_actrange_id = worldid % actuator_actrange.shape[0]
  actuator_gainprm_id = worldid % actuator_gainprm.shape[0]
  actuator_biasprm_id = worldid % actuator_biasprm.shape[0]

  actadr = actuator_actadr[uid]
  actnum = actuator_actnum[uid]
  dyntype = actuator_dyntype[uid]

  if dyntype == DynType.DCMOTOR:
    dynprm = actuator_dynprm[actuator_dynprm_id, uid]
    gainprm = actuator_gainprm[actuator_gainprm_id, uid]
    biasprm = actuator_biasprm[actuator_biasprm_id, uid]
    slots = util_misc.dcmotor_slots(dynprm, gainprm)

    for j in range(actadr, actadr + actnum):
      offset = j - actadr
      act = act_in[worldid, j]
      act_dot = act_dot_in[worldid, j]

      if offset == slots[4]:  # current
        R = gainprm[0]
        te = wp.max(MJ_MINVAL, dynprm[0])
        act = act + act_dot * te * (1.0 - wp.exp(-opt_timestep[opt_timestep_id] / te))
      elif offset == slots[3]:  # bristle
        F_C = biasprm[3]
        F_S = biasprm[4]
        v_S = biasprm[5]
        sigma0 = dynprm[5]
        velocity = actuator_velocity_in[worldid, uid]
        g = util_misc.lugre_stribeck(velocity, F_C, F_S, v_S)

        a = -sigma0 * wp.abs(velocity) / wp.max(MJ_MINVAL, g)
        h = opt_timestep[opt_timestep_id]
        exp_ah = wp.exp(a * h)
        int_h = h
        if wp.abs(a) > MJ_MINVAL:
          int_h = (exp_ah - 1.0) / a
        act = exp_ah * act + int_h * velocity
      elif offset == slots[1]:  # integral
        act = act + act_dot * opt_timestep[opt_timestep_id]
        Imax = dynprm[8]
        if Imax > 0.0:
          act = wp.clamp(act, -Imax, Imax)
      else:  # temperature and slew
        act = act + act_dot * opt_timestep[opt_timestep_id]

      act_out[worldid, j] = act
  else:
    for j in range(actadr, actadr + actnum):
      act = next_act(
        opt_timestep[opt_timestep_id],
        dyntype,
        actuator_dynprm[actuator_dynprm_id, uid],
        actuator_actrange[actuator_actrange_id, uid],
        act_in[worldid, j],
        act_dot_in[worldid, j],
        act_dot_scale,
        limit and actuator_actlimited[uid],
      )
      act_out[worldid, j] = act


@cache_kernel
def _next_time_builder(warn_overflow: int):
  @wp.kernel(module="unique", enable_backward=False)
  def _next_time(
    # Model:
    opt_timestep: wp.array[float],
    is_sparse: bool,
    # Data in:
    nefc_in: wp.array[int],
    time_in: wp.array[float],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    nworld_in: int,
    naconmax_in: int,
    njmax_in: int,
    njmax_nnz_in: int,
    nacon_in: wp.array[int],
    ncollision_in: wp.array[int],
    # Data out:
    time_out: wp.array[float],
    overflow_out: wp.array[int],
  ):
    worldid = wp.tid()
    time_out[worldid] = time_in[worldid] + opt_timestep[worldid % opt_timestep.shape[0]]
    nefc = nefc_in[worldid]

    if nefc > njmax_in:
      if wp.static(bool(warn_overflow & OverflowType.NEFC)):
        wp.printf(
          "nefc overflow - please increase njmax beyond %u\n"
          "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.NEFC (or = 0 for all)\n",
          njmax_in,
        )
      overflow_out[worldid] = overflow_out[worldid] | OverflowType.NEFC
    elif nefc > 0 and is_sparse:
      efcid = wp.min(nefc, njmax_in) - 1
      efc_nnz = efc_J_rowadr_in[worldid, efcid] + efc_J_rownnz_in[worldid, efcid]
      if efc_nnz > njmax_nnz_in:
        if wp.static(bool(warn_overflow & OverflowType.NJMAX_NNZ)):
          wp.printf(
            "njmax_nnz overflow - please increase njmax_nnz beyond %u\n"
            "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.NJMAX_NNZ (or = 0 for all)\n",
            njmax_nnz_in,
          )
        overflow_out[worldid] = overflow_out[worldid] | OverflowType.NJMAX_NNZ

    ncollision = ncollision_in[0]
    if ncollision > naconmax_in:
      if worldid == 0 and wp.static(bool(warn_overflow & OverflowType.BROADPHASE)):
        wp.printf(
          "broadphase overflow - please increase nconmax beyond %u or naconmax beyond %u\n"
          "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.BROADPHASE (or = 0 for all)\n",
          naconmax_in // nworld_in,
          naconmax_in,
        )
      overflow_out[worldid] = overflow_out[worldid] | OverflowType.BROADPHASE

    nacon = nacon_in[0]
    if nacon > naconmax_in:
      if worldid == 0 and wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)):
        wp.printf(
          "narrowphase overflow - please increase nconmax beyond %u or naconmax beyond %u\n"
          "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.NARROWPHASE (or = 0 for all)\n",
          naconmax_in // nworld_in,
          naconmax_in,
        )
      overflow_out[worldid] = overflow_out[worldid] | OverflowType.NARROWPHASE

  return _next_time


def _advance(m: Model, d: Data, qacc: wp.array, qvel: Optional[wp.array] = None):
  """Advance state and time given activation derivatives and acceleration."""
  # TODO(team): can we assume static timesteps?

  # advance activations
  wp.launch(
    _next_activation,
    dim=(d.nworld, m.nu),
    inputs=[
      m.opt.timestep,
      m.actuator_dyntype,
      m.actuator_actadr,
      m.actuator_actnum,
      m.actuator_dynprm,
      m.actuator_gainprm,
      m.actuator_biasprm,
      m.actuator_actlimited,
      m.actuator_actrange,
      d.act,
      d.act_dot,
      d.actuator_velocity,
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

  # advance history buffers before time advance
  history.insert_ctrl_history(m, d)

  wp.launch(
    _next_time_builder(int(m.opt.warn_overflow)),
    dim=d.nworld,
    inputs=[
      m.opt.timestep,
      m.is_sparse,
      d.nefc,
      d.time,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.nworld,
      d.naconmax,
      d.njmax,
      d.njmax_nnz,
      d.nacon,
      d.ncollision,
    ],
    outputs=[d.time, d.overflow],
  )

  wp.copy(d.qacc_warmstart, d.qacc)

  sleep_enabled = bool(m.opt.enableflags & EnableBit.SLEEP) and not bool(m.opt.disableflags & DisableBit.ISLAND)
  if sleep_enabled:
    sleep.sleep(m, d)
    fwd_velocity(m, d)
    sleep.update_sleep(m, d)


@wp.kernel
def _compute_damping_deriv(
  # Model:
  dof_damping: wp.array2d[float],
  dof_dampingpoly: wp.array2d[wp.vec2],
  # Data in:
  qvel_in: wp.array2d[float],
  # Out:
  deriv_out: wp.array2d[float],
):
  worldid, tid = wp.tid()
  damping = dof_damping[worldid % dof_damping.shape[0], tid]
  dpoly = dof_dampingpoly[worldid % dof_dampingpoly.shape[0], tid]
  v = qvel_in[worldid, tid]
  deriv_out[worldid, tid] = util_misc._poly_force_deriv(damping, dpoly, v, 1)


@wp.kernel
def _euler_damp_qfrc(
  # Model:
  opt_timestep: wp.array[float],
  M_rownnz: wp.array[int],
  M_rowadr: wp.array[int],
  # In:
  damp_deriv: wp.array2d[float],
  # Out:
  M_integration_out: wp.array2d[float],
):
  worldid, tid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  adr = M_rowadr[tid] + M_rownnz[tid] - 1
  M_integration_out[worldid, adr] += timestep * damp_deriv[worldid, tid]


@event_scope
def euler(m: Model, d: Data):
  """Euler integrator, semi-implicit in velocity."""
  # integrate damping implicitly
  if not (m.opt.disableflags & (DisableBit.EULERDAMP | DisableBit.DAMPER)):
    qacc = wp.empty((d.nworld, m.nv), dtype=float)

    # Compute damping derivative
    damp_deriv = wp.empty((d.nworld, m.nv), dtype=float)
    wp.launch(
      _compute_damping_deriv,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_damping, m.dof_dampingpoly, d.qvel],
      outputs=[damp_deriv],
    )

    # Clone M, add the damping to the diagonal, and factor-solve. factor_solve_i factors each block
    # per-block (packed dense and/or sparse LDL); the scratch qLD matches d.qLD.
    M = wp.clone(d.M)
    qLD = wp.empty_like(d.qLD)
    qLDiagInv = wp.empty((d.nworld, m.nv), dtype=float)
    wp.launch(
      _euler_damp_qfrc,
      dim=(d.nworld, m.nv),
      inputs=[m.opt.timestep, m.M_rownnz, m.M_rowadr, damp_deriv],
      outputs=[M],
    )
    smooth.factor_solve_i(m, d, M, qLD, qLDiagInv, qacc, d.efc.Ma)
    _advance(m, d, qacc)
  else:
    _advance(m, d, d.qacc)


def _rk_perturb_state(
  m: Model,
  d: Data,
  scale: float,
  qpos_t0: wp.array2d[float],
  qvel_t0: wp.array2d[float],
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
      dim=(d.nworld, m.nu),
      inputs=[
        m.opt.timestep,
        m.actuator_dyntype,
        m.actuator_actadr,
        m.actuator_actnum,
        m.actuator_dynprm,
        m.actuator_gainprm,
        m.actuator_biasprm,
        m.actuator_actlimited,
        m.actuator_actrange,
        act_t0,
        d.act_dot,
        d.actuator_velocity,
        scale,
        False,
      ],
      outputs=[d.act],
    )


@wp.kernel
def _rk_accumulate_velocity_acceleration(
  # Data in:
  qvel_in: wp.array2d[float],
  qacc_in: wp.array2d[float],
  # In:
  scale: float,
  # Data out:
  qvel_out: wp.array2d[float],
  qacc_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  qvel_out[worldid, dofid] += scale * qvel_in[worldid, dofid]
  qacc_out[worldid, dofid] += scale * qacc_in[worldid, dofid]


@wp.kernel
def _rk_accumulate_activation_velocity(
  # Data in:
  act_dot_in: wp.array2d[float],
  # In:
  scale: float,
  # Data out:
  act_dot_out: wp.array2d[float],
):
  worldid, actid = wp.tid()
  act_dot_out[worldid, actid] += scale * act_dot_in[worldid, actid]


def _rk_accumulate(
  m: Model,
  d: Data,
  scale: float,
  qvel_rk: wp.array2d[float],
  qacc_rk: wp.array2d[float],
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


@wp.kernel
def _map_m2d(
  # Model:
  mapM2D: wp.array[int],
  # In:
  qH_M: wp.array2d[float],
  # Data out:
  qLU_out: wp.array2d[float],
):
  # Scatter qH_M (M-structure) into the D-structure qLU via mapM2D.
  worldid, elemid = wp.tid()
  m_idx = mapM2D[elemid]
  if m_idx >= 0:
    qLU_out[worldid, elemid] = qH_M[worldid, m_idx]
  else:
    qLU_out[worldid, elemid] = 0.0


@wp.kernel
def _implicit_free_body_reset_m(
  # Model:
  body_dofadr: wp.array[int],
  M_rownnz: wp.array[int],
  M_rowadr: wp.array[int],
  body_freeadr: wp.array[int],
  # Data in:
  M_in: wp.array2d[float],
  # Out:
  qH_out: wp.array2d[float],
):
  worldid, freeid = wp.tid()
  bodyid = body_freeadr[freeid]

  dof_adr = body_dofadr[bodyid]
  for r in range(6):
    row = dof_adr + r
    start = M_rowadr[row]
    nnz = M_rownnz[row]
    for k in range(nnz):
      qH_out[worldid, start + k] = M_in[worldid, start + k]


@wp.func
def _project_spatial_B(
  # Data in:
  cdof_in: wp.array2d[wp.spatial_vector],
  # In:
  worldid: int,
  dof_adr: int,
  geom_rotT: wp.mat33,
  offset: wp.vec3,
  B00: wp.mat33,
  B01: wp.mat33,
  B10: wp.mat33,
  B11: wp.mat33,
) -> mat66:
  c0 = cdof_in[worldid, dof_adr + 0]
  c1 = cdof_in[worldid, dof_adr + 1]
  c2 = cdof_in[worldid, dof_adr + 2]
  c3 = cdof_in[worldid, dof_adr + 3]
  c4 = cdof_in[worldid, dof_adr + 4]
  c5 = cdof_in[worldid, dof_adr + 5]

  ll0 = geom_rotT @ wp.vec3(c0[3], c0[4], c0[5])
  ll1 = geom_rotT @ wp.vec3(c1[3], c1[4], c1[5])
  ll2 = geom_rotT @ wp.vec3(c2[3], c2[4], c2[5])

  a3 = wp.vec3(c3[0], c3[1], c3[2])
  a4 = wp.vec3(c4[0], c4[1], c4[2])
  a5 = wp.vec3(c5[0], c5[1], c5[2])

  p3 = wp.vec3(c3[3], c3[4], c3[5]) + wp.cross(a3, offset)
  p4 = wp.vec3(c4[3], c4[4], c4[5]) + wp.cross(a4, offset)
  p5 = wp.vec3(c5[3], c5[4], c5[5]) + wp.cross(a5, offset)

  la3 = geom_rotT @ a3
  la4 = geom_rotT @ a4
  la5 = geom_rotT @ a5

  ll3 = geom_rotT @ p3
  ll4 = geom_rotT @ p4
  ll5 = geom_rotT @ p5

  ll_trans = wp.mat33(
    ll0[0],
    ll1[0],
    ll2[0],
    ll0[1],
    ll1[1],
    ll2[1],
    ll0[2],
    ll1[2],
    ll2[2],
  )
  la_rot = wp.mat33(
    la3[0],
    la4[0],
    la5[0],
    la3[1],
    la4[1],
    la5[1],
    la3[2],
    la4[2],
    la5[2],
  )
  ll_rot = wp.mat33(
    ll3[0],
    ll4[0],
    ll5[0],
    ll3[1],
    ll4[1],
    ll5[1],
    ll3[2],
    ll4[2],
    ll5[2],
  )

  ll_trans_T = wp.transpose(ll_trans)
  la_rot_T = wp.transpose(la_rot)
  ll_rot_T = wp.transpose(ll_rot)

  T_lin = B11 @ ll_trans
  T_rot = B10 @ la_rot + B11 @ ll_rot

  top_left = ll_trans_T @ T_lin
  top_right = ll_trans_T @ T_rot

  bot_left = la_rot_T @ (B01 @ ll_trans) + ll_rot_T @ T_lin
  bot_right = la_rot_T @ (B00 @ la_rot + B01 @ ll_rot) + ll_rot_T @ T_rot

  out = mat66(0.0)
  for r in range(3):
    for c in range(3):
      out[r, c] = top_left[r, c]
      out[r, 3 + c] = top_right[r, c]
      out[3 + r, c] = bot_left[r, c]
      out[3 + r, 3 + c] = bot_right[r, c]
  return out


@wp.kernel
def _implicit_free_body_solve(
  # Model:
  opt_timestep: wp.array[float],
  opt_wind: wp.array[wp.vec3],
  opt_density: wp.array[float],
  opt_viscosity: wp.array[float],
  opt_integrator: int,
  opt_disableflags: int,
  opt_enableflags: int,
  body_dofadr: wp.array[int],
  body_geomnum: wp.array[int],
  body_geomadr: wp.array[int],
  body_mass: wp.array2d[float],
  body_inertia: wp.array2d[wp.vec3],
  dof_treeid: wp.array[int],
  dof_damping: wp.array2d[float],
  dof_dampingpoly: wp.array2d[wp.vec2],
  geom_type: wp.array[int],
  geom_size: wp.array2d[wp.vec3],
  geom_fluid: wp.array2d[float],
  M_rownnz: wp.array[int],
  M_rowadr: wp.array[int],
  M_colind: wp.array[int],
  body_fluid_ellipsoid: wp.array[bool],
  body_freeadr: wp.array[int],
  # Data in:
  qvel_in: wp.array2d[float],
  xpos_in: wp.array2d[wp.vec3],
  xmat_in: wp.array2d[wp.mat33],
  xipos_in: wp.array2d[wp.vec3],
  ximat_in: wp.array2d[wp.mat33],
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  M_in: wp.array2d[float],
  tree_awake_in: wp.array2d[int],
  cvel_in: wp.array2d[wp.spatial_vector],
  # In:
  qfrc_in: wp.array2d[float],
  # Data out:
  qacc_out: wp.array2d[float],
):
  worldid, freeid = wp.tid()
  bodyid = body_freeadr[freeid]

  dof_adr = body_dofadr[bodyid]

  # Sleep guard matching mjd_freeMhat: skip if sleeping
  treeid = dof_treeid[dof_adr]
  if (opt_enableflags & EnableBit.SLEEP) and (tree_awake_in[worldid, treeid] == 0):
    return

  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  # 1. A = M block (gather from sparse lower triangle)
  A = mat66(0.0)
  for r in range(6):
    row = dof_adr + r
    start = M_rowadr[row]
    nnz = M_rownnz[row]
    for k in range(nnz):
      c = M_colind[start + k] - dof_adr
      val = M_in[worldid, start + k]
      A[r, c] = val
      A[c, r] = val

  # 2. Add joint damping (with damper disable check and polynomial damping)
  if not (opt_disableflags & DisableBit.DAMPER):
    for r in range(6):
      dof = dof_adr + r
      damp = dof_damping[worldid % dof_damping.shape[0], dof]
      dpoly = dof_dampingpoly[worldid % dof_dampingpoly.shape[0], dof]
      v = qvel_in[worldid, dof]
      A[r, r] += timestep * util_misc._poly_force_deriv(damp, dpoly, v, 1)

  # 3. Add gyroscopic bias velocity derivative
  mass = body_mass[worldid % body_mass.shape[0], bodyid]
  R = xmat_in[worldid, bodyid]
  Xi = ximat_in[worldid, bodyid]
  inertia = body_inertia[worldid % body_inertia.shape[0], bodyid]
  s = xipos_in[worldid, bodyid] - xpos_in[worldid, bodyid]
  qvel_rot = wp.vec3(
    qvel_in[worldid, dof_adr + 3],
    qvel_in[worldid, dof_adr + 4],
    qvel_in[worldid, dof_adr + 5],
  )
  lin, rot = math.free_bias_vel_blocks(mass, R, Xi, inertia, s, qvel_rot)
  h_mass = -timestep * mass
  for r in range(3):
    for c in range(3):
      A[r, 3 + c] += h_mass * lin[r, c]
      A[3 + r, 3 + c] += timestep * rot[r, c]

  # 4. Fluid force derivatives
  density = opt_density[worldid % opt_density.shape[0]]
  viscosity = opt_viscosity[worldid % opt_viscosity.shape[0]]
  wind = opt_wind[worldid % opt_wind.shape[0]]

  if density > 0.0 or viscosity > 0.0:
    if body_fluid_ellipsoid[bodyid]:
      subtree_root = subtree_com_in[worldid, bodyid]
      xipos = xipos_in[worldid, bodyid]
      cvel = cvel_in[worldid, bodyid]
      ang_global = wp.spatial_top(cvel)
      lin_global = wp.spatial_bottom(cvel)
      lin_com = lin_global - wp.cross(xipos - subtree_root, ang_global)

      geomadr = body_geomadr[bodyid]
      geomnum = body_geomnum[bodyid]

      for g in range(geomnum):
        geomid = geomadr + g
        coef = geom_fluid[geomid, 0]
        if coef <= 0.0:
          continue

        size = geom_size[worldid % geom_size.shape[0], geomid]
        semiaxes = passive.geom_semiaxes(size, geom_type[geomid])
        geom_rot = geom_xmat_in[worldid, geomid]
        geom_rotT = wp.transpose(geom_rot)
        geom_pos = geom_xpos_in[worldid, geomid]

        # compute local velocity
        lin_point = lin_com + wp.cross(ang_global, geom_pos - xipos)
        l_ang = geom_rotT @ ang_global
        l_lin = geom_rotT @ lin_point

        if wind[0] != 0.0 or wind[1] != 0.0 or wind[2] != 0.0:
          l_lin -= geom_rotT @ wind

        ang_vel = l_ang
        lin_vel = l_lin

        blunt_drag_coef = geom_fluid[geomid, 1]
        slender_drag_coef = geom_fluid[geomid, 2]
        ang_drag_coef = geom_fluid[geomid, 3]
        kutta_lift_coef = geom_fluid[geomid, 4]
        magnus_lift_coef = geom_fluid[geomid, 5]
        virtual_mass = wp.vec3(geom_fluid[geomid, 6], geom_fluid[geomid, 7], geom_fluid[geomid, 8])
        virtual_inertia = wp.vec3(geom_fluid[geomid, 9], geom_fluid[geomid, 10], geom_fluid[geomid, 11])

        # Compute 6x6 spatial B matrix once per geom (unsymmetrized for free body)
        B00, B01, B10, B11 = derivative._geom_ellipsoid_fluid_B(
          semiaxes,
          blunt_drag_coef,
          slender_drag_coef,
          ang_drag_coef,
          kutta_lift_coef,
          magnus_lift_coef,
          virtual_mass,
          virtual_inertia,
          ang_vel,
          lin_vel,
          density,
          viscosity,
        )

        # 3x3 block projection of J^T @ B @ J
        offset = geom_pos - subtree_root
        J_T_B_J = _project_spatial_B(cdof_in, worldid, dof_adr, geom_rotT, offset, B00, B01, B10, B11)
        for r in range(6):
          for c in range(6):
            A[r, c] -= timestep * J_T_B_J[r, c]

    elif mass >= MJ_MINVAL:
      b_ipos = xipos_in[worldid, bodyid]
      b_imat = ximat_in[worldid, bodyid]
      subtree_root = subtree_com_in[worldid, bodyid]

      vel_subtree = cvel_in[worldid, bodyid]
      v_subtree_ang = wp.vec3(vel_subtree[0], vel_subtree[1], vel_subtree[2])
      v_subtree_lin = wp.vec3(vel_subtree[3], vel_subtree[4], vel_subtree[5])

      lin_com = v_subtree_lin - wp.cross(b_ipos - subtree_root, v_subtree_ang)
      b_imat_T = wp.transpose(b_imat)
      v_local_ang = b_imat_T @ v_subtree_ang
      v_local_lin = b_imat_T @ (lin_com - wind)

      lvel = wp.spatial_vector(
        v_local_ang[0],
        v_local_ang[1],
        v_local_ang[2],
        v_local_lin[0],
        v_local_lin[1],
        v_local_lin[2],
      )

      B_box = derivative._deriv_box_fluid(
        opt_integrator,
        body_mass,
        body_inertia,
        worldid,
        bodyid,
        lvel,
        density,
        viscosity,
      )
      B00 = wp.diag(wp.vec3(B_box[0, 0], B_box[1, 1], B_box[2, 2]))
      B11 = wp.diag(wp.vec3(B_box[3, 3], B_box[4, 4], B_box[5, 5]))
      zero33 = wp.mat33(0.0)
      offset_box = b_ipos - subtree_root
      J_T_B_J = _project_spatial_B(cdof_in, worldid, dof_adr, b_imat_T, offset_box, B00, zero33, zero33, B11)
      for r in range(6):
        for c in range(6):
          A[r, c] -= timestep * J_T_B_J[r, c]

  # 5. Solve A * x = qfrc
  A_fact, pivot, ok = math.lu_factor_6x6(A)
  if ok:
    b_vec = vec6(0.0)
    for r in range(6):
      b_vec[r] = qfrc_in[worldid, dof_adr + r]
    x = math.lu_solve_6x6(A_fact, pivot, b_vec)
    for r in range(6):
      qacc_out[worldid, dof_adr + r] = x[r]


def _launch_implicit_free_body_solve(m: Model, d: Data, qacc: wp.array2d[float]):
  if m.body_freeadr.size == 0:
    return
  wp.launch(
    _implicit_free_body_solve,
    dim=(d.nworld, m.body_freeadr.size),
    inputs=[
      m.opt.timestep,
      m.opt.wind,
      m.opt.density,
      m.opt.viscosity,
      m.opt.integrator,
      m.opt.disableflags,
      m.opt.enableflags,
      m.body_dofadr,
      m.body_geomnum,
      m.body_geomadr,
      m.body_mass,
      m.body_inertia,
      m.dof_treeid,
      m.dof_damping,
      m.dof_dampingpoly,
      m.geom_type,
      m.geom_size,
      m.geom_fluid,
      m.M_rownnz,
      m.M_rowadr,
      m.M_colind,
      m.body_fluid_ellipsoid,
      m.body_freeadr,
      d.qvel,
      d.xpos,
      d.xmat,
      d.xipos,
      d.ximat,
      d.geom_xpos,
      d.geom_xmat,
      d.subtree_com,
      d.cdof,
      d.M,
      d.tree_awake,
      d.cvel,
      d.efc.Ma,
    ],
    outputs=[qacc],
  )


@event_scope
def implicit(m: Model, d: Data):
  """Integrates fully implicit in velocity."""
  if m.opt.integrator == IntegratorType.IMPLICIT:
    # 1. Smooth velocity derivatives into M-structure
    qH_M = wp.empty((d.nworld, m.nC), dtype=float)
    derivative.deriv_smooth_vel(m, d, qH_M)

    # 2. Map qH_M (M-structure) to qLU (D-structure) via mapM2D.
    wp.launch(
      _map_m2d,
      dim=(d.nworld, m.nD),
      inputs=[m.mapM2D, qH_M],
      outputs=[d.qLU],
    )

    # 3. Compute RNE derivatives, scale by timestep, and subtract in-place from qLU
    derivative.deriv_rne_vel(m, d, d.qLU, flg_subtract=True)

    # 4. Factorize and solve: qacc = qLU \ Ma
    qacc = wp.empty((d.nworld, m.nv), dtype=float)
    smooth.factor_solve_lu(m, d, d.qLU, qacc, d.efc.Ma)
    _launch_implicit_free_body_solve(m, d, qacc)
    _advance(m, d, qacc)
  elif ~(m.opt.disableflags | ~(DisableBit.ACTUATION | DisableBit.SPRING | DisableBit.DAMPER)):
    # qDeriv is in M-structure; the scratch qLD matches d.qLD (per-block).
    qDeriv = wp.empty((d.nworld, m.nC), dtype=float)
    qLD = wp.empty_like(d.qLD)
    qLDiagInv = wp.empty((d.nworld, m.nv), dtype=float)
    derivative.deriv_smooth_vel(m, d, qDeriv)
    if m.body_freeadr.size > 0:
      wp.launch(
        _implicit_free_body_reset_m,
        dim=(d.nworld, m.body_freeadr.size),
        inputs=[
          m.body_dofadr,
          m.M_rownnz,
          m.M_rowadr,
          m.body_freeadr,
          d.M,
        ],
        outputs=[qDeriv],
      )
    qacc = wp.empty((d.nworld, m.nv), dtype=float)
    smooth.factor_solve_i(m, d, qDeriv, qLD, qLDiagInv, qacc, d.efc.Ma)
    _launch_implicit_free_body_solve(m, d, qacc)
    _advance(m, d, qacc)
  else:
    _advance(m, d, d.qacc)


@event_scope
def fwd_kinematics(m: Model, d: Data):
  """Kinematics-dependent computations.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
  """
  smooth.kinematics(m, d)
  smooth.com_pos(m, d)
  smooth.camlight(m, d)
  smooth.flex(m, d)
  smooth.tendon(m, d)

  sleep_enabled = bool(m.opt.enableflags & EnableBit.SLEEP) and not bool(m.opt.disableflags & DisableBit.ISLAND)
  if sleep_enabled and m.ntendon > 0:
    sleep.wake_tendon(m, d)
    sleep.update_sleep_trees(m, d)


@event_scope
def fwd_position(m: Model, d: Data, factorize: bool = True):
  """Position-dependent computations.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    factorize: Flag to factorize inertia matrix.
  """
  fwd_kinematics(m, d)

  sleep_enabled = bool(m.opt.enableflags & EnableBit.SLEEP) and not bool(m.opt.disableflags & DisableBit.ISLAND)

  smooth.crb(m, d)
  smooth.tendon_armature(m, d)
  if factorize:
    smooth.factor_m(m, d)
  if m.opt.run_collision_detection:
    if sleep_enabled:
      # pass 1
      collision_driver.collision(m, d)
      # wake any sleeping tree touched by an awake one
      sleep.wake_collision(m, d)
      # snapshot the awake state pass 1 used, before update_sleep overwrites it. a body is "newly
      # awakened" if it was asleep here but awake after update_sleep below.
      awake_prev = wp.clone(d.body_awake)
      sleep.update_sleep(m, d)
      # pass 2: passing awake_prev runs the incremental pass, emitting only pairs involving a
      # newly-awakened body and appending them to the pass-1 buffer.
      collision_driver.collision(m, d, awake_prev=awake_prev)
    else:
      collision_driver.collision(m, d)

  constraint.make_constraint(m, d)

  if sleep_enabled:
    if m.neq > 0:
      sleep.wake_equality(m, d)
    sleep.update_sleep(m, d)

  if sleep_enabled:
    island.island(m, d)
  smooth.transmission(m, d)


@wp.kernel
def _actuator_velocity(
  # Data in:
  qvel_in: wp.array2d[float],
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  # Data out:
  actuator_velocity_out: wp.array2d[float],
):
  worldid, actid = wp.tid()

  rownnz = moment_rownnz_in[worldid, actid]
  rowadr = moment_rowadr_in[worldid, actid]

  vel = float(0.0)
  for i in range(rownnz):
    sparseid = rowadr + i
    colind = moment_colind_in[worldid, sparseid]
    vel += actuator_moment_in[worldid, sparseid] * qvel_in[worldid, colind]

  actuator_velocity_out[worldid, actid] = vel


@wp.kernel
def _tendon_velocity(
  # Model:
  ten_J_rownnz: wp.array[int],
  ten_J_rowadr: wp.array[int],
  ten_J_colind: wp.array[int],
  # Data in:
  qvel_in: wp.array2d[float],
  ten_J_in: wp.array2d[float],
  # Data out:
  ten_velocity_out: wp.array2d[float],
):
  worldid, tenid = wp.tid()

  velocity = float(0.0)
  rownnz = ten_J_rownnz[tenid]
  rowadr = ten_J_rowadr[tenid]
  for i in range(rownnz):
    sparseid = rowadr + i
    J = ten_J_in[worldid, sparseid]
    if J != 0.0:
      colind = ten_J_colind[sparseid]
      velocity += J * qvel_in[worldid, colind]

  ten_velocity_out[worldid, tenid] = velocity


@event_scope
def fwd_velocity(m: Model, d: Data):
  """Velocity-dependent computations."""
  wp.launch(
    _actuator_velocity,
    dim=(d.nworld, m.nactuator),
    inputs=[d.qvel, d.moment_rownnz, d.moment_rowadr, d.moment_colind, d.actuator_moment],
    outputs=[d.actuator_velocity],
    block_dim=m.block_dim.actuator_velocity,
  )

  wp.launch(
    _tendon_velocity,
    dim=(d.nworld, m.ntendon),
    inputs=[m.ten_J_rownnz, m.ten_J_rowadr, m.ten_J_colind, d.qvel, d.ten_J],
    outputs=[d.ten_velocity],
  )

  smooth.com_vel(m, d)
  passive.passive(m, d)
  smooth.rne(m, d)
  smooth.tendon_bias(m, d, d.qfrc_bias)


@wp.kernel
def _actuator_force(
  # Model:
  na: int,
  opt_timestep: wp.array[float],
  actuator_dyntype: wp.array[int],
  actuator_gaintype: wp.array[int],
  actuator_biastype: wp.array[int],
  actuator_ctrladr: wp.array[int],
  actuator_ctrlnum: wp.array[int],
  actuator_ctrlspec: wp.array[int],
  actuator_actadr: wp.array[int],
  actuator_actnum: wp.array[int],
  actuator_dynprm: wp.array2d[vec10],
  actuator_gainprm: wp.array2d[vec10],
  actuator_biasprm: wp.array2d[vec10],
  actuator_actlimited: wp.array[bool],
  actuator_actrange: wp.array2d[wp.vec2],
  actuator_actearly: wp.array[bool],
  actuator_forcelimited: wp.array[bool],
  actuator_forcerange: wp.array2d[wp.vec2],
  actuator_ctrllimited: wp.array[bool],
  actuator_ctrlrange: wp.array2d[wp.vec2],
  actuator_acc0: wp.array2d[float],
  actuator_lengthrange: wp.array2d[wp.vec2],
  # Data in:
  act_in: wp.array2d[float],
  ctrl_in: wp.array2d[float],
  actuator_length_in: wp.array2d[float],
  actuator_velocity_in: wp.array2d[float],
  # In:
  dsbl_clampctrl: int,
  # Data out:
  act_dot_out: wp.array2d[float],
  actuator_force_out: wp.array2d[float],
):
  worldid, uid = wp.tid()

  actuator_ctrlrange_id = worldid % actuator_ctrlrange.shape[0]

  uadr = actuator_ctrladr[uid]
  ctrlnum = actuator_ctrlnum[uid]
  ctrl = 0.0
  if ctrlnum > 0:
    ctrl = ctrl_in[worldid, uadr]
    if actuator_ctrllimited[uadr] and not dsbl_clampctrl:
      ctrlrange = actuator_ctrlrange[actuator_ctrlrange_id, uadr]
      ctrl = wp.clamp(ctrl, ctrlrange[0], ctrlrange[1])
  ctrl_act = ctrl
  u_first = ctrl

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
      dynprm = actuator_dynprm[worldid % actuator_dynprm.shape[0], uid]
      act = act_in[worldid, act_last]
      act_dot = util_misc.muscle_dynamics(ctrl, act, dynprm)
    elif dyntype == DynType.DCMOTOR:
      gainprm = actuator_gainprm[worldid % actuator_gainprm.shape[0], uid]
      slots = util_misc.dcmotor_slots(dynprm, gainprm)
      adr = act_first

      act_dot = 0.0

      # slew rate
      if slots[0] >= 0:
        u_prev = act_in[worldid, adr]
        slew_s = dynprm[7]
        slew = slew_s * opt_timestep[worldid % opt_timestep.shape[0]]
        u_eff = wp.clamp(u_first, u_prev - slew, u_prev + slew)
        act_dot = (u_eff - u_prev) / opt_timestep[worldid % opt_timestep.shape[0]]
        act_dot_out[worldid, adr] = act_dot
        u_first = u_eff
        adr += 1

      # integral
      x_I = 0.0
      if slots[1] >= 0:
        x_I = act_in[worldid, adr]
        Imax = dynprm[8]
        act_dot = u_first - actuator_length_in[worldid, uid]

        if Imax > 0.0:
          if x_I >= Imax:
            act_dot = wp.min(act_dot, 0.0)
          elif x_I <= -Imax:
            act_dot = wp.max(act_dot, 0.0)

        act_dot_out[worldid, adr] = act_dot
        adr += 1

      # voltage
      V = 0.0
      if ctrlnum > 0:
        V = util_misc.dcmotor_voltage(
          ctrl_in,
          worldid,
          uadr,
          actuator_ctrlspec[uid],
          u_first,
          actuator_length_in[worldid, uid],
          actuator_velocity_in[worldid, uid],
          x_I,
          gainprm,
        )

      # temperature
      R = gainprm[0]
      K = gainprm[1]
      te = wp.max(MJ_MINVAL, dynprm[0])

      if slots[2] >= 0:
        RT = dynprm[2]
        C = dynprm[3]
        Ta = dynprm[4]
        alpha = gainprm[2]
        T0 = gainprm[3]
        T = act_in[worldid, adr]
        R_eff = R * (1.0 + alpha * (T + Ta - T0))

        current = (V - K * actuator_velocity_in[worldid, uid]) / R_eff
        if slots[4] >= 0:
          current = act_in[worldid, act_last]

        act_dot = (R_eff * current * current - T / RT) / C
        act_dot_out[worldid, adr] = act_dot
        adr += 1
        R = R_eff

      # bristle
      if slots[3] >= 0:
        sigma0 = dynprm[5]
        biasprm = actuator_biasprm[worldid % actuator_biasprm.shape[0], uid]
        F_C = biasprm[3]
        F_S = biasprm[4]
        v_S = biasprm[5]
        z = act_in[worldid, adr]
        g = util_misc.lugre_stribeck(actuator_velocity_in[worldid, uid], F_C, F_S, v_S)
        a = -sigma0 * wp.abs(actuator_velocity_in[worldid, uid]) / wp.max(MJ_MINVAL, g)
        act_dot = a * z + actuator_velocity_in[worldid, uid]
        act_dot_out[worldid, adr] = act_dot
        adr += 1

      # current
      if slots[4] >= 0:
        dimax = dynprm[1]
        act_dot = (V / R - K / R * actuator_velocity_in[worldid, uid] - act_in[worldid, act_last]) / te
        if dimax > 0.0:
          act_dot = wp.clamp(act_dot, -dimax, dimax)
        act_dot_out[worldid, act_last] = act_dot

    elif dyntype == DynType.USER:
      act_dot = 0.0  # set by act_dyn_callback
    else:  # DynType.NONE
      act_dot = 0.0

    act_dot_out[worldid, act_last] = act_dot

    if actuator_actearly[uid]:
      if dyntype == DynType.INTEGRATOR or dyntype == DynType.NONE or dyntype == DynType.DCMOTOR:
        act = act_in[worldid, act_last]

      if dyntype == DynType.DCMOTOR:
        gainprm = actuator_gainprm[worldid % actuator_gainprm.shape[0], uid]
        slots = util_misc.dcmotor_slots(dynprm, gainprm)
        offset = actuator_actnum[uid] - 1

        if offset == slots[4]:  # current
          te = wp.max(MJ_MINVAL, dynprm[0])
          ctrl_act = act + act_dot * te * (1.0 - wp.exp(-opt_timestep[worldid % opt_timestep.shape[0]] / te))
        elif offset == slots[3]:  # bristle
          sigma0 = dynprm[5]
          biasprm = actuator_biasprm[worldid % actuator_biasprm.shape[0], uid]
          F_C = biasprm[3]
          F_S = biasprm[4]
          v_S = biasprm[5]
          velocity = actuator_velocity_in[worldid, uid]
          g = util_misc.lugre_stribeck(velocity, F_C, F_S, v_S)
          a = -sigma0 * wp.abs(velocity) / wp.max(MJ_MINVAL, g)
          h = opt_timestep[worldid % opt_timestep.shape[0]]
          exp_ah = wp.exp(a * h)
          int_h = h
          if wp.abs(a) > MJ_MINVAL:
            int_h = (exp_ah - 1.0) / a
          ctrl_act = exp_ah * act + int_h * velocity
        elif offset == slots[1]:  # integral
          ctrl_act = act + act_dot * opt_timestep[worldid % opt_timestep.shape[0]]
          Imax = dynprm[8]
          if Imax > 0.0:
            ctrl_act = wp.clamp(ctrl_act, -Imax, Imax)
        else:  # temperature or slew or default
          ctrl_act = act + act_dot * opt_timestep[worldid % opt_timestep.shape[0]]

        if actuator_actlimited[uid]:
          actrange = actuator_actrange[worldid % actuator_actrange.shape[0], uid]
          ctrl_act = wp.clamp(ctrl_act, actrange[0], actrange[1])
      else:
        ctrl_act = next_act(
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
  dynprm = actuator_dynprm[worldid % actuator_dynprm.shape[0], uid]

  gain = 0.0
  if gaintype == GainType.FIXED:
    gain = gainprm[0]
  elif gaintype == GainType.AFFINE:
    gain = gainprm[0] + gainprm[1] * length + gainprm[2] * velocity
  elif gaintype == GainType.MUSCLE:
    acc0 = actuator_acc0[worldid % actuator_acc0.shape[0], uid]
    lengthrange = actuator_lengthrange[worldid % actuator_lengthrange.shape[0], uid]
    gain = util_misc.muscle_gain(length, velocity, lengthrange, acc0, gainprm)
  elif gaintype == GainType.DCMOTOR:
    R = gainprm[0]
    K = gainprm[1]
    te = dynprm[0]

    if na and act_first >= 0:
      slots = util_misc.dcmotor_slots(dynprm, gainprm)
      adr = act_first
      if slots[2] >= 0:
        T = act_in[worldid, adr + slots[2]]
        alpha = gainprm[2]
        T0 = gainprm[3]
        Ta = dynprm[4]
        R *= 1.0 + alpha * (T + Ta - T0)

    gain = K if te > 0.0 else K / wp.max(MJ_MINVAL, R)

    if te <= 0.0:
      if ctrlnum == 0:
        ctrl_act = 0.0
      elif (actuator_ctrlspec[uid] & 7) != 0:
        x_I = 0.0
        if na and act_first >= 0:
          slots = util_misc.dcmotor_slots(dynprm, gainprm)
          if slots[1] >= 0:
            x_I = act_in[worldid, act_first + slots[1]]
        ctrl_act = util_misc.dcmotor_voltage(
          ctrl_in,
          worldid,
          uadr,
          actuator_ctrlspec[uid],
          u_first,
          length,
          velocity,
          x_I,
          gainprm,
        )
      else:
        ctrl_act = u_first
  # GainType.USER: gain stays 0, modified by act_gain_callback

  # bias
  biastype = actuator_biastype[uid]
  biasprm = actuator_biasprm[worldid % actuator_biasprm.shape[0], uid]

  bias = 0.0  # BiasType.NONE or BiasType.USER (modified by act_bias_callback)
  if biastype == BiasType.AFFINE:
    bias = biasprm[0] + biasprm[1] * length + biasprm[2] * velocity
  elif biastype == BiasType.MUSCLE:
    acc0 = actuator_acc0[worldid % actuator_acc0.shape[0], uid]
    lengthrange = actuator_lengthrange[worldid % actuator_lengthrange.shape[0], uid]
    bias = util_misc.muscle_bias(length, lengthrange, acc0, biasprm)
  elif biastype == BiasType.DCMOTOR:
    if dynprm[0] <= 0.0:
      K = gainprm[1]
      bias -= gain * K * velocity

  force = gain * ctrl_act + bias

  if actuator_forcelimited[uid]:
    forcerange = actuator_forcerange[worldid % actuator_forcerange.shape[0], uid]
    force = wp.clamp(force, forcerange[0], forcerange[1])

  # add DC motor mechanical forces (not subject to current limits)
  if biastype == BiasType.DCMOTOR:
    # cogging torque
    A = biasprm[0]
    if A != 0.0:
      Np = biasprm[1]
      phi = biasprm[2]
      force += A * wp.sin(Np * length + phi)

    # LuGre friction
    sigma0 = dynprm[5]
    if sigma0 > 0.0:
      sigma1 = dynprm[6]
      slots = util_misc.dcmotor_slots(dynprm, gainprm)
      adr = act_first + slots[3]  # slots[3] is bristle
      z = act_in[worldid, adr]
      z_dot = act_dot_out[worldid, adr]
      force -= sigma0 * z + sigma1 * z_dot

  actuator_force_out[worldid, uid] = force


@wp.kernel
def _tendon_actuator_force(
  # Model:
  actuator_trntype: wp.array[int],
  actuator_trnid: wp.array[wp.vec2i],
  # Data in:
  actuator_force_in: wp.array2d[float],
  # Out:
  ten_actfrc_out: wp.array2d[float],
):
  worldid, actid = wp.tid()

  if actuator_trntype[actid] == TrnType.TENDON:
    tenid = actuator_trnid[actid][0]
    # TODO(team): only compute for tendons with force limits?
    wp.atomic_add(ten_actfrc_out[worldid], tenid, actuator_force_in[worldid, actid])


@wp.kernel
def _tendon_actuator_force_clamp(
  # Model:
  tendon_actfrclimited: wp.array[bool],
  tendon_actfrcrange: wp.array2d[wp.vec2],
  actuator_trntype: wp.array[int],
  actuator_trnid: wp.array[wp.vec2i],
  # In:
  ten_actfrc_in: wp.array2d[float],
  # Data out:
  actuator_force_out: wp.array2d[float],
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
  # Data in:
  moment_rownnz_in: wp.array2d[int],
  moment_rowadr_in: wp.array2d[int],
  moment_colind_in: wp.array2d[int],
  actuator_moment_in: wp.array2d[float],
  actuator_force_in: wp.array2d[float],
  # Data out:
  qfrc_actuator_out: wp.array2d[float],
):
  worldid, actid = wp.tid()

  rownnz = moment_rownnz_in[worldid, actid]
  rowadr = moment_rowadr_in[worldid, actid]

  for i in range(rownnz):
    sparseid = rowadr + i
    colind = moment_colind_in[worldid, sparseid]
    qfrc = actuator_moment_in[worldid, sparseid] * actuator_force_in[worldid, actid]
    wp.atomic_add(qfrc_actuator_out[worldid], colind, qfrc)


@wp.kernel
def _qfrc_actuator_gravcomp_limits(
  # Model:
  jnt_actfrclimited: wp.array[bool],
  jnt_actgravcomp: wp.array[int],
  jnt_actfrcrange: wp.array2d[wp.vec2],
  dof_jntid: wp.array[int],
  # Data in:
  qfrc_gravcomp_in: wp.array2d[float],
  qfrc_actuator_in: wp.array2d[float],
  # In:
  gravity_enabled: bool,
  # Data out:
  qfrc_actuator_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  jntid = dof_jntid[dofid]

  qfrc = qfrc_actuator_in[worldid, dofid]

  # actuator-level gravity compensation, skip if added as passive force
  if gravity_enabled and jnt_actgravcomp[jntid]:
    qfrc += qfrc_gravcomp_in[worldid, dofid]

  # limits
  if jnt_actfrclimited[jntid]:
    frcrange = jnt_actfrcrange[worldid % jnt_actfrcrange.shape[0], jntid]
    qfrc = wp.clamp(qfrc, frcrange[0], frcrange[1])

  qfrc_actuator_out[worldid, dofid] = qfrc


@event_scope
def fwd_actuation(m: Model, d: Data):
  """Actuation-dependent computations."""
  if not m.nactuator or (m.opt.disableflags & DisableBit.ACTUATION):
    d.act_dot.zero_()
    d.qfrc_actuator.zero_()
    d.actuator_force.zero_()
    return

  # read delayed ctrl (or direct copy if no delay)
  if m.nhistory > 0:
    ctrl = wp.empty((d.nworld, m.nu), dtype=float)
    history.read_ctrl_delayed(m, d, ctrl)
  else:
    ctrl = d.ctrl

  wp.launch(
    _actuator_force,
    dim=(d.nworld, m.nactuator),
    inputs=[
      m.na,
      m.opt.timestep,
      m.actuator_dyntype,
      m.actuator_gaintype,
      m.actuator_biastype,
      m.actuator_ctrladr,
      m.actuator_ctrlnum,
      m.actuator_ctrlspec,
      m.actuator_actadr,
      m.actuator_actnum,
      m.actuator_dynprm,
      m.actuator_gainprm,
      m.actuator_biasprm,
      m.actuator_actlimited,
      m.actuator_actrange,
      m.actuator_actearly,
      m.actuator_forcelimited,
      m.actuator_forcerange,
      m.actuator_ctrllimited,
      m.actuator_ctrlrange,
      m.actuator_acc0,
      m.actuator_lengthrange,
      d.act,
      ctrl,
      d.actuator_length,
      d.actuator_velocity,
      m.opt.disableflags & DisableBit.CLAMPCTRL,
    ],
    outputs=[d.act_dot, d.actuator_force],
  )

  if m.callback.act_dyn:
    m.callback.act_dyn(m, d)
  if m.callback.act_gain:
    m.callback.act_gain(m, d)
  if m.callback.act_bias:
    m.callback.act_bias(m, d)

  if m.ntendon:
    # total actuator force at tendon
    ten_actfrc = wp.zeros((d.nworld, m.ntendon), dtype=float)
    wp.launch(
      _tendon_actuator_force,
      dim=(d.nworld, m.nactuator),
      inputs=[m.actuator_trntype, m.actuator_trnid, d.actuator_force],
      outputs=[ten_actfrc],
    )

    wp.launch(
      _tendon_actuator_force_clamp,
      dim=(d.nworld, m.nactuator),
      inputs=[m.tendon_actfrclimited, m.tendon_actfrcrange, m.actuator_trntype, m.actuator_trnid, ten_actfrc],
      outputs=[d.actuator_force],
    )

  # TODO(team): optimize performance
  d.qfrc_actuator.zero_()
  wp.launch(
    _qfrc_actuator,
    dim=(d.nworld, m.nactuator),
    inputs=[
      d.moment_rownnz,
      d.moment_rowadr,
      d.moment_colind,
      d.actuator_moment,
      d.actuator_force,
    ],
    outputs=[d.qfrc_actuator],
  )
  gravity_enabled = not (m.opt.disableflags & DisableBit.GRAVITY)
  wp.launch(
    _qfrc_actuator_gravcomp_limits,
    dim=(d.nworld, m.nv),
    inputs=[
      m.jnt_actfrclimited,
      m.jnt_actgravcomp,
      m.jnt_actfrcrange,
      m.dof_jntid,
      d.qfrc_gravcomp,
      d.qfrc_actuator,
      gravity_enabled,
    ],
    outputs=[d.qfrc_actuator],
  )


@cache_kernel
def _qfrc_smooth(enable_sleep: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    body_treeid: wp.array[int],
    dof_bodyid: wp.array[int],
    # Data in:
    qfrc_applied_in: wp.array2d[float],
    tree_awake_in: wp.array2d[int],
    qfrc_bias_in: wp.array2d[float],
    qfrc_passive_in: wp.array2d[float],
    qfrc_actuator_in: wp.array2d[float],
    # Data out:
    qfrc_smooth_out: wp.array2d[float],
  ):
    worldid, dofid = wp.tid()

    if wp.static(enable_sleep):
      bodyid = dof_bodyid[dofid]
      tree = body_treeid[bodyid]
      if tree >= 0 and tree_awake_in[worldid, tree] == 0:
        qfrc_smooth_out[worldid, dofid] = 0.0
        return

    qfrc_smooth_out[worldid, dofid] = (
      qfrc_passive_in[worldid, dofid]
      - qfrc_bias_in[worldid, dofid]
      + qfrc_actuator_in[worldid, dofid]
      + qfrc_applied_in[worldid, dofid]
    )

  return kernel


@event_scope
def fwd_acceleration(m: Model, d: Data, factorize: bool = False):
  """Add up all non-constraint forces, compute qacc_smooth.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    factorize: Flag to factorize inertia matrix.
  """
  enable_sleep = bool(m.opt.enableflags & EnableBit.SLEEP) and not bool(m.opt.disableflags & DisableBit.ISLAND)
  wp.launch(
    _qfrc_smooth(enable_sleep),
    dim=(d.nworld, m.nv),
    inputs=[
      m.body_treeid,
      m.dof_bodyid,
      d.qfrc_applied,
      d.tree_awake,
      d.qfrc_bias,
      d.qfrc_passive,
      d.qfrc_actuator,
    ],
    outputs=[d.qfrc_smooth],
  )
  xfrc_accumulate(m, d, d.qfrc_smooth)

  if enable_sleep:
    # update the active-DOF set (needs contacts from fwd_position) and solve
    # the smooth acceleration in compacted dense space.
    island.update_active_dofs(m, d)
    solver.smooth_solve_compact(m, d)
  elif factorize:
    smooth.factor_solve_i(m, d, d.M, d.qLD, d.qLDiagInv, d.qacc_smooth, d.qfrc_smooth)
  else:
    smooth.solve_m(m, d, d.qacc_smooth, d.qfrc_smooth)


def _energy_pos(m: Model, d: Data):
  if m.opt.enableflags & EnableBit.ENERGY:
    if m.sensor_e_potential == 0:  # not computed by sensor
      sensor.energy_pos(m, d)
  else:
    d.energy.zero_()


def _energy_vel(m: Model, d: Data):
  if m.opt.enableflags & EnableBit.ENERGY:
    if m.sensor_e_kinetic == 0:  # not computed by sensor
      sensor.energy_vel(m, d)


@event_scope
def forward(m: Model, d: Data):
  """Forward dynamics."""
  sleep_enabled = bool(m.opt.enableflags & EnableBit.SLEEP) and not bool(m.opt.disableflags & DisableBit.ISLAND)
  if sleep_enabled:
    sleep.wake(m, d)
    sleep.update_sleep(m, d)

  fwd_position(m, d, factorize=False)
  d.sensordata.zero_()
  sensor.sensor_pos(m, d)
  _energy_pos(m, d)

  fwd_velocity(m, d)
  sensor.sensor_vel(m, d)
  _energy_vel(m, d)

  if not (m.opt.disableflags & DisableBit.ACTUATION):
    if m.callback.control:
      m.callback.control(m, d)
  fwd_actuation(m, d)
  fwd_acceleration(m, d, factorize=True)

  solver.solve(m, d)
  if m.opt.run_rne_postconstraint or (not (m.opt.disableflags & DisableBit.SENSOR) and m.sensor_rne_postconstraint):
    smooth.rne_postconstraint(m, d)
  sensor.sensor_acc(m, d, skip_rne_postconstraint=True)


@event_scope
def step(m: Model, d: Data):
  """Advance simulation."""
  forward(m, d)

  if m.opt.integrator == IntegratorType.EULER:
    euler(m, d)
  elif m.opt.integrator == IntegratorType.RK4:
    rungekutta4(m, d)
  elif m.opt.integrator in (IntegratorType.IMPLICITFAST, IntegratorType.IMPLICIT):
    implicit(m, d)
  else:
    raise NotImplementedError(f"integrator {m.opt.integrator} not implemented.")


@event_scope
def step1(m: Model, d: Data):
  """Advance simulation in two phases: before input is set by user."""
  fwd_position(m, d)
  d.sensordata.zero_()
  sensor.sensor_pos(m, d)

  _energy_pos(m, d)

  fwd_velocity(m, d)
  sensor.sensor_vel(m, d)

  _energy_vel(m, d)

  if not (m.opt.disableflags & DisableBit.ACTUATION):
    if m.callback.control:
      m.callback.control(m, d)


@event_scope
def step2(m: Model, d: Data):
  """Advance simulation in two phases: after input is set by user."""
  fwd_actuation(m, d)
  fwd_acceleration(m, d)
  solver.solve(m, d)
  if m.opt.run_rne_postconstraint or (not (m.opt.disableflags & DisableBit.SENSOR) and m.sensor_rne_postconstraint):
    smooth.rne_postconstraint(m, d)
  sensor.sensor_acc(m, d, skip_rne_postconstraint=True)

  # integrate with Euler or implicitfast
  if m.opt.integrator in (IntegratorType.IMPLICITFAST, IntegratorType.IMPLICIT):
    implicit(m, d)
  else:
    # note: RK4 defaults to Euler
    euler(m, d)
