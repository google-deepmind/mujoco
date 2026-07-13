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
from mujoco.mjx.third_party.mujoco_warp._src.passive import ellipsoid_max_moment
from mujoco.mjx.third_party.mujoco_warp._src.passive import geom_semiaxes
from mujoco.mjx.third_party.mujoco_warp._src.support import next_act
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import DynType
from mujoco.mjx.third_party.mujoco_warp._src.types import GainType
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType
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
  qDeriv_out: wp.array2d[float],
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
        wp.atomic_add(qDeriv_out[worldid], elemid, contrib)


@wp.kernel
def _qderiv_actuator_passive(
  # Model:
  opt_timestep: wp.array[float],
  opt_disableflags: int,
  dof_damping: wp.array2d[float],
  dof_dampingpoly: wp.array2d[wp.vec2],
  M_elemid: wp.array2d[int],
  # Data in:
  qvel_in: wp.array2d[float],
  M_in: wp.array2d[float],
  # In:
  Mi: wp.array[int],
  Mj: wp.array[int],
  qDeriv_in: wp.array2d[float],
  # Out:
  qDeriv_out: wp.array2d[float],
):
  worldid, elemid = wp.tid()

  dofiid = Mi[elemid]
  dofjid = Mj[elemid]

  # Off-pattern (dofiid, dofjid) pairs have no CSR entry (madr < 0).
  madr = M_elemid[dofiid, dofjid]
  if madr < 0:
    return

  qderiv = qDeriv_in[worldid, madr]

  if not (opt_disableflags & DisableBit.DAMPER) and dofiid == dofjid:
    damping = dof_damping[worldid % dof_damping.shape[0], dofiid]
    dpoly = dof_dampingpoly[worldid % dof_dampingpoly.shape[0], dofiid]
    v = qvel_in[worldid, dofiid]
    qderiv -= util_misc._poly_force_deriv(damping, dpoly, v, 1)

  qderiv *= opt_timestep[worldid % opt_timestep.shape[0]]

  qDeriv_out[worldid, madr] = M_in[worldid, madr] - qderiv


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
  M_elemid: wp.array2d[int],
  # Data in:
  ten_J_in: wp.array2d[float],
  ten_velocity_in: wp.array2d[float],
  # In:
  Mi: wp.array[int],
  Mj: wp.array[int],
  # Out:
  qDeriv_out: wp.array2d[float],
):
  worldid, elemid = wp.tid()
  dofiid = Mi[elemid]
  dofjid = Mj[elemid]

  # Off-pattern (dofiid, dofjid) pairs have no CSR entry (madr < 0).
  madr = M_elemid[dofiid, dofjid]
  if madr < 0:
    return

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

  qDeriv_out[worldid, madr] -= qderiv


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
  qDeriv_out: wp.array2d[float],
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
    wp.atomic_sub(qDeriv_out[worldid], elemid, dt * term)
  else:
    wp.atomic_add(qDeriv_out[worldid], elemid, dt * term)


def deriv_rne_vel(m: Model, d: Data, out: wp.array2d[float], flg_subtract: bool = False):
  """Compute RNE velocity derivatives and add/subtract from the output.

  Implements the analytical derivative of inverse-dynamics Coriolis/centrifugal
  forces with respect to joint velocities.

  Args:
    m: The model (device).
    d: The data (device).
    out: D-structure output array (nworld, nD) to accumulate RNE terms into.
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


@wp.func
def _deriv_ellipsoid_fluid(
  # Model:
  opt_integrator: int,
  geom_type: wp.array[int],
  geom_size: wp.array2d[wp.vec3],
  geom_fluid: wp.array2d[float],
  # Data in:
  xipos_in: wp.array2d[wp.vec3],
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  subtree_com_in: wp.array2d[wp.vec3],
  cvel_in: wp.array2d[wp.spatial_vector],
  # In:
  worldid: int,
  bodyid: int,
  rootid: int,
  geomadr: int,
  geomnum: int,
  cdof_i: wp.spatial_vector,
  cdof_j: wp.spatial_vector,
  wind: wp.vec3,
  density: float,
  viscosity: float,
) -> float:
  """Compute one body's ellipsoid fluid derivative contribution for a DOF pair.

  Returns the scalar J_i^T @ B @ J_j contribution accumulated across geoms.
  """
  is_implicitfast = opt_integrator == IntegratorType.IMPLICITFAST

  # Body kinematics
  xipos = xipos_in[worldid, bodyid]
  cvel = cvel_in[worldid, bodyid]
  ang_global = wp.spatial_top(cvel)
  lin_global = wp.spatial_bottom(cvel)
  subtree_root = subtree_com_in[worldid, rootid]
  lin_com = lin_global - wp.cross(xipos - subtree_root, ang_global)

  qderiv_contrib = float(0.0)

  cdof_ang_i = wp.vec3(cdof_i[0], cdof_i[1], cdof_i[2])
  cdof_lin_i = wp.vec3(cdof_i[3], cdof_i[4], cdof_i[5])
  cdof_ang_j = wp.vec3(cdof_j[0], cdof_j[1], cdof_j[2])
  cdof_lin_j = wp.vec3(cdof_j[3], cdof_j[4], cdof_j[5])

  for g in range(geomnum):
    geomid = geomadr + g
    coef = geom_fluid[geomid, 0]
    if coef <= 0.0:
      continue

    size = geom_size[worldid % geom_size.shape[0], geomid]
    semiaxes = geom_semiaxes(size, geom_type[geomid])
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

    # read fluid coefficients
    blunt_drag_coef = geom_fluid[geomid, 1]
    slender_drag_coef = geom_fluid[geomid, 2]
    ang_drag_coef = geom_fluid[geomid, 3]
    kutta_lift_coef = geom_fluid[geomid, 4]
    magnus_lift_coef = geom_fluid[geomid, 5]
    virtual_mass = wp.vec3(geom_fluid[geomid, 6], geom_fluid[geomid, 7], geom_fluid[geomid, 8])
    virtual_inertia = wp.vec3(geom_fluid[geomid, 9], geom_fluid[geomid, 10], geom_fluid[geomid, 11])

    # ===== Build 6x6 B matrix as four 3x3 quadrants =====
    # B = [[B00, B01], [B10, B11]] where rows are [ang; lin], cols are [ang; lin]
    B00 = wp.mat33(0.0)  # torque wrt ang_vel
    B01 = wp.mat33(0.0)  # torque wrt lin_vel
    B10 = wp.mat33(0.0)  # force wrt ang_vel
    B11 = wp.mat33(0.0)  # force wrt lin_vel

    if density > 0.0:
      # --- added mass forces ---
      density_vm = density * virtual_mass
      density_vi = density * virtual_inertia
      virtual_lin_mom = wp.cw_mul(density_vm, lin_vel)
      virtual_ang_mom = wp.cw_mul(density_vi, ang_vel)

      # torque += cross(virtual_ang_mom, ang_vel) -> B00
      B00 += wp.skew(virtual_ang_mom) - wp.skew(ang_vel) @ wp.diag(density_vi)

      # torque += cross(virtual_lin_mom, lin_vel) -> B01
      B01 += wp.skew(virtual_lin_mom) - wp.skew(lin_vel) @ wp.diag(density_vm)

      # force += cross(virtual_lin_mom, ang_vel) -> B10
      B10 += wp.skew(virtual_lin_mom)
      # Da = d/d(vlm via lin) = _skew_neg(ang), scaled by density*vm -> B11
      B11 += -wp.skew(ang_vel) @ wp.diag(density_vm)

    # --- Magnus force: force += magnus_coef * cross(ang_vel, lin_vel) ---
    volume = wp.static(4.0 / 3.0 * wp.pi) * semiaxes[0] * semiaxes[1] * semiaxes[2]
    magnus_coef = magnus_lift_coef * density * volume
    B10 -= wp.skew(lin_vel) * magnus_coef
    B11 += wp.skew(ang_vel) * magnus_coef

    # --- Kutta lift (3x3 -> B11) ---
    a = (semiaxes[1] * semiaxes[2]) * (semiaxes[1] * semiaxes[2])
    b = (semiaxes[2] * semiaxes[0]) * (semiaxes[2] * semiaxes[0])
    c = (semiaxes[0] * semiaxes[1]) * (semiaxes[0] * semiaxes[1])
    aa = a * a
    bb = b * b
    cc = c * c

    x = lin_vel[0]
    y = lin_vel[1]
    z = lin_vel[2]
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    yz = y * z
    xz = x * z

    proj_denom = aa * xx + bb * yy + cc * zz
    proj_num = a * xx + b * yy + c * zz
    norm2 = xx + yy + zz
    df_denom = wp.pi * kutta_lift_coef * density / wp.max(MJ_MINVAL, wp.sqrt(proj_denom * proj_num * norm2))

    dfx_coef = yy * (a - b) + zz * (a - c)
    dfy_coef = xx * (b - a) + zz * (b - c)
    dfz_coef = xx * (c - a) + yy * (c - b)
    proj_term = proj_num / wp.max(MJ_MINVAL, proj_denom)
    cos_term = proj_num / wp.max(MJ_MINVAL, norm2)

    D = wp.skew(wp.vec3(b - c, c - a, a - b)) * (2.0 * proj_num)

    df_coef = wp.vec3(dfx_coef, dfy_coef, dfz_coef)
    inner_term = wp.vec3(
      aa * proj_term - a + cos_term,
      bb * proj_term - b + cos_term,
      cc * proj_term - c + cos_term,
    )

    D += wp.outer(df_coef, inner_term)

    V = wp.diag(lin_vel)
    D = V @ D @ V - wp.diag(df_coef * proj_num)

    D *= df_denom
    B11 += D

    # --- viscous drag (3x3 -> B11) ---
    d_max = wp.max(wp.max(semiaxes[0], semiaxes[1]), semiaxes[2])
    d_min = wp.min(wp.min(semiaxes[0], semiaxes[1]), semiaxes[2])
    d_mid = semiaxes[0] + semiaxes[1] + semiaxes[2] - d_max - d_min
    eq_sphere_D = wp.static(2.0 / 3.0) * (semiaxes[0] + semiaxes[1] + semiaxes[2])
    A_max = wp.pi * d_max * d_mid

    A_proj = wp.pi * wp.sqrt(proj_denom / wp.max(MJ_MINVAL, proj_num))

    norm = wp.sqrt(xx + yy + zz)
    inv_norm = 1.0 / wp.max(MJ_MINVAL, norm)

    lin_coef = viscosity * wp.static(3.0 * wp.pi) * eq_sphere_D
    quad_coef = density * (A_proj * blunt_drag_coef + slender_drag_coef * (A_max - A_proj))
    Aproj_coef = density * norm * (blunt_drag_coef - slender_drag_coef)
    dA_coef = wp.pi / wp.max(MJ_MINVAL, wp.sqrt(proj_num * proj_num * proj_num * proj_denom))

    dAproj_dv = wp.vec3(
      Aproj_coef * dA_coef * a * x * (b * yy * (a - b) + c * zz * (a - c)),
      Aproj_coef * dA_coef * b * y * (a * xx * (b - a) + c * zz * (b - c)),
      Aproj_coef * dA_coef * c * z * (a * xx * (c - a) + b * yy * (c - b)),
    )

    inner = wp.length_sq(lin_vel)
    D = (wp.outer(lin_vel, lin_vel) + wp.diag(wp.vec3(inner))) * (-quad_coef * inv_norm)
    D -= wp.outer(lin_vel, dAproj_dv)
    D -= wp.diag(wp.vec3(lin_coef))

    B11 += D

    # --- viscous torque (3x3 -> B00) ---
    lin_visc_torq_coef = wp.pi * eq_sphere_D * eq_sphere_D * eq_sphere_D
    I_max = wp.static(8.0 / 15.0 * wp.pi) * d_mid * d_max * d_max * d_max * d_max
    II = wp.vec3(
      ellipsoid_max_moment(semiaxes, 0),
      ellipsoid_max_moment(semiaxes, 1),
      ellipsoid_max_moment(semiaxes, 2),
    )

    mom_coef = wp.vec3(
      ang_drag_coef * II[0] + slender_drag_coef * (I_max - II[0]),
      ang_drag_coef * II[1] + slender_drag_coef * (I_max - II[1]),
      ang_drag_coef * II[2] + slender_drag_coef * (I_max - II[2]),
    )

    mom_visc = wp.cw_mul(ang_vel, mom_coef)
    norm_mom = wp.length(mom_visc)
    density_scaled = density / wp.max(MJ_MINVAL, norm_mom)

    mom_sq = -density_scaled * wp.cw_mul(wp.cw_mul(ang_vel, mom_coef), mom_coef)

    torq_lin_coef = viscosity * lin_visc_torq_coef
    diag_val = wp.dot(ang_vel, mom_sq) - torq_lin_coef

    D = wp.outer(ang_vel, mom_sq) + wp.diag(wp.vec3(diag_val))
    B00 += D

    # symmetrize for implicitfast
    if is_implicitfast:
      B00 = 0.5 * (B00 + wp.transpose(B00))
      B11 = 0.5 * (B11 + wp.transpose(B11))
      B01_sym = 0.5 * (B01 + wp.transpose(B10))
      B01 = B01_sym
      B10 = wp.transpose(B01_sym)

    # --- Jacobian transformation: J_i^T @ B @ J_j ---
    offset = geom_pos - subtree_root

    jac_p_i = cdof_lin_i + wp.cross(cdof_ang_i, offset)
    la_i = geom_rotT @ cdof_ang_i
    ll_i = geom_rotT @ jac_p_i

    jac_p_j = cdof_lin_j + wp.cross(cdof_ang_j, offset)
    la_j = geom_rotT @ cdof_ang_j
    ll_j = geom_rotT @ jac_p_j

    # B @ J_j = [B00 @ la_j + B01 @ ll_j; B10 @ la_j + B11 @ ll_j]
    Bj_ang = B00 @ la_j + B01 @ ll_j
    Bj_lin = B10 @ la_j + B11 @ ll_j

    # J_i^T @ (B @ J_j) = la_i . Bj_ang + ll_i . Bj_lin
    qderiv_contrib += wp.dot(la_i, Bj_ang) + wp.dot(ll_i, Bj_lin)

  return qderiv_contrib


@wp.kernel
def _qderiv_ellipsoid_fluid(
  # Model:
  opt_timestep: wp.array[float],
  opt_wind: wp.array[wp.vec3],
  opt_density: wp.array[float],
  opt_viscosity: wp.array[float],
  opt_integrator: int,
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  body_geomnum: wp.array[int],
  body_geomadr: wp.array[int],
  dof_bodyid: wp.array[int],
  geom_type: wp.array[int],
  geom_size: wp.array2d[wp.vec3],
  geom_fluid: wp.array2d[float],
  body_fluid_ellipsoid_adr: wp.array[int],
  body_isdofancestor: wp.array2d[int],
  M_elemid: wp.array2d[int],
  # Data in:
  xipos_in: wp.array2d[wp.vec3],
  geom_xpos_in: wp.array2d[wp.vec3],
  geom_xmat_in: wp.array2d[wp.mat33],
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  cvel_in: wp.array2d[wp.spatial_vector],
  # In:
  Mi: wp.array[int],
  Mj: wp.array[int],
  # Out:
  qDeriv_out: wp.array2d[float],
):
  """Compute ellipsoid fluid force derivative contribution to qDeriv.

  Parallelized over (world, fluid_body, elem). For each fluid body and DOF
  pair, computes the 6x6 derivative matrix B in local geom frame via
  _deriv_ellipsoid_fluid and accumulates J_i^T @ B @ J_j into qDeriv.
  """
  worldid, fluid_idx, elemid = wp.tid()

  bodyid = body_fluid_ellipsoid_adr[fluid_idx]

  dofiid = Mi[elemid]
  dofjid = Mj[elemid]

  madr = M_elemid[dofiid, dofjid]
  if madr < 0:
    return

  # dofiid is the "deeper" DOF (Mi >= Mj in tree ordering).
  # Any body that has dofiid in its chain also has dofjid.
  bodyid_i = dof_bodyid[dofiid]

  if bodyid_i == 0:
    return

  if body_isdofancestor[bodyid, dofiid] == 0:
    return

  wind = opt_wind[worldid % opt_wind.shape[0]]
  density = opt_density[worldid % opt_density.shape[0]]
  viscosity = opt_viscosity[worldid % opt_viscosity.shape[0]]
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  if density <= 0.0 and viscosity <= 0.0:
    return

  cdof_i = cdof_in[worldid, dofiid]
  cdof_j = cdof_in[worldid, dofjid]

  contrib = _deriv_ellipsoid_fluid(
    opt_integrator,
    geom_type,
    geom_size,
    geom_fluid,
    xipos_in,
    geom_xpos_in,
    geom_xmat_in,
    subtree_com_in,
    cvel_in,
    worldid,
    bodyid,
    body_rootid[bodyid],
    body_geomadr[bodyid],
    body_geomnum[bodyid],
    cdof_i,
    cdof_j,
    wind,
    density,
    viscosity,
  )

  contrib *= timestep

  if contrib != 0.0:
    wp.atomic_add(qDeriv_out[worldid], madr, -contrib)


@wp.func
def _deriv_box_fluid(
  # Model:
  opt_integrator: int,
  body_mass: wp.array2d[float],
  body_inertia: wp.array2d[wp.vec3],
  # In:
  worldid: int,
  bodyid: int,
  lvel: wp.spatial_vector,
  density: float,
  viscosity: float,
) -> wp.spatial_matrix:
  B = wp.spatial_matrix(0.0)

  mass = body_mass[worldid % body_mass.shape[0], bodyid]
  inertia = body_inertia[worldid % body_inertia.shape[0], bodyid]
  scl = 6.0 / mass

  # Equivalent inertia box
  box = wp.vec3(
    wp.sqrt(wp.max(MJ_MINVAL, inertia[1] + inertia[2] - inertia[0]) * scl),
    wp.sqrt(wp.max(MJ_MINVAL, inertia[0] + inertia[2] - inertia[1]) * scl),
    wp.sqrt(wp.max(MJ_MINVAL, inertia[0] + inertia[1] - inertia[2]) * scl),
  )

  # Viscous force and torque
  if viscosity > 0.0:
    diam = (box[0] + box[1] + box[2]) * wp.static(1.0 / 3.0)

    # Rotational viscosity
    visc_rot = -wp.pi * diam * diam * diam * viscosity
    B[0, 0] += visc_rot
    B[1, 1] += visc_rot
    B[2, 2] += visc_rot

    # Translational viscosity
    visc_lin = wp.static(-3.0 * wp.pi) * diam * viscosity
    B[3, 3] += visc_lin
    B[4, 4] += visc_lin
    B[5, 5] += visc_lin

  # Lift and drag force and torque
  if density > 0.0:
    term0 = box[1] * box[1] * box[1] * box[1] + box[2] * box[2] * box[2] * box[2]
    term1 = box[0] * box[0] * box[0] * box[0] + box[2] * box[2] * box[2] * box[2]
    term2 = box[0] * box[0] * box[0] * box[0] + box[1] * box[1] * box[1] * box[1]

    inv_32 = wp.static(1.0 / 32.0)
    B[0, 0] -= density * box[0] * term0 * wp.abs(lvel[0]) * inv_32
    B[1, 1] -= density * box[1] * term1 * wp.abs(lvel[1]) * inv_32
    B[2, 2] -= density * box[2] * term2 * wp.abs(lvel[2]) * inv_32

    B[3, 3] -= density * box[1] * box[2] * wp.abs(lvel[3])
    B[4, 4] -= density * box[0] * box[2] * wp.abs(lvel[4])
    B[5, 5] -= density * box[0] * box[1] * wp.abs(lvel[5])

  if opt_integrator == IntegratorType.IMPLICITFAST:
    B = 0.5 * (B + wp.transpose(B))

  return B


@wp.func
def _get_jac_column_local(
  # Model:
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  dof_bodyid: wp.array[int],
  # Data in:
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  # In:
  point_global: wp.vec3,
  bodyid: int,
  dofid: int,
  worldid: int,
  b_imat: wp.mat33,
) -> wp.spatial_vector:
  offset = point_global - subtree_com_in[worldid, body_rootid[bodyid]]
  cdof_val = cdof_in[worldid, dofid]
  cdof_ang = wp.spatial_top(cdof_val)
  cdof_lin = wp.spatial_bottom(cdof_val)

  jacp = cdof_lin + wp.cross(cdof_ang, offset)
  jacr = cdof_ang

  b_imat_T = wp.transpose(b_imat)
  jacp_loc = b_imat_T @ jacp
  jacr_loc = b_imat_T @ jacr
  return wp.spatial_vector(jacr_loc, jacp_loc)


@wp.kernel
def _qderiv_box_fluid(
  # Model:
  opt_timestep: wp.array[float],
  opt_wind: wp.array[wp.vec3],
  opt_density: wp.array[float],
  opt_viscosity: wp.array[float],
  opt_integrator: int,
  body_parentid: wp.array[int],
  body_rootid: wp.array[int],
  body_mass: wp.array2d[float],
  body_inertia: wp.array2d[wp.vec3],
  dof_bodyid: wp.array[int],
  body_fluid_box_adr: wp.array[int],
  body_isdofancestor: wp.array2d[int],
  M_elemid: wp.array2d[int],
  # Data in:
  xipos_in: wp.array2d[wp.vec3],
  ximat_in: wp.array2d[wp.mat33],
  subtree_com_in: wp.array2d[wp.vec3],
  cdof_in: wp.array2d[wp.spatial_vector],
  cvel_in: wp.array2d[wp.spatial_vector],
  # In:
  Mi: wp.array[int],
  Mj: wp.array[int],
  # Out:
  qDeriv_out: wp.array2d[float],
):
  worldid, fluid_idx, elemid = wp.tid()

  bodyid = body_fluid_box_adr[fluid_idx]

  dofiid = Mi[elemid]
  dofjid = Mj[elemid]

  madr = M_elemid[dofiid, dofjid]
  if madr < 0:
    return

  bodyid_i = dof_bodyid[dofiid]

  if bodyid_i == 0:
    return

  if body_isdofancestor[bodyid, dofiid] == 0:
    return

  wind = opt_wind[worldid % opt_wind.shape[0]]
  density = opt_density[worldid % opt_density.shape[0]]
  viscosity = opt_viscosity[worldid % opt_viscosity.shape[0]]
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  if density <= 0.0 and viscosity <= 0.0:
    return

  # Body velocity and kinematics
  b_ipos = xipos_in[worldid, bodyid]
  b_imat = ximat_in[worldid, bodyid]
  subtree_root = subtree_com_in[worldid, body_rootid[bodyid]]

  vel_subtree = cvel_in[worldid, bodyid]
  v_subtree_ang = wp.vec3(vel_subtree[0], vel_subtree[1], vel_subtree[2])
  v_subtree_lin = wp.vec3(vel_subtree[3], vel_subtree[4], vel_subtree[5])

  lin_com = v_subtree_lin - wp.cross(b_ipos - subtree_root, v_subtree_ang)
  b_imat_T = wp.transpose(b_imat)
  v_local_ang = b_imat_T @ v_subtree_ang
  v_local_lin = b_imat_T @ lin_com
  wind_local = b_imat_T @ wind

  lvel = wp.spatial_vector(v_local_ang, v_local_lin - wind_local)

  B = _deriv_box_fluid(
    opt_integrator,
    body_mass,
    body_inertia,
    worldid,
    bodyid,
    lvel,
    density,
    viscosity,
  )

  # Jacobian transformation: J_i^T @ B @ J_j
  J_i = _get_jac_column_local(
    body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, b_ipos, bodyid, dofiid, worldid, b_imat
  )
  J_j = _get_jac_column_local(
    body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, b_ipos, bodyid, dofjid, worldid, b_imat
  )

  contrib = wp.dot(J_i, B @ J_j) * timestep

  if contrib != 0.0:
    wp.atomic_add(qDeriv_out[worldid], madr, -contrib)


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
      # out (qDeriv) is in M-structure.
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
    wp.launch(
      _qderiv_actuator_passive,
      dim=(d.nworld, Mi.size),
      inputs=[
        m.opt.timestep,
        m.opt.disableflags,
        m.dof_damping,
        m.dof_dampingpoly,
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
        m.M_elemid,
        d.ten_J,
        d.ten_velocity,
        Mi,
        Mj,
      ],
      outputs=[out],
    )
  if m.has_fluid:
    if m.body_fluid_ellipsoid_adr.size > 0:
      wp.launch(
        _qderiv_ellipsoid_fluid,
        dim=(d.nworld, m.body_fluid_ellipsoid_adr.size, Mi.size),
        inputs=[
          m.opt.timestep,
          m.opt.wind,
          m.opt.density,
          m.opt.viscosity,
          m.opt.integrator,
          m.body_parentid,
          m.body_rootid,
          m.body_geomnum,
          m.body_geomadr,
          m.dof_bodyid,
          m.geom_type,
          m.geom_size,
          m.geom_fluid,
          m.body_fluid_ellipsoid_adr,
          m.body_isdofancestor,
          m.M_elemid,
          d.xipos,
          d.geom_xpos,
          d.geom_xmat,
          d.subtree_com,
          d.cdof,
          d.cvel,
          Mi,
          Mj,
        ],
        outputs=[out],
      )
    if m.body_fluid_box_adr.size > 0:
      wp.launch(
        _qderiv_box_fluid,
        dim=(d.nworld, m.body_fluid_box_adr.size, Mi.size),
        inputs=[
          m.opt.timestep,
          m.opt.wind,
          m.opt.density,
          m.opt.viscosity,
          m.opt.integrator,
          m.body_parentid,
          m.body_rootid,
          m.body_mass,
          m.body_inertia,
          m.dof_bodyid,
          m.body_fluid_box_adr,
          m.body_isdofancestor,
          m.M_elemid,
          d.xipos,
          d.ximat,
          d.subtree_com,
          d.cdof,
          d.cvel,
          Mi,
          Mj,
        ],
        outputs=[out],
      )
