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
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DisableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.func
def _pow2(val: float) -> float:
  return val * val


@wp.func
def _pow4(val: float) -> float:
  sq = val * val
  return sq * sq


@wp.func
def _geom_semiaxes(size: wp.vec3, geom_type: int) -> wp.vec3:  # kernel_analyzer: ignore
  if geom_type == GeomType.SPHERE:
    r = size[0]
    return wp.vec3(r, r, r)

  if geom_type == GeomType.CAPSULE:
    radius = size[0]
    half_length = size[1]
    return wp.vec3(radius, radius, half_length + radius)

  if geom_type == GeomType.CYLINDER:
    radius = size[0]
    half_length = size[1]
    return wp.vec3(radius, radius, half_length)

  # ellipsoid, box, mesh, sdf -> use size directly
  return size


@wp.func
def _ellipsoid_max_moment(size: wp.vec3, dir: int) -> float:
  d0 = size[dir]
  d1 = size[(dir + 1) % 3]
  d2 = size[(dir + 2) % 3]
  return wp.static(8.0 / 15.0 * wp.pi) * d0 * _pow4(wp.max(d1, d2))


@wp.kernel
def _spring_damper_dof_passive(
  # Model:
  opt_disableflags: int,
  qpos_spring: wp.array2d(dtype=float),
  jnt_type: wp.array(dtype=int),
  jnt_qposadr: wp.array(dtype=int),
  jnt_dofadr: wp.array(dtype=int),
  jnt_stiffness: wp.array2d(dtype=float),
  dof_damping: wp.array2d(dtype=float),
  # Data in:
  qpos_in: wp.array2d(dtype=float),
  qvel_in: wp.array2d(dtype=float),
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
  qfrc_damper_out: wp.array2d(dtype=float),
):
  worldid, jntid = wp.tid()
  dofid = jnt_dofadr[jntid]
  stiffness = jnt_stiffness[worldid % jnt_stiffness.shape[0], jntid]
  damping = dof_damping[worldid % dof_damping.shape[0], dofid]

  has_stiffness = stiffness != 0.0 and not opt_disableflags & DisableBit.SPRING
  has_damping = damping != 0.0 and not opt_disableflags & DisableBit.DAMPER

  if not has_stiffness:
    qfrc_spring_out[worldid, dofid] = 0.0

  if not has_damping:
    qfrc_damper_out[worldid, dofid] = 0.0

  if not (has_stiffness or has_damping):
    return

  jnttype = jnt_type[jntid]
  qposid = jnt_qposadr[jntid]
  qpos_spring_id = worldid % qpos_spring.shape[0]

  if jnttype == JointType.FREE:
    # spring
    if has_stiffness:
      dif = wp.vec3(
        qpos_in[worldid, qposid + 0] - qpos_spring[qpos_spring_id, qposid + 0],
        qpos_in[worldid, qposid + 1] - qpos_spring[qpos_spring_id, qposid + 1],
        qpos_in[worldid, qposid + 2] - qpos_spring[qpos_spring_id, qposid + 2],
      )
      qfrc_spring_out[worldid, dofid + 0] = -stiffness * dif[0]
      qfrc_spring_out[worldid, dofid + 1] = -stiffness * dif[1]
      qfrc_spring_out[worldid, dofid + 2] = -stiffness * dif[2]
      rot = wp.quat(
        qpos_in[worldid, qposid + 3],
        qpos_in[worldid, qposid + 4],
        qpos_in[worldid, qposid + 5],
        qpos_in[worldid, qposid + 6],
      )
      rot = wp.normalize(rot)
      ref = wp.quat(
        qpos_spring[qpos_spring_id, qposid + 3],
        qpos_spring[qpos_spring_id, qposid + 4],
        qpos_spring[qpos_spring_id, qposid + 5],
        qpos_spring[qpos_spring_id, qposid + 6],
      )
      dif = math.quat_sub(rot, ref)
      qfrc_spring_out[worldid, dofid + 3] = -stiffness * dif[0]
      qfrc_spring_out[worldid, dofid + 4] = -stiffness * dif[1]
      qfrc_spring_out[worldid, dofid + 5] = -stiffness * dif[2]

    # damper
    if has_damping:
      qfrc_damper_out[worldid, dofid + 0] = -damping * qvel_in[worldid, dofid + 0]
      qfrc_damper_out[worldid, dofid + 1] = -damping * qvel_in[worldid, dofid + 1]
      qfrc_damper_out[worldid, dofid + 2] = -damping * qvel_in[worldid, dofid + 2]
      qfrc_damper_out[worldid, dofid + 3] = -damping * qvel_in[worldid, dofid + 3]
      qfrc_damper_out[worldid, dofid + 4] = -damping * qvel_in[worldid, dofid + 4]
      qfrc_damper_out[worldid, dofid + 5] = -damping * qvel_in[worldid, dofid + 5]
  elif jnttype == JointType.BALL:
    # spring
    if has_stiffness:
      rot = wp.quat(
        qpos_in[worldid, qposid + 0],
        qpos_in[worldid, qposid + 1],
        qpos_in[worldid, qposid + 2],
        qpos_in[worldid, qposid + 3],
      )
      rot = wp.normalize(rot)
      ref = wp.quat(
        qpos_spring[qpos_spring_id, qposid + 0],
        qpos_spring[qpos_spring_id, qposid + 1],
        qpos_spring[qpos_spring_id, qposid + 2],
        qpos_spring[qpos_spring_id, qposid + 3],
      )
      dif = math.quat_sub(rot, ref)
      qfrc_spring_out[worldid, dofid + 0] = -stiffness * dif[0]
      qfrc_spring_out[worldid, dofid + 1] = -stiffness * dif[1]
      qfrc_spring_out[worldid, dofid + 2] = -stiffness * dif[2]

    # damper
    if has_damping:
      qfrc_damper_out[worldid, dofid + 0] = -damping * qvel_in[worldid, dofid + 0]
      qfrc_damper_out[worldid, dofid + 1] = -damping * qvel_in[worldid, dofid + 1]
      qfrc_damper_out[worldid, dofid + 2] = -damping * qvel_in[worldid, dofid + 2]
  else:  # mjJNT_SLIDE, mjJNT_HINGE
    # spring
    if has_stiffness:
      fdif = qpos_in[worldid, qposid] - qpos_spring[qpos_spring_id, qposid]
      qfrc_spring_out[worldid, dofid] = -stiffness * fdif

    # damper
    if has_damping:
      qfrc_damper_out[worldid, dofid] = -damping * qvel_in[worldid, dofid]


@wp.kernel
def _spring_damper_tendon_passive(
  # Model:
  tendon_stiffness: wp.array2d(dtype=float),
  tendon_damping: wp.array2d(dtype=float),
  tendon_lengthspring: wp.array2d(dtype=wp.vec2),
  # Data in:
  ten_J_in: wp.array3d(dtype=float),
  ten_length_in: wp.array2d(dtype=float),
  ten_velocity_in: wp.array2d(dtype=float),
  # In:
  dsbl_spring: bool,
  dsbl_damper: bool,
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
  qfrc_damper_out: wp.array2d(dtype=float),
):
  worldid, tenid, dofid = wp.tid()

  stiffness = tendon_stiffness[worldid % tendon_stiffness.shape[0], tenid]
  damping = tendon_damping[worldid % tendon_damping.shape[0], tenid]

  has_stiffness = stiffness != 0.0 and not dsbl_spring
  has_damping = damping != 0.0 and not dsbl_damper

  if not has_stiffness and not has_damping:
    return

  J = ten_J_in[worldid, tenid, dofid]

  if has_stiffness:
    # compute spring force along tendon
    length = ten_length_in[worldid, tenid]
    lengthspring = tendon_lengthspring[worldid % tendon_lengthspring.shape[0], tenid]
    lower = lengthspring[0]
    upper = lengthspring[1]

    if length > upper:
      frc_spring = stiffness * (upper - length)
    elif length < lower:
      frc_spring = stiffness * (lower - length)
    else:
      frc_spring = 0.0

    # transform to joint torque
    wp.atomic_add(qfrc_spring_out[worldid], dofid, J * frc_spring)

  if has_damping:
    # compute damper linear force along tendon
    frc_damper = -damping * ten_velocity_in[worldid, tenid]

    # transform to joint torque
    wp.atomic_add(qfrc_damper_out[worldid], dofid, J * frc_damper)


@wp.kernel
def _gravity_force(
  # Model:
  opt_gravity: wp.array(dtype=wp.vec3),
  body_parentid: wp.array(dtype=int),
  body_rootid: wp.array(dtype=int),
  body_mass: wp.array2d(dtype=float),
  body_gravcomp: wp.array2d(dtype=float),
  dof_bodyid: wp.array(dtype=int),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cdof_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  qfrc_gravcomp_out: wp.array2d(dtype=float),
):
  worldid, bodyid, dofid = wp.tid()
  bodyid += 1  # skip world body
  gravcomp = body_gravcomp[worldid % body_gravcomp.shape[0], bodyid]
  gravity = opt_gravity[worldid % opt_gravity.shape[0]]

  if gravcomp:
    force = -gravity * body_mass[worldid % body_mass.shape[0], bodyid] * gravcomp
    pos = xipos_in[worldid, bodyid]
    jac, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, pos, bodyid, dofid, worldid)

    wp.atomic_add(qfrc_gravcomp_out[worldid], dofid, wp.dot(jac, force))


@wp.kernel
def _fluid_force(
  # Model:
  opt_density: wp.array(dtype=float),
  opt_viscosity: wp.array(dtype=float),
  opt_wind: wp.array(dtype=wp.vec3),
  body_rootid: wp.array(dtype=int),
  body_geomnum: wp.array(dtype=int),
  body_geomadr: wp.array(dtype=int),
  body_mass: wp.array2d(dtype=float),
  body_inertia: wp.array2d(dtype=wp.vec3),
  geom_type: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  geom_fluid: wp.array2d(dtype=float),
  body_fluid_ellipsoid: wp.array(dtype=bool),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # Out:
  fluid_applied_out: wp.array2d(dtype=wp.spatial_vector),
):
  """Computes body-space fluid forces for both inertia-box and ellipsoid models."""
  worldid, bodyid = wp.tid()
  zero_force = wp.spatial_vector(wp.vec3(0.0), wp.vec3(0.0))

  if bodyid == 0:
    fluid_applied_out[worldid, bodyid] = zero_force
    return

  wind = opt_wind[worldid % opt_wind.shape[0]]
  density = opt_density[worldid % opt_density.shape[0]]
  viscosity = opt_viscosity[worldid % opt_viscosity.shape[0]]

  # Body kinematics
  xipos = xipos_in[worldid, bodyid]
  rot = ximat_in[worldid, bodyid]
  rotT = wp.transpose(rot)
  cvel = cvel_in[worldid, bodyid]
  ang_global = wp.spatial_top(cvel)
  lin_global = wp.spatial_bottom(cvel)
  subtree_root = subtree_com_in[worldid, body_rootid[bodyid]]
  lin_com = lin_global - wp.cross(xipos - subtree_root, ang_global)

  if body_fluid_ellipsoid[bodyid]:
    force_global = wp.vec3(0.0)
    torque_global = wp.vec3(0.0)

    start = body_geomadr[bodyid]
    count = body_geomnum[bodyid]

    for i in range(count):
      geomid = start + i
      coef = geom_fluid[geomid, 0]
      if coef <= 0.0:
        continue

      size = geom_size[worldid % geom_size.shape[0], geomid]
      semiaxes = _geom_semiaxes(size, geom_type[geomid])
      geom_rot = geom_xmat_in[worldid, geomid]
      geom_rotT = wp.transpose(geom_rot)
      geom_pos = geom_xpos_in[worldid, geomid]

      lin_point = lin_com + wp.cross(ang_global, geom_pos - xipos)

      l_ang = geom_rotT @ ang_global
      l_lin = geom_rotT @ lin_point

      if wind[0] or wind[1] or wind[2]:
        l_lin -= geom_rotT @ wind

      lfrc_torque = wp.vec3(0.0)
      lfrc_force = wp.vec3(0.0)

      if density > 0.0:
        # added-mass forces and torques
        virtual_mass = wp.vec3(geom_fluid[geomid, 6], geom_fluid[geomid, 7], geom_fluid[geomid, 8])
        virtual_inertia = wp.vec3(geom_fluid[geomid, 9], geom_fluid[geomid, 10], geom_fluid[geomid, 11])

        virtual_lin_mom = wp.vec3(
          density * virtual_mass[0] * l_lin[0],
          density * virtual_mass[1] * l_lin[1],
          density * virtual_mass[2] * l_lin[2],
        )
        virtual_ang_mom = wp.vec3(
          density * virtual_inertia[0] * l_ang[0],
          density * virtual_inertia[1] * l_ang[1],
          density * virtual_inertia[2] * l_ang[2],
        )

        added_mass_force = wp.cross(virtual_lin_mom, l_ang)
        added_mass_torque = wp.cross(virtual_lin_mom, l_lin) + wp.cross(virtual_ang_mom, l_ang)

        lfrc_force += added_mass_force
        lfrc_torque += added_mass_torque

      # lift force orthogonal to velocity from Kutta-Joukowski theorem
      magnus_coef = geom_fluid[geomid, 5]
      kutta_coef = geom_fluid[geomid, 4]
      blunt_drag_coef = geom_fluid[geomid, 1]
      slender_drag_coef = geom_fluid[geomid, 2]
      ang_drag_coef = geom_fluid[geomid, 3]

      volume = wp.static(4.0 / 3.0 * wp.pi) * semiaxes[0] * semiaxes[1] * semiaxes[2]
      d_max = wp.max(wp.max(semiaxes[0], semiaxes[1]), semiaxes[2])
      d_min = wp.min(wp.min(semiaxes[0], semiaxes[1]), semiaxes[2])
      d_mid = semiaxes[0] + semiaxes[1] + semiaxes[2] - d_max - d_min
      A_max = wp.pi * d_max * d_mid

      lin_speed = wp.length(l_lin)

      magnus_force = wp.cross(l_ang, l_lin) * (magnus_coef * density * volume)

      s12 = semiaxes[1] * semiaxes[2]
      s20 = semiaxes[2] * semiaxes[0]
      s01 = semiaxes[0] * semiaxes[1]

      proj_denom = _pow4(s12) * _pow2(l_lin[0]) + _pow4(s20) * _pow2(l_lin[1]) + _pow4(s01) * _pow2(l_lin[2])
      proj_num = _pow2(s12 * l_lin[0]) + _pow2(s20 * l_lin[1]) + _pow2(s01 * l_lin[2])

      A_proj = 0.0
      cos_alpha = 0.0
      if proj_num > MJ_MINVAL and proj_denom > MJ_MINVAL:
        A_proj = wp.pi * wp.sqrt(proj_denom / wp.max(MJ_MINVAL, proj_num))
        if lin_speed > MJ_MINVAL:
          cos_alpha = proj_num / wp.max(MJ_MINVAL, lin_speed * proj_denom)

      norm = wp.vec3(
        _pow2(s12) * l_lin[0],
        _pow2(s20) * l_lin[1],
        _pow2(s01) * l_lin[2],
      )

      kutta_force = wp.vec3(0.0)
      if density > 0.0 and kutta_coef != 0.0 and lin_speed > MJ_MINVAL:
        kutta_circ = wp.cross(norm, l_lin) * (kutta_coef * density * cos_alpha * A_proj)
        kutta_force = wp.cross(kutta_circ, l_lin)

      eq_sphere_D = wp.static(2.0 / 3.0) * (semiaxes[0] + semiaxes[1] + semiaxes[2])
      lin_visc_force_coef = wp.static(3.0 * wp.pi) * eq_sphere_D
      lin_visc_torq_coef = wp.pi * eq_sphere_D * eq_sphere_D * eq_sphere_D

      I_max = wp.static(8.0 / 15.0 * wp.pi) * d_mid * _pow4(d_max)
      II0 = _ellipsoid_max_moment(semiaxes, 0)
      II1 = _ellipsoid_max_moment(semiaxes, 1)
      II2 = _ellipsoid_max_moment(semiaxes, 2)

      mom_visc = wp.vec3(
        l_ang[0] * (ang_drag_coef * II0 + slender_drag_coef * (I_max - II0)),
        l_ang[1] * (ang_drag_coef * II1 + slender_drag_coef * (I_max - II1)),
        l_ang[2] * (ang_drag_coef * II2 + slender_drag_coef * (I_max - II2)),
      )

      drag_lin_coef = viscosity * lin_visc_force_coef + density * lin_speed * (
        A_proj * blunt_drag_coef + slender_drag_coef * (A_max - A_proj)
      )
      drag_ang_coef = viscosity * lin_visc_torq_coef + density * wp.length(mom_visc)

      lfrc_torque -= drag_ang_coef * l_ang
      lfrc_force += magnus_force + kutta_force - drag_lin_coef * l_lin

      lfrc_torque *= coef
      lfrc_force *= coef

      # map force/torque from local to world frame: lfrc -> bfrc
      torque_global += geom_rot @ lfrc_torque
      force_global += geom_rot @ lfrc_force

    fluid_applied_out[worldid, bodyid] = wp.spatial_vector(force_global, torque_global)
    return

  l_ang = rotT @ ang_global
  l_lin = rotT @ lin_com

  if wind[0] or wind[1] or wind[2]:
    l_lin -= rotT @ wind

  lfrc_torque = wp.vec3(0.0)
  lfrc_force = wp.vec3(0.0)

  has_viscosity = viscosity > 0.0
  has_density = density > 0.0

  if has_viscosity or has_density:
    inertia = body_inertia[worldid % body_inertia.shape[0], bodyid]
    mass = body_mass[worldid % body_mass.shape[0], bodyid]
    scl = 6.0 / mass
    box0 = wp.sqrt(wp.max(MJ_MINVAL, inertia[1] + inertia[2] - inertia[0]) * scl)
    box1 = wp.sqrt(wp.max(MJ_MINVAL, inertia[0] + inertia[2] - inertia[1]) * scl)
    box2 = wp.sqrt(wp.max(MJ_MINVAL, inertia[0] + inertia[1] - inertia[2]) * scl)

  if has_viscosity:
    diam = (box0 + box1 + box2) / 3.0
    lfrc_torque = -l_ang * wp.pow(diam, 3.0) * wp.pi * viscosity
    lfrc_force = -3.0 * l_lin * diam * wp.pi * viscosity

  if has_density:
    lfrc_force -= wp.vec3(
      0.5 * density * box1 * box2 * wp.abs(l_lin[0]) * l_lin[0],
      0.5 * density * box0 * box2 * wp.abs(l_lin[1]) * l_lin[1],
      0.5 * density * box0 * box1 * wp.abs(l_lin[2]) * l_lin[2],
    )

    scl = density / 64.0
    box0_pow4 = wp.pow(box0, 4.0)
    box1_pow4 = wp.pow(box1, 4.0)
    box2_pow4 = wp.pow(box2, 4.0)
    lfrc_torque -= wp.vec3(
      box0 * (box1_pow4 + box2_pow4) * wp.abs(l_ang[0]) * l_ang[0] * scl,
      box1 * (box0_pow4 + box2_pow4) * wp.abs(l_ang[1]) * l_ang[1] * scl,
      box2 * (box0_pow4 + box1_pow4) * wp.abs(l_ang[2]) * l_ang[2] * scl,
    )

  torque_global = rot @ lfrc_torque
  force_global = rot @ lfrc_force

  fluid_applied_out[worldid, bodyid] = wp.spatial_vector(force_global, torque_global)


def _fluid(m: Model, d: Data):
  fluid_applied = wp.empty((d.nworld, m.nbody), dtype=wp.spatial_vector)

  wp.launch(
    _fluid_force,
    dim=(d.nworld, m.nbody),
    inputs=[
      m.opt.density,
      m.opt.viscosity,
      m.opt.wind,
      m.body_rootid,
      m.body_geomnum,
      m.body_geomadr,
      m.body_mass,
      m.body_inertia,
      m.geom_type,
      m.geom_size,
      m.geom_fluid,
      m.body_fluid_ellipsoid,
      d.xipos,
      d.ximat,
      d.geom_xpos,
      d.geom_xmat,
      d.subtree_com,
      d.cvel,
    ],
    outputs=[fluid_applied],
  )

  support.apply_ft(m, d, fluid_applied, d.qfrc_fluid, False)


@wp.kernel
def _qfrc_passive(
  # Model:
  opt_has_fluid: bool,
  jnt_actgravcomp: wp.array(dtype=int),
  dof_jntid: wp.array(dtype=int),
  # Data in:
  qfrc_spring_in: wp.array2d(dtype=float),
  qfrc_damper_in: wp.array2d(dtype=float),
  qfrc_gravcomp_in: wp.array2d(dtype=float),
  qfrc_fluid_in: wp.array2d(dtype=float),
  # In:
  gravcomp: bool,
  # Data out:
  qfrc_passive_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()
  qfrc_passive = qfrc_spring_in[worldid, dofid]
  qfrc_passive += qfrc_damper_in[worldid, dofid]

  # add gravcomp unless added by actuators
  if gravcomp and not jnt_actgravcomp[dof_jntid[dofid]]:
    qfrc_passive += qfrc_gravcomp_in[worldid, dofid]

  # add fluid force
  if opt_has_fluid:
    qfrc_passive += qfrc_fluid_in[worldid, dofid]

  qfrc_passive_out[worldid, dofid] = qfrc_passive


@wp.kernel
def _flex_elasticity(
  # Model:
  nflex: int,
  opt_timestep: wp.array(dtype=float),
  body_dofadr: wp.array(dtype=int),
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_edgeadr: wp.array(dtype=int),
  flex_elemadr: wp.array(dtype=int),
  flex_elemnum: wp.array(dtype=int),
  flex_elemedgeadr: wp.array(dtype=int),
  flex_vertbodyid: wp.array(dtype=int),
  flex_elem: wp.array(dtype=int),
  flex_elemedge: wp.array(dtype=int),
  flexedge_length0: wp.array(dtype=float),
  flex_stiffness: wp.array2d(dtype=float),
  flex_damping: wp.array(dtype=float),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  flexedge_length_in: wp.array2d(dtype=float),
  flexedge_velocity_in: wp.array2d(dtype=float),
  # In:
  dsbl_damper: bool,
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
):
  worldid, elemid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]

  for i in range(nflex):
    locid = elemid - flex_elemadr[i]
    if locid >= 0 and locid < flex_elemnum[i]:
      f = i
      break

  dim = flex_dim[f]
  nvert = dim + 1
  nedge = nvert * (nvert - 1) / 2
  edges = wp.where(
    dim == 3,
    wp.types.matrix(0, 1, 1, 2, 2, 0, 2, 3, 0, 3, 1, 3, shape=(6, 2), dtype=int),
    wp.types.matrix(1, 2, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, shape=(6, 2), dtype=int),
  )
  if timestep > 0.0 and not dsbl_damper:
    kD = flex_damping[f] / timestep
  else:
    kD = 0.0

  gradient = wp.types.matrix(0.0, shape=(6, 6))
  for e in range(nedge):
    vert0 = flex_elem[(dim + 1) * elemid + edges[e, 0]]
    vert1 = flex_elem[(dim + 1) * elemid + edges[e, 1]]
    xpos0 = flexvert_xpos_in[worldid, vert0]
    xpos1 = flexvert_xpos_in[worldid, vert1]
    for i in range(3):
      gradient[e, 0 + i] = xpos0[i] - xpos1[i]
      gradient[e, 3 + i] = xpos1[i] - xpos0[i]

  elongation = wp.spatial_vectorf(0.0)
  for e in range(nedge):
    idx = flex_elemedge[elemid * nedge + e]
    vel = flexedge_velocity_in[worldid, flex_edgeadr[f] + idx]
    deformed = flexedge_length_in[worldid, flex_edgeadr[f] + idx]
    reference = flexedge_length0[flex_edgeadr[f] + idx]
    previous = deformed - vel * timestep
    elongation[e] = deformed * deformed - reference * reference + (deformed * deformed - previous * previous) * kD

  metric = wp.types.matrix(0.0, shape=(6, 6))
  id = int(0)
  for ed1 in range(nedge):
    for ed2 in range(ed1, nedge):
      metric[ed1, ed2] = flex_stiffness[elemid, id]
      metric[ed2, ed1] = flex_stiffness[elemid, id]
      id += 1

  force = wp.types.matrix(0.0, shape=(6, 3))
  for ed1 in range(nedge):
    for ed2 in range(nedge):
      for i in range(2):
        for x in range(3):
          force[edges[ed2, i], x] -= elongation[ed1] * gradient[ed2, 3 * i + x] * metric[ed1, ed2]

  for v in range(nvert):
    vert = flex_elem[(dim + 1) * elemid + v]
    bodyid = flex_vertbodyid[flex_vertadr[f] + vert]
    for x in range(3):
      wp.atomic_add(qfrc_spring_out, worldid, body_dofadr[bodyid] + x, force[v, x])


@wp.kernel
def _flex_bending(
  # Model:
  nflex: int,
  body_dofadr: wp.array(dtype=int),
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_edgeadr: wp.array(dtype=int),
  flex_edgenum: wp.array(dtype=int),
  flex_vertbodyid: wp.array(dtype=int),
  flex_edge: wp.array(dtype=wp.vec2i),
  flex_edgeflap: wp.array(dtype=wp.vec2i),
  flex_bending: wp.array2d(dtype=float),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
):
  worldid, edgeid = wp.tid()
  nvert = 4

  for i in range(nflex):
    locid = edgeid - flex_edgeadr[i]
    if locid >= 0 and locid < flex_edgenum[i]:
      f = i
      break

  if flex_dim[f] != 2:
    return

  if flex_edgeflap[edgeid][1] == -1:
    return

  v = wp.vec4i(
    flex_vertadr[f] + flex_edge[edgeid][0],
    flex_vertadr[f] + flex_edge[edgeid][1],
    flex_vertadr[f] + flex_edgeflap[edgeid][0],
    flex_vertadr[f] + flex_edgeflap[edgeid][1],
  )

  frc = wp.types.matrix(0.0, shape=(4, 3))
  if flex_bending[edgeid, 16]:
    v0 = flexvert_xpos_in[worldid, v[0]]
    v1 = flexvert_xpos_in[worldid, v[1]]
    v2 = flexvert_xpos_in[worldid, v[2]]
    v3 = flexvert_xpos_in[worldid, v[3]]
    frc[1] = wp.cross(v2 - v0, v3 - v0)
    frc[2] = wp.cross(v3 - v0, v1 - v0)
    frc[3] = wp.cross(v1 - v0, v2 - v0)
    frc[0] = -(frc[1] + frc[2] + frc[3])

  force = wp.types.matrix(0.0, shape=(nvert, 3))
  for i in range(nvert):
    for x in range(3):
      for j in range(nvert):
        force[i, x] -= flex_bending[edgeid, 4 * i + j] * flexvert_xpos_in[worldid, v[j]][x]
    force[i, x] -= flex_bending[edgeid, 16] * frc[i, x]

  for i in range(nvert):
    bodyid = flex_vertbodyid[flex_vertadr[f] + v[i]]
    for x in range(3):
      wp.atomic_add(qfrc_spring_out, worldid, body_dofadr[bodyid] + x, force[i, x])


@event_scope
def passive(m: Model, d: Data):
  """Adds all passive forces."""
  dsbl_spring = m.opt.disableflags & DisableBit.SPRING
  dsbl_damper = m.opt.disableflags & DisableBit.DAMPER

  if dsbl_spring and dsbl_damper:
    d.qfrc_spring.zero_()
    d.qfrc_damper.zero_()
    d.qfrc_gravcomp.zero_()
    d.qfrc_fluid.zero_()
    d.qfrc_passive.zero_()
    return

  wp.launch(
    _spring_damper_dof_passive,
    dim=(d.nworld, m.njnt),
    inputs=[
      m.opt.disableflags,
      m.qpos_spring,
      m.jnt_type,
      m.jnt_qposadr,
      m.jnt_dofadr,
      m.jnt_stiffness,
      m.dof_damping,
      d.qpos,
      d.qvel,
    ],
    outputs=[d.qfrc_spring, d.qfrc_damper],
  )

  if m.ntendon:
    wp.launch(
      _spring_damper_tendon_passive,
      dim=(d.nworld, m.ntendon, m.nv),
      inputs=[
        m.tendon_stiffness,
        m.tendon_damping,
        m.tendon_lengthspring,
        d.ten_J,
        d.ten_length,
        d.ten_velocity,
        dsbl_spring,
        dsbl_damper,
      ],
      outputs=[
        d.qfrc_spring,
        d.qfrc_damper,
      ],
    )

  if not dsbl_spring:
    wp.launch(
      _flex_elasticity,
      dim=(d.nworld, m.nflexelem),
      inputs=[
        m.nflex,
        m.opt.timestep,
        m.body_dofadr,
        m.flex_dim,
        m.flex_vertadr,
        m.flex_edgeadr,
        m.flex_elemadr,
        m.flex_elemnum,
        m.flex_elemedgeadr,
        m.flex_vertbodyid,
        m.flex_elem,
        m.flex_elemedge,
        m.flexedge_length0,
        m.flex_stiffness,
        m.flex_damping,
        d.flexvert_xpos,
        d.flexedge_length,
        d.flexedge_velocity,
        dsbl_damper,
      ],
      outputs=[d.qfrc_spring],
    )
  wp.launch(
    _flex_bending,
    dim=(d.nworld, m.nflexedge),
    inputs=[
      m.nflex,
      m.body_dofadr,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_edgeadr,
      m.flex_edgenum,
      m.flex_vertbodyid,
      m.flex_edge,
      m.flex_edgeflap,
      m.flex_bending,
      d.flexvert_xpos,
    ],
    outputs=[d.qfrc_spring],
  )

  gravcomp = m.ngravcomp and not (m.opt.disableflags & DisableBit.GRAVITY)

  if gravcomp:
    d.qfrc_gravcomp.zero_()
    wp.launch(
      _gravity_force,
      dim=(d.nworld, m.nbody - 1, m.nv),
      inputs=[
        m.opt.gravity,
        m.body_parentid,
        m.body_rootid,
        m.body_mass,
        m.body_gravcomp,
        m.dof_bodyid,
        d.xipos,
        d.subtree_com,
        d.cdof,
      ],
      outputs=[d.qfrc_gravcomp],
    )

  if m.opt.has_fluid:
    _fluid(m, d)

  wp.launch(
    _qfrc_passive,
    dim=(d.nworld, m.nv),
    inputs=[
      m.opt.has_fluid,
      m.jnt_actgravcomp,
      m.dof_jntid,
      d.qfrc_spring,
      d.qfrc_damper,
      d.qfrc_gravcomp,
      d.qfrc_fluid,
      gravcomp,
    ],
    outputs=[
      d.qfrc_passive,
    ],
  )
