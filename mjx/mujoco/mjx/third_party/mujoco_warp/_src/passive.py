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
from mujoco.mjx.third_party.mujoco_warp._src.types import JointType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _spring_damper_dof_passive(
  # Model:
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
  stiffness = jnt_stiffness[worldid, jntid]
  damping = dof_damping[worldid, dofid]

  has_stiffness = stiffness != 0.0
  has_damping = damping != 0.0

  if not has_stiffness:
    qfrc_spring_out[worldid, dofid] = 0.0

  if not has_damping:
    qfrc_damper_out[worldid, dofid] = 0.0

  if not (has_stiffness or has_damping):
    return

  jnttype = jnt_type[jntid]
  qposid = jnt_qposadr[jntid]

  if jnttype == wp.static(JointType.FREE.value):
    # spring
    if has_stiffness:
      dif = wp.vec3(
        qpos_in[worldid, qposid + 0] - qpos_spring[worldid, qposid + 0],
        qpos_in[worldid, qposid + 1] - qpos_spring[worldid, qposid + 1],
        qpos_in[worldid, qposid + 2] - qpos_spring[worldid, qposid + 2],
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
        qpos_spring[worldid, qposid + 3],
        qpos_spring[worldid, qposid + 4],
        qpos_spring[worldid, qposid + 5],
        qpos_spring[worldid, qposid + 6],
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
  elif jnttype == wp.static(JointType.BALL.value):
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
        qpos_spring[worldid, qposid + 0],
        qpos_spring[worldid, qposid + 1],
        qpos_spring[worldid, qposid + 2],
        qpos_spring[worldid, qposid + 3],
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
      fdif = qpos_in[worldid, qposid] - qpos_spring[worldid, qposid]
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
  ten_velocity_in: wp.array2d(dtype=float),
  ten_length_in: wp.array2d(dtype=float),
  ten_J_in: wp.array3d(dtype=float),
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
  qfrc_damper_out: wp.array2d(dtype=float),
):
  worldid, tenid, dofid = wp.tid()

  stiffness = tendon_stiffness[worldid, tenid]
  damping = tendon_damping[worldid, tenid]

  if stiffness == 0.0 and damping == 0.0:
    return

  J = ten_J_in[worldid, tenid, dofid]

  if stiffness:
    # compute spring force along tendon
    length = ten_length_in[worldid, tenid]
    lengthspring = tendon_lengthspring[worldid, tenid]
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

  if damping:
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
  gravcomp = body_gravcomp[worldid, bodyid]
  gravity = opt_gravity[worldid]

  if gravcomp:
    force = -gravity * body_mass[worldid, bodyid] * gravcomp

    pos = xipos_in[worldid, bodyid]
    jac, _ = support.jac(body_parentid, body_rootid, dof_bodyid, subtree_com_in, cdof_in, pos, bodyid, dofid, worldid)

    wp.atomic_add(qfrc_gravcomp_out[worldid], dofid, wp.dot(jac, force))


@wp.kernel
def _box_fluid(
  # Model:
  opt_wind: wp.array(dtype=wp.vec3),
  opt_density: wp.array(dtype=float),
  opt_viscosity: wp.array(dtype=float),
  body_rootid: wp.array(dtype=int),
  body_mass: wp.array2d(dtype=float),
  body_inertia: wp.array2d(dtype=wp.vec3),
  # Data in:
  xipos_in: wp.array2d(dtype=wp.vec3),
  ximat_in: wp.array2d(dtype=wp.mat33),
  subtree_com_in: wp.array2d(dtype=wp.vec3),
  cvel_in: wp.array2d(dtype=wp.spatial_vector),
  # Data out:
  fluid_applied_out: wp.array2d(dtype=wp.spatial_vector),
):
  """Fluid forces based on inertia-box approximation."""

  worldid, bodyid = wp.tid()
  wind = opt_wind[worldid]
  density = opt_density[worldid]
  viscosity = opt_viscosity[worldid]

  # map from CoM-centered to local body-centered 6D velocity

  # body-inertial
  pos = xipos_in[worldid, bodyid]
  rot = ximat_in[worldid, bodyid]
  rotT = wp.transpose(rot)

  # transform velocity
  cvel = cvel_in[worldid, bodyid]
  torque = wp.spatial_top(cvel)
  force = wp.spatial_bottom(cvel)
  subtree_com = subtree_com_in[worldid, body_rootid[bodyid]]
  dif = pos - subtree_com
  force -= wp.cross(dif, torque)

  lvel_torque = rotT @ torque
  lvel_force = rotT @ force

  if wind[0] or wind[1] or wind[2]:
    # subtract translational component from body velocity
    lvel_force -= rotT @ wind

  lfrc_torque = wp.vec3(0.0)
  lfrc_force = wp.vec3(0.0)

  has_viscosity = viscosity > 0.0
  has_density = density > 0.0

  if has_viscosity or has_density:
    inertia = body_inertia[worldid, bodyid]
    mass = body_mass[worldid, bodyid]
    scl = 6.0 / mass
    box0 = wp.sqrt(wp.max(MJ_MINVAL, inertia[1] + inertia[2] - inertia[0]) * scl)
    box1 = wp.sqrt(wp.max(MJ_MINVAL, inertia[0] + inertia[2] - inertia[1]) * scl)
    box2 = wp.sqrt(wp.max(MJ_MINVAL, inertia[0] + inertia[1] - inertia[2]) * scl)

  if has_viscosity:
    # diameter of sphere approximation
    diam = (box0 + box1 + box2) / 3.0

    # angular viscosity
    lfrc_torque = -lvel_torque * wp.pow(diam, 3.0) * wp.pi * viscosity

    # linear viscosity
    lfrc_force = -3.0 * lvel_force * diam * wp.pi * viscosity

  if has_density:
    # force
    lfrc_force -= wp.vec3(
      0.5 * density * box1 * box2 * wp.abs(lvel_force[0]) * lvel_force[0],
      0.5 * density * box0 * box2 * wp.abs(lvel_force[1]) * lvel_force[1],
      0.5 * density * box0 * box1 * wp.abs(lvel_force[2]) * lvel_force[2],
    )

    # torque
    scl = density / 64.0
    box0_pow4 = wp.pow(box0, 4.0)
    box1_pow4 = wp.pow(box1, 4.0)
    box2_pow4 = wp.pow(box2, 4.0)
    lfrc_torque -= wp.vec3(
      box0 * (box1_pow4 + box2_pow4) * wp.abs(lvel_torque[0]) * lvel_torque[0] * scl,
      box1 * (box0_pow4 + box2_pow4) * wp.abs(lvel_torque[1]) * lvel_torque[1] * scl,
      box2 * (box0_pow4 + box1_pow4) * wp.abs(lvel_torque[2]) * lvel_torque[2] * scl,
    )

  # rotate to global orientation: lfrc -> bfrc
  torque = rot @ lfrc_torque
  force = rot @ lfrc_force

  fluid_applied_out[worldid, bodyid] = wp.spatial_vector(force, torque)


def _fluid(m: Model, d: Data):
  wp.launch(
    _box_fluid,
    dim=(d.nworld, m.nbody),
    inputs=[
      m.opt.wind,
      m.opt.density,
      m.opt.viscosity,
      m.body_rootid,
      m.body_mass,
      m.body_inertia,
      d.xipos,
      d.ximat,
      d.subtree_com,
      d.cvel,
    ],
    outputs=[
      d.fluid_applied,
    ],
  )

  # TODO(team): ellipsoid fluid model

  support.apply_ft(m, d, d.fluid_applied, d.qfrc_fluid, False)


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
  opt_timestep: wp.array(dtype=float),
  body_dofadr: wp.array(dtype=int),
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_edgeadr: wp.array(dtype=int),
  flex_elemedgeadr: wp.array(dtype=int),
  flex_vertbodyid: wp.array(dtype=int),
  flex_elem: wp.array(dtype=int),
  flex_elemedge: wp.array(dtype=int),
  flexedge_length0: wp.array(dtype=float),
  flex_stiffness: wp.array(dtype=float),
  flex_damping: wp.array(dtype=float),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  flexedge_length_in: wp.array2d(dtype=float),
  flexedge_velocity_in: wp.array2d(dtype=float),
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
):
  worldid, elemid = wp.tid()
  timestep = opt_timestep[worldid]
  f = 0  # TODO(quaglino): this should become a function of t

  dim = flex_dim[f]
  nvert = dim + 1
  nedge = nvert * (nvert - 1) / 2
  edges = wp.where(
    dim == 3,
    wp.mat(0, 1, 1, 2, 2, 0, 2, 3, 0, 3, 1, 3, shape=(6, 2), dtype=int),
    wp.mat(1, 2, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, shape=(6, 2), dtype=int),
  )
  kD = flex_damping[f] / timestep

  gradient = wp.mat(0.0, shape=(6, 6))
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
    idx = flex_elemedge[flex_elemedgeadr[f] + elemid * nedge + e]
    vel = flexedge_velocity_in[worldid, flex_edgeadr[f] + idx]
    deformed = flexedge_length_in[worldid, flex_edgeadr[f] + idx]
    reference = flexedge_length0[flex_edgeadr[f] + idx]
    previous = deformed - vel * timestep
    elongation[e] = deformed * deformed - reference * reference + (deformed * deformed - previous * previous) * kD

  metric = wp.mat(0.0, shape=(6, 6))
  id = int(0)
  for ed1 in range(nedge):
    for ed2 in range(ed1, nedge):
      metric[ed1, ed2] = flex_stiffness[21 * elemid + id]
      metric[ed2, ed1] = flex_stiffness[21 * elemid + id]
      id += 1

  force = wp.mat(0.0, shape=(6, 3))
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
  body_dofadr: wp.array(dtype=int),
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_edgeadr: wp.array(dtype=int),
  flex_vertbodyid: wp.array(dtype=int),
  flex_edge: wp.array(dtype=wp.vec2i),
  flex_edgeflap: wp.array(dtype=wp.vec2i),
  flex_bending: wp.array(dtype=float),
  # Data in:
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  # Data out:
  qfrc_spring_out: wp.array2d(dtype=float),
):
  worldid, edgeid = wp.tid()
  nvert = 4
  f = 0  # TODO(quaglino): this should become a function of t

  if flex_dim[f] != 2:
    return

  v = wp.vec4i(
    flex_edge[edgeid + flex_edgeadr[f]][0],
    flex_edge[edgeid + flex_edgeadr[f]][1],
    flex_edgeflap[edgeid + flex_edgeadr[f]][0],
    flex_edgeflap[edgeid + flex_edgeadr[f]][1],
  )

  if v[3] == -1:
    return

  frc = wp.mat(0.0, shape=(4, 3))
  if flex_bending[17 * edgeid + 16]:
    v0 = flexvert_xpos_in[worldid, v[0]]
    v1 = flexvert_xpos_in[worldid, v[1]]
    v2 = flexvert_xpos_in[worldid, v[2]]
    v3 = flexvert_xpos_in[worldid, v[3]]
    frc[1] = wp.cross(v2 - v0, v3 - v0)
    frc[2] = wp.cross(v3 - v0, v1 - v0)
    frc[3] = wp.cross(v1 - v0, v2 - v0)
    frc[0] = -(frc[1] + frc[2] + frc[3])

  force = wp.mat(0.0, shape=(nvert, 3))
  for i in range(nvert):
    for x in range(3):
      for j in range(nvert):
        force[i, x] -= flex_bending[17 * edgeid + 4 * i + j] * flexvert_xpos_in[worldid, v[j]][x]
    force[i, x] -= flex_bending[17 * edgeid + 16] * frc[i, x]

  for i in range(nvert):
    bodyid = flex_vertbodyid[flex_vertadr[f] + v[i]]
    for x in range(3):
      wp.atomic_add(qfrc_spring_out, worldid, body_dofadr[bodyid] + x, force[i, x])


@event_scope
def passive(m: Model, d: Data):
  """Adds all passive forces."""

  if m.opt.disableflags & (DisableBit.SPRING | DisableBit.DAMPER):
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
        d.ten_velocity,
        d.ten_length,
        d.ten_J,
      ],
      outputs=[
        d.qfrc_spring,
        d.qfrc_damper,
      ],
    )

  wp.launch(
    _flex_elasticity,
    dim=(d.nworld, m.nflexelem),
    inputs=[
      m.opt.timestep,
      m.body_dofadr,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_edgeadr,
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
    ],
    outputs=[d.qfrc_spring],
  )
  wp.launch(
    _flex_bending,
    dim=(d.nworld, m.nflexedge),
    inputs=[
      m.body_dofadr,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_edgeadr,
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
