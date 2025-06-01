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
"""Passive forces."""

from typing import Tuple

import jax
from jax import numpy as jp
from mujoco.mjx._src import math
from mujoco.mjx._src import scan
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DataJAX
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import ModelJAX
from mujoco.mjx._src.types import OptionJAX
from mjx_fluid._src.types import GeomType
import mujoco
# pylint: enable=g-importing-member


def _spring_damper(m: Model, d: Data) -> jax.Array:
  """Applies joint level spring and damping forces."""

  def fn(jnt_typs, stiffness, qpos_spring, qpos):
    qpos_i = 0
    qfrcs = []
    for i in range(len(jnt_typs)):
      jnt_typ = JointType(jnt_typs[i])
      q = qpos[qpos_i : qpos_i + jnt_typ.qpos_width()]
      qs = qpos_spring[qpos_i : qpos_i + jnt_typ.qpos_width()]
      qfrc = jp.zeros(jnt_typ.dof_width())
      if jnt_typ == JointType.FREE:
        qfrc = qfrc.at[:3].set(-stiffness[i] * (q[:3] - qs[:3]))
        qfrc = qfrc.at[3:6].set(-stiffness[i] * math.quat_sub(q[3:7], qs[3:7]))
      elif jnt_typ == JointType.BALL:
        qfrc = -stiffness[i] * math.quat_sub(q, qs)
      elif jnt_typ in (
          JointType.SLIDE,
          JointType.HINGE,
      ):
        qfrc = -stiffness[i] * (q - qs)
      else:
        raise RuntimeError(f'unrecognized joint type: {jnt_typ}')
      qfrcs.append(qfrc)
      qpos_i += jnt_typ.qpos_width()
    return jp.concatenate(qfrcs)

  # dof-level springs
  qfrc = scan.flat(
      m,
      fn,
      'jjqq',
      'v',
      m.jnt_type,
      m.jnt_stiffness,
      m.qpos_spring,
      d.qpos,
  )

  # dof-level dampers
  qfrc -= m.dof_damping * d.qvel

  # tendon-level spring-dampers
  below, above = m.tendon_lengthspring.T - d._impl.ten_length
  frc_spring = jp.where(below > 0, m.tendon_stiffness * below, 0)
  frc_spring = jp.where(above < 0, m.tendon_stiffness * above, frc_spring)
  frc_damper = -m.tendon_damping * d._impl.ten_velocity
  qfrc += d._impl.ten_J.T @ (frc_spring + frc_damper)

  return qfrc


def _gravcomp(m: Model, d: Data) -> jax.Array:
  """Applies body-level gravity compensation."""
  force = -m.opt.gravity * (m.body_mass * m.body_gravcomp)[:, None]

  apply_f = lambda f, pos, body_id: support.jac(m, d, pos, body_id)[0] @ f
  qfrc = jax.vmap(apply_f)(force, d.xipos, jp.arange(m.nbody)).sum(axis=0)

  return qfrc


def _fluid(m: Model, d: Data) -> jax.Array:
    """Applies body and geom-level viscosity, lift and drag based on appropriate model type."""

    # Determine which bodies use ellipsoid model (for box model application)
    geom_interaction_coef = m.geom_fluid[:, 0]  # (n_geom,)
    use_ellipsoid_model = jax.ops.segment_max(geom_interaction_coef, m.geom_bodyid, num_segments=m.nbody).astype(bool) # (n_body,)
    use_box_model = jp.logical_and(~use_ellipsoid_model, m.body_mass >= mujoco.mjMINVAL) # (n_body,)

    # Compute Inertia box model forces for bodies
    box_force, box_torque = jax.vmap(
        _inertia_box_fluid_model, in_axes=(None, 0, 0, 0, 0, 0, 0, 0)
    )(
        m,
        m.body_inertia,
        m.body_mass,
        d.subtree_com[jp.array(m.body_rootid)],
        d.xipos,
        d.ximat,
        d.cvel,
        use_box_model,
    )

    # Apply box model forces directly to bodies
    qfrc_box = jax.vmap(support.apply_ft, in_axes=(None, None, 0, 0, 0, 0))(
        m, d, box_force, box_torque, d.xipos, jp.arange(m.nbody)
    )

    # Compute ellipsoid model forces for geoms
    ellipsoid_force, ellipsoid_torque = jax.vmap(
        _ellipsoid_fluid_model, in_axes=(None, None, None, 0, 0, 0, 0, 0, 0, 0)
    )(
        m,
        d.subtree_com[jp.array(m.body_rootid)],
        d.cvel,
        d.geom_xpos,
        d.geom_xmat,
        m.geom_bodyid,
        m.geom_size,
        m.geom_type,
        m.geom_fluid,
        geom_interaction_coef,
    )

    # Apply forces directly to geoms
    qfrc_ellipsoid = jax.vmap(support.apply_ft, in_axes=(None, None, 0, 0, 0, 0))(
        m, d, ellipsoid_force, ellipsoid_torque, d.geom_xpos, m.geom_bodyid
    )

    return jp.sum(qfrc_box, axis=0) + jp.sum(qfrc_ellipsoid, axis=0)


def passive(m: Model, d: Data) -> Data:
  """Adds all passive forces."""
  if m.opt.disableflags & DisableBit.PASSIVE:
    return d.replace(qfrc_passive=jp.zeros(m.nv), qfrc_gravcomp=jp.zeros(m.nv), qfrc_fluid=jp.zeros(m.nv))

  qfrc_passive = _spring_damper(m, d)
  qfrc_gravcomp = jp.zeros(m.nv)
  qfrc_fluid = jp.zeros(m.nv)

  if m.ngravcomp and not m.opt.disableflags & DisableBit.GRAVITY:
    qfrc_gravcomp = _gravcomp(m, d)
    # add gravcomp unless added via actuators
    qfrc_passive += qfrc_gravcomp * (1 - m.jnt_actgravcomp[m.dof_jntid])

  if m.opt.has_fluid_params:  # pytype: disable=attribute-error
    qfrc_fluid = _fluid(m, d)
    qfrc_passive += qfrc_fluid

  d = d.replace(qfrc_passive=qfrc_passive, qfrc_gravcomp=qfrc_gravcomp, qfrc_fluid=qfrc_fluid)
  return d


def _inertia_box_fluid_model(
    m: Model,
    inertia: jax.Array,
    mass: jax.Array,
    root_com: jax.Array,
    xipos: jax.Array,
    ximat: jax.Array,
    cvel: jax.Array,
    use_box_model: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Fluid forces based on inertia-box approximation."""
  box = jp.repeat(inertia[None, :], 3, axis=0)
  box *= jp.ones((3, 3)) - 2 * jp.eye(3)
  box = 6.0 * jp.clip(jp.sum(box, axis=-1), a_min=mujoco.mjMINVAL)
  box = jp.sqrt(box / jp.maximum(mass, mujoco.mjMINVAL)) * (mass > 0.0)

  # transform to local coordinate frame
  offset = xipos - root_com
  lvel = math.transform_motion(cvel, offset, ximat)
  lwind = ximat.T @ m.opt.wind
  lvel = lvel.at[3:].add(-lwind)

  # set viscous force and torque
  diam = jp.mean(box, axis=-1)
  lfrc_ang = lvel[:3] * -jp.pi * diam**3 * m.opt.viscosity
  lfrc_vel = lvel[3:] * -3.0 * jp.pi * diam * m.opt.viscosity

  # add lift and drag force and torque
  scale_vel = jp.array([box[1] * box[2], box[0] * box[2], box[0] * box[1]])
  scale_ang = jp.array([
      box[0] * (box[1] ** 4 + box[2] ** 4),
      box[1] * (box[0] ** 4 + box[2] ** 4),
      box[2] * (box[0] ** 4 + box[1] ** 4),
  ])
  lfrc_vel -= 0.5 * m.opt.density * scale_vel * jp.abs(lvel[3:]) * lvel[3:]
  lfrc_ang -= (
      1.0 * m.opt.density * scale_ang * jp.abs(lvel[:3]) * lvel[:3] / 64.0
  )

  # rotate to global orientation: lfrc -> bfrc
  force, torque = ximat @ lfrc_vel, ximat @ lfrc_ang

  # If use_box_model is False, return zero forces and torques
  force = jp.where(use_box_model, force, jp.zeros(3))
  torque = jp.where(use_box_model, torque, jp.zeros(3))

  return force, torque

def _get_SPHERE_semiaxes(size: jax.Array) -> jax.Array:
  """Returns the semi-axes of the inertia ellipsoid for each geom."""
  return jp.array([size[0], size[0], size[0]])

def _get_CAPSULE_semiaxes(size: jax.Array) -> jax.Array:
  """Returns the semi-axes of the inertia ellipsoid for each geom."""
  return jp.array([size[0], size[0], size[0] * size[1]])

def _get_CYLINDER_semiaxes(size: jax.Array) -> jax.Array:
  """Returns the semi-axes of the inertia ellipsoid for each geom."""
  return jp.array([size[0], size[0], size[1]])

def _get_DEFAULT_semiaxes(size: jax.Array) -> jax.Array:
  """Returns the semi-axes of the inertia ellipsoid for each geom."""
  return size

def _get_geom_semiaxes(geom_size: jax.Array, geom_type: jax.Array) -> jax.Array:
    """Returns the semi-axes of the inertia ellipsoid for each geom."""
    # Convert geometry type to index for the switch
    index = jax.lax.select(
        (geom_type == GeomType.SPHERE),
        0,
        jax.lax.select(
            (geom_type == GeomType.CAPSULE),
            1,
            jax.lax.select(
                (geom_type == GeomType.CYLINDER),
                2,
                3  # Default case
            )
        )
    )
    # Use a single switch statement
    return jax.lax.switch(
        index,
        [
            lambda _: _get_SPHERE_semiaxes(geom_size),
            lambda _: _get_CAPSULE_semiaxes(geom_size),
            lambda _: _get_CYLINDER_semiaxes(geom_size),
            lambda _: _get_DEFAULT_semiaxes(geom_size)
        ],
        None
    )

def _ellipsoid_fluid_model(
    m: Model,
    root_com: jax.Array,  # (nbody, 3)
    body_cvel: jax.Array,  # (nbody, 6)
    geom_xpos: jax.Array,
    geom_xmat: jax.Array,
    geom_bodyid: jax.Array,
    geom_size: jax.Array,
    geom_type: jax.Array,
    geom_fluid: jax.Array,
    use_ellipsoid_model: jax.Array,  # (n_geom,) per geom
):
  """Fluid forces based on inertia-ellipsoid approximation."""

  # Get semi-axes of geometry
  semiaxis = _get_geom_semiaxes(geom_size, geom_type)

  # map from CoM-centered to local body-centered 6D velocity
  offset = geom_xpos - root_com[geom_bodyid]
  lvel = math.transform_motion(body_cvel[geom_bodyid], offset, geom_xmat)
  # Compute wind in local coordinates and subtract translation component
  lwind = geom_xmat.T @ m.opt.wind
  lvel = lvel.at[3:].add(-lwind)

  lfrc = mj_viscousForces(lvel, m.opt.density, m.opt.viscosity, semiaxis, geom_fluid)
  lfrc += mj_addedMassForces(lvel, None, m.opt.density, geom_fluid[6:9],
                              geom_fluid[9:12]) 

  lfrc_vel, lfrc_ang = lfrc[3:], lfrc[:3]
  force, torque = geom_xmat @ lfrc_vel, geom_xmat @ lfrc_ang

  force = jp.where(use_ellipsoid_model, force, jp.zeros(3))
  torque = jp.where(use_ellipsoid_model, torque, jp.zeros(3))

  return force, torque

def mj_addedMassForces(local_vels: jax.Array,
                       local_accels: jax.Array,
                       fluid_density: jax.Array,
                       virtual_mass: jax.Array,
                       virtual_inertia: jax.Array) -> jax.Array:
    local_force = jp.zeros(6)
    lin_vel = local_vels[3:]
    ang_vel = local_vels[:3]
    virtual_lin_mom = fluid_density * virtual_mass * lin_vel
    virtual_ang_mom = fluid_density * virtual_inertia * ang_vel

    if local_accels is not None:
        local_force = local_force.at[:3].add(-fluid_density * virtual_inertia * local_accels[:3])
        local_force = local_force.at[3:].add(-fluid_density * virtual_mass * local_accels[3:])

    added_mass_force = jp.cross(virtual_lin_mom, ang_vel)
    added_mass_torque1 = jp.cross(virtual_lin_mom, lin_vel)
    added_mass_torque2 = jp.cross(virtual_ang_mom, ang_vel)

    local_force = local_force.at[:3].add(added_mass_torque1 + added_mass_torque2)
    local_force = local_force.at[3:].add(added_mass_force)

    return local_force

def ellipsoid_max_moment(size, dir):
    # Extract the primary dimension and the other two dimensions
    d0 = size[dir]
    d1 = size[(dir + 1) % 3]
    d2 = size[(dir + 2) % 3]
    # Compute the moment using the formula:
    # 8/15 * jp.pi * d0 * (max(d1, d2))**4
    return 8/15 * jp.pi * d0 * jp.power(jp.maximum(d1, d2), 4)

def mj_viscousForces(lvel: jax.Array, 
                     fluid_density: jax.Array,
                     fluid_viscosity: jax.Array,
                     geom_size: jax.Array,
                     geom_fluid: jax.Array):
  """Computes viscous forces acting on a body."""
  blunt_drag_coef = geom_fluid[1]
  slender_drag_coef = geom_fluid[2]
  ang_drag_coef = geom_fluid[3]
  kutta_lift_coef = geom_fluid[4]
  magnus_lift_coef = geom_fluid[5]

  ang_vel = lvel[:3]
  lin_vel = lvel[3:]
  volume = 4.0/3.0 * jp.pi * geom_size.prod()
  d_max = jp.max(geom_size)
  d_min = jp.min(geom_size)
  d_mid = jp.sum(geom_size) - d_max - d_min
  A_max = jp.pi * d_max * d_mid

  # Magnus forces
  magnus_force = jp.cross(ang_vel, lin_vel) * magnus_lift_coef * fluid_density * volume   # <-- Verified to be correct

  # the dot product between velocity and the normal to the cross-section that
  # defines the body's projection along velocity is proj_num/sqrt(proj_denom)
  proj_denom = jp.power(geom_size[1] * geom_size[2], 4) * jp.power(lin_vel[0], 2) + \
               jp.power(geom_size[2] * geom_size[0], 4) * jp.power(lin_vel[1], 2) + \
               jp.power(geom_size[0] * geom_size[1], 4) * jp.power(lin_vel[2], 2)
  proj_num = jp.power(geom_size[1] * geom_size[2] * lin_vel[0], 2) + \
             jp.power(geom_size[2] * geom_size[0] * lin_vel[1], 2) + \
             jp.power(geom_size[0] * geom_size[1] * lin_vel[2], 2)
  
  # projected surface in the direction of the velocity
  A_proj = jp.pi * jp.sqrt(proj_denom / jp.maximum(proj_num, mujoco.mjMINVAL))

  # nondimensionalized normal to ellipsoid's projected area in the direction of velocity
  norm = jp.array([
    jp.power(geom_size[1] * geom_size[2], 2) * lin_vel[0],
    jp.power(geom_size[2] * geom_size[0], 2) * lin_vel[1],
    jp.power(geom_size[0] * geom_size[1], 2) * lin_vel[2]
  ])

  # cosine between velocity and normal to the surface
  # Applying normalization since cosine values diverged
  cos_alpha = proj_num / jp.maximum(jp.linalg.norm(lin_vel) * proj_denom, mujoco.mjMINVAL)
  kutta_circ = jp.cross(norm, lin_vel) * kutta_lift_coef * fluid_density * cos_alpha * A_proj
  kutta_force = jp.cross(kutta_circ, lin_vel)

  # viscous force and torque in Stokes flow, analytical for spherical bodies
  eq_sphere_D = 2.0/3.0 * geom_size.sum()
  lin_visc_force_coef = 3.0 * jp.pi * eq_sphere_D
  lin_visc_torque_coef = jp.pi * jp.power(eq_sphere_D, 3)

  # moments of inertia used to compute angular quadratic drag
  I_max = 8.0/15.0 * jp.pi * d_mid * jp.power(d_max, 4)
  II = jp.array([
    ellipsoid_max_moment(geom_size, 0),
    ellipsoid_max_moment(geom_size, 1),
    ellipsoid_max_moment(geom_size, 2)
    ])
  
  mom_visc = ang_vel * (ang_drag_coef*II + slender_drag_coef*(I_max - II))

  # linear plus quadratic
  drag_lin_coef = fluid_viscosity*lin_visc_force_coef + fluid_density*jp.linalg.norm(lin_vel)* \
                  (A_proj*blunt_drag_coef + slender_drag_coef*(A_max - A_proj))
  drag_ang_coef = fluid_viscosity * lin_visc_torque_coef + fluid_density * jp.linalg.norm(mom_visc)

  local_force = jp.zeros(6)
  local_force = local_force.at[:3].set(-drag_ang_coef * ang_vel)
  local_force = local_force.at[3:].set(magnus_force + kutta_force - drag_lin_coef * lin_vel)

  return local_force