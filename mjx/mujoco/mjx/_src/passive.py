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
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member


def _inertia_box_fluid_model(
    m: Model,
    inertia: jax.Array,
    mass: jax.Array,
    root_com: jax.Array,
    xipos: jax.Array,
    ximat: jax.Array,
    cvel: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Fluid forces based on inertia-box approximation."""
  box = jp.repeat(inertia[None, :], 3, axis=0)
  box *= jp.ones((3, 3)) - 2 * jp.eye(3)
  box = 6.0 * jp.clip(jp.sum(box, axis=-1), a_min=1e-12)
  box = jp.sqrt(box / jp.maximum(mass, 1e-12)) * (mass > 0.0)

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

  return force, torque


def passive(m: Model, d: Data) -> Data:
  """Adds all passive forces."""
  if m.opt.disableflags & DisableBit.PASSIVE:
    return d.replace(qfrc_passive=jp.zeros(m.nv))

  # joint-level springs
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

  qfrc_passive = scan.flat(
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
  qfrc_passive -= m.dof_damping * d.qvel

  # TODO(robotics-simulation): body-level gravity compensation

  # body-level viscosity, lift and drag
  if m.opt.has_fluid_params:
    force, torque = jax.vmap(
        _inertia_box_fluid_model, in_axes=(None, 0, 0, 0, 0, 0, 0)
    )(
        m,
        m.body_inertia,
        m.body_mass,
        d.subtree_com[jp.array(m.body_rootid)],
        d.xipos,
        d.ximat,
        d.cvel,
    )
    qfrc_target = jax.vmap(support.apply_ft, in_axes=(None, None, 0, 0, 0, 0))(
        m, d, force, torque, d.xipos, jp.arange(m.nbody)
    )
    qfrc_passive += jp.sum(qfrc_target, axis=0)

  d = d.replace(qfrc_passive=qfrc_passive)
  return d
