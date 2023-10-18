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
"""Core non-smooth constraint functions."""

from typing import Tuple

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import math
from mujoco.mjx._src import scan
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.dataclasses import PyTreeNode
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import EqType
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np


class _Efc(PyTreeNode):
  J: jax.Array
  R: jax.Array
  aref: jax.Array
  frictionloss: jax.Array

  @classmethod
  def zero(cls, m: Model) -> '_Efc':
    z = jp.empty((0,))
    return _Efc(J=jp.empty((0, m.nv)), R=z, aref=z, frictionloss=z)


def _kbi(
    m: Model,
    solref: jax.Array,
    solimp: jax.Array,
    pos: jax.Array,
) -> Tuple[jax.Array, jax.Array, jax.Array]:
  """Calculates stiffness, damping, and impedance of a constraint."""
  timeconst, dampratio = solref

  if not m.opt.disableflags & DisableBit.REFSAFE:
    timeconst = jp.maximum(timeconst, 2 * m.opt.timestep) * (timeconst > 0)

  dmin, dmax, width, mid, power = solimp

  dmin = jp.clip(dmin, mujoco.mjMINIMP, mujoco.mjMAXIMP)
  dmax = jp.clip(dmax, mujoco.mjMINIMP, mujoco.mjMAXIMP)
  width = jp.maximum(0, width)
  mid = jp.clip(mid, mujoco.mjMINIMP, mujoco.mjMAXIMP)
  power = jp.maximum(1, power)

  # See https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters
  k = 1 / (dmax * dmax * timeconst * timeconst * dampratio * dampratio)
  b = 2 / (dmax * timeconst)
  # TODO(robotics-simulation): check various solparam settings in model gen test
  k = jp.where(dampratio <= 0, -dampratio / (dmax * dmax), k)
  b = jp.where(timeconst <= 0, -timeconst / dmax, b)

  imp_x = jp.abs(pos) / width
  imp_a = (1.0 / jp.power(mid, power - 1)) * jp.power(imp_x, power)
  imp_b = 1 - (1.0 / jp.power(1 - mid, power - 1)) * jp.power(1 - imp_x, power)
  imp_y = jp.where(imp_x < mid, imp_a, imp_b)
  imp = dmin + imp_y * (dmax - dmin)
  imp = jp.clip(imp, dmin, dmax)
  imp = jp.where(imp_x > 1.0, dmax, imp)

  return k, b, imp  # corresponds to K, B, I of efc_KBIP


def _instantiate_connect(m: Model, d: Data) -> _Efc:
  """Returns jacobians and supporting data for connect equality constraints."""

  if (m.opt.disableflags & DisableBit.EQUALITY) or m.neq == 0:
    return _Efc.zero(m)

  connect_id = np.nonzero(m.eq_type == EqType.CONNECT)[0]

  if connect_id.size == 0:
    return _Efc.zero(m)

  body1id, body2id = m.eq_obj1id[connect_id], m.eq_obj2id[connect_id]
  data = m.eq_data[connect_id]
  solref, solimp = m.eq_solref[connect_id], m.eq_solimp[connect_id]

  def fn(data, id1, id2, solref, solimp):
    anchor1, anchor2 = data[0:3], data[3:6]
    # find global points
    pos1 = d.xmat[id1] @ anchor1 + d.xpos[id1]
    pos2 = d.xmat[id2] @ anchor2 + d.xpos[id2]

    # compute position error
    cpos = pos1 - pos2

    # compute Jacobian difference (opposite of contact: 0 - 1)
    jacp1, _ = support.jac(m, d, pos1, id1)
    jacp2, _ = support.jac(m, d, pos2, id2)
    j = (jacp1 - jacp2).T

    # impedance, inverse constraint mass, reference acceleration
    k, b, imp = _kbi(m, solref, solimp, math.norm(cpos))
    invweight = m.body_invweight0[id1, 0] + m.body_invweight0[id2, 0]
    r = jp.maximum(invweight * (1 - imp) / imp, mujoco.mjMINVAL).repeat(3)
    aref = -b * (j @ d.qvel) - k * imp * cpos

    return _Efc(J=j, R=r, aref=aref, frictionloss=jp.zeros_like(r))

  efcs = jax.vmap(fn)(data, body1id, body2id, solref, solimp)

  return jax.tree_map(jp.concatenate, efcs)


def _instantiate_weld(m: Model, d: Data) -> _Efc:
  """Returns jacobians and supporting data for connect weld constraints."""

  if (m.opt.disableflags & DisableBit.EQUALITY) or m.neq == 0:
    return _Efc.zero(m)

  weld_id = np.nonzero(m.eq_type == EqType.WELD)[0]

  if weld_id.size == 0:
    return _Efc.zero(m)

  body1id, body2id = m.eq_obj1id[weld_id], m.eq_obj2id[weld_id]
  data = m.eq_data[weld_id]
  solref, solimp = m.eq_solref[weld_id], m.eq_solimp[weld_id]

  def fn(data, id1, id2, solref, solimp):
    anchor1, anchor2 = data[0:3], data[3:6]
    relpose, torquescale = data[6:10], data[10]

    # find global points
    pos1 = d.xmat[id1] @ anchor2 + d.xpos[id1]
    pos2 = d.xmat[id2] @ anchor1 + d.xpos[id2]

    # compute position error
    cpos = pos1 - pos2

    # compute Jacobian difference (opposite of contact: 0 - 1)
    jacp1, jacr1 = support.jac(m, d, pos1, id1)
    jacp2, jacr2 = support.jac(m, d, pos2, id2)
    jacdifp = jacp1 - jacp2
    jacdifr = (jacr1 - jacr2) * torquescale

    # compute orientation error: neg(q1) * q0 * relpose (axis components only)
    quat = math.quat_mul(d.xquat[id1], relpose)
    quat1 = math.quat_inv(d.xquat[id2])
    crot = math.quat_mul(quat1, quat)[1:]  # copy axis components

    # correct rotation Jacobian: 0.5 * neg(q1) * (jac0-jac1) * q0 * relpose
    jac_fn = lambda j: math.quat_mul(math.quat_mul_axis(quat1, j), quat)[1:]
    jacdifr = 0.5 * jax.vmap(jac_fn)(jacdifr)

    j = jp.concatenate((jacdifp.T, jacdifr.T))
    pos = jp.concatenate((cpos, crot))

    # impedance, inverse constraint mass, reference acceleration
    k, b, imp = _kbi(m, solref, solimp, math.norm(pos.at[3:].mul(torquescale)))
    invweight = m.body_invweight0[id1] + m.body_invweight0[id2]
    r = jp.maximum(invweight * (1 - imp) / imp, mujoco.mjMINVAL).repeat(3)
    aref = -b * (j @ d.qvel) - k * imp * pos

    return _Efc(J=j, R=r, aref=aref, frictionloss=jp.zeros_like(r))

  efcs = jax.vmap(fn)(data, body1id, body2id, solref, solimp)

  return jax.tree_map(jp.concatenate, efcs)


def _instantiate_friction(m: Model, d: Data) -> _Efc:
  # TODO(robotics-team): implement _instantiate_friction
  del d
  return _Efc.zero(m)


def _instantiate_limit(m: Model, d: Data) -> _Efc:
  """Returns jacobians and supporting data for joint limits."""

  if (m.opt.disableflags & DisableBit.LIMIT) or not m.jnt_limited.any():
    return _Efc.zero(m)

  def fn(jnt_typs, jnt_range, solref, solimp, margin, qpos, dofs, invweight0):
    js, rs, arefs = [], [], []
    qpos_i, dof_i = 0, 0

    for i in range(len(jnt_typs)):
      jnt_typ = JointType(jnt_typs[i])

      if jnt_typ == JointType.FREE:
        return None  # omit constraint rows for free joints
      elif jnt_typ == JointType.BALL:
        axis, angle = math.quat_to_axis_angle(qpos[qpos_i : qpos_i + 4])
        dist = jp.amax(jnt_range[i]) - angle
        j = jp.sum(
            jax.vmap(jp.multiply)(dofs[dof_i : dof_i + 3], -axis), axis=0
        )
      elif jnt_typ in (JointType.HINGE, JointType.SLIDE):
        dist_min = qpos[qpos_i] - jnt_range[i, 0]
        dist_max = jnt_range[i, 1] - qpos[qpos_i]
        dist = jp.minimum(dist_min, dist_max)
        j = dofs[dof_i] * ((dist_min < dist_max) * 2 - 1)
      else:
        raise RuntimeError(f'unrecognized joint type: {jnt_typ}')

      dist = dist - margin[i]
      k, b, imp = _kbi(m, solref[i], solimp[i], dist)
      r = jp.maximum(invweight0[dof_i] * (1 - imp) / imp, mujoco.mjMINVAL)
      aref = -b * (j @ d.qvel) - k * imp * dist
      j, aref = j * (dist < 0), aref * (dist < 0)
      js, rs, arefs = js + [j], rs + [r], arefs + [aref]
      dof_i, qpos_i = dof_i + jnt_typ.dof_width(), qpos_i + jnt_typ.qpos_width()

    return jp.stack(js), jp.stack(rs), jp.stack(arefs)

  j, r, aref = scan.flat(
      m,
      fn,
      'jjjjjqvv',
      'jjj',
      m.jnt_type,
      m.jnt_range,
      m.jnt_solref,
      m.jnt_solimp,
      m.jnt_margin,
      d.qpos,
      jp.eye(m.nv),
      m.dof_invweight0,
  )

  return _Efc(J=j, R=r, aref=aref, frictionloss=jp.zeros_like(r))


def _instantiate_contact(m: Model, d: Data) -> _Efc:
  """Returns jacobians and supporitng data for contacts."""

  if (m.opt.disableflags & DisableBit.CONTACT) or d.ncon == 0:
    return _Efc.zero(m)

  def fn(contact: Contact):
    dist = contact.dist - contact.includemargin
    k, b, imp = _kbi(m, contact.solref, contact.solimp, dist)

    geom_bodyid = jp.array(m.geom_bodyid)
    body1, body2 = geom_bodyid[contact.geom1], geom_bodyid[contact.geom2]
    diff = support.jac_dif_pair(m, d, contact.pos, body1, body2)
    t = m.body_invweight0[body1, 0] + m.body_invweight0[body2, 0]

    # rotate Jacobian differences to contact frame
    diff_con = contact.frame @ diff.T

    # TODO(robotics-simulation): add support for other friction dimensions
    # 4 pyramidal friction directions
    js, rs = [], []
    for diff_tan, friction in zip(diff_con[1:], contact.friction[:2]):
      for f in (friction, -friction):
        js.append(diff_con[0] + diff_tan * f)
        rs.append((t + f * f * t) * 2 * f * f * (1 - imp) / imp)

    j, r = jp.stack(js), jp.stack(rs)
    r = jp.maximum(r, mujoco.mjMINVAL)
    aref = -b * (j @ d.qvel) - k * imp * dist
    mask_fn = jax.vmap(lambda x, mask=(dist < 0): x * mask)
    j, aref = jax.tree_map(mask_fn, (j, aref))

    return _Efc(J=j, R=r, aref=aref, frictionloss=jp.zeros_like(r))

  return jax.tree_map(jp.concatenate, jax.vmap(fn)(d.contact))


def count_constraints(m: Model, d: Data) -> Tuple[int, int, int, int]:
  """Returns equality, friction, limit, and contact constraint counts."""
  if m.opt.disableflags & DisableBit.CONSTRAINT:
    return 0, 0, 0, 0

  if m.opt.disableflags & DisableBit.EQUALITY:
    ne = 0
  else:
    ne_weld = (m.eq_type == EqType.WELD).sum()
    ne_connect = (m.eq_type == EqType.CONNECT).sum()
    ne = ne_weld * 6 + ne_connect * 3

  nf = 0

  if (m.opt.disableflags & DisableBit.LIMIT) or not m.jnt_limited.any():
    nl = 0
  else:
    nl = (m.jnt_type != JointType.FREE).sum()

  if (m.opt.disableflags & DisableBit.CONTACT):
    nc = 0
  else:
    nc = d.ncon * 4

  return ne, nf, nl, nc


def make_constraint(m: Model, d: Data) -> Data:
  """Creates constraint jacobians and other supporting data."""

  ns = sum(count_constraints(m, d)[:-1])
  # TODO(robotics-simulation): make device_put set nefc/efc_address instead
  d = d.tree_replace({'contact.efc_address': np.arange(ns, ns + d.ncon * 4, 4)})

  if m.opt.disableflags & DisableBit.CONSTRAINT:
    efc = _Efc.zero(m)
  else:
    efcs = (
        _instantiate_connect(m, d),
        _instantiate_weld(m, d),
        _instantiate_friction(m, d),
        _instantiate_limit(m, d),
        _instantiate_contact(m, d),
    )
    efc = jax.tree_map(lambda *x: jp.concatenate(x), *efcs)

  d = d.replace(
      efc_J=efc.J,
      efc_D=1 / efc.R,
      efc_aref=efc.aref,
      efc_frictionloss=efc.frictionloss,
      nefc=efc.aref.shape[0],
  )

  return d
