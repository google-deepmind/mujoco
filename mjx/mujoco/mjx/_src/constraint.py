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

from typing import Optional, Tuple, Union

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import math
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.dataclasses import PyTreeNode
from mujoco.mjx._src.types import ConeType
from mujoco.mjx._src.types import ConstraintType
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DataJAX
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import EqType
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import ModelJAX
from mujoco.mjx._src.types import ObjType
# pylint: enable=g-importing-member
import numpy as np


class _Efc(PyTreeNode):
  """Support data for creating constraint matrices."""

  J: jax.Array
  pos_aref: jax.Array
  pos_imp: jax.Array
  invweight: jax.Array
  solref: jax.Array
  solimp: jax.Array
  margin: jax.Array
  frictionloss: jax.Array


def _kbi(
    m: Model,
    solref: jax.Array,
    solimp: jax.Array,
    pos: jax.Array,
) -> Tuple[jax.Array, jax.Array, jax.Array]:
  """Calculates stiffness, damping, and impedance of a constraint."""
  timeconst, dampratio = solref

  if not m.opt.disableflags & DisableBit.REFSAFE:
    timeconst = jp.maximum(timeconst, 2 * m.opt.timestep)

  dmin, dmax, width, mid, power = solimp

  dmin = jp.clip(dmin, mujoco.mjMINIMP, mujoco.mjMAXIMP)
  dmax = jp.clip(dmax, mujoco.mjMINIMP, mujoco.mjMAXIMP)
  width = jp.maximum(mujoco.mjMINVAL, width)
  mid = jp.clip(mid, mujoco.mjMINIMP, mujoco.mjMAXIMP)
  power = jp.maximum(1, power)

  # See https://mujoco.readthedocs.io/en/latest/modeling.html#solver-parameters
  k = 1 / (dmax * dmax * timeconst * timeconst * dampratio * dampratio)
  b = 2 / (dmax * timeconst)
  # TODO(robotics-simulation): check various solparam settings in model gen test
  k = jp.where(solref[0] <= 0, -solref[0] / (dmax * dmax), k)
  b = jp.where(solref[1] <= 0, -solref[1] / dmax, b)

  imp_x = jp.abs(pos) / width
  imp_a = (1.0 / jp.power(mid, power - 1)) * jp.power(imp_x, power)
  imp_b = 1 - (1.0 / jp.power(1 - mid, power - 1)) * jp.power(1 - imp_x, power)
  imp_y = jp.where(imp_x < mid, imp_a, imp_b)
  imp = dmin + imp_y * (dmax - dmin)
  imp = jp.clip(imp, dmin, dmax)
  imp = jp.where(imp_x > 1.0, dmax, imp)

  return k, b, imp  # corresponds to K, B, I of efc_KBIP


def _row(j: jax.Array, *args) -> _Efc:
  """Creates an efc row, ensuring args all have same row count."""
  if len(j.shape) < 2:
    return _Efc(j, *args)  # if j isn't batched, ignore

  args = list(args)
  for i, arg in enumerate(args):
    if not arg.shape or arg.shape[0] != j.shape[0]:
      args[i] = jp.tile(arg, (j.shape[0],) + (1,) * (len(arg.shape)))
  return _Efc(j, *args)


def _efc_equality_connect(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for connect equality constraints."""

  eq_id = np.nonzero(m.eq_type == EqType.CONNECT)[0]
  if (m.opt.disableflags & DisableBit.EQUALITY) or eq_id.size == 0:
    return None

  @jax.vmap
  def rows(
      is_site, obj1id, obj2id, body1id, body2id, data, solref, solimp, active
  ):
    anchor1, anchor2 = data[0:3], data[3:6]

    pos1 = d.xmat[body1id] @ anchor1 + d.xpos[body1id]
    pos2 = d.xmat[body2id] @ anchor2 + d.xpos[body2id]

    if m.nsite:
      pos1 = jp.where(is_site, d.site_xpos[obj1id], pos1)
      pos2 = jp.where(is_site, d.site_xpos[obj2id], pos2)

    # error is difference in global positions
    pos = pos1 - pos2

    # compute Jacobian difference (opposite of contact: 0 - 1)
    jacp1, _ = support.jac(m, d, pos1, body1id)
    jacp2, _ = support.jac(m, d, pos2, body2id)
    j = (jacp1 - jacp2).T
    pos_imp = math.norm(pos)
    invweight = m.body_invweight0[body1id, 0] + m.body_invweight0[body2id, 0]
    zero = jp.zeros_like(pos)

    efc = _row(j, pos, pos_imp, invweight, solref, solimp, zero, zero)
    return jax.tree_util.tree_map(lambda x: x * active, efc)

  is_site = m.eq_objtype == ObjType.SITE
  body1id = np.copy(m.eq_obj1id)
  body2id = np.copy(m.eq_obj2id)

  if m.nsite:
    body1id[is_site] = m.site_bodyid[m.eq_obj1id[is_site]]
    body2id[is_site] = m.site_bodyid[m.eq_obj2id[is_site]]

  args = (
      is_site,
      m.eq_obj1id,
      m.eq_obj2id,
      body1id,
      body2id,
      m.eq_data,
      m.eq_solref,
      m.eq_solimp,
      d.eq_active,
  )
  args = jax.tree_util.tree_map(lambda x: x[eq_id], args)
  # concatenate to drop row grouping
  return jax.tree_util.tree_map(jp.concatenate, rows(*args))


def _efc_equality_weld(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for weld equality constraints."""

  eq_id = np.nonzero(m.eq_type == EqType.WELD)[0]
  if (m.opt.disableflags & DisableBit.EQUALITY) or eq_id.size == 0:
    return None

  @jax.vmap
  def rows(
      is_site, obj1id, obj2id, body1id, body2id, data, solref, solimp, active
  ):
    anchor1, anchor2 = data[0:3], data[3:6]
    relpose, torquescale = data[6:10], data[10]

    # error is difference in global position and orientation
    pos1 = d.xmat[body1id] @ anchor2 + d.xpos[body1id]
    pos2 = d.xmat[body2id] @ anchor1 + d.xpos[body2id]

    if m.nsite:
      pos1 = jp.where(is_site, d.site_xpos[obj1id], pos1)
      pos2 = jp.where(is_site, d.site_xpos[obj2id], pos2)

    cpos = pos1 - pos2

    # compute Jacobian difference (opposite of contact: 0 - 1)
    jacp1, jacr1 = support.jac(m, d, pos1, body1id)
    jacp2, jacr2 = support.jac(m, d, pos2, body2id)
    jacdifp = jacp1 - jacp2
    jacdifr = (jacr1 - jacr2) * torquescale

    # compute orientation error: neg(q1) * q0 * relpose (axis components only)
    quat = math.quat_mul(d.xquat[body1id], relpose)
    quat1 = math.quat_inv(d.xquat[body2id])

    if m.nsite:
      quat = jp.where(
          is_site, math.quat_mul(d.xquat[body1id], m.site_quat[obj1id]), quat
      )
      quat1 = jp.where(
          is_site,
          math.quat_inv(math.quat_mul(d.xquat[body2id], m.site_quat[obj2id])),
          quat1,
      )

    crot = math.quat_mul(quat1, quat)[1:]  # copy axis components

    pos = jp.concatenate((cpos, crot * torquescale))

    # correct rotation Jacobian: 0.5 * neg(q1) * (jac0-jac1) * q0 * relpose
    jac_fn = lambda j: math.quat_mul(math.quat_mul_axis(quat1, j), quat)[1:]
    jacdifr = 0.5 * jax.vmap(jac_fn)(jacdifr)
    j = jp.concatenate((jacdifp.T, jacdifr.T))
    pos_imp = math.norm(pos)
    invweight = m.body_invweight0[body1id] + m.body_invweight0[body2id]
    invweight = jp.repeat(invweight, 3, axis=0)
    zero = jp.zeros_like(pos)

    efc = _row(j, pos, pos_imp, invweight, solref, solimp, zero, zero)
    return jax.tree_util.tree_map(lambda x: x * active, efc)

  is_site = m.eq_objtype == ObjType.SITE
  body1id = np.copy(m.eq_obj1id)
  body2id = np.copy(m.eq_obj2id)

  if m.nsite:
    body1id[is_site] = m.site_bodyid[m.eq_obj1id[is_site]]
    body2id[is_site] = m.site_bodyid[m.eq_obj2id[is_site]]

  args = (
      is_site,
      m.eq_obj1id,
      m.eq_obj2id,
      body1id,
      body2id,
      m.eq_data,
      m.eq_solref,
      m.eq_solimp,
      d.eq_active,
  )
  args = jax.tree_util.tree_map(lambda x: x[eq_id], args)
  # concatenate to drop row grouping
  return jax.tree_util.tree_map(jp.concatenate, rows(*args))


def _efc_equality_joint(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for joint equality constraints."""

  eq_id = np.nonzero(m.eq_type == EqType.JOINT)[0]

  if (m.opt.disableflags & DisableBit.EQUALITY) or eq_id.size == 0:
    return None

  @jax.vmap
  def rows(
      obj2id, data, solref, solimp, active, dofadr1, dofadr2, qposadr1, qposadr2
  ):
    pos1, pos2 = d.qpos[qposadr1], d.qpos[qposadr2]
    ref1, ref2 = m.qpos0[qposadr1], m.qpos0[qposadr2]
    dif = (pos2 - ref2) * (obj2id > -1)
    dif_power = jp.power(dif, jp.arange(0, 5))
    pos = pos1 - ref1 - jp.dot(data[:5], dif_power)
    deriv = jp.dot(data[1:5], dif_power[:4] * jp.arange(1, 5)) * (obj2id > -1)

    j = jp.zeros((m.nv)).at[dofadr2].set(-deriv).at[dofadr1].set(1.0)
    invweight = m.dof_invweight0[dofadr1]
    invweight += m.dof_invweight0[dofadr2] * (obj2id > -1)
    zero = jp.zeros_like(pos)

    efc = _row(j, pos, pos, invweight, solref, solimp, zero, zero)
    return jax.tree_util.tree_map(lambda x: x * active, efc)

  args = (m.eq_obj1id, m.eq_obj2id, m.eq_data, m.eq_solref, m.eq_solimp)
  args += (d.eq_active,)
  args = jax.tree_util.tree_map(lambda x: x[eq_id], args)
  dofadr1, dofadr2 = m.jnt_dofadr[args[0]], m.jnt_dofadr[args[1]]
  qposadr1, qposadr2 = m.jnt_qposadr[args[0]], m.jnt_qposadr[args[1]]
  args = args[1:] + (dofadr1, dofadr2, qposadr1, qposadr2)

  return rows(*args)


def _efc_equality_tendon(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for tendon equality constraints."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError(
        '_efc_equality_tendon requires JAX backend implementation.'
    )

  eq_id = np.nonzero(m.eq_type == EqType.TENDON)[0]

  if (m.opt.disableflags & DisableBit.EQUALITY) or eq_id.size == 0:
    return None

  obj1id, obj2id, data, solref, solimp, active = jax.tree_util.tree_map(
      lambda x: x[eq_id],
      (
          m.eq_obj1id,
          m.eq_obj2id,
          m.eq_data,
          m.eq_solref,
          m.eq_solimp,
          d.eq_active,
      ),
  )

  @jax.vmap
  def rows(
      obj2id, data, solref, solimp, invweight, jac1, jac2, pos1, pos2, active
  ):
    dif = pos2 * (obj2id > -1)
    dif_power = jp.power(dif, jp.arange(0, 5))
    pos = pos1 - jp.dot(data[:5], dif_power)
    deriv = jp.dot(data[1:5], dif_power[:4] * jp.arange(1, 5)) * (obj2id > -1)
    j = jac1 + jac2 * -deriv
    zero = jp.zeros_like(pos)

    efc = _row(j, pos, pos, invweight, solref, solimp, zero, zero)
    return jax.tree_util.tree_map(lambda x: x * active, efc)

  inv1, inv2 = m.tendon_invweight0[obj1id], m.tendon_invweight0[obj2id]
  jac1, jac2 = d._impl.ten_J[obj1id], d._impl.ten_J[obj2id]
  pos1 = d._impl.ten_length[obj1id] - m.tendon_length0[obj1id]
  pos2 = d._impl.ten_length[obj2id] - m.tendon_length0[obj2id]
  invweight = inv1 + inv2 * (obj2id > -1)

  return rows(
      obj2id, data, solref, solimp, invweight, jac1, jac2, pos1, pos2, active
  )


def _efc_friction(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for dof frictionloss."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('_efc_friction requires JAX backend implementation.')

  dof_id = np.nonzero(m._impl.dof_hasfrictionloss)[0]
  tendon_id = np.nonzero(m._impl.tendon_hasfrictionloss)[0]

  size = dof_id.size + tendon_id.size
  if (m.opt.disableflags & DisableBit.FRICTIONLOSS) or (size == 0):
    return None

  args_dof = (jp.eye(m.nv), m.dof_frictionloss, m.dof_invweight0, m.dof_solref)
  args_dof += (m.dof_solimp,)
  args_dof = jax.tree_util.tree_map(lambda x: x[dof_id], args_dof)

  args_ten = (d._impl.ten_J, m.tendon_frictionloss, m.tendon_invweight0)
  args_ten += (m.tendon_solref_fri, m.tendon_solimp_fri)
  args_ten = jax.tree_util.tree_map(lambda x: x[tendon_id], args_ten)

  args = jax.tree_util.tree_map(
      lambda *x: jp.concatenate(x), args_dof, args_ten
  )

  @jax.vmap
  def rows(j, frictionloss, invweight, solref, solimp):
    z = jp.zeros_like(frictionloss)
    return _row(j, z, z, invweight, solref, solimp, z, frictionloss)

  return rows(*args)


def _efc_limit_ball(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for ball joint limits."""

  jnt_id = np.nonzero((m.jnt_type == JointType.BALL) & m.jnt_limited)[0]

  if (m.opt.disableflags & DisableBit.LIMIT) or jnt_id.size == 0:
    return None

  @jax.vmap
  def rows(qposadr, dofadr, jnt_range, jnt_margin, solref, solimp):
    axis, angle = math.quat_to_axis_angle(d.qpos[jp.arange(4) + qposadr])
    # ball rotation angle is always positive
    axis, angle = math.normalize_with_norm(axis * angle)
    pos = jp.amax(jnt_range) - angle - jnt_margin
    active = pos < 0
    j = jp.zeros(m.nv).at[jp.arange(3) + dofadr].set(-axis)
    invweight = m.dof_invweight0[dofadr]
    z = jp.zeros_like(pos)

    return _row(
        j * active, pos * active, pos, invweight, solref, solimp, jnt_margin, z
    )

  args = (m.jnt_qposadr, m.jnt_dofadr, m.jnt_range, m.jnt_margin, m.jnt_solref)
  args += (m.jnt_solimp,)
  args = jax.tree_util.tree_map(lambda x: x[jnt_id], args)

  return rows(*args)


def _efc_limit_slide_hinge(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for slide and hinge joint limits."""

  slide_hinge = np.isin(m.jnt_type, (JointType.SLIDE, JointType.HINGE))
  jnt_id = np.nonzero(slide_hinge & m.jnt_limited)[0]

  if (m.opt.disableflags & DisableBit.LIMIT) or jnt_id.size == 0:
    return None

  @jax.vmap
  def rows(qposadr, dofadr, jnt_range, jnt_margin, solref, solimp):
    qpos = d.qpos[qposadr]
    dist_min, dist_max = qpos - jnt_range[0], jnt_range[1] - qpos
    pos = jp.minimum(dist_min, dist_max) - jnt_margin
    active = pos < 0
    j = jp.zeros(m.nv).at[dofadr].set((dist_min < dist_max) * 2 - 1)
    invweight = m.dof_invweight0[dofadr]
    z = jp.zeros_like(pos)

    return _row(
        j * active, pos * active, pos, invweight, solref, solimp, jnt_margin, z
    )

  args = (m.jnt_qposadr, m.jnt_dofadr, m.jnt_range, m.jnt_margin, m.jnt_solref)
  args += (m.jnt_solimp,)
  args = jax.tree_util.tree_map(lambda x: x[jnt_id], args)

  return rows(*args)


def _efc_limit_tendon(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for tendon limits."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('_efc_limit_tendon requires JAX backend implementation.')

  tendon_id = np.nonzero(m.tendon_limited)[0]

  if (m.opt.disableflags & DisableBit.LIMIT) or tendon_id.size == 0:
    return None

  length, j, range_, margin, invweight, solref, solimp = jax.tree_util.tree_map(
      lambda x: x[tendon_id],
      (
          d._impl.ten_length,
          d._impl.ten_J,
          m.tendon_range,
          m.tendon_margin,
          m.tendon_invweight0,
          m.tendon_solref_lim,
          m.tendon_solimp_lim,
      ),
  )

  dist_min, dist_max = length - range_[:, 0], range_[:, 1] - length
  pos = jp.minimum(dist_min, dist_max) - margin
  active = pos < 0
  j = jax.vmap(jp.multiply)(j, ((dist_min < dist_max) * 2 - 1) * active)
  zero = jp.zeros_like(pos)

  return jax.vmap(_row)(
      j, pos * active, pos, invweight, solref, solimp, margin, zero
  )


def _efc_contact_frictionless(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for frictionless contacts."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError(
        '_efc_contact_frictionless requires JAX backend implementation.'
    )

  con_id = np.nonzero(d._impl.contact.dim == 1)[0]

  if con_id.size == 0:
    return None

  @jax.vmap
  def rows(c: Contact):
    pos = c.dist - c.includemargin
    active = pos < 0
    body1, body2 = jp.array(m.geom_bodyid)[c.geom]
    jac1p, _ = support.jac(m, d, c.pos, body1)
    jac2p, _ = support.jac(m, d, c.pos, body2)
    j = (c.frame @ (jac2p - jac1p).T)[0]
    invweight = m.body_invweight0[body1, 0] + m.body_invweight0[body2, 0]

    return _row(
        j * active,
        pos * active,
        pos,
        invweight,
        c.solref,
        c.solimp,
        c.includemargin,
        jp.zeros_like(pos),
    )

  contact = jax.tree_util.tree_map(lambda x: x[con_id], d._impl.contact)

  return rows(contact)


def _efc_contact_pyramidal(m: Model, d: Data, condim: int) -> Optional[_Efc]:
  """Calculates constraint rows for frictional pyramidal contacts."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError(
        '_efc_contact_pyramidal requires JAX backend implementation.'
    )

  con_id = np.nonzero(d._impl.contact.dim == condim)[0]

  if con_id.size == 0:
    return None

  @jax.vmap
  def rows(c: Contact):
    pos = c.dist - c.includemargin
    active = pos < 0
    body1, body2 = jp.array(m.geom_bodyid)[c.geom]
    jac1p, jac1r = support.jac(m, d, c.pos, body1)
    jac2p, jac2r = support.jac(m, d, c.pos, body2)
    diff = c.frame @ (jac2p - jac1p).T
    if condim > 3:
      diff = jp.concatenate((diff, (c.frame @ (jac2r - jac1r).T)), axis=0)
    # a pair of opposing pyramid edges per friction dimension
    # repeat friction directions with positive and negative sign
    fri = jp.repeat(c.friction[: condim - 1], 2, axis=0).at[1::2].mul(-1)
    # repeat condims of jacdiff to match +/- friction directions
    j = diff[0] + jp.repeat(diff[1:condim], 2, axis=0) * fri[:, None]

    # pyramidal has common invweight across all edges
    invweight = m.body_invweight0[body1, 0] + m.body_invweight0[body2, 0]
    invweight = invweight + fri[0] * fri[0] * invweight
    invweight = invweight * 2 * fri[0] * fri[0] / m.opt.impratio

    return _row(
        j * active,
        pos * active,
        pos,
        invweight,
        c.solref,
        c.solimp,
        c.includemargin,
        jp.zeros_like(pos),
    )

  contact = jax.tree_util.tree_map(lambda x: x[con_id], d._impl.contact)
  # concatenate to drop row grouping
  return jax.tree_util.tree_map(jp.concatenate, rows(contact))


def _efc_contact_elliptic(m: Model, d: Data, condim: int) -> Optional[_Efc]:
  """Calculates constraint rows for frictional elliptic contacts."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError(
        '_efc_contact_elliptic requires JAX backend implementation.'
    )

  con_id = np.nonzero(d._impl.contact.dim == condim)[0]

  if con_id.size == 0:
    return None

  @jax.vmap
  def rows(c: Contact):
    pos = c.dist - c.includemargin
    active = pos < 0
    obj1id, obj2id = jp.array(m.geom_bodyid)[c.geom]
    jac1p, jac1r = support.jac(m, d, c.pos, obj1id)
    jac2p, jac2r = support.jac(m, d, c.pos, obj2id)
    j = c.frame @ (jac2p - jac1p).T
    if condim > 3:
      j = jp.concatenate((j, (c.frame @ (jac2r - jac1r).T)[: condim - 3]))
    invweight = m.body_invweight0[obj1id, 0] + m.body_invweight0[obj2id, 0]

    # normal row comes from solref, remaining rows from solreffriction
    solreffriction = c.solreffriction + c.solref * ~c.solreffriction.any()
    solreffriction = jp.tile(solreffriction, (condim - 1, 1))
    solref = jp.concatenate((c.solref[None], solreffriction))
    fri = jp.square(c.friction[0]) / jp.square(c.friction[1 : condim - 1])
    invweight = jp.array([invweight, invweight / m.opt.impratio])
    invweight = jp.concatenate((invweight, invweight[1] * fri))
    pos_aref = jp.zeros(condim).at[0].set(pos)

    return _row(
        j * active,
        pos_aref * active,
        pos,
        invweight,
        solref,
        c.solimp,
        c.includemargin,
        jp.zeros_like(pos),
    )

  contact = jax.tree_util.tree_map(lambda x: x[con_id], d._impl.contact)
  # concatenate to drop row grouping
  return jax.tree_util.tree_map(jp.concatenate, rows(contact))


def counts(efc_type: np.ndarray) -> Tuple[int, int, int, int]:
  """Returns equality, friction, limit, and contact constraint counts."""
  ne = (efc_type == ConstraintType.EQUALITY).sum()
  nf = (efc_type == ConstraintType.FRICTION_DOF).sum()
  nf += (efc_type == ConstraintType.FRICTION_TENDON).sum()
  nl = (efc_type == ConstraintType.LIMIT_JOINT).sum()
  nl += (efc_type == ConstraintType.LIMIT_TENDON).sum()
  nc_f = (efc_type == ConstraintType.CONTACT_FRICTIONLESS).sum()
  nc_p = (efc_type == ConstraintType.CONTACT_PYRAMIDAL).sum()
  nc_e = (efc_type == ConstraintType.CONTACT_ELLIPTIC).sum()
  nc = nc_f + nc_p + nc_e

  return ne, nf, nl, nc


def make_efc_type(
    m: Union[Model, mujoco.MjModel], dim: Optional[np.ndarray] = None
) -> np.ndarray:
  """Returns efc_type that outlines the type of each constraint row."""
  if m.opt.disableflags & DisableBit.CONSTRAINT:
    return np.empty(0, dtype=int)

  dim = collision_driver.make_condim(m) if dim is None else dim
  efc_types = []

  if not m.opt.disableflags & DisableBit.EQUALITY:
    num_rows = (m.eq_type == EqType.CONNECT).sum() * 3
    num_rows += (m.eq_type == EqType.WELD).sum() * 6
    num_rows += (m.eq_type == EqType.JOINT).sum()
    num_rows += (m.eq_type == EqType.TENDON).sum()
    efc_types += [ConstraintType.EQUALITY] * num_rows

  if not m.opt.disableflags & DisableBit.FRICTIONLOSS:
    nf_dof = (
        m._impl.dof_hasfrictionloss.sum()
        if isinstance(m, Model) and isinstance(m._impl, ModelJAX)
        else (m.dof_frictionloss > 0).sum()
    )
    efc_types += [ConstraintType.FRICTION_DOF] * nf_dof
    nf_tendon = (
        m._impl.tendon_hasfrictionloss.sum()
        if isinstance(m, Model) and isinstance(m._impl, ModelJAX)
        else (m.tendon_frictionloss > 0).sum()
    )
    efc_types += [ConstraintType.FRICTION_TENDON] * nf_tendon

  if not m.opt.disableflags & DisableBit.LIMIT:
    efc_types += [ConstraintType.LIMIT_JOINT] * m.jnt_limited.sum()
    efc_types += [ConstraintType.LIMIT_TENDON] * m.tendon_limited.sum()

  if not m.opt.disableflags & DisableBit.CONTACT:
    for condim in (1, 3, 4, 6):
      n = (dim == condim).sum()
      if condim == 1:
        efc_types += [ConstraintType.CONTACT_FRICTIONLESS] * n
      elif m.opt.cone == ConeType.PYRAMIDAL:
        efc_types += [ConstraintType.CONTACT_PYRAMIDAL] * (condim - 1) * 2 * n
      elif m.opt.cone == ConeType.ELLIPTIC:
        efc_types += [ConstraintType.CONTACT_ELLIPTIC] * condim * n
      else:
        raise ValueError(f'Unknown cone: {m.opt.cone}')

  return np.array(efc_types)


def make_efc_address(
    m: Union[Model, mujoco.MjModel], dim: np.ndarray, efc_type: np.ndarray
) -> np.ndarray:
  """Returns efc_address that maps contacts to constraint row address."""
  offsets = np.array([0], dtype=int)
  for condim in (1, 3, 4, 6):
    n = (dim == condim).sum()
    if n == 0:
      continue
    if condim == 1:
      offsets = np.concatenate((offsets, [1] * n))
    elif m.opt.cone == ConeType.PYRAMIDAL:
      offsets = np.concatenate((offsets, [(condim - 1) * 2] * n))
    elif m.opt.cone == ConeType.ELLIPTIC:
      offsets = np.concatenate((offsets, [condim] * n))
    else:
      raise ValueError(f'Unknown cone: {m.opt.cone}')

  _, _, _, nc = counts(efc_type)
  address = efc_type.size - nc + np.cumsum(offsets)[:-1]

  return address


def make_constraint(m: Model, d: Data) -> Data:
  """Creates constraint jacobians and other supporting data."""

  if m.opt.disableflags & DisableBit.CONSTRAINT:
    efcs = ()
  else:
    efcs = (
        _efc_equality_connect(m, d),
        _efc_equality_weld(m, d),
        _efc_equality_joint(m, d),
        _efc_equality_tendon(m, d),
        _efc_friction(m, d),
        _efc_limit_ball(m, d),
        _efc_limit_slide_hinge(m, d),
        _efc_limit_tendon(m, d),
        _efc_contact_frictionless(m, d),
    )
    if m.opt.cone == ConeType.ELLIPTIC:
      con_fn = _efc_contact_elliptic
    else:
      con_fn = _efc_contact_pyramidal
    efcs += tuple(con_fn(m, d, dim) for dim in (3, 4, 6))
    efcs = tuple(efc for efc in efcs if efc is not None)

  if not efcs:
    z = jp.empty(0)
    d = d.tree_replace({'_impl.efc_J': jp.empty((0, m.nv))})
    d = d.tree_replace({
        '_impl.efc_D': z,
        '_impl.efc_aref': z,
        '_impl.efc_frictionloss': z,
        '_impl.efc_pos': z,
        '_impl.efc_margin': z,
    })
    return d

  efc = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *efcs)

  @jax.vmap
  def fn(efc):
    k, b, imp = _kbi(m, efc.solref, efc.solimp, efc.pos_imp)
    r = jp.maximum(efc.invweight * (1 - imp) / imp, mujoco.mjMINVAL)
    aref = -b * (efc.J @ d.qvel) - k * imp * efc.pos_aref
    return aref, r, efc.pos_aref + efc.margin, efc.margin, efc.frictionloss

  aref, r, pos, margin, frictionloss = fn(efc)
  d = d.tree_replace({
      '_impl.efc_J': efc.J,
      '_impl.efc_D': 1 / r,
      '_impl.efc_aref': aref,
      '_impl.efc_pos': pos,
      '_impl.efc_margin': margin,
  })
  d = d.tree_replace({'_impl.efc_frictionloss': frictionloss})

  return d
