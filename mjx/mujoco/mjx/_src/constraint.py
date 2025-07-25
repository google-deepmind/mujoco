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
from mujoco.mjx._src.types import ConstraintType
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import EqType
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np


_CONDIM_EFC_COUNT = {1: 1, 3: 4, 4: 6, 6: 10}


class _Efc(PyTreeNode):
  """Support data for creating constraint matrices."""
  J: jax.Array
  pos: jax.Array
  pos_norm: jax.Array
  invweight: jax.Array
  solref: jax.Array
  solimp: jax.Array
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


def _instantiate_equality_connect(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for connect equality constraints."""

  ids = np.nonzero(m.eq_type == EqType.CONNECT)[0]

  if (m.opt.disableflags & DisableBit.EQUALITY) or ids.size == 0:
    return None

  id1, id2, data = m.eq_obj1id[ids], m.eq_obj2id[ids], m.eq_data[ids]

  @jax.vmap
  def fn(data, id1, id2):
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

    return j, cpos, jp.repeat(math.norm(cpos), 3)

  # concatenate to drop connect grouping dimension
  j, pos, pos_norm = jax.tree_util.tree_map(jp.concatenate, fn(data, id1, id2))
  invweight = m.body_invweight0[id1, 0] + m.body_invweight0[id2, 0]
  invweight = jp.repeat(invweight, 3)
  solref = jp.tile(m.eq_solref[ids], (3, 1))
  solimp = jp.tile(m.eq_solimp[ids], (3, 1))
  frictionloss = jp.zeros_like(pos_norm)

  return _Efc(j, pos, pos_norm, invweight, solref, solimp, frictionloss)


def _instantiate_equality_weld(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for weld equality constraints."""

  ids = np.nonzero(m.eq_type == EqType.WELD)[0]

  if (m.opt.disableflags & DisableBit.EQUALITY) or ids.size == 0:
    return None

  id1, id2, data = m.eq_obj1id[ids], m.eq_obj2id[ids], m.eq_data[ids]

  @jax.vmap
  def fn(data, id1, id2):
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
    pos = jp.concatenate((cpos, crot * torquescale))

    return j, pos, jp.repeat(math.norm(pos), 6)

  # concatenate to drop weld grouping dimension
  j, pos, pos_norm = jax.tree_util.tree_map(jp.concatenate, fn(data, id1, id2))
  invweight = m.body_invweight0[id1] + m.body_invweight0[id2]
  invweight = jp.repeat(invweight, 3)
  solref = jp.tile(m.eq_solref[ids], (6, 1))
  solimp = jp.tile(m.eq_solimp[ids], (6, 1))
  frictionloss = jp.zeros_like(pos_norm)

  return _Efc(j, pos, pos_norm, invweight, solref, solimp, frictionloss)


def _instantiate_equality_joint(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for joint equality constraints."""

  ids = np.nonzero(m.eq_type == EqType.JOINT)[0]

  if (m.opt.disableflags & DisableBit.EQUALITY) or ids.size == 0:
    return None

  id1, id2, data = m.eq_obj1id[ids], m.eq_obj2id[ids], m.eq_data[ids]
  dofadr1, dofadr2 = m.jnt_dofadr[id1], m.jnt_dofadr[id2]
  qposadr1, qposadr2 = m.jnt_qposadr[id1], m.jnt_qposadr[id2]

  @jax.vmap
  def fn(data, id2, dofadr1, dofadr2, qposadr1, qposadr2):
    pos1, pos2 = d.qpos[qposadr1], d.qpos[qposadr2]
    ref1, ref2 = m.qpos0[qposadr1], m.qpos0[qposadr2]
    pos2, ref2 = pos2 * (id2 > -1), ref2 * (id2 > -1)

    dif = pos2 - ref2
    dif_power = jp.power(dif, jp.arange(0, 5))

    deriv = jp.dot(data[1:5], dif_power[:4] * jp.arange(1, 5))
    j = jp.zeros((m.nv)).at[dofadr1].set(1.0).at[dofadr2].set(-deriv)
    pos = pos1 - ref1 - jp.dot(data[:5], dif_power)
    return j, pos

  j, pos = fn(data, id2, dofadr1, dofadr2, qposadr1, qposadr2)
  invweight = m.dof_invweight0[dofadr1] + m.dof_invweight0[dofadr2] * (id2 > -1)
  solref, solimp = m.eq_solref[ids], m.eq_solimp[ids]
  frictionloss = jp.zeros_like(pos)

  return _Efc(j, pos, pos, invweight, solref, solimp, frictionloss)


def _instantiate_friction(m: Model, d: Data) -> Optional[_Efc]:
  # TODO(robotics-team): implement _instantiate_friction
  del m, d
  return None


def _instantiate_limit_ball(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for ball joint limits."""

  ids = np.nonzero((m.jnt_type == JointType.BALL) & m.jnt_limited)[0]

  if (m.opt.disableflags & DisableBit.LIMIT) or ids.size == 0:
    return None

  jnt_range = m.jnt_range[ids]
  jnt_margin = m.jnt_margin[ids]
  qposadr = np.array([np.arange(q, q + 4) for q in m.jnt_qposadr[ids]])
  dofadr = np.array([np.arange(d, d + 3) for d in m.jnt_dofadr[ids]])

  @jax.vmap
  def fn(jnt_range, jnt_margin, qposadr, dofadr):
    axis, angle = math.quat_to_axis_angle(d.qpos[qposadr])
    j = jp.zeros(m.nv).at[dofadr].set(-axis)
    pos = jp.amax(jnt_range) - angle - jnt_margin
    active = pos < 0
    return j * active, pos * active

  j, pos = fn(jnt_range, jnt_margin, qposadr, dofadr)
  invweight = m.dof_invweight0[m.jnt_dofadr[ids]]
  solref, solimp = m.jnt_solref[ids], m.jnt_solimp[ids]
  frictionloss = jp.zeros_like(pos)

  return _Efc(j, pos, pos, invweight, solref, solimp, frictionloss)


def _instantiate_limit_slide_hinge(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for slide and hinge joint limits."""

  slide_hinge = np.isin(m.jnt_type, (JointType.SLIDE, JointType.HINGE))
  ids = np.nonzero(slide_hinge & m.jnt_limited)[0]

  if (m.opt.disableflags & DisableBit.LIMIT) or ids.size == 0:
    return None

  jnt_range = m.jnt_range[ids]
  jnt_margin = m.jnt_margin[ids]
  qposadr = m.jnt_qposadr[ids]
  dofadr = m.jnt_dofadr[ids]

  @jax.vmap
  def fn(jnt_range, jnt_margin, qposadr, dofadr):
    dist_min = d.qpos[qposadr] - jnt_range[0]
    dist_max = jnt_range[1] - d.qpos[qposadr]
    j = jp.zeros(m.nv).at[dofadr].set((dist_min < dist_max) * 2 - 1)
    pos = jp.minimum(dist_min, dist_max) - jnt_margin
    active = pos < 0
    return j * active, pos * active

  j, pos = fn(jnt_range, jnt_margin, qposadr, dofadr)
  invweight = m.dof_invweight0[dofadr]
  solref, solimp = m.jnt_solref[ids], m.jnt_solimp[ids]
  frictionloss = jp.zeros_like(pos)

  return _Efc(j, pos, pos, invweight, solref, solimp, frictionloss)


def _instantiate_contact(m: Model, d: Data) -> Optional[_Efc]:
  """Calculates constraint rows for contacts."""

  if d.ncon == 0:
    return None

  def contact_efc(c: Contact, condim: int):

    @jax.vmap
    def fn(c: Contact):
      dist = c.dist - c.includemargin
      active = dist < 0
      body1, body2 = jp.array(m.geom_bodyid)[c.geom]
      jac1p, jac1r = support.jac(m, d, c.pos, body1)
      jac2p, jac2r = support.jac(m, d, c.pos, body2)
      diff = c.frame @ (jac2p - jac1p).T
      if condim > 3:  # only calculate rotational diff if needed
        diff = jp.concatenate((diff, c.frame @ (jac2r - jac1r).T), axis=0)
      tran = m.body_invweight0[body1, 0] + m.body_invweight0[body2, 0]

      if condim == 1:
        return diff[0] * active, tran, dist * active, c.solref, c.solimp

      # a pair of opposing pyramid edges per friction dimension
      # repeat friction directions with positive and negative sign
      fri = jp.repeat(c.friction[: condim - 1], 2, axis=0).at[1::2].mul(-1)
      # repeat condims of jacdiff to match +/- friction directions
      j = diff[0] + jp.repeat(diff[1:condim], 2, axis=0) * fri[:, None]
      # pyramidal has common invweight across all edges
      diag_approx = tran + fri[0] * fri[0] * tran
      inv_w = diag_approx * 2 * fri[0] * fri[0] / m.opt.impratio
      repeat_fn = lambda x: jp.repeat(x[None], (condim - 1) * 2, axis=0)
      inv_w, pos, solref, solimp = jax.tree_util.tree_map(
          repeat_fn, (inv_w, dist, c.solref, c.solimp)
      )
      return j * active, inv_w, pos * active, solref, solimp

    return fn(c)

  # group efc calculations by condim
  dims, begs = np.unique(d.contact.dim, return_index=True)
  efcs = []
  for i in range(len(dims)):
    dim, beg = dims[i], begs[i]
    end = begs[i + 1] if i < len(dims) - 1 else None
    c = jax.tree_util.tree_map(lambda x, b=beg, e=end: x[b:e], d.contact)
    efc = contact_efc(c, dim)
    if dim > 1:
      # remove efc grouping dimension
      efc = jax.tree_util.tree_map(jp.concatenate, efc)
    efcs.append(efc)

  efc = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *efcs)
  j, invweight, pos, solref, solimp = efc
  frictionloss = jp.zeros_like(pos)

  return _Efc(j, pos, pos, invweight, solref, solimp, frictionloss)


def counts(efc_type: np.ndarray) -> Tuple[int, int, int, int]:
  """Returns equality, friction, limit, and contact constraint counts."""
  ne = (efc_type == ConstraintType.EQUALITY).sum()
  nf = 0  # no support for friction loss yet
  nl = (efc_type == ConstraintType.LIMIT_JOINT).sum()
  nc_f = (efc_type == ConstraintType.CONTACT_FRICTIONLESS).sum()
  nc_p = (efc_type == ConstraintType.CONTACT_PYRAMIDAL).sum()
  nc = nc_f + nc_p

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
    efc_types.extend([ConstraintType.EQUALITY] * num_rows)

  if not m.opt.disableflags & DisableBit.LIMIT:
    efc_types.extend([ConstraintType.LIMIT_JOINT] * m.jnt_limited.sum())

  if not m.opt.disableflags & DisableBit.CONTACT:
    num_rows = sum(_CONDIM_EFC_COUNT[d] for d in dim)
    efc_types.extend([ConstraintType.CONTACT_PYRAMIDAL] * num_rows)

  return np.array(efc_types)


def make_efc_address(efc_type: np.ndarray, dim: np.ndarray) -> np.ndarray:
  """Returns efc_address that maps contacts to constraint row address."""
  nc = (efc_type == ConstraintType.CONTACT_PYRAMIDAL).sum()
  nc_start = efc_type.size - nc
  offsets = np.cumsum([0] + [_CONDIM_EFC_COUNT[d] for d in dim])[:-1]

  return nc_start + offsets


def make_constraint(m: Model, d: Data) -> Data:
  """Creates constraint jacobians and other supporting data."""

  if m.opt.disableflags & DisableBit.CONSTRAINT:
    efcs = ()
  else:
    efcs = tuple(efc for efc in (
        _instantiate_equality_connect(m, d),
        _instantiate_equality_weld(m, d),
        _instantiate_equality_joint(m, d),
        _instantiate_friction(m, d),
        _instantiate_limit_ball(m, d),
        _instantiate_limit_slide_hinge(m, d),
        _instantiate_contact(m, d),
    ) if efc is not None)

  if not efcs:
    z = jp.empty(0)
    d = d.replace(efc_J=jp.empty((0, m.nv)))
    d = d.replace(efc_D=z, efc_aref=z, efc_frictionloss=z)
    return d

  efc = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *efcs)

  @jax.vmap
  def fn(efc):
    k, b, imp = _kbi(m, efc.solref, efc.solimp, efc.pos_norm)
    r = jp.maximum(efc.invweight * (1 - imp) / imp, mujoco.mjMINVAL)
    aref = -b * (efc.J @ d.qvel) - k * imp * efc.pos
    return aref, r

  aref, r = fn(efc)
  d = d.replace(efc_J=efc.J, efc_D=1 / r, efc_aref=aref)
  d = d.replace(efc_frictionloss=efc.frictionloss)

  return d
