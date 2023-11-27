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
"""Core smooth dynamics functions."""

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import math
from mujoco.mjx._src import scan
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member


def kinematics(m: Model, d: Data) -> Data:
  """Converts position/velocity from generalized coordinates to maximal."""

  def fn(carry, jnt_typs, jnt_pos, jnt_axis, qpos, qpos0, pos, quat):
    # calculate joint anchors, axes, body pos and quat in global frame
    # also normalize qpos while we're at it

    if carry is not None:
      _, _, _, parent_pos, parent_quat, _ = carry
      pos = parent_pos + math.rotate(pos, parent_quat)
      quat = math.quat_mul(parent_quat, quat)

    anchors, axes = [], []

    qpos_i = 0
    for i, jnt_typ in enumerate(jnt_typs):
      if jnt_typ == JointType.FREE:
        anchor, axis = qpos[qpos_i : qpos_i + 3], jp.array([0.0, 0.0, 1.0])
      else:
        anchor = math.rotate(jnt_pos[i], quat) + pos
        axis = math.rotate(jnt_axis[i], quat)
      anchors, axes = anchors + [anchor], axes + [axis]

      if jnt_typ == JointType.FREE:
        pos = qpos[qpos_i : qpos_i + 3]
        quat = math.normalize(qpos[qpos_i + 3 : qpos_i + 7])
        qpos = qpos.at[qpos_i + 3 : qpos_i + 7].set(quat)
        qpos_i += 7
      elif jnt_typ == JointType.BALL:
        qloc = math.normalize(qpos[qpos_i : qpos_i + 4])
        qpos = qpos.at[qpos_i : qpos_i + 4].set(qloc)
        quat = math.quat_mul(quat, qloc)
        pos = anchor - math.rotate(jnt_pos[i], quat)  # off-center rotation
        qpos_i += 4
      elif jnt_typ == JointType.HINGE:
        angle = qpos[qpos_i] - qpos0[qpos_i]
        qloc = math.axis_angle_to_quat(jnt_axis[i], angle)
        quat = math.quat_mul(quat, qloc)
        pos = anchor - math.rotate(jnt_pos[i], quat)  # off-center rotation
        qpos_i += 1
      elif jnt_typ == JointType.SLIDE:
        pos += axis * (qpos[qpos_i] - qpos0[qpos_i])
        qpos_i += 1
      else:
        raise RuntimeError(f'unrecognized joint type: {jnt_typ}')

    anchor = jp.stack(anchors) if anchors else jp.empty((0, 3))
    axis = jp.stack(axes) if axes else jp.empty((0, 3))
    mat = math.quat_to_mat(quat)

    return qpos, anchor, axis, pos, quat, mat

  qpos, xanchor, xaxis, xpos, xquat, xmat = scan.body_tree(
      m,
      fn,
      'jjjqqbb',
      'qjjbbb',
      m.jnt_type,
      m.jnt_pos,
      m.jnt_axis,
      d.qpos,
      m.qpos0,
      m.body_pos,
      m.body_quat,
  )

  @jax.vmap
  def local_to_global(pos1, quat1, pos2, quat2):
    pos = pos1 + math.rotate(pos2, quat1)
    mat = math.quat_to_mat(math.quat_mul(quat1, quat2))
    return pos, mat

  # TODO(erikfrey): confirm that quats are more performant for mjx than mats
  xipos, ximat = local_to_global(xpos, xquat, m.body_ipos, m.body_iquat)
  d = d.replace(qpos=qpos, xanchor=xanchor, xaxis=xaxis, xpos=xpos)
  d = d.replace(xquat=xquat, xmat=xmat, xipos=xipos, ximat=ximat)

  if m.ngeom:
    geom_xpos, geom_xmat = local_to_global(
        xpos[m.geom_bodyid], xquat[m.geom_bodyid], m.geom_pos, m.geom_quat
    )
    d = d.replace(geom_xpos=geom_xpos, geom_xmat=geom_xmat)

  if m.nsite:
    site_xpos, site_xmat = local_to_global(
        xpos[m.site_bodyid], xquat[m.site_bodyid], m.site_pos, m.site_quat
    )
    d = d.replace(site_xpos=site_xpos, site_xmat=site_xmat)

  return d


def com_pos(m: Model, d: Data) -> Data:
  """Maps inertias and motion dofs to global frame centered at subtree-CoM."""

  # calculate center of mass of each subtree
  def subtree_sum(carry, xipos, body_mass):
    pos, mass = xipos * body_mass, body_mass
    if carry is not None:
      subtree_pos, subtree_mass = carry
      pos, mass = pos + subtree_pos, mass + subtree_mass
    return pos, mass

  pos, mass = scan.body_tree(
      m, subtree_sum, 'bb', 'bb', d.xipos, m.body_mass, reverse=True
  )
  cond = jp.tile(mass < jp.array(mujoco.mjMINVAL), (3, 1)).T
  subtree_com = jp.where(cond, d.xipos, jax.vmap(jp.divide)(pos, mass))
  d = d.replace(subtree_com=subtree_com)

  # map inertias to frame centered at subtree_com
  @jax.vmap
  def inert_com(inert, ximat, off, mass):
    h = jp.cross(off, -jp.eye(3))
    inert = ximat @ jp.diag(inert) @ ximat.T + h @ h.T * mass
    # cinert is triu(inert), mass * off, mass
    inert = inert[(jp.array([0, 1, 2, 0, 0, 1]), jp.array([0, 1, 2, 1, 2, 2]))]
    return jp.concatenate([inert, off * mass, jp.expand_dims(mass, 0)])

  root_com = subtree_com[jp.array(m.body_rootid)]
  offset = d.xipos - root_com
  cinert = inert_com(m.body_inertia, d.ximat, offset, m.body_mass)
  d = d.replace(cinert=cinert)

  # map motion dofs to global frame centered at subtree_com
  def cdof_fn(jnt_typs, root_com, xmat, xanchor, xaxis):
    cdofs = []

    dof_com_fn = lambda a, o: jp.concatenate([a, jp.cross(a, o)])

    for i, jnt_typ in enumerate(jnt_typs):
      offset = root_com - xanchor[i]
      if jnt_typ == JointType.FREE:
        cdofs.append(jp.eye(3, 6, 3))  # free translation
        cdofs.append(jax.vmap(dof_com_fn, in_axes=(0, None))(xmat.T, offset))
      elif jnt_typ == JointType.BALL:
        cdofs.append(jax.vmap(dof_com_fn, in_axes=(0, None))(xmat.T, offset))
      elif jnt_typ == JointType.HINGE:
        cdof = dof_com_fn(xaxis[i], offset)
        cdofs.append(jp.expand_dims(cdof, 0))
      elif jnt_typ == JointType.SLIDE:
        cdof = jp.concatenate((jp.zeros((3,)), xaxis[i]))
        cdofs.append(jp.expand_dims(cdof, 0))
      else:
        raise RuntimeError(f'unrecognized joint type: {jnt_typ}')

    cdof = jp.concatenate(cdofs) if cdofs else jp.empty((0, 6))

    return cdof

  cdof = scan.flat(
      m,
      cdof_fn,
      'jbbjj',
      'v',
      m.jnt_type,
      root_com,
      d.xmat,
      d.xanchor,
      d.xaxis,
  )
  d = d.replace(cdof=cdof)

  return d


def crb(m: Model, d: Data) -> Data:
  """Runs composite rigid body inertia algorithm."""

  def crb_fn(crb_child, crb_body):
    if crb_child is not None:
      crb_body += crb_child
    return crb_body

  crb_body = scan.body_tree(m, crb_fn, 'b', 'b', d.cinert, reverse=True)
  crb_body = crb_body.at[0].set(0.0)
  d = d.replace(crb=crb_body)

  # TODO(erikfrey): do centralized take fn?
  crb_dof = jp.take(crb_body, jp.array(m.dof_bodyid), axis=0)
  crb_cdof = jax.vmap(math.inert_mul)(crb_dof, d.cdof)

  dof_i, dof_j, diag = [], [], []
  for i in range(m.nv):
    diag.append(len(dof_i))
    j = i
    while j > -1:
      dof_i, dof_j = dof_i + [i], dof_j + [j]
      j = m.dof_parentid[j]

  crb_codf_i = jp.take(crb_cdof, jp.array(dof_i), axis=0)
  cdof_j = jp.take(d.cdof, jp.array(dof_j), axis=0)
  qm = jax.vmap(jp.dot)(crb_codf_i, cdof_j)

  # add armature to diagonal
  qm = qm.at[jp.array(diag)].add(m.dof_armature)

  d = d.replace(qM=qm)

  return d


def factor_m(
    m: Model,
    d: Data,
    qM: jax.Array,  # pylint:disable=invalid-name
) -> Data:
  """Gets sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd."""

  # build up indices for where we will do backwards updates over qLD
  # TODO(erikfrey): do fewer updates by combining non-overlapping ranges
  dof_madr = jp.array(m.dof_Madr)
  updates = {}
  madr_ds = []
  for i in range(m.nv):
    madr_d = madr_ij = m.dof_Madr[i]
    j = i
    while True:
      madr_ds.append(madr_d)
      madr_ij, j = madr_ij + 1, m.dof_parentid[j]
      if j == -1:
        break
      madr_j_range = tuple(m.dof_Madr[j : j + 2])
      updates.setdefault(madr_j_range, []).append((madr_d, madr_ij))

  qld = qM

  for (out_beg, out_end), vals in sorted(updates.items(), reverse=True):
    madr_d, madr_ij = jp.array(vals).T

    @jax.vmap
    def off_diag_fn(madr_d, madr_ij, qld=qld, width=out_end - out_beg):
      qld_row = jax.lax.dynamic_slice(qld, (madr_ij,), (width,))
      return -(qld_row[0] / qld[madr_d]) * qld_row

    qld_update = jp.sum(off_diag_fn(madr_d, madr_ij), axis=0)
    qld = qld.at[out_beg:out_end].add(qld_update)
    # TODO(erikfrey): determine if this minimum value guarding is necessary:
    # qld = qld.at[dof_madr].set(jp.maximum(qld[dof_madr], _MJ_MINVAL))

  qld_diag = qld[dof_madr]
  qld = (qld / qld[jp.array(madr_ds)]).at[dof_madr].set(qld_diag)

  d = d.replace(qLD=qld, qLDiagInv=1 / qld_diag)

  return d


def solve_m(m: Model, d: Data, x: jax.Array) -> jax.Array:
  """Computes sparse backsubstitution:  x = inv(L'*D*L)*y ."""

  updates_i, updates_j = {}, {}
  for i in range(m.nv):
    madr_ij, j = m.dof_Madr[i], i
    while True:
      madr_ij, j = madr_ij + 1, m.dof_parentid[j]
      if j == -1:
        break
      updates_i.setdefault(i, []).append((madr_ij, j))
      updates_j.setdefault(j, []).append((madr_ij, i))

  # x <- inv(L') * x
  for j, vals in sorted(updates_j.items(), reverse=True):
    madr_ij, i = jp.array(vals).T
    x = x.at[j].add(-jp.sum(d.qLD[madr_ij] * x[i]))

  # x <- inv(D) * x
  x = x * d.qLDiagInv

  # x <- inv(L) * x
  for i, vals in sorted(updates_i.items()):
    madr_ij, j = jp.array(vals).T
    x = x.at[i].add(-jp.sum(d.qLD[madr_ij] * x[j]))

  return x


def dense_m(m: Model, d: Data) -> jax.Array:
  """Reconstitute dense mass matrix from qM."""

  is_, js, madr_ijs = [], [], []
  for i in range(m.nv):
    madr_ij, j = m.dof_Madr[i], i

    while True:
      madr_ij, j = madr_ij + 1, m.dof_parentid[j]
      if j == -1:
        break
      is_, js, madr_ijs = is_ + [i], js + [j], madr_ijs + [madr_ij]

  i, j, madr_ij = (jp.array(x, dtype=jp.int32) for x in (is_, js, madr_ijs))

  mat = jp.zeros((m.nv, m.nv)).at[(i, j)].set(d.qM[madr_ij])

  # diagonal, upper triangular, lower triangular
  mat = jp.diag(d.qM[jp.array(m.dof_Madr)]) + mat + mat.T

  return mat


def mul_m(m: Model, d: Data, vec: jax.Array) -> jax.Array:
  """Multiply vector by inertia matrix."""

  diag_mul = d.qM[jp.array(m.dof_Madr)] * vec

  is_, js, madr_ijs = [], [], []
  for i in range(m.nv):
    madr_ij, j = m.dof_Madr[i], i

    while True:
      madr_ij, j = madr_ij + 1, m.dof_parentid[j]
      if j == -1:
        break
      is_, js, madr_ijs = is_ + [i], js + [j], madr_ijs + [madr_ij]

  i, j, madr_ij = (jp.array(x, dtype=jp.int32) for x in (is_, js, madr_ijs))

  out = diag_mul.at[i].add(d.qM[madr_ij] * vec[j])
  out = out.at[j].add(d.qM[madr_ij] * vec[i])

  return out


def com_vel(m: Model, d: Data) -> Data:
  """Computes cvel, cdof_dot."""

  # forward scan down tree: accumulate link center of mass velocity
  def fn(parent, jnt_typs, cdof, qvel):
    cvel = jp.zeros((6,)) if parent is None else parent[0]

    cross_fn = jax.vmap(math.motion_cross, in_axes=(None, 0))
    cdof_x_qvel = jax.vmap(jp.multiply)(cdof, qvel)

    dof_beg = 0
    cdof_dots = []
    for jnt_typ in jnt_typs:
      dof_end = dof_beg + JointType(jnt_typ).dof_width()
      if jnt_typ == JointType.FREE:
        cvel += jp.sum(cdof_x_qvel[:3], axis=0)
        cdof_ang_dot = cross_fn(cvel, cdof[3:])
        cvel += jp.sum(cdof_x_qvel[3:], axis=0)
        cdof_dots.append(jp.concatenate((jp.zeros((3, 6)), cdof_ang_dot)))
      else:
        cdof_dots.append(cross_fn(cvel, cdof[dof_beg:dof_end]))
        cvel += jp.sum(cdof_x_qvel[dof_beg:dof_end], axis=0)
      dof_beg = dof_end

    cdof_dot = jp.concatenate(cdof_dots) if cdof_dots else jp.empty((0, 6))
    return cvel, cdof_dot

  cvel, cdof_dot = scan.body_tree(
      m,
      fn,
      'jvv',
      'bv',
      m.jnt_type,
      d.cdof,
      d.qvel,
  )

  d = d.replace(cvel=cvel, cdof_dot=cdof_dot)

  return d


def rne(m: Model, d: Data) -> Data:
  """Computes inverse dynamics using the recursive Newton-Euler algorithm."""
  # forward scan over tree: accumulate link center of mass acceleration
  def cacc_fn(cacc, cdof_dot, qvel):
    if cacc is None:
      if m.opt.disableflags & DisableBit.GRAVITY:
        cacc = jp.zeros((6,))
      else:
        cacc = jp.concatenate((jp.zeros((3,)), -m.opt.gravity))

    cacc += jp.sum(jax.vmap(jp.multiply)(cdof_dot, qvel), axis=0)

    return cacc

  cacc = scan.body_tree(m, cacc_fn, 'vv', 'b', d.cdof_dot, d.qvel)

  def frc(cinert, cacc, cvel):
    frc = math.inert_mul(cinert, cacc)
    frc += math.motion_cross_force(cvel, math.inert_mul(cinert, cvel))

    return frc

  loc_cfrc = jax.vmap(frc)(d.cinert, cacc, d.cvel)

  # backward scan up tree: accumulate body forces
  def cfrc_fn(cfrc_child, cfrc):
    if cfrc_child is not None:
      cfrc += cfrc_child
    return cfrc

  cfrc = scan.body_tree(m, cfrc_fn, 'b', 'b', loc_cfrc, reverse=True)
  qfrc_bias = jax.vmap(jp.dot)(d.cdof, cfrc[jp.array(m.dof_bodyid)])

  d = d.replace(qfrc_bias=qfrc_bias)

  return d


def transmission(m: Model, d: Data) -> Data:
  """Computes actuator/transmission lengths and moments."""
  if not m.nu:
    return d

  def fn(gear, jnt_typ, m_i, m_j, qpos):
    # handles joint transmissions only
    if jnt_typ == JointType.FREE:
      length = jp.zeros(1)
      moment = gear
      m_i = jp.repeat(m_i, 6)
      m_j = m_j + jp.arange(6)
    elif jnt_typ == JointType.BALL:
      axis, _ = math.quat_to_axis_angle(qpos)
      length = jp.dot(axis, gear[:3])[None]
      moment = gear[:3]
      m_i = jp.repeat(m_i, 3)
      m_j = m_j + jp.arange(3)
    elif jnt_typ in (JointType.SLIDE, JointType.HINGE):
      length = qpos * gear[0]
      moment = gear[:1]
      m_i, m_j = m_i[None], m_j[None]
    else:
      raise RuntimeError(f'unrecognized joint type: {jnt_typ}')
    return length, moment, m_i, m_j

  length, m_val, m_i, m_j = scan.flat(
      m,
      fn,
      'ujujq',
      'uvvv',
      m.actuator_gear,
      m.jnt_type,
      jp.arange(m.nu),
      jp.array(m.jnt_dofadr),
      d.qpos,
      group_by='u',
  )
  moment = jp.zeros((m.nu, m.nv)).at[m_i, m_j].set(m_val)
  length = length.reshape((m.nu,))
  d = d.replace(actuator_length=length, actuator_moment=moment)
  return d
