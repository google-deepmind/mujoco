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
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import JointType
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import TrnType
# pylint: enable=g-importing-member
import numpy as np


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
  cond = jp.tile(mass < mujoco.mjMINVAL, (3, 1)).T
  # take maximum to avoid NaN in gradient of jp.where
  subtree_com = jax.vmap(jp.divide)(pos, jp.maximum(mass, mujoco.mjMINVAL))
  subtree_com = jp.where(cond, d.xipos, subtree_com)
  d = d.replace(subtree_com=subtree_com)

  # map inertias to frame centered at subtree_com
  @jax.vmap
  def inert_com(inert, ximat, off, mass):
    h = jp.cross(off, -jp.eye(3))
    inert = (ximat * inert) @ ximat.T + h @ h.T * mass
    # cinert is triu(inert), mass * off, mass
    inert = inert[([0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2])]
    return jp.concatenate([inert, off * mass, mass[None]])

  root_com = subtree_com[m.body_rootid]
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

  crb_dof = jp.take(crb_body, jp.array(m.dof_bodyid), axis=0)
  crb_cdof = jax.vmap(math.inert_mul)(crb_dof, d.cdof)
  qm = support.make_m(m, crb_cdof, d.cdof, m.dof_armature)
  d = d.replace(qM=qm)

  return d


def factor_m(m: Model, d: Data) -> Data:
  """Gets factorizaton of inertia-like matrix M, assumed spd."""

  if not support.is_sparse(m):
    qh, _ = jax.scipy.linalg.cho_factor(d.qM)
    d = d.replace(qLD=qh)
    return d

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

  qld = d.qM

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

  if not support.is_sparse(m):
    return jax.scipy.linalg.cho_solve((d.qLD, False), x)

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


def _site_dof_mask(m: Model) -> np.ndarray:
  """Creates a dof mask for site transmissions."""
  mask = np.ones((m.nu, m.nv))
  for i in np.nonzero(m.actuator_trnid[:, 1] != -1)[0]:
    id_, refid = m.actuator_trnid[i]
    # intialize last dof address for each body
    b0 = m.body_weldid[m.site_bodyid[id_]]
    b1 = m.body_weldid[m.site_bodyid[refid]]
    dofadr0 = m.body_dofadr[b0] + m.body_dofnum[b0] - 1
    dofadr1 = m.body_dofadr[b1] + m.body_dofnum[b1] - 1

    # find common ancestral dof, if any
    while dofadr0 != dofadr1:
      if dofadr0 < dofadr1:
        dofadr1 = m.dof_parentid[dofadr1]
      else:
        dofadr0 = m.dof_parentid[dofadr0]
      if dofadr0 == -1 or dofadr1 == -1:
        break

    # if common ancestral dof was found, clear the columns of its parental chain
    da = dofadr0 if dofadr0 == dofadr1 else -1
    while da >= 0:
      mask[i, da] = 0.0
      da = m.dof_parentid[da]

  return mask


def transmission(m: Model, d: Data) -> Data:
  """Computes actuator/transmission lengths and moments."""
  # TODO: consider combining transmission calculation into fwd_actuation.
  if not m.nu:
    return d

  def fn(
      trntype,
      trnid,
      gear,
      jnt_typ,
      m_j,
      qpos,
      has_refsite,
      site_dof_mask,
      site_xpos,
      site_xmat,
      site_quat,
  ):
    if trntype == TrnType.JOINT:
      if jnt_typ == JointType.FREE:
        length = jp.zeros(1)
        moment = gear
        m_j = m_j + jp.arange(6)
      elif jnt_typ == JointType.BALL:
        axis, angle = math.quat_to_axis_angle(qpos)
        length = jp.dot(axis * angle, gear[:3])[None]
        moment = gear[:3]
        m_j = m_j + jp.arange(3)
      elif jnt_typ in (JointType.SLIDE, JointType.HINGE):
        length = qpos * gear[0]
        moment = gear[:1]
        m_j = m_j[None]
      else:
        raise RuntimeError(f'unrecognized joint type: {JointType(jnt_typ)}')

      moment = jp.zeros((m.nv,)).at[m_j].set(moment)
    elif trntype == TrnType.SITE:
      length = jp.zeros(1)
      id_, refid = jp.array(m.site_bodyid)[trnid]
      jacp, jacr = support.jac(m, d, site_xpos[0], id_)
      frame_xmat = site_xmat[0]
      if has_refsite:
        vecp = site_xmat[1].T @ (site_xpos[0] - site_xpos[1])
        vecr = math.quat_sub(site_quat[0], site_quat[1])
        length += jp.dot(jp.concatenate([vecp, vecr]), gear)
        jacrefp, jacrefr = support.jac(m, d, site_xpos[1], refid)
        jacp, jacr = jacp - jacrefp, jacr - jacrefr
        frame_xmat = site_xmat[1]

      jac = jp.concatenate((jacp, jacr), axis=1) * site_dof_mask[:, None]
      wrench = jp.concatenate((frame_xmat @ gear[:3], frame_xmat @ gear[3:]))
      moment = jac @ wrench
    else:
      raise RuntimeError(f'unrecognized trntype: {TrnType(trntype)}')

    return length, moment

  # pre-compute values for site transmissions
  has_refsite = m.actuator_trnid[:, 1] != -1
  site_dof_mask = _site_dof_mask(m)
  site_quat = jax.vmap(math.quat_mul)(m.site_quat, d.xquat[m.site_bodyid])

  length, moment = scan.flat(
      m,
      fn,
      'uuujjquusss',
      'uu',
      m.actuator_trntype,
      jp.array(m.actuator_trnid),
      m.actuator_gear,
      m.jnt_type,
      jp.array(m.jnt_dofadr),
      d.qpos,
      has_refsite,
      jp.array(site_dof_mask),
      d.site_xpos,
      d.site_xmat,
      site_quat,
      group_by='u',
  )
  length = length.reshape((m.nu,))
  moment = moment.reshape((m.nu, m.nv))

  d = d.replace(actuator_length=length, actuator_moment=moment)
  return d
