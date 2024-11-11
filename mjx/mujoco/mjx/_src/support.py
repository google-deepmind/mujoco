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
"""Engine support functions."""
from typing import Optional, Tuple, Union

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import math
from mujoco.mjx._src import scan
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import ConeType
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import JacobianType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np


def is_sparse(m: Union[mujoco.MjModel, Model]) -> bool:
  """Return True if this model should create sparse mass matrices.

  Args:
    m: a MuJoCo or MJX model

  Returns:
    True if provided model should create sparse mass matrices

  Modern TPUs have specialized hardware for rapidly operating over sparse
  matrices, whereas GPUs tend to be faster with dense matrices as long as they
  fit onto the device.  As such, the default behavior in MJX (via
  ``JacobianType.AUTO``) is sparse if ``nv`` is >= 60 or MJX detects a TPU as
  the default backend, otherwise dense.
  """
  # AUTO is a rough heuristic - you may see better performance for your workload
  # and compute by explicitly setting jacobian to dense or sparse
  if m.opt.jacobian == JacobianType.AUTO:
    return m.nv >= 60 or jax.default_backend() == 'tpu'
  return m.opt.jacobian == JacobianType.SPARSE


def make_m(
    m: Model, a: jax.Array, b: jax.Array, d: Optional[jax.Array] = None
) -> jax.Array:
  """Computes M = a @ b.T + diag(d)."""

  ij = []
  for i in range(m.nv):
    j = i
    while j > -1:
      ij.append((i, j))
      j = m.dof_parentid[j]

  i, j = (jp.array(x) for x in zip(*ij))

  if not is_sparse(m):
    qm = a @ b.T
    if d is not None:
      qm += jp.diag(d)
    mask = jp.zeros((m.nv, m.nv), dtype=bool).at[(i, j)].set(True)
    qm = qm * mask
    qm = qm + jp.tril(qm, -1).T
    return qm

  a_i = jp.take(a, i, axis=0)
  b_j = jp.take(b, j, axis=0)
  qm = jax.vmap(jp.dot)(a_i, b_j)

  # add diagonal
  if d is not None:
    qm = qm.at[m.dof_Madr].add(d)

  return qm


def full_m(m: Model, d: Data) -> jax.Array:
  """Reconstitute dense mass matrix from qM."""

  if not is_sparse(m):
    return d.qM

  ij = []
  for i in range(m.nv):
    j = i
    while j > -1:
      ij.append((i, j))
      j = m.dof_parentid[j]

  i, j = (jp.array(x) for x in zip(*ij))

  mat = jp.zeros((m.nv, m.nv)).at[(i, j)].set(d.qM)

  # also set upper triangular
  mat = mat + jp.tril(mat, -1).T

  return mat


def mul_m(m: Model, d: Data, vec: jax.Array) -> jax.Array:
  """Multiply vector by inertia matrix."""

  if not is_sparse(m):
    return d.qM @ vec

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


def jac(
    m: Model, d: Data, point: jax.Array, body_id: jax.Array
) -> Tuple[jax.Array, jax.Array]:
  """Compute pair of (NV, 3) Jacobians of global point attached to body."""
  fn = lambda carry, b: b if carry is None else b + carry
  mask = (jp.arange(m.nbody) == body_id) * 1
  mask = scan.body_tree(m, fn, 'b', 'b', mask, reverse=True)
  mask = mask[jp.array(m.dof_bodyid)] > 0

  offset = point - d.subtree_com[jp.array(m.body_rootid)[body_id]]
  jacp = jax.vmap(lambda a, b=offset: a[3:] + jp.cross(a[:3], b))(d.cdof)
  jacp = jax.vmap(jp.multiply)(jacp, mask)
  jacr = jax.vmap(jp.multiply)(d.cdof[:, :3], mask)

  return jacp, jacr


def apply_ft(
    m: Model,
    d: Data,
    force: jax.Array,
    torque: jax.Array,
    point: jax.Array,
    body_id: jax.Array,
) -> jax.Array:
  """Apply Cartesian force and torque."""
  jacp, jacr = jac(m, d, point, body_id)
  return jacp @ force + jacr @ torque


def xfrc_accumulate(m: Model, d: Data) -> jax.Array:
  """Accumulate xfrc_applied into a qfrc."""
  qfrc = jax.vmap(apply_ft, in_axes=(None, None, 0, 0, 0, 0))(
      m,
      d,
      d.xfrc_applied[:, :3],
      d.xfrc_applied[:, 3:],
      d.xipos,
      jp.arange(m.nbody),
  )
  return jp.sum(qfrc, axis=0)


def local_to_global(
    world_pos: jax.Array,
    world_quat: jax.Array,
    local_pos: jax.Array,
    local_quat: jax.Array,
) -> Tuple[jax.Array, jax.Array]:
  """Converts local position/orientation to world frame."""
  pos = world_pos + math.rotate(local_pos, world_quat)
  mat = math.quat_to_mat(math.quat_mul(world_quat, local_quat))
  return pos, mat


def _getnum(m: Union[Model, mujoco.MjModel], obj: mujoco._enums.mjtObj) -> int:
  """Gets the number of objects for the given object type."""
  return {
      mujoco.mjtObj.mjOBJ_BODY: m.nbody,
      mujoco.mjtObj.mjOBJ_JOINT: m.njnt,
      mujoco.mjtObj.mjOBJ_GEOM: m.ngeom,
      mujoco.mjtObj.mjOBJ_SITE: m.nsite,
      mujoco.mjtObj.mjOBJ_CAMERA: m.ncam,
      mujoco.mjtObj.mjOBJ_MESH: m.nmesh,
      mujoco.mjtObj.mjOBJ_HFIELD: m.nhfield,
      mujoco.mjtObj.mjOBJ_PAIR: m.npair,
      mujoco.mjtObj.mjOBJ_EQUALITY: m.neq,
      mujoco.mjtObj.mjOBJ_TENDON: m.ntendon,
      mujoco.mjtObj.mjOBJ_ACTUATOR: m.nu,
      mujoco.mjtObj.mjOBJ_SENSOR: m.nsensor,
      mujoco.mjtObj.mjOBJ_NUMERIC: m.nnumeric,
      mujoco.mjtObj.mjOBJ_TUPLE: m.ntuple,
      mujoco.mjtObj.mjOBJ_KEY: m.nkey,
  }.get(obj, 0)


def _getadr(
    m: Union[Model, mujoco.MjModel], obj: mujoco._enums.mjtObj
) -> np.ndarray:
  """Gets the name addresses for the given object type."""
  return {
      mujoco.mjtObj.mjOBJ_BODY: m.name_bodyadr,
      mujoco.mjtObj.mjOBJ_JOINT: m.name_jntadr,
      mujoco.mjtObj.mjOBJ_GEOM: m.name_geomadr,
      mujoco.mjtObj.mjOBJ_SITE: m.name_siteadr,
      mujoco.mjtObj.mjOBJ_CAMERA: m.name_camadr,
      mujoco.mjtObj.mjOBJ_MESH: m.name_meshadr,
      mujoco.mjtObj.mjOBJ_HFIELD: m.name_hfieldadr,
      mujoco.mjtObj.mjOBJ_PAIR: m.name_pairadr,
      mujoco.mjtObj.mjOBJ_EQUALITY: m.name_eqadr,
      mujoco.mjtObj.mjOBJ_TENDON: m.name_tendonadr,
      mujoco.mjtObj.mjOBJ_ACTUATOR: m.name_actuatoradr,
      mujoco.mjtObj.mjOBJ_SENSOR: m.name_sensoradr,
      mujoco.mjtObj.mjOBJ_NUMERIC: m.name_numericadr,
      mujoco.mjtObj.mjOBJ_TUPLE: m.name_tupleadr,
      mujoco.mjtObj.mjOBJ_KEY: m.name_keyadr,
  }[obj]


def id2name(
    m: Union[Model, mujoco.MjModel], typ: mujoco._enums.mjtObj, i: int
) -> Optional[str]:
  """Gets the name of an object with the specified mjtObj type and id.

  See mujoco.id2name for more info.

  Args:
    m: mujoco.MjModel or mjx.Model
    typ: mujoco.mjtObj type
    i: the id

  Returns:
    the name string, or None if not found
  """
  num = _getnum(m, typ)
  if i < 0 or i >= num:
    return None

  adr = _getadr(m, typ)
  name = m.names[adr[i] :].decode('utf-8').split('\x00', 1)[0]
  return name or None


def name2id(
    m: Union[Model, mujoco.MjModel], typ: mujoco._enums.mjtObj, name: str
) -> int:
  """Gets the id of an object with the specified mjtObj type and name.

  See mujoco.mj_name2id for more info.

  Args:
    m: mujoco.MjModel or mjx.Model
    typ: mujoco.mjtObj type
    name: the name of the object

  Returns:
   the id, or -1 if not found
  """
  num = _getnum(m, typ)
  adr = _getadr(m, typ)

  # TODO: consider using MjModel.names_map instead
  names_map = {
      m.names[adr[i] :].decode('utf-8').split('\x00', 1)[0]: i
      for i in range(num)
  }

  return names_map.get(name, -1)


def _decode_pyramid(
    pyramid: jax.Array, mu: jax.Array, condim: int
) -> jax.Array:
  """Converts pyramid representation to contact force."""
  force = jp.zeros(6, dtype=float)
  if condim == 1:
    return force.at[0].set(pyramid[0])

  # force_normal = sum(pyramid0_i + pyramid1_i)
  force = force.at[0].set(pyramid[0 : 2 * (condim - 1)].sum())

  # force_tangent_i = (pyramid0_i - pyramid1_i) * mu_i
  i = np.arange(0, condim - 1)
  force = force.at[i + 1].set((pyramid[2 * i] - pyramid[2 * i + 1]) * mu[i])

  return force


def contact_force(
    m: Model, d: Data, contact_id: int, to_world_frame: bool = False
) -> jax.Array:
  """Extract 6D force:torque for one contact, in contact frame by default."""
  efc_address = d.contact.efc_address[contact_id]
  condim = d.contact.dim[contact_id]
  if m.opt.cone == ConeType.PYRAMIDAL:
    force = _decode_pyramid(
        d.efc_force[efc_address:], d.contact.friction[contact_id], condim
    )
  elif m.opt.cone == ConeType.ELLIPTIC:
    force = d.efc_force[efc_address : efc_address + condim]
    force = jp.concatenate([force, jp.zeros((6 - condim))])
  else:
    raise ValueError(f'Unknown cone type: {m.opt.cone}')

  if to_world_frame:
    force = force.reshape((-1, 3)) @ d.contact.frame[contact_id]
    force = force.reshape(-1)

  return force * (efc_address >= 0)


def contact_force_dim(
    m: Model, d: Data, dim: int
) -> Tuple[jax.Array, np.ndarray]:
  """Extract 6D force:torque for contacts with dimension dim."""
  # valid contact and condim indices
  idx_dim = (d.contact.efc_address >= 0) & (d.contact.dim == dim)

  # contact force from efc
  if m.opt.cone == ConeType.PYRAMIDAL:
    efc_address = (
        d.contact.efc_address[idx_dim, None]
        + np.arange(np.where(dim == 1, 1, 2 * (dim - 1)))[None]
    )
    efc_force = d.efc_force[efc_address]
    force = jax.vmap(_decode_pyramid, in_axes=(0, 0, None))(
        efc_force, d.contact.friction[idx_dim], dim
    )
  elif m.opt.cone == ConeType.ELLIPTIC:
    efc_address = d.contact.efc_address[idx_dim, None] + np.arange(dim)[None]
    force = d.efc_force[efc_address]
    force = jp.hstack([force, jp.zeros((force.shape[0], 6 - dim))])
  else:
    raise ValueError(f'Unknown cone type: {m.opt.cone}.')
  return force, np.where(idx_dim)[0]


def _length_circle(
    p0: jax.Array, p1: jax.Array, ind: jax.Array, rad: jax.Array
) -> jax.Array:
  """Compute length of circle."""
  # compute angle between 0 and pi
  p0n = math.normalize(p0).reshape(-1)
  p1n = math.normalize(p1).reshape(-1)

  angle = jp.arccos(jp.dot(p0n, p1n))

  # flip if necessary
  cross = p0[1] * p1[0] - p0[0] * p1[1]
  flip = ((cross > 0) & (ind != 0)) | ((cross < 0) & (ind == 0))
  angle = jp.where(flip, 2 * jp.pi - angle, angle)

  return rad * angle


def _is_intersect(
    p1: jax.Array, p2: jax.Array, p3: jax.Array, p4: jax.Array
) -> jax.Array:
  """Check for intersection between two lines defined by their endpoints."""
  # compute determinant
  det = (p4[1] - p3[1]) * (p2[0] - p1[0]) - (p4[0] - p3[0]) * (p2[1] - p1[1])

  # compute intersection point on each line
  a = (
      (p4[0] - p3[0]) * (p1[1] - p3[1]) - (p4[1] - p3[1]) * (p1[0] - p3[0])
  ) / det
  b = (
      (p2[0] - p1[0]) * (p1[1] - p3[1]) - (p2[1] - p1[1]) * (p1[0] - p3[0])
  ) / det

  return (a >= 0) & (a <= 1) & (b >= 0) & (b <= 1)


def wrap_circle(
    d: jax.Array, sd: jax.Array, sidesite: jax.Array, rad: jax.Array
) -> Tuple[jax.Array, jax.Array]:
  """Compute circle wrap arc length and end points."""
  # check cases
  sqlen0 = d[0] ** 2 + d[1] ** 2
  sqlen1 = d[2] ** 2 + d[3] ** 2
  sqrad = rad * rad
  dif = jp.array([d[2] - d[0], d[3] - d[1]])
  dd = dif[0] ** 2 + dif[1] ** 2
  a = jp.clip(-(dif[0] * d[0] + dif[1] * d[1]) / dd, 0, 1)
  seg = jp.array([a * dif[0] + d[0], a * dif[1] + d[1]])

  point_inside0 = sqlen0 < sqrad
  point_inside1 = sqlen1 < sqrad
  circle_too_small = rad < mujoco.mjMINVAL
  points_too_close = dd < mujoco.mjMINVAL

  intersect_and_side = (seg[0] ** 2 + seg[1] ** 2 > sqrad) & (
      jp.where(sidesite, 0, 1) | (jp.dot(sd, seg) >= 0)
  )

  # construct the two solutions, compute goodness
  def _sol(sgn):
    sqrt0 = jp.sqrt(sqlen0 - sqrad)
    sqrt1 = jp.sqrt(sqlen1 - sqrad)

    d00 = (d[0] * sqrad + sgn * rad * d[1] * sqrt0) / sqlen0
    d01 = (d[1] * sqrad - sgn * rad * d[0] * sqrt0) / sqlen0
    d10 = (d[2] * sqrad - sgn * rad * d[3] * sqrt1) / sqlen1
    d11 = (d[3] * sqrad + sgn * rad * d[2] * sqrt1) / sqlen1

    sol = jp.array([[d00, d01], [d10, d11]])

    # goodness: close to sd, or shorter path
    tmp0 = sol[0] + sol[1]
    tmp0 = math.normalize(tmp0).reshape(-1)
    good0 = jp.dot(tmp0, sd)

    tmp1 = (sol[0] - sol[1]).reshape(-1)
    good1 = -jp.dot(tmp1, tmp1)

    good = jp.where(sidesite, good0, good1)

    # penalize for intersection
    intersect = _is_intersect(d[:2], sol[0], d[2:], sol[1])
    good = jp.where(intersect, -10000, good)

    return sol, good

  sol, good = jax.vmap(_sol)(jp.array([1, -1]))

  # select the better solution
  i = jp.argmax(good)
  sol = sol[i]
  pnt = sol.reshape(-1)

  # check for intersection
  intersect = _is_intersect(d[:2], pnt[:2], d[2:], pnt[2:])

  # compute curve length
  wlen = _length_circle(sol[0], sol[1], i, rad)

  # check cases
  invalid = (
      point_inside0
      | point_inside1
      | circle_too_small
      | points_too_close
      | intersect_and_side
      | intersect
  )

  wlen = jp.where(invalid, -1, wlen)
  pnt = jp.where(invalid, jp.zeros(4), pnt)

  return wlen, pnt


def wrap(
    x0: jax.Array,
    x1: jax.Array,
    xpos: jax.Array,
    xmat: jax.Array,
    size: jax.Array,
    side: jax.Array,
    sidesite: jax.Array,
    is_sphere: jax.Array,
):
  """Wrap tendon around sphere or cylinder."""
  # map sites to wrap object's local frame
  p0 = xmat.T @ (x0 - xpos)
  p1 = xmat.T @ (x1 - xpos)

  close_to_origin = (jp.linalg.norm(p0) < mujoco.mjMINVAL) | (
      jp.linalg.norm(p1) < mujoco.mjMINVAL
  )

  # compute axes for sphere
  # 1st axis
  axis0 = p0
  axis0 = math.normalize(axis0)

  # compute normal to p0-0-p1 plane = cross(p0, p1)
  normal = jp.cross(p0, p1)
  normal, nrm = math.normalize_with_norm(normal)

  # compute alternative normal (if (p0, p1) are parallel)
  # find max component of axis0
  axis_alt = jp.ones(3).at[jp.argmax(axis0)].set(0)
  normal_alt = jp.cross(axis0, axis_alt)
  normal_alt = math.normalize(normal_alt)

  normal = jp.where(nrm < mujoco.mjMINVAL, normal_alt, normal)

  # 2nd axis
  axis1 = jp.cross(normal, axis0)
  axis1 = math.normalize(axis1)

  # set geom dependent axes
  axis0 = jp.where(is_sphere, axis0, jp.array([1.0, 0.0, 0.0]))
  axis1 = jp.where(is_sphere, axis1, jp.array([0.0, 1.0, 0.0]))

  # project points in 2D frame: p => d
  d = jp.array([
      jp.dot(p0, axis0),
      jp.dot(p0, axis1),
      jp.dot(p1, axis0),
      jp.dot(p1, axis1),
  ])

  # compute sidesite projection
  s = xmat.T @ (side - xpos)
  sd = jp.array([jp.dot(s, axis0), jp.dot(s, axis1)])
  sd = math.normalize(sd) * size

  # TODO(taylorhowell): implement wrap_inside for internal wrapping case
  wlen, pnt = wrap_circle(d, sd, sidesite, size)
  no_wrap = wlen < 0

  # reconstruct 3D points in local frame: res
  res0 = axis0 * pnt[0] + axis1 * pnt[1]
  res1 = axis0 * pnt[2] + axis1 * pnt[3]
  res = jp.concatenate([res0, res1])

  # perform correction for cylinder case
  l0 = jp.sqrt(
      (p0[0] - res[0]) * (p0[0] - res[0]) + (p0[1] - res[1]) * (p0[1] - res[1])
  )
  l1 = jp.sqrt(
      (p1[0] - res[3]) * (p1[0] - res[3]) + (p1[1] - res[4]) * (p1[1] - res[4])
  )
  r2 = p0[2] + (p1[2] - p0[2]) * l0 / (l0 + wlen + l1)
  r5 = p0[2] + (p1[2] - p0[2]) * (l0 + wlen) / (l0 + wlen + l1)
  height = jp.abs(r5 - r2)

  wlen = jp.where(is_sphere, wlen, jp.sqrt(wlen * wlen + height * height))
  res = jp.where(
      is_sphere, res, res.at[jp.array([2, 5])].set(jp.concatenate([r2, r5]))
  )

  # map wrap points back to global frame
  wpnt0 = xmat @ res[:3] + xpos
  wpnt1 = xmat @ res[3:] + xpos

  # check cases for no wrap
  invalid = close_to_origin | no_wrap

  wlen = jp.where(invalid, -1, wlen)
  wpnt0 = jp.where(invalid, jp.zeros(3), wpnt0)
  wpnt1 = jp.where(invalid, jp.zeros(3), wpnt1)

  return wlen, wpnt0, wpnt1


def muscle_gain_length(
    length: jax.Array, lmin: jax.Array, lmax: jax.Array
) -> jax.Array:
  """Normalized muscle length-gain curve."""
  # mid-ranges (maximum is at 1.0)
  a = 0.5 * (lmin + 1)
  b = 0.5 * (1 + lmax)

  out0 = 0.5 * jp.square(
      (length - lmin) / jp.maximum(mujoco.mjMINVAL, a - lmin)
  )
  out1 = 1 - 0.5 * jp.square((1 - length) / jp.maximum(mujoco.mjMINVAL, 1 - a))
  out2 = 1 - 0.5 * jp.square((length - 1) / jp.maximum(mujoco.mjMINVAL, b - 1))
  out3 = 0.5 * jp.square(
      (lmax - length) / jp.maximum(mujoco.mjMINVAL, lmax - b)
  )

  out = jp.where(length <= b, out2, out3)
  out = jp.where(length <= 1, out1, out)
  out = jp.where(length <= a, out0, out)
  out = jp.where((lmin <= length) & (length <= lmax), out, 0.0)

  return out


def muscle_gain(
    length: jax.Array,
    vel: jax.Array,
    lengthrange: jax.Array,
    acc0: jax.Array,
    prm: jax.Array,
) -> jax.Array:
  """Muscle active force."""
  # unpack parameters
  lrange = prm[:2]
  force, scale, lmin, lmax, vmax, _, fvmax = prm[2:9]

  force = jp.where(force < 0, scale / jp.maximum(mujoco.mjMINVAL, acc0), force)

  # optimum length
  L0 = (lengthrange[1] - lengthrange[0]) / jp.maximum(  # pylint:disable=invalid-name
      mujoco.mjMINVAL, lrange[1] - lrange[0]
  )

  # normalized length and velocity
  L = lrange[0] + (length - lengthrange[0]) / jp.maximum(mujoco.mjMINVAL, L0)  # pylint:disable=invalid-name
  V = vel / jp.maximum(mujoco.mjMINVAL, L0 * vmax)  # pylint:disable=invalid-name

  # length curve
  FL = muscle_gain_length(L, lmin, lmax)  # pylint:disable=invalid-name

  # velocity curve
  y = fvmax - 1
  FV = fvmax  # pylint:disable=invalid-name
  FV = jp.where(  # pylint:disable=invalid-name
      V <= y, fvmax - jp.square(y - V) / jp.maximum(mujoco.mjMINVAL, y), FV
  )
  FV = jp.where(V <= 0, jp.square(V + 1), FV)  # pylint:disable=invalid-name
  FV = jp.where(V <= -1, 0, FV)  # pylint:disable=invalid-name

  # compute FVL and scale, make it negative
  return -force * FL * FV


def muscle_bias(
    length: jax.Array, lengthrange: jax.Array, acc0: jax.Array, prm: jax.Array
) -> jax.Array:
  """Muscle passive force."""
  # unpack parameters
  lrange = prm[:2]
  force, scale, _, lmax, _, fpmax = prm[2:8]

  force = jp.where(force < 0, scale / jp.maximum(mujoco.mjMINVAL, acc0), force)

  # optimum length
  L0 = (lengthrange[1] - lengthrange[0]) / jp.maximum(  # pylint:disable=invalid-name
      mujoco.mjMINVAL, lrange[1] - lrange[0]
  )

  # normalized length
  L = lrange[0] + (length - lengthrange[0]) / jp.maximum(mujoco.mjMINVAL, L0)  # pylint:disable=invalid-name

  # half-quadratic to (L0 + lmax) / 2, linear beyond
  b = 0.5 * (1 + lmax)

  out1 = (
      -force
      * fpmax
      * 0.5
      * jp.square((L - 1) / jp.maximum(mujoco.mjMINVAL, b - 1))
  )
  out2 = -force * fpmax * (0.5 + (L - b) / jp.maximum(mujoco.mjMINVAL, b - 1))

  out = jp.where(L <= b, out1, out2)
  out = jp.where(L <= 1, 0.0, out)

  return out


def muscle_dynamics_timescale(
    dctrl: jax.Array,
    tau_act: jax.Array,
    tau_deact: jax.Array,
    smoothing_width: jax.Array,
) -> jax.Array:
  """Muscle time constant with optional smoothing."""
  # hard switching
  tau_hard = jp.where(dctrl > 0, tau_act, tau_deact)

  def _sigmoid(x):
    # sigmoid function over 0 <= x <= 1 using quintic polynomial
    # sigmoid: f(x) = 6 * x^5 - 15 * x^4 + 10 * x^3
    # solution of f(0) = f'(0) = f''(0) = 0, f(1) = 1, f'(1) = f''(1) = 0
    return jp.clip(x**3 * (3 * x * (2 * x - 5) + 10), 0, 1)

  # smooth switching
  # scale by width, center around 0.5 midpoint, rescale to bounds
  tau_smooth = tau_deact + (tau_act - tau_deact) * _sigmoid(
      dctrl / smoothing_width + 0.5
  )

  return jp.where(smoothing_width < mujoco.mjMINVAL, tau_hard, tau_smooth)


def muscle_dynamics(
    ctrl: jax.Array, act: jax.Array, prm: jax.Array
) -> jax.Array:
  """Muscle activation dynamics."""
  # clamp control
  ctrlclamp = jp.clip(ctrl, 0, 1)

  # clamp activation
  actclamp = jp.clip(act, 0, 1)

  # compute timescales as in Millard et at. (2013)
  # https://doi.org/10.1115/1.4023390
  tau_act = prm[0] * (0.5 + 1.5 * actclamp)  # activation timescale
  tau_deact = prm[1] / (0.5 + 1.5 * actclamp)  # deactivation timescale
  smoothing_width = prm[2]  # width of smoothing sigmoid
  dctrl = ctrlclamp - act  # excess excitation

  tau = muscle_dynamics_timescale(dctrl, tau_act, tau_deact, smoothing_width)

  # filter output
  return dctrl / jp.maximum(mujoco.mjMINVAL, tau)
