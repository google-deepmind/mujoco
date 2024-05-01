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
      mujoco.mjtObj.mjOBJ_PAIR: m.npair,
      mujoco.mjtObj.mjOBJ_EQUALITY: m.neq,
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
      mujoco.mjtObj.mjOBJ_PAIR: m.name_pairadr,
      mujoco.mjtObj.mjOBJ_EQUALITY: m.name_eqadr,
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
