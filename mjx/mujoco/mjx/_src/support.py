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

from typing import Tuple

import jax
from jax import numpy as jp
from mujoco.mjx._src import scan
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member


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


def jac_dif_pair(
    m: Model,
    d: Data,
    pos: jax.Array,
    body_1: jax.Array,
    body_2: jax.Array,
) -> jax.Array:
  """Compute Jacobian difference for two body points."""
  jacp2, _ = jac(m, d, pos, body_2)
  jacp1, _ = jac(m, d, pos, body_1)
  return jacp2 - jacp1


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
