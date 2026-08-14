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
"""Tests the collision driver."""

import dataclasses

from absl.testing import absltest
from absl.testing import parameterized
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Contact
# pylint: enable=g-importing-member
import numpy as np


def _assert_attr_eq(mjx_d, mj_d, attr, name, atol):
  if attr == 'efc_address':
    # contact order not guaranteed to match
    np.testing.assert_array_equal(
        np.sort(mjx_d.efc_address), np.sort(mj_d.efc_address)
    )
    return
  err_msg = f'mismatch: {attr} in run: {name}'
  mjx_d, mj_d = getattr(mjx_d, attr), getattr(mj_d, attr)
  if attr == 'frame':
    mj_d = mj_d.reshape((-1, 3, 3))
  if mjx_d.shape != mj_d.shape:
    raise AssertionError(f'{attr} shape mismatch: {mjx_d.shape}, {mj_d.shape}')
  np.testing.assert_allclose(mjx_d, mj_d, err_msg=err_msg, atol=atol)


class CollisionDriverIntegrationTest(parameterized.TestCase):

  @parameterized.parameters(list(range(256)))
  def test_collision_driver(self, seed):
    enable_contact = False if seed == 0 else True
    mjcf = test_util.create_mjcf(
        seed,
        body_pos=(0.0, 0.0, 0.14),
        disable_actuation_pct=100,
        root_always_free=True,
        min_trees=1,
        max_trees=5,
        max_tree_depth=1,
        enable_contact=enable_contact,
    )

    m = mujoco.MjModel.from_xml_string(mjcf)
    mx = mjx.put_model(m)
    d = mujoco.MjData(m)
    dx = mjx.put_data(m, d)

    mujoco.mj_step(m, d)
    collision_jit_fn = jax.jit(mjx.collision)
    kinematics_jit_fn = jax.jit(mjx.kinematics)
    dx = kinematics_jit_fn(mx, dx)
    dx = collision_jit_fn(mx, dx)

    if not d.contact.geom1.shape[0]:
      self.assertTrue((dx.contact.dist > 0).all())
      return  # no contacts to test

    # re-order MJX contacts to match MJ order
    idx_mj = list(zip(d.contact.geom1, d.contact.geom2))
    idx_mjx = list(zip(dx.contact.geom1, dx.contact.geom2))
    idx_mjx = [tuple(np.array(i)) for i in idx_mjx]
    self.assertSequenceEqual(set(idx_mjx), set(idx_mj))
    idx = sorted(range(len(idx_mj)), key=lambda x: idx_mj.index(idx_mjx[x]))

    mjx_contact = jax.tree_util.tree_map(
        lambda x: x.take(np.array(idx), axis=0), dx.contact
    )
    mjx_contact = mjx_contact.replace(
        dim=mjx_contact.dim[idx], efc_address=mjx_contact.efc_address[idx]
    )
    for field in dataclasses.fields(Contact):
      _assert_attr_eq(mjx_contact, d.contact, field.name, seed, 1e-7)


if __name__ == '__main__':
  absltest.main()
