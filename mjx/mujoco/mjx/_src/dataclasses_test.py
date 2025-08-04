# Copyright 2025 DeepMind Technologies Limited
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
"""Tests for custom PyTreeNode object."""

from absl.testing import absltest
import jax
from jax import numpy as jp
from mujoco.mjx._src import dataclasses
import numpy as np


class Obj(dataclasses.PyTreeNode):
  a: int
  b: np.ndarray
  c: tuple[int, ...]
  d: tuple[np.ndarray, ...]
  e: jax.Array
  f: tuple[jax.Array, ...]


class DataclassesTest(absltest.TestCase):

  def test_pytree_structure(self):
    obj = Obj(
        a=1,
        b=np.array([1, 2, 3]),
        c=(4, 5, 6),
        d=(np.array([7, 8]), np.array([9, 10])),
        e=jax.numpy.array([11, 12]),
        f=(jax.numpy.array([13, 14]), jax.numpy.array([15, 16])),
    )

    data, meta = jax.tree_util.tree_flatten_with_path(obj)

    # data fields
    self.assertLen(data, 3)
    self.assertEqual(data[0][0][0].name, 'e')
    np.testing.assert_array_equal(data[0][1], jp.array([11, 12]))
    self.assertEqual(data[1][0][0].name, 'f')
    np.testing.assert_array_equal(data[1][1], jp.array([13, 14]))
    self.assertEqual(data[2][0][0].name, 'f')
    np.testing.assert_array_equal(data[2][1], jp.array([15, 16]))

    # meta fields
    unflattened_meta = meta.unflatten([x[1] for x in data])
    self.assertEqual(unflattened_meta.a, 1)
    np.testing.assert_array_equal(unflattened_meta.b, np.array([1, 2, 3]))
    self.assertEqual(unflattened_meta.c, (4, 5, 6))
    np.testing.assert_array_equal(unflattened_meta.d[0], np.array([7, 8]))
    np.testing.assert_array_equal(unflattened_meta.d[1], np.array([9, 10]))

    # ensure hashable meta
    hash(meta)


if __name__ == '__main__':
  absltest.main()
