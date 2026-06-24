# Copyright 2026 DeepMind Technologies Limited
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
"""Tests for generated MJX Warp types."""

from absl.testing import absltest
from absl.testing import parameterized
import numpy as np

from mujoco.mjx.warp import types


class TileSetTest(parameterized.TestCase):

  @parameterized.parameters(1, 3)
  def test_structural_equality_and_hash(self, count):
    tile_a = types.TileSet(np.arange(count) * 6, 16)
    tile_b = types.TileSet(np.arange(count) * 6, 16)
    tile_c = types.TileSet(np.arange(count) * 6 + 1, 16)

    self.assertEqual(tile_a, tile_b)
    self.assertEqual(hash(tile_a), hash(tile_b))
    self.assertNotEqual(tile_a, tile_c)


if __name__ == '__main__':
  absltest.main()
