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
"""Tests for io functions."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from mujoco import mjx
from mujoco.mjx._src import test_util


class IoTest(parameterized.TestCase):

  @parameterized.parameters(test_util.TEST_FILES)
  def test_make_data(self, fname):
    """Test that data created by make_data matches data returned by step."""

    m = test_util.load_test_file(fname)
    mx = mjx.device_put(m)
    dx = mjx.make_data(mx)
    dx_step = mjx.step(mx, dx)

    _, dx_treedef = jax.tree_util.tree_flatten(dx)
    _, dx_step_treedef = jax.tree_util.tree_flatten(dx_step)

    self.assertEqual(dx_treedef, dx_step_treedef)


if __name__ == '__main__':
  absltest.main()
