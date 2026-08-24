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
"""Regression tests for MJX ray quadratics."""

from absl.testing import absltest
import jax
from jax import numpy as jp
from mujoco.mjx._src import ray


class RayQuadTest(absltest.TestCase):

  def test_negative_discriminant_returns_no_intersection(self):
    x0, x1 = jax.jit(ray._ray_quad)(
        jp.array(1.0), jp.array(0.0), jp.array(1.0)
    )

    self.assertTrue(bool(jp.isinf(x0)))
    self.assertTrue(bool(jp.isinf(x1)))
    self.assertFalse(bool(jp.isnan(x0)))
    self.assertFalse(bool(jp.isnan(x1)))


if __name__ == '__main__':
  absltest.main()
