# Copyright 2024 DeepMind Technologies Limited
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
"""Tests for minimize.py."""

import io
from typing import Tuple

from absl.testing import absltest
from mujoco import minimize
import numpy as np


class MinimizeTest(absltest.TestCase):

  def test_basic(self) -> None:
    def residual(x: np.ndarray) -> np.ndarray:
      return np.array([1 - x[0], 10 * (x[1] - x[0] ** 2)], dtype=np.float64)

    for central in [False, True]:
      out = io.StringIO()
      x0 = np.array((0.0, 0.0))
      x, _ = minimize.least_squares(x0, residual, output=out, central=central)
      expected_x = np.array((1.0, 1.0))
      np.testing.assert_array_almost_equal(x, expected_x)
      self.assertContainsSubsequence(out.getvalue(), 'norm(dx) < tol')

  def test_start_at_minimum(self) -> None:
    def residual(x: np.ndarray) -> np.ndarray:
      return np.array([1 - x[0], 10 * (x[1] - x[0] ** 2)])

    out = io.StringIO()
    x0 = np.array((1.0, 1.0))
    x, _ = minimize.least_squares(x0, residual, output=out)
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertContainsSubsequence(out.getvalue(), 'norm(dx) < tol')
    self.assertContainsSubsequence(out.getvalue(), 'exact minimum found')

  def test_jac_callback(self) -> None:
    def residual(x: np.ndarray) -> np.ndarray:
      return np.array([1 - x[0], 10 * (x[1] - x[0] ** 2)])

    def jacobian(x: np.ndarray, r: np.ndarray) -> Tuple[float, np.ndarray]:
      del r  # Unused.
      return np.array([[-1, 0], [-20 * x[0], 10]])

    x0 = np.array((0.0, 0.0))
    out = io.StringIO()
    x, _ = minimize.least_squares(x0, residual, jacobian=jacobian, output=out)
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertContainsSubsequence(out.getvalue(), 'norm(dx) < tol')

    # Try with bad Jacobian, expect no improvement.
    def jac_bad1(x: np.ndarray, r: np.ndarray) -> Tuple[float, np.ndarray]:
      return -jacobian(x, r)
    out1 = io.StringIO()
    minimize.least_squares(x0, residual, jacobian=jac_bad1, output=out1)
    self.assertContainsSubsequence(out1.getvalue(), 'insufficient reduction')

  def test_max_iter(self) -> None:
    dim = 20  # High-D Rosenbrock

    def residual(x: np.ndarray) -> np.ndarray:
      res0 = [1 - x[i] for i in range(dim - 1)]
      res1 = [10 * (x[i] - x[i + 1] ** 2) for i in range(dim - 1)]
      return np.asarray(res0 + res1)

    # Fail to reach minimum after 20 iterations.
    x0 = np.zeros(dim)
    out = io.StringIO()
    minimize.least_squares(x0, residual, max_iter=20, output=out)
    self.assertContainsSubsequence(out.getvalue(), 'maximum iterations')

    # Succeed after 100 iterations (default).
    x, _ = minimize.least_squares(x0, residual)
    expected_x = np.ones(20)
    np.testing.assert_array_almost_equal(x, expected_x)

  def test_bounds(self) -> None:
    def residual(x: np.ndarray) -> np.ndarray:
      return np.array([1 - x[0], 10 * (x[1] - x[0] ** 2)])

    out = io.StringIO()
    x0 = np.array((0.0, 0.0))
    expected_x = np.array((1.0, 1.0))

    bounds_types = {'inbounds': [np.array((-2.0, -2.0)), np.array((2.0, 2.0))],
                    'onlower': [np.array((-2.0, 2.0)), np.array((0.5, 3.0))],
                    'onupper': [np.array((-2.0, -2.0)), np.array((0.5, 2.0))]}

    # In bounds finds true minimum.
    x, _ = minimize.least_squares(x0, residual, bounds=bounds_types['inbounds'],
                                  output=out)
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertContainsSubsequence(out.getvalue(), 'norm(dx) < tol')

    # Test different bounds conditions.
    verbose = minimize.Verbosity.FULLITER

    for central in [False, True]:
      for bounds in bounds_types.values():
        out = io.StringIO()
        x, trace = minimize.least_squares(
            x0,
            residual,
            bounds=bounds,
            output=out,
            central=central,
            verbose=verbose,
        )
        self.assertContainsSubsequence(out.getvalue(), ' < tol')
        grad = trace[-2].jacobian.T @ trace[-2].residual
        # If x_i is on the boundary, gradient points out, otherwise it is 0.
        for i, xi in enumerate(x):
          if xi == bounds[0][i]:
            self.assertGreater(grad[i], 0)
          elif xi == bounds[1][i]:
            self.assertLess(grad[i], 0)
          else:
            self.assertAlmostEqual(grad[i], 0, places=4)

  def test_bad_bounds(self) -> None:
    def residual(x: np.ndarray) -> np.ndarray:
      return np.array([1 - x[0], 10 * (x[1] - x[0] ** 2)])

    out = io.StringIO()
    x0 = np.array((0.0, 0.0))

    bad_bounds = [
        [0, 1, 2],
        [np.array((-2, 2, 0)), np.array((0.5, 3, 4))],
        [np.array((-2, 2, 0)), np.array((0.5, 3, np.inf))],
        [np.array((-2, 2, 0)), np.array((-5, 3, 6))],
    ]

    for bounds in bad_bounds:
      with self.assertRaises(ValueError):
        minimize.least_squares(x0, residual, bounds=bounds, output=out)

if __name__ == '__main__':
  absltest.main()
