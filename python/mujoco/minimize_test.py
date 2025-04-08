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

import io

from absl.testing import absltest
from mujoco import minimize
import numpy as np


class MinimizeTest(absltest.TestCase):

  def test_basic(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    out = io.StringIO()
    x0 = np.array((0.0, 0.0))
    x, _ = minimize.least_squares(x0, residual, output=out)
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('norm(gradient) < tol', out.getvalue())

  def test_start_at_minimum(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    out = io.StringIO()
    x0 = np.array((1.0, 1.0))
    x, _ = minimize.least_squares(x0, residual, output=out)
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('norm(gradient) < tol', out.getvalue())
    self.assertIn('exact minimum found', out.getvalue())

  def test_jac_callback(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    def jacobian(x, r):
      del r  # Unused.
      return np.array([[-1, 0], [-20 * x[0, 0], 10]])

    x0 = np.array((0.0, 0.0))
    out = io.StringIO()
    x, _ = minimize.least_squares(
        x0, residual, jacobian=jacobian, output=out, check_derivatives=True
    )
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('norm(gradient) < tol', out.getvalue())
    self.assertIn('Jacobian matches', out.getvalue())

    # Try with bad Jacobian, ask least_squares to check it.
    def bad_jacobian(x, r):
      del r  # Unused.
      return np.array([[-1, 0], [-20 * x[0, 0], 15]])

    with self.assertRaisesRegex(ValueError, r'\bJacobian does not match\b'):
      minimize.least_squares(
          x0,
          residual,
          jacobian=bad_jacobian,
          output=out,
          check_derivatives=True,
      )

  def test_max_iter(self) -> None:
    dim = 20  # High-D Rosenbrock

    def residual(x):
      res0 = [1 - x[i, :] for i in range(dim - 1)]
      res1 = [10 * (x[i, :] - x[i + 1, :] ** 2) for i in range(dim - 1)]
      return np.asarray(res0 + res1)

    # Fail to reach minimum after 20 iterations.
    x0 = np.zeros(dim)
    out = io.StringIO()
    minimize.least_squares(x0, residual, max_iter=20, output=out)
    self.assertIn('maximum iterations', out.getvalue())

    # Succeed after 100 iterations (default).
    x, _ = minimize.least_squares(x0, residual)
    expected_x = np.ones(20)
    np.testing.assert_array_almost_equal(x, expected_x)

  def test_bounds(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    out = io.StringIO()
    x0 = np.array((0.0, 0.0))
    expected_x = np.array((1.0, 1.0))

    bounds_types = {
        'inbounds': [np.array((-2.0, -2.0)), np.array((2.0, 2.0))],
        'onlower': [np.array((-2.0, 2.0)), np.array((0.5, 3.0))],
        'onupper': [np.array((-2.0, -2.0)), np.array((0.5, 2.0))],
    }

    # In bounds finds true minimum.
    x, _ = minimize.least_squares(
        x0, residual, bounds=bounds_types['inbounds'], output=out
    )
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('norm(gradient) < tol', out.getvalue())

    # Test different bounds conditions.
    for bounds in bounds_types.values():
      out = io.StringIO()
      x, trace = minimize.least_squares(
          x0,
          residual,
          bounds=bounds,
          output=out,
          verbose=minimize.Verbosity.FULLITER,
      )
      self.assertIn('norm(gradient) < tol', out.getvalue())
      grad = trace[-1].grad
      # If x_i is on the boundary, gradient points out, otherwise it is 0.
      for i, xi in enumerate(x):
        if xi == bounds[0][i]:
          self.assertGreater(grad[i], 0)
        elif xi == bounds[1][i]:
          self.assertLess(grad[i], 0)
        else:
          self.assertAlmostEqual(grad[i].item(), 0, places=4)

  def test_bad_bounds(self) -> None:
    def residual(x):
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

  def test_iter_callback(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    out = io.StringIO()

    def iter_callback(trace):
      print(f'Hello iteration {len(trace)}!', file=out)

    x0 = np.array((0.0, 0.0))
    x, _ = minimize.least_squares(
        x0, residual, output=out, iter_callback=iter_callback
    )
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('Hello iteration 3!', out.getvalue())

  def test_norm(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    p = 0.01  # Smoothing radius for smooth-L2 norm.

    class SmoothL2(minimize.Norm):

      def value(self, r):
        return np.sqrt((r.T @ r).item() + p * p) - p

      def grad_hess(self, r, proj):
        s = np.sqrt((r.T @ r).item() + p * p)
        y_r = r / s
        grad = proj.T @ y_r
        y_rr = (np.eye(r.size) - y_r @ y_r.T) / s
        hess = proj.T @ y_rr @ proj
        return grad, hess

    out = io.StringIO()
    x0 = np.array((0.0, 0.0))
    x, _ = minimize.least_squares(
        x0, residual, norm=SmoothL2(), output=out, check_derivatives=True
    )
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('norm(dx) < tol', out.getvalue())
    self.assertIn('User-provided norm gradient matches', out.getvalue())
    self.assertIn('User-provided norm Hessian matches', out.getvalue())

    class SmoothL2BadGrad(minimize.Norm):

      def value(self, r):
        return np.sqrt((r.T @ r).item() + p * p) - p

      def grad_hess(self, r, proj):
        s = np.sqrt((r.T @ r).item() + p * p)
        y_r = r / s
        grad = proj.T @ (y_r + 0.001)  # 0.001 is erronous.
        y_rr = (np.eye(r.size) - y_r @ y_r.T) / s
        hess = proj.T @ y_rr @ proj
        return grad, hess

    with self.assertRaisesRegex(ValueError, r'\bgradient does not match\b'):
      minimize.least_squares(
          x0,
          residual,
          norm=SmoothL2BadGrad(),
          output=out,
          check_derivatives=True,
      )

    class SmoothL2BadHess(minimize.Norm):

      def value(self, r):
        return np.sqrt((r.T @ r).item() + p * p) - p

      def grad_hess(self, r, proj):
        s = np.sqrt((r.T @ r).item() + p * p)
        y_r = r / s
        grad = proj.T @ y_r
        y_rr = (1.001 * np.eye(r.size) - y_r @ y_r.T) / s  # 1.001 is erronous.
        hess = proj.T @ y_rr @ proj
        return grad, hess

    with self.assertRaisesRegex(ValueError, r'\bHessian does not match\b'):
      minimize.least_squares(
          x0,
          residual,
          norm=SmoothL2BadHess(),
          output=out,
          check_derivatives=True,
      )

    class SmoothL2AsymHess(minimize.Norm):

      def value(self, r):
        return np.sqrt((r.T @ r).item() + p * p) - p

      def grad_hess(self, r, proj):
        s = np.sqrt((r.T @ r).item() + p * p)
        y_r = r / s
        grad = proj.T @ y_r
        y_rr = (np.eye(r.size) - (y_r + 0.0001) @ y_r.T) / s
        hess = proj.T @ y_rr @ proj
        return grad, hess

    with self.assertRaisesRegex(ValueError, r'\bnot symmetric\b'):
      minimize.least_squares(
          x0,
          residual,
          norm=SmoothL2AsymHess(),
          output=out,
          check_derivatives=True,
      )

    class SmoothL2NegHess(minimize.Norm):

      def value(self, r):
        return np.sqrt((r.T @ r).item() + p * p) - p

      def grad_hess(self, r, proj):
        s = np.sqrt((r.T @ r).item() + p * p)
        y_r = r / s
        grad = proj.T @ y_r
        y_rr = -(np.eye(r.size) - y_r @ y_r.T) / s  # Negative-definite.
        hess = proj.T @ y_rr @ proj
        return grad, hess

    with self.assertRaisesRegex(ValueError, r'\bnot positive definite\b'):
      minimize.least_squares(
          x0,
          residual,
          norm=SmoothL2NegHess(),
          output=out,
          check_derivatives=True,
      )

  def test_soft_l1_norm(self) -> None:
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    class SoftL1(minimize.Norm):
      """Implementation of the loss called 'soft_l1' in scipy least_squares."""

      def value(self, r):
        return np.sum(np.sqrt(r**2 + 1) - 1)

      def grad_hess(self, r, proj):
        s = np.sqrt(r**2 + 1)
        y_r = r / s
        grad = proj.T @ y_r
        y_rr = (1 - y_r ** 2) / s
        hess = proj.T @ (y_rr * proj)
        return grad, hess

    out = io.StringIO()
    x0 = np.array((0.0, 0.0))
    x, _ = minimize.least_squares(
        x0, residual, norm=SoftL1(), output=out, check_derivatives=True
    )
    expected_x = np.array((1.0, 1.0))
    np.testing.assert_array_almost_equal(x, expected_x)
    self.assertIn('User-provided norm gradient matches', out.getvalue())
    self.assertIn('User-provided norm Hessian matches', out.getvalue())


if __name__ == '__main__':
  absltest.main()
