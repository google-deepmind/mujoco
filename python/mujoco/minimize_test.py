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

  def test_jacobian_fd_respects_bounds(self) -> None:
    # jacobian_fd must step inward from whichever bound x sits on, which
    # requires comparing x to the box midpoint (lo+hi)/2, not the half-width.
    eps = np.float64(np.finfo(np.float64).eps ** 0.5)
    cases = {
        'lower_positive_box': (10.0, 20.0, 10.0),
        'upper_positive_box': (10.0, 20.0, 20.0),
        'lower_negative_box': (-20.0, -10.0, -20.0),
        'upper_negative_box': (-20.0, -10.0, -10.0),
    }
    for name, (lo, hi, x0) in cases.items():
      with self.subTest(name):
        bounds = [np.array([[lo]]), np.array([[hi]])]
        x = np.array([[x0]])
        evaluated = []

        def residual(xx, _ev=evaluated):
          _ev.append(np.asarray(xx, dtype=np.float64).copy())
          return np.atleast_2d(np.sum(xx, axis=0))

        minimize.jacobian_fd(residual, x, residual(x), eps, 0, bounds)
        pts = np.concatenate([e.ravel() for e in evaluated])
        self.assertGreaterEqual(pts.min(), lo)
        self.assertLessEqual(pts.max(), hi)

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

  def test_x_scale_default_is_no_op(self) -> None:
    """x_scale=1.0 produces an identical trace to the default unscaled run."""
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    x0 = np.array((0.0, 0.0))
    x_default, trace_default = minimize.least_squares(
        x0, residual, verbose=minimize.Verbosity.SILENT)
    x_one, trace_one = minimize.least_squares(
        x0, residual, x_scale=1.0, verbose=minimize.Verbosity.SILENT)
    np.testing.assert_array_equal(x_default, x_one)
    self.assertEqual(len(trace_default), len(trace_one))

  def test_x_scale_reaches_same_minimum(self) -> None:
    """Both 'jac' and an explicit array reach the unscaled run's minimum."""
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    x0 = np.array((0.0, 0.0))
    x_unscaled, _ = minimize.least_squares(
        x0, residual, verbose=minimize.Verbosity.SILENT)
    x_jac, _ = minimize.least_squares(
        x0, residual, x_scale='jac', verbose=minimize.Verbosity.SILENT)
    x_array, _ = minimize.least_squares(
        x0, residual, x_scale=np.array([10.0, 0.1]),
        verbose=minimize.Verbosity.SILENT)
    np.testing.assert_allclose(x_jac, x_unscaled, atol=1e-6)
    np.testing.assert_allclose(x_array, x_unscaled, atol=1e-6)

  def test_x_scale_validation(self) -> None:
    """Invalid x_scale arguments raise ValueError."""
    def residual(x):
      return x

    x0 = np.array((1.0, 1.0))
    bad_values = [
        'bogus',                          # unknown string
        np.array([1.0, -1.0]),            # non-positive entry
        np.array([np.inf, 1.0]),          # non-finite entry
        np.array([1.0]),                  # wrong shape
    ]
    for bad in bad_values:
      with self.assertRaises(ValueError):
        minimize.least_squares(
            x0, residual, x_scale=bad,
            verbose=minimize.Verbosity.SILENT)

  def test_x_scale_with_bounds(self) -> None:
    """x_scale combined with bounds reaches the constrained optimum."""
    def residual(x):
      return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])

    x0 = np.array((2.0, 0.0))
    bounds = [np.array((1.5, -10.0)), np.array((10.0, 10.0))]
    expected = np.array([1.5, 2.25])

    # Without scaling.
    x_noscale, _ = minimize.least_squares(
        x0, residual, bounds=bounds, verbose=minimize.Verbosity.SILENT)
    np.testing.assert_allclose(x_noscale, expected, atol=1e-4)

    # With fixed x_scale array.
    x_scaled, _ = minimize.least_squares(
        x0, residual, bounds=bounds, x_scale=np.array([1.0, 1e-5]),
        verbose=minimize.Verbosity.SILENT)
    np.testing.assert_allclose(x_scaled, expected, atol=1e-4)

    # With adaptive x_scale='jac'.
    x_jac, _ = minimize.least_squares(
        x0, residual, bounds=bounds, x_scale='jac',
        verbose=minimize.Verbosity.SILENT)
    np.testing.assert_allclose(x_jac, expected, atol=1e-4)

  def test_x_scale_jac_convergence(self) -> None:
    """'jac' scaling converges faster on Powell's badly scaled function."""
    # Powell's badly scaled function (Moré, Garbow, Hillstrom #3):
    #   r1 = 1e4 * x1 * x2 - 1
    #   r2 = exp(-x1) + exp(-x2) - 1.0001
    # The 1e4 multiplier creates Jacobian columns with wildly different
    # norms, making the unscaled LM regularizer ineffective.
    def residual(x):
      return np.stack([
          1e4 * x[0, :] * x[1, :] - 1,
          np.exp(-x[0, :]) + np.exp(-x[1, :]) - 1.0001,
      ])

    x0 = np.array([0.0, 1.0])
    # Analytical solution: x1*x2 = 1e-4, exp(-x1)+exp(-x2) = 1.0001.
    x_star = np.array([1.098159e-5, 9.106146])

    x_default, trace_default = minimize.least_squares(
        x0, residual, verbose=minimize.Verbosity.SILENT, max_iter=400)
    x_jac, trace_jac = minimize.least_squares(
        x0, residual, x_scale='jac', verbose=minimize.Verbosity.SILENT)

    # Both reach the correct minimum.
    np.testing.assert_allclose(x_default, x_star, rtol=1e-4)
    np.testing.assert_allclose(x_jac, x_star, rtol=1e-4)

    # Without scaling the solver needs 331 iterations to converge.
    # With 'jac' scaling it converges in 55: a 6x improvement.
    self.assertGreater(len(trace_default) - 1, 300)
    self.assertLess(len(trace_jac) - 1, 70)


if __name__ == '__main__':
  absltest.main()
