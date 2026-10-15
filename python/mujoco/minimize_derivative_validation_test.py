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
from absl.testing import parameterized
from mujoco import minimize
import numpy as np


class DerivativeValidationTest(parameterized.TestCase):

  @parameterized.product(
      value=(np.nan, np.inf, -np.inf), position=((0, 0), (1, 0), (1, 1))
  )
  def test_nonfinite_analytic_jacobian_is_rejected(self, value, position):
    x = np.array([[1.0], [2.0]])
    jacobian = np.eye(2)
    jacobian[position] = value
    original = jacobian.copy()
    output = io.StringIO()
    with np.errstate(invalid='ignore'):
      with self.assertRaisesRegex(ValueError, 'Jacobian.*finite'):
        minimize.check_jacobian(
            lambda p: p,
            x,
            x.copy(),
            jacobian,
            np.float64(1e-6),
            7,
            output=output,
        )
    self.assertNotIn('matches finite-differences', output.getvalue())
    np.testing.assert_array_equal(jacobian, original)
    np.testing.assert_array_equal(x, [[1.0], [2.0]])

  @parameterized.product(
      value=(np.nan, np.inf, -np.inf), invalid_initial=(False, True)
  )
  def test_nonfinite_numerical_jacobian_is_rejected(
      self, value, invalid_initial
  ):
    x = np.array([[1.0], [2.0]])
    r = x.copy()
    if invalid_initial:
      r[1, 0] = value

    def residual(points):
      result = points.copy()
      if not invalid_initial:
        result[1] = value
      return result

    output = io.StringIO()
    with np.errstate(invalid='ignore'):
      with self.assertRaisesRegex(ValueError, '[Ff]inite-difference.*finite'):
        minimize.check_jacobian(
            residual, x, r, np.eye(2), np.float64(1e-6), 0, output=output
        )
    self.assertNotIn('matches finite-differences', output.getvalue())

  @parameterized.parameters(np.nan, np.inf, -np.inf)
  def test_least_squares_rejects_invalid_user_derivatives(self, value):
    x0 = np.array([1.0])

    def residual(x):
      return x - 3.0

    def jacobian(x, r):
      del x, r
      return np.array([[value]])

    output = io.StringIO()
    with np.errstate(all='ignore'):
      with self.assertRaisesRegex(ValueError, 'Jacobian.*finite'):
        minimize.least_squares(
            x0,
            residual,
            jacobian=jacobian,
            check_derivatives=True,
            max_iter=1,
            output=output,
        )
    self.assertNotIn('Jacobian matches', output.getvalue())
    np.testing.assert_array_equal(x0, [1.0])

  @parameterized.parameters(np.nan, np.inf, -np.inf)
  def test_custom_norm_gradient_cannot_be_certified_when_nonfinite(self, value):
    class InvalidGradient(minimize.Norm):

      def value(self, r):
        return float((r.T @ r).item() / 2)

      def grad_hess(self, r, proj):
        return proj.T @ np.full_like(r, value), proj.T @ proj

    output = io.StringIO()
    with np.errstate(invalid='ignore'):
      with self.assertRaisesRegex(ValueError, 'norm gradient.*finite'):
        minimize.check_norm(
            np.array([[1.0]]),
            InvalidGradient(),
            np.float64(1e-6),
            output=output,
        )
    self.assertNotIn('matches finite-differences', output.getvalue())

  @parameterized.parameters('Jacobian', 'norm gradient', 'custom derivative')
  def test_valid_finite_jacobians_keep_counts_and_success_message(self, name):
    x = np.array([[1.0], [2.0]])
    matrix = np.array([[1.0, -2.0], [0.5, 3.0]])
    output = io.StringIO()
    count = minimize.check_jacobian(
        lambda p: matrix @ p,
        x,
        matrix @ x,
        matrix,
        np.float64(1e-6),
        9,
        output=output,
        name=name,
    )
    self.assertEqual(count, 11)
    self.assertIn(
        f'User-provided {name} matches finite-differences.', output.getvalue()
    )

  def test_finite_mismatch_remains_rejected(self):
    with self.assertRaisesRegex(ValueError, 'Jacobian does not match'):
      minimize.check_jacobian(
          lambda x: x,
          np.array([[1.0]]),
          np.array([[1.0]]),
          np.array([[2.0]]),
          np.float64(1e-6),
          0,
      )

  def test_valid_bounded_optimization_still_converges(self):
    initial = np.array([0.0, 0.0])
    target = np.array([[2.0], [3.0]])

    def residual(x):
      return x - target

    def jacobian(x, r):
      del x, r
      return np.eye(2)

    lower, upper = np.array([-1.0, -1.0]), np.array([1.0, 4.0])
    output = io.StringIO()
    actual, trace = minimize.least_squares(
        initial,
        residual,
        bounds=(lower, upper),
        jacobian=jacobian,
        check_derivatives=True,
        output=output,
    )
    np.testing.assert_allclose(actual, [1.0, 3.0], atol=1e-6)
    self.assertGreater(len(trace), 1)
    self.assertIn('Jacobian matches', output.getvalue())
    np.testing.assert_array_equal(initial, [0.0, 0.0])


if __name__ == '__main__':
  absltest.main()
