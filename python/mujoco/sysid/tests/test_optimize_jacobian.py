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

"""Regressions for forwarding Jacobian options to the SciPy sysid backend."""

from mujoco.sysid._src.optimize import optimize
from mujoco.sysid._src.parameter import Parameter
from mujoco.sysid._src.parameter import ParameterDict
import numpy as np
import pytest


def _linear_problem(size):
  matrix = np.vstack((np.diag(np.arange(1, size + 1)), np.ones(size)))
  expected = np.linspace(0.5, 1.5, size)
  params = ParameterDict()
  params.add(
      Parameter(
          "values", np.zeros(size), -5 * np.ones(size), 5 * np.ones(size)
      )
  )
  calls = {"residual": 0, "jacobian": 0}

  def residual(x, current_params):
    calls["residual"] += 1
    current_params.update_from_vector(x)
    return [matrix @ current_params.as_vector() - matrix @ expected], None, None

  def jacobian(x):
    del x
    calls["jacobian"] += 1
    return matrix

  return params, residual, jacobian, expected, calls


@pytest.mark.parametrize("jacobian_kind", ["analytic", "3-point"])
def test_scipy_backend_forwards_explicit_jacobian(jacobian_kind):
  params, residual, jacobian, expected, calls = _linear_problem(2)
  jac = jacobian if jacobian_kind == "analytic" else jacobian_kind

  fitted, result = optimize(
      params, residual, optimizer="scipy", verbose=False, jac=jac
  )

  assert result.success
  np.testing.assert_allclose(fitted.as_vector(), expected, atol=1e-8)
  np.testing.assert_allclose(result.fun, 0.0, atol=1e-8)
  np.testing.assert_array_equal(params.as_vector(), np.zeros(2))
  if jacobian_kind == "analytic":
    assert calls["jacobian"] > 0
    np.testing.assert_array_equal(result.jac, jacobian(result.x))


def test_scipy_backend_keeps_default_finite_differences():
  params, residual, _, expected, calls = _linear_problem(2)

  fitted, result = optimize(
      params, residual, optimizer="scipy", verbose=False
  )

  assert result.success
  assert calls["jacobian"] == 0
  np.testing.assert_allclose(fitted.as_vector(), expected, atol=1e-8)


def test_analytic_jacobian_avoids_per_parameter_residual_evaluations():
  params, residual, _, expected, default_calls = _linear_problem(8)
  default_fit, default_result = optimize(
      params, residual, optimizer="scipy", verbose=False
  )
  params, residual, jacobian, _, analytic_calls = _linear_problem(8)

  analytic_fit, analytic_result = optimize(
      params, residual, optimizer="scipy", verbose=False, jac=jacobian
  )

  assert default_result.success and analytic_result.success
  np.testing.assert_allclose(default_fit.as_vector(), expected, atol=1e-8)
  np.testing.assert_allclose(analytic_fit.as_vector(), expected, atol=1e-8)
  assert analytic_calls["jacobian"] > 0
  assert analytic_calls["residual"] < default_calls["residual"]
