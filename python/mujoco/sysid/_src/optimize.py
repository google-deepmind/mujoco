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

"""Optimization routines for system identification."""

from collections.abc import Callable
from typing import Any, Literal

from absl import logging
from mujoco import minimize as mujoco_minimize
from mujoco.sysid._src import parameter
import numpy as np
import scipy.optimize as scipy_optimize
import scipy.special


def _scipy_least_squares(
    x0: np.ndarray,
    residual_fn: Callable[..., Any],
    bounds: tuple[np.ndarray, np.ndarray],
    use_mujoco_jac: bool = False,
    **kwargs,
) -> scipy_optimize.OptimizeResult:
  """Run scipy least_squares with optional MuJoCo finite-difference Jacobian."""
  max_nfev = kwargs.pop("max_iters", 200)
  if kwargs.pop("verbose", True):
    verbose = 2
  else:
    verbose = 0
  x_scale = kwargs.pop("x_scale", "jac")
  loss = kwargs.pop("loss", "linear")

  jac_arg: str | Callable[..., Any]
  if use_mujoco_jac:
    # This is the default step size for finite difference used in
    # scipy's least_squares and mujoco's minimize finite difference
    # https://github.com/scipy/scipy/blob/91e18f3bd355477b
    # 8b7747ec82d70ac98ffd2422/scipy/optimize/_numdiff.py#L404
    eps = np.finfo(np.float64).eps ** 0.5
    if "diff_step" in kwargs:
      eps = kwargs.pop("diff_step")

    def _jac_fn(x):
      return mujoco_minimize.jacobian_fd(
          residual=residual_fn,
          x=x.reshape((-1, 1)),
          r=residual_fn(x).reshape((-1, 1)),
          eps=eps,
          n_res=0,
          bounds=[bounds[0].reshape((-1, 1)), bounds[1].reshape((-1, 1))],
      )[0]

    jac_arg = _jac_fn
  else:
    jac_arg = "2-point"

  return scipy_optimize.least_squares(
      residual_fn,
      x0,
      bounds=bounds,
      max_nfev=max_nfev,
      verbose=verbose,
      x_scale=x_scale,
      loss=loss,
      jac=jac_arg,  # pyright: ignore[reportArgumentType]
      **kwargs,
  )


def _mujoco_least_squares(
    x0: np.ndarray,
    residual_fn: Callable[..., Any],
    bounds: tuple[np.ndarray, np.ndarray],
    **kwargs,
) -> scipy_optimize.OptimizeResult:
  """Run MuJoCo's native least_squares optimizer."""
  if kwargs.pop("verbose", True):
    verbose = mujoco_minimize.Verbosity.FULLITER
  else:
    verbose = mujoco_minimize.Verbosity.SILENT
  max_iter = kwargs.pop("max_iters", 200)
  x, log = mujoco_minimize.least_squares(
      x0=x0,
      bounds=bounds,
      residual=residual_fn,
      verbose=verbose,
      max_iter=max_iter,
      **kwargs,
  )

  # If verbose, return the full optimization log.
  extras = {}
  if verbose == mujoco_minimize.Verbosity.FULLITER:
    extras["objective"] = [entry.objective for entry in log]
    extras["candidate"] = [entry.candidate[:, 0] for entry in log]

  return scipy_optimize.OptimizeResult(
      x=x,
      jac=log[-1].jacobian,
      grad=log[-1].grad,
      extras=extras,
  )


def _dispatch_optimizer(
    x0: np.ndarray,
    residual_fn: Callable[..., Any],
    bounds: tuple[np.ndarray, np.ndarray],
    optimizer: Literal["scipy", "mujoco", "scipy_parallel_fd"],
    **kwargs,
) -> scipy_optimize.OptimizeResult:
  """Dispatch to the appropriate least-squares backend."""
  if optimizer in ["scipy", "scipy_parallel_fd"]:
    return _scipy_least_squares(
        x0,
        residual_fn,
        bounds,
        use_mujoco_jac=optimizer == "scipy_parallel_fd",
        **kwargs,
    )
  elif optimizer == "mujoco":
    return _mujoco_least_squares(x0, residual_fn, bounds, **kwargs)
  else:
    raise ValueError(
        f"Unsupported optimizer: '{optimizer}'. Expected one of: 'scipy',"
        " 'scipy_parallel_fd', or 'mujoco'."
    )


def optimize(
    initial_params: parameter.ParameterDict,
    residual_fn: Callable[..., Any],
    optimizer: Literal["scipy", "mujoco", "scipy_parallel_fd"] = "mujoco",
    verbose: bool = True,
    **optimizer_kwargs,
) -> tuple[parameter.ParameterDict, scipy_optimize.OptimizeResult]:
  """Run nonlinear least-squares optimization on the residual.

  Args:
    initial_params: Starting parameter values and bounds.
    residual_fn: Callable with signature ``(x, params) -> (residuals, ...)`` as
      returned by :func:`build_residual_fn`.
    optimizer: Backend — ``"mujoco"`` (default), ``"scipy"``, or
      ``"scipy_parallel_fd"`` (scipy with MuJoCo finite-difference Jacobian).
    verbose: If True, log parameter comparison table after optimization.
    **optimizer_kwargs: Forwarded to the backend (e.g. ``max_iters``,
      ``verbose``, ``loss``).

  Returns:
    ``(opt_params, opt_result)`` — the optimized ParameterDict and a
    ``scipy.optimize.OptimizeResult`` with at least ``x``, ``jac``, ``grad``.
  """
  x0 = initial_params.as_vector()
  bounds = initial_params.get_bounds()
  opt_params = initial_params.copy()

  # Check if there are any parameters to optimize.
  if not opt_params or opt_params.size == 0:
    logging.warning(
        "The ParameterDict is empty or contains only frozen Parameters. "
        "Please declare all Parameters that need to be optimized."
    )
    return opt_params, scipy_optimize.OptimizeResult(
        x=x0,
        jac=np.zeros((0, x0.shape[0])),
        grad=np.zeros_like(x0),
        extras={},
    )

  def optimized_residual_fn(x):
    residuals, _, _ = residual_fn(x, opt_params)
    return np.concatenate(residuals)

  opt_result = _dispatch_optimizer(
      x0, optimized_residual_fn, bounds, optimizer, **optimizer_kwargs
  )

  opt_params.update_from_vector(opt_result.x)

  if verbose:
    logging.info(
        "\n%s",
        opt_params.compare_parameters(
            initial_params.as_vector(),
            opt_params.as_vector(),
            measured_params=initial_params.as_nominal_vector(),
        ),
    )

  return opt_params, opt_result


def calculate_intervals(
    residuals_star,
    J,
    alpha=0.05,
    lambda_zero_thresh=1e-15,
    v_zero_thresh=1e-8,
):
  """Calculate confidence intervals from the Jacobian at the optimum.

  Args:
    residuals_star: List of residual arrays at the optimum.
    J: Jacobian matrix at the optimum, shape ``(n_residuals, n_params)``.
    alpha: Significance level for the confidence intervals.
    lambda_zero_thresh: Threshold below which eigenvalues are treated as zero.
    v_zero_thresh: Threshold below which eigenvector elements are treated as
      zero.

  Returns:
    ``(Sigma_X, intervals)`` — the parameter covariance matrix and the
    half-width confidence intervals for each parameter.
  """
  if J is None or J.size == 0:
    return np.empty((0, 0)), np.empty((0,))

  # TODO(levi): account for per sensor variance
  # Estimate sensor variance by assuming a good model fit, so
  # remaining variance in the residual is due to sensor noise.
  # Dividing by n - p is an unbiased estimate of the noise.
  final_r = np.concatenate(residuals_star)
  s2 = np.dot(final_r, final_r) / (final_r.size - J.shape[1])
  H = J.T @ J

  # Calculate the diagonals of the inverse of H
  # using the observation that division by zero
  # of eig(H) close to zero is canceled by numerically
  # zero elements of the eigenvectors
  # That is numerically zero eigenvalues only
  # cause a confidence bound to be infinite if that eigenvalue
  # has a numerically non-zero effect on the considered parameter
  lamb, V = np.linalg.eigh(H)
  lamb_max = np.max(lamb)
  diag_inv_H = []
  for j in range(H.shape[0]):
    inv_H_jj = 0.0
    v_j_max = np.max(np.abs(V[:, j]))
    for i in range(H.shape[0]):
      lambda_i = lamb[i]
      if lambda_i / lamb_max < lambda_zero_thresh:
        lambda_i = 0.0

      v_j_i = V[j, i]
      if np.abs(v_j_i / v_j_max) < v_zero_thresh:
        v_j_i = 0.0

      if lambda_i == 0.0 and v_j_i != 0.0:
        inv_H_jj += np.inf
      elif lambda_i == 0.0 and v_j_i == 0.0:
        pass
      else:
        inv_H_jj += v_j_i**2 / lambda_i
    diag_inv_H.append(inv_H_jj)
  diag_inv_H = np.array(diag_inv_H)

  # In general eigenvalue decomposition should be more accurate
  # than calculating the inverse of H using a general method
  # TODO(levi): expand the eigenvalue/eigenvector element
  # cancelation above to the full inverse matrix
  # inv_H = V @ np.diag(np.divide(
  #     1, lamb, out=np.inf*np.zeros_like(lamb),
  #     where=lamb != 0.0)) @ V.T
  lamb[lamb == 0] = lambda_zero_thresh
  inv_H = V @ np.diag(1 / lamb) @ V.T
  # print('inv test')
  # print(np.diag(inv_H @ H))
  # print(np.diag(np.linalg.inv(H) @ H)))
  Sigma_X = s2 * inv_H
  intervals = np.sqrt(diag_inv_H * s2) * scipy.special.stdtrit(
      final_r.size - J.shape[1], 1 - alpha / 2
  )
  return Sigma_X, intervals
