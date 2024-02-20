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
"""Nonlinear Least Squares minimization with box bounds."""

import dataclasses
import enum
import time
from typing import Callable, List, Optional, TextIO, Tuple, Union

import mujoco
import numpy as np


class Verbosity(enum.Enum):
  SILENT = 0
  FINAL = 1
  ITER = 2
  FULLITER = 3


class Status(enum.Enum):
  FACTORIZATION_FAILED = enum.auto()
  NO_IMPORVEMENT = enum.auto()
  MAX_ITER = enum.auto()
  DX_TOL = enum.auto()


_STATUS_MESSAGE = {
    Status.FACTORIZATION_FAILED: 'factorization failed.',
    Status.NO_IMPORVEMENT: 'no improvement found.',
    Status.MAX_ITER: 'maximum iterations reached.',
    Status.DX_TOL: 'norm(step) < tolerance.',
}


@dataclasses.dataclass(frozen=True)
class IterLog:
  """Log of a single iteration of the non-linear least-squares solver.

  Attributes:
    candidate: Value of the decision variable at the beginning this iteration.
    objective: Value of the objective at the candidate.
    reduction: Reduction of the objective during this iteration.
    regularizer: Value of the regularizer used for this iteration.
    residual: Optional value of the residual at the candidate.
    jacobian: Optional value of the Jacobian at the candidate.
    step: Optional change in decision variable during this iteration.
  """

  candidate: np.ndarray
  objective: np.float64
  reduction: np.float64
  regularizer: np.float64
  residual: Optional[np.ndarray] = None
  jacobian: Optional[np.ndarray] = None
  step: Optional[np.ndarray] = None


def jacobian_fd(
    residual: Callable[[np.ndarray], np.ndarray],
    x: np.ndarray,
    r: np.ndarray,
    eps: float,
    bounds: Optional[List[np.ndarray]] = None,
):
  """Finite-difference Jacobian of a residual function.

  Args:
    residual: function that returns the residual for a given point.
    x: point at which to evaluate the Jacobian.
    r: residual at x.
    eps: finite-difference step size.
    bounds: optional pair of lower and upper bounds of the solution.

  Returns:
    jac: Jacobian of the residual at x.
  """
  nx = x.size
  nr = r.size
  jac = np.zeros((nr, nx))
  xh = x.copy()
  for i in range(nx):
    if bounds is not None:
      # Have bounds: scale eps, don't cross bounds.
      lower, upper = bounds
      eps_i = eps * (upper[i] - lower[i])
      if xh[i] < upper[i] - eps_i:
        # Not near upper bound, use forward.
        xh[i] += eps_i
        rh = residual(xh)
        jac[:, i] = (rh - r) / eps_i
      else:
        # Near upper bound, use backward.
        xh[i] -= eps_i
        rh = residual(xh)
        jac[:, i] = (r - rh) / eps_i
    else:
      # No bounds, just use forward fin-diff.
      xh[i] += eps
      rh = residual(xh)
      jac[:, i] = (rh - r) / eps
    xh[i] = x[i]
  return jac


def least_squares(
    x0: np.ndarray,
    residual: Callable[[np.ndarray], np.ndarray],
    bounds: Optional[List[np.ndarray]] = None,
    jacobian: Optional[Callable[[np.ndarray, np.ndarray], np.ndarray]] = None,
    eps: Optional[float] = -6,
    mu_min: Optional[float] = -6,
    mu_max: Optional[float] = 8,
    mu_delta: Optional[float] = 0.5,
    tol: Optional[float] = 1e-7,
    max_iter: Optional[int] = 100,
    verbose: Optional[Union[Verbosity, int]] = Verbosity.ITER,
    output: Optional[TextIO] = None,
) -> Tuple[np.ndarray, List[IterLog]]:
  """Nonlinear Least Squares minimization with box bounds.

  Args:
    x0: initial guess
    residual: function that returns the residual for a given point x.
    bounds: optional pair of lower and upper bounds on the solution.
    jacobian: optional function that returns Jacobian of the residual at a given
      point and residual. If not given, `residual` will be finite-differenced.
    eps: log10 of the perurbation used for automatic finite-differencing.
    mu_min: log10 of the minimum value of the regularizer.
    mu_max: log10 of the maximum value of the regularizer.
    mu_delta: log10 of the factor increasing or decreasing the regularizer.
    tol: termination tolerance on the step size.
    max_iter: maximum number of iterations.
    verbose: verbosity level.
    output: optional file or StringIO to which to print messages.

  Returns:
    x: best solution found
    trace: sequence of solution iterates.
  """
  t_start_total = time.time()

  # Convert verbosity to int.
  verbose = Verbosity(verbose).value

  # Initialize locals.
  x = x0.copy()
  mu = -np.inf  # Optimistically start with no regularization.
  n = x.size
  i = 0
  trace = []
  dx = np.zeros((n,))
  scratch = np.zeros((n, n + 7))
  dx_norm = 0.0
  xnew = np.zeros((n,))
  status = Status.MAX_ITER
  n_res = 0
  n_jac = 0
  t_res = 0.0
  t_jac = 0.0
  t_qp = 0.0

  # Regularization control functions.
  def increase_mu(mu):
    return min(mu_max, max(mu_min, mu_delta + mu))

  def decrease_mu(mu):
    return -np.inf if mu - mu_delta < mu_min else mu - mu_delta

  if bounds is not None:
    # Checks bounds.
    if len(bounds) != 2:
      raise ValueError('bounds must have 2 elements.')
    if bounds[0].size != n or bounds[1].size != n:
      raise ValueError('bounds must have the same size as x0.')
    if not np.all(np.isfinite(bounds[0])) or not np.all(np.isfinite(bounds[1])):
      raise ValueError('bounds must be finite.')
    if not np.all(bounds[0] < bounds[1]):
      raise ValueError('bounds[0] must be smaller than bounds[1].')
    # Clip.
    np.clip(x, bounds[0], bounds[1], out=x)

  # Get initial residual.
  t_start = time.time()
  r = residual(x)
  rnew = r
  t_res += time.time() - t_start
  n_res += 1

  # Minimize.
  for i in range(max_iter):
    if status != Status.MAX_ITER:
      break

    # Get objective y.
    y = 0.5 * r.dot(r)

    # Get Jacobian jac.
    t_start = time.time()
    if jacobian is None:
      jac = jacobian_fd(residual, x, r, 10**eps, bounds)
      t_res += time.time() - t_start
      n_res += n
    else:
      jac = jacobian(x, r)
      t_jac += time.time() - t_start
      n_jac += 1

    # Get gradient, Gauss-Newton Hessian.
    grad = jac.T @ r
    hess = jac.T @ jac
    gnorm = np.linalg.norm(grad)

    # Bounds relative to x
    dbounds = [None, None] if bounds is None else [bounds[0] - x, bounds[1] - x]

    # Find some reduction.
    reduction = -1
    while reduction < 0:
      # Increase mu until factorizabl.
      factorizable = False
      while not factorizable:
        # Formula from https://arxiv.org/abs/2112.02089
        reg = np.sqrt(gnorm * 10**mu) * np.eye(n)
        t_start = time.time()
        nfree = mujoco.mju_boxQP(
            dx, scratch, None, hess + reg, grad, dbounds[0], dbounds[1]
        )
        t_qp += time.time() - t_start
        if nfree > -1:
          factorizable = True
        elif mu >= mu_max:
          status = Status.FACTORIZATION_FAILED
          break
        else:
          mu += mu_delta

      if status != Status.MAX_ITER:
        break

      # New candidate, residual.
      xnew = x + dx
      t_start = time.time()
      rnew = residual(xnew)
      t_res += time.time() - t_start
      n_res += 1

      # New objective, evaluate reduction.
      ynew = 0.5 * rnew.dot(rnew)
      reduction = y - ynew

      if reduction < 0:
        if mu >= mu_max:
          status = Status.NO_IMPORVEMENT
          break
        mu = increase_mu(mu)

    if status != Status.MAX_ITER:
      break

    # Compute reduction ratio.
    expected_reduction = -(grad.dot(dx) + 0.5 * dx.T @ hess @ dx)
    reduction_ratio = 0.0
    if expected_reduction == 0:
      print('Zero expected reduction: exact minimum found?', file=output)
    elif expected_reduction < 0:
      print('Negative expected reduction: should not occur.', file=output)
    else:
      reduction_ratio = reduction / expected_reduction

    # Iteration message.
    if verbose >= Verbosity.ITER.value:
      message = (
          f'iter: {i:<3d}  y: {y:<8.3g}  mu: {mu:>4.1f}  '
          f'ratio: {reduction_ratio:<5.2g}  '
          f'dx: {dx_norm:<8.3g}  reduction: {reduction:<8.3g}'
      )
      print(message, file=output)

    # Append log to trace.
    log = IterLog(candidate=x, objective=y, reduction=reduction, regularizer=mu)
    if verbose >= Verbosity.FULLITER.value:
      log = dataclasses.replace(log, residual=r, jacobian=jac, step=dx)
    trace.append(log)

    # Check for success.
    dx_norm = np.linalg.norm(dx)
    if dx_norm < tol:
      status = Status.DX_TOL
      break

    # Modify regularizer like in (Bazaraa, Sherali, and Shetty)
    if reduction_ratio > 0.75:
      mu = decrease_mu(mu)
    elif reduction_ratio < 0.25:
      mu = increase_mu(mu)

    # Accept proposal.
    x = xnew
    r = rnew

  # Print final diagnostics.
  if verbose > Verbosity.SILENT.value:
    message = f'Terminated after {i} iterations: '
    message += _STATUS_MESSAGE[status]

    message += f' Residual evals: {n_res:d}'
    if n_jac > 0:
      message += f', Jacobian evals: {n_jac:d}'
    print(message, file=output)

    time_total = time.time() - t_start_total
    if time_total > 0:
      qp_percent = 100 * t_qp / time_total
      r_percent = 100 * t_res / time_total
      time_scale = 1 if time_total > 1 else 1000
      time_units = 's' if time_total > 1 else 'ms'
      message = f'total time {time_scale * time_total:<.1f}{time_units}'
      message += f' of which QP {qp_percent:<.1f}%, residual {r_percent:<.1f}%'
      if t_jac > 0:
        jac_percent = 100 * t_jac / time_total
        message += f' Jacobian {jac_percent:<.1f}%'
      print(message, file=output)

  return x, trace
