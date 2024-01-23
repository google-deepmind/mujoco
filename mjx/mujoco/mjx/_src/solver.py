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
"""Constraint solvers."""

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import constraint
from mujoco.mjx._src import math
from mujoco.mjx._src import smooth
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.dataclasses import PyTreeNode
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import SolverType
# pylint: enable=g-importing-member


class _Context(PyTreeNode):
  """Data updated during each solver iteration.

  Attributes:
    qacc: acceleration (from Data)                    (nv,)
    qfrc_constraint: constraint force (from Data)     (nv,)
    Jaref: Jac*qacc - aref                            (nefc,)
    efc_force: constraint force in constraint space   (nefc,)
    Ma: M*qacc                                        (nv,)
    grad: gradient of master cost                     (nv,)
    Mgrad: M / grad                                   (nv,)
    search: linesearch vector                         (nv,)
    gauss: gauss Cost
    cost: constraint + Gauss cost
    prev_cost: cost from previous iter
    solver_niter: number of solver iterations
  """

  qacc: jax.Array
  qfrc_constraint: jax.Array
  Jaref: jax.Array  # pylint: disable=invalid-name
  efc_force: jax.Array
  Ma: jax.Array  # pylint: disable=invalid-name
  grad: jax.Array
  Mgrad: jax.Array  # pylint: disable=invalid-name
  search: jax.Array
  gauss: jax.Array
  cost: jax.Array
  prev_cost: jax.Array
  solver_niter: jax.Array

  @classmethod
  def create(cls, m: Model, d: Data, grad: bool = True) -> '_Context':
    jaref = d.efc_J @ d.qacc - d.efc_aref
    # TODO(robotics-team): determine nv at which sparse mul is faster
    ma = support.mul_m(m, d, d.qacc)
    nv_0 = jp.zeros(m.nv)
    ctx = _Context(
        qacc=d.qacc,
        qfrc_constraint=d.qfrc_constraint,
        Jaref=jaref,
        efc_force=d.efc_force,
        Ma=ma,
        grad=nv_0,
        Mgrad=nv_0,
        search=nv_0,
        gauss=0.0,
        cost=jp.inf,
        prev_cost=0.0,
        solver_niter=0,
    )
    ctx = _update_constraint(m, d, ctx)
    if grad:
      ctx = _update_gradient(m, d, ctx)
      ctx = ctx.replace(search=-ctx.Mgrad)  # start with preconditioned gradient

    return ctx


class _LSPoint(PyTreeNode):
  """Line search evaluation point.

  Attributes:
    alpha: step size that reduces f(x + alpha * p) given search direction p
    cost: line search cost
    deriv_0: first derivative of quadratic
    deriv_1: second derivative of quadratic
  """

  alpha: jax.Array
  cost: jax.Array
  deriv_0: jax.Array
  deriv_1: jax.Array

  @classmethod
  def create(
      cls,
      m: Model,
      ctx: _Context,
      alpha: jax.Array,
      jv: jax.Array,
      quad: jax.Array,
      quad_gauss: jax.Array,
  ) -> '_LSPoint':
    """Creates a linesearch point with first and second derivatives."""
    # roughly corresponds to CGEval in mujoco/src/engine/engine_solver.c

    # TODO(robotics-team): change this to support friction constraints
    ne, nf, *_ = constraint.count_constraints(m)
    active = ((ctx.Jaref + alpha * jv) < 0).at[:ne + nf].set(True)
    quad = jax.vmap(jp.multiply)(quad, active)  # only active
    quad_total = quad_gauss + jp.sum(quad, axis=0)

    cost = alpha * alpha * quad_total[2] + alpha * quad_total[1] + quad_total[0]
    deriv_0 = 2 * alpha * quad_total[2] + quad_total[1]
    deriv_1 = 2 * quad_total[2] + (quad_total[2] == 0) * mujoco.mjMINVAL
    return _LSPoint(alpha=alpha, cost=cost, deriv_0=deriv_0, deriv_1=deriv_1)


class _LSContext(PyTreeNode):
  """Data updated during each line search iteration.

  Attributes:
    lo: low point bounding the line search interval
    hi: high point bounding the line search interval
    swap: True if low or hi was swapped in the line search iteration
    ls_iter: number of linesearch iterations
  """

  lo: _LSPoint
  hi: _LSPoint
  swap: jax.Array
  ls_iter: jax.Array


def _while_loop_scan(cond_fun, body_fun, init_val, max_iter):
  """Scan-based implementation (jit ok, reverse-mode autodiff ok)."""
  def _iter(val):
    next_val = body_fun(val)
    next_cond = cond_fun(next_val)
    return next_val, next_cond

  def _fun(tup, it):
    val, cond = tup
    # When cond is met, we start doing no-ops.
    return jax.lax.cond(cond, _iter, lambda x: (x, False), val), it

  init = (init_val, cond_fun(init_val))
  return jax.lax.scan(_fun, init, None, length=max_iter)[0][0]


def _update_constraint(m: Model, d: Data, ctx: _Context) -> _Context:
  """Updates constraint force and resulting cost given latst solver iteration.

  Corresponds to CGupdateConstraint in mujoco/src/engine/engine_solver.c

  Args:
    m: model defining constraints
    d: data which contains latest qacc and smooth terms
    ctx: current solver context

  Returns:
    context with new constraint force and costs
  """
  # TODO(robotics-team): add friction constraints

  # only count active constraints
  ne, nf, *_ = constraint.count_constraints(m)
  active = (ctx.Jaref < 0).at[:ne + nf].set(True)

  efc_force = d.efc_D * -ctx.Jaref * active
  qfrc_constraint = d.efc_J.T @ efc_force
  gauss = 0.5 * jp.dot(ctx.Ma - d.qfrc_smooth, ctx.qacc - d.qacc_smooth)
  cost = 0.5 * jp.sum(d.efc_D * ctx.Jaref * ctx.Jaref * active) + gauss

  ctx = ctx.replace(
      qfrc_constraint=qfrc_constraint,
      gauss=gauss,
      cost=cost,
      prev_cost=ctx.cost,
      efc_force=efc_force,
  )

  return ctx


def _update_gradient(m: Model, d: Data, ctx: _Context) -> _Context:
  """Updates grad and M / grad given latest solver iteration.

  Corresponds to CGupdateGradient in mujoco/src/engine/engine_solver.c

  Args:
    m: model defining constraints
    d: data which contains latest smooth terms
    ctx: current solver context

  Returns:
    context with new grad and M / grad
  Raises:
    NotImplementedError: for unsupported solver type
  """

  grad = ctx.Ma - d.qfrc_smooth - ctx.qfrc_constraint

  if m.opt.solver == SolverType.CG:
    mgrad = smooth.solve_m(m, d, grad)
  elif m.opt.solver == SolverType.NEWTON:
    ne, nf, *_ = constraint.count_constraints(m)
    active = (ctx.Jaref < 0).at[:ne + nf].set(True)
    h = d.qM + support.make_m(m, d.efc_J.T * d.efc_D * active, d.efc_J.T)
    dh = d.replace(qM=h)
    dh = smooth.factor_m(m, dh)
    mgrad = smooth.solve_m(m, dh, grad)
  else:
    raise NotImplementedError(f"unsupported solver type: {m.opt.solver}")

  ctx = ctx.replace(grad=grad, Mgrad=mgrad)

  return ctx


def _rescale(m: Model, value: jax.Array) -> jax.Array:
  return value / (m.stat.meaninertia * max(1, m.nv))


def _linesearch(m: Model, d: Data, ctx: _Context) -> _Context:
  """Performs a zoom linesearch to find optimal search step size.

  Args:
    m: model defining search options and other needed terms
    d: data with inertia matrix and other needed terms
    ctx: current solver context

  Returns:
    updated context with new qacc, Ma, Jaref
  """
  smag = math.norm(ctx.search) * m.stat.meaninertia * max(1, m.nv)
  gtol = m.opt.tolerance * m.opt.ls_tolerance * smag

  # compute Mv, Jv
  mv = support.mul_m(m, d, ctx.search)
  jv = d.efc_J @ ctx.search

  # prepare quadratics
  quad_gauss = jp.stack((
      ctx.gauss,
      jp.dot(ctx.search, ctx.Ma) - jp.dot(ctx.search, d.qfrc_smooth),
      0.5 * jp.dot(ctx.search, mv),
  ))
  quad = jp.stack((0.5 * ctx.Jaref * ctx.Jaref, jv * ctx.Jaref, 0.5 * jv * jv))
  quad = (quad * d.efc_D).T

  point_fn = lambda a: _LSPoint.create(m, ctx, a, jv, quad, quad_gauss)

  def cond(ctx: _LSContext) -> jax.Array:
    done = ctx.ls_iter >= m.opt.ls_iterations
    done |= ~ctx.swap  # if we did not adjust the interval
    done |= (ctx.lo.deriv_0 < 0) & (ctx.lo.deriv_0 > -gtol)
    done |= (ctx.hi.deriv_0 > 0) & (ctx.hi.deriv_0 < gtol)

    return ~done

  def body(ctx: _LSContext) -> _LSContext:
    # always compute new bracket boundaries and a midpoint
    lo, hi = ctx.lo, ctx.hi
    lo_next = point_fn(lo.alpha - lo.deriv_0 / lo.deriv_1)
    hi_next = point_fn(hi.alpha - hi.deriv_0 / hi.deriv_1)
    mid = point_fn(0.5 * (lo.alpha + hi.alpha))

    # we swap lo/hi if:
    # 1) they are not correctly at a bracket boundary (e.g. lo.deriv_0 > 0), OR
    # 2) if moving to next or mid narrows the bracket
    swap_lo_next = (lo.deriv_0 > 0) | (lo.deriv_0 < lo_next.deriv_0)
    lo = jax.tree_map(lambda x, y: jp.where(swap_lo_next, y, x), lo, lo_next)
    swap_lo_mid = (mid.deriv_0 < 0) & (lo.deriv_0 < mid.deriv_0)
    lo = jax.tree_map(lambda x, y: jp.where(swap_lo_mid, y, x), lo, mid)

    swap_hi_next = (hi.deriv_0 < 0) | (hi.deriv_0 > hi_next.deriv_0)
    hi = jax.tree_map(lambda x, y: jp.where(swap_hi_next, y, x), hi, hi_next)
    swap_hi_mid = (mid.deriv_0 > 0) & (hi.deriv_0 > mid.deriv_0)
    hi = jax.tree_map(lambda x, y: jp.where(swap_hi_mid, y, x), hi, mid)

    swap = swap_lo_next | swap_lo_mid | swap_hi_next | swap_hi_mid

    ctx = ctx.replace(lo=lo, hi=hi, swap=swap, ls_iter=ctx.ls_iter + 1)

    return ctx

  # initialize interval
  p0 = point_fn(jp.array(0.0))
  lo = point_fn(p0.alpha - p0.deriv_0 / p0.deriv_1)
  lesser_fn = lambda x, y: jp.where(lo.deriv_0 < p0.deriv_0, x, y)
  hi = jax.tree_map(lesser_fn, p0, lo)
  lo = jax.tree_map(lesser_fn, lo, p0)
  ls_ctx = _LSContext(lo=lo, hi=hi, swap=jp.array(True), ls_iter=0)
  ls_ctx = _while_loop_scan(cond, body, ls_ctx, m.opt.ls_iterations)

  # move to new solution if improved
  lo, hi = ls_ctx.lo, ls_ctx.hi
  improved = (lo.cost < p0.cost) | (hi.cost < p0.cost)
  alpha = jp.where(lo.cost < hi.cost, lo.alpha, hi.alpha)
  qacc = ctx.qacc + improved * ctx.search * alpha
  ma = ctx.Ma + improved * mv * alpha
  jaref = ctx.Jaref + improved * jv * alpha

  ctx = ctx.replace(qacc=qacc, Ma=ma, Jaref=jaref)

  return ctx


def solve(m: Model, d: Data) -> Data:
  """Finds forces that satisfy constraints using conjugate gradient descent."""

  def cond(ctx: _Context) -> jax.Array:
    improvement = _rescale(m, ctx.prev_cost - ctx.cost)
    gradient = _rescale(m, math.norm(ctx.grad))

    done = ctx.solver_niter >= m.opt.iterations
    done |= improvement < m.opt.tolerance
    done |= gradient < m.opt.tolerance

    return ~done

  def body(ctx: _Context) -> _Context:
    ctx = _linesearch(m, d, ctx)
    prev_grad, prev_Mgrad = ctx.grad, ctx.Mgrad  # pylint: disable=invalid-name
    ctx = _update_constraint(m, d, ctx)
    ctx = _update_gradient(m, d, ctx)

    # polak-ribiere:
    beta = jp.dot(ctx.grad, ctx.Mgrad - prev_Mgrad)
    beta = beta / jp.maximum(mujoco.mjMINVAL, jp.dot(prev_grad, prev_Mgrad))
    beta = jp.maximum(0, beta)
    search = -ctx.Mgrad + beta * ctx.search
    ctx = ctx.replace(search=search, solver_niter=ctx.solver_niter + 1)

    return ctx

  # warmstart:
  qacc = d.qacc_smooth
  if not m.opt.disableflags & DisableBit.WARMSTART:
    warm = _Context.create(m, d.replace(qacc=d.qacc_warmstart), grad=False)
    smth = _Context.create(m, d.replace(qacc=d.qacc_smooth), grad=False)
    qacc = jp.where(warm.cost < smth.cost, d.qacc_warmstart, d.qacc_smooth)
  d = d.replace(qacc=qacc)

  ctx = _Context.create(m, d)
  if m.opt.iterations == 1:
    ctx = body(ctx)
  else:
    ctx = jax.lax.while_loop(cond, body, ctx)

  d = d.replace(
      qacc_warmstart=ctx.qacc,
      qacc=ctx.qacc,
      qfrc_constraint=ctx.qfrc_constraint,
      efc_force=ctx.efc_force,
  )

  return d
