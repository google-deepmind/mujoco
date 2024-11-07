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
from mujoco.mjx._src import math
from mujoco.mjx._src import smooth
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.dataclasses import PyTreeNode
from mujoco.mjx._src.types import ConeType
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
    active: active (quadratic) constraints            (nefc,)
    fri: friction of regularized cone                 (num(con.dim > 1), 6)
    dm: regularized constraint mass                   (num(con.dim > 1))
    u: friction cone (normal and tangents)            (num(con.dim > 1), 6)
    h: cone hessian                                   (num(con.dim > 1), 6, 6)
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
  active: jax.Array
  fri: jax.Array
  dm: jax.Array
  u: jax.Array
  h: jax.Array

  @classmethod
  def create(cls, m: Model, d: Data, grad: bool = True) -> '_Context':
    jaref = d.efc_J @ d.qacc - d.efc_aref
    # TODO(robotics-team): determine nv at which sparse mul is faster
    ma = support.mul_m(m, d, d.qacc)
    nv_0 = jp.zeros(m.nv)
    fri = 0.0
    if m.opt.cone == ConeType.ELLIPTIC:
      friction = d.contact.friction[d.contact.dim > 1]
      dim = d.contact.dim[d.contact.dim > 1]
      mu = friction[:, 0] / jp.sqrt(m.opt.impratio)
      fri = jp.concatenate((mu[:, None], friction), axis=1)
      for condim in (3, 4, 6):
        fri = fri.at[dim == condim, condim:].set(0)

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
        active=0.0,
        fri=fri,
        dm=0.0,
        u=0.0,
        h=0.0,
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
      d: Data,
      ctx: _Context,
      alpha: jax.Array,
      jv: jax.Array,
      quad: jax.Array,
      quad_gauss: jax.Array,
      uu: jax.Array,
      v0: jax.Array,
      uv: jax.Array,
      vv: jax.Array,
  ) -> '_LSPoint':
    """Creates a linesearch point with first and second derivatives."""
    # roughly corresponds to CGEval in mujoco/src/engine/engine_solver.c

    cost, deriv_0, deriv_1 = 0.0, 0.0, 0.0
    quad_total = quad_gauss
    x = ctx.Jaref + alpha * jv
    active = (x < 0).at[: d.ne + d.nf].set(True)

    dof_fl, ten_fl = m.dof_hasfrictionloss, m.tendon_hasfrictionloss
    if (dof_fl.any() or ten_fl.any()) and not (
        m.opt.disableflags & DisableBit.FRICTIONLOSS
    ):
      f = d.efc_frictionloss
      r = 1.0 / (d.efc_D + (d.efc_D == 0.0) * mujoco.mjMINVAL)
      rf, z = r * f, jp.zeros_like(f)
      linear_neg = (x <= -rf)[:, None]
      linear_pos = (x >= rf)[:, None]
      qf = linear_neg * jp.array([f * (-0.5 * rf - ctx.Jaref), -f * jv, z]).T
      qf += linear_pos * jp.array([f * (-0.5 * rf + ctx.Jaref), f * jv, z]).T
      quad = jp.where((linear_neg | linear_pos) & (f[:, None] > 0), qf, quad)

    if m.opt.cone == ConeType.ELLIPTIC:
      mu, u0 = ctx.fri[:, 0], ctx.u[:, 0]
      n = u0 + alpha * v0
      tsqr = uu + alpha * (2 * uv + alpha * vv)
      t = jp.sqrt(tsqr)  # tangential force

      bottom_zone = ((tsqr <= 0) & (n < 0)) | ((tsqr > 0) & ((mu * n + t) <= 0))
      middle_zone = (tsqr > 0) & (n < (mu * t)) & ((mu * n + t) > 0)

      # quadratic cost for equality, friction, limits, frictionless contacts
      dim1 = d.contact.efc_address[d.contact.dim == 1]
      nefl = d.ne + d.nf + d.nl
      active = active.at[nefl:].set(False).at[dim1].set(active[dim1])
      quad_efld = jax.vmap(jp.multiply)(quad, active)
      quad_total += jp.sum(quad_efld, axis=0)
      # elliptic bottom zone: quadratic cost
      efc_elliptic = d.contact.efc_address[d.contact.dim > 1]
      quad_c = jax.vmap(jp.multiply)(quad[efc_elliptic], bottom_zone)
      quad_total += jp.sum(quad_c, axis=0)
      # elliptic middle zone
      t += (t == 0) * mujoco.mjMINVAL
      tsqr += (tsqr == 0) * mujoco.mjMINVAL
      n1 = v0
      t1 = (uv + alpha * vv) / t
      t2 = vv / t - (uv + alpha * vv) * t1 / tsqr
      dm = ctx.dm * middle_zone
      nmt = n - mu * t
      cost = 0.5 * jp.sum(dm * jp.square(nmt))
      deriv_0 = jp.sum(dm * nmt * (n1 - mu * t1))
      deriv_1 = jp.sum(dm * (jp.square(n1 - mu * t1) - nmt * mu * t2))
    elif m.opt.cone == ConeType.PYRAMIDAL:
      quad = jax.vmap(jp.multiply)(quad, active)  # only active
      quad_total += jp.sum(quad, axis=0)
    else:
      raise NotImplementedError(f'unsupported cone type: {m.opt.cone}')

    alpha_sq = alpha * alpha
    cost += alpha_sq * quad_total[2] + alpha * quad_total[1] + quad_total[0]
    deriv_0 += 2 * alpha * quad_total[2] + quad_total[1]
    deriv_1 += 2 * quad_total[2] + (quad_total[2] == 0) * mujoco.mjMINVAL

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
  """Updates constraint force and resulting cost given last solver iteration.

  Corresponds to CGupdateConstraint in mujoco/src/engine/engine_solver.c

  Args:
    m: model defining constraints
    d: data which contains latest qacc and smooth terms
    ctx: current solver context

  Returns:
    context with new constraint force and costs
  """
  # ne constraints are always active, nf are conditionally active, others are
  # non-negative constraints.
  active = (ctx.Jaref < 0).at[: d.ne + d.nf].set(True)

  floss_force, floss_cost = jp.zeros(d.nefc), 0.0
  dof_fl, ten_fl = m.dof_hasfrictionloss, m.tendon_hasfrictionloss
  if (dof_fl.any() or ten_fl.any()) and not (
      m.opt.disableflags & DisableBit.FRICTIONLOSS
  ):
    f = d.efc_frictionloss
    r = 1.0 / (d.efc_D + (d.efc_D == 0.0) * mujoco.mjMINVAL)
    linear_neg = (ctx.Jaref <= -r * f) * (f > 0)
    linear_pos = (ctx.Jaref >= r * f) * (f > 0)
    active = active & ~linear_neg & ~linear_pos
    floss_force = linear_neg * f + linear_pos * -f
    floss_cost = linear_neg * (-0.5 * r * f * f - f * ctx.Jaref)
    floss_cost += linear_pos * (-0.5 * r * f * f + f * ctx.Jaref)
    floss_cost = floss_cost.sum()

  if m.opt.cone == ConeType.PYRAMIDAL:
    efc_force = d.efc_D * -ctx.Jaref * active + floss_force
    cost = 0.5 * jp.sum(d.efc_D * ctx.Jaref * ctx.Jaref * active)
    dm, u, h = 0.0, 0.0, 0.0
  elif m.opt.cone == ConeType.ELLIPTIC:
    friction = d.contact.friction[d.contact.dim > 1]
    efc_address = d.contact.efc_address[d.contact.dim > 1]
    dim = d.contact.dim[d.contact.dim > 1]
    slice_fn = jax.vmap(lambda x: jax.lax.dynamic_slice(ctx.Jaref, (x,), (6,)))
    u = slice_fn(efc_address) * ctx.fri
    mu, n, t = ctx.fri[:, 0], u[:, 0], jax.vmap(math.norm)(u[:, 1:])

    # bottom zone: quadratic
    bottom_zone = ((t <= 0) & (n < 0)) | ((t > 0) & ((mu * n + t) <= 0))
    adr_i, adr_j = [], []
    for i, (condim, addr) in enumerate(zip(dim, efc_address)):
      adr_i.extend(range(addr, addr + condim))
      adr_j.extend([i] * condim)
    active = active.at[jp.array(adr_i)].set(bottom_zone[jp.array(adr_j)])
    efc_force = d.efc_D * -ctx.Jaref * active + floss_force
    cost = 0.5 * jp.sum(d.efc_D * ctx.Jaref * ctx.Jaref * active)

    # middle zone: cone
    middle_zone = (t > 0) & (n < (mu * t)) & ((mu * n + t) > 0)
    dm = d.efc_D[efc_address] / jp.maximum(
        mu * mu * (1 + mu * mu), mujoco.mjMINVAL
    )
    nmt = n - mu * t
    cost += 0.5 * jp.sum(dm * nmt * nmt * middle_zone)
    # tangent and friction for middle zone:
    force = -dm * nmt * mu * middle_zone
    force_fri = -force / (t + ~middle_zone * mujoco.mjMINVAL)
    force_fri = force_fri[:, None] * u[:, 1:] * friction
    efc_force = efc_force.at[efc_address].add(force)
    efc_adr, adr_i, adr_j = [], [], []
    for i, (condim, addr) in enumerate(zip(dim, efc_address)):
      efc_adr.extend(range(addr + 1, addr + condim))
      adr_i.extend([i] * (condim - 1))
      adr_j.extend(range(condim - 1))
    efc_adr, adr_i, adr_j = jp.array(efc_adr), jp.array(adr_i), jp.array(adr_j)
    efc_force = efc_force.at[efc_adr].add(force_fri[(adr_i, adr_j)])

    # cone hessian
    h = 0.0
    if m.opt.solver == SolverType.NEWTON:
      t = jp.maximum(t, mujoco.mjMINVAL)
      # h = mu*N/T^3 * U*U'
      ttt = jp.maximum(t * t * t, mujoco.mjMINVAL)
      h = jax.vmap(lambda x, y: x * jp.outer(y, y.T))(mu * n / ttt, u)
      # add to diagonal: (mu^2 - mu*N/T) * I
      h += jax.vmap(lambda x: x * jp.eye(6, 6))(mu * mu - mu * n / t)
      # set first row: (1, -mu/T * U)
      h_0 = jax.vmap(lambda mu, t, u: jp.append(1, -mu / t * u[1:]))(mu, t, u)
      h = h.at[:, 0].set(h_0).at[:, :, 0].set(h_0)
      # pre and post multiply by diag(mu, friction), scale by Dm
      h *= jax.vmap(lambda d, f: d * jp.outer(f, f.T))(dm, ctx.fri)
      # only cone constraints
      h = jax.vmap(jp.multiply)(h, middle_zone)
  else:
    raise NotImplementedError(f'unsupported cone type: {m.opt.cone}')

  qfrc_constraint = d.efc_J.T @ efc_force
  gauss = 0.5 * jp.dot(ctx.Ma - d.qfrc_smooth, ctx.qacc - d.qacc_smooth)
  ctx = ctx.replace(
      qfrc_constraint=qfrc_constraint,
      gauss=gauss,
      cost=cost + gauss + floss_cost,
      prev_cost=ctx.cost,
      efc_force=efc_force,
      active=active,
      dm=dm,
      u=u,
      h=h,
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
    if m.opt.cone == ConeType.ELLIPTIC:
      cm = jp.diag(d.efc_D * ctx.active)
      efc_address = d.contact.efc_address[d.contact.dim > 1]
      dim = d.contact.dim[d.contact.dim > 1]
      # set efc of cone H along diagonal
      for i, (condim, addr) in enumerate(zip(dim, efc_address)):
        h_cone = ctx.h[i, :condim, :condim]
        cm = cm.at[addr:addr+condim, addr:addr+condim].add(h_cone)
      h = d.efc_J.T @ cm @ d.efc_J
    else:
      h = (d.efc_J.T * d.efc_D * ctx.active) @ d.efc_J
    h = support.full_m(m, d) + h
    h_ = jax.scipy.linalg.cho_factor(h)
    mgrad = jax.scipy.linalg.cho_solve(h_, grad)
  else:
    raise NotImplementedError(f'unsupported solver type: {m.opt.solver}')

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
  uu, v0, uv, vv = 0.0, 0.0, 0.0, 0.0
  if m.opt.cone == ConeType.ELLIPTIC:
    mask = d.contact.dim > 1
    # complete vector quadratic (for bottom zone)
    efc_con, efc_fri = [], []
    for condim, addr in zip(d.contact.dim[mask], d.contact.efc_address[mask]):
      efc_con.extend([addr] * (condim - 1))
      efc_fri.extend(range(addr + 1, addr + condim))
    quad = quad.at[jp.array(efc_con)].add(quad[jp.array(efc_fri)])

    # rescale to make primal cone circular
    jv_fn = jax.vmap(lambda x: jax.lax.dynamic_slice(jv, (x,), (6,)))
    efc_elliptic = d.contact.efc_address[mask]
    v = jv_fn(efc_elliptic) * ctx.fri
    uu = jp.sum(ctx.u[:, 1:] * ctx.u[:, 1:], axis=1)
    v0 = v[:, 0]
    uv = jp.sum(ctx.u[:, 1:] * v[:, 1:], axis=1)
    vv = jp.sum(v[:, 1:] * v[:, 1:], axis=1)

  point_fn = lambda a: _LSPoint.create(
      m, d, ctx, a, jv, quad, quad_gauss, uu, v0, uv, vv
  )

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

    # swap lo/hi if the derivative points to a narrower bracket width
    in_bracket = lambda x, y: ((x < y) & (y < 0) | (x > y) & (y > 0))
    swap_lo_next = in_bracket(lo.deriv_0, lo_next.deriv_0)
    lo = jax.tree_util.tree_map(
        lambda x, y: jp.where(swap_lo_next, y, x), lo, lo_next
    )
    swap_lo_mid = in_bracket(lo.deriv_0, mid.deriv_0)
    lo = jax.tree_util.tree_map(
        lambda x, y: jp.where(swap_lo_mid, y, x), lo, mid
    )
    swap_lo_hi_next = in_bracket(lo.deriv_0, hi_next.deriv_0)
    lo = jax.tree_util.tree_map(
        lambda x, y: jp.where(swap_lo_hi_next, y, x), lo, hi_next
    )
    swap_hi_next = in_bracket(hi.deriv_0, hi_next.deriv_0)
    hi = jax.tree_util.tree_map(
        lambda x, y: jp.where(swap_hi_next, y, x), hi, hi_next
    )
    swap_hi_mid = in_bracket(hi.deriv_0, mid.deriv_0)
    hi = jax.tree_util.tree_map(
        lambda x, y: jp.where(swap_hi_mid, y, x), hi, mid
    )
    swap_hi_lo_next = in_bracket(hi.deriv_0, lo_next.deriv_0)
    hi = jax.tree_util.tree_map(
        lambda x, y: jp.where(swap_hi_lo_next, y, x), hi, lo_next
    )
    swap = swap_lo_next | swap_lo_mid | swap_lo_hi_next
    swap = swap | swap_hi_next | swap_hi_mid | swap_hi_lo_next
    ctx = ctx.replace(lo=lo, hi=hi, swap=swap, ls_iter=ctx.ls_iter + 1)

    return ctx

  # initialize interval
  p0 = point_fn(jp.array(0.0))
  lo = point_fn(p0.alpha - p0.deriv_0 / p0.deriv_1)
  lesser_fn = lambda x, y: jp.where(lo.deriv_0 < p0.deriv_0, x, y)
  hi = jax.tree_util.tree_map(lesser_fn, p0, lo)
  lo = jax.tree_util.tree_map(lesser_fn, lo, p0)
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

    if m.opt.solver == SolverType.NEWTON:
      search = -ctx.Mgrad
    else:
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
