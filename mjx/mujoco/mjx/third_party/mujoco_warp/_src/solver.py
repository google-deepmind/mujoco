# Copyright 2025 The Newton Developers
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

from math import ceil

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import island
from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_factorize_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.types import InverseContext
from mujoco.mjx.third_party.mujoco_warp._src.types import IslandSolverContext
from mujoco.mjx.third_party.mujoco_warp._src.types import SolverContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})

_BLOCK_CHOLESKY_DIM = 32


def create_inverse_context(m: types.Model, d: types.Data) -> InverseContext:
  """Create an InverseContext with allocated workspace arrays.

  Args:
    m: Model.
    d: Data.

  Returns:
    InverseContext with allocated arrays.
  """
  nworld = d.nworld
  njmax = d.njmax

  return InverseContext(
    Jaref=wp.empty((nworld, njmax), dtype=float),
    search_dot=wp.empty((nworld,), dtype=float),
    done=wp.empty((nworld,), dtype=bool),
    changed_efc_ids=wp.empty((nworld, 0), dtype=int),
    changed_efc_count=wp.empty((0,), dtype=int),
  )


def _create_island_solver_context(m: types.Model, d: types.Data) -> IslandSolverContext:
  """Create an IslandSolverContext with allocated workspace arrays.

  Args:
    m: Model.
    d: Data.

  Returns:
    IslandSolverContext with allocated arrays.
  """
  nworld = d.nworld
  nv = m.nv
  nv_pad = m.nv_pad
  njmax = d.njmax
  ntree = m.ntree

  alloc_h = m.opt.solver == types.SolverType.NEWTON
  alloc_island_cg = m.opt.solver == types.SolverType.CG

  return IslandSolverContext(
    Jaref=wp.empty((nworld, njmax), dtype=float),
    jv=wp.empty((nworld, njmax), dtype=float),
    search=wp.empty((nworld, nv), dtype=float),
    mv=wp.empty((nworld, nv), dtype=float),
    grad=wp.zeros((nworld, nv_pad), dtype=float),
    Mgrad=wp.zeros((nworld, nv_pad), dtype=float),
    prev_grad=wp.empty((nworld, nv), dtype=float) if alloc_island_cg else wp.empty((nworld, 0), dtype=float),
    prev_Mgrad=wp.empty((nworld, nv), dtype=float) if alloc_island_cg else wp.empty((nworld, 0), dtype=float),
    h=wp.zeros((nworld, nv_pad, nv_pad), dtype=float) if alloc_h else wp.empty((nworld, 0, 0), dtype=float),
    # Per-island solver scalars
    cost=wp.empty((nworld, ntree), dtype=float),
    prev_cost=wp.empty((nworld, ntree), dtype=float),
    gauss=wp.empty((nworld, ntree), dtype=float),
    search_dot=wp.empty((nworld, ntree), dtype=float),
    grad_dot=wp.empty((nworld, ntree), dtype=float),
    done=wp.empty((nworld, ntree), dtype=bool),
    solver_niter=wp.empty((nworld, ntree), dtype=int),
    beta=wp.empty((nworld, ntree), dtype=float) if alloc_island_cg else wp.empty((nworld, 0), dtype=float),
    beta_den=wp.empty((nworld, ntree), dtype=float) if alloc_island_cg else wp.empty((nworld, 0), dtype=float),
    alpha=wp.empty((nworld, ntree), dtype=float),
    Ma=wp.empty((nworld, nv), dtype=float),
  )


def _create_solver_context(m: types.Model, d: types.Data) -> SolverContext:
  """Create a SolverContext with allocated workspace arrays.

  Args:
    m: Model.
    d: Data.

  Returns:
    SolverContext with allocated arrays.
  """
  nworld = d.nworld
  nv = m.nv
  nv_pad = m.nv_pad
  njmax = d.njmax

  alloc_h = m.opt.solver == types.SolverType.NEWTON
  alloc_hfactor = alloc_h and nv > _BLOCK_CHOLESKY_DIM

  return SolverContext(
    Jaref=wp.empty((nworld, njmax), dtype=float),
    search_dot=wp.empty((nworld,), dtype=float),
    done=wp.empty((nworld,), dtype=bool),
    grad=wp.zeros((nworld, nv_pad), dtype=float),
    grad_dot=wp.empty((nworld,), dtype=float),
    Mgrad=wp.empty((nworld, nv_pad), dtype=float),
    search=wp.empty((nworld, nv), dtype=float),
    mv=wp.empty((nworld, nv), dtype=float),
    jv=wp.empty((nworld, njmax), dtype=float),
    quad=wp.empty((nworld, njmax), dtype=wp.vec3),
    alpha=wp.empty((nworld,), dtype=float),
    improvement=wp.empty((nworld,), dtype=float),
    prev_grad=wp.empty((nworld, nv), dtype=float),
    prev_Mgrad=wp.empty((nworld, nv), dtype=float),
    beta=wp.empty((nworld,), dtype=float),
    beta_den=wp.empty((nworld,), dtype=float),
    h=wp.empty((nworld, nv_pad, nv_pad), dtype=float) if alloc_h else wp.empty((nworld, 0, 0), dtype=float),
    hfactor=wp.empty((nworld, nv_pad, nv_pad), dtype=float) if alloc_hfactor else wp.empty((nworld, 0, 0), dtype=float),
    changed_efc_ids=wp.empty((nworld, njmax), dtype=int) if alloc_h else wp.empty((nworld, 0), dtype=int),
    changed_efc_count=wp.empty((nworld,), dtype=int) if alloc_h else wp.empty((0,), dtype=int),
  )


@wp.func
def _rescale(nv: int, meaninertia: float, value: float) -> float:
  return value / (meaninertia * float(nv))


@wp.func
def _in_bracket(x: wp.vec3, y: wp.vec3) -> bool:
  return (x[1] < y[1] and y[1] < 0.0) or (x[1] > y[1] and y[1] > 0.0)


@wp.func
def _eval_pt_direct_alpha_zero(jaref: float, jv: float, d: float) -> wp.vec3:
  """Eval quadratic constraint at alpha=0."""
  jvD = jv * d
  return wp.vec3(0.5 * d * jaref * jaref, jvD * jaref, jv * jvD)


@wp.func
def _eval_pt_direct(jaref: float, jv: float, d: float, alpha: float) -> wp.vec3:
  """Eval quadratic constraint."""
  x = jaref + alpha * jv
  jvD = jv * d
  return wp.vec3(0.5 * d * x * x, jvD * x, jv * jvD)


@wp.func
def _eval_pt_direct_cost_alpha_zero(jaref: float, d: float) -> float:
  return 0.5 * d * jaref * jaref


@wp.func
def _eval_pt_direct_shifted(jaref: float, jv: float, d: float, alpha: float, offset: float) -> wp.vec3:
  """Eval quadratic constraint shifted by alpha=0, plus a constant cost offset."""
  jvD = jv * d
  hessian = jv * jvD
  alpha_h = alpha * hessian
  return wp.vec3(alpha * (jvD * jaref + 0.5 * alpha_h) + offset, jvD * jaref + alpha_h, hessian)


@wp.func
def _eval_pt_direct_3alphas(
  jaref: float, jv: float, d: float, lo_alpha: float, hi_alpha: float, mid_alpha: float
) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Eval quadratic constraint for 3 alphas."""
  x_lo = jaref + lo_alpha * jv
  x_hi = jaref + hi_alpha * jv
  x_mid = jaref + mid_alpha * jv
  jvD = jv * d
  hessian = jv * jvD
  half_d = 0.5 * d
  return (
    wp.vec3(half_d * x_lo * x_lo, jvD * x_lo, hessian),
    wp.vec3(half_d * x_hi * x_hi, jvD * x_hi, hessian),
    wp.vec3(half_d * x_mid * x_mid, jvD * x_mid, hessian),
  )


@wp.func
def _eval_pt_direct_shifted_3alphas(
  jaref: float, jv: float, d: float, lo_alpha: float, hi_alpha: float, mid_alpha: float, offset: float
) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Eval shifted quadratic constraint for 3 alphas, plus a constant cost offset."""
  jvD = jv * d
  grad0 = jvD * jaref
  hessian = jv * jvD
  lo_ah = lo_alpha * hessian
  hi_ah = hi_alpha * hessian
  mid_ah = mid_alpha * hessian
  return (
    wp.vec3(lo_alpha * (grad0 + 0.5 * lo_ah) + offset, grad0 + lo_ah, hessian),
    wp.vec3(hi_alpha * (grad0 + 0.5 * hi_ah) + offset, grad0 + hi_ah, hessian),
    wp.vec3(mid_alpha * (grad0 + 0.5 * mid_ah) + offset, grad0 + mid_ah, hessian),
  )


@wp.func
def _eval_cost(quad: wp.vec3, alpha: float) -> float:
  return alpha * alpha * quad[2] + alpha * quad[1] + quad[0]


@wp.func
def _eval_pt(quad: wp.vec3, alpha: float) -> wp.vec3:
  """Eval quad polynomial at alpha, return (cost, grad, hessian)."""
  aq2 = alpha * quad[2]
  return wp.vec3(
    alpha * aq2 + alpha * quad[1] + quad[0],
    2.0 * aq2 + quad[1],
    2.0 * quad[2],
  )


@wp.func
def _eval_pt_3alphas(quad: wp.vec3, lo_alpha: float, hi_alpha: float, mid_alpha: float) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Eval quad polynomial for 3 alphas."""
  q0, q1, q2 = quad[0], quad[1], quad[2]
  hessian = 2.0 * q2
  lo_aq2 = lo_alpha * q2
  hi_aq2 = hi_alpha * q2
  mid_aq2 = mid_alpha * q2
  return (
    wp.vec3(lo_alpha * lo_aq2 + lo_alpha * q1 + q0, 2.0 * lo_aq2 + q1, hessian),
    wp.vec3(hi_alpha * hi_aq2 + hi_alpha * q1 + q0, 2.0 * hi_aq2 + q1, hessian),
    wp.vec3(mid_alpha * mid_aq2 + mid_alpha * q1 + q0, 2.0 * mid_aq2 + q1, hessian),
  )


@wp.func
def _shift_cost(pt: wp.vec3, cost0: float) -> wp.vec3:
  return wp.vec3(pt[0] - cost0, pt[1], pt[2])


@wp.func
def _eval_frictionloss_pt(x: float, f: float, rf: float, jv: float, d: float) -> wp.vec3:
  """Eval frictionloss and return (cost, grad, hessian). x = Jaref + alpha * jv."""
  if (-rf < x) and (x < rf):
    jvD = jv * d
    return wp.vec3(0.5 * d * x * x, jvD * x, jv * jvD)
  elif x <= -rf:
    return wp.vec3(f * (-0.5 * rf - x), -f * jv, 0.0)
  else:
    return wp.vec3(f * (-0.5 * rf + x), f * jv, 0.0)


@wp.func
def _eval_frictionloss_cost(x: float, f: float, rf: float, d: float) -> float:
  if (-rf < x) and (x < rf):
    return 0.5 * d * x * x
  elif x <= -rf:
    return f * (-0.5 * rf - x)
  return f * (-0.5 * rf + x)


@wp.func
def _eval_frictionloss_pt_one(x: float, f: float, rf: float, half_d: float, jvD: float, hessian: float, f_jv: float) -> wp.vec3:
  """Eval frictionloss with precomputed shared values."""
  if (-rf < x) and (x < rf):
    return wp.vec3(half_d * x * x, jvD * x, hessian)
  elif x <= -rf:
    return wp.vec3(f * (-0.5 * rf - x), -f_jv, 0.0)
  else:
    return wp.vec3(f * (-0.5 * rf + x), f_jv, 0.0)


@wp.func
def _eval_frictionloss_pt_3alphas(
  x_lo: float, x_hi: float, x_mid: float, f: float, rf: float, jv: float, d: float
) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Eval frictionloss for 3 x values with shared precomputation."""
  jvD = jv * d
  half_d = 0.5 * d
  hessian = jv * jvD
  f_jv = f * jv
  return (
    _eval_frictionloss_pt_one(x_lo, f, rf, half_d, jvD, hessian, f_jv),
    _eval_frictionloss_pt_one(x_hi, f, rf, half_d, jvD, hessian, f_jv),
    _eval_frictionloss_pt_one(x_mid, f, rf, half_d, jvD, hessian, f_jv),
  )


@wp.func
def _eval_elliptic(
  # In:
  mu: float,
  quad: wp.vec3,
  quad1: wp.vec3,
  quad2: wp.vec3,
  alpha: float,
) -> wp.vec3:
  u0 = quad1[0]
  v0 = quad1[1]
  uu = quad1[2]
  uv = quad2[0]
  vv = quad2[1]
  dm = quad2[2]

  # compute N, Tsqr
  N = u0 + alpha * v0
  Tsqr = uu + alpha * (2.0 * uv + alpha * vv)

  # no tangential force: top or bottom zone
  if Tsqr <= 0.0:
    # bottom zone: quadratic cost
    if N < 0.0:
      return _eval_pt(quad, alpha)

    # top zone: nothing to do
  # otherwise regular processing
  else:
    # tangential force
    T = wp.sqrt(Tsqr)

    # N >= mu * T : top zone
    if N >= mu * T:
      # nothing to do
      pass
    # mu * N + T <= 0 : bottom zone
    elif mu * N + T <= 0.0:
      return _eval_pt(quad, alpha)

    # otherwise middle zone
    else:
      # derivatives
      N1 = v0
      T1 = (uv + alpha * vv) / T
      T2 = vv / T - (uv + alpha * vv) * T1 / (T * T)

      # add to cost
      cost = wp.vec3(
        0.5 * dm * (N - mu * T) * (N - mu * T),
        dm * (N - mu * T) * (N1 - mu * T1),
        dm * ((N1 - mu * T1) * (N1 - mu * T1) + (N - mu * T) * (-mu * T2)),
      )

      return cost

  return wp.vec3(0.0, 0.0, 0.0)


@wp.func
def _eval_elliptic_cost(
  # In:
  mu: float,
  quad: wp.vec3,
  quad1: wp.vec3,
  quad2: wp.vec3,
  alpha: float,
) -> float:
  u0 = quad1[0]
  v0 = quad1[1]
  uu = quad1[2]
  uv = quad2[0]
  vv = quad2[1]
  dm = quad2[2]

  N = u0 + alpha * v0
  Tsqr = uu + alpha * (2.0 * uv + alpha * vv)

  if Tsqr <= 0.0:
    if N < 0.0:
      return _eval_cost(quad, alpha)
  else:
    T = wp.sqrt(Tsqr)
    if N >= mu * T:
      pass
    elif mu * N + T <= 0.0:
      return _eval_cost(quad, alpha)
    else:
      return 0.5 * dm * (N - mu * T) * (N - mu * T)

  return 0.0


@wp.func
def _eval_constraint(
  # In:
  is_equality: bool,
  is_friction: bool,
  is_elliptic: bool,
  jaref: float,
  D: float,
  frictionloss: float,
  efcid: int,
  efcid0: int,
  jaref0: float,
  D0: float,
  mu: float,
  ufrictionj: float,
  TT: float,
) -> wp.vec3:
  if is_equality:
    force = -D * jaref
    cost = 0.5 * D * jaref * jaref
    return wp.vec3(force, float(types.ConstraintState.QUADRATIC.value), cost)

  if is_friction:
    rf = math.safe_div(frictionloss, D)
    if jaref <= -rf:
      return wp.vec3(frictionloss, float(types.ConstraintState.LINEARNEG.value), -frictionloss * (0.5 * rf + jaref))
    elif jaref >= rf:
      return wp.vec3(-frictionloss, float(types.ConstraintState.LINEARPOS.value), -frictionloss * (0.5 * rf - jaref))
    else:
      return wp.vec3(-D * jaref, float(types.ConstraintState.QUADRATIC.value), 0.5 * D * jaref * jaref)

  if is_elliptic:
    N = jaref0 * mu
    if TT <= 0.0:
      T = 0.0
    else:
      T = wp.sqrt(TT)

    # Top zone
    if (N >= mu * T) or ((T <= 0.0) and (N >= 0.0)):
      return wp.vec3(0.0, float(types.ConstraintState.SATISFIED.value), 0.0)
    # Bottom zone
    elif (mu * N + T <= 0.0) or ((T <= 0.0) and (N < 0.0)):
      return wp.vec3(-D * jaref, float(types.ConstraintState.QUADRATIC.value), 0.5 * D * jaref * jaref)
    # Middle zone
    else:
      dm = math.safe_div(D0, mu * mu * (1.0 + mu * mu))
      nmt = N - mu * T
      force_normal = -dm * nmt * mu

      if efcid == efcid0:
        return wp.vec3(force_normal, float(types.ConstraintState.CONE.value), 0.5 * dm * nmt * nmt)
      else:
        force_tangent = -math.safe_div(force_normal, T) * ufrictionj

      return wp.vec3(force_tangent, float(types.ConstraintState.CONE.value), 0.0)

  if jaref >= 0.0:
    return wp.vec3(0.0, float(types.ConstraintState.SATISFIED.value), 0.0)
  else:
    return wp.vec3(-D * jaref, float(types.ConstraintState.QUADRATIC.value), 0.5 * D * jaref * jaref)


# kernel_analyzer: off
@wp.func
def _compute_efc_eval_pt_pyramidal(
  efcid: int,
  alpha: float,
  ne: int,
  nf: int,
  # Per-row data:
  efc_D: float,
  efc_frictionloss: wp.array[float],
  ctx_Jaref: float,
  ctx_jv: float,
) -> wp.vec3:
  """Compute shifted cost, gradient, and hessian for pyramidal cones.

  Returns (cost(alpha) - cost(0), grad(alpha), hessian(alpha)) summed across the row.
  """
  # Limit/other constraint
  if efcid >= ne + nf:
    x = ctx_Jaref + alpha * ctx_jv
    quad0 = _eval_pt_direct_cost_alpha_zero(ctx_Jaref, efc_D)
    cost0 = wp.where(ctx_Jaref < 0.0, quad0, 0.0)
    # _eval_pt_direct_shifted returns quad(alpha) - quad(0); add back quad(0) when the
    # constraint was inactive at alpha=0 (i.e. cost(0) = 0) so we get quad(alpha) - 0.
    offset = quad0 - cost0
    if x < 0.0:
      return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, offset)
    return wp.vec3(-cost0, 0.0, 0.0)

  # Friction constraint - needs quad for frictionloss computation
  if efcid >= ne:
    f = efc_frictionloss[efcid]
    x = ctx_Jaref + alpha * ctx_jv
    rf = math.safe_div(f, efc_D)
    return _shift_cost(_eval_frictionloss_pt(x, f, rf, ctx_jv, efc_D), _eval_frictionloss_cost(ctx_Jaref, f, rf, efc_D))

  # Equality constraint
  return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, 0.0)


@wp.func
def _compute_efc_eval_pt_elliptic(
  efcid: int,
  alpha: float,
  ne: int,
  nf: int,
  impratio_invsqrt: float,
  # Per-row data (arrays for deferred load):
  efc_type: int,
  efc_D_in: wp.array[float],
  efc_frictionloss: wp.array[float],
  ctx_Jaref: float,
  ctx_jv: float,
  ctx_quad: wp.vec3,
  # Contact data (for elliptic):
  contact_friction: types.vec5,
  efc_address0: int,
  quad1: wp.vec3,
  quad2: wp.vec3,
) -> wp.vec3:
  """Compute shifted cost, gradient, and hessian for elliptic cones.

  Returns (cost(alpha) - cost(0), grad(alpha), hessian(alpha)) summed across the row.
  """
  # Contact/limit/other constraints
  if efcid >= ne + nf:
    if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
      if efcid != efc_address0:  # Not primary row
        return wp.vec3(0.0)
      mu = contact_friction[0] * impratio_invsqrt
      cost0 = _eval_elliptic_cost(mu, ctx_quad, quad1, quad2, 0.0)
      return _shift_cost(_eval_elliptic(mu, ctx_quad, quad1, quad2, alpha), cost0)

    # Limit/other constraint — direct eval (no quad read)
    x = ctx_Jaref + alpha * ctx_jv
    efc_D = efc_D_in[efcid]
    quad0 = _eval_pt_direct_cost_alpha_zero(ctx_Jaref, efc_D)
    cost0 = wp.where(ctx_Jaref < 0.0, quad0, 0.0)
    # See _compute_efc_eval_pt_pyramidal for the offset rationale.
    offset = quad0 - cost0
    if x < 0.0:
      return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, offset)
    return wp.vec3(-cost0, 0.0, 0.0)

  # Friction constraint - load D and frictionloss only here
  if efcid >= ne:
    efc_D = efc_D_in[efcid]
    f = efc_frictionloss[efcid]
    x = ctx_Jaref + alpha * ctx_jv
    rf = math.safe_div(f, efc_D)
    return _shift_cost(_eval_frictionloss_pt(x, f, rf, ctx_jv, efc_D), _eval_frictionloss_cost(ctx_Jaref, f, rf, efc_D))

  # Equality constraint — direct eval (no quad read)
  efc_D = efc_D_in[efcid]
  return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, 0.0)


@wp.func
def _compute_efc_eval_pt_alpha_zero_pyramidal(
  efcid: int,
  ne: int,
  nf: int,
  # Per-row data:
  efc_D: float,
  efc_frictionloss: wp.array[float],
  ctx_Jaref: float,
  ctx_jv: float,
) -> wp.vec3:
  """Optimized version for alpha=0.0, pyramidal cones."""
  # Limit/other constraint
  if efcid >= ne + nf:
    if ctx_Jaref < 0.0:
      return _eval_pt_direct_alpha_zero(ctx_Jaref, ctx_jv, efc_D)
    return wp.vec3(0.0)

  # Friction constraint - needs quad for frictionloss computation
  if efcid >= ne:
    f = efc_frictionloss[efcid]
    rf = math.safe_div(f, efc_D)
    return _eval_frictionloss_pt(ctx_Jaref, f, rf, ctx_jv, efc_D)

  # Equality constraint
  return _eval_pt_direct_alpha_zero(ctx_Jaref, ctx_jv, efc_D)


@wp.func
def _compute_efc_eval_pt_alpha_zero_elliptic(
  efcid: int,
  ne: int,
  nf: int,
  impratio_invsqrt: float,
  # Per-row data (arrays for deferred load):
  efc_type: int,
  efc_D_in: wp.array[float],
  efc_frictionloss: wp.array[float],
  ctx_Jaref: float,
  ctx_jv: float,
  ctx_quad: wp.vec3,
  # Contact data (for elliptic):
  contact_friction: types.vec5,
  efc_address0: int,
  quad1: wp.vec3,
  quad2: wp.vec3,
) -> wp.vec3:
  """Optimized version for alpha=0.0, elliptic cones."""
  # Contact/limit/other constraints
  if efcid >= ne + nf:
    if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
      if efcid != efc_address0:  # Not primary row
        return wp.vec3(0.0)
      mu = contact_friction[0] * impratio_invsqrt
      return _eval_elliptic(mu, ctx_quad, quad1, quad2, 0.0)

    # Limit/other constraint — direct eval (no quad read)
    if ctx_Jaref < 0.0:
      return _eval_pt_direct_alpha_zero(ctx_Jaref, ctx_jv, efc_D_in[efcid])
    return wp.vec3(0.0)

  # Friction constraint - load D and frictionloss only here
  if efcid >= ne:
    efc_D = efc_D_in[efcid]
    f = efc_frictionloss[efcid]
    rf = math.safe_div(f, efc_D)
    return _eval_frictionloss_pt(ctx_Jaref, f, rf, ctx_jv, efc_D)

  # Equality constraint — direct eval (no quad read)
  return _eval_pt_direct_alpha_zero(ctx_Jaref, ctx_jv, efc_D_in[efcid])


@wp.func
def _compute_efc_eval_pt_3alphas_pyramidal(
  efcid: int,
  lo_alpha: float,
  hi_alpha: float,
  mid_alpha: float,
  ne: int,
  nf: int,
  # Per-row data:
  efc_D: float,
  efc_frictionloss: wp.array[float],
  ctx_Jaref: float,
  ctx_jv: float,
) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Compute shifted cost, gradient, and hessian for 3 alphas, pyramidal cones.

  Returns a tuple of 3 vec3s for (lo_alpha, hi_alpha, mid_alpha).
  Constraint types checked in order: limit/other -> friction -> equality.
  """
  # Limit/other constraints: active only when x < 0
  if efcid >= ne + nf:
    x_lo = ctx_Jaref + lo_alpha * ctx_jv
    x_hi = ctx_Jaref + hi_alpha * ctx_jv
    x_mid = ctx_Jaref + mid_alpha * ctx_jv
    quad0 = _eval_pt_direct_cost_alpha_zero(ctx_Jaref, efc_D)
    cost0 = wp.where(ctx_Jaref < 0.0, quad0, 0.0)
    # See _compute_efc_eval_pt_pyramidal for the offset rationale.
    offset = quad0 - cost0
    pt_lo, pt_hi, pt_mid = _eval_pt_direct_shifted_3alphas(ctx_Jaref, ctx_jv, efc_D, lo_alpha, hi_alpha, mid_alpha, offset)
    inactive = wp.vec3(-cost0, 0.0, 0.0)
    return (
      wp.where(x_lo < 0.0, pt_lo, inactive),
      wp.where(x_hi < 0.0, pt_hi, inactive),
      wp.where(x_mid < 0.0, pt_mid, inactive),
    )

  # Friction constraint - needs quad for frictionloss computation
  if efcid >= ne:
    x_lo = ctx_Jaref + lo_alpha * ctx_jv
    x_hi = ctx_Jaref + hi_alpha * ctx_jv
    x_mid = ctx_Jaref + mid_alpha * ctx_jv
    f = efc_frictionloss[efcid]
    rf = math.safe_div(f, efc_D)
    cost0 = _eval_frictionloss_cost(ctx_Jaref, f, rf, efc_D)
    lo, hi, mid = _eval_frictionloss_pt_3alphas(x_lo, x_hi, x_mid, f, rf, ctx_jv, efc_D)
    return (_shift_cost(lo, cost0), _shift_cost(hi, cost0), _shift_cost(mid, cost0))

  # Equality constraint: always active
  return _eval_pt_direct_shifted_3alphas(ctx_Jaref, ctx_jv, efc_D, lo_alpha, hi_alpha, mid_alpha, 0.0)


@wp.func
def _compute_efc_eval_pt_3alphas_elliptic(
  efcid: int,
  lo_alpha: float,
  hi_alpha: float,
  mid_alpha: float,
  ne: int,
  nf: int,
  impratio_invsqrt: float,
  # Per-row data (arrays for deferred load):
  efc_type: int,
  efc_D_in: wp.array[float],
  efc_frictionloss: wp.array[float],
  ctx_Jaref: float,
  ctx_jv: float,
  ctx_quad: wp.vec3,
  # Contact data (for elliptic):
  contact_friction: types.vec5,
  efc_address0: int,
  quad1: wp.vec3,
  quad2: wp.vec3,
) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Compute shifted cost, gradient, and hessian for 3 alphas, elliptic cones.

  Returns a tuple of 3 vec3s for (lo_alpha, hi_alpha, mid_alpha).
  Constraint types checked in order: contact elliptic/limit/other -> friction -> equality.
  """
  # x = search point, needed for friction and limit constraints
  x_lo = ctx_Jaref + lo_alpha * ctx_jv
  x_hi = ctx_Jaref + hi_alpha * ctx_jv
  x_mid = ctx_Jaref + mid_alpha * ctx_jv

  # Contact/limit/other constraints
  if efcid >= ne + nf:
    # Contact elliptic: uses special elliptic cone evaluation
    if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
      if efcid != efc_address0:  # secondary rows contribute nothing
        return (wp.vec3(0.0), wp.vec3(0.0), wp.vec3(0.0))
      mu = contact_friction[0] * impratio_invsqrt
      cost0 = _eval_elliptic_cost(mu, ctx_quad, quad1, quad2, 0.0)
      lo = _eval_elliptic(mu, ctx_quad, quad1, quad2, lo_alpha)
      hi = _eval_elliptic(mu, ctx_quad, quad1, quad2, hi_alpha)
      mid = _eval_elliptic(mu, ctx_quad, quad1, quad2, mid_alpha)
      return (_shift_cost(lo, cost0), _shift_cost(hi, cost0), _shift_cost(mid, cost0))

    # Limit/other constraints — direct eval (no quad read)
    efc_D = efc_D_in[efcid]
    quad0 = _eval_pt_direct_cost_alpha_zero(ctx_Jaref, efc_D)
    cost0 = wp.where(ctx_Jaref < 0.0, quad0, 0.0)
    # See _compute_efc_eval_pt_pyramidal for the offset rationale.
    offset = quad0 - cost0
    pt_lo, pt_hi, pt_mid = _eval_pt_direct_shifted_3alphas(ctx_Jaref, ctx_jv, efc_D, lo_alpha, hi_alpha, mid_alpha, offset)
    inactive = wp.vec3(-cost0, 0.0, 0.0)
    return (
      wp.where(x_lo < 0.0, pt_lo, inactive),
      wp.where(x_hi < 0.0, pt_hi, inactive),
      wp.where(x_mid < 0.0, pt_mid, inactive),
    )

  # Friction constraint - load D and frictionloss only here
  if efcid >= ne:
    efc_D = efc_D_in[efcid]
    f = efc_frictionloss[efcid]
    rf = math.safe_div(f, efc_D)
    cost0 = _eval_frictionloss_cost(ctx_Jaref, f, rf, efc_D)
    lo, hi, mid = _eval_frictionloss_pt_3alphas(x_lo, x_hi, x_mid, f, rf, ctx_jv, efc_D)
    return (_shift_cost(lo, cost0), _shift_cost(hi, cost0), _shift_cost(mid, cost0))

  # Equality constraint — direct eval (no quad read)
  efc_D = efc_D_in[efcid]
  return _eval_pt_direct_shifted_3alphas(ctx_Jaref, ctx_jv, efc_D, lo_alpha, hi_alpha, mid_alpha, 0.0)


# kernel_analyzer: on

# =============================================================================
# Iterative Linesearch
# =============================================================================
#
# Iterative linesearch implementation using Warp's tiled execution model with
# parallel reductions over constraint (EFC) rows.
#
# Key optimizations:
#
# 1. KERNEL FUSION - Reduces kernel launch overhead by combining:
#    - linesearch_jv_fused: jv = J @ search (for small nv <= 50)
#    - linesearch_prepare_quad: quad coefficients (pyramidal: computed directly,
#      elliptic: computed in a prepare phase with __syncthreads barrier)
#    - linesearch_prepare_gauss: quad_gauss via tile reduction over DOFs
#    - linesearch_qacc_ma: qacc and Ma updates at kernel end
#    - linesearch_jaref: Jaref update at kernel end
#
# 2. PARALLEL REDUCTIONS - Uses wp.tile_reduce for summing cost/gradient/hessian
#    contributions across EFC rows within each world. The main iteration loop
#    packs 3 vec3 reductions into a single mat33 reduction for efficiency.
#
# 3. COMPILE-TIME SPECIALIZATION via factory parameters:
#    - cone_type: Eliminates elliptic cone branches for pyramidal-only models
#    - ls_iterations: Enables loop unrolling for the main bracket search
#    - fuse_jv: Conditionally includes jv computation based on nv size
#
# 4. DIRECT EVALUATION (pyramidal only) - For equality and limit constraints,
#    computes cost/gradient/hessian directly from (Jaref, jv, efc_D, alpha)
#    without intermediate quad coefficients, using _eval_pt_direct functions.
#
# 5. BATCHED 3-ALPHA EVALUATION - The main iteration loop evaluates 3 alpha
#    values per iteration (lo_next, hi_next, mid). Instead of calling
#    _compute_efc_eval_pt 3 times per constraint row (which would repeat
#    constraint type checks and data loads), we use _compute_efc_eval_pt_3alphas
#    which:
#    - Performs constraint type branching once per row
#    - Loads efc_D, efc_frictionloss, contact data once
#    - Computes x = Jaref + alpha * jv for all 3 alphas
#    - For pyramidal direct evaluation: shares jvD = jv * efc_D, hessian = jv * jvD
#    - For quad-based evaluation: uses _eval_pt_3alphas which computes the
#      constant hessian (2.0 * quad[2]) once and reuses for all 3 alphas
#
# 6. DEFERRED DATA LOADING - efc_D and efc_frictionloss are only loaded
#    inside the constraint branches where they're needed, reducing register
#    pressure for other constraint types.
#
# Trade-offs:
# - Requires block synchronization (__syncthreads) for elliptic quad preparation
# - Separate kernel compilation for each (block_dim, ls_iterations, cone_type,
#   fuse_jv) combination (cached by Warp)
#
# Optimizations attempted but not beneficial:
# - Caching EFC data (Jaref, jv, quad, etc.) in shared memory tiles for reuse
#   across the p0, lo_in, and main iteration loops.
#
# =============================================================================


@cache_kernel
def _linesearch_iterative_kernel(ls_iterations: int, cone_type: types.ConeType, fuse_jv: bool, is_sparse: bool):
  """Factory for iterative linesearch kernel.

  Args:
    ls_iterations: Max linesearch iterations (compile-time constant for loop optimization).
    cone_type: Friction cone type (PYRAMIDAL or ELLIPTIC) for compile-time optimization.
    fuse_jv: Whether to compute jv = J @ search in-kernel (efficient for small nv).
    is_sparse: Use sparse matrix representation for constraint Jacobian.
  """
  LS_ITERATIONS = ls_iterations
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  FUSE_JV = fuse_jv
  IS_SPARSE = is_sparse

  # Native snippet for CUDA __syncthreads()
  @wp.func_native(snippet="WP_TILE_SYNC();")
  def _syncthreads():
    pass

  # Select specialized helper functions based on cone type
  if IS_ELLIPTIC:
    _compute_efc_eval_pt = _compute_efc_eval_pt_elliptic
    _compute_efc_eval_pt_alpha_zero = _compute_efc_eval_pt_alpha_zero_elliptic
    _compute_efc_eval_pt_3alphas = _compute_efc_eval_pt_3alphas_elliptic
  else:
    _compute_efc_eval_pt = _compute_efc_eval_pt_pyramidal
    _compute_efc_eval_pt_alpha_zero = _compute_efc_eval_pt_alpha_zero_pyramidal
    _compute_efc_eval_pt_3alphas = _compute_efc_eval_pt_3alphas_pyramidal

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_tolerance: wp.array[float],
    opt_ls_tolerance: wp.array[float],
    opt_impratio_invsqrt: wp.array[float],
    stat_meaninertia: wp.array[float],
    # Data in:
    ne_in: wp.array[int],
    nf_in: wp.array[int],
    nefc_in: wp.array[int],
    qfrc_smooth_in: wp.array2d[float],
    contact_friction_in: wp.array[types.vec5],
    contact_dim_in: wp.array[int],
    contact_efc_address_in: wp.array2d[int],
    efc_type_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_frictionloss_in: wp.array2d[float],
    njmax_in: int,
    nacon_in: wp.array[int],
    # In:
    ctx_Jaref_in: wp.array2d[float],
    ctx_search_in: wp.array2d[float],
    ctx_search_dot_in: wp.array[float],
    ctx_mv_in: wp.array2d[float],
    ctx_jv_in: wp.array2d[float],
    ctx_quad_in: wp.array2d[wp.vec3],
    ctx_done_in: wp.array[bool],
    # Data out:
    qacc_out: wp.array2d[float],
    efc_Ma_out: wp.array2d[float],
    # Out:
    ctx_Jaref_out: wp.array2d[float],
    ctx_jv_out: wp.array2d[float],
    ctx_quad_out: wp.array2d[wp.vec3],
    ctx_improvement_out: wp.array[float],
  ):
    worldid, tid = wp.tid()

    if ctx_done_in[worldid]:
      return

    ne = ne_in[worldid]
    nf = nf_in[worldid]
    nefc = wp.min(njmax_in, nefc_in[worldid])

    # jv = J @ search (fused for small nv)
    if wp.static(FUSE_JV):
      for efcid in range(tid, nefc, wp.block_dim()):
        jv = float(0.0)
        if wp.static(IS_SPARSE):
          rownnz = efc_J_rownnz_in[worldid, efcid]
          rowadr = efc_J_rowadr_in[worldid, efcid]
          for k in range(rownnz):
            sparseid = rowadr + k
            colind = efc_J_colind_in[worldid, 0, sparseid]
            jv += efc_J_in[worldid, 0, sparseid] * ctx_search_in[worldid, colind]
        else:
          for i in range(nv):
            jv += efc_J_in[worldid, efcid, i] * ctx_search_in[worldid, i]
        ctx_jv_out[worldid, efcid] = jv

      _syncthreads()  # ensure all jv values are written before reading

    # quad coefficients (elliptic contacts only, requires barrier sync)
    # Non-elliptic constraints (equality, friction, limit) now use direct
    # evaluation from (Jaref, jv, efc_D), avoiding quad reads entirely.
    if wp.static(IS_ELLIPTIC):
      # elliptic-only config values
      impratio_invsqrt = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
      nacon = nacon_in[0]

      for efcid in range(tid, nefc, wp.block_dim()):
        # Only compute and store quad for CONTACT_ELLIPTIC (needs inter-row data)
        if efc_type_in[worldid, efcid] == types.ConstraintType.CONTACT_ELLIPTIC:
          conid = efc_id_in[worldid, efcid]
          if conid < nacon:
            efcid0 = contact_efc_address_in[conid, 0]
            if efcid == efcid0:
              Jaref = ctx_Jaref_in[worldid, efcid]
              jv = ctx_jv_in[worldid, efcid]
              efc_D = efc_D_in[worldid, efcid]

              jvD = jv * efc_D
              quad = wp.vec3(0.5 * Jaref * Jaref * efc_D, jvD * Jaref, 0.5 * jv * jvD)

              # primary row: accumulate secondary rows and write quad, quad1, quad2
              dim = contact_dim_in[conid]
              friction = contact_friction_in[conid]
              mu = friction[0] * impratio_invsqrt

              u0 = Jaref * mu
              v0 = jv * mu

              uu = float(0.0)
              uv = float(0.0)
              vv = float(0.0)
              for j in range(1, dim):
                efcidj = contact_efc_address_in[conid, j]
                if efcidj >= 0:
                  jvj = ctx_jv_in[worldid, efcidj]
                  jarefj = ctx_Jaref_in[worldid, efcidj]
                  dj = efc_D_in[worldid, efcidj]
                  DJj = dj * jarefj

                  quad += wp.vec3(0.5 * jarefj * DJj, jvj * DJj, 0.5 * jvj * dj * jvj)

                  # rescale to make primal cone circular
                  frictionj = friction[j - 1]
                  uj = jarefj * frictionj
                  vj = jvj * frictionj

                  uu += uj * uj
                  uv += uj * vj
                  vv += vj * vj

              ctx_quad_out[worldid, efcid] = quad

              efcid1 = contact_efc_address_in[conid, 1]
              ctx_quad_out[worldid, efcid1] = wp.vec3(u0, v0, uu)

              mu2 = mu * mu
              efcid2 = contact_efc_address_in[conid, 2]
              ctx_quad_out[worldid, efcid2] = wp.vec3(uv, vv, efc_D / (mu2 * (1.0 + mu2)))

      _syncthreads()  # ensure all quads are written before reading

    # gtol (tolerance values loaded here, deferred from kernel start)
    tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
    ls_tolerance = opt_ls_tolerance[worldid % opt_ls_tolerance.shape[0]]
    snorm = wp.sqrt(ctx_search_dot_in[worldid])
    meaninertia = stat_meaninertia[worldid % stat_meaninertia.shape[0]]
    scale = meaninertia * wp.float(nv)
    gtol = wp.max(tolerance * ls_tolerance * snorm * scale, 1e-6)

    # p0 via parallel reduction
    local_p0 = wp.vec3(0.0)
    for efcid in range(tid, nefc, wp.block_dim()):
      if wp.static(IS_ELLIPTIC):
        efc_type = efc_type_in[worldid, efcid]
        efc_id = 0
        contact_friction = types.vec5(0.0)
        efc_addr0 = int(0)
        ctx_quad = wp.vec3(0.0)
        quad1 = wp.vec3(0.0)
        quad2 = wp.vec3(0.0)

        if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
          efc_id = efc_id_in[worldid, efcid]
          contact_friction = contact_friction_in[efc_id]
          efc_addr0 = contact_efc_address_in[efc_id, 0]
          efc_addr1 = contact_efc_address_in[efc_id, 1]
          efc_addr2 = contact_efc_address_in[efc_id, 2]
          ctx_quad = ctx_quad_in[worldid, efcid]
          quad1 = ctx_quad_in[worldid, efc_addr1]
          quad2 = ctx_quad_in[worldid, efc_addr2]

        local_p0 += _compute_efc_eval_pt_alpha_zero(
          efcid,
          ne,
          nf,
          impratio_invsqrt,
          efc_type,
          efc_D_in[worldid],
          efc_frictionloss_in[worldid],
          ctx_Jaref_in[worldid, efcid],
          ctx_jv_in[worldid, efcid],
          ctx_quad,
          contact_friction,
          efc_addr0,
          quad1,
          quad2,
        )
      else:
        # direct evaluation for pyramidal cones (no intermediate quad)
        local_p0 += _compute_efc_eval_pt_alpha_zero(
          efcid,
          ne,
          nf,
          efc_D_in[worldid, efcid],
          efc_frictionloss_in[worldid],
          ctx_Jaref_in[worldid, efcid],
          ctx_jv_in[worldid, efcid],
        )

    # at this point, every thread has computed some contributions to p0 in local_p0
    # we now create a tile of all local_p0 contributions and reduce them to a single value
    # this is done in parallel using a tile reduction
    p0_tile = wp.tile(local_p0, preserve_type=True)
    p0_sum = wp.tile_reduce(wp.add, p0_tile)

    # quad_gauss = [0, search.T @ Ma - search.T @ qfrc_smooth, 0.5 * search.T @ mv]
    local_gauss = wp.vec2(0.0)
    for dofid in range(tid, nv, wp.block_dim()):
      search = ctx_search_in[worldid, dofid]
      local_gauss += wp.vec2(
        search * (efc_Ma_out[worldid, dofid] - qfrc_smooth_in[worldid, dofid]),
        0.5 * search * ctx_mv_in[worldid, dofid],
      )

    gauss_tile = wp.tile(local_gauss, preserve_type=True)
    gauss_sum = wp.tile_reduce(wp.add, gauss_tile)
    gauss_reduced = gauss_sum[0]
    ctx_quad_gauss = wp.vec3(0.0, gauss_reduced[0], gauss_reduced[1])

    # add quad_gauss contribution to p0
    p0 = wp.vec3(ctx_quad_gauss[0], ctx_quad_gauss[1], 2.0 * ctx_quad_gauss[2]) + p0_sum[0]
    p0_delta = wp.vec3(0.0, p0[1], p0[2])

    # lo_in at lo_alpha_in = -p0[1] / p0[2]
    lo_alpha_in = -math.safe_div(p0[1], p0[2])

    local_lo_in = wp.vec3(0.0)
    for efcid in range(tid, nefc, wp.block_dim()):
      if wp.static(IS_ELLIPTIC):
        efc_type = efc_type_in[worldid, efcid]
        efc_id = 0
        contact_friction = types.vec5(0.0)
        efc_addr0 = int(0)
        ctx_quad = wp.vec3(0.0)
        quad1 = wp.vec3(0.0)
        quad2 = wp.vec3(0.0)

        if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
          efc_id = efc_id_in[worldid, efcid]
          contact_friction = contact_friction_in[efc_id]
          efc_addr0 = contact_efc_address_in[efc_id, 0]
          efc_addr1 = contact_efc_address_in[efc_id, 1]
          efc_addr2 = contact_efc_address_in[efc_id, 2]
          ctx_quad = ctx_quad_in[worldid, efcid]
          quad1 = ctx_quad_in[worldid, efc_addr1]
          quad2 = ctx_quad_in[worldid, efc_addr2]

        local_lo_in += _compute_efc_eval_pt(
          efcid,
          lo_alpha_in,
          ne,
          nf,
          impratio_invsqrt,
          efc_type,
          efc_D_in[worldid],
          efc_frictionloss_in[worldid],
          ctx_Jaref_in[worldid, efcid],
          ctx_jv_in[worldid, efcid],
          ctx_quad,
          contact_friction,
          efc_addr0,
          quad1,
          quad2,
        )
      else:
        # direct evaluation for pyramidal cones (no intermediate quad)
        local_lo_in += _compute_efc_eval_pt(
          efcid,
          lo_alpha_in,
          ne,
          nf,
          efc_D_in[worldid, efcid],
          efc_frictionloss_in[worldid],
          ctx_Jaref_in[worldid, efcid],
          ctx_jv_in[worldid, efcid],
        )

    lo_in_tile = wp.tile(local_lo_in, preserve_type=True)
    lo_in_sum = wp.tile_reduce(wp.add, lo_in_tile)
    lo_in = _eval_pt(ctx_quad_gauss, lo_alpha_in) + lo_in_sum[0]

    # accept Newton step if derivative is small and cost improved
    initial_converged = wp.abs(lo_in[1]) < gtol and lo_in[0] < 0.0

    # main iterative loop - skip if already converged
    if not initial_converged:
      alpha = float(0.0)
      improvement = float(0.0)

      # initialize bounds
      lo_less = lo_in[1] < p0[1]
      lo = wp.where(lo_less, lo_in, p0_delta)
      lo_alpha = wp.where(lo_less, lo_alpha_in, 0.0)
      hi = wp.where(lo_less, p0_delta, lo_in)
      hi_alpha = wp.where(lo_less, 0.0, lo_alpha_in)

      for _ in range(LS_ITERATIONS):
        lo_next_alpha = lo_alpha - math.safe_div(lo[1], lo[2])
        hi_next_alpha = hi_alpha - math.safe_div(hi[1], hi[2])
        mid_alpha = 0.5 * (lo_alpha + hi_alpha)

        local_lo = wp.vec3(0.0)
        local_hi = wp.vec3(0.0)
        local_mid = wp.vec3(0.0)

        for efcid in range(tid, nefc, wp.block_dim()):
          if wp.static(IS_ELLIPTIC):
            efc_type = efc_type_in[worldid, efcid]
            efc_id = 0
            contact_friction = types.vec5(0.0)
            efc_addr0 = int(0)
            ctx_quad = wp.vec3(0.0)
            quad1 = wp.vec3(0.0)
            quad2 = wp.vec3(0.0)

            if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
              efc_id = efc_id_in[worldid, efcid]
              contact_friction = contact_friction_in[efc_id]
              efc_addr0 = contact_efc_address_in[efc_id, 0]
              efc_addr1 = contact_efc_address_in[efc_id, 1]
              efc_addr2 = contact_efc_address_in[efc_id, 2]
              ctx_quad = ctx_quad_in[worldid, efcid]
              quad1 = ctx_quad_in[worldid, efc_addr1]
              quad2 = ctx_quad_in[worldid, efc_addr2]

            r_lo, r_hi, r_mid = _compute_efc_eval_pt_3alphas(
              efcid,
              lo_next_alpha,
              hi_next_alpha,
              mid_alpha,
              ne,
              nf,
              impratio_invsqrt,
              efc_type,
              efc_D_in[worldid],
              efc_frictionloss_in[worldid],
              ctx_Jaref_in[worldid, efcid],
              ctx_jv_in[worldid, efcid],
              ctx_quad,
              contact_friction,
              efc_addr0,
              quad1,
              quad2,
            )
          else:
            # direct evaluation for pyramidal cones (no intermediate quad)
            r_lo, r_hi, r_mid = _compute_efc_eval_pt_3alphas(
              efcid,
              lo_next_alpha,
              hi_next_alpha,
              mid_alpha,
              ne,
              nf,
              efc_D_in[worldid, efcid],
              efc_frictionloss_in[worldid],
              ctx_Jaref_in[worldid, efcid],
              ctx_jv_in[worldid, efcid],
            )
          local_lo += r_lo
          local_hi += r_hi
          local_mid += r_mid

        # reduce with packed mat33 (3 vec3s into columns: col0=lo, col1=hi, col2=mid)
        local_combined = wp.mat33(
          local_lo[0],
          local_hi[0],
          local_mid[0],
          local_lo[1],
          local_hi[1],
          local_mid[1],
          local_lo[2],
          local_hi[2],
          local_mid[2],
        )

        # reduce with packed mat33 (3 vec3s into columns: col0=lo, col1=hi, col2=mid)
        # this is faster than 3 vec3 reductions because it avoids synchronization barriers
        combined_tile = wp.tile(local_combined, preserve_type=True)
        combined_sum = wp.tile_reduce(wp.add, combined_tile)
        result = combined_sum[0]

        # extract columns back to vec3s and add quad_gauss contributions
        gauss_lo, gauss_hi, gauss_mid = _eval_pt_3alphas(ctx_quad_gauss, lo_next_alpha, hi_next_alpha, mid_alpha)
        lo_next = gauss_lo + wp.vec3(result[0, 0], result[1, 0], result[2, 0])
        hi_next = gauss_hi + wp.vec3(result[0, 1], result[1, 1], result[2, 1])
        mid = gauss_mid + wp.vec3(result[0, 2], result[1, 2], result[2, 2])

        # bracket swapping
        # swap lo:
        swap_lo_lo_next = _in_bracket(lo, lo_next)
        lo = wp.where(swap_lo_lo_next, lo_next, lo)
        lo_alpha = wp.where(swap_lo_lo_next, lo_next_alpha, lo_alpha)
        swap_lo_mid = _in_bracket(lo, mid)
        lo = wp.where(swap_lo_mid, mid, lo)
        lo_alpha = wp.where(swap_lo_mid, mid_alpha, lo_alpha)
        swap_lo_hi_next = _in_bracket(lo, hi_next)
        lo = wp.where(swap_lo_hi_next, hi_next, lo)
        lo_alpha = wp.where(swap_lo_hi_next, hi_next_alpha, lo_alpha)
        swap_lo = swap_lo_lo_next or swap_lo_mid or swap_lo_hi_next

        # swap hi:
        swap_hi_hi_next = _in_bracket(hi, hi_next)
        hi = wp.where(swap_hi_hi_next, hi_next, hi)
        hi_alpha = wp.where(swap_hi_hi_next, hi_next_alpha, hi_alpha)
        swap_hi_mid = _in_bracket(hi, mid)
        hi = wp.where(swap_hi_mid, mid, hi)
        hi_alpha = wp.where(swap_hi_mid, mid_alpha, hi_alpha)
        swap_hi_lo_next = _in_bracket(hi, lo_next)
        hi = wp.where(swap_hi_lo_next, lo_next, hi)
        hi_alpha = wp.where(swap_hi_lo_next, lo_next_alpha, hi_alpha)
        swap_hi = swap_hi_hi_next or swap_hi_mid or swap_hi_lo_next

        # check for convergence
        ls_done = (not swap_lo and not swap_hi) or (lo[1] < 0.0 and lo[1] > -gtol) or (hi[1] > 0.0 and hi[1] < gtol)

        # update alpha if improved
        improved = lo[0] < 0.0 or hi[0] < 0.0
        lo_better = lo[0] < hi[0]
        best_alpha = wp.where(lo_better, lo_alpha, hi_alpha)
        best_delta = wp.where(lo_better, lo[0], hi[0])
        alpha = wp.where(improved, best_alpha, alpha)
        improvement = wp.where(improved, -best_delta, improvement)

        if ls_done:
          break
    else:
      alpha = lo_alpha_in
      improvement = -lo_in[0]

    # qacc and Ma update
    for dofid in range(tid, nv, wp.block_dim()):
      qacc_out[worldid, dofid] += alpha * ctx_search_in[worldid, dofid]
      efc_Ma_out[worldid, dofid] += alpha * ctx_mv_in[worldid, dofid]

    # Jaref update
    for efcid in range(tid, nefc, wp.block_dim()):
      ctx_Jaref_out[worldid, efcid] += alpha * ctx_jv_in[worldid, efcid]

    if tid == 0:
      ctx_improvement_out[worldid] = improvement

  return kernel


def _linesearch_iterative(m: types.Model, d: types.Data, ctx: SolverContext, fuse_jv: bool):
  """Iterative linesearch with parallel reductions over efc rows and dofs.

  Args:
    m: Model.
    d: Data.
    ctx: SolverContext.
    fuse_jv: Whether jv is computed in-kernel (True) or pre-computed (False).
  """
  wp.launch_tiled(
    _linesearch_iterative_kernel(m.opt.ls_iterations, m.opt.cone, fuse_jv, m.is_sparse),
    dim=d.nworld,
    inputs=[
      m.nv,
      m.opt.tolerance,
      m.opt.ls_tolerance,
      m.opt.impratio_invsqrt,
      m.stat.meaninertia,
      d.ne,
      d.nf,
      d.nefc,
      d.qfrc_smooth,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.efc.type,
      d.efc.id,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.J,
      d.efc.D,
      d.efc.frictionloss,
      d.njmax,
      d.nacon,
      ctx.Jaref,
      ctx.search,
      ctx.search_dot,
      ctx.mv,
      ctx.jv,
      ctx.quad,
      ctx.done,
    ],
    outputs=[d.qacc, d.efc.Ma, ctx.Jaref, ctx.jv, ctx.quad, ctx.improvement],
    block_dim=m.block_dim.linesearch_iterative,
  )


@wp.kernel
def _linesearch_zero_jv(
  # Data in:
  nefc_in: wp.array[int],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_jv_out: wp.array2d[float],
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if ctx_done_in[worldid]:
    return

  ctx_jv_out[worldid, efcid] = 0.0


@cache_kernel
def _linesearch_jv_fused_kernel(is_sparse: bool, nv: int, dofs_per_thread: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    # In:
    ctx_search_in: wp.array2d[float],
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_jv_out: wp.array2d[float],
  ):
    worldid, efcid, dofstart = wp.tid()

    if efcid >= nefc_in[worldid]:
      return

    if ctx_done_in[worldid]:
      return

    jv_out = float(0.0)

    if wp.static(dofs_per_thread >= nv):
      if wp.static(is_sparse):
        # Sparse: iterate over non-zero entries in the row
        rownnz = efc_J_rownnz_in[worldid, efcid]
        rowadr = efc_J_rowadr_in[worldid, efcid]
        for k in range(rownnz):
          sparseid = rowadr + k
          colind = efc_J_colind_in[worldid, 0, sparseid]
          jv_out += efc_J_in[worldid, 0, sparseid] * ctx_search_in[worldid, colind]
      else:
        for i in range(wp.static(min(dofs_per_thread, nv))):
          jv_out += efc_J_in[worldid, efcid, i] * ctx_search_in[worldid, i]
      ctx_jv_out[worldid, efcid] = jv_out

    else:
      if wp.static(is_sparse):
        # Sparse: thread 0 handles entire row (sparse entries << nv typically)
        if dofstart == 0:
          rownnz = efc_J_rownnz_in[worldid, efcid]
          rowadr = efc_J_rowadr_in[worldid, efcid]
          for k in range(rownnz):
            sparseid = rowadr + k
            colind = efc_J_colind_in[worldid, 0, sparseid]
            jv_out += efc_J_in[worldid, 0, sparseid] * ctx_search_in[worldid, colind]
          ctx_jv_out[worldid, efcid] = jv_out
      else:
        for i in range(wp.static(dofs_per_thread)):
          ii = dofstart * wp.static(dofs_per_thread) + i
          if ii < nv:
            jv_out += efc_J_in[worldid, efcid, ii] * ctx_search_in[worldid, ii]
        wp.atomic_add(ctx_jv_out, worldid, efcid, jv_out)

  return kernel


@event_scope
def _linesearch(m: types.Model, d: types.Data, ctx: SolverContext):
  """Linesearch for constraint solver.

  Args:
    m: Model
    d: Data
    ctx: SolverContext
  """
  # mv = M @ search (common to both parallel and iterative)
  support.mul_m(m, d, ctx.mv, ctx.search, skip=ctx.done)

  # Fuse jv computation in-kernel for small nv (iterative only, dense only)
  # Sparse mode requires pre-computed jv since in-kernel uses dense indexing
  fuse_jv = m.nv <= 50 and not m.is_sparse

  # jv = J @ search (when not fused into iterative kernel)
  if not fuse_jv:
    dofs_per_thread = 20 if m.nv > 50 else 50
    threads_per_efc = ceil(m.nv / dofs_per_thread)

    if threads_per_efc > 1:
      wp.launch(
        _linesearch_zero_jv,
        dim=(d.nworld, d.njmax),
        inputs=[d.nefc, ctx.done],
        outputs=[ctx.jv],
      )

    wp.launch(
      _linesearch_jv_fused_kernel(m.is_sparse, m.nv, dofs_per_thread),
      dim=(d.nworld, d.njmax, threads_per_efc),
      inputs=[d.nefc, d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J, ctx.search, ctx.done],
      outputs=[ctx.jv],
    )

  _linesearch_iterative(m, d, ctx, fuse_jv)


@wp.kernel
def _solve_init_efc(
  # Data out:
  solver_niter_out: wp.array[int],
  # Out:
  ctx_search_dot_out: wp.array[float],
  ctx_done_out: wp.array[bool],
):
  worldid = wp.tid()
  solver_niter_out[worldid] = 0
  ctx_done_out[worldid] = False
  ctx_search_dot_out[worldid] = 0.0


@cache_kernel
def _solve_init_jaref_kernel(is_sparse: bool, nv: int, dofs_per_thread: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    qacc_in: wp.array2d[float],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_aref_in: wp.array2d[float],
    # Out:
    ctx_Jaref_out: wp.array2d[float],
  ):
    worldid, efcid, dofstart = wp.tid()

    if efcid >= nefc_in[worldid]:
      return

    jaref = float(0.0)
    if wp.static(is_sparse):
      rownnz = efc_J_rownnz_in[worldid, efcid]
      rowadr = efc_J_rowadr_in[worldid, efcid]
      for i in range(rownnz):
        sparseid = rowadr + i
        colind = efc_J_colind_in[worldid, 0, sparseid]
        jaref += efc_J_in[worldid, 0, sparseid] * qacc_in[worldid, colind]
      ctx_Jaref_out[worldid, efcid] = jaref - efc_aref_in[worldid, efcid]
    else:
      if wp.static(dofs_per_thread >= nv):
        for i in range(wp.static(min(dofs_per_thread, nv))):
          jaref += efc_J_in[worldid, efcid, i] * qacc_in[worldid, i]
        ctx_Jaref_out[worldid, efcid] = jaref - efc_aref_in[worldid, efcid]

      else:
        for i in range(wp.static(dofs_per_thread)):
          ii = dofstart * wp.static(dofs_per_thread) + i
          if ii < nv:
            jaref += efc_J_in[worldid, efcid, ii] * qacc_in[worldid, ii]

        if dofstart == 0:
          wp.atomic_add(ctx_Jaref_out, worldid, efcid, jaref - efc_aref_in[worldid, efcid])
        else:
          wp.atomic_add(ctx_Jaref_out, worldid, efcid, jaref)

  return kernel


@wp.kernel
def _solve_init_search(
  # In:
  ctx_Mgrad_in: wp.array2d[float],
  # Out:
  ctx_search_out: wp.array2d[float],
  ctx_search_dot_out: wp.array[float],
):
  worldid, dofid = wp.tid()
  search = -1.0 * ctx_Mgrad_in[worldid, dofid]
  ctx_search_out[worldid, dofid] = search
  wp.atomic_add(ctx_search_dot_out, worldid, search * search)


@wp.kernel
def _solve_init_search_cg_tiled(
  # Model:
  nv: int,
  # In:
  ctx_grad_in: wp.array2d[float],
  ctx_Mgrad_in: wp.array2d[float],
  # Out:
  ctx_search_out: wp.array2d[float],
  ctx_search_dot_out: wp.array[float],
  ctx_prev_grad_out: wp.array2d[float],
  ctx_prev_Mgrad_out: wp.array2d[float],
):
  worldid, tid = wp.tid()

  local_search_dot = float(0.0)
  BLOCK_DIM = wp.block_dim()

  for dofid in range(tid, nv, BLOCK_DIM):
    mgrad = ctx_Mgrad_in[worldid, dofid]
    search = -1.0 * mgrad
    ctx_search_out[worldid, dofid] = search
    local_search_dot += search * search

    ctx_prev_grad_out[worldid, dofid] = ctx_grad_in[worldid, dofid]
    ctx_prev_Mgrad_out[worldid, dofid] = mgrad

  search_dot_tile = wp.tile(local_search_dot, preserve_type=True)
  search_dot_sum = wp.tile_reduce(wp.add, search_dot_tile)

  if tid == 0:
    ctx_search_dot_out[worldid] = search_dot_sum[0]


@cache_kernel
def _update_constraint_efc(track_changes: bool):
  TRACK_CHANGES = track_changes

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_impratio_invsqrt: wp.array[float],
    # Data in:
    ne_in: wp.array[int],
    nf_in: wp.array[int],
    nefc_in: wp.array[int],
    contact_friction_in: wp.array[types.vec5],
    contact_dim_in: wp.array[int],
    contact_efc_address_in: wp.array2d[int],
    efc_type_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    efc_D_in: wp.array2d[float],
    efc_frictionloss_in: wp.array2d[float],
    nacon_in: wp.array[int],
    # In:
    ctx_Jaref_in: wp.array2d[float],
    ctx_done_in: wp.array[bool],
    # Data out:
    efc_force_out: wp.array2d[float],
    efc_state_out: wp.array2d[int],
    # Out:
    changed_ids_out: wp.array2d[int],
    changed_count_out: wp.array[int],
  ):
    worldid, efcid = wp.tid()

    if efcid >= nefc_in[worldid]:
      return

    if ctx_done_in[worldid]:
      return

    # Read old QUADRATIC status before overwriting
    if wp.static(TRACK_CHANGES):
      old_quad = efc_state_out[worldid, efcid] == types.ConstraintState.QUADRATIC.value

    efc_D = efc_D_in[worldid, efcid]
    Jaref = ctx_Jaref_in[worldid, efcid]

    ne = ne_in[worldid]
    nf = nf_in[worldid]

    is_equality = efcid < ne
    is_friction = (not is_equality) and (efcid < ne + nf)
    is_elliptic = efc_type_in[worldid, efcid] == types.ConstraintType.CONTACT_ELLIPTIC

    frictionloss = efc_frictionloss_in[worldid, efcid] if is_friction else 0.0

    efcid0 = -1
    jaref0 = float(0.0)
    D0 = float(0.0)
    mu = float(0.0)
    ufrictionj = float(0.0)
    TT = float(0.0)

    if is_elliptic:
      conid = efc_id_in[worldid, efcid]
      if conid >= nacon_in[0]:
        return
      efcid0 = contact_efc_address_in[conid, 0]
      if efcid0 < 0:
        return

      dim = contact_dim_in[conid]
      friction = contact_friction_in[conid]
      mu = friction[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
      jaref0 = ctx_Jaref_in[worldid, efcid0]
      D0 = efc_D_in[worldid, efcid0]

      for j in range(1, dim):
        efcidj = contact_efc_address_in[conid, j]
        if efcidj < 0:
          return
        frictionj = friction[j - 1]
        uj = ctx_Jaref_in[worldid, efcidj] * frictionj
        TT += uj * uj
        if efcid == efcidj:
          ufrictionj = uj * frictionj

    res = _eval_constraint(
      is_equality,
      is_friction,
      is_elliptic,
      Jaref,
      efc_D,
      frictionloss,
      efcid,
      efcid0,
      jaref0,
      D0,
      mu,
      ufrictionj,
      TT,
    )

    new_state = int(res[1])
    efc_force_out[worldid, efcid] = res[0]
    efc_state_out[worldid, efcid] = new_state

    if wp.static(TRACK_CHANGES):
      new_quad = new_state == types.ConstraintState.QUADRATIC.value
      if old_quad != new_quad:
        idx = wp.atomic_add(changed_count_out, worldid, 1)
        changed_ids_out[worldid, idx] = efcid

  return kernel


@wp.kernel
def _update_constraint_init_qfrc_constraint_sparse(
  # Data in:
  nefc_in: wp.array[int],
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  efc_force_in: wp.array2d[float],
  # In:
  ctx_done_in: wp.array[bool],
  # Data out:
  qfrc_constraint_out: wp.array2d[float],
):
  worldid, efcid = wp.tid()

  if ctx_done_in[worldid]:
    return

  if efcid >= nefc_in[worldid]:
    return

  force = efc_force_in[worldid, efcid]

  rownnz = efc_J_rownnz_in[worldid, efcid]
  rowadr = efc_J_rowadr_in[worldid, efcid]
  for i in range(rownnz):
    sparseid = rowadr + i
    colind = efc_J_colind_in[worldid, 0, sparseid]
    efc_J = efc_J_in[worldid, 0, sparseid]
    wp.atomic_add(qfrc_constraint_out[worldid], colind, efc_J * force)


@wp.kernel
def _update_constraint_init_qfrc_constraint_dense(
  # Data in:
  nefc_in: wp.array[int],
  efc_J_in: wp.array3d[float],
  efc_force_in: wp.array2d[float],
  njmax_in: int,
  # In:
  ctx_done_in: wp.array[bool],
  # Data out:
  qfrc_constraint_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()

  if ctx_done_in[worldid]:
    return

  sum_qfrc = float(0.0)
  for efcid in range(min(njmax_in, nefc_in[worldid])):
    efc_J = efc_J_in[worldid, efcid, dofid]
    force = efc_force_in[worldid, efcid]
    sum_qfrc += efc_J * force

  qfrc_constraint_out[worldid, dofid] = sum_qfrc


@wp.kernel
def _update_gradient_h_incremental(
  # Data in:
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  # In:
  changed_ids_in: wp.array2d[int],
  changed_count_in: wp.array[int],
  # Out:
  ctx_h_out: wp.array3d[float],
):
  """Incrementally update upper triangle of H for changed constraints.

  Each thread handles one unique (i, j) element and writes it to the upper triangle.
  For each changed constraint, adds or subtracts D * J[i] * J[j].
  """
  worldid, elementid = wp.tid()

  n_changes = changed_count_in[worldid]
  if n_changes == 0:
    return

  # Upper-triangle enumeration: elementid -> (row, col) where row <= col.
  col = (int(wp.sqrt(float(1 + 8 * elementid))) - 1) // 2
  row = elementid - (col * (col + 1)) // 2

  delta = float(0.0)
  for change_idx in range(n_changes):
    efcid = changed_ids_in[worldid, change_idx]
    Jrow = efc_J_in[worldid, efcid, row]
    if Jrow == 0.0:
      continue
    Jcol = efc_J_in[worldid, efcid, col]
    if Jcol == 0.0:
      continue

    D = efc_D_in[worldid, efcid]
    if efc_state_in[worldid, efcid] == types.ConstraintState.QUADRATIC.value:
      delta += D * Jrow * Jcol
    else:
      delta -= D * Jrow * Jcol

  if delta != 0.0:
    ctx_h_out[worldid, row, col] += delta


@wp.kernel
def _update_gradient_h_incremental_sparse(
  # Data in:
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  # In:
  changed_ids_in: wp.array2d[int],
  changed_count_in: wp.array[int],
  # Out:
  ctx_h_out: wp.array3d[float],
):
  """Incrementally update upper triangle of H for changed constraints (sparse J)."""
  worldid, change_idx = wp.tid()

  n_changes = changed_count_in[worldid]
  if change_idx >= n_changes:
    return

  efcid = changed_ids_in[worldid, change_idx]
  D = efc_D_in[worldid, efcid]
  sign = float(0.0)
  if efc_state_in[worldid, efcid] == types.ConstraintState.QUADRATIC.value:
    sign = D
  else:
    sign = -D

  rownnz = efc_J_rownnz_in[worldid, efcid]
  rowadr = efc_J_rowadr_in[worldid, efcid]

  for ii in range(rownnz):
    sparseidi = rowadr + ii
    Ji = efc_J_in[worldid, 0, sparseidi]
    if Ji == 0.0:
      continue
    colindi = efc_J_colind_in[worldid, 0, sparseidi]
    for jj in range(ii + 1):
      sparseidj = rowadr + jj
      Jj = efc_J_in[worldid, 0, sparseidj]
      if Jj == 0.0:
        continue
      colindj = efc_J_colind_in[worldid, 0, sparseidj]
      h = sign * Ji * Jj
      # Ensure upper triangle: smaller index first.
      if colindi <= colindj:
        wp.atomic_add(ctx_h_out[worldid, colindi], colindj, h)
      else:
        wp.atomic_add(ctx_h_out[worldid, colindj], colindi, h)


def _update_constraint(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, track_changes: bool = False):
  """Update constraint arrays after each solve iteration."""
  efc_inputs = [
    m.opt.impratio_invsqrt,
    d.ne,
    d.nf,
    d.nefc,
    d.contact.friction,
    d.contact.dim,
    d.contact.efc_address,
    d.efc.type,
    d.efc.id,
    d.efc.D,
    d.efc.frictionloss,
    d.nacon,
    ctx.Jaref,
    ctx.done,
  ]

  wp.launch(
    _update_constraint_efc(track_changes),
    dim=(d.nworld, d.njmax),
    inputs=efc_inputs,
    outputs=[d.efc.force, d.efc.state, ctx.changed_efc_ids, ctx.changed_efc_count],
  )

  # qfrc_constraint = efc_J.T @ efc_force
  if m.is_sparse:
    d.qfrc_constraint.zero_()
    wp.launch(
      _update_constraint_init_qfrc_constraint_sparse,
      dim=(d.nworld, d.njmax),
      inputs=[d.nefc, d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J, d.efc.force, ctx.done],
      outputs=[d.qfrc_constraint],
    )
  else:
    wp.launch(
      _update_constraint_init_qfrc_constraint_dense,
      dim=(d.nworld, m.nv),
      inputs=[d.nefc, d.efc.J, d.efc.force, d.njmax, ctx.done],
      outputs=[d.qfrc_constraint],
    )


@wp.kernel
def _update_gradient_zero_grad_dot(
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_grad_dot_out: wp.array[float],
):
  worldid = wp.tid()

  if ctx_done_in[worldid]:
    return

  ctx_grad_dot_out[worldid] = 0.0


@wp.kernel
def _update_gradient_grad(
  # Data in:
  qfrc_smooth_in: wp.array2d[float],
  qfrc_constraint_in: wp.array2d[float],
  efc_Ma_in: wp.array2d[float],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_grad_out: wp.array2d[float],
  ctx_grad_dot_out: wp.array[float],
):
  worldid, dofid = wp.tid()

  if ctx_done_in[worldid]:
    return

  grad = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - qfrc_constraint_in[worldid, dofid]
  ctx_grad_out[worldid, dofid] = grad
  wp.atomic_add(ctx_grad_dot_out, worldid, grad * grad)


@wp.kernel
def _update_gradient_grad_tiled(
  # Model:
  nv: int,
  # Data in:
  qfrc_smooth_in: wp.array2d[float],
  qfrc_constraint_in: wp.array2d[float],
  efc_Ma_in: wp.array2d[float],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_grad_out: wp.array2d[float],
  ctx_grad_dot_out: wp.array[float],
):
  worldid, tid = wp.tid()

  if ctx_done_in[worldid]:
    return

  local_grad_dot = float(0.0)
  BLOCK_DIM = wp.block_dim()

  for dofid in range(tid, nv, BLOCK_DIM):
    grad = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - qfrc_constraint_in[worldid, dofid]
    ctx_grad_out[worldid, dofid] = grad
    local_grad_dot += grad * grad

  grad_dot_tile = wp.tile(local_grad_dot, preserve_type=True)
  grad_dot_sum = wp.tile_reduce(wp.add, grad_dot_tile)

  if tid == 0:
    ctx_grad_dot_out[worldid] = grad_dot_sum[0]


@wp.kernel
def _update_gradient_init_h_sparse(
  # Model:
  nv: int,
  M_elemid: wp.array2d[int],
  # Data in:
  M_in: wp.array3d[float],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_h_out: wp.array3d[float],
):
  worldid, i, j = wp.tid()

  if ctx_done_in[worldid]:
    return

  # only write the upper triangle; Cholesky reads the upper triangle only
  if j < i:
    return

  if i >= nv or j >= nv:
    ctx_h_out[worldid, i, j] = 0.0
    return

  # M is stored in the lower triangle, so transpose the lookup for the upper
  elemid = M_elemid[j, i]
  if elemid >= 0:
    ctx_h_out[worldid, i, j] = M_in[worldid, 0, elemid]
  else:
    ctx_h_out[worldid, i, j] = 0.0


@wp.func
def _state_check(D: float, state: int) -> float:
  if state == types.ConstraintState.QUADRATIC.value:
    return D
  else:
    return 0.0


@wp.func
def _active_check(tid: int, threshold: int) -> float:
  if tid >= threshold:
    return 0.0
  else:
    return 1.0


@cache_kernel
def _update_gradient_JTDAJ_dense_tiled(nv_pad: int, tile_size: int, njmax: int):
  if njmax < tile_size:
    tile_size = njmax

  TILE_SIZE_K = tile_size

  @wp.kernel(module="unique", enable_backward=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    M_in: wp.array3d[float],
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_state_in: wp.array2d[int],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    worldid = wp.tid()

    if ctx_done_in[worldid]:
      return

    nefc = nefc_in[worldid]

    sum_val = wp.tile_load(M_in[worldid], shape=(nv_pad, nv_pad), bounds_check=True)

    # Each tile processes one output tile by looping over all constraints
    for k in range(0, njmax, TILE_SIZE_K):
      if k >= nefc:
        break

      # AD: leaving bounds-check disabled here because I'm not entirely sure that
      # everything always hits the fast path. The padding takes care of any
      #  potential OOB accesses.
      J_kj = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE_K, nv_pad), offset=(k, 0), bounds_check=False)

      # state check
      D_k = wp.tile_load(efc_D_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
      state = wp.tile_load(efc_state_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)

      D_k = wp.tile_map(_state_check, D_k, state)

      # force unused elements to be zero
      tid_tile = wp.tile_arange(TILE_SIZE_K, dtype=int)
      threshold_tile = wp.tile_ones(shape=TILE_SIZE_K, dtype=int) * (nefc - k)

      active_tile = wp.tile_map(_active_check, tid_tile, threshold_tile)
      D_k = wp.tile_map(wp.mul, active_tile, D_k)

      J_ki = wp.tile_map(wp.mul, wp.tile_transpose(J_kj), wp.tile_broadcast(D_k, shape=(nv_pad, TILE_SIZE_K)))

      sum_val += wp.tile_matmul(J_ki, J_kj)

    wp.tile_store(ctx_h_out[worldid], sum_val, bounds_check=False)

  return kernel


# TODO(thowell): combine with JTDAJ ?
@wp.kernel
def _update_gradient_JTCJ_sparse(
  # Model:
  opt_impratio_invsqrt: wp.array[float],
  # Data in:
  contact_dist_in: wp.array[float],
  contact_includemargin_in: wp.array[float],
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  contact_worldid_in: wp.array[int],
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  naconmax_in: int,
  nacon_in: wp.array[int],
  # In:
  ctx_Jaref_in: wp.array2d[float],
  ctx_done_in: wp.array[bool],
  nblocks_perblock: int,
  dim_block: int,
  # Out:
  ctx_h_out: wp.array3d[float],
):
  conid_start, pairid = wp.tid()

  for i in range(nblocks_perblock):
    conid = conid_start + i * dim_block

    if conid >= min(nacon_in[0], naconmax_in):
      return

    worldid = contact_worldid_in[conid]
    if ctx_done_in[worldid]:
      continue

    condim = contact_dim_in[conid]

    if condim == 1:
      continue

    # check contact status
    if contact_dist_in[conid] - contact_includemargin_in[conid] >= 0.0:
      continue

    efcid0 = contact_efc_address_in[conid, 0]
    if efc_state_in[worldid, efcid0] != types.ConstraintState.CONE:
      continue

    # One thread per (contact, support-pair): the support dofs are exactly the colind entries,
    # so decode pairid -> (pos1, pos2) with pos1 <= pos2 directly. No colind scan, and no
    # membership skip (which the all-dof-pairs version wasted on ~99% absent dofs).
    rownnz = efc_J_rownnz_in[worldid, efcid0]
    npairs = rownnz * (rownnz + 1) // 2
    if pairid >= npairs:
      continue
    rowadr0 = efc_J_rowadr_in[worldid, efcid0]
    pos1 = int(0)
    rem = pairid
    while rem >= rownnz - pos1:
      rem -= rownnz - pos1
      pos1 += 1
    pos2 = pos1 + rem
    dofa = efc_J_colind_in[worldid, 0, rowadr0 + pos1]
    dofb = efc_J_colind_in[worldid, 0, rowadr0 + pos2]
    dof1id = wp.min(dofa, dofb)
    dof2id = wp.max(dofa, dofb)

    fri = contact_friction_in[conid]
    mu = fri[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

    mu2 = mu * mu
    dm = math.safe_div(efc_D_in[worldid, efcid0], mu2 * (1.0 + mu2))

    if dm == 0.0:
      continue

    n = ctx_Jaref_in[worldid, efcid0] * mu
    u = types.vec6(n, 0.0, 0.0, 0.0, 0.0, 0.0)

    tt = float(0.0)
    for j in range(1, condim):
      efcidj = contact_efc_address_in[conid, j]
      uj = ctx_Jaref_in[worldid, efcidj] * fri[j - 1]
      tt += uj * uj
      u[j] = uj

    if tt <= 0.0:
      t = 0.0
    else:
      t = wp.sqrt(tt)
    t = wp.max(t, types.MJ_MINVAL)
    ttt = wp.max(t * t * t, types.MJ_MINVAL)

    # Precompute common subexpressions.
    mu_over_t = math.safe_div(mu, t)
    mu_n_over_ttt = mu * math.safe_div(n, ttt)
    mu2_minus_mu_n_over_t = mu2 - mu * math.safe_div(n, t)

    h = float(0.0)

    for dim1id in range(condim):
      if dim1id == 0:
        rowadr1 = rowadr0
        dm_fri1 = dm * mu
      else:
        efcid1 = contact_efc_address_in[conid, dim1id]
        rowadr1 = efc_J_rowadr_in[worldid, efcid1]
        dm_fri1 = dm * fri[dim1id - 1]

      # Direct J reads using cached sparse positions.
      efc_J11 = efc_J_in[worldid, 0, rowadr1 + pos1]
      efc_J12 = efc_J_in[worldid, 0, rowadr1 + pos2]

      ui = u[dim1id]

      for dim2id in range(0, dim1id + 1):
        if dim2id == 0:
          rowadr2 = rowadr0
          dm_fri12 = dm_fri1 * mu
        else:
          efcid2 = contact_efc_address_in[conid, dim2id]
          rowadr2 = efc_J_rowadr_in[worldid, efcid2]
          dm_fri12 = dm_fri1 * fri[dim2id - 1]

        # Direct J reads using cached sparse positions.
        efc_J21 = efc_J_in[worldid, 0, rowadr2 + pos1]
        efc_J22 = efc_J_in[worldid, 0, rowadr2 + pos2]

        uj = u[dim2id]

        # set first row/column: (1, -mu/t * u)
        if dim1id == 0 and dim2id == 0:
          hcone = 1.0
        elif dim1id == 0:
          hcone = -mu_over_t * uj
        elif dim2id == 0:
          hcone = -mu_over_t * ui
        else:
          hcone = mu_n_over_ttt * ui * uj

          # add to diagonal: mu^2 - mu * n / t
          if dim1id == dim2id:
            hcone += mu2_minus_mu_n_over_t

        hcone *= dm_fri12

        if hcone != 0.0:
          h += hcone * efc_J11 * efc_J22

          if dim1id != dim2id:
            h += hcone * efc_J12 * efc_J21

    # multiple contacts can contribute to the same (dof1id, dof2id); atomic_add is exact
    wp.atomic_add(ctx_h_out[worldid, dof1id], dof2id, h)


@wp.kernel
def _update_gradient_JTCJ_dense(
  # Model:
  opt_impratio_invsqrt: wp.array[float],
  dof_tri_row: wp.array[int],
  dof_tri_col: wp.array[int],
  # Data in:
  contact_dist_in: wp.array[float],
  contact_includemargin_in: wp.array[float],
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  contact_worldid_in: wp.array[int],
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  naconmax_in: int,
  nacon_in: wp.array[int],
  # In:
  ctx_Jaref_in: wp.array2d[float],
  ctx_done_in: wp.array[bool],
  nblocks_perblock: int,
  dim_block: int,
  # Out:
  ctx_h_out: wp.array3d[float],
):
  conid_start, elementid = wp.tid()

  dof1id = dof_tri_row[elementid]
  dof2id = dof_tri_col[elementid]

  for i in range(nblocks_perblock):
    conid = conid_start + i * dim_block

    if conid >= min(nacon_in[0], naconmax_in):
      return

    worldid = contact_worldid_in[conid]
    if ctx_done_in[worldid]:
      continue

    condim = contact_dim_in[conid]

    if condim == 1:
      continue

    # check contact status
    if contact_dist_in[conid] - contact_includemargin_in[conid] >= 0.0:
      continue

    efcid0 = contact_efc_address_in[conid, 0]
    if efc_state_in[worldid, efcid0] != types.ConstraintState.CONE:
      continue

    fri = contact_friction_in[conid]
    mu = fri[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

    mu2 = mu * mu
    dm = math.safe_div(efc_D_in[worldid, efcid0], mu2 * (1.0 + mu2))

    if dm == 0.0:
      continue

    n = ctx_Jaref_in[worldid, efcid0] * mu
    u = types.vec6(n, 0.0, 0.0, 0.0, 0.0, 0.0)

    tt = float(0.0)
    for j in range(1, condim):
      efcidj = contact_efc_address_in[conid, j]
      uj = ctx_Jaref_in[worldid, efcidj] * fri[j - 1]
      tt += uj * uj
      u[j] = uj

    if tt <= 0.0:
      t = 0.0
    else:
      t = wp.sqrt(tt)
    t = wp.max(t, types.MJ_MINVAL)
    ttt = wp.max(t * t * t, types.MJ_MINVAL)

    h = float(0.0)

    for dim1id in range(condim):
      if dim1id == 0:
        efcid1 = efcid0
      else:
        efcid1 = contact_efc_address_in[conid, dim1id]

      efc_J11 = efc_J_in[worldid, efcid1, dof1id]
      efc_J12 = efc_J_in[worldid, efcid1, dof2id]

      ui = u[dim1id]

      for dim2id in range(0, dim1id + 1):
        if dim2id == 0:
          efcid2 = efcid0
        else:
          efcid2 = contact_efc_address_in[conid, dim2id]

        efc_J21 = efc_J_in[worldid, efcid2, dof1id]
        efc_J22 = efc_J_in[worldid, efcid2, dof2id]

        uj = u[dim2id]

        # set first row/column: (1, -mu/t * u)
        if dim1id == 0 and dim2id == 0:
          hcone = 1.0
        elif dim1id == 0:
          hcone = -math.safe_div(mu, t) * uj
        elif dim2id == 0:
          hcone = -math.safe_div(mu, t) * ui
        else:
          hcone = mu * math.safe_div(n, ttt) * ui * uj

          # add to diagonal: mu^2 - mu * n / t
          if dim1id == dim2id:
            hcone += mu2 - mu * math.safe_div(n, t)

        # pre and post multiply by diag(mu, friction) scale by dm
        if dim1id == 0:
          fri1 = mu
        else:
          fri1 = fri[dim1id - 1]

        if dim2id == 0:
          fri2 = mu
        else:
          fri2 = fri[dim2id - 1]

        hcone *= dm * fri1 * fri2

        if hcone != 0.0:
          h += hcone * efc_J11 * efc_J22

          if dim1id != dim2id:
            h += hcone * efc_J12 * efc_J21

    ctx_h_out[worldid, dof1id, dof2id] += h


@cache_kernel
def _update_gradient_cholesky(tile_size: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # In:
    ctx_grad_in: wp.array2d[float],
    h_in: wp.array3d[float],
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_Mgrad_out: wp.array2d[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if ctx_done_in[worldid]:
      return

    mat_tile = wp.tile_load(h_in[worldid], shape=(TILE_SIZE, TILE_SIZE))
    fact_tile = wp.tile_cholesky(mat_tile, fill_mode="upper")
    input_tile = wp.tile_load(ctx_grad_in[worldid], shape=TILE_SIZE)
    output_tile = wp.tile_cholesky_solve(fact_tile, input_tile, fill_mode="upper")
    wp.tile_store(ctx_Mgrad_out[worldid], output_tile)

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked(tile_size: int, matrix_size: int):
  @wp.kernel(module="unique", enable_backward=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array3d[float],
    ctx_h_in: wp.array3d[float],
    ctx_hfactor: wp.array3d[float],
    # Out:
    ctx_Mgrad_out: wp.array3d[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if ctx_done_in[worldid]:
      return

    # We need matrix size both as a runtime input as well as a static input:
    # static input is needed to specify the tile sizes for the compiler
    # runtime input is needed for the loop bounds, otherwise warp will unroll
    # unconditionally leading to shared memory capacity issues.

    wp.static(create_blocked_cholesky_factorize_solve_func(TILE_SIZE, matrix_size))(
      ctx_h_in[worldid], ctx_grad_in[worldid], matrix_size, ctx_hfactor[worldid], ctx_Mgrad_out[worldid]
    )

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked_skip_unchanged(tile_size: int, matrix_size: int):
  """Blocked Cholesky that skips factorization when no constraints changed."""

  @wp.kernel(module="unique", enable_backward=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array3d[float],
    ctx_h_in: wp.array3d[float],
    changed_count_in: wp.array[int],
    ctx_hfactor: wp.array3d[float],
    # Out:
    ctx_Mgrad_out: wp.array3d[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if ctx_done_in[worldid]:
      return

    if changed_count_in[worldid] > 0:
      wp.static(create_blocked_cholesky_factorize_solve_func(TILE_SIZE, matrix_size))(
        ctx_h_in[worldid], ctx_grad_in[worldid], matrix_size, ctx_hfactor[worldid], ctx_Mgrad_out[worldid]
      )
    else:
      wp.static(create_blocked_cholesky_solve_func(TILE_SIZE, matrix_size))(
        ctx_hfactor[worldid], ctx_grad_in[worldid], matrix_size, ctx_Mgrad_out[worldid]
      )

  return kernel


@wp.kernel
def _padding_h(nv: int, ctx_done_in: wp.array[bool], ctx_h_out: wp.array3d[float]):
  worldid, elementid = wp.tid()

  if ctx_done_in[worldid]:
    return

  dofid = nv + elementid
  ctx_h_out[worldid, dofid, dofid] = 1.0


def _cholesky_factorize_solve(m: types.Model, d: types.Data, ctx: SolverContext, skip_unchanged: bool = False):
  """Cholesky factorize ctx.h and solve for Mgrad.

  If skip_unchanged is True (blocked path only), worlds where no constraints
  changed reuse the cached factorization in hfactor instead of refactorizing.
  """
  if m.nv <= _BLOCK_CHOLESKY_DIM:
    wp.launch_tiled(
      _update_gradient_cholesky(m.nv),
      dim=d.nworld,
      inputs=[ctx.grad, ctx.h, ctx.done],
      outputs=[ctx.Mgrad],
      block_dim=m.block_dim.update_gradient_cholesky,
    )
  else:
    wp.launch(
      _padding_h,
      dim=(d.nworld, m.nv_pad - m.nv),
      inputs=[m.nv, ctx.done],
      outputs=[ctx.h],
    )

    if skip_unchanged:
      wp.launch_tiled(
        _update_gradient_cholesky_blocked_skip_unchanged(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad),
        dim=d.nworld,
        inputs=[ctx.done, ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)), ctx.h, ctx.changed_efc_count, ctx.hfactor],
        outputs=[ctx.Mgrad.reshape(shape=(d.nworld, ctx.Mgrad.shape[1], 1))],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )
    else:
      wp.launch_tiled(
        _update_gradient_cholesky_blocked(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad),
        dim=d.nworld,
        inputs=[ctx.done, ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)), ctx.h, ctx.hfactor],
        outputs=[ctx.Mgrad.reshape(shape=(d.nworld, ctx.Mgrad.shape[1], 1))],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )


@wp.kernel
def _JTDAJ_sparse(
  # Data in:
  nefc_in: wp.array[int],
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  h_out: wp.array3d[float],
):
  worldid, efcid = wp.tid()

  if ctx_done_in[worldid]:
    return

  if efcid >= nefc_in[worldid]:
    return

  efc_D = efc_D_in[worldid, efcid]
  efc_state = efc_state_in[worldid, efcid]

  if _state_check(efc_D, efc_state) == 0.0:
    return

  rownnz = efc_J_rownnz_in[worldid, efcid]
  rowadr = efc_J_rowadr_in[worldid, efcid]

  for i in range(rownnz):
    sparseidi = rowadr + i
    Ji = efc_J_in[worldid, 0, sparseidi]
    colindi = efc_J_colind_in[worldid, 0, sparseidi]
    for j in range(i, rownnz):
      if j == i:
        sparseidj = sparseidi
        Jj = Ji
        colindj = colindi
      else:
        sparseidj = rowadr + j
        Jj = efc_J_in[worldid, 0, sparseidj]
        colindj = efc_J_colind_in[worldid, 0, sparseidj]

      h = Ji * Jj * efc_D
      # Store in upper triangle only: ensure row <= col.
      row = wp.min(colindi, colindj)
      col = wp.max(colindi, colindj)
      wp.atomic_add(h_out[worldid, row], col, h)


def _update_gradient(m: types.Model, d: types.Data, ctx: SolverContext):
  # grad = Ma - qfrc_smooth - qfrc_constraint
  if m.opt.solver == types.SolverType.CG:
    wp.launch_tiled(
      _update_gradient_grad_tiled,
      dim=d.nworld,
      inputs=[m.nv, d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, ctx.done],
      outputs=[ctx.grad, ctx.grad_dot],
      block_dim=m.block_dim.update_gradient_grad,
    )
  else:
    wp.launch(_update_gradient_zero_grad_dot, dim=d.nworld, inputs=[ctx.done], outputs=[ctx.grad_dot])
    wp.launch(
      _update_gradient_grad,
      dim=(d.nworld, m.nv),
      inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, ctx.done],
      outputs=[ctx.grad, ctx.grad_dot],
    )

  if m.opt.solver == types.SolverType.CG:
    smooth.solve_m(m, d, ctx.Mgrad, ctx.grad)
  elif m.opt.solver == types.SolverType.NEWTON:
    # h = M + (efc_J.T * efc_D * active) @ efc_J
    if m.is_sparse:
      wp.launch(
        _update_gradient_init_h_sparse,
        dim=(d.nworld, m.nv_pad, m.nv_pad),
        inputs=[m.nv, m.M_elemid, d.M, ctx.done],
        outputs=[ctx.h],
      )

      wp.launch(
        _JTDAJ_sparse,
        dim=(d.nworld, d.njmax),
        inputs=[d.nefc, d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J, d.efc.D, d.efc.state, ctx.done],
        outputs=[ctx.h],
      )
    else:
      wp.launch_tiled(
        _update_gradient_JTDAJ_dense_tiled(m.nv_pad, types.TILE_SIZE_JTDAJ_DENSE, d.njmax),
        dim=d.nworld,
        inputs=[
          d.nefc,
          d.M,
          d.efc.J,
          d.efc.D,
          d.efc.state,
          ctx.done,
        ],
        outputs=[ctx.h],
        block_dim=m.block_dim.update_gradient_JTDAJ_dense,
      )

    if m.opt.cone == types.ConeType.ELLIPTIC:
      # Optimization: launching update_gradient_JTCJ with limited number of blocks on a GPU.
      # Profiling suggests that only a fraction of blocks out of the original
      # d.njmax blocks do the actual work. It aims to minimize #CTAs with no
      # effective work. It launches with #blocks that's proportional to the number
      # of SMs on the GPU. We can now query the SM count:
      # https://github.com/NVIDIA/warp/commit/f3814e7e5459e5fd13032cf0fddb3daddd510f30

      # Block-limit the launch: cap the grid near SM-filling width and stride over contacts, so
      # we don't over-launch naconmax (capacity) threads when active contacts are far fewer. The
      # sparse kernel uses one thread per (contact, support-pair) (jtcj_max_pairs), the dense one
      # per (contact, dof-pair) (dof_tri_row.size).
      jtcj_second_dim = m.jtcj_max_pairs if m.is_sparse else m.dof_tri_row.size
      if wp.get_device().is_cuda:
        sm_count = wp.get_device().sm_count

        # Here we assume one block has 256 threads. We use a factor of 6, which
        # can be changed in the future to fine-tune the perf. The optimal factor will
        # depend on the kernel's occupancy, which determines how many blocks can
        # simultaneously run on the SM. TODO: This factor can be tuned further.
        dim_block = ceil((sm_count * 6 * 256) / jtcj_second_dim)
      else:
        # fall back for CPU
        dim_block = d.naconmax

      nblocks_perblock = int((d.naconmax + dim_block - 1) / dim_block)

      if m.is_sparse:
        wp.launch(
          _update_gradient_JTCJ_sparse,
          dim=(dim_block, m.jtcj_max_pairs),
          inputs=[
            m.opt.impratio_invsqrt,
            d.contact.dist,
            d.contact.includemargin,
            d.contact.friction,
            d.contact.dim,
            d.contact.efc_address,
            d.contact.worldid,
            d.efc.J_rownnz,
            d.efc.J_rowadr,
            d.efc.J_colind,
            d.efc.J,
            d.efc.D,
            d.efc.state,
            d.naconmax,
            d.nacon,
            ctx.Jaref,
            ctx.done,
            nblocks_perblock,
            dim_block,
          ],
          outputs=[ctx.h],
        )
      else:
        wp.launch(
          _update_gradient_JTCJ_dense,
          dim=(dim_block, m.dof_tri_row.size),
          inputs=[
            m.opt.impratio_invsqrt,
            m.dof_tri_row,
            m.dof_tri_col,
            d.contact.dist,
            d.contact.includemargin,
            d.contact.friction,
            d.contact.dim,
            d.contact.efc_address,
            d.contact.worldid,
            d.efc.J,
            d.efc.D,
            d.efc.state,
            d.naconmax,
            d.nacon,
            ctx.Jaref,
            ctx.done,
            nblocks_perblock,
            dim_block,
          ],
          outputs=[ctx.h],
        )

    _cholesky_factorize_solve(m, d, ctx)
  else:
    raise ValueError(f"Unknown solver type: {m.opt.solver}")


def _update_gradient_incremental(m: types.Model, d: types.Data, ctx: SolverContext):
  """Incremental gradient update: update H for changed constraints + re-factorize.

  Skips the full J^T*D*J rebuild by applying only the delta from constraints
  that changed QUADRATIC state, then re-factorizes and solves.
  """
  wp.launch(_update_gradient_zero_grad_dot, dim=d.nworld, inputs=[ctx.done], outputs=[ctx.grad_dot])

  wp.launch(
    _update_gradient_grad,
    dim=(d.nworld, m.nv),
    inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, ctx.done],
    outputs=[ctx.grad, ctx.grad_dot],
  )

  # Update upper triangle of H with delta from changed constraints.
  if m.is_sparse:
    wp.launch(
      _update_gradient_h_incremental_sparse,
      dim=(d.nworld, ctx.changed_efc_ids.shape[1]),
      inputs=[
        d.efc.J_rownnz,
        d.efc.J_rowadr,
        d.efc.J_colind,
        d.efc.J,
        d.efc.D,
        d.efc.state,
        ctx.changed_efc_ids,
        ctx.changed_efc_count,
      ],
      outputs=[ctx.h],
    )
  else:
    tri_dim = m.nv * (m.nv + 1) // 2
    wp.launch(
      _update_gradient_h_incremental,
      dim=(d.nworld, tri_dim),
      inputs=[
        d.efc.J,
        d.efc.D,
        d.efc.state,
        ctx.changed_efc_ids,
        ctx.changed_efc_count,
      ],
      outputs=[ctx.h],
    )

  _cholesky_factorize_solve(m, d, ctx, skip_unchanged=True)


@wp.kernel
def _solve_beta_zero(
  # Out:
  ctx_beta_num_out: wp.array[float],
  ctx_beta_den_out: wp.array[float],
):
  worldid = wp.tid()
  ctx_beta_num_out[worldid] = 0.0
  ctx_beta_den_out[worldid] = 0.0


@wp.kernel
def _solve_beta_accumulate_tiled(
  # Model:
  nv: int,
  # In:
  ctx_grad_in: wp.array2d[float],
  ctx_Mgrad_in: wp.array2d[float],
  ctx_prev_grad_in: wp.array2d[float],
  ctx_prev_Mgrad_in: wp.array2d[float],
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_beta_num_out: wp.array[float],
  ctx_beta_den_out: wp.array[float],
):
  worldid, tid = wp.tid()

  if ctx_done_in[worldid]:
    return

  local_num = float(0.0)
  local_den = float(0.0)
  BLOCK_DIM = wp.block_dim()

  for dofid in range(tid, nv, BLOCK_DIM):
    prev_Mgrad = ctx_prev_Mgrad_in[worldid, dofid]
    num = ctx_grad_in[worldid, dofid] * (ctx_Mgrad_in[worldid, dofid] - prev_Mgrad)
    den = ctx_prev_grad_in[worldid, dofid] * prev_Mgrad
    local_num += num
    local_den += den

  num_tile = wp.tile(local_num, preserve_type=True)
  num_sum = wp.tile_reduce(wp.add, num_tile)

  den_tile = wp.tile(local_den, preserve_type=True)
  den_sum = wp.tile_reduce(wp.add, den_tile)

  if tid == 0:
    ctx_beta_num_out[worldid] = num_sum[0]
    ctx_beta_den_out[worldid] = den_sum[0]


@wp.kernel
def _solve_beta_accumulate(
  # In:
  ctx_grad_in: wp.array2d[float],
  ctx_Mgrad_in: wp.array2d[float],
  ctx_prev_grad_in: wp.array2d[float],
  ctx_prev_Mgrad_in: wp.array2d[float],
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_beta_num_out: wp.array[float],
  ctx_beta_den_out: wp.array[float],
):
  worldid, dofid = wp.tid()

  if ctx_done_in[worldid]:
    return

  prev_Mgrad = ctx_prev_Mgrad_in[worldid, dofid]
  num = ctx_grad_in[worldid, dofid] * (ctx_Mgrad_in[worldid, dofid] - prev_Mgrad)
  den = ctx_prev_grad_in[worldid, dofid] * prev_Mgrad
  wp.atomic_add(ctx_beta_num_out, worldid, num)
  wp.atomic_add(ctx_beta_den_out, worldid, den)


@wp.kernel
def _solve_zero_search_dot(
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_search_dot_out: wp.array[float],
):
  worldid = wp.tid()

  if ctx_done_in[worldid]:
    return

  ctx_search_dot_out[worldid] = 0.0


@wp.kernel
def _solve_search_update(
  # Model:
  opt_solver: int,
  # In:
  ctx_Mgrad_in: wp.array2d[float],
  ctx_search_in: wp.array2d[float],
  ctx_beta_in: wp.array[float],
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_search_out: wp.array2d[float],
  ctx_search_dot_out: wp.array[float],
):
  worldid, dofid = wp.tid()

  if ctx_done_in[worldid]:
    return

  search = -1.0 * ctx_Mgrad_in[worldid, dofid]

  if opt_solver == types.SolverType.CG:
    search += ctx_beta_in[worldid] * ctx_search_in[worldid, dofid]

  ctx_search_out[worldid, dofid] = search
  wp.atomic_add(ctx_search_dot_out, worldid, search * search)


@wp.kernel
def _solve_search_update_cg_tiled(
  # Model:
  nv: int,
  # In:
  ctx_grad_in: wp.array2d[float],
  ctx_Mgrad_in: wp.array2d[float],
  ctx_search_in: wp.array2d[float],
  ctx_beta_in: wp.array[float],
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_search_out: wp.array2d[float],
  ctx_search_dot_out: wp.array[float],
  ctx_prev_grad_out: wp.array2d[float],
  ctx_prev_Mgrad_out: wp.array2d[float],
):
  worldid, tid = wp.tid()

  if ctx_done_in[worldid]:
    return

  local_search_dot = float(0.0)
  BLOCK_DIM = wp.block_dim()
  beta = ctx_beta_in[worldid]

  for dofid in range(tid, nv, BLOCK_DIM):
    mgrad = ctx_Mgrad_in[worldid, dofid]
    search = -1.0 * mgrad + beta * ctx_search_in[worldid, dofid]

    ctx_search_out[worldid, dofid] = search
    local_search_dot += search * search

    ctx_prev_grad_out[worldid, dofid] = ctx_grad_in[worldid, dofid]
    ctx_prev_Mgrad_out[worldid, dofid] = mgrad

  search_dot_tile = wp.tile(local_search_dot, preserve_type=True)
  search_dot_sum = wp.tile_reduce(wp.add, search_dot_tile)

  if tid == 0:
    ctx_search_dot_out[worldid] = search_dot_sum[0]


@wp.kernel
def _solve_cg_finalize(
  # Model:
  nv: int,
  opt_tolerance: wp.array[float],
  opt_iterations: int,
  stat_meaninertia: wp.array[float],
  # In:
  ctx_beta_num_in: wp.array[float],
  ctx_beta_den_in: wp.array[float],
  ctx_improvement_in: wp.array[float],
  ctx_done_in: wp.array[bool],
  ctx_grad_dot_in: wp.array[float],
  # Data out:
  solver_niter_out: wp.array[int],
  # Out:
  ctx_beta_out: wp.array[float],
  nsolving_out: wp.array[int],
  ctx_done_out: wp.array[bool],
):
  worldid = wp.tid()

  if ctx_done_in[worldid]:
    return

  # 1. solve_beta_finalize
  ctx_beta_out[worldid] = wp.max(0.0, ctx_beta_num_in[worldid] / wp.max(types.MJ_MINVAL, ctx_beta_den_in[worldid]))

  # 2. solve_done
  solver_niter_out[worldid] += 1
  tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
  meaninertia = stat_meaninertia[worldid % stat_meaninertia.shape[0]]

  grad_dot = ctx_grad_dot_in[worldid]

  improvement = _rescale(nv, meaninertia, ctx_improvement_in[worldid])
  gradient = _rescale(nv, meaninertia, wp.sqrt(grad_dot))
  done = (improvement < tolerance) or (gradient < tolerance)
  if done or solver_niter_out[worldid] == opt_iterations:
    ctx_done_out[worldid] = True
    wp.atomic_add(nsolving_out, 0, -1)


@wp.kernel
def _solve_done(
  # Model:
  nv: int,
  opt_tolerance: wp.array[float],
  opt_iterations: int,
  stat_meaninertia: wp.array[float],
  # In:
  ctx_grad_dot_in: wp.array[float],
  ctx_improvement_in: wp.array[float],
  ctx_done_in: wp.array[bool],
  # Data out:
  solver_niter_out: wp.array[int],
  # Out:
  nsolving_out: wp.array[int],
  ctx_done_out: wp.array[bool],
):
  worldid = wp.tid()

  if ctx_done_in[worldid]:
    return

  solver_niter_out[worldid] += 1
  tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
  meaninertia = stat_meaninertia[worldid % stat_meaninertia.shape[0]]

  improvement = _rescale(nv, meaninertia, ctx_improvement_in[worldid])
  gradient = _rescale(nv, meaninertia, wp.sqrt(ctx_grad_dot_in[worldid]))
  done = (improvement < tolerance) or (gradient < tolerance)
  if done or solver_niter_out[worldid] == opt_iterations:
    # if the solver has converged or the maximum number of iterations has been reached then
    # mark this world as done and remove it from the number of unconverged worlds
    ctx_done_out[worldid] = True
    wp.atomic_add(nsolving_out, 0, -1)


@event_scope
def _solver_iteration(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext,
  nsolving: wp.array[int],
):
  _linesearch(m, d, ctx)

  # Incremental H is only valid for non-elliptic cones. The elliptic cone
  # path in _update_constraint_efc has early returns that skip state change
  # tracking, and the additional JTCJ Hessian term depends on Jaref which
  # changes every iteration.
  incremental = m.opt.solver == types.SolverType.NEWTON and m.opt.cone != types.ConeType.ELLIPTIC

  if incremental:
    # Must complete before _update_constraint_efc which atomically increments.
    ctx.changed_efc_count.zero_()

  _update_constraint(m, d, ctx, track_changes=incremental)

  if incremental:
    _update_gradient_incremental(m, d, ctx)
  else:
    _update_gradient(m, d, ctx)

  # polak-ribiere
  if m.opt.solver == types.SolverType.CG:
    wp.launch(
      _solve_beta_zero,
      dim=d.nworld,
      outputs=[ctx.beta, ctx.beta_den],
    )
    wp.launch_tiled(
      _solve_beta_accumulate_tiled,
      dim=d.nworld,
      inputs=[m.nv, ctx.grad, ctx.Mgrad, ctx.prev_grad, ctx.prev_Mgrad, ctx.done],
      outputs=[ctx.beta, ctx.beta_den],
      block_dim=m.block_dim.solve_beta_accumulate,
    )
    wp.launch(
      _solve_cg_finalize,
      dim=d.nworld,
      inputs=[
        m.nv,
        m.opt.tolerance,
        m.opt.iterations,
        m.stat.meaninertia,
        ctx.beta,
        ctx.beta_den,
        ctx.improvement,
        ctx.done,
        ctx.grad_dot,
      ],
      outputs=[
        d.solver_niter,
        ctx.beta,
        nsolving,
        ctx.done,
      ],
    )
    wp.launch_tiled(
      _solve_search_update_cg_tiled,
      dim=d.nworld,
      inputs=[m.nv, ctx.grad, ctx.Mgrad, ctx.search, ctx.beta, ctx.done],
      outputs=[ctx.search, ctx.search_dot, ctx.prev_grad, ctx.prev_Mgrad],
      block_dim=m.block_dim.solve_search_update_cg,
    )

  else:
    wp.launch(_solve_zero_search_dot, dim=d.nworld, inputs=[ctx.done], outputs=[ctx.search_dot])

    wp.launch(
      _solve_search_update,
      dim=(d.nworld, m.nv),
      inputs=[m.opt.solver, ctx.Mgrad, ctx.search, ctx.beta, ctx.done],
      outputs=[ctx.search, ctx.search_dot],
    )

    wp.launch(
      _solve_done,
      dim=d.nworld,
      inputs=[
        m.nv,
        m.opt.tolerance,
        m.opt.iterations,
        m.stat.meaninertia,
        ctx.grad_dot,
        ctx.improvement,
        ctx.done,
      ],
      outputs=[d.solver_niter, nsolving, ctx.done],
    )


def init_context(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, grad: bool = True):
  # initialize some efc arrays
  wp.launch(
    _solve_init_efc,
    dim=d.nworld,
    outputs=[d.solver_niter, ctx.search_dot, ctx.done],
  )

  # jaref = d.efc_J @ d.qacc - d.efc_aref

  # if we are only using 1 thread, it makes sense to do more dofs as we can also skip the
  # init kernel. For more than 1 thread, dofs_per_thread is lower for better load balancing.

  if m.nv > 50:
    dofs_per_thread = 20
  else:
    dofs_per_thread = 50

  threads_per_efc = ceil(m.nv / dofs_per_thread)
  # we need to clear the jaref array if we're doing atomic adds.
  if threads_per_efc > 1:
    ctx.Jaref.zero_()

  wp.launch(
    _solve_init_jaref_kernel(m.is_sparse, m.nv, dofs_per_thread),
    dim=(d.nworld, d.njmax, threads_per_efc),
    inputs=[d.nefc, d.qacc, d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J, d.efc.aref],
    outputs=[ctx.Jaref],
  )

  # Ma = M @ qacc
  support.mul_m(m, d, d.efc.Ma, d.qacc, skip=ctx.done)

  _update_constraint(m, d, ctx)

  if grad:
    _update_gradient(m, d, ctx)


@event_scope
def solve(m: types.Model, d: types.Data):
  if d.njmax == 0 or m.nv == 0:
    wp.copy(d.qacc, d.qacc_smooth)
    d.solver_niter.fill_(0)
  else:
    if m.ntree > 1 and not (m.opt.disableflags & types.DisableBit.ISLAND):
      ctx = _create_island_solver_context(m, d)
      island.compute_island_mapping(m, d, ctx)
      island.gather_island_inputs(m, d, ctx)
      _solve_island(m, d, ctx)
      # Ma is needed by Euler/implicit integrators for implicit damping
      scatter_Ma = m.opt.integrator != types.IntegratorType.RK4
      island.scatter_island_results(m, d, ctx, scatter_Ma=scatter_Ma)
    else:
      ctx = _create_solver_context(m, d)
      _solve(m, d, ctx)


def _solve(m: types.Model, d: types.Data, ctx: SolverContext):
  """Finds forces that satisfy constraints."""
  if not (m.opt.disableflags & types.DisableBit.WARMSTART):
    wp.copy(d.qacc, d.qacc_warmstart)
  else:
    wp.copy(d.qacc, d.qacc_smooth)

  #  context
  init_context(m, d, ctx, grad=True)

  # search = -Mgrad
  if m.opt.solver == types.SolverType.CG:
    wp.launch_tiled(
      _solve_init_search_cg_tiled,
      dim=d.nworld,
      inputs=[m.nv, ctx.grad, ctx.Mgrad],
      outputs=[ctx.search, ctx.search_dot, ctx.prev_grad, ctx.prev_Mgrad],
      block_dim=m.block_dim.solve_init_search_cg,
    )

  else:
    wp.launch(
      _solve_init_search,
      dim=(d.nworld, m.nv),
      inputs=[ctx.Mgrad],
      outputs=[ctx.search, ctx.search_dot],
    )

  nsolving = wp.full(shape=(1,), value=d.nworld, dtype=int)
  if m.opt.iterations != 0 and m.opt.graph_conditional:
    # Note: the iteration kernel (indicated by while_body) is repeatedly launched
    # as long as condition_iteration is not zero.
    # condition_iteration is a warp array of size 1 and type int, it counts the number
    # of worlds that are not converged, it becomes 0 when all worlds are converged.
    # When the number of iterations reaches m.opt.iterations, solver_niter
    # becomes zero and all worlds are marked as converged to avoid an infinite loop.
    # note: we only launch the iteration kernel if everything is not done
    wp.capture_while(nsolving, while_body=_solver_iteration, m=m, d=d, ctx=ctx, nsolving=nsolving)
  else:
    # This branch is mostly for when JAX is used as it is currently not compatible
    # with CUDA graph conditional.
    # It should be removed when JAX becomes compatible.
    for _ in range(m.opt.iterations):
      _solver_iteration(m, d, ctx, nsolving)


# TODO(team): Consolidate monolithic and island solver code where possible
@event_scope
def _solve_island(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Solve constraints for all islands in parallel.

  All islands are processed simultaneously. Island-local arrays in ctx
  (iacc, iefc_J, iefc_D, etc.) are indexed by idof/iefc, and each thread
  determines its island via idof_islandid/iefc_islandid lookup tables.
  """
  # Initialize iacc from warmstart or smooth
  if not (m.opt.disableflags & types.DisableBit.WARMSTART):
    wp.launch(
      _gather_warmstart_island,
      dim=(d.nworld, m.nv),
      inputs=[d.nidof, d.qacc_warmstart, d.map_idof2dof],
      outputs=[d.iqacc],
    )
  else:
    wp.copy(d.iqacc, d.iqacc_smooth)

  # nsolving tracks how many active islands still have unconverged globally
  nsolving = wp.zeros((1,), dtype=int)

  # Initialize island context
  _init_context_island(m, d, ctx, nsolving)

  # search = -Mgrad
  wp.launch(
    _solve_init_search_island,
    dim=(d.nworld, m.nv),
    inputs=[d.nidof, ctx.Mgrad, d.dof_islandid, ctx.done],
    outputs=[ctx.search, ctx.search_dot],
  )

  if m.opt.iterations != 0 and m.opt.graph_conditional:
    wp.capture_while(
      nsolving,
      while_body=_solver_iteration_island,
      m=m,
      d=d,
      ctx=ctx,
      nsolving=nsolving,
    )
  else:
    for _ in range(m.opt.iterations):
      _solver_iteration_island(m, d, ctx, nsolving)


@wp.kernel
def _gather_warmstart_island(
  # Data in:
  nidof_in: wp.array[int],
  qacc_warmstart_in: wp.array2d[float],
  map_idof2dof_in: wp.array2d[int],
  # Out:
  iacc_out: wp.array2d[float],
):
  """Gather qacc_warmstart into island-local order."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  dof = map_idof2dof_in[worldid, idofid]
  iacc_out[worldid, idofid] = qacc_warmstart_in[worldid, dof]


@cache_kernel
def _solve_init_efc_island(enable_sleep: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    ntree: int,
    # Data in:
    nisland_in: wp.array[int],
    tree_awake_in: wp.array2d[int],
    tree_island_in: wp.array2d[int],
    # Out:
    island_cost_out: wp.array2d[float],
    island_search_dot_out: wp.array2d[float],
    island_done_out: wp.array2d[bool],
    island_solver_niter_out: wp.array2d[int],
    nsolving_out: wp.array[int],
  ):
    """Initialize per-island solver scalars."""
    worldid, islandid = wp.tid()

    if islandid >= nisland_in[worldid]:
      return

    is_asleep_flag = int(0)
    if wp.static(enable_sleep):
      has_awake_tree = int(0)
      for t in range(ntree):
        if tree_island_in[worldid, t] == islandid:
          if tree_awake_in[worldid, t] == 1:
            has_awake_tree = int(1)
            break
      if has_awake_tree == 0:
        is_asleep_flag = int(1)

    island_cost_out[worldid, islandid] = 0.0
    island_search_dot_out[worldid, islandid] = 0.0
    island_done_out[worldid, islandid] = is_asleep_flag == 1
    island_solver_niter_out[worldid, islandid] = 0
    if is_asleep_flag == 0:
      wp.atomic_add(nsolving_out, 0, 1)

  return kernel


@wp.kernel
def _solve_init_jaref_island(
  # Model:
  is_sparse: bool,
  # Data in:
  nefc_in: wp.array[int],
  island_idofadr_in: wp.array2d[int],
  island_nv_in: wp.array2d[int],
  njmax_in: int,
  # In:
  iefc_J_rownnz_in: wp.array2d[int],
  iefc_J_rowadr_in: wp.array2d[int],
  iefc_J_colind_in: wp.array3d[int],
  iefc_J_in: wp.array3d[float],
  iacc_in: wp.array2d[float],
  iefc_aref_in: wp.array2d[float],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  Jaref_out: wp.array2d[float],
):
  """Jaref[iefcid] = iefc_J[iefcid] @ iacc - iefc_aref[iefcid] for all island EFCs."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  islandid = iefc_islandid_in[worldid, iefcid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  acc = float(0.0)
  if is_sparse:
    rownnz = iefc_J_rownnz_in[worldid, iefcid]
    rowadr = iefc_J_rowadr_in[worldid, iefcid]
    for k in range(rownnz):
      adr = rowadr + k
      Ji = iefc_J_in[worldid, 0, adr]
      idof = iefc_J_colind_in[worldid, 0, adr]
      acc += Ji * iacc_in[worldid, idof]
  else:
    idofadr = island_idofadr_in[worldid, islandid]
    inv = island_nv_in[worldid, islandid]
    for i in range(inv):
      idof = idofadr + i
      acc += iefc_J_in[worldid, iefcid, idof] * iacc_in[worldid, idof]

  Jaref_out[worldid, iefcid] = acc - iefc_aref_in[worldid, iefcid]


@wp.kernel
def _solve_init_search_island(
  # Data in:
  nidof_in: wp.array[int],
  # In:
  Mgrad_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  search_out: wp.array2d[float],
  island_search_dot_out: wp.array2d[float],
):
  """Search = -Mgrad for all island DOFs."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  s = -Mgrad_in[worldid, idofid]
  search_out[worldid, idofid] = s
  wp.atomic_add(island_search_dot_out, worldid, islandid, s * s)


# TODO(team): remove after updating island solver done criteria to use delta cost
@wp.kernel
def _update_constraint_init_cost(
  # In:
  cost_in: wp.array[float],
  done_in: wp.array[bool],
  # Out:
  gauss_out: wp.array[float],
  cost_out: wp.array[float],
  prev_cost_out: wp.array[float],
):
  tid = wp.tid()
  if done_in[tid]:
    return

  prev_cost_out[tid] = cost_in[tid]
  cost_out[tid] = 0.0
  gauss_out[tid] = 0.0


@wp.kernel
def _update_constraint_efc_island(
  # Model:
  opt_impratio_invsqrt: wp.array[float],
  # Data in:
  nefc_in: wp.array[int],
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  island_nefc_in: wp.array2d[int],
  island_ne_in: wp.array2d[int],
  island_nf_in: wp.array2d[int],
  island_efcadr_in: wp.array2d[int],
  map_efc2iefc_in: wp.array2d[int],
  njmax_in: int,
  nacon_in: wp.array[int],
  # In:
  iefc_type_in: wp.array2d[int],
  iefc_id_in: wp.array2d[int],
  iefc_D_in: wp.array2d[float],
  iefc_frictionloss_in: wp.array2d[float],
  Jaref_in: wp.array2d[float],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  iefc_force_out: wp.array2d[float],
  iefc_state_out: wp.array2d[int],
  island_cost_out: wp.array2d[float],
):
  """Compute force, state, and cost for each island constraint."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  islandid = iefc_islandid_in[worldid, iefcid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  # Local position within island
  iefcadr = island_efcadr_in[worldid, islandid]
  local_iefcid = iefcid - iefcadr
  ine = island_ne_in[worldid, islandid]
  inf = island_nf_in[worldid, islandid]

  jaref = Jaref_in[worldid, iefcid]
  D = iefc_D_in[worldid, iefcid]

  is_equality = local_iefcid < ine
  is_friction = (not is_equality) and (local_iefcid < ine + inf)
  is_elliptic = iefc_type_in[worldid, iefcid] == types.ConstraintType.CONTACT_ELLIPTIC

  frictionloss = iefc_frictionloss_in[worldid, iefcid] if is_friction else 0.0

  ic0 = int(-1)
  jaref0 = float(0.0)
  D0 = float(0.0)
  mu = float(0.0)
  ufrictionj = float(0.0)
  TT = float(0.0)

  if is_elliptic:
    conid = iefc_id_in[worldid, iefcid]
    if conid >= nacon_in[0]:
      return
    efcid0_global = contact_efc_address_in[conid, 0]
    if efcid0_global < 0:
      return
    ic0 = map_efc2iefc_in[worldid, efcid0_global]

    dim = contact_dim_in[conid]
    friction = contact_friction_in[conid]
    mu = friction[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
    jaref0 = Jaref_in[worldid, ic0]
    D0 = iefc_D_in[worldid, ic0]

    for j in range(1, dim):
      efcidj_global = contact_efc_address_in[conid, j]
      if efcidj_global < 0:
        return
      icj = map_efc2iefc_in[worldid, efcidj_global]
      frictionj = friction[j - 1]
      uj = Jaref_in[worldid, icj] * frictionj
      TT += uj * uj
      if iefcid == icj:
        ufrictionj = uj * frictionj

  res = _eval_constraint(
    is_equality,
    is_friction,
    is_elliptic,
    jaref,
    D,
    frictionloss,
    iefcid,
    ic0,
    jaref0,
    D0,
    mu,
    ufrictionj,
    TT,
  )

  iefc_force_out[worldid, iefcid] = res[0]
  iefc_state_out[worldid, iefcid] = int(res[1])
  cost = res[2]
  if cost != 0.0:
    wp.atomic_add(island_cost_out, worldid, islandid, cost)


@wp.kernel
def _update_constraint_init_qfrc_constraint_dense_island(
  # Data in:
  nefc_in: wp.array[int],
  nidof_in: wp.array[int],
  island_nefc_in: wp.array2d[int],
  island_efcadr_in: wp.array2d[int],
  njmax_in: int,
  # In:
  iefc_J_in: wp.array3d[float],
  iefc_force_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  ifrc_constraint_out: wp.array2d[float],
):
  """ifrc_constraint = iefc_J.T @ iefc_force for all island DOFs."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    ifrc_constraint_out[worldid, idofid] = 0.0
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    ifrc_constraint_out[worldid, idofid] = 0.0
    return
  if island_done_in[worldid, islandid]:
    return

  iefcadr = island_efcadr_in[worldid, islandid]
  inefc = island_nefc_in[worldid, islandid]
  acc = float(0.0)
  for iefcid in range(iefcadr, iefcadr + inefc):
    acc += iefc_J_in[worldid, iefcid, idofid] * iefc_force_in[worldid, iefcid]

  ifrc_constraint_out[worldid, idofid] = acc


@wp.kernel
def _update_constraint_init_qfrc_constraint_sparse_island(
  # Data in:
  nefc_in: wp.array[int],
  njmax_in: int,
  # In:
  iefc_J_rownnz_in: wp.array2d[int],
  iefc_J_rowadr_in: wp.array2d[int],
  iefc_J_colind_in: wp.array3d[int],
  iefc_J_in: wp.array3d[float],
  iefc_force_in: wp.array2d[float],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  ifrc_constraint_out: wp.array2d[float],
):
  """ifrc_constraint += iefc_J.T @ iefc_force for all island EFCs (sparse parallel per EFC)."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  islandid = iefc_islandid_in[worldid, iefcid]
  if islandid < 0:
    return

  rownnz = iefc_J_rownnz_in[worldid, iefcid]
  rowadr = iefc_J_rowadr_in[worldid, iefcid]
  force = iefc_force_in[worldid, iefcid]

  for k in range(rownnz):
    adr = rowadr + k
    Ji = iefc_J_in[worldid, 0, adr]
    idof = iefc_J_colind_in[worldid, 0, adr]
    wp.atomic_add(ifrc_constraint_out, worldid, idof, Ji * force)


@wp.kernel
def _update_constraint_gauss_cost_island(
  # Data in:
  nidof_in: wp.array[int],
  # In:
  iacc_in: wp.array2d[float],
  ifrc_smooth_in: wp.array2d[float],
  iacc_smooth_in: wp.array2d[float],
  iMa_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  island_gauss_out: wp.array2d[float],
  island_cost_out: wp.array2d[float],
):
  """Gauss cost: 0.5 * (Ma - qfrc_smooth).T @ (qacc - qacc_smooth) per island."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  dq = iacc_in[worldid, idofid] - iacc_smooth_in[worldid, idofid]
  df = iMa_in[worldid, idofid] - ifrc_smooth_in[worldid, idofid]
  gauss = 0.5 * df * dq

  wp.atomic_add(island_gauss_out, worldid, islandid, gauss)
  wp.atomic_add(island_cost_out, worldid, islandid, gauss)


@wp.kernel
def _update_gradient_grad_island(
  # Data in:
  nidof_in: wp.array[int],
  # In:
  ifrc_smooth_in: wp.array2d[float],
  ifrc_constraint_in: wp.array2d[float],
  iMa_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  grad_out: wp.array2d[float],
  island_grad_dot_out: wp.array2d[float],
):
  """Grad = Ma - qfrc_smooth - qfrc_constraint, grad_dot per island."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  g = iMa_in[worldid, idofid] - ifrc_smooth_in[worldid, idofid] - ifrc_constraint_in[worldid, idofid]
  grad_out[worldid, idofid] = g
  wp.atomic_add(island_grad_dot_out, worldid, islandid, g * g)


@wp.kernel
def _linesearch_jv_island(
  # Model:
  is_sparse: bool,
  # Data in:
  nefc_in: wp.array[int],
  nidof_in: wp.array[int],
  island_idofadr_in: wp.array2d[int],
  island_nv_in: wp.array2d[int],
  njmax_in: int,
  # In:
  iefc_J_rownnz_in: wp.array2d[int],
  iefc_J_rowadr_in: wp.array2d[int],
  iefc_J_colind_in: wp.array3d[int],
  iefc_J_in: wp.array3d[float],
  search_in: wp.array2d[float],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  jv_out: wp.array2d[float],
):
  """Jv = iefc_J @ search for all island EFCs."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  islandid = iefc_islandid_in[worldid, iefcid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  acc = float(0.0)
  if is_sparse:
    rownnz = iefc_J_rownnz_in[worldid, iefcid]
    rowadr = iefc_J_rowadr_in[worldid, iefcid]
    for k in range(rownnz):
      adr = rowadr + k
      Ji = iefc_J_in[worldid, 0, adr]
      idof = iefc_J_colind_in[worldid, 0, adr]
      acc += Ji * search_in[worldid, idof]
  else:
    idofadr = island_idofadr_in[worldid, islandid]
    inv = island_nv_in[worldid, islandid]
    for i in range(inv):
      idof = idofadr + i
      acc += iefc_J_in[worldid, iefcid, idof] * search_in[worldid, idof]

  jv_out[worldid, iefcid] = acc


@wp.func
def _eval_elliptic_cost_island(
  # Model:
  opt_impratio_invsqrt: float,  # kernel_analyzer: off
  # Data in:
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  map_efc2iefc_in: wp.array2d[int],
  # In:
  alpha: float,
  conid: int,
  iefc_D_in: wp.array2d[float],
  Jaref_in: wp.array2d[float],
  jv_in: wp.array2d[float],
  worldid: int,
) -> wp.vec3:
  dim = contact_dim_in[conid]
  friction = contact_friction_in[conid]
  mu = friction[0] * opt_impratio_invsqrt

  ic0 = map_efc2iefc_in[worldid, contact_efc_address_in[conid, 0]]
  D0 = iefc_D_in[worldid, ic0]
  ja0 = Jaref_in[worldid, ic0]
  jv0 = jv_in[worldid, ic0]

  # Bottom-zone quad for the full contact (scalar quadratic over all rows)
  quad = wp.vec3(0.5 * ja0 * ja0 * D0, jv0 * ja0 * D0, 0.5 * jv0 * jv0 * D0)

  u0 = ja0 * mu
  v0 = jv0 * mu
  uu = float(0.0)
  uv = float(0.0)
  vv = float(0.0)

  for j in range(1, dim):
    icj = map_efc2iefc_in[worldid, contact_efc_address_in[conid, j]]
    jaj = Jaref_in[worldid, icj]
    jvj = jv_in[worldid, icj]
    dj = iefc_D_in[worldid, icj]
    DJj = dj * jaj

    quad += wp.vec3(0.5 * jaj * DJj, jvj * DJj, 0.5 * jvj * dj * jvj)

    frictionj = friction[j - 1]
    uj = jaj * frictionj
    vj = jvj * frictionj
    uu += uj * uj
    uv += uj * vj
    vv += vj * vj

  mu2 = mu * mu
  dm = math.safe_div(D0, mu2 * (1.0 + mu2))

  quad1 = wp.vec3(u0, v0, uu)
  quad2 = wp.vec3(uv, vv, dm)

  return _eval_elliptic(mu, quad, quad1, quad2, alpha)


# TODO(team): refactor _linesearch_kernel_island
@wp.kernel
def _linesearch_kernel_island(
  # Model:
  opt_tolerance: wp.array[float],
  opt_ls_tolerance: wp.array[float],
  opt_ls_iterations: int,
  opt_impratio_invsqrt: wp.array[float],
  stat_meaninertia: wp.array[float],
  # Data in:
  nefc_in: wp.array[int],
  nisland_in: wp.array[int],
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  nidof_in: wp.array[int],
  island_nv_in: wp.array2d[int],
  island_ne_in: wp.array2d[int],
  island_nf_in: wp.array2d[int],
  island_efcadr_in: wp.array2d[int],
  island_nefc_in: wp.array2d[int],
  map_efc2iefc_in: wp.array2d[int],
  njmax_in: int,
  nacon_in: wp.array[int],
  island_idofadr_in: wp.array2d[int],
  # In:
  iefc_type_in: wp.array2d[int],
  iefc_id_in: wp.array2d[int],
  iefc_D_in: wp.array2d[float],
  iefc_frictionloss_in: wp.array2d[float],
  Jaref_in: wp.array2d[float],
  jv_in: wp.array2d[float],
  mv_in: wp.array2d[float],
  search_in: wp.array2d[float],
  ifrc_smooth_in: wp.array2d[float],
  iMa_in: wp.array2d[float],
  island_search_dot_in: wp.array2d[float],
  island_gauss_in: wp.array2d[float],
  island_done_in: wp.array2d[bool],
  # Out:
  island_alpha_out: wp.array2d[float],
):
  """Linesearch per island."""
  worldid, islandid = wp.tid()
  nisland = nisland_in[worldid]
  if islandid >= nisland:
    island_alpha_out[worldid, islandid] = 0.0
    return
  nefc = wp.min(njmax_in, nefc_in[worldid])
  tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
  ls_tolerance = opt_ls_tolerance[worldid % opt_ls_tolerance.shape[0]]
  meaninertia = stat_meaninertia[worldid % stat_meaninertia.shape[0]]
  impratio_invsqrt = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

  if island_done_in[worldid, islandid]:
    island_alpha_out[worldid, islandid] = 0.0
    return

  iefcadr = island_efcadr_in[worldid, islandid]
  ine = island_ne_in[worldid, islandid]
  inf = island_nf_in[worldid, islandid]
  inv = island_nv_in[worldid, islandid]
  idofadr = island_idofadr_in[worldid, islandid]

  # Get island nefc
  isle_nefc_end = iefcadr + island_nefc_in[worldid, islandid]

  # Compute gauss quad: [gauss, s.T @ (Ma - frc_smooth), 0.5 * s.T @ mv]
  quad_gauss_1 = float(0.0)
  quad_gauss_2 = float(0.0)
  for i in range(inv):
    idof = idofadr + i
    s = search_in[worldid, idof]
    quad_gauss_1 += s * (iMa_in[worldid, idof] - ifrc_smooth_in[worldid, idof])
    quad_gauss_2 += 0.5 * s * mv_in[worldid, idof]
  # Costs are evaluated as deltas from alpha=0 to keep float32 precision on large
  # absolute costs, so the constant gauss cost (island_gauss_in) is dropped here.
  quad_gauss = wp.vec3(0.0, quad_gauss_1, quad_gauss_2)

  # gtol
  snorm = wp.sqrt(island_search_dot_in[worldid, islandid])
  scale = meaninertia * float(inv)
  gtol = wp.max(tolerance * ls_tolerance * snorm * scale, 1e-6)

  # p0: cost/grad/hessian at alpha=0
  p0 = wp.vec3(quad_gauss[0], quad_gauss[1], 2.0 * quad_gauss[2])
  for iefcid in range(iefcadr, isle_nefc_end):
    if iefcid >= nefc:
      break
    local_iefcid = iefcid - iefcadr
    D = iefc_D_in[worldid, iefcid]
    ja = Jaref_in[worldid, iefcid]
    jv_val = jv_in[worldid, iefcid]
    if local_iefcid < ine:
      # Equality: always active
      jvD = jv_val * D
      p0 += wp.vec3(0.5 * D * ja * ja, jvD * ja, jv_val * jvD)
    elif local_iefcid < ine + inf:
      # Friction
      f = iefc_frictionloss_in[worldid, iefcid]
      rf = math.safe_div(f, D)
      p0 += _eval_frictionloss_pt(ja, f, rf, jv_val, D)
    elif iefc_type_in[worldid, iefcid] == types.ConstraintType.CONTACT_ELLIPTIC:
      conid = iefc_id_in[worldid, iefcid]
      if conid < nacon_in[0]:
        ic0 = map_efc2iefc_in[worldid, contact_efc_address_in[conid, 0]]
        if iefcid == ic0:
          p0 += _eval_elliptic_cost_island(
            impratio_invsqrt,
            contact_friction_in,
            contact_dim_in,
            contact_efc_address_in,
            map_efc2iefc_in,
            0.0,
            conid,
            iefc_D_in,
            Jaref_in,
            jv_in,
            worldid,
          )
    else:
      # Inequality
      if ja < 0.0:
        jvD = jv_val * D
        p0 += wp.vec3(0.5 * D * ja * ja, jvD * ja, jv_val * jvD)

  # Zero the cost component: alpha=0 is the delta reference (grad/hessian are kept).
  p0 = wp.vec3(0.0, p0[1], p0[2])

  # Newton step: lo_alpha_in = -p0[1] / p0[2]
  lo_alpha_in = -math.safe_div(p0[1], p0[2])

  # Evaluate at Newton step
  lo_in = _eval_pt(quad_gauss, lo_alpha_in)
  for iefcid in range(iefcadr, isle_nefc_end):
    if iefcid >= nefc:
      break
    local_iefcid = iefcid - iefcadr
    D = iefc_D_in[worldid, iefcid]
    ja = Jaref_in[worldid, iefcid]
    jv_val = jv_in[worldid, iefcid]
    if local_iefcid < ine:
      lo_in += _eval_pt_direct_shifted(ja, jv_val, D, lo_alpha_in, 0.0)
    elif local_iefcid < ine + inf:
      f = iefc_frictionloss_in[worldid, iefcid]
      rf = math.safe_div(f, D)
      x_a = ja + lo_alpha_in * jv_val
      lo_in += _shift_cost(_eval_frictionloss_pt(x_a, f, rf, jv_val, D), _eval_frictionloss_cost(ja, f, rf, D))
    elif iefc_type_in[worldid, iefcid] == types.ConstraintType.CONTACT_ELLIPTIC:
      conid = iefc_id_in[worldid, iefcid]
      if conid < nacon_in[0]:
        ic0 = map_efc2iefc_in[worldid, contact_efc_address_in[conid, 0]]
        if iefcid == ic0:
          cost0 = _eval_elliptic_cost_island(
            impratio_invsqrt,
            contact_friction_in,
            contact_dim_in,
            contact_efc_address_in,
            map_efc2iefc_in,
            0.0,
            conid,
            iefc_D_in,
            Jaref_in,
            jv_in,
            worldid,
          )[0]
          lo_in += _shift_cost(
            _eval_elliptic_cost_island(
              impratio_invsqrt,
              contact_friction_in,
              contact_dim_in,
              contact_efc_address_in,
              map_efc2iefc_in,
              lo_alpha_in,
              conid,
              iefc_D_in,
              Jaref_in,
              jv_in,
              worldid,
            ),
            cost0,
          )
    else:
      # Inequality
      x_a = ja + lo_alpha_in * jv_val
      quad0 = _eval_pt_direct_cost_alpha_zero(ja, D)
      cost0 = wp.where(ja < 0.0, quad0, 0.0)
      if x_a < 0.0:
        lo_in += _eval_pt_direct_shifted(ja, jv_val, D, lo_alpha_in, quad0 - cost0)
      else:
        lo_in += wp.vec3(-cost0, 0.0, 0.0)

  # Accept Newton step if derivative is small and cost improved
  initial_converged = wp.abs(lo_in[1]) < gtol and lo_in[0] < 0.0

  if initial_converged:
    alpha = lo_alpha_in
  else:
    alpha = float(0.0)

    # Initialize brackets
    lo_less = int(0)
    if lo_in[1] < p0[1]:
      lo_less = int(1)
    if lo_less == 1:
      lo = lo_in
      lo_alpha = lo_alpha_in
      hi = p0
      hi_alpha = float(0.0)
    else:
      lo = p0
      lo_alpha = float(0.0)
      hi = lo_in
      hi_alpha = lo_alpha_in

    for _iter in range(opt_ls_iterations):
      lo_next_alpha = lo_alpha - math.safe_div(lo[1], lo[2])
      hi_next_alpha = hi_alpha - math.safe_div(hi[1], hi[2])
      mid_alpha = 0.5 * (lo_alpha + hi_alpha)

      # Evaluate at 3 candidate alphas
      lo_next = _eval_pt(quad_gauss, lo_next_alpha)
      hi_next = _eval_pt(quad_gauss, hi_next_alpha)
      mid = _eval_pt(quad_gauss, mid_alpha)

      for iefcid in range(iefcadr, isle_nefc_end):
        if iefcid >= nefc:
          break
        local_iefcid = iefcid - iefcadr
        D = iefc_D_in[worldid, iefcid]
        ja = Jaref_in[worldid, iefcid]
        jv_val = jv_in[worldid, iefcid]
        if local_iefcid < ine:
          r_lo, r_hi, r_mid = _eval_pt_direct_shifted_3alphas(ja, jv_val, D, lo_next_alpha, hi_next_alpha, mid_alpha, 0.0)
        elif local_iefcid < ine + inf:
          f = iefc_frictionloss_in[worldid, iefcid]
          rf = math.safe_div(f, D)
          cost0 = _eval_frictionloss_cost(ja, f, rf, D)
          x_lo = ja + lo_next_alpha * jv_val
          x_hi = ja + hi_next_alpha * jv_val
          x_mid = ja + mid_alpha * jv_val
          r_lo = _shift_cost(_eval_frictionloss_pt(x_lo, f, rf, jv_val, D), cost0)
          r_hi = _shift_cost(_eval_frictionloss_pt(x_hi, f, rf, jv_val, D), cost0)
          r_mid = _shift_cost(_eval_frictionloss_pt(x_mid, f, rf, jv_val, D), cost0)
        elif iefc_type_in[worldid, iefcid] == types.ConstraintType.CONTACT_ELLIPTIC:
          conid = iefc_id_in[worldid, iefcid]
          r_lo = wp.vec3(0.0)
          r_hi = wp.vec3(0.0)
          r_mid = wp.vec3(0.0)
          if conid < nacon_in[0]:
            ic0 = map_efc2iefc_in[worldid, contact_efc_address_in[conid, 0]]
            if iefcid == ic0:
              cost0 = _eval_elliptic_cost_island(
                impratio_invsqrt,
                contact_friction_in,
                contact_dim_in,
                contact_efc_address_in,
                map_efc2iefc_in,
                0.0,
                conid,
                iefc_D_in,
                Jaref_in,
                jv_in,
                worldid,
              )[0]
              r_lo = _shift_cost(
                _eval_elliptic_cost_island(
                  impratio_invsqrt,
                  contact_friction_in,
                  contact_dim_in,
                  contact_efc_address_in,
                  map_efc2iefc_in,
                  lo_next_alpha,
                  conid,
                  iefc_D_in,
                  Jaref_in,
                  jv_in,
                  worldid,
                ),
                cost0,
              )
              r_hi = _shift_cost(
                _eval_elliptic_cost_island(
                  impratio_invsqrt,
                  contact_friction_in,
                  contact_dim_in,
                  contact_efc_address_in,
                  map_efc2iefc_in,
                  hi_next_alpha,
                  conid,
                  iefc_D_in,
                  Jaref_in,
                  jv_in,
                  worldid,
                ),
                cost0,
              )
              r_mid = _shift_cost(
                _eval_elliptic_cost_island(
                  impratio_invsqrt,
                  contact_friction_in,
                  contact_dim_in,
                  contact_efc_address_in,
                  map_efc2iefc_in,
                  mid_alpha,
                  conid,
                  iefc_D_in,
                  Jaref_in,
                  jv_in,
                  worldid,
                ),
                cost0,
              )
        else:
          # Inequality
          x_lo = ja + lo_next_alpha * jv_val
          x_hi = ja + hi_next_alpha * jv_val
          x_mid = ja + mid_alpha * jv_val
          quad0 = _eval_pt_direct_cost_alpha_zero(ja, D)
          cost0 = wp.where(ja < 0.0, quad0, 0.0)
          offset = quad0 - cost0
          neg_cost0 = wp.vec3(-cost0, 0.0, 0.0)
          r_lo = neg_cost0
          r_hi = neg_cost0
          r_mid = neg_cost0
          if x_lo < 0.0:
            r_lo = _eval_pt_direct_shifted(ja, jv_val, D, lo_next_alpha, offset)
          if x_hi < 0.0:
            r_hi = _eval_pt_direct_shifted(ja, jv_val, D, hi_next_alpha, offset)
          if x_mid < 0.0:
            r_mid = _eval_pt_direct_shifted(ja, jv_val, D, mid_alpha, offset)
        lo_next += r_lo
        hi_next += r_hi
        mid += r_mid

      # Bracket swapping
      swap_lo = int(0)
      if _in_bracket(lo, lo_next):
        lo = lo_next
        lo_alpha = lo_next_alpha
        swap_lo = int(1)
      if _in_bracket(lo, mid):
        lo = mid
        lo_alpha = mid_alpha
        swap_lo = int(1)
      if _in_bracket(lo, hi_next):
        lo = hi_next
        lo_alpha = hi_next_alpha
        swap_lo = int(1)

      swap_hi = int(0)
      if _in_bracket(hi, hi_next):
        hi = hi_next
        hi_alpha = hi_next_alpha
        swap_hi = int(1)
      if _in_bracket(hi, mid):
        hi = mid
        hi_alpha = mid_alpha
        swap_hi = int(1)
      if _in_bracket(hi, lo_next):
        hi = lo_next
        hi_alpha = lo_next_alpha
        swap_hi = int(1)

      # Done check
      ls_done = (swap_lo == 0 and swap_hi == 0) or (lo[1] < 0.0 and lo[1] > -gtol) or (hi[1] > 0.0 and hi[1] < gtol)

      # Update alpha if improved
      if lo[0] < 0.0 or hi[0] < 0.0:
        if lo[0] < hi[0]:
          alpha = lo_alpha
        else:
          alpha = hi_alpha

      if ls_done:
        break

  island_alpha_out[worldid, islandid] = alpha


@wp.kernel
def _linesearch_qacc_ma_island(
  # Data in:
  nidof_in: wp.array[int],
  # In:
  search_in: wp.array2d[float],
  mv_in: wp.array2d[float],
  island_alpha_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  iacc_out: wp.array2d[float],
  iMa_out: wp.array2d[float],
):
  """Update iacc and iMa after linesearch."""
  worldid, tid = wp.tid()

  # Process DOFs
  if tid < nidof_in[worldid]:
    idof = tid
    islandid = idof_islandid_in[worldid, idof]
    if islandid >= 0 and not island_done_in[worldid, islandid]:
      alpha = island_alpha_in[worldid, islandid]
      iacc_out[worldid, idof] += alpha * search_in[worldid, idof]
      iMa_out[worldid, idof] += alpha * mv_in[worldid, idof]


@wp.kernel
def _linesearch_jaref_island(
  # Data in:
  nefc_in: wp.array[int],
  njmax_in: int,
  # In:
  jv_in: wp.array2d[float],
  island_alpha_in: wp.array2d[float],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  Jaref_out: wp.array2d[float],
):
  """Update Jaref after linesearch."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  islandid = iefc_islandid_in[worldid, iefcid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  alpha = island_alpha_in[worldid, islandid]
  Jaref_out[worldid, iefcid] += alpha * jv_in[worldid, iefcid]


@wp.kernel
def _solve_prev_grad_Mgrad_island(
  # Data in:
  nidof_in: wp.array[int],
  # In:
  grad_in: wp.array2d[float],
  Mgrad_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  prev_grad_out: wp.array2d[float],
  prev_Mgrad_out: wp.array2d[float],
):
  """Save prev_grad and prev_Mgrad per island DOF."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  prev_grad_out[worldid, idofid] = grad_in[worldid, idofid]
  prev_Mgrad_out[worldid, idofid] = Mgrad_in[worldid, idofid]


@wp.kernel
def _solve_beta_island_zero(
  # In:
  nisland_in: wp.array[int],
  # Out:
  island_beta_num_out: wp.array2d[float],
  island_beta_den_out: wp.array2d[float],
):
  """Zero Polak-Ribière numerator and denominator per island."""
  worldid, islandid = wp.tid()

  if islandid >= nisland_in[worldid]:
    return

  island_beta_num_out[worldid, islandid] = 0.0
  island_beta_den_out[worldid, islandid] = 0.0


@wp.kernel
def _solve_beta_island_accumulate(
  # Data in:
  nidof_in: wp.array[int],
  # In:
  idof_islandid_in: wp.array2d[int],
  grad_in: wp.array2d[float],
  Mgrad_in: wp.array2d[float],
  prev_grad_in: wp.array2d[float],
  prev_Mgrad_in: wp.array2d[float],
  island_done_in: wp.array2d[bool],
  # Out:
  island_beta_num_out: wp.array2d[float],
  island_beta_den_out: wp.array2d[float],
):
  """Parallel Polak-Ribière beta accumulation per island DOF."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  pMg = prev_Mgrad_in[worldid, idofid]
  num = grad_in[worldid, idofid] * (Mgrad_in[worldid, idofid] - pMg)
  den = prev_grad_in[worldid, idofid] * pMg
  wp.atomic_add(island_beta_num_out, worldid, islandid, num)
  wp.atomic_add(island_beta_den_out, worldid, islandid, den)


@wp.kernel
def _solve_beta_island_finalize(
  # Data in:
  nisland_in: wp.array[int],
  # In:
  island_beta_num_in: wp.array2d[float],
  island_beta_den_in: wp.array2d[float],
  island_done_in: wp.array2d[bool],
  # Out:
  island_beta_out: wp.array2d[float],
):
  """Finalize Polak-Ribière beta per island."""
  worldid, islandid = wp.tid()

  if islandid >= nisland_in[worldid]:
    return

  if island_done_in[worldid, islandid]:
    island_beta_out[worldid, islandid] = 0.0
    return

  island_beta_out[worldid, islandid] = wp.max(
    0.0, island_beta_num_in[worldid, islandid] / wp.max(types.MJ_MINVAL, island_beta_den_in[worldid, islandid])
  )


@wp.kernel
def _solve_search_update_island(
  # Model:
  opt_solver: int,
  # Data in:
  nidof_in: wp.array[int],
  # In:
  Mgrad_in: wp.array2d[float],
  search_in: wp.array2d[float],
  island_beta_in: wp.array2d[float],
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  search_out: wp.array2d[float],
  island_search_dot_out: wp.array2d[float],
):
  """Update search direction per island DOF."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  s = -Mgrad_in[worldid, idofid]
  if opt_solver == types.SolverType.CG:
    s += island_beta_in[worldid, islandid] * search_in[worldid, idofid]

  search_out[worldid, idofid] = s
  wp.atomic_add(island_search_dot_out, worldid, islandid, s * s)


@wp.kernel
def _solve_done_island(
  # Model:
  opt_tolerance: wp.array[float],
  opt_iterations: int,
  stat_meaninertia: wp.array[float],
  # Data in:
  nisland_in: wp.array[int],
  island_nv_in: wp.array2d[int],
  # In:
  island_grad_dot_in: wp.array2d[float],
  island_cost_in: wp.array2d[float],
  island_prev_cost_in: wp.array2d[float],
  island_done_in: wp.array2d[bool],
  # Data out:
  solver_niter_out: wp.array[int],
  # Out:
  island_done_out: wp.array2d[bool],
  island_solver_niter_out: wp.array2d[int],
  nsolving_out: wp.array[int],
):
  """Check convergence per island."""
  worldid, islandid = wp.tid()

  if islandid >= nisland_in[worldid]:
    return

  tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
  meaninertia = stat_meaninertia[worldid % stat_meaninertia.shape[0]]

  if island_done_in[worldid, islandid]:
    niter = island_solver_niter_out[worldid, islandid]
    wp.atomic_max(solver_niter_out, worldid, niter)
    return

  island_solver_niter_out[worldid, islandid] += 1
  niter = island_solver_niter_out[worldid, islandid]
  wp.atomic_max(solver_niter_out, worldid, niter)

  inv = island_nv_in[worldid, islandid]
  improvement = _rescale(inv, meaninertia, island_prev_cost_in[worldid, islandid] - island_cost_in[worldid, islandid])
  gradient = _rescale(inv, meaninertia, wp.sqrt(island_grad_dot_in[worldid, islandid]))
  done = (improvement < tolerance) or (gradient < tolerance)
  if done or niter >= opt_iterations:
    island_done_out[worldid, islandid] = True
    wp.atomic_sub(nsolving_out, 0, 1)


@wp.kernel
def _update_gradient_JTDAJ_island(
  # Model:
  is_sparse: bool,
  # Data in:
  nefc_in: wp.array[int],
  njmax_in: int,
  island_idofadr_in: wp.array2d[int],
  island_nv_in: wp.array2d[int],
  # In:
  iefc_J_rownnz_in: wp.array2d[int],
  iefc_J_rowadr_in: wp.array2d[int],
  iefc_J_colind_in: wp.array3d[int],
  iefc_J_in: wp.array3d[float],
  iefc_D_in: wp.array2d[float],
  iefc_state_in: wp.array2d[int],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  ih_out: wp.array3d[float],
):
  """Build island Hessian: ih += Jᵀ·D·J for active constraints."""
  worldid, iefcid = wp.tid()

  if iefcid >= wp.min(njmax_in, nefc_in[worldid]):
    return

  islandid = iefc_islandid_in[worldid, iefcid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  state = iefc_state_in[worldid, iefcid]
  if state != types.ConstraintState.QUADRATIC.value:
    return
  D = iefc_D_in[worldid, iefcid]

  idofadr = island_idofadr_in[worldid, islandid]
  inv = island_nv_in[worldid, islandid]
  if is_sparse:
    rownnz = iefc_J_rownnz_in[worldid, iefcid]
    rowadr = iefc_J_rowadr_in[worldid, iefcid]
    for k1 in range(rownnz):
      adr1 = rowadr + k1
      Ji = iefc_J_in[worldid, 0, adr1]
      i = iefc_J_colind_in[worldid, 0, adr1]

      for k2 in range(k1 + 1):
        adr2 = rowadr + k2
        Jj = iefc_J_in[worldid, 0, adr2]
        j = iefc_J_colind_in[worldid, 0, adr2]

        h = Ji * Jj * D
        wp.atomic_add(ih_out[worldid, i], j, h)
        if i != j:
          wp.atomic_add(ih_out[worldid, j], i, h)
  else:
    for ii in range(inv):
      i = idofadr + ii
      Ji = iefc_J_in[worldid, iefcid, i]
      if Ji == 0.0:
        continue
      for jj in range(ii + 1):
        j = idofadr + jj
        Jj = iefc_J_in[worldid, iefcid, j]
        if Jj == 0.0:
          continue
        h = Ji * Jj * D
        wp.atomic_add(ih_out[worldid, i], j, h)
        if i != j:
          wp.atomic_add(ih_out[worldid, j], i, h)


@wp.kernel
def _update_gradient_set_h_M_sparse_island(
  # Model:
  M_fullm_i: wp.array[int],
  M_fullm_j: wp.array[int],
  M_elemid: wp.array2d[int],
  # Data in:
  nidof_in: wp.array[int],
  M_in: wp.array3d[float],
  dof_island_in: wp.array2d[int],
  map_dof2idof_in: wp.array2d[int],
  # In:
  island_done_in: wp.array2d[bool],
  # Out:
  ih_out: wp.array3d[float],
):
  """Add sparse mass matrix to island Hessian using global-to-island DOF mapping."""
  worldid, elementid = wp.tid()

  i_global = M_fullm_i[elementid]
  j_global = M_fullm_j[elementid]

  madr = M_elemid[i_global, j_global]
  if madr < 0:
    return

  # Check both DOFs belong to an island
  island_i = dof_island_in[worldid, i_global]
  if island_i < 0:
    return
  if island_done_in[worldid, island_i]:
    return

  island_j = dof_island_in[worldid, j_global]
  if island_j < 0:
    return

  # Both DOFs must be in the same island
  if island_i != island_j:
    return

  idof_i = map_dof2idof_in[worldid, i_global]
  idof_j = map_dof2idof_in[worldid, j_global]

  val = M_in[worldid, 0, madr]
  ih_out[worldid, idof_i, idof_j] += val
  if idof_i != idof_j:
    ih_out[worldid, idof_j, idof_i] += val


@wp.kernel
def _update_gradient_set_h_M_dense_island(
  # Model:
  nv: int,
  # Data in:
  nidof_in: wp.array[int],
  M_in: wp.array3d[float],
  map_idof2dof_in: wp.array2d[int],
  # In:
  idof_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  ih_out: wp.array3d[float],
):
  """Add dense mass matrix to island Hessian."""
  worldid, idofid = wp.tid()

  if idofid >= nidof_in[worldid]:
    return

  islandid = idof_islandid_in[worldid, idofid]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  dof_i = map_idof2dof_in[worldid, idofid]

  # Copy row from M to ih, mapping columns
  nid = nidof_in[worldid]
  for jdof in range(nid):
    dof_j = map_idof2dof_in[worldid, jdof]
    ih_out[worldid, idofid, jdof] += M_in[worldid, dof_i, dof_j]


@wp.kernel
def _update_gradient_JTCJ_island(
  # Model:
  opt_impratio_invsqrt: wp.array[float],
  is_sparse: bool,
  # Data in:
  nacon_in: wp.array[int],
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  contact_worldid_in: wp.array[int],
  island_idofadr_in: wp.array2d[int],
  naconmax_in: int,
  nidof_in: wp.array[int],
  map_efc2iefc_in: wp.array2d[int],
  island_nv_in: wp.array2d[int],
  # In:
  iefc_J_rownnz_in: wp.array2d[int],
  iefc_J_rowadr_in: wp.array2d[int],
  iefc_J_colind_in: wp.array3d[int],
  iefc_J_in: wp.array3d[float],
  iefc_D_in: wp.array2d[float],
  iefc_state_in: wp.array2d[int],
  Jaref_in: wp.array2d[float],
  iefc_islandid_in: wp.array2d[int],
  island_done_in: wp.array2d[bool],
  # Out:
  ih_out: wp.array3d[float],
):
  """Add elliptic cone Hessian correction: Jᵀ·C·J for contacts in CONE state."""
  conid = wp.tid()

  if conid >= wp.min(naconmax_in, nacon_in[0]):
    return

  worldid = contact_worldid_in[conid]
  condim = contact_dim_in[conid]

  if condim == 1:
    return

  efcid0_global = contact_efc_address_in[conid, 0]
  if efcid0_global < 0:
    return

  ic0 = map_efc2iefc_in[worldid, efcid0_global]
  if iefc_state_in[worldid, ic0] != types.ConstraintState.CONE.value:
    return

  islandid = iefc_islandid_in[worldid, ic0]
  if islandid < 0:
    return
  if island_done_in[worldid, islandid]:
    return

  inv = island_nv_in[worldid, islandid]
  idofadr = island_idofadr_in[worldid, islandid]

  fri = contact_friction_in[conid]
  mu = fri[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
  mu2 = mu * mu
  dm = math.safe_div(iefc_D_in[worldid, ic0], mu2 * (1.0 + mu2))

  if dm == 0.0:
    return

  # Compute n and u vector
  n = Jaref_in[worldid, ic0] * mu
  u = types.vec6(n, 0.0, 0.0, 0.0, 0.0, 0.0)

  tt = float(0.0)
  for j in range(1, condim):
    efcidj_global = contact_efc_address_in[conid, j]
    if efcidj_global < 0:
      return
    icj = map_efc2iefc_in[worldid, efcidj_global]
    uj = Jaref_in[worldid, icj] * fri[j - 1]
    tt += uj * uj
    u[j] = uj

  if tt <= 0.0:
    t = 0.0
  else:
    t = wp.sqrt(tt)
  t = wp.max(t, types.MJ_MINVAL)
  ttt = wp.max(t * t * t, types.MJ_MINVAL)

  # Accumulate cone correction into ih
  for dim1id in range(condim):
    if dim1id == 0:
      ic1 = ic0
    else:
      efcid1_global = contact_efc_address_in[conid, dim1id]
      if efcid1_global < 0:
        return
      ic1 = map_efc2iefc_in[worldid, efcid1_global]

    ui = u[dim1id]

    for dim2id in range(dim1id + 1):
      if dim2id == 0:
        ic2 = ic0
      else:
        efcid2_global = contact_efc_address_in[conid, dim2id]
        if efcid2_global < 0:
          return
        ic2 = map_efc2iefc_in[worldid, efcid2_global]

      uj = u[dim2id]

      # Cone correction matrix
      if dim1id == 0 and dim2id == 0:
        hcone = 1.0
      elif dim1id == 0:
        hcone = -math.safe_div(mu, t) * uj
      elif dim2id == 0:
        hcone = -math.safe_div(mu, t) * ui
      else:
        hcone = mu * math.safe_div(n, ttt) * ui * uj
        if dim1id == dim2id:
          hcone += mu2 - mu * math.safe_div(n, t)

      # Scale by dm * friction
      if dim1id == 0:
        fri1 = mu
      else:
        fri1 = fri[dim1id - 1]
      if dim2id == 0:
        fri2 = mu
      else:
        fri2 = fri[dim2id - 1]

      hcone *= dm * fri1 * fri2

      if hcone == 0.0:
        continue

      # Accumulate J1^T * hcone * J2 into ih (lower triangle)
      if is_sparse:
        if dim1id == dim2id:
          rownnz = iefc_J_rownnz_in[worldid, ic1]
          rowadr = iefc_J_rowadr_in[worldid, ic1]
          for k1 in range(rownnz):
            adr1 = rowadr + k1
            J1 = iefc_J_in[worldid, 0, adr1]
            i = iefc_J_colind_in[worldid, 0, adr1]

            for k2 in range(k1 + 1):
              adr2 = rowadr + k2
              J2 = iefc_J_in[worldid, 0, adr2]
              j = iefc_J_colind_in[worldid, 0, adr2]

              val = hcone * J1 * J2
              wp.atomic_add(ih_out[worldid, i], j, val)
              if i != j:
                wp.atomic_add(ih_out[worldid, j], i, val)
        else:
          rownnz1 = iefc_J_rownnz_in[worldid, ic1]
          rowadr1 = iefc_J_rowadr_in[worldid, ic1]
          rownnz2 = iefc_J_rownnz_in[worldid, ic2]
          rowadr2 = iefc_J_rowadr_in[worldid, ic2]

          for k1 in range(rownnz1):
            adr1 = rowadr1 + k1
            J1 = iefc_J_in[worldid, 0, adr1]
            i = iefc_J_colind_in[worldid, 0, adr1]

            for k2 in range(rownnz2):
              adr2 = rowadr2 + k2
              J2 = iefc_J_in[worldid, 0, adr2]
              j = iefc_J_colind_in[worldid, 0, adr2]

              val = hcone * J1 * J2
              if i == j:
                wp.atomic_add(ih_out[worldid, i], j, val * 2.0)
              else:
                wp.atomic_add(ih_out[worldid, i], j, val)
                wp.atomic_add(ih_out[worldid, j], i, val)
      else:
        for i in range(inv):
          J1i = iefc_J_in[worldid, ic1, idofadr + i]
          if J1i == 0.0:
            continue
          for jj in range(i + 1):
            J2j = iefc_J_in[worldid, ic2, idofadr + jj]
            if J2j == 0.0:
              continue
            val = hcone * J1i * J2j
            wp.atomic_add(ih_out[worldid, idofadr + i], idofadr + jj, val)
            if i != jj:
              wp.atomic_add(ih_out[worldid, idofadr + jj], idofadr + i, val)

        if dim1id != dim2id:
          # Swap-pair contribution: hcone * J[ic2, i] * J[ic1, j].
          # Together with the loop above this gives the full
          # hcone * (J[ic1, i] * J[ic2, j] + J[ic2, i] * J[ic1, j])
          # contribution to cell (i, j).
          for i in range(inv):
            J2i = iefc_J_in[worldid, ic2, idofadr + i]
            if J2i == 0.0:
              continue
            for jj in range(i + 1):
              J1j = iefc_J_in[worldid, ic1, idofadr + jj]
              if J1j == 0.0:
                continue
              val = hcone * J2i * J1j
              wp.atomic_add(ih_out[worldid, idofadr + i], idofadr + jj, val)
              if i != jj:
                wp.atomic_add(ih_out[worldid, idofadr + jj], idofadr + i, val)


@wp.kernel
def _cholesky_factorize_solve_island(
  # Data in:
  nisland_in: wp.array[int],
  island_idofadr_in: wp.array2d[int],
  island_nv_in: wp.array2d[int],
  # In:
  grad_in: wp.array2d[float],
  ih_in: wp.array3d[float],
  island_done_in: wp.array2d[bool],
  # Out:
  Mgrad_out: wp.array2d[float],
):
  """Per-island Cholesky factorize and solve: Mgrad = H⁻¹ @ grad.

  One thread per (world, island). Performs dense in-place Cholesky factorization
  on the island's inv x inv subblock of ih, then forward/backward substitution.
  """
  worldid, islandid = wp.tid()

  if islandid >= nisland_in[worldid]:
    return
  if island_done_in[worldid, islandid]:
    return

  inv = island_nv_in[worldid, islandid]

  if inv == 0:
    return

  adr = island_idofadr_in[worldid, islandid]
  # Cholesky factorization in-place: L such that H = L @ L^T
  for i in range(inv):
    for j in range(i + 1):
      s = ih_in[worldid, adr + i, adr + j]
      for k in range(j):
        s -= ih_in[worldid, adr + i, adr + k] * ih_in[worldid, adr + j, adr + k]
      if i == j:
        if s <= 1e-6:
          s = 1e-6
        ih_in[worldid, adr + i, adr + j] = wp.sqrt(s)
      else:
        div = ih_in[worldid, adr + j, adr + j]
        ih_in[worldid, adr + i, adr + j] = s / wp.max(1e-6, div)

  # Forward substitution: L @ y = grad  =>  y
  for i in range(inv):
    s = grad_in[worldid, adr + i]
    for k in range(i):
      s -= ih_in[worldid, adr + i, adr + k] * Mgrad_out[worldid, adr + k]
    Mgrad_out[worldid, adr + i] = s / wp.max(1e-6, ih_in[worldid, adr + i, adr + i])

  # Backward substitution: L^T @ x = y  =>  x = Mgrad
  for i_rev in range(inv):
    i = inv - 1 - i_rev
    s = Mgrad_out[worldid, adr + i]
    for k in range(i + 1, inv):
      s -= ih_in[worldid, adr + k, adr + i] * Mgrad_out[worldid, adr + k]
    Mgrad_out[worldid, adr + i] = s / wp.max(types.MJ_MINVAL, ih_in[worldid, adr + i, adr + i])


def _init_context_island(m: types.Model, d: types.Data, ctx: IslandSolverContext, nsolving: wp.array):
  """Initialize island solver context."""
  # Init per-island scalars
  d.solver_niter.zero_()
  enable_sleep = bool(m.opt.enableflags & types.EnableBit.SLEEP)
  wp.launch(
    _solve_init_efc_island(enable_sleep),
    dim=(d.nworld, m.ntree),
    inputs=[
      m.ntree,
      d.nisland,
      d.tree_awake,
      d.tree_island,
    ],
    outputs=[ctx.cost, ctx.search_dot, ctx.done, ctx.solver_niter, nsolving],
  )

  # Jaref = iefc_J @ iacc - iefc_aref
  wp.launch(
    _solve_init_jaref_island,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.is_sparse,
      d.nefc,
      d.island_idofadr,
      d.island_nv,
      d.njmax,
      d.efc.iJ_rownnz,
      d.efc.iJ_rowadr,
      d.efc.iJ_colind,
      d.efc.iJ,
      d.iqacc,
      d.efc.iaref,
      d.efc_islandid,
      ctx.done,
    ],
    outputs=[ctx.Jaref],
  )

  # iMa = M @ iacc (all islands in parallel)
  support.mul_m_island(
    m,
    d,
    ctx.Ma,
    d.iqacc,
    d.nidof,
    d.map_idof2dof,
    d.map_dof2idof,
    d.dof_islandid,
  )

  # Update constraint
  _update_constraint_island(m, d, ctx)

  # Update gradient
  if m.opt.solver == types.SolverType.NEWTON:
    _update_gradient_incremental_island(m, d, ctx)
  else:
    _update_gradient_island(m, d, ctx)


@event_scope
def _update_constraint_island(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Update constraint arrays for island solver."""
  # Save prev cost, zero cost/gauss
  wp.launch(
    _update_constraint_init_cost,
    dim=d.nworld * m.ntree,
    inputs=[ctx.cost.reshape(-1), ctx.done.reshape(-1)],
    outputs=[
      ctx.gauss.reshape(-1),
      ctx.cost.reshape(-1),
      ctx.prev_cost.reshape(-1),
    ],
  )

  # Compute force, state, cost per EFC
  wp.launch(
    _update_constraint_efc_island,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.opt.impratio_invsqrt,
      d.nefc,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.island_nefc,
      d.island_ne,
      d.island_nf,
      d.island_efcadr,
      d.map_efc2iefc,
      d.njmax,
      d.nacon,
      d.efc.itype,
      d.efc.iid,
      d.efc.iD,
      d.efc.ifrictionloss,
      ctx.Jaref,
      d.efc_islandid,
      ctx.done,
    ],
    outputs=[d.efc.iforce, d.efc.istate, ctx.cost],
  )

  # qfrc_constraint = J^T @ force
  if m.is_sparse:
    d.iqfrc_constraint.zero_()
    wp.launch(
      _update_constraint_init_qfrc_constraint_sparse_island,
      dim=(d.nworld, d.njmax),
      inputs=[
        d.nefc,
        d.njmax,
        d.efc.iJ_rownnz,
        d.efc.iJ_rowadr,
        d.efc.iJ_colind,
        d.efc.iJ,
        d.efc.iforce,
        d.efc_islandid,
        ctx.done,
      ],
      outputs=[d.iqfrc_constraint],
    )
  else:
    wp.launch(
      _update_constraint_init_qfrc_constraint_dense_island,
      dim=(d.nworld, m.nv),
      inputs=[
        d.nefc,
        d.nidof,
        d.island_nefc,
        d.island_efcadr,
        d.njmax,
        d.efc.iJ,
        d.efc.iforce,
        d.dof_islandid,
        ctx.done,
      ],
      outputs=[d.iqfrc_constraint],
    )

  # Gauss cost
  wp.launch(
    _update_constraint_gauss_cost_island,
    dim=(d.nworld, m.nv),
    inputs=[
      d.nidof,
      d.iqacc,
      d.iqfrc_smooth,
      d.iqacc_smooth,
      ctx.Ma,
      d.dof_islandid,
      ctx.done,
    ],
    outputs=[ctx.gauss, ctx.cost],
  )


@event_scope
def _update_gradient_island(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Update gradient for island solver."""
  # Zero grad_dot per island
  ctx.grad_dot.zero_()

  # grad = Ma - frc_smooth - frc_constraint, accumulate grad_dot
  wp.launch(
    _update_gradient_grad_island,
    dim=(d.nworld, m.nv),
    inputs=[
      d.nidof,
      d.iqfrc_smooth,
      d.iqfrc_constraint,
      ctx.Ma,
      d.dof_islandid,
      ctx.done,
    ],
    outputs=[ctx.grad, ctx.grad_dot],
  )

  # CG preconditioner: Mgrad = M^{-1} @ grad (direct solve)
  support.solve_m_island(
    m,
    d,
    ctx.Mgrad,
    ctx.grad,
    d.nidof,
    d.map_idof2dof,
  )


@event_scope
def _update_gradient_incremental_island(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Full Newton gradient update for islands: build H, factorize, solve."""
  # Zero grad_dot per island
  ctx.grad_dot.zero_()

  # grad = Ma - frc_smooth - frc_constraint, accumulate grad_dot
  wp.launch(
    _update_gradient_grad_island,
    dim=(d.nworld, m.nv),
    inputs=[
      d.nidof,
      d.iqfrc_smooth,
      d.iqfrc_constraint,
      ctx.Ma,
      d.dof_islandid,
      ctx.done,
    ],
    outputs=[ctx.grad, ctx.grad_dot],
  )

  # Build H = qM + Jᵀ·D·J
  ctx.h.zero_()

  # JTDAJ
  wp.launch(
    _update_gradient_JTDAJ_island,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.is_sparse,
      d.nefc,
      d.njmax,
      d.island_idofadr,
      d.island_nv,
      d.efc.iJ_rownnz,
      d.efc.iJ_rowadr,
      d.efc.iJ_colind,
      d.efc.iJ,
      d.efc.iD,
      d.efc.istate,
      d.efc_islandid,
      ctx.done,
    ],
    outputs=[ctx.h],
  )

  # Add mass matrix
  if m.is_sparse:
    wp.launch(
      _update_gradient_set_h_M_sparse_island,
      dim=(d.nworld, m.M_fullm_i.shape[0]),
      inputs=[
        m.M_fullm_i,
        m.M_fullm_j,
        m.M_elemid,
        d.nidof,
        d.M,
        d.dof_island,
        d.map_dof2idof,
        ctx.done,
      ],
      outputs=[ctx.h],
    )
  else:
    wp.launch(
      _update_gradient_set_h_M_dense_island,
      dim=(d.nworld, m.nv),
      inputs=[
        m.nv,
        d.nidof,
        d.M,
        d.map_idof2dof,
        d.dof_islandid,
        ctx.done,
      ],
      outputs=[ctx.h],
    )

  # Elliptic cone correction: JTCJ
  if m.opt.cone == types.ConeType.ELLIPTIC and d.naconmax > 0:
    wp.launch(
      _update_gradient_JTCJ_island,
      dim=d.naconmax,
      inputs=[
        m.opt.impratio_invsqrt,
        m.is_sparse,
        d.nacon,
        d.contact.friction,
        d.contact.dim,
        d.contact.efc_address,
        d.contact.worldid,
        d.island_idofadr,
        d.naconmax,
        d.nidof,
        d.map_efc2iefc,
        d.island_nv,
        d.efc.iJ_rownnz,
        d.efc.iJ_rowadr,
        d.efc.iJ_colind,
        d.efc.iJ,
        d.efc.iD,
        d.efc.istate,
        ctx.Jaref,
        d.efc_islandid,
        ctx.done,
      ],
      outputs=[ctx.h],
    )

  # Cholesky factorize and solve: Mgrad = H⁻¹ @ grad
  wp.launch(
    _cholesky_factorize_solve_island,
    dim=(d.nworld, m.ntree),
    inputs=[
      d.nisland,
      d.island_idofadr,
      d.island_nv,
      ctx.grad,
      ctx.h,
      ctx.done,
    ],
    outputs=[ctx.Mgrad],
  )


@event_scope
def _linesearch_island(m: types.Model, d: types.Data, ctx: IslandSolverContext):
  """Linesearch for island solver."""
  # mv = M @ search (all islands)
  support.mul_m_island(
    m,
    d,
    ctx.mv,
    ctx.search,
    d.nidof,
    d.map_idof2dof,
    d.map_dof2idof,
    d.dof_islandid,
    island_done=ctx.done,
  )

  # jv = J @ search
  wp.launch(
    _linesearch_jv_island,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.is_sparse,
      d.nefc,
      d.nidof,
      d.island_idofadr,
      d.island_nv,
      d.njmax,
      d.efc.iJ_rownnz,
      d.efc.iJ_rowadr,
      d.efc.iJ_colind,
      d.efc.iJ,
      ctx.search,
      d.efc_islandid,
      ctx.done,
    ],
    outputs=[ctx.jv],
  )

  # linesearch
  wp.launch(
    _linesearch_kernel_island,
    dim=(d.nworld, m.ntree),
    inputs=[
      m.opt.tolerance,
      m.opt.ls_tolerance,
      m.opt.ls_iterations,
      m.opt.impratio_invsqrt,
      m.stat.meaninertia,
      d.nefc,
      d.nisland,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.nidof,
      d.island_nv,
      d.island_ne,
      d.island_nf,
      d.island_efcadr,
      d.island_nefc,
      d.map_efc2iefc,
      d.njmax,
      d.nacon,
      d.island_idofadr,
      d.efc.itype,
      d.efc.iid,
      d.efc.iD,
      d.efc.ifrictionloss,
      ctx.Jaref,
      ctx.jv,
      ctx.mv,
      ctx.search,
      d.iqfrc_smooth,
      ctx.Ma,
      ctx.search_dot,
      ctx.gauss,
      ctx.done,
    ],
    outputs=[ctx.alpha],
  )

  # Update iacc, iMa
  wp.launch(
    _linesearch_qacc_ma_island,
    dim=(d.nworld, m.nv),
    inputs=[
      d.nidof,
      ctx.search,
      ctx.mv,
      ctx.alpha,
      d.dof_islandid,
      ctx.done,
    ],
    outputs=[d.iqacc, ctx.Ma],
  )

  # Update Jaref
  wp.launch(
    _linesearch_jaref_island,
    dim=(d.nworld, d.njmax),
    inputs=[
      d.nefc,
      d.njmax,
      ctx.jv,
      ctx.alpha,
      d.efc_islandid,
      ctx.done,
    ],
    outputs=[ctx.Jaref],
  )


@event_scope
def _solver_iteration_island(
  m: types.Model,
  d: types.Data,
  ctx: IslandSolverContext,
  nsolving: wp.array[int],
):
  """One iteration of island solver for all islands in parallel."""
  _linesearch_island(m, d, ctx)

  is_newton = m.opt.solver == types.SolverType.NEWTON
  is_cg = not is_newton

  # Save prev_grad, prev_Mgrad for CG
  if is_cg:
    wp.launch(
      _solve_prev_grad_Mgrad_island,
      dim=(d.nworld, m.nv),
      inputs=[d.nidof, ctx.grad, ctx.Mgrad, d.dof_islandid, ctx.done],
      outputs=[ctx.prev_grad, ctx.prev_Mgrad],
    )

  # Update constraint
  _update_constraint_island(m, d, ctx)

  # Update gradient
  if is_newton:
    _update_gradient_incremental_island(m, d, ctx)
  else:
    _update_gradient_island(m, d, ctx)

  # Polak-Ribière beta (CG only)
  if is_cg:
    wp.launch(
      _solve_beta_island_zero,
      dim=(d.nworld, m.ntree),
      inputs=[d.nisland],
      outputs=[ctx.beta, ctx.beta_den],
    )
    wp.launch(
      _solve_beta_island_accumulate,
      dim=(d.nworld, m.nv),
      inputs=[
        d.nidof,
        d.dof_islandid,
        ctx.grad,
        ctx.Mgrad,
        ctx.prev_grad,
        ctx.prev_Mgrad,
        ctx.done,
      ],
      outputs=[ctx.beta, ctx.beta_den],
    )
    wp.launch(
      _solve_beta_island_finalize,
      dim=(d.nworld, m.ntree),
      inputs=[d.nisland, ctx.beta, ctx.beta_den, ctx.done],
      outputs=[ctx.beta],
    )

  # Zero search_dot
  wp.launch(
    _solve_zero_search_dot,
    dim=d.nworld * m.ntree,
    inputs=[ctx.done.reshape(-1)],
    outputs=[ctx.search_dot.reshape(-1)],
  )

  # Search update
  wp.launch(
    _solve_search_update_island,
    dim=(d.nworld, m.nv),
    inputs=[
      m.opt.solver,
      d.nidof,
      ctx.Mgrad,
      ctx.search,
      ctx.beta,
      d.dof_islandid,
      ctx.done,
    ],
    outputs=[ctx.search, ctx.search_dot],
  )

  # Convergence check
  d.solver_niter.zero_()
  wp.launch(
    _solve_done_island,
    dim=(d.nworld, m.ntree),
    inputs=[
      m.opt.tolerance,
      m.opt.iterations,
      m.stat.meaninertia,
      d.nisland,
      d.island_nv,
      ctx.grad_dot,
      ctx.cost,
      ctx.prev_cost,
      ctx.done,
    ],
    outputs=[d.solver_niter, ctx.done, ctx.solver_niter, nsolving],
  )
