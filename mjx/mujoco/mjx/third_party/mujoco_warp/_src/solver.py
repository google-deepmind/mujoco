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

import dataclasses
from math import ceil
from typing import Any

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import island
from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_augmented_factorize_solve_newton_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_factorize_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_solve_newton_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import solve_search_sums
from mujoco.mjx.third_party.mujoco_warp._src.types import InverseContext
from mujoco.mjx.third_party.mujoco_warp._src.types import OverflowType
from mujoco.mjx.third_party.mujoco_warp._src.types import SolverContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False, "default_grid_stride": False})

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
    quad_changed_ids=wp.empty((nworld, 0), dtype=int),
    quad_changed_count=wp.empty((0,), dtype=int),
    state_changed_count=wp.empty((0,), dtype=int),
    ls_exhausted=wp.empty((0,), dtype=bool),
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
  alloc_mgrad = m.opt.solver == types.SolverType.CG
  alloc_incremental = _use_incremental(m)

  return SolverContext(
    Jaref=wp.empty((nworld, njmax), dtype=float),
    search_dot=wp.empty((nworld,), dtype=float),
    done=wp.empty((nworld,), dtype=bool),
    grad=wp.zeros((nworld, nv_pad), dtype=float),
    grad_dot=wp.empty((nworld,), dtype=float),
    newton_decrement=wp.empty((nworld,), dtype=float),
    Mgrad=wp.empty((nworld, nv_pad), dtype=float) if alloc_mgrad else wp.empty((nworld, 0), dtype=float),
    search=wp.empty((nworld, nv), dtype=float),
    mv=wp.empty((nworld, nv), dtype=float),
    jv=wp.empty((nworld, njmax), dtype=float),
    quad=wp.empty((nworld, njmax), dtype=wp.vec3),
    alpha=wp.empty((nworld,), dtype=float),
    grad_scale=wp.empty((nworld,), dtype=float),
    improvement=wp.empty((nworld,), dtype=float),
    ls_exhausted=wp.zeros((nworld,), dtype=bool),
    search_unchanged=wp.empty((nworld,), dtype=bool),
    prev_grad=wp.empty((nworld, nv), dtype=float) if alloc_mgrad else wp.empty((nworld, 0), dtype=float),
    prev_Mgrad=wp.empty((nworld, nv), dtype=float) if alloc_mgrad else wp.empty((nworld, 0), dtype=float),
    beta=wp.empty((nworld,), dtype=float) if alloc_mgrad else wp.empty((0,), dtype=float),
    beta_den=wp.empty((nworld,), dtype=float) if alloc_mgrad else wp.empty((0,), dtype=float),
    h=wp.empty((nworld, nv_pad, nv_pad), dtype=float) if alloc_h else wp.empty((nworld, 0, 0), dtype=float),
    hfactor=wp.empty((nworld, nv_pad, nv_pad), dtype=float) if alloc_hfactor else wp.empty((nworld, 0, 0), dtype=float),
    quad_changed_ids=wp.empty((nworld, njmax), dtype=int) if alloc_incremental else wp.empty((nworld, 0), dtype=int),
    quad_changed_count=wp.empty((nworld,), dtype=int) if alloc_incremental else wp.empty((0,), dtype=int),
    state_changed_count=wp.empty((nworld,), dtype=int) if alloc_incremental else wp.empty((0,), dtype=int),
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
def _eval_elliptic_reference(
  # In:
  mu: float,
  quad: wp.vec3,
  quad1: wp.vec3,
  quad2: wp.vec3,
) -> tuple[float, float, float, int]:
  u0 = quad1[0]
  uu = quad1[2]
  dm = quad2[2]

  if uu <= 0.0:
    if u0 < 0.0:
      return quad[0], 0.0, 0.0, int(types.ConstraintState.QUADRATIC)
    return 0.0, 0.0, 0.0, int(types.ConstraintState.SATISFIED)

  T0 = wp.sqrt(uu)
  if u0 >= mu * T0:
    return 0.0, T0, 0.0, int(types.ConstraintState.SATISFIED)
  if mu * u0 + T0 <= 0.0:
    return quad[0], T0, 0.0, int(types.ConstraintState.QUADRATIC)

  r0 = u0 - mu * T0
  return 0.5 * dm * r0 * r0, T0, r0, int(types.ConstraintState.CONE)


@wp.func
def _eval_elliptic_alpha_zero(mu: float, quad: wp.vec3, quad1: wp.vec3, quad2: wp.vec3) -> wp.vec3:
  cost0, T0, r0, state0 = _eval_elliptic_reference(mu, quad, quad1, quad2)
  if state0 == int(types.ConstraintState.QUADRATIC):
    return _eval_pt(quad, 0.0)
  if state0 == int(types.ConstraintState.CONE):
    T0_inv = 1.0 / T0
    T1 = quad2[0] * T0_inv
    T2 = (quad2[1] - T1 * T1) * T0_inv
    r1 = quad1[1] - mu * T1
    dm = quad2[2]
    return wp.vec3(cost0, dm * r0 * r1, dm * (r1 * r1 - mu * r0 * T2))
  return wp.vec3(0.0)


@wp.func
def _eval_elliptic_quadratic_cone_gap(mu: float, N: float, T: float, dm: float) -> float:
  """Return quadratic cost minus cone cost at the same point."""
  boundary = mu * N + T
  return 0.5 * dm * boundary * boundary


@wp.func
def _eval_elliptic_quadratic_shifted(
  # In:
  mu: float,
  quad: wp.vec3,
  alpha: float,
  N: float,
  Tsqr: float,
  u0: float,
  T0: float,
  dm: float,
  state0: int,
) -> wp.vec3:
  aq2 = alpha * quad[2]
  cost = alpha * (aq2 + quad[1])
  if state0 == int(types.ConstraintState.CONE):
    cost += _eval_elliptic_quadratic_cone_gap(mu, u0, T0, dm)
  elif state0 == int(types.ConstraintState.SATISFIED):
    cost = 0.5 * dm * (1.0 + mu * mu) * (N * N + wp.max(Tsqr, 0.0))
  return wp.vec3(cost, 2.0 * aq2 + quad[1], 2.0 * quad[2])


@wp.func
def _eval_elliptic_shifted(
  # In:
  mu: float,
  quad: wp.vec3,
  quad1: wp.vec3,
  quad2: wp.vec3,
  alpha: float,
  cost0: float,
  T0: float,
  r0: float,
  state0: int,
) -> wp.vec3:
  u0 = quad1[0]
  v0 = quad1[1]
  uu = quad1[2]
  uv = quad2[0]
  vv = quad2[1]
  dm = quad2[2]

  N = u0 + alpha * v0
  Tsqr_delta = alpha * (2.0 * uv + alpha * vv)
  Tsqr = uu + Tsqr_delta

  if Tsqr <= 0.0:
    if N < 0.0:
      return _eval_elliptic_quadratic_shifted(mu, quad, alpha, N, Tsqr, u0, T0, dm, state0)
  else:
    T = wp.sqrt(Tsqr)
    if N >= mu * T:
      pass
    elif mu * N + T <= 0.0:
      return _eval_elliptic_quadratic_shifted(mu, quad, alpha, N, Tsqr, u0, T0, dm, state0)
    else:
      T_inv = 1.0 / T
      T1 = (uv + alpha * vv) * T_inv
      T2 = (vv - T1 * T1) * T_inv
      r = N - mu * T
      r1 = v0 - mu * T1

      if state0 == int(types.ConstraintState.CONE):
        # Rationalize T - T0 before forming the small cone residual change.
        T_delta = Tsqr_delta / (T + T0)
        r_delta = alpha * v0 - mu * T_delta
        cost = 0.5 * dm * r_delta * (2.0 * r0 + r_delta)
      elif state0 == int(types.ConstraintState.QUADRATIC):
        aq2 = alpha * quad[2]
        cost = alpha * (aq2 + quad[1]) - _eval_elliptic_quadratic_cone_gap(mu, N, T, dm)
      else:
        cost = 0.5 * dm * r * r

      return wp.vec3(
        cost,
        dm * r * r1,
        dm * (r1 * r1 + r * (-mu * T2)),
      )

  return wp.vec3(-cost0, 0.0, 0.0)


@wp.func
def _eval_elliptic_middle(
  # In:
  N: float,
  T: float,
  D0: float,
  mu: float,
  ufrictionj: float,
  is_normal: bool,
) -> wp.vec2:
  """Computes the elliptic-cone middle-zone (force, cost) for one row (cost 0 on tangent rows)."""
  dm = math.safe_div(D0, mu * mu * (1.0 + mu * mu))
  nmt = N - mu * T
  force_normal = -dm * nmt * mu
  if is_normal:
    return wp.vec2(force_normal, 0.5 * dm * nmt * nmt)
  return wp.vec2(-math.safe_div(force_normal, T) * ufrictionj, 0.0)


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
      is_normal = efcid == efcid0
      fc = _eval_elliptic_middle(N, T, D0, mu, ufrictionj, is_normal)
      return wp.vec3(fc[0], float(types.ConstraintState.CONE.value), fc[1])

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
      cost0, T0, r0, state0 = _eval_elliptic_reference(mu, ctx_quad, quad1, quad2)
      return _eval_elliptic_shifted(mu, ctx_quad, quad1, quad2, alpha, cost0, T0, r0, state0)

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
      return _eval_elliptic_alpha_zero(mu, ctx_quad, quad1, quad2)

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
  # Contact/limit/other constraints
  if efcid >= ne + nf:
    # Contact elliptic: uses special elliptic cone evaluation
    if efc_type == types.ConstraintType.CONTACT_ELLIPTIC:
      if efcid != efc_address0:  # secondary rows contribute nothing
        return (wp.vec3(0.0), wp.vec3(0.0), wp.vec3(0.0))
      mu = contact_friction[0] * impratio_invsqrt
      cost0, T0, r0, state0 = _eval_elliptic_reference(mu, ctx_quad, quad1, quad2)
      return (
        _eval_elliptic_shifted(mu, ctx_quad, quad1, quad2, lo_alpha, cost0, T0, r0, state0),
        _eval_elliptic_shifted(mu, ctx_quad, quad1, quad2, hi_alpha, cost0, T0, r0, state0),
        _eval_elliptic_shifted(mu, ctx_quad, quad1, quad2, mid_alpha, cost0, T0, r0, state0),
      )

    # Limit/other constraints — direct eval (no quad read)
    x_lo = ctx_Jaref + lo_alpha * ctx_jv
    x_hi = ctx_Jaref + hi_alpha * ctx_jv
    x_mid = ctx_Jaref + mid_alpha * ctx_jv
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
    x_lo = ctx_Jaref + lo_alpha * ctx_jv
    x_hi = ctx_Jaref + hi_alpha * ctx_jv
    x_mid = ctx_Jaref + mid_alpha * ctx_jv
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
def _linesearch_iterative_kernel(
  ls_iterations: int,
  cone_type: types.ConeType,
  fuse_jv: bool,
  is_sparse: bool,
  incremental: bool,
  warn_overflow: bool,
):
  """Factory for iterative linesearch kernel.

  Args:
    ls_iterations: Max linesearch iterations (compile-time constant for loop optimization).
    cone_type: Friction cone type (PYRAMIDAL or ELLIPTIC) for compile-time optimization.
    fuse_jv: Whether to compute jv = J @ search in-kernel (efficient for small nv).
    is_sparse: Use sparse matrix representation for constraint Jacobian.
    incremental: State changes are tracked: flag exhausted rays, reuse jv on unchanged search.
    warn_overflow: Whether to print overflow warnings.
  """
  LS_ITERATIONS = ls_iterations
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  FUSE_JV = fuse_jv
  INCREMENTAL = incremental
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

  @wp.kernel(module="unique", enable_backward=False, grid_stride=True)
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
    ctx_search_unchanged_in: wp.array[bool],
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
    overflow_out: wp.array[int],
    # Out:
    ctx_Jaref_out: wp.array2d[float],
    ctx_jv_out: wp.array2d[float],
    ctx_quad_out: wp.array2d[wp.vec3],
    ctx_improvement_out: wp.array[float],
    ctx_alpha_out: wp.array[float],
    ctx_ls_exhausted_out: wp.array[bool],
  ):
    worldid, tid = wp.tid()

    if ctx_done_in[worldid]:
      return

    ne = ne_in[worldid]
    nf = nf_in[worldid]
    nefc = wp.min(njmax_in, nefc_in[worldid])

    # jv = J @ search (fused for small nv); on unchanged search the buffer
    # already holds jv (see _linesearch).
    if wp.static(FUSE_JV):
      recompute_jv = True
      if wp.static(INCREMENTAL):
        recompute_jv = not ctx_search_unchanged_in[worldid]
      if recompute_jv:
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

    # The bracketing search reads derivative sums whose rounding noise is ~eps
    # times their gross magnitude, so roots below eps * |q1| / p0[2] (and never
    # below 8 ulps of the unit ray anchor) are noise, not descent: an accepted
    # step under this floor means the stale ray is exhausted and the fast path
    # must rebuild the world. Per quadratic row |q1| = sqrt(2 * cost * hessian),
    # so Cauchy-Schwarz bounds the row total by sums already reduced for p0; the
    # smooth term is added exactly. Friction linear rows fall outside the bound,
    # which only lowers the floor toward the fixed 8-ulp base.
    noise_floor = float(0.0)
    if wp.static(INCREMENTAL):
      rows = p0_sum[0]
      q1_abs = wp.sqrt(2.0 * wp.max(rows[0], 0.0) * wp.max(rows[2], 0.0)) + wp.abs(ctx_quad_gauss[1])
      noise_floor = _ALPHA_NOISE_EPS * wp.max(1.0, math.safe_div(q1_abs, p0[2]))

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
    ls_converged = initial_converged

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
        ls_done = (
          (not swap_lo and not swap_hi)
          or (lo[0] < 0.0 and lo[1] < 0.0 and lo[1] > -gtol)
          or (hi[0] < 0.0 and hi[1] > 0.0 and hi[1] < gtol)
        )

        # update alpha if improved
        improved = lo[0] < 0.0 or hi[0] < 0.0
        lo_better = lo[0] < hi[0]
        best_alpha = wp.where(lo_better, lo_alpha, hi_alpha)
        best_delta = wp.where(lo_better, lo[0], hi[0])
        alpha = wp.where(improved, best_alpha, alpha)
        improvement = wp.where(improved, -best_delta, improvement)

        if ls_done:
          ls_converged = True
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
      ctx_alpha_out[worldid] = alpha
      if wp.static(INCREMENTAL):
        ctx_ls_exhausted_out[worldid] = wp.abs(alpha) < noise_floor
      if not ls_converged:
        if wp.static(warn_overflow):
          wp.printf(
            "linesearch iterations limit reached - please increase ls_iterations to %u\n",
            wp.static(LS_ITERATIONS),
          )
        wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.LS_ITERATIONS))

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
    _linesearch_iterative_kernel(
      m.opt.ls_iterations,
      m.opt.cone,
      fuse_jv,
      m.is_sparse,
      _use_incremental(m),
      bool(m.opt.warn_overflow),
    ),
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
      ctx.search_unchanged,
      ctx.Jaref,
      ctx.search,
      ctx.search_dot,
      ctx.mv,
      ctx.jv,
      ctx.quad,
      ctx.done,
    ],
    outputs=[
      d.qacc,
      d.efc.Ma,
      d.overflow,
      ctx.Jaref,
      ctx.jv,
      ctx.quad,
      ctx.improvement,
      ctx.alpha,
      ctx.ls_exhausted,
    ],
    block_dim=m.block_dim.linesearch_iterative,
  )


@wp.kernel
def _linesearch_zero_jv(
  # Data in:
  nefc_in: wp.array[int],
  # In:
  skip_in: wp.array[bool],
  # Out:
  ctx_jv_out: wp.array2d[float],
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if skip_in[worldid]:
    return

  ctx_jv_out[worldid, efcid] = 0.0


@cache_kernel
def _linesearch_jv_fused_kernel(is_sparse: bool, nv: int, dofs_per_thread: int, compact: bool):
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    dof_cdof_in: wp.array2d[int],
    # In:
    ctx_search_in: wp.array2d[float],
    skip_in: wp.array[bool],
    # Out:
    ctx_jv_out: wp.array2d[float],
  ):
    worldid, efcid, dofstart = wp.tid()

    if efcid >= nefc_in[worldid]:
      return

    if skip_in[worldid]:
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
          if wp.static(COMPACT):
            colind = dof_cdof_in[worldid, colind]
            if colind < 0:
              continue
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
            if wp.static(COMPACT):
              colind = dof_cdof_in[worldid, colind]
              if colind < 0:
                continue
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

  When state changes are tracked, worlds with ctx.search_unchanged reuse last
  iteration's mv/jv, so a fresh search requires clearing the flag (see the
  invalidation in _solve and the writer in _update_gradient_zero_grad_dot).

  Args:
    m: Model
    d: Data
    ctx: SolverContext
  """
  # mv and jv are pure functions of the search direction, and M and J are
  # constant within a solve, so worlds whose search was kept reuse last
  # iteration's values.
  skip = ctx.search_unchanged if _use_incremental(m) else ctx.done

  # mv = M @ search (common to both parallel and iterative)
  _mul_m_compact_aware(m, d, ctx, ctx.mv, ctx.search, skip)

  # Fuse jv computation in-kernel for small nv (iterative only, dense only)
  # Sparse mode requires pre-computed jv since in-kernel uses dense indexing
  # the sparse-compact J path reads the full model's sparse J structures;
  # dense full models keep the gathered dense cJ path
  sc = _sparse_compact(ctx)
  fuse_jv = m.nv <= 50 and not m.is_sparse and not sc

  # jv = J @ search (when not fused into iterative kernel)
  if not fuse_jv:
    dj = ctx.compact_d_full if sc else d
    if sc or m.is_sparse:
      # Sparse J has few nonzeros per row, one thread handles them all.
      dofs_per_thread = m.nv
      threads_per_efc = 1
    else:
      dofs_per_thread = 20 if m.nv > 50 else 50
      threads_per_efc = ceil(m.nv / dofs_per_thread)

    if threads_per_efc > 1:
      wp.launch(
        _linesearch_zero_jv,
        dim=(d.nworld, d.njmax),
        inputs=[d.nefc, skip],
        outputs=[ctx.jv],
      )

    wp.launch(
      _linesearch_jv_fused_kernel(sc or m.is_sparse, m.nv, dofs_per_thread, sc),
      dim=(d.nworld, d.njmax, threads_per_efc),
      inputs=[d.nefc, dj.efc.J_rownnz, dj.efc.J_rowadr, dj.efc.J_colind, dj.efc.J, dj.dof_cdof, ctx.search, skip],
      outputs=[ctx.jv],
    )

  _linesearch_iterative(m, d, ctx, fuse_jv)


@cache_kernel
def _solve_init_dof(warmstart: bool, sparse: bool):
  WARMSTART = warmstart
  SPARSE = sparse

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    qacc_warmstart_in: wp.array2d[float],
    qacc_smooth_in: wp.array2d[float],
    # Data out:
    qacc_out: wp.array2d[float],
    qfrc_constraint_out: wp.array2d[float],
  ):
    worldid, dofid = wp.tid()

    if wp.static(WARMSTART):
      qacc_out[worldid, dofid] = qacc_warmstart_in[worldid, dofid]
    else:
      qacc_out[worldid, dofid] = qacc_smooth_in[worldid, dofid]

    if wp.static(SPARSE):
      if nefc_in[worldid] == 0:
        qfrc_constraint_out[worldid, dofid] = 0.0

  return kernel


@wp.kernel(grid_stride=True)
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
def _solve_init_jaref_kernel(is_sparse: bool, nv: int, dofs_per_thread: int, compact: bool):
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=True)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    qacc_in: wp.array2d[float],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_aref_in: wp.array2d[float],
    dof_cdof_in: wp.array2d[int],
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
        if wp.static(COMPACT):
          colind = dof_cdof_in[worldid, colind]
          if colind < 0:
            continue
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

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
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
    ctx_ls_exhausted_in: wp.array[bool],
    ctx_done_in: wp.array[bool],
    # Data out:
    efc_force_out: wp.array2d[float],
    efc_state_out: wp.array2d[int],
    # Out:
    quad_changed_ids_out: wp.array2d[int],
    quad_changed_count_out: wp.array[int],
    state_changed_count_out: wp.array[int],
  ):
    worldid, efcid = wp.tid()

    if ctx_done_in[worldid]:
      return

    # The linesearch flags worlds whose accepted step was rounding noise (stale
    # ray exhausted); count it as a state change so the fast path rebuilds them.
    if wp.static(TRACK_CHANGES):
      if efcid == 0 and ctx_ls_exhausted_in[worldid]:
        wp.atomic_add(state_changed_count_out, worldid, 1)

    if efcid >= nefc_in[worldid]:
      return

    # Read old state before overwriting
    if wp.static(TRACK_CHANGES):
      old_state = efc_state_out[worldid, efcid]

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
      old_quad = old_state == types.ConstraintState.QUADRATIC.value
      new_quad = new_state == types.ConstraintState.QUADRATIC.value
      if old_quad != new_quad:
        idx = wp.atomic_add(quad_changed_count_out, worldid, 1)
        quad_changed_ids_out[worldid, idx] = efcid
      # LINEARNEG <-> LINEARPOS friction transitions change the force without
      # changing the quadratic flag (or H); the fast path must still see them.
      if old_state != new_state:
        wp.atomic_add(state_changed_count_out, worldid, 1)

  return kernel


@wp.kernel
def _zero_qfrc_constraint_sparse(
  # In:
  state_changed_count_in: wp.array[int],
  ctx_done_in: wp.array[bool],
  # Data out:
  qfrc_constraint_out: wp.array2d[float],
):
  # Only zero worlds the rebuild will repopulate; done worlds keep their value.
  worldid, dofid = wp.tid()

  if ctx_done_in[worldid]:
    return

  if state_changed_count_in[worldid] == 0:
    return

  qfrc_constraint_out[worldid, dofid] = 0.0


@cache_kernel
def _update_constraint_init_qfrc_constraint_sparse(compact: bool):
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=True)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_force_in: wp.array2d[float],
    dof_cdof_in: wp.array2d[int],
    # In:
    state_changed_count_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Data out:
    qfrc_constraint_out: wp.array2d[float],
  ):
    worldid, efcid = wp.tid()

    if ctx_done_in[worldid]:
      return

    if state_changed_count_in[worldid] == 0:
      return

    if efcid >= nefc_in[worldid]:
      return

    force = efc_force_in[worldid, efcid]
    if force == 0.0:
      return

    rownnz = efc_J_rownnz_in[worldid, efcid]
    rowadr = efc_J_rowadr_in[worldid, efcid]
    for i in range(rownnz):
      sparseid = rowadr + i
      colind = efc_J_colind_in[worldid, 0, sparseid]
      if wp.static(COMPACT):
        colind = dof_cdof_in[worldid, colind]
        if colind < 0:
          continue
      efc_J = efc_J_in[worldid, 0, sparseid]
      wp.atomic_add(qfrc_constraint_out[worldid], colind, efc_J * force)

  return kernel


@wp.kernel
def _qfrc_constraint_from_grad(
  # Data in:
  qfrc_smooth_in: wp.array2d[float],
  efc_Ma_in: wp.array2d[float],
  # In:
  ctx_grad_in: wp.array2d[float],
  ctx_grad_scale_in: wp.array[float],
  # Data out:
  qfrc_constraint_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()

  grad = ctx_grad_scale_in[worldid] * ctx_grad_in[worldid, dofid]
  qfrc_constraint_out[worldid, dofid] = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - grad


@cache_kernel
def _update_constraint_init_qfrc_constraint_dense(stable_fast: bool):
  STABLE_FAST = stable_fast

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    efc_J_in: wp.array3d[float],
    efc_force_in: wp.array2d[float],
    njmax_in: int,
    # In:
    state_changed_count_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Data out:
    qfrc_constraint_out: wp.array2d[float],
  ):
    worldid, dofid = wp.tid()

    if ctx_done_in[worldid]:
      return

    # Fast path: stale qfrc_constraint is never read; recovered after the solve.
    if wp.static(STABLE_FAST):
      if state_changed_count_in[worldid] == 0:
        return

    sum_qfrc = float(0.0)
    for efcid in range(min(njmax_in, nefc_in[worldid])):
      efc_J = efc_J_in[worldid, efcid, dofid]
      force = efc_force_in[worldid, efcid]
      sum_qfrc += efc_J * force

    qfrc_constraint_out[worldid, dofid] = sum_qfrc

  return kernel


@wp.kernel
def _update_gradient_h_incremental(
  # Data in:
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  # In:
  quad_changed_ids_in: wp.array2d[int],
  quad_changed_count_in: wp.array[int],
  # Out:
  ctx_h_out: wp.array3d[float],
):
  """Incrementally update upper triangle of H for changed constraints.

  Each thread handles one unique (i, j) element and writes it to the upper triangle.
  For each changed constraint, adds or subtracts D * J[i] * J[j].
  """
  worldid, elementid = wp.tid()

  n_changes = quad_changed_count_in[worldid]
  if n_changes == 0:
    return

  # Upper-triangle enumeration: elementid -> (row, col) where row <= col.
  col = (int(wp.sqrt(float(1 + 8 * elementid))) - 1) // 2
  row = elementid - (col * (col + 1)) // 2

  delta = float(0.0)
  for change_idx in range(n_changes):
    efcid = quad_changed_ids_in[worldid, change_idx]
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


@cache_kernel
def _update_gradient_h_incremental_sparse(compact: bool):
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Data in:
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_state_in: wp.array2d[int],
    dof_cdof_in: wp.array2d[int],
    # In:
    quad_changed_ids_in: wp.array2d[int],
    quad_changed_count_in: wp.array[int],
    slots_per_world: int,
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    """Incrementally update upper triangle of H for changed constraints (sparse J).

    One warp per changed constraint row: the lanes split the row's upper-triangular
    entries (same sqrt triangular-number decode as _JTDACJ_sparse), replacing the
    serial nnz^2 loop that dominated this kernel.
    """
    worldid, slot, lane = wp.tid()

    n_changes = quad_changed_count_in[worldid]
    for change_idx in range(slot, n_changes, slots_per_world):
      efcid = quad_changed_ids_in[worldid, change_idx]
      D = efc_D_in[worldid, efcid]
      sign = float(0.0)
      if efc_state_in[worldid, efcid] == types.ConstraintState.QUADRATIC.value:
        sign = D
      else:
        sign = -D

      rownnz = efc_J_rownnz_in[worldid, efcid]
      rowadr = efc_J_rowadr_in[worldid, efcid]
      n_entries = rownnz * (rownnz + 1) // 2

      for entry in range(lane, n_entries, wp.static(_JTDAJ_THREADS_PER_GROUP)):
        ii = int((wp.sqrt(float(8 * entry + 1)) - 1.0) * 0.5)
        jj = entry - ii * (ii + 1) // 2
        Ji = efc_J_in[worldid, 0, rowadr + ii]
        Jj = efc_J_in[worldid, 0, rowadr + jj]
        h = sign * Ji * Jj
        if h != 0.0:
          colindi = efc_J_colind_in[worldid, 0, rowadr + ii]
          colindj = efc_J_colind_in[worldid, 0, rowadr + jj]
          if wp.static(COMPACT):
            colindi = dof_cdof_in[worldid, colindi]
            colindj = dof_cdof_in[worldid, colindj]
            if colindi < 0 or colindj < 0:
              continue
          wp.atomic_add(ctx_h_out[worldid, wp.min(colindi, colindj)], wp.max(colindi, colindj), h)

  return kernel


def _update_constraint(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext | InverseContext,
  track_changes: bool = False,
  stable_fast: bool = False,
):
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
    ctx.ls_exhausted,
    ctx.done,
  ]

  wp.launch(
    _update_constraint_efc(track_changes),
    dim=(d.nworld, d.njmax),
    inputs=efc_inputs,
    outputs=[d.efc.force, d.efc.state, ctx.quad_changed_ids, ctx.quad_changed_count, ctx.state_changed_count],
  )

  # qfrc_constraint = efc_J.T @ efc_force. Fast-path worlds with no state flips
  # skip the rebuild; the public value is recovered after the solve.
  changed = ctx.state_changed_count if stable_fast else d.nefc
  sc = _sparse_compact(ctx)
  if m.is_sparse or sc:
    dj = ctx.compact_d_full if sc else d
    wp.launch(
      _zero_qfrc_constraint_sparse,
      dim=(d.nworld, m.nv),
      inputs=[changed, ctx.done],
      outputs=[d.qfrc_constraint],
    )
    wp.launch(
      _update_constraint_init_qfrc_constraint_sparse(sc),
      dim=(d.nworld, d.njmax),
      inputs=[d.nefc, dj.efc.J_rownnz, dj.efc.J_rowadr, dj.efc.J_colind, dj.efc.J, d.efc.force, dj.dof_cdof, changed, ctx.done],
      outputs=[d.qfrc_constraint],
    )
  else:
    wp.launch(
      _update_constraint_init_qfrc_constraint_dense(stable_fast),
      dim=(d.nworld, m.nv),
      inputs=[d.nefc, d.efc.J, d.efc.force, d.njmax, changed, ctx.done],
      outputs=[d.qfrc_constraint],
    )


@cache_kernel
def _update_gradient_zero_grad_dot(stable_fast: bool):
  STABLE_FAST = stable_fast

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # In:
    state_changed_count_in: wp.array[int],
    ctx_alpha_in: wp.array[float],
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_grad_dot_out: wp.array[float],
    ctx_newton_decrement_out: wp.array[float],
    ctx_grad_scale_out: wp.array[float],
    ctx_search_unchanged_out: wp.array[bool],
  ):
    worldid = wp.tid()

    if wp.static(STABLE_FAST):
      ctx_search_unchanged_out[worldid] = ctx_done_in[worldid] or state_changed_count_in[worldid] == 0
    else:
      ctx_search_unchanged_out[worldid] = False

    if ctx_done_in[worldid]:
      return

    # Fast path: grad stays stale at its last rebuilt value g. The true
    # gradient is grad_scale * g, and a linesearch step t along the (equally
    # stale) search direction changes it to (grad_scale - t) * g.
    if wp.static(STABLE_FAST):
      if state_changed_count_in[worldid] == 0:
        sigma = ctx_grad_scale_out[worldid]
        new_sigma = sigma - ctx_alpha_in[worldid]
        ratio = float(0.0)
        if sigma != 0.0:
          ratio = new_sigma / sigma
        ratio_sq = ratio * ratio
        ctx_grad_dot_out[worldid] *= ratio_sq
        ctx_newton_decrement_out[worldid] *= ratio_sq
        ctx_grad_scale_out[worldid] = new_sigma
        return

    ctx_grad_dot_out[worldid] = 0.0
    ctx_newton_decrement_out[worldid] = 0.0
    ctx_grad_scale_out[worldid] = 1.0

  return kernel


@cache_kernel
def _update_gradient_grad(stable_fast: bool):
  STABLE_FAST = stable_fast

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Data in:
    qfrc_smooth_in: wp.array2d[float],
    qfrc_constraint_in: wp.array2d[float],
    efc_Ma_in: wp.array2d[float],
    # In:
    state_changed_count_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_grad_out: wp.array2d[float],
    ctx_grad_dot_out: wp.array[float],
  ):
    worldid, dofid = wp.tid()

    if ctx_done_in[worldid]:
      return

    # Fast path: grad stays stale (see _update_gradient_zero_grad_dot).
    if wp.static(STABLE_FAST):
      if state_changed_count_in[worldid] == 0:
        return

    grad = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - qfrc_constraint_in[worldid, dofid]
    ctx_grad_out[worldid, dofid] = grad
    wp.atomic_add(ctx_grad_dot_out, worldid, grad * grad)

  return kernel


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


@cache_kernel
def _update_gradient_init_h_sparse(compact: bool):
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=True)
  def kernel(
    # Model:
    nv: int,
    M_elemid: wp.array2d[int],
    # Data in:
    M_in: wp.array2d[float],
    cdof_dof_in: wp.array2d[int],
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

    if wp.static(COMPACT):
      dof_i = cdof_dof_in[worldid, i]
      dof_j = cdof_dof_in[worldid, j]
      if dof_i < 0 or dof_j < 0:
        # per-world padded block: identity keeps the factorization well conditioned
        ctx_h_out[worldid, i, j] = wp.where(i == j, 1.0, 0.0)
        return
    else:
      dof_i = i
      dof_j = j
      if i >= nv or j >= nv:
        ctx_h_out[worldid, i, j] = 0.0
        return

    # sparse M is stored in the lower triangle, so look up (larger, smaller)
    elemid = M_elemid[wp.max(dof_i, dof_j), wp.min(dof_i, dof_j)]
    if elemid >= 0:
      ctx_h_out[worldid, i, j] = M_in[worldid, elemid]
    else:
      ctx_h_out[worldid, i, j] = 0.0

  return kernel


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
def _update_gradient_JTDAJ_dense_tiled_compact(nv_pad: int, tile_size: int, njmax: int):
  """Compact-path variant of _update_gradient_JTDAJ_dense_tiled.

  Takes M_in as a dense 3D array (nworld, nv_pad, nv_pad) -- the compacted active-DOF
  inertia block cM -- instead of a 2D CSR array.  Cholesky reads fill_mode="upper";
  cM is full-symmetric so the tile_load covers both triangles and the upper triangle
  of the result is correct.
  """
  if njmax < tile_size:
    tile_size = njmax

  TILE_SIZE_K = tile_size

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    M_in: wp.array3d[float],  # kernel_analyzer: ignore; compact dense inertia (nworld, nv_pad, nv_pad)
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_state_in: wp.array2d[int],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    worldid, rank = wp.tid()

    if ctx_done_in[worldid]:
      return

    nefc = nefc_in[worldid]

    # Load full symmetric compact inertia tile (Cholesky only reads upper triangle).
    sum_val = wp.tile_load(M_in[worldid], shape=(wp.static(nv_pad), wp.static(nv_pad)), bounds_check=True)

    for k in range(0, njmax, TILE_SIZE_K):
      if k >= nefc:
        break

      J_kj = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE_K, nv_pad), offset=(k, 0), bounds_check=False)

      D_k = wp.tile_load(efc_D_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
      state = wp.tile_load(efc_state_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)

      D_k = wp.tile_map(_state_check, D_k, state)

      tid_tile = wp.tile_arange(TILE_SIZE_K, dtype=int)
      threshold_tile = wp.tile_ones(shape=TILE_SIZE_K, dtype=int) * (nefc - k)

      active_tile = wp.tile_map(_active_check, tid_tile, threshold_tile)
      D_k = wp.tile_map(wp.mul, active_tile, D_k)

      J_ki = wp.tile_map(wp.mul, wp.tile_transpose(J_kj), wp.tile_broadcast(D_k, shape=(nv_pad, TILE_SIZE_K)))

      sum_val += wp.tile_matmul(J_ki, J_kj)

    wp.tile_store(ctx_h_out[worldid], sum_val, bounds_check=False)

  return kernel


@cache_kernel
def _update_gradient_JTDAJ_dense_tiled(nv_pad: int, tile_size: int, njmax: int, nC: int):
  if njmax < tile_size:
    tile_size = njmax

  TILE_SIZE_K = tile_size

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # Model:
    M_colind: wp.array[int],  # column index of each CSR entry
    M_hinit_i: wp.array[int],  # row index of each CSR entry
    # Data in:
    nefc_in: wp.array[int],
    M_in: wp.array2d[float],  # CSR M (nworld, nC)
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_state_in: wp.array2d[int],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    worldid, rank = wp.tid()

    if ctx_done_in[worldid]:
      return

    nefc = nefc_in[worldid]

    # Densify M's upper triangle from CSR into the shared H tile (Cholesky reads fill_mode="upper").
    # Entry (i, col<=i) of M goes to the upper position (col, i); the lower triangle stays zero.
    m_tile = wp.tile_zeros(shape=(wp.static(nv_pad * nv_pad),), dtype=float, storage="shared")
    # Uniform trip count from the RUNTIME lane count: every lane iterates iters times (so the
    # collective tile_scatter_add is always called by all lanes -- a divergent range(rank, nC, bd)
    # deadlocks). wp.block_dim() is 1 on the CPU backend, so lane 0 then covers every entry.
    lanes = wp.block_dim()
    iters = (nC + lanes - 1) // lanes
    for it in range(iters):
      e = it * lanes + rank
      enable = e < nC
      ec = wp.where(enable, e, 0)
      pos = M_colind[ec] * nv_pad + M_hinit_i[ec]
      wp.tile_scatter_add(m_tile, pos, wp.where(enable, M_in[worldid, ec], 0.0), enable)
    sum_val = wp.tile_reshape(m_tile, (nv_pad, nv_pad))

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


@wp.func
def _elliptic_hessian_entry_from_projections(
  # In:
  dm: float,
  mu_over_t: float,
  mu_n_over_ttt: float,
  tangent_diag: float,
  z01: float,
  z02: float,
  projection1: float,
  projection2: float,
  tangent_dot: float,
) -> float:
  # Contract the diagonal-plus-rank-one curvature without materializing the cone Hessian.
  return dm * (
    z01 * z02
    - mu_over_t * (z01 * projection2 + z02 * projection1)
    + mu_n_over_ttt * projection1 * projection2
    + tangent_diag * tangent_dot
  )


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
    if efcid0 < 0:
      continue
    if efc_state_in[worldid, efcid0] != types.ConstraintState.CONE:
      continue

    fri = contact_friction_in[conid]
    mu = fri[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

    mu2 = mu * mu
    dm = math.safe_div(efc_D_in[worldid, efcid0], mu2 * (1.0 + mu2))

    if dm == 0.0:
      continue

    n = ctx_Jaref_in[worldid, efcid0] * mu
    z01 = mu * efc_J_in[worldid, efcid0, dof1id]
    z02 = mu * efc_J_in[worldid, efcid0, dof2id]
    tt = float(0.0)
    projection1 = float(0.0)
    projection2 = float(0.0)
    tangent_dot = float(0.0)
    for dim in range(1, condim):
      efcid = contact_efc_address_in[conid, dim]
      if efcid >= 0:
        scale = fri[dim - 1]
        u = ctx_Jaref_in[worldid, efcid] * scale
        z1 = scale * efc_J_in[worldid, efcid, dof1id]
        z2 = scale * efc_J_in[worldid, efcid, dof2id]
        tt += u * u
        projection1 += u * z1
        projection2 += u * z2
        tangent_dot += z1 * z2

    t = wp.max(wp.sqrt(tt), types.MJ_MINVAL)
    ttt = wp.max(t * t * t, types.MJ_MINVAL)
    mu_tinv = math.safe_div(mu, t)
    h = _elliptic_hessian_entry_from_projections(
      dm,
      mu_tinv,
      mu * math.safe_div(n, ttt),
      mu2 - n * mu_tinv,
      z01,
      z02,
      projection1,
      projection2,
      tangent_dot,
    )

    ctx_h_out[worldid, dof1id, dof2id] += h


@cache_kernel
def _update_gradient_cholesky(tile_size: int, skip_noflip: bool = False):
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # In:
    ctx_grad_in: wp.array2d[float],
    h_in: wp.array3d[float],
    state_changed_count_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_search_out: wp.array2d[float],
    ctx_search_dot_out: wp.array[float],
    ctx_newton_decrement_out: wp.array[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if ctx_done_in[worldid]:
      return

    # Fast path: skip the solve (see the blocked skip_unchanged variant).
    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    mat_tile = wp.tile_load(h_in[worldid], shape=(TILE_SIZE, TILE_SIZE))
    wp.tile_cholesky_inplace(mat_tile, fill_mode="upper")
    input_tile = wp.tile_load(ctx_grad_in[worldid], shape=TILE_SIZE)
    output_tile = wp.tile_cholesky_solve(mat_tile, input_tile, fill_mode="upper")
    sums = wp.tile_reduce(wp.add, wp.tile_map(solve_search_sums, input_tile, output_tile))[0]
    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]
    wp.tile_store(ctx_search_out[worldid], wp.tile_map(wp.mul, output_tile, -1.0))

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked(tile_size: int, matrix_size: int, vector_size: int):
  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array3d[float],
    ctx_h_in: wp.array3d[float],
    ctx_hfactor: wp.array3d[float],
    # Out:
    ctx_search_out: wp.array3d[float],
    ctx_search_dot_out: wp.array[float],
    ctx_newton_decrement_out: wp.array[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if ctx_done_in[worldid]:
      return

    # We need matrix size both as a runtime input as well as a static input:
    # static input is needed to specify the tile sizes for the compiler
    # runtime input is needed for the loop bounds, otherwise warp will unroll
    # unconditionally leading to shared memory capacity issues.

    sums = wp.static(create_blocked_cholesky_augmented_factorize_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
      ctx_h_in[worldid],
      ctx_grad_in[worldid],
      matrix_size,
      ctx_hfactor[worldid],
      ctx_search_out[worldid],
    )
    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]

  return kernel


@cache_kernel
def _cholesky_factorize_solve_blocked(tile_size: int, matrix_size: int):
  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # In:
    A_in: wp.array3d[float],
    b_in: wp.array3d[float],
    # Out:
    U_out: wp.array3d[float],
    x_out: wp.array3d[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    wp.static(create_blocked_cholesky_factorize_solve_func(TILE_SIZE, matrix_size))(
      A_in[worldid],
      b_in[worldid],
      matrix_size,
      U_out[worldid],
      x_out[worldid],
    )

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked_skip_unchanged(
  tile_size: int, matrix_size: int, vector_size: int, skip_noflip: bool = False
):
  """Blocked Cholesky that skips factorization when no constraints changed."""
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array3d[float],
    ctx_h_in: wp.array3d[float],
    quad_changed_count_in: wp.array[int],
    state_changed_count_in: wp.array[int],
    ctx_hfactor: wp.array3d[float],
    # Out:
    ctx_search_out: wp.array3d[float],
    ctx_search_dot_out: wp.array[float],
    ctx_newton_decrement_out: wp.array[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if ctx_done_in[worldid]:
      return

    # The linesearch is invariant to direction scale, so an unchanged ray can
    # keep its previous search direction.
    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    if quad_changed_count_in[worldid] > 0:
      sums = wp.static(create_blocked_cholesky_augmented_factorize_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
        ctx_h_in[worldid],
        ctx_grad_in[worldid],
        matrix_size,
        ctx_hfactor[worldid],
        ctx_search_out[worldid],
      )
    else:
      sums = wp.static(create_blocked_cholesky_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
        ctx_hfactor[worldid],
        ctx_grad_in[worldid],
        matrix_size,
        ctx_search_out[worldid],
      )

    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]

  return kernel


@wp.kernel(grid_stride=True)
def _padding_h(nv: int, ctx_done_in: wp.array[bool], ctx_h_out: wp.array3d[float]):
  worldid, elementid = wp.tid()

  if ctx_done_in[worldid]:
    return

  dofid = nv + elementid
  ctx_h_out[worldid, dofid, dofid] = 1.0


def _cholesky_factorize_solve(
  m: types.Model, d: types.Data, ctx: SolverContext, skip_unchanged: bool = False, skip_noflip: bool = False
):
  """Cholesky factorize ctx.h and form the Newton search direction.

  If skip_unchanged is True (blocked path only), worlds where no constraints
  changed reuse the cached factorization in hfactor instead of refactorizing.
  """
  if m.nv <= _BLOCK_CHOLESKY_DIM:
    wp.launch_tiled(
      _update_gradient_cholesky(m.nv, skip_noflip),
      dim=d.nworld,
      inputs=[ctx.grad, ctx.h, ctx.state_changed_count if skip_noflip else d.nefc, ctx.done],
      outputs=[ctx.search, ctx.search_dot, ctx.newton_decrement],
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
        _update_gradient_cholesky_blocked_skip_unchanged(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad, m.nv, skip_noflip),
        dim=d.nworld,
        inputs=[
          ctx.done,
          ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)),
          ctx.h,
          ctx.quad_changed_count,
          ctx.state_changed_count if skip_noflip else ctx.quad_changed_count,
          ctx.hfactor,
        ],
        outputs=[
          ctx.search.reshape(shape=(d.nworld, m.nv, 1)),
          ctx.search_dot,
          ctx.newton_decrement,
        ],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )
    else:
      wp.launch_tiled(
        _update_gradient_cholesky_blocked(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad, m.nv),
        dim=d.nworld,
        inputs=[ctx.done, ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)), ctx.h, ctx.hfactor],
        outputs=[
          ctx.search.reshape(shape=(d.nworld, m.nv, 1)),
          ctx.search_dot,
          ctx.newton_decrement,
        ],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )


# ---------------------------------------------------------------------------
# Constraint groups contain consecutive rows with identical sparse support. Each thread group
# accumulates their upper-triangular Hessian block, including elliptic cone curvature.
# ---------------------------------------------------------------------------
_JTDAJ_THREADS_PER_GROUP = 32
_JTDAJ_OVERSUBSCRIBE_WAVES = 6


@cache_kernel
def _JTDACJ_sparse(compact: bool, cone_type: types.ConeType, max_condim: int):
  COMPACT = compact
  ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  MAX_CONDIM = max_condim

  def make_curvature_terms(condim: int):
    @wp.func
    def func(
      # Model:
      opt_impratio_invsqrt: wp.array[float],
      # Data in:
      efc_D_in: wp.array2d[float],
      # In:
      fri: types.vec5,
      ctx_Jaref_in: wp.array2d[float],
      worldid: int,
      efcid0: int,
      block_rows: int,
    ) -> types.vec16:
      mu = fri[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
      mu2 = mu * mu
      dm = math.safe_div(efc_D_in[worldid, efcid0], mu2 * (1.0 + mu2))
      if dm == 0.0:
        return types.vec16()

      n = ctx_Jaref_in[worldid, efcid0] * mu
      terms = types.vec16()
      terms[6] = mu
      tt = float(0.0)
      for dim in range(1, wp.static(condim)):
        if dim < block_rows:
          efcid = efcid0 + dim
          scale = fri[dim - 1]
          u = ctx_Jaref_in[worldid, efcid] * scale
          terms[dim] = u
          terms[6 + dim] = scale
          tt += u * u

      t = wp.max(wp.sqrt(tt), types.MJ_MINVAL)
      ttt = wp.max(t * t * t, types.MJ_MINVAL)
      mu_over_t = math.safe_div(mu, t)
      mu_n_over_ttt = mu * math.safe_div(n, ttt)
      tangent_diag = mu2 - n * mu_over_t

      # Layout: tangent u[1:6], scales[6:12], dm, mu/t, mu*n/t^3, tangent diagonal.
      terms[12] = dm
      terms[13] = mu_over_t
      terms[14] = mu_n_over_ttt
      terms[15] = tangent_diag
      return terms

    return func

  def make_hessian_entry(condim: int):
    @wp.func
    def func(
      # Data in:
      efc_J_in: wp.array3d[float],
      # In:
      terms: Any,
      rowadr: types.vec6i,
      worldid: int,
      pos1: int,
      pos2: int,
    ) -> float:
      z01 = terms[6] * efc_J_in[worldid, 0, rowadr[0] + pos1]
      z02 = terms[6] * efc_J_in[worldid, 0, rowadr[0] + pos2]
      projection1 = float(0.0)
      projection2 = float(0.0)
      tangent_dot = float(0.0)
      for dim in range(1, wp.static(condim)):
        z1 = terms[6 + dim] * efc_J_in[worldid, 0, rowadr[dim] + pos1]
        z2 = terms[6 + dim] * efc_J_in[worldid, 0, rowadr[dim] + pos2]
        projection1 += terms[dim] * z1
        projection2 += terms[dim] * z2
        tangent_dot += z1 * z2

      return _elliptic_hessian_entry_from_projections(
        terms[12],
        terms[13],
        terms[14],
        terms[15],
        z01,
        z02,
        projection1,
        projection2,
        tangent_dot,
      )

    return func

  curvature_terms3 = make_curvature_terms(3)
  curvature_terms4 = make_curvature_terms(4)
  curvature_terms6 = make_curvature_terms(6)
  hessian_entry3 = make_hessian_entry(3)
  hessian_entry4 = make_hessian_entry(4)
  hessian_entry6 = make_hessian_entry(6)

  @wp.func
  def curvature_terms(
    # Model:
    opt_impratio_invsqrt: wp.array[float],
    # Data in:
    efc_D_in: wp.array2d[float],
    # In:
    fri: types.vec5,
    ctx_Jaref_in: wp.array2d[float],
    worldid: int,
    efcid0: int,
    condim: int,
    block_rows: int,
  ) -> types.vec16:
    if wp.static(MAX_CONDIM == 3):
      return curvature_terms3(opt_impratio_invsqrt, efc_D_in, fri, ctx_Jaref_in, worldid, efcid0, block_rows)

    if condim == 3:
      return curvature_terms3(opt_impratio_invsqrt, efc_D_in, fri, ctx_Jaref_in, worldid, efcid0, block_rows)
    if wp.static(MAX_CONDIM == 4):
      return curvature_terms4(opt_impratio_invsqrt, efc_D_in, fri, ctx_Jaref_in, worldid, efcid0, block_rows)
    if condim == 4:
      return curvature_terms4(opt_impratio_invsqrt, efc_D_in, fri, ctx_Jaref_in, worldid, efcid0, block_rows)
    return curvature_terms6(opt_impratio_invsqrt, efc_D_in, fri, ctx_Jaref_in, worldid, efcid0, block_rows)

  @wp.func
  def hessian_entry(
    # Data in:
    efc_J_in: wp.array3d[float],
    # In:
    terms: Any,
    rowadr: types.vec6i,
    worldid: int,
    pos1: int,
    pos2: int,
    condim: int,
  ) -> float:
    if wp.static(MAX_CONDIM == 3):
      return hessian_entry3(efc_J_in, terms, rowadr, worldid, pos1, pos2)

    if condim == 3:
      return hessian_entry3(efc_J_in, terms, rowadr, worldid, pos1, pos2)
    if wp.static(MAX_CONDIM == 4):
      return hessian_entry4(efc_J_in, terms, rowadr, worldid, pos1, pos2)
    if condim == 4:
      return hessian_entry4(efc_J_in, terms, rowadr, worldid, pos1, pos2)
    return hessian_entry6(efc_J_in, terms, rowadr, worldid, pos1, pos2)

  @wp.kernel(module="unique", enable_backward=False, grid_stride=True)
  def kernel(
    # Model:
    opt_impratio_invsqrt: wp.array[float],
    # Data in:
    contact_friction_in: wp.array[types.vec5],
    contact_dim_in: wp.array[int],
    efc_id_in: wp.array2d[int],
    efc_jtdaj_adr_in: wp.array2d[int],
    efc_jtdaj_nrow_in: wp.array2d[int],
    efc_jtdaj_nblock_in: wp.array[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_state_in: wp.array2d[int],
    dof_cdof_in: wp.array2d[int],
    # In:
    ctx_Jaref_in: wp.array2d[float],
    ctx_done_in: wp.array[bool],
    groups_per_world: int,
    # Out:
    h_out: wp.array3d[float],
  ):
    worldid, slot, lane = wp.tid()
    if wp.static(ELLIPTIC):
      lanes = wp.block_dim()
    else:
      lanes = wp.static(_JTDAJ_THREADS_PER_GROUP)
    if ctx_done_in[worldid]:
      return
    count = efc_jtdaj_nblock_in[worldid]
    for groupid in range(slot, count, groups_per_world):
      head_row = efc_jtdaj_adr_in[worldid, groupid]
      block_rows = efc_jtdaj_nrow_in[worldid, groupid]
      head_adr = efc_J_rowadr_in[worldid, head_row]
      support = efc_J_rownnz_in[worldid, head_row]
      n_entries = support * (support + 1) // 2

      is_cone = False
      if wp.static(ELLIPTIC):
        is_cone = efc_state_in[worldid, head_row] == types.ConstraintState.CONE
        condim = int(0)
        # Clipped cone rows retain the safe head address and a zero scale.
        cone_rowadr = types.vec6i(head_adr, head_adr, head_adr, head_adr, head_adr, head_adr)
        if is_cone:
          conid = efc_id_in[worldid, head_row]
          if wp.static(MAX_CONDIM == 3):
            condim = int(3)
          else:
            condim = contact_dim_in[conid]
          for dim in range(1, wp.static(MAX_CONDIM)):
            if dim < block_rows:
              cone_rowadr[dim] = head_adr + dim * support
          local_terms = types.vec16()
          if lane == 0:
            local_terms = curvature_terms(
              opt_impratio_invsqrt,
              efc_D_in,
              contact_friction_in[conid],
              ctx_Jaref_in,
              worldid,
              head_row,
              condim,
              block_rows,
            )
          cone_terms = wp.tile_zeros(shape=(16,), dtype=float, storage="shared")
          for term_id in range(1, 16):
            wp.tile_scatter_masked(cone_terms, term_id, local_terms[term_id], lane == 0)

      for entry in range(lane, n_entries, lanes):
        block_col = int((wp.sqrt(float(8 * entry + 1)) - 1.0) * 0.5)
        block_row = entry - block_col * (block_col + 1) // 2
        dof_row = efc_J_colind_in[worldid, 0, head_adr + block_row]
        dof_col = efc_J_colind_in[worldid, 0, head_adr + block_col]
        hval = float(0.0)
        if wp.static(ELLIPTIC):
          if is_cone:
            hval = hessian_entry(
              efc_J_in,
              cone_terms,
              cone_rowadr,
              worldid,
              block_row,
              block_col,
              condim,
            )
        if not is_cone:
          for member in range(block_rows):
            member_row = head_row + member
            if efc_state_in[worldid, member_row] == types.ConstraintState.QUADRATIC.value:
              member_adr = efc_J_rowadr_in[worldid, member_row]
              j_row = efc_J_in[worldid, 0, member_adr + block_row]
              j_col = efc_J_in[worldid, 0, member_adr + block_col]
              hval += j_row * efc_D_in[worldid, member_row] * j_col
        if hval != 0.0:
          if wp.static(COMPACT):
            dof_row = dof_cdof_in[worldid, dof_row]
            dof_col = dof_cdof_in[worldid, dof_col]
            if dof_row < 0 or dof_col < 0:
              continue
          wp.atomic_add(h_out[worldid, wp.min(dof_row, dof_col)], wp.max(dof_row, dof_col), hval)

  return kernel


def _jtdaj_groups_per_world(nworld: int, njmax: int) -> int:
  # njmax is capacity and often mostly empty, so cap slots at a few resident waves.
  block_size, min_grid_size = wp.get_suggested_block_size(_JTDACJ_sparse(False, types.ConeType.PYRAMIDAL, 3))
  device_warps = max(1, block_size * min_grid_size // _JTDAJ_THREADS_PER_GROUP)
  return max(1, min(njmax, _JTDAJ_OVERSUBSCRIBE_WAVES * device_warps // nworld))


def _update_gradient(m: types.Model, d: types.Data, ctx: SolverContext, compact: bool = False):
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
    wp.launch(
      _update_gradient_zero_grad_dot(False),
      dim=d.nworld,
      inputs=[d.nefc, ctx.alpha, ctx.done],
      outputs=[ctx.grad_dot, ctx.newton_decrement, ctx.grad_scale, ctx.search_unchanged],
    )
    wp.launch(
      _update_gradient_grad(False),
      dim=(d.nworld, m.nv),
      inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, d.nefc, ctx.done],
      outputs=[ctx.grad, ctx.grad_dot],
    )

  if m.opt.solver == types.SolverType.CG:
    smooth.solve_m(m, d, ctx.Mgrad, ctx.grad)
  elif m.opt.solver == types.SolverType.NEWTON:
    # h = M + (efc_J.T * efc_D * active) @ efc_J
    sc = _sparse_compact(ctx)
    if m.is_sparse or sc:
      mj = ctx.compact_m_full if sc else m
      dj = ctx.compact_d_full if sc else d
      wp.launch(
        _update_gradient_init_h_sparse(sc),
        dim=(d.nworld, m.nv_pad, m.nv_pad),
        inputs=[mj.nv, mj.M_elemid, dj.M, dj.cdof_dof, ctx.done],
        outputs=[ctx.h],
      )

      groups_per_world = _jtdaj_groups_per_world(d.nworld, d.njmax)
      max_condim = 3
      if m.opt.cone == types.ConeType.ELLIPTIC and m.nmaxcondim > 3:
        max_condim = int(m.nmaxcondim)
      jtdaj_kernel = _JTDACJ_sparse(sc, m.opt.cone, max_condim)
      jtdaj_inputs = [
        m.opt.impratio_invsqrt,
        d.contact.friction,
        d.contact.dim,
        d.efc.id,
        dj.efc.jtdaj_adr,
        dj.efc.jtdaj_nrow,
        dj.efc.jtdaj_nblock,
        dj.efc.J_rownnz,
        dj.efc.J_rowadr,
        dj.efc.J_colind,
        dj.efc.J,
        d.efc.D,
        d.efc.state,
        dj.dof_cdof,
        ctx.Jaref,
        ctx.done,
        groups_per_world,
      ]
      elliptic = m.opt.cone == types.ConeType.ELLIPTIC
      threads_per_group = 1 if elliptic and wp.get_device().is_cpu else _JTDAJ_THREADS_PER_GROUP
      block_dim = threads_per_group if elliptic else mj.block_dim.update_gradient_JTDAJ_sparse
      wp.launch(
        jtdaj_kernel,
        dim=(d.nworld, groups_per_world, threads_per_group),
        inputs=jtdaj_inputs,
        outputs=[ctx.h],
        block_dim=block_dim,
      )
    else:
      if compact:
        # compact path: d.M is the dense 3D compact inertia block (nworld, nv_pad, nv_pad)
        wp.launch_tiled(
          _update_gradient_JTDAJ_dense_tiled_compact(m.nv_pad, types.TILE_SIZE_JTDAJ_DENSE, d.njmax),
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
      else:
        wp.launch_tiled(
          _update_gradient_JTDAJ_dense_tiled(m.nv_pad, types.TILE_SIZE_JTDAJ_DENSE, d.njmax, m.M_colind.shape[0]),
          dim=d.nworld,
          inputs=[
            m.M_colind,
            m.M_hinit_i,
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

    if m.opt.cone == types.ConeType.ELLIPTIC and not (m.is_sparse or sc):
      # Optimization: launching update_gradient_JTCJ with limited number of blocks on a GPU.
      # Profiling suggests that only a fraction of blocks out of the original
      # d.njmax blocks do the actual work. It aims to minimize #CTAs with no
      # effective work. It launches with #blocks that's proportional to the number
      # of SMs on the GPU. We can now query the SM count:
      # https://github.com/NVIDIA/warp/commit/f3814e7e5459e5fd13032cf0fddb3daddd510f30

      if wp.get_device().is_cuda:
        sm_count = wp.get_device().sm_count

        # Here we assume one block has 256 threads. We use a factor of 6, which
        # can be changed in the future to fine-tune the perf. The optimal factor will
        # depend on the kernel's occupancy, which determines how many blocks can
        # simultaneously run on the SM. TODO: This factor can be tuned further.
        dim_block = ceil((sm_count * 6 * 256) / m.dof_tri_row.size)
      else:
        # fall back for CPU
        dim_block = d.naconmax

      nblocks_perblock = int((d.naconmax + dim_block - 1) / dim_block)

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


def _update_gradient_incremental(m: types.Model, d: types.Data, ctx: SolverContext, stable_fast: bool = False):
  """Incremental gradient update: update H for changed constraints + re-factorize.

  Skips the full J^T*D*J rebuild by applying only the delta from constraints
  that changed QUADRATIC state, then re-factorizes and solves.
  """
  changed = ctx.state_changed_count if stable_fast else d.nefc
  wp.launch(
    _update_gradient_zero_grad_dot(stable_fast),
    dim=d.nworld,
    inputs=[changed, ctx.alpha, ctx.done],
    outputs=[ctx.grad_dot, ctx.newton_decrement, ctx.grad_scale, ctx.search_unchanged],
  )

  wp.launch(
    _update_gradient_grad(stable_fast),
    dim=(d.nworld, m.nv),
    inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, changed, ctx.done],
    outputs=[ctx.grad, ctx.grad_dot],
  )

  # Update upper triangle of H with delta from changed constraints.
  sc = _sparse_compact(ctx)
  if m.is_sparse or sc:
    dj = ctx.compact_d_full if sc else d
    slots = _jtdaj_groups_per_world(d.nworld, ctx.quad_changed_ids.shape[1])
    wp.launch(
      _update_gradient_h_incremental_sparse(sc),
      dim=(d.nworld, slots, _JTDAJ_THREADS_PER_GROUP),
      inputs=[
        dj.efc.J_rownnz,
        dj.efc.J_rowadr,
        dj.efc.J_colind,
        dj.efc.J,
        d.efc.D,
        d.efc.state,
        dj.dof_cdof,
        ctx.quad_changed_ids,
        ctx.quad_changed_count,
        slots,
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
        ctx.quad_changed_ids,
        ctx.quad_changed_count,
      ],
      outputs=[ctx.h],
    )

  _cholesky_factorize_solve(m, d, ctx, skip_unchanged=True, skip_noflip=stable_fast)


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


@cache_kernel
def _solve_cg_finalize(warn_overflow: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
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
    overflow_out: wp.array[int],
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
      if not done and solver_niter_out[worldid] == opt_iterations:
        if wp.static(warn_overflow):
          wp.printf("solver iterations limit reached - please increase iterations to %u\n", opt_iterations)
        overflow_out[worldid] = overflow_out[worldid] | OverflowType.ITERATIONS
      ctx_done_out[worldid] = True
      wp.atomic_add(nsolving_out, 0, -1)

  return kernel


@cache_kernel
def _solve_done(warn_overflow: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_tolerance: wp.array[float],
    opt_iterations: int,
    stat_meaninertia: wp.array[float],
    # In:
    ctx_grad_dot_in: wp.array[float],
    ctx_newton_decrement_in: wp.array[float],
    ctx_improvement_in: wp.array[float],
    ctx_done_in: wp.array[bool],
    # Data out:
    solver_niter_out: wp.array[int],
    overflow_out: wp.array[int],
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
    model_improvement = _rescale(nv, meaninertia, 0.5 * ctx_newton_decrement_in[worldid])
    done = (improvement < tolerance) or (gradient < tolerance) or (model_improvement < tolerance)
    if done or solver_niter_out[worldid] == opt_iterations:
      # if the solver has converged or the maximum number of iterations has been reached then
      # mark this world as done and remove it from the number of unconverged worlds
      if not done and solver_niter_out[worldid] == opt_iterations:
        if wp.static(warn_overflow):
          wp.printf("solver iterations limit reached - please increase iterations to %u\n", opt_iterations)
        overflow_out[worldid] = overflow_out[worldid] | OverflowType.ITERATIONS
      ctx_done_out[worldid] = True
      wp.atomic_add(nsolving_out, 0, -1)

  return kernel


# The linesearch runs in ray units anchored at the last gradient rebuild, and
# its bracketing arithmetic cannot resolve steps below ~eps times the gross
# derivative-term magnitude over the curvature (see _linesearch_iterative_kernel).
# A world whose tolerance sits below that floor could never terminate on the
# stale ray; rebuilding re-anchors the arithmetic at the current (much smaller)
# gradient scale. The 8 is an allowance for the reduction depth of the sums.
_ALPHA_NOISE_EPS = 8.0 * 1.1920929e-07  # 8 * float32 eps


def _use_incremental(m: types.Model) -> bool:
  """Whether constraint state changes are tracked for incremental H updates."""
  return m.opt.solver == types.SolverType.NEWTON and m.opt.cone != types.ConeType.ELLIPTIC


@wp.kernel(grid_stride=True)
def _zero_change_counters(
  # Out:
  quad_changed_count_out: wp.array[int],
  state_changed_count_out: wp.array[int],
):
  worldid = wp.tid()
  quad_changed_count_out[worldid] = 0
  state_changed_count_out[worldid] = 0


@event_scope
def _solver_iteration(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext,
  nsolving: wp.array[int],
  compact: bool = False,
):
  _linesearch(m, d, ctx)

  # Incremental H is only valid for non-elliptic cones. The elliptic cone
  # path in _update_constraint_efc has early returns that skip state change
  # tracking, and the additional JTCJ Hessian term depends on Jaref which
  # changes every iteration.
  incremental = _use_incremental(m)

  if incremental:
    # Must complete before _update_constraint_efc which atomically increments.
    wp.launch(
      _zero_change_counters,
      dim=d.nworld,
      outputs=[ctx.quad_changed_count, ctx.state_changed_count],
    )

  # The tracking also enables the stable-state fast path: worlds with no state
  # flips this iteration were exactly quadratic over the step, so grad/search
  # only changed by a scalar along the same ray. Skip their qfrc/grad/
  # solve/search updates and track the scalar in ctx.grad_scale.
  _update_constraint(m, d, ctx, track_changes=incremental, stable_fast=incremental)

  if incremental:
    _update_gradient_incremental(m, d, ctx, stable_fast=incremental)
  else:
    _update_gradient(m, d, ctx, compact=compact)

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
      _solve_cg_finalize(bool(m.opt.warn_overflow)),
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
        d.overflow,
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
    wp.launch(
      _solve_done(bool(m.opt.warn_overflow)),
      dim=d.nworld,
      inputs=[
        m.nv,
        m.opt.tolerance,
        m.opt.iterations,
        m.stat.meaninertia,
        ctx.grad_dot,
        ctx.newton_decrement,
        ctx.improvement,
        ctx.done,
      ],
      outputs=[d.solver_niter, d.overflow, nsolving, ctx.done],
    )


def init_context(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, grad: bool = True, compact: bool = False):
  # initialize some efc arrays
  wp.launch(
    _solve_init_efc,
    dim=d.nworld,
    outputs=[d.solver_niter, ctx.search_dot, ctx.done],
  )

  # jaref = d.efc_J @ d.qacc - d.efc_aref

  # if we are only using 1 thread, it makes sense to do more dofs as we can also skip the
  # init kernel. For more than 1 thread, dofs_per_thread is lower for better load balancing.

  if m.is_sparse:
    # Sparse J has few nonzeros per row, one thread handles them all.
    dofs_per_thread = m.nv
    threads_per_efc = 1
  elif m.nv > 50:
    dofs_per_thread = 20
    threads_per_efc = ceil(m.nv / dofs_per_thread)
  else:
    dofs_per_thread = 50
    threads_per_efc = ceil(m.nv / dofs_per_thread)
  # we need to clear the jaref array if we're doing atomic adds.
  if threads_per_efc > 1:
    ctx.Jaref.zero_()

  sc = _sparse_compact(ctx)
  dj = ctx.compact_d_full if sc else d
  if sc:
    dofs_per_thread = m.nv
    threads_per_efc = 1
  wp.launch(
    _solve_init_jaref_kernel(sc or m.is_sparse, m.nv, dofs_per_thread, sc),
    dim=(d.nworld, d.njmax, threads_per_efc),
    inputs=[d.nefc, d.qacc, dj.efc.J_rownnz, dj.efc.J_rowadr, dj.efc.J_colind, dj.efc.J, d.efc.aref, dj.dof_cdof],
    outputs=[ctx.Jaref],
  )

  # Ma = M @ qacc
  _mul_m_compact_aware(m, d, ctx, d.efc.Ma, d.qacc, ctx.done)

  _update_constraint(m, d, ctx)

  if grad:
    _update_gradient(m, d, ctx, compact=compact)


@event_scope
def solve(m: types.Model, d: types.Data):
  if m.opt.enableflags & types.EnableBit.SLEEP:
    # Self-contained like the island branch below: rebuild the active-DOF mapping from
    # tree_awake so solve() works when called directly (not only via fwd_acceleration).
    island.update_active_dofs(m, d)
    solve_compact(m, d)
    if m.ntree > 1:
      island.compute_island_mapping(m, d)
    return

  if d.njmax == 0 or m.nv == 0:
    wp.copy(d.qacc, d.qacc_smooth)
    d.solver_niter.fill_(0)
  else:
    ctx = _create_solver_context(m, d)
    _solve(m, d, ctx)


def _solve(m: types.Model, d: types.Data, ctx: SolverContext, compact: bool = False):
  """Finds forces that satisfy constraints."""
  warmstart = not (m.opt.disableflags & types.DisableBit.WARMSTART)
  wp.launch(
    _solve_init_dof(warmstart, m.is_sparse),
    dim=(d.nworld, m.nv),
    inputs=[d.nefc, d.qacc_warmstart, d.qacc_smooth],
    outputs=[d.qacc, d.qfrc_constraint],
  )

  #  context
  init_context(m, d, ctx, grad=True, compact=compact)

  if _use_incremental(m):
    # A new solve computes a new search direction: invalidate the mv/jv reuse
    # left over from the previous solve.
    ctx.search_unchanged.zero_()

  # CG search = -Mgrad
  if m.opt.solver == types.SolverType.CG:
    wp.launch_tiled(
      _solve_init_search_cg_tiled,
      dim=d.nworld,
      inputs=[m.nv, ctx.grad, ctx.Mgrad],
      outputs=[ctx.search, ctx.search_dot, ctx.prev_grad, ctx.prev_Mgrad],
      block_dim=m.block_dim.solve_init_search_cg,
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
    wp.capture_while(nsolving, while_body=_solver_iteration, m=m, d=d, ctx=ctx, nsolving=nsolving, compact=compact)
  else:
    # This branch is mostly for when JAX is used as it is currently not compatible
    # with CUDA graph conditional.
    # It should be removed when JAX becomes compatible.
    for _ in range(m.opt.iterations):
      _solver_iteration(m, d, ctx, nsolving, compact=compact)

  # Recover qfrc_constraint (the compacted buffer when run under solve_compact):
  # the fast path leaves it stale, and the per-iteration zeroing wiped it for
  # worlds that converged early.
  if _use_incremental(m):
    wp.launch(
      _qfrc_constraint_from_grad,
      dim=(d.nworld, m.nv),
      inputs=[d.qfrc_smooth, d.efc.Ma, ctx.grad, ctx.grad_scale],
      outputs=[d.qfrc_constraint],
    )


# Active-DOF compaction solve (nvmax < nv).
#
# When fewer than nv DOFs are active, the active set is packed into a contiguous
# [0, ncdof) range (see island.update_active_dofs) and the dense factor/solve runs at
# size nvmax_pad instead of nv. The compacted workspace lives on Data as c* fields
# (mirroring the island-local i* fields). The constrained solve gathers into compacted
# arrays, shallow-replaces (m, d) so the stock dense Newton solver runs at nvmax_pad,
# then scatters qacc/qfrc_constraint back, freezing inactive DOFs to 0.


@wp.kernel
def _init_compact_inertia(
  # Data in:
  ncdof_in: wp.array[int],
  # Out:
  M_c_out: wp.array3d[float],
):
  worldid, i, j = wp.tid()
  val = 0.0
  if i == j and i >= ncdof_in[worldid]:
    val = 1.0
  M_c_out[worldid, i, j] = val


@wp.kernel
def _mul_m_sparse_compact(
  # Model:
  M_mulm_rowadr: wp.array[int],
  M_mulm_col: wp.array[int],
  M_mulm_madr: wp.array[int],
  # Data in:
  M_in: wp.array2d[float],
  dof_cdof_in: wp.array2d[int],
  cdof_dof_in: wp.array2d[int],
  # In:
  vec: wp.array2d[float],
  skip: wp.array[bool],
  # Out:
  res: wp.array2d[float],
):
  """Compacted res = M @ vec via the full-coordinate sparse M (no gathered cM)."""
  worldid, ci = wp.tid()

  if skip[worldid]:
    return

  dof = cdof_dof_in[worldid, ci]
  if dof < 0:
    res[worldid, ci] = 0.0
    return

  acc = float(0.0)
  start = M_mulm_rowadr[dof]
  end = M_mulm_rowadr[dof + 1]
  for k in range(start, end):
    # tree-internal columns are awake with the row; guard like _gather_M_sparse
    # in case M ever carries cross-tree entries (their gathered value is zero)
    cj = dof_cdof_in[worldid, M_mulm_col[k]]
    if cj >= 0:
      acc += M_in[worldid, M_mulm_madr[k]] * vec[worldid, cj]
  res[worldid, ci] = acc


def _sparse_compact(ctx: SolverContext | InverseContext) -> bool:
  """Whether this solve is compact over a sparse full model (full J structures exist)."""
  return ctx.compact_d_full is not None and ctx.compact_m_full.is_sparse


def _mul_m_compact_aware(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, res, vec, skip):
  """M @ vec: full-coordinate sparse walk under compact, support.mul_m natively."""
  dfull = ctx.compact_d_full
  if dfull is not None:
    mfull = ctx.compact_m_full
    wp.launch(
      _mul_m_sparse_compact,
      dim=(d.nworld, m.nv),
      inputs=[
        mfull.M_mulm_rowadr,
        mfull.M_mulm_col,
        mfull.M_mulm_madr,
        dfull.M,
        dfull.dof_cdof,
        dfull.cdof_dof,
        vec,
        skip,
      ],
      outputs=[res],
    )
  else:
    support.mul_m(m, d, res, vec, skip=skip)


@wp.kernel
def _gather_M_sparse(
  # Model:
  M_rownnz: wp.array[int],
  M_rowadr: wp.array[int],
  M_colind: wp.array[int],
  # Data in:
  M_in: wp.array2d[float],
  dof_cdof_in: wp.array2d[int],
  # Out:
  M_c_out: wp.array3d[float],
):
  worldid, i = wp.tid()
  ci = dof_cdof_in[worldid, i]
  if ci < 0:
    return
  rowadr = M_rowadr[i]
  for k in range(M_rownnz[i]):
    adr = rowadr + k
    cj = dof_cdof_in[worldid, M_colind[adr]]
    if cj >= 0:
      val = M_in[worldid, adr]
      M_c_out[worldid, ci, cj] = val
      M_c_out[worldid, cj, ci] = val


@wp.kernel
def _gather_rhs_compact(
  # Data in:
  cdof_dof_in: wp.array2d[int],
  # In:
  vec_in: wp.array2d[float],
  # Out:
  rhs_out: wp.array3d[float],
):
  worldid, ci = wp.tid()
  dof = cdof_dof_in[worldid, ci]
  if dof >= 0:
    rhs_out[worldid, ci, 0] = vec_in[worldid, dof]
  else:
    rhs_out[worldid, ci, 0] = 0.0


@wp.kernel
def _scatter_solution(
  # Data in:
  dof_cdof_in: wp.array2d[int],
  # In:
  x_in: wp.array3d[float],
  # Out:
  vec_out: wp.array2d[float],
):
  worldid, i = wp.tid()
  ci = dof_cdof_in[worldid, i]
  if ci >= 0:
    vec_out[worldid, i] = x_in[worldid, ci, 0]
  else:
    vec_out[worldid, i] = 0.0  # frozen inactive DOF


@event_scope
def smooth_solve_compact(m: types.Model, d: types.Data):
  """Compacted equivalent of solve_m: qacc_smooth[active] = cM^-1 qfrc_smooth[active].

  Inactive DOFs are frozen (qacc_smooth set to 0). Reads the sparse Model inertia.
  """
  wp.launch(
    _init_compact_inertia,
    dim=(d.nworld, d.nvmax_pad, d.nvmax_pad),
    inputs=[d.ncdof],
    outputs=[d.cM],
  )
  wp.launch(
    _gather_M_sparse,
    dim=(d.nworld, m.nv),
    inputs=[m.M_rownnz, m.M_rowadr, m.M_colind, d.M, d.dof_cdof],
    outputs=[d.cM],
  )
  wp.launch(
    _gather_rhs_compact,
    dim=(d.nworld, d.nvmax_pad),
    inputs=[d.cdof_dof, d.qfrc_smooth],
    outputs=[d.crhs],
  )
  wp.launch_tiled(
    _cholesky_factorize_solve_blocked(types.TILE_SIZE_JTDAJ_DENSE, d.nvmax_pad),
    dim=d.nworld,
    inputs=[d.cM, d.crhs],
    outputs=[d.cqLD, d.cx],
    block_dim=m.block_dim.update_gradient_cholesky_blocked,
  )
  wp.launch(_scatter_solution, dim=(d.nworld, m.nv), inputs=[d.dof_cdof, d.cx], outputs=[d.qacc_smooth])


@wp.kernel
def _gather_dof_vecs_compact(
  # Data in:
  qacc_warmstart_in: wp.array2d[float],
  qfrc_smooth_in: wp.array2d[float],
  qacc_smooth_in: wp.array2d[float],
  cdof_dof_in: wp.array2d[int],
  # Out:
  qfrc_smooth_c_out: wp.array2d[float],
  qacc_smooth_c_out: wp.array2d[float],
  qacc_warmstart_c_out: wp.array2d[float],
):
  worldid, ci = wp.tid()
  dof = cdof_dof_in[worldid, ci]
  if dof >= 0:
    qfrc_smooth_c_out[worldid, ci] = qfrc_smooth_in[worldid, dof]
    qacc_smooth_c_out[worldid, ci] = qacc_smooth_in[worldid, dof]
    qacc_warmstart_c_out[worldid, ci] = qacc_warmstart_in[worldid, dof]
  else:
    qfrc_smooth_c_out[worldid, ci] = 0.0
    qacc_smooth_c_out[worldid, ci] = 0.0
    qacc_warmstart_c_out[worldid, ci] = 0.0


@wp.kernel
def _scatter_dof_vecs(
  # Data in:
  dof_cdof_in: wp.array2d[int],
  # In:
  qacc_c_in: wp.array2d[float],
  qfrc_constraint_c_in: wp.array2d[float],
  # Data out:
  qacc_out: wp.array2d[float],
  qfrc_constraint_out: wp.array2d[float],
):
  worldid, i = wp.tid()
  ci = dof_cdof_in[worldid, i]
  if ci >= 0:
    qacc_out[worldid, i] = qacc_c_in[worldid, ci]
    qfrc_constraint_out[worldid, i] = qfrc_constraint_c_in[worldid, ci]
  else:
    qacc_out[worldid, i] = 0.0
    qfrc_constraint_out[worldid, i] = 0.0


@wp.kernel
def _gather_J_sparse(
  # Data in:
  nefc_in: wp.array[int],
  dof_cdof_in: wp.array2d[int],
  # In:
  J_rownnz_in: wp.array2d[int],
  J_rowadr_in: wp.array2d[int],
  J_colind_in: wp.array3d[int],
  J_in: wp.array3d[float],
  # Out:
  J_c_out: wp.array3d[float],
):
  worldid, efcid = wp.tid()
  if efcid >= nefc_in[worldid]:
    return
  rowadr = J_rowadr_in[worldid, efcid]
  for k in range(J_rownnz_in[worldid, efcid]):
    adr = rowadr + k
    cj = dof_cdof_in[worldid, J_colind_in[worldid, 0, adr]]
    if cj >= 0:
      J_c_out[worldid, efcid, cj] = J_in[worldid, 0, adr]


@wp.kernel
def _gather_J_dense(
  # Data in:
  nefc_in: wp.array[int],
  dof_cdof_in: wp.array2d[int],
  # In:
  J_in: wp.array3d[float],
  # Out:
  J_c_out: wp.array3d[float],
):
  worldid, efcid = wp.tid()
  if efcid >= nefc_in[worldid]:
    return
  nv = dof_cdof_in.shape[1]
  for j in range(nv):
    cj = dof_cdof_in[worldid, j]
    if cj >= 0:
      J_c_out[worldid, efcid, cj] = J_in[worldid, efcid, j]


@event_scope
def solve_compact(m: types.Model, d: types.Data):
  """Run the dense Newton constraint solver in compacted DOF space.

  Gathers the active-DOF inertia, constraint Jacobian, and smooth/warmstart vectors
  into nvmax_pad-sized dense workspaces, runs the stock dense Newton solver on a
  shallow-replaced (m, d) at nvmax_pad, then scatters qacc/qfrc_constraint back.
  Inactive DOFs are frozen to 0. On the incremental Newton path the solver
  kernels read the sparse M and J directly through the compaction maps.
  """
  _compact_gather(m, d)

  # shallow-replace (m, d) so the stock dense Newton solver runs at nvmax_pad.
  # Keep graph-conditional early-exit on CUDA (matches baseline: stops at convergence
  # instead of running all iterations); fall back to the plain loop on CPU.
  nvp = d.nvmax_pad
  gc = m.opt.graph_conditional and wp.get_device().is_cuda
  opt2 = dataclasses.replace(m.opt, graph_conditional=gc, tolerance=d.ctol, ls_tolerance=d.cls_tol)
  m2 = dataclasses.replace(
    m, opt=opt2, nv=nvp, nv_pad=nvp, is_sparse=False, dof_tri_row=d.cdof_tri_row, dof_tri_col=d.cdof_tri_col
  )
  efc2 = dataclasses.replace(d.efc, J=d.cJ, Ma=d.cMa)
  d2 = dataclasses.replace(
    d,
    M=d.cM,
    qfrc_smooth=d.cqfrc_smooth,
    qacc_smooth=d.cqacc_smooth,
    qacc_warmstart=d.cqacc_warmstart,
    qacc=d.cqacc,
    qfrc_constraint=d.cqfrc_constraint,
    efc=efc2,
  )

  sctx = _create_solver_context(m2, d2)
  # compact kernels read the full-coordinate sparse structures (M, J) through
  # the compaction maps instead of dense products on gathered blocks
  sctx.compact_m_full = m
  sctx.compact_d_full = d
  _solve(m2, d2, sctx, compact=True)

  _compact_scatter(m, d)


@event_scope
def _compact_gather(m: types.Model, d: types.Data):
  nvp = d.nvmax_pad
  # gather compacted dense inertia and Jacobian only for dense models;
  # sparse models read sparse M and J directly through compaction maps
  if not m.is_sparse:
    wp.launch(
      _init_compact_inertia,
      dim=(d.nworld, nvp, nvp),
      inputs=[d.ncdof],
      outputs=[d.cM],
    )
    wp.launch(
      _gather_M_sparse,
      dim=(d.nworld, m.nv),
      inputs=[m.M_rownnz, m.M_rowadr, m.M_colind, d.M, d.dof_cdof],
      outputs=[d.cM],
    )
    d.cJ.zero_()
    wp.launch(
      _gather_J_dense,
      dim=(d.nworld, d.njmax),
      inputs=[d.nefc, d.dof_cdof, d.efc.J],
      outputs=[d.cJ],
    )
  # gather compacted DOF-space vectors in a single launch
  wp.launch(
    _gather_dof_vecs_compact,
    dim=(d.nworld, nvp),
    inputs=[
      d.qacc_warmstart,
      d.qfrc_smooth,
      d.qacc_smooth,
      d.cdof_dof,
    ],
    outputs=[
      d.cqfrc_smooth,
      d.cqacc_smooth,
      d.cqacc_warmstart,
    ],
  )


@event_scope
def _compact_scatter(m: types.Model, d: types.Data):
  # scatter results back to full DOF space (inactive frozen to 0) in one launch
  wp.launch(
    _scatter_dof_vecs,
    dim=(d.nworld, m.nv),
    inputs=[d.dof_cdof, d.cqacc, d.cqfrc_constraint],
    outputs=[d.qacc, d.qfrc_constraint],
  )

  # Refresh full d.efc.Ma = M @ qacc. The integrators (Euler/implicit damping) use Ma as
  # the RHS; the compact solve only populated the compacted Ma, so recompute it in full
  # space. Inactive DOFs have qacc=0 so their Ma is 0 and they stay frozen.
  support.mul_m(m, d, d.efc.Ma, d.qacc)
