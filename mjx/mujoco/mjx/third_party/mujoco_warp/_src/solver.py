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
from math import sqrt
from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import nested_kernel

wp.set_module_options({"enable_backward": False})

_BLOCK_CHOLESKY_DIM = 32


@wp.func
def _rescale(nv: int, stat_meaninertia: float, value: float) -> float:
  return value / (stat_meaninertia * float(nv))


@wp.func
def _in_bracket(x: wp.vec3, y: wp.vec3) -> bool:
  return (x[1] < y[1] and y[1] < 0.0) or (x[1] > y[1] and y[1] > 0.0)


@wp.func
def _eval_cost(quad: wp.vec3, alpha: float) -> float:
  return alpha * alpha * quad[2] + alpha * quad[1] + quad[0]


@wp.func
def _eval_pt(quad: wp.vec3, alpha: float) -> wp.vec3:
  return wp.vec3(
    _eval_cost(quad, alpha),
    2.0 * alpha * quad[2] + quad[1],
    2.0 * quad[2],
  )


@wp.func
def _eval_frictionloss(
  # In:
  x: float,
  f: float,
  rf: float,
  Jaref: float,
  jv: float,
  quad: wp.vec3,
) -> wp.vec3:
  # -bound < x < bound : quadratic
  if (-rf < x) and (x < rf):
    return quad
  # x < -bound: linear negative
  elif x <= -rf:
    return wp.vec3(f * (-0.5 * rf - Jaref), -f * jv, 0.0)
  # bound < x : linear positive
  else:
    return wp.vec3(f * (-0.5 * rf + Jaref), f * jv, 0.0)


@wp.func
def _eval_elliptic(
  # In:
  impratio_invsqrt: float,
  friction: types.vec5,
  quad: wp.vec3,
  quad1: wp.vec3,
  quad2: wp.vec3,
  alpha: float,
) -> wp.vec3:
  mu = friction[0] * impratio_invsqrt

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
def _eval_init(
  # Data in:
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_efc_address_in: wp.array2d(dtype=int),
  # In:
  ne_clip: int,
  nef_clip: int,
  nefc_clip: int,
  impratio_invsqrt: float,
  type_in: wp.array(dtype=int),
  id_in: wp.array(dtype=int),
  D_in: wp.array(dtype=float),
  frictionloss_in: wp.array(dtype=float),
  Jaref_in: wp.array(dtype=float),
  jv_in: wp.array(dtype=float),
  quad_in: wp.array(dtype=wp.vec3),
  alpha: float,
) -> wp.vec3:
  lo = wp.vec3(0.0, 0.0, 0.0)
  for efcid in range(ne_clip):
    quad = quad_in[efcid]
    lo += _eval_pt(quad, alpha)

  for efcid in range(ne_clip, nef_clip):
    D = D_in[efcid]
    f = frictionloss_in[efcid]
    Jaref = Jaref_in[efcid]
    jv = jv_in[efcid]

    # search point, friction loss, bound (rf)
    x = Jaref + alpha * jv
    rf = math.safe_div(f, D)

    quad_f = _eval_frictionloss(x, f, rf, Jaref, jv, quad_in[efcid])
    lo += _eval_pt(quad_f, alpha)

  for efcid in range(nef_clip, nefc_clip):
    if type_in[efcid] == types.ConstraintType.CONTACT_ELLIPTIC:
      conid = id_in[efcid]

      efcid0 = contact_efc_address_in[conid, 0]
      if efcid != efcid0:
        continue

      efcid1 = contact_efc_address_in[conid, 1]
      efcid2 = contact_efc_address_in[conid, 2]
      efc_quad0 = quad_in[efcid0]
      efc_quad1 = quad_in[efcid1]
      efc_quad2 = quad_in[efcid2]
      friction = contact_friction_in[conid]

      lo += _eval_elliptic(impratio_invsqrt, friction, efc_quad0, efc_quad1, efc_quad2, alpha)
    else:
      Jaref = Jaref_in[efcid]
      jv = jv_in[efcid]
      quad = quad_in[efcid]

      x = Jaref + alpha * jv
      res = _eval_pt(quad, alpha)
      lo += res * float(x < 0.0)

  return lo


@wp.func
def _eval(
  # Data in:
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_efc_address_in: wp.array2d(dtype=int),
  # In:
  ne_clip: int,
  nef_clip: int,
  nefc_clip: int,
  impratio_invsqrt: float,
  type_in: wp.array(dtype=int),
  id_in: wp.array(dtype=int),
  D_in: wp.array(dtype=float),
  frictionloss_in: wp.array(dtype=float),
  Jaref_in: wp.array(dtype=float),
  jv_in: wp.array(dtype=float),
  quad_in: wp.array(dtype=wp.vec3),
  lo_alpha: float,
  hi_alpha: float,
  mid_alpha: float,
) -> Tuple[wp.vec3, wp.vec3, wp.vec3]:
  lo = wp.vec3(0.0, 0.0, 0.0)
  hi = wp.vec3(0.0, 0.0, 0.0)
  mid = wp.vec3(0.0, 0.0, 0.0)
  for efcid in range(ne_clip):
    quad = quad_in[efcid]
    lo += _eval_pt(quad, lo_alpha)
    hi += _eval_pt(quad, hi_alpha)
    mid += _eval_pt(quad, mid_alpha)

  for efcid in range(ne_clip, nef_clip):
    quad = quad_in[efcid]
    D = D_in[efcid]
    f = frictionloss_in[efcid]
    Jaref = Jaref_in[efcid]
    jv = jv_in[efcid]

    # search point, friction loss, bound (rf)
    rf = math.safe_div(f, D)
    x_lo = Jaref + lo_alpha * jv
    x_hi = Jaref + hi_alpha * jv
    x_mid = Jaref + mid_alpha * jv

    quad_f = _eval_frictionloss(x_lo, f, rf, Jaref, jv, quad)
    lo += _eval_pt(quad_f, lo_alpha)
    quad_f = _eval_frictionloss(x_hi, f, rf, Jaref, jv, quad)
    hi += _eval_pt(quad_f, hi_alpha)
    quad_f = _eval_frictionloss(x_mid, f, rf, Jaref, jv, quad)
    mid += _eval_pt(quad_f, mid_alpha)

  for efcid in range(nef_clip, nefc_clip):
    if type_in[efcid] == types.ConstraintType.CONTACT_ELLIPTIC:
      conid = id_in[efcid]

      efcid0 = contact_efc_address_in[conid, 0]
      if efcid != efcid0:
        continue

      efcid1 = contact_efc_address_in[conid, 1]
      efcid2 = contact_efc_address_in[conid, 2]
      efc_quad0 = quad_in[efcid0]
      efc_quad1 = quad_in[efcid1]
      efc_quad2 = quad_in[efcid2]
      friction = contact_friction_in[conid]

      lo += _eval_elliptic(impratio_invsqrt, friction, efc_quad0, efc_quad1, efc_quad2, lo_alpha)
      hi += _eval_elliptic(impratio_invsqrt, friction, efc_quad0, efc_quad1, efc_quad2, hi_alpha)
      mid += _eval_elliptic(impratio_invsqrt, friction, efc_quad0, efc_quad1, efc_quad2, mid_alpha)
    else:
      Jaref = Jaref_in[efcid]
      jv = jv_in[efcid]
      quad = quad_in[efcid]

      x_lo = Jaref + lo_alpha * jv
      x_hi = Jaref + hi_alpha * jv
      x_mid = Jaref + mid_alpha * jv
      lo += _eval_pt(quad, lo_alpha) * float(x_lo < 0.0)
      hi += _eval_pt(quad, hi_alpha) * float(x_hi < 0.0)
      mid += _eval_pt(quad, mid_alpha) * float(x_mid < 0.0)

  return lo, hi, mid


@wp.kernel
def linesearch_iterative(
  # Model:
  nv: int,
  opt_tolerance: wp.array(dtype=float),
  opt_ls_tolerance: wp.array(dtype=float),
  opt_ls_iterations: int,
  opt_impratio_invsqrt: wp.array(dtype=float),
  stat_meaninertia: float,
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_frictionloss_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_search_dot_in: wp.array(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_quad_gauss_in: wp.array(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  njmax_in: int,
  # Data out:
  efc_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  impratio_invsqrt = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
  efc_type = efc_type_in[worldid]
  efc_id = efc_id_in[worldid]
  efc_D = efc_D_in[worldid]
  efc_frictionloss = efc_frictionloss_in[worldid]
  efc_Jaref = efc_Jaref_in[worldid]
  efc_jv = efc_jv_in[worldid]
  efc_quad = efc_quad_in[worldid]
  efc_quad_gauss = efc_quad_gauss_in[worldid]
  tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
  ls_tolerance = opt_ls_tolerance[worldid % opt_ls_tolerance.shape[0]]
  ne_clip = min(njmax_in, ne_in[worldid])
  nef_clip = min(njmax_in, ne_clip + nf_in[worldid])
  nefc_clip = min(njmax_in, nefc_in[worldid])

  # Calculate p0
  snorm = wp.sqrt(efc_search_dot_in[worldid])
  scale = stat_meaninertia * wp.float(nv)
  gtol = tolerance * ls_tolerance * snorm * scale
  p0 = wp.vec3(efc_quad_gauss[0], efc_quad_gauss[1], 2.0 * efc_quad_gauss[2])
  p0 += _eval_init(
    contact_friction_in,
    contact_efc_address_in,
    ne_clip,
    nef_clip,
    nefc_clip,
    impratio_invsqrt,
    efc_type,
    efc_id,
    efc_D,
    efc_frictionloss,
    efc_Jaref,
    efc_jv,
    efc_quad,
    0.0,
  )

  # Calculate lo bound
  lo_alpha_in = -math.safe_div(p0[1], p0[2])
  lo_in = _eval_pt(efc_quad_gauss, lo_alpha_in)
  lo_in += _eval_init(
    contact_friction_in,
    contact_efc_address_in,
    ne_clip,
    nef_clip,
    nefc_clip,
    impratio_invsqrt,
    efc_type,
    efc_id,
    efc_D,
    efc_frictionloss,
    efc_Jaref,
    efc_jv,
    efc_quad,
    lo_alpha_in,
  )

  # Initialize bounds
  lo_less = lo_in[1] < p0[1]
  lo = wp.where(lo_less, lo_in, p0)
  lo_alpha = wp.where(lo_less, lo_alpha_in, 0.0)
  hi = wp.where(lo_less, p0, lo_in)
  hi_alpha = wp.where(lo_less, 0.0, lo_alpha_in)

  # Launch main linesearch iterative loop
  alpha = float(0.0)
  for _ in range(opt_ls_iterations):
    lo_next_alpha = lo_alpha - math.safe_div(lo[1], lo[2])
    hi_next_alpha = hi_alpha - math.safe_div(hi[1], hi[2])
    mid_alpha = 0.5 * (lo_alpha + hi_alpha)

    lo_next, hi_next, mid = _eval(
      contact_friction_in,
      contact_efc_address_in,
      ne_clip,
      nef_clip,
      nefc_clip,
      impratio_invsqrt,
      efc_type,
      efc_id,
      efc_D,
      efc_frictionloss,
      efc_Jaref,
      efc_jv,
      efc_quad,
      lo_next_alpha,
      hi_next_alpha,
      mid_alpha,
    )
    lo_next += _eval_pt(efc_quad_gauss, lo_next_alpha)
    hi_next += _eval_pt(efc_quad_gauss, hi_next_alpha)
    mid += _eval_pt(efc_quad_gauss, mid_alpha)

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

    # if we did not adjust the interval, we are done
    # also done if either low or hi slope is nearly flat
    ls_done = (not swap_lo and not swap_hi) or (lo[1] < 0 and lo[1] > -gtol) or (hi[1] > 0 and hi[1] < gtol)

    # update alpha if we have an improvement
    improved = lo[0] < p0[0] or hi[0] < p0[0]
    lo_better = lo[0] < hi[0]
    alpha = wp.where(improved and lo_better, lo_alpha, alpha)
    alpha = wp.where(improved and not lo_better, hi_alpha, alpha)
    if ls_done:
      break

  efc_alpha_out[worldid] = alpha


def _linesearch_iterative(m: types.Model, d: types.Data):
  """Iterative linesearch."""
  wp.launch(
    linesearch_iterative,
    dim=d.nworld,
    inputs=[
      m.nv,
      m.opt.tolerance,
      m.opt.ls_tolerance,
      m.opt.ls_iterations,
      m.opt.impratio_invsqrt,
      m.stat.meaninertia,
      d.ne,
      d.nf,
      d.nefc,
      d.contact.friction,
      d.contact.efc_address,
      d.efc.type,
      d.efc.id,
      d.efc.D,
      d.efc.frictionloss,
      d.efc.Jaref,
      d.efc.search_dot,
      d.efc.jv,
      d.efc.quad,
      d.efc.quad_gauss,
      d.efc.done,
      d.njmax,
    ],
    outputs=[d.efc.alpha],
    block_dim=m.block_dim.linesearch_iterative,
  )


@wp.func
def _log_scale(min_value: float, max_value: float, num_values: int, i: int) -> float:
  step = (wp.log(max_value) - wp.log(min_value)) / wp.max(1.0, float(num_values - 1))
  return wp.exp(wp.log(min_value) + float(i) * step)


@wp.kernel
def linesearch_parallel_fused(
  # Model:
  opt_ls_iterations: int,
  opt_impratio_invsqrt: wp.array(dtype=float),
  opt_ls_parallel_min_step: float,
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_frictionloss_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_quad_gauss_in: wp.array(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  njmax_in: int,
  nacon_in: wp.array(dtype=int),
  # Out:
  cost_out: wp.array2d(dtype=float),
):
  worldid, alphaid = wp.tid()

  if efc_done_in[worldid]:
    return

  alpha = _log_scale(opt_ls_parallel_min_step, 1.0, opt_ls_iterations, alphaid)

  out = _eval_cost(efc_quad_gauss_in[worldid], alpha)

  ne = ne_in[worldid]
  nf = nf_in[worldid]

  # TODO(team): _eval with option to only compute cost
  for efcid in range(min(njmax_in, nefc_in[worldid])):
    # equality
    if efcid < ne:
      out += _eval_cost(efc_quad_in[worldid, efcid], alpha)
    # friction
    elif efcid < ne + nf:
      # search point, friction loss, bound (rf)
      start = efc_Jaref_in[worldid, efcid]
      dir = efc_jv_in[worldid, efcid]
      x = start + alpha * dir
      f = efc_frictionloss_in[worldid, efcid]
      rf = math.safe_div(f, efc_D_in[worldid, efcid])

      # -bound < x < bound : quadratic
      if (-rf < x) and (x < rf):
        quad = efc_quad_in[worldid, efcid]
      # x < -bound: linear negative
      elif x <= -rf:
        quad = wp.vec3(f * (-0.5 * rf - start), -f * dir, 0.0)
      # bound < x : linear positive
      else:
        quad = wp.vec3(f * (-0.5 * rf + start), f * dir, 0.0)

      out += _eval_cost(quad, alpha)
    # limit and contact
    elif efc_type_in[worldid, efcid] == types.ConstraintType.CONTACT_ELLIPTIC:
      # extract contact info
      conid = efc_id_in[worldid, efcid]

      if conid >= nacon_in[0]:
        continue

      efcid0 = contact_efc_address_in[conid, 0]
      if efcid != efcid0:
        continue

      friction = contact_friction_in[conid]
      mu = friction[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

      # unpack quad
      efcid1 = contact_efc_address_in[conid, 1]
      efcid2 = contact_efc_address_in[conid, 2]
      u0 = efc_quad_in[worldid, efcid1][0]
      v0 = efc_quad_in[worldid, efcid1][1]
      uu = efc_quad_in[worldid, efcid1][2]
      uv = efc_quad_in[worldid, efcid2][0]
      vv = efc_quad_in[worldid, efcid2][1]
      dm = efc_quad_in[worldid, efcid2][2]

      # compute N, Tsqr
      N = u0 + alpha * v0
      Tsqr = uu + alpha * (2.0 * uv + alpha * vv)

      # no tangential force: top or bottom zone
      if Tsqr <= 0.0:
        # bottom zone: quadratic cost
        if N < 0.0:
          out += _eval_cost(efc_quad_in[worldid, efcid], alpha)
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
          out += _eval_cost(efc_quad_in[worldid, efcid], alpha)
        # otherwise middle zone
        else:
          out += 0.5 * dm * (N - mu * T) * (N - mu * T)
    else:
      # search point
      x = efc_Jaref_in[worldid, efcid] + alpha * efc_jv_in[worldid, efcid]

      # active
      if x < 0.0:
        out += _eval_cost(efc_quad_in[worldid, efcid], alpha)

  cost_out[worldid, alphaid] = out


@wp.kernel
def linesearch_parallel_best_alpha(
  # Model:
  opt_ls_iterations: int,
  opt_ls_parallel_min_step: float,
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  # In:
  cost_in: wp.array2d(dtype=float),
  # Data out:
  efc_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  bestid = int(0)
  best_cost = float(wp.inf)
  for i in range(opt_ls_iterations):
    cost = cost_in[worldid, i]
    if cost < best_cost:
      best_cost = cost
      bestid = i

  efc_alpha_out[worldid] = _log_scale(opt_ls_parallel_min_step, 1.0, opt_ls_iterations, bestid)


def _linesearch_parallel(m: types.Model, d: types.Data, cost: wp.array2d(dtype=float)):
  wp.launch(
    linesearch_parallel_fused,
    dim=(d.nworld, m.opt.ls_iterations),
    inputs=[
      m.opt.ls_iterations,
      m.opt.impratio_invsqrt,
      m.opt.ls_parallel_min_step,
      d.ne,
      d.nf,
      d.nefc,
      d.contact.friction,
      d.contact.efc_address,
      d.efc.type,
      d.efc.id,
      d.efc.D,
      d.efc.frictionloss,
      d.efc.Jaref,
      d.efc.jv,
      d.efc.quad,
      d.efc.quad_gauss,
      d.efc.done,
      d.njmax,
      d.nacon,
    ],
    outputs=[cost],
  )

  wp.launch(
    linesearch_parallel_best_alpha,
    dim=(d.nworld),
    inputs=[m.opt.ls_iterations, m.opt.ls_parallel_min_step, d.efc.done, cost],
    outputs=[d.efc.alpha],
  )


@wp.kernel
def linesearch_zero_jv(
  # Data in:
  nefc_in: wp.array(dtype=int),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_jv_out: wp.array2d(dtype=float),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  efc_jv_out[worldid, efcid] = 0.0


@cache_kernel
def linesearch_jv_fused(nv: int, dofs_per_thread: int):
  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array(dtype=int),
    efc_J_in: wp.array3d(dtype=float),
    efc_search_in: wp.array2d(dtype=float),
    efc_done_in: wp.array(dtype=bool),
    # Data out:
    efc_jv_out: wp.array2d(dtype=float),
  ):
    worldid, efcid, dofstart = wp.tid()

    if efcid >= nefc_in[worldid]:
      return

    if efc_done_in[worldid]:
      return

    jv_out = float(0.0)

    if wp.static(dofs_per_thread >= nv):
      for i in range(wp.static(min(dofs_per_thread, nv))):
        jv_out += efc_J_in[worldid, efcid, i] * efc_search_in[worldid, i]
      efc_jv_out[worldid, efcid] = jv_out

    else:
      for i in range(wp.static(dofs_per_thread)):
        ii = dofstart * wp.static(dofs_per_thread) + i
        if ii < nv:
          jv_out += efc_J_in[worldid, efcid, ii] * efc_search_in[worldid, ii]
      wp.atomic_add(efc_jv_out, worldid, efcid, jv_out)

  return kernel


@wp.kernel
def linesearch_prepare_gauss(
  # Model:
  nv: int,
  # Data in:
  qfrc_smooth_in: wp.array2d(dtype=float),
  efc_Ma_in: wp.array2d(dtype=float),
  efc_search_in: wp.array2d(dtype=float),
  efc_gauss_in: wp.array(dtype=float),
  efc_mv_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_quad_gauss_out: wp.array(dtype=wp.vec3),
):
  worldid = wp.tid()
  if efc_done_in[worldid]:
    return

  quad_gauss_0 = efc_gauss_in[worldid]
  quad_gauss_1 = float(0.0)
  quad_gauss_2 = float(0.0)
  for i in range(nv):
    search = efc_search_in[worldid, i]
    quad_gauss_1 += search * (efc_Ma_in[worldid, i] - qfrc_smooth_in[worldid, i])
    quad_gauss_2 += 0.5 * search * efc_mv_in[worldid, i]

  efc_quad_gauss_out[worldid] = wp.vec3(quad_gauss_0, quad_gauss_1, quad_gauss_2)


@wp.kernel
def linesearch_prepare_quad(
  # Model:
  opt_impratio_invsqrt: wp.array(dtype=float),
  # Data in:
  nefc_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  nacon_in: wp.array(dtype=int),
  # Data out:
  efc_quad_out: wp.array2d(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  Jaref = efc_Jaref_in[worldid, efcid]
  jv = efc_jv_in[worldid, efcid]
  efc_D = efc_D_in[worldid, efcid]

  # init with scalar quadratic
  quad = wp.vec3(0.5 * Jaref * Jaref * efc_D, jv * Jaref * efc_D, 0.5 * jv * jv * efc_D)

  # elliptic cone: extra processing
  if efc_type_in[worldid, efcid] == types.ConstraintType.CONTACT_ELLIPTIC:
    # extract contact info
    conid = efc_id_in[worldid, efcid]

    if conid >= nacon_in[0]:
      return

    efcid0 = contact_efc_address_in[conid, 0]

    if efcid != efcid0:
      return

    dim = contact_dim_in[conid]
    friction = contact_friction_in[conid]
    mu = friction[0] * opt_impratio_invsqrt[worldid]

    u0 = Jaref * mu
    v0 = jv * mu

    uu = float(0.0)
    uv = float(0.0)
    vv = float(0.0)
    for j in range(1, dim):
      # complete vector quadratic (for bottom zone)
      efcidj = contact_efc_address_in[conid, j]
      if efcidj < 0:
        return
      jvj = efc_jv_in[worldid, efcidj]
      jarefj = efc_Jaref_in[worldid, efcidj]
      dj = efc_D_in[worldid, efcidj]
      DJj = dj * jarefj

      quad += wp.vec3(
        0.5 * jarefj * DJj,
        jvj * DJj,
        0.5 * jvj * dj * jvj,
      )

      # rescale to make primal cone circular
      frictionj = friction[j - 1]
      uj = jarefj * frictionj
      vj = jvj * frictionj

      # accumulate sums of squares
      uu += uj * uj
      uv += uj * vj
      vv += vj * vj

    quad1 = wp.vec3(u0, v0, uu)
    efcid1 = contact_efc_address_in[conid, 1]
    efc_quad_out[worldid, efcid1] = quad1

    mu2 = mu * mu
    quad2 = wp.vec3(uv, vv, efc_D / (mu2 * (1.0 + mu2)))
    efcid2 = contact_efc_address_in[conid, 2]
    efc_quad_out[worldid, efcid2] = quad2

  efc_quad_out[worldid, efcid] = quad


@wp.kernel
def linesearch_qacc_ma(
  # Data in:
  efc_search_in: wp.array2d(dtype=float),
  efc_mv_in: wp.array2d(dtype=float),
  efc_alpha_in: wp.array(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  qacc_out: wp.array2d(dtype=float),
  efc_Ma_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  alpha = efc_alpha_in[worldid]
  qacc_out[worldid, dofid] += alpha * efc_search_in[worldid, dofid]
  efc_Ma_out[worldid, dofid] += alpha * efc_mv_in[worldid, dofid]


@wp.kernel
def linesearch_jaref(
  # Data in:
  nefc_in: wp.array(dtype=int),
  efc_jv_in: wp.array2d(dtype=float),
  efc_alpha_in: wp.array(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_Jaref_out: wp.array2d(dtype=float),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  efc_Jaref_out[worldid, efcid] += efc_alpha_in[worldid] * efc_jv_in[worldid, efcid]


@event_scope
def _linesearch(m: types.Model, d: types.Data, cost: wp.array2d(dtype=float)):
  # mv = qM @ search
  support.mul_m(m, d, d.efc.mv, d.efc.search, skip=d.efc.done)

  # jv = efc_J @ search
  # TODO(team): is there a better way of doing batched matmuls with dynamic array sizes?

  # if we are only using 1 thread, it makes sense to do more dofs as we can also skip the
  # init kernel. For more than 1 thread, dofs_per_thread is lower for better load balancing.

  if m.nv > 50:
    dofs_per_thread = 20
  else:
    dofs_per_thread = 50

  threads_per_efc = ceil(m.nv / dofs_per_thread)
  # we need to clear the jv array if we're doing atomic adds.
  if threads_per_efc > 1:
    wp.launch(
      linesearch_zero_jv,
      dim=(d.nworld, d.njmax),
      inputs=[d.nefc, d.efc.done],
      outputs=[d.efc.jv],
    )

  wp.launch(
    linesearch_jv_fused(m.nv, dofs_per_thread),
    dim=(d.nworld, d.njmax, threads_per_efc),
    inputs=[d.nefc, d.efc.J, d.efc.search, d.efc.done],
    outputs=[d.efc.jv],
  )

  # prepare quadratics
  # quad_gauss = [gauss, search.T @ Ma - search.T @ qfrc_smooth, 0.5 * search.T @ mv]
  wp.launch(
    linesearch_prepare_gauss,
    dim=(d.nworld),
    inputs=[m.nv, d.qfrc_smooth, d.efc.Ma, d.efc.search, d.efc.gauss, d.efc.mv, d.efc.done],
    outputs=[d.efc.quad_gauss],
  )

  # quad = [0.5 * Jaref * Jaref * efc_D, jv * Jaref * efc_D, 0.5 * jv * jv * efc_D]
  wp.launch(
    linesearch_prepare_quad,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.opt.impratio_invsqrt,
      d.nefc,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.efc.type,
      d.efc.id,
      d.efc.D,
      d.efc.Jaref,
      d.efc.jv,
      d.efc.done,
      d.nacon,
    ],
    outputs=[d.efc.quad],
  )

  if m.opt.ls_parallel:
    _linesearch_parallel(m, d, cost)
  else:
    _linesearch_iterative(m, d)

  wp.launch(
    linesearch_qacc_ma,
    dim=(d.nworld, m.nv),
    inputs=[d.efc.search, d.efc.mv, d.efc.alpha, d.efc.done],
    outputs=[d.qacc, d.efc.Ma],
  )

  wp.launch(
    linesearch_jaref,
    dim=(d.nworld, d.njmax),
    inputs=[d.nefc, d.efc.jv, d.efc.alpha, d.efc.done],
    outputs=[d.efc.Jaref],
  )


@wp.kernel
def solve_init_efc(
  # Data out:
  solver_niter_out: wp.array(dtype=int),
  efc_search_dot_out: wp.array(dtype=float),
  efc_cost_out: wp.array(dtype=float),
  efc_done_out: wp.array(dtype=bool),
):
  worldid = wp.tid()
  efc_cost_out[worldid] = wp.inf
  solver_niter_out[worldid] = 0
  efc_done_out[worldid] = False
  efc_search_dot_out[worldid] = 0.0


@wp.kernel
def solve_init_jaref(
  # Model:
  nv: int,
  # Data in:
  nefc_in: wp.array(dtype=int),
  qacc_in: wp.array2d(dtype=float),
  efc_J_in: wp.array3d(dtype=float),
  efc_aref_in: wp.array2d(dtype=float),
  # Data out:
  efc_Jaref_out: wp.array2d(dtype=float),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  jaref = float(0.0)
  for i in range(nv):
    jaref += efc_J_in[worldid, efcid, i] * qacc_in[worldid, i]

  efc_Jaref_out[worldid, efcid] = jaref - efc_aref_in[worldid, efcid]


@wp.kernel
def solve_init_search(
  # Data in:
  efc_Mgrad_in: wp.array2d(dtype=float),
  # Data out:
  efc_search_out: wp.array2d(dtype=float),
  efc_search_dot_out: wp.array(dtype=float),
):
  worldid, dofid = wp.tid()
  search = -1.0 * efc_Mgrad_in[worldid, dofid]
  efc_search_out[worldid, dofid] = search
  wp.atomic_add(efc_search_dot_out, worldid, search * search)


@wp.kernel
def update_constraint_init_cost(
  # Data in:
  efc_cost_in: wp.array(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_gauss_out: wp.array(dtype=float),
  efc_cost_out: wp.array(dtype=float),
  efc_prev_cost_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  efc_gauss_out[worldid] = 0.0
  efc_prev_cost_out[worldid] = efc_cost_in[worldid]
  efc_cost_out[worldid] = 0.0


@wp.kernel
def update_constraint_efc(
  # Model:
  opt_impratio_invsqrt: wp.array(dtype=float),
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  efc_type_in: wp.array2d(dtype=int),
  efc_id_in: wp.array2d(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_frictionloss_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  nacon_in: wp.array(dtype=int),
  # Data out:
  efc_force_out: wp.array2d(dtype=float),
  efc_cost_out: wp.array(dtype=float),
  efc_state_out: wp.array2d(dtype=int),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  efc_D = efc_D_in[worldid, efcid]
  Jaref = efc_Jaref_in[worldid, efcid]

  ne = ne_in[worldid]
  nf = nf_in[worldid]

  if efcid < ne:
    # equality
    efc_force_out[worldid, efcid] = -efc_D * Jaref
    efc_state_out[worldid, efcid] = types.ConstraintState.QUADRATIC
    wp.atomic_add(efc_cost_out, worldid, 0.5 * efc_D * Jaref * Jaref)
  elif efcid < ne + nf:
    # friction
    f = efc_frictionloss_in[worldid, efcid]
    rf = math.safe_div(f, efc_D)
    if Jaref <= -rf:
      efc_force_out[worldid, efcid] = f
      efc_state_out[worldid, efcid] = types.ConstraintState.LINEARNEG
      wp.atomic_add(efc_cost_out, worldid, -f * (0.5 * rf + Jaref))
    elif Jaref >= rf:
      efc_force_out[worldid, efcid] = -f
      efc_state_out[worldid, efcid] = types.ConstraintState.LINEARPOS
      wp.atomic_add(efc_cost_out, worldid, -f * (0.5 * rf - Jaref))
    else:
      efc_force_out[worldid, efcid] = -efc_D * Jaref
      efc_state_out[worldid, efcid] = types.ConstraintState.QUADRATIC
      wp.atomic_add(efc_cost_out, worldid, 0.5 * efc_D * Jaref * Jaref)
  elif efc_type_in[worldid, efcid] != types.ConstraintType.CONTACT_ELLIPTIC:
    # limit, frictionless contact, pyramidal friction cone contact
    if Jaref >= 0.0:
      efc_force_out[worldid, efcid] = 0.0
      efc_state_out[worldid, efcid] = types.ConstraintState.SATISFIED
    else:
      efc_force_out[worldid, efcid] = -efc_D * Jaref
      efc_state_out[worldid, efcid] = types.ConstraintState.QUADRATIC
      wp.atomic_add(efc_cost_out, worldid, 0.5 * efc_D * Jaref * Jaref)
  else:  # elliptic friction cone contact
    conid = efc_id_in[worldid, efcid]

    if conid >= nacon_in[0]:
      return

    dim = contact_dim_in[conid]
    friction = contact_friction_in[conid]
    mu = friction[0] * opt_impratio_invsqrt[worldid]

    efcid0 = contact_efc_address_in[conid, 0]
    if efcid0 < 0:
      return

    N = efc_Jaref_in[worldid, efcid0] * mu

    ufrictionj = float(0.0)
    TT = float(0.0)
    for j in range(1, dim):
      efcidj = contact_efc_address_in[conid, j]
      if efcidj < 0:
        return
      frictionj = friction[j - 1]
      uj = efc_Jaref_in[worldid, efcidj] * frictionj
      TT += uj * uj
      if efcid == efcidj:
        ufrictionj = uj * frictionj

    if TT <= 0.0:
      T = 0.0
    else:
      T = wp.sqrt(TT)

    # top zone
    if (N >= mu * T) or ((T <= 0.0) and (N >= 0.0)):
      efc_force_out[worldid, efcid] = 0.0
      efc_state_out[worldid, efcid] = types.ConstraintState.SATISFIED
    # bottom zone
    elif (mu * N + T <= 0.0) or ((T <= 0.0) and (N < 0.0)):
      efc_force_out[worldid, efcid] = -efc_D * Jaref
      efc_state_out[worldid, efcid] = types.ConstraintState.QUADRATIC
      wp.atomic_add(efc_cost_out, worldid, 0.5 * efc_D * Jaref * Jaref)
    # middle zone
    else:
      dm = math.safe_div(efc_D_in[worldid, efcid0], mu * mu * (1.0 + mu * mu))
      nmt = N - mu * T

      force = -dm * nmt * mu

      if efcid == efcid0:
        efc_force_out[worldid, efcid] = force
        wp.atomic_add(efc_cost_out, worldid, 0.5 * dm * nmt * nmt)
      else:
        efc_force_out[worldid, efcid] = -math.safe_div(force, T) * ufrictionj

      efc_state_out[worldid, efcid] = types.ConstraintState.CONE


@wp.kernel
def update_constraint_init_qfrc_constraint(
  # Data in:
  nefc_in: wp.array(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  efc_force_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  njmax_in: int,
  # Data out:
  qfrc_constraint_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  sum_qfrc = float(0.0)
  for efcid in range(min(njmax_in, nefc_in[worldid])):
    efc_J = efc_J_in[worldid, efcid, dofid]
    force = efc_force_in[worldid, efcid]
    sum_qfrc += efc_J * force

  qfrc_constraint_out[worldid, dofid] = sum_qfrc


@cache_kernel
def update_constraint_gauss_cost(nv: int, dofs_per_thread: int):
  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    qacc_in: wp.array2d(dtype=float),
    qfrc_smooth_in: wp.array2d(dtype=float),
    qacc_smooth_in: wp.array2d(dtype=float),
    efc_Ma_in: wp.array2d(dtype=float),
    efc_done_in: wp.array(dtype=bool),
    # Data out:
    efc_gauss_out: wp.array(dtype=float),
    efc_cost_out: wp.array(dtype=float),
  ):
    worldid, dofstart = wp.tid()

    if efc_done_in[worldid]:
      return

    gauss_cost = float(0.0)

    if wp.static(dofs_per_thread >= nv):
      for i in range(wp.static(min(dofs_per_thread, nv))):
        gauss_cost += (efc_Ma_in[worldid, i] - qfrc_smooth_in[worldid, i]) * (qacc_in[worldid, i] - qacc_smooth_in[worldid, i])
      efc_gauss_out[worldid] += 0.5 * gauss_cost
      efc_cost_out[worldid] += 0.5 * gauss_cost

    else:
      for i in range(wp.static(dofs_per_thread)):
        ii = dofstart * wp.static(dofs_per_thread) + i
        if ii < nv:
          gauss_cost += (efc_Ma_in[worldid, ii] - qfrc_smooth_in[worldid, ii]) * (
            qacc_in[worldid, ii] - qacc_smooth_in[worldid, ii]
          )
      wp.atomic_add(efc_gauss_out, worldid, gauss_cost)
      wp.atomic_add(efc_cost_out, worldid, gauss_cost)

  return kernel


def _update_constraint(m: types.Model, d: types.Data):
  """Update constraint arrays after each solve iteration."""
  wp.launch(
    update_constraint_init_cost,
    dim=(d.nworld),
    inputs=[d.efc.cost, d.efc.done],
    outputs=[d.efc.gauss, d.efc.cost, d.efc.prev_cost],
  )

  wp.launch(
    update_constraint_efc,
    dim=(d.nworld, d.njmax),
    inputs=[
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
      d.efc.Jaref,
      d.efc.done,
      d.nacon,
    ],
    outputs=[d.efc.force, d.efc.cost, d.efc.state],
  )

  # qfrc_constraint = efc_J.T @ efc_force
  wp.launch(
    update_constraint_init_qfrc_constraint,
    dim=(d.nworld, m.nv),
    inputs=[d.nefc, d.efc.J, d.efc.force, d.efc.done, d.njmax],
    outputs=[d.qfrc_constraint],
  )

  # if we are only using 1 thread, it makes sense to do more dofs and skip the atomics.
  # For more than 1 thread, dofs_per_thread is lower for better load balancing.
  if m.nv > 50:
    dofs_per_thread = 20
  else:
    dofs_per_thread = 50

  threads_per_efc = ceil(m.nv / dofs_per_thread)

  # gauss = 0.5 * (Ma - qfrc_smooth).T @ (qacc - qacc_smooth)
  wp.launch(
    update_constraint_gauss_cost(m.nv, dofs_per_thread),
    dim=(d.nworld, threads_per_efc),
    inputs=[d.qacc, d.qfrc_smooth, d.qacc_smooth, d.efc.Ma, d.efc.done],
    outputs=[d.efc.gauss, d.efc.cost],
  )


@wp.kernel
def update_gradient_zero_grad_dot(
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_grad_dot_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  efc_grad_dot_out[worldid] = 0.0


@wp.kernel
def update_gradient_grad(
  # Data in:
  qfrc_smooth_in: wp.array2d(dtype=float),
  qfrc_constraint_in: wp.array2d(dtype=float),
  efc_Ma_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_grad_out: wp.array2d(dtype=float),
  efc_grad_dot_out: wp.array(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  grad = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - qfrc_constraint_in[worldid, dofid]
  efc_grad_out[worldid, dofid] = grad
  wp.atomic_add(efc_grad_dot_out, worldid, grad * grad)


@wp.kernel
def update_gradient_set_h_qM_lower_sparse(
  # Model:
  qM_fullm_i: wp.array(dtype=int),
  qM_fullm_j: wp.array(dtype=int),
  # Data in:
  qM_in: wp.array3d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Out:
  h_out: wp.array3d(dtype=float),
):
  worldid, elementid = wp.tid()

  if efc_done_in[worldid]:
    return

  i = qM_fullm_i[elementid]
  j = qM_fullm_j[elementid]
  h_out[worldid, i, j] += qM_in[worldid, 0, elementid]


@wp.func
def state_check(D: float, state: int) -> float:
  if state == types.ConstraintState.QUADRATIC.value:
    return D
  else:
    return 0.0


@wp.func
def active_check(tid: int, threshold: int) -> float:
  if tid >= threshold:
    return 0.0
  else:
    return 1.0


@cache_kernel
def update_gradient_JTDAJ_sparse_tiled(tile_size: int, njmax: int):
  TILE_SIZE = tile_size

  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array(dtype=int),
    efc_J_in: wp.array3d(dtype=float),
    efc_D_in: wp.array2d(dtype=float),
    efc_state_in: wp.array2d(dtype=int),
    efc_done_in: wp.array(dtype=bool),
    # Out:
    h_out: wp.array3d(dtype=float),
  ):
    worldid, elementid = wp.tid()

    if efc_done_in[worldid]:
      return

    nefc = nefc_in[worldid]

    # get lower diagonal index
    i = (int(sqrt(float(1 + 8 * elementid))) - 1) // 2
    j = elementid - (i * (i + 1)) // 2

    offset_i = i * TILE_SIZE
    offset_j = j * TILE_SIZE

    sum_val = wp.tile_zeros(shape=(TILE_SIZE, TILE_SIZE), dtype=wp.float32)

    # Each tile processes looping over all constraints, producing 1 output tile
    for k in range(0, njmax, TILE_SIZE):
      if k >= nefc:
        break

      # AD: leaving bounds-check disabled here because I'm not entirely sure that
      # everything always hits the fast path. The padding takes care of any
      # potential OOB accesses.
      J_ki = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(k, offset_i), bounds_check=False)

      if offset_i != offset_j:
        J_kj = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE, TILE_SIZE), offset=(k, offset_j), bounds_check=False)
      else:
        wp.tile_assign(J_kj, J_ki, (0, 0))

      D_k = wp.tile_load(efc_D_in[worldid], shape=TILE_SIZE, offset=k, bounds_check=False)
      state = wp.tile_load(efc_state_in[worldid], shape=TILE_SIZE, offset=k, bounds_check=False)

      D_k = wp.tile_map(state_check, D_k, state)

      # force unused elements to be zero
      tid_tile = wp.tile_arange(TILE_SIZE, dtype=int)
      threshold_tile = wp.tile_ones(shape=TILE_SIZE, dtype=int) * (nefc - k)

      active_tile = wp.tile_map(active_check, tid_tile, threshold_tile)
      D_k = wp.tile_map(wp.mul, active_tile, D_k)

      J_ki = wp.tile_map(wp.mul, wp.tile_transpose(J_ki), wp.tile_broadcast(D_k, shape=(TILE_SIZE, TILE_SIZE)))

      sum_val += wp.tile_matmul(J_ki, J_kj)

    # AD: setting bounds_check to True explicitly here because for some reason it was
    # slower to disable it.
    wp.tile_store(h_out[worldid], sum_val, offset=(offset_i, offset_j), bounds_check=True)

  return kernel


@cache_kernel
def update_gradient_JTDAJ_dense_tiled(nv_padded: int, tile_size: int, njmax: int):
  if njmax < tile_size:
    tile_size = njmax

  TILE_SIZE_K = tile_size

  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array(dtype=int),
    qM_in: wp.array3d(dtype=float),
    efc_J_in: wp.array3d(dtype=float),
    efc_D_in: wp.array2d(dtype=float),
    efc_state_in: wp.array2d(dtype=int),
    efc_done_in: wp.array(dtype=bool),
    # Out:
    h_out: wp.array3d(dtype=float),
  ):
    worldid = wp.tid()

    if efc_done_in[worldid]:
      return

    nefc = nefc_in[worldid]

    sum_val = wp.tile_load(qM_in[worldid], shape=(nv_padded, nv_padded), bounds_check=True)

    # Each tile processes one output tile by looping over all constraints
    for k in range(0, njmax, TILE_SIZE_K):
      if k >= nefc:
        break

      # AD: leaving bounds-check disabled here because I'm not entirely sure that
      # everything always hits the fast path. The padding takes care of any
      #  potential OOB accesses.
      J_ki = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE_K, nv_padded), offset=(k, 0), bounds_check=False)
      J_kj = J_ki

      # state check
      D_k = wp.tile_load(efc_D_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
      state = wp.tile_load(efc_state_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)

      D_k = wp.tile_map(state_check, D_k, state)

      # force unused elements to be zero
      tid_tile = wp.tile_arange(TILE_SIZE_K, dtype=int)
      threshold_tile = wp.tile_ones(shape=TILE_SIZE_K, dtype=int) * (nefc - k)

      active_tile = wp.tile_map(active_check, tid_tile, threshold_tile)
      D_k = wp.tile_map(wp.mul, active_tile, D_k)

      J_ki = wp.tile_map(wp.mul, wp.tile_transpose(J_ki), wp.tile_broadcast(D_k, shape=(nv_padded, TILE_SIZE_K)))

      sum_val += wp.tile_matmul(J_ki, J_kj)

    wp.tile_store(h_out[worldid], sum_val, bounds_check=False)

  return kernel


# TODO(thowell): combine with JTDAJ ?
@wp.kernel
def update_gradient_JTCJ(
  # Model:
  opt_impratio_invsqrt: wp.array(dtype=float),
  dof_tri_row: wp.array(dtype=int),
  dof_tri_col: wp.array(dtype=int),
  # Data in:
  contact_dist_in: wp.array(dtype=float),
  contact_includemargin_in: wp.array(dtype=float),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  efc_D_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_state_in: wp.array2d(dtype=int),
  efc_done_in: wp.array(dtype=bool),
  naconmax_in: int,
  nacon_in: wp.array(dtype=int),
  # In:
  nblocks_perblock: int,
  dim_block: int,
  # Out:
  h_out: wp.array3d(dtype=float),
):
  conid_start, elementid = wp.tid()

  dof1id = dof_tri_row[elementid]
  dof2id = dof_tri_col[elementid]

  for i in range(nblocks_perblock):
    conid = conid_start + i * dim_block

    if conid >= min(nacon_in[0], naconmax_in):
      return

    worldid = contact_worldid_in[conid]
    if efc_done_in[worldid]:
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
    mu = fri[0] * opt_impratio_invsqrt[worldid]

    mu2 = mu * mu
    dm = math.safe_div(efc_D_in[worldid, efcid0], mu2 * (1.0 + mu2))

    if dm == 0.0:
      continue

    n = efc_Jaref_in[worldid, efcid0] * mu
    u = types.vec6(n, 0.0, 0.0, 0.0, 0.0, 0.0)

    tt = float(0.0)
    for j in range(1, condim):
      efcidj = contact_efc_address_in[conid, j]
      uj = efc_Jaref_in[worldid, efcidj] * fri[j - 1]
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

    h_out[worldid, dof1id, dof2id] += h


@cache_kernel
def update_gradient_cholesky(tile_size: int):
  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    efc_grad_in: wp.array2d(dtype=float),
    h_in: wp.array3d(dtype=float),
    efc_done_in: wp.array(dtype=bool),
    # Data out:
    efc_Mgrad_out: wp.array2d(dtype=float),
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if efc_done_in[worldid]:
      return

    mat_tile = wp.tile_load(h_in[worldid], shape=(TILE_SIZE, TILE_SIZE))
    fact_tile = wp.tile_cholesky(mat_tile)
    input_tile = wp.tile_load(efc_grad_in[worldid], shape=TILE_SIZE)
    output_tile = wp.tile_cholesky_solve(fact_tile, input_tile)
    wp.tile_store(efc_Mgrad_out[worldid], output_tile)

  return kernel


@cache_kernel
def update_gradient_cholesky_blocked(tile_size: int, matrix_size: int):
  @nested_kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    efc_grad_in: wp.array3d(dtype=float),
    h_in: wp.array3d(dtype=float),
    efc_done_in: wp.array(dtype=bool),
    hfactor: wp.array3d(dtype=float),
    # Data out:
    efc_Mgrad_out: wp.array3d(dtype=float),
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if efc_done_in[worldid]:
      return

    # We need matrix size both as a runtime input as well as a static input:
    # static input is needed to specify the tile sizes for the compiler
    # runtime input is needed for the loop bounds, otherwise warp will unroll
    # unconditionally leading to shared memory capacity issues.

    wp.static(create_blocked_cholesky_func(TILE_SIZE))(h_in[worldid], matrix_size, hfactor[worldid])
    wp.static(create_blocked_cholesky_solve_func(TILE_SIZE, matrix_size))(
      hfactor[worldid], efc_grad_in[worldid], matrix_size, efc_Mgrad_out[worldid]
    )

  return kernel


@wp.kernel
def padding_h(nv: int, efc_done_in: wp.array(dtype=bool), h_out: wp.array3d(dtype=float)):
  worldid, elementid = wp.tid()

  if efc_done_in[worldid]:
    return

  dofid = nv + elementid
  h_out[worldid, dofid, dofid] = 1.0


def _update_gradient(m: types.Model, d: types.Data, h: wp.array3d(dtype=float), hfactor: wp.array3d(dtype=float)):
  # grad = Ma - qfrc_smooth - qfrc_constraint
  wp.launch(update_gradient_zero_grad_dot, dim=(d.nworld), inputs=[d.efc.done], outputs=[d.efc.grad_dot])

  wp.launch(
    update_gradient_grad,
    dim=(d.nworld, m.nv),
    inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, d.efc.done],
    outputs=[d.efc.grad, d.efc.grad_dot],
  )

  if m.opt.solver == types.SolverType.CG:
    smooth.solve_m(m, d, d.efc.Mgrad, d.efc.grad)
  elif m.opt.solver == types.SolverType.NEWTON:
    # h = qM + (efc_J.T * efc_D * active) @ efc_J
    if m.opt.is_sparse:
      num_blocks_ceil = ceil(m.nv / types.TILE_SIZE_JTDAJ_SPARSE)
      lower_triangle_dim = int(num_blocks_ceil * (num_blocks_ceil + 1) / 2)
      wp.launch_tiled(
        update_gradient_JTDAJ_sparse_tiled(types.TILE_SIZE_JTDAJ_SPARSE, d.njmax),
        dim=(d.nworld, lower_triangle_dim),
        inputs=[
          d.nefc,
          d.efc.J,
          d.efc.D,
          d.efc.state,
          d.efc.done,
        ],
        outputs=[h],
        block_dim=m.block_dim.update_gradient_JTDAJ_sparse,
      )

      wp.launch(
        update_gradient_set_h_qM_lower_sparse,
        dim=(d.nworld, m.qM_fullm_i.size),
        inputs=[m.qM_fullm_i, m.qM_fullm_j, d.qM, d.efc.done],
        outputs=[h],
      )
    else:
      nv_padded = d.efc.J.shape[2]
      wp.launch_tiled(
        update_gradient_JTDAJ_dense_tiled(nv_padded, types.TILE_SIZE_JTDAJ_DENSE, d.njmax),
        dim=d.nworld,
        inputs=[
          d.nefc,
          d.qM,
          d.efc.J,
          d.efc.D,
          d.efc.state,
          d.efc.done,
        ],
        outputs=[h],
        block_dim=m.block_dim.update_gradient_JTDAJ_dense,
      )

    if m.opt.cone == types.ConeType.ELLIPTIC:
      # Optimization: launching update_gradient_JTCJ with limited number of blocks on a GPU.
      # Profiling suggests that only a fraction of blocks out of the original
      # d.njmax blocks do the actual work. It aims to minimize #CTAs with no
      # effective work. It launches with #blocks that's proportional to the number
      # of SMs on the GPU. We can now query the SM count:
      # https://github.com/NVIDIA/warp/commit/f3814e7e5459e5fd13032cf0fddb3daddd510f30

      # make dim_block and nblocks_perblock static for update_gradient_JTCJ to allow
      # loop unrolling
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
        update_gradient_JTCJ,
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
          d.efc.Jaref,
          d.efc.state,
          d.efc.done,
          d.naconmax,
          d.nacon,
          nblocks_perblock,
          dim_block,
        ],
        outputs=[h],
      )

    # TODO(team): Define good threshold for blocked vs non-blocked cholesky
    if m.nv <= _BLOCK_CHOLESKY_DIM:
      wp.launch_tiled(
        update_gradient_cholesky(m.nv),
        dim=d.nworld,
        inputs=[d.efc.grad, h, d.efc.done],
        outputs=[d.efc.Mgrad],
        block_dim=m.block_dim.update_gradient_cholesky,
      )
    else:
      wp.launch(
        padding_h,
        dim=(d.nworld, m.nv_pad - m.nv),
        inputs=[m.nv, d.efc.done],
        outputs=[h],
      )

      wp.launch_tiled(
        update_gradient_cholesky_blocked(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad),
        dim=d.nworld,
        inputs=[d.efc.grad.reshape(shape=(d.nworld, d.efc.grad.shape[1], 1)), h, d.efc.done, hfactor],
        outputs=[d.efc.Mgrad.reshape(shape=(d.nworld, d.efc.Mgrad.shape[1], 1))],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )
  else:
    raise ValueError(f"Unknown solver type: {m.opt.solver}")


@wp.kernel
def solve_prev_grad_Mgrad(
  # Data in:
  efc_grad_in: wp.array2d(dtype=float),
  efc_Mgrad_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_prev_grad_out: wp.array2d(dtype=float),
  efc_prev_Mgrad_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  efc_prev_grad_out[worldid, dofid] = efc_grad_in[worldid, dofid]
  efc_prev_Mgrad_out[worldid, dofid] = efc_Mgrad_in[worldid, dofid]


@wp.kernel
def solve_beta(
  # Model:
  nv: int,
  # Data in:
  efc_grad_in: wp.array2d(dtype=float),
  efc_Mgrad_in: wp.array2d(dtype=float),
  efc_prev_grad_in: wp.array2d(dtype=float),
  efc_prev_Mgrad_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_beta_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  beta_num = float(0.0)
  beta_den = float(0.0)
  for dofid in range(nv):
    prev_Mgrad = efc_prev_Mgrad_in[worldid][dofid]
    beta_num += efc_grad_in[worldid, dofid] * (efc_Mgrad_in[worldid, dofid] - prev_Mgrad)
    beta_den += efc_prev_grad_in[worldid, dofid] * prev_Mgrad

  efc_beta_out[worldid] = wp.max(0.0, beta_num / wp.max(types.MJ_MINVAL, beta_den))


@wp.kernel
def solve_zero_search_dot(
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_search_dot_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  efc_search_dot_out[worldid] = 0.0


@wp.kernel
def solve_search_update(
  # Model:
  opt_solver: int,
  # Data in:
  efc_Mgrad_in: wp.array2d(dtype=float),
  efc_search_in: wp.array2d(dtype=float),
  efc_beta_in: wp.array(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_search_out: wp.array2d(dtype=float),
  efc_search_dot_out: wp.array(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  search = -1.0 * efc_Mgrad_in[worldid, dofid]

  if opt_solver == types.SolverType.CG:
    search += efc_beta_in[worldid] * efc_search_in[worldid, dofid]

  efc_search_out[worldid, dofid] = search
  wp.atomic_add(efc_search_dot_out, worldid, search * search)


@wp.kernel
def solve_done(
  # Model:
  nv: int,
  opt_tolerance: wp.array(dtype=float),
  opt_iterations: int,
  stat_meaninertia: float,
  # Data in:
  efc_grad_dot_in: wp.array(dtype=float),
  efc_cost_in: wp.array(dtype=float),
  efc_prev_cost_in: wp.array(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  solver_niter_out: wp.array(dtype=int),
  efc_done_out: wp.array(dtype=bool),
  nsolving_out: wp.array(dtype=int),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  solver_niter_out[worldid] += 1
  tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]

  improvement = _rescale(nv, stat_meaninertia, efc_prev_cost_in[worldid] - efc_cost_in[worldid])
  gradient = _rescale(nv, stat_meaninertia, wp.sqrt(efc_grad_dot_in[worldid]))
  done = (improvement < tolerance) or (gradient < tolerance)
  if done or solver_niter_out[worldid] == opt_iterations:
    # if the solver has converged or the maximum number of iterations has been reached then
    # mark this world as done and remove it from the number of unconverged worlds
    efc_done_out[worldid] = True
    wp.atomic_add(nsolving_out, 0, -1)


@event_scope
def _solver_iteration(
  m: types.Model,
  d: types.Data,
  h: wp.array3d(dtype=float),
  hfactor: wp.array3d(dtype=float),
  step_size_cost: wp.array2d(dtype=float),
):
  _linesearch(m, d, step_size_cost)

  if m.opt.solver == types.SolverType.CG:
    wp.launch(
      solve_prev_grad_Mgrad,
      dim=(d.nworld, m.nv),
      inputs=[d.efc.grad, d.efc.Mgrad, d.efc.done],
      outputs=[d.efc.prev_grad, d.efc.prev_Mgrad],
    )

  _update_constraint(m, d)
  _update_gradient(m, d, h, hfactor)

  # polak-ribiere
  if m.opt.solver == types.SolverType.CG:
    wp.launch(
      solve_beta,
      dim=d.nworld,
      inputs=[m.nv, d.efc.grad, d.efc.Mgrad, d.efc.prev_grad, d.efc.prev_Mgrad, d.efc.done],
      outputs=[d.efc.beta],
    )

  wp.launch(solve_zero_search_dot, dim=(d.nworld), inputs=[d.efc.done], outputs=[d.efc.search_dot])

  wp.launch(
    solve_search_update,
    dim=(d.nworld, m.nv),
    inputs=[m.opt.solver, d.efc.Mgrad, d.efc.search, d.efc.beta, d.efc.done],
    outputs=[d.efc.search, d.efc.search_dot],
  )

  wp.launch(
    solve_done,
    dim=d.nworld,
    inputs=[
      m.nv,
      m.opt.tolerance,
      m.opt.iterations,
      m.stat.meaninertia,
      d.efc.grad_dot,
      d.efc.cost,
      d.efc.prev_cost,
      d.efc.done,
    ],
    outputs=[d.solver_niter, d.efc.done, d.nsolving],
  )


def create_context(
  m: types.Model, d: types.Data, h: wp.array3d(dtype=float), hfactor: wp.array3d(dtype=float), grad: bool = True
):
  # initialize some efc arrays
  wp.launch(
    solve_init_efc,
    dim=(d.nworld),
    outputs=[d.solver_niter, d.efc.search_dot, d.efc.cost, d.efc.done],
  )

  # jaref = d.efc_J @ d.qacc - d.efc_aref
  wp.launch(
    solve_init_jaref,
    dim=(d.nworld, d.njmax),
    inputs=[m.nv, d.nefc, d.qacc, d.efc.J, d.efc.aref],
    outputs=[d.efc.Jaref],
  )

  # Ma = qM @ qacc
  support.mul_m(m, d, d.efc.Ma, d.qacc, skip=d.efc.done)

  _update_constraint(m, d)

  if grad:
    _update_gradient(m, d, h, hfactor)


@event_scope
def solve(m: types.Model, d: types.Data):
  if d.njmax == 0 or m.nv == 0:
    wp.copy(d.qacc, d.qacc_smooth)
    d.solver_niter.fill_(0)
  else:
    _solve(m, d)


def _solve(m: types.Model, d: types.Data):
  """Finds forces that satisfy constraints."""
  if not (m.opt.disableflags & types.DisableBit.WARMSTART):
    wp.copy(d.qacc, d.qacc_warmstart)
  else:
    wp.copy(d.qacc, d.qacc_smooth)

  # Newton solver Hessian
  if m.opt.solver == types.SolverType.NEWTON:
    h = wp.zeros((d.nworld, m.nv_pad, m.nv_pad), dtype=float)
    if m.nv > _BLOCK_CHOLESKY_DIM:
      hfactor = wp.zeros((d.nworld, m.nv_pad, m.nv_pad), dtype=float)
    else:
      hfactor = wp.empty((d.nworld, 0, 0), dtype=float)
  else:
    h = wp.empty((d.nworld, 0, 0), dtype=float)
    hfactor = wp.empty((d.nworld, 0, 0), dtype=float)

  # create context
  create_context(m, d, h, hfactor, grad=True)

  # search = -Mgrad
  wp.launch(
    solve_init_search,
    dim=(d.nworld, m.nv),
    inputs=[d.efc.Mgrad],
    outputs=[d.efc.search, d.efc.search_dot],
  )

  step_size_cost = wp.empty((d.nworld, m.opt.ls_iterations if m.opt.ls_parallel else 0), dtype=float)

  if m.opt.iterations != 0 and m.opt.graph_conditional:
    # Note: the iteration kernel (indicated by while_body) is repeatedly launched
    # as long as condition_iteration is not zero.
    # condition_iteration is a warp array of size 1 and type int, it counts the number
    # of worlds that are not converged, it becomes 0 when all worlds are converged.
    # When the number of iterations reaches m.opt.iterations, solver_niter
    # becomes zero and all worlds are marked as converged to avoid an infinite loop.
    # note: we only launch the iteration kernel if everything is not done
    d.nsolving.fill_(d.nworld)
    wp.capture_while(d.nsolving, while_body=_solver_iteration, m=m, d=d, h=h, hfactor=hfactor, step_size_cost=step_size_cost)
  else:
    # This branch is mostly for when JAX is used as it is currently not compatible
    # with CUDA graph conditional.
    # It should be removed when JAX becomes compatible.
    for _ in range(m.opt.iterations):
      _solver_iteration(m, d, h, hfactor, step_size_cost)
