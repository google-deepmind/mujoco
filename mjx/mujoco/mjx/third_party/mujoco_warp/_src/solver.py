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

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel


@wp.func
def _rescale(nv: int, stat_meaninertia: float, value: float) -> float:
  return value / (stat_meaninertia * float(wp.max(1, nv)))


@wp.func
def _in_bracket(x: wp.vec3, y: wp.vec3) -> bool:
  return (x[1] < y[1] and y[1] < 0.0) or (x[1] > y[1] and y[1] > 0.0)


@wp.func
def _eval_pt(quad: wp.vec3, alpha: float) -> wp.vec3:
  return wp.vec3(
    alpha * alpha * quad[2] + alpha * quad[1] + quad[0],
    2.0 * alpha * quad[2] + quad[1],
    2.0 * quad[2],
  )


@wp.func
def _eval_pt_elliptic(
  # In:
  impratio: float,
  friction: types.vec5,
  u0: float,
  uu: float,
  uv: float,
  vv: float,
  jv: float,
  D: float,
  quad: wp.vec3,
  alpha: float,
) -> wp.vec3:
  mu = friction[0] / wp.sqrt(impratio)
  v0 = jv * mu
  n = u0 + alpha * v0
  tsqr = uu + alpha * (2.0 * uv + alpha * vv)
  t = wp.sqrt(tsqr)  # tangential force

  bottom_zone = ((tsqr <= 0.0) and (n < 0)) or ((tsqr > 0.0) and ((mu * n + t) <= 0.0))
  middle_zone = (tsqr > 0) and (n < (mu * t)) and ((mu * n + t) > 0.0)

  # elliptic bottom zone: quadratic cose
  if bottom_zone:
    pt = _eval_pt(quad, alpha)
  else:
    pt = wp.vec3(0.0)

  # elliptic middle zone
  if t == 0.0:
    t += types.MJ_MINVAL

  if tsqr == 0.0:
    tsqr += types.MJ_MINVAL

  n1 = v0
  t1 = (uv + alpha * vv) / t
  t2 = vv / t - (uv + alpha * vv) * t1 / tsqr

  if middle_zone:
    mu2 = mu * mu
    dm = D / wp.max(mu2 * (1.0 + mu2), types.MJ_MINVAL)
    nmt = n - mu * t
    n1mut1 = n1 - mu * t1

    pt += wp.vec3(
      0.5 * dm * nmt * nmt,
      dm * nmt * n1mut1,
      dm * (n1mut1 * n1mut1 - nmt * mu * t2),
    )

  return pt


@wp.kernel
def linesearch_iterative_init_gtol_p0_gauss(
  # Model:
  nv: int,
  opt_tolerance: wp.array(dtype=float),
  opt_ls_tolerance: wp.array(dtype=float),
  stat_meaninertia: float,
  # Data in:
  efc_search_dot_in: wp.array(dtype=float),
  efc_quad_gauss_in: wp.array(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_gtol_out: wp.array(dtype=float),
  efc_p0_out: wp.array(dtype=wp.vec3),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  tolerance = opt_tolerance[worldid]
  ls_tolerance = opt_ls_tolerance[worldid]
  snorm = wp.math.sqrt(efc_search_dot_in[worldid])
  scale = stat_meaninertia * wp.float(wp.max(1, nv))
  efc_gtol_out[worldid] = tolerance * ls_tolerance * snorm * scale

  quad = efc_quad_gauss_in[worldid]
  efc_p0_out[worldid] = wp.vec3(quad[0], quad[1], 2.0 * quad[2])


@wp.kernel
def linesearch_iterative_init_p0_elliptic0(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_condim_in: wp.array2d(dtype=int),
  # Data out:
  efc_p0_out: wp.array(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  active = efc_Jaref_in[worldid, efcid] < 0.0

  nef = ne_in[worldid] + nf_in[worldid]
  nefl = nef + nl_in[worldid]
  if efcid < nef:
    active = True
  elif efcid >= nefl and efc_condim_in[worldid, efcid] > 1:
    active = False

  if active:
    quad = efc_quad_in[worldid, efcid]
    wp.atomic_add(efc_p0_out, worldid, wp.vec3(quad[0], quad[1], 2.0 * quad[2]))


@wp.kernel
def linesearch_iterative_init_p0_elliptic1(
  # Model:
  opt_impratio: wp.array(dtype=float),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_u_in: wp.array(dtype=types.vec6),
  efc_uu_in: wp.array(dtype=float),
  efc_uv_in: wp.array(dtype=float),
  efc_vv_in: wp.array(dtype=float),
  # Data out:
  efc_p0_out: wp.array(dtype=wp.vec3),
):
  conid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]
  if efc_done_in[worldid]:
    return

  if contact_dim_in[conid] < 2:
    return

  efcid = contact_efc_address_in[conid, 0]

  pt = _eval_pt_elliptic(
    opt_impratio[worldid],
    contact_friction_in[conid],
    efc_u_in[conid][0],
    efc_uu_in[conid],
    efc_uv_in[conid],
    efc_vv_in[conid],
    efc_jv_in[worldid, efcid],
    efc_D_in[worldid, efcid],
    efc_quad_in[worldid, efcid],
    0.0,
  )

  wp.atomic_add(efc_p0_out, worldid, pt)


@wp.kernel
def linesearch_iterative_init_p0_pyramidal(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_p0_out: wp.array(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  if efc_Jaref_in[worldid, efcid] >= 0.0 and efcid >= ne_in[worldid] + nf_in[worldid]:
    return

  quad = efc_quad_in[worldid, efcid]

  wp.atomic_add(efc_p0_out, worldid, wp.vec3(quad[0], quad[1], 2.0 * quad[2]))


@wp.kernel
def linesearch_iterative_init_lo_gauss(
  # Data in:
  efc_quad_gauss_in: wp.array(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_p0_in: wp.array(dtype=wp.vec3),
  # Data out:
  efc_lo_out: wp.array(dtype=wp.vec3),
  efc_lo_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  p0 = efc_p0_in[worldid]
  alpha = -math.safe_div(p0[1], p0[2])
  efc_lo_out[worldid] = _eval_pt(efc_quad_gauss_in[worldid], alpha)
  efc_lo_alpha_out[worldid] = alpha


@wp.kernel
def linesearch_iterative_init_lo_elliptic0(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_lo_alpha_in: wp.array(dtype=float),
  efc_condim_in: wp.array2d(dtype=int),
  # Data out:
  efc_lo_out: wp.array(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  alpha = efc_lo_alpha_in[worldid]

  active = efc_Jaref_in[worldid, efcid] + alpha * efc_jv_in[worldid, efcid] < 0.0

  nef = ne_in[worldid] + nf_in[worldid]
  nefl = nef + nl_in[worldid]
  if efcid < nef:
    active = True
  elif efcid >= nefl and efc_condim_in[worldid, efcid] > 1:
    active = False

  if active:
    wp.atomic_add(efc_lo_out, worldid, _eval_pt(efc_quad_in[worldid, efcid], alpha))


@wp.kernel
def linesearch_iterative_init_lo_elliptic1(
  # Model:
  opt_impratio: wp.array(dtype=float),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_lo_alpha_in: wp.array(dtype=float),
  efc_u_in: wp.array(dtype=types.vec6),
  efc_uu_in: wp.array(dtype=float),
  efc_uv_in: wp.array(dtype=float),
  efc_vv_in: wp.array(dtype=float),
  # Data out:
  efc_lo_out: wp.array(dtype=wp.vec3),
):
  conid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]
  if efc_done_in[worldid]:
    return

  if contact_dim_in[conid] < 2:
    return

  efcid = contact_efc_address_in[conid, 0]
  alpha = efc_lo_alpha_in[worldid]
  pt = _eval_pt_elliptic(
    opt_impratio[worldid],
    contact_friction_in[conid],
    efc_u_in[conid][0],
    efc_uu_in[conid],
    efc_uv_in[conid],
    efc_vv_in[conid],
    efc_jv_in[worldid, efcid],
    efc_D_in[worldid, efcid],
    efc_quad_in[worldid, efcid],
    alpha,
  )
  wp.atomic_add(efc_lo_out, worldid, pt)


@wp.kernel
def linesearch_iterative_init_lo_pyramidal(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_lo_alpha_in: wp.array(dtype=float),
  # Data out:
  efc_lo_out: wp.array(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  alpha = efc_lo_alpha_in[worldid]

  if efc_Jaref_in[worldid, efcid] + alpha * efc_jv_in[worldid, efcid] < 0.0 or (efcid < ne_in[worldid] + nf_in[worldid]):
    wp.atomic_add(efc_lo_out, worldid, _eval_pt(efc_quad_in[worldid, efcid], alpha))


@wp.kernel
def linesearch_iterative_init_bounds(
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  efc_p0_in: wp.array(dtype=wp.vec3),
  efc_lo_in: wp.array(dtype=wp.vec3),
  efc_lo_alpha_in: wp.array(dtype=float),
  # Data out:
  efc_lo_out: wp.array(dtype=wp.vec3),
  efc_lo_alpha_out: wp.array(dtype=float),
  efc_hi_out: wp.array(dtype=wp.vec3),
  efc_hi_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  p0 = efc_p0_in[worldid]
  lo = efc_lo_in[worldid]
  lo_alpha = efc_lo_alpha_in[worldid]
  lo_less = lo[1] < p0[1]

  efc_lo_out[worldid] = wp.where(lo_less, lo, p0)
  efc_lo_alpha_out[worldid] = wp.where(lo_less, lo_alpha, 0.0)
  efc_hi_out[worldid] = wp.where(lo_less, p0, lo)
  efc_hi_alpha_out[worldid] = wp.where(lo_less, 0.0, lo_alpha)


@wp.kernel
def linesearch_iterative_next_alpha_gauss(
  # Data in:
  efc_quad_gauss_in: wp.array(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_ls_done_in: wp.array(dtype=bool),
  efc_lo_in: wp.array(dtype=wp.vec3),
  efc_lo_alpha_in: wp.array(dtype=float),
  efc_hi_in: wp.array(dtype=wp.vec3),
  efc_hi_alpha_in: wp.array(dtype=float),
  # Data out:
  efc_lo_next_out: wp.array(dtype=wp.vec3),
  efc_lo_next_alpha_out: wp.array(dtype=float),
  efc_hi_next_out: wp.array(dtype=wp.vec3),
  efc_hi_next_alpha_out: wp.array(dtype=float),
  efc_mid_out: wp.array(dtype=wp.vec3),
  efc_mid_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_ls_done_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  quad = efc_quad_gauss_in[worldid]

  lo = efc_lo_in[worldid]
  lo_alpha = efc_lo_alpha_in[worldid]
  lo_next_alpha = lo_alpha - math.safe_div(lo[1], lo[2])
  efc_lo_next_out[worldid] = _eval_pt(quad, lo_next_alpha)
  efc_lo_next_alpha_out[worldid] = lo_next_alpha

  hi = efc_hi_in[worldid]
  hi_alpha = efc_hi_alpha_in[worldid]
  hi_next_alpha = hi_alpha - math.safe_div(hi[1], hi[2])
  efc_hi_next_out[worldid] = _eval_pt(quad, hi_next_alpha)
  efc_hi_next_alpha_out[worldid] = hi_next_alpha

  mid_alpha = 0.5 * (lo_alpha + hi_alpha)
  efc_mid_out[worldid] = _eval_pt(quad, mid_alpha)
  efc_mid_alpha_out[worldid] = mid_alpha


@wp.kernel
def linesearch_iterative_next_quad_elliptic0(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_ls_done_in: wp.array(dtype=bool),
  efc_lo_next_alpha_in: wp.array(dtype=float),
  efc_hi_next_alpha_in: wp.array(dtype=float),
  efc_mid_alpha_in: wp.array(dtype=float),
  efc_condim_in: wp.array2d(dtype=int),
  # Data out:
  efc_lo_next_out: wp.array(dtype=wp.vec3),
  efc_hi_next_out: wp.array(dtype=wp.vec3),
  efc_mid_out: wp.array(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  if efc_ls_done_in[worldid]:
    return

  nef = ne_in[worldid] + nf_in[worldid]
  nefl = nef + nl_in[worldid]

  quad = efc_quad_in[worldid, efcid]
  jaref = efc_Jaref_in[worldid, efcid]
  jv = efc_jv_in[worldid, efcid]

  alpha = efc_lo_next_alpha_in[worldid]

  active = jaref + alpha * jv < 0.0
  if efcid < nef:
    active = True
  elif efcid >= nefl and efc_condim_in[worldid, efcid] > 1:
    active = False

  if active:
    wp.atomic_add(efc_lo_next_out, worldid, _eval_pt(quad, alpha))

  alpha = efc_hi_next_alpha_in[worldid]

  active = jaref + alpha * jv < 0.0
  if efcid < nef:
    active = True
  elif efcid >= nefl and efc_condim_in[worldid, efcid] > 1:
    active = False

  if active:
    wp.atomic_add(efc_hi_next_out, worldid, _eval_pt(quad, alpha))

  alpha = efc_mid_alpha_in[worldid]

  active = jaref + alpha * jv < 0.0
  if efcid < nef:
    active = True
  elif efcid >= nefl and efc_condim_in[worldid, efcid] > 1:
    active = False

  if active:
    wp.atomic_add(efc_mid_out, worldid, _eval_pt(quad, alpha))


@wp.kernel
def linesearch_iterative_next_quad_elliptic1(
  # Model:
  opt_impratio: wp.array(dtype=float),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_lo_next_alpha_in: wp.array(dtype=float),
  efc_hi_next_alpha_in: wp.array(dtype=float),
  efc_mid_alpha_in: wp.array(dtype=float),
  efc_u_in: wp.array(dtype=types.vec6),
  efc_uu_in: wp.array(dtype=float),
  efc_uv_in: wp.array(dtype=float),
  efc_vv_in: wp.array(dtype=float),
  # Data out:
  efc_lo_next_out: wp.array(dtype=wp.vec3),
  efc_hi_next_out: wp.array(dtype=wp.vec3),
  efc_mid_out: wp.array(dtype=wp.vec3),
):
  conid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]

  if efc_done_in[worldid]:
    return

  if contact_dim_in[conid] < 2:
    return

  efcid = contact_efc_address_in[conid, 0]
  impratio = opt_impratio[worldid]
  friction = contact_friction_in[conid]
  u = efc_u_in[conid][0]
  uu = efc_uu_in[conid]
  uv = efc_uv_in[conid]
  vv = efc_vv_in[conid]
  jv = efc_jv_in[worldid, efcid]
  d = efc_D_in[worldid, efcid]
  quad = efc_quad_in[worldid, efcid]

  alpha = efc_lo_next_alpha_in[worldid]
  pt = _eval_pt_elliptic(impratio, friction, u, uu, uv, vv, jv, d, quad, alpha)
  wp.atomic_add(efc_lo_next_out, worldid, pt)

  alpha = efc_hi_next_alpha_in[worldid]
  pt = _eval_pt_elliptic(impratio, friction, u, uu, uv, vv, jv, d, quad, alpha)
  wp.atomic_add(efc_hi_next_out, worldid, pt)

  alpha = efc_mid_alpha_in[worldid]
  pt = _eval_pt_elliptic(impratio, friction, u, uu, uv, vv, jv, d, quad, alpha)
  wp.atomic_add(efc_mid_out, worldid, pt)


@wp.kernel
def linesearch_iterative_next_quad_pyramidal(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_ls_done_in: wp.array(dtype=bool),
  efc_lo_next_alpha_in: wp.array(dtype=float),
  efc_hi_next_alpha_in: wp.array(dtype=float),
  efc_mid_alpha_in: wp.array(dtype=float),
  # Data out:
  efc_lo_next_out: wp.array(dtype=wp.vec3),
  efc_hi_next_out: wp.array(dtype=wp.vec3),
  efc_mid_out: wp.array(dtype=wp.vec3),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  if efc_done_in[worldid]:
    return

  if efc_ls_done_in[worldid]:
    return

  nef_active = efcid < ne_in[worldid] + nf_in[worldid]

  quad = efc_quad_in[worldid, efcid]
  jaref = efc_Jaref_in[worldid, efcid]
  jv = efc_jv_in[worldid, efcid]

  alpha = efc_lo_next_alpha_in[worldid]
  if jaref + alpha * jv < 0.0 or nef_active:
    wp.atomic_add(efc_lo_next_out, worldid, _eval_pt(quad, alpha))

  alpha = efc_hi_next_alpha_in[worldid]
  if jaref + alpha * jv < 0.0 or nef_active:
    wp.atomic_add(efc_hi_next_out, worldid, _eval_pt(quad, alpha))

  alpha = efc_mid_alpha_in[worldid]
  if jaref + alpha * jv < 0.0 or nef_active:
    wp.atomic_add(efc_mid_out, worldid, _eval_pt(quad, alpha))


@wp.kernel
def linesearch_iterative_swap(
  # Data in:
  efc_gtol_in: wp.array(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  efc_ls_done_in: wp.array(dtype=bool),
  efc_p0_in: wp.array(dtype=wp.vec3),
  efc_lo_in: wp.array(dtype=wp.vec3),
  efc_lo_alpha_in: wp.array(dtype=float),
  efc_hi_in: wp.array(dtype=wp.vec3),
  efc_hi_alpha_in: wp.array(dtype=float),
  efc_lo_next_in: wp.array(dtype=wp.vec3),
  efc_lo_next_alpha_in: wp.array(dtype=float),
  efc_hi_next_in: wp.array(dtype=wp.vec3),
  efc_hi_next_alpha_in: wp.array(dtype=float),
  efc_mid_in: wp.array(dtype=wp.vec3),
  efc_mid_alpha_in: wp.array(dtype=float),
  # Data out:
  efc_alpha_out: wp.array(dtype=float),
  efc_ls_done_out: wp.array(dtype=bool),
  efc_lo_out: wp.array(dtype=wp.vec3),
  efc_lo_alpha_out: wp.array(dtype=float),
  efc_hi_out: wp.array(dtype=wp.vec3),
  efc_hi_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  if efc_ls_done_in[worldid]:
    return

  lo = efc_lo_in[worldid]
  lo_alpha = efc_lo_alpha_in[worldid]
  hi = efc_hi_in[worldid]
  hi_alpha = efc_hi_alpha_in[worldid]
  lo_next = efc_lo_next_in[worldid]
  lo_next_alpha = efc_lo_next_alpha_in[worldid]
  hi_next = efc_hi_next_in[worldid]
  hi_next_alpha = efc_hi_next_alpha_in[worldid]
  mid = efc_mid_in[worldid]
  mid_alpha = efc_mid_alpha_in[worldid]

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
  efc_lo_out[worldid] = lo
  efc_lo_alpha_out[worldid] = lo_alpha
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
  efc_hi_out[worldid] = hi
  efc_hi_alpha_out[worldid] = hi_alpha
  swap_hi = swap_hi_hi_next or swap_hi_mid or swap_hi_lo_next

  # if we did not adjust the interval, we are done
  # also done if either low or hi slope is nearly flat
  gtol = efc_gtol_in[worldid]
  efc_ls_done_out[worldid] = (not swap_lo and not swap_hi) or (lo[1] < 0 and lo[1] > -gtol) or (hi[1] > 0 and hi[1] < gtol)

  # update alpha if we have an improvement
  p0 = efc_p0_in[worldid]
  alpha = 0.0
  improved = lo[0] < p0[0] or hi[0] < p0[0]
  lo_better = lo[0] < hi[0]
  alpha = wp.where(improved and lo_better, lo_alpha, alpha)
  alpha = wp.where(improved and not lo_better, hi_alpha, alpha)
  efc_alpha_out[worldid] = alpha


def _linesearch_iterative(m: types.Model, d: types.Data):
  """Iterative linesearch."""
  d.efc.ls_done.zero_()

  wp.launch(
    linesearch_iterative_init_gtol_p0_gauss,
    dim=(d.nworld,),
    inputs=[
      m.nv, m.opt.tolerance, m.opt.ls_tolerance, m.stat.meaninertia, d.efc.search_dot,
      d.efc.quad_gauss, d.efc.done
    ],
    outputs=[d.efc.gtol, d.efc.p0])  # fmt: skip

  if m.opt.cone == types.ConeType.ELLIPTIC:
    wp.launch(
      linesearch_iterative_init_p0_elliptic0,
      dim=(d.nworld, d.njmax,),
      inputs=[
        d.ne, d.nf, d.nl, d.nefc, d.efc.Jaref, d.efc.quad, d.efc.done,
        d.efc.condim
      ],
      outputs=[d.efc.p0])  # fmt: skip
    wp.launch(
      linesearch_iterative_init_p0_elliptic1,
      dim=(d.nconmax),
      inputs=[
        m.opt.impratio, d.ncon, d.contact.friction, d.contact.dim,
        d.contact.efc_address, d.contact.worldid, d.efc.D, d.efc.jv, d.efc.quad,
        d.efc.done, d.efc.u, d.efc.uu, d.efc.uv, d.efc.vv
      ],
      outputs=[d.efc.p0])  # fmt: skip
  else:
    wp.launch(
      linesearch_iterative_init_p0_pyramidal,
      dim=(d.nworld, d.njmax,),
      inputs=[
        d.ne, d.nf, d.nefc, d.efc.Jaref, d.efc.quad, d.efc.done
      ], outputs=[d.efc.p0])  # fmt: skip

  wp.launch(
    linesearch_iterative_init_lo_gauss,
    dim=(d.nworld,),
    inputs=[
      d.efc.quad_gauss, d.efc.done, d.efc.p0
    ],
    outputs=[d.efc.lo, d.efc.lo_alpha])  # fmt: skip

  if m.opt.cone == types.ConeType.ELLIPTIC:
    wp.launch(
      linesearch_iterative_init_lo_elliptic0,
      dim=(d.nworld, d.njmax,),
      inputs=[
        d.ne, d.nf, d.nl, d.nefc, d.efc.Jaref, d.efc.jv, d.efc.quad,
        d.efc.done, d.efc.lo_alpha, d.efc.condim
      ],
      outputs=[d.efc.lo])  # fmt: skip
    wp.launch(
      linesearch_iterative_init_lo_elliptic1,
      dim=(d.nconmax),
      inputs=[
        m.opt.impratio, d.ncon, d.contact.friction, d.contact.dim,
        d.contact.efc_address, d.contact.worldid, d.efc.D, d.efc.jv, d.efc.quad,
        d.efc.done, d.efc.lo_alpha, d.efc.u, d.efc.uu, d.efc.uv, d.efc.vv
      ],
      outputs=[d.efc.lo])  # fmt: skip
  else:
    wp.launch(
      linesearch_iterative_init_lo_pyramidal,
      dim=(d.nworld, d.njmax,),
      inputs=[
        d.ne, d.nf, d.nefc, d.efc.Jaref, d.efc.jv,
        d.efc.quad, d.efc.done, d.efc.lo_alpha
      ],
      outputs=[d.efc.lo])  # fmt: skip

  # set the lo/hi interval bounds

  wp.launch(
    linesearch_iterative_init_bounds,
    dim=(d.nworld,),
    inputs=[d.efc.done, d.efc.p0, d.efc.lo, d.efc.lo_alpha],
    outputs=[d.efc.lo, d.efc.lo_alpha, d.efc.hi, d.efc.hi_alpha])  # fmt: skip

  for _ in range(m.opt.ls_iterations):
    # NOTE: we always launch ls_iterations kernels, but the kernels may early exit if done
    # is true. this preserves cudagraph requirements (no dynamic kernel launching) at the
    # expense of extra launches
    wp.launch(
      linesearch_iterative_next_alpha_gauss,
      dim=(d.nworld,),
      inputs=[
        d.efc.quad_gauss, d.efc.done, d.efc.ls_done, d.efc.lo, d.efc.lo_alpha, d.efc.hi, d.efc.hi_alpha
      ],
      outputs=[
        d.efc.lo_next, d.efc.lo_next_alpha, d.efc.hi_next, d.efc.hi_next_alpha, d.efc.mid, d.efc.mid_alpha
      ])  # fmt: skip

    if m.opt.cone == types.ConeType.ELLIPTIC:
      wp.launch(
        linesearch_iterative_next_quad_elliptic0,
        dim=(d.nworld, d.njmax,),
        inputs=[
          d.ne, d.nf, d.nl, d.nefc, d.efc.Jaref, d.efc.jv, d.efc.quad, d.efc.done, d.efc.ls_done,
          d.efc.lo_next_alpha, d.efc.hi_next_alpha, d.efc.mid_alpha, d.efc.condim
        ],
        outputs=[d.efc.lo_next, d.efc.hi_next, d.efc.mid])  # fmt: skip
      wp.launch(
        linesearch_iterative_next_quad_elliptic1,
        dim=(d.nconmax),
        inputs=[
          m.opt.impratio, d.ncon, d.contact.friction, d.contact.dim, d.contact.efc_address, d.contact.worldid, d.efc.D,
          d.efc.jv, d.efc.quad, d.efc.done, d.efc.lo_next_alpha, d.efc.hi_next_alpha, d.efc.mid_alpha, d.efc.u, d.efc.uu,
          d.efc.uv, d.efc.vv
        ],
        outputs=[d.efc.lo_next, d.efc.hi_next, d.efc.mid])  # fmt: skip
    else:
      wp.launch(
        linesearch_iterative_next_quad_pyramidal,
        dim=(d.nworld, d.njmax,),
        inputs=[
          d.ne, d.nf, d.nefc, d.efc.Jaref, d.efc.jv, d.efc.quad, d.efc.done, d.efc.ls_done,
          d.efc.lo_next_alpha, d.efc.hi_next_alpha, d.efc.mid_alpha
        ],
        outputs=[d.efc.lo_next, d.efc.hi_next, d.efc.mid])  # fmt: skip

    wp.launch(
      linesearch_iterative_swap,
      dim=(d.nworld,),
      inputs=[
        d.efc.gtol, d.efc.done, d.efc.ls_done, d.efc.p0, d.efc.lo, d.efc.lo_alpha, d.efc.hi, d.efc.hi_alpha,
        d.efc.lo_next, d.efc.lo_next_alpha, d.efc.hi_next, d.efc.hi_next_alpha, d.efc.mid, d.efc.mid_alpha
      ],
      outputs=[
        d.efc.alpha, d.efc.ls_done, d.efc.lo, d.efc.lo_alpha, d.efc.hi, d.efc.hi_alpha
      ])  # fmt: skip


@wp.kernel
def linesearch_parallel_fused(
  # Model:
  nlsp: int,
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_quad_gauss_in: wp.array(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_cost_candidate_out: wp.array2d(dtype=float),
):
  worldid, alphaid = wp.tid()

  if efc_done_in[worldid]:
    return

  efc_quad_total_candidate = efc_quad_gauss_in[worldid]

  alpha = float(alphaid) / float(nlsp - 1)
  ne = ne_in[worldid]
  nf = nf_in[worldid]
  for efcid in range(nefc_in[worldid]):
    Jaref = efc_Jaref_in[worldid, efcid]
    jv = efc_jv_in[worldid, efcid]
    quad = efc_quad_in[worldid, efcid]

    if (Jaref + alpha * jv) < 0.0 or (efcid < ne + nf):
      efc_quad_total_candidate += quad

  alpha_sq = alpha * alpha
  quad_total0 = efc_quad_total_candidate[0]
  quad_total1 = efc_quad_total_candidate[1]
  quad_total2 = efc_quad_total_candidate[2]

  efc_cost_candidate_out[worldid, alphaid] = alpha_sq * quad_total2 + alpha * quad_total1 + quad_total0


@wp.kernel
def linesearch_parallel_best_alpha(
  # Model:
  nlsp: int,
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  efc_cost_candidate_in: wp.array2d(dtype=float),
  # Data out:
  efc_alpha_out: wp.array(dtype=float),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  # TODO(team): investigate alternatives to wp.argmin
  # TODO(thowell): how did this use to work?
  bestid = int(0)
  best_cost = float(wp.inf)
  for i in range(nlsp):
    cost = efc_cost_candidate_in[worldid, i]
    if cost < best_cost:
      best_cost = cost
      bestid = i

  efc_alpha_out[worldid] = float(bestid) / float(nlsp - 1)


def _linesearch_parallel(m: types.Model, d: types.Data):
  wp.launch(
    linesearch_parallel_fused,
    dim=(d.nworld, m.nlsp),
    inputs=[
      m.nlsp,
      d.ne,
      d.nf,
      d.nefc,
      d.efc.Jaref,
      d.efc.jv,
      d.efc.quad,
      d.efc.quad_gauss,
      d.efc.done,
    ],
    outputs=[d.efc.cost_candidate],
  )

  wp.launch(
    linesearch_parallel_best_alpha,
    dim=(d.nworld),
    inputs=[m.nlsp, d.efc.done, d.efc.cost_candidate],
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
  @nested_kernel
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
def linesearch_init_quad_gauss(
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
def linesearch_init_quad(
  # Data in:
  nefc_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_frictionloss_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_jv_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # In:
  disable_floss: bool,
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
  floss = efc_frictionloss_in[worldid, efcid]

  if floss > 0.0 and not disable_floss:
    rf = math.safe_div(floss, efc_D)
    if Jaref <= -rf:
      efc_quad_out[worldid, efcid] = wp.vec3(floss * (-0.5 * rf - Jaref), -floss * jv, 0.0)
      return
    elif Jaref >= rf:
      efc_quad_out[worldid, efcid] = wp.vec3(floss * (-0.5 * rf + Jaref), floss * jv, 0.0)
      return

  efc_quad_out[worldid, efcid] = wp.vec3(0.5 * Jaref * Jaref * efc_D, jv * Jaref * efc_D, 0.5 * jv * jv * efc_D)


@wp.kernel
def linesearch_quad_elliptic(
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_jv_in: wp.array2d(dtype=float),
  efc_quad_in: wp.array2d(dtype=wp.vec3),
  efc_done_in: wp.array(dtype=bool),
  efc_u_in: wp.array(dtype=types.vec6),
  # Data out:
  efc_quad_out: wp.array2d(dtype=wp.vec3),
  efc_uv_out: wp.array(dtype=float),
  efc_vv_out: wp.array(dtype=float),
):
  conid, dimid = wp.tid()
  dimid += 1

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]
  if efc_done_in[worldid]:
    return

  condim = contact_dim_in[conid]

  if condim == 1 or (dimid >= condim):
    return

  efcid0 = contact_efc_address_in[conid, 0]
  efcid = contact_efc_address_in[conid, dimid]

  # complete vector quadratic (for bottom zone)
  wp.atomic_add(efc_quad_out, worldid, efcid0, efc_quad_in[worldid, efcid])

  # rescale to make primal cone circular
  u = efc_u_in[conid][dimid]
  v = efc_jv_in[worldid, efcid] * contact_friction_in[conid][dimid - 1]
  wp.atomic_add(efc_uv_out, conid, u * v)
  wp.atomic_add(efc_vv_out, conid, v * v)


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
def _linesearch(m: types.Model, d: types.Data):
  # mv = qM @ search
  support.mul_m(m, d, d.efc.mv, d.efc.search, d.efc.done)

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
    linesearch_init_quad_gauss,
    dim=(d.nworld),
    inputs=[m.nv, d.qfrc_smooth, d.efc.Ma, d.efc.search, d.efc.gauss, d.efc.mv, d.efc.done],
    outputs=[d.efc.quad_gauss],
  )

  # quad = [0.5 * Jaref * Jaref * efc_D, jv * Jaref * efc_D, 0.5 * jv * jv * efc_D]

  disable_floss = m.opt.disableflags & types.DisableBit.FRICTIONLOSS
  wp.launch(
    linesearch_init_quad,
    dim=(d.nworld, d.njmax),
    inputs=[
      d.nefc,
      d.efc.D,
      d.efc.frictionloss,
      d.efc.Jaref,
      d.efc.jv,
      d.efc.done,
      disable_floss,
    ],
    outputs=[d.efc.quad],
  )

  if m.opt.cone == types.ConeType.ELLIPTIC:
    d.efc.uv.zero_()
    d.efc.vv.zero_()
    wp.launch(
      linesearch_quad_elliptic,
      dim=(d.nconmax, m.condim_max - 1),
      inputs=[
        d.ncon,
        d.contact.friction,
        d.contact.dim,
        d.contact.efc_address,
        d.contact.worldid,
        d.efc.jv,
        d.efc.quad,
        d.efc.done,
        d.efc.u,
      ],
      outputs=[d.efc.quad, d.efc.uv, d.efc.vv],
    )

  if m.opt.ls_parallel:
    _linesearch_parallel(m, d)
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
def update_constraint_efc_pyramidal(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_frictionloss_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  # In:
  disable_floss: int,
  # Data out:
  efc_force_out: wp.array2d(dtype=float),
  efc_cost_out: wp.array(dtype=float),
  efc_active_out: wp.array2d(dtype=bool),
):
  worldid, efcid = wp.tid()

  if efcid >= nefc_in[worldid]:
    return

  efc_D = efc_D_in[worldid, efcid]
  Jaref = efc_Jaref_in[worldid, efcid]

  cost = 0.5 * efc_D * Jaref * Jaref
  efc_force = -efc_D * Jaref

  ne = ne_in[worldid]
  nf = nf_in[worldid]

  if efcid < ne:
    # equality
    pass
  elif efcid < ne + nf and not disable_floss:
    # friction
    f = efc_frictionloss_in[worldid, efcid]
    if f > 0.0:
      rf = math.safe_div(f, efc_D)
      if Jaref <= -rf:
        efc_force_out[worldid, efcid] = f
        efc_active_out[worldid, efcid] = False
        wp.atomic_add(efc_cost_out, worldid, -0.5 * rf - Jaref)
        return
      elif Jaref >= rf:
        efc_force_out[worldid, efcid] = -f
        efc_active_out[worldid, efcid] = False
        wp.atomic_add(efc_cost_out, worldid, -0.5 * rf + Jaref)
        return
  else:
    # limit, contact
    if Jaref >= 0.0:
      efc_force_out[worldid, efcid] = 0.0
      efc_active_out[worldid, efcid] = False
      return

  efc_force_out[worldid, efcid] = efc_force
  efc_active_out[worldid, efcid] = True
  wp.atomic_add(efc_cost_out, worldid, cost)


@wp.kernel
def update_constraint_u_elliptic(
  # Model:
  opt_impratio: wp.array(dtype=float),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_u_out: wp.array(dtype=types.vec6),
  efc_uu_out: wp.array(dtype=float),
  efc_condim_out: wp.array2d(dtype=int),
):
  conid, dimid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]
  if efc_done_in[worldid]:
    return

  efcid = contact_efc_address_in[conid, dimid]

  condim = contact_dim_in[conid]
  efc_condim_out[worldid, efcid] = condim

  if condim == 1:
    return

  if dimid < condim:
    if dimid == 0:
      fri = contact_friction_in[conid][0] / wp.sqrt(opt_impratio[worldid])
    else:
      fri = contact_friction_in[conid][dimid - 1]
    u = efc_Jaref_in[worldid, efcid] * fri
    efc_u_out[conid][dimid] = u
    if dimid > 0:
      wp.atomic_add(efc_uu_out, conid, u * u)


@wp.kernel
def update_constraint_active_elliptic_bottom_zone(
  # Model:
  opt_impratio: wp.array(dtype=float),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_done_in: wp.array(dtype=bool),
  efc_u_in: wp.array(dtype=types.vec6),
  efc_uu_in: wp.array(dtype=float),
  # Data out:
  efc_active_out: wp.array2d(dtype=bool),
):
  conid, dimid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]
  if efc_done_in[worldid]:
    return

  condim = contact_dim_in[conid]
  if condim == 1:
    return

  mu = contact_friction_in[conid][0] / wp.sqrt(opt_impratio[worldid])
  n = efc_u_in[conid][0]
  tt = efc_uu_in[conid]
  if tt <= 0.0:
    t = 0.0
  else:
    t = wp.sqrt(tt)

  # bottom zone: quadratic
  bottom_zone = ((t <= 0.0) and (n < 0.0)) or ((t > 0.0) and ((mu * n + t) <= 0.0))

  # update active
  efcid = contact_efc_address_in[conid, dimid]
  efc_active_out[worldid, efcid] = bottom_zone


@wp.kernel
def update_constraint_efc_elliptic0(
  # Data in:
  ne_in: wp.array(dtype=int),
  nf_in: wp.array(dtype=int),
  nl_in: wp.array(dtype=int),
  nefc_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_frictionloss_in: wp.array2d(dtype=float),
  efc_Jaref_in: wp.array2d(dtype=float),
  efc_active_in: wp.array2d(dtype=bool),
  efc_done_in: wp.array(dtype=bool),
  # In:
  disable_floss: int,
  # Data out:
  efc_force_out: wp.array2d(dtype=float),
  efc_cost_out: wp.array(dtype=float),
  efc_active_out: wp.array2d(dtype=bool),
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
  nl = nl_in[worldid]

  if efcid < ne:
    # equality
    efc_active_out[worldid, efcid] = True
  elif efcid < ne + nf and not disable_floss:
    # friction
    f = efc_frictionloss_in[worldid, efcid]
    if f > 0.0:
      rf = math.safe_div(f, efc_D)
      if Jaref <= -rf:
        efc_force_out[worldid, efcid] = f
        efc_active_out[worldid, efcid] = False
        wp.atomic_add(efc_cost_out, worldid, -0.5 * rf - Jaref)
        return
      elif Jaref >= rf:
        efc_force_out[worldid, efcid] = -f
        efc_active_out[worldid, efcid] = False
        wp.atomic_add(efc_cost_out, worldid, -0.5 * rf + Jaref)
        return
  elif efcid < ne + nf + nl:
    # limits
    if Jaref < 0.0:
      efc_active_out[worldid, efcid] = True
    else:
      efc_force_out[worldid, efcid] = 0.0
      efc_active_out[worldid, efcid] = False
      return
  else:
    # contact
    if not efc_active_in[worldid, efcid]:  # calculated by solve_active_elliptic_bottom_zone
      efc_force_out[worldid, efcid] = 0.0
      return

  efc_force_out[worldid, efcid] = -efc_D * Jaref
  wp.atomic_add(efc_cost_out, worldid, 0.5 * efc_D * Jaref * Jaref)


@wp.kernel
def update_constraint_efc_elliptic1(
  # Model:
  opt_impratio: wp.array(dtype=float),
  # Data in:
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_D_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  efc_u_in: wp.array(dtype=types.vec6),
  efc_uu_in: wp.array(dtype=float),
  # Data out:
  efc_force_out: wp.array2d(dtype=float),
  efc_cost_out: wp.array(dtype=float),
):
  conid, dimid = wp.tid()

  if conid >= ncon_in[0]:
    return

  worldid = contact_worldid_in[conid]
  if efc_done_in[worldid]:
    return

  condim = contact_dim_in[conid]

  if condim == 1 or dimid >= condim:
    return

  friction = contact_friction_in[conid]
  efcid = contact_efc_address_in[conid, dimid]

  mu = friction[0] / wp.sqrt(opt_impratio[worldid])
  n = efc_u_in[conid][0]
  tt = efc_uu_in[conid]
  if tt <= 0.0:
    t = 0.0
  else:
    t = wp.sqrt(tt)

  # middle zone: cone
  middle_zone = (t > 0.0) and (n < (mu * t)) and ((mu * n + t) > 0.0)

  # tangent and friction for middle zone:
  if middle_zone:
    efcid0 = contact_efc_address_in[conid, 0]
    mu2 = mu * mu
    dm = efc_D_in[worldid, efcid0] / wp.max(mu2 * float(1.0 + mu2), types.MJ_MINVAL)

    nmt = n - mu * t

    force = -dm * nmt * mu
    if dimid > 0:
      force_fri = -force / t
      force_fri *= efc_u_in[conid][dimid] * friction[dimid - 1]
      efc_force_out[worldid, efcid] += force_fri
    else:
      efc_force_out[worldid, efcid] += force
      worldid = contact_worldid_in[conid]
      wp.atomic_add(efc_cost_out, worldid, 0.5 * dm * nmt * nmt)


@wp.kernel
def update_constraint_zero_qfrc_constraint(
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  qfrc_constraint_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  qfrc_constraint_out[worldid, dofid] = 0.0


@wp.kernel
def update_constraint_init_qfrc_constraint(
  # Data in:
  nefc_in: wp.array(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  efc_force_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  qfrc_constraint_out: wp.array2d(dtype=float),
):
  worldid, dofid = wp.tid()

  if efc_done_in[worldid]:
    return

  sum_qfrc = float(0.0)
  for efcid in range(nefc_in[worldid]):
    efc_J = efc_J_in[worldid, efcid, dofid]
    force = efc_force_in[worldid, efcid]
    sum_qfrc += efc_J * force

  qfrc_constraint_out[worldid, dofid] += sum_qfrc


@cache_kernel
def update_constraint_gauss_cost(nv: int, dofs_per_thread: int):
  @nested_kernel
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

  disable_floss = m.opt.disableflags & types.DisableBit.FRICTIONLOSS

  wp.launch(
    update_constraint_init_cost,
    dim=(d.nworld),
    inputs=[d.efc.cost, d.efc.done],
    outputs=[d.efc.gauss, d.efc.cost, d.efc.prev_cost],
  )

  if m.opt.cone == types.ConeType.PYRAMIDAL:
    wp.launch(
      update_constraint_efc_pyramidal,
      dim=(d.nworld, d.njmax),
      inputs=[
        d.ne,
        d.nf,
        d.nefc,
        d.efc.D,
        d.efc.frictionloss,
        d.efc.Jaref,
        disable_floss,
      ],
      outputs=[d.efc.force, d.efc.cost, d.efc.active],
    )
  elif m.opt.cone == types.ConeType.ELLIPTIC:
    d.efc.uu.zero_()
    d.efc.active.zero_()
    d.efc.condim.fill_(-1)
    wp.launch(
      update_constraint_u_elliptic,
      dim=(d.nconmax, m.condim_max),
      inputs=[
        m.opt.impratio,
        d.ncon,
        d.contact.friction,
        d.contact.dim,
        d.contact.efc_address,
        d.contact.worldid,
        d.efc.Jaref,
        d.efc.done,
      ],
      outputs=[d.efc.u, d.efc.uu, d.efc.condim],
    )
    wp.launch(
      update_constraint_active_elliptic_bottom_zone,
      dim=(d.nconmax, m.condim_max),
      inputs=[
        m.opt.impratio,
        d.ncon,
        d.contact.friction,
        d.contact.dim,
        d.contact.efc_address,
        d.contact.worldid,
        d.efc.done,
        d.efc.u,
        d.efc.uu,
      ],
      outputs=[d.efc.active],
    )
    wp.launch(
      update_constraint_efc_elliptic0,
      dim=(d.nworld, d.njmax),
      inputs=[
        d.ne,
        d.nf,
        d.nl,
        d.nefc,
        d.efc.D,
        d.efc.frictionloss,
        d.efc.Jaref,
        d.efc.active,
        d.efc.done,
        disable_floss,
      ],
      outputs=[d.efc.force, d.efc.cost, d.efc.active],
    )
    wp.launch(
      update_constraint_efc_elliptic1,
      dim=(d.nconmax, m.condim_max),
      inputs=[
        m.opt.impratio,
        d.ncon,
        d.contact.friction,
        d.contact.dim,
        d.contact.efc_address,
        d.contact.worldid,
        d.efc.D,
        d.efc.done,
        d.efc.u,
        d.efc.uu,
      ],
      outputs=[d.efc.force, d.efc.cost],
    )
  else:
    raise ValueError(f"Unknown cone type: {m.opt.cone}")

  # qfrc_constraint = efc_J.T @ efc_force
  wp.launch(
    update_constraint_zero_qfrc_constraint,
    dim=(d.nworld, m.nv),
    inputs=[d.efc.done],
    outputs=[d.qfrc_constraint],
  )

  wp.launch(
    update_constraint_init_qfrc_constraint,
    dim=(d.nworld, m.nv),
    inputs=[d.nefc, d.efc.J, d.efc.force, d.efc.done],
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
def update_gradient_zero_h_lower(
  # Model:
  dof_tri_row: wp.array(dtype=int),
  dof_tri_col: wp.array(dtype=int),
  # Data in:
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_h_out: wp.array3d(dtype=float),
):
  worldid, elementid = wp.tid()

  if efc_done_in[worldid]:
    return

  rowid = dof_tri_row[elementid]
  colid = dof_tri_col[elementid]
  efc_h_out[worldid, rowid, colid] = 0.0


@wp.kernel
def update_gradient_set_h_qM_lower_sparse(
  # Model:
  qM_fullm_i: wp.array(dtype=int),
  qM_fullm_j: wp.array(dtype=int),
  # Data in:
  qM_in: wp.array3d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_h_out: wp.array3d(dtype=float),
):
  worldid, elementid = wp.tid()

  if efc_done_in[worldid]:
    return

  i = qM_fullm_i[elementid]
  j = qM_fullm_j[elementid]
  efc_h_out[worldid, i, j] = qM_in[worldid, 0, elementid]


@wp.kernel
def update_gradient_copy_lower_triangle(
  # Model:
  dof_tri_row: wp.array(dtype=int),
  dof_tri_col: wp.array(dtype=int),
  # Data in:
  qM_in: wp.array3d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_h_out: wp.array3d(dtype=float),
):
  worldid, elementid = wp.tid()

  if efc_done_in[worldid]:
    return

  rowid = dof_tri_row[elementid]
  colid = dof_tri_col[elementid]
  efc_h_out[worldid, rowid, colid] = qM_in[worldid, rowid, colid]


@wp.kernel
def update_gradient_JTDAJ(
  # Data in:
  nefc_in: wp.array(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  efc_D_in: wp.array2d(dtype=float),
  efc_active_in: wp.array2d(dtype=bool),
  efc_done_in: wp.array(dtype=bool),
  # Data out:
  efc_h_out: wp.array3d(dtype=float),
):
  worldid, elementid = wp.tid()

  if efc_done_in[worldid]:
    return

  nefc = nefc_in[worldid]

  dofi = (int(sqrt(float(1 + 8 * elementid))) - 1) // 2
  dofj = elementid - (dofi * (dofi + 1)) // 2

  sum_h = float(0.0)
  efc_D = efc_D_in[worldid, 0]
  active = efc_active_in[worldid, 0]
  efc_Ji = efc_J_in[worldid, 0, dofi]
  efc_Jj = efc_J_in[worldid, 0, dofj]
  for efcid in range(nefc - 1):
    # TODO(team): sparse efc_J
    sum_h += efc_Ji * efc_Jj * efc_D * float(active)

    jj = efcid + 1
    efc_D = efc_D_in[worldid, jj]
    active = efc_active_in[worldid, jj]
    efc_Ji = efc_J_in[worldid, jj, dofi]
    efc_Jj = efc_J_in[worldid, jj, dofj]

  sum_h += efc_Ji * efc_Jj * efc_D * float(active)
  efc_h_out[worldid, dofi, dofj] += sum_h


@wp.kernel
def update_gradient_JTCJ(
  # Model:
  opt_impratio: wp.array(dtype=float),
  dof_tri_row: wp.array(dtype=int),
  dof_tri_col: wp.array(dtype=int),
  # Data in:
  nconmax_in: int,
  ncon_in: wp.array(dtype=int),
  contact_friction_in: wp.array(dtype=types.vec5),
  contact_dim_in: wp.array(dtype=int),
  contact_efc_address_in: wp.array2d(dtype=int),
  contact_worldid_in: wp.array(dtype=int),
  efc_J_in: wp.array3d(dtype=float),
  efc_D_in: wp.array2d(dtype=float),
  efc_done_in: wp.array(dtype=bool),
  efc_u_in: wp.array(dtype=types.vec6),
  efc_uu_in: wp.array(dtype=float),
  # In:
  nblocks_perblock: int,
  dim_block: int,
  # Data out:
  efc_h_out: wp.array3d(dtype=float),
):
  conid_start, elementid = wp.tid()

  dof1id = dof_tri_row[elementid]
  dof2id = dof_tri_col[elementid]

  for i in range(nblocks_perblock):
    conid = conid_start + i * dim_block

    if conid >= min(ncon_in[0], nconmax_in):
      return

    worldid = contact_worldid_in[conid]
    if efc_done_in[worldid]:
      continue

    condim = contact_dim_in[conid]

    if condim == 1:
      continue

    fri = contact_friction_in[conid]
    mu = fri[0] / wp.sqrt(opt_impratio[worldid])
    n = efc_u_in[conid][0]
    tt = efc_uu_in[conid]
    if tt <= 0.0:
      t = 0.0
    else:
      t = wp.sqrt(tt)

    middle_zone = (t > 0) and (n < (mu * t)) and ((mu * n + t) > 0.0)

    if not middle_zone:
      continue

    t = wp.max(t, types.MJ_MINVAL)
    ttt = wp.max(t * t * t, types.MJ_MINVAL)

    mu2 = mu * mu
    efc0 = contact_efc_address_in[conid, 0]
    dm = efc_D_in[worldid, efc0] / wp.max(mu2 * (1.0 + mu2), types.MJ_MINVAL)

    if dm == 0.0:
      continue

    u = efc_u_in[conid]

    efc_h = float(0.0)

    for dim1id in range(condim):
      if dim1id == 0:
        efcid1 = efc0
      else:
        efcid1 = contact_efc_address_in[conid, dim1id]

      efc_J11 = efc_J_in[worldid, efcid1, dof1id]
      efc_J12 = efc_J_in[worldid, efcid1, dof2id]

      ui = u[dim1id]

      for dim2id in range(0, dim1id + 1):
        if dim2id == 0:
          efcid2 = efc0
        else:
          efcid2 = contact_efc_address_in[conid, dim2id]

        efc_J21 = efc_J_in[worldid, efcid2, dof1id]
        efc_J22 = efc_J_in[worldid, efcid2, dof2id]

        uj = u[dim2id]

        # set first row/column: (1, -mu/t * u)
        if dim1id == 0 and dim2id == 0:
          hcone = 1.0
        elif dim1id == 0:
          hcone = -mu / t * uj
        elif dim2id == 0:
          hcone = -mu / t * ui
        else:
          hcone = mu * n / ttt * ui * uj

          # add to diagonal: mu^2 - mu * n / t
          if dim1id == dim2id:
            hcone += mu2 - mu * n / t

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
          efc_h += hcone * efc_J11 * efc_J22

          if dim1id != dim2id:
            efc_h += hcone * efc_J12 * efc_J21

    worldid = contact_worldid_in[conid]
    efc_h_out[worldid, dof1id, dof2id] += efc_h


@cache_kernel
def update_gradient_cholesky(tile_size: int):
  @nested_kernel
  def kernel(
    # Data in:
    efc_grad_in: wp.array2d(dtype=float),
    efc_h_in: wp.array3d(dtype=float),
    efc_done_in: wp.array(dtype=bool),
    # Data out:
    efc_Mgrad_out: wp.array2d(dtype=float),
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if efc_done_in[worldid]:
      return

    mat_tile = wp.tile_load(efc_h_in[worldid], shape=(TILE_SIZE, TILE_SIZE))
    fact_tile = wp.tile_cholesky(mat_tile)
    input_tile = wp.tile_load(efc_grad_in[worldid], shape=TILE_SIZE)
    output_tile = wp.tile_cholesky_solve(fact_tile, input_tile)
    wp.tile_store(efc_Mgrad_out[worldid], output_tile)

  return kernel


@cache_kernel
def update_gradient_cholesky_blocked(tile_size: int):
  @nested_kernel
  def kernel(
    # Data in:
    efc_grad_in: wp.array3d(dtype=float),
    efc_h_in: wp.array3d(dtype=float),
    efc_done_in: wp.array(dtype=bool),
    matrix_size: int,
    cholesky_L_tmp: wp.array3d(dtype=float),
    cholesky_y_tmp: wp.array3d(dtype=float),
    # Data out:
    efc_Mgrad_out: wp.array3d(dtype=float),
  ):
    worldid, tid_block = wp.tid()
    TILE_SIZE = wp.static(tile_size)

    if efc_done_in[worldid]:
      return

    wp.static(create_blocked_cholesky_func(TILE_SIZE))(tid_block, efc_h_in[worldid], matrix_size, cholesky_L_tmp[worldid])
    wp.static(create_blocked_cholesky_solve_func(TILE_SIZE))(
      tid_block, cholesky_L_tmp[worldid], efc_grad_in[worldid], cholesky_y_tmp[worldid], matrix_size, efc_Mgrad_out[worldid]
    )

  return kernel


def _update_gradient(m: types.Model, d: types.Data):
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
      wp.launch(
        update_gradient_zero_h_lower,
        dim=(d.nworld, m.dof_tri_row.size),
        inputs=[m.dof_tri_row, m.dof_tri_col, d.efc.done],
        outputs=[d.efc.h],
      )
      wp.launch(
        update_gradient_set_h_qM_lower_sparse,
        dim=(d.nworld, m.qM_fullm_i.size),
        inputs=[m.qM_fullm_i, m.qM_fullm_j, d.qM, d.efc.done],
        outputs=[d.efc.h],
      )
    else:
      wp.launch(
        update_gradient_copy_lower_triangle,
        dim=(d.nworld, m.dof_tri_row.size),
        inputs=[m.dof_tri_row, m.dof_tri_col, d.qM, d.efc.done],
        outputs=[d.efc.h],
      )

    lower_triangle_dim = int(m.nv * (m.nv + 1) / 2)
    # TODO(team): Investigate whether d.efc.h initialization can be merged into this kernel
    wp.launch(
      update_gradient_JTDAJ,
      dim=(d.nworld, lower_triangle_dim),
      inputs=[
        d.nefc,
        d.efc.J,
        d.efc.D,
        d.efc.active,
        d.efc.done,
      ],
      outputs=[d.efc.h],
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
        dim_block = d.nconmax

      nblocks_perblock = int((d.nconmax + dim_block - 1) / dim_block)

      wp.launch(
        update_gradient_JTCJ,
        dim=(dim_block, m.dof_tri_row.size),
        inputs=[
          m.opt.impratio,
          m.dof_tri_row,
          m.dof_tri_col,
          d.nconmax,
          d.ncon,
          d.contact.friction,
          d.contact.dim,
          d.contact.efc_address,
          d.contact.worldid,
          d.efc.J,
          d.efc.D,
          d.efc.done,
          d.efc.u,
          d.efc.uu,
          nblocks_perblock,
          dim_block,
        ],
        outputs=[d.efc.h],
      )

    # TODO(team): Define good threshold for blocked vs non-blocked cholesky
    if m.nv < 32:
      wp.launch_tiled(
        update_gradient_cholesky(m.nv),
        dim=(d.nworld,),
        inputs=[d.efc.grad, d.efc.h, d.efc.done],
        outputs=[d.efc.Mgrad],
        block_dim=m.block_dim.update_gradient_cholesky,
      )
    else:
      wp.launch_tiled(
        update_gradient_cholesky_blocked(32),
        dim=(d.nworld,),
        inputs=[
          d.efc.grad.reshape(shape=(d.nworld, m.nv, 1)),
          d.efc.h,
          d.efc.done,
          m.nv,
          d.efc.cholesky_L_tmp,
          d.efc.cholesky_y_tmp.reshape(shape=(d.nworld, m.nv, 1)),
        ],
        outputs=[d.efc.Mgrad.reshape(shape=(d.nworld, m.nv, 1))],
        block_dim=m.block_dim.update_gradient_cholesky,
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

  if opt_solver == wp.static(types.SolverType.CG.value):
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
  nsolving_out: wp.array(dtype=int),
  efc_done_out: wp.array(dtype=bool),
):
  worldid = wp.tid()

  if efc_done_in[worldid]:
    return

  solver_niter_out[worldid] += 1
  tolerance = opt_tolerance[worldid]

  improvement = _rescale(nv, stat_meaninertia, efc_prev_cost_in[worldid] - efc_cost_in[worldid])
  gradient = _rescale(nv, stat_meaninertia, wp.math.sqrt(efc_grad_dot_in[worldid]))
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
):
  _linesearch(m, d)

  if m.opt.solver == types.SolverType.CG:
    wp.launch(
      solve_prev_grad_Mgrad,
      dim=(d.nworld, m.nv),
      inputs=[d.efc.grad, d.efc.Mgrad, d.efc.done],
      outputs=[d.efc.prev_grad, d.efc.prev_Mgrad],
    )

  _update_constraint(m, d)
  _update_gradient(m, d)

  # polak-ribiere
  if m.opt.solver == types.SolverType.CG:
    wp.launch(
      solve_beta,
      dim=(d.nworld,),
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
    dim=(d.nworld,),
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
    outputs=[d.solver_niter, d.nsolving, d.efc.done],
  )


def create_context(m: types.Model, d: types.Data, grad: bool = True):
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
  support.mul_m(m, d, d.efc.Ma, d.qacc, d.efc.done)

  _update_constraint(m, d)

  if grad:
    _update_gradient(m, d)


def _copy_acc(m: types.Model, d: types.Data):
  wp.copy(d.qacc, d.qacc_smooth)
  wp.copy(d.qacc_warmstart, d.qacc_smooth)
  d.solver_niter.fill_(0)


@event_scope
def solve(m: types.Model, d: types.Data):
  if d.njmax == 0:
    _copy_acc(m, d)
  else:
    _solve(m, d)


def _solve(m: types.Model, d: types.Data):
  """Finds forces that satisfy constraints."""
  if not (m.opt.disableflags & types.DisableBit.WARMSTART):
    wp.copy(d.qacc, d.qacc_warmstart)
  else:
    wp.copy(d.qacc, d.qacc_smooth)

  # create context
  create_context(m, d, grad=True)

  # search = -Mgrad
  wp.launch(
    solve_init_search,
    dim=(d.nworld, m.nv),
    inputs=[d.efc.Mgrad],
    outputs=[d.efc.search, d.efc.search_dot],
  )

  if m.opt.iterations != 0 and m.opt.graph_conditional:
    # Note: the iteration kernel (indicated by while_body) is repeatedly launched
    # as long as condition_iteration is not zero.
    # condition_iteration is a warp array of size 1 and type int, it counts the number
    # of worlds that are not converged, it becomes 0 when all worlds are converged.
    # When the number of iterations reaches m.opt.iterations, solver_niter
    # becomes zero and all worlds are marked as converged to avoid an infinite loop.
    # note: we only launch the iteration kernel if everything is not done
    d.nsolving.fill_(d.nworld)
    wp.capture_while(
      d.nsolving,
      while_body=_solver_iteration,
      m=m,
      d=d,
    )
  else:
    # This branch is mostly for when JAX is used as it is currently not compatible
    # with CUDA graph conditional.
    # It should be removed when JAX becomes compatible.
    for i in range(m.opt.iterations):
      _solver_iteration(m, d)

  wp.copy(d.qacc_warmstart, d.qacc)
