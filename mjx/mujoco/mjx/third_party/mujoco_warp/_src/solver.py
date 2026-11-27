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
from typing import Any, Optional

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import derivative
from mujoco.mjx.third_party.mujoco_warp._src import island
from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import passive
from mujoco.mjx.third_party.mujoco_warp._src import smooth
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import clz64
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_augmented_factorize_solve_newton_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_factorize_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_solve_newton_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_sparse_tiles_factorize_solve_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_sparse_tiles_factorize_solve_newton_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import create_blocked_cholesky_sparse_tiles_solve_newton_func
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import ctz64
from mujoco.mjx.third_party.mujoco_warp._src.block_cholesky import solve_search_sums
from mujoco.mjx.third_party.mujoco_warp._src.types import InverseContext
from mujoco.mjx.third_party.mujoco_warp._src.types import OverflowType
from mujoco.mjx.third_party.mujoco_warp._src.types import SolverContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import efc_threads_per_world
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import side_stream

wp.set_module_options({"enable_backward": False, "default_grid_stride": False})

_SPARSE_BLOCK_CHOLESKY_DIM = 96


def _has_passive(m: types.Model) -> bool:
  """Whether passive spring or damper forces are enabled."""
  return not ((m.opt.disableflags & types.DisableBit.SPRING) and (m.opt.disableflags & types.DisableBit.DAMPER))


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


def create_solver_context(
  m: types.Model,
  d: types.Data,
  inplace_cholesky: bool = False,
  dense_hessian: bool = True,
) -> SolverContext:
  """Create a SolverContext with allocated workspace arrays.

  Args:
    m: Model.
    d: Data.
    inplace_cholesky: If True, alias hfactor to h for non-incremental Newton
      solves where blocked Cholesky overwrites h in-place each iteration.
    dense_hessian: False for the compact solve of a sparse model (sparse Hessian kernels).

  Returns:
    SolverContext with allocated arrays.
  """
  nworld = d.nworld
  nv = m.nv
  nv_pad = m.nv_pad
  njmax = d.njmax

  alloc_h = m.opt.solver == types.SolverType.NEWTON
  alloc_hfactor = alloc_h and nv > types.H_ONE_TILE_MAX
  alloc_sparse_tiles = alloc_h and nv > _SPARSE_BLOCK_CHOLESKY_DIM and not m.M_tile_full
  alloc_mgrad = m.opt.solver == types.SolverType.CG
  alloc_incremental = _use_incremental(m)
  alloc_cone_rows = alloc_h and m.opt.cone == types.ConeType.ELLIPTIC and not m.is_sparse and dense_hessian
  alloc_solve_ld_scratch = (
    alloc_mgrad and d.qLD.shape[1] > m.qLD_block_total and bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
  )
  has_passive = _has_passive(m)
  alloc_tw_diag = (
    m.opt.integrator == types.IntegratorType.DISCRETE
    and alloc_mgrad
    and (m.nefmdof > 0 or (m.nflexrodedge > 0 and has_passive))
  )

  h = wp.empty((nworld, nv_pad, nv_pad), dtype=float) if alloc_h else wp.empty((nworld, 0, 0), dtype=float)
  if alloc_hfactor:
    # Non-incremental Newton rebuilds the upper triangle of h every iteration,
    # and blocked Cholesky loads each upper-triangular tile before storing U.
    if inplace_cholesky and not alloc_incremental:
      if not alloc_sparse_tiles and nv_pad <= nv and not (0 < d.nvmax < nv_pad):
        raise ValueError(f"inplace_cholesky requires an augmented padding column (nv={nv}, nv_pad={nv_pad})")
      hfactor = h
    else:
      hfactor = wp.empty((nworld, nv_pad, nv_pad), dtype=float)
    ntiles = nv_pad // types.TILE_SIZE_JTDAJ_DENSE
    nwords = (ntiles + 63) // 64
    # H tiles follow from M and the sparse J, except for the discrete integrator's metric terms;
    # compacted solves run on a dense model and also detect them from H
    structural = alloc_sparse_tiles and m.is_sparse and m.opt.integrator != types.IntegratorType.DISCRETE
    h_tile_nz = wp.empty((nworld, ntiles, ntiles), dtype=int) if alloc_sparse_tiles and not structural else None
    h_row_mask = wp.empty((nworld, ntiles, nwords), dtype=wp.uint64) if alloc_sparse_tiles else None
    h_col_mask = wp.empty((nworld, ntiles, nwords), dtype=wp.uint64) if alloc_sparse_tiles else None
    h_tile_is_diag = wp.empty((nworld, ntiles), dtype=int) if alloc_sparse_tiles else None
    h_rhs_work = wp.empty((nworld, nv_pad, 1), dtype=float) if alloc_sparse_tiles else None
  else:
    hfactor = wp.empty((nworld, 0, 0), dtype=float)
    h_tile_nz = None
    h_row_mask = None
    h_col_mask = None
    h_tile_is_diag = None
    h_rhs_work = None

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
    h=h,
    hfactor=hfactor,
    quad_changed_ids=wp.empty((nworld, njmax), dtype=int) if alloc_incremental else wp.empty((nworld, 0), dtype=int),
    quad_changed_count=wp.empty((nworld,), dtype=int) if alloc_incremental else wp.empty((0,), dtype=int),
    state_changed_count=wp.empty((nworld,), dtype=int) if alloc_incremental else wp.empty((0,), dtype=int),
    solve_ld_scratch=wp.empty((nworld, nv_pad), dtype=float) if alloc_solve_ld_scratch else None,
    tw_diag=wp.empty((nworld, nv), dtype=float) if alloc_tw_diag else None,
    h_tile_nz=h_tile_nz,
    h_row_mask=h_row_mask,
    h_col_mask=h_col_mask,
    h_tile_is_diag=h_tile_is_diag,
    h_rhs_work=h_rhs_work,
    inertia_scale=wp.empty((nworld,), dtype=float),
    cone_coef=wp.empty((nworld, njmax), dtype=types.vec6) if alloc_cone_rows else None,
    # tile-loaded in whole chunks, like efc.D
    cone_weight=wp.empty((nworld, d.njmax_pad), dtype=float) if alloc_cone_rows else None,
    cone_base=wp.empty((nworld, njmax), dtype=int) if alloc_cone_rows else None,
    nsolving=wp.empty((1,), dtype=int),
    qfrc_smooth_eff=wp.zeros((nworld, nv_pad), dtype=float) if m.opt.integrator == types.IntegratorType.DISCRETE else None,
    flex_scratch=(
      passive.flex_stretch_scratch(m, d)
      if (m.opt.integrator == types.IntegratorType.DISCRETE and m.nflex > 0 and m.nflexvert > 0)
      else None
    ),
  )


@wp.kernel
def _add_qfrc_smooth_eff(
  # Model:
  opt_enableflags: int,
  dof_treeid: wp.array[int],
  # Data in:
  tree_awake_in: wp.array2d[int],
  qfrc_smooth_in: wp.array2d[float],
  efm_c_in: wp.array2d[float],
  efm_ca_in: wp.array2d[float],
  # Out:
  qfrc_smooth_eff_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  if (opt_enableflags & types.EnableBit.SLEEP) and tree_awake_in[worldid, dof_treeid[dofid]] == 0:
    qfrc_smooth_eff_out[worldid, dofid] = 0.0
    return
  qfrc_smooth_eff_out[worldid, dofid] = qfrc_smooth_in[worldid, dofid] + efm_c_in[worldid, dofid] + efm_ca_in[worldid, dofid]


@wp.func
def _diag_adr(rowadr: int, rownnz: int) -> int:
  return rowadr + rownnz - 1


@cache_kernel
def _inertia_scale(enable_sleep: bool = False):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    M_rownnz: wp.array[int],
    M_rowadr: wp.array[int],
    # Data in:
    M_in: wp.array2d[float],
    ncdof_in: wp.array[int],
    cdof_dof_in: wp.array2d[int],
    # Out:
    inertia_scale_out: wp.array[float],
  ):
    """Inverse trace of the active inertia matrix, which scales the termination criteria."""
    worldid = wp.tid()
    trace = float(0.0)
    if wp.static(enable_sleep):
      ncdof = ncdof_in[worldid]
      for c in range(ncdof):
        dof = cdof_dof_in[worldid, c]
        trace += M_in[worldid, M_rowadr[dof] + M_rownnz[dof] - 1]
    else:
      for i in range(nv):
        trace += M_in[worldid, M_rowadr[i] + M_rownnz[i] - 1]
    inertia_scale_out[worldid] = 1.0 / wp.max(types.MJ_MINVAL, trace)

  return kernel


@wp.kernel(module="unique", enable_backward=False)
def _inertia_scale_tiled(
  # Model:
  nv: int,
  M_rownnz: wp.array[int],
  M_rowadr: wp.array[int],
  # Data in:
  M_in: wp.array2d[float],
  # Out:
  inertia_scale_out: wp.array[float],
):
  worldid, lane = wp.tid()
  trace = float(0.0)
  for start in range(0, nv, smooth.WORLD_SUM_CHUNK):
    rowadr = wp.tile_load(M_rowadr, shape=(smooth.WORLD_SUM_CHUNK,), offset=(start,))
    rownnz = wp.tile_load(M_rownnz, shape=(smooth.WORLD_SUM_CHUNK,), offset=(start,))
    diag = wp.tile_load_indexed(
      M_in[worldid], wp.tile_map(_diag_adr, rowadr, rownnz), shape=(smooth.WORLD_SUM_CHUNK,), storage="shared"
    )
    if lane == 0:
      for i in range(wp.min(smooth.WORLD_SUM_CHUNK, nv - start)):
        trace += diag[i]
  if lane == 0:
    inertia_scale_out[worldid] = 1.0 / wp.max(types.MJ_MINVAL, trace)


def launch_inertia_scale(m: types.Model, M: wp.array2d[float], inertia_scale: wp.array[float], d: types.Data | None = None):
  enable_sleep = bool(m.opt.enableflags & types.EnableBit.SLEEP) and d is not None
  if not enable_sleep and m.nv > smooth.WORLD_SUM_BLOCK:
    wp.launch_tiled(
      _inertia_scale_tiled,
      dim=M.shape[0],
      inputs=[m.nv, m.M_rownnz, m.M_rowadr, M],
      outputs=[inertia_scale],
      block_dim=smooth.WORLD_SUM_BLOCK,
    )
  else:
    ncdof = d.ncdof if d is not None else wp.empty(0, dtype=int, device=M.device)
    cdof_dof = d.cdof_dof if d is not None else wp.empty((0, 0), dtype=int, device=M.device)
    wp.launch(
      _inertia_scale(enable_sleep),
      dim=M.shape[0],
      inputs=[m.nv, m.M_rownnz, m.M_rowadr, M, ncdof, cdof_dof],
      outputs=[inertia_scale],
    )


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
def _eval_frictionloss_alpha_zero(jaref: float, jv: float, f: float, rf: float, d: float) -> wp.vec3:
  """Eval frictionloss at alpha=0 and return (cost, grad, hessian)."""
  if (-rf < jaref) and (jaref < rf):
    jvD = jv * d
    return wp.vec3(0.5 * d * jaref * jaref, jvD * jaref, jv * jvD)
  elif jaref <= -rf:
    return wp.vec3(f * (-0.5 * rf - jaref), -f * jv, 0.0)
  else:
    return wp.vec3(f * (-0.5 * rf + jaref), f * jv, 0.0)


@wp.func
def _eval_frictionloss_cost_diff(start: float, dx: float, f: float, rf: float, d: float) -> float:
  """Eval shifted frictionloss cost(start + dx) - cost(start) without float32 cancellation."""
  x = start + dx
  if (-rf < start) and (start < rf) and (-rf < x) and (x < rf):
    return 0.5 * d * dx * (x + start)
  if start <= -rf and x <= -rf:
    return -f * dx
  if start >= rf and x >= rf:
    return f * dx
  # Cross-zone transitions use the exact quadratic-to-linear boundary gap at +/-rf.
  if (-rf < start) and (start < rf):
    gap = wp.where(x >= rf, rf - start, rf + start)
    return wp.where(x >= rf, f * dx, -f * dx) - 0.5 * d * gap * gap
  if (-rf < x) and (x < rf):
    abs_x = wp.abs(x)
    if x * start >= 0.0:
      gap = rf - abs_x
      return wp.where(start >= rf, f * dx, -f * dx) + 0.5 * d * gap * gap
    return -(f * (wp.abs(start) - rf) + 0.5 * d * (rf - abs_x) * (rf + abs_x))
  return wp.where(x >= rf, f, -f) * (x + start)


@wp.func
def _eval_frictionloss_shifted_one(
  # In:
  start: float,
  dx: float,
  f: float,
  rf: float,
  d: float,
  jvD: float,
  hessian: float,
  f_jv: float,
) -> wp.vec3:
  """Eval shifted frictionloss with precomputed shared values."""
  x = start + dx
  cost = _eval_frictionloss_cost_diff(start, dx, f, rf, d)
  if (-rf < x) and (x < rf):
    return wp.vec3(cost, jvD * x, hessian)
  elif x <= -rf:
    return wp.vec3(cost, -f_jv, 0.0)
  else:
    return wp.vec3(cost, f_jv, 0.0)


@wp.func
def _eval_frictionloss_shifted(jaref: float, jv: float, f: float, rf: float, d: float, alpha: float) -> wp.vec3:
  """Eval frictionloss shifted by alpha=0."""
  jvD = jv * d
  return _eval_frictionloss_shifted_one(jaref, alpha * jv, f, rf, d, jvD, jv * jvD, f * jv)


@wp.func
def _eval_frictionloss_shifted_3alphas(
  # In:
  jaref: float,
  jv: float,
  f: float,
  rf: float,
  d: float,
  lo_alpha: float,
  hi_alpha: float,
  mid_alpha: float,
) -> tuple[wp.vec3, wp.vec3, wp.vec3]:
  """Eval shifted frictionloss for 3 alphas with shared precomputation."""
  jvD = jv * d
  hessian = jv * jvD
  f_jv = f * jv
  return (
    _eval_frictionloss_shifted_one(jaref, lo_alpha * jv, f, rf, d, jvD, hessian, f_jv),
    _eval_frictionloss_shifted_one(jaref, hi_alpha * jv, f, rf, d, jvD, hessian, f_jv),
    _eval_frictionloss_shifted_one(jaref, mid_alpha * jv, f, rf, d, jvD, hessian, f_jv),
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
  u_perp_sq = quad1[2]
  u_par = quad2[0]
  dm = quad2[2]
  uu = u_par * u_par + u_perp_sq

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
    u_perp_sq = quad1[2]
    u_par = quad2[0]
    v_norm = quad2[1]
    dm = quad2[2]
    T0_inv = 1.0 / T0
    T1 = u_par * v_norm * T0_inv
    T2 = (u_perp_sq / (T0 * T0)) * (v_norm * v_norm * T0_inv)
    r1 = quad1[1] - mu * T1
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
  u_perp_sq = quad1[2]
  u_par = quad2[0]
  v_norm = quad2[1]
  dm = quad2[2]

  N = u0 + alpha * v0
  w_par = u_par + alpha * v_norm
  Tsqr = w_par * w_par + u_perp_sq

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
      T1 = w_par * v_norm * T_inv
      T2 = (u_perp_sq / Tsqr) * (v_norm * v_norm * T_inv)
      r = N - mu * T
      r1 = v0 - mu * T1

      if state0 == int(types.ConstraintState.CONE):
        # Rationalize T - T0 before forming the small cone residual change.
        Tsqr_delta = alpha * v_norm * (u_par + w_par)
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
    rf = math.safe_div(f, efc_D)
    return _eval_frictionloss_shifted(ctx_Jaref, ctx_jv, f, rf, efc_D, alpha)

  # Equality constraint
  return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, 0.0)


@wp.func
def _elliptic_mu_scale(D0: float, D1: float) -> float:
  """Regularized cone scale sqrt(R1/R0) from the normal and first friction rows."""
  return wp.sqrt(math.safe_div(D0, D1))


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
    rf = math.safe_div(f, efc_D)
    return _eval_frictionloss_shifted(ctx_Jaref, ctx_jv, f, rf, efc_D, alpha)

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
    return _eval_frictionloss_alpha_zero(ctx_Jaref, ctx_jv, f, rf, efc_D)

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
    return _eval_frictionloss_alpha_zero(ctx_Jaref, ctx_jv, f, rf, efc_D)

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
    f = efc_frictionloss[efcid]
    rf = math.safe_div(f, efc_D)
    return _eval_frictionloss_shifted_3alphas(ctx_Jaref, ctx_jv, f, rf, efc_D, lo_alpha, hi_alpha, mid_alpha)

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
    efc_D = efc_D_in[efcid]
    f = efc_frictionloss[efcid]
    rf = math.safe_div(f, efc_D)
    return _eval_frictionloss_shifted_3alphas(ctx_Jaref, ctx_jv, f, rf, efc_D, lo_alpha, hi_alpha, mid_alpha)

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


@wp.func_native(snippet="WP_TILE_SYNC();")
def _syncthreads():
  pass


@wp.func
def _sparse_row_dot(
  # Data in:
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  # In:
  worldid: int,
  efcid: int,
  vec_in: wp.array2d[float],
) -> float:
  val = float(0.0)
  rownnz = efc_J_rownnz_in[worldid, efcid]
  rowadr = efc_J_rowadr_in[worldid, efcid]
  for k in range(rownnz):
    sparseid = rowadr + k
    colind = efc_J_colind_in[worldid, 0, sparseid]
    val += efc_J_in[worldid, 0, sparseid] * vec_in[worldid, colind]
  return val


@cache_kernel
def _linesearch_iterative_kernel(
  ls_iterations: int,
  cone_type: types.ConeType,
  fuse_jv: bool,
  is_sparse: bool,
  incremental: bool,
  warn_overflow: int,
  is_discrete: bool,
):
  """Factory for iterative linesearch kernel.

  Args:
    ls_iterations: Max linesearch iterations (compile-time constant for loop optimization).
    cone_type: Friction cone type (PYRAMIDAL or ELLIPTIC) for compile-time optimization.
    fuse_jv: Whether to compute jv = J @ search in-kernel (efficient for small nv).
    is_sparse: Use sparse matrix representation for constraint Jacobian.
    incremental: Use incremental linesearch updates.
    warn_overflow: Overflow warning bitmask.
    is_discrete: Discrete integrator (per-contact regularized cone scale).
  """
  LS_ITERATIONS = ls_iterations
  IS_ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  FUSE_JV = fuse_jv
  INCREMENTAL = incremental
  IS_SPARSE = is_sparse
  IS_DISCRETE = is_discrete

  # Select specialized helper functions based on cone type
  if IS_ELLIPTIC:
    _compute_efc_eval_pt = _compute_efc_eval_pt_elliptic
    _compute_efc_eval_pt_alpha_zero = _compute_efc_eval_pt_alpha_zero_elliptic
    _compute_efc_eval_pt_3alphas = _compute_efc_eval_pt_3alphas_elliptic
  else:
    _compute_efc_eval_pt = _compute_efc_eval_pt_pyramidal
    _compute_efc_eval_pt_alpha_zero = _compute_efc_eval_pt_alpha_zero_pyramidal
    _compute_efc_eval_pt_3alphas = _compute_efc_eval_pt_3alphas_pyramidal

  # fast math only approximates div/sqrt here; keep transcendentals out of this kernel
  @wp.kernel(module="unique", enable_backward=False, grid_stride=True, module_options={"fast_math": True})
  def kernel(
    # Model:
    nv: int,
    opt_tolerance: wp.array[float],
    opt_ls_tolerance: wp.array[float],
    opt_impratio_invsqrt: wp.array[float],
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
    ctx_inertia_scale_in: wp.array[float],
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
          if wp.static(IS_SPARSE):
            jv = _sparse_row_dot(efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, ctx_search_in)
          else:
            jv = float(0.0)
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
              mu_scale = impratio_invsqrt
              if wp.static(IS_DISCRETE):
                mu_scale = _elliptic_mu_scale(efc_D, efc_D_in[worldid, contact_efc_address_in[conid, 1]])
              mu = friction[0] * mu_scale

              u0 = Jaref * mu
              v0 = jv * mu

              u_tan = types.vec5(0.0)
              v_tan = types.vec5(0.0)
              uu = float(0.0)
              uv = float(0.0)
              vv = float(0.0)
              det = float(0.0)
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

                  for k in range(j - 1):
                    djk = uj * v_tan[k] - u_tan[k] * vj
                    det += djk * djk
                  u_tan[j - 1] = uj
                  v_tan[j - 1] = vj

                  uu += uj * uj
                  uv += uj * vj
                  vv += vj * vj

              v_norm = float(0.0)
              u_par = float(0.0)
              u_perp_sq = uu
              if vv > 0.0:
                v_norm = wp.sqrt(vv)
                u_par = uv / v_norm
                u_perp_sq = det / vv

              ctx_quad_out[worldid, efcid] = quad

              efcid1 = contact_efc_address_in[conid, 1]
              ctx_quad_out[worldid, efcid1] = wp.vec3(u0, v0, u_perp_sq)

              mu2 = mu * mu
              efcid2 = contact_efc_address_in[conid, 2]
              ctx_quad_out[worldid, efcid2] = wp.vec3(u_par, v_norm, efc_D / (mu2 * (1.0 + mu2)))

      _syncthreads()  # ensure all quads are written before reading

    # gtol (tolerance values loaded here, deferred from kernel start)
    tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
    ls_tolerance = opt_ls_tolerance[worldid % opt_ls_tolerance.shape[0]]
    snorm = wp.sqrt(ctx_search_dot_in[worldid])
    gtol = tolerance * ls_tolerance * snorm / ctx_inertia_scale_in[worldid]

    # quad_gauss = [0, search.T @ Ma - search.T @ qfrc_smooth, 0.5 * search.T @ mv] + equality
    local_gauss = wp.vec2(0.0)
    for dofid in range(tid, nv, wp.block_dim()):
      search = ctx_search_in[worldid, dofid]
      local_gauss += wp.vec2(
        search * (efc_Ma_out[worldid, dofid] - qfrc_smooth_in[worldid, dofid]),
        0.5 * search * ctx_mv_in[worldid, dofid],
      )

    for efcid in range(tid, ne, wp.block_dim()):
      jv = ctx_jv_in[worldid, efcid]
      jvD = jv * efc_D_in[worldid, efcid]
      local_gauss += wp.vec2(
        jvD * ctx_Jaref_in[worldid, efcid],
        0.5 * jv * jvD,
      )

    # p0 via parallel reduction over non-equality constraints
    local_p0 = wp.vec3(0.0)
    for efcid in range(ne + tid, nefc, wp.block_dim()):
      if wp.static(IS_ELLIPTIC):
        efc_type = efc_type_in[worldid, efcid]
        efc_id = 0
        contact_friction = types.vec5(0.0)
        efc_addr0 = int(0)
        mu_scale = impratio_invsqrt
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
          if wp.static(IS_DISCRETE):
            if efcid == efc_addr0:
              mu_scale = _elliptic_mu_scale(efc_D_in[worldid, efc_addr0], efc_D_in[worldid, efc_addr1])

        local_p0 += _compute_efc_eval_pt_alpha_zero(
          efcid,
          ne,
          nf,
          mu_scale,
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

    # Pack independent sums to share reduction synchronization.
    initial = types.vec5(local_gauss[0], local_gauss[1], local_p0[0], local_p0[1], local_p0[2])
    initial_sum = wp.tile_reduce(wp.add, wp.tile(initial, preserve_type=True))[0]
    ctx_quad_gauss = wp.vec3(0.0, initial_sum[0], initial_sum[1])
    rows = wp.vec3(initial_sum[2], initial_sum[3], initial_sum[4])

    # add quad_gauss contribution to p0
    p0 = wp.vec3(ctx_quad_gauss[0], ctx_quad_gauss[1], 2.0 * ctx_quad_gauss[2]) + rows
    p0_delta = wp.vec3(0.0, p0[1], p0[2])

    # The bracketing search reads derivative sums whose rounding noise is ~eps
    # times their gross magnitude, so roots below eps * |q1| / p0[2] (and never
    # below 8 ulps of the unit ray anchor) are noise, not descent: an accepted
    # step under this floor means the stale ray is exhausted and the fast path
    # must rebuild the world. Per quadratic row |q1| = sqrt(2 * cost * hessian),
    # so Cauchy-Schwarz bounds the row total by sums already reduced for p0; the
    # smooth term is added exactly. Friction linear rows fall outside the bound,
    # which only lowers the floor toward the fixed 8-ulp base.
    q1_abs = wp.sqrt(2.0 * wp.max(rows[0], 0.0) * wp.max(rows[2], 0.0)) + wp.abs(ctx_quad_gauss[1])

    # set acceptance tolerance to avoid exceeding gtol in f32
    gtol_accept = wp.max(gtol, _ALPHA_NOISE_EPS * q1_abs)

    noise_floor = float(0.0)
    if wp.static(INCREMENTAL):
      noise_floor = _ALPHA_NOISE_EPS * wp.max(1.0, math.safe_div(q1_abs, p0[2]))

    alpha = float(0.0)
    improvement = float(0.0)

    # a slope at alpha = 0 below rounding noise means the start is already the line minimum
    ls_converged = wp.abs(p0[1]) < _ALPHA_NOISE_EPS * q1_abs

    # lo_in at lo_alpha_in = -p0[1] / p0[2]
    lo_alpha_in = -math.safe_div(p0[1], p0[2])
    lo_in = wp.vec3(0.0)

    if not ls_converged:
      local_lo_in = wp.vec3(0.0)
      for efcid in range(ne + tid, nefc, wp.block_dim()):
        if wp.static(IS_ELLIPTIC):
          efc_type = efc_type_in[worldid, efcid]
          efc_id = 0
          contact_friction = types.vec5(0.0)
          efc_addr0 = int(0)
          mu_scale = impratio_invsqrt
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
            if wp.static(IS_DISCRETE):
              if efcid == efc_addr0:
                mu_scale = _elliptic_mu_scale(efc_D_in[worldid, efc_addr0], efc_D_in[worldid, efc_addr1])

          local_lo_in += _compute_efc_eval_pt(
            efcid,
            lo_alpha_in,
            ne,
            nf,
            mu_scale,
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
      if wp.abs(lo_in[1]) < gtol_accept and lo_in[0] < 0.0:
        ls_converged = True
        alpha = lo_alpha_in
        improvement = -lo_in[0]

    # main iterative loop - skip if already converged
    if not ls_converged:
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

        for efcid in range(ne + tid, nefc, wp.block_dim()):
          if wp.static(IS_ELLIPTIC):
            efc_type = efc_type_in[worldid, efcid]
            efc_id = 0
            contact_friction = types.vec5(0.0)
            efc_addr0 = int(0)
            mu_scale = impratio_invsqrt
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
              if wp.static(IS_DISCRETE):
                if efcid == efc_addr0:
                  mu_scale = _elliptic_mu_scale(efc_D_in[worldid, efc_addr0], efc_D_in[worldid, efc_addr1])

            r_lo, r_hi, r_mid = _compute_efc_eval_pt_3alphas(
              efcid,
              lo_next_alpha,
              hi_next_alpha,
              mid_alpha,
              ne,
              nf,
              mu_scale,
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

        # accept the lowest-cost converged candidate regardless of the sign of its derivative
        conv_lo = wp.abs(lo_next[1]) < gtol_accept and lo_next[0] < 0.0
        conv_hi = wp.abs(hi_next[1]) < gtol_accept and hi_next[0] < 0.0
        conv_mid = wp.abs(mid[1]) < gtol_accept and mid[0] < 0.0
        converged = conv_lo or conv_hi or conv_mid
        swap_lo = False
        swap_hi = False
        if converged:
          conv_pt = wp.vec3(types.MJ_MAXVAL, 0.0, 0.0)
          conv_alpha = float(0.0)
          if conv_lo and lo_next[0] < conv_pt[0]:
            conv_pt = lo_next
            conv_alpha = lo_next_alpha
          if conv_hi and hi_next[0] < conv_pt[0]:
            conv_pt = hi_next
            conv_alpha = hi_next_alpha
          if conv_mid and mid[0] < conv_pt[0]:
            conv_pt = mid
            conv_alpha = mid_alpha
          lo = conv_pt
          lo_alpha = conv_alpha
          hi = conv_pt
          hi_alpha = conv_alpha
        else:
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
          # also accept a Newton step that crosses the minimizer from an undershooting hi,
          # turning the one-sided search into an opposite-sign bracket
          swap_hi_hi_next = _in_bracket(hi, hi_next) or (hi[1] < 0.0 and hi_next[1] > 0.0)
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
        ls_done = converged or (not swap_lo and not swap_hi)

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
        if wp.static(bool(warn_overflow & OverflowType.LS_ITERATIONS)):
          wp.printf(
            "linesearch iterations limit reached - please increase ls_iterations beyond %u\n"
            "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.LS_ITERATIONS (or = 0 for all)\n",
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
      int(m.opt.warn_overflow),
      m.opt.integrator == types.IntegratorType.DISCRETE,
    ),
    dim=d.nworld,
    inputs=[
      m.nv,
      m.opt.tolerance,
      m.opt.ls_tolerance,
      m.opt.impratio_invsqrt,
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
      ctx.inertia_scale,
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
def _mul_jac_vec_kernel(
  is_sparse: bool, nv: int, dofs_per_thread: int, compact: bool, subtract_aref: bool, deterministic: bool = False
):
  COMPACT = compact
  SUBTRACT_AREF = subtract_aref
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN

  @wp.kernel(module="unique", module_options=module_options, grid_stride=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_aref_in: wp.array2d[float],
    dof_cdof_in: wp.array2d[int],
    njmax_in: int,
    # In:
    vec_in: wp.array2d[float],
    skip_in: wp.array[bool],
    efc_stride: int,
    # Out:
    out: wp.array2d[float],
  ):
    worldid, efcid, dofstart = wp.tid()
    if skip_in[worldid]:
      return
    nefc = wp.min(nefc_in[worldid], njmax_in)

    if wp.static(is_sparse):
      for efcid in range(efcid, nefc, efc_stride):
        if wp.static(COMPACT):
          val = float(0.0)
          rownnz = efc_J_rownnz_in[worldid, efcid]
          rowadr = efc_J_rowadr_in[worldid, efcid]
          for k in range(rownnz):
            sparseid = rowadr + k
            colind = dof_cdof_in[worldid, efc_J_colind_in[worldid, 0, sparseid]]
            if colind >= 0:
              val += efc_J_in[worldid, 0, sparseid] * vec_in[worldid, colind]
        else:
          val = _sparse_row_dot(efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, vec_in)
        if wp.static(SUBTRACT_AREF):
          val -= efc_aref_in[worldid, efcid]
        out[worldid, efcid] = val
      return

    if efcid >= nefc:
      return

    val = float(0.0)
    if wp.static(dofs_per_thread >= nv):
      for i in range(wp.static(min(dofs_per_thread, nv))):
        val += efc_J_in[worldid, efcid, i] * vec_in[worldid, i]
      if wp.static(SUBTRACT_AREF):
        val -= efc_aref_in[worldid, efcid]
      out[worldid, efcid] = val
    else:
      for i in range(wp.static(dofs_per_thread)):
        ii = dofstart * wp.static(dofs_per_thread) + i
        if ii < nv:
          val += efc_J_in[worldid, efcid, ii] * vec_in[worldid, ii]
      if wp.static(SUBTRACT_AREF):
        if dofstart == 0:
          val -= efc_aref_in[worldid, efcid]
      wp.atomic_add(out, worldid, efcid, val)

  return kernel


def _mul_jac_vec(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext | InverseContext,
  vec: wp.array2d[float],
  out: wp.array2d[float],
  skip: wp.array[bool],
  subtract_aref: bool = False,
):
  sc = _sparse_compact(ctx)
  sparse = sc or m.is_sparse
  if sparse:
    dofs_per_thread = m.nv
    threads_per_efc = 1
  else:
    dofs_per_thread = 20 if m.nv > 50 else 50
    threads_per_efc = ceil(m.nv / dofs_per_thread)

  if threads_per_efc > 1:
    wp.launch(_linesearch_zero_jv, dim=(d.nworld, d.njmax), inputs=[d.nefc, skip], outputs=[out])

  dj = ctx.compact_d_full if sc else d
  det_atomics = bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
  kernel = _mul_jac_vec_kernel(sparse, m.nv, dofs_per_thread, sc, subtract_aref, det_atomics)
  efc_stride = efc_threads_per_world(d.nworld, d.njmax, d.qacc.device, kernel) if sparse else d.njmax
  wp.launch(
    kernel,
    dim=(d.nworld, efc_stride, threads_per_efc),
    inputs=[
      d.nefc,
      dj.efc.J_rownnz,
      dj.efc.J_rowadr,
      dj.efc.J_colind,
      dj.efc.J,
      d.efc.aref,
      dj.dof_cdof,
      d.njmax,
      vec,
      skip,
      efc_stride,
    ],
    outputs=[out],
  )


@event_scope
def _linesearch(m: types.Model, d: types.Data, ctx: SolverContext):
  """Linesearch for constraint solver.

  When state changes are tracked, worlds with ctx.search_unchanged reuse last
  iteration's mv/jv, so a fresh search requires clearing the flag (see
  _update_gradient_zero_grad_dot).

  Args:
    m: Model
    d: Data
    ctx: SolverContext
  """
  # mv and jv are pure functions of the search direction, and M and J are
  # constant within a solve, so worlds whose search was kept reuse last
  # iteration's values.
  skip = ctx.search_unchanged if _use_incremental(m) else ctx.done

  # Fuse small dense models, or sparse models with enough CUDA worlds.
  # Sparse compact solves retain the full model's sparse J traversal.
  sc = _sparse_compact(ctx)
  fuse_jv = m.nv <= 50 and not sc and (not m.is_sparse or (d.qacc.device.is_cuda and d.nworld >= 4 * d.qacc.device.sm_count))

  # jv = J @ search (when not fused into iterative kernel), independent of mv below
  jv_stream = None if fuse_jv else side_stream(d.qacc.device)
  if not fuse_jv:
    with wp.ScopedStream(jv_stream):
      _mul_jac_vec(m, d, ctx, ctx.search, ctx.jv, skip)

  # mv = M @ search (common to both parallel and iterative)
  _mul_m_compact_aware(m, d, ctx, ctx.mv, ctx.search, skip)
  if jv_stream is not None:
    wp.wait_stream(jv_stream)

  _linesearch_iterative(m, d, ctx, fuse_jv)


@cache_kernel
def _solve_init_dof(warmstart: bool, sparse: bool, run_qacc_smooth: bool):
  WARMSTART = warmstart
  SPARSE = sparse
  RUN_QACC_SMOOTH = run_qacc_smooth

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

    if wp.static(not RUN_QACC_SMOOTH):
      # without constraint rows, Newton reaches qacc_smooth in one step from the warmstart
      qacc_out[worldid, dofid] = qacc_warmstart_in[worldid, dofid]
    elif wp.static(WARMSTART):
      if nefc_in[worldid] > 0:
        qacc_out[worldid, dofid] = qacc_warmstart_in[worldid, dofid]
      else:
        qacc_out[worldid, dofid] = qacc_smooth_in[worldid, dofid]
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


@wp.kernel
def _solve_skip_empty_efc(
  # Data in:
  nefc_in: wp.array[int],
  # Out:
  ctx_done_out: wp.array[bool],
  nsolving_out: wp.array[int],
):
  worldid = wp.tid()
  if not ctx_done_out[worldid] and nefc_in[worldid] == 0:
    ctx_done_out[worldid] = True
    wp.atomic_sub(nsolving_out, 0, 1)  # kernel_analyzer: ignore[atomic]


@wp.kernel(module="unique")
def _solve_init_search_cg_tiled(
  # Model:
  nv: int,
  opt_tolerance: wp.array[float],
  opt_integrator: int,
  # In:
  ctx_inertia_scale_in: wp.array[float],
  ctx_grad_in: wp.array2d[float],
  ctx_grad_dot_in: wp.array[float],
  ctx_Mgrad_in: wp.array2d[float],
  # Out:
  ctx_search_out: wp.array2d[float],
  ctx_search_dot_out: wp.array[float],
  ctx_prev_grad_out: wp.array2d[float],
  ctx_prev_Mgrad_out: wp.array2d[float],
  ctx_done_out: wp.array[bool],
  nsolving_out: wp.array[int],
):
  worldid, tid = wp.tid()

  if ctx_done_out[worldid]:
    return

  local_search_dot = float(0.0)
  local_grad_mgrad = float(0.0)
  BLOCK_DIM = wp.block_dim()

  for dofid in range(tid, nv, BLOCK_DIM):
    grad = ctx_grad_in[worldid, dofid]
    mgrad = ctx_Mgrad_in[worldid, dofid]
    search = -1.0 * mgrad
    ctx_search_out[worldid, dofid] = search
    local_search_dot += search * search
    local_grad_mgrad += grad * mgrad

    ctx_prev_grad_out[worldid, dofid] = grad
    ctx_prev_Mgrad_out[worldid, dofid] = mgrad

  search_dot_tile = wp.tile(local_search_dot, preserve_type=True)
  search_dot_sum = wp.tile_reduce(wp.add, search_dot_tile)
  grad_mgrad_tile = wp.tile(local_grad_mgrad, preserve_type=True)
  grad_mgrad_sum = wp.tile_reduce(wp.add, grad_mgrad_tile)

  if tid == 0:
    ctx_search_dot_out[worldid] = search_dot_sum[0]
    if not ctx_done_out[worldid]:
      tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
      scale = ctx_inertia_scale_in[worldid]
      flg_gap = wp.max(0.0, 0.5 * scale * grad_mgrad_sum[0]) < tolerance
      if flg_gap and (
        opt_integrator != int(types.IntegratorType.DISCRETE.value) or scale * wp.sqrt(ctx_grad_dot_in[worldid]) < tolerance
      ):
        ctx_done_out[worldid] = True
        wp.atomic_sub(nsolving_out, 0, 1)  # kernel_analyzer: ignore[atomic]


@cache_kernel
def _update_constraint_efc(
  track_changes: bool,
  cone_rows: bool = False,
  is_discrete: bool = False,
  deterministic: bool = False,
):
  TRACK_CHANGES = track_changes
  CONE_ROWS = cone_rows
  IS_DISCRETE = is_discrete
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    # a deterministic launch gives each thread one row (see _update_efc_force): one atomic per row
    # for the quadratic change list, one for the state change count
    module_options["deterministic_max_records"] = 2

  @wp.kernel(module="unique", module_options=module_options, grid_stride=False)
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
    njmax_in: int,
    nacon_in: wp.array[int],
    # In:
    efc_stride: int,
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
    ctx_cone_coef_out: wp.array2d[types.vec6],
    ctx_cone_weight_out: wp.array2d[float],
    ctx_cone_base_out: wp.array2d[int],
  ):
    worldid, tid = wp.tid()

    if ctx_done_in[worldid]:
      return

    ne = ne_in[worldid]
    nf = nf_in[worldid]
    changed = int(0)
    if wp.static(TRACK_CHANGES):
      if tid == 0 and ctx_ls_exhausted_in[worldid]:
        changed = 1

    for efcid in range(tid, wp.min(nefc_in[worldid], njmax_in), efc_stride):
      # Read old state before overwriting
      if wp.static(TRACK_CHANGES):
        old_state = efc_state_out[worldid, efcid]

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
      if wp.static(CONE_ROWS):
        dim = int(0)
        friction = types.vec5()
        u = types.vec6()

      # rows of a contact truncated by an overflow are dropped, else they keep a stale force/state
      if is_elliptic:
        conid = efc_id_in[worldid, efcid]
        if conid >= wp.min(nacon_in[0], contact_dim_in.shape[0]):
          efc_force_out[worldid, efcid] = 0.0
          efc_state_out[worldid, efcid] = types.ConstraintState.SATISFIED.value
          continue
        efcid0 = contact_efc_address_in[conid, 0]
        if efcid0 < 0:
          efc_force_out[worldid, efcid] = 0.0
          efc_state_out[worldid, efcid] = types.ConstraintState.SATISFIED.value
          continue

        dim = contact_dim_in[conid]
        friction = contact_friction_in[conid]
        jaref0 = ctx_Jaref_in[worldid, efcid0]
        D0 = efc_D_in[worldid, efcid0]
        mu_scale = float(0.0)
        if wp.static(IS_DISCRETE):
          efcid1 = contact_efc_address_in[conid, 1]
          if efcid1 < 0:
            efc_force_out[worldid, efcid] = 0.0
            efc_state_out[worldid, efcid] = types.ConstraintState.SATISFIED.value
            continue
          mu_scale = _elliptic_mu_scale(D0, efc_D_in[worldid, efcid1])
        else:
          mu_scale = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
        mu = friction[0] * mu_scale

        valid = bool(True)
        for j in range(1, dim):
          efcidj = contact_efc_address_in[conid, j]
          if efcidj < 0:
            valid = False
            break
          else:
            frictionj = friction[j - 1]
            uj = ctx_Jaref_in[worldid, efcidj] * frictionj
            if wp.static(CONE_ROWS):
              u[j] = uj
            TT += uj * uj
            if efcid == efcidj:
              ufrictionj = uj * frictionj
        if not valid:
          efc_force_out[worldid, efcid] = 0.0
          efc_state_out[worldid, efcid] = types.ConstraintState.SATISFIED.value
          continue

      res = _eval_constraint(
        is_equality,
        is_friction,
        is_elliptic,
        ctx_Jaref_in[worldid, efcid],
        efc_D_in[worldid, efcid],
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

      if wp.static(CONE_ROWS):
        if new_state == types.ConstraintState.CONE.value:
          coef, weight = _cone_virtual_row(efcid - efcid0, dim, mu, D0, jaref0, u, TT, friction)
          ctx_cone_coef_out[worldid, efcid] = coef
          ctx_cone_weight_out[worldid, efcid] = weight
          ctx_cone_base_out[worldid, efcid] = efcid0

      if wp.static(TRACK_CHANGES):
        if (old_state == types.ConstraintState.QUADRATIC.value) != (new_state == types.ConstraintState.QUADRATIC.value):
          quad_changed_ids_out[worldid, wp.atomic_add(quad_changed_count_out, worldid, 1)] = efcid
        # LINEARNEG <-> LINEARPOS friction transitions change the force without
        # changing the quadratic flag (or H); the fast path must still see them.
        if old_state != new_state:
          changed += 1

    if wp.static(TRACK_CHANGES):
      if changed:
        wp.atomic_add(state_changed_count_out, worldid, changed)

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
def _update_constraint_init_qfrc_constraint_sparse(compact: bool, nv: int, deterministic: bool = False):
  COMPACT = compact
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = nv

  @wp.kernel(module="unique", module_options=module_options, grid_stride=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_force_in: wp.array2d[float],
    dof_cdof_in: wp.array2d[int],
    njmax_in: int,
    # In:
    efc_stride: int,
    state_changed_count_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Data out:
    qfrc_constraint_out: wp.array2d[float],
  ):
    worldid, tid = wp.tid()

    if ctx_done_in[worldid]:
      return

    if state_changed_count_in[worldid] == 0:
      return

    for efcid in range(tid, wp.min(nefc_in[worldid], njmax_in), efc_stride):
      force = efc_force_in[worldid, efcid]
      if force == 0.0:
        continue

      rownnz = efc_J_rownnz_in[worldid, efcid]
      rowadr = efc_J_rowadr_in[worldid, efcid]
      for i in range(rownnz):
        sparseid = rowadr + i
        colind = efc_J_colind_in[worldid, 0, sparseid]
        if wp.static(COMPACT):
          colind = dof_cdof_in[worldid, colind]
          if colind < 0:
            continue
        wp.atomic_add(qfrc_constraint_out[worldid], colind, efc_J_in[worldid, 0, sparseid] * force)

  return kernel


@cache_kernel
def _update_constraint_qfrc_gradient_sparse_tiled(nv: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    ne_in: wp.array[int],
    nf_in: wp.array[int],
    nefc_in: wp.array[int],
    qfrc_smooth_in: wp.array2d[float],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_frictionloss_in: wp.array2d[float],
    efc_Ma_in: wp.array2d[float],
    njmax_in: int,
    # In:
    ctx_Jaref_in: wp.array2d[float],
    ctx_ls_exhausted_in: wp.array[bool],
    ctx_alpha_in: wp.array[float],
    ctx_done_in: wp.array[bool],
    # Data out:
    qfrc_constraint_out: wp.array2d[float],
    efc_force_out: wp.array2d[float],
    efc_state_out: wp.array2d[int],
    # Out:
    quad_changed_ids_out: wp.array2d[int],
    quad_changed_count_out: wp.array[int],
    state_changed_count_out: wp.array[int],
    ctx_grad_out: wp.array2d[float],
    ctx_grad_dot_out: wp.array[float],
    ctx_newton_decrement_out: wp.array[float],
    ctx_grad_scale_out: wp.array[float],
    ctx_search_unchanged_out: wp.array[bool],
  ):
    worldid, tid = wp.tid()
    if ctx_done_in[worldid]:
      if tid == 0:
        quad_changed_count_out[worldid] = 0
        state_changed_count_out[worldid] = 0
        ctx_search_unchanged_out[worldid] = True
      return

    if tid == 0:
      quad_changed_count_out[worldid] = 0
    # Counter initialization must precede row atomics in every warp of the block.
    _syncthreads()
    ne = ne_in[worldid]
    nf = nf_in[worldid]
    nefc = wp.min(nefc_in[worldid], njmax_in)
    changed = int(0)
    if tid == 0 and ctx_ls_exhausted_in[worldid]:
      changed = 1
    for efcid in range(tid, nefc, wp.block_dim()):
      old_state = efc_state_out[worldid, efcid]
      is_equality = efcid < ne
      is_friction = not is_equality and efcid < ne + nf
      frictionloss = efc_frictionloss_in[worldid, efcid] if is_friction else 0.0
      Jaref = ctx_Jaref_in[worldid, efcid]
      D = efc_D_in[worldid, efcid]
      res = _eval_constraint(is_equality, is_friction, False, Jaref, D, frictionloss, efcid, -1, 0.0, 0.0, 0.0, 0.0, 0.0)
      new_state = int(res[1])
      efc_force_out[worldid, efcid] = res[0]
      efc_state_out[worldid, efcid] = new_state
      if (old_state == types.ConstraintState.QUADRATIC.value) != (new_state == types.ConstraintState.QUADRATIC.value):
        slot = wp.atomic_add(quad_changed_count_out, worldid, 1)  # kernel_analyzer: ignore[atomic]
        quad_changed_ids_out[worldid, slot] = efcid
      if old_state != new_state:
        changed += 1

    changed_sum = wp.tile_reduce(wp.add, wp.tile(changed, preserve_type=True))[0]
    if tid == 0:
      state_changed_count_out[worldid] = changed_sum
      ctx_search_unchanged_out[worldid] = changed_sum == 0

    # Stable worlds retain their force/gradient buffers and stay on the existing ray.
    if changed_sum == 0:
      if tid == 0:
        sigma = ctx_grad_scale_out[worldid]
        new_sigma = sigma - ctx_alpha_in[worldid]
        ratio = float(0.0)
        if sigma != 0.0:
          ratio = new_sigma / sigma
        ctx_grad_dot_out[worldid] *= ratio * ratio
        ctx_newton_decrement_out[worldid] *= ratio * ratio
        ctx_grad_scale_out[worldid] = new_sigma
      return

    for dofid in range(tid, wp.static(nv), wp.block_dim()):
      qfrc_constraint_out[worldid, dofid] = 0.0
    # All DOFs must be zero before any warp accumulates J^T f.
    _syncthreads()
    for efcid in range(tid, nefc, wp.block_dim()):
      force = efc_force_out[worldid, efcid]
      if force == 0.0:
        continue
      rownnz = efc_J_rownnz_in[worldid, efcid]
      rowadr = efc_J_rowadr_in[worldid, efcid]
      for i in range(rownnz):
        sparseid = rowadr + i
        colind = efc_J_colind_in[worldid, 0, sparseid]
        wp.atomic_add(
          qfrc_constraint_out[worldid], colind, efc_J_in[worldid, 0, sparseid] * force
        )  # kernel_analyzer: ignore[atomic]
    _syncthreads()

    local_grad_dot = float(0.0)
    for dofid in range(tid, wp.static(nv), wp.block_dim()):
      grad = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - qfrc_constraint_out[worldid, dofid]
      ctx_grad_out[worldid, dofid] = grad
      local_grad_dot += grad * grad
    grad_dot_sum = wp.tile_reduce(wp.add, wp.tile(local_grad_dot, preserve_type=True))
    if tid == 0:
      ctx_grad_dot_out[worldid] = grad_dot_sum[0]
      ctx_newton_decrement_out[worldid] = 0.0
      ctx_grad_scale_out[worldid] = 1.0

  return kernel


@wp.kernel
def _qfrc_constraint_from_grad(
  # Data in:
  nefc_in: wp.array[int],
  qfrc_smooth_in: wp.array2d[float],
  efc_Ma_in: wp.array2d[float],
  # In:
  ctx_grad_in: wp.array2d[float],
  ctx_grad_scale_in: wp.array[float],
  # Data out:
  qfrc_constraint_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()

  if nefc_in[worldid] == 0:
    qfrc_constraint_out[worldid, dofid] = 0.0
    return

  grad = ctx_grad_scale_in[worldid] * ctx_grad_in[worldid, dofid]
  qfrc_constraint_out[worldid, dofid] = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - grad


@cache_kernel
def _update_constraint_init_qfrc_constraint_dense(stable_fast: bool, write_grad: bool = False):
  STABLE_FAST = stable_fast
  WRITE_GRAD = write_grad

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    qfrc_smooth_in: wp.array2d[float],
    efc_J_in: wp.array3d[float],
    efc_force_in: wp.array2d[float],
    efc_Ma_in: wp.array2d[float],
    njmax_in: int,
    # In:
    state_changed_count_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Data out:
    qfrc_constraint_out: wp.array2d[float],
    # Out:
    ctx_grad_out: wp.array2d[float],
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

    if wp.static(WRITE_GRAD):
      ctx_grad_out[worldid, dofid] = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - sum_qfrc

  return kernel


@wp.kernel
def _update_gradient_h_incremental(
  # Model:
  nv: int,
  # Data in:
  efc_J_in: wp.array3d[float],
  efc_D_in: wp.array2d[float],
  efc_state_in: wp.array2d[int],
  # In:
  quad_changed_ids_in: wp.array2d[int],
  quad_changed_count_in: wp.array[int],
  threads_per_world: int,
  # Out:
  ctx_h_out: wp.array3d[float],
):
  """Incrementally update upper triangle of H for changed constraints.

  Each world's threads stride over its unique (i, j) elements, so a world without
  changes costs a few warps instead of one thread per element. For each changed
  constraint, adds or subtracts D * J[i] * J[j].
  """
  worldid, threadid = wp.tid()

  n_changes = quad_changed_count_in[worldid]
  if n_changes == 0:
    return

  for elementid in range(threadid, nv * (nv + 1) // 2, threads_per_world):
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
def _update_gradient_h_incremental_sparse(compact: bool, max_records: int = 0, deterministic: bool = False):
  COMPACT = compact
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = max_records

  @wp.kernel(module="unique", module_options=module_options, grid_stride=False)
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
  _update_efc_force(m, d, ctx, track_changes)
  _update_qfrc_constraint(m, d, ctx, stable_fast)


def _update_efc_force(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, track_changes: bool = False):
  """Update constraint forces and states, recording state changes if track_changes."""
  cone_rows = isinstance(ctx, SolverContext) and ctx.cone_coef is not None
  det_atomics = bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
  kernel = _update_constraint_efc(track_changes, cone_rows, m.opt.integrator == types.IntegratorType.DISCRETE, det_atomics)
  sparse = m.is_sparse or _sparse_compact(ctx)
  efc_stride = efc_threads_per_world(d.nworld, d.njmax, d.qacc.device, kernel) if sparse and not det_atomics else d.njmax
  wp.launch(
    kernel,
    dim=(d.nworld, efc_stride),
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
      d.njmax,
      d.nacon,
      efc_stride,
      ctx.Jaref,
      ctx.ls_exhausted,
      ctx.done,
    ],
    outputs=[
      d.efc.force,
      d.efc.state,
      ctx.quad_changed_ids,
      ctx.quad_changed_count,
      ctx.state_changed_count,
      ctx.cone_coef if cone_rows else None,
      ctx.cone_weight if cone_rows else None,
      ctx.cone_base if cone_rows else None,
    ],
  )


def _update_qfrc_constraint(
  m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, stable_fast: bool = False, write_grad: bool = False
):
  """Update qfrc_constraint from the constraint forces, and ctx.grad if write_grad."""
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
    det_atomics = bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
    kernel = _update_constraint_init_qfrc_constraint_sparse(sc, m.nv if det_atomics else 0, det_atomics)
    # a deterministic launch gives each thread one row, within the nv records per thread
    efc_stride = d.njmax if det_atomics else efc_threads_per_world(d.nworld, d.njmax, d.qacc.device, kernel)
    wp.launch(
      kernel,
      dim=(d.nworld, efc_stride),
      inputs=[
        d.nefc,
        dj.efc.J_rownnz,
        dj.efc.J_rowadr,
        dj.efc.J_colind,
        dj.efc.J,
        d.efc.force,
        dj.dof_cdof,
        d.njmax,
        efc_stride,
        changed,
        ctx.done,
      ],
      outputs=[d.qfrc_constraint],
    )
    if write_grad:
      wp.launch(
        _update_gradient_grad_cg,
        dim=(d.nworld, m.nv),
        inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, ctx.done],
        outputs=[ctx.grad],
      )
  else:
    wp.launch(
      _update_constraint_init_qfrc_constraint_dense(stable_fast, write_grad),
      dim=(d.nworld, m.nv),
      inputs=[d.nefc, d.qfrc_smooth, d.efc.J, d.efc.force, d.efc.Ma, d.njmax, changed, ctx.done],
      outputs=[d.qfrc_constraint, ctx.grad if write_grad else None],
    )

  if m.opt.integrator == types.IntegratorType.DISCRETE and (m.opt.enableflags & types.EnableBit.SLEEP):
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_treeid, d.tree_awake],
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
      ctx_search_unchanged_out[worldid] = ctx_done_in[worldid]

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
def _update_gradient_grad(stable_fast: bool, deterministic: bool = False):
  STABLE_FAST = stable_fast
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN

  @wp.kernel(module="unique", module_options=module_options, grid_stride=False)
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
def _update_gradient_grad_cg(
  # Data in:
  qfrc_smooth_in: wp.array2d[float],
  qfrc_constraint_in: wp.array2d[float],
  efc_Ma_in: wp.array2d[float],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  ctx_grad_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()

  if ctx_done_in[worldid]:
    return

  ctx_grad_out[worldid, dofid] = efc_Ma_in[worldid, dofid] - qfrc_smooth_in[worldid, dofid] - qfrc_constraint_in[worldid, dofid]


@wp.kernel(module="unique")
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


# sparse Newton Hessian init: floats per 32 B sector, one lane each per (world, row)
_INIT_H_SECTOR = 8


@cache_kernel
def _update_gradient_init_h_sparse(compact: bool, has_friction: bool = True):
  COMPACT = compact
  HAS_FRICTION = has_friction

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    M_elemid: wp.array2d[int],
    # Data in:
    nf_in: wp.array[int],
    M_in: wp.array2d[float],
    efc_D_in: wp.array2d[float],
    efc_state_in: wp.array2d[int],
    dof_frictionadr_in: wp.array2d[int],
    cdof_dof_in: wp.array2d[int],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    worldid, i, lane = wp.tid()

    if ctx_done_in[worldid]:
      return

    # write whole 32 B sectors of the world's n x n block: a partially written sector costs a DRAM
    # read once h is not in L2 (Cholesky reads the upper triangle only, nothing the zeros below it)
    n = ctx_h_out.shape[1]
    sector = wp.static(_INIT_H_SECTOR)
    row_end = (i + 1) * n

    # cdof_dof is increasing with a -1 tail, so a padded row only meets padded columns
    if wp.static(COMPACT):
      dof_i = cdof_dof_in[worldid, i]
      # the last row's sectors end with the block: min keeps this read in bounds
      dof_next = cdof_dof_in[worldid, wp.min(i + 1, n - 1)]
      padded_i = dof_i < 0
      padded_next = dof_next < 0
    else:
      dof_i = i
      dof_next = i + 1
      padded_i = i >= nv
      padded_next = i + 1 >= nv

    # the sectors starting in row i, from the one holding its diagonal (the block starts on a
    # sector, n being a multiple of 4)
    first_sector = wp.max((i * n + sector - 1) // sector, (i * n + i) // sector)
    last_sector = (row_end - 1) // sector
    for s in range(first_sector, last_sector + 1):
      e = s * sector + lane
      # a sector spans rows i and i + 1 at most: where row i ends mid-sector, it writes the next
      # row's head too
      next_row = e >= row_end
      r = wp.where(next_row, i + 1, i)
      c = e - r * n
      h = float(0.0)
      if c >= r:
        dof_r = wp.where(next_row, dof_next, dof_i)
        padded_row = wp.where(next_row, padded_next, padded_i)
        if wp.static(COMPACT):
          dof_c = cdof_dof_in[worldid, c]
          padded = dof_c < 0
        else:
          dof_c = c
          padded = c >= nv
        if padded_row:
          # identity on padded dofs keeps the factorization well conditioned
          h = wp.where(r == c, 1.0, 0.0)
        elif not padded:
          # sparse M is stored in the lower triangle and dof_c >= dof_r
          elemid = M_elemid[dof_c, dof_r]
          if elemid >= 0:
            h = M_in[worldid, elemid]
          # a DOF friction row (J = e_dof) has no JTDAJ block: it adds D to the diagonal while
          # quadratic
          if wp.static(HAS_FRICTION):
            if r == c and nf_in[worldid] > 0:
              efcid = dof_frictionadr_in[worldid, dof_r]
              if efcid >= 0:
                if efc_state_in[worldid, efcid] == types.ConstraintState.QUADRATIC.value:
                  h += efc_D_in[worldid, efcid]
      ctx_h_out[worldid, r, c] = h

  return kernel


@cache_kernel
def _add_tendon_metric_dense(max_ten_J_rownnz: int, deterministic: bool = False):
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = max_ten_J_rownnz * (max_ten_J_rownnz - 1) // 2

  @wp.kernel(module="unique", module_options=module_options)
  def kernel(
    # Model:
    ten_J_rownnz: wp.array[int],
    ten_J_rowadr: wp.array[int],
    ten_J_colind: wp.array[int],
    # Data in:
    ten_J_in: wp.array2d[float],
    efm_ts_in: wp.array2d[float],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    h_out: wp.array3d[float],
  ):
    worldid, t = wp.tid()
    if ctx_done_in[worldid]:
      return
    ts = efm_ts_in[worldid, t]
    if ts != 0.0:
      radr = ten_J_rowadr[t]
      rnnz = ten_J_rownnz[t]
      for i in range(rnnz):
        c1 = ten_J_colind[radr + i]
        v1 = ten_J_in[worldid, radr + i]
        ts_v1 = ts * v1
        for j in range(rnnz):
          c2 = ten_J_colind[radr + j]
          if c2 > c1:
            v2 = ten_J_in[worldid, radr + j]
            wp.atomic_add(h_out, worldid, c1, c2, ts_v1 * v2)

  return kernel


@cache_kernel
def _add_actuator_metric_dense(nv: int, deterministic: bool = False):
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = nv * (nv - 1) // 2

  @wp.kernel(module="unique", module_options=module_options)
  def kernel(
    # Data in:
    moment_rownnz_in: wp.array2d[int],
    moment_rowadr_in: wp.array2d[int],
    moment_colind_in: wp.array2d[int],
    actuator_moment_in: wp.array2d[float],
    efm_as_in: wp.array2d[float],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    h_out: wp.array3d[float],
  ):
    worldid, a = wp.tid()
    if ctx_done_in[worldid]:
      return
    as_val = efm_as_in[worldid, a]
    if as_val != 0.0:
      radr = moment_rowadr_in[worldid, a]
      rnnz = moment_rownnz_in[worldid, a]
      for i in range(rnnz):
        c1 = moment_colind_in[worldid, radr + i]
        v1 = actuator_moment_in[worldid, radr + i]
        as_v1 = as_val * v1
        for j in range(rnnz):
          c2 = moment_colind_in[worldid, radr + j]
          if c2 > c1:
            v2 = actuator_moment_in[worldid, radr + j]
            wp.atomic_add(h_out, worldid, c1, c2, as_v1 * v2)

  return kernel


@cache_kernel
def _add_efmK_metric_dense(nv: int, deterministic: bool = False):
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = nv

  @wp.kernel(module="unique", module_options=module_options)
  def kernel(
    # Model:
    efm_K_rownnz: wp.array[int],
    efm_K_rowadr: wp.array[int],
    efm_K_colind: wp.array[int],
    # Data in:
    efm_K_val_in: wp.array2d[float],
    # In:
    ctx_done_in: wp.array[bool],
    # Out:
    h_out: wp.array3d[float],
  ):
    worldid, r = wp.tid()
    if ctx_done_in[worldid]:
      return
    radr = efm_K_rowadr[r]
    rnnz = efm_K_rownnz[r]
    for k in range(rnnz):
      c = efm_K_colind[radr + k]
      if c >= r:
        wp.atomic_add(h_out, worldid, r, c, efm_K_val_in[worldid, radr + k])

  return kernel


@cache_kernel
def _add_flexcon_metric_dense(max_dof: int, deterministic: bool = False):
  module_options = {"enable_backward": False}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = max_dof * (max_dof + 1) // 2

  @wp.kernel(module="unique", module_options=module_options)
  def kernel(
    # Data in:
    contact_worldid_in: wp.array[int],
    nacon_in: wp.array[int],
    # In:
    efm_con_dof_in: wp.array2d[int],
    efm_con_val_in: wp.array2d[float],
    efm_con_scale_in: wp.array[float],
    efm_con_nnz_in: wp.array[int],
    ctx_done_in: wp.array[bool],
    # Out:
    h_out: wp.array3d[float],
  ):
    cid = wp.tid()
    nacon = wp.min(nacon_in[0], contact_worldid_in.shape[0])
    if cid >= nacon:
      return
    worldid = contact_worldid_in[cid]
    if ctx_done_in[worldid]:
      return
    scale = efm_con_scale_in[cid]
    nnz = efm_con_nnz_in[cid]
    if scale != 0.0 and nnz > 0:
      for i in range(nnz):
        c1 = efm_con_dof_in[cid, i]
        s_v1 = scale * efm_con_val_in[cid, i]
        for j in range(nnz):
          c2 = efm_con_dof_in[cid, j]
          if c2 >= c1:
            wp.atomic_add(h_out, worldid, c1, c2, s_v1 * efm_con_val_in[cid, j])

  return kernel


@wp.kernel
def _add_rod_twist_metric_dense(
  # Model:
  opt_timestep: wp.array[float],
  opt_disableflags: int,
  jnt_dofadr: wp.array[int],
  flex_bendingadr: wp.array[int],
  flex_bending: wp.array[float],
  flex_damping: wp.array[float],
  flex_rigid: wp.array[bool],
  flex_rod_has_twist: wp.array[bool],
  flex_rod_edgeadr: wp.array[int],
  flex_rod_edgenum: wp.array[int],
  flex_rodedge_jnt: wp.array[int],
  # In:
  ctx_done_in: wp.array[bool],
  # Out:
  h_out: wp.array3d[float],
):
  worldid, flexid = wp.tid()
  if ctx_done_in[worldid]:
    return
  if flex_rigid[flexid] or not support.rod_has_twist(flex_rod_has_twist, flexid):
    return
  h_dt = opt_timestep[worldid % opt_timestep.shape[0]]
  s1 = 0.0 if (opt_disableflags & types.DisableBit.SPRING) else h_dt * h_dt
  s2 = 0.0 if (opt_disableflags & types.DisableBit.DAMPER) else h_dt
  scale_f = s1 + s2 * wp.max(0.0, flex_damping[flexid])
  b_adr = flex_bendingadr[flexid]
  nhinge = int(flex_bending[b_adr])
  if scale_f <= 0.0 or nhinge <= 0:
    return
  eadr = flex_rod_edgeadr[flexid]
  for hi in range(nhinge):
    kt = flex_bending[b_adr + 1 + types.ROD_HINGE_SIZE * hi + 8]
    if kt <= 0.0:
      continue
    jnt0 = flex_rodedge_jnt[eadr + hi]
    jnt1 = flex_rodedge_jnt[eadr + hi + 1]
    dof0 = jnt_dofadr[jnt0] if jnt0 >= 0 else -1
    dof1 = jnt_dofadr[jnt1] if jnt1 >= 0 else -1
    if dof0 < 0 or dof1 < 0 or dof0 == dof1:
      continue
    r = wp.min(dof0, dof1)
    c = wp.max(dof0, dof1)
    h_out[worldid, r, c] -= scale_f * 2.0 * kt


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


@wp.func
def _cone_virtual_row(
  # In:
  slot: int,
  condim: int,
  mu: float,
  D0: float,
  jaref0: float,
  u: types.vec6,
  tt: float,
  friction: types.vec5,
):
  """Coefficients over its contact's rows and weight of the virtual Hessian row of row `slot`.

  The cone curvature dm [(a - mu p)(a - mu p)' + s Z'(I - uu')Z] (a = mu J_0, Z rows f_d J_d,
  p = Z'u) splits over a Householder basis h_k of the tangent space with h_1 along u: slot 0 is
  a - mu p, slots 1..m-1 are Z'h_{slot+1}, slot m is Z'h_1 with weight (1 - |u|^2) dm s.
  """
  coef = types.vec6()
  mu2 = mu * mu
  dm = math.safe_div(D0, mu2 * (1.0 + mu2))
  if dm == 0.0:
    return coef, 0.0

  t = wp.max(wp.sqrt(tt), types.MJ_MINVAL)
  inv_t = math.safe_div(1.0, t)
  uhat = u * inv_t
  uu = wp.dot(uhat, uhat)
  s = mu2 * (1.0 - jaref0 * inv_t)

  if slot == 0:
    coef[0] = mu
    for dim in range(1, 6):
      if dim < condim:
        coef[dim] = -mu * uhat[dim] * friction[dim - 1]
    return coef, dm

  # Householder reflector mapping e_1 to -+u/|u| (e_1 if u = 0)
  r = wp.sqrt(uu)
  v = types.vec6()
  if r > 0.0:
    v = uhat * (1.0 / r)
  else:
    v[1] = 1.0
  v[1] = v[1] + wp.where(v[1] >= 0.0, 1.0, -1.0)
  vv = wp.dot(v, v)

  col = slot + 1
  weight = dm * s
  if slot == condim - 1:
    col = 1
    weight = dm * s * (1.0 - uu)
  vcol = float(0.0)
  for dim in range(1, 6):
    if dim == col:
      vcol = v[dim]
  scale = -2.0 * vcol / vv
  for dim in range(1, 6):
    if dim < condim:
      h = scale * v[dim]
      if dim == col:
        h += 1.0
      coef[dim] = h * friction[dim - 1]
  return coef, weight


@wp.func
def _row_weight(D: float, state: int, cone_weight: float, active: float) -> float:
  # rows past nefc have stale states
  if active == 0.0:
    return 0.0
  if state == types.ConstraintState.QUADRATIC.value:
    return D
  if state == types.ConstraintState.CONE.value:
    return cone_weight
  return 0.0


@wp.func
def _is_cone(state: int, active: float) -> int:
  return wp.where(active != 0.0 and state == types.ConstraintState.CONE.value, 1, 0)


@wp.func
def _rewrite_cone_rows(
  # Data in:
  efc_J_in: wp.array3d[float],
  efc_state_in: wp.array2d[int],
  # In:
  ctx_cone_coef_in: wp.array2d[types.vec6],
  ctx_cone_base_in: wp.array2d[int],
  worldid: int,
  rank: int,
  lanes: int,
  nefc_world: int,
  k: int,
  nrow: int,
  ncol: int,
  J_kj: Any,
):
  """Replaces the cone-state rows of the shared Jacobian chunk with their virtual rows."""
  for it in range((nrow * ncol + lanes - 1) // lanes):
    e = it * lanes + rank
    in_tile = e < nrow * ncol
    ec = wp.where(in_tile, e, 0)
    erow = ec // ncol
    dofid = ec - erow * ncol
    efcid = k + erow
    enable = in_tile and (efcid < nefc_world)
    if enable:
      enable = efc_state_in[worldid, efcid] == types.ConstraintState.CONE.value
    value = float(0.0)
    if enable:
      base = ctx_cone_base_in[worldid, efcid]
      coef = ctx_cone_coef_in[worldid, efcid]
      # zero past condim; bound rows of contacts truncated by njmax
      for dim in range(6):
        row = base + dim
        if (coef[dim] != 0.0) and (row >= 0) and (row < nefc_world):
          value += coef[dim] * efc_J_in[worldid, row, dofid]
    wp.tile_scatter_masked(J_kj, erow, dofid, value, enable)


@cache_kernel
def _update_gradient_JTDAJ_dense_tiled_compact(nv_pad: int, tile_size: int, njmax: int, elliptic: bool = False):
  """Compact-path variant of _update_gradient_JTDAJ_dense_tiled.

  Takes M_in as a dense 3D array (nworld, nv_pad, nv_pad) -- the compacted active-DOF
  inertia block cM -- instead of a 2D CSR array.  Cholesky reads fill_mode="upper";
  cM is full-symmetric so the tile_load covers both triangles and the upper triangle
  of the result is correct.
  """
  if njmax < tile_size:
    tile_size = njmax

  TILE_SIZE_K = tile_size
  ELLIPTIC = elliptic

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
    ctx_cone_coef_in: wp.array2d[types.vec6],
    ctx_cone_weight_in: wp.array2d[float],
    ctx_cone_base_in: wp.array2d[int],
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    worldid, rank = wp.tid()

    if ctx_done_in[worldid]:
      return

    nefc = wp.min(nefc_in[worldid], njmax)

    # Load full symmetric compact inertia tile (Cholesky only reads upper triangle).
    sum_val = wp.tile_load(M_in[worldid], shape=(wp.static(nv_pad), wp.static(nv_pad)), bounds_check=True)
    lanes = wp.block_dim()

    for k in range(0, njmax, TILE_SIZE_K):
      if k >= nefc:
        break

      if wp.static(ELLIPTIC):
        J_kj = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE_K, nv_pad), offset=(k, 0), bounds_check=False, storage="shared")
        # not a shared wp.func: Warp does not reserve shared memory for a tile returned by one
        state = wp.tile_load(efc_state_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
        D_k = wp.tile_load(efc_D_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
        cone_weight = wp.tile_load(ctx_cone_weight_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
        active = wp.tile_map(
          _active_check, wp.tile_arange(TILE_SIZE_K, dtype=int), wp.tile_ones(shape=TILE_SIZE_K, dtype=int) * (nefc - k)
        )
        D_k = wp.tile_map(_row_weight, D_k, state, cone_weight, active)
        if wp.tile_reduce(wp.add, wp.tile_map(_is_cone, state, active))[0] > 0:
          _rewrite_cone_rows(
            efc_J_in, efc_state_in, ctx_cone_coef_in, ctx_cone_base_in, worldid, rank, lanes, nefc, k, TILE_SIZE_K, nv_pad, J_kj
          )
      else:
        J_kj = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE_K, nv_pad), offset=(k, 0), bounds_check=False)

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


@cache_kernel
def _update_gradient_JTDAJ_dense_tiled(nv_pad: int, tile_size: int, njmax: int, nC: int, elliptic: bool = False):
  if njmax < tile_size:
    tile_size = njmax

  TILE_SIZE_K = tile_size
  ELLIPTIC = elliptic

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # Model:
    nv: int,
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
    ctx_cone_coef_in: wp.array2d[types.vec6],
    ctx_cone_weight_in: wp.array2d[float],
    ctx_cone_base_in: wp.array2d[int],
    # Out:
    ctx_h_out: wp.array3d[float],
  ):
    worldid, rank = wp.tid()

    if ctx_done_in[worldid]:
      return

    nefc = wp.min(nefc_in[worldid], njmax)

    # CSR entries have unique destinations, so initializing M needs no atomics.
    # Densify M's upper triangle from CSR into the shared H tile (Cholesky reads fill_mode="upper").
    # Entry (i, col<=i) of M goes to the upper position (col, i); the lower triangle stays zero.
    m_tile = wp.tile_zeros(shape=(wp.static(nv_pad * nv_pad),), dtype=float, storage="shared")
    # Uniform trip count from the RUNTIME lane count: every lane iterates iters times (so the
    # collective scatter is always called by all lanes -- a divergent range(rank, nC, bd)
    # deadlocks). wp.block_dim() is 1 on the CPU backend, so lane 0 then covers every entry.
    lanes = wp.block_dim()
    iters = (nC + lanes - 1) // lanes
    for it in range(iters):
      e = it * lanes + rank
      enable = e < nC
      ec = wp.where(enable, e, 0)
      pos = M_colind[ec] * nv_pad + M_hinit_i[ec]
      wp.tile_scatter_masked(m_tile, pos, M_in[worldid, ec], enable)
    # identity on padded dofs keeps the factorization well conditioned
    for it in range((nv_pad - nv + lanes - 1) // lanes):
      dofid = nv + it * lanes + rank
      enable = dofid < nv_pad
      wp.tile_scatter_masked(m_tile, wp.where(enable, dofid * (nv_pad + 1), 0), 1.0, enable)
    sum_val = wp.tile_reshape(m_tile, (nv_pad, nv_pad))

    # Each tile processes one output tile by looping over all constraints
    for k in range(0, njmax, TILE_SIZE_K):
      if k >= nefc:
        break

      if wp.static(ELLIPTIC):
        J_kj = wp.tile_load(efc_J_in[worldid], shape=(TILE_SIZE_K, nv_pad), offset=(k, 0), bounds_check=False, storage="shared")
        # not a shared wp.func: Warp does not reserve shared memory for a tile returned by one
        state = wp.tile_load(efc_state_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
        D_k = wp.tile_load(efc_D_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
        cone_weight = wp.tile_load(ctx_cone_weight_in[worldid], shape=TILE_SIZE_K, offset=k, bounds_check=False)
        active = wp.tile_map(
          _active_check, wp.tile_arange(TILE_SIZE_K, dtype=int), wp.tile_ones(shape=TILE_SIZE_K, dtype=int) * (nefc - k)
        )
        D_k = wp.tile_map(_row_weight, D_k, state, cone_weight, active)
        if wp.tile_reduce(wp.add, wp.tile_map(_is_cone, state, active))[0] > 0:
          _rewrite_cone_rows(
            efc_J_in, efc_state_in, ctx_cone_coef_in, ctx_cone_base_in, worldid, rank, lanes, nefc, k, TILE_SIZE_K, nv_pad, J_kj
          )
      else:
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
  mu: float,
  mu_n_over_t: float,
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
    - mu * (z01 * projection2 + z02 * projection1)
    + mu_n_over_t * projection1 * projection2
    + tangent_diag * tangent_dot
  )


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
    # search has nv entries: the bounds-checked store drops the padding
    wp.tile_store(ctx_search_out[worldid], wp.tile_map(wp.mul, output_tile, -1.0))

  return kernel


@cache_kernel
def _detect_h_tile_sparsity(tile_size: int, skip_unchanged: bool = False, skip_noflip: bool = False):
  TILE_SIZE = tile_size
  SKIP_UNCHANGED = skip_unchanged
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_h_in: wp.array3d[float],
    quad_changed_count_in: wp.array[int],
    state_changed_count_in: wp.array[int],
    # Out:
    tile_nz_out: wp.array3d[int],
  ):
    worldid, pair_id, lane = wp.tid()

    if ctx_done_in[worldid]:
      return

    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    if wp.static(SKIP_UNCHANGED):
      if quad_changed_count_in[worldid] == 0:
        return

    ti = int((wp.sqrt(float(8 * pair_id + 1)) - 1.0) * 0.5)
    while ti * (ti + 1) // 2 > pair_id:
      ti -= 1
    while (ti + 1) * (ti + 2) // 2 <= pair_id:
      ti += 1
    tk = pair_id - ti * (ti + 1) // 2

    # Note: with incremental H updates, subtracting removed constraints can leave
    # tiny float32 cancellation residues that keep a tile marked nonzero; this only
    # reduces sparsity and does not affect numerical correctness.
    STRIDE = wp.block_dim()
    nz = int(0)
    base_r = tk * wp.static(TILE_SIZE)
    base_c = ti * wp.static(TILE_SIZE)
    for c_off in range(lane, wp.static(TILE_SIZE), STRIDE):
      col = base_c + c_off
      r_end = c_off if tk == ti else wp.static(TILE_SIZE)
      for r in range(r_end):
        if ctx_h_in[worldid, base_r + r, col] != 0.0:
          nz = 1
          break
      if nz != 0:
        break

    nz_tile = wp.tile(nz, preserve_type=True)
    nz_sum = wp.tile_reduce(wp.add, nz_tile)[0]
    if lane == 0:
      tile_nz_out[worldid, tk, ti] = wp.where(nz_sum > 0, 1, 0)

  return kernel


@cache_kernel
def _init_h_tile_masks(nwords: int, skip_unchanged: bool = False, skip_noflip: bool = False):
  NWORDS = nwords
  SKIP_UNCHANGED = skip_unchanged
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    M_tile_colmask: wp.array2d[wp.uint64],
    M_tile_offdiag: wp.array[int],
    # In:
    ctx_done_in: wp.array[bool],
    quad_changed_count_in: wp.array[int],
    state_changed_count_in: wp.array[int],
    # Out:
    row_mask_out: wp.array3d[wp.uint64],
    col_mask_out: wp.array3d[wp.uint64],
    tile_offdiag_out: wp.array2d[int],
  ):
    """Resets the tile masks to the tile pattern of M."""
    worldid, tk = wp.tid()

    if ctx_done_in[worldid]:
      return

    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    if wp.static(SKIP_UNCHANGED):
      if quad_changed_count_in[worldid] == 0:
        return

    for w in range(wp.static(NWORDS)):
      row_mask_out[worldid, tk, w] = wp.uint64(0)
      col_mask_out[worldid, tk, w] = M_tile_colmask[tk, w]
    tile_offdiag_out[worldid, tk] = M_tile_offdiag[tk]

  return kernel


@wp.func
def _block_col_nonzero(efc_J_in: wp.array3d[float], worldid: int, rowadr: int, rownnz: int, nrow: int, pos: int) -> bool:
  for r in range(nrow):
    if efc_J_in[worldid, 0, rowadr + r * rownnz + pos] != 0.0:
      return True
  return False


@cache_kernel
def _mark_h_tile_pattern_efc(nwords: int, skip_unchanged: bool = False, skip_noflip: bool = False):
  NWORDS = nwords
  SKIP_UNCHANGED = skip_unchanged
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    contact_dim_in: wp.array[int],
    contact_efc_address_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_state_in: wp.array2d[int],
    # In:
    ctx_done_in: wp.array[bool],
    quad_changed_count_in: wp.array[int],
    state_changed_count_in: wp.array[int],
    # Out:
    col_mask_out: wp.array3d[wp.uint64],
    tile_offdiag_out: wp.array2d[int],
  ):
    """Adds the J^T D J tiles of the rows that contribute to H."""
    worldid, efcid = wp.tid()

    if efcid >= nefc_in[worldid] or ctx_done_in[worldid]:
      return

    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    if wp.static(SKIP_UNCHANGED):
      if quad_changed_count_in[worldid] == 0:
        return

    state = efc_state_in[worldid, efcid]
    if state != types.ConstraintState.QUADRATIC.value and state != types.ConstraintState.CONE.value:
      return

    # A cone block couples the columns of all its rows, which follow the head row with the same
    # support, so the head row marks the columns nonzero in any of them.
    nrow = int(1)
    if state == types.ConstraintState.CONE.value:
      conid = efc_id_in[worldid, efcid]
      if efcid != contact_efc_address_in[conid, 0]:
        return
      # a contact that overflows njmax keeps only its leading rows
      nrow = wp.min(contact_dim_in[conid], efc_J_rownnz_in.shape[1] - efcid)

    rownnz = efc_J_rownnz_in[worldid, efcid]
    rowadr = efc_J_rowadr_in[worldid, efcid]

    # one word: collect the tiles of the row, then mark each tile's coupling to the tiles above it
    if wp.static(NWORDS == 1):
      tiles = wp.uint64(0)
      multi = wp.uint64(0)
      for a in range(rownnz):
        # sparse J keeps structural zeros, e.g. flex vertices with zero barycentric weight
        if _block_col_nonzero(efc_J_in, worldid, rowadr, rownnz, nrow, a):
          bit = wp.uint64(1) << wp.uint64(efc_J_colind_in[worldid, 0, rowadr + a] // types.TILE_SIZE_JTDAJ_DENSE)
          multi = multi | (tiles & bit)
          tiles = tiles | bit
      m = tiles
      while m != wp.uint64(0):
        t = ctz64(m)
        m = m & (m - wp.uint64(1))
        bit = wp.uint64(1) << wp.uint64(t)
        above = tiles & (bit - wp.uint64(1))
        if above != wp.uint64(0):
          wp.atomic_or(col_mask_out, worldid, t, 0, above)
        if multi & bit:
          tile_offdiag_out[worldid, t] = 1
      return

    prev_ta = int(-1)
    for a in range(rownnz):
      if not _block_col_nonzero(efc_J_in, worldid, rowadr, rownnz, nrow, a):
        continue
      ta = efc_J_colind_in[worldid, 0, rowadr + a] // types.TILE_SIZE_JTDAJ_DENSE
      if ta == prev_ta:
        continue
      prev_ta = ta
      w_a = ta // 64
      bit_a = wp.uint64(1) << wp.uint64(ta - w_a * 64)
      nsame = int(0)
      for b in range(rownnz):
        if not _block_col_nonzero(efc_J_in, worldid, rowadr, rownnz, nrow, b):
          continue
        tb = efc_J_colind_in[worldid, 0, rowadr + b] // types.TILE_SIZE_JTDAJ_DENSE
        if tb == ta:
          nsame += 1
        elif tb > ta:
          wp.atomic_or(col_mask_out, worldid, tb, w_a, bit_a)
      if nsame > 1:
        tile_offdiag_out[worldid, ta] = 1

  return kernel


@cache_kernel
def _symbolic_h_tile_cholesky(ntiles: int, skip_unchanged: bool = False, skip_noflip: bool = False, structural: bool = False):
  """Tile fill-in of the reverse block Cholesky.

  With structural, the column masks hold the tile pattern of H and tile_is_diag whether each
  diagonal tile has off-diagonal entries; otherwise both come from tile_nz.
  """
  NTILES = ntiles
  STRUCTURAL = structural
  NWORDS = (ntiles + 63) // 64
  SKIP_UNCHANGED = skip_unchanged
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    tile_nz_in: wp.array3d[int],
    quad_changed_count_in: wp.array[int],
    state_changed_count_in: wp.array[int],
    # Out:
    row_mask_out: wp.array3d[wp.uint64],
    col_mask_out: wp.array3d[wp.uint64],
    tile_is_diag_out: wp.array2d[int],
  ):
    worldid = wp.tid()

    if ctx_done_in[worldid]:
      return

    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    if wp.static(SKIP_UNCHANGED):
      if quad_changed_count_in[worldid] == 0:
        return

    if wp.static(not STRUCTURAL):
      for tk in range(NTILES):
        for w in range(wp.static(NWORDS)):
          row_mask_out[worldid, tk, w] = wp.uint64(0)
          col_mask_out[worldid, tk, w] = wp.uint64(0)
        for ti in range(tk):
          if tile_nz_in[worldid, ti, tk] > 0:
            w_i = ti // 64
            bit_i = wp.uint64(1) << wp.uint64(ti - w_i * 64)
            col_mask_out[worldid, tk, w_i] = col_mask_out[worldid, tk, w_i] | bit_i
        tile_is_diag_out[worldid, tk] = tile_nz_in[worldid, tk, tk]

    # Fill-in only needs to reach the elimination-tree parent (the next tile eliminated);
    # the parent passes it on to the remaining coupled tiles.
    for tk in range(NTILES - 1, -1, -1):
      parent = int(-1)
      for w_rev in range(wp.static(NWORDS)):
        w = wp.static(NWORDS) - 1 - w_rev
        cmask = col_mask_out[worldid, tk, w]
        if parent < 0 and cmask != wp.uint64(0):
          parent = w * 64 + 63 - clz64(cmask)
      if parent >= 0:
        p_w = parent // 64
        below_parent = (wp.uint64(1) << wp.uint64(parent - p_w * 64)) - wp.uint64(1)
        for w in range(wp.static(NWORDS)):
          cmask = col_mask_out[worldid, tk, w]
          if w == p_w:
            cmask = cmask & below_parent
          if w <= p_w and cmask != wp.uint64(0):
            col_mask_out[worldid, parent, w] = col_mask_out[worldid, parent, w] | cmask

    for tk in range(NTILES):
      tk_w = tk // 64
      tk_bit = wp.uint64(1) << wp.uint64(tk - tk_w * 64)
      for w in range(wp.static(NWORDS)):
        m = col_mask_out[worldid, tk, w]
        while m != wp.uint64(0):
          ti = w * 64 + ctz64(m)
          m = m & (m - wp.uint64(1))
          row_mask_out[worldid, ti, tk_w] = row_mask_out[worldid, ti, tk_w] | tk_bit

    for tk in range(NTILES):
      is_diag = tile_is_diag_out[worldid, tk] == 0
      for w in range(wp.static(NWORDS)):
        if row_mask_out[worldid, tk, w] != wp.uint64(0) or col_mask_out[worldid, tk, w] != wp.uint64(0):
          is_diag = False
      tile_is_diag_out[worldid, tk] = wp.where(is_diag, 1, 0)

  return kernel


@cache_kernel
def _prepare_h_rhs_work(
  tile_size: int,
  nv: int,
  skip_unchanged: bool = False,
  skip_noflip: bool = False,
):
  TILE_SIZE = tile_size
  NV = nv
  SKIP_UNCHANGED = skip_unchanged
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array2d[float],
    ctx_h_in: wp.array3d[float],
    tile_is_diag_in: wp.array2d[int],
    quad_changed_count_in: wp.array[int],
    state_changed_count_in: wp.array[int],
    # Out:
    ctx_hfactor_out: wp.array3d[float],
    rhs_work_out: wp.array3d[float],
  ):
    worldid, dof = wp.tid()

    if ctx_done_in[worldid]:
      return

    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    tk = dof // wp.static(TILE_SIZE)
    is_diag = tile_is_diag_in[worldid, tk] != 0
    b_val = ctx_grad_in[worldid, dof] if dof < wp.static(NV) else 0.0
    if is_diag:
      refactor = True
      if wp.static(SKIP_UNCHANGED):
        refactor = quad_changed_count_in[worldid] > 0
      if refactor:
        diag = ctx_h_in[worldid, dof, dof] if dof < wp.static(NV) else 1.0
        u_diag = wp.sqrt(wp.max(diag, types.MJ_MINVAL))
        ctx_hfactor_out[worldid, dof, dof] = u_diag
      else:
        u_diag = ctx_hfactor_out[worldid, dof, dof]
      rhs_work_out[worldid, dof, 0] = (b_val / u_diag) / u_diag
    else:
      rhs_work_out[worldid, dof, 0] = b_val

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked_sparse_tiles(tile_size: int, matrix_size: int, vector_size: int):
  @wp.kernel(
    module="unique",
    enable_backward=False,
    grid_stride=False,
    module_options={"enable_mathdx_gemm": False},
  )
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array3d[float],
    ctx_rhs_work_in: wp.array3d[float],
    ctx_h_in: wp.array3d[float],
    ctx_row_mask_in: wp.array3d[wp.uint64],
    ctx_col_mask_in: wp.array3d[wp.uint64],
    ctx_tile_is_diag_in: wp.array2d[int],
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

    sums = wp.static(create_blocked_cholesky_sparse_tiles_factorize_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
      ctx_h_in[worldid],
      ctx_grad_in[worldid],
      ctx_rhs_work_in[worldid],
      ctx_row_mask_in[worldid],
      ctx_col_mask_in[worldid],
      ctx_tile_is_diag_in[worldid],
      matrix_size,
      ctx_hfactor[worldid],
      ctx_search_out[worldid],
    )
    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked_sparse_tiles_skip_unchanged(
  tile_size: int,
  matrix_size: int,
  vector_size: int,
  skip_noflip: bool = False,
):
  """Blocked sparse-tile Cholesky that skips factorization when no constraints changed."""
  SKIP_NOFLIP = skip_noflip

  @wp.kernel(
    module="unique",
    enable_backward=False,
    grid_stride=False,
    module_options={"enable_mathdx_gemm": False},
  )
  def kernel(
    # In:
    ctx_done_in: wp.array[bool],
    ctx_grad_in: wp.array3d[float],
    ctx_rhs_work_in: wp.array3d[float],
    ctx_h_in: wp.array3d[float],
    ctx_row_mask_in: wp.array3d[wp.uint64],
    ctx_col_mask_in: wp.array3d[wp.uint64],
    ctx_tile_is_diag_in: wp.array2d[int],
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

    if wp.static(SKIP_NOFLIP):
      if state_changed_count_in[worldid] == 0:
        return

    if quad_changed_count_in[worldid] > 0:
      sums = wp.static(create_blocked_cholesky_sparse_tiles_factorize_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
        ctx_h_in[worldid],
        ctx_grad_in[worldid],
        ctx_rhs_work_in[worldid],
        ctx_row_mask_in[worldid],
        ctx_col_mask_in[worldid],
        ctx_tile_is_diag_in[worldid],
        matrix_size,
        ctx_hfactor[worldid],
        ctx_search_out[worldid],
      )
    else:
      sums = wp.static(create_blocked_cholesky_sparse_tiles_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
        ctx_hfactor[worldid],
        ctx_grad_in[worldid],
        ctx_rhs_work_in[worldid],
        ctx_row_mask_in[worldid],
        ctx_col_mask_in[worldid],
        ctx_tile_is_diag_in[worldid],
        matrix_size,
        ctx_search_out[worldid],
      )

    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked(tile_size: int, matrix_size: int, vector_size: int, compact: bool = False):
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # Data in:
    ncdof_in: wp.array[int],
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
    active_size = matrix_size
    if wp.static(COMPACT):
      ncdof = ncdof_in[worldid]
      active_size = wp.min(matrix_size, ((ncdof + TILE_SIZE) // TILE_SIZE) * TILE_SIZE)

    sums = wp.static(create_blocked_cholesky_augmented_factorize_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
      ctx_h_in[worldid],
      ctx_grad_in[worldid],
      active_size,
      ctx_hfactor[worldid],
      ctx_search_out[worldid],
    )
    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]

  return kernel


@cache_kernel
def _cholesky_factorize_solve_blocked(tile_size: int, matrix_size: int, vector_size: int = 0):
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

    wp.static(create_blocked_cholesky_factorize_solve_func(TILE_SIZE, matrix_size, vector_size))(
      A_in[worldid],
      b_in[worldid],
      matrix_size,
      U_out[worldid],
      x_out[worldid],
    )

  return kernel


@cache_kernel
def _update_gradient_cholesky_blocked_skip_unchanged(
  tile_size: int, matrix_size: int, vector_size: int, skip_noflip: bool = False, compact: bool = False
):
  """Blocked Cholesky that skips factorization when no constraints changed."""
  SKIP_NOFLIP = skip_noflip
  COMPACT = compact

  @wp.kernel(module="unique", enable_backward=False, grid_stride=False, module_options={"enable_mathdx_gemm": False})
  def kernel(
    # Data in:
    ncdof_in: wp.array[int],
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

    active_size = matrix_size
    if wp.static(COMPACT):
      ncdof = ncdof_in[worldid]
      active_size = wp.min(matrix_size, ((ncdof + TILE_SIZE) // TILE_SIZE) * TILE_SIZE)

    if quad_changed_count_in[worldid] > 0:
      sums = wp.static(create_blocked_cholesky_augmented_factorize_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
        ctx_h_in[worldid],
        ctx_grad_in[worldid],
        active_size,
        ctx_hfactor[worldid],
        ctx_search_out[worldid],
      )
    else:
      sums = wp.static(create_blocked_cholesky_solve_newton_func(TILE_SIZE, matrix_size, vector_size))(
        ctx_hfactor[worldid],
        ctx_grad_in[worldid],
        active_size,
        ctx_search_out[worldid],
      )

    ctx_search_dot_out[worldid] = sums[0]
    ctx_newton_decrement_out[worldid] = sums[1]

  return kernel


def _prepare_cholesky_sparse_tiles(
  m: types.Model,
  d: types.Data,
  done: wp.array[bool],
  h: wp.array3d[float],
  hfactor: wp.array3d[float],
  rhs: wp.array2d[float],
  h_tile_nz: Optional[wp.array3d[int]],
  h_row_mask: wp.array3d[wp.uint64],
  h_col_mask: wp.array3d[wp.uint64],
  h_tile_is_diag: wp.array2d[int],
  h_rhs_work: wp.array3d[float],
  quad_changed: Optional[wp.array[int]] = None,
  state_changed: Optional[wp.array[int]] = None,
  skip_unchanged: bool = False,
  skip_noflip: bool = False,
):
  """Computes h tile sparsity, runs symbolic tile Cholesky, and fills hfactor and h_rhs_work."""
  tile_size = types.TILE_SIZE_JTDAJ_DENSE
  ntiles = m.nv_pad // tile_size
  # the change counts are only read when skipping, so d.nefc stands in for them otherwise
  if quad_changed is None:
    quad_changed = d.nefc
  if state_changed is None:
    state_changed = quad_changed
  # create_solver_context allocates tile_nz only when the pattern is detected from H
  structural = h_tile_nz is None
  if structural:
    wp.launch(
      _init_h_tile_masks(h_col_mask.shape[2], skip_unchanged, skip_noflip),
      dim=(d.nworld, ntiles),
      inputs=[m.M_tile_colmask, m.M_tile_offdiag, done, quad_changed, state_changed],
      outputs=[h_row_mask, h_col_mask, h_tile_is_diag],
    )
    wp.launch(
      _mark_h_tile_pattern_efc(h_col_mask.shape[2], skip_unchanged, skip_noflip),
      dim=(d.nworld, d.njmax),
      inputs=[
        d.nefc,
        d.contact.dim,
        d.contact.efc_address,
        d.efc.id,
        d.efc.J_rownnz,
        d.efc.J_rowadr,
        d.efc.J_colind,
        d.efc.J,
        d.efc.state,
        done,
        quad_changed,
        state_changed,
      ],
      outputs=[h_col_mask, h_tile_is_diag],
    )
  else:
    npairs = ntiles * (ntiles + 1) // 2
    wp.launch_tiled(
      _detect_h_tile_sparsity(tile_size, skip_unchanged, skip_noflip),
      dim=(d.nworld, npairs),
      inputs=[done, h, quad_changed, state_changed],
      outputs=[h_tile_nz],
      block_dim=32,
    )
  wp.launch(
    _symbolic_h_tile_cholesky(ntiles, skip_unchanged, skip_noflip, structural),
    dim=d.nworld,
    inputs=[done, h_tile_nz, quad_changed, state_changed],
    outputs=[h_row_mask, h_col_mask, h_tile_is_diag],
  )
  wp.launch(
    _prepare_h_rhs_work(tile_size, m.nv, skip_unchanged, skip_noflip),
    dim=(d.nworld, m.nv_pad),
    inputs=[done, rhs, h, h_tile_is_diag, quad_changed, state_changed],
    outputs=[hfactor, h_rhs_work],
  )
def _cholesky_factorize_solve(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext,
  skip_unchanged: bool = False,
  skip_noflip: bool = False,
  compact: bool = False,
):
  """Cholesky factorize ctx.h and form the Newton search direction.

  If skip_unchanged is True (blocked path only), worlds where no constraints
  changed reuse the cached factorization in hfactor instead of refactorizing.
  """
  if m.nv <= types.H_ONE_TILE_MAX:
    wp.launch_tiled(
      _update_gradient_cholesky(m.nv_pad, skip_noflip),
      dim=d.nworld,
      inputs=[ctx.grad, ctx.h, ctx.state_changed_count if skip_noflip else d.nefc, ctx.done],
      outputs=[ctx.search, ctx.search_dot, ctx.newton_decrement],
      block_dim=m.block_dim.update_gradient_cholesky,
    )
  elif m.nv <= _SPARSE_BLOCK_CHOLESKY_DIM or m.M_tile_full:
    if skip_unchanged:
      if ctx.hfactor.ptr == ctx.h.ptr:
        raise ValueError("skip_unchanged requires separate h and hfactor buffers")
      wp.launch_tiled(
        _update_gradient_cholesky_blocked_skip_unchanged(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad, m.nv, skip_noflip, compact),
        dim=d.nworld,
        inputs=[
          d.ncdof,
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
        _update_gradient_cholesky_blocked(types.TILE_SIZE_JTDAJ_DENSE, m.nv_pad, m.nv, compact),
        dim=d.nworld,
        inputs=[d.ncdof, ctx.done, ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)), ctx.h, ctx.hfactor],
        outputs=[
          ctx.search.reshape(shape=(d.nworld, m.nv, 1)),
          ctx.search_dot,
          ctx.newton_decrement,
        ],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )
  else:
    tile_size = types.TILE_SIZE_JTDAJ_DENSE
    quad_changed = ctx.quad_changed_count if skip_unchanged else d.nefc
    state_changed = ctx.state_changed_count if skip_noflip else quad_changed
    _prepare_cholesky_sparse_tiles(
      m,
      d,
      ctx.done,
      ctx.h,
      ctx.hfactor,
      ctx.grad,
      ctx.h_tile_nz,
      ctx.h_row_mask,
      ctx.h_col_mask,
      ctx.h_tile_is_diag,
      ctx.h_rhs_work,
      quad_changed,
      state_changed,
      skip_unchanged,
      skip_noflip,
    )
    if skip_unchanged:
      if ctx.hfactor.ptr == ctx.h.ptr:
        raise ValueError("skip_unchanged requires separate h and hfactor buffers")
      wp.launch_tiled(
        _update_gradient_cholesky_blocked_sparse_tiles_skip_unchanged(tile_size, m.nv_pad, m.nv, skip_noflip),
        dim=d.nworld,
        inputs=[
          ctx.done,
          ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)),
          ctx.h_rhs_work,
          ctx.h,
          ctx.h_row_mask,
          ctx.h_col_mask,
          ctx.h_tile_is_diag,
          ctx.quad_changed_count,
          state_changed,
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
        _update_gradient_cholesky_blocked_sparse_tiles(tile_size, m.nv_pad, m.nv),
        dim=d.nworld,
        inputs=[
          ctx.done,
          ctx.grad.reshape(shape=(d.nworld, ctx.grad.shape[1], 1)),
          ctx.h_rhs_work,
          ctx.h,
          ctx.h_row_mask,
          ctx.h_col_mask,
          ctx.h_tile_is_diag,
          ctx.hfactor,
        ],
        outputs=[
          ctx.search.reshape(shape=(d.nworld, m.nv, 1)),
          ctx.search_dot,
          ctx.newton_decrement,
        ],
        block_dim=m.block_dim.update_gradient_cholesky_blocked,
      )


@cache_kernel
def _eff_cholesky_solve(tile_size: int):
  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # In:
    rhs_in: wp.array2d[float],
    h_in: wp.array3d[float],
    # Out:
    res_out: wp.array2d[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)
    mat_tile = wp.tile_load(h_in[worldid], shape=(TILE_SIZE, TILE_SIZE))
    wp.tile_cholesky_inplace(mat_tile, fill_mode="upper")
    rhs_tile = wp.tile_load(rhs_in[worldid], shape=TILE_SIZE)
    out_tile = wp.tile_cholesky_solve(mat_tile, rhs_tile, fill_mode="upper")
    wp.tile_store(res_out[worldid], out_tile)

  return kernel


@cache_kernel
def _eff_cholesky_blocked_sparse_tiles(tile_size: int, matrix_size: int, vector_size: int):
  @wp.kernel(
    module="unique",
    enable_backward=False,
    grid_stride=False,
    module_options={"enable_mathdx_gemm": False},
  )
  def kernel(
    # In:
    rhs_work_in: wp.array3d[float],
    h_in: wp.array3d[float],
    row_mask_in: wp.array3d[wp.uint64],
    col_mask_in: wp.array3d[wp.uint64],
    tile_is_diag_in: wp.array2d[int],
    # Out:
    hfactor_out: wp.array3d[float],
    res_out: wp.array3d[float],
  ):
    worldid = wp.tid()
    TILE_SIZE = wp.static(tile_size)
    wp.static(create_blocked_cholesky_sparse_tiles_factorize_solve_func(TILE_SIZE, matrix_size, vector_size))(
      h_in[worldid],
      rhs_work_in[worldid],
      rhs_work_in[worldid],
      row_mask_in[worldid],
      col_mask_in[worldid],
      tile_is_diag_in[worldid],
      matrix_size,
      hfactor_out[worldid],
      res_out[worldid],
    )

  return kernel


def _add_discrete_metric_h(
  m: types.Model,
  d: types.Data,
  h: wp.array3d[float],
  done: wp.array[bool],
  efm_con: Optional[tuple] = None,
):
  """Accumulates discrete effective-metric terms into the upper triangle of h."""
  det_atomics = bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
  wp.launch(
    _add_tendon_metric_dense(m.max_ten_J_rownnz if det_atomics else 0, det_atomics),
    dim=(d.nworld, m.ntendon),
    inputs=[m.ten_J_rownnz, m.ten_J_rowadr, m.ten_J_colind, d.ten_J, d.efm_ts, done],
    outputs=[h],
  )
  wp.launch(
    _add_actuator_metric_dense(m.nv if det_atomics else 0, det_atomics),
    dim=(d.nworld, m.nactuator),
    inputs=[d.moment_rownnz, d.moment_rowadr, d.moment_colind, d.actuator_moment, d.efm_as, done],
    outputs=[h],
  )
  if m.nefmK > 0:
    wp.launch(
      _add_efmK_metric_dense(m.nv if det_atomics else 0, det_atomics),
      dim=(d.nworld, m.nv),
      inputs=[m.efm_K_rownnz, m.efm_K_rowadr, m.efm_K_colind, d.efm_K_val, done],
      outputs=[h],
    )
  if m.nflexrodedge > 0 and _has_passive(m):
    wp.launch(
      _add_rod_twist_metric_dense,
      dim=(d.nworld, m.nflex),
      inputs=[
        m.opt.timestep,
        m.opt.disableflags,
        m.jnt_dofadr,
        m.flex_bendingadr,
        m.flex_bending,
        m.flex_damping,
        m.flex_rigid,
        m.flex_rod_has_twist,
        m.flex_rod_edgeadr,
        m.flex_rod_edgenum,
        m.flex_rodedge_jnt,
        done,
      ],
      outputs=[h],
    )
  if passive.flex_passive_contact(m):
    if efm_con is None:
      efm_con = derivative.build_efm_contact(m, d)
    efm_con_dof, efm_con_val, efm_con_scale, _, efm_con_nnz = efm_con
    wp.launch(
      _add_flexcon_metric_dense(m.efm_con_max_dof if det_atomics else 0, det_atomics),
      dim=d.naconmax,
      inputs=[
        d.contact.worldid,
        d.nacon,
        efm_con_dof,
        efm_con_val,
        efm_con_scale,
        efm_con_nnz,
        done,
      ],
      outputs=[h],
    )


def use_eff_solve_newton(m: types.Model, d: types.Data) -> bool:
  """Whether direct Cholesky can solve the unconstrained effective-metric system."""
  return (
    m.opt.solver == types.SolverType.NEWTON
    and passive.flex_passive_contact(m)
    and not m.has_non_simple_flex
    and m.flex_interp_assemblable
    and not (m.opt.enableflags & types.EnableBit.SLEEP)
    and m.nv > 0
    and d.naconmax > 0
  )


@event_scope
def eff_solve_newton(m: types.Model, d: types.Data, qacc: wp.array2d[float]):
  """Solves (M + S) qacc = qfrc_eff via direct Cholesky factorization."""
  nworld = d.nworld
  nv = m.nv
  nv_pad = m.nv_pad
  done = wp.zeros((nworld,), dtype=bool)
  h = wp.empty((nworld, nv_pad, nv_pad), dtype=float)
  wp.launch(
    _update_gradient_init_h_sparse(False, False),
    dim=(nworld, nv_pad, _INIT_H_SECTOR),
    inputs=[nv, m.M_elemid, d.nf, d.qH, d.efc.D, d.efc.state, d.dof_frictionadr, d.cdof_dof, done],
    outputs=[h],
  )
  _add_discrete_metric_h(m, d, h, done)

  # the one-tile and blocked solvers read a zero-padded right-hand side
  if nv <= _SPARSE_BLOCK_CHOLESKY_DIM or m.M_tile_full:
    rhs = wp.zeros((nworld, nv_pad), dtype=float)
  else:
    rhs = wp.empty((nworld, nv), dtype=float)
  wp.launch(
    _add_qfrc_smooth_eff,
    dim=(nworld, nv),
    inputs=[m.opt.enableflags, m.dof_treeid, d.tree_awake, d.qfrc_smooth, d.efm_c, d.efm_ca],
    outputs=[rhs],
  )

  if nv <= types.H_ONE_TILE_MAX:
    wp.launch_tiled(
      _eff_cholesky_solve(nv_pad),
      dim=nworld,
      inputs=[rhs, h],
      outputs=[qacc],
      block_dim=m.block_dim.update_gradient_cholesky,
    )
  elif nv <= _SPARSE_BLOCK_CHOLESKY_DIM or m.M_tile_full:
    wp.launch_tiled(
      _cholesky_factorize_solve_blocked(types.TILE_SIZE_JTDAJ_DENSE, nv_pad, nv),
      dim=nworld,
      inputs=[h, rhs.reshape(shape=(nworld, nv_pad, 1))],
      outputs=[h, qacc.reshape(shape=(nworld, nv, 1))],
      block_dim=m.block_dim.update_gradient_cholesky_blocked,
    )
  else:
    tile_size = types.TILE_SIZE_JTDAJ_DENSE
    ntiles = nv_pad // tile_size
    nwords = (ntiles + 63) // 64
    h_tile_nz = wp.empty((nworld, ntiles, ntiles), dtype=int)
    h_row_mask = wp.empty((nworld, ntiles, nwords), dtype=wp.uint64)
    h_col_mask = wp.empty((nworld, ntiles, nwords), dtype=wp.uint64)
    h_tile_is_diag = wp.empty((nworld, ntiles), dtype=int)
    h_rhs_work = wp.empty((nworld, nv_pad, 1), dtype=float)
    _prepare_cholesky_sparse_tiles(m, d, done, h, h, rhs, h_tile_nz, h_row_mask, h_col_mask, h_tile_is_diag, h_rhs_work)
    wp.launch_tiled(
      _eff_cholesky_blocked_sparse_tiles(tile_size, nv_pad, nv),
      dim=nworld,
      inputs=[h_rhs_work, h, h_row_mask, h_col_mask, h_tile_is_diag],
      outputs=[h, qacc.reshape(shape=(nworld, nv, 1))],
      block_dim=m.block_dim.update_gradient_cholesky_blocked,
    )


# ---------------------------------------------------------------------------
# Constraint groups contain consecutive rows with identical sparse support. Each thread group
# accumulates their upper-triangular Hessian block, including elliptic cone curvature.
# ---------------------------------------------------------------------------
_JTDAJ_THREADS_PER_GROUP = 32
_JTDAJ_OVERSUBSCRIBE_WAVES = 6


@cache_kernel
def _JTDACJ_sparse(
  compact: bool,
  cone_type: types.ConeType,
  max_condim: int,
  block_dim: int = types.BlockDim.update_gradient_JTDAJ_sparse,
  max_records: int = 0,
  is_discrete: bool = False,
  deterministic: bool = False,
):
  COMPACT = compact
  IS_DISCRETE = is_discrete
  ELLIPTIC = cone_type == types.ConeType.ELLIPTIC
  MAX_CONDIM = max_condim
  module_options = {"enable_backward": False, "block_dim": block_dim}
  if deterministic:
    module_options["deterministic"] = wp.DeterministicMode.RUN_TO_RUN
    module_options["deterministic_max_records"] = max_records

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
      D0 = efc_D_in[worldid, efcid0]
      mu_scale = float(0.0)
      if wp.static(IS_DISCRETE):
        if block_rows < 2:
          return types.vec16()
        mu_scale = _elliptic_mu_scale(D0, efc_D_in[worldid, efcid0 + 1])
      else:
        mu_scale = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
      mu = fri[0] * mu_scale
      mu2 = mu * mu
      dm = math.safe_div(D0, mu2 * (1.0 + mu2))
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
      inv_t = math.safe_div(1.0, t)
      for dim in range(1, wp.static(condim)):
        terms[dim] = terms[dim] * inv_t

      mu_n_over_t = mu * n * inv_t
      tangent_diag = mu2 - mu_n_over_t

      # Layout: normalized tangent u/t[1:6], scales[6:12], dm, mu, mu*n/t, tangent diagonal.
      terms[12] = dm
      terms[13] = mu
      terms[14] = mu_n_over_t
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

  @wp.kernel(module="unique", module_options=module_options, grid_stride=True)
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


def _jtdaj_groups_per_world(nworld: int, max_groups: int, kernel: wp.Kernel, min_groups: int = 1) -> int:
  # Per-world work is sized for capacity and often idle, so cap groups at a few resident waves.
  block_size, min_grid_size = wp.get_suggested_block_size(kernel)
  device_warps = max(1, block_size * min_grid_size // _JTDAJ_THREADS_PER_GROUP)
  return max(min_groups, min(max_groups, _JTDAJ_OVERSUBSCRIBE_WAVES * device_warps // nworld))


def _update_gradient(m: types.Model, d: types.Data, ctx: SolverContext, compact: bool = False, grad: bool = True):
  # grad = Ma - qfrc_smooth - qfrc_constraint (CG iterations set it in _update_qfrc_constraint)
  if m.opt.solver == types.SolverType.CG:
    if grad:
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
      _update_gradient_grad(False, bool(m.opt.deterministic & types.DeterminismType.ATOMICS)),
      dim=(d.nworld, m.nv),
      inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, d.nefc, ctx.done],
      outputs=[ctx.grad, ctx.grad_dot],
    )

  is_discrete = m.opt.integrator == types.IntegratorType.DISCRETE
  if m.opt.solver == types.SolverType.CG:
    if is_discrete:
      if m.nefmK > 0 or m.nflexrodhinge > 0 or m.efm0_active or (m.opt.enableflags & types.EnableBit.SLEEP):
        derivative.eff_prec(
          m,
          d,
          ctx.Mgrad,
          ctx.grad,
          x_scratch=ctx.solve_ld_scratch,
          tw_diag=ctx.tw_diag,
        )
      else:
        smooth.solve_LD(m, d, d.qHLD, d.qHDiagInv, ctx.Mgrad, ctx.grad, x_scratch=ctx.solve_ld_scratch)
    else:
      smooth.solve_m(m, d, ctx.Mgrad, ctx.grad, x_scratch=ctx.solve_ld_scratch)
  elif m.opt.solver == types.SolverType.NEWTON:
    # h = M + (efc_J.T * efc_D * active) @ efc_J
    sc = _sparse_compact(ctx)
    if m.is_sparse or sc:
      mj = ctx.compact_m_full if sc else m
      dj = ctx.compact_d_full if sc else d
      m_mat = dj.qH if is_discrete else dj.M
      wp.launch(
        _update_gradient_init_h_sparse(sc),
        dim=(d.nworld, m.nv_pad, _INIT_H_SECTOR),
        inputs=[
          mj.nv,
          mj.M_elemid,
          d.nf,
          m_mat,
          d.efc.D,
          d.efc.state,
          d.dof_frictionadr,
          dj.cdof_dof,
          ctx.done,
        ],
        outputs=[ctx.h],
      )

      max_condim = 3
      if m.opt.cone == types.ConeType.ELLIPTIC and m.nmaxcondim > 3:
        max_condim = int(m.nmaxcondim)
      elliptic = m.opt.cone == types.ConeType.ELLIPTIC
      threads_per_group = 1 if elliptic and wp.get_device().is_cpu else _JTDAJ_THREADS_PER_GROUP
      block_dim = threads_per_group if elliptic else mj.block_dim.update_gradient_JTDAJ_sparse
      jtdaj_kernel = _JTDACJ_sparse(sc, m.opt.cone, max_condim, block_dim, 0, is_discrete, False)
      groups_per_world = _jtdaj_groups_per_world(d.nworld, d.njmax, jtdaj_kernel)
      if bool(m.opt.deterministic & types.DeterminismType.ATOMICS):
        entries_per_lane = ceil((mj.nv * (mj.nv + 1) // 2) / threads_per_group)
        max_records = ceil(d.njmax / groups_per_world) * entries_per_lane
        jtdaj_kernel = _JTDACJ_sparse(sc, m.opt.cone, max_condim, block_dim, max_records, is_discrete, True)
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
      wp.launch(
        jtdaj_kernel,
        dim=(d.nworld, groups_per_world, threads_per_group),
        inputs=jtdaj_inputs,
        outputs=[ctx.h],
        block_dim=block_dim,
      )
    else:
      m_mat = d.qH if is_discrete else d.M
      elliptic = m.opt.cone == types.ConeType.ELLIPTIC
      if compact:
        # compact path: d.M is the dense 3D compact inertia block (nworld, nv_pad, nv_pad)
        wp.launch_tiled(
          _update_gradient_JTDAJ_dense_tiled_compact(m.nv_pad, types.TILE_SIZE_JTDAJ_DENSE, d.njmax, elliptic),
          dim=d.nworld,
          inputs=[
            d.nefc,
            m_mat,
            d.efc.J,
            d.efc.D,
            d.efc.state,
            ctx.done,
            ctx.cone_coef,
            ctx.cone_weight,
            ctx.cone_base,
          ],
          outputs=[ctx.h],
          block_dim=m.block_dim.update_gradient_JTDAJ_dense,
        )
      else:
        wp.launch_tiled(
          _update_gradient_JTDAJ_dense_tiled(m.nv_pad, types.TILE_SIZE_JTDAJ_DENSE, d.njmax, m.M_colind.shape[0], elliptic),
          dim=d.nworld,
          inputs=[
            m.nv,
            m.M_colind,
            m.M_hinit_i,
            d.nefc,
            m_mat,
            d.efc.J,
            d.efc.D,
            d.efc.state,
            ctx.done,
            ctx.cone_coef,
            ctx.cone_weight,
            ctx.cone_base,
          ],
          outputs=[ctx.h],
          block_dim=m.block_dim.update_gradient_JTDAJ_dense,
        )

    if is_discrete:
      _add_discrete_metric_h(m, d, ctx.h, ctx.done, ctx.efm_con)

    _cholesky_factorize_solve(m, d, ctx, compact=compact)
  else:
    raise ValueError(f"Unknown solver type: {m.opt.solver}")

  if is_discrete and (m.opt.enableflags & types.EnableBit.SLEEP):
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, min(m.nv, ctx.Mgrad.shape[1])),
      inputs=[m.dof_treeid, d.tree_awake],
      outputs=[ctx.Mgrad],
    )
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_treeid, d.tree_awake],
      outputs=[ctx.search],
    )


def _update_h_incremental(m: types.Model, d: types.Data, ctx: SolverContext):
  """Update upper triangle of H with delta from constraints that changed QUADRATIC state."""
  det_atomics = bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
  sc = _sparse_compact(ctx)
  if m.is_sparse or sc:
    mj = ctx.compact_m_full if sc else m
    dj = ctx.compact_d_full if sc else d
    inc_kernel = _update_gradient_h_incremental_sparse(sc)
    slots = _jtdaj_groups_per_world(d.nworld, ctx.quad_changed_ids.shape[1], inc_kernel)
    if det_atomics:
      entries_per_lane = ceil((mj.nv * (mj.nv + 1) // 2) / _JTDAJ_THREADS_PER_GROUP)
      max_records = ceil(ctx.quad_changed_ids.shape[1] / slots) * entries_per_lane
      inc_kernel = _update_gradient_h_incremental_sparse(sc, max_records, True)
    wp.launch(
      inc_kernel,
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
    # two groups per world keep busy iterations at large nworld as fast as one thread per element
    ntri = m.nv * (m.nv + 1) // 2
    groups = _jtdaj_groups_per_world(d.nworld, ceil(ntri / _JTDAJ_THREADS_PER_GROUP), _update_gradient_h_incremental, 2)
    threads_per_world = min(groups * _JTDAJ_THREADS_PER_GROUP, ntri)
    wp.launch(
      _update_gradient_h_incremental,
      dim=(d.nworld, threads_per_world),
      inputs=[
        m.nv,
        d.efc.J,
        d.efc.D,
        d.efc.state,
        ctx.quad_changed_ids,
        ctx.quad_changed_count,
        threads_per_world,
      ],
      outputs=[ctx.h],
    )


def _update_gradient_incremental(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext,
  stable_fast: bool = False,
  compact: bool = False,
  h_stream: wp.Stream | None = None,
  gradient_ready: bool = False,
):
  """Incremental gradient update: update H for changed constraints + re-factorize.

  Skips the full J^T*D*J rebuild by applying only the delta from constraints
  that changed QUADRATIC state, then re-factorizes and solves. If h_stream is given,
  the caller already launched _update_h_incremental on it. If gradient_ready, the
  caller's fused kernel already produced the gradient.
  """
  if not gradient_ready:
    changed = ctx.state_changed_count if stable_fast else d.nefc
    wp.launch(
      _update_gradient_zero_grad_dot(stable_fast),
      dim=d.nworld,
      inputs=[changed, ctx.alpha, ctx.done],
      outputs=[ctx.grad_dot, ctx.newton_decrement, ctx.grad_scale, ctx.search_unchanged],
    )

    det_atomics = bool(m.opt.deterministic & types.DeterminismType.ATOMICS)
    wp.launch(
      _update_gradient_grad(stable_fast, det_atomics),
      dim=(d.nworld, m.nv),
      inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, changed, ctx.done],
      outputs=[ctx.grad, ctx.grad_dot],
    )

  if h_stream is None:
    _update_h_incremental(m, d, ctx)
  else:
    wp.wait_stream(h_stream)

  _cholesky_factorize_solve(m, d, ctx, skip_unchanged=True, skip_noflip=stable_fast, compact=compact)
  if m.opt.integrator == types.IntegratorType.DISCRETE and (m.opt.enableflags & types.EnableBit.SLEEP):
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, min(m.nv, ctx.Mgrad.shape[1])),
      inputs=[m.dof_treeid, d.tree_awake],
      outputs=[ctx.Mgrad],
    )
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_treeid, d.tree_awake],
      outputs=[ctx.search],
    )


@cache_kernel
def _solve_search_update_cg_tiled(warn_overflow: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_tolerance: wp.array[float],
    opt_iterations: int,
    # In:
    ctx_inertia_scale_in: wp.array[float],
    ctx_grad_in: wp.array2d[float],
    ctx_Mgrad_in: wp.array2d[float],
    ctx_prev_grad_in: wp.array2d[float],
    ctx_prev_Mgrad_in: wp.array2d[float],
    ctx_search_in: wp.array2d[float],
    ctx_improvement_in: wp.array[float],
    ctx_alpha_in: wp.array[float],
    ctx_done_in: wp.array[bool],
    # Data out:
    solver_niter_out: wp.array[int],
    overflow_out: wp.array[int],
    # Out:
    ctx_search_out: wp.array2d[float],
    ctx_search_dot_out: wp.array[float],
    ctx_prev_grad_out: wp.array2d[float],
    ctx_prev_Mgrad_out: wp.array2d[float],
    nsolving_out: wp.array[int],
    ctx_done_out: wp.array[bool],
  ):
    worldid, tid = wp.tid()

    if ctx_done_in[worldid]:
      return

    niter = solver_niter_out[worldid] + 1
    BLOCK_DIM = wp.block_dim()

    # Hager-Zhang: d' * y, y' * M * y, y' * Mgrad, d' * grad, and grad' * grad
    local_sums = types.vec5(0.0)
    for dofid in range(tid, nv, BLOCK_DIM):
      grad = ctx_grad_in[worldid, dofid]
      Mgrad = ctx_Mgrad_in[worldid, dofid]
      graddif = grad - ctx_prev_grad_in[worldid, dofid]
      mgraddif = Mgrad - ctx_prev_Mgrad_in[worldid, dofid]
      search = ctx_search_in[worldid, dofid]
      local_sums += types.vec5(search * graddif, graddif * mgraddif, graddif * Mgrad, search * grad, grad * grad)

    # also syncs the block before niter, done, search and prev_grad are written
    sums = wp.tile_reduce(wp.add, wp.tile(local_sums, preserve_type=True))[0]

    tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]
    scale = ctx_inertia_scale_in[worldid]
    alpha = ctx_alpha_in[worldid]
    improvement = scale * ctx_improvement_in[worldid]
    gradient = scale * wp.sqrt(sums[4])
    done = (alpha == 0.0) or (improvement > 0.0 and improvement < tolerance) or (gradient < tolerance)
    stop = done or niter == opt_iterations

    if tid == 0:
      solver_niter_out[worldid] = niter
      if stop:
        if not done:
          if wp.static(bool(warn_overflow & OverflowType.ITERATIONS)):
            wp.printf(
              "solver iterations limit reached - please increase iterations beyond %u\n"
              "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.ITERATIONS (or = 0 for all)\n",
              opt_iterations,
            )
          overflow_out[worldid] = overflow_out[worldid] | OverflowType.ITERATIONS
        ctx_done_out[worldid] = True
        wp.atomic_add(nsolving_out, 0, -1)  # kernel_analyzer: ignore[atomic]

    if stop:
      return

    d_dot_y = sums[0]
    if d_dot_y < types.MJ_MINVAL:
      # conjugacy lost: restart to steepest descent
      beta = 0.0
    else:
      inv_dy = 1.0 / d_dot_y
      beta_hz = (sums[2] - 2.0 * sums[1] * inv_dy * sums[3]) * inv_dy
      # diverge from MuJoCo C (eta_k < 0): clamp beta >= 0 (HZ+) to avoid amplifying 1-ulp
      # parallel-reduction gradient differences (e.g. tree.xml) and an extra d' * d norm reduction
      beta = wp.max(0.0, beta_hz)

    local_search_dot = float(0.0)
    for dofid in range(tid, nv, BLOCK_DIM):
      mgrad = ctx_Mgrad_in[worldid, dofid]
      search = -1.0 * mgrad + beta * ctx_search_in[worldid, dofid]

      ctx_search_out[worldid, dofid] = search
      local_search_dot += search * search

      ctx_prev_grad_out[worldid, dofid] = ctx_grad_in[worldid, dofid]
      ctx_prev_Mgrad_out[worldid, dofid] = mgrad

    search_dot_sum = wp.tile_reduce(wp.add, wp.tile(local_search_dot, preserve_type=True))

    if tid == 0:
      ctx_search_dot_out[worldid] = search_dot_sum[0]

  return kernel


@cache_kernel
def _solve_done(warn_overflow: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_tolerance: wp.array[float],
    opt_iterations: int,
    # In:
    ctx_inertia_scale_in: wp.array[float],
    ctx_grad_dot_in: wp.array[float],
    ctx_newton_decrement_in: wp.array[float],
    ctx_improvement_in: wp.array[float],
    ctx_alpha_in: wp.array[float],
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
    scale = ctx_inertia_scale_in[worldid]

    alpha = ctx_alpha_in[worldid]
    improvement = scale * ctx_improvement_in[worldid]
    gradient = scale * wp.sqrt(ctx_grad_dot_in[worldid])
    model_improvement = scale * 0.5 * ctx_newton_decrement_in[worldid]
    done = (
      (alpha == 0.0)
      or (improvement > 0.0 and improvement < tolerance)
      or (gradient < tolerance)
      or (model_improvement < tolerance)
    )
    if done or solver_niter_out[worldid] == opt_iterations:
      # if the solver has converged or the maximum number of iterations has been reached then
      # mark this world as done and remove it from the number of unconverged worlds
      if not done and solver_niter_out[worldid] == opt_iterations:
        if wp.static(bool(warn_overflow & OverflowType.ITERATIONS)):
          wp.printf(
            "solver iterations limit reached - please increase iterations beyond %u\n"
            "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.ITERATIONS (or = 0 for all)\n",
            opt_iterations,
          )
        overflow_out[worldid] = overflow_out[worldid] | OverflowType.ITERATIONS
      ctx_done_out[worldid] = True
      wp.atomic_add(nsolving_out, 0, -1)  # kernel_analyzer: ignore[atomic]

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
  # Small sparse pyramidal Newton models can evaluate constraint states,
  # generalized forces, and the gradient together in one block per world.
  _linesearch(m, d, ctx)

  incremental = _use_incremental(m)
  fuse_constraint_update = (
    incremental
    and m.is_sparse
    and not compact
    and m.nv <= 50
    # The fused kernel accumulates qfrc_constraint and the gradient with unordered atomics.
    and not (m.opt.deterministic & types.DeterminismType.ATOMICS)
    # Discrete sleeping needs separate force zeroing before gradient evaluation.
    and not (m.opt.integrator == types.IntegratorType.DISCRETE and (m.opt.enableflags & types.EnableBit.SLEEP))
    and d.qacc.device.is_cuda
    and d.nworld >= 4 * d.qacc.device.sm_count
  )

  if fuse_constraint_update:
    wp.launch_tiled(
      _update_constraint_qfrc_gradient_sparse_tiled(m.nv),
      dim=d.nworld,
      inputs=[
        d.ne,
        d.nf,
        d.nefc,
        d.qfrc_smooth,
        d.efc.J_rownnz,
        d.efc.J_rowadr,
        d.efc.J_colind,
        d.efc.J,
        d.efc.D,
        d.efc.frictionloss,
        d.efc.Ma,
        d.njmax,
        ctx.Jaref,
        ctx.ls_exhausted,
        ctx.alpha,
        ctx.done,
      ],
      outputs=[
        d.qfrc_constraint,
        d.efc.force,
        d.efc.state,
        ctx.quad_changed_ids,
        ctx.quad_changed_count,
        ctx.state_changed_count,
        ctx.grad,
        ctx.grad_dot,
        ctx.newton_decrement,
        ctx.grad_scale,
        ctx.search_unchanged,
      ],
      block_dim=m.block_dim.update_constraint_qfrc_gradient_sparse_tiled,
    )
    _update_gradient_incremental(m, d, ctx, stable_fast=True, compact=compact, gradient_ready=True)
  elif incremental:
    # Must complete before _update_constraint_efc which atomically increments.
    wp.launch(
      _zero_change_counters,
      dim=d.nworld,
      outputs=[ctx.quad_changed_count, ctx.state_changed_count],
    )

    _update_efc_force(m, d, ctx, track_changes=True)

    # The H update only needs the state changes recorded above, so it can run beside
    # qfrc_constraint and the gradient.
    h_stream = side_stream(d.qacc.device)
    if h_stream is not None:
      with wp.ScopedStream(h_stream):
        _update_h_incremental(m, d, ctx)

    # The tracking also enables the stable-state fast path: worlds with no state
    # flips this iteration were exactly quadratic over the step, so grad/search
    # only changed by a scalar along the same ray. Skip their qfrc/grad/
    # solve/search updates and track the scalar in ctx.grad_scale.
    _update_qfrc_constraint(m, d, ctx, stable_fast=True)
    _update_gradient_incremental(m, d, ctx, stable_fast=True, compact=compact, h_stream=h_stream)
  else:
    _update_efc_force(m, d, ctx)
    cg = m.opt.solver == types.SolverType.CG
    _update_qfrc_constraint(m, d, ctx, write_grad=cg)
    _update_gradient(m, d, ctx, compact=compact, grad=not cg)

  # Hager-Zhang conjugate direction update
  if m.opt.solver == types.SolverType.CG:
    wp.launch_tiled(
      _solve_search_update_cg_tiled(int(m.opt.warn_overflow)),
      dim=d.nworld,
      inputs=[
        m.nv,
        m.opt.tolerance,
        m.opt.iterations,
        ctx.inertia_scale,
        ctx.grad,
        ctx.Mgrad,
        ctx.prev_grad,
        ctx.prev_Mgrad,
        ctx.search,
        ctx.improvement,
        ctx.alpha,
        ctx.done,
      ],
      outputs=[
        d.solver_niter,
        d.overflow,
        ctx.search,
        ctx.search_dot,
        ctx.prev_grad,
        ctx.prev_Mgrad,
        nsolving,
        ctx.done,
      ],
      block_dim=m.block_dim.solve_search_update_cg,
    )

  else:
    wp.launch(
      _solve_done(int(m.opt.warn_overflow)),
      dim=d.nworld,
      inputs=[
        m.opt.tolerance,
        m.opt.iterations,
        ctx.inertia_scale,
        ctx.grad_dot,
        ctx.newton_decrement,
        ctx.improvement,
        ctx.alpha,
        ctx.done,
      ],
      outputs=[d.solver_niter, d.overflow, nsolving, ctx.done],
    )


@wp.func
def _elliptic_row_cost(
  # Data in:
  contact_efc_address_in: wp.array2d[int],
  efc_D_in: wp.array2d[float],
  # In:
  friction: types.vec5,
  mu: float,
  dim: int,
  Dm: float,
  conid: int,
  worldid: int,
  jar: float,
  jar_in: wp.array2d[float],
) -> float:
  N = jar * mu
  T2 = float(0.0)
  for j in range(1, dim):
    uj = jar_in[worldid, contact_efc_address_in[conid, j]] * friction[j - 1]
    T2 += uj * uj
  T = wp.sqrt(T2)
  if N >= mu * T or (T <= 0.0 and N >= 0.0):
    return 0.0
  if mu * N + T <= 0.0 or (T <= 0.0 and N < 0.0):
    cost = float(0.0)
    for j in range(dim):
      rowj = contact_efc_address_in[conid, j]
      jarj = jar_in[worldid, rowj]
      cost += 0.5 * efc_D_in[worldid, rowj] * jarj * jarj
    return cost
  NmT = N - mu * T
  return 0.5 * Dm * NmT * NmT


@wp.func
def _constraint_row_cost_diff(
  # Data in:
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  efc_type_in: wp.array2d[int],
  efc_id_in: wp.array2d[int],
  efc_D_in: wp.array2d[float],
  efc_frictionloss_in: wp.array2d[float],
  # In:
  impratio_invsqrt: float,
  nrow_equality: int,
  nrow_friction: int,
  efcid: int,
  worldid: int,
  jar_warmstart_in: wp.array2d[float],
  jar_smooth_in: wp.array2d[float],
) -> float:
  """Cost difference (warmstart - smooth) of row efcid (MuJoCo C mj_constraintUpdate_impl).

  An elliptic contact's cost is counted on its first row.
  """
  D = efc_D_in[worldid, efcid]
  jar_w = jar_warmstart_in[worldid, efcid]
  jar_s = jar_smooth_in[worldid, efcid]
  if efcid < nrow_equality:
    return 0.5 * D * (jar_w * jar_w - jar_s * jar_s)
  if efcid < nrow_equality + nrow_friction:
    f = efc_frictionloss_in[worldid, efcid]
    rf = math.safe_div(f, D)
    return _eval_frictionloss_cost_diff(jar_s, jar_w - jar_s, f, rf, D)
  if efc_type_in[worldid, efcid] != types.ConstraintType.CONTACT_ELLIPTIC:
    cost_w = 0.5 * D * jar_w * jar_w if jar_w < 0.0 else 0.0
    cost_s = 0.5 * D * jar_s * jar_s if jar_s < 0.0 else 0.0
    return cost_w - cost_s
  conid = efc_id_in[worldid, efcid]
  if efcid != contact_efc_address_in[conid, 0]:
    return 0.0
  friction = contact_friction_in[conid]
  efcid1 = contact_efc_address_in[conid, 1]
  if efcid1 >= 0:
    mu_scale = _elliptic_mu_scale(D, efc_D_in[worldid, efcid1])
  else:
    mu_scale = impratio_invsqrt
  mu = friction[0] * mu_scale
  dim = contact_dim_in[conid]
  Dm = math.safe_div(D, mu * mu * (1.0 + mu * mu))
  return _elliptic_row_cost(
    contact_efc_address_in, efc_D_in, friction, mu, dim, Dm, conid, worldid, jar_w, jar_warmstart_in
  ) - _elliptic_row_cost(contact_efc_address_in, efc_D_in, friction, mu, dim, Dm, conid, worldid, jar_s, jar_smooth_in)


@wp.kernel(module="unique", enable_backward=False)
def _warmstart_cost(
  # Model:
  nv: int,
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
  # In:
  jar_warmstart_in: wp.array2d[float],
  jar_smooth_in: wp.array2d[float],
  da_in: wp.array2d[float],
  Mda_in: wp.array2d[float],
  # Out:
  use_smooth_out: wp.array[bool],
):
  """Selects qacc_smooth when it costs less than the warmstart, one block per world."""
  worldid, tid = wp.tid()
  nthread = wp.block_dim()
  if tid == 0:
    use_smooth_out[worldid] = False
  nefc = wp.min(nefc_in[worldid], efc_type_in.shape[1])
  if nefc == 0:
    return
  impratio_invsqrt = opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]
  ne = ne_in[worldid]
  nf = nf_in[worldid]
  # cost difference warmstart - smooth, rows and dofs split over the block
  diff = float(0.0)
  for efcid in range(tid, nefc, nthread):
    diff += _constraint_row_cost_diff(
      contact_friction_in,
      contact_dim_in,
      contact_efc_address_in,
      efc_type_in,
      efc_id_in,
      efc_D_in,
      efc_frictionloss_in,
      impratio_invsqrt,
      ne,
      nf,
      efcid,
      worldid,
      jar_warmstart_in,
      jar_smooth_in,
    )
  for dofid in range(tid, nv, nthread):
    diff += 0.5 * da_in[worldid, dofid] * Mda_in[worldid, dofid]
  total = wp.tile_sum(wp.tile(diff))[0]
  if tid == 0:
    use_smooth_out[worldid] = total > 0.0


@wp.kernel
def _warmstart_diff(
  # Data in:
  qacc_in: wp.array2d[float],
  qacc_smooth_in: wp.array2d[float],
  # Out:
  da_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  da_out[worldid, dofid] = qacc_in[worldid, dofid] - qacc_smooth_in[worldid, dofid]


@wp.kernel
def _warmstart_select(
  # Data in:
  qacc_smooth_in: wp.array2d[float],
  # In:
  use_smooth_in: wp.array[bool],
  # Data out:
  qacc_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  if use_smooth_in[worldid]:
    qacc_out[worldid, dofid] = qacc_smooth_in[worldid, dofid]


def _warmstart_best(m: types.Model, d: types.Data, ctx: SolverContext):
  """Starts from the better of qacc_warmstart and qacc_smooth (MuJoCo C warmstart).

  Uses solver context buffers that are overwritten before their first use in the solve:
  Jaref and jv for the two constraint residuals, search and mv for the metric Gauss term, and
  search_unchanged for the per-world choice.
  """
  # jar at qacc_smooth (into ctx.jv) and at qacc_warmstart (into ctx.Jaref, d.qacc = warmstart)
  _init_jaref(m, dataclasses.replace(d, qacc=d.qacc_smooth), dataclasses.replace(ctx, Jaref=ctx.jv))
  _init_jaref(m, d, ctx)
  wp.launch(_warmstart_diff, dim=(d.nworld, m.nv), inputs=[d.qacc, d.qacc_smooth], outputs=[ctx.search])
  derivative.eff_mul_m(m, d, ctx.mv, ctx.search, efm_con=ctx.efm_con, flex_scratch=ctx.flex_scratch)
  wp.launch_tiled(
    _warmstart_cost,
    dim=d.nworld,
    block_dim=m.block_dim.warmstart_cost,
    inputs=[
      m.nv,
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
      ctx.Jaref,
      ctx.jv,
      ctx.search,
      ctx.mv,
    ],
    outputs=[ctx.search_unchanged],
  )
  wp.launch(_warmstart_select, dim=(d.nworld, m.nv), inputs=[d.qacc_smooth, ctx.search_unchanged], outputs=[d.qacc])


def _init_jaref(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext):
  """Computes ctx.Jaref = d.efc.J @ d.qacc - d.efc.aref."""
  # ctx.done may be stale from a previous solve here, so no world is skipped
  _mul_jac_vec(m, d, ctx, d.qacc, ctx.Jaref, wp.zeros(d.nworld, dtype=bool, device=d.qacc.device), subtract_aref=True)


def _get_qfrc_smooth_eff(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext) -> wp.array2d[float]:
  is_solver_ctx = isinstance(ctx, types.SolverContext)
  qfrc_smooth_eff = ctx.qfrc_smooth_eff if is_solver_ctx else None
  if qfrc_smooth_eff is None:
    qfrc_smooth_eff = wp.zeros((d.nworld, m.nv_pad), dtype=float, device=d.qacc.device)
  wp.launch(
    _add_qfrc_smooth_eff,
    dim=(d.nworld, m.nv),
    inputs=[m.opt.enableflags, m.dof_treeid, d.tree_awake, d.qfrc_smooth, d.efm_c, d.efm_ca],
    outputs=[qfrc_smooth_eff],
  )
  return qfrc_smooth_eff


def init_context(
  m: types.Model,
  d: types.Data,
  ctx: SolverContext | InverseContext,
  grad: bool = True,
  compact: bool = False,
):
  if m.opt.integrator == types.IntegratorType.DISCRETE:
    if isinstance(ctx, types.SolverContext):
      if passive.flex_passive_contact(m) and ctx.efm_con is None:
        ctx.efm_con = derivative.build_efm_contact(m, d)
      if ctx.tw_diag is not None and m.opt.solver == types.SolverType.CG:
        derivative.eff_prec_fold(m, d, tw_diag=ctx.tw_diag, efm_con=ctx.efm_con)
    if grad:
      d = dataclasses.replace(d, qfrc_smooth=_get_qfrc_smooth_eff(m, d, ctx))

  # inverse trace of the active inertia matrix, of the full model in a compacted solve
  if isinstance(ctx, SolverContext):
    mf = m if ctx.compact_m_full is None else ctx.compact_m_full
    df = d if ctx.compact_d_full is None else ctx.compact_d_full
    launch_inertia_scale(mf, df.M, ctx.inertia_scale, df)

  # initialize some efc arrays
  wp.launch(
    _solve_init_efc,
    dim=d.nworld,
    outputs=[d.solver_niter, ctx.search_dot, ctx.done],
  )

  # jaref = d.efc_J @ d.qacc - d.efc_aref
  _init_jaref(m, d, ctx)

  # Ma = M @ qacc
  _mul_m_compact_aware(m, d, ctx, d.efc.Ma, d.qacc, ctx.done)

  _update_constraint(m, d, ctx)

  if grad:
    _update_gradient(m, d, ctx, compact=compact)


@event_scope
def solve(m: types.Model, d: types.Data):
  if m.nv == 0:
    d.solver_niter.fill_(0)
    return

  enable_sleep = bool(m.opt.enableflags & types.EnableBit.SLEEP)
  if enable_sleep:
    island.update_active_dofs(m, d)

  if d.njmax == 0:
    wp.copy(d.qacc, d.qacc_smooth)
    d.qfrc_constraint.zero_()
    support.mul_m(m, d, d.efc.Ma, d.qacc)
    d.solver_niter.fill_(0)
    if enable_sleep and m.ntree > 1 and not (m.opt.disableflags & types.DisableBit.ISLAND):
      island.compute_island_mapping(m, d)
    return

  if enable_sleep and m.opt.integrator != types.IntegratorType.DISCRETE:
    solve_compact(m, d)
    if m.opt.noslip_iterations > 0:
      noslip(m, d)
    if m.ntree > 1 and not (m.opt.disableflags & types.DisableBit.ISLAND):
      island.compute_island_mapping(m, d)
    return

  ctx = create_solver_context(m, d, inplace_cholesky=True)
  _solve(m, d, ctx)
  if m.opt.noslip_iterations > 0:
    noslip(m, d)

  if enable_sleep and m.ntree > 1 and not (m.opt.disableflags & types.DisableBit.ISLAND):
    island.compute_island_mapping(m, d)


def _solve(m: types.Model, d: types.Data, ctx: SolverContext, compact: bool = False):
  """Finds forces that satisfy constraints."""
  if m.opt.integrator == types.IntegratorType.DISCRETE:
    d = dataclasses.replace(d, qfrc_smooth=_get_qfrc_smooth_eff(m, d, ctx))

    # effective-metric scratch for the solver iterations, which may run inside wp.capture_while
    if passive.flex_passive_contact(m):
      ctx.efm_con = derivative.build_efm_contact(m, d)
    if m.nflex and m.nflexvert and ctx.flex_scratch is None:
      ctx.flex_scratch = passive.flex_stretch_scratch(m, d)

  warmstart = not (m.opt.disableflags & types.DisableBit.WARMSTART)
  run_qacc_smooth = not smooth.skip_qacc_smooth(m, d)
  wp.launch(
    _solve_init_dof(warmstart, m.is_sparse or _sparse_compact(ctx), run_qacc_smooth),
    dim=(d.nworld, m.nv),
    inputs=[d.nefc, d.qacc_warmstart, d.qacc_smooth],
    outputs=[d.qacc, d.qfrc_constraint],
  )
  if m.opt.integrator == types.IntegratorType.DISCRETE and (m.opt.enableflags & types.EnableBit.SLEEP):
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_treeid, d.tree_awake],
      outputs=[d.qacc],
    )
  # TODO(team): extend _warmstart_best to non-discrete integrators and compact solve
  if warmstart and run_qacc_smooth and m.opt.integrator == types.IntegratorType.DISCRETE and not compact:
    _warmstart_best(m, d, ctx)

  #  context
  init_context(m, d, ctx, grad=False, compact=compact)
  if ctx.nsolving is not None:
    nsolving = ctx.nsolving
    nsolving.fill_(d.nworld)
  else:
    nsolving = wp.full(shape=(1,), value=d.nworld, dtype=int)
  if run_qacc_smooth:
    wp.launch(
      _solve_skip_empty_efc,
      dim=d.nworld,
      inputs=[d.nefc],
      outputs=[ctx.done, nsolving],
    )
  _update_gradient(m, d, ctx, compact=compact)

  # CG search = -Mgrad
  if m.opt.solver == types.SolverType.CG:
    wp.launch_tiled(
      _solve_init_search_cg_tiled,
      dim=d.nworld,
      inputs=[m.nv, m.opt.tolerance, m.opt.integrator, ctx.inertia_scale, ctx.grad, ctx.grad_dot, ctx.Mgrad],
      outputs=[ctx.search, ctx.search_dot, ctx.prev_grad, ctx.prev_Mgrad, ctx.done, nsolving],
      block_dim=m.block_dim.solve_init_search_cg,
    )
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
      inputs=[d.nefc, d.qfrc_smooth, d.efc.Ma, ctx.grad, ctx.grad_scale],
      outputs=[d.qfrc_constraint],
    )
  if m.opt.integrator == types.IntegratorType.DISCRETE and (m.opt.enableflags & types.EnableBit.SLEEP):
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_treeid, d.tree_awake],
      outputs=[d.qacc],
    )
    wp.launch(
      derivative.zero_sleeping_dofs,
      dim=(d.nworld, m.nv),
      inputs=[m.dof_treeid, d.tree_awake],
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
  M_mulm_entry: wp.array[wp.uint64],
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
    col, madr = support.mul_m_indices(M_mulm_entry[k])
    cj = dof_cdof_in[worldid, col]
    if cj >= 0:
      acc += M_in[worldid, madr] * vec[worldid, cj]
  res[worldid, ci] = acc


def _sparse_compact(ctx: SolverContext | InverseContext) -> bool:
  """Whether this solve is compact over a sparse full model (full J structures exist)."""
  return ctx.compact_d_full is not None and ctx.compact_m_full.is_sparse


def _mul_m_compact_aware(m: types.Model, d: types.Data, ctx: SolverContext | InverseContext, res, vec, skip):
  """M @ vec: full-coordinate sparse walk under compact, support.mul_m natively."""
  if m.opt.integrator == types.IntegratorType.DISCRETE:
    derivative.eff_mul_m(m, d, res, vec, skip=skip, efm_con=ctx.efm_con, flex_scratch=ctx.flex_scratch)
    return
  dfull = ctx.compact_d_full
  if dfull is not None:
    mfull = ctx.compact_m_full
    wp.launch(
      _mul_m_sparse_compact,
      dim=(d.nworld, m.nv),
      inputs=[
        mfull.M_mulm_rowadr,
        mfull.M_mulm_entry,
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
  cMa_in: wp.array2d[float],
  # In:
  qacc_c_in: wp.array2d[float],
  qfrc_constraint_c_in: wp.array2d[float],
  # Data out:
  qacc_out: wp.array2d[float],
  qfrc_constraint_out: wp.array2d[float],
  efc_Ma_out: wp.array2d[float],
):
  worldid, i = wp.tid()
  ci = dof_cdof_in[worldid, i]
  if ci >= 0:
    qacc_out[worldid, i] = qacc_c_in[worldid, ci]
    qfrc_constraint_out[worldid, i] = qfrc_constraint_c_in[worldid, ci]
    efc_Ma_out[worldid, i] = cMa_in[worldid, ci]
  else:
    qacc_out[worldid, i] = 0.0
    qfrc_constraint_out[worldid, i] = 0.0
    efc_Ma_out[worldid, i] = 0.0


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
  opt2 = dataclasses.replace(m.opt, graph_conditional=gc)
  m2 = dataclasses.replace(
    m,
    opt=opt2,
    nv=nvp,
    nv_pad=nvp,
    is_sparse=False,
    M_tile_full=False,  # describes the uncompacted M
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

  sctx = create_solver_context(m2, d2, inplace_cholesky=True, dense_hessian=not m.is_sparse)
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
    inputs=[d.dof_cdof, d.cMa, d.cqacc, d.cqfrc_constraint],
    outputs=[d.qacc, d.qfrc_constraint, d.efc.Ma],
  )


@cache_kernel
def _noslip_minv_jt(is_sparse: bool, sleep: bool):
  IS_SPARSE = is_sparse
  SLEEP = sleep

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    dof_treeid: wp.array[int],
    tree_dofnum: wp.array[int],
    M_rownnz: wp.array[int],
    M_rowadr: wp.array[int],
    M_colind: wp.array[int],
    qLD_block_total: int,
    qLD_block_adr: wp.array[int],
    # Data in:
    nefc_in: wp.array[int],
    qLD_in: wp.array2d[float],
    qLDiagInv_in: wp.array2d[float],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    dof_cdof_in: wp.array2d[int],
    # Out:
    noslip_Y_out: wp.array3d[float],
  ):
    """Y[efcid] = M^-1 J[efcid]'."""
    worldid, efcid = wp.tid()

    if efcid >= nefc_in[worldid]:
      return

    # load row of J, inactive (sleeping) dofs are frozen
    if wp.static(IS_SPARSE):
      for k in range(nv):
        noslip_Y_out[worldid, efcid, k] = 0.0
      rowadr = efc_J_rowadr_in[worldid, efcid]
      for k in range(efc_J_rownnz_in[worldid, efcid]):
        sparseid = rowadr + k
        colind = efc_J_colind_in[worldid, 0, sparseid]
        if wp.static(SLEEP):
          if dof_cdof_in[worldid, colind] < 0:
            continue
        noslip_Y_out[worldid, efcid, colind] = efc_J_in[worldid, 0, sparseid]
    else:
      for k in range(nv):
        if wp.static(SLEEP):
          if dof_cdof_in[worldid, k] < 0:
            noslip_Y_out[worldid, efcid, k] = 0.0
            continue
        noslip_Y_out[worldid, efcid, k] = efc_J_in[worldid, efcid, k]

    # solve per kinematic tree with its block of qLD, skipping trees where the row is zero
    y = noslip_Y_out[worldid, efcid]
    L = qLD_in[worldid]
    start = int(0)
    while start < nv:
      end = start + tree_dofnum[dof_treeid[start]]
      nonzero = bool(False)
      for i in range(start, end):
        if y[i] != 0.0:
          nonzero = True
          break
      if not nonzero:
        start = end
        continue

      factor_adr = qLD_block_adr[start]
      if factor_adr == types.Q_LD_BLOCK_COMPACT:
        # diagonal block: M = D
        for i in range(start, end):
          y[i] = y[i] * qLDiagInv_in[worldid, i]
      elif factor_adr == types.Q_LD_BLOCK_SPARSE:
        # sparse block, as in mj_solveLD: M = L'DL with L in the sparsity of M
        # backward substitution: L' z = J'
        for ii in range(start, end):
          i = start + end - 1 - ii
          yi = y[i]
          if yi != 0.0:
            rowadr = M_rowadr[i]
            for k in range(M_rownnz[i] - 1):
              colind = M_colind[rowadr + k]
              y[colind] = y[colind] - L[qLD_block_total + rowadr + k] * yi
        # forward substitution: L y = D^-1 z
        for i in range(start, end):
          rowadr = M_rowadr[i]
          val = y[i] * qLDiagInv_in[worldid, i]
          for k in range(M_rownnz[i] - 1):
            val -= L[qLD_block_total + rowadr + k] * y[M_colind[rowadr + k]]
          y[i] = val
      else:
        # dense block: M = L L' with L' row major at factor_adr
        size = end - start
        # forward substitution: L z = J'
        for i in range(size):
          val = y[start + i]
          for k in range(i):
            val -= L[factor_adr + k * size + i] * y[start + k]
          y[start + i] = val / L[factor_adr + i * size + i]
        # backward substitution: L' y = z
        for ii in range(size):
          i = size - 1 - ii
          val = y[start + i]
          for k in range(i + 1, size):
            val -= L[factor_adr + i * size + k] * y[start + k]
          y[start + i] = val / L[factor_adr + i * size + i]
      start = end

  return kernel


@wp.func
def _noslip_dot(
  # Model:
  nv: int,
  is_sparse: bool,
  # Data in:
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  # In:
  worldid: int,
  efcid: int,
  vec_in: wp.array[float],
) -> float:
  """Returns J[efcid] * vec."""
  res = float(0.0)
  if is_sparse:
    rowadr = efc_J_rowadr_in[worldid, efcid]
    for k in range(efc_J_rownnz_in[worldid, efcid]):
      sparseid = rowadr + k
      res += efc_J_in[worldid, 0, sparseid] * vec_in[efc_J_colind_in[worldid, 0, sparseid]]
  else:
    for k in range(nv):
      res += efc_J_in[worldid, efcid, k] * vec_in[k]
  return res


@wp.func
def _noslip_add(nv: int, vec_in: wp.array[float], scale: float, vec_out: wp.array[float]):
  """Adds scale * vec_in to vec_out."""
  if scale != 0.0:
    for k in range(nv):
      vec_out[k] += scale * vec_in[k]


@wp.func
def _qcqp2(Ain: types.mat55, bin: types.vec5, d: types.vec5, r: float):
  """Solves min 0.5 x' A x + x' b s.t.

  sum (x_i / d_i)^2 <= r^2 in 2 dimensions (mju_QCQP2).
  """
  # scale A, b so that constraint becomes x' x <= r^2
  b1 = bin[0] * d[0]
  b2 = bin[1] * d[1]
  A11 = Ain[0, 0] * d[0] * d[0]
  A22 = Ain[1, 1] * d[1] * d[1]
  A12 = Ain[0, 1] * d[0] * d[1]

  # Newton iteration
  la = float(0.0)
  v1 = float(0.0)
  v2 = float(0.0)
  for _ in range(20):
    # det(A + la)
    det = (A11 + la) * (A22 + la) - A12 * A12

    # check SPD, with 1e-10 threshold
    if det < 1.0e-10:
      return types.vec5(0.0), False

    # P = inv(A + la)
    detinv = 1.0 / det
    P11 = (A22 + la) * detinv
    P22 = (A11 + la) * detinv
    P12 = -A12 * detinv

    # v = -P b
    v1 = -P11 * b1 - P12 * b2
    v2 = -P12 * b1 - P22 * b2

    # check for convergence, or initial solution inside constraint set
    val = v1 * v1 + v2 * v2 - r * r
    if val < 1.0e-10:
      break

    # deriv = -2 v' P v
    deriv = -2.0 * (P11 * v1 * v1 + 2.0 * P12 * v1 * v2 + P22 * v2 * v2)

    # compute update, exit if too small
    delta = -val / deriv
    if delta < 1.0e-10:
      break

    la += delta

  # undo scaling
  return types.vec5(v1 * d[0], v2 * d[1], 0.0, 0.0, 0.0), la != 0.0


@wp.func
def _qcqp3(Ain: types.mat55, bin: types.vec5, d: types.vec5, r: float):
  """Solves min 0.5 x' A x + x' b s.t.

  sum (x_i / d_i)^2 <= r^2 in 3 dimensions (mju_QCQP3).
  """
  # scale A, b so that constraint becomes x' x <= r^2
  b1 = bin[0] * d[0]
  b2 = bin[1] * d[1]
  b3 = bin[2] * d[2]
  A11 = Ain[0, 0] * d[0] * d[0]
  A22 = Ain[1, 1] * d[1] * d[1]
  A33 = Ain[2, 2] * d[2] * d[2]
  A12 = Ain[0, 1] * d[0] * d[1]
  A13 = Ain[0, 2] * d[0] * d[2]
  A23 = Ain[1, 2] * d[1] * d[2]

  # Newton iteration
  la = float(0.0)
  v1 = float(0.0)
  v2 = float(0.0)
  v3 = float(0.0)
  for _ in range(20):
    # unscaled P
    P11 = (A22 + la) * (A33 + la) - A23 * A23
    P22 = (A11 + la) * (A33 + la) - A13 * A13
    P33 = (A11 + la) * (A22 + la) - A12 * A12
    P12 = A13 * A23 - A12 * (A33 + la)
    P13 = A12 * A23 - A13 * (A22 + la)
    P23 = A12 * A13 - A23 * (A11 + la)

    # det(A + la)
    det = (A11 + la) * P11 + A12 * P12 + A13 * P13

    # check SPD, with 1e-10 threshold
    if det < 1.0e-10:
      return types.vec5(0.0), False

    # final P
    detinv = 1.0 / det
    P11 *= detinv
    P22 *= detinv
    P33 *= detinv
    P12 *= detinv
    P13 *= detinv
    P23 *= detinv

    # v = -P b
    v1 = -P11 * b1 - P12 * b2 - P13 * b3
    v2 = -P12 * b1 - P22 * b2 - P23 * b3
    v3 = -P13 * b1 - P23 * b2 - P33 * b3

    # check for convergence, or initial solution inside constraint set
    val = v1 * v1 + v2 * v2 + v3 * v3 - r * r
    if val < 1.0e-10:
      break

    # deriv = -2 v' P v
    deriv = -2.0 * (P11 * v1 * v1 + P22 * v2 * v2 + P33 * v3 * v3) - 4.0 * (P12 * v1 * v2 + P13 * v1 * v3 + P23 * v2 * v3)

    # compute update, exit if too small
    delta = -val / deriv
    if delta < 1.0e-10:
      break

    la += delta

  # undo scaling
  return types.vec5(v1 * d[0], v2 * d[1], v3 * d[2], 0.0, 0.0), la != 0.0


@wp.func
def _chol_solve5(L: types.mat55, b: types.vec5) -> types.vec5:
  """Solves L L' x = b in 5 dimensions (mju_cholSolve)."""
  # constant loop bounds let Warp unroll the loops and keep L and x in registers
  x = types.vec5(0.0)
  for i in range(5):
    val = b[i]
    for k in range(i):
      val -= L[i, k] * x[k]
    x[i] = val / L[i, i]
  for i in range(4, -1, -1):
    val = x[i]
    for k in range(4, i, -1):
      val -= L[k, i] * x[k]
    x[i] = val / L[i, i]
  return x


@wp.func
def _qcqp5(Ain: types.mat55, bin: types.vec5, d: types.vec5, r: float):
  """Solves min 0.5 x' A x + x' b s.t.

  sum (x_i / d_i)^2 <= r^2 in 5 dimensions (mju_QCQP).
  """
  # constant loop bounds let Warp unroll the loops and keep A and Ala in registers

  # scale A, b so that constraint becomes x' x <= r^2
  b = wp.cw_mul(bin, d)
  A = types.mat55()
  for i in range(5):
    for j in range(5):
      A[i, j] = Ain[i, j] * d[i] * d[j]

  # Newton iteration
  res = types.vec5(0.0)
  la = float(0.0)
  for _ in range(20):
    # make A + la
    Ala = A
    for i in range(5):
      Ala[i, i] = A[i, i] + la

    # factorize, check rank with 1e-10 threshold (mju_cholFactor)
    for j in range(5):
      diag = Ala[j, j]
      for k in range(j):
        diag -= Ala[j, k] * Ala[j, k]
      if diag < 1.0e-10:
        return types.vec5(0.0), False
      diag = wp.sqrt(diag)
      Ala[j, j] = diag
      for i in range(4, j, -1):
        val = Ala[i, j]
        for k in range(j):
          val -= Ala[i, k] * Ala[j, k]
        Ala[i, j] = val / diag

    # res = -Ala \ b
    res = -_chol_solve5(Ala, b)

    # val = b' Ala^-2 b - r^2
    val = wp.dot(res, res) - r * r

    # check for convergence, or initial solution inside constraint set
    if val < 1.0e-10:
      break

    # deriv = -2 b' Ala^-3 b
    tmp = _chol_solve5(Ala, res)
    deriv = -2.0 * wp.dot(res, tmp)

    # compute update, exit if too small
    delta = -val / deriv
    if delta < 1.0e-10:
      break

    la += delta

  # undo scaling
  return wp.cw_mul(res, d), la != 0.0


# rounding a new force to float32 moves the cost by ~eps * |force * gradient|
_NOSLIP_NOISE_EPS = 8.0 * 1.1920929e-07  # 8 * float32 eps


@wp.func
def _noslip_pyramidal(
  # Model:
  nv: int,
  is_sparse: bool,
  # Data in:
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  efc_aref_in: wp.array2d[float],
  # In:
  sleep: bool,
  worldid: int,
  efcid: int,
  noslip_Y_in: wp.array2d[float],
  noslip_A_in: wp.array2d[types.vec5],
  # Data out:
  qacc_out: wp.array2d[float],
  efc_force_out: wp.array2d[float],
) -> float:
  """Updates a pair of opposing pyramid edges, returns the cost change."""
  efcid1 = efcid + 1

  # A submatrix
  A0 = noslip_A_in[worldid, efcid]
  A00 = A0[0]
  A01 = A0[1]
  A11 = noslip_A_in[worldid, efcid1][0]

  # rows without active (awake) dofs
  if sleep and A00 == 0.0 and A11 == 0.0:
    return 0.0

  # clamp diagonal
  A00 = wp.max(1.0e-10, A00)
  A11 = wp.max(1.0e-10, A11)

  # residual
  qacc = qacc_out[worldid]
  res0 = (
    _noslip_dot(nv, is_sparse, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, qacc)
    - efc_aref_in[worldid, efcid]
  )
  res1 = (
    _noslip_dot(nv, is_sparse, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid1, qacc)
    - efc_aref_in[worldid, efcid1]
  )

  oldforce0 = efc_force_out[worldid, efcid]
  oldforce1 = efc_force_out[worldid, efcid1]

  # b subvector + A_rest f_rest
  bc0 = res0 - (A00 * oldforce0 + A01 * oldforce1)
  bc1 = res1 - (A01 * oldforce0 + A11 * oldforce1)

  # f0 = mid + y, f1 = mid - y
  mid = 0.5 * (oldforce0 + oldforce1)

  # K1 = A00 + A11 - 2 A01, K0 = mid (A00 - A11) + b0 - b1
  K1 = A00 + A11 - A01 - A01
  K0 = mid * (A00 - A11) + bc0 - bc1

  force0 = float(mid)
  force1 = float(mid)

  # guard against A == 0, otherwise minimize over y in [-mid, mid]
  if K1 >= types.MJ_MINVAL:
    y = -K0 / K1
    if y < -mid:
      force0 = 0.0
      force1 = 2.0 * mid
    elif y > mid:
      force0 = 2.0 * mid
      force1 = 0.0
    else:
      force0 = mid + y
      force1 = mid - y

  # cost change, restore force if positive
  delta0 = force0 - oldforce0
  delta1 = force1 - oldforce1
  Adelta0 = A00 * delta0 + A01 * delta1
  Adelta1 = A01 * delta0 + A11 * delta1
  change = 0.5 * (delta0 * Adelta0 + delta1 * Adelta1)
  change += delta0 * res0 + delta1 * res1
  noise = wp.abs(force0 * (res0 + Adelta0)) + wp.abs(force1 * (res1 + Adelta1))
  if change > 1.0e-10 + _NOSLIP_NOISE_EPS * noise:
    return 0.0

  efc_force_out[worldid, efcid] = force0
  efc_force_out[worldid, efcid1] = force1
  if delta0 != 0.0 or delta1 != 0.0:
    Y0 = noslip_Y_in[efcid]
    Y1 = noslip_Y_in[efcid1]
    for k in range(nv):
      qacc[k] += delta0 * Y0[k] + delta1 * Y1[k]

  return change


@wp.func
def _noslip_elliptic(
  # Model:
  nv: int,
  is_sparse: bool,
  # Data in:
  efc_J_rownnz_in: wp.array2d[int],
  efc_J_rowadr_in: wp.array2d[int],
  efc_J_colind_in: wp.array3d[int],
  efc_J_in: wp.array3d[float],
  efc_aref_in: wp.array2d[float],
  # In:
  sleep: bool,
  worldid: int,
  efcid: int,
  dim: int,
  friction: types.vec5,
  noslip_Y_in: wp.array2d[float],
  noslip_A_in: wp.array2d[types.vec5],
  # Data out:
  qacc_out: wp.array2d[float],
  efc_force_out: wp.array2d[float],
) -> float:
  """Updates the frictional rows of an elliptic contact, returns the cost change."""
  n = dim - 1

  # A submatrix (upper triangle is precomputed)
  Ac = types.mat55()
  Adiag = float(0.0)
  for k in range(n):
    Ak = noslip_A_in[worldid, efcid + 1 + k]
    Adiag += Ak[k]
    for l in range(k, n):
      Ac[k, l] = Ak[l]
      Ac[l, k] = Ak[l]

  # rows without active (awake) dofs
  if sleep and Adiag == 0.0:
    return 0.0

  # residual, old force, clamp diagonal
  qacc = qacc_out[worldid]
  res = types.vec5(0.0)
  oldforce = types.vec5(0.0)
  for k in range(n):
    rowk = efcid + 1 + k
    res[k] = (
      _noslip_dot(nv, is_sparse, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, rowk, qacc)
      - efc_aref_in[worldid, rowk]
    )
    oldforce[k] = efc_force_out[worldid, rowk]
    Ac[k, k] = wp.max(1.0e-10, Ac[k, k])

  # b subvector + A_rest f_rest
  bc = types.vec5(0.0)
  for k in range(n):
    val = res[k]
    for l in range(n):
      val -= Ac[k, l] * oldforce[l]
    bc[k] = val

  # guard for normal force == 0, otherwise QCQP
  normal = efc_force_out[worldid, efcid]
  force = types.vec5(0.0)
  if normal >= types.MJ_MINVAL:
    active = bool(False)
    if n == 2:
      force, active = _qcqp2(Ac, bc, friction, normal)
    elif n == 3:
      force, active = _qcqp3(Ac, bc, friction, normal)
    else:
      force, active = _qcqp5(Ac, bc, friction, normal)

    # on constraint: put force on ellipsoid, in case QCQP is approximate
    if active:
      s = float(0.0)
      for k in range(n):
        fk = math.safe_div(force[k], friction[k])
        s += fk * fk
      scl = wp.sqrt(normal * normal / wp.max(types.MJ_MINVAL, s))
      for k in range(n):
        force[k] = force[k] * scl

  # cost change, restore force if positive
  delta = types.vec5(0.0)
  for k in range(n):
    delta[k] = force[k] - oldforce[k]
  change = float(0.0)
  noise = float(0.0)
  for k in range(n):
    Adelta = float(0.0)
    for l in range(n):
      Adelta += Ac[k, l] * delta[l]
    change += 0.5 * delta[k] * Adelta + delta[k] * res[k]
    noise += wp.abs(force[k] * (res[k] + Adelta))
  if change > 1.0e-10 + _NOSLIP_NOISE_EPS * noise:
    return 0.0

  for k in range(n):
    efc_force_out[worldid, efcid + 1 + k] = force[k]
  for i in range(nv):
    dqacc = float(0.0)
    for k in range(n):
      dqacc += delta[k] * noslip_Y_in[efcid + 1 + k, i]
    qacc[i] += dqacc

  return change


@cache_kernel
def _noslip_delassus(is_sparse: bool):
  IS_SPARSE = is_sparse

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    # Data in:
    nefc_in: wp.array[int],
    contact_dim_in: wp.array[int],
    contact_efc_address_in: wp.array2d[int],
    efc_type_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    # In:
    noslip_Y_in: wp.array3d[float],
    # Out:
    noslip_A_out: wp.array2d[types.vec5],
  ):
    """Diagonal blocks of A = J M^-1 J' for the frictional rows.

    Dry friction: A[efcid][0] = A_ii. Pyramidal: A[efcid][0] = A_ii and, for the first edge of a
    pair of opposing edges, A[efcid][1] = A_i,i+1. Elliptic: for frictional row k of a contact,
    A[efcid][l] = A_kl for l >= k.
    """
    worldid, efcid = wp.tid()
    nefc = wp.min(nefc_in[worldid], noslip_Y_in.shape[1])

    if efcid >= nefc:
      return

    efc_type = efc_type_in[worldid, efcid]
    Y = noslip_Y_in[worldid]
    A = types.vec5(0.0)

    if efc_type == int(types.ConstraintType.FRICTION_DOF) or efc_type == int(types.ConstraintType.FRICTION_TENDON):
      A[0] = _noslip_dot(nv, IS_SPARSE, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, Y[efcid])
    elif efc_type == int(types.ConstraintType.CONTACT_PYRAMIDAL):
      A[0] = _noslip_dot(nv, IS_SPARSE, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, Y[efcid])
      conid = efc_id_in[worldid, efcid]
      if (efcid - contact_efc_address_in[conid, 0]) % 2 == 0 and efcid + 1 < nefc:
        Y1 = Y[efcid + 1]
        A[1] = _noslip_dot(nv, IS_SPARSE, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, Y1)
    elif efc_type == int(types.ConstraintType.CONTACT_ELLIPTIC):
      conid = efc_id_in[worldid, efcid]
      adr = contact_efc_address_in[conid, 0]
      n = contact_dim_in[conid] - 1
      k = efcid - adr - 1

      # skip the normal row and contacts that do not fit
      if k < 0 or adr + n >= nefc:
        return

      for l in range(k, n):
        Yl = Y[adr + 1 + l]
        A[l] = _noslip_dot(nv, IS_SPARSE, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, Yl)
    else:
      return

    noslip_A_out[worldid, efcid] = A

  return kernel


@cache_kernel
def _noslip_qacc_init(sleep: bool):
  SLEEP = sleep

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    qacc_smooth_in: wp.array2d[float],
    efc_force_in: wp.array2d[float],
    dof_cdof_in: wp.array2d[int],
    njmax_in: int,
    # In:
    noslip_Y_in: wp.array3d[float],
    # Data out:
    qacc_out: wp.array2d[float],
  ):
    """Computes qacc = qacc_smooth + M^-1 J' force, the residual is A force + b = J qacc - aref."""
    worldid, dofid = wp.tid()
    nefc = wp.min(nefc_in[worldid], njmax_in)

    if nefc == 0:
      return

    # inactive (sleeping) dofs are frozen
    if wp.static(SLEEP):
      if dof_cdof_in[worldid, dofid] < 0:
        qacc_out[worldid, dofid] = 0.0
        return

    qacc = qacc_smooth_in[worldid, dofid]
    for efcid in range(nefc):
      qacc += noslip_Y_in[worldid, efcid, dofid] * efc_force_in[worldid, efcid]
    qacc_out[worldid, dofid] = qacc

  return kernel


@cache_kernel
def _noslip_solve(is_sparse: bool, sleep: bool):
  IS_SPARSE = is_sparse
  SLEEP = sleep

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nv: int,
    opt_noslip_tolerance: wp.array[float],
    opt_noslip_iterations: int,
    # Data in:
    nefc_in: wp.array[int],
    contact_friction_in: wp.array[types.vec5],
    contact_dim_in: wp.array[int],
    efc_type_in: wp.array2d[int],
    efc_id_in: wp.array2d[int],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_D_in: wp.array2d[float],
    efc_aref_in: wp.array2d[float],
    efc_frictionloss_in: wp.array2d[float],
    njmax_in: int,
    # In:
    noslip_Y_in: wp.array3d[float],
    noslip_A_in: wp.array2d[types.vec5],
    inertia_scale_in: wp.array[float],
    # Data out:
    solver_niter_out: wp.array[int],
    qacc_out: wp.array2d[float],
    efc_force_out: wp.array2d[float],
  ):
    """Projected Gauss-Seidel sweeps over the frictional constraint rows."""
    worldid = wp.tid()
    nefc = wp.min(nefc_in[worldid], njmax_in)

    if nefc == 0:
      return

    noslip_Y = noslip_Y_in[worldid]
    qacc = qacc_out[worldid]
    efc_force = efc_force_out[worldid]

    scale = inertia_scale_in[worldid]
    tolerance = opt_noslip_tolerance[worldid % opt_noslip_tolerance.shape[0]]

    # main iteration
    niter = int(0)
    while niter < opt_noslip_iterations:
      improvement = float(0.0)

      # correct for cost change at iter 0
      if niter == 0:
        for efcid in range(nefc):
          efc_D = efc_D_in[worldid, efcid]
          if efc_D > 0.0:
            force = efc_force[efcid]
            improvement += 0.5 * force * force / efc_D

      # perform one sweep: dry friction
      for efcid in range(nefc):
        efc_type = efc_type_in[worldid, efcid]
        if efc_type != int(types.ConstraintType.FRICTION_DOF) and efc_type != int(types.ConstraintType.FRICTION_TENDON):
          continue

        # diagonal of A
        Adiag = noslip_A_in[worldid, efcid][0]

        # rows without active (awake) dofs
        if wp.static(SLEEP):
          if Adiag == 0.0:
            continue

        # residual
        res = _noslip_dot(nv, IS_SPARSE, efc_J_rownnz_in, efc_J_rowadr_in, efc_J_colind_in, efc_J_in, worldid, efcid, qacc)
        res -= efc_aref_in[worldid, efcid]

        Adiag = wp.max(types.MJ_MINVAL, Adiag)

        # unconstrained minimum, impose interval constraints
        oldforce = efc_force[efcid]
        frictionloss = efc_frictionloss_in[worldid, efcid]
        force = wp.clamp(oldforce - res / Adiag, -frictionloss, frictionloss)

        # add to improvement
        delta = force - oldforce
        improvement -= 0.5 * delta * delta * Adiag + delta * res

        efc_force[efcid] = force
        _noslip_add(nv, noslip_Y[efcid], delta, qacc)

      # perform one sweep: contact friction
      rowid = int(0)
      while rowid < nefc:
        efc_type = efc_type_in[worldid, rowid]
        if efc_type == int(types.ConstraintType.CONTACT_PYRAMIDAL):
          conid = efc_id_in[worldid, rowid]
          nrow = 2 * (contact_dim_in[conid] - 1)
          if rowid + nrow > nefc:
            break

          # loop over pairs of opposing pyramid edges
          for j in range(rowid, rowid + nrow, 2):
            improvement -= _noslip_pyramidal(
              nv,
              IS_SPARSE,
              efc_J_rownnz_in,
              efc_J_rowadr_in,
              efc_J_colind_in,
              efc_J_in,
              efc_aref_in,
              SLEEP,
              worldid,
              j,
              noslip_Y,
              noslip_A_in,
              qacc_out,
              efc_force_out,
            )
          rowid += nrow
        elif efc_type == int(types.ConstraintType.CONTACT_ELLIPTIC):
          conid = efc_id_in[worldid, rowid]
          dim = contact_dim_in[conid]
          if rowid + dim > nefc:
            break

          improvement -= _noslip_elliptic(
            nv,
            IS_SPARSE,
            efc_J_rownnz_in,
            efc_J_rowadr_in,
            efc_J_colind_in,
            efc_J_in,
            efc_aref_in,
            SLEEP,
            worldid,
            rowid,
            dim,
            contact_friction_in[conid],
            noslip_Y,
            noslip_A_in,
            qacc_out,
            efc_force_out,
          )
          rowid += dim
        else:
          rowid += 1

      # scale improvement, increment iteration count
      improvement *= scale
      niter += 1

      # terminate
      if improvement < tolerance:
        break

    solver_niter_out[worldid] += niter

  return kernel


@cache_kernel
def _noslip_qacc_qfrc(is_sparse: bool, sleep: bool):
  IS_SPARSE = is_sparse
  SLEEP = sleep

  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Data in:
    nefc_in: wp.array[int],
    qacc_smooth_in: wp.array2d[float],
    efc_J_rownnz_in: wp.array2d[int],
    efc_J_rowadr_in: wp.array2d[int],
    efc_J_colind_in: wp.array3d[int],
    efc_J_in: wp.array3d[float],
    efc_force_in: wp.array2d[float],
    dof_cdof_in: wp.array2d[int],
    njmax_in: int,
    # In:
    noslip_Y_in: wp.array3d[float],
    # Data out:
    qacc_out: wp.array2d[float],
    qfrc_constraint_out: wp.array2d[float],
  ):
    """Computes qacc = qacc_smooth + M^-1 J' force and qfrc_constraint = J' force."""
    worldid, dofid = wp.tid()
    nefc = wp.min(nefc_in[worldid], njmax_in)

    if nefc == 0:
      return

    # inactive (sleeping) dofs are frozen
    if wp.static(SLEEP):
      if dof_cdof_in[worldid, dofid] < 0:
        qacc_out[worldid, dofid] = 0.0
        qfrc_constraint_out[worldid, dofid] = 0.0
        return

    qacc = qacc_smooth_in[worldid, dofid]
    qfrc = float(0.0)
    for efcid in range(nefc):
      force = efc_force_in[worldid, efcid]
      if force == 0.0:
        continue
      qacc += noslip_Y_in[worldid, efcid, dofid] * force
      if wp.static(IS_SPARSE):
        rowadr = efc_J_rowadr_in[worldid, efcid]
        for k in range(efc_J_rownnz_in[worldid, efcid]):
          sparseid = rowadr + k
          if efc_J_colind_in[worldid, 0, sparseid] == dofid:
            qfrc += efc_J_in[worldid, 0, sparseid] * force
            break
      else:
        qfrc += efc_J_in[worldid, efcid, dofid] * force

    qacc_out[worldid, dofid] = qacc
    qfrc_constraint_out[worldid, dofid] = qfrc

  return kernel


@wp.kernel
def _noslip_state(
  # Model:
  opt_impratio_invsqrt: wp.array[float],
  # Data in:
  nefc_in: wp.array[int],
  contact_friction_in: wp.array[types.vec5],
  contact_dim_in: wp.array[int],
  contact_efc_address_in: wp.array2d[int],
  efc_type_in: wp.array2d[int],
  efc_id_in: wp.array2d[int],
  efc_frictionloss_in: wp.array2d[float],
  efc_force_in: wp.array2d[float],
  njmax_in: int,
  # Data out:
  efc_state_out: wp.array2d[int],
):
  """Dual constraint state."""
  worldid, efcid = wp.tid()
  nefc = wp.min(nefc_in[worldid], njmax_in)

  if efcid >= nefc:
    return

  efc_type = efc_type_in[worldid, efcid]
  force = efc_force_in[worldid, efcid]

  state = int(types.ConstraintState.QUADRATIC)
  if efc_type == int(types.ConstraintType.EQUALITY):
    state = int(types.ConstraintState.QUADRATIC)
  elif efc_type == int(types.ConstraintType.FRICTION_DOF) or efc_type == int(types.ConstraintType.FRICTION_TENDON):
    frictionloss = efc_frictionloss_in[worldid, efcid]
    if force <= -frictionloss:
      state = int(types.ConstraintState.LINEARPOS)
    elif force >= frictionloss:
      state = int(types.ConstraintState.LINEARNEG)
  elif efc_type == int(types.ConstraintType.CONTACT_ELLIPTIC):
    conid = efc_id_in[worldid, efcid]
    adr = contact_efc_address_in[conid, 0]
    dim = contact_dim_in[conid]
    friction = contact_friction_in[conid]
    mu = friction[0] * opt_impratio_invsqrt[worldid % opt_impratio_invsqrt.shape[0]]

    # map force to regular-cone space, N = normal, T = norm of tangent vector
    N = math.safe_div(efc_force_in[worldid, adr], mu)
    T = float(0.0)
    for j in range(1, dim):
      if adr + j < nefc:
        fj = math.safe_div(efc_force_in[worldid, adr + j], friction[j - 1])
        T += fj * fj
    T = wp.sqrt(T)

    state = int(types.ConstraintState.CONE)
    if mu * N >= T:
      state = int(types.ConstraintState.SATISFIED)
    elif N + mu * T <= 0.0:
      state = int(types.ConstraintState.QUADRATIC)
  elif force <= 0.0:
    state = int(types.ConstraintState.SATISFIED)

  efc_state_out[worldid, efcid] = state


@event_scope
def noslip(m: types.Model, d: types.Data):
  """Noslip solver: projected Gauss-Seidel over the frictional constraints.

  Post-processes the main solver result, keeping the non-frictional constraint forces fixed
  and re-solving the dof/tendon friction loss and contact friction forces with the
  unregularized Delassus operator J M^-1 J'. Updates efc.force, efc.state, qacc,
  qfrc_constraint, and efc.Ma. With sleeping enabled, inactive dofs stay frozen and
  constraint rows without active dofs are left unchanged.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
  """
  if m.opt.integrator == types.IntegratorType.DISCRETE:
    raise NotImplementedError("noslip solver not implemented for the discrete integrator.")

  if d.njmax == 0 or m.nv == 0:
    return

  noslip_Y = wp.empty((d.nworld, d.njmax, m.nv), dtype=float)
  noslip_A = wp.empty((d.nworld, d.njmax), dtype=types.vec5)
  inertia_scale = wp.empty((d.nworld,), dtype=float)

  sleep = bool(m.opt.enableflags & types.EnableBit.SLEEP)

  launch_inertia_scale(m, d.M, inertia_scale, d)
  wp.launch(
    _noslip_minv_jt(m.is_sparse, sleep),
    dim=(d.nworld, d.njmax),
    inputs=[
      m.nv,
      m.dof_treeid,
      m.tree_dofnum,
      m.M_rownnz,
      m.M_rowadr,
      m.M_colind,
      m.qLD_block_total,
      m.qLD_block_adr,
      d.nefc,
      d.qLD,
      d.qLDiagInv,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.J,
      d.dof_cdof,
    ],
    outputs=[noslip_Y],
  )
  wp.launch(
    _noslip_delassus(m.is_sparse),
    dim=(d.nworld, d.njmax),
    inputs=[
      m.nv,
      d.nefc,
      d.contact.dim,
      d.contact.efc_address,
      d.efc.type,
      d.efc.id,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.J,
      noslip_Y,
    ],
    outputs=[noslip_A],
  )
  wp.launch(
    _noslip_qacc_init(sleep),
    dim=(d.nworld, m.nv),
    inputs=[d.nefc, d.qacc_smooth, d.efc.force, d.dof_cdof, d.njmax, noslip_Y],
    outputs=[d.qacc],
  )
  wp.launch(
    _noslip_solve(m.is_sparse, sleep),
    dim=d.nworld,
    inputs=[
      m.nv,
      m.opt.noslip_tolerance,
      m.opt.noslip_iterations,
      d.nefc,
      d.contact.friction,
      d.contact.dim,
      d.efc.type,
      d.efc.id,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.J,
      d.efc.D,
      d.efc.aref,
      d.efc.frictionloss,
      d.njmax,
      noslip_Y,
      noslip_A,
      inertia_scale,
    ],
    outputs=[d.solver_niter, d.qacc, d.efc.force],
    block_dim=m.block_dim.noslip_solve,
  )
  wp.launch(
    _noslip_qacc_qfrc(m.is_sparse, sleep),
    dim=(d.nworld, m.nv),
    inputs=[
      d.nefc,
      d.qacc_smooth,
      d.efc.J_rownnz,
      d.efc.J_rowadr,
      d.efc.J_colind,
      d.efc.J,
      d.efc.force,
      d.dof_cdof,
      d.njmax,
      noslip_Y,
    ],
    outputs=[d.qacc, d.qfrc_constraint],
  )
  wp.launch(
    _noslip_state,
    dim=(d.nworld, d.njmax),
    inputs=[
      m.opt.impratio_invsqrt,
      d.nefc,
      d.contact.friction,
      d.contact.dim,
      d.contact.efc_address,
      d.efc.type,
      d.efc.id,
      d.efc.frictionloss,
      d.efc.force,
      d.njmax,
    ],
    outputs=[d.efc.state],
  )

  # efc.Ma = M @ qacc is the right-hand side for the integrators
  support.mul_m(m, d, d.efc.Ma, d.qacc)
