# Copyright 2026 The Newton Developers
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
"""Flex collision detection (geom vs flex triangles)."""

import dataclasses
import math

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import collision_primitive_core
from mujoco.mjx.third_party.mujoco_warp._src import sort
from mujoco.mjx.third_party.mujoco_warp._src import support
from mujoco.mjx.third_party.mujoco_warp._src.collision_core import Geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import ccd
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import support as gjk_support
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.math import orthogonals
from mujoco.mjx.third_party.mujoco_warp._src.math import quat_to_mat
from mujoco.mjx.third_party.mujoco_warp._src.math import safe_div
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAX_EPAFACES
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAX_EPAHORIZON
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXCONPAIR
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINMU
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import ContactType
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import DeterminismType
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import OverflowType
from mujoco.mjx.third_party.mujoco_warp._src.types import mat23
from mujoco.mjx.third_party.mujoco_warp._src.types import mat63
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False, "default_grid_stride": False})

_FPS_SLOTS: int = 8  # candidates per thread kept in registers during FPS
ENABLE_SAT_PREFILTER: bool = True
# traversal stack for the flex BVHs, which MuJoCo builds balanced (depth ~ log2(nelem))
_FLEX_BVH_STACK_SIZE: int = 32
_FLEX_BVH_QUERY_THREADS: int = 1 << 16
_FLEX_BVH_MAX_SPLIT_DEPTH: int = 4
_FLEX_CONTACT_DEDUP_TOLERANCE: float = 1e-3

_vec_bvh_stack = wp.types.vector(length=_FLEX_BVH_STACK_SIZE, dtype=int)


@wp.func
def _cmp_vec2i(a: wp.vec2i, b: wp.vec2i) -> int:
  """Lexicographic comparison of two vec2i values (-1, 0, or 1)."""
  for k in range(2):
    if a[k] != b[k]:
      return -1 if a[k] < b[k] else 1
  return 0


@wp.func
def _cmp_vec3f(a: wp.vec3, b: wp.vec3) -> int:
  """Lexicographic comparison of two vec3 values using total float bit order (-1, 0, or 1)."""
  for k in range(3):
    ka = sort.float_key(wp.cast(a[k], wp.uint32))
    kb = sort.float_key(wp.cast(b[k], wp.uint32))
    if ka != kb:
      return -1 if ka < kb else 1
  return 0


@wp.func
def _flex_element_aabb_filter(
  # In:
  box1_min: wp.vec3,
  box1_max: wp.vec3,
  box2_min: wp.vec3,
  box2_max: wp.vec3,
):
  """Return True if the two AABBs do NOT intersect (discard pair)."""
  if box1_max[0] < box2_min[0] or box1_min[0] > box2_max[0]:
    return True
  if box1_max[1] < box2_min[1] or box1_min[1] > box2_max[1]:
    return True
  if box1_max[2] < box2_min[2] or box1_min[2] > box2_max[2]:
    return True
  return False


@wp.func
def _axis_separated(
  # In:
  p0: wp.vec3,
  p1: wp.vec3,
  p2: wp.vec3,
  q0: wp.vec3,
  q1: wp.vec3,
  q2: wp.vec3,
  ax: wp.vec3,
  cutoff_sq: float,
) -> bool:
  min1 = wp.min(wp.dot(p0, ax), wp.min(wp.dot(p1, ax), wp.dot(p2, ax)))
  max1 = wp.max(wp.dot(p0, ax), wp.max(wp.dot(p1, ax), wp.dot(p2, ax)))
  min2 = wp.min(wp.dot(q0, ax), wp.min(wp.dot(q1, ax), wp.dot(q2, ax)))
  max2 = wp.max(wp.dot(q0, ax), wp.max(wp.dot(q1, ax), wp.dot(q2, ax)))
  diff = wp.max(min1 - max2, min2 - max1)
  if diff > 0.0:
    lax_sq = wp.length_sq(ax)
    if lax_sq > 1e-12 and diff * diff > cutoff_sq * lax_sq:
      return True
  return False


@wp.func
def _point_segment_axis(p: wp.vec3, a: wp.vec3, b: wp.vec3) -> wp.vec3:
  ab = b - a
  ab_len_sq = wp.length_sq(ab)
  if ab_len_sq > 1e-12:
    ap = p - a
    t = wp.clamp(wp.dot(ap, ab) / ab_len_sq, 0.0, 1.0)
    return ap - t * ab
  return p - a


@wp.func
def _triangle_sat_separated(
  # In:
  p0: wp.vec3,
  p1: wp.vec3,
  p2: wp.vec3,
  q0: wp.vec3,
  q1: wp.vec3,
  q2: wp.vec3,
  cutoff_sq: float,
) -> bool:
  """Returns True if two triangles inflated by cutoff are separated (no contact possible)."""
  e1_0 = p1 - p0
  e1_1 = p2 - p1
  e1_2 = p0 - p2

  # Face normal of triangle 1
  n1 = wp.cross(e1_0, e1_1)
  p_proj = wp.dot(p0, n1)
  q0_d = wp.dot(q0, n1)
  q1_d = wp.dot(q1, n1)
  q2_d = wp.dot(q2, n1)
  diff1 = wp.max(p_proj - wp.max(q0_d, wp.max(q1_d, q2_d)), wp.min(q0_d, wp.min(q1_d, q2_d)) - p_proj)
  if diff1 > 0.0:
    ln1_sq = wp.length_sq(n1)
    if ln1_sq > 1e-12 and diff1 * diff1 > cutoff_sq * ln1_sq:
      return True

  # Face normal of triangle 2
  e2_0 = q1 - q0
  e2_1 = q2 - q1
  e2_2 = q0 - q2
  n2 = wp.cross(e2_0, e2_1)
  q_proj = wp.dot(q0, n2)
  p0_d = wp.dot(p0, n2)
  p1_d = wp.dot(p1, n2)
  p2_d = wp.dot(p2, n2)
  diff2 = wp.max(wp.min(p0_d, wp.min(p1_d, p2_d)) - q_proj, q_proj - wp.max(p0_d, wp.max(p1_d, p2_d)))
  if diff2 > 0.0:
    ln2_sq = wp.length_sq(n2)
    if ln2_sq > 1e-12 and diff2 * diff2 > cutoff_sq * ln2_sq:
      return True

  # Edge-edge cross products (3x3 = 9 axes)
  for i in range(3):
    e1 = e1_0
    if i == 1:
      e1 = e1_1
    elif i == 2:
      e1 = e1_2
    for j in range(3):
      e2 = e2_0
      if j == 1:
        e2 = e2_1
      elif j == 2:
        e2 = e2_2
      if _axis_separated(p0, p1, p2, q0, q1, q2, wp.cross(e1, e2), cutoff_sq):
        return True

  # In-plane edge normals for triangle 1 (3 axes)
  for i in range(3):
    e = e1_0
    if i == 1:
      e = e1_1
    elif i == 2:
      e = e1_2
    if _axis_separated(p0, p1, p2, q0, q1, q2, wp.cross(e, n1), cutoff_sq):
      return True

  # In-plane edge normals for triangle 2 (3 axes)
  for i in range(3):
    e = e2_0
    if i == 1:
      e = e2_1
    elif i == 2:
      e = e2_2
    if _axis_separated(p0, p1, p2, q0, q1, q2, wp.cross(e, n2), cutoff_sq):
      return True

  # Vertex-to-vertex axes (3x3 = 9 axes)
  for i in range(3):
    p = p0
    if i == 1:
      p = p1
    elif i == 2:
      p = p2
    for j in range(3):
      q = q0
      if j == 1:
        q = q1
      elif j == 2:
        q = q2
      if _axis_separated(p0, p1, p2, q0, q1, q2, p - q, cutoff_sq):
        return True

  # Vertex-to-edge axes: vertices of T1 to edges of T2 (9 axes)
  for i in range(3):
    p = p0
    if i == 1:
      p = p1
    elif i == 2:
      p = p2
    for j in range(3):
      qa = q0
      qb = q1
      if j == 1:
        qa = q1
        qb = q2
      elif j == 2:
        qa = q2
        qb = q0
      if _axis_separated(p0, p1, p2, q0, q1, q2, _point_segment_axis(p, qa, qb), cutoff_sq):
        return True

  # Vertex-to-edge axes: vertices of T2 to edges of T1 (9 axes)
  for i in range(3):
    q = q0
    if i == 1:
      q = q1
    elif i == 2:
      q = q2
    for j in range(3):
      pa = p0
      pb = p1
      if j == 1:
        pa = p1
        pb = p2
      elif j == 2:
        pa = p2
        pb = p0
      if _axis_separated(p0, p1, p2, q0, q1, q2, _point_segment_axis(q, pa, pb), cutoff_sq):
        return True

  return False


@wp.kernel
def _flex_broadphase_bounds(
  # Model:
  flex_margin: wp.array[float],
  flex_gap: wp.array[float],
  flex_vertadr: wp.array[int],
  flex_vertnum: wp.array[int],
  flex_radius: wp.array[float],
  flex_rod_is_box: wp.array[bool],
  flex_rod_box_size: wp.array[wp.vec2],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  # Data out:
  flex_aabb_min_out: wp.array2d[wp.vec3],
  flex_aabb_max_out: wp.array2d[wp.vec3],
):
  worldid, flexid = wp.tid()

  start = flex_vertadr[flexid]
  num = flex_vertnum[flexid]
  if num == 0:
    return

  min_bound = wp.vec3(MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL)
  max_bound = wp.vec3(-MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL)

  for i in range(num):
    pos = flexvert_xpos_in[worldid, start + i]
    min_bound = wp.min(min_bound, pos)
    max_bound = wp.max(max_bound, pos)

  margin = flex_margin[flexid] + flex_gap[flexid]
  rad = flex_radius[flexid]
  is_box, box_2d = support.rod_box_size(flex_rod_is_box, flex_rod_box_size, flexid)
  if is_box:
    rad = wp.max(rad, wp.length(box_2d))
  bound = rad + margin
  inflate = wp.vec3(bound, bound, bound)

  flex_aabb_min_out[worldid, flexid] = min_bound - inflate
  flex_aabb_max_out[worldid, flexid] = max_bound + inflate


# TODO(team): generalize into a shared contact parameter mixing function
#   (mj_contactParam) that works for both geom-geom and geom-flex contacts.
@wp.func
def _mix_flex_contact_params(
  # In:
  a_condim: int,
  a_priority: int,
  a_solmix: float,
  a_solref: wp.vec2,
  a_solimp: vec5,
  a_friction: wp.vec3,
  a_gap: float,
  b_condim: int,
  b_priority: int,
  b_solmix: float,
  b_solref: wp.vec2,
  b_solimp: vec5,
  b_friction: wp.vec3,
  b_gap: float,
) -> tuple[int, float, wp.vec2, vec5, vec5]:
  """Mix contact parameters between geom and flex, matching mj_contactParam."""
  gap = a_gap + b_gap

  if a_priority > b_priority:
    condim = a_condim
    solref = a_solref
    solimp = a_solimp
    fri = a_friction
  elif a_priority < b_priority:
    condim = b_condim
    solref = b_solref
    solimp = b_solimp
    fri = b_friction
  else:
    # same priority
    condim = wp.max(a_condim, b_condim)

    # compute solver mix factor
    if a_solmix >= MJ_MINVAL and b_solmix >= MJ_MINVAL:
      mix = a_solmix / (a_solmix + b_solmix)
    elif a_solmix < MJ_MINVAL and b_solmix < MJ_MINVAL:
      mix = 0.5
    elif a_solmix < MJ_MINVAL:
      mix = 0.0
    else:
      mix = 1.0

    # solref: mix if both standard, min if either direct
    if a_solref[0] > 0.0 and b_solref[0] > 0.0:
      solref = wp.vec2(
        mix * a_solref[0] + (1.0 - mix) * b_solref[0],
        mix * a_solref[1] + (1.0 - mix) * b_solref[1],
      )
    else:
      solref = wp.vec2(
        wp.min(a_solref[0], b_solref[0]),
        wp.min(a_solref[1], b_solref[1]),
      )

    # solimp: mix
    solimp = vec5(
      mix * a_solimp[0] + (1.0 - mix) * b_solimp[0],
      mix * a_solimp[1] + (1.0 - mix) * b_solimp[1],
      mix * a_solimp[2] + (1.0 - mix) * b_solimp[2],
      mix * a_solimp[3] + (1.0 - mix) * b_solimp[3],
      mix * a_solimp[4] + (1.0 - mix) * b_solimp[4],
    )

    # friction: max
    fri = wp.vec3(
      wp.max(a_friction[0], b_friction[0]),
      wp.max(a_friction[1], b_friction[1]),
      wp.max(a_friction[2], b_friction[2]),
    )

  # unpack 5D friction with MJ_MINMU floor
  friction = vec5(
    wp.max(MJ_MINMU, fri[0]),
    wp.max(MJ_MINMU, fri[0]),
    wp.max(MJ_MINMU, fri[1]),
    wp.max(MJ_MINMU, fri[2]),
    wp.max(MJ_MINMU, fri[2]),
  )

  return condim, gap, solref, solimp, friction


@wp.func
def _write_candidate(
  # In:
  max_candidates: int,
  dist: float,
  pos: wp.vec3,
  nrm: wp.vec3,
  geom: int,
  flexid1: int,
  flexid2: int,
  elemid: int,
  vertid: int,
  worldid: int,
  warn_overflow: bool,
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  cand_dist_out: wp.array[float],
  cand_pos_out: wp.array[wp.vec3],
  cand_nrm_out: wp.array[wp.vec3],
  cand_geom_out: wp.array[wp.vec2i],
  cand_flex_out: wp.array[wp.vec2i],
  cand_elem_out: wp.array[wp.vec2i],
  cand_vert_out: wp.array[wp.vec2i],
  cand_worldid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  if dist >= MJ_MAXVAL:
    return

  candid = wp.atomic_add(ncand_out, 0, 1)  # kernel_analyzer: ignore[atomic]
  if candid >= max_candidates:
    if warn_overflow:
      wp.printf(
        "flex candidate overflow - please increase naconmax beyond %u\n"
        "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.NARROWPHASE (or = 0 for all)\n",
        max_candidates,
      )
    wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.NARROWPHASE))
    return

  cand_dist_out[candid] = dist
  cand_pos_out[candid] = pos
  cand_nrm_out[candid] = nrm
  if geom >= 0:
    cand_geom_out[candid] = wp.vec2i(geom, -1)
    cand_flex_out[candid] = wp.vec2i(flexid1, flexid2)
    cand_elem_out[candid] = wp.vec2i(-1, elemid)
    cand_vert_out[candid] = wp.vec2i(-1, vertid)
  elif geom == -2:
    cand_geom_out[candid] = wp.vec2i(-1, -1)
    cand_flex_out[candid] = wp.vec2i(flexid1, flexid2)
    cand_elem_out[candid] = wp.vec2i(elemid, vertid)
    cand_vert_out[candid] = wp.vec2i(-1, -1)
  else:
    cand_geom_out[candid] = wp.vec2i(-1, -1)
    cand_flex_out[candid] = wp.vec2i(flexid1, flexid2)
    cand_elem_out[candid] = wp.vec2i(-1, elemid)
    cand_vert_out[candid] = wp.vec2i(vertid, -1)
  cand_worldid_out[candid] = worldid


@wp.func
def _collide_geom_capsule_detect(
  # In:
  max_candidates: int,
  gtype: int,
  pos: wp.vec3,
  rot: wp.mat33,
  size_val: wp.vec3,
  cap_pos: wp.vec3,
  cap_axis: wp.vec3,
  cap_radius: float,
  cap_half_len: float,
  margin: float,
  geomid: int,
  flexid: int,
  elemid: int,
  vertex_id: int,
  worldid: int,
  warn_overflow: bool,
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  cand_dist_out: wp.array[float],
  cand_pos_out: wp.array[wp.vec3],
  cand_nrm_out: wp.array[wp.vec3],
  cand_geom_out: wp.array[wp.vec2i],
  cand_flex_out: wp.array[wp.vec2i],
  cand_elem_out: wp.array[wp.vec2i],
  cand_vert_out: wp.array[wp.vec2i],
  cand_worldid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  if gtype == int(GeomType.SPHERE):
    sphere_radius = size_val[0]
    dist, contact_pos, nrm = collision_primitive_core.sphere_capsule(
      pos, sphere_radius, cap_pos, cap_axis, cap_radius, cap_half_len
    )
    if dist < margin:
      _write_candidate(
        max_candidates,
        dist,
        contact_pos,
        nrm,
        geomid,
        -1,
        flexid,
        elemid,
        vertex_id,
        worldid,
        warn_overflow,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        ncand_out,
      )
    return

  dists = wp.vec2(collision_primitive_core.MJ_MAXVAL, collision_primitive_core.MJ_MAXVAL)
  poss = mat23(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  nrms = mat23(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

  if gtype == int(GeomType.CAPSULE):
    g_radius = size_val[0]
    g_half_len = size_val[1]
    g_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    dists, poss, nrms = collision_primitive_core.capsule_capsule(
      pos, g_axis, g_radius, g_half_len, cap_pos, cap_axis, cap_radius, cap_half_len
    )
  elif gtype == int(GeomType.BOX):
    dists, poss, box_nrms = collision_primitive_core.capsule_box(
      cap_pos, cap_axis, cap_radius, cap_half_len, pos, rot, size_val
    )
    nrms = -box_nrms

  for i in range(2):
    if dists[i] < margin:
      _write_candidate(
        max_candidates,
        dists[i],
        poss[i],
        nrms[i],
        geomid,
        -1,
        flexid,
        elemid,
        vertex_id,
        worldid,
        warn_overflow,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        ncand_out,
      )


@wp.func
def _collide_geom_box_rod_detect(
  # In:
  max_candidates: int,
  gtype: int,
  pos: wp.vec3,
  rot: wp.mat33,
  size_val: wp.vec3,
  box_pos: wp.vec3,
  box_rot: wp.mat33,
  box_size: wp.vec3,
  margin: float,
  geomid: int,
  flexid: int,
  elemid: int,
  vertex_id: int,
  worldid: int,
  warn_overflow: bool,
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  cand_dist_out: wp.array[float],
  cand_pos_out: wp.array[wp.vec3],
  cand_nrm_out: wp.array[wp.vec3],
  cand_geom_out: wp.array[wp.vec2i],
  cand_flex_out: wp.array[wp.vec2i],
  cand_elem_out: wp.array[wp.vec2i],
  cand_vert_out: wp.array[wp.vec2i],
  cand_worldid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  if gtype == int(GeomType.SPHERE):
    dist, contact_pos, nrm = collision_primitive_core.sphere_box(pos, size_val[0], box_pos, box_rot, box_size)
    if dist < margin:
      _write_candidate(
        max_candidates,
        dist,
        contact_pos,
        nrm,
        geomid,
        -1,
        flexid,
        elemid,
        vertex_id,
        worldid,
        warn_overflow,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        ncand_out,
      )
    return

  if gtype == int(GeomType.CAPSULE):
    g_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    dists, poss, nrms = collision_primitive_core.capsule_box(pos, g_axis, size_val[0], size_val[1], box_pos, box_rot, box_size)
    for i in range(2):
      if dists[i] < margin:
        _write_candidate(
          max_candidates,
          dists[i],
          poss[i],
          nrms[i],
          geomid,
          -1,
          flexid,
          elemid,
          vertex_id,
          worldid,
          warn_overflow,
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          ncand_out,
        )
    return

  if gtype == int(GeomType.BOX):
    ncon, b_dists, b_poss, b_nrm = collision_primitive_core.box_box(pos, rot, size_val, box_pos, box_rot, box_size, margin)
    for i in range(ncon):
      if b_dists[i] < margin:
        _write_candidate(
          max_candidates,
          b_dists[i],
          b_poss[i],
          b_nrm,
          geomid,
          -1,
          flexid,
          elemid,
          vertex_id,
          worldid,
          warn_overflow,
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          ncand_out,
        )


@wp.func
def _collide_geom_triangle_detect(
  # In:
  max_candidates: int,
  gtype: int,
  pos: wp.vec3,
  rot: wp.mat33,
  size_val: wp.vec3,
  t1: wp.vec3,
  t2: wp.vec3,
  t3: wp.vec3,
  tri_radius: float,
  margin: float,
  geomid: int,
  flexid: int,
  elemid: int,
  vertex_id: int,
  worldid: int,
  warn_overflow: bool,
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  cand_dist_out: wp.array[float],
  cand_pos_out: wp.array[wp.vec3],
  cand_nrm_out: wp.array[wp.vec3],
  cand_geom_out: wp.array[wp.vec2i],
  cand_flex_out: wp.array[wp.vec2i],
  cand_elem_out: wp.array[wp.vec2i],
  cand_vert_out: wp.array[wp.vec2i],
  cand_worldid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  if gtype == int(GeomType.SPHERE):
    sphere_radius = size_val[0]
    dist, contact_pos, nrm = collision_primitive_core.sphere_triangle(pos, sphere_radius, t1, t2, t3, tri_radius, margin)
    _write_candidate(
      max_candidates,
      dist,
      contact_pos,
      nrm,
      geomid,
      -1,
      flexid,
      elemid,
      vertex_id,
      worldid,
      warn_overflow,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      ncand_out,
    )
    return

  if gtype == int(GeomType.CAPSULE):
    cap_radius = size_val[0]
    cap_half_len = size_val[1]
    cap_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    cap_dists, cap_poss, cap_nrms = collision_primitive_core.capsule_triangle(
      pos, cap_axis, cap_radius, cap_half_len, t1, t2, t3, tri_radius, margin
    )
    for i in range(5):
      _write_candidate(
        max_candidates,
        cap_dists[i],
        cap_poss[i],
        cap_nrms[i],
        geomid,
        -1,
        flexid,
        elemid,
        vertex_id,
        worldid,
        warn_overflow,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        ncand_out,
      )
    return

  if gtype == int(GeomType.BOX):
    box_dists, box_poss, box_nrms = collision_primitive_core.box_triangle(pos, rot, size_val, t1, t2, t3, tri_radius, margin)
    for i in range(11):
      _write_candidate(
        max_candidates,
        box_dists[i],
        box_poss[i],
        box_nrms[i],
        geomid,
        -1,
        flexid,
        elemid,
        vertex_id,
        worldid,
        warn_overflow,
        overflow_out,
        cand_dist_out,
        cand_pos_out,
        cand_nrm_out,
        cand_geom_out,
        cand_flex_out,
        cand_elem_out,
        cand_vert_out,
        cand_worldid_out,
        ncand_out,
      )
    return


@wp.func
def _collide_mesh_convex(
  # Model:
  mesh_vertadr: wp.array[int],
  mesh_vertnum: wp.array[int],
  mesh_graphadr: wp.array[int],
  mesh_vert: wp.array[wp.vec3],
  mesh_graph: wp.array[int],
  mesh_polyadr: wp.array[int],
  mesh_polynormal: wp.array[wp.vec3],
  mesh_polyvertadr: wp.array[int],
  mesh_polyvertnum: wp.array[int],
  mesh_polyvert: wp.array[int],
  mesh_polymapadr: wp.array[int],
  mesh_polymapnum: wp.array[int],
  mesh_polymap: wp.array[int],
  # In:
  max_candidates: int,
  mesh_geom_pos: wp.vec3,
  geom_rot: wp.mat33,
  geom_size_val: wp.vec3,
  r_geom: float,
  did: int,
  geom2: Geom,
  geom2_type: int,
  geom2_pos: wp.vec3,
  margin: float,
  geomid: int,
  flexid: int,
  elemid: int,
  vertid: int,
  worldid: int,
  epa_vert: wp.array[wp.vec3],
  epa_vert_index: wp.array[int],
  epa_face: wp.array[int],
  epa_pr: wp.array[wp.vec3],
  epa_norm2: wp.array[float],
  epa_horizon: wp.array[int],
  tolerance: float,
  ccd_iterations: int,
  warn_overflow: bool,
  # Data out:
  overflow_out: wp.array[int],
  # Out:
  cand_dist_out: wp.array[float],
  cand_pos_out: wp.array[wp.vec3],
  cand_nrm_out: wp.array[wp.vec3],
  cand_geom_out: wp.array[wp.vec2i],
  cand_flex_out: wp.array[wp.vec2i],
  cand_elem_out: wp.array[wp.vec2i],
  cand_vert_out: wp.array[wp.vec2i],
  cand_worldid_out: wp.array[int],
  ncand_out: wp.array[int],
):
  # Construct Mesh Geom (geom1)
  geom1 = Geom()
  geom1.pos = mesh_geom_pos
  geom1.rot = geom_rot
  geom1.size = geom_size_val
  geom1.margin = 0.0
  geom1.index = -1
  geom1.vertadr = wp.where(did >= 0, mesh_vertadr[did], -1)
  geom1.vertnum = wp.where(did >= 0, mesh_vertnum[did], -1)
  geom1.graphadr = wp.where(did >= 0, mesh_graphadr[did], -1)
  geom1.vert = mesh_vert
  geom1.graph = mesh_graph

  dist, ncontact, w1, w2, _ = ccd(
    tolerance,
    margin,
    ccd_iterations,
    ccd_iterations,
    geom1,
    geom2,
    int(GeomType.MESH),
    geom2_type,
    mesh_geom_pos,
    geom2_pos,
    epa_vert,
    epa_vert_index,
    epa_face,
    epa_pr,
    epa_norm2,
    epa_horizon,
    warn_overflow,
    worldid,
    overflow_out,
  )

  if ncontact > 0 and dist < margin:
    diff = (w1 - w2) if dist < 0.0 else (w2 - w1)
    gjk_normal = wp.normalize(diff)
    if wp.length(diff) < MJ_MINVAL:
      fallback = geom2_pos - mesh_geom_pos
      gjk_normal = wp.normalize(fallback) if wp.length(fallback) >= MJ_MINVAL else wp.vec3(1.0, 0.0, 0.0)

    # If the mesh witness point lies strictly in the interior of a mesh face (a flex vertex or edge
    # against the face), use the exact face normal instead of the single-precision GJK/EPA normal,
    # which loses precision as the distance goes to zero.
    normal = gjk_normal
    if did >= 0:
      sv0 = gjk_support(geom1, int(GeomType.MESH), gjk_normal).vertex_index
      if sv0 >= 0:
        vertadr = geom1.vertadr
        polyadr = mesh_polyadr[did]
        geom_rot_T = wp.transpose(geom_rot)
        w1_local = geom_rot_T @ (w1 - mesh_geom_pos)
        gjk_normal_local = geom_rot_T @ gjk_normal
        plane_tol = 1.0e-3 * r_geom

        polymapadr = mesh_polymapadr[vertadr + sv0]
        polymapnum = mesh_polymapnum[vertadr + sv0]
        for k in range(polymapnum):
          poly = polyadr + mesh_polymap[polymapadr + k]
          npv = mesh_polyvertnum[poly]
          if npv < 3:
            continue
          poly_normal = mesh_polynormal[poly]
          if wp.dot(poly_normal, gjk_normal_local) <= 0.0:
            continue
          pvadr = mesh_polyvertadr[poly]
          v0 = mesh_vert[vertadr + mesh_polyvert[pvadr]]
          if wp.abs(wp.dot(w1_local - v0, poly_normal)) > plane_tol:
            continue

          v_curr = v0
          inside = bool(True)
          for j in range(npv):
            v_next = v0 if j + 1 == npv else mesh_vert[vertadr + mesh_polyvert[pvadr + j + 1]]
            edge = v_next - v_curr
            edge_len_sq = wp.length_sq(edge)
            if (
              edge_len_sq >= MJ_MINVAL * MJ_MINVAL
              and wp.dot(wp.cross(poly_normal, edge), w1_local - v_curr) <= 1.0e-5 * edge_len_sq
            ):
              inside = False
              break
            v_curr = v_next
          if inside:
            normal = wp.normalize(geom_rot @ poly_normal)
            break

    contact_pos = 0.5 * (w1 + w2)

    _write_candidate(
      max_candidates,
      dist,
      contact_pos,
      normal,
      geomid,
      -1,
      flexid,
      elemid,
      vertid,
      worldid,
      warn_overflow,
      overflow_out,
      cand_dist_out,
      cand_pos_out,
      cand_nrm_out,
      cand_geom_out,
      cand_flex_out,
      cand_elem_out,
      cand_vert_out,
      cand_worldid_out,
      ncand_out,
    )


@wp.func
def _fix_cylinder_normal(
  # In:
  gpos: wp.vec3,
  grot: wp.mat33,
  gsize: wp.vec3,
  pos: wp.vec3,
  normal: wp.vec3,
) -> wp.vec3:
  """Analytic cylinder normal for cylinder : 2D flex contacts (MuJoCo C mjc_fixNormal)."""
  pos1 = wp.transpose(grot) @ (pos - gpos)
  # keep the collider normal near the flat wall, unless 4x closer to the round wall
  if wp.abs(pos1[2]) > 0.95 * gsize[1]:
    return normal
  dst1 = wp.abs(gsize[1] - wp.abs(pos1[2]))
  dst2 = wp.abs(gsize[0] - wp.sqrt(pos1[0] * pos1[0] + pos1[1] * pos1[1]))
  if dst1 < 0.25 * dst2:
    return normal
  return grot @ wp.normalize(wp.vec3(pos1[0], pos1[1], 0.0))


@wp.func
def _fix_ellipsoid_normal(
  # In:
  gpos: wp.vec3,
  grot: wp.mat33,
  gsize: wp.vec3,
  pos: wp.vec3,
  normal: wp.vec3,
) -> wp.vec3:
  """Analytic ellipsoid normal for ellipsoid : 2D flex contacts (MuJoCo C mjc_fixNormal)."""
  if gsize[0] < MJ_MINVAL or gsize[1] < MJ_MINVAL or gsize[2] < MJ_MINVAL:
    return normal

  pos1 = wp.transpose(grot) @ (pos - gpos)
  nrm = wp.transpose(grot) @ normal
  s2 = wp.cw_mul(gsize, gsize)
  s2_inv = wp.vec3(1.0 / s2[0], 1.0 / s2[1], 1.0 / s2[2])
  dst1 = pos1[0] * pos1[0] * s2_inv[0] + pos1[1] * pos1[1] * s2_inv[1] + pos1[2] * pos1[2] * s2_inv[2]

  if dst1 <= 1.0:
    # ray-projection SQP for point inside or on ellipsoid (mjc_ellipsoidInside)
    c_val = dst1 - 1.0
    nrm = wp.normalize(nrm)
    processed = bool(True)
    for iter_idx in range(30):
      a_val = nrm[0] * nrm[0] * s2_inv[0] + nrm[1] * nrm[1] * s2_inv[1] + nrm[2] * nrm[2] * s2_inv[2]
      b_val = pos1[0] * nrm[0] * s2_inv[0] + pos1[1] * nrm[1] * s2_inv[1] + pos1[2] * nrm[2] * s2_inv[2]
      det = b_val * b_val - a_val * c_val
      if det < MJ_MINVAL or a_val < MJ_MINVAL:
        processed = iter_idx > 0
        break
      x = (-b_val + wp.sqrt(det)) / a_val
      if x < 0.0:
        processed = iter_idx > 0
        break
      pnt = pos1 + x * nrm
      newnrm = wp.normalize(wp.cw_mul(pnt, s2_inv))
      change = wp.length(nrm - newnrm)
      nrm = newnrm
      if change < 1e-6:
        break
    if not processed:
      return normal
    return grot @ wp.normalize(nrm)

  # diagonal QCQP for point outside ellipsoid (mjc_ellipsoidOutside)
  ps2 = wp.vec3(pos1[0] * pos1[0] * s2[0], pos1[1] * pos1[1] * s2[1], pos1[2] * pos1[2] * s2[2])
  la = float(0.0)
  for _ in range(30):
    r0 = safe_div(1.0, s2[0] + la)
    r1 = safe_div(1.0, s2[1] + la)
    r2 = safe_div(1.0, s2[2] + la)
    r0_2 = r0 * r0
    r1_2 = r1 * r1
    r2_2 = r2 * r2
    val = ps2[0] * r0_2 + ps2[1] * r1_2 + ps2[2] * r2_2 - 1.0
    if val < 1e-6:
      break
    deriv = -2.0 * (ps2[0] * r0_2 * r0 + ps2[1] * r1_2 * r1 + ps2[2] * r2_2 * r2)
    if deriv > -MJ_MINVAL:
      break
    delta = -val / deriv
    if delta < 1e-6:
      break
    la += delta

  nrm = wp.normalize(
    wp.vec3(
      safe_div(pos1[0], s2[0] + la),
      safe_div(pos1[1], s2[1] + la),
      safe_div(pos1[2], s2[2] + la),
    )
  )
  return grot @ nrm


@wp.func
def _is_passive_flex_cand(
  # Model:
  opt_integrator: int,
  body_weldid: wp.array[int],
  geom_bodyid: wp.array[int],
  flex_passive: wp.array[int],
  flex_dim: wp.array[int],
  flex_interp: wp.array[int],
  flex_rigid: wp.array[bool],
  # In:
  cand_geom: wp.array[wp.vec2i],
  cand_flex: wp.array[wp.vec2i],
  cid: int,
) -> bool:
  if opt_integrator != int(IntegratorType.DISCRETE):
    return False
  flex_pair = cand_flex[cid]
  geom_pair = cand_geom[cid]
  f0 = flex_pair[0]
  f1 = flex_pair[1]
  g0 = geom_pair[0]
  g1 = geom_pair[1]
  wants = (f0 >= 0 and not flex_rigid[f0] and flex_passive[f0] != 0 and flex_interp[f0] == 0 and flex_dim[f0] >= 2) or (
    f1 >= 0 and not flex_rigid[f1] and flex_passive[f1] != 0 and flex_interp[f1] == 0 and flex_dim[f1] >= 2
  )
  ok = (f0 >= 0 or (g0 >= 0 and body_weldid[geom_bodyid[g0]] == 0)) and (
    f1 >= 0 or (g1 >= 0 and body_weldid[geom_bodyid[g1]] == 0)
  )
  return wants and ok


@cache_kernel
def _flex_plane_narrowphase(warn_overflow: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    ngeom: int,
    qpos0: wp.array2d[float],
    jnt_qposadr: wp.array[int],
    geom_type: wp.array[int],
    geom_contype: wp.array[int],
    geom_conaffinity: wp.array[int],
    geom_margin: wp.array2d[float],
    geom_gap: wp.array2d[float],
    flex_contype: wp.array[int],
    flex_conaffinity: wp.array[int],
    flex_margin: wp.array[float],
    flex_gap: wp.array[float],
    flex_vertadr: wp.array[int],
    flex_elemadr: wp.array[int],
    flex_elemnum: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_plasticadr: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_radius: wp.array[float],
    flex_vertflexid: wp.array[int],
    flex_rod_is_box: wp.array[bool],
    flex_rod_root_clamped: wp.array[bool],
    flex_rod_tail_clamped: wp.array[bool],
    flex_rod_box_size: wp.array[wp.vec2],
    flex_rod_edgeadr: wp.array[int],
    flex_rod_edgenum: wp.array[int],
    flex_rodedge_vert: wp.array[wp.vec2i],
    flex_rodedge_jnt: wp.array[int],
    flex_elem_rodedge: wp.array[wp.vec2i],
    flex_vert_rodelem: wp.array[int],
    # Data in:
    qpos_in: wp.array2d[float],
    plastic_in: wp.array2d[float],
    xquat_in: wp.array2d[wp.quat],
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    flex_aabb_min_in: wp.array2d[wp.vec3],
    flex_aabb_max_in: wp.array2d[wp.vec3],
    flex_rod_frame0_loc_in: wp.array3d[wp.vec3],
    flex_rod_frame0_ref_in: wp.array3d[wp.vec3],
    flex_rod_tail_loc_in: wp.array3d[wp.vec3],
    # In:
    max_candidates: int,
    # Data out:
    overflow_out: wp.array[int],
    # Out:
    cand_dist_out: wp.array[float],
    cand_pos_out: wp.array[wp.vec3],
    cand_nrm_out: wp.array[wp.vec3],
    cand_geom_out: wp.array[wp.vec2i],
    cand_flex_out: wp.array[wp.vec2i],
    cand_elem_out: wp.array[wp.vec2i],
    cand_vert_out: wp.array[wp.vec2i],
    cand_worldid_out: wp.array[int],
    ncand_out: wp.array[int],
  ):
    worldid, vertid = wp.tid()

    flexid = flex_vertflexid[vertid]
    radius = flex_radius[flexid]
    flex_margin_val = flex_margin[flexid] + flex_gap[flexid]
    local_vertid = vertid - flex_vertadr[flexid]

    elemnum = flex_elemnum[flexid]
    is_box = bool(False)
    box_2d = wp.vec2(0.0, 0.0)
    if elemnum > 0:
      is_box, box_2d = support.rod_box_size(flex_rod_is_box, flex_rod_box_size, flexid)
    bound = wp.length(box_2d) if is_box else radius

    ok_box = bool(False)
    box_evaluated = bool(False)
    bmat = wp.mat33(0.0)

    vert = flexvert_xpos_in[worldid, vertid]
    flex_aabb_min = flex_aabb_min_in[worldid, flexid]
    flex_aabb_max = flex_aabb_max_in[worldid, flexid]
    flex_center = 0.5 * (flex_aabb_min + flex_aabb_max)
    flex_half_size = 0.5 * (flex_aabb_max - flex_aabb_min)

    for geomid in range(ngeom):
      if geom_type[geomid] != int(GeomType.PLANE):
        continue

      g_contype = geom_contype[geomid]
      g_conaffinity = geom_conaffinity[geomid]
      f_contype = flex_contype[flexid]
      f_conaffinity = flex_conaffinity[flexid]
      if not ((g_contype & f_conaffinity) or (f_contype & g_conaffinity)):
        continue

      margin = (
        geom_margin[worldid % geom_margin.shape[0], geomid] + geom_gap[worldid % geom_gap.shape[0], geomid] + flex_margin_val
      )
      geom_pos = geom_xpos_in[worldid, geomid]
      geom_rot = quat_to_mat(geom_xquat_in[worldid, geomid])
      plane_normal = wp.vec3(geom_rot[0, 2], geom_rot[1, 2], geom_rot[2, 2])

      # Coarse AABB vs plane test
      proj_half = (
        wp.abs(flex_half_size[0] * plane_normal[0])
        + wp.abs(flex_half_size[1] * plane_normal[1])
        + wp.abs(flex_half_size[2] * plane_normal[2])
      )
      diff_center = flex_center - geom_pos
      dist_center = wp.dot(diff_center, plane_normal)
      if dist_center - proj_half > margin:
        continue

      diff = vert - geom_pos
      signed_dist = wp.dot(diff, plane_normal)
      if signed_dist > margin + bound:
        continue

      if is_box and not box_evaluated:
        box_evaluated = True
        e_rod = flex_vert_rodelem[vertid]
        if e_rod >= 0:
          ok_box, _mid_box, bmat, _hsize_box = support.rod_box_elem(
            qpos0,
            jnt_qposadr,
            flex_vertadr,
            flex_elemadr,
            flex_elemnum,
            flex_elemdataadr,
            flex_plasticadr,
            flex_vertbodyid,
            flex_elem,
            flex_rod_is_box,
            flex_rod_root_clamped,
            flex_rod_tail_clamped,
            flex_rod_box_size,
            flex_rod_edgeadr,
            flex_rod_edgenum,
            flex_rodedge_vert,
            flex_rodedge_jnt,
            flex_elem_rodedge,
            qpos_in,
            plastic_in,
            xquat_in,
            flexvert_xpos_in,
            flex_rod_frame0_loc_in,
            flex_rod_frame0_ref_in,
            flex_rod_tail_loc_in,
            worldid,
            flexid,
            e_rod,
          )

      if ok_box:
        m1 = wp.vec3(bmat[0, 0], bmat[1, 0], bmat[2, 0])
        m2 = wp.vec3(bmat[0, 1], bmat[1, 1], bmat[2, 1])
        n1 = wp.dot(plane_normal, m1)
        n2 = wp.dot(plane_normal, m2)
        s1 = -1.0 if n1 >= 0.0 else 1.0
        s2 = -1.0 if n2 >= 0.0 else 1.0
        c1_0 = s1 * box_2d[0]
        c1_1 = c1_0
        c2_0 = s2 * box_2d[1]
        c2_1 = c2_0
        if box_2d[0] * wp.abs(n1) <= box_2d[1] * wp.abs(n2):
          c1_1 = -c1_0
        else:
          c2_1 = -c2_0
        c1_vec = wp.vec2(c1_0, c1_1)
        c2_vec = wp.vec2(c2_0, c2_1)
        for k in range(2):
          corner = c1_vec[k] * m1 + c2_vec[k] * m2
          ldist = wp.dot(corner, plane_normal)
          cdist = signed_dist + ldist
          if cdist < margin and not (k == 1 and ldist > 0.0):
            contact_pos = vert + corner - (0.5 * cdist) * plane_normal
            _write_candidate(
              max_candidates,
              cdist,
              contact_pos,
              plane_normal,
              geomid,
              -1,
              flexid,
              -1,
              local_vertid,
              worldid,
              wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
              overflow_out,
              cand_dist_out,
              cand_pos_out,
              cand_nrm_out,
              cand_geom_out,
              cand_flex_out,
              cand_elem_out,
              cand_vert_out,
              cand_worldid_out,
              ncand_out,
            )
        continue

      dist = signed_dist - radius

      if dist < margin:
        contact_pos = vert - plane_normal * (dist * 0.5 + radius)
        _write_candidate(
          max_candidates,
          dist,
          contact_pos,
          plane_normal,
          geomid,
          -1,
          flexid,
          -1,
          local_vertid,
          worldid,
          wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          ncand_out,
        )

  return kernel


@wp.func
def _exclude_self_collision(
  # Model:
  flex_vertbodyid: wp.array[int],
  # In:
  v1: wp.vec4i,
  n1: int,
  v2: wp.vec4i,
  n2: int,
  vert_adr: int,
) -> bool:
  for i in range(n1):
    idx1 = v1[i]
    if idx1 >= 0:
      b1 = flex_vertbodyid[vert_adr + idx1]
      for j in range(n2):
        idx2 = v2[j]
        if idx1 == idx2:
          return True
        if idx2 >= 0 and b1 >= 0:
          b2 = flex_vertbodyid[vert_adr + idx2]
          if b1 == b2:
            return True
  return False


@wp.func
def _get_element_vertices(
  # Model:
  flex_elem: wp.array[int],
  # In:
  dim: int,
  elem_data_idx: int,
) -> wp.vec4i:
  v0 = flex_elem[elem_data_idx]
  v1 = flex_elem[elem_data_idx + 1]
  v2 = int(-1)
  v3 = int(-1)
  if dim >= 2:
    v2 = flex_elem[elem_data_idx + 2]
  if dim >= 3:
    v3 = flex_elem[elem_data_idx + 3]
  return wp.vec4i(v0, v1, v2, v3)


@wp.func
def _elem_active(
  # Model:
  flex_activelayers: wp.array[int],
  flex_dim: wp.array[int],
  flex_elemadr: wp.array[int],
  flex_elemlayer: wp.array[int],
  # In:
  flexid: int,
  elemid: int,
) -> bool:
  if flex_dim[flexid] < 3:
    return True
  return flex_elemlayer[flex_elemadr[flexid] + elemid] < flex_activelayers[flexid]


@wp.kernel
def _flex_bvh_leaf(
  # Model:
  nbvhstatic: int,
  bvh_nodeid: wp.array[int],
  flex_dim: wp.array[int],
  flex_vertadr: wp.array[int],
  flex_elemdataadr: wp.array[int],
  flex_elem: wp.array[int],
  flex_radius: wp.array[float],
  flex_bvhflexid: wp.array[int],
  flex_rod_is_box: wp.array[bool],
  flex_rod_box_size: wp.array[wp.vec2],
  # Data in:
  flexvert_xpos_in: wp.array2d[wp.vec3],
  # Out:
  bvh_lower_out: wp.array2d[wp.vec3],
  bvh_upper_out: wp.array2d[wp.vec3],
):
  worldid, nodeid = wp.tid()

  # leaves hold active elements only (MuJoCo excludes inactive 3D elements from the flex BVH)
  e = bvh_nodeid[nbvhstatic + nodeid]
  if e < 0:
    return

  flexid = flex_bvhflexid[nodeid]
  dim = flex_dim[flexid]
  vert_adr = flex_vertadr[flexid]
  v = _get_element_vertices(flex_elem, dim, flex_elemdataadr[flexid] + e * (dim + 1))

  p0 = flexvert_xpos_in[worldid, vert_adr + v[0]]
  p1 = flexvert_xpos_in[worldid, vert_adr + v[1]]
  aabb_min = wp.min(p0, p1)
  aabb_max = wp.max(p0, p1)

  if dim >= 2:
    p2 = flexvert_xpos_in[worldid, vert_adr + v[2]]
    aabb_min = wp.min(aabb_min, p2)
    aabb_max = wp.max(aabb_max, p2)
  if dim >= 3:
    p3 = flexvert_xpos_in[worldid, vert_adr + v[3]]
    aabb_min = wp.min(aabb_min, p3)
    aabb_max = wp.max(aabb_max, p3)

  radius = flex_radius[flexid]
  is_box, box_2d = support.rod_box_size(flex_rod_is_box, flex_rod_box_size, flexid)
  if is_box:
    radius = wp.max(radius, wp.length(box_2d))
  inflate = wp.vec3(radius, radius, radius)
  bvh_lower_out[worldid, nodeid] = aabb_min - inflate
  bvh_upper_out[worldid, nodeid] = aabb_max + inflate


@wp.kernel
def _flex_bvh_refit(
  # Model:
  nbvhstatic: int,
  bvh_child: wp.array[wp.vec2i],
  flex_bvhadr: wp.array[int],
  flex_bvhflexid: wp.array[int],
  # In:
  level: wp.array[int],
  # Out:
  bvh_lower_out: wp.array2d[wp.vec3],
  bvh_upper_out: wp.array2d[wp.vec3],
):
  worldid, levelid = wp.tid()

  bvhid = level[levelid]
  nodeid = bvhid - nbvhstatic
  child = bvh_child[bvhid]
  adr = flex_bvhadr[flex_bvhflexid[nodeid]] - nbvhstatic
  child1 = adr + child[0]
  child2 = adr + child[1]
  bvh_lower_out[worldid, nodeid] = wp.min(bvh_lower_out[worldid, child1], bvh_lower_out[worldid, child2])
  bvh_upper_out[worldid, nodeid] = wp.max(bvh_upper_out[worldid, child1], bvh_upper_out[worldid, child2])


@cache_kernel
def _flex_bvh_query(is_self: bool, warn_overflow: int, enable_sat: bool, split_depth: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    nbvhstatic: int,
    nflex: int,
    bvh_child: wp.array[wp.vec2i],
    bvh_nodeid: wp.array[int],
    flex_contype: wp.array[int],
    flex_conaffinity: wp.array[int],
    flex_margin: wp.array[float],
    flex_gap: wp.array[float],
    flex_selfcollide: wp.array[int],
    flex_dim: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemadr: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_radius: wp.array[float],
    flex_bvhadr: wp.array[int],
    flex_bvhflexid: wp.array[int],
    # Data in:
    flexvert_xpos_in: wp.array2d[wp.vec3],
    flex_aabb_min_in: wp.array2d[wp.vec3],
    flex_aabb_max_in: wp.array2d[wp.vec3],
    # In:
    bvh_lower_in: wp.array2d[wp.vec3],
    bvh_upper_in: wp.array2d[wp.vec3],
    max_pairs: int,
    # Data out:
    ncollision_out: wp.array[int],
    overflow_out: wp.array[int],
    # Out:
    collision_pair_out: wp.array[wp.vec2i],
    collision_worldid_out: wp.array[int],
  ):
    # one thread per leaf and subtree split: query the trees of the flexes the leaf element can
    # collide with
    worldid, nodeid1, splitid = wp.tid()

    bvhid1 = nbvhstatic + nodeid1
    e1 = bvh_nodeid[bvhid1]
    if e1 < 0:
      return

    flexid1 = flex_bvhflexid[nodeid1]
    if wp.static(is_self):
      if flex_selfcollide[flexid1] == 0 or (flex_contype[flexid1] & flex_conaffinity[flexid1]) == 0:
        return
    elif flexid1 >= nflex - 1:
      return

    elem1 = flex_elemadr[flexid1] + e1
    lower1 = bvh_lower_in[worldid, nodeid1]
    upper1 = bvh_upper_in[worldid, nodeid1]
    dim1 = flex_dim[flexid1]
    vert_adr1 = flex_vertadr[flexid1]
    v1_indices = _get_element_vertices(flex_elem, dim1, flex_elemdataadr[flexid1] + e1 * (dim1 + 1))

    if wp.static(is_self):
      flexid_start = flexid1
      flexid_end = flexid1 + 1
    else:
      flexid_start = flexid1 + 1
      flexid_end = nflex

    stack = _vec_bvh_stack()

    for flexid2 in range(flexid_start, flexid_end):
      bvhadr = flex_bvhadr[flexid2]
      if bvhadr < 0:
        continue

      q_lower1 = lower1
      q_upper1 = upper1
      pair_margin = float(0.0)
      if not wp.static(is_self):
        if _flex_element_aabb_filter(
          flex_aabb_min_in[worldid, flexid1],
          flex_aabb_max_in[worldid, flexid1],
          flex_aabb_min_in[worldid, flexid2],
          flex_aabb_max_in[worldid, flexid2],
        ):
          continue

        contype1 = flex_contype[flexid1]
        conaffinity1 = flex_conaffinity[flexid1]
        contype2 = flex_contype[flexid2]
        conaffinity2 = flex_conaffinity[flexid2]
        if not ((contype1 & conaffinity2) or (contype2 & conaffinity1)):
          continue

        pair_margin = flex_margin[flexid1] + flex_margin[flexid2] + flex_gap[flexid1] + flex_gap[flexid2]
        margin_vec = wp.vec3(pair_margin, pair_margin, pair_margin)
        q_lower1 = lower1 - margin_vec
        q_upper1 = upper1 + margin_vec

      # trees are stored in preorder, the subtree of a left child ends where its sibling starts:
      # self-collision skips subtrees of leaves before this one, so each pair is visited once
      leaf1 = bvhid1 - bvhadr

      # start at the subtree selected by the bits of splitid, split_depth levels below the root
      node = int(0)
      valid = bool(True)
      for k in range(wp.static(split_depth)):
        if bvh_nodeid[bvhadr + node] >= 0:
          valid = (splitid >> k) == 0
          break
        child = bvh_child[bvhadr + node]
        if ((splitid >> k) & 1) != 0:
          node = child[1]
        else:
          if wp.static(is_self):
            if child[1] <= leaf1:
              valid = False
              break
          node = child[0]
      if not valid:
        continue

      startid = bvhadr + node - nbvhstatic
      if _flex_element_aabb_filter(q_lower1, q_upper1, bvh_lower_in[worldid, startid], bvh_upper_in[worldid, startid]):
        continue
      stack[0] = node
      nstack = int(1)

      dim2 = flex_dim[flexid2]
      vert_adr2 = flex_vertadr[flexid2]
      elem_adr2 = flex_elemadr[flexid2]
      elem_data_adr2 = flex_elemdataadr[flexid2]
      cutoff_sq = float(0.0)
      p0 = wp.vec3(0.0)
      p1 = wp.vec3(0.0)
      p2 = wp.vec3(0.0)
      if wp.static(enable_sat):
        if dim1 == 2 and dim2 == 2:
          r1 = flex_radius[flexid1]
          cutoff = float(2.0) * r1
          if not wp.static(is_self):
            cutoff = r1 + flex_radius[flexid2] + pair_margin
          cutoff_sq = cutoff * cutoff
          p0 = flexvert_xpos_in[worldid, vert_adr1 + v1_indices[0]]
          p1 = flexvert_xpos_in[worldid, vert_adr1 + v1_indices[1]]
          p2 = flexvert_xpos_in[worldid, vert_adr1 + v1_indices[2]]

      # nodes on the stack overlap the query element
      while nstack > 0:
        nstack -= 1
        bvhid2 = bvhadr + stack[nstack]
        e2 = bvh_nodeid[bvhid2]

        if e2 < 0:
          child = bvh_child[bvhid2]
          for i in range(2):
            if wp.static(is_self):
              if i == 0 and child[1] <= leaf1:
                continue
            childid = bvhadr + child[i] - nbvhstatic
            if not _flex_element_aabb_filter(
              q_lower1, q_upper1, bvh_lower_in[worldid, childid], bvh_upper_in[worldid, childid]
            ):
              stack[nstack] = child[i]
              nstack += 1
          continue

        if wp.static(is_self):
          if bvhid2 - bvhadr <= leaf1:
            continue

        elem2 = elem_adr2 + e2
        v2_indices = _get_element_vertices(flex_elem, dim2, elem_data_adr2 + e2 * (dim2 + 1))
        if wp.static(is_self):
          if _exclude_self_collision(flex_vertbodyid, v1_indices, dim1 + 1, v2_indices, dim1 + 1, vert_adr1):
            continue
        else:
          shared_body = bool(False)
          for ii in range(dim1 + 1):
            idx1 = v1_indices[ii]
            if idx1 >= 0:
              b1 = flex_vertbodyid[vert_adr1 + idx1]
              for jj in range(dim2 + 1):
                idx2 = v2_indices[jj]
                if idx2 >= 0 and b1 >= 0:
                  b2 = flex_vertbodyid[vert_adr2 + idx2]
                  if b1 == b2:
                    shared_body = bool(True)
                    break
              if shared_body:
                break

          if shared_body:
            continue

        if dim1 == 2 and dim2 == 2 and wp.static(enable_sat):
          q0 = flexvert_xpos_in[worldid, vert_adr2 + v2_indices[0]]
          q1 = flexvert_xpos_in[worldid, vert_adr2 + v2_indices[1]]
          q2 = flexvert_xpos_in[worldid, vert_adr2 + v2_indices[2]]
          if _triangle_sat_separated(p0, p1, p2, q0, q1, q2, cutoff_sq):
            continue

        idx = wp.atomic_add(ncollision_out, 0, 1)  # kernel_analyzer: ignore[atomic]
        if idx >= max_pairs:
          if wp.static(bool(warn_overflow & OverflowType.BROADPHASE)):
            wp.printf(
              "Flex BVH buffer overflow - please increase naconmax beyond %u\n"
              "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.BROADPHASE (or = 0 for all)\n",
              max_pairs,
            )
          wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.BROADPHASE))
          return

        elem_a = wp.min(elem1, elem2) if wp.static(is_self) else elem1
        elem_b = wp.max(elem1, elem2) if wp.static(is_self) else elem2
        collision_pair_out[idx] = wp.vec2i(elem_a, elem_b)
        collision_worldid_out[idx] = worldid

  return kernel


@cache_kernel
def _flex_narrowphase(warn_overflow: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_ccd_tolerance: wp.array[float],
    qpos0: wp.array2d[float],
    jnt_qposadr: wp.array[int],
    flex_margin: wp.array[float],
    flex_gap: wp.array[float],
    flex_activelayers: wp.array[int],
    flex_dim: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemadr: wp.array[int],
    flex_elemnum: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_plasticadr: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_elemlayer: wp.array[int],
    flex_radius: wp.array[float],
    flex_elemflexid: wp.array[int],
    flex_rod_is_box: wp.array[bool],
    flex_rod_root_clamped: wp.array[bool],
    flex_rod_tail_clamped: wp.array[bool],
    flex_rod_box_size: wp.array[wp.vec2],
    flex_rod_edgeadr: wp.array[int],
    flex_rod_edgenum: wp.array[int],
    flex_rodedge_vert: wp.array[wp.vec2i],
    flex_rodedge_jnt: wp.array[int],
    flex_elem_rodedge: wp.array[wp.vec2i],
    # Data in:
    qpos_in: wp.array2d[float],
    plastic_in: wp.array2d[float],
    xquat_in: wp.array2d[wp.quat],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    naccdmax_in: int,
    ncollision_in: wp.array[int],
    flex_rod_frame0_loc_in: wp.array3d[wp.vec3],
    flex_rod_frame0_ref_in: wp.array3d[wp.vec3],
    flex_rod_tail_loc_in: wp.array3d[wp.vec3],
    # In:
    max_candidates: int,
    gjk_iterations: int,
    epa_iterations: int,
    collision_pair_in: wp.array[wp.vec2i],
    collision_worldid_in: wp.array[int],
    # Data out:
    overflow_out: wp.array[int],
    # Out:
    workspace_verts_out: wp.array[wp.vec3],
    epa_vert_out: wp.array2d[wp.vec3],
    epa_vert_index_out: wp.array2d[int],
    epa_face_out: wp.array2d[int],
    epa_pr_out: wp.array2d[wp.vec3],
    epa_norm2_out: wp.array2d[float],
    epa_horizon_out: wp.array2d[int],
    cand_dist_out: wp.array[float],
    cand_pos_out: wp.array[wp.vec3],
    cand_nrm_out: wp.array[wp.vec3],
    cand_geom_out: wp.array[wp.vec2i],
    cand_flex_out: wp.array[wp.vec2i],
    cand_elem_out: wp.array[wp.vec2i],
    cand_vert_out: wp.array[wp.vec2i],
    cand_worldid_out: wp.array[int],
    ncand_out: wp.array[int],
  ):
    """Process broadphase-identified flex pairs through narrowphase."""
    pairid = wp.tid()

    # Check bounds
    if pairid >= ncollision_in[0]:
      return

    pair = collision_pair_in[pairid]
    elem1_global = pair[0]
    elem2_global = pair[1]
    worldid = collision_worldid_in[pairid]

    flexid1 = flex_elemflexid[elem1_global]
    flexid2 = flex_elemflexid[elem2_global]

    radius1 = flex_radius[flexid1]
    radius2 = flex_radius[flexid2]
    dim1 = flex_dim[flexid1]
    dim2 = flex_dim[flexid2]
    vert_adr1 = flex_vertadr[flexid1]
    vert_adr2 = flex_vertadr[flexid2]
    elem_adr1 = flex_elemadr[flexid1]
    elem_adr2 = flex_elemadr[flexid2]

    e1 = elem1_global - elem_adr1
    e2 = elem2_global - elem_adr2

    if not _elem_active(flex_activelayers, flex_dim, flex_elemadr, flex_elemlayer, flexid1, e1):
      return
    if not _elem_active(flex_activelayers, flex_dim, flex_elemadr, flex_elemlayer, flexid2, e2):
      return
    elem_data_idx1 = flex_elemdataadr[flexid1] + e1 * (dim1 + 1)
    elem_data_idx2 = flex_elemdataadr[flexid2] + e2 * (dim2 + 1)

    v1_indices = _get_element_vertices(flex_elem, dim1, elem_data_idx1)
    v2_indices = _get_element_vertices(flex_elem, dim2, elem_data_idx2)

    # Workspace for this pair (8 slots per pair)
    offset1 = pairid * 8
    for idx in range(dim1 + 1):
      workspace_verts_out[offset1 + idx] = flexvert_xpos_in[worldid, vert_adr1 + v1_indices[idx]]

    mix_margin = 0.0
    if flexid1 != flexid2:
      mix_margin = flex_margin[flexid1] + flex_margin[flexid2] + flex_gap[flexid1] + flex_gap[flexid2]

    if dim1 == 1 and dim2 == 1:
      box1, bpos1, bmat1, bsize1 = support.rod_box_elem(
        qpos0,
        jnt_qposadr,
        flex_vertadr,
        flex_elemadr,
        flex_elemnum,
        flex_elemdataadr,
        flex_plasticadr,
        flex_vertbodyid,
        flex_elem,
        flex_rod_is_box,
        flex_rod_root_clamped,
        flex_rod_tail_clamped,
        flex_rod_box_size,
        flex_rod_edgeadr,
        flex_rod_edgenum,
        flex_rodedge_vert,
        flex_rodedge_jnt,
        flex_elem_rodedge,
        qpos_in,
        plastic_in,
        xquat_in,
        flexvert_xpos_in,
        flex_rod_frame0_loc_in,
        flex_rod_frame0_ref_in,
        flex_rod_tail_loc_in,
        worldid,
        flexid1,
        e1,
      )
      box2, bpos2, bmat2, bsize2 = support.rod_box_elem(
        qpos0,
        jnt_qposadr,
        flex_vertadr,
        flex_elemadr,
        flex_elemnum,
        flex_elemdataadr,
        flex_plasticadr,
        flex_vertbodyid,
        flex_elem,
        flex_rod_is_box,
        flex_rod_root_clamped,
        flex_rod_tail_clamped,
        flex_rod_box_size,
        flex_rod_edgeadr,
        flex_rod_edgenum,
        flex_rodedge_vert,
        flex_rodedge_jnt,
        flex_elem_rodedge,
        qpos_in,
        plastic_in,
        xquat_in,
        flexvert_xpos_in,
        flex_rod_frame0_loc_in,
        flex_rod_frame0_ref_in,
        flex_rod_tail_loc_in,
        worldid,
        flexid2,
        e2,
      )
      if box1 and box2:
        ncon_bb, bb_dists, bb_poss, bb_nrm = collision_primitive_core.box_box(
          bpos1, bmat1, bsize1, bpos2, bmat2, bsize2, mix_margin
        )
        for c in range(ncon_bb):
          d_val = bb_dists[c]
          if d_val <= mix_margin:
            _write_candidate(
              max_candidates,
              d_val,
              bb_poss[c],
              bb_nrm,
              -2,
              flexid1,
              flexid2,
              e1,
              e2,
              worldid,
              wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
              overflow_out,
              cand_dist_out,
              cand_pos_out,
              cand_nrm_out,
              cand_geom_out,
              cand_flex_out,
              cand_elem_out,
              cand_vert_out,
              cand_worldid_out,
              ncand_out,
            )
        return

      p0 = workspace_verts_out[offset1]
      p1 = workspace_verts_out[offset1 + 1]
      cap1_pos = 0.5 * (p0 + p1)
      cap1_axis = wp.normalize(p1 - p0)
      cap1_half_len = 0.5 * wp.length(p1 - p0)

      p2_0 = flexvert_xpos_in[worldid, vert_adr2 + v2_indices[0]]
      p2_1 = flexvert_xpos_in[worldid, vert_adr2 + v2_indices[1]]
      cap2_pos = 0.5 * (p2_0 + p2_1)
      cap2_axis = wp.normalize(p2_1 - p2_0)
      cap2_half_len = 0.5 * wp.length(p2_1 - p2_0)

      if box1:
        contact_dist, contact_pos, box_nrms = collision_primitive_core.capsule_box(
          cap2_pos,
          cap2_axis,
          radius2,
          cap2_half_len,
          bpos1,
          bmat1,
          bsize1,
        )
        contact_normal = -box_nrms
      elif box2:
        contact_dist, contact_pos, contact_normal = collision_primitive_core.capsule_box(
          cap1_pos,
          cap1_axis,
          radius1,
          cap1_half_len,
          bpos2,
          bmat2,
          bsize2,
        )
      else:
        contact_dist, contact_pos, contact_normal = collision_primitive_core.capsule_capsule(
          cap1_pos,
          cap1_axis,
          radius1,
          cap1_half_len,
          cap2_pos,
          cap2_axis,
          radius2,
          cap2_half_len,
        )

      for c in range(2):
        d_val = contact_dist[c]
        if d_val <= mix_margin:
          _write_candidate(
            max_candidates,
            d_val,
            contact_pos[c],
            contact_normal[c],
            -2,
            flexid1,
            flexid2,
            e1,
            e2,
            worldid,
            wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
            overflow_out,
            cand_dist_out,
            cand_pos_out,
            cand_nrm_out,
            cand_geom_out,
            cand_flex_out,
            cand_elem_out,
            cand_vert_out,
            cand_worldid_out,
            ncand_out,
          )
    else:
      if pairid >= naccdmax_in:
        if wp.static(bool(warn_overflow & OverflowType.CCD)):
          wp.printf(
            "CCD overflow in flex narrowphase - please increase naccdmax beyond %u\n"
            "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.CCD (or = 0 for all)\n",
            naccdmax_in,
          )
        wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.CCD))
        return

      # GJK/EPA narrowphase
      offset2 = pairid * 8 + 4
      for idx in range(dim2 + 1):
        workspace_verts_out[offset2 + idx] = flexvert_xpos_in[worldid, vert_adr2 + v2_indices[idx]]

      geom1 = Geom()
      geom1.pos = wp.vec3(0.0)
      geom1.rot = wp.identity(n=3, dtype=float)
      geom1.size = wp.vec3(0.0)
      geom1.margin = 2.0 * radius1 + mix_margin
      geom1.vert = workspace_verts_out
      geom1.vertadr = offset1
      geom1.vertnum = dim1 + 1
      geom1.graphadr = -1
      geom1.index = -1

      geom2 = Geom()
      geom2.pos = wp.vec3(0.0)
      geom2.rot = wp.identity(n=3, dtype=float)
      geom2.size = wp.vec3(0.0)
      geom2.margin = 2.0 * radius2 + mix_margin
      geom2.vert = workspace_verts_out
      geom2.vertadr = offset2
      geom2.vertnum = dim2 + 1
      geom2.graphadr = -1
      geom2.index = -1

      center1 = wp.vec3(0.0)
      for idx in range(dim1 + 1):
        center1 += workspace_verts_out[offset1 + idx]
      center1 = center1 / float(dim1 + 1)

      center2 = wp.vec3(0.0)
      for idx in range(dim2 + 1):
        center2 += workspace_verts_out[offset2 + idx]
      center2 = center2 / float(dim2 + 1)

      tol = opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]]

      dist, ncontact, w1, w2, _ = ccd(
        tol,
        radius1 + radius2 + mix_margin,
        gjk_iterations,
        epa_iterations,
        geom1,
        geom2,
        int(GeomType.MESH),
        int(GeomType.MESH),
        center1,
        center2,
        epa_vert_out[pairid],
        epa_vert_index_out[pairid],
        epa_face_out[pairid],
        epa_pr_out[pairid],
        epa_norm2_out[pairid],
        epa_horizon_out[pairid],
        wp.static(bool(warn_overflow & OverflowType.EPA_HORIZON)),
        worldid,
        overflow_out,
      )

      if ncontact > 0 and dist < 0.0:
        phys_dist = dist + mix_margin
        pos = 0.5 * (w1 + w2)
        nrm = wp.normalize(w1 - w2)
        _write_candidate(
          max_candidates,
          phys_dist,
          pos,
          nrm,
          -2,
          flexid1,
          flexid2,
          e1,
          e2,
          worldid,
          wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          ncand_out,
        )

  return kernel


@cache_kernel
def _flex_narrowphase_elem_detect(warn_overflow: int, has_cylinder_geom: bool, has_ellipsoid_geom: bool):
  @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
  def kernel(
    # Model:
    ngeom: int,
    opt_ccd_tolerance: wp.array[float],
    qpos0: wp.array2d[float],
    jnt_qposadr: wp.array[int],
    geom_type: wp.array[int],
    geom_contype: wp.array[int],
    geom_conaffinity: wp.array[int],
    geom_bodyid: wp.array[int],
    geom_dataid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    geom_aabb: wp.array3d[wp.vec3],
    geom_rbound: wp.array2d[float],
    geom_margin: wp.array2d[float],
    geom_gap: wp.array2d[float],
    flex_contype: wp.array[int],
    flex_conaffinity: wp.array[int],
    flex_margin: wp.array[float],
    flex_gap: wp.array[float],
    flex_activelayers: wp.array[int],
    flex_dim: wp.array[int],
    flex_vertadr: wp.array[int],
    flex_elemadr: wp.array[int],
    flex_elemnum: wp.array[int],
    flex_elemdataadr: wp.array[int],
    flex_plasticadr: wp.array[int],
    flex_vertbodyid: wp.array[int],
    flex_elem: wp.array[int],
    flex_elemlayer: wp.array[int],
    flex_radius: wp.array[float],
    mesh_vertadr: wp.array[int],
    mesh_vertnum: wp.array[int],
    mesh_graphadr: wp.array[int],
    mesh_vert: wp.array[wp.vec3],
    mesh_graph: wp.array[int],
    mesh_polyadr: wp.array[int],
    mesh_polynormal: wp.array[wp.vec3],
    mesh_polyvertadr: wp.array[int],
    mesh_polyvertnum: wp.array[int],
    mesh_polyvert: wp.array[int],
    mesh_polymapadr: wp.array[int],
    mesh_polymapnum: wp.array[int],
    mesh_polymap: wp.array[int],
    flex_elemflexid: wp.array[int],
    flex_rod_is_box: wp.array[bool],
    flex_rod_root_clamped: wp.array[bool],
    flex_rod_tail_clamped: wp.array[bool],
    flex_rod_box_size: wp.array[wp.vec2],
    flex_rod_edgeadr: wp.array[int],
    flex_rod_edgenum: wp.array[int],
    flex_rodedge_vert: wp.array[wp.vec2i],
    flex_rodedge_jnt: wp.array[int],
    flex_elem_rodedge: wp.array[wp.vec2i],
    # Data in:
    qpos_in: wp.array2d[float],
    plastic_in: wp.array2d[float],
    xquat_in: wp.array2d[wp.quat],
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    naccdmax_in: int,
    flex_aabb_min_in: wp.array2d[wp.vec3],
    flex_aabb_max_in: wp.array2d[wp.vec3],
    flex_rod_frame0_loc_in: wp.array3d[wp.vec3],
    flex_rod_frame0_ref_in: wp.array3d[wp.vec3],
    flex_rod_tail_loc_in: wp.array3d[wp.vec3],
    # In:
    epa_vert: wp.array2d[wp.vec3],
    epa_vert_index: wp.array2d[int],
    epa_face: wp.array2d[int],
    epa_pr: wp.array2d[wp.vec3],
    epa_norm2: wp.array2d[float],
    epa_horizon: wp.array2d[int],
    nccd: wp.array[int],
    ccd_iterations: int,
    max_candidates: int,
    # Data out:
    overflow_out: wp.array[int],
    # Out:
    cand_dist_out: wp.array[float],
    cand_pos_out: wp.array[wp.vec3],
    cand_nrm_out: wp.array[wp.vec3],
    cand_geom_out: wp.array[wp.vec2i],
    cand_flex_out: wp.array[wp.vec2i],
    cand_elem_out: wp.array[wp.vec2i],
    cand_vert_out: wp.array[wp.vec2i],
    cand_worldid_out: wp.array[int],
    ncand_out: wp.array[int],
  ):
    worldid, elemid = wp.tid()

    flexid = flex_elemflexid[elemid]
    f_dim = flex_dim[flexid]

    # skip inactive elements of 3D flexes, see mj_isElemActive
    if f_dim == 3 and flex_elemlayer[elemid] >= flex_activelayers[flexid]:
      return

    local_elemid = elemid - flex_elemadr[flexid]
    vert_adr = flex_vertadr[flexid]
    elem_radius = flex_radius[flexid]
    elem_margin = flex_margin[flexid] + flex_gap[flexid]

    geom2 = Geom()
    geom2.pos = wp.vec3(0.0, 0.0, 0.0)
    geom2.rot = wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    geom2.margin = 0.0
    geom2.index = -1

    centroid = wp.vec3(0.0, 0.0, 0.0)
    r_elem = float(0.0)
    cap_axis = wp.vec3(0.0, 0.0, 1.0)
    geom2_type = int(GeomType.TRIANGLE)
    elem_min = wp.vec3(0.0, 0.0, 0.0)
    elem_max = wp.vec3(0.0, 0.0, 0.0)
    t1 = wp.vec3(0.0, 0.0, 0.0)
    t2 = wp.vec3(0.0, 0.0, 0.0)
    t3 = wp.vec3(0.0, 0.0, 0.0)
    p0 = wp.vec3(0.0, 0.0, 0.0)
    p1 = wp.vec3(0.0, 0.0, 0.0)
    p2 = wp.vec3(0.0, 0.0, 0.0)
    p3 = wp.vec3(0.0, 0.0, 0.0)
    v0 = int(-1)
    v1 = int(-1)
    v2 = int(-1)
    v3 = int(-1)
    ok_box = False
    bpos = wp.vec3(0.0, 0.0, 0.0)
    bmat = wp.mat33(0.0)
    bsize = wp.vec3(0.0, 0.0, 0.0)

    if f_dim == 1:
      edata_idx = flex_elemdataadr[flexid] + local_elemid * 2
      v0 = flex_elem[edata_idx]
      v1 = flex_elem[edata_idx + 1]

      t1 = flexvert_xpos_in[worldid, vert_adr + v0]
      t2 = flexvert_xpos_in[worldid, vert_adr + v1]

      centroid = 0.5 * (t1 + t2)
      seg = t1 - t2
      seg_len = wp.length(seg)
      r_elem = 0.5 * seg_len
      if seg_len > MJ_MINVAL:
        cap_axis = seg / seg_len
      b_ax, c_ax = orthogonals(cap_axis)

      geom2.pos = centroid
      geom2.rot = wp.matrix_from_cols(b_ax, c_ax, cap_axis)
      geom2.size = wp.vec3(elem_radius, r_elem, 0.0)
      geom2_type = int(GeomType.CAPSULE)

      ok_box, bpos, bmat, bsize = support.rod_box_elem(
        qpos0,
        jnt_qposadr,
        flex_vertadr,
        flex_elemadr,
        flex_elemnum,
        flex_elemdataadr,
        flex_plasticadr,
        flex_vertbodyid,
        flex_elem,
        flex_rod_is_box,
        flex_rod_root_clamped,
        flex_rod_tail_clamped,
        flex_rod_box_size,
        flex_rod_edgeadr,
        flex_rod_edgenum,
        flex_rodedge_vert,
        flex_rodedge_jnt,
        flex_elem_rodedge,
        qpos_in,
        plastic_in,
        xquat_in,
        flexvert_xpos_in,
        flex_rod_frame0_loc_in,
        flex_rod_frame0_ref_in,
        flex_rod_tail_loc_in,
        worldid,
        flexid,
        local_elemid,
      )
      pad = elem_radius
      if ok_box:
        pad = wp.max(pad, wp.sqrt(bsize[0] * bsize[0] + bsize[1] * bsize[1]))

      elem_min = wp.min(t1, t2) - wp.vec3(pad, pad, pad)
      elem_max = wp.max(t1, t2) + wp.vec3(pad, pad, pad)
    elif f_dim == 2:
      edata_idx = flex_elemdataadr[flexid] + local_elemid * 3
      v0 = flex_elem[edata_idx]
      v1 = flex_elem[edata_idx + 1]
      v2 = flex_elem[edata_idx + 2]

      t1 = flexvert_xpos_in[worldid, vert_adr + v0]
      t2 = flexvert_xpos_in[worldid, vert_adr + v1]
      t3 = flexvert_xpos_in[worldid, vert_adr + v2]

      geom2.rot = wp.mat33(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], t3[0], t3[1], t3[2])
      geom2_type = int(GeomType.TRIANGLE)
      centroid = (t1 + t2 + t3) * wp.static(1.0 / 3.0)
      d1 = wp.length(t1 - centroid)
      d2 = wp.length(t2 - centroid)
      d3 = wp.length(t3 - centroid)
      r_elem = wp.max(d1, wp.max(d2, d3))

      elem_min = wp.min(t1, wp.min(t2, t3)) - wp.vec3(elem_radius, elem_radius, elem_radius)
      elem_max = wp.max(t1, wp.max(t2, t3)) + wp.vec3(elem_radius, elem_radius, elem_radius)
    elif f_dim == 3:
      edata_idx = flex_elemdataadr[flexid] + local_elemid * 4
      v0 = flex_elem[edata_idx]
      v1 = flex_elem[edata_idx + 1]
      v2 = flex_elem[edata_idx + 2]
      v3 = flex_elem[edata_idx + 3]

      p0 = flexvert_xpos_in[worldid, vert_adr + v0]
      p1 = flexvert_xpos_in[worldid, vert_adr + v1]
      p2 = flexvert_xpos_in[worldid, vert_adr + v2]
      p3 = flexvert_xpos_in[worldid, vert_adr + v3]

      geom2.polyvert = mat63(
        p0[0],
        p0[1],
        p0[2],
        p1[0],
        p1[1],
        p1[2],
        p2[0],
        p2[1],
        p2[2],
        p3[0],
        p3[1],
        p3[2],
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
      )
      geom2_type = int(GeomType.FLEX)
      geom2.margin = 2.0 * elem_radius
      centroid = (p0 + p1 + p2 + p3) * wp.static(0.25)
      d0 = wp.length(p0 - centroid)
      d1 = wp.length(p1 - centroid)
      d2 = wp.length(p2 - centroid)
      d3 = wp.length(p3 - centroid)
      r_elem = wp.max(d0, wp.max(d1, wp.max(d2, d3)))

      elem_min = wp.min(p0, wp.min(p1, p2))
      elem_min = wp.min(elem_min, p3) - wp.vec3(elem_radius, elem_radius, elem_radius)
      elem_max = wp.max(p0, wp.max(p1, p2))
      elem_max = wp.max(elem_max, p3) + wp.vec3(elem_radius, elem_radius, elem_radius)
    else:
      return

    b0 = flex_vertbodyid[vert_adr + v0]
    b1 = flex_vertbodyid[vert_adr + v1]
    b2 = int(-1)
    b3 = int(-1)
    if f_dim >= 2:
      b2 = flex_vertbodyid[vert_adr + v2]
    if f_dim == 3:
      b3 = flex_vertbodyid[vert_adr + v3]

    f_contype = flex_contype[flexid]
    f_conaffinity = flex_conaffinity[flexid]
    flex_aabb_min_val = flex_aabb_min_in[worldid, flexid]
    flex_aabb_max_val = flex_aabb_max_in[worldid, flexid]

    for geomid in range(ngeom):
      gtype = geom_type[geomid]
      if (
        gtype != int(GeomType.SPHERE)
        and gtype != int(GeomType.CAPSULE)
        and gtype != int(GeomType.BOX)
        and gtype != int(GeomType.CYLINDER)
        and gtype != int(GeomType.ELLIPSOID)
        and gtype != int(GeomType.MESH)
      ):
        continue

      g_contype = geom_contype[geomid]
      g_conaffinity = geom_conaffinity[geomid]
      if not ((g_contype & f_conaffinity) or (f_contype & g_conaffinity)):
        continue

      # skip if element has vertices on the same body as geom
      b = geom_bodyid[geomid]
      if b >= 0 and (b == b0 or b == b1 or b == b2 or b == b3):
        continue

      geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid] + geom_gap[worldid % geom_gap.shape[0], geomid]
      margin = geom_margin_val + elem_margin

      geom_pos = geom_xpos_in[worldid, geomid]
      geom_rot = quat_to_mat(geom_xquat_in[worldid, geomid])
      geom_size_val = geom_size[worldid % geom_size.shape[0], geomid]

      # Stage 1: Coarse flex object AABB vs Geom world AABB check
      aabb_id = worldid % geom_aabb.shape[0]
      geom_center_local = geom_aabb[aabb_id, geomid, 0]
      geom_half_size_local = geom_aabb[aabb_id, geomid, 1]
      geom_center_global = geom_rot @ geom_center_local + geom_pos
      geom_half_size_global = wp.vec3(
        wp.abs(geom_rot[0, 0]) * geom_half_size_local[0]
        + wp.abs(geom_rot[0, 1]) * geom_half_size_local[1]
        + wp.abs(geom_rot[0, 2]) * geom_half_size_local[2],
        wp.abs(geom_rot[1, 0]) * geom_half_size_local[0]
        + wp.abs(geom_rot[1, 1]) * geom_half_size_local[1]
        + wp.abs(geom_rot[1, 2]) * geom_half_size_local[2],
        wp.abs(geom_rot[2, 0]) * geom_half_size_local[0]
        + wp.abs(geom_rot[2, 1]) * geom_half_size_local[1]
        + wp.abs(geom_rot[2, 2]) * geom_half_size_local[2],
      )
      inflate = wp.vec3(margin, margin, margin)
      geom_box_min = geom_center_global - geom_half_size_global - inflate
      geom_box_max = geom_center_global + geom_half_size_global + inflate

      if _flex_element_aabb_filter(geom_box_min, geom_box_max, flex_aabb_min_val, flex_aabb_max_val):
        continue

      # Stage 2: Element AABB vs Geom world AABB check
      if _flex_element_aabb_filter(geom_box_min, geom_box_max, elem_min, elem_max):
        continue

      # sphere/capsule : tetrahedron, analytic (falls back to ccd for degenerate tetrahedra)
      if f_dim == 3 and (gtype == int(GeomType.SPHERE) or gtype == int(GeomType.CAPSULE)):
        cap_len = wp.where(gtype == int(GeomType.CAPSULE), geom_size_val[1], 0.0)
        cap_axis_g = geom_rot[:, 2]
        valid, cdist, cpos, cnrm = collision_primitive_core.capsule_tetrahedron(
          geom_pos - cap_len * cap_axis_g, geom_pos + cap_len * cap_axis_g, geom_size_val[0], p0, p1, p2, p3, elem_radius
        )
        if valid:
          if cdist < margin:
            _write_candidate(
              max_candidates,
              cdist,
              cpos,
              cnrm,
              geomid,
              -1,
              flexid,
              local_elemid,
              -1,
              worldid,
              wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
              overflow_out,
              cand_dist_out,
              cand_pos_out,
              cand_nrm_out,
              cand_geom_out,
              cand_flex_out,
              cand_elem_out,
              cand_vert_out,
              cand_worldid_out,
              ncand_out,
            )
          continue

      if gtype == int(GeomType.MESH):
        r_geom = geom_rbound[worldid % geom_rbound.shape[0], geomid]
        if wp.length(centroid - geom_pos) > r_geom + r_elem + margin + elem_radius:
          continue

        ccdid = wp.atomic_add(nccd, 0, 1)  # kernel_analyzer: ignore[atomic]
        if ccdid >= naccdmax_in:
          if wp.static(bool(warn_overflow & OverflowType.CCD)):
            wp.printf(
              "CCD overflow in flex narrowphase - please increase naccdmax beyond %u\n"
              "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.CCD (or = 0 for all)\n",
              naccdmax_in,
            )
          wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.CCD))
          continue

        did = geom_dataid[worldid % geom_dataid.shape[0], geomid]
        tolerance = opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]]

        if f_dim == 2:
          geom2.margin = 2.0 * elem_radius
        _collide_mesh_convex(
          mesh_vertadr,
          mesh_vertnum,
          mesh_graphadr,
          mesh_vert,
          mesh_graph,
          mesh_polyadr,
          mesh_polynormal,
          mesh_polyvertadr,
          mesh_polyvertnum,
          mesh_polyvert,
          mesh_polymapadr,
          mesh_polymapnum,
          mesh_polymap,
          max_candidates,
          geom_pos,
          geom_rot,
          geom_size_val,
          r_geom,
          did,
          geom2,
          geom2_type,
          centroid,
          margin,
          geomid,
          flexid,
          local_elemid,
          -1,
          worldid,
          epa_vert[ccdid],
          epa_vert_index[ccdid],
          epa_face[ccdid],
          epa_pr[ccdid],
          epa_norm2[ccdid],
          epa_horizon[ccdid],
          tolerance,
          ccd_iterations,
          wp.static(bool(warn_overflow & OverflowType.EPA_HORIZON)),
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          ncand_out,
        )
        if f_dim == 2:
          geom2.margin = 0.0

      elif f_dim == 1 and (gtype == int(GeomType.SPHERE) or gtype == int(GeomType.CAPSULE) or gtype == int(GeomType.BOX)):
        if ok_box:
          _collide_geom_box_rod_detect(
            max_candidates,
            gtype,
            geom_pos,
            geom_rot,
            geom_size_val,
            bpos,
            bmat,
            bsize,
            margin,
            geomid,
            flexid,
            local_elemid,
            -1,
            worldid,
            wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
            overflow_out,
            cand_dist_out,
            cand_pos_out,
            cand_nrm_out,
            cand_geom_out,
            cand_flex_out,
            cand_elem_out,
            cand_vert_out,
            cand_worldid_out,
            ncand_out,
          )
        else:
          _collide_geom_capsule_detect(
            max_candidates,
            gtype,
            geom_pos,
            geom_rot,
            geom_size_val,
            centroid,
            cap_axis,
            elem_radius,
            r_elem,
            margin,
            geomid,
            flexid,
            local_elemid,
            -1,
            worldid,
            wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
            overflow_out,
            cand_dist_out,
            cand_pos_out,
            cand_nrm_out,
            cand_geom_out,
            cand_flex_out,
            cand_elem_out,
            cand_vert_out,
            cand_worldid_out,
            ncand_out,
          )

      elif f_dim == 2 and (gtype == int(GeomType.SPHERE) or gtype == int(GeomType.CAPSULE) or gtype == int(GeomType.BOX)):
        _collide_geom_triangle_detect(
          max_candidates,
          gtype,
          geom_pos,
          geom_rot,
          geom_size_val,
          t1,
          t2,
          t3,
          elem_radius,
          margin,
          geomid,
          flexid,
          local_elemid,
          -1,
          worldid,
          wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
          overflow_out,
          cand_dist_out,
          cand_pos_out,
          cand_nrm_out,
          cand_geom_out,
          cand_flex_out,
          cand_elem_out,
          cand_vert_out,
          cand_worldid_out,
          ncand_out,
        )

      else:
        ccdid = wp.atomic_add(nccd, 0, 1)  # kernel_analyzer: ignore[atomic]
        if ccdid >= naccdmax_in:
          if wp.static(bool(warn_overflow & OverflowType.CCD)):
            wp.printf(
              "CCD overflow in flex narrowphase - please increase naccdmax beyond %u\n"
              "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.CCD (or = 0 for all)\n",
              naccdmax_in,
            )
          wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.CCD))
          continue

        tolerance = opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]]

        geom1 = Geom()
        geom1.pos = geom_pos
        geom1.rot = geom_rot
        geom1.size = geom_size_val
        geom1.margin = 0.0
        geom1.index = -1

        r_geom = wp.length(geom_size_val)
        if wp.length(centroid - geom_pos) <= r_geom + r_elem + margin + elem_radius + 0.04:
          # cylinder/ellipsoid : triangle sees the triangle swept by the flex radius, as MuJoCo C's
          # support does (deep penetrations of the bare triangle give in-plane escape directions)
          swept = False
          if wp.static(bool(has_cylinder_geom or has_ellipsoid_geom)):
            swept = f_dim == 2 and (gtype == int(GeomType.CYLINDER) or gtype == int(GeomType.ELLIPSOID))
          if swept:
            geom2.margin = 2.0 * elem_radius
          ccd_cutoff = (margin + elem_radius) if (f_dim == 2 and not swept) else margin
          dist, ncontact, w1, w2, idx = ccd(
            tolerance,
            ccd_cutoff,
            ccd_iterations,
            ccd_iterations,
            geom1,
            geom2,
            gtype,
            geom2_type,
            geom_pos,
            centroid,
            epa_vert[ccdid],
            epa_vert_index[ccdid],
            epa_face[ccdid],
            epa_pr[ccdid],
            epa_norm2[ccdid],
            epa_horizon[ccdid],
            wp.static(bool(warn_overflow & OverflowType.EPA_HORIZON)),
            worldid,
            overflow_out,
          )

          cand_dist = (dist - elem_radius) if (f_dim == 2 and not swept) else dist
          if swept:
            geom2.margin = 0.0
          if ncontact > 0 and cand_dist < margin:
            diff = (w1 - w2) if dist < 0.0 else (w2 - w1)
            normal = wp.normalize(diff)
            if wp.length_sq(diff) < 1e-12:
              normal = wp.normalize(centroid - geom_pos)

            contact_pos = 0.5 * (w1 + w2)
            if f_dim == 2:
              if swept:
                if wp.static(bool(has_cylinder_geom)):
                  if gtype == int(GeomType.CYLINDER):
                    normal = _fix_cylinder_normal(geom_pos, geom_rot, geom_size_val, contact_pos, normal)
                if wp.static(bool(has_ellipsoid_geom)):
                  if gtype == int(GeomType.ELLIPSOID):
                    normal = _fix_ellipsoid_normal(geom_pos, geom_rot, geom_size_val, contact_pos, normal)
              else:
                contact_pos -= 0.5 * elem_radius * normal

            _write_candidate(
              max_candidates,
              cand_dist,
              contact_pos,
              normal,
              geomid,
              -1,
              flexid,
              local_elemid,
              -1,
              worldid,
              wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)),
              overflow_out,
              cand_dist_out,
              cand_pos_out,
              cand_nrm_out,
              cand_geom_out,
              cand_flex_out,
              cand_elem_out,
              cand_vert_out,
              cand_worldid_out,
              ncand_out,
            )

  return kernel


@wp.kernel
def _compute_filter_key(
  # Model:
  ngeom: int,
  nflex: int,
  # In:
  ncand: wp.array[int],
  cand_geom: wp.array[wp.vec2i],
  cand_flex: wp.array[wp.vec2i],
  cand_pos: wp.array[wp.vec3],
  cand_worldid: wp.array[int],
  # Out:
  key_out: wp.array[wp.int64],
  val_out: wp.array[int],
):
  """Compute sort key for candidate grouping.

  Groups candidates by (worldid, flex_id, geom_id) in high bits and spatial projection in low bits.
  """
  i = wp.tid()
  if i >= ncand[0]:
    key_out[i] = wp.int64(9223372036854775807)  # (largest int64 value): sort unused entries to end
    val_out[i] = i
    return

  worldid = cand_worldid[i]
  flex_id = cand_flex[i][1]
  geom_id = cand_geom[i][0]
  flex1 = cand_flex[i][0]

  if geom_id >= 0:
    group = geom_id * nflex + flex_id
  else:
    f1 = wp.min(flex1, flex_id)
    f2 = wp.max(flex1, flex_id)
    group = ngeom * nflex + f1 * nflex + f2

  group_id = worldid * (ngeom * nflex + nflex * nflex) + group
  is_self = int(flex1 == flex_id) if geom_id < 0 else 0

  group_key = (wp.int64(group_id) << wp.int64(1)) | wp.int64(is_self)

  # Spatial projection key: project position onto diagonal vector u = (1, 1, 1) / sqrt(3).
  # Clamped monotonic 1D integer mapping with 1 um resolution.
  p = cand_pos[i]
  s = (p[0] + p[1] + p[2]) * wp.static(1.0 / math.sqrt(3.0))
  spatial_key = wp.clamp(wp.int64(s * 1000000.0) + wp.int64(100000000), wp.int64(0), wp.int64(2147483647))

  key_out[i] = (group_key << wp.int64(32)) | spatial_key
  val_out[i] = i


@wp.func
def _compare_candidate(
  # In:
  dist_i: float,
  dist_j: float,
  cand_idx_i: int,
  cand_idx_j: int,
) -> bool:
  """Returns True if candidate i is dominated by (and should yield to) candidate j."""
  if dist_j < dist_i:
    return True
  if dist_j == dist_i and cand_idx_j < cand_idx_i:
    return True
  return False


@wp.func
def _compare_candidate_canonical(
  # In:
  dist_i: float,
  dist_j: float,
  pos_i: wp.vec3,
  pos_j: wp.vec3,
  cand_idx_i: int,
  cand_idx_j: int,
  cand_nrm: wp.array[wp.vec3],
  cand_flex: wp.array[wp.vec2i],
  cand_elem: wp.array[wp.vec2i],
  cand_vert: wp.array[wp.vec2i],
) -> bool:
  """Returns True if candidate i is dominated by j, breaking ties by candidate content."""
  dist_key_i = sort.float_key(wp.cast(dist_i, wp.uint32))
  dist_key_j = sort.float_key(wp.cast(dist_j, wp.uint32))
  if dist_key_j != dist_key_i:
    return dist_key_j < dist_key_i
  # Break exact distance ties canonically by candidate content before raw atomic index.
  c = _cmp_vec2i(cand_flex[cand_idx_j], cand_flex[cand_idx_i])
  if c != 0:
    return c < 0
  c = _cmp_vec2i(cand_elem[cand_idx_j], cand_elem[cand_idx_i])
  if c != 0:
    return c < 0
  c = _cmp_vec2i(cand_vert[cand_idx_j], cand_vert[cand_idx_i])
  if c != 0:
    return c < 0
  c = _cmp_vec3f(pos_j, pos_i)
  if c != 0:
    return c < 0
  c = _cmp_vec3f(cand_nrm[cand_idx_j], cand_nrm[cand_idx_i])
  if c != 0:
    return c < 0
  return cand_idx_j < cand_idx_i


# Note: MuJoCo Warp uses proximity deduplication (1 mm threshold) only for 1D and 2D
# flex-geom collisions and ContactType.PASSIVE flex pairs. Non-passive 3D flex-geom,
# flex self-collision, and flex-flex collisions apply proximity deduplication before
# Farthest Point Sampling (FPS) limited to MJ_MAXCONPAIR.
# TODO(thowell): Figure out an FPS version that utilizes a configurable mj_maxconpair.
@cache_kernel
def _filter_flex_candidates_sorted(has_passive_flex: bool, canonical_ties: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_integrator: int,
    body_weldid: wp.array[int],
    geom_bodyid: wp.array[int],
    flex_passive: wp.array[int],
    flex_dim: wp.array[int],
    flex_interp: wp.array[int],
    flex_rigid: wp.array[bool],
    # In:
    ncand: wp.array[int],
    epsilon: float,
    sort_key: wp.array[wp.int64],
    sort_val: wp.array[int],
    cand_dist: wp.array[float],
    cand_pos: wp.array[wp.vec3],
    cand_nrm: wp.array[wp.vec3],
    cand_geom: wp.array[wp.vec2i],
    cand_flex: wp.array[wp.vec2i],
    cand_elem: wp.array[wp.vec2i],
    cand_vert: wp.array[wp.vec2i],
    # Out:
    cand_active_out: wp.array[int],
  ):
    """Filter duplicate candidates using sorted order.

    After sorting by group key, candidates in the same group are contiguous.
    Each candidate only compares with neighbors sharing the same key, reducing
    complexity from O(n^2) to O(n * k) where k is the average group size.
    """
    si = wp.tid()
    ncand_limit = wp.min(ncand[0], cand_active_out.shape[0])
    if si >= ncand_limit:
      if si < cand_active_out.shape[0]:
        cand_active_out[sort_val[si]] = 0
      return

    i = sort_val[si]
    my_key = sort_key[si]
    my_group = my_key >> wp.int64(32)
    my_spatial = my_key & wp.int64(0x7FFFFFFF)
    eps_key = wp.int64(wp.ceil(epsilon * 1000000.0)) + wp.int64(1)
    pos_i = cand_pos[i]
    dist_i = cand_dist[i]
    eps2 = epsilon * epsilon
    is_passive = False
    elem_i = wp.vec2i(-1, -1)
    vert_i = wp.vec2i(-1, -1)
    if wp.static(has_passive_flex):
      is_passive = _is_passive_flex_cand(
        opt_integrator,
        body_weldid,
        geom_bodyid,
        flex_passive,
        flex_dim,
        flex_interp,
        flex_rigid,
        cand_geom,
        cand_flex,
        i,
      )
      if is_passive:
        elem_i = cand_elem[i]
        vert_i = cand_vert[i]

    keep = int(1)

    # Compare with same-key neighbors (backward)
    j = si - 1
    while j >= 0:
      key_j = sort_key[j]
      if (key_j >> wp.int64(32)) != my_group:
        break
      spatial_j = key_j & wp.int64(0x7FFFFFFF)
      if my_spatial - spatial_j >= eps_key:
        break

      oj = sort_val[j]
      if not is_passive or (cand_elem[oj] == elem_i and cand_vert[oj] == vert_i):
        pos_j = cand_pos[oj]
        diff = pos_i - pos_j
        if wp.dot(diff, diff) < eps2:
          if wp.static(canonical_ties):
            if _compare_candidate_canonical(
              dist_i, cand_dist[oj], pos_i, pos_j, i, oj, cand_nrm, cand_flex, cand_elem, cand_vert
            ):
              keep = 0
              break
          elif _compare_candidate(dist_i, cand_dist[oj], i, oj):
            keep = 0
            break
      j -= 1

    # Compare with same-key neighbors (forward)
    if keep == 1:
      j = si + 1
      while j < ncand_limit:
        key_j = sort_key[j]
        if (key_j >> wp.int64(32)) != my_group:
          break
        spatial_j = key_j & wp.int64(0x7FFFFFFF)
        if spatial_j - my_spatial >= eps_key:
          break

        oj = sort_val[j]
        if not is_passive or (cand_elem[oj] == elem_i and cand_vert[oj] == vert_i):
          pos_j = cand_pos[oj]
          diff = pos_i - pos_j
          if wp.dot(diff, diff) < eps2:
            if wp.static(canonical_ties):
              if _compare_candidate_canonical(
                dist_i, cand_dist[oj], pos_i, pos_j, i, oj, cand_nrm, cand_flex, cand_elem, cand_vert
              ):
                keep = 0
                break
            elif _compare_candidate(dist_i, cand_dist[oj], i, oj):
              keep = 0
              break
        j += 1

    cand_active_out[i] = keep

  return kernel


@cache_kernel
def _write_filtered_contacts(warn_overflow: int, has_passive_flex: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_integrator: int,
    body_weldid: wp.array[int],
    geom_condim: wp.array[int],
    geom_bodyid: wp.array[int],
    geom_priority: wp.array[int],
    geom_solmix: wp.array2d[float],
    geom_solref: wp.array2d[wp.vec2],
    geom_solimp: wp.array2d[vec5],
    geom_friction: wp.array2d[wp.vec3],
    geom_margin: wp.array2d[float],
    geom_gap: wp.array2d[float],
    flex_condim: wp.array[int],
    flex_priority: wp.array[int],
    flex_solmix: wp.array[float],
    flex_solref: wp.array[wp.vec2],
    flex_solimp: wp.array[vec5],
    flex_friction: wp.array[wp.vec3],
    flex_margin: wp.array[float],
    flex_gap: wp.array[float],
    flex_passive: wp.array[int],
    flex_dim: wp.array[int],
    flex_interp: wp.array[int],
    flex_rigid: wp.array[bool],
    # Data in:
    naconmax_in: int,
    # In:
    ncand: wp.array[int],
    cand_dist: wp.array[float],
    cand_pos: wp.array[wp.vec3],
    cand_nrm: wp.array[wp.vec3],
    cand_geom: wp.array[wp.vec2i],
    cand_flex: wp.array[wp.vec2i],
    cand_elem: wp.array[wp.vec2i],
    cand_vert: wp.array[wp.vec2i],
    cand_worldid: wp.array[int],
    cand_active: wp.array[int],
    # Data out:
    contact_dist_out: wp.array[float],
    contact_pos_out: wp.array[wp.vec3],
    contact_frame_out: wp.array[wp.mat33],
    contact_includemargin_out: wp.array[float],
    contact_friction_out: wp.array[vec5],
    contact_solref_out: wp.array[wp.vec2],
    contact_solreffriction_out: wp.array[wp.vec2],
    contact_solimp_out: wp.array[vec5],
    contact_dim_out: wp.array[int],
    contact_geom_out: wp.array[wp.vec2i],
    contact_flex_out: wp.array[wp.vec2i],
    contact_elem_out: wp.array[wp.vec2i],
    contact_vert_out: wp.array[wp.vec2i],
    contact_efc_address_out: wp.array2d[int],
    contact_worldid_out: wp.array[int],
    contact_type_out: wp.array[int],
    contact_geomcollisionid_out: wp.array[int],
    nacon_out: wp.array[int],
    overflow_out: wp.array[int],
  ):
    i = wp.tid()
    if i >= ncand[0]:
      return

    if cand_active[i] == 0:
      return

    geomid = cand_geom[i][0]
    worldid = cand_worldid[i]

    condim = int(0)
    margin = float(0.0)
    gap = float(0.0)
    solref = wp.vec2(0.0, 0.0)
    solimp = vec5(0.0, 0.0, 0.0, 0.0, 0.0)
    friction = vec5(0.0, 0.0, 0.0, 0.0, 0.0)

    if geomid >= 0:
      flexid = cand_flex[i][1]
      geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
      tri_margin = flex_margin[flexid]
      margin = geom_margin_val + tri_margin

      condim, gap, solref, solimp, friction = _mix_flex_contact_params(
        geom_condim[geomid],
        geom_priority[geomid],
        geom_solmix[worldid % geom_solmix.shape[0], geomid],
        geom_solref[worldid % geom_solref.shape[0], geomid],
        geom_solimp[worldid % geom_solimp.shape[0], geomid],
        geom_friction[worldid % geom_friction.shape[0], geomid],
        geom_gap[worldid % geom_gap.shape[0], geomid],
        flex_condim[flexid],
        flex_priority[flexid],
        flex_solmix[flexid],
        flex_solref[flexid],
        flex_solimp[flexid],
        flex_friction[flexid],
        flex_gap[flexid],
      )
    else:
      flex1 = cand_flex[i][0]
      flex2 = cand_flex[i][1]
      is_self = flex1 == flex2
      if is_self:
        margin = float(0.0)
        gap1 = float(0.0)
        gap2 = float(0.0)
      else:
        margin = flex_margin[flex1] + flex_margin[flex2]
        gap1 = flex_gap[flex1]
        gap2 = flex_gap[flex2]
      mixed_condim, gap, solref, solimp, friction = _mix_flex_contact_params(
        flex_condim[flex1],
        flex_priority[flex1],
        flex_solmix[flex1],
        flex_solref[flex1],
        flex_solimp[flex1],
        flex_friction[flex1],
        gap1,
        flex_condim[flex2],
        flex_priority[flex2],
        flex_solmix[flex2],
        flex_solref[flex2],
        flex_solimp[flex2],
        flex_friction[flex2],
        gap2,
      )

      if cand_vert[i][0] >= 0 and flex_dim[flex1] == 3:
        condim = 1
      else:
        condim = mixed_condim

    # contacts beyond the margin are inactive, see mj_collision
    if cand_dist[i] >= margin + gap:
      return

    id_ = wp.atomic_add(nacon_out, 0, 1)  # kernel_analyzer: ignore[atomic]
    if id_ >= naconmax_in:
      if wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)):
        wp.printf(
          "flex contact overflow - please increase naconmax beyond %u\n"
          "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.NARROWPHASE (or = 0 for all)\n",
          naconmax_in,
        )
      wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.NARROWPHASE))
      return

    contact_dist_out[id_] = cand_dist[i]
    contact_pos_out[id_] = cand_pos[i]
    contact_frame_out[id_] = make_frame(cand_nrm[i])
    contact_includemargin_out[id_] = margin
    contact_friction_out[id_] = friction
    contact_solref_out[id_] = solref
    contact_solreffriction_out[id_] = solref
    contact_solimp_out[id_] = solimp
    contact_dim_out[id_] = condim
    contact_geom_out[id_] = cand_geom[i]
    contact_flex_out[id_] = cand_flex[i]
    contact_elem_out[id_] = cand_elem[i]
    contact_vert_out[id_] = cand_vert[i]
    contact_worldid_out[id_] = cand_worldid[i]
    # clear the rows of whatever contact last used this slot (constraint construction writes only
    # this contact's dims, and the Jacobian kernels skip -1 entries)
    for k in range(contact_efc_address_out.shape[1]):
      contact_efc_address_out[id_, k] = -1
    is_passive = False
    if wp.static(has_passive_flex):
      is_passive = _is_passive_flex_cand(
        opt_integrator,
        body_weldid,
        geom_bodyid,
        flex_passive,
        flex_dim,
        flex_interp,
        flex_rigid,
        cand_geom,
        cand_flex,
        i,
      )
    contact_type_out[id_] = int(ContactType.PASSIVE) if is_passive else int(ContactType.CONSTRAINT)
    contact_geomcollisionid_out[id_] = 0

  return kernel


@wp.kernel
def _populate_active_sorted(
  # In:
  ncand: wp.array[int],
  sort_val: wp.array[int],
  cand_active: wp.array[int],
  # Out:
  cand_active_sorted_out: wp.array[int],
):
  si = wp.tid()
  ncand_limit = wp.min(ncand[0], cand_active_sorted_out.shape[0])
  if si < ncand_limit:
    cand_active_sorted_out[si] = cand_active[sort_val[si]]
  else:
    if si < cand_active_sorted_out.shape[0]:
      cand_active_sorted_out[si] = 0


@wp.kernel
def _find_group_starts(
  # In:
  ncand: wp.array[int],
  sort_key: wp.array[wp.int64],
  # Out:
  flex_group_temp_out: wp.array[int],
):
  si = wp.tid()
  ncand_limit = wp.min(ncand[0], flex_group_temp_out.shape[0])
  if si >= ncand_limit:
    return

  if si == 0:
    flex_group_temp_out[si] = 1
  else:
    my_key = sort_key[si]
    my_group = my_key >> wp.int64(32)
    prev_key = sort_key[si - 1]
    prev_group = prev_key >> wp.int64(32)
    if my_group != prev_group:
      flex_group_temp_out[si] = 1
    else:
      flex_group_temp_out[si] = 0


@cache_kernel
def _populate_group_starts(warn_overflow: int):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # In:
    flex_group_temp_in: wp.array[int],
    flex_group_ids_in: wp.array[int],
    ncand: wp.array[int],
    filter_val_in: wp.array[int],
    cand_worldid: wp.array[int],
    # Data out:
    overflow_out: wp.array[int],
    # Out:
    flex_group_start_indices_out: wp.array[int],
    flex_num_groups_out: wp.array[int],
  ):
    si = wp.tid()
    ncand_limit = wp.min(ncand[0], flex_group_temp_in.shape[0])
    if si >= ncand_limit:
      return

    if flex_group_temp_in[si] == 1:
      g = flex_group_ids_in[si] - 1
      if g >= flex_group_start_indices_out.shape[0]:
        if wp.static(bool(warn_overflow & OverflowType.NARROWPHASE)):
          wp.printf(
            "flex candidate group overflow - please increase naconmax beyond %u\n"
            "To disable the print warning: m.opt.warn_overflow &= ~mjw.OverflowType.NARROWPHASE (or = 0 for all)\n",
            flex_group_start_indices_out.shape[0],
          )
        worldid = cand_worldid[filter_val_in[si]]
        wp.atomic_or(overflow_out, worldid, wp.static(OverflowType.NARROWPHASE))
      else:
        flex_group_start_indices_out[g] = si

    if si == ncand_limit - 1:
      flex_num_groups_out[0] = flex_group_ids_in[si]

  return kernel


@wp.func
def _tie_break_fps(
  # In:
  curr_idx: int,
  sel_idx: int,
  cand_elem: wp.array[wp.vec2i],
  cand_vert: wp.array[wp.vec2i],
):
  if sel_idx < 0:
    return True

  elem_curr = cand_elem[curr_idx]
  elem_sel = cand_elem[sel_idx]
  if elem_curr[0] != elem_sel[0]:
    return elem_curr[0] < elem_sel[0]
  if elem_curr[1] != elem_sel[1]:
    return elem_curr[1] < elem_sel[1]

  vert_curr = cand_vert[curr_idx]
  vert_sel = cand_vert[sel_idx]
  if vert_curr[0] != vert_sel[0]:
    return vert_curr[0] < vert_sel[0]
  if vert_curr[1] != vert_sel[1]:
    return vert_curr[1] < vert_sel[1]
  return curr_idx < sel_idx


@wp.func
def _tie_break_fps_canonical(
  # In:
  curr_idx: int,
  sel_idx: int,
  cand_pos: wp.array[wp.vec3],
  cand_elem: wp.array[wp.vec2i],
  cand_vert: wp.array[wp.vec2i],
) -> bool:
  if sel_idx < 0:
    return True
  c = _cmp_vec2i(cand_elem[curr_idx], cand_elem[sel_idx])
  if c != 0:
    return c < 0
  c = _cmp_vec2i(cand_vert[curr_idx], cand_vert[sel_idx])
  if c != 0:
    return c < 0
  c = _cmp_vec3f(cand_pos[curr_idx], cand_pos[sel_idx])
  if c != 0:
    return c < 0
  return curr_idx < sel_idx


_fps_vecf = wp.types.vector(length=_FPS_SLOTS, dtype=float)
_fps_veci = wp.types.vector(length=_FPS_SLOTS, dtype=int)


@wp.struct
class _FpsPick:
  dist: float
  cidx: int
  elem: wp.vec2i
  vert: wp.vec2i


@wp.struct
class _FpsPickCanonical:
  dist: float
  cidx: int
  elem: wp.vec2i
  vert: wp.vec2i
  pos: wp.vec3


@wp.func
def _fps_pick(dist: float, cidx: int, cand_elem: wp.array[wp.vec2i], cand_vert: wp.array[wp.vec2i]) -> _FpsPick:
  pick = _FpsPick()
  pick.dist = dist
  pick.cidx = cidx
  if cidx >= 0:
    pick.elem = cand_elem[cidx]
    pick.vert = cand_vert[cidx]
  return pick


@wp.func
def _fps_pick_canonical(
  # In:
  dist: float,
  cidx: int,
  cand_pos: wp.array[wp.vec3],
  cand_elem: wp.array[wp.vec2i],
  cand_vert: wp.array[wp.vec2i],
) -> _FpsPickCanonical:
  pick = _FpsPickCanonical()
  pick.dist = dist
  pick.cidx = cidx
  if cidx >= 0:
    pick.elem = cand_elem[cidx]
    pick.vert = cand_vert[cidx]
    pick.pos = cand_pos[cidx]
  return pick


@wp.func
def _fps_pick_max(a: _FpsPick, b: _FpsPick) -> _FpsPick:
  """Returns the pick with the larger distance, breaking exact ties like _tie_break_fps."""
  if b.cidx < 0:
    return a
  if a.cidx < 0:
    return b
  if a.dist > b.dist:
    return a
  if b.dist > a.dist:
    return b
  if a.elem[0] != b.elem[0]:
    if a.elem[0] < b.elem[0]:
      return a
    return b
  if a.elem[1] != b.elem[1]:
    if a.elem[1] < b.elem[1]:
      return a
    return b
  if a.vert[0] != b.vert[0]:
    if a.vert[0] < b.vert[0]:
      return a
    return b
  if a.vert[1] != b.vert[1]:
    if a.vert[1] < b.vert[1]:
      return a
    return b
  if a.cidx < b.cidx:
    return a
  return b


@wp.func
def _fps_pick_max_canonical(a: _FpsPickCanonical, b: _FpsPickCanonical) -> _FpsPickCanonical:
  """Returns the pick with the larger distance, breaking ties like _tie_break_fps_canonical."""
  if b.cidx < 0:
    return a
  if a.cidx < 0:
    return b
  if a.dist > b.dist:
    return a
  if b.dist > a.dist:
    return b
  c = _cmp_vec2i(a.elem, b.elem)
  if c != 0:
    return a if c < 0 else b
  c = _cmp_vec2i(a.vert, b.vert)
  if c != 0:
    return a if c < 0 else b
  c = _cmp_vec3f(a.pos, b.pos)
  if c != 0:
    return a if c < 0 else b
  return a if a.cidx < b.cidx else b


@cache_kernel
def _parallel_fps(has_passive_flex: bool, canonical_ties: bool):
  @wp.kernel(module="unique", enable_backward=False)
  def kernel(
    # Model:
    opt_integrator: int,
    body_weldid: wp.array[int],
    geom_bodyid: wp.array[int],
    flex_passive: wp.array[int],
    flex_dim: wp.array[int],
    flex_interp: wp.array[int],
    flex_rigid: wp.array[bool],
    # In:
    flex_group_start_indices_in: wp.array[int],
    flex_num_groups_in: wp.array[int],
    ncand: wp.array[int],
    cand_active_sorted: wp.array[int],
    sort_val: wp.array[int],
    cand_dist: wp.array[float],
    cand_pos: wp.array[wp.vec3],
    cand_geom: wp.array[wp.vec2i],
    cand_flex: wp.array[wp.vec2i],
    cand_elem: wp.array[wp.vec2i],
    cand_vert: wp.array[wp.vec2i],
    # Out:
    fps_min_dist_out: wp.array[float],
    cand_active_out: wp.array[int],
  ):
    """Runs all FPS rounds for one candidate group per block.

    The first _FPS_SLOTS candidates per thread keep their position and min distance in
    registers; the rest fall back to global memory. Selected candidates are marked with
    a negative min distance.
    """
    g, tid = wp.tid()
    if g >= flex_num_groups_in[0] or ncand[0] <= MJ_MAXCONPAIR:
      return

    ncand_limit = wp.min(ncand[0], cand_active_sorted.shape[0])
    g_start = flex_group_start_indices_in[g]
    if g_start < 0 or g_start >= ncand_limit:
      return

    # skip FPS capping for 1D/2D flex-geom groups and passive flex groups
    first_cand = sort_val[g_start]
    if cand_geom[first_cand][0] >= 0 and flex_dim[cand_flex[first_cand][1]] < 3:
      return
    if wp.static(has_passive_flex):
      if _is_passive_flex_cand(
        opt_integrator,
        body_weldid,
        geom_bodyid,
        flex_passive,
        flex_dim,
        flex_interp,
        flex_rigid,
        cand_geom,
        cand_flex,
        first_cand,
      ):
        return

    g_end = ncand_limit
    if g < flex_num_groups_in[0] - 1:
      g_end = wp.min(ncand_limit, flex_group_start_indices_in[g + 1])

    nthread = wp.block_dim()

    # load candidates and find the seed (smallest contact distance)
    slot_cidx = _fps_veci(-1)
    slot_md = _fps_vecf(wp.inf)
    slot_x = _fps_vecf(0.0)
    slot_y = _fps_vecf(0.0)
    slot_z = _fps_vecf(0.0)
    local_active = int(0)
    min_d = float(1e10)
    seed = int(-1)
    for j in range(wp.static(_FPS_SLOTS)):
      si = g_start + tid + j * nthread
      if si < g_end:
        if cand_active_sorted[si] == 1:
          c_idx = sort_val[si]
          slot_cidx[j] = c_idx
          p = cand_pos[c_idx]
          slot_x[j] = p[0]
          slot_y[j] = p[1]
          slot_z[j] = p[2]
          local_active += 1
          d_val = cand_dist[c_idx]
          if d_val < min_d:
            min_d = d_val
            seed = c_idx
          elif d_val == min_d:
            if wp.static(canonical_ties):
              if _tie_break_fps_canonical(c_idx, seed, cand_pos, cand_elem, cand_vert):
                seed = c_idx
            elif _tie_break_fps(c_idx, seed, cand_elem, cand_vert):
              seed = c_idx

    overflow_start = g_start + tid + wp.static(_FPS_SLOTS) * nthread
    for si in range(overflow_start, g_end, nthread):
      if cand_active_sorted[si] == 1:
        c_idx = sort_val[si]
        fps_min_dist_out[c_idx] = wp.inf
        local_active += 1
        d_val = cand_dist[c_idx]
        if d_val < min_d:
          min_d = d_val
          seed = c_idx
        elif d_val == min_d:
          if wp.static(canonical_ties):
            if _tie_break_fps_canonical(c_idx, seed, cand_pos, cand_elem, cand_vert):
              seed = c_idx
          elif _tie_break_fps(c_idx, seed, cand_elem, cand_vert):
            seed = c_idx

    if wp.tile_sum(wp.tile(local_active))[0] <= MJ_MAXCONPAIR:
      return

    # negating the distance turns the min search into a max with the same tie-break
    if wp.static(canonical_ties):
      sel = wp.tile_reduce(
        _fps_pick_max_canonical,
        wp.tile(_fps_pick_canonical(-min_d, seed, cand_pos, cand_elem, cand_vert)),
      )[0].cidx
    else:
      sel = wp.tile_reduce(_fps_pick_max, wp.tile(_fps_pick(-min_d, seed, cand_elem, cand_vert)))[0].cidx

    for _ in range(MJ_MAXCONPAIR - 1):
      sel_p = cand_pos[sel]
      max_d = float(-1e10)
      best = int(-1)
      for j in range(wp.static(_FPS_SLOTS)):
        c_idx = slot_cidx[j]
        if c_idx >= 0:
          md = slot_md[j]
          if c_idx == sel:
            slot_md[j] = -1.0
          elif md > 0.0:
            md = wp.min(md, wp.length(wp.vec3(slot_x[j], slot_y[j], slot_z[j]) - sel_p))
            slot_md[j] = md
            if md > max_d:
              max_d = md
              best = c_idx
            elif md == max_d:
              if wp.static(canonical_ties):
                if _tie_break_fps_canonical(c_idx, best, cand_pos, cand_elem, cand_vert):
                  best = c_idx
              elif _tie_break_fps(c_idx, best, cand_elem, cand_vert):
                best = c_idx

      for si in range(overflow_start, g_end, nthread):
        if cand_active_sorted[si] == 1:
          c_idx = sort_val[si]
          if c_idx == sel:
            fps_min_dist_out[c_idx] = -1.0
          else:
            md = fps_min_dist_out[c_idx]
            if md > 0.0:
              d = wp.length(cand_pos[c_idx] - sel_p)
              if d < md:
                md = d
                fps_min_dist_out[c_idx] = md
              if md > max_d:
                max_d = md
                best = c_idx
              elif md == max_d:
                if wp.static(canonical_ties):
                  if _tie_break_fps_canonical(c_idx, best, cand_pos, cand_elem, cand_vert):
                    best = c_idx
                elif _tie_break_fps(c_idx, best, cand_elem, cand_vert):
                  best = c_idx

      if wp.static(canonical_ties):
        pick = wp.tile_reduce(
          _fps_pick_max_canonical,
          wp.tile(_fps_pick_canonical(max_d, best, cand_pos, cand_elem, cand_vert)),
        )[0]
      else:
        pick = wp.tile_reduce(_fps_pick_max, wp.tile(_fps_pick(max_d, best, cand_elem, cand_vert)))[0]
      if pick.cidx < 0 or pick.dist <= 0.0:
        sel = -1
        break
      sel = pick.cidx

    for j in range(wp.static(_FPS_SLOTS)):
      c_idx = slot_cidx[j]
      if c_idx >= 0:
        cand_active_out[c_idx] = wp.where(slot_md[j] < 0.0 or c_idx == sel, 1, 0)

    for si in range(overflow_start, g_end, nthread):
      if cand_active_sorted[si] == 1:
        c_idx = sort_val[si]
        cand_active_out[c_idx] = wp.where(fps_min_dist_out[c_idx] < 0.0 or c_idx == sel, 1, 0)

  return kernel


def flex_broadphase_aabb(m: Model, d: Data):
  """Precompute dynamic flex object bounding boxes."""
  wp.launch(
    _flex_broadphase_bounds,
    dim=(d.nworld, m.nflex),
    inputs=[
      m.flex_margin,
      m.flex_gap,
      m.flex_vertadr,
      m.flex_vertnum,
      m.flex_radius,
      m.flex_rod_is_box,
      m.flex_rod_box_size,
      d.flexvert_xpos,
    ],
    outputs=[
      d.flex_aabb_min,
      d.flex_aabb_max,
    ],
  )


@dataclasses.dataclass
class FlexWorkspace:
  """GPU workspace buffers for flex collision detection, EPA CCD, and contact filtering."""

  # Candidate buffers
  dist: wp.array
  pos: wp.array
  nrm: wp.array
  geom: wp.array
  flex: wp.array
  elem: wp.array
  vert: wp.array
  worldid: wp.array
  ncand: wp.array

  # Filtering & FPS buffers
  filter_key: wp.array
  filter_val: wp.array
  cand_active: wp.array

  # EPA Narrowphase buffers
  epa_vert: wp.array
  epa_vert_index: wp.array
  epa_face: wp.array
  epa_pr: wp.array
  epa_norm2: wp.array
  epa_horizon: wp.array

  # Optional FPS & CCD buffers
  cand_active_sorted: wp.array | None = None
  flex_group_temp: wp.array | None = None
  flex_group_ids: wp.array | None = None
  flex_group_start_indices: wp.array | None = None
  flex_fps_min_dist: wp.array | None = None
  flex_num_groups: wp.array | None = None
  nccd: wp.array | None = None


def _allocate_flex_workspace(m: Model, d: Data) -> FlexWorkspace:
  epa_iterations = m.opt.ccd_iterations
  # cylinder : triangle uses the general convex collider (MuJoCo C mjc_ConvexElem)
  cylinder_elem = m.has_cylinder_geom and m.has_2d_flex
  has_epa = (
    m.nmesh > 0
    or m.has_ellipsoid_geom
    or cylinder_elem
    or m.has_flex_selfcollide
    or m.nflex > 1
    or m.has_1d_flex
    or m.has_3d_flex
  )
  capacity = d.naccdmax if has_epa else 1

  needs_nccd = m.nmesh > 0 or m.has_ellipsoid_geom or cylinder_elem or m.has_1d_flex or m.has_3d_flex
  nccd = wp.zeros(1, dtype=int) if needs_nccd else None

  has_geom_fps = m.has_3d_flex and m.ngeom > 0
  has_sap_fps = m.has_flex_selfcollide or m.nflex > 1
  has_fps = has_geom_fps or has_sap_fps
  if has_fps:
    nmax_geom_groups = d.nworld * m.ngeom * m.nflex if has_geom_fps else 0
    nmax_sap_groups = d.nworld * m.nflex * m.nflex if has_sap_fps else 0
    nmax_groups = min(d.naconmax, max(nmax_geom_groups, nmax_sap_groups))
    cand_active_sorted = wp.empty(d.naconmax, dtype=int)
    flex_group_temp = wp.empty(d.naconmax, dtype=int)
    flex_group_ids = wp.empty(d.naconmax, dtype=int)
    flex_group_start_indices = wp.full(nmax_groups, -1, dtype=int)
    flex_fps_min_dist = wp.empty(d.naconmax, dtype=float)
    flex_num_groups = wp.zeros(1, dtype=int)
  else:
    cand_active_sorted = None
    flex_group_temp = None
    flex_group_ids = None
    flex_group_start_indices = None
    flex_fps_min_dist = None
    flex_num_groups = None

  return FlexWorkspace(
    dist=wp.empty(d.naconmax, dtype=float),
    pos=wp.empty(d.naconmax, dtype=wp.vec3),
    nrm=wp.empty(d.naconmax, dtype=wp.vec3),
    geom=wp.empty(d.naconmax, dtype=wp.vec2i),
    flex=wp.empty(d.naconmax, dtype=wp.vec2i),
    elem=wp.empty(d.naconmax, dtype=wp.vec2i),
    vert=wp.empty(d.naconmax, dtype=wp.vec2i),
    worldid=wp.empty(d.naconmax, dtype=int),
    ncand=wp.zeros(1, dtype=int),
    filter_key=wp.empty(d.naconmax * 2, dtype=wp.int64),
    filter_val=wp.empty(d.naconmax * 2, dtype=int),
    cand_active=wp.empty(d.naconmax, dtype=int),
    cand_active_sorted=cand_active_sorted,
    flex_group_temp=flex_group_temp,
    flex_group_ids=flex_group_ids,
    flex_group_start_indices=flex_group_start_indices,
    flex_fps_min_dist=flex_fps_min_dist,
    flex_num_groups=flex_num_groups,
    epa_vert=wp.empty(shape=(capacity, 10 + 2 * epa_iterations), dtype=wp.vec3),
    epa_vert_index=wp.empty(shape=(capacity, 10 + 2 * epa_iterations), dtype=int),
    epa_face=wp.empty(shape=(capacity, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=int),
    epa_pr=wp.empty(shape=(capacity, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=wp.vec3),
    epa_norm2=wp.empty(shape=(capacity, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=float),
    epa_horizon=wp.empty(shape=(capacity, MJ_MAX_EPAHORIZON), dtype=int),
    nccd=nccd,
  )


def _run_filter_flex_fps(m: Model, ws: FlexWorkspace, has_passive_flex: bool, canonical_ties: bool):
  """Applies Far Point Sampling to limit contact points per group to MJ_MAXCONPAIR in parallel."""
  wp.launch_tiled(
    _parallel_fps(has_passive_flex, canonical_ties),
    dim=ws.flex_group_start_indices.shape[0],
    inputs=[
      m.opt.integrator,
      m.body_weldid,
      m.geom_bodyid,
      m.flex_passive,
      m.flex_dim,
      m.flex_interp,
      m.flex_rigid,
      ws.flex_group_start_indices,
      ws.flex_num_groups,
      ws.ncand,
      ws.cand_active_sorted,
      ws.filter_val,
      ws.dist,
      ws.pos,
      ws.geom,
      ws.flex,
      ws.elem,
      ws.vert,
    ],
    outputs=[
      ws.flex_fps_min_dist,
      ws.cand_active,
    ],
    block_dim=m.block_dim.flex_fps,
  )


def _filter_and_write_contacts(
  m: Model,
  d: Data,
  ws: FlexWorkspace,
  enable_fps: bool = False,
):
  """Deduplicates candidates, optionally applies FPS filtering, and writes contacts to d.contact."""
  wp.launch(
    _compute_filter_key,
    dim=d.naconmax,
    inputs=[
      m.ngeom,
      m.nflex,
      ws.ncand,
      ws.geom,
      ws.flex,
      ws.pos,
      ws.worldid,
    ],
    outputs=[
      ws.filter_key,
      ws.filter_val,
    ],
  )

  wp.utils.radix_sort_pairs(ws.filter_key, ws.filter_val, d.naconmax)

  has_passive_flex = bool(m.opt.integrator == IntegratorType.DISCRETE and m.has_flex_passive)
  canonical_ties = bool(m.opt.deterministic & DeterminismType.CONTACTS)
  wp.launch(
    _filter_flex_candidates_sorted(has_passive_flex, canonical_ties),
    dim=d.naconmax,
    inputs=[
      m.opt.integrator,
      m.body_weldid,
      m.geom_bodyid,
      m.flex_passive,
      m.flex_dim,
      m.flex_interp,
      m.flex_rigid,
      ws.ncand,
      _FLEX_CONTACT_DEDUP_TOLERANCE,
      ws.filter_key,
      ws.filter_val,
      ws.dist,
      ws.pos,
      ws.nrm,
      ws.geom,
      ws.flex,
      ws.elem,
      ws.vert,
    ],
    outputs=[ws.cand_active],
  )

  if enable_fps:
    wp.launch(
      _populate_active_sorted,
      dim=d.naconmax,
      inputs=[ws.ncand, ws.filter_val, ws.cand_active],
      outputs=[ws.cand_active_sorted],
    )

    wp.launch(
      _find_group_starts,
      dim=d.naconmax,
      inputs=[
        ws.ncand,
        ws.filter_key,
      ],
      outputs=[ws.flex_group_temp],
    )

    wp.utils.array_scan(ws.flex_group_temp, ws.flex_group_ids, True)

    wp.launch(
      _populate_group_starts(int(m.opt.warn_overflow)),
      dim=d.naconmax,
      inputs=[
        ws.flex_group_temp,
        ws.flex_group_ids,
        ws.ncand,
        ws.filter_val,
        ws.worldid,
      ],
      outputs=[
        d.overflow,
        ws.flex_group_start_indices,
        ws.flex_num_groups,
      ],
    )

    _run_filter_flex_fps(m, ws, has_passive_flex, canonical_ties)

  wp.launch(
    _write_filtered_contacts(int(m.opt.warn_overflow), has_passive_flex),
    dim=d.naconmax,
    inputs=[
      m.opt.integrator,
      m.body_weldid,
      m.geom_condim,
      m.geom_bodyid,
      m.geom_priority,
      m.geom_solmix,
      m.geom_solref,
      m.geom_solimp,
      m.geom_friction,
      m.geom_margin,
      m.geom_gap,
      m.flex_condim,
      m.flex_priority,
      m.flex_solmix,
      m.flex_solref,
      m.flex_solimp,
      m.flex_friction,
      m.flex_margin,
      m.flex_gap,
      m.flex_passive,
      m.flex_dim,
      m.flex_interp,
      m.flex_rigid,
      d.naconmax,
      ws.ncand,
      ws.dist,
      ws.pos,
      ws.nrm,
      ws.geom,
      ws.flex,
      ws.elem,
      ws.vert,
      ws.worldid,
      ws.cand_active,
    ],
    outputs=[
      d.contact.dist,
      d.contact.pos,
      d.contact.frame,
      d.contact.includemargin,
      d.contact.friction,
      d.contact.solref,
      d.contact.solreffriction,
      d.contact.solimp,
      d.contact.dim,
      d.contact.geom,
      d.contact.flex,
      d.contact.elem,
      d.contact.vert,
      d.contact.efc_address,
      d.contact.worldid,
      d.contact.type,
      d.contact.geomcollisionid,
      d.nacon,
      d.overflow,
    ],
  )


def _detect_plane_flex_candidates(
  m: Model,
  d: Data,
  ws: FlexWorkspace,
):
  """Detect candidates between plane geoms and flex vertices."""
  if m.nflexvert == 0 or not m.has_plane_geom:
    return

  wp.launch(
    _flex_plane_narrowphase(int(m.opt.warn_overflow)),
    dim=(d.nworld, m.nflexvert),
    inputs=[
      m.ngeom,
      m.qpos0,
      m.jnt_qposadr,
      m.geom_type,
      m.geom_contype,
      m.geom_conaffinity,
      m.geom_margin,
      m.geom_gap,
      m.flex_contype,
      m.flex_conaffinity,
      m.flex_margin,
      m.flex_gap,
      m.flex_vertadr,
      m.flex_elemadr,
      m.flex_elemnum,
      m.flex_elemdataadr,
      m.flex_plasticadr,
      m.flex_vertbodyid,
      m.flex_elem,
      m.flex_radius,
      m.flex_vertflexid,
      m.flex_rod_is_box,
      m.flex_rod_root_clamped,
      m.flex_rod_tail_clamped,
      m.flex_rod_box_size,
      m.flex_rod_edgeadr,
      m.flex_rod_edgenum,
      m.flex_rodedge_vert,
      m.flex_rodedge_jnt,
      m.flex_elem_rodedge,
      m.flex_vert_rodelem,
      d.qpos,
      d.plastic,
      d.xquat,
      d.geom_xpos,
      d.geom_xquat,
      d.flexvert_xpos,
      d.flex_aabb_min,
      d.flex_aabb_max,
      d.flex_rod_frame0_loc,
      d.flex_rod_frame0_ref,
      d.flex_rod_tail_loc,
      d.naconmax,
    ],
    outputs=[
      d.overflow,
      ws.dist,
      ws.pos,
      ws.nrm,
      ws.geom,
      ws.flex,
      ws.elem,
      ws.vert,
      ws.worldid,
      ws.ncand,
    ],
  )


def _detect_elem_geom_candidates(
  m: Model,
  d: Data,
  ws: FlexWorkspace,
):
  """Detect candidates between 1D/2D/3D flex elements and geoms."""
  epa_iterations = m.opt.ccd_iterations
  wp.launch(
    _flex_narrowphase_elem_detect(int(m.opt.warn_overflow), bool(m.has_cylinder_geom), bool(m.has_ellipsoid_geom)),
    dim=(d.nworld, m.nflexelem),
    inputs=[
      m.ngeom,
      m.opt.ccd_tolerance,
      m.qpos0,
      m.jnt_qposadr,
      m.geom_type,
      m.geom_contype,
      m.geom_conaffinity,
      m.geom_bodyid,
      m.geom_dataid,
      m.geom_size,
      m.geom_aabb,
      m.geom_rbound,
      m.geom_margin,
      m.geom_gap,
      m.flex_contype,
      m.flex_conaffinity,
      m.flex_margin,
      m.flex_gap,
      m.flex_activelayers,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_elemadr,
      m.flex_elemnum,
      m.flex_elemdataadr,
      m.flex_plasticadr,
      m.flex_vertbodyid,
      m.flex_elem,
      m.flex_elemlayer,
      m.flex_radius,
      m.mesh_vertadr,
      m.mesh_vertnum,
      m.mesh_graphadr,
      m.mesh_vert,
      m.mesh_graph,
      m.mesh_polyadr,
      m.mesh_polynormal,
      m.mesh_polyvertadr,
      m.mesh_polyvertnum,
      m.mesh_polyvert,
      m.mesh_polymapadr,
      m.mesh_polymapnum,
      m.mesh_polymap,
      m.flex_elemflexid,
      m.flex_rod_is_box,
      m.flex_rod_root_clamped,
      m.flex_rod_tail_clamped,
      m.flex_rod_box_size,
      m.flex_rod_edgeadr,
      m.flex_rod_edgenum,
      m.flex_rodedge_vert,
      m.flex_rodedge_jnt,
      m.flex_elem_rodedge,
      d.qpos,
      d.plastic,
      d.xquat,
      d.geom_xpos,
      d.geom_xquat,
      d.flexvert_xpos,
      d.naccdmax,
      d.flex_aabb_min,
      d.flex_aabb_max,
      d.flex_rod_frame0_loc,
      d.flex_rod_frame0_ref,
      d.flex_rod_tail_loc,
      ws.epa_vert,
      ws.epa_vert_index,
      ws.epa_face,
      ws.epa_pr,
      ws.epa_norm2,
      ws.epa_horizon,
      ws.nccd,
      epa_iterations,
      d.naconmax,
    ],
    outputs=[
      d.overflow,
      ws.dist,
      ws.pos,
      ws.nrm,
      ws.geom,
      ws.flex,
      ws.elem,
      ws.vert,
      ws.worldid,
      ws.ncand,
    ],
  )


def _flex_bvh_update(m: Model, d: Data) -> tuple[wp.array, wp.array]:
  """Refits the flex BVHs to the current element AABBs, leaves first then by decreasing depth."""
  bvh_lower = wp.empty((d.nworld, m.nbvhdynamic), dtype=wp.vec3)
  bvh_upper = wp.empty((d.nworld, m.nbvhdynamic), dtype=wp.vec3)

  wp.launch(
    _flex_bvh_leaf,
    dim=(d.nworld, m.nbvhdynamic),
    inputs=[
      m.nbvhstatic,
      m.bvh_nodeid,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_elemdataadr,
      m.flex_elem,
      m.flex_radius,
      m.flex_bvhflexid,
      m.flex_rod_is_box,
      m.flex_rod_box_size,
      d.flexvert_xpos,
    ],
    outputs=[bvh_lower, bvh_upper],
  )

  for level in reversed(m.flex_bvh_tree):
    wp.launch(
      _flex_bvh_refit,
      dim=(d.nworld, level.size),
      inputs=[m.nbvhstatic, m.bvh_child, m.flex_bvhadr, m.flex_bvhflexid, level],
      outputs=[bvh_lower, bvh_upper],
    )

  return bvh_lower, bvh_upper


def _run_flex_narrowphase(
  m: Model,
  d: Data,
  ctx,
  ws: FlexWorkspace,
):
  """Executes narrowphase collision detection for element pairs."""
  epa_iterations = m.opt.ccd_iterations
  workspace_verts = wp.empty(d.naconmax * 8, dtype=wp.vec3)
  wp.launch(
    _flex_narrowphase(int(m.opt.warn_overflow)),
    dim=d.naconmax,
    inputs=[
      m.opt.ccd_tolerance,
      m.qpos0,
      m.jnt_qposadr,
      m.flex_margin,
      m.flex_gap,
      m.flex_activelayers,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_elemadr,
      m.flex_elemnum,
      m.flex_elemdataadr,
      m.flex_plasticadr,
      m.flex_vertbodyid,
      m.flex_elem,
      m.flex_elemlayer,
      m.flex_radius,
      m.flex_elemflexid,
      m.flex_rod_is_box,
      m.flex_rod_root_clamped,
      m.flex_rod_tail_clamped,
      m.flex_rod_box_size,
      m.flex_rod_edgeadr,
      m.flex_rod_edgenum,
      m.flex_rodedge_vert,
      m.flex_rodedge_jnt,
      m.flex_elem_rodedge,
      d.qpos,
      d.plastic,
      d.xquat,
      d.flexvert_xpos,
      d.naccdmax,
      d.ncollision,
      d.flex_rod_frame0_loc,
      d.flex_rod_frame0_ref,
      d.flex_rod_tail_loc,
      d.naconmax,
      m.opt.ccd_iterations,
      epa_iterations,
      ctx.collision_pair,
      ctx.collision_worldid,
    ],
    outputs=[
      d.overflow,
      workspace_verts,
      ws.epa_vert,
      ws.epa_vert_index,
      ws.epa_face,
      ws.epa_pr,
      ws.epa_norm2,
      ws.epa_horizon,
      ws.dist,
      ws.pos,
      ws.nrm,
      ws.geom,
      ws.flex,
      ws.elem,
      ws.vert,
      ws.worldid,
      ws.ncand,
    ],
  )


def _flex_geom_collision(
  m: Model,
  d: Data,
  ws: FlexWorkspace,
):
  """Detect and write collisions between rigid geoms and flex objects."""
  if m.nflex == 0:
    return

  ws.ncand.zero_()
  if ws.flex_num_groups is not None:
    ws.flex_num_groups.zero_()

  # Plane collisions (flex vertices vs infinite planes)
  _detect_plane_flex_candidates(m, d, ws)

  # 1D cable, 2D cloth, and 3D softbody element collisions (elements vs rigid geoms)
  _detect_elem_geom_candidates(m, d, ws)

  # Contact writing pass
  _filter_and_write_contacts(m, d, ws, enable_fps=m.has_3d_flex and m.ngeom > 0)


def _flex_bvh_collision(
  m: Model,
  d: Data,
  ctx,
  ws: FlexWorkspace,
  is_self: bool,
  bvh_bounds: tuple[wp.array, wp.array],
  enable_sat: bool,
):
  """Detect and write flex self or flex-flex collision contacts (broadphase and narrowphase)."""
  if is_self and not m.has_flex_selfcollide:
    return
  if not is_self and m.nflex <= 1:
    return

  ws.ncand.zero_()
  if ws.flex_num_groups is not None:
    ws.flex_num_groups.zero_()

  bvh_lower, bvh_upper = bvh_bounds
  d.ncollision.zero_()

  # with few worlds, split each query over subtrees to fill the GPU
  split_depth = 0
  while split_depth < _FLEX_BVH_MAX_SPLIT_DEPTH and (d.nworld * m.nbvhdynamic << (split_depth + 1)) <= _FLEX_BVH_QUERY_THREADS:
    split_depth += 1

  wp.launch(
    _flex_bvh_query(is_self, int(m.opt.warn_overflow), enable_sat, split_depth),
    dim=(d.nworld, m.nbvhdynamic, 1 << split_depth),
    inputs=[
      m.nbvhstatic,
      m.nflex,
      m.bvh_child,
      m.bvh_nodeid,
      m.flex_contype,
      m.flex_conaffinity,
      m.flex_margin,
      m.flex_gap,
      m.flex_selfcollide,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_elemadr,
      m.flex_elemdataadr,
      m.flex_vertbodyid,
      m.flex_elem,
      m.flex_radius,
      m.flex_bvhadr,
      m.flex_bvhflexid,
      d.flexvert_xpos,
      d.flex_aabb_min,
      d.flex_aabb_max,
      bvh_lower,
      bvh_upper,
      d.naconmax,
    ],
    outputs=[
      d.ncollision,
      d.overflow,
      ctx.collision_pair,
      ctx.collision_worldid,
    ],
  )

  _run_flex_narrowphase(
    m,
    d,
    ctx,
    ws,
  )

  _filter_and_write_contacts(m, d, ws, enable_fps=True)


@event_scope
def flex_collision(m: Model, d: Data, ctx, enable_sat: bool | None = None):
  """Runs collision detection for all flex collisions."""
  if m.nflex == 0 or m.nflexelem == 0:
    return

  if enable_sat is None:
    enable_sat = ENABLE_SAT_PREFILTER

  # Update dynamic flex object bounding boxes
  flex_broadphase_aabb(m, d)

  ws = _allocate_flex_workspace(m, d)

  # 1. Flex-Geom Collision (Primitive and CCD)
  _flex_geom_collision(m, d, ws)

  # Refit the flex BVHs once for self and flex-flex collision
  if m.nbvhdynamic == 0 or not (m.has_flex_selfcollide or m.nflex > 1):
    return
  bvh_bounds = _flex_bvh_update(m, d)

  # 2. Flex Self-Collision (Broadphase and Narrowphase)
  _flex_bvh_collision(m, d, ctx, ws, is_self=True, bvh_bounds=bvh_bounds, enable_sat=enable_sat)

  # 3. Flex-Flex Collision (Broadphase and Narrowphase)
  _flex_bvh_collision(m, d, ctx, ws, is_self=False, bvh_bounds=bvh_bounds, enable_sat=enable_sat)
